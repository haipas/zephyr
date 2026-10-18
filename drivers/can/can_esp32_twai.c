/*
 * Copyright (c) 2022 Henrik Brix Andersen <henrik@brixandersen.dk>
 * Copyright (c) 2022 Martin Jäger <martin@libre.solar>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT espressif_esp32_twai

#include "can_sja1000.h"

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include <soc.h>
#if defined(CONFIG_SOC_SERIES_ESP32P4)
#include <esp_clk_tree.h>
#endif

LOG_MODULE_REGISTER(can_esp32_twai, CONFIG_CAN_LOG_LEVEL);

/*
 * Newer ESP32-series MCUs like ESP32-C3 and ESP32-S2 have some slightly different registers
 * compared to the original ESP32, which is fully compatible with the SJA1000 controller.
 *
 * The names with TWAI_ prefixes from Espressif reference manuals are used for these incompatible
 * registers.
 */
#ifndef CONFIG_SOC_SERIES_ESP32

/* TWAI_BUS_TIMING_0_REG is incompatible with CAN_SJA1000_BTR0 */
#define TWAI_BUS_TIMING_0_REG           (6U)
#define TWAI_BAUD_PRESC_MASK            GENMASK(12, 0)
#define TWAI_SYNC_JUMP_WIDTH_MASK       GENMASK(15, 14)
#define TWAI_BAUD_PRESC_PREP(brp)	FIELD_PREP(TWAI_BAUD_PRESC_MASK, brp)
#define TWAI_SYNC_JUMP_WIDTH_PREP(sjw)	FIELD_PREP(TWAI_SYNC_JUMP_WIDTH_MASK, sjw)

/*
 * TWAI_BUS_TIMING_1_REG is compatible with CAN_SJA1000_BTR1, but needed here for the custom
 * set_timing() function.
 */
#define TWAI_BUS_TIMING_1_REG           (7U)
#define TWAI_TIME_SEG1_MASK             GENMASK(3, 0)
#define TWAI_TIME_SEG2_MASK             GENMASK(6, 4)
#define TWAI_TIME_SAMP                  BIT(7)
#define TWAI_TIME_SEG1_PREP(seg1)       FIELD_PREP(TWAI_TIME_SEG1_MASK, seg1)
#define TWAI_TIME_SEG2_PREP(seg2)       FIELD_PREP(TWAI_TIME_SEG2_MASK, seg2)

/* TWAI_CLOCK_DIVIDER_REG is incompatible with CAN_SJA1000_CDR */
#define TWAI_CLOCK_DIVIDER_REG          (31U)
#define TWAI_CD_MASK			GENMASK(7, 0)
#define TWAI_CLOCK_OFF			BIT(8)

/*
 * Further incompatible registers currently not used by the driver:
 * - TWAI_STATUS_REG has new bit 8: TWAI_MISS_ST
 * - TWAI_INT_RAW_REG has new bit 8: TWAI_BUS_STATE_INT_ST
 * - TWAI_INT_ENA_REG has new bit 8: TWAI_BUS_STATE_INT_ENA
 */
#else

/* Redefinitions of the SJA1000 CDR bits to simplify driver config */
#define TWAI_CD_MASK			GENMASK(2, 0)
#define TWAI_CLOCK_OFF			BIT(3)

#endif /* !CONFIG_SOC_SERIES_ESP32 */

struct can_esp32_twai_config {
	mm_reg_t base;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	const clock_control_subsys_t clock_subsys;
	int irq_source;
	int irq_priority;
	int irq_flags;
#if defined(CONFIG_CAN_ESP32_TWAI_RX_IRAM)
	k_thread_stack_t *rx_stack;
	size_t rx_stack_size;
#endif
#ifndef CONFIG_SOC_SERIES_ESP32
	/* 32-bit variant of output clock divider register required for non-ESP32 MCUs */
	uint32_t cdr32;
#endif /* !CONFIG_SOC_SERIES_ESP32 */
};

static uint8_t can_esp32_twai_read_reg(const struct device *dev, uint8_t reg)
{
	const struct can_sja1000_config *sja1000_config = dev->config;
	const struct can_esp32_twai_config *twai_config = sja1000_config->custom;
	mm_reg_t addr = twai_config->base + reg * sizeof(uint32_t);

	return sys_read32(addr) & 0xFF;
}

static void can_esp32_twai_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct can_sja1000_config *sja1000_config = dev->config;
	const struct can_esp32_twai_config *twai_config = sja1000_config->custom;
	mm_reg_t addr = twai_config->base + reg * sizeof(uint32_t);

	sys_write32(val & 0xFF, addr);
}

#ifndef CONFIG_SOC_SERIES_ESP32

/*
 * Required for newer ESP32-series MCUs which violate the original SJA1000 8-bit register size.
 */
static void can_esp32_twai_write_reg32(const struct device *dev, uint8_t reg, uint32_t val)
{
	const struct can_sja1000_config *sja1000_config = dev->config;
	const struct can_esp32_twai_config *twai_config = sja1000_config->custom;
	mm_reg_t addr = twai_config->base + reg * sizeof(uint32_t);

	sys_write32(val, addr);
}

/*
 * Custom implementation instead of can_sja1000_set_timing required because TWAI_BUS_TIMING_0_REG
 * is incompatible with CAN_SJA1000_BTR0.
 */
static int can_esp32_twai_set_timing(const struct device *dev, const struct can_timing *timing)
{
	struct can_sja1000_data *data = dev->data;
	uint8_t btr0;
	uint8_t btr1;

	if (data->common.started) {
		return -EBUSY;
	}

	k_mutex_lock(&data->mod_lock, K_FOREVER);

	btr0 = TWAI_BAUD_PRESC_PREP(timing->prescaler - 1) |
	       TWAI_SYNC_JUMP_WIDTH_PREP(timing->sjw - 1);
	btr1 = TWAI_TIME_SEG1_PREP(timing->phase_seg1 - 1) |
	       TWAI_TIME_SEG2_PREP(timing->phase_seg2 - 1);

	if ((data->common.mode & CAN_MODE_3_SAMPLES) != 0) {
		btr1 |= TWAI_TIME_SAMP;
	}

	can_esp32_twai_write_reg32(dev, TWAI_BUS_TIMING_0_REG, btr0);
	can_esp32_twai_write_reg32(dev, TWAI_BUS_TIMING_1_REG, btr1);

	k_mutex_unlock(&data->mod_lock);

	return 0;
}

#endif /* !CONFIG_SOC_SERIES_ESP32 */

static int can_esp32_twai_get_core_clock(const struct device *dev, uint32_t *rate)
{
	ARG_UNUSED(dev);

#if defined(CONFIG_SOC_SERIES_ESP32P4)
	/*
	 * The ESP32-P4 TWAI controllers are not clocked from the APB clock: the only
	 * TWAI clock source on this SoC is the 40 MHz XTAL (SOC_TWAI_CLKS in
	 * soc/clk_tree_defs.h, TWAI_CLK_SRC_DEFAULT == SOC_MOD_CLK_XTAL), and
	 * soc.h's APB_CLK_FREQ (90 MHz) is unrelated to it. Using APB_CLK_FREQ / 2
	 * (45 MHz) here made the requested bit rate run at 20/45 = 0.444x: 1 Mbit/s
	 * became 444 kbit/s and the controller never received a frame on a real
	 * 1 Mbit/s bus (error_passive, REC stuck at 128, rx_frames 0).
	 *
	 * Ask the clock tree for the frequency of the default TWAI source instead
	 * (same query the TWAI-FD driver uses); like on the other SoCs the SJA1000
	 * core runs at half of it, i.e. 20 MHz on the P4. Bench-verified on a
	 * Waveshare ESP32-P4 module by JTAG: BTR0=0x00/BTR1=0x3E (20 tq @ 20 MHz,
	 * 1 Mbit/s) receives; the old BTR0=0x02/BTR1=0x39 (Zephyr-computed against
	 * 45 MHz) does not.
	 */
	uint32_t source_hz = 0;
	int err;

	err = esp_clk_tree_src_get_freq_hz(TWAI_CLK_SRC_DEFAULT,
					   ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &source_hz);
	if (err != 0 || source_hz == 0) {
		LOG_ERR("failed to query TWAI clock frequency (err %d)", err);
		return err != 0 ? -EIO : -EINVAL;
	}

	*rate = source_hz / 2;
#else
	/* The internal clock operates at half of the oscillator frequency */
	*rate = APB_CLK_FREQ / 2;
#endif

	return 0;
}

#if !defined(CONFIG_CAN_ESP32_TWAI_RX_IRAM)
static void IRAM_ATTR can_esp32_twai_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;

	can_sja1000_isr(dev);
}
#endif

#if defined(CONFIG_CAN_ESP32_TWAI_RX_IRAM)
/*
 * 3d local (3d-fw#1277): a receive path that keeps running while flash is written.
 *
 * A flash write or erase locks every interrupt of the core that runs it and parks the
 * other core in the level-1 stall IPI with its non-IRAM interrupts masked
 * (soc/espressif/common/esp_mp_stall.c), for one sector erase ~20 ms. can_sja1000_isr()
 * is flash code, so the TWAI interrupt stayed blind that long, the 64-byte RX FIFO
 * (about five classic frames) overran, and the frames behind the overrun were read
 * anyway: TWAI_MISS_ST marks them, the SJA1000 code does not know the bit.
 *
 * Here the interrupt is allocated ESP_INTR_FLAG_IRAM at level
 * CONFIG_CAN_ESP32_TWAI_RX_IRAM_LEVEL (above the stall IPI) on
 * CONFIG_CAN_ESP32_TWAI_RX_IRAM_CPU, from a thread pinned there -- the core that runs no
 * flash operation. Its handler (IRAM, internal RAM only) reads IR, copies every receive
 * buffer window into a ring in internal RAM, releases the frames TWAI_MISS_ST marks
 * without reading them (counted) and clears a data overrun. The rest -- decoding, the RX
 * filter callbacks, transmit done, error and state handling -- runs right away from the
 * same handler when this core is not parked (then the cache is on: a flash operation of
 * the other core parks this core first), else from the thread within
 * CONFIG_CAN_ESP32_TWAI_RX_POLL_MS, with the interrupt disabled on this core.
 */
#include "can_sja1000_priv.h"

#include <esp_attr.h>
#include <esp_cpu.h>
#include <esp_intr_alloc.h>
#include <esp_mp_stall.h>

#define TWAI_RX_RING CONFIG_CAN_ESP32_TWAI_RX_RING_FRAMES
BUILD_ASSERT(IS_POWER_OF_TWO(TWAI_RX_RING), "CAN_ESP32_TWAI_RX_RING_FRAMES: a power of two");
BUILD_ASSERT(!IS_ENABLED(CONFIG_TICKET_SPINLOCKS) && !IS_ENABLED(CONFIG_SPIN_VALIDATE),
	     "the IRAM handler takes buf_lock the way k_spin_lock() does without these");
#define TWAI_SR_MISS BIT(8) /* TWAI_MISS_ST: the frame in the RX buffer is an overrun frame */
#define TWAI_RX_MAX_PER_IRQ 64U

struct can_esp32_twai_rx_entry {
	uint8_t buf[CAN_SJA1000_RX_BUFFER_LEN];
};

/* Everything the IRAM handler touches: plain .bss, internal RAM. */
struct can_esp32_twai_rx {
	mm_reg_t base;
	struct can_sja1000_data *sja;
	const struct device *dev;
	uint32_t head;       /* written by the handler only */
	uint32_t tail;       /* written by the dispatcher only */
	uint32_t pending_ir; /* IR bits besides RI, for the dispatcher */
	uint32_t missed;     /* overrun frames released unread (TWAI_MISS_ST) */
	uint32_t ring_full;  /* frames released unread: ring full */
	uint32_t high_water;
	intr_handle_t handle;
	int alloc_result;
	struct k_thread thread;
	struct can_esp32_twai_rx_entry ring[TWAI_RX_RING];
};

static ALWAYS_INLINE uint32_t twai_rx_reg(const struct can_esp32_twai_rx *rx, uint8_t reg)
{
	return sys_read32(rx->base + reg * sizeof(uint32_t));
}

static ALWAYS_INLINE void twai_rx_reg_write(const struct can_esp32_twai_rx *rx, uint8_t reg,
					    uint32_t val)
{
	sys_write32(val & 0xFF, rx->base + reg * sizeof(uint32_t));
}

/*
 * k_spin_lock()/k_spin_unlock() on buf_lock, spelled out with builtins so that nothing
 * outside IRAM is called (the transmit path writes the same register window).
 */
static ALWAYS_INLINE unsigned int twai_rx_buf_lock(struct k_spinlock *l)
{
	unsigned int key = arch_irq_lock();

#if defined(CONFIG_SMP)
	for (;;) {
		atomic_val_t expected = 0;

		if (__atomic_compare_exchange_n(&l->locked, &expected, 1, false, __ATOMIC_SEQ_CST,
						__ATOMIC_SEQ_CST)) {
			break;
		}
		arch_nop();
	}
#else
	ARG_UNUSED(l);
#endif
	return key;
}

static ALWAYS_INLINE void twai_rx_buf_unlock(struct k_spinlock *l, unsigned int key)
{
#if defined(CONFIG_SMP)
	__atomic_store_n(&l->locked, 0, __ATOMIC_SEQ_CST);
#else
	ARG_UNUSED(l);
#endif
	arch_irq_unlock(key);
}

/* Flash code: only with the cache on (see the comment above). */
static void can_esp32_twai_rx_dispatch(struct can_esp32_twai_rx *rx)
{
	uint32_t tail = rx->tail;
	uint32_t head = __atomic_load_n(&rx->head, __ATOMIC_ACQUIRE);
	uint32_t ir;

	while (tail != head) {
		struct can_frame frame;

		can_sja1000_decode_rx_buffer(rx->ring[tail % TWAI_RX_RING].buf, &frame);
		tail++;
		__atomic_store_n(&rx->tail, tail, __ATOMIC_RELEASE);
		can_sja1000_rx_dispatch(rx->dev, &frame);
		head = __atomic_load_n(&rx->head, __ATOMIC_ACQUIRE);
	}
	ir = __atomic_exchange_n(&rx->pending_ir, 0U, __ATOMIC_ACQ_REL);
	if (ir != 0U) {
		can_sja1000_handle_irq_status(rx->dev, (uint8_t)ir);
	}
}

static void IRAM_ATTR can_esp32_twai_rx_isr(void *arg)
{
	struct can_esp32_twai_rx *rx = arg;
	const uint32_t ir = twai_rx_reg(rx, CAN_SJA1000_IR);
	uint32_t head = rx->head;
	uint32_t fill;
	unsigned int key;

	key = twai_rx_buf_lock(&rx->sja->buf_lock);
	for (uint32_t n = 0U; n < TWAI_RX_MAX_PER_IRQ; n++) {
		const uint32_t sr = twai_rx_reg(rx, CAN_SJA1000_SR);

		if ((sr & CAN_SJA1000_SR_RBS) == 0U) {
			break;
		}
		if ((sr & TWAI_SR_MISS) != 0U) {
			rx->missed++;
		} else if (head - __atomic_load_n(&rx->tail, __ATOMIC_ACQUIRE) >= TWAI_RX_RING) {
			rx->ring_full++;
		} else {
			uint8_t *buf = rx->ring[head % TWAI_RX_RING].buf;

			for (uint32_t i = 0U; i < CAN_SJA1000_RX_BUFFER_LEN; i++) {
				buf[i] = (uint8_t)twai_rx_reg(rx, CAN_SJA1000_FRAME_INFO + i);
			}
			head++;
		}
		twai_rx_reg_write(rx, CAN_SJA1000_CMR, CAN_SJA1000_CMR_RRB);
	}
	if ((twai_rx_reg(rx, CAN_SJA1000_SR) & CAN_SJA1000_SR_DOS) != 0U) {
		twai_rx_reg_write(rx, CAN_SJA1000_CMR, CAN_SJA1000_CMR_CDO);
	}
	twai_rx_buf_unlock(&rx->sja->buf_lock, key);

	__atomic_store_n(&rx->head, head, __ATOMIC_RELEASE);
	fill = head - __atomic_load_n(&rx->tail, __ATOMIC_ACQUIRE);
	if (fill > rx->high_water) {
		rx->high_water = fill;
	}
	if ((ir & ~CAN_SJA1000_IR_RI) != 0U) {
		(void)__atomic_fetch_or(&rx->pending_ir, ir & ~CAN_SJA1000_IR_RI, __ATOMIC_ACQ_REL);
	}
	if (!esp_mp_stall_cpu_parked(esp_cpu_get_core_id())) {
		can_esp32_twai_rx_dispatch(rx);
	}
}

static void can_esp32_twai_rx_thread(void *p1, void *p2, void *p3)
{
	struct can_esp32_twai_rx *rx = p1;
	const struct can_sja1000_config *sja1000_config = rx->dev->config;
	const struct can_esp32_twai_config *twai_config = sja1000_config->custom;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	/* An ESP32 interrupt goes to the core that allocates it: this thread's. */
	rx->alloc_result = esp_intr_alloc(twai_config->irq_source,
					  BIT(CONFIG_CAN_ESP32_TWAI_RX_IRAM_LEVEL) |
						  ESP_INT_FLAGS_CHECK(twai_config->irq_flags) |
						  ESP_INTR_FLAG_IRAM,
					  can_esp32_twai_rx_isr, rx, &rx->handle);
	if (rx->alloc_result != 0) {
		LOG_ERR("could not allocate the IRAM receive interrupt (err %d)",
			rx->alloc_result);
		return;
	}
	LOG_INF("IRAM receive interrupt on cpu%d, level %d, ring %d frames",
		(int)esp_cpu_get_core_id(), CONFIG_CAN_ESP32_TWAI_RX_IRAM_LEVEL, TWAI_RX_RING);

	uint32_t reported_missed = 0U;
	uint32_t reported_full = 0U;
	int64_t next_report = k_uptime_get() + 10000;

	for (;;) {
		k_msleep(CONFIG_CAN_ESP32_TWAI_RX_POLL_MS);
		if (k_uptime_get() >= next_report) {
			next_report += 10000;
			/* Only when a frame was lost since the last line. */
			if (rx->missed != reported_missed || rx->ring_full != reported_full) {
				reported_missed = rx->missed;
				reported_full = rx->ring_full;
				LOG_WRN("rx lost: overrun %u ring_full %u (high water %u of %d)",
					reported_missed, reported_full, rx->high_water,
					TWAI_RX_RING);
			}
		}
		if (__atomic_load_n(&rx->head, __ATOMIC_ACQUIRE) != rx->tail ||
		    __atomic_load_n(&rx->pending_ir, __ATOMIC_ACQUIRE) != 0U) {
			/*
			 * Left over from a stall. Same core: with the line off the handler
			 * cannot run beside this.
			 */
			esp_intr_disable(rx->handle);
			can_esp32_twai_rx_dispatch(rx);
			esp_intr_enable(rx->handle);
		}
	}
}

static int can_esp32_twai_rx_start(const struct device *dev, k_thread_stack_t *stack,
				   size_t stack_size)
{
	const struct can_sja1000_config *sja1000_config = dev->config;
	const struct can_esp32_twai_config *twai_config = sja1000_config->custom;
	struct can_sja1000_data *data = dev->data;
	struct can_esp32_twai_rx *rx = data->custom;
	k_tid_t tid;

	rx->base = twai_config->base;
	rx->sja = data;
	rx->dev = dev;
	rx->alloc_result = -EAGAIN;
	/* Created before the second core runs (POST_KERNEL); it starts there once it is up. */
	tid = k_thread_create(&rx->thread, stack, stack_size, can_esp32_twai_rx_thread, rx, NULL,
			      NULL, K_PRIO_PREEMPT(CONFIG_CAN_ESP32_TWAI_RX_THREAD_PRIORITY), 0,
			      K_FOREVER);
	k_thread_name_set(tid, "twai_rx");
	if (k_thread_cpu_pin(tid, CONFIG_CAN_ESP32_TWAI_RX_IRAM_CPU) != 0) {
		return -EINVAL;
	}
	k_thread_start(tid);
	return 0;
}
#endif /* CONFIG_CAN_ESP32_TWAI_RX_IRAM */

static int can_esp32_twai_init(const struct device *dev)
{
	const struct can_sja1000_config *sja1000_config = dev->config;
	const struct can_esp32_twai_config *twai_config = sja1000_config->custom;
	int err;

	if (!device_is_ready(twai_config->clock_dev)) {
		LOG_ERR_DEVICE_NOT_READY(twai_config->clock_dev);
		return -ENODEV;
	}

	err = pinctrl_apply_state(twai_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		LOG_ERR("failed to configure TWAI pins (err %d)", err);
		return err;
	}

	err = clock_control_on(twai_config->clock_dev, twai_config->clock_subsys);
	if (err != 0) {
		LOG_ERR("failed to enable CAN clock (err %d)", err);
		return err;
	}

	err = can_sja1000_init(dev);
	if (err != 0) {
		LOG_ERR("failed to initialize controller (err %d)", err);
		return err;
	}

#ifndef CONFIG_SOC_SERIES_ESP32
	/*
	 * TWAI_CLOCK_DIVIDER_REG is incompatible with CAN_SJA1000_CDR for non-ESP32 MCUs
	 *   - TWAI_CD has length of 8 bits instead of 3 bits
	 *   - TWAI_CLOCK_OFF at BIT(8) instead of BIT(3)
	 *   - TWAI_EXT_MODE bit missing (always "extended" = PeliCAN mode)
	 *
	 * Overwrite with 32-bit register variant configured via devicetree.
	 */
	can_esp32_twai_write_reg32(dev, TWAI_CLOCK_DIVIDER_REG, twai_config->cdr32);
#endif /* !CONFIG_SOC_SERIES_ESP32 */

#if defined(CONFIG_CAN_ESP32_TWAI_RX_IRAM)
	err = can_esp32_twai_rx_start(dev, twai_config->rx_stack, twai_config->rx_stack_size);
#else
	err = esp_intr_alloc(twai_config->irq_source,
			ESP_PRIO_TO_FLAGS(twai_config->irq_priority) |
			ESP_INT_FLAGS_CHECK(twai_config->irq_flags) | ESP_INTR_FLAG_IRAM,
			can_esp32_twai_isr, (void *)dev, NULL);
#endif

	if (err != 0) {
		LOG_ERR("could not allocate interrupt (err %d)", err);
	}

	return err;
}

DEVICE_API(can, can_esp32_twai_driver_api) = {
	.get_capabilities = can_sja1000_get_capabilities,
	.start = can_sja1000_start,
	.stop = can_sja1000_stop,
	.set_mode = can_sja1000_set_mode,
#ifdef CONFIG_SOC_SERIES_ESP32
	.set_timing = can_sja1000_set_timing,
#else
	.set_timing = can_esp32_twai_set_timing,
#endif /* CONFIG_SOC_SERIES_ESP32 */
	.send = can_sja1000_send,
	.add_rx_filter = can_sja1000_add_rx_filter,
	.remove_rx_filter = can_sja1000_remove_rx_filter,
	.get_state = can_sja1000_get_state,
	.get_core_clock = can_esp32_twai_get_core_clock,
	.get_max_filters = can_sja1000_get_max_filters,
#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
	.recover = can_sja1000_recover,
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */
	.timing_min = CAN_SJA1000_TIMING_MIN_INITIALIZER,
#ifdef CONFIG_SOC_SERIES_ESP32
	.timing_max = CAN_SJA1000_TIMING_MAX_INITIALIZER,
#else
	/* larger prescaler allowed for newer ESP32-series MCUs */
	.timing_max = {
		.sjw = 0x4,
		.prop_seg = 0x0,
		.phase_seg1 = 0x10,
		.phase_seg2 = 0x8,
		.prescaler = 0x2000,
	}
#endif /* CONFIG_SOC_SERIES_ESP32 */
};

#ifdef CONFIG_SOC_SERIES_ESP32
#define TWAI_CLKOUT_DIVIDER_MAX (14)
#define TWAI_CDR32_INIT(inst)
#else
#define TWAI_CLKOUT_DIVIDER_MAX (490)
#define TWAI_CDR32_INIT(inst) .cdr32 = CAN_ESP32_TWAI_DT_CDR_INST_GET(inst)
#endif /* CONFIG_SOC_SERIES_ESP32 */

#define CAN_ESP32_TWAI_ASSERT_CLKOUT_DIVIDER(inst)                                                 \
	BUILD_ASSERT(COND_CODE_0(DT_INST_NODE_HAS_PROP(inst, clkout_divider), (1),                 \
		(DT_INST_PROP(inst, clkout_divider) == 1 ||                                        \
		(DT_INST_PROP(inst, clkout_divider) % 2 == 0 &&                                    \
		DT_INST_PROP(inst, clkout_divider) / 2 <= TWAI_CLKOUT_DIVIDER_MAX))),              \
		"TWAI clkout-divider from dts invalid")

#define CAN_ESP32_TWAI_DT_CDR_INST_GET(inst)                                                       \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, clkout_divider),                                   \
		    COND_CODE_1(DT_INST_PROP(inst, clkout_divider) == 1, (TWAI_CD_MASK),           \
				((DT_INST_PROP(inst, clkout_divider)) / 2 - 1)),                   \
		    (TWAI_CLOCK_OFF))

#if defined(CONFIG_CAN_ESP32_TWAI_RX_IRAM)
#define TWAI_RX_IRAM_DEFINE(inst)                                                                  \
	static struct can_esp32_twai_rx can_esp32_twai_rx_##inst;                                  \
	static K_THREAD_STACK_DEFINE(can_esp32_twai_rx_stack_##inst,                               \
				     CONFIG_CAN_ESP32_TWAI_RX_THREAD_STACK_SIZE);
#define TWAI_RX_IRAM_CONFIG(inst)                                                                  \
	.rx_stack = can_esp32_twai_rx_stack_##inst,                                                \
	.rx_stack_size = K_THREAD_STACK_SIZEOF(can_esp32_twai_rx_stack_##inst),
#define TWAI_RX_IRAM_CUSTOM(inst) (&can_esp32_twai_rx_##inst)
#else
#define TWAI_RX_IRAM_DEFINE(inst)
#define TWAI_RX_IRAM_CONFIG(inst)
#define TWAI_RX_IRAM_CUSTOM(inst) NULL
#endif

#define CAN_ESP32_TWAI_INIT(inst)                                                                  \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	TWAI_RX_IRAM_DEFINE(inst)                                                                  \
                                                                                                   \
	static const struct can_esp32_twai_config can_esp32_twai_config_##inst = {                 \
		TWAI_RX_IRAM_CONFIG(inst)                                                          \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, offset),         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.irq_source = DT_INST_IRQ_BY_IDX(inst, 0, irq),                                    \
		.irq_priority = DT_INST_IRQ_BY_IDX(inst, 0, priority),                             \
		.irq_flags = DT_INST_IRQ_BY_IDX(inst, 0, flags),                                   \
		TWAI_CDR32_INIT(inst)                                                              \
	};                                                                                         \
	CAN_ESP32_TWAI_ASSERT_CLKOUT_DIVIDER(inst);                                                \
	static const struct can_sja1000_config can_sja1000_config_##inst =                         \
		CAN_SJA1000_DT_CONFIG_INST_GET(inst, &can_esp32_twai_config_##inst,                \
					can_esp32_twai_read_reg, can_esp32_twai_write_reg,         \
					CAN_SJA1000_OCR_OCMODE_BIPHASE,                            \
					COND_CODE_0(IS_ENABLED(CONFIG_SOC_SERIES_ESP32), (0),      \
					(CAN_ESP32_TWAI_DT_CDR_INST_GET(inst))), 25000);           \
                                                                                                   \
	static struct can_sja1000_data can_sja1000_data_##inst =                                   \
		CAN_SJA1000_DATA_INITIALIZER(TWAI_RX_IRAM_CUSTOM(inst));                           \
                                                                                                   \
	CAN_DEVICE_DT_INST_DEFINE(inst, can_esp32_twai_init, NULL, &can_sja1000_data_##inst,       \
				  &can_sja1000_config_##inst, POST_KERNEL,                         \
				  CONFIG_CAN_INIT_PRIORITY, &can_esp32_twai_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CAN_ESP32_TWAI_INIT)
