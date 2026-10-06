/*
 * Copyright (c) 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>

#include <esp_cpu.h>
#include <esp_attr.h>
#include <esp_rom_sys.h>
#include <esp_intr_alloc.h>
#include <esp_ipc_isr.h>
#include <soc/system_reg.h>

#include <esp_mp_stall.h>

#if defined(CONFIG_XTENSA)
#include <xtensa/corebits.h>
#endif

#define STALL_SPIN_MAX 100000000U

static volatile DRAM_ATTR uint32_t s_stall_req[CONFIG_MP_MAX_NUM_CPUS];
static volatile DRAM_ATTR uint32_t s_stall_ack[CONFIG_MP_MAX_NUM_CPUS];
static volatile DRAM_ATTR bool s_stall_enabled;
static volatile DRAM_ATTR bool s_cpu_up[CONFIG_MP_MAX_NUM_CPUS];

/* Taken above any flash or cache lock; re-entrant on the owning core. */
static DRAM_ATTR atomic_t s_pause_owner = ATOMIC_INIT(-1);
static volatile DRAM_ATTR uint32_t s_pause_nest;
static volatile DRAM_ATTR unsigned int s_pause_irq_key[CONFIG_MP_MAX_NUM_CPUS];

/* 3d local (haipas/3d-fw#989): once set, every other core is held by the
 * hardware run-stall (esp_cpu_stall(), the RTC_CNTL SW_STALL bits) instead of
 * the cooperative IPI park, and pause/resume become no-ops. Set only on the
 * way into a fatal error: a stall request that was never acked, or a
 * requester that cannot be parked itself (exception, ISR or interrupts already
 * masked) finding the pause owned by a core that waits for that very ack. Both
 * used to end with both cores spinning on each other forever -- with the task
 * watchdog a SoC-fallback reset without a coredump. ESP-IDF's panic handler
 * stalls the other core the same way (panic_handler.c, esp_cpu_stall()). The
 * app core start clears the stall again (esp_cpu_unstall(1)).
 */
static volatile DRAM_ATTR bool s_hw_stalled;

/* How long a requester that cannot be parked waits for a foreign pause owner
 * before it stops cooperating; far above any flash operation. */
#define PAUSE_UNPARKABLE_SPIN_MAX 20000000U

static void IRAM_ATTR hw_stall_others(void)
{
	int me = esp_cpu_get_core_id();

	for (int cpu = 0; cpu < CONFIG_MP_MAX_NUM_CPUS; cpu++) {
		if (cpu != me && s_cpu_up[cpu]) {
			esp_cpu_stall(cpu);
		}
	}
	s_hw_stalled = true;
	barrier_dmem_fence_full();
}

static ALWAYS_INLINE void IRAM_ATTR stall_trigger_clear(int core_id)
{
	if (core_id == 0) {
		WRITE_PERI_REG(SYSTEM_CPU_INTR_FROM_CPU_2_REG, 0);
	} else {
		WRITE_PERI_REG(SYSTEM_CPU_INTR_FROM_CPU_3_REG, 0);
	}
}

static ALWAYS_INLINE void IRAM_ATTR stall_trigger_set(int other, bool assert_line)
{
	if (other == 1) {
		WRITE_PERI_REG(SYSTEM_CPU_INTR_FROM_CPU_3_REG,
			       assert_line ? SYSTEM_CPU_INTR_FROM_CPU_3 : 0);
	} else {
		WRITE_PERI_REG(SYSTEM_CPU_INTR_FROM_CPU_2_REG,
			       assert_line ? SYSTEM_CPU_INTR_FROM_CPU_2 : 0);
	}
}

/* 3d local: the parked core does NOT open its interrupt level. Opening it let
 * the CCOMPARE tick run on the parked core while the other core had the cache
 * off; sys_clock_announce() then ran expired k_timer callbacks, which live in
 * flash, and the core died on an instruction fetch (EXCCAUSE 0x14) without
 * ever leaving the stall ("esp_mp: cpu N did not leave the stall", Jenni
 * 2026-10-01). Keeping the level is what a single-core build does around a
 * flash operation anyway (irq_lock), so the radio already lives with it.
 */
#if defined(CONFIG_XTENSA) && defined(CONFIG_SOC_ESP32_MP_STALL_OPEN_IRQ)
static ALWAYS_INLINE uint32_t IRAM_ATTR stall_irq_open(void)
{
	uint32_t ps;

	__asm__ volatile("rsr.ps %0" : "=r"(ps));
	__asm__ volatile("wsr.ps %0\n\trsync" : : "r"(ps & ~PS_INTLEVEL_MASK));
	return ps;
}

static ALWAYS_INLINE void IRAM_ATTR stall_irq_restore(uint32_t ps)
{
	__asm__ volatile("wsr.ps %0\n\trsync" : : "r"(ps));
}
#else
static ALWAYS_INLINE uint32_t IRAM_ATTR stall_irq_open(void)
{
	return 0;
}

static ALWAYS_INLINE void IRAM_ATTR stall_irq_restore(uint32_t state)
{
	ARG_UNUSED(state);
}
#endif

void IRAM_ATTR esp_mp_stall_isr(const void *arg)
{
	int core_id = esp_cpu_get_core_id();
	uint32_t irq_state;
	uint32_t masked;

	ARG_UNUSED(arg);

	stall_trigger_clear(core_id);

	/* Mask before acking: the requester suspends the cache once it sees the ack. */
	masked = esp_intr_noniram_mask_local();
	s_stall_ack[core_id] = 1;
	barrier_dmem_fence_full();

	irq_state = stall_irq_open();

	/* arch_spin_relax() asserts on open interrupts, so spin on plain nops. */
	while (s_stall_req[core_id] != 0U) {
		arch_nop();
	}

	stall_irq_restore(irq_state);

	barrier_dmem_fence_full();
	s_stall_ack[core_id] = 0;
	esp_intr_noniram_unmask_local(masked);
}

static void IRAM_ATTR stall_other_cpu(void)
{
	int other = (esp_cpu_get_core_id() == 0) ? 1 : 0;
	uint32_t spins = 0;

	if (!s_stall_enabled || !s_cpu_up[other] || s_hw_stalled) {
		return;
	}

	/* s_stall_ack is owned by the parked core. */
	s_stall_req[other] = 1;

	barrier_dmem_fence_full();

	stall_trigger_set(other, true);

	while (s_stall_ack[other] == 0U) {
		arch_spin_relax();
		if (++spins > STALL_SPIN_MAX) {
			esp_rom_printf("esp_mp: cpu %d did not ack the stall\n", other);
			/* The fatal path writes the coredump to flash and pauses
			 * again: hold the peer in hardware so nobody waits on it a
			 * second time (3d-fw#989). */
			hw_stall_others();
			k_panic();
		}
	}
}

static void IRAM_ATTR release_other_cpu(void)
{
	int other = (esp_cpu_get_core_id() == 0) ? 1 : 0;
	uint32_t spins = 0;

	if (!s_stall_enabled || !s_cpu_up[other] || s_hw_stalled) {
		return;
	}

	s_stall_req[other] = 0;

	barrier_dmem_fence_full();

	stall_trigger_set(other, false);

	while (s_stall_ack[other] != 0U) {
		arch_spin_relax();
		if (++spins > STALL_SPIN_MAX) {
			esp_rom_printf("esp_mp: cpu %d did not leave the stall\n", other);
			k_panic();
		}
	}
}

void IRAM_ATTR soc_mp_pause_others(void)
{
	unsigned int key;
	atomic_val_t me;
	uint32_t unparkable_spins = 0;

	if (!s_stall_enabled || s_hw_stalled) {
		return;
	}

	key = arch_irq_lock();
	me = (atomic_val_t)esp_cpu_get_core_id();

	/* A parked core cannot be granted the pause. */
	if (s_stall_ack[me] != 0U) {
		esp_rom_printf("esp_mp: cpu %d requested a pause while parked\n", (int)me);
		k_panic();
	}

	if (atomic_get(&s_pause_owner) == me) {
		s_pause_nest++;
		arch_irq_unlock(key);
		return;
	}

	while (!atomic_cas(&s_pause_owner, -1, me)) {
		/* Open interrupts while waiting so the owner can park this core. */
		arch_irq_unlock(key);
		arch_nop();
		key = arch_irq_lock();

		/* 3d local (3d-fw#989): a requester in an exception or ISR, or one
		 * called with interrupts already masked, cannot take the park
		 * interrupt, so an owner waiting for its ack waits forever, and
		 * so does this loop -- the fatal/coredump path meeting a flash
		 * operation on the other core. Stop cooperating: hold the others
		 * in hardware and carry on as the only running core.
		 */
		if ((k_is_in_isr() || !arch_irq_unlocked(key)) &&
		    (++unparkable_spins > PAUSE_UNPARKABLE_SPIN_MAX)) {
			esp_rom_printf("esp_mp: cpu %d cannot be parked, pause owner %d: hw stall\n",
				       (int)me, (int)atomic_get(&s_pause_owner));
			hw_stall_others();
			arch_irq_unlock(key);
			return;
		}
	}

	s_pause_irq_key[me] = key;
	stall_other_cpu();

	s_pause_nest = 1;
}

void IRAM_ATTR soc_mp_resume_others(void)
{
	unsigned int key;

	if (!s_stall_enabled || s_hw_stalled) {
		return;
	}

	__ASSERT_NO_MSG(atomic_get(&s_pause_owner) == (atomic_val_t)esp_cpu_get_core_id());

	if (--s_pause_nest != 0) {
		return;
	}

	release_other_cpu();

	key = s_pause_irq_key[esp_cpu_get_core_id()];
	atomic_set(&s_pause_owner, -1);
	arch_irq_unlock(key);
}

void esp_mp_set_cpu_online(int cpu, bool online)
{
	barrier_dmem_fence_full();
	s_cpu_up[cpu] = online;
}

void esp_mp_stall_enable(void)
{
	barrier_dmem_fence_full();
	s_stall_enabled = true;
}

bool esp_mp_cpu_online(int cpu)
{
	return s_cpu_up[cpu];
}

void IRAM_ATTR esp_ipc_isr_stall_other_cpu(void)
{
	soc_mp_pause_others();
}

void IRAM_ATTR esp_ipc_isr_release_other_cpu(void)
{
	soc_mp_resume_others();
}
