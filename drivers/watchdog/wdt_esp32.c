/*
 * Copyright (C) 2017 Intel Corporation
 * Copyright (c) 2025-2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT espressif_esp32_watchdog

#if defined(CONFIG_SOC_SERIES_ESP32C5) || defined(CONFIG_SOC_SERIES_ESP32C61) ||                   \
	defined(CONFIG_SOC_SERIES_ESP32C6) || defined(CONFIG_SOC_SERIES_ESP32H2)
#include <soc/lp_aon_reg.h>
#elif !defined(CONFIG_SOC_SERIES_ESP32P4)
#include <soc/rtc_cntl_reg.h>
#endif
#include <soc/timer_group_reg.h>
#include <hal/mwdt_ll.h>
#include <hal/wdt_hal.h>
#include <esp_clk_tree.h>

#include <string.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/device.h>

#if CONFIG_ESP32_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP && SOC_MWDT_SUPPORT_SLEEP_RETENTION
#define WDT_SLEEP_RETENTION_ENABLED 1
#else
#define WDT_SLEEP_RETENTION_ENABLED 0
#endif

#if WDT_SLEEP_RETENTION_ENABLED
#include <hal/mwdt_periph.h>
#include <esp_private/sleep_retention.h>
#endif

#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
LOG_MODULE_REGISTER(wdt_esp32, CONFIG_WDT_LOG_LEVEL);

#define MWDT_TICK_PRESCALER		40000
#define MWDT_TICKS_PER_US		500

struct wdt_esp32_data {
	wdt_hal_context_t hal;
	uint32_t timeout;
	wdt_stage_action_t mode;
	wdt_callback_t callback;
	struct k_spinlock lock;
};

struct wdt_esp32_config {
	wdt_inst_t wdt_inst;
	const struct device *clock_dev;
	const clock_control_subsys_t clock_subsys;
	void (*connect_irq)(void);
	int irq_source;
	int irq_priority;
	int irq_flags;
};

static inline void wdt_esp32_seal(const struct device *dev)
{
	struct wdt_esp32_data *data = dev->data;

	wdt_hal_write_protect_enable(&data->hal);
}

static inline void wdt_esp32_unseal(const struct device *dev)
{
	struct wdt_esp32_data *data = dev->data;

	wdt_hal_write_protect_disable(&data->hal);
}

static void wdt_esp32_enable_locked(const struct device *dev)
{
	struct wdt_esp32_data *data = dev->data;

	wdt_esp32_unseal(dev);
	wdt_hal_enable(&data->hal);
	wdt_esp32_seal(dev);
}

static int wdt_esp32_disable(const struct device *dev)
{
	struct wdt_esp32_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	wdt_esp32_unseal(dev);
	wdt_hal_disable(&data->hal);
	wdt_esp32_seal(dev);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static void wdt_esp32_isr(void *arg);

static void wdt_esp32_feed_locked(const struct device *dev)
{
	struct wdt_esp32_data *data = dev->data;

	wdt_esp32_unseal(dev);
	wdt_hal_feed(&data->hal);
	wdt_esp32_seal(dev);
}

static int wdt_esp32_feed(const struct device *dev, int channel_id)
{
	struct wdt_esp32_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	wdt_esp32_feed_locked(dev);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int wdt_esp32_set_config(const struct device *dev, uint8_t options)
{
	struct wdt_esp32_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	wdt_esp32_unseal(dev);
	if (data->callback != NULL) {
		/* Stage 0 raises the interrupt that runs the callback, stage 1
		 * takes the configured action one timeout later unless the
		 * timer is fed in between. 3d local: before 3d-fw#1016 the ISR
		 * fed the timer itself, so the callback repeated every timeout and
		 * the action never came while interrupts ran; a callback user that
		 * wants to survive must now feed (or disable) from the callback.
		 */
		wdt_hal_config_stage(&data->hal, WDT_STAGE0, data->timeout, WDT_STAGE_ACTION_INT);
		wdt_hal_config_stage(&data->hal, WDT_STAGE1, data->timeout, data->mode);
	} else {
		/* 3d local: without a callback there is nobody to warn -- take the
		 * action at the configured timeout itself, independent of whether
		 * any CPU can still take interrupts.
		 */
		wdt_hal_config_stage(&data->hal, WDT_STAGE0, data->timeout, data->mode);
		wdt_hal_config_stage(&data->hal, WDT_STAGE1, 0, WDT_STAGE_ACTION_OFF);
	}
	wdt_esp32_seal(dev);

	wdt_esp32_enable_locked(dev);
	wdt_esp32_feed_locked(dev);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int wdt_esp32_install_timeout(const struct device *dev,
				     const struct wdt_timeout_cfg *cfg)
{
	struct wdt_esp32_data *data = dev->data;

	if (cfg->window.min != 0U || cfg->window.max == 0U) {
		return -EINVAL;
	}

	/* 3d local: window.max is in ms. The timer counts MWDT_CLK_SRC_DEFAULT /
	 * MWDT_TICK_PRESCALER: 1 kHz only from a 40 MHz XTAL (ESP32-P4); the
	 * APB-clocked ESP32/-S2/-S3 count at 2 kHz and a 48 MHz XTAL (ESP32-C5)
	 * at 1.2 kHz, where the window used to be shorter than requested.
	 */
	uint32_t src_hz = 0;

	if (esp_clk_tree_src_get_freq_hz((soc_module_clk_t)MWDT_CLK_SRC_DEFAULT,
					 ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &src_hz) != ESP_OK ||
	    src_hz == 0U) {
		return -EINVAL;
	}
	uint64_t ticks = (uint64_t)cfg->window.max * src_hz / MWDT_TICK_PRESCALER / 1000U;

	data->timeout = (uint32_t)MIN(MAX(ticks, 1U), UINT32_MAX);
	data->callback = cfg->callback;

	/* Set mode of watchdog and callback */
	switch (cfg->flags) {
	case WDT_FLAG_RESET_SOC:
		data->mode = WDT_STAGE_ACTION_RESET_SYSTEM;
		LOG_DBG("Configuring reset SOC mode");
		break;

	case WDT_FLAG_RESET_CPU_CORE:
		data->mode = WDT_STAGE_ACTION_RESET_CPU;
		LOG_DBG("Configuring reset CPU mode");
		break;

	case WDT_FLAG_RESET_NONE:
		data->mode = WDT_STAGE_ACTION_OFF;
		LOG_DBG("Configuring non-reset mode");
		break;

	default:
		LOG_ERR("Unsupported watchdog config flag");
		return -EINVAL;
	}

	return 0;
}

#if WDT_SLEEP_RETENTION_ENABLED
static esp_err_t wdt_create_sleep_retention_cb(void *arg)
{
	uint32_t group_id = (uint32_t)(uintptr_t)arg;
	sleep_retention_module_t module =
		(group_id == 0) ? SLEEP_RETENTION_MODULE_TG0_WDT : SLEEP_RETENTION_MODULE_TG1_WDT;

	return sleep_retention_entries_create(tg_wdt_regs_retention[group_id].link_list,
					      tg_wdt_regs_retention[group_id].link_num,
					      REGDMA_LINK_PRI_SYS_PERIPH_LOW, module);
}

static void wdt_esp32_sleep_retention_init(uint32_t group_id)
{
	sleep_retention_module_t module =
		(group_id == 0) ? SLEEP_RETENTION_MODULE_TG0_WDT : SLEEP_RETENTION_MODULE_TG1_WDT;

	sleep_retention_module_init_param_t init_param = {
		.cbs = {.create = {.handle = wdt_create_sleep_retention_cb,
				   .arg = (void *)(uintptr_t)group_id}},
		.attribute = SLEEP_RETENTION_MODULE_ATTR_ATTACH,
		.depends = RETENTION_MODULE_BITMAP_INIT(CLOCK_SYSTEM)};

	esp_err_t err = sleep_retention_module_init(module, &init_param);

	if (err == ESP_OK) {
		err = sleep_retention_module_allocate(module);
	}
	if (err == ESP_OK) {
		err = sleep_retention_module_attach(module);
	}
	if (err != ESP_OK) {
		LOG_WRN("WDT%d sleep retention init failed (%d)", group_id, err);
	}
}
#endif /* WDT_SLEEP_RETENTION_ENABLED */

static int wdt_esp32_init(const struct device *dev)
{
	const struct wdt_esp32_config *const config = dev->config;
	struct wdt_esp32_data *data = dev->data;
	int ret, flags;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	clock_control_on(config->clock_dev, config->clock_subsys);

	wdt_hal_init(&data->hal, config->wdt_inst, MWDT_TICK_PRESCALER, true);

	flags = ESP_PRIO_TO_FLAGS(config->irq_priority) | ESP_INT_FLAGS_CHECK(config->irq_flags) |
		ESP_INTR_FLAG_IRAM;
	ret = esp_intr_alloc(config->irq_source, flags, (intr_handler_t)wdt_esp32_isr, (void *)dev,
			     NULL);

	if (ret != 0) {
		LOG_ERR("could not allocate interrupt (err %d)", ret);
		return ret;
	}

#if WDT_SLEEP_RETENTION_ENABLED
	wdt_esp32_sleep_retention_init(config->wdt_inst - WDT_MWDT0);
#endif

	return 0;
}

static DEVICE_API(wdt, wdt_api) = {
	.setup = wdt_esp32_set_config,
	.disable = wdt_esp32_disable,
	.install_timeout = wdt_esp32_install_timeout,
	.feed = wdt_esp32_feed
};

#define ESP32_WDT_INIT(idx)							   \
	static struct wdt_esp32_data wdt##idx##_data;				   \
	static struct wdt_esp32_config wdt_esp32_config##idx = {		   \
		.wdt_inst = WDT_MWDT##idx,	\
		.irq_source = DT_IRQ_BY_IDX(DT_NODELABEL(wdt##idx), 0, irq),	\
		.irq_priority = DT_IRQ_BY_IDX(DT_NODELABEL(wdt##idx), 0, priority),	\
		.irq_flags = DT_IRQ_BY_IDX(DT_NODELABEL(wdt##idx), 0, flags),	\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(idx)), \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(idx, offset), \
	};									   \
										   \
	DEVICE_DT_INST_DEFINE(idx,						   \
			      wdt_esp32_init,					   \
			      NULL,						   \
			      &wdt##idx##_data,					   \
			      &wdt_esp32_config##idx,				   \
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	   \
			      &wdt_api)

static void IRAM_ATTR wdt_esp32_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct wdt_esp32_data *data = dev->data;
	k_spinlock_key_t key;

	if (data->callback) {
		data->callback(dev, 0);
	}

	/* 3d local: only acknowledge the stage-0 interrupt. wdt_hal_handle_intr()
	 * also feeds the timer, which restarted stage 0 on every expiry: as long
	 * as this ISR could run, stage 1 (the reset) was never reached, so a
	 * system that still took interrupts was never reset (3d-fw#1016).
	 */
	key = k_spin_lock(&data->lock);
	wdt_esp32_unseal(dev);
	mwdt_ll_clear_intr_status(data->hal.mwdt_dev);
	wdt_esp32_seal(dev);
	k_spin_unlock(&data->lock, key);
}


#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(wdt0))
ESP32_WDT_INIT(0);
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(wdt1))
ESP32_WDT_INIT(1);
#endif
