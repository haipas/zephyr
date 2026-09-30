/*
 * Copyright (c) 2022 Grant Ramsay <grant.ramsay@hotmail.com>
 * Copyright (c) 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_ETH_ESP32_PRIV_H_
#define ZEPHYR_DRIVERS_ETHERNET_ETH_ESP32_PRIV_H_

#include <stdlib.h>

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/esp-pinctrl-common.h>

#include <esp_clk_tree.h>
#include <esp_private/esp_clk_tree_common.h>
#include <hal/emac_periph.h>
#include <soc/gpio_periph.h>
#include <soc/io_mux_reg.h>
#if SOC_EMAC_USE_MULTI_IO_MUX
#include <hal/gpio_ll.h>
#endif
#if SOC_EMAC_SUPPORT_1000M
#include <hal/emac_clk.h>
#include <hal/emac_hal.h>
#include <hal/emac_ll.h>
#endif

/*
 * Select an IOMUX function on a pad and enable a pad's input buffer.
 * ESP32-S31 has no GPIO_PIN_MUX_REG[] table; its IO_MUX is accessed
 * through the gpio_ll helpers instead.
 */
#if defined(CONFIG_SOC_SERIES_ESP32S31)
#define ESP32_EMAC_PAD_FUNC_SEL(gpio, func) gpio_ll_func_sel(&GPIO, (gpio), (func))
#define ESP32_EMAC_PAD_INPUT_ENABLE(gpio)   gpio_ll_input_enable(&GPIO, (gpio))
#else
#define ESP32_EMAC_PAD_FUNC_SEL(gpio, func) PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], func)
#define ESP32_EMAC_PAD_INPUT_ENABLE(gpio)   PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[gpio])
#endif

/*
 * Find the IOMUX entry matching a specific GPIO number.
 * Returns the entry pointer, or the first entry if gpio_num < 0.
 */
static inline const emac_iomux_info_t *esp32_emac_iomux_find(const emac_iomux_info_t *info,
							     int gpio_num)
{
	if (info == NULL) {
		return NULL;
	}

	if (gpio_num < 0) {
		return info;
	}

	while (info->gpio_num != GPIO_NUM_MAX) {
		if (info->gpio_num == gpio_num) {
			return info;
		}
		info++;
	}

	return NULL;
}

static inline void esp32_emac_iomux_output(const emac_iomux_info_t *pin, uint32_t sig_idx)
{
	if (pin == NULL) {
		return;
	}
	ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	ARG_UNUSED(sig_idx);
}

static inline void esp32_emac_iomux_input(const emac_iomux_info_t *pin, uint32_t sig_idx)
{
	if (pin == NULL) {
		return;
	}
	ESP32_EMAC_PAD_INPUT_ENABLE(pin->gpio_num);
	ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
#if SOC_EMAC_USE_MULTI_IO_MUX
	/*
	 * On P4, tell the peripheral to read input from IOMUX pad
	 * (sig_in_sel=0) rather than through GPIO matrix.
	 */
	gpio_ll_set_input_signal_from(&GPIO, sig_idx, false);
#else
	ARG_UNUSED(sig_idx);
#endif
}

/*
 * Initialize RMII data plane IOMUX pins.
 *
 * On ESP32, each signal has a single dedicated IOMUX pin.
 * On ESP32-P4 (SOC_EMAC_USE_MULTI_IO_MUX), each signal has
 * multiple IOMUX options and input signals need sig_in_sel
 * configured to read from IOMUX pad.
 * Pass the desired GPIO number per signal, or -1 for default.
 */
static inline void esp32_emac_iomux_init_rmii(int gpio_tx_en, int gpio_txd0, int gpio_txd1,
					      int gpio_crs_dv, int gpio_rxd0, int gpio_rxd1)
{
	esp32_emac_iomux_output(esp32_emac_iomux_find(emac_rmii_iomux_pins.tx_en, gpio_tx_en),
				emac_io_idx.mii_tx_en_o_idx);
	esp32_emac_iomux_output(esp32_emac_iomux_find(emac_rmii_iomux_pins.txd0, gpio_txd0),
				emac_io_idx.mii_txd0_o_idx);
	esp32_emac_iomux_output(esp32_emac_iomux_find(emac_rmii_iomux_pins.txd1, gpio_txd1),
				emac_io_idx.mii_txd1_o_idx);
	esp32_emac_iomux_input(esp32_emac_iomux_find(emac_rmii_iomux_pins.crs_dv, gpio_crs_dv),
			       emac_io_idx.mii_rx_dv_i_idx);
	esp32_emac_iomux_input(esp32_emac_iomux_find(emac_rmii_iomux_pins.rxd0, gpio_rxd0),
			       emac_io_idx.mii_rxd0_i_idx);
	esp32_emac_iomux_input(esp32_emac_iomux_find(emac_rmii_iomux_pins.rxd1, gpio_rxd1),
			       emac_io_idx.mii_rxd1_i_idx);
}

/*
 * Read the RMII GPIOs from the pinctrl-0 "default" state and route them
 * through the dedicated EMAC IOMUX. RMII pads bypass the GPIO matrix, so
 * pinctrl_apply_state() is not used; the cells only carry (slot, GPIO)
 * pairs (see ESP32_RMII_PINMUX). REF_CLK is returned via clk_gpio.
 */
static inline int esp32_emac_iomux_init_rmii_pinctrl(const struct pinctrl_dev_config *pcfg,
						     int *clk_gpio)
{
	const struct pinctrl_state *state;
	int gpios[ESP_RMII_SLOT_COUNT];
	int res;

	BUILD_ASSERT(ESP_RMII_SLOT_COUNT == ESP_RMII_RXD1 + 1,
		     "ESP_RMII_SLOT_COUNT out of sync with the ESP_RMII_* slots");

	for (int i = 0; i < ESP_RMII_SLOT_COUNT; i++) {
		gpios[i] = -1;
	}

	res = pinctrl_lookup_state(pcfg, PINCTRL_STATE_DEFAULT, &state);
	if (res != 0) {
		return res;
	}

	for (uint8_t i = 0; i < state->pin_cnt; i++) {
		uint32_t cell = state->pins[i].pinmux;

		if ((cell & ESP32_RMII_MARKER) == 0) {
			continue;
		}

		uint32_t slot = (cell >> ESP32_RMII_SLOT_SHIFT) & ESP32_RMII_SLOT_MASK;
		int gpio = (cell >> ESP32_PIN_NUM_SHIFT) & ESP32_PIN_NUM_MASK;

		if (slot < ARRAY_SIZE(gpios)) {
			gpios[slot] = gpio;
		}
	}

	esp32_emac_iomux_init_rmii(gpios[ESP_RMII_TX_EN], gpios[ESP_RMII_TXD0],
				   gpios[ESP_RMII_TXD1], gpios[ESP_RMII_CRS_DV],
				   gpios[ESP_RMII_RXD0], gpios[ESP_RMII_RXD1]);

	if (clk_gpio != NULL) {
		*clk_gpio = gpios[ESP_RMII_CLK];
	}

	return 0;
}

static inline void esp32_emac_iomux_rmii_clk_output(int gpio_num)
{
	const emac_iomux_info_t *pin = emac_rmii_iomux_pins.clko;

	/*
	 * GPIO0 uses CLK_OUT1 instead of dedicated IOMUX, so skip
	 * IOMUX setup for it. GPIO16/17 use dedicated IOMUX.
	 */
	if (gpio_num == 0) {
		return;
	}

	esp32_emac_iomux_output(esp32_emac_iomux_find(pin, gpio_num), 0);
}

#if DT_NODE_HAS_PROP(DT_INST(0, espressif_esp32_eth), ref_clk_output_gpios)
static inline int esp32_emac_config_apll_clock(void)
{
	uint32_t expt_freq = MHZ(50);
	uint32_t real_freq = 0;
	esp_err_t ret = esp_clk_tree_src_set_freq_hz(SOC_MOD_CLK_APLL, expt_freq, &real_freq);

	if (ret == ESP_ERR_INVALID_ARG) {
		LOG_ERR("Set APLL clock coefficients failed");
		return -EIO;
	}

	/* If the difference of real APLL frequency
	 * is not within 50 ppm, i.e. 2500 Hz,
	 * the APLL is unavailable
	 */
	if (abs((int)real_freq - (int)expt_freq) > 2500) {
		LOG_ERR("The APLL is working at an unusable frequency");
		return -EIO;
	}

	return 0;
}
#else
static inline void esp32_emac_iomux_rmii_clk_input(int gpio_num)
{
	const emac_iomux_info_t *pin;

	pin = esp32_emac_iomux_find(emac_rmii_iomux_pins.clki, gpio_num);
	if (pin != NULL) {
		ESP32_EMAC_PAD_INPUT_ENABLE(pin->gpio_num);
		ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	}
}
#endif

static inline void esp32_emac_iomux_init_mii(void)
{
	const emac_iomux_info_t *pin;

	/*
	 * ESP32 EMAC uses dedicated IOMUX pins, not GPIO matrix.
	 * Only PIN_FUNC_SELECT is needed to route signals.
	 */

	/* TX_CLK - input */
	pin = emac_mii_iomux_pins.clk_tx;
	if (pin != NULL) {
		ESP32_EMAC_PAD_INPUT_ENABLE(pin->gpio_num);
		ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	}

	/* TX_EN - output */
	pin = emac_mii_iomux_pins.tx_en;
	if (pin != NULL) {
		ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	}

	/* TXD0-3 - outputs */
	pin = emac_mii_iomux_pins.txd0;
	if (pin != NULL) {
		ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	}

	pin = emac_mii_iomux_pins.txd1;
	if (pin != NULL) {
		ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	}

	pin = emac_mii_iomux_pins.txd2;
	if (pin != NULL) {
		ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	}

	pin = emac_mii_iomux_pins.txd3;
	if (pin != NULL) {
		ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	}

	/* RX_CLK - input */
	pin = emac_mii_iomux_pins.clk_rx;
	if (pin != NULL) {
		ESP32_EMAC_PAD_INPUT_ENABLE(pin->gpio_num);
		ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	}

	/* RX_DV - input */
	pin = emac_mii_iomux_pins.rx_dv;
	if (pin != NULL) {
		ESP32_EMAC_PAD_INPUT_ENABLE(pin->gpio_num);
		ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	}

	/* RXD0-3 - inputs */
	pin = emac_mii_iomux_pins.rxd0;
	if (pin != NULL) {
		ESP32_EMAC_PAD_INPUT_ENABLE(pin->gpio_num);
		ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	}

	pin = emac_mii_iomux_pins.rxd1;
	if (pin != NULL) {
		ESP32_EMAC_PAD_INPUT_ENABLE(pin->gpio_num);
		ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	}

	pin = emac_mii_iomux_pins.rxd2;
	if (pin != NULL) {
		ESP32_EMAC_PAD_INPUT_ENABLE(pin->gpio_num);
		ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	}

	pin = emac_mii_iomux_pins.rxd3;
	if (pin != NULL) {
		ESP32_EMAC_PAD_INPUT_ENABLE(pin->gpio_num);
		ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	}
}

#if SOC_EMAC_SUPPORT_1000M

/* RGMII TX clock for 1000 Mbit/s, and the clocks for the slower modes */
#define ESP32_RGMII_CLK_1000M_HZ 125000000U
#define ESP32_RGMII_CLK_100M_HZ  25000000U
#define ESP32_RGMII_CLK_10M_HZ   2500000U
/* Allowed deviation of the RGMII TX clock */
#define ESP32_RGMII_CLK_PPM      100U

/*
 * Read the RGMII GPIOs from the pinctrl-0 "default" state. Like RMII, RGMII
 * pads bypass the GPIO matrix, so the cells only carry (slot, GPIO) pairs
 * (see ESP32_RGMII_PINMUX). A slot without a cell is left at -1, which
 * selects the first (default) IOMUX option of that signal.
 */
static inline int esp32_emac_rgmii_pinctrl_gpios(const struct pinctrl_dev_config *pcfg,
						 int gpios[ESP_RGMII_SLOT_COUNT])
{
	const struct pinctrl_state *state;
	int res;

	BUILD_ASSERT(ESP_RGMII_SLOT_COUNT == ESP_RGMII_RXD3 + 1,
		     "ESP_RGMII_SLOT_COUNT out of sync with the ESP_RGMII_* slots");

	for (int i = 0; i < ESP_RGMII_SLOT_COUNT; i++) {
		gpios[i] = -1;
	}

	res = pinctrl_lookup_state(pcfg, PINCTRL_STATE_DEFAULT, &state);
	if (res != 0) {
		return res;
	}

	for (uint8_t i = 0; i < state->pin_cnt; i++) {
		uint32_t cell = state->pins[i].pinmux;

		if ((cell & ESP32_RGMII_MARKER) == 0) {
			continue;
		}

		uint32_t slot = (cell >> ESP32_RMII_SLOT_SHIFT) & ESP32_RMII_SLOT_MASK;

		if (slot < ESP_RGMII_SLOT_COUNT) {
			gpios[slot] = (cell >> ESP32_PIN_NUM_SHIFT) & ESP32_PIN_NUM_MASK;
		}
	}

	return 0;
}

/*
 * Route one RGMII signal to its pad through the dedicated EMAC IOMUX
 * function, the way ESP-IDF's emac_esp_iomux_init() does: GPIO13..19 are
 * CNNT-domain pads that must first be handed over to the GMAC, inputs read
 * the pad directly (not through the GPIO matrix), and no pulls are applied.
 */
static inline int esp32_emac_rgmii_pad_init(int slot, int gpio_num)
{
	const emac_iomux_info_t *list;
	const emac_iomux_info_t *pin;
	uint32_t sig_idx;
	bool input = false;

	switch (slot) {
	case ESP_RGMII_TX_CLK:
		list = emac_rgmii_iomux_pins.clk_tx;
		sig_idx = emac_io_idx.mii_tx_clk_i_idx;
		break;
	case ESP_RGMII_TX_CTL:
		list = emac_rgmii_iomux_pins.tx_ctl;
		sig_idx = emac_io_idx.mii_tx_en_o_idx;
		break;
	case ESP_RGMII_TXD0:
		list = emac_rgmii_iomux_pins.txd0;
		sig_idx = emac_io_idx.mii_txd0_o_idx;
		break;
	case ESP_RGMII_TXD1:
		list = emac_rgmii_iomux_pins.txd1;
		sig_idx = emac_io_idx.mii_txd1_o_idx;
		break;
	case ESP_RGMII_TXD2:
		list = emac_rgmii_iomux_pins.txd2;
		sig_idx = emac_io_idx.mii_txd2_o_idx;
		break;
	case ESP_RGMII_TXD3:
		list = emac_rgmii_iomux_pins.txd3;
		sig_idx = emac_io_idx.mii_txd3_o_idx;
		break;
	case ESP_RGMII_RX_CLK:
		list = emac_rgmii_iomux_pins.clk_rx;
		sig_idx = emac_io_idx.mii_rx_clk_i_idx;
		input = true;
		break;
	case ESP_RGMII_RX_CTL:
		list = emac_rgmii_iomux_pins.rx_ctl;
		sig_idx = emac_io_idx.mii_rx_dv_i_idx;
		input = true;
		break;
	case ESP_RGMII_RXD0:
		list = emac_rgmii_iomux_pins.rxd0;
		sig_idx = emac_io_idx.mii_rxd0_i_idx;
		input = true;
		break;
	case ESP_RGMII_RXD1:
		list = emac_rgmii_iomux_pins.rxd1;
		sig_idx = emac_io_idx.mii_rxd1_i_idx;
		input = true;
		break;
	case ESP_RGMII_RXD2:
		list = emac_rgmii_iomux_pins.rxd2;
		sig_idx = emac_io_idx.mii_rxd2_i_idx;
		input = true;
		break;
	case ESP_RGMII_RXD3:
		list = emac_rgmii_iomux_pins.rxd3;
		sig_idx = emac_io_idx.mii_rxd3_i_idx;
		input = true;
		break;
	default:
		return -EINVAL;
	}

	pin = esp32_emac_iomux_find(list, gpio_num);
	if (pin == NULL) {
		return -EINVAL;
	}

	emac_ll_gpio_init(NULL, pin->gpio_num);

	if (input) {
		gpio_ll_set_input_signal_from(&GPIO, sig_idx, false);
		ESP32_EMAC_PAD_INPUT_ENABLE(pin->gpio_num);
	}
	ESP32_EMAC_PAD_FUNC_SEL(pin->gpio_num, pin->func);
	gpio_ll_pullup_dis(&GPIO, pin->gpio_num);
	gpio_ll_pulldown_dis(&GPIO, pin->gpio_num);

	return 0;
}

/*
 * Divide the EMAC reference clock (the PLL selected by
 * esp32_emac_rgmii_clock_init()) down to the RGMII TX clock of a link speed:
 * 125 MHz for 1000 Mbit/s, 25 MHz for 100 Mbit/s, 2.5 MHz for 10 Mbit/s.
 */
static inline int esp32_emac_rgmii_set_tx_clk(emac_hal_context_t *hal, uint32_t pll_hz,
					      uint32_t out_hz)
{
	uint32_t div = (pll_hz > out_hz) ? (pll_hz / out_hz) : 1U;
	uint32_t set_hz = pll_hz;
	uint32_t tol_hz = (uint32_t)((uint64_t)out_hz * ESP32_RGMII_CLK_PPM / 1000000U);

	if (emac_hal_ref_clock_div(hal, div - 1U) == ESP_OK) {
		set_hz = pll_hz / div;
	}

	if (abs((int)set_hz - (int)out_hz) > (int)tol_hz) {
		return -EIO;
	}

	return 0;
}

/*
 * Set up the internal RGMII TX clock like ESP-IDF does with
 * CONFIG_ETH_EMAC_RGMII_TX_CLK_SRC_AUTO: take the first internal EMAC clock
 * source (MPLL), program it to the lowest frequency on its grid that is a
 * multiple of 125 MHz (500 MHz), divide down to 125 MHz and route it to the
 * EMAC. The MPLL is shared with PSRAM: if PSRAM already runs it at another
 * frequency (e.g. 400 MHz for 200 MHz PSRAM), 125 MHz cannot be derived and
 * -EIO is returned. The PLL frequency is returned in pll_hz for later speed
 * changes.
 */
static inline int esp32_emac_rgmii_clock_init(emac_hal_context_t *hal, uint32_t *pll_hz)
{
	const emac_clk_info_t *info;
	uint32_t expt_hz = 0;
	uint32_t real_hz = 0;
	esp_err_t err;

	if (emac_clk_internal.clk_count == 0) {
		return -ENOTSUP;
	}
	info = emac_clk_internal.clk_src[0];

	if (info->step_hz > 0) {
		for (int32_t step = 1; step <= info->max_steps; step++) {
			uint32_t hz = (uint32_t)info->step_hz * (uint32_t)step;

			if (hz % ESP32_RGMII_CLK_1000M_HZ == 0) {
				expt_hz = hz;
				break;
			}
		}
	} else {
		expt_hz = ESP32_RGMII_CLK_1000M_HZ;
	}
	if (expt_hz == 0) {
		LOG_ERR("No %s frequency is a multiple of 125 MHz", info->clk_name);
		return -ENOTSUP;
	}

	if (esp_clk_tree_enable_src(info->clk_id, true) != ESP_OK) {
		LOG_ERR("%s enable failed", info->clk_name);
		return -EIO;
	}

	err = esp_clk_tree_src_set_freq_hz(info->clk_id, expt_hz, &real_hz);
	if (err == ESP_ERR_INVALID_STATE) {
		LOG_WRN("%s is in use already, running at %u Hz", info->clk_name, real_hz);
	} else if (err != ESP_OK) {
		esp_clk_tree_enable_src(info->clk_id, false);
		LOG_ERR("Setting %s to %u Hz failed", info->clk_name, expt_hz);
		return -EIO;
	}

	if (esp32_emac_rgmii_set_tx_clk(hal, real_hz, ESP32_RGMII_CLK_1000M_HZ) != 0) {
		LOG_ERR("Cannot derive the 125 MHz RGMII clock from %s at %u Hz "
			"(with PSRAM, use CONFIG_SPIRAM_SPEED_250M)",
			info->clk_name, real_hz);
		return -EIO;
	}

	/* The first entry of emac_clk_internal is selector value 0 (MPLL) */
	if (emac_clk_internal.clk_count > 1) {
		emac_hal_ref_clock_select(hal, 0);
	}
	emac_hal_ref_clock_enable(hal, true);

	*pll_hz = real_hz;

	return 0;
}

#endif /* SOC_EMAC_SUPPORT_1000M */

#endif /* ZEPHYR_DRIVERS_ETHERNET_ETH_ESP32_PRIV_H_ */
