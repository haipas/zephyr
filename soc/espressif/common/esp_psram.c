/*
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <rom/ets_sys.h>
#include <esp_psram.h>
#include <esp_private/esp_psram_extram.h>
#include <hal/cache_hal.h>
#include <zephyr/multi_heap/shared_multi_heap.h>
#if defined(CONFIG_SOC_ESP32_SPI_MEM_SUPPORT_TIMING_TUNING)
#include <esp_flash.h>
#include <esp_private/spi_flash_os.h>
#include <hal/spi_flash_hal.h>
#endif

extern int _instruction_reserved_start;
extern int _instruction_reserved_end;
extern int _rodata_reserved_start;
extern int _rodata_reserved_end;

extern int _ext_ram_bss_start;
extern int _ext_ram_bss_end;
extern int _ext_ram_heap_start;
extern int _ext_ram_heap_end;

struct shared_multi_heap_region smh_psram = {
	.attr = SMH_REG_ATTR_EXTERNAL,
};

int esp_psram_smh_init(void)
{
	shared_multi_heap_pool_init();
	return shared_multi_heap_add(&smh_psram, NULL);
}

#if defined(CONFIG_SOC_ESP32_SPI_MEM_SUPPORT_TIMING_TUNING)
/*
 * PSRAM timing tuning reprograms the MSPI core clock, which is shared
 * between flash and PSRAM. The esp_flash driver latched the flash
 * timing into its HAL context back in esp_flash_config(), so those
 * cached values are stale once tuning has run and SPI1 accesses
 * (erase, write) fail. Re-read the timing the tuning code settled on.
 */
static void esp_psram_refresh_flash_timing(void)
{
	spi_flash_hal_context_t *host;
	spi_flash_hal_timing_config_t timing = {0};

	if (!spi_flash_timing_is_tuned() || esp_flash_default_chip == NULL) {
		return;
	}

	host = (spi_flash_hal_context_t *)esp_flash_default_chip->host;

	spi_timing_get_flash_timing_param(&timing);

	host->clock_conf = timing.clock_config;
	host->extra_dummy = timing.extra_dummy;
	host->cs_setup = timing.cs_setup;
	host->cs_hold = timing.cs_hold;
}
#endif

#if CONFIG_ESP_SPIRAM && defined(CONFIG_SOC_SERIES_ESP32P4)
#include <hal/wdt_hal.h>
#include <soc/rtc.h>
#include <soc/reset_reasons.h>
#include <esp_rom_sys.h>
#include <esp_rom_serial_output.h>

/* 3d local: the RTC watchdog guards the PSRAM bring-up (3d-fw#1066).
 *
 * A reset that cuts an earlier boot inside PSRAM init (seen with a JTAG
 * system reset followed by a hart reset) can leave the PSRAM side in a state
 * that neither a CPU reset nor a system reset clears: the next boot's first
 * PSRAM register read (s_psram_common_transaction()) never completes, and no
 * other watchdog is armed this early (config_wdt() has disabled the boot-time
 * ones). Measured on the bench, an RTC watchdog reset (stage action
 * RESET_RTC) brings such a board back. So the RTC watchdog runs from the
 * first PSRAM transaction until the PSRAM is mapped, as the ESP-IDF
 * bootloader's RTC watchdog covers this phase, and turns the hang into a
 * reset that recovers. It is disarmed before the optional memory test and the
 * .ext_ram.bss clear, whose duration scales with the PSRAM size.
 *
 * The same leftover state can also make the PSRAM answer garbage instead of
 * nothing ("PSRAM chip is not connected"): init fails fast, and every CPU
 * reset that follows meets the same state again. A failed init therefore
 * waits for the armed RTC watchdog -- at most ESP32P4_PSRAM_INIT_TRIES times
 * in a row, counted in RTC memory, so that an image with CONFIG_ESP_SPIRAM on
 * a board without (working) PSRAM still boots on without it afterwards, as it
 * did before this guard existed.
 */
#define ESP32P4_PSRAM_INIT_RWDT_MS 2000U
#define ESP32P4_PSRAM_INIT_TRIES   3U
#define ESP32P4_PSRAM_TRIES_MAGIC  0x50535257U /* "PSRW" */

static uint32_t esp32p4_psram_tries_magic __attribute__((section(".rtc_noinit")));
static uint32_t esp32p4_psram_tries __attribute__((section(".rtc_noinit")));

static void esp32p4_psram_rwdt(bool arm)
{
	wdt_hal_context_t rwdt = RWDT_HAL_CONTEXT_DEFAULT();

	if (arm) {
		uint32_t ticks = (uint32_t)((uint64_t)ESP32P4_PSRAM_INIT_RWDT_MS *
					    rtc_clk_slow_freq_get_hz() / 1000U);

		wdt_hal_init(&rwdt, WDT_RWDT, 0, false);
		wdt_hal_write_protect_disable(&rwdt);
		wdt_hal_config_stage(&rwdt, WDT_STAGE0, ticks, WDT_STAGE_ACTION_RESET_RTC);
		wdt_hal_enable(&rwdt);
	} else {
		wdt_hal_write_protect_disable(&rwdt);
		wdt_hal_disable(&rwdt);
	}
	wdt_hal_write_protect_enable(&rwdt);
}
#define PSRAM_INIT_GUARD(arm) esp32p4_psram_rwdt(arm)
#else
#define PSRAM_INIT_GUARD(arm)
#endif

void esp_init_psram(void)
{
	intptr_t mapped_vaddr = 0;
	size_t mapped_size = 0;

	/*
	 * PSRAM chip init transitions the MSPI clock through low and high
	 * speed modes and reprograms the flash/PSRAM tuning registers.
	 * Cache lines fetched during that window can hold garbage. Run the
	 * chip-init step first, invalidate the flash IROM/DROM ranges, then
	 * run the rest of PSRAM init so the next flash fetch reloads with
	 * the final MSPI settings. ESP32 cache has no per-address
	 * invalidate primitive, so the invalidate step is skipped there.
	 */
	PSRAM_INIT_GUARD(true);
	if (esp_psram_chip_init()) {
		ets_printf("Failed to Initialize external RAM, aborting.\n");
		return;
	}

#if defined(CONFIG_SOC_ESP32_SPI_MEM_SUPPORT_TIMING_TUNING)
	esp_psram_refresh_flash_timing();
#endif

#if !defined(CONFIG_SOC_SERIES_ESP32)
	cache_hal_invalidate_addr((uint32_t)&_instruction_reserved_start,
				  (uint32_t)&_instruction_reserved_end -
					  (uint32_t)&_instruction_reserved_start);
	cache_hal_invalidate_addr((uint32_t)&_rodata_reserved_start,
				  (uint32_t)&_rodata_reserved_end -
					  (uint32_t)&_rodata_reserved_start);
#endif

	if (esp_psram_init()) {
		ets_printf("Failed to Initialize external RAM, aborting.\n");
		return;
	}
	PSRAM_INIT_GUARD(false);

	if (esp_psram_get_size() < CONFIG_ESP_SPIRAM_SIZE) {
		ets_printf("External RAM size is less than configured.\n");
	}

	esp_psram_get_mapped_region(&mapped_vaddr, &mapped_size);
	ARG_UNUSED(mapped_vaddr);
	ARG_UNUSED(mapped_size);

	/*
	 * Use the linker-reserved heap window inside the PSRAM-backed
	 * .ext_ram.data output section. The linker places ext_ram_noinit
	 * and ext_ram_bss content before the heap, so starting the SMH
	 * region at the raw MMU-mapped base would overlap and clobber
	 * those allocations (e.g. relocated thread stacks and net packet
	 * pools under CONFIG_ESP32_WIFI_NET_ALLOC_SPIRAM).
	 */
	smh_psram.addr = (uintptr_t)&_ext_ram_heap_start;
	smh_psram.size = (uintptr_t)&_ext_ram_heap_end -
			 (uintptr_t)&_ext_ram_heap_start;

	if (IS_ENABLED(CONFIG_ESP_SPIRAM_MEMTEST)) {
		if (esp_psram_is_initialized()) {
			if (!esp_psram_extram_test()) {
				ets_printf("External RAM failed memory test!");
				return;
			}
		}
	}

	memset(&_ext_ram_bss_start, 0,
	       (&_ext_ram_bss_end - &_ext_ram_bss_start) * sizeof(_ext_ram_bss_start));
}

#if CONFIG_ESP_SPIRAM && defined(CONFIG_SOC_SERIES_ESP32P4)
static int esp32p4_psram_init(void)
{
	if (esp32p4_psram_tries_magic != ESP32P4_PSRAM_TRIES_MAGIC ||
	    esp_rom_get_reset_reason(0) == RESET_REASON_CHIP_POWER_ON) {
		esp32p4_psram_tries_magic = ESP32P4_PSRAM_TRIES_MAGIC;
		esp32p4_psram_tries = 0U;
	}

	esp_init_psram();
	if (!esp_psram_is_initialized()) {
		if (esp32p4_psram_tries < ESP32P4_PSRAM_INIT_TRIES) {
			esp32p4_psram_tries++;
			ets_printf("PSRAM init failed (%u/%u), waiting for the RTC watchdog reset\n",
				   (unsigned int)esp32p4_psram_tries, ESP32P4_PSRAM_INIT_TRIES);
			esp32p4_psram_rwdt(true); /* fresh 2 s, whatever init left */
			for (;;) {
			}
		}
		esp32p4_psram_rwdt(false);
		ets_printf("PSRAM init failed %u times in a row, continuing without PSRAM\n",
			   ESP32P4_PSRAM_INIT_TRIES);
#ifdef CONFIG_ESP_CONSOLE_UART_NUM
		/* the UART driver resets the FIFO next; let the line out first */
		esp_rom_output_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
#endif
	} else {
		esp32p4_psram_tries = 0U;
	}

	if (esp_psram_smh_init()) {
		printk("Failed to initialize PSRAM shared multi heap\n");
	}

	return 0;
}

/*
 * On ESP32-P4 the PSRAM/MPLL rail is powered by an internal LDO owned by the
 * devicetree regulator driver, which comes up at PRE_KERNEL_1. Initialize
 * PSRAM at PRE_KERNEL_1 as well, at the default priority so it runs after the
 * regulator (lower priority value), and before POST_KERNEL consumers of the
 * external RAM heap.
 */
SYS_INIT(esp32p4_psram_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif
