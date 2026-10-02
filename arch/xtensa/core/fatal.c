/*
 * Copyright (c) 2016 Cadence Design Systems, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/arch/cpu.h>
#include <inttypes.h>
#include <xtensa/config/specreg.h>
#include <xtensa_backtrace.h>
#include <zephyr/arch/xtensa/xtensa_ptr.h>
#include <zephyr/arch/common/exc_handle.h>

#include <xtensa_exc.h>
#include <xtensa_internal.h>
#include <xtensa_asm2_context.h>
#include <esp_rom_sys.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

#if defined(CONFIG_SIMULATOR_XTENSA) || defined(XT_SIMULATOR)
#include <xtensa/simcall.h>
#endif

char *xtensa_exccause(unsigned int cause_code)
{
#if defined(CONFIG_PRINTK) || defined(CONFIG_LOG)
	switch (cause_code) {
	case 0:
		return "illegal instruction";
	case 1:
		return "syscall";
	case 2:
		return "instr fetch error";
	case 3:
		return "load/store error";
	case 4:
		return "level-1 interrupt";
	case 5:
		return "alloca";
	case 6:
		return "divide by zero";
	case 8:
		return "privileged";
	case 9:
		return "load/store alignment";
	case 11:
		return "exclusive error";
	case 12:
		return "instr PIF data error";
	case 13:
		return "load/store PIF data error";
	case 14:
		return "instr PIF addr error";
	case 15:
		return "load/store PIF addr error";
	case 16:
		return "instr TLB miss";
	case 17:
		return "instr TLB multi hit";
	case 18:
		return "instr fetch privilege";
	case 20:
		return "inst fetch prohibited";
	case 24:
		return "load/store TLB miss";
	case 25:
		return "load/store TLB multi hit";
	case 26:
		return "load/store privilege";
	case 28:
		return "load prohibited";
	case 29:
		return "store prohibited";
	case 32: case 33: case 34: case 35: case 36: case 37: case 38: case 39:
		return "coprocessor disabled";
	case XTENSA_EXCCAUSE_CUSTOM_ZEPHYR_EXCEPTION:
		/* i.e. z_except_reason */
		return "zephyr exception";
	case XTENSA_EXCCAUSE_CUSTOM_KERNEL_OOPS:
		return "kernel oops";
	default:
		return "unknown/reserved";
	}
#else
	ARG_UNUSED(cause_code);
	return "na";
#endif
}

void xtensa_fatal_error(unsigned int reason, const struct arch_esf *esf)
{
#if defined(CONFIG_SMP)
	/* 3d DIAG (bench only, do not keep): one synchronous line before
	 * anything else, so the console has the first fault even when the
	 * rest of the fatal path wedges.
	 */
	{
		extern uint32_t xt_fatal_rec[16];
		const char *n = k_thread_name_get(_current);

		xt_fatal_rec[1] = reason;
		xt_fatal_rec[2] = arch_curr_cpu()->id;
		xt_fatal_rec[3] = (uint32_t)_current;
		xt_fatal_rec[4] = (uint32_t)_current->stack_info.start;
		xt_fatal_rec[5] = (uint32_t)_current->stack_info.size;
		xt_fatal_rec[6] = esf != NULL ? (uint32_t)*(int **)esf : 0U;
		xt_fatal_rec[7] = esf != NULL ? ((const _xtensa_irq_bsa_t *)*(int **)esf)->pc : 0U;
		for (int i = 0; i < 32; i++) {
			((char *)&xt_fatal_rec[8])[i] = (n != NULL && i < 31) ? n[i] : 0;
			if (n != NULL && n[i] == 0) {
				n = NULL;
			}
		}
		xt_fatal_rec[0] = 0x46544c31U;
	}
	if (esf != NULL) {
		const _xtensa_irq_bsa_t *bsa = (void *)*(int **)esf;
		const char *name = k_thread_name_get(_current);

		esp_rom_printf("\n!!FATAL cpu%u reason %u thread '%s' pc 0x%08x cause %u vaddr 0x%08x"
		       " ps 0x%08x a0 0x%08x nested %u\n",
		       (unsigned int)arch_curr_cpu()->id, reason, name != NULL ? name : "?",
		       bsa->pc, bsa->exccause, bsa->excvaddr, bsa->ps, bsa->a0,
		       arch_curr_cpu()->nested);
	} else {
		esp_rom_printf("\n!!FATAL cpu%u reason %u (no esf)\n",
		       (unsigned int)arch_curr_cpu()->id, reason);
	}
#endif
#ifdef CONFIG_EXCEPTION_DEBUG
	if (esf != NULL) {
		/* Don't want to get elbowed by xtensa_switch
		 * in between printing registers and dumping them;
		 * corrupts backtrace
		 */
		unsigned int key = arch_irq_lock();

		xtensa_dump_stack(esf);


#if defined(CONFIG_XTENSA_ENABLE_BACKTRACE)
#if XCHAL_HAVE_WINDOWED
		xtensa_backtrace_print(100, (int *)esf);
#endif
#endif
		arch_irq_unlock(key);
	}
#endif /* CONFIG_EXCEPTION_DEBUG */

#if defined(CONFIG_EXCEPTION_DUMP_HOOK)
	arch_exception_call_drain_hook(false);
#endif
	z_fatal_error(reason, esf);
}

#if defined(CONFIG_SIMULATOR_XTENSA) || defined(XT_SIMULATOR)
void xtensa_simulator_exit(int return_code)
{
	__asm__ (
	    "mov a3, %[code]\n\t"
	    "movi a2, %[call]\n\t"
	    "simcall\n\t"
	    :
	    : [code] "r" (return_code), [call] "i" (SYS_exit)
	    : "a3", "a2", "memory");

	CODE_UNREACHABLE;
}

FUNC_NORETURN void arch_system_halt(unsigned int reason)
{
	xtensa_simulator_exit(255 - reason);
	CODE_UNREACHABLE;
}
#endif

FUNC_NORETURN void arch_syscall_oops(void *ssf)
{
	xtensa_arch_kernel_oops(K_ERR_KERNEL_OOPS, ssf);

	CODE_UNREACHABLE;
}

#ifdef CONFIG_USERSPACE
void z_impl_xtensa_user_fault(unsigned int reason)
{
	if ((_current->base.user_options & K_USER) != 0) {
		if ((reason != K_ERR_KERNEL_OOPS) &&
				(reason != K_ERR_STACK_CHK_FAIL)) {
			reason = K_ERR_KERNEL_OOPS;
		}
	}
	xtensa_arch_except(reason);
}

static void z_vrfy_xtensa_user_fault(unsigned int reason)
{
	z_impl_xtensa_user_fault(reason);
}

#include <zephyr/syscalls/xtensa_user_fault_mrsh.c>

#endif /* CONFIG_USERSPACE */

#if defined(CONFIG_SMP) && defined(CONFIG_SOC_SERIES_ESP32S3)
/* 3d DIAG (bench only, do not keep): print the double exception record
 * xt_dblexc_capture (xtensa_asm2_util.S) left in RTC memory before the
 * watchdog reset; repeated every minute so the console capture catches it.
 */
__attribute__((section(".rtc_noinit"))) uint32_t xt_dblexc_rec[32];
/* 3d DIAG: window-state record of xt_ws_capture, read over JTAG. */
__attribute__((section(".rtc_noinit"))) uint32_t xt_ws_rec[32];
/* 3d DIAG: last fatal error (thread name, stack bounds), printed at boot. */
__attribute__((section(".rtc_noinit"))) uint32_t xt_fatal_rec[16];

static void xt_dblexc_print(void)
{
	if (xt_fatal_rec[0] == 0x46544c31U) {
		esp_rom_printf("\n!!LASTFATAL reason %u cpu %u thread %p '%s' stack 0x%08x+%u sp 0x%08x"
			       " pc 0x%08x\n",
			       xt_fatal_rec[1], xt_fatal_rec[2], (void *)xt_fatal_rec[3],
			       (const char *)&xt_fatal_rec[8], xt_fatal_rec[4], xt_fatal_rec[5],
			       xt_fatal_rec[6], xt_fatal_rec[7]);
	}
	for (int c = 0; c < 2; c++) {
		const uint32_t *r = &xt_dblexc_rec[c * 16];

		if (r[0] != 0x44424c31U) {
			continue;
		}
		printk("!!DBLEXC slot%d depc 0x%08x cause %u vaddr 0x%08x epc1 0x%08x ps 0x%08x"
		       " prid 0x%08x a0 0x%08x a1 0x%08x excsave1 0x%08x wb %u ws 0x%08x"
		       " cc 0x%08x hits %u\n",
		       c, r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10], r[11],
		       r[12], r[13]);
	}
}

static void xt_dblexc_timer_fn(struct k_timer *t)
{
	ARG_UNUSED(t);
	xt_dblexc_print();
}

static K_TIMER_DEFINE(xt_dblexc_timer, xt_dblexc_timer_fn, NULL);

static int xt_dblexc_report(void)
{
	xt_dblexc_print();
	k_timer_start(&xt_dblexc_timer, K_SECONDS(30), K_SECONDS(60));
	return 0;
}
SYS_INIT(xt_dblexc_report, APPLICATION, 99);
#endif
