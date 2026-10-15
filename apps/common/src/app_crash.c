/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Fatal errors and watchdog resets leave a record in RTC memory, which a
 * software reset keeps: with the console on USB the crash dump itself is
 * lost, so the next boot reports it instead.
 */

#include <stddef.h>
#include <string.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/fatal.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

#include <esp_attr.h>

#include "app_crash.h"

#define MAGIC 0x43524153U /* "CRAS" */

/* Base save area offsets (zephyr/offsets.h, ___xtensa_irq_bsa_t_*). */
#define BSA_EXCCAUSE 0x08
#define BSA_EXCVADDR 0x0c
#define BSA_PS       0x20
#define BSA_PC       0x24
#define BSA_A0       0x28

static RTC_NOINIT_ATTR struct app_crash record;

/* Written by the double exception breadcrumb (app_crash_dx.S). */
#define DX_MAGIC     0x44584331U /* "DXC1": not reported yet */
#define DX_REPORTED  0x44584330U
RTC_NOINIT_ATTR struct app_crash_dx app_crash_dx_record;
BUILD_ASSERT(offsetof(struct app_crash_dx, depc) == 4 && offsetof(struct app_crash_dx, prid) == 28,
	     "app_crash_dx.S stores by these offsets");
static bool have_dx;
static struct app_crash last;
static bool have_last;

static void note(uint32_t reason)
{
	const char *name = k_thread_name_get(k_current_get());

	record.reason = reason;
	record.cpu = arch_curr_cpu()->id;
	record.uptime_ms = k_uptime_get_32();
	memset(record.thread, 0, sizeof(record.thread));
	if (name != NULL) {
		strncpy(record.thread, name, sizeof(record.thread) - 1);
	}
	record.magic = MAGIC;
}

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	memset(&record, 0, sizeof(record));
	if (esf != NULL) {
		/* The frame starts with a pointer to the base save area. */
		const uint8_t *bsa = *(const uint8_t *const *)esf;

		record.exccause = *(const uint32_t *)(bsa + BSA_EXCCAUSE);
		record.excvaddr = *(const uint32_t *)(bsa + BSA_EXCVADDR);
		record.ps = *(const uint32_t *)(bsa + BSA_PS);
		record.pc = *(const uint32_t *)(bsa + BSA_PC);
		record.a0 = *(const uint32_t *)(bsa + BSA_A0);
	}
	note(reason);
	sys_reboot(SYS_REBOOT_COLD);
	CODE_UNREACHABLE;
}

void app_crash_note_watchdog(void)
{
	memset(&record, 0, sizeof(record));
	note(APP_CRASH_WATCHDOG);
}

bool app_crash_last(struct app_crash *out)
{
	if (have_last) {
		*out = last;
	}
	return have_last;
}

bool app_crash_dx_last(struct app_crash_dx *out)
{
	if (have_dx) {
		*out = app_crash_dx_record;
	}
	return have_dx;
}

static int app_crash_init(void)
{
	/* The values stay in RTC memory; only the magic marks them as old. */
	if (app_crash_dx_record.magic == DX_MAGIC) {
		have_dx = true;
		app_crash_dx_record.magic = DX_REPORTED;
	}
	if (record.magic == MAGIC) {
		last = record;
		have_last = true;
	}
	record.magic = 0;
	return 0;
}

SYS_INIT(app_crash_init, PRE_KERNEL_1, 0);
