/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef APP_CRASH_H_
#define APP_CRASH_H_

#include <stdbool.h>
#include <stdint.h>

/* What the last fatal error or watchdog reset left in RTC memory. */
struct app_crash {
	uint32_t magic;
	/* K_ERR_* reason, or APP_CRASH_WATCHDOG. */
	uint32_t reason;
	uint32_t pc, a0, ps, exccause, excvaddr;
	uint32_t cpu;
	uint32_t uptime_ms;
	char thread[16];
};

#define APP_CRASH_WATCHDOG 0xdeadU

/* Record a liveness watchdog expiry before the reset. */
void app_crash_note_watchdog(void);

/* The record from before this boot, if any. */
bool app_crash_last(struct app_crash *out);

/*
 * What a double exception left in RTC memory (no fatal handler runs for it).
 * The layout is fixed: app_crash_dx.S stores the fields by offset.
 */
struct app_crash_dx {
	uint32_t magic;
	uint32_t depc, exccause, excvaddr, epc1, ps;
	/* Stack pointer. */
	uint32_t a1;
	/* CPU (PRID). */
	uint32_t prid;
};

/* The double exception from before this boot, if any. */
bool app_crash_dx_last(struct app_crash_dx *out);

#endif /* APP_CRASH_H_ */
