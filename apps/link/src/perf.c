/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * iperf style throughput test over the MAC, on the "link" shell command:
 *
 *   link server [-i <s>] | link server stop
 *   link client <addr> [-t <s>] [-m qpsk|16qam|...|4096qam] [-f rs|hamming|none]
 *               [-n <units>] [-w 1|2]
 *               [-b <kbit/s>] [-i <s>]
 *   link stop | status | cal | set <key> <value> | dump [<ms>]
 *
 * The client sends test frames as fast as the MAC goes (or at -b), the
 * server counts what arrives; both print one line per interval and a total.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_heap.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>

#include <esp_attr.h>

#include "app_crash.h"
#include "mac.h"
#include "qam.h"

static const char *const fec_names[QAM_FECS] = {"rs", "hamming", "none"};
static const char *const mod_names[QAM_MODS] = {"qpsk",   "16qam",   "64qam",
							      "256qam", "1024qam", "4096qam"};

enum role {
	ROLE_NONE,
	ROLE_CLIENT,
	ROLE_SERVER,
};

static struct {
	enum role role;
	const struct shell *sh;
	uint32_t interval_ms;
	int64_t start_ms, last_ms;
	struct mac_stats first, prev;
} rep;

/*
 * Reports print from their own thread: shell output blocks while the host
 * does not read the port, which must not stall the system work queue (USB
 * runs there, and the liveness watchdog is fed from it).
 */
#define REPORT_STACK_SIZE 2048
#define REPORT_PRIORITY   K_PRIO_PREEMPT(13)

K_THREAD_STACK_DEFINE(report_stack, REPORT_STACK_SIZE);
static struct k_thread report_thread;
static K_SEM_DEFINE(report_kick, 0, 1);

static double mbit(uint64_t bytes, int64_t ms)
{
	return ms > 0 ? (double)bytes * 8.0 / (double)ms / 1000.0 : 0.0;
}

static void print_client(const struct mac_stats *a, const struct mac_stats *b, int64_t t0,
			 int64_t t1, const char *tag)
{
	uint64_t bytes = b->tx_bytes_acked - a->tx_bytes_acked;
	uint32_t sent = b->tx_attempts - a->tx_attempts, acked = b->tx_acked - a->tx_acked;

	shell_print(rep.sh,
		    "[client]%s %5.1f-%5.1f s %8.1f KB %7.3f Mbit/s  acked %u/%u sent, "
		    "dropped %u, busy %u, backoff %u",
		    tag, (double)(t0 - rep.start_ms) / 1000.0, (double)(t1 - rep.start_ms) / 1000.0,
		    (double)bytes / 1000.0, mbit(bytes, t1 - t0), acked, sent,
		    b->tx_dropped - a->tx_dropped, b->cca_busy - a->cca_busy,
		    b->backoff_slots - a->backoff_slots);
}

static void print_server(const struct mac_stats *a, const struct mac_stats *b, int64_t t0,
			 int64_t t1, const char *tag)
{
	uint64_t bytes = b->rx_bytes - a->rx_bytes;
	uint32_t ok = b->rx_ok - a->rx_ok, lost = b->rx_lost - a->rx_lost;
	uint32_t n = b->mer_n - a->mer_n;
	double mer = n ? (double)(b->mer_cdb_sum - a->mer_cdb_sum) / 100.0 / n : 0.0;
	double cfo = n ? (double)(b->cfo_hz_sum - a->cfo_hz_sum) / 1000.0 / n : 0.0;

	shell_print(rep.sh,
		    "[server]%s %5.1f-%5.1f s %8.1f KB %7.3f Mbit/s  frames %u, lost %u (%.1f%%), "
		    "dup %u, errors %u hdr %u data, MER %.1f dB, cfo %.1f kHz, FEC fixed %u",
		    tag, (double)(t0 - rep.start_ms) / 1000.0, (double)(t1 - rep.start_ms) / 1000.0,
		    (double)bytes / 1000.0, mbit(bytes, t1 - t0), ok, lost,
		    ok + lost ? 100.0 * lost / (ok + lost) : 0.0, b->rx_dup - a->rx_dup,
		    b->rx_hdr_err - a->rx_hdr_err, b->rx_data_err - a->rx_data_err, mer, cfo,
		    b->corrected - a->corrected);
}

/* One report; false once the role has ended. */
static bool report(void)
{
	struct mac_stats now;
	int64_t t = k_uptime_get();

	if (rep.role == ROLE_NONE) {
		return false;
	}
	link_mac_get_stats(&now);
	if (rep.role == ROLE_CLIENT) {
		print_client(&rep.prev, &now, rep.last_ms, t, "");
		if (!link_mac_client_active()) {
			/* The last frame went out before this report: end there. */
			print_client(&rep.first, &now, rep.start_ms,
				     MIN(t, link_mac_client_end_ms()), " total");
			rep.role = ROLE_NONE;
			return false;
		}
	} else {
		print_server(&rep.prev, &now, rep.last_ms, t, "");
	}
	rep.prev = now;
	rep.last_ms = t;
	return true;
}

static void report_loop(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		int64_t next;

		(void)k_sem_take(&report_kick, K_FOREVER);
		next = rep.start_ms;
		do {
			next += rep.interval_ms;
			/* A kick restarts the schedule (new role or stop). */
			if (k_sem_take(&report_kick, K_TIMEOUT_ABS_MS(next)) == 0) {
				k_sem_give(&report_kick);
				break;
			}
		} while (report());
	}
}

static void report_start(const struct shell *sh, enum role role, uint32_t interval_ms)
{
	rep.role = role;
	rep.sh = sh;
	rep.interval_ms = interval_ms;
	rep.start_ms = rep.last_ms = k_uptime_get();
	link_mac_get_stats(&rep.first);
	rep.prev = rep.first;
	k_sem_give(&report_kick);
}

static int parse_u32(const char *s, uint32_t *v)
{
	char *end;
	unsigned long x = strtoul(s, &end, 0);

	if (*s == '\0' || *end != '\0') {
		return -EINVAL;
	}
	*v = (uint32_t)x;
	return 0;
}

/* Seconds with an optional fraction, as ms. */
static int parse_ms(const char *s, uint32_t *ms)
{
	char *end;
	double x = strtod(s, &end);

	if (*s == '\0' || *end != '\0' || x < 0.0 || x > 86400.0) {
		return -EINVAL;
	}
	*ms = (uint32_t)(x * 1000.0);
	return 0;
}

static int cmd_server(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t interval = 1000;
	struct mac_cfg c;

	if (argc > 1 && strcmp(argv[1], "stop") == 0) {
		if (rep.role == ROLE_SERVER) {
			struct mac_stats now;

			link_mac_get_stats(&now);
			/* Up to the last frame: the client stopped before the server. */
			print_server(&rep.first, &now, rep.start_ms,
				     now.rx_last_ms > rep.start_ms ? now.rx_last_ms : k_uptime_get(),
				     " total");
			rep.role = ROLE_NONE;
		}
		return 0;
	}
	for (size_t i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-i") == 0 && i + 1 < argc &&
		    parse_ms(argv[++i], &interval) == 0 && interval >= 100) {
			continue;
		}
		shell_error(sh, "bad argument %s", argv[i]);
		return -EINVAL;
	}
	link_mac_get_cfg(&c);
	shell_print(sh, "server on address 0x%02x, reports every %u ms", c.addr, interval);
	report_start(sh, ROLE_SERVER, interval);
	return 0;
}

static int cmd_client(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t dst, duration = 10000, interval = 1000, ncw = 0, rate = 0, window = 2;
	enum qam_mod mod = QAM_QAM16;
	enum qam_fec fec = QAM_FEC_RS;
	int ret;

	if (argc < 2 || parse_u32(argv[1], &dst) != 0 || dst > 0xff) {
		shell_error(sh, "usage: link client <addr> [-t s] [-m mod] [-f fec] [-n units] [-w 1|2] [-b kbit/s] [-i s]");
		return -EINVAL;
	}
	for (size_t i = 2; i < argc; i++) {
		const char *opt = argv[i], *val = i + 1 < argc ? argv[i + 1] : NULL;

		if (val == NULL) {
			ret = -EINVAL;
		} else if (strcmp(opt, "-t") == 0) {
			ret = parse_ms(val, &duration);
		} else if (strcmp(opt, "-i") == 0) {
			ret = parse_ms(val, &interval);
			ret = ret == 0 && interval < 100 ? -EINVAL : ret;
		} else if (strcmp(opt, "-n") == 0) {
			ret = parse_u32(val, &ncw);
		} else if (strcmp(opt, "-w") == 0) {
			ret = parse_u32(val, &window);
		} else if (strcmp(opt, "-b") == 0) {
			ret = parse_u32(val, &rate);
		} else if (strcmp(opt, "-f") == 0) {
			ret = -EINVAL;
			for (int f = 0; f < QAM_FECS; f++) {
				if (strcmp(val, fec_names[f]) == 0) {
					fec = (enum qam_fec)f;
					ret = 0;
				}
			}
		} else if (strcmp(opt, "-m") == 0) {
			ret = -EINVAL;
			for (int m = 0; m < QAM_MODS; m++) {
				if (strcmp(val, mod_names[m]) == 0) {
					mod = (enum qam_mod)m;
					ret = 0;
				}
			}
		} else {
			ret = -EINVAL;
		}
		if (ret != 0) {
			shell_error(sh, "bad argument %s", opt);
			return -EINVAL;
		}
		i++;
	}
	ret = link_mac_client_start((uint8_t)dst, mod, fec, ncw, window, duration, rate);
	if (ret != 0) {
		shell_error(sh, "%s with %s takes 1 to %u units, window 1 or 2", mod_names[mod],
			    fec_names[fec], qam_ncw_max(mod, fec));
		return ret;
	}
	{
		struct qam_hdr h = {.mod = mod, .fec = fec, .ncw = ncw, .full = ncw == 0,
				    .pair = window == 2};

		struct mac_cfg mc;

		link_mac_get_cfg(&mc);
		if (mc.phy == MAC_PHY_OFDM) {
			shell_print(sh, "client to 0x%02x: OFDM %s, rs, %u B per frame, %u ms%s", dst,
				    mod_names[mod], (unsigned int)link_mac_frame_bytes(mod),
				    duration, duration == 0 ? " (until stopped)" : "");
		} else {
			shell_print(sh, "client to 0x%02x: %s, %s, %s, %u x %u B per transmission, %u ms%s",
				    dst, mod_names[mod], fec_names[fec],
				    ncw == 0 ? "full frames" : "whole units", window,
				    (unsigned int)qam_hdr_payload_bytes(&h), duration,
				    duration == 0 ? " (until stopped)" : "");
		}
	}
	report_start(sh, ROLE_CLIENT, interval);
	return 0;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	link_mac_client_stop();
	if (rep.role == ROLE_SERVER) {
		return cmd_server(sh, 2, (char *[]){"server", "stop"});
	}
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	struct mac_cfg c;
	struct mac_stats s;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	link_mac_get_cfg(&c);
	link_mac_get_stats(&s);
	uint32_t cause = 0;

	(void)hwinfo_get_reset_cause(&cause);
	shell_print(sh, "addr 0x%02x, %u MHz, tx gain %d, rx gain %d, bw %d MHz, cbw %d, amp %d, "
		    "%u samples/symbol, up %lld s, reset cause 0x%x",
		    c.addr, c.freq_mhz, c.tx_gain, c.rx_gain, c.bw_mhz, c.cbw, c.amp, QAM_SPS,
		    k_uptime_get() / 1000, cause);
	shell_print(sh, "air data %u us, ack %u us, ack timeout %u us, cca +%d dB over %.1f",
		    c.data_air_us, c.ack_air_us, c.ack_timeout_us, c.cca_db,
		    (double)link_mac_noise_floor());
	shell_print(sh, "difs %u, cw %u..%u, retries %u, turnaround %u us%s", c.difs, c.cw_min,
		    c.cw_max, c.retries, c.turn_us, c.turn_retune ? " with retune" : "");
	shell_print(sh, "tx: frames %u, sent %u, acked %u, dropped %u, %llu B acked",
		    s.tx_frames, s.tx_attempts, s.tx_acked, s.tx_dropped, s.tx_bytes_acked);
	shell_print(sh, "rx: triggers %u, none %u, hdr err %u, data err %u, ok %u, dup %u, "
		    "lost %u, other %u, acks sent %u rx %u",
		    s.rx_triggers, s.rx_none, s.rx_hdr_err, s.rx_data_err, s.rx_ok, s.rx_dup,
		    s.rx_lost, s.rx_other, s.acks_sent, s.acks_rx);
	shell_print(sh, "timing: build %llu us, decode %llu us avg %u max, tx on %llu off %llu us, "
		    "sense %llu us",
		    s.build_n ? s.build_us_sum / s.build_n : 0,
		    s.decode_n ? s.decode_us_sum / s.decode_n : 0, s.decode_us_max,
		    s.tx_switches ? s.tx_on_us_sum / s.tx_switches : 0,
		    s.tx_switches ? s.tx_off_us_sum / s.tx_switches : 0,
		    s.sense_n ? s.sense_us_sum / s.sense_n : 0);
	if (s.pair_n > 0) {
		shell_print(sh, "pairs %u: second frame (this CPU) %llu us, first payload (other CPU) %llu us, both %llu us", s.pair_n,
			    s.pair_a_us_sum / s.pair_n, s.pair_b_us_sum / s.pair_n,
			    s.pair_us_sum / s.pair_n);
	}
	{
		struct esp_sdr_stats d;

		esp_sdr_get_stats(&d);
		shell_print(sh, "engine: %u transmissions, rx gain re-forced %u times (%u us each)",
			    d.dac_sessions, d.gain_refreshed, d.gain_apply_us);
	}
	if (s.decode_n > 0 && c.phy == MAC_PHY_OFDM) {
		shell_print(sh, "decode stages avg us (OFDM): search %llu, channel+header %llu, "
			    "payload %llu, fec %llu; engine window %u us",
			    s.prof_us[0] / s.decode_n, s.prof_us[1] / s.decode_n,
			    s.prof_us[4] / s.decode_n, s.prof_us[5] / s.decode_n,
			    s.window_engine_us);
	} else if (s.decode_n > 0) {
		shell_print(sh, "decode stages avg us: search %llu, fine %llu, train %llu, header %llu, "
			    "payload %llu, fec %llu; engine sense %u window %u us",
			    s.prof_us[0] / s.decode_n, s.prof_us[1] / s.decode_n,
			    s.prof_us[2] / s.decode_n, s.prof_us[3] / s.decode_n,
			    s.prof_us[4] / s.decode_n, s.prof_us[5] / s.decode_n,
			    s.sense_engine_us, s.window_engine_us);
	}
	{
		struct app_crash cr;

		if (app_crash_last(&cr)) {
			shell_print(sh, "last crash: %s reason %u on cpu %u in %s at %u ms: pc 0x%08x "
				    "a0 0x%08x ps 0x%08x exccause %u excvaddr 0x%08x",
				    cr.reason == APP_CRASH_WATCHDOG ? "watchdog" : "fatal", cr.reason,
				    cr.cpu, cr.thread, cr.uptime_ms, cr.pc, cr.a0, cr.ps, cr.exccause,
				    cr.excvaddr);
		}
	}
	for (int m = 0; m < QAM_MODS; m++) {
		char line[128];
		int len = snprintf(line, sizeof(line), "%-7s", mod_names[m]);

		for (int f = 0; f < QAM_FECS && len < (int)sizeof(line); f++) {
			unsigned int n = qam_ncw_max((enum qam_mod)m, (enum qam_fec)f);

			len += snprintf(&line[len], sizeof(line) - len, "  %s %4u B (%u units %u B)",
					fec_names[f],
					(unsigned int)qam_full_payload_bytes((enum qam_mod)m,
									     (enum qam_fec)f),
					n, (unsigned int)qam_payload_bytes((enum qam_fec)f, n));
		}
		shell_print(sh, "%s", line);
	}
	return 0;
}

static int cmd_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	link_mac_clear_stats();
	return 0;
}

static int cmd_cal(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (link_mac_calibrate() != 0) {
		shell_error(sh, "calibration timed out");
		return -ETIMEDOUT;
	}
	shell_print(sh, "noise floor %.1f", (double)link_mac_noise_floor());
	return 0;
}

static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
	struct mac_cfg c;
	long v;
	char *end;

	if (argc == 1 || (argc - 1U) % 2U != 0U) {
		shell_print(sh, "usage: set <key> <value> [<key> <value> ...], applied together; keys:");
		shell_print(sh, "  phy (0 qam, 1 ofdm) freq txgain rxgain (auto|<index>) avg bw cbw txlpf "
				"txlpf2 amp air ackair acktimeout turn retune cca difs cwmin cwmax retries addr");
		return argc == 1 ? 0 : -EINVAL;
	}
	link_mac_get_cfg(&c);
	for (size_t a = 1; a + 1U < argc; a += 2U) {
		const char *key = argv[a], *val = argv[a + 1U];

		if (strcmp(key, "rxgain") == 0 && strcmp(val, "auto") == 0) {
			v = -1; /* the hardware AGC */
		} else {
			v = strtol(val, &end, 0);
			if (*val == '\0' || *end != '\0') {
				shell_error(sh, "bad value %s for %s", val, key);
				return -EINVAL;
			}
		}
		if (strcmp(key, "freq") == 0) {
			c.freq_mhz = (uint32_t)v;
		} else if (strcmp(key, "txgain") == 0) {
			c.tx_gain = (int)v;
		} else if (strcmp(key, "rxgain") == 0) {
			c.rx_gain = (int)v;
		} else if (strcmp(key, "txlpf") == 0) {
			c.tx_lpf_a = (int)v;
		} else if (strcmp(key, "txlpf2") == 0) {
			c.tx_lpf_b = (int)v;
		} else if (strcmp(key, "phy") == 0) {
			c.phy = (int)v;
		} else if (strcmp(key, "avg") == 0) {
			c.avg = (int)v;
		} else if (strcmp(key, "cbw") == 0) {
			c.cbw = (int)v;
		} else if (strcmp(key, "bw") == 0) {
			c.bw_mhz = (int)v;
		} else if (strcmp(key, "amp") == 0) {
			c.amp = (int)v;
		} else if (strcmp(key, "air") == 0) {
			c.data_air_us = (uint32_t)v;
		} else if (strcmp(key, "ackair") == 0) {
			c.ack_air_us = (uint32_t)v;
		} else if (strcmp(key, "acktimeout") == 0) {
			c.ack_timeout_us = (uint32_t)v;
		} else if (strcmp(key, "turn") == 0) {
			c.turn_us = (uint32_t)v;
		} else if (strcmp(key, "retune") == 0) {
			c.turn_retune = v != 0;
		} else if (strcmp(key, "cca") == 0) {
			c.cca_db = (int)v;
		} else if (strcmp(key, "difs") == 0) {
			c.difs = (uint8_t)v;
		} else if (strcmp(key, "cwmin") == 0) {
			c.cw_min = (uint16_t)v;
		} else if (strcmp(key, "cwmax") == 0) {
			c.cw_max = (uint16_t)v;
		} else if (strcmp(key, "retries") == 0) {
			c.retries = (uint8_t)v;
		} else if (strcmp(key, "addr") == 0 && v >= 0 && v < MAC_ADDR_BROADCAST) {
			c.addr = (uint8_t)v;
		} else {
			shell_error(sh, "unknown key %s", key);
			return -EINVAL;
		}
	}
	if (link_mac_set_cfg(&c) != 0) {
		shell_error(sh, "rejected");
		return -EINVAL;
	}
	return 0;
}

/* Internal RAM read speed: 4096 reads over 1024 words. */
static uint32_t bench_ram[1024];
static ESP_SDR_HIGH_RAM uint32_t bench_high[1024];

struct noise_decode {
	const struct esp_sdr_rx_burst *b;
	struct qam_rx_info info;
	uint32_t cycles;
	int ret;
};

/* A decode of noise on the paused MAC thread (link_mac_run_paused()), with the decoder's stack. */
static void noise_decode(void *arg)
{
	/* PSRAM: internal RAM is short with both dump banks reserved. */
	static EXT_RAM_BSS_ATTR struct qam_rx rx;
	static EXT_RAM_BSS_ATTR uint8_t cw[QAM_RX_CW_BYTES];
	struct noise_decode *job = arg;
	struct qam_hdr h;
	uint32_t t0;

	rx.cw = cw;
	t0 = k_cycle_get_32();
	job->ret = qam_rx_decode(&rx, job->b->words, job->b->count, &h, &job->info);
	job->cycles = k_cycle_get_32() - t0;
}

static int cmd_bench(const struct shell *sh, size_t argc, char **argv)
{
	struct esp_sdr_rx_burst b;
	uint32_t t0, t1, acc = 0;
	volatile float f = 1.0001f;
	float x = 1.0f;

	int ret;

	link_mac_pause(true);
	t0 = k_cycle_get_32();
	ret = esp_sdr_rx_capture(ESP_SDR_RATE_80MSPS, 1024, &b);
	t1 = k_cycle_get_32();
	shell_print(sh, "capture 1024: %u us (engine %u us, ret %d)", k_cyc_to_us_floor32(t1 - t0),
		    b.elapsed_us, ret);
	t0 = k_cycle_get_32();
	(void)esp_sdr_rx_capture(ESP_SDR_RATE_80MSPS, 16380, &b);
	t1 = k_cycle_get_32();
	shell_print(sh, "capture 16380: %u us (engine %u us)", k_cyc_to_us_floor32(t1 - t0),
		    b.elapsed_us);
	t0 = k_cycle_get_32();
	for (int i = 0; i < 4096; i++) {
		acc += b.words[i * 4];
	}
	t1 = k_cycle_get_32();
	shell_print(sh, "4096 bank reads: %u cycles (%u)", t1 - t0, acc);
	t0 = k_cycle_get_32();
	for (int i = 0; i < 4096; i++) {
		acc += ((volatile uint32_t *)bench_ram)[i & 1023];
	}
	t1 = k_cycle_get_32();
	shell_print(sh, "4096 ram reads: %u cycles", t1 - t0);
	t0 = k_cycle_get_32();
	for (int i = 0; i < 4096; i++) {
		acc += ((volatile uint32_t *)bench_high)[i & 1023];
	}
	t1 = k_cycle_get_32();
	shell_print(sh, "4096 high ram reads: %u cycles", t1 - t0);
	{
		uint32_t key = arch_irq_lock();

		t0 = k_cycle_get_32();
		for (int i = 0; i < 4096; i++) {
			acc += ((volatile uint32_t *)bench_ram)[i & 1023];
		}
		t1 = k_cycle_get_32();
		arch_irq_unlock(key);
		shell_print(sh, "4096 ram reads, irqs locked: %u cycles", t1 - t0);
	}
	t0 = k_cycle_get_32();
	for (int i = 0; i < 4096; i++) {
		x = x * f + 0.5f;
	}
	t1 = k_cycle_get_32();
	shell_print(sh, "4096 float madd: %u cycles (%d)", t1 - t0, (int)x);
	t0 = k_cycle_get_32();
	x = qam_rx_power(b.words, 1024);
	t1 = k_cycle_get_32();
	shell_print(sh, "power 1024: %u cycles (%d)", t1 - t0, (int)x);
	t0 = k_cycle_get_32();
	for (int i = 0; i < 4096; i++) {
		acc += b.words[i * 4];
	}
	t1 = k_cycle_get_32();
	shell_print(sh, "4096 bank reads again: %u cycles (%u)", t1 - t0, acc);
	{
		struct noise_decode job = {.b = &b};

		(void)esp_sdr_rx_capture(ESP_SDR_RATE_80MSPS, 16380, &b);
		ret = link_mac_run_paused(noise_decode, &job);
		if (ret == 0) {
			ret = job.ret;
		}
		shell_print(sh, "decode noise: %u us, ret %d, stages %u %u cycles",
			    k_cyc_to_us_floor32(job.cycles), ret, job.info.prof[0], job.info.prof[1]);
	}
	shell_print(sh, "cycle clock %u Hz, cpu %d", sys_clock_hw_cycles_per_sec(),
		    arch_curr_cpu()->id);
	if (argc > 1) {
		/* bench <rate 0|1> <us>: one loop session on bank 1 at 80 or 40 MS/s. */
		enum esp_sdr_rate rate = argv[1][0] == '1' ? ESP_SDR_RATE_40MSPS : ESP_SDR_RATE_80MSPS;
		uint32_t us = argc > 2 ? (uint32_t)atoi(argv[2]) : 700U;
		size_t words;
		uint32_t *bank = esp_sdr_tx_loop_buf(1, &words);

		for (size_t i = 0; i < 4096; i++) {
			bank[i] = 0;
		}
		t0 = k_cycle_get_32();
		ret = esp_sdr_tx_loop_begin(rate);
		t1 = k_cycle_get_32();
		shell_print(sh, "tx begin %d: %u us", ret, k_cyc_to_us_floor32(t1 - t0));
		if (ret == 0) {
			(void)esp_sdr_tx_loop_start(1, 4096);
			k_busy_wait(us);
			t0 = k_cycle_get_32();
			esp_sdr_tx_loop_end();
			t1 = k_cycle_get_32();
			shell_print(sh, "tx end: %u us", k_cyc_to_us_floor32(t1 - t0));
		}
	}
	link_mac_pause(false);
	return 0;
}

static int cmd_decode(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	link_mac_pause(true);
	for (int m = 0; m < QAM_MODS; m++) {
		struct qam_rx_info info = {0};
		uint32_t b = 0, d = 0;
		int ret = link_mac_bench_decode((enum qam_mod)m, &info, &b, &d);

		if (ret == -ENODEV || ret == -ETIMEDOUT) {
			shell_error(sh, "the MAC thread runs the decoder and is not running (%d)", ret);
			break;
		}
		shell_print(sh, "%-7s build %5u us, decode %5u us (ret %d, MER %.1f dB): search %u "
			    "fine %u train %u header %u payload %u fec %u us",
			    mod_names[m], b, d, ret, (double)info.mer_db,
			    k_cyc_to_us_floor32(info.prof[0]), k_cyc_to_us_floor32(info.prof[1]),
			    k_cyc_to_us_floor32(info.prof[2]), k_cyc_to_us_floor32(info.prof[3]),
			    k_cyc_to_us_floor32(info.prof[4]), k_cyc_to_us_floor32(info.prof[5]));
	}
	link_mac_pause(false);
	return 0;
}

static int cmd_mem(const struct shell *sh, size_t argc, char **argv)
{
	struct sys_memory_stats st;
	extern struct sys_heap _system_heap;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (sys_heap_runtime_stats_get(&_system_heap, &st) == 0) {
		shell_print(sh, "system heap: %u used, %u free, %u peak", (unsigned int)st.allocated_bytes,
			    (unsigned int)st.free_bytes, (unsigned int)st.max_allocated_bytes);
	}
	return 0;
}

static int cmd_reg(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 3) {
		esp_sdr_bbtop_write((unsigned int)strtoul(argv[1], NULL, 0),
				    (unsigned int)strtoul(argv[2], NULL, 0));
		return 0;
	}
	for (unsigned int r = 0; r < 0x39; r += 8) {
		char line[64];
		int len = snprintf(line, sizeof(line), "%02x:", r);

		for (unsigned int k = r; k < MIN(r + 8U, 0x39U); k++) {
			len += snprintf(&line[len], sizeof(line) - len, " %02x", esp_sdr_bbtop_read(k));
		}
		shell_print(sh, "%s", line);
	}
	return 0;
}

static int cmd_dump(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t ms = 2000;
	const uint32_t *w;
	size_t n;
	bool decoded = false;

	for (size_t i = 1; i < argc; i++) {
		if (strcmp(argv[i], "ok") == 0) {
			decoded = true;
		} else if (parse_u32(argv[i], &ms) != 0) {
			return -EINVAL;
		}
	}
	if (link_mac_dump_take(&w, &n, decoded, K_MSEC(ms)) != 0) {
		shell_error(sh, "nothing above the carrier sense threshold");
		return -EAGAIN;
	}
	shell_print(sh, "dump %u words", (unsigned int)n);
	for (size_t i = 0; i < n; i += 8) {
		shell_print(sh, "%08x %08x %08x %08x %08x %08x %08x %08x", w[i], w[i + 1], w[i + 2],
			    w[i + 3], w[i + 4], w[i + 5], w[i + 6], i + 7 < n ? w[i + 7] : 0U);
	}
	shell_print(sh, "dump end");
	link_mac_dump_release();
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(link_cmds,
	SHELL_CMD(server, NULL, "Count received test frames: [-i <s>] | stop", cmd_server),
	SHELL_CMD_ARG(client, NULL,
		      "Send test frames: <addr> [-t <s>] [-m qpsk|16qam|...|4096qam] [-f rs|hamming|none] [-n <units>] [-w 1|2] "
		      "[-b <kbit/s>] [-i <s>]",
		      cmd_client, 2, 14),
	SHELL_CMD(stop, NULL, "Stop the client and the server report", cmd_stop),
	SHELL_CMD(status, NULL, "Settings, counters, timing", cmd_status),
	SHELL_CMD(cal, NULL, "Measure the noise floor again", cmd_cal),
	SHELL_CMD(clear, NULL, "Zero the counters", cmd_clear),
	SHELL_CMD_ARG(set, NULL, "[<key> <value> ...]: settings, applied together (no arguments: the keys)",
		      cmd_set, 1, 16),
	SHELL_CMD(mem, NULL, "System heap use", cmd_mem),
	SHELL_CMD_ARG(reg, NULL, "Analog baseband registers (I2C 0x67): dump | <reg> <value>",
		      cmd_reg, 1, 2),
	SHELL_CMD(decode, NULL, "Build and decode one frame per modulation, no radio", cmd_decode),
	SHELL_CMD_ARG(bench, NULL, "Time captures and inner loops [<0: 80 MS/s | 1: 40> <us>]",
		      cmd_bench, 1, 2),
	SHELL_CMD_ARG(dump, NULL, "Hex dump of the next busy capture: [ok] [<timeout ms>]",
		      cmd_dump, 1, 2),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(link, &link_cmds, "ESP-SDR QAM link", NULL);

static int perf_init(void)
{
	k_thread_create(&report_thread, report_stack, K_THREAD_STACK_SIZEOF(report_stack),
			report_loop, NULL, NULL, NULL, REPORT_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&report_thread, "link_report");
	return 0;
}

SYS_INIT(perf_init, APPLICATION, 0);
