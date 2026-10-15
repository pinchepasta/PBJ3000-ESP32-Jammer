/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * "sdr" shell commands: settings and counters.
 */

#include <stdlib.h>

#include <zephyr/shell/shell.h>

#include <esp_sdr/esp_sdr.h>

#include "sdr.h"

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	struct sdr_settings s;
	struct esdr_stats st;
	struct esp_sdr_stats d;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	sdr_get_settings(&s);
	sdr_get_stats(&st);
	esp_sdr_get_stats(&d);
	shell_print(sh, "mode %u, %u bit, %u Hz at %llu Hz, gain %u, digital %u dB, bw %u Hz, "
		    "corr %d ppb, opt 0x%x",
		    s.mode, s.bits, s.rate_hz, (unsigned long long)s.freq_hz, s.rx_gain,
		    sdr_dgain_db(&s), s.bandwidth_hz, s.corr_ppb, s.options);
	shell_print(sh, "rx: %llu samples, %u blocks, %llu overflowed (%u times), %llu restart gap",
		    (unsigned long long)st.rx_samples, st.rx_blocks,
		    (unsigned long long)st.rx_overflow_samples, st.rx_overflows,
		    (unsigned long long)st.rx_restart_samples);
	shell_print(sh, "ring: %u runs, %u errors (last %u/0x%08x), %u abandoned, %llu pairs lost, "
		    "late max %u, %u.%02u cycles/pair, gain re-forced %u",
		    st.ring_runs, st.ring_errors, st.ring_last_status, st.ring_last_detail,
		    st.ring_abandoned, (unsigned long long)st.rx_ring_lost_pairs, st.ring_late_max,
		    st.ring_cycles_x100 / 100U, st.ring_cycles_x100 % 100U, st.ring_gain_refreshed);
	shell_print(sh, "usb: %llu bytes in, %llu bytes out", (unsigned long long)st.usb_in_bytes,
		    (unsigned long long)st.usb_out_bytes);
	return 0;
}

static int cmd_bench(const struct shell *sh, size_t argc, char **argv)
{
	unsigned int decim = argc > 1 ? strtoul(argv[1], NULL, 0) : 64U;
	struct esp_sdr_ring_bench_result r;

	for (int v = 0; v < 3; v++) {
		int ret = sdr_bench(decim, v == 1, v == 2, &r);

		if (ret != 0) {
			shell_error(sh, "bench: %d", ret);
			return ret;
		}
		shell_print(sh, "%-10s %u.%02u cycles/pair: unpack %u.%02u, stage 1 %u.%02u, "
			    "stage 2 %u.%02u, sink %u.%02u (%u samples)",
			    v == 0 ? "null sink" : v == 1 ? "real sink" : "fs/4 mix",
			    r.cycles_x100 / 100U, r.cycles_x100 % 100U, r.stage_x100[0] / 100U,
			    r.stage_x100[0] % 100U, r.stage_x100[1] / 100U, r.stage_x100[1] % 100U,
			    r.stage_x100[2] / 100U, r.stage_x100[2] % 100U, r.stage_x100[3] / 100U,
			    r.stage_x100[3] % 100U, r.samples);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sdr_cmds,
	SHELL_CMD_ARG(bench, NULL, "[decimation]: filter cycles per input pair", cmd_bench, 1, 1),
	SHELL_CMD(status, NULL, "Settings and counters", cmd_status),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(sdr, &sdr_cmds, "USB SDR", NULL);
