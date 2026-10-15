/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Raw OFDM test on the radio, outside the MAC (paused meanwhile): one board
 * loops a frame of known test bits on the DAC, the other decodes captures of
 * it and counts bit errors. Both must be set alike ("ofdm set").
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>

#include <esp_attr.h>

#include "mac.h"
#include "ofdm.h"
#include "rs.h"

/* Transmit at the layout's sample rate, receive at that divided by rx_div. */
#define TX_RATE   (cfg.sample_rate_hz == 40000000U ? ESP_SDR_RATE_40MSPS : ESP_SDR_RATE_80MSPS)
#define RX_RATE   (cfg.sample_rate_hz / cfg.rx_div == 40000000U ? ESP_SDR_RATE_40MSPS \
							       : ESP_SDR_RATE_80MSPS)
#define WINDOW    ESP_SDR_SAMPLES_MAX
#define TX_BANK   1
#define BITS_SEED 1U
#define BITS_MAX  (WINDOW / 2U)
#define HDR_BITS  64U

/* FFT tables and buffers above the capture bank: internal SRAM, outside the libc heap. */
static ESP_SDR_HIGH_RAM struct ofdm_ctx ctx __aligned(16);
/* Transmit builds are not timed: PSRAM will do. */
static EXT_RAM_BSS_ATTR struct ofdm_txbuf txb __aligned(16);
static EXT_RAM_BSS_ATTR uint8_t ref_bits[BITS_MAX], rx_bits[BITS_MAX];
static uint8_t ref_hdr[HDR_BITS / 8U], rx_hdr[HDR_BITS / 8U];
static enum ofdm_mod mod = OFDM_QPSK;

static struct ofdm_cfg cfg = {
	.sample_rate_hz = CONFIG_APP_OFDM_FS_MHZ * 1000000U,
	.bandwidth_hz = CONFIG_APP_OFDM_BW_KHZ * 1000U,
	.channels = CONFIG_APP_OFDM_CHANNELS,
	.cp_div = 8,
	.amp = 90,
	.symbols = 0,
	/* Set by layout_limit(): a whole copy must fit the receive window. */
	.max_samples = 0,
	.rx_div = CONFIG_APP_OFDM_RX_DIV,
	.pilots = 2,
	.smooth = true,
	.hdr_bits = HDR_BITS,
	.mid_pilot = true,
};
static bool ready;

static const char *const mod_names[OFDM_MODS] = {"bpsk", "qpsk", "16qam", "64qam"};

static uint32_t cyc(void)
{
	return k_cycle_get_32();
}

/*
 * Longest frame (transmit samples): the DAC bank, and two copies plus a prefix
 * in the receive window. At half the receive rate that is the whole bank.
 */
static void layout_limit(struct ofdm_cfg *c)
{
	c->max_samples = c->rx_div == 2U ? WINDOW - 2U * OFDM_NFFT_MAX / 4U : (WINDOW + 1U) / 2U;
}

/* 80 or 40 MS/s; the receiver below 40 MS/s has no rate. */
static bool rates_ok(const struct ofdm_cfg *c)
{
	return (c->sample_rate_hz == 80000000U || c->sample_rate_hz == 40000000U) &&
	       c->sample_rate_hz / c->rx_div >= 40000000U;
}

enum esp_sdr_rate link_ofdm_tx_rate(void)
{
	return TX_RATE;
}

enum esp_sdr_rate link_ofdm_rx_rate(void)
{
	return RX_RATE;
}

static int prepare(const struct shell *sh)
{
	if (!ready) {
		layout_limit(&cfg);
		if (ofdm_init(&ctx, &cfg) != 0) {
			if (sh != NULL) {
				shell_error(sh, "no frame layout for these settings");
			}
			return -EINVAL;
		}
		if (ofdm_frame_bits(&ctx, OFDM_64QAM) > BITS_MAX * 8U) {
			if (sh != NULL) {
				shell_error(sh, "frame too long");
			}
			return -EINVAL;
		}
		/* A prefix of the same sequence for every modulation. */
		ofdm_test_bits(ref_bits, BITS_MAX * 8U, BITS_SEED);
		ofdm_test_bits(ref_hdr, HDR_BITS, BITS_SEED + 1U);
		ofdm_clock = cyc;
		ready = true;
	}
	return 0;
}

static void show(const struct shell *sh)
{
	size_t bits = ofdm_frame_bits(&ctx, mod);
	float us = (float)ctx.len * 1e6f / (float)cfg.sample_rate_hz;

	shell_print(sh, "%s, %u subcarriers, %.3f MHz (asked %.3f), spacing %.1f kHz, fft %u, "
		    "cp %u, %u pilots%s, %u header + %u data symbols", mod_names[mod],
		    cfg.channels, (double)(ctx.bw_hz / 1e6f), (double)(cfg.bandwidth_hz / 1e6f),
		    (double)(ctx.spacing_hz / 1e3f), ctx.nfft, ctx.cp, cfg.pilots,
		    cfg.smooth ? " smoothed" : "", ctx.nhdr, ctx.ndata);
	shell_print(sh, "frame %u samples at %u MS/s (%.1f us), %u bits: %.2f Mbit/s on air, "
		    "tx rms %d, rx at %u MS/s (fft %u)%s",
		    (unsigned int)ctx.len, cfg.sample_rate_hz / 1000000U, (double)us,
		    (unsigned int)bits, (double)(bits / us),
		    cfg.amp, cfg.sample_rate_hz / cfg.rx_div / 1000000U, ctx.rnfft,
		    IS_ENABLED(CONFIG_APP_OFDM_ESP_DSP) ? ", esp-dsp fft" : ", C fft");
}

struct ofdm_ctx *link_ofdm(void)
{
	(void)prepare(NULL);
	return &ctx;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (prepare(sh) != 0) {
		return -EINVAL;
	}
	show(sh);
	return 0;
}

/* One key into @p c (or @p m); false if unknown. */
static bool set_key(struct ofdm_cfg *c, enum ofdm_mod *m, const char *k, const char *v)
{
	if (strcmp(k, "bw") == 0) {
		c->bandwidth_hz = (uint32_t)(strtof(v, NULL) * 1e6f);
	} else if (strcmp(k, "ch") == 0) {
		c->channels = (unsigned int)strtoul(v, NULL, 0);
	} else if (strcmp(k, "cp") == 0) {
		c->cp_div = (unsigned int)strtoul(v, NULL, 0);
	} else if (strcmp(k, "amp") == 0) {
		c->amp = (int)strtol(v, NULL, 0);
	} else if (strcmp(k, "fs") == 0) {
		c->sample_rate_hz = (uint32_t)strtoul(v, NULL, 0) * 1000000U;
	} else if (strcmp(k, "rxdiv") == 0) {
		c->rx_div = (unsigned int)strtoul(v, NULL, 0);
	} else if (strcmp(k, "syms") == 0) {
		c->symbols = (unsigned int)strtoul(v, NULL, 0);
	} else if (strcmp(k, "pilots") == 0) {
		c->pilots = (unsigned int)strtoul(v, NULL, 0);
	} else if (strcmp(k, "smooth") == 0) {
		c->smooth = strtoul(v, NULL, 0) != 0U;
	} else if (strcmp(k, "mod") == 0) {
		*m = OFDM_MODS;
		for (int i = 0; i < OFDM_MODS; i++) {
			if (strcmp(v, mod_names[i]) == 0) {
				*m = (enum ofdm_mod)i;
			}
		}
	} else {
		return false;
	}
	return true;
}

/* Several pairs at once: the layout is checked on the whole combination. */
static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
	struct ofdm_cfg c = cfg;
	enum ofdm_mod m = mod;

	for (size_t i = 1; i + 1 < argc; i += 2) {
		if (!set_key(&c, &m, argv[i], argv[i + 1])) {
			argc = 0;
			break;
		}
	}
	if (argc % 2U == 0U) {
		shell_error(sh, "<key> <value> ...; keys: bw <MHz> ch <n> cp <fft/cp> "
			    "mod bpsk|qpsk|16qam|64qam amp <rms> syms <n, 0: fill> "
			    "rxdiv 1|2 fs 80|40 pilots 1..4 smooth 0|1");
		return -EINVAL;
	}
	layout_limit(&c);
	/* The MAC builds and decodes with this context: hold it off meanwhile. */
	link_mac_pause(true);
	if (m >= OFDM_MODS || !rates_ok(&c) || ofdm_init(&ctx, &c) != 0 || c.amp < 1 ||
	    c.amp > 400) {
		ready = false;
		(void)prepare(sh);
		link_mac_pause(false);
		shell_error(sh, "no frame layout for that, unchanged");
		return -EINVAL;
	}
	cfg = c;
	mod = m;
	ready = false;
	(void)prepare(sh);
	link_mac_pause(false);
	show(sh);
	return 0;
}

static int cmd_tx(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t ms = 10000, t0;
	size_t words;
	uint32_t *bank;
	int ret;

	for (size_t i = 1; i + 1 < argc; i += 2) {
		if (strcmp(argv[i], "-t") == 0) {
			ms = (uint32_t)(strtof(argv[i + 1], NULL) * 1000.0f);
		}
	}
	if (prepare(sh) != 0) {
		return -EINVAL;
	}
	link_mac_pause(true);
	bank = esp_sdr_tx_loop_buf(TX_BANK, &words);
	t0 = cyc();
	(void)ofdm_tx_build(&ctx, &txb, mod, ref_hdr, ref_bits, bank);
	t0 = k_cyc_to_us_floor32(cyc() - t0);
	ret = esp_sdr_tx_loop_begin(TX_RATE);
	if (ret == 0) {
		ret = esp_sdr_tx_loop_start(TX_BANK, ctx.len);
		if (ret == 0) {
			shell_print(sh, "looping %u samples for %u ms (built in %u us, %u clipped)",
				    (unsigned int)ctx.len, ms, t0, txb.clipped);
			k_msleep(ms);
		}
		esp_sdr_tx_loop_end();
	}
	link_mac_pause(false);
	if (ret != 0) {
		shell_error(sh, "transmit failed (%d)", ret);
	}
	return ret;
}

/* Channel magnitude per subcarrier in dB against the mean, from the last pilot. */
static void print_channel(const struct shell *sh)
{
	float mean = 0.0f;
	char line[100];
	int len = 0;

	for (unsigned int a = 0; a < cfg.channels; a++) {
		struct ofdm_cf e = ctx.eq[a];

		mean += e.re * e.re + e.im * e.im;
	}
	mean /= (float)cfg.channels;
	shell_print(sh, "channel |H| dB vs mean, subcarriers from lowest frequency:");
	for (unsigned int a = 0; a < cfg.channels; a++) {
		struct ofdm_cf e = ctx.eq[a];
		float g = e.re * e.re + e.im * e.im;
		int db = g > 0.0f ? (int)lrintf(10.0f * log10f(mean / g)) : -99;

		len += snprintf(&line[len], sizeof(line) - len, "%4d", db);
		if (len > (int)sizeof(line) - 6 || a + 1 == cfg.channels) {
			shell_print(sh, "%s", line);
			len = 0;
		}
	}
}

static int cmd_rx(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t n = 100, found = 0, bad = 0, hbad = 0;
	uint64_t errs = 0, prof[3] = {0}, cap_us = 0;
	float mer = 0.0f, cfo = 0.0f, mer_min = 99.0f;
	bool verbose = false, full = false, dump = false;
	size_t bits;

	for (size_t i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
			n = (uint32_t)strtoul(argv[++i], NULL, 0);
		} else if (strcmp(argv[i], "-v") == 0) {
			verbose = true;
		} else if (strcmp(argv[i], "-w") == 0) {
			full = true;
		} else if (strcmp(argv[i], "-d") == 0) {
			dump = true;
		}
	}
	if (prepare(sh) != 0) {
		return -EINVAL;
	}
	bits = ofdm_frame_bits(&ctx, mod);
	link_mac_pause(true);
	for (uint32_t k = 0; k < n; k++) {
		struct esp_sdr_rx_burst b;
		struct ofdm_rx_info info;
		uint32_t t0 = cyc(), e = 0;

		/* Two frames hold a whole copy wherever the capture starts. */
		if (esp_sdr_rx_capture(RX_RATE, full ? WINDOW : MIN(WINDOW, 2U * ctx.rlen + ctx.rnfft),
				       &b) != 0) {
			continue;
		}
		cap_us += k_cyc_to_us_floor32(cyc() - t0);
		if (ofdm_rx_begin(&ctx, b.words, b.count, rx_hdr, &info) != 0) {
			continue;
		}
		found++;
		hbad += memcmp(rx_hdr, ref_hdr, sizeof(rx_hdr)) != 0;
		ofdm_rx_finish(&ctx, b.words, mod, rx_bits, &info);
		for (size_t i = 0; i < (bits + 7U) / 8U; i++) {
			uint8_t d = rx_bits[i] ^ ref_bits[i];

			if (i == (bits - 1U) / 8U && (bits & 7U) != 0U) {
				d &= (uint8_t)(0xffU << (8U - (bits & 7U)));
			}
			e += (uint32_t)__builtin_popcount(d);
		}
		errs += e;
		bad += e != 0U;
		if (dump && e != 0U) {
			/* The capture of this frame, as "link dump" prints it, then stop. */
			shell_print(sh, "frame with %u bit errors, start %d, MER %.1f dB", e, info.start,
				    (double)info.mer_db);
			shell_print(sh, "dump %u words", (unsigned int)b.count);
			for (size_t i = 0; i < b.count; i += 8) {
				char line[96];
				int len = 0;

				for (size_t j = i; j < MIN(i + 8U, b.count); j++) {
					len += snprintf(&line[len], sizeof(line) - len, "%08x ",
							b.words[j]);
				}
				shell_print(sh, "%s", line);
			}
			shell_print(sh, "dump end");
			n = k + 1U;
			break;
		}
		mer += info.mer_db;
		mer_min = MIN(mer_min, info.mer_db);
		cfo += info.cfo_hz;
		for (int i = 0; i < 3; i++) {
			prof[i] += k_cyc_to_us_floor32(info.prof[i]);
		}
	}
	link_mac_pause(false);
	shell_print(sh, "found %u/%u, header errors %u, frame errors %u, bit errors %llu/%llu "
		    "(BER %.2e)", found, n, hbad, bad, errs, (uint64_t)found * bits,
		    found ? (double)errs / ((double)found * (double)bits) : 0.0);
	if (found > 0) {
		shell_print(sh, "MER %.1f dB avg, %.1f min, cfo %.1f kHz; us per frame: search %llu, "
			    "channel+header %llu, data %llu (capture %llu)",
			    (double)(mer / found), (double)mer_min, (double)(cfo / found / 1e3f),
			    prof[0] / found, prof[1] / found, prof[2] / found, cap_us / n);
		if (verbose) {
			print_channel(sh);
		}
	}
	shell_print(sh, "rx done");
	return 0;
}

static int cmd_bench(const struct shell *sh, size_t argc, char **argv)
{
	const unsigned int rounds = 1000;
	uint32_t t0;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (prepare(sh) != 0) {
		return -EINVAL;
	}
	for (unsigned int i = 0; i < ctx.nfft; i++) {
		ctx.w0.buf[i] = (struct ofdm_cf){(float)(i % 7) - 3.0f, (float)(i % 5) - 2.0f};
	}
	t0 = cyc();
	for (unsigned int r = 0; r < rounds; r++) {
		ofdm_test_fft(&ctx);
	}
	t0 = cyc() - t0;
	shell_print(sh, "fft %u (%s): %.2f us", ctx.nfft,
		    IS_ENABLED(CONFIG_APP_OFDM_ESP_DSP) ? "esp-dsp" : "C",
		    (double)k_cyc_to_ns_floor64(t0) / 1e3 / rounds);
	/* RS(255,223) decode with e byte errors, on the stack. */
	for (unsigned int e = 0; e <= 10; e += (e == 0 ? 2 : (e == 2 ? 3 : 5))) {
		uint8_t ref[RS_N], cw[RS_N];
		uint32_t sum = 0;
		int r = 0;

		for (unsigned int i = 0; i < RS_K; i++) {
			ref[i] = (uint8_t)(i * 37U + 11U);
		}
		rs_encode(ref);
		for (unsigned int k = 0; k < 50; k++) {
			memcpy(cw, ref, RS_N);
			for (unsigned int j = 0; j < e; j++) {
				cw[(k * 31U + j * 23U) % RS_N] ^= (uint8_t)(j + 1U);
			}
			t0 = cyc();
			r = rs_decode(cw);
			sum += cyc() - t0;
		}
		shell_print(sh, "rs_decode, %u errors: %u us (returned %d)", e,
			    k_cyc_to_us_floor32(sum / 50U), r);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(ofdm_cmds,
	SHELL_CMD(status, NULL, "Frame layout and settings", cmd_status),
	SHELL_CMD_ARG(set, NULL, "<bw|ch|cp|mod|amp|syms|rxdiv|fs|pilots|smooth> <value> ...", cmd_set,
		      3, 16),
	SHELL_CMD_ARG(tx, NULL, "Loop the test frame: [-t <s>]", cmd_tx, 1, 2),
	SHELL_CMD_ARG(rx, NULL, "Decode captures: [-n <captures>] [-v] [-w: whole window] "
		      "[-d: dump the first frame with errors]", cmd_rx, 1, 5),
	SHELL_CMD(bench, NULL, "Time one FFT", cmd_bench),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(ofdm, &ofdm_cmds, "Raw OFDM test (MAC paused)", NULL);
