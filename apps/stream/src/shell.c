/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The "sdr" shell command: receiver and transmitter controls and tests. The
 * receive stream picks changes up per burst.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_heap.h>

#include <esp_attr.h>

#include <esp_sdr/esp_sdr.h>

#include "vrt_rx.h"

#if defined(CONFIG_APP_TX)
#include <esp_sdr/esp_sdr_tx.h>

#include "vrt_tx.h"
#endif

extern struct k_heap _system_heap;

static int cmd_mem(const struct shell *sh, size_t argc, char **argv)
{
	struct sys_memory_stats st;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (sys_heap_runtime_stats_get(&_system_heap.heap, &st) != 0) {
		shell_error(sh, "no heap stats");
		return -EIO;
	}
	shell_print(sh, "system heap: %u allocated, %u free, %u peak",
		    (unsigned int)st.allocated_bytes, (unsigned int)st.free_bytes,
		    (unsigned int)st.max_allocated_bytes);
	return 0;
}

#if defined(CONFIG_APP_TX)


/*
 * Transmit synthesis. The DAC runs at its own rate (40 or 80 MS/s, never the
 * 16 MS/s the receiver uses), so every generator works in Hz and seconds and
 * converts with esp_sdr_tx_rate_hz(). One buffer serves all of them.
 */
#define TX_RATE        ESP_SDR_RATE_40MSPS
#define TX_BUF_SAMPLES 2048U /* 51 us at 40 MS/s; RAM below the capture bank is tight */
#define FM_TWO_PI      6.28318530717958647692
#define FM_TWO_PI_F    6.28318530717958647692f
/* PSRAM: only CPU copies read it, the DAC plays from the bank. */
static EXT_RAM_BSS_ATTR uint32_t tx_buf[TX_BUF_SAMPLES];

static enum esp_sdr_rate tx_rate_arg(size_t argc, char **argv, size_t idx)
{
	return argc > idx ? (enum esp_sdr_rate)strtol(argv[idx], NULL, 0) : TX_RATE;
}

static int cmd_fm(const struct shell *sh, size_t argc, char **argv)
{
	double fm_hz = strtod(argv[1], NULL);
	double dev_hz = strtod(argv[2], NULL);
	long amp = argc > 3 ? strtol(argv[3], NULL, 0) : 400;
	double fs = (double)esp_sdr_tx_rate_hz(TX_RATE);
	double phase = 0.0;
	int ret;

	if (amp <= 0 || amp > 511) {
		shell_error(sh, "amplitude must be 1..511");
		return -EINVAL;
	}

	/* Instantaneous offset dev_hz * sin(2 pi fm_hz t), integrated to phase. */
	for (size_t n = 0; n < TX_BUF_SAMPLES; n++) {
		double inst_hz = dev_hz * sin(FM_TWO_PI * fm_hz * (double)n / fs);

		phase += FM_TWO_PI * inst_hz / fs;
		tx_buf[n] = esp_sdr_tx_word((int16_t)lround((double)amp * cos(phase)),
					    (int16_t)lround((double)amp * sin(phase)));
	}

	ret = esp_sdr_tx_play(TX_RATE, tx_buf, TX_BUF_SAMPLES);
	if (ret != 0) {
		shell_error(sh, "play: %d", ret);
		return ret;
	}
	shell_print(sh, "played %u samples at %u MS/s, %u MHz, fm %.1f Hz dev %.1f Hz amp %ld",
		    TX_BUF_SAMPLES, (unsigned int)(fs / 1e6), esp_sdr_get_freq(), fm_hz, dev_hz,
		    amp);
	return 0;
}

/*
 * Period length for a seamless tone: the n in [MIN, TX_BUF_SAMPLES] whose
 * whole number of cycles comes closest to the requested frequency.
 */
static size_t tone_period(double req, double fs, long *cycles)
{
	size_t best_n = TX_BUF_SAMPLES;
	double best_err = INFINITY;

	for (size_t n = TX_BUF_SAMPLES; n >= ESP_SDR_SAMPLES_MIN; n--) {
		long c = lround(req * (double)n / fs);
		double err = fabs((double)c * fs / (double)n - req);

		if (err < best_err) {
			best_err = err;
			best_n = n;
			*cycles = c;
			if (err < 1.0) {
				break;
			}
		}
	}
	return best_n;
}

/*
 * Quick link check: a steady tone (no modulation) and a one-shot receiver
 * that reports average power and average frequency over a whole burst, with
 * no search loop. Use this before fsktx/fskrx to confirm TX actually
 * reaches the other board at a sane level and offset.
 */
static int cmd_tonetx(const struct shell *sh, size_t argc, char **argv)
{
	double req = argc > 1 ? strtod(argv[1], NULL) : 1000000.0;
	uint32_t ms = argc > 2 ? (uint32_t)strtoul(argv[2], NULL, 0) : 0U;
	long amp = argc > 3 ? strtol(argv[3], NULL, 0) : 511;
	enum esp_sdr_rate rate = tx_rate_arg(argc, argv, 4);
	double fs = (double)esp_sdr_tx_rate_hz(rate);
	uint32_t bursts = 0;
	long cycles = 0;
	size_t n;
	int ret;

	if (fs == 0.0) {
		shell_error(sh, "transmit rate index must be 0 (80 MS/s) or 1 (40 MS/s)");
		return -EINVAL;
	}
	if (amp <= 0 || amp > 511 || fabs(req) >= fs / 2.0) {
		shell_error(sh, "amplitude must be 1..511 and |hz| below %.0f", fs / 2.0);
		return -EINVAL;
	}
	n = tone_period(req, fs, &cycles);
	for (size_t k = 0; k < n; k++) {
		double phase = FM_TWO_PI * (double)cycles * (double)k / (double)n;

		tx_buf[k] = esp_sdr_tx_word((int16_t)lround((double)amp * cos(phase)),
					    (int16_t)lround((double)amp * sin(phase)));
	}
	ret = esp_sdr_tx_play_for(rate, tx_buf, n, ms, &bursts);
	if (ret != 0) {
		shell_error(sh, "play: %d", ret);
		return ret;
	}
	shell_print(sh,
		    "tonetx: %u MHz %+.0f Hz, %u MS/s, period %u, amp %ld, txgain %d, %u bursts in %u ms",
		    esp_sdr_get_freq(), (double)cycles * fs / (double)n, (unsigned int)(fs / 1e6),
		    (unsigned int)n, amp, esp_sdr_tx_get_gain(), bursts, ms);
	return 0;
}

static int cmd_sweep(const struct shell *sh, size_t argc, char **argv)
{
	float f0 = strtof(argv[1], NULL), f1 = strtof(argv[2], NULL);
	uint32_t ms = (uint32_t)strtoul(argv[3], NULL, 0);
	int amp = argc > 4 ? (int)strtol(argv[4], NULL, 0) : 256;
	enum esp_sdr_rate rate = tx_rate_arg(argc, argv, 5);
	struct esp_sdr_tx_sweep_stats st;
	int ret = esp_sdr_tx_sweep(rate, f0, f1, ms, amp, &st);
	double fs = (double)esp_sdr_tx_rate_hz(rate);

	if (ret != 0) {
		shell_error(sh, "sweep: %d (|hz| below rate/2, ms 1..%u, amp 1..511)", ret,
			    ESP_SDR_TX_PLAY_MAX_MS);
		return ret;
	}
	shell_print(sh,
		    "sweep: %+.0f -> %+.0f Hz in %u ms at %u MS/s, txgain %d, amp %d: %u segments, "
		    "%u samples, duty %.0f %%",
		    (double)f0, (double)f1, st.elapsed_ms, (unsigned int)(fs / 1e6),
		    esp_sdr_tx_get_gain(), amp, st.segments, st.samples,
		    100.0 * st.samples / fs / (st.elapsed_ms / 1e3));
	return 0;
}

static int cmd_tonerx(const struct shell *sh, size_t argc, char **argv)
{
	struct esp_sdr_rx_burst b;
	float fs = (float)esp_sdr_rx_rate_hz(ESP_SDR_RATE_16MSPS);
	float sum_sq = 0.0f, sum_delta = 0.0f;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = esp_sdr_rx_capture(ESP_SDR_RATE_16MSPS, ESP_SDR_SAMPLES_MAX, &b);
	if (ret != 0) {
		shell_error(sh, "capture: %d", ret);
		return ret;
	}
	for (size_t n = 0; n < b.count; n++) {
		float i = (float)esp_sdr_rx_i(b.words[n]), q = (float)esp_sdr_rx_q(b.words[n]);

		sum_sq += i * i + q * q;
	}
	for (size_t n = 1; n < b.count; n++) {
		float i0 = (float)esp_sdr_rx_i(b.words[n - 1]), q0 = (float)esp_sdr_rx_q(b.words[n - 1]);
		float i1 = (float)esp_sdr_rx_i(b.words[n]), q1 = (float)esp_sdr_rx_q(b.words[n]);
		float cross = q1 * i0 - i1 * q0, dot = i1 * i0 + q1 * q0;

		sum_delta += atan2f(cross, dot);
	}

	float power_dbfs = 10.0f * log10f(MAX(sum_sq / (float)b.count / (512.0f * 512.0f), 1e-12f));
	float avg_freq = sum_delta / (float)(b.count - 1) * fs / (float)FM_TWO_PI;

	shell_print(sh, "tonerx: %u samples, %.1f dBFS, avg freq %+.0f Hz", (unsigned int)b.count,
		    (double)power_dbfs, (double)avg_freq);
	return 0;
}

/*
 * Minimal 2-FSK: one bit is FSK_BIT_US of a continuous-phase tone at
 * +-FSK_DEV_HZ from the tuned LO. Frame: 0xAA preamble byte (for bit-timing
 * search on receive), a length byte, then up to FSK_MAX_MSG message bytes,
 * all MSB first. No error correction and no CRC: this is a demo of the
 * modulation and of differential-phase demodulation, not a protocol.
 * The whole frame must fit tx_buf at the transmit rate, hence the short bits.
 */
#define FSK_BIT_US     2U  /* 500 kbit/s */
#define FSK_SPB        32U /* receive samples per bit: FSK_BIT_US at 16 MS/s */
#define FSK_DEV_HZ     200000.0f
#define FSK_PREAMBLE   0xAAU
#define FSK_MAX_MSG    3U
#define FSK_MAX_BITS   ((2U + FSK_MAX_MSG) * 8U)
#define FSK_SEARCH_STEP 8U
/* No shared clock between boards: keep capturing bursts for this long so an
 * independently, manually triggered sdr fsktx has a real chance of landing
 * inside the window instead of between two 1 ms-ish captures.
 */
#define FSK_RX_WINDOW_MS 4000U

static int cmd_fsktx(const struct shell *sh, size_t argc, char **argv)
{
	const char *text = argv[1];
	size_t len = strlen(text);
	uint8_t frame[2U + FSK_MAX_MSG];
	float fs = (float)esp_sdr_tx_rate_hz(TX_RATE);
	size_t spb = (size_t)(fs / 1e6f) * FSK_BIT_US;
	float phase = 0.0f;
	size_t bit = 0, nbits, nsamples;
	int ret;

	ARG_UNUSED(argc);
	if (len == 0 || len > FSK_MAX_MSG) {
		shell_error(sh, "message must be 1..%u bytes", FSK_MAX_MSG);
		return -EINVAL;
	}

	frame[0] = FSK_PREAMBLE;
	frame[1] = (uint8_t)len;
	memcpy(&frame[2], text, len);
	nbits = (2U + len) * 8U;
	nsamples = nbits * spb;
	if (nsamples > TX_BUF_SAMPLES) {
		shell_error(sh, "frame needs %u samples, buffer holds %u", (unsigned int)nsamples,
			    TX_BUF_SAMPLES);
		return -ENOMEM;
	}

	for (size_t byte = 0; byte < 2U + len; byte++) {
		for (int b = 7; b >= 0; b--, bit++) {
			float freq = ((frame[byte] >> b) & 1) ? FSK_DEV_HZ : -FSK_DEV_HZ;

			for (size_t s = 0; s < spb; s++) {
				phase += FM_TWO_PI_F * freq / fs;
				tx_buf[bit * spb + s] =
					esp_sdr_tx_word((int16_t)(400.0f * cosf(phase)),
							(int16_t)(400.0f * sinf(phase)));
			}
		}
	}

	ret = esp_sdr_tx_play(TX_RATE, tx_buf, nsamples);
	if (ret != 0) {
		shell_error(sh, "play: %d", ret);
		return ret;
	}
	shell_print(sh, "fsk tx: %u bytes ('%s'), %u bits, %u samples", (unsigned int)len, text,
		    (unsigned int)nbits, (unsigned int)nsamples);
	return 0;
}

/*
 * Instantaneous frequency of the bit starting at words[start], from the
 * phase angle between each pair of subsequent samples: delta = angle(w[n] *
 * conj(w[n-1])), computed as atan2 of the cross and dot products so there is
 * no need to unwrap a running phase.
 */
static bool fsk_bit(const uint32_t *words, size_t start)
{
	float cross_sum = 0.0f, dot_sum = 0.0f;

	/* Sum the per-sample cross/dot products first and take one atan2f of the
	 * total, instead of one atan2f per sample pair: equivalent for the small
	 * per-sample phase steps here, and ~FSK_SPB times fewer atan2f calls -
	 * that call was the search loop's real cost, not the float/double choice.
	 */
	for (size_t s = 1; s < FSK_SPB; s++) {
		size_t idx = start + s;
		float i0 = (float)esp_sdr_rx_i(words[idx - 1]), q0 = (float)esp_sdr_rx_q(words[idx - 1]);
		float i1 = (float)esp_sdr_rx_i(words[idx]), q1 = (float)esp_sdr_rx_q(words[idx]);

		cross_sum += q1 * i0 - i1 * q0;
		dot_sum += i1 * i0 + q1 * q0;
	}
	return atan2f(cross_sum, dot_sum) > 0.0f;
}

static int cmd_fskrx(const struct shell *sh, size_t argc, char **argv)
{
	int64_t t0 = k_uptime_get();
	unsigned int attempt = 0;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (; k_uptime_get() - t0 < FSK_RX_WINDOW_MS; attempt++) {
		struct esp_sdr_rx_burst b;
		size_t best_off = 0, pos;
		unsigned int best_errors = 9U;
		uint8_t len;
		char msg[FSK_MAX_MSG + 1U];

		if (esp_sdr_rx_capture(ESP_SDR_RATE_16MSPS, ESP_SDR_SAMPLES_MAX, &b) != 0) {
			continue;
		}

		/* Brute-force bit-timing search: slide a candidate start and score
		 * it against the known preamble bits.
		 */
		for (size_t off = 1; off + 8U * FSK_SPB <= b.count; off += FSK_SEARCH_STEP) {
			unsigned int errors = 0;

			for (unsigned int k = 0; k < 8U; k++) {
				bool bitval = fsk_bit(b.words, off + (size_t)k * FSK_SPB);
				bool expect = ((FSK_PREAMBLE >> (7U - k)) & 1U) != 0U;

				errors += (bitval != expect);
			}
			if (errors < best_errors) {
				best_errors = errors;
				best_off = off;
			}
		}
		if (best_errors > 1U) {
			continue; /* no preamble in this burst; try the next one */
		}

		pos = best_off + 8U * FSK_SPB;
		if (pos + 8U * FSK_SPB > b.count) {
			continue;
		}
		len = 0;
		for (unsigned int k = 0; k < 8U; k++) {
			len = (uint8_t)((len << 1) | (fsk_bit(b.words, pos + (size_t)k * FSK_SPB)
							       ? 1U
							       : 0U));
		}
		pos += 8U * FSK_SPB;
		if (len == 0U || len > FSK_MAX_MSG || pos + (size_t)len * 8U * FSK_SPB > b.count) {
			continue;
		}

		for (uint8_t byte = 0; byte < len; byte++) {
			uint8_t v = 0;

			for (unsigned int k = 0; k < 8U; k++) {
				v = (uint8_t)((v << 1) |
					      (fsk_bit(b.words, pos + (size_t)k * FSK_SPB) ? 1U
										    : 0U));
			}
			pos += 8U * FSK_SPB;
			msg[byte] = (char)v;
		}
		msg[len] = '\0';
		shell_print(sh, "fsk rx: %u bytes, %u preamble errors, attempt %u: \"%s\"",
			    (unsigned int)len, best_errors, attempt + 1, msg);
		return 0;
	}

	shell_error(sh, "fsk rx: no signal found in %u attempts (%u ms)", attempt,
		    FSK_RX_WINDOW_MS);
	return -ENOMSG;
}

#define SDR_TX_CMD                                                                                \
	SHELL_CMD_ARG(fm, NULL, "<fm_hz> <dev_hz> [amp] play one FM sine burst", cmd_fm, 3, 1),   \
	SHELL_CMD_ARG(tonetx, NULL, "[hz] [ms] [amp] [rate 0|1] tone at 80|40 MS/s, default +1 MHz",\
		      cmd_tonetx, 1, 4),                                                          \
	SHELL_CMD_ARG(sweep, NULL, "<f0_hz> <f1_hz> <ms> [amp] [rate 0|1] linear frequency sweep",\
		      cmd_sweep, 4, 2),                                                           \
	SHELL_CMD_ARG(tonerx, NULL, "one-shot capture: average power and frequency", cmd_tonerx,  \
		      1, 0),                                                                      \
	SHELL_CMD_ARG(fsktx, NULL, "<text> send one FSK frame (<=3 bytes)", cmd_fsktx, 2, 0),     \
	SHELL_CMD_ARG(fskrx, NULL, "receive one FSK frame (differential-phase demod)", cmd_fskrx, \
		      1, 0),
#else
#define SDR_TX_CMD
#endif

static int parse_long(const struct shell *sh, const char *s, long min, long max, long *out)
{
	char *end;
	long v = strtol(s, &end, 0);

	if (*s == '\0' || *end != '\0' || v < min || v > max) {
		shell_error(sh, "bad value: %s (%ld to %ld)", s, min, max);
		return -EINVAL;
	}
	*out = v;
	return 0;
}

static int set_freq(const struct shell *sh, const char *value)
{
	long mhz;

	if (parse_long(sh, value, ESP_SDR_FREQ_MIN_MHZ, ESP_SDR_FREQ_MAX_MHZ, &mhz) != 0) {
		return -EINVAL;
	}
	return esp_sdr_set_freq((uint32_t)mhz);
}

static int set_rxgain(const struct shell *sh, const char *value)
{
	long g = ESP_SDR_RX_GAIN_AUTO;

	if (strcmp(value, "auto") != 0 &&
	    parse_long(sh, value, ESP_SDR_RX_GAIN_AUTO, esp_sdr_rx_gain_max(), &g) != 0) {
		return -EINVAL;
	}
	return esp_sdr_rx_set_gain((int)g);
}

static int set_decim(const struct shell *sh, const char *value)
{
	if (strcmp(value, "cic") == 0) {
		return vrt_rx_set_decim(VRT_RX_DECIM_CIC);
	}
	if (strcmp(value, "fold") == 0) {
		return vrt_rx_set_decim(VRT_RX_DECIM_FOLD);
	}
	shell_error(sh, "decim: cic or fold");
	return -EINVAL;
}

#if defined(CONFIG_APP_TX)
static int set_txgain(const struct shell *sh, const char *value)
{
	long step;

	if (parse_long(sh, value, 0, esp_sdr_tx_gain_max(), &step) != 0) {
		return -EINVAL;
	}
	return esp_sdr_tx_set_gain((int)step);
}

static void print_tx_status(const struct shell *sh)
{
	struct esp_sdr_tx_stats bs;
	struct vrt_tx_stats st;
	uint64_t freq;
	uint32_t rate;

	vrt_tx_get_stats(&st);
	vrt_tx_get_settings(&freq, &rate);
	esp_sdr_tx_get_stats(&bs);
	shell_print(sh, "tx %s, backend %s, %llu Hz, %u S/s",
		    esp_sdr_tx_active() ? "active" : "idle", esp_sdr_tx_get_backend()->name,
		    (unsigned long long)freq, rate);
	shell_print(sh,
		    "packets %u data, %u commands, %u acks, %u gaps, %u underruns, %u bad, "
		    "%u errors; samples %u hole-filled, %u late",
		    st.data, st.commands, st.acks, st.gaps, st.underruns, st.bad, st.errors, st.holes,
		    st.late);
	shell_print(sh, "backend %llu samples in %u writes, %u errors",
		    (unsigned long long)bs.samples, bs.writes, bs.errors);
#if defined(CONFIG_ESP_SDR_TX_DAC)
	struct esp_sdr_tx_dac_stats ds;

	esp_sdr_tx_dac_get_stats(&ds);
	shell_print(sh,
		    "dac x%u: %llu played, %llu dropped (%.0f %% on air), %u switches, %u underruns, "
		    "%u overruns, %u errors, %u ms",
		    ds.interp, (unsigned long long)ds.played, (unsigned long long)ds.dropped,
		    ds.played + ds.dropped ? 100.0 * ds.played / (ds.played + ds.dropped) : 0.0,
		    ds.switches, ds.underruns, ds.overruns, ds.errors, ds.elapsed_us / 1000U);
	shell_print(sh, "dac interpolator: %s", ds.simd ? "vector (PIE)" : "C");
	shell_print(sh, "dac loop: %u switches, %u us mean apart, %u us max; slack min %d us, "
		    "%u restarts",
		    ds.switches, ds.switches > 1U ? ds.switch_us_sum / (ds.switches - 1U) : 0U,
		    ds.switch_us_max, ds.slack_us_min, ds.restarts);
	shell_print(sh, "dac leader: fill max %u us, woke late max %u us, switch-to-fill max %u us",
		    ds.fill_us_max, ds.wake_late_us_max, ds.idle_us_max);
	for (int f = 0; f < 2; f++) {
		shell_print(sh, "dac filler %d: %u blocks, %u ms filling, cpu %u", f,
			    ds.fill_blocks[f], ds.fill_us[f] / 1000U, ds.fill_cpu[f]);
	}
#endif
}

#endif

static const struct setting {
	const char *key, *help;
	int (*set)(const struct shell *sh, const char *value);
} settings[] = {
	{"freq", "<MHz>: LO, shared by receive and transmit", set_freq},
	{"rxgain", "auto|<index>: hardware AGC or a fixed gain index", set_rxgain},
	{"decim", "cic|fold: decimation below the capture rate (fold: full-band power)", set_decim},
#if defined(CONFIG_APP_TX)
	{"txgain", "<step>: transmit power step, 0 weakest", set_txgain},
#endif
};

/* set [<key> <value> ...]: settings, applied in order; without arguments the keys. */
static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		ARRAY_FOR_EACH(settings, k) {
			shell_print(sh, "  %-7s %s", settings[k].key, settings[k].help);
		}
		return 0;
	}
	if ((argc - 1U) % 2U != 0U) {
		shell_error(sh, "usage: set <key> <value> [<key> <value> ...]");
		return -EINVAL;
	}
	for (size_t a = 1; a + 1U < argc; a += 2U) {
		const struct setting *st = NULL;
		int ret;

		ARRAY_FOR_EACH(settings, k) {
			if (strcmp(argv[a], settings[k].key) == 0) {
				st = &settings[k];
			}
		}
		if (st == NULL) {
			shell_error(sh, "unknown key %s (sdr set lists them)", argv[a]);
			return -EINVAL;
		}
		ret = st->set(sh, argv[a + 1U]);
		shell_print(sh, "%s %s: %d", argv[a], argv[a + 1U], ret);
		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

/* status: receive settings, and the transmit path's state and counters. */
static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	int gain = esp_sdr_rx_get_gain();
	const char *decim = vrt_rx_get_decim() == VRT_RX_DECIM_FOLD ? "fold" : "cic";

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (gain < 0) {
		shell_print(sh, "rx: freq %u MHz, rxgain auto (max %d), decim %s, %u S/s, %u bit",
			    esp_sdr_get_freq(), esp_sdr_rx_gain_max(), decim, vrt_rx_get_rate(),
			    vrt_rx_get_bits());
	} else {
		shell_print(sh, "rx: freq %u MHz, rxgain %d (max %d), decim %s, %u S/s, %u bit",
			    esp_sdr_get_freq(), gain, esp_sdr_rx_gain_max(), decim, vrt_rx_get_rate(),
			    vrt_rx_get_bits());
	}
#if defined(CONFIG_APP_TX)
	shell_print(sh, "txgain %d (max %d, vendor target power %d)", esp_sdr_tx_get_gain(),
		    esp_sdr_tx_gain_max(), esp_sdr_tx_gain_power(esp_sdr_tx_get_gain()));
	print_tx_status(sh);
#endif
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sdr_cmds,
			       SHELL_CMD(status, NULL, "settings, transmit path state and counters",
					 cmd_status),
			       SHELL_CMD_ARG(set, NULL,
					     "[<key> <value> ...]: settings (without arguments: the keys)",
					     cmd_set, 1, 8),
			       SHELL_CMD(mem, NULL, "system heap use", cmd_mem),
			       SDR_TX_CMD SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(sdr, &sdr_cmds, "ESP-SDR radio", NULL);
