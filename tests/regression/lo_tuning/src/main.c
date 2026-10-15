/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Firmware of the two-board LO tuning regression test (pytest/test_lo_tuning.py):
 * the ESP32-S3 transmits a tone, the ESP32-C6 captures and says where it landed.
 *
 *   lo f <MHz>            tune; prints the esp_sdr_set_freq() return (-ERANGE: out of window)
 *   lo gain <rx> [tx]     fixed receive gain index (-1: AGC), transmit power step (S3)
 *   lo tone on <Hz> [amp] S3: loop a tone at the LO plus Hz until "lo tone off"
 *   lo tone off
 *   lo rx <Hz>            C6: capture at 80 MS/s; strongest tone and the level at LO plus Hz
 *
 * Every filter is wide open (receive and, on the S3, transmit low-pass code 0).
 */

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>
#include <esp_sdr/esp_sdr_rx.h>
#if defined(CONFIG_SOC_SERIES_ESP32S3)
#include <esp_sdr/esp_sdr_tx.h>
#endif

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include "app_usb.h"
#endif

#define TWO_PI_F 6.28318530717958647692f

#if defined(CONFIG_SOC_SERIES_ESP32S3)
/* A tone's loop session holds the radio until "lo tone off". */
static bool lo_tone_active;
#endif

#if !defined(CONFIG_SOC_SERIES_ESP32S3)
/* Capture and analysis on the C6: Welch average of Hann-windowed FFT segments. */
#define RX_SAMPLES   8192U
#define NFFT         1024U
#define SEGMENTS     (RX_SAMPLES / NFFT)
#define RX_FS_HZ     80000000.0f
#define BIN_HZ       (RX_FS_HZ / (float)NFFT)
/* The receiver's own DC and LO leakage. */
#define DC_SKIP_HZ   150000.0f
/* Window around the expected offset: crystal tolerance of both boards plus a bin. */
#define MATCH_HZ     150000.0f
/* Local noise floor: median of the bins within +-1 MHz. */
#define FLOOR_HALF   13U

static float re[NFFT], im[NFFT], win[NFFT], psd[NFFT], floor_psd[NFFT];
static float tw_cos[NFFT / 2], tw_sin[NFFT / 2];

static void fft(float *xr, float *xi)
{
	for (unsigned int i = 1, j = 0; i < NFFT; i++) {
		unsigned int bit = NFFT >> 1;

		for (; j & bit; bit >>= 1) {
			j ^= bit;
		}
		j |= bit;
		if (i < j) {
			float t = xr[i];

			xr[i] = xr[j];
			xr[j] = t;
			t = xi[i];
			xi[i] = xi[j];
			xi[j] = t;
		}
	}
	for (unsigned int len = 2, step = NFFT / 2; len <= NFFT; len <<= 1, step >>= 1) {
		for (unsigned int i = 0; i < NFFT; i += len) {
			for (unsigned int k = 0; k < len / 2; k++) {
				float wr = tw_cos[k * step], wi = -tw_sin[k * step];
				unsigned int a = i + k, b = a + len / 2;
				float tr = xr[b] * wr - xi[b] * wi;
				float ti = xr[b] * wi + xi[b] * wr;

				xr[b] = xr[a] - tr;
				xi[b] = xi[a] - ti;
				xr[a] += tr;
				xi[a] += ti;
			}
		}
	}
}

/* Baseband frequency of FFT bin k (natural order: 0 .. +fs/2, then -fs/2 .. 0). */
static float bin_hz(unsigned int k)
{
	return (k < NFFT / 2 ? (float)k : (float)k - (float)NFFT) * BIN_HZ;
}

static float median_floor(unsigned int k)
{
	float v[2 * FLOOR_HALF + 1];
	unsigned int n = 0;

	for (int d = -(int)FLOOR_HALF; d <= (int)FLOOR_HALF; d++) {
		float x = psd[(k + NFFT + d) % NFFT];
		unsigned int j = n++;

		/* Insertion sort, 27 values. */
		for (; j > 0 && v[j - 1] > x; j--) {
			v[j] = v[j - 1];
		}
		v[j] = x;
	}
	return v[FLOOR_HALF];
}

/* Bin k's power over its local floor, in tenths of a dB. */
static int level_ddb(unsigned int k)
{
	float ratio = psd[k] / MAX(floor_psd[k], 1e-20f);

	return (int)lroundf(100.0f * log10f(MAX(ratio, 1e-6f)));
}

static int cmd_rx(const struct shell *sh, size_t argc, char **argv)
{
	long expect_hz = strtol(argv[1], NULL, 0);
	struct esp_sdr_rx_burst b = {0};
	float dc_i = 0.0f, dc_q = 0.0f, power = 0.0f;
	unsigned int clipped = 0, peak = 0, match = 0;
	int peak_ddb = INT32_MIN, match_ddb = INT32_MIN;
	int ret;

	ARG_UNUSED(argc);
	ret = esp_sdr_rx_capture(ESP_SDR_RATE_80MSPS, RX_SAMPLES, &b);
	if (ret != 0 || b.count < RX_SAMPLES) {
		shell_error(sh, "rx: capture %d, %u samples", ret, (unsigned int)b.count);
		return ret != 0 ? ret : -EIO;
	}
	for (size_t n = 0; n < RX_SAMPLES; n++) {
		int i = esp_sdr_rx_i(b.words[n]), q = esp_sdr_rx_q(b.words[n]);

		dc_i += (float)i;
		dc_q += (float)q;
		clipped += abs(i) >= 511 || abs(q) >= 511;
	}
	dc_i /= (float)RX_SAMPLES;
	dc_q /= (float)RX_SAMPLES;

	memset(psd, 0, sizeof(psd));
	for (unsigned int s = 0; s < SEGMENTS; s++) {
		const uint32_t *w = &b.words[s * NFFT];

		for (unsigned int k = 0; k < NFFT; k++) {
			float i = (float)esp_sdr_rx_i(w[k]) - dc_i;
			float q = (float)esp_sdr_rx_q(w[k]) - dc_q;

			power += i * i + q * q;
			re[k] = i * win[k];
			im[k] = q * win[k];
		}
		fft(re, im);
		for (unsigned int k = 0; k < NFFT; k++) {
			psd[k] += re[k] * re[k] + im[k] * im[k];
		}
	}
	for (unsigned int k = 0; k < NFFT; k++) {
		floor_psd[k] = median_floor(k);
	}
	for (unsigned int k = 0; k < NFFT; k++) {
		float f = bin_hz(k);
		int ddb = level_ddb(k);

		if (fabsf(f) > DC_SKIP_HZ && ddb > peak_ddb) {
			peak_ddb = ddb;
			peak = k;
		}
		if (fabsf(f - (float)expect_hz) <= MATCH_HZ && ddb > match_ddb) {
			match_ddb = ddb;
			match = k;
		}
	}
	if (match_ddb == INT32_MIN) {
		shell_error(sh, "rx: %ld Hz is outside the capture", expect_hz);
		return -EINVAL;
	}
	shell_print(sh,
		    "rx lo %u peak %d Hz %d ddB expect %ld Hz %d ddB at %d Hz rms %d clip %u",
		    esp_sdr_get_freq(), (int)bin_hz(peak), peak_ddb, expect_hz, match_ddb,
		    (int)bin_hz(match),
		    (int)lroundf(10.0f * sqrtf(power / (float)RX_SAMPLES)), clipped);
	return 0;
}

static void rx_init(void)
{
	for (unsigned int k = 0; k < NFFT; k++) {
		win[k] = 0.5f - 0.5f * cosf(TWO_PI_F * (float)k / (float)NFFT);
	}
	for (unsigned int k = 0; k < NFFT / 2; k++) {
		tw_cos[k] = cosf(TWO_PI_F * (float)k / (float)NFFT);
		tw_sin[k] = sinf(TWO_PI_F * (float)k / (float)NFFT);
	}
}
#endif /* !CONFIG_SOC_SERIES_ESP32S3 */

#if defined(CONFIG_SOC_SERIES_ESP32S3)
#define TX_RATE ESP_SDR_RATE_80MSPS

/* tone on <Hz> [amp]: a whole number of periods in the bank, looped until "tone off". */
static int cmd_tone_on(const struct shell *sh, size_t argc, char **argv)
{
	long hz = strtol(argv[1], NULL, 0);
	long amp = argc > 2 ? strtol(argv[2], NULL, 0) : 400;
	double fs = (double)esp_sdr_tx_rate_hz(TX_RATE);
	size_t words, n = 0;
	long cycles = 0;
	double best = INFINITY;
	uint32_t *bank;
	int ret;

	if (lo_tone_active) {
		shell_error(sh, "tone: already on");
		return -EBUSY;
	}
	if (labs(hz) >= (long)(fs / 2.0) || amp < 1 || amp > 511) {
		shell_error(sh, "tone: |Hz| below %ld, amp 1..511", (long)(fs / 2.0));
		return -EINVAL;
	}
	ret = esp_sdr_tx_loop_begin(TX_RATE);
	if (ret != 0) {
		shell_error(sh, "tone: begin %d", ret);
		return ret;
	}
	bank = esp_sdr_tx_loop_buf(0, &words);
	words = MIN(words, ESP_SDR_SAMPLES_MAX);
	/* The period length whose whole number of cycles comes closest to hz. */
	for (size_t len = words; len >= ESP_SDR_SAMPLES_MIN; len--) {
		long c = lround((double)hz * (double)len / fs);
		double err = fabs((double)c * fs / (double)len - (double)hz);

		if (err < best) {
			best = err;
			n = len;
			cycles = c;
			if (err < 1.0) {
				break;
			}
		}
	}
	for (size_t k = 0; k < n; k++) {
		float ph = TWO_PI_F * (float)((double)cycles * (double)k / (double)n -
					      floor((double)cycles * (double)k / (double)n));

		bank[k] = esp_sdr_tx_word((int16_t)lroundf((float)amp * cosf(ph)),
					  (int16_t)lroundf((float)amp * sinf(ph)));
	}
	ret = esp_sdr_tx_loop_start(0, n);
	if (ret != 0) {
		esp_sdr_tx_loop_end();
		shell_error(sh, "tone: start %d", ret);
		return ret;
	}
	lo_tone_active = true;
	shell_print(sh, "tone on %u MHz %+ld Hz, period %u", esp_sdr_get_freq(),
		    lround((double)cycles * fs / (double)n), (unsigned int)n);
	return 0;
}

static int cmd_tone_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (lo_tone_active) {
		esp_sdr_tx_loop_end();
		lo_tone_active = false;
	}
	shell_print(sh, "tone off");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(tone_cmds,
	SHELL_CMD_ARG(on, NULL, "<Hz> [amp]", cmd_tone_on, 2, 1),
	SHELL_CMD(off, NULL, "stop", cmd_tone_off),
	SHELL_SUBCMD_SET_END);
#endif

static int cmd_f(const struct shell *sh, size_t argc, char **argv)
{
	long mhz = strtol(argv[1], NULL, 0);
	int ret;

	ARG_UNUSED(argc);
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	/* The tone's session holds the radio; a retune from this thread would cut into it. */
	if (lo_tone_active) {
		shell_error(sh, "f: tone on");
		return -EBUSY;
	}
#endif
	ret = mhz > 0 ? esp_sdr_set_freq((uint32_t)mhz) : -EINVAL;
	shell_print(sh, "f %ld ret %d", mhz, ret);
	return 0;
}

static int cmd_gain(const struct shell *sh, size_t argc, char **argv)
{
	int ret = esp_sdr_rx_set_gain((int)strtol(argv[1], NULL, 0));

#if defined(CONFIG_SOC_SERIES_ESP32S3)
	if (argc > 2) {
		int tret = esp_sdr_tx_set_gain((int)strtol(argv[2], NULL, 0));

		shell_print(sh, "txgain %d ret %d", esp_sdr_tx_get_gain(), tret);
	}
#else
	ARG_UNUSED(argc);
#endif
	shell_print(sh, "rxgain %d ret %d", esp_sdr_rx_get_gain(), ret);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(lo_cmds,
	SHELL_CMD_ARG(f, NULL, "<MHz>: tune, prints the return code", cmd_f, 2, 0),
	SHELL_CMD_ARG(gain, NULL, "<rx index|-1> [tx step]", cmd_gain, 2, 1),
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	SHELL_CMD(tone, &tone_cmds, "loop a tone", NULL),
#else
	SHELL_CMD_ARG(rx, NULL, "<Hz>: capture, strongest tone and the level at LO + Hz",
		      cmd_rx, 2, 0),
#endif
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(lo, &lo_cmds, "LO tuning regression test", NULL);

int main(void)
{
	int ret;

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	/* S3: the USB shell and DFU, and the trial watchdog that reverts an image that hangs. */
	if (app_usb_init() != 0) {
		printk("lo: usb init failed\n");
	}
#endif
#if !defined(CONFIG_SOC_SERIES_ESP32S3)
	rx_init();
#endif
	ret = esp_sdr_init();
	if (ret != 0) {
		printk("lo: init failed (%d)\n", ret);
		return 0;
	}
	(void)esp_sdr_rx_set_lpf(0);
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	(void)esp_sdr_tx_set_lpf(0, 0);
#endif
	printk("lo: ready\n");
	return 0;
}
