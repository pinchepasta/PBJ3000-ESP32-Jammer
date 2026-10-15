/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Burst I/Q captures across a few settings, each summarized with power
 * statistics and a coarse ASCII spectrum.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/app_version.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>

#define SAMPLES   4096U
#define NFFT      256U
#define COLUMNS   64U
#define FULLSCALE 512.0f
#define FLOOR_DB  (-120.0f)
#define TWO_PI    6.28318530718f

struct survey_step {
	uint32_t mhz;
	enum esp_sdr_rate rate;
	uint32_t bandwidth_mhz; /* 0: PHY-calibrated filter */
};

static const struct survey_step survey[] = {
	{.mhz = 2412, .rate = ESP_SDR_RATE_80MSPS},
	{.mhz = 2437, .rate = ESP_SDR_RATE_40MSPS},
	{.mhz = 2462, .rate = ESP_SDR_RATE_16MSPS},
	{.mhz = 2412, .rate = ESP_SDR_RATE_80MSPS, .bandwidth_mhz = 69},
	{.mhz = 2412, .rate = ESP_SDR_RATE_80MSPS, .bandwidth_mhz = 13},
	{.mhz = 1900, .rate = ESP_SDR_RATE_80MSPS},
};

static float re[NFFT], im[NFFT], window[NFFT], psd[NFFT], column_db[COLUMNS];
/* NFFT * sum(window^2): bins then sum to the mean power. */
static float window_gain;

static void fft(float *xr, float *xi, unsigned int n)
{
	for (unsigned int i = 1, j = 0; i < n; i++) {
		unsigned int bit = n >> 1;

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
	for (unsigned int len = 2; len <= n; len <<= 1) {
		float ang = -TWO_PI / (float)len;

		for (unsigned int i = 0; i < n; i += len) {
			for (unsigned int k = 0; k < len / 2; k++) {
				float wr = cosf(ang * (float)k), wi = sinf(ang * (float)k);
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

static float to_db(float power)
{
	return power > 0.0f ? 10.0f * log10f(power) : FLOOR_DB;
}

/* Welch average of Hann-windowed segments, per-bin power relative to full scale. */
static void spectrum(const struct esp_sdr_rx_burst *b, float dc_i, float dc_q)
{
	static const char ramp[] = " .:-=+*#%@";
	unsigned int segments = b->count / NFFT;
	char line[COLUMNS + 1];
	float top = FLOOR_DB, low = 0.0f;

	for (unsigned int k = 0; k < NFFT; k++) {
		psd[k] = 0.0f;
	}
	for (unsigned int s = 0; s < segments; s++) {
		const uint32_t *w = &b->words[s * NFFT];

		for (unsigned int k = 0; k < NFFT; k++) {
			re[k] = ((float)esp_sdr_rx_i(w[k]) - dc_i) * window[k] / FULLSCALE;
			im[k] = ((float)esp_sdr_rx_q(w[k]) - dc_q) * window[k] / FULLSCALE;
		}
		fft(re, im, NFFT);
		for (unsigned int k = 0; k < NFFT; k++) {
			psd[k] += (re[k] * re[k] + im[k] * im[k]) / (window_gain * (float)segments);
		}
	}

	/* Columns run from -fs/2 to +fs/2; each holds the peak of its bins. */
	for (unsigned int c = 0; c < COLUMNS; c++) {
		float peak = 0.0f;

		for (unsigned int k = c * NFFT / COLUMNS; k < (c + 1) * NFFT / COLUMNS; k++) {
			peak = MAX(peak, psd[(k + NFFT / 2) % NFFT]);
		}
		column_db[c] = to_db(peak);
		top = MAX(top, column_db[c]);
		low = c == 0 ? column_db[c] : MIN(low, column_db[c]);
	}
	for (unsigned int c = 0; c < COLUMNS; c++) {
		float span = MAX(top - low, 1.0f);
		unsigned int level =
			(unsigned int)((column_db[c] - low) / span * (sizeof(ramp) - 2) + 0.5f);

		line[c] = ramp[MIN(level, sizeof(ramp) - 2)];
	}
	line[COLUMNS] = '\0';
	printf("  |%s| %.0f..%.0f dBFS/bin\n", line, (double)low, (double)top);
}

static int capture_step(const struct survey_step *step)
{
	struct esp_sdr_rx_burst b;
	float sum_i = 0.0f, sum_q = 0.0f, power = 0.0f;
	unsigned int clipped = 0, peak = 0;
	uint32_t rate = esp_sdr_rx_rate_hz(step->rate);
	int ret;

	/* The C6 dump engine only runs at 80 MS/s. */
	if (rate == 0U) {
		printf("esp-sdr rx: %u MHz, rate %d not supported, skipped\n", step->mhz,
		       (int)step->rate);
		return 0;
	}
	ret = esp_sdr_set_freq(step->mhz);
	if (ret == 0 && step->bandwidth_mhz) {
		uint32_t min, max;

		/* The analog filter range differs between chips. */
		esp_sdr_rx_bandwidth_range(&min, &max);
		ret = esp_sdr_rx_set_bandwidth(CLAMP(step->bandwidth_mhz, min, max));
	} else if (ret == 0) {
		ret = esp_sdr_rx_set_lpf(ESP_SDR_RX_LPF_AUTO);
	}
	if (ret == 0) {
		ret = esp_sdr_rx_capture(step->rate, SAMPLES, &b);
	}
	if (ret != 0) {
		printf("esp-sdr rx: %u MHz capture failed (%d)\n", step->mhz, ret);
		return ret;
	}

	for (size_t j = 0; j < b.count; j++) {
		int i = esp_sdr_rx_i(b.words[j]), q = esp_sdr_rx_q(b.words[j]);

		sum_i += (float)i;
		sum_q += (float)q;
		peak = MAX(peak, (unsigned int)MAX(abs(i), abs(q)));
		clipped += abs(i) >= 511 || abs(q) >= 511;
	}
	sum_i /= (float)b.count;
	sum_q /= (float)b.count;
	for (size_t j = 0; j < b.count; j++) {
		float i = (float)esp_sdr_rx_i(b.words[j]) - sum_i;
		float q = (float)esp_sdr_rx_q(b.words[j]) - sum_q;

		power += (i * i + q * q) / (FULLSCALE * FULLSCALE);
	}
	power /= (float)b.count;

	/* elapsed_us includes arming and polling, a few us on top of count / rate. */
	printf("esp-sdr rx: %u MHz, %u MS/s, lpf %d, %u samples in %u us\n", step->mhz,
	       rate / 1000000U, esp_sdr_rx_get_lpf(), (unsigned int)b.count, b.elapsed_us);
	printf("  dc %+.1f/%+.1f, power %.1f dBFS, peak %u, clipped %u\n", (double)sum_i,
	       (double)sum_q, (double)to_db(power), peak, clipped);
	spectrum(&b, sum_i, sum_q);
	return 0;
}

int main(void)
{
	int ret;

	printk("capture %s\n", APP_VERSION_STRING);
	for (unsigned int k = 0; k < NFFT; k++) {
		window[k] = 0.5f - 0.5f * cosf(TWO_PI * (float)k / (float)NFFT);
		window_gain += (float)NFFT * window[k] * window[k];
	}

	ret = esp_sdr_init();
	if (ret != 0) {
		printf("esp-sdr rx: init failed (%d)\n", ret);
		return 0;
	}
	printf("esp-sdr rx: ready\n");

	for (;;) {
		unsigned int failed = 0;

		for (size_t s = 0; s < ARRAY_SIZE(survey); s++) {
			failed += capture_step(&survey[s]) != 0;
		}
		printf("esp-sdr rx: survey done, %u failed\n", failed);
		k_sleep(K_SECONDS(10));
	}
	return 0;
}
