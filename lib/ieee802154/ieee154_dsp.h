/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Signal processing of the esp_sdr 802.15.4 driver, portable C so the unit
 * test (tests/unit/ieee154_dsp) runs the same code: the transmit
 * waveform from slot templates at the DAC rate, and the receive mix and
 * decimation of 16 MS/s capture words to the PHY's 4 MS/s.
 */

#ifndef IEEE154_DSP_H_
#define IEEE154_DSP_H_

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <esp_sdr/esp_sdr_rx.h>
#include <esp_sdr/esp_sdr_tx.h>
#include <esp_sdr/ieee154_phy.h>

/* The LO sits this far below the channel: its leakage and the DC offset stay out of the band. */
#define IEEE154_DSP_LO_OFFSET_MHZ 4U

/* ------------------------------------------------------------- transmit */

/* DAC rate 40 MS/s: one chip period (0.5 us) is one slot. */
#define IEEE154_DSP_SLOT      20U
#define IEEE154_DSP_CHIPS_MAX ((6U + IEEE154_PHY_PSDU_MAX) * 2U * 32U)

/*
 * Slot s carries the first half of chip s's half-sine pulse and the second
 * half of chip s - 1's, on I for even chips and Q for odd ones (O-QPSK). The
 * offset rotation (period 10 samples at 40 MS/s) restarts every slot, so 2
 * parities x 3 x 3 chip states (none, 0, 1) cover every slot.
 */
typedef uint32_t ieee154_dsp_tmpl_t[2][3][3][IEEE154_DSP_SLOT];

struct ieee154_dsp_tx {
	uint32_t chips[IEEE154_DSP_CHIPS_MAX / 32U];
	uint32_t nchips;
	const ieee154_dsp_tmpl_t *tmpl;
};

/* DAC words of every slot kind at amplitude amp (of 511); conj mirrors the spectrum. */
static inline void ieee154_dsp_tmpl_init(ieee154_dsp_tmpl_t *tmpl, float amp, bool conj)
{
	const float pi = 3.14159265f;

	for (int p = 0; p < 2; p++) {
		for (int c = 0; c < 3; c++) {
			for (int v = 0; v < 3; v++) {
				for (unsigned int t = 0; t < IEEE154_DSP_SLOT; t++) {
					float cur = c == 0 ? 0.0f : (c == 2 ? 1.0f : -1.0f);
					float prv = v == 0 ? 0.0f : (v == 2 ? 1.0f : -1.0f);
					float a = cur * sinf(pi * (float)t / 40.0f);
					float b = prv * sinf(pi * (float)(t + 20U) / 40.0f);
					float i = p == 0 ? a : b, q = p == 0 ? b : a;
					float th = 2.0f * pi * (float)IEEE154_DSP_LO_OFFSET_MHZ / 40.0f *
						   (float)t;
					float ri = amp * (i * cosf(th) - q * sinf(th));
					float rq = amp * (i * sinf(th) + q * cosf(th));

					(*tmpl)[p][c][v][t] = esp_sdr_tx_word(
						(int16_t)lrintf(ri), (int16_t)lrintf(conj ? -rq : rq));
				}
			}
		}
	}
}

/* Chips of SHR, PHR and psdu (FCS included), symbols low nibble first. */
static inline void ieee154_dsp_tx_build(struct ieee154_dsp_tx *tx, const uint8_t *psdu,
					uint8_t len)
{
	uint32_t nbytes = 6U + len;

	for (uint32_t b = 0; b < nbytes; b++) {
		uint8_t v = b < 4U ? 0x00U : (b == 4U ? 0xa7U : (b == 5U ? len : psdu[b - 6U]));

		tx->chips[2U * b] = ieee154_phy_chips(v & 0xf);
		tx->chips[2U * b + 1U] = ieee154_phy_chips(v >> 4);
	}
	tx->nchips = nbytes * 64U;
}

/* DAC samples of the built frame, including the last chip's second half. */
static inline uint64_t ieee154_dsp_tx_samples(const struct ieee154_dsp_tx *tx)
{
	return (uint64_t)(tx->nchips + 1U) * IEEE154_DSP_SLOT;
}

/* State of chip n: 0 none, 1 for a 0 chip, 2 for a 1 chip. */
static inline int ieee154_dsp_chip(const struct ieee154_dsp_tx *tx, uint32_t n)
{
	if (n >= tx->nchips) {
		return 0;
	}
	return 1 + (int)((tx->chips[n / 32U] >> (31U - n % 32U)) & 1U);
}

/*
 * DAC words for samples [index, index + n) of the frame (an esp_sdr_tx_dac_gen_t
 * body). Has to beat the DAC (40 MS/s) with room to spare: whole slots are
 * one fixed-size copy each.
 */
static inline void ieee154_dsp_tx_gen(const struct ieee154_dsp_tx *tx, uint32_t *dst,
				      uint64_t index, uint32_t n)
{
	uint32_t s = (uint32_t)(index / IEEE154_DSP_SLOT);
	uint32_t t = (uint32_t)(index % IEEE154_DSP_SLOT);
	int prv = s == 0U ? 0 : ieee154_dsp_chip(tx, s - 1U);

	while (n > 0U) {
		int cur = ieee154_dsp_chip(tx, s);
		const uint32_t *src = (*tx->tmpl)[s & 1U][cur][prv];

		if (t == 0U && n >= IEEE154_DSP_SLOT) {
			/* Plain word stores; a memcpy call per slot costs more than the copy. */
			for (uint32_t k = 0; k < IEEE154_DSP_SLOT; k += 4U) {
				uint32_t w0 = src[k], w1 = src[k + 1U], w2 = src[k + 2U],
					 w3 = src[k + 3U];

				dst[k] = w0;
				dst[k + 1U] = w1;
				dst[k + 2U] = w2;
				dst[k + 3U] = w3;
			}
			dst += IEEE154_DSP_SLOT;
			n -= IEEE154_DSP_SLOT;
		} else {
			uint32_t cnt = n < IEEE154_DSP_SLOT - t ? n : IEEE154_DSP_SLOT - t;

			for (uint32_t k = 0; k < cnt; k++) {
				dst[k] = src[t + k];
			}
			dst += cnt;
			n -= cnt;
			t = 0;
		}
		prv = cur;
		s++;
	}
}

/* -------------------------------------------------------------- receive */

#define IEEE154_DSP_DECIM 4U
#define IEEE154_DSP_NTAPS 27U

/* Outputs of ieee154_dsp_mix_decimate() for n input words. */
#define IEEE154_DSP_DECIM_OUT(n) (((n) - IEEE154_DSP_NTAPS) / IEEE154_DSP_DECIM + 1U)

/* Hamming windowed sinc, cutoff 1.6 MHz at 16 MS/s, Q14: -0.8 dB at 1 MHz, -51 dB from 3 MHz. */
static const int16_t ieee154_dsp_taps[IEEE154_DSP_NTAPS] = {
	30, 38, 37, 0, -95, -233, -342, -303, 0, 612, 1457, 2342, 3015, 3266,
	3015, 2342, 1457, 612, 0, -303, -342, -233, -95, 0, 37, 38, 30,
};

/*
 * 16 MS/s receive words to interleaved I/Q at 4 MS/s, +-8192 full scale:
 * output k = sum over t of taps[t] x[4k + t] (-j)^(4k + t). The mix by
 * -fs/4 depends on t only, so it folds into the taps as a swap and a sign.
 * qsign -1 conjugates the result. Returns the output count.
 */
static inline size_t ieee154_dsp_mix_decimate(const uint32_t *w, size_t n, int16_t *out,
					      int32_t qsign)
{
	const int16_t *h = ieee154_dsp_taps;
	size_t outs = IEEE154_DSP_DECIM_OUT(n);

	for (size_t k = 0; k < outs; k++) {
		const uint32_t *x = &w[IEEE154_DSP_DECIM * k];
		int32_t ai = 0, aq = 0;

		for (size_t t = 0; t < IEEE154_DSP_NTAPS; t++) {
			int32_t i = esp_sdr_rx_i(x[t]), q = esp_sdr_rx_q(x[t]);

			switch (t & 3U) {
			case 0:
				ai += h[t] * i;
				aq += h[t] * q;
				break;
			case 1: /* (i + jq)(-j) = q - ji */
				ai += h[t] * q;
				aq -= h[t] * i;
				break;
			case 2:
				ai -= h[t] * i;
				aq -= h[t] * q;
				break;
			default: /* (i + jq)(j) = -q + ji */
				ai -= h[t] * q;
				aq += h[t] * i;
				break;
			}
		}
		/* 10-bit input, Q14 taps. */
		out[2 * k] = (int16_t)(ai >> 10);
		out[2 * k + 1] = (int16_t)(qsign * (aq >> 10));
	}
	return outs;
}

/* ----------------------------------------------------------- measurement */

/*
 * Carrier offset in Hz of an O-QPSK signal in 4 MS/s I/Q (2 samples per
 * chip): x[n] conj(x[n - 2]) turns by +-90 degrees per chip (MSK) plus
 * 2 pi f 0.5 us; its square loses the sign, so the mean square points at
 * 180 degrees + 4 pi f 0.5 us. Uses samples above half the mean power (the
 * signal, when it fills most of the block); unambiguous within +-500 kHz.
 * Sets *power to the mean |x|^2 and *used to the samples counted.
 */
static inline float ieee154_dsp_cfo(const int16_t *iq, size_t n, float *power, size_t *used)
{
	double p = 0.0, mi = 0.0, mq = 0.0, sr = 0.0, si = 0.0;
	size_t cnt = 0, nm = 0;

	for (size_t k = 0; k < n; k++) {
		p += (double)iq[2 * k] * iq[2 * k] + (double)iq[2 * k + 1] * iq[2 * k + 1];
	}
	p = n != 0U ? p / (double)n : 0.0;
	/* Carrier feedthrough sits at the channel centre, 0 Hz here: remove the signal's mean. */
	for (size_t k = 0; k < n; k++) {
		double xi = iq[2 * k], xq = iq[2 * k + 1];

		if (xi * xi + xq * xq >= p / 2.0) {
			mi += xi;
			mq += xq;
			nm++;
		}
	}
	if (nm != 0U) {
		mi /= (double)nm;
		mq /= (double)nm;
	}
	for (size_t k = 2; k < n; k++) {
		double xi = iq[2 * k] - mi, xq = iq[2 * k + 1] - mq;
		double yi = iq[2 * k - 4] - mi, yq = iq[2 * k - 3] - mq;

		if ((double)iq[2 * k] * iq[2 * k] + (double)iq[2 * k + 1] * iq[2 * k + 1] < p / 2.0) {
			continue;
		}
		/* z = x conj(y), then z^2 */
		double zr = xi * yi + xq * yq, zi = xq * yi - xi * yq;

		sr += zr * zr - zi * zi;
		si += 2.0 * zr * zi;
		cnt++;
	}
	*power = (float)p;
	*used = cnt;
	if (cnt == 0U) {
		return 0.0f;
	}
	/* arg(-S) = 4 pi f T, T = 0.5 us */
	return (float)(atan2(-si, -sr) / (4.0 * 3.14159265358979 * 0.5e-6));
}

/*
 * Coarse carrier offset in Hz: the mean instantaneous frequency (phase step
 * per sample at 4 MS/s) of the samples above half the mean power, mean
 * removed. MSK's +-500 kHz deviation averages out over random chips; the
 * estimate is unambiguous within +-1.5 MHz but noisier than
 * ieee154_dsp_cfo().
 */
static inline float ieee154_dsp_freq(const int16_t *iq, size_t n)
{
	double p = 0.0, mi = 0.0, mq = 0.0, sum = 0.0;
	size_t cnt = 0, nm = 0;

	for (size_t k = 0; k < n; k++) {
		p += (double)iq[2 * k] * iq[2 * k] + (double)iq[2 * k + 1] * iq[2 * k + 1];
	}
	p = n != 0U ? p / (double)n : 0.0;
	for (size_t k = 0; k < n; k++) {
		double xi = iq[2 * k], xq = iq[2 * k + 1];

		if (xi * xi + xq * xq >= p / 2.0) {
			mi += xi;
			mq += xq;
			nm++;
		}
	}
	if (nm != 0U) {
		mi /= (double)nm;
		mq /= (double)nm;
	}
	for (size_t k = 1; k < n; k++) {
		double xi = iq[2 * k] - mi, xq = iq[2 * k + 1] - mq;
		double yi = iq[2 * k - 2] - mi, yq = iq[2 * k - 1] - mq;

		if ((double)iq[2 * k] * iq[2 * k] + (double)iq[2 * k + 1] * iq[2 * k + 1] < p / 2.0) {
			continue;
		}
		sum += atan2(xq * yi - xi * yq, xi * yi + xq * yq);
		cnt++;
	}
	/* One sample is 0.25 us. */
	return cnt != 0U ? (float)(sum / (double)cnt / (2.0 * 3.14159265358979 * 0.25e-6)) : 0.0f;
}

/*
 * Histogram of the one-chip phase steps arg(x[n] conj(x[n - 2])) of the
 * samples above half the mean power, in 45 degree bins from -180: an
 * O-QPSK (MSK) signal fills the bins around -90 and +90: bins 1 or 2 and
 * 5 or 6, by the sign of the carrier offset.
 */
static inline void ieee154_dsp_steps(const int16_t *iq, size_t n, uint32_t hist[8])
{
	double p = 0.0;

	for (int b = 0; b < 8; b++) {
		hist[b] = 0U;
	}
	for (size_t k = 0; k < n; k++) {
		p += (double)iq[2 * k] * iq[2 * k] + (double)iq[2 * k + 1] * iq[2 * k + 1];
	}
	p = n != 0U ? p / (double)n : 0.0;
	for (size_t k = 2; k < n; k++) {
		double xi = iq[2 * k], xq = iq[2 * k + 1];
		double yi = iq[2 * k - 4], yq = iq[2 * k - 3];

		if (xi * xi + xq * xq < p / 2.0) {
			continue;
		}
		double a = atan2(xq * yi - xi * yq, xi * yi + xq * yq);
		int b = (int)((a + 3.14159265358979) / (3.14159265358979 / 4.0));

		hist[b < 0 ? 0 : (b > 7 ? 7 : b)]++;
	}
}

#endif /* IEEE154_DSP_H_ */
