/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <zephyr/sys/util.h>

#include "iq_pack.h"

size_t iq_samples_per_words(unsigned int bits, size_t max_words)
{
	size_t n = max_words * 32U / (2U * bits);

	return n - n % iq_sample_quantum(bits);
}

/* Item of `bits` from a sample: scaled by 2^-shift (rounded) or 2^shift, saturated. */
static inline uint32_t item(int32_t x, int32_t shift, int32_t lim, uint32_t mask)
{
	if (shift > 0) {
		x = (x + (1 << (shift - 1))) >> shift;
	} else {
		x <<= -shift;
	}
	return (uint32_t)CLAMP(x, -lim, lim - 1) & mask;
}

size_t iq_pack(const struct esp_sdr_iq16 *s, size_t n, unsigned int bits, int32_t full_scale,
	       uint32_t *words)
{
	int32_t lim = 1 << (bits - 1U);
	uint32_t mask = (1U << bits) - 1U;
	/* full_scale is a power of two: 512 (raw) or 32768 (decimated). */
	int32_t shift = (int32_t)(31 - __builtin_clz((uint32_t)full_scale)) - (int32_t)(bits - 1U);
	uint64_t acc = 0;
	unsigned int have = 0;
	size_t w = 0, j = 0;

	if (bits == 16U) {
		/* Legacy layout: native scale, I in the upper half word. */
		for (; j < n; j++) {
			words[j] = ((uint32_t)(uint16_t)s[j].i << 16) | (uint16_t)s[j].q;
		}
		return n;
	}
	if (bits == 8U) {
		/* Two samples per word. */
		for (; j + 2U <= n; j += 2U) {
			words[w++] = item(s[j].i, shift, lim, mask) << 24 |
				     item(s[j].q, shift, lim, mask) << 16 |
				     item(s[j + 1U].i, shift, lim, mask) << 8 |
				     item(s[j + 1U].q, shift, lim, mask);
		}
	} else if (bits == 12U) {
		/* Four samples, eight items, in three words. */
		for (; j + 4U <= n; j += 4U) {
			uint32_t a = item(s[j].i, shift, lim, mask), b = item(s[j].q, shift, lim, mask);
			uint32_t c = item(s[j + 1U].i, shift, lim, mask);
			uint32_t d = item(s[j + 1U].q, shift, lim, mask);
			uint32_t e = item(s[j + 2U].i, shift, lim, mask);
			uint32_t f = item(s[j + 2U].q, shift, lim, mask);
			uint32_t g = item(s[j + 3U].i, shift, lim, mask);
			uint32_t h = item(s[j + 3U].q, shift, lim, mask);

			words[w++] = a << 20 | b << 8 | c >> 4;
			words[w++] = c << 28 | d << 16 | e << 4 | f >> 8;
			words[w++] = f << 24 | g << 12 | h;
		}
	}
	/* Whatever is left (none in whole quanta), MSB first. */
	for (; j < n; j++) {
		acc = (acc << (2U * bits)) | ((uint64_t)item(s[j].i, shift, lim, mask) << bits) |
		      item(s[j].q, shift, lim, mask);
		have += 2U * bits;
		while (have >= 32U) {
			have -= 32U;
			words[w++] = (uint32_t)(acc >> have);
		}
	}
	if (have > 0U) {
		/* Left aligned, zero padded (less than one item thanks to the quantum). */
		words[w++] = (uint32_t)(acc << (32U - have));
	}
	return w;
}

size_t iq_unpack(const uint32_t *words, size_t nwords, unsigned int bits, int16_t *iq,
		 size_t max)
{
	uint64_t acc = 0;
	unsigned int have = 0;
	size_t n = 0;

	if (bits == 16U) {
		for (; n < nwords && n < max; n++) {
			iq[2U * n] = (int16_t)(words[n] >> 16);
			iq[2U * n + 1U] = (int16_t)(words[n] & 0xffffU);
		}
		return n;
	}
	for (size_t k = 0; k < nwords && n < max; k++) {
		acc = (acc << 32) | words[k];
		have += 32U;
		while (have >= 2U * bits && n < max) {
			uint32_t pair = (uint32_t)(acc >> (have - 2U * bits));

			have -= 2U * bits;
			/* Item to the top of 32 bits (sign there), down 16: int16 full scale. */
			iq[2U * n] = (int16_t)((int32_t)((pair >> bits) << (32U - bits)) >> 16);
			iq[2U * n + 1U] = (int16_t)((int32_t)(pair << (32U - bits)) >> 16);
			n++;
		}
	}
	return n;
}
