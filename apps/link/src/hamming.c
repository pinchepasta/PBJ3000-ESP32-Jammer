/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Data bit k (byte k / 8, bit k % 8) sits at code position pos[k], the k-th
 * of 1..127 that is not a power of two; the parity byte's bits 0..6 are the
 * check bits at positions 1, 2, 4, .., 64, and bit 7 is the overall parity.
 * The syndrome of a byte is looked up per byte position and nibble.
 */

#include "hamming.h"

/* Syndromes per byte position and nibble: 480 bytes instead of 3840. */
static uint8_t syn_lo[HAM_K][16], syn_hi[HAM_K][16];
/* Data bit at each code position, 0xff for the check bit positions. */
static uint8_t bit_at[128];

static unsigned int parity8(unsigned int v)
{
	v ^= v >> 4;
	v ^= v >> 2;
	v ^= v >> 1;
	return v & 1U;
}

void ham_init(void)
{
	uint8_t pos[HAM_K * 8];
	unsigned int k = 0;

	for (unsigned int p = 0; p < 128; p++) {
		bit_at[p] = 0xff;
	}
	for (unsigned int p = 3; p < 128; p++) {
		if ((p & (p - 1U)) != 0) {
			bit_at[p] = (uint8_t)k;
			pos[k++] = (uint8_t)p;
		}
	}
	for (unsigned int b = 0; b < HAM_K; b++) {
		for (unsigned int v = 0; v < 16; v++) {
			uint8_t lo = 0, hi = 0;

			for (unsigned int i = 0; i < 4; i++) {
				if (v & (1U << i)) {
					lo ^= pos[b * 8 + i];
					hi ^= pos[b * 8 + 4 + i];
				}
			}
			syn_lo[b][v] = lo;
			syn_hi[b][v] = hi;
		}
	}
}

static inline unsigned int data_syndrome(const uint8_t *blk, unsigned int *par)
{
	unsigned int s = 0, x = 0;

	for (unsigned int b = 0; b < HAM_K; b++) {
		s ^= syn_lo[b][blk[b] & 15U] ^ syn_hi[b][blk[b] >> 4];
		x ^= blk[b];
	}
	*par = parity8(x);
	return s;
}

void ham_encode(uint8_t *blk)
{
	unsigned int par, s = data_syndrome(blk, &par);

	/* Check bits equal the data syndrome, so the full syndrome is zero. */
	blk[HAM_K] = (uint8_t)(s | ((par ^ parity8(s)) << 7));
}

int ham_decode(uint8_t *blk)
{
	unsigned int par, s = data_syndrome(blk, &par) ^ (blk[HAM_K] & 0x7fU);
	unsigned int odd = par ^ parity8(blk[HAM_K]);

	if (s == 0) {
		/* Clean, or only the overall parity bit flipped. */
		return odd ? 1 : 0;
	}
	if (!odd) {
		return -1;
	}
	if (bit_at[s] != 0xff) {
		blk[bit_at[s] / 8] ^= (uint8_t)(1U << (bit_at[s] % 8));
	}
	return 1;
}
