/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Unit test of apps/link/src/hamming.c: extended Hamming(128,120), one
 * block is 15 data bytes (120 bits) and one parity byte (8 bits). Must
 * correct any single bit error (exhaustive over all 128 bit positions)
 * and detect any two bit errors (exhaustive over all pairs). Runs on
 * native_sim.
 */
#include <string.h>

#include <zephyr/ztest.h>

#include "hamming.h"

static void flip(uint8_t *blk, unsigned int bit)
{
	blk[bit / 8U] ^= (uint8_t)(1U << (bit % 8U));
}

ZTEST_SUITE(link_hamming, NULL, NULL, NULL, NULL, NULL);

ZTEST(link_hamming, test_clean_block)
{
	uint8_t blk[HAM_N], data[HAM_K];

	ham_init();
	for (int i = 0; i < HAM_K; i++) {
		blk[i] = (uint8_t)(0x55 ^ i * 37);
	}
	memcpy(data, blk, HAM_K);
	ham_encode(blk);

	zassert_equal(ham_decode(blk), 0, "a clean block must decode with 0 corrections");
	zassert_mem_equal(blk, data, HAM_K, "payload changed on a clean decode");
}

/* Every one of the 128 bit positions (120 data + 8 parity) must be corrected. */
ZTEST(link_hamming, test_corrects_any_single_bit)
{
	uint8_t clean[HAM_N];

	ham_init();
	for (int i = 0; i < HAM_K; i++) {
		clean[i] = (uint8_t)(0xa5 ^ i * 61);
	}
	ham_encode(clean);

	for (unsigned int bit = 0; bit < HAM_N * 8U; bit++) {
		uint8_t blk[HAM_N];
		int ret;

		memcpy(blk, clean, HAM_N);
		flip(blk, bit);
		ret = ham_decode(blk);
		zassert_equal(ret, 1, "bit %u: ham_decode() returned %d, expected 1", bit, ret);
		/*
		 * A flip inside the parity byte itself (bit >= HAM_K * 8) needs no
		 * data correction, and ham_decode() does not rewrite its own
		 * (now stale, but no longer consulted) check bits: only the data
		 * bytes are guaranteed restored. A flip in a data byte must
		 * restore the whole block, parity byte included, since that byte
		 * was never touched.
		 */
		if (bit < HAM_K * 8U) {
			zassert_mem_equal(blk, clean, HAM_N, "bit %u: block not fully restored",
					  bit);
		} else {
			zassert_mem_equal(blk, clean, HAM_K,
					  "bit %u (a check bit): data bytes changed", bit);
		}
	}
}

/* Every one of the C(128,2) = 8128 bit pairs must be flagged, never corrected to the
 * wrong codeword.
 */
ZTEST(link_hamming, test_detects_any_two_bits)
{
	uint8_t clean[HAM_N];
	unsigned int total_bits = HAM_N * 8U;

	ham_init();
	for (int i = 0; i < HAM_K; i++) {
		clean[i] = (uint8_t)(0x3c ^ i * 97);
	}
	ham_encode(clean);

	for (unsigned int a = 0; a < total_bits; a++) {
		for (unsigned int b = a + 1U; b < total_bits; b++) {
			uint8_t blk[HAM_N];
			int ret;

			memcpy(blk, clean, HAM_N);
			flip(blk, a);
			flip(blk, b);
			ret = ham_decode(blk);
			zassert_equal(ret, -1, "bits %u,%u: ham_decode() returned %d, expected -1",
				      a, b, ret);
		}
	}
}
