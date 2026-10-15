/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Unit test of apps/link/src/rs.c: RS(255,223), 32 parity bytes, should
 * correct up to 16 byte errors per codeword and reliably flag more as
 * uncorrectable. Runs on native_sim.
 */
#include <string.h>

#include <zephyr/ztest.h>

#include "rs.h"

#define RNG_SEED 0x9e3779b97f4a7c15ULL
static uint64_t rs_rng;
static uint32_t rnd(void)
{
	rs_rng ^= rs_rng << 13;
	rs_rng ^= rs_rng >> 7;
	rs_rng ^= rs_rng << 17;
	return (uint32_t)(rs_rng >> 32);
}

/* e distinct positions in [0, RS_N), each XORed with a random non-zero byte. */
static void corrupt(uint8_t *cw, unsigned int e)
{
	uint8_t pos[RS_N];

	for (int i = 0; i < RS_N; i++) {
		pos[i] = (uint8_t)i;
	}
	/* Partial Fisher-Yates: the first e entries end up a random, distinct subset. */
	for (unsigned int i = 0; i < e; i++) {
		unsigned int j = i + rnd() % (unsigned int)(RS_N - i);
		uint8_t t = pos[i];

		pos[i] = pos[j];
		pos[j] = t;
	}
	for (unsigned int i = 0; i < e; i++) {
		cw[pos[i]] ^= (uint8_t)(1U + rnd() % 255U); /* never 0 */
	}
}

static void before_each(void *fixture)
{
	ARG_UNUSED(fixture);
	rs_rng = RNG_SEED;
}

ZTEST_SUITE(link_rs, NULL, NULL, before_each, NULL, NULL);

static void random_payload(uint8_t *cw)
{
	for (int i = 0; i < RS_K; i++) {
		cw[i] = (uint8_t)rnd();
	}
}

ZTEST(link_rs, test_clean_codeword)
{
	uint8_t cw[RS_N], data[RS_K];

	rs_init();
	random_payload(cw);
	memcpy(data, cw, RS_K);
	rs_encode(cw);

	zassert_equal(rs_decode(cw), 0, "a clean codeword must decode with 0 corrections");
	zassert_mem_equal(cw, data, RS_K, "payload changed on a clean decode");
}

/* Every error count up to the guaranteed t = RS_NROOTS / 2 = 16 must be corrected exactly. */
ZTEST(link_rs, test_corrects_up_to_t_errors)
{
	rs_init();
	for (unsigned int e = 0; e <= RS_NROOTS / 2U; e++) {
		for (int trial = 0; trial < 5; trial++) {
			uint8_t cw[RS_N], data[RS_K];
			int ret;

			random_payload(cw);
			memcpy(data, cw, RS_K);
			rs_encode(cw);
			corrupt(cw, e);

			ret = rs_decode(cw);
			zassert_equal(ret, (int)e, "%u errors: rs_decode() returned %d", e, ret);
			zassert_mem_equal(cw, data, RS_K, "%u errors: payload not fully restored",
					  e);
		}
	}
}

/*
 * More errors than the code guarantees: almost always -1 (uncorrectable). A
 * decode to the wrong valid codeword instead of -1 is possible in principle
 * for an adversarial error pattern, but not for random ones at this count;
 * this guards against a decoder that silently "corrects" past its limit.
 */
ZTEST(link_rs, test_detects_uncorrectable)
{
	rs_init();
	for (int trial = 0; trial < 20; trial++) {
		uint8_t cw[RS_N], data[RS_K];

		random_payload(cw);
		memcpy(data, cw, RS_K);
		rs_encode(cw);
		corrupt(cw, RS_NROOTS / 2U + 1U);

		zassert_equal(rs_decode(cw), -1,
			      "trial %d: %u errors (one past the guarantee) was not flagged",
			      trial, RS_NROOTS / 2U + 1U);
	}
}
