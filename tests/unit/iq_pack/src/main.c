/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Unit test of apps/stream/src/iq_pack.c: pack/unpack round trip at
 * 8, 12 and 16 bits. 16-bit items are a lossless passthrough; 8 and
 * 12-bit items are quantized and always unpacked back out at full int16
 * scale (32767), not at the original full_scale, so this checks
 * implementation-agnostic properties (exact zero, correct sign and
 * magnitude at the extremes, monotonicity) rather than one specific
 * rounding convention. Runs on native_sim.
 */
#include <string.h>

#include <zephyr/ztest.h>

#include "iq_pack.h"

ZTEST_SUITE(iq_pack, NULL, NULL, NULL, NULL, NULL);

static void pack1(unsigned int bits, int32_t full_scale, int16_t i, int16_t q, int16_t *out_i,
		   int16_t *out_q)
{
	/* A whole quantum of samples so the block/tail packing paths both see real data;
	 * only sample 0 is examined (item() has no cross-sample dependency).
	 */
	struct esp_sdr_iq16 s[4] = {{.i = i, .q = q}};
	uint32_t words[8];
	int16_t iq[8];

	size_t n = iq_sample_quantum(bits);
	size_t w = iq_pack(s, n, bits, full_scale, words);
	size_t got = iq_unpack(words, w, bits, iq, n);

	zassert_equal(got, n, "unpacked %zu of %zu samples", got, n);
	*out_i = iq[0];
	*out_q = iq[1];
}

ZTEST(iq_pack, test_16bit_is_lossless)
{
	static const int16_t values[] = {0, 1, -1, 100, -100, 32767, -32768, 12345, -12345};

	ARRAY_FOR_EACH(values, k) {
		struct esp_sdr_iq16 s[1] = {{.i = values[k], .q = (int16_t)(values[k] ^ 0x5a5a)}};
		uint32_t word;
		int16_t iq[2];

		zassert_equal(iq_pack(s, 1, 16, 32768, &word), 1);
		zassert_equal(iq_unpack(&word, 1, 16, iq, 1), 1);
		zassert_equal(iq[0], s[0].i, "I: packed %d, got back %d", s[0].i, iq[0]);
		zassert_equal(iq[1], s[0].q, "Q: packed %d, got back %d", s[0].q, iq[1]);
	}
}

ZTEST(iq_pack, test_zero_packs_to_zero)
{
	static const unsigned int bit_opts[] = {8, 12, 16};
	static const int32_t scales[] = {512, 32768};

	ARRAY_FOR_EACH(bit_opts, b) {
		ARRAY_FOR_EACH(scales, f) {
			int16_t oi, oq;

			pack1(bit_opts[b], scales[f], 0, 0, &oi, &oq);
			zassert_equal(oi, 0, "%u bit, full_scale %d: I %d, expected 0",
				      bit_opts[b], scales[f], oi);
			zassert_equal(oq, 0, "%u bit, full_scale %d: Q %d, expected 0",
				      bit_opts[b], scales[f], oq);
		}
	}
}

/*
 * Any correct quantizer is monotonic: more input never decodes to less
 * output. The sweep's endpoints are the input extremes, so checking their
 * sign there also confirms the quantizer does not, say, wrap a large
 * positive input around to a negative output.
 *
 * It does NOT check the positive extreme lands near +32767: full_scale
 * represents a two's complement peak (-full_scale..full_scale-1), so the
 * true positive extreme (full_scale-1) is reachable but one part in
 * full_scale short of it, landing near +32767 * (full_scale-1)/full_scale,
 * not at +32767 itself (confirmed by hand for 8 and 12 bit at full_scale
 * 512: 32512 and 32704). The negative extreme (-full_scale) has no such
 * asymmetry and does land exactly on -32768.
 */
ZTEST(iq_pack, test_monotonic_and_signed_correctly)
{
	static const unsigned int bit_opts[] = {8, 12};
	int32_t full_scale = 512;

	ARRAY_FOR_EACH(bit_opts, b) {
		unsigned int bits = bit_opts[b];
		int16_t prev = 0, first = 0, last = 0;
		bool have_prev = false;
		int32_t last_i = 0;

		for (int32_t i = -full_scale; i < full_scale; i += 7) {
			int16_t oi, oq;

			pack1(bits, full_scale, (int16_t)i, 0, &oi, &oq);
			if (have_prev) {
				zassert_true(oi >= prev,
					     "%u bit: input %d decoded to %d, less than input %d's %d",
					     bits, i, oi, i - 7, prev);
			} else {
				first = oi;
			}
			prev = oi;
			last = oi;
			last_i = i;
			have_prev = true;
		}
		zassert_equal(first, -32768, "%u bit: input %d (the minimum) decoded to %d",
			      bits, -full_scale, first);
		zassert_true(last > 0, "%u bit: input %d (near the maximum) decoded to %d", bits,
			     last_i, last);
	}
}

ZTEST(iq_pack, test_samples_per_words_quantum_and_budget)
{
	static const unsigned int bit_opts[] = {8, 12, 16};

	ARRAY_FOR_EACH(bit_opts, b) {
		unsigned int bits = bit_opts[b];
		size_t quantum = iq_sample_quantum(bits);
		size_t max_words = 128;
		size_t n = iq_samples_per_words(bits, max_words);

		zassert_equal(n % quantum, 0, "%u bit: %zu samples is not a multiple of %zu",
			      bits, n, quantum);
		zassert_true(n * 2U * bits <= max_words * 32U,
			     "%u bit: %zu samples need more than %zu words", bits, n, max_words);
		/* One more quantum must not fit (n is the most that fits). */
		zassert_true((n + quantum) * 2U * bits > max_words * 32U,
			     "%u bit: %zu samples fit, but is not the most that do", bits, n);
	}
}

/* n not a multiple of the quantum: the tail (bit-accumulator) path alone, or a full
 * block plus a tail. Every sample must still come back, in order, matching a clean
 * monotonic ramp.
 */
ZTEST(iq_pack, test_partial_quantum_tail)
{
	static const unsigned int bit_opts[] = {8, 12};

	ARRAY_FOR_EACH(bit_opts, b) {
		unsigned int bits = bit_opts[b];
		size_t quantum = iq_sample_quantum(bits);

		for (size_t n = 1; n <= 2U * quantum + 1U; n++) {
			struct esp_sdr_iq16 s[2 * 4 + 1];
			uint32_t words[16];
			int16_t iq[2 * (2 * 4 + 1)];
			size_t w, got;

			for (size_t k = 0; k < n; k++) {
				s[k].i = (int16_t)(16 * (int)k - 64);
				s[k].q = (int16_t)(-8 * (int)k);
			}
			w = iq_pack(s, n, bits, 512, words);
			got = iq_unpack(words, w, bits, iq, n);
			zassert_equal(got, n, "%u bit, n=%zu: unpacked %zu samples", bits, n, got);
			/* I rises and Q falls by steps every width resolves; I starts negative. */
			zassert_true(iq[0] < 0, "%u bit, n=%zu: first I %d not negative", bits, n,
				     iq[0]);
			for (size_t k = 1; k < n; k++) {
				zassert_true(iq[2 * k] > iq[2 * (k - 1)],
					     "%u bit, n=%zu: I of sample %zu (%d) <= sample %zu (%d)",
					     bits, n, k, iq[2 * k], k - 1, iq[2 * (k - 1)]);
				zassert_true(iq[2 * k + 1] < iq[2 * k - 1],
					     "%u bit, n=%zu: Q of sample %zu (%d) >= sample %zu (%d)",
					     bits, n, k, iq[2 * k + 1], k - 1, iq[2 * k - 1]);
			}
		}
	}
}
