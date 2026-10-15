/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * VITA 49 link-efficient complex payloads: I then Q per sample, 8, 12 or 16
 * bits per item, packed MSB first across 32-bit words (CPU order; the stream
 * code swaps to network order).
 */

#ifndef STREAM_IQ_PACK_H_
#define STREAM_IQ_PACK_H_

#include <stddef.h>
#include <stdint.h>

#include "vrt_rx.h"

/* Supported item sizes. */
static inline int iq_bits_valid(unsigned int bits)
{
	return bits == 8U || bits == 12U || bits == 16U;
}

/*
 * Samples per packet must fill whole words without a padding gap a whole item
 * wide, or the receiver would see a phantom sample: multiples of this.
 */
static inline size_t iq_sample_quantum(unsigned int bits)
{
	return bits == 12U ? 4U : bits == 8U ? 2U : 1U;
}

/* Most samples (a whole quantum) that fit max_words. */
size_t iq_samples_per_words(unsigned int bits, size_t max_words);

/*
 * Pack n samples into words. 16-bit items keep the stream's own scale (full
 * scale 512 raw, 32768 decimated); 8 and 12-bit items are scaled to their own
 * full scale (2^(bits-1)), rounded and saturated. Returns words written.
 */
size_t iq_pack(const struct esp_sdr_iq16 *s, size_t n, unsigned int bits, int32_t full_scale,
	       uint32_t *words);

/*
 * Unpack nwords into interleaved int16 I, Q (8 and 12-bit items shifted up to
 * full scale 32767, 16-bit items as they are); returns samples written, at
 * most max.
 */
size_t iq_unpack(const uint32_t *words, size_t nwords, unsigned int bits, int16_t *iq,
		 size_t max);

#endif /* STREAM_IQ_PACK_H_ */
