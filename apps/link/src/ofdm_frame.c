/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Link frames on the OFDM PHY, see ofdm_frame.h. The payload capacity (whole
 * bytes of the payload symbols) is split into the fewest RS units of at most
 * 255 bytes, all the same length: shortened codes, the missing data bytes
 * virtual zeros in front. Frame byte j belongs to unit j % units.
 */

#include <string.h>

#include "ofdm_frame.h"
#include "qam.h"
#include "rs.h"

#define HDR_VERSION 0xa5U

struct layout {
	unsigned int units, len, k;
	size_t cap, user;
};

static bool layout_of(const struct ofdm_ctx *ctx, enum ofdm_mod mod, struct layout *l)
{
	if (mod >= OFDM_MODS) {
		return false;
	}
	l->cap = ofdm_frame_bits(ctx, mod) / 8U;
	/* The work buffer holds the payload bits rounded up to bytes. */
	if (l->cap >= OFDM_FRAME_CAP_MAX) {
		return false;
	}
	l->units = (unsigned int)((l->cap + RS_N - 1U) / RS_N);
	if (l->units == 0U) {
		return false;
	}
	l->len = (unsigned int)(l->cap / l->units);
	/* A unit needs its parity and some data; the user data its CRC. */
	if (l->len < RS_NROOTS + 8U) {
		return false;
	}
	l->k = l->len - RS_NROOTS;
	l->user = (size_t)l->units * l->k - 4U;
	return true;
}

size_t ofdm_frame_user_bytes(const struct ofdm_ctx *ctx, enum ofdm_mod mod)
{
	struct layout l;

	return layout_of(ctx, mod, &l) ? l.user : 0U;
}

static void hdr_pack(const struct ofdm_fhdr *h, uint8_t b[8])
{
	uint16_t crc;

	b[0] = (uint8_t)(((h->type & 1U) << 7) | ((unsigned int)h->mod << 4));
	b[1] = h->dst;
	b[2] = h->src;
	b[3] = (uint8_t)h->seq;
	b[4] = (uint8_t)(h->seq >> 8);
	b[5] = HDR_VERSION;
	crc = qam_crc16(b, 6);
	b[6] = (uint8_t)crc;
	b[7] = (uint8_t)(crc >> 8);
}

static bool hdr_unpack(const uint8_t b[8], struct ofdm_fhdr *h)
{
	if (b[5] != HDR_VERSION || qam_crc16(b, 6) != (uint16_t)(b[6] | (b[7] << 8)) ||
	    (b[0] & 0x0fU) != 0U || ((b[0] >> 4) & 7U) >= OFDM_MODS) {
		return false;
	}
	h->type = b[0] >> 7;
	h->mod = (enum ofdm_mod)((b[0] >> 4) & 7U);
	h->dst = b[1];
	h->src = b[2];
	h->seq = (uint16_t)(b[3] | (b[4] << 8));
	return true;
}

/* Unit u's byte p as sent (data, then parity), inside its 255 byte buffer. */
static inline uint8_t *unit_byte(struct ofdm_fwork *wk, const struct layout *l, unsigned int u,
				 unsigned int p)
{
	return &wk->units[u][RS_N - l->len + p];
}

size_t ofdm_frame_build(const struct ofdm_ctx *ctx, struct ofdm_txbuf *tb, struct ofdm_fwork *wk,
			const struct ofdm_fhdr *h, const uint8_t *data, uint32_t *out)
{
	struct layout l;
	uint8_t hb[8];
	uint32_t crc;
	size_t j;

	if (!layout_of(ctx, h->mod, &l)) {
		return 0;
	}
	crc = qam_crc32(data, l.user);
	/* User data, then its CRC (little endian), over the units' data bytes. */
	for (unsigned int u = 0; u < l.units; u++) {
		memset(wk->units[u], 0, RS_N);
	}
	for (size_t d = 0; d < l.user + 4U; d++) {
		uint8_t v = d < l.user ? data[d] : (uint8_t)(crc >> (8U * (d - l.user)));

		*unit_byte(wk, &l, (unsigned int)(d / l.k), (unsigned int)(d % l.k)) = v;
	}
	for (unsigned int u = 0; u < l.units; u++) {
		rs_encode(wk->units[u]);
	}
	for (j = 0; j < (size_t)l.units * l.len; j++) {
		wk->bytes[j] = *unit_byte(wk, &l, (unsigned int)(j % l.units),
					  (unsigned int)(j / l.units));
	}
	memset(&wk->bytes[j], 0, l.cap - j + 1U);
	hdr_pack(h, hb);
	return ofdm_tx_build(ctx, tb, h->mod, hb, wk->bytes, out);
}

size_t ofdm_frame_build_hdr(const struct ofdm_ctx *ctx, struct ofdm_txbuf *tb,
			    const struct ofdm_fhdr *h, uint32_t *out)
{
	uint8_t hb[8];

	hdr_pack(h, hb);
	return ofdm_tx_build(ctx, tb, OFDM_BPSK, hb, NULL, out);
}

int ofdm_frame_begin_hdr(struct ofdm_ctx *ctx, const uint32_t *words, size_t n,
			 struct ofdm_fhdr *h, struct ofdm_rx_info *info)
{
	uint8_t hb[8];

	if (ofdm_rx_begin_period(ctx, words, n, ofdm_hdr_rsamples(ctx), hb, info) != 0) {
		return -1;
	}
	return hdr_unpack(hb, h) ? 0 : -2;
}

int ofdm_frame_begin(struct ofdm_ctx *ctx, const uint32_t *words, size_t n, struct ofdm_fhdr *h,
		     struct ofdm_rx_info *info)
{
	uint8_t hb[8];

	if (ofdm_rx_begin(ctx, words, n, hb, info) != 0) {
		return -1;
	}
	return hdr_unpack(hb, h) ? 0 : -2;
}

int ofdm_frame_finish(struct ofdm_ctx *ctx, const uint32_t *words, const struct ofdm_fhdr *h,
		      struct ofdm_fwork *wk, uint8_t *data, int *corrected,
		      struct ofdm_rx_info *info)
{
	struct layout l;

	if (!layout_of(ctx, h->mod, &l)) {
		*corrected = 0;
		return -3;
	}
	ofdm_rx_finish(ctx, words, h->mod, wk->bytes, info);
	return ofdm_frame_fec(ctx, h, wk, data, corrected);
}

int ofdm_frame_fec_units(const struct ofdm_ctx *ctx, const struct ofdm_fhdr *h,
			 struct ofdm_fwork *wk, unsigned int u0, unsigned int u1, int *corrected)
{
	struct layout l;

	*corrected = 0;
	if (!layout_of(ctx, h->mod, &l)) {
		return -3;
	}
	u1 = u1 > l.units ? l.units : u1;
	for (unsigned int u = u0; u < u1; u++) {
		/* Decoded on the stack: the work memory may be PSRAM. */
		uint8_t cw[RS_N];
		int r;

		memset(cw, 0, RS_N - l.len);
		for (unsigned int p = 0; p < l.len; p++) {
			cw[RS_N - l.len + p] = wk->bytes[(size_t)p * l.units + u];
		}
		r = rs_decode(cw);
		if (r < 0) {
			return -3;
		}
		/* A miscorrection may land in the virtual zeros. */
		for (unsigned int i = 0; i < RS_N - l.len; i++) {
			if (cw[i] != 0U) {
				return -3;
			}
		}
		memcpy(wk->units[u], cw, RS_N);
		*corrected += r;
	}
	return 0;
}

unsigned int ofdm_frame_units(const struct ofdm_ctx *ctx, enum ofdm_mod mod)
{
	struct layout l;

	return layout_of(ctx, mod, &l) ? l.units : 0U;
}

int ofdm_frame_check(const struct ofdm_ctx *ctx, const struct ofdm_fhdr *h, struct ofdm_fwork *wk,
		     uint8_t *data)
{
	struct layout l;
	uint32_t crc = 0;

	if (!layout_of(ctx, h->mod, &l)) {
		return -3;
	}
	for (size_t d = 0; d < l.user + 4U; d++) {
		uint8_t v = *unit_byte(wk, &l, (unsigned int)(d / l.k), (unsigned int)(d % l.k));

		if (d < l.user) {
			data[d] = v;
		} else {
			crc |= (uint32_t)v << (8U * (d - l.user));
		}
	}
	return qam_crc32(data, l.user) == crc ? 0 : -3;
}

int ofdm_frame_fec(const struct ofdm_ctx *ctx, const struct ofdm_fhdr *h, struct ofdm_fwork *wk,
		   uint8_t *data, int *corrected)
{
	int ret = ofdm_frame_fec_units(ctx, h, wk, 0, ~0U, corrected);

	return ret != 0 ? ret : ofdm_frame_check(ctx, h, wk, data);
}
