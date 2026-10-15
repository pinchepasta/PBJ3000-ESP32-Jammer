/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Software 802.15.4 2.4 GHz O-QPSK PHY (portable C, no Zephyr deps).
 * Assumes 4 MS/s interleaved int16 I/Q (2 samples per chip, 2 Mchip/s).
 *
 * RX: MSK-style discriminator -> hard chips -> 32-chip correlation.
 * TX: reference modulator (also used to calibrate MSK_MASK, see below).
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include <esp_sdr/ieee154_phy.h>

/* CALIBRATE: one of 0, 0x55555555, 0xAAAAAAAA, 0xFFFFFFFF.
 * Modulate a frame with ieee154_phy_modulate(), feed it to
 * ieee154_phy_rx_block() and pick the mask that decodes. Then confirm on a
 * real over-the-air capture. */
#ifndef MSK_MASK
#define MSK_MASK     0xAAAAAAAAu  /* see test_calibration_noiseless */
#endif
#define SYNC_MAX_ERR 6    /* chip errors allowed when hunting preamble */
#define DATA_MAX_ERR 10   /* chip errors allowed per data symbol */
#define MAX_PSDU     IEEE154_PHY_PSDU_MAX

static uint32_t chips_raw[16]; /* c0 = MSB */
static uint32_t chips_msk[16]; /* as seen by the discriminator */

/* The discriminator measures the phase step over one chip period, i.e. the
 * DIFFERENCE of adjacent chips (MSK), not the chip itself:
 *   d_k = c_k ^ c_(k+1) ^ parity(k)  (^ global inversion, both in MSK_MASK)
 * The 32nd bit of a symbol depends on the next symbol's first chip, so it is
 * masked out of every comparison. */
#define DMASK 0xFFFFFFFEu

/* SWAR population count: cores without a popcount instruction otherwise call libgcc. */
static inline int popcount32(uint32_t v)
{
	v = v - ((v >> 1) & 0x55555555u);
	v = (v & 0x33333333u) + ((v >> 2) & 0x33333333u);
	v = (v + (v >> 4)) & 0x0f0f0f0fu;
	return (int)((v * 0x01010101u) >> 24);
}

/* Bits set per byte, for the preamble test. */
#define P2(n) n, n + 1, n + 1, n + 2
#define P4(n) P2(n), P2(n + 1), P2(n + 1), P2(n + 2)
#define P6(n) P4(n), P4(n + 1), P4(n + 1), P4(n + 2)
static const uint8_t pop8[256] = {P6(0), P6(1), P6(1), P6(2)};

/* popcount(x) <= SYNC_MAX_ERR; noise usually fails on the first two bytes. */
static inline int sync_close(uint32_t x)
{
	unsigned int a = pop8[x & 0xffu] + pop8[(x >> 8) & 0xffu];

	if (a > SYNC_MAX_ERR) {
		return 0;
	}
	a += pop8[(x >> 16) & 0xffu];
	if (a > SYNC_MAX_ERR) {
		return 0;
	}
	return a + pop8[x >> 24] <= SYNC_MAX_ERR;
}

static inline uint32_t ror32(uint32_t v, int n)
{
	n &= 31;
	return n ? (v >> n) | (v << (32 - n)) : v;
}

void ieee154_phy_init(void)
{
	/* 11011001110000110101001000101110 */
	const uint32_t s0 = 0xD9C3522Eu;

	for (int k = 0; k < 8; k++) {
		chips_raw[k] = ror32(s0, 4 * k);            /* shift 4 chips/symbol */
		chips_raw[k + 8] = chips_raw[k] ^ 0x55555555u; /* invert odd chips */
	}
	for (int k = 0; k < 16; k++) {
		chips_msk[k] = (chips_raw[k] ^ (chips_raw[k] << 1) ^ MSK_MASK) & DMASK;
	}
}

static uint16_t crc16(const uint8_t *p, int n)
{
	uint16_t c = 0;

	while (n--) {
		c ^= *p++;
		for (int i = 0; i < 8; i++) {
			c = (c & 1) ? (c >> 1) ^ 0x8408 : (c >> 1);
		}
	}
	return c;
}

/* ---------------------------------------------------------------- RX */

enum { ST_HUNT, ST_PREAMBLE, ST_SFD2, ST_PHR, ST_PSDU };

struct trk {
	uint32_t win, nchip, anchor;
	uint8_t run, st, nib_have, nib_lo, len, pos;
	/* The header callback still wants bytes of this frame. */
	uint8_t hdr_want;
	uint16_t err_sum, nsym;
	/* Sample count at which the frame's last symbol will be decided. */
	uint32_t end_pos;
	uint8_t buf[MAX_PSDU];
};

static struct trk g_trk[2]; /* one tracker per sampling phase */
static ieee154_phy_frame_cb_t g_cb;
static void *g_user;
static ieee154_phy_hdr_cb_t g_hdr_cb;
static void *g_hdr_user;
/* Samples fed before the block, or up to the sample being decided in it. */
static uint32_t g_pos;

void ieee154_phy_set_hdr_cb(ieee154_phy_hdr_cb_t cb, void *user)
{
	g_hdr_cb = cb;
	g_hdr_user = user;
}

void ieee154_phy_set_rx_cb(ieee154_phy_frame_cb_t cb, void *user)
{
	g_cb = cb;
	g_user = user;
}

static int match(uint32_t w, int *err)
{
	int best = 0, be = 33;

	for (int k = 0; k < 16; k++) {
		int e = popcount32((w ^ chips_msk[k]) & DMASK);

		if (e < be) {
			be = e;
			best = k;
		}
	}
	*err = be;
	return best;
}

static void trk_reset(struct trk *t)
{
	t->st = ST_HUNT;
	t->run = 0;
}

/*
 * Everything but a preamble miss while hunting: t->win and t->nchip already
 * hold the new chip (ieee154_phy_rx_block() keeps the common case inline).
 */
static __attribute__((noinline)) void trk_chip_slow(struct trk *t)
{
	if (t->st == ST_HUNT) {
		if (popcount32((t->win ^ chips_msk[0]) & DMASK) <= SYNC_MAX_ERR) {
			t->run = (t->run && t->nchip - t->anchor == 32) ?
				 t->run + 1 : 1;
			t->anchor = t->nchip;
			if (t->run >= 4) {
				t->st = ST_PREAMBLE;
			}
		}
		return;
	}

	if (t->nchip - t->anchor < 32) {
		return; /* not at a symbol boundary yet */
	}
	t->anchor = t->nchip;

	int err;
	int s = match(t->win, &err);

	if (err > DATA_MAX_ERR) {
		trk_reset(t);
		return;
	}

	switch (t->st) {
	case ST_PREAMBLE:      /* more zero symbols, then SFD 0xA7 (7 then A) */
		if (s == 7) {
			t->st = ST_SFD2;
		} else if (s != 0) {
			trk_reset(t);
		}
		break;
	case ST_SFD2:
		if (s == 10) {
			t->st = ST_PHR;
			t->nib_have = 0;
			t->err_sum = 0;
			t->nsym = 0;
		} else {
			trk_reset(t);
		}
		break;
	default: {             /* PHR, then PSDU; low nibble first */
		t->err_sum += err;
		t->nsym++;
		if (!t->nib_have) {
			t->nib_lo = s;
			t->nib_have = 1;
			break;
		}
		uint8_t byte = t->nib_lo | (uint8_t)(s << 4);

		t->nib_have = 0;
		if (t->st == ST_PHR) {
			t->len = byte & 0x7F;
			if (t->len < 5) {   /* shortest legal frame is an ACK */
				trk_reset(t);
				break;
			}
			t->pos = 0;
			t->st = ST_PSDU;
			/* A symbol is 32 chips, 64 samples at 2 per chip; a byte two symbols. */
			t->end_pos = g_pos + 128U * t->len;
			t->hdr_want = g_hdr_cb != NULL &&
				      g_hdr_cb(t->buf, 0, t->len, t->end_pos, g_hdr_user);
		} else {
			t->buf[t->pos++] = byte;
			if (t->hdr_want && t->pos < t->len) {
				t->hdr_want = g_hdr_cb(t->buf, t->pos, t->len, t->end_pos,
						       g_hdr_user);
			}
			if (t->pos == t->len) {
				uint16_t fcs = t->buf[t->len - 2] |
					       (t->buf[t->len - 1] << 8);

				if (crc16(t->buf, t->len - 2) == fcs) {
					int avg = t->err_sum / t->nsym;
					int lqi = 255 - avg * 25;

					if (g_cb) {
						g_cb(t->buf, t->len,
						     lqi < 0 ? 0 : lqi, g_user);
					}
					/* Only on success: stop the sibling
					 * phase tracker delivering it twice. */
					trk_reset(t == &g_trk[0] ? &g_trk[1] : &g_trk[0]);
				}
				trk_reset(t);
			}
		}
		break;
	}
	}
}

/* DC removal state (Q8) and discriminator history */
static int32_t dc_i, dc_q, pi1, pq1, pi2, pq2;
static uint32_t n_total;

uint32_t ieee154_phy_rx_position(void)
{
	return g_pos;
}

void ieee154_phy_rx_reset(void)
{
	trk_reset(&g_trk[0]);
	trk_reset(&g_trk[1]);
	pi1 = pq1 = pi2 = pq2 = 0;
}

uint32_t ieee154_phy_chips(int sym)
{
	return chips_raw[sym & 0xF];
}

/*
 * iq: interleaved I,Q int16 at 4 MS/s. Call with consecutive blocks.
 * Even samples (from n_total) go to tracker 0, odd ones to tracker 1. The
 * state lives in locals (builds without strict aliasing would reload it
 * after every store), chip counts are derived from the sample index, and
 * only a preamble hit or a frame in progress leaves the inline path.
 */
#define RX_STEP(w_, s_, a_, t, cnt)                                                         \
	do {                                                                                 \
		int32_t xi = iq[2 * k], xq = iq[2 * k + 1], d;                               \
                                                                                             \
		/* Track DC only while idle: MSK has energy near DC. */                      \
		if (hunting) {                                                               \
			dci += (xi * 256 - dci) >> 8;                                        \
			dcq += (xq * 256 - dcq) >> 8;                                        \
		}                                                                            \
		xi -= dci >> 8;                                                              \
		xq -= dcq >> 8;                                                              \
		/* Im(x[n] * conj(x[n-2])): sign = phase step over one chip */             \
		d = xq * i2 - xi * q2;                                                       \
		i2 = i1; q2 = q1;                                                            \
		i1 = xi; q1 = xq;                                                            \
		w_ = (w_ << 1) | (uint32_t)(d > 0);                                          \
		/* Hunting: a preamble chip match; in a frame: a symbol boundary. */      \
		if ((s_) == ST_HUNT ? sync_close((w_ ^ ref) & DMASK)                        \
				    : (uint32_t)(cnt) - (a_) >= 32U) {                       \
			(t)->win = w_;                                                       \
			(t)->nchip = (cnt);                                                  \
			g_pos = n_total + (uint32_t)k + 1U;                                  \
			trk_chip_slow(t);                                                    \
			s0 = t0->st;                                                         \
			s1 = t1->st;                                                         \
			a0 = t0->anchor;                                                     \
			a1 = t1->anchor;                                                     \
			hunting = s0 == ST_HUNT && s1 == ST_HUNT;                            \
		}                                                                            \
	} while (0)

void ieee154_phy_rx_block(const int16_t *iq, size_t n)
{
	int32_t dci = dc_i, dcq = dc_q, i1 = pi1, q1 = pq1, i2 = pi2, q2 = pq2;
	struct trk *t0 = &g_trk[n_total & 1], *t1 = &g_trk[(n_total + 1) & 1];
	uint32_t w0 = t0->win, w1 = t1->win, c0 = t0->nchip, c1 = t1->nchip;
	uint8_t s0 = t0->st, s1 = t1->st;
	uint32_t a0 = t0->anchor, a1 = t1->anchor;
	int hunting = s0 == ST_HUNT && s1 == ST_HUNT;
	const uint32_t ref = chips_msk[0];
	size_t k = 0;

	g_pos = n_total;

	/* Sample k goes to t0 for even k: chip count c0 + k / 2 + 1 after it. */
	for (; k + 1 < n; k++) {
		RX_STEP(w0, s0, a0, t0, c0 + (uint32_t)k / 2U + 1U);
		k++;
		RX_STEP(w1, s1, a1, t1, c1 + (uint32_t)k / 2U + 1U);
	}
	if (k < n) {
		RX_STEP(w0, s0, a0, t0, c0 + (uint32_t)k / 2U + 1U);
	}
	t0->win = w0;
	t0->nchip = c0 + (uint32_t)(n + 1) / 2U;
	t1->win = w1;
	t1->nchip = c1 + (uint32_t)n / 2U;
	dc_i = dci; dc_q = dcq;
	pi1 = i1; pq1 = q1; pi2 = i2; pq2 = q2;
	n_total += (uint32_t)n;
}

/* ---------------------------------------------------------------- TX */

/* Half-sine pulse over 2 chips = 4 samples (x1000) */
static const int16_t half_sine[4] = { 0, 707, 1000, 707 };

/*
 * Modulate SHR + PHR + psdu (psdu must already include the 2-byte FCS).
 * Returns number of complex samples written to out (interleaved I,Q),
 * or 0 if out is too small. Even chips -> I, odd chips -> Q, Q delayed
 * by one chip (2 samples), which is the O-QPSK offset.
 */
size_t ieee154_phy_modulate(const uint8_t *psdu, uint8_t len, int16_t amp, int16_t *out,
			    size_t max_samples)
{
	size_t nbytes = 6 + len;
	size_t nchips = nbytes * 2 * 32;
	size_t ns = 2 * nchips + 4;

	if (ns > max_samples) {
		return 0;
	}
	memset(out, 0, ns * 2 * sizeof(int16_t));

	size_t k = 0;

	for (size_t b = 0; b < nbytes; b++) {
		uint8_t v = b < 4 ? 0x00 : b == 4 ? 0xA7 :
			    b == 5 ? len : psdu[b - 6];

		for (int nib = 0; nib < 2; nib++) {
			uint32_t w = chips_raw[(v >> (4 * nib)) & 0xF];

			for (int c = 0; c < 32; c++, k++) {
				int a = ((w >> (31 - c)) & 1) ? amp : -amp;
				int16_t *rail = out + (k & 1); /* I or Q */

				for (int m = 0; m < 4; m++) {
					rail[2 * (2 * k + m)] +=
						(int16_t)(a * half_sine[m] / 1000);
				}
			}
		}
	}
	return ns;
}

/* Append FCS (little endian) to buf[0..n-1]; returns new length. */
size_t ieee154_phy_append_fcs(uint8_t *buf, size_t n)
{
	uint16_t c = crc16(buf, n);

	buf[n] = c & 0xFF;
	buf[n + 1] = c >> 8;
	return n + 2;
}
