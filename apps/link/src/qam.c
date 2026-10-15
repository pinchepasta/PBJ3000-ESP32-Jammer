/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QAM physical layer, see qam.h. Transmit sums a pulse table over the span
 * (one load and add per span symbol and sample, I and Q packed in one
 * word). Receive: Schmidl-Cox search on every QAM_SPS-th sample, fine timing
 * against the known preamble, frequency offset from the two preamble
 * halves, a 7 tap symbol spaced equalizer solved by least squares over the
 * preamble, then a decision directed phase loop over header and payload.
 */

#include <math.h>
#include <string.h>

#include "hamming.h"
#include "qam.h"

#ifdef QAM_DEBUG
#include <stdio.h>
#include <stdlib.h>
#endif

#define PI     3.14159265358979323846f
#define TWO_PI 6.28318530717958647692f
#define MIN_SZ(a, b) ((a) < (b) ? (a) : (b))
#define MIN_INT(a, b) ((a) < (b) ? (a) : (b))
#define MAX_INT(a, b) ((a) > (b) ? (a) : (b))
/* Samples compared between two frame copies at a wrap. */
#define WRAP_OVERLAP 512
#define TAPS   QAM_EQ_TAPS
#define HALF   (TAPS / 2)
#define UNK    (TAPS + 1) /* taps and a bias */
/* Phase loop gains (per symbol). */
#ifndef QAM_LMS_MU
#define QAM_LMS_MU 0.04f
#endif
/* LMS update on every n-th symbol (power of two), with n times the step. */
#ifndef QAM_LMS_EVERY
#define QAM_LMS_EVERY 4
#endif

#ifndef QAM_PLL_KP
#define QAM_PLL_KP 0.05f
#define QAM_PLL_KI 0.0008f
#endif

/* Hot loops in IRAM on the target: code and literals then skip the flash cache. */
#if defined(__ZEPHYR__)
#include <esp_attr.h>
#define QAM_HOT IRAM_ATTR __attribute__((noinline))
#else
#define QAM_HOT
#endif
/* Least preamble metric (normalized correlation, squared) for a detection. */
#define METRIC_MIN 0.3f
/* Fine timing search in samples: the coarse plateau widens when the symbols
 * before the preamble happen to repeat its end.
 */
#define FINE_RANGE QAM_FINE_RANGE

struct cf {
	float re, im;
};

static inline struct cf cmul(struct cf a, struct cf b)
{
	return (struct cf){a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re};
}

uint32_t (*qam_clock)(void);

static inline uint32_t clk(void)
{
	return qam_clock != NULL ? qam_clock() : 0U;
}

#define PROF(i)                                                                            \
	do {                                                                               \
		uint32_t t_ = clk();                                                       \
		info->prof[i] = t_ - prof_t;                                               \
		prof_t = t_;                                                               \
	} while (0)

/* Least normalized cross-correlation with the preamble: tones pass Schmidl-Cox. */
#define XCORR_MIN 0.25f

/* Preamble half: 64 QPSK points (0..3) from a 16-bit Galois LFSR. */
static uint8_t preamble[QAM_PRE_HALF];
static bool preamble_ready;

static void preamble_init(void)
{
	uint16_t lfsr = 0xace1;

	if (preamble_ready) {
		return;
	}
	for (int k = 0; k < QAM_PRE_HALF; k++) {
		uint8_t p = 0;

		for (int b = 0; b < 2; b++) {
			unsigned int bit = lfsr & 1U;

			lfsr = (uint16_t)((lfsr >> 1) ^ (bit ? 0xb400U : 0U));
			p = (uint8_t)((p << 1) | bit);
		}
		preamble[k] = p;
	}
	preamble_ready = true;
}

/* QPSK point p: I bit 1, Q bit 0; bit 1 = positive. */
static inline struct cf qpsk_point(unsigned int p)
{
	const float a = 0.70710678f;

	return (struct cf){(p & 2U) ? a : -a, (p & 1U) ? a : -a};
}

unsigned int qam_bits(enum qam_mod mod)
{
	return 2U * ((unsigned int)mod + 1U);
}

/* Levels per axis, and the level spacing half-step for unit average power. */
static unsigned int axis_levels(enum qam_mod mod)
{
	return 1U << ((unsigned int)mod + 1U);
}

static float axis_scale(unsigned int k)
{
	return sqrtf(3.0f / (2.0f * (float)(k * k - 1U)));
}

static const uint16_t unit_coded[QAM_FECS] = {RS_N, HAM_N * 15, QAM_UNIT};
static const uint16_t unit_data[QAM_FECS] = {RS_K, HAM_K * 15, QAM_UNIT};

unsigned int qam_unit_coded(enum qam_fec fec)
{
	return fec < QAM_FECS ? unit_coded[fec] : 0;
}

unsigned int qam_unit_data(enum qam_fec fec)
{
	return fec < QAM_FECS ? unit_data[fec] : 0;
}

/* Offset of data byte d (0 .. qam_unit_data()) inside its coded unit. */
static inline unsigned int data_offset(enum qam_fec fec, unsigned int d)
{
	return fec == QAM_FEC_HAMMING ? d / HAM_K * HAM_N + d % HAM_K : d;
}

static size_t payload_syms(enum qam_mod mod, enum qam_fec fec, unsigned int ncw)
{
	unsigned int bps = qam_bits(mod);

	return (ncw * qam_unit_coded(fec) * 8U + bps - 1U) / bps;
}

size_t qam_frame_samples(enum qam_mod mod, enum qam_fec fec, unsigned int ncw)
{
	size_t n;

	if (mod >= QAM_MODS || fec >= QAM_FECS) {
		return 0;
	}
	n = (QAM_PRE_SYMS + QAM_HDR_SYMS + payload_syms(mod, fec, ncw)) * QAM_SPS;
	return n <= QAM_FRAME_MAX_SAMPLES ? n : 0;
}

unsigned int qam_ncw_max(enum qam_mod mod, enum qam_fec fec)
{
	unsigned int ncw = 0;

	while (ncw < QAM_NCW_MAX && qam_frame_samples(mod, fec, ncw + 1U) != 0) {
		ncw++;
	}
	return ncw;
}

uint16_t qam_crc16(const uint8_t *p, size_t n)
{
	uint16_t crc = 0xffff;

	for (size_t i = 0; i < n; i++) {
		crc ^= (uint16_t)(p[i] << 8);
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

static const uint32_t crc32_nibble[16] = {
	0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4,
	0x4db26158, 0x5005713c, 0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
	0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
};

static uint32_t crc32_update(uint32_t crc, const uint8_t *p, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		crc ^= p[i];
		crc = (crc >> 4) ^ crc32_nibble[crc & 15U];
		crc = (crc >> 4) ^ crc32_nibble[crc & 15U];
	}
	return crc;
}

uint32_t qam_crc32(const uint8_t *p, size_t n)
{
	return ~crc32_update(0xffffffffU, p, n);
}

static void hdr_pack(const struct qam_hdr *h, uint8_t b[QAM_HDR_BYTES])
{
	uint16_t crc;

	b[0] = (uint8_t)((h->pair ? 0x80U : 0U) | ((h->type & 1U) << 6) | ((h->mod & 7U) << 3) |
			 ((h->fec & 3U) << 1) | (h->full ? 1U : 0U));
	b[1] = h->dst;
	b[2] = h->src;
	b[3] = h->ncw;
	b[4] = (uint8_t)h->seq;
	b[5] = (uint8_t)(h->seq >> 8);
	crc = qam_crc16(b, 6);
	b[6] = (uint8_t)crc;
	b[7] = (uint8_t)(crc >> 8);
}

static bool hdr_unpack(const uint8_t b[QAM_HDR_BYTES], struct qam_hdr *h)
{
	if (qam_crc16(b, 6) != (uint16_t)(b[6] | (b[7] << 8))) {
		return false;
	}
	h->pair = b[0] >> 7;
	h->type = (b[0] >> 6) & 1U;
	h->mod = (b[0] >> 3) & 7U;
	h->fec = (b[0] >> 1) & 3U;
	h->full = b[0] & 1U;
	h->ncw = b[3];
	h->dst = b[1];
	h->src = b[2];
	h->seq = (uint16_t)(b[4] | (b[5] << 8));
	return true;
}

/* Raised cosine at t symbols. */
static float rc(float t)
{
	const float b = QAM_RC_BETA;
	float x = 2.0f * b * t, s;

	if (fabsf(t) < 1e-6f) {
		return 1.0f;
	}
	s = sinf(PI * t) / (PI * t);
	if (fabsf(1.0f - x * x) < 1e-5f) {
		return PI / 4.0f * sinf(PI / (2.0f * b)) /
		       (PI / (2.0f * b));
	}
	return s * cosf(PI * b * t) / (1.0f - x * x);
}

void qam_tx_init(struct qam_tx *tx, enum qam_mod mod, int amp)
{
	unsigned int k = axis_levels(mod);
	float c = axis_scale(k), lev[QAM_LEVELS_MAX + 2];

	preamble_init();
	for (unsigned int i = 0; i < k; i++) {
		lev[i] = c * (float)(2 * (int)i - (int)(k - 1U));
	}
	lev[QAM_LEVEL_QPSK] = -0.70710678f;
	lev[QAM_LEVEL_QPSK + 1] = 0.70710678f;
	memset(tx->table, 0, sizeof(tx->table));
	for (int s = 0; s < QAM_SPAN; s++) {
		for (int ph = 0; ph < QAM_SPS; ph++) {
			/* Span position s holds symbol k + 4 - s at output symbol k. */
			float g = 16.0f * (float)amp *
				  rc((float)(s - QAM_SPAN / 2) + (float)ph / (float)QAM_SPS);

			for (unsigned int i = 0; i < QAM_LEVELS_MAX + 2; i++) {
				if (i < k || i >= QAM_LEVEL_QPSK) {
					int32_t v = (int32_t)lrintf(g * lev[i]);

					tx->table[s][i][ph / 2] += ph % 2 ? v * 65536 : v;
				}
			}
		}
	}
	tx->amp = amp;
	tx->mod = mod;
}

/* Bits to axis level, Gray per axis. */
static inline uint8_t gray_decode(unsigned int g)
{
	unsigned int b = g;

	for (unsigned int s = 1; s < 8; s <<= 1) {
		b ^= b >> s;
	}
	return (uint8_t)b;
}

/*
 * Units of a frame: ncw whole ones, or with "full" as many as fit plus a
 * short last unit (RS shortened by virtual leading zeros, fewer Hamming
 * blocks, fewer uncoded bytes).
 */
struct layout {
	unsigned int ncw, un, uk;
	unsigned int last_coded, last_data, last_start;
	size_t coded, user;
};

/* Longest frame: half of it for each frame of a pair. */
static inline size_t frame_max(bool pair)
{
	return pair ? QAM_FRAME_MAX_SAMPLES / 2 / QAM_SPS * QAM_SPS : QAM_FRAME_MAX_SAMPLES;
}

static bool layout_of(enum qam_mod mod, enum qam_fec fec, unsigned int ncw, bool full, bool pair,
		      struct layout *l)
{
	if (mod >= QAM_MODS || fec >= QAM_FECS) {
		return false;
	}
	l->un = qam_unit_coded(fec);
	l->uk = qam_unit_data(fec);
	if (full) {
		size_t bytes = (size_t)(frame_max(pair) / QAM_SPS - QAM_PRE_SYMS - QAM_HDR_SYMS) *
			       qam_bits(mod) / 8U;
		unsigned int r;

		ncw = (unsigned int)(bytes / l->un);
		r = (unsigned int)(bytes - (size_t)ncw * l->un);
		if (fec == QAM_FEC_HAMMING) {
			r -= r % HAM_N;
		}
		l->last_coded = l->un;
		l->last_data = l->uk;
		l->last_start = 0;
		if ((fec == QAM_FEC_RS && r > RS_NROOTS) || (fec != QAM_FEC_RS && r > 0)) {
			ncw++;
			l->last_coded = r;
			l->last_data = fec == QAM_FEC_RS        ? r - RS_NROOTS
				       : fec == QAM_FEC_HAMMING ? r / HAM_N * HAM_K
								: r;
			l->last_start = fec == QAM_FEC_RS ? RS_N - r : 0;
		}
	} else {
		l->last_coded = l->un;
		l->last_data = l->uk;
		l->last_start = 0;
	}
	if (ncw > QAM_NCW_MAX) {
		return false;
	}
	l->ncw = ncw;
	l->coded = ncw == 0 ? 0 : (size_t)(ncw - 1U) * l->un + l->last_coded;
	l->user = ncw == 0 ? 0 : (size_t)(ncw - 1U) * l->uk + l->last_data - QAM_CRC_BYTES;
	if (ncw > 0 && (size_t)(ncw - 1U) * l->uk + l->last_data <= QAM_CRC_BYTES) {
		return false;
	}
	return (QAM_PRE_SYMS + QAM_HDR_SYMS + (l->coded * 8U + qam_bits(mod) - 1U) / qam_bits(mod)) *
		       QAM_SPS <=
	       frame_max(pair);
}

static inline bool hdr_layout(const struct qam_hdr *h, struct layout *l)
{
	return layout_of((enum qam_mod)h->mod, (enum qam_fec)h->fec, h->ncw, h->full, h->pair, l);
}

/* Coded unit c's length, and where its on-air bytes start in its slot. */
static inline unsigned int unit_len(const struct layout *l, unsigned int c)
{
	return c + 1U == l->ncw ? l->last_coded : l->un;
}

static inline unsigned int unit_start(const struct layout *l, unsigned int c)
{
	return c + 1U == l->ncw ? l->last_start : 0U;
}

/* Slot offset of data byte d of the frame. */
static inline size_t data_slot(const struct layout *l, enum qam_fec fec, size_t d)
{
	unsigned int c = (unsigned int)(d / l->uk), i;

	if (c >= l->ncw) {
		c = l->ncw - 1U;
	}
	i = (unsigned int)(d - (size_t)c * l->uk);
	return (size_t)c * QAM_UNIT + unit_start(l, c) + data_offset(fec, i);
}

/* Next interleaved position: units in turn, skipping a short last unit once it ends. */
static inline void interleave_next(const struct layout *l, unsigned int *c, unsigned int *pos)
{
	if (++*c + 1U == l->ncw && *pos >= l->last_coded) {
		++*c;
	}
	if (*c >= l->ncw) {
		*c = 0;
		++*pos;
		if (l->ncw == 1U && *pos >= l->last_coded) {
			return;
		}
	}
}

size_t qam_hdr_frame_samples(const struct qam_hdr *h)
{
	struct layout l;

	if (!hdr_layout(h, &l)) {
		return 0;
	}
	return (QAM_PRE_SYMS + QAM_HDR_SYMS + (l.coded * 8U + qam_bits((enum qam_mod)h->mod) - 1U) /
						     qam_bits((enum qam_mod)h->mod)) *
	       QAM_SPS;
}

size_t qam_hdr_payload_bytes(const struct qam_hdr *h)
{
	struct layout l;

	return hdr_layout(h, &l) ? l.user : 0;
}

size_t qam_full_payload_bytes(enum qam_mod mod, enum qam_fec fec)
{
	struct layout l;

	return layout_of(mod, fec, 0, true, false, &l) ? l.user : 0;
}

/* Map one frame's symbols to tx->sym_* from @p ks0; returns its length in samples, 0 if bad. */
static size_t map_frame(struct qam_tx *tx, const struct qam_hdr *hdr, const uint8_t *data,
			size_t ks0)
{
	enum qam_mod mod = (enum qam_mod)hdr->mod;
	enum qam_fec fec = (enum qam_fec)hdr->fec;
	unsigned int q = (unsigned int)mod + 1U, k = 1U << q;
	unsigned int bps = 2U * q, nbits = 0, j = 0;
	size_t len = qam_hdr_frame_samples(hdr), ks;
	uint8_t hb[QAM_HDR_BYTES];
	uint32_t acc = 0;
	struct layout l;
	unsigned int ncw;
	size_t user;

	if (len == 0 || !hdr_layout(hdr, &l) || hdr->mod != tx->mod) {
		return 0;
	}
	ncw = l.ncw;
	user = l.user;
	if (ncw > 0 && data == NULL && user > 0) {
		return 0;
	}

	hdr_pack(hdr, hb);
	for (int i = 0; i < QAM_PRE_SYMS + QAM_HDR_SYMS; i++) {
		/* QPSK point p: I bit 1, Q bit 0. */
		unsigned int p = i < QAM_PRE_SYMS ? preamble[i % QAM_PRE_HALF]
						  : (hb[(i - QAM_PRE_SYMS) / 4] >>
						     (6 - 2 * ((i - QAM_PRE_SYMS) % 4))) & 3U;

		tx->sym_i[ks0 + i] = (uint8_t)(QAM_LEVEL_QPSK + (p >> 1));
		tx->sym_q[ks0 + i] = (uint8_t)(QAM_LEVEL_QPSK + (p & 1U));
	}

	if (ncw > 0) {
		uint32_t crc = ~crc32_update(0xffffffffU, data, user);

		memset(tx->cw, 0, (size_t)ncw * QAM_UNIT);
		/* User data, then its CRC-32 (little endian), into the units' data bytes. */
		for (size_t d = 0; d < user + QAM_CRC_BYTES; d++) {
			uint8_t v = d < user ? data[d] : (uint8_t)(crc >> (8 * (d - user)));

			tx->cw[data_slot(&l, fec, d)] = v;
		}
		for (unsigned int c = 0; c < ncw; c++) {
			uint8_t *u = &tx->cw[c * QAM_UNIT];

			if (fec == QAM_FEC_RS) {
				/* A short unit is shortened: the zeros in front stay virtual. */
				rs_encode(u);
			} else if (fec == QAM_FEC_HAMMING) {
				for (unsigned int b = 0; b < unit_len(&l, c); b += HAM_N) {
					ham_encode(&u[b]);
				}
			}
		}
	}

	/* Interleaved bytes (the units in turn), MSB first, into symbols. */
	ks = ks0 + QAM_PRE_SYMS + QAM_HDR_SYMS;
	for (unsigned int pos = 0, c = 0; j < l.coded; j++) {
		acc = (acc << 8) | tx->cw[c * QAM_UNIT + unit_start(&l, c) + pos];
		nbits += 8;
		interleave_next(&l, &c, &pos);
		while (nbits >= bps) {
			unsigned int v = (acc >> (nbits - bps)) & ((1U << bps) - 1U);

			nbits -= bps;
			tx->sym_i[ks] = gray_decode(v >> q);
			tx->sym_q[ks++] = gray_decode(v & (k - 1U));
		}
	}
	if (nbits > 0) {
		unsigned int v = (acc << (bps - nbits)) & ((1U << bps) - 1U);

		tx->sym_i[ks] = gray_decode(v >> q);
		tx->sym_q[ks++] = gray_decode(v & (k - 1U));
	}
	/* Normally a no-op: the frame length is the mapped symbols rounded up. */
	while (ks < ks0 + len / QAM_SPS) {
		tx->sym_i[ks] = 0;
		tx->sym_q[ks++] = 0;
	}
	return len;
}

/* Cyclic convolution with the pulse over nsym symbols: the loop is seamless. */
QAM_HOT static void shape(struct qam_tx *tx, size_t nsym, uint32_t *out)
{
	tx->clipped = 0;
	for (size_t s = 0; s < nsym; s++) {
		int32_t ai[QAM_SPS / 2] = {0}, aq[QAM_SPS / 2] = {0};
		ptrdiff_t m = (ptrdiff_t)s + QAM_SPAN / 2;

		if (m >= (ptrdiff_t)nsym) {
			m -= (ptrdiff_t)nsym;
		}
		/* Span position p holds symbol s + SPAN / 2 - p. */
		for (int p = 0; p < QAM_SPAN; p++) {
			const int32_t *ri = tx->table[p][tx->sym_i[m]];
			const int32_t *rq = tx->table[p][tx->sym_q[m]];

			for (int ph = 0; ph < QAM_SPS / 2; ph++) {
				ai[ph] += ri[ph];
				aq[ph] += rq[ph];
			}
			if (--m < 0) {
				m += (ptrdiff_t)nsym;
			}
		}
		for (int ph = 0; ph < QAM_SPS; ph++) {
			/* Unpack the phase pair: the low half is signed, borrows go up. */
			int32_t pi = ai[ph / 2], pq = aq[ph / 2];
			int32_t li = (int16_t)pi, lq = (int16_t)pq;
			int32_t i = ph % 2 ? (pi - li) >> 16 : li;
			int32_t qv = ph % 2 ? (pq - lq) >> 16 : lq;

			i = (i + 8) >> 4;
			qv = (qv + 8) >> 4;

			if (i > 511 || i < -511 || qv > 511 || qv < -511) {
				tx->clipped++;
				i = i > 511 ? 511 : (i < -511 ? -511 : i);
				qv = qv > 511 ? 511 : (qv < -511 ? -511 : qv);
			}
			out[s * QAM_SPS + ph] = esp_sdr_tx_word(i, qv);
		}
	}
}

QAM_HOT size_t qam_tx_build(struct qam_tx *tx, const struct qam_hdr *hdr, const uint8_t *data,
			    uint32_t *out)
{
	size_t len = map_frame(tx, hdr, data, 0);

	if (len > 0) {
		shape(tx, len / QAM_SPS, out);
	}
	return len;
}

QAM_HOT size_t qam_tx_build_pair(struct qam_tx *tx, const struct qam_hdr *ha, const uint8_t *a,
				 const struct qam_hdr *hb, const uint8_t *b, uint32_t *out)
{
	size_t la, lb;

	if (!ha->pair || !hb->pair || ha->mod != hb->mod || ha->fec != hb->fec ||
	    ha->ncw != hb->ncw || ha->full != hb->full) {
		return 0;
	}
	la = map_frame(tx, ha, a, 0);
	lb = la > 0 ? map_frame(tx, hb, b, la / QAM_SPS) : 0;
	if (lb == 0) {
		return 0;
	}
	shape(tx, (la + lb) / QAM_SPS, out);
	return la + lb;
}


QAM_HOT float qam_rx_power(const uint32_t *words, size_t n)
{
	float e = 0.0f, mi = 0.0f, mq = 0.0f;

	/* 32-bit sums are exact for chunks of 1024 (10-bit samples). */
	for (size_t j0 = 0; j0 < n; j0 += 1024) {
		int32_t si = 0, sq = 0, ee = 0;
		size_t j1 = MIN_SZ(n, j0 + 1024);

		for (size_t j = j0; j < j1; j++) {
			int i = esp_sdr_rx_i(words[j]), q = esp_sdr_rx_q(words[j]);

			si += i;
			sq += q;
			ee += i * i + q * q;
		}
		e += (float)ee;
		mi += (float)si;
		mq += (float)sq;
	}
	if (n == 0) {
		return 0.0f;
	}
	mi /= (float)n;
	mq /= (float)n;
	return e / (float)n - mi * mi - mq * mq;
}

/* Copies on either side of a symbol that averaging may use. */
#define AVG_K 16
/* Energy map of the window for averaging: stretches of this many samples (32 cover a bank). */
#define AVG_CHUNK 512
/* A copy joins only if it matches this one this well (normalized correlation). */
#define AVG_COHERENCE 0.97f

/* Receive state while decoding one frame. */
struct rxs {
	const uint32_t *w;
	int n;
	float dcfi, dcfq;
	/* Frame start, length in transmit samples (0 until the header is known). */
	int t0;
	int len;
	/*
	 * Next symbol: position in receive samples (Q32; the sample clocks
	 * differ like the carriers, so it advances by QAM_SPS * ratio), the
	 * frame length in receive samples (Q32), derotation phasor, its step per
	 * symbol, the phase correction at a wrap.
	 */
	int64_t pos, step_q, len_q;
	struct cf rot, step, wrap;
	int count;
	/*
	 * Copy averaging (payload): up to navg loop copies per symbol, copy c
	 * (c * len_q away) phase aligned by cph[c + AVG_K].
	 */
	int navg;
	struct cf cph[2 * AVG_K + 1];
	float inv_navg;
	/* Window stretches (AVG_CHUNK samples) where the signal is on: bit per stretch. */
	uint32_t on;
};

static inline struct cf sample(const struct rxs *r, int n)
{
	uint32_t w = r->w[n];

	return (struct cf){(float)esp_sdr_rx_i(w) - r->dcfi, (float)esp_sdr_rx_q(w) - r->dcfq};
}

/* Cubic Lagrange interpolation between samples ip and ip + 1, mu in [0, 1). */
static inline struct cf interp(const struct rxs *r, int ip, float mu)
{
	struct cf a = sample(r, ip > 0 ? ip - 1 : 0), b = sample(r, ip);
	struct cf c = sample(r, ip + 1), d = sample(r, ip + 2);
	float m1 = mu - 1.0f, m2 = mu - 2.0f, p1 = mu + 1.0f;
	float ca = -mu * m1 * m2 * (1.0f / 6.0f), cb = p1 * m1 * m2 * 0.5f;
	float cc = -p1 * mu * m2 * 0.5f, cd = p1 * mu * m1 * (1.0f / 6.0f);

	return (struct cf){ca * a.re + cb * b.re + cc * c.re + cd * d.re,
			   ca * a.im + cb * b.im + cc * c.im + cd * d.im};
}

/*
 * Averaged symbol sample: every copy of this frame position in the window
 * (pos + c * len_q for integer c), each phase aligned, the sum scaled back to
 * one copy. pos runs on without wrapping; the copies cover the window.
 */
static struct cf next_y_avg(struct rxs *r)
{
	struct cf acc = {0.0f, 0.0f}, y;
	float lr = (float)r->len_q * 2.3283064e-10f;
	float fv = (float)r->pos * 2.3283064e-10f;
	int c = (int)ceilf((1.0f - fv) / lr), used = 0;
	int64_t p = r->pos + (int64_t)c * r->len_q;

	for (; used < r->navg && c <= AVG_K; c++, p += r->len_q) {
		int ip = (int)(p >> 32);
		struct cf s;

		if (c < -AVG_K || ip < 1) {
			continue;
		}
		if (ip + 2 >= r->n) {
			break;
		}
		/* Not this copy: it did not match, or the transmitter was off there. */
		if ((r->cph[c + AVG_K].re == 0.0f && r->cph[c + AVG_K].im == 0.0f) ||
		    !(r->on & (1UL << ((ip + 1) / AVG_CHUNK)))) {
			continue;
		}
		s = cmul(interp(r, ip, (float)(uint32_t)p * 2.3283064e-10f), r->cph[c + AVG_K]);
		acc.re += s.re;
		acc.im += s.im;
		used++;
	}
	if (used > 0) {
		float g = 1.0f / (float)used;

		acc.re *= g;
		acc.im *= g;
	}
	y = cmul(acc, r->rot);
	r->pos += r->step_q;
	r->rot = cmul(r->rot, r->step);
	if (++r->count == 256) {
		float m = 1.0f / sqrtf(r->rot.re * r->rot.re + r->rot.im * r->rot.im);

		r->rot.re *= m;
		r->rot.im *= m;
		r->count = 0;
	}
	return y;
}

/* Next symbol spaced sample, derotated, following the cyclic frame across the window end. */
static inline struct cf next_y(struct rxs *r)
{
	int ip = (int)(r->pos >> 32);
	struct cf y;

	if (r->navg > 1) {
		return next_y_avg(r);
	}

	if (ip + 2 >= r->n) {
		if (r->len_q == 0) {
			return (struct cf){0.0f, 0.0f};
		}
		r->pos -= r->len_q;
		r->rot = cmul(r->rot, r->wrap);
		ip = (int)(r->pos >> 32);
	}
	y = cmul(interp(r, ip, (float)(uint32_t)r->pos * 2.3283064e-10f), r->rot);
	r->pos += r->step_q;
	r->rot = cmul(r->rot, r->step);
	if (++r->count == 256) {
		float m = 1.0f / sqrtf(r->rot.re * r->rot.re + r->rot.im * r->rot.im);

		r->rot.re *= m;
		r->rot.im *= m;
		r->count = 0;
	}
	return y;
}

/* Schmidl-Cox runs on every SEARCH_STEP-th sample: two symbols. */
#define SEARCH_STEP (2 * QAM_SPS)
#define SEARCH_HALF (QAM_PRE_HALF * QAM_SPS / SEARCH_STEP)
/* Stop once past a clear peak: the metric falls as (1 - d / SEARCH_HALF)^2. */
#define SEARCH_CLEAR 0.7f
#define SEARCH_PAST  0.25f

/*
 * Schmidl-Cox over u(i) = x(SEARCH_STEP i) - dc, halves of SEARCH_HALF
 * subsamples, i from imin to imax. Returns the best i (or -1), its metric
 * and the correlation there.
 */
QAM_HOT static int search(const uint32_t *w, int dci, int dcq, int imin, int imax, float *metric,
			  float *pr_best, float *pi_best)
{
	const int h = SEARCH_HALF, step = SEARCH_STEP, lag = SEARCH_HALF * SEARCH_STEP;
	int32_t pr = 0, pi = 0, r1 = 0, r2 = 0;
	int best_i = -1;
	float best = 0.0f, bpr = 0.0f, bpi = 0.0f;
	const uint32_t *p = &w[imin * step];

	for (int m = 0; m < h; m++) {
		uint32_t wa = p[m * step], wb = p[m * step + lag];
		int ai = esp_sdr_rx_i(wa) - dci, aq = esp_sdr_rx_q(wa) - dcq;
		int bi = esp_sdr_rx_i(wb) - dci, bq = esp_sdr_rx_q(wb) - dcq;

		pr += ai * bi + aq * bq;
		pi += aq * bi - ai * bq;
		r1 += ai * ai + aq * aq;
		r2 += bi * bi + bq * bq;
	}
	for (int i = imin;; i++, p += step) {
		float m2 = (float)pr * (float)pr + (float)pi * (float)pi;
		float den = (float)r1 * (float)r2;

		if (m2 > best * den && den > 0.0f) {
			best = m2 / den;
			best_i = i;
			bpr = (float)pr;
			bpi = (float)pi;
		} else if (best > SEARCH_CLEAR && m2 < SEARCH_PAST * best * den) {
			break;
		}
		if (i == imax) {
			break;
		}
		/* Slide: drop m = i, add m = i + h. */
		uint32_t wa = p[0], wb = p[lag], wc = p[2 * lag];
		int ai = esp_sdr_rx_i(wa) - dci, aq = esp_sdr_rx_q(wa) - dcq;
		int bi = esp_sdr_rx_i(wb) - dci, bq = esp_sdr_rx_q(wb) - dcq;
		int ci = esp_sdr_rx_i(wc) - dci, cq = esp_sdr_rx_q(wc) - dcq;
		int eb = bi * bi + bq * bq;

		pr += bi * ci + bq * cq - (ai * bi + aq * bq);
		pi += bq * ci - bi * cq - (aq * bi - ai * bq);
		r1 += eb - (ai * ai + aq * aq);
		r2 += ci * ci + cq * cq - eb;
	}
	*metric = best;
	*pr_best = bpr;
	*pi_best = bpi;
	return best_i;
}

/* |sum over the first syms preamble symbols of y(t + QAM_SPS k) ph(k) conj(a_k)|^2. */
QAM_HOT static float preamble_corr(const struct rxs *r, const struct cf *ph, int t, int syms)
{
	struct cf s = {0.0f, 0.0f};

	for (int kk = 0; kk < syms; kk++) {
		struct cf y = cmul(sample(r, t + kk * QAM_SPS), ph[kk]);
		unsigned int a = preamble[kk % QAM_PRE_HALF];

		/* y * conj(a), a = (+-1 +-j) / sqrt(2) without the scale. */
		float yr = (a & 2U) ? y.re : -y.re, yi = (a & 2U) ? y.im : -y.im;
		float zr = (a & 1U) ? y.im : -y.im, zi = (a & 1U) ? -y.re : y.re;

		s.re += yr + zr;
		s.im += yi + zi;
	}
	return 0.5f * (s.re * s.re + s.im * s.im);
}

/* Solve the Hermitian positive definite a x = v (Cholesky); false if singular. */
static bool solve(struct cf a[UNK][UNK], struct cf v[UNK], struct cf x[UNK])
{
	struct cf l[UNK][UNK];
	struct cf z[UNK];
	float inv[UNK];

	memset(l, 0, sizeof(l));
	for (int j = 0; j < UNK; j++) {
		float d = a[j][j].re;

		for (int k = 0; k < j; k++) {
			d -= l[j][k].re * l[j][k].re + l[j][k].im * l[j][k].im;
		}
		if (d <= 0.0f) {
			return false;
		}
		d = sqrtf(d);
		inv[j] = 1.0f / d;
		l[j][j] = (struct cf){d, 0.0f};
		for (int i = j + 1; i < UNK; i++) {
			struct cf s = a[i][j];

			for (int k = 0; k < j; k++) {
				/* s -= l[i][k] * conj(l[j][k]) */
				s.re -= l[i][k].re * l[j][k].re + l[i][k].im * l[j][k].im;
				s.im -= l[i][k].im * l[j][k].re - l[i][k].re * l[j][k].im;
			}
			l[i][j] = (struct cf){s.re * inv[j], s.im * inv[j]};
		}
	}
	/* L z = v, then L^H x = z. */
	for (int i = 0; i < UNK; i++) {
		struct cf s = v[i];

		for (int k = 0; k < i; k++) {
			struct cf p = cmul(l[i][k], z[k]);

			s.re -= p.re;
			s.im -= p.im;
		}
		z[i] = (struct cf){s.re * inv[i], s.im * inv[i]};
	}
	for (int i = UNK - 1; i >= 0; i--) {
		struct cf s = z[i];

		for (int k = i + 1; k < UNK; k++) {
			/* conj(l[k][i]) * x[k] */
			struct cf c = {l[k][i].re, -l[k][i].im};
			struct cf p = cmul(c, x[k]);

			s.re -= p.re;
			s.im -= p.im;
		}
		x[i] = (struct cf){s.re * inv[i], s.im * inv[i]};
	}
	return true;
}

/* Equalizer and decision directed phase loop. */
struct eq {
	struct cf w[TAPS], b;
	/* The last TAPS inputs twice over, so a window never wraps. */
	struct cf ring[2 * TAPS];
	int pos;
	/* exp(-j theta), the loop's integrator, symbols since renormalizing. */
	struct cf rot;
	float nu;
	int count;
	float sig, err;
	/* LMS step, normalized by the input power (0: taps fixed after training). */
	float mu, mu_b;
};

static inline struct cf eq_out(struct eq *e, struct cf y)
{
	struct cf z = e->b;
	const struct cf *x;

	e->ring[e->pos] = y;
	e->ring[e->pos + TAPS] = y;
	if (++e->pos == TAPS) {
		e->pos = 0;
	}
	/* Oldest first: ring[pos .. pos + TAPS - 1]. */
	x = &e->ring[e->pos];
	for (int l = 0; l < TAPS; l++) {
		z.re += e->w[l].re * x[l].re - e->w[l].im * x[l].im;
		z.im += e->w[l].re * x[l].im + e->w[l].im * x[l].re;
	}
	return cmul(z, e->rot);
}

static inline void eq_track(struct eq *e, struct cf z, struct cf d)
{
	float er = z.re - d.re, ei = z.im - d.im;

	if (e->mu > 0.0f && (e->count & (QAM_LMS_EVERY - 1)) == 0) {
		/*
		 * Decision directed LMS on the taps: the error back before the
		 * phase loop's rotation, w -= mu e conj(x).
		 */
		struct cf eb = cmul((struct cf){er, ei}, (struct cf){e->rot.re, -e->rot.im});
		const struct cf *x = &e->ring[e->pos];

		for (int l = 0; l < TAPS; l++) {
			e->w[l].re -= e->mu * (eb.re * x[l].re + eb.im * x[l].im);
			e->w[l].im -= e->mu * (eb.im * x[l].re - eb.re * x[l].im);
		}
		e->b.re -= e->mu * e->mu_b * eb.re;
		e->b.im -= e->mu * e->mu_b * eb.im;
	}
	/* Im(z conj(d)): weighs the outer points more, saves a division. */
	float pe = z.im * d.re - z.re * d.im;
	float delta;

	e->sig += d.re * d.re + d.im * d.im;
	e->err += er * er + ei * ei;
	e->nu += QAM_PLL_KI * pe;
	delta = QAM_PLL_KP * pe + e->nu;
	/* rot *= exp(-j delta), small angle. */
	e->rot = cmul(e->rot, (struct cf){1.0f - 0.5f * delta * delta, -delta});
	if (++e->count == 64) {
		float m = 1.0f / sqrtf(e->rot.re * e->rot.re + e->rot.im * e->rot.im);

		e->rot.re *= m;
		e->rot.im *= m;
		e->count = 0;
	}
}

static inline unsigned int slice(float v, unsigned int k, float inv_c, float c, float *level)
{
	float t = (v * inv_c + (float)(k - 1U)) * 0.5f + 0.5f;
	int i = t > 0.0f ? (int)t : 0;

	if (i > (int)k - 1) {
		i = (int)k - 1;
	}
	*level = c * (float)(2 * i - (int)(k - 1U));
	return (unsigned int)i;
}

/* Least squares equalizer over the preamble; false if it is singular. */
static bool train(struct qam_rx *rx, struct rxs *r, struct eq *e, struct qam_rx_info *info)
{
	struct cf a[UNK][UNK], v[UNK], x[UNK];
	struct cf *ytr = (struct cf *)rx->ytr;
	float trace = 0.0f, side = 0.0f;

	for (int kk = 0; kk < QAM_PRE_SYMS + HALF; kk++) {
		ytr[kk] = next_y(r);
	}
	memset(a, 0, sizeof(a));
	memset(v, 0, sizeof(v));
	for (int kk = HALF; kk < QAM_PRE_SYMS; kk++) {
		struct cf xr[UNK];
		struct cf d = qpsk_point(preamble[kk % QAM_PRE_HALF]);

		for (int l = 0; l < TAPS; l++) {
			xr[l] = ytr[kk - HALF + l];
		}
		xr[TAPS] = (struct cf){1.0f, 0.0f};
		for (int i = 0; i < UNK; i++) {
			/* conj(x_i) x_j and conj(x_i) d */
			for (int j = i; j < UNK; j++) {
				a[i][j].re += xr[i].re * xr[j].re + xr[i].im * xr[j].im;
				a[i][j].im += xr[i].re * xr[j].im - xr[i].im * xr[j].re;
			}
			v[i].re += xr[i].re * d.re + xr[i].im * d.im;
			v[i].im += xr[i].re * d.im - xr[i].im * d.re;
		}
	}
	for (int i = 0; i < UNK; i++) {
		for (int j = 0; j < i; j++) {
			a[i][j] = (struct cf){a[j][i].re, -a[j][i].im};
		}
		trace += a[i][i].re;
	}
	for (int i = 0; i < UNK; i++) {
		a[i][i].re += 1e-4f * trace / UNK;
	}
	if (!solve(a, v, x)) {
		return false;
	}
	memset(e, 0, sizeof(*e));
	e->rot = (struct cf){1.0f, 0.0f};
	for (int l = 0; l < TAPS; l++) {
		e->w[l] = x[l];
		/* The ring then holds y(PRE - HALF - 1 .. PRE + HALF - 1), oldest first. */
		e->ring[l] = e->ring[l + TAPS] = ytr[QAM_PRE_SYMS - TAPS + HALF + l];
		rx->w_re[l] = x[l].re;
		rx->w_im[l] = x[l].im;
		if (l != HALF) {
			side += x[l].re * x[l].re + x[l].im * x[l].im;
		}
	}
	e->b = x[TAPS];
	{
		float py = 0.0f;

		for (int kk = 0; kk < QAM_PRE_SYMS; kk++) {
			py += ytr[kk].re * ytr[kk].re + ytr[kk].im * ytr[kk].im;
		}
		py /= QAM_PRE_SYMS;
		e->mu = QAM_LMS_MU * QAM_LMS_EVERY / ((float)TAPS * py);
		/* The bias input is 1, not py. */
		e->mu_b = py;
	}
	rx->b_re = x[TAPS].re;
	rx->b_im = x[TAPS].im;
	info->isi_db = 10.0f * log10f((x[HALF].re * x[HALF].re + x[HALF].im * x[HALF].im) /
				      (side + 1e-20f));
	return true;
}

/* Demap the payload into the interleaved codeword buffer. */
QAM_HOT static void demod_payload(struct qam_rx *rx, struct rxs *r, struct eq *e,
				  const struct qam_hdr *hdr)
{
	uint8_t *cw = rx->cw;
	unsigned int q = hdr->mod + 1U, k = 1U << q, bps = 2U * q;
	unsigned int nbits = 0, pos = 0, c = 0;
	size_t j = 0, nb;
	float cs = axis_scale(k), inv_cs = 1.0f / cs;
	size_t nsym;
	uint32_t acc = 0;
	struct layout l;

	(void)hdr_layout(hdr, &l);
	nb = l.coded;
	nsym = (nb * 8U + bps - 1U) / bps;

	for (size_t s = 0; s < nsym; s++) {
		struct cf z = eq_out(e, next_y(r));
		struct cf d;
		unsigned int ii = slice(z.re, k, inv_cs, cs, &d.re);
		unsigned int iq = slice(z.im, k, inv_cs, cs, &d.im);

		eq_track(e, z, d);
#ifdef QAM_DEBUG
		{
			extern void qam_debug_sym(size_t s, struct cf z, struct cf d);

			qam_debug_sym(s, z, d);
		}
#endif
		acc = (acc << bps) | ((ii ^ (ii >> 1)) << q) | (iq ^ (iq >> 1));
		nbits += bps;
		while (nbits >= 8 && j < nb) {
			nbits -= 8;
			cw[c * QAM_UNIT + unit_start(&l, c) + pos] = (uint8_t)(acc >> nbits);
			j++;
			interleave_next(&l, &c, &pos);
		}
	}
}

_Static_assert(sizeof(struct eq) <= sizeof(((struct qam_rx *)0)->eq), "qam_rx.eq too small");

/* Between qam_rx_begin*() and qam_rx_finish(). */
struct rx_job {
	struct rxs r;
	uint32_t prof_t;
};

_Static_assert(sizeof(struct rx_job) <= sizeof(((struct qam_rx *)0)->job), "qam_rx.job too small");

/* DC of every SEARCH_STEP-th sample; returns the integer DC for the search. */
static void dc_estimate(struct rxs *r, int *dci, int *dcq)
{
	int32_t si = 0, sq = 0, cnt = 0;

	for (int m = 0; m < r->n; m += SEARCH_STEP) {
		si += esp_sdr_rx_i(r->w[m]);
		sq += esp_sdr_rx_q(r->w[m]);
		cnt++;
	}
	r->dcfi = (float)si / (float)cnt;
	r->dcfq = (float)sq / (float)cnt;
	*dci = (si + cnt / 2) / cnt;
	*dcq = (sq + cnt / 2) / cnt;
}

/*
 * Copy averaging from here on (the payload): how many copies the window
 * holds, and each copy's phase against the one at the current position,
 * measured from the samples (offset times the period plus phase noise).
 */
static void setup_avg(struct rxs *r, int want)
{
	int len = (int)(r->len_q >> 32), ip = (int)(r->pos >> 32);
	int fit = len > 0 ? (r->n - 4) / len : 0;

	r->navg = MIN_INT(MIN_INT(want, fit), 2 * AVG_K + 1);
	if (r->navg < 2) {
		r->navg = 0;
		return;
	}
	r->inv_navg = 1.0f / (float)r->navg;
	/* Energy per stretch (every 4th sample): on where above half the loudest. */
	{
		float e[32] = {0}, emax = 0.0f;
		int nch = MIN_INT((r->n + AVG_CHUNK - 1) / AVG_CHUNK, 32);

		for (int k = 0; k < nch; k++) {
			for (int m = k * AVG_CHUNK; m < MIN_INT((k + 1) * AVG_CHUNK, r->n); m += 4) {
				struct cf x = sample(r, m);

				e[k] += x.re * x.re + x.im * x.im;
			}
			emax = e[k] > emax ? e[k] : emax;
		}
		r->on = 0;
		for (int k = 0; k < nch; k++) {
			if (e[k] > 0.5f * emax) {
				r->on |= 1UL << k;
			}
		}
	}
	for (int c = -AVG_K; c <= AVG_K; c++) {
		struct cf d = {0.0f, 0.0f};
		float e0 = 0.0f, e1 = 0.0f;
		/* A stretch of this copy and of copy c, both inside the window. */
		int k0 = ip - WRAP_OVERLAP / 2, sh = c * len;

		if (k0 + sh < 1) {
			k0 = 1 - sh;
		}
		if (k0 < 1) {
			k0 = 1;
		}
		if (k0 + sh + WRAP_OVERLAP + 3 > r->n) {
			k0 = r->n - 3 - WRAP_OVERLAP - sh;
		}
		if (c == 0) {
			r->cph[c + AVG_K] = (struct cf){1.0f, 0.0f};
			continue;
		}
		if (k0 < 1 || k0 + WRAP_OVERLAP + 3 > r->n) {
			r->cph[c + AVG_K] = (struct cf){0.0f, 0.0f};
			continue;
		}
		for (int m = k0; m < k0 + WRAP_OVERLAP; m++) {
			struct cf s1 = sample(r, m + sh), s0 = sample(r, m);

			d.re += s1.re * s0.re + s1.im * s0.im;
			d.im += s1.im * s0.re - s1.re * s0.im;
			e0 += s0.re * s0.re + s0.im * s0.im;
			e1 += s1.re * s1.re + s1.im * s1.im;
		}
		{
			float mg = sqrtf(d.re * d.re + d.im * d.im);

			/* exp(-j arg d): copy c back onto this one; none if they do not match. */
			r->cph[c + AVG_K] = mg > AVG_COHERENCE * sqrtf(e0 * e1)
						    ? (struct cf){d.re / mg, -d.im / mg}
						    : (struct cf){0.0f, 0.0f};
		}
	}
}

/*
 * From a coarse start and offset: fine timing, the offset again, training
 * and the header. Leaves rx->job ready for the payload.
 */
static int acquire(struct qam_rx *rx, size_t n, int t_center, float f, struct qam_hdr *hdr,
		   struct qam_rx_info *info)
{
	struct rx_job *j = (struct rx_job *)rx->job;
	int tbest = 0;
	float ratio, cbest = -1.0f;
	struct cf *ph = (struct cf *)rx->ph;
	struct eq *e = (struct eq *)rx->eq;
	uint8_t hb[QAM_HDR_BYTES];
	uint32_t prof_t = j->prof_t;

	/* Fine timing against the known preamble: whole symbols first, then samples. */
	{
		struct cf st = {cosf(-TWO_PI * f * QAM_SPS), sinf(-TWO_PI * f * QAM_SPS)};
		struct cf p = {1.0f, 0.0f};
		int center = t_center;

		for (int kk = 0; kk < QAM_PRE_SYMS; kk++) {
			ph[kk] = p;
			p = cmul(p, st);
		}
		for (int pass = 0; pass < 2; pass++) {
			int lo = pass == 0 ? -FINE_RANGE : -(QAM_SPS - 1);
			int hi = pass == 0 ? FINE_RANGE : QAM_SPS - 1;
			int stride = pass == 0 ? QAM_SPS : 1;

			for (int tau = lo; tau <= hi; tau += stride) {
				int t = center + tau;
				float m2;

				if (t < 0 || (pass == 1 && tau == 0)) {
					continue;
				}
				/* One half is enough to pick the symbol. */
				m2 = preamble_corr(&j->r, ph, t, pass == 0 ? QAM_PRE_HALF : QAM_PRE_SYMS);
#ifdef QAM_DEBUG
				if (getenv("TAU")) {
					printf("tau %d %.3g\n", t - t_center, (double)m2);
				}
#endif
				if (m2 > cbest) {
					cbest = m2;
					tbest = t;
				}
			}
			center = tbest;
			if (pass == 0) {
				cbest = preamble_corr(&j->r, ph, tbest, QAM_PRE_SYMS);
			}
		}
	}
	{
		float ey = 0.0f;

		for (int kk = 0; kk < QAM_PRE_SYMS; kk++) {
			struct cf y = sample(&j->r, tbest + kk * QAM_SPS);

			ey += y.re * y.re + y.im * y.im;
		}
		info->xcorr = ey > 0.0f ? cbest / (ey * (float)QAM_PRE_SYMS) : 0.0f;
	}
	j->r.t0 = tbest;
	info->start = tbest;
	if (info->xcorr < XCORR_MIN) {
		PROF(1);
		return QAM_RX_NONE;
	}

	/* Offset again at the full rate over the preamble, edges excluded (pulse tails). */
	{
		struct cf p = {0.0f, 0.0f};
		const int lag = QAM_PRE_HALF * QAM_SPS;

		for (int m = QAM_SPAN * QAM_SPS / 2; m < lag - QAM_SPAN * QAM_SPS / 2; m++) {
			struct cf s0 = sample(&j->r, j->r.t0 + m), s1 = sample(&j->r, j->r.t0 + m + lag);

			p.re += s0.re * s1.re + s0.im * s1.im;
			p.im += s0.im * s1.re - s0.re * s1.im;
		}
		f = -atan2f(p.im, p.re) / (TWO_PI * (float)lag);
	}
	info->cfo_hz = f * 80e6f;
	PROF(1);

	/* Receive samples per transmit sample, from the carrier offset (one crystal each). */
	ratio = rx->carrier_hz > 0.0f ? 1.0f - info->cfo_hz / rx->carrier_hz : 1.0f;
	j->r.pos = (int64_t)j->r.t0 << 32;
	j->r.step_q = (int64_t)((double)QAM_SPS * (double)ratio * 4294967296.0);
	j->r.rot = (struct cf){cosf(-TWO_PI * f * (float)j->r.t0), sinf(-TWO_PI * f * (float)j->r.t0)};
	j->r.step = (struct cf){cosf(-TWO_PI * f * QAM_SPS * ratio), sinf(-TWO_PI * f * QAM_SPS * ratio)};
	if (!train(rx, &j->r, e, info)) {
		return QAM_RX_NONE;
	}
	PROF(2);

	/* Header: each new sample y(k + HALF) yields the output for symbol k. */
	memset(hb, 0, sizeof(hb));
	for (int kk = 0; kk < QAM_HDR_SYMS; kk++) {
		struct cf z = eq_out(e, next_y(&j->r));
		struct cf d = {z.re > 0.0f ? 0.70710678f : -0.70710678f,
			       z.im > 0.0f ? 0.70710678f : -0.70710678f};

		hb[kk / 4] |= (uint8_t)((((z.re > 0.0f) << 1) | (z.im > 0.0f)) << (6 - 2 * (kk % 4)));
		eq_track(e, z, d);
	}
	PROF(3);
	if (!hdr_unpack(hb, hdr)) {
		return QAM_RX_HEADER;
	}
	/* A pair loops two frames: the copies repeat after both. */
	j->r.len = (int)qam_hdr_frame_samples(hdr) * (hdr->pair ? 2 : 1);
	if (hdr->mod >= QAM_MODS || j->r.len == 0 || j->r.len > (int)n) {
		return QAM_RX_HEADER;
	}
	j->r.len_q = (int64_t)((double)j->r.len * (double)ratio * 4294967296.0);
	j->r.wrap = (struct cf){cosf(TWO_PI * f * (float)j->r.len), sinf(TWO_PI * f * (float)j->r.len)};
	/*
	 * The equalizer reads HALF symbols past the frame end and interpolation
	 * two samples more: measure whenever those reads leave the window.
	 */
	if (j->r.t0 + j->r.len + (HALF + 1) * QAM_SPS + 3 > (int)n) {
		/*
		 * The frame wraps: its end comes from the copy before. Both
		 * copies overlap at the window end; their phase step there
		 * (offset plus phase noise over one period) replaces the one
		 * the offset estimate predicts.
		 */
		struct cf d = {0.0f, 0.0f};
		int from = MAX_INT(j->r.t0, j->r.len), to = MIN_INT((int)n, from + WRAP_OVERLAP);

		for (int m = from; m < to; m++) {
			struct cf s1 = sample(&j->r, m), s0 = sample(&j->r, m - j->r.len);

			d.re += s1.re * s0.re + s1.im * s0.im;
			d.im += s1.im * s0.re - s1.re * s0.im;
		}
		if (to - from >= WRAP_OVERLAP / 4) {
			float m = sqrtf(d.re * d.re + d.im * d.im);

			if (m > 0.0f) {
				j->r.wrap = (struct cf){d.re / m, d.im / m};
			}
		}
	}

	if (rx->avg > 1) {
		setup_avg(&j->r, rx->avg);
	}
	j->prof_t = prof_t;
	return 0;
}

int qam_rx_begin(struct qam_rx *rx, const uint32_t *words, size_t n, struct qam_hdr *hdr,
		 struct qam_rx_info *info)
{
	struct rx_job *j = (struct rx_job *)rx->job;
	int imin = (FINE_RANGE + SEARCH_STEP - 1) / SEARCH_STEP;
	int imax = ((int)n - QAM_SEARCH_MARGIN) / SEARCH_STEP;
	int ibest, dci, dcq;
	float best, pbr, pbi;
	uint32_t prof_t = clk();

	preamble_init();
	memset(info, 0, sizeof(*info));
	memset(j, 0, sizeof(*j));
	j->r.w = words;
	j->r.n = (int)n;
	if (imax <= imin) {
		return QAM_RX_NONE;
	}
	dc_estimate(&j->r, &dci, &dcq);
	ibest = search(words, dci, dcq, imin, imax, &best, &pbr, &pbi);
	PROF(0);
	info->metric = best;
#ifdef QAM_DEBUG
	if (getenv("TAU")) {
		printf("coarse %d metric %.4f\n", ibest * SEARCH_STEP, (double)best);
	}
#endif
	if (ibest < 0 || best < METRIC_MIN) {
		return QAM_RX_NONE;
	}
	j->prof_t = prof_t;
	/* Coarse offset (cycles per sample) from the half lag. */
	return acquire(rx, n, ibest * SEARCH_STEP,
		       -atan2f(pbi, pbr) / (TWO_PI * (float)(SEARCH_HALF * SEARCH_STEP)), hdr, info);
}

int qam_rx_begin_at(struct qam_rx *rx, const uint32_t *words, size_t n, int start, float cfo_hz,
		    struct qam_hdr *hdr, struct qam_rx_info *info)
{
	struct rx_job *j = (struct rx_job *)rx->job;
	int dci, dcq;

	preamble_init();
	memset(info, 0, sizeof(*info));
	memset(j, 0, sizeof(*j));
	j->r.w = words;
	j->r.n = (int)n;
	j->prof_t = clk();
	if (start < 0 || start + QAM_SEARCH_MARGIN > (int)n) {
		return QAM_RX_NONE;
	}
	dc_estimate(&j->r, &dci, &dcq);
	info->metric = 1.0f;
	return acquire(rx, n, start, cfo_hz / 80e6f, hdr, info);
}

int qam_rx_finish(struct qam_rx *rx, const struct qam_hdr *hdr, struct qam_rx_info *info)
{
	struct rx_job *j = (struct rx_job *)rx->job;
	struct eq *e = (struct eq *)rx->eq;
	uint32_t prof_t = j->prof_t;

	demod_payload(rx, &j->r, e, hdr);
	info->mer_db = 10.0f * log10f(e->sig / (e->err + 1e-20f));
	PROF(4);

	{
		struct layout l;

		(void)hdr_layout(hdr, &l);
		for (unsigned int cw = 0; cw < l.ncw; cw++) {
			uint8_t *u = &rx->cw[cw * QAM_UNIT];
			int ret = 0;

			if (hdr->fec == QAM_FEC_RS) {
				unsigned int st = unit_start(&l, cw);

				/* A shortened unit's virtual zeros must stay zero. */
				memset(u, 0, st);
				ret = rs_decode(u);
				for (unsigned int i = 0; i < st && ret >= 0; i++) {
					ret = u[i] == 0 ? ret : -1;
				}
			} else if (hdr->fec == QAM_FEC_HAMMING) {
				for (unsigned int b = 0; b < unit_len(&l, cw) && ret >= 0; b += HAM_N) {
					int r1 = ham_decode(&u[b]);

					ret = r1 < 0 ? -1 : ret + r1;
				}
			}
			if (ret < 0) {
				info->failed++;
			} else {
				info->corrected += ret;
			}
		}
		PROF(5);
		if (info->failed > 0) {
			return QAM_RX_DATA;
		}
		if (l.ncw > 0) {
			uint32_t crc = 0xffffffffU, got = 0;

			for (size_t d = 0; d < l.user + QAM_CRC_BYTES; d++) {
				uint8_t v = rx->cw[data_slot(&l, (enum qam_fec)hdr->fec, d)];

				if (d < l.user) {
					crc = crc32_update(crc, &v, 1);
				} else {
					got |= (uint32_t)v << (8 * (d - l.user));
				}
			}
			if (~crc != got) {
				return QAM_RX_DATA;
			}
		}
	}
	return 0;
}

int qam_rx_decode(struct qam_rx *rx, const uint32_t *words, size_t n, struct qam_hdr *hdr,
		  struct qam_rx_info *info)
{
	int ret = qam_rx_begin(rx, words, n, hdr, info);

	return ret != 0 ? ret : qam_rx_finish(rx, hdr, info);
}

void qam_rx_data(const struct qam_rx *rx, const struct qam_hdr *hdr, uint8_t *out)
{
	struct layout l;

	if (!hdr_layout(hdr, &l)) {
		return;
	}
	for (size_t d = 0; d < l.user; d++) {
		out[d] = rx->cw[data_slot(&l, (enum qam_fec)hdr->fec, d)];
	}
}
