/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * OFDM physical layer, see ofdm.h. Transmit: subcarriers into the FFT buffer,
 * an unscaled radix-2 inverse FFT, the cyclic prefix and the body out as DAC
 * words. Receive, straight from the capture words: DC from every 4th sample,
 * a Schmidl-Cox search as exact integer sliding sums, the carrier offset from
 * its phase, then per symbol a derotation by table, a forward FFT and per
 * subcarrier one complex multiply by the equalizer the pilots set up. A
 * common phase per symbol, measured against its own decisions, follows the
 * residual offset across the frame.
 */

#include <math.h>
#include <string.h>

#include "ofdm.h"

#define PI     3.14159265358979323846f
#define TWO_PI 6.28318530717958647692f

#if defined(__ZEPHYR__)
#include <esp_attr.h>
#define OFDM_HOT IRAM_ATTR __attribute__((noinline))
#else
#define OFDM_HOT
#endif

#define PREAMBLE_SEED 0x5a17e4a1U
#define PILOT_SEED    0xc39a2b7fU
/* Least preamble metric for a detection. */
#define METRIC_MIN 0.3f
/* Plateau edge: the earliest start whose metric is within this of the best. */
#define PLATEAU 0.9f

uint32_t (*ofdm_clock)(void);

static inline uint32_t clk(void)
{
	return ofdm_clock ? ofdm_clock() : 0U;
}

static float pn_sign(uint32_t seed, unsigned int k)
{
	uint32_t x = seed + k * 2654435761U;

	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return (x & 1U) ? 1.0f : -1.0f;
}

void ofdm_test_bits(uint8_t *bits, size_t nbits, uint32_t seed)
{
	uint32_t x = seed | 1U;

	for (size_t i = 0; i < (nbits + 7U) / 8U; i++) {
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		bits[i] = (uint8_t)x;
	}
}

static inline unsigned int get_bit(const uint8_t *b, size_t i)
{
	return (b[i >> 3] >> (7U - (i & 7U))) & 1U;
}

/*
 * Square QAM, Gray coded per axis (I bits high), unit average energy: k
 * levels per axis at (2 l - (k - 1)) s. BPSK is +-1 on I alone.
 */
static const struct cons {
	uint8_t k, q;
	float s, inv2s, off;
} cons[OFDM_MODS] = {
	[OFDM_BPSK] = {2, 1, 1.0f, 0.5f, 0.5f},
	[OFDM_QPSK] = {2, 1, 0.70710678f, 0.70710678f, 0.5f},
	[OFDM_16QAM] = {4, 2, 0.31622777f, 1.58113883f, 1.5f},
	[OFDM_64QAM] = {8, 3, 0.15430335f, 3.24037035f, 3.5f},
};

/* Gray code to level index. */
static const uint8_t gray_dec[8] = {0, 1, 3, 2, 7, 6, 4, 5};

static inline struct ofdm_cf map_point(enum ofdm_mod mod, unsigned int v)
{
	const struct cons *k = &cons[mod];

	if (mod == OFDM_BPSK) {
		return (struct ofdm_cf){v ? 1.0f : -1.0f, 0.0f};
	}
	return (struct ofdm_cf){(float)(2 * gray_dec[v >> k->q] - (k->k - 1)) * k->s,
				(float)(2 * gray_dec[v & (k->k - 1U)] - (k->k - 1)) * k->s};
}

/* Nearest level on one axis: its Gray code, and the level through @p d. */
static inline unsigned int slice_axis(const struct cons *k, float x, float *d)
{
	float t = x * k->inv2s + k->off + 0.5f;
	int l = t < 0.0f ? 0 : (int)t;

	l = l > k->k - 1 ? k->k - 1 : l;
	*d = (float)(2 * l - (k->k - 1)) * k->s;
	return (unsigned int)(l ^ (l >> 1));
}

/* The bits of the nearest point to @p z, and that point through @p d. */
static inline unsigned int decide(enum ofdm_mod mod, struct ofdm_cf z, struct ofdm_cf *d)
{
	const struct cons *k = &cons[mod];
	unsigned int gi, gq;

	if (mod == OFDM_BPSK) {
		d->re = z.re > 0.0f ? 1.0f : -1.0f;
		d->im = 0.0f;
		return z.re > 0.0f;
	}
	gi = slice_axis(k, z.re, &d->re);
	gq = slice_axis(k, z.im, &d->im);
	return (gi << k->q) | gq;
}

/* Frequency index of subcarrier a: -floor(c/2) .. -1, 1 .. ceil(c/2). */
static inline int sub_freq(unsigned int c, unsigned int a)
{
	int f = (int)a - (int)(c / 2U);

	return f >= 0 ? f + 1 : f;
}

int ofdm_init(struct ofdm_ctx *ctx, const struct ofdm_cfg *cfg)
{
	unsigned int c = cfg->channels, nfft = OFDM_NFFT_MIN, log2n = 4, nsym, p = cfg->pilots;
	float ideal;

	if (cfg->sample_rate_hz == 0U || cfg->bandwidth_hz == 0U || c < 2U || cfg->cp_div < 4U ||
	    cfg->cp_div > OFDM_CP_DIV_MAX || p < 1U || p > OFDM_PILOTS_MAX ||
	    cfg->hdr_bits > OFDM_HDR_BITS_MAX) {
		return -1;
	}
	/* The power of two nearest (in octaves) to the size that spaces c subcarriers over bw. */
	ideal = (float)cfg->sample_rate_hz * (float)c / (float)cfg->bandwidth_hz;
	while (nfft < OFDM_NFFT_MAX && (float)nfft * 1.41421356f < ideal) {
		nfft <<= 1;
		log2n++;
	}
	/* Clamped at either end: the bandwidth would be off by more than an octave's half. */
	if ((float)nfft * 1.41421356f < ideal || (float)nfft > ideal * 1.41421356f) {
		return -1;
	}
	/* Leave a quarter of the band free around Nyquist (aliases of the DAC images). */
	if (c > nfft * 3U / 4U || c > OFDM_CH_MAX || nfft / cfg->cp_div == 0U) {
		return -1;
	}
	ctx->cfg = *cfg;
	ctx->nfft = nfft;
	ctx->log2n = log2n;
	ctx->cp = nfft / cfg->cp_div;
	ctx->slot = ctx->cp + nfft;
	ctx->spacing_hz = (float)cfg->sample_rate_hz / (float)nfft;
	ctx->bw_hz = ctx->spacing_hz * (float)c;
	if (cfg->symbols == 0U) {
		if (cfg->max_samples < (2U + p) * ctx->slot) {
			return -1;
		}
		nsym = (unsigned int)(cfg->max_samples / ctx->slot) - 1U - p;
	} else {
		nsym = cfg->symbols;
	}
	ctx->nhdr = (cfg->hdr_bits + c - 1U) / c;
	if (nsym <= ctx->nhdr + (cfg->mid_pilot ? 3U : 0U)) {
		return -1;
	}
	ctx->ndata = nsym - ctx->nhdr - (cfg->mid_pilot ? 1U : 0U);
	ctx->mid = ctx->ndata;
	if (cfg->mid_pilot) {
		/* Near the middle, the second half starting a byte at any modulation. */
		for (unsigned int m = ctx->ndata / 2U; m > 0U; m--) {
			if ((m * c) % 8U == 0U) {
				ctx->mid = m;
				break;
			}
		}
		if (ctx->mid == ctx->ndata) {
			return -1;
		}
	}
	ctx->len = (1U + p + nsym) * ctx->slot;
	if (ctx->len > cfg->max_samples) {
		return -1;
	}
	/* At half the rate the subcarriers must still fit (three quarters of the band). */
	if ((cfg->rx_div != 1U && cfg->rx_div != 2U) || c > (nfft / cfg->rx_div) * 3U / 4U ||
	    ctx->cp % cfg->rx_div != 0U || nfft / cfg->rx_div < OFDM_NFFT_MIN / 2U) {
		return -1;
	}
	ctx->rnfft = nfft / cfg->rx_div;
	ctx->rcp = ctx->cp / cfg->rx_div;
	ctx->rslot = ctx->rcp + ctx->rnfft;
	ctx->rlen = (1U + p + nsym) * ctx->rslot;

	for (unsigned int i = 0; i < nfft; i++) {
		unsigned int r = 0;

		for (unsigned int b = 0; b < log2n; b++) {
			r |= ((i >> b) & 1U) << (log2n - 1U - b);
		}
		ctx->rev[i] = (uint16_t)r;
	}
	for (unsigned int a = 0; a < c; a++) {
		int f = sub_freq(c, a);
		unsigned int rb = (unsigned int)(f < 0 ? f + (int)(nfft / cfg->rx_div) : f);

		ctx->bin[a] = (uint16_t)(f < 0 ? f + (int)nfft : f);
		/* Bit reversal over fewer bits: the top one of a smaller index is zero. */
		ctx->binr[a] = (uint16_t)(ctx->rev[rb] >> (cfg->rx_div - 1U));
	}
	/* esp-dsp radix-2 table: cos, sin of 2 pi i / nfft for i < nfft / 2, in bit reversed order. */
	for (unsigned int i = 0; i < nfft / 2U; i++) {
		unsigned int r = ctx->rev[i] >> 1;
		float a = TWO_PI * (float)r / (float)nfft;

		ctx->w[2U * i] = cosf(a);
		ctx->w[2U * i + 1U] = sinf(a);
	}
	return 0;
}

/*
 * Forward FFT of @p buf in place, unscaled: natural order in, bit reversed out
 * (read through ctx->rev / ctx->binr). The esp-dsp S3 kernel, or the same
 * algorithm in C (host, and the reference it is checked against).
 */
#if defined(CONFIG_APP_OFDM_ESP_DSP)
extern int dsps_fft2r_fc32_aes3_(float *data, int N, float *w);

static inline void fft(const struct ofdm_ctx *ctx, struct ofdm_cf *buf, unsigned int n)
{
	(void)dsps_fft2r_fc32_aes3_((float *)buf, (int)n, (float *)ctx->w);
}
#else
OFDM_HOT static void fft(const struct ofdm_ctx *ctx, struct ofdm_cf *buf, unsigned int n)
{
	float *d = (float *)buf;
	const float *w = ctx->w;

	for (unsigned int n2 = n / 2U, ie = 1; n2 > 0; n2 >>= 1, ie <<= 1) {
		unsigned int ia = 0;

		for (unsigned int j = 0; j < ie; j++, ia += n2) {
			float c = w[2U * j], s = w[2U * j + 1U];

			for (unsigned int i = 0; i < n2; i++, ia++) {
				float *a = &d[2U * ia], *b = &d[2U * (ia + n2)];
				float tr = c * b[0] + s * b[1], ti = c * b[1] - s * b[0];

				b[0] = a[0] - tr;
				b[1] = a[1] - ti;
				a[0] += tr;
				a[1] += ti;
			}
		}
	}
}
#endif

void ofdm_test_fft(struct ofdm_ctx *ctx)
{
	fft(ctx, ctx->w0.buf, ctx->nfft);
}

/*
 * One slot out of the FFT buffer: the inverse transform is the forward one of
 * the conjugate, conjugated again here (Q negated); the samples are read
 * through the bit reversal. Cyclic prefix first, scaled and clipped.
 */
OFDM_HOT static void emit(const struct ofdm_ctx *ctx, struct ofdm_txbuf *tb, float g,
			  uint32_t *out)
{
	unsigned int n = ctx->nfft;
	uint32_t clipped = 0;

	for (unsigned int i = 0; i < ctx->slot; i++) {
		const struct ofdm_cf *s = &tb->buf[ctx->rev[(i + n - ctx->cp) & (n - 1U)]];
		float fi = s->re * g, fq = -s->im * g;
		/* Rounded inline: lrintf() is a library call (in flash on the target). */
		int vi = (int)(fi + (fi >= 0.0f ? 0.5f : -0.5f));
		int vq = (int)(fq + (fq >= 0.0f ? 0.5f : -0.5f));

		if (vi > 511 || vi < -511 || vq > 511 || vq < -511) {
			clipped++;
			vi = vi > 511 ? 511 : (vi < -511 ? -511 : vi);
			vq = vq > 511 ? 511 : (vq < -511 ? -511 : vq);
		}
		out[i] = esp_sdr_tx_word(vi, vq);
	}
	tb->clipped += clipped;
}

size_t ofdm_tx_build(const struct ofdm_ctx *ctx, struct ofdm_txbuf *tb, enum ofdm_mod mod,
		     const uint8_t *hdr, const uint8_t *bits, uint32_t *out)
{
	unsigned int c = ctx->cfg.channels, bps = ofdm_mod_bits(mod), p = ctx->cfg.pilots;
	unsigned int h0 = 1U + p, d0 = h0 + ctx->nhdr, nh = ctx->cfg.hdr_bits;
	/* The mid pilot's slot (none: past the end). */
	unsigned int mp = ctx->cfg.mid_pilot ? d0 + ctx->mid : ~0U;
	/* No payload: a header-only frame (an ACK), the preamble, pilots and header. */
	unsigned int nslots = bits == NULL ? d0 : d0 + ctx->ndata + (ctx->cfg.mid_pilot ? 1U : 0U);
	/* The unscaled inverse FFT of c unit subcarriers has RMS sqrt(c). */
	float g = (float)ctx->cfg.amp / sqrtf((float)c);
	size_t bi = 0;

	if (mod >= OFDM_MODS) {
		return 0;
	}
	tb->clipped = 0;
	for (unsigned int s = 0; s < nslots; s++) {
		memset(tb->buf, 0, ctx->nfft * sizeof(tb->buf[0]));
		for (unsigned int a = 0; a < c; a++) {
			struct ofdm_cf *x = &tb->buf[ctx->bin[a]];

			if (s == 0) {
				/* Even subcarriers only, at twice the power: the same RMS. */
				if ((sub_freq(c, a) & 1) == 0) {
					*x = (struct ofdm_cf){1.41421356f * pn_sign(PREAMBLE_SEED, a),
							      0.0f};
				}
			} else if (s < h0 || s == mp) {
				*x = (struct ofdm_cf){pn_sign(PILOT_SEED, a), 0.0f};
			} else if (s < d0) {
				/* The header bits repeated over all header subcarriers. */
				unsigned int k = ((s - h0) * c + a) % nh;

				*x = (struct ofdm_cf){get_bit(hdr, k) ? 1.0f : -1.0f, 0.0f};
			} else {
				unsigned int v = 0;

				for (unsigned int b = 0; b < bps; b++, bi++) {
					v = (v << 1) | get_bit(bits, bi);
				}
				*x = map_point(mod, v);
				/* Conjugated in: see emit(). */
				x->im = -x->im;
			}
		}
		fft(ctx, tb->buf, ctx->nfft);
		emit(ctx, tb, g, &out[s * ctx->slot]);
	}
	return (size_t)nslots * ctx->slot;
}

struct sc_best {
	float num, den;
	int32_t pr, pi;
	int t, start;
};

/*
 * Schmidl-Cox scan over t = t0, t0 + st, .. t1: P(t) = sum conj(x[t + m])
 * x[t + m + L] and both halves' energies over m = 0, st, .. < L, as exact
 * integer sliding sums of the DC-free samples (the preamble's period holds on
 * any subsampling). The metric |P|^2 / (Ra Rb) is at most 1 and compared by
 * cross multiplication (the S3 FPU has no divide). At full resolution (st 1)
 * the cyclic prefix widens the peak to a plateau; best->start is its early
 * edge (later ones cut into the next symbol), from a ring of the last cp + 1
 * metrics.
 */
OFDM_HOT static void sc_scan(struct ofdm_ctx *ctx, const uint32_t *w, int t0, int t1, int st,
			     int dci, int dcq, struct sc_best *b)
{
	enum { MASK = OFDM_RING - 1U };
	int L = (int)ctx->rnfft / 2, cp = (int)ctx->rcp;
	int32_t pr = 0, pi = 0, ra = 0, rb = 0;
	float *rnum = ctx->rnum, *rden = ctx->rden;

	_Static_assert(OFDM_RING > OFDM_NFFT_MAX / 4U && (OFDM_RING & (OFDM_RING - 1U)) == 0U,
		       "ring: a power of two longer than the longest prefix");
	b->num = 0.0f;
	b->den = 1.0f;
	b->t = b->start = -1;
	for (int m = 0; m < L; m += st) {
		uint32_t wa = w[t0 + m], wb = w[t0 + m + L];
		int ai = esp_sdr_rx_i(wa) - dci, aq = esp_sdr_rx_q(wa) - dcq;
		int bi = esp_sdr_rx_i(wb) - dci, bq = esp_sdr_rx_q(wb) - dcq;

		pr += ai * bi + aq * bq;
		pi += ai * bq - aq * bi;
		ra += ai * ai + aq * aq;
		rb += bi * bi + bq * bq;
	}
	for (int t = t0;; t += st) {
		float fr = (float)pr, fi = (float)pi;
		float num = fr * fr + fi * fi, den = (float)ra * (float)rb;

		if (st == 1) {
			rnum[t & MASK] = num;
			rden[t & MASK] = den;
		}
		/* num / den > b->num / b->den */
		if (num * b->den > b->num * den) {
			b->num = num;
			b->den = den;
			b->pr = pr;
			b->pi = pi;
			b->t = b->start = t;
			for (int k = 1; st == 1 && k <= cp && t - k >= t0; k++) {
				int j = (t - k) & MASK;

				if (rnum[j] * b->den < PLATEAU * b->num * rden[j]) {
					break;
				}
				b->start = t - k;
			}
		}
		if (t + st > t1) {
			break;
		}
		{
			uint32_t wa = w[t], wb = w[t + L], wc = w[t + 2 * L];
			int ai = esp_sdr_rx_i(wa) - dci, aq = esp_sdr_rx_q(wa) - dcq;
			int bi = esp_sdr_rx_i(wb) - dci, bq = esp_sdr_rx_q(wb) - dcq;
			int ci = esp_sdr_rx_i(wc) - dci, cq = esp_sdr_rx_q(wc) - dcq;
			int eb = bi * bi + bq * bq;

			pr += (bi * ci + bq * cq) - (ai * bi + aq * bq);
			pi += (bi * cq - bq * ci) - (ai * bq - aq * bi);
			ra += eb - (ai * ai + aq * aq);
			rb += (ci * ci + cq * cq) - eb;
		}
	}
}

/*
 * The frame start in t = 0 .. tmax (tlim: the last start a whole frame fits): a coarse scan on every 2nd or 4th sample
 * (both the position and the correlation, which keeps at least 32 terms),
 * then a full resolution one over the prefix around its peak.
 */
/* A coarse peak this clear (normalized metric) needs no finer coarse scan. */
#define COARSE_CLEAR 0.7f

static int search(struct ofdm_ctx *ctx, const uint32_t *w, int tmax, int tlim, int period,
		  int dci, int dcq, float *best_out, float *ang_out)
{
	int L = (int)ctx->rnfft / 2, st = L >= 128 ? 4 : (L >= 64 ? 2 : 1), lo, hi;
	struct sc_best c, f;

	/*
	 * Twice the stride first: at a good SNR its peak is clear and the scan
	 * costs half. A weak one (fewer correlation terms are noisier) is
	 * scanned again at the normal stride.
	 */
	if (L / (2 * st) >= 16) {
		sc_scan(ctx, w, 0, tmax, 2 * st, dci, dcq, &c);
		if (c.t < 0 || c.num < COARSE_CLEAR * c.den) {
			sc_scan(ctx, w, 0, tmax, st, dci, dcq, &c);
		} else {
			st *= 2;
		}
	} else {
		sc_scan(ctx, w, 0, tmax, st, dci, dcq, &c);
	}
	if (c.t < 0) {
		return -1;
	}
	/*
	 * The coarse peak is somewhere on the plateau, give or take a step. Near
	 * the window's beginning the plateau may be cut (the copy began before
	 * the window, its edge would come out late): look at the next copy, one
	 * period on, instead; up to tlim it lies whole in the window.
	 */
	lo = c.t - (int)ctx->rcp - st;
	if (lo < 0 && c.t + period + st <= tlim) {
		c.t += period;
		lo += period;
		tmax = tlim;
	}
	hi = c.t + st;
	lo = lo < 0 ? 0 : lo;
	hi = hi > tmax ? tmax : hi;
	sc_scan(ctx, w, lo, hi, 1, dci, dcq, &f);
	*best_out = f.den > 0.0f ? f.num / f.den : 0.0f;
	*ang_out = atan2f((float)f.pi, (float)f.pr);
	return f.start;
}

/*
 * Derotation over one receive FFT window: rot[i] = exp(-j 2 pi eps i). The
 * phase at each window's first sample is the same for all its subcarriers, so
 * it goes into the tracked phase instead (rx.u).
 */
static void rot_init(struct ofdm_ctx *ctx, float eps)
{
	float sr = cosf(-TWO_PI * eps), si = sinf(-TWO_PI * eps), rr = 1.0f, ri = 0.0f;

	for (unsigned int i = 0; i < ctx->rnfft; i++) {
		float t;

		/* Exact every 64 samples, steps in between. */
		if ((i & 63U) == 0U) {
			rr = cosf(-TWO_PI * eps * (float)i);
			ri = sinf(-TWO_PI * eps * (float)i);
		}
		ctx->rot[i] = (struct ofdm_cf){rr, ri};
		t = rr * sr - ri * si;
		ri = rr * si + ri * sr;
		rr = t;
	}
}

/*
 * Slot s of the frame at rx.start into the FFT buffer, DC removed and
 * derotated, the window cp/4 early inside the prefix (a late start then still
 * stays clear of the next symbol), and the FFT. Steps rx.u to this slot:
 * against the first pilot's, slot s comes out turned by exp(+j 2 pi eps rslot
 * (s - 1)), which u = exp(-j 2 pi eps rslot (s - 1)) undoes.
 */
OFDM_HOT static void take_slot(const struct ofdm_ctx *ctx, struct ofdm_worker *wk,
			       const uint32_t *w, unsigned int s)
{
	const uint32_t *__restrict x = &w[ctx->rx.start + (int)(s * ctx->rslot) + (int)ctx->rcp -
					  (int)(ctx->rcp / 4U)];
	const struct ofdm_cf *__restrict r = ctx->rot;
	struct ofdm_cf *__restrict b = wk->buf;
	unsigned int n = ctx->rnfft;
	int dci = ctx->rx.dci, dcq = ctx->rx.dcq;

	for (unsigned int i = 0; i < n; i++) {
		uint32_t v = x[i];
		float xr = (float)(esp_sdr_rx_i(v) - dci), xq = (float)(esp_sdr_rx_q(v) - dcq);
		float rr = r[i].re, ri = r[i].im;

		b[i].re = xr * rr - xq * ri;
		b[i].im = xr * ri + xq * rr;
	}
	fft(ctx, wk->buf, n);
	if (s > 1U) {
		struct ofdm_cf u = wk->u, us = ctx->rx.us;

		wk->u = (struct ofdm_cf){u.re * us.re - u.im * us.im, u.re * us.im + u.im * us.re};
	}
}

/* Decided bits, packed MSB first and written a byte at a time (the buffer may be in PSRAM). */
struct bitsink {
	uint8_t *out;
	uint32_t acc;
	unsigned int n;
};

static inline void sink_put(struct bitsink *k, unsigned int v, unsigned int nb)
{
	k->acc = (k->acc << nb) | v;
	k->n += nb;
	if (k->n >= 8U) {
		k->n -= 8U;
		*k->out++ = (uint8_t)(k->acc >> k->n);
	}
}

static inline void sink_flush(struct bitsink *k)
{
	if (k->n > 0U) {
		*k->out = (uint8_t)(k->acc << (8U - k->n));
		k->n = 0;
	}
}

/* Subcarrier a of the worker's FFT output, equalized and rotated by (cr, ci). */
static inline struct ofdm_cf eq_rot(const struct ofdm_ctx *ctx, const struct ofdm_worker *wk,
				    unsigned int a, float cr, float ci)
{
	struct ofdm_cf y = wk->buf[ctx->binr[a]], e = ctx->eq[a];
	float zr = y.re * e.re - y.im * e.im, zi = y.re * e.im + y.im * e.re;

	return (struct ofdm_cf){zr * cr - zi * ci, zr * ci + zi * cr};
}

static inline struct ofdm_cf cmul(struct ofdm_cf a, struct ofdm_cf b)
{
	return (struct ofdm_cf){a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re};
}

/* Gain of the rate update: a residual offset shows as a steady step in the measured phase. */
#define RATE_GAIN 0.25f

/*
 * The rotation for the symbol in the worker's buffer: the tracked phase advanced by its
 * rate, and this window's known phase; then the symbol's own residual
 * rotation (against its decisions at @p mod) corrects the phase in full and
 * the rate in part (a second order loop: a residual carrier offset is
 * followed without lag).
 */
OFDM_HOT static struct ofdm_cf track(const struct ofdm_ctx *ctx, struct ofdm_worker *wk,
				     enum ofdm_mod mod)
{
	struct ofdm_worker *rx = wk;
	struct ofdm_cf p = cmul(rx->trk, rx->rate), r = cmul(p, rx->u);
	float ar = 0.0f, ai = 0.0f, m;

	for (unsigned int a = 0; a < ctx->cfg.channels; a++) {
		struct ofdm_cf z = eq_rot(ctx, wk, a, r.re, r.im), d;

		(void)decide(mod, z, &d);
		ar += z.re * d.re + z.im * d.im;
		ai += z.im * d.re - z.re * d.im;
	}
	m = sqrtf(ar * ar + ai * ai);
	if (m > 0.0f) {
		struct ofdm_cf c = {ar / m, -ai / m}, q;
		float qm;

		p = cmul(p, c);
		r = cmul(r, c);
		/* Rate: a step of the way towards it times this correction. */
		q = (struct ofdm_cf){1.0f + RATE_GAIN * (c.re - 1.0f), RATE_GAIN * c.im};
		q = cmul(rx->rate, q);
		qm = 1.0f / sqrtf(q.re * q.re + q.im * q.im);
		rx->rate = (struct ofdm_cf){q.re * qm, q.im * qm};
	}
	rx->trk = p;
	return r;
}

/* One payload symbol: rotation, decisions into @p k and their error. */
OFDM_HOT static void data_symbol(const struct ofdm_ctx *ctx, struct ofdm_worker *wk,
				 enum ofdm_mod mod, struct bitsink *k, float *err)
{
	struct ofdm_cf r = track(ctx, wk, mod);
	unsigned int c = ctx->cfg.channels, bps = ofdm_mod_bits(mod);
	float e2 = 0.0f;

	for (unsigned int a = 0; a < c; a++) {
		struct ofdm_cf z = eq_rot(ctx, wk, a, r.re, r.im), d;
		unsigned int v = decide(mod, z, &d);

		e2 += (z.re - d.re) * (z.re - d.re) + (z.im - d.im) * (z.im - d.im);
		sink_put(k, v, bps);
	}
#ifdef OFDM_DEBUG
	{
		extern void ofdm_debug_symbol(float e2, unsigned int c, struct ofdm_cf r);

		ofdm_debug_symbol(e2, c, r);
	}
#endif
	*err += e2;
}

/*
 * Channel per subcarrier from the pilots: each one's known sign undone and its
 * window phase (rx.u) too, averaged; optionally smoothed over the neighbours
 * (3 taps 1 2 1, after taking out the phase slope between subcarriers that a
 * timing offset puts there); then conj(H) / |H|^2.
 */
static void channel(struct ofdm_ctx *ctx, const uint32_t *w)
{
	unsigned int c = ctx->cfg.channels, np = ctx->cfg.pilots;
	struct ofdm_cf *h = ctx->eq, rho = {0.0f, 0.0f}, rho2;
	float g = 1.0f / (float)np;

	struct ofdm_cf dr = {0.0f, 0.0f};

	memset(h, 0, c * sizeof(h[0]));
	for (unsigned int p = 0; p < np; p++) {
		struct ofdm_cf u;

		take_slot(ctx, &ctx->w0, w, 1U + p);
		u = ctx->w0.u;
		for (unsigned int a = 0; a < c; a++) {
			struct ofdm_cf y = cmul(ctx->w0.buf[ctx->binr[a]], u);
			float s = pn_sign(PILOT_SEED, a);

			/* Pilot p against the sum of the ones before: the phase step per slot. */
			if (p > 0U) {
				struct ofdm_cf q = h[a];

				dr.re += s * (y.re * q.re + y.im * q.im);
				dr.im += s * (y.im * q.re - y.re * q.im);
			}
			h[a].re += s * g * y.re;
			h[a].im += s * g * y.im;
		}
	}
	/*
	 * A residual carrier offset turns each pilot by the same step theta: the
	 * tracking starts undoing it, rate exp(-j theta) per slot, from the
	 * channel's reference halfway through the pilots (the average).
	 */
	{
		float th = (dr.re != 0.0f || dr.im != 0.0f) ? atan2f(dr.im, dr.re) : 0.0f;
		float a = -0.5f * th * (float)(np - 1U);

		ctx->w0.rate = (struct ofdm_cf){cosf(th), -sinf(th)};
		ctx->w0.trk = (struct ofdm_cf){cosf(a), sinf(a)};
	}
	/*
	 * The phase step between neighbouring subcarriers: a window d samples
	 * early in the prefix turns subcarrier k by -2 pi k d / N, the channel's
	 * delay adds to it. Kept as rx.delay for the fine timing.
	 */
	for (unsigned int a = 0; a + 1U < c; a++) {
		if (sub_freq(c, a + 1U) - sub_freq(c, a) == 1) {
			rho.re += h[a + 1].re * h[a].re + h[a + 1].im * h[a].im;
			rho.im += h[a + 1].im * h[a].re - h[a + 1].re * h[a].im;
		}
	}
	ctx->rx.delay = (rho.re != 0.0f || rho.im != 0.0f)
				? -atan2f(rho.im, rho.re) * (float)ctx->rnfft / TWO_PI
				: 0.0f;
	if (ctx->cfg.smooth) {
		struct ofdm_cf *hs = ctx->w0.buf;
		float m;

		m = sqrtf(rho.re * rho.re + rho.im * rho.im);
		rho = m > 0.0f ? (struct ofdm_cf){rho.re / m, rho.im / m} : (struct ofdm_cf){1, 0};
		rho2 = (struct ofdm_cf){rho.re * rho.re - rho.im * rho.im, 2.0f * rho.re * rho.im};
		for (unsigned int a = 0; a < c; a++) {
			struct ofdm_cf acc = {2.0f * h[a].re, 2.0f * h[a].im};
			float ws = 2.0f;

			/* A neighbour df subcarriers away, turned back by rho^-df. */
			if (a > 0U) {
				struct ofdm_cf r = sub_freq(c, a) - sub_freq(c, a - 1U) == 1 ? rho : rho2;

				acc.re += h[a - 1].re * r.re - h[a - 1].im * r.im;
				acc.im += h[a - 1].re * r.im + h[a - 1].im * r.re;
				ws += 1.0f;
			}
			if (a + 1U < c) {
				struct ofdm_cf r = sub_freq(c, a + 1U) - sub_freq(c, a) == 1 ? rho : rho2;

				acc.re += h[a + 1].re * r.re + h[a + 1].im * r.im;
				acc.im += h[a + 1].im * r.re - h[a + 1].re * r.im;
				ws += 1.0f;
			}
			hs[a] = (struct ofdm_cf){acc.re / ws, acc.im / ws};
		}
		memcpy(h, hs, c * sizeof(h[0]));
	}
	for (unsigned int a = 0; a < c; a++) {
		float h2 = h[a].re * h[a].re + h[a].im * h[a].im;

		h[a] = h2 > 0.0f ? (struct ofdm_cf){h[a].re / h2, -h[a].im / h2}
				 : (struct ofdm_cf){0.0f, 0.0f};
	}
}

int ofdm_rx_begin(struct ofdm_ctx *ctx, const uint32_t *words, size_t n, uint8_t *hdr,
		  struct ofdm_rx_info *info)
{
	return ofdm_rx_begin_period(ctx, words, n, ctx->rlen, hdr, info);
}

int ofdm_rx_begin_period(struct ofdm_ctx *ctx, const uint32_t *words, size_t n, size_t period,
			 uint8_t *hdr, struct ofdm_rx_info *info)
{
	struct ofdm_rxstate *rx = &ctx->rx;
	int32_t si = 0, sq = 0, cnt = 0;
	int per = (int)period, tmax = (int)n - per;
	float best = 0.0f, ang = 0.0f, a;
	unsigned int c = ctx->cfg.channels, nh = ctx->cfg.hdr_bits, h0 = 1U + ctx->cfg.pilots;
	uint32_t t0 = clk(), t1;

	memset(info, 0, sizeof(*info));
	if (tmax < 0) {
		return -1;
	}
	/* The frame repeats every rlen samples: one period (plus its prefix plateau) holds a start. */
	if (tmax > per + (int)ctx->rcp) {
		tmax = per + (int)ctx->rcp;
	}
	for (size_t k = 0; k < n; k += 4) {
		si += esp_sdr_rx_i(words[k]);
		sq += esp_sdr_rx_q(words[k]);
		cnt++;
	}
	rx->dci = (si + (si >= 0 ? cnt / 2 : -cnt / 2)) / cnt;
	rx->dcq = (sq + (sq >= 0 ? cnt / 2 : -cnt / 2)) / cnt;
	rx->start = search(ctx, words, tmax, (int)n - per, per, rx->dci, rx->dcq, &best, &ang);
	t1 = clk();
	info->prof[0] = t1 - t0;
	info->metric = best;
	if (rx->start < 0 || best < METRIC_MIN) {
		return -1;
	}
	/* Phase over half a symbol: pi per subcarrier spacing. */
	info->cfo_frac = ang / PI;
	info->cfo_hz = info->cfo_frac * ctx->spacing_hz;
	info->start = rx->start;
	/* Cycles per receive sample. */
	rx->eps = info->cfo_frac / (float)ctx->rnfft;
	rot_init(ctx, rx->eps);
	a = -TWO_PI * rx->eps * (float)ctx->rslot;
	rx->us = (struct ofdm_cf){cosf(a), sinf(a)};
	ctx->w0.u = (struct ofdm_cf){1.0f, 0.0f};

	channel(ctx, words);
	/*
	 * Fine timing from the pilots: the window should sit in the middle of
	 * the prefix, where a receive filter's ringing on either side clears it
	 * (the preamble's plateau edge is only good to some samples). Moved,
	 * and the channel measured again, if it is a sample or more off.
	 */
	{
		/* Later by how much more than half the prefix the window is early. */
		int shift = (int)lrintf(rx->delay - (float)ctx->rcp / 2.0f);

		if (shift != 0 && shift > -(int)ctx->rcp && shift < (int)ctx->rcp &&
		    rx->start + shift >= 0 && rx->start + shift + per <= (int)n) {
			rx->start += shift;
			info->start = rx->start;
			ctx->w0.u = (struct ofdm_cf){1.0f, 0.0f};
			channel(ctx, words);
		}
	}

	/* Header: BPSK, its bits repeated over the subcarriers and summed soft. */
	memset(rx->soft, 0, sizeof(rx->soft));
	for (unsigned int s = 0; s < ctx->nhdr; s++) {
		struct ofdm_cf r;

		take_slot(ctx, &ctx->w0, words, h0 + s);
		r = track(ctx, &ctx->w0, OFDM_BPSK);
		for (unsigned int k = 0; k < c; k++) {
			rx->soft[(s * c + k) % nh] += eq_rot(ctx, &ctx->w0, k, r.re, r.im).re;
		}
	}
	if (nh > 0U) {
		memset(hdr, 0, (nh + 7U) / 8U);
		for (unsigned int k = 0; k < nh; k++) {
			if (rx->soft[k] > 0.0f) {
				hdr[k >> 3] |= (uint8_t)(0x80U >> (k & 7U));
			}
		}
	}
	rx->slot = h0 + ctx->nhdr;
	info->prof[1] = clk() - t1;
	return 0;
}

/* Slot of payload symbol s (after the mid pilot one further). */
static inline unsigned int data_slot(const struct ofdm_ctx *ctx, unsigned int s)
{
	return ctx->rx.slot + s + (ctx->cfg.mid_pilot && s >= ctx->mid ? 1U : 0U);
}

void ofdm_rx_fork(const struct ofdm_ctx *ctx, struct ofdm_worker *wk, const uint32_t *words)
{
	unsigned int ps = ctx->rx.slot + ctx->mid;
	/* The window phase before the pilot's slot (take_slot() steps it once). */
	float a = -TWO_PI * ctx->rx.eps * (float)ctx->rslot * (float)(ps - 2U);
	struct ofdm_cf acc = {0.0f, 0.0f};
	float m;

	wk->u = (struct ofdm_cf){cosf(a), sinf(a)};
	wk->rate = ctx->w0.rate;
	take_slot(ctx, wk, words, ps);
	/* The pilot equalized: its phase against the channel's is the one to undo. */
	for (unsigned int k = 0; k < ctx->cfg.channels; k++) {
		struct ofdm_cf z = eq_rot(ctx, wk, k, wk->u.re, wk->u.im);
		float sg = pn_sign(PILOT_SEED, k);

		acc.re += sg * z.re;
		acc.im += sg * z.im;
	}
	m = sqrtf(acc.re * acc.re + acc.im * acc.im);
	wk->trk = m > 0.0f ? (struct ofdm_cf){acc.re / m, -acc.im / m} : (struct ofdm_cf){1, 0};
}

void ofdm_rx_part(const struct ofdm_ctx *ctx, struct ofdm_worker *wk, const uint32_t *words,
		  enum ofdm_mod mod, unsigned int s0, unsigned int s1, uint8_t *bits, float *err)
{
	struct bitsink k = {.out = bits};

	for (unsigned int s = s0; s < s1; s++) {
		take_slot(ctx, wk, words, data_slot(ctx, s));
		data_symbol(ctx, wk, mod, &k, err);
	}
	sink_flush(&k);
}

float ofdm_rx_mer(const struct ofdm_ctx *ctx, float err)
{
	return 10.0f * log10f((float)ctx->ndata * (float)ctx->cfg.channels /
			      (err > 1e-12f ? err : 1e-12f));
}

void ofdm_rx_finish(struct ofdm_ctx *ctx, const uint32_t *words, enum ofdm_mod mod, uint8_t *bits,
		    struct ofdm_rx_info *info)
{
	float err = 0.0f;
	uint32_t t0 = clk();

	ofdm_rx_part(ctx, &ctx->w0, words, mod, 0, ctx->mid, bits, &err);
	if (ctx->mid < ctx->ndata) {
		/* On one CPU too: re-anchored on the mid pilot. */
		size_t off = (size_t)ctx->mid * ctx->cfg.channels * ofdm_mod_bits(mod) / 8U;

		ofdm_rx_fork(ctx, &ctx->w0, words);
		ofdm_rx_part(ctx, &ctx->w0, words, mod, ctx->mid, ctx->ndata, bits + off, &err);
	}
	info->prof[2] = clk() - t0;
	info->mer_db = ofdm_rx_mer(ctx, err);
}
