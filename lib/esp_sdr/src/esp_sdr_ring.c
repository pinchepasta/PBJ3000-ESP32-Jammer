/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Continuous receive (esp_sdr_ring.h), ported from upstream esp-sdr
 * (main/common/ring_capture.c, its S3 IQ stream; rotation scheme after
 * h0m3us3r/eSpDR capture.c), reduced to one core: ring control and filter
 * slices alternate between polls of the write index.
 *
 * The dump engine writes I/Q pairs at a ring index that advances
 * continuously (mod 16384) into whichever bank is selected. Switching the
 * bank while it runs splits the stream into gapless units, one per visit:
 *  - while the engine fills bank b, bank b + 1 gets sentinels where the next
 *    unit is predicted to start and to end;
 *  - once a unit length of pairs is in bank b, the engine moves to bank b + 1;
 *  - the finished unit's first and last pair are found by binary search in
 *    the sentinel windows, and each unit must start where the previous one
 *    ended, which proves the stream gapless.
 * The CPUs cannot read the bank the engine owns, so the filter runs on
 * finished units. A unit still unfiltered when its bank comes round again is
 * abandoned: a jump in the output index.
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <esp_attr.h>

#include <esp_sdr/esp_sdr.h>
#include <esp_sdr/esp_sdr_ring.h>

#include "esp_sdr_priv.h"

#define HOT IRAM_ATTR __attribute__((optimize("O3")))

#define DUMP_WRITE_INDEX_REG 0x60033d60U
/* Circular 16384-pair ring, I/Q source 0. */
#define DUMP_CTRL_CIRCULAR   0x00024000U

#define RING_BANKS     3U
#define RING_PAIRS     16384U
#define RING_MASK      (RING_PAIRS - 1U)
/* Unit 0: its whole bank holds sentinels; stay clear of its start. */
#define UNIT0_END_WIN  1024U
#define NOT_FOUND      0xffffffffU
#define RING_SENTINEL  0xa5c33c5aU

/* Filter work per slice at most, and the input chunk through both stages. */
#define SLICE_PAIRS 1024U
#define FIR_CHUNK   512U

/*
 * The unit geometry of a run, from its unit length (esp_sdr_ring_cfg):
 * the engine moves on after thr pairs, at most late pairs late; sentinels
 * cover sg pairs from a unit's earliest start and eg from its earliest end.
 * The default unit: thr 12288, sg 3072, eg 4096, late 2000.
 */
struct geom {
	uint32_t thr, sg, eg, late;
	/* Unit 0: start window, shortest length. */
	uint32_t sg0, min0;
	uint32_t slice;
};

static int geom_init(struct geom *g, uint32_t unit)
{
	if (unit == 0U) {
		unit = ESP_SDR_RING_UNIT_DEFAULT;
	}
	if (unit < ESP_SDR_RING_UNIT_MIN || unit > ESP_SDR_RING_UNIT_DEFAULT || unit % 64U != 0U) {
		return -EINVAL;
	}
	g->thr = unit;
	/* A start window longer than the unit would overlap the end window. */
	g->sg = MIN(3072U, unit);
	g->late = MIN(2000U, g->sg - 128U);
	g->eg = MIN(4096U, 2U * g->late + 128U);
	g->sg0 = 1024U + MIN(2048U, unit / 2U);
	g->min0 = unit - MIN(1024U, unit / 2U);
	g->slice = MIN(SLICE_PAIRS, unit / 4U);
	if (g->late + 64U >= g->sg || 2U * g->late + 64U >= g->eg ||
	    g->sg + g->thr > RING_PAIRS || g->eg + g->thr > RING_PAIRS) {
		return -EINVAL;
	}
	return 0;
}

/*
 * Two banks: a bank gets the next unit but one while the unit before it may
 * still be filtered, so the next unit's windows (up to two units and the
 * end window ahead) must stay clear of where the previous visit began (a
 * unit and its lateness back), mod the ring.
 */
static bool geom_two_banks(const struct geom *g)
{
	return 3U * g->thr + g->eg + g->late <= RING_PAIRS;
}

#define PI_F 3.14159265f

extern uint32_t __esp_sdr_bank0_start[];

#if defined(CONFIG_SMP)
#include <esp_mp_stall.h>
#include <soc/system_reg.h>

/*
 * A flash operation on the other CPU parks this one with an IPI (cache off)
 * and panics if it is not acknowledged: the run keeps interrupts masked, so
 * answer it from the poll loop. The run usually fails late afterwards.
 */
static ALWAYS_INLINE bool stall_poll(void)
{
	uint32_t reg = arch_curr_cpu()->id == 0 ? SYSTEM_CPU_INTR_FROM_CPU_2_REG
						 : SYSTEM_CPU_INTR_FROM_CPU_3_REG;

	if (REG_READ(reg) == 0U) {
		return false;
	}
	esp_mp_stall_isr(NULL);
	return true;
}
#else
static ALWAYS_INLINE bool stall_poll(void)
{
	return false;
}
#endif

static atomic_t stop_req;
static atomic_t active;

static inline uint32_t ccount(void)
{
	uint32_t c;

	__asm__ volatile("rsr.ccount %0" : "=r"(c));
	return c;
}

static inline uint32_t *bank_ptr(unsigned int b)
{
	return __esp_sdr_bank0_start + b * (DUMP_BANK_SIZE / sizeof(uint32_t));
}

static inline uint32_t write_index(void)
{
	return REG_READ(DUMP_WRITE_INDEX_REG) & RING_MASK;
}

static inline void select_bank(uint32_t saved, uint32_t mask)
{
	REG_WRITE(SENSITIVE_INTERNAL_SRAM_USAGE_3_REG,
		  (saved & ~SENSITIVE_INTERNAL_SRAM_MAC_DUMP_USAGE_M) |
			  (mask << SENSITIVE_INTERNAL_SRAM_MAC_DUMP_USAGE_S));
}

/* ---- sentinel windows ---- */

/* Ring indices [at, at + count) of bank b, wrapping; 128-bit stores (eSpDR). */
HOT static void fill_sentinels(unsigned int b, unsigned int at, unsigned int count)
{
	at &= RING_MASK;
	while (count != 0U) {
		unsigned int n = MIN(RING_PAIRS - at, count);
		uint32_t *p = bank_ptr(b) + at;
		unsigned int head = MIN((-at) & 3U, n);
		unsigned int vectors;

		for (unsigned int i = 0; i < head; i++) {
			*p++ = RING_SENTINEL;
		}
		vectors = (n - head) / 4U;
		if (vectors != 0U) {
			uint32_t v = RING_SENTINEL;

			__asm__ volatile("ee.movi.32.q q6, %[v], 0\n"
					 "ee.movi.32.q q6, %[v], 1\n"
					 "ee.movi.32.q q6, %[v], 2\n"
					 "ee.movi.32.q q6, %[v], 3\n"
					 "loopnez %[n], 1f\n"
					 "ee.vst.128.ip q6, %[p], 16\n"
					 "1:"
					 : [p] "+a"(p)
					 : [v] "a"(v), [n] "a"(vectors)
					 : "memory");
		}
		for (unsigned int i = head + vectors * 4U; i < n; i++) {
			*p++ = RING_SENTINEL;
		}
		count -= n;
		at = 0;
	}
	__asm__ volatile("memw" ::: "memory");
}

/* First written pair in [origin, origin + guard): sentinels up to the unit start. */
HOT static uint32_t find_first(unsigned int b, unsigned int origin, unsigned int guard)
{
	const uint32_t *p = bank_ptr(b);
	unsigned int lo = 0, hi = guard - 1U;

	if (p[origin & RING_MASK] != RING_SENTINEL ||
	    p[(origin + guard - 1U) & RING_MASK] == RING_SENTINEL) {
		return NOT_FOUND;
	}
	while (lo < hi) {
		unsigned int mid = (lo + hi) / 2U;

		if (p[(origin + mid) & RING_MASK] == RING_SENTINEL) {
			lo = mid + 1U;
		} else {
			hi = mid;
		}
	}
	return (origin + lo) & RING_MASK;
}

/* One past the last written pair in [origin, origin + guard), from the last index seen. */
HOT static uint32_t find_end(unsigned int b, unsigned int origin, unsigned int wi,
			     unsigned int guard)
{
	const uint32_t *p = bank_ptr(b);
	unsigned int lo = (wi - origin) & RING_MASK, hi = guard - 1U;

	if (lo >= guard || p[(origin + guard - 1U) & RING_MASK] != RING_SENTINEL) {
		return NOT_FOUND;
	}
	while (lo < hi) {
		unsigned int mid = (lo + hi) / 2U;

		if (p[(origin + mid) & RING_MASK] == RING_SENTINEL) {
			hi = mid;
		} else {
			lo = mid + 1U;
		}
	}
	return (origin + lo) & RING_MASK;
}

/*
 * ---- two-stage decimating FIR (split I/Q arrays, vector unit) ----
 *
 * D = 8 x (D / 8), D >= 64: every window starts on a multiple of 8 samples,
 * 16-byte aligned. Stage 1: 32 taps, an 8-sample boxcar squared (double
 * zeros on every multiple of fs/8, the bands that alias onto 0 Hz) convolved
 * with an 18-tap Kaiser low pass. Stage 2: 18 D / 8 taps (at most 256),
 * Kaiser beta 5.65 (~60 dB), pass band 0.4 and stop band 0.6 of the output
 * rate. Each stage keeps L samples of history, so the output is continuous
 * across slices and units. Scale: ring << 6, stage 1 x 16, stage 2 x 32.
 *
 * Upstream's lanes take the low field of a word as I. Receive words carry Q
 * there, so the filter works on j conj(z); swapping the outputs undoes it,
 * and its +fs/4 rotation becomes a -fs/4 one.
 */
struct fir_job {
	const int16_t *xi, *xq, *h;
	uint32_t groups, nout, step, shift;
	int16_t *oi, *oq;
};

void esp_sdr_ring_unpack_iq10(const uint32_t *src, int16_t *di, int16_t *dq, unsigned int groups8);
void esp_sdr_ring_fir_split(const struct fir_job *job);
void esp_sdr_ring_fir8_l32(const struct fir_job *job);
void esp_sdr_ring_rot_fs4(int16_t *wi, int16_t *wq, unsigned int groups8, const int16_t *masks);
/* esp_sdr_ring_fir_split with the data loads fused, for an even number of tap groups. */
void esp_sdr_ring_dot(const struct fir_job *job);

#define FS1_L   32U
#define FS2_LMAX 256U
/* Two windows, a chunk, and room for the dot product's look-ahead load. */
#define FS_CAP(l, in) (2U * (l) + (in) + 16U)
#define FS1_CAP FS_CAP(FS1_L, FIR_CHUNK)
#define FS2_CAP FS_CAP(FS2_LMAX, FIR_CHUNK / 8U)
#define OUT_MAX 80U

struct fstage {
	/* Decimation, taps, samples in wi/wq, accumulator shift. */
	uint32_t d, l, n, shift;
	/* Input index of wi[0]; last input index of the next output. */
	uint64_t base, next_end;
	int16_t *h, *wi, *wq;
};

/* Internal RAM for the vector loads: the high region above the capture bank. */
static int16_t fs1_h[FS1_L] __aligned(16) ESP_SDR_HIGH_RAM;
static int16_t fs1_wi[FS1_CAP] __aligned(16) ESP_SDR_HIGH_RAM;
static int16_t fs1_wq[FS1_CAP] __aligned(16) ESP_SDR_HIGH_RAM;
static int16_t fs2_h[FS2_LMAX] __aligned(16) ESP_SDR_HIGH_RAM;
static int16_t fs2_wi[FS2_CAP] __aligned(16) ESP_SDR_HIGH_RAM;
static int16_t fs2_wq[FS2_CAP] __aligned(16) ESP_SDR_HIGH_RAM;
static int16_t out_i[OUT_MAX] __aligned(16) ESP_SDR_HIGH_RAM;
static int16_t out_q[OUT_MAX] __aligned(16) ESP_SDR_HIGH_RAM;

static struct fstage fs1 = {.h = fs1_h, .wi = fs1_wi, .wq = fs1_wq};
static struct fstage fs2 = {.h = fs2_h, .wi = fs2_wi, .wq = fs2_wq};
static bool fir_rot;
static const int16_t rot_masks[24] __aligned(16) = {
	1, 0, -1, 0, 1, 0, -1, 0, 0, -1, 0, 1, 0, -1, 0, 1, 0, 1, 0, -1, 0, 1, 0, -1,
};

static float bessel_i0(float x)
{
	float s = 1.0f, t = 1.0f;

	for (int k = 1; k < 30; k++) {
		float q = x / (2.0f * (float)k);

		t *= q * q;
		s += t;
	}
	return s;
}

static float kaiser_tap(uint32_t i, uint32_t l, float fc)
{
	float m = (float)i - 0.5f * (float)(l - 1U);
	float x = 2.0f * fc * m;
	float sinc = fabsf(x) < 1e-6f ? 1.0f : sinf(PI_F * x) / (PI_F * x);
	float r = 2.0f * m / (float)(l - 1U);

	return 2.0f * fc * sinc * bessel_i0(5.65f * sqrtf(fmaxf(0.0f, 1.0f - r * r))) /
	       bessel_i0(5.65f);
}

static void fstage_init(struct fstage *s, uint32_t d, uint32_t l, uint32_t shift)
{
	float fc = 0.5f / (float)d, sum = 0.0f;
	float h[FS2_LMAX];

	s->d = d;
	s->l = l;
	s->shift = shift;
	if (d == 8U && l == FS1_L) {
		float k[18];

		for (uint32_t i = 0; i < 18U; i++) {
			k[i] = kaiser_tap(i, 18U, fc);
		}
		for (uint32_t i = 0; i < l; i++) {
			h[i] = 0.0f;
		}
		for (uint32_t t = 0; t < 15U; t++) {
			float tri = (float)(t < 8U ? t + 1U : 15U - t);

			for (uint32_t i = 0; i < 18U; i++) {
				h[t + i] += tri * k[i];
			}
		}
	} else {
		for (uint32_t i = 0; i < l; i++) {
			h[i] = kaiser_tap(i, l, fc);
		}
	}
	for (uint32_t i = 0; i < l; i++) {
		sum += h[i];
	}
	for (uint32_t i = 0; i < l; i++) {
		s->h[i] = (int16_t)roundf(32768.0f * h[i] / sum);
	}
}

static void fstage_reset(struct fstage *s, uint64_t start)
{
	memset(s->wi, 0, 2U * s->l);
	memset(s->wq, 0, 2U * s->l);
	s->n = s->l;
	s->base = start - s->l;
	s->next_end = start + s->d - 1U;
}

/* All complete outputs to oi/oq; returns their count; drops consumed history. */
HOT static uint32_t fstage_run(struct fstage *s, int16_t *oi, int16_t *oq)
{
	uint32_t no = 0, keep;

	if (s->next_end < s->base + s->n) {
		uint32_t off;
		struct fir_job j;

		no = (uint32_t)(s->base + s->n - 1U - s->next_end) / s->d + 1U;
		off = (uint32_t)(s->next_end + 1U - s->l - s->base);
		j = (struct fir_job){s->wi + off, s->wq + off, s->h, s->l / 8U, no,
				     2U * s->d, s->shift, oi, oq};
		if (s->d == 8U && s->l == FS1_L) {
			esp_sdr_ring_fir8_l32(&j);
		} else if ((j.groups & 1U) == 0U) {
			esp_sdr_ring_dot(&j);
		} else {
			esp_sdr_ring_fir_split(&j);
		}
		s->next_end += (uint64_t)no * s->d;
	}
	/*
	 * Drop consumed samples only when the rest (< L) does not overlap them,
	 * so a plain memcpy does. Without a drop n stays below 2 L.
	 */
	keep = (uint32_t)(s->next_end + 1U - s->l - s->base);
	if (keep != 0U && keep >= s->n - keep) {
		memcpy(s->wi, s->wi + keep, 2U * (s->n - keep));
		memcpy(s->wq, s->wq + keep, 2U * (s->n - keep));
		s->base += keep;
		s->n -= keep;
	}
	return no;
}

/* One ring word into stage 1 (the unaligned head and tail of a chunk). */
HOT static void fir_scalar(uint32_t w)
{
	int16_t i = (int16_t)(((int32_t)(w << 22) >> 22) * 64);
	int16_t q = (int16_t)(((int32_t)(w << 12) >> 22) * 64);

	if (fir_rot) {
		/* z j^n */
		switch ((uint32_t)(fs1.base + fs1.n) & 3U) {
		case 1: {
			int16_t t = i;

			i = (int16_t)-q;
			q = t;
			break;
		}
		case 2:
			i = (int16_t)-i;
			q = (int16_t)-q;
			break;
		case 3: {
			int16_t t = i;

			i = q;
			q = (int16_t)-t;
			break;
		}
		default:
			break;
		}
	}
	fs1.wi[fs1.n] = i;
	fs1.wq[fs1.n] = q;
	fs1.n++;
}

/* Run state, reset by esp_sdr_ring_run(). */
struct unit {
	uint32_t bank, first, count, pos;
	/* Ring pair index of the unit's first pair. */
	uint64_t index;
	bool started;
};

static struct {
	const struct esp_sdr_ring_cfg *cfg;
	struct esp_sdr_ring_stats *st;
	uint32_t log2d;
	/* Pair index the filter continues from (~0: none, realign). */
	uint64_t next_pair;
	/* Output index of the next sample. */
	uint64_t out_index;
	/* Finished units waiting for the filter, oldest first. */
	struct unit q[RING_BANKS - 1U];
	unsigned int q_len;
	uint32_t slice_cost;
	uint64_t fir_cycles, fir_pairs;
	/* The interrupt key of the run, for the windows in irq_window(). */
	unsigned int irq_key;
	struct geom g;
	/* Banks in the rotation; burst_ok: the spare bank is for bursts. */
	unsigned int nbanks;
	bool burst_ok;
	/* Slowest waveform slice seen. */
	uint32_t gen_cost;
} rs ESP_SDR_HIGH_RAM; /* cleared at each run or bench start */

/*
 * The run's position for other CPUs (esp_sdr_ring_now()): pair index @c index
 * sits at ring position @c pos, renewed once per unit (the write index wraps
 * after 16384 pairs). @c seq is odd while it changes, and 0 outside a run
 * and before unit 0 is located.
 */
static struct {
	atomic_t seq;
	uint64_t index;
	uint32_t pos;
} now_ref;

static void now_publish(uint64_t index, uint32_t pos)
{
	atomic_val_t q = atomic_get(&now_ref.seq);

	atomic_set(&now_ref.seq, q + 1);
	now_ref.index = index;
	now_ref.pos = pos;
	atomic_set(&now_ref.seq, q + 2);
}

bool esp_sdr_ring_now(uint64_t *pair)
{
	for (int tries = 0; tries < 8; tries++) {
		atomic_val_t q = atomic_get(&now_ref.seq);
		uint64_t index = now_ref.index;
		uint32_t pos = now_ref.pos;
		uint32_t wi = write_index();

		if (q == 0) {
			return false;
		}
		if ((q & 1) == 0 && atomic_get(&now_ref.seq) == q) {
			*pair = index + ((wi - pos) & RING_MASK);
			return true;
		}
	}
	return false;
}

/*
 * Interrupts pending on this CPU run in a short window between polls. The
 * kernel timer arms the comparator of whichever CPU sets the next timeout:
 * if that is this one, masking it for the run would stall every timeout in
 * the system (the other CPU idles with its comparator a counter wrap away).
 * IPIs get through the same way. From a cooperative thread nothing preempts
 * the run at the end of an interrupt.
 */
static ALWAYS_INLINE bool irq_window(uint32_t *lines)
{
	uint32_t pend, en;

	__asm__ volatile("rsr.interrupt %0" : "=r"(pend));
	__asm__ volatile("rsr.intenable %0" : "=r"(en));
	if ((pend & en) == 0U) {
		return false;
	}
	*lines = pend & en;
	arch_irq_unlock(rs.irq_key);
	(void)arch_irq_lock();
	return true;
}

/*
 * The Wi-Fi MAC interrupt (CPU line 0, reserved for it by the HAL) fires for
 * any packet the receiver decodes, hundreds of times a second on busy air,
 * and its handler is far too long for the ring's windows: units were
 * abandoned until the run failed. The line stays disabled on this CPU for
 * the run and the interrupt is served when it ends, as before the windows
 * existed (the MAC does not need serving while the ring owns the radio).
 */
static bool wmac_was_enabled;

static ALWAYS_INLINE void wmac_irq_hold(void)
{
	uint32_t en;

	__asm__ volatile("rsr.intenable %0" : "=r"(en));
	wmac_was_enabled = (en & BIT(ETS_WMAC_INUM)) != 0U;
	en &= ~BIT(ETS_WMAC_INUM);
	__asm__ volatile("wsr.intenable %0\n\trsync" : : "r"(en) : "memory");
}

static ALWAYS_INLINE void wmac_irq_release(void)
{
	uint32_t en;

	/* Only our bit: a window's handler may have changed others. */
	__asm__ volatile("rsr.intenable %0" : "=r"(en));
	if (wmac_was_enabled) {
		en |= BIT(ETS_WMAC_INUM);
	}
	__asm__ volatile("wsr.intenable %0\n\trsync" : : "r"(en) : "memory");
}

/* Per-stage cycles for esp_sdr_ring_bench(): unpack, stage 1, stage 2, sink. */
static bool prof;
static uint32_t prof_cyc[4];
#define PROF(k, t) do { if (prof) { uint32_t t1_ = ccount(); prof_cyc[k] += t1_ - (t); (t) = t1_; } } while (0)

#if defined(CONFIG_ESP_SDR_RING_BOX4)
/* Boxcar-of-4 state (ESP_SDR_RING_DECIM_BOX4): partial sums and the pair phase. */
static struct {
	int32_t ai, aq;
	uint32_t ph;
} b4;

static inline void b4_reset(void)
{
	b4.ai = 0;
	b4.aq = 0;
	b4.ph = 0;
}

/*
 * ESP_SDR_RING_DECIM_BOX4: the -fs/4 mix as swaps and signs ((-j)^n, n the
 * pair index, so phase 0 sits on the decimation grid), summed over 4 pairs.
 * Summing costs about 14 cycles per pair, over the budget at 16 MS/s once a sink
 * runs too: cfg->subsample takes only the phase-0 pairs for about 1 cycle.
 */
HOT static void box4_feed(const uint32_t *p, uint32_t a, uint32_t m)
{
	const uint32_t *w = p + a;
	uint32_t k2 = 0;

	if (rs.cfg->subsample) {
		/*
		 * Phase-0 pairs only: there the -fs/4 mix is 1. Same full scale as
		 * the sums. Locals throughout: the build does not assume strict
		 * aliasing, so every store would reload the run state.
		 */
		const esp_sdr_ring_sink_t sink = rs.cfg->sink;
		void *const user = rs.cfg->user;
		uint32_t skip = (4U - b4.ph) & 3U, n;
		uint64_t at = rs.out_index;

		if (skip >= m) {
			b4.ph = (b4.ph + m) & 3U;
			return;
		}
		w += skip;
		m -= skip;
		/* Pairs 0, 4, 8, ... of the m left; the next one is m & 3 past the last. */
		n = (m + 3U) / 4U;
		b4.ph = m & 3U;
		while (n != 0U) {
			uint32_t c = MIN(n, OUT_MAX);
			int16_t *oi = out_i, *oq = out_q;

			for (uint32_t k = 0; k < c; k++) {
				uint32_t v = w[4U * k];

				oi[k] = (int16_t)(((int32_t)(v << 12) >> 22) * 16);
				oq[k] = (int16_t)(((int32_t)(v << 22) >> 22) * 16);
			}
			sink(user, out_i, out_q, c, at);
			at += c;
			w += 4U * c;
			n -= c;
		}
		rs.st->samples += at - rs.out_index;
		rs.out_index = at;
		return;
	}

	while (m != 0U) {
		/*
		 * Vector unpack and rotation (the FIR path's kernels, lanes
		 * swapped: lane i carries Q), then sums of 4 lanes. On the
		 * decimation grid only, so lane 0 is pair phase 0.
		 */
		uint32_t g = MIN(m, FIR_CHUNK) / 8U;

		/* The unpack reads 16 bytes ahead: never past the bank's end. */
		if (g != 0U && (uint32_t)(w - p) + 8U * g + 4U > RING_PAIRS) {
			g--;
		}
		if (b4.ph == 0U && g != 0U) {
			uint32_t t = prof ? ccount() : 0U;

			esp_sdr_ring_unpack_iq10(w, fs1_wi, fs1_wq, g);
			if (fir_rot) {
				esp_sdr_ring_rot_fs4(fs1_wi, fs1_wq, g, rot_masks);
			}
			PROF(0, t);
			for (uint32_t k = 0; k < 2U * g; k++) {
				const int16_t *li = &fs1_wi[4U * k], *lq = &fs1_wq[4U * k];
				/* Lanes hold value << 6; the output scale is 4 x the sum. */
				int32_t si = lq[0] + lq[1] + lq[2] + lq[3];
				int32_t sq = li[0] + li[1] + li[2] + li[3];

				out_i[k2] = (int16_t)(si >> 4);
				out_q[k2] = (int16_t)(sq >> 4);
				if (++k2 == OUT_MAX) {
					rs.cfg->sink(rs.cfg->user, out_i, out_q, k2, rs.out_index);
					rs.out_index += k2;
					rs.st->samples += k2;
					k2 = 0;
				}
			}
			PROF(1, t);
			w += 8U * g;
			m -= 8U * g;
			continue;
		}
		if (b4.ph == 0U && m >= 4U) {
			int32_t i0 = esp_sdr_rx_i(w[0]), q0 = esp_sdr_rx_q(w[0]);
			int32_t i1 = esp_sdr_rx_i(w[1]), q1 = esp_sdr_rx_q(w[1]);
			int32_t i2 = esp_sdr_rx_i(w[2]), q2 = esp_sdr_rx_q(w[2]);
			int32_t i3 = esp_sdr_rx_i(w[3]), q3 = esp_sdr_rx_q(w[3]);

			if (fir_rot) {
				/* (-j)^n: 1, -j, -1, j */
				b4.ai = i0 + q1 - i2 - q3;
				b4.aq = q0 - i1 - q2 + i3;
			} else {
				b4.ai = i0 + i1 + i2 + i3;
				b4.aq = q0 + q1 + q2 + q3;
			}
			w += 4;
			m -= 4U;
			b4.ph = 4U;
		} else {
			int32_t i = esp_sdr_rx_i(*w), q = esp_sdr_rx_q(*w);

			if (!fir_rot || b4.ph == 0U) {
				b4.ai += i;
				b4.aq += q;
			} else if (b4.ph == 1U) {
				b4.ai += q;
				b4.aq -= i;
			} else if (b4.ph == 2U) {
				b4.ai -= i;
				b4.aq -= q;
			} else {
				b4.ai -= q;
				b4.aq += i;
			}
			w++;
			m--;
			b4.ph++;
		}
		if (b4.ph == 4U) {
			/* 4 x 10 bit to +-8192 (ESP_SDR_RING_BOX4_FULL_SCALE). */
			out_i[k2] = (int16_t)(b4.ai * 4);
			out_q[k2] = (int16_t)(b4.aq * 4);
			b4_reset();
			if (++k2 == OUT_MAX) {
				rs.cfg->sink(rs.cfg->user, out_i, out_q, k2, rs.out_index);
				rs.out_index += k2;
				rs.st->samples += k2;
				k2 = 0;
			}
		}
	}
	if (k2 != 0U) {
		rs.cfg->sink(rs.cfg->user, out_i, out_q, k2, rs.out_index);
		rs.out_index += k2;
		rs.st->samples += k2;
	}
}
#endif /* CONFIG_ESP_SDR_RING_BOX4 */

/* m contiguous ring pairs from position a (no wrap) through both stages. */
HOT static void fir_feed(const uint32_t *p, uint32_t a, uint32_t m)
{
#if defined(CONFIG_ESP_SDR_RING_BOX4)
	if (rs.log2d == 2U) {
		box4_feed(p, a, m);
		return;
	}
#endif
	while (m != 0U) {
		uint32_t take = MIN(m, FIR_CHUNK), done = 0, g, k2;
		uint32_t t = prof ? ccount() : 0U;

		while ((fs1.n & 7U) != 0U && done < take) {
			fir_scalar(p[a + done++]);
		}
		/* The unpack reads 16 bytes ahead: never past the bank's end. */
		g = (take - done) / 8U;
		if (a + done + 8U * g + 4U > RING_PAIRS) {
			g = g != 0U ? g - 1U : 0U;
		}
		if (g != 0U) {
			esp_sdr_ring_unpack_iq10(p + a + done, fs1.wi + fs1.n, fs1.wq + fs1.n, g);
			if (fir_rot) {
				esp_sdr_ring_rot_fs4(fs1.wi + fs1.n, fs1.wq + fs1.n, g, rot_masks);
			}
			fs1.n += 8U * g;
			done += 8U * g;
		}
		while (done < take) {
			fir_scalar(p[a + done++]);
		}
		PROF(0, t);
		fs2.n += fstage_run(&fs1, fs2.wi + fs2.n, fs2.wq + fs2.n);
		PROF(1, t);
		k2 = fstage_run(&fs2, out_i, out_q);
		PROF(2, t);
		if (k2 != 0U) {
			/* Undo the I/Q swap of the lanes (see above). */
			rs.cfg->sink(rs.cfg->user, out_q, out_i, k2, rs.out_index);
			rs.out_index += k2;
			rs.st->samples += k2;
		}
		PROF(3, t);
		a += take;
		m -= take;
	}
}

static void fir_setup(unsigned int d)
{
	uint32_t d2 = d / 8U, l2 = MIN((18U * d2 + 7U) & ~7U, FS2_LMAX);

#if defined(CONFIG_ESP_SDR_RING_BOX4)
	if (d == ESP_SDR_RING_DECIM_BOX4) {
		b4_reset();
		return;
	}
#endif

	fstage_init(&fs1, 8U, FS1_L, 17U);
	fstage_init(&fs2, d2, l2, 14U);
}

HOT static void unit_abandon(struct unit *u)
{
	rs.st->lost_pairs += u->count - u->pos;
	rs.st->abandoned++;
	rs.next_pair = ~0ULL;
}

HOT static void queue_pop(void)
{
	rs.q_len--;
	for (unsigned int k = 0; k < rs.q_len; k++) {
		rs.q[k] = rs.q[k + 1U];
	}
}

/* Bank b is about to be overwritten: drop what it still holds. */
HOT static void release_bank(unsigned int b)
{
	for (unsigned int k = 0; k < rs.q_len;) {
		if (rs.q[k].bank == b) {
			unit_abandon(&rs.q[k]);
			rs.q_len--;
			for (unsigned int j = k; j < rs.q_len; j++) {
				rs.q[j] = rs.q[j + 1U];
			}
		} else {
			k++;
		}
	}
}

static inline bool bank_idle(unsigned int b)
{
	for (unsigned int k = 0; k < rs.q_len; k++) {
		if (rs.q[k].bank == b) {
			return false;
		}
	}
	return true;
}

/* A unit that does not continue the filtered stream restarts it on the decimation grid. */
HOT static bool unit_begin(struct unit *u)
{
	uint32_t dec = 1U << rs.log2d;

	u->started = true;
	if (u->index == rs.next_pair) {
		return true;
	}
	uint64_t skip = (dec - (u->index & (dec - 1U))) & (dec - 1U);

	if (skip >= u->count) {
		rs.next_pair = ~0ULL;
		return false;
	}
	u->first = (u->first + (uint32_t)skip) & RING_MASK;
	u->count -= (uint32_t)skip;
	u->index += skip;
	rs.out_index = u->index >> rs.log2d;
#if defined(CONFIG_ESP_SDR_RING_BOX4)
	if (rs.log2d == 2U) {
		b4_reset();
		return true;
	}
#endif
	fstage_reset(&fs1, u->index);
	fstage_reset(&fs2, u->index / 8U);
	return true;
}

/* Filter up to n pairs of the oldest unit; returns whether work remains. */
HOT static bool fir_slice(uint32_t n)
{
	struct unit *u;
	const uint32_t *p;
	uint32_t at;

	if (rs.q_len == 0U) {
		return false;
	}
	u = &rs.q[0];
	if (!u->started && !unit_begin(u)) {
		queue_pop();
		return rs.q_len != 0U;
	}
	p = bank_ptr(u->bank);
	n = MIN(n, u->count - u->pos);
	at = (u->first + u->pos) & RING_MASK;
	for (uint32_t k = 0; k < n;) {
		uint32_t a = (at + k) & RING_MASK, m = MIN(n - k, RING_PAIRS - a);

		fir_feed(p, a, m);
		k += m;
	}
	u->pos += n;
	if (u->pos >= u->count) {
		rs.next_pair = u->index + u->count;
		queue_pop();
	}
	return rs.q_len != 0U;
}

static void unit_accept(unsigned int b, uint32_t first, uint32_t count, uint64_t index)
{
	if (rs.q_len == ARRAY_SIZE(rs.q)) {
		unit_abandon(&rs.q[0]);
		queue_pop();
	}
	rs.q[rs.q_len++] = (struct unit){.bank = b, .first = first, .count = count, .index = index};
}

static inline void fail(enum esp_sdr_ring_status code, uint32_t detail)
{
	if (rs.st->status == ESP_SDR_RING_OK) {
		rs.st->status = code;
		rs.st->detail = detail;
	}
}

/* The vector unit is coprocessor 3; Zephyr neither enables nor saves it. */
static inline void pie_enable(void)
{
	uint32_t cp;

	__asm__ volatile("rsr.cpenable %0" : "=r"(cp));
	cp |= BIT(3);
	__asm__ volatile("wsr.cpenable %0\n\trsync" : : "r"(cp));
}

/* Prepare bank nb for the unit after the one now expected at e. */
HOT static void prepare(unsigned int nb, uint32_t e, uint32_t *start_probe, uint32_t *end_probe)
{
	/* Two banks: nb still holds the unit before, released at the switch. */
	if (rs.nbanks == RING_BANKS) {
		release_bank(nb);
	}
	start_probe[nb] = (e + rs.g.thr) & RING_MASK;
	end_probe[nb] = (start_probe[nb] + rs.g.thr) & RING_MASK;
	fill_sentinels(nb, start_probe[nb], rs.g.sg);
	fill_sentinels(nb, end_probe[nb], rs.g.eg);
}

/*
 * ---- bursts (esp_sdr_ring_burst_*()) ----
 *
 * CPU 0 arms, fires or cancels through burst_state; the run (CPU 1) builds
 * the waveform between polls, stops at the first unit boundary at or after
 * the end pair, hands the last unit on, turns the front end to TX, plays
 * the spare bank once at the fire pair, turns back to RX and restarts.
 */
enum { BURST_IDLE, BURST_ARMED, BURST_FIRE, BURST_CANCEL };
#define BURST_BANK 2U
/* Waveform words per slice between polls (about 1700 cycles with slot copies). */
#define GEN_SLICE  400U

static atomic_t burst_state;
/* Above the banks (not zeroed at boot; internal RAM below them is short). */
static struct {
	uint64_t end_pair, limit_pair, at_pair;
	esp_sdr_ring_gen_t gen;
	void *user;
	uint32_t from, words;
} burst ESP_SDR_HIGH_RAM;
static struct esp_sdr_ring_burst_stats bst ESP_SDR_HIGH_RAM;

uint32_t *esp_sdr_ring_burst_buf(size_t *words)
{
	*words = ESP_SDR_SAMPLES_MAX;
	return bank_ptr(BURST_BANK);
}

int esp_sdr_ring_burst_arm(uint64_t end_pair, uint64_t limit_pair, esp_sdr_ring_gen_t gen,
			   void *user, uint32_t from, uint32_t words)
{
	if (words > ESP_SDR_SAMPLES_MAX || words < ESP_SDR_SAMPLES_MIN || from > words ||
	    gen == NULL) {
		return -EINVAL;
	}
	if (!rs.burst_ok || !esp_sdr_ring_active() || atomic_get(&burst_state) != BURST_IDLE) {
		return -EBUSY;
	}
	burst.end_pair = end_pair;
	burst.limit_pair = limit_pair;
	burst.gen = gen;
	burst.user = user;
	burst.from = from;
	burst.words = words;
	/* Atomic after the fields: the run reads them once it sees the state. */
	return atomic_cas(&burst_state, BURST_IDLE, BURST_ARMED) ? 0 : -EBUSY;
}

bool esp_sdr_ring_burst_fire(uint64_t at_pair)
{
	burst.at_pair = at_pair;
	return atomic_cas(&burst_state, BURST_ARMED, BURST_FIRE);
}

void esp_sdr_ring_burst_cancel(void)
{
	(void)atomic_cas(&burst_state, BURST_ARMED, BURST_CANCEL);
}

bool esp_sdr_ring_burst_busy(void)
{
	return atomic_get(&burst_state) != BURST_IDLE;
}

void esp_sdr_ring_burst_get_stats(struct esp_sdr_ring_burst_stats *stats)
{
	*stats = bst;
}

/* One waveform slice; returns whether more is left. */
static __attribute__((noinline)) bool gen_slice(uint32_t *pos, uint32_t n)
{
	n = MIN(n, burst.words - *pos);
	if (n != 0U) {
		burst.gen(burst.user, bank_ptr(BURST_BANK) + *pos, *pos, n);
		*pos += n;
	}
	return *pos < burst.words;
}

/*
 * The capture has stopped at pair p_stop (cycle t_stop): play the burst if
 * it is fired in time, and leave the front end in RX.
 */
/* Not in IRAM (internal RAM is short): only its spin to the fire pair is timed, from the cache. */
static __attribute__((noinline, optimize("O3"))) void burst_run(uint32_t cpp, uint32_t saved, uint64_t p_stop, uint32_t t_stop,
			  uint32_t *gen_pos)
{
	const uint32_t dac_ctrl = DUMP_CTRL_RUN | FIELD_PREP(DUMP_CTRL_COUNT, burst.words);
	atomic_val_t s;
	uint32_t t, lines;

#define PAIR_NOW() (p_stop + (ccount() - t_stop) / cpp)
	/* The last unit to the PHY first: its verdict runs meanwhile on the other CPU. */
	while (fir_slice(rs.g.slice)) {
	}
	bst.gen_left_max = MAX(bst.gen_left_max, burst.words - *gen_pos);
	while (gen_slice(gen_pos, ESP_SDR_SAMPLES_MAX)) {
	}
	/* The capture side's control register clear, as for any DAC session. */
	REG_WRITE(DUMP_CTRL_REG, 0);
	/*
	 * No settle wait: a fire due earlier starts the preamble (eight symbols,
	 * 128 us; a receiver needs about half) while the front end still settles.
	 */
	esp_sdr_tx_front_end();
	bst.ready_last_pairs = (int32_t)(PAIR_NOW() - burst.end_pair);
	bst.ready_max_pairs = MAX(bst.ready_max_pairs, bst.ready_last_pairs);
	for (;;) {
		uint32_t tw = ccount();

		(void)stall_poll();
		(void)irq_window(&lines);
		bst.window_max_cyc = MAX(bst.window_max_cyc, ccount() - tw);
		s = atomic_get(&burst_state);
		if (s == BURST_FIRE || s == BURST_CANCEL) {
			break;
		}
		if (PAIR_NOW() > burst.limit_pair &&
		    atomic_cas(&burst_state, BURST_ARMED, BURST_CANCEL)) {
			bst.timeouts++;
			s = BURST_CANCEL;
			break;
		}
	}
	if (s == BURST_FIRE) {
		uint64_t at = burst.at_pair;

		bst.seen_last_pairs = (int32_t)(PAIR_NOW() - burst.end_pair);
		bst.seen_max_pairs = MAX(bst.seen_max_pairs, bst.seen_last_pairs);
		if (PAIR_NOW() > burst.limit_pair) {
			bst.skipped++;
		} else {
			int32_t f;

			/* Spin to the fire pair, unless it has passed. */
			if (at > p_stop) {
				t = t_stop + (uint32_t)(at - p_stop) * cpp;
				while ((int32_t)(t - ccount()) > 0) {
				}
			}
			select_bank(saved, BIT(BURST_BANK));
			REG_WRITE(DAC_TRIG_REG, 0);
			REG_WRITE(DAC_TRIG_REG, dac_ctrl);
			REG_WRITE(DAC_TRIG_REG, dac_ctrl | DUMP_CTRL_TRIGGER);
			REG_WRITE(DAC_TRIG_REG, dac_ctrl);
			t = ccount();
			f = (int32_t)(p_stop + (t - t_stop) / cpp - burst.end_pair);
			bst.fired++;
			/* More than 2 us after the fire pair: the front end or the verdict was late. */
			if ((int64_t)(p_stop + (t - t_stop) / cpp) - (int64_t)at > 32) {
				bst.late++;
				/* The fire was waiting when the front end got ready. */
				if (bst.seen_last_pairs - bst.ready_last_pairs < 16) {
					bst.late_tx++;
				}
			}
			bst.fire_last_pairs = f;
			bst.fire_min_pairs = bst.fired == 1U ? f : MIN(bst.fire_min_pairs, f);
			bst.fire_max_pairs = bst.fired == 1U ? f : MAX(bst.fire_max_pairs, f);
			/* One pass at 40 MS/s: 6 cycles per word at 240 MHz, plus a margin. */
			while ((REG_READ(DAC_TRIG_REG) & DUMP_CTRL_DONE) == 0U &&
			       ccount() - t < 7U * burst.words + 24000U) {
			}
			bst.dac_last_cyc = (REG_READ(DAC_TRIG_REG) & DUMP_CTRL_DONE) != 0U ? ccount() - t
											   : 0U;
			REG_WRITE(DAC_TRIG_REG, 0);
		}
	} else {
		bst.cancelled++;
	}
	REG_WRITE(DUMP_CTRL_REG, 0);
	esp_sdr_rx_resume();
#undef PAIR_NOW
}

/*
 * A burst the run has not taken yet: armed, or already fired or cancelled
 * before the run saw it armed. Returns whether it is now the run's.
 */
static __attribute__((noinline)) bool burst_pickup(uint64_t index, uint32_t *gen_pos)
{
	if (atomic_get(&burst_state) == BURST_CANCEL) {
		bst.cancelled++;
		atomic_set(&burst_state, BURST_IDLE);
		return false;
	}
	*gen_pos = burst.from;
	if (index > burst.end_pair) {
		bst.arm_late++;
	}
	return true;
}

/* Fresh sentinels in every bank of the rotation and the engine running into bank 0. */
static __attribute__((noinline)) void engine_start(uint32_t ctrl, uint32_t saved, uint32_t *epoch, uint32_t *w0)
{
	for (unsigned int k = 0; k < rs.nbanks; k++) {
		fill_sentinels(k, 0, RING_PAIRS);
	}
	REG_WRITE(DUMP_CTRL_REG, 0);
	REG_WRITE(DUMP_CONFIG_REG, DUMP_CONFIG_IQ);
	REG_WRITE(DUMP_CTRL_REG, ctrl);
	select_bank(saved, BIT(0));
	REG_WRITE(DUMP_CTRL_REG, ctrl | DUMP_CTRL_RUN);
	*epoch = ccount();
	*w0 = write_index();
}

HOT static void ring_loop(void)
{
	const uint32_t cpp = sys_clock_hw_cycles_per_sec() / ESP_SDR_RING_RATE_HZ;
	const uint32_t max_age = (RING_PAIRS - 128U) * cpp;
	const uint32_t settle = 16U * cpp;
	const uint32_t prep_pairs = 20000U / cpp + 1024U;
	const struct geom g = rs.g;
	const uint32_t ctrl = DUMP_CTRL_CIRCULAR | DUMP_CTRL_16MSPS;
	const uint32_t saved = REG_READ(SENSITIVE_INTERNAL_SRAM_USAGE_3_REG);
	const unsigned int nbanks = rs.nbanks;
	uint32_t start_probe[RING_BANKS] = {0}, end_probe[RING_BANKS] = {0};
	struct esp_sdr_ring_stats *st = rs.st;
	uint32_t epoch, w0, expected, gen_pos = 0;
	uint64_t index = 0, t_start;
	unsigned int b = 0;
	bool prepared = false, fresh = true, armed = false;

	t_start = k_cycle_get_64();
	engine_start(ctrl, saved, &epoch, &w0);
	/* Unit 0 starts near w0; its exact start is found after the switch. */
	expected = (w0 - 256U) & RING_MASK;

	for (;;) {
		const unsigned int next = (b + 1U) % nbanks;
		uint32_t wi, written, late, first, end, count, t;
		bool stop = false, stop_checked = false, burst_stop = false;

		/* 1. Poll, with filter and waveform slices between polls. */
		for (;;) {
			if (stall_poll()) {
				st->stalls++;
			}
			uint32_t lines;

			if (irq_window(&lines)) {
				st->irq_windows++;
				st->irq_lines |= lines;
			}
			wi = write_index();
			written = (wi - expected) & RING_MASK;
			if (ccount() - epoch > max_age) {
				fail(ESP_SDR_RING_AGE, ccount() - epoch);
				break;
			}
			if (written >= g.thr) {
				if (!prepared) {
					/* A slice ran past the preparation deadline. */
					prepare(next, expected, start_probe, end_probe);
					prepared = true;
					wi = write_index();
					written = (wi - expected) & RING_MASK;
				}
				break;
			}
			if (!stop_checked && written >= g.thr / 2U) {
				/* Decided mid-unit: only the index read sits before the switch. */
				stop_checked = true;
				stop = atomic_get(&stop_req) != 0;
				continue;
			}
			if (!prepared &&
			    (bank_idle(next) || written + prep_pairs + g.late >= g.thr)) {
				prepare(next, expected, start_probe, end_probe);
				prepared = true;
				continue;
			}
			/* A slice only if the slowest one seen still fits. */
			int32_t limit = prepared ? (int32_t)(g.thr + g.late / 2U)
						 : (int32_t)g.thr - (int32_t)(prep_pairs + g.late);

			if (rs.q_len != 0U &&
			    (int32_t)(written + rs.slice_cost / cpp + 256U) < limit) {
				uint32_t t0 = ccount(), dt;
				uint32_t left = rs.q[0].count - rs.q[0].pos;

				fir_slice(g.slice);
				dt = ccount() - t0;
				rs.fir_cycles += dt;
				rs.fir_pairs += MIN(left, g.slice);
				rs.slice_cost = MAX(rs.slice_cost, dt);
				st->slice_max = MAX(st->slice_max, dt);
				continue;
			}
			if (rs.burst_ok && !armed && atomic_get(&burst_state) != BURST_IDLE) {
				armed = burst_pickup(index, &gen_pos);
			}
			if (armed && gen_pos < burst.words &&
			    (int32_t)(written + rs.gen_cost / cpp + 64U) < limit) {
				uint32_t t0 = ccount();

				(void)gen_slice(&gen_pos, GEN_SLICE);
				rs.gen_cost = MAX(rs.gen_cost, ccount() - t0);
			}
		}
		if (st->status != ESP_SDR_RING_OK) {
			break;
		}
		late = written - g.thr;
		st->late_max = MAX(st->late_max, late);
		if (late > g.late) {
			fail(ESP_SDR_RING_LATE, written);
			break;
		}
		/* An armed burst whose frame ends in this unit stops it; a cancelled one is dropped. */
		if (armed) {
			if (atomic_get(&burst_state) == BURST_CANCEL) {
				armed = false;
				bst.cancelled++;
				atomic_set(&burst_state, BURST_IDLE);
			} else {
				burst_stop = index + written >= burst.end_pair;
			}
		}

		/* 2. Stop or switch. */
		if (stop || burst_stop) {
			REG_WRITE(DUMP_CTRL_REG, ctrl);
			select_bank(saved, 0);
		} else {
			if (nbanks < RING_BANKS) {
				/* Two banks: the unit before must be done before the engine takes its bank. */
				release_bank(next);
			}
			select_bank(saved, BIT(next));
			epoch = ccount();
		}
		t = ccount();
		while (ccount() - t < settle) {
		}

		/* 3. Locate the finished unit exactly. */
		if (fresh) {
			first = find_first(b, (w0 - 1024U) & RING_MASK, g.sg0);
			end = find_end(b, wi, wi, UNIT0_END_WIN);
		} else {
			first = find_first(b, start_probe[b], g.sg);
			end = find_end(b, end_probe[b], wi, g.eg);
		}
		if (first == NOT_FOUND || (!fresh && first != expected)) {
			fail(ESP_SDR_RING_START, (first & 0xffffU) | (expected << 16));
			break;
		}
		if (end == NOT_FOUND) {
			fail(ESP_SDR_RING_END, wi);
			break;
		}
		count = (end - first) & RING_MASK;
		if (count < (!fresh ? g.thr : g.min0) || count > g.thr + g.late + 64U) {
			fail(ESP_SDR_RING_LENGTH, count);
			break;
		}
		unit_accept(b, first, count, index);
		st->units++;
		index += count;
		expected = end;
		b = next;
		prepared = false;
		fresh = false;
		if (burst_stop) {
			/* The engine stopped at pair index, at cycle t. */
			uint32_t t_go;

			bst.stop_last_pairs = (int32_t)(index - burst.end_pair);
			atomic_set(&now_ref.seq, 0);
			burst_run(cpp, saved, index, t, &gen_pos);
			armed = false;
			atomic_set(&burst_state, BURST_IDLE);
			if (stop) {
				break;
			}
			engine_start(ctrl, saved, &epoch, &w0);
			t_go = ccount();
			/* The pair index keeps counting real time across the gap. */
			index += (t_go - t) / cpp;
			bst.rx_back_last_pairs = (int32_t)(index - burst.end_pair);
			expected = (w0 - 256U) & RING_MASK;
			b = 0;
			fresh = true;
			continue;
		}
		now_publish(index, end);
		if (stop) {
			break;
		}
		/* The forced gain gets lost now and then (see esp_sdr_rx_gain_refresh()). */
		if (esp_sdr_rx_gain_refresh()) {
			st->gain_refreshed++;
		}
	}

	/* Engine stopped (normally or by a failure): all banks are the CPU's. */
	atomic_set(&now_ref.seq, 0);
	REG_WRITE(DUMP_CTRL_REG, 0);
	REG_WRITE(SENSITIVE_INTERNAL_SRAM_USAGE_3_REG, saved);
	if (atomic_get(&burst_state) != BURST_IDLE) {
		bst.cancelled++;
		atomic_set(&burst_state, BURST_IDLE);
	}
	st->elapsed_us = k_cyc_to_us_floor64(k_cycle_get_64() - t_start);
	st->pairs = index;
	while (fir_slice(4096U)) {
	}
}

int esp_sdr_ring_run(const struct esp_sdr_ring_cfg *cfg, struct esp_sdr_ring_stats *stats)
{
	struct geom g_check;
	unsigned int d = cfg->decim, l = 0, key;
	int ret = 0;

	while ((1U << l) < d) {
		l++;
	}
	if ((d < ESP_SDR_RING_DECIM_MIN &&
	     !(IS_ENABLED(CONFIG_ESP_SDR_RING_BOX4) && d == ESP_SDR_RING_DECIM_BOX4)) ||
	    d > ESP_SDR_RING_DECIM_MAX || (1U << l) != d ||
	    cfg->sink == NULL || geom_init(&g_check, cfg->unit_pairs) != 0 ||
	    (cfg->spare_bank && !geom_two_banks(&g_check))) {
		return -EINVAL;
	}
	if ((uintptr_t)__esp_sdr_bank0_start != DUMP_BANK0_ADDR) {
		return -ENOMEM;
	}

	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	if (!esp_sdr_ready) {
		k_mutex_unlock(&esp_sdr_lock);
		return -EAGAIN;
	}
	memset(stats, 0, sizeof(*stats));
	memset(&rs, 0, sizeof(rs));
	(void)geom_init(&rs.g, cfg->unit_pairs);
	rs.nbanks = cfg->spare_bank ? RING_BANKS - 1U : RING_BANKS;
	rs.burst_ok = cfg->spare_bank;
	memset(&bst, 0, sizeof(bst));
	rs.gen_cost = 2000U;
	rs.cfg = cfg;
	rs.st = stats;
	rs.log2d = l;
	rs.next_pair = ~0ULL;
	/* About 8 cycles per pair until the slices have been timed. */
	rs.slice_cost = 8U * rs.g.slice;
	fir_rot = cfg->shift_fs4;
	fir_setup(d);
	atomic_set(&stop_req, 0);
	atomic_set(&active, 1);

	pie_enable();
	/* Local mask only: irq_lock() would take the global SMP lock for the run. */
	key = arch_irq_lock();
	rs.irq_key = key;
	wmac_irq_hold();
	ring_loop();
	wmac_irq_release();
	arch_irq_unlock(key);

	stats->cycles_x100 =
		rs.fir_pairs != 0U ? (uint32_t)(rs.fir_cycles * 100U / rs.fir_pairs) : 0U;
	atomic_set(&active, 0);
	k_mutex_unlock(&esp_sdr_lock);
	if (stats->status != ESP_SDR_RING_OK) {
		ret = -EIO;
	}
	return ret;
}

int esp_sdr_ring_bench(const struct esp_sdr_ring_cfg *cfg, unsigned int units,
		       struct esp_sdr_ring_bench_result *res)
{
	struct esp_sdr_ring_stats st = {0};
	unsigned int d = cfg->decim, l = 0, key;
	uint32_t *p = bank_ptr(0), t0, total;

	while ((1U << l) < d) {
		l++;
	}
	if ((d < ESP_SDR_RING_DECIM_MIN &&
	     !(IS_ENABLED(CONFIG_ESP_SDR_RING_BOX4) && d == ESP_SDR_RING_DECIM_BOX4)) ||
	    d > ESP_SDR_RING_DECIM_MAX || (1U << l) != d ||
	    cfg->sink == NULL || units == 0U) {
		return -EINVAL;
	}
	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	/* Noise-like words in bank 0, which the CPUs own outside a run. */
	for (uint32_t k = 0, x = 1; k < RING_PAIRS; k++) {
		x = x * 1664525U + 1013904223U;
		p[k] = x & 0xfffffU;
	}
	memset(&rs, 0, sizeof(rs));
	(void)geom_init(&rs.g, 0);
	rs.nbanks = RING_BANKS;
	rs.cfg = cfg;
	rs.st = &st;
	rs.log2d = l;
	rs.next_pair = ~0ULL;
	fir_rot = cfg->shift_fs4;
	fir_setup(d);
	memset(prof_cyc, 0, sizeof(prof_cyc));
	pie_enable();
	key = arch_irq_lock();
	prof = true;
	t0 = ccount();
	for (unsigned int u = 0; u < units; u++) {
		unit_accept(0, (u * rs.g.thr) & RING_MASK, rs.g.thr, (uint64_t)u * rs.g.thr);
		while (fir_slice(SLICE_PAIRS)) {
		}
	}
	total = ccount() - t0;
	prof = false;
	arch_irq_unlock(key);
	k_mutex_unlock(&esp_sdr_lock);

	res->pairs = units * rs.g.thr;
	res->cycles_x100 = (uint32_t)((uint64_t)total * 100U / res->pairs);
	for (int k = 0; k < 4; k++) {
		res->stage_x100[k] = (uint32_t)((uint64_t)prof_cyc[k] * 100U / res->pairs);
	}
	res->samples = (uint32_t)st.samples;
	return 0;
}

void esp_sdr_ring_stop(void)
{
	atomic_set(&stop_req, 1);
}

bool esp_sdr_ring_active(void)
{
	return atomic_get(&active) != 0;
}
