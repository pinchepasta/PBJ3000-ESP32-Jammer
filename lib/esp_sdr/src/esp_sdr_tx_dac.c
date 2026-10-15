/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Transmit backend: samples arrive at a low rate (fit for USB) and are
 * interpolated linearly to the 40 MS/s DAC rate. The DAC runs in loop mode
 * (gapless, at exactly the DAC rate) over one of two 64 KiB dump banks; while
 * it plays one, the next block is interpolated straight into the other, with
 * the S3 vector unit on one core (or, as the C fallback, split across both
 * cores), and the bank select flips at the block's sample time. Output stays
 * locked to real time: input not there in time plays as zeros.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <esp_attr.h>

#include <esp_sdr/esp_sdr.h>
#include <esp_sdr/esp_sdr_tx.h>

#include "esp_sdr_priv.h"

LOG_MODULE_DECLARE(esp_sdr, CONFIG_ESP_SDR_LOG_LEVEL);

#define DAC_RATE  ESP_SDR_RATE_40MSPS
#define RING_SIZE CONFIG_ESP_SDR_TX_DAC_RING_SAMPLES

BUILD_ASSERT(IS_POWER_OF_TWO(RING_SIZE), "ring size must be a power of two");

/* Packed input samples, I in the upper half word. PSRAM: written by the network thread. */
static EXT_RAM_BSS_ATTR uint32_t ring[RING_SIZE];

#define NFILL       2
/* One full bank per block. */
#define BLOCK_WORDS ESP_SDR_SAMPLES_MAX

static K_SEM_DEFINE(start_sem, 0, 1);
static K_SEM_DEFINE(done_sem, 0, 1);
/* Per filler: session start / half job, and session done / half done. */
static struct k_sem job_sem[NFILL], ready_sem[NFILL], half_sem, half_done_sem;
static atomic_t running, stop_req;
/*
 * Free-running sample indices (wrap-safe differences, sessions < 2^31
 * samples): ring_w is written by write() on the network thread, ring_r is the
 * player's.
 */
static atomic_t ring_w;
/*
 * Input index the filled blocks reach (zeros included): input written below
 * it came too late to play. Kept by the leader, read by dac_write().
 */
static atomic_t filled_in, late_in;
static uint32_t ring_r, in_base;
static uint32_t in_rate, interp;
static struct esp_sdr_tx_dac_stats stats;

/* Generator session (esp_sdr_tx_dac_play_gen()): fill() calls it instead of interpolating. */
static esp_sdr_tx_dac_gen_t gen_fn;
static void *gen_user;
static uint64_t gen_total;
/* Set by the player when a session could not start. */
static int sess_err;

static int dac_start(uint64_t freq_hz, uint32_t rate_hz)
{
	uint32_t fs = esp_sdr_tx_rate_hz(DAC_RATE);
	int ret;

	/* Integer interpolation factor of at least 2. */
	if (rate_hz == 0U || fs % rate_hz != 0U || fs / rate_hz < 2U) {
		return -EINVAL;
	}
	/* One LO for both directions: this also retunes the receiver. */
	if (freq_hz % 1000000U != 0U) {
		return -EINVAL;
	}
	if (!atomic_cas(&running, 0, 1)) {
		return -EBUSY;
	}
	ret = esp_sdr_set_freq((uint32_t)(freq_hz / 1000000U));
	if (ret != 0) {
		atomic_set(&running, 0);
		return ret;
	}
	in_rate = rate_hz;
	interp = fs / rate_hz;
	atomic_set(&ring_w, 0);
	atomic_set(&filled_in, 0);
	atomic_set(&late_in, 0);
	ring_r = 0;
	stats = (struct esp_sdr_tx_dac_stats){.interp = interp};
	atomic_set(&stop_req, 0);
	k_sem_give(&start_sem);
	LOG_INF("tx dac: %llu Hz, %u S/s in, x%u to %u S/s", (unsigned long long)freq_hz, rate_hz,
		interp, fs);
	return 0;
}

static int dac_write(const struct esp_sdr_iq16 *samples, size_t count)
{
	uint32_t w = (uint32_t)atomic_get(&ring_w);
	int32_t late = (int32_t)((uint32_t)atomic_get(&filled_in) - w);

	/* Slots whose play time passed already: only counted, written anyway. */
	if (late > 0 && atomic_get(&running)) {
		atomic_add(&late_in, (atomic_val_t)MIN((size_t)late, count));
	}

	for (size_t j = 0; j < count; j++) {
		ring[(w + j) & (RING_SIZE - 1U)] =
			((uint32_t)(uint16_t)samples[j].i << 16) | (uint16_t)samples[j].q;
	}
	/* Atomic store after the data: the player never reads unwritten slots. */
	atomic_set(&ring_w, (atomic_val_t)(w + count));
	return (int)count;
}

static int dac_stop(void)
{
	/* A generator session ends by itself; its caller waits for it. */
	if (!atomic_get(&running) || gen_fn != NULL) {
		return 0;
	}
	atomic_set(&stop_req, 1);
	k_sem_take(&done_sem, K_FOREVER);
	atomic_set(&running, 0);
	LOG_INF("tx dac: %llu played, %llu dropped, %u underruns, %u overruns",
		(unsigned long long)stats.played, (unsigned long long)stats.dropped,
		stats.underruns, stats.overruns);
	return 0;
}

/*
 * Interpolator fixed point: I in Q10, Q in Q20, so (a >> 10) puts I's integer
 * part in bits 9:0 and Q's in bits 19:10 of the DAC word; the vector routine
 * relies on this. Ten fraction bits keep the I ramp within half an LSB.
 */
#define I_FRAC 10
#define Q_FRAC 20

static inline int32_t to_fx(uint32_t half, int frac)
{
	/* int16 full scale to the DAC's 10 bits, plus half an LSB for rounding. */
	return (((int32_t)(int16_t)half) >> 6) * (1 << frac) + (1 << (frac - 1));
}

static inline uint32_t dac_word(int32_t ai, int32_t aq)
{
	return ((uint32_t)(ai >> I_FRAC) & 0x3ffU) | ((uint32_t)(aq >> (Q_FRAC - 10)) & 0xffc00U);
}

#if defined(CONFIG_ESP_SDR_TX_DAC_SIMD)
void esp_sdr_pie_ramp(uint32_t *dst, uint32_t groups, const int32_t *vec);

/* The vector unit is coprocessor 3; Zephyr neither enables nor saves it. */
static inline void pie_enable(void)
{
	uint32_t cp;

	__asm__ volatile("rsr.cpenable %0" : "=r"(cp));
	cp |= BIT(3);
	__asm__ volatile("wsr.cpenable %0\n\trsync" : : "r"(cp));
}
#endif

/* Set by the boot self-test: the vector ramp matched the scalar one bit for bit. */
static bool pie_ok;


#if defined(CONFIG_ESP_SDR_TX_DAC_SIMD)
/* Lane starts and steps for the vector ramp, then its two constant masks. */
static int32_t pie_vec[24] __aligned(16) = {
	[16] = 0x3ff, 0x3ff, 0x3ff, 0x3ff, 0xffc00, 0xffc00, 0xffc00, 0xffc00,
};
#endif

/*
 * The fill and the bank switch run from IRAM on input staged in internal RAM:
 * while the other core runs a flash operation the shared cache is suspended,
 * and code or data behind it (flash, PSRAM) stalls for milliseconds.
 */
#define HOT IRAM_ATTR __attribute__((optimize("O3")))

/* Buffers and stacks in the SRAM above the capture bank, or in the DRAM image. */
#if defined(CONFIG_ESP_SDR_TX_DAC_HIGH_RAM)
#define DAC_RAM   ESP_SDR_HIGH_RAM
#define DAC_STACK ESP_SDR_HIGH_RAM
#else
#define DAC_RAM
#define DAC_STACK __kstackmem
#endif

/* Input for the block being filled, copied from the PSRAM ring. */
#define STAGE_WORDS 512U
static uint32_t stage[STAGE_WORDS] DAC_RAM;
/* Where fill() reads input index i: src[(i - src_base) & src_mask]. */
static const uint32_t *src;
static uint32_t src_base, src_mask;

/* cnt DAC words of one linear segment: start (ai, aq), step (di, dq) per sample. */
HOT static void ramp(uint32_t *dst, uint32_t cnt, int32_t ai,
						 int32_t aq, int32_t di, int32_t dq)
{
#if defined(CONFIG_ESP_SDR_TX_DAC_SIMD)
	uint32_t groups;

	if (!pie_ok) {
		goto scalar;
	}
	/* 128-bit stores need 16-byte alignment (always given for factors of 4). */
	while (cnt > 0U && ((uintptr_t)dst & 15U) != 0U) {
		*dst++ = dac_word(ai, aq);
		ai += di;
		aq += dq;
		cnt--;
	}
	groups = cnt / 4U;
	if (groups > 0U) {
		int32_t *v = pie_vec;

		v[0] = ai;
		v[1] = ai + di;
		v[2] = ai + 2 * di;
		v[3] = ai + 3 * di;
		v[4] = aq;
		v[5] = aq + dq;
		v[6] = aq + 2 * dq;
		v[7] = aq + 3 * dq;
		v[8] = v[9] = v[10] = v[11] = 4 * di;
		v[12] = v[13] = v[14] = v[15] = 4 * dq;
		esp_sdr_pie_ramp(dst, groups, v);
		dst += groups * 4U;
		ai += (int32_t)(groups * 4U) * di;
		aq += (int32_t)(groups * 4U) * dq;
		cnt -= groups * 4U;
	}
scalar:
#endif
	while (cnt-- > 0U) {
		*dst++ = dac_word(ai, aq);
		ai += di;
		aq += dq;
	}
}

/*
 * Linear interpolation of output samples [out, out + n) into dst; output
 * sample o lies between input samples o / interp and o / interp + 1. One
 * division for the start, then segment by segment.
 */
HOT static void fill(uint32_t *dst, uint64_t out, uint32_t n)
{
	if (gen_fn != NULL) {
		gen_fn(gen_user, dst, out, n);
		return;
	}

	const uint32_t L = interp;
	const uint32_t *in = src;
	const uint32_t mask = src_mask;
	uint32_t idx = in_base + (uint32_t)(out / L) - src_base;
	uint32_t j = (uint32_t)(out % L);
	uint32_t x1, x0 = in[idx & mask];

	while (n > 0U) {
		uint32_t cnt = MIN(L - j, n);
		int32_t ai = to_fx(x0 >> 16, I_FRAC), aq = to_fx(x0 & 0xffffU, Q_FRAC);
		int32_t di, dq;

		x1 = in[(idx + 1U) & mask];
		di = (to_fx(x1 >> 16, I_FRAC) - ai) / (int32_t)L;
		dq = (to_fx(x1 & 0xffffU, Q_FRAC) - aq) / (int32_t)L;
		ramp(dst, cnt, ai + di * (int32_t)j, aq + dq * (int32_t)j, di, dq);
		dst += cnt;
		n -= cnt;
		j = 0;
		idx++;
		x0 = x1;
	}
}

/*
 * Loop mode timeline. The engine replays BLOCK_WORDS of the selected bank;
 * loop sample s (since the trigger) sits at bank offset s % BLOCK_WORDS and
 * carries output sample out_base + s. Block k (k >= 1) owns loop samples
 * [k * LOOP_STEP, (k + 1) * LOOP_STEP): its bank is selected at the start and
 * holds the block plus LOOP_LEAD samples before it (a switch a little early
 * is harmless) and LOOP_TAIL after it (a switch up to LOOP_TAIL late is too).
 * Lead, block and tail make up exactly one bank pass.
 */
#define LOOP_LEAD 64U
#define LOOP_TAIL 2048U
#define LOOP_STEP (BLOCK_WORDS - LOOP_LEAD - LOOP_TAIL)
BUILD_ASSERT(LOOP_STEP % 4U == 0U && LOOP_LEAD % 4U == 0U && BLOCK_WORDS % 4U == 0U,
	     "loop geometry must keep 16-byte alignment");

/* Session state: real time origin, output origin of the running loop. */
static uint64_t sess_t0, out_base;
/* CCOUNT of the leader's CPU when the running loop started. */
static uint32_t loop_t0;
static uint32_t cyc_per_sample;
static uint64_t played_out, dropped_out;

static inline uint64_t out_now(void)
{
	return (k_cycle_get_64() - sess_t0) / cyc_per_sample;
}

/* Output samples [out, out + n) into bank at offset s % BLOCK_WORDS, wrapping. */
HOT static void fill_wrapped(uint32_t *bank, uint64_t s, uint32_t n, bool zero)
{
	uint32_t off = (uint32_t)(s % BLOCK_WORDS);

	while (n > 0U) {
		uint32_t cnt = MIN(n, BLOCK_WORDS - off);

		if (zero) {
			ramp(bank + off, cnt, to_fx(0, I_FRAC), to_fx(0, Q_FRAC), 0, 0);
		} else {
			fill(bank + off, out_base + s, cnt);
		}
		s += cnt;
		n -= cnt;
		off = 0;
	}
}

/* Second half of a block for filler 1 (C fallback only). */
static struct {
	uint32_t *bank;
	uint64_t s;
	uint32_t n, data; /* n 0: stopping; data: leading samples with input */
} half_job;

/* Loop samples [s, s + n) with input for the first `data` of them, zeros after. */
HOT static void fill_part(uint32_t *bank, uint64_t s, uint32_t n, uint32_t data)
{
	data = MIN(data, n);
	fill_wrapped(bank, s, data, false);
	fill_wrapped(bank, s + data, n - data, true);
}

static void timed_fill(int f, uint32_t *bank, uint64_t s, uint32_t n, uint32_t data)
{
	uint32_t c = k_cycle_get_32();

#if defined(CONFIG_ESP_SDR_TX_DAC_SIMD)
	/*
	 * The vector registers are not part of the thread context: keep other
	 * threads off this core while they are live. Interrupts do not use them.
	 */
	k_sched_lock();
	pie_enable();
	{
		/*
		 * Interrupts too: the USB controller and scheduler IPIs land on
		 * this CPU and stretched a 280 us fill past the block tail
		 * (measured: 375..420 us). They wait at most one fill instead.
		 */
		unsigned int key = irq_lock();

		fill_part(bank, s, n, data);
		irq_unlock(key);
	}
	k_sched_unlock();
#else
	fill_part(bank, s, n, data);
#endif
	c = k_cyc_to_us_floor32(k_cycle_get_32() - c);
	stats.fill_us[f] += c;
	stats.fill_us_max = MAX(stats.fill_us_max, c);
	stats.fill_blocks[f]++;
	stats.fill_cpu[f] = arch_curr_cpu()->id;
}

/*
 * Fill loop samples [s, s + n) of bank: as much as the input has (the last
 * output sample needs input o / interp + 1), zeros for the rest. Returns the
 * zero filled count.
 */
static uint32_t fill_block(uint32_t *bank, uint64_t s, uint32_t n)
{
	if (gen_fn != NULL) {
		uint64_t first = out_base + s;
		uint32_t data = first < gen_total ? (uint32_t)MIN(gen_total - first, (uint64_t)n)
						  : 0U;

		timed_fill(0, bank, s, n, data);
		barrier_dmem_fence_full();
		return n - data;
	}

	uint32_t w = (uint32_t)atomic_get(&ring_w);
	uint64_t have = w - in_base >= 2U ? (uint64_t)(w - in_base - 1U) * interp : 0U;
	uint64_t first = out_base + s;
	uint32_t data = have > first ? (uint32_t)MIN(have - first, (uint64_t)n) : 0U;

	atomic_set(&filled_in, (atomic_val_t)(in_base + (uint32_t)((first + n) / interp)));

	/* Stage the block's input (plus the next sample) out of PSRAM when it fits. */
	{
		uint32_t i0 = in_base + (uint32_t)(first / interp);
		uint32_t cnt = (uint32_t)((first + n) / interp) - (i0 - in_base) + 2U;

		if (cnt <= STAGE_WORDS) {
			for (uint32_t k = 0; k < cnt; k++) {
				stage[k] = ring[(i0 + k) & (RING_SIZE - 1U)];
			}
			src = stage;
			src_base = i0;
			src_mask = UINT32_MAX;
		} else {
			src = ring;
			src_base = 0;
			src_mask = RING_SIZE - 1U;
		}
	}

	/* Input more than a ring ahead overwrote what this block reads. */
	if ((int32_t)(w - (in_base + (uint32_t)(first / interp))) > (int32_t)(RING_SIZE - 2U)) {
		stats.overruns++;
	}

	if (pie_ok) {
		/*
		 * Every store is cheap for the DAC in loop mode, but one core
		 * with 128-bit stores still beats two with 32-bit ones.
		 */
		timed_fill(0, bank, s, n, data);
	} else {
		uint32_t h = (n / 2U) & ~3U;

		half_job.bank = bank;
		half_job.s = s + h;
		half_job.n = n - h;
		half_job.data = data > h ? data - h : 0U;
		k_sem_give(&half_sem);
		timed_fill(0, bank, s, h, data);
		k_sem_take(&half_done_sem, K_FOREVER);
	}
	barrier_dmem_fence_full();
	return n - data;
}

static inline uint32_t ccount(void)
{
	uint32_t c;

	__asm__ volatile("rsr.ccount %0" : "=r"(c));
	return c;
}

/* Spin on this core's cycle counter until t; IRAM, so a flash operation cannot stall it. */
HOT static void spin_until(uint32_t t)
{
	while ((int32_t)(t - ccount()) > 0) {
	}
}

/*
 * Sleep, then spin until cycle t (this core's CCOUNT, the leader stays on its
 * CPU); returns with interrupts locked (the key) so the caller's register
 * write lands on time.
 */
static unsigned int wait_until_locked(uint32_t t)
{
	unsigned int key;
	int32_t d = (int32_t)(t - ccount());

	/* A sleep can end up to two ticks late (100 us each at 10 kHz): wake 300 us early. */
	if (d > (int32_t)k_us_to_cyc_ceil32(400)) {
		k_usleep((int32_t)k_cyc_to_us_floor32((uint32_t)d - k_us_to_cyc_ceil32(300)));
	}
	d = (int32_t)(ccount() - t);
	if (d > 0) {
		stats.wake_late_us_max = MAX(stats.wake_late_us_max, k_cyc_to_us_floor32((uint32_t)d));
	}
	spin_until(t - k_us_to_cyc_ceil32(5));
	key = irq_lock();
	spin_until(t);
	return key;
}

/*
 * Start a loop whose sample 0 is output sample `out` (a multiple of 4), at
 * the real time that output sample is due. Block 0 has no lead.
 */
static bool loop_start(uint64_t out)
{
	size_t words;
	uint32_t *bank = esp_sdr_dac_buf(0, &words);
	uint64_t t;
	int64_t d;
	unsigned int key;
	uint32_t zeros;

	out_base = out;
	zeros = MIN(fill_block(bank, 0, LOOP_STEP + LOOP_TAIL), LOOP_STEP);
	played_out -= zeros;
	/* Real time on the 64-bit clock, to this core's CCOUNT for the spin. */
	t = sess_t0 + out * cyc_per_sample;
	d = (int64_t)(t - k_cycle_get_64());
	if (out == 0U) {
		stats.start_margin_us = (int32_t)(d / (int64_t)k_us_to_cyc_ceil64(1));
	}
	if (d < (int64_t)k_us_to_cyc_ceil64(10)) {
		return false;
	}
	key = wait_until_locked(ccount() + (uint32_t)d);
	esp_sdr_dac_loop(0, BLOCK_WORDS);
	loop_t0 = ccount();
	irq_unlock(key);
	played_out += LOOP_STEP;
	return true;
}

/* Output sample to (re)start at: some fill time from now, a multiple of 4. */
static uint64_t restart_point(void)
{
	return ROUND_UP(out_now() + (uint64_t)esp_sdr_tx_rate_hz(DAC_RATE) / 1000U, 4U);
}

/*
 * A generator waveform restarted mid-way would radiate the middle of a
 * frame: end it instead (the restart plays one block of silence, and
 * esp_sdr_tx_dac_play_gen() reports -EIO).
 */
static void gen_abort(void)
{
	if (gen_fn != NULL) {
		gen_total = 0;
	}
}

/* Filler 0 leads: fills each block into the idle bank and switches on time. */
static void leader(void)
{
	uint64_t k = 0;
	uint32_t sw_t = 0;
	bool first = true;

	/* Output sample 0 (the first input) is due a little later: time to fill. */
	sess_t0 = k_cycle_get_64() + k_ms_to_cyc_ceil64(1);
	if (!loop_start(0)) {
		do {
			stats.restarts++;
			gen_abort();
		} while (!loop_start(restart_point()));
	}
	sw_t = k_cycle_get_32();
	/* A generator session ends once its last sample is in the running block. */
	while (!atomic_get(&stop_req) &&
	       !(gen_fn != NULL && out_base + (k + 1U) * LOOP_STEP >= gen_total)) {
		size_t words;
		int b = (int)((k + 1U) & 1U);
		uint32_t *bank = esp_sdr_dac_buf(b, &words);
		uint64_t s = (k + 1U) * LOOP_STEP;
		uint32_t t_sw = loop_t0 + (uint32_t)(s * cyc_per_sample);
		uint32_t zeros;

		stats.idle_us_max = MAX(stats.idle_us_max, k_cyc_to_us_floor32(k_cycle_get_32() - sw_t));
		zeros = fill_block(bank, s - LOOP_LEAD, BLOCK_WORDS);
		int32_t slack = (int32_t)(t_sw - ccount());
		int32_t slack_us = slack / (int32_t)k_us_to_cyc_ceil32(1);

		stats.slack_us_min = first ? slack_us : MIN(stats.slack_us_min, slack_us);
		first = false;
		if (slack < -(int32_t)((LOOP_TAIL - 64U) * cyc_per_sample)) {
			/* Past the old bank's tail: stale samples are playing. Start over. */
			uint64_t played_to = out_base + s;
			uint64_t next;

			esp_sdr_dac_halt();
			stats.errors++;
			stats.restarts++;
			gen_abort();
			do {
				next = restart_point();
				dropped_out += next - played_to;
				played_to = next;
			} while (!loop_start(next));
			sw_t = k_cycle_get_32();
			k = 0;
			continue;
		}
		{
			unsigned int key = wait_until_locked(t_sw);

			esp_sdr_dac_select(b);
			irq_unlock(key);
		}
		if (stats.switches > 0U) {
			uint32_t d = k_cyc_to_us_floor32(k_cycle_get_32() - sw_t);

			stats.switch_us_sum += d;
			stats.switch_us_max = MAX(stats.switch_us_max, d);
		}
		sw_t = k_cycle_get_32();
		stats.switches++;
		/* Input ran out in this block (not counted again while idle). */
		if (zeros > 0U && zeros < BLOCK_WORDS) {
			stats.underruns++;
		}
		/* The block just handed over, minus its zero filled share. */
		played_out += LOOP_STEP - MIN(zeros, LOOP_STEP);
		stats.played = played_out / interp;
		stats.dropped = dropped_out / interp + (uint64_t)atomic_get(&late_in);
		k++;
	}
	/* Let the running block play out (a generator: up to its last sample), then stop. */
	{
		uint64_t end = (k + 1U) * LOOP_STEP;

		if (gen_fn != NULL && gen_total > out_base && gen_total - out_base < end) {
			end = gen_total - out_base;
		}
		unsigned int key = wait_until_locked(loop_t0 + (uint32_t)(end * cyc_per_sample));

		esp_sdr_dac_halt();
		irq_unlock(key);
	}
	half_job.n = 0;
	k_sem_give(&half_sem); /* release filler 1 */
}

/* Filler 1: the second half of each block, on the other core (C fallback). */
static void helper(void)
{
	for (;;) {
		k_sem_take(&half_sem, K_FOREVER);
		if (half_job.n == 0U) {
			return;
		}
		timed_fill(1, half_job.bank, half_job.s, half_job.n, half_job.data);
		barrier_dmem_fence_full();
		k_sem_give(&half_done_sem);
	}
}

static void filler(void *p1, void *p2, void *p3)
{
	int f = (int)(intptr_t)p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		k_sem_take(&job_sem[f], K_FOREVER); /* session start */
		if (f == 0) {
			leader();
		} else {
			helper();
		}
		k_sem_give(&ready_sem[f]); /* session done */
	}
}

/* Session manager: front end and lock around the fillers' run. */
static void player(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		uint32_t prebuf;

		k_sem_take(&start_sem, K_FOREVER);
		prebuf = gen_fn != NULL ? 0U
					: MAX(in_rate / 1000U * CONFIG_ESP_SDR_TX_DAC_PREBUF_MS,
					      2U * BLOCK_WORDS / interp + 4U);
		while (!atomic_get(&stop_req) && (uint32_t)atomic_get(&ring_w) - ring_r < prebuf) {
			k_sleep(K_MSEC(1));
		}
		if (atomic_get(&stop_req) || esp_sdr_dac_begin(DAC_RATE) != 0) {
			sess_err = -EIO;
			k_sem_give(&done_sem);
			continue;
		}
		in_base = ring_r;
		atomic_set(&filled_in, (atomic_val_t)in_base);
		atomic_set(&late_in, 0);
		played_out = 0;
		dropped_out = 0;
		cyc_per_sample = sys_clock_hw_cycles_per_sec() / esp_sdr_tx_rate_hz(DAC_RATE);
		k_sem_reset(&half_sem);
		k_sem_reset(&half_done_sem);
		sess_t0 = k_cycle_get_64();
		for (int f = 0; f < NFILL; f++) {
			k_sem_give(&job_sem[f]);
		}
		while (gen_fn == NULL && !atomic_get(&stop_req)) {
			k_sleep(K_MSEC(5));
			/* The leader puts sess_t0 a little ahead at the start. */
			stats.elapsed_us = (uint32_t)k_cyc_to_us_floor64(
				(uint64_t)MAX((int64_t)(k_cycle_get_64() - sess_t0), 0));
		}
		for (int f = 0; f < NFILL; f++) {
			k_sem_take(&ready_sem[f], K_FOREVER);
		}
		esp_sdr_dac_end();
		k_sem_give(&done_sem);
	}
}

static Z_KERNEL_STACK_DEFINE_IN(player_stack, CONFIG_ESP_SDR_TX_DAC_STACK_SIZE, DAC_STACK);
static struct k_thread player_thread;

/* The fill loop needs little stack; internal RAM is tight. */
#define FILLER_STACK_SIZE 1536
static Z_KERNEL_STACK_ARRAY_DEFINE_IN(filler_stacks, NFILL, FILLER_STACK_SIZE, DAC_STACK);
static struct k_thread filler_threads[NFILL];

#if defined(CONFIG_ESP_SDR_TX_DAC_SIMD)
/* Vector ramp against the scalar one: odd alignment, both slopes, full scale. */
static bool pie_selftest(void)
{
	/* Scratch in the transmit bank, idle at boot. */
	uint32_t *got = __esp_sdr_bank1_start, *want = __esp_sdr_bank1_start + 256;
	static const int32_t cases[][4] = {
		{-512 << I_FRAC, 511 << Q_FRAC, 5 << (I_FRAC - 2), -(7 << (Q_FRAC - 3))},
		{511 << I_FRAC, -512 << Q_FRAC, -(1022 << I_FRAC) / 200, (1022 << Q_FRAC) / 200},
		{3 << (I_FRAC - 1), -(5 << (Q_FRAC - 1)), 1, -1},
	};

	pie_enable();
	for (size_t c = 0; c < ARRAY_SIZE(cases); c++) {
		for (uint32_t off = 0; off < 4U; off++) {
			uint32_t n = 199U;

			pie_ok = false;
			ramp(want + off, n, cases[c][0], cases[c][1], cases[c][2], cases[c][3]);
			pie_ok = true;
			ramp(got + off, n, cases[c][0], cases[c][1], cases[c][2], cases[c][3]);
			if (memcmp(got + off, want + off, n * sizeof(uint32_t)) != 0) {
				pie_ok = false;
				LOG_ERR("tx dac: vector ramp mismatch (case %u, offset %u), using C",
					(unsigned int)c, off);
				return false;
			}
		}
	}
	LOG_INF("tx dac: vector ramp self-test passed");
	return true;
}
#endif

/* One filler per core: both cores interpolate each block. */
static int fillers_init(void)
{
#if defined(CONFIG_ESP_SDR_TX_DAC_SIMD)
	pie_ok = pie_selftest();
#endif
	k_sem_init(&half_sem, 0, 1);
	k_sem_init(&half_done_sem, 0, 1);
	k_thread_create(&player_thread, player_stack, K_KERNEL_STACK_SIZEOF(player_stack), player,
			NULL, NULL, NULL, CONFIG_ESP_SDR_TX_DAC_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&player_thread, "esp_sdr_dac_player");
	for (int s = 0; s < NFILL; s++) {
		k_sem_init(&job_sem[s], 0, 1);
		k_sem_init(&ready_sem[s], 0, 1);
		k_thread_create(&filler_threads[s], filler_stacks[s],
				K_KERNEL_STACK_SIZEOF(filler_stacks[s]), filler, (void *)(intptr_t)s,
				NULL, NULL, CONFIG_ESP_SDR_TX_DAC_FILLER_PRIORITY, 0, K_FOREVER);
		k_thread_name_set(&filler_threads[s], s == 0 ? "dac_fill0" : "dac_fill1");
#if defined(CONFIG_SCHED_CPU_MASK)
		if (arch_num_cpus() > 1) {
			/* Filler 0 leads, the helper takes the other CPU. */
			k_thread_cpu_pin(&filler_threads[s],
					 s == 0 ? CONFIG_ESP_SDR_TX_DAC_LEADER_CPU
						: !CONFIG_ESP_SDR_TX_DAC_LEADER_CPU);
		}
#endif
		k_thread_start(&filler_threads[s]);
	}
	return 0;
}

SYS_INIT(fillers_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

const struct esp_sdr_tx_backend esp_sdr_tx_dac_backend = {
	.name = "dac",
	.start = dac_start,
	.write = dac_write,
	.stop = dac_stop,
};

int esp_sdr_tx_dac_play_gen(esp_sdr_tx_dac_gen_t gen, void *user, uint64_t samples)
{
	if (gen == NULL || samples == 0U) {
		return -EINVAL;
	}
	if (!atomic_cas(&running, 0, 1)) {
		return -EBUSY;
	}
	gen_fn = gen;
	gen_user = user;
	gen_total = samples;
	in_rate = esp_sdr_tx_rate_hz(DAC_RATE);
	interp = 1U;
	stats = (struct esp_sdr_tx_dac_stats){.interp = 1U};
	sess_err = 0;
	atomic_set(&stop_req, 0);
	k_sem_give(&start_sem);
	k_sem_take(&done_sem, K_FOREVER);
	gen_fn = NULL;
	atomic_set(&running, 0);
	return sess_err != 0 || stats.restarts != 0U ? -EIO : 0;
}

uint32_t esp_sdr_tx_dac_queued(void)
{
	int32_t q = (int32_t)((uint32_t)atomic_get(&ring_w) - (uint32_t)atomic_get(&filled_in));

	return q > 0 ? (uint32_t)q : 0U;
}

void esp_sdr_tx_dac_get_stats(struct esp_sdr_tx_dac_stats *out)
{
	*out = stats;
#if defined(CONFIG_ESP_SDR_TX_DAC_SIMD)
	out->simd = pie_ok;
#endif
}
