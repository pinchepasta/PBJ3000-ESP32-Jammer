/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The SDR behind the USB function: settings from the vendor requests, the
 * receive stream and the throughput tests.
 *
 * Receive: a cooperative worker thread pinned to CPU 1 runs
 * esp_sdr_ring_run(), which masks that CPU's interrupts except for short
 * windows in which pending ones run; its sink packs the
 * decimated samples into blocks (fine tuning NCO, 8 or 16 bit items). A
 * thread on CPU 0 hands full blocks to bulk IN. Blocks are used strictly in
 * order: prod (filled) >= sub (queued on USB) >= done (completed). With no
 * block free the sink drops samples and flags the next block. A settings
 * change stops the run, applies them and starts a new one; the sample index
 * keeps counting real time across the restart, so the host sees the hole.
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>
#include <esp_sdr/esp_sdr_ring.h>
#include <esp_sdr/esp_sdr_tx.h>

#include "esdr_proto.h"
#include "sdr.h"
#include "usb_sdr.h"

LOG_MODULE_REGISTER(sdr, LOG_LEVEL_INF);

#define RX_CPU  1
#define USB_CPU 0

/* LO below the centre with offset tuning: the ring's -fs/4 mix. */
#define OFFSET_HZ (ESP_SDR_RING_RATE_HZ / 4U)

/* Internal RAM for the USB DMA: what fits above the capture bank, the rest in DRAM. */
#define BLOCKS      CONFIG_APP_RX_BLOCKS
#define HIGH_BLOCKS MIN(BLOCKS, CONFIG_APP_RX_HIGH_BLOCKS)
ESP_SDR_HIGH_BUF_DEFINE(uint8_t, blocks_high, [HIGH_BLOCKS][ESDR_BLOCK_BYTES], 16);
#if BLOCKS > HIGH_BLOCKS
static uint8_t blocks_low[BLOCKS - HIGH_BLOCKS][ESDR_BLOCK_BYTES] __aligned(16);
#endif

static inline uint8_t *block(uint32_t k)
{
	k %= BLOCKS;
#if BLOCKS > HIGH_BLOCKS
	if (k >= HIGH_BLOCKS) {
		return blocks_low[k - HIGH_BLOCKS];
	}
#endif
	return blocks_high[k];
}

/* The OUT test sinks into receive blocks: the modes exclude each other. */
#define OUT_BUFS 2

static struct k_spinlock lock;
static struct sdr_settings want = {
	.mode = ESDR_MODE_OFF,
	.bits = 8,
	.options = ESDR_OPT_OFFSET_TUNING,
	.rx_gain = ESDR_GAIN_AUTO,
	.dgain = ESDR_DGAIN_DEFAULT,
	.freq_hz = (uint64_t)CONFIG_APP_FREQ_MHZ * 1000000U,
	.rate_hz = ESP_SDR_RING_RATE_HZ / ESP_SDR_RING_DECIM_MIN,
	/* Set in sdr_init() from CONFIG_APP_RX_BW_MHZ. */
};
static uint32_t want_gen;

/* Kconfig cannot express the gap: 1 to 12 MHz is below esp_sdr_rx_bandwidth_range(). */
BUILD_ASSERT(CONFIG_APP_RX_BW_MHZ <= 0 || CONFIG_APP_RX_BW_MHZ >= 13,
	     "CONFIG_APP_RX_BW_MHZ: -1, 0 or 13 to 69");

/* What the radio runs with (worker thread only). */
static struct sdr_settings cur;
static uint64_t cur_center_hz;
static uint32_t cur_lo_mhz;
static int32_t cur_lo_khz;

static struct esdr_stats stats;

static K_SEM_DEFINE(rx_wake, 0, 1);
static K_SEM_DEFINE(usb_wake, 0, 1);

/* Block pipeline, see above. */
static atomic_t prod, sub, done;

/* Sink state (worker thread, inside the run). */
static struct {
	uint8_t *blk;
	uint32_t fill, samples, max_samples;
	uint16_t flags;
	bool bits16;
	/* Digital gain as a left shift. */
	uint32_t shl;
	/* Expected index of the next sample, and the offset onto the stream's time axis. */
	uint64_t expect, base;
	/* Fine tuning: rotation per sample and the running phasor. */
	bool nco;
	float rc, rs, pc, ps;
	uint32_t renorm;
} snk;

/*
 * Transmit: OUT buffers (the receive blocks, idle meanwhile) go back to the
 * host only while the DAC backlog is short, which paces the host. Parked
 * buffers wait in a ring: pushed by the OUT completion, popped by the USB
 * thread.
 */
static atomic_t tx_on;
static uint8_t *tx_park[BLOCKS + 1];
static atomic_t tx_park_w, tx_park_r;
static uint32_t tx_limit;
static struct {
	bool bits16, nco;
	float rc, rs, pc, ps;
	uint32_t renorm;
	/* Bytes of a sample split across transfers. */
	uint8_t carry[4];
	uint32_t carry_len;
} txs;
/* Converted samples on their way to the backend. */
static struct esp_sdr_iq16 tx_conv[256] ESP_SDR_HIGH_RAM;

static void snapshot(struct sdr_settings *s, uint32_t *gen)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	*s = want;
	if (gen != NULL) {
		*gen = want_gen;
	}
	k_spin_unlock(&lock, key);
}

static uint32_t current_gen(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t g = want_gen;

	k_spin_unlock(&lock, key);
	return g;
}

/* Nearest supported rate on a log scale. */
static uint32_t rate_round(uint32_t hz)
{
	for (uint32_t d = ESP_SDR_RING_DECIM_MIN; d < ESP_SDR_RING_DECIM_MAX; d *= 2U) {
		uint32_t r = ESP_SDR_RING_RATE_HZ / d;

		/* Above the geometric mean of r and r / 2. */
		if (2U * (uint64_t)hz * hz >= (uint64_t)r * r) {
			return r;
		}
	}
	return ESP_SDR_RING_RATE_HZ / ESP_SDR_RING_DECIM_MAX;
}

/* ---- control requests (USB stack thread) ---- */

static void changed(void)
{
	want_gen++;
}

int usb_sdr_control_out(uint8_t req, uint16_t value, const uint8_t *data, size_t len)
{
	k_spinlock_key_t key;
	int ret = 0;

	key = k_spin_lock(&lock);
	switch (req) {
	case ESDR_REQ_SET_MODE:
		if (value != ESDR_MODE_OFF && value != ESDR_MODE_RX && value != ESDR_MODE_TX &&
		    value != ESDR_MODE_TEST_IN && value != ESDR_MODE_TEST_OUT) {
			ret = -ENOTSUP;
			break;
		}
		want.mode = value;
		changed();
		break;
	case ESDR_REQ_SET_FREQ:
		if (len != 8U) {
			ret = -EINVAL;
			break;
		}
		{
			uint64_t hz = sys_get_le64(data);

			if (hz < (uint64_t)ESP_SDR_FREQ_MIN_MHZ * 1000000U ||
			    hz > (uint64_t)ESP_SDR_FREQ_MAX_MHZ * 1000000U) {
				ret = -EINVAL;
				break;
			}
			want.freq_hz = hz;
		}
		changed();
		break;
	case ESDR_REQ_SET_SAMPLE_RATE:
		if (len != 4U) {
			ret = -EINVAL;
			break;
		}
		want.rate_hz = rate_round(sys_get_le32(data));
		changed();
		break;
	case ESDR_REQ_SET_TX_GAIN:
		if (value > (uint16_t)esp_sdr_tx_gain_max()) {
			ret = -EINVAL;
			break;
		}
		want.tx_gain = value;
		changed();
		break;
	case ESDR_REQ_SET_RX_GAIN:
		if (value != ESDR_GAIN_AUTO && value > CONFIG_APP_RX_GAIN_MAX) {
			ret = -EINVAL;
			break;
		}
		want.rx_gain = value;
		changed();
		break;
	case ESDR_REQ_SET_BANDWIDTH:
		if (len != 4U) {
			ret = -EINVAL;
			break;
		}
		want.bandwidth_hz = sys_get_le32(data);
		changed();
		break;
	case ESDR_REQ_SET_FORMAT:
		if (value != 8U && value != 16U) {
			ret = -EINVAL;
			break;
		}
		want.bits = value;
		changed();
		break;
	case ESDR_REQ_SET_FREQ_CORR:
		if (len != 4U) {
			ret = -EINVAL;
			break;
		}
		want.corr_ppb = (int32_t)sys_get_le32(data);
		changed();
		break;
	case ESDR_REQ_SET_OPTIONS:
		want.options = value & ESDR_OPT_OFFSET_TUNING;
		changed();
		break;
	case ESDR_REQ_SET_DIGITAL_GAIN:
		if (value != ESDR_DGAIN_DEFAULT && (value > ESDR_DGAIN_MAX_DB || value % 6U != 0U)) {
			ret = -EINVAL;
			break;
		}
		want.dgain = value;
		changed();
		break;
	default:
		ret = -ENOTSUP;
		break;
	}
	k_spin_unlock(&lock, key);
	if (ret == 0) {
		/* A running ring holds the radio: end the run so the worker applies the change. */
		esp_sdr_ring_stop();
		k_sem_give(&rx_wake);
		k_sem_give(&usb_wake);
	}
	return ret;
}

int usb_sdr_control_in(uint8_t req, uint16_t value, uint8_t *buf, size_t max)
{
	ARG_UNUSED(value);

	switch (req) {
	case ESDR_REQ_GET_INFO: {
		struct esdr_info info = {
			.magic = ESDR_INFO_MAGIC,
			.proto_version = ESDR_PROTO_VERSION,
			.info_size = sizeof(info),
			.freq_min_hz = (uint64_t)ESP_SDR_FREQ_MIN_MHZ * 1000000U,
			.freq_max_hz = (uint64_t)ESP_SDR_FREQ_MAX_MHZ * 1000000U,
			.rx_rate_max_hz = ESP_SDR_RING_RATE_HZ / ESP_SDR_RING_DECIM_MIN,
			.rx_rate_count = LOG2(ESP_SDR_RING_DECIM_MAX / ESP_SDR_RING_DECIM_MIN) + 1,
			.rx_gain_max = CONFIG_APP_RX_GAIN_MAX,
			.tx_gain_max = (uint16_t)esp_sdr_tx_gain_max(),
			.block_bytes = ESDR_BLOCK_BYTES,
			.full_scale16 = 32767,
		};

		strncpy(info.firmware, "esp-sdr osmosdr 0.1", sizeof(info.firmware) - 1);
		memcpy(buf, &info, MIN(max, sizeof(info)));
		return MIN(max, sizeof(info));
	}
	case ESDR_REQ_GET_STATE: {
		struct sdr_settings s;
		struct esdr_state st;

		snapshot(&s, NULL);
		st = (struct esdr_state){
			.mode = s.mode,
			.bits = s.bits,
			.options = s.options,
			.rx_gain = s.rx_gain,
			.tx_gain = s.tx_gain,
			.digital_gain_db = (uint16_t)sdr_dgain_db(&s),
			.freq_hz = s.freq_hz,
			.center_hz = cur_center_hz,
			.sample_rate_hz = s.rate_hz,
			.bandwidth_hz = s.bandwidth_hz,
			.freq_corr_ppb = s.corr_ppb,
			.lo_mhz = cur_lo_mhz,
			.lo_offset_khz = cur_lo_khz,
		};
		memcpy(buf, &st, MIN(max, sizeof(st)));
		return MIN(max, sizeof(st));
	}
	case ESDR_REQ_GET_STATS:
		memcpy(buf, &stats, MIN(max, sizeof(stats)));
		return MIN(max, sizeof(stats));
	default:
		return -ENOTSUP;
	}
}

/* ---- block pipeline ---- */

/* Drop everything queued on USB and wait for the cancelled transfers to complete. */
static void usb_flush(void)
{
	usb_sdr_cancel();
	for (int t = 0; t < 100 && atomic_get(&done) != atomic_get(&sub); t++) {
		k_sleep(K_MSEC(1));
	}
	atomic_set(&prod, atomic_get(&sub));
	atomic_set(&done, atomic_get(&sub));
}

/* Hand full blocks to bulk IN (USB thread, CPU 0). */
static void usb_thread_fn(void *a, void *b, void *c)
{
	uint16_t mode = ESDR_MODE_OFF;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		struct sdr_settings s;

		/* Transmit polls the DAC backlog; the rest waits for work. */
		(void)k_sem_take(&usb_wake, atomic_get(&tx_on) ? K_MSEC(2) : K_MSEC(100));
		snapshot(&s, NULL);
		if (atomic_get(&tx_on)) {
			while (atomic_get(&tx_park_r) != atomic_get(&tx_park_w) &&
			       esp_sdr_tx_dac_queued() < tx_limit) {
				uint32_t r = (uint32_t)atomic_get(&tx_park_r);

				(void)usb_sdr_submit_out(tx_park[r % ARRAY_SIZE(tx_park)],
							 ESDR_BLOCK_BYTES);
				atomic_set(&tx_park_r, (atomic_val_t)(r + 1U));
			}
			stats.tx_queued_max = MAX(stats.tx_queued_max, esp_sdr_tx_dac_queued());
			continue;
		}
		if (s.mode != mode) {
			/*
			 * Leaving a test mode: no partial block may stay queued, or the
			 * next session would start mid-block. Receive flushes itself
			 * when a new stream starts.
			 */
			if (mode == ESDR_MODE_TEST_IN || mode == ESDR_MODE_TEST_OUT) {
				mode = s.mode;
				usb_flush();
			}
			mode = s.mode;
			if (mode == ESDR_MODE_TEST_OUT && usb_sdr_enabled()) {
				for (int k = 0; k < OUT_BUFS; k++) {
					(void)usb_sdr_submit_out(block(k), ESDR_BLOCK_BYTES);
				}
			}
		}
		if (mode == ESDR_MODE_TEST_IN) {
			/* As fast as the bus takes them: keep every block queued. */
			while (usb_sdr_enabled() && (uint32_t)(atomic_get(&prod) - atomic_get(&done)) < BLOCKS) {
				uint32_t k = (uint32_t)atomic_get(&prod);
				uint8_t *blk = block(k);
				struct esdr_block_hdr h = {
					.magic = ESDR_BLOCK_MAGIC,
					.flags = ESDR_BLK_TEST,
					.samples = 0,
					.index = k,
				};

				memcpy(blk, &h, sizeof(h));
				atomic_inc(&prod);
			}
		}
		while (atomic_get(&sub) != atomic_get(&prod)) {
			uint32_t k = (uint32_t)atomic_get(&sub);

			if (usb_sdr_submit_in(block(k), ESDR_BLOCK_BYTES) != 0) {
				/* Host gone: the block counts as sent. */
				atomic_inc(&done);
			}
			atomic_inc(&sub);
		}
	}
}

void usb_sdr_in_done(uint8_t *data, size_t len, int err)
{
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	if (err == 0) {
		/* Always whole blocks (the controller consumes the buffer's length). */
		stats.usb_in_bytes += ESDR_BLOCK_BYTES;
	}
	atomic_inc(&done);
	k_sem_give(&usb_wake);
}

static inline int16_t tx_item(const uint8_t *p)
{
	return txs.bits16 ? (int16_t)sys_get_le16(p) : (int16_t)((int8_t)p[0] * 256);
}

/* Host samples to the backend, rotated by the fine tuning residual. */
static void tx_convert(const uint8_t *p, size_t len)
{
	const size_t item = txs.bits16 ? 4U : 2U;
	size_t n = 0;

	while (len > 0U) {
		const uint8_t *smp;

		if (txs.carry_len > 0U || len < item) {
			size_t k = MIN(item - txs.carry_len, len);

			memcpy(txs.carry + txs.carry_len, p, k);
			txs.carry_len += k;
			p += k;
			len -= k;
			if (txs.carry_len < item) {
				break;
			}
			txs.carry_len = 0;
			smp = txs.carry;
		} else {
			smp = p;
			p += item;
			len -= item;
		}
		int16_t vi = tx_item(smp), vq = tx_item(smp + item / 2U);

		if (txs.nco) {
			float fi = (float)vi, fq = (float)vq, t;

			vi = (int16_t)CLAMP((int32_t)(fi * txs.pc - fq * txs.ps), INT16_MIN, INT16_MAX);
			vq = (int16_t)CLAMP((int32_t)(fi * txs.ps + fq * txs.pc), INT16_MIN, INT16_MAX);
			t = txs.pc * txs.rc - txs.ps * txs.rs;
			txs.ps = txs.pc * txs.rs + txs.ps * txs.rc;
			txs.pc = t;
			if (++txs.renorm == 1024U) {
				float g = 1.5f - 0.5f * (txs.pc * txs.pc + txs.ps * txs.ps);

				txs.pc *= g;
				txs.ps *= g;
				txs.renorm = 0;
			}
		}
		tx_conv[n].i = vi;
		tx_conv[n].q = vq;
		if (++n == ARRAY_SIZE(tx_conv)) {
			(void)esp_sdr_tx_write(tx_conv, n);
			n = 0;
		}
	}
	if (n > 0U) {
		(void)esp_sdr_tx_write(tx_conv, n);
	}
}

void usb_sdr_out_done(uint8_t *data, size_t len, int err)
{
	struct sdr_settings s;

	if (err != 0) {
		return;
	}
	stats.usb_out_bytes += len;
	if (atomic_get(&tx_on)) {
		uint32_t w = (uint32_t)atomic_get(&tx_park_w);

		tx_convert(data, len);
		tx_park[w % ARRAY_SIZE(tx_park)] = data;
		atomic_set(&tx_park_w, (atomic_val_t)(w + 1U));
		k_sem_give(&usb_wake);
		return;
	}
	snapshot(&s, NULL);
	if (s.mode == ESDR_MODE_TEST_OUT) {
		(void)usb_sdr_submit_out(data, ESDR_BLOCK_BYTES);
	}
}

void usb_sdr_link(bool up)
{
	if (!up) {
		esp_sdr_ring_stop();
	}
	k_sem_give(&rx_wake);
	k_sem_give(&usb_wake);
}

/* ---- receive sink (CPU 1, interrupts masked between the ring's windows) ---- */

static inline void block_close(void)
{
	struct esdr_block_hdr *h = (struct esdr_block_hdr *)snk.blk;

	h->flags = snk.flags | (snk.bits16 ? ESDR_BLK_16BIT : 0U);
	h->samples = (uint16_t)snk.samples;
	/* Zero the unused tail: the host may read the whole block. */
	memset(snk.blk + sizeof(*h) + snk.fill, 0, ESDR_BLOCK_PAYLOAD - snk.fill);
	__asm__ volatile("memw" ::: "memory");
	atomic_inc(&prod);
	stats.rx_blocks++;
	snk.blk = NULL;
	snk.flags = 0;
	k_sem_give(&usb_wake);
}

static inline bool block_open(uint64_t index)
{
	struct esdr_block_hdr *h;

	if ((uint32_t)(atomic_get(&prod) - atomic_get(&done)) >= BLOCKS) {
		return false;
	}
	snk.blk = block((uint32_t)atomic_get(&prod));
	h = (struct esdr_block_hdr *)snk.blk;
	h->magic = ESDR_BLOCK_MAGIC;
	h->index = index;
	snk.fill = 0;
	snk.samples = 0;
	return true;
}

static inline int32_t sat(int32_t v, int32_t lim)
{
	return CLAMP(v, -lim - 1, lim);
}

/* Items of m samples at the block's fill position: 4-byte aligned, little endian. */
static void put_items(const int32_t *vi, const int32_t *vq, size_t m)
{
	uint8_t *o = snk.blk + sizeof(struct esdr_block_hdr) + snk.fill;
	const uint32_t shl = snk.shl;

	if (snk.bits16) {
		uint32_t *w = (uint32_t *)o;

		/* Ring full scale 16384 to 32768. */
		for (size_t k = 0; k < m; k++) {
			w[k] = (uint16_t)sat((vi[k] << shl) * 2, 32767) |
			       ((uint32_t)(uint16_t)sat((vq[k] << shl) * 2, 32767) << 16);
		}
		snk.fill += 4U * m;
	} else {
		uint16_t *h = (uint16_t *)o;

		/* Round to nearest: a plain shift floors and leaves a DC offset. */
		for (size_t k = 0; k < m; k++) {
			h[k] = (uint8_t)sat(((vi[k] << shl) + 64) >> 7, 127) |
			       ((uint16_t)(uint8_t)sat(((vq[k] << shl) + 64) >> 7, 127) << 8);
		}
		snk.fill += 2U * m;
	}
}

#define SINK_RUN 32

static void sink_pack(const int16_t *i, const int16_t *q, size_t m)
{
	int32_t vi[SINK_RUN], vq[SINK_RUN];

	while (m > 0U) {
		size_t r = MIN(m, SINK_RUN);

		for (size_t k = 0; k < r; k++) {
			vi[k] = i[k];
			vq[k] = q[k];
		}
		put_items(vi, vq, r);
		i += r;
		q += r;
		m -= r;
	}
}

/* With the fine tuning rotation. */
static void sink_nco(const int16_t *i, const int16_t *q, size_t m)
{
	int32_t vi[SINK_RUN], vq[SINK_RUN];

	while (m > 0U) {
		size_t r = MIN(m, SINK_RUN);

		for (size_t k = 0; k < r; k++) {
			float fi = (float)i[k], fq = (float)q[k], t;

			vi[k] = (int32_t)(fi * snk.pc - fq * snk.ps);
			vq[k] = (int32_t)(fi * snk.ps + fq * snk.pc);
			t = snk.pc * snk.rc - snk.ps * snk.rs;
			snk.ps = snk.pc * snk.rs + snk.ps * snk.rc;
			snk.pc = t;
		}
		/* First order magnitude correction keeps the phasor on the unit circle. */
		if ((snk.renorm += r) >= 1024U) {
			float g = 1.5f - 0.5f * (snk.pc * snk.pc + snk.ps * snk.ps);

			snk.pc *= g;
			snk.ps *= g;
			snk.renorm = 0;
		}
		put_items(vi, vq, r);
		i += r;
		q += r;
		m -= r;
	}
}

/* Runs inside the ring loop: every cycle here comes off the filter's margin. */
/* The settings generation the running ring was started for. */
static uint32_t run_gen;

static void sink(void *user, const int16_t *i, const int16_t *q, size_t n,
			  uint64_t index)
{
	ARG_UNUSED(user);

	/*
	 * esp_sdr_ring_run() clears a stop requested just before it starts: a
	 * change that slipped in there ends the run from here instead.
	 */
	if (*(volatile uint32_t *)&want_gen != run_gen) {
		esp_sdr_ring_stop();
	}
	index += snk.base;
	if (index != snk.expect) {
		/* Ring input lost: the block in progress ends before the hole. */
		if (snk.blk != NULL) {
			block_close();
		}
		snk.flags |= ESDR_BLK_GAP;
	}
	for (size_t k = 0; k < n;) {
		size_t m;

		if (snk.blk == NULL && !block_open(index + k)) {
			stats.rx_overflow_samples += n - k;
			stats.rx_overflows++;
			snk.flags |= ESDR_BLK_OVERFLOW | ESDR_BLK_GAP;
			break;
		}
		/* A run of samples into the open block. */
		m = MIN(n - k, snk.max_samples - snk.samples);
		if (snk.nco) {
			sink_nco(&i[k], &q[k], m);
		} else {
			sink_pack(&i[k], &q[k], m);
		}
		k += m;
		snk.samples += m;
		if (snk.samples == snk.max_samples) {
			block_close();
		}
	}
	snk.expect = index + n;
	stats.rx_samples += n;
}

/* ---- receive worker (CPU 1) ---- */

/*
 * LO plan: the wanted LO @p lo as whole MHz plus a kHz PLL offset on the
 * corrected crystal; returns how far the real LO lies below the wanted one,
 * for the NCO.
 */
static double lo_plan(double lo, int32_t corr_ppb, uint32_t *mhz, int32_t *khz)
{
	double corr = 1.0 + (double)corr_ppb * 1e-9;
	double nominal = lo / corr;

	*mhz = (uint32_t)llround(nominal / 1e6);
	*khz = (int32_t)llround((nominal - (double)*mhz * 1e6) / 1e3);
	return lo - ((double)*mhz * 1e6 + (double)*khz * 1e3) * corr;
}

/* Offset tuning, unless the LO would fall below the tuning range. */
static bool use_offset(const struct sdr_settings *s)
{
	return (s->options & ESDR_OPT_OFFSET_TUNING) != 0U &&
	       s->freq_hz >= (uint64_t)ESP_SDR_FREQ_MIN_MHZ * 1000000U + OFFSET_HZ;
}

static int radio_apply(const struct sdr_settings *s, bool force)
{
	bool offset = use_offset(s);
	double lo = (double)s->freq_hz - (offset ? (double)OFFSET_HZ : 0.0);
	uint32_t mhz;
	int32_t khz;
	double residual = lo_plan(lo, s->corr_ppb, &mhz, &khz);
	double actual = lo - residual;
	int ret = 0;

	if (force || mhz != cur_lo_mhz || khz != cur_lo_khz) {
		ret = esp_sdr_set_freq(mhz);
		if (ret == 0 && (force || khz != cur_lo_khz)) {
			ret = esp_sdr_set_freq_offset(khz);
		}
		if (ret != 0) {
			LOG_ERR("tune %u MHz %+d kHz: %d", mhz, khz, ret);
			return ret;
		}
		cur_lo_mhz = mhz;
		cur_lo_khz = khz;
	}
	if (force || s->rx_gain != cur.rx_gain) {
		ret = esp_sdr_rx_set_gain(s->rx_gain == ESDR_GAIN_AUTO ? ESP_SDR_RX_GAIN_AUTO
								: (int)s->rx_gain);
		if (ret != 0) {
			LOG_ERR("rx gain %u: %d", s->rx_gain, ret);
		}
	}
	if (force || s->bandwidth_hz != cur.bandwidth_hz) {
		/* 0 is the PHY's own setting (ESDR_REQ_SET_BANDWIDTH). */
		ret = s->bandwidth_hz == 0U ? esp_sdr_rx_set_lpf(ESP_SDR_RX_LPF_AUTO)
					    : esp_sdr_rx_set_bandwidth(s->bandwidth_hz / 1000000U);
		if (ret != 0) {
			LOG_ERR("bandwidth %u Hz: %d", s->bandwidth_hz, ret);
		}
	}
	/* The NCO shifts the residual down: rotate by -residual. */
	{
		double w = -2.0 * 3.14159265358979323846 * residual / (double)s->rate_hz;

		snk.nco = fabs(residual) > 0.5;
		snk.rc = (float)cos(w);
		snk.rs = (float)sin(w);
		snk.pc = 1.0f;
		snk.ps = 0.0f;
		snk.renorm = 0;
	}
	cur_center_hz = (uint64_t)llround(actual + residual + (offset ? (double)OFFSET_HZ : 0.0));
	cur = *s;
	return 0;
}

/* One transmit session: until the mode or any setting changes. */
static void tx_session(const struct sdr_settings *s, uint32_t gen)
{
	struct esp_sdr_tx_dac_stats ds;
	uint32_t mhz;
	int32_t khz;
	double residual = lo_plan((double)s->freq_hz, s->corr_ppb, &mhz, &khz);
	double w = 2.0 * 3.14159265358979323846 * residual / (double)s->rate_hz;
	int ret;

	/* The real LO lies residual below the wanted one: rotate the samples up by it. */
	txs.bits16 = s->bits == 16U;
	txs.nco = fabs(residual) > 0.5;
	txs.rc = (float)cos(w);
	txs.rs = (float)sin(w);
	txs.pc = 1.0f;
	txs.ps = 0.0f;
	txs.renorm = 0;
	txs.carry_len = 0;
	/* 50 ms ahead, beyond the backend's 20 ms start buffer. */
	tx_limit = s->rate_hz / 20U + ESDR_BLOCK_BYTES;

	ret = esp_sdr_set_freq_offset(khz);
	if (ret == 0) {
		ret = esp_sdr_tx_set_gain(s->tx_gain);
	}
	if (ret == 0) {
		ret = esp_sdr_tx_start((uint64_t)mhz * 1000000U, s->rate_hz);
	}
	cur_lo_mhz = 0;
	cur_center_hz = s->freq_hz;
	if (ret != 0) {
		LOG_ERR("tx start %u MHz %+d kHz at %u S/s: %d", mhz, khz, s->rate_hz, ret);
		while (current_gen() == gen) {
			(void)k_sem_take(&rx_wake, K_MSEC(100));
		}
		return;
	}
	stats.tx_sessions++;
	usb_flush();
	atomic_set(&tx_park_r, 0);
	for (uint32_t k = 0; k < BLOCKS; k++) {
		tx_park[k] = block(k);
	}
	atomic_set(&tx_park_w, BLOCKS);
	atomic_set(&tx_on, 1);
	k_sem_give(&usb_wake);

	while (current_gen() == gen && usb_sdr_enabled()) {
		(void)k_sem_take(&rx_wake, K_MSEC(100));
		esp_sdr_tx_dac_get_stats(&ds);
		stats.tx_samples = ds.played;
		stats.tx_dropped = ds.dropped;
		stats.tx_underruns = ds.underruns;
		stats.tx_restarts = ds.restarts;
	}

	/* No more submissions, then cancel what the host may still fill. */
	atomic_set(&tx_on, 0);
	k_sem_give(&usb_wake);
	k_sleep(K_MSEC(5));
	usb_flush();
	(void)esp_sdr_tx_stop();
	esp_sdr_tx_dac_get_stats(&ds);
	stats.tx_samples = ds.played;
	stats.tx_dropped = ds.dropped;
	stats.tx_underruns = ds.underruns;
	stats.tx_restarts = ds.restarts;
}

static void rx_thread_fn(void *a, void *b, void *c)
{
	struct esp_sdr_ring_stats rst;
	struct esp_sdr_ring_cfg cfg = {.sink = sink};
	uint64_t end_cycles = 0;
	bool first = true, was_rx = false;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		struct sdr_settings s;
		uint32_t gen;
		int ret;

		snapshot(&s, &gen);
		if (s.mode == ESDR_MODE_TX && usb_sdr_enabled()) {
			was_rx = false;
			tx_session(&s, gen);
			/* The transmitter moved the LO: retune for the next receive. */
			first = true;
			continue;
		}
		if (s.mode != ESDR_MODE_RX || !usb_sdr_enabled()) {
			if (was_rx) {
				/* Stream over: drop the partial block, restart the axis next time. */
				snk.blk = NULL;
				was_rx = false;
			}
			(void)k_sem_take(&rx_wake, K_FOREVER);
			continue;
		}
		if (radio_apply(&s, first) != 0) {
			k_sleep(K_MSEC(100));
			continue;
		}
		first = false;

		if (!was_rx) {
			/* A new stream: blocks of an old one must not reach the host. */
			usb_flush();
			/* The index starts at 0. */
			snk.base = 0;
			snk.expect = 0;
			snk.blk = NULL;
			snk.flags = 0;
			end_cycles = 0;
			was_rx = true;
		} else {
			/* Keep the time axis: count the restart as missing samples. */
			uint64_t gap = (k_cycle_get_64() - end_cycles) * s.rate_hz /
				       sys_clock_hw_cycles_per_sec();

			if (snk.blk != NULL) {
				block_close();
			}
			snk.base = snk.expect + gap;
			snk.expect = snk.base;
			stats.rx_restart_samples += gap;
		}
		snk.bits16 = s.bits == 16U;
		snk.shl = sdr_dgain_db(&s) / 6U;
		snk.max_samples = ESDR_BLOCK_PAYLOAD / (snk.bits16 ? 4U : 2U);
		cfg.decim = ESP_SDR_RING_RATE_HZ / s.rate_hz;
		cfg.shift_fs4 = use_offset(&s);

		/* A change between the snapshot and here must not wait for the next one. */
		if (current_gen() != gen) {
			continue;
		}
		stats.ring_runs++;
		run_gen = gen;
		ret = esp_sdr_ring_run(&cfg, &rst);
		end_cycles = k_cycle_get_64();
		stats.rx_ring_lost_pairs += rst.lost_pairs;
		stats.ring_abandoned += rst.abandoned;
		stats.ring_gain_refreshed += rst.gain_refreshed;
		stats.ring_late_max = MAX(stats.ring_late_max, rst.late_max);
		stats.ring_cycles_x100 = rst.cycles_x100;
		if (ret != 0) {
			stats.ring_errors++;
			stats.ring_last_status = rst.status;
			stats.ring_last_detail = rst.detail;
			LOG_WRN("ring stopped: %d, status %d detail 0x%08x after %u units", ret,
				rst.status, rst.detail, rst.units);
			k_sleep(K_MSEC(10));
		}
	}
}

/*
 * Stacks above the capture bank: the DRAM image must end below the ring banks.
 * Receive peak: 1808 bytes, 1920 in multiapp.
 */
ESP_SDR_HIGH_STACK_DEFINE(sdr_rx_stack, 2560);
ESP_SDR_HIGH_STACK_DEFINE(sdr_usb_stack, 1024);
static struct k_thread rx_thread, usb_thread;

unsigned int sdr_dgain_db(const struct sdr_settings *s)
{
	if (s->dgain != ESDR_DGAIN_DEFAULT) {
		return s->dgain;
	}
	return s->bits == 16U ? 0U : 24U;
}

void sdr_get_settings(struct sdr_settings *s)
{
	snapshot(s, NULL);
}

static void null_sink(void *user, const int16_t *i, const int16_t *q, size_t n, uint64_t index)
{
	ARG_UNUSED(user);
	ARG_UNUSED(i);
	ARG_UNUSED(q);
	ARG_UNUSED(n);
	ARG_UNUSED(index);
}

int sdr_bench(unsigned int decim, bool real_sink, bool rot, struct esp_sdr_ring_bench_result *r)
{
	struct esp_sdr_ring_cfg cfg = {
		.decim = decim, .shift_fs4 = rot, .sink = real_sink ? sink : null_sink,
	};
	struct sdr_settings s;

	snapshot(&s, NULL);
	if (s.mode != ESDR_MODE_OFF) {
		return -EBUSY;
	}
	/* The real sink fills blocks: start from an empty pipeline, leave it empty. */
	snk.blk = NULL;
	snk.max_samples = ESDR_BLOCK_PAYLOAD / 4U;
	snk.bits16 = true;
	usb_flush();
	int ret = esp_sdr_ring_bench(&cfg, 8, r);

	atomic_set(&prod, atomic_get(&sub));
	return ret;
}

void sdr_get_stats(struct esdr_stats *s)
{
	*s = stats;
}

int sdr_init(void)
{
	int ret = esp_sdr_init();

	if (ret != 0) {
		LOG_ERR("radio init: %d", ret);
		return ret;
	}
	(void)esp_sdr_set_channel_bw(CONFIG_APP_CBW);
	{
		/* Kconfig as in apps/link (0 widest, -1 the PHY's); the protocol's 0 is the PHY's. */
		uint32_t bw_min, bw_max;

		esp_sdr_rx_bandwidth_range(&bw_min, &bw_max);
		want.bandwidth_hz = CONFIG_APP_RX_BW_MHZ < 0
					    ? 0U
					    : (CONFIG_APP_RX_BW_MHZ == 0 ? bw_max : (uint32_t)CONFIG_APP_RX_BW_MHZ) *
						      1000000U;
	}

	k_thread_create(&rx_thread, sdr_rx_stack, K_KERNEL_STACK_SIZEOF(sdr_rx_stack), rx_thread_fn, NULL,
			NULL, NULL, K_PRIO_COOP(2), K_FP_REGS, K_FOREVER);
	k_thread_create(&usb_thread, sdr_usb_stack, K_KERNEL_STACK_SIZEOF(sdr_usb_stack), usb_thread_fn,
			NULL, NULL, NULL, K_PRIO_PREEMPT(3), 0, K_FOREVER);
	k_thread_name_set(&rx_thread, "sdr_radio");
	k_thread_name_set(&usb_thread, "sdr_usb");
#if defined(CONFIG_SCHED_CPU_MASK)
	if (arch_num_cpus() > 1) {
		k_thread_cpu_pin(&rx_thread, RX_CPU);
		k_thread_cpu_pin(&usb_thread, USB_CPU);
	}
#endif
	k_thread_start(&rx_thread);
	k_thread_start(&usb_thread);
	return 0;
}
