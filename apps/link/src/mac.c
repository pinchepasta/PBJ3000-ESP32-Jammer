/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * CSMA/CA MAC, see mac.h. The MAC thread alternates sense slots (a short
 * capture and its power) with the work they call for:
 *
 *   busy slot   capture a full window, decode it; a data frame for us is
 *               ACKed at once (no carrier sense: the sender is waiting)
 *   quiet slot  count DIFS, then the backoff; at zero the pending frame
 *               goes on air for data_air_us
 *
 * After a transmission the sender listens up to ack_timeout_us for the ACK,
 * else doubles the contention window and contends again, up to `retries`.
 * Bank 1 holds the pending data frame (retries replay it unchanged); ACKs
 * are built into bank 0, the capture bank, once its capture is decoded.
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>

#include <esp_attr.h>

#include "mac.h"
#include "ofdm.h"
#include "ofdm_frame.h"
#include "qam.h"
#include "hamming.h"
#include "rs.h"

LOG_MODULE_REGISTER(link_mac, LOG_LEVEL_INF);

#define RADIO_RATE      ESP_SDR_RATE_80MSPS
#define SENSE_SAMPLES   512U
#define WINDOW_SAMPLES  ESP_SDR_SAMPLES_MAX
/* An ACK copy plus the search margin, whole kilo-samples. */
#define ACK_WINDOW_SAMPLES                                                                 \
	MIN(((QAM_PRE_SYMS + QAM_HDR_SYMS) * QAM_SPS + QAM_SEARCH_MARGIN + 1023U) / 1024U * 1024U, \
	    (unsigned int)ESP_SDR_SAMPLES_MAX)
#define DATA_BANK       1
#define ACK_BANK        0
#define CAL_SLOTS       32
#define MAC_STACK_SIZE  6144
#define BUILD_STACK_SIZE 2048
#define RX2_STACK_SIZE   4096
#define WIN_MAX          2

BUILD_ASSERT(QAM_FRAME_MAX_SAMPLES + QAM_SEARCH_MARGIN <= WINDOW_SAMPLES);
BUILD_ASSERT((QAM_PRE_SYMS + QAM_HDR_SYMS) * QAM_SPS + QAM_SEARCH_MARGIN <= ACK_WINDOW_SAMPLES);
BUILD_ASSERT(ESP_SDR_BANKS >= 2, "the MAC needs CONFIG_ESP_SDR_BANK1");

static struct mac_cfg cfg = {
	.phy = MAC_PHY_QAM,
	.freq_mhz = CONFIG_APP_FREQ_MHZ,
	.tx_gain = CONFIG_APP_TX_GAIN,
	.rx_gain = CONFIG_APP_RX_GAIN,
	.bw_mhz = CONFIG_APP_RX_BW_MHZ,
	.cbw = CONFIG_APP_CBW,
	.avg = 8,
	.tx_lpf_a = -1,
	.tx_lpf_b = -1,
	.amp = CONFIG_APP_AMP,
	.data_air_us = CONFIG_APP_DATA_AIR_US,
	.ack_air_us = CONFIG_APP_ACK_AIR_US,
	.ack_timeout_us = CONFIG_APP_ACK_TIMEOUT_US,
	.turn_us = CONFIG_APP_TURN_US,
	.turn_retune = IS_ENABLED(CONFIG_APP_TURN_RETUNE),
	.cca_db = CONFIG_APP_CCA_DB,
	.difs = 2,
	.cw_min = 15,
	.cw_max = 255,
	.retries = 7,
};

/* Kconfig cannot express the gap: 1 to 12 MHz is below esp_sdr_rx_bandwidth_range(). */
BUILD_ASSERT(CONFIG_APP_RX_BW_MHZ <= 0 || CONFIG_APP_RX_BW_MHZ >= 13,
	     "CONFIG_APP_RX_BW_MHZ: -1, 0 or 13 to 69");

/* Pulse tables and frame buffers: above the capture bank, outside the libc heap. */
static ESP_SDR_HIGH_RAM struct qam_tx qam_tx_ctx;
/* ACKs are built while the data context may be busy; they are short, PSRAM will do. */
static EXT_RAM_BSS_ATTR struct qam_tx ack_tx_ctx;
/* Two decoders (a pair decodes on both CPUs): state here, codewords in PSRAM. */
static struct qam_rx qam_rx_ctx, qam_rx2_ctx;
static EXT_RAM_BSS_ATTR uint8_t rx_cw[2][QAM_RX_CW_BYTES];
/* Test data, only copied into the codewords: PSRAM is fast enough. */
static EXT_RAM_BSS_ATTR uint8_t payload[WIN_MAX][QAM_NCW_MAX * QAM_UNIT];
/*
 * OFDM frames: transmit buffers for the builder, receive ones for the MAC
 * thread. The FFT buffer in internal RAM (in PSRAM a build took 20 ms), the
 * byte work in PSRAM.
 */
static struct ofdm_txbuf ofdm_tb __aligned(16);
static EXT_RAM_BSS_ATTR struct ofdm_fwork ofdm_txw, ofdm_rxw;
static EXT_RAM_BSS_ATTR uint8_t ofdm_rx_data[OFDM_FRAME_CAP_MAX];
/* OFDM ACKs, built by the MAC thread (the builder may be using ofdm_tb meanwhile). */
static struct ofdm_txbuf ofdm_ack_tb __aligned(16);
/* The second half of an OFDM payload is demodulated on the other CPU with this worker. */
static struct ofdm_worker ofdm_wk2 __aligned(16);

static struct mac_stats stats;
static struct k_spinlock stats_lock;
static float noise_floor = 1.0f;
static float cca_threshold = 10.0f;

/* Requests from the shell, picked up by the MAC thread. */
static K_MUTEX_DEFINE(cfg_lock);
static atomic_t cfg_dirty, cal_req;
static K_SEM_DEFINE(cal_done, 0, 1);

static struct {
	bool active;
	uint8_t dst;
	enum qam_mod mod;
	enum qam_fec fec;
	unsigned int ncw;
	unsigned int window;
	int64_t end_ms;
	uint32_t rate_kbps;
} client, client_req;
static atomic_t client_dirty;
static int64_t client_end_ms;

/* Frames that go on air together: one, or a pair looped as one (qam_tx_build_pair()). */
struct txf {
	struct qam_hdr hdr;
	bool acked;
	/* Transmissions without an ACK for this frame. */
	unsigned int tries;
};

/* What is in flight. */
static struct {
	bool pending;
	bool wait_ack;
	unsigned int n;
	struct txf f[WIN_MAX];
	size_t len;
	/* The data bank still holds f[] (a background build may have replaced it). */
	bool built;
	/* An ACK covered only part of f[]. */
	bool partial;
	unsigned int cw;
	int backoff;
	unsigned int quiet;
	uint32_t ack_deadline;
	int64_t next_due_us;
} out;

/*
 * Per source: the highest sequence number seen and which of the 64 before it
 * arrived. Pairs may resend an older frame next to a new one, so order is
 * not monotonic; a frame that leaves the window unseen counts as lost.
 */
#define SEQ_WINDOW 64

static EXT_RAM_BSS_ATTR struct {
	bool seen;
	uint16_t max_seq;
	uint64_t got;
} srcs[256];
static uint16_t next_seq;

static atomic_t dump_req, paused, clear_req;
static K_SEM_DEFINE(pause_ack, 0, 1);
static bool pause_acked;
static K_SEM_DEFINE(dump_ready, 0, 1);
static K_SEM_DEFINE(dump_done, 0, 1);

/* Work for the paused MAC thread (link_mac_run_paused()). */
struct paused_job {
	void (*fn)(void *arg);
	void *arg;
};
static atomic_ptr_t paused_job;
static K_SEM_DEFINE(paused_done, 0, 1);
/* Set once the radio is up and the MAC thread loops; it never stops after that. */
static atomic_t mac_running;
static const uint32_t *dump_words;
static size_t dump_n;

K_THREAD_STACK_DEFINE(mac_stack, MAC_STACK_SIZE);
static struct k_thread mac_thread;

#define STAT(expr)                                                                         \
	do {                                                                               \
		k_spinlock_key_t key_ = k_spin_lock(&stats_lock);                          \
		expr;                                                                      \
		k_spin_unlock(&stats_lock, key_);                                          \
	} while (0)

static inline uint32_t now_cyc(void)
{
	return k_cycle_get_32();
}

static inline uint32_t cyc_us(uint32_t cyc)
{
	return k_cyc_to_us_floor32(cyc);
}

/* The OFDM modulation of a QAM one (data frames carry it in qam_hdr.mod). */
static enum ofdm_mod ofdm_mod_of(unsigned int m)
{
	return m == QAM_QPSK ? OFDM_QPSK : m == QAM_QAM16 ? OFDM_16QAM
		: m == QAM_QAM64 ? OFDM_64QAM : OFDM_MODS;
}

static unsigned int qam_mod_of(enum ofdm_mod m)
{
	return m == OFDM_QPSK ? QAM_QPSK : m == OFDM_16QAM ? QAM_QAM16
		: m == OFDM_64QAM ? QAM_QAM64 : QAM_MODS;
}

/* User bytes of data frame @p h with the PHY in use. */
static size_t frame_bytes(const struct qam_hdr *h)
{
	if (cfg.phy == MAC_PHY_OFDM) {
		enum ofdm_mod m = ofdm_mod_of(h->mod);

		return m < OFDM_MODS ? ofdm_frame_user_bytes(link_ofdm(), m) : 0U;
	}
	return qam_hdr_payload_bytes(h);
}

size_t link_mac_frame_bytes(enum qam_mod mod)
{
	struct qam_hdr h = {.mod = (uint8_t)mod, .fec = QAM_FEC_RS, .full = 1};

	return frame_bytes(&h);
}

static void calibrate(void);

static void apply_cfg(void)
{
	int ret;

	ret = esp_sdr_set_channel_bw((unsigned int)cfg.cbw);
	if (ret == 0) {
		ret = esp_sdr_set_freq(cfg.freq_mhz);
	}
	if (ret == 0) {
		ret = esp_sdr_rx_set_gain(cfg.rx_gain);
	}
	if (ret == 0) {
		ret = esp_sdr_tx_set_gain(cfg.tx_gain);
	}
	if (ret == 0) {
		ret = esp_sdr_set_turnaround(cfg.turn_us, cfg.turn_retune);
	}
	if (ret == 0) {
		ret = esp_sdr_tx_set_lpf(cfg.tx_lpf_a, cfg.tx_lpf_b);
	}
	if (ret == 0) {
		ret = cfg.bw_mhz < 0 ? esp_sdr_rx_set_lpf(ESP_SDR_RX_LPF_AUTO)
				     : esp_sdr_rx_set_bandwidth((uint32_t)cfg.bw_mhz);
	}
	if (ret != 0) {
		LOG_ERR("radio settings rejected (%d)", ret);
	}
	/* Gain, filter and frequency move the noise floor: measure it again. */
	calibrate();
	/* Sample clock tracking follows the carrier offset. */
	qam_rx_ctx.carrier_hz = (float)cfg.freq_mhz * 1e6f;
	qam_rx2_ctx.carrier_hz = qam_rx_ctx.carrier_hz;
	qam_rx_ctx.avg = cfg.avg;
	qam_rx2_ctx.avg = cfg.avg;
}

/* One sense slot: power of a short capture, or a negative errno. */
static float sense(void)
{
	struct esp_sdr_rx_burst b;
	uint32_t t0 = now_cyc();
	float p;

	if (esp_sdr_rx_capture(RADIO_RATE, SENSE_SAMPLES, &b) != 0) {
		return -1.0f;
	}
	p = qam_rx_power(b.words, b.count);
	STAT(stats.sense_us_sum += cyc_us(now_cyc() - t0); stats.sense_n++;
	     stats.sense_engine_us = b.elapsed_us);
	return p;
}

static void calibrate(void)
{
	float p[CAL_SLOTS];

	for (int i = 0; i < CAL_SLOTS; i++) {
		p[i] = sense();
	}
	/* Median: a frame on air during calibration only moves the top. */
	for (int i = 1; i < CAL_SLOTS; i++) {
		for (int j = i; j > 0 && p[j - 1] > p[j]; j--) {
			float t = p[j];

			p[j] = p[j - 1];
			p[j - 1] = t;
		}
	}
	noise_floor = MAX(p[CAL_SLOTS / 2], 1e-3f);
	cca_threshold = noise_floor * powf(10.0f, (float)cfg.cca_db / 10.0f);
	LOG_INF("noise floor %.1f, carrier sense above %.1f (raw units squared)",
		(double)noise_floor, (double)cca_threshold);
}

/* Play bank @p bank in a loop for @p us, at @p rate. */
static void transmit(int bank, size_t len, uint32_t us, enum esp_sdr_rate rate)
{
	uint32_t t0 = now_cyc(), t1, t2;

	if (esp_sdr_tx_loop_begin(rate) != 0) {
		return;
	}
	t1 = now_cyc();
	(void)esp_sdr_tx_loop_start(bank, len);
	k_busy_wait(us);
	t2 = now_cyc();
	esp_sdr_tx_loop_end();
	STAT(stats.tx_on_us_sum += cyc_us(t1 - t0);
	     stats.tx_off_us_sum += cyc_us(now_cyc() - t2); stats.tx_switches++);
}

/*
 * ACK frames carry no payload, so their modulation field is free: bit 0
 * says the other frame of a pair arrived as well.
 */
#define ACK_OTHER BIT(0)

static void send_ack(uint8_t dst, uint16_t seq, bool other)
{
	struct qam_hdr h = {
		.type = MAC_ACK,
		.mod = other ? ACK_OTHER : 0U,
		.ncw = 0,
		.dst = dst,
		.src = cfg.addr,
		.seq = seq,
	};
	size_t words;
	uint32_t *bank = esp_sdr_tx_loop_buf(ACK_BANK, &words);
	size_t len;

	if (cfg.phy == MAC_PHY_OFDM) {
		/* A header-only OFDM frame: decoded in a fraction of a QAM ACK's time. */
		struct ofdm_fhdr fh = {.type = MAC_ACK, .mod = OFDM_BPSK, .dst = dst,
				       .src = cfg.addr, .seq = seq};

		ARG_UNUSED(other);
		len = ofdm_frame_build_hdr(link_ofdm(), &ofdm_ack_tb, &fh, bank);
		if (len != 0) {
			transmit(ACK_BANK, len, cfg.ack_air_us, link_ofdm_tx_rate());
			STAT(stats.acks_sent++);
		}
		return;
	}
	if (ack_tx_ctx.amp != cfg.amp) {
		qam_tx_init(&ack_tx_ctx, QAM_QPSK, cfg.amp);
	}
	/* The context's modulation must match the header's for the build check. */
	ack_tx_ctx.mod = h.mod;
	len = qam_tx_build(&ack_tx_ctx, &h, NULL, bank);
	if (len == 0) {
		return;
	}
	transmit(ACK_BANK, len, cfg.ack_air_us, RADIO_RATE);
	STAT(stats.acks_sent++);
}

/* Count a data frame for us; false if it is a duplicate. */
static bool on_data(const struct qam_hdr *h)
{
	size_t bytes = frame_bytes(h);
	uint32_t lost = 0;
	bool dup = false;
	typeof(srcs[0]) *st = &srcs[h->src];
	int16_t d = (int16_t)(h->seq - st->max_seq);

	if (!st->seen) {
		/* Numbers before the first frame were never expected: mark them received. */
		st->seen = true;
		st->max_seq = h->seq;
		st->got = UINT64_MAX;
	} else if (d > 0) {
		/*
		 * Bit k of got: max_seq - k arrived. Advancing by d pushes bits
		 * SEQ_WINDOW - d .. SEQ_WINDOW - 1 out of the window: those not set
		 * are lost, as are numbers skipped past the window entirely.
		 */
		if (d >= SEQ_WINDOW) {
			lost = SEQ_WINDOW - (uint32_t)__builtin_popcountll(st->got) +
			       (uint32_t)(d - SEQ_WINDOW);
			st->got = 1;
		} else {
			uint64_t leaving = st->got >> (SEQ_WINDOW - d);

			lost = (uint32_t)d - (uint32_t)__builtin_popcountll(leaving);
			st->got = (st->got << d) | 1;
		}
		st->max_seq = h->seq;
	} else if (-d >= SEQ_WINDOW) {
		/* Far behind: the sender restarted with new numbers. Start over. */
		st->max_seq = h->seq;
		st->got = UINT64_MAX;
	} else if (!(st->got & BIT64(-d))) {
		/* Late: a resent frame from inside the window. */
		st->got |= BIT64(-d);
	} else {
		dup = true;
	}
	STAT(if (dup) { stats.rx_dup++; } else {
		stats.rx_last_ms = k_uptime_get();
		stats.rx_ok++;
		stats.rx_bytes += bytes;
		stats.rx_lost += lost;
	});
	return !dup;
}

/*
 * A pair is decoded on both CPUs: once the first frame's header is known,
 * the other CPU finishes its payload while this one decodes the second
 * frame (whose position follows from the first frame's length).
 */
static struct {
	struct qam_rx *ctx;
	struct qam_hdr hdr;
	struct qam_rx_info info;
	int ret;
	uint32_t us;
	/* Or the second half of an OFDM payload (ofdm set), or of its RS units. */
	bool ofdm, fec;
	const uint32_t *words;
	enum ofdm_mod mod;
	float err;
	struct ofdm_fhdr fh;
	unsigned int u0;
	int corr;
} rx2;
static K_SEM_DEFINE(rx2_req, 0, 1);
static K_SEM_DEFINE(rx2_done, 0, 1);
K_THREAD_STACK_DEFINE(rx2_stack, RX2_STACK_SIZE);
static struct k_thread rx2_thread;

static void rx2_loop(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		uint32_t t0;

		(void)k_sem_take(&rx2_req, K_FOREVER);
		t0 = now_cyc();
		if (rx2.ofdm) {
			struct ofdm_ctx *o = link_ofdm();
			size_t off = (size_t)o->mid * o->cfg.channels * ofdm_mod_bits(rx2.mod) / 8U;

			if (rx2.fec) {
				/* The second half of the RS units. */
				rx2.ret = ofdm_frame_fec_units(o, &rx2.fh, &ofdm_rxw, rx2.u0, ~0U,
							       &rx2.corr);
				k_sem_give(&rx2_done);
				continue;
			}
			rx2.err = 0.0f;
			ofdm_rx_fork(o, &ofdm_wk2, rx2.words);
			ofdm_rx_part(o, &ofdm_wk2, rx2.words, rx2.mod, o->mid, o->ndata,
				     ofdm_rxw.bytes + off, &rx2.err);
			rx2.us = cyc_us(now_cyc() - t0);
			k_sem_give(&rx2_done);
			continue;
		}
		rx2.ret = qam_rx_finish(rx2.ctx, &rx2.hdr, &rx2.info);
		rx2.us = cyc_us(now_cyc() - t0);
		k_sem_give(&rx2_done);
	}
}

static void count_decode(int ret, const struct qam_rx_info *info, uint32_t us, uint32_t engine_us)
{
	STAT({
		stats.rx_triggers++;
		stats.decode_us_sum += us;
		stats.decode_n++;
		stats.decode_us_max = MAX(stats.decode_us_max, us);
		stats.window_engine_us = engine_us;
		for (int i = 0; i < ARRAY_SIZE(info->prof); i++) {
			stats.prof_us[i] += cyc_us(info->prof[i]);
		}
		if (ret == 0 || ret == QAM_RX_DATA) {
			stats.mer_cdb_sum += (int64_t)(info->mer_db * 100.0f);
			stats.cfo_hz_sum += (int64_t)info->cfo_hz;
			stats.mer_n++;
			stats.corrected += (uint32_t)info->corrected;
		}
		if (ret == QAM_RX_NONE) {
			stats.rx_none++;
		} else if (ret == QAM_RX_HEADER) {
			stats.rx_hdr_err++;
		} else if (ret == QAM_RX_DATA) {
			stats.rx_data_err++;
		}
	});
}

/* An ACK for what is in flight. */
static void on_ack(const struct qam_hdr *h)
{
	unsigned int done = 0;

	if (h->dst != cfg.addr || !out.wait_ack) {
		return;
	}
	for (unsigned int i = 0; i < out.n; i++) {
		if (out.f[i].hdr.seq == h->seq && out.f[i].hdr.dst == h->src) {
			out.f[i].acked = true;
			if (out.n == 2 && (h->mod & ACK_OTHER)) {
				out.f[i ^ 1U].acked = true;
			}
		}
	}
	STAT(stats.acks_rx++);
	for (unsigned int i = 0; i < out.n; i++) {
		done += out.f[i].acked;
	}
	if (done == 0) {
		return;
	}
	out.wait_ack = false;
	STAT(for (unsigned int i = 0; i < out.n; i++) {
		if (out.f[i].acked) {
			stats.tx_acked++;
			stats.tx_bytes_acked += frame_bytes(&out.f[i].hdr);
		}
	});
	if (done == out.n) {
		out.pending = false;
	} else {
		/* The rest stays pending: retry_or_drop() pairs it anew. */
		out.partial = true;
	}
}

static void dump_hand_out(const struct esp_sdr_rx_burst *b)
{
	dump_words = b->words;
	dump_n = b->count;
	k_sem_give(&dump_ready);
	(void)k_sem_take(&dump_done, K_SECONDS(30));
}

/*
 * OFDM data frames: a window of two frames at the OFDM receive rate, the
 * header, the payload if the frame is for us, and the ACK (a QAM frame).
 */
static void receive_ofdm(void)
{
	struct ofdm_ctx *o = link_ofdm();
	struct esp_sdr_rx_burst b;
	struct ofdm_fhdr fh;
	struct ofdm_rx_info oi;
	uint32_t t0, tf = 0;
	int ret, corr = 0;

	if (esp_sdr_rx_capture(link_ofdm_rx_rate(), MIN(WINDOW_SAMPLES, 2U * o->rlen + o->rnfft),
			       &b) != 0) {
		return;
	}
	if (atomic_cas(&dump_req, 1, 0)) {
		dump_hand_out(&b);
	}
	t0 = now_cyc();
	ret = ofdm_frame_begin(o, b.words, b.count, &fh, &oi);
	if (ret == 0 && fh.type == MAC_DATA &&
	    (fh.dst == cfg.addr || fh.dst == MAC_ADDR_BROADCAST)) {
		uint32_t t1 = now_cyc(), t2;
		float err = 0.0f;

		/* Payload on both CPUs: the second half (from the mid pilot) on the other. */
		if (o->mid < o->ndata && arch_num_cpus() > 1) {
			rx2.ofdm = true;
			rx2.fec = false;
			rx2.words = b.words;
			rx2.mod = fh.mod;
			k_sem_give(&rx2_req);
			ofdm_rx_part(o, &o->w0, b.words, fh.mod, 0, o->mid, ofdm_rxw.bytes, &err);
			(void)k_sem_take(&rx2_done, K_FOREVER);
			err += rx2.err;
			oi.mer_db = ofdm_rx_mer(o, err);
		} else {
			ofdm_rx_finish(o, b.words, fh.mod, ofdm_rxw.bytes, &oi);
		}
		t2 = now_cyc();
		oi.prof[2] = t2 - t1;
		if (arch_num_cpus() > 1) {
			/* RS units split between the CPUs, then the CRC. */
			unsigned int nu = ofdm_frame_units(o, fh.mod);

			rx2.ofdm = true;
			rx2.fec = true;
			rx2.fh = fh;
			rx2.u0 = nu / 2U;
			k_sem_give(&rx2_req);
			ret = ofdm_frame_fec_units(o, &fh, &ofdm_rxw, 0, nu / 2U, &corr);
			(void)k_sem_take(&rx2_done, K_FOREVER);
			corr += rx2.corr;
			if (ret == 0 && rx2.ret == 0) {
				ret = ofdm_frame_check(o, &fh, &ofdm_rxw, ofdm_rx_data);
			} else {
				ret = -3;
			}
		} else {
			ret = ofdm_frame_fec(o, &fh, &ofdm_rxw, ofdm_rx_data, &corr);
		}
		tf = now_cyc() - t2;
	} else if (ret == 0) {
		STAT(stats.rx_other++);
		ret = 1;
	}
	STAT({
		uint32_t us = cyc_us(now_cyc() - t0);

		stats.rx_triggers++;
		stats.decode_us_sum += us;
		stats.decode_n++;
		stats.decode_us_max = MAX(stats.decode_us_max, us);
		stats.window_engine_us = b.elapsed_us;
		stats.prof_us[0] += cyc_us(oi.prof[0]);
		stats.prof_us[1] += cyc_us(oi.prof[1]);
		stats.prof_us[4] += cyc_us(oi.prof[2]);
		stats.prof_us[5] += cyc_us(tf);
		if (ret == 0 || ret == -3) {
			stats.mer_cdb_sum += (int64_t)(oi.mer_db * 100.0f);
			stats.cfo_hz_sum += (int64_t)oi.cfo_hz;
			stats.mer_n++;
			stats.corrected += (uint32_t)corr;
		}
		if (ret == -1) {
			stats.rx_none++;
		} else if (ret == -2) {
			stats.rx_hdr_err++;
		} else if (ret == -3) {
			stats.rx_data_err++;
		}
	});
	if (ret != 0) {
		return;
	}
	if (atomic_cas(&dump_req, 2, 0)) {
		dump_hand_out(&b);
	}
	{
		struct qam_hdr h = {.type = fh.type, .mod = (uint8_t)qam_mod_of(fh.mod),
				    .dst = fh.dst, .src = fh.src, .seq = fh.seq};

		(void)on_data(&h);
		if (h.dst == cfg.addr) {
			send_ack(h.src, h.seq, false);
		}
	}
}

/* An OFDM ACK: a short window, the header only. */
static void receive_ofdm_ack(void)
{
	struct ofdm_ctx *o = link_ofdm();
	struct esp_sdr_rx_burst b;
	struct ofdm_fhdr fh;
	struct ofdm_rx_info oi;
	uint32_t t0;
	int ret;

	if (esp_sdr_rx_capture(link_ofdm_rx_rate(),
			       MIN(WINDOW_SAMPLES, 2U * ofdm_hdr_rsamples(o) + o->rnfft), &b) != 0) {
		return;
	}
	t0 = now_cyc();
	ret = ofdm_frame_begin_hdr(o, b.words, b.count, &fh, &oi);
	STAT({
		uint32_t us = cyc_us(now_cyc() - t0);

		stats.rx_triggers++;
		stats.decode_us_sum += us;
		stats.decode_n++;
		stats.decode_us_max = MAX(stats.decode_us_max, us);
		if (ret == -1) {
			stats.rx_none++;
		} else if (ret == -2) {
			stats.rx_hdr_err++;
		}
	});
	if (ret == 0 && fh.type == MAC_ACK) {
		struct qam_hdr h = {.type = MAC_ACK, .dst = fh.dst, .src = fh.src, .seq = fh.seq};

		on_ack(&h);
	}
}

/* Capture a full window and act on what it holds. */
static void receive(void)
{
	struct esp_sdr_rx_burst b;
	struct qam_hdr h;
	struct qam_rx_info info;
	bool pair = false, ok_a = false, ok_b = false;
	uint32_t t0;
	int ret;

	/* OFDM: data frames, or the ACK a sender waits for. */
	if (cfg.phy == MAC_PHY_OFDM) {
		if (out.wait_ack) {
			receive_ofdm_ack();
		} else {
			receive_ofdm();
		}
		return;
	}

	/* An ACK is short: a small window holds a whole copy and decodes faster. */
	if (esp_sdr_rx_capture(RADIO_RATE, out.wait_ack ? ACK_WINDOW_SAMPLES : WINDOW_SAMPLES,
			       &b) != 0) {
		return;
	}
	if (atomic_cas(&dump_req, 1, 0)) {
		dump_hand_out(&b);
	}
	t0 = now_cyc();
	ret = qam_rx_begin(&qam_rx_ctx, b.words, b.count, &h, &info);
	if (ret == 0 && h.pair && h.type == MAC_DATA) {
		/* The other frame starts one frame length away, within the loop. */
		int lf = (int)qam_hdr_frame_samples(&h);
		int other = info.start + lf;
		uint32_t ta;

		if (other + QAM_SEARCH_MARGIN > (int)b.count) {
			other -= 2 * lf;
		}
		if (other < 0) {
			other = info.start - lf;
		}
		pair = true;
		/* The first frame's payload on the other CPU; its context stays with it. */
		rx2.ofdm = false;
		rx2.ctx = &qam_rx_ctx;
		rx2.hdr = h;
		rx2.info = info;
		k_sem_give(&rx2_req);
		ret = qam_rx_begin_at(&qam_rx2_ctx, b.words, b.count, other, info.cfo_hz, &h, &info);
		if (ret == 0) {
			ret = qam_rx_finish(&qam_rx2_ctx, &h, &info);
		}
		ta = cyc_us(now_cyc() - t0);
		(void)k_sem_take(&rx2_done, K_FOREVER);
		count_decode(ret, &info, ta, b.elapsed_us);
		count_decode(rx2.ret, &rx2.info, rx2.us, b.elapsed_us);
		STAT(stats.pair_a_us_sum += ta; stats.pair_b_us_sum += rx2.us;
		     stats.pair_us_sum += cyc_us(now_cyc() - t0); stats.pair_n++);
	} else {
		if (ret == 0) {
			ret = qam_rx_finish(&qam_rx_ctx, &h, &info);
		}
		count_decode(ret, &info, cyc_us(now_cyc() - t0), b.elapsed_us);
	}
	/* "dump ok": a window that decoded, before an ACK reuses the bank. */
	if (ret == 0 && h.type == MAC_DATA && atomic_cas(&dump_req, 2, 0)) {
		dump_hand_out(&b);
	}
	if (ret == 0 && h.type == MAC_ACK) {
		on_ack(&h);
		return;
	}
	/* Data for us (or broadcast) from both decoders; one ACK covers them. */
	ok_a = ret == 0 && h.type == MAC_DATA &&
	       (h.dst == cfg.addr || h.dst == MAC_ADDR_BROADCAST);
	ok_b = pair && rx2.ret == 0 && rx2.hdr.type == MAC_DATA &&
	       (ret != 0 || rx2.hdr.src == h.src) &&
	       (rx2.hdr.dst == cfg.addr || rx2.hdr.dst == MAC_ADDR_BROADCAST);
	if ((ret == 0 && !ok_a && h.type == MAC_DATA) ||
	    (pair && rx2.ret == 0 && !ok_b && rx2.hdr.type == MAC_DATA)) {
		STAT(stats.rx_other++);
	}
	if (ok_a) {
		(void)on_data(&h);
	}
	if (ok_b) {
		(void)on_data(&rx2.hdr);
	}
	if (ok_a && h.dst == cfg.addr) {
		send_ack(h.src, h.seq, ok_b);
	} else if (ok_b && rx2.hdr.dst == cfg.addr) {
		send_ack(rx2.hdr.src, rx2.hdr.seq, false);
	}
}

static uint32_t xorshift(uint32_t *s)
{
	uint32_t x = *s;

	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*s = x;
	return x;
}

/*
 * Frames are built on the other CPU while the MAC waits for an ACK: the
 * data bank is free once its frames have gone out. Retries are rare, so
 * frames that have to go out again are rebuilt (their payload follows from
 * the sequence number).
 */
static struct {
	unsigned int n;
	struct qam_hdr hdr[WIN_MAX];
	size_t len;
	bool busy;
	bool ready;
} job;
static K_SEM_DEFINE(build_req, 0, 1);
static K_SEM_DEFINE(build_done, 0, 1);
K_THREAD_STACK_DEFINE(build_stack, BUILD_STACK_SIZE);
static struct k_thread build_thread;

/* Test data: whitened, a function of the sequence number. */
static void fill_payload(uint8_t *p, uint16_t seq, size_t n)
{
	uint32_t s = 0x9e3779b9U ^ seq;

	for (size_t i = 0; i < n; i += 4) {
		uint32_t v = xorshift(&s);

		memcpy(&p[i], &v, MIN(4U, n - i));
	}
}

static size_t build(const struct qam_hdr *h, unsigned int n)
{
	size_t words;
	uint32_t *bank = esp_sdr_tx_loop_buf(DATA_BANK, &words);

	if (cfg.phy == MAC_PHY_OFDM) {
		struct ofdm_fhdr fh = {.type = h[0].type, .mod = ofdm_mod_of(h[0].mod),
				       .dst = h[0].dst, .src = h[0].src, .seq = h[0].seq};

		fill_payload(payload[0], h[0].seq, frame_bytes(&h[0]));
		return ofdm_frame_build(link_ofdm(), &ofdm_tb, &ofdm_txw, &fh, payload[0], bank);
	}

	if (qam_tx_ctx.mod != (int)h[0].mod || qam_tx_ctx.amp != cfg.amp) {
		qam_tx_init(&qam_tx_ctx, (enum qam_mod)h[0].mod, cfg.amp);
	}
	for (unsigned int i = 0; i < n; i++) {
		fill_payload(payload[i], h[i].seq, frame_bytes(&h[i]));
	}
	return n == 2 ? qam_tx_build_pair(&qam_tx_ctx, &h[0], payload[0], &h[1], payload[1], bank)
		      : qam_tx_build(&qam_tx_ctx, &h[0], payload[0], bank);
}

static void build_loop(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		uint32_t t0;

		(void)k_sem_take(&build_req, K_FOREVER);
		t0 = now_cyc();
		job.len = build(job.hdr, job.n);
		STAT(stats.build_us_sum += cyc_us(now_cyc() - t0); stats.build_n++);
		k_sem_give(&build_done);
	}
}

/* Header of a new test frame. */
static struct qam_hdr new_hdr(bool pair)
{
	return (struct qam_hdr){
		.type = MAC_DATA,
		.mod = (uint8_t)client.mod,
		.fec = (uint8_t)client.fec,
		.full = client.ncw == 0,
		.ncw = (uint8_t)client.ncw,
		.pair = pair,
		.dst = client.dst,
		.src = cfg.addr,
		.seq = next_seq++,
	};
}

/* Start building the next new frames in the background. */
static void build_next(void)
{
	job.n = client.window;
	for (unsigned int i = 0; i < job.n; i++) {
		job.hdr[i] = new_hdr(job.n == 2);
	}
	job.busy = true;
	job.ready = false;
	k_sem_give(&build_req);
}

/* Wait for a background build; true if one was running. */
static bool build_wait(void)
{
	if (!job.busy) {
		return false;
	}
	(void)k_sem_take(&build_done, K_FOREVER);
	job.busy = false;
	job.ready = job.len > 0;
	return true;
}

/* Poll a background build. */
static void build_poll(void)
{
	if (job.busy && k_sem_take(&build_done, K_NO_WAIT) == 0) {
		job.busy = false;
		job.ready = job.len > 0;
	}
}

/* Drop a built or running background job; its new sequence numbers are reused. */
static void build_discard(void)
{
	(void)build_wait();
	if (job.ready) {
		next_seq = job.hdr[0].seq;
		job.ready = false;
	}
}

/* The built frames become the ones in flight. */
static void take_frames(void)
{
	out.n = job.n;
	for (unsigned int i = 0; i < job.n; i++) {
		out.f[i] = (struct txf){.hdr = job.hdr[i]};
	}
	out.len = job.len;
	out.built = true;
	out.partial = false;
	out.pending = true;
	out.wait_ack = false;
	out.cw = cfg.cw_min;
	out.backoff = -1;
	out.quiet = 0;
	job.ready = false;
	STAT(stats.tx_frames += job.n);
}

/* Put what is in flight back into the data bank, now (rare: retries). */
static void rebuild(void)
{
	struct qam_hdr h[WIN_MAX];
	uint32_t t0;

	build_discard();
	for (unsigned int i = 0; i < out.n; i++) {
		h[i] = out.f[i].hdr;
	}
	t0 = now_cyc();
	out.len = build(h, out.n);
	out.built = out.len > 0;
	STAT(stats.build_us_sum += cyc_us(now_cyc() - t0); stats.build_n++);
	if (!out.built) {
		out.pending = false;
	}
}

/* Charge the rate limiter for @p h's bytes. */
static void rate_charge(const struct qam_hdr *h)
{
	if (client.rate_kbps != 0) {
		out.next_due_us += (int64_t)frame_bytes(h) * 8000 / client.rate_kbps;
	}
}

/*
 * No ACK in time (@p timeout), or a pair half ACKed. The frames still
 * missing go out again, each up to `retries` times without an ACK; a pair
 * with one frame left takes a new frame along while the client runs, else
 * the frame goes alone (its pair header still decodes: the loop period is a
 * divisor of the pair's).
 */
static void retry_or_drop(bool timeout)
{
	unsigned int left = 0;
	struct txf keep[WIN_MAX];

	out.wait_ack = false;
	for (unsigned int i = 0; i < out.n; i++) {
		if (out.f[i].acked) {
			continue;
		}
		if (timeout && ++out.f[i].tries > cfg.retries) {
			STAT(stats.tx_dropped++);
			continue;
		}
		keep[left++] = out.f[i];
	}
	if (left == 0) {
		out.pending = false;
		return;
	}
	if (left < out.n) {
		build_discard();
		out.f[0] = keep[0];
		if (out.n == 2 && client.active) {
			out.f[1] = (struct txf){.hdr = new_hdr(true)};
			rate_charge(&out.f[1].hdr);
			STAT(stats.tx_frames++);
		} else {
			out.n = 1;
		}
		out.built = false;
	}
	for (unsigned int i = 0; i < out.n; i++) {
		out.f[i].acked = false;
	}
	/* Only a missing ACK means contention; a partial ACK was a success. */
	out.cw = timeout ? MIN(2U * out.cw + 1U, (unsigned int)cfg.cw_max) : cfg.cw_min;
	out.backoff = -1;
	out.quiet = 0;
}

static void mac_loop(void *p1, void *p2, void *p3)
{
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	rs_init();
	ham_init();
	qam_clock = now_cyc;
	qam_rx_ctx.cw = rx_cw[0];
	qam_rx2_ctx.cw = rx_cw[1];
	ret = esp_sdr_init();
	if (ret != 0) {
		LOG_ERR("radio init failed (%d)", ret);
		return;
	}
	qam_tx_init(&qam_tx_ctx, QAM_QPSK, cfg.amp);
	apply_cfg();
	atomic_set(&mac_running, 1);

	for (;;) {
		float p;

		if (atomic_get(&paused)) {
			struct paused_job *job;

			/* Hand the data bank and the TX context over: drop builds, rebuild later. */
			if (!pause_acked) {
				build_discard();
				out.built = false;
				pause_acked = true;
				k_sem_give(&pause_ack);
			}
			/* Taken before it runs: link_mac_run_paused() can only withdraw a waiting one. */
			job = atomic_ptr_set(&paused_job, NULL);
			if (job != NULL) {
				job->fn(job->arg);
				k_sem_give(&paused_done);
				continue;
			}
			k_sleep(K_MSEC(10));
			continue;
		}
		pause_acked = false;
		if (atomic_cas(&clear_req, 1, 0)) {
			memset(srcs, 0, sizeof(srcs));
		}
		if (atomic_cas(&cfg_dirty, 1, 0)) {
			k_mutex_lock(&cfg_lock, K_FOREVER);
			apply_cfg();
			k_mutex_unlock(&cfg_lock);
		}
		if (atomic_cas(&cal_req, 1, 0)) {
			calibrate();
			k_sem_give(&cal_done);
		}
		if (atomic_cas(&client_dirty, 1, 0)) {
			client_end_ms = 0;
			/* Unsent frames: reuse their sequence numbers, the server sees no gap. */
			build_discard();
			client = client_req;
			out.pending = false;
			out.wait_ack = false;
			out.next_due_us = k_ticks_to_us_floor64(k_uptime_ticks());
		}
		if (client.active && client.end_ms != 0 && k_uptime_get() >= client.end_ms) {
			client.active = false;
		}
		if (!client.active && !out.pending && client_end_ms == 0) {
			client_end_ms = k_uptime_get();
		}

		build_poll();
		if (!out.pending && client.active) {
			int64_t now = k_ticks_to_us_floor64(k_uptime_ticks());

			if (!job.busy && !job.ready) {
				build_next();
			}
			if (job.ready && (client.rate_kbps == 0 || now >= out.next_due_us)) {
				take_frames();
				out.next_due_us = MAX(out.next_due_us, now - 100000);
				for (unsigned int i = 0; i < out.n; i++) {
					rate_charge(&out.f[i].hdr);
				}
			}
		}

		p = sense();
		if (p < 0.0f) {
			k_sleep(K_MSEC(10));
			continue;
		}
		if (p > cca_threshold) {
			STAT(stats.cca_busy++);
			out.quiet = 0;
			receive();
			/* A busy channel must not hold off the ACK timeout forever. */
			if (out.wait_ack && (int32_t)(now_cyc() - out.ack_deadline) > 0) {
				retry_or_drop(true);
				continue;
			}
			if (out.partial) {
				out.partial = false;
				retry_or_drop(false);
			}
			continue;
		}

		if (out.wait_ack) {
			if ((int32_t)(now_cyc() - out.ack_deadline) > 0) {
				retry_or_drop(true);
			}
			continue;
		}
		if (!out.pending) {
			/*
			 * Idle: one tick (100 us) between sense slots leaves the CPU
			 * mostly free; the airtime covers the longer gap.
			 */
			k_usleep(1);
			continue;
		}
		if (out.quiet < cfg.difs) {
			out.quiet++;
			continue;
		}
		if (out.backoff < 0) {
			out.backoff = (int)(sys_rand32_get() % (out.cw + 1U));
		}
		if (out.backoff > 0) {
			out.backoff--;
			STAT(stats.backoff_slots++);
			continue;
		}
		if (!out.built) {
			rebuild();
			if (!out.pending) {
				continue;
			}
		}
		transmit(DATA_BANK, out.len, cfg.data_air_us,
			 cfg.phy == MAC_PHY_OFDM ? link_ofdm_tx_rate() : RADIO_RATE);
		STAT(stats.tx_attempts++);
		if (out.f[0].hdr.dst == MAC_ADDR_BROADCAST) {
			out.pending = false;
			continue;
		}
		/* The bank is free until a retry: build the next frames meanwhile. */
		if (client.active && !job.busy && !job.ready) {
			build_next();
			out.built = false;
		}
		out.wait_ack = true;
		out.ack_deadline = now_cyc() + k_us_to_cyc_ceil32(cfg.ack_timeout_us);
	}
}

int link_mac_init(void)
{
	uint8_t id[8];
	ssize_t n = 0;

#if defined(CONFIG_HWINFO)
	n = hwinfo_get_device_id(id, sizeof(id));
#endif
	/* PSRAM .bss is not zeroed at boot. */
	memset(srcs, 0, sizeof(srcs));
	memset(&ack_tx_ctx, 0, sizeof(ack_tx_ctx));
	cfg.addr = n > 0 ? id[n - 1] : 1U;
	if (cfg.addr == MAC_ADDR_BROADCAST) {
		cfg.addr = 0xfe;
	}
	next_seq = (uint16_t)sys_rand32_get();

	k_thread_create(&mac_thread, mac_stack, K_THREAD_STACK_SIZEOF(mac_stack), mac_loop, NULL,
			NULL, NULL, K_PRIO_PREEMPT(CONFIG_APP_MAC_PRIORITY), K_FP_REGS, K_FOREVER);
	k_thread_name_set(&mac_thread, "link_mac");
#if defined(CONFIG_SCHED_CPU_MASK)
	if (arch_num_cpus() > 1) {
		k_thread_cpu_pin(&mac_thread, CONFIG_APP_RADIO_CPU);
	}
#endif
	k_thread_create(&build_thread, build_stack, K_THREAD_STACK_SIZEOF(build_stack), build_loop,
			NULL, NULL, NULL, K_PRIO_PREEMPT(CONFIG_APP_BUILD_PRIORITY), K_FP_REGS,
			K_FOREVER);
	k_thread_name_set(&build_thread, "link_build");
#if defined(CONFIG_SCHED_CPU_MASK)
	if (arch_num_cpus() > 1 && CONFIG_APP_NET_CPU >= 0) {
		k_thread_cpu_pin(&build_thread, CONFIG_APP_NET_CPU);
	}
#endif
	k_thread_create(&rx2_thread, rx2_stack, K_THREAD_STACK_SIZEOF(rx2_stack), rx2_loop, NULL,
			NULL, NULL, K_PRIO_PREEMPT(CONFIG_APP_RX2_PRIORITY), K_FP_REGS, K_FOREVER);
	k_thread_name_set(&rx2_thread, "link_rx2");
#if defined(CONFIG_SCHED_CPU_MASK)
	if (arch_num_cpus() > 1 && CONFIG_APP_NET_CPU >= 0) {
		k_thread_cpu_pin(&rx2_thread, CONFIG_APP_NET_CPU);
	}
#endif
	k_thread_start(&mac_thread);
	k_thread_start(&build_thread);
	k_thread_start(&rx2_thread);
	return 0;
}

void link_mac_get_cfg(struct mac_cfg *c)
{
	k_mutex_lock(&cfg_lock, K_FOREVER);
	*c = cfg;
	k_mutex_unlock(&cfg_lock);
}

int link_mac_set_cfg(const struct mac_cfg *c)
{
	if (c->cw_min == 0 || c->cw_max < c->cw_min || c->amp < 1 || c->amp > 300 ||
	    c->turn_us > 3000 ||
	    c->data_air_us > 100000 || c->ack_air_us > 100000) {
		return -EINVAL;
	}
	k_mutex_lock(&cfg_lock, K_FOREVER);
	cfg = *c;
	k_mutex_unlock(&cfg_lock);
	atomic_set(&cfg_dirty, 1);
	return 0;
}

float link_mac_noise_floor(void)
{
	return noise_floor;
}

int link_mac_calibrate(void)
{
	k_sem_reset(&cal_done);
	atomic_set(&cal_req, 1);
	return k_sem_take(&cal_done, K_SECONDS(5));
}

int link_mac_client_start(uint8_t dst, enum qam_mod mod, enum qam_fec fec, unsigned int ncw,
			  unsigned int window, uint32_t duration_ms, uint32_t rate_kbps)
{
	struct qam_hdr h = {.mod = mod, .fec = fec, .ncw = ncw, .full = ncw == 0,
			    .pair = window == 2};

	if (cfg.phy == MAC_PHY_OFDM) {
		/* One frame per transmission, the layout from "ofdm set"; -n and -f do not apply. */
		size_t bytes = link_mac_frame_bytes(mod);

		if (bytes == 0U || bytes > sizeof(payload[0])) {
			return -EINVAL;
		}
		window = 1;
		fec = QAM_FEC_RS;
		ncw = 0;
	} else if (window < 1 || window > WIN_MAX || qam_hdr_payload_bytes(&h) == 0) {
		return -EINVAL;
	}
	/* ncw 0: full frames (as many units as fit, the last one short). */
	if (cfg.phy != MAC_PHY_OFDM &&
	    (mod >= QAM_MODS || fec >= QAM_FECS || ncw > qam_ncw_max(mod, fec) ||
	     (ncw == 0 && qam_full_payload_bytes(mod, fec) == 0))) {
		return -EINVAL;
	}
	client_req = (typeof(client_req)){
		.active = true,
		.dst = dst,
		.mod = mod,
		.fec = fec,
		.ncw = ncw,
		.window = window,
		.end_ms = duration_ms ? k_uptime_get() + duration_ms : 0,
		.rate_kbps = rate_kbps,
	};
	atomic_set(&client_dirty, 1);
	return 0;
}

void link_mac_client_stop(void)
{
	client_req.active = false;
	atomic_set(&client_dirty, 1);
}

bool link_mac_client_active(void)
{
	return atomic_get(&client_dirty) ? client_req.active : client.active || out.pending;
}

void link_mac_get_stats(struct mac_stats *st)
{
	k_spinlock_key_t key = k_spin_lock(&stats_lock);

	*st = stats;
	k_spin_unlock(&stats_lock, key);
}

int link_mac_dump_take(const uint32_t **words, size_t *n, bool decoded, k_timeout_t timeout)
{
	atomic_val_t req = decoded ? 2 : 1;

	k_sem_reset(&dump_ready);
	/* A release after an earlier timeout must not count for this dump. */
	k_sem_reset(&dump_done);
	atomic_set(&dump_req, req);
	if (k_sem_take(&dump_ready, timeout) != 0) {
		/* Too late to withdraw if the MAC thread just took it. */
		if (!atomic_cas(&dump_req, req, 0)) {
			(void)k_sem_take(&dump_ready, K_FOREVER);
		} else {
			return -EAGAIN;
		}
	}
	*words = dump_words;
	*n = dump_n;
	return 0;
}

void link_mac_dump_release(void)
{
	k_sem_give(&dump_done);
}

void link_mac_pause(bool pause)
{
	if (pause) {
		k_sem_reset(&pause_ack);
		atomic_set(&paused, 1);
		/* The MAC thread acknowledges between slots, after dropping its builds. */
		if (atomic_get(&mac_running) != 0) {
			(void)k_sem_take(&pause_ack, K_SECONDS(2));
		}
	} else {
		atomic_set(&paused, 0);
	}
}

int link_mac_run_paused(void (*fn)(void *arg), void *arg)
{
	struct paused_job job = {.fn = fn, .arg = arg};

	if (atomic_get(&mac_running) == 0) {
		return -ENODEV;
	}
	k_sem_reset(&paused_done);
	(void)atomic_ptr_set(&paused_job, &job);
	if (k_sem_take(&paused_done, K_SECONDS(10)) != 0) {
		if (atomic_ptr_cas(&paused_job, &job, NULL)) {
			return -ETIMEDOUT;
		}
		/* Started meanwhile. */
		(void)k_sem_take(&paused_done, K_FOREVER);
	}
	return 0;
}

void link_mac_clear_stats(void)
{
	k_spinlock_key_t key = k_spin_lock(&stats_lock);

	memset(&stats, 0, sizeof(stats));
	k_spin_unlock(&stats_lock, key);
	/* The duplicate windows too, by the MAC thread. */
	atomic_set(&clear_req, 1);
}

struct bench_decode {
	enum qam_mod mod;
	struct qam_rx_info *info;
	uint32_t build_us, decode_us;
	int ret;
};

/* On the paused MAC thread (link_mac_run_paused()), with the decoder's stack. */
static void bench_decode(void *arg)
{
	struct bench_decode *job = arg;
	size_t words;
	uint32_t *tx = esp_sdr_tx_loop_buf(DATA_BANK, &words);
	uint32_t *rx = esp_sdr_tx_loop_buf(ACK_BANK, &words);
	struct qam_hdr h = {.type = MAC_DATA, .mod = (uint8_t)job->mod, .fec = QAM_FEC_RS,
			    .ncw = (uint8_t)qam_ncw_max(job->mod, QAM_FEC_RS),
			    .dst = 1, .src = 2, .seq = 7};
	size_t len, n = WINDOW_SAMPLES;
	uint32_t t0;

	for (size_t i = 0; i < qam_payload_bytes(QAM_FEC_RS, h.ncw); i++) {
		payload[0][i] = (uint8_t)(i * 37U + 11U);
	}
	qam_tx_init(&qam_tx_ctx, job->mod, cfg.amp);
	t0 = now_cyc();
	len = qam_tx_build(&qam_tx_ctx, &h, payload[0], tx);
	job->build_us = cyc_us(now_cyc() - t0);
	if (len == 0) {
		job->ret = -EINVAL;
		return;
	}
	/* Loop it into a capture window in receive word order (Q low, I high). */
	for (size_t i = 0; i < n; i++) {
		uint32_t w = tx[(i + 1000) % len];

		rx[i] = ((w >> 10) & 0x3ffU) | ((w & 0x3ffU) << 10);
	}
	qam_rx_ctx.carrier_hz = 0.0f;
	t0 = now_cyc();
	job->ret = qam_rx_decode(&qam_rx_ctx, rx, n, &h, job->info);
	job->decode_us = cyc_us(now_cyc() - t0);
	qam_rx_ctx.carrier_hz = (float)cfg.freq_mhz * 1e6f;
	qam_tx_init(&qam_tx_ctx, QAM_QPSK, cfg.amp);
}

int link_mac_bench_decode(enum qam_mod mod, struct qam_rx_info *info, uint32_t *build_us,
			  uint32_t *decode_us)
{
	struct bench_decode job = {.mod = mod, .info = info};
	int ret = link_mac_run_paused(bench_decode, &job);

	if (ret != 0) {
		return ret;
	}
	*build_us = job.build_us;
	*decode_us = job.decode_us;
	return job.ret;
}

int64_t link_mac_client_end_ms(void)
{
	return client_end_ms != 0 ? client_end_ms : k_uptime_get();
}
