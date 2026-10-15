/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * IEEE 802.15.4 radio driver on the esp_sdr receiver and DAC, with the
 * software O-QPSK PHY (ieee154_phy.c).
 *
 * The LO sits LO_OFFSET_MHZ below the channel, so its leakage and the
 * receiver's DC offset stay out of the band.
 * RX: the 16 MS/s ring, mixed down by fs/4 and decimated by 4 to the PHY's
 * 4 MS/s (CONFIG_ESP_SDR_IEEE802154_RING, gapless), or back-to-back bank
 * captures of about 1 ms each, where a frame is only received if it lies
 * inside one capture.
 * TX: the frame is synthesized at the 40 MS/s DAC rate from half-sine
 * templates and played gapless by the DAC backend.
 *
 * Frames that ask for an ACK wait for it (one capture after the frame);
 * frames to this node that ask for one get it from inside the ring run
 * (CONFIG_ESP_SDR_IEEE802154_AUTO_ACK). CCA and energy detection are not done.
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <esp_attr.h>

#include <esp_sdr/esp_sdr.h>
#if defined(CONFIG_ESP_SDR_IEEE802154_RING)
#include <esp_sdr/esp_sdr_ring.h>
#endif
#include <esp_sdr/ieee154_esp_sdr.h>
#include <esp_sdr/ieee154_phy.h>

#include "ieee154_dsp.h"

LOG_MODULE_REGISTER(esp_sdr_154, CONFIG_ESP_SDR_IEEE802154_LOG_LEVEL);

#define LO_OFFSET_MHZ IEEE154_DSP_LO_OFFSET_MHZ
#define TX_AMP        300.0f

/*
 * Conjugate the received signal after the mix (a peer with swapped I/Q
 * mirrors its spectrum about the channel), for checking its convention.
 */
static int32_t rx_qsign = 1;

/* One bank capture: burst receive, and the ACK window after a transmission. */
#define RX_RATE  ESP_SDR_RATE_16MSPS
#define RX_WORDS ESP_SDR_SAMPLES_MAX
#define RX_OUT   IEEE154_DSP_DECIM_OUT(RX_WORDS)

/*
 * Interleaved I/Q at 4 MS/s for the PHY. With the ring it doubles as the
 * sample ring: captures only run while the ring is stopped, and both go
 * through the PHY lock. Internal RAM: the ring's sink runs at 4 MS/s with
 * interrupts masked, and PSRAM stores cost it about 20 cycles per sample.
 */
#define RX_IQ_SAMPLES MAX(RX_OUT, 4096U)

#define RING_STACK_SIZE 1536
#define RX_STACK_SIZE   2048

/*
 * The driver's internal RAM: the high region, or what the application gives
 * it for the boot (CONFIG_ESP_SDR_IEEE802154_HIGH_RAM_APP).
 */
struct sdr154_high {
	int16_t rx_iq[2 * RX_IQ_SAMPLES];
	ieee154_dsp_tmpl_t tmpl;
	struct ieee154_dsp_tx tx;
#if defined(CONFIG_ESP_SDR_IEEE802154_RING)
	K_KERNEL_STACK_MEMBER(ring_stack, RING_STACK_SIZE);
#endif
	K_KERNEL_STACK_MEMBER(rx_stack, RX_STACK_SIZE);
};

#if defined(CONFIG_ESP_SDR_IEEE802154_HIGH_RAM_APP)
static struct sdr154_high *high;
#else
static struct sdr154_high high_ram ESP_SDR_HIGH_RAM;
static struct sdr154_high *const high = &high_ram;
#endif

__attribute__((optimize("O3"))) static void mix_decimate(const uint32_t *w, int16_t *out)
{
	(void)ieee154_dsp_mix_decimate(w, RX_WORDS, out, rx_qsign);
}

/* The PHY is one instance: receive and the ACK check take turns. */
static K_MUTEX_DEFINE(phy_lock);
/* Sequence number of the ACK awaited (-1: none), and whether it came. */
static int ack_seq = -1;
static bool ack_seen;

static bool tx_conj;

static void tmpl_init(void)
{
	ieee154_dsp_tmpl_init(&high->tmpl, TX_AMP, tx_conj);
	high->tx.tmpl = (const ieee154_dsp_tmpl_t *)&high->tmpl;
}

/* Runs in the DAC filler (interrupts locked with the vector interpolator): IRAM, tables in internal RAM. */
static IRAM_ATTR __attribute__((optimize("O3", "no-tree-loop-distribute-patterns"))) void tx_gen(void *user, uint32_t *dst,
							     uint64_t index, uint32_t n)
{
	ARG_UNUSED(user);
	ieee154_dsp_tx_gen(&high->tx, dst, index, n);
}

/* --------------------------------------------------------------- driver */

struct sdr154_data {
	struct net_if *iface;
	uint8_t mac[8];
	uint16_t channel;
	bool sdr_up;
	atomic_t running;
	struct k_sem run_sem;
	/* Serializes captures (the bank) and transmissions. */
	struct k_mutex radio_lock;
	/* Threads waiting for the radio; the ring stops for them (ring receive). */
	atomic_t waiters;
	struct ieee154_esp_sdr_stats stats;
};

static struct sdr154_data sdr154;

/* Address filter (filter(), configure()): what an ACK is due for. */
static struct {
	uint16_t pan_id, short_addr;
	uint8_t ext_addr[8];
	bool pan_coord, promisc;
} addr = {.pan_id = 0xffffU, .short_addr = 0xfffeU};

/*
 * Breadcrumbs in RTC memory, which a reset keeps: the last driver state
 * changes with their uptime, for resets that leave no other trace.
 */
#define TRAIL_LEN   64U
#define TRAIL_MAGIC 0x54524c31U /* "TRL1" */
static RTC_NOINIT_ATTR struct {
	uint32_t magic, next;
	uint32_t ev[TRAIL_LEN];
} trail;

/* Written from both CPUs (ring thread, tx, shell): irq_lock() is per CPU. */
static struct k_spinlock trail_lock;

/* The counters are updated from the ring, PHY, receive and transmit threads. */
static struct k_spinlock stats_lock;
#define STATS_UPDATE(...)                                                                          \
	do {                                                                                       \
		k_spinlock_key_t stats_key_ = k_spin_lock(&stats_lock);                            \
		__VA_ARGS__;                                                                       \
		k_spin_unlock(&stats_lock, stats_key_);                                            \
	} while (0)

static void crumb(enum ieee154_esp_sdr_event e, uint32_t arg)
{
	k_spinlock_key_t key = k_spin_lock(&trail_lock);

	if (trail.magic != TRAIL_MAGIC || trail.next >= TRAIL_LEN) {
		memset(&trail, 0, sizeof(trail));
		trail.magic = TRAIL_MAGIC;
	}
	/* Event 4 bits, cpu 1 bit, argument 7 bits, uptime in ms 20 bits (wraps). */
	trail.ev[trail.next] = ((uint32_t)e << 28) | ((uint32_t)arch_curr_cpu()->id << 27) |
			       ((arg & 0x7fU) << 20) | (k_uptime_get_32() & 0xfffffU);
	trail.next = (trail.next + 1U) % TRAIL_LEN;
	k_spin_unlock(&trail_lock, key);
}

size_t ieee154_esp_sdr_get_trail(uint32_t *out, size_t max)
{
	size_t n = 0;
	k_spinlock_key_t key = k_spin_lock(&trail_lock);

	if (trail.magic != TRAIL_MAGIC || trail.next >= TRAIL_LEN) {
		k_spin_unlock(&trail_lock, key);
		return 0;
	}
	for (uint32_t k = 0; k < TRAIL_LEN && n < max; k++) {
		uint32_t v = trail.ev[(trail.next + k) % TRAIL_LEN];

		if (v != 0U) {
			out[n++] = v;
		}
	}
	k_spin_unlock(&trail_lock, key);
	return n;
}

/*
 * Take the radio. With the ring it is held by the ring thread for a whole
 * run, and a run clears stop requests made before it started: ask until it
 * lets go.
 */
static void radio_take(struct sdr154_data *d)
{
	crumb(IEEE154_ESP_SDR_EV_TAKE, (uint32_t)atomic_get(&d->waiters));
#if defined(CONFIG_ESP_SDR_IEEE802154_RING)
	atomic_inc(&d->waiters);
	while (k_mutex_lock(&d->radio_lock, K_MSEC(1)) != 0) {
		esp_sdr_ring_stop();
	}
#else
	k_mutex_lock(&d->radio_lock, K_FOREVER);
#endif
}

/*
 * Like radio_take(), but gives up after @p ms: for callers that only need the
 * ring stopped. Another holder (a transmission, a diagnostic capture) means
 * no ring runs. Returns whether the radio is held.
 */
static bool radio_try_take(struct sdr154_data *d, uint32_t ms)
{
	crumb(IEEE154_ESP_SDR_EV_TAKE, (uint32_t)atomic_get(&d->waiters));
#if defined(CONFIG_ESP_SDR_IEEE802154_RING)
	atomic_inc(&d->waiters);
#endif
	for (uint32_t t = 0; t < ms; t++) {
		if (k_mutex_lock(&d->radio_lock, K_MSEC(1)) == 0) {
			return true;
		}
#if defined(CONFIG_ESP_SDR_IEEE802154_RING)
		esp_sdr_ring_stop();
#endif
	}
#if defined(CONFIG_ESP_SDR_IEEE802154_RING)
	atomic_dec(&d->waiters);
#endif
	return false;
}

static void radio_give(struct sdr154_data *d)
{
	crumb(IEEE154_ESP_SDR_EV_GIVE, 0);
#if defined(CONFIG_ESP_SDR_IEEE802154_RING)
	atomic_dec(&d->waiters);
#endif
	k_mutex_unlock(&d->radio_lock);
}

#if !defined(CONFIG_NET_L2_IEEE802154)
static ieee154_esp_sdr_rx_cb_t rx_cb;
#endif

void ieee154_esp_sdr_set_rx_cb(ieee154_esp_sdr_rx_cb_t cb)
{
#if defined(CONFIG_NET_L2_IEEE802154)
	ARG_UNUSED(cb); /* frames go to the network interface */
#else
	rx_cb = cb;
#endif
}

static uint32_t channel_mhz(uint16_t ch)
{
	return 2405U + 5U * (ch - 11U);
}

/* Diagnostic: LO above the channel (receive by subsampling only, see ieee154_esp_sdr_set_lo_above()). */
static bool lo_above;

static int tune(struct sdr154_data *d)
{
	return esp_sdr_set_freq(lo_above ? channel_mhz(d->channel) + LO_OFFSET_MHZ
					      : channel_mhz(d->channel) - LO_OFFSET_MHZ);
}

#if defined(CONFIG_ESP_SDR_IEEE802154_RING)
static void verdict_latency(struct sdr154_data *d);
#endif
#if defined(CONFIG_ESP_SDR_IEEE802154_AUTO_ACK)
static void ack_verdict(struct sdr154_data *d);
#endif

static void on_frame(const uint8_t *psdu, uint8_t len, int lqi, void *user)
{
	struct sdr154_data *d = user;
	struct net_pkt *pkt;

#if defined(CONFIG_ESP_SDR_IEEE802154_AUTO_ACK)
	ack_verdict(d);
#endif
#if defined(CONFIG_ESP_SDR_IEEE802154_RING)
	verdict_latency(d);
#endif
	/* Imm-ACK: frame type 2, 5 bytes with the FCS. */
	if (len == 5U && (psdu[0] & 0x7U) == 2U) {
		if (ack_seq >= 0 && psdu[2] == (uint8_t)ack_seq) {
			ack_seen = true;
			STATS_UPDATE(d->stats.acks++);
			return;
		}
#if defined(CONFIG_NET_L2_IEEE802154)
		/* An ACK nobody awaits is no data for the stack. */
		return;
#endif
	}
	STATS_UPDATE(d->stats.frames++);
	/* IEEE802154_HW_FCS: frames go up without the FCS, checked here already. */
	pkt = net_pkt_rx_alloc_with_buffer(d->iface, len - 2U, NET_AF_UNSPEC, 0, K_NO_WAIT);
	if (pkt == NULL) {
		STATS_UPDATE(d->stats.frames_dropped++);
		return;
	}
	if (net_pkt_write(pkt, psdu, len - 2U) < 0) {
		goto drop;
	}
	net_pkt_set_ieee802154_lqi(pkt, (uint8_t)lqi);
	net_pkt_set_ieee802154_rssi_dbm(pkt, IEEE802154_MAC_RSSI_DBM_UNDEFINED);
#if defined(CONFIG_NET_L2_IEEE802154)
	/*
	 * Not ieee802154_handle_ack(): for a driver with HW_TX_RX_ACK it takes
	 * every packet it is given. ACKs are consumed above.
	 */
	if (net_recv_data(d->iface, pkt) < 0) {
		goto drop;
	}
#else
	if (rx_cb == NULL) {
		goto drop;
	}
	rx_cb(pkt);
#endif
	return;
drop:
	STATS_UPDATE(d->stats.frames_dropped++);
	net_pkt_unref(pkt);
}

#if defined(CONFIG_ESP_SDR_IEEE802154_RING)
/*
 * Gapless receive: the ring (CPU 1, interrupts masked) mixes and decimates
 * to 4 MS/s into the sample ring (rx_iq, internal RAM); the PHY thread
 * drains it.
 */
#define SRING_SAMPLES 4096U
/* Samples per PHY slice (0.25 ms). */
#define PHY_CHUNK     1024U
#define sring         (high->rx_iq)
BUILD_ASSERT(RX_IQ_SAMPLES >= SRING_SAMPLES, "sample ring does not fit rx_iq");
/* Output index past the last sample written, where the last gap ended, and the run. */
static atomic_t sring_w, sring_gap, sring_gen;
static uint32_t sink_next;

/*
 * Start a new generation of the sample ring: a ring run, or a bank capture
 * that reused the buffer (radio lock held, so no run). The generation is odd
 * while the indices change, so the PHY thread never pairs an old generation
 * with new indices.
 */
static void sring_new_gen(void)
{
	atomic_inc(&sring_gen);
	sink_next = 0;
	atomic_set(&sring_gap, 0);
	atomic_set(&sring_w, 0);
	atomic_inc(&sring_gen);
}

/*
 * Stall catcher: the PHY thread on CPU 0 counts its loop passes. If the
 * count stands still for about a second while the ring (CPU 1) runs, the
 * sink samples CPU 0's PC from the assist-debug recorder into RTC memory,
 * which a following watchdog reset keeps (ieee154_esp_sdr_get_stall()).
 */
#include <soc/assist_debug_reg.h>

#define STALL_SINK_CALLS 50000U /* about 1 s of sink calls (80 samples each at 4 MS/s) */
#define STALL_SAMPLES    16U
static atomic_t phy_beat;
static uint32_t stall_last_beat, stall_calls;
static RTC_NOINIT_ATTR struct {
	uint32_t magic, n;
	uint32_t pc[STALL_SAMPLES], ls[STALL_SAMPLES];
} stall;

static IRAM_ATTR void stall_check(void)
{
	uint32_t beat = (uint32_t)atomic_get(&phy_beat);

	if (beat != stall_last_beat) {
		stall_last_beat = beat;
		stall_calls = 0;
		return;
	}
	stall_calls++;
	if (stall_calls == STALL_SINK_CALLS) {
		stall.magic = TRAIL_MAGIC;
		stall.n = 0;
	}
	if (stall_calls >= STALL_SINK_CALLS && stall.n < STALL_SAMPLES &&
	    (stall_calls - STALL_SINK_CALLS) % 1000U == 0U) {
		stall.pc[stall.n] = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGPC_REG);
		stall.ls[stall.n] = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGLS0ADDR_REG);
		stall.n++;
	}
}

size_t ieee154_esp_sdr_get_stall(uint32_t *pc, uint32_t *ls, size_t max)
{
	size_t n = 0;

	if (stall.magic == TRAIL_MAGIC && stall.n <= STALL_SAMPLES) {
		for (; n < stall.n && n < max; n++) {
			pc[n] = stall.pc[n];
			ls[n] = stall.ls[n];
		}
	}
	return n;
}

static IRAM_ATTR void ring_sink(void *user, const int16_t *i, const int16_t *q, size_t n,
				uint64_t index)
{
	uint32_t at = (uint32_t)index;

	ARG_UNUSED(user);
	stall_check();
	if (at != sink_next) {
		atomic_set(&sring_gap, (atomic_val_t)at);
	}
	for (size_t k = 0; k < n; k++) {
		uint32_t s = (at + k) & (SRING_SAMPLES - 1U);

		sring[2U * s] = i[k];
		sring[2U * s + 1U] = (int16_t)(rx_qsign * q[k]);
	}
	sink_next = at + (uint32_t)n;
	atomic_set(&sring_w, (atomic_val_t)sink_next);
}

static struct esp_sdr_ring_cfg ring_cfg = {
	.decim = ESP_SDR_RING_DECIM_BOX4,
	.shift_fs4 = true,
	.sink = ring_sink,
	/* Summing four costs about 14 cycles per pair, over budget at 16 MS/s with the sink. */
	.subsample = true,
	/*
	 * 64 us units: the PHY decides a frame about 100 us after its end
	 * (768 us units: about 900), and a unit (256 samples) never comes
	 * near the size of the sample ring.
	 */
	.unit_pairs = ESP_SDR_RING_UNIT_MIN,
	/* Bank 2 holds the ACK waveform. */
	.spare_bank = IS_ENABLED(CONFIG_ESP_SDR_IEEE802154_AUTO_ACK),
};

/* The sample ring index and the PHY's sample count where the block being fed starts. */
static uint32_t blk_r, blk_pos;

/*
 * In the frame callback: how long after the frame's last sample was on the
 * air the PHY decided it, from the ring's live position (pair 4 k is output
 * sample k). Receive chain delays not included.
 */
static void verdict_latency(struct sdr154_data *d)
{
	uint32_t end = blk_r + (ieee154_phy_rx_position() - blk_pos);
	uint64_t pair;
	uint32_t us;

	if (!esp_sdr_ring_now(&pair)) {
		return;
	}
	us = ((uint32_t)(pair / 4U) - end) / 4U;
	STATS_UPDATE({
		d->stats.verdict_n++;
		d->stats.verdict_sum_us += us;
		d->stats.verdict_min_us = d->stats.verdict_n == 1U ? us
				       : MIN(d->stats.verdict_min_us, us);
		d->stats.verdict_max_us = MAX(d->stats.verdict_max_us, us);
	});
}

int ieee154_esp_sdr_set_ring_unit(uint32_t pairs)
{
	struct esp_sdr_ring_cfg cfg = ring_cfg;

	cfg.unit_pairs = pairs != 0U ? pairs : ESP_SDR_RING_UNIT_MIN;
	if (pairs != 0U && (pairs < ESP_SDR_RING_UNIT_MIN || pairs > ESP_SDR_RING_UNIT_DEFAULT ||
			    pairs % 64U != 0U)) {
		return -EINVAL;
	}
	radio_take(&sdr154);
	ring_cfg = cfg;
	radio_give(&sdr154);
	return 0;
}

#if defined(CONFIG_ESP_SDR_IEEE802154_AUTO_ACK)
/*
 * ACKs from inside the ring run (doc/ieee802154-ack.md): the PHY's header
 * callback decides from the addresses whether an ACK is due and arms a ring
 * burst with the frame's end; the waveform's first 416 slots (preamble,
 * SFD, PHR and the frame control's low nibble) are the same for every ACK
 * and sit in the burst bank from the run's start, the rest is built by the
 * ring between its polls. The frame callback fires it 192 us after the end,
 * a failed FCS cancels it.
 */
#define ACK_PREFIX_SLOTS 416U
#define ACK_PREFIX_WORDS (ACK_PREFIX_SLOTS * IEEE154_DSP_SLOT)
/* 16 ring pairs per us. */
#define PAIRS_PER_US 16U

/* Start of the ACK after the frame's end; no start after the limit. */
static uint32_t ack_delay_us = 192U;
static uint32_t ack_limit_us = 512U;

/* The armed ACK (PHY thread only): the frame's end in PHY samples and ring pairs. */
static struct {
	bool armed, fired;
	uint32_t end_pos;
	uint64_t end_pair;
	/* The ring's pair and this CPU's cycle counter at the arm, to time the verdict. */
	uint64_t arm_pair;
	uint32_t arm_cyc;
} ack ESP_SDR_HIGH_RAM;

static void ack_build(uint8_t seq)
{
	uint8_t psdu[5] = {0x02, 0x00, seq};

	(void)ieee154_phy_append_fcs(psdu, 3U);
	ieee154_dsp_tx_build(&high->tx, psdu, sizeof(psdu));
}

/* Before a run: the constant part of every ACK into the burst bank. Radio held. */
static void ack_prefix(void)
{
	size_t words;
	uint32_t *bank = esp_sdr_ring_burst_buf(&words);

	ack_build(0);
	tx_gen(NULL, bank, 0, ACK_PREFIX_WORDS);
	ack.armed = false;
	ack.fired = false;
}

static bool addr_is_ours(const uint8_t *p, uint8_t mode)
{
	if (mode == 2U) {
		return sys_get_le16(p) == addr.short_addr;
	}
	return memcmp(p, addr.ext_addr, sizeof(addr.ext_addr)) == 0;
}

/* PHY thread, PHY lock held: is an ACK due for the frame coming in? */
static bool on_header(const uint8_t *psdu, uint8_t have, uint8_t len, uint32_t end_pos,
		      void *user)
{
	struct sdr154_data *d = user;
	uint16_t fcf;
	uint8_t type, dst_mode, need;
	uint32_t e_out;
	uint64_t pair, end_pair;
	int ret;

	ARG_UNUSED(len);
	if (addr.promisc) {
		return false;
	}
	if (have < 3U) {
		return true;
	}
	fcf = sys_get_le16(psdu);
	type = fcf & 0x7U;
	dst_mode = (fcf >> 10) & 0x3U;
	/* Data or command, AR set, frame version 2003 or 2006 (2015 wants an Enh-Ack). */
	if ((type != 1U && type != 3U) || (fcf & BIT(5)) == 0U || ((fcf >> 12) & 0x3U) > 1U) {
		return false;
	}
	if (dst_mode != 2U && dst_mode != 3U) {
		/* No destination: for the PAN coordinator (not decided here yet). */
		STATS_UPDATE(d->stats.ack_unsupported++);
		return false;
	}
	need = 5U + (dst_mode == 2U ? 2U : 8U);
	if (have < need) {
		return true;
	}
	if (sys_get_le16(&psdu[3]) != addr.pan_id && sys_get_le16(&psdu[3]) != 0xffffU) {
		return false;
	}
	if (!addr_is_ours(&psdu[5], dst_mode)) {
		return false;
	}
	/* Both phase trackers may report the frame. */
	if (ack.armed && (int32_t)(end_pos - ack.end_pos) < 8 &&
	    (int32_t)(ack.end_pos - end_pos) < 8) {
		return false;
	}
	STATS_UPDATE(d->stats.ack_due++);
	if (!esp_sdr_ring_now(&pair)) {
		STATS_UPDATE(d->stats.ack_arm_failed++);
		return false;
	}
	/* The frame's last sample is output sample e_out - 1, pair 4 e_out - 4 on. */
	e_out = blk_r + (end_pos - blk_pos);
	end_pair = pair + 4 * (int64_t)(int32_t)(e_out - (uint32_t)(pair / 4U));
	ack_build(psdu[2]);
	ret = esp_sdr_ring_burst_arm(end_pair, end_pair + ack_limit_us * PAIRS_PER_US, tx_gen,
				     NULL, ACK_PREFIX_WORDS,
				     (uint32_t)ieee154_dsp_tx_samples(&high->tx));
	if (ret != 0) {
		STATS_UPDATE(d->stats.ack_arm_failed++);
		return false;
	}
	ack.armed = true;
	ack.fired = false;
	ack.end_pos = end_pos;
	ack.end_pair = end_pair;
	ack.arm_pair = pair;
	ack.arm_cyc = k_cycle_get_32();
	{
		/* How far the PHY was behind the air when the header came. */
		int32_t lag = (int32_t)((uint32_t)(pair / 4U) - e_out) + (int32_t)(end_pos -
				ieee154_phy_rx_position());

		STATS_UPDATE(d->stats.ack_lag_max_us = MAX(d->stats.ack_lag_max_us, (uint32_t)MAX(lag, 0) / 4U));
	}
	/*
	 * Until the verdict (about a unit after the frame's end) nothing on
	 * this CPU may sit between the PHY and the last samples.
	 */
	k_thread_priority_set(k_current_get(), CONFIG_ESP_SDR_IEEE802154_ACK_PRIORITY);
	return false;
}

/* Frame callback: the armed frame passed its FCS check. */
static void ack_verdict(struct sdr154_data *d)
{
	uint32_t pos = ieee154_phy_rx_position();

	if (!ack.armed || ack.fired || (int32_t)(pos - ack.end_pos) > 8 ||
	    (int32_t)(ack.end_pos - pos) > 8) {
		return;
	}
	ack.fired = true;
	{
		/* The verdict on the ring's time base, through this CPU's clock since the arm. */
		uint64_t now = ack.arm_pair + (uint64_t)k_cyc_to_us_floor32(k_cycle_get_32() -
									 ack.arm_cyc) * PAIRS_PER_US;
		uint32_t us = now > ack.end_pair ? (uint32_t)(now - ack.end_pair) / PAIRS_PER_US : 0U;

		STATS_UPDATE({
			d->stats.ack_verdict_max_us = MAX(d->stats.ack_verdict_max_us, us);
			d->stats.ack_verdict_sum_us += us;
			if (us > ack_delay_us) {
				d->stats.ack_verdict_late++;
			}
		});
	}
	if (esp_sdr_ring_burst_fire(ack.end_pair + ack_delay_us * PAIRS_PER_US)) {
		STATS_UPDATE(d->stats.ack_fired++);
	} else {
		STATS_UPDATE(d->stats.ack_fire_failed++);
	}
}

/* After each PHY block: an armed frame the PHY is past without a verdict failed. */
static void ack_check(struct sdr154_data *d, uint32_t pos)
{
	if (!ack.armed) {
		return;
	}
	if (!ack.fired && esp_sdr_ring_burst_busy()) {
		if ((int32_t)(pos - ack.end_pos) <= 8) {
			return;
		}
		esp_sdr_ring_burst_cancel();
		STATS_UPDATE(d->stats.ack_cancelled++);
	}
	/* Decided: the PHY thread goes back below the system threads. */
	ack.armed = false;
	ack.fired = false;
	k_thread_priority_set(k_current_get(), CONFIG_ESP_SDR_IEEE802154_PHY_PRIORITY);
}

int ieee154_esp_sdr_ack_test(uint8_t seq)
{
	uint64_t pair, end;
	int ret;

	k_mutex_lock(&phy_lock, K_FOREVER);
	if (ack.armed || !esp_sdr_ring_now(&pair)) {
		k_mutex_unlock(&phy_lock);
		return -EBUSY;
	}
	/* A frame "ending" two units from now. */
	end = pair + 2U * ESP_SDR_RING_UNIT_MIN;
	ack_build(seq);
	ret = esp_sdr_ring_burst_arm(end, end + ack_limit_us * PAIRS_PER_US, tx_gen, NULL,
				     ACK_PREFIX_WORDS, (uint32_t)ieee154_dsp_tx_samples(&high->tx));
	if (ret == 0 && !esp_sdr_ring_burst_fire(end + ack_delay_us * PAIRS_PER_US)) {
		ret = -EIO;
	}
	k_mutex_unlock(&phy_lock);
	return ret;
}

int ieee154_esp_sdr_set_ack_timing(uint32_t delay_us, uint32_t limit_us)
{
	if (delay_us > limit_us || limit_us > 2000U) {
		return -EINVAL;
	}
	ack_delay_us = delay_us;
	ack_limit_us = limit_us;
	return 0;
}
#else
int ieee154_esp_sdr_ack_test(uint8_t seq)
{
	ARG_UNUSED(seq);
	return -ENOTSUP;
}

int ieee154_esp_sdr_set_ack_timing(uint32_t delay_us, uint32_t limit_us)
{
	ARG_UNUSED(delay_us);
	ARG_UNUSED(limit_us);
	return -ENOTSUP;
}
#endif /* CONFIG_ESP_SDR_IEEE802154_AUTO_ACK */

/* Runs the ring while receiving and no transmission waits for the radio. */
static void ring_thread_fn(void *p1, void *p2, void *p3)
{
	struct sdr154_data *d = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	for (;;) {
		struct esp_sdr_ring_stats st = {0};
		int ret;

		if (!atomic_get(&d->running)) {
			k_sem_take(&d->run_sem, K_FOREVER);
			continue;
		}
		if (atomic_get(&d->waiters) != 0) {
			k_sleep(K_USEC(200));
			continue;
		}
		k_mutex_lock(&d->radio_lock, K_FOREVER);
		if (!atomic_get(&d->running) || atomic_get(&d->waiters) != 0) {
			k_mutex_unlock(&d->radio_lock);
			continue;
		}
		/* Each run restarts the output index at 0: a new generation for the PHY thread. */
		sring_new_gen();
		STATS_UPDATE(d->stats.captures++);
		crumb(IEEE154_ESP_SDR_EV_RING, 0);
#if defined(CONFIG_ESP_SDR_IEEE802154_AUTO_ACK)
		/* The transmit path reuses the chips and the bank between runs. */
		k_mutex_lock(&phy_lock, K_FOREVER);
		ack_prefix();
		k_mutex_unlock(&phy_lock);
#endif
		ret = esp_sdr_ring_run(&ring_cfg, &st);
		/* Status of a run that started; 0x7f: it could not. */
		crumb(IEEE154_ESP_SDR_EV_RING_END, ret == 0 || ret == -EIO ? (uint32_t)st.status : 0x7fU);
		k_mutex_unlock(&d->radio_lock);
		if (ret == 0 || ret == -EIO) {
			STATS_UPDATE({
				d->stats.ring_status = st.status;
				d->stats.ring_units += st.units;
				d->stats.ring_lost += st.lost_pairs;
				d->stats.ring_cycles_x100 = st.cycles_x100;
				d->stats.ring_slice_max = st.slice_max;
				d->stats.ring_abandoned = st.abandoned;
				d->stats.ring_irq_windows = st.irq_windows;
				d->stats.ring_irq_lines = st.irq_lines;
				d->stats.ring_late_max = MAX(d->stats.ring_late_max, st.late_max);
			});
		}
		if (ret != 0) {
			STATS_UPDATE(d->stats.capture_errors++);
			/* A failed run is retried at once; a ring that cannot start, not so often. */
			k_sleep(ret == -EIO ? K_MSEC(1) : K_MSEC(100));
		}
	}
}

/* Feeds the PHY from the sample ring, on the other CPU. */
static void rx_thread_fn(void *p1, void *p2, void *p3)
{
	struct sdr154_data *d = p1;
	atomic_val_t gen = 0;
	uint32_t r = 0;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	ieee154_phy_set_rx_cb(on_frame, d);
#if defined(CONFIG_ESP_SDR_IEEE802154_AUTO_ACK)
	ieee154_phy_set_hdr_cb(on_header, d);
#endif

	/* PC recording of CPU 0 for the stall catcher. */
	REG_WRITE(ASSIST_DEBUG_CORE_0_RCD_PDEBUGENABLE_REG, 1);
	REG_WRITE(ASSIST_DEBUG_CORE_0_RCD_RECORDING_REG, 1);

	for (;;) {
		atomic_val_t g_now = atomic_get(&sring_gen);
		uint32_t w, g, t0;

		atomic_inc(&phy_beat);
		if ((g_now & 1) != 0) {
			/* The ring thread is renewing the indices (microseconds). */
			k_yield();
			continue;
		}
		w = (uint32_t)atomic_get(&sring_w);
		if (atomic_get(&sring_gen) != g_now) {
			/* w may belong to the next generation. */
			continue;
		}
		/* Indices wrap after about 18 minutes: compare differences only. */
		if (g_now != gen) {
			/* A new ring run: its samples start at index 0. */
			gen = g_now;
			r = 0;
			k_mutex_lock(&phy_lock, K_FOREVER);
			ieee154_phy_rx_reset();
			k_mutex_unlock(&phy_lock);
			w = (uint32_t)atomic_get(&sring_w);
		}
		if (w - r > SRING_SAMPLES - 1024U) {
			/* The PHY fell a ring behind. */
			STATS_UPDATE(d->stats.phy_skipped += w - r);
			r = w;
			k_mutex_lock(&phy_lock, K_FOREVER);
			ieee154_phy_rx_reset();
			k_mutex_unlock(&phy_lock);
		}
		g = (uint32_t)atomic_get(&sring_gap);
		if ((int32_t)(g - r) > 0 && (int32_t)(w - g) >= 0) {
			r = g;
			k_mutex_lock(&phy_lock, K_FOREVER);
			ieee154_phy_rx_reset();
			k_mutex_unlock(&phy_lock);
		}
		if (w == r) {
#if defined(CONFIG_ESP_SDR_IEEE802154_AUTO_ACK)
			if (ack.armed) {
				/* The verdict is due within a unit: no sleep until then. */
				k_mutex_lock(&phy_lock, K_FOREVER);
				ack_check(d, ieee154_phy_rx_position());
				k_mutex_unlock(&phy_lock);
				k_yield();
				continue;
			}
#endif
			/*
			 * One tick: a frame's start found late leaves the PHY behind,
			 * and at about 87 % load it catches up slowly.
			 */
			k_sleep(K_USEC(100));
			continue;
		}
		t0 = k_cycle_get_32();
		k_mutex_lock(&phy_lock, K_FOREVER);
		if (atomic_get(&sring_gen) != gen) {
			/* A bank capture reused the buffer while we waited for the lock. */
			k_mutex_unlock(&phy_lock);
			continue;
		}
		while (r != w) {
			uint32_t s = r & (SRING_SAMPLES - 1U);
			uint32_t n = MIN(MIN(w - r, SRING_SAMPLES - s), PHY_CHUNK);

			blk_r = r;
			blk_pos = ieee154_phy_rx_position();
			ieee154_phy_rx_block(&sring[2U * s], n);
			r += n;
#if defined(CONFIG_ESP_SDR_IEEE802154_AUTO_ACK)
			ack_check(d, ieee154_phy_rx_position());
#endif
		}
		k_mutex_unlock(&phy_lock);
		/* Bounded slices: threads of the same priority get their turn. */
		k_yield();
		STATS_UPDATE(d->stats.phy_us += k_cyc_to_us_floor32(k_cycle_get_32() - t0));
		STATS_UPDATE(d->stats.phy_read = r);
	}
}

static struct k_thread sdr154_ring_thread;
#else
/* Stalls are watched by the ring only. */
size_t ieee154_esp_sdr_get_stall(uint32_t *pc, uint32_t *ls, size_t max)
{
	ARG_UNUSED(pc);
	ARG_UNUSED(ls);
	ARG_UNUSED(max);
	return 0;
}

static void rx_thread_fn(void *p1, void *p2, void *p3)
{
	struct sdr154_data *d = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	ieee154_phy_set_rx_cb(on_frame, d);

	for (;;) {
		struct esp_sdr_rx_burst b;
		uint32_t t0, t1;
		int ret;

		if (!atomic_get(&d->running)) {
			k_sem_take(&d->run_sem, K_FOREVER);
			continue;
		}
		/* Lock order: radio, then PHY (rx_iq is the PHY's too, see ack_wait()). */
		k_mutex_lock(&d->radio_lock, K_FOREVER);
		k_mutex_lock(&phy_lock, K_FOREVER);
		crumb(IEEE154_ESP_SDR_EV_CAPTURE, 0);
		ret = esp_sdr_rx_capture(RX_RATE, RX_WORDS, &b);
		t0 = k_cycle_get_32();
		if (ret == 0) {
			/* Under the radio lock: a transmission reuses the bank. */
			mix_decimate(b.words, high->rx_iq);
		}
		k_mutex_unlock(&d->radio_lock);
		if (ret != 0) {
			k_mutex_unlock(&phy_lock);
			STATS_UPDATE(d->stats.capture_errors++);
			k_sleep(K_MSEC(1));
			continue;
		}
		t1 = k_cycle_get_32();
		ieee154_phy_rx_reset();
		ieee154_phy_rx_block(high->rx_iq, RX_OUT);
		k_mutex_unlock(&phy_lock);
		STATS_UPDATE(d->stats.captures++);
		STATS_UPDATE(d->stats.filter_us = k_cyc_to_us_floor32(t1 - t0));
		STATS_UPDATE(d->stats.phy_us = k_cyc_to_us_floor32(k_cycle_get_32() - t1));
	}
}
#endif

#if !defined(CONFIG_ESP_SDR_IEEE802154_RING)
/* Burst receive has no sample ring to invalidate. */
static inline void sring_new_gen(void)
{
}
#endif

static struct k_thread sdr154_rx_thread;

static enum ieee802154_hw_caps sdr154_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* TX_RX_ACK: tx() waits for the ACK of a frame that asks for one. */
	return IEEE802154_HW_FCS | IEEE802154_HW_PROMISC | IEEE802154_HW_TX_RX_ACK |
	       (IS_ENABLED(CONFIG_ESP_SDR_IEEE802154_AUTO_ACK)
			? IEEE802154_HW_RX_TX_ACK | IEEE802154_HW_FILTER
			: 0);
}

static int sdr154_cca(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int sdr154_set_channel(const struct device *dev, uint16_t ch)
{
	struct sdr154_data *d = dev->data;
	uint16_t prev;
	int ret = 0;

	if (ch < 11U || ch > 26U) {
		return -EINVAL;
	}
	radio_take(d);
	prev = d->channel;
	d->channel = ch;
	if (d->sdr_up) {
		ret = tune(d);
		if (ret != 0) {
			d->channel = prev;
		}
	}
	radio_give(d);
	return ret;
}

static int sdr154_filter(const struct device *dev, bool set, enum ieee802154_filter_type type,
			 const struct ieee802154_filter *filter)
{
	ARG_UNUSED(dev);
	if (!set) {
		return -ENOTSUP;
	}
	switch (type) {
	case IEEE802154_FILTER_TYPE_PAN_ID:
		addr.pan_id = filter->pan_id;
		return 0;
	case IEEE802154_FILTER_TYPE_SHORT_ADDR:
		addr.short_addr = filter->short_addr;
		return 0;
	case IEEE802154_FILTER_TYPE_IEEE_ADDR:
		/* Little endian, as in the frame. */
		memcpy(addr.ext_addr, filter->ieee_addr, sizeof(addr.ext_addr));
		return 0;
	default:
		return -ENOTSUP;
	}
}

/* Strongest step taken as +20 dBm; the vendor ladder is in quarter dB (4.24 per dB measured). */
static int sdr154_set_txpower(const struct device *dev, int16_t dbm)
{
	struct sdr154_data *d = dev->data;
	int max = esp_sdr_tx_gain_max(), top = esp_sdr_tx_gain_power(max), best = 0;
	int ret;

	for (int s = 0; s <= max; s++) {
		int p = 20 - (top - esp_sdr_tx_gain_power(s)) * 100 / 424;

		if (p <= dbm) {
			best = s;
		}
	}
	/* esp_sdr takes the radio lock, which a ring run holds throughout. */
	radio_take(d);
	ret = esp_sdr_tx_set_gain(best);
	radio_give(d);
	return ret;
}

/* A frame other than an ACK with the AR bit set. */
static bool ack_requested(const uint8_t *psdu, size_t len)
{
	return len >= 5U && (psdu[0] & 0x7U) != 2U && (psdu[0] & BIT(5)) != 0U;
}

/*
 * The playback ends at the frame's last sample and the front end needs 50 us
 * back to RX, so one 1 ms capture holds an ACK sent 192 us (turnaround time)
 * after the frame. Radio lock held.
 */
static int ack_wait(struct sdr154_data *d, uint8_t seq)
{
	struct esp_sdr_rx_burst b;
	int ret;

	k_mutex_lock(&phy_lock, K_FOREVER);
	crumb(IEEE154_ESP_SDR_EV_CAPTURE, 0);
	ret = esp_sdr_rx_capture(RX_RATE, RX_WORDS, &b);
	if (ret != 0) {
		k_mutex_unlock(&phy_lock);
		STATS_UPDATE(d->stats.capture_errors++);
		return -EIO;
	}
	mix_decimate(b.words, high->rx_iq);
	ack_seq = seq;
	ack_seen = false;
	ieee154_phy_rx_reset();
	ieee154_phy_rx_block(high->rx_iq, RX_OUT);
	ieee154_phy_rx_reset();
	ack_seq = -1;
	/* rx_iq is the sample ring too: what the PHY thread had left in it is gone. */
	sring_new_gen();
	k_mutex_unlock(&phy_lock);
	if (!ack_seen) {
		STATS_UPDATE(d->stats.no_acks++);
		return -ENOMSG;
	}
	return 0;
}

static int sdr154_tx(const struct device *dev, enum ieee802154_tx_mode mode, struct net_pkt *pkt,
		     struct net_buf *frag)
{
	struct sdr154_data *d = dev->data;
	uint8_t psdu[IEEE154_PHY_PSDU_MAX];
	size_t len = frag->len;
	int ret;

	ARG_UNUSED(pkt);
	if (mode != IEEE802154_TX_MODE_DIRECT && mode != IEEE802154_TX_MODE_CSMA_CA) {
		return -ENOTSUP;
	}
	if (len + 2U > sizeof(psdu)) {
		return -EMSGSIZE;
	}
	if (!d->sdr_up) {
		return -ENETDOWN;
	}
	memcpy(psdu, frag->data, len);
	len = ieee154_phy_append_fcs(psdu, len);

	radio_take(d);
	ieee154_dsp_tx_build(&high->tx, psdu, (uint8_t)len);
	crumb(IEEE154_ESP_SDR_EV_TX, (uint32_t)len);
	ret = esp_sdr_tx_dac_play_gen(tx_gen, NULL, ieee154_dsp_tx_samples(&high->tx));
	crumb(IEEE154_ESP_SDR_EV_TX_END, (uint32_t)-ret);
	if (ret == 0) {
		STATS_UPDATE(d->stats.tx_frames++);
		if (ack_requested(psdu, len)) {
			ret = ack_wait(d, psdu[2]);
		}
	} else {
		STATS_UPDATE(d->stats.tx_errors++);
		STATS_UPDATE(d->stats.tx_last_err = ret);
	}
	radio_give(d);
	return ret;
}

static int sdr154_start(const struct device *dev)
{
	struct sdr154_data *d = dev->data;
	int ret;

	radio_take(d);
	if (!d->sdr_up) {
		/* Needs the Wi-Fi driver up, so not at device init. */
		ret = esp_sdr_init();
		if (ret == 0) {
			/* Turnaround: front end only, the LO is shared. */
			(void)esp_sdr_set_turnaround(50, false);
			ret = tune(d);
		}
		if (ret != 0) {
			radio_give(d);
			LOG_ERR("esp_sdr start failed: %d", ret);
			return ret;
		}
		d->sdr_up = true;
	}
	radio_give(d);
	atomic_set(&d->running, 1);
	k_sem_give(&d->run_sem);
	return 0;
}

static int sdr154_stop(const struct device *dev)
{
	struct sdr154_data *d = dev->data;

	atomic_set(&d->running, 0);
	/*
	 * Holding the radio once proves the ring has stopped; it does not
	 * restart while stopped. Another holder means no ring runs, and may
	 * need the thread that called us (a diagnostic dump prints for seconds
	 * through USB): do not wait for it.
	 */
	if (radio_try_take(d, 50U)) {
		radio_give(d);
	}
	return 0;
}

static int sdr154_configure(const struct device *dev, enum ieee802154_config_type type,
			    const struct ieee802154_config *config)
{
	ARG_UNUSED(dev);
	switch (type) {
	case IEEE802154_CONFIG_PROMISCUOUS:
		/* Every frame with a good FCS goes up either way; promiscuous: no ACKs. */
		addr.promisc = config->promiscuous;
		return 0;
	case IEEE802154_CONFIG_PAN_COORDINATOR:
		addr.pan_coord = config->pan_coordinator;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int sdr154_attr_get(const struct device *dev, enum ieee802154_attr attr,
			   struct ieee802154_attr_value *value)
{
	static const struct ieee802154_phy_channel_range ranges[] = {
		{.from_channel = 11, .to_channel = 26},
	};
	static const struct ieee802154_phy_supported_channels chans = {
		.ranges = ranges,
		.num_ranges = 1,
	};

	ARG_UNUSED(dev);
	return ieee802154_attr_get_channel_page_and_range(
		attr, IEEE802154_ATTR_PHY_CHANNEL_PAGE_ZERO_OQPSK_2450_BPSK_868_915, &chans, value);
}

void ieee154_esp_sdr_set_invert(bool rx_inv, bool tx_inv)
{
	radio_take(&sdr154);
	rx_qsign = rx_inv ? -1 : 1;
	tx_conj = tx_inv;
	tmpl_init();
	radio_give(&sdr154);
}

int ieee154_esp_sdr_set_lo_above(bool above)
{
	bool prev;
	int ret = 0;

	radio_take(&sdr154);
	prev = lo_above;
	lo_above = above;
	if (sdr154.sdr_up) {
		ret = tune(&sdr154);
		if (ret != 0) {
			lo_above = prev;
		}
	}
	radio_give(&sdr154);
	return ret;
}

int ieee154_esp_sdr_dump(ieee154_esp_sdr_dump_cb_t out, void *user)
{
	struct sdr154_data *d = &sdr154;
	struct esp_sdr_rx_burst b;
	int ret;

	radio_take(d);
	if (!d->sdr_up) {
		radio_give(d);
		return -ENETDOWN;
	}
	k_mutex_lock(&phy_lock, K_FOREVER);
	crumb(IEEE154_ESP_SDR_EV_CAPTURE, 0);
	ret = esp_sdr_rx_capture(RX_RATE, RX_WORDS, &b);
	if (ret == 0) {
		mix_decimate(b.words, high->rx_iq);
		/* rx_iq is the ring's sample buffer too: hand it over before receive resumes. */
		out(high->rx_iq, RX_OUT, user);
		sring_new_gen();
	}
	k_mutex_unlock(&phy_lock);
	radio_give(d);
	return ret;
}

int ieee154_esp_sdr_set_rx_gain(int index)
{
	int ret;

	radio_take(&sdr154);
	ret = esp_sdr_rx_set_gain(index);
	radio_give(&sdr154);
	return ret;
}

int ieee154_esp_sdr_measure(struct ieee154_esp_sdr_measurement *m)
{
	struct sdr154_data *d = &sdr154;
	struct esp_sdr_rx_burst b;
	size_t n;
	float p;
	int ret;

	radio_take(d);
	if (!d->sdr_up) {
		radio_give(d);
		return -ENETDOWN;
	}
	k_mutex_lock(&phy_lock, K_FOREVER);
	crumb(IEEE154_ESP_SDR_EV_CAPTURE, 0);
	ret = esp_sdr_rx_capture(RX_RATE, RX_WORDS, &b);
	if (ret == 0) {
		mix_decimate(b.words, high->rx_iq);
		m->cfo_hz = (int32_t)ieee154_dsp_cfo(high->rx_iq, RX_OUT, &p, &n);
		m->coarse_cfo_hz = (int32_t)ieee154_dsp_freq(high->rx_iq, RX_OUT);
		ieee154_dsp_steps(high->rx_iq, RX_OUT, m->steps);
		/* Relative to the decimated full scale, +-8192 on each axis. */
		m->power_dbfs = p > 0.0f ? (int32_t)lrintf(10.0f * log10f(p / (8192.0f * 8192.0f)))
					 : -200;
		m->samples_used = (uint32_t)n;
		sring_new_gen();
	}
	k_mutex_unlock(&phy_lock);
	radio_give(d);
	return ret;
}

void ieee154_esp_sdr_get_stats(struct ieee154_esp_sdr_stats *stats)
{
	k_spinlock_key_t key = k_spin_lock(&stats_lock);

	*stats = sdr154.stats;
	k_spin_unlock(&stats_lock, key);
}

int ieee154_esp_sdr_bench(struct ieee154_esp_sdr_bench *b)
{
	struct sdr154_data *d = &sdr154;
	/* An Imm-Ack with frame pending clear, its FCS appended below. */
	uint8_t ack[5] = {0x02, 0x00, 0x5a};
	uint32_t *bank, t0, t1;
	size_t words;
	uint32_t n;
	int ret;

	radio_take(d);
	if (!d->sdr_up) {
		radio_give(d);
		return -ENETDOWN;
	}
	(void)ieee154_phy_append_fcs(ack, 3U);
	/* The capture bank: the CPUs' while the radio is held and nothing plays. */
	bank = esp_sdr_tx_loop_buf(0, &words);
	t0 = k_cycle_get_32();
	ieee154_dsp_tx_build(&high->tx, ack, sizeof(ack));
	t1 = k_cycle_get_32();
	b->ack_build_cyc = t1 - t0;
	n = (uint32_t)ieee154_dsp_tx_samples(&high->tx);
	b->ack_words = n;
	if (n > words) {
		radio_give(d);
		return -ENOMEM;
	}
	t0 = k_cycle_get_32();
	tx_gen(NULL, bank, 0, n);
	t1 = k_cycle_get_32();
	b->ack_gen_cyc = t1 - t0;
	/* What changes between ACKs: from the frame control's high nibble (slot 416) on. */
	t0 = k_cycle_get_32();
	tx_gen(NULL, bank + 416U * IEEE154_DSP_SLOT, 416U * IEEE154_DSP_SLOT,
	       n - 416U * IEEE154_DSP_SLOT);
	t1 = k_cycle_get_32();
	b->ack_tail_cyc = t1 - t0;

	/* Front end: RX to TX and back (a loop session without playback). */
	t0 = k_cycle_get_32();
	ret = esp_sdr_tx_loop_begin(ESP_SDR_RATE_40MSPS);
	t1 = k_cycle_get_32();
	if (ret == 0) {
		b->to_tx_us = k_cyc_to_us_floor32(t1 - t0);
		t0 = k_cycle_get_32();
		esp_sdr_tx_loop_end();
		t1 = k_cycle_get_32();
		b->to_rx_us = k_cyc_to_us_floor32(t1 - t0);
	}

	radio_give(d);
	return ret;
}

void ieee154_esp_sdr_clear_stats(void)
{
	k_spinlock_key_t key = k_spin_lock(&stats_lock);

	memset(&sdr154.stats, 0, sizeof(sdr154.stats));
	k_spin_unlock(&stats_lock, key);
}

#if defined(CONFIG_NET_L2_IEEE802154)
static void sdr154_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct sdr154_data *d = dev->data;
	uint8_t id[6];

	d->iface = iface;
	/* Extended address: the chip's MAC-48 (eFuse) as an EUI-64, FF FE in the middle. */
	if (hwinfo_get_device_id(id, sizeof(id)) != (ssize_t)sizeof(id)) {
		LOG_WRN("no device id, extended address from zeros");
		memset(id, 0, sizeof(id));
	}
	memcpy(&d->mac[0], &id[0], 3);
	d->mac[3] = 0xff;
	d->mac[4] = 0xfe;
	memcpy(&d->mac[5], &id[3], 3);
	net_if_set_link_addr(iface, d->mac, sizeof(d->mac), NET_LINK_IEEE802154);
	ieee802154_init(iface);
}
#endif

static int sdr154_init(const struct device *dev)
{
	struct sdr154_data *d = dev->data;

#if defined(CONFIG_ESP_SDR_IEEE802154_HIGH_RAM_APP)
	high = ieee154_esp_sdr_high_ram(sizeof(*high));
	if (high == NULL) {
		/* The application gives this RAM to something else this boot. */
		return -ENODEV;
	}
	__ASSERT_NO_MSG(IS_ALIGNED(high, __alignof__(*high)));
#endif
	crumb(IEEE154_ESP_SDR_EV_BOOT, 0);
#if defined(CONFIG_ESP_SDR_IEEE802154_AUTO_ACK)
	memset(&ack, 0, sizeof(ack));
#endif
	ieee154_phy_init();
	tmpl_init();
	k_sem_init(&d->run_sem, 0, 1);
	k_mutex_init(&d->radio_lock);
	d->channel = 11U;
	/*
	 * With the ring the PHY thread runs whenever samples wait: below the
	 * system threads, so a PHY slower than real time drops samples
	 * (counted) instead of starving USB and the shell.
	 */
	k_thread_create(&sdr154_rx_thread, high->rx_stack, K_KERNEL_STACK_SIZEOF(high->rx_stack),
			rx_thread_fn, d, NULL, NULL,
			IS_ENABLED(CONFIG_ESP_SDR_IEEE802154_RING)
				? CONFIG_ESP_SDR_IEEE802154_PHY_PRIORITY
				: CONFIG_ESP_SDR_IEEE802154_RX_PRIORITY,
			0, K_FOREVER);
	k_thread_name_set(&sdr154_rx_thread, "sdr154_rx");
#if defined(CONFIG_SCHED_CPU_MASK)
	if (arch_num_cpus() > 1) {
		(void)k_thread_cpu_pin(&sdr154_rx_thread,
				       IS_ENABLED(CONFIG_ESP_SDR_IEEE802154_RING) ? 0 : 1);
	}
#endif
	k_thread_start(&sdr154_rx_thread);
#if defined(CONFIG_ESP_SDR_IEEE802154_RING)
	/* The ring takes CPU 1 whole (interrupts masked); the PHY runs on CPU 0. */
	/* Cooperative: interrupts between the ring's polls must not preempt it. */
	k_thread_create(&sdr154_ring_thread, high->ring_stack,
			K_KERNEL_STACK_SIZEOF(high->ring_stack), ring_thread_fn, d, NULL, NULL,
			K_PRIO_COOP(2), 0, K_FOREVER);
	k_thread_name_set(&sdr154_ring_thread, "sdr154_ring");
#if defined(CONFIG_SCHED_CPU_MASK)
	(void)k_thread_cpu_pin(&sdr154_ring_thread, 1);
#endif
	k_thread_start(&sdr154_ring_thread);
#endif
	return 0;
}

static const struct ieee802154_radio_api sdr154_api = {
#if defined(CONFIG_NET_L2_IEEE802154)
	.iface_api.init = sdr154_iface_init,
#endif
	.get_capabilities = sdr154_caps,
	.cca = sdr154_cca,
	.set_channel = sdr154_set_channel,
	.filter = sdr154_filter,
	.set_txpower = sdr154_set_txpower,
	.start = sdr154_start,
	.stop = sdr154_stop,
	.tx = sdr154_tx,
	.configure = sdr154_configure,
	.attr_get = sdr154_attr_get,
};

#if !defined(CONFIG_NET_L2_IEEE802154)
DEVICE_DEFINE(esp_sdr_154, "esp_sdr_154", sdr154_init, NULL, &sdr154, NULL, POST_KERNEL,
	      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &sdr154_api);
#else
NET_DEVICE_INIT(esp_sdr_154, "esp_sdr_154", sdr154_init, NULL, &sdr154, NULL,
		CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &sdr154_api, IEEE802154_L2,
		NET_L2_GET_CTX_TYPE(IEEE802154_L2), IEEE802154_MTU);
#endif
