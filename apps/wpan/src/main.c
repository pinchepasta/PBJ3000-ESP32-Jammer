/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * 802.15.4 test frames over any Zephyr radio in raw mode: a sender with a
 * numbered, checkable payload and a receiver that counts what arrives.
 * The same shell runs on the native C6 radio and on the esp-sdr software
 * radio, so either side can be swapped.
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/app_version.h>
#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/base64.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/sys_heap.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include "app_cpu.h"
#include "app_crash.h"
#include "app_usb.h"
#endif
#if defined(CONFIG_ESP_SDR)
#include <esp_sdr/esp_sdr.h>
#endif
#if defined(CONFIG_ESP_SDR_RING)
#include <esp_sdr/esp_sdr_ring.h>
#endif
#if defined(CONFIG_IEEE802154_ESP32)
#include <esp_ieee802154.h>
#endif
#if defined(CONFIG_ESP_SDR_IEEE802154)
#include <esp_sdr/ieee154_esp_sdr.h>
#include <esp_sdr/ieee154_phy.h>
#endif

LOG_MODULE_REGISTER(wpan, LOG_LEVEL_INF);

#if DT_HAS_CHOSEN(zephyr_ieee802154)
static const struct device *const radio = DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154));
#else
/* The esp-sdr driver has no devicetree node. */
#define RADIO_NAME "esp_sdr_154"
static const struct device *radio;
#endif

static const struct ieee802154_radio_api *api;

/* Data frame, PAN ID compression, short addresses: FCF, seq, PAN, dst, src. */
#define FCF_DATA       0x8841U
#define FCF_ACK_REQ    BIT(5)
#define HDR_LEN        9U
#define FCS_LEN        2U
/* Test payload: magic, counter, then a pattern derived from both. */
#define MAGIC          0x34353158U /* "X154" */
#define TEST_HDR_LEN   (HDR_LEN + 8U)
#define PSDU_MAX       127U
#define PAYLOAD_MAX    (PSDU_MAX - HDR_LEN - FCS_LEN)
#define BROADCAST      0xffffU

static uint16_t pan_id = 0xabcdU;
static uint16_t short_addr = 0x0001U;
static uint16_t channel = 11U;
/* Receiving (api->start() done). */
static bool radio_on;

static uint8_t pattern(uint32_t counter, size_t k)
{
	return (uint8_t)(counter * 31U + k * 7U + 1U);
}

/* ---------------------------------------------------------------- receive */

struct rx_stats {
	uint32_t frames, test, bad, dups, lost, rssi_n;
	int32_t lqi_sum, rssi_sum;
	int16_t rssi_min, rssi_max;
	uint32_t last_counter;
	uint16_t last_src;
	bool have_last;
};

static struct rx_stats rx;
static struct k_spinlock rx_lock;
static atomic_t rx_verbose = ATOMIC_INIT(1);

static void rx_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&rx_lock);

	memset(&rx, 0, sizeof(rx));
	k_spin_unlock(&rx_lock, key);
}

/* Returns the frame's test counter, or -1 for a non-test frame, -2 for a corrupt one. */
static int64_t check_test(const uint8_t *p, size_t len)
{
	uint32_t counter;

	if (len < TEST_HDR_LEN || (sys_get_le16(p) & 0x7U) != 1U ||
	    sys_get_le32(&p[HDR_LEN]) != MAGIC) {
		return -1;
	}
	counter = sys_get_le32(&p[HDR_LEN + 4U]);
	for (size_t k = TEST_HDR_LEN; k < len; k++) {
		if (p[k] != pattern(counter, k)) {
			return -2;
		}
	}
	return counter;
}

static void rx_frame(struct net_pkt *pkt)
{
	static uint8_t buf[PSDU_MAX];
	size_t len = MIN(net_pkt_get_len(pkt), sizeof(buf));
	int lqi = net_pkt_ieee802154_lqi(pkt);
	int rssi = net_pkt_ieee802154_rssi_dbm(pkt);
	uint16_t src = 0U;
	int64_t counter;
	k_spinlock_key_t key;

	net_pkt_cursor_init(pkt);
	if (net_pkt_read(pkt, buf, len) != 0) {
		net_pkt_unref(pkt);
		return;
	}
	net_pkt_unref(pkt);
	/* Raw mode can hand over the FCS too (on the ESP32 overwritten with RSSI and LQI). */
	if (IS_ENABLED(CONFIG_IEEE802154_L2_PKT_INCL_FCS) && len >= FCS_LEN) {
		len -= FCS_LEN;
	}

	counter = check_test(buf, len);
	if (len >= HDR_LEN) {
		src = sys_get_le16(&buf[7]);
	}

	key = k_spin_lock(&rx_lock);
	rx.frames++;
	rx.lqi_sum += lqi;
	/* The software radio has no RSSI: undefined values stay out of the average. */
	if (rssi != IEEE802154_MAC_RSSI_DBM_UNDEFINED) {
		rx.rssi_sum += rssi;
		rx.rssi_min = rx.rssi_n == 0U ? rssi : MIN(rx.rssi_min, rssi);
		rx.rssi_max = rx.rssi_n == 0U ? rssi : MAX(rx.rssi_max, rssi);
		rx.rssi_n++;
	}
	if (counter == -2) {
		rx.bad++;
	} else if (counter >= 0) {
		rx.test++;
		if (rx.have_last && rx.last_src == src) {
			uint32_t step = (uint32_t)counter - rx.last_counter;

			if (step == 0U) {
				rx.dups++;
			} else if (step < 0x10000U) {
				rx.lost += step - 1U;
			}
		}
		rx.last_counter = (uint32_t)counter;
		rx.last_src = src;
		rx.have_last = true;
	}
	k_spin_unlock(&rx_lock, key);

	/* Deferred log: under a flood it drops lines instead of holding up the radio thread. */
	if (atomic_get(&rx_verbose) >= 1) {
		LOG_INF("rx len %u lqi %d rssi %d seq %u src %04x %s%d", (unsigned int)len, lqi, rssi,
			len > 2U ? buf[2] : 0U, src,
			counter >= 0 ? "test " : (counter == -2 ? "BAD " : ""),
			counter >= 0 ? (int)counter : 0);
	}
	if (atomic_get(&rx_verbose) >= 2) {
		LOG_HEXDUMP_INF(buf, len, "frame");
	}
}

#if defined(CONFIG_IEEE802154_RAW_MODE)
int net_recv_data(struct net_if *iface, struct net_pkt *pkt)
{
	ARG_UNUSED(iface);
	rx_frame(pkt);
	return 0;
}

enum net_verdict ieee802154_handle_ack(struct net_if *iface, struct net_pkt *pkt)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(pkt);
	return NET_OK;
}
#endif

/* --------------------------------------------------------------- transmit */

struct tx_job {
	uint32_t count, interval_ms, payload, counter;
	uint16_t dst;
	bool ack, csma;
};

struct tx_result {
	uint32_t sent, ok, noack, busy, err;
	int64_t ms;
};

static struct tx_job job;
static struct tx_result txr;
static atomic_t tx_stop;
static K_SEM_DEFINE(tx_go, 0, 1);
/* Ends the wait between frames when a tx is stopped. */
static K_SEM_DEFINE(tx_wake, 0, 1);
static atomic_t tx_running;
static uint8_t seq;

NET_BUF_POOL_DEFINE(tx_pool, 1, PSDU_MAX, 0, NULL);

static int send_frame(uint32_t counter, size_t payload, uint16_t dst, bool ack, bool csma)
{
	uint8_t frame[PSDU_MAX - FCS_LEN];
	size_t len = HDR_LEN + MAX(payload, 8U);
	struct net_pkt *pkt;
	struct net_buf *buf;
	int ret;

	sys_put_le16(FCF_DATA | (ack ? FCF_ACK_REQ : 0U), &frame[0]);
	frame[2] = seq++;
	sys_put_le16(pan_id, &frame[3]);
	sys_put_le16(dst, &frame[5]);
	sys_put_le16(short_addr, &frame[7]);
	sys_put_le32(MAGIC, &frame[HDR_LEN]);
	sys_put_le32(counter, &frame[HDR_LEN + 4U]);
	for (size_t k = TEST_HDR_LEN; k < len; k++) {
		frame[k] = pattern(counter, k);
	}

	/* Radio drivers take the frame as one buffer, whatever the network buffer size. */
	buf = net_buf_alloc(&tx_pool, K_MSEC(100));
	if (buf == NULL) {
		return -ENOMEM;
	}
	net_buf_add_mem(buf, frame, len);
	pkt = net_pkt_alloc(K_MSEC(100));
	if (pkt == NULL) {
		net_buf_unref(buf);
		return -ENOMEM;
	}
	ret = api->tx(radio, csma ? IEEE802154_TX_MODE_CSMA_CA : IEEE802154_TX_MODE_DIRECT, pkt,
		      buf);
	net_pkt_unref(pkt);
	net_buf_unref(buf);
	return ret;
}

static void tx_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		int64_t t0;

		k_sem_take(&tx_go, K_FOREVER);
		k_sem_reset(&tx_wake);
		memset(&txr, 0, sizeof(txr));
		t0 = k_uptime_get();
		for (uint32_t n = 0; (job.count == 0U || n < job.count) && !atomic_get(&tx_stop);
		     n++) {
			int ret = send_frame(job.counter++, job.payload, job.dst, job.ack, job.csma);

			txr.sent++;
			if (ret == 0) {
				txr.ok++;
			} else if (ret == -ENOMSG) {
				txr.noack++;
			} else if (ret == -EBUSY) {
				txr.busy++;
			} else {
				txr.err++;
			}
			if (job.interval_ms != 0U) {
				(void)k_sem_take(&tx_wake, K_MSEC(job.interval_ms));
			}
		}
		txr.ms = k_uptime_get() - t0;
		printk("tx done: sent %u ok %u noack %u busy %u err %u in %lld ms\n", txr.sent,
		       txr.ok, txr.noack, txr.busy, txr.err, txr.ms);
		atomic_set(&tx_running, 0);
	}
}

#if defined(CONFIG_ESP_SDR)
/* Internal RAM below the dump banks is tight on the S3. */
#define TX_STACK_ATTR ESP_SDR_HIGH_RAM
#else
#define TX_STACK_ATTR __kstackmem
#endif
static Z_KERNEL_STACK_DEFINE_IN(tx_stack, 2048, TX_STACK_ATTR);
static struct k_thread tx_thread;

/* ------------------------------------------------------------------ shell */

/* The shell outlives main() returning on "radio not ready". */
static bool radio_ok(const struct shell *sh)
{
	if (api == NULL || radio == NULL) {
		shell_error(sh, "no radio");
		return false;
	}
	return true;
}

static int parse_u32(const struct shell *sh, const char *s, uint32_t max, uint32_t *out)
{
	char *end;
	unsigned long v = strtoul(s, &end, 0);

	if (*s == '\0' || *end != '\0' || v > max) {
		shell_error(sh, "bad value: %s", s);
		return -EINVAL;
	}
	*out = (uint32_t)v;
	return 0;
}

/* ------------------------------------------------------------- settings */

/* Last transmit power set (the radio API has no getter), INT16_MIN: the driver's default. */
static int16_t tx_power_dbm = INT16_MIN;
static bool promiscuous;
#if defined(CONFIG_ESP_SDR_IEEE802154)
static bool rx_invert, tx_invert, lo_above;
#endif

static int parse_long(const struct shell *sh, const char *s, long min, long max, long *out)
{
	char *end;
	long v = strtol(s, &end, 0);

	if (*s == '\0' || *end != '\0' || v < min || v > max) {
		shell_error(sh, "bad value: %s (%ld to %ld)", s, min, max);
		return -EINVAL;
	}
	*out = v;
	return 0;
}

static int set_chan(const struct shell *sh, const char *value)
{
	long ch;
	int ret;

	if (parse_long(sh, value, 11, 26, &ch) != 0) {
		return -EINVAL;
	}
	ret = api->set_channel(radio, (uint16_t)ch);
	if (ret == 0) {
		channel = (uint16_t)ch;
		/* The ESP32 radio applies a new channel when it (re)enters receive. */
		if (radio_on) {
			ret = api->start(radio);
		}
	}
	return ret;
}

static int set_txpower(const struct shell *sh, const char *value)
{
	long dbm;
	int ret;

	if (parse_long(sh, value, -40, 30, &dbm) != 0) {
		return -EINVAL;
	}
	ret = api->set_txpower(radio, (int16_t)dbm);
	if (ret == 0) {
		tx_power_dbm = (int16_t)dbm;
	}
	return ret;
}

static int set_promisc(const struct shell *sh, const char *value)
{
	struct ieee802154_config cfg;
	long on;
	int ret;

	if (parse_long(sh, value, 0, 1, &on) != 0) {
		return -EINVAL;
	}
	cfg.promiscuous = on != 0;
	ret = api->configure(radio, IEEE802154_CONFIG_PROMISCUOUS, &cfg);
	if (ret == 0) {
		promiscuous = cfg.promiscuous;
	}
	return ret;
}

/* PAN ID and short address go to the radio's filter together (what auto-ACK answers). */
static int apply_addr(void)
{
	struct ieee802154_filter f = {.pan_id = pan_id};
	int ret = api->filter(radio, true, IEEE802154_FILTER_TYPE_PAN_ID, &f);

	if (ret == 0) {
		f.short_addr = short_addr;
		ret = api->filter(radio, true, IEEE802154_FILTER_TYPE_SHORT_ADDR, &f);
	}
	return ret;
}

static int set_pan(const struct shell *sh, const char *value)
{
	long v;

	if (parse_long(sh, value, 0, 0xffff, &v) != 0) {
		return -EINVAL;
	}
	pan_id = (uint16_t)v;
	return apply_addr();
}

static int set_addr(const struct shell *sh, const char *value)
{
	long v;

	if (parse_long(sh, value, 0, 0xffff, &v) != 0) {
		return -EINVAL;
	}
	short_addr = (uint16_t)v;
	return apply_addr();
}

static int set_print(const struct shell *sh, const char *value)
{
	long level;

	if (parse_long(sh, value, 0, 2, &level) != 0) {
		return -EINVAL;
	}
	atomic_set(&rx_verbose, (atomic_val_t)level);
	return 0;
}

#if defined(CONFIG_ESP_SDR_IEEE802154)
static int set_rxgain(const struct shell *sh, const char *value)
{
	long g = ESP_SDR_RX_GAIN_AUTO;

	if (strcmp(value, "auto") != 0 && parse_long(sh, value, -1, 255, &g) != 0) {
		return -EINVAL;
	}
	return ieee154_esp_sdr_set_rx_gain((int)g);
}

static int set_invert(const struct shell *sh, const char *value, bool *flag)
{
	long on;

	if (parse_long(sh, value, 0, 1, &on) != 0) {
		return -EINVAL;
	}
	*flag = on != 0;
	ieee154_esp_sdr_set_invert(rx_invert, tx_invert);
	return 0;
}

static int set_rxinvert(const struct shell *sh, const char *value)
{
	return set_invert(sh, value, &rx_invert);
}

static int set_txinvert(const struct shell *sh, const char *value)
{
	return set_invert(sh, value, &tx_invert);
}

static int set_lo(const struct shell *sh, const char *value)
{
	int ret;

	if (strcmp(value, "above") != 0 && strcmp(value, "below") != 0) {
		shell_error(sh, "lo: above or below");
		return -EINVAL;
	}
	ret = ieee154_esp_sdr_set_lo_above(strcmp(value, "above") == 0);
	if (ret == 0) {
		lo_above = strcmp(value, "above") == 0;
	}
	return ret;
}

#if defined(CONFIG_ESP_SDR_IEEE802154_AUTO_ACK)
/* ack <delay us>[,<limit us>] */
static int set_ack(const struct shell *sh, const char *value)
{
	char *end;
	long delay = strtol(value, &end, 0), limit = 512;

	if (*end == ',') {
		limit = strtol(end + 1, &end, 0);
	}
	if (*end != '\0' || delay < 0 || limit < 0) {
		shell_error(sh, "ack: <delay us>[,<limit us>]");
		return -EINVAL;
	}
	return ieee154_esp_sdr_set_ack_timing((uint32_t)delay, (uint32_t)limit);
}
#endif

#if defined(CONFIG_ESP_SDR_IEEE802154_RING)
static int set_unit(const struct shell *sh, const char *value)
{
	long pairs;

	if (parse_long(sh, value, 0, 12288, &pairs) != 0) {
		return -EINVAL;
	}
	return ieee154_esp_sdr_set_ring_unit((uint32_t)pairs);
}
#endif
#endif

static const struct setting {
	const char *key, *help;
	int (*set)(const struct shell *sh, const char *value);
} settings[] = {
	{"chan", "11..26 (2405 + 5 (n - 11) MHz)", set_chan},
	{"txpower", "<dBm>", set_txpower},
	{"promisc", "0|1", set_promisc},
	{"pan", "<PAN ID>", set_pan},
	{"addr", "<short address>", set_addr},
	{"print", "0|1|2: nothing, a line, a line and a hex dump per frame", set_print},
#if defined(CONFIG_ESP_SDR_IEEE802154)
	{"rxgain", "auto|<index>", set_rxgain},
	{"rxinvert", "0|1: conjugate the received samples", set_rxinvert},
	{"txinvert", "0|1: conjugate the transmitted waveform", set_txinvert},
	{"lo", "below|above the channel (diagnostic)", set_lo},
#endif
#if defined(CONFIG_ESP_SDR_IEEE802154_AUTO_ACK)
	{"ack", "<delay us>[,<limit us>]: ACK start after the frame (192,512)", set_ack},
#endif
#if defined(CONFIG_ESP_SDR_IEEE802154_RING)
	{"unit", "<pairs>: ring unit length, 1024..12288 in steps of 64 (0: default, 1024)",
	 set_unit},
#endif
};

/* set [<key> <value> ...]: settings, applied in order; without arguments the keys. */
static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
	if (!radio_ok(sh)) {
		return -ENODEV;
	}
	if (argc == 1) {
		ARRAY_FOR_EACH(settings, k) {
			shell_print(sh, "  %-9s %s", settings[k].key, settings[k].help);
		}
		return 0;
	}
	if ((argc - 1U) % 2U != 0U) {
		shell_error(sh, "usage: set <key> <value> [<key> <value> ...]");
		return -EINVAL;
	}
	for (size_t a = 1; a + 1U < argc; a += 2U) {
		const struct setting *st = NULL;
		int ret;

		ARRAY_FOR_EACH(settings, k) {
			if (strcmp(argv[a], settings[k].key) == 0) {
				st = &settings[k];
			}
		}
		if (st == NULL) {
			shell_error(sh, "unknown key %s (wpan set lists them)", argv[a]);
			return -EINVAL;
		}
		ret = st->set(sh, argv[a + 1U]);
		shell_print(sh, "%s %s: %d", argv[a], argv[a + 1U], ret);
		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

/* tx <count, 0 = until stop> [interval_ms] [payload] [dst] [ack] [csma] */
static int cmd_tx(const struct shell *sh, size_t argc, char **argv)
{
	if (!radio_ok(sh)) {
		return -ENODEV;
	}
	uint32_t v[6] = {1U, 10U, 16U, BROADCAST, 0U, 0U};
	static const uint32_t max[6] = {UINT32_MAX, 60000U, PAYLOAD_MAX, 0xffffU, 1U, 1U};

	for (size_t k = 1; k < argc && k <= ARRAY_SIZE(v); k++) {
		if (parse_u32(sh, argv[k], max[k - 1], &v[k - 1]) != 0) {
			return -EINVAL;
		}
	}
	if (!atomic_cas(&tx_running, 0, 1)) {
		shell_error(sh, "tx running");
		return -EBUSY;
	}
	job.count = v[0];
	job.interval_ms = v[1];
	job.payload = MAX(v[2], 8U);
	job.dst = (uint16_t)v[3];
	job.ack = v[4] != 0U && job.dst != BROADCAST;
	job.csma = v[5] != 0U;
	atomic_set(&tx_stop, 0);
	k_sem_give(&tx_go);
	shell_print(sh, "tx %u frames, %u ms apart, psdu %u bytes, dst 0x%04x%s%s", job.count,
		    job.interval_ms, HDR_LEN + job.payload + FCS_LEN, job.dst,
		    job.ack ? ", ack" : "", job.csma ? ", csma" : "");
	return 0;
}

static int cmd_on(const struct shell *sh, size_t argc, char **argv)
{
	if (!radio_ok(sh)) {
		return -ENODEV;
	}
	int ret = api->start(radio);

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	radio_on = ret == 0;
	shell_print(sh, "radio on: %d", ret);
	return ret;
}

static int cmd_off(const struct shell *sh, size_t argc, char **argv)
{
	if (!radio_ok(sh)) {
		return -ENODEV;
	}
	int ret = api->stop(radio);

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	radio_on = false;
	shell_print(sh, "radio off: %d", ret);
	return ret;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	atomic_set(&tx_stop, 1);
	k_sem_give(&tx_wake);
	return 0;
}

#if defined(CONFIG_ESP_SDR_IEEE802154)
static void print_sdr_status(const struct shell *sh)
{
	struct ieee154_esp_sdr_stats s;
	struct esp_sdr_tx_dac_stats t;
	int gain = esp_sdr_rx_get_gain();

	ieee154_esp_sdr_get_stats(&s);
	esp_sdr_tx_dac_get_stats(&t);
	if (gain < 0) {
		shell_print(sh, "sdr: rxgain auto, rxinvert %d, txinvert %d, lo %s", rx_invert,
			    tx_invert, lo_above ? "above" : "below");
	} else {
		shell_print(sh, "sdr: rxgain %d, rxinvert %d, txinvert %d, lo %s", gain, rx_invert,
			    tx_invert, lo_above ? "above" : "below");
	}
	shell_print(sh,
		    "captures %u errors %u frames %u dropped %u tx %u tx errors %u filter %u us"
		    " phy %u us",
		    s.captures, s.capture_errors, s.frames, s.frames_dropped, s.tx_frames, s.tx_errors,
		    s.filter_us, s.phy_us);
	shell_print(sh, "ring status %u units %u lost pairs %u, phy read %u skipped %u",
		    s.ring_status, s.ring_units, s.ring_lost, s.phy_read, s.phy_skipped);
	shell_print(sh, "ring last run: %u.%02u cycles/pair, slice max %u cycles, abandoned %u",
		    s.ring_cycles_x100 / 100U, s.ring_cycles_x100 % 100U, s.ring_slice_max,
		    s.ring_abandoned);
	shell_print(sh, "ring irq windows %u, lines 0x%08x, late max %u pairs", s.ring_irq_windows,
		    s.ring_irq_lines, s.ring_late_max);
	shell_print(sh, "verdict after frame end: %u frames, min %u avg %u max %u us", s.verdict_n,
		    s.verdict_min_us, s.verdict_n ? s.verdict_sum_us / s.verdict_n : 0U,
		    s.verdict_max_us);
#if defined(CONFIG_ESP_SDR_IEEE802154_AUTO_ACK)
	{
		struct esp_sdr_ring_burst_stats b;

		esp_sdr_ring_burst_get_stats(&b);
		shell_print(sh,
			    "acks due %u fired %u arm failed %u fire failed %u cancelled %u"
			    " no dst %u",
			    s.ack_due, s.ack_fired, s.ack_arm_failed, s.ack_fire_failed,
			    s.ack_cancelled, s.ack_unsupported);
		shell_print(sh, "ack verdict after frame end: avg %u max %u us, %u late;"
			    " phy behind at the arm max %u us",
			    s.ack_fired ? s.ack_verdict_sum_us / s.ack_fired : 0U,
			    s.ack_verdict_max_us, s.ack_verdict_late, s.ack_lag_max_us);
		/* Ring pairs are 1/16 us. */
		shell_print(sh,
			    "bursts fired %u (late %u, front end %u) cancelled %u timeouts %u"
			    " skipped %u arm late %u; start after frame end min %d max %d last %d us",
			    b.fired, b.late, b.late_tx, b.cancelled, b.timeouts, b.skipped, b.arm_late,
			    b.fire_min_pairs / 16, b.fire_max_pairs / 16, b.fire_last_pairs / 16);
		shell_print(sh, "last burst after frame end: stop %d, tx ready %d, fire seen %d,"
			    " rx again %d us; dac pass %u cycles", b.stop_last_pairs / 16,
			    b.ready_last_pairs / 16, b.seen_last_pairs / 16,
			    b.rx_back_last_pairs / 16, b.dac_last_cyc);
		shell_print(sh, "bursts: tx ready max %d us, fire seen max %d us, wait poll max %u cycles,"
			    " waveform left at the stop max %u words",
			    b.ready_max_pairs / 16, b.seen_max_pairs / 16, b.window_max_cyc,
			    b.gen_left_max);
	}
#endif
	shell_print(sh, "acks %u, no ack %u, last tx error %d", s.acks, s.no_acks, s.tx_last_err);
	shell_print(sh,
		    "dac: switches %u errors %u restarts %u slack min %d us fill max %u us"
		    " wake late max %u us start margin %d us",
		    t.switches, t.errors, t.restarts, t.slack_us_min, t.fill_us_max,
		    t.wake_late_us_max, t.start_margin_us);
}
#endif

/* status: radio, settings and counters. */
static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	struct rx_stats s;
	k_spinlock_key_t key;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!radio_ok(sh)) {
		return -ENODEV;
	}
	key = k_spin_lock(&rx_lock);
	s = rx;
	k_spin_unlock(&rx_lock, key);
	shell_print(sh, "radio %s %s, caps 0x%x, chan %u (%u MHz), pan 0x%04x, addr 0x%04x",
		    radio->name, radio_on ? "on" : "off", (unsigned int)api->get_capabilities(radio),
		    channel, 2405U + 5U * (channel - 11U), pan_id, short_addr);
	if (tx_power_dbm == INT16_MIN) {
		shell_print(sh, "txpower default, promisc %d, print %d", promiscuous,
			    (int)atomic_get(&rx_verbose));
	} else {
		shell_print(sh, "txpower %d dBm, promisc %d, print %d", tx_power_dbm, promiscuous,
			    (int)atomic_get(&rx_verbose));
	}
	shell_print(sh, "rx frames %u test %u bad %u dups %u lost %u lqi %d last %u", s.frames,
		    s.test, s.bad, s.dups, s.lost, s.frames ? s.lqi_sum / (int32_t)s.frames : 0,
		    s.last_counter);
	if (s.rssi_n != 0U) {
		shell_print(sh, "rssi %d dBm (%d..%d)", s.rssi_sum / (int32_t)s.rssi_n, s.rssi_min,
			    s.rssi_max);
	} else {
		shell_print(sh, "rssi n/a");
	}
	shell_print(sh, "tx sent %u ok %u noack %u busy %u err %u%s", txr.sent, txr.ok, txr.noack,
		    txr.busy, txr.err, atomic_get(&tx_running) ? " (running)" : "");
#if defined(CONFIG_IEEE802154_ESP32)
	/* How long the native radio waits for an ACK (hardware register, 16 us units). */
	shell_print(sh, "ack timeout %u us", (unsigned int)esp_ieee802154_get_ack_timeout());
#endif
#if defined(CONFIG_ESP_SDR_IEEE802154)
	print_sdr_status(sh);
#endif
	return 0;
}

/* clear: zero the receive counters. */
static int cmd_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	rx_reset();
#if defined(CONFIG_ESP_SDR_IEEE802154)
	ieee154_esp_sdr_clear_stats();
#endif
	return 0;
}

#if defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
static int cmd_mem(const struct shell *sh, size_t argc, char **argv)
{
	extern struct k_heap _system_heap;
	struct sys_memory_stats hs;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (sys_heap_runtime_stats_get(&_system_heap.heap, &hs) != 0) {
		shell_error(sh, "no heap stats");
		return -EIO;
	}
	shell_print(sh, "system heap: %u allocated, %u free, %u peak",
		    (unsigned int)hs.allocated_bytes, (unsigned int)hs.free_bytes,
		    (unsigned int)hs.max_allocated_bytes);
	return 0;
}
#endif

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
static int cmd_crash(const struct shell *sh, size_t argc, char **argv)
{
	struct app_crash c;
	uint32_t cause = 0;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	(void)hwinfo_get_reset_cause(&cause);
	shell_print(sh, "reset cause 0x%x (zephyr/drivers/hwinfo.h RESET_*)", cause);
#if defined(CONFIG_ESP_SDR_IEEE802154)
	{
		static const char *const names[] = {"?", "take", "give", "ring", "ring end",
						    "tx", "tx end", "capture", "boot"};
		uint32_t ev[64];
		size_t n = ieee154_esp_sdr_get_trail(ev, ARRAY_SIZE(ev));

		for (size_t k = 0; k < n; k++) {
			uint32_t e = ev[k] >> 28;

			shell_print(sh, "  %7u ms cpu %u %-8s %u", ev[k] & 0xfffffU, (ev[k] >> 27) & 1U,
				    e < ARRAY_SIZE(names) ? names[e] : "?", (ev[k] >> 20) & 0x7fU);
		}
		uint32_t pc[16], ls[16];

		n = ieee154_esp_sdr_get_stall(pc, ls, ARRAY_SIZE(pc));
		for (size_t k = 0; k < n; k++) {
			shell_print(sh, "  stall: cpu0 pc 0x%08x ls 0x%08x", pc[k], ls[k]);
		}
	}
#endif
	{
		struct app_crash_dx dx;

		if (app_crash_dx_last(&dx)) {
			shell_print(sh,
				    "double exception: depc 0x%08x exccause %u excvaddr 0x%08x epc1 0x%08x"
				    " ps 0x%08x a1 0x%08x prid 0x%x",
				    dx.depc, dx.exccause, dx.excvaddr, dx.epc1, dx.ps, dx.a1, dx.prid);
		}
	}
	if (!app_crash_last(&c)) {
		shell_print(sh, "no crash record");
		return 0;
	}
	shell_print(sh,
		    "reason 0x%x thread %s cpu %u uptime %u ms pc 0x%08x a0 0x%08x ps 0x%08x"
		    " exccause %u excvaddr 0x%08x",
		    c.reason, c.thread, c.cpu, c.uptime_ms, c.pc, c.a0, c.ps, c.exccause,
		    c.excvaddr);
	return 0;
}
#endif

#if defined(CONFIG_ESP_SDR_RING)
static void bench_sink(void *user, const int16_t *i, const int16_t *q, size_t n, uint64_t index)
{
	ARG_UNUSED(user);
	ARG_UNUSED(i);
	ARG_UNUSED(q);
	ARG_UNUSED(n);
	ARG_UNUSED(index);
}

static int cmd_bench(const struct shell *sh, size_t argc, char **argv)
{
	const struct esp_sdr_ring_cfg cfg = {.decim = ESP_SDR_RING_DECIM_BOX4,
					     .shift_fs4 = true,
					     .sink = bench_sink,
					     .subsample = argc > 2 && atoi(argv[2]) != 0};
	struct esp_sdr_ring_bench_result r;
	int ret;

#if defined(CONFIG_ESP_SDR_IEEE802154_AUTO_ACK)
	if (argc > 1 && strcmp(argv[1], "acktx") == 0) {
		/* Not a bench: one ACK on the air while receiving (wpan on). */
		ret = ieee154_esp_sdr_ack_test(argc > 2 ? (uint8_t)atoi(argv[2]) : 0x5aU);
		shell_print(sh, "acktx: %d", ret);
		return ret;
	}
#endif
	if (radio_on || atomic_get(&tx_running)) {
		shell_error(sh, "wpan off and tx stopped first");
		return -EBUSY;
	}
#if defined(CONFIG_ESP_SDR_IEEE802154)
	if (argc > 1 && strcmp(argv[1], "ack") == 0) {
		struct ieee154_esp_sdr_bench b = {0};

		ret = ieee154_esp_sdr_bench(&b);
		shell_print(sh,
			    "ack %d: %u words, chips %u cycles, words %u cycles (from slot 416: %u)",
			    ret, b.ack_words, b.ack_build_cyc, b.ack_gen_cyc, b.ack_tail_cyc);
		shell_print(sh, "front end: to tx %u us, to rx %u us (settle times included)",
			    b.to_tx_us, b.to_rx_us);
		return ret;
	}
#endif
	if (argc > 1 && strcmp(argv[1], "phy") == 0) {
		/*
		 * The PHY alone on noise-like samples (hunting, the common case),
		 * in the transmit bank: idle and the CPUs' while the radio is off.
		 */
		size_t words;
		int16_t *buf = (int16_t *)esp_sdr_tx_loop_buf(1, &words);
		uint32_t x = 1, t0, cyc;

		if (buf == NULL || words < 4096U) {
			return -ENOMEM;
		}
		for (size_t k = 0; k < 2U * 4096U; k++) {
			x = x * 1664525U + 1013904223U;
			buf[k] = (int16_t)(x >> 20) - 2048;
		}
		t0 = k_cycle_get_32();
		for (int r = 0; r < 16; r++) {
			ieee154_phy_rx_block(buf, 4096);
		}
		cyc = k_cycle_get_32() - t0;
		shell_print(sh, "phy: %u cycles/sample (%u samples)", cyc / (16U * 4096U), 16U * 4096U);
		return 0;
	}
	ret = esp_sdr_ring_bench(&cfg, argc > 1 ? (unsigned int)atoi(argv[1]) : 8U, &r);
	shell_print(sh, "bench %d: %u pairs -> %u samples, %u.%02u cycles/pair", ret, r.pairs,
		    r.samples, r.cycles_x100 / 100U, r.cycles_x100 % 100U);
	shell_print(sh, "x100: unpack+rot %u, sums %u, sink %u", r.stage_x100[0], r.stage_x100[1],
		    r.stage_x100[3]);
	return ret;
}
#endif

#if defined(CONFIG_ESP_SDR_IEEE802154)
/* The capture as base64 of little-endian int16 I, Q pairs, 48 bytes per line. */
static void dump_out(const int16_t *iq, size_t n, void *user)
{
	const struct shell *sh = user;
	const uint8_t *p = (const uint8_t *)iq;
	size_t bytes = 4U * n;
	char line[72];

	shell_print(sh, "dump %u samples 4000000 S/s", (unsigned int)n);
	for (size_t at = 0; at < bytes; at += 48U) {
		size_t olen;

		(void)base64_encode((uint8_t *)line, sizeof(line), &olen, p + at,
				    MIN(48U, bytes - at));
		line[olen] = '\0';
		shell_print(sh, "b64 %s", line);
	}
}

/* measure: one 1 ms capture's carrier offset, power and chip phase steps. */
static int cmd_measure(const struct shell *sh, size_t argc, char **argv)
{
	struct ieee154_esp_sdr_measurement m = {0};
	const uint32_t *st = m.steps;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	ret = ieee154_esp_sdr_measure(&m);
	shell_print(sh, "measure %d: cfo %d Hz (coarse %d Hz), power %d dBFS, %u samples", ret,
		    m.cfo_hz, m.coarse_cfo_hz, m.power_dbfs, m.samples_used);
	shell_print(sh, "  chip steps from -180 by 45 deg: %u %u %u %u | %u %u %u %u", st[0], st[1],
		    st[2], st[3], st[4], st[5], st[6], st[7]);
	return ret;
}

/* dump: one 1 ms capture as the PHY sees it (tools/iq_dump.py). */
static int cmd_dump(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	ret = ieee154_esp_sdr_dump(dump_out, (void *)sh);
	shell_print(sh, "dump end %d", ret);
	return ret;
}
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(wpan_cmds,
	SHELL_CMD(status, NULL, "Radio, settings and counters", cmd_status),
	SHELL_CMD_ARG(set, NULL, "[<key> <value> ...]: settings (without arguments: the keys)",
		      cmd_set, 1, 16),
	SHELL_CMD(clear, NULL, "Zero the receive counters", cmd_clear),
	SHELL_CMD(on, NULL, "Start receiving", cmd_on),
	SHELL_CMD(off, NULL, "Stop receiving", cmd_off),
	SHELL_CMD_ARG(tx, NULL, "<count|0> [ms] [payload] [dst] [ack] [csma]: test frames",
		      cmd_tx, 2, 5),
	SHELL_CMD(stop, NULL, "Stop a running tx", cmd_stop),
#if defined(CONFIG_ESP_SDR_IEEE802154)
	SHELL_CMD(measure, NULL, "One 1 ms capture: carrier offset, power, chip phase steps",
		  cmd_measure),
	SHELL_CMD(dump, NULL, "One 1 ms capture as base64 I/Q (tools/iq_dump.py)", cmd_dump),
#endif
#if defined(CONFIG_ESP_SDR_RING)
	SHELL_CMD_ARG(bench, NULL,
		      "[units] [subsample] | phy | ack: ring decimation by 4, the PHY, or the ACK"
		      " waveform and front end",
		      cmd_bench, 1, 2),
#endif
#if defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
	SHELL_CMD(mem, NULL, "System heap use", cmd_mem),
#endif
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	SHELL_CMD(crash, NULL, "Last fatal error or watchdog reset", cmd_crash),
#endif
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(wpan, &wpan_cmds, "802.15.4 test frames", NULL);

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
/* The ring keeps CPU 1 busy with interrupts masked: the DFU download writes flash. */
void app_usb_dfu_prepare(void)
{
	/* A running tx job would start another frame, which holds the radio. */
	atomic_set(&tx_stop, 1);
	k_sem_give(&tx_wake);
	/* Let a frame in progress (and its ACK wait) finish before the radio stops. */
	for (int ms = 0; ms < 100 && atomic_get(&tx_running); ms++) {
		k_sleep(K_MSEC(1));
	}
	if (api != NULL) {
		(void)api->stop(radio);
	}
	radio_on = false;
}
#endif

int main(void)
{
	struct ieee802154_filter f;
	int ret;

	printk("wpan %s\n", APP_VERSION_STRING);
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	/* Before USB starts: its controller interrupt lands on the CPU that allocates it. */
	app_cpu_pin_system_threads(0);
	if (app_usb_init() != 0) {
		printk("wpan: usb init failed\n");
	}
#endif
#if !DT_HAS_CHOSEN(zephyr_ieee802154)
	radio = device_get_binding(RADIO_NAME);
#endif
#if defined(CONFIG_ESP_SDR_IEEE802154)
	ieee154_esp_sdr_set_rx_cb(rx_frame);
#endif
	if (radio == NULL || !device_is_ready(radio)) {
		printk("wpan: radio not ready\n");
		return 0;
	}
	api = radio->api;
	k_thread_create(&tx_thread, tx_stack, K_KERNEL_STACK_SIZEOF(tx_stack), tx_thread_fn, NULL,
			NULL, NULL, 7, 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread, "wpan_tx");

	f.pan_id = pan_id;
	(void)api->filter(radio, true, IEEE802154_FILTER_TYPE_PAN_ID, &f);
	f.short_addr = short_addr;
	(void)api->filter(radio, true, IEEE802154_FILTER_TYPE_SHORT_ADDR, &f);
	ret = api->set_channel(radio, channel);
	/* The software radio waits for "wpan on": a broken receive path cannot block updates. */
	if (ret == 0 && !IS_ENABLED(CONFIG_ESP_SDR_IEEE802154)) {
		ret = api->start(radio);
		radio_on = ret == 0;
	}
	printk("wpan: %s on channel %u, pan 0x%04x addr 0x%04x, %s: %d\n", radio->name, channel,
	       pan_id, short_addr, radio_on ? "receiving" : "off (wpan on)", ret);
	return 0;
}
