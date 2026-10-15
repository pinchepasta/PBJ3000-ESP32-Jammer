/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * VITA 49.2 from the host over UDP: signal data placed by timestamp into the
 * esp_sdr_tx streaming backend, and command packets that set the transmitter
 * or (by stream ID) the receiver, answered with acknowledges.
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>
#include <esp_sdr/esp_sdr_tx.h>

#include <vrt/vrt_init.h>
#include <vrt/vrt_read.h>
#include <vrt/vrt_types.h>
#include <vrt/vrt_util.h>
#include <vrt/vrt_words.h>
#include <vrt/vrt_write.h>

#include "iq_pack.h"
#include "vrt_rx.h"
#include "vrt_tx.h"

LOG_MODULE_DECLARE(stream, LOG_LEVEL_INF);

/* A 1500 byte datagram. */
#define RX_WORDS       375
#define ACK_WORDS      32
#define IDLE_STOP_MS   500
#define UNDERRUN_SLACK (20 * NSEC_PER_MSEC)
#define TX_DAC_HZ      40000000.0
#define TX_RATE_MAX_HZ (TX_DAC_HZ / 2)
#define STACK_SIZE     2048

static ESP_SDR_HIGH_RAM uint32_t rx_buf[RX_WORDS];
static uint32_t ack_buf[ACK_WORDS];
/* Room for a full datagram of 8-bit items, two samples per word. */
static ESP_SDR_HIGH_RAM struct esp_sdr_iq16 samples[2 * RX_WORDS];
/* Item size the host sends, 8, 12 or 16 bits; set by a VITA command. */
static unsigned int tx_bits = 16;
static struct vrt_packet pkt, ack;
static struct vrt_command cmd, ack_cmd;

static int sock = -1;
static uint64_t tx_freq_hz = (uint64_t)CONFIG_APP_FREQ_MHZ * 1000000U;
static uint32_t tx_rate_hz = CONFIG_APP_TX_RATE;
static uint8_t next_count;
static bool counted;
/* Next expected sample index from the packet timestamps, once one arrived. */
static uint64_t next_index;
static bool indexed;
static int64_t last_data_ms;
static int64_t pace_t0_ns;
static uint64_t pace_samples;
static struct vrt_tx_stats stats;

static K_THREAD_STACK_DEFINE(vrt_tx_stack, STACK_SIZE);
static struct k_thread tx_thread;

static int64_t now_ns(void)
{
	return (int64_t)k_cyc_to_ns_floor64(k_cycle_get_64());
}

static void restart(void)
{
	(void)esp_sdr_tx_stop();
	counted = false;
	indexed = false;
}

/* Self-paced backends schedule samples in real time: do not sleep the socket. */
static bool backend_paces_itself(void)
{
#if defined(CONFIG_ESP_SDR_TX_DAC)
	return esp_sdr_tx_get_backend() == &esp_sdr_tx_dac_backend;
#else
	return false;
#endif
}

/* Write zeros for count samples that never arrived. */
static void fill_hole(uint64_t count)
{
	stats.holes += (uint32_t)count;
	memset(samples, 0, sizeof(samples));
	while (count > 0U) {
		size_t n = (size_t)MIN(count, (uint64_t)ARRAY_SIZE(samples));

		if (esp_sdr_tx_write(samples, n) < 0) {
			stats.errors++;
		}
		count -= n;
	}
}

/*
 * Place the packet by its timestamp (sample time since the stream start):
 * missing packets become zero-filled holes, late or repeated samples are
 * dropped. Returns how many leading samples of the packet to skip.
 */
static size_t align_to_timestamp(size_t n)
{
	uint64_t index, ns;

	if (pkt.header.tsi == VRT_TSI_UNDEFINED || pkt.header.tsf != VRT_TSF_REAL_TIME) {
		return 0;
	}
	/* Seconds and fraction apart (ns is far finer than a sample) to stay in 64 bits. */
	ns = pkt.fields.fractional_seconds_timestamp / 1000U;
	index = (uint64_t)pkt.fields.integer_seconds_timestamp * tx_rate_hz +
		(ns * tx_rate_hz + NSEC_PER_SEC / 2U) / NSEC_PER_SEC;
	if (!indexed) {
		indexed = true;
		next_index = index;
	}
	if (index > next_index) {
		uint64_t gap = index - next_index;

		if (gap <= tx_rate_hz / 10U) {
			fill_hole(gap);
		}
		/* A longer jump restarts the schedule rather than sending silence. */
		next_index = index;
	} else if (index < next_index) {
		size_t skip = (size_t)MIN(next_index - index, (uint64_t)n);

		stats.late += (uint32_t)skip;
		next_index += n - skip;
		return skip;
	}
	next_index += n;
	return 0;
}

/* Link efficient complex, tx_bits per item, I first (see iq_unpack()). */
static void handle_data(void)
{
	int64_t expected, elapsed;
	size_t n;
	size_t skip;

	if (counted && pkt.header.packet_count != next_count) {
		stats.gaps++;
	}
	next_count = (pkt.header.packet_count + 1U) & 0xfU;
	counted = true;
	stats.data++;
	last_data_ms = k_uptime_get();

	if (!esp_sdr_tx_active()) {
		if (esp_sdr_tx_start(tx_freq_hz, tx_rate_hz) != 0) {
			stats.errors++;
			return;
		}
		pace_t0_ns = now_ns();
		pace_samples = 0;
	}

	BUILD_ASSERT(sizeof(struct esp_sdr_iq16) == 2 * sizeof(int16_t));
	n = iq_unpack(pkt.body, (size_t)pkt.words_body, tx_bits, (int16_t *)samples,
		      ARRAY_SIZE(samples));
	skip = align_to_timestamp(n);
	if (n > skip && esp_sdr_tx_write(samples + skip, n - skip) < 0) {
		stats.errors++;
	}
	if (backend_paces_itself()) {
		return;
	}

	/* Hold the sample clock: sleep when ahead, restart it after an underrun. */
	pace_samples += n;
	expected = (int64_t)(pace_samples * NSEC_PER_SEC / tx_rate_hz);
	elapsed = now_ns() - pace_t0_ns;
	if (expected > elapsed) {
		k_sleep(K_NSEC(expected - elapsed));
	} else if (elapsed - expected > UNDERRUN_SLACK) {
		stats.underruns++;
		pace_t0_ns = now_ns();
		pace_samples = 0;
	}
}

/* Item size from a payload format field: link efficient complex signed fixed point only. */
static int payload_bits(const struct vrt_data_packet_payload_format *f)
{
	unsigned int bits = (unsigned int)f->data_item_size + 1U;

	if (f->packing_method != VRT_PM_LINK_EFFICIENT ||
	    f->real_or_complex != VRT_ROC_COMPLEX_CARTESIAN ||
	    f->data_item_format != VRT_DIF_SIGNED_FIXED_POINT ||
	    f->item_packing_field_size != f->data_item_size || !iq_bits_valid(bits)) {
		return -EINVAL;
	}
	return (int)bits;
}

static void report_payload(struct vrt_if_context *c, unsigned int bits)
{
	struct vrt_data_packet_payload_format *f = &c->data_packet_payload_format;

	c->has.data_packet_payload_format = true;
	f->packing_method = VRT_PM_LINK_EFFICIENT;
	f->real_or_complex = VRT_ROC_COMPLEX_CARTESIAN;
	f->data_item_format = VRT_DIF_SIGNED_FIXED_POINT;
	f->item_packing_field_size = (uint8_t)(bits - 1U);
	f->data_item_size = (uint8_t)(bits - 1U);
}

static void send_ack(const struct sockaddr *to, socklen_t to_len, bool query,
		     const struct vrt_ack_responses *errors, bool executed, bool rx)
{
	int32_t words;

	vrt_init_packet(&ack);
	vrt_init_command(&ack_cmd);
	ack.command = &ack_cmd;
	ack.header.packet_type = VRT_PT_COMMAND;
	ack.header.acknowledge = true;
	ack.fields.stream_id = pkt.fields.stream_id;
	ack_cmd.message_id = cmd.message_id;
	ack_cmd.cam.controllee_enable = cmd.cam.controllee_enable;
	ack_cmd.cam.controllee_format = cmd.cam.controllee_format;
	ack_cmd.controllee_id = cmd.controllee_id;
	memcpy(ack_cmd.controllee_uuid, cmd.controllee_uuid, sizeof(cmd.controllee_uuid));
	ack_cmd.cam.controller_enable = cmd.cam.controller_enable;
	ack_cmd.cam.controller_format = cmd.cam.controller_format;
	ack_cmd.controller_id = cmd.controller_id;
	memcpy(ack_cmd.controller_uuid, cmd.controller_uuid, sizeof(cmd.controller_uuid));

	if (query) {
		/* Query-state acknowledge: the current TX settings as context fields. */
		ack_cmd.cam.query_state = true;
		ack.if_context.has.rf_reference_frequency = true;
		ack.if_context.has.sample_rate = true;
		if (rx) {
			int gain = esp_sdr_rx_get_gain();

			ack.if_context.rf_reference_frequency = esp_sdr_get_freq() * 1e6;
			ack.if_context.sample_rate = vrt_rx_get_rate();
			/* Uncalibrated: stage 1 carries the PHY gain table index, not dB. */
			ack.if_context.has.gain = gain >= 0;
			ack.if_context.gain.stage1 = (float)gain;
			report_payload(&ack.if_context, vrt_rx_get_bits());
		} else {
			ack.if_context.rf_reference_frequency = (double)tx_freq_hz;
			ack.if_context.sample_rate = (double)tx_rate_hz;
			/* Stage 1 carries the transmit power step, see esp_sdr_tx_set_gain(). */
			ack.if_context.has.gain = true;
			ack.if_context.gain.stage1 = (float)esp_sdr_tx_get_gain();
			report_payload(&ack.if_context, tx_bits);
		}
	} else {
		ack_cmd.cam.execution = true;
		ack_cmd.cam.scheduled_or_executed = executed;
		ack_cmd.cam.errors = cmd.cam.errors && errors != NULL;
		if (ack_cmd.cam.errors) {
			ack_cmd.errors = *errors;
		}
	}

	ack.header.packet_size = (uint16_t)vrt_words_packet(&ack);
	words = vrt_write_packet(&ack, ack_buf, ARRAY_SIZE(ack_buf), true);
	if (words < 0) {
		stats.errors++;
		return;
	}
	vrt_to_big_endian(ack_buf, words);
	if (zsock_sendto(sock, ack_buf, (size_t)words * sizeof(uint32_t), 0, to, to_len) < 0) {
		stats.errors++;
		return;
	}
	stats.acks++;
}

/*
 * Commands addressed to the receive stream (CONFIG_APP_RX_STREAM_ID) steer
 * the receiver: RF reference frequency in whole MHz, and gain, where stage 1
 * is the PHY gain table index (uncalibrated, not dB) and stage 2 must be 0.
 * A sample rate is only accepted if it is the stream's.
 */
static void handle_rx_command(const struct sockaddr *from, socklen_t from_len)
{
	struct vrt_ack_responses errors = {0};
	const struct vrt_if_context *c = &pkt.if_context;
	uint32_t mhz = esp_sdr_get_freq();
	uint32_t rate = vrt_rx_get_rate();
	int gain = esp_sdr_rx_get_gain();
	int bits = 0;
	bool any_error = false, executed = false;

	if (c->has.rf_reference_frequency) {
		double f = c->rf_reference_frequency / 1e6;

		if (f < ESP_SDR_FREQ_MIN_MHZ || f > ESP_SDR_FREQ_MAX_MHZ || f != floor(f)) {
			errors.has.rf_reference_frequency = true;
			errors.rf_reference_frequency = VRT_WEF_PARAMETER_OUT_OF_RANGE;
			any_error = true;
		} else {
			mhz = (uint32_t)f;
		}
	}
	if (c->has.gain) {
		float g = c->gain.stage1;

		if (c->gain.stage2 != 0.0f || g < 0.0f || g > (float)esp_sdr_rx_gain_max() ||
		    g != floorf(g)) {
			errors.has.gain = true;
			errors.gain = VRT_WEF_PARAMETER_OUT_OF_RANGE;
			any_error = true;
		} else {
			gain = (int)g;
		}
	}
	if (c->has.data_packet_payload_format) {
		bits = payload_bits(&c->data_packet_payload_format);
		if (bits < 0) {
			errors.has.data_packet_payload_format = true;
			errors.data_packet_payload_format = VRT_WEF_PARAMETER_OUT_OF_RANGE;
			any_error = true;
		}
	}
	if (c->has.sample_rate) {
		/* Capture rate (raw bursts) or capture rate / m, decimated on the board. */
		double r = c->sample_rate;
		uint32_t fs = esp_sdr_rx_rate_hz(CONFIG_APP_RX_RATE);

		if (r < 1.0 || r > fs || r != floor(r) || fs % (uint32_t)r != 0U ||
		    (fs / (uint32_t)r != 1U && (fs / (uint32_t)r < VRT_RX_DECIM_MIN ||
						 fs / (uint32_t)r > ESP_SDR_RX_DECIM_MAX))) {
			errors.has.sample_rate = true;
			errors.sample_rate = VRT_WEF_PARAMETER_OUT_OF_RANGE;
			any_error = true;
		} else {
			rate = (uint32_t)r;
		}
	}

	if (cmd.cam.action_mode == VRT_AM_EXECUTE && (!any_error || cmd.cam.permit_errors)) {
		if (mhz != esp_sdr_get_freq() && esp_sdr_set_freq(mhz) != 0) {
			stats.errors++;
		}
		if (c->has.gain && !errors.has.gain && esp_sdr_rx_set_gain(gain) != 0) {
			stats.errors++;
		}
		if (rate != vrt_rx_get_rate() && vrt_rx_set_rate(rate) != 0) {
			stats.errors++;
		}
		if (bits > 0 && vrt_rx_set_bits((unsigned int)bits) != 0) {
			stats.errors++;
		}
		executed = true;
	}

	if (cmd.cam.execution && (!cmd.cam.nack_only || any_error)) {
		send_ack(from, from_len, false, any_error ? &errors : NULL, executed, true);
	}
	if (cmd.cam.query_state) {
		send_ack(from, from_len, true, NULL, false, true);
	}
}

static void handle_command(const struct sockaddr *from, socklen_t from_len)
{
	struct vrt_ack_responses errors = {0};
	const struct vrt_if_context *c = &pkt.if_context;
	uint64_t freq = tx_freq_hz;
	uint32_t rate = tx_rate_hz;
	int gain = esp_sdr_tx_get_gain();
	int bits = 0;
	bool any_error = false, executed = false;

	stats.commands++;
	if (pkt.header.acknowledge || pkt.header.cancellation) {
		return;
	}
	if (pkt.fields.stream_id == CONFIG_APP_RX_STREAM_ID) {
		handle_rx_command(from, from_len);
		return;
	}

	if (c->has.rf_reference_frequency) {
		double f = c->rf_reference_frequency;

		/* Whole MHz: the transmit LO is the receiver's and tunes in MHz. */
		if (f < ESP_SDR_FREQ_MIN_MHZ * 1e6 || f > ESP_SDR_FREQ_MAX_MHZ * 1e6 ||
		    fmod(f, 1e6) != 0.0) {
			errors.has.rf_reference_frequency = true;
			errors.rf_reference_frequency = VRT_WEF_PARAMETER_OUT_OF_RANGE;
			any_error = true;
		} else {
			freq = (uint64_t)f;
		}
	}
	if (c->has.gain) {
		float g = c->gain.stage1;

		if (c->gain.stage2 != 0.0f || g < 0.0f || g > (float)esp_sdr_tx_gain_max() ||
		    g != floorf(g)) {
			errors.has.gain = true;
			errors.gain = VRT_WEF_PARAMETER_OUT_OF_RANGE;
			any_error = true;
		} else {
			gain = (int)g;
		}
	}
	if (c->has.data_packet_payload_format) {
		bits = payload_bits(&c->data_packet_payload_format);
		if (bits < 0) {
			errors.has.data_packet_payload_format = true;
			errors.data_packet_payload_format = VRT_WEF_PARAMETER_OUT_OF_RANGE;
			any_error = true;
		}
	}
	if (c->has.sample_rate) {
		double r = c->sample_rate;

		/* The DAC backend interpolates by an integer factor of at least 2. */
		if (r < 1.0 || r > TX_RATE_MAX_HZ || fmod(TX_DAC_HZ, r) != 0.0) {
			errors.has.sample_rate = true;
			errors.sample_rate = VRT_WEF_PARAMETER_OUT_OF_RANGE;
			any_error = true;
		} else {
			rate = (uint32_t)r;
		}
	}

	if (cmd.cam.action_mode == VRT_AM_EXECUTE && (!any_error || cmd.cam.permit_errors)) {
		bool changed = freq != tx_freq_hz || rate != tx_rate_hz ||
			       gain != esp_sdr_tx_get_gain();

		tx_freq_hz = freq;
		tx_rate_hz = rate;
		/* Applied when the next transmission switches the front end. */
		if (esp_sdr_tx_set_gain(gain) != 0) {
			stats.errors++;
		}
		if (bits > 0) {
			tx_bits = (unsigned int)bits;
		}
		if (changed && esp_sdr_tx_active()) {
			restart();
		}
		executed = true;
	}

	if (cmd.cam.execution && (!cmd.cam.nack_only || any_error)) {
		send_ack(from, from_len, false, any_error ? &errors : NULL, executed, false);
	}
	if (cmd.cam.query_state) {
		send_ack(from, from_len, true, NULL, false, false);
	}
}

static void tx_loop(void *p1, void *p2, void *p3)
{
	struct sockaddr_in6 from;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		socklen_t from_len = sizeof(from);
		ssize_t len = zsock_recvfrom(sock, rx_buf, sizeof(rx_buf), 0,
					     (struct sockaddr *)&from, &from_len);
		int32_t words, rv;

		if (len < 0) {
			/* Receive timeout: stop after a pause in the sample stream. */
			if (esp_sdr_tx_active() && k_uptime_get() - last_data_ms > IDLE_STOP_MS) {
				restart();
			}
			continue;
		}
		if (len % sizeof(uint32_t) != 0) {
			stats.bad++;
			continue;
		}
		words = (int32_t)(len / sizeof(uint32_t));
		vrt_from_big_endian(rx_buf, words);
		vrt_init_packet(&pkt);
		vrt_init_command(&cmd);
		pkt.command = &cmd;
		rv = vrt_read_packet(rx_buf, words, &pkt, true);
		if (rv < 0) {
			stats.bad++;
			continue;
		}
		if (vrt_is_data(&pkt.header)) {
			handle_data();
		} else if (vrt_is_command(&pkt.header)) {
			handle_command((struct sockaddr *)&from, from_len);
		} else {
			stats.bad++;
		}
	}
}

int vrt_tx_init(void)
{
	struct sockaddr_in6 addr = {
		.sin6_family = AF_INET6,
		.sin6_port = htons(CONFIG_APP_TX_UDP_PORT),
		.sin6_addr = IN6ADDR_ANY_INIT,
	};
	struct zsock_timeval timeout = {.tv_usec = 100000};

	sock = zsock_socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		return -errno;
	}
	if (zsock_bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
	    zsock_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
		return -errno;
	}
	k_thread_create(&tx_thread, vrt_tx_stack, K_THREAD_STACK_SIZEOF(vrt_tx_stack), tx_loop,
			NULL, NULL, NULL, K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread, "vrt_tx");
	LOG_INF("tx: VITA 49.2 on UDP port %d", CONFIG_APP_TX_UDP_PORT);
	return 0;
}

void vrt_tx_get_stats(struct vrt_tx_stats *out)
{
	*out = stats;
}

void vrt_tx_get_settings(uint64_t *freq_hz, uint32_t *rate_hz)
{
	*freq_hz = tx_freq_hz;
	*rate_hz = tx_rate_hz;
}
