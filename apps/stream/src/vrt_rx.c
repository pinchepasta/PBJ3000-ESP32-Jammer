/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Receive stream: captured bursts as VITA 49.2 signal data (8, 12 or 16-bit
 * link efficient items) and context packets on start, on change and once per
 * second. Network order.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>

#include <vrt/vrt_init.h>
#include <vrt/vrt_read.h>
#include <vrt/vrt_string.h>
#include <vrt/vrt_types.h>
#include <vrt/vrt_util.h>
#include <vrt/vrt_words.h>
#include <vrt/vrt_write.h>

#include "iq_pack.h"
#include "vrt_rx.h"
#include "transport.h"

LOG_MODULE_DECLARE(stream, LOG_LEVEL_INF);

#define SAMPLES_PER_PACKET CONFIG_APP_RX_SAMPLES_PER_PACKET
/* Header, stream ID, TSI, two TSF words, trailer. */
#define DATA_OVERHEAD      6
#define PACKET_WORDS       (SAMPLES_PER_PACKET + DATA_OVERHEAD)
#define CONTEXT_PERIOD_MS  1000
#define SELFTEST_PACKETS   4

static ESP_SDR_HIGH_RAM uint32_t packet[PACKET_WORDS];
static ESP_SDR_HIGH_RAM uint32_t body[SAMPLES_PER_PACKET];
static uint8_t data_count, context_count;
static uint32_t context_freq_mhz, context_rate_hz, context_bits;
/* Item size of the signal data, 8, 12 or 16 bits; set by a VITA command. */
static atomic_t rx_bits = ATOMIC_INIT(16);
static int64_t context_ms;
static unsigned int selftest_left = SELFTEST_PACKETS;
static struct {
	uint32_t data, context, bytes, errors;
} stats;

static void split_time(uint64_t ns, struct vrt_fields *f)
{
	f->integer_seconds_timestamp = (uint32_t)(ns / NSEC_PER_SEC);
	f->fractional_seconds_timestamp = (ns % NSEC_PER_SEC) * 1000U; /* picoseconds */
}

static void init_header(struct vrt_packet *p, enum vrt_packet_type type, uint8_t count)
{
	vrt_init_packet(p);
	p->header.packet_type = type;
	p->header.packet_count = count & 0xfU;
	/* No absolute time source: seconds since boot, picosecond fraction. */
	p->header.tsi = VRT_TSI_OTHER;
	p->header.tsf = VRT_TSF_REAL_TIME;
	p->fields.stream_id = CONFIG_APP_RX_STREAM_ID;
}

/* Decode a few packets back with libvrt to check the encoder output. */
static void selftest(int32_t words)
{
	static ESP_SDR_HIGH_RAM uint32_t copy[PACKET_WORDS];
	struct vrt_packet r;
	int32_t rv;

	if (selftest_left == 0) {
		return;
	}
	selftest_left--;
	memcpy(copy, packet, (size_t)words * sizeof(uint32_t));
	vrt_from_big_endian(copy, words);
	rv = vrt_read_packet(copy, words, &r, true);
	if (rv != words || r.fields.stream_id != CONFIG_APP_RX_STREAM_ID) {
		LOG_ERR("vrt selftest failed (%d: %s)", rv,
			rv < 0 ? vrt_string_error(rv) : "size");
		stats.errors++;
		return;
	}
	LOG_DBG("vrt selftest %s packet ok, %d words",
		r.header.packet_type == VRT_PT_IF_CONTEXT ? "context" : "data", rv);
}

static void send(const struct vrt_packet *p)
{
	int32_t words = vrt_write_packet(p, packet, ARRAY_SIZE(packet), true);

	if (words < 0) {
		LOG_ERR("vrt write failed: %s", vrt_string_error(words));
		stats.errors++;
		return;
	}
	vrt_to_big_endian(packet, words);
	selftest(words);
	if (transport_send(packet, (size_t)words * sizeof(uint32_t)) != 0) {
		stats.errors++;
		return;
	}
	stats.bytes += (uint32_t)words * sizeof(uint32_t);
}

static void send_context(const struct vrt_rx_burst *b, unsigned int bits, bool changed)
{
	struct vrt_packet p;
	struct vrt_data_packet_payload_format *fmt = &p.if_context.data_packet_payload_format;

	init_header(&p, VRT_PT_IF_CONTEXT, context_count++);
	/* CIF1 is a 49.2 feature: declare compliance and mark the packet not 49.0. */
	p.header.nd0 = true;
	p.if_context.has.v49_spec_compliance = true;
	p.if_context.v49_spec_compliance = VRT_V49_2;
	split_time(b->timestamp_ns, &p.fields);
	p.if_context.context_field_change_indicator = changed;
	p.if_context.has.rf_reference_frequency = true;
	p.if_context.rf_reference_frequency = (double)b->freq_mhz * 1e6;
	p.if_context.has.sample_rate = true;
	p.if_context.sample_rate = (double)b->rate_hz;
	/* Complex sampling: the captured band is one sample rate wide. */
	p.if_context.has.bandwidth = true;
	p.if_context.bandwidth = (double)b->rate_hz;
	p.if_context.has.data_packet_payload_format = true;
	fmt->packing_method = VRT_PM_LINK_EFFICIENT;
	fmt->real_or_complex = VRT_ROC_COMPLEX_CARTESIAN;
	fmt->data_item_format = VRT_DIF_SIGNED_FIXED_POINT;
	/* Size fields are one less than the bit count. */
	fmt->item_packing_field_size = (uint8_t)(bits - 1U);
	fmt->data_item_size = (uint8_t)(bits - 1U);
	p.header.packet_size = (uint16_t)vrt_words_packet(&p);

	send(&p);
	stats.context++;
	context_freq_mhz = b->freq_mhz;
	context_rate_hz = b->rate_hz;
	context_bits = bits;
	context_ms = k_uptime_get();
}

void vrt_rx_burst(const struct vrt_rx_burst *b)
{
	unsigned int bits = (unsigned int)atomic_get(&rx_bits);
	uint32_t per_packet = (uint32_t)iq_samples_per_words(bits, SAMPLES_PER_PACKET);
	bool changed = b->freq_mhz != context_freq_mhz || b->rate_hz != context_rate_hz ||
		       bits != context_bits;

	if (changed || k_uptime_get() - context_ms >= CONTEXT_PERIOD_MS) {
		/* The burst's own item size: rx_bits may change meanwhile. */
		send_context(b, bits, changed);
	}

	for (uint32_t off = 0; off < b->count; off += per_packet) {
		uint32_t n = MIN(per_packet, b->count - off);
		bool clipped = false;
		struct vrt_packet p;
		size_t words;

		/* Whole quanta only (see iq_sample_quantum): the burst's last few may go. */
		n -= n % (uint32_t)iq_sample_quantum(bits);
		if (n == 0U) {
			break;
		}
		for (uint32_t j = 0; j < n; j++) {
			const struct esp_sdr_iq16 *s = &b->iq[off + j];

			clipped |= s->i >= b->full_scale - 1 || s->i <= -b->full_scale ||
				   s->q >= b->full_scale - 1 || s->q <= -b->full_scale;
		}
		/* Link efficient, I first: MSB first across words. */
		words = iq_pack(&b->iq[off], n, bits, b->full_scale, body);

		init_header(&p, VRT_PT_IF_DATA_WITH_STREAM_ID, data_count++);
		split_time(b->timestamp_ns + (uint64_t)off * NSEC_PER_SEC / b->rate_hz, &p.fields);
		p.header.has.trailer = true;
		p.body = body;
		p.words_body = (int32_t)words;
		p.trailer.has.valid_data = true;
		p.trailer.valid_data = true;
		p.trailer.has.agc_or_mgc = true;
		p.trailer.agc_or_mgc = b->gain < 0; /* true: AGC in charge */
		p.trailer.has.over_range = true;
		p.trailer.over_range = clipped;
		/* Bursts are not contiguous: flag the gap at each burst start. */
		p.trailer.has.sample_loss = true;
		p.trailer.sample_loss = off == 0;
		p.header.packet_size = (uint16_t)vrt_words_packet(&p);

		send(&p);
		stats.data++;
	}
}

int vrt_rx_set_bits(unsigned int bits)
{
	if (!iq_bits_valid(bits)) {
		return -EINVAL;
	}
	atomic_set(&rx_bits, (atomic_val_t)bits);
	return 0;
}

unsigned int vrt_rx_get_bits(void)
{
	return (unsigned int)atomic_get(&rx_bits);
}

int vrt_rx_init(void)
{
	return transport_init();
}

void vrt_rx_report(void)
{
	LOG_DBG("vrt %u data, %u context packets, %u kB, %u errors", stats.data, stats.context,
		stats.bytes / 1024U, stats.errors);
	memset(&stats, 0, sizeof(stats));
	transport_report();
}
