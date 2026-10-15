/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Host tool for the osmosdr app: device info, USB throughput tests and a
 * receive stream to a file. libusb with asynchronous bulk transfers, the
 * way libhackrf streams.
 *
 *   cc -O2 -Wall -I../src -I<libusb include> esdr_usb.c -lusb-1.0 -o esdr_usb
 *
 *   esdr_usb [-s serial] info
 *   esdr_usb [-s serial] test-in  [-t seconds]
 *   esdr_usb [-s serial] test-out [-t seconds]
 *   esdr_usb [-s serial] rx [-f Hz] [-r rate] [-g gain|auto] [-d dB] [-b 8|16] [-B bw Hz]
 *                          [-p ppb] [-n] [-t seconds] [-o file]
 *
 *   esdr_usb [-s serial] tx [-f Hz] [-r rate] [-G step] [-b 8|16] [-p ppb] [-t seconds]
 *                          (-i file | -T tone Hz [-a amplitude 0..1])
 *
 * rx writes interleaved I/Q (cs8 or cs16, little endian) with the samples a
 * gap lost replaced by zeros, so the file keeps the real time axis.
 */

#include <errno.h>
#include <inttypes.h>
#include <libusb.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "esdr_proto.h"

#define VID 0x2fe3
#define PID 0x0005

#define XFERS      8
#define XFER_BYTES (16 * ESDR_BLOCK_BYTES)
#define CTRL_MS    1000

static libusb_device_handle *dev;
static uint8_t ep_in, ep_out;
static volatile sig_atomic_t stop;
/* rx: retune by this much once, half way through (restart test). */
static double retune_step;
static uint64_t retune_base;

static double now(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void on_sigint(int sig)
{
	(void)sig;
	stop = 1;
}

static int open_device(const char *serial)
{
	libusb_device **list;
	ssize_t n = libusb_get_device_list(NULL, &list);
	int ret = -ENODEV;

	for (ssize_t i = 0; i < n && dev == NULL; i++) {
		struct libusb_device_descriptor dd;
		struct libusb_config_descriptor *cfg;
		libusb_device_handle *h;
		char sn[64] = "";

		if (libusb_get_device_descriptor(list[i], &dd) != 0 || dd.idVendor != VID ||
		    dd.idProduct != PID || libusb_open(list[i], &h) != 0) {
			continue;
		}
		if (dd.iSerialNumber != 0) {
			libusb_get_string_descriptor_ascii(h, dd.iSerialNumber, (uint8_t *)sn,
							   sizeof(sn));
		}
		if (serial != NULL && strcmp(serial, sn) != 0) {
			libusb_close(h);
			continue;
		}
		if (libusb_get_active_config_descriptor(list[i], &cfg) != 0) {
			libusb_close(h);
			continue;
		}
		/* The vendor interface with a bulk endpoint each way. */
		for (int k = 0; k < cfg->bNumInterfaces && dev == NULL; k++) {
			const struct libusb_interface_descriptor *id = &cfg->interface[k].altsetting[0];

			if (id->bInterfaceClass != LIBUSB_CLASS_VENDOR_SPEC || id->bNumEndpoints != 2) {
				continue;
			}
			for (int e = 0; e < 2; e++) {
				uint8_t a = id->endpoint[e].bEndpointAddress;

				if (a & LIBUSB_ENDPOINT_IN) {
					ep_in = a;
				} else {
					ep_out = a;
				}
			}
			if (libusb_claim_interface(h, id->bInterfaceNumber) == 0) {
				dev = h;
				fprintf(stderr, "device %s, interface %d, in 0x%02x, out 0x%02x\n", sn,
					id->bInterfaceNumber, ep_in, ep_out);
				ret = 0;
			}
		}
		libusb_free_config_descriptor(cfg);
		if (dev == NULL) {
			libusb_close(h);
		}
	}
	libusb_free_device_list(list, 1);
	return ret;
}

static int ctrl_out(uint8_t req, uint16_t value, const void *data, uint16_t len)
{
	int r = libusb_control_transfer(dev,
					LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
						LIBUSB_RECIPIENT_DEVICE,
					req, value, 0, (unsigned char *)data, len, CTRL_MS);

	if (r < 0) {
		fprintf(stderr, "request 0x%02x: %s\n", req, libusb_error_name(r));
		return r;
	}
	return 0;
}

static int ctrl_in(uint8_t req, void *data, uint16_t len)
{
	int r = libusb_control_transfer(dev,
					LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR |
						LIBUSB_RECIPIENT_DEVICE,
					req, 0, 0, data, len, CTRL_MS);

	if (r < 0) {
		fprintf(stderr, "request 0x%02x: %s\n", req, libusb_error_name(r));
	}
	return r;
}

static int set_u32(uint8_t req, uint32_t v)
{
	uint8_t b[4] = {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};

	return ctrl_out(req, 0, b, 4);
}

int set_freq(uint64_t hz);
int set_freq(uint64_t hz)
{
	uint8_t b[8];

	for (int i = 0; i < 8; i++) {
		b[i] = (uint8_t)(hz >> (8 * i));
	}
	return ctrl_out(ESDR_REQ_SET_FREQ, 0, b, 8);
}

static void print_state(void)
{
	struct esdr_state st;
	struct esdr_stats s;

	if (ctrl_in(ESDR_REQ_GET_STATE, &st, sizeof(st)) == sizeof(st)) {
		printf("state: mode %u, %u bit, %u S/s, centre %" PRIu64 " Hz (actual %" PRIu64
		       "), gain %u, digital %u dB, bw %u Hz, corr %d ppb, LO %u MHz %+d kHz, opt 0x%x\n",
		       st.mode, st.bits, st.sample_rate_hz, st.freq_hz, st.center_hz, st.rx_gain,
		       st.digital_gain_db, st.bandwidth_hz, st.freq_corr_ppb, st.lo_mhz,
		       st.lo_offset_khz, st.options);
	}
	if (ctrl_in(ESDR_REQ_GET_STATS, &s, sizeof(s)) == sizeof(s)) {
		printf("stats: rx %" PRIu64 " samples, %u blocks, overflow %" PRIu64
		       " samples (%u times), ring lost %" PRIu64 " pairs, restart gaps %" PRIu64 "\n",
		       s.rx_samples, s.rx_blocks, s.rx_overflow_samples, s.rx_overflows,
		       s.rx_ring_lost_pairs, s.rx_restart_samples);
		printf("ring: %u runs, %u errors (last %u, 0x%08x), %u abandoned, late max %u, "
		       "%.2f cycles/pair, gain re-forced %u\n",
		       s.ring_runs, s.ring_errors, s.ring_last_status, s.ring_last_detail,
		       s.ring_abandoned, s.ring_late_max, s.ring_cycles_x100 / 100.0,
		       s.ring_gain_refreshed);
		printf("usb: %" PRIu64 " bytes in, %" PRIu64 " bytes out\n", s.usb_in_bytes,
		       s.usb_out_bytes);
		printf("tx: %u sessions, %" PRIu64 " played, %" PRIu64 " dropped, %u underruns, "
		       "%u DAC restarts, backlog max %u\n",
		       s.tx_sessions, s.tx_samples, s.tx_dropped, s.tx_underruns, s.tx_restarts,
		       s.tx_queued_max);
	}
}

static int cmd_info(void)
{
	struct esdr_info in;

	if (ctrl_in(ESDR_REQ_GET_INFO, &in, sizeof(in)) != sizeof(in) || in.magic != ESDR_INFO_MAGIC) {
		fprintf(stderr, "bad info\n");
		return 1;
	}
	printf("%.32s, protocol %u, %" PRIu64 "..%" PRIu64 " Hz, rx %u S/s / 2^0..%u, "
	       "rx gain 0..%u, tx gain 0..%u, block %u bytes\n",
	       in.firmware, in.proto_version, in.freq_min_hz, in.freq_max_hz, in.rx_rate_max_hz,
	       in.rx_rate_count - 1, in.rx_gain_max, in.tx_gain_max, in.block_bytes);
	print_state();
	return 0;
}

/* ---- streaming ---- */

struct stream {
	FILE *out;
	bool test;
	uint64_t bytes, blocks, samples, gap_samples, gaps, overflows, bad;
	uint64_t next_index;
	bool have_index;
	int bits;
	unsigned int pending;
	/* Block alignment: lost when a previous session stopped mid-block. */
	bool synced;
	uint64_t resyncs;
	uint8_t carry[ESDR_BLOCK_BYTES];
	size_t carry_len;
	/* tx source: a file (looped), or a tone. */
	FILE *in;
	double tone_hz, amp, phase, rate;
};

/* Fill a transmit buffer from the source. */
static void tx_fill(struct stream *s, uint8_t *buf, int len)
{
	int item = s->bits == 16 ? 4 : 2;

	if (s->in != NULL) {
		int got = 0;

		while (got < len) {
			size_t r = fread(buf + got, 1, (size_t)(len - got), s->in);

			if (r == 0) {
				rewind(s->in);
				if (fread(buf + got, 1, 1, s->in) == 0) {
					memset(buf + got, 0, (size_t)(len - got));
					return;
				}
				got++;
				continue;
			}
			got += (int)r;
		}
		return;
	}
	for (int k = 0; k + item <= len; k += item) {
		double fi = s->amp * cos(s->phase), fq = s->amp * sin(s->phase);

		s->phase += 2.0 * M_PI * s->tone_hz / s->rate;
		if (s->phase > M_PI) {
			s->phase -= 2.0 * M_PI;
		} else if (s->phase < -M_PI) {
			s->phase += 2.0 * M_PI;
		}
		if (item == 4) {
			int16_t vi = (int16_t)lrint(fi * 32767.0), vq = (int16_t)lrint(fq * 32767.0);

			buf[k] = (uint8_t)vi;
			buf[k + 1] = (uint8_t)(vi >> 8);
			buf[k + 2] = (uint8_t)vq;
			buf[k + 3] = (uint8_t)(vq >> 8);
		} else {
			buf[k] = (uint8_t)(int8_t)lrint(fi * 127.0);
			buf[k + 1] = (uint8_t)(int8_t)lrint(fq * 127.0);
		}
	}
}

static void parse_block(struct stream *s, const uint8_t *b)
{
	struct esdr_block_hdr h;
	size_t item = s->bits == 16 ? 4 : 2;

	memcpy(&h, b, sizeof(h));
	s->blocks++;
	if (s->test) {
		if (s->have_index && h.index != s->next_index) {
			s->gaps++;
		}
		s->next_index = h.index + 1;
		s->have_index = true;
		return;
	}
	if (h.flags & ESDR_BLK_OVERFLOW) {
		s->overflows++;
	}
	if (s->have_index && h.index != s->next_index) {
		uint64_t missing = h.index - s->next_index;

		s->gaps++;
		if (h.index > s->next_index && missing < 100000000ULL) {
			s->gap_samples += missing;
			if (s->out != NULL) {
				static const uint8_t zeros[4096];

				for (uint64_t z = missing * item; z > 0;) {
					size_t k = z > sizeof(zeros) ? sizeof(zeros) : (size_t)z;

					fwrite(zeros, 1, k, s->out);
					z -= k;
				}
			}
		}
	}
	s->next_index = h.index + h.samples;
	s->have_index = true;
	s->samples += h.samples;
	if (s->out != NULL) {
		fwrite(b + sizeof(h), item, h.samples, s->out);
	}
}

/* A plausible block header (the magic alone could occur in sample data). */
static bool header_ok(const uint8_t *b)
{
	struct esdr_block_hdr h;

	memcpy(&h, b, sizeof(h));
	return h.magic == ESDR_BLOCK_MAGIC && h.samples <= ESDR_BLOCK_PAYLOAD / 2U &&
	       (h.flags & ~0x000fU) == 0U;
}

/* Bulk data in any chunking: whole blocks out, the device's 64-byte packets as the search grid. */
static void parse_bytes(struct stream *s, const uint8_t *p, size_t n)
{
	while (n > 0) {
		if (!s->synced) {
			if (n < 64) {
				return;
			}
			if (n >= sizeof(struct esdr_block_hdr) && header_ok(p)) {
				s->synced = true;
				s->carry_len = 0;
				continue;
			}
			s->resyncs++;
			p += 64;
			n -= 64;
			continue;
		}
		if (s->carry_len > 0 || n < ESDR_BLOCK_BYTES) {
			size_t k = ESDR_BLOCK_BYTES - s->carry_len;

			k = k < n ? k : n;
			memcpy(s->carry + s->carry_len, p, k);
			s->carry_len += k;
			p += k;
			n -= k;
			if (s->carry_len < ESDR_BLOCK_BYTES) {
				return;
			}
			s->carry_len = 0;
			if (!header_ok(s->carry)) {
				s->bad++;
				s->synced = false;
				continue;
			}
			parse_block(s, s->carry);
			continue;
		}
		if (!header_ok(p)) {
			s->bad++;
			s->synced = false;
			continue;
		}
		parse_block(s, p);
		p += ESDR_BLOCK_BYTES;
		n -= ESDR_BLOCK_BYTES;
	}
}

static void LIBUSB_CALL in_cb(struct libusb_transfer *t)
{
	struct stream *s = t->user_data;

	/* A cancelled transfer may still hold data: never drop what arrived. */
	if (t->status == LIBUSB_TRANSFER_COMPLETED || t->status == LIBUSB_TRANSFER_CANCELLED ||
	    t->status == LIBUSB_TRANSFER_TIMED_OUT) {
		s->bytes += (uint64_t)t->actual_length;
		parse_bytes(s, t->buffer, (size_t)t->actual_length);
	}
	if (t->status != LIBUSB_TRANSFER_COMPLETED && t->status != LIBUSB_TRANSFER_CANCELLED &&
	    t->status != LIBUSB_TRANSFER_TIMED_OUT) {
		fprintf(stderr, "in transfer: status %d\n", t->status);
		stop = 1;
	}
	if (!stop && libusb_submit_transfer(t) == 0) {
		return;
	}
	s->pending--;
}

static void LIBUSB_CALL out_cb(struct libusb_transfer *t)
{
	struct stream *s = t->user_data;

	if (t->status == LIBUSB_TRANSFER_COMPLETED) {
		s->bytes += (uint64_t)t->actual_length;
	} else if (t->status != LIBUSB_TRANSFER_CANCELLED) {
		fprintf(stderr, "out transfer: status %d\n", t->status);
		stop = 1;
	}
	if (!s->test) {
		tx_fill(s, t->buffer, t->length);
	}
	if (!stop && libusb_submit_transfer(t) == 0) {
		return;
	}
	s->pending--;
}

static int run_stream(struct stream *s, bool in, double seconds, double rate)
{
	struct libusb_transfer *x[XFERS];
	double t0, last, tl;
	uint64_t last_bytes = 0, last_samples = 0;

	for (int k = 0; k < XFERS; k++) {
		uint8_t *buf = calloc(1, XFER_BYTES);

		x[k] = libusb_alloc_transfer(0);
		/*
		 * No timeout on IN: queued transfers fill one after another, so one
		 * counted from submission expires on the later ones at low rates.
		 */
		libusb_fill_bulk_transfer(x[k], dev, in ? ep_in : ep_out, buf, XFER_BYTES,
					  in ? in_cb : out_cb, s, in ? 0 : 2000);
		if (!in && !s->test) {
			tx_fill(s, buf, XFER_BYTES);
		}
		if (libusb_submit_transfer(x[k]) == 0) {
			s->pending++;
		}
	}
	t0 = last = now();
	while (!stop && now() - t0 < seconds) {
		struct timeval tv = {0, 100000};

		libusb_handle_events_timeout(NULL, &tv);
		tl = now();
		if (retune_step != 0 && tl - t0 >= seconds / 2) {
			int set_freq(uint64_t hz);

			set_freq(retune_base + (uint64_t)retune_step);
			fprintf(stderr, "retuned by %.0f Hz\n", retune_step);
			retune_step = 0;
		}
		if (tl - last >= 1.0) {
			fprintf(stderr, "%6.1f s: %7.1f kB/s", tl - t0,
				(double)(s->bytes - last_bytes) / (tl - last) / 1e3);
			if (in && !s->test) {
				double r = (double)(s->samples - last_samples) / (tl - last);

				fprintf(stderr, ", %8.0f S/s (%.1f %%), gaps %" PRIu64 ", overflows %" PRIu64,
					r, rate > 0 ? 100.0 * r / rate : 0.0, s->gaps, s->overflows);
			} else if (in) {
				fprintf(stderr, ", %" PRIu64 " blocks, %" PRIu64 " gaps", s->blocks, s->gaps);
			} else if (!s->test) {
				double r = (double)(s->bytes - last_bytes) / (tl - last) /
					   (s->bits == 16 ? 4 : 2);

				fprintf(stderr, ", %8.0f S/s (%.1f %%)", r, 100.0 * r / rate);
			}
			fprintf(stderr, "\n");
			last = tl;
			last_bytes = s->bytes;
			last_samples = s->samples;
		}
	}
	stop = 1;
	for (int k = 0; k < XFERS; k++) {
		libusb_cancel_transfer(x[k]);
	}
	while (s->pending > 0) {
		struct timeval tv = {0, 100000};

		libusb_handle_events_timeout(NULL, &tv);
	}
	tl = now() - t0;
	fprintf(stderr, "total %.1f s: %.1f kB/s", tl, (double)s->bytes / tl / 1e3);
	if (in && !s->test) {
		fprintf(stderr, ", %" PRIu64 " samples (%.0f S/s, %.2f %% of %.0f), %" PRIu64
			" gaps (%" PRIu64 " samples), %" PRIu64 " overflow flags, %" PRIu64
			" bad blocks, %" PRIu64 " resync steps",
			s->samples, (double)s->samples / tl, 100.0 * (double)s->samples / tl / rate,
			rate, s->gaps, s->gap_samples, s->overflows, s->bad, s->resyncs);
	} else if (in) {
		fprintf(stderr, ", %" PRIu64 " blocks, %" PRIu64 " sequence gaps, %" PRIu64
			" bad, %" PRIu64 " resync steps", s->blocks, s->gaps, s->bad, s->resyncs);
	}
	fprintf(stderr, "\n");
	for (int k = 0; k < XFERS; k++) {
		free(x[k]->buffer);
		libusb_free_transfer(x[k]);
	}
	return 0;
}

static void usage(void)
{
	fprintf(stderr, "usage: esdr_usb [-s serial] info | test-in | test-out | rx | tx\n"
			"  [-t seconds] [-f Hz] [-r S/s] [-g gain|auto] [-d digital dB] [-b 8|16] [-B bw Hz]\n"
			"  [-p ppb] [-n (no offset tuning)] [-o file] [-x Hz (retune half way)]\n"
			"  tx: [-G step] (-i file | -T tone Hz [-a amplitude])\n");
}

int main(int argc, char **argv)
{
	const char *serial = NULL, *outfile = NULL, *cmd;
	double seconds = 10, freq = 2450e6;
	uint32_t rate = 250000, bw = 13000000;
	int gain = -1, bits = 8, ppb = 0, dgain = -1, txgain = 4, opt;
	const char *infile = NULL;
	double tone = 0, amp = 0.5;
	bool offset = true;
	struct stream s = {0};
	int ret = 0;

	while ((opt = getopt(argc, argv, "s:t:f:r:g:b:B:p:no:x:d:G:i:T:a:h")) != -1) {
		switch (opt) {
		case 's':
			serial = optarg;
			break;
		case 't':
			seconds = atof(optarg);
			break;
		case 'f':
			freq = atof(optarg);
			break;
		case 'r':
			rate = (uint32_t)atof(optarg);
			break;
		case 'g':
			gain = strcmp(optarg, "auto") == 0 ? -1 : atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'B':
			bw = (uint32_t)atof(optarg);
			break;
		case 'p':
			ppb = atoi(optarg);
			break;
		case 'n':
			offset = false;
			break;
		case 'o':
			outfile = optarg;
			break;
		case 'x':
			retune_step = atof(optarg);
			break;
		case 'd':
			dgain = atoi(optarg);
			break;
		case 'G':
			txgain = atoi(optarg);
			break;
		case 'i':
			infile = optarg;
			break;
		case 'T':
			tone = atof(optarg);
			break;
		case 'a':
			amp = atof(optarg);
			break;
		default:
			usage();
			return 2;
		}
	}
	if (optind >= argc) {
		usage();
		return 2;
	}
	cmd = argv[optind];
	signal(SIGINT, on_sigint);
	if (libusb_init(NULL) != 0 || open_device(serial) != 0) {
		fprintf(stderr, "no device %04x:%04x%s%s\n", VID, PID, serial ? " serial " : "",
			serial ? serial : "");
		return 1;
	}

	if (strcmp(cmd, "info") == 0) {
		ret = cmd_info();
	} else if (strcmp(cmd, "test-in") == 0 || strcmp(cmd, "test-out") == 0) {
		bool in = cmd[5] == 'i';

		s.test = true;
		ctrl_out(ESDR_REQ_SET_MODE, ESDR_MODE_OFF, NULL, 0);
		ctrl_out(ESDR_REQ_SET_MODE, in ? ESDR_MODE_TEST_IN : ESDR_MODE_TEST_OUT, NULL, 0);
		run_stream(&s, in, seconds, 0);
		ctrl_out(ESDR_REQ_SET_MODE, ESDR_MODE_OFF, NULL, 0);
		print_state();
	} else if (strcmp(cmd, "rx") == 0) {
		struct esdr_state st;

		s.bits = bits;
		if (outfile != NULL) {
			s.out = strcmp(outfile, "-") == 0 ? stdout : fopen(outfile, "wb");
			if (s.out == NULL) {
				perror(outfile);
				return 1;
			}
		}
		ctrl_out(ESDR_REQ_SET_MODE, ESDR_MODE_OFF, NULL, 0);
		set_freq((uint64_t)freq);
		retune_base = (uint64_t)freq;
		set_u32(ESDR_REQ_SET_SAMPLE_RATE, rate);
		set_u32(ESDR_REQ_SET_BANDWIDTH, bw);
		set_u32(ESDR_REQ_SET_FREQ_CORR, (uint32_t)ppb);
		ctrl_out(ESDR_REQ_SET_RX_GAIN, gain < 0 ? ESDR_GAIN_AUTO : (uint16_t)gain, NULL, 0);
		ctrl_out(ESDR_REQ_SET_FORMAT, (uint16_t)bits, NULL, 0);
		ctrl_out(ESDR_REQ_SET_DIGITAL_GAIN, dgain < 0 ? ESDR_DGAIN_DEFAULT : (uint16_t)dgain,
			 NULL, 0);
		ctrl_out(ESDR_REQ_SET_OPTIONS, offset ? ESDR_OPT_OFFSET_TUNING : 0, NULL, 0);
		if (ctrl_in(ESDR_REQ_GET_STATE, &st, sizeof(st)) == sizeof(st)) {
			rate = st.sample_rate_hz;
		}
		ctrl_out(ESDR_REQ_SET_MODE, ESDR_MODE_RX, NULL, 0);
		run_stream(&s, true, seconds, rate);
		ctrl_out(ESDR_REQ_SET_MODE, ESDR_MODE_OFF, NULL, 0);
		print_state();
		if (s.out != NULL && s.out != stdout) {
			fclose(s.out);
		}
	} else if (strcmp(cmd, "tx") == 0) {
		struct esdr_state st;

		s.bits = bits;
		s.tone_hz = tone;
		s.amp = amp;
		if (infile != NULL && (s.in = fopen(infile, "rb")) == NULL) {
			perror(infile);
			return 1;
		}
		ctrl_out(ESDR_REQ_SET_MODE, ESDR_MODE_OFF, NULL, 0);
		set_freq((uint64_t)freq);
		set_u32(ESDR_REQ_SET_SAMPLE_RATE, rate);
		set_u32(ESDR_REQ_SET_FREQ_CORR, (uint32_t)ppb);
		ctrl_out(ESDR_REQ_SET_TX_GAIN, (uint16_t)txgain, NULL, 0);
		ctrl_out(ESDR_REQ_SET_FORMAT, (uint16_t)bits, NULL, 0);
		if (ctrl_in(ESDR_REQ_GET_STATE, &st, sizeof(st)) == sizeof(st)) {
			rate = st.sample_rate_hz;
		}
		s.rate = rate;
		ctrl_out(ESDR_REQ_SET_MODE, ESDR_MODE_TX, NULL, 0);
		run_stream(&s, false, seconds, rate);
		ctrl_out(ESDR_REQ_SET_MODE, ESDR_MODE_OFF, NULL, 0);
		usleep(200000);
		print_state();
	} else {
		usage();
		ret = 2;
	}
	libusb_release_interface(dev, 0);
	libusb_close(dev);
	libusb_exit(NULL);
	return ret;
}
