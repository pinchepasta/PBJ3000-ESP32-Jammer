/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * USB protocol of the osmosdr app, shared with the host tools: vendor
 * control requests to the device (recipient device) set the radio, bulk
 * endpoints carry the samples, after HackRF's model.
 *
 * - Bulk IN (0x81): receive blocks of ESDR_BLOCK_BYTES, a header then
 *   interleaved I/Q (int8, or int16 little endian). The device sends whole
 *   blocks (multiples of 64 bytes, no short packets), so any read size that
 *   is a multiple of ESDR_BLOCK_BYTES stays aligned to them.
 * - Bulk OUT: transmit samples, interleaved I/Q in the format set (int8, or
 *   int16 little endian), at the sample rate set. The device takes them as
 *   fast as the DAC plays them: USB flow control paces the host, about 50 ms
 *   ahead. Input that runs out plays as zeros.
 *
 * All multi-byte fields are little endian. Plain C99, no Zephyr headers.
 */

#ifndef ESDR_PROTO_H_
#define ESDR_PROTO_H_

#include <stdint.h>

#define ESDR_PROTO_VERSION 1U

/* Vendor requests (bRequest). */
enum esdr_req {
	/* IN: struct esdr_info. */
	ESDR_REQ_GET_INFO = 0x01,
	/* wValue: enum esdr_mode. */
	ESDR_REQ_SET_MODE = 0x02,
	/* OUT 8 bytes: centre frequency in Hz (uint64_t). */
	ESDR_REQ_SET_FREQ = 0x03,
	/* OUT 4 bytes: sample rate in Hz (uint32_t); the nearest supported one is taken. */
	ESDR_REQ_SET_SAMPLE_RATE = 0x04,
	/* wValue: PHY gain index (0 to info.rx_gain_max), or ESDR_GAIN_AUTO. */
	ESDR_REQ_SET_RX_GAIN = 0x05,
	/* wValue: transmit power step (0 weakest to info.tx_gain_max). */
	ESDR_REQ_SET_TX_GAIN = 0x06,
	/* OUT 4 bytes: analog low-pass bandwidth in Hz (uint32_t), 0 for automatic. */
	ESDR_REQ_SET_BANDWIDTH = 0x07,
	/* wValue: bits per I and Q item, 8 or 16. */
	ESDR_REQ_SET_FORMAT = 0x08,
	/* OUT 4 bytes: crystal correction in parts per billion (int32_t). */
	ESDR_REQ_SET_FREQ_CORR = 0x09,
	/* IN: struct esdr_state. */
	ESDR_REQ_GET_STATE = 0x0a,
	/* IN: struct esdr_stats. */
	ESDR_REQ_GET_STATS = 0x0b,
	/* wValue: ESDR_OPT_* bits. */
	ESDR_REQ_SET_OPTIONS = 0x0c,
	/*
	 * wValue: digital gain in dB before the items are rounded, a multiple of
	 * 6 up to ESDR_DGAIN_MAX_DB, or ESDR_DGAIN_DEFAULT (24 dB at 8 bit, 0 at
	 * 16). Decimation leaves a quiet band's noise far below an 8-bit item.
	 */
	ESDR_REQ_SET_DIGITAL_GAIN = 0x0d,
};

enum esdr_mode {
	ESDR_MODE_OFF = 0,
	ESDR_MODE_RX = 1,
	ESDR_MODE_TX = 2,
	/* Throughput tests: IN sends counting blocks as fast as the bus takes them, OUT sinks. */
	ESDR_MODE_TEST_IN = 0x10,
	ESDR_MODE_TEST_OUT = 0x11,
};

#define ESDR_GAIN_AUTO 0xffffU
#define ESDR_DGAIN_DEFAULT 0xffffU
#define ESDR_DGAIN_MAX_DB  42U

/*
 * Offset tuning (default on): the LO sits 4 MHz below the centre and the
 * stream is mixed down digitally, so the LO leakage and the 1/f noise at
 * the LO stay outside the output band.
 */
#define ESDR_OPT_OFFSET_TUNING 0x0001U

#define ESDR_INFO_MAGIC 0x52445345U /* "ESDR" */

struct esdr_info {
	uint32_t magic;
	uint16_t proto_version;
	uint16_t info_size;
	char firmware[32];
	uint64_t freq_min_hz, freq_max_hz;
	/* Supported receive rates: rx_rate_max_hz >> k for k = 0 .. rx_rate_count - 1. */
	uint32_t rx_rate_max_hz;
	uint16_t rx_rate_count;
	uint16_t rx_gain_max;
	uint16_t tx_gain_max;
	uint16_t block_bytes;
	/* Full scale of the 16-bit items (8-bit items: 128). */
	uint16_t full_scale16;
	uint16_t reserved;
} __attribute__((packed));

struct esdr_state {
	uint16_t mode;
	uint16_t bits;
	uint16_t options;
	/* Gain index or ESDR_GAIN_AUTO; transmit step. */
	uint16_t rx_gain, tx_gain;
	/* Digital gain in dB as applied. */
	uint16_t digital_gain_db;
	/* Requested centre, and where the stream is actually centred (Hz, rounded). */
	uint64_t freq_hz;
	uint64_t center_hz;
	uint32_t sample_rate_hz;
	uint32_t bandwidth_hz;
	int32_t freq_corr_ppb;
	/* LO as programmed (MHz plus kHz offset). */
	uint32_t lo_mhz;
	int32_t lo_offset_khz;
} __attribute__((packed));

struct esdr_stats {
	/* Samples delivered to USB, and samples dropped for want of free blocks. */
	uint64_t rx_samples, rx_overflow_samples;
	/* Input lost in the ring (filter too slow), and samples lost to restarts. */
	uint64_t rx_ring_lost_pairs, rx_restart_samples;
	uint64_t usb_in_bytes, usb_out_bytes;
	uint32_t rx_blocks, rx_overflows;
	uint32_t ring_runs, ring_errors, ring_late_max, ring_cycles_x100;
	uint32_t ring_abandoned, ring_gain_refreshed;
	/* Last ring error (enum esp_sdr_ring_status) and its detail. */
	uint32_t ring_last_status, ring_last_detail;
	/* Transmit: samples played, and dropped (late, or lost in a DAC restart). */
	uint64_t tx_samples, tx_dropped;
	/* Blocks the DAC played short of input (zero filled), sessions, DAC restarts. */
	uint32_t tx_underruns, tx_sessions, tx_restarts;
	/* Largest backlog of samples waiting for the DAC. */
	uint32_t tx_queued_max;
} __attribute__((packed));

#define ESDR_BLOCK_BYTES 4096U
#define ESDR_BLOCK_MAGIC 0x4b4c4245U /* "EBLK" */

/* Block flags. */
/* The block's index does not follow the previous block's: samples are missing before it. */
#define ESDR_BLK_GAP      0x0001U
/* Blocks were dropped before this one (the host did not read in time). */
#define ESDR_BLK_OVERFLOW 0x0002U
/* Items are int16 (else int8). */
#define ESDR_BLK_16BIT    0x0004U
/* Counting test pattern, not samples. */
#define ESDR_BLK_TEST     0x0008U

struct esdr_block_hdr {
	uint32_t magic;
	uint16_t flags;
	/* Complex samples in this block. */
	uint16_t samples;
	/* Index of the first sample on the stream's time axis (rate since the stream started). */
	uint64_t index;
} __attribute__((packed));

#define ESDR_BLOCK_PAYLOAD (ESDR_BLOCK_BYTES - sizeof(struct esdr_block_hdr))

/*
 * Serial transport (apps/osmosdr_serial, ESP32-C6 over USB-Serial-JTAG: one
 * byte stream each way). The vendor requests travel as frames: struct
 * esdr_ser_hdr, then len bytes.
 *
 * - Host to device: req is an enum esdr_req. An OUT request carries its data
 *   (len bytes) and wValue; an IN request (ESDR_SER_IN) carries none, len is
 *   the most it accepts. ESDR_SER_TX_DATA frames carry transmit samples.
 * - Device to host: every request gets one reply frame with the same req,
 *   status 0 or a negative errno, and the IN data. Receive blocks (as on USB,
 *   starting with ESDR_BLOCK_MAGIC) come between frames, never inside one.
 *
 * A request other than ESDR_SER_TX_DATA stops a running stream for as long as
 * it takes to apply it; the next receive block then carries ESDR_BLK_GAP.
 */
#define ESDR_SER_MAGIC 0x4c525345U /* "ESRL" */
#define ESDR_SER_IN      0x01U
#define ESDR_SER_TX_DATA 0x80U
#define ESDR_SER_MAX_DATA 512U

struct esdr_ser_hdr {
	uint32_t magic;
	uint8_t req;
	uint8_t flags;
	uint16_t value;
	uint16_t len;
	int16_t status;
} __attribute__((packed));

#endif /* ESDR_PROTO_H_ */
