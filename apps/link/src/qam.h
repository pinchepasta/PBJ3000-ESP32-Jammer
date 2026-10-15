/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Single carrier QAM physical layer at the native 80 MS/s, portable C.
 *
 * A frame is QAM_SPS samples per symbol, raised cosine shaped, and built to
 * be played in a loop: its waveform is cyclic, so any capture window at
 * least QAM_FRAME_MAX_SAMPLES + QAM_SEARCH_MARGIN long holds one whole copy,
 * possibly split across the wrap. Layout in symbols:
 *
 *   preamble  2 x 64 QPSK, the same PN sequence twice (timing, frequency
 *             offset, equalizer training)
 *   header    32 QPSK: 8 bytes with a CRC-16
 *   payload   ncw units (RS(255,223), Hamming or uncoded), byte
 *             interleaved, in QPSK to 4096-QAM (Gray coded per axis); the
 *             data bytes end in a CRC-32
 */

#ifndef LINK_QAM_H_
#define LINK_QAM_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Receive and transmit words: esp_sdr_rx_i/q(), esp_sdr_tx_word(). */
#include <esp_sdr/esp_sdr.h>

#include "rs.h"

/* Samples per symbol at 80 MS/s (even): 4 is 20 Mbaud, 6 is 13.3 Mbaud. */
#ifndef QAM_SPS
#define QAM_SPS         4
#endif
#define QAM_PRE_HALF    64
#define QAM_PRE_SYMS    (2 * QAM_PRE_HALF)
#define QAM_HDR_BYTES   8
#define QAM_HDR_SYMS    (QAM_HDR_BYTES * 4)
#define QAM_NCW_MAX     24
/* Capture window the frames are sized for (one dump bank). */
#define QAM_WINDOW            16380
/* Fine timing search around the coarse estimate, in samples. */
#define QAM_FINE_RANGE        (12 * QAM_SPS)
/* Extra window the receiver needs to find a frame. */
#define QAM_SEARCH_MARGIN     ((QAM_PRE_SYMS + QAM_HDR_SYMS + 4) * QAM_SPS + QAM_FINE_RANGE + 2 * QAM_SPS)
/* Longest frame: a window still holds one whole copy, from wherever the search starts. */
#define QAM_FRAME_MAX_SAMPLES                                                                  \
	((QAM_WINDOW - QAM_SEARCH_MARGIN - QAM_FINE_RANGE - 2 * QAM_SPS) / QAM_SPS * QAM_SPS)
#define QAM_CRC_BYTES   4

/*
 * Error correction per unit of QAM_UNIT bytes on air: RS(255,223), 15
 * extended Hamming (128,120) blocks (240 bytes, 225 data), or none (255 data).
 */
enum qam_fec {
	QAM_FEC_RS,
	QAM_FEC_HAMMING,
	QAM_FEC_NONE,
	QAM_FECS,
};

#define QAM_UNIT RS_N

/* Coded and data bytes per unit. */
unsigned int qam_unit_coded(enum qam_fec fec);
unsigned int qam_unit_data(enum qam_fec fec);

/* Square constellations, 2 (mod + 1) bits per symbol. */
enum qam_mod {
	QAM_QPSK,
	QAM_QAM16,
	QAM_QAM64,
	QAM_QAM256,
	QAM_QAM1024,
	QAM_QAM4096,
	QAM_MODS,
};

/* Frame header; seq, type and addresses are for the MAC. */
struct qam_hdr {
	uint8_t type;
	uint8_t mod;
	uint8_t ncw;
	uint8_t dst;
	uint8_t src;
	uint8_t fec;
	/* The frame fills its maximum length (ncw counts a short last unit). */
	uint8_t full;
	/*
	 * One of two equal frames looped together (qam_tx_build_pair()): it
	 * fills at most half the maximum length, and the loop period is twice
	 * its length.
	 */
	uint8_t pair;
	uint16_t seq;
};

/* Constellation bits per symbol. */
unsigned int qam_bits(enum qam_mod mod);

/* Most units per frame at @p mod and @p fec. */
unsigned int qam_ncw_max(enum qam_mod mod, enum qam_fec fec);

/* Frame length in samples, 0 if @p ncw does not fit. */
size_t qam_frame_samples(enum qam_mod mod, enum qam_fec fec, unsigned int ncw);

/* User data bytes per frame: ncw units' data minus the CRC-32. */
static inline size_t qam_payload_bytes(enum qam_fec fec, unsigned int ncw)
{
	return ncw == 0 ? 0 : ncw * qam_unit_data(fec) - QAM_CRC_BYTES;
}

/* Frame length in samples and user data bytes for a header (whole or full frames), 0 if invalid. */
size_t qam_hdr_frame_samples(const struct qam_hdr *h);
size_t qam_hdr_payload_bytes(const struct qam_hdr *h);

/* User data bytes of a full frame at @p mod and @p fec. */
size_t qam_full_payload_bytes(enum qam_mod mod, enum qam_fec fec);

/* Symbol spaced equalizer length (odd). */
#ifndef QAM_EQ_TAPS
#define QAM_EQ_TAPS 11
#endif

/* Raised cosine roll-off and span. */
#define QAM_RC_BETA 0.35f
#define QAM_SPAN    8
/* Levels per axis: up to 64 payload levels, then the two QPSK ones. */
#define QAM_LEVELS_MAX 64
#define QAM_LEVEL_QPSK QAM_LEVELS_MAX

struct qam_tx {
	/*
	 * Pulse table: span position, axis level, sample phase pair; 1/16 DAC
	 * steps, phase 2n + phase 2n + 1 * 65536 so one add sums two phases.
	 */
	int32_t table[QAM_SPAN][QAM_LEVELS_MAX + 2][QAM_SPS / 2];
	/* Axis levels of each symbol. */
	uint8_t sym_i[QAM_FRAME_MAX_SAMPLES / QAM_SPS];
	uint8_t sym_q[QAM_FRAME_MAX_SAMPLES / QAM_SPS];
	uint8_t cw[QAM_NCW_MAX * QAM_UNIT];
	int amp;
	int mod;
	/* Samples clipped in the last build. */
	unsigned int clipped;
};

/*
 * Prepare @p tx for @p mod at RMS amplitude @p amp (DAC steps per axis,
 * 1 to 300; the DAC clips at 511).
 */
void qam_tx_init(struct qam_tx *tx, enum qam_mod mod, int amp);

/*
 * Build one frame into @p out (at least qam_frame_samples() words). @p data
 * holds qam_payload_bytes(hdr->fec, hdr->ncw) bytes; hdr->mod must match
 * qam_tx_init().
 *
 * @return Frame length in samples, 0 on a bad header.
 */
size_t qam_tx_build(struct qam_tx *tx, const struct qam_hdr *hdr, const uint8_t *data,
		    uint32_t *out);

/* What the receiver measured on the last frame. */
struct qam_rx_info {
	/* Preamble metric (1 = perfect), carrier offset, start sample. */
	float metric;
	float cfo_hz;
	int start;
	/* Modulation error ratio over header and payload, in dB. */
	float mer_db;
	/* Bytes (RS) or bits (Hamming) corrected; units that could not be corrected. */
	int corrected;
	int failed;
	/* Equalizer taps' energy outside the centre tap, in dB below it. */
	float isi_db;
	/* Normalized cross-correlation with the known preamble (1 = perfect). */
	float xcorr;
	/*
	 * With qam_clock set: clock ticks spent in search, fine timing,
	 * training, header, payload and FEC.
	 */
	uint32_t prof[6];
};

/* Optional cycle counter for qam_rx_info.prof. */
extern uint32_t (*qam_clock)(void);

struct qam_rx {
	/*
	 * Carrier in Hz, set by the caller: the carrier offset then also gives
	 * the sample clock offset (0: no sample clock tracking).
	 */
	float carrier_hz;
	/*
	 * Average up to this many loop copies of the payload (0 or 1: off).
	 * Frames short enough to repeat in the window gain 10 log10(copies) dB.
	 */
	int avg;
	/* Codeword buffer, QAM_RX_CW_BYTES, set by the caller (may live in slower RAM). */
	uint8_t *cw;
	/* Equalizer taps and bias of the last frame. */
	float w_re[QAM_EQ_TAPS], w_im[QAM_EQ_TAPS], b_re, b_im;
	/* Scratch: training samples, preamble derotation, equalizer state. */
	float ytr[2 * (QAM_PRE_SYMS + QAM_EQ_TAPS / 2)];
	float ph[2 * QAM_PRE_SYMS];
	float eq[8 * QAM_EQ_TAPS + 16];
	/* Decoder state between qam_rx_begin*() and qam_rx_finish(). */
	uint32_t job[112];
};

#define QAM_RX_CW_BYTES (QAM_NCW_MAX * QAM_UNIT)

#define QAM_RX_NONE   (-1) /* no preamble */
#define QAM_RX_HEADER (-2) /* header CRC */
#define QAM_RX_DATA   (-3) /* payload RS or CRC-32 */

/*
 * Find and decode one frame in @p n receive words captured at 80 MS/s.
 * On 0 the header is in @p hdr and the data at qam_rx_data().
 *
 * @return 0, QAM_RX_NONE, QAM_RX_HEADER or QAM_RX_DATA (hdr valid).
 */
int qam_rx_decode(struct qam_rx *rx, const uint32_t *words, size_t n, struct qam_hdr *hdr,
		  struct qam_rx_info *info);


/*
 * The decoder in two steps: find a frame and read its header, then its
 * payload. qam_rx_decode() is both. Two contexts can decode the two frames
 * of a pair from one window at once (the window is only read).
 */
int qam_rx_begin(struct qam_rx *rx, const uint32_t *words, size_t n, struct qam_hdr *hdr,
		 struct qam_rx_info *info);

/* Like qam_rx_begin() for a frame known to start near sample @p start (+-QAM_FINE_RANGE). */
int qam_rx_begin_at(struct qam_rx *rx, const uint32_t *words, size_t n, int start, float cfo_hz,
		    struct qam_hdr *hdr, struct qam_rx_info *info);

int qam_rx_finish(struct qam_rx *rx, const struct qam_hdr *hdr, struct qam_rx_info *info);

/*
 * Build two frames looped as one (both with hdr->pair set, same mod, fec,
 * ncw and full): any window then holds both. @p a and @p b are their data.
 *
 * @return Total length in samples, 0 on a bad header.
 */
size_t qam_tx_build_pair(struct qam_tx *tx, const struct qam_hdr *ha, const uint8_t *a,
			 const struct qam_hdr *hb, const uint8_t *b, uint32_t *out);

/* Copy the decoded user data (qam_payload_bytes() bytes) to @p out. */
void qam_rx_data(const struct qam_rx *rx, const struct qam_hdr *hdr, uint8_t *out);

/* Mean power of @p n receive words in raw units squared, DC removed. */
float qam_rx_power(const uint32_t *words, size_t n);

/* CRC-16-CCITT (0xffff start), as in the frame headers. */
uint16_t qam_crc16(const uint8_t *p, size_t n);

uint32_t qam_crc32(const uint8_t *p, size_t n);

#endif /* LINK_QAM_H_ */
