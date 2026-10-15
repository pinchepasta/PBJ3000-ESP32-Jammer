/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * OFDM physical layer on raw radio words. Configured by occupied bandwidth,
 * subcarrier count and cyclic prefix; the FFT size follows (the power of two
 * that puts that many subcarriers into that bandwidth at the sample rate).
 * Complex baseband: the subcarriers sit on both sides of DC, DC unused. The
 * modulation (BPSK to 64-QAM) is chosen per frame.
 *
 * A frame is a Schmidl-Cox preamble (known BPSK on the even subcarriers, so
 * its time halves repeat), one or more pilot symbols (known BPSK on all, for
 * the channel estimate), optional header symbols (BPSK, the header bits
 * repeated over all their subcarriers) and the payload symbols, each with a
 * cyclic prefix. The frame is built to loop in the DAC like the QAM frames;
 * the receiver decodes one whole copy from a capture window at least twice
 * the frame long.
 *
 * Carrier offset correction covers +-1 subcarrier spacing.
 */

#ifndef ESDR_LINK_OFDM_H
#define ESDR_LINK_OFDM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Receive and transmit words: esp_sdr_rx_i/q(), esp_sdr_tx_word(). */
#include <esp_sdr/esp_sdr.h>

#define OFDM_NFFT_MIN 16U
#define OFDM_NFFT_MAX 512U
/* Shortest cyclic prefix as a fraction of the FFT size (cp_div at most this). */
#define OFDM_CP_DIV_MAX 32U
/* Subcarriers in use at most: a quarter of the band stays free around Nyquist. */
#define OFDM_CH_MAX (OFDM_NFFT_MAX * 3U / 4U)
/* Power of two above the longest cyclic prefix (nfft / 4). */
#define OFDM_RING 256U
#define OFDM_PILOTS_MAX 4U
#define OFDM_HDR_BITS_MAX 64U

enum ofdm_mod {
	OFDM_BPSK,
	OFDM_QPSK,
	OFDM_16QAM,
	OFDM_64QAM,
	OFDM_MODS
};

struct ofdm_cfg {
	/* Radio sample rate, Hz. */
	uint32_t sample_rate_hz;
	/* Occupied bandwidth, Hz: rounded so that spacing = sample_rate / nfft. */
	uint32_t bandwidth_hz;
	/* Subcarriers in use (pilot and data), DC excluded. */
	unsigned int channels;
	/* Cyclic prefix: nfft / cp_div samples (4 to OFDM_CP_DIV_MAX). */
	unsigned int cp_div;
	/* Transmit RMS amplitude, DAC units (full scale 511). */
	int amp;
	/* Symbols after the pilots (header and payload), 0: as many as fit max_samples. */
	unsigned int symbols;
	/* Longest frame in samples. */
	size_t max_samples;
	/* The receiver samples at sample_rate_hz / rx_div (1 or 2): half the work at 2. */
	unsigned int rx_div;
	/* Pilot symbols (1 to OFDM_PILOTS_MAX), averaged for the channel estimate. */
	unsigned int pilots;
	/* Also smooth the estimate over neighbouring subcarriers (3 taps). */
	bool smooth;
	/* Header bits per frame (0 to OFDM_HDR_BITS_MAX), on BPSK symbols before the payload. */
	unsigned int hdr_bits;
	/*
	 * Another pilot symbol in the middle of the payload: the phase is
	 * measured again there, so the second half can be decoded on its own
	 * (on another CPU) and a long frame does not drift.
	 */
	bool mid_pilot;
};

struct ofdm_cf {
	float re, im;
};

/*
 * One receiver working through symbols: its FFT buffer, the tracked common
 * phase and its rate per slot (a residual carrier offset), and the known
 * phase of its current window (u). Two of them decode one frame on two CPUs.
 */
struct ofdm_worker {
	struct ofdm_cf buf[OFDM_NFFT_MAX] __attribute__((aligned(16)));
	struct ofdm_cf trk, rate, u;
};

/* Receive state between ofdm_rx_begin() and ofdm_rx_finish(). */
struct ofdm_rxstate {
	int start, dci, dcq;
	float eps;
	/* Step of the window phase per slot. */
	struct ofdm_cf us;
	/* How early the FFT window starts, from the pilots' phase slope (samples). */
	float delay;
	/* First payload slot. */
	unsigned int slot;
	float soft[OFDM_HDR_BITS_MAX];
};

struct ofdm_ctx {
	/* FFT twiddles (esp-dsp radix-2 layout: cos, sin, bit reversed; serves every size up
	 * to nfft).
	 */
	float w[OFDM_NFFT_MAX] __attribute__((aligned(16)));
	/* The receiver's own worker (search, channel, header, payload). */
	struct ofdm_worker w0;
	struct ofdm_cfg cfg;
	/* Derived by ofdm_init(): transmit layout at sample_rate_hz; the payload symbol the
	 * mid pilot comes before (ndata if none).
	 */
	unsigned int nfft, log2n, cp, nhdr, ndata, mid;
	size_t slot, len;
	float spacing_hz, bw_hz;
	/* The same frame at the receive rate. */
	unsigned int rnfft, rcp;
	size_t rslot, rlen;
	/* Bit reversal over nfft; per subcarrier (frequency order) its transmit FFT bin and
	 * where it comes out of the (bit reversed) receive FFT.
	 */
	uint16_t rev[OFDM_NFFT_MAX];
	uint16_t bin[OFDM_CH_MAX], binr[OFDM_CH_MAX];
	/* Equalizer per subcarrier, conj(H) / |H|^2, from the last pilots. */
	struct ofdm_cf eq[OFDM_CH_MAX];
	union {
		/* Search: the last preamble metrics (numerator, denominator) over the prefix. */
		struct {
			float rnum[OFDM_RING], rden[OFDM_RING];
		};
		/* Then the carrier offset derotation over one receive FFT window. */
		struct ofdm_cf rot[OFDM_NFFT_MAX];
	};
	struct ofdm_rxstate rx;
};

/* A transmit work buffer (ofdm_tx_build()), separate so that building and decoding can overlap. */
struct ofdm_txbuf {
	struct ofdm_cf buf[OFDM_NFFT_MAX] __attribute__((aligned(16)));
	/* Samples clipped by the last build. */
	uint32_t clipped;
};

/*
 * Derive the frame layout from @p cfg.
 *
 * @retval 0 on success.
 * @retval -1 if no FFT size fits (bandwidth, channel count, cp_div, frame length).
 */
int ofdm_init(struct ofdm_ctx *ctx, const struct ofdm_cfg *cfg);

static inline unsigned int ofdm_mod_bits(enum ofdm_mod mod)
{
	static const uint8_t bits[OFDM_MODS] = {1, 2, 4, 6};

	return mod < OFDM_MODS ? bits[mod] : 0U;
}

/* Payload bits per frame at @p mod. */
static inline size_t ofdm_frame_bits(const struct ofdm_ctx *ctx, enum ofdm_mod mod)
{
	return (size_t)ctx->ndata * ctx->cfg.channels * ofdm_mod_bits(mod);
}

/* Length of a header-only frame (preamble, pilots, header), at the transmit and receive rate. */
static inline size_t ofdm_hdr_samples(const struct ofdm_ctx *ctx)
{
	return (size_t)(1U + ctx->cfg.pilots + ctx->nhdr) * ctx->slot;
}

static inline size_t ofdm_hdr_rsamples(const struct ofdm_ctx *ctx)
{
	return (size_t)(1U + ctx->cfg.pilots + ctx->nhdr) * ctx->rslot;
}

/* Test payload: whitened bits, a function of @p seed (packed, MSB first). */
void ofdm_test_bits(uint8_t *bits, size_t nbits, uint32_t seed);

/*
 * Build one frame into @p out (ctx->len words): cfg.hdr_bits header bits from
 * @p hdr, ofdm_frame_bits(mod) payload bits from @p bits (packed, MSB first).
 * With @p bits NULL a header-only frame (ofdm_hdr_samples() words). Only
 * reads @p ctx.
 *
 * @return Its length in samples, 0 for a bad modulation.
 */
size_t ofdm_tx_build(const struct ofdm_ctx *ctx, struct ofdm_txbuf *tb, enum ofdm_mod mod,
		     const uint8_t *hdr, const uint8_t *bits, uint32_t *out);

struct ofdm_rx_info {
	/* Frame start in the window, sample index. */
	int start;
	/* Preamble metric of that start (normalized correlation, squared). */
	float metric;
	/* Carrier offset: in subcarrier spacings, and in Hz. */
	float cfo_frac, cfo_hz;
	/* Error of the payload symbols against their decisions, dB. */
	float mer_db;
	/* Cycles (ofdm_clock): search, channel estimate and header, payload. */
	uint32_t prof[3];
};

/* Optional cycle counter for ofdm_rx_info.prof. */
extern uint32_t (*ofdm_clock)(void);

/*
 * Find a whole frame copy in @p n receive words (at the receive rate), set up
 * the channel and decode the header into @p hdr (cfg.hdr_bits bits). The window
 * must hold one whole copy (a looped frame: n >= 2 * ctx->rlen + ctx->rcp).
 *
 * @retval 0 on success.
 * @retval -1 if no preamble was found.
 */
int ofdm_rx_begin(struct ofdm_ctx *ctx, const uint32_t *words, size_t n, uint8_t *hdr,
		  struct ofdm_rx_info *info);

/* The same for a frame looping with @p period receive samples (header-only: ofdm_hdr_rsamples()). */
int ofdm_rx_begin_period(struct ofdm_ctx *ctx, const uint32_t *words, size_t n, size_t period,
			 uint8_t *hdr, struct ofdm_rx_info *info);

/* Then the ofdm_frame_bits(mod) payload bits into @p bits, from the same words. */
void ofdm_rx_finish(struct ofdm_ctx *ctx, const uint32_t *words, enum ofdm_mod mod, uint8_t *bits,
		    struct ofdm_rx_info *info);

/*
 * The payload in two parts, e.g. on two CPUs: ctx->w0 decodes payload
 * symbols 0 .. mid - 1, a second worker set up by ofdm_rx_fork() (which
 * measures the phase on the mid pilot) symbols mid .. ndata - 1.
 * ofdm_rx_part() decodes s0 .. s1 - 1 into @p bits (the frame's bits from
 * symbol s0 on; mid starts a byte) and adds their squared error to @p err.
 * Only the workers are written.
 */
void ofdm_rx_fork(const struct ofdm_ctx *ctx, struct ofdm_worker *wk, const uint32_t *words);
void ofdm_rx_part(const struct ofdm_ctx *ctx, struct ofdm_worker *wk, const uint32_t *words,
		  enum ofdm_mod mod, unsigned int s0, unsigned int s1, uint8_t *bits, float *err);

/* MER of a payload from its squared error. */
float ofdm_rx_mer(const struct ofdm_ctx *ctx, float err);

/* Forward FFT of the receive buffer in place (output bit reversed, see ctx->rev); for tests. */
void ofdm_test_fft(struct ofdm_ctx *ctx);

#endif
