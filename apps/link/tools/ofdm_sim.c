/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Host simulator for ofdm.c. Checks the FFT against a direct DFT, then loops
 * a frame into a capture window at a random offset (as the DAC plays it),
 * through carrier offset, optional multipath, noise and 10 bit quantization,
 * and decodes it; or decodes a real capture (link_perf.py dump).
 *
 *   cc -O2 -Wall -Isrc -I../../include src/ofdm.c src/ofdm_frame.c src/qam.c src/rs.c src/hamming.c \
 *      tools/ofdm_sim.c -lm -o /tmp/ofdm_sim
 *   BW=16 CH=52 MOD=64qam /tmp/ofdm_sim [capture.bin]
 *
 * Environment: BW (MHz), CH, CPDIV, MOD (bpsk .. 64qam, default all), AMP,
 * SYMS, CFO (Hz), MP (1: multipath), GAIN (receive scale), RXDIV (1, 2:
 * receive at 40 MS/s), PILOTS, SMOOTH, HDR (header bits), TRIALS. SNR is over
 * the receive bandwidth.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ofdm.h"
#include "ofdm_frame.h"
#include "rs.h"

#define FS     80000000U
#define WIN    16380U
#define TWO_PI 6.28318530717958647692

static uint32_t frame[WIN], win[WIN];
static uint8_t bits[WIN], got[WIN], hdr[OFDM_HDR_BITS_MAX / 8], hgot[OFDM_HDR_BITS_MAX / 8];
static struct ofdm_ctx ctx __attribute__((aligned(16)));
static struct ofdm_ctx ref __attribute__((aligned(16)));
static struct ofdm_txbuf tb;
static struct ofdm_fwork fwk;
static struct ofdm_worker wk2;
static const char *const mod_names[OFDM_MODS] = {"bpsk", "qpsk", "16qam", "64qam"};

static int dbg_sym;

void ofdm_debug_symbol(float e2, unsigned int c, struct ofdm_cf r)
{
	if (getenv("SYMDBG")) {
		printf(" %d:%.0f/%.0f", dbg_sym, 10.0 * log10((double)c / (e2 + 1e-9)),
		       atan2((double)r.im, (double)r.re) * 57.2958);
	}
	dbg_sym++;
}

static double env_f(const char *k, double d)
{
	const char *v = getenv(k);

	return v ? atof(v) : d;
}

static double randn(void)
{
	double u1 = (rand() + 1.0) / ((double)RAND_MAX + 2.0), u2 = rand() / (double)RAND_MAX;

	return sqrt(-2.0 * log(u1)) * cos(TWO_PI * u2);
}

static unsigned int bit_at(const uint8_t *b, size_t i)
{
	return (b[i >> 3] >> (7U - (i & 7U))) & 1U;
}

/* The FFT against a direct DFT: a build and decode round trip could hide an error both share. */
static int check_fft(void)
{
	unsigned int n = ctx.nfft;
	struct ofdm_cf x[OFDM_NFFT_MAX];
	double worst = 0.0;

	for (unsigned int i = 0; i < n; i++) {
		x[i] = (struct ofdm_cf){(float)randn(), (float)randn()};
	}
	ref = ctx;
	memcpy(ref.w0.buf, x, n * sizeof(x[0]));
	ofdm_test_fft(&ref);
	for (unsigned int k = 0; k < n; k++) {
		double re = 0.0, im = 0.0;

		for (unsigned int t = 0; t < n; t++) {
			double a = -TWO_PI * (double)k * (double)t / (double)n;

			re += x[t].re * cos(a) - x[t].im * sin(a);
			im += x[t].re * sin(a) + x[t].im * cos(a);
		}
		{
			struct ofdm_cf y = ref.w0.buf[ref.rev[k]];
			double e = hypot(y.re - re, y.im - im);

			worst = e > worst ? e : worst;
		}
	}
	printf("fft %u vs DFT: max error %.2e (of ~%.0f)\n", n, worst, sqrt((double)n));
	return worst < 1e-3 * sqrt((double)n) ? 0 : -1;
}

/*
 * Loop the frame into the window from @p off: multipath and carrier offset at
 * the transmit rate, decimation by rx_div (a windowed sinc low pass, as the
 * radio's receive filter), then noise and 10 bits at the receive rate.
 */
static void channel(size_t len, unsigned int off, double snr_db, double cfo_hz, int mp,
		    double gain, unsigned int rx_div, unsigned int *clipped)
{
	enum { TAPS = 31 };
	static const double taps_mp[3][2] = {{1.0, 0.0}, {0.25, -0.1}, {0.0, 0.08}};
	static double fr[2 * WIN + TAPS], fq[2 * WIN + TAPS];
	double h[TAPS], p = 0.0, ns, ph = TWO_PI * cfo_hz / FS, hs = 0.0;
	unsigned int nfull = rx_div * WIN + TAPS;
	int ntaps = mp ? 3 : 1;

	/* Low pass at 0.45 of the receive rate (Hann window), unity gain. */
	for (int k = 0; k < TAPS; k++) {
		double x = k - (TAPS - 1) / 2.0, fc = 0.45 / rx_div;

		h[k] = (x == 0.0 ? 2.0 * fc : sin(TWO_PI * fc * x) / (M_PI * x)) *
		       (0.5 - 0.5 * cos(TWO_PI * k / (TAPS - 1)));
		hs += h[k];
	}
	for (int k = 0; k < TAPS; k++) {
		h[k] /= hs;
	}
	for (unsigned int i = 0; i < nfull; i++) {
		double yi = 0.0, yq = 0.0, c = cos(ph * i), s = sin(ph * i);

		for (int t = 0; t < ntaps; t++) {
			uint32_t f = frame[(i + off + len - (unsigned int)t) % len];
			double xi = (int32_t)(f << 22) >> 22, xq = (int32_t)(f << 12) >> 22;
			double hr = taps_mp[t][0], hi = taps_mp[t][1];

			yi += xi * hr - xq * hi;
			yq += xi * hi + xq * hr;
		}
		fr[i] = gain * (yi * c - yq * s);
		fq[i] = gain * (yi * s + yq * c);
	}
	for (unsigned int i = 0; i < WIN; i++) {
		double ri = 0.0, rq = 0.0;

		if (rx_div == 1) {
			ri = fr[i];
			rq = fq[i];
		} else {
			for (int k = 0; k < TAPS; k++) {
				ri += h[k] * fr[i * rx_div + k];
				rq += h[k] * fq[i * rx_div + k];
			}
		}
		fr[i] = ri;
		fq[i] = rq;
		p += ri * ri + rq * rq;
	}
	p /= WIN;
	ns = sqrt(p / pow(10.0, snr_db / 10.0) / 2.0);
	*clipped = 0;
	for (unsigned int i = 0; i < WIN; i++) {
		int qi = (int)lrint(fr[i] + ns * randn() + 3.0), qq = (int)lrint(fq[i] + ns * randn() - 2.0);

		if (qi > 511 || qi < -512 || qq > 511 || qq < -512) {
			(*clipped)++;
		}
		qi = qi > 511 ? 511 : (qi < -512 ? -512 : qi);
		qq = qq > 511 ? 511 : (qq < -512 ? -512 : qq);
		win[i] = (((uint32_t)qi & 0x3ffU) << 10) | ((uint32_t)qq & 0x3ffU);
	}
}

int main(int argc, char **argv)
{
	const char *ms = getenv("MOD");
	struct ofdm_cfg cfg = {
		.sample_rate_hz = FS,
		.bandwidth_hz = (uint32_t)(env_f("BW", 16.0) * 1e6),
		.channels = (unsigned int)env_f("CH", 52),
		.cp_div = (unsigned int)env_f("CPDIV", 8),
		.amp = (int)env_f("AMP", 90),
		.symbols = (unsigned int)env_f("SYMS", 0),
		.max_samples = (size_t)env_f("MAXS", (WIN + 1U) / 2U),
		.rx_div = (unsigned int)env_f("RXDIV", 1),
		.pilots = (unsigned int)env_f("PILOTS", 1),
		.smooth = env_f("SMOOTH", 0) != 0.0,
		.hdr_bits = (unsigned int)env_f("HDR", 64),
		.mid_pilot = env_f("MIDP", 1) != 0.0,
	};
	int m0 = 0, m1 = OFDM_MODS - 1;

	srand(1);
	if (ms) {
		for (int m = 0; m < OFDM_MODS; m++) {
			if (strcmp(ms, mod_names[m]) == 0) {
				m0 = m1 = m;
			}
		}
	}
	if (ofdm_init(&ctx, &cfg) != 0) {
		printf("no layout for these settings\n");
		return 1;
	}
	printf("rx fft %u at %u MS/s; nfft %u, spacing %.1f kHz, %u subcarriers = %.2f MHz, cp %u, "
	       "%u pilots, %u header + %u data symbols, %zu samples\n",
	       ctx.rnfft, FS / cfg.rx_div / 1000000U, ctx.nfft, ctx.spacing_hz / 1e3, cfg.channels,
	       ctx.bw_hz / 1e6, ctx.cp, cfg.pilots, ctx.nhdr, ctx.ndata, ctx.len);
	if (check_fft() != 0) {
		return 1;
	}
	ofdm_test_bits(hdr, cfg.hdr_bits, 7);

	if (argc > 1) {
		FILE *f = fopen(argv[1], "rb");
		size_t n = f ? fread(win, 4, WIN, f) : 0;
		struct ofdm_rx_info info;
		size_t nbits = ofdm_frame_bits(&ctx, (enum ofdm_mod)m1);
		int ret, errs = 0;

		if (f) {
			fclose(f);
		}
		ofdm_test_bits(bits, nbits, 1);
		if (getenv("HDRONLY")) {
			struct ofdm_fhdr fh;

			rs_init();
			ret = ofdm_frame_begin(&ctx, win, n, &fh, &info);
			printf("%s: frame_begin %d start %d metric %.3f cfo %.1f kHz", argv[1], ret,
			       info.start, info.metric, info.cfo_hz / 1e3);
			if (ret == 0) {
				struct ofdm_fwork *w = &fwk;
				int corr, r2 = ofdm_frame_finish(&ctx, win, &fh, w, got, &corr, &info);

				printf(" type %u mod %u dst %02x src %02x seq %u; payload %d (RS fixed %d) MER %.1f",
				       fh.type, fh.mod, fh.dst, fh.src, fh.seq, r2, corr, info.mer_db);
			}
			printf("\n");
			return 0;
		}
		ret = ofdm_rx_begin(&ctx, win, n, hgot, &info);
		if (ret == 0) {
			ofdm_rx_finish(&ctx, win, (enum ofdm_mod)m1, got, &info);
		}
		for (size_t i = 0; ret == 0 && i < nbits; i++) {
			errs += bit_at(bits, i) != bit_at(got, i);
		}
		printf("%s: ret %d start %d metric %.3f cfo %.1f kHz MER %.1f dB errors %d/%zu\n",
		       argv[1], ret, info.start, info.metric, info.cfo_hz / 1e3, info.mer_db, errs,
		       nbits);
		if (ret == 0 && errs > 0) {
			size_t per = cfg.channels * ofdm_mod_bits((enum ofdm_mod)m1);

			printf("errors per payload symbol:");
			for (size_t y = 0; y < ctx.ndata; y++) {
				int e = 0;

				for (size_t i = y * per; i < (y + 1) * per; i++) {
					e += bit_at(bits, i) != bit_at(got, i);
				}
				printf(" %d", e);
			}
			printf("\n");
		}
		return 0;
	}

	if (getenv("ACK")) {
		struct ofdm_fhdr h = {.type = 1, .mod = OFDM_BPSK, .dst = 0x80, .src = 0x00};
		size_t al = ofdm_hdr_samples(&ctx), n = 2U * ofdm_hdr_rsamples(&ctx) + ctx.rnfft;
		const double snrs[] = {6, 9, 12, 18, 24, 30};
		int trials = (int)env_f("TRIALS", 50);

		printf("ack: %zu samples (%.1f us), window %zu\n", al, (double)al * 1e6 / FS, n);
		for (size_t s = 0; s < sizeof(snrs) / sizeof(snrs[0]); s++) {
			int ok = 0, none = 0, bad = 0;
			unsigned int clipped;

			for (int t = 0; t < trials; t++) {
				struct ofdm_rx_info info;
				struct ofdm_fhdr g;
				int ret;

				h.seq = (uint16_t)(t * 7);
				if (ofdm_frame_build_hdr(&ctx, &tb, &h, frame) != al) {
					printf("ack build failed\n");
					return 1;
				}
				channel(al, (unsigned int)rand() % al, snrs[s],
					env_f("CFO", 20000.0) * (0.5 + rand() / (double)RAND_MAX),
					(int)env_f("MP", 0), env_f("GAIN", 0.6), cfg.rx_div, &clipped);
				ret = ofdm_frame_begin_hdr(&ctx, win, n, &g, &info);
				if (ret == -1) {
					none++;
				} else if (ret != 0 || g.seq != h.seq || g.type != 1 || g.dst != h.dst) {
					bad++;
				} else {
					ok++;
				}
			}
			printf("  snr %4.0f dB: ok %d/%d, none %d, bad %d\n", snrs[s], ok, trials, none, bad);
		}
		return 0;
	}

	if (getenv("FRAME")) {
		rs_init();
		for (int m = m0; m <= m1; m++) {
			struct ofdm_fhdr h = {.type = 0, .mod = (enum ofdm_mod)m, .dst = 2, .src = 1};
			size_t user = ofdm_frame_user_bytes(&ctx, h.mod);
			const double snrs[] = {12, 15, 18, 21, 24, 27, 30};
			int trials = (int)env_f("TRIALS", 20);

			printf("%s frames: %zu user bytes = %.1f Mbit/s on air\n", mod_names[m], user,
			       (double)user * 8.0 / ((double)ctx.len / FS) / 1e6);
			for (size_t s = 0; s < sizeof(snrs) / sizeof(snrs[0]); s++) {
				int ok = 0, hdr_err = 0, data_err = 0, none = 0, fixed = 0;
				unsigned int clipped;

				for (int t = 0; t < trials; t++) {
					struct ofdm_rx_info info;
					struct ofdm_fhdr g;
					int corr, ret;

					h.seq = (uint16_t)t;
					ofdm_test_bits(bits, user * 8U, (uint32_t)t + 3U);
					if (ofdm_frame_build(&ctx, &tb, &fwk, &h, bits, frame) == 0) {
						printf("build failed\n");
						return 1;
					}
					channel(ctx.len, (unsigned int)rand() % ctx.len, snrs[s],
						env_f("CFO", 20000.0) * (0.5 + rand() / (double)RAND_MAX),
						(int)env_f("MP", 0), env_f("GAIN", 0.6), cfg.rx_div, &clipped);
					ret = ofdm_frame_begin(&ctx, win, WIN, &g, &info);
					if (ret == -1) {
						none++;
						continue;
					}
					if (ret != 0 || g.seq != h.seq || g.mod != h.mod || g.dst != h.dst ||
					    g.src != h.src) {
						hdr_err++;
						continue;
					}
					ret = ofdm_frame_finish(&ctx, win, &g, &fwk, got, &corr, &info);
					if (ret != 0 || memcmp(got, bits, user) != 0) {
						data_err++;
						continue;
					}
					ok++;
					fixed += corr;
				}
				printf("  snr %4.0f dB: ok %3d/%d, none %d, header %d, data %d, RS fixed %.1f B/frame\n",
				       snrs[s], ok, trials, none, hdr_err, data_err,
				       ok ? (double)fixed / ok : 0.0);
			}
		}
		return 0;
	}

	for (int m = m0; m <= m1; m++) {
		enum ofdm_mod mod = (enum ofdm_mod)m;
		size_t nbits = ofdm_frame_bits(&ctx, mod);
		const double snrs[] = {12, 18, 24, 27, 30, 33, 36};
		int trials = (int)env_f("TRIALS", 20), mp = (int)env_f("MP", 0);
		double cfo = env_f("CFO", 20000.0), gain = env_f("GAIN", 0.6);

		ofdm_test_bits(bits, nbits, 1);
		(void)ofdm_tx_build(&ctx, &tb, mod, hdr, bits, frame);
		printf("%s: %zu bits = %.1f Mbit/s on air, tx clipped %u\n", mod_names[m], nbits,
		       (double)nbits / ((double)ctx.len / FS) / 1e6, tb.clipped);
		for (size_t s = 0; s < sizeof(snrs) / sizeof(snrs[0]); s++) {
			int ok = 0, hbad = 0;
			long errs = 0;
			double mer = 0.0, pe_sum = 0.0, pe_max = 0.0;
			unsigned int clipped = 0;

			for (int t = 0; t < trials; t++) {
				struct ofdm_rx_info info;
				unsigned int off = (unsigned int)rand() % ctx.len;
				double c = cfo * (0.5 + rand() / (double)RAND_MAX);

				channel(ctx.len, off, snrs[s], c, mp, gain, cfg.rx_div, &clipped);
				if (ofdm_rx_begin(&ctx, win, WIN, hgot, &info) != 0) {
					errs += (long)nbits;
					continue;
				}
				ok++;
				hbad += memcmp(hdr, hgot, cfg.hdr_bits / 8U) != 0;
				dbg_sym = 0;
				if (getenv("SYMDBG")) {
					/* True start: the loop offset, and the decimator's delay (15 samples at the tx rate). */
					double tru = fmod(((double)ctx.len - off - (cfg.rx_div == 2 ? 15.0 : 0.0)) / cfg.rx_div,
							  (double)ctx.rlen);

					printf("    trial %d (cfo %.0f est %.0f, start %d true %.1f mod %zu: %.1f):", t, c,
					       info.cfo_hz, info.start, tru, ctx.rlen,
					       fmod((double)info.start, (double)ctx.rlen) - tru);
				}
				if (getenv("SPLIT")) {
					/* Two workers, as on two CPUs: fork before worker 0 moves on. */
					unsigned int mid = ctx.mid;
					size_t off = (size_t)mid * cfg.channels * ofdm_mod_bits(mod) / 8U;
					float err = 0.0f, pa, aa, d;

					ofdm_rx_fork(&ctx, &wk2, win);
					ofdm_rx_part(&ctx, &ctx.w0, win, mod, 0, mid, got, &err);
					/* The fork's phase against worker 0's carried to the same slot. */
					pa = atan2f(wk2.trk.im, wk2.trk.re);
					aa = atan2f(ctx.w0.trk.im, ctx.w0.trk.re) +
					     atan2f(ctx.w0.rate.im, ctx.w0.rate.re);
					d = fabsf(remainderf(pa - aa, 6.2831853f)) * 57.29578f;
					pe_sum += d;
					pe_max = d > pe_max ? d : pe_max;
					ofdm_rx_part(&ctx, &wk2, win, mod, mid, ctx.ndata, got + off, &err);
					info.mer_db = ofdm_rx_mer(&ctx, err);
				} else {
					ofdm_rx_finish(&ctx, win, mod, got, &info);
				}
				mer += info.mer_db;
				{
					long e0 = errs;
					long first = -1;

					for (size_t i = 0; i < nbits; i++) {
						int d = bit_at(bits, i) != bit_at(got, i);

						errs += d;
						if (d && first < 0) {
							first = (long)i;
						}
					}
					if (getenv("DBG") && errs > e0) {
						printf("    trial %d: %ld errors from symbol %ld (sub %ld), cfo %.0f est %.0f, MER %.1f, start %d off %u\n",
						       t, errs - e0, first / (long)(cfg.channels * ofdm_mod_bits(mod)),
						       (first / (long)ofdm_mod_bits(mod)) % (long)cfg.channels, c, info.cfo_hz,
						       info.mer_db, info.start, off);
					}
				}
			}
			printf("  snr %4.0f dB: found %2d/%d, header bad %d, BER %.2e, MER %5.1f dB%s",
			       snrs[s], ok, trials, hbad, (double)errs / ((double)nbits * trials),
			       ok ? mer / ok : 0.0, clipped ? ", rx clipped" : "");
			if (getenv("SPLIT") && ok) {
				printf(", fork phase error %.2f deg avg, %.2f max", pe_sum / ok, pe_max);
			}
			printf("\n");
		}
	}
	return 0;
}
