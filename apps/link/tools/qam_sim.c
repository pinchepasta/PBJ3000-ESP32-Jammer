/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Host check of the link PHY and RS codec (no Zephyr):
 *
 *   cc -O2 -Isrc -I../../include src/qam.c src/rs.c src/hamming.c tools/qam_sim.c -lm -o /tmp/qam_sim
 *   /tmp/qam_sim            simulated channel sweep
 *   /tmp/qam_sim file.bin   decode raw receive words (little endian u32)
 *
 * The channel loops the frame like the DAC does and cuts a capture window
 * at a random offset, with carrier offset, phase noise, a short ISI filter,
 * DC, white noise and 10-bit quantization.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "qam.h"
#include "hamming.h"
#include "rs.h"

#define WIN 16380

static struct qam_tx tx;
static struct qam_rx rx;
static uint32_t frame[QAM_FRAME_MAX_SAMPLES], win[WIN];
static uint8_t data[QAM_NCW_MAX * QAM_UNIT], got[QAM_NCW_MAX * QAM_UNIT];

static double gauss(void)
{
	double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);

	return sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
}

static int rs_selftest(void)
{
	uint8_t cw[RS_N], ref[RS_N];
	int bad = 0;

	for (int trial = 0; trial < 2000; trial++) {
		int nerr = trial % 20, ret;

		for (int i = 0; i < RS_K; i++) {
			cw[i] = (uint8_t)rand();
		}
		rs_encode(cw);
		memcpy(ref, cw, RS_N);
		for (int e = 0; e < nerr; e++) {
			cw[rand() % RS_N] ^= (uint8_t)(1 + rand() % 255);
		}
		ret = rs_decode(cw);
		if (nerr <= 16 && (ret < 0 || memcmp(cw, ref, RS_N) != 0)) {
			bad++;
		}
		if (nerr > 16 && ret >= 0 && memcmp(cw, ref, RS_N) != 0) {
			/* miscorrection, possible but rare */
		}
	}
	printf("rs: %d failures in 2000 (<= 16 errors must all correct)\n", bad);
	return bad;
}

static int ham_selftest(void)
{
	uint8_t b[HAM_N], ref[HAM_N];
	int bad = 0;

	for (int trial = 0; trial < 3000; trial++) {
		int nerr = trial % 3, ret;

		for (int i = 0; i < HAM_K; i++) {
			b[i] = (uint8_t)rand();
		}
		ham_encode(b);
		memcpy(ref, b, HAM_N);
		for (int e = 0, p0 = -1; e < nerr; e++) {
			int p = rand() % (HAM_N * 8);

			if (p == p0) {
				p = (p + 1) % (HAM_N * 8);
			}
			b[p / 8] ^= (uint8_t)(1U << (p % 8));
			p0 = p;
		}
		ret = ham_decode(b);
		if ((nerr < 2 && (ret < 0 || memcmp(b, ref, HAM_K) != 0)) || (nerr == 2 && ret >= 0)) {
			bad++;
		}
	}
	printf("hamming: %d failures in 3000 (1 error corrected, 2 detected)\n", bad);
	return bad;
}

struct chan {
	double snr_db, cfo_hz, pn_rad, dc, gain;
	double isi[3];
};

/* Loop the frame, cut WIN samples at a random offset, apply the channel. */
static void channel(const uint32_t *f, size_t len, const struct chan *c)
{
	size_t off = (size_t)rand() % len;

	if (getenv("SEG")) {
		printf("true start %zu (and +%zu)\n", (len - off) % len, len);
	}
	double ph = 2.0 * M_PI * rand() / RAND_MAX, sig = 0.0, prev[2][2] = {{0}};
	double fr = c->cfo_hz / 80e6;
	static double xi[WIN], xq[WIN];

	for (int n = 0; n < WIN; n++) {
		uint32_t w = f[(off + n) % len];
		double i = (double)((int32_t)(w << 22) >> 22), q = (double)((int32_t)(w << 12) >> 22);
		/* 3 tap ISI at the sample rate */
		double yi = c->isi[0] * prev[1][0] + c->isi[1] * prev[0][0] + c->isi[2] * i;
		double yq = c->isi[0] * prev[1][1] + c->isi[1] * prev[0][1] + c->isi[2] * q;

		prev[1][0] = prev[0][0];
		prev[1][1] = prev[0][1];
		prev[0][0] = i;
		prev[0][1] = q;
		ph += 2.0 * M_PI * fr + c->pn_rad * gauss();
		xi[n] = c->gain * (yi * cos(ph) - yq * sin(ph));
		xq[n] = c->gain * (yi * sin(ph) + yq * cos(ph));
		sig += xi[n] * xi[n] + xq[n] * xq[n];
	}
	sig /= WIN;
	for (int n = 0; n < WIN; n++) {
		double s = sqrt(sig / 2.0 / pow(10.0, c->snr_db / 10.0));
		long vi = lround(xi[n] + c->dc + s * gauss()), vq = lround(xq[n] - c->dc + s * gauss());

		vi = vi > 511 ? 511 : (vi < -512 ? -512 : vi);
		vq = vq > 511 ? 511 : (vq < -512 ? -512 : vq);
		win[n] = ((uint32_t)vq & 0x3ffU) | (((uint32_t)vi & 0x3ffU) << 10);
	}
}

static double now_us(void)
{
	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec * 1e6 + t.tv_nsec / 1e3;
}

/* Two frames looped as a pair: decode the one the search finds, then the other. */
static void pair_test(void)
{
	static uint8_t data_b[QAM_NCW_MAX * QAM_UNIT];
	static struct qam_rx rx2;
	static uint8_t cw2[QAM_RX_CW_BYTES];
	const double snrs[] = {24, 30, 36};

	rx2.cw = cw2;

	for (int mod = 0; mod < QAM_MODS - 1; mod++) {
		for (size_t s = 0; s < sizeof(snrs) / sizeof(snrs[0]); s++) {
			int ok = 0, ok2 = 0;
			const int trials = 40;
			size_t total = 0;

			qam_tx_init(&tx, mod, 150);
			for (int t = 0; t < trials; t++) {
				struct qam_hdr ha = {.mod = mod, .fec = QAM_FEC_RS, .full = 1, .pair = 1,
						     .seq = (uint16_t)(2 * t)};
				struct qam_hdr hb = ha, rh, rh2;
				struct qam_rx_info info, info2;
				struct chan c = {.snr_db = snrs[s], .cfo_hz = -60e3 + 120e3 * rand() / RAND_MAX,
						 .pn_rad = 0.002, .dc = 6, .gain = 0.4,
						 .isi = {0.1, 0.25, 1.0}};
				size_t n1 = qam_hdr_payload_bytes(&ha), lf;
				int ret, other;

				if (getenv("IDEAL")) {
					c = (struct chan){.snr_db = snrs[s], .gain = 1.0, .isi = {0, 0, 1}};
				}
				hb.seq = (uint16_t)(2 * t + 1);
				for (size_t i = 0; i < n1; i++) {
					data[i] = (uint8_t)rand();
					data_b[i] = (uint8_t)rand();
				}
				total = qam_tx_build_pair(&tx, &ha, data, &hb, data_b, frame);
				if (total == 0) {
					printf("pair build failed for mod %d\n", mod);
					return;
				}
				lf = total / 2;
				channel(frame, total, &c);
				ret = qam_rx_begin(&rx, win, WIN, &rh, &info);
				if (ret == 0) {
					ret = qam_rx_finish(&rx, &rh, &info);
				}
				if (ret != 0) {
					continue;
				}
				qam_rx_data(&rx, &rh, got);
				ok += memcmp(got, rh.seq & 1 ? data_b : data, n1) == 0;
				/* The other frame starts one frame length later (or earlier). */
				other = info.start + (int)lf;
				if (other + QAM_SEARCH_MARGIN > WIN) {
					other -= (int)total;
				}
				if (other < 0) {
					other = info.start - (int)lf;
				}
				if (qam_rx_begin_at(&rx2, win, WIN, other, info.cfo_hz, &rh2, &info2) == 0 &&
				    qam_rx_finish(&rx2, &rh2, &info2) == 0 && rh2.seq == (rh.seq ^ 1)) {
					qam_rx_data(&rx2, &rh2, got);
					ok2 += memcmp(got, rh2.seq & 1 ? data_b : data, n1) == 0;
				}
			}
			printf("pair mod %d (%zu B x 2, %zu samples) snr %.0f: first %d/%d, other %d/%d\n",
			       mod, qam_hdr_payload_bytes(&(struct qam_hdr){.mod = mod, .full = 1, .pair = 1}),
			       total, snrs[s], ok, trials, ok2, trials);
		}
	}
}

static void sweep(void)
{
	static const char *names[] = {"qpsk", "16qam", "64qam", "256qam", "1024qam", "4096qam"};
	static const char *fecs[] = {"rs", "ham", "none"};
	const double snrs[] = {12, 16, 20, 24, 28, 32, 36, 40, 45, 50};
	const int trials = 40;

	for (int fm = 0; fm < QAM_MODS * QAM_FECS; fm++) {
		int mod = fm % QAM_MODS, fec = fm / QAM_MODS;
		unsigned int ncw = getenv("WHOLE") ? qam_ncw_max(mod, fec) : 0;
		struct qam_hdr h = {.type = 0, .mod = mod, .fec = fec, .ncw = ncw, .full = ncw == 0,
				    .dst = 2, .src = 1};
		size_t len;

		qam_tx_init(&tx, mod, 150);
		for (size_t s = 0; s < sizeof(snrs) / sizeof(snrs[0]); s++) {
			int ok = 0, hdr = 0, none = 0, dat = 0, corr = 0;
			double mer = 0, cfo_err = 0, t_us = 0;

			for (int t = 0; t < trials; t++) {
				struct chan c = {.snr_db = snrs[s], .cfo_hz = -60e3 + 120e3 * rand() / RAND_MAX,
						 .pn_rad = 0.002, .dc = 6, .gain = 0.4,
						 .isi = {0.1, 0.25, 1.0}};

				if (getenv("IDEAL")) {
					double cfo = c.cfo_hz;

					c = (struct chan){.snr_db = 60, .gain = 1.0, .isi = {0, 0, 1}};
					if (getenv("CFO")) {
						c.cfo_hz = cfo;
					}
					if (getenv("PN")) {
						c.pn_rad = atof(getenv("PN"));
					}
					if (getenv("DC")) {
						c.dc = 6;
					}
					if (getenv("ISI")) {
						c.isi[0] = 0.1;
						c.isi[1] = 0.25;
					}
				}
				struct qam_hdr rh;
				struct qam_rx_info info;
				double t0;
				int ret;

				h.seq = (uint16_t)rand();
				for (size_t i = 0; i < qam_hdr_payload_bytes(&h); i++) {
					data[i] = (uint8_t)rand();
				}
				len = qam_tx_build(&tx, &h, data, frame);
				channel(frame, len, &c);
				t0 = now_us();
				ret = qam_rx_decode(&rx, win, WIN, &rh, &info);
				t_us += now_us() - t0;
				if (ret == 0) {
					qam_rx_data(&rx, &rh, got);
					if (rh.seq == h.seq &&
					    memcmp(got, data, qam_hdr_payload_bytes(&h)) == 0) {
						ok++;
					}
				} else if (ret == QAM_RX_NONE) {
					none++;
				} else if (ret == QAM_RX_HEADER) {
					hdr++;
				} else {
					dat++;
				}
				if (ret == 0 || ret == QAM_RX_DATA) {
					mer += info.mer_db;
					corr += info.corrected;
				}
				cfo_err += fabs(info.cfo_hz - c.cfo_hz);
#ifdef QAM_DEBUG
				if (getenv("SEG")) {
					extern void qam_debug_dump(void);
					printf("ret %d start %d cfo %.0f\n", ret, info.start, info.cfo_hz);
					qam_debug_dump();
				}
#endif
				if (getenv("DBG") && ret != 0) printf("  ret %d start %d len %zu wrapbound %d corr %d failed %d mer %.1f\n", ret, info.start, len, (int)(WIN - len), info.corrected, info.failed, info.mer_db);
			}
			printf("%-4s %-6s ncw %2u len %5zu snr %4.1f: ok %2d/%d none %d hdr %d data %d, "
			       "MER %5.1f dB, corrected %5.1f B/frame, |cfo err| %6.0f Hz, %4.0f us, "
			       "clipped %u\n",
			       fecs[fec], names[mod], ncw, len, snrs[s], ok, trials, none, hdr, dat,
			       ok + dat ? mer / (ok + dat) : 0.0,
			       ok + dat ? (double)corr / (ok + dat) : 0.0, cfo_err / trials,
			       t_us / trials, tx.clipped);
		}
	}
}

int main(int argc, char **argv)
{
	static uint8_t cw1[QAM_RX_CW_BYTES];

	srand(1);
	rx.cw = cw1;
	rs_init();
	ham_init();
	if (rs_selftest() != 0 || ham_selftest() != 0) {
		return 1;
	}
	if (argc > 1) {
		FILE *fp = fopen(argv[1], "rb");
		size_t n;
		struct qam_hdr rh;
		struct qam_rx_info info;
		int ret;

		if (fp == NULL) {
			perror(argv[1]);
			return 1;
		}
		n = fread(win, 4, WIN, fp);
		fclose(fp);
		rx.avg = getenv("AVG") ? atoi(getenv("AVG")) : 0;
		/* Real captures: the sample clocks differ like the carriers. */
		rx.carrier_hz = getenv("CARRIER") ? (float)atof(getenv("CARRIER")) : 2412e6f;
		ret = qam_rx_decode(&rx, win, n, &rh, &info);
		printf("ret %d metric %.3f start %d cfo %.0f Hz MER %.1f dB isi %.1f dB "
		       "corrected %d failed %d; type %u mod %u ncw %u dst %u src %u seq %u\n",
		       ret, info.metric, info.start, info.cfo_hz, info.mer_db, info.isi_db,
		       info.corrected, info.failed, rh.type, rh.mod, rh.ncw, rh.dst, rh.src,
		       rh.seq);
		return 0;
	}
	if (getenv("PAIR")) {
		pair_test();
		return 0;
	}
	sweep();
	return 0;
}

#ifdef QAM_DEBUG
struct cf {
	float re, im;
};
static double seg_err[64], seg_sig[64];
void qam_debug_sym(size_t s, struct cf z, struct cf d)
{
	size_t b = s / 64;
	static FILE *syms;

	if (getenv("SYMS")) {
		if (syms == NULL) {
			syms = fopen(getenv("SYMS"), "w");
		}
		fprintf(syms, "%g %g %g %g\n", z.re, z.im, d.re, d.im);
	}

	if (b < 64) {
		seg_err[b] += (z.re - d.re) * (z.re - d.re) + (z.im - d.im) * (z.im - d.im);
		seg_sig[b] += d.re * d.re + d.im * d.im;
	}
}
void qam_debug_dump(void)
{
	for (int b = 0; b < 64; b++) {
		if (seg_sig[b] > 0) {
			printf("%5.1f%s", 10 * log10(seg_sig[b] / seg_err[b]), b % 16 == 15 ? "\n" : " ");
		}
		seg_err[b] = seg_sig[b] = 0;
	}
	printf("\n");
}
#endif
