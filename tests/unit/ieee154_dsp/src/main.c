/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Unit test of the esp_sdr 802.15.4 driver's signal processing
 * (lib/ieee802154/ieee154_dsp.h) end to end: frame -> DAC words at 40 MS/s
 * -> channel (resampling to 16 MS/s, delay, CFO, DC, noise, 10-bit receive
 * words) -> -fs/4 mix and decimation (the burst FIR, the ring's boxcar of
 * 4, or a plain pick-every-4th) -> PHY. Runs on native_sim.
 *
 * Besides the pass/fail cases, this file also holds the decimator sweep table
 * (test_decimator_table), opt-in with IEEE154_TABLES (see doc/testing.md): the
 * decoded fraction over SNR through the three decimators. The case is skipped
 * unless the variable is set; with it, test_cfo_estimator_accuracy also prints
 * its estimates.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/ztest.h>

#include <esp_sdr/ieee154_phy.h>
#include <ieee154_dsp.h>

#define AMP          300.0
#define FRAMES       60  /* pass/fail cases: fewer than the table's, still exact at these margins */
#define TABLE_FRAMES 200 /* per point of the decimator table */
#define PI           3.14159265358979323846

static ieee154_dsp_tmpl_t tmpl;
static struct ieee154_dsp_tx tx = {.tmpl = (const ieee154_dsp_tmpl_t *)&tmpl};

#define RNG_SEED 88172645463325252ULL
static uint64_t rs;
static uint32_t rnd(void)
{
	rs ^= rs << 13;
	rs ^= rs >> 7;
	rs ^= rs << 17;
	return (uint32_t)(rs >> 32);
}
static double urand(void)
{
	return (rnd() + 1.0) / 4294967297.0;
}
static double gauss(void)
{
	return sqrt(-2 * log(urand())) * cos(2 * PI * urand());
}

static uint8_t expect[IEEE154_PHY_PSDU_MAX];
static uint8_t expect_len;
static int got;

static void cb(const uint8_t *p, uint8_t len, int lqi, void *user)
{
	ARG_UNUSED(lqi);
	ARG_UNUSED(user);
	if (len == expect_len && memcmp(p, expect, len) == 0) {
		got++;
	}
}

static int16_t s10(uint32_t v)
{
	return (int16_t)((int32_t)(v << 22) >> 22);
}

static uint32_t rx_word(double i, double q)
{
	long ri = lrint(i), rq = lrint(q);

	ri = ri > 511 ? 511 : (ri < -512 ? -512 : ri);
	rq = rq > 511 ? 511 : (rq < -512 ? -512 : rq);
	/* Receive words: I in bits 19:10, Q in bits 9:0 (esp_sdr_rx_i/q). */
	return (((uint32_t)ri & 0x3ffU) << 10) | ((uint32_t)rq & 0x3ffU);
}

/* The ring's ESP_SDR_RING_DECIM_BOX4 (esp_sdr_ring.c), grid aligned. */
static size_t box4(const uint32_t *w, size_t n, int16_t *out)
{
	size_t outs = n / 4U;

	for (size_t k = 0; k < outs; k++) {
		const uint32_t *x = &w[4 * k];
		int32_t ai = esp_sdr_rx_i(x[0]) + esp_sdr_rx_q(x[1]) - esp_sdr_rx_i(x[2]) -
			     esp_sdr_rx_q(x[3]);
		int32_t aq = esp_sdr_rx_q(x[0]) - esp_sdr_rx_i(x[1]) - esp_sdr_rx_q(x[2]) +
			     esp_sdr_rx_i(x[3]);

		out[2 * k] = (int16_t)(ai * 4);
		out[2 * k + 1] = (int16_t)(aq * 4);
	}
	return outs;
}

/* Every 4th pair: the channel, 4 MHz above the LO, aliases to 0 Hz; no filter at all. */
static size_t pick4(const uint32_t *w, size_t n, int16_t *out)
{
	size_t outs = n / 4U;

	for (size_t k = 0; k < outs; k++) {
		out[2 * k] = (int16_t)(esp_sdr_rx_i(w[4 * k]) * 16);
		out[2 * k + 1] = (int16_t)(esp_sdr_rx_q(w[4 * k]) * 16);
	}
	return outs;
}

/*
 * Fraction of frames decoded, out of the given number, at snr_db with carrier offset cfo_hz,
 * through decimator dec (0 the burst FIR, 1 box4, 2 pick4).
 */
static double run(int dec, double snr_db, double cfo_hz, int frames)
{
	static uint32_t dac[(6 + 127) * 64 * 20 + 40];
	static uint32_t words[(6 + 127) * 64 * 8 + 4096];
	static int16_t iq[2 * ((6 + 127) * 64 * 2 + 1024)];
	double sigma = AMP / sqrt(2 * pow(10, snr_db / 10));
	int ok = 0;

	for (int f = 0; f < frames; f++) {
		uint8_t psdu[IEEE154_PHY_PSDU_MAX];
		int plen = 3 + (int)(rnd() % 123);
		uint64_t n40;
		size_t lead = 200 + rnd() % 64, n16, nout;
		double ph = urand() * 2 * PI;

		for (int i = 0; i < plen; i++) {
			psdu[i] = (uint8_t)rnd();
		}
		expect_len = (uint8_t)ieee154_phy_append_fcs(psdu, (size_t)plen);
		memcpy(expect, psdu, expect_len);
		ieee154_dsp_tx_build(&tx, psdu, expect_len);
		n40 = ieee154_dsp_tx_samples(&tx);
		/* In two calls, as the DAC backend fills block by block. */
		ieee154_dsp_tx_gen(&tx, dac, 0, 777);
		ieee154_dsp_tx_gen(&tx, dac + 777, 777, (uint32_t)n40 - 777);

		n16 = (size_t)(n40 * 2 / 5) + lead + 400;
		for (size_t m = 0; m < n16; m++) {
			double vi = 0, vq = 0, th, ri, rq;

			if (m >= lead) {
				double tt = (double)(m - lead) * 2.5, fr;
				size_t a = (size_t)tt;

				fr = tt - (double)a;
				if (a + 1 < n40) {
					vi = s10(dac[a]) * (1 - fr) + s10(dac[a + 1]) * fr;
					vq = s10(dac[a] >> 10) * (1 - fr) +
					     s10(dac[a + 1] >> 10) * fr;
				}
			}
			th = ph + 2 * PI * cfo_hz * (double)m / 16e6;
			ri = vi * cos(th) - vq * sin(th) + sigma * gauss() + 20;
			rq = vi * sin(th) + vq * cos(th) + sigma * gauss() - 15;
			words[m] = rx_word(ri, rq);
		}
		nout = dec == 0 ? ieee154_dsp_mix_decimate(words, n16, iq, 1)
				: dec == 1 ? box4(words, n16, iq) : pick4(words, n16, iq);
		got = 0;
		ieee154_phy_rx_reset();
		ieee154_phy_rx_block(iq, nout);
		ok += got == 1;
	}
	return (double)ok / frames;
}

/* IEEE154_TABLES enables the tables when set to anything. */
static bool tables_enabled(void)
{
	return getenv("IEEE154_TABLES") != NULL;
}

static void *suite_setup(void)
{
	ieee154_phy_init();
	ieee154_dsp_tmpl_init(&tmpl, (float)AMP, false);
	ieee154_phy_set_rx_cb(cb, NULL);
	return NULL;
}

static void before_each(void *fixture)
{
	ARG_UNUSED(fixture);
	rs = RNG_SEED;
	ieee154_phy_rx_reset();
}

ZTEST_SUITE(ieee154_dsp, NULL, suite_setup, before_each, NULL, NULL);

/*
 * Comfortable margins: every point is 3 dB or more above the SNR at which its decimator first
 * reads 1.000 in test_decimator_table.
 */
ZTEST(ieee154_dsp, test_decodes_at_operating_points)
{
	static const char *const name[] = {"fir", "box4", "pick"};
	static const struct {
		int dec;
		double snr, cfo;
	} pts[] = {
		{0, 3, 0}, {1, 3, 0}, {0, 6, 100e3}, {1, 6, 100e3}, {0, 40, 0}, {1, 40, 0},
		{2, 9, 0}, {2, 12, 100e3}, {2, 40, 0},
	};

	ARRAY_FOR_EACH(pts, k) {
		double r = run(pts[k].dec, pts[k].snr, pts[k].cfo, FRAMES);

		zassert_equal(r, 1.0, "%s snr %.0f dB cfo %.0f Hz: %.3f decoded, expected 1.0",
			      name[pts[k].dec], pts[k].snr, pts[k].cfo, r);
	}
}

/*
 * The sweep table, opt-in with IEEE154_TABLES: the decoded fraction over SNR through each
 * decimator. Every column must read 1.000 from 2 dB above the SNR where it first does (-2, 0 and
 * 6 dB for fir, box4 and pick).
 */
ZTEST(ieee154_dsp, test_decimator_table)
{
	/* Per decimator, as run() numbers them (fir, box4, pick): the SNR from which all decode. */
	static const int all_from_db[] = {0, 2, 8};
	int bad = 0;

	if (!tables_enabled()) {
		ztest_test_skip();
	}

	printk("decoded fraction, %d frames/point, SNR over 16 MHz\n", TABLE_FRAMES);
	for (int snr = -6; snr <= 10; snr += 2) {
		double r[ARRAY_SIZE(all_from_db)];

		/*
		 * Pick, box4, fir, one statement each: the three runs share one RNG stream, so
		 * their order decides the numbers, and an argument list would leave it to the
		 * compiler.
		 */
		r[2] = run(2, snr, 0, TABLE_FRAMES);
		r[1] = run(1, snr, 0, TABLE_FRAMES);
		r[0] = run(0, snr, 0, TABLE_FRAMES);
		printk("%4d dB: fir %.3f  box4 %.3f  pick %.3f\n", snr, r[0], r[1], r[2]);

		ARRAY_FOR_EACH(r, d) {
			if (snr >= all_from_db[d] && r[d] < 1.0) {
				bad++;
			}
		}
	}
	/* Asserted only now, so that a failing run shows the whole table. */
	zassert_equal(bad, 0,
		      "%d points below 1.000 (fir from %d dB, box4 from %d dB, pick from %d dB)",
		      bad, all_from_db[0], all_from_db[1], all_from_db[2]);
}

/* ieee154_dsp_cfo() and ieee154_dsp_freq() through the FIR chain, at 30 dB SNR. */
ZTEST(ieee154_dsp, test_cfo_estimator_accuracy)
{
	static uint32_t dac[(6 + 127) * 64 * 20 + 40];
	static uint32_t words[(6 + 127) * 64 * 8 + 4096];
	static int16_t iq[2 * ((6 + 127) * 64 * 2 + 1024)];
	static const double offs[] = {0, 40e3, -120e3, 250e3, -350e3, 500e3, -700e3};
	double sigma = AMP / sqrt(2 * pow(10, 30.0 / 10));
	uint8_t psdu[IEEE154_PHY_PSDU_MAX];

	for (int i = 0; i < 100; i++) {
		psdu[i] = (uint8_t)rnd();
	}
	ieee154_dsp_tx_build(&tx, psdu, (uint8_t)ieee154_phy_append_fcs(psdu, 100));
	ieee154_dsp_tx_gen(&tx, dac, 0, (uint32_t)ieee154_dsp_tx_samples(&tx));
	ARRAY_FOR_EACH(offs, o) {
		size_t n16 = (size_t)(ieee154_dsp_tx_samples(&tx) * 2 / 5), nout, used;
		float p, f, fc;

		for (size_t m = 0; m < n16; m++) {
			double tt = (double)m * 2.5, fr, vi, vq, th;
			size_t a = (size_t)tt;

			fr = tt - (double)a;
			vi = s10(dac[a]) * (1 - fr) + s10(dac[a + 1]) * fr;
			vq = s10(dac[a] >> 10) * (1 - fr) + s10(dac[a + 1] >> 10) * fr;
			/* Sender's carrier feedthrough: 20 % at the channel centre (+4 MHz). */
			vi += 0.2 * AMP * cos(2 * PI * 4e6 * (double)m / 16e6 + 0.3);
			vq += 0.2 * AMP * sin(2 * PI * 4e6 * (double)m / 16e6 + 0.3);
			th = 2 * PI * offs[o] * (double)m / 16e6;
			words[m] = rx_word(vi * cos(th) - vq * sin(th) + sigma * gauss(),
					   vi * sin(th) + vq * cos(th) + sigma * gauss());
		}
		nout = ieee154_dsp_mix_decimate(words, n16, iq, 1);
		f = ieee154_dsp_cfo(iq, nout, &p, &used);
		fc = ieee154_dsp_freq(iq, nout);

		if (tables_enabled()) {
			printk("cfo %8.0f Hz: estimated %8.0f Hz (%zu samples), coarse %8.0f Hz\n",
			       offs[o], (double)f, used, (double)fc);
		}
		/* The fine estimate wraps at +-500 kHz. */
		if (fabs(offs[o]) < 450e3) {
			zassert_true(fabs((double)f - offs[o]) <= 10e3,
				     "fine cfo at %.0f Hz: estimated %.0f Hz", offs[o], (double)f);
		}
		zassert_true(fabs((double)fc - offs[o]) <= 30e3 + 0.1 * fabs(offs[o]),
			     "coarse cfo at %.0f Hz: estimated %.0f Hz", offs[o], (double)fc);
	}
}
