/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Unit test of the software IEEE 802.15.4 PHY (lib/ieee802154/ieee154_phy.c):
 * modulate, impair (carrier offset, DC offset, AWGN), stream through the
 * receiver block by block, check what came back. Runs on native_sim, no
 * radio or Zephyr dependency beyond ztest.
 *
 * Besides the pass/fail cases, this file also holds the PER sweep table
 * (test_per_table), opt-in with IEEE154_TABLES (see doc/testing.md): it is
 * skipped unless the variable is set.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/ztest.h>

#include <esp_sdr/ieee154_phy.h>

#define AMP 1500
#define FS  4e6
/* 64 chips/byte, 2 samples/chip, longest PSDU. */
#define MAXS ((6 + IEEE154_PHY_PSDU_MAX) * 128 + 4)
#define PI   3.14159265358979323846

/* Deterministic xorshift64, reseeded at the start of every test. */
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
static int got, bad;
static double gphase; /* running carrier phase, continuous across frames in one test */

static void cb(const uint8_t *p, uint8_t len, int lqi, void *user)
{
	ARG_UNUSED(lqi);
	ARG_UNUSED(user);
	if (len == expect_len && memcmp(p, expect, len) == 0) {
		got++;
	} else {
		bad++;
	}
}

/* Rotate by CFO, add DC offset and AWGN. noise 0: exact, no impairment added. */
static void impair(int16_t *iq, size_t n, int noise, double snr_db, double cfo_hz, double dc)
{
	double sigma = noise ? AMP / sqrt(2 * pow(10, snr_db / 10)) : 0;
	double dphi = 2 * PI * cfo_hz / FS;

	for (size_t k = 0; k < n; k++) {
		double c = cos(gphase), s = sin(gphase);
		double i = iq[2 * k], q = iq[2 * k + 1];
		double ri, rq;

		gphase += dphi;
		ri = i * c - q * s + dc + sigma * gauss();
		rq = i * s + q * c + dc * 0.5 + sigma * gauss();
		iq[2 * k] = (int16_t)lrint(ri);
		iq[2 * k + 1] = (int16_t)lrint(rq);
	}
}

/* Modulates, impairs and decodes nframes random frames; returns the PER, *dups counts frames
 * delivered more than once.
 */
static double run_point(int nframes, int noise, double snr_db, double cfo, double dc, int *dups)
{
	static int16_t frame[2 * MAXS], buf[2 * (MAXS + 2000)];
	int ok = 0;

	*dups = 0;
	for (int f = 0; f < nframes; f++) {
		int plen = 3 + (int)(rnd() % 123); /* payload 3..125 */
		uint8_t psdu[IEEE154_PHY_PSDU_MAX];
		size_t len, ns, lead, trail, tot;

		for (int i = 0; i < plen; i++) {
			psdu[i] = (uint8_t)rnd();
		}
		len = ieee154_phy_append_fcs(psdu, (size_t)plen);
		ns = ieee154_phy_modulate(psdu, (uint8_t)len, AMP, frame, MAXS);
		zassert_not_equal(ns, 0, "modulator buffer too small");

		memcpy(expect, psdu, len);
		expect_len = (uint8_t)len;
		got = 0;

		/* Random lead gap sets the sampling phase (which tracker locks, and
		 * where within a symbol): varying it is what the dup-delivery
		 * regression below relies on to visit many tracker phases.
		 */
		lead = 1500 + rnd() % 256;
		trail = 200;
		tot = lead + ns + trail;

		memset(buf, 0, tot * 2 * sizeof(int16_t));
		memcpy(buf + 2 * lead, frame, ns * 2 * sizeof(int16_t));
		impair(buf, tot, noise, snr_db, cfo, dc);

		for (size_t p = 0; p < tot; p += 256) { /* block streaming, as the driver does */
			size_t nb = tot - p < 256 ? tot - p : 256;

			ieee154_phy_rx_block(buf + 2 * p, nb);
		}
		if (got >= 1) {
			ok++;
		}
		if (got > 1) {
			(*dups)++;
		}
	}
	return 1.0 - (double)ok / nframes;
}

static void *suite_setup(void)
{
	ieee154_phy_init();
	ieee154_phy_set_rx_cb(cb, NULL);
	return NULL;
}

/* Independent of execution order: same RNG seed, receiver and carrier phase every test. */
static void before_each(void *fixture)
{
	ARG_UNUSED(fixture);
	rs = RNG_SEED;
	gphase = 0;
	got = 0;
	bad = 0;
	ieee154_phy_rx_reset();
}

ZTEST_SUITE(ieee154_phy, NULL, suite_setup, before_each, NULL, NULL);

/* Noiseless: every frame must decode, exactly once, byte for byte. */
ZTEST(ieee154_phy, test_calibration_noiseless)
{
	int dups;
	double per = run_point(50, 0, 0, 0, 0, &dups);

	zassert_equal(per, 0, "PER %.3f, expected 0", per);
	zassert_equal(dups, 0, "%d duplicate deliveries", dups);
	zassert_equal(bad, 0, "%d CRC-ok frames with wrong content", bad);
}

/*
 * Regression: a frame end reset the OTHER phase tracker unconditionally,
 * which could in turn reset the tracker that was mid-frame and about to
 * succeed, losing it (0.3 to 1% error floor even at 30 dB SNR, seen only
 * because the random lead below sweeps many tracker-phase offsets). The fix
 * resets the sibling only after a valid CRC (ieee154_phy.c, trk_chip_slow()).
 */
ZTEST(ieee154_phy, test_no_premature_sibling_reset_regression)
{
	int dups;
	double per = run_point(300, 1, 30, 0, 0, &dups);

	zassert_equal(per, 0, "PER %.4f at 30 dB, expected 0 (sibling tracker reset too early?)",
		      per);
	zassert_equal(dups, 0, "%d duplicate deliveries", dups);
}

/* Operating points measured error-free in doc/ieee802154.md's host test table. */
ZTEST(ieee154_phy, test_decodes_at_operating_points)
{
	static const struct {
		double snr_db, cfo_hz;
	} pts[] = {
		{6, 0}, {6, 50e3}, {6, 100e3}, {10, 0}, {10, 100e3}, {16, 0},
	};

	ARRAY_FOR_EACH(pts, k) {
		int dups;
		double per = run_point(80, 1, pts[k].snr_db, pts[k].cfo_hz, 100, &dups);

		zassert_equal(per, 0, "PER %.4f at %.0f dB, %.0f Hz CFO, expected 0", per,
			      pts[k].snr_db, pts[k].cfo_hz);
		zassert_equal(dups, 0, "%d duplicate deliveries", dups);
	}
}

/* Modulates psdu (len bytes, FCS included) after a quiet lead and feeds it to the receiver. */
static void feed_frame(const uint8_t *psdu, size_t len)
{
	static int16_t frame[2 * MAXS], buf[2 * (MAXS + 2000)];
	size_t ns = ieee154_phy_modulate(psdu, (uint8_t)len, AMP, frame, MAXS);

	zassert_not_equal(ns, 0, "modulator buffer too small");
	memset(buf, 0, sizeof(buf));
	memcpy(buf + 2 * 1500, frame, ns * 2 * sizeof(int16_t));
	for (size_t p = 0; p < 1500 + ns + 200; p += 256) {
		size_t nb = (1500 + ns + 200) - p < 256 ? (1500 + ns + 200) - p : 256;

		ieee154_phy_rx_block(buf + 2 * p, nb);
	}
}

/*
 * A frame whose FCS does not match its payload must never reach the callback, while the same
 * frame with its FCS intact does (else a receiver that decodes nothing would pass).
 */
ZTEST(ieee154_phy, test_rejects_bad_fcs)
{
	uint8_t psdu[IEEE154_PHY_PSDU_MAX] = {0};
	size_t len;

	for (int i = 0; i < 40; i++) {
		psdu[i] = (uint8_t)rnd();
	}
	len = ieee154_phy_append_fcs(psdu, 40);
	memcpy(expect, psdu, len);
	expect_len = (uint8_t)len;

	got = 0;
	bad = 0;
	feed_frame(psdu, len);
	zassert_equal(got, 1, "the intact frame was not delivered (got %d)", got);

	psdu[len - 1] ^= 0xFFU; /* corrupt the FCS */
	got = 0;
	ieee154_phy_rx_reset();
	feed_frame(psdu, len);
	zassert_equal(got + bad, 0, "a frame with a bad FCS was delivered");
}

/* IEEE154_TABLES set to anything (even empty) enables the sweep tables. */
static bool tables_enabled(void)
{
	return getenv("IEEE154_TABLES") != NULL;
}

/*
 * Opt-in PER table: SNR 0 to 16 dB against three carrier offsets, 300 frames per point, with a
 * DC offset. Printed whole, then the claim of doc/ieee802154.md is asserted: no frame lost from
 * 6 dB up, none delivered twice, none with wrong content.
 */
ZTEST(ieee154_phy, test_per_table)
{
	static const double cfos[] = {0, 50e3, 100e3};
	const int nframes = 300;
	int total_dups = 0, lossy = 0;

	if (!tables_enabled()) {
		ztest_test_skip();
	}

	printk("%d frames/point, DC offset 100 LSB\n", nframes);
	printk("SNR dB |  PER @ CFO 0 | 50 kHz | 100 kHz   (dups)\n");
	for (int snr = 0; snr <= 16; snr += 2) {
		int row_dups = 0;

		printk("%6d |", snr);
		ARRAY_FOR_EACH(cfos, c) {
			int dups;
			double per = run_point(nframes, 1, snr, cfos[c], 100, &dups);

			row_dups += dups;
			if (snr >= 6 && per > 0) {
				lossy++;
			}
			printk(" %8.3f %s", per, c < ARRAY_SIZE(cfos) - 1 ? "|" : "");
		}
		printk("  (%d)\n", row_dups);
		total_dups += row_dups;
	}
	printk("bad (CRC-ok, wrong content): %d\n", bad);

	zassert_equal(lossy, 0, "%d table points at 6 dB SNR or more lost frames", lossy);
	zassert_equal(total_dups, 0, "%d duplicate deliveries", total_dups);
	zassert_equal(bad, 0, "%d CRC-ok frames with wrong content", bad);
}
