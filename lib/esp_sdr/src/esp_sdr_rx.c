/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Receive: gain, analog low-pass, burst and decimated capture.
 */

#include <errno.h>
#include <limits.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>

/* Upstream helpers, used unmodified. */
#include "rx_bandwidth.h"
#if !defined(CONFIG_SOC_SERIES_ESP32C5)
#include "rx_lo.h"
#endif

#include "esp_sdr_priv.h"

LOG_MODULE_DECLARE(esp_sdr, CONFIG_ESP_SDR_LOG_LEVEL);

static int lpf_code = ESP_SDR_RX_LPF_AUTO;
static int gain_index = ESP_SDR_RX_GAIN_AUTO;
static uint32_t lpf_saved[2];

static unsigned int gain_max(void)
{
	unsigned int max = FIELD_GET(AGC_GAIN_MAX, REG_READ(AGC_FORCE_REG));

	return max <= AGC_GAIN_MAX_VALID ? max : 0U;
}

#if defined(CONFIG_ESP_SDR_RFTEST)
/* When gain_apply() last ran (k_uptime_get()). */
static int64_t gain_applied_ms;
#endif

static void gain_apply(void)
{
#if defined(CONFIG_ESP_SDR_RFTEST)
	bool manual = gain_index >= 0;
	unsigned int index = manual ? MIN((unsigned int)gain_index, gain_max())
				    : MIN(AGC_DEFAULT_INDEX, gain_max());
	uint32_t t0 = k_cycle_get_32();

	/* Releasing the force (manual false) hands control back to the AGC. */
	force_rx_gain(manual, index, 0);
	esp_sdr_counters.gain_apply_us = k_cyc_to_us_ceil32(k_cycle_get_32() - t0);
	gain_applied_ms = k_uptime_get();
#endif
}

/*
 * A forced gain is occasionally lost (measured, cause unknown: after some
 * thousand transmissions; the AGC then takes over and, with nothing on air,
 * raises the noise floor by up to 30 dB, until the next turnaround forces it
 * again). Re-forced at a capture when the last time is more than 20 ms ago.
 */
#define GAIN_REFRESH_MS 20

bool esp_sdr_rx_gain_refresh(void)
{
#if defined(CONFIG_ESP_SDR_RFTEST)
	if (gain_index >= 0 && k_uptime_get() - gain_applied_ms > GAIN_REFRESH_MS) {
		esp_sdr_counters.gain_refreshed++;
		gain_apply();
		return true;
	}
#endif
	return false;
}

static void lpf_apply(void);

static void rx_front_end(bool retune, uint32_t settle_us)
{
	if (retune) {
		esp_sdr_tune();
	}
	phy_stop_tx_tone(1);
#if defined(CONFIG_ESP_SDR_RFTEST) && defined(CONFIG_SOC_SERIES_ESP32S3)
	force_txon_mode(0, 0, 0);
#endif
	phy_pbus_workmode();
	phy_pbus_xpd_tx_off();
	phy_pbus_xpd_rx_on(1);
	phy_set_rxclk_en(1);
	gain_apply();
#if !defined(CONFIG_SOC_SERIES_ESP32C5)
	/* The 5/6 LO divider is selected after RX setup, as upstream. */
	regi2c_enter_critical();
	rx_lo_select(esp_sdr_lo_divided());
	regi2c_exit_critical();
#endif
	lpf_apply();
	k_busy_wait(settle_us);
}

void esp_sdr_rx_prepare(void)
{
	rx_front_end(true, SDR_RETUNE_SETTLE_US);
}

void esp_sdr_rx_resume(void)
{
	rx_front_end(esp_sdr_turn_retune, esp_sdr_turn_settle_us);
}

/*
 * The filter code is written at each front end setup and when it changes,
 * not around every capture: each I2C access costs tens of microseconds. The
 * PHY's own setting is read once, when leaving auto, and restored on return.
 */
static bool lpf_forced;

static void lpf_apply_locked(void);

static void lpf_apply(void)
{
	regi2c_enter_critical();
	lpf_apply_locked();
	regi2c_exit_critical();
}

static void lpf_apply_locked(void)
{
	if (lpf_code < 0) {
		if (lpf_forced) {
			for (unsigned int j = 0; j < ARRAY_SIZE(lpf_saved); j++) {
				rom_chip_i2c_writeReg(BBTOP_BLOCK, BBTOP_HOST, BBTOP_LPF_REG + j,
						      lpf_saved[j]);
			}
			lpf_forced = false;
		}
		return;
	}
	for (unsigned int j = 0; j < ARRAY_SIZE(lpf_saved); j++) {
		unsigned int reg = BBTOP_LPF_REG + j;

		if (!lpf_forced) {
			lpf_saved[j] = rom_chip_i2c_readReg(BBTOP_BLOCK, BBTOP_HOST, reg);
		}
		rom_chip_i2c_writeReg(BBTOP_BLOCK, BBTOP_HOST, reg,
				      (lpf_saved[j] & ~BBTOP_LPF_DCAP) | (unsigned int)lpf_code);
	}
	lpf_forced = true;
}

int esp_sdr_rx_set_gain(int index)
{
	int ret;

	if (index < ESP_SDR_RX_GAIN_AUTO) {
		return -EINVAL;
	}
	if (index >= 0 && !IS_ENABLED(CONFIG_ESP_SDR_RFTEST)) {
		return -ENOTSUP;
	}
	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	if (index > (int)gain_max()) {
		ret = -EINVAL;
	} else {
		/* As upstream: a forced index alone can leave stale RX state. */
		gain_index = index;
		ret = esp_sdr_retune();
	}
	k_mutex_unlock(&esp_sdr_lock);
	return ret;
}

int esp_sdr_rx_get_gain(void)
{
	return gain_index;
}

int esp_sdr_rx_gain_max(void)
{
	return (int)gain_max();
}

int esp_sdr_rx_set_bandwidth(uint32_t mhz)
{
	if (mhz != 0U && (mhz < RX_BANDWIDTH_MIN || mhz > RX_BANDWIDTH_MAX)) {
		return -EINVAL;
	}
#if defined(CONFIG_SOC_SERIES_ESP32C5)
	/* As upstream: above 23 MHz (or open) the PHY's wide path, a full channel setup. */
	int ret = esp_sdr_set_channel_bw(rx_bandwidth_phy_mode(mhz));

	if (ret != 0 && ret != -EAGAIN) {
		return ret;
	}
#endif
	return esp_sdr_rx_set_lpf(rx_bandwidth_dcap(mhz));
}

void esp_sdr_rx_bandwidth_range(uint32_t *min, uint32_t *max)
{
	*min = RX_BANDWIDTH_MIN;
	*max = RX_BANDWIDTH_MAX;
}

int esp_sdr_rx_set_lpf(int code)
{
	if (code < ESP_SDR_RX_LPF_AUTO || code > ESP_SDR_RX_LPF_MAX) {
		return -EINVAL;
	}
	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	lpf_code = code;
	if (esp_sdr_ready) {
		lpf_apply();
	}
	k_mutex_unlock(&esp_sdr_lock);
	return 0;
}

int esp_sdr_rx_get_lpf(void)
{
	return lpf_code;
}

uint32_t esp_sdr_rx_rate_hz(enum esp_sdr_rate rate)
{
	switch (rate) {
	case ESP_SDR_RATE_80MSPS:
		return 80000000U;
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	case ESP_SDR_RATE_40MSPS:
		return 40000000U;
	case ESP_SDR_RATE_16MSPS:
		return 16000000U;
#elif defined(CONFIG_SOC_SERIES_ESP32C5)
	case ESP_SDR_RATE_40MSPS:
		return 40000000U;
	case ESP_SDR_RATE_20MSPS:
		return 20000000U;
	case ESP_SDR_RATE_10MSPS:
		return 10000000U;
	case ESP_SDR_RATE_8MSPS:
		return 8000000U;
	case ESP_SDR_RATE_4MSPS:
		return 4000000U;
#endif
	default:
		return 0U;
	}
}

#if defined(CONFIG_SOC_SERIES_ESP32S3)
static uint32_t rate_bits(enum esp_sdr_rate rate)
{
	switch (rate) {
	case ESP_SDR_RATE_40MSPS:
		return DUMP_CTRL_40MSPS;
	case ESP_SDR_RATE_16MSPS:
		return DUMP_CTRL_16MSPS;
	default:
		return 0U;
	}
}
#endif

/* Point the engine at the receive I/Q; returns the saved bank usage. */
static uint32_t dump_arm(uint32_t usage)
{
	uint32_t saved = REG_READ(DUMP_USAGE_REG);

	REG_WRITE(DUMP_CTRL_REG, 0);
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	REG_WRITE(DUMP_CONFIG_REG, DUMP_CONFIG_IQ);
	REG_WRITE(DUMP_USAGE_REG, (saved & ~DUMP_USAGE_M) | usage);
#else
#if defined(CONFIG_SOC_SERIES_ESP32C5)
	REG_WRITE(DUMP_CLK_FORCE_REG, UINT32_MAX);
	REG_SET_BIT(DUMP_CLK_EN_REG, DUMP_CLK_EN_BIT);
#else
	REG_WRITE(DUMP_CLK_FORCE0_REG, UINT32_MAX);
	REG_WRITE(DUMP_CLK_FORCE2_REG, DUMP_CLK_FORCE2);
	REG_WRITE(DUMP_CLK_FORCE1_REG, UINT32_MAX);
#endif
	REG_WRITE(DUMP_CONFIG_REG, (REG_READ(DUMP_CONFIG_REG) & ~DUMP_CONFIG_LANES) | DUMP_CONFIG_IQ);
	REG_WRITE(DUMP_MODE_REG, (REG_READ(DUMP_MODE_REG) & ~DUMP_MODE_SOURCE) |
					 FIELD_PREP(DUMP_MODE_SOURCE, DUMP_SOURCE_IQ));
#if !defined(CONFIG_SOC_SERIES_ESP32C5)
	REG_CLR_BIT(DUMP_GATE_REG, DUMP_GATE_BIT);
#endif
	REG_WRITE(DUMP_USAGE_REG, (saved & ~DUMP_USAGE_M) | usage);
	/* The bank switch must land before the engine starts writing. */
	__asm__ volatile("fence rw, rw" ::: "memory");
	(void)REG_READ(DUMP_USAGE_REG);
#endif
	return saved;
}

static void dump_disarm(uint32_t saved)
{
	REG_WRITE(DUMP_CTRL_REG, 0);
	REG_WRITE(DUMP_USAGE_REG, saved);
#if !defined(CONFIG_SOC_SERIES_ESP32S3)
	__asm__ volatile("fence rw, rw" ::: "memory");
	(void)REG_READ(DUMP_USAGE_REG);
#endif
}

/* One capture into the bank; caller holds esp_sdr_lock. t0 is the cycle count at arming. */
static int capture_locked(enum esp_sdr_rate rate, size_t count, volatile uint32_t *words,
			  uint32_t usage, uint32_t *elapsed_us, uint64_t *t0)
{
	uint32_t saved, ctrl, start, elapsed;
	bool done;

	/*
	 * The engine fills the bank in order: a sentinel left in the first or
	 * last word means a short capture. Checking every word cost 1.2 ms for
	 * a full bank.
	 */
	words[0] = SENTINEL;
	words[count - 1] = SENTINEL;

	(void)esp_sdr_rx_gain_refresh();
	*t0 = k_cycle_get_64();
	start = (uint32_t)*t0;
	saved = dump_arm(usage);

#if defined(CONFIG_SOC_SERIES_ESP32S3)
	ctrl = DUMP_CTRL_RUN | rate_bits(rate) | FIELD_PREP(DUMP_CTRL_COUNT, count);
#else
#if defined(CONFIG_SOC_SERIES_ESP32C5)
	/* The enum values are the engine's rate indices. */
	REG_WRITE(DUMP_MODE_REG, (REG_READ(DUMP_MODE_REG) & ~DUMP_MODE_RATE) |
					 FIELD_PREP(DUMP_MODE_RATE, (uint32_t)rate));
#else
	ARG_UNUSED(rate);
#endif
	ctrl = DUMP_CTRL_RUN | FIELD_PREP(DUMP_CTRL_COUNT, count);
	/* As upstream: clear a stale done flag before running. */
	REG_WRITE(DUMP_CTRL_REG, FIELD_PREP(DUMP_CTRL_COUNT, count) | DUMP_CTRL_DONE);
#endif
	REG_WRITE(DUMP_CTRL_REG, ctrl);
	REG_WRITE(DUMP_CTRL_REG, ctrl | DUMP_CTRL_TRIGGER);
	REG_WRITE(DUMP_CTRL_REG, ctrl);
	do {
		done = (REG_READ(DUMP_CTRL_REG) & DUMP_CTRL_DONE) != 0U;
		elapsed = k_cyc_to_us_floor32(k_cycle_get_32() - start);
	} while (!done && elapsed < DUMP_TIMEOUT_US);

	dump_disarm(saved);

	if (!done) {
		return -ETIMEDOUT;
	}
	if (words[0] == SENTINEL || words[count - 1] == SENTINEL) {
		return -ETIMEDOUT;
	}
	*elapsed_us = elapsed;
	return 0;
}

int esp_sdr_rx_capture(enum esp_sdr_rate rate, size_t count, struct esp_sdr_rx_burst *burst)
{
	uint32_t usage, elapsed;
	uint64_t t0;
	int ret;

	if (esp_sdr_rx_rate_hz(rate) == 0U || count < ESP_SDR_SAMPLES_MIN ||
	    count > ESP_SDR_SAMPLES_MAX || esp_sdr_bank_usage(&usage) != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	ret = esp_sdr_ready ? capture_locked(rate, count, __esp_sdr_bank_start, usage, &elapsed, &t0)
			: -EAGAIN;
	if (ret == 0) {
		burst->words = __esp_sdr_bank_start;
		burst->count = count;
		burst->elapsed_us = elapsed;
	}
	k_mutex_unlock(&esp_sdr_lock);
	return ret;
}

int esp_sdr_rx_capture_iq(enum esp_sdr_rate rate, size_t count, struct esp_sdr_iq16 *out,
			  uint64_t *first_ns)
{
	uint32_t usage, elapsed;
	uint64_t t0;
	int ret;

	if (esp_sdr_rx_rate_hz(rate) == 0U || count < ESP_SDR_SAMPLES_MIN ||
	    count > ESP_SDR_SAMPLES_MAX || esp_sdr_bank_usage(&usage) != 0) {
		return -EINVAL;
	}
	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	ret = esp_sdr_ready ? capture_locked(rate, count, __esp_sdr_bank_start, usage, &elapsed, &t0)
			: -EAGAIN;
	if (ret == 0) {
		const uint32_t *w = __esp_sdr_bank_start;

		/* Still under the lock: a transmit session or capture would reuse the bank. */
		for (size_t j = 0; j < count; j++) {
			out[j].i = esp_sdr_rx_i(w[j]);
			out[j].q = esp_sdr_rx_q(w[j]);
		}
		*first_ns = k_cyc_to_ns_floor64(t0);
	}
	k_mutex_unlock(&esp_sdr_lock);
	return ret;
}

int esp_sdr_rx_capture_bank(enum esp_sdr_rate rate, size_t count, int bank,
			    struct esp_sdr_rx_burst *burst)
{
	uint32_t usage, elapsed;
	uint32_t *words;
	uint64_t t0;
	int ret;

	if (bank == 0) {
		return esp_sdr_rx_capture(rate, count, burst);
	}
	if (bank != 1 || ESP_SDR_BANKS < 2 || esp_sdr_rx_rate_hz(rate) == 0U ||
	    count < ESP_SDR_SAMPLES_MIN || count > ESP_SDR_SAMPLES_MAX) {
		return -EINVAL;
	}
#if defined(CONFIG_ESP_SDR_BANK1)
	words = __esp_sdr_bank1_start;
#else
	words = NULL;
#endif
	usage = BIT(1) << DUMP_USAGE_S;

	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	ret = esp_sdr_ready ? capture_locked(rate, count, words, usage, &elapsed, &t0) : -EAGAIN;
	if (ret == 0) {
		burst->words = words;
		burst->count = count;
		burst->elapsed_us = elapsed;
	}
	k_mutex_unlock(&esp_sdr_lock);
	return ret;
}

#if defined(CONFIG_ESP_SDR_RX_DECIM)
#define CIC_STAGES 3

/*
 * Third order CIC decimator over the bank, m input samples per output. Wraps
 * in 32 bits by design (only differences matter); exact while 512 * m^3 fits,
 * hence ESP_SDR_RX_DECIM_MAX. Output scaled so the 10-bit full scale is +-32767.
 */
__attribute__((optimize("O3"))) static size_t cic(const volatile uint32_t *vwords,
						  size_t count, unsigned int m,
						  struct esp_sdr_iq16 *out, size_t max_out)
{
	const uint32_t *words = (const uint32_t *)vwords;
	int32_t ii1 = 0, ii2 = 0, ii3 = 0, qi1 = 0, qi2 = 0, qi3 = 0;
	int32_t ic[CIC_STAGES] = {0}, qc[CIC_STAGES] = {0};
	/* Output gain m^3 down to 32767 / 512: one int64 multiply per output. */
	int64_t scale = ((int64_t)64 << 32) / ((int64_t)m * m * m);
	size_t n = 0, outs = count / m;

	for (size_t k = 0; k < outs; k++) {
		const uint32_t *w = &words[k * m];

		for (unsigned int j = 0; j < m; j++) {
			/* Receive words: I in bits 19:10, Q in bits 9:0. */
			ii1 += (int32_t)(w[j] << 12) >> 22;
			qi1 += (int32_t)(w[j] << 22) >> 22;
			ii2 += ii1;
			qi2 += qi1;
			ii3 += ii2;
			qi3 += qi2;
		}
		int32_t yi = ii3, yq = qi3;

		for (int s = 0; s < CIC_STAGES; s++) {
			int32_t ti = yi - ic[s], tq = yq - qc[s];

			ic[s] = yi;
			qc[s] = yq;
			yi = ti;
			yq = tq;
		}
		/* The first outputs still see the integrators settling. */
		if (k >= CIC_STAGES && n < max_out) {
			out[n].i = (int16_t)CLAMP((yi * scale) >> 32, INT16_MIN, INT16_MAX);
			out[n].q = (int16_t)CLAMP((yq * scale) >> 32, INT16_MIN, INT16_MAX);
			n++;
		}
	}
	return n;
}

int esp_sdr_rx_capture_decimated(enum esp_sdr_rate rate, size_t count, unsigned int m,
			      struct esp_sdr_iq16 *out, size_t max_out, size_t *n_out,
			      uint64_t *first_ns)
{
	uint32_t usage, elapsed;
	uint64_t t0;
	int ret;

	if (esp_sdr_rx_rate_hz(rate) == 0U || count < ESP_SDR_SAMPLES_MIN ||
	    count > ESP_SDR_SAMPLES_MAX || m < ESP_SDR_RX_DECIM_MIN || m > ESP_SDR_RX_DECIM_MAX ||
	    count / m <= CIC_STAGES || esp_sdr_bank_usage(&usage) != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	ret = esp_sdr_ready ? capture_locked(rate, count, __esp_sdr_bank_start, usage, &elapsed, &t0)
			: -EAGAIN;
	if (ret == 0) {
		/* Still under the lock: a transmit session would overwrite the bank. */
		*n_out = cic(__esp_sdr_bank_start, count, m, out, max_out);
		/* Output k is centred 1.5 (m - 1) samples before input (k + 1) m - 1. */
		*first_ns = k_cyc_to_ns_floor64(t0) +
			    ((uint64_t)(CIC_STAGES + 1U) * m - 1U - (3U * (m - 1U)) / 2U) *
				    NSEC_PER_SEC / esp_sdr_rx_rate_hz(rate);
	}
	k_mutex_unlock(&esp_sdr_lock);
	return ret;
}
/*
 * Decimation in frequency: sum n time-shifted blocks of count / n words
 * each, averaged back to the raw scale. Equivalent to picking every n-th bin
 * of a count-point FFT and taking a count/n-point IFFT, without computing
 * either. No settling delay (unlike the CIC): every output is a full sum.
 */
__attribute__((optimize("O3"))) static size_t fold(const volatile uint32_t *vwords, size_t count,
						    unsigned int n, struct esp_sdr_iq16 *out,
						    size_t max_out)
{
	const uint32_t *words = (const uint32_t *)vwords;
	size_t period = count / n;
	size_t outs = MIN(period, max_out);

	for (size_t k = 0; k < outs; k++) {
		int32_t si = 0, sq = 0;

		for (unsigned int j = 0; j < n; j++) {
			uint32_t w = words[k + j * period];

			/* Receive words: I in bits 19:10, Q in bits 9:0. */
			si += (int32_t)(w << 12) >> 22;
			sq += (int32_t)(w << 22) >> 22;
		}
		out[k].i = (int16_t)(si / (int32_t)n);
		out[k].q = (int16_t)(sq / (int32_t)n);
	}
	return outs;
}

int esp_sdr_rx_capture_folded(enum esp_sdr_rate rate, size_t count, unsigned int n,
			      struct esp_sdr_iq16 *out, size_t max_out, size_t *n_out,
			      uint64_t *first_ns)
{
	uint32_t usage, elapsed;
	uint64_t t0;
	int ret;

	if (esp_sdr_rx_rate_hz(rate) == 0U || count < ESP_SDR_SAMPLES_MIN ||
	    count > ESP_SDR_SAMPLES_MAX || n < ESP_SDR_RX_FOLD_MIN || n > ESP_SDR_RX_FOLD_MAX ||
	    count / n == 0U || esp_sdr_bank_usage(&usage) != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	ret = esp_sdr_ready ? capture_locked(rate, count, __esp_sdr_bank_start, usage, &elapsed, &t0)
			: -EAGAIN;
	if (ret == 0) {
		/* Still under the lock: a transmit session would overwrite the bank. */
		*n_out = fold(__esp_sdr_bank_start, count, n, out, max_out);
		*first_ns = k_cyc_to_ns_floor64(t0);
	}
	k_mutex_unlock(&esp_sdr_lock);
	return ret;
}
#endif /* CONFIG_ESP_SDR_RX_DECIM */

size_t esp_sdr_rx_pack_iq8(const uint32_t *words, size_t count, uint8_t *out)
{
	for (size_t j = 0; j < count; j++) {
		uint32_t w = words[j];

		out[2 * j] = (uint8_t)(w >> 12);
		out[2 * j + 1] = (uint8_t)(w >> 2);
	}
	return 2 * count;
}

size_t esp_sdr_rx_pack_iq10(const uint32_t *words, size_t count, uint8_t *out)
{
	uint8_t *p = out;

	for (size_t j = 0; j < count; j += 2, p += 5) {
		uint32_t a = words[j] & GENMASK(19, 0);
		uint32_t b = j + 1 < count ? words[j + 1] & GENMASK(19, 0) : 0U;

		p[0] = (uint8_t)a;
		p[1] = (uint8_t)(a >> 8);
		p[2] = (uint8_t)((a >> 16) | (b << 4));
		if (j + 1 < count) {
			p[3] = (uint8_t)(b >> 4);
			p[4] = (uint8_t)(b >> 12);
		}
	}
	return (count * 20U + 7U) / 8U;
}

#if defined(CONFIG_SOC_SERIES_ESP32S3)
/* TX baseband low-pass codes, inferred from the vendor PHY: 0x67 regs 0x0c/0x0d and 0x0e/0x0f. */
#define BBTOP_TX_LPF_A 0x0cU
#define BBTOP_TX_LPF_B 0x0eU

static int tx_lpf[2] = {ESP_SDR_RX_LPF_AUTO, ESP_SDR_RX_LPF_AUTO};
static uint32_t tx_lpf_saved[2][2];
static bool tx_lpf_forced[2];

/* Codes are written once and stay: no retune rewrites these registers. */
static void tx_lpf_apply_locked(void);

static void tx_lpf_apply(void)
{
	regi2c_enter_critical();
	tx_lpf_apply_locked();
	regi2c_exit_critical();
}

static void tx_lpf_apply_locked(void)
{
	static const unsigned int base[2] = {BBTOP_TX_LPF_A, BBTOP_TX_LPF_B};

	for (unsigned int p = 0; p < 2; p++) {
		for (unsigned int j = 0; j < 2; j++) {
			unsigned int reg = base[p] + j;

			if (tx_lpf[p] < 0) {
				if (tx_lpf_forced[p]) {
					rom_chip_i2c_writeReg(BBTOP_BLOCK, BBTOP_HOST, reg,
							      tx_lpf_saved[p][j]);
				}
				continue;
			}
			if (!tx_lpf_forced[p]) {
				tx_lpf_saved[p][j] = rom_chip_i2c_readReg(BBTOP_BLOCK, BBTOP_HOST, reg);
			}
			rom_chip_i2c_writeReg(BBTOP_BLOCK, BBTOP_HOST, reg,
					      (tx_lpf_saved[p][j] & ~BBTOP_LPF_DCAP) |
						      (unsigned int)tx_lpf[p]);
		}
		tx_lpf_forced[p] = tx_lpf[p] >= 0;
	}
}

int esp_sdr_tx_set_lpf(int code_a, int code_b)
{
	if (code_a < ESP_SDR_RX_LPF_AUTO || code_a > ESP_SDR_RX_LPF_MAX ||
	    code_b < ESP_SDR_RX_LPF_AUTO || code_b > ESP_SDR_RX_LPF_MAX) {
		return -EINVAL;
	}
	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	tx_lpf[0] = code_a;
	tx_lpf[1] = code_b;
	if (esp_sdr_ready) {
		tx_lpf_apply();
	}
	k_mutex_unlock(&esp_sdr_lock);
	return 0;
}

#else
int esp_sdr_tx_set_lpf(int code_a, int code_b)
{
	ARG_UNUSED(code_a);
	ARG_UNUSED(code_b);
	return -ENOTSUP;
}
#endif

int esp_sdr_bbtop_read(unsigned int reg)
{
	int v;

	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	regi2c_enter_critical();
	v = (int)rom_chip_i2c_readReg(BBTOP_BLOCK, BBTOP_HOST, reg);
	regi2c_exit_critical();
	k_mutex_unlock(&esp_sdr_lock);
	return v;
}

void esp_sdr_bbtop_write(unsigned int reg, unsigned int val)
{
	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	regi2c_enter_critical();
	rom_chip_i2c_writeReg(BBTOP_BLOCK, BBTOP_HOST, reg, val);
	regi2c_exit_critical();
	k_mutex_unlock(&esp_sdr_lock);
}

