/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * CPU-paced streaming on the ESP32-C6 (esp_sdr_paced.h). Timing comes from the
 * CPU cycle counter; the caller polls with interrupts locked.
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>
#include <esp_sdr/esp_sdr_paced.h>

#include "esp_sdr_priv.h"

LOG_MODULE_DECLARE(esp_sdr, CONFIG_ESP_SDR_LOG_LEVEL);

#define CPU_HZ (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000U)
/* One estimator run (IQE_COUNT_MAX inputs at 80 MHz) lasts 65536 CPU cycles: restart well before. */
#define IQE_RUN_CYCLES 60000U
#define FIR_TAPS 8U
#define FIR_MAX_L 8U
#define TX_POWER_MAX 15U
/* librftest: tone backoff in quarter dB; the tone path also turns the TX chain on. */
#define TX_BACKOFF_STEP 8U

extern void esp_phy_wifi_tx_tone(uint32_t start, uint32_t chan, uint32_t backoff);

static struct esp_sdr_paced_stats stats;
static bool rx_on, tx_on;

static inline uint32_t cycles(void)
{
	return esp_sdr_paced_cycles();
}

static void cycles_enable(void)
{
#if defined(CONFIG_SOC_SERIES_ESP32C6)
	/* The machine performance counter (mpccr) counts CPU cycles. */
	__asm__ volatile("csrw 0x7e0, %0" ::"r"(1));
	__asm__ volatile("csrw 0x7e1, %0" ::"r"(1));
#endif
}

static inline void note_late(uint32_t late, uint32_t step)
{
	if (late > stats.late_max) {
		stats.late_max = late;
	}
	if (late >= step) {
		stats.late++;
	}
}

static struct {
	uint32_t step, target, rearm_every, left, ctrl1;
	int32_t pi, pq;
} rxp;

static void iqe_start(void)
{
	sys_write32(rxp.ctrl1 | IQE_CTRL1_EN, IQE_CTRL1_REG);
	sys_write32(rxp.ctrl1 | IQE_CTRL1_EN | IQE_CTRL1_START, IQE_CTRL1_REG);
}

int esp_sdr_rx_paced_start(uint32_t rate_hz)
{
	uint32_t c0;

	if (rate_hz < 10000U || CPU_HZ % rate_hz != 0U || ESP_SDR_PACED_INPUT_HZ % rate_hz != 0U) {
		return -EINVAL;
	}
	if (!esp_sdr_ready) {
		return -EAGAIN;
	}
	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	esp_sdr_rx_prepare();
	cycles_enable();
	memset(&stats, 0, sizeof(stats));
	rxp.step = CPU_HZ / rate_hz;
	rxp.rearm_every = MAX(1U, IQE_RUN_CYCLES / rxp.step);
	rxp.left = rxp.rearm_every;
	rxp.pi = 0;
	rxp.pq = 0;
	c0 = sys_read32(IQE_CTRL0_REG);
	sys_write32((c0 & ~IQE_CTRL0_MODE) | FIELD_PREP(IQE_CTRL0_MODE, 1), IQE_CTRL0_REG);
	rxp.ctrl1 = (sys_read32(IQE_CTRL1_REG) &
		     ~(IQE_CTRL1_SRC | IQE_CTRL1_COUNT | IQE_CTRL1_START | IQE_CTRL1_EN)) |
		    FIELD_PREP(IQE_CTRL1_SRC, 2) | FIELD_PREP(IQE_CTRL1_COUNT, IQE_COUNT_MAX - 1U);
	sys_write32(rxp.ctrl1, IQE_CTRL1_REG);
	sys_write32(rxp.ctrl1 | IQE_CTRL1_EN, IQE_CTRL1_REG);
	k_busy_wait(1);
	sys_write32(rxp.ctrl1 | IQE_CTRL1_EN | IQE_CTRL1_START, IQE_CTRL1_REG);
	rxp.target = cycles() + rxp.step;
	rx_on = true;
	return 0;
}

bool esp_sdr_rx_paced_poll(int32_t *i, int32_t *q)
{
	uint32_t now = cycles();
	int32_t si, sq, q1;

	if ((int32_t)(now - rxp.target) < 0) {
		return false;
	}
	note_late(now - rxp.target, rxp.step);
	/* Q before and after I: their mean is taken at the instant I is (no I/Q skew). */
	q1 = (int32_t)sys_read32(IQE_SUM_Q_REG) >> 6;
	si = (int32_t)sys_read32(IQE_SUM_I_REG) >> 6;
	sq = (q1 + ((int32_t)sys_read32(IQE_SUM_Q_REG) >> 6)) / 2;
	*i = si - rxp.pi;
	*q = sq - rxp.pq;
	rxp.pi = si;
	rxp.pq = sq;
	if (--rxp.left == 0U) {
		/* Restart before the count runs out; the next window starts from zero. */
		iqe_start();
		rxp.pi = 0;
		rxp.pq = 0;
		rxp.left = rxp.rearm_every;
		stats.events++;
	}
	rxp.target += rxp.step;
	stats.samples++;
	return true;
}

uint32_t esp_sdr_rx_paced_slack(void)
{
	int32_t d = (int32_t)(rxp.target - cycles());

	return d > 0 ? (uint32_t)d : 0U;
}

void esp_sdr_rx_paced_stop(void)
{
	if (!rx_on) {
		return;
	}
	sys_write32(rxp.ctrl1 | IQE_CTRL1_EN, IQE_CTRL1_REG);
	k_busy_wait(1);
	sys_write32(rxp.ctrl1, IQE_CTRL1_REG);
	rx_on = false;
	k_mutex_unlock(&esp_sdr_lock);
}

static struct {
	uint32_t step, target, ph, l, pos, upper, next, orig;
	esp_sdr_tx_paced_input_t input;
	void *user;
	uint32_t power, mhz;
	int16_t coef[FIR_MAX_L][FIR_TAPS];
	/* History, newest first, mirrored so that a window is contiguous. */
	int16_t hi[2 * FIR_TAPS], hq[2 * FIR_TAPS];
} txp;

/* Blackman-windowed sinc low-pass at the input Nyquist frequency, split into L phases (Q15). */
static void fir_design(uint32_t l)
{
	const double pi = 3.14159265358979323846;
	uint32_t n = l * FIR_TAPS;
	double h[FIR_MAX_L * FIR_TAPS], sum = 0.0;

	for (uint32_t k = 0; k < n; k++) {
		double x = ((double)k - (double)(n - 1U) / 2.0) / (double)l;

		h[k] = (fabs(x) < 1e-9 ? 1.0 : sin(pi * x) / (pi * x)) *
		       (0.42 - 0.5 * cos(2.0 * pi * k / (n - 1U)) + 0.08 * cos(4.0 * pi * k / (n - 1U)));
		sum += h[k];
	}
	for (uint32_t ph = 0; ph < l; ph++) {
		for (uint32_t k = 0; k < FIR_TAPS; k++) {
			txp.coef[ph][k] = (int16_t)lround(h[ph + k * l] * (double)l / sum * 32767.0);
		}
	}
}

unsigned int esp_sdr_tx_paced_power_max(void)
{
	return TX_POWER_MAX;
}

void esp_sdr_tx_paced_range(uint32_t *min_mhz, uint32_t *max_mhz)
{
#if defined(CONFIG_SOC_SERIES_ESP32C5)
	/* The C5's tone path takes any MHz, 2.4 and 5 GHz. */
	*min_mhz = 2400U;
	*max_mhz = 5900U;
#else
	*min_mhz = 2400U;
	*max_mhz = 2500U;
#endif
}

int esp_sdr_tx_paced_start(uint32_t rate_hz, unsigned int interp, unsigned int power,
			   esp_sdr_tx_paced_input_t input, void *user)
{
	uint32_t min_mhz, max_mhz;

	if (rate_hz == 0U || !(interp == 1U || interp == 2U || interp == 4U || interp == 8U) ||
	    CPU_HZ % (rate_hz * interp) != 0U || power > TX_POWER_MAX || input == NULL) {
		return -EINVAL;
	}
	if (!esp_sdr_ready) {
		return -EAGAIN;
	}
	esp_sdr_tx_paced_range(&min_mhz, &max_mhz);
	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	if (esp_sdr_freq_mhz < min_mhz || esp_sdr_freq_mhz > max_mhz) {
		k_mutex_unlock(&esp_sdr_lock);
		return -ERANGE;
	}
	/*
	 * The vendor CW turns the transmit chain on (pbus debug mode, TX clock,
	 * calibrated DC) on the channel holding the frequency; stopping its tone
	 * hands the DAC to the data path and keeps the chain on. Then only the PLL
	 * moves to the exact frequency: a second channel setup would undo the
	 * front-end forcing.
	 */
	txp.mhz = esp_sdr_freq_mhz;
	txp.power = power;
	esp_phy_wifi_tx_tone(1, txp.mhz, (TX_POWER_MAX - power) * TX_BACKOFF_STEP);
	phy_stop_tx_tone(1);
#if defined(CONFIG_SOC_SERIES_ESP32C5)
	/* The tone path tuned the exact MHz; the kHz offset as esp_sdr_tune() applies it. */
	if (esp_sdr_fofs_khz != 0) {
		phy_set_rf_freq_offset(phy_param[PHY_PARAM_XTAL], txp.mhz, esp_sdr_fofs_khz);
	}
#else
	phy_set_freq(txp.mhz, esp_sdr_fofs_khz);
#endif
	cycles_enable();
	memset(&stats, 0, sizeof(stats));
	fir_design(interp);
	memset(txp.hi, 0, sizeof(txp.hi));
	memset(txp.hq, 0, sizeof(txp.hq));
	txp.l = interp;
	txp.ph = 0;
	txp.pos = 0;
	txp.input = input;
	txp.user = user;
	txp.orig = sys_read32(TXDC_REG);
	txp.upper = (txp.orig & ~TXDC_IQ) | TXDC_INJECT;
	txp.next = txp.upper;
	txp.step = CPU_HZ / (rate_hz * interp);
	txp.target = cycles() + txp.step;
	tx_on = true;
	return 0;
}

static inline uint32_t iq10(int32_t i, int32_t q)
{
	return ((uint32_t)CLAMP(i, -511, 511) & 0x3ffU) | (((uint32_t)CLAMP(q, -511, 511) & 0x3ffU) << 10);
}

bool esp_sdr_tx_paced_poll(void)
{
	uint32_t now = cycles();
	const int16_t *c, *xi, *xq;
	int32_t ai, aq;

	if ((int32_t)(now - txp.target) < 0) {
		return false;
	}
	note_late(now - txp.target, txp.step);
	sys_write32(txp.next, TXDC_REG);
	if (txp.ph == 0U) {
		int16_t i = 0, q = 0;

		if (!txp.input(txp.user, &i, &q)) {
			i = 0;
			q = 0;
			stats.events++;
		}
		stats.samples++;
		txp.pos = txp.pos == 0U ? FIR_TAPS - 1U : txp.pos - 1U;
		/* Full scale 32767 to the DAC's 511. */
		txp.hi[txp.pos] = txp.hi[txp.pos + FIR_TAPS] = (int16_t)(i >> 6);
		txp.hq[txp.pos] = txp.hq[txp.pos + FIR_TAPS] = (int16_t)(q >> 6);
	}
	if (txp.l == 1U) {
		/* No interpolation: the input goes to the DAC as it is. */
		txp.next = txp.upper | iq10(txp.hi[txp.pos], txp.hq[txp.pos]);
		txp.target += txp.step;
		return true;
	}
	c = txp.coef[txp.ph];
	xi = &txp.hi[txp.pos];
	xq = &txp.hq[txp.pos];
	ai = c[0] * xi[0] + c[1] * xi[1] + c[2] * xi[2] + c[3] * xi[3] + c[4] * xi[4] +
	     c[5] * xi[5] + c[6] * xi[6] + c[7] * xi[7];
	aq = c[0] * xq[0] + c[1] * xq[1] + c[2] * xq[2] + c[3] * xq[3] + c[4] * xq[4] +
	     c[5] * xq[5] + c[6] * xq[6] + c[7] * xq[7];
	txp.next = txp.upper | iq10(ai >> 15, aq >> 15);
	txp.ph = (txp.ph + 1U) & (txp.l - 1U);
	txp.target += txp.step;
	return true;
}

uint32_t esp_sdr_tx_paced_slack(void)
{
	int32_t d = (int32_t)(txp.target - cycles());

	return d > 0 ? (uint32_t)d : 0U;
}

void esp_sdr_tx_paced_stop(void)
{
	if (!tx_on) {
		return;
	}
	sys_write32(txp.orig, TXDC_REG);
	esp_phy_wifi_tx_tone(0, txp.mhz, (TX_POWER_MAX - txp.power) * TX_BACKOFF_STEP);
	tx_on = false;
	/* Back to receive at the frequency in use. */
	(void)esp_sdr_retune();
	k_mutex_unlock(&esp_sdr_lock);
}
void esp_sdr_paced_sync(void)
{
	if (tx_on) {
		txp.target = cycles() + txp.step;
		return;
	}
	rxp.target = cycles() + rxp.step;
}

uint32_t esp_sdr_paced_deadline(void)
{
	if (tx_on) {
		return txp.target;
	}
	return rxp.target;
}

void esp_sdr_paced_get_stats(struct esp_sdr_paced_stats *s)
{
	*s = stats;
}
