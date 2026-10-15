/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Transmit through the dump engine's DAC side: power ladder, one-shot
 * playback and sweeps, and the DAC session the streaming backend uses.
 */

#include <errno.h>
#include <math.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>

#include <esp_attr.h>

/* Upstream helper, used unmodified. */
#include "rx_lo.h"

#include "esp_sdr_priv.h"

/* One physical LO selector feeds both mixers; named for the transmit side. */
#define tx_lo_select rx_lo_select

LOG_MODULE_DECLARE(esp_sdr, CONFIG_ESP_SDR_LOG_LEVEL);

/*
 * Vendor TX power ladder (phy_tx_gain.o .rodata, read by tx_gain_set()):
 * gain memory index, baseband gain code and target power per step, strongest
 * first. esp_sdr_tx_set_gain() counts the other way, 0 = weakest.
 */
static const struct {
	uint8_t gain;
	uint16_t bb;
	int16_t power;
} tx_ladder[] = {
	{127, 0x0a0, 48},  {127, 0x020, 40},  {127, 0x100, 32},  {127, 0x080, 24},
	{127, 0x000, 16},  {111, 0x000, 12},  {95, 0x000, 6},    {107, 0x000, 0},
	{119, 0x000, -7},  {87, 0x000, -16},  {115, 0x000, -30}, {82, 0x000, -53},
	{81, 0x000, -63},  {49, 0x000, -78},  {48, 0x000, -99},  {32, 0x000, -110},
	{16, 0x000, -124}, {0, 0x000, -148},
};
#define TX_GAIN_INDEX_MAX ((int)ARRAY_SIZE(tx_ladder) - 1)
/* Weakest step by default; raise deliberately, with the attenuated link connected. */
#define TX_GAIN_DEFAULT_INDEX 0

static int tx_gain_index = TX_GAIN_DEFAULT_INDEX;

/*
 * Mirror of esp_sdr_rx_prepare() for esp_sdr_tx_play(): swap the analog front end from
 * RX to TX before arming the DAC trigger, esp_sdr_rx_resume() afterwards; the
 * module's resting state is always RX-ready.
 */
/* Ladder step 0 (weakest) .. TX_GAIN_INDEX_MAX into gain memory slot 0. */
static void tx_gain_apply(int step)
{
#if defined(CONFIG_ESP_SDR_RFTEST)
	force_tx_gain(tx_ladder[TX_GAIN_INDEX_MAX - step].gain,
		      tx_ladder[TX_GAIN_INDEX_MAX - step].bb, 0);
#else
	ARG_UNUSED(step);
#endif
}

void esp_sdr_tx_prepare(void)
{
	esp_sdr_tx_front_end();
	k_busy_wait(esp_sdr_turn_settle_us);
}

void esp_sdr_tx_front_end(void)
{
	if (esp_sdr_turn_retune) {
		esp_sdr_tune();
	}
	rom_pbus_workmode();
	rom_pbus_xpd_rx_on(0);
	rom_pbus_xpd_tx_on(1);
	tx_gain_apply(tx_gain_index);
	/*
	 * esp_sdr_tune() leaves the 5/6 LO divider off; select it after the TX
	 * setup, as rx_front_end() does for RX, or the divider band goes out at 6/5.
	 */
	regi2c_enter_critical();
	tx_lo_select(esp_sdr_lo_divided());
	regi2c_exit_critical();
#if defined(CONFIG_ESP_SDR_RFTEST)
	/* Without a frame in flight the MAC never enables TX on its own. */
	force_txon_mode(1, 0, 0);
#endif
}

int esp_sdr_tx_set_gain(int index)
{
	if (!IS_ENABLED(CONFIG_ESP_SDR_RFTEST)) {
		return -ENOTSUP;
	}
	if (index < 0 || index > TX_GAIN_INDEX_MAX) {
		return -EINVAL;
	}
	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	/* Picked up by esp_sdr_tx_prepare() on the next esp_sdr_tx_play(); no immediate effect. */
	tx_gain_index = index;
	k_mutex_unlock(&esp_sdr_lock);
	return 0;
}

int esp_sdr_tx_get_gain(void)
{
	return tx_gain_index;
}

int esp_sdr_tx_gain_max(void)
{
	return TX_GAIN_INDEX_MAX;
}

int esp_sdr_tx_gain_power(int index)
{
	if (index < 0 || index > TX_GAIN_INDEX_MAX) {
		return INT_MIN;
	}
	return tx_ladder[TX_GAIN_INDEX_MAX - index].power;
}

/*
 * DAC-side trigger (dactrig() in librftest.a, minus its ramp-fill): plays
 * `words` out of the shared capture bank through the TX DAC. Swaps the
 * analog front end to TX for the duration (esp_sdr_tx_prepare()) and always leaves
 * it back in its resting RX-ready state (esp_sdr_rx_resume()) before returning.
 * Caller is still responsible for RF safety (dummy load / attenuated link).
 */
uint32_t esp_sdr_tx_rate_hz(enum esp_sdr_rate rate)
{
	switch (rate) {
	case ESP_SDR_RATE_80MSPS:
		return 80000000U;
	case ESP_SDR_RATE_40MSPS:
		return 40000000U;
	default:
		return 0U;
	}
}

static uint32_t dac_t0;

/* Start one DAC burst over the first `count` words of the bank the engine owns. */
static void dac_trigger(enum esp_sdr_rate rate, size_t count)
{
	uint32_t ctrl = DUMP_CTRL_RUN | (rate == ESP_SDR_RATE_80MSPS ? DAC_CTRL_80MSPS : 0U) |
			FIELD_PREP(DUMP_CTRL_COUNT, count);

	dac_t0 = k_cycle_get_32();
	REG_WRITE(DAC_TRIG_REG, 0);
	REG_WRITE(DAC_TRIG_REG, ctrl);
	REG_WRITE(DAC_TRIG_REG, ctrl | DUMP_CTRL_TRIGGER);
	REG_WRITE(DAC_TRIG_REG, ctrl);
}

/* True once the engine reports the burst done. */
static bool dac_wait_done(void)
{
	bool done;

	do {
		done = (REG_READ(DAC_TRIG_REG) & DUMP_CTRL_DONE) != 0U;
	} while (!done && k_cyc_to_us_floor32(k_cycle_get_32() - dac_t0) < DUMP_TIMEOUT_US);
	return done;
}

static bool dac_burst(enum esp_sdr_rate rate, size_t count)
{
	dac_trigger(rate, count);
	return dac_wait_done();
}

int esp_sdr_tx_play_for(enum esp_sdr_rate rate, const uint32_t *words, size_t count,
		     uint32_t duration_ms, uint32_t *bursts)
{
	volatile uint32_t *bank = __esp_sdr_bank_start;
	uint32_t usage, saved, n = 0;
	size_t len = count;
	int64_t end;
	bool done;
	int ret = 0;

	if (esp_sdr_tx_rate_hz(rate) == 0U || count < ESP_SDR_SAMPLES_MIN ||
	    count > ESP_SDR_SAMPLES_MAX || duration_ms > ESP_SDR_TX_PLAY_MAX_MS ||
	    esp_sdr_bank_usage(&usage) != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	if (!esp_sdr_ready) {
		ret = -EAGAIN;
		goto out;
	}

	/* Each retrigger leaves a ~1.35 us hole: tile the burst over the bank to
	 * make the holes rare.
	 */
	if (duration_ms > 0U) {
		len = ESP_SDR_SAMPLES_MAX / count * count;
	}
	for (size_t j = 0; j < len; j++) {
		bank[j] = words[j % count];
	}
	count = len;

	/* Front end and bank switch once, then retrigger back to back: the
	 * switch costs ~6 ms, a burst is at most ~1 ms.
	 */
	esp_sdr_tx_prepare();
	saved = REG_READ(SENSITIVE_INTERNAL_SRAM_USAGE_3_REG);
	REG_WRITE(SENSITIVE_INTERNAL_SRAM_USAGE_3_REG,
		  (saved & ~SENSITIVE_INTERNAL_SRAM_MAC_DUMP_USAGE_M) | usage);

	end = k_uptime_get() + duration_ms;
	do {
		done = dac_burst(rate, count);
		n++;
	} while (done && k_uptime_get() < end);

	REG_WRITE(DAC_TRIG_REG, 0);
	REG_WRITE(DUMP_CTRL_REG, 0); /* dactrig() also clears the ADC-side register */
	REG_WRITE(SENSITIVE_INTERNAL_SRAM_USAGE_3_REG, saved);
	esp_sdr_rx_resume(); /* back to the module's resting state */

	if (!done) {
		ret = -ETIMEDOUT;
	}
	if (bursts != NULL) {
		*bursts = n;
	}
out:
	k_mutex_unlock(&esp_sdr_lock);
	return ret;
}


static enum esp_sdr_rate dac_rate;
static uint32_t dac_usage, dac_saved;

int esp_sdr_dac_begin(enum esp_sdr_rate rate)
{
	if (esp_sdr_tx_rate_hz(rate) == 0U || esp_sdr_bank_usage(&dac_usage) != 0) {
		return -EINVAL;
	}
	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	if (!esp_sdr_ready) {
		k_mutex_unlock(&esp_sdr_lock);
		return -EAGAIN;
	}
	dac_rate = rate;
	esp_sdr_tx_prepare();
	dac_saved = REG_READ(SENSITIVE_INTERNAL_SRAM_USAGE_3_REG);
	return 0;
}

uint32_t *esp_sdr_dac_buf(int idx, size_t *words)
{
	*words = DUMP_BANK_SIZE / sizeof(uint32_t);
#if defined(CONFIG_ESP_SDR_BANK1)
	if (idx != 0) {
		return (uint32_t *)__esp_sdr_bank1_start;
	}
#endif
	return (uint32_t *)__esp_sdr_bank_start;
}

void esp_sdr_dac_start(int idx, size_t count)
{
	/* One-hot: the engine gets this bank, the CPUs keep every other one. */
	uint32_t usage = idx == 0 ? dac_usage : BIT(1) << SENSITIVE_INTERNAL_SRAM_MAC_DUMP_USAGE_S;

	REG_WRITE(SENSITIVE_INTERNAL_SRAM_USAGE_3_REG,
		  (dac_saved & ~SENSITIVE_INTERNAL_SRAM_MAC_DUMP_USAGE_M) | usage);
	dac_trigger(dac_rate, count);
}

bool esp_sdr_dac_wait(void)
{
	return dac_wait_done();
}

IRAM_ATTR void esp_sdr_dac_select(int idx)
{
	uint32_t usage = idx == 0 ? dac_usage : BIT(1) << SENSITIVE_INTERNAL_SRAM_MAC_DUMP_USAGE_S;

	REG_WRITE(SENSITIVE_INTERNAL_SRAM_USAGE_3_REG,
		  (dac_saved & ~SENSITIVE_INTERNAL_SRAM_MAC_DUMP_USAGE_M) | usage);
}

void esp_sdr_dac_loop(int idx, size_t count)
{
	uint32_t ctrl = DUMP_CTRL_RUN | (dac_rate == ESP_SDR_RATE_80MSPS ? DAC_CTRL_80MSPS : 0U) |
			FIELD_PREP(DUMP_CTRL_COUNT, count);

	esp_sdr_dac_select(idx);
	REG_WRITE(DAC_TRIG_REG, 0);
	REG_WRITE(DAC_TRIG_REG, ctrl);
	/* Held, not pulsed: the engine wraps instead of stopping after one pass. */
	REG_WRITE(DAC_TRIG_REG, ctrl | DUMP_CTRL_TRIGGER);
}

IRAM_ATTR void esp_sdr_dac_halt(void)
{
	REG_WRITE(DAC_TRIG_REG, 0);
}

void esp_sdr_dac_end(void)
{
	REG_WRITE(DAC_TRIG_REG, 0);
	REG_WRITE(DUMP_CTRL_REG, 0);
	esp_sdr_counters.dac_sessions++;
	REG_WRITE(SENSITIVE_INTERNAL_SRAM_USAGE_3_REG, dac_saved);
	esp_sdr_rx_resume();
	k_mutex_unlock(&esp_sdr_lock);
}

int esp_sdr_tx_play(enum esp_sdr_rate rate, const uint32_t *words, size_t count)
{
	return esp_sdr_tx_play_for(rate, words, count, 0, NULL);
}

#define SWEEP_TWO_PI 6.28318530717958647692

/*
 * Fill bank[0..n) with a chirp starting at t (s): frequency f0 + k t. Phase
 * is taken from absolute time, so segments line up across the synthesis
 * gaps; per sample only a complex rotation.
 */
static void sweep_fill(volatile uint32_t *bank, size_t n, double t, double fs, double f0,
		       double k, float amp)
{
	double phase = fmod(SWEEP_TWO_PI * (f0 * t + 0.5 * k * t * t), SWEEP_TWO_PI);
	double f = f0 + k * t;
	float zr = cosf((float)phase), zi = sinf((float)phase);
	float wr = cosf((float)(SWEEP_TWO_PI * f / fs)), wi = sinf((float)(SWEEP_TWO_PI * f / fs));
	float cr = cosf((float)(SWEEP_TWO_PI * k / (fs * fs)));
	float ci = sinf((float)(SWEEP_TWO_PI * k / (fs * fs)));

	for (size_t j = 0; j < n; j++) {
		float tr;

		float vi = amp * zr, vq = amp * zi;

		/* Inline rounding: lroundf() is a library call and dominated the loop. */
		bank[j] = esp_sdr_tx_word((int16_t)CLAMP((int)(vi + (vi < 0.0f ? -0.5f : 0.5f)), -511, 511),
					  (int16_t)CLAMP((int)(vq + (vq < 0.0f ? -0.5f : 0.5f)), -511, 511));
		tr = zr * wr - zi * wi;
		zi = zr * wi + zi * wr;
		zr = tr;
		tr = wr * cr - wi * ci;
		wi = wr * ci + wi * cr;
		wr = tr;
	}
}

int esp_sdr_tx_sweep(enum esp_sdr_rate rate, float f0_hz, float f1_hz, uint32_t duration_ms,
		  int amp, struct esp_sdr_tx_sweep_stats *stats)
{
	volatile uint32_t *bank = __esp_sdr_bank_start;
	struct esp_sdr_tx_sweep_stats st = {0};
	double fs = (double)esp_sdr_tx_rate_hz(rate);
	double T = (double)duration_ms / 1e3;
	double k = (double)(f1_hz - f0_hz) / T;
	uint32_t usage, saved;
	uint64_t start;
	int ret = 0;

	if (fs == 0.0 || duration_ms == 0U || duration_ms > ESP_SDR_TX_PLAY_MAX_MS ||
	    (double)fabsf(f0_hz) >= fs / 2 || (double)fabsf(f1_hz) >= fs / 2 || amp < 1 ||
	    amp > 511 || esp_sdr_bank_usage(&usage) != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	if (!esp_sdr_ready) {
		ret = -EAGAIN;
		goto out;
	}
	esp_sdr_tx_prepare();
	saved = REG_READ(SENSITIVE_INTERNAL_SRAM_USAGE_3_REG);

	start = k_cycle_get_64();
	for (;;) {
		double t = (double)k_cyc_to_ns_floor64(k_cycle_get_64() - start) / 1e9;
		size_t n = MIN((size_t)ESP_SDR_SAMPLES_MAX, (size_t)((T - t) * fs));

		if (t >= T || n < ESP_SDR_SAMPLES_MIN) {
			break;
		}
		/* The engine only sees bank writes made while the CPU owns the bank. */
		REG_WRITE(SENSITIVE_INTERNAL_SRAM_USAGE_3_REG, saved);
		sweep_fill(bank, n, t, fs, f0_hz, k, (float)amp);
		REG_WRITE(SENSITIVE_INTERNAL_SRAM_USAGE_3_REG,
			  (saved & ~SENSITIVE_INTERNAL_SRAM_MAC_DUMP_USAGE_M) | usage);
		if (!dac_burst(rate, n)) {
			ret = -ETIMEDOUT;
			break;
		}
		st.segments++;
		st.samples += n;
	}
	st.elapsed_ms = (uint32_t)(k_cyc_to_ns_floor64(k_cycle_get_64() - start) / 1000000U);

	REG_WRITE(DAC_TRIG_REG, 0);
	REG_WRITE(DUMP_CTRL_REG, 0);
	REG_WRITE(SENSITIVE_INTERNAL_SRAM_USAGE_3_REG, saved);
	esp_sdr_rx_resume();
	if (stats != NULL) {
		*stats = st;
	}
out:
	k_mutex_unlock(&esp_sdr_lock);
	return ret;
}

int esp_sdr_tx_loop_begin(enum esp_sdr_rate rate)
{
	return esp_sdr_dac_begin(rate);
}

uint32_t *esp_sdr_tx_loop_buf(int bank, size_t *words)
{
	if (bank < 0 || bank >= ESP_SDR_BANKS) {
		return NULL;
	}
	return esp_sdr_dac_buf(bank, words);
}

int esp_sdr_tx_loop_start(int bank, size_t count)
{
	if (bank < 0 || bank >= ESP_SDR_BANKS || count < ESP_SDR_SAMPLES_MIN ||
	    count > ESP_SDR_SAMPLES_MAX) {
		return -EINVAL;
	}
	esp_sdr_dac_loop(bank, count);
	return 0;
}

void esp_sdr_tx_loop_halt(void)
{
	esp_sdr_dac_halt();
}

void esp_sdr_tx_loop_end(void)
{
	esp_sdr_dac_end();
}
