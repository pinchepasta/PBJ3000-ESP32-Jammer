/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file
 * @brief ESP-SDR transmit: power steps, one-shot DAC playback and sweeps, and
 * the streaming interface with its backends.
 *
 * One-shot calls (esp_sdr_tx_play(), esp_sdr_tx_sweep()) play prepared words
 * from the dump bank. The streaming interface takes complex 16-bit samples at
 * a low rate and hands them to a backend: the DAC backend
 * (CONFIG_ESP_SDR_TX_DAC) interpolates them to 40 MS/s and plays them
 * gapless, the stub backend only counts them.
 *
 * The one-shot, power and loop calls exist on the ESP32-S3 only, the streaming
 * interface with CONFIG_ESP_SDR_TX; the ESP32-C6 has esp_sdr_tx_set_lpf()
 * (returning -ENOTSUP) and esp_sdr_tx_word() only.
 */

#ifndef ESP_SDR_ESP_SDR_TX_H_
#define ESP_SDR_ESP_SDR_TX_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <esp_sdr/esp_sdr.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup esp_sdr_tx
 * @{
 */

#if defined(CONFIG_SOC_SERIES_ESP32S3)
/**
 * @brief Select a transmit power step, picked up by the next transmission.
 *
 * Steps follow the vendor PHY's TX power ladder (gain memory index plus
 * baseband gain code per step). TX has no AGC, so there is no "auto" step.
 *
 * @param index 0 (weakest) to esp_sdr_tx_gain_max() (strongest).
 * @retval 0 on success.
 * @retval -EINVAL if out of range.
 * @retval -ENOTSUP without CONFIG_ESP_SDR_RFTEST.
 */
int esp_sdr_tx_set_gain(int index);

/** @return Selected transmit power step. */
int esp_sdr_tx_get_gain(void);

/** @return Strongest transmit power step. */
int esp_sdr_tx_gain_max(void);

/**
 * @return The vendor's target power for @p index in quarter dB (relative, not
 * dBm; measured 4.24 units per dB over the ladder), INT_MIN if out of range.
 */
int esp_sdr_tx_gain_power(int index);
#endif

/**
 * @brief Select the TX baseband low-pass capacitor codes (experimental).
 *
 * The two code pairs the vendor PHY calibrates for transmit (inferred from
 * its TX setup; larger is narrower, as on the receive side). Written at
 * once and kept across retunes.
 *
 * @param code_a,code_b 0 to ESP_SDR_RX_LPF_MAX, or ESP_SDR_RX_LPF_AUTO for the PHY's code.
 * @retval 0 on success.
 * @retval -EINVAL if out of range.
 */
int esp_sdr_tx_set_lpf(int code_a, int code_b);

#if defined(CONFIG_SOC_SERIES_ESP32S3)
/** @return DAC sample rate in Hz, or 0 if the DAC cannot run at @p rate. */
uint32_t esp_sdr_tx_rate_hz(enum esp_sdr_rate rate);
#endif

/** @return Raw transmit word for esp_sdr_tx_play(): I in bits 9:0, Q in bits 19:10. */
static inline uint32_t esp_sdr_tx_word(int16_t i, int16_t q)
{
	return ((uint32_t)i & 0x3ffU) | (((uint32_t)q & 0x3ffU) << 10);
}

#if defined(CONFIG_SOC_SERIES_ESP32S3)
/**
 * @brief Play one burst of complex samples out through the TX DAC.
 *
 * Reverse direction of esp_sdr_rx_capture(): drives the dump engine's DAC
 * side through the same shared SRAM bank. Swaps the analog front end to TX
 * for the duration and restores RX afterwards. The caller must still have RF
 * safety in place (dummy load / attenuated link); this function does not
 * limit transmit power beyond the weakest default step.
 *
 * The DAC side has its own clock: only ESP_SDR_RATE_40MSPS and
 * ESP_SDR_RATE_80MSPS exist, see esp_sdr_tx_rate_hz().
 *
 * @param rate Transmit sample rate.
 * @param words Complex samples built with esp_sdr_tx_word().
 * @param count Complex samples, ESP_SDR_SAMPLES_MIN to ESP_SDR_SAMPLES_MAX.
 * @retval 0 on success.
 * @retval -EINVAL for an unsupported rate or count.
 * @retval -EAGAIN if esp_sdr_init() has not run.
 * @retval -ETIMEDOUT if the dump engine did not finish.
 */
int esp_sdr_tx_play(enum esp_sdr_rate rate, const uint32_t *words, size_t count);

/** Longest esp_sdr_tx_play_for() duration; the caller's thread busy-waits throughout. */
#define ESP_SDR_TX_PLAY_MAX_MS 5000U

/**
 * @brief Replay one burst back to back for a while, like esp_sdr_tx_play().
 *
 * Switches the front end once and retriggers the DAC until @p duration_ms has
 * passed (0: exactly one burst). With a duration, @p words is tiled over as
 * much of the bank as whole copies fit, so each retrigger (~1.35 us gap) comes
 * after up to ESP_SDR_SAMPLES_MAX samples. Seamless only if @p words holds a
 * whole number of signal periods. Holds the radio lock throughout, so
 * captures wait.
 *
 * @param rate Transmit sample rate.
 * @param words Complex samples built with esp_sdr_tx_word().
 * @param count Complex samples, ESP_SDR_SAMPLES_MIN to ESP_SDR_SAMPLES_MAX.
 * @param duration_ms 0 to ESP_SDR_TX_PLAY_MAX_MS.
 * @param bursts If not NULL, set to the number of bursts played.
 * @retval 0 on success.
 * @retval -EINVAL for an unsupported rate, count or duration.
 * @retval -EAGAIN if esp_sdr_init() has not run.
 * @retval -ETIMEDOUT if the dump engine did not finish a burst.
 */
int esp_sdr_tx_play_for(enum esp_sdr_rate rate, const uint32_t *words, size_t count,
			uint32_t duration_ms, uint32_t *bursts);

/** What an esp_sdr_tx_sweep() run did. */
struct esp_sdr_tx_sweep_stats {
	/** DAC bursts played. */
	uint32_t segments;
	/** Samples played; divide by the rate and elapsed_ms for the duty cycle. */
	uint32_t samples;
	/** Wall-clock run time in ms. */
	uint32_t elapsed_ms;
};

/**
 * @brief Transmit a linear frequency sweep at fixed power.
 *
 * Frequency goes linearly from @p f0_hz to @p f1_hz (constant Hz per second)
 * over @p duration_ms of wall-clock time, at the selected transmit step
 * (esp_sdr_tx_set_gain()) and digital amplitude @p amp. Each segment of up to
 * ESP_SDR_SAMPLES_MAX samples is synthesized into the bank while the DAC is
 * idle, so transmission pauses between segments; phase follows absolute time,
 * so the chirp stays continuous across the pauses. Holds the radio lock
 * throughout.
 *
 * @param rate Transmit sample rate, see esp_sdr_tx_rate_hz().
 * @param f0_hz,f1_hz Baseband offsets from the LO, below half the rate.
 * @param duration_ms 1 to ESP_SDR_TX_PLAY_MAX_MS.
 * @param amp Digital amplitude 1 to 511; the TX baseband compresses near 511.
 * @param stats If not NULL, filled with what was played.
 * @retval 0 on success.
 * @retval -EINVAL for an unsupported rate, frequency, duration or amplitude.
 * @retval -EAGAIN if esp_sdr_init() has not run.
 * @retval -ETIMEDOUT if the dump engine did not finish a burst.
 */
int esp_sdr_tx_sweep(enum esp_sdr_rate rate, float f0_hz, float f1_hz, uint32_t duration_ms,
		     int amp, struct esp_sdr_tx_sweep_stats *stats);

/**
 * @defgroup esp_sdr_tx_loop Loop playback
 *
 * Gapless playback of prepared banks at the DAC rate: the engine replays the
 * first count words of one bank back to back until halted. While it plays one
 * bank, the CPUs may write the other (CONFIG_ESP_SDR_BANK1), then restart on
 * it. A session holds the radio from esp_sdr_tx_loop_begin() to
 * esp_sdr_tx_loop_end(), so receive calls wait meanwhile.
 * @{
 */

/**
 * @brief Take the radio and switch the front end to TX at the selected step.
 *
 * @retval 0 on success.
 * @retval -EINVAL for an unsupported rate.
 * @retval -EAGAIN if esp_sdr_init() has not run.
 */
int esp_sdr_tx_loop_begin(enum esp_sdr_rate rate);

/**
 * @return Bank @p bank (0 to ESP_SDR_BANKS - 1) and its size in words through
 * @p words, NULL if there is no such bank. Write it with esp_sdr_tx_word()
 * words only while the engine plays another bank or is halted.
 */
uint32_t *esp_sdr_tx_loop_buf(int bank, size_t *words);

/**
 * @brief (Re)start looping over the first @p count words of @p bank.
 *
 * Restarting cuts the running pass; playback of the new one starts at once.
 *
 * @retval 0 on success.
 * @retval -EINVAL for a bad bank or count (ESP_SDR_SAMPLES_MIN to ESP_SDR_SAMPLES_MAX).
 */
int esp_sdr_tx_loop_start(int bank, size_t count);

/** @brief Stop playback at once; the session stays open. */
void esp_sdr_tx_loop_halt(void);

/** @brief Stop playback, back to RX, release the radio. */
void esp_sdr_tx_loop_end(void);

/** @} */
#endif /* CONFIG_SOC_SERIES_ESP32S3 */

/** Streaming backend operations. All may sleep; called from one thread. */
struct esp_sdr_tx_backend {
	/** Backend name for diagnostics. */
	const char *name;
	/** Prepare to transmit at @p freq_hz with @p rate_hz samples per second. */
	int (*start)(uint64_t freq_hz, uint32_t rate_hz);
	/** Accept up to @p count samples; returns how many were taken or a negative errno. */
	int (*write)(const struct esp_sdr_iq16 *samples, size_t count);
	/** Stop transmitting and drop anything queued. */
	int (*stop)(void);
};

/** Streaming statistics since the last start. */
struct esp_sdr_tx_stats {
	uint64_t samples;
	uint32_t writes;
	uint32_t errors;
};

#if defined(CONFIG_ESP_SDR_TX)
/**
 * @brief Select the streaming backend; NULL restores the default (DAC if built, else the stub).
 *
 * @retval 0 on success.
 * @retval -EBUSY while transmitting.
 */
int esp_sdr_tx_set_backend(const struct esp_sdr_tx_backend *backend);

/** @return The selected streaming backend. */
const struct esp_sdr_tx_backend *esp_sdr_tx_get_backend(void);

/**
 * @brief Start streaming.
 *
 * @retval 0 on success.
 * @retval -EALREADY if already started.
 * @retval -EINVAL for a zero rate.
 */
int esp_sdr_tx_start(uint64_t freq_hz, uint32_t rate_hz);

/**
 * @brief Hand samples to the backend.
 *
 * @return Samples taken, or a negative errno (-ENOTCONN when not started).
 */
int esp_sdr_tx_write(const struct esp_sdr_iq16 *samples, size_t count);

/** @brief Stop streaming; no-op when stopped. */
int esp_sdr_tx_stop(void);

/** @return Whether streaming is started. */
bool esp_sdr_tx_active(void);

/** Copy the streaming statistics into @p stats. */
void esp_sdr_tx_get_stats(struct esp_sdr_tx_stats *stats);

/** Backend that consumes and counts samples, for tests. */
extern const struct esp_sdr_tx_backend esp_sdr_tx_stub_backend;
#endif /* CONFIG_ESP_SDR_TX */

/** Statistics of the DAC backend since its last start. */
struct esp_sdr_tx_dac_stats {
	/** Input samples played, and dropped (not there in time, or lost in a restart). */
	uint64_t played, dropped;
	/** Blocks short of input (zero filled), and times the input lapped the ring. */
	uint32_t underruns, overruns;
	/** Bank switches, and blocks not filled in time (the DAC restarted). */
	uint32_t switches, errors;
	/** Interpolation factor (DAC rate / input rate) and run time in us. */
	uint32_t interp, elapsed_us;
	/** Per filler: time inside the fill, blocks filled, and the CPU it ran on last. */
	uint32_t fill_us[2], fill_blocks[2], fill_cpu[2];
	/** The vector interpolator passed its self-test and is in use. */
	bool simd;
	/** Time between consecutive bank switches: sum and maximum in us. */
	uint32_t switch_us_sum, switch_us_max;
	/** Least time left between a finished fill and its bank switch, in us. */
	int32_t slack_us_min;
	/** Restarts after a late fill. */
	uint32_t restarts;
	/** Longest fill, latest wake-up for a switch, longest switch-to-fill gap, in us. */
	uint32_t fill_us_max, wake_late_us_max, idle_us_max;
	/** Time left before the due time after the session's first loop fill, in us. */
	int32_t start_margin_us;
};

#if defined(CONFIG_ESP_SDR_TX_DAC)
/**
 * DAC backend: interpolates the input linearly to 40 MS/s and plays it. The
 * input rate must divide 40 MS/s by at least 2 and the frequency must be a
 * whole MHz (it retunes the shared LO, so the receiver moves too). Output
 * is gapless while the fill keeps up; input that is not there in time plays
 * as zeros.
 */
extern const struct esp_sdr_tx_backend esp_sdr_tx_dac_backend;

/** Copy the DAC backend statistics into @p stats. */
void esp_sdr_tx_dac_get_stats(struct esp_sdr_tx_dac_stats *stats);

/**
 * Waveform source for esp_sdr_tx_dac_play_gen(): write @p n DAC words
 * (esp_sdr_tx_word()) for output samples [@p index, @p index + n) at the
 * 40 MS/s DAC rate. Runs in the filler, ahead of the air by up to two
 * banks, with interrupts locked when the vector interpolator is built
 * (CONFIG_ESP_SDR_TX_DAC_SIMD) and unlocked otherwise: table lookups and
 * copies only, from IRAM.
 */
typedef void (*esp_sdr_tx_dac_gen_t)(void *user, uint32_t *dst, uint64_t index, uint32_t n);

/**
 * @brief Play @p samples generated DAC samples gapless at the current LO.
 *
 * Returns once the last sample has played, about 1.5 ms after the call plus
 * the waveform's duration. Not while the backend streams.
 *
 * @retval 0 on success.
 * @retval -EINVAL without a generator or samples.
 * @retval -EBUSY while the backend streams or plays.
 * @retval -EIO if the session could not start or was stopped, or a block was
 *	   filled late: the waveform is cut off at that point (a late block
 *	   ends the session with a block of silence, it is not resumed).
 */
int esp_sdr_tx_dac_play_gen(esp_sdr_tx_dac_gen_t gen, void *user, uint64_t samples);

/**
 * @return Input samples written but not yet interpolated for the DAC: what a
 * writer adds to the latency. Keep it below the ring size
 * (CONFIG_ESP_SDR_TX_DAC_RING_SAMPLES), or new input overwrites old.
 */
uint32_t esp_sdr_tx_dac_queued(void);
#endif

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ESP_SDR_ESP_SDR_TX_H_ */
