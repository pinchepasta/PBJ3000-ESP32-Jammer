/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file
 * @brief ESP-SDR CPU-paced streaming (ESP32-C6, ESP32-C5): continuous
 * receive and transmit, one sample at a time from the caller's loop.
 *
 * The C6 has no dump path that runs gapless at a low rate and no DAC buffer
 * the CPU can refill while it plays, so the CPU paces both directions:
 *
 * - Receive differences the running sums of the modem's IQ estimator, which
 *   adds up the 80 MHz baseband samples in hardware: each output is the sum
 *   of the 80 MHz / rate samples since the previous one (a boxcar
 *   decimator). Gapless up to about 4.7 MS/s with nothing else in the loop.
 * - Transmit writes the TX DC value register, which drives the DAC directly
 *   once the vendor tone path has turned the transmit chain on, at the rate
 *   times an interpolation factor, through an 8-tap-per-phase FIR.
 *
 * Between start and stop the caller polls in a tight loop with interrupts
 * locked and serves its own I/O between polls. A poll that comes late moves
 * that sample's instant; the stats count it. One direction at a time.
 */

#ifndef ESP_SDR_ESP_SDR_PACED_H_
#define ESP_SDR_ESP_SDR_PACED_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Input rate of the estimator: each receive output sums 80 MHz / rate of its samples. */
#define ESP_SDR_PACED_INPUT_HZ 80000000U

struct esp_sdr_paced_stats {
	/** Samples produced (receive) or input samples consumed (transmit). */
	uint64_t samples;
	/** Polls that came a whole period or more late. */
	uint32_t late;
	/** Worst lateness, in CPU cycles. */
	uint32_t late_max;
	/** Receive: estimator restarts. Transmit: input samples that were missing (played as 0). */
	uint32_t events;
};

/**
 * @brief Start receiving at @p rate_hz.
 *
 * Takes the radio until esp_sdr_rx_paced_stop(). The receiver keeps its
 * frequency, gain and filter.
 *
 * @param rate_hz A divisor of both the CPU clock and 80 MHz, from 10 kHz.
 * @return 0, -EINVAL for an unsupported rate, -EAGAIN before esp_sdr_init().
 */
int esp_sdr_rx_paced_start(uint32_t rate_hz);

/**
 * @brief Produce the next receive sample if it is due.
 *
 * @param i, q The sums over this sample's window of 80 MHz / rate inputs,
 *             each input within -512 to 511.
 * @return true when a sample was produced.
 */
bool esp_sdr_rx_paced_poll(int32_t *i, int32_t *q);

/**
 * @brief CPU cycles until the next receive sample is due, 0 when it is.
 *
 * I/O between polls that may take longer than this delays the sample.
 */
uint32_t esp_sdr_rx_paced_slack(void);

/** @brief Stop receiving and release the radio; no-op when stopped. */
void esp_sdr_rx_paced_stop(void);

/**
 * Transmit input: store the next sample (full scale +-32767) and return true,
 * or return false when none is available (played as 0). Called from the poll,
 * once per input sample: must not block.
 */
typedef bool (*esp_sdr_tx_paced_input_t)(void *user, int16_t *i, int16_t *q);

/**
 * @brief Turn the transmit chain on at the current frequency and start
 * playing input at @p rate_hz.
 *
 * @param rate_hz Input rate; rate times @p interp must divide the CPU clock.
 * @param interp Interpolation factor: 1 (input straight to the DAC), 2, 4 or 8.
 * @param power Power step, 0 weakest to esp_sdr_tx_paced_power_max().
 * @param input Called once per input sample from esp_sdr_tx_paced_poll().
 * @param user Passed to @p input.
 * @return 0, -EINVAL, -ERANGE outside the transmit range, -EAGAIN before
 *         esp_sdr_init().
 */
int esp_sdr_tx_paced_start(uint32_t rate_hz, unsigned int interp, unsigned int power,
			   esp_sdr_tx_paced_input_t input, void *user);

/**
 * @brief Write the next DAC value if it is due, then compute the one after.
 * @return true when a value was written.
 */
bool esp_sdr_tx_paced_poll(void);

/** @brief CPU cycles until the next DAC value is due, 0 when it is. */
uint32_t esp_sdr_tx_paced_slack(void);

/** @brief Turn the transmit chain off and release the radio; no-op when stopped. */
void esp_sdr_tx_paced_stop(void);

/** @brief Highest power step: 2 dB per step. */
unsigned int esp_sdr_tx_paced_power_max(void);

/** @brief Transmit range in MHz: what the vendor tone path tunes (C6 2.4 GHz, C5 also 5 GHz). */
void esp_sdr_tx_paced_range(uint32_t *min_mhz, uint32_t *max_mhz);

/** @brief The CPU cycle counter the paced streams run on. */
static inline uint32_t esp_sdr_paced_cycles(void)
{
	uint32_t v;

#if defined(CONFIG_SOC_SERIES_ESP32C6)
	/* The C6's machine performance counter (mpccr). */
	__asm__ volatile("csrr %0, 0x7e2" : "=r"(v));
#else
	__asm__ volatile("csrr %0, cycle" : "=r"(v));
#endif
	return v;
}

/**
 * @brief When the running stream's next sample is due, on esp_sdr_paced_cycles().
 *
 * For I/O between polls: stop it while (int32_t)(deadline - now) is small.
 */
uint32_t esp_sdr_paced_deadline(void);

/**
 * @brief Restart the running stream's clock: the next sample is due one
 * period from now. Call once interrupts are locked, after the start.
 */
void esp_sdr_paced_sync(void);

/** @brief Statistics of the running or last stream. */
void esp_sdr_paced_get_stats(struct esp_sdr_paced_stats *stats);

#ifdef __cplusplus
}
#endif

#endif /* ESP_SDR_ESP_SDR_PACED_H_ */
