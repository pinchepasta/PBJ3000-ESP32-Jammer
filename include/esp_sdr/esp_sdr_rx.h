/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file
 * @brief ESP-SDR receive: gain, analog low-pass, burst and decimated capture.
 *
 * Captures bursts of raw baseband I/Q samples from the Wi-Fi receiver through
 * an undocumented debug path.
 */

#ifndef ESP_SDR_ESP_SDR_RX_H_
#define ESP_SDR_ESP_SDR_RX_H_

#include <stddef.h>
#include <stdint.h>

#include <esp_sdr/esp_sdr.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup esp_sdr_rx
 * @{
 */

/** Gain index that selects the hardware AGC. */
#define ESP_SDR_RX_GAIN_AUTO (-1)

/** Low-pass filter code that restores the PHY-calibrated setting. */
#define ESP_SDR_RX_LPF_AUTO (-1)
/** Largest raw low-pass filter capacitor code. */
#define ESP_SDR_RX_LPF_MAX  63

/** Decimation factor limits for esp_sdr_rx_capture_decimated(). */
#define ESP_SDR_RX_DECIM_MIN 2U
#define ESP_SDR_RX_DECIM_MAX 160U

/** Fold factor limits for esp_sdr_rx_capture_folded(). */
#define ESP_SDR_RX_FOLD_MIN 2U
#define ESP_SDR_RX_FOLD_MAX 160U

/** Result of one burst capture. */
struct esp_sdr_rx_burst {
	/** Raw words, Q in bits 9:0 and I in bits 19:10 (signed). Valid until the next capture. */
	uint32_t *words;
	/** Number of complex samples. */
	size_t count;
	/** Time from arming to completion in microseconds. */
	uint32_t elapsed_us;
};

/**
 * @brief Select hardware AGC or a fixed receive gain.
 *
 * Indices are PHY gain table entries, not dB; higher is more gain.
 *
 * @param index ESP_SDR_RX_GAIN_AUTO, or 0 to esp_sdr_rx_gain_max().
 * @retval 0 on success.
 * @retval -EINVAL if out of range.
 * @retval -ENOTSUP for a fixed gain without CONFIG_ESP_SDR_RFTEST.
 * @retval -EAGAIN if esp_sdr_init() has not run.
 */
int esp_sdr_rx_set_gain(int index);

/** @return Selected gain index, or ESP_SDR_RX_GAIN_AUTO. */
int esp_sdr_rx_get_gain(void);

/** @return Largest calibrated gain index, 0 if the PHY reports none. */
int esp_sdr_rx_gain_max(void);

/**
 * @brief Select the analog low-pass filter by approximate bandwidth.
 *
 * @param mhz Two-sided bandwidth in MHz, 0 selects the widest setting.
 * @retval 0 on success.
 * @retval -EINVAL if outside the characterized range.
 */
int esp_sdr_rx_set_bandwidth(uint32_t mhz);

/**
 * @brief Supported bandwidth range in MHz.
 *
 * @param min Set to the narrowest setting.
 * @param max Set to the widest setting.
 */
void esp_sdr_rx_bandwidth_range(uint32_t *min, uint32_t *max);

/**
 * @brief Select a raw low-pass filter capacitor code.
 *
 * @param code 0 (widest) to ESP_SDR_RX_LPF_MAX, or ESP_SDR_RX_LPF_AUTO.
 * @retval 0 on success.
 * @retval -EINVAL if out of range.
 */
int esp_sdr_rx_set_lpf(int code);

/** @return Selected low-pass filter code, or ESP_SDR_RX_LPF_AUTO. */
int esp_sdr_rx_get_lpf(void);

/**
 * @brief Capture one burst of complex samples.
 *
 * The samples live in a reserved SRAM bank and stay valid until the next
 * capture or transmit session by any thread: with other radio users, copy
 * them with esp_sdr_rx_capture_iq() instead. The caller may rewrite them in
 * place, for example to pack them.
 *
 * @param rate Native sample rate.
 * @param count Complex samples, ESP_SDR_SAMPLES_MIN to ESP_SDR_SAMPLES_MAX.
 * @param burst Filled with the result.
 * @retval 0 on success.
 * @retval -EINVAL for an unsupported rate or count.
 * @retval -EAGAIN if esp_sdr_init() has not run.
 * @retval -ETIMEDOUT if the capture engine did not finish.
 */
int esp_sdr_rx_capture(enum esp_sdr_rate rate, size_t count, struct esp_sdr_rx_burst *burst);

/**
 * @brief Capture one burst and convert it to I/Q (raw units, -512 to 511) under the radio lock.
 *
 * @param rate Native sample rate.
 * @param count Complex samples, ESP_SDR_SAMPLES_MIN to ESP_SDR_SAMPLES_MAX.
 * @param out Output samples, @p count of them.
 * @param first_ns Set to the arming time (k_cycle_get_64() clock, ns).
 * @return Like esp_sdr_rx_capture().
 */
int esp_sdr_rx_capture_iq(enum esp_sdr_rate rate, size_t count, struct esp_sdr_iq16 *out,
			  uint64_t *first_ns);

/**
 * @brief Capture one burst into a chosen bank, like esp_sdr_rx_capture().
 *
 * With two banks (CONFIG_ESP_SDR_BANK1), one burst can be processed while
 * the next is captured into the other bank. Bank 0 is the one
 * esp_sdr_rx_capture() uses.
 *
 * @param rate Native sample rate.
 * @param count Complex samples, ESP_SDR_SAMPLES_MIN to ESP_SDR_SAMPLES_MAX.
 * @param bank 0 to ESP_SDR_BANKS - 1.
 * @param burst Filled with the result.
 * @return Like esp_sdr_rx_capture(); -EINVAL also for a bank that does not exist.
 */
int esp_sdr_rx_capture_bank(enum esp_sdr_rate rate, size_t count, int bank,
			    struct esp_sdr_rx_burst *burst);

/**
 * @brief Capture a burst and decimate it with a third order CIC filter.
 *
 * Captures @p count samples like esp_sdr_rx_capture(), then (still holding
 * the radio lock) decimates by @p m into @p out, I and Q in the usual order.
 * The first 3 outputs (filter settling) are dropped, so up to count / m - 3
 * samples come out. The CIC needs no multiplies per input sample but droops
 * towards the band edge: -2.7 dB at a quarter of the output rate, -9.3 dB at
 * 0.45 of it. Needs CONFIG_ESP_SDR_RX_DECIM.
 *
 * @param rate Native sample rate.
 * @param count Complex samples, ESP_SDR_SAMPLES_MIN to ESP_SDR_SAMPLES_MAX.
 * @param m Decimation factor, ESP_SDR_RX_DECIM_MIN to ESP_SDR_RX_DECIM_MAX.
 * @param out Output samples, full scale +-32767.
 * @param max_out Capacity of @p out.
 * @param n_out Set to the number of samples written.
 * @param first_ns Set to the time of the first output sample (k_cycle_get_64() clock, ns).
 * @retval 0 on success.
 * @retval -EINVAL for an unsupported rate, count or factor.
 * @retval -EAGAIN if esp_sdr_init() has not run.
 * @retval -ETIMEDOUT if the capture engine did not finish.
 */
int esp_sdr_rx_capture_decimated(enum esp_sdr_rate rate, size_t count, unsigned int m,
				 struct esp_sdr_iq16 *out, size_t max_out, size_t *n_out,
				 uint64_t *first_ns);

/**
 * @brief Capture a burst and fold its spectrum by @p n: a power estimation mode.
 *
 * Captures @p count samples like esp_sdr_rx_capture(), then (still holding
 * the radio lock) sums @p n time-shifted blocks of count / n samples each,
 * averaged back to the raw 10-bit scale, into @p out. This is decimation in
 * frequency rather than in time: a tone originally at offset f reappears at
 * f mod (rate / n), summed with every other tone an integer multiple of
 * rate / n away, instead of being low-pass filtered out. The whole original
 * band's power is represented at 1/n the data rate, so it suits a wideband
 * power/occupancy survey; it does not suit recovering an individual signal,
 * since two sources that land in the same fold group add instead of one of
 * them surviving cleanly. Needs CONFIG_ESP_SDR_RX_DECIM.
 *
 * @param rate Native sample rate.
 * @param count Complex samples, ESP_SDR_SAMPLES_MIN to ESP_SDR_SAMPLES_MAX.
 * @param n Fold factor, ESP_SDR_RX_FOLD_MIN to ESP_SDR_RX_FOLD_MAX.
 * @param out Output samples, full scale +-511 (raw 10-bit, averaged not summed).
 * @param max_out Capacity of @p out.
 * @param n_out Set to the number of samples written (count / n).
 * @param first_ns Set to the time of the first input sample in the burst.
 * @retval 0 on success.
 * @retval -EINVAL for an unsupported rate, count or factor.
 * @retval -EAGAIN if esp_sdr_init() has not run.
 * @retval -ETIMEDOUT if the capture engine did not finish.
 */
int esp_sdr_rx_capture_folded(enum esp_sdr_rate rate, size_t count, unsigned int n,
			      struct esp_sdr_iq16 *out, size_t max_out, size_t *n_out,
			      uint64_t *first_ns);

/** @return Capture sample rate in Hz, or 0 for an unknown rate. */
uint32_t esp_sdr_rx_rate_hz(enum esp_sdr_rate rate);

/*
 * Receive words carry Q in the low field: with I low, a tone below the LO
 * showed up above it (checked by detuning the receiver against a known
 * carrier). Transmit words are the other way round, see esp_sdr_tx_word().
 */

/** @return In-phase component of a raw receive word, -512 to 511. */
static inline int16_t esp_sdr_rx_i(uint32_t word)
{
	return (int16_t)((int32_t)(word << 12) >> 22);
}

/** @return Quadrature component of a raw receive word, -512 to 511. */
static inline int16_t esp_sdr_rx_q(uint32_t word)
{
	return (int16_t)((int32_t)(word << 22) >> 22);
}

/**
 * @brief Pack raw words to signed 8-bit I then Q (upper 8 of 10 bits).
 *
 * @p out may alias @p words.
 *
 * @return Bytes written, 2 * count.
 */
size_t esp_sdr_rx_pack_iq8(const uint32_t *words, size_t count, uint8_t *out);

/**
 * @brief Pack raw words to the upstream packed 10-bit format.
 *
 * Little-endian 20-bit I/Q pairs, two samples per five bytes. @p out may alias @p words.
 *
 * @return Bytes written, ceil(20 * count / 8).
 */
size_t esp_sdr_rx_pack_iq10(const uint32_t *words, size_t count, uint8_t *out);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ESP_SDR_ESP_SDR_RX_H_ */
