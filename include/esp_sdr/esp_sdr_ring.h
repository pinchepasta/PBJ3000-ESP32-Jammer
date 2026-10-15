/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file
 * @brief ESP-SDR continuous receive: gapless, decimated I/Q from the 16 MS/s ring.
 *
 * The dump engine runs in circular mode and rotates through three SRAM
 * banks; a two-stage decimating FIR (vector unit) filters every finished
 * bank. Needs CONFIG_ESP_SDR_RING, which reserves SRAM banks 0 to 2.
 */

#ifndef ESP_SDR_ESP_SDR_RING_H_
#define ESP_SDR_ESP_SDR_RING_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <esp_sdr/esp_sdr.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup esp_sdr_ring
 * @{
 */

/** Ring sample rate; outputs come at this rate divided by the decimation. */
#define ESP_SDR_RING_RATE_HZ 16000000U

/** Decimation limits (powers of two). */
#define ESP_SDR_RING_DECIM_MIN 64U
#define ESP_SDR_RING_DECIM_MAX 1024U

/** Output full scale: the raw 10-bit full scale times 32. */
#define ESP_SDR_RING_FULL_SCALE 16384

/**
 * Decimation by 4 (4 MS/s) for signals about 2 MHz wide, such as IEEE
 * 802.15.4: the optional -fs/4 mix and a boxcar of 4 instead of the FIR
 * chain, which cannot keep up at this rate. Aliasing is only suppressed by
 * the boxcar's nulls (about 13 dB at +-1 MHz from them); full scale
 * ESP_SDR_RING_BOX4_FULL_SCALE. Needs CONFIG_ESP_SDR_RING_BOX4 (else
 * esp_sdr_ring_run() returns -EINVAL for it).
 */
#define ESP_SDR_RING_DECIM_BOX4       4U
#define ESP_SDR_RING_BOX4_FULL_SCALE  8192

/**
 * Output callback, called from the ring loop with interrupts masked on its
 * CPU: copy the samples and return quickly, never block. @p index is the
 * output sample index of i[0] and q[0] since esp_sdr_ring_run() started;
 * a jump from the previous call's index + n is a gap (lost input).
 */
typedef void (*esp_sdr_ring_sink_t)(void *user, const int16_t *i, const int16_t *q, size_t n,
				    uint64_t index);

struct esp_sdr_ring_cfg {
	/** ESP_SDR_RING_DECIM_MIN to ESP_SDR_RING_DECIM_MAX, a power of two, or ESP_SDR_RING_DECIM_BOX4. */
	unsigned int decim;
	/**
	 * Mix by -fs/4 before filtering: the output is centred 4 MHz above
	 * the LO, keeping the LO leakage and the 1/f noise at 0 Hz out of it.
	 */
	bool shift_fs4;
	esp_sdr_ring_sink_t sink;
	void *user;
	/**
	 * With ESP_SDR_RING_DECIM_BOX4 and shift_fs4: take every 4th pair
	 * instead of summing 4 (where the mix is 1, so the band 4 MHz above
	 * the LO lands at 0 Hz). About 1 cycle per pair against about 14 for
	 * the sums, which do not keep up at 16 MS/s, for about 6 dB more
	 * noise; the LO leakage folds onto 0 Hz as a constant offset.
	 */
	bool subsample;
	/**
	 * Unit length in ring pairs (0: ESP_SDR_RING_UNIT_DEFAULT): how often
	 * the engine changes banks, and so how long after a pair arrives the
	 * sink gets it at the latest (unit length plus the filter's lag).
	 * Shorter units cost more per pair. A multiple of 64 from
	 * ESP_SDR_RING_UNIT_MIN.
	 */
	uint32_t unit_pairs;
	/**
	 * Rotate through banks 0 and 1 only and keep bank 2 for bursts
	 * (esp_sdr_ring_burst_arm()). Needs a unit length of at most about
	 * 3000 pairs (-EINVAL otherwise).
	 */
	bool spare_bank;
};

/** Default unit length: 768 us at 16 MS/s. */
#define ESP_SDR_RING_UNIT_DEFAULT 12288U
/** Shortest unit: 64 us. */
#define ESP_SDR_RING_UNIT_MIN     1024U

/** Why a run ended early. */
enum esp_sdr_ring_status {
	ESP_SDR_RING_OK = 0,
	/** A bank switch came later than the ring allows (detail: pairs written). */
	ESP_SDR_RING_LATE,
	/** A bank was held longer than the ring allows (detail: cycles). */
	ESP_SDR_RING_AGE,
	/** A unit did not start where the previous one ended. */
	ESP_SDR_RING_START,
	/** A unit's end was not inside its sentinel window. */
	ESP_SDR_RING_END,
	/** A unit of implausible length (detail: pairs). */
	ESP_SDR_RING_LENGTH,
};

struct esp_sdr_ring_stats {
	/** Ring pairs captured, output samples delivered. */
	uint64_t pairs, samples;
	uint64_t elapsed_us;
	/** Bank visits; units dropped unfiltered, and their pairs. */
	uint32_t units, abandoned, lost_pairs;
	/** Worst switch lateness in pairs past the threshold. */
	uint32_t late_max;
	/** Longest filter slice in CPU cycles, and filter cycles per pair x 100. */
	uint32_t slice_max, cycles_x100;
	/** Forced gain re-applied during the run. */
	uint32_t gain_refreshed;
	/** Times the other CPU parked this one for a flash operation. */
	uint32_t stalls;
	/** Interrupt windows opened for pending interrupts (timer, IPIs). */
	uint32_t irq_windows;
	/** The CPU interrupt lines that were pending in those windows (bit n: line n). */
	uint32_t irq_lines;
	enum esp_sdr_ring_status status;
	uint32_t detail;
};

/**
 * @brief Stream until esp_sdr_ring_stop() or a ring failure.
 *
 * Runs on the calling thread with interrupts masked on its CPU, except for
 * short windows between polls in which pending interrupts run (the kernel
 * timer may be armed on this CPU, and IPIs; the Wi-Fi MAC interrupt waits
 * until the run ends), and holds the radio for the
 * whole run: call it from a thread pinned to a CPU that nothing else needs
 * meanwhile (cooperative, or a higher-priority thread made ready by such an
 * interrupt preempts the run, which then ends late), and change frequency
 * or gain between runs.
 * A flash write on the other CPU parks this one (the loop answers the stall
 * request) and usually ends the run with ESP_SDR_RING_LATE.
 *
 * @retval 0 after esp_sdr_ring_stop().
 * @retval -EINVAL for a bad configuration.
 * @retval -EAGAIN if esp_sdr_init() has not run.
 * @retval -EIO if the ring failed (@p stats says why).
 */
int esp_sdr_ring_run(const struct esp_sdr_ring_cfg *cfg, struct esp_sdr_ring_stats *stats);

struct esp_sdr_ring_bench_result {
	uint32_t pairs, samples;
	/* Filter cycles per input pair x 100: all, and unpack, stage 1, stage 2, sink. */
	uint32_t cycles_x100, stage_x100[4];
};

/**
 * @brief Time the filter on synthetic data in bank 0, the engine idle.
 *
 * Runs @p units ring units (12288 pairs each) through the filter and @p cfg's
 * sink with interrupts masked on the calling CPU. Not while a run is active.
 */
int esp_sdr_ring_bench(const struct esp_sdr_ring_cfg *cfg, unsigned int units,
		       struct esp_sdr_ring_bench_result *res);

/**
 * @brief Ask a running esp_sdr_ring_run() to return; safe from any thread or ISR.
 *
 * A request made while no run is active is discarded when the next run
 * starts. A caller that must not lose one (a settings change racing the
 * start of a run) repeats it, or calls it again from the sink once it
 * sees the change.
 */
void esp_sdr_ring_stop(void);

/** @return Whether esp_sdr_ring_run() is running. */
bool esp_sdr_ring_active(void);

/**
 * @brief The running ring's position, from any CPU.
 *
 * @param pair The index of the pair the engine writes now, counted from the
 *             run's first pair (output sample k of a decimating run is pair
 *             k times the decimation).
 * @return false outside a run, before its first unit has been located, or
 *         if the position kept changing while read.
 */
bool esp_sdr_ring_now(uint64_t *pair);

/**
 * @defgroup esp_sdr_ring_burst Bursts inside a run
 *
 * A run with esp_sdr_ring_cfg::spare_bank can interrupt its capture for one
 * transmitted burst (an 802.15.4 ACK) from bank 2: armed with the pair at
 * which a received frame ends, the run stops the capture at the first unit
 * boundary at or after it, hands the last unit to the sink, turns the front
 * end to TX and waits; fired, it plays the bank once at 40 MS/s from the
 * given pair on; then it turns back to RX and restarts the capture (the
 * output index jumps over the gap, the pair index keeps counting real
 * time). All pairs are on the run's pair index (esp_sdr_ring_now()).
 * @{
 */

/** Writes DAC words [index, index + n) of the burst to dst (esp_sdr_tx_word()). */
typedef void (*esp_sdr_ring_gen_t)(void *user, uint32_t *dst, uint64_t index, uint32_t n);

/**
 * @brief The burst bank, the CPUs' to write while no burst is armed and
 * outside its waveform's [from, words) once armed.
 */
uint32_t *esp_sdr_ring_burst_buf(size_t *words);

/**
 * @brief Arm a burst (from another CPU than the run's).
 *
 * @param end_pair The capture runs at least until this pair.
 * @param limit_pair Without a fire by this pair the burst is dropped, and a
 *                   fire seen later is skipped.
 * @param gen Builds words [from, words) of the bank between polls before the
 *            stop, and the rest after it; words below @p from are kept.
 * @param user Passed to @p gen.
 * @param from First word @p gen builds; the caller has written those before it.
 * @param words Length of the burst in DAC words, ESP_SDR_SAMPLES_MIN to
 *              ESP_SDR_SAMPLES_MAX.
 * @return 0, -EINVAL, or -EBUSY when no run with a spare bank is active or
 *         a burst is armed already.
 */
int esp_sdr_ring_burst_arm(uint64_t end_pair, uint64_t limit_pair, esp_sdr_ring_gen_t gen,
			   void *user, uint32_t from, uint32_t words);

/**
 * @brief Play the armed burst with its first sample at @p at_pair (at once
 * if that has passed). @return false if no burst was armed (dropped).
 */
bool esp_sdr_ring_burst_fire(uint64_t at_pair);

/** @brief Drop the armed burst; the capture goes on or restarts. */
void esp_sdr_ring_burst_cancel(void);

/** @return Whether a burst is armed or running. */
bool esp_sdr_ring_burst_busy(void);

/** Burst counters of the running or last run; positions in pairs after the burst's end pair. */
struct esp_sdr_ring_burst_stats {
	uint32_t fired, cancelled, timeouts, skipped;
	/** Fired more than 2 us after the fire pair; of those, with the front end the later one. */
	uint32_t late, late_tx;
	/** Armed after the unit holding the end pair had been switched away. */
	uint32_t arm_late;
	/** Where the first sample went out: last, min, max. */
	int32_t fire_last_pairs, fire_min_pairs, fire_max_pairs;
	/** Last burst: capture stopped, front end in TX, fire seen, capture restarted. */
	int32_t stop_last_pairs, ready_last_pairs, seen_last_pairs, rx_back_last_pairs;
	/** Last burst's DAC pass in CPU cycles (0: no done flag). */
	uint32_t dac_last_cyc;
	/** Latest front end ready and fire seen; longest pass through the wait's poll (cycles). */
	int32_t ready_max_pairs, seen_max_pairs;
	uint32_t window_max_cyc;
	/** Most waveform words still to build at a stop. */
	uint32_t gen_left_max;
};

void esp_sdr_ring_burst_get_stats(struct esp_sdr_ring_burst_stats *stats);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ESP_SDR_ESP_SDR_RING_H_ */
