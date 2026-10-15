/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file
 * @brief Diagnostics of the esp_sdr IEEE 802.15.4 radio driver (device "esp_sdr_154").
 */

#ifndef ESP_SDR_IEEE154_ESP_SDR_H_
#define ESP_SDR_IEEE154_ESP_SDR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup ieee154_esp_sdr
 * @{
 */

/** Driver counters, ieee154_esp_sdr_get_stats(). */
struct ieee154_esp_sdr_stats {
	/** Receive captures processed, and failed captures. */
	uint32_t captures, capture_errors;
	/** Frames with a good FCS, and those the stack could not take. */
	uint32_t frames, frames_dropped;
	/** Frames transmitted, and transmissions that failed. */
	uint32_t tx_frames, tx_errors;
	/** ACKs received for frames that asked for one, and frames left without. */
	uint32_t acks, no_acks;
	/** Error of the last failed transmission. */
	int32_t tx_last_err;
	/** Last capture's mix/decimate time; PHY time in us (ring: total). */
	uint32_t filter_us, phy_us;
	/**
	 * Ring receive: last run's status; units and pairs lost, summed over all runs; samples
	 * the PHY skipped and read.
	 */
	uint32_t ring_status, ring_units, ring_lost, phy_skipped, phy_read;
	/** Last ring run: filter cycles per pair x 100, longest slice in cycles, units abandoned. */
	uint32_t ring_cycles_x100, ring_slice_max, ring_abandoned;
	/** Last ring run: interrupt windows opened, and the CPU interrupt lines pending in them. */
	uint32_t ring_irq_windows, ring_irq_lines;
	/** Worst bank switch lateness of all runs, in ring pairs. */
	uint32_t ring_late_max;
	/**
	 * Ring receive: frames decided, and the time from a frame's last
	 * sample to the PHY's decision (min, max, sum), in us.
	 */
	uint32_t verdict_n, verdict_min_us, verdict_max_us, verdict_sum_us;
	/**
	 * ACKs sent (CONFIG_ESP_SDR_IEEE802154_AUTO_ACK): frames that asked
	 * this node for one, armed bursts that failed, fired, refused by the
	 * ring (too late), cancelled (FCS failed); frames without a
	 * destination address (not acknowledged).
	 */
	uint32_t ack_due, ack_arm_failed, ack_fired, ack_fire_failed, ack_cancelled;
	uint32_t ack_unsupported;
	/**
	 * Armed frames: the verdict after the frame's end (max, sum, how many
	 * after the ACK delay), and how far the PHY was behind at the arm (max), in us.
	 */
	uint32_t ack_verdict_max_us, ack_verdict_sum_us, ack_verdict_late, ack_lag_max_us;
};

struct net_pkt;

/**
 * Frame sink without the 802.15.4 L2 (raw use of the radio API): receives
 * each frame without its FCS, LQI set, and owns the packet. It runs on the
 * PHY thread with the PHY lock held: it must not call back into the driver
 * (transmit, channel, diagnostics).
 */
typedef void (*ieee154_esp_sdr_rx_cb_t)(struct net_pkt *pkt);

/** Set the frame sink used when CONFIG_NET_L2_IEEE802154 is off (ignored with it on). */
void ieee154_esp_sdr_set_rx_cb(ieee154_esp_sdr_rx_cb_t cb);

/**
 * Conjugate received samples and/or the transmitted waveform (spectral
 * inversion), to check another radio's I/Q convention. Both off at boot.
 */
void ieee154_esp_sdr_set_invert(bool rx_inv, bool tx_inv);

/**
 * Diagnostic: put the LO 4 MHz above the channel instead of below. Only the
 * subsampling ring receive handles it (transmit and the ACK capture assume
 * the LO below).
 *
 * @retval 0 on success, also before the radio is started (applied at start).
 * @retval <0 the retune failed (esp_sdr_set_freq()); the LO side is unchanged.
 */
int ieee154_esp_sdr_set_lo_above(bool above);

/** Result of ieee154_esp_sdr_measure(). */
struct ieee154_esp_sdr_measurement {
	/** Carrier offset in Hz (meaningless without a signal), wraps at +-500 kHz. */
	int32_t cfo_hz;
	/** Coarse carrier offset in Hz, unambiguous within +-1.5 MHz. */
	int32_t coarse_cfo_hz;
	/** Mean power relative to the receive full scale. */
	int32_t power_dbfs;
	/** Samples above half the mean power that went into the estimate. */
	uint32_t samples_used;
	/**
	 * Histogram of the one-chip phase steps in 45 degree bins from -180
	 * degrees: O-QPSK fills the bins at -90 and +90.
	 */
	uint32_t steps[8];
};

/**
 * Diagnostic: one 1 ms capture at the current channel, its carrier offset
 * against an O-QPSK signal filling it (within +-500 kHz) and its power.
 *
 * @retval 0 on success.
 * @retval -ENETDOWN if the radio was never started.
 * @retval <0 an error of the capture (esp_sdr_rx_capture()).
 */
int ieee154_esp_sdr_measure(struct ieee154_esp_sdr_measurement *m);

/**
 * esp_sdr_rx_set_gain() for the radio: stops the receive ring around the
 * call, which otherwise waits for the radio lock a ring run holds. Calls
 * into esp_sdr that take the radio go through the driver while it runs.
 */
int ieee154_esp_sdr_set_rx_gain(int index);

/** Receives the samples of ieee154_esp_sdr_dump(): @p n interleaved I/Q pairs. */
typedef void (*ieee154_esp_sdr_dump_cb_t)(const int16_t *iq, size_t n, void *user);

/**
 * Diagnostic: one 1 ms capture at the current channel as the PHY would see
 * it (mixed down and filtered to 4 MS/s, interleaved I/Q, +-8192 full
 * scale), handed to @p out while the driver still holds the radio.
 *
 * @retval 0 on success.
 * @retval -ENETDOWN if the radio was never started.
 * @retval <0 an error of the capture (esp_sdr_rx_capture()).
 */
int ieee154_esp_sdr_dump(ieee154_esp_sdr_dump_cb_t out, void *user);

/** Driver state changes kept across resets (ieee154_esp_sdr_get_trail()). */
enum ieee154_esp_sdr_event {
	IEEE154_ESP_SDR_EV_TAKE = 1, /* radio taken, argument: waiters before */
	IEEE154_ESP_SDR_EV_GIVE,     /* radio released */
	IEEE154_ESP_SDR_EV_RING,     /* ring run starts */
	IEEE154_ESP_SDR_EV_RING_END, /* ring run ended, argument: status */
	IEEE154_ESP_SDR_EV_TX,       /* transmission starts, argument: PSDU length */
	IEEE154_ESP_SDR_EV_TX_END,   /* transmission ended, argument: -error */
	IEEE154_ESP_SDR_EV_CAPTURE,  /* bank capture (ACK window, measure, dump, burst receive) */
	IEEE154_ESP_SDR_EV_BOOT,     /* driver init */
};

/**
 * Copy the last driver events, oldest first: event in bits 31:28, CPU in
 * bit 27, argument in bits 26:20, uptime in ms (modulo 2^20) in bits 19:0.
 * Kept in RTC memory across resets. Returns the count.
 */
size_t ieee154_esp_sdr_get_trail(uint32_t *out, size_t max);

/**
 * Diagnostic: CPU 0 PCs (and last load/store addresses) sampled by the
 * receive ring when the PHY thread made no progress for about a second,
 * kept across resets. Returns the count (0 if no stall was seen, and always 0
 * without CONFIG_ESP_SDR_IEEE802154_RING).
 */
size_t ieee154_esp_sdr_get_stall(uint32_t *pc, uint32_t *ls, size_t max);

/** Copy the driver counters into @p stats. */
void ieee154_esp_sdr_get_stats(struct ieee154_esp_sdr_stats *stats);

struct ieee154_esp_sdr_bench {
	/** ACK waveform: chips, DAC words (all, and from slot 416 on) in CPU cycles; its length. */
	uint32_t ack_build_cyc, ack_gen_cyc, ack_tail_cyc, ack_words;
	/** Front end switch RX to TX and back, settle times included, in us. */
	uint32_t to_tx_us, to_rx_us;
};

/** @brief Time the ACK waveform build and the front end switches (radio taken meanwhile). */
int ieee154_esp_sdr_bench(struct ieee154_esp_sdr_bench *b);

/**
 * @brief ACK timing (CONFIG_ESP_SDR_IEEE802154_AUTO_ACK): first sample @p delay_us
 * after the frame's end (192 us by the standard), none after @p limit_us.
 * @return 0, -EINVAL, -ENOTSUP.
 */
int ieee154_esp_sdr_set_ack_timing(uint32_t delay_us, uint32_t limit_us);

/** @brief Diagnostic: send one ACK with sequence number @p seq now (receive running). */
int ieee154_esp_sdr_ack_test(uint8_t seq);

/** @brief Zero the driver's counters. */
void ieee154_esp_sdr_clear_stats(void);

/**
 * @brief Set the ring's unit length (ring receive), for latency measurements.
 *
 * @param pairs 0 for the driver's default (ESP_SDR_RING_UNIT_MIN), else see
 *              esp_sdr_ring_cfg::unit_pairs.
 * @return 0 or -EINVAL.
 */
int ieee154_esp_sdr_set_ring_unit(uint32_t pairs);

/**
 * With CONFIG_ESP_SDR_IEEE802154_HIGH_RAM_APP, defined by the application:
 * @p size bytes of 16-byte aligned internal RAM for the driver this boot, or
 * NULL to leave it off. Called once, at device init.
 */
void *ieee154_esp_sdr_high_ram(size_t size);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ESP_SDR_IEEE154_ESP_SDR_H_ */
