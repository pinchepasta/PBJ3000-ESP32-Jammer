/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * CSMA/CA MAC over the QAM PHY: carrier sense on short captures, DIFS and
 * binary exponential backoff counted in sense slots, stop and wait ARQ with
 * immediate ACKs, duplicate detection by sequence number. One thread owns
 * the radio; frames play from a DAC bank in a loop for a fixed airtime so
 * that a listening receiver catches one whole copy.
 */

#ifndef LINK_MAC_H_
#define LINK_MAC_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include <esp_sdr/esp_sdr.h>

#include "qam.h"

#define MAC_ADDR_BROADCAST 0xffU

enum mac_type {
	MAC_DATA,
	MAC_ACK,
};

/* PHY of the frames (with OFDM the ACKs are header-only OFDM frames). */
enum mac_phy {
	MAC_PHY_QAM,
	MAC_PHY_OFDM,
};

struct mac_cfg {
	uint32_t freq_mhz;
	/* enum mac_phy; both ends alike, and for OFDM the same "ofdm set" settings. */
	int phy;
	/* Transmit power step, receive gain index (ESP_SDR_RX_GAIN_AUTO for AGC). */
	int tx_gain;
	int rx_gain;
	/* TX low-pass codes (esp_sdr_tx_set_lpf(), -1 the PHY's). */
	int tx_lpf_a, tx_lpf_b;
	/* PHY channel bandwidth flag (esp_sdr_set_channel_bw()). */
	int cbw;
	/* Receive filter: two-sided MHz, 0 widest, -1 the PHY's own setting. */
	int bw_mhz;
	/* Receiver: average up to this many loop copies of short frames. */
	int avg;
	/* RMS DAC amplitude of a frame. */
	int amp;
	/* Time a data frame and an ACK stay on air, in us. */
	uint32_t data_air_us;
	uint32_t ack_air_us;
	/* How long a sender listens for the ACK after its frame, in us. */
	uint32_t ack_timeout_us;
	/* Front end settling at each TX/RX switch, and whether it retunes the LO. */
	uint32_t turn_us;
	bool turn_retune;
	/* Carrier sense threshold above the measured noise floor, in dB. */
	int cca_db;
	/* Quiet sense slots before the backoff counts, contention window limits, retries. */
	uint8_t difs;
	uint16_t cw_min;
	uint16_t cw_max;
	uint8_t retries;
	uint8_t addr;
};

/* Counters since boot; the reporter diffs snapshots. */
struct mac_stats {
	/* Sender: new frames, transmissions (with retries), ACKed, given up. */
	uint32_t tx_frames;
	uint32_t tx_attempts;
	uint32_t tx_acked;
	uint32_t tx_dropped;
	uint64_t tx_bytes_acked;
	/* Sense slots found busy, backoff slots counted down. */
	uint32_t cca_busy;
	uint32_t backoff_slots;
	/* Receiver: energy triggers and what decoding them gave. */
	uint32_t rx_triggers;
	uint32_t rx_none;
	uint32_t rx_hdr_err;
	uint32_t rx_data_err;
	uint32_t rx_ok;
	uint32_t rx_dup;
	uint32_t rx_lost;
	uint32_t rx_other;
	uint64_t rx_bytes;
	/* Uptime of the last new data frame, in ms. */
	int64_t rx_last_ms;
	uint32_t acks_sent;
	uint32_t acks_rx;
	/* Sums for averages over decoded frames: MER (centi dB), corrected bytes. */
	int64_t mer_cdb_sum;
	int64_t cfo_hz_sum;
	uint32_t mer_n;
	uint32_t corrected;
	/* Timing in us: decode, front end to TX and back, sense slot. */
	uint64_t decode_us_sum;
	uint32_t decode_n;
	uint32_t decode_us_max;
	uint64_t tx_on_us_sum;
	uint64_t tx_off_us_sum;
	uint32_t tx_switches;
	uint64_t sense_us_sum;
	uint32_t sense_n;
	/* Pair decoding: first frame (this CPU), second (other CPU), both done. */
	uint64_t pair_a_us_sum, pair_b_us_sum, pair_us_sum;
	uint32_t pair_n;
	/* Frame builds (background and retries). */
	uint64_t build_us_sum;
	uint32_t build_n;
	/* Decoder stages (qam_rx_info.prof), summed in us. */
	uint64_t prof_us[6];
	/* Capture engine time of the last sense slot and full window, in us. */
	uint32_t sense_engine_us;
	uint32_t window_engine_us;
};

int link_mac_init(void);

/* Current configuration; link_mac_set_cfg() applies a changed copy while idle. */
void link_mac_get_cfg(struct mac_cfg *cfg);
int link_mac_set_cfg(const struct mac_cfg *cfg);

/* Noise floor in raw units squared (re-measured with link_mac_calibrate()). */
float link_mac_noise_floor(void);
int link_mac_calibrate(void);

/*
 * Send test frames (@p ncw units of @p fec, 0: full frames) to @p dst, @p window
 * of them per transmission (1, or 2 as a pair the receiver decodes on both
 * CPUs), until @p duration_ms passes (0: until stopped),
 * at most @p rate_kbps of user data (0: as fast as the MAC goes).
 */
int link_mac_client_start(uint8_t dst, enum qam_mod mod, enum qam_fec fec, unsigned int ncw,
			  unsigned int window, uint32_t duration_ms, uint32_t rate_kbps);
void link_mac_client_stop(void);
bool link_mac_client_active(void);
/* When the client last finished (its last frame done), in ms of uptime. */
int64_t link_mac_client_end_ms(void);

void link_mac_get_stats(struct mac_stats *st);
void link_mac_clear_stats(void);

/*
 * Build a frame at @p mod and decode it from memory (MAC paused), no radio:
 * the decoder's own time.
 */
int link_mac_bench_decode(enum qam_mod mod, struct qam_rx_info *info, uint32_t *build_us,
			  uint32_t *decode_us);

/* Park the MAC thread (radio idle) for measurements. */
void link_mac_pause(bool pause);

/*
 * Run @p fn(@p arg) on the paused MAC thread (link_mac_pause()) and wait for
 * it: the decoders need about 3.5 KB of stack, more than the shell has.
 *
 * @return 0; -ENODEV if the MAC thread does not run (the radio failed to
 * start); -ETIMEDOUT if it did not take the job within 10 s.
 */
int link_mac_run_paused(void (*fn)(void *arg), void *arg);

/*
 * Hand out the next capture that crossed the carrier sense threshold (with
 * @p decoded: that held a data frame that decoded) within @p timeout: the MAC thread then holds the bank until link_mac_dump_release().
 */
int link_mac_dump_take(const uint32_t **words, size_t *n, bool decoded, k_timeout_t timeout);
void link_mac_dump_release(void);

/* User bytes per data frame at @p mod with the current PHY settings, 0 if none fit. */
size_t link_mac_frame_bytes(enum qam_mod mod);

struct ofdm_ctx;

/* The OFDM context the MAC and the "ofdm" shell test share, set up (ofdm_test.c). */
struct ofdm_ctx *link_ofdm(void);

/* Its transmit and receive rates. */
enum esp_sdr_rate link_ofdm_tx_rate(void);
enum esp_sdr_rate link_ofdm_rx_rate(void);

#endif /* LINK_MAC_H_ */
