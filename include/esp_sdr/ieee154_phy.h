/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file
 * @brief Software IEEE 802.15.4 2.4 GHz O-QPSK PHY, portable C.
 *
 * All sample buffers are interleaved int16 I/Q at 4 MS/s (2 samples per
 * chip). One receiver instance: the state is global.
 */

#ifndef ESP_SDR_IEEE154_PHY_H_
#define ESP_SDR_IEEE154_PHY_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup ieee154_phy
 * @{
 */

/** Longest PSDU, FCS included. */
#define IEEE154_PHY_PSDU_MAX 127

/** Complex samples ieee154_phy_modulate() needs for a PSDU of @p len bytes. */
#define IEEE154_PHY_MOD_SAMPLES(len) ((6 + (size_t)(len)) * 128 + 4)

/**
 * Called for each CRC-valid frame: @p psdu includes the 2-byte FCS, @p lqi
 * is 0 to 255 from the mean chip-error count, @p user is the pointer given
 * to ieee154_phy_set_rx_cb().
 */
typedef void (*ieee154_phy_frame_cb_t)(const uint8_t *psdu, uint8_t len, int lqi, void *user);

/**
 * Called while a frame comes in, from the PHR on: @p len is the PSDU length
 * (FCS included), @p have the PSDU bytes received so far (0 at the PHR, at
 * most len - 1), @p end_pos the ieee154_phy_rx_position() at which the
 * frame's last symbol will be decided. Called again with each further byte
 * while it returns true. Both phase trackers may report the same frame.
 */
typedef bool (*ieee154_phy_hdr_cb_t)(const uint8_t *psdu, uint8_t have, uint8_t len,
				     uint32_t end_pos, void *user);

/** Build the chip tables. Call once before anything else. */
void ieee154_phy_init(void);

/** Set the receiver's frame callback (NULL: frames are dropped). */
void ieee154_phy_set_rx_cb(ieee154_phy_frame_cb_t cb, void *user);

/** Set the header callback (NULL: none). */
void ieee154_phy_set_hdr_cb(ieee154_phy_hdr_cb_t cb, void *user);

/** Feed the next @p n complex samples; blocks must be consecutive. */
void ieee154_phy_rx_block(const int16_t *iq, size_t n);

/** Drop any frame in progress, before a block that does not follow the last one. */
void ieee154_phy_rx_reset(void);

/**
 * Samples fed so far, counted over all blocks (wraps). In the frame
 * callback: up to and including the frame's last sample, which completed
 * its last symbol.
 */
uint32_t ieee154_phy_rx_position(void);

/** The 32 chips of symbol @p sym (0..15), chip c0 in the MSB. */
uint32_t ieee154_phy_chips(int sym);

/**
 * Modulate SHR + PHR + @p psdu (FCS already appended) into @p out.
 * Returns the complex samples written, 0 if @p max_samples is too small.
 */
size_t ieee154_phy_modulate(const uint8_t *psdu, uint8_t len, int16_t amp, int16_t *out,
			    size_t max_samples);

/** Append the little-endian CRC-16 FCS to buf[0..n-1]; returns n + 2. */
size_t ieee154_phy_append_fcs(uint8_t *buf, size_t n);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ESP_SDR_IEEE154_PHY_H_ */
