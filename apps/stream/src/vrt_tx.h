/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef STREAM_VRT_TX_H_
#define STREAM_VRT_TX_H_

#include <stdint.h>

struct vrt_tx_stats {
	uint32_t data;      /* signal data packets */
	uint32_t commands;  /* command packets */
	uint32_t acks;      /* acknowledges sent */
	uint32_t gaps;      /* packet counter discontinuities */
	uint32_t underruns; /* host fell behind the sample clock (paced backends) */
	uint32_t holes;     /* samples zero-filled where packets were missing (timestamps) */
	uint32_t late;      /* samples dropped as already passed (timestamps) */
	uint32_t bad;       /* undecodable datagrams */
	uint32_t errors;    /* backend or socket failures */
};

/* Open the UDP port and start the receive thread. */
int vrt_tx_init(void);
/* Packet and sample counters. */
void vrt_tx_get_stats(struct vrt_tx_stats *stats);
/* The transmit frequency and sample rate in use. */
void vrt_tx_get_settings(uint64_t *freq_hz, uint32_t *rate_hz);

#endif /* STREAM_VRT_TX_H_ */
