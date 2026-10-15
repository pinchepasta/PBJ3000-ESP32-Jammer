/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef SDR_H_
#define SDR_H_

#include <stdint.h>

#include <stdbool.h>

#include <esp_sdr/esp_sdr_ring.h>

#include "esdr_proto.h"

/* Settings as requested by the host. */
struct sdr_settings {
	uint16_t mode, bits, options;
	uint16_t rx_gain, tx_gain;
	/* dB, or ESDR_DGAIN_DEFAULT. */
	uint16_t dgain;
	uint64_t freq_hz;
	uint32_t rate_hz, bandwidth_hz;
	int32_t corr_ppb;
};

/* Bring the radio up and start the receive and USB threads. */
int sdr_init(void);

void sdr_get_settings(struct sdr_settings *s);
/* Digital gain in dB for these settings. */
unsigned int sdr_dgain_db(const struct sdr_settings *s);
void sdr_get_stats(struct esdr_stats *s);
/* Filter timing with a null sink or the stream's, in mode off. */
int sdr_bench(unsigned int decim, bool real_sink, bool rot, struct esp_sdr_ring_bench_result *r);

#endif /* SDR_H_ */
