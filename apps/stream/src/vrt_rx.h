/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef STREAM_VRT_RX_H_
#define STREAM_VRT_RX_H_

#include <stdint.h>

#include <esp_sdr/esp_sdr.h>

/* One captured burst, handed from the capture core to the stream core. */
struct vrt_rx_burst {
	uint32_t seq;
	uint32_t count;
	uint64_t timestamp_ns;
	uint32_t freq_mhz;
	uint32_t rate_hz;
	int gain; /* ESP_SDR_RX_GAIN_AUTO or a fixed index */
	int32_t full_scale; /* 512 raw (10-bit), 32768 decimated */
	struct esp_sdr_iq16 iq[CONFIG_APP_RX_BURST_SAMPLES];
};

/* Smallest decimation: a decimated burst (16380 / m - 3 samples) must fit a burst. */
#define VRT_RX_DECIM_MIN 16U

/*
 * Receive stream rate and decimation, implemented by the capture side
 * (main.c): the capture rate (raw) or capture rate / m, VRT_RX_DECIM_MIN..160.
 */
int vrt_rx_set_rate(uint32_t hz);
uint32_t vrt_rx_get_rate(void);

/* Decimation algorithm applied at vrt_rx_set_rate()'s factor, m > 1. */
enum vrt_rx_decim {
	VRT_RX_DECIM_CIC,  /* low-pass + decimate: a clean narrowband signal, out-of-band dropped */
	VRT_RX_DECIM_FOLD, /* fold (decimate in frequency): full-band power estimate, aliased */
};

int vrt_rx_set_decim(enum vrt_rx_decim mode);
enum vrt_rx_decim vrt_rx_get_decim(void);

/* Signal data item size: 8, 12 or 16 bits (see iq_pack.h). */
int vrt_rx_set_bits(unsigned int bits);
unsigned int vrt_rx_get_bits(void);

int vrt_rx_init(void);
/* Called on the process core for every burst. */
void vrt_rx_burst(const struct vrt_rx_burst *b);
/* Called once per second after the statistics line. */
void vrt_rx_report(void);

#endif /* STREAM_VRT_RX_H_ */
