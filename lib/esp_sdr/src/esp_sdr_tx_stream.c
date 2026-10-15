/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Transmit front end: serializes callers and keeps statistics. The stub
 * backend consumes samples and discards them.
 */

#include <errno.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <esp_sdr/esp_sdr_tx.h>

LOG_MODULE_DECLARE(esp_sdr, CONFIG_ESP_SDR_LOG_LEVEL);

static int stub_start(uint64_t freq_hz, uint32_t rate_hz)
{
	LOG_INF("tx stub: %llu Hz, %u S/s", (unsigned long long)freq_hz, rate_hz);
	return 0;
}

static int stub_write(const struct esp_sdr_iq16 *samples, size_t count)
{
	ARG_UNUSED(samples);
	return (int)count;
}

static int stub_stop(void)
{
	return 0;
}

const struct esp_sdr_tx_backend esp_sdr_tx_stub_backend = {
	.name = "stub",
	.start = stub_start,
	.write = stub_write,
	.stop = stub_stop,
};

static K_MUTEX_DEFINE(tx_lock);
#if defined(CONFIG_ESP_SDR_TX_DAC)
#define DEFAULT_BACKEND (&esp_sdr_tx_dac_backend)
#else
#define DEFAULT_BACKEND (&esp_sdr_tx_stub_backend)
#endif

static const struct esp_sdr_tx_backend *backend = DEFAULT_BACKEND;
static bool active;
static struct esp_sdr_tx_stats stats;

int esp_sdr_tx_set_backend(const struct esp_sdr_tx_backend *b)
{
	int ret = 0;

	k_mutex_lock(&tx_lock, K_FOREVER);
	if (active) {
		ret = -EBUSY;
	} else {
		backend = b != NULL ? b : DEFAULT_BACKEND;
	}
	k_mutex_unlock(&tx_lock);
	return ret;
}

const struct esp_sdr_tx_backend *esp_sdr_tx_get_backend(void)
{
	return backend;
}

int esp_sdr_tx_start(uint64_t freq_hz, uint32_t rate_hz)
{
	int ret;

	if (rate_hz == 0U) {
		return -EINVAL;
	}
	k_mutex_lock(&tx_lock, K_FOREVER);
	if (active) {
		ret = -EALREADY;
	} else {
		ret = backend->start(freq_hz, rate_hz);
		if (ret == 0) {
			active = true;
			stats = (struct esp_sdr_tx_stats){0};
		}
	}
	k_mutex_unlock(&tx_lock);
	return ret;
}

int esp_sdr_tx_write(const struct esp_sdr_iq16 *samples, size_t count)
{
	int ret;

	k_mutex_lock(&tx_lock, K_FOREVER);
	if (!active) {
		ret = -ENOTCONN;
	} else {
		ret = backend->write(samples, count);
		stats.writes++;
		if (ret < 0) {
			stats.errors++;
		} else {
			stats.samples += (uint64_t)ret;
		}
	}
	k_mutex_unlock(&tx_lock);
	return ret;
}

int esp_sdr_tx_stop(void)
{
	int ret = 0;

	k_mutex_lock(&tx_lock, K_FOREVER);
	if (active) {
		ret = backend->stop();
		active = false;
	}
	k_mutex_unlock(&tx_lock);
	return ret;
}

bool esp_sdr_tx_active(void)
{
	return active;
}

void esp_sdr_tx_get_stats(struct esp_sdr_tx_stats *out)
{
	k_mutex_lock(&tx_lock, K_FOREVER);
	*out = stats;
	k_mutex_unlock(&tx_lock);
}
