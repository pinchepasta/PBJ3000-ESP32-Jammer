/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Hardware integration test of the esp_sdr receive API's contract: error
 * codes and boundary values, plus one real capture per supported rate.
 * Runs on the real radio (xiao_esp32s3 or xiao_esp32c6); needs no antenna,
 * peer board or host tool, and never transmits.
 */
#include <errno.h>

#include <zephyr/init.h>
#include <zephyr/ztest.h>

#include <esp_sdr/esp_sdr.h>

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include "app_usb.h"

/*
 * The S3 variant needs a USB shell and DFU to deploy (see Kconfig): start
 * them before ztest's own main() runs the suites below. A SYS_INIT hook,
 * not a competing main() of our own: CONFIG_ZTEST provides main() itself
 * (ztest.c), not merely a weak default one.
 */
static int usb_bringup(void)
{
	(void)app_usb_init();
	return 0;
}
SYS_INIT(usb_bringup, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif

static void *suite_setup(void)
{
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	/*
	 * The console is USB CDC-ACM on this variant: ztest's own main() runs
	 * immediately at boot, well before the host has enumerated the
	 * device (and the test suite finishes in milliseconds, so there is
	 * no second chance to reconnect and catch it live). Give the host a
	 * few seconds.
	 */
	k_sleep(K_SECONDS(10));
#endif
	zassert_ok(esp_sdr_init(), "esp_sdr_init() failed");
	zassert_ok(esp_sdr_set_freq(2412), "set_freq(2412) failed");
	return NULL;
}

ZTEST_SUITE(esp_sdr_rx_api, NULL, suite_setup, NULL, NULL, NULL);

ZTEST(esp_sdr_rx_api, test_set_freq_bounds)
{
	zassert_equal(esp_sdr_set_freq(ESP_SDR_FREQ_MIN_MHZ - 1), -EINVAL);
	zassert_equal(esp_sdr_set_freq(ESP_SDR_FREQ_MAX_MHZ + 1), -EINVAL);
	zassert_ok(esp_sdr_set_freq(2412));
	zassert_equal(esp_sdr_get_freq(), 2412);
}

ZTEST(esp_sdr_rx_api, test_set_channel_bw_bounds)
{
	zassert_equal(esp_sdr_set_channel_bw(3), -EINVAL);
	zassert_ok(esp_sdr_set_channel_bw(0));
}

ZTEST(esp_sdr_rx_api, test_capture_rejects_bad_count)
{
	struct esp_sdr_rx_burst b;

	zassert_equal(esp_sdr_rx_capture(ESP_SDR_RATE_80MSPS, ESP_SDR_SAMPLES_MIN - 1, &b),
		      -EINVAL);
	zassert_equal(esp_sdr_rx_capture(ESP_SDR_RATE_80MSPS, ESP_SDR_SAMPLES_MAX + 1, &b),
		      -EINVAL);
}

/* Whichever native rate this SoC does not support (esp_sdr_rx_rate_hz() returns 0 for
 * it) must be rejected outright; which that is varies by SoC (the C6 captures at
 * 80 MS/s only), so ask the library instead of assuming.
 */
ZTEST(esp_sdr_rx_api, test_capture_rejects_unsupported_rate)
{
	static const enum esp_sdr_rate rates[] = {
		ESP_SDR_RATE_80MSPS,
		ESP_SDR_RATE_40MSPS,
		ESP_SDR_RATE_16MSPS,
	};
	struct esp_sdr_rx_burst b;
	bool checked_one = false;

	ARRAY_FOR_EACH(rates, k) {
		if (esp_sdr_rx_rate_hz(rates[k]) == 0U) {
			zassert_equal(esp_sdr_rx_capture(rates[k], 256, &b), -EINVAL,
				      "rate index %u: this SoC does not support it, but capture"
				      " did not reject it",
				      rates[k]);
			checked_one = true;
		}
	}
	if (!checked_one) {
		ztest_test_skip(); /* this SoC supports all three native rates */
	}
}

/* One real capture of the requested length at every rate this SoC supports. */
ZTEST(esp_sdr_rx_api, test_capture_returns_requested_count)
{
	static const enum esp_sdr_rate rates[] = {
		ESP_SDR_RATE_80MSPS,
		ESP_SDR_RATE_40MSPS,
		ESP_SDR_RATE_16MSPS,
	};

	/* Both SoCs capture at 80 MS/s: without it the loop below would test nothing. */
	zassert_true(esp_sdr_rx_rate_hz(ESP_SDR_RATE_80MSPS) > 0U, "80 MS/s reported unsupported");
	ARRAY_FOR_EACH(rates, k) {
		struct esp_sdr_rx_burst b = {0};
		int ret;

		if (esp_sdr_rx_rate_hz(rates[k]) == 0U) {
			continue;
		}
		ret = esp_sdr_rx_capture(rates[k], 1024, &b);
		zassert_ok(ret, "rate index %u: capture failed (%d)", rates[k], ret);
		zassert_equal(b.count, 1024, "rate index %u: got %zu samples, asked for 1024",
			      rates[k], b.count);
		zassert_not_null(b.words, "rate index %u: no sample buffer", rates[k]);
	}
}

ZTEST(esp_sdr_rx_api, test_gain_bounds)
{
	int max = esp_sdr_rx_gain_max();

	zassert_equal(esp_sdr_rx_set_gain(ESP_SDR_RX_GAIN_AUTO - 1), -EINVAL);
	zassert_equal(esp_sdr_rx_set_gain(max + 1), -EINVAL);
	zassert_ok(esp_sdr_rx_set_gain(0));
	zassert_equal(esp_sdr_rx_get_gain(), 0);
	/* Restore the hardware AGC: a later test, or a human on the shell, should not
	 * find the gain stuck at a fixed index this test chose.
	 */
	zassert_ok(esp_sdr_rx_set_gain(ESP_SDR_RX_GAIN_AUTO));
	zassert_equal(esp_sdr_rx_get_gain(), ESP_SDR_RX_GAIN_AUTO);
}

ZTEST(esp_sdr_rx_api, test_lpf_bounds)
{
	zassert_equal(esp_sdr_rx_set_lpf(ESP_SDR_RX_LPF_AUTO - 1), -EINVAL);
	zassert_equal(esp_sdr_rx_set_lpf(ESP_SDR_RX_LPF_MAX + 1), -EINVAL);
	zassert_ok(esp_sdr_rx_set_lpf(0));
	zassert_equal(esp_sdr_rx_get_lpf(), 0);
	zassert_ok(esp_sdr_rx_set_lpf(ESP_SDR_RX_LPF_AUTO)); /* restore */
}

ZTEST(esp_sdr_rx_api, test_bandwidth_bounds)
{
	uint32_t min, max;

	esp_sdr_rx_bandwidth_range(&min, &max);
	zassert_true(min > 0 && max > min, "bandwidth range %u..%u MHz looks wrong", min, max);
	zassert_equal(esp_sdr_rx_set_bandwidth(max + 1), -EINVAL);
	zassert_ok(esp_sdr_rx_set_bandwidth(0)); /* 0: the PHY's own setting, always valid */
	zassert_ok(esp_sdr_rx_set_lpf(ESP_SDR_RX_LPF_AUTO)); /* restore */
}
