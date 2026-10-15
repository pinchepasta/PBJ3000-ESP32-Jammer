/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Radio core, ported from esp-sdr main/targets/esp32s3/receiver.c and
 * main/targets/esp32c6/chip.h: bring-up, the shared LO (both directions use
 * it) and the dump bank.
 */

#include <errno.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>

#include <esp_wifi.h>

/* Upstream helpers, used unmodified. */
#if !defined(CONFIG_SOC_SERIES_ESP32C5)
#include "rx_lo.h"
#endif
#include "rx_tuning.h"

#include "esp_sdr_priv.h"

LOG_MODULE_REGISTER(esp_sdr, CONFIG_ESP_SDR_LOG_LEVEL);

#if defined(CONFIG_SOC_SERIES_ESP32C6)
#include <zephyr/init.h>
#include <hal/modem_lpcon_ll.h>
#include <hal/modem_syscon_ll.h>

/*
 * A chip reset (USB-Serial-JTAG, esptool) leaves the C6's modem subsystem as
 * the previous firmware left it; ESP-IDF resets it on a restart, a chip reset
 * does not. After the native 802.15.4 radio had run, the receive floor stayed
 * about 25 dB up and the vendor's channel tuning landed MHz off, until a power
 * cycle or deep sleep. Reset it before the Wi-Fi driver brings the radio up.
 */
static int esp_sdr_modem_reset(void)
{
	modem_syscon_ll_reset_all(&MODEM_SYSCON);
	modem_lpcon_ll_reset_all(&MODEM_LPCON);
	return 0;
}
SYS_INIT(esp_sdr_modem_reset, PRE_KERNEL_1, 0);
#endif

K_MUTEX_DEFINE(esp_sdr_lock);
struct esp_sdr_stats esp_sdr_counters;
bool esp_sdr_ready;
uint32_t esp_sdr_freq_mhz = CAL_CHANNEL_MHZ;
int32_t esp_sdr_fofs_khz;
uint32_t esp_sdr_turn_settle_us = SDR_RETUNE_SETTLE_US;
/* Channel bandwidth flag for the vendor tuning call, 0 = 20 MHz. */
static unsigned int sdr_cbw;
bool esp_sdr_turn_retune = true;

#if !defined(CONFIG_SOC_SERIES_ESP32C5)
static bool is_channel(uint32_t mhz)
{
	return (mhz >= 2412U && mhz <= 2472U && (mhz - 2412U) % 5U == 0U) || mhz == 2484U;
}
#endif

/*
 * LO plan. Upstream's rx_lo_plan() uses the 5/6 divider from 1842 MHz only and
 * tries the PLL directly below. The VCO does not reach below about 2130 MHz
 * (ESP32-C6) to 2180 MHz (ESP32-S3), so below 2210 MHz the divider is always
 * the better choice: it reaches down to the VCO's lower limit / 1.2.
 */
#define LO_DIVIDER_BELOW_MHZ 2210U

bool esp_sdr_lo_divided(void)
{
#if defined(CONFIG_SOC_SERIES_ESP32C5)
	/* No divided LO path qualified on the C5. */
	return false;
#else
	return esp_sdr_freq_mhz < LO_DIVIDER_BELOW_MHZ;
#endif
}

/*
 * RF PLL status after the vendor calibration (both chips, I2C block 0x62):
 * bits 3:2 compare the requested frequency with the VCO's tuning window: 0
 * inside it, 1 below it (capacitor code at its maximum), 2 above it (code 0).
 * Outside the window the PLL may still lock for about 12 MHz (measured on one
 * S3 and one C6), so this errs on the safe side.
 */
#define RFPLL_BLOCK       0x62U
#define RFPLL_STATUS_REG  12U
#define RFPLL_STATUS_VCO  0x0cU

static bool pll_in_window(void)
{
#if defined(CONFIG_SOC_SERIES_ESP32C5)
	/* The C5's PLL status bits are not mapped yet. */
	return true;
#else
	unsigned int status;

	regi2c_enter_critical();
	status = rx_lo_read(RFPLL_BLOCK, RX_LO_HOST, RFPLL_STATUS_REG);
	regi2c_exit_critical();
	return (status & RFPLL_STATUS_VCO) == 0U;
#endif
}

#if defined(CONFIG_SOC_SERIES_ESP32C5)
void esp_sdr_tune(void)
{
	/* A full channel setup at any MHz: the PHY takes values above 14 as MHz. */
	phy_set_chanfreq(esp_sdr_freq_mhz, sdr_cbw);
	if (esp_sdr_fofs_khz != 0) {
		phy_set_rf_freq_offset(phy_param[PHY_PARAM_XTAL], esp_sdr_freq_mhz, esp_sdr_fofs_khz);
	}
}
#else
void esp_sdr_tune(void)
{
	bool divided = esp_sdr_lo_divided();
	uint32_t pll_khz = esp_sdr_freq_mhz * (divided ? 1200U : 1000U);
	rx_lo_plan_t plan = {pll_khz / 1000U, (int)(pll_khz % 1000U), divided};
	bool channel = esp_sdr_fofs_khz == 0 && is_channel(esp_sdr_freq_mhz);

	regi2c_enter_critical();
	rx_lo_select(false);
	regi2c_exit_critical();
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	set_chanfreq(channel ? esp_sdr_freq_mhz : CAL_CHANNEL_MHZ, sdr_cbw);
	if (!channel) {
		set_rf_freq_offset(0, plan.mhz, plan.offset_khz + esp_sdr_fofs_khz);
	}
#else
	/* The channel call goes through mhz2ieee and loses off-grid requests. */
	chip_v7_set_chan(channel ? esp_sdr_freq_mhz : CAL_CHANNEL_MHZ, sdr_cbw);
	if (!channel) {
		phy_set_freq(plan.mhz, plan.offset_khz + esp_sdr_fofs_khz);
	}
#endif
}
#endif

int esp_sdr_bank_usage(uint32_t *usage)
{
	uintptr_t base = (uintptr_t)__esp_sdr_bank_start;
	uintptr_t bank = (base - DUMP_BANK0_ADDR) / DUMP_BANK_SIZE;

	if (base < DUMP_BANK0_ADDR || (base - DUMP_BANK0_ADDR) % DUMP_BANK_SIZE != 0U ||
	    bank >= DUMP_BANKS) {
		return -ENOMEM;
	}
	*usage = (BIT(bank) << DUMP_USAGE_S) | DUMP_USAGE_ALLOC;
	return 0;
}

int esp_sdr_init(void)
{
	const struct device *wifi = DEVICE_DT_GET(DT_NODELABEL(wifi));
	uint32_t usage;
	int ret = 0;

	if (!device_is_ready(wifi)) {
		return -ENODEV;
	}
	if (esp_sdr_bank_usage(&usage) != 0) {
		LOG_ERR("capture bank %p is not a dump bank", (void *)__esp_sdr_bank_start);
		return -ENOMEM;
	}

	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	if (esp_sdr_ready) {
		goto out;
	}
	/* The driver already applies its Kconfig power-save mode at boot. */
	if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) {
		LOG_WRN("power save still enabled");
	}
	/* Promiscuous first: NULL mode only accepts a channel in promiscuous mode. */
	if (esp_wifi_set_promiscuous(true) != ESP_OK) {
		ret = -EIO;
		goto out;
	}
	if (esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
		(void)esp_wifi_set_promiscuous(false);
		ret = -EIO;
		goto out;
	}
	esp_sdr_rx_prepare();
	esp_sdr_ready = true;
	LOG_INF("receiver ready, bank %p", (void *)__esp_sdr_bank_start);
out:
	k_mutex_unlock(&esp_sdr_lock);
	return ret;
}

int esp_sdr_retune(void)
{
	if (!esp_sdr_ready) {
		return -EAGAIN;
	}
	esp_sdr_rx_prepare();
	return 0;
}

int esp_sdr_set_freq(uint32_t mhz)
{
	int ret;

	if (!rx_frequency_valid(mhz)) {
		return -EINVAL;
	}
	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	esp_sdr_freq_mhz = mhz;
	ret = esp_sdr_retune();
	if (ret == 0 && !pll_in_window()) {
		ret = -ERANGE;
	}
	k_mutex_unlock(&esp_sdr_lock);
	return ret;
}

uint32_t esp_sdr_get_freq(void)
{
	return esp_sdr_freq_mhz;
}

int esp_sdr_set_freq_offset(int32_t khz)
{
	int ret;

	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	esp_sdr_fofs_khz = khz;
	ret = esp_sdr_retune();
	if (ret == 0 && !pll_in_window()) {
		ret = -ERANGE;
	}
	k_mutex_unlock(&esp_sdr_lock);
	return ret;
}

int esp_sdr_set_turnaround(uint32_t settle_us, bool retune)
{
	if (settle_us > SDR_RETUNE_SETTLE_US) {
		return -EINVAL;
	}
	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	esp_sdr_turn_settle_us = settle_us;
	esp_sdr_turn_retune = retune;
	k_mutex_unlock(&esp_sdr_lock);
	return 0;
}

int esp_sdr_set_channel_bw(unsigned int cbw)
{
	int ret;

	if (cbw > 2U) {
		return -EINVAL;
	}
	k_mutex_lock(&esp_sdr_lock, K_FOREVER);
	sdr_cbw = cbw;
	ret = esp_sdr_retune();
	k_mutex_unlock(&esp_sdr_lock);
	return ret;
}

void esp_sdr_get_stats(struct esp_sdr_stats *stats)
{
	*stats = esp_sdr_counters;
}
