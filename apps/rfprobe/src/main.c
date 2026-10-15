/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Bench probe for the radio: tune (through the library's frequency plan or
 * the PLL directly), loop a tone (ESP32-S3), dump raw captures as base64 for
 * host-side analysis and read the analog I2C registers. tools/rfprobe.py
 * drives it; README.rst lists the commands.
 */

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/app_version.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/base64.h>
#include <zephyr/sys/util.h>

#include <esp_rom_sys.h>

#include <esp_sdr/esp_sdr.h>
#include <esp_sdr/esp_sdr_rx.h>
#include <esp_sdr/esp_sdr_tx.h>

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include "app_usb.h"
#endif

#if defined(CONFIG_POWEROFF)
#include <esp_sleep.h>
#include <zephyr/sys/poweroff.h>
#endif

#define TWO_PI 6.28318530717958647692

/* Vendor and ROM calls, as lib/esp_sdr uses them (doc/undocumented-registers-*.md). */
extern void regi2c_enter_critical(void);
extern void regi2c_exit_critical(void);
#if defined(CONFIG_SOC_SERIES_ESP32C5)
/* The C5 PHY's I2C read and direct PLL setting; phy_param byte 49 is the crystal selector. */
extern unsigned int phy_chip_i2c_readReg(unsigned int block, unsigned int host, unsigned int reg);
#define rom_chip_i2c_readReg phy_chip_i2c_readReg
extern void phy_set_rf_freq_offset(unsigned int xtal, unsigned int mhz, int khz);
extern unsigned char phy_param[];
#else
extern unsigned int rom_chip_i2c_readReg(unsigned int block, unsigned int host, unsigned int reg);
#endif
#if defined(CONFIG_SOC_SERIES_ESP32S3)
extern void set_rf_freq_offset(unsigned int xtal, unsigned int mhz, int khz);
#elif !defined(CONFIG_SOC_SERIES_ESP32C5)
extern void phy_set_freq(unsigned int mhz, int khz);
#endif

/* Settle after a direct PLL write, as the library's retune. */
#define RAW_SETTLE_US 3000U

#if defined(CONFIG_SOC_SERIES_ESP32S3)
/* A tone's loop session holds the radio until "probe tone off". */
static bool tone_active;
#endif

static long arg(char **argv, size_t argc, size_t i, long def)
{
	return argc > i ? strtol(argv[i], NULL, 0) : def;
}

static bool busy(const struct shell *sh)
{
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	if (tone_active) {
		shell_error(sh, "tone on: probe tone off first");
		return true;
	}
#else
	ARG_UNUSED(sh);
#endif
	return false;
}

/* f <MHz>: tune through the library's plan; prints esp_sdr_set_freq()'s return. */
static int cmd_f(const struct shell *sh, size_t argc, char **argv)
{
	long mhz = arg(argv, argc, 1, 2412);
	int ret;

	if (busy(sh)) {
		return -EBUSY;
	}
	(void)esp_sdr_set_turnaround(RAW_SETTLE_US, true);
	ret = mhz > 0 ? esp_sdr_set_freq((uint32_t)mhz) : -EINVAL;
	shell_print(sh, "f %ld ret %d", mhz, ret);
	return 0;
}

/*
 * raw <PLL MHz> [kHz]: program the PLL directly, bypassing the frequency plan,
 * and stop retuning around captures and transmissions. The LO divider stays as
 * the last "f" left it (on below 2210 MHz), so the LO is the PLL or PLL / 1.2.
 */
static int cmd_raw(const struct shell *sh, size_t argc, char **argv)
{
	long mhz = arg(argv, argc, 1, 2412);
	long khz = arg(argv, argc, 2, 0);

	if (busy(sh)) {
		return -EBUSY;
	}
	if (mhz <= 0) {
		shell_error(sh, "raw: MHz must be positive");
		return -EINVAL;
	}
	(void)esp_sdr_set_turnaround(RAW_SETTLE_US, false);
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	set_rf_freq_offset(0, (unsigned int)mhz, (int)khz);
#elif defined(CONFIG_SOC_SERIES_ESP32C5)
	phy_set_rf_freq_offset(phy_param[49], (unsigned int)mhz, (int)khz);
#else
	phy_set_freq((unsigned int)mhz, (int)khz);
#endif
	k_busy_wait(RAW_SETTLE_US);
	shell_print(sh, "raw %ld %ld", mhz, khz);
	return 0;
}

/* lpf <rx> [txa txb]: low-pass codes, -1 the PHY's, 0 the widest. */
static int cmd_lpf(const struct shell *sh, size_t argc, char **argv)
{
	int ret = esp_sdr_rx_set_lpf((int)arg(argv, argc, 1, 0));
	int tret = -ENOTSUP;

	if (argc > 2) {
		tret = esp_sdr_tx_set_lpf((int)arg(argv, argc, 2, 0),
					  (int)arg(argv, argc, 3, arg(argv, argc, 2, 0)));
	}
	shell_print(sh, "lpf rx %d ret %d tx ret %d", esp_sdr_rx_get_lpf(), ret, tret);
	return 0;
}

/* gain <rx index|-1> [tx step]: fixed receive gain or AGC; transmit step (S3). */
static int cmd_gain(const struct shell *sh, size_t argc, char **argv)
{
	int ret = esp_sdr_rx_set_gain((int)arg(argv, argc, 1, ESP_SDR_RX_GAIN_AUTO));

#if defined(CONFIG_SOC_SERIES_ESP32S3)
	if (argc > 2) {
		int tret = esp_sdr_tx_set_gain((int)arg(argv, argc, 2, 0));

		shell_print(sh, "txgain %d ret %d max %d", esp_sdr_tx_get_gain(), tret,
			    esp_sdr_tx_gain_max());
	}
#endif
	shell_print(sh, "rxgain %d ret %d max %d", esp_sdr_rx_get_gain(), ret,
		    esp_sdr_rx_gain_max());
	return 0;
}

#if defined(CONFIG_SOC_SERIES_ESP32S3)
/* tone on <Hz> [amp] [rate 0|1]: a whole number of periods in the bank, looped. */
static int cmd_tone_on(const struct shell *sh, size_t argc, char **argv)
{
	long hz = arg(argv, argc, 1, 1000000);
	long amp = arg(argv, argc, 2, 400);
	enum esp_sdr_rate rate = (enum esp_sdr_rate)arg(argv, argc, 3, ESP_SDR_RATE_80MSPS);
	double fs = (double)esp_sdr_tx_rate_hz(rate);
	double best = INFINITY;
	size_t words, n = 0;
	long cycles = 0;
	uint32_t *bank;
	int ret;

	if (busy(sh)) {
		return -EBUSY;
	}
	if (fs == 0.0 || fabs((double)hz) >= fs / 2.0 || amp < 1 || amp > 511) {
		shell_error(sh, "tone: rate 0 or 1, |Hz| below rate / 2, amp 1..511");
		return -EINVAL;
	}
	ret = esp_sdr_tx_loop_begin(rate);
	if (ret != 0) {
		shell_error(sh, "tone: begin %d", ret);
		return ret;
	}
	bank = esp_sdr_tx_loop_buf(0, &words);
	words = MIN(words, ESP_SDR_SAMPLES_MAX);
	/* The period length whose whole number of cycles comes closest to hz. */
	for (size_t len = words; len >= ESP_SDR_SAMPLES_MIN; len--) {
		long c = lround((double)hz * (double)len / fs);
		double err = fabs((double)c * fs / (double)len - (double)hz);

		if (err < best) {
			best = err;
			n = len;
			cycles = c;
			if (err < 1.0) {
				break;
			}
		}
	}
	for (size_t k = 0; k < n; k++) {
		double cyc = (double)cycles * (double)k / (double)n;
		double ph = TWO_PI * (cyc - floor(cyc));

		bank[k] = esp_sdr_tx_word((int16_t)lround((double)amp * cos(ph)),
					  (int16_t)lround((double)amp * sin(ph)));
	}
	ret = esp_sdr_tx_loop_start(0, n);
	if (ret != 0) {
		esp_sdr_tx_loop_end();
		shell_error(sh, "tone: start %d", ret);
		return ret;
	}
	tone_active = true;
	shell_print(sh, "tone on %u MHz %+ld Hz period %u amp %ld txgain %d",
		    esp_sdr_get_freq(), lround((double)cycles * fs / (double)n), (unsigned int)n,
		    amp, esp_sdr_tx_get_gain());
	return 0;
}

static int cmd_tone_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (tone_active) {
		esp_sdr_tx_loop_end();
		tone_active = false;
	}
	shell_print(sh, "tone off");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(tone_cmds,
	SHELL_CMD_ARG(on, NULL, "<Hz> [amp] [rate 0|1]: loop a tone at the LO + Hz", cmd_tone_on,
		      1, 3),
	SHELL_CMD(off, NULL, "stop the tone", cmd_tone_off),
	SHELL_SUBCMD_SET_END);
#endif

/* rx [n] [rate]: one capture, dumped as base64 of the raw little-endian words. */
static int cmd_rx(const struct shell *sh, size_t argc, char **argv)
{
	size_t n = (size_t)arg(argv, argc, 1, ESP_SDR_SAMPLES_MAX);
	enum esp_sdr_rate rate = (enum esp_sdr_rate)arg(argv, argc, 2, ESP_SDR_RATE_80MSPS);
	struct esp_sdr_rx_burst b = {0};
	char line[80];
	int ret;

	if (busy(sh)) {
		return -EBUSY;
	}
	ret = esp_sdr_rx_capture(rate, n, &b);
	if (ret != 0) {
		shell_print(sh, "rx ret %d", ret);
		return 0;
	}
	shell_print(sh, "rx n %u f %u fs %u lpf %d gain %d", (unsigned int)b.count,
		    esp_sdr_get_freq(), esp_sdr_rx_rate_hz(rate), esp_sdr_rx_get_lpf(),
		    esp_sdr_rx_get_gain());
	for (size_t w = 0; w < b.count; w += 12) {
		size_t words = MIN(12U, b.count - w), olen;

		(void)base64_encode((uint8_t *)line, sizeof(line), &olen,
				    (const uint8_t *)&b.words[w], words * 4U);
		line[olen] = '\0';
		shell_print(sh, "d %s", line);
	}
	shell_print(sh, "end");
	return 0;
}

/* dump <block> [regs]: analog I2C registers 0..regs-1 of one block, read only. */
static int cmd_dump(const struct shell *sh, size_t argc, char **argv)
{
	unsigned int block = (unsigned int)arg(argv, argc, 1, 0x62);
	unsigned int regs = (unsigned int)CLAMP(arg(argv, argc, 2, 16), 1, 64);
	char line[3 * 64 + 1];
	int len = 0;

	for (unsigned int r = 0; r < regs; r++) {
		unsigned int v;

		/* The host argument is ignored by the ROM: the block picks the master. */
		regi2c_enter_critical();
		v = rom_chip_i2c_readReg(block, 1, r) & 0xffU;
		regi2c_exit_critical();
		len += snprintf(line + len, sizeof(line) - len, " %02x", v);
	}
	shell_print(sh, "dump %02x:%s", block, line);
	return 0;
}

#if defined(CONFIG_POWEROFF)
/* sleep <ms>: deep sleep with a timer wakeup; the chip boots again afterwards. */
static int cmd_sleep(const struct shell *sh, size_t argc, char **argv)
{
	long ms = CLAMP(arg(argv, argc, 1, 100), 1, 60000);

	if (busy(sh)) {
		return -EBUSY;
	}
	shell_print(sh, "sleep %ld ms", ms);
	k_msleep(50);
	esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000U);
	sys_poweroff();
	return 0;
}
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(probe_cmds,
	SHELL_CMD_ARG(f, NULL, "<MHz>: tune through the library's plan", cmd_f, 2, 0),
	SHELL_CMD_ARG(raw, NULL, "<PLL MHz> [kHz]: program the PLL directly", cmd_raw, 2, 1),
	SHELL_CMD_ARG(lpf, NULL, "<rx> [txa [txb]]: low-pass codes, -1 PHY, 0 widest", cmd_lpf,
		      2, 2),
	SHELL_CMD_ARG(gain, NULL, "<rx index|-1> [tx step]", cmd_gain, 2, 1),
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	SHELL_CMD(tone, &tone_cmds, "loop a tone", NULL),
#endif
	SHELL_CMD_ARG(rx, NULL, "[n] [rate 0..6]: capture, base64 dump", cmd_rx, 1, 2),
	SHELL_CMD_ARG(dump, NULL, "<block> [regs]: analog I2C registers", cmd_dump, 2, 1),
#if defined(CONFIG_POWEROFF)
	SHELL_CMD_ARG(sleep, NULL, "<ms>: deep sleep, timer wakeup", cmd_sleep, 2, 0),
#endif
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(probe, &probe_cmds, "Radio bench probe", NULL);

int main(void)
{
	int ret;

	printk("rfprobe %s\n", APP_VERSION_STRING);
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	/* S3: the USB shell and DFU, and the trial watchdog that reverts an image that hangs. */
	if (app_usb_init() != 0) {
		printk("rfprobe: usb init failed\n");
	}
#endif
	printk("rfprobe: reset reason %d\n", (int)esp_rom_get_reset_reason(0));
	ret = esp_sdr_init();
	if (ret != 0) {
		printk("rfprobe: init failed (%d)\n", ret);
		return 0;
	}
	/* Every filter wide open. */
	(void)esp_sdr_rx_set_lpf(0);
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	(void)esp_sdr_tx_set_lpf(0, 0);
#endif
	printk("rfprobe: ready\n");
	return 0;
}
