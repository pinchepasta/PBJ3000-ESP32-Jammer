/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QAM link at 80 MS/s with a CSMA/CA MAC and an iperf style test on the
 * shell ("link"). The MAC thread owns the radio on one CPU, the system
 * threads go to the other.
 */

#include <zephyr/app_version.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app_cpu.h"
#include "app_usb.h"
#include "mac.h"

LOG_MODULE_REGISTER(link, LOG_LEVEL_INF);

int main(void)
{
	printk("link %s\n", APP_VERSION_STRING);
	app_cpu_pin_system_threads(CONFIG_APP_NET_CPU);
	if (app_usb_init() != 0) {
		LOG_ERR("usb init failed");
	}
	if (link_mac_init() != 0) {
		LOG_ERR("mac init failed");
		return 0;
	}
	/* Again for the threads started since (usbd, UDC). */
	k_sleep(K_MSEC(500));
	app_cpu_pin_system_threads(CONFIG_APP_NET_CPU);
	return 0;
}
