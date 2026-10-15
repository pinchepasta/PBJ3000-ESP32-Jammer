/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * USB SDR after HackRF's model, for osmosdr / GNU Radio hosts: a vendor
 * interface with control requests and bulk sample endpoints (esdr_proto.h),
 * next to the CDC-ACM shell and DFU. The receive ring owns CPU 1; the system
 * threads and USB stay on CPU 0.
 */

#include <zephyr/app_version.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app_cpu.h"
#include "app_usb.h"
#include "sdr.h"

LOG_MODULE_REGISTER(osmosdr, LOG_LEVEL_INF);

int main(void)
{
	printk("osmosdr %s\n", APP_VERSION_STRING);
	/* Before USB starts: its controller interrupt lands on the CPU that allocates it. */
	app_cpu_pin_system_threads(0);
	if (app_usb_init() != 0) {
		LOG_ERR("usb init failed");
	}
	if (sdr_init() != 0) {
		LOG_ERR("sdr init failed");
		return 0;
	}
	/* Again for the threads started since (usbd, UDC). */
	k_sleep(K_MSEC(500));
	app_cpu_pin_system_threads(0);
	return 0;
}
