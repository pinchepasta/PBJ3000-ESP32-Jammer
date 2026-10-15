/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * multiapp host (doc/multiapp.md): the USB shell and DFU, the radio, and the
 * apps built as extensions (apps.c).
 */

#include <zephyr/app_version.h>
#include <zephyr/kernel.h>

#include <esp_sdr/esp_sdr.h>

#include "app_cpu.h"
#include "app_usb.h"
#include "apps.h"

int main(void)
{
	int ret;

	printk("multiapp %s\n", APP_VERSION_STRING);
	/* Before USB: its interrupt lands on the CPU that allocates it, CPU 1 is the radio's. */
	app_cpu_pin_system_threads(0);
	ret = esp_sdr_init();
	if (ret != 0) {
		printk("multiapp: radio init failed (%d)\n", ret);
	}
	printk("multiapp: ready\n");
	/*
	 * The files while bank 0 and the flash are the host's alone, then the app
	 * before USB, whose classes call its handlers (osmosdr's).
	 */
	(void)multiapp_drive_scan();
	if (!multiapp_boot()) {
		/* No app: the volume goes to the PC as a USB drive. */
		(void)multiapp_drive_export();
	}
	/* The USB shell and DFU, and the trial watchdog that reverts an image that hangs. */
	if (app_usb_init() != 0) {
		printk("multiapp: usb init failed\n");
	}
	multiapp_note_main_stack();
	return 0;
}
