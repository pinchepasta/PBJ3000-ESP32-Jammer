/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef APP_USB_H_
#define APP_USB_H_

#include <stdbool.h>
#include <stddef.h>

/*
 * Schedule the USB device start (CDC-ACM shell, the app's classes, DFU) and
 * arm the trial watchdog on an unconfirmed image; returns at once.
 */
int app_usb_init(void);

struct usbd_context;

/*
 * Called between class registration and usbd_init(), to unregister the
 * classes this boot does not use. Weak, keeps them all.
 */
void app_usb_classes(struct usbd_context *ctx);

/*
 * Called in the USB thread when the host detaches into DFU, before the
 * download: stop whatever keeps a CPU from taking flash stall requests
 * (an esp_sdr ring run). Weak, empty by default.
 */
void app_usb_dfu_prepare(void);

/*
 * After a DFU download: whether it went to the image slot, which MCUboot then
 * test-boots. Weak, true.
 */
bool app_usb_dfu_was_slot(void);

/* The USB thread's stack use in the last DFU download (it writes the flash), or false. */
bool app_usb_dfu_stack(size_t *used, size_t *size);

#endif /* APP_USB_H_ */
