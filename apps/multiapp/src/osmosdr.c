/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * osmosdr's vendor USB function lives in the host: apps/osmosdr/src/usb_sdr.c
 * is built into the image, as USBD_DEFINE_CLASS() and its buffer pool need
 * the host's sections. Its handlers (usb_sdr.h) are osmosdr's own, found in
 * the loaded app by name; without it, transfers end and requests stall.
 */

#include <errno.h>

#include "apps.h"
#include "usb_sdr.h"

/* The loaded app's handler @p name, looked up once it is there. */
#define HANDLER(fn, name)                                                                          \
	do {                                                                                       \
		if ((fn) == NULL) {                                                                \
			(fn) = multiapp_app_fn(name);                                              \
		}                                                                                  \
	} while (false)

void usb_sdr_in_done(uint8_t *data, size_t len, int err)
{
	static void (*fn)(uint8_t *data, size_t len, int err);

	HANDLER(fn, "usb_sdr_in_done");
	if (fn != NULL) {
		fn(data, len, err);
	}
}

void usb_sdr_out_done(uint8_t *data, size_t len, int err)
{
	static void (*fn)(uint8_t *data, size_t len, int err);

	HANDLER(fn, "usb_sdr_out_done");
	if (fn != NULL) {
		fn(data, len, err);
	}
}

void usb_sdr_link(bool up)
{
	static void (*fn)(bool up);

	HANDLER(fn, "usb_sdr_link");
	if (fn != NULL) {
		fn(up);
	}
}

int usb_sdr_control_out(uint8_t req, uint16_t value, const uint8_t *data, size_t len)
{
	static int (*fn)(uint8_t req, uint16_t value, const uint8_t *data, size_t len);

	HANDLER(fn, "usb_sdr_control_out");
	return fn != NULL ? fn(req, value, data, len) : -ENOTSUP;
}

int usb_sdr_control_in(uint8_t req, uint16_t value, uint8_t *buf, size_t max)
{
	static int (*fn)(uint8_t req, uint16_t value, uint8_t *buf, size_t max);

	HANDLER(fn, "usb_sdr_control_in");
	return fn != NULL ? fn(req, value, buf, max) : -ENOTSUP;
}
