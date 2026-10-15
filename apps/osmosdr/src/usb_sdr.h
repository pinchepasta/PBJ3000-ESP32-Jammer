/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The SDR's USB function: one vendor interface with a bulk IN and a bulk OUT
 * endpoint, and the vendor control requests of esdr_proto.h.
 */

#ifndef USB_SDR_H_
#define USB_SDR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Whether the host has configured the interface (bulk transfers possible). */
bool usb_sdr_enabled(void);

/*
 * Queue @p len bytes at @p data (internal RAM, stays untouched until done) on
 * bulk IN, or a receive buffer of @p cap bytes on bulk OUT. Completion calls
 * the handlers below from the USB stack thread, in order per endpoint.
 */
int usb_sdr_submit_in(uint8_t *data, size_t len);
int usb_sdr_submit_out(uint8_t *data, size_t cap);
/* Cancel everything queued on both endpoints (completions report -ECONNABORTED). */
void usb_sdr_cancel(void);

/* Provided by the app. */
void usb_sdr_in_done(uint8_t *data, size_t len, int err);
void usb_sdr_out_done(uint8_t *data, size_t len, int err);
/* Interface enabled (alternate setting 0 selected) or gone. */
void usb_sdr_link(bool up);
/*
 * Vendor requests: to the device with @p len bytes of data (0 for none), or
 * to the host filling at most @p max bytes (return the length). Called from
 * the USB stack thread: must not block. Negative errno stalls.
 */
int usb_sdr_control_out(uint8_t req, uint16_t value, const uint8_t *data, size_t len);
int usb_sdr_control_in(uint8_t req, uint16_t value, uint8_t *buf, size_t max);

#endif /* USB_SDR_H_ */
