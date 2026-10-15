/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Vendor USB function of the SDR (usb_sdr.h): one interface, bulk IN for
 * receive blocks, bulk OUT for transmit samples. Transfers point at the
 * app's own buffers (net_buf_alloc_with_data), so no data is copied here.
 * Control requests to the device are claimed by bRequest and passed to the
 * app.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/usb/udc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usbd.h>

#include "esdr_proto.h"
#include "usb_sdr.h"

LOG_MODULE_REGISTER(usb_sdr, LOG_LEVEL_INF);

/* Bulk transfers in flight on both endpoints together. */
#define SDR_XFERS 16

UDC_BUF_POOL_DEFINE(sdr_ep_pool, SDR_XFERS, 0, sizeof(struct udc_buf_info), NULL);

static const struct usbd_cctx_vendor_req sdr_vregs = USBD_VENDOR_REQ(
	ESDR_REQ_GET_INFO, ESDR_REQ_SET_MODE, ESDR_REQ_SET_FREQ, ESDR_REQ_SET_SAMPLE_RATE,
	ESDR_REQ_SET_RX_GAIN, ESDR_REQ_SET_TX_GAIN, ESDR_REQ_SET_BANDWIDTH, ESDR_REQ_SET_FORMAT,
	ESDR_REQ_SET_FREQ_CORR, ESDR_REQ_GET_STATE, ESDR_REQ_GET_STATS, ESDR_REQ_SET_OPTIONS,
	ESDR_REQ_SET_DIGITAL_GAIN);

struct sdr_desc {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor in_ep;
	struct usb_ep_descriptor out_ep;
	struct usb_desc_header nil_desc;
};

static struct sdr_desc sdr_desc = {
	.if0 = {
		.bLength = sizeof(struct usb_if_descriptor),
		.bDescriptorType = USB_DESC_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_BCC_VENDOR,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 0,
	},
	.in_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = 0x81,
		.bmAttributes = USB_EP_TYPE_BULK,
		.wMaxPacketSize = sys_cpu_to_le16(64U),
		.bInterval = 0,
	},
	.out_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = 0x01,
		.bmAttributes = USB_EP_TYPE_BULK,
		.wMaxPacketSize = sys_cpu_to_le16(64U),
		.bInterval = 0,
	},
	.nil_desc = {
		.bLength = 0,
		.bDescriptorType = 0,
	},
};

static const struct usb_desc_header *const sdr_fs_desc[] = {
	(struct usb_desc_header *)&sdr_desc.if0,
	(struct usb_desc_header *)&sdr_desc.in_ep,
	(struct usb_desc_header *)&sdr_desc.out_ep,
	(struct usb_desc_header *)&sdr_desc.nil_desc,
};

static struct usbd_class_data *sdr_c_data;
static atomic_t sdr_up;

bool usb_sdr_enabled(void)
{
	return atomic_get(&sdr_up) != 0;
}

static int submit(uint8_t ep, uint8_t *data, size_t len, bool in)
{
	struct net_buf *buf;
	struct udc_buf_info *bi;
	int err;

	if (!usb_sdr_enabled()) {
		return -ENOTCONN;
	}
	buf = net_buf_alloc_with_data(&sdr_ep_pool, data, len, K_NO_WAIT);
	if (buf == NULL) {
		return -ENOMEM;
	}
	if (!in) {
		/* Empty: the controller writes what the host sends. */
		buf->len = 0;
	}
	bi = udc_get_buf_info(buf);
	memset(bi, 0, sizeof(*bi));
	bi->ep = ep;
	err = usbd_ep_enqueue(sdr_c_data, buf);
	if (err != 0) {
		net_buf_unref(buf);
	}
	return err;
}

int usb_sdr_submit_in(uint8_t *data, size_t len)
{
	return submit(sdr_desc.in_ep.bEndpointAddress, data, len, true);
}

int usb_sdr_submit_out(uint8_t *data, size_t cap)
{
	return submit(sdr_desc.out_ep.bEndpointAddress, data, cap, false);
}

void usb_sdr_cancel(void)
{
	if (sdr_c_data == NULL) {
		return;
	}
	(void)usbd_ep_dequeue(usbd_class_get_ctx(sdr_c_data), sdr_desc.in_ep.bEndpointAddress);
	(void)usbd_ep_dequeue(usbd_class_get_ctx(sdr_c_data), sdr_desc.out_ep.bEndpointAddress);
}

static int sdr_request(struct usbd_class_data *const c_data, struct net_buf *buf, int err)
{
	struct udc_buf_info *bi = udc_get_buf_info(buf);
	uint8_t *data = buf->__buf;
	size_t len = buf->len;
	uint8_t ep = bi->ep;

	ARG_UNUSED(c_data);
	net_buf_unref(buf);
	if (ep == sdr_desc.in_ep.bEndpointAddress) {
		usb_sdr_in_done(data, len, err);
	} else if (ep == sdr_desc.out_ep.bEndpointAddress) {
		usb_sdr_out_done(data, len, err);
	}
	return 0;
}

static struct net_buf *sdr_control_to_host(struct usbd_class_data *c_data,
					   const struct usb_setup_packet *const setup)
{
	struct net_buf *buf;
	int len;

	if (setup->RequestType.recipient != USB_REQTYPE_RECIPIENT_DEVICE) {
		return NULL;
	}
	buf = usbd_ep_ctrl_data_in_alloc(usbd_class_get_ctx(c_data), setup->wLength);
	if (buf == NULL) {
		return NULL;
	}
	len = usb_sdr_control_in(setup->bRequest, setup->wValue, buf->data,
				 MIN(setup->wLength, net_buf_tailroom(buf)));
	if (len < 0) {
		net_buf_unref(buf);
		return NULL;
	}
	net_buf_add(buf, len);
	return buf;
}

static int sdr_control_to_dev(struct usbd_class_data *c_data,
			      const struct usb_setup_packet *const setup,
			      const struct net_buf *const buf)
{
	ARG_UNUSED(c_data);

	if (setup->RequestType.recipient != USB_REQTYPE_RECIPIENT_DEVICE) {
		return -ENOTSUP;
	}
	if (setup->wLength == 0U) {
		return usb_sdr_control_out(setup->bRequest, setup->wValue, NULL, 0);
	}
	if (buf == NULL) {
		/* Setup stage: accept the data stage, handled when it arrives. */
		return 0;
	}
	return usb_sdr_control_out(setup->bRequest, setup->wValue, buf->data, buf->len);
}

static const void *sdr_get_desc(struct usbd_class_data *const c_data, const enum usbd_speed speed)
{
	ARG_UNUSED(c_data);
	ARG_UNUSED(speed);
	return sdr_fs_desc;
}

static void sdr_enable(struct usbd_class_data *const c_data)
{
	sdr_c_data = c_data;
	atomic_set(&sdr_up, 1);
	LOG_INF("interface enabled (in 0x%02x, out 0x%02x)", sdr_desc.in_ep.bEndpointAddress,
		sdr_desc.out_ep.bEndpointAddress);
	usb_sdr_link(true);
}

static void sdr_disable(struct usbd_class_data *const c_data)
{
	ARG_UNUSED(c_data);
	atomic_set(&sdr_up, 0);
	LOG_INF("interface disabled");
	usb_sdr_link(false);
}

static int sdr_init(struct usbd_class_data *c_data)
{
	sdr_c_data = c_data;
	return 0;
}

static const struct usbd_class_api sdr_api = {
	.request = sdr_request,
	.control_to_host = sdr_control_to_host,
	.control_to_dev = sdr_control_to_dev,
	.get_desc = sdr_get_desc,
	.enable = sdr_enable,
	.disable = sdr_disable,
	.init = sdr_init,
};

USBD_DEFINE_CLASS(esp_sdr_usb, &sdr_api, NULL, &sdr_vregs);
