/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * VRT packets as UDP datagrams over IPv6 on the USB (CDC-NCM) interface.
 * The default peer is the link-local all-nodes group, so the host needs no
 * address setup; a unicast peer works as well.
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>

#include "transport.h"

LOG_MODULE_DECLARE(stream, LOG_LEVEL_INF);

static int sock = -1;
static struct net_if *usb_if;
static struct sockaddr_in6 peer;
static struct {
	uint32_t sent, down, errors;
	int last_errno;
} stats;

int transport_init(void)
{
	const struct device *ncm = DEVICE_DT_GET(DT_NODELABEL(cdc_ncm_eth0));

	usb_if = net_if_lookup_by_dev(ncm);
	if (usb_if == NULL) {
		LOG_ERR("udp: no USB network interface");
		return -ENODEV;
	}

	peer.sin6_family = AF_INET6;
	peer.sin6_port = htons(CONFIG_APP_RX_UDP_PORT);
	if (zsock_inet_pton(AF_INET6, CONFIG_APP_RX_UDP_PEER, &peer.sin6_addr) != 1) {
		LOG_ERR("udp: bad peer address %s", CONFIG_APP_RX_UDP_PEER);
		return -EINVAL;
	}
	/* Link-local and multicast peers need the interface as scope. */
	peer.sin6_scope_id = (uint32_t)net_if_get_by_iface(usb_if);

	sock = zsock_socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("udp: socket failed (%d)", errno);
		return -errno;
	}
	LOG_INF("udp: sending to [%s]:%d on interface %d", CONFIG_APP_RX_UDP_PEER,
		CONFIG_APP_RX_UDP_PORT, peer.sin6_scope_id);
	return 0;
}

int transport_send(const void *buf, size_t len)
{
	/* Admin up is set at boot; carrier means a host configured the link. */
	if (!net_if_is_carrier_ok(usb_if)) {
		/* No host yet: drop instead of queueing stale samples. */
		stats.down++;
		return 0;
	}
	if (zsock_sendto(sock, buf, len, 0, (struct sockaddr *)&peer, sizeof(peer)) < 0) {
		stats.errors++;
		stats.last_errno = errno;
		return -errno;
	}
	stats.sent++;
	return 0;
}

void transport_report(void)
{
	LOG_DBG("udp: %u sent, %u while down, %u errors (last %d)", stats.sent, stats.down,
		stats.errors, stats.last_errno);
	stats.sent = 0;
	stats.down = 0;
	stats.errors = 0;
}
