/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Packet sink without a network, for measuring the encoder alone.
 */

#include "transport.h"

int transport_init(void)
{
	return 0;
}

int transport_send(const void *buf, size_t len)
{
	(void)buf;
	(void)len;
	return 0;
}

void transport_report(void)
{
}
