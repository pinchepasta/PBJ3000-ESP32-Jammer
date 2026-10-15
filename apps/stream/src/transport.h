/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef STREAM_TRANSPORT_H_
#define STREAM_TRANSPORT_H_

#include <stddef.h>

int transport_init(void);
/* Send one packet; returns 0 or a negative errno. */
int transport_send(const void *buf, size_t len);
void transport_report(void);

#endif /* STREAM_TRANSPORT_H_ */
