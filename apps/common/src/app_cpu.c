/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Measured with the DAC streaming backend: sysworkq, net_mgmt and the like
 * on the radio CPU caused 2 to 9 ms stalls. Lower priority threads (shell,
 * logging) stay free, so a busy CPU cannot starve them.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "app_cpu.h"

LOG_MODULE_REGISTER(app_cpu, LOG_LEVEL_INF);

#if defined(CONFIG_SCHED_CPU_MASK)
static const char *const system_thread_prefixes[] = {
	"tx_q[", "rx_q[", "usbd", "usb_otg", "net_mgmt", "sysworkq", "esp32_wifi_event",
};

struct system_threads {
	k_tid_t tid[16];
	size_t n;
};

static void find_system_thread(const struct k_thread *thread, void *user_data)
{
	struct system_threads *found = user_data;
	const char *name = k_thread_name_get((k_tid_t)thread);

	if (name == NULL || found->n == ARRAY_SIZE(found->tid)) {
		return;
	}
	ARRAY_FOR_EACH(system_thread_prefixes, i) {
		if (strncmp(name, system_thread_prefixes[i],
			    strlen(system_thread_prefixes[i])) == 0) {
			found->tid[found->n++] = (k_tid_t)thread;
			return;
		}
	}
}
#endif

/* Pinning only takes while a thread is not runnable: retry until it waits. */
void app_cpu_pin_system_threads(int cpu)
{
#if defined(CONFIG_SCHED_CPU_MASK)
	struct system_threads found = {0};
	unsigned int pinned = 0;

	if (cpu < 0 || arch_num_cpus() < 2) {
		return;
	}
	k_thread_foreach(find_system_thread, &found);
	for (size_t t = 0; t < found.n; t++) {
		for (int tries = 0; tries < 100; tries++) {
			if (k_thread_cpu_pin(found.tid[t], cpu) == 0) {
				pinned++;
				break;
			}
			k_sleep(K_MSEC(1));
		}
	}
	LOG_INF("%u of %u system threads pinned to cpu %d", pinned, (unsigned int)found.n, cpu);
#else
	ARG_UNUSED(cpu);
#endif
}
