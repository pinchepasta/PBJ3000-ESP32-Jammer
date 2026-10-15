/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef APP_CPU_H_
#define APP_CPU_H_

/*
 * Pin the cooperative and high priority system threads (network queues,
 * net_mgmt, USB, system work queue, Wi-Fi events) to @p cpu, so they never
 * delay time critical radio threads on the other one. Lower priority threads
 * (shell, logging) stay free. Call before USB starts (its controller
 * interrupt lands on the CPU that allocates it) and again after. No-op
 * without CONFIG_SCHED_CPU_MASK or on one CPU.
 */
void app_cpu_pin_system_threads(int cpu);

#endif /* APP_CPU_H_ */
