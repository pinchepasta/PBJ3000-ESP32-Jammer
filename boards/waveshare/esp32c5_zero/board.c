/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Antenna select: the RF switch takes the U.FL connector while GPIO26 is high.
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

void board_late_init_hook(void)
{
	const struct gpio_dt_spec ant_sel = GPIO_DT_SPEC_GET(DT_NODELABEL(rf_switch), select_gpios);

	if (gpio_is_ready_dt(&ant_sel)) {
		gpio_pin_configure_dt(&ant_sel, GPIO_OUTPUT_ACTIVE);
	}
}
