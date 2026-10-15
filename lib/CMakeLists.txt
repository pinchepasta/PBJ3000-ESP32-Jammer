# SPDX-License-Identifier: GPL-3.0-or-later

add_subdirectory_ifdef(CONFIG_ESP_SDR esp_sdr)
add_subdirectory_ifdef(CONFIG_ESP_SDR_IEEE802154 ieee802154)
