# SPDX-License-Identifier: GPL-3.0-or-later

if(CONFIG_WAVESHARE_ESP32C5_ZERO_EXT_ANTENNA)
  zephyr_library()
  zephyr_library_sources(board.c)
endif()
