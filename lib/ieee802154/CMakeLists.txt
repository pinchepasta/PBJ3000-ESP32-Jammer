# SPDX-License-Identifier: GPL-3.0-or-later

zephyr_library()
zephyr_library_sources(
  ieee154_phy.c
  ieee154_esp_sdr.c
)
# Runs at 4 MS/s on one core.
set_source_files_properties(ieee154_phy.c PROPERTIES COMPILE_OPTIONS -O3)
