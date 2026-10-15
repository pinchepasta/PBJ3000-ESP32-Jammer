# SPDX-License-Identifier: GPL-3.0-or-later
#
# Firmware of the two-board LO tuning regression test (pytest/test_lo_tuning.py,
# see doc/testing.md): the ESP32-S3 transmits tones, the ESP32-C6 captures them.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lo_tuning_regression)

# ESP32-S3: USB shell and DFU on the OTG port, as the apps.
if(CONFIG_USB_DEVICE_STACK_NEXT)
  include(${CMAKE_CURRENT_LIST_DIR}/../../../apps/common/common.cmake)
endif()

target_sources(app PRIVATE src/main.c)
