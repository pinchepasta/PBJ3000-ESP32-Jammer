# SPDX-License-Identifier: GPL-3.0-or-later
#
# Hardware integration test of the esp_sdr receive API's contract (error
# codes, boundary values) on a real board. See doc/testing.md.
#
# apps/common is only needed by the S3 variant (USB shell and DFU, see
# Kconfig); same if(CONFIG_USB_DEVICE_STACK_NEXT) guard every app uses.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(esp_sdr_rx_api)

if(CONFIG_USB_DEVICE_STACK_NEXT)
  include(${CMAKE_CURRENT_LIST_DIR}/../../../apps/common/common.cmake)
endif()

target_sources(app PRIVATE src/main.c)
