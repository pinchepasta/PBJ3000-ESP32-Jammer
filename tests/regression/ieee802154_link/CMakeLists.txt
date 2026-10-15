# SPDX-License-Identifier: GPL-3.0-or-later
#
# Two-board hardware regression test (pytest/test_interop.py, see
# doc/testing.md). The DUT image is the apps/wpan application, unmodified:
# this test's whole job is to regress what that application actually ships,
# not a rebuilt stand-in. src/main.c is built from there by path, and
# APPLICATION_CONFIG_DIR points Zephyr's own board-aware config lookup at
# that same directory, so prj.conf and the per-board boards/*.conf and
# *.overlay come from there too, unmodified. Only sysbuild.conf is a plain
# copy here: its own lookup (done by the outer sysbuild pass, before this
# file runs) is rooted at this directory, not APPLICATION_CONFIG_DIR, and
# the file is one static line anyway.

cmake_minimum_required(VERSION 3.20.0)

set(WPAN_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../apps/wpan)
set(APPLICATION_CONFIG_DIR ${WPAN_DIR})
set(KCONFIG_ROOT ${WPAN_DIR}/Kconfig)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ieee802154_link_regression)

# The ESP32-S3 software radio: USB shell and DFU on the OTG port.
if(CONFIG_USB_DEVICE_STACK_NEXT)
  include(${WPAN_DIR}/../common/common.cmake)
endif()

target_sources(app PRIVATE ${WPAN_DIR}/src/main.c)
