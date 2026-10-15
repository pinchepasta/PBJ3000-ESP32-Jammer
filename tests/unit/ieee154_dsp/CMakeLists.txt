# SPDX-License-Identifier: GPL-3.0-or-later
#
# Unit test of lib/ieee802154/ieee154_dsp.h (the 802.15.4 driver's signal
# processing: DAC waveform generation and the receive mix/decimate) on
# native_sim. Header-only, but its inline functions call into
# ieee154_phy.c (ieee154_phy_chips()), so that is compiled in too.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ieee154_dsp_unit)

set(IEEE154_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../lib/ieee802154)

target_sources(app PRIVATE
  src/main.c
  ${IEEE154_DIR}/ieee154_phy.c)
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/../../../include
  ${IEEE154_DIR})
