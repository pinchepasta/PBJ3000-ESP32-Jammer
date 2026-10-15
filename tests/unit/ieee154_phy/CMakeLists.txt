# SPDX-License-Identifier: GPL-3.0-or-later
#
# Unit test of lib/ieee802154/ieee154_phy.c on native_sim: the module has no
# Zephyr dependencies, so it is compiled directly as an extra source instead
# of through the esp_sdr_ieee802154 Kconfig library (which needs a real
# radio). See doc/testing.md.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ieee154_phy_unit)

set(PHY_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../lib/ieee802154)

target_sources(app PRIVATE
  src/main.c
  ${PHY_DIR}/ieee154_phy.c)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../../include)
