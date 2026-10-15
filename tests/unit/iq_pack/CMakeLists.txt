# SPDX-License-Identifier: GPL-3.0-or-later
#
# Unit test of apps/stream/src/iq_pack.c (8/12/16-bit sample packing)
# on native_sim. iq_pack.h pulls in vrt_rx.h for struct vrt_rx_burst, whose
# array is sized by the stream app's own CONFIG_APP_RX_BURST_SAMPLES;
# this test never uses that struct, so the symbol is stood in directly
# instead of pulling in that app's whole Kconfig tree.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(iq_pack_unit)

target_sources(app PRIVATE
  src/main.c
  ${CMAKE_CURRENT_LIST_DIR}/../../../apps/stream/src/iq_pack.c)
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/../../../apps/stream/src
  ${CMAKE_CURRENT_LIST_DIR}/../../../include)
target_compile_definitions(app PRIVATE CONFIG_APP_RX_BURST_SAMPLES=16)
