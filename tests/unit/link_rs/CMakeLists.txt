# SPDX-License-Identifier: GPL-3.0-or-later
#
# Unit test of apps/link/src/rs.c (RS(255,223)) on native_sim. rs.c takes
# the __ZEPHYR__ branch (IRAM_ATTR from hal_espressif's esp_attr.h, not
# available here) when built inside any Zephyr app, native_sim included:
# undefine it for this one file, same as the host simulators
# (apps/link/tools/qam_sim.c, ofdm_sim.c) already do by building outside
# Zephyr entirely. The source is otherwise unmodified.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(link_rs_unit)

set(RS_SRC ${CMAKE_CURRENT_LIST_DIR}/../../../apps/link/src/rs.c)

target_sources(app PRIVATE src/main.c ${RS_SRC})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../../apps/link/src)
set_source_files_properties(${RS_SRC} PROPERTIES COMPILE_OPTIONS "-U__ZEPHYR__")
