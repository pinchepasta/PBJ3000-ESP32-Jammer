# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(esp_sdr_osmosdr)

include(${CMAKE_CURRENT_LIST_DIR}/../common/common.cmake)

target_sources(app PRIVATE
  src/main.c
  src/sdr.c
  src/shell.c
  src/usb_sdr.c)
set_source_files_properties(src/sdr.c PROPERTIES COMPILE_OPTIONS "-O3;-ffp-contract=fast")
