# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(esp_stream)

include(${CMAKE_CURRENT_LIST_DIR}/../common/common.cmake)

target_sources(app PRIVATE
  src/main.c
  src/vrt_rx.c
  src/iq_pack.c)
target_sources_ifdef(CONFIG_SHELL app PRIVATE src/shell.c)
target_sources_ifdef(CONFIG_APP_TX app PRIVATE src/vrt_tx.c)

if(CONFIG_USBD_CDC_NCM_CLASS)
  target_sources(app PRIVATE src/transport_udp.c)
else()
  target_sources(app PRIVATE src/transport_null.c)
endif()
# Per-sample loops on the stream's critical path.
set_source_files_properties(src/iq_pack.c PROPERTIES COMPILE_OPTIONS -O3)
