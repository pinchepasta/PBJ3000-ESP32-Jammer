# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(esp_sdr_osmosdr_serial)

# The protocol header is the osmosdr app's.
target_include_directories(app PRIVATE ../osmosdr/src)
target_sources(app PRIVATE src/main.c)
