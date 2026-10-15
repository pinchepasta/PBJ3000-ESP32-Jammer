# SPDX-License-Identifier: GPL-3.0-or-later
#
# Unit test of apps/link/src/hamming.c (extended Hamming(128,120)) on
# native_sim. Plain portable C, no Zephyr dependency, unmodified.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(link_hamming_unit)

target_sources(app PRIVATE
  src/main.c
  ${CMAKE_CURRENT_LIST_DIR}/../../../apps/link/src/hamming.c)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../../apps/link/src)
