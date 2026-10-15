# SPDX-License-Identifier: GPL-3.0-or-later
#
# SoapySDR module for the osmosdr app ("driver=esdr").

cmake_minimum_required(VERSION 3.16)
project(SoapyEsdr CXX)

find_package(SoapySDR CONFIG REQUIRED)
find_path(LIBUSB_INCLUDE_DIR libusb.h PATH_SUFFIXES libusb-1.0 REQUIRED)
find_library(LIBUSB_LIBRARY NAMES usb-1.0 REQUIRED)

SOAPY_SDR_MODULE_UTIL(
  TARGET EsdrSupport
  SOURCES SoapyEsdr.cpp
  LIBRARIES ${LIBUSB_LIBRARY})
target_include_directories(EsdrSupport PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src
                                               ${LIBUSB_INCLUDE_DIR})
target_compile_features(EsdrSupport PRIVATE cxx_std_17)
