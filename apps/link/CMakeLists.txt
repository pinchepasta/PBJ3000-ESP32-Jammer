# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(esp_sdr_link)

include(${CMAKE_CURRENT_LIST_DIR}/../common/common.cmake)

target_sources(app PRIVATE
  src/main.c
  src/mac.c
  src/perf.c
  src/qam.c
  src/rs.c
  src/hamming.c
  src/ofdm.c
  src/ofdm_frame.c
  src/ofdm_test.c)
target_compile_definitions(app PRIVATE QAM_SPS=${CONFIG_APP_SPS})
# Per-sample and per-byte loops of the modem and the codec. Zephyr's ISO C
# mode turns contraction off: fused multiply-adds halve the FPU work.
set_source_files_properties(src/qam.c src/ofdm.c PROPERTIES COMPILE_OPTIONS "-O3;-ffp-contract=fast")
set_source_files_properties(src/rs.c src/hamming.c PROPERTIES COMPILE_OPTIONS -O3)

# Only the ESP32-S3 float FFT kernel of esp-dsp: its IDF scaffolding stays out.
if(CONFIG_APP_OFDM_ESP_DSP)
  set(ESP_DSP_DIR "${WEST_TOPDIR}/modules/lib/esp-dsp")
  set(ESP_DSP_FFT "${ESP_DSP_DIR}/modules/fft/float/dsps_fft2r_fc32_aes3_.S")
  if(NOT EXISTS "${ESP_DSP_FFT}")
    message(FATAL_ERROR "esp-dsp not found at ${ESP_DSP_DIR} (west project esp-dsp)")
  endif()
  target_sources(app PRIVATE "${ESP_DSP_FFT}")
  # Its platform header enables the S3 kernel by the IDF target macro.
  set_source_files_properties("${ESP_DSP_FFT}" PROPERTIES
    COMPILE_DEFINITIONS CONFIG_IDF_TARGET_ESP32S3=1
    INCLUDE_DIRECTORIES "${ESP_DSP_DIR}/modules/fft/include")
endif()
