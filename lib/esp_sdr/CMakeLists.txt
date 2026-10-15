# SPDX-License-Identifier: GPL-3.0-or-later

# Upstream ESPARGOS esp-sdr (west project "esp-sdr", see west.yml): only its
# header-only helpers (tuning, LO plan, bandwidth curve) are used.
set(ESP_SDR_UPSTREAM_DIR "${WEST_TOPDIR}/modules/lib/esp-sdr" CACHE PATH
    "Checkout of github.com/ESPARGOS/esp-sdr")
if(NOT EXISTS ${ESP_SDR_UPSTREAM_DIR}/main/common/rx_tuning.h)
  message(FATAL_ERROR "esp-sdr: upstream checkout not found at ${ESP_SDR_UPSTREAM_DIR} "
                      "(west update, or set ESP_SDR_UPSTREAM_DIR)")
endif()

zephyr_library()
zephyr_library_sources(
  src/esp_sdr.c
  src/esp_sdr_rx.c
)
zephyr_library_sources_ifdef(CONFIG_SOC_SERIES_ESP32S3 src/esp_sdr_tx.c)
zephyr_library_sources_ifdef(CONFIG_ESP_SDR_PACED src/esp_sdr_paced.c)
zephyr_library_sources_ifdef(CONFIG_ESP_SDR_TX src/esp_sdr_tx_stream.c)
zephyr_library_sources_ifdef(CONFIG_ESP_SDR_TX_DAC src/esp_sdr_tx_dac.c)
zephyr_library_sources_ifdef(CONFIG_ESP_SDR_TX_DAC_SIMD src/esp_sdr_pie.S)
zephyr_library_sources_ifdef(CONFIG_ESP_SDR_RING src/esp_sdr_ring.c src/esp_sdr_ring_fir.S
  src/esp_sdr_ring_dot.S)
zephyr_library_include_directories(${ESP_SDR_UPSTREAM_DIR}/main/common)

# Three banks for the continuous ring; two: loop playback ping-pongs between
# them, captures may use either.
if(CONFIG_SOC_SERIES_ESP32C6)
  zephyr_linker_sources(SECTIONS esp_sdr_bank_c6.ld)
elseif(CONFIG_SOC_SERIES_ESP32C5)
  # Among the RAM sections, so that .noinit and .bss follow the bank.
  zephyr_linker_sources(RAM_SECTIONS esp_sdr_bank_c5.ld)
elseif(CONFIG_ESP_SDR_RING)
  zephyr_linker_sources(SECTIONS esp_sdr_bank_ring.ld)
elseif(CONFIG_ESP_SDR_BANK1)
  zephyr_linker_sources(SECTIONS esp_sdr_bank_tx.ld)
else()
  zephyr_linker_sources(SECTIONS esp_sdr_bank.ld)
endif()

if(CONFIG_ESP_SDR_RFTEST)
  # Espressif RF test library, fetched with "west blobs fetch esp-sdr-zephyr".
  set(esp_sdr_rftest
      ${ZEPHYR_CURRENT_MODULE_DIR}/zephyr/blobs/lib/${CONFIG_SOC_SERIES}/librftest.a)
  zephyr_blobs_verify(FILES ${esp_sdr_rftest} REQUIRED)
  zephyr_link_libraries(${esp_sdr_rftest})
endif()
