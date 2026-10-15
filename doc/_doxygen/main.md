# esp-sdr-zephyr API

API reference of the esp_sdr library: raw I/Q receive and transmit on the
ESP32-S3 Wi-Fi radio (burst capture on the ESP32-C6), and the IEEE 802.15.4
software PHY and radio driver built on it. See the
[topics](topics.html) for the parts:

- @ref esp_sdr : @ref esp_sdr_radio, @ref esp_sdr_rx, @ref esp_sdr_tx and
  @ref esp_sdr_ring.
- @ref ieee154 : @ref ieee154_phy and @ref ieee154_esp_sdr.

Declarations that depend on the SoC or on Kconfig are all listed here; each
says what it needs. The guides (hardware quirks, the 802.15.4 design,
testing) are in the project documentation built from `doc/` with Sphinx.

Source: [esp-sdr-zephyr](https://github.com/cyber-murmel/esp-sdr-zephyr).
