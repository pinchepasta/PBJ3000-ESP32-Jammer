PBJ3000 - ESP-Jammer
=================

<img src="https://github.com/pinchepasta/PBJ3000-ESP32-Jammer/blob/main/pbjlogo.png" alt="Logo" width="20%" height="20%">

It's a Jammer! And NO, you don't need any additional hardware, an esp32s3 board is enough.

And the longer version is:
It is a software defined radio jammer based on the ESP32-S3 Wi-Fi radio for Zephyr: I/Q
capture from the receiver, I/Q playback through the transmit DAC. Ported
from `ESPARGOS esp-sdr <https://github.com/ESPARGOS/esp-sdr>`__ and
extended; a west manifest repository and Zephyr module laid out like
`example-application <https://github.com/zephyrproject-rtos/example-application>`__.

<br>

How do I get it to work?
--------

Go to <a href="https://github.com/pinchepasta/PBJ3000-ESP32-Jammer/releases">Releases</a> and download PBJ3000.bin and PBJ3000.apk.
Install PBJ3000.bin on your esp32s3 board, just upload image via terminal or with your tool of choice.
Now download and install PBJ3000.apk on your android smartphone, connect the ESP32S§ with an usb-c cable - start the app, click connect, and you're good to go.

<br>

What does it look like?
--------

<img src="https://github.com/pinchepasta/PBJ3000-ESP32-Jammer/blob/main/pbj1.jpg" alt="Logo" width="20%" height="20%"> <img src="https://github.com/pinchepasta/PBJ3000-ESP32-Jammer/blob/main/pbj2.jpg" alt="Logo" width="20%" height="20%"> <img src="https://github.com/pinchepasta/PBJ3000-ESP32-Jammer/blob/main/pbj3.jpg" alt="Logo" width="20%" height="20%"> <img src="https://github.com/pinchepasta/PBJ3000-ESP32-Jammer/blob/main/pbj4.jpg" alt="Logo" width="20%" height="20%">

<br>

Show it in action!
--------

<img src="https://github.com/pinchepasta/PBJ3000-ESP32-Jammer/blob/main/jm.jpg" alt="Logo" width="30%" height="30%">

<img src="https://github.com/pinchepasta/PBJ3000-ESP32-Jammer/blob/main/pbj6.jpg" alt="Logo" width="40%" height="40%"><img src="https://github.com/pinchepasta/PBJ3000-ESP32-Jammer/blob/main/pbj5.jpg" alt="Logo" width="40%" height="40%"> 

I have added 4 magents to mine, to make them hotswappable with my mostly-a-Flipper headless modules based on the exact same board but with an added cc1101.
You might wanna do a similar thing, or find another creative way to attach them.

<br>

Contents
--------


- **Chips:** ESP32-S3 (XIAO ESP32S3), SMP or single core. ESP32-C6
  (XIAO ESP32C6) and ESP32-C5 (C5-Zero): burst captures, and continuous
  CPU-paced receive and transmit up to 250 kS/s (``esp_sdr_paced.h``);
  ``capture``, ``rfprobe``, ``wpan`` (C6) and ``osmosdr_serial`` run on
  them.
- **Tuning:** 100 to 6000 MHz in the API, but the PLL locks over a
  chip-specific range (about 1.8 to 2.8 GHz on the S3); below 2210 MHz
  through the 5/6 divider. ``-ERANGE`` when the VCO is out of range.
- **Capture:** 256 to 16380 samples at 80, 40 or 16 MS/s, analog
  low-pass filter, optional CIC decimation; gapless ring receive.
- **Gain:** AGC or a fixed index; the fixed gain and TX power steps use
  ``librftest.a`` (``west blobs fetch esp-sdr-zephyr``, or
  ``CONFIG_ESP_SDR_RFTEST=n``).
- **Transmit:** sample playback at 40 or 80 MS/s, gapless streaming at
  40 MS/s / n (``CONFIG_ESP_SDR_TX_DAC``, needs PSRAM), 18 power steps.

Chip details and quirks: `doc/hardware-quirks.md <doc/hardware-quirks.md>`__.
Enable it with:

::

   CONFIG_NETWORKING=y
   CONFIG_WIFI=y
   CONFIG_ESP32_WIFI_STA_POWER_SAVE_NONE=y
   CONFIG_ESP_SDR=y

``esp_sdr_init()`` takes over the radio the Wi-Fi driver brought up; do
not scan, connect or start an access point while using it.

Dependencies
------------

``west.yml`` pins:

- `Zephyr esp-sdr-multiapp <https://github.com/cyber-murmel/zephyr/tree/esp-sdr-multiapp>`__:
  ESP32-S3 SMP (zephyrproject-rtos/zephyr#120082), CDC-NCM aggregation,
  llext and Xtensa changes for multiapp
- `hal_espressif esp-sdr <https://github.com/cyber-murmel/hal_espressif/tree/esp-sdr>`__:
  the HAL side of the SMP port
- `libvrt esp-sdr <https://github.com/cyber-murmel/libvrt/tree/esp-sdr>`__:
  VITA 49.2
- `esp-sdr <https://github.com/ESPARGOS/esp-sdr>`__ (upstream,
  unmodified): receiver helper headers
- `esp-dsp <https://github.com/espressif/esp-dsp>`__: the FFT for
  ``apps/link`` OFDM



Important info:
-------

A radio frequency jamming device like this one here is pretty much illegal in most places, so please make sure you know your local laws before youb push that fancy transmission button!
Also: The device creates quite drastic harmonics, please be aware of this and use a bandpass filter or use the device in a shielded lab environment only.


License
-------

GPL-3.0-or-later, like upstream esp-sdr; images that enable the library
are subject to the GPL.
