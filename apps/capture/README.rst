.. _esp_sdr_capture:

capture
#######

Overview
********

Captures bursts of raw baseband I/Q samples from the ESP32-S3 or ESP32-C6 Wi-Fi receiver
with the ``esp-sdr`` module and prints, for each step of a fixed survey:

- tuning, sample rate, low-pass filter code and the measured capture rate,
- DC offset, mean power in dBFS, peak magnitude and clipped samples,
- a 64-column spectrum from -fs/2 to +fs/2 (Welch average of 256-point FFTs),
  scaled between its lowest and highest column in dBFS per bin.

The capture time includes arming and polling, a few microseconds on top of
the burst length.

The survey covers Wi-Fi channels 1, 6 and 11 at 80, 40 and 16 MS/s, a wide and
a narrow low-pass filter at 2412 MHz, and 1900 MHz through the 5/6 LO mode. It
repeats every 10 seconds.

Requirements
************

An ESP32-S3, ESP32-C6 or ESP32-C5 board; tested on ``xiao_esp32s3``,
``xiao_esp32c6/esp32c6/hpcore`` and ``waveshare_esp32c5_zero/esp32c5/hpcore``.
The C6 captures at 80 MS/s only: the survey skips the 40 and 16 MS/s steps and
clamps the filter bandwidths to its 12 to 54 MHz range. The C5 skips the
16 MS/s step and clamps to 11 to 48 MHz; it has no divided LO, so its 1900 MHz
step tunes the PLL directly. Attach an antenna to see more
than the receiver noise floor.

Building and running
********************

.. code-block:: console

   west build -b xiao_esp32s3/esp32s3/procpu esp-sdr-zephyr/apps/capture
   west flash

Sample output
*************

From a XIAO ESP32S3 without antenna: receiver noise shaped by the analog
low-pass filter (compare ``lpf 0`` and ``lpf 60``).

.. code-block:: console

   esp-sdr rx: ready
   esp-sdr rx: 2412 MHz, 80 MS/s, lpf -1, 4096 samples in 60 us
     dc +3.7/+20.1, power -26.1 dBFS, peak 85, clipped 0
     |  ... .          ...:-+#%%%@@@@@@@@@@@@%%#=::....         .   ..| -77..-42 dBFS/bin
   esp-sdr rx: 2437 MHz, 40 MS/s, lpf -1, 4096 samples in 102 us
     dc +0.3/-3.5, power -26.5 dBFS, peak 81, clipped 0
     |      ....::--*+++*****++**@+*****+*++++*++**+*+++-::.... . .   | -76..-32 dBFS/bin
   esp-sdr rx: 2462 MHz, 16 MS/s, lpf -1, 4096 samples in 256 us
     dc -2.5/+8.0, power -26.1 dBFS, peak 70, clipped 0
     |+****+=+=*=:=-:-:-=--=-+# :++=+%@+==:--=:--:---::---=+=*++*+****| -53..-45 dBFS/bin
   esp-sdr rx: 2412 MHz, 80 MS/s, lpf 0, 4096 samples in 51 us
     dc +0.6/+21.6, power -20.6 dBFS, peak 162, clipped 0
     | ..= ::.:@::.:::-::.:::-:-:::---=::-::-::#:.:.::-.:::::..... .  | -48..-32 dBFS/bin
   esp-sdr rx: 2412 MHz, 80 MS/s, lpf 60, 4096 samples in 51 us
     dc +4.1/+25.1, power -27.7 dBFS, peak 67, clipped 0
     |     ..  ..     . ..  ..:+#%%%%%@%%%%%*:..... . .   .   ..   .. | -77..-41 dBFS/bin
   esp-sdr rx: 1900 MHz, 80 MS/s, lpf -1, 4096 samples in 54 us
     dc +252.3/-128.1, power -34.8 dBFS, peak 276, clipped 0
     |.... . .....  .....:.:+#%@%@@%%@@@@@%%%%#=-:... ....  .... .....| -77..-51 dBFS/bin
   esp-sdr rx: survey done, 0 failed
