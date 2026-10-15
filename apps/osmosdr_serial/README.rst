.. _esp_sdr_osmosdr_serial:

osmosdr over a serial link
##########################

Overview
********

The :ref:`osmosdr <esp_sdr_osmosdr>` protocol (``../osmosdr/src/esdr_proto.h``) for the
ESP32-C6 and the ESP32-C5, whose only USB port is USB-Serial-JTAG: the vendor
requests travel as frames on the byte stream, receive blocks and transmit samples in between
(the serial transport in ``esdr_proto.h``). The app drives the port's FIFOs itself; the console
is on UART0 (D6 TX, D7 RX on the XIAO). One direction at a time, as on the S3.

- **Receive**: continuous I/Q at 250, 125, 62.5, 31.25 or 15.625 kS/s, 8 or 16 bit. The modem's
  IQ estimator sums the 80 MHz baseband samples in hardware; the CPU reads the running sums at
  the sample rate and sends their differences (``esp_sdr_rx_paced_*()``): each sample is the
  sum of 80 MHz / rate inputs, a boxcar decimator. No offset tuning: the LO leakage stays at
  the centre.
- **Transmit**: the host's samples go to the TX DC value register, which drives the DAC once
  the vendor tone path has turned the transmit chain on (``esp_sdr_tx_paced_*()``), through an
  8-tap-per-phase FIR; 250 kS/s goes to the DAC as it is. The DAC runs at up to 250 kS/s on
  the C6 (125 kS/s x 2) and 500 kS/s on the C5 (125 kS/s x 4). C6 2400 to 2500 MHz, C5 2400 to
  5900 MHz (whole MHz); power steps 0 to 15 (2 dB each).
- The CPU paces both with interrupts locked and serves USB between samples, never so late
  that a sample instant moves.

USB-Serial-JTAG carries about 500 kB/s each way: 250 kS/s receive fits as 8-bit items only,
16 bit up to 125 kS/s. Transmit at 250 kS/s keeps up on the C5; on the C6 about 10 % of the
input arrives late and plays as zeros, so 125 kS/s is its reliable rate.

Building and flashing
*********************

.. code-block:: console

   west build -b xiao_esp32c6/esp32c6/hpcore esp-sdr-zephyr/apps/osmosdr_serial
   west build -b waveshare_esp32c5_zero/esp32c5/hpcore esp-sdr-zephyr/apps/osmosdr_serial
   west flash --esp-device /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_<mac>-if00

The board configs select the U.FL connector. The C5 has the same IQ estimator (0x24 lower) and
its TX DC value register at 0x600a0c08, where bit 26 enables the injection. Opening the port does not reset the board as long as the host leaves
DTR and RTS alone (the host tool and the SoapySDR module open it raw); tools that toggle them
(esptool, terminal programs) reset it.

Host tool
*********

``tools/esdr_serial.py`` speaks the transport from Python:

.. code-block:: console

   tools/esdr_serial.py <port> info
   tools/esdr_serial.py <port> rx --rate 250000 --seconds 4 --out rx.npy
   tools/esdr_serial.py <port> tx --rate 125000 --seconds 4 --tone 30000

SoapySDR, GNU Radio, gqrx
*************************

The osmosdr app's SoapySDR module (``../osmosdr/soapy``) has a serial backend: name the port in
the device string. It does not probe serial ports by itself (that would write into other
boards' consoles). Over the serial link the native format is CS8, and CF32 travels as 8-bit
items.

.. code-block:: console

   SoapySDRUtil --probe="driver=esdr,port=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_<mac>-if00"
   gqrx        # device string: soapy=0,driver=esdr,port=/dev/serial/by-id/...

Statistics
**********

``GET_STATS`` fills the osmosdr fields; the pacing reports in the S3 ring's: ``ring_runs``
counts streams, ``ring_errors`` samples taken a whole period late, ``ring_late_max`` the worst
lateness in CPU cycles.
