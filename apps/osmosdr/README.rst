.. _esp_sdr_osmosdr:

osmosdr
#######

Overview
********

A USB SDR after HackRF's model, for osmosdr and GNU Radio hosts: vendor
control requests set the radio, bulk endpoints carry the samples, one
direction at a time. The protocol is in ``src/esdr_proto.h``, shared with
the host tool ``tools/esdr_usb.c``.

- **Receive**: gapless I/Q at 250, 125, 62.5, 31.25 or 15.625 kS/s, 8 or 16
  bit. The dump engine runs in circular mode through three SRAM banks at
  16 MS/s; a two-stage decimating FIR on the vector unit filters every
  bank (``esp_sdr_ring_run()``, ported from upstream esp-sdr). It owns CPU 1
  with that CPU's interrupts masked; USB, Wi-Fi and the shell run on CPU 0.
- **Offset tuning** (default): the LO sits 4 MHz below the centre and the
  ring mixes the stream down, so the LO leakage and the 1/f noise stay out
  of the band. The LO is whole MHz plus a kHz PLL offset; a rotation of the
  output removes the rest, so the centre is exact to the Hz, after the
  crystal correction.
- **Transmit**: the host's samples go to the DAC backend (PSRAM ring, linear
  interpolation to 40 MS/s, gapless loop playback). USB flow control paces
  the host about 50 ms ahead of the air.
- USB: the SDR's vendor interface, CDC-ACM shell, DFU (MCUboot).

Building and flashing
*********************

.. code-block:: console

   west build --sysbuild -b xiao_esp32s3/esp32s3/procpu esp-sdr-zephyr/apps/osmosdr
   west flash

The app uses the same USB IDs as the other apps (2fe3:0005, host udev rules
as in ``apps/stream/README.rst``); hosts find it by its vendor
interface and the ``GET_INFO`` magic.

The three ring banks leave little internal RAM: sample blocks, filter
buffers, the transmit backend's buffers and the thread stacks live in
``ESP_SDR_HIGH_RAM`` (``doc/hardware-quirks.md``, Memory).

The first flash goes over USB-Serial-JTAG. From then on the app owns the USB
port, so RTS/DTR can no longer reset the chip into the ROM bootloader. Update
over DFU instead:

.. code-block:: console

   west dfu -s <usb serial>

``west dfu`` (``scripts/west_commands/dfu.py``, build directory ``build`` unless
``-d`` names another, ``-s`` needed with several boards) detaches the app into
DFU, downloads the image, waits for MCUboot to test-boot it, checks that its
shell keeps answering and only then confirms it (``mcuboot confirm``). An image
that is never confirmed is reset by a watchdog after about 30 s and MCUboot
reverts to the previous one, so a broken update never needs a replug.
``CONFIG_APP_USB_START_DELAY_MS`` keeps USB-Serial-JTAG alive for a few
seconds after reset for recovery builds.

Protocol
********

Vendor requests to the device (``bmRequestType`` 0x40 / 0xc0), all values
little endian:

.. list-table::
   :header-rows: 1

   * - Request
     - Value / data
   * - ``GET_INFO`` (0x01, in)
     - ``struct esdr_info``: firmware, tuning range, rates, gain ranges
   * - ``SET_MODE`` (0x02)
     - off 0, receive 1, transmit 2; throughput tests 0x10 (in), 0x11 (out)
   * - ``SET_FREQ`` (0x03)
     - centre in Hz, 8 bytes
   * - ``SET_SAMPLE_RATE`` (0x04)
     - Hz, 4 bytes; the nearest supported rate is taken
   * - ``SET_RX_GAIN`` (0x05)
     - PHY gain index 0 to 82 (not dB), or 0xffff for the hardware AGC
   * - ``SET_TX_GAIN`` (0x06)
     - power step 0 (weakest) to 17
   * - ``SET_BANDWIDTH`` (0x07)
     - analog low-pass in Hz (13 to 69 MHz two-sided), 0 for the PHY's
   * - ``SET_FORMAT`` (0x08)
     - 8 or 16 bits per I and Q item
   * - ``SET_FREQ_CORR`` (0x09)
     - crystal correction in ppb, 4 bytes signed
   * - ``GET_STATE`` (0x0a, in) / ``GET_STATS`` (0x0b, in)
     - settings as applied (``struct esdr_state``), counters
   * - ``SET_OPTIONS`` (0x0c)
     - bit 0: offset tuning (default on)
   * - ``SET_DIGITAL_GAIN`` (0x0d)
     - dB in steps of 6 up to 42, or 0xffff: 24 dB at 8 bit, 0 at 16

Receive data comes in 4096-byte blocks on bulk IN: a 16-byte header
(magic ``EBLK``, flags, sample count, index of the first sample on the
stream's time axis) and interleaved I/Q. A jump in the index is a gap (the
host did not read in time, or a retune restarted the ring); the host fills
it with zeros to keep the time axis. Transmit data is plain interleaved I/Q
on bulk OUT.

Changing a setting while streaming restarts the ring (a gap of about 4 ms
for a retune) or the transmit session.

Host tool
*********

.. code-block:: console

   cc -O2 -Iesp-sdr-zephyr/apps/osmosdr/src esp-sdr-zephyr/apps/osmosdr/tools/esdr_usb.c \
      -lusb-1.0 -lm -o esdr_usb
   ./esdr_usb -s <serial> info
   ./esdr_usb -s <serial> test-in        # USB throughput in; test-out for out
   ./esdr_usb -s <serial> rx -f 2450e6 -r 250e3 -g 32 -b 16 -t 10 -o rx.cs16
   ./esdr_usb -s <serial> tx -f 2450e6 -r 250e3 -G 4 -T 50e3 -a 0.3 -t 10

SoapySDR, GNU Radio, gqrx
*************************

``soapy/`` holds a SoapySDR module (``driver=esdr``) over the same protocol:
receive and transmit (half duplex: activating one direction stops the
other), CS8, CS16 and CF32, gains ``RF`` (receive index) and ``TX`` (power
step), frequency correction in ppm. gr-osmosdr reaches it through its
``soapy`` backend, which is how GNU Radio and gqrx use it.

.. code-block:: console

   cmake -S esp-sdr-zephyr/apps/osmosdr/soapy -B build-soapy && cmake --build build-soapy
   export SOAPY_SDR_PLUGIN_PATH=$PWD/build-soapy
   SoapySDRUtil --find="driver=esdr"
   gqrx        # device string: soapy=0,driver=esdr  (add ,serial=<serial> for one board)

``port=/dev/...`` in the device string selects the serial link instead, for the ESP32-C6's
:ref:`osmosdr_serial <esp_sdr_osmosdr_serial>` app.

In GNU Radio: ``osmosdr.source(args="numchan=1 soapy=0,driver=esdr")`` and
``osmosdr.sink(...)`` with the same string, at one of the receive rates
(250 kS/s down to 15.625 kS/s; other requests get the nearest one, which
``getSampleRate()`` reports). Changing a setting while streaming restarts
the device's stream: expect a few ms of zeros.

Spectrum/waterfall viewer
*************************

``tools/esdr_scope.py`` is a small standalone GNU Radio + PyQt5 app: one
window, a frequency spectrum above a waterfall, with on-screen frequency,
sample rate, gain and ppm correction controls. It talks to the board
through the SoapySDR module above (build that first).

.. code-block:: console

   esp-sdr-zephyr/apps/osmosdr/tools/esdr_scope.sh --serial <serial> \
       --freq 2450e6 --rate 250e3 --gain 32 --corr -7.5

``esdr_scope.sh`` runs the script with GNU Radio's own Python (PyQt5 and
``gnuradio.qtgui`` live there) and the Qt platform plugin path, both found
from the ``gnuradio-companion`` wrapper nixpkgs builds, so it needs no
changes to the project's ``shell.nix``.

Measured
********

Two XIAO ESP32S3 boards cabled through a 30 dB attenuator.

- USB full speed, vendor bulk: 1.16 MB/s in, 1.18 MB/s out.
- Receive 250 kS/s, 16 bit (1.0 MB/s) and 8 bit, with fine tuning: 60 s,
  15.0 M samples, no gaps. The filter takes 12.7 to 13.3 of the 15 cycles
  per input pair CPU 1 has at 16 MS/s (``sdr bench`` breaks it down).
- Transmit 250 kS/s from the host: 66 s, nothing dropped, no underruns. A
  tone from one board lands where expected on the other (after the 7.5 ppm
  crystal correction between the two boards, to within their drift of
  about 100 Hz). The other board's carrier leak is about 40 dB down, the
  image about 43 dB.
- Receive to transmit and back on one board: no errors.
- GNU Radio 3.10 through gr-osmosdr's soapy backend: 20 s at 250 and
  125 kS/s with no gaps (the module logs its counters when a stream stops); transmit a ``sig_source`` tone
  for 7 s, nothing dropped. ``SoapySDRUtil --rate`` measures 0.24 MS/s
  (start included).

Limitations
***********

- USB full speed caps receive at 250 kS/s (the filter allows no less than
  64 times decimation); 500 kS/s at 8 bit would fit USB but needs a filter
  stage with an odd window step.
- Transmit interpolates linearly: images at multiples of the sample rate
  are only attenuated by the interpolator's sinc squared response.
- Gains are uncalibrated PHY indices; the crystal error (several ppm) needs
  ``SET_FREQ_CORR``.
- No flash writes while streaming: the ring CPU runs with interrupts
  masked. DFU stops the stream first.
