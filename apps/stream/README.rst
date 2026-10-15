.. _esp_stream:

stream
######

Overview
********

Turns a XIAO ESP32S3 into a small SDR on a USB network link: raw I/Q from the
Wi-Fi receiver goes to the host as VITA 49.2 over UDP and IPv6, and VITA 49.2
from the host is transmitted through the Wi-Fi DAC.

- Receive: CPU 0 captures bursts with the ``esp-sdr`` library, CPU 1 packs
  them with ``libvrt`` and sends them (SMP). Raw bursts of 1404 samples at
  16 MS/s, or a 3rd order CIC decimated stream at 16 MS/s / m
  (m 16 to 160) for a continuous low rate view of the band.
- Signal data packets: stream ID 0x5D5D0001, timestamps (seconds since boot
  plus picoseconds), trailer with sample loss at each burst start and
  over-range. Context packets (VITA 49.2, CIF1 compliance field) carry RF
  frequency, sample rate, bandwidth and payload format, at start, on change
  and once per second.
- Sample formats: link efficient complex signed fixed point with 16, 12 or
  8 bit items, chosen by the host per direction (payload format field).
- Transmit: signal data packets (stream ID 0x5D5D0101) are placed by their
  timestamps, interpolated linearly to 40 MS/s and played gapless by the DAC
  in loop mode (see Transmit path).
- USB on the OTG port: CDC-NCM network, CDC-ACM console and shell, DFU.

Requirements
************

A XIAO ESP32S3 (two for a link: one transmits, one receives). The host needs
access to the USB device (VID 0x2fe3) for DFU, an IPv6 link-local address on
the board's network interface and UDP port 4991 open on it. On NixOS:

.. code-block:: nix

   services.udev.extraRules = ''
     SUBSYSTEM=="usb", ATTRS{idVendor}=="2fe3", MODE="660", GROUP="plugdev", TAG+="uaccess"
     SUBSYSTEM=="tty", ATTRS{idVendor}=="2fe3", MODE="660", GROUP="dialout", TAG+="uaccess"
     ATTRS{idVendor}=="2fe3", ENV{ID_MM_DEVICE_IGNORE}="1"
   '';
   systemd.network.links."10-esp-sdr" = {
     matchConfig.MACAddress = "00:00:5e:00:53:01";
     linkConfig.Name = "esdr0";
   };
   networking.firewall.interfaces.esdr0.allowedUDPPorts = [ 4991 ];

Every board uses the same MAC address: with several boards, match the links by
USB path instead and keep them link-local only. The host tools find a board's
interface from its USB serial number.

Building and flashing
*********************

The sample needs MCUboot (sysbuild) and the manual gain blob:

.. code-block:: console

   west blobs fetch esp-sdr-zephyr
   west build --sysbuild -b xiao_esp32s3/esp32s3/procpu esp-sdr-zephyr/apps/stream
   west flash

Later updates: ``west dfu -s <usb serial>`` (see ``apps/osmosdr/README.rst``).

Receiving
*********

.. code-block:: console

   python3 esp-sdr-zephyr/apps/stream/tools/vrt_rx.py --iface esdr0 --seconds 10 --out iq.cs16

The script prints rates, decoded context and counter gaps, writes the samples
as little endian complex int16, and exits 0 when the stream is valid.

Command packets to the receive stream ID set the receiver: RF reference
frequency (whole MHz), gain (stage 1 is the PHY gain table index, not dB;
stage 2 must be 0), sample rate (16 MS/s, or 16 MS/s / m for decimation) and
payload format (16, 12 or 8 bit items). Query-state requests return them.

USB full speed limits the receive stream to about 1 MB/s: 0.24 MS/s raw at
16 bit, 0.32 MS/s at 12 bit, 0.48 MS/s at 8 bit. Decimated streams are
limited by the CIC on the capture CPU (about 30 % of the time covered at
200 kS/s).

The shell is on the CDC-ACM port (``/dev/ttyACM*``): ``sdr status``
(settings, transmit path state and counters), ``sdr set <key> <value> ...``
(``freq <MHz>``, ``rxgain auto|<index>``, ``decim cic|fold``, ``txgain
<step>``; ``sdr set`` lists them), ``sdr tonetx``, ``sdr sweep``, ``sdr mem``,
``net iface``, ``mcuboot``, ``kernel reboot cold``.

Transmit path
*************

The board accepts VITA 49.2 packets on UDP port 4992:

- Command packets to the transmit stream ID set the TX RF frequency (whole
  MHz, it is the receiver's LO too), sample rate (40 MS/s divided by an
  integer of at least 2), gain (stage 1 is the power step, 0 weakest, see
  ``sdr set txgain``) and payload format. Errors are answered with execution
  acknowledges, query-state requests return the settings.
- Signal data packets are placed by their timestamps; missing packets become
  zero filled holes. TX starts with the first data packet and stops 500 ms
  after the last.

The DAC backend keeps a 256k sample ring in PSRAM and interpolates each block
into one of two SRAM banks while the DAC plays the other. The DAC runs in
loop mode, so playback is gapless and locked to real time; the bank switches
on a precomputed cycle of the leader's CPU. Measured on air (``sdr status``):
100 % at 16 bit / 200 kS/s, 12 bit / 250 kS/s and 8 bit / 400 kS/s, which is
also where USB full speed tops out.

Host tools
**********

``tools/esp_sdr_lab.py`` drives two boards (by USB serial) and records the
receiver:

.. code-block:: shell

   # tone or chirp from A to B, decimated capture at B, cs16 + json on a real time axis
   python3 esp_sdr_lab.py link --tx <serial> --rx <serial> --signal chirp --offset 0 \
       --span 80000 --tx-rate 250000 --tx-bits 12 --rx-bits 12 --out chirp.cs16
   # spectrum painter: a picture on the receiver's waterfall, also saved as picture_rx.png
   python3 esp_sdr_lab.py link --tx <serial> --rx <serial> --signal image --image picture.png \
       --offset 0 --tx-rate 250000 --tx-bits 12 --rx-bits 12 --out picture_rx.cs16
   # the same picture to an IQ file, no boards
   python3 esp_sdr_lab.py paint picture.png --out picture.cs16

``tools/vrt_tx.py`` checks the transmit control and data handling on its own.

Sample output
*************

.. code-block:: console

   vrt_rx: stream 0x5d5d0001 from fe80::ff:fea5:ae0
   vrt_rx: context VITA 49.2, rf 2412.000 MHz, rate 16.000 MS/s, bandwidth 16.000 MHz
   vrt_rx: 597 data packets/s, 829.7 kB/s, 0.204 MS/s, -25.2 dBFS, gaps 0, bad 0
   ...
   vrt_rx: 3570 data, 6 context packets, 1218560 samples, 0 gaps, 0 bad: PASS
