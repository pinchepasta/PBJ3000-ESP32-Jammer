.. _esp_sdr_link:

link
####

Overview
********

A packet radio link between two ESP32-S3 boards on the raw Wi-Fi I/Q paths
of the ``esp-sdr`` module, at the native 80 MS/s, with an iperf style
throughput test on the USB shell.

- **PHY** (``src/qam.c``): single carrier QPSK to 4096-QAM, Gray coded per
  axis, raised cosine pulses (roll-off 0.35) at 4 samples per symbol
  (20 Mbaud, ``CONFIG_APP_SPS``). A frame is a preamble (twice the same 64
  QPSK symbols), a 32 symbol QPSK header with a CRC-16, and a payload of
  error correction units ending in a CRC-32. Frames are built cyclic and
  played in a DAC loop, so any capture window holds one whole copy. The
  receiver finds the preamble (Schmidl-Cox), times it against the known
  symbols, estimates the carrier offset (and from it the sample clock
  offset), trains an 11 tap equalizer by least squares and tracks phase and
  taps decision directed.
- **Error correction** per frame: RS(255,223), extended Hamming (128,120),
  or none. Full frames fill the whole capture window with a short last unit
  (a shortened RS codeword, fewer Hamming blocks).
- **Pairs**: two half length frames looped together. The receiver decodes
  them on both CPUs and answers with one ACK.
- **MAC** (``src/mac.c``): CSMA/CA. Carrier sense on short captures against
  the measured noise floor, DIFS and binary exponential backoff in sense
  slots, stop and wait ARQ with immediate ACKs, retries and duplicate
  detection. Frames for the next transmission are built on the other CPU
  while the sender waits for the ACK.

Measured
********

Two XIAO ESP32S3 boards cabled through a 30 dB attenuator, TX step 4, RX
gain 32, widest receive filter (``bw 0``), PHY in its 40 MHz mode
(``cbw 2``), 50 us TX/RX turnaround, 5 s per run. MER is about 31.5 dB.

=======  =======  ================  =================
Mod      FEC      pairs (``-w 2``)  single (``-w 1``)
=======  =======  ================  =================
64-QAM   RS       1.29 Mbit/s       1.19 Mbit/s
64-QAM   Hamming  1.41 Mbit/s       1.28 Mbit/s
64-QAM   none     1.54 Mbit/s       1.40 Mbit/s
256-QAM  RS       1.69 Mbit/s       1.47 Mbit/s
256-QAM  Hamming  1.44 Mbit/s       1.35 Mbit/s
256-QAM  none     0.89 Mbit/s       0.60 Mbit/s
=======  =======  ================  =================

The link is bound by the receiver's decode time (about 2.3 us per symbol
per CPU), not by airtime. The MER ceiling of about 31 dB, the same with
or without attenuation, rules out 1024-QAM and above on these boards.

OFDM
****

A second PHY (``src/ofdm.c``) with its link frames (``src/ofdm_frame.c``),
used by the MAC instead of QAM with ``link set phy 1`` on both boards (and
the same ``ofdm set`` settings on both).

- Configured by occupied bandwidth, subcarrier count and cyclic prefix; the
  FFT size follows (the power of two, 16 to 512, that spaces that many
  subcarriers over that bandwidth at the sample rate). Subcarriers on both
  sides of DC, DC unused. BPSK to 64-QAM (square, Gray coded per axis),
  chosen per frame (the client's ``-m``).
- The whole PHY runs at 40 MS/s by default (``fs 40``; the DAC at 40 MS/s
  costs about 4 dB MER against 80 MS/s, but halves the work on both ends),
  or at 80 MS/s with the receiver at 40 MS/s (``fs 80 rxdiv 2``) or 80.
- A frame: a Schmidl-Cox preamble (known BPSK on the even subcarriers), two
  pilot symbols, a BPSK header (type, modulation, addresses, sequence
  number, CRC-16; its bits repeated over the subcarriers and combined soft),
  the payload with another pilot in its middle, each symbol with a cyclic
  prefix. The payload is RS(255,223) in equally shortened units, byte
  interleaved over the frame, with a CRC-32. The frame loops in the DAC and
  fills half the capture window, so a whole copy always fits.
- Receiver, on the raw capture words: an integer sliding Schmidl-Cox search
  (coarse at twice the stride when its peak is clear, then at full
  resolution), the carrier offset from its phase, the channel from the two
  pilots (averaged, smoothed over neighbouring subcarriers, their phase step
  giving the residual offset), fine timing from the pilots' phase slope (the
  FFT window moved to the middle of the prefix), then per symbol a
  derotation table, the FFT and one complex multiply per subcarrier, with a
  second order phase tracker. The mid pilot measures the phase again, so the
  payload is demodulated on both CPUs (each half on one), and RS units are
  decoded on both.
- ACKs are header-only OFDM frames (18 us), decoded in 0.8 ms.
- The FFT is the esp-dsp ESP32-S3 kernel (``CONFIG_APP_OFDM_ESP_DSP``,
  west project ``esp-dsp``), 1.8 times faster than the same algorithm in C
  from IRAM. The receive context (tables and buffers, 16 KB) sits in internal
  RAM above the capture bank.

Measured over the MAC (unique bytes ACKed per second; same setup as above,
TX step 4, RX gain 32, ``cbw 2``, ``ackair 200``, 20 s each, no frame lost
in any run):

=========  ===========  ======  ===========  ===========  =======
Bandwidth  Subcarriers  Mod     Bytes/frame  Effective    MER
=========  ===========  ======  ===========  ===========  =======
16 MHz     52           QPSK    548          0.60 Mbit/s  27 dB
16 MHz     52           16-QAM  1100         1.14 Mbit/s  27 dB
16 MHz     52           64-QAM  1684         1.63 Mbit/s  27 dB
26 MHz     84           QPSK    906          0.88 Mbit/s  24 dB
26 MHz     84           16-QAM  1850         1.67 Mbit/s  24 dB
26 MHz     84           64-QAM  2791         2.19 Mbit/s  24 dB
30 MHz     96           QPSK    1056         0.99 Mbit/s  23 dB
30 MHz     96           16-QAM  2116         1.84 Mbit/s  23 dB
30 MHz     96           64-QAM  3176         2.24 Mbit/s  23 dB
=========  ===========  ======  ===========  ===========  =======

The QAM link measured 1.62 Mbit/s in the same session. Each exchange is
bound by the receiver's decode (about 9 ms at 30 MHz: search 1.1, channel
and header 0.7, payload 3.7, RS 3.1 with its many corrections) and, just as
long, the sender's build of the next frame on the other CPU. 64-QAM at
30 MHz runs close to the code's limit (RS corrects 80 bytes per frame);
26 MHz is the default. 256-QAM would need about 33 dB MER.

Wide frames need the 40 MHz mode (``cbw 2``, the default) and ``amp`` 90 or
less (the default); the forced receive gain is lost now and then and the
library re-forces it (``link status``, engine line). Details in
``doc/hardware-quirks.md``.

Building and running
********************

Build with sysbuild (MCUboot, USB DFU) and flash each board once:

.. code-block:: console

   west build --sysbuild -b xiao_esp32s3/esp32s3/procpu esp-sdr-zephyr/apps/link
   west flash

Later updates go over USB DFU with ``west dfu -d <build dir> [-s <usb serial>]``
(see ``apps/osmosdr/README.rst``).

On the shell (``/dev/serial/by-id/usb-Zephyr_Project_ESP-SDR_Link_*``):

.. code-block:: console

   link status                       # address, settings, counters, timing
   link set txgain 4 rxgain 32       # several at once; also bw, cbw, amp, air, turn, ...
   link server                       # on the receiving board
   link client 0x00 -t 10 -m 256qam -f rs -w 2

or from the host, for both boards at once:

.. code-block:: console

   tools/link_perf.py run --server <serial> --client <serial> -t 10 -m 256qam \
           -f rs -w 2 --set txgain=4 --set rxgain=32

``tools/link_perf.py dump`` saves a capture for ``tools/qam_sim.c``, the
host simulator of the PHY (see its header for the build line).

OFDM over the MAC, both boards set alike:

.. code-block:: console

   tools/link_perf.py run --server <serial> --client <serial> -t 20 -m 64qam \
           --set phy=1 --set txgain=4 --set rxgain=32 --set ackair=200

``ofdm set`` changes the layout (several keys at once: bw, ch, cp, fs,
rxdiv, pilots, smooth, amp, syms, mod for the raw test). The raw PHY test
outside the MAC, with known test bits:

.. code-block:: console

   ofdm set bw 26 ch 84 mod 64qam
   ofdm tx -t 10                     # on the sending board
   ofdm rx -n 100 -v                 # on the receiving one; -v: channel per subcarrier,
                                     # -d: dump the first capture with errors
   ofdm bench                        # FFT and RS decode times

   tools/link_perf.py ofdm --rx <serial> --tx <serial> -n 100 \
           --set txgain=4 --set rxgain=32 --set bw=30 --set ch=96

``tools/ofdm_sim.c`` is its host simulator (PHY, link frames with
``FRAME=1``, ACKs with ``ACK=1``, the two-CPU split with ``SPLIT=1``); it
also decodes a capture saved with ``dump``.
