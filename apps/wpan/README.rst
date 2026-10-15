.. _esp_sdr_wpan:

wpan
####

IEEE 802.15.4 test shell

Overview
********

Sends and receives IEEE 802.15.4 test frames through any Zephyr 802.15.4
radio driver, with a ``wpan`` shell to compare radios against each other:

- on the ESP32-C6, the native radio (``ieee802154_esp32``) in raw mode;
- on the ESP32-S3, the ``esp-sdr`` software radio (``CONFIG_ESP_SDR_IEEE802154``,
  device ``esp_sdr_154``), without the L2 (``doc/ieee802154.md``); frames come
  through ``ieee154_esp_sdr_set_rx_cb()``.

Test frames are data frames (PAN ID compression, short addresses) whose
payload starts with the magic ``X154`` and a 32-bit counter, followed by a
pattern derived from the counter. The receiver checks the pattern and counts
frames, test frames, corrupt ones, duplicates and counter gaps (lost frames).

Shell
*****

.. code-block:: none

   wpan status                   radio, settings, receive and transmit counters
                                 (and the software radio's counters on the S3)
   wpan set [<key> <value> ...]  settings, several at once; without arguments the keys:
                                 chan 11..26 (2405 + 5 (n - 11) MHz), txpower <dBm>,
                                 promisc 0|1, pan <PAN ID>, addr <short> (auto-ACK on
                                 both radios), print 0|1|2 (nothing, a line, a line and a hex
                                 dump per frame); on the S3 also rxgain auto|<index>,
                                 rxinvert 0|1, txinvert 0|1, lo below|above,
                                 unit <pairs> (ring unit length, default 1024),
                                 ack <delay us>[,<limit us>] (ACK timing, 192,512)
   wpan clear                    zero the receive and driver counters
   wpan on | off                 start or stop receiving (the S3 software radio starts off)
   wpan tx <count|0> [ms] [payload] [dst] [ack] [csma]
                                 send test frames (0: until wpan stop)
   wpan stop                     stop a running tx
   wpan measure                  one 1 ms capture: carrier offset, power, chip phase steps (S3)
   wpan dump                     one 1 ms capture as base64 I/Q, for tools/iq_dump.py (S3)
   wpan crash                    reset cause, driver event trail, stalled CPU 0, last crash (S3)
   wpan mem                      system heap use (with CONFIG_SYS_HEAP_RUNTIME_STATS)
   wpan bench acktx [seq]        one ACK on the air now (S3, receiving)
   wpan bench [units] [subsample]|phy|ack
                                 ring decimation by 4 and PHY cost on synthetic data, or the
                                 ACK waveform build and front end switch times (S3, radio off
                                 after one wpan on)

The shells of the other apps follow the same pattern: ``status``, ``set <key>
<value> ...``, ``clear``, and verbs for actions.

Building and running
********************

ESP32-C6, native radio, over USB-Serial-JTAG:

.. code-block:: console

   west build -b xiao_esp32c6/esp32c6/hpcore esp-sdr-zephyr/apps/wpan
   west flash

ESP32-S3, software radio, with the shell on the OTG port and DFU updates as
the other apps (see ``apps/osmosdr/README.rst``):

.. code-block:: console

   west build --sysbuild -b xiao_esp32s3/esp32s3/procpu esp-sdr-zephyr/apps/wpan
   west flash                    # first install; later: west dfu -s <usb serial>

The S3 build uses the gapless ring receive (three dump banks), so its thread
stacks and tables live in ``ESP_SDR_HIGH_RAM`` (``doc/hardware-quirks.md``).

Example: C6 sends, S3 receives on channel 15:

.. code-block:: none

   s3:~$ wpan set chan 15 print 0
   s3:~$ wpan on
   c6:~$ wpan set chan 15
   c6:~$ wpan tx 1000 5 16
   s3:~$ wpan status
