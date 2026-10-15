.. SPDX-License-Identifier: GPL-3.0-or-later

esp-sdr-zephyr
##############

A Zephyr module that turns the Wi-Fi radio of the ESP32-S3 into a software
defined radio (raw I/Q receive and transmit; burst capture on the ESP32-C6),
an IEEE 802.15.4 software radio built on it, and the apps that use both.

The API reference of the library is generated with Doxygen:
`API reference <doxygen/index.html>`_.

.. toctree::
   :maxdepth: 1
   :caption: Overview

   overview

.. toctree::
   :maxdepth: 1
   :caption: Apps

   apps/capture
   apps/stream
   apps/osmosdr
   apps/osmosdr_serial
   apps/link
   apps/wpan
   apps/rfprobe
   apps/multiapp

.. toctree::
   :maxdepth: 1
   :caption: Guides

   hardware-quirks
   undocumented-registers-esp32s3
   undocumented-registers-esp32c6
   signal_path
   ieee802154
   ieee802154-ack
   testing
   multiapp
   zigbit-reference

Indices
=======

* :ref:`genindex`
* :ref:`search`
