.. _esp_sdr_multiapp:

multiapp
########

one firmware that runs the others

Overview
********

One image that runs the other apps as Zephyr loadable extensions (llext,
``doc/multiapp.md``), with the USB classes they need living in the image
itself. ``capture``, ``rfprobe``, ``wpan``, ``stream``, ``osmosdr`` and
``link`` run as extensions, built from their own sources and loaded from their
files on a FAT volume in the flash, with copies built into the image as the
fallback; ``hello`` (``ext/hello.c``) is the loader check (``hello atomic``
tests atomics in PSRAM from both CPUs).

An extension's code and data are loaded into PSRAM; the code runs at the
instruction bus alias of its PSRAM copy (``CONFIG_LLEXT_EXEC_ALIAS``, on the
Zephyr branch ``west.yml`` pins). One app runs per boot: an app cannot be
stopped cleanly, so starting another one reboots into it. USB enumerates once
per boot too: the classes an app needs besides the CDC-ACM shell and DFU
(``stream``'s CDC-NCM, osmosdr's vendor class) come with a reboot into it, as
does the internal RAM area that osmosdr and the 802.15.4 driver take turns on.

Building and flashing
*********************

.. code-block:: console

   west build -d build/multiapp --sysbuild -b xiao_esp32s3/esp32s3/procpu esp-sdr-zephyr/apps/multiapp
   west flash -d build/multiapp            # first install
   west dfu --storage -d build/multiapp    # the apps' files: the FAT volume

Later, ``west dfu -d build/multiapp`` updates the image and ``west dfu
--storage`` the volume; a new image needs its volume too (see "Apps as
files"). ``west dfu --esptool [--storage]`` writes MCUboot, the image (and the
volume) with esptool, without the boot button: the running app hands the USB
port to the USB-Serial-JTAG first (shell command ``usj``).

Then, on the CDC-ACM shell:

.. code-block:: none

   uart:~$ app list
   hello          3448 bytes, file 3448 bytes
   capture        5124 bytes, file 5124 bytes
   rfprobe        8596 bytes, file 8596 bytes
   wpan          18216 bytes, file 18216 bytes
   stream    30264 bytes, file 30264 bytes
   osmosdr       17900 bytes, file 17900 bytes
   link          81320 bytes, file 81320 bytes
   uart:~$ app run rfprobe
   rfprobe: rebooting into it (eject the drive first: the PC may not have written it out)

and, on the shell of the boot into it:

.. code-block:: none

   uart:~$ app info
   host 0.1.0, build 8a61693d
   boot: rfprobe, start 0, USB profile 0x0
   rfprobe (file): text 3002 at 0x42167c5c, data 448, bss 4, rodata 872, 6067 bytes in heaps
     command probe
   uart:~$ probe f 2437
   f 2437 ret 0

The app's root commands (``probe`` here) answer as in the standalone app.
``app run <name>`` reboots into the app unless no app runs yet and USB already
has its classes; ``CONFIG_MULTIAPP_AUTOSTART`` names an app to start at every
boot. ``app info`` shows the host's build id, the stack its ``main()`` used and
the loaded app's origin, sizes and commands; ``app crash`` what ended the
previous boot (fatal error, watchdog, how far a boot into an app got) and the
USB thread's stack use in the last DFU download.

Apps as files
*************

The upper 4 MB of the flash hold a FAT volume with one ``<name>.llext`` per
app (``src/drive.c``). Every boot reads it before USB starts: ``app list``
shows each app's file next to the built-in copy (``differs`` when they are
not the same), and the boot's app is loaded from its file when the file is
good, else from the built-in copy. "Good" is checked before llext sees the
file, from a trailer the build appends (``apptrail.py``): size, CRC-32, the
stack the app needs, and the host's build id (``buildid.py``), so only files
from the same build load. ``CONFIG_MULTIAPP_EMBEDDED=n`` drops the built-in
copies (165 KB of flash); a boot then runs an app only from its file.

In a boot without an app the PC sees the volume as a USB drive next to the
shell. Copy files from ``build/multiapp/multiapp/llext/files/`` onto it, eject
it (the PC may cache what it writes), then ``app run`` the app. Writes are
slow, about 60 kB/s: every 4 KiB flash page is erased and rewritten. ``west
dfu --storage`` writes the whole volume (``mkfat.py`` builds its image).

Adding an app
*************

One line in ``CMakeLists.txt``:

.. code-block:: cmake

   multiapp_extension(rfprobe STACK 2048 SOURCES ${apps_dir}/rfprobe/src/main.c)

The build puts ``include/multiapp/ext.h`` in front of every source (shell
commands, ``SYS_INIT``, log modules, memory slabs and net_buf pools become
things an extension can do) and exports from the host exactly what the
extensions import (``exports.py``). ``APP <dir>`` gives the extension its app's
own ``CONFIG_APP_*`` values (``appconfig.py``), ``USB`` the classes it needs,
``AREA`` its internal RAM buffers (``ESP_SDR_HIGH_BUF_DEFINE()``) in the area.
The host needs the app's other Kconfig options; a missing function fails the
host's link.

Testing
*******

``sample.esp_sdr.multiapp.hil`` checks a board on the desk: the image and its
volume go on with ``scripts/twister-flash.py``, the board boots into ``hello``
from its file, and ``pytest/test_multiapp.py`` checks the files, the boot and
hello's commands (``doc/testing.md``).
