For the Seeed XIAO ESP32S3 (8 MB flash) and an x86_64 Linux PC:

- `@IMAGE@`: multiapp, with MCUboot and every app on its FAT volume.
- `@PLUGIN@`: the SoapySDR module for the board's osmosdr app (`driver=esdr`), built on
  Ubuntu 24.04 for SoapySDR ABI @ABI@ and libusb 1.0. Elsewhere, build it from
  `apps/osmosdr/soapy`.
- `esp32s3-esp-sdr-@TAG@.svd`, `esp32c6-esp-sdr-@TAG@.svd`: Espressif's SVDs with the radio
  registers this project documents (`doc/undocumented-registers-*.md`), for debuggers.

## gqrx

1. Flash, with the board in download mode (hold BOOT while plugging it in, or run `usj` on the
   shell of a running esp-sdr app):
   ```
   esptool --chip esp32s3 write-flash 0x0 @IMAGE@
   ```
2. Load the plugin: SoapySDR loads the modules in the directories `SOAPY_SDR_PLUGIN_PATH` names.
   ```
   mkdir -p ~/.local/lib/esdr && cp @PLUGIN@ ~/.local/lib/esdr/
   export SOAPY_SDR_PLUGIN_PATH=~/.local/lib/esdr
   ```
3. Start osmosdr on the board, which reboots into it (again after every power cycle);
   `SoapySDRUtil --find=driver=esdr` then lists it:
   ```
   printf 'app run osmosdr\r' > /dev/serial/by-id/usb-Zephyr_Project_ESP-SDR_multiapp_*
   ```
4. Run `gqrx` from the same shell. In its I/O device configuration: device string
   `driver=esdr,soapy=1`, input rate 250000 (the most USB full speed carries).

On Ubuntu, `apt install gqrx-sdr soapysdr-tools` brings gqrx and SoapySDR. Without root, the PC
needs udev rules for the board (USB vendor 2fe3), for example in
`/etc/udev/rules.d/70-esp-sdr.rules`:
```
SUBSYSTEM=="usb", ATTRS{idVendor}=="2fe3", MODE="660", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2fe3", MODE="660", GROUP="dialout", TAG+="uaccess"
ATTRS{idVendor}=="2fe3", ENV{ID_MM_DEVICE_IGNORE}="1"
```

The other apps: `app list` and `app run <name>` on the board's USB shell
(`apps/multiapp/README.rst`).
