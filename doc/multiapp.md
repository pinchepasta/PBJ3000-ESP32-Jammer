# multiapp: one firmware that runs the others

Status: all six apps (`capture`, `rfprobe`, `wpan`, `stream`, `osmosdr`, `link`) run as
llext extensions from PSRAM, built from their unchanged sources, one app per boot, on the image of
`apps/multiapp/` ("the host"). The host loads each app from its `<name>.llext` file on a FAT
volume in flash, which the PC sees as a USB drive in boots without an app; built-in copies are
the fallback. It needs Zephyr changes that are on the fork `west.yml` pins ("Zephyr changes").
Usage: `apps/multiapp/README.rst`.

## Constraints

- **Every app still builds and runs standalone**, with its own `prj.conf`, overlay and `main()`,
  unchanged in behavior. multiapp compiles the same sources; no app source moves.
- **Internal RAM is the binding constraint.** IRAM and DRAM share one pool, and the image must
  end below the dump banks the configured apps reserve ([hardware-quirks.md](hardware-quirks.md),
  "Memory"). Extensions therefore run from PSRAM.
- **Kernel configuration cannot differ per app.** USB classes, network buffers, SMP and heap
  sizes are fixed when the host is built; the host carries the union of what its apps need.
- **Constructs that register through linker iterable sections** (`SHELL_CMD_REGISTER`,
  `USBD_DEFINE_CLASS`, `SYS_INIT`, `LOG_MODULE_REGISTER`, `K_THREAD_STACK_DEFINE`, static memory
  slabs, `NET_BUF_POOL_DEFINE`) do not work in an extension as written: its sections do not join
  the host's.
- **Exit is a reboot.** The Wi-Fi PHY, ring and DAC state cannot be torn down cleanly. `app run`
  with an app loaded leaves the name in RTC memory and resets.

## How it works

- **Code in PSRAM.** The S3's cache MMU maps the data bus window (0x3c000000) and the
  instruction bus window (0x42000000) through one table, so a PSRAM page written at data address
  D executes at D + 0x06000000 after a data cache write-back and an instruction cache
  invalidate. llext keeps one address per region, so a local Zephyr change
  (`CONFIG_LLEXT_EXEC_ALIAS`, `llext_heap_init_harvard_exec()`) copies and patches through the
  writable address and uses the alias for relocations, the symbol table and the init/fini check.
  The ESP cache driver has no instruction cache operations: the host syncs the range itself after
  `llext_load()` (`sync_text()` in `src/apps.c`). Heaps: `CONFIG_MULTIAPP_INSTR_HEAP_KB` (256) and
  `CONFIG_MULTIAPP_DATA_HEAP_KB` (512), both in PSRAM.
- **A prelude** (`include/multiapp/ext.h`, put in front of every C source with `-include`) turns
  what an extension cannot do into what it can: `SHELL_CMD_REGISTER()` and `SYS_INIT()` become
  constructors, `LOG_MODULE_REGISTER()` a declaration, slabs are initialized in a constructor,
  `NET_BUF_POOL_DEFINE` leaves a descriptor in `.multiapp_host` from which the host defines the
  pool (net_buf finds pools by their index in a host section). `.noinit` and the PSRAM section
  attributes join `.bss`, because llext loads only one NOBITS section. The app's `app_usb_init()`
  becomes a call that does nothing (see "Bugs found").
- **Generated host glue** (`exports.py`): the host exports exactly the symbols the extensions
  import, in its own export group (Zephyr's default groups, every syscall kept in IRAM, are off),
  and registers their log modules. A symbol the host lacks fails the host's link. With
  `CONFIG_LLEXT_IMPORT_ALL_GLOBALS` the host finds an app's hooks by name, such as
  `app_usb_dfu_prepare()`.
- **The app's own Kconfig values.** An extension compiles against the host's `autoconf.h`;
  `appconfig.py` writes the app's `CONFIG_APP_*` values into a header included after it.
- **Proxy roots** (`src/apps.c`): the host registers the apps' root commands (`probe`, `wpan`,
  `sdr`, `link`, `ofdm`, `hello`) as dynamic commands that forward to the loaded app, so the shell
  and the tools that drive it look as standalone. The app's `main()` runs in a host thread
  (`CONFIG_MULTIAPP_STACK_SIZE`, stack in PSRAM).
- **USB profiles.** All class code lives in the host. The S3's OTG controller has 4 IN endpoints
  for classes (`GHWCFG4.INEps`), and the union of the apps' classes needs 6, so the host registers
  a subset per boot: `app_usb_classes()` (a weak hook in `apps/common` between registering the
  classes and `usbd_init()`) unregisters what the boot's app does not name in its `USB` mask.

  | Class | Used by | IN endpoints |
  | --- | --- | --- |
  | CDC-ACM (shell) | every boot | 2 |
  | CDC-NCM | stream | 2 |
  | vendor bulk (`usb_sdr.c`) | osmosdr | 1 |
  | MSC (the volume as a drive) | boots without an app | 1 |
  | DFU | every boot | 0 |

  osmosdr's vendor class is built in the host; its calls into the app go through forwarders in
  `src/osmosdr.c` that look the app's functions up by name, so the boot's app is loaded before
  USB starts.
- **NCM's DMA buffers in bank 0** (`multiapp_bank0.ld`): USB DMA needs internal RAM, and
  `stream` never runs the ring, so bank 0 is free in its boots.
- **An internal RAM area, one user per boot.** Buffers an app must keep in internal RAM
  (`ESP_SDR_HIGH_BUF_DEFINE()`, `ESP_SDR_HIGH_STACK_DEFINE()`) become host objects in
  `multiapp_area`, 24 KB of the high region (`AREA` in `multiapp_extension()`). osmosdr takes
  23 KB; the 802.15.4 driver gets it in the other boots
  (`CONFIG_ESP_SDR_IEEE802154_HIGH_RAM_APP`). `app run` reboots when the profile or the area owner
  changes.
- **The apps as files** (`src/drive.c`). A FAT volume in the upper 4 MB of the flash, which the
  board's 4 MB partition table leaves out (partition `fat` at 0x400000, flash disk `FLASH`),
  holds `<name>.llext`. The host mounts it read-only at boot, before USB starts and before an app
  runs, compares every file with the built-in copy (`app list`), and reads the boot's app whole
  into a PSRAM buffer (`CONFIG_MULTIAPP_FILE_MAX_KB`); llext then loads it from there. The FATFS
  object, the flash disk cache and the drive's SCSI buffer are in bank 0, which the ring
  overwrites once an app runs: nothing touches the volume after that. Reads go in 256-byte
  chunks through the FATFS window, so the flash driver never gets a PSRAM buffer.
- **A trailer on every file** (`apptrail.py`): ELF size, CRC-32, the stack the app's `main()`
  needs and a 16-byte build id (`buildid.py`: a hash of `autoconf.h`, the devicetree, the
  extensions' imports, the headers both sides compile against and the compiler). The host checks
  it before llext reads the file: a truncated, damaged or foreign file (built for another host)
  is refused and the built-in copy runs (`CONFIG_MULTIAPP_EMBEDDED`, default y; without it the
  image is 165 KB smaller and such a boot comes up without an app). `app info` says where the app
  came from.
- **The drive.** A boot without an app (no request, no autostart, or an app that failed to
  start) registers USB mass storage on the volume next to the shell and DFU, for copying app
  files from the PC; boots with an app unregister it, and `app run` from a drive boot reboots
  into the app. The PC must eject the drive first: the host cannot see what the PC still caches.
  `west dfu --storage` writes the build's volume image (`mkfat.py`: deterministic FAT12, 4 KiB
  clusters on the erase page, long names) to the DFU target `fat`, and `west dfu --esptool`
  writes MCUboot, the image and the volume over the USB-Serial-JTAG, which the running app hands
  over with the shell command `usj`.

## Memory budget

The host's image ends 3840 bytes below bank 0 (0x3fcb0000), with the volume (1.1 KB, mostly the
DFU write buffer) and the drive (2.2 KB). What got it there: the Wi-Fi driver's
fewest static receive buffers (2, no A-MPDU receive; system heap peak 12216 bytes, heap 13312),
`CONFIG_ESP32_WIFI_NET_ALLOC_SPIRAM`, the export group above, `CONFIG_NUM_MBOX_ASYNC_MSGS=0`, the
app's `main()` stack in PSRAM. The high region has 1.2 of its 37.75 KB free (DAC, ring, area), the
area 0.5 KB. Log queue 256 messages, link's shell stack 3072 bytes. Internal RAM stacks are
sized so their peak with every app exercised is at most 80 % (`kernel thread stacks`; main's in
`app info`, the USB thread's in the last DFU download in `app crash`): main 1120 of 1536, USB
thread 1008 of 1280, drive thread 880 of 1152 while the PC copies 2 MB.

## Measured on the XIAO

- wpan against the C6 (`interop.py`, 100 frames of 100 bytes): 100 of 100 each way and 50 of 50
  ACKs on channels 11, 15, 20 and 25; DFU while the ring runs.
- `stream`: 947 kB/s of VITA 49.2 over NCM (standalone 968 kB/s); `vrt_tx.py` passes as
  standalone.
- osmosdr (`esdr_usb`): `test-in` 1154 kB/s, receive at 250 kS/s (8 and 16 bit) without overflow
  or gaps, 1.25 M samples transmitted with none dropped, ring at 13.3 of 15 cycles per pair.
- link: QAM build and decode within 1 to 4 % of standalone, FFT and RS the same, OFDM test frame
  5583 us (standalone 5589). `link bench`'s float loop and power sum are 1.4 and 2.1 times slower
  (the shell runs them on CPU 0 here, CPU 1 standalone; not isolated). PSRAM reads miss the cache
  once, then match internal RAM. Not tested: a link between two boards.
- 48 app switches, each into the requested app; all six apps run from their files with the same
  results.
- The volume: `west dfu --storage` reads back byte for byte (DFU upload), fsck.vfat is clean;
  copying through the drive runs at about 63 kB/s (every 4 KiB page erased and rewritten), files
  read back identical after a remount and run after ejecting. `west dfu --esptool --storage`
  writes MCUboot, image and volume in 27 s. A host built with one Kconfig value changed refuses
  the old files. The desk check `sample.esp_sdr.multiapp.hil` ([testing.md](testing.md)) passes.

## Bugs found

- **Atomics in PSRAM are not atomic** (`hello atomic`); local Zephyr fix
  ([hardware-quirks.md](hardware-quirks.md), "Memory").
- **No flash access from a PSRAM stack.** The app's `main()` raced the host's `app_usb_init()`
  and ran MCUboot's image check on its PSRAM stack: double fault, board hung after some app
  switches ([hardware-quirks.md](hardware-quirks.md), "Memory").
- **Extensions built at `-O0`:** Zephyr drops `-Os` from Xtensa extensions; a 1472-byte frame
  (32 at `-Os`) overflowed a 2048-byte stack. multiapp keeps the host's `-Os`.
- **llext relocations:** `R_XTENSA_GLOB_DAT` dropped its addend; alignment padding made `.bss`
  claim `.exported_sym`'s relocations; a shared object's `.bss` sat 12 bytes off its 16-byte
  alignment, which the S3's 128-bit loads round down silently. Local Zephyr fixes.
- **Stale extensions:** Zephyr's llext CMake rule never rebuilt `<target>_debug.elf` after an
  edit. Local Zephyr fix.
- **Unregistering NCM before init** crashed (its shutdown removed a descriptor it never added).
  Local Zephyr fix.
- `link decode` and `link bench` overflowed the shell stack (a 3.6 KB decode chain): they run on
  the paused MAC thread now (`link_mac_run_paused()`). osmosdr's receive thread has 2560 bytes.
- llext frees an extension's symbol table after loading; the shell recognizes dynamic entries by
  host section address.
- `flash_img` writes an MCUboot image, from the start the swap-using-offset layout gives the
  update slot: the volume landed 4 KiB into its partition. Its DFU target uses `stream_flash`.

## Zephyr changes

Eight commits on the fork's `esp-sdr-multiapp` branch, which `west.yml` pins, on top of the
SMP port: exec alias, llext repackage dependency, NCM/ECM unregister before init, Xtensa
GLOB_DAT addend, alignment padding, PSRAM atomics (two commits), shared object region alignment.
They are candidates for upstream Zephyr.

## The standalone apps (2026-10-09)

Each app's S3 build, read from `zephyr.elf` (the gap is from the end of the DRAM image to the
first dump bank the app reserves):

| App | IRAM | DRAM | Gap to first bank | High region | Flash code | Flash rodata | PSRAM |
| --- | --- | --- | --- | --- | --- | --- | --- |
| capture | 61 KB | 170 KB | 125 KB | 0 | 341 KB | 447 KB | 0 |
| rfprobe | 67 KB | 189 KB | 106 KB | 0 | 408 KB | 528 KB | 0 |
| wpan | 89 KB | 164 KB | 0.2 KB | 37 KB | 430 KB | 532 KB | 2.6 MB |
| osmosdr | 87 KB | 163 KB | 0.4 KB | 37 KB | 413 KB | 530 KB | 2.6 MB |
| stream | 84 KB | 228 KB | 1.7 KB | 20 KB | 491 KB | 628 KB | 2.6 MB |
| link | 92 KB | 228 KB | 1.0 KB | 39 KB | 460 KB | 599 KB | 1.7 MB |

The part every app has (kernel, drivers, Wi-Fi and PHY blobs) is 60 to 75 KB of IRAM and 60 to
90 KB of DRAM; the apps' own DRAM is mostly buffers. Imports per app from the host: capture 17,
rfprobe 56, wpan 60, osmosdr 68, stream 108, link 92; 197 symbols in the union.

## Design choices

- **No new partition table.** The volume takes the upper 4 MB the board's 4 MB table leaves
  unused, so MCUboot, the slots and DFU stay compatible with the standalone apps.
- **No `fs_loader`.** It reads the symbol table and every relocation entry by entry (about four
  times the file size in flash reads) and would overflow main's stack; the host reads the file
  whole into PSRAM and keeps the buffer loader.
- **A build id, not an ABI hash with a manual bump.** llext binds by name only, so a file is
  accepted only from a host built from the same inputs.
- **Built-in copies stay on** (`CONFIG_MULTIAPP_EMBEDDED`) until the volume has proven itself on
  more boards; off, the image shrinks by 165 KB.

## Risks

- Extension code cannot run while flash is written (cache off): interrupt handlers and code that
  runs then stay in the host.
- No clean unload: exit is a reboot.
- The local Zephyr commits above, until upstream.
- link throughput between two boards is untested.
- The PC's write cache: files copied onto the drive reach the flash only when the PC writes them
  out, which ejecting guarantees; `app run` reminds of it in a drive boot.
