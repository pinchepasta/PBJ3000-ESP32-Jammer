# ESP32-S3 SDR: hardware quirks and how they are handled

Behaviour of the ESP32-S3 radio, its undocumented capture/DAC engine and the
toolchain that matters when you build on the `esp_sdr` library, found while
developing the apps. Each entry says what happens, how it shows up, and where
the code deals with it. Link measurements: two XIAO ESP32S3 boards through a
30 dB attenuator; C6 entries: one XIAO ESP32C6.

## Radio

### The forced receive gain gets lost

**What happens.** A manual receive gain (`esp_sdr_rx_set_gain()`, forced
through the vendor blob's `force_rx_gain()`) is occasionally lost after
minutes of traffic, on either board. The Wi-Fi AGC then takes over. With
nothing on air it winds the gain up, and the quiet noise floor rises by up
to 30 dB.

**How it shows up.** Carrier sense reads a busy channel all the time. A
sender waiting for an ACK never transmits again. A receiver clips strong
frames and decodes bursts of corrupted headers. It looks like hardware
damage, but re-applying the gain cures it at once. The cause is unknown,
and comparing the AGC force register (`0x6001c02c`) does not detect it:
`force_rx_gain()` writes other registers too.

**How it is handled.** The library re-forces the gain at a capture when the
last force is more than 20 ms old (`esp_sdr_rx_gain_refresh()` in
`lib/esp_sdr/src/esp_sdr_rx.c`, `GAIN_REFRESH_MS`). Each re-force costs 14 to
51 us. `esp_sdr_get_stats()` counts them, and `link status` prints them on
its engine line. A turnaround back to receive also forces the gain.

When debugging this kind of fault: any `link set` in `apps/link` re-applies
the whole radio configuration, which hides it.

### The C6 keeps the previous firmware's modem state across a chip reset

**What happens.** A chip reset of the ESP32-C6 (through USB-Serial-JTAG, as
esptool and every port open do) leaves its modem subsystem as the previous
firmware left it. ESP-IDF resets it on a software restart
(`esp_system_reset_modules_on_exit()`), a chip reset does not. After
`apps/wpan` had run the C6's native 802.15.4 radio, `esp_sdr` on the next
firmware received with a noise floor about 25 dB up (rms 40 to 50 instead of
2 at fixed gain 40, the same at every gain index and frequency), and the
vendor's channel tuning put the PLL MHz off (2404 instead of 2412 MHz in its
fractional divider). A power cycle or a deep sleep cleared it.

**How it shows up.** Tones from another board land at the wrong offset or
not at all at Wi-Fi channel frequencies, and captures look like a much
noisier receiver. `tests/regression/lo_tuning` failed its link check whenever
it ran after `tests/regression/ieee802154_link`.

**How it is handled.** On the C6 the library resets the modem subsystem at
`PRE_KERNEL_1`, before the Wi-Fi driver brings the radio up, as ESP-IDF's
restart path does (`esp_sdr_modem_reset()` in `lib/esp_sdr/src/esp_sdr.c`).

### 40 MHz mode widens the analog filters

The PHY's channel bandwidth mode (`esp_sdr_set_channel_bw()`, the second
argument of the blob's `set_chanfreq()`) selects the analog filter. In its
40 MHz mode (2) the TX and RX path passes +-18 MHz within +-3 dB end to end:
a 36 MHz OFDM frame decodes error free. The 20 MHz mode (0) cuts it off.
`apps/link` defaults to `CONFIG_APP_CBW=2`. The spectrum edge of a
single-carrier signal is a misleading measure of this, since it mixes the TX
filter, the RX filter and the pulse shaping. Measure the channel per
subcarrier instead (`ofdm rx -v`).

### The transmitter compresses before the DAC words clip

Wide multicarrier signals distort in the analog TX path long before the
10-bit DAC words reach full scale. At an RMS amplitude of 120 DAC units, 116
subcarriers drop to 16 dB MER; at 60 to 90 they reach 26 to 29 dB. The
digital clip counter stays at 0, and lowering the RX gain does not help,
which places it in the transmitter. The OFDM default is `amp 90`.

### Receive gain headroom

Through the 30 dB attenuator at TX step 4, RX gain 32 is the working point
and 44 saturates the receiver (MER 4 dB).

### MER ceilings

| Signal | MER |
|---|---|
| Single carrier QAM, 20 Mbaud, 80 MS/s | about 31.5 dB, with or without attenuation |
| OFDM 16 MHz, DAC at 80 MS/s | about 31 dB |
| OFDM, DAC at 40 MS/s, 16 / 22 / 26 / 30 MHz | 27 / 25 / 24 / 23 dB |

The MER ceiling rules out 256-QAM on OFDM (about 33 dB needed) and 1024-QAM
on a single carrier.

### Carrier and sample clock offset

The two boards differ by about 19 to 21 kHz at 2412 MHz (about 8 ppm), and
the sample clocks by the same ratio. Within a frame the estimate can be off
by a few kHz, so phase tracking must follow a frequency error, not only a
phase. `apps/link/src/ofdm.c` takes the residual offset from the phase step
between its two pilot symbols and tracks it with a second order loop
(`track()`, `RATE_GAIN`). The QAM receiver derives the sample clock offset
from the carrier offset.

### Looped frames are not phase coherent across copies

Phase noise over one loop period makes copies of a looped frame differ in
phase. Decode one contiguous copy (OFDM), or measure the phase step at the
wrap from the overlap of two copies (QAM, `j->r.wrap` in `qam.c`).

### Receiving at 40 MS/s

`ESP_SDR_RATE_40MSPS` captures go through the radio's decimation filter,
with the same MER as 80 MS/s and half the samples to process. The filter
spreads each symbol by about +-8 samples, so an OFDM FFT window that starts
only a quarter of the prefix after its beginning picks up the previous
symbol. `ofdm_rx_begin_period()` measures the window position from the
pilots' phase slope across the subcarriers and moves the window to the
middle of the prefix ("fine timing").

The DAC also runs at 40 MS/s (`esp_sdr_tx_loop_begin(ESP_SDR_RATE_40MSPS)`),
at about 4 dB less MER than 80 MS/s.

## Capture and DAC engine

The capture and DAC playback use the Wi-Fi MAC's undocumented dump engine
(layout from upstream esp-sdr, see `lib/esp_sdr/src/esp_sdr_priv.h`).

- **A bank is off limits while the engine owns it.** The engine writes or
  reads one 64 KiB SRAM bank, selected one-hot in `MAC_DUMP_USAGE`. While it
  does, the CPUs cannot reach any of that bank, so nothing else may live
  there. The linker snippets reserve the banks (see Memory); a link error
  "DRAM image overlaps the ... bank(s)" means the image grew into one.
- **Short captures go unreported.** The engine can stop early without an
  error. `capture_locked()` writes a sentinel into the first and last word
  and fails the capture if either survives. Checking every word cost 1.2 ms
  for a full bank.
- **Capture length.** At most 16380 samples (`ESP_SDR_SAMPLES_MAX`, a 14-bit
  count). A looped frame must fit twice into one window so that a whole copy
  is always present. That caps the bytes per received window: what matters
  is the payload bits per captured sample.
- **Circular capture.** With `DUMP_CTRL` 0x24000 the engine writes a
  16384-pair ring continuously and exposes its write index at 0x60033d60;
  switching the bank select while it runs splits the stream into gapless
  units. `lib/esp_sdr/src/esp_sdr_ring.c` (from upstream) rotates through
  banks 0 to 2 and proves continuity with sentinel windows: the switch must
  come within 2000 pairs (125 us at 16 MS/s) of its threshold, and a bank
  must be filtered before the engine comes round to it again.
- **The Wi-Fi MAC interrupt must not run inside the ring.** The ring masks
  interrupts on its CPU and opens short windows for the kernel timer and
  IPIs (`irq_window()`). CPU interrupt line 0 is the Wi-Fi MAC's
  (`ETS_WMAC_INUM`), and it fires for every packet the Wi-Fi receiver decodes
  at the tuned frequency, hundreds of times a second on busy air. Its handler
  is far longer than a window, so the ring abandoned units until the run
  failed: the receive path then lost whole frames, in bursts that came and
  went with the neighbours' traffic and flipped with unrelated code changes.
  `esp_sdr_ring_run()` now disables line 0 on its CPU for the run and serves
  the interrupt afterwards. `ring irq windows` and `lines` in `wpan status`
  show how many windows ran and which lines were pending (0x40 is the tick
  timer; 0x41 would be the MAC). With the fix, abandoned units stay at 0 even
  with the C6 sending back to back.
- **Loop playback wraps only while the trigger is held.** Loop playback
  (`esp_sdr_tx_loop_*()`) holds the trigger bit; pulsing it plays one pass.
  Retriggered bursts (`esp_sdr_tx_play_for()`) leave a hole of about 1.35 us
  each.
- **Turnaround.** The default TX/RX switch with a retune takes about 3 ms.
  `esp_sdr_set_turnaround()` with 50 us and no retune works for packet
  links.
- **Analog I2C must take the PHY's lock.** The vendor PHY serializes its
  analog I2C traffic with `regi2c_enter_critical()`. Every `rom_chip_i2c_*`
  sequence in the library takes it too, because on two cores the Wi-Fi PHY
  code may run one at the same time.

## Memory

On the S3 the engine's 64 KiB banks and the `ESP_SDR_HIGH_RAM` region sit at
the top of internal DRAM, laid out by the linker snippets
`lib/esp_sdr/esp_sdr_bank*.ld` (picked in `lib/esp_sdr/CMakeLists.txt`). Bank 2
(0x3fcd0000) is always the capture bank. `CONFIG_ESP_SDR_BANK1`, which
`ESP_SDR_TX_DAC` and `ESP_SDR_RING` select, adds bank 1 (0x3fcc0000) as the
second DAC and capture bank, and the ring takes banks 0 to 2 (from
0x3fcb0000). Each snippet lowers `_heap_sentry` to its lowest bank, so the libc
heap stays below it, and fails the link if the DRAM image grows into a bank
(see "A bank is off limits" above). `ESP_SDR_HIGH_RAM` places buffers from the
capture bank's end up to `DRAM_USER_END` (0x3fce9704).

- **Internal RAM is the scarce resource.** Besides the banks, the region
  above the capture bank (`.esp_sdr_high`, `ESP_SDR_HIGH_RAM`) holds `apps/link`'s
  QAM transmit context and OFDM receive context, with about 128 bytes left.
  "high buffers beyond user DRAM" means it is full. Large buffers that are
  not timing critical go to PSRAM (`EXT_RAM_BSS_ATTR`).
- **Why past `user_dram_end`.** Zephyr's `user_dram_end` (0x3fce4f00) only
  protects MCUboot's loader while the image loads; the ROM's download-mode
  buffers above it are free at runtime, so NOLOAD buffers can use the whole
  range (about 38 KiB).
- **Three banks for the ring** move the DRAM limit to 0x3fcb0000.
  `apps/osmosdr` and `apps/wpan` fit by trimming the network stack and moving
  buffers and stacks into the high region (their READMEs). The Wi-Fi driver
  peaks at about 15.5 KiB of kernel heap with its default receive buffers,
  12 KiB with 2 (`doc/multiapp.md`).
- **Not zeroed at boot.** Neither `.esp_sdr_high` nor PSRAM `.bss` is
  cleared. `apps/link` clears what needs it at init (`link_mac_init()`).
- **PSRAM is slow for strided or per-sample work.** RS units and payload
  bytes may live there. Hot buffers (FFT work buffers, the RS codeword being
  decoded, which goes on the stack) should not.
- **Compare-and-set is not atomic in PSRAM.** S32C1I on external RAM lets
  both CPUs succeed: 2 x 200000 `atomic_inc()` from the two CPUs on one PSRAM
  counter lost counts. A spinlock, `k_msgq`, `k_mem_slab` or `atomic_t` in
  PSRAM breaks under SMP. Espressif's HAL emulates it
  (`esp_cpu_compare_and_set()`), and so does the Zephyr branch `west.yml`
  pins ("soc: espressif: esp32s3: emulate atomics in PSRAM"): every atomic in
  the PSRAM range goes through a lock in internal RAM.
- **No flash access from a thread whose stack is PSRAM.** A flash read or
  write (the flash API, so also MCUboot's `boot_is_img_confirmed()`) parks
  the other CPU and suspends the cache, and PSRAM goes with it: the thread's
  register window spills read back as 0xbad00add, and it double-faults with
  the other CPU still parked. Before the watchdog is armed (at boot, before
  USB starts) the board then hangs for good. A multiapp app's `main()` runs on
  a PSRAM stack, which is why the prelude turns its `app_usb_init()` into a
  call that does nothing.

## ESP32-C5 (receive only)

Measured on one Waveshare ESP32-C5-Zero (ESP32-C5HF4 rev v1.0, 48 MHz
crystal, 4 MB flash, no PSRAM), board `waveshare_esp32c5_zero/esp32c5/hpcore`
(`boards/waveshare/esp32c5_zero/`: console on USB-Serial-JTAG, 4 MB flash
with Espressif's partitions at the C5's 0x2000 boot offset, PSRAM disabled,
simple boot). Nothing was transmitted.

- **The capture engine is the C6's with a rate field.** Raw source 15 in
  `0x600a9008` bits 20:17, rate index in bits 23:21, a 17-bit count, lanes in
  `0x600a9018` (as librftest's `adctrig()`). The six indices 0 to 5 give
  80, 40, 20, 10, 8 and 4 MS/s: 16380 samples took 208, 412, 822, 1640, 2050
  and 4098 us, and the count field (`0x600a9008` bits 15:0) advances by 10
  samples per 2.5 us at index 5. `ESP_SDR_RATE_20MSPS` to `_4MSPS` are C5
  only.
- **The dump gate register stalls the bus.** Any access to `0x600a20b4` (the
  C6's dump source gate, in the Bluetooth baseband) takes 1.64 ms and reads
  0. Captures work without it, so the C5 build skips it; with it, every
  capture took 1.7 ms longer.
- **The owner bit takes a whole 128 KiB block.** `SRAM_USAGE` (`0x60095004`
  bits 11:8) = 2 with `MAC_DUMP_ALLOC` (bit 16) puts the samples at
  0x40830000, 64 KiB into block 1, but the CPU loses all of block 1
  (0x40820000 to 0x4083ffff): reads return the last word it read there,
  writes are dropped, nothing faults. Blocks 0 and 2 stay with the CPU.
  Block 2 holds the ROM's data, so block 1 is the only capture bank.
- **The image is split around the bank.** `esp_sdr_bank_c5.ld` is a
  `RAM_SECTIONS` snippet: code and initialized data link below 0x40820000,
  the bank takes block 1, and every section linked after the snippet
  (logging and kernel object data, the rest of `.noinit`, `.bss`, the libc
  heap) moves to block 2. The network stack's and the Wi-Fi driver's
  `.noinit` go below the bank to balance the halves. Margins with the Wi-Fi
  driver's defaults: `apps/capture` leaves 8.3 KiB in block 0 and 21 KiB of
  libc heap in block 2; `apps/rfprobe` 4.9 KiB and 20 KiB. "code and data
  overlap the capture bank" means block 0 is full; "will not fit in region
  `sram0_0_seg`" means block 2 is.
- **Tuning.** `phy_set_chanfreq()` takes any MHz above 14 and runs a full
  channel setup, including 5 GHz. Wi-Fi bursts received with the LO off the
  channel grid (2433, 2444, 5190, 5200 MHz) centre on 2437, 5180 and 5220 MHz
  within one 0.31 MHz bin. The PHY takes its 5 GHz path from 4500 MHz up.
  Below about 1100 MHz the PLL's SDM word goes negative, so those requests
  tune nothing. No PLL lock or VCO window flag is known on the C5, so
  `esp_sdr_set_freq()` never returns -ERANGE there.
- **Receive DC grows with gain, and the AGC idles high.** The largest
  calibrated gain index is 77 at 2412 MHz and 83 at 5500 MHz. With a forced
  index up to 60 the DC stays within about 10 of 512. At 65 to 77 it reaches
  20 to 50, and at 5500 MHz with index 77 it reaches 78. Under the AGC
  (`ESP_SDR_RX_GAIN_AUTO`) it measured +8 to +230 on I at 2412 MHz, varying from boot
  to boot. Force a gain for measurements.
- **The antenna switch.** GPIO26 drives the board's RF switch (200 kOhm
  pull-down, so the chip antenna while the pin floats).
  `CONFIG_WAVESHARE_ESP32C5_ZERO_EXT_ANTENNA` drives it high at boot
  (`board.c`), which selects the U.FL, as `CONFIG_XIAO_ESP32C6_EXT_ANTENNA`
  does on the XIAO C6; `capture` and `rfprobe` set it for the coax bench. At
  gain 60 over 2412 to 2472 MHz, low gave ambient Wi-Fi up to -1 dBFS
  (clipping), high at most -16 dBFS with the U.FL on coax and -30 dBFS with
  it open. The library does not drive the pin.
- **No ring and no transmit.** The C5 build has neither; `ESP_SDR_RFTEST`
  links the C5 librftest only for `force_rx_gain()`.

## CPU and toolchain

- **No hardware float divide.** A float `/` calls `__divsf3`. In hot loops,
  compare ratios by cross multiplication (`sc_scan()` in `ofdm.c`). An
  integer `%` by a constant that is not a power of two is a real division
  too; use a power-of-two size and a mask.
- **`lrintf()` is a library call**, out of line in flash. Two per sample in
  the OFDM transmit path cost about 7.5 ms per frame. Round inline (`emit()`
  in `ofdm.c`).
- **Many accumulators spill.** Loops with four or more running sums get
  them spilled to the stack (about 100 cycles per step for the sliding
  Schmidl-Cox sums). Do less work per step instead: the OFDM search scans
  coarsely first.
- **Contraction is off.** Zephyr's ISO C mode disables fused multiply-adds.
  `apps/link` turns them on for its DSP files (`-ffp-contract=fast`).
- **Hot code belongs in IRAM** (`IRAM_ATTR`): flash code goes through the
  cache, which the other core and PSRAM traffic contend for.
- **esp-dsp's S3 FFT kernel assembles empty without the target macro.**
  `apps/link` builds only `dsps_fft2r_fc32_aes3_.S` from the `esp-dsp` west
  project, with `-DCONFIG_IDF_TARGET_ESP32S3=1` (hal_espressif's
  `sdkconfig.h` lacks it), and its own twiddle table in esp-dsp's layout (cos,
  sin, bit reversed; one table built for N serves every smaller size). From
  flash it is still 1.76 times faster than the same algorithm in C from IRAM
  (85.7 against 150.6 us for 256 points). The S3 mask ROM has the twiddle
  table, but no FFT code.
- **The ring CPU runs with interrupts masked except for short windows.** The
  ring runs on CPU 1 under `arch_irq_lock()`: on SMP, `irq_lock()` takes a
  global lock and would stall the other CPU. Nothing may be pinned to that CPU meanwhile,
  no flash writes may happen (code from flash would stall), and the vector
  registers it uses are not part of any thread context.
- **The ring filter's budget** at 16 MS/s is 15 cycles per pair (240 MHz):
  unpack 3.0, the fs/4 mix 1.6, stage 1 4.5, stage 2 2.6 (decimation 64),
  the app's packing about 1. The multiply-accumulate chain on the single
  `accx` accumulator, not the instruction count, limits stage 1.
- **RS(255,223) decoding cost.** About 49 us for a clean codeword, 220 us
  with 2 errors and 400 us with 10. With many corrections, RS decoding
  dominates a receiver.

## Building and flashing

- Every S3 app except `apps/capture` needs `west build --sysbuild` (MCUboot,
  USB DFU). A plain build of an existing sysbuild directory fails with a
  cached-source mismatch.
- Kconfig values passed with `-D` stay in the CMake cache. An old override
  (for example `-Dlink_CONFIG_APP_SPS=8`) persists until set again.
- Update over USB DFU with `west dfu -d <build dir> [-s <usb serial>]`.
  The signed image grows in steps of 64 KiB (flash MMU page alignment), so
  its size is no evidence of a new build.
- Host udev rules match USB product IDs: a build with a new PID cannot be
  opened (or DFU-detached) until the rule covers it. The apps share
  2fe3:0005; `west dfu` finds only 2fe3:0005 (app) and 2fe3:ffff (DFU).
- A shell port serves one process: a host tool holding it (such as a running
  `link_perf.py run`) makes another tool's open of the same board fail.
- An S3 image put on by DFU must start USB OTG (`app_usb_init()` in
  `apps/common`, which also arms the trial watchdog) or clear
  `RTC_CNTL_SW_HW_USB_PHY_SEL` and `RTC_CNTL_SW_USB_PHY_SEL` early in boot.
  Otherwise it stays off USB until a power cycle: the OTG driver sets both
  bits and a software reset keeps them.
- esptool reaches an S3 whose app owns OTG once the app's shell command `usj`
  has handed the port to the USB-Serial-JTAG (`apps/common`); its reset at
  the end boots the app, which takes the port back. Hence `west dfu
  --esptool` writes MCUboot, the image and multiapp's volume in one esptool
  run: a sysbuild `west flash` runs esptool once per image, and the first
  one's reset leaves the second without a port.
- Gate an update on the build's exit status (a pipe into `grep` hides it),
  else `west dfu` flashes the previous image.

## Debugging

- **JTAG on an image that owns OTG.** On a running app, `usj` takes the USB
  device off the bus and selects the USB-Serial-JTAG: the PC sees the app go
  and the USB-Serial-JTAG come, and OpenOCD and esptool work until the next
  boot of the app. On a hung one, a shutdown handler (or a `PRE_KERNEL_1` hook
  in a diagnostic build) that clears the two PHY select bits above hands the
  PHY over at the next restart; the PC sees no disconnect then, and a
  `USBDEVFS_RESET` ioctl on the port's `/dev/bus/usb` node makes it
  enumerate.
- **A reset with no crash record** (`app crash`, `link status`) is the TG0
  watchdog's first stage: a CPU wedged with interrupts masked.
- **Stacks.** With `CONFIG_INIT_STACKS`, `kernel thread stacks` after
  exercising every path; a measured peak up to 80 % of the size is enough
  margin. An overflowing shell stack shows as 100 % on the logging stack below
  it. Xtensa frame sizes are in the disassembly as `entry a1, N`.
