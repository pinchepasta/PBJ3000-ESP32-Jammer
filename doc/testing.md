# Testing

Every pass/fail test is a twister test; all but the S3 variant of the integration test also run
end to end under twister (that one is deployed by hand, see below). Four tiers:

| Tier | Where | Runs on | Covers |
| --- | --- | --- | --- |
| Unit | `tests/unit/` | host (`native_sim/native/64`), no board | Portable code as ztest suites: the 802.15.4 PHY (`ieee154_phy`), the driver's signal processing (`ieee154_dsp`), link RS(255,223) (`link_rs`) and Hamming(128,120) (`link_hamming`), the `stream` app's I/Q packing (`iq_pack`). With `IEEE154_TABLES` set, `ieee154_phy` and `ieee154_dsp` also print their sweep tables (`scripts/unit-tables.sh`). |
| Integration | `tests/integration/` | one real board | ztest on the radio itself. `esp_sdr_rx_api`: the receive API's contract (error codes, bounds, one real capture per supported rate). Needs no antenna, peer or host tool and never transmits. |
| Regression | `tests/regression/` | two real boards | pytest through twister's multi-board harness. `ieee802154_link`: the C6 native radio and the S3 software radio exchange `apps/wpan` test frames both ways on channels 11, 15, 20 and 25, plus ACK requests. Guards against the two bugs that only ever showed as lost frames between real boards (`doc/hardware-quirks.md`). `lo_tuning`: the S3 sends tones to the C6 at overlapping LO frequencies, through the 5/6 divider and directly tuned; guards the transmit divider and `-ERANGE`. |
| Smoke | `apps/*/sample.yaml` | build, and one board | Every app builds for its boards; `apps/capture` also boots on the S3 and the C6 and must print `esp-sdr rx: ready` (console harness); `apps/multiapp` boots into an app from its file on the FAT volume (pytest, `sample.esp_sdr.multiapp.hil`). |

## Continuous integration

`.github/workflows/build.yml` runs on every push and pull request. It builds every app and test
for each board it allows (`xiao_esp32s3/esp32s3/procpu` and `xiao_esp32c6/esp32c6/hpcore`, one job
each) and builds and runs the unit tests on `native_sim` and `native_sim/native/64`. A GitHub
runner has no boards, so the integration and regression tests and the console smoke test are only
built there; run them on the desk as below. The twister reports of each job are kept as
artifacts.

## Unit tests

```
esp-sdr-zephyr/scripts/run-unit-tests.sh
esp-sdr-zephyr/scripts/run-unit-tests.sh -s unit.esp_sdr.link_rs    # one suite
```

The script runs twister on `native_sim/native/64` with `NSI_OPT=-O2`. Plain `native_sim` is
32-bit and this nix shell has no 32-bit glibc. Without an optimization level, glibc's
`_FORTIFY_SOURCE` check warns inside native_sim's own runtime code, and `-Werror` makes that
fatal.

`tests/unit/link_rs` builds `apps/link/src/rs.c` with `-U__ZEPHYR__`, so its host branch is
compiled instead of the ESP-IDF one.

### Sweep tables

`ieee154_phy` and `ieee154_dsp` each hold one more test case that is skipped unless the
environment variable `IEEE154_TABLES` is set (to anything): the PHY's packet error rate over SNR
and carrier offset (`test_per_table`), and the decoded fraction of the driver's chain through each
decimator over SNR (`test_decimator_table`; with the variable set, `test_cfo_estimator_accuracy`
also prints its carrier offset estimates). These are the tables `doc/ieee802154.md` quotes, and
what tuning the PHY or a decimator needs.

```
esp-sdr-zephyr/scripts/unit-tables.sh          # both
esp-sdr-zephyr/scripts/unit-tables.sh phy      # the PHY error rate table
esp-sdr-zephyr/scripts/unit-tables.sh dsp      # the decimator table
```

The script runs the two tests with `IEEE154_TABLES=1` and prints their logs. On an idle machine
each table takes between half a minute and a minute, plus the build; concurrent builds can triple
that, hence the generous twister timeouts of these ids (300 s and 900 s). The table cases also
assert what the documentation claims (`phy`: no frame lost from 6 dB SNR up, none twice, none with
wrong content; `dsp`: each decimator column reads 1.000 from 2 dB above the SNR where it first
does), after printing the whole table, so a regression fails the run and still shows the numbers.
Without the variable the cases are skipped and the unit tests stay at a few seconds.

## Integration tests (one board)

Each test has one id per board: `integration.esp_sdr.rx_api.c6` and `.s3`.

- C6: flashed directly with twister's device testing. Opening the C6's USB-Serial-JTAG port
  resets the chip, and the suite finishes before a plain `--device-serial` attach after the
  flash would see it: open the console after the flash so the reset reruns the suite
  (`--flash-before`), or use a `--device-serial-pty` script that resets the chip once it is
  attached.
- S3: the board normally runs an app that owns the USB OTG port, so esptool cannot reset
  it into the ROM bootloader. The S3 variant therefore builds with sysbuild and MCUboot and
  has its own CDC-ACM shell and DFU. Deploy it with `west dfu -d <build dir> -s <usb serial>`
  and read its CDC-ACM console. The suite sleeps 10 s after boot so the host can enumerate the
  port first: CDC-ACM does not replay output printed before a reader attaches.

## multiapp desk check (one board)

`sample.esp_sdr.multiapp.hil` builds the multiapp host with `hello` as its autostart app
(`apps/multiapp/hil.conf`). `scripts/twister-flash.py` puts the image on the S3 with `west dfu`
and, because the build has a FAT volume image, the volume with `west dfu --storage`;
`apps/multiapp/pytest/test_multiapp.py` then checks that every app's file is on the volume and
equals its built-in copy, that the boot ran `hello` from its file, and that hello's commands
answer, its PSRAM atomics included. From the repository root, with the S3 attached:

```
west twister -T apps/multiapp -s sample.esp_sdr.multiapp.hil --device-testing \
    --hardware-map tests/regression/hardware-map.yaml \
    --flash-command "$PWD/scripts/twister-flash.py"
```

About a minute, flashing included. It leaves the test image on the board: `west dfu` and
`west dfu --storage` of a normal multiapp build afterwards.

## Regression tests (two boards)

Two scenarios, both pytest through twister's multi-board harness, each with its own pair of
images:

- `tests/regression/ieee802154_link`: 802.15.4 frames between the C6's native radio and the S3's
  software radio.
- `tests/regression/lo_tuning`: tones from the S3 to the C6 at overlapping LO frequencies.

Each has three ids: `regression.esp_sdr.<test>.c6` and `.s3` only build the two images
(`build_only`); `regression.esp_sdr.<test>` runs `pytest/` on both boards (`required_devices` and
`required_applications`).

Run them with both boards attached:

```
esp-sdr-zephyr/scripts/run-regression-tests.sh -O <output dir>
```

For each scenario it builds the two images (the scenario id builds none of its own), puts them on
the boards and runs the tests, a few minutes in all; `-s regression.esp_sdr.lo_tuning` picks one.
A relative `-O` is relative to the repository root. What is specific to a desk is in
`tests/regression/hardware-map.yaml` (change the serial numbers for another desk), apart from
the radio levels of `lo_tuning` (below). Four pieces make twister cope with these two boards:

- `scripts/twister-flash.py`, passed as `--flash-command`. Twister calls it once per board with that
  board's build directory and its id from the map. An MCUboot image (the S3) goes on with
  `west dfu -s <id>`, anything else with `west flash` to the port of that id. Twister's own `west
  flash` (esptool) cannot reset an S3 whose app owns the USB OTG port. Both boards flash at once.
  Afterwards `/tmp/esp-sdr-tty/<id>` points at the board's port, found by serial number alone;
  the map names the S3's port by this link, because its `/dev/serial/by-id` name carries the USB
  product string of whichever scenario's image is on it.
- `flash_before: true` in the map: the console is opened after the flash, because the C6 resets when
  its port is opened and the S3's port goes away and comes back during the DFU update.
- `pytest_dut_scope: module` and a generous `timeout` in each scenario: both boards are flashed
  once for all tests, and the flashing counts against the timeout.
- `tests/regression/conftest.py` presses Ctrl-C on each shell before twister looks for the
  prompt. Twister presses Enter with a bare LF, which a Zephyr shell swallows after a CR
  (`process_nl()` in `subsys/shell/shell.c`), and `west dfu` ends its lines with CR LF, so the S3
  would never show a prompt. Pytest loads this conftest for both scenarios because twister runs it
  from its own working directory (the repository root with the script), which with no pytest ini
  file in the tree becomes pytest's rootdir.

Twister compiles with `-Werror`, a plain `west build` does not. With CDC-ACM as console and shell,
`usbd_cdc_acm.c` turns its own logging off with a `#warning`, which is fatal under twister. Every
S3 app with a CDC-ACM shell therefore sets `CONFIG_USBD_CDC_ACM_LOG_LEVEL_OFF=y` itself.

### ieee802154_link

`tests/regression/ieee802154_link` builds `apps/wpan` from its own directory: `src/main.c` by
path, and `APPLICATION_CONFIG_DIR` and `KCONFIG_ROOT` point Zephyr's config lookup there, so
`prj.conf`, `boards/` and `Kconfig` are the app's own and cannot drift. Only `sysbuild.conf` is a
copy, because sysbuild looks it up before the image's `CMakeLists.txt` runs.

What passes: on channels 11, 15, 20 and 25, in both directions, 100 test frames of 100 bytes at 5 ms
spacing. Every frame the transmitter completed must arrive, none corrupt or twice. A transmitter may
fail up to 3 of the 100 itself: the S3's DAC generator fills a block late about once in 400 frames
and cuts the frame (`-EIO`), and the C6 now and then reports a tx error of its own. That is not a
receive loss, which is what the test is for (the Wi-Fi MAC interrupt, `doc/hardware-quirks.md`,
and the sample ring generation race, `lib/ieee802154/ieee154_esp_sdr.c`, both showed as frames
the sender had sent that never arrived), so it is bounded loosely and the receiver is held to what
the transmitter reports as sent. The ACK check sends 20 frames with an ACK request from the S3 to
the C6 and allows 2 to fail or go unanswered. When a check fails, the message carries `wpan
status` of both boards.

`apps/wpan/tools/interop.py --native <C6> --sdr <S3> --channels 11,15,20,25` runs a similar exchange
by hand, with its own defaults (200 frames of 16 bytes, 10 ms apart) and, when a direction fails
completely, the I/Q inversion fallbacks.

### lo_tuning

`tests/regression/lo_tuning` is a small firmware of its own for both boards, with a `lo` shell
command: `lo f` tunes and prints `esp_sdr_set_freq()`'s return code, `lo tone on`/`off` loops a
tone on the S3 (loop playback, no gaps), and `lo rx` captures 8192 samples at 80 MS/s on the C6
and prints the strongest tone and the level at an expected offset. The C6 does the analysis
itself: a Welch average of eight 1024-point FFTs, each bin compared with the median of the bins
within 1 MHz, so the C6's uneven noise floor does not count as a tone. Every filter is open
(receive and transmit low-pass code 0). The C6 receives at fixed gain index 40 on its U.FL port
and the S3 transmits at its weakest step: on this desk the tones arrive 35 to 51 dB above the
floor without clipping. Another RF path (an attenuator, antennas) may need other `RX_GAIN` and
`TX_STEP` in `pytest/test_lo_tuning.py`. Before any other check the scenario measures the link at
2412 MHz, a Wi-Fi channel tuned the vendor's way, and stops with a "no usable link" message if the
tone is missing there.

A tone sent at TX LO + 3 MHz must show up at (TX LO - RX LO) + 3 MHz, at least 20 dB above the
local floor, the strongest one in the capture, within 150 kHz (the two crystals plus an FFT bin).
Equal LOs alone would miss two PLLs that both land on the same wrong frequency, so each frequency
is also checked with the boards 10 MHz apart, both ways round. The scenario checks:

- 1900 to 2610 MHz, in the divider band (PLL at 6/5) and directly tuned. Together with the pairs
  around 2210 MHz they keep the PLL between 2210 and 2651 MHz, the envelope upstream esp-sdr
  qualified on all chips.
- The switch at 2209/2210 MHz: divider on one board, direct tuning on the other.
- A 2000 MHz transmission is absent at 2400 MHz + 3 MHz, where it went out before the transmit
  path selected the divider.
- With the transmitter silent, nothing at the offset (the detector itself).
- `-ERANGE` on both boards at 1000, 1700, 3000 and 5000 MHz.
- No `-ERANGE` means the PLL locks. The S3's window edges are found by tuning MHz by MHz, from 1880
  MHz down and from 2610 MHz up, and must exist. At the last 3 MHz inside each end, wherever both
  boards tune without `-ERANGE` (the C6 8 MHz further in, or 16 if it reports `-ERANGE` itself), the
  tone must arrive; at least 2 of the 3 per end must tune. `-ERANGE` itself is conservative
  (`esp_sdr.h`) and is not checked for being reported too often.

## Other tools

Measuring tools, not pass/fail gates (the host tables are in the unit tests, above):

- `apps/rfprobe` and its `tools/rfprobe.py`: LO pair sweeps with a tone between two boards, raw PLL
  sweeps, PLL register scans, analog register dumps and diffs, the receive noise floor.
- `apps/wpan/tools/interop.py` and `iq_dump.py`: two-board interop runs and raw captures.
