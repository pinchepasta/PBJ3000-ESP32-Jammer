# Sending ACKs from the software radio

Built (2026-10-12): the S3 acknowledges frames to its address from inside the ring run, 192 us after their end in more than 99 % of the cases. "Built" below says how; the plan and the phase 0 measurements that led there follow it. The S3 software radio receives 802.15.4 frames and the ACKs to its own frames ([ieee802154.md](ieee802154.md)), but it cannot send an ACK in time. Zigbee and Thread need one for every unicast frame to the node that has the AR bit set: without it the sender retries three times and reports the frame as lost, so the S3 cannot even join a network as an end device (the association response from the parent asks for an ACK). This page plans an ACK path that meets the 802.15.4 deadline and lists what has to be measured first.

## The deadline

- An Imm-Ack starts aTurnaroundTime after the last symbol of the frame it acknowledges: 12 symbols, 192 us.
- The sender waits macAckWaitDuration = 54 symbols (864 us) from the end of its frame for the whole ACK: 20 (backoff period) + 12 (turnaround) + 10 (SHR) + 12 (PHR and the 5-byte frame). The ACK lasts 22 symbols (352 us), so a sender that follows the formula accepts an ACK that starts up to 32 symbols, 512 us, after its frame.
- The binding limit is the peer's. Neither the Espressif HAL nor Zephyr's ESP32 driver sets the ESP32-C6's ACK timeout (`IEEE802154_ACK_TIMEOUT_REG`, 16 bits in units of 16 us), so the C6 runs on the hardware default; `esp_ieee802154_get_ack_timeout()` reads it. Whether the C6 needs the start or the end of the ACK inside that time is found by the offset sweep in phase 5.
- Target: start 192 us after the frame. Earlier is not better (the sender may still be turning its receiver on); later is accepted up to the peer's limit and counted; an ACK that would start beyond the limit is skipped.

## Where the time goes today

| Stage | Today | Where |
| --- | --- | --- |
| Ring hand-over | The CPUs cannot read a bank the engine owns, so the PHY sees a sample only after its unit is finished: units of 12288 pairs, 768 us at 16 MS/s. | `THRESHOLD`, `lib/esp_sdr/src/esp_sdr_ring.c` |
| PHY lag | The PHY needs about 52 of the 60 cycles it has per sample, so it takes about 0.87 unit lengths to work through a unit: about 670 us. | [ieee802154.md](ieee802154.md) |
| PHY thread | Priority 14, the lowest application priority, on CPU 0: below USB and network, level with the shell and logging threads; sleeps 500 us whenever the sample ring is empty. | `rx_thread_fn()`, `CONFIG_ESP_SDR_IEEE802154_PHY_PRIORITY` |
| Frame check | Bitwise CRC over the whole PSDU once the last byte is in: tens of microseconds for a long frame (estimate). | `crc16()`, `lib/ieee802154/ieee154_phy.c` |
| Ring stop | `radio_take()` waits 1 ms for the lock before its first stop request. The ring looks for a request once per unit, at its middle, and stops at that unit's end: 0.4 to 1.15 ms later. | `radio_take()`, `ring_loop()` |
| Transmit start | The DAC generator starts a session 1 ms after the call and plays from two banks in turn. | `leader()`, `lib/esp_sdr/src/esp_sdr_tx_dac.c` |
| Front end | RX to TX: vendor calls, TX gain, LO select (analog I2C), force TX on, 50 us settle. TX to RX: vendor calls, the forced gain (14 to 51 us), LO select and filter code (analog I2C), 50 us settle. Not timed as a whole yet. | `esp_sdr_tx_prepare()`, `rx_front_end()` |

In all: milliseconds against 192 us. With the Zephyr L2, a driver without `IEEE802154_HW_RX_TX_ACK` gets its ACKs sent by the L2 through `tx()` (`ieee802154_acknowledge()`); on this driver that ACK leaves milliseconds late and stops reception while it does.

The delay is hand-over and start-up, not computation. None of the radio's fixed-function blocks surveyed for acceleration (power and DC estimator, CCA counters, other dump sources) is on this path; one appears below as an optional way to save ring overhead.

## Built

With `CONFIG_ESP_SDR_IEEE802154_AUTO_ACK` (default with the ring receive) the driver reports `IEEE802154_HW_RX_TX_ACK` and `IEEE802154_HW_FILTER`:

- **Ring** (`esp_sdr_ring_cfg::spare_bank`, `esp_sdr_ring_burst_*()`): units of 1024 pairs, rotation through banks 0 and 1 only, so bank 2 is the CPUs' for the ACK waveform for the whole run. Armed with the frame's end pair, the run builds the rest of the waveform between its polls, stops at the first unit boundary at or after the end, hands the last unit to the PHY, switches the front end to TX without the settle wait, plays bank 2 once when CPU 0 fires (or drops it on a cancel or at the limit), switches back to RX and restarts the capture. The pair index keeps counting real time over the gap, so the PHY resets there as at any gap.
- **PHY** (`ieee154_phy_set_hdr_cb()`): reports the length and the predicted end at the PHR and each further byte.
- **Driver:** decides from the frame control, PAN ID and destination address (`filter()`, not in promiscuous mode) whether an ACK is due; arms with the sequence number; fires `ack_delay` (192 us) after the end when the FCS passes, cancels when the PHY is past the end without a verdict. The first 416 slots of every ACK (preamble, SFD, PHR, frame control low nibble) go to bank 2 before each run. While armed the PHY thread does not sleep and runs at `CONFIG_ESP_SDR_IEEE802154_ACK_PRIORITY`. Not done: Enh-Ack (frame version 2), frames without a destination address to a PAN coordinator, the frame pending bit (always 0).
- **wpan:** `wpan set ack <delay>[,<limit>]`, `wpan bench acktx [seq]` (one ACK on the air now), counters in `wpan status`.

Measured with the C6 sending to the S3 (`wpan tx 200 10 <payload> 0x0001 1`, the C6 reports ok when its hardware got the ACK), coax with 30 dB:

| Case | Result |
| --- | --- |
| Channel 15, payloads 8, 16, 50, 100 bytes | 200 of 200 ACKs each |
| Channels 11, 20, 25, payloads 8 and 100 | 197 to 200 of 200; each miss a frame the S3 did not receive (the PHY's sample skips, see phase 0) |
| Promiscuous, broadcast, another address | no ACK sent |
| Start after the frame's end | 192 us for 397 of 400; the rest up to about 350 us |
| Verdict (FCS checked) after the end | average 115 to 125 us, at most 150 to 226 us |
| Front end in TX after the end | 86 us typically, at most 208 us (waveform still being built after the stop for short frames) |
| Capture running again after the end | about 670 us (the ACK ends at 544 us) |

Steps that got there: the front end's 50 us settle no longer delays an ACK (late ACKs 35 of 400 went to 0); the PHY thread's raised priority while armed (34 to 17 of 400 before that). The interop test (frames both ways, the S3's own ACK requests) passes as before on the two-bank ring.

Open: the S3 receiving ACKs to its own frames gets 49 of 50 per channel since the 1024-pair units (50 of 50 before); the PHY's sample skips on CPU 0; the TX gain call (38 us) on every ACK (skip it when unchanged, if the forced gain survives the RX phase); the internal RAM is nearly used up in the standalone wpan image (the ISR stacks went from 1280 to 1024 bytes for it).

## Phase 0 results (2026-10-12)

Measured on the XIAO S3 against the C6, coax with 30 dB between them, channel 15 unless noted. The code for them is in the tree, uncommitted at the time of writing: the ring's unit length as a run parameter (`esp_sdr_ring_cfg::unit_pairs`, guards derived at run start), `esp_sdr_ring_now()`, `ieee154_phy_rx_position()`, the driver's verdict counters, `wpan set unit` and `wpan bench ack`.

- **The C6's ACK timeout is 1728 us** (`wpan status` on the C6: 108 units of 16 us, the hardware default). Far more lenient than the 864 us of the standard; other peers will not be.
- **Verdict latency** (frame's last sample to the PHY's frame callback, from the ring's live pair index), 200 frames of 16 bytes:

  | Unit (pairs) | min | avg | max (us) |
  | --- | --- | --- | --- |
  | 12288 | 751 | 917 | 1067 |
  | 4096 | 272 | 522 | 865 |
  | 2048 | 146 | 273 | 829 |
  | 1024 | 84 | 104 | 177 |

  With 100-byte frames on channels 11 to 25 at 1024: average about 118 us, maxima 135 to 414 us (the tail is CPU 0 contention, see below).
- **The ring at 1024-pair units** keeps up: 8.5 cycles per pair (subsampling sink included, budget 15), worst switch 60 to 137 pairs late (limit 896), no unit abandoned over minutes. This needed a faster subsampling path: 9.3 cycles per pair with an empty sink before (every store reloaded the run state, the build does not assume strict aliasing), 3.05 after. The driver now runs 1024-pair units always: with the faster path, 12288-pair units arrive as one burst larger than the PHY's "fell behind" margin, and 73 of 200 frames were lost that way.
- **CPU 0 contention:** the PHY (87 % of CPU 0, lowest priority) still skips about 0.2 % of the samples while idle and a burst at each ring restart. With 1024 units interop is 197 to 200 of 200 per direction and 48 to 49 of 50 ACKs received, slightly below the phase 1 bar; a skip at a frame end is also a lost ACK. Open.
- **ACK waveform**, scalar slot copies into the capture bank: chips 3.2 to 3.8k cycles (16 us); all 14100 words 60 to 63k cycles (**251 us**); from slot 416 on, the part that depends on the sequence number, frame pending bit and FCS, 25k cycles (105 us). About 85 cycles per 20-word slot.
- **Front end**, `esp_sdr_tx_loop_begin()` / `_end()` with the 50 us settle: RX to TX 143 to 156 us, TX to RX 140 to 158 us. Per call, in cycles at 240 MHz:

  | RX to TX | cycles | TX to RX | cycles |
  | --- | --- | --- | --- |
  | `rom_pbus_workmode()` | 1811 | `phy_stop_tx_tone(1)` | 4512 |
  | `rom_pbus_xpd_rx_on(0)` | 753 | `force_txon_mode(0, ...)` | 365 |
  | `rom_pbus_xpd_tx_on(1)` | 1092 | workmode, xpd tx off, rx on, rxclk | 2533 |
  | `force_tx_gain()` | 9013 to 9511 | `force_rx_gain()` | 9767 to 11822 |
  | `tx_lo_select()` (I2C) | 2766 | `rx_lo_select()` (I2C) | 1720 |
  | `force_txon_mode(1, ...)` | 2227 | filter code (I2C) | 1412 |

  The two gain calls are half of it. A fast path that skips them when unchanged (to be checked: does a forced TX gain survive the RX phase, and the RX gain the TX phase?) and the tone stop (no tone runs in DAC mode) leaves about 35 to 40 us each way plus the settle time.

**Consequences for the design below.** At 1024-pair units the next unit's sentinel windows never overlap the unit still being filtered, so the ring needs only two banks. The third bank then holds the ACK waveform for good: preamble, SFD, PHR and the low frame control nibble (416 slots) built once, and per frame only the slots from 416 on patched in as soon as the header gives the sequence number (the frame pending bit and FCS again at the verdict if the bit changes). This replaces "the waveform into a free bank" in step 3, and with the slow scalar build (105 us for the tail) it is the variant that fits; a 128-bit copy would cut the tail further. Budget with the measured numbers: verdict at E + 84 to 177 us; the stop at the first boundary after E (0 to 64 us) plus the TX fast path (about 40 us plus settle) puts the transmitter ready by about E + 160 us; so the ACK can start at E + 192 us in most cases.

## Design

Three changes, each measurable on its own:

1. **Short ring units** while ACKs are enabled, so that the PHY sees the end of a frame tens of microseconds after it is on the air.
2. **An early decision.** The PHY reports the frame length and the header bytes as they arrive; the driver knows whether an ACK is due, its sequence number and when the frame ends, well before the end, and arms the ring.
3. **The ACK as a burst inside the ring run.** No stop request, no thread hand-over, no DAC session: CPU 1 ends the capture at the first unit boundary after the frame, switches the front end, plays a waveform it has already built in a free bank at a set time, and restarts the capture.

```
air     | preamble SFD PHR | MAC header ...  payload  FCS |E      F = E + 192 us
        |============== data frame (peer) ===============|-------|==== ACK, 352 us ====|
ring    |unit|unit|unit|unit|unit|unit|unit|unit|unit|unit|stop
PHY        (about one unit behind)  header: arm         CRC: fire or cancel
CPU 1                               ACK waveform -> free bank | front end to TX | DAC at F | RX, restart
```

### 1. Short ring units

- The unit length becomes a parameter of the run (`struct esp_sdr_ring_cfg`) instead of the `THRESHOLD` constant: 12288 pairs stays the default (the `stream` and osmosdr apps, the sniffer), the 802.15.4 driver asks for 1024 pairs (64 us) when auto-ACK is on.
- The guards follow from it at run start, with the checks the `BUILD_ASSERT`s make now: a unit must be longer than `START_GUARD`, which must exceed `LATE_LIMIT` + 64, and `END_GUARD` must exceed 2 `LATE_LIMIT` + 64. At 1024 pairs the lateness limit must stay below 960 pairs (60 us) instead of 2000; a smaller one keeps the sentinel windows short. `prep_pairs` (budgeted at 20000 cycles) is recomputed from the measured sentinel fill.
- Work per unit at 1024 pairs (estimate): binary searches in the windows (under 1 us), about 1500 to 3000 sentinel words (5 to 10 us with 128-bit stores), the subsampling sink (about 1 cycle per pair plus the sample stores, under 10 us). That fits in 64 us; what has to be checked is the lateness: the forced-gain refresh (14 to 51 us, every 20 ms) runs right after a switch, which leaves 64 us plus the lateness limit until the next deadline, and the interrupt windows add to it.
- The ring publishes its absolute pair index (current unit start plus the live write index) for CPU 0: the driver turns a frame end in PHY samples into a pair index (output sample k is pair 4k), and the same reading measures the PHY's real latency without a second clock.

### 2. PHY and driver: the early decision

PHY (`lib/ieee802154/ieee154_phy.c`, `include/esp_sdr/ieee154_phy.h`):

- A header callback, called after the PHR (length, and the sample index at which the frame will end: the PHR's end plus 128 samples per byte) and after each further PSDU byte until the driver returns false. Either phase tracker can call it; both may lock on one frame.
- The frame callback also gets the sample index of the frame end, to match the arm and for timestamps.
- CRC updated per byte (table driven), so the check costs nothing after the last byte.
- A cheap "in frame" query, so the thread polls instead of sleeping while a frame is in progress.

Driver (`lib/ieee802154/ieee154_esp_sdr.c`):

- `filter()`: PAN ID, short and extended address (`apps/wpan` already calls it with the PAN ID and short address and gets `-ENOTSUP` today); `configure()`: `IEEE802154_CONFIG_PAN_COORDINATOR`, `AUTO_ACK_FPB`, `ACK_FPB` (a small table of addresses with pending data), promiscuous.
- An ACK is due for a data or command frame with AR set, frame version 0 or 1, the destination PAN ID ours (or 0xffff) and the destination address ours; without a destination address only as PAN coordinator. Never in promiscuous mode: the sniffer stays passive. Frames of version 2 (802.15.4-2015) would need an Enh-Ack: not sent, counted.
- When the header says an ACK is due: arm the ring burst with the frame end as a pair index, the sequence number, and the frame pending bit if already known.
- Frame pending bit: with `AUTO_ACK_FPB` off, Zephyr asks for it set in ACKs to data requests; with it on, for listed source addresses. A data request is recognized by the command byte after the addresses, which in the shortest frames (a 12-byte data request lasts 576 us from the first preamble symbol) reaches the PHY only around the frame end. The bit is therefore decided at the verdict at the latest, and the waveform is built in both variants when it is still open (see below).
- On a CRC pass: fire at E + 192 us. On a CRC failure, or when the arming tracker resets and its sibling does not finish the frame: cancel.
- PHY thread: no 500 us sleep while a frame is in progress or a burst is armed, and a raised priority while armed (lowered again afterwards), so that USB, logging and the network stack on CPU 0 cannot sit between the last samples and the verdict.
- Capabilities, with `CONFIG_ESP_SDR_IEEE802154_AUTO_ACK` (default off until phase 3 passes): `IEEE802154_HW_RX_TX_ACK` (the L2 then stops sending its own) and `IEEE802154_HW_FILTER` (the L2 then hands over the addresses).
- Counters: ACKs sent, sent late (after 192 us but within the limit), skipped (beyond the limit), cancelled (CRC failure), arms too late; turnaround minimum and maximum from the burst's own timestamps.

### 3. The ACK as a burst inside the ring run

Library (`lib/esp_sdr/src/esp_sdr_ring.c`, front end in `esp_sdr_tx.c` and `esp_sdr_rx.c`). A sketch of the interface, generic so that the link app's ACKs could use it later:

```c
/* Arm a burst: end the capture at the first unit boundary at or after end_pair,
 * and build the waveform with gen() into a free bank meanwhile. */
int esp_sdr_ring_burst_arm(uint64_t end_pair, esp_sdr_tx_dac_gen_t gen, void *user,
			   uint32_t words, enum esp_sdr_rate rate);
/* Play it so that its first sample leaves at at_pair (the ring's time base). */
void esp_sdr_ring_burst_fire(uint64_t at_pair);
/* Drop it: the capture goes on, or restarts if it has already stopped. */
void esp_sdr_ring_burst_cancel(void);
/* The ring's absolute pair index now, from any CPU. */
uint64_t esp_sdr_ring_now(void);
```

Sequence on CPU 1 (interrupts masked, as for the whole run):

1. Armed: build the waveform between polls, in bounded slices like the filter slices, into a bank the rotation does not need before the end: the bank of the unit before the final one, free once that unit has gone to the sample ring.
2. At the first boundary at or after the frame end: switch as usual, stop the engine, pass the final unit on (the PHY needs it), and switch the front end to TX at once, before the verdict. After the end of the frame nothing more is expected from the sender, and the switch overlaps the PHY's last unit on CPU 0.
3. Finish the waveform if needed (the settle time is spent on it instead of in a busy wait). When the frame pending bit is still open, build the second variant in the other free bank (two banks are free after the stop).
4. Wait for fire or cancel. On fire: select the bank and start one DAC pass when CCOUNT reaches the fire time (the stop recorded CCOUNT against the pair index; 15 cycles per pair at 240 MHz). A fire after the limit is turned into a cancel.
5. Front end back to RX, restart the circular capture, continue the run. The output index jumps, and the PHY resets on the gap as it does today.

The waveform: SHR, PHR and the 5-byte frame are 11 bytes, 704 chips, 14100 DAC words at 40 MS/s with the existing slot templates (`ieee154_dsp_tx_gen()`): one DAC pass (at most 16380 words), from any of the three ring banks. It depends only on the sequence number and the frame pending bit: the first 416 slots (preamble, SFD, PHR, the low nibble of the frame control field) never change. The scalar generator copies 20 words per slot; at a guessed 40 to 60 cycles per slot the whole ACK takes 120 to 180 us of CPU 1, a 128-bit copy perhaps a third of that: measured in phase 0. Starting it at the arm hides most of it for long frames; for the shortest ones the arm comes about 50 us before the frame end, so the vector copy may be needed.

Front end: a fast path for a switch without retune that skips writes that did not change (TX gain, LO divider select, filter code), takes the settle times as parameters (the minimum found in phase 5) and runs on CPU 1 with interrupts masked: vendor calls and the analog I2C lock only, CCOUNT waits instead of `k_busy_wait()`.

Restart: the run's first unit fills all three banks with sentinels today (49152 words). After a burst only the windows around the expected start are needed.

### Optional: cut the unit at the predicted end

Stopping at the first boundary after the frame costs 0 to 64 us of waiting. If phase 3 misses 192 us, the ring ends the unit at the predicted end itself (`end_pair` plus a margin of a few microseconds for the receive chain), which needs end windows that also cover the shorter unit. Halving the unit length instead gains about as much, but at 512 pairs a worst-case gain refresh (51 us) plus the unit's own work no longer fits before the next deadline (32 us plus a lateness limit under 28 us).

## Estimated budget

Estimates for 1024-pair units, to be replaced by the phase 0 numbers. E is the end of the frame on the air.

| Step | Estimate |
| --- | --- |
| Wait for the next unit boundary | 0 to 64 us (0 to 5 us with the cut) |
| Switch, end search, final unit into the sample ring | about 10 us |
| PHY works through the last unit (0.87 x 64 us) | about 56 us |
| CRC verdict, decision, fire flag seen by CPU 1 | a few microseconds |
| **Verdict** | **E + 70 to 130 us** |
| CPU 1 from the stop: front end to TX, waveform finished | not measured; must end before E + 192 us |
| DAC start | E + 192 us when both are ready, else as soon as they are |
| ACK on the air | 352 us, ends at E + 544 us |
| Front end back to RX (forced gain 14 to 51 us, I2C, settle), capture restarted | about 100 to 200 us |

So the S3 does not receive from the stop until about 150 to 200 us after the end of its ACK. A sender with nothing to wait for (backoff 0) can start its next preamble about 140 us after the ACK (CCA of 8 symbols plus its turnaround), and the PHY needs 4 of the 8 preamble symbols: about 200 us. Tight, see Risks.

With 2048-pair units the verdict moves to E + 120 to 250 us: within 512 us, but not 192.

## Phases

| Phase | Work | Done when |
| --- | --- | --- |
| 0 | Measurements, no change of behaviour: the C6's ACK timeout (one line in the C6 build's `wpan status`); today's frame-end-to-verdict latency on the S3 (ring pair index at the frame callback minus the frame end); each front end call and the settle; the ACK generator's speed (a `wpan bench` entry); the ring at 4096, 2048 and 1024 pairs (late_max, abandoned, failures) on live air. | The numbers are in this page and the budget is updated from them. |
| 1 | Ring: unit length as a parameter, guards derived and checked at run start, absolute pair index for CPU 0; the driver asks for short units in ACK mode. | `interop.py` 200 of 200 on channels 11, 15, 20 and 25 at 1024-pair units; abandoned units 0 and ring status OK over 10 minutes. |
| 2 | PHY: header callback, frame end index, per-byte CRC. Driver: `filter()`, FPB configuration, the ACK decision with "would ACK" counters and the verdict latency per frame, no transmission yet; PHY thread polling and priority while armed. | Unit tests pass; on the bench the verdict lands within E + 130 us for payloads of 1 to 100 bytes. |
| 3 | Ring burst (arm, waveform into a free bank, stop at the first boundary, front end fast path, timed DAC pass, RX and restart); driver fire and cancel; capabilities and Kconfig. | The C6 reports ok for at least 99 % of frames to the S3 that ask for an ACK, payloads 1 to 100 bytes, on channels 11, 15, 20 and 25; no ACK in promiscuous mode, to broadcast frames or to another address. |
| 4 | Only if phase 3 misses 192 us: the cut at the predicted end; trims found in phase 3. | The burst's own timestamps put every ACK start at E + 192 us within a few microseconds. |
| 5 | Fire offset sweep (the C6's acceptance window), front end settle sweep, back-to-back frames, frames printed over USB meanwhile; an "S3 sends ACKs" section in `interop.py`; [ieee802154.md](ieee802154.md) updated. | Results recorded there and the open issue closed. |
| 6 (later) | Turnaround on one clock with the ZigBit's timer ([zigbit-reference.md](zigbit-reference.md)); frame pending table with a polling end device; the Zephyr L2 and OpenThread on top. | |

## Tests

- Unit, `tests/unit/ieee154_dsp/`: the ACK waveform through the existing chain (40 MS/s DAC words resampled to 16 MS/s, offset, noise, 10-bit words, mix and decimation, PHY) decodes for all 256 sequence numbers and both frame pending values, the prefix-plus-variant build equals a full build.
- Unit, `tests/unit/ieee154_phy/`: the header callback's length and end index for every frame length and both trackers; the per-byte CRC equals the old one.
- Bench, S3 and C6 on the coax with the attenuator as for `interop.py`: the C6 sends with `wpan tx <n> <ms> <payload> <S3 short address> 1` and reports `ok` or `noack` from its own hardware ACK check; the S3 sets `wpan set pan <PAN ID> addr <short address> promisc 0`. Cases: payloads 1, 8, 50 and 100 bytes; interval 0 (back to back); promiscuous on, broadcast, another address (all: no ACK); ring statistics in `wpan status` throughout.

## Alternatives considered

- **ACK before the CRC verdict:** rejected. An ACK to a damaged frame makes the sender drop a frame the S3 never got.
- **Short units only once the PHY sees a frame:** the PHY sees a frame up to about 1.9 long units (1.4 ms) after its SFD, later than the end of a short frame.
- **Unit length driven by a hardware energy detector** (the Wi-Fi AGC's power reading or the CCA busy counters, read by CPU 1 between polls, switching to short units when energy appears): long units while the air is quiet. Depends on registers not yet tested with the forced gain; a fallback if short units all the time cost too much.
- **The PHY on CPU 1 inside the ring loop:** no hand-over between CPUs and no preemption, but the PHY alone needs about 87 % of a CPU.
- **The BLE 2M modem as the 802.15.4 radio:** BLE hardware turns around in 150 us, but the controller is closed, and a BLE packet of at most 255 bytes holds only about 30 bytes of 802.15.4 frame.
- **Two-bank rotation with a reserved ACK bank:** the constant 416-slot prefix is built once and 289 slots per ACK; the lowest latency, but changes to the rotation, and no slack left for the gain refresh at 64 us units. The fallback if building the ACK in a free bank is too slow.
- **A stored waveform per sequence number:** 256 x 2 x 56 KiB.

## Risks and open points

- **Ring lateness at short units:** a limit under 60 us leaves little room for the gain refresh, interrupt windows and slices; a flash write on CPU 0 parks CPU 1 and ends the run. Phases 0 and 1 measure it; the refresh can move to the RX return after an ACK, which forces the gain anyway.
- **CPU 0:** the PHY needs about 87 % of it and runs below USB and network; the priority raised while armed has to be shown not to starve USB.
- **CPU left for a Zigbee or Thread stack:** the PHY's 87 % of CPU 0 plus the ring's CPU 1 leave little for a network stack. Separate from the ACK, but it decides whether a full node on the S3 is realistic.
- **Internal RAM:** IRAM comes out of the same SRAM as the DRAM image, and the multiapp image has about 6 KB left below bank 0. The additions to the poll loop must stay small; the burst's slow parts can run from flash.
- **Analog I2C:** the lock is shared with the Wi-Fi PHY on CPU 0; a long hold there delays the front end switch.
- **Receive and transmit chain delays:** E is measured in samples, not on the air; the fire offset absorbs the difference and is set by the sweep (later checked on the ZigBit's timer).
- **Back in RX in time** for a sender's next frame (see the budget); trimming the RX return is part of phase 3.
- **Out of scope:** Enh-Ack and CSL (802.15.4-2015, Thread 1.2), and the start delay of the S3's own frames through `tx()`, which delays a CSMA transmission but misses no deadline.
