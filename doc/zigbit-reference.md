# ZigBit reference radio

A plan, nothing built yet. The ATZB-X-233-XPRO (ZigBit ATxmega256A3U + AT86RF233) becomes a second native 802.15.4 radio next to the ESP32-C6 or -S3, to check the software radio in [ieee802154.md](ieee802154.md) against independent silicon and an independent vendor implementation.

The ZigBit is a radio coprocessor (RCP) behind a generic Zephyr board, at first a second XIAO ESP32S3, over UART or SPI. On the Zephyr side it is one more `ieee802154_radio_api` driver in `lib/ieee802154/`, next to the software radio, so `apps/wpan`, `tools/interop.py` and later the Zephyr L2 run on it unchanged.

## Test setup

```
 PC (USB) ── XIAO ESP32S3 "reference"  ──UART/SPI──  ZigBit (XMEGA + RF233)
               wpan,                                     │ RF test connector
               radio = RCP driver                        │
                                                    coax, 30 to 40 dB attenuator
                                                         │
 PC (USB) ── XIAO ESP32S3 "SDR"  ◄───────────────────────┘ antenna connector (U.FL)
               wpan,
               radio = esp_sdr_154 (software radio)
```

Both XIAOs run the same `wpan` app and differ only in the chosen radio. The reference S3 is the controller of the ZigBit and has no part in the RF path; its own Wi-Fi radio stays off. The only RF connection is the coax from the ZigBit to the SDR S3's antenna port, so what the SDR receives and sends is checked against the RF233 alone. `interop.py --native <reference S3> --sdr <SDR S3>` drives the pair. The ESP32-C6 stays available as a third radio on the same coax (through a splitter) or in place of the ZigBit.

## The board

From the ZigBit Extension User Guide (Atmel 42186C):

| Item | Detail |
| --- | --- |
| Radio | The AT86RF233 is wired to the XMEGA inside the module. The SPI pins on J100 are the XMEGA's SPID (PD4 to PD7); the "RF233 pin" column of the guide's J100 table applies to the radio-only boards. A host cannot reach the RF233 directly, so Zephyr's `rf2xx` driver does not apply: the XMEGA runs the radio. |
| UART | J100 pin 13 = PD3/TXD0 (ZigBit transmits), pin 14 = PD2/RXD0 (ZigBit receives). USARTD0, 3.3 V levels. |
| SPI | J100 pins 15 to 18 = PD4 SS, PD5 MOSI, PD6 MISO, PD7 SCK (SPID, slave mode). |
| Free lines | J100 pin 9 = PF0 (IRQ to the host), pin 5 = PA4 (GPIO1), pin 10 = PE2. |
| Reset | J100 pin 7, button SW1. |
| Power | 3.3 V on J100 pin 20 or J4 (reverse-protected). The J5 strap must be fitted (current measurement). |
| Programming | J2, the JTAGICE header; or the preprogrammed AVR2054 serial bootloader over the UART (SREC files, reset on request). |
| Shipped firmware | Bootloader and Performance Analyzer (driven by Atmel's Windows GUI, not scriptable). |
| Persistent memory | Factory MAC (EUI-64) and an XTAL calibration value. |
| LEDs, button | LED1 red PA6, LED2 green PA7, LED3 yellow PA5; SW2 on PF2. |
| RF | Chip antenna and an RF test connector. |

Not yet checked (needs the schematic or the ZigBit module datasheet): whether J2 carries PDI or JTAG, which XMEGA port and pins the RF233 uses inside the module, whether the test connector disconnects the chip antenna, and where the persistent memory lives (user signature row or flash).

## Wiring to a XIAO ESP32S3

`wpan` on the S3 has its shell on USB (CDC-ACM), so `uart0` (GPIO43/44) is free; `spi2` is on GPIO7 to 9.

| J100 pin | Signal | XIAO ESP32S3 |
| --- | --- | --- |
| 20 | VCC 3.3 V | 3V3 |
| 2, 19 | GND | GND |
| 14 | ZigBit RXD0 | D6, GPIO43, UART0 TX |
| 13 | ZigBit TXD0 | D7, GPIO44, UART0 RX |
| 7 | RESET (active low) | D0, GPIO1, open drain |
| 9 | IRQ, PF0 | D1, GPIO2 (SPI: required; UART: optional wake-up) |
| 15 | SS | D2, GPIO3 (SPI only) |
| 16 | MOSI | D10, GPIO9 (SPI only) |
| 17 | MISO | D9, GPIO8 (SPI only) |
| 18 | SCK | D8, GPIO7 (SPI only) |

Any other Zephyr board with a free UART or SPI and two GPIOs works the same way: everything board-specific is in the devicetree overlay.

## Architecture

```
 apps/wpan (wpan shell)                     unchanged; plus `wpan ref ...` when the RCP is present
        |
 ieee802154_radio_api                        DT_CHOSEN(zephyr_ieee802154) = the RCP node
        |
 lib/ieee802154/ieee154_rcp.c                Zephyr driver: radio API <-> RCP messages
        |
 lib/ieee802154/ieee154_rcp_proto.c          portable: message codec, framing, CRC (no Zephyr deps)
        |
 UART (framed) or SPI (+ IRQ line)
        |
 XMEGA firmware                              same ieee154_rcp_proto.c; RF233 driver; timestamps
        |
 AT86RF233                                   auto-ACK, CSMA-CA and retries in hardware
```

### What lives where

- **The radio timing stays on the ZigBit.** The RF233 does ACKs (RX_AACK), CSMA-CA and retransmission (TX_ARET) in hardware, so the 192 us turnaround never crosses the link. The host only needs millisecond-level latency, which a Zephyr L2 or OpenThread tolerates.
- **The protocol is portable C shared by both sides,** in the style of `ieee154_phy.c`: one header `include/esp_sdr/ieee154_rcp.h` and one codec file compiled into the Zephyr driver, the AVR firmware and a host test. It is not ZigBit-specific; another radio could implement the device side.
- **The Zephyr driver is board-independent:** only the UART, SPI and GPIO APIs and a devicetree binding (for example `esp-sdr,ieee154-rcp`, a child of a `uart` or `spi` node, with `reset-gpios` and `irq-gpios`). Kconfig `CONFIG_ESP_SDR_IEEE802154_RCP`, next to `CONFIG_ESP_SDR_IEEE802154`.
- **The XMEGA firmware** (bare-metal avr-gcc; Zephyr has no AVR port) holds the RF233 register driver, the device side of the protocol, a 1 us timer for timestamps, the factory EUI-64 and the XTAL trim.

### Messages

| Host to RCP | RCP to host |
| --- | --- |
| `RESET`, `GET_INFO` (part number, firmware version, EUI-64, XTAL trim) | `INFO` |
| `SET_CHANNEL`, `SET_TXPOWER`, `SET_FILTER` (PAN ID, short, extended, coordinator), `SET_CONFIG` (promiscuous, auto-ACK, frame pending, deliver bad-FCS frames) | `ACK` (status per command) |
| `START`, `STOP` (receive on or off) | `RX_FRAME` (PSDU with FCS, FCS valid flag, LQI, ED in dBm, timestamp of the frame start, channel) |
| `TX` (frame, mode: direct, CCA or CSMA-CA, ACK wait, retries) | `TX_DONE` (success, no ACK, channel busy; ACK frame and its timestamp) |
| `ED_SCAN` (duration), `CCA` | `ED_DONE`, `CCA_DONE` |
| `TEST` (continuous carrier or PRBS on a channel, off) | `EVENT` (receive overflow, link errors, counters) |

Framing on the UART: HDLC-style flags and escapes with a CRC-16, 1 Mbaud to start (the XMEGA's 32 MHz clock gives exact baud rates of 1 and 2 Mbaud), flow control by a window of outstanding commands. A 127-byte frame takes about 1.4 ms on the link against 4.3 ms on the air. On SPI: length-prefixed transfers, the ZigBit raising IRQ when it has data; SPI slave mode on the XMEGA has a single-byte buffer, so the clock has to stay low (about 1 MHz) or use the XMEGA's DMA.

### Mapping to `ieee802154_radio_api`

| API | RCP |
| --- | --- |
| `get_capabilities` | `ENERGY_SCAN`, `FCS`, `PROMISC`, `FILTER`, `CSMA`, `TX_RX_ACK`, `RX_TX_ACK`, `RETRANSMISSION` (the band comes from `attr_get`) |
| `set_channel`, `set_txpower`, `filter` | `SET_CHANNEL`, `SET_TXPOWER`, `SET_FILTER` |
| `start`, `stop` | `START`, `STOP` |
| `tx` (direct, CCA, CSMA-CA) | `TX`, then wait for `TX_DONE` |
| `cca`, `ed_scan` | `CCA`, `ED_SCAN` |
| `configure` (promiscuous, auto-ACK, event handler) | `SET_CONFIG` |
| `attr_get` | channel page 0, channels 11 to 26, answered by the driver |

### Alternatives considered

- **Register bridge for Zephyr's `rf2xx` driver** (the XMEGA forwards SPI and GPIO, the host runs the upstream driver): the smallest XMEGA firmware and an upstream driver, but it needs a patch to `rf2xx_iface.c` in `modules/zephyr` or a copy of the driver, every register access becomes a round trip over the link, and timestamps would be taken on the far side of it.
- **Spinel RCP for `hdlc_rcp_if`:** OpenThread has no AVR port, and Spinel serves only an OpenThread host, not the plain radio API the test app uses.
- **`ieee802154_uart_pipe`:** frames only (no channel, ACK or filter control), and a single global pipe made for QEMU.

## Phases

| Phase | Work | Done when |
| --- | --- | --- |
| 0 | Protocol header and codec, with a unit test like `tests/unit/ieee154_phy/` (encode, decode, framing errors). Devicetree binding and Kconfig. AVR toolchain (avr-gcc, avr-libc, avrdude) in `shell.nix`. | The unit test passes. |
| 1 | Wire the ZigBit to the XIAO as above. Back up flash (if not locked) and the persistent memory before any erase. XMEGA: LED, UART, `PART_NUM` = 0x0B from the RF233. | The backup is stored and the RF233 answers. |
| 2 | XMEGA: basic mode TX and RX at 250 kb/s O-QPSK, channels 11 to 26, FCS, LQI, ED, XTAL trim applied. Protocol over UART: `GET_INFO`, `TX`, `RX_FRAME`. | Frames from the host's shell reach the C6, and back. |
| 3 | Zephyr driver complete; RF233 RX_AACK and TX_ARET; `wpan` on the second S3 with the RCP as the chosen radio (board overlay or snippet). | `interop.py --native <host S3 + ZigBit> --sdr <SDR S3>` and ZigBit against C6, on channels 11, 15, 20, 25 and 26: both ways and ACK. |
| 4 | SPI transport with the IRQ line. | The same results as phase 3 over SPI. |
| 5 | Measurements below, as `wpan ref ...` commands over vendor messages. | Each one answered in [ieee802154.md](ieee802154.md). |
| 6 (optional) | The Zephyr L2 (and OpenThread) on the RCP, a known-good node for the L2 work on the software radio. Wireshark sniffing through the same record format as the S3 sniffer, to diff captures. | |

## What the RF233 answers that the C6 does not

| Open issue in [ieee802154.md](ieee802154.md) | Measurement |
| --- | --- |
| Channel 26 from the C6 | ZigBit to S3 on 2480 MHz. If it decodes, the scrambled phase steps are the C6's; if it fails the same way, the S3 receiver is suspect. |
| Bit errors in the S3's transmission | In basic RX_ON mode the RF233 hands over frames with a bad FCS too, flagged by `RX_CRC_VALID`. With the "deliver bad-FCS frames" option the host diffs the bytes against the test pattern and sees where a transmitted frame breaks. The C6 drops such frames silently. |
| ACK turnaround (192 us) | The ZigBit's 1 us timer stamps the end of its own frame and the start of the S3's ACK on one clock: the S3's real turnaround, independent of the link. TX_ARET gives the strict pass or fail. |
| TX power, CCA and energy detection | The RF233's ED is in dBm (base -94 dBm). Through a known attenuator it maps the S3's gain index to dBm and gives a CCA threshold. |
| Carrier frequency offset | `TEST` puts out a continuous carrier with the XTAL trim applied, for the S3 to measure its own LO error. |

## Setup

- Coax from the ZigBit's RF test connector to the SDR S3's U.FL antenna connector with 30 to 40 dB of attenuation in line: the RF233 sends up to +4 dBm (down to -17 dBm), and an attenuator gives a realistic SNR as well as protecting both front ends. The connector type of the ZigBit's test port (U.FL or a switched test connector that needs its own probe cable) is still to be checked.
- One host S3 per ZigBit. Putting the ZigBit on the SDR S3 itself would give one clock and one shell for both radios, but that image has about 200 bytes of DRAM left and CPU 1 belongs to the ring: not before the separate setup works.

## Open points

- Which programmer is available (Atmel-ICE, JTAGICE3, PICkit 4). Without one, the serial bootloader needs a small uploader and there is no debugger.
- A cable for the RF test connector and an attenuator.
- Where the XMEGA firmware lives (for example `firmware/zigbit-rcp/`), the binding's vendor prefix, and the `shell.nix` change: decided when code is written.
