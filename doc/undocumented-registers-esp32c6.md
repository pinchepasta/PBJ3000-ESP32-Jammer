# ESP32-C6: undocumented radio registers

What this project knows about the ESP32-C6 registers that `lib/esp_sdr` relies
on, or that a later feature could use, and that Espressif does not document, or
documents without the behaviour that matters: where each one is, its fields,
what it does, the values seen, and how sure we are. The ESP32-S3 has its own
page, [undocumented-registers-esp32s3.md](undocumented-registers-esp32s3.md).
How the library copes with the radio's quirks is in
[hardware-quirks.md](hardware-quirks.md).

## How to read this page

- The register part follows Espressif's technical reference manual: a register
  summary, then each register with a bit-field table and its fields'
  descriptions. The table's Init row, where present, is the value after the
  PHY's initialization (read on the bench after it), not the reset value.
  "(unknown)" marks bits nobody has tied to a function yet, "(documented)" bits
  that Espressif's SVD or headers document in a register this page only adds to.
- Names are spelled as in Espressif's headers (`REGISTER_REG`, `FIELD`):
  Espressif's own in the peripherals its SVD has (HP_SYS, IO_MUX, MODEM_SYSCON,
  MODEM_LPCON), this project's elsewhere.
- Access: R/W read and write, RO read only (set by the hardware), WO written only
  (reads unknown). It is inferred from the vendor code and this project's use.
- Analog I2C registers are 8 bits wide and addressed `block:reg` with a decimal
  register number; `0x62:12[3:2]` is bits 3:2 of register 12 in analog block
  0x62 (upstream esp-sdr uses the same form). Their blocks are not
  memory-mapped (see "Analog I2C access").
- The register part is generated from [registers/esp32c6.svd](registers/esp32c6.svd),
  a CMSIS-SVD file of this project's additions to Espressif's SVD
  ([espressif/svd](https://github.com/espressif/svd) at be20aa1), by
  `doc/registers/svd2md.py`: edit the SVD, then run the script.
- Evidence paths are relative to this repository (`lib/`, `doc/`, ...) or, for
  files outside it, to the west workspace: `modules/lib/esp-sdr/` is upstream
  esp-sdr (commit 1fe5535), `modules/hal/espressif/` the HAL. `git show
  <commit>:<path>` cites a file at an older upstream commit: run it in
  `modules/lib/esp-sdr/` (34bb79b and e605684 are in the history of 1fe5535).
  "Build" with a date is the link map of an app of this repository built that
  day. "Bench" with a date is a measurement with scratch tools that are not in
  the repository: 2026-10-07 to 09 and 2026-10-11 on one XIAO ESP32C6 next to
  one XIAO ESP32S3 (2026-10-11: the C6 transmitting into the S3 over coax, the
  S3's receive gain fixed at index 0).
- Entries marked decompile come from `objdump` of the C6 blobs
  (`modules/hal/espressif/zephyr/blobs/lib/esp32c6/libphy.a`,
  `zephyr/blobs/lib/esp32c6/librftest.a` and, where an entry names them,
  `libcoexist.a` and `libble_app.a` from the same directory as `libphy.a`) and
  of the C6 ROM. ROM addresses refer to `esp32c6_rev0_rom.elf` of esp-rom-elfs
  20230320. An entry citing an S3 object, as in "S3 `phy_rfpll.o`", means a
  member of `modules/hal/espressif/zephyr/blobs/lib/esp32s3/libphy.a`. ESP32-C5 and ESP32-C61 entries marked decompile come from `objdump`
  of those chips' `libphy.a`, `libcoexist.a` and `libble_app.a` in
  `modules/hal/espressif/zephyr/blobs/lib/esp32c5/` and `esp32c61/`, their
  `librftest.a` from esp-phy-lib e37ecdc9, and their ROMs from esp-rom-elfs
  20260528.

| Confidence | Meaning |
|---|---|
| measured | Seen on the bench boards, on air between them, or in the link map of a build. |
| working | The library depends on it and works; the meaning is not proven beyond that. |
| upstream | Stated by upstream esp-sdr (code, comments, docs); not re-checked here. |
| decompile | Read from vendor code (decompile or disassembly); not confirmed on hardware. |
| speculative | A guess. Treat the field as unknown. |

## Analog I2C access

The library reaches the analog blocks through two ROM functions,
`rom_chip_i2c_readReg(block, host, reg)` and
`rom_chip_i2c_writeReg(block, host, reg, value)`, on 8-bit registers. The
Zephyr link maps them to the ROM at 0x40003dc0 and 0x400040b4.

| Fact | Evidence |
|---|---|
| The `host` argument is ignored. The I2C master is picked from the block id: 0x62, 0x63, 0x64, 0x67 and 0x6b go to master 1, every other block to master 0 (bit mask 0x227 over `block - 0x62`). | *decompile*: C6 ROM `rom_chip_i2c_readReg` and `rom_get_i2c_hostid` (0x40003d4a, `andi 551`). *measured*: host 0 and 1 read identically in every register of blocks 0x60 to 0x6f (bench 2026-10-09). |
| Host values in code are therefore cosmetic: the library passes 1 for BBTOP, the RF PLL and the LO selector, and upstream's docs say host 1 for BBTOP on every chip. Across the two chips they are inconsistent (see the ESP32-S3 page). | `lib/esp_sdr/src/esp_sdr_priv.h:106`; `modules/lib/esp-sdr/docs/rx-controls.md:43` |
| The ROM functions take no lock (shown on the ESP32-S3, whose ROM lock functions are empty; see its page). libphy's `ram_chip_i2c_*` call `phy_i2c_enter_critical()`, which the HAL maps to `regi2c_enter_critical()`. The Wi-Fi PHY may run a sequence at any time, so the library takes `regi2c_enter_critical()` around every `rom_chip_i2c_*` sequence. | `modules/hal/espressif/components/esp_phy/src/phy_override.c:50-58`; *working*: `lib/esp_sdr/src/esp_sdr_priv.h:175-180`, hardware-quirks.md |

The I2C masters themselves are memory-mapped (I2C_ANA_MST below).

### Block map

Register counts are the loop bounds of the vendor's debug dump
`phy_i2c_check()`, names the HAL's comment. The HAL defines fields only for
0x61, 0x66, 0x69, 0x6a and 0x6d.

| Block | HAL name and registers | Used by esp_sdr |
|---|---|---|
| 0x61 | ULP, 11 | no (stale-state symptom) |
| 0x62 | PLL, 19 (0 to 18) | VCO status, LO divider |
| 0x63 | SDM, 7 | no (read on the bench) |
| 0x64 | absent | |
| 0x65 | absent | |
| 0x66 | BBPLL, 11 | no |
| 0x67 | BB, 57 | RX filter codes |
| 0x69 | SAR, 9 | no |
| 0x6a | BIAS, 4 | no |
| 0x6b | TXRF, 1 to 15 | no |
| 0x6d | PMU, 15 | no |

Evidence: *decompile* C6 `libphy.a` `phy_debug.o` (`phy_i2c_check`);
`modules/hal/espressif/components/esp_hal_regi2c/esp32c6/regi2c_impl.c:9-19`.
*measured*: blocks 0x60, 0x64, 0x65, 0x68, 0x6c, 0x6e and 0x6f read 0xff in
every register (bench 2026-10-09).

## Register summary

<!-- svd:summary -->

| Name | Description | Address | Access |
|---|---|---|---|
| **Dump and DAC engine** | | | |
| [DUMP_CTRL_REG](#register-dump_ctrl_reg-0x600a9004) | Receive side: run, trigger and count of a capture | 0x600a9004 | varies |
| [DUMP_MODE_REG](#register-dump_mode_reg-0x600a9008) | Dump source; the live write index in upstream's ring | 0x600a9008 | varies |
| [DUMP_DAC_TRIG_REG](#register-dump_dac_trig_reg-0x600a900c) | Transmit side: run, loop, rate and count of a playback | 0x600a900c | varies |
| [DUMP_CONFIG_REG](#register-dump_config_reg-0x600a9014) | Lane selectors: the sources packed into a dump word | 0x600a9014 | R/W |
| [DUMP_DIAG_SEL0_REG, DUMP_DIAG_SEL1_REG](#register-dump_diag_sel0_reg-dump_diag_sel1_reg-0x600a9018-0x600a901c) | Modem diagnostic bus selectors | 0x600a9018, 0x600a901c | R/W |
| **SRAM bank ownership** | | | |
| [HP_SYSTEM_SRAM_USAGE_CONF_REG](#register-hp_system_sram_usage_conf_reg-0x60095004) | Hands an SRAM block to the dump engine | 0x60095004 | R/W |
| **Modem clocks and reset** | | | |
| [MODEM_SYSCON_CLK_CONF_REG](#register-modem_syscon_clk_conf_reg-0x600a9804) | Modem clock enables, forced on for the dump engine | 0x600a9804 | R/W |
| [MODEM_SYSCON_CLK_CONF_POWER_ST_REG](#register-modem_syscon_clk_conf_power_st_reg-0x600a980c) | Forced to 0xffffffff for the dump engine | 0x600a980c | R/W |
| [MODEM_SYSCON_MODEM_RST_CONF_REG](#register-modem_syscon_modem_rst_conf_reg-0x600a9810) | Modem resets, pulsed at boot against stale modem state | 0x600a9810 | R/W |
| [MODEM_SYSCON_CLK_CONF1_REG](#register-modem_syscon_clk_conf1_reg-0x600a9814) | Bits 18:0 forced on for the dump engine | 0x600a9814 | R/W |
| [MODEM_SYSCON_WIFI_BB_CFG_REG](#register-modem_syscon_wifi_bb_cfg_reg-0x600a981c) | Wi-Fi baseband configuration; the 802.11p clock mode in bits 4:2 | 0x600a981c | R/W |
| **Modem LP reset** | | | |
| [MODEM_LPCON_RST_CONF_REG](#register-modem_lpcon_rst_conf_reg-0x600af024) | Modem LP resets, pulsed at boot against stale modem state | 0x600af024 | WO |
| **Modem diagnostic lines** | | | |
| [IO_MUX_MODEM_DIAG_EN_REG](#register-io_mux_modem_diag_en_reg-0x600900bc) | Routes modem diagnostic lines into the GPIO matrix | 0x600900bc | R/W |
| **BTBB page (0x600a2000)** | | | |
| [BTBB_RB4_REG](#register-btbb_rb4_reg-0x600a20b4) | Dump gate in bit 0 | 0x600a20b4 | R/W |
| [BTBB_R8A0_REG](#register-btbb_r8a0_reg-0x600a28a0) | Unknown; bits 31:30 written by the BT branch of `force_txon_mode()` | 0x600a28a0 | R/W |
| **Wi-Fi MAC page (0x600a4000)** | | | |
| [WIFIMAC_EVENT_SEL_REG](#register-wifimac_event_sel_reg-0x600a4e30) | Wi-Fi MAC event selector, used by the dump's `adctrig()` mode 7 | 0x600a4e30 | R/W |
| **AGC force** | | | |
| [AGC_FORCE_REG](#register-agc_force_reg-0x600a702c) | Forced receive gain index, force bit and largest calibrated index | 0x600a702c | R/W |
| [AGC_R30_REG](#register-agc_r30_reg-0x600a7030) | Unknown; bit 5 cleared in the 802.11p clock mode | 0x600a7030 | R/W |
| [AGC_RB8_REG](#register-agc_rb8_reg-0x600a70b8) | Unknown; bits 2:0 written by `set_dump_mode()` | 0x600a70b8 | R/W |
| [AGC_RDC_REG](#register-agc_rdc_reg-0x600a70dc) | Unknown; bit 16 cleared by `force_txon()` | 0x600a70dc | R/W |
| **Front end** | | | |
| [FE_RC0_REG](#register-fe_rc0_reg-0x600a00c0) | Hardware frequency engine stop | 0x600a00c0 | R/W |
| [FE_PWR_INDEX_REG](#register-fe_pwr_index_reg-0x600a0410) | Power index force | 0x600a0410 | R/W |
| [FE_DAC_SRC_REG](#register-fe_dac_src_reg-0x600a0414) | DAC source select | 0x600a0414 | R/W |
| [FE_TXDC_CAL_REG](#register-fe_txdc_cal_reg-0x600a0418) | TX DC calibration comparator | 0x600a0418 | varies |
| [FE_TONE0_REG, FE_TONE1_REG](#register-fe_tone0_reg-fe_tone1_reg-0x600a0420-0x600a0424) | Tones 0 and 1: enable, attenuation, frequency | 0x600a0420, 0x600a0424 | R/W |
| [FE_TONE_FREQ_LO_REG](#register-fe_tone_freq_lo_reg-0x600a042c) | Fine tone frequency: the low two bits of each tone's frequency | 0x600a042c | R/W |
| [FE_TXDC_REG](#register-fe_txdc_reg-0x600a0444) | TX DC value | 0x600a0444 | R/W |
| [FE_R44C_REG](#register-fe_r44c_reg-0x600a044c) | Unknown; init sets bits 27 and 25 and clears bit 22 | 0x600a044c | R/W |
| [FE_FORCE_REG](#register-fe_force_reg-0x600a0910) | Receive, transmit and BT mode force pairs; receive and transmit clocks | 0x600a0910 | R/W |
| [FE_R958_REG](#register-fe_r958_reg-0x600a0958) | Unknown; `set_dump_mode()` clears bits 1:0 | 0x600a0958 | R/W |
| **Analog I2C master** | | | |
| [I2C_ANA_MST_I2C0_CTRL_REG, I2C_ANA_MST_I2C1_CTRL_REG](#register-i2c_ana_mst_i2c0_ctrl_reg-i2c_ana_mst_i2c1_ctrl_reg-0x600af800-0x600af804) | Command word of master 0 and master 1 | 0x600af800, 0x600af804 | varies |
| [I2C_ANA_MST_ANA_CONF1_REG](#register-i2c_ana_mst_ana_conf1_reg-0x600af81c) | Read mask | 0x600af81c | R/W |
| [I2C_ANA_MST_ANA_CONF2_REG](#register-i2c_ana_mst_ana_conf2_reg-0x600af820) | Master select | 0x600af820 | R/W |
| **Analog I2C block 0x61 (ULP)** | | | |
| [ULP_R4](#register-ulp_r4-0x614) | HAL `I2C_ULP_OCODE`, function undocumented | 0x61:4 | R/W |
| [ULP_R9](#register-ulp_r9-0x619) | Unknown | 0x61:9 | R/W |
| **Analog I2C block 0x62 (RF PLL)** | | | |
| [RFPLL_R0](#register-rfpll_r0-0x620) | VCO calibration start | 0x62:0 | varies |
| [RFPLL_R1](#register-rfpll_r1-0x621) | Manual capacitor code, bits 7:0 | 0x62:1 | R/W |
| [RFPLL_R2](#register-rfpll_r2-0x622) | Manual code bit 8 and a 4-bit code override | 0x62:2 | R/W |
| [RFPLL_R4](#register-rfpll_r4-0x624) | Unknown | 0x62:4 | R/W |
| [RFPLL_R5](#register-rfpll_r5-0x625) | Capacitor code readback, bits 7:0 | 0x62:5 | RO |
| [RFPLL_R6](#register-rfpll_r6-0x626) | Calibrated 4-bit code | 0x62:6 | RO |
| [RFPLL_R7](#register-rfpll_r7-0x627) | Capacitor code bit 8 and calibration done | 0x62:7 | RO |
| [RFPLL_R11](#register-rfpll_r11-0x6211) | Capacitor code source | 0x62:11 | R/W |
| [RFPLL_R12](#register-rfpll_r12-0x6212) | VCO window comparator and two unknown status bits | 0x62:12 | RO |
| [RFPLL_R14](#register-rfpll_r14-0x6214) | Charge pump calibration done and result | 0x62:14 | RO |
| [RFPLL_R15](#register-rfpll_r15-0x6215) | Charge pump calibration | 0x62:15 | varies |
| [RFPLL_R16](#register-rfpll_r16-0x6216) | LO 5/6 divider select | 0x62:16 | R/W |
| [RFPLL_R17](#register-rfpll_r17-0x6217) | Unknown | 0x62:17 | R/W |
| [RFPLL_R18](#register-rfpll_r18-0x6218) | Unknown | 0x62:18 | R/W |
| **Analog I2C block 0x63 (RF PLL fractional-N divider)** | | | |
| [RFPLL_SDM_R0](#register-rfpll_sdm_r0-0x630) | Bit 3 cleared and set around a divider write; other bits unknown | 0x63:0 | R/W |
| [RFPLL_SDM_R1](#register-rfpll_sdm_r1-0x631) | Unknown | 0x63:1 | R/W |
| [RFPLL_SDM_R3](#register-rfpll_sdm_r3-0x633) | Divider integer part | 0x63:3 | R/W |
| [RFPLL_SDM_R4](#register-rfpll_sdm_r4-0x634) | Divider fraction, high byte | 0x63:4 | R/W |
| [RFPLL_SDM_R5](#register-rfpll_sdm_r5-0x635) | Divider fraction, middle byte | 0x63:5 | R/W |
| [RFPLL_SDM_R6](#register-rfpll_sdm_r6-0x636) | Divider fraction, low 3 bits | 0x63:6 | R/W |
| **Analog I2C block 0x66 (BBPLL)** | | | |
| **Analog I2C block 0x67 (BBTOP, analog baseband)** | | | |
| [BBTOP_R4, BBTOP_R5](#register-bbtop_r4-bbtop_r5-0x674-0x675) | Receive low-pass capacitor code, I (BBTOP_R4) and Q (BBTOP_R5) | 0x67:4, 0x67:5 | R/W |
| [BBTOP_R12, BBTOP_R13](#register-bbtop_r12-bbtop_r13-0x6712-0x6713) | Probably the transmit low-pass pair A, as on the S3 | 0x67:12, 0x67:13 | R/W |
| [BBTOP_R14, BBTOP_R15](#register-bbtop_r14-bbtop_r15-0x6714-0x6715) | Probably the transmit low-pass pair B, as on the S3 | 0x67:14, 0x67:15 | R/W |
| **Analog I2C block 0x6a (bias)** | | | |

<!-- svd:end -->

## Registers

<!-- svd:registers -->

### Dump and DAC engine

Same RUN, TRIGGER, DONE and COUNT bits as the S3 at another address (*upstream*
`modules/lib/esp-sdr/main/families/c5_c6_c61/receiver.c`,
`modules/lib/esp-sdr/main/targets/esp32c6/chip.h`). Receive runs at 80 MS/s
only; the transmit side is a DAC engine at 0x600a900c (DUMP_DAC_TRIG_REG,
below), which no vendor code programs: the C6 `dactrig()` only writes a ramp
into 0x40840000 (*decompile* C6 `librftest.a` `mac_common.o`). Base 0x600a9000
has no name in the HAL. Each capture and each playback needs the clocks in
MODEM_SYSCON forced on and an SRAM block handed to the engine in
HP_SYSTEM_SRAM_USAGE_CONF_REG.

**Receive rate.** The C6 `adctrig()` starts only mode 0 by software; the other
modes wait for a hardware event. It writes no part of DUMP_MODE_REG (0x600a9008)
but the source field, DUMP_SRC, and never reads the argument that the C5 and C61
versions turn into a rate (*decompile* C6 `librftest.a` `mac_common.o`):

| `adctrig()` mode | DUMP_SRC | Also |
|---|---|---|
| 0 | 15 | software start |
| 1, 3 | 1 | |
| 2 | 0 | |
| 4 | 2 | |
| 5 | 3 | tries to send a frame each poll, at most one per `tx_frame_delay_us` (800 by default) |
| 6 | 4 | as mode 5 |
| 7 | 5 | Wi-Fi MAC event selector WIFIMAC_EVENT_SEL_REG (0x600a4e30): bit 8 set, bits 7:0 the event |
| 8 | 0 | BTBB_DUMP_GATE (0x600a20b4 bit 0) set, lanes 0x0e to 0x11, BLE receiver started |

Upstream found no other source that gives a uniform stream, and neither
`adc_rate_set()` nor candidate divider bits changed the timing (*upstream*
`git show 34bb79b:docs/rx-controls.md`, "C6 sample-rate investigation").
Upstream's C6 ring is gapless at 80 MS/s, rotating SRAM blocks 1 and 2 under a
circular DUMP_CTRL_REG (DUMP_CIRCULAR, bit 17, count 16384), and consumes it as FFT
windows (*upstream* `modules/lib/esp-sdr/main/common/ring_capture.c:96-131,
1485-1507`): the 160 MHz CPU has 2 cycles per sample, too few to decimate the
whole stream.

Leads for a continuous C6 receive below 80 MS/s, none tried:

- The modem diagnostic bus into PARL_IO. A third-party project, not re-checked
  here, reports for the ESP32-C5 that the dump's I/Q MSBs appear on MODEM_DIAG
  (Q[9:6] on lines 9:6, I[9:6] on 19:16), and that routed through the GPIO
  matrix into PARL_IO, a cyclic 40 MS/s capture takes every second sample of
  the 80 MS/s bus as 4+4-bit I/Q, bit-perfect in a bounded capture (it claims
  no gapless running across ring wraps;
  [Twotoz/C5VRX](https://github.com/Twotoz/C5VRX/blob/a165e19db47ebd49051217d7c968a1b8220c20fb/docs/continuous-iq-findings.md)).
  The C6 selector registers are DUMP_DIAG_SEL0_REG and DUMP_DIAG_SEL1_REG
  (0x600a9018 and 0x600a901c), and the diag lines are enabled in
  IO_MUX_MODEM_DIAG_EN_REG (0x600900bc); both below. Which selector
  value puts receive I/Q on the C6 bus is unknown. Sampling below 80 MHz
  decimates without filtering, so the analog low-pass has to be narrowed first.
- The 802.11p clock mode: `phy_11p_set(1, m)` writes MODEM_SYSCON_WIFI_BB_CFG_REG
  (0x600a981c) bits 4:2 = 4 for m = 0, else 5, presumably for 10 and 5 MHz
  channels, and clears AGC_R30_REG (0x600a7030) bit 5 (*decompile* C6 `libphy.a`
  `phy_feature.o`). Whether the dump follows the slower clock is unknown.
- The BLE dump (mode 8 above): rate unknown; it runs only while the BLE receiver
  does.

#### Register DUMP_CTRL_REG (0x600a9004)

Receive side: run, trigger and count of a capture

| Bits | 31 | 30:20 | 19 | 18 | 17 | 16:14 | 13:0 |
|---|---|---|---|---|---|---|---|
| Field | DUMP_RUN | (unknown) | DUMP_TRIGGER | DUMP_DONE | DUMP_CIRCULAR | (unknown) | DUMP_COUNT |

**DUMP_RUN** Enables the engine, as on the S3. (R/W)

**DUMP_TRIGGER** As on the S3, write 1, then 0, to start a one-shot capture. (WO)

**DUMP_DONE** As on the S3, reads 1 once DUMP_COUNT words are written. Write DUMP_COUNT with DUMP_DONE first to clear a stale DUMP_DONE, as the stock sequence does. (R/W)

**DUMP_CIRCULAR** Circular ring, in upstream's C6 ring code. Not used here. (R/W)

**DUMP_COUNT** Words to capture, 256 to 16380; 16380 samples take about 205 us. (R/W)

No rate field, unlike the S3's: receive runs at 80 MS/s only.

Evidence:

- *working* `lib/esp_sdr/src/esp_sdr_priv.h:77-81`, `lib/esp_sdr/src/esp_sdr_rx.c:302-310`; *upstream* `modules/lib/esp-sdr/main/targets/esp32c6/chip.h:43-46` (DUMP_RUN, DUMP_TRIGGER, DUMP_DONE, DUMP_COUNT)
- DUMP_CIRCULAR: *upstream* `modules/lib/esp-sdr/main/common/ring_capture.c:104, 1499-1505`

#### Register DUMP_MODE_REG (0x600a9008)

Dump source; the live write index in upstream's ring

| Bits | 31:24 | 23:21 | 20:19 | 18:15 | 14:0 |
|---|---|---|---|---|---|
| Field | (unknown) | DUMP_B23_21 | (unknown) | DUMP_SRC | (unknown) |

**DUMP_B23_21** Not a rate field, unlike on the C5 and C61, where these bits take a rate code: written bits do not stay, and captures and replays keep their timing. (RO)

**DUMP_SRC** Dump source; 15 = raw receive I/Q, as the stock `adctrig()` (see "Receive rate" above for the other sources). The library writes 15. (R/W)

Upstream's ring code reads the live write index from the low bits of this
register.

Evidence:

- *upstream* `modules/lib/esp-sdr/main/common/ring_capture.c:96, 1507` (write index)
- DUMP_B23_21: *measured* bench 2026-10-11; C5 and C61: *upstream* `modules/lib/esp-sdr/main/targets/esp32c5/chip.h:11`, `modules/lib/esp-sdr/main/targets/esp32c61/chip.h:20-21`
- DUMP_SRC: *working* `lib/esp_sdr/src/esp_sdr_priv.h:82-84`; *upstream* `modules/lib/esp-sdr/main/targets/esp32c6/chip.h:6, 39`

#### Register DUMP_DAC_TRIG_REG (0x600a900c)

Transmit side: run, loop, rate and count of a playback

| Bits | 31 | 30:28 | 27:20 | 19 | 18 | 17:16 | 15 | 14 | 13:0 |
|---|---|---|---|---|---|---|---|---|---|
| Field | DUMP_DAC_RUN | (unknown) | DUMP_B27_20 | DUMP_DAC_LOOP | DUMP_DAC_DONE | (unknown) | DUMP_DAC_RATE_80M | (unknown) | DUMP_DAC_COUNT |

**DUMP_DAC_RUN** Written with DUMP_DAC_COUNT and with DUMP_DAC_LOOP clear: one pass, with no trigger pulse as on the S3. (R/W)

**DUMP_B27_20** Read back as written; effect untested. The S3 `dactrig()` writes the count's low byte here (see the ESP32-S3 page); upstream's C5 replay uses it as a repeat count, 0 meaning endless with bit 19 set. (R/W)

**DUMP_DAC_LOOP** Set with DUMP_DAC_RUN (DUMP_B27_20 at 0): the engine replays the first DUMP_DAC_COUNT words back to back until DUMP_DAC_RUN is cleared; DUMP_DAC_DONE stays clear. (R/W)

**DUMP_DAC_DONE** Sets after one pass. (RO)

**DUMP_DAC_RATE_80M** 1 = 80 MS/s, 0 = 40 MS/s: a pass takes 4 cycles of the 160 MHz CPU per word at 40 MS/s (DUMP_DAC_COUNT 1000 to 16000), 2 at 80 MS/s (DUMP_DAC_COUNT 16000). The rate is this bit, as on the S3, not bit 17 as in upstream's C5 replay. (R/W)

**DUMP_DAC_COUNT** Words to play from the start of the owned block, up to 16383. (R/W)

8 bytes above DUMP_CTRL_REG, as on the S3. Bits 14, 16 and 17 do not read back.

- One word per sample, I in bits 9:0 and Q in 19:10, as for the S3's DAC (see
  "Sample word formats"): a complex tone of k cycles in n words plays at +k/n
  of the rate (200 cycles in 16000 words: 0.5 MHz at 40 MS/s, 1 MHz at
  80 MS/s), its image 37 to 40 dB down. All-zero words play nothing, a
  constant word a carrier.
- What playback needs: the dump clocks forced on as for a capture (without
  them DUMP_DAC_DONE still sets on time but nothing reaches the antenna; see
  MODEM_SYSCON_CLK_CONF_REG), SRAM block 2 handed to the engine
  (HP_SYSTEM_SRAM_USAGE = 4, that is 0x60095004 bits 11:8, samples from
  0x40840000), and the transmit chain on without its tone: the vendor CW
  (`esp_phy_wifi_tx_tone(1, 6, 80)`), then bit 18 (FE_TONE_EN) of 0x600a0420
  and 0x600a0424 cleared. That leaves the DAC source select, FE_DAC_SRC
  (0x600a0414 bits 1:0), at 0, the tone generator's setting (see
  "Front end").
- While the engine owns the block, CPU stores are dropped and loads return
  other data, as during a capture.

No vendor blob or ROM of the C6, C5 or C61 writes this register. Only upstream
code does: the C5 transmitter of its first upload (`emit_dma()`, removed in
ab76232), and the C61 capture and ring code, which writes 0 to it before each
capture. Upstream's S31 code clears bit 31 of its counterpart, 0x2010900c, when
it sets up the dump.

Evidence:

- *measured* bench 2026-10-11 (read-back, sample format, what playback needs, CPU access)
- *decompile* (no vendor or ROM writer)
- `git show e605684:main/c5_main.c`, `emit_dma()` (C5 transmitter)
- *upstream* `modules/lib/esp-sdr/main/targets/esp32c61/chip.h:12, 17`, `modules/lib/esp-sdr/main/common/ring_capture.c:1493` (C61)
- *upstream* `modules/lib/esp-sdr/main/targets/esp32s31/burst/receiver.c:37, 123` (S31)
- DUMP_DAC_RUN: *measured* bench 2026-10-11
- DUMP_B27_20: *measured* bench 2026-10-11 (read-back only); *upstream* `git show e605684:main/c5_main.c`, comment above `emit_dma()`
- DUMP_DAC_LOOP: *measured* bench 2026-10-11
- DUMP_DAC_DONE: *measured* bench 2026-10-11
- DUMP_DAC_RATE_80M: *measured* bench 2026-10-11
- DUMP_DAC_COUNT: *measured* bench 2026-10-11

#### Register DUMP_CONFIG_REG (0x600a9014)

Lane selectors: the sources packed into a dump word

| Bits | 31:25 | 24:0 |
|---|---|---|
| Field | (unknown) | DUMP_LANES |

**DUMP_LANES** Lane selectors as the S3's DUMP_CONFIG_REG (0x60033d90, see the ESP32-S3 page): four 6-bit lane selectors, lane n in bits 6n+5:6n. The library clears the field, then sets it to 0x000c2040. `adctrig()` mode 8, the BLE dump, uses lanes 0x0e to 0x11 (see "Receive rate" above). (R/W)

Evidence:

- DUMP_LANES: *working* `lib/esp_sdr/src/esp_sdr_rx.c:258`; *upstream* `modules/lib/esp-sdr/main/targets/esp32c6/chip.h:38`
- DUMP_LANES: *decompile* C6 `librftest.a` `mac_common.o` (mode 8)

#### Register DUMP_DIAG_SEL0_REG, DUMP_DIAG_SEL1_REG (0x600a9018, 0x600a901c)

Modem diagnostic bus selectors

| Bits | 31:0 |
|---|---|
| Field | (unknown) |

`gpio_diag_init_ble()` writes 1 to DUMP_DIAG_SEL0_REG and
`coex_hw_debug_matrix_config()` 2 to DUMP_DIAG_SEL1_REG, on the C6 as on the
C5, which has the pair at 0x600a9404 and 0x600a9408. On the C6 each function
then enables diag lines in IO_MUX_MODEM_DIAG_EN_REG: 0xffff0000 and
0x0000000f. Which selector value puts receive I/Q on the C6 bus is unknown
(see the leads above).

Evidence:

- *decompile* C6 and C5 `libble_app.a` `gpio_diag.c.o`, `libcoexist.a` `coexist_hw.o`

### SRAM bank ownership

#### Register HP_SYSTEM_SRAM_USAGE_CONF_REG (0x60095004)

Hands an SRAM block to the dump engine

| Bits | 31:17 | 16 | 15:12 | 11:8 | 7:1 | 0 |
|---|---|---|---|---|---|---|
| Field | (unknown) | HP_SYSTEM_MAC_DUMP_ALLOC | (unknown) | HP_SYSTEM_SRAM_USAGE | (unknown) | (documented) |

**HP_SYSTEM_MAC_DUMP_ALLOC** Adds a 64 KiB offset (HAL). The library leaves it 0; upstream's ring clears it. (R/W)

**HP_SYSTEM_SRAM_USAGE** One-hot: field bit n (register bit 8 + n) hands block n, 0x40800000 + n x 128 KiB, to the engine, for capture and playback alike; 0 gives every block back to the CPUs. n = 2 (0x40840000, the stock `adctrig()` bank) is the capture bank. The HAL says "0: cpu use hp-memory, 1: mac-dump accessing hp-memory". (R/W)

The library saves the register and restores it after each use. Undocumented
behaviour:

- While the engine owns a block the CPUs cannot reach any of it, so nothing
  else may live there; the linker snippet reserves the block and lowers
  `_heap_sentry`. The engine owns the whole 128 KiB block during a capture,
  although a capture uses at most 64 KiB, so the library's other buffers live
  in block 3.
- On the RISC-V core the switch must land before the engine starts: a
  `fence rw, rw` and a read-back follow each write.
- Only block 2 can be the engine's. Block 3 ends in the ROM's .bss and .data
  (0x4087e610 to 0x40880000), which ROM functions use at any time, and a
  Zephyr image with Wi-Fi fills blocks 0 and 1 (the C6 build of `apps/rfprobe`
  has `_end` at 0x408366b0 and its libc heap above that up to block 2), so
  upstream's rotation over blocks 1 and 2 does not fit.

Evidence:

- `lib/esp_sdr/src/esp_sdr_rx.c:246-278`, `lib/esp_sdr/src/esp_sdr.c:123-133` (save and restore)
- *measured* bench 2026-10-08, `lib/esp_sdr/esp_sdr_bank_c6.ld:18-20` (whole block)
- *working* `lib/esp_sdr/src/esp_sdr_rx.c:262-265, 274-277` (fence)
- *measured* build 2026-10-11 (only block 2); `modules/zephyr/soc/espressif/esp32c6/memory.h:29-40` (ROM .bss and .data); `lib/esp_sdr/esp_sdr_bank_c6.ld:29-30` (`_heap_sentry`, rfprobe heap)
- HP_SYSTEM_MAC_DUMP_ALLOC: `modules/hal/espressif/components/soc/esp32c6/register/soc/hp_system_reg.h:63-68`
- HP_SYSTEM_SRAM_USAGE: `modules/hal/espressif/components/soc/esp32c6/register/soc/hp_system_reg.h:46-62`

### Modem clocks and reset

Documented registers; the entries give the behaviour the HAL does not document.

#### Register MODEM_SYSCON_CLK_CONF_REG (0x600a9804)

Modem clock enables, forced on for the dump engine

| Bits | 31 | 30:22 | 21 | 20:0 |
|---|---|---|---|---|
| Field | MODEM_SYSCON_CLK_DATA_DUMP_EN | (documented) | MODEM_SYSCON_CLK_DATA_DUMP_MUX | (unknown) |

**MODEM_SYSCON_CLK_DATA_DUMP_EN** Written 1 with the rest of the register (0xffffffff). (R/W)

**MODEM_SYSCON_CLK_DATA_DUMP_MUX** Written 1 with the rest of the register (0xffffffff). (R/W)

The dump engine needs this register, MODEM_SYSCON_CLK_CONF1_REG and
MODEM_SYSCON_CLK_CONF_POWER_ST_REG forced on before each capture and each
playback, as the vendor's `rftest_init()`/`phy_set_clk_conf()` does. The
library writes 0xffffffff here, then 0x7ffff to MODEM_SYSCON_CLK_CONF1_REG,
then 0xffffffff to MODEM_SYSCON_CLK_CONF_POWER_ST_REG. What playback needs
besides: see DUMP_DAC_TRIG_REG.

Evidence:

- *working* `lib/esp_sdr/src/esp_sdr_rx.c:255-257`; *measured* bench 2026-10-11 (playback); *upstream* `modules/lib/esp-sdr/main/targets/esp32c6/chip.h:35-36`; `modules/hal/espressif/components/soc/esp32c6/include/modem/modem_syscon_reg.h:21-27, 82-87`

#### Register MODEM_SYSCON_CLK_CONF_POWER_ST_REG (0x600a980c)

Forced to 0xffffffff for the dump engine

| Bits | 31:8 | 7:0 |
|---|---|---|
| Field | (documented) | (unknown) |

Written 0xffffffff before each capture and each playback, the last of the three
clock writes (see MODEM_SYSCON_CLK_CONF_REG).

Evidence:

- *working* `lib/esp_sdr/src/esp_sdr_rx.c:255-257`; *measured* bench 2026-10-11 (playback); *upstream* `modules/lib/esp-sdr/main/targets/esp32c6/chip.h:35-36`; `modules/hal/espressif/components/soc/esp32c6/include/modem/modem_syscon_reg.h:151`

#### Register MODEM_SYSCON_MODEM_RST_CONF_REG (0x600a9810)

Modem resets, pulsed at boot against stale modem state

| Bits | 31:29 | 28 | 27:24 | 23 | 22 | 21:19 | 18:14 | 13:11 | 10 | 9 | 8 | 7:0 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Field | (documented) | (unknown) | (documented) | (unknown) | (documented) | (unknown) | (documented) | (unknown) | (documented) | (unknown) | (documented) | (unknown) |

Undocumented: the modem subsystem keeps the previous firmware's state across a
chip reset (USB-Serial-JTAG, esptool). The library writes 0xffffffff, then 0,
here and 0xf, then 0, to MODEM_LPCON_RST_CONF_REG at `PRE_KERNEL_1`, before the
Wi-Fi driver starts, through `modem_syscon_ll_reset_all()` and
`modem_lpcon_ll_reset_all()`, as ESP-IDF's
`esp_system_reset_modules_on_exit()` does on a restart.

Evidence:

- *measured* bench 2026-10-09 (symptoms under "Stale modem state"); *working* `lib/esp_sdr/src/esp_sdr.c:29-48`; `modules/hal/espressif/components/hal/esp32c6/include/hal/modem_syscon_ll.h:325-329`, `modules/hal/espressif/components/esp_system/port/soc/esp32c6/system_internal.c:45-46`

#### Register MODEM_SYSCON_CLK_CONF1_REG (0x600a9814)

Bits 18:0 forced on for the dump engine

| Bits | 31:24 | 23:0 |
|---|---|---|
| Field | (unknown) | (documented) |

Bits 18:0 written 1 (0x7ffff) before each capture and each playback, the second
of the three clock writes (see MODEM_SYSCON_CLK_CONF_REG).

Evidence:

- *working* `lib/esp_sdr/src/esp_sdr_rx.c:255-257`; *measured* bench 2026-10-11 (playback); *upstream* `modules/lib/esp-sdr/main/targets/esp32c6/chip.h:35-36`; `modules/hal/espressif/components/soc/esp32c6/include/modem/modem_syscon_reg.h:281`

#### Register MODEM_SYSCON_WIFI_BB_CFG_REG (0x600a981c)

Wi-Fi baseband configuration; the 802.11p clock mode in bits 4:2

| Bits | 31:0 |
|---|---|
| Field | MODEM_SYSCON_WIFI_BB_CFG |

**MODEM_SYSCON_WIFI_BB_CFG** Bits 4:2: `phy_11p_set(1, m)` writes 4 for m = 0, else 5, presumably for 10 and 5 MHz channels, and clears AGC_R30_REG bit 5. Whether the dump follows the slower clock is unknown (a lead under "Dump and DAC engine"). (R/W)

Evidence:

- MODEM_SYSCON_WIFI_BB_CFG: *decompile* C6 `libphy.a` `phy_feature.o`

### Modem LP reset

Documented register; the behaviour the HAL does not document is under
MODEM_SYSCON_MODEM_RST_CONF_REG.

#### Register MODEM_LPCON_RST_CONF_REG (0x600af024)

Modem LP resets, pulsed at boot against stale modem state

| Bits | 31:4 | 3 | 2 | 1 | 0 |
|---|---|---|---|---|---|
| Field | (unknown) | MODEM_LPCON_RST_LP_TIMER | MODEM_LPCON_RST_I2C_MST | MODEM_LPCON_RST_COEX | MODEM_LPCON_RST_WIFIPWR |

**MODEM_LPCON_RST_LP_TIMER** Set, then cleared, at boot (0xf, then 0). (WO)

**MODEM_LPCON_RST_I2C_MST** Set, then cleared, at boot (0xf, then 0). (WO)

**MODEM_LPCON_RST_COEX** Set, then cleared, at boot (0xf, then 0). (WO)

**MODEM_LPCON_RST_WIFIPWR** Set, then cleared, at boot (0xf, then 0). (WO)

The library writes 0xf, then 0, at `PRE_KERNEL_1` through
`modem_lpcon_ll_reset_all()`, with MODEM_SYSCON_MODEM_RST_CONF_REG (see there
for why).

Evidence:

- *measured* bench 2026-10-09 (symptoms under "Stale modem state"); *working* `lib/esp_sdr/src/esp_sdr.c:29-48`; `modules/hal/espressif/components/hal/esp32c6/include/hal/modem_lpcon_ll.h:266-270`, `modules/hal/espressif/components/esp_system/port/soc/esp32c6/system_internal.c:45-46`

### Modem diagnostic lines

#### Register IO_MUX_MODEM_DIAG_EN_REG (0x600900bc)

Routes modem diagnostic lines into the GPIO matrix

| Bits | 31:0 |
|---|---|
| Field | IO_MUX_MODEM_DIAG_EN |

**IO_MUX_MODEM_DIAG_EN** Bit i routes modem_diag[i] into the GPIO matrix. After writing their selectors (DUMP_DIAG_SEL0_REG, DUMP_DIAG_SEL1_REG), `gpio_diag_init_ble()` enables 0xffff0000 here and `coex_hw_debug_matrix_config()` 0x0000000f. A lead for a continuous receive below 80 MS/s (see "Dump and DAC engine"). (R/W)

Evidence:

- IO_MUX_MODEM_DIAG_EN: `modules/hal/espressif/components/soc/esp32c6/register/soc/io_mux_reg.h:325-336`; *decompile* C6 `libble_app.a` `gpio_diag.c.o`, `libcoexist.a` `coexist_hw.o`

### BTBB page (0x600a2000)

#### Register BTBB_RB4_REG (0x600a20b4)

Dump gate in bit 0

| Bits | 31:1 | 0 |
|---|---|---|
| Field | (unknown) | BTBB_DUMP_GATE |

**BTBB_DUMP_GATE** Cleared before every capture, as the stock sequence. The C6 `adctrig()` clears it in every mode but 8, the BLE baseband dump, which sets it (see "Receive rate" under "Dump and DAC engine"); upstream's ESP32-H2 notes call it the trigger source. (R/W)

Evidence:

- BTBB_DUMP_GATE: *working* `lib/esp_sdr/src/esp_sdr_rx.c:261`; *decompile* C6 `librftest.a` `mac_common.o`; *upstream* `modules/lib/esp-sdr/main/targets/esp32c6/chip.h:40`, `modules/lib/esp-sdr/docs/esp32h2.md:20`

#### Register BTBB_R8A0_REG (0x600a28a0)

Unknown; bits 31:30 written by the BT branch of `force_txon_mode()`

| Bits | 31:30 | 29:0 |
|---|---|---|
| Field | BTBB_B31_30 | (unknown) |

**BTBB_B31_30** Only with the BT flag (its second argument), `force_txon_mode(on, bt, idx)` stops the hardware frequency engine (FE_HW_FREQ_STOP, 0x600a00c0 bit 26), writes 0x600af4b4 bit 0, FE_BT_FORCE (0x600a0910 bits 13:12) and these bits (all ones with `on`, 0 without), and restarts the engine if it ran before (see "Front end"). (R/W)

### Wi-Fi MAC page (0x600a4000)

#### Register WIFIMAC_EVENT_SEL_REG (0x600a4e30)

Wi-Fi MAC event selector, used by the dump's `adctrig()` mode 7

| Bits | 31:9 | 8 | 7:0 |
|---|---|---|---|
| Field | (unknown) | WIFIMAC_B8 | WIFIMAC_EVENT |

**WIFIMAC_B8** Set by `adctrig()` mode 7. (R/W)

**WIFIMAC_EVENT** The event, written by `adctrig()` mode 7 (DUMP_SRC 5; see "Receive rate" under "Dump and DAC engine"). (R/W)

Evidence:

- *decompile* C6 `librftest.a` `mac_common.o`

### AGC force

#### Register AGC_FORCE_REG (0x600a702c)

Forced receive gain index, force bit and largest calibrated index

| Bits | 31:24 | 23 | 22:15 | 14:8 | 7:0 |
|---|---|---|---|---|---|
| Field | AGC_FORCE_INDEX | AGC_FORCE_EN | (unknown) | AGC_GAIN_MAX | (unknown) |

**AGC_FORCE_INDEX** Forced receive gain index. (R/W)

**AGC_FORCE_EN** Force: 1 holds the index, 0 hands the gain to the AGC. (R/W)

**AGC_GAIN_MAX** Largest calibrated gain index, as on the S3: below 80 (library cap 79). Larger reads point at uncalibrated slots; the library treats them as 0. (R/W)

Same layout as the S3's 0x6001c02c; upstream calls the register AGCPWR_CTRL7.
On the C6, `force_rx_gain(force, index, 0)` writes only this register. A new
forced index alone can leave stale receive state, so a gain change reruns the
front-end setup.

Evidence:

- *working* `lib/esp_sdr/src/esp_sdr_priv.h:110-112`; *decompile* C6 `librftest.a` `phy_test.o` (`force_rx_gain`); *upstream* `modules/lib/esp-sdr/main/common/burst_gain.h:11-12, 36`
- `lib/esp_sdr/src/esp_sdr_rx.c:164-166` (front-end rerun)

#### Register AGC_R30_REG (0x600a7030)

Unknown; bit 5 cleared in the 802.11p clock mode

| Bits | 31:6 | 5 | 4:0 |
|---|---|---|---|
| Field | (unknown) | AGC_B5 | (unknown) |

**AGC_B5** `phy_11p_set(1, m)` writes MODEM_SYSCON_WIFI_BB_CFG_REG bits 4:2 and clears this bit (the 802.11p clock mode, a lead under "Dump and DAC engine"). (R/W)

Evidence:

- AGC_B5: *decompile* C6 `libphy.a` `phy_feature.o`

#### Register AGC_RB8_REG (0x600a70b8)

Unknown; bits 2:0 written by `set_dump_mode()`

| Bits | 31:3 | 2:0 |
|---|---|---|
| Field | (unknown) | AGC_B2_0 |

**AGC_B2_0** `set_dump_mode(x)` clears FE_R958_REG (0x600a0958) bits 1:0 and writes 1 here when x is 0, else 0. (R/W)

Evidence:

- AGC_B2_0: C6 `librftest.a` `phy_test.o`

#### Register AGC_RDC_REG (0x600a70dc)

Unknown; bit 16 cleared by `force_txon()`

| Bits | 31:17 | 16 | 15:0 |
|---|---|---|---|
| Field | (unknown) | AGC_B16 | (unknown) |

**AGC_B16** `force_txon(on)` clears it and sets FE_RX_FORCE (0x600a0910 bits 9:8) to 2, then FE_TX_FORCE (bits 11:10) to 3; off goes in reverse. `force_txon_mode()` calls `force_txon(0)`, then `force_txon(on)` (see "Front end"). (R/W)

### Front end

Unused by the library (it has no C6 transmit path). Evidence marked *decompile* in this
section comes from C6 `libphy.a` `phy_reg.o`, `phy_pbus.o`, `phy_init.o` and
`phy_tx_cal.o`, `librftest.a` `rf_test.o`, `esp_phy_api.o`, `phy_test.o` and `wifi.o`,
and the C6 ROM.

How the vendor functions drive the front end:

- `esp_phy_wifi_tx_tone(start, chan, backoff)` (`wifiscwout()`) tunes with
  `chip_v7_set_chan(chan, 0)` (above 2000 the argument is MHz, truncated to a channel:
  2437 to 2441 all give channel 6), forces the analog TX chain on through pbus debug
  mode (`ram_txcal_debuge_mode()`: `pbus_force_mode(1)`, `ram_pbus_xpd_tx_on()`, the
  calibrated DC offset, the TX clock), and starts tone 0 at frequency 0, attenuated by
  the calibrated value + backoff + 12 in quarter dB. It never calls `force_txon()`.
  Stopping runs ROM `txcal_work_mode()` (0x400047d6): tone off, TX clock off, pbus back
  to receive and to normal mode. On the bench the carrier arrives at the channel centre
  (*measured* bench 2026-10-11).
- `ram_stop_tx_tone()` stops the tones and selects the data path, leaving the pbus
  forcing and the TX clock on: the C6 counterpart of the C5's `phy_stop_tx_tone(1)`.
  The DAC engine played with the source left at 0 (see "Dump and DAC engine"); with 3
  it is untested.
- `force_txon(on)` clears `0x600a70dc` bit 16 and sets FE_RX_FORCE to 2, then
  FE_TX_FORCE to 3; off goes in reverse.
- In `force_txon_mode(on, bt, idx)` the second argument, which the ESP32-S3 page calls
  mode, acts as a BT flag: the function always writes the power index force
  (FE_PWR_INDEX_FORCE = `on`, index in FE_PWR_INDEX), then calls `force_txon(0)` and
  `force_txon(on)`; only with the flag it first adds 32 to the index, stops the
  hardware frequency engine (FE_HW_FREQ_STOP), writes `0x600af4b4` bit 0, FE_BT_FORCE
  and `0x600a28a0` bits 31:30 (all ones with `on`, 0 without), and restarts the engine
  if it ran before.
- `set_dump_mode(x)` clears FE_R958_REG bits 1:0 and writes `0x600a70b8` bits 2:0 = 1
  when x is 0, else 0 (C6 `librftest.a` `phy_test.o`).

#### Register FE_RC0_REG (0x600a00c0)

Hardware frequency engine stop

| Bits | 31:27 | 26 | 25:0 |
|---|---|---|---|
| Field | (unknown) | FE_HW_FREQ_STOP | (unknown) |

**FE_HW_FREQ_STOP** The hardware frequency engine stop: the BT branch of `force_txon_mode()` stops the engine (this bit) and restarts it afterwards if it ran before. (R/W)

#### Register FE_PWR_INDEX_REG (0x600a0410)

Power index force

| Bits | 31:24 | 23 | 22 | 21:16 | 15:0 |
|---|---|---|---|---|---|
| Field | (unknown) | FE_PWR_INDEX_FORCE | (unknown) | FE_PWR_INDEX | (unknown) |

**FE_PWR_INDEX_FORCE** Forces the power index FE_PWR_INDEX. `force_txon_mode()` sets it to its `on` argument, so `force_txon_mode(0, ...)` releases the force. (R/W)

**FE_PWR_INDEX** The forced power index: `force_txon_mode(on, bt, idx)` writes idx here, plus 32 with the BT flag. (R/W)

Evidence:

- FE_PWR_INDEX_FORCE: *decompile* C6 `libphy.a` `phy_reg.o` (`force_txon_mode()` passes `on != 0`), C6 ROM `phy_force_pwr_index` (0x400057e2, bit 23 = its first argument)

#### Register FE_DAC_SRC_REG (0x600a0414)

DAC source select

| Bits | 31:2 | 1:0 |
|---|---|---|
| Field | (unknown) | FE_DAC_SRC |

**FE_DAC_SRC** DAC source select as the vendor uses it: `ram_start_tx_tone_step()` writes 0 with a tone and 3 without, `ram_stop_tx_tone()` writes 3; the ROM tone functions leave it alone. The DAC engine plays with 0 (see "Dump and DAC engine"); with 3 it is untested. (R/W)

Evidence:

- FE_DAC_SRC: *decompile*; *measured* bench 2026-10-11

#### Register FE_TXDC_CAL_REG (0x600a0418)

TX DC calibration comparator

| Bits | 31:30 | 29 | 28 | 27:23 | 22 | 21:2 | 1 | 0 |
|---|---|---|---|---|---|---|---|---|
| Field | (unknown) | FE_TXDC_CMP_ROW2 | FE_TXDC_CMP_ROW3 | (unknown) | FE_TXDC_CAL_DONE | (unknown) | FE_TXDC_CAL_EN | FE_TXDC_CAL_START |

**FE_TXDC_CMP_ROW2** Comparator output for pbus row 2 (presumably I). (RO)

**FE_TXDC_CMP_ROW3** Comparator output for pbus row 3 (presumably Q). (RO)

**FE_TXDC_CAL_DONE** Calibration done. (RO)

**FE_TXDC_CAL_EN** Calibration enable. (R/W)

**FE_TXDC_CAL_START** Calibration start. (R/W)

Layout as libphy `txdc_cal_new()` drives it. The result goes into pbus rows 2 and 3
through `rom_pbus_force_test()`. The ROM `txdc_cal()` (0x4000761e) uses another
layout on the same register and no C6 library calls it. Shell writes sometimes made
a carrier, not reproducibly.

Evidence:

- *decompile* `libphy.a` `phy_tx_cal.o`, C6 ROM; *measured* bench 2026-10-11

#### Register FE_TONE0_REG, FE_TONE1_REG (0x600a0420, 0x600a0424)

Tones 0 and 1: enable, attenuation, frequency

| Bits | 31:19 | 18 | 17:10 | 9:0 |
|---|---|---|---|---|
| Field | (unknown) | FE_TONE_EN | FE_TONE_ATTEN | FE_TONE_FREQ |

**FE_TONE_EN** Tone enable. (R/W)

**FE_TONE_ATTEN** The attenuation, negated. (R/W)

**FE_TONE_FREQ** The frequency >> 2. libphy `ram_start_tx_tone_step()` puts the frequency's low two bits in FE_TONE_FREQ_LO_REG; the ROM version splits the frequency so only while FE_TONE0_REG bit 29 is set, else it writes it unshifted and leaves FE_TONE_FREQ_LO_REG alone. (R/W)

The vendor keeps bits 31:28 and writes 0 to bits 27:19. The vendor CW leaves tone 0
at 0x00068400 or 0x00068c00: frequency 0, a carrier at the LO.

Evidence:

- *decompile*; *measured* bench 2026-10-11

#### Register FE_TONE_FREQ_LO_REG (0x600a042c)

Fine tone frequency: the low two bits of each tone's frequency

| Bits | 31:4 | 3:2 | 1:0 |
|---|---|---|---|
| Field | (unknown) | FE_TONE1_FREQ_LO | FE_TONE0_FREQ_LO |

**FE_TONE1_FREQ_LO** Low two bits of tone 1's frequency. (R/W)

**FE_TONE0_FREQ_LO** Low two bits of tone 0's frequency. (R/W)

libphy `ram_start_tx_tone_step()` puts the frequency's low two bits here. The ROM
version does so only while FE_TONE0_REG bit 29 is set, else it writes the frequency
unshifted and leaves this register alone.

Evidence:

- *decompile*

#### Register FE_TXDC_REG (0x600a0444)

TX DC value

| Bits | 31:20 | 19:10 | 9:0 |
|---|---|---|---|
| Field | (unknown) | FE_TXDC_Q | FE_TXDC_I |

**FE_TXDC_Q** TX DC value, Q. (R/W)

**FE_TXDC_I** TX DC value, I. (R/W)

The C6 `set_txdc_dig(i, q)`, the only code that writes the register, writes only
FE_TXDC_I and FE_TXDC_Q; the identical C61 function writes 0x600a0c08, the register
upstream's first C5 code drove CPU-paced I/Q through (`TXDC_REG`, `emit_iq()`).
Untested on the C6.

Evidence:

- *decompile* C6 and C61 `librftest.a` `phy_test.o`; *upstream* `git show e605684:main/c5_main.c:32, 211-238`

#### Register FE_R44C_REG (0x600a044c)

Unknown; init sets bits 27 and 25 and clears bit 22

| Bits | 31:28 | 27 | 26 | 25 | 24:23 | 22 | 21:0 |
|---|---|---|---|---|---|---|---|
| Field | (unknown) | FE_B27 | (unknown) | FE_B25 | (unknown) | FE_B22 | (unknown) |

**FE_B27** Set at init. (R/W)

**FE_B25** Set at init. (R/W)

**FE_B22** Cleared at init. (R/W)

ROM `fe_reg_init()` and libphy `reg_set_new()` set bits 27 and 25 and clear bit 22
at init, at the step where the C61's `phy_fe_reg_init()` sets bits 25 (IQ swap) and
26 (inject) of its TX DC register 0x600a0c08. That 25 and 27 are the C6's IQ swap
and inject bits is a guess.

Evidence:

- *decompile* (the writes; C61 `libphy.a` `phy_reg.o`); *speculative* (the meaning)

#### Register FE_FORCE_REG (0x600a0910)

Receive, transmit and BT mode force pairs; receive and transmit clocks

| Bits | 31:18 | 17:16 | 15:14 | 13:12 | 11:10 | 9:8 | 7:0 |
|---|---|---|---|---|---|---|---|
| Field | (unknown) | FE_TX_CLK | FE_RX_CLK | FE_BT_FORCE | FE_TX_FORCE | FE_RX_FORCE | (unknown) |

**FE_TX_CLK** Transmit clock, 3 on, 0 off, as ROM `set_txclk_en()` (0x40005268) writes it. (R/W)

**FE_RX_CLK** Receive clock, 3 on, 0 off, as ROM `set_rxclk_en()` (0x4000528c) writes it. (R/W)

**FE_BT_FORCE** BT mode force pair: 3 from the BT branch of `force_txon_mode()`, `txon_set(1)` and `bt_rx_force()`. (R/W)

**FE_TX_FORCE** Transmit force pair: 3 from `force_txon(1)` and `txon_set(1)`; 2 from `force_txrx_off(1)` and ROM `phy_close_pa(1)`. (R/W)

**FE_RX_FORCE** Receive force pair: 2 from `force_txon(1)`, `txon_set(1)` and `force_txrx_off(1)`; 3 from `bt_rx_force()` and ROM `fe_adc_on()`. (R/W)

FE_RX_FORCE, FE_TX_FORCE and FE_BT_FORCE are force pairs, high bit force, low bit
value: 0 released, 2 forced off, 3 forced on (inferred from all writers).

Evidence:

- *decompile*

#### Register FE_R958_REG (0x600a0958)

Unknown; `set_dump_mode()` clears bits 1:0

| Bits | 31:2 | 1:0 |
|---|---|---|
| Field | (unknown) | FE_B1_0 |

**FE_B1_0** Cleared by `set_dump_mode(x)`. (R/W)

Evidence:

- FE_B1_0: C6 `librftest.a` `phy_test.o`

### Analog I2C master

`I2C_ANA_MST` at 0x600af800 is in the HAL; the register names are the HAL's.

#### Register I2C_ANA_MST_I2C0_CTRL_REG, I2C_ANA_MST_I2C1_CTRL_REG (0x600af800, 0x600af804)

Command word of master 0 and master 1

| Bits | 31:27 | 26 | 25 | 24 | 23:16 | 15:8 | 7:0 |
|---|---|---|---|---|---|---|---|
| Field | (unknown) | I2C_ANA_MST_B26 | I2C_ANA_MST_BUSY | I2C_ANA_MST_WRITE | I2C_ANA_MST_DATA | I2C_ANA_MST_REG_ADDR | I2C_ANA_MST_BLOCK |

**I2C_ANA_MST_B26** Unknown. The ROM sets it in every command word, as on the ESP32-S3; the HAL's own access code does not set it. (R/W)

**I2C_ANA_MST_BUSY** Busy; polled after each command word. (RO)

**I2C_ANA_MST_WRITE** Set for a write, clear for a read. (R/W)

**I2C_ANA_MST_DATA** Data to write; after a read, the result. (R/W)

**I2C_ANA_MST_REG_ADDR** Register number in the block. (R/W)

**I2C_ANA_MST_BLOCK** Analog block id. (R/W)

The ROM uses the same command word as the ESP32-S3: one command word per access,
then poll I2C_ANA_MST_BUSY; block, register and I2C_ANA_MST_B26 for a read
(0x04000000 base), plus data and I2C_ANA_MST_WRITE for a write (0x05000000 base).
The result is in I2C_ANA_MST_DATA.

Evidence:

- `modules/hal/espressif/components/soc/esp32c6/include/modem/i2c_ana_mst_reg.h:15-45`; *decompile* C6 ROM 0x40003d88
- I2C_ANA_MST_B26: `modules/hal/espressif/components/esp_hal_regi2c/esp32c6/regi2c_impl.c:112-180`

#### Register I2C_ANA_MST_ANA_CONF1_REG (0x600af81c)

Read mask

| Bits | 31:24 | 23:0 |
|---|---|---|
| Field | (documented) | I2C_ANA_MST_ANA_CONF1 |

**I2C_ANA_MST_ANA_CONF1** The read mask. (R/W)

Evidence:

- `modules/hal/espressif/components/soc/esp32c6/include/modem/i2c_ana_mst_reg.h:105-150`

#### Register I2C_ANA_MST_ANA_CONF2_REG (0x600af820)

Master select

| Bits | 31:24 | 23:0 |
|---|---|---|
| Field | (documented) | I2C_ANA_MST_ANA_CONF2 |

**I2C_ANA_MST_ANA_CONF2** The master select. The ROM's `get_i2c_hostid()` keeps the register's bits outside 17:4 and ORs in 0x1fe00. (R/W)

Evidence:

- `modules/hal/espressif/components/soc/esp32c6/include/modem/i2c_ana_mst_reg.h:105-150`
- I2C_ANA_MST_ANA_CONF2: *decompile* C6 ROM 0x40003d4a

### Analog I2C block 0x61 (ULP)

Registers 0 to 10 at 2412 MHz: `1f 02 08 49 53 40 51 2f 33 59 20` (bench
2026-10-09). The Init rows below come from this dump.

#### Register ULP_R4 (0x61:4)

HAL `I2C_ULP_OCODE`, function undocumented

| Bits | 7:0 |
|---|---|
| Field | ULP_OCODE |
| Init | 0x53 |

**ULP_OCODE** HAL name `I2C_ULP_OCODE`; the function is undocumented. Reads 0x4f instead of 0x53 after the native 802.15.4 radio ran in the previous firmware (see "Stale modem state"). (R/W)

Evidence:

- ULP_OCODE: *measured* bench 2026-10-09; `modules/hal/espressif/components/soc/esp32c6/include/soc/regi2c_lp_bias.h:45`

#### Register ULP_R9 (0x61:9)

Unknown

| Bits | 7:0 |
|---|---|
| Field | (unknown) |
| Init | 0x59 |

Reads 0x5a in the same stale state as ULP_OCODE (see "Stale modem
state").

Evidence:

- *measured* bench 2026-10-09

### Analog I2C block 0x62 (RF PLL)

The RF PLL's VCO and its calibration (the S3 has the same block). The library reads
RFPLL_VCO_WINDOW after every tune and drives the LO 5/6 divider through
RFPLL_LO_DIV. The capacitor code is `0x62:5 | 0x62:7[2] << 8` (9 bits; the
vendor clamps to 0x1ff); a higher code means a lower VCO frequency.

Some field meanings come from the S3 vendor code, as the fields and their evidence
say.

Upstream esp-sdr does a manual 512-code search on the ESP32-S2 through the same
fields (RFPLL_MANUAL_CODE, RFPLL_MANUAL_CODE_B8, RFPLL_CODE_SRC, `0x62:0[7]`,
and RFPLL_VCO_WINDOW as the window indication); it does not apply it to the S3 or
the C6 (*upstream* `modules/lib/esp-sdr/main/targets/esp32s2/pll.h:8-36`).

Registers 0 to 18 at 2412 MHz:
`68 bb 8d 61 28 bb 0d 03 80 f0 a0 44 00 00 14 74 07 ff 60` (bench 2026-10-09).
The Init rows below come from this dump.

#### Register RFPLL_R0 (0x62:0)

VCO calibration start

| Bits | 7 | 6 | 5 | 4:0 |
|---|---|---|---|---|
| Field | (unknown) | RFPLL_R0_B6 | RFPLL_CAL_START | (unknown) |
| Init | 0 | 1 | 1 | 0x8 |

**RFPLL_R0_B6** Unknown; toggled around the calibration start (see RFPLL_CAL_START). (R/W)

**RFPLL_CAL_START** `restart_cal()` starts the VCO calibration with a rising edge on this bit while RFPLL_R0_B6 is toggled: the C6 clears RFPLL_R0_B6, pulses this bit and sets RFPLL_R0_B6; the S3 sets it first and clears it after. (WO)

Evidence:

- *decompile* C6 ROM `restart_cal` (0x4000587e), S3 `phy_rfpll.o`; *measured* C6 dump

#### Register RFPLL_R1 (0x62:1)

Manual capacitor code, bits 7:0

| Bits | 7:0 |
|---|---|
| Field | RFPLL_MANUAL_CODE |
| Init | 0xbb |

**RFPLL_MANUAL_CODE** Bits 7:0 of a manual capacitor code. The meaning comes from the S3 vendor code, where `write_pll_cap()` writes the code here. After a tune it holds the low byte of the code in use (on the C6 it reads as RFPLL_R5). (R/W)

Init: 0xbb, the same as RFPLL_R5 (see also "Stale modem state")

Evidence:

- RFPLL_MANUAL_CODE: *decompile* S3 `phy_rfpll.o`; *measured* C6 dump

#### Register RFPLL_R2 (0x62:2)

Manual code bit 8 and a 4-bit code override

| Bits | 7 | 6:5 | 4 | 3:0 |
|---|---|---|---|---|
| Field | RFPLL_CODE4_OVR | (unknown) | RFPLL_MANUAL_CODE_B8 | RFPLL_CODE4_VAL |
| Init | 1 | 0x0 | 0 | 0xd |

**RFPLL_CODE4_OVR** Cleared before and set after the vendor copies the calibrated 4-bit code into RFPLL_CODE4_VAL (see below). (R/W)

**RFPLL_MANUAL_CODE_B8** Bit 8 of the manual capacitor code, written with RFPLL_MANUAL_CODE. The meaning comes from the S3 vendor code. (R/W)

**RFPLL_CODE4_VAL** The 4-bit code of the override, copied from RFPLL_CODE4_CAL (see below). (R/W)

Init: 0x8d, read with RFPLL_R6 = 0x0d

The 4-bit code override: the S3's `get_rf_freq_init()` clears RFPLL_CODE4_OVR,
calibrates at 2437 MHz, copies RFPLL_CODE4_CAL into RFPLL_CODE4_VAL and sets
RFPLL_CODE4_OVR. What the 4-bit code controls is unknown.

Evidence:

- *decompile* S3 `phy_hw_freq.o` (override sequence); *measured* C6 dump
- RFPLL_MANUAL_CODE_B8: *decompile* S3 `phy_rfpll.o`

#### Register RFPLL_R4 (0x62:4)

Unknown

| Bits | 7:0 |
|---|---|
| Field | (unknown) |
| Init | 0x28 |

Init: 0x28 (40) at every PLL frequency read, 2280 to 2438 MHz

Evidence:

- *measured* bench 2026-10-09

#### Register RFPLL_R5 (0x62:5)

Capacitor code readback, bits 7:0

| Bits | 7:0 |
|---|---|
| Field | RFPLL_CAP_CODE |
| Init | 0xbb |

**RFPLL_CAP_CODE** Bits 7:0 of the capacitor code; bit 8 is RFPLL_CAP_CODE_B8. Values: see "Measured PLL behaviour". (RO)

Evidence:

- RFPLL_CAP_CODE: *decompile* S3 `phy_rfpll.o`; *measured*

#### Register RFPLL_R6 (0x62:6)

Calibrated 4-bit code

| Bits | 7:4 | 3:0 |
|---|---|---|
| Field | (unknown) | RFPLL_CODE4_CAL |
| Init | 0x0 | 0xd |

**RFPLL_CODE4_CAL** The calibrated 4-bit code. The vendor copies it into RFPLL_CODE4_VAL (see RFPLL_R2). (RO)

Evidence:

- RFPLL_CODE4_CAL: *decompile* as RFPLL_R2

#### Register RFPLL_R7 (0x62:7)

Capacitor code bit 8 and calibration done

| Bits | 7:3 | 2 | 1 | 0 |
|---|---|---|---|---|
| Field | (unknown) | RFPLL_CAP_CODE_B8 | RFPLL_CAL_DONE | (unknown) |
| Init | 0x0 | 0 | 1 | 1 |

**RFPLL_CAP_CODE_B8** Bit 8 of the capacitor code readback (see RFPLL_CAP_CODE). (RO)

**RFPLL_CAL_DONE** Calibration done; 1 after every tune. The S3's `wait_rfpll_cal_end()` polls it every 20 us, 100 times, then prints `error: pll_cal exceeds 2ms!!!`. (RO)

Evidence:

- RFPLL_CAP_CODE_B8: *decompile* S3 `phy_rfpll.o`; *measured*
- RFPLL_CAL_DONE: *decompile* S3 `phy_rfpll.o`; *measured*

#### Register RFPLL_R11 (0x62:11)

Capacitor code source

| Bits | 7 | 6 | 5:0 |
|---|---|---|---|
| Field | (unknown) | RFPLL_CODE_SRC | (unknown) |
| Init | 0 | 1 | 0x4 |

**RFPLL_CODE_SRC** Cleared before each software calibration (`set_rfpll_freq()`), set by `rfpll_cap_init_cal()` before it tries manual codes. Inferred: 0 = code from the calibration engine, 1 = code from RFPLL_MANUAL_CODE and RFPLL_MANUAL_CODE_B8. (R/W)

Evidence:

- RFPLL_CODE_SRC: *decompile* C6 ROM `set_rfpll_freq` and `rfpll_cap_init_cal`, S3 `phy_rfpll.o`

#### Register RFPLL_R12 (0x62:12)

VCO window comparator and two unknown status bits

| Bits | 7:4 | 3:2 | 1 | 0 |
|---|---|---|---|---|
| Field | (unknown) | RFPLL_VCO_WINDOW | RFPLL_R12_B1 | RFPLL_R12_B0 |
| Init | 0x0 | 0x0 | 0 | 0 |

**RFPLL_VCO_WINDOW** After calibration: 0 = the VCO inside its tuning window; 1 = target below the window (code pinned at its maximum); 2 = target above it (code 0). Live for the current code: the S3 vendor's `rfpll_cap_correct()` moves the code in steps of 2, up while it reads 1 and down while it reads 2. `esp_sdr_set_freq()` and `esp_sdr_set_freq_offset()` return -ERANGE when it is not 0. Window (PLL): 2140 to 2884 MHz. (RO)

**RFPLL_R12_B1** Unknown; set from about 2700 MHz (PLL) up, on both chips. (RO)

**RFPLL_R12_B0** Unknown; set below about 2412 MHz. (RO)

Evidence:

- RFPLL_VCO_WINDOW: *measured* bench 2026-10-09; *working* `lib/esp_sdr/src/esp_sdr.c:78-97, 194-196, 213-215`; *decompile* S3 `phy_rfpll.o`
- RFPLL_R12_B1: *measured* bench 2026-10-09
- RFPLL_R12_B0: *measured* bench 2026-10-09

#### Register RFPLL_R14 (0x62:14)

Charge pump calibration done and result

| Bits | 7 | 6:5 | 4:0 |
|---|---|---|---|
| Field | RFPLL_CHGP_CAL_DONE | (unknown) | RFPLL_CHGP_CAL_RESULT |
| Init | 0 | 0x0 | 0x14 |

**RFPLL_CHGP_CAL_DONE** Charge pump calibration done (see RFPLL_R15). (RO)

**RFPLL_CHGP_CAL_RESULT** Charge pump calibration result (see RFPLL_R15). (RO)

Evidence:

- *decompile* C6 `libphy.a` `phy_rfpll.o` (`rfpll_chgp_cal`)

#### Register RFPLL_R15 (0x62:15)

Charge pump calibration

| Bits | 7 | 6 | 5 | 4:0 |
|---|---|---|---|---|
| Field | (unknown) | RFPLL_R15_B6 | RFPLL_R15_B5 | RFPLL_CHGP_CODE |
| Init | 0 | 1 | 1 | 0x14 |

**RFPLL_R15_B6** Cleared before the charge pump calibration and set after it (see below). (R/W)

**RFPLL_R15_B5** Pulsed before RFPLL_CHGP_CAL_DONE is polled (see below). (WO)

**RFPLL_CHGP_CODE** Written from the calibration result, min(31, 7r / 6 + 9) (see below). (R/W)

`rfpll_chgp_cal()` clears RFPLL_R15_B6, pulses RFPLL_R15_B5, polls
RFPLL_CHGP_CAL_DONE (20 us, 100 tries), reads r = RFPLL_CHGP_CAL_RESULT, sets
RFPLL_R15_B6 and writes RFPLL_CHGP_CODE = min(31, 7r / 6 + 9). The C6 read
RFPLL_R14 = 0x14 and RFPLL_R15 = 0x74 at 2412 MHz, which this formula does not
produce, so the reading is incomplete or RFPLL_R14 changes later.

Evidence:

- *decompile* C6 `libphy.a` `phy_rfpll.o` (`rfpll_chgp_cal`); *measured* C6 dump

#### Register RFPLL_R16 (0x62:16)

LO 5/6 divider select

| Bits | 7:4 | 3 | 2:0 |
|---|---|---|---|
| Field | (unknown) | RFPLL_LO_DIV | (unknown) |
| Init | 0x0 | 0 | 0x7 |

**RFPLL_LO_DIV** 0: the LO is the PLL. 1: LO = PLL / 1.2 (5/6 conversion). One selector feeds both the receive and the transmit mixer. Mask 0x08; the S3 has the same selector in block 0x65, which the C6 does not have. (R/W)

- Change only the bit (read-modify-write). Clear it before the vendor tuning
  call, which calibrates in normal mode; set it after the receive front-end
  setup.
- The library uses the divider for every frequency below 2210 MHz (PLL at 1.2
  times the LO), upstream only from 1842 MHz.
- A scan of the C6 blob objects found no access to `0x62:16`.
- Upstream qualifies the selector on the ESP32, S2, S3, C2, C3 and C6 and warns
  that the C5, C61, H2 and S31 lay out their analog blocks differently.

Evidence:

- *working* `lib/esp_sdr/src/esp_sdr.c:106-108`, `lib/esp_sdr/src/esp_sdr_rx.c:92-95` (when to clear and set)
- `lib/esp_sdr/src/esp_sdr.c:65-76`, `modules/lib/esp-sdr/main/common/rx_lo.h:15-19` (divider band)
- *decompile*, objdump (no access in the C6 blobs)
- `modules/lib/esp-sdr/docs/rx-controls.md:102-107` (qualified chips)
- RFPLL_LO_DIV: *upstream* `modules/lib/esp-sdr/main/common/rx_lo.h:47-62`, external-tone tests `modules/lib/esp-sdr/docs/rx-controls.md:76-100`; *measured* on air for receive and transmit (bench 2026-10-09)

#### Register RFPLL_R17 (0x62:17)

Unknown

| Bits | 7:0 |
|---|---|
| Field | (unknown) |
| Init | 0xff |

Init: 0xff at 2412 MHz; 0x7e at 2413 MHz

Evidence:

- *measured* C6 dump

#### Register RFPLL_R18 (0x62:18)

Unknown

| Bits | 7:0 |
|---|---|
| Field | (unknown) |
| Init | 0x60 |

Evidence:

- *measured* C6 dump

### Analog I2C block 0x63 (RF PLL fractional-N divider)

Registers 0 and 3 to 6:

`f_PLL = 1.5 * f_xtal * (32 + r3 + (r4 * 2^11 + r5 * 2^3 + r6[2:0]) / 2^19)`

That is 60 MHz per integer step with the 40 MHz crystal. The vendor computes a
word `W = r3 << 19 | r4 << 11 | r5 << 3 | r6` with
`f_PLL = 3 * f_xtal * (16 + W / 2^20)`; W bit 27 is computed but not written.
`write_rfpll_sdm()` clears RFPLL_SDM_R0_B3 (`0x63:0[3]`), writes registers 3 to 6
and sets the bit again.

| PLL MHz | reg 3 | reg 4 | reg 5 | reg 6 | Note |
|---|---|---|---|---|---|
| 2412 | 0x08 | 0x33 | 0x33 | 0x01 | |
| 2413 | 0x08 | 0x37 | 0x77 | 0x03 | |
| 2417 | 0x08 | 0x48 | 0x88 | 0x04 | |
| 2437 | 0x08 | 0x9d | 0xdd | 0x06 | |
| 2438 | 0x08 | 0xa2 | 0x22 | 0x01 | |
| 2280 | 0x06 | 0x00 | 0x00 | 0x00 | LO 1900 MHz through the divider |
| 2404 | 0x08 | 0x11 | 0x11 | 0x00 | asked for 2412 MHz in the stale modem state |

Evidence: *measured* bench 2026-10-09; *decompile* C6 ROM `write_rfpll_sdm`
(0x400058ee), C6 `libphy.a` `phy_rfpll.o` (`ram_rfpll_set_freq`).

The bench reading `60 MHz * (N + F / 2^20)` with N = 32 + reg 3 and
`F = r4 << 12 | r5 << 4 | r6` is the same formula except that the vendor
code weights reg 6 twice as much (it holds W bits 2:0), a difference below
0.2 kHz. The bench reading also took reg 3 as the low 4 bits of N; the vendor code
puts 8 bits there (N - 32), which agrees for every value seen.

The crystal selector of the vendor calls picks the divisor, as on the S3: 1 =
26 MHz, 2 = 32 MHz, anything else = 40 MHz (*decompile* C6 `ram_rfpll_set_freq`).

The Init rows below are the values at 2412 MHz.

#### Register RFPLL_SDM_R0 (0x63:0)

Bit 3 cleared and set around a divider write; other bits unknown

| Bits | 7:4 | 3 | 2:0 |
|---|---|---|---|
| Field | (unknown) | RFPLL_SDM_R0_B3 | (unknown) |
| Init | 0x5 | 1 | 0x3 |

**RFPLL_SDM_R0_B3** `write_rfpll_sdm()` clears it, writes RFPLL_SDM_R3 to RFPLL_SDM_R6 and sets it again. (R/W)

Evidence:

- *measured* bench 2026-10-09
- RFPLL_SDM_R0_B3: *decompile* C6 ROM `write_rfpll_sdm` (0x400058ee)

#### Register RFPLL_SDM_R1 (0x63:1)

Unknown

| Bits | 7:0 |
|---|---|
| Field | (unknown) |
| Init | 0xab |

Evidence:

- *measured* bench 2026-10-09

#### Register RFPLL_SDM_R3 (0x63:3)

Divider integer part

| Bits | 7:0 |
|---|---|
| Field | RFPLL_SDM_INTEGER |
| Init | 0x8 |

**RFPLL_SDM_INTEGER** r3 in the formula: N - 32. The vendor code puts 8 bits here. (R/W)

#### Register RFPLL_SDM_R4 (0x63:4)

Divider fraction, high byte

| Bits | 7:0 |
|---|---|
| Field | RFPLL_SDM_FRAC_H |
| Init | 0x33 |

**RFPLL_SDM_FRAC_H** r4 in the formula. (R/W)

#### Register RFPLL_SDM_R5 (0x63:5)

Divider fraction, middle byte

| Bits | 7:0 |
|---|---|
| Field | RFPLL_SDM_FRAC_M |
| Init | 0x33 |

**RFPLL_SDM_FRAC_M** r5 in the formula. (R/W)

#### Register RFPLL_SDM_R6 (0x63:6)

Divider fraction, low 3 bits

| Bits | 7:3 | 2:0 |
|---|---|---|
| Field | (unknown) | RFPLL_SDM_FRAC_L |
| Init | 0x0 | 0x1 |

**RFPLL_SDM_FRAC_L** r6[2:0] in the formula: W bits 2:0. (R/W)

### Analog I2C block 0x66 (BBPLL)

Documented by the HAL; esp_sdr does not touch the block. Upstream looked for other
ADC rates (on the S3 through this block); for the C6 it reports that no reliable
lower-rate path was found (*upstream* `modules/lib/esp-sdr/docs/rx-controls.md:59-61`).

Registers 0 to 10 at 2412 MHz: `18 25 50 08 6b 80 73 00 c2 96 03` (bench
2026-10-09).

### Analog I2C block 0x67 (BBTOP, analog baseband)

The C6 libphy groups the filter codes as the S3 does (`filter_dcap_set()`:
registers 4, 5, 12, 13 from one PHY parameter, 6, 7, 14, 15 from the next), so the
transmit pairs are probably at the same place as on the S3 (*decompile* C6
`libphy.a` `phy_i2c.o`). `esp_sdr_tx_set_lpf()` returns -ENOTSUP on the C6.
`esp_sdr_bbtop_read()` and `esp_sdr_bbtop_write()` give unchecked access
(`lib/esp_sdr/src/esp_sdr_rx.c:648-667`).

#### Register BBTOP_R4, BBTOP_R5 (0x67:4, 0x67:5)

Receive low-pass capacitor code, I (BBTOP_R4) and Q (BBTOP_R5)

| Bits | 7:6 | 5:0 |
|---|---|---|
| Field | (unknown) | BBTOP_RX_LPF |

**BBTOP_RX_LPF** Values 0 to 63; larger is narrower, 54 to 12 MHz on the C6. 0 is the widest setting, not a bypass. (R/W)

Bits 7:6 are kept.

Evidence:

- BBTOP_RX_LPF: *working* `lib/esp_sdr/src/esp_sdr_rx.c:126-148`; *upstream* curves `modules/lib/esp-sdr/main/common/rx_bandwidth.h:79-84`

#### Register BBTOP_R12, BBTOP_R13 (0x67:12, 0x67:13)

Probably the transmit low-pass pair A, as on the S3

| Bits | 7:0 |
|---|---|
| Field | (unknown) |

`filter_dcap_set()` writes them from the same PHY parameter as BBTOP_R4 and
BBTOP_R5 (see the introduction).

Evidence:

- *decompile* C6 `libphy.a` `phy_i2c.o`

#### Register BBTOP_R14, BBTOP_R15 (0x67:14, 0x67:15)

Probably the transmit low-pass pair B, as on the S3

| Bits | 7:0 |
|---|---|
| Field | (unknown) |

`filter_dcap_set()` writes them from the same PHY parameter as registers 6 and 7
(see the introduction).

Evidence:

- *decompile* C6 `libphy.a` `phy_i2c.o`

### Analog I2C block 0x6a (bias)

The HAL names the block (`I2C_BIAS`). Registers 0 to 3 at 2412 MHz read
`cf 7c 1a 07` (bench 2026-10-09).

<!-- svd:end -->

## Sample word formats

| Word | Bits 9:0 | Bits 19:10 | Bits 31:20 | Evidence |
|---|---|---|---|---|
| receive | Q (assumed) | I (assumed) | | the same helpers as on the S3 (`include/esp_sdr/esp_sdr_rx.h:209-225`); not checked against a tone |
| transmit, played by the DAC engine | I | Q | 0 | *measured* bench 2026-10-11 (tone side and image) |

- Each field is 10-bit two's complement, -512 to 511.
- Packed forms: IQ8 is bits 19:12 then 9:2 (I then Q); IQ10 packs bits 19:0 of
  two words into 5 bytes (`lib/esp_sdr/src/esp_sdr_rx.c:548-576`).

## Vendor library functions as used

| Function | Library | Called as | What it does | Evidence |
|---|---|---|---|---|
| `chip_v7_set_chan(mhz, mode)` | libphy | `(channel ? mhz : 2412, cbw)` | Channel setup; converts MHz through mhz2ieee and loses off-grid requests, hence the calibration channel plus `phy_set_freq()`. | *working* `lib/esp_sdr/src/esp_sdr.c:114-120`; *upstream* `modules/lib/esp-sdr/main/targets/esp32c6/chip.h:12-19` |
| `phy_set_freq(mhz, khz)` | libphy | `(PLL MHz, kHz + offset)` | Stores khz at `phy_param + 0x20` and calls `set_rf_freq_offset(phy_param[81], mhz, khz)` (crystal selector from the PHY). | *decompile* C6 `libphy.a` `phy_rfpll.o` |
| `ram_stop_tx_tone`, `pbus_workmode`, `rom_pbus_xpd_tx_off`, `ram_pbus_xpd_rx_on`, `set_rxclk_en` | libphy, ROM | RX: `stop_tx_tone(1)`, `workmode()`, `xpd_tx_off()`, `xpd_rx_on(1)`, `set_rxclk_en(1)` | Forces the analog chain and the receive clock on with the MAC idle, so the engine sees live ADC data (sequence from upstream `prepare_rx()`). Settle 3 ms after a retune. | *working* `lib/esp_sdr/src/esp_sdr_rx.c:78-98`; *upstream* `modules/lib/esp-sdr/main/targets/esp32s3/receiver.c:92-106`; *measured* hardware-quirks.md |
| `force_rx_gain(force, index, bt)` | librftest | `(manual, index, 0)`; index 40 while the AGC is in charge | See AGC_FORCE_REG. | `lib/esp_sdr/src/esp_sdr_rx.c:41-54` |
| `esp_wifi_set_ps()`, `esp_wifi_set_promiscuous()`, `esp_wifi_set_channel()` | Wi-Fi driver | `WIFI_PS_NONE`; `true`; `(1, WIFI_SECOND_CHAN_NONE)` | The engine and the PHY calls need the Wi-Fi driver up. NULL mode accepts a channel only in promiscuous mode. | *working* `lib/esp_sdr/src/esp_sdr.c:153-166` |

For reading the decompile: the C6 libphy calls ROM helpers through the function
table `g_phyFuns`, slots 0x50 `i2c_readReg(block, host, reg)`, 0x58
`i2c_writeReg(block, host, reg, data)`, 0x5c
`i2c_readReg_Mask(block, host, reg, msb, lsb)` and 0x60
`i2c_writeReg_Mask(block, host, reg, msb, lsb, data)`. Evidence: *decompile* C6
`libphy.a` `phy_rfpll.o`.

## Measured PLL behaviour

Bench 2026-10-09, frequencies are the PLL's (the LO is PLL / 1.2 behind the
divider). Code is the capacitor code (`0x62:5` plus 256 times `0x62:7[2]`),
reg 12 the raw `0x62:12`.

| PLL MHz | Code | reg 12 |
|---|---|---|
| 2110 to 2138 | 351 | 0x05 (below the window) |
| 2140 | 351 | 0x01 |
| 2412 | 189 (187 in another run) | 0x00 |
| 2413 | 187 | |
| 2437 | 177 | |
| 2600 | 102 | 0x00 |
| 2800 | 29 | 0x02 |
| 2884 | 3 | 0x02 |
| 2888 to 2900 | erratic: 324, 430, 337, 321 | 0x09 |
| 2904 | 0 | 0x02 |
| 2908 and up | 0 | 0x0a (above the window) |

| | |
|---|---|
| VCO window, `0x62:12[3:2]` = 0 (PLL) | 2140 to 2884 MHz |
| PLL locks on air | 2128 to at least 2856 MHz (limit of the S3 transmitter) |
| LO without -ERANGE | 1784 to 2884 MHz |
| LO on air | 1774 to at least 2856 MHz |

- Below the window the calibration pins the code at 351, above it at 0; 430,
  seen in the erratic band, shows that all 9 bits exist. Repeated calibrations
  at one frequency land a few codes apart.
- The window sits about 12 MHz inside the lock limits, so -ERANGE errs on the
  safe side (`include/esp_sdr/esp_sdr.h:31-38, 110-125`, `README.rst`, "The library").
- The limits are the same with and without the divider, so they are VCO limits,
  per chip and untested over temperature. Below 2210 MHz the library always
  uses the divider, which reaches the VCO's lower limit / 1.2
  (`lib/esp_sdr/src/esp_sdr.c:65-76`; `tests/regression/lo_tuning` guards it,
  `doc/testing.md`, "lo_tuning").
- `0x62:12[1]` is set from about 2700 MHz up, `0x62:12[0]` below about 2412 MHz;
  their meaning is unknown.

## Stale modem state

After the native 802.15.4 radio ran (`apps/wpan`) and the chip was only reset,
not power cycled:

| Item | Clean | Stale |
|---|---|---|
| PLL for 2412 MHz through the vendor channel path | 2412 MHz, `0x63` regs 3 to 6 `08 33 33 01` | about 2404 MHz, `08 11 11 00` |
| `0x62` regs 1 and 5 (code) at 2412 MHz | 0xbb (187) | 0xbf (191) |
| `0x61:4` (`I2C_ULP_OCODE`) | 0x53 | 0x4f |
| `0x61:9` | 0x59 | 0x5a |

A power cycle or a deep sleep clears it, and so does the modem reset at
`PRE_KERNEL_1` (see MODEM_SYSCON_MODEM_RST_CONF_REG). Evidence: *measured*
bench 2026-10-09; the behaviour is in hardware-quirks.md.
