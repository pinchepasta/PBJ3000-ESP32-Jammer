/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Library internals shared by esp_sdr.c, esp_sdr_rx.c, esp_sdr_tx.c and the DAC
 * streaming backend: registers, ROM entry points, the shared state and the
 * DAC session.
 */

#ifndef ESP_SDR_PRIV_H_
#define ESP_SDR_PRIV_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>

#include <soc/soc.h>

#if defined(CONFIG_SOC_SERIES_ESP32S3)
#include <soc/sensitive_reg.h>

/* Undocumented Wi-Fi MAC dump engine; layout from upstream. */
#define DUMP_CTRL_REG     0x60033d5cU
#define DUMP_CTRL_RUN     BIT(31)
#define DUMP_CTRL_TRIGGER BIT(19)
#define DUMP_CTRL_DONE    BIT(18)
#define DUMP_CTRL_16MSPS  BIT(16)
#define DUMP_CTRL_40MSPS  BIT(15)
#define DUMP_CTRL_COUNT   GENMASK(13, 0)
#define DUMP_CONFIG_REG   0x60033d90U
/* Four 6-bit lane selectors on sources 0..3: receive I/Q. */
#define DUMP_CONFIG_IQ    0x000c2040U
#define DUMP_TIMEOUT_US   20000U

/*
 * The dump engine's DAC-side mirror, 8 bytes above DUMP_CTRL_REG. Same RUN/
 * TRIGGER/DONE/rate bit positions as the ADC side; librftest.a's dactrig()
 * fills the bank with a ramp before arming this, which esp_sdr_tx_play() skips
 * so the caller's own samples survive.
 */
#define DAC_TRIG_REG 0x60033d64U
/* Measured: the DAC side runs at 40 MS/s, bit 15 selects 80 MS/s; bits 16 and 27:20 do
 * nothing to the rate (dactrig() writes 27:20).
 */
#define DAC_CTRL_80MSPS BIT(15)

/* The dump engine writes one of four 64 KiB banks, one-hot in MAC_DUMP_USAGE. */
#define DUMP_BANK0_ADDR 0x3fcb0000U
#define DUMP_BANK_SIZE  0x10000U
#define DUMP_BANKS      3U /* bank 3 also holds ROM data */
#define DUMP_USAGE_REG  SENSITIVE_INTERNAL_SRAM_USAGE_3_REG
#define DUMP_USAGE_M    SENSITIVE_INTERNAL_SRAM_MAC_DUMP_USAGE_M
#define DUMP_USAGE_S    SENSITIVE_INTERNAL_SRAM_MAC_DUMP_USAGE_S

/* BBTOP analog baseband: I/Q low-pass capacitor codes in registers 4 and 5. */
#define BBTOP_BLOCK    0x67U
#define BBTOP_HOST     0U
#define BBTOP_LPF_REG  4U
#define BBTOP_LPF_DCAP GENMASK(5, 0)

/* Wi-Fi AGC: forced index 31:24, force 23, largest calibrated index 14:8. */
#define AGC_FORCE_REG      0x6001c02cU
#define AGC_GAIN_MAX       GENMASK(14, 8)
/* Larger maxima on the S3 point at uncalibrated table slots. */
#define AGC_GAIN_MAX_VALID 82U

#elif defined(CONFIG_SOC_SERIES_ESP32C6)

/*
 * C6 dump engine (upstream families/c5_c6_c61 and ring_capture.c): same
 * control bits as the S3 at another address, raw source 15, 80 MS/s only.
 */
#define DUMP_CTRL_REG     0x600a9004U
#define DUMP_CTRL_RUN     BIT(31)
#define DUMP_CTRL_TRIGGER BIT(19)
#define DUMP_CTRL_DONE    BIT(18)
#define DUMP_CTRL_COUNT   GENMASK(13, 0)
#define DUMP_MODE_REG     0x600a9008U
#define DUMP_MODE_SOURCE  GENMASK(18, 15)
#define DUMP_SOURCE_IQ    15U
#define DUMP_CONFIG_REG   0x600a9014U
#define DUMP_CONFIG_LANES GENMASK(24, 0)
#define DUMP_CONFIG_IQ    (BIT(6) | (2U << 12) | (3U << 18))
/* Clock forces from the C6 rftest_init/phy_set_clk_conf. */
#define DUMP_CLK_FORCE0_REG 0x600a9804U
#define DUMP_CLK_FORCE1_REG 0x600a980cU
#define DUMP_CLK_FORCE2_REG 0x600a9814U
#define DUMP_CLK_FORCE2     GENMASK(18, 0)
#define DUMP_GATE_REG       0x600a20b4U
#define DUMP_GATE_BIT       BIT(0)
#define DUMP_TIMEOUT_US     20000U

/* 128 KiB SRAM blocks; the engine writes the one selected one-hot in bits 11:8. */
#define DUMP_BANK0_ADDR 0x40800000U
#define DUMP_BANK_SIZE  0x20000U
#define DUMP_BANKS      3U
#define DUMP_USAGE_REG  0x60095004U
#define DUMP_USAGE_M    GENMASK(11, 8)
#define DUMP_USAGE_S    8U

#define BBTOP_BLOCK    0x67U
#define BBTOP_HOST     1U
#define BBTOP_LPF_REG  4U
#define BBTOP_LPF_DCAP GENMASK(5, 0)

#define AGC_FORCE_REG      0x600a702cU
#define AGC_GAIN_MAX       GENMASK(14, 8)
#define AGC_GAIN_MAX_VALID 79U

/*
 * IQ estimator (ROM rom_iq_est_enable, dc_iq_est): sums 80 MHz baseband
 * samples; the sums (>> 6) can be read while it counts.
 */
#define IQE_CTRL0_REG    0x600a0470U
#define IQE_CTRL0_MODE   GENMASK(27, 26)
#define IQE_CTRL1_REG    0x600a0474U
#define IQE_CTRL1_SRC    GENMASK(20, 19)
#define IQE_CTRL1_COUNT  GENMASK(16, 2)
#define IQE_CTRL1_START  BIT(1)
#define IQE_CTRL1_EN     BIT(0)
#define IQE_SUM_I_REG    0x600a0488U
#define IQE_SUM_Q_REG    0x600a048cU
#define IQE_COUNT_MAX    32768U
/* TX DC value: I 9:0, Q 19:10, drives the DAC once ram_stop_tx_tone() selected the data path. */
#define TXDC_REG         0x600a0444U
#define TXDC_IQ          GENMASK(19, 0)
#define TXDC_INJECT      0U

#elif defined(CONFIG_SOC_SERIES_ESP32C5)
#include <soc/hp_system_reg.h>

/*
 * C5 dump engine (librftest adctrig(), upstream's C61 capture): the C6
 * control bits with a 17-bit count, raw source 15 in bits 20:17, the rate
 * index (80, 40, 20, 10, 8, 4 MS/s) in bits 23:21.
 */
#define DUMP_CTRL_REG       0x600a9004U
#define DUMP_CTRL_RUN       BIT(31)
#define DUMP_CTRL_TRIGGER   BIT(19)
#define DUMP_CTRL_DONE      BIT(18)
#define DUMP_CTRL_COUNT     GENMASK(16, 0)
#define DUMP_MODE_REG       0x600a9008U
#define DUMP_MODE_SOURCE    GENMASK(20, 17)
#define DUMP_MODE_RATE      GENMASK(23, 21)
#define DUMP_SOURCE_IQ      15U
#define DUMP_CONFIG_REG     0x600a9018U
#define DUMP_CONFIG_LANES   GENMASK(24, 0)
#define DUMP_CONFIG_IQ      (24U | (25U << 6) | (26U << 12) | (27U << 18) | BIT(24))
/* Clock forces as adctrig() sets them. */
#define DUMP_CLK_FORCE_REG  0x600a9c04U
#define DUMP_CLK_EN_REG     0x600a0800U
#define DUMP_CLK_EN_BIT     BIT(2)
/* No source gate write: 0x600a20b4 reads 0 and any access stalls 1.6 ms here. */
#define DUMP_TIMEOUT_US     20000U

/*
 * 128 KiB SRAM blocks, owned one-hot in SRAM_USAGE; the owner takes the whole
 * block from the CPU. MAC_DUMP_ALLOC puts the samples 64 KiB into the block.
 * Block 2 holds ROM data, so only block 1 serves.
 */
#define DUMP_BANK0_ADDR     (0x40800000U + DUMP_ALLOC_OFFSET)
#define DUMP_BANK_SIZE      0x20000U
#define DUMP_BANKS          2U
#define DUMP_ALLOC_OFFSET   0x10000U
#define DUMP_USAGE_REG      HP_SYSTEM_SRAM_USAGE_CONF_REG
#define DUMP_USAGE_M        (HP_SYSTEM_SRAM_USAGE_M | HP_SYSTEM_MAC_DUMP_ALLOC_M)
#define DUMP_USAGE_S        HP_SYSTEM_SRAM_USAGE_S
#define DUMP_USAGE_ALLOC    HP_SYSTEM_MAC_DUMP_ALLOC

/* The I/Q low-pass capacitor codes sit in registers 6 and 7 here. */
#define BBTOP_BLOCK    0x67U
#define BBTOP_HOST     1U
#define BBTOP_LPF_REG  6U
#define BBTOP_LPF_DCAP GENMASK(5, 0)

#define AGC_FORCE_REG      0x600a702cU
#define AGC_GAIN_MAX       GENMASK(14, 8)
/* Upstream's C5 limit. */
#define AGC_GAIN_MAX_VALID 89U

/* IQ estimator: the C6 block 0x24 lower (libphy phy_iq_est_enable_new, dc_iq_est_test). */
#define IQE_CTRL0_REG    0x600a044cU
#define IQE_CTRL0_MODE   GENMASK(27, 26)
#define IQE_CTRL1_REG    0x600a0450U
#define IQE_CTRL1_SRC    GENMASK(20, 19)
#define IQE_CTRL1_COUNT  GENMASK(16, 2)
#define IQE_CTRL1_START  BIT(1)
#define IQE_CTRL1_EN     BIT(0)
#define IQE_SUM_I_REG    0x600a0464U
#define IQE_SUM_Q_REG    0x600a0468U
#define IQE_COUNT_MAX    32768U
/*
 * TX DC value as upstream's first C5 transmit used it (git show
 * e605684:main/c5_main.c, emit_iq()): I 9:0, Q 19:10, bit 26 enables the
 * injection, bit 25 is the band's I/Q swap (kept).
 */
#define TXDC_REG         0x600a0c08U
#define TXDC_IQ          GENMASK(19, 0)
#define TXDC_INJECT      BIT(26)

#else
#error "esp_sdr: unsupported SoC"
#endif

#ifndef DUMP_USAGE_ALLOC
#define DUMP_USAGE_ALLOC 0U
#endif

#define SENTINEL 0xa5a0055aU

/* Housekeeping counters (esp_sdr_get_stats()). */
extern struct esp_sdr_stats esp_sdr_counters;

/* Index upstream passes while the hardware AGC is in charge. */
#define AGC_DEFAULT_INDEX  40U

/* Default calibration channel for direct PLL tuning. */
#define CAL_CHANNEL_MHZ 2412U

/* ROM and PHY library entry points. */
#if defined(CONFIG_SOC_SERIES_ESP32S3)
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_rx_on(unsigned int en);
extern void rom_pbus_xpd_tx_off(void);
extern void rom_pbus_xpd_tx_on(unsigned int en);
extern void rom_set_rxclk_en(unsigned int en);
extern void set_chanfreq(unsigned int mhz, unsigned int bw);
/* xtal selects the crystal divisor: 1 = 26 MHz, 2 = 32 MHz, other = 40 MHz (0 on the XIAO). */
extern void set_rf_freq_offset(unsigned int xtal, unsigned int mhz, int khz);
extern void stop_tx_tone(unsigned int en);
#define phy_pbus_workmode   rom_pbus_workmode
#define phy_pbus_xpd_rx_on  rom_pbus_xpd_rx_on
#define phy_pbus_xpd_tx_off rom_pbus_xpd_tx_off
#define phy_set_rxclk_en    rom_set_rxclk_en
#define phy_stop_tx_tone    stop_tx_tone
#elif defined(CONFIG_SOC_SERIES_ESP32C5)
/* The C5 PHY's entry points all carry the phy_ prefix. */
extern void phy_pbus_workmode(void);
extern void phy_pbus_xpd_rx_on(unsigned int en);
extern void phy_pbus_xpd_tx_off(void);
extern void phy_set_rxclk_en(unsigned int en);
extern void phy_stop_tx_tone(unsigned int en);
/* libphy: channel setup at any MHz above 14 (5 GHz path from 4500 MHz), mode 1 the wide RX path. */
extern void phy_set_chanfreq(unsigned int mhz, unsigned int mode);
extern void phy_set_rf_freq_offset(unsigned int xtal, unsigned int mhz, int khz);
/* The PHY's calibration parameters; byte 49 is the crystal selector. */
extern unsigned char phy_param[];
#define PHY_PARAM_XTAL 49
extern unsigned int phy_chip_i2c_readReg(unsigned int block, unsigned int host, unsigned int reg);
extern void phy_i2c_writeReg(unsigned int block, unsigned int host, unsigned int reg,
			     unsigned int val);
#define rom_chip_i2c_readReg  phy_chip_i2c_readReg
#define rom_chip_i2c_writeReg phy_i2c_writeReg
#else
extern void pbus_workmode(void);
extern void ram_pbus_xpd_rx_on(unsigned int en);
extern void rom_pbus_xpd_tx_off(void);
extern void set_rxclk_en(unsigned int en);
extern void ram_stop_tx_tone(unsigned int en);
/* libphy: channel setup on a Wi-Fi channel, then direct PLL programming. */
extern void chip_v7_set_chan(unsigned int mhz, unsigned int mode);
extern void phy_set_freq(unsigned int mhz, int khz);
#define phy_pbus_workmode   pbus_workmode
#define phy_pbus_xpd_rx_on  ram_pbus_xpd_rx_on
#define phy_pbus_xpd_tx_off rom_pbus_xpd_tx_off
#define phy_set_rxclk_en    set_rxclk_en
#define phy_stop_tx_tone    ram_stop_tx_tone
#endif
#if defined(CONFIG_ESP_SDR_RFTEST)
/* librftest.a: force, index, bt (0 selects the Wi-Fi AGC). */
extern void force_rx_gain(unsigned int force, unsigned int index, unsigned int bt);
#if defined(CONFIG_SOC_SERIES_ESP32S3)
/*
 * librftest.a: gain memory index, baseband gain code, digital gain. Writes
 * gain memory slot 0 (the one force_txon_mode(1, 0, 0) selects) and turns
 * the vendor's TX power tracking off.
 */
extern void force_tx_gain(unsigned int gain, unsigned int bb, int dig);
/* librftest.a: en sets the front end's force-TX-on bit (0x60006000 bit 1). */
extern void force_txon_mode(unsigned int en, unsigned int mode, unsigned int ofs);
#endif
#endif

/*
 * The vendor PHY serializes its analog I2C traffic with this lock (hal
 * regi2c_ctrl.c); every rom_chip_i2c_* sequence of ours takes it too.
 */
void regi2c_enter_critical(void);
void regi2c_exit_critical(void);

/* Reserved by the module linker snippet. */
extern uint32_t __esp_sdr_bank_start[];
/* Second bank, only linked with CONFIG_ESP_SDR_BANK1 (esp_sdr_bank_tx.ld). */
extern uint32_t __esp_sdr_bank1_start[];

/* Radio state, guarded by esp_sdr_lock (esp_sdr.c). */
extern struct k_mutex esp_sdr_lock;
extern bool esp_sdr_ready;
extern uint32_t esp_sdr_freq_mhz;
extern int32_t esp_sdr_fofs_khz;

/* Tune the LO to esp_sdr_freq_mhz (plus offset). */
void esp_sdr_tune(void);
/* The LO plan puts esp_sdr_freq_mhz behind the 5/6 divider (below 2210 MHz). */
bool esp_sdr_lo_divided(void);
/* Back to the resting RX-ready state after a tuning change; lock held. */
int esp_sdr_retune(void);
/* MAC_DUMP_USAGE one-hot bits of the capture bank. */
int esp_sdr_bank_usage(uint32_t *usage);
/* Settling after a retune (PLL lock), and the TX/RX turnaround (esp_sdr_set_turnaround()). */
#define SDR_RETUNE_SETTLE_US 3000U
extern uint32_t esp_sdr_turn_settle_us;
extern bool esp_sdr_turn_retune;

/*
 * Front end to RX (the resting state) after a retune, back to RX after a
 * transmission, or to TX, at the current gains; lock held.
 */
void esp_sdr_rx_prepare(void);
/* Re-force a fixed receive gain last applied more than 20 ms ago; lock held. Returns whether it did. */
bool esp_sdr_rx_gain_refresh(void);
void esp_sdr_rx_resume(void);
void esp_sdr_tx_prepare(void);
/* esp_sdr_tx_prepare() without the settle time. */
void esp_sdr_tx_front_end(void);

/* Take the receiver lock and switch the front end to TX at the current step. */
int esp_sdr_dac_begin(enum esp_sdr_rate rate);

/*
 * Two DAC banks: 0 is the capture bank (SRAM bank 2), 1 the transmit bank
 * (SRAM bank 1, reserved by esp_sdr_bank_tx.ld). The engine owns the bank it
 * plays, the CPUs the other one: fill that one with plain stores, then
 * esp_sdr_dac_start() it once the previous burst is done.
 */
#define ESP_SDR_DAC_BANKS 2

/* Bank idx and its size in words. */
uint32_t *esp_sdr_dac_buf(int idx, size_t *words);

/* Hand bank idx to the engine and play its first count words; returns at once. */
void esp_sdr_dac_start(int idx, size_t count);

/* Wait for the burst started last; false on timeout. */
bool esp_sdr_dac_wait(void);

/*
 * Loop mode: with the trigger bit held the engine replays the first count
 * words of the selected bank back to back, gapless, at exactly the DAC rate
 * (stores to other banks do not slow it), until esp_sdr_dac_halt(). It reads
 * whichever bank is selected at the moment, so esp_sdr_dac_select() switches
 * mid-pass at the current offset. Call with interrupts locked to timestamp
 * the start: sample s plays s DAC periods after the call.
 */
void esp_sdr_dac_loop(int idx, size_t count);

/* Hand bank idx to the engine (the CPUs keep every other one). */
void esp_sdr_dac_select(int idx);

/* Stop the engine at once. */
void esp_sdr_dac_halt(void);

/* Back to RX and release the lock. */
void esp_sdr_dac_end(void);

#endif /* ESP_SDR_PRIV_H_ */
