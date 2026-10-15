#!/usr/bin/env bash
#
# esp-sdr-jammer-autopatch.sh  (rev 5)
#
# Turns https://github.com/cyber-murmel/esp-sdr-zephyr into a spectrum jammer
# with an exact-frequency picker, and writes the WebSerial control page.
# Every file it needs is embedded below: this script IS the patch.
#
# rev 5: the page paced its commands and applies the plan as one "jam plan"
# line, and the shell RX ring is 2048 bytes: a burst used to overflow the
# 64 byte ring, so the last command (jam start) was dropped and the buttons
# looked dead while typing worked. The page also verifies every command and
# retries once. rev 4 added truncated-save guards, rev 3 fixed the
# cmd_harm()/cmd_amp() signatures, rev 2 fixed "Start" being silent (a retune
# inside the transmit session left the analog front end in RX).
#
#   ./esp-sdr-jammer-autopatch.sh [options]
#
#   --ws DIR        west workspace (default ./esp-sdr-ws). Relocated to
#                   $HOME/<name> if the path contains a space, because Zephyr
#                   splits devicetree overlay paths on them.
#   --board BOARD   default esp32s3_devkitc/esp32s3/procpu
#   --repo URL      manifest repo (default cyber-murmel/esp-sdr-zephyr)
#   --ref REV       manifest revision (default main)
#   --keep-path     do not relocate a workspace whose path has spaces
#   --multiapp      also register the app in apps/multiapp
#   --sysbuild / --no-sysbuild
#   --no-deps / --deps-only
#   --no-update / --no-build / --flash / --dfu / --force
#   -h | --help
#
# The last line of this file is a sentinel; if it is missing, the save was cut.
#
set -euo pipefail

JAM_REV="5"
REPO_URL="https://github.com/cyber-murmel/esp-sdr-zephyr"
REPO_REF="main"
DEFAULT_WS="$(pwd)/esp-sdr-ws"
BOARD="esp32s3_devkitc/esp32s3/procpu"
DO_MULTIAPP=0; DO_UPDATE=1; DO_BUILD=1; DO_FLASH=0; DO_DFU=0
DO_DEPS=1; DEPS_ONLY=0; FORCE=0; KEEP_PATH=0; SYSBUILD=""

die() { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }
log() { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn(){ printf '\033[33mwarn:\033[0m %s\n' "$*" >&2; }

while [ $# -gt 0 ]; do
  case "$1" in
    --ws)          DEFAULT_WS="$2"; shift 2 ;;
    --board)       BOARD="$2"; shift 2 ;;
    --repo)        REPO_URL="$2"; shift 2 ;;
    --ref)         REPO_REF="$2"; shift 2 ;;
    --multiapp)    DO_MULTIAPP=1; shift ;;
    --keep-path)   KEEP_PATH=1; shift ;;
    --sysbuild)    SYSBUILD=1; shift ;;
    --no-sysbuild) SYSBUILD=0; shift ;;
    --no-deps)     DO_DEPS=0; shift ;;
    --deps-only)   DEPS_ONLY=1; shift ;;
    --no-update)   DO_UPDATE=0; shift ;;
    --no-build)    DO_BUILD=0; shift ;;
    --flash)       DO_FLASH=1; shift ;;
    --dfu)         DO_DFU=1; shift ;;
    --force)       FORCE=1; shift ;;
    -h|--help)     sed -n '2,30p' "$0"; exit 0 ;;
    *)             die "unknown option $1" ;;
  esac
done

# A save that was cut short fails halfway through otherwise. Say so first.
if ! tail -n 1 "$0" | grep -qF 'end of esp-sdr-jammer-autopatch.sh'; then
  warn "this file looks truncated: its last line is not the end marker"
  printf '      got: %s\n' "$(tail -n 1 "$0" | cut -c1-70)"
  printf '      re-save the whole file, then check with: bash -n %s\n' "$0"
fi

[ -n "$SYSBUILD" ] || { case "$BOARD" in *esp32s3*) SYSBUILD=1 ;; *) SYSBUILD=0 ;; esac; }

command -v west    >/dev/null || die "west not found. Either 'pip install west' in a venv, or see the notes at the end."
command -v python3 >/dev/null || die "python3 not found"

# --- 0. workspace location (see the header note) ---------------------------
if [ ! -d "$DEFAULT_WS/.west" ] && [ -d "$HOME/esp-sdr-ws/.west" ]; then
  DEFAULT_WS="$HOME/esp-sdr-ws"
fi
WS="$DEFAULT_WS"

if [[ "$WS" == *" "* ]]; then
  ALT="$HOME/$(basename "${WS%/}")"
  if [ "$KEEP_PATH" = 1 ]; then
    warn "workspace path has spaces; Zephyr will split the devicetree overlay list"
  elif [ "$ALT" = "$WS" ]; then
    warn "workspace path and \$HOME both have spaces"
  elif [ -d "$ALT/.west" ]; then
    log "path has spaces; using the existing workspace at $ALT"
    WS="$ALT"
  elif [ -d "$WS/.west" ]; then
    log "path has spaces; moving workspace"
    log "  $WS"
    log "  -> $ALT"
    mv "$WS" "$ALT"
    WS="$ALT"
  else
    log "path has spaces; creating the workspace at $ALT instead"
    WS="$ALT"
  fi
fi

REPO="$WS/esp-sdr-zephyr"; APP="$REPO/apps/jammer"
BUILD="$WS/build-jammer"; BUILD_LOG="$WS/build-jammer.log"; HTML="$WS/jammer.html"

# --- 1. workspace ----------------------------------------------------------
[ -d "$WS/.west" ] || { log "creating west workspace in $WS"; west init -m "$REPO_URL" --mr "$REPO_REF" "$WS"; }
[ "$DO_UPDATE" = 1 ] && { log "west update (this takes a while)"; ( cd "$WS" && west update ); }
[ -d "$REPO" ] || die "$REPO missing after west init/update"

# --- 2. python dependency preflight ---------------------------------------
west_python() {
  local p
  p="$(sed -n '1s/^#!//p' "$(command -v west)" 2>/dev/null || true)"
  p="${p##* }"
  case "$p" in */python3|*/python3.*|python3) [ -x "$p" ] && { printf '%s' "$p"; return; } ;; esac
  command -v python3
}
deps_ok() { "$1" -c 'import jsonschema, pykwalify, anytree, elftools, packaging, yaml' 2>/dev/null; }

if [ "$DO_DEPS" = 1 ]; then
  PYW="$(west_python)"
  if deps_ok "$PYW"; then
    log "python deps present ($PYW)"
  else
    log "west's interpreter ($PYW) is missing Zephyr's Python deps"
    REQ="$WS/modules/zephyr/scripts/requirements-base.txt"
    [ -f "$REQ" ] || REQ="$WS/modules/zephyr/scripts/requirements.txt"
    [ -f "$REQ" ] || die "no requirements file under $WS/modules/zephyr/scripts"
    "$PYW" -m pip install -q --break-system-packages -r "$REQ" esptool 2>/dev/null \
      || "$PYW" -m pip install -q -r "$REQ" esptool \
      || die "pip failed for $PYW -- use a venv instead"
    deps_ok "$PYW" || die "deps still missing after install into $PYW"
    log "deps installed"
  fi
fi
[ "$DEPS_ONLY" = 1 ] && { log "--deps-only, stopping"; exit 0; }

( cd "$WS" && west build --help >/dev/null 2>&1 ) \
  || die "west cannot load Zephyr's build extension -- re-run without --no-deps, or use a venv"

log "fetching blobs (librftest: TX power steps)"
( cd "$WS" && west blobs fetch hal_espressif  >/dev/null 2>&1 || true )
( cd "$WS" && west blobs fetch esp-sdr-zephyr >/dev/null 2>&1 || true )

# --- 3. the app ------------------------------------------------------------
if [ -e "$APP" ] && [ "$FORCE" != 1 ] && [ "$(cat "$APP/.jam_rev" 2>/dev/null)" = "$JAM_REV" ]; then
  log "apps/jammer is already at rev $JAM_REV, leaving it (--force to rewrite)"
else
  log "writing $APP (rev $JAM_REV)"
  rm -rf "$APP"
  mkdir -p "$APP/src" "$APP/boards"

  cat > "$APP/CMakeLists.txt" <<'JAM_EOF'
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(esp_sdr_jammer)

if(CONFIG_USB_DEVICE_STACK_NEXT)
  include(${CMAKE_CURRENT_LIST_DIR}/../common/common.cmake)
endif()

target_sources(app PRIVATE src/main.c)
JAM_EOF

  cat > "$APP/Kconfig" <<'JAM_EOF'
# SPDX-License-Identifier: GPL-3.0-or-later

mainmenu "jammer: ESP-SDR spectrum jammer"

rsource "../common/Kconfig"
source "samples/subsys/usb/common/Kconfig.sample_usbd"

source "Kconfig.zephyr"
JAM_EOF

  cat > "$APP/prj.conf" <<'JAM_EOF'
# The Wi-Fi driver brings the radio up at boot; nothing may scan or connect.
CONFIG_NETWORKING=y
CONFIG_WIFI=y
CONFIG_ESP32_WIFI_STA_POWER_SAVE_NONE=y
CONFIG_ESP_SDR=y

CONFIG_SHELL=y
# The jammer thread runs below the shell, so a command is served between dwells.
CONFIG_SHELL_THREAD_PRIORITY=10
CONFIG_SHELL_STACK_SIZE=4096
CONFIG_MAIN_STACK_SIZE=3072
CONFIG_THREAD_NAME=y

# A browser sends several commands at once: the default 64 byte RX ring drops
# characters when they arrive in a burst, and the shell then eats half a line.
CONFIG_SHELL_BACKEND_SERIAL_RX_RING_BUFFER_SIZE=2048
CONFIG_SHELL_BACKEND_SERIAL_TX_RING_BUFFER_SIZE=4096

CONFIG_LOG=y
CONFIG_LOG_MODE_DEFERRED=y
CONFIG_PICOLIBC_IO_FLOAT=y
JAM_EOF

  cat > "$APP/sysbuild.conf" <<'JAM_EOF'
SB_CONFIG_BOOTLOADER_MCUBOOT=y
JAM_EOF

  for b in esp32s3_devkitc_procpu xiao_esp32s3_procpu; do
    cat > "$APP/boards/$b.conf" <<'JAM_EOF'
CONFIG_FPU=y
CONFIG_FPU_SHARING=y
CONFIG_RESET_ON_FATAL_ERROR=y

CONFIG_USB_DEVICE_STACK_NEXT=y
CONFIG_SAMPLE_USBD_PRODUCT="ESP-SDR jammer"
CONFIG_SAMPLE_USBD_PID=0x0007
CONFIG_USBD_LOG_LEVEL_WRN=y
CONFIG_UDC_DRIVER_LOG_LEVEL_WRN=y
CONFIG_USBD_CDC_ACM_CLASS=y
CONFIG_USBD_CDC_ACM_LOG_LEVEL_OFF=y
CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT=n
CONFIG_LOG_BACKEND_UART=n
CONFIG_USBD_CDC_ACM_WORKQUEUE=y
CONFIG_KERNEL_SHELL=y
CONFIG_SHELL_BACKEND_SERIAL_LOG_MESSAGE_QUEUE_SIZE=256
JAM_EOF
    cat > "$APP/boards/$b.overlay" <<'JAM_EOF'
/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Console and shell on the native USB port (the one silkscreened "USB"),
 * so a phone can drive it with WebSerial.
 */

/ {
	chosen {
		zephyr,console = &cdc_acm_uart0;
		zephyr,shell-uart = &cdc_acm_uart0;
	};
};

zephyr_udc0: &usb_otg {
	status = "okay";

	cdc_acm_uart0: cdc_acm_uart0 {
		compatible = "zephyr,cdc-acm-uart";
		label = "ESP-SDR jammer shell";
	};
};
JAM_EOF
  done

  cat > "$APP/boards/waveshare_esp32c5_zero_hpcore.conf" <<'JAM_EOF'
CONFIG_LOG=n
CONFIG_MAIN_STACK_SIZE=2048
CONFIG_SHELL_STACK_SIZE=2048
JAM_EOF

  cat > "$APP/boards/xiao_esp32c6_hpcore.conf" <<'JAM_EOF'
CONFIG_LOG=n
CONFIG_MAIN_STACK_SIZE=2048
CONFIG_SHELL_STACK_SIZE=2048
JAM_EOF

  cat > "$APP/sample.yaml" <<'JAM_EOF'
sample:
  name: ESP-SDR spectrum jammer
common:
  tags: esp-sdr
  build_only: true
tests:
  sample.esp_sdr.jammer.s3:
    sysbuild: true
    platform_allow: [esp32s3_devkitc/esp32s3/procpu, xiao_esp32s3/esp32s3/procpu]
  sample.esp_sdr.jammer.c5:
    platform_allow: [waveshare_esp32c5_zero/esp32c5/hpcore]
  sample.esp_sdr.jammer.c6:
    platform_allow: [xiao_esp32c6/esp32c6/hpcore]
JAM_EOF

  cat > "$APP/README.rst" <<'JAM_EOF'
ESP-SDR spectrum jammer
=======================

Transmits a jamming waveform: white noise, a sawtooth chirp, a CW carrier or
a 1 kHz gated pulse, hopped over a list of *target* frequencies in kHz.

The LO is target / harm with harm chosen automatically (1st inside the LO
range, else 2nd, else 3rd). ESP32-S3 LO locks around 1817-2790 MHz, so
2.4 GHz is direct and 5-8.3 GHz rides a PA harmonic (15-25 dB down, short
range). ``jam span`` is the half-width at the target, and it shapes the
noise and the pulse too (two one-pole sections at the span edge).

The transmit engine is a loop session (front end switched to TX once,
gapless playback, retune between hops). **The retune is the tricky part:**
``esp_sdr_set_freq()`` runs the RX front end sequence, which powers TX down
and leaves the analog front end in RX, while ``esp_sdr_tx_loop_start()``
only touches the DAC. Every retune is therefore followed by
``esp_sdr_tx_prepare()`` (``lib/esp_sdr/src/esp_sdr_priv.h``), which puts the
front end back into TX inside the session. Without that call the DAC plays
into a front end that is in RX and nothing reaches the antenna -- which is
why ``jam test`` (it tunes before opening the session) worked and
``jam start`` did not. ``jam engine play`` selects the one-shot
``esp_sdr_tx_play_for()`` path instead, which handles the front end itself.

``jam plan`` sets the whole plan in one line, so a browser can apply it with
a single command (a burst of commands loses characters in the shell's RX
ring, which is why the control page paces its writes and verifies them).

Settings are re-read between hops, so the mode buttons and the sliders act
while jamming.

If nothing transmits, run ``jam test``: it puts a 2441.1 MHz carrier on air
for 3 s and prints every return code. Then ``jam status`` for counters.

Shell::

  jam test / jam status   # is it actually transmitting
  jam ref                 # what lives on which frequency
  jam plan 39000 noise 450 40 -1 auto 2412 2437   # span mode amp dwell gain harm MHz...
  jam lo 2412 2437 2462   # exact targets, MHz, decimals allowed
  jam harm auto           # or 1 / 2 / 3
  jam span 12000          # half-width at the target, kHz
  jam mode noise|chirp|cw|pulse
  jam start / jam stop / jam mon

SPDX-License-Identifier: GPL-3.0-or-later
JAM_EOF

  cat > "$APP/src/main.c" <<'JAM_MAIN_C'
/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * apps/jammer -- an ESP-SDR spectrum jammer with exact-frequency targets.
 *
 * Transmit engine: a loop session (esp_sdr_tx_loop_*) opened once. It holds
 * the radio, playback is gapless and the LO is retuned between hops.
 *
 * The one thing that made "jam start" silent while "jam test" worked:
 * esp_sdr_set_freq() runs esp_sdr_rx_prepare(), which powers the TX path down
 * and puts the analog front end back into RX. esp_sdr_tx_loop_start() only
 * touches the DAC, so after a retune the DAC was playing into a front end in
 * RX and nothing reached the antenna. Every retune is therefore followed by
 * esp_sdr_tx_prepare() (lib/esp_sdr/src/esp_sdr_priv.h), which switches the
 * front end back to TX inside the session. "jam test" never hit this because
 * it tunes before opening the session.
 *
 * "jam plan" sets everything in one line: a burst of shell commands can lose
 * characters in the shell's RX ring, and then half a line is executed.
 *
 * Settings are re-read from the shell between hops, so the mode buttons and
 * the sliders act while jamming.
 *
 * Every radio call's return code is reported: the first few print, and
 * "jam status" keeps counters. "jam test" puts a real carrier on air.
 */

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/app_version.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>

#if defined(CONFIG_ESP_SDR_PACED)
#include <esp_sdr/esp_sdr_paced.h>
#endif

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include "app_usb.h"
#endif

/*
 * Library internals (lib/esp_sdr/src/esp_sdr_priv.h): the front end to TX at
 * the current gains, to be called with the radio lock held -- which it is
 * inside a loop session. It undoes the RX sequence esp_sdr_set_freq() runs.
 */
extern void esp_sdr_tx_prepare(void);

#define TWO_PI         6.28318530717958647692
#define JAM_MAX_HOPS   32
#define JAM_BANK_WORDS ESP_SDR_SAMPLES_MAX
#define JAM_PRIO       12
#define JAM_STACK      6144	/* the waveform builders use the FPU */
#define MON_BINS       48
#define MON_MAXN       2048

#if defined(CONFIG_SOC_SERIES_ESP32S3)
/* The LO plan stays below the PLL's own floor: attempted, may not lock. */
#define JAM_LO_MIN_KHZ  1514000U
/* Measured PLL floor: below this the VCO ends outside its window. */
#define JAM_PLL_MIN_KHZ 1817000U
#define JAM_LO_MAX_KHZ  2790000U
#elif defined(CONFIG_SOC_SERIES_ESP32C5)
#define JAM_LO_MIN_KHZ   100000U
#define JAM_PLL_MIN_KHZ  100000U
#define JAM_LO_MAX_KHZ  6000000U
#elif defined(CONFIG_SOC_SERIES_ESP32C6)
#define JAM_LO_MIN_KHZ  2400000U
#define JAM_PLL_MIN_KHZ 2400000U
#define JAM_LO_MAX_KHZ  2500000U
#else
#define JAM_LO_MIN_KHZ   100000U
#define JAM_PLL_MIN_KHZ  100000U
#define JAM_LO_MAX_KHZ  6000000U
#endif

/* CONFIG_FPU_SHARING: a thread that touches the FPU must say so. */
#if defined(CONFIG_FPU)
#if !defined(K_FP_REGS)
#define K_FP_REGS BIT(1)
#endif
#define JAM_THREAD_OPT K_FP_REGS
#else
#define JAM_THREAD_OPT 0
#endif

enum jam_mode { JAM_NOISE, JAM_CHIRP, JAM_CW, JAM_PULSE };
static const char *const mode_name[] = { "noise", "chirp", "cw", "pulse" };

typedef struct {
	enum jam_mode mode;
	uint32_t hop[JAM_MAX_HOPS];
	int n_hop;
	int harm;
	uint32_t span_khz;
	uint32_t dwell_ms;
	int amp;
	int gain;
	int engine;			/* 0 = loop, 1 = one-shot play */
	enum esp_sdr_rate rate;
	uint32_t prate_khz;
} jam_cfg_t;

static jam_cfg_t cfg;

static K_MUTEX_DEFINE(cfg_lock);
static K_SEM_DEFINE(jam_done, 0, 1);
static volatile bool jam_stop;
static volatile bool jam_active;

static struct k_thread jam_tcb;
static K_THREAD_STACK_DEFINE(jam_stack, JAM_STACK);

/* Only the one-shot "play" engine needs a separate source buffer. */
static uint32_t jam_bank[JAM_BANK_WORDS];
static uint32_t rng_state = 0x9e3779b9u;

/* Last LO offset handed to the library: 0 has to be written back too. */
static int32_t jam_last_off = INT32_MIN;

/* Diagnostics: a silent failure used to look exactly like success. */
static volatile uint32_t jam_dwells, jam_errors, jam_range_warn;
static volatile int jam_last_ret;
static volatile const char *jam_last_where = "none";

struct jam_band {
	const char *name;
	uint32_t span_khz;
	uint32_t target_khz[JAM_MAX_HOPS];
	int n;
};

static const struct jam_band bands[] = {
	{ "24",       39000, { 2441000 }, 1 },
	{ "wifi24",   39000, { 2441000 }, 1 },
	{ "wifi24h",  12000, { 2412000, 2437000, 2462000, 2472000 }, 4 },
	{ "ble",       4000, { 2402000, 2426000, 2480000 }, 3 },
	{ "zigbee",    4000, { 2405000, 2410000, 2415000, 2420000, 2425000, 2430000,
			       2435000, 2440000, 2445000, 2450000, 2455000, 2460000,
			       2465000, 2470000, 2475000, 2480000 }, 16 },
	{ "nrf24",     2000, { 2400000, 2425000, 2450000, 2475000, 2500000, 2525000 }, 6 },
	{ "microwave", 4000, { 2450000 }, 1 },
	{ "lte2600",  35000, { 2620000 }, 1 },
	{ "umts2100", 35000, { 2140000 }, 1 },
	{ "gsm1800",  35000, { 1840000 }, 1 },
	{ "gsm1900",  35000, { 1880000 }, 1 },
	{ "dect",     12000, { 1890000 }, 1 },
	{ "gps",       3000, { 1575420 }, 1 },
	{ "unii1",    50000, { 5200000 }, 1 },
	{ "unii2",    50000, { 5300000 }, 1 },
	{ "unii2c",   45000, { 5540000, 5620000, 5700000 }, 3 },
	{ "unii3",    40000, { 5745000, 5785000, 5825000 }, 3 },
	{ "unii4",    40000, { 5860000, 5900000 }, 2 },
	{ "fpv",      40000, { 5700000, 5800000, 5900000 }, 3 },
	{ "5g",       40000, { 5200000, 5300000, 5540000, 5620000, 5700000, 5785000 }, 6 },
	{ "6g",       75000, { 6000000, 6150000, 6300000, 6450000, 6600000, 6750000,
			       6900000, 7050000 }, 8 },
};

static void cfg_defaults(void)
{
	cfg.mode = JAM_NOISE;
	cfg.hop[0] = 2441000;
	cfg.n_hop = 1;
	cfg.harm = 0;
	cfg.span_khz = 39000;
	cfg.dwell_ms = 40;
	cfg.amp = 450;
	cfg.gain = -1;
	cfg.engine = 0;
	cfg.rate = ESP_SDR_RATE_80MSPS;
	cfg.prate_khz = 1000;
}

static long arg(char **argv, size_t argc, size_t i, long def)
{
	return argc > i ? strtol(argv[i], NULL, 0) : def;
}

static void report_err(const char *where, int ret)
{
	jam_last_where = where;
	jam_last_ret = ret;
	jam_errors++;
	if (jam_errors <= 6) {
		printk("jammer: %s failed (%d)\n", where, ret);
	}
}

/* ---- frequency plan ---- */

static int auto_harm(uint32_t target_khz)
{
	if (target_khz <= JAM_LO_MAX_KHZ) {
		return 1;
	}
	if ((target_khz / 2U) <= JAM_LO_MAX_KHZ) {
		return 2;
	}
	return 3;
}

static int harm_of(const jam_cfg_t *r, uint32_t target_khz)
{
	return r->harm ? r->harm : auto_harm(target_khz);
}

static uint32_t lo_of(uint32_t target_khz, int harm)
{
	return (target_khz + (uint32_t)harm / 2U) / (uint32_t)harm;
}

static bool reachable(uint32_t target_khz, int harm)
{
	uint32_t lo = lo_of(target_khz, harm);

	return lo >= JAM_LO_MIN_KHZ && lo <= JAM_LO_MAX_KHZ;
}

/* Below the measured PLL floor: tuned, but the VCO may not lock. */
static bool marginal(uint32_t target_khz, int harm)
{
	uint32_t lo = lo_of(target_khz, harm);

	return lo >= JAM_LO_MIN_KHZ && lo < JAM_PLL_MIN_KHZ;
}

/* Does the waveform have to be rebuilt for these settings? */
static bool wave_key_changed(const jam_cfg_t *a, const jam_cfg_t *b)
{
	return a->mode != b->mode || a->amp != b->amp || a->span_khz != b->span_khz ||
	       a->harm != b->harm;
}

/* ---- waveforms ---- */

static inline uint32_t rng32(void)
{
	uint32_t x = rng_state;

	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	rng_state = x;
	return x;
}

static inline int rnd(int amp)
{
	return (int)((rng32() >> 20) % (uint32_t)(2 * amp + 1)) - amp;
}

static inline int16_t clamp511(long v)
{
	return (int16_t)CLAMP(v, -511, 511);
}

/*
 * Noise, shaped to +-half by two one-pole sections so the span slider does
 * something in noise and pulse mode too, then scaled back up so the level
 * does not change with the span. Full band when the span reaches fs / 2.
 * gate_hz != 0 gates the noise on for a quarter of the period.
 */
static void fill_noise(uint32_t *w, size_t n, int amp, uint32_t gate_hz, double fs, double half)
{
	double a = 1.0, g = 1.0, y1i = 0.0, y1q = 0.0, y2i = 0.0, y2q = 0.0;
	size_t period = 0, on = 0;

	if (gate_hz != 0U) {
		period = (size_t)(fs / (double)gate_hz);
		if (period < 4U) {
			period = 4U;
		}
		on = period / 4U;
	}
	if (half > 0.0 && half < fs * 0.45) {
		a = 1.0 - exp(-TWO_PI * half / fs);
		a = CLAMP(a, 0.01, 1.0);
		/* Two poles lose (a / (2 - a))^2 in amplitude; back it out. */
		g = ((2.0 - a) / a) * ((2.0 - a) / a);
	}
	for (size_t k = 0; k < n; k++) {
		bool open = period == 0U || (k % period) < on;
		double xi = open ? (double)rnd(amp) : 0.0;
		double xq = open ? (double)rnd(amp) : 0.0;

		if (a < 1.0) {
			y1i += a * (xi - y1i);
			y2i += a * (y1i - y2i);
			y1q += a * (xq - y1q);
			y2q += a * (y1q - y2q);
			xi = y2i * g;
			xq = y2q * g;
		}
		w[k] = esp_sdr_tx_word(clamp511(lround(xi)), clamp511(lround(xq)));
	}
}

static void fill_tone(uint32_t *w, size_t n, int amp, double f, double fs)
{
	long cycles = lround(f * (double)n / fs);

	if (cycles == 0) {
		cycles = 1;
	}
	for (size_t k = 0; k < n; k++) {
		double ph = TWO_PI * (double)cycles * (double)k / (double)n;

		w[k] = esp_sdr_tx_word((int16_t)lround((double)amp * cos(ph)),
				       (int16_t)lround((double)amp * sin(ph)));
	}
}

static void fill_chirp(uint32_t *w, size_t n, int amp, double f0, double f1, double fs)
{
	double ph = 0.0;

	for (size_t k = 0; k < n; k++) {
		double f = f0 + (f1 - f0) * (double)k / (double)n;

		ph += TWO_PI * f / fs;
		while (ph >= TWO_PI) {
			ph -= TWO_PI;
		}
		while (ph < 0.0) {
			ph += TWO_PI;
		}
		w[k] = esp_sdr_tx_word((int16_t)lround((double)amp * cos(ph)),
				       (int16_t)lround((double)amp * sin(ph)));
	}
}

static void build_wave(uint32_t *dst, size_t n, const jam_cfg_t *r, int harm, double fs)
{
	double half = (double)r->span_khz * 1000.0 / (double)harm;

	if (half > fs * 0.47) {
		half = fs * 0.47;
	}
	switch (r->mode) {
	case JAM_NOISE:
		fill_noise(dst, n, r->amp, 0U, fs, half);
		break;
	case JAM_PULSE:
		fill_noise(dst, n, r->amp, 1000U, fs, half);
		break;
	case JAM_CHIRP:
		fill_chirp(dst, n, r->amp, -half, half, fs);
		break;
	case JAM_CW:
		fill_tone(dst, n, r->amp, half > 1.0 ? half : 1.0e6, fs);
		break;
	}
}

/*
 * Retune the LO. -ERANGE means "tuned, but the VCO ended outside its tuning
 * window": counted and tolerated, never a reason to skip the hop. The offset
 * is only written when it changes, and 0 is written back as well.
 */
static int jam_tune(uint32_t lo_khz)
{
	int32_t off = (int32_t)(lo_khz % 1000U);
	int ret = esp_sdr_set_freq(lo_khz / 1000U);

	if (ret == -ERANGE) {
		jam_range_warn++;
	}
	if (ret != 0 && ret != -ERANGE) {
		return ret;
	}
	if (off != jam_last_off) {
		jam_last_off = off;
		ret = esp_sdr_set_freq_offset(off);
		if (ret == -ERANGE) {
			jam_range_warn++;
		}
		if (ret != 0 && ret != -ERANGE) {
			return ret;
		}
	}
	return 0;
}

/* ---- the jammer thread ---- */

#if defined(CONFIG_SOC_SERIES_ESP32S3)
/*
 * Loop engine: one session, front end in TX for its whole duration, gapless
 * playback, retune between hops. Every retune is followed by
 * esp_sdr_tx_prepare(): esp_sdr_set_freq() goes through the RX front end
 * sequence, and loop_start() does not switch the front end back, so without
 * that call nothing reaches the antenna.
 */
static void jam_run_loop(const jam_cfg_t *r0)
{
	jam_cfg_t r = *r0;
	enum esp_sdr_rate rate = r.rate;
	uint32_t fs = esp_sdr_tx_rate_hz(rate);
	uint32_t *bank;
	size_t words = 0;
	int built_harm = -1;
	int applied_gain = INT_MIN;
	int ret;

	if (fs == 0U) {
		rate = ESP_SDR_RATE_80MSPS;
		fs = esp_sdr_tx_rate_hz(rate);
	}
	if (fs == 0U) {
		report_err("dac_rate", -EINVAL);
		return;
	}

	(void)esp_sdr_rx_set_lpf(0);
	ret = esp_sdr_tx_set_lpf(0, 0);
	if (ret != 0) {
		report_err("tx_lpf", ret);
	}

	ret = esp_sdr_tx_loop_begin(rate);
	if (ret != 0) {
		report_err("loop_begin", ret);
		return;
	}

	bank = esp_sdr_tx_loop_buf(0, &words);
	if (bank == NULL || words < ESP_SDR_SAMPLES_MIN) {
		report_err("loop_buf", -ENOMEM);
		esp_sdr_tx_loop_end();
		return;
	}
	words = MIN(words, (size_t)ESP_SDR_SAMPLES_MAX);

	while (!jam_stop) {
		jam_cfg_t s;
		int gain;

		/* Read between hops: the page's sliders stay live. */
		k_mutex_lock(&cfg_lock, K_FOREVER);
		s = cfg;
		k_mutex_unlock(&cfg_lock);

		if (s.n_hop <= 0) {
			k_msleep(20);
			continue;
		}
		if (wave_key_changed(&s, &r)) {
			/* Only writable while the engine is halted. */
			esp_sdr_tx_loop_halt();
			built_harm = -1;
			r = s;
		}
		gain = s.gain < 0 ? esp_sdr_tx_gain_max() : s.gain;
		if (gain != applied_gain) {
			ret = esp_sdr_tx_set_gain(gain);
			if (ret != 0) {
				report_err("tx_gain", ret);
			}
			applied_gain = gain;
		}

		for (int h = 0; h < s.n_hop && !jam_stop; h++) {
			int hh = harm_of(&s, s.hop[h]);
			uint32_t lo = lo_of(s.hop[h], hh);

			if (!reachable(s.hop[h], hh)) {
				report_err("range", -ERANGE);
				continue;
			}
			if (hh != built_harm) {
				esp_sdr_tx_loop_halt();
				build_wave(bank, words, &s, hh, (double)fs);
				built_harm = hh;
			}
			ret = jam_tune(lo);
			if (ret != 0) {
				report_err("set_freq", ret);
				continue;
			}
			/*
			 * jam_tune() left the front end in RX (that is what
			 * esp_sdr_set_freq() does); put it back into TX or the
			 * DAC plays into a receiver. This is the fix.
			 */
			esp_sdr_tx_prepare();

			ret = esp_sdr_tx_loop_start(0, words);
			if (ret != 0) {
				report_err("loop_start", ret);
				continue;
			}
			jam_dwells++;
			for (uint32_t t = 0; t < s.dwell_ms && !jam_stop; t += 20U) {
				k_msleep(MIN(20U, s.dwell_ms - t));
			}
		}
	}
	esp_sdr_tx_loop_end();
}

/* One-shot engine: esp_sdr_tx_play_for() per dwell, our buffer tiled in. */
static void jam_run_play(const jam_cfg_t *r0)
{
	jam_cfg_t r = *r0;
	enum esp_sdr_rate rate = r.rate;
	uint32_t fs = esp_sdr_tx_rate_hz(rate);
	size_t n = JAM_BANK_WORDS;
	int built_harm = -1;
	int applied_gain = INT_MIN;
	int ret;

	if (fs == 0U) {
		rate = ESP_SDR_RATE_80MSPS;
		fs = esp_sdr_tx_rate_hz(rate);
	}
	if (fs == 0U) {
		report_err("dac_rate", -EINVAL);
		return;
	}

	(void)esp_sdr_rx_set_lpf(0);
	(void)esp_sdr_tx_set_lpf(0, 0);

	while (!jam_stop) {
		jam_cfg_t s;
		int gain;

		k_mutex_lock(&cfg_lock, K_FOREVER);
		s = cfg;
		k_mutex_unlock(&cfg_lock);

		if (s.n_hop <= 0) {
			k_msleep(20);
			continue;
		}
		if (wave_key_changed(&s, &r)) {
			r = s;
			built_harm = -1;
		}
		gain = s.gain < 0 ? esp_sdr_tx_gain_max() : s.gain;
		if (gain != applied_gain) {
			(void)esp_sdr_tx_set_gain(gain);
			applied_gain = gain;
		}

		for (int h = 0; h < s.n_hop && !jam_stop; h++) {
			int hh = harm_of(&s, s.hop[h]);
			uint32_t lo = lo_of(s.hop[h], hh);

			if (!reachable(s.hop[h], hh)) {
				report_err("range", -ERANGE);
				continue;
			}
			if (hh != built_harm) {
				build_wave(jam_bank, n, &s, hh, (double)fs);
				built_harm = hh;
			}
			/*
			 * play_for() takes the radio and switches the front end
			 * itself, so retuning between the calls is safe here.
			 */
			ret = jam_tune(lo);
			if (ret != 0) {
				report_err("set_freq", ret);
				continue;
			}
			ret = esp_sdr_tx_play_for(rate, jam_bank, n,
						  MIN(s.dwell_ms, ESP_SDR_TX_PLAY_MAX_MS), NULL);
			if (ret != 0) {
				report_err("play_for", ret);
				continue;
			}
			jam_dwells++;
		}
	}
}
#endif

#if defined(CONFIG_ESP_SDR_PACED)
static volatile uint32_t paced_rng = 0x2545f491u;

static bool paced_input(void *user, int16_t *i, int16_t *q)
{
	uint32_t x = paced_rng;

	ARG_UNUSED(user);
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	paced_rng = x;
	*i = (int16_t)(((int)(x & 0x3ffU) - 512) * 60);
	*q = (int16_t)(((int)((x >> 10) & 0x3ffU) - 512) * 60);
	return true;
}

static void jam_run_paced(const jam_cfg_t *r0)
{
	jam_cfg_t r = *r0;
	uint32_t pmin = 0U, pmax = 0U;
	unsigned int power = esp_sdr_tx_paced_power_max();
	uint32_t rate = r.prate_khz * 1000U;
	int ret;

	if (rate == 0U) {
		rate = 1000000U;
	}
	esp_sdr_tx_paced_range(&pmin, &pmax);

	while (!jam_stop) {
		jam_cfg_t s;

		k_mutex_lock(&cfg_lock, K_FOREVER);
		s = cfg;
		k_mutex_unlock(&cfg_lock);

		if (s.n_hop <= 0) {
			k_msleep(20);
			continue;
		}
		for (int h = 0; h < s.n_hop && !jam_stop; h++) {
			int hh = harm_of(&s, s.hop[h]);
			uint32_t mhz = lo_of(s.hop[h], hh) / 1000U;
			int64_t until;

			if (mhz < pmin || mhz > pmax) {
				report_err("range", -ERANGE);
				continue;
			}
			ret = esp_sdr_set_freq(mhz);
			if (ret != 0) {
				report_err("set_freq", ret);
				continue;
			}
			ret = esp_sdr_tx_paced_start(rate, 1U, power, paced_input, NULL);
			if (ret != 0) {
				report_err("paced_start", ret);
				k_msleep(20);
				continue;
			}
			esp_sdr_paced_sync();
			until = (int64_t)k_uptime_get() + (int64_t)s.dwell_ms;
			while (!jam_stop) {
				for (int k = 0; k < 128; k++) {
					(void)esp_sdr_tx_paced_poll();
				}
				if ((int64_t)k_uptime_get() >= until) {
					break;
				}
			}
			esp_sdr_tx_paced_stop();
			jam_dwells++;
		}
	}
}
#endif

static void jam_thread(void *p1, void *p2, void *p3)
{
	jam_cfg_t r;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_mutex_lock(&cfg_lock, K_FOREVER);
	r = cfg;
	k_mutex_unlock(&cfg_lock);

#if defined(CONFIG_SOC_SERIES_ESP32S3)
	if (r.engine == 1) {
		jam_run_play(&r);
	} else {
		jam_run_loop(&r);
	}
#elif defined(CONFIG_ESP_SDR_PACED)
	jam_run_paced(&r);
#else
	k_msleep(100);
#endif

	jam_active = false;
	k_sem_give(&jam_done);
}

static int jam_start(void)
{
	if (jam_active) {
		return -EALREADY;
	}
	k_sem_reset(&jam_done);
	jam_stop = false;
	k_thread_create(&jam_tcb, jam_stack, K_THREAD_STACK_SIZEOF(jam_stack), jam_thread,
			NULL, NULL, NULL, JAM_PRIO, JAM_THREAD_OPT, K_NO_WAIT);
	k_thread_name_set(&jam_tcb, "jammer");
	jam_active = true;
	return 0;
}

static void jam_stop_run(void)
{
	if (!jam_active) {
		return;
	}
	jam_stop = true;
	k_sem_take(&jam_done, K_FOREVER);
	jam_active = false;
}

/* ---- spectrum monitor ---- */

static void mon_print(const struct shell *sh, const struct esp_sdr_rx_burst *b)
{
	static const char ramp[] = " .:-=+*#%@";
	unsigned int n = MIN(b->count, (size_t)MON_MAXN);
	uint32_t fs = esp_sdr_rx_rate_hz(ESP_SDR_RATE_80MSPS);
	double dci = 0.0, dcq = 0.0, p[MON_BINS], db[MON_BINS], dtop = -200.0;
	char line[MON_BINS + 1], pk[12];
	int pt;

	if (n < 64U || fs == 0U) {
		shell_print(sh, "mon: no data");
		return;
	}
	for (unsigned int j = 0; j < n; j++) {
		dci += (double)esp_sdr_rx_i(b->words[j]);
		dcq += (double)esp_sdr_rx_q(b->words[j]);
	}
	dci /= (double)n;
	dcq /= (double)n;

	for (int k = 0; k < MON_BINS; k++) {
		double c = 2.0 * cos(TWO_PI * (double)k / (double)MON_BINS);
		double s1r = 0.0, s1i = 0.0, s2r = 0.0, s2i = 0.0;

		for (unsigned int j = 0; j < n; j++) {
			double xr = (double)esp_sdr_rx_i(b->words[j]) - dci;
			double xi = (double)esp_sdr_rx_q(b->words[j]) - dcq;
			double tr = xr + c * s1r - s2r;
			double ti = xi + c * s1i - s2i;

			s2r = s1r; s2i = s1i; s1r = tr; s1i = ti;
		}
		p[k] = (s1r * s1r + s1i * s1i + s2r * s2r + s2i * s2i -
			c * (s1r * s2r + s1i * s2i)) * 4.0 /
		       ((double)n * (double)n * 512.0 * 512.0);
	}
	for (int c = 0; c < MON_BINS; c++) {
		int k = (c + MON_BINS / 2) % MON_BINS;

		db[c] = p[k] > 0.0 ? 10.0 * log10(p[k]) : -200.0;
		if (db[c] > dtop) {
			dtop = db[c];
		}
	}
	for (int c = 0; c < MON_BINS; c++) {
		int lvl = (int)(((db[c] - dtop) + 40.0) / 40.0 * 9.0 + 0.5);

		line[c] = ramp[CLAMP(lvl, 0, 9)];
	}
	line[MON_BINS] = '\0';

	pt = (int)lround(dtop * 10.0);
	if (pt < 0) {
		snprintf(pk, sizeof(pk), "-%d.%d", (-pt) / 10, (-pt) % 10);
	} else {
		snprintf(pk, sizeof(pk), "%d.%d", pt / 10, pt % 10);
	}
	shell_print(sh, "mon f=%u.%03u h=%u.%03u b=%u w=%u.%02u p=%s",
		    esp_sdr_get_freq(), 0U,
		    fs / 2000000U, (fs / 2000U) % 1000U,
		    MON_BINS, fs / 1000000U / MON_BINS, (fs / 10000U / MON_BINS) % 100U,
		    pk);
	shell_print(sh, "  |%s|", line);
}

/* ---- shell ---- */

static void plan_lines(const struct shell *sh, const jam_cfg_t *c)
{
	for (int h = 0; h < c->n_hop; h++) {
		int hh = harm_of(c, c->hop[h]);
		uint32_t lo = lo_of(c->hop[h], hh);

		shell_print(sh, "  %u.%03u -> LO %u.%03u MHz x%d  +/-%u.%03u%s%s",
			    c->hop[h] / 1000U, c->hop[h] % 1000U,
			    lo / 1000U, lo % 1000U, hh,
			    c->span_khz / 1000U, c->span_khz % 1000U,
			    reachable(c->hop[h], hh) ? "" : "  UNREACHABLE",
			    marginal(c->hop[h], hh) ? "  MARGINAL (below the PLL window)" : "");
	}
}

static int cmd_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "mode %s  harm %s  span %u kHz  dwell %u ms  amp %d  gain %d  engine %s",
		    mode_name[cfg.mode], cfg.harm ? "forced" : "auto", cfg.span_khz,
		    cfg.dwell_ms, cfg.amp, cfg.gain, cfg.engine ? "play" : "loop");
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	shell_print(sh, "DAC %s  LO window %u..%u MHz (PLL from %u MHz)",
		    cfg.rate == ESP_SDR_RATE_40MSPS ? "40 MS/s" : "80 MS/s",
		    JAM_LO_MIN_KHZ / 1000U, JAM_LO_MAX_KHZ / 1000U, JAM_PLL_MIN_KHZ / 1000U);
#else
	shell_print(sh, "paced TX %u kHz  LO window %u..%u MHz", cfg.prate_khz,
		    JAM_LO_MIN_KHZ / 1000U, JAM_LO_MAX_KHZ / 1000U);
#endif
	plan_lines(sh, &cfg);
	shell_print(sh, "state %s", jam_active ? "JAMMING" : "idle");
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "state %s  engine %s  dwells %u  errors %u",
		    jam_active ? "JAMMING" : "idle", cfg.engine ? "play" : "loop",
		    jam_dwells, jam_errors);
	if (jam_errors) {
		shell_print(sh, "last error: %s -> %d",
			    (const char *)jam_last_where, jam_last_ret);
	}
	if (jam_range_warn) {
		shell_print(sh, "range warnings %u (VCO outside its window: tuned, may not lock)",
			    jam_range_warn);
	}
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	shell_print(sh, "gain %d (max %d)  amp %d  span %u kHz  LO %u MHz",
		    esp_sdr_tx_get_gain(), esp_sdr_tx_gain_max(), cfg.amp,
		    cfg.span_khz, esp_sdr_get_freq());
#else
	shell_print(sh, "amp %d  span %u kHz  LO %u MHz", cfg.amp, cfg.span_khz,
		    esp_sdr_get_freq());
#endif
	return 0;
}

#if defined(CONFIG_SOC_SERIES_ESP32S3)
/* Blocking end-to-end check: a real carrier on air, every return code printed. */
static int cmd_test(const struct shell *sh, size_t argc, char **argv)
{
	bool was = jam_active;
	uint32_t *bank;
	size_t words = 0;
	uint32_t fs;
	int ret;

	if (was) {
		jam_stop_run();
	}

	ret = esp_sdr_init();
	shell_print(sh, "init        %d", ret);

	fs = esp_sdr_tx_rate_hz(ESP_SDR_RATE_80MSPS);
	shell_print(sh, "dac rate    %u Hz", fs);
	shell_print(sh, "tx gain     max %d, current %d",
		    esp_sdr_tx_gain_max(), esp_sdr_tx_get_gain());

	ret = esp_sdr_set_freq(2441);
	shell_print(sh, "set_freq    %d", ret);

	ret = esp_sdr_tx_set_lpf(0, 0);
	shell_print(sh, "tx lpf      %d", ret);

	ret = esp_sdr_tx_set_gain(esp_sdr_tx_gain_max());
	shell_print(sh, "set gain    %d", ret);

	ret = esp_sdr_tx_loop_begin(ESP_SDR_RATE_80MSPS);
	shell_print(sh, "loop_begin  %d", ret);
	if (ret == 0) {
		bank = esp_sdr_tx_loop_buf(0, &words);
		shell_print(sh, "loop_buf    %u, words %u", (unsigned int)(uintptr_t)bank,
			    (unsigned int)words);
		if (bank != NULL && words >= ESP_SDR_SAMPLES_MIN) {
			words = MIN(words, (size_t)ESP_SDR_SAMPLES_MAX);
			fill_tone(bank, words, 450, 100000.0, (double)fs);
			ret = esp_sdr_tx_loop_start(0, words);
			shell_print(sh, "loop_start  %d", ret);
			if (ret == 0) {
				shell_print(sh, "CARRIER at 2441.1 MHz for 3 s");
				k_msleep(3000);
			}
		}
		esp_sdr_tx_loop_end();
	}
	shell_print(sh, "test done");
	if (was) {
		jam_start();
	}
	return 0;
}
#endif

static int cmd_ref(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "2.4 GHz ISM 2400-2483.5, licence-free worldwide:");
	shell_print(sh, "  2402 BLE adv ch37, BT inquiry    2405 Zigbee ch11");
	shell_print(sh, "  2412 WiFi ch1   2437 WiFi ch6 (AP default)   2462 WiFi ch11");
	shell_print(sh, "  2426 BLE adv ch38   2450 WiFi ch9, Zigbee ch20, ovens");
	shell_print(sh, "  2480 BLE adv ch39, BT paging   2484 WiFi ch14 (JP)");
	shell_print(sh, "  2404-2478 Bluetooth Classic, 79 x 1 MHz, AFH");
	shell_print(sh, "  Channels are 5 MHz apart; only 1/6/11 do not overlap.");
	shell_print(sh, "2.4 GHz outside ISM: 2310-2390 ham 13cm + TD-LTE B40;");
	shell_print(sh, "  2500-2570 LTE B7 UL; 2570-2620 LTE B38; 2620-2690 B7 DL/B41;");
	shell_print(sh, "  2332.5-2345 SiriusXM.");
	shell_print(sh, "Direct, sub-2.4: 1880-1900 DECT; 1805-1880 GSM1800 DL;");
	shell_print(sh, "  1850-1910 GSM1900 DL; 1930-1990 UMTS B2 DL; 2110-2170 UMTS B1 DL;");
	shell_print(sh, "  1575.42 GPS L1 (marginal, below the PLL window);");
	shell_print(sh, "  1616.5 Iridium (marginal).");
	shell_print(sh, "5 GHz: 5150-5250 UNII-1 ch36/40/44/48 (indoor);");
	shell_print(sh, "  5250-5350 UNII-2A ch52-64 (DFS); 5470-5725 UNII-2C ch100-144 (DFS);");
	shell_print(sh, "  5725-5850 UNII-3 ch149-165; 5850-5925 UNII-4 + C-V2X 5905.");
	shell_print(sh, "  5350-5470 U-NII-2B weather radar; 5645-5945 analog FPV video.");
	shell_print(sh, "6 GHz: 5925-6425 U-NII-5 (also C-band satellite uplink),");
	shell_print(sh, "  6425-6525 U-NII-6, 6525-6875 U-NII-7, 6875-7125 U-NII-8;");
	shell_print(sh, "  WiFi 6E ch1=5955 .. ch233=7115.");
	shell_print(sh, "Not reachable, LO min %u MHz: 433.92, 868, 915, 1090 ADS-B, GPS L2.",
		    JAM_LO_MIN_KHZ / 1000U);
	return 0;
}

static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "mode %s", mode_name[cfg.mode]);
		return 0;
	}
	for (int m = 0; m < (int)ARRAY_SIZE(mode_name); m++) {
		if (strcmp(argv[1], mode_name[m]) == 0) {
			k_mutex_lock(&cfg_lock, K_FOREVER);
			cfg.mode = (enum jam_mode)m;
			k_mutex_unlock(&cfg_lock);
			shell_print(sh, "mode %s", mode_name[m]);
			return 0;
		}
	}
	shell_error(sh, "modes: noise chirp cw pulse");
	return -EINVAL;
}

static int cmd_engine(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1) {
		k_mutex_lock(&cfg_lock, K_FOREVER);
		cfg.engine = strcmp(argv[1], "play") == 0 ? 1 : 0;
		k_mutex_unlock(&cfg_lock);
	}
	shell_print(sh, "engine %s (takes effect on the next start)", cfg.engine ? "play" : "loop");
	return 0;
}

static int cmd_band(const struct shell *sh, size_t argc, char **argv)
{
	const struct jam_band *b = NULL;

	if (argc < 2) {
		for (size_t i = 0; i < ARRAY_SIZE(bands); i++) {
			shell_print(sh, "  %-10s span %5u kHz  %2d target(s)", bands[i].name,
				    bands[i].span_khz, bands[i].n);
		}
		return 0;
	}
	for (size_t i = 0; i < ARRAY_SIZE(bands); i++) {
		if (strcmp(argv[1], bands[i].name) == 0) {
			b = &bands[i];
			break;
		}
	}
	if (b == NULL) {
		shell_error(sh, "unknown band");
		return -EINVAL;
	}
	k_mutex_lock(&cfg_lock, K_FOREVER);
	cfg.harm = 0;
	cfg.span_khz = b->span_khz;
	cfg.n_hop = b->n;
	memcpy(cfg.hop, b->target_khz, sizeof(uint32_t) * (size_t)b->n);
	k_mutex_unlock(&cfg_lock);
	shell_print(sh, "band %s: %d target(s), span %u kHz", b->name, b->n, b->span_khz);
	plan_lines(sh, &cfg);
	return 0;
}

/* Parse everything first: a bad value must not leave a half-applied plan. */
static int cmd_lo(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t hop[JAM_MAX_HOPS];
	int n = 0;

	if (argc < 2) {
		plan_lines(sh, &cfg);
		return 0;
	}
	for (size_t i = 1; i < argc && n < JAM_MAX_HOPS; i++) {
		char *end;
		double v = strtod(argv[i], &end);
		long khz;

		if (end == argv[i]) {
			shell_error(sh, "%s is not a frequency", argv[i]);
			return -EINVAL;
		}
		khz = lround(v * 1000.0);
		if (khz < 100000 || khz > 6000000) {
			shell_error(sh, "%s MHz out of range", argv[i]);
			return -EINVAL;
		}
		hop[n++] = (uint32_t)khz;
	}
	if (n == 0) {
		shell_error(sh, "no target");
		return -EINVAL;
	}
	k_mutex_lock(&cfg_lock, K_FOREVER);
	memcpy(cfg.hop, hop, sizeof(hop[0]) * (size_t)n);
	cfg.n_hop = n;
	k_mutex_unlock(&cfg_lock);
	shell_print(sh, "%d target(s) set", n);
	plan_lines(sh, &cfg);
	return 0;
}

/*
 * plan <span_khz> <mode> <amp> <dwell_ms> <gain> <harm|auto> [MHz ...]:
 * the whole plan in one line, so the control page can apply it with a single
 * command instead of a burst. Targets left out are kept.
 */
static int cmd_plan(const struct shell *sh, size_t argc, char **argv)
{
	jam_cfg_t c;
	uint32_t hop[JAM_MAX_HOPS];
	int n = 0, mode = -1;

	if (argc < 7) {
		shell_error(sh, "usage: jam plan <span_khz> <mode> <amp> <dwell_ms> <gain> <harm|auto> [MHz ...]");
		return -EINVAL;
	}
	for (int m = 0; m < (int)ARRAY_SIZE(mode_name); m++) {
		if (strcmp(argv[2], mode_name[m]) == 0) {
			mode = m;
		}
	}
	if (mode < 0) {
		shell_error(sh, "mode: noise chirp cw pulse");
		return -EINVAL;
	}
	for (size_t i = 7; i < argc && n < JAM_MAX_HOPS; i++) {
		char *end;
		double v = strtod(argv[i], &end);
		long khz;

		if (end == argv[i]) {
			shell_error(sh, "%s is not a frequency", argv[i]);
			return -EINVAL;
		}
		khz = lround(v * 1000.0);
		if (khz < 100000 || khz > 6000000) {
			shell_error(sh, "%s MHz out of range", argv[i]);
			return -EINVAL;
		}
		hop[n++] = (uint32_t)khz;
	}

	k_mutex_lock(&cfg_lock, K_FOREVER);
	c = cfg;
	c.span_khz = (uint32_t)CLAMP(strtol(argv[1], NULL, 0), 10, 40000);
	c.mode = (enum jam_mode)mode;
	c.amp = (int)CLAMP(strtol(argv[3], NULL, 0), 1, 511);
	c.dwell_ms = (uint32_t)CLAMP(strtol(argv[4], NULL, 0), 5, 5000);
	c.gain = (int)strtol(argv[5], NULL, 0);
	c.harm = strcmp(argv[6], "auto") == 0 ? 0 : (int)CLAMP(strtol(argv[6], NULL, 0), 0, 3);
	if (n > 0) {
		memcpy(c.hop, hop, sizeof(hop[0]) * (size_t)n);
		c.n_hop = n;
	}
	cfg = c;
	k_mutex_unlock(&cfg_lock);

	shell_print(sh, "plan set: %d target(s), %s, span %u kHz, dwell %u ms, amp %d, gain %d",
		    cfg.n_hop, mode_name[cfg.mode], cfg.span_khz, cfg.dwell_ms, cfg.amp,
		    cfg.gain);
	plan_lines(sh, &cfg);
	return 0;
}

static int cmd_harm(const struct shell *sh, size_t argc, char **argv)
{
	int harm;

	if (argc > 1 && strcmp(argv[1], "auto") == 0) {
		harm = 0;
	} else {
		harm = (int)CLAMP(arg(argv, argc, 1, 0), 0, 3);
	}
	k_mutex_lock(&cfg_lock, K_FOREVER);
	cfg.harm = harm;
	k_mutex_unlock(&cfg_lock);
	shell_print(sh, "harm %s", cfg.harm ? "forced" : "auto");
	return 0;
}

static int cmd_span(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t span = (uint32_t)CLAMP(arg(argv, argc, 1, 12000), 10, 40000);

	k_mutex_lock(&cfg_lock, K_FOREVER);
	cfg.span_khz = span;
	k_mutex_unlock(&cfg_lock);
	shell_print(sh, "span %u kHz at the target", span);
	return 0;
}

static int cmd_dwell(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t dwell = (uint32_t)CLAMP(arg(argv, argc, 1, 40), 5, 5000);

	k_mutex_lock(&cfg_lock, K_FOREVER);
	cfg.dwell_ms = dwell;
	k_mutex_unlock(&cfg_lock);
	shell_print(sh, "dwell %u ms", dwell);
	return 0;
}

static int cmd_amp(const struct shell *sh, size_t argc, char **argv)
{
	int amp = (int)CLAMP(arg(argv, argc, 1, 450), 1, 511);

	k_mutex_lock(&cfg_lock, K_FOREVER);
	cfg.amp = amp;
	k_mutex_unlock(&cfg_lock);
	shell_print(sh, "amp %d", amp);
	return 0;
}

static int cmd_gain(const struct shell *sh, size_t argc, char **argv)
{
	int gain = (int)arg(argv, argc, 1, -1);

	k_mutex_lock(&cfg_lock, K_FOREVER);
	cfg.gain = gain;
	k_mutex_unlock(&cfg_lock);
	shell_print(sh, "gain %d (-1 = strongest)", gain);
	return 0;
}

static int cmd_rate(const struct shell *sh, size_t argc, char **argv)
{
	enum esp_sdr_rate rate = arg(argv, argc, 1, 0) == 0 ? ESP_SDR_RATE_80MSPS
							    : ESP_SDR_RATE_40MSPS;

	k_mutex_lock(&cfg_lock, K_FOREVER);
	cfg.rate = rate;
	k_mutex_unlock(&cfg_lock);
	shell_print(sh, "DAC rate %s (takes effect on the next start)",
		    rate == ESP_SDR_RATE_80MSPS ? "80 MS/s" : "40 MS/s");
	return 0;
}

static int cmd_prate(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t prate = (uint32_t)CLAMP(arg(argv, argc, 1, 1000), 10, 4000);

	k_mutex_lock(&cfg_lock, K_FOREVER);
	cfg.prate_khz = prate;
	k_mutex_unlock(&cfg_lock);
	shell_print(sh, "paced rate %u kHz", prate);
	return 0;
}

static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (jam_active) {
		shell_print(sh, "already jamming");
		return 0;
	}
	/* Idempotent: recovers a start if the boot-time init did not take. */
	ret = esp_sdr_init();
	if (ret != 0) {
		shell_error(sh, "radio not up: esp_sdr_init %d", ret);
	}
	if (cfg.n_hop <= 0) {
		shell_error(sh, "no target: jam lo <MHz>");
		return -EINVAL;
	}
	if (jam_start() != 0) {
		shell_error(sh, "start failed");
		return -EIO;
	}
	shell_print(sh, "jamming: %s, %d target(s), first %u.%03u MHz, engine %s",
		    mode_name[cfg.mode], cfg.n_hop, cfg.hop[0] / 1000U, cfg.hop[0] % 1000U,
		    cfg.engine ? "play" : "loop");
	plan_lines(sh, &cfg);
	return 0;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!jam_active) {
		shell_print(sh, "idle");
		return 0;
	}
	jam_stop_run();
	shell_print(sh, "stopped after %u dwell(s), %u error(s)", jam_dwells, jam_errors);
	return 0;
}

static int cmd_mon(const struct shell *sh, size_t argc, char **argv)
{
	size_t n = (size_t)CLAMP(arg(argv, argc, 1, 2048), (long)ESP_SDR_SAMPLES_MIN,
				 (long)ESP_SDR_SAMPLES_MAX);
	struct esp_sdr_rx_burst b = { 0 };
	bool was = jam_active;
	int ret;

	/* The loop engine holds the radio, so park the jammer for the capture. */
	if (was) {
		jam_stop_run();
	}
	ret = esp_sdr_rx_capture(ESP_SDR_RATE_80MSPS, n, &b);
	if (ret != 0) {
		shell_error(sh, "capture %d", ret);
	} else {
		mon_print(sh, &b);
	}
	if (was) {
		jam_start();
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(jam_cmds,
	SHELL_CMD(info, NULL, "plan, settings and state", cmd_info),
	SHELL_CMD(status, NULL, "counters and last error", cmd_status),
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	SHELL_CMD(test, NULL, "put a carrier on air for 3 s, print every ret", cmd_test),
#endif
	SHELL_CMD(ref, NULL, "what lives on which frequency", cmd_ref),
	SHELL_CMD_ARG(mode, NULL, "[noise|chirp|cw|pulse]", cmd_mode, 1, 1),
	SHELL_CMD_ARG(engine, NULL, "[loop|play]", cmd_engine, 1, 1),
	SHELL_CMD_ARG(band, NULL, "[name] preset", cmd_band, 1, 1),
	SHELL_CMD_ARG(plan, NULL,
		      "<span_khz> <mode> <amp> <dwell_ms> <gain> <harm|auto> [MHz ...]",
		      cmd_plan, 7, JAM_MAX_HOPS),
	SHELL_CMD_ARG(lo, NULL, "<MHz> [MHz ...] exact targets, decimals ok", cmd_lo, 1, JAM_MAX_HOPS),
	SHELL_CMD_ARG(harm, NULL, "[auto|1|2|3] LO = target / harm", cmd_harm, 1, 1),
	SHELL_CMD_ARG(span, NULL, "<kHz> half width at the target", cmd_span, 2, 0),
	SHELL_CMD_ARG(dwell, NULL, "<ms> per target", cmd_dwell, 2, 0),
	SHELL_CMD_ARG(amp, NULL, "<1..511> digital amplitude", cmd_amp, 2, 0),
	SHELL_CMD_ARG(gain, NULL, "<step|-1> TX power, -1 = strongest", cmd_gain, 2, 0),
	SHELL_CMD_ARG(rate, NULL, "<0|1> S3 DAC rate, 0 = 80 MS/s", cmd_rate, 2, 0),
	SHELL_CMD_ARG(prate, NULL, "<kHz> paced TX rate (C5/C6)", cmd_prate, 2, 0),
	SHELL_CMD(start, NULL, "start jamming", cmd_start),
	SHELL_CMD(stop, NULL, "stop jamming", cmd_stop),
	SHELL_CMD_ARG(mon, NULL, "[n] capture and print a spectrum", cmd_mon, 1, 1),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(jam, &jam_cmds, "ESP-SDR spectrum jammer", NULL);

int main(void)
{
	int ret;

	printk("jammer %s\n", APP_VERSION_STRING);
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	if (app_usb_init() != 0) {
		printk("jammer: usb init failed\n");
	}
#endif
	cfg_defaults();

	ret = esp_sdr_init();
	printk("jammer: esp_sdr_init %d\n", ret);
	if (ret != 0) {
		printk("jammer: radio is NOT up -- jam start will do nothing\n");
	}
	(void)esp_sdr_rx_set_lpf(0);
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	(void)esp_sdr_tx_set_lpf(0, 0);
	printk("jammer: dac %u Hz, tx gain max %d\n",
	       esp_sdr_tx_rate_hz(ESP_SDR_RATE_80MSPS), esp_sdr_tx_gain_max());
#endif
	printk("jammer: ready -- run 'jam test' first, then 'jam start'\n");
	return 0;
}
JAM_MAIN_C

  cat "$REPO/VERSION" > "$APP/VERSION"
  printf '%s\n' "$JAM_REV" > "$APP/.jam_rev"
  tail -n 1 "$APP/src/main.c" | grep -q '^}' || warn "apps/jammer/src/main.c looks truncated"
  log "wrote $(find "$APP" -type f | wc -l) files into apps/jammer"
fi

# --- 4. the WebSerial control page -----------------------------------------
# A half-written page (cut save) has no rev marker or no closing tag: rewrite it.
if [ -e "$HTML" ] && [ "$FORCE" != 1 ] && grep -q "JAMREV:$JAM_REV" "$HTML" \
   && tail -n 1 "$HTML" | grep -q '</html>'; then
  log "jammer.html is already at rev $JAM_REV, leaving it (--force to rewrite)"
else
  log "writing $HTML"
  cat > "$HTML" <<'JAM_HTML'
<!doctype html>
<!-- JAMREV:5 -->
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover" />
<meta name="color-scheme" content="dark" />
<title>ESP-SDR Jammer</title>
<style>
  :root{--bg:#0b0e13;--panel:#141a23;--panel2:#1b2330;--line:#26303f;--fg:#e6edf6;
        --dim:#8b9bb0;--ok:#4ade80;--bad:#f87171;--acc:#60a5fa;--warn:#fbbf24}
  *{box-sizing:border-box}
  html,body{margin:0;background:var(--bg);color:var(--fg);
    font:15px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;-webkit-text-size-adjust:100%}
  body{padding:10px 10px calc(24px + env(safe-area-inset-bottom));max-width:760px;margin:0 auto}
  header{display:flex;align-items:center;gap:9px;margin:4px 0 12px}
  h1{font-size:16px;margin:0;flex:1}
  h1 span{color:var(--dim);font-weight:400;font-size:12px;display:block}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:14px;
    padding:12px;margin-bottom:10px}
  .card h2{font-size:12px;text-transform:uppercase;letter-spacing:.08em;color:var(--dim);
    margin:0 0 9px;font-weight:600}
  .row{display:flex;gap:8px;flex-wrap:wrap}
  button{font:inherit;color:var(--fg);background:var(--panel2);border:1px solid var(--line);
    border-radius:10px;padding:11px 13px;cursor:pointer;-webkit-tap-highlight-color:transparent}
  button:active{transform:scale(.98)}
  button:disabled{opacity:.4}
  button.acc{background:#1d3a2a;border-color:#2c5a40;color:#b7f0cd}
  button.stop{background:#3a1d1d;border-color:#5a2c2c;color:#f5c2c2}
  button.test{background:#2a2a3a;border-color:#4a4a6a;color:#d6d6ff}
  button.on{background:#20406b;border-color:#356399;color:#cfe3ff}
  .modes button{flex:1}
  .status{font-size:12px;padding:4px 9px;border-radius:999px;background:var(--panel2);
    border:1px solid var(--line);color:var(--dim);white-space:nowrap}
  .status.ok{color:#b7f0cd;border-color:#2c5a40;background:#12271c}
  .status.bad{color:#f5c2c2;border-color:#5a2c2c;background:#2a1414}
  .sl{margin:9px 0 2px}
  .sl label{display:flex;justify-content:space-between;font-size:12.5px;color:var(--dim)}
  .sl label b{color:var(--fg);font-variant-numeric:tabular-nums}
  input[type=range]{width:100%;margin:6px 0 0;accent-color:var(--acc)}
  canvas{width:100%;height:150px;display:block;background:#0d1219;border-radius:10px;
    border:1px solid var(--line)}
  #monhdr{font-size:12px;color:var(--dim);margin-top:7px;font-variant-numeric:tabular-nums}
  pre#log{margin:0;height:220px;overflow:auto;background:#0d1219;border:1px solid var(--line);
    border-radius:10px;padding:8px;font:12px/1.4 ui-monospace,Menlo,Consolas,monospace;
    white-space:pre-wrap;word-break:break-all;color:#c7d3e2}
  input[type=text],input[type=search]{font:13px ui-monospace,Menlo,Consolas,monospace;
    background:#0d1219;color:var(--fg);border:1px solid var(--line);border-radius:10px;
    padding:10px;min-width:0}
  .raw{display:flex;gap:8px;margin-top:9px}
  .raw input{flex:1}
  .note{font-size:12px;color:var(--dim);margin:7px 0 0}
  .note b{color:var(--warn);font-weight:600}
  .grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
  .chips{display:flex;flex-wrap:wrap;gap:6px;margin:9px 0}
  .chip{display:flex;align-items:center;gap:7px;background:#1b2330;border:1px solid var(--line);
    border-radius:999px;padding:6px 6px 6px 12px;font:12.5px ui-monospace,monospace}
  .chip button{padding:2px 8px;border-radius:999px;font-size:13px;line-height:1}
  .chip em{font-style:normal;color:var(--dim)}
  #reflist{max-height:300px;overflow:auto;border:1px solid var(--line);border-radius:10px;
    background:#0d1219}
  .grp{position:sticky;top:0;background:#121924;color:var(--dim);font-size:11px;
    letter-spacing:.07em;text-transform:uppercase;padding:7px 10px;border-bottom:1px solid var(--line)}
  .refrow{display:flex;gap:9px;align-items:baseline;padding:8px 10px;
    border-bottom:1px solid #161e29;cursor:pointer}
  .refrow:active{background:#151d28}
  .refrow.sel{background:#16283c}
  .refrow .f{font:12.5px ui-monospace,monospace;color:var(--fg);width:74px;flex:none;
    font-variant-numeric:tabular-nums}
  .refrow .t{flex:1;font-size:12.5px;color:#c7d3e2}
  .refrow .w{font-size:11px;color:var(--dim);flex:none}
  .refrow .r{font-size:10.5px;padding:2px 7px;border-radius:999px;flex:none;
    border:1px solid var(--line);color:var(--dim);white-space:nowrap}
  .refrow .r.d{color:#b7f0cd;border-color:#2c5a40}
  .refrow .r.h2{color:#ffe0a3;border-color:#5a4a20}
  .refrow .r.h3{color:#f5c2c2;border-color:#5a2c2c}
  .refrow .r.warn{color:#ffe0a3;border-color:#5a4a20}
  .refrow.out{opacity:.5}
  .seg{display:flex;border:1px solid var(--line);border-radius:10px;overflow:hidden}
  .seg button{flex:1;border:0;border-radius:0;background:transparent}
  .seg button.on{background:#20406b;color:#cfe3ff}
</style>
</head>
<body>

<header>
  <h1>ESP-SDR Jammer<span>exact-frequency picker</span></h1>
  <div id="status" class="status">offline</div>
  <button id="connect">Connect</button>
</header>

<div class="card">
  <h2>0 · Is it transmitting?</h2>
  <div class="grid2">
    <button id="test" class="test">Test carrier (3 s)</button>
    <button id="stat">Status / errors</button>
  </div>
  <p class="note">Do this first. <b>Test carrier</b> puts a 2441.1 MHz CW carrier on
  air for 3 s and prints every radio return code — watch the log below.</p>
</div>

<div class="card">
  <h2>1 · Pick the exact target frequency</h2>
  <input type="search" id="search" placeholder="search: wifi, ble, 5180, dfs…"
         style="width:100%" autocapitalize="off" autocorrect="off" spellcheck="false" />
  <div id="reflist" style="margin-top:9px"></div>
  <div class="raw">
    <input type="text" id="manual" placeholder="exact MHz, e.g. 2412.5" />
    <button id="add">Add</button>
  </div>
  <div class="chips" id="chips"></div>
  <div class="row">
    <button id="apply" class="acc" style="flex:1">Apply to radio</button>
    <button id="clear">Clear</button>
  </div>
  <p class="note" id="plan">no target selected</p>
</div>

<div class="card">
  <h2>2 · Waveform</h2>
  <div class="row modes" id="modes"></div>
  <div class="grid2" style="margin-top:10px">
    <button id="start" class="acc">Start jamming</button>
    <button id="stop" class="stop">Stop</button>
  </div>
  <p class="note">Start applies the plan and then starts. Mode and the sliders act
  <b>while jamming</b>. Commands go out one at a time and every one is
  confirmed in the log, so a dropped character cannot leave you guessing.</p>
</div>

<div class="card">
  <h2>3 · Tuning</h2>
  <div class="sl"><label>Half width at target <b><span id="v_span"></span> kHz</b></label>
    <input type="range" id="span" min="100" max="40000" step="100" value="39000"></div>
  <div class="sl"><label>Dwell per target <b><span id="v_dwell"></span> ms</b></label>
    <input type="range" id="dwell" min="5" max="1000" step="5" value="40"></div>
  <div class="sl"><label>Digital amplitude <b><span id="v_amp"></span></b></label>
    <input type="range" id="amp" min="10" max="511" step="1" value="450"></div>
  <div class="sl"><label>TX power step <b><span id="v_gain"></span></b> (−1 = strongest)</label>
    <input type="range" id="gain" min="-1" max="17" step="1" value="-1"></div>
  <div class="sl"><label>Harmonic</label>
    <div class="seg" id="harm">
      <button data-h="0" class="on">auto</button>
      <button data-h="1">1st</button>
      <button data-h="2">2nd</button>
      <button data-h="3">3rd</button>
    </div>
  </div>
  <p class="note">Span is the half-width <b>at the target</b>; the baseband is
  span/harm. Pick the row whose width matches the thing you want to cover
  (a whole band, or one 22 MHz channel row for that channel).</p>
</div>

<div class="card">
  <h2>Spectrum</h2>
  <canvas id="spec"></canvas>
  <div class="row" style="margin-top:9px">
    <button id="mon">Monitor</button>
    <button id="info">Read plan</button>
  </div>
  <div id="monhdr">no capture yet — note the jammer parks while capturing</div>
</div>

<div class="card">
  <h2>Shell</h2>
  <pre id="log"></pre>
  <div class="raw">
    <input type="text" id="cmd" placeholder="jam ref" autocapitalize="off"
           autocomplete="off" autocorrect="off" spellcheck="false" />
    <button id="send">Send</button>
  </div>
</div>

<script>
"use strict";
const RAMP = " .:-=+*#%@";
const enc  = new TextEncoder();
const el   = id => document.getElementById(id);
const sleep = ms => new Promise(r => setTimeout(r, ms));

let port = null, writer = null, reader = null, reading = false, connected = false;
let running = false, rxbuf = "", pending = "", monTimer = null, monBusy = false;
let monHeader = null, harmMode = 0, currentMode = "noise";
const lines = [];
const selected = new Map();

/*
 * The device's shell has a small RX ring: writing several commands at once
 * loses characters, and the shell then eats half a command line. So writes go
 * out one at a time, spaced, and every important command is verified.
 */
const GAP_MS = 40;
const outq = [];
let pumping = false;

async function pump() {
  if (pumping) return;
  pumping = true;
  while (outq.length) {
    const it = outq.shift();
    if (writer) {
      try {
        await Promise.race([writer.write(enc.encode(it.cmd + '\r')), sleep(1200)]);
      } catch (e) {
        log('write failed: ' + (e && e.message ? e.message : e));
      }
    }
    if (it.done) it.done();
    await sleep(it.gap);
  }
  pumping = false;
}

function send(cmd, gap = GAP_MS) {
  if (!writer) return Promise.resolve();
  if (outq.length > 64) outq.splice(0, outq.length - 64);
  return new Promise(done => { outq.push({ cmd, gap, done }); pump(); });
}

/* Resolves true when a line matching re arrives, false on timeout. */
let waiters = [];
function expect(re, ms = 2500) {
  return new Promise(res => {
    const w = { re, res };
    w.t = setTimeout(() => { waiters = waiters.filter(x => x !== w); res(false); }, ms);
    waiters.push(w);
  });
}
function feedWaiters(line) {
  if (!waiters.length) return;
  waiters = waiters.filter(w => {
    if (w.re.test(line)) { clearTimeout(w.t); w.res(true); return false; }
    return true;
  });
}

/* Send a command and insist on a reply; one retry after flushing a partial line. */
async function ensure(re, cmd, tries = 2) {
  for (let i = 1; i <= tries; i++) {
    const p = expect(re, 2500);
    await send(cmd);
    if (await p) return true;
    log('no reply to "' + cmd + '"' + (i < tries ? ' -- retrying' : ''));
    if (i < tries) await send('');
  }
  log('WARNING: "' + cmd + '" was not confirmed -- press it again');
  return false;
}

const on = (id, fn) => {
  el(id).onclick = async () => {
    try { await fn(); } catch (e) { log('error: ' + (e && e.message ? e.message : e)); }
  };
};

/*
 * The S3 PLL locks about 1817..2790 MHz (doc/hardware-quirks.md). Below that
 * the LO is still attempted (marginal, may not lock); outside 1514..2790 the
 * firmware skips the hop, so those rows are dropped here too.
 */
const LO_MIN = 1817, LO_MAX = 2790, LO_TRY_MIN = 1514;

function reach(t) {
  const h = t <= LO_MAX ? 1 : (t / 2 <= LO_MAX ? 2 : 3);
  const lo = t / h;
  if (lo > LO_MAX || lo < LO_TRY_MIN) return { h: 0, cls: "out", lbl: "out of range" };
  if (lo < LO_MIN) return { h, cls: "warn", lbl: "marginal" };
  return { h, cls: h === 1 ? "d" : "h" + h, lbl: h === 1 ? "direct" : (h === 2 ? "2nd harm" : "3rd harm") };
}

const REF = [
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2402, "BLE advertising ch37 · BT inquiry · nRF24 ch2", 2],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2405, "Zigbee / 802.15.4 ch11", 2],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2412, "Wi-Fi ch1 (US/EU non-overlapping 1·6·11)", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2417, "Wi-Fi ch2", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2422, "Wi-Fi ch3", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2426, "BLE advertising ch38 · Zigbee ch15", 2],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2427, "Wi-Fi ch4", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2432, "Wi-Fi ch5", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2437, "Wi-Fi ch6 — most common AP default", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2442, "Wi-Fi ch7", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2447, "Wi-Fi ch8", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2450, "Wi-Fi ch9 · Zigbee ch20 · microwave ovens", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2452, "Wi-Fi ch10", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2457, "Wi-Fi ch11", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2462, "Wi-Fi ch11 centre · Zigbee ch22", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2467, "Wi-Fi ch12 (EU/JP)", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2472, "Wi-Fi ch13 (EU/JP)", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2475, "Zigbee ch25", 2],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2480, "BLE adv ch39 · BT paging · Zigbee ch26", 2],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2484, "Wi-Fi ch14 (Japan only)", 22],
["2.4 GHz ISM 2400–2483.5 — licence-free worldwide", 2441, "whole ISM band, one LO at 2441", 83],
["2.4 GHz, outside ISM", 2320, "TD-LTE band 40 / amateur 13 cm", 20],
["2.4 GHz, outside ISM", 2338, "SiriusXM satellite radio", 12],
["2.4 GHz, outside ISM", 2535, "LTE band 7 uplink", 70],
["2.4 GHz, outside ISM", 2595, "LTE band 38 TDD", 50],
["2.4 GHz, outside ISM", 2655, "LTE band 7 downlink / band 41", 70],
["Sub-2.4 GHz and cellular", 1575.42, "GPS L1 · Galileo E1 · BeiDou B1 (marginal)", 4],
["Sub-2.4 GHz and cellular", 1616.5, "Iridium (marginal)", 10],
["Sub-2.4 GHz and cellular", 1840, "GSM1800 / LTE band 3 downlink", 75],
["Sub-2.4 GHz and cellular", 1880, "GSM1900 / PCS / LTE band 2 downlink", 60],
["Sub-2.4 GHz and cellular", 1890, "DECT cordless phones (EU)", 20],
["Sub-2.4 GHz and cellular", 1960, "UMTS / LTE band 2 downlink", 60],
["Sub-2.4 GHz and cellular", 2140, "UMTS / LTE band 1 downlink", 60],
["5 GHz Wi-Fi and friends", 5180, "Wi-Fi ch36 · UNII-1 (indoor)", 20],
["5 GHz Wi-Fi and friends", 5200, "Wi-Fi ch40 · UNII-1", 20],
["5 GHz Wi-Fi and friends", 5220, "Wi-Fi ch44 · UNII-1", 20],
["5 GHz Wi-Fi and friends", 5240, "Wi-Fi ch48 · UNII-1", 20],
["5 GHz Wi-Fi and friends", 5200, "whole UNII-1 5150–5250, one LO at 5200", 100],
["5 GHz Wi-Fi and friends", 5260, "Wi-Fi ch52 · UNII-2A (DFS)", 20],
["5 GHz Wi-Fi and friends", 5280, "Wi-Fi ch56 · UNII-2A (DFS)", 20],
["5 GHz Wi-Fi and friends", 5300, "Wi-Fi ch60 · UNII-2A (DFS)", 20],
["5 GHz Wi-Fi and friends", 5320, "Wi-Fi ch64 · UNII-2A (DFS)", 20],
["5 GHz Wi-Fi and friends", 5410, "U-NII-2B — weather and military radar", 120],
["5 GHz Wi-Fi and friends", 5500, "Wi-Fi ch100 · UNII-2C (DFS)", 20],
["5 GHz Wi-Fi and friends", 5580, "Wi-Fi ch116 · UNII-2C (DFS)", 20],
["5 GHz Wi-Fi and friends", 5660, "Wi-Fi ch132 · UNII-2C (DFS)", 20],
["5 GHz Wi-Fi and friends", 5700, "Wi-Fi ch140 · UNII-2C (DFS)", 20],
["5 GHz Wi-Fi and friends", 5720, "Wi-Fi ch144 · UNII-2C (DFS)", 20],
["5 GHz Wi-Fi and friends", 5745, "Wi-Fi ch149 · UNII-3", 20],
["5 GHz Wi-Fi and friends", 5765, "Wi-Fi ch153 · UNII-3", 20],
["5 GHz Wi-Fi and friends", 5785, "Wi-Fi ch157 · UNII-3", 20],
["5 GHz Wi-Fi and friends", 5805, "Wi-Fi ch161 · UNII-3", 20],
["5 GHz Wi-Fi and friends", 5825, "Wi-Fi ch165 · UNII-3", 20],
["5 GHz Wi-Fi and friends", 5785, "whole UNII-3 / 5.8 ISM 5725–5850, one LO at 5785", 125],
["5 GHz Wi-Fi and friends", 5880, "U-NII-4 (US) · DSRC 5895 · C-V2X 5905", 75],
["5 GHz Wi-Fi and friends", 5795, "analog FPV video and wireless senders 5645–5945", 300],
["6 GHz Wi-Fi 6E / 7", 5955, "Wi-Fi 6E ch1", 20],
["6 GHz Wi-Fi 6E / 7", 6135, "Wi-Fi 6E ch37", 20],
["6 GHz Wi-Fi 6E / 7", 6315, "Wi-Fi 6E ch73", 20],
["6 GHz Wi-Fi 6E / 7", 6495, "Wi-Fi 6E ch109", 20],
["6 GHz Wi-Fi 6E / 7", 6675, "Wi-Fi 6E ch145", 20],
["6 GHz Wi-Fi 6E / 7", 6855, "Wi-Fi 6E ch181", 20],
["6 GHz Wi-Fi 6E / 7", 7035, "Wi-Fi 6E ch217", 20],
["6 GHz Wi-Fi 6E / 7", 7115, "Wi-Fi 6E ch233 (last)", 20],
["6 GHz Wi-Fi 6E / 7", 6175, "U-NII-5 5925–6425 — also the C-band satellite uplink", 500],
["6 GHz Wi-Fi 6E / 7", 6475, "U-NII-6 6425–6525", 100],
["6 GHz Wi-Fi 6E / 7", 6700, "U-NII-7 6525–6875", 350],
["6 GHz Wi-Fi 6E / 7", 7000, "U-NII-8 6875–7125", 250],
["Below the LO window", 433.92, "ISM 433 — remotes, sensors, TPMS", 1.7],
["Below the LO window", 868, "ISM 868 (EU) — LoRa, smart meters", 1],
["Below the LO window", 915, "ISM 915 (US) — LoRa, ISM", 26],
["Below the LO window", 1090, "ADS-B / Mode-S", 2],
["Below the LO window", 1227.6, "GPS L2", 20]
];

function setStatus(t, cls) { const s = el('status'); s.textContent = t; s.className = 'status ' + (cls || ''); }
function log(s) {
  lines.push(s);
  if (lines.length > 500) lines.splice(0, lines.length - 500);
  const p = el('log'); p.textContent = lines.join('\n'); p.scrollTop = p.scrollHeight;
}

async function connect() {
  if (!('serial' in navigator)) { alert('No WebSerial here. Use Chrome or Edge.'); return; }
  try {
    const filt = { filters: [{ usbVendorId: 0x303a }, { usbVendorId: 0x2fe3 },
                             { usbVendorId: 0x1a86 }, { usbVendorId: 0x10c4 }] };
    try { port = await navigator.serial.requestPort(filt); }
    catch (e) { port = await navigator.serial.requestPort(); }
    await port.open({ baudRate: 115200 });
    writer = port.writable.getWriter();
    reader = port.readable.getReader();
    reading = true; connected = true;
    setStatus('connected', 'ok');
    el('connect').textContent = 'Disconnect';
    readLoop();
    await send('');
    await ensure(/^state /, 'jam status');
    await send('jam info');
    try { await navigator.wakeLock.request('screen'); } catch (e) {}
  } catch (e) {
    setStatus('failed', 'bad');
    log('connect: ' + (e && e.message ? e.message : e));
  }
}

async function disconnect() {
  reading = false; connected = false;
  if (monTimer) { clearInterval(monTimer); monTimer = null; el('mon').textContent = 'Monitor'; }
  while (outq.length) { const it = outq.shift(); if (it.done) it.done(); }
  try { if (reader) { await reader.cancel(); reader.releaseLock(); } } catch (e) {}
  try { if (writer) writer.releaseLock(); } catch (e) {}
  try { if (port) await port.close(); } catch (e) {}
  reader = writer = port = null;
  setStatus('offline');
  el('connect').textContent = 'Connect';
}

async function readLoop() {
  const dec = new TextDecoder();
  while (reading) {
    let r;
    try { r = await reader.read(); } catch (e) { break; }
    if (r.done) break;
    rxbuf += dec.decode(r.value, { stream: true });
    let i;
    while ((i = rxbuf.indexOf('\n')) >= 0) {
      onLine(rxbuf.slice(0, i).replace(/\r$/, ''));
      rxbuf = rxbuf.slice(i + 1);
    }
    if (rxbuf.length > 8192) rxbuf = rxbuf.slice(-4096);
  }
  if (connected) { connected = false; setStatus('lost', 'bad'); }
}

const RE_MON = /mon f=(\d+)\.(\d{3}) h=(\d+)\.(\d{3}) b=(\d+) w=(\d+)\.(\d{2}) p=(-?\d+)\.(\d)/;
const RE_BAR = /^\s*\|([^|]+)\|\s*$/;

function onLine(raw) {
  const line = pending ? pending + raw : raw;
  if (line.indexOf('mon f=') >= 0 && line.indexOf(' p=') < 0) { pending = line; return; }
  pending = '';
  log(line);
  feedWaiters(line);

  /* The device is the authority on what it is doing. */
  if (/state JAMMING|^jamming:/.test(line)) { running = true; setStatus('jamming', 'ok'); }
  else if (/^state idle|^stopped|^already jamming|^idle$/.test(line)) {
    running = false; if (connected) setStatus('connected', 'ok');
  } else if (/failed \(|not up|UNREACHABLE/.test(line) && connected) {
    setStatus('check log', 'bad');
  }

  let m = line.match(RE_MON);
  if (m) {
    monHeader = { lo: +m[1] + (+m[2]) / 1000, half: +m[3] + (+m[4]) / 1000,
                  bins: +m[5], bw: +m[6] + (+m[7]) / 100, peak: parseFloat(m[8] + '.' + m[9]) };
    return;
  }
  m = line.match(RE_BAR);
  if (m && monHeader) {
    drawSpectrum(m[1], monHeader);
    el('monhdr').textContent = 'LO ' + monHeader.lo.toFixed(3) + ' MHz  ·  ±' +
      monHeader.half.toFixed(3) + ' MHz  ·  ' + monHeader.bins + ' bins @ ' +
      monHeader.bw.toFixed(2) + ' MHz  ·  peak ' + monHeader.peak.toFixed(1) + ' dBFS';
    monBusy = false; monHeader = null;
  }
}

function drawSpectrum(bars, h) {
  const c = el('spec'), ctx = c.getContext('2d');
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const W = c.clientWidth, H = c.clientHeight;
  c.width = W * dpr; c.height = H * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const n = bars.length, bw = W / n, base = H - 16;
  const f0 = h.lo - h.half, f1 = h.lo + h.half;
  ctx.strokeStyle = '#1e2836'; ctx.lineWidth = 1;
  for (let g = 0; g <= 4; g++) {
    const x = Math.round(W * g / 4) + .5;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, base); ctx.stroke();
  }
  for (let i = 0; i < n; i++) {
    const v = RAMP.indexOf(bars[i]) / (RAMP.length - 1);
    const bh = Math.max(1, v * (base - 2));
    ctx.fillStyle = v > 0.72 ? '#f87171' : v > 0.45 ? '#fbbf24' : '#4ade80';
    ctx.fillRect(i * bw, base - bh, Math.max(1, bw - 0.5), bh);
  }
  ctx.fillStyle = '#8b9bb0'; ctx.font = '10px system-ui';
  for (let g = 0; g <= 4; g++) {
    const lbl = Math.round(f0 + (f1 - f0) * g / 4) + '';
    ctx.fillText(lbl, Math.min(W - 34, Math.max(2, W * g / 4 - lbl.length * 3)));
  }
}

function buildRef() {
  const q = el('search').value.trim().toLowerCase();
  const list = el('reflist');
  list.innerHTML = '';
  let group = null;
  REF.forEach(row => {
    const [g, mhz, what, w] = row;
    if (q && (g + ' ' + mhz + ' ' + what).toLowerCase().indexOf(q) < 0) return;
    if (g !== group) {
      group = g;
      const h = document.createElement('div');
      h.className = 'grp'; h.textContent = g;
      list.appendChild(h);
    }
    const r = reach(mhz);
    const d = document.createElement('div');
    d.className = 'refrow' + (selected.has(mhz) ? ' sel' : '') + (r.h === 0 ? ' out' : '');
    d.innerHTML = '<span class="f">' + mhz.toFixed(2) + '</span>' +
                  '<span class="t">' + what + '</span>' +
                  '<span class="w">' + (w >= 1 ? w + ' MHz' : '') + '</span>' +
                  '<span class="r ' + r.cls + '">' + (harmMode === 1 && r.h ? 'direct' : r.lbl) + '</span>';
    d.onclick = () => toggle(mhz, what, w);
    list.appendChild(d);
  });
}

function toggle(mhz, what, w) {
  if (selected.has(mhz)) selected.delete(mhz);
  else selected.set(mhz, { what, w });
  if (selected.size === 1) {
    /* Half width at the target = half the row's width, in kHz. */
    const want = Math.min(40000, Math.max(100, Math.round(w * 500 / 100) * 100));
    el('span').value = want; el('v_span').textContent = want;
  }
  buildRef(); buildChips();
}

function buildChips() {
  const c = el('chips');
  c.innerHTML = '';
  if (!selected.size) { el('plan').textContent = 'no target selected'; return; }
  const fr = [];
  selected.forEach((v, mhz) => {
    fr.push(mhz);
    const r = reach(mhz);
    const chip = document.createElement('span');
    chip.className = 'chip';
    chip.innerHTML = mhz.toFixed(3) + ' <em>' +
      (r.h === 0 ? 'out' : 'LO ' + (mhz / r.h).toFixed(3) + (r.h === 1 ? '' : ' ×' + r.h)) + '</em>';
    const x = document.createElement('button');
    x.textContent = '×';
    x.onclick = () => { selected.delete(mhz); buildRef(); buildChips(); };
    chip.appendChild(x);
    c.appendChild(chip);
  });
  fr.sort((a, b) => a - b);
  el('plan').textContent = fr.length + ' target(s): ' + fr.map(f => f.toFixed(3)).join(', ') +
    ' MHz  ·  span ±' + el('span').value + ' kHz each';
}

/* The whole plan as one command; targets left out keep their value. */
async function applyPlan() {
  const fr = [];
  if (selected.size) {
    selected.forEach((v, mhz) => { if (reach(mhz).h || harmMode === 1) fr.push(mhz); });
    if (!fr.length) { log('all selected targets are outside the LO window'); return false; }
    fr.sort((a, b) => a - b);
  }
  const cmd = 'jam plan ' + el('span').value + ' ' + currentMode + ' ' + el('amp').value +
              ' ' + el('dwell').value + ' ' + el('gain').value + ' ' + (harmMode || 'auto') +
              (fr.length ? ' ' + fr.map(f => f.toFixed(3)).join(' ') : '');
  return ensure(/^plan set|usage:|out of range|is not a frequency/, cmd);
}

const MODES = ['noise', 'chirp', 'cw', 'pulse'];

function buildModes() {
  MODES.forEach(m => {
    const b = document.createElement('button');
    b.textContent = m; b.dataset.m = m;
    b.onclick = () => { currentMode = m; markMode(); send('jam mode ' + m); };
    el('modes').appendChild(b);
  });
  markMode();
}
function markMode() {
  el('modes').querySelectorAll('button').forEach(b => b.classList.toggle('on', b.dataset.m === currentMode));
}

on('connect', () => connected ? disconnect() : connect());
on('test', () => send('jam test'));
on('stat', () => send('jam status'));
on('info', () => send('jam info'));
on('apply', async () => { await applyPlan(); });
on('clear', () => { selected.clear(); buildRef(); buildChips(); });
on('add', () => {
  const v = parseFloat(el('manual').value);
  if (isNaN(v) || v < 100 || v > 6000) { log('enter 100–6000 MHz'); return; }
  toggle(Math.round(v * 1000) / 1000, 'manual', 20);
  el('manual').value = '';
});
on('start', async () => {
  if (!(await applyPlan())) return;
  if (await ensure(/^jamming:|already jamming/, 'jam start')) {
    running = true; setStatus('jamming', 'ok');
  }
  await send('jam status');
});
on('stop', async () => {
  await ensure(/^stopped|^idle$/, 'jam stop');
  running = false;
  if (connected) setStatus('connected', 'ok');
  await send('jam status');
});
on('mon', () => {
  if (monTimer) {
    clearInterval(monTimer); monTimer = null;
    el('mon').textContent = 'Monitor'; el('mon').classList.remove('on');
    return;
  }
  el('mon').textContent = 'Stop monitor'; el('mon').classList.add('on');
  monTimer = setInterval(() => {
    if (!connected || monBusy) return;
    monBusy = true; send('jam mon 2048'); setTimeout(() => monBusy = false, 2200);
  }, 1500);
});
on('send', () => { const v = el('cmd').value.trim(); if (v) { send(v); el('cmd').value = ''; } });

el('search').oninput = buildRef;
el('manual').onkeydown = e => { if (e.key === 'Enter') el('add').click(); };
el('cmd').onkeydown = e => { if (e.key === 'Enter') el('send').click(); };

el('harm').querySelectorAll('button').forEach(b => b.onclick = () => {
  harmMode = +b.dataset.h;
  el('harm').querySelectorAll('button').forEach(x => x.classList.toggle('on', x === b));
  buildRef();
  send('jam harm ' + (harmMode || 'auto'));
});

function bindSlider(id, setter) {
  const s = el(id), out = el('v_' + id);
  const show = () => out.textContent = s.value;
  s.oninput = show;
  s.onchange = () => { show(); send(setter + ' ' + s.value); if (id === 'span') buildChips(); };
  show();
}
bindSlider('span', 'jam span');
bindSlider('dwell', 'jam dwell');
bindSlider('amp', 'jam amp');
bindSlider('gain', 'jam gain');

buildModes();
buildRef();
buildChips();
window.addEventListener('beforeunload', () => { if (connected) disconnect(); });
if (!('serial' in navigator)) setStatus('no WebSerial', 'bad');
</script>
</body>
</html>
JAM_HTML
  tail -n 1 "$HTML" | grep -q '</html>' || warn "$HTML looks truncated"
fi

# --- 5. optional multiapp registration -------------------------------------
if [ "$DO_MULTIAPP" = 1 ]; then
  python3 - "$REPO/apps/multiapp/CMakeLists.txt" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text()
if "multiapp_extension(jammer" in s:
    print("multiapp: jammer already registered")
else:
    a = "# Buffers of the apps that never run the receive ring"
    if a not in s: sys.exit("multiapp: anchor not found")
    p.write_text(s.replace(a, "# The jammer, from its own sources.\n"
        "multiapp_extension(jammer STACK 6144 SOURCES ${apps_dir}/jammer/src/main.c)\n\n" + a, 1))
    print("multiapp: registered jammer")
PY
fi

# --- 6. build --------------------------------------------------------------
do_build() {
  ( cd "$WS" && west build -d "$BUILD" -b "$BOARD" $1 esp-sdr-zephyr/apps/jammer ) 2>&1 \
    | tee "$BUILD_LOG"
}
relocate() {
  local alt="$HOME/$(basename "${WS%/}")"
  [ "$alt" = "$WS" ] && alt="$HOME/esp-sdr-ws-relocated"
  [ -e "$alt" ] && die "$alt already exists; move or remove it"
  log "relocating workspace to a path without spaces:"
  log "  $alt"
  mv "$WS" "$alt"
  WS="$alt"; REPO="$WS/esp-sdr-zephyr"; APP="$REPO/apps/jammer"
  BUILD="$WS/build-jammer"; BUILD_LOG="$WS/build-jammer.log"; HTML="$WS/jammer.html"
  [ -f "$WS/.west/config" ] && python3 - "$WS/.west/config" "$DEFAULT_WS" "$WS" <<'PY' || true
import sys, pathlib
cfg, old, new = pathlib.Path(sys.argv[1]), sys.argv[2], sys.argv[3]
t = cfg.read_text()
if old in t:
    cfg.write_text(t.replace(old, new))
PY
}
space_failure() {
  [[ "$WS" == *" "* ]] && grep -qE 'No such file or directory' "$BUILD_LOG" 2>/dev/null
}

if [ "$DO_BUILD" = 1 ]; then
  log "building for $BOARD (sysbuild=$SYSBUILD)"
  rm -rf "$BUILD"

  ok=0
  if [ "$SYSBUILD" = 1 ]; then
    if do_build "--sysbuild"; then ok=1
    elif grep -qiE 'partition|mcuboot|slot0|slot1|sysbuild' "$BUILD_LOG"; then
      warn "sysbuild/MCUboot problem -- rebuilding without it (no USB DFU; the shell is unaffected)"
      rm -rf "$BUILD"
      do_build "" && ok=1
    fi
  else
    do_build "" && ok=1
  fi

  if [ "$ok" != 1 ] && space_failure; then
    warn "devicetree preprocessor choked on a path with spaces"
    relocate
    rm -rf "$BUILD"
    if [ "$SYSBUILD" = 1 ]; then do_build "--sysbuild" && ok=1; else do_build "" && ok=1; fi
  fi

  [ "$ok" = 1 ] || die "build failed -- see $BUILD_LOG"
  log "build ok"
fi

[ "$DO_FLASH" = 1 ] && { log "flashing"; ( cd "$WS" && west flash -d "$BUILD" ); }
[ "$DO_DFU" = 1 ]   && { log "west dfu";  ( cd "$WS" && west dfu -d "$BUILD" ); }

cat <<JAM_EOF

done.  (rev $JAM_REV)

  workspace    : $WS
  control page : $HTML
  firmware     : $APP
  build log    : $BUILD_LOG

Flash:  cd "$WS" && west flash -d build-jammer --erase --esp-device /dev/cu.usbserial-XXXX

AFTER FLASHING, on the page:
  1. Connect
  2. "Test carrier (3 s)"  -> a 2441.1 MHz carrier for 3 s. If a line reads
     "xxx failed (-N)", that is the fault, and "Status / errors" repeats it.
  3. Pick a row ("whole ISM band", or one channel row for that channel), Start.
     The log shows "plan set: ..." and "jamming: ..." -- both are confirmed,
     and a missing reply is reported instead of failing silently.
  4. Stop prints how many dwells ran. Mode, span, amp, dwell and gain act
     while jamming.
  5. To hear the effect, watch a Wi-Fi analyser app on a second phone.

If Start is silent again: "jam status" shows dwells and the last error, and
"jam engine play" switches to the one-shot playback path (which sets the
front end up itself) to tell a front end problem from a waveform one.

JAM_EOF
# end of esp-sdr-jammer-autopatch.sh (rev 5)