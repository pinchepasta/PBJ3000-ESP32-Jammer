#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Launches esdr_scope.py with GNU Radio's own Python (which has PyQt5,
# matplotlib and gnuradio.qtgui built in) and the Qt platform plugin path
# that gnuradio-companion's wrapper sets, borrowed from that wrapper so
# nothing here depends on where nixpkgs puts them this week. Run inside
# `nix-shell mp-lvgl-example/shell.nix`.
set -euo pipefail

gc=$(command -v gnuradio-companion) || { echo "gnuradio-companion not found: run this inside nix-shell" >&2; exit 1; }
wrapped="$(dirname "$(readlink -f "$gc")")/.gnuradio-companion-wrapped"
[ -f "$wrapped" ] || wrapped="$gc"

py=$(head -1 "$wrapped" | sed 's/^#! *//; s/ .*//')
# Several plugin dirs accumulate (xcb lives under qtbase's, not qtwayland's): join them all.
plugin_path=$(grep -oE "QT_PLUGIN_PATH='[^']*'" "$gc" 2>/dev/null | sed "s/QT_PLUGIN_PATH='//; s/'$//" | paste -sd: -)

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SOAPY_SDR_PLUGIN_PATH="${SOAPY_SDR_PLUGIN_PATH:-$here/../build-soapy}"
[ -n "$plugin_path" ] && export QT_PLUGIN_PATH="$plugin_path"

exec "$py" "$here/esdr_scope.py" "$@"
