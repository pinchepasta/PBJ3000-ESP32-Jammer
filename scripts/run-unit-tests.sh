#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Runs every unit test (tests/unit/): native_sim, no boards needed, a minute
# or two. See doc/testing.md for the other test tiers.
#
# NSI_OPT=-O2 works around a native_sim/glibc interaction: without an
# optimization level, glibc's _FORTIFY_SOURCE check becomes a warning Zephyr
# treats as an error, in native_sim's own runtime support code (not ours).
# Runs from the repository root: give relative paths (-O) from there.
set -eu
cd "$(dirname "$0")/.."
NSI_OPT=${NSI_OPT:--O2} exec west twister -p native_sim/native/64 -T tests/unit "$@"
