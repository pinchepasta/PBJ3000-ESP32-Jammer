#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Runs the two-board regression tests (tests/regression/) on the boards of
# tests/regression/hardware-map.yaml: builds both images, puts them on the
# boards (scripts/twister-flash.py: west dfu for the S3, west flash for the C6)
# and runs the pytest scenarios. A few minutes. See doc/testing.md.
# Runs from the repository root: give relative paths (-O) from there.
set -eu
cd "$(dirname "$0")/.."
exec west twister -T tests/regression --device-testing \
	--hardware-map tests/regression/hardware-map.yaml \
	--flash-command "$PWD/scripts/twister-flash.py" "$@"
