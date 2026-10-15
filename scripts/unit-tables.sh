#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Prints the sweep tables of the 802.15.4 unit tests: the PHY's packet error rate over SNR and
# carrier offset (phy) and the decoded fraction of the driver chain per decimator over SNR (dsp),
# each about half a minute on an idle machine plus the build. They are test cases that skip unless IEEE154_TABLES is set,
# which this script does, and they assert the region doc/ieee802154.md documents. The dsp run
# also prints the carrier offset estimates. See doc/testing.md.
#
# usage: unit-tables.sh [phy|dsp]    (default: both)
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)

case "${1:-all}" in
phy) tests=(ieee154_phy) ;;
dsp) tests=(ieee154_dsp) ;;
all) tests=(ieee154_phy ieee154_dsp) ;;
*)
	echo "usage: $0 [phy|dsp]" >&2
	exit 2
	;;
esac

args=()
for t in "${tests[@]}"; do
	args+=(-s "unit.esp_sdr.$t")
done

out=$root/twister-out/tables
log=$(mktemp)
status=0
IEEE154_TABLES=1 "$root/scripts/run-unit-tests.sh" --clobber-output -O "$out" "${args[@]}" \
	>"$log" 2>&1 || status=$?

for t in "${tests[@]}"; do
	f=$(find "$out" -path "*unit.esp_sdr.$t/handler.log" | head -n 1)
	if [ -n "$f" ]; then
		cat "$f"
	fi
done
if [ "$status" -ne 0 ]; then
	cat "$log" >&2
fi
rm -f "$log"
exit "$status"
