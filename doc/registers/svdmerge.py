#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Merge doc/registers/<chip>.svd into Espressif's SVD: --out DIR gets <chip>-esp-sdr.svd.

Espressif's SVD comes from github.com/espressif/svd at a pinned commit (checked
by SHA-256), or from --official DIR. Our peripherals are added; in Espressif's,
our registers are added and our field descriptions replace theirs (a field with
other bits, or a register or peripheral overlapping theirs, is an error).
Analog I2C blocks (group ANALOG_I2C) are left out: they are not memory-mapped.
"""

import argparse
import copy
import hashlib
import sys
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

COMMIT = "be20aa12560889d6125d144cdb48cf615ac17628"
SHA256 = {
    "esp32s3": "d7629249eba2ad660a960530d124966e9274c8228341380dd4102f251e879e17",
    "esp32c6": "519b374c472483b170d6e5b479e48c07675c18af5bf2613c61eff888dc0fc8cd",
}


def num(text):
    return int(text.strip(), 0)


def official(chip, cache):
    path = cache / f"{chip}.svd"
    if not path.exists():
        url = f"https://raw.githubusercontent.com/espressif/svd/{COMMIT}/svd/{chip}.svd"
        cache.mkdir(parents=True, exist_ok=True)
        path.write_bytes(urllib.request.urlopen(url, timeout=60).read())
    if hashlib.sha256(path.read_bytes()).hexdigest() != SHA256[chip]:
        sys.exit(f"{path}: not Espressif's {chip}.svd at {COMMIT[:7]}")
    return ET.parse(path)


def bits(f):
    if f.find("bitRange") is not None:
        msb, lsb = map(int, f.findtext("bitRange").strip("[]").split(":"))
    elif f.find("lsb") is not None:
        msb, lsb = num(f.findtext("msb")), num(f.findtext("lsb"))
    else:
        lsb = num(f.findtext("bitOffset"))
        msb = lsb + num(f.findtext("bitWidth")) - 1
    return msb, lsb


def span(p):
    base = num(p.findtext("baseAddress"))
    blocks = p.findall("addressBlock")
    if not blocks:
        return base, base
    lo = min(base + num(b.findtext("offset")) for b in blocks)
    hi = max(base + num(b.findtext("offset")) + num(b.findtext("size")) - 1 for b in blocks)
    return lo, hi


def merge_register(chip, theirs, ours):
    fields = theirs.find("fields")
    if fields is None:
        fields = ET.SubElement(theirs, "fields")
    for f in ours.iterfind("fields/field"):
        name, (msb, lsb) = f.findtext("name"), bits(f)
        same = next((g for g in fields if g.findtext("name") == name), None)
        if same is not None:
            if bits(same) != (msb, lsb):
                sys.exit(f"{chip}: {theirs.findtext('name')}.{name}: bits differ from Espressif's")
            desc = same.find("description")
            if desc is None:
                desc = ET.Element("description")
                same.insert(list(same).index(same.find("name")) + 1, desc)
            desc.text = f.findtext("description")
            continue
        for g in fields:
            gm, gl = bits(g)
            if lsb <= gm and gl <= msb:
                sys.exit(f"{chip}: {theirs.findtext('name')}.{name} overlaps {g.findtext('name')}")
        fields.append(copy.deepcopy(f))


def merge(chip, base, ours):
    periphs = base.getroot().find("peripherals")
    for p in ours.getroot().iterfind("peripherals/peripheral"):
        if p.findtext("groupName") == "ANALOG_I2C":
            continue
        name = p.findtext("name")
        same = next((q for q in periphs if q.findtext("name") == name), None)
        if same is None:
            lo, hi = span(p)
            for q in periphs:
                qlo, qhi = span(q)
                if lo <= qhi and qlo <= hi:
                    sys.exit(f"{chip}: {name} overlaps Espressif's {q.findtext('name')}")
            periphs.append(copy.deepcopy(p))
            continue
        regs = same.find("registers")
        for r in p.iterfind("registers/register"):
            rname, offset = r.findtext("name"), num(r.findtext("addressOffset"))
            theirs = next((x for x in regs if x.findtext("name") == rname), None)
            if theirs is not None:
                merge_register(chip, theirs, r)
            elif any(num(x.findtext("addressOffset")) == offset for x in regs):
                sys.exit(f"{chip}: {name}.{rname} at an offset Espressif's SVD uses")
            else:
                regs.append(copy.deepcopy(r))
    root = base.getroot()
    root.find("version").text += "+esp-sdr"
    root.find("description").text += (
        " With the undocumented radio registers of esp-sdr-zephyr (doc/registers).")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--official", type=Path, help="directory with Espressif's <chip>.svd")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    for svd in sorted(Path(__file__).parent.glob("*.svd")):
        base = official(svd.stem, args.official or args.out / "espressif")
        merge(svd.stem, base, ET.parse(svd))
        ET.indent(base)
        base.write(args.out / f"{svd.stem}-esp-sdr.svd", encoding="utf-8", xml_declaration=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
