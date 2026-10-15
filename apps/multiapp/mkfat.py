#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Write a FAT12/FAT16 image holding files in its root directory.

The multiapp volume (src/drive.c): deterministic (fixed volume ID and
timestamps), 4 KiB clusters with the data area aligned to the flash erase
page, long file names (".llext" is not 8.3). The image ends at its last used
cluster: FAT ignores what free clusters hold.
"""

import argparse
import os
import struct
import sys

SECTOR = 512
ROOT_ENTRIES = 128          # 8 sectors
DATE = ((2026 - 1980) << 9) | (1 << 5) | 1   # 2026-01-01
TIME = 0


def sfn_checksum(name11):
    s = 0
    for c in name11:
        s = (((s & 1) << 7) + (s >> 1) + c) & 0xFF
    return s


def short_name(name, taken):
    """8.3 name, and whether a long name entry is needed."""
    base, dot, ext = name.rpartition(".")
    if not dot:
        base, ext = name, ""
    ok = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789$%'-_@~`!(){}^#&"
    clean = lambda s: "".join(c if c in ok else "_" for c in s.upper())
    b, e = clean(base.replace(".", "")), clean(ext)
    fits = name == name.upper() and len(b) <= 8 and len(e) <= 3 and b == base and e == ext
    if fits and (b, e) not in taken:
        taken.add((b, e))
        return (b.ljust(8) + e.ljust(3)).encode("ascii"), False
    for n in range(1, 10):
        tail = f"~{n}"
        cand = (b[: 8 - len(tail)] + tail, e[:3])
        if cand not in taken:
            taken.add(cand)
            return (cand[0].ljust(8) + cand[1].ljust(3)).encode("ascii"), True
    sys.exit(f"mkfat.py: too many names like {name}")


def lfn_entries(name, csum):
    """Long name entries, last part first, as they precede the 8.3 entry."""
    u16 = list(struct.unpack(f"<{len(name.encode('utf-16-le')) // 2}H",
                             name.encode("utf-16-le"))) + [0x0000]
    if len(u16) % 13 == 1 and len(u16) > 13:
        u16 = u16[:-1]          # a name filling the last entry exactly has no terminator
    u16 += [0xFFFF] * (-len(u16) % 13)
    n = len(u16) // 13
    out = []
    for i in range(n, 0, -1):
        part = u16[(i - 1) * 13: i * 13]
        out.append(struct.pack("<B5HBBB6HH2H", (i | 0x40) if i == n else i, *part[0:5], 0x0F, 0,
                               csum, *part[5:11], 0, *part[11:13]))
    return out


def dir_entry(name11, attr, cluster, size):
    return struct.pack("<11sBBBHHHHHHHI", name11, attr, 0, 0, TIME, DATE, DATE, 0, TIME, DATE,
                       cluster, size)


def build(size, files, label, volid, cluster_bytes):
    spc = cluster_bytes // SECTOR
    total = size // SECTOR
    root_secs = ROOT_ENTRIES * 32 // SECTOR
    # FAT type and size: iterate, the FAT size depends on the cluster count.
    fat_secs = 1
    for _ in range(8):
        rsvd = 1
        while (rsvd + 2 * fat_secs + root_secs) % spc:
            rsvd += 1
        data_secs = total - rsvd - 2 * fat_secs - root_secs
        clusters = data_secs // spc
        fat12 = clusters < 4085
        need = ((clusters + 2) * 3 + 1) // 2 if fat12 else (clusters + 2) * 2
        need_secs = -(-need // SECTOR)
        if need_secs == fat_secs:
            break
        fat_secs = need_secs
    if not fat12 and clusters >= 65525:
        sys.exit("mkfat.py: volume too large for FAT16 with these clusters")
    img = bytearray(size)
    bs = struct.pack("<3s8sHBHBHHBHHHII", b"\xEB\x3C\x90", b"MULTIAPP", SECTOR, spc, rsvd, 2,
                     ROOT_ENTRIES, total if total < 0x10000 else 0, 0xF8, fat_secs, 32, 2, 0,
                     0 if total < 0x10000 else total)
    bs += struct.pack("<BBBI11s8s", 0x80, 0, 0x29, volid, label.upper().ljust(11)[:11].encode(),
                      b"FAT12   " if fat12 else b"FAT16   ")
    img[0:len(bs)] = bs
    img[510:512] = b"\x55\xAA"

    fat = [0] * (clusters + 2)
    fat[0], fat[1] = (0xFF8, 0xFFF) if fat12 else (0xFFF8, 0xFFFF)
    eoc = 0xFFF if fat12 else 0xFFFF
    root = [dir_entry(label.upper().ljust(11)[:11].encode(), 0x08, 0, 0)]
    taken = set()
    nxt = 2
    data0 = (rsvd + 2 * fat_secs + root_secs) * SECTOR
    for path in files:
        name = os.path.basename(path)
        data = open(path, "rb").read()
        n = -(-len(data) // cluster_bytes)
        if nxt + n > clusters + 2:
            sys.exit(f"mkfat.py: {name} does not fit")
        first = nxt if n else 0
        for c in range(nxt, nxt + n):
            fat[c] = c + 1 if c < nxt + n - 1 else eoc
        off = data0 + (nxt - 2) * cluster_bytes
        img[off:off + len(data)] = data
        nxt += n
        sfn, needs_lfn = short_name(name, taken)
        if needs_lfn:
            root += lfn_entries(name, sfn_checksum(sfn))
        root.append(dir_entry(sfn, 0x20, first, len(data)))
    if len(root) > ROOT_ENTRIES:
        sys.exit("mkfat.py: root directory full")
    if fat12:
        raw = bytearray()
        for i in range(0, len(fat), 2):
            a, b = fat[i], fat[i + 1] if i + 1 < len(fat) else 0
            raw += bytes((a & 0xFF, ((a >> 8) & 0x0F) | ((b & 0x0F) << 4), (b >> 4) & 0xFF))
    else:
        raw = b"".join(struct.pack("<H", v) for v in fat)
    for k in range(2):
        off = (rsvd + k * fat_secs) * SECTOR
        img[off:off + len(raw)] = raw
    off = (rsvd + 2 * fat_secs) * SECTOR
    rootb = b"".join(root)
    img[off:off + len(rootb)] = rootb
    used = data0 + (nxt - 2) * cluster_bytes
    return img, ("FAT12" if fat12 else "FAT16"), clusters, data0, used


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--size", type=lambda s: int(s, 0), required=True, help="image size, bytes")
    p.add_argument("--cluster", type=int, default=4096)
    p.add_argument("--label", default="MULTIAPP")
    p.add_argument("--volid", type=lambda s: int(s, 0), default=0x4D415050)
    p.add_argument("--out", required=True)
    p.add_argument("files", nargs="*")
    a = p.parse_args()
    img, kind, clusters, data0, used = build(a.size, a.files, a.label, a.volid, a.cluster)
    with open(a.out, "wb") as f:
        f.write(img[:used])
    print(f"{a.out}: {kind}, {clusters} clusters of {a.cluster} B, data at {data0:#x}, "
          f"{len(a.files)} files, {used} bytes used")


if __name__ == "__main__":
    main()
