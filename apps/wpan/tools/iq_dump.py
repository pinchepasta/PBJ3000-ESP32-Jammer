#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Fetch one `wpan dump` capture (4 MS/s, as the PHY sees it) into a cs16 file.

usage: iq_dump.py BOARD OUT.cs16 [COMMAND...]
BOARD is the USB serial number of the software radio board, or its port.
Commands given are sent before the dump (e.g. "wpan set chan 25").
"""

import base64
import glob
import re
import sys
import time

import serial


def port_of(board):
    """A board's shell port: a /dev path, or its USB serial number (as in /dev/serial/by-id)."""
    if board.startswith("/"):
        return board
    ports = glob.glob(f"/dev/serial/by-id/*_{board}-if00")
    if len(ports) != 1:
        raise SystemExit(f"{board}: {len(ports)} matching ports in /dev/serial/by-id")
    return ports[0]


def main():
    port, out = port_of(sys.argv[1]), sys.argv[2]
    s = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(0.3)
    s.read(65536)
    for c in sys.argv[3:]:
        s.write((c + "\r\n").encode())
        time.sleep(0.8)
        s.read(65536)
    s.write(b"wpan dump\r\n")
    buf, end = b"", time.time() + 30
    while time.time() < end and b"dump end" not in buf:
        buf += s.read(65536)
    text = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", buf.decode(errors="replace"))
    end_m = re.search(r"dump end (-?\d+)", text)
    if not end_m:
        raise SystemExit("no 'dump end' line within 30 s: nothing written")
    if int(end_m.group(1)) != 0:
        raise SystemExit(f"wpan dump failed ({end_m.group(1)}): nothing written")
    data = b"".join(base64.b64decode(m) for m in re.findall(r"b64 ([A-Za-z0-9+/=]+)", text))
    with open(out, "wb") as f:
        f.write(data)
    print(f"{len(data) // 4} samples to {out}")


if __name__ == "__main__":
    main()
