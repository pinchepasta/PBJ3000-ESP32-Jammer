#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compare two wpan boards: frames each way, ACKs, I/Q convention.

Both boards run apps/wpan, for example the native ESP32-C6 radio
(USB-Serial-JTAG console, opening it resets the chip) and the esp-sdr
software radio on an ESP32-S3 (CDC-ACM shell on the OTG port). The script
starts the software radio itself (wpan on); by hand it stays off until then.

usage: interop.py --native BOARD --sdr BOARD [--channels 11,15,20,25] [--frames 200]
                  [--payload 16] [--interval 10]

For each channel it reports, in both directions, how many test frames the
receiver counted (and lost, corrupt, duplicate), then ACK requests from the
software radio to the native one (which ACKs frames to its own address),
and, if a direction fails completely, retries it with the software radio's
receive or transmit I/Q conjugated (wpan set rxinvert / txinvert).
"""

import argparse
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

ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
STATS = re.compile(r"rx frames (\d+) test (\d+) bad (\d+) dups (\d+) lost (\d+)")
TXDONE = re.compile(r"tx done: sent (\d+) ok (\d+) noack (\d+) busy (\d+) err (\d+)")


class Board:
    def __init__(self, name, port, reset):
        self.name = name
        self.s = serial.Serial()
        self.s.port = port
        self.s.baudrate = 115200
        self.s.timeout = 0.05
        # USB-Serial-JTAG: RTS with DTR low resets; CDC-ACM ignores the lines.
        self.s.dtr = False
        self.s.rts = False
        self.s.open()
        if reset:
            self.s.rts = True
            time.sleep(0.1)
            self.s.rts = False
        self.buf = ""

    def read(self, seconds, until=None):
        end = time.time() + seconds
        while time.time() < end:
            data = self.s.read(4096)
            if data:
                self.buf += ANSI.sub("", data.decode(errors="replace"))
                if until is not None and re.search(until, self.buf):
                    break
        out, self.buf = self.buf, ""
        return out

    def cmd(self, line, wait=0.5, until=None):
        self.s.write((line + "\r\n").encode())
        return self.read(wait, until)


def stats(board):
    out = board.cmd("wpan status", 1.0, STATS.pattern)
    m = STATS.search(out)
    return tuple(int(x) for x in m.groups()) if m else None


def send(tx, rx, frames, payload, interval, dst="0xffff", ack=0):
    rx.cmd("wpan clear")
    timeout = frames * (interval / 1000.0 + 0.01) + 5
    out = tx.cmd(f"wpan tx {frames} {interval} {payload} {dst} {ack}", timeout, TXDONE.pattern)
    m = TXDONE.search(out)
    time.sleep(0.2)
    return (tuple(int(x) for x in m.groups()) if m else None), stats(rx)


def report(label, txr, rxs):
    print(f"  {label}: tx {txr}  rx frames/test/bad/dups/lost {rxs}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--native", required=True,
                    help="native radio board: USB serial number or console port")
    ap.add_argument("--sdr", required=True,
                    help="esp-sdr board: USB serial number or shell port")
    # Channel 26 from the C6 does not decode yet (doc/ieee802154.md).
    ap.add_argument("--channels", default="11,15,20,25")
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--payload", type=int, default=16)
    ap.add_argument("--interval", type=int, default=10, help="ms between frames")
    a = ap.parse_args()

    nat = Board("native", port_of(a.native), reset=True)
    print(nat.read(4, r"wpan: .*: -?\d+"), end="")
    sdr = Board("sdr", port_of(a.sdr), reset=False)
    print(next((l for l in sdr.cmd("wpan status", 1.0).splitlines() if l.startswith("radio ")), ""))
    for b in (nat, sdr):
        b.cmd("wpan set print 0")
        b.cmd("wpan on", 1.0)
    # The native board answers ACKs as PAN 0xabcd, address 0x0002.
    nat.cmd("wpan set pan 0xabcd addr 0x0002")
    sdr.cmd("wpan set pan 0xabcd addr 0x0001")

    fails = 0
    for ch in [int(c) for c in a.channels.split(",")]:
        print(f"channel {ch}", flush=True)
        nat.cmd(f"wpan set chan {ch}")
        sdr.cmd(f"wpan set chan {ch}", 1.0)
        for inv in ((0, 0), (1, 0), (0, 1)):
            sdr.cmd(f"wpan set rxinvert {inv[0]} txinvert {inv[1]}")
            t1, r1 = send(nat, sdr, a.frames, a.payload, a.interval)
            report(f"native -> sdr  (invert {inv})", t1, r1)
            t2, r2 = send(sdr, nat, a.frames, a.payload, a.interval)
            report(f"sdr -> native  (invert {inv})", t2, r2)
            t3, _ = send(sdr, nat, min(a.frames, 50), a.payload, a.interval, "0x0002", 1)
            print(f"  sdr -> native with ACK request (invert {inv}): tx {t3}", flush=True)
            ok = r1 and r2 and r1[1] > 0 and r2[1] > 0
            if ok:
                break
        if not ok:
            fails += 1
        sdr.cmd("wpan set rxinvert 0 txinvert 0")
        print(sdr.cmd("wpan status", 1.0), end="")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
