#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""iperf style test between two link boards, over their USB shells.

    link_perf.py run --server <serial> --client <serial> [-t 10] [-m 16qam]
                     [-f rs|hamming|none] [-n <units>] [-b <kbit/s>]
                     [--set key=value ...]
    link_perf.py dump <serial> <file.bin> [--timeout-ms 2000]
    link_perf.py ofdm --rx <serial> --tx <serial> [-n 100] [-v]
                      [--set key=value ...]

"run" applies the --set options to both boards ("link set <key> <value>"),
starts "link server" on one and "link client <addr>" on the other, and
prints both reports as they come. "dump" saves the next capture above the
carrier sense threshold as raw little endian receive words, for
tools/qam_sim.c and tools/ofdm_sim.c. "ofdm" applies the --set options to
both boards ("ofdm set" for the frame keys, "link set" for the radio ones,
such as txgain=4 rxgain=32), loops the OFDM test frame on one and decodes
captures of it on the other.
"""

import argparse
import glob
import re
import struct
import sys
import threading
import time

import serial

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
OFDM_KEYS = ("bw", "ch", "cp", "mod", "amp", "syms", "rxdiv", "fs", "pilots", "smooth")


class Board:
    def __init__(self, sn):
        ports = glob.glob(f"/dev/serial/by-id/usb-Zephyr_Project_ESP-SDR*_{sn}-if00")
        if not ports:
            sys.exit(f"{sn}: no shell port")
        self.sn = sn
        self.s = serial.Serial(ports[0], 115200, timeout=0.1)
        time.sleep(0.2)
        self.s.read(1 << 20)

    def send(self, cmd):
        self.s.write(cmd.encode() + b"\r\n")

    def lines(self, seconds, until=None):
        """Yield shell lines for up to seconds, stop early at a line matching until."""
        buf, end = "", time.time() + seconds
        while time.time() < end:
            buf += ANSI.sub("", self.s.read(4096).decode(errors="replace")).replace("\r", "")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.replace("uart:~$ ", "").strip()
                if not line:
                    continue
                yield line
                if until and re.search(until, line):
                    return

    def cmd(self, cmd, seconds=1.0, until=None):
        self.send(cmd)
        return list(self.lines(seconds, until))

    def addr(self):
        for line in self.cmd("link status", 1.5, r"^tx:"):
            m = re.search(r"addr 0x([0-9a-f]+)", line)
            if m:
                return int(m.group(1), 16)
        sys.exit(f"{self.sn}: no link status")


def run(args):
    srv, cli = Board(args.server), Board(args.client)
    for kv in args.set:
        key, _, val = kv.partition("=")
        for b in (srv, cli):
            for line in b.cmd(f"link set {key} {val}", 0.5):
                if "rejected" in line or "unknown" in line or "bad" in line:
                    sys.exit(f"{b.sn}: {line}")
    for b in (srv, cli):
        b.cmd("link clear", 0.3)
    addr = srv.addr()
    copt = f" -t {args.t} -m {args.m} -f {args.f} -w {args.w}" + (f" -n {args.n}" if args.n else "") + \
        (f" -b {args.b}" if args.b else "")
    srv.cmd(f"link server -i {args.i}", 0.5)
    cli.send(f"link client 0x{addr:02x}{copt} -i {args.i}")

    def pump(board, tag, seconds, until):
        for line in board.lines(seconds, until):
            if line.startswith("[") or "client to" in line or "error" in line.lower():
                print(f"{tag} {line}", flush=True)

    t = threading.Thread(target=pump, args=(srv, "B", args.t + 3, None), daemon=True)
    t.start()
    pump(cli, "A", args.t + 5, r"\[client\] total")
    t.join()
    for line in srv.cmd("link stop", 1.0, r"total"):
        if line.startswith("["):
            print(f"B {line}")
    for b, tag in ((cli, "A"), (srv, "B")):
        for line in b.cmd("link status", 1.5, r"^decode stages"):
            if line.startswith(("tx:", "rx:", "timing:", "decode")):
                print(f"{tag} {line}")


def dump(args):
    b = Board(args.board)
    b.send(f"link dump {'ok ' if args.ok else ''}{args.timeout_ms}")
    words, started = [], False
    for line in b.lines(args.timeout_ms / 1000 + 30, r"dump end|nothing above"):
        if line.startswith("dump end"):
            break
        if "nothing above" in line:
            sys.exit(line)
        if line.startswith("dump "):
            started = True
            continue
        if started:
            words += [int(w, 16) for w in line.split()]
    with open(args.file, "wb") as f:
        f.write(struct.pack(f"<{len(words)}I", *words))
    print(f"{len(words)} words to {args.file}")


def ofdm(args):
    rx, tx = Board(args.rx), Board(args.tx)
    frame = [kv.partition("=") for kv in args.set if kv.partition("=")[0] in OFDM_KEYS]
    radio = [kv.partition("=") for kv in args.set if kv.partition("=")[0] not in OFDM_KEYS]
    for b in (rx, tx):
        for key, _, val in radio:
            for line in b.cmd(f"link set {key} {val}", 0.5):
                if "rejected" in line or "keys:" in line:
                    sys.exit(f"{b.sn}: {line}")
        # The frame keys in one command: the layout is checked on the whole set.
        if frame:
            for line in b.cmd("ofdm set " + " ".join(f"{k} {v}" for k, _, v in frame), 0.5):
                if "no frame layout" in line or "keys:" in line:
                    sys.exit(f"{b.sn}: {line}")
    for line in rx.cmd("ofdm status", 0.5):
        if line.startswith(("bpsk", "qpsk", "16qam", "64qam", "frame")):
            print(line)
    # Long enough for the captures and their decoding (a few ms each), with margin.
    seconds = 2.0 + args.n * 0.02
    tx.send(f"ofdm tx -t {seconds + 2.0:.1f}")
    for line in tx.lines(1.0, r"looping|failed"):
        if "failed" in line:
            sys.exit(f"{tx.sn}: {line}")
    for line in rx.cmd(f"ofdm rx -n {args.n}{' -v' if args.v else ''}", seconds + 10.0,
                       r"^rx done"):
        if not line.startswith(("ofdm rx", "rx done")):
            print(line)
    for _ in tx.lines(seconds + 3.0):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--server", required=True, help="USB serial of the receiving board")
    r.add_argument("--client", required=True, help="USB serial of the sending board")
    r.add_argument("-t", type=float, default=10.0, help="seconds")
    r.add_argument("-m", default="16qam", choices=["qpsk", "16qam", "64qam", "256qam", "1024qam", "4096qam"])
    r.add_argument("-f", default="rs", choices=["rs", "hamming", "none"], help="error correction")
    r.add_argument("-n", type=int, default=0, help="FEC units per frame (0: full frames)")
    r.add_argument("-w", type=int, default=2, choices=[1, 2], help="frames per transmission")
    r.add_argument("-b", type=int, default=0, help="offered kbit/s (0: as fast as possible)")
    r.add_argument("-i", type=float, default=1.0, help="report interval in s")
    r.add_argument("--set", action="append", default=[], help="key=value for both boards")
    d = sub.add_parser("dump")
    d.add_argument("board")
    d.add_argument("file")
    d.add_argument("--timeout-ms", type=int, default=2000)
    d.add_argument("--ok", action="store_true", help="only a capture that decoded")
    o = sub.add_parser("ofdm")
    o.add_argument("--rx", required=True, help="USB serial of the receiving board")
    o.add_argument("--tx", required=True, help="USB serial of the sending board")
    o.add_argument("-n", type=int, default=100, help="captures to decode")
    o.add_argument("-v", action="store_true", help="also the channel per subcarrier")
    o.add_argument("--set", action="append", default=[], help="key=value for both boards")
    args = ap.parse_args()
    {"run": run, "dump": dump, "ofdm": ofdm}[args.cmd](args)


if __name__ == "__main__":
    main()
