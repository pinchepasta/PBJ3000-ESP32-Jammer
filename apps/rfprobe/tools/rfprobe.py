#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Host side of apps/rfprobe: tone sweeps, PLL scans, register dumps, noise floor.

  sweep      the S3 loops a tone, another board captures it: at pairs of LO frequencies
             the tone must land at (TX LO - RX LO) + offset
  raw        the same with both PLLs programmed directly, bypassing the frequency plan
  pll        capacitor code and VCO window comparator (block 0x62) per PLL frequency
  dump       analog I2C blocks after tuning, to a JSON file
  diff       two dump files, register by register
  floor      receive level with nothing transmitted
  summarize  a sweep's JSON lines: per LO the three pairs, and the verified runs

Needs pyserial and numpy (both in the nix shell). Ports are /dev paths. A
USB-Serial-JTAG port (the C6) is reset cleanly on open: DTR and RTS low, then a
pulse on RTS alone (DTR high during a reset would select the ROM bootloader). A
CDC-ACM port (the S3) is opened without a reset. See ../README.rst.
"""

import argparse
import base64
import json
import os
import re
import sys
import time

import numpy as np
import serial
from serial.tools import list_ports

ANSI = re.compile(rb"\x1b\[[0-9;]*[A-Za-z]")
USJ = (0x303A, 0x1001)
DIVIDER_BELOW_MHZ = 2210
TONE_HZ = 3_000_000
# A tone arrives when it stands this far above the local floor, within MATCH_HZ
# of where the LOs put it (crystal tolerance of two boards plus an FFT bin).
ARRIVES_DB = 20.0
MATCH_HZ = 150e3
DC_SKIP_HZ = 150e3
FLOOR_HALF_HZ = 1e6


class Shell:
    """One board's `probe` shell."""

    def __init__(self, path, boot_s=7.0):
        real = os.path.realpath(path)
        usj = any(p.device == real and (p.vid, p.pid) == USJ for p in list_ports.comports())
        self.s = serial.Serial()
        self.s.port, self.s.baudrate, self.s.timeout = path, 115200, 0.05
        self.s.dtr = False
        self.s.rts = False
        self.s.open()
        if usj:
            self.s.rts = True
            time.sleep(0.1)
            self.s.rts = False
        end = time.time() + (boot_s if usj else 0.5)
        while time.time() < end:
            self.s.read(4096)
        self.s.write(b"\x03")
        time.sleep(0.2)
        self.s.read(65536)

    def cmd(self, line, until, timeout=5.0):
        self.s.reset_input_buffer()
        self.s.write(line.encode() + b"\r")
        buf = b""
        pat = re.compile(until.encode(), re.M)
        end = time.time() + timeout
        while time.time() < end:
            buf += self.s.read(65536)
            clean = ANSI.sub(b"", buf)
            if pat.search(clean):
                return clean.decode(errors="replace")
        raise TimeoutError(f"{line!r}: no {until!r} in {ANSI.sub(b'', buf)[-300:]!r}")

    def tune(self, mhz):
        out = self.cmd(f"probe f {mhz}", r"^f -?\d+ ret -?\d+")
        return int(re.search(r"^f -?\d+ ret (-?\d+)", out, re.M).group(1))

    def raw(self, mhz, khz=0):
        self.cmd(f"probe raw {mhz} {khz}", r"^raw ")

    def capture(self, n):
        out = self.cmd(f"probe rx {n}", r"^(end|rx ret)", timeout=20)
        m = re.search(r"rx n (\d+) f (\d+) fs (\d+)", out)
        if not m:
            raise RuntimeError(out[-300:])
        data = b"".join(base64.b64decode(l[2:].strip()) for l in out.splitlines()
                        if l.startswith("d "))
        w = np.frombuffer(data, dtype="<u4")
        i = ((w >> 10) & 0x3FF).astype(np.int32)
        q = (w & 0x3FF).astype(np.int32)
        i[i >= 512] -= 1024
        q[q >= 512] -= 1024
        return i + 1j * q, int(m.group(3))

    def dump(self, block, regs):
        out = self.cmd(f"probe dump {block} {regs}", r"^dump ")
        return [int(v, 16) for v in re.search(r"dump \w+:(.*)", out).group(1).split()]


def analyse(x, fs, expect_hz):
    """Strongest tone outside DC and the level at expect_hz, each against the median of
    the bins within +-1 MHz (the receiver's floor is not flat)."""
    x = x - x.mean()
    n = len(x)
    spec = np.fft.fftshift(np.abs(np.fft.fft(x * np.hanning(n))) ** 2)
    f = np.fft.fftshift(np.fft.fftfreq(n, 1 / fs))
    half = max(1, int(FLOOR_HALF_HZ / (fs / n)))
    pad = np.pad(spec, half, mode="reflect")
    centers = np.arange(0, n, max(1, half // 8))
    floor = np.interp(np.arange(n), centers,
                      [np.median(pad[c:c + 2 * half + 1]) for c in centers])
    db = 10 * np.log10(spec / floor + 1e-12)
    k = np.argmax(np.where(np.abs(f) > DC_SKIP_HZ, db, -99))
    near = np.abs(f - expect_hz) < MATCH_HZ
    ke = np.argmax(np.where(near, db, -99))
    clip = int(np.sum((np.abs(x.real) >= 510) | (np.abs(x.imag) >= 510)))
    # An expected offset outside the capture has no level (NaN, which fails).
    return {"peak_hz": float(f[k]), "peak_db": float(db[k]),
            "exp_db": float(db[ke]) if near.any() else float("nan"),
            "exp_err_hz": float(f[ke] - expect_hz) if near.any() else float("nan"),
            "rms": float(np.sqrt(np.mean(np.abs(x) ** 2))), "clip": clip}


def arrives(r):
    return r["exp_db"] > ARRIVES_DB and abs(r["peak_hz"] - r["expect_hz"]) < MATCH_HZ


def tone_capture(tx, rx, expect_hz, n, ofs, amp):
    tx.cmd(f"probe tone on {ofs} {amp}", r"^tone on ")
    try:
        x, fs = rx.capture(n)
    finally:
        tx.cmd("probe tone off", r"^tone off")
    return analyse(x, fs, expect_hz)


def mhz_list(spec):
    """'2170:2200:2,2412' -> [2170, 2172, ..., 2200, 2412] (ranges inclusive)."""
    out = []
    for part in spec.split(","):
        p = [int(v, 0) for v in part.split(":")]
        if len(p) == 1:
            out.append(p[0])
        else:
            lo, hi, step = p[0], p[1], p[2] if len(p) > 2 else 1
            step = abs(step) if hi >= lo else -abs(step)
            out += list(range(lo, hi + (1 if step > 0 else -1), step))
    return out


def pair_list(args):
    pairs = []
    if args.pairs:
        pairs += [tuple(int(v) for v in p.split(":")) for p in args.pairs.split(",")]
    if args.range:
        lo, hi, st = (int(v) for v in args.range.split(":"))
        for f in range(lo, hi + 1, st):
            pairs.append((f, f))
            if f + st <= hi:
                pairs += [(f, f + st), (f + st, f)]
    return pairs


def setup_pair(args):
    rx = Shell(args.rx)
    tx = Shell(args.tx)
    for sh in (rx, tx):
        sh.cmd("probe lpf 0 0", r"^lpf rx")
    rx.cmd(f"probe gain {args.rxgain}", r"^rxgain")
    tx.cmd(f"probe gain -1 {args.txgain}", r"^rxgain")
    return tx, rx


def emit(out, r, line):
    if out:
        out.write(json.dumps(r) + "\n")
        out.flush()
    print(line, flush=True)


def describe(r):
    level = "outside the capture" if np.isnan(r["exp_db"]) else f"{r['exp_db']:5.1f} dB"
    return (f"expect {r['expect_hz'] / 1e6:+8.3f} MHz {level}, strongest "
            f"{r['peak_hz'] / 1e6:+8.3f} MHz {r['peak_db']:5.1f} dB, rms {r['rms']:6.1f} "
            f"clip {r['clip']} {'PASS' if r['pass'] else 'FAIL'}")


def cmd_sweep(args):
    tx, rx = setup_pair(args)
    out = open(args.out, "a") if args.out else None
    for a, b in pair_list(args):
        try:
            ra, rb = tx.tune(a), rx.tune(b)
            r = tone_capture(tx, rx, (a - b) * 1e6 + args.ofs, args.n, args.ofs, args.amp)
            r.update(a=a, b=b, a_ret=ra, b_ret=rb, expect_hz=(a - b) * 1e6 + args.ofs)
            r["pass"] = arrives(r)
            emit(out, r, f"tx {a:5d} ({ra:3d}) rx {b:5d} ({rb:3d}) " + describe(r))
        except Exception as e:  # keep sweeping
            emit(out, {"a": a, "b": b, "error": str(e), "pass": False},
                 f"tx {a:5d} rx {b:5d} ERROR {str(e)[:120]}")


def cmd_raw(args):
    tx, rx = setup_pair(args)
    div = 1.2 if args.base < DIVIDER_BELOW_MHZ else 1.0
    for sh in (tx, rx):
        sh.tune(args.base)
    out = open(args.out, "a") if args.out else None
    hi, lo, step = (int(v) for v in args.range.split(":"))
    for p in range(hi, lo - 1, -step):
        for a, b in ((p, p), (p, p - step), (p - step, p)):
            if min(a, b) < lo:
                continue
            try:
                rx.raw(b)
                tx.raw(a)
                expect = (a - b) * 1e6 / div + args.ofs
                r = tone_capture(tx, rx, expect, args.n, args.ofs, args.amp)
                r.update(a=a, b=b, base=args.base, div=div, expect_hz=expect)
                r["pass"] = arrives(r)
                emit(out, r, f"pll tx {a} rx {b} (RF {a / div:7.1f}/{b / div:7.1f}) "
                     + describe(r))
            except Exception as e:
                emit(out, {"a": a, "b": b, "error": str(e), "pass": False},
                     f"pll tx {a} rx {b} ERROR {str(e)[:120]}")


def cmd_pll(args):
    sh = Shell(args.port)
    sh.tune(2412)  # divider off: the PLL frequency is the one asked for
    print("PLL MHz  code  reg12  bits3:2  (0 in the window, 1 below, 2 above)")
    for mhz in mhz_list(args.freqs):
        sh.raw(mhz)
        r = sh.dump(0x62, 13)
        code = r[5] | ((r[7] >> 2) & 1) << 8
        print(f"{mhz:7d}  {code:4d}  0x{r[12]:02x}   {(r[12] >> 2) & 3}", flush=True)


def cmd_dump(args):
    sh = Shell(args.port)
    sh.tune(args.freq)
    lo, hi = (int(v, 0) for v in args.blocks.split(":"))
    res = {f"0x{b:02x}": sh.dump(b, args.regs) for b in range(lo, hi + 1)}
    with open(args.out, "w") as f:
        json.dump({"freq": args.freq, "port": args.port, "blocks": res}, f, indent=1)
    print(f"{args.out}: blocks 0x{lo:02x}..0x{hi:02x}, {args.regs} registers each, at "
          f"{args.freq} MHz")


def cmd_diff(args):
    a, b = (json.load(open(p)) for p in (args.a, args.b))
    same = True
    for blk in sorted(set(a["blocks"]) & set(b["blocks"])):
        ra, rb = a["blocks"][blk], b["blocks"][blk]
        d = [f"r{i}: {x:02x} -> {y:02x}" for i, (x, y) in enumerate(zip(ra, rb)) if x != y]
        if d:
            same = False
            print(f"block {blk}: " + ", ".join(d))
    if same:
        print("no differences")


def cmd_floor(args):
    sh = Shell(args.port)
    sh.cmd(f"probe gain {args.gain}", r"^rxgain")
    for mhz in mhz_list(args.freqs):
        ret = sh.tune(mhz)
        rms = [analyse(*sh.capture(args.n), 0.0)["rms"] for _ in range(args.repeat)]
        print(f"{mhz:5d} MHz (ret {ret:3d}): rms " + " ".join(f"{v:5.1f}" for v in rms),
              flush=True)


def cmd_summarize(args):
    rows = [json.loads(l) for l in open(args.file)]
    res = {(r["a"], r["b"]): r for r in rows}
    fs = sorted({r["a"] for r in rows} | {r["b"] for r in rows})
    step = min(b - a for a, b in zip(fs, fs[1:])) if len(fs) > 1 else 1

    def mark(r):
        if r is None:
            return "   . "
        if "error" in r:
            return "  ERR"
        return f"{r['exp_db']:4.0f}{'+' if r['pass'] else 'x'}"

    print("  MHz  same    up  down   (dB above the local floor at the expected offset, + pass)")
    good = set()
    for f in fs:
        s, u, d = res.get((f, f)), res.get((f, f + step)), res.get((f + step, f))
        print(f"{f:5d} {mark(s)} {mark(u)} {mark(d)}")
        if u and d and u.get("pass") and d.get("pass"):
            good |= {f, f + step}
    run = []
    for f in fs + [None]:
        if f in good and (not run or f - run[-1] == step):
            run.append(f)
            continue
        if run:
            print(f"verified through cross pairs: {run[0]} to {run[-1]} MHz")
        run = [f] if f in good else []


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    def pair_args(p):
        p.add_argument("--tx", required=True, help="the S3's port (it loops the tone)")
        p.add_argument("--rx", required=True, help="the receiving board's port")
        p.add_argument("--ofs", type=float, default=TONE_HZ, help="tone offset in Hz")
        p.add_argument("--amp", type=int, default=400, help="tone amplitude, 1..511")
        p.add_argument("--n", type=int, default=8192, help="samples per capture")
        p.add_argument("--rxgain", type=int, default=40, help="receive gain index, -1 AGC")
        p.add_argument("--txgain", type=int, default=0, help="transmit step, 0 weakest")
        p.add_argument("--out", help="append JSON lines here")

    p = sub.add_parser("sweep", help="tones at pairs of LO frequencies")
    pair_args(p)
    p.add_argument("--pairs", help="TX:RX,TX:RX,... in MHz")
    p.add_argument("--range", help="LO:HI:STEP: each LO, and the cross pairs to the next")
    p.set_defaults(func=cmd_sweep)

    p = sub.add_parser("raw", help="tones with both PLLs programmed directly")
    pair_args(p)
    p.add_argument("--base", type=int, required=True,
                   help="LO tuned first, for the divider: below 2210 MHz on, PLL = LO x 1.2")
    p.add_argument("--range", required=True, help="HI:LO:STEP in PLL MHz, downwards")
    p.set_defaults(func=cmd_raw)

    p = sub.add_parser("pll", help="capacitor code and VCO window per PLL frequency")
    p.add_argument("--port", required=True)
    p.add_argument("--freqs", required=True, help="PLL MHz: '2170:2200:2,2412'")
    p.set_defaults(func=cmd_pll)

    p = sub.add_parser("dump", help="analog I2C blocks to a JSON file")
    p.add_argument("--port", required=True)
    p.add_argument("--freq", type=int, default=2412, help="LO tuned first (MHz)")
    p.add_argument("--blocks", default="0x60:0x6f", help="first:last block")
    p.add_argument("--regs", type=int, default=32, help="registers per block (max 64)")
    p.add_argument("--out", required=True)
    p.set_defaults(func=cmd_dump)

    p = sub.add_parser("diff", help="two dump files")
    p.add_argument("a")
    p.add_argument("b")
    p.set_defaults(func=cmd_diff)

    p = sub.add_parser("floor", help="receive level with nothing transmitted")
    p.add_argument("--port", required=True)
    p.add_argument("--freqs", default="2412,2413,1900", help="LO MHz: '2400:2480:10,1900'")
    p.add_argument("--gain", type=int, default=40, help="receive gain index, -1 AGC")
    p.add_argument("--n", type=int, default=8192)
    p.add_argument("--repeat", type=int, default=3)
    p.set_defaults(func=cmd_floor)

    p = sub.add_parser("summarize", help="a sweep's JSON lines file")
    p.add_argument("file")
    p.set_defaults(func=cmd_summarize)

    args = ap.parse_args()
    if getattr(args, "cmd", None) == "sweep" and not (args.pairs or args.range):
        ap.error("sweep needs --pairs or --range")
    args.func(args)


if __name__ == "__main__":
    sys.exit(main())
