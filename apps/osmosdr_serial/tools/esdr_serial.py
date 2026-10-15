#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Host side of the osmosdr serial transport (esdr_proto.h): requests, receive, transmit.

    esdr_serial.py PORT info
    esdr_serial.py PORT rx [--rate HZ] [--seconds S] [--bits 8|16] [--freq HZ] [--out FILE.npy]
    esdr_serial.py PORT tx [--rate HZ] [--seconds S] [--tone HZ] [--freq HZ] [--gain STEP]

The port is opened raw, without touching DTR/RTS: that would reset the board.
"""

import argparse
import os
import struct
import sys
import termios
import time

import numpy as np

SER_MAGIC = 0x4C525345
BLOCK_MAGIC = 0x4B4C4245
SER_IN, SER_TX_DATA = 0x01, 0x80
HDR = struct.Struct("<IBBHHh")
BLK = struct.Struct("<IHHQ")
BLOCK_BYTES = 4096
REQ = dict(GET_INFO=1, SET_MODE=2, SET_FREQ=3, SET_SAMPLE_RATE=4, SET_RX_GAIN=5, SET_TX_GAIN=6,
           SET_BANDWIDTH=7, SET_FORMAT=8, SET_FREQ_CORR=9, GET_STATE=10, GET_STATS=11,
           SET_OPTIONS=12, SET_DIGITAL_GAIN=13)
MODE_OFF, MODE_RX, MODE_TX = 0, 1, 2
INFO = struct.Struct("<IHH32sQQIHHHHHH")
STATS = struct.Struct("<QQQQQQIIIIIIIIIIQQIIII")
STATS_NAMES = ("rx_samples rx_overflow_samples rx_ring_lost_pairs rx_restart_samples usb_in_bytes "
               "usb_out_bytes rx_blocks rx_overflows ring_runs ring_errors ring_late_max "
               "ring_cycles_x100 ring_abandoned ring_gain_refreshed ring_last_status "
               "ring_last_detail tx_samples tx_dropped tx_underruns tx_sessions tx_restarts "
               "tx_queued_max").split()


class Esdr:
    def __init__(self, port):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0
        a[2] = (a[2] | termios.CLOCAL | termios.CREAD) & ~termios.HUPCL
        a[6][termios.VMIN], a[6][termios.VTIME] = 0, 1
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        # A previous host may have stopped inside a transmit frame: zeros complete
        # any payload, then the device resynchronizes on the magic.
        os.write(self.fd, bytes(HDR.size + 4096))
        self.buf = b""
        self.blocks = []

    def close(self):
        os.close(self.fd)

    def _fill(self, n, timeout):
        end = time.time() + timeout
        while len(self.buf) < n:
            if time.time() > end:
                raise TimeoutError(f"wanted {n} bytes, have {len(self.buf)}")
            self.buf += os.read(self.fd, 65536)

    def _next(self, timeout=2.0):
        """The next item from the device: ('frame', hdr, data) or ('block', hdr, payload)."""
        while True:
            self._fill(4, timeout)
            magic = struct.unpack_from("<I", self.buf)[0]
            if magic == BLOCK_MAGIC:
                self._fill(BLOCK_BYTES, timeout)
                blk, self.buf = self.buf[:BLOCK_BYTES], self.buf[BLOCK_BYTES:]
                return "block", BLK.unpack_from(blk), blk[BLK.size:]
            if magic == SER_MAGIC:
                self._fill(HDR.size, timeout)
                h = HDR.unpack_from(self.buf)
                self._fill(HDR.size + h[4], timeout)
                data = self.buf[HDR.size:HDR.size + h[4]]
                self.buf = self.buf[HDR.size + h[4]:]
                return "frame", h, data
            self.buf = self.buf[1:]

    def request(self, name, value=0, data=b"", in_len=0, timeout=2.0):
        req = REQ[name]
        flags = SER_IN if in_len else 0
        os.write(self.fd, HDR.pack(SER_MAGIC, req, flags, value, in_len or len(data), 0) + data)
        while True:
            kind, h, payload = self._next(timeout)
            if kind == "block":
                self.blocks.append((h, payload))
                continue
            if h[1] != req:
                continue
            if h[5] != 0:
                raise OSError(-h[5], f"{name}: {os.strerror(-h[5])}")
            return payload

    def info(self):
        f = INFO.unpack(self.request("GET_INFO", in_len=INFO.size)[:INFO.size])
        return dict(zip("magic proto size firmware fmin fmax rate_max rate_count rx_gain_max "
                        "tx_gain_max block_bytes full_scale16 reserved".split(), f))

    def stats(self):
        p = self.request("GET_STATS", in_len=STATS.size)
        return dict(zip(STATS_NAMES, STATS.unpack(p[:STATS.size])))

    def set_u32(self, name, v):
        self.request(name, data=struct.pack("<I", v))

    def set_freq(self, hz):
        self.request("SET_FREQ", data=struct.pack("<Q", hz))

    def send_tx(self, raw):
        for k in range(0, len(raw), 4096):
            chunk = raw[k:k + 4096]
            os.write(self.fd, HDR.pack(SER_MAGIC, SER_TX_DATA, 0, 0, len(chunk), 0) + chunk)


def peaks(x, fs, k=5):
    x = x - x.mean()
    n = len(x)
    spec = np.fft.fftshift(np.abs(np.fft.fft(x * np.hanning(n))) ** 2)
    f = np.fft.fftshift(np.fft.fftfreq(n, 1 / fs))
    db = 10 * np.log10(spec / np.median(spec) + 1e-12)
    out, used = [], np.zeros(n, bool)
    for _ in range(k):
        i = int(np.argmax(np.where(used, -99, db)))
        out.append(f"{f[i] / 1e3:+.2f} kHz {db[i]:.1f} dB")
        used[max(0, i - n // 400):i + n // 400] = True
    return ", ".join(out)


def cmd_rx(d, a):
    d.set_freq(a.freq)
    d.set_u32("SET_SAMPLE_RATE", a.rate)
    d.request("SET_FORMAT", value=a.bits)
    if a.gain is not None:
        d.request("SET_RX_GAIN", value=a.gain)
    d.blocks = []
    d.request("SET_MODE", value=MODE_RX)
    t0, got, items, flags, last = time.time(), 0, [], 0, None
    while time.time() - t0 < a.seconds:
        kind, h, payload = d._next(2.0)
        if kind != "block":
            continue
        _, fl, n, index = h
        flags |= fl
        dt = np.int16 if fl & 4 else np.int8
        v = np.frombuffer(payload, dt)[:2 * n].astype(np.float32)
        items.append(v[0::2] + 1j * v[1::2])
        if last is not None and index != last:
            print(f"index jump at block {len(items)}: {last} -> {index}, flags {fl:#x}")
        last = index + n
        got += n
    dt = time.time() - t0
    d.request("SET_MODE", value=MODE_OFF)
    x = np.concatenate(items)
    print(f"{got} samples in {dt:.2f} s = {got / dt / 1e3:.1f} kS/s, flags seen {flags:#x}")
    print("spectrum:", peaks(x[-(1 << 16):], a.rate))
    print("stats:", {k: v for k, v in d.stats().items() if v})
    if a.out:
        np.save(a.out, x)


def cmd_tx(d, a):
    n = int(a.rate * a.seconds)
    ph = 2 * np.pi * a.tone / a.rate * np.arange(n)
    if a.bits == 16:
        v = np.empty(2 * n, np.int16)
        v[0::2], v[1::2] = 16000 * np.cos(ph), 16000 * np.sin(ph)
    else:
        v = np.empty(2 * n, np.int8)
        v[0::2], v[1::2] = 100 * np.cos(ph), 100 * np.sin(ph)
    d.set_freq(a.freq)
    d.set_u32("SET_SAMPLE_RATE", a.rate)
    d.request("SET_FORMAT", value=a.bits)
    d.request("SET_TX_GAIN", value=a.gain)
    d.request("SET_MODE", value=MODE_TX)
    t0 = time.time()
    d.send_tx(v.tobytes())
    dt = time.time() - t0
    d.request("SET_MODE", value=MODE_OFF, timeout=10.0)
    print(f"sent {n} samples in {dt:.2f} s = {n / dt / 1e3:.1f} kS/s")
    print("stats:", {k: v for k, v in d.stats().items() if v})


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    for name in ("rx", "tx"):
        p = sub.add_parser(name)
        p.add_argument("--rate", type=int, default=250000)
        p.add_argument("--seconds", type=float, default=4.0)
        p.add_argument("--bits", type=int, default=8)
        p.add_argument("--freq", type=int, default=2437000000)
        if name == "rx":
            p.add_argument("--gain", type=int)
            p.add_argument("--out")
        else:
            p.add_argument("--tone", type=float, default=50000)
            p.add_argument("--gain", type=int, default=8)
    a = ap.parse_args()
    d = Esdr(a.port)
    try:
        if a.cmd == "info":
            for k, v in d.info().items():
                print(f"{k}: {v.rstrip(b'\0').decode() if isinstance(v, bytes) else v}")
        elif a.cmd == "rx":
            cmd_rx(d, a)
        else:
            cmd_tx(d, a)
    finally:
        d.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
