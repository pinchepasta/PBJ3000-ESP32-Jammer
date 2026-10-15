#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Drive the stream VITA 49.2 transmit path and check it.

Sends control, query-state and signal data packets to the board, checks the
acknowledges, then compares the board's own counters (shell "sdr status").

Usage: vrt_tx.py [--iface esdr0] [--device ADDR] [--freq MHZ] [--rate HZ]
                 [--tone HZ] [--seconds N] [--shell /dev/serial/by-id/...]

The board's address is learnt from its receive stream unless --device is given.
Exit status is 0 when every check passes.
"""

import argparse
import glob
import math
import re
import socket
import struct
import sys
import time

PT_DATA_SID, PT_COMMAND = 1, 6
TSI_OTHER, TSF_REAL_TIME = 3, 2
CIF0_RF, CIF0_RATE = 1 << 27, 1 << 21
# CAM bits (VITA 49.2 control/acknowledge mode word)
CAM_CONTROLLEE = 1 << 31
CAM_EXECUTE = 2 << 23
CAM_V, CAM_X, CAM_S, CAM_W, CAM_ER = 1 << 20, 1 << 19, 1 << 18, 1 << 17, 1 << 16
CAM_DONE = 1 << 10  # scheduled or executed (acknowledge only)
HDR_ACK = 1 << 26
WEF_OUT_OF_RANGE = 0x10000000
TRAILER_VALID = 0x40040000  # valid data: enable and indicator
SAMPLES_PER_PACKET = 352


def q44_20(hz):
    v = int(round(hz * (1 << 20))) & 0xFFFFFFFFFFFFFFFF
    return [v >> 32, v & 0xFFFFFFFF]


def from_q44_20(hi, lo):
    return struct.unpack(">q", struct.pack(">Q", hi << 32 | lo))[0] / (1 << 20)


def header(ptype, count, words, bit26=False, tsi=0, tsf=0):
    return (ptype << 28 | (bit26 << 26) | tsi << 22 | tsf << 20 | (count & 15) << 16 | words)


def pack(words):
    return struct.pack(f">{len(words)}I", *words)


def control(stream_id, msg_id, cam, fields):
    """fields: list of (cif0 bit, words) in CIF0 order."""
    cif0 = 0
    body = []
    for bit, w in sorted(fields, key=lambda f: -f[0]):
        cif0 |= bit
        body += w
    words = [0, stream_id, CAM_CONTROLLEE | cam, msg_id, stream_id, cif0] + body
    words[0] = header(PT_COMMAND, msg_id, len(words))
    return pack(words)


def parse_ack(data):
    w = struct.unpack(f">{len(data) // 4}I", data)
    hdr = w[0]
    if hdr >> 28 != PT_COMMAND or not hdr & HDR_ACK:
        raise ValueError(f"not an acknowledge: {hdr:#010x}")
    if hdr & 0xFFFF != len(w):
        raise ValueError("size field mismatch")
    a = {"stream_id": w[1], "cam": w[2], "msg_id": w[3]}
    i = 4 + (1 if w[2] & CAM_CONTROLLEE else 0) + (1 if w[2] & (1 << 29) else 0)
    if w[2] & CAM_S:
        cif0 = w[i]
        i += 1
        if cif0 & CIF0_RF:
            a["rf"] = from_q44_20(w[i], w[i + 1])
            i += 2
        if cif0 & CIF0_RATE:
            a["rate"] = from_q44_20(w[i], w[i + 1])
            i += 2
    else:
        wif = w[i] if w[2] & CAM_W else 0
        i += 1 if w[2] & CAM_W else 0
        eif = w[i] if w[2] & CAM_ER else 0
        i += 1 if w[2] & CAM_ER else 0
        i += bin(wif & 0xFFFFFF00).count("1")
        a["errors"] = {}
        for bit in range(31, 7, -1):
            if eif >> bit & 1:
                a["errors"][bit] = w[i]
                i += 1
    return a


def learn_device(iface, port, timeout=3.0):
    s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, iface.encode())
    except OSError:
        pass
    s.bind(("::", port))
    s.settimeout(timeout)
    try:
        _, addr = s.recvfrom(65536)
    finally:
        s.close()
    return addr[0].split("%")[0]


def shell_tx_stats(path):
    import serial

    s = serial.Serial(path, 115200, timeout=0.3)
    s.dtr = True
    time.sleep(0.4)
    s.read(65536)
    s.write(b"sdr status\r\n")
    time.sleep(1.0)
    out = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", s.read(65536).decode(errors="replace"))
    s.close()
    m = re.search(r"packets (\d+) data, (\d+) commands, (\d+) acks, (\d+) gaps, (\d+) underruns, "
                  r"(\d+) bad, (\d+) errors", out)
    b = re.search(r"backend (\d+) samples", out)
    if not m or not b:
        raise RuntimeError("no 'sdr status' output:\n" + out)
    keys = ["data", "commands", "acks", "gaps", "underruns", "bad", "errors"]
    st = dict(zip(keys, map(int, m.groups())))
    st["samples"] = int(b.group(1))
    return st


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--iface", default="esdr0")
    ap.add_argument("--device", help="board IPv6 link-local address")
    ap.add_argument("--port", type=int, default=4992)
    ap.add_argument("--rx-port", type=int, default=4991)
    ap.add_argument("--freq", type=float, default=2437.0, help="TX frequency in MHz")
    ap.add_argument("--rate", type=float, default=100000.0, help="TX sample rate in Hz")
    ap.add_argument("--tone", type=float, default=10000.0, help="tone offset in Hz")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--stream-id", type=lambda v: int(v, 0), default=0x5D5D0101)
    ap.add_argument("--shell", default=None, help="board shell port (default: auto)")
    args = ap.parse_args()

    scope = socket.if_nametoindex(args.iface)
    device = args.device or learn_device(args.iface, args.rx_port)
    dest = (device, args.port, 0, scope)
    print(f"vrt_tx: board {device}%{args.iface}, port {args.port}")
    s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    s.settimeout(2.0)
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        print(f"vrt_tx: {'PASS' if cond else 'FAIL'} {name} {detail}".rstrip())

    def exchange(pkt):
        s.sendto(pkt, dest)
        return parse_ack(s.recvfrom(65536)[0])

    sid = args.stream_id
    rf_ok = [(CIF0_RF, q44_20(args.freq * 1e6)), (CIF0_RATE, q44_20(args.rate))]
    a = exchange(control(sid, 1, CAM_EXECUTE | CAM_X | CAM_ER, rf_ok))
    check("control execute", a["msg_id"] == 1 and a["cam"] & CAM_X and a["cam"] & CAM_DONE
          and not a["errors"], f"(cam {a['cam']:#010x}, errors {a['errors']})")

    bad = [(CIF0_RF, q44_20(7000e6))]
    a = exchange(control(sid, 2, CAM_EXECUTE | CAM_X | CAM_ER, bad))
    check("control out of range", a["msg_id"] == 2 and not a["cam"] & CAM_DONE
          and a["errors"].get(27) == WEF_OUT_OF_RANGE, f"(errors {a['errors']})")

    a = exchange(control(sid, 3, CAM_S, []))
    check("query state", abs(a.get("rf", 0) - args.freq * 1e6) < 1 and
          abs(a.get("rate", 0) - args.rate) < 1,
          f"(rf {a.get('rf', 0) / 1e6:.3f} MHz, rate {a.get('rate', 0):.0f} S/s)")

    shell = args.shell or (glob.glob("/dev/serial/by-id/usb-Zephyr_Project_ESP-SDR*") or [None])[0]
    before = shell_tx_stats(shell) if shell else None

    # Complex tone, paced at the sample rate.
    n_packets = int(args.seconds * args.rate / SAMPLES_PER_PACKET)
    step = 2 * math.pi * args.tone / args.rate
    t0 = time.perf_counter()
    n = 0
    for k in range(n_packets):
        body = []
        for j in range(SAMPLES_PER_PACKET):
            ph = step * (n + j)
            i_, q_ = int(16000 * math.cos(ph)), int(16000 * math.sin(ph))
            body.append((i_ & 0xFFFF) << 16 | (q_ & 0xFFFF))
        ts = n / args.rate
        frac_ps = int(round((ts % 1) * 1e12))
        words = [0, sid, int(ts), frac_ps >> 32, frac_ps & 0xFFFFFFFF] + body + [TRAILER_VALID]
        words[0] = header(PT_DATA_SID, k, len(words), bit26=True, tsi=TSI_OTHER, tsf=TSF_REAL_TIME)
        s.sendto(pack(words), dest)
        n += SAMPLES_PER_PACKET
        lag = t0 + n / args.rate - time.perf_counter()
        if lag > 0:
            time.sleep(lag)
    elapsed = time.perf_counter() - t0
    print(f"vrt_tx: sent {n_packets} packets, {n} samples in {elapsed:.2f} s "
          f"({n / elapsed:.0f} S/s)")

    if shell:
        time.sleep(1.0)
        after = shell_tx_stats(shell)
        d = {k: after[k] - before[k] for k in after}
        check("board received all packets", d["data"] == n_packets, f"({d['data']}/{n_packets})")
        check("no gaps or bad packets", d["gaps"] == 0 and d["bad"] == 0 and d["errors"] == 0,
              f"(gaps {d['gaps']}, bad {d['bad']}, errors {d['errors']}, "
              f"underruns {d['underruns']})")
        # The backend restarts its counters on every start; compare the last run.
        check("backend consumed every sample", after["samples"] == n, f"({after['samples']}/{n})")
    print(f"vrt_tx: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
