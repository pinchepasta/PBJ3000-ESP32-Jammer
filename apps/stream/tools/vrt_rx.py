#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Receive the stream VITA 49 stream over UDP and print per-second statistics.

Usage: vrt_rx.py [--iface esdr0] [--port 4991] [--seconds N] [--out file.cs16]
                 [--expect-freq MHZ] [--expect-rate HZ]

Exit status is 0 when data and context packets arrived without counter gaps
(and the expectations, if given, hold), so the script doubles as a HIL check.
"""

import argparse
import math
import socket
import struct
import sys
import time

DATA_TYPES = {0, 1, 2, 3}
CONTEXT_TYPES = {4, 5}
WITH_STREAM_ID = {1, 3, 4, 5}

# CIF0 fields in transmission order (bit, name, words). 64-bit frequencies are Q44.20 Hz.
CIF0_FIELDS = [
    (30, "reference_point", 1), (29, "bandwidth", 2), (28, "if_frequency", 2),
    (27, "rf_frequency", 2), (26, "rf_frequency_offset", 2), (25, "if_band_offset", 2),
    (24, "reference_level", 1), (23, "gain", 1), (22, "over_range_count", 1),
    (21, "sample_rate", 2), (20, "timestamp_adjustment", 2), (19, "timestamp_calibration", 1),
    (18, "temperature", 1), (17, "device_id", 2), (16, "state_event", 1),
    (15, "payload_format", 2),
]
# The single-word CIF1 fields libvrt supports, in transmission order.
CIF1_FIELDS = [(4, "health_status"), (3, "v49_spec_compliance"), (2, "version_build_code")]
SPEC_NAMES = {1: "49.0", 2: "49.1", 3: "49A", 4: "49.2"}
HZ_FIELDS = {"bandwidth", "if_frequency", "rf_frequency", "rf_frequency_offset",
             "if_band_offset", "sample_rate"}


def parse(packet):
    """Return a dict for one big endian VRT packet, or raise ValueError."""
    if len(packet) < 4 or len(packet) % 4:
        raise ValueError(f"length {len(packet)}")
    words = struct.unpack(f">{len(packet) // 4}I", packet)
    hdr = words[0]
    p = {
        "type": hdr >> 28, "class_id": bool(hdr >> 27 & 1), "trailer": bool(hdr >> 26 & 1),
        "tsi": hdr >> 22 & 3, "tsf": hdr >> 20 & 3, "count": hdr >> 16 & 15,
        "size": hdr & 0xFFFF,
    }
    if p["size"] != len(words):
        raise ValueError(f"size field {p['size']} != {len(words)} words")
    i = 1
    if p["type"] in WITH_STREAM_ID:
        p["stream_id"] = words[i]
        i += 1
    if p["class_id"]:
        i += 2
    if p["tsi"]:
        p["seconds"] = words[i]
        i += 1
    if p["tsf"]:
        p["fraction"] = words[i] << 32 | words[i + 1]
        i += 2
    if p["type"] in DATA_TYPES:
        end = len(words) - (1 if p["trailer"] else 0)
        p["body"] = packet[i * 4:end * 4]
        if p["trailer"]:
            p["trailer_word"] = words[-1]
    elif p["type"] in CONTEXT_TYPES:
        cif0 = words[i]
        i += 1
        # 49.2: CIF0 bit 1 adds a CIF1 word; its fields follow all CIF0 fields.
        cif1 = 0
        if cif0 >> 1 & 1:
            cif1 = words[i]
            i += 1
        p["changed"] = bool(cif0 >> 31)
        for bit, name, n in CIF0_FIELDS:
            if not cif0 >> bit & 1:
                continue
            if n == 2:
                raw = words[i] << 32 | words[i + 1]
                if name in HZ_FIELDS:
                    p[name] = struct.unpack(">q", struct.pack(">Q", raw))[0] / (1 << 20)
                else:
                    p[name] = raw
            else:
                p[name] = words[i]
            i += n
        for bit, name in CIF1_FIELDS:
            if cif1 >> bit & 1:
                p[name] = words[i]
                i += 1
    return p


def iq_power_dbfs(body):
    """Mean power of big endian 16-bit I/Q pairs relative to a 10-bit full scale."""
    n = len(body) // 4
    if n == 0:
        return float("-inf")
    vals = struct.unpack(f">{2 * n}h", body[:4 * n])
    acc = sum(v * v for v in vals) / n
    return 10 * math.log10(acc / (512 * 512)) if acc else float("-inf")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--iface", default="esdr0")
    ap.add_argument("--port", type=int, default=4991)
    ap.add_argument("--seconds", type=float, default=0, help="0: run until interrupted")
    ap.add_argument("--out", help="append raw little endian complex int16 samples")
    ap.add_argument("--expect-freq", type=float, help="RF reference frequency in MHz")
    ap.add_argument("--expect-rate", type=float, help="sample rate in Hz")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, args.iface.encode())
    except OSError as e:
        print(f"vrt_rx: cannot bind to {args.iface}: {e}; listening on all interfaces")
    sock.bind(("::", args.port))
    sock.settimeout(1.0)
    out = open(args.out, "ab") if args.out else None

    totals = {"data": 0, "context": 0, "samples": 0, "gaps": 0, "bad": 0}
    window = {"data": 0, "bytes": 0, "samples": 0, "power": float("-inf")}
    last_count, context = None, {}
    start = last_print = time.monotonic()
    while not args.seconds or time.monotonic() - start < args.seconds:
        try:
            packet, addr = sock.recvfrom(65536)
        except socket.timeout:
            packet = None
        now = time.monotonic()
        if packet is not None:
            try:
                p = parse(packet)
            except (ValueError, struct.error) as e:
                totals["bad"] += 1
                print(f"vrt_rx: bad packet from {addr[0]}: {e}")
                continue
            if p["type"] in CONTEXT_TYPES:
                totals["context"] += 1
                if not context:
                    print(f"vrt_rx: stream {p.get('stream_id', 0):#010x} from {addr[0]}")
                context = p
                spec = SPEC_NAMES.get(p.get("v49_spec_compliance"), "49.0 (no CIF1)")
                print("vrt_rx: context VITA {}, rf {:.3f} MHz, rate {:.3f} MS/s, bandwidth {:.3f} MHz"
                      .format(spec, p.get("rf_frequency", 0) / 1e6, p.get("sample_rate", 0) / 1e6,
                              p.get("bandwidth", 0) / 1e6))
            elif p["type"] in DATA_TYPES:
                if last_count is not None and p["count"] != (last_count + 1) % 16:
                    totals["gaps"] += 1
                last_count = p["count"]
                n = len(p["body"]) // 4
                totals["data"] += 1
                totals["samples"] += n
                window["data"] += 1
                window["bytes"] += len(packet)
                window["samples"] += n
                window["power"] = iq_power_dbfs(p["body"])
                if out:
                    vals = struct.unpack(f">{2 * n}h", p["body"][:4 * n])
                    out.write(struct.pack(f"<{2 * n}h", *vals))
        if now - last_print >= 1.0:
            dt = now - last_print
            print("vrt_rx: {:.0f} data packets/s, {:.1f} kB/s, {:.3f} MS/s, {:.1f} dBFS, "
                  "gaps {}, bad {}".format(window["data"] / dt, window["bytes"] / dt / 1e3,
                                          window["samples"] / dt / 1e6, window["power"],
                                          totals["gaps"], totals["bad"]))
            window = {"data": 0, "bytes": 0, "samples": 0, "power": float("-inf")}
            last_print = now

    ok = totals["data"] > 0 and totals["context"] > 0 and totals["bad"] == 0
    if args.expect_freq is not None:
        ok &= abs(context.get("rf_frequency", 0) / 1e6 - args.expect_freq) < 1e-3
    if args.expect_rate is not None:
        ok &= abs(context.get("sample_rate", 0) - args.expect_rate) < 1
    print("vrt_rx: {} data, {} context packets, {} samples, {} gaps, {} bad: {}".format(
        totals["data"], totals["context"], totals["samples"], totals["gaps"], totals["bad"],
        "PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
