#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Two-board TX/RX lab for stream: drive both shells, record the receiver's
VRT stream with numpy, analyse it with a GNU Radio FFT flowgraph.

Boards are addressed by USB serial; the receiver's NCM interface is found from
the same serial in sysfs. Each received packet is stamped with host time, so
bursts can be matched to what the transmitter was doing (the two boards have
no common clock).

Subcommands:
  onoff   tone on/off comparison at one offset (is anything there at all?)
  sweep   tone offset sweep per transmit gain, one raw .cs16 file per gain
  paint   image to IQ for a waterfall (spectrum painter), to a file, no boards
  link    VITA 49.2 stream to the transmitter, decimated capture at the receiver;
          --signal image paints a picture, --out also writes the waterfall .png

Raw files (onoff, sweep, chirp, txstream) hold the captured bursts back to
back (cs16: little endian int16 I, Q); the .json sidecar has the per-burst
host times and the transmitter state. link writes cs16 on a real time axis
instead: bursts at their VITA timestamps, zeros in between.
"""

import argparse
import glob
import json
import os
import re
import socket
import struct
import sys
import threading
import time

import numpy as np
from gnuradio import analog, blocks, fft, gr
from gnuradio.fft import window

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vrt_rx  # noqa: E402
import vrt_tx  # noqa: E402

FS = 16_000_000
NFFT = 1024
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


def iface_for_serial(serial):
    for path in glob.glob("/sys/class/net/*/device"):
        d = os.path.realpath(path)
        while d != "/":
            f = os.path.join(d, "serial")
            if os.path.exists(f):
                if open(f).read().strip() == serial:
                    return path.split("/")[4]
                break
            d = os.path.dirname(d)
    raise SystemExit(f"no network interface for board {serial}")


class Board:
    def __init__(self, serial):
        import serial as pyserial
        self.serial = serial
        # "ESP-SDR" (older images: "ESP-SDR RX stream") plus the serial number.
        ports = glob.glob(f"/dev/serial/by-id/usb-Zephyr_Project_ESP-SDR*_{serial}-if00")
        if not ports:
            sys.exit(f"{serial}: no shell port under /dev/serial/by-id")
        self.port = pyserial.Serial(ports[0], 115200, timeout=0.05)
        self.port.reset_input_buffer()

    def cmd(self, line, timeout=2.0, expect=None):
        """Run one shell command, return its cleaned output once `expect` (or the
        command's own echo plus a following prompt) shows up."""
        self.port.reset_input_buffer()
        self.port.write((line + "\r\n").encode())
        buf, t0 = "", time.monotonic()
        while time.monotonic() - t0 < timeout:
            buf += self.port.read(4096).decode(errors="replace")
            clean = ANSI.sub("", buf)
            if expect and re.search(expect, clean):
                return clean
            if not expect and line in clean and clean.rstrip().endswith("$"):
                return clean
        return ANSI.sub("", buf)


CIF0_PAYLOAD = 1 << 15


def payload_words(bits):
    """Payload format field: link efficient, complex cartesian, signed fixed point, bits per item."""
    return [1 << 31 | 1 << 29 | (bits - 1) << 6 | (bits - 1), 0]


def pack_iq(iq, bits):
    """Big endian body for complex samples at full scale 1; 12 bits: whole groups of 4 samples."""
    if bits == 16:
        v = np.empty(2 * len(iq), dtype=">i2")
        v[0::2] = np.clip(np.round(iq.real * 32767), -32767, 32767)
        v[1::2] = np.clip(np.round(iq.imag * 32767), -32767, 32767)
        return v.tobytes()  # I in the upper half word
    lim = (1 << (bits - 1)) - 1
    v = np.empty(2 * len(iq), dtype=np.int32)
    v[0::2] = np.clip(np.round(iq.real * lim), -lim, lim)
    v[1::2] = np.clip(np.round(iq.imag * lim), -lim, lim)
    if bits == 8:
        return v.astype(np.int8).tobytes()
    u = (v & 0xFFF).reshape(-1, 2)  # MSB first: two items per three bytes
    b = np.empty((len(u), 3), dtype=np.uint8)
    b[:, 0] = u[:, 0] >> 4
    b[:, 1] = (u[:, 0] & 0xF) << 4 | u[:, 1] >> 8
    b[:, 2] = u[:, 1] & 0xFF
    return b.tobytes()


def unpack_iq(body, bits):
    """Items of a big endian body as float32, at 2^(bits-1) full scale (16: native)."""
    if bits == 16:
        return np.frombuffer(body, dtype=">i2").astype(np.float32)
    if bits == 8:
        return np.frombuffer(body, dtype=np.int8).astype(np.float32)
    b = np.frombuffer(body[:len(body) // 3 * 3], dtype=np.uint8).reshape(-1, 3).astype(np.int32)
    v = np.empty(2 * len(b), dtype=np.int32)
    v[0::2] = b[:, 0] << 4 | b[:, 1] >> 4
    v[1::2] = (b[:, 1] & 0xF) << 8 | b[:, 2]
    return (v - ((v & 0x800) << 1)).astype(np.float32)


class Recorder(threading.Thread):
    """Collect VRT data packets from one interface: (host_time, complex64 array)."""

    def __init__(self, iface, port=4991):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, iface.encode())
        self.sock.bind(("::", port))
        self.sock.settimeout(0.2)
        self.iface, self.peer, self.rate, self.bits = iface, None, FS, 16
        self.packets, self.running, self.lock = [], True, threading.Lock()

    def run(self):
        while self.running:
            try:
                pkt, addr = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            t = time.monotonic()
            self.peer = addr[0].split("%")[0]
            try:
                p = vrt_rx.parse(pkt)
            except (ValueError, struct.error):
                continue
            if p["type"] in vrt_rx.CONTEXT_TYPES:
                self.rate = p.get("sample_rate", self.rate)
                if "payload_format" in p:
                    self.bits = (p["payload_format"] >> 32 & 0x3F) + 1
                continue
            if p["type"] not in vrt_rx.DATA_TYPES:
                continue
            ts = p.get("seconds", 0) + p.get("fraction", 0) / 1e12
            v = unpack_iq(p["body"], self.bits)
            if self.bits != 16:
                # Back to the board's native scale: 512 raw, 32768 decimated.
                v *= (512 if self.rate == FS else 32768) / (1 << (self.bits - 1))
            with self.lock:
                self.packets.append((t, ts, v[0::2] + 1j * v[1::2], self.rate))

    def take(self):
        with self.lock:
            out, self.packets = self.packets, []
        return out

    def stop(self):
        self.running = False
        self.join()


CIF0_GAIN = 1 << 23
RX_STREAM_ID = 0x5D5D0001


def gain_word(stage1, stage2=0.0):
    """VITA 49 gain field: two radix-7 fixed point halves, stage 2 in the upper one."""
    q = lambda db: int(round(db * 128)) & 0xFFFF
    return (q(stage2) << 16) | q(stage1)


def parse_state(data):
    """Query-state acknowledge: rf, gain, sample rate (the fields the board reports)."""
    w = struct.unpack(f">{len(data) // 4}I", data)
    i = 4 + (1 if w[2] & vrt_tx.CAM_CONTROLLEE else 0) + (1 if w[2] & (1 << 29) else 0)
    cif0, i, out = w[i], i + 1, {}
    if cif0 & vrt_tx.CIF0_RF:
        out["rf_hz"], i = vrt_tx.from_q44_20(w[i], w[i + 1]), i + 2
    if cif0 & CIF0_GAIN:
        out["gain_stage1"], i = struct.unpack(">h", struct.pack(">H", w[i] & 0xFFFF))[0] / 128, i + 1
    if cif0 & vrt_tx.CIF0_RATE:
        out["rate_hz"], i = vrt_tx.from_q44_20(w[i], w[i + 1]), i + 2
    if cif0 & CIF0_PAYLOAD:
        out["bits"], i = (w[i] & 0x3F) + 1, i + 2
    return out


def exchange(s, dest, packet, tries=3):
    """Send a command and return the reply: UDP may drop either, and commands are idempotent."""
    for k in range(tries):
        s.sendto(packet, dest)
        try:
            return s.recvfrom(65536)[0]
        except socket.timeout:
            if k == tries - 1:
                raise


def vita_rx_control(rec, gain=None, mhz=None, msg_id=1, rate=None, bits=None):
    """Set the receiver over VITA 49.2 (command to the RX stream ID), then query it back."""
    for _ in range(50):
        if rec.peer:
            break
        time.sleep(0.1)
    dest = (rec.peer, 4992, 0, socket.if_nametoindex(rec.iface))
    s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    s.settimeout(2.0)
    fields = []
    if gain is not None:
        fields.append((CIF0_GAIN, [gain_word(gain)]))
    if mhz is not None:
        fields.append((vrt_tx.CIF0_RF, vrt_tx.q44_20(mhz * 1e6)))
    if rate is not None:
        fields.append((vrt_tx.CIF0_RATE, vrt_tx.q44_20(rate)))
    if bits is not None:
        fields.append((CIF0_PAYLOAD, payload_words(bits)))
    cam = vrt_tx.CAM_EXECUTE | vrt_tx.CAM_X | vrt_tx.CAM_ER
    ack = vrt_tx.parse_ack(exchange(s, dest, vrt_tx.control(RX_STREAM_ID, msg_id, cam, fields)))
    state = parse_state(exchange(s, dest, vrt_tx.control(RX_STREAM_ID, msg_id + 1, vrt_tx.CAM_S, [])))
    s.close()
    return {"executed": bool(ack["cam"] & vrt_tx.CAM_DONE), "errors": ack["errors"], **state}


def bursts(packets):
    """Merge packets into bursts using the device timestamps (contiguous samples)."""
    out = []
    for t, ts, iq, rate in packets:
        if out and out[-1]["rate"] == rate and abs(ts - out[-1]["end"]) < 2.0 / rate:
            out[-1]["iq"].append(iq)
            out[-1]["end"] = ts + len(iq) / rate
        else:
            out.append({"host": t, "start": ts, "end": ts + len(iq) / rate, "iq": [iq],
                        "rate": rate})
    for b in out:
        b["iq"] = np.concatenate(b["iq"]).astype(np.complex64)
    return out


def gr_spectrum(iq):
    """Mean power spectrum (linear, fftshifted) of iq via a GNU Radio flowgraph."""
    n = len(iq) // NFFT * NFFT
    if n == 0:
        return None
    tb = gr.top_block()
    src = blocks.vector_source_c(iq[:n].tolist(), False)
    s2v = blocks.stream_to_vector(gr.sizeof_gr_complex, NFFT)
    f = fft.fft_vcc(NFFT, True, window.blackmanharris(NFFT), True)
    mag = blocks.complex_to_mag_squared(NFFT)
    snk = blocks.vector_sink_f(NFFT)
    tb.connect(src, s2v, f, mag, snk)
    tb.run()
    return np.asarray(snk.data(), dtype=np.float64).reshape(-1, NFFT).mean(axis=0)


def tone_snr_db(spec, offset_hz):
    """Power in the tone's bins over the median bin, in dB (DC bins excluded)."""
    k = NFFT // 2 + round(offset_hz * NFFT / FS)
    noise = np.median(np.delete(spec, range(NFFT // 2 - 2, NFFT // 2 + 3)))
    return 10 * np.log10(spec[k - 1:k + 2].max() / noise)


def spectrum_of(bs):
    specs = [gr_spectrum(b["iq"]) for b in bs]
    specs = [s for s in specs if s is not None]
    return np.mean(specs, axis=0) if specs else None


def write_raw(path, bs, meta):
    iq = np.concatenate([b["iq"] for b in bs]) if bs else np.zeros(0, np.complex64)
    out = np.empty(2 * len(iq), dtype="<i2")
    out[0::2], out[1::2] = np.round(iq.real), np.round(iq.imag)
    out.tofile(path)
    meta["samples"] = len(iq)
    meta["bursts"] = [{"host_s": round(b["host"], 6), "samples": len(b["iq"]),
                       "tx": b.get("tx")} for b in bs]
    with open(os.path.splitext(path)[0] + ".json", "w") as f:
        json.dump(meta, f, indent=1)


def tx_tone(tx, offset, ms, amp=511):
    out = tx.cmd(f"sdr tonetx {offset} {ms} {amp}", timeout=ms / 1000 + 3, expect=r"tonetx: .*\n|error|play:")
    m = re.search(r"(\d+) bursts", out)
    if not m:
        raise SystemExit(f"tonetx failed: {out.strip()}")
    return int(m.group(1))


def label(bs, windows):
    """Tag each burst with the tx state window (host time) it falls into."""
    for b in bs:
        b["tx"] = None
        for t0, t1, state in windows:
            if t0 <= b["host"] <= t1:
                b["tx"] = state
                break
    return bs


def cmd_onoff(a, tx, rec):
    windows = []
    for rep in range(a.reps):
        t0 = time.monotonic()
        n = tx_tone(tx, a.offset, a.ms)
        # Shell answers after the play: the TX window is [t0, now] minus the
        # ~6 ms front-end switch at each end; trim 20 ms for USB latency.
        windows.append((t0 + 0.02, time.monotonic() - 0.02, "on"))
        t1 = time.monotonic()
        time.sleep(a.ms / 1000)
        windows.append((t1 + 0.02, time.monotonic() - 0.02, "off"))
        print(f"rep {rep}: {n} tx bursts")
    time.sleep(0.3)
    bs = label(bursts(rec.take()), windows)
    on = [b for b in bs if b["tx"] == "on"]
    off = [b for b in bs if b["tx"] == "off"]
    s_on, s_off = spectrum_of(on), spectrum_of(off)
    if s_on is None or s_off is None:
        print(f"not enough bursts: on {len(on)}, off {len(off)}")
        return False
    snr_on, snr_off = tone_snr_db(s_on, a.offset), tone_snr_db(s_off, a.offset)
    p_on = 10 * np.log10(np.mean([np.mean(abs(b["iq"]) ** 2) for b in on]) / 512 ** 2)
    p_off = 10 * np.log10(np.mean([np.mean(abs(b["iq"]) ** 2) for b in off]) / 512 ** 2)
    print(f"tx on : {len(on)} bursts, {p_on:.1f} dBFS, tone bin {snr_on:+.1f} dB over median")
    print(f"tx off: {len(off)} bursts, {p_off:.1f} dBFS, tone bin {snr_off:+.1f} dB over median")
    if a.out:
        write_raw(a.out, on + off, {"rate_hz": FS, "offset_hz": a.offset, "mode": "onoff"})
        np.save(os.path.splitext(a.out)[0] + "_spec.npy", np.vstack([s_on, s_off]))
    seen = snr_on - snr_off >= a.threshold
    print("tone VISIBLE" if seen else "tone not visible")
    return seen


def excess_peak(spec, ref):
    """Strongest TX-caused energy: (centroid Hz, peak dB over the TX-off median)."""
    f = (np.arange(NFFT) - NFFT // 2) * FS / NFFT
    ex = np.clip(spec - ref, 0, None)
    ex[NFFT // 2 - 2:NFFT // 2 + 3] = 0  # DC
    k = int(np.argmax(np.convolve(ex, np.ones(33), "same")))
    w = slice(max(k - 24, 0), k + 25)
    c = float((f[w] * ex[w]).sum() / ex[w].sum()) if ex[w].sum() > 0 else 0.0
    return c, float(10 * np.log10(spec[w].max() / np.median(ref)))


def tone_power_dbfs(bs, offset_hz):
    """Tone power from the spectrum (bins around the tone), dB relative to full scale."""
    s = spectrum_of(bs)
    k = NFFT // 2 + round(offset_hz * NFFT / FS)
    win = window.blackmanharris(NFFT)
    # Undo FFT scaling and window power: a full-scale complex tone gives 0 dB.
    ref = (512 * sum(win)) ** 2
    return float(10 * np.log10(s[k - 3:k + 4].sum() / ref)), s


def sdr_set(board, key, value):
    """sdr set <key> <value>, exiting if the board refuses it; returns the board's answer line."""
    out = board.cmd(f"sdr set {key} {value}")
    m = re.search(rf"^{re.escape(key)} {re.escape(str(value))}: (-?\d+)", out, re.M)
    if not m or int(m.group(1)) != 0:
        sys.exit(f"{board.serial}: sdr set {key} {value} failed: {out.strip()[-200:]!r}")
    return m.group(0)


def cmd_power(a, tx, rx, rec):
    rows = []
    rec.take()
    t0 = time.monotonic()
    time.sleep(a.ms / 1000)
    windows = [(t0 + 0.02, time.monotonic() - 0.02, "off")]
    for step in a.steps:
        sdr_set(tx, "txgain", step)
        out = tx.cmd("sdr status")
        m = re.search(r"vendor target power (-?\d+)", out)
        t0 = time.monotonic()
        tx_tone(tx, a.offset, a.ms, a.amp)
        windows.append((t0 + 0.02, time.monotonic() - 0.02, step))
        rows.append({"step": step, "vendor_power": int(m.group(1)) if m else None})
    time.sleep(0.3)
    bs = label(bursts(rec.take()), windows)
    p_off, _ = tone_power_dbfs([b for b in bs if b["tx"] == "off"], a.offset)
    for r in rows:
        sel = [b for b in bs if b["tx"] == r["step"]]
        iq = np.concatenate([b["iq"] for b in sel])
        r["tone_dbfs"], _ = tone_power_dbfs(sel, a.offset)
        r["tone_dbfs"] = round(r["tone_dbfs"], 1)
        r["peak"] = int(max(np.abs(iq.real).max(), np.abs(iq.imag).max()))
        r["bursts"] = len(sel)
        print(f"step {r['step']:2d}: vendor {r['vendor_power']:5d}  tone {r['tone_dbfs']:+6.1f} dBFS"
              f"  peak {r['peak']:3d}{'  CLIP' if r['peak'] >= 511 else ''}")
    print(f"tx off: same bins {p_off:+6.1f} dBFS")
    if a.out:
        with open(a.out, "w") as f:
            json.dump({"offset_hz": a.offset, "amp": a.amp, "off_dbfs": round(p_off, 1),
                       "steps": rows}, f, indent=1)
    return True


def burst_peak(iq, dc_hz=60e3):
    """Peak frequency (Hz, parabolic interpolation) and its height over the median bin (dB)."""
    n = len(iq)
    spec = np.abs(np.fft.fftshift(np.fft.fft(iq * np.blackman(n)))) ** 2
    # TX carrier leakage sits near DC (plus the boards' crystal offset) whenever TX is on.
    dc = int(np.ceil(dc_hz * n / FS))
    spec[n // 2 - dc:n // 2 + dc + 1] = np.median(spec)
    k = int(np.argmax(spec))
    if 0 < k < n - 1:
        a, b, c = np.log(spec[k - 1:k + 2] + 1e-30)
        d = 0.5 * (a - c) / (a - 2 * b + c) if a - 2 * b + c != 0 else 0.0
    else:
        d = 0.0
    return (k + d - n // 2) * FS / n, float(10 * np.log10(spec[k] / np.median(spec)))


def cmd_chirp(a, tx, rx, rec):
    print(sdr_set(tx, "txgain", a.txgain))
    rec.take()
    out = tx.cmd(f"sdr sweep {a.f0} {a.f1} {a.ms} {a.amp}", timeout=a.ms / 1000 + 5,
                 expect=r"sweep: .*\n|error")
    line = next((l for l in out.splitlines() if l.startswith("sweep:")), out.strip())
    print(line)
    time.sleep(0.3)
    bs = bursts(rec.take())
    pts = []
    for b in bs:
        f, snr = burst_peak(b["iq"])
        b["peak_hz"], b["snr_db"] = round(f), round(snr, 1)
        if snr >= 15:
            pts.append((b["start"], f))
    want = (a.f1 - a.f0) / (a.ms / 1000)
    k = None
    if len(pts) < 4:
        print(f"{len(bs)} bursts, only {len(pts)} with the tone: no fit")
    else:
        t = np.array([p[0] for p in pts]); f = np.array([p[1] for p in pts])
        for _ in range(3):  # drop outliers (e.g. room Wi-Fi), refit
            k, c = np.polyfit(t, f, 1)
            keep = np.abs(f - (k * t + c)) < max(3 * np.std(f - (k * t + c)), 20e3)
            t, f = t[keep], f[keep]
        k, c = np.polyfit(t, f, 1)
        print(f"{len(bs)} bursts, {len(t)} with the tone over {t.max() - t.min():.2f} s: slope "
              f"{k / 1e6:+.3f} MHz/s (commanded {want / 1e6:+.3f}), {f.min() / 1e6:+.2f} .. "
              f"{f.max() / 1e6:+.2f} MHz, rms residual {np.std(f - (k * t + c)) / 1e3:.1f} kHz")
    if a.out:
        write_raw(a.out, bs, {"rate_hz": FS, "rx_lo_mhz": a.lo, "f0_hz": a.f0, "f1_hz": a.f1,
                              "duration_ms": a.ms, "txgain": a.txgain, "amp": a.amp,
                              "board_line": line, "fit_slope_hz_per_s": None if k is None else round(k),
                              "commanded_slope_hz_per_s": round(want),
                              "note": "bursts back to back; per-burst device time in 'bursts'"})
        meta_path = os.path.splitext(a.out)[0] + ".json"
        meta = json.load(open(meta_path))
        for m, b in zip(meta["bursts"], bs):
            m.update({"device_s": round(b["start"], 7), "peak_hz": b["peak_hz"], "snr_db": b["snr_db"]})
        json.dump(meta, open(meta_path, "w"), indent=1)
    return k is not None and abs(k - want) < 0.05 * abs(want)


TX_STREAM_ID = 0x5D5D0101
TX_SAMPLES_PER_PACKET = 352  # 1500 byte datagrams


def gr_signal(kind, rate, seconds, offset, span, amp):
    """Complex baseband test signal from a GNU Radio flowgraph, complex64 numpy array."""
    n = int(rate * seconds)
    tb = gr.top_block()
    if kind == "tone":
        src = analog.sig_source_c(rate, analog.GR_COS_WAVE, offset, amp)
        head = blocks.head(gr.sizeof_gr_complex, n)
        snk = blocks.vector_sink_c()
        tb.connect(src, head, snk)
    else:
        # Linear chirp offset-span/2 .. offset+span/2 once over the run: a sawtooth
        # instantaneous frequency into an FM modulator.
        # Phase pi: GNU Radio's sawtooth otherwise starts mid-ramp and wraps halfway.
        saw = analog.sig_source_f(rate, analog.GR_SAW_WAVE, 1.0 / seconds, span, offset - span / 2,
                                  np.pi)
        head = blocks.head(gr.sizeof_float, n)
        fm = analog.frequency_modulator_fc(2 * np.pi / rate)
        gain = blocks.multiply_const_cc(amp)
        snk = blocks.vector_sink_c()
        tb.connect(saw, head, fm, gain, snk)
    tb.run()
    return np.asarray(snk.data(), dtype=np.complex64)


def paint_image(path, rate, line_s, width=None, amp=0.5, flip=False, offset=0.0, seed=1):
    """Spectrum painter (after github.com/polygon/spectrum_painter): each image row
    becomes one spectrum on a waterfall. Grey level squared sets the bin magnitude,
    random phases keep the crest factor down, the FFT is twice the image width with
    the picture in the centre half (rate / 2 wide), and each row repeats until it
    lasts line_s. Rows go out bottom first, so the picture stands upright on a
    waterfall with the newest line on top (flip: top row first)."""
    from PIL import Image

    pic = Image.open(path).convert("L")
    if width and pic.width != width:
        pic = pic.resize((width, max(1, round(pic.height * width / pic.width))), Image.LANCZOS)
    rows = np.asarray(pic, dtype=np.float32) / 255.0
    if not flip:
        rows = rows[::-1]
    w = rows.shape[1]
    nfft = 2 * w
    reps = max(1, int(np.ceil(line_s * rate / nfft)))
    spec = np.zeros((rows.shape[0] * reps, nfft), dtype=np.float32)
    spec[:, nfft // 4:nfft // 4 + w] = np.repeat(rows ** 2, reps, axis=0)
    rng = np.random.default_rng(seed)
    bins = spec * np.exp(2j * np.pi * rng.random(spec.shape, dtype=np.float32))
    iq = np.fft.ifft(np.fft.ifftshift(bins, axes=1), axis=1).ravel()
    if offset:
        iq *= np.exp(2j * np.pi * offset / rate * np.arange(len(iq)))
    iq *= amp / max(np.abs(iq).max(), 1e-12)
    meta = {"image": os.path.abspath(path), "image_px": [w, rows.shape[0]], "nfft": nfft,
            "line_s": reps * nfft / rate, "line_repeats": reps, "bandwidth_hz": rate / 2,
            "offset_hz": offset, "flip": flip, "seconds": len(iq) / rate}
    return iq.astype(np.complex64), meta


def write_waterfall(path, iq, rate, line_s, nfft=256, span_db=40.0):
    """Waterfall PNG of iq (complex, real time axis, zeros = no data): one row per
    line_s, newest on top, nfft bins from -rate/2 to +rate/2 left to right. Rows
    with no data stay black."""
    from PIL import Image

    hop = max(1, int(round(line_s * rate)))
    # Data runs: a gap is a run of zeros (quiet signal quantizes to the odd zero
    # sample too). Decimated bursts can be shorter than nfft: each piece is
    # windowed over its own length and zero padded to nfft.
    data = np.convolve((iq != 0).astype(np.float32), np.ones(16, np.float32), "same") > 0
    edge = np.diff(np.r_[0, data.astype(np.int8), 0])
    runs = list(zip(np.flatnonzero(edge == 1), np.flatnonzero(edge == -1)))
    pieces = [(k, min(k + nfft, b)) for a, b in runs for k in range(a, b, nfft)
              if min(k + nfft, b) - k >= 64]
    rows, p_i = [], 0
    for start in range(0, len(iq), hop):
        acc, n = None, 0
        while p_i < len(pieces) and pieces[p_i][0] < start + hop:
            a, b = pieces[p_i]
            seg = iq[a:b] * np.hanning(b - a)
            p = np.abs(np.fft.fftshift(np.fft.fft(seg, nfft))) ** 2 / np.sum(np.hanning(b - a) ** 2)
            acc = p if acc is None else acc + p
            n += 1
            p_i += 1
        rows.append(10 * np.log10(acc / n + 1e-12) if n else np.full(nfft, np.nan))
    db = np.array(rows)[::-1]
    top = np.nanpercentile(db, 99.5) if np.isfinite(db).any() else 0.0
    px = np.clip((db - (top - span_db)) / span_db, 0, 1)
    px = np.nan_to_num(px, nan=0.0)
    Image.fromarray((px * 255).astype(np.uint8), "L").save(path)
    return db.shape


def make_signal(a, rate):
    """The IQ to send, full scale 1: a cs16 file, a painted image, or a GNU Radio test signal."""
    if a.file:
        raw = np.fromfile(a.file, dtype="<i2").astype(np.float32) / 32767
        return (raw[0::2] + 1j * raw[1::2]).astype(np.complex64), {"file": a.file}
    if a.signal == "image":
        if not a.image:
            sys.exit("--signal image needs --image")
        return paint_image(a.image, rate, a.line_ms / 1e3, a.width, a.amp, a.flip, a.offset)
    return gr_signal(a.signal, rate, a.seconds, a.offset, a.span, a.amp), {}


def cmd_paint(a):
    iq, meta = paint_image(a.image, a.rate, a.line_ms / 1e3, a.width, a.amp, a.flip, a.offset)
    v = np.empty(2 * len(iq), dtype="<i2")
    v[0::2] = np.round(iq.real * 32767)
    v[1::2] = np.round(iq.imag * 32767)
    v.tofile(a.out)
    meta.update({"format": "cs16 little endian int16 I, Q, full scale 32767",
                 "rate_hz": a.rate, "samples": len(iq)})
    base = os.path.splitext(a.out)[0]
    with open(base + ".json", "w") as f:
        json.dump(meta, f, indent=1)
    shape = write_waterfall(base + ".png", iq, a.rate, meta["line_s"], nfft=meta["nfft"])
    print(f"wrote {a.out}: {len(iq)} samples, {meta['seconds']:.2f} s at {a.rate} S/s, "
          f"{meta['image_px'][0]} x {meta['image_px'][1]} px, {meta['line_s'] * 1e3:.1f} ms per line, "
          f"{meta['bandwidth_hz'] / 1e3:.0f} kHz wide; preview {base}.png ({shape[1]} x {shape[0]})")
    return True


def vita_tx_control(peer, iface, mhz, rate, gain, msg_id=1, bits=16):
    """Configure the board's transmitter over VITA 49.2 and read the settings back."""
    dest = (peer, 4992, 0, socket.if_nametoindex(iface))
    s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    s.settimeout(2.0)
    fields = [(vrt_tx.CIF0_RF, vrt_tx.q44_20(mhz * 1e6)), (vrt_tx.CIF0_RATE, vrt_tx.q44_20(rate)),
              (CIF0_GAIN, [gain_word(gain)]), (CIF0_PAYLOAD, payload_words(bits))]
    cam = vrt_tx.CAM_EXECUTE | vrt_tx.CAM_X | vrt_tx.CAM_ER
    ack = vrt_tx.parse_ack(exchange(s, dest, vrt_tx.control(TX_STREAM_ID, msg_id, cam, fields)))
    state = parse_state(exchange(s, dest, vrt_tx.control(TX_STREAM_ID, msg_id + 1, vrt_tx.CAM_S, [])))
    s.close()
    return {"executed": bool(ack["cam"] & vrt_tx.CAM_DONE), "errors": ack["errors"], **state}


def vita_tx_send(peer, iface, iq, rate, bits=16):
    """Stream iq as VITA signal data packets, paced at rate; timestamps = sample time."""
    dest = (peer, 4992, 0, socket.if_nametoindex(iface))
    s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
    # Same body size for every item size, whole packing quanta (4 samples at 12 bits).
    quantum = {8: 2, 12: 4, 16: 1}[bits]
    per_packet = TX_SAMPLES_PER_PACKET * 32 // (2 * bits) // quantum * quantum
    iq = iq[:len(iq) // quantum * quantum]
    t0 = time.perf_counter()
    k = 0
    for off in range(0, len(iq), per_packet):
        chunk = iq[off:off + per_packet]
        body = pack_iq(chunk, bits)
        ps = int(round(off / rate * 1e12))
        hdr = [0, TX_STREAM_ID, ps // 10**12, (ps % 10**12) >> 32, (ps % 10**12) & 0xFFFFFFFF]
        n = len(hdr) + len(body) // 4 + 1
        hdr[0] = vrt_tx.header(vrt_tx.PT_DATA_SID, k, n, bit26=True, tsi=vrt_tx.TSI_OTHER,
                               tsf=vrt_tx.TSF_REAL_TIME)
        pkt = struct.pack(f">{len(hdr)}I", *hdr) + body + struct.pack(">I", vrt_tx.TRAILER_VALID)
        s.sendto(pkt, dest)
        k += 1
        lag = t0 + (off + len(chunk)) / rate - time.perf_counter()
        if lag > 0:
            time.sleep(lag)
    s.close()
    return k, time.perf_counter() - t0


def learn_peer(iface):
    """Board address on iface, from its own VRT stream."""
    r = Recorder(iface)
    r.start()
    for _ in range(50):
        if r.peer:
            break
        time.sleep(0.1)
    r.stop()
    if not r.peer:
        raise SystemExit(f"no VRT stream on {iface}")
    return r.peer


def cmd_txstream(a, tx, rx, rec, tx_peer, tx_iface):
    iq, _ = make_signal(a, a.rate)
    r = vita_tx_control(tx_peer, tx_iface, a.lo, a.rate, a.txgain)
    print(f"VITA 49.2 tx control: {a.lo} MHz, {a.rate} S/s, step {a.txgain} -> executed "
          f"{r['executed']}, errors {r['errors']}, state {r}")
    if not r["executed"]:
        return False
    rec.take()
    n, dt = vita_tx_send(tx_peer, tx_iface, iq, a.rate)
    print(f"sent {len(iq)} samples in {n} packets over {dt:.2f} s ({len(iq) / dt / 1e3:.1f} kS/s)")
    time.sleep(1.0)  # board stops 500 ms after the last packet
    bs = bursts(rec.take())
    hits = []
    for b in bs:
        f, snr = burst_peak(b["iq"], dc_hz=35e3)
        b["peak_hz"], b["snr_db"] = round(f), round(snr, 1)
        if snr >= 15:
            hits.append((b["start"], f))
    print(f"board B: {len(bs)} bursts, {len(hits)} with the signal")
    if hits:
        f = np.array([h[1] for h in hits])
        print(f"  peak frequency {np.median(f) / 1e3:+.1f} kHz median, {f.min() / 1e3:+.1f} .. "
              f"{f.max() / 1e3:+.1f} kHz")
    out = tx.cmd("sdr status", timeout=2)
    for line in out.splitlines():
        if line.startswith(("tx ", "packets", "backend", "dac")):
            print("board A: " + line.strip())
    if a.out:
        write_raw(a.out, bs, {"rate_hz": FS, "rx_lo_mhz": a.lo, "tx_rate_hz": a.rate,
                              "signal": a.file or a.signal, "offset_hz": a.offset,
                              "span_hz": a.span, "txgain": a.txgain})
        meta_path = os.path.splitext(a.out)[0] + ".json"
        meta = json.load(open(meta_path))
        for m, b in zip(meta["bursts"], bs):
            m.update({"device_s": round(b["start"], 7), "peak_hz": b["peak_hz"], "snr_db": b["snr_db"]})
        json.dump(meta, open(meta_path, "w"), indent=1)
    return len(hits) > 0


def write_timed(path, bs, rate, full_scale, meta):
    """cs16 on a true time axis: each burst at its device timestamp, zeros in the gaps."""
    t_first = bs[0]["start"]
    n = int(round((bs[-1]["end"] - t_first) * rate))
    out = np.zeros(2 * n, dtype="<i2")
    covered = 0
    for b in bs:
        k = int(round((b["start"] - t_first) * rate))
        iq = b["iq"][:max(0, n - k)]
        out[2 * k:2 * (k + len(iq)):2] = np.round(iq.real)
        out[2 * k + 1:2 * (k + len(iq)):2] = np.round(iq.imag)
        covered += len(iq)
    out.tofile(path)
    meta.update({"rate_hz": rate, "full_scale": full_scale, "samples": n,
                 "seconds": round(n / rate, 6), "covered": round(covered / max(n, 1), 4),
                 "note": "real time axis: bursts at their VITA timestamps, gaps are zeros"})
    with open(os.path.splitext(path)[0] + ".json", "w") as f:
        json.dump(meta, f, indent=1)
    return covered / max(n, 1)


def cmd_link(a, tx, rx, rec, tx_peer, tx_iface):
    r = vita_rx_control(rec, gain=a.rx_gain if a.rx_gain is not None else 0, rate=a.rx_rate,
                        msg_id=20, bits=a.rx_bits)
    print(f"VITA 49.2 rx control: rate {a.rx_rate} S/s, {a.rx_bits} bit -> executed {r['executed']}, "
          f"errors {r['errors']}, state {r}")
    if not r["executed"]:
        return False
    iq, sig_meta = make_signal(a, a.tx_rate)
    r = vita_tx_control(tx_peer, tx_iface, a.lo, a.tx_rate, a.txgain, msg_id=30, bits=a.tx_bits)
    print(f"VITA 49.2 tx control: {a.lo} MHz, {a.tx_rate} S/s, step {a.txgain}, {a.tx_bits} bit "
          f"-> executed {r['executed']}, errors {r['errors']}, state {r}")
    if not r["executed"]:
        return False
    time.sleep(0.5)
    rec.take()
    t_lead = 0.5
    time.sleep(t_lead)  # some signal-free stream first, for reference
    n, dt = vita_tx_send(tx_peer, tx_iface, iq, a.tx_rate, a.tx_bits)
    time.sleep(1.0)
    bs = [b for b in bursts(rec.take()) if b["rate"] == a.rx_rate]
    print(f"sent {len(iq)} samples ({n} packets) in {dt:.2f} s; board B: {len(bs)} bursts at "
          f"{a.rx_rate} S/s")
    if not bs:
        return False
    out = tx.cmd("sdr status", timeout=2)
    for line in out.splitlines():
        if line.startswith(("packets", "dac")):
            print("board A: " + line.strip())
    peaks = []
    for b in bs:
        x = b["iq"]
        spec = np.abs(np.fft.fftshift(np.fft.fft(x * np.blackman(len(x))))) ** 2
        # The receiver's own DC offset dominates a narrow decimated band.
        dc = int(np.ceil(3e3 * len(x) / a.rx_rate))
        spec[len(x) // 2 - dc:len(x) // 2 + dc + 1] = np.median(spec)
        k = int(np.argmax(spec))
        peaks.append(((k - len(x) // 2) * a.rx_rate / len(x),
                      10 * np.log10(spec[k] / np.median(spec))))
    strong = [f for f, snr in peaks if snr >= 20]
    if strong:
        print(f"board B: {len(strong)} of {len(bs)} bursts with a peak >= 20 dB over the median; "
              f"peak {np.median(strong) / 1e3:+.1f} kHz median, {min(strong) / 1e3:+.1f} .. "
              f"{max(strong) / 1e3:+.1f} kHz")
    if a.out:
        cov = write_timed(a.out, bs, a.rx_rate, 32768,
                          {"rx_lo_mhz": a.lo, "signal": a.file or a.signal, "tx_rate_hz": a.tx_rate,
                           "offset_hz": a.offset, "span_hz": a.span,
                           "tx_seconds": len(iq) / a.tx_rate, "tx_signal": sig_meta,
                           "txgain": a.txgain, "tx_starts_after_s": t_lead,
                           "board_a": [l.strip() for l in out.splitlines()
                                       if l.startswith(("packets", "dac"))]})
        print(f"wrote {a.out}: {cov * 100:.0f} % of the time covered by bursts")
        if a.signal == "image" and not a.file:
            raw = np.fromfile(a.out, dtype="<i2").astype(np.float32)
            png = os.path.splitext(a.out)[0] + ".png"
            shape = write_waterfall(png, raw[0::2] + 1j * raw[1::2], a.rx_rate, sig_meta["line_s"])
            print(f"wrote {png}: receiver waterfall, {shape[1]} x {shape[0]}, newest line on top")
    return bool(strong)


def cmd_sweep(a, tx, rec):
    offsets = [int(round(x)) for x in np.arange(a.start, a.stop + a.step / 2, a.step)]
    results = {}
    for gain in a.gains:
        print(sdr_set(tx, "txgain", gain))
        rec.take()
        t0 = time.monotonic()
        time.sleep(a.ms / 1000)
        windows = [(t0 + 0.02, time.monotonic() - 0.02, "off")]
        for off in offsets:
            t0 = time.monotonic()
            tx_tone(tx, off, a.ms)
            windows.append((t0 + 0.02, time.monotonic() - 0.02, off))
            time.sleep(a.gap_ms / 1000)
        time.sleep(0.3)
        bs = [b for b in label(bursts(rec.take()), windows) if b["tx"] is not None]
        path = os.path.join(a.outdir, f"sweep_txgain{gain:02d}.cs16")
        ref = spectrum_of([b for b in bs if b["tx"] == "off"])
        row = []
        for off in offsets:
            sel = [b for b in bs if b["tx"] == off]
            s = spectrum_of(sel)
            if s is None or ref is None:
                row.append(None)
                continue
            c, pk = excess_peak(s, ref)
            clip = max(float(np.abs(np.concatenate([b["iq"] for b in sel]).view(np.float32)).max()), 0)
            p = 10 * np.log10(np.mean(np.concatenate([np.abs(b["iq"]) ** 2 for b in sel])) / 512 ** 2)
            row.append({"cmd_hz": off, "peak_hz": round(c), "peak_db": round(pk, 1),
                        "power_dbfs": round(float(p), 1), "clipped": clip >= 511, "bursts": len(sel)})
        results[gain] = row
        write_raw(path, bs, {"rate_hz": FS, "rx_lo_mhz": a.lo, "txgain": gain, "step_ms": a.ms,
                             "first_step": "tx off", "offsets_cmd_hz": offsets, "steps": row})
        print(f"txgain {gain:2d}: {len(bs)} bursts -> {path}")
        for r in row:
            if r:
                print(f"   cmd {r['cmd_hz']/1e6:+.1f} MHz -> {r['peak_hz']/1e6:+.3f} MHz, "
                      f"{r['peak_db']:+5.1f} dB, {r['power_dbfs']:+5.1f} dBFS"
                      f"{' CLIP' if r['clipped'] else ''}")
    with open(os.path.join(a.outdir, "sweep_summary.json"), "w") as f:
        json.dump(results, f, indent=1)
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--tx", default="A4CB8FD1D880")
    ap.add_argument("--rx", default="A4CB8FD1C700")
    ap.add_argument("--lo", type=int, default=2412, help="both boards' LO in MHz")
    ap.add_argument("--rx-gain", type=int, help="receiver gain index, set over VITA 49.2")
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("rxgain", help="only set --rx-gain over VITA 49.2 and query it back")
    o = sub.add_parser("onoff")
    o.add_argument("--offset", type=int, default=1_000_000)
    o.add_argument("--ms", type=int, default=500)
    o.add_argument("--reps", type=int, default=4)
    o.add_argument("--threshold", type=float, default=10.0)
    o.add_argument("--out")
    w = sub.add_parser("power", help="tone level per transmit power step")
    w.add_argument("--steps", type=int, nargs="+", default=list(range(18)))
    w.add_argument("--offset", type=int, default=1_000_000)
    w.add_argument("--amp", type=int, default=511)
    w.add_argument("--ms", type=int, default=300)
    w.add_argument("--out")
    c = sub.add_parser("chirp", help="record a linear frequency sweep (sdr sweep) and fit it")
    c.add_argument("--f0", type=int, default=-6_000_000)
    c.add_argument("--f1", type=int, default=6_000_000)
    c.add_argument("--ms", type=int, default=2000)
    c.add_argument("--txgain", type=int, default=6)
    c.add_argument("--amp", type=int, default=256)
    c.add_argument("--out")
    x = sub.add_parser("txstream", help="stream an IQ signal to the transmitter over VITA 49.2")
    x.add_argument("--signal", choices=["tone", "chirp", "image"], default="tone")
    x.add_argument("--file", help="cs16 file to send instead (little endian int16 I, Q)")
    x.add_argument("--rate", type=int, default=200_000, help="stream rate, divides 40 MS/s")
    x.add_argument("--seconds", type=float, default=2.0)
    x.add_argument("--offset", type=float, default=60_000, help="tone, or chirp centre, Hz")
    x.add_argument("--span", type=float, default=160_000, help="chirp span, Hz")
    x.add_argument("--amp", type=float, default=0.5, help="digital amplitude, full scale 1")
    x.add_argument("--txgain", type=int, default=6)
    x.add_argument("--out")
    x.add_argument("--image", help="picture to paint (--signal image), any format Pillow reads")
    x.add_argument("--line-ms", type=float, default=20.0, help="image: time per picture row")
    x.add_argument("--width", type=int, default=256, help="image: resize to this many columns")
    x.add_argument("--flip", action="store_true", help="image: top row first (time runs down)")
    l = sub.add_parser("link", help="VITA 49.2 stream to the transmitter, decimated capture at the receiver")
    l.add_argument("--signal", choices=["tone", "chirp", "image"], default="tone")
    l.add_argument("--file", help="cs16 file to send instead (little endian int16 I, Q)")
    l.add_argument("--tx-rate", type=int, default=100_000, help="transmit stream rate, divides 40 MS/s")
    l.add_argument("--rx-rate", type=int, default=200_000, help="receive stream rate, 16 MS/s / m")
    l.add_argument("--tx-bits", type=int, choices=[8, 12, 16], default=16, help="TX item size")
    l.add_argument("--rx-bits", type=int, choices=[8, 12, 16], default=16, help="RX item size")
    l.add_argument("--seconds", type=float, default=2.0)
    l.add_argument("--offset", type=float, default=20_000, help="tone, or chirp centre, Hz")
    l.add_argument("--span", type=float, default=80_000, help="chirp span, Hz")
    l.add_argument("--amp", type=float, default=0.5, help="digital amplitude, full scale 1")
    l.add_argument("--txgain", type=int, default=6)
    l.add_argument("--out")
    l.add_argument("--image", help="picture to paint (--signal image), any format Pillow reads")
    l.add_argument("--line-ms", type=float, default=20.0, help="image: time per picture row")
    l.add_argument("--width", type=int, default=256, help="image: resize to this many columns")
    l.add_argument("--flip", action="store_true", help="image: top row first (time runs down)")
    q = sub.add_parser("paint", help="image to IQ for a waterfall, written to a file (no boards)")
    q.add_argument("image", help="picture, any format Pillow reads; grey levels are used")
    q.add_argument("--out", required=True, help="cs16 output; .json and a preview .png beside it")
    q.add_argument("--rate", type=int, default=250_000, help="sample rate; the picture is rate/2 wide")
    q.add_argument("--line-ms", type=float, default=20.0, help="time per picture row")
    q.add_argument("--width", type=int, default=256, help="resize to this many columns")
    q.add_argument("--flip", action="store_true", help="top row first (time runs down)")
    q.add_argument("--offset", type=float, default=0.0, help="shift the picture by this many Hz")
    q.add_argument("--amp", type=float, default=0.5, help="peak amplitude, full scale 1")
    s = sub.add_parser("sweep")
    s.add_argument("--gains", type=int, nargs="+", default=[10, 30, 50, 70, 90])
    s.add_argument("--start", type=int, default=-2_400_000)
    s.add_argument("--stop", type=int, default=2_400_000)
    s.add_argument("--step", type=int, default=400_000)
    s.add_argument("--ms", type=int, default=400)
    s.add_argument("--gap-ms", type=int, default=100)
    s.add_argument("--outdir", default=".")
    a = ap.parse_args()
    if a.cmd == "paint":
        return 0 if cmd_paint(a) else 1

    tx_iface = iface_for_serial(a.tx)
    tx_peer = learn_peer(tx_iface) if a.cmd in ("txstream", "link") else None
    tx, rx = Board(a.tx), Board(a.rx)
    for b in (tx, rx):
        print(f"{b.serial}: " + sdr_set(b, "freq", a.lo))
    rec = Recorder(iface_for_serial(a.rx))
    rec.start()
    time.sleep(0.5)
    try:
        if a.rx_gain is not None:
            r = vita_rx_control(rec, gain=a.rx_gain)
            print(f"VITA 49.2 rx control: gain {a.rx_gain} -> executed {r['executed']}, "
                  f"errors {r['errors']}, state {r}")
            status = rx.cmd("sdr status").splitlines()
            print(f"{rx.serial} shell: " + next((l for l in status if l.startswith("rx:")), "?"))
        if a.cmd == "rxgain":
            ok = a.rx_gain is not None
        else:
            ok = {"onoff": lambda: cmd_onoff(a, tx, rec), "sweep": lambda: cmd_sweep(a, tx, rec),
                  "power": lambda: cmd_power(a, tx, rx, rec),
                  "chirp": lambda: cmd_chirp(a, tx, rx, rec),
                  "txstream": lambda: cmd_txstream(a, tx, rx, rec, tx_peer, tx_iface),
                  "link": lambda: cmd_link(a, tx, rx, rec, tx_peer, tx_iface)}[a.cmd]()
    finally:
        rec.stop()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
