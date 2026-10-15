#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# A small standalone spectrum/waterfall viewer for the osmosdr app, built on
# GNU Radio and the esdr SoapySDR module (see ../soapy/). One window: a
# frequency spectrum above a waterfall, with frequency, sample rate, gain and
# bandwidth controls. No GNU Radio Companion project needed.
#
# Run (see esdr_scope.sh for a launcher that finds the right Python):
#   ./esdr_scope.sh [--serial SERIAL] [--freq HZ] [--rate HZ] [--gain N|auto]

import argparse
import sys

from PyQt5 import Qt
import sip

from gnuradio import analog, gr, qtgui
from gnuradio.fft import window
import osmosdr

FFT_SIZE = 1024
RATES = [250000, 125000, 62500, 31250, 15625]


class Scope(gr.top_block, Qt.QWidget):
    def __init__(self, device_args, freq_hz, rate_hz, gain, corr_ppm):
        gr.top_block.__init__(self, "ESP-SDR scope")
        Qt.QWidget.__init__(self)

        self.rate_hz = rate_hz

        self.src = osmosdr.source(args="numchan=1 " + device_args)
        self.src.set_sample_rate(rate_hz)
        self.src.set_center_freq(freq_hz)
        self.src.set_freq_corr(corr_ppm)
        if gain is None:
            self.src.set_gain_mode(True)
        else:
            self.src.set_gain_mode(False)
            self.src.set_gain(gain)

        self.freq_sink = qtgui.freq_sink_c(
            FFT_SIZE, window.WIN_BLACKMAN_hARRIS, freq_hz, rate_hz, "spectrum", 1
        )
        self.freq_sink.set_update_time(0.05)
        self.freq_sink.set_y_axis(-80, 10)
        self.freq_sink.enable_grid(True)
        self.freq_sink.enable_autoscale(False)

        self.waterfall = qtgui.waterfall_sink_c(
            FFT_SIZE, window.WIN_BLACKMAN_hARRIS, freq_hz, rate_hz, "waterfall", 1
        )
        self.waterfall.set_update_time(0.05)
        self.waterfall.set_intensity_range(-80, 10)

        self.connect(self.src, self.freq_sink)
        self.connect(self.src, self.waterfall)

        self._build_ui(freq_hz, rate_hz, gain, corr_ppm)

    # ---- UI ----

    def _build_ui(self, freq_hz, rate_hz, gain, corr_ppm):
        self.setWindowTitle("ESP-SDR scope")
        layout = Qt.QVBoxLayout(self)

        controls = Qt.QHBoxLayout()
        layout.addLayout(controls)

        controls.addWidget(Qt.QLabel("Freq (MHz)"))
        self.freq_box = Qt.QDoubleSpinBox()
        self.freq_box.setRange(100.0, 6000.0)
        self.freq_box.setDecimals(3)
        self.freq_box.setSingleStep(0.1)
        self.freq_box.setValue(freq_hz / 1e6)
        self.freq_box.valueChanged.connect(self._on_freq)
        controls.addWidget(self.freq_box)

        controls.addWidget(Qt.QLabel("Rate"))
        self.rate_box = Qt.QComboBox()
        for r in RATES:
            self.rate_box.addItem("%g kS/s" % (r / 1e3), r)
        self.rate_box.setCurrentIndex(RATES.index(rate_hz) if rate_hz in RATES else 0)
        self.rate_box.currentIndexChanged.connect(self._on_rate)
        controls.addWidget(self.rate_box)

        controls.addWidget(Qt.QLabel("Gain"))
        self.gain_auto = Qt.QCheckBox("auto")
        self.gain_auto.setChecked(gain is None)
        self.gain_auto.toggled.connect(self._on_gain_mode)
        controls.addWidget(self.gain_auto)
        self.gain_box = Qt.QSpinBox()
        self.gain_box.setRange(0, 82)
        self.gain_box.setValue(gain if gain is not None else 32)
        self.gain_box.setEnabled(gain is not None)
        self.gain_box.valueChanged.connect(self._on_gain)
        controls.addWidget(self.gain_box)

        controls.addWidget(Qt.QLabel("Corr (ppm)"))
        self.corr_box = Qt.QDoubleSpinBox()
        self.corr_box.setRange(-100.0, 100.0)
        self.corr_box.setDecimals(3)
        self.corr_box.setValue(corr_ppm)
        self.corr_box.valueChanged.connect(self._on_corr)
        controls.addWidget(self.corr_box)

        controls.addStretch(1)

        layout.addWidget(sip.wrapinstance(self.freq_sink.qwidget(), Qt.QWidget))
        layout.addWidget(sip.wrapinstance(self.waterfall.qwidget(), Qt.QWidget))

    # ---- control callbacks (GNU Radio calls are safe from the Qt thread) ----

    def _on_freq(self, mhz):
        hz = mhz * 1e6
        self.src.set_center_freq(hz)
        self.freq_sink.set_frequency_range(hz, self.rate_hz)
        self.waterfall.set_frequency_range(hz, self.rate_hz)

    def _on_rate(self, index):
        rate = self.rate_box.itemData(index)
        self.rate_hz = rate
        self.src.set_sample_rate(rate)
        freq = self.freq_box.value() * 1e6
        self.freq_sink.set_frequency_range(freq, rate)
        self.waterfall.set_frequency_range(freq, rate)

    def _on_gain_mode(self, auto):
        self.gain_box.setEnabled(not auto)
        self.src.set_gain_mode(auto)
        if not auto:
            self.src.set_gain(self.gain_box.value())

    def _on_gain(self, value):
        if not self.gain_auto.isChecked():
            self.src.set_gain(value)

    def _on_corr(self, ppm):
        self.src.set_freq_corr(ppm)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--serial", help="board USB serial (default: first esdr device found)")
    ap.add_argument("--freq", type=float, default=2450e6, help="centre frequency in Hz")
    ap.add_argument("--rate", type=float, default=250e3, help="sample rate in Hz")
    ap.add_argument("--gain", default="auto", help="RX gain index 0..82, or 'auto'")
    ap.add_argument("--corr", type=float, default=0.0, help="frequency correction in ppm")
    args = ap.parse_args()

    device_args = "driver=esdr" + (",serial=%s" % args.serial if args.serial else "")
    gain = None if args.gain == "auto" else int(args.gain)

    app = Qt.QApplication(sys.argv)
    tb = Scope(device_args, args.freq, args.rate, gain, args.corr)
    tb.start()
    tb.show()
    tb.resize(900, 700)

    def shutdown():
        tb.stop()
        tb.wait()

    app.aboutToQuit.connect(shutdown)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
