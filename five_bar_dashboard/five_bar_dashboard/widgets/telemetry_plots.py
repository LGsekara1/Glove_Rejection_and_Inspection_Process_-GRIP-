"""High-rate rolling pyqtgraph telemetry plots."""
from __future__ import annotations

import math
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import QCheckBox, QDoubleSpinBox, QHBoxLayout, QLabel, QTabWidget, QVBoxLayout, QWidget

from ..models import TelemetrySample


class TelemetryPlots(QWidget):
    def __init__(self, window_s: float = 20.0, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.started = time.monotonic()
        self.window_s = window_s
        self.buffers = {
            "t": deque(maxlen=30000),
            "p0": deque(maxlen=30000),
            "p1": deque(maxlen=30000),
            "vr0": deque(maxlen=30000),
            "vr1": deque(maxlen=30000),
            "vf0": deque(maxlen=30000),
            "vf1": deque(maxlen=30000),
            "i0": deque(maxlen=30000),
            "i1": deque(maxlen=30000),
        }

        controls = QHBoxLayout()
        self.raw_overlay = QCheckBox("Overlay raw velocity")
        self.raw_overlay.setChecked(True)
        self.window_spin = QDoubleSpinBox()
        self.window_spin.setRange(2.0, 120.0)
        self.window_spin.setValue(window_s)
        self.window_spin.setSuffix(" s")
        self.window_spin.valueChanged.connect(self._set_window)
        controls.addWidget(self.raw_overlay)
        controls.addStretch(1)
        controls.addWidget(QLabel("Rolling window"))
        controls.addWidget(self.window_spin)

        self.tabs = QTabWidget()
        self.position_plot = self._make_plot("Joint position", "deg")
        self.velocity_plot = self._make_plot("Joint velocity", "deg/s")
        self.current_plot = self._make_plot("Motor current", "A")
        self.tabs.addTab(self.position_plot, "Position")
        self.tabs.addTab(self.velocity_plot, "Velocity")
        self.tabs.addTab(self.current_plot, "Current")

        self.position_plot.addLegend(offset=(8, 8))
        self.velocity_plot.addLegend(offset=(8, 8))
        self.current_plot.addLegend(offset=(8, 8))

        self.p0_curve = self.position_plot.plot(pen=pg.mkPen("#60A5FA", width=2), name="axis0")
        self.p1_curve = self.position_plot.plot(pen=pg.mkPen("#34D399", width=2), name="axis1")
        self.vr0_curve = self.velocity_plot.plot(
            pen=pg.mkPen((96, 165, 250, 95), width=1, style=Qt.PenStyle.DashLine),
            name="axis0 raw",
        )
        self.vr1_curve = self.velocity_plot.plot(
            pen=pg.mkPen((52, 211, 153, 95), width=1, style=Qt.PenStyle.DashLine),
            name="axis1 raw",
        )
        self.vf0_curve = self.velocity_plot.plot(
            pen=pg.mkPen("#93C5FD", width=3), name="axis0 filtered"
        )
        self.vf1_curve = self.velocity_plot.plot(
            pen=pg.mkPen("#6EE7B7", width=3), name="axis1 filtered"
        )
        self.i0_curve = self.current_plot.plot(pen=pg.mkPen("#F59E0B", width=2), name="axis0")
        self.i1_curve = self.current_plot.plot(pen=pg.mkPen("#F472B6", width=2), name="axis1")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(controls)
        layout.addWidget(self.tabs)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)

    @staticmethod
    def _make_plot(title: str, units: str) -> pg.PlotWidget:
        plot = pg.PlotWidget()
        plot.setBackground("#0B1220")
        plot.showGrid(x=True, y=True, alpha=0.18)
        plot.setLabel("bottom", "Time", units="s")
        plot.setLabel("left", title, units=units)
        plot.getAxis("left").setTextPen("#CBD5E1")
        plot.getAxis("bottom").setTextPen("#CBD5E1")
        return plot

    def _set_window(self, value: float) -> None:
        self.window_s = float(value)

    def set_raw_overlay(self, enabled: bool) -> None:
        self.raw_overlay.setChecked(enabled)

    def set_window(self, seconds: float) -> None:
        self.window_spin.setValue(seconds)

    def clear(self) -> None:
        for buffer in self.buffers.values():
            buffer.clear()
        self.started = time.monotonic()
        self.refresh()

    def append(self, sample: TelemetrySample) -> None:
        t = time.monotonic() - self.started
        values = {
            "t": t,
            "p0": sample.pos_deg[0],
            "p1": sample.pos_deg[1],
            "vr0": sample.vel_raw_deg_s[0],
            "vr1": sample.vel_raw_deg_s[1],
            "vf0": sample.vel_filtered_deg_s[0],
            "vf1": sample.vel_filtered_deg_s[1],
            "i0": sample.current_a[0],
            "i1": sample.current_a[1],
        }
        for key, value in values.items():
            self.buffers[key].append(float(value))

    def _visible_arrays(self) -> dict[str, np.ndarray]:
        if not self.buffers["t"]:
            return {key: np.array([]) for key in self.buffers}
        t = np.fromiter(self.buffers["t"], dtype=float)
        cutoff = t[-1] - self.window_s
        start = int(np.searchsorted(t, cutoff, side="left"))
        count = len(t) - start
        stride = max(1, math.ceil(count / 800))
        return {
            key: np.fromiter(buffer, dtype=float)[start::stride]
            for key, buffer in self.buffers.items()
        }

    def refresh(self) -> None:
        data = self._visible_arrays()
        t = data["t"]
        self.p0_curve.setData(t, data["p0"])
        self.p1_curve.setData(t, data["p1"])
        self.vf0_curve.setData(t, data["vf0"])
        self.vf1_curve.setData(t, data["vf1"])
        self.i0_curve.setData(t, data["i0"])
        self.i1_curve.setData(t, data["i1"])
        show_raw = self.raw_overlay.isChecked()
        self.vr0_curve.setVisible(show_raw)
        self.vr1_curve.setVisible(show_raw)
        if show_raw:
            self.vr0_curve.setData(t, data["vr0"])
            self.vr1_curve.setData(t, data["vr1"])
