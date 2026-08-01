"""Functional control panels used by the main dashboard window."""
from __future__ import annotations

from collections import deque
import json
from typing import Any

import pyqtgraph as pg
from PySide6.QtCore import QTimer, Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QAbstractItemView,
    QAbstractSpinBox,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextBrowser,
    QVBoxLayout,
    QWidget,
)

from .constants import ENCODER_MODE_LABELS, ENCODER_MODE_VALUES_TO_LABELS, FILTER_TYPES
from .models import (
    AxisMappingConfig,
    DashboardConfig,
    DisplayConfig,
    EncoderInterfaceConfig,
    EncoderNoiseConfig,
    GeometryConfig,
    StepResponseResult,
    TrajectoryConfig,
    VelocityControlConfig,
    VelocityFilterConfig,
)


def dspin(
    minimum: float,
    maximum: float,
    value: float = 0.0,
    decimals: int = 3,
    step: float = 0.1,
    suffix: str = "",
) -> QDoubleSpinBox:
    widget = QDoubleSpinBox()
    widget.setRange(minimum, maximum)
    widget.setDecimals(decimals)
    widget.setSingleStep(step)
    widget.setValue(value)
    widget.setSuffix(suffix)
    widget.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.UpDownArrows)
    return widget


def section_title(text: str) -> QLabel:
    label = QLabel(text)
    font = QFont()
    font.setPointSize(11)
    font.setBold(True)
    label.setFont(font)
    return label


class JointControlPanel(QWidget):
    move_requested = Signal(float, float)
    raw_nudge_requested = Signal(float, float)

    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(section_title("Firmware Trapezoidal Joint Move"))
        form = QFormLayout()
        self.theta0 = dspin(-720, 720, 90.0, 3, 1.0, " °")
        self.theta1 = dspin(-720, 720, 90.0, 3, 1.0, " °")
        form.addRow("Axis0 target angle", self.theta0)
        form.addRow("Axis1 target angle", self.theta1)
        layout.addLayout(form)
        move = QPushButton("Move Both Axes (Firmware Planner)")
        move.clicked.connect(lambda: self.move_requested.emit(self.theta0.value(), self.theta1.value()))
        layout.addWidget(move)

        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        layout.addWidget(line)
        layout.addWidget(section_title("Raw-Turn Instant Nudge"))
        warning = QLabel(
            "Bypasses trajectory planning. Limited to ±0.05 turns per axis and intended only for small manual nudges."
        )
        warning.setWordWrap(True)
        warning.setProperty("class", "warning")
        layout.addWidget(warning)
        nudge_form = QFormLayout()
        self.delta0 = dspin(-0.05, 0.05, 0.0, 5, 0.001, " turns")
        self.delta1 = dspin(-0.05, 0.05, 0.0, 5, 0.001, " turns")
        nudge_form.addRow("Axis0 increment", self.delta0)
        nudge_form.addRow("Axis1 increment", self.delta1)
        layout.addLayout(nudge_form)
        raw = QPushButton("Send Raw Instant Nudge")
        raw.clicked.connect(lambda: self.raw_nudge_requested.emit(self.delta0.value(), self.delta1.value()))
        layout.addWidget(raw)
        layout.addStretch(1)


class InverseKinematicsPanel(QWidget):
    compute_requested = Signal(float, float, str, str, bool)

    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(section_title("Inverse Kinematics: Cartesian Target → Joint Angles"))
        form = QFormLayout()
        self.x = dspin(-2000, 2000, 0.0, 2, 5.0, " mm")
        self.y = dspin(-2000, 2000, 400.0, 2, 5.0, " mm")
        self.elbow0 = QComboBox(); self.elbow0.addItems(["up", "down"])
        self.elbow1 = QComboBox(); self.elbow1.addItems(["up", "down"]); self.elbow1.setCurrentText("down")
        form.addRow("Target X", self.x)
        form.addRow("Target Y", self.y)
        form.addRow("Axis0 elbow", self.elbow0)
        form.addRow("Axis1 elbow", self.elbow1)
        layout.addLayout(form)
        buttons = QHBoxLayout()
        compute = QPushButton("Compute Only")
        move = QPushButton("Compute & Move")
        compute.clicked.connect(lambda: self._emit(False))
        move.clicked.connect(lambda: self._emit(True))
        buttons.addWidget(compute); buttons.addWidget(move)
        layout.addLayout(buttons)
        self.result = QLabel("No result yet.")
        self.result.setWordWrap(True)
        layout.addWidget(self.result)
        layout.addStretch(1)

    def _emit(self, move: bool) -> None:
        self.compute_requested.emit(
            self.x.value(), self.y.value(), self.elbow0.currentText(), self.elbow1.currentText(), move
        )

    def set_result(self, text: str, ok: bool = True) -> None:
        self.result.setText(text)
        self.result.setProperty("state", "ok" if ok else "error")
        self.result.style().unpolish(self.result); self.result.style().polish(self.result)

    def set_elbows(self, elbow0: str, elbow1: str) -> None:
        self.elbow0.setCurrentText(elbow0)
        self.elbow1.setCurrentText(elbow1)


class ForwardKinematicsPanel(QWidget):
    compute_requested = Signal(float, float)
    use_current_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(section_title("Forward Kinematics: Joint Angles → Cartesian Pose"))
        form = QFormLayout()
        self.theta0 = dspin(-720, 720, 90.0, 3, 1.0, " °")
        self.theta1 = dspin(-720, 720, 90.0, 3, 1.0, " °")
        form.addRow("Theta 1", self.theta0)
        form.addRow("Theta 2", self.theta1)
        layout.addLayout(form)
        buttons = QHBoxLayout()
        compute = QPushButton("Compute FK")
        current = QPushButton("Use Current Motor Angles")
        compute.clicked.connect(lambda: self.compute_requested.emit(self.theta0.value(), self.theta1.value()))
        current.clicked.connect(self.use_current_requested)
        buttons.addWidget(compute); buttons.addWidget(current)
        layout.addLayout(buttons)
        self.result = QLabel("No result yet.")
        self.result.setWordWrap(True)
        layout.addWidget(self.result)
        layout.addStretch(1)

    def set_angles(self, theta0: float, theta1: float) -> None:
        self.theta0.setValue(theta0); self.theta1.setValue(theta1)

    def set_result(self, text: str, ok: bool = True) -> None:
        self.result.setText(text)
        self.result.setProperty("state", "ok" if ok else "error")
        self.result.style().unpolish(self.result); self.result.style().polish(self.result)


class VelocityControlPanel(QWidget):
    start_requested = Signal(str, object)
    stop_requested = Signal()
    jog_command = Signal(float, float)
    overlay_changed = Signal(bool)

    def __init__(self) -> None:
        super().__init__()
        self.held_vector: tuple[float, float] | None = None
        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(100)
        self.refresh_timer.timeout.connect(self._refresh_command)

        layout = QVBoxLayout(self)
        layout.addWidget(section_title("Jacobian-Resolved Cartesian Velocity Control"))
        self.mode = QComboBox()
        self.mode.addItems(["Cartesian velocity jog", "PC-side Cartesian position control"])
        layout.addWidget(self.mode)
        self.stack = QStackedWidget()
        self.stack.addWidget(self._build_jog_page())
        self.stack.addWidget(self._build_position_page())
        self.mode.currentIndexChanged.connect(self.stack.setCurrentIndex)
        layout.addWidget(self.stack)

        controls = QHBoxLayout()
        self.start_button = QPushButton("Start Velocity Session")
        self.stop_button = QPushButton("Stop Session")
        self.stop_button.setEnabled(False)
        self.start_button.clicked.connect(self._start)
        self.stop_button.clicked.connect(lambda: self.stop_requested.emit())
        controls.addWidget(self.start_button); controls.addWidget(self.stop_button)
        layout.addLayout(controls)
        self.overlay = QCheckBox("Overlay raw velocity on filtered chart")
        self.overlay.setChecked(True)
        self.overlay.toggled.connect(self.overlay_changed)
        layout.addWidget(self.overlay)
        self.filter_status = QLabel("DSP filters: loaded from Config panel")
        layout.addWidget(self.filter_status)
        self.status = QLabel("Inactive")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        layout.addStretch(1)

    def _build_jog_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        form = QFormLayout()
        self.jog_speed = dspin(0.1, 500.0, 25.0, 2, 5.0, " mm/s")
        form.addRow("D-pad speed", self.jog_speed)
        layout.addLayout(form)
        grid = QGridLayout()
        up = QPushButton("+Y")
        down = QPushButton("−Y")
        left = QPushButton("−X")
        right = QPushButton("+X")
        grid.addWidget(up, 0, 1); grid.addWidget(left, 1, 0); grid.addWidget(right, 1, 2); grid.addWidget(down, 2, 1)
        self._wire_hold(up, lambda: (0.0, self.jog_speed.value()))
        self._wire_hold(down, lambda: (0.0, -self.jog_speed.value()))
        self._wire_hold(left, lambda: (-self.jog_speed.value(), 0.0))
        self._wire_hold(right, lambda: (self.jog_speed.value(), 0.0))
        layout.addLayout(grid)
        explicit = QGroupBox("Typed explicit vector")
        explicit_form = QFormLayout(explicit)
        self.explicit_vx = dspin(-500, 500, 0.0, 2, 1.0, " mm/s")
        self.explicit_vy = dspin(-500, 500, 0.0, 2, 1.0, " mm/s")
        self.typed_toggle = QPushButton("Hold Typed Vector")
        self.typed_toggle.setCheckable(True)
        self.typed_toggle.toggled.connect(self._typed_toggled)
        explicit_form.addRow("Vx", self.explicit_vx); explicit_form.addRow("Vy", self.explicit_vy)
        explicit_form.addRow(self.typed_toggle)
        layout.addWidget(explicit)
        return page

    def _build_position_page(self) -> QWidget:
        page = QWidget()
        form = QFormLayout(page)
        self.target_x = dspin(-2000, 2000, 0.0, 2, 5.0, " mm")
        self.target_y = dspin(-2000, 2000, 400.0, 2, 5.0, " mm")
        form.addRow("Target X", self.target_x); form.addRow("Target Y", self.target_y)
        return page

    def _wire_hold(self, button: QPushButton, provider) -> None:
        button.pressed.connect(lambda: self._begin_hold(provider()))
        button.released.connect(self._end_hold)

    def _begin_hold(self, vector: tuple[float, float]) -> None:
        self.held_vector = vector
        self.jog_command.emit(*vector)
        self.refresh_timer.start()

    def _end_hold(self) -> None:
        self.held_vector = None
        if not self.typed_toggle.isChecked():
            self.refresh_timer.stop()
            self.jog_command.emit(0.0, 0.0)

    def _typed_toggled(self, checked: bool) -> None:
        if checked:
            self.held_vector = (self.explicit_vx.value(), self.explicit_vy.value())
            self.refresh_timer.start()
            self._refresh_command()
        else:
            self.held_vector = None
            self.refresh_timer.stop()
            self.jog_command.emit(0.0, 0.0)

    def _refresh_command(self) -> None:
        if self.typed_toggle.isChecked():
            self.held_vector = (self.explicit_vx.value(), self.explicit_vy.value())
        if self.held_vector is not None:
            self.jog_command.emit(*self.held_vector)

    def _start(self) -> None:
        if self.mode.currentIndex() == 0:
            self.start_requested.emit("jog", None)
        else:
            self.start_requested.emit("position", (self.target_x.value(), self.target_y.value()))

    def set_running(self, running: bool) -> None:
        self.start_button.setEnabled(not running)
        self.stop_button.setEnabled(running)
        self.mode.setEnabled(not running)
        if not running:
            self.typed_toggle.setChecked(False)
            self._end_hold()

    def set_status(self, text: str, severity: str = "info") -> None:
        self.status.setText(text)
        self.status.setProperty("state", severity)
        self.status.style().unpolish(self.status); self.status.style().polish(self.status)

    def set_filter_status(self, text: str) -> None:
        self.filter_status.setText(text)


class CalibrationPanel(QWidget):
    action_requested = Signal(str)
    calibrate_requested = Signal(object)
    sync_zero_requested = Signal(object)
    flash_requested = Signal(str, object)

    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(section_title("Calibration / Software Homing"))
        row = QGridLayout()
        buttons = {
            "Clear Errors": "clear_errors",
            "Enable Closed Loop (Both)": "enable_closed_loop",
            "Idle Both Axes": "idle",
            "Show Errors": "show_errors",
        }
        for index, (text, action) in enumerate(buttons.items()):
            button = QPushButton(text)
            button.clicked.connect(lambda checked=False, a=action: self.action_requested.emit(a))
            row.addWidget(button, index // 2, index % 2)
        layout.addLayout(row)

        cal_group = QGroupBox("Motor and encoder calibration")
        cal_layout = QHBoxLayout(cal_group)
        for text, axes in (("Calibrate Axis0", (0,)), ("Calibrate Axis1", (1,)), ("Calibrate Both", (0, 1))):
            button = QPushButton(text)
            button.clicked.connect(lambda checked=False, a=axes: self.calibrate_requested.emit(a))
            cal_layout.addWidget(button)
        layout.addWidget(cal_group)

        sync_group = QGroupBox("Software reference from known physical pose")
        sync_layout = QFormLayout(sync_group)
        self.reference_angles: dict[int, QDoubleSpinBox] = {
            0: dspin(-100000, 100000, 90.0, 6, 1.0, " °"),
            1: dspin(-100000, 100000, 90.0, 6, 1.0, " °"),
        }
        sync_layout.addRow("Current physical angle axis0", self.reference_angles[0])
        sync_layout.addRow("Current physical angle axis1", self.reference_angles[1])
        sync = QPushButton("Sync Software Offsets")
        sync.setToolTip(
            "Reads each absolute encoder and changes only dashboard offset_turns so the "
            "live angles equal the known physical angles entered above. It does not move the robot."
        )
        sync.clicked.connect(
            lambda: self.sync_zero_requested.emit(
                {0: self.reference_angles[0].value(), 1: self.reference_angles[1].value()}
            )
        )
        sync_layout.addRow(sync)
        note = QLabel(
            "Keep the robot stationary. The sync now samples many encoder positions and uses a "
            "robust median. Raw encoder velocity noise is ignored; sustained position drift still blocks sync. "
            "Gear ratio and direction must already be correct."
        )
        note.setWordWrap(True); note.setProperty("class", "warning")
        sync_layout.addRow(note)
        layout.addWidget(sync_group)

        flags_group = QGroupBox("ODrive flash-persisted calibration")
        flags_layout = QVBoxLayout(flags_group)
        read_flags = QPushButton("Read Startup/Calibration Flags")
        read_flags.clicked.connect(lambda: self.action_requested.emit("read_flags"))
        flags_layout.addWidget(read_flags)
        self.flags_display = QPlainTextEdit()
        self.flags_display.setReadOnly(True)
        self.flags_display.setMaximumHeight(170)
        flags_layout.addWidget(self.flags_display)
        self.auto_closed = QCheckBox("Auto-enter closed loop control on power-up")
        self.auto_closed.setChecked(True)
        flags_layout.addWidget(self.auto_closed)
        warning = QLabel(
            "Warning: when enabled, ODrive may servo to the last input_pos immediately after boot. Physically support the arm on the first power-cycle."
        )
        warning.setWordWrap(True); warning.setProperty("class", "warning")
        flags_layout.addWidget(warning)
        mark = QPushButton("Mark Calibrated & Save (reboots)")
        clear = QPushButton("Clear Pre-Calibrated Flags (reboots)")
        mark.clicked.connect(lambda: self.flash_requested.emit("mark_calibrated", {"auto_closed_loop": self.auto_closed.isChecked()}))
        clear.clicked.connect(lambda: self.flash_requested.emit("clear_precalibrated", {}))
        flags_layout.addWidget(mark); flags_layout.addWidget(clear)
        layout.addWidget(flags_group)
        layout.addStretch(1)

    def show_flags(self, flags: dict[str, Any]) -> None:
        lines = []
        for axis, values in flags.items():
            lines.append(axis)
            for key, value in values.items():
                lines.append(f"  {key}: {value}")
        self.flags_display.setPlainText("\n".join(lines))


class PIDTuningPanel(QWidget):
    read_requested = Signal(int)
    apply_requested = Signal(int, object)
    step_requested = Signal(int, float, float)
    save_flash_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.history: deque[StepResponseResult] = deque(maxlen=4)
        layout = QVBoxLayout(self)
        top = QHBoxLayout()
        top.addWidget(QLabel("Axis"))
        self.axis = QComboBox(); self.axis.addItems(["axis0", "axis1"])
        top.addWidget(self.axis); top.addStretch(1)
        layout.addLayout(top)

        gain_group = QGroupBox("Controller and motor settings")
        grid = QGridLayout(gain_group)
        grid.addWidget(QLabel("Field"), 0, 0); grid.addWidget(QLabel("Value"), 0, 1)
        grid.addWidget(QLabel("Nudge"), 0, 2)
        self.fields: dict[str, QDoubleSpinBox] = {
            "pos_gain": dspin(0, 100000, 20.0, 6, 0.1),
            "vel_gain": dspin(0, 100000, 0.16, 7, 0.01),
            "vel_integrator_gain": dspin(0, 100000, 0.32, 7, 0.01),
            "vel_limit": dspin(0.001, 10000, 10.0, 4, 0.1, " turns/s"),
            "current_lim": dspin(0.0, 500.0, 20.0, 3, 0.5, " A"),
        }
        labels = {
            "pos_gain": "pos_gain",
            "vel_gain": "vel_gain",
            "vel_integrator_gain": "vel_integrator_gain",
            "vel_limit": "vel_limit",
            "current_lim": "current_lim",
        }
        for row, key in enumerate(self.fields, start=1):
            grid.addWidget(QLabel(labels[key]), row, 0)
            grid.addWidget(self.fields[key], row, 1)
            if key in {"pos_gain", "vel_gain", "vel_integrator_gain"}:
                box = QHBoxLayout()
                minus = QPushButton("−20%"); plus = QPushButton("+25%")
                minus.clicked.connect(lambda checked=False, k=key: self._scale(k, 0.8))
                plus.clicked.connect(lambda checked=False, k=key: self._scale(k, 1.25))
                box.addWidget(minus); box.addWidget(plus)
                grid.addLayout(box, row, 2)
        buttons = QHBoxLayout()
        read = QPushButton("Read From Axis")
        apply = QPushButton("Apply in RAM")
        read.clicked.connect(lambda: self.read_requested.emit(self.axis.currentIndex()))
        apply.clicked.connect(lambda: self.apply_requested.emit(self.axis.currentIndex(), self.values()))
        buttons.addWidget(read); buttons.addWidget(apply)
        grid.addLayout(buttons, len(self.fields) + 1, 0, 1, 3)
        layout.addWidget(gain_group)

        step_group = QGroupBox("Step-response test")
        step_layout = QVBoxLayout(step_group)
        step_form = QFormLayout()
        self.step_size = dspin(-2.0, 2.0, 0.05, 5, 0.01, " turns")
        self.duration = dspin(0.2, 20.0, 1.5, 2, 0.1, " s")
        step_form.addRow("Step size", self.step_size); step_form.addRow("Sample duration", self.duration)
        step_layout.addLayout(step_form)
        step_buttons = QHBoxLayout()
        run = QPushButton("Run Step Test")
        clear = QPushButton("Clear History")
        run.clicked.connect(lambda: self.step_requested.emit(self.axis.currentIndex(), self.step_size.value(), self.duration.value()))
        clear.clicked.connect(self.clear_history)
        step_buttons.addWidget(run); step_buttons.addWidget(clear)
        step_layout.addLayout(step_buttons)
        self.metrics = QLabel("No step response recorded.")
        step_layout.addWidget(self.metrics)
        self.step_plot = pg.PlotWidget()
        self.step_plot.setBackground("#0B1220")
        self.step_plot.showGrid(x=True, y=True, alpha=0.18)
        self.step_plot.setLabel("bottom", "Time", units="s")
        self.step_plot.setLabel("left", "Position", units="turns")
        self.legend = self.step_plot.addLegend()
        self.step_plot.setMinimumHeight(240)
        step_layout.addWidget(self.step_plot)
        layout.addWidget(step_group)
        save = QPushButton("Save Config to Flash (reboots ODrive)")
        save.clicked.connect(self.save_flash_requested)
        layout.addWidget(save)

    def _scale(self, key: str, factor: float) -> None:
        self.fields[key].setValue(self.fields[key].value() * factor)

    def values(self) -> dict[str, float]:
        return {key: field.value() for key, field in self.fields.items()}

    def set_values(self, values: dict[str, float]) -> None:
        for key, value in values.items():
            if key in self.fields:
                self.fields[key].setValue(float(value))

    def add_step_result(self, result: StepResponseResult) -> None:
        self.history.append(result)
        settling = "not settled" if result.settling_time_s is None else f"{result.settling_time_s:.3f} s"
        self.metrics.setText(
            f"Latest: overshoot {result.overshoot_pct:.2f}% | 2% settling {settling} | "
            f"final error {result.final_error_turns:+.6f} turns"
        )
        self._redraw_history()

    def clear_history(self) -> None:
        self.history.clear(); self.metrics.setText("No step response recorded."); self._redraw_history()

    def _redraw_history(self) -> None:
        self.step_plot.clear()
        try:
            self.legend.clear()
        except Exception:
            pass
        colours = ["#60A5FA", "#34D399", "#F59E0B", "#F472B6"]
        for index, run in enumerate(self.history):
            x = [sample[0] for sample in run.samples]
            y = [sample[1] for sample in run.samples]
            gains = run.gains
            name = (
                f"pos={gains['pos_gain']:.3f} vel={gains['vel_gain']:.5f} "
                f"vel_i={gains['vel_integrator_gain']:.5f}"
            )
            self.step_plot.plot(x, y, pen=pg.mkPen(colours[index], width=2), name=name)
        if self.history:
            latest = self.history[-1]
            line = pg.InfiniteLine(
                pos=latest.target_turns,
                angle=0,
                pen=pg.mkPen("#E5E7EB", width=1.5, style=Qt.PenStyle.DashLine),
                label="latest target",
            )
            self.step_plot.addItem(line)

class CoordinateSequencePanel(QWidget):
    """Compact waypoint editor with per-point trapezoid and feedforward settings."""

    run_requested = Signal(object, int, bool)
    stop_requested = Signal()
    preview_requested = Signal(object)
    global_profile_requested = Signal(float, float, float)

    COL_INDEX = 0
    COL_X = 1
    COL_Y = 2
    COL_DWELL = 3
    COL_VMAX = 4
    COL_ACCEL = 5
    COL_DECEL = 6
    COL_VFF0 = 7
    COL_VFF1 = 8
    COL_TFF0 = 9
    COL_TFF1 = 10
    COL_STATE = 11

    def __init__(self, trajectory: TrajectoryConfig | None = None) -> None:
        super().__init__()
        defaults = trajectory or TrajectoryConfig()
        self._updating_table = False

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)
        root.addWidget(section_title("Cartesian Coordinate Sequence"))
        note = QLabel(
            "Build waypoints in the first page, edit per-point motion/feedforward in the second, "
            "then validate and run from the third. Feedforward is disabled by default and must be "
            "explicitly armed before a sequence."
        )
        note.setWordWrap(True)
        root.addWidget(note)

        self.inner_tabs = QTabWidget()
        self.inner_tabs.setDocumentMode(True)
        root.addWidget(self.inner_tabs, 1)

        self._build_waypoints_tab(defaults)
        self._build_profile_tab(defaults)
        self._build_run_tab()
        self.update_trap_preview()

    def _build_waypoints_tab(self, defaults: TrajectoryConfig) -> None:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        toolbar = QGridLayout()
        toolbar.setHorizontalSpacing(5)
        toolbar.setVerticalSpacing(5)
        buttons = [
            ("Add", lambda: self.add_point()),
            ("Insert", self.insert_above),
            ("Duplicate", self.duplicate_selected),
            ("Delete", self.delete_selected),
            ("Up", lambda: self.move_selected(-1)),
            ("Down", lambda: self.move_selected(+1)),
            ("Clear", self.clear_points),
        ]
        self.row_edit_buttons: list[QPushButton] = []
        for index, (text, callback) in enumerate(buttons):
            button = QPushButton(text)
            button.clicked.connect(callback)
            toolbar.addWidget(button, index // 4, index % 4)
            self.row_edit_buttons.append(button)
        layout.addLayout(toolbar)

        self.table = QTableWidget(0, 12)
        self.table.setHorizontalHeaderLabels(
            [
                "#",
                "X mm",
                "Y mm",
                "Dwell s",
                "Vmax °/s",
                "Accel °/s²",
                "Decel °/s²",
                "Vel FF A0 turn/s",
                "Vel FF A1 turn/s",
                "Torque FF A0 Nm",
                "Torque FF A1 Nm",
                "State",
            ]
        )
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.table.setAlternatingRowColors(True)
        self.table.setHorizontalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table.setMinimumHeight(290)
        header = self.table.horizontalHeader()
        header.setStretchLastSection(False)
        for column in range(self.table.columnCount()):
            header.setSectionResizeMode(column, QHeaderView.ResizeMode.Interactive)
        widths = {
            self.COL_INDEX: 38,
            self.COL_X: 78,
            self.COL_Y: 78,
            self.COL_DWELL: 70,
            self.COL_VMAX: 82,
            self.COL_ACCEL: 88,
            self.COL_DECEL: 88,
            self.COL_VFF0: 108,
            self.COL_VFF1: 108,
            self.COL_TFF0: 112,
            self.COL_TFF1: 112,
            self.COL_STATE: 210,
        }
        for column, width in widths.items():
            self.table.setColumnWidth(column, width)
        self.table.itemChanged.connect(self._mark_row_dirty)
        self.table.itemSelectionChanged.connect(self._selection_changed)
        layout.addWidget(self.table, 1)

        hint = QLabel(
            "The table scrolls horizontally. For less clutter, edit full trapezoid/feedforward "
            "profiles from the Motion & Feedforward page and apply them to selected rows."
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)

        files = QHBoxLayout()
        self.load_button = QPushButton("Load JSON")
        self.save_button = QPushButton("Save JSON")
        self.load_button.clicked.connect(self.load_json)
        self.save_button.clicked.connect(self.save_json)
        files.addWidget(self.load_button)
        files.addWidget(self.save_button)
        files.addStretch(1)
        layout.addLayout(files)
        self.inner_tabs.addTab(page, "1. Waypoints")

    def _build_profile_tab(self, defaults: TrajectoryConfig) -> None:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        trap_box = QGroupBox("Trapezoid profile")
        trap_layout = QGridLayout(trap_box)
        self.profile_vel = dspin(0.01, 10000.0, defaults.max_vel_deg_s, 3, 5.0, " deg/s")
        self.profile_acc = dspin(0.01, 100000.0, defaults.max_accel_deg_s2, 3, 10.0, " deg/s²")
        self.profile_dec = dspin(0.01, 100000.0, defaults.max_decel_deg_s2, 3, 10.0, " deg/s²")
        self.preview_distance = dspin(0.001, 100000.0, 30.0, 3, 5.0, " deg")
        trap_layout.addWidget(QLabel("Maximum velocity"), 0, 0)
        trap_layout.addWidget(self.profile_vel, 0, 1)
        trap_layout.addWidget(QLabel("Acceleration"), 1, 0)
        trap_layout.addWidget(self.profile_acc, 1, 1)
        trap_layout.addWidget(QLabel("Deceleration"), 2, 0)
        trap_layout.addWidget(self.profile_dec, 2, 1)
        trap_layout.addWidget(QLabel("Preview joint distance"), 3, 0)
        trap_layout.addWidget(self.preview_distance, 3, 1)

        self.trap_plot = pg.PlotWidget()
        self.trap_plot.setMinimumHeight(180)
        self.trap_plot.setLabel("bottom", "Time", units="s")
        self.trap_plot.setLabel("left", "Joint speed", units="deg/s")
        self.trap_plot.showGrid(x=True, y=True, alpha=0.25)
        self.trap_curve = self.trap_plot.plot([], [], pen=pg.mkPen(width=2))
        trap_layout.addWidget(self.trap_plot, 4, 0, 1, 2)
        self.trap_summary = QLabel()
        self.trap_summary.setWordWrap(True)
        trap_layout.addWidget(self.trap_summary, 5, 0, 1, 2)
        for widget in (self.profile_vel, self.profile_acc, self.profile_dec, self.preview_distance):
            widget.valueChanged.connect(self.update_trap_preview)
        layout.addWidget(trap_box)

        ff_box = QGroupBox("Per-waypoint feedforward")
        ff_layout = QGridLayout(ff_box)
        self.velocity_ff0 = dspin(-50.0, 50.0, 0.0, 4, 0.01, " turn/s")
        self.velocity_ff1 = dspin(-50.0, 50.0, 0.0, 4, 0.01, " turn/s")
        self.torque_ff0 = dspin(-50.0, 50.0, 0.0, 4, 0.01, " Nm")
        self.torque_ff1 = dspin(-50.0, 50.0, 0.0, 4, 0.01, " Nm")
        ff_layout.addWidget(QLabel("Axis0 velocity FF"), 0, 0)
        ff_layout.addWidget(self.velocity_ff0, 0, 1)
        ff_layout.addWidget(QLabel("Axis1 velocity FF"), 1, 0)
        ff_layout.addWidget(self.velocity_ff1, 1, 1)
        ff_layout.addWidget(QLabel("Axis0 torque FF"), 2, 0)
        ff_layout.addWidget(self.torque_ff0, 2, 1)
        ff_layout.addWidget(QLabel("Axis1 torque FF"), 3, 0)
        ff_layout.addWidget(self.torque_ff1, 3, 1)
        warning = QLabel(
            "Velocity feedforward is applied only during the planned moving phase and is zeroed "
            "before settling. Torque feedforward remains active through the waypoint dwell and is "
            "cleared at sequence completion, stop or fault. Non-zero torque is clamped from the "
            "ODrive current limit and motor torque constant when those fields are available."
        )
        warning.setWordWrap(True)
        warning.setProperty("class", "warning")
        ff_layout.addWidget(warning, 4, 0, 1, 2)
        layout.addWidget(ff_box)

        action_box = QGroupBox("Apply editor values")
        actions = QGridLayout(action_box)
        self.load_selected_profile_button = QPushButton("Load Selected Row")
        self.apply_selected_profile_button = QPushButton("Apply Trap to Selected")
        self.apply_all_profile_button = QPushButton("Apply Trap to All")
        self.apply_selected_ff_button = QPushButton("Apply Feedforward to Selected")
        self.apply_all_ff_button = QPushButton("Apply Feedforward to All")
        self.set_global_profile_button = QPushButton("Set Trap as Global Move Defaults")
        self.load_selected_profile_button.clicked.connect(self.load_selected_profile)
        self.apply_selected_profile_button.clicked.connect(self.apply_profile_to_selected)
        self.apply_all_profile_button.clicked.connect(self.apply_profile_to_all)
        self.apply_selected_ff_button.clicked.connect(self.apply_feedforward_to_selected)
        self.apply_all_ff_button.clicked.connect(self.apply_feedforward_to_all)
        self.set_global_profile_button.clicked.connect(
            lambda: self.global_profile_requested.emit(*self._current_profile())
        )
        actions.addWidget(self.load_selected_profile_button, 0, 0, 1, 2)
        actions.addWidget(self.apply_selected_profile_button, 1, 0)
        actions.addWidget(self.apply_all_profile_button, 1, 1)
        actions.addWidget(self.apply_selected_ff_button, 2, 0)
        actions.addWidget(self.apply_all_ff_button, 2, 1)
        actions.addWidget(self.set_global_profile_button, 3, 0, 1, 2)
        layout.addWidget(action_box)
        layout.addStretch(1)
        scroll.setWidget(content)
        self.inner_tabs.addTab(scroll, "2. Motion & Feedforward")

    def _build_run_tab(self) -> None:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(10)

        settings = QGroupBox("Sequence settings")
        form = QFormLayout(settings)
        self.repeat_count = QSpinBox()
        self.repeat_count.setRange(1, 1000)
        self.repeat_count.setValue(1)
        self.repeat_count.setToolTip("Number of complete passes through the waypoint list")
        self.feedforward_enabled = QCheckBox("Arm per-waypoint feedforward")
        self.feedforward_enabled.setChecked(False)
        self.feedforward_enabled.setToolTip(
            "When unchecked, every stored feedforward value is ignored and zero is commanded."
        )
        form.addRow("Repeat sequence", self.repeat_count)
        form.addRow("Feedforward", self.feedforward_enabled)
        layout.addWidget(settings)

        safety = QLabel(
            "Feedforward is an additive command, not a safety controller. Begin with zero, then use "
            "very small values under conservative current/velocity limits. Saved JSON may contain "
            "non-zero values, but they will not be applied unless the checkbox above is armed."
        )
        safety.setWordWrap(True)
        safety.setProperty("class", "warning")
        layout.addWidget(safety)

        controls = QGridLayout()
        self.preview_button = QPushButton("Validate + Preview")
        self.run_button = QPushButton("Run Sequence")
        self.stop_button = QPushButton("Stop Sequence")
        self.stop_button.setEnabled(False)
        self.preview_button.clicked.connect(self._request_preview)
        self.run_button.clicked.connect(self._request_run)
        self.stop_button.clicked.connect(lambda: self.stop_requested.emit())
        controls.addWidget(self.preview_button, 0, 0)
        controls.addWidget(self.run_button, 0, 1)
        controls.addWidget(self.stop_button, 1, 0, 1, 2)
        layout.addLayout(controls)

        self.progress = QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        self.progress.setFormat("No sequence running")
        layout.addWidget(self.progress)
        self.status = QLabel("Add coordinates, edit profiles, then validate before running.")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        layout.addStretch(1)
        self.inner_tabs.addTab(page, "3. Validate & Run")

    def _selection_changed(self) -> None:
        if self.table.currentRow() >= 0:
            self.status.setText(
                f"Selected point {self.table.currentRow() + 1}. Use Motion & Feedforward to load or apply its settings."
            )

    def set_profile_defaults(self, trajectory: TrajectoryConfig) -> None:
        self.profile_vel.setValue(float(trajectory.max_vel_deg_s))
        self.profile_acc.setValue(float(trajectory.max_accel_deg_s2))
        self.profile_dec.setValue(float(trajectory.max_decel_deg_s2))

    def update_trap_preview(self, *_: object) -> None:
        try:
            from .trajectory import compute_trap_profile, profile_velocity_samples

            profile = compute_trap_profile(
                self.preview_distance.value(),
                self.profile_vel.value(),
                self.profile_acc.value(),
                self.profile_dec.value(),
            )
            x, y = profile_velocity_samples(profile)
            self.trap_curve.setData(x, y)
            shape = "Triangular" if profile.triangular else "Trapezoidal"
            self.trap_summary.setText(
                f"{shape}: peak={profile.peak_velocity:.3f} deg/s, "
                f"t_acc={profile.accel_time:.3f} s, t_cruise={profile.cruise_time:.3f} s, "
                f"t_dec={profile.decel_time:.3f} s, total={profile.total_time:.3f} s. "
                "Actual per-axis values may be reduced by firmware caps and synchronisation."
            )
        except Exception as exc:
            self.trap_curve.setData([], [])
            self.trap_summary.setText(f"Profile preview unavailable: {exc}")

    @staticmethod
    def _editable_item(text: str) -> QTableWidgetItem:
        item = QTableWidgetItem(text)
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        return item

    @staticmethod
    def _readonly_item(text: str) -> QTableWidgetItem:
        item = QTableWidgetItem(text)
        item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        return item

    def _current_profile(self) -> tuple[float, float, float]:
        return self.profile_vel.value(), self.profile_acc.value(), self.profile_dec.value()

    def _current_feedforward(self) -> tuple[float, float, float, float]:
        return (
            self.velocity_ff0.value(),
            self.velocity_ff1.value(),
            self.torque_ff0.value(),
            self.torque_ff1.value(),
        )

    def add_point(
        self,
        x: float = 0.0,
        y: float = 400.0,
        dwell_s: float = 0.0,
        max_vel_deg_s: float | None = None,
        max_accel_deg_s2: float | None = None,
        max_decel_deg_s2: float | None = None,
        velocity_ff0_turns_s: float | None = None,
        velocity_ff1_turns_s: float | None = None,
        torque_ff0_nm: float | None = None,
        torque_ff1_nm: float | None = None,
    ) -> None:
        trap = self._current_profile()
        ff = self._current_feedforward()
        row = self.table.rowCount()
        self.table.insertRow(row)
        self._set_row(
            row,
            x,
            y,
            dwell_s,
            trap[0] if max_vel_deg_s is None else max_vel_deg_s,
            trap[1] if max_accel_deg_s2 is None else max_accel_deg_s2,
            trap[2] if max_decel_deg_s2 is None else max_decel_deg_s2,
            ff[0] if velocity_ff0_turns_s is None else velocity_ff0_turns_s,
            ff[1] if velocity_ff1_turns_s is None else velocity_ff1_turns_s,
            ff[2] if torque_ff0_nm is None else torque_ff0_nm,
            ff[3] if torque_ff1_nm is None else torque_ff1_nm,
            "Not validated",
        )
        self.table.selectRow(row)

    def _set_row(
        self,
        row: int,
        x: float,
        y: float,
        dwell_s: float,
        vmax: float,
        accel: float,
        decel: float,
        vel_ff0: float,
        vel_ff1: float,
        torque_ff0: float,
        torque_ff1: float,
        state: str,
    ) -> None:
        self._updating_table = True
        try:
            values = {
                self.COL_INDEX: self._readonly_item(str(row + 1)),
                self.COL_X: self._editable_item(f"{float(x):.4f}"),
                self.COL_Y: self._editable_item(f"{float(y):.4f}"),
                self.COL_DWELL: self._editable_item(f"{float(dwell_s):.3f}"),
                self.COL_VMAX: self._editable_item(f"{float(vmax):.3f}"),
                self.COL_ACCEL: self._editable_item(f"{float(accel):.3f}"),
                self.COL_DECEL: self._editable_item(f"{float(decel):.3f}"),
                self.COL_VFF0: self._editable_item(f"{float(vel_ff0):.5f}"),
                self.COL_VFF1: self._editable_item(f"{float(vel_ff1):.5f}"),
                self.COL_TFF0: self._editable_item(f"{float(torque_ff0):.5f}"),
                self.COL_TFF1: self._editable_item(f"{float(torque_ff1):.5f}"),
                self.COL_STATE: self._readonly_item(state),
            }
            for column, item in values.items():
                self.table.setItem(row, column, item)
        finally:
            self._updating_table = False

    def _mark_row_dirty(self, item: QTableWidgetItem) -> None:
        if self._updating_table or item.column() in (self.COL_INDEX, self.COL_STATE):
            return
        self._updating_table = True
        try:
            self.table.setItem(item.row(), self.COL_STATE, self._readonly_item("Not validated"))
        finally:
            self._updating_table = False

    def _renumber(self) -> None:
        self._updating_table = True
        try:
            for row in range(self.table.rowCount()):
                self.table.setItem(row, self.COL_INDEX, self._readonly_item(str(row + 1)))
        finally:
            self._updating_table = False

    def _row_values(
        self, row: int
    ) -> tuple[float, float, float, float, float, float, float, float, float, float, str]:
        try:
            x = float(self.table.item(row, self.COL_X).text())
            y = float(self.table.item(row, self.COL_Y).text())
            dwell = float(self.table.item(row, self.COL_DWELL).text())
            vmax = float(self.table.item(row, self.COL_VMAX).text())
            accel = float(self.table.item(row, self.COL_ACCEL).text())
            decel = float(self.table.item(row, self.COL_DECEL).text())
            vel_ff0 = float(self.table.item(row, self.COL_VFF0).text())
            vel_ff1 = float(self.table.item(row, self.COL_VFF1).text())
            torque_ff0 = float(self.table.item(row, self.COL_TFF0).text())
            torque_ff1 = float(self.table.item(row, self.COL_TFF1).text())
        except Exception as exc:
            raise ValueError(
                f"Point {row + 1}: coordinate, dwell, trapezoid and feedforward cells must be numeric."
            ) from exc
        if dwell < 0:
            raise ValueError(f"Point {row + 1}: dwell cannot be negative.")
        if min(vmax, accel, decel) <= 0:
            raise ValueError(f"Point {row + 1}: Vmax, acceleration and deceleration must be positive.")
        state_item = self.table.item(row, self.COL_STATE)
        state = state_item.text() if state_item else ""
        return x, y, dwell, vmax, accel, decel, vel_ff0, vel_ff1, torque_ff0, torque_ff1, state

    def points(self) -> list[dict[str, float]]:
        if self.table.rowCount() == 0:
            raise ValueError("Add at least one coordinate before validating or running.")
        output: list[dict[str, float]] = []
        for row in range(self.table.rowCount()):
            values = self._row_values(row)
            output.append(
                {
                    "x_mm": values[0],
                    "y_mm": values[1],
                    "dwell_s": values[2],
                    "max_vel_deg_s": values[3],
                    "max_accel_deg_s2": values[4],
                    "max_decel_deg_s2": values[5],
                    "velocity_ff0_turns_s": values[6],
                    "velocity_ff1_turns_s": values[7],
                    "torque_ff0_nm": values[8],
                    "torque_ff1_nm": values[9],
                }
            )
        return output

    def selected_rows(self) -> list[int]:
        return sorted({index.row() for index in self.table.selectionModel().selectedRows()})

    def _require_selected_rows(self) -> list[int]:
        rows = self.selected_rows()
        if not rows:
            QMessageBox.information(self, "No rows selected", "Select one or more waypoint rows first.")
        return rows

    def load_selected_profile(self) -> None:
        row = self.table.currentRow()
        if row < 0:
            QMessageBox.information(self, "No row selected", "Select a waypoint row first.")
            return
        try:
            values = self._row_values(row)
            self.profile_vel.setValue(values[3])
            self.profile_acc.setValue(values[4])
            self.profile_dec.setValue(values[5])
            self.velocity_ff0.setValue(values[6])
            self.velocity_ff1.setValue(values[7])
            self.torque_ff0.setValue(values[8])
            self.torque_ff1.setValue(values[9])
            self.status.setText(f"Loaded trap and feedforward settings from point {row + 1}.")
        except Exception as exc:
            QMessageBox.warning(self, "Invalid row", str(exc))

    def apply_profile_to_selected(self) -> None:
        rows = self._require_selected_rows()
        if not rows:
            return
        vmax, accel, decel = self._current_profile()
        self._updating_table = True
        try:
            for row in rows:
                self.table.setItem(row, self.COL_VMAX, self._editable_item(f"{vmax:.3f}"))
                self.table.setItem(row, self.COL_ACCEL, self._editable_item(f"{accel:.3f}"))
                self.table.setItem(row, self.COL_DECEL, self._editable_item(f"{decel:.3f}"))
                self.table.setItem(row, self.COL_STATE, self._readonly_item("Not validated"))
        finally:
            self._updating_table = False
        self.status.setText(f"Applied trapezoid profile to {len(rows)} selected row(s).")

    def apply_profile_to_all(self) -> None:
        if self.table.rowCount() == 0:
            return
        vmax, accel, decel = self._current_profile()
        self._updating_table = True
        try:
            for row in range(self.table.rowCount()):
                self.table.setItem(row, self.COL_VMAX, self._editable_item(f"{vmax:.3f}"))
                self.table.setItem(row, self.COL_ACCEL, self._editable_item(f"{accel:.3f}"))
                self.table.setItem(row, self.COL_DECEL, self._editable_item(f"{decel:.3f}"))
                self.table.setItem(row, self.COL_STATE, self._readonly_item("Not validated"))
        finally:
            self._updating_table = False
        self.status.setText(f"Applied trapezoid profile to all {self.table.rowCount()} row(s).")

    def apply_feedforward_to_selected(self) -> None:
        rows = self._require_selected_rows()
        if not rows:
            return
        vel_ff0, vel_ff1, torque_ff0, torque_ff1 = self._current_feedforward()
        self._updating_table = True
        try:
            for row in rows:
                self.table.setItem(row, self.COL_VFF0, self._editable_item(f"{vel_ff0:.5f}"))
                self.table.setItem(row, self.COL_VFF1, self._editable_item(f"{vel_ff1:.5f}"))
                self.table.setItem(row, self.COL_TFF0, self._editable_item(f"{torque_ff0:.5f}"))
                self.table.setItem(row, self.COL_TFF1, self._editable_item(f"{torque_ff1:.5f}"))
                self.table.setItem(row, self.COL_STATE, self._readonly_item("Not validated"))
        finally:
            self._updating_table = False
        self.status.setText(f"Applied feedforward to {len(rows)} selected row(s).")

    def apply_feedforward_to_all(self) -> None:
        if self.table.rowCount() == 0:
            return
        vel_ff0, vel_ff1, torque_ff0, torque_ff1 = self._current_feedforward()
        self._updating_table = True
        try:
            for row in range(self.table.rowCount()):
                self.table.setItem(row, self.COL_VFF0, self._editable_item(f"{vel_ff0:.5f}"))
                self.table.setItem(row, self.COL_VFF1, self._editable_item(f"{vel_ff1:.5f}"))
                self.table.setItem(row, self.COL_TFF0, self._editable_item(f"{torque_ff0:.5f}"))
                self.table.setItem(row, self.COL_TFF1, self._editable_item(f"{torque_ff1:.5f}"))
                self.table.setItem(row, self.COL_STATE, self._readonly_item("Not validated"))
        finally:
            self._updating_table = False
        self.status.setText(f"Applied feedforward to all {self.table.rowCount()} row(s).")

    def insert_above(self) -> None:
        row = self.table.currentRow()
        if row < 0:
            row = self.table.rowCount()
        self.table.insertRow(row)
        self._set_row(row, 0.0, 400.0, 0.0, *self._current_profile(), *self._current_feedforward(), "Not validated")
        self._renumber()
        self.table.selectRow(row)

    def duplicate_selected(self) -> None:
        row = self.table.currentRow()
        if row < 0:
            return
        values = self._row_values(row)
        self.table.insertRow(row + 1)
        self._set_row(row + 1, *values)
        self._renumber()
        self.table.selectRow(row + 1)

    def delete_selected(self) -> None:
        for row in sorted(self.selected_rows(), reverse=True):
            self.table.removeRow(row)
        self._renumber()

    def move_selected(self, direction: int) -> None:
        row = self.table.currentRow()
        target = row + direction
        if row < 0 or target < 0 or target >= self.table.rowCount():
            return
        current = self._row_values(row)
        other = self._row_values(target)
        self._set_row(row, *other)
        self._set_row(target, *current)
        self._renumber()
        self.table.selectRow(target)

    def clear_points(self) -> None:
        if self.table.rowCount() and QMessageBox.question(
            self, "Clear coordinate list", "Remove every coordinate from the sequence?"
        ) != QMessageBox.StandardButton.Yes:
            return
        self.table.setRowCount(0)
        self.status.setText("Coordinate list cleared.")

    def _request_preview(self) -> None:
        try:
            points = self.points()
        except Exception as exc:
            self.set_validation_error(str(exc))
            self.inner_tabs.setCurrentIndex(2)
            return
        self.preview_requested.emit(points)

    def _request_run(self) -> None:
        try:
            points = self.points()
        except Exception as exc:
            self.set_validation_error(str(exc))
            return
        if self.feedforward_enabled.isChecked():
            nonzero = any(
                abs(point[key]) > 0.0
                for point in points
                for key in (
                    "velocity_ff0_turns_s",
                    "velocity_ff1_turns_s",
                    "torque_ff0_nm",
                    "torque_ff1_nm",
                )
            )
            if nonzero:
                answer = QMessageBox.question(
                    self,
                    "Arm feedforward?",
                    "This sequence contains non-zero additive feedforward values. Continue with feedforward armed?",
                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                    QMessageBox.StandardButton.No,
                )
                if answer != QMessageBox.StandardButton.Yes:
                    return
        self.run_requested.emit(points, self.repeat_count.value(), self.feedforward_enabled.isChecked())

    def set_validation(self, compiled: object) -> None:
        points = list(compiled) if compiled is not None else []
        self._updating_table = True
        try:
            for row, point in enumerate(points):
                theta0 = float(getattr(point, "theta0_deg"))
                theta1 = float(getattr(point, "theta1_deg"))
                ff = (
                    float(getattr(point, "velocity_ff0_turns_s")),
                    float(getattr(point, "velocity_ff1_turns_s")),
                    float(getattr(point, "torque_ff0_nm")),
                    float(getattr(point, "torque_ff1_nm")),
                )
                ff_text = "FF=0" if all(abs(v) < 1e-12 for v in ff) else (
                    f"FF v=({ff[0]:+.3f},{ff[1]:+.3f}) t=({ff[2]:+.3f},{ff[3]:+.3f})"
                )
                self.table.setItem(
                    row,
                    self.COL_STATE,
                    self._readonly_item(f"Ready θ=({theta0:.2f},{theta1:.2f}) | {ff_text}"),
                )
        finally:
            self._updating_table = False
        self.status.setText(f"All {len(points)} coordinates, trapezoids and feedforward values are valid.")

    def set_validation_error(self, message: str) -> None:
        self.status.setText(f"Validation failed: {message}")
        self._updating_table = True
        try:
            for row in range(self.table.rowCount()):
                self.table.setItem(row, self.COL_STATE, self._readonly_item("Check row values"))
        finally:
            self._updating_table = False

    def set_running(self, running: bool, total_moves: int = 1) -> None:
        self.run_button.setEnabled(not running)
        self.preview_button.setEnabled(not running)
        self.stop_button.setEnabled(running)
        self.repeat_count.setEnabled(not running)
        self.feedforward_enabled.setEnabled(not running)
        self.table.setEnabled(not running)
        self.load_button.setEnabled(not running)
        self.save_button.setEnabled(not running)
        for button in self.row_edit_buttons:
            button.setEnabled(not running)
        for widget in (
            self.profile_vel,
            self.profile_acc,
            self.profile_dec,
            self.preview_distance,
            self.velocity_ff0,
            self.velocity_ff1,
            self.torque_ff0,
            self.torque_ff1,
            self.apply_selected_profile_button,
            self.apply_all_profile_button,
            self.apply_selected_ff_button,
            self.apply_all_ff_button,
            self.load_selected_profile_button,
            self.set_global_profile_button,
        ):
            widget.setEnabled(not running)
        if running:
            self.inner_tabs.setCurrentIndex(2)
            self.progress.setRange(0, max(1, total_moves))
            self.progress.setValue(0)
            self.progress.setFormat("Starting sequence...")
        elif self.progress.value() >= self.progress.maximum():
            self.progress.setFormat("Sequence complete")
        else:
            self.progress.setFormat("Sequence stopped")

    def set_progress_payload(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return
        completed = int(payload.get("completed", 0))
        total = max(1, int(payload.get("total", 1)))
        row = int(payload.get("row_index", -1))
        phase = str(payload.get("phase", ""))
        text = str(payload.get("text", ""))
        self.progress.setRange(0, total)
        self.progress.setValue(max(0, min(total, completed)))
        self.progress.setFormat(text or f"{completed}/{total}")
        if 0 <= row < self.table.rowCount():
            self._updating_table = True
            try:
                for index in range(self.table.rowCount()):
                    current = self.table.item(index, self.COL_STATE)
                    if index == row:
                        self.table.setItem(index, self.COL_STATE, self._readonly_item(phase or "Active"))
                    elif current and current.text() in {"Moving", "Dwelling"}:
                        self.table.setItem(index, self.COL_STATE, self._readonly_item("Reached"))
            finally:
                self._updating_table = False
            self.table.scrollToItem(self.table.item(row, self.COL_INDEX))
        self.status.setText(text)

    def save_json(self) -> None:
        try:
            points = self.points()
        except Exception as exc:
            QMessageBox.warning(self, "Cannot save sequence", str(exc))
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save coordinate sequence", "coordinate_sequence.json", "JSON files (*.json)"
        )
        if not path:
            return
        payload = {
            "format": "five-bar-cartesian-sequence-v3",
            "repeat": self.repeat_count.value(),
            "feedforward_enabled": self.feedforward_enabled.isChecked(),
            "editor_profile": {
                "max_vel_deg_s": self.profile_vel.value(),
                "max_accel_deg_s2": self.profile_acc.value(),
                "max_decel_deg_s2": self.profile_dec.value(),
            },
            "feedforward_editor": {
                "velocity_ff0_turns_s": self.velocity_ff0.value(),
                "velocity_ff1_turns_s": self.velocity_ff1.value(),
                "torque_ff0_nm": self.torque_ff0.value(),
                "torque_ff1_nm": self.torque_ff1.value(),
            },
            "points": points,
        }
        try:
            with open(path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2)
            self.status.setText(f"Saved {len(points)} coordinates to {path}")
        except Exception as exc:
            QMessageBox.critical(self, "Save failed", str(exc))

    def load_json(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Load coordinate sequence", "", "JSON files (*.json)")
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            editor_profile: dict[str, Any] = {}
            ff_editor: dict[str, Any] = {}
            ff_enabled = False
            if isinstance(payload, list):
                points = payload
                repeat = 1
            elif isinstance(payload, dict):
                points = payload.get("points", [])
                repeat = int(payload.get("repeat", 1))
                editor_profile = payload.get("editor_profile", {}) or {}
                ff_editor = payload.get("feedforward_editor", {}) or {}
                ff_enabled = bool(payload.get("feedforward_enabled", False))
            else:
                raise ValueError("Expected a JSON list or an object containing a 'points' list.")
            if editor_profile:
                self.profile_vel.setValue(float(editor_profile.get("max_vel_deg_s", self.profile_vel.value())))
                self.profile_acc.setValue(float(editor_profile.get("max_accel_deg_s2", self.profile_acc.value())))
                self.profile_dec.setValue(float(editor_profile.get("max_decel_deg_s2", self.profile_dec.value())))
            if ff_editor:
                self.velocity_ff0.setValue(float(ff_editor.get("velocity_ff0_turns_s", 0.0)))
                self.velocity_ff1.setValue(float(ff_editor.get("velocity_ff1_turns_s", 0.0)))
                self.torque_ff0.setValue(float(ff_editor.get("torque_ff0_nm", 0.0)))
                self.torque_ff1.setValue(float(ff_editor.get("torque_ff1_nm", 0.0)))
            self.table.setRowCount(0)
            for point in points:
                if isinstance(point, dict):
                    self.add_point(
                        point.get("x_mm", point.get("x", 0.0)),
                        point.get("y_mm", point.get("y", 0.0)),
                        point.get("dwell_s", point.get("dwell", 0.0)),
                        point.get("max_vel_deg_s", point.get("vel_deg_s")),
                        point.get("max_accel_deg_s2", point.get("accel_deg_s2")),
                        point.get("max_decel_deg_s2", point.get("decel_deg_s2")),
                        point.get("velocity_ff0_turns_s", point.get("vel_ff0", 0.0)),
                        point.get("velocity_ff1_turns_s", point.get("vel_ff1", 0.0)),
                        point.get("torque_ff0_nm", point.get("torque_ff0", 0.0)),
                        point.get("torque_ff1_nm", point.get("torque_ff1", 0.0)),
                    )
                else:
                    values = list(point)
                    self.add_point(
                        values[0],
                        values[1],
                        values[2] if len(values) > 2 else 0.0,
                        values[3] if len(values) > 3 else None,
                        values[4] if len(values) > 4 else None,
                        values[5] if len(values) > 5 else None,
                        values[6] if len(values) > 6 else 0.0,
                        values[7] if len(values) > 7 else 0.0,
                        values[8] if len(values) > 8 else 0.0,
                        values[9] if len(values) > 9 else 0.0,
                    )
            self.repeat_count.setValue(max(1, min(1000, repeat)))
            # Loading a file never silently arms additive feedforward. Preserve the file's
            # intent in the status but require the operator to tick the checkbox manually.
            self.feedforward_enabled.setChecked(False)
            suffix = " File requested feedforward; it remains disarmed until manually enabled." if ff_enabled else ""
            self.status.setText(
                f"Loaded {self.table.rowCount()} coordinates from {path}; validate before running.{suffix}"
            )
            self.inner_tabs.setCurrentIndex(0)
        except Exception as exc:
            QMessageBox.critical(self, "Load failed", str(exc))

class PIDTuningGuidePanel(QWidget):
    """In-app, mechanism-specific PID tuning workflow and interpretation guide."""

    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(section_title("ODrive Cascaded PID Tuning Guide"))
        browser = QTextBrowser()
        browser.setOpenExternalLinks(False)
        browser.setHtml(self._guide_html())
        layout.addWidget(browser)

    @staticmethod
    def _guide_html() -> str:
        return """
        <h2>Before applying any gain</h2>
        <ol>
          <li>Support the linkage or test at a pose where a sudden movement cannot hit the frame.</li>
          <li>Confirm encoder direction, gear ratio and software zero. PID cannot correct a wrong mapping.</li>
          <li>Use conservative <b>current_lim</b> and <b>vel_limit</b>. Keep the E-stop reachable.</li>
          <li>Tune one axis at a time. Read and record the original hardware values first.</li>
          <li>Apply changes in RAM only. Save to flash only after repeated tests are stable.</li>
        </ol>

        <h2>What the three gains do</h2>
        <table border="1" cellspacing="0" cellpadding="5">
          <tr><th>Field</th><th>Role</th><th>Too low</th><th>Too high</th></tr>
          <tr><td><b>vel_gain</b></td><td>Inner velocity-loop proportional gain; main damping and torque response.</td><td>Soft, slow velocity tracking.</td><td>Buzzing, current noise, high-frequency oscillation.</td></tr>
          <tr><td><b>pos_gain</b></td><td>Outer position loop; converts position error into a velocity demand.</td><td>Slow, compliant position response.</td><td>Overshoot or repeated position oscillation.</td></tr>
          <tr><td><b>vel_integrator_gain</b></td><td>Removes persistent velocity/position error under load.</td><td>Residual steady error.</td><td>Slow oscillation, wind-up, overshoot after saturation.</td></tr>
        </table>

        <h2>Recommended tuning order</h2>
        <ol>
          <li><b>Establish a safe baseline.</b> Set a low current limit and velocity limit. Set the velocity integrator to zero. Keep position gain low enough that the axis moves gently.</li>
          <li><b>Tune vel_gain first.</b> Run a small position step. Increase <b>vel_gain</b> using +25% increments until the response becomes firm. At the first sign of buzzing or rapid oscillation, reduce it by at least 20%. Do not use current saturation as a tuning target.</li>
          <li><b>Tune pos_gain.</b> Increase it until settling is fast enough. If overshoot or position oscillation grows after a position-gain change, reduce <b>pos_gain</b>.</li>
          <li><b>Add vel_integrator_gain last.</b> Raise it only enough to remove repeatable final error under the real load. If it creates slow oscillation or delayed overshoot, reduce it.</li>
          <li><b>Validate.</b> Repeat the test in both directions, at several arm poses, and with the expected end-effector load.</li>
        </ol>

        <h2>Using the Step-response panel</h2>
        <ul>
          <li>For the first physical test, start smaller than the default 0.05-turn step, for example 0.01-0.02 turns, then increase only after the motion is safe.</li>
          <li>Use 1.5-3 s duration so the final settling behavior is visible.</li>
          <li>Change only one gain between runs. The chart keeps the latest four runs and records the gains in the legend.</li>
          <li>A practical starting goal is low overshoot, repeatable settling and final error near the encoder-noise floor. There is no universal numeric gain or settling target for every linkage.</li>
          <li>Compare raw and filtered velocity. A heavy low-pass filter can hide oscillation and make a poor tune look stable.</li>
        </ul>

        <h2>How to interpret a bad response</h2>
        <table border="1" cellspacing="0" cellpadding="5">
          <tr><th>Observed response</th><th>Likely action</th></tr>
          <tr><td>Very slow, no overshoot, low current</td><td>Increase vel_gain first if velocity tracking is soft; then increase pos_gain.</td></tr>
          <tr><td>Fast overshoot after increasing pos_gain</td><td>Reduce pos_gain. Verify vel_gain is not too low to provide damping.</td></tr>
          <tr><td>High-frequency buzz or noisy current at rest</td><td>Reduce vel_gain and investigate encoder noise, magnet alignment, grounding and mechanical resonance.</td></tr>
          <tr><td>Persistent final error under load</td><td>After P gains are stable, add a small amount of vel_integrator_gain.</td></tr>
          <tr><td>Slow oscillation or delayed overshoot</td><td>Reduce vel_integrator_gain; check for saturation and integral wind-up.</td></tr>
          <tr><td>Current limit reached</td><td>Reduce step size/load or acceleration. Do not keep raising gains.</td></tr>
          <tr><td>Different behavior at different arm poses</td><td>The five-bar dynamics change with pose. Tune for the worst safe pose, not only the easiest pose.</td></tr>
        </table>

        <h2>Before saving to flash</h2>
        <ul>
          <li>Run at least four repeatable tests, both positive and negative.</li>
          <li>Check motor and controller temperature and confirm no sustained current at rest.</li>
          <li>Test low-speed motion and the maximum intended speed with conservative acceleration.</li>
          <li>Save to flash only after both axes have been validated. Saving reboots the ODrive and drops the USB connection.</li>
        </ul>
        """


class ConfigPanel(QWidget):
    apply_requested = Signal(object)
    save_requested = Signal()
    reload_requested = Signal()
    read_spi_requested = Signal()
    read_hardware_requested = Signal()
    apply_spi_requested = Signal(object)

    def __init__(self, config: DashboardConfig) -> None:
        super().__init__()
        root = QVBoxLayout(self)
        scroll = QScrollArea(); scroll.setWidgetResizable(True)
        content = QWidget(); self.layout = QVBoxLayout(content)
        scroll.setWidget(content); root.addWidget(scroll)

        self._build_hardware_snapshot()
        self._build_geometry()
        self._build_axis_mapping()
        self._build_spi()
        self._build_trajectory()
        self._build_velocity()
        self._build_filters()
        self._build_encoder_noise()
        self._build_display()
        self.layout.addStretch(1)
        controls = QHBoxLayout()
        apply = QPushButton("Apply + Auto-save")
        save = QPushButton("Save Now")
        reload_button = QPushButton("Reload From File")
        apply.clicked.connect(lambda: self.apply_requested.emit(self.collect_config()))
        save.clicked.connect(self.save_requested)
        reload_button.clicked.connect(self.reload_requested)
        controls.addWidget(apply); controls.addWidget(save); controls.addWidget(reload_button)
        root.addLayout(controls)
        self.load_config(config)

    def _build_hardware_snapshot(self) -> None:
        group = QGroupBox("Connected ODrive hardware snapshot (read-only)")
        box = QVBoxLayout(group)
        explanation = QLabel(
            "The linkage dimensions, gear ratio, direction and software zero are dashboard-side "
            "settings and cannot be discovered from the ODrive. The snapshot below reads the "
            "settings that really exist in the connected controller."
        )
        explanation.setWordWrap(True)
        box.addWidget(explanation)
        read = QPushButton("Read Hardware Configuration")
        read.clicked.connect(self.read_hardware_requested)
        box.addWidget(read)
        self.hardware_snapshot = QPlainTextEdit()
        self.hardware_snapshot.setReadOnly(True)
        self.hardware_snapshot.setMaximumHeight(280)
        self.hardware_snapshot.setPlaceholderText("Connect to the ODrive, then read the hardware configuration.")
        box.addWidget(self.hardware_snapshot)
        self.layout.addWidget(group)

    def _build_geometry(self) -> None:
        group = QGroupBox("Link geometry (mm)")
        form = QFormLayout(group)
        self.geometry = {
            "L0": dspin(1, 5000, 300, 2, 5, " mm"),
            "l1a": dspin(1, 5000, 300, 2, 5, " mm"),
            "l2a": dspin(1, 5000, 450, 2, 5, " mm"),
            "l1b": dspin(1, 5000, 300, 2, 5, " mm"),
            "l2b": dspin(1, 5000, 450, 2, 5, " mm"),
        }
        for key, widget in self.geometry.items(): form.addRow(key, widget)
        self.elbow0 = QComboBox(); self.elbow0.addItems(["up", "down"])
        self.elbow1 = QComboBox(); self.elbow1.addItems(["up", "down"])
        self.fk_branch = QComboBox(); self.fk_branch.addItems(["upper", "lower"])
        form.addRow("elbow1 / axis0", self.elbow0); form.addRow("elbow2 / axis1", self.elbow1); form.addRow("FK branch", self.fk_branch)
        self.layout.addWidget(group)

    def _build_axis_mapping(self) -> None:
        group = QGroupBox("Dashboard-only motor ↔ joint mapping (not stored in ODrive)")
        tabs = QTabWidget(); self.axis_mapping: dict[int, dict[str, QDoubleSpinBox]] = {}
        self.home_fields: dict[int, QDoubleSpinBox] = {}
        for axis in (0, 1):
            page = QWidget(); form = QFormLayout(page)
            widgets = {
                "gear_ratio": dspin(0.000001, 10000, 1.0, 6, 0.1),
                "offset_turns": dspin(-100000, 100000, 0.0, 7, 0.01, " turns"),
                "direction": dspin(-1, 1, -1.0, 0, 2.0),
            }
            widgets["direction"].setRange(-1.0, 1.0); widgets["direction"].setSingleStep(2.0)
            home = dspin(-100000, 100000, 90.0, 6, 1.0, " °")
            for key, widget in widgets.items(): form.addRow(key, widget)
            form.addRow("home_angle_deg", home)
            self.axis_mapping[axis] = widgets; self.home_fields[axis] = home
            tabs.addTab(page, f"axis{axis}")
        box = QVBoxLayout(group); box.addWidget(tabs); self.layout.addWidget(group)

    def _build_spi(self) -> None:
        group = QGroupBox("SPI absolute encoder interface")
        layout = QVBoxLayout(group)
        tabs = QTabWidget(); self.spi_widgets: dict[int, dict[str, QWidget]] = {}
        for axis in (0, 1):
            page = QWidget(); form = QFormLayout(page)
            mode = QComboBox()
            for label, value in ENCODER_MODE_LABELS.items(): mode.addItem(label, value)
            cs = QSpinBox(); cs.setRange(0, 64)
            form.addRow("Encoder mode", mode); form.addRow("CS GPIO", cs)
            self.spi_widgets[axis] = {"mode": mode, "cs": cs}
            tabs.addTab(page, f"axis{axis}")
        layout.addWidget(tabs)
        buttons = QHBoxLayout()
        read = QPushButton("Read Current Config")
        apply = QPushButton("Apply to ODrive (saves & reboots)")
        read.clicked.connect(self.read_spi_requested)
        apply.clicked.connect(lambda: self.apply_spi_requested.emit(self.collect_spi_payload()))
        buttons.addWidget(read); buttons.addWidget(apply); layout.addLayout(buttons)
        warning = QLabel("Changing the actual chip mode or CS wiring requires recalibration after reboot.")
        warning.setWordWrap(True); warning.setProperty("class", "warning"); layout.addWidget(warning)
        self.layout.addWidget(group)

    def _build_trajectory(self) -> None:
        group = QGroupBox("Point-to-point move limits")
        form = QFormLayout(group)
        self.traj_vel = dspin(0.01, 10000, 60.0, 2, 5, " deg/s")
        self.traj_acc = dspin(0.01, 100000, 120.0, 2, 10, " deg/s²")
        self.traj_dec = dspin(0.01, 100000, 120.0, 2, 10, " deg/s²")
        form.addRow("max_vel_deg_s", self.traj_vel)
        form.addRow("max_accel_deg_s2", self.traj_acc)
        form.addRow("max_decel_deg_s2", self.traj_dec)
        self.layout.addWidget(group)

    def _build_velocity(self) -> None:
        group = QGroupBox("Velocity-control safety and PC position loop")
        form = QFormLayout(group)
        definitions = {
            "loop_hz": (1, 500, 60.0, " Hz"),
            "joint_vel_cap_deg_s": (0.1, 10000, 45.0, " deg/s"),
            "joint_accel_cap_deg_s2": (0.1, 100000, 180.0, " deg/s²"),
            "max_cart_speed_mm_s": (0.1, 10000, 80.0, " mm/s"),
            "pos_kp": (0.0, 10000, 3.0, ""),
            "pos_tol_mm": (0.001, 1000, 1.0, " mm"),
            "manip_soft_deg_mm": (0.0, 1000, 3.0, " deg/mm"),
            "manip_hard_deg_mm": (0.001, 1000, 8.0, " deg/mm"),
            "watchdog_s": (0.01, 10, 0.15, " s"),
            "deadman_s": (0.01, 60, 0.5, " s"),
        }
        self.velocity: dict[str, QDoubleSpinBox] = {}
        for key, (minimum, maximum, value, suffix) in definitions.items():
            widget = dspin(minimum, maximum, value, 4, 0.1, suffix)
            self.velocity[key] = widget; form.addRow(key, widget)
        self.layout.addWidget(group)

    def _make_filter_page(self, axis: int) -> QWidget:
        page = QWidget(); layout = QFormLayout(page)
        combo = QComboBox(); combo.addItems(FILTER_TYPES)
        stack = QStackedWidget()
        none = QWidget(); QVBoxLayout(none).addWidget(QLabel("Raw passthrough; no parameters."))
        ma = QWidget(); ma_form = QFormLayout(ma); ma_window = QSpinBox(); ma_window.setRange(1, 999); ma_form.addRow("Window samples", ma_window)
        lp = QWidget(); lp_form = QFormLayout(lp); lp_cutoff = dspin(0.01, 1000, 10.0, 3, 1, " Hz"); lp_form.addRow("Cutoff", lp_cutoff)
        bw = QWidget(); bw_form = QFormLayout(bw); bw_cutoff = dspin(0.01, 1000, 10.0, 3, 1, " Hz"); bw_order = QSpinBox(); bw_order.setRange(1, 10); bw_form.addRow("Cutoff", bw_cutoff); bw_form.addRow("Order", bw_order)
        med = QWidget(); med_form = QFormLayout(med); med_window = QSpinBox(); med_window.setRange(1, 999); med_form.addRow("Window samples", med_window)
        for widget in (none, ma, lp, bw, med): stack.addWidget(widget)
        combo.currentIndexChanged.connect(stack.setCurrentIndex)
        layout.addRow("Filter type", combo); layout.addRow(stack)
        self.filter_widgets[axis] = {
            "combo": combo, "stack": stack, "ma_window": ma_window,
            "lp_cutoff": lp_cutoff, "bw_cutoff": bw_cutoff, "bw_order": bw_order,
            "med_window": med_window,
        }
        return page

    def _build_filters(self) -> None:
        group = QGroupBox("DSP velocity filters")
        box = QVBoxLayout(group); tabs = QTabWidget(); self.filter_widgets: dict[int, dict[str, QWidget]] = {}
        tabs.addTab(self._make_filter_page(0), "axis0"); tabs.addTab(self._make_filter_page(1), "axis1")
        box.addWidget(tabs); self.layout.addWidget(group)


    def _build_encoder_noise(self) -> None:
        group = QGroupBox("Encoder noise handling")
        form = QFormLayout(group)
        definitions = {
            "position_median_window": (1, 101, 7.0, " samples", 0),
            "motion_window_s": (0.10, 5.0, 0.40, " s", 3),
            "motion_deadband_turns_s": (0.0, 1.0, 0.003, " turns/s", 6),
            "sync_sample_duration_s": (0.25, 10.0, 1.50, " s", 2),
            "sync_sample_hz": (5.0, 200.0, 60.0, " Hz", 1),
            "sync_max_drift_turns_s": (0.0, 1.0, 0.003, " turns/s", 6),
            "sync_noise_warning_span_turns": (0.0, 1.0, 0.003, " turns", 6),
            "sync_hard_span_turns": (0.000001, 2.0, 0.015, " turns", 6),
        }
        self.encoder_noise: dict[str, QDoubleSpinBox] = {}
        for key, (minimum, maximum, value, suffix, decimals) in definitions.items():
            widget = dspin(minimum, maximum, value, decimals, 1.0 if decimals == 0 else 0.001, suffix)
            self.encoder_noise[key] = widget
            form.addRow(key, widget)
        explanation = QLabel(
            "Software sync uses a multi-sample median and position drift. ODrive raw "
            "vel_estimate is displayed for diagnosis but is not used to decide whether the robot moves."
        )
        explanation.setWordWrap(True)
        explanation.setProperty("class", "warning")
        form.addRow(explanation)
        self.layout.addWidget(group)

    def _build_display(self) -> None:
        group = QGroupBox("Display")
        form = QFormLayout(group)
        self.show_workspace = QCheckBox(); self.show_workspace.setChecked(True)
        self.auto_fit = QCheckBox(); self.auto_fit.setChecked(True)
        self.px_per_mm = dspin(0.05, 20, 1.0, 3, 0.1, " px/mm")
        self.telemetry_window = dspin(2, 120, 20.0, 1, 1.0, " s")
        form.addRow("Show reachable workspace", self.show_workspace)
        form.addRow("Auto-fit linkage view", self.auto_fit)
        form.addRow("Manual px/mm", self.px_per_mm)
        form.addRow("Telemetry window", self.telemetry_window)
        self.layout.addWidget(group)

    def load_config(self, cfg: DashboardConfig) -> None:
        g = cfg.geometry
        for key, widget in self.geometry.items(): widget.setValue(float(getattr(g, key)))
        self.elbow0.setCurrentText(g.elbow1); self.elbow1.setCurrentText(g.elbow2); self.fk_branch.setCurrentText(g.fk_branch)
        for axis in (0, 1):
            mapping = cfg.axes[axis]
            for key, widget in self.axis_mapping[axis].items(): widget.setValue(float(getattr(mapping, key)))
            self.home_fields[axis].setValue(cfg.home_angle_deg[axis])
            spi = cfg.spi[axis]
            combo = self.spi_widgets[axis]["mode"]
            index = combo.findData(spi.mode); combo.setCurrentIndex(max(0, index))
            self.spi_widgets[axis]["cs"].setValue(spi.cs_gpio)
            filt = cfg.filters[axis]; fw = self.filter_widgets[axis]
            fw["combo"].setCurrentText(filt.type)
            fw["ma_window"].setValue(filt.window); fw["med_window"].setValue(filt.window)
            fw["lp_cutoff"].setValue(filt.cutoff_hz); fw["bw_cutoff"].setValue(filt.cutoff_hz); fw["bw_order"].setValue(filt.order)
        self.traj_vel.setValue(cfg.trajectory.max_vel_deg_s)
        self.traj_acc.setValue(cfg.trajectory.max_accel_deg_s2)
        self.traj_dec.setValue(cfg.trajectory.max_decel_deg_s2)
        for key, widget in self.velocity.items(): widget.setValue(float(getattr(cfg.velocity, key)))
        for key, widget in self.encoder_noise.items(): widget.setValue(float(getattr(cfg.encoder_noise, key)))
        self.show_workspace.setChecked(cfg.display.show_workspace); self.auto_fit.setChecked(cfg.display.auto_fit)
        self.px_per_mm.setValue(cfg.display.px_per_mm); self.telemetry_window.setValue(cfg.display.telemetry_window_s)

    def collect_config(self) -> DashboardConfig:
        geometry = GeometryConfig(
            **{key: widget.value() for key, widget in self.geometry.items()},
            elbow1=self.elbow0.currentText(), elbow2=self.elbow1.currentText(), fk_branch=self.fk_branch.currentText(),
        )
        axes = {
            axis: AxisMappingConfig(
                gear_ratio=self.axis_mapping[axis]["gear_ratio"].value(),
                offset_turns=self.axis_mapping[axis]["offset_turns"].value(),
                direction=self.axis_mapping[axis]["direction"].value(),
            ) for axis in (0, 1)
        }
        spi = {
            axis: EncoderInterfaceConfig(
                mode=int(self.spi_widgets[axis]["mode"].currentData()),
                cs_gpio=int(self.spi_widgets[axis]["cs"].value()),
            ) for axis in (0, 1)
        }
        filters: dict[int, VelocityFilterConfig] = {}
        for axis in (0, 1):
            fw = self.filter_widgets[axis]; filter_type = fw["combo"].currentText()
            if filter_type == "Moving Average": window = fw["ma_window"].value()
            elif filter_type == "Median": window = fw["med_window"].value()
            else: window = 5
            cutoff = fw["bw_cutoff"].value() if filter_type == "Butterworth" else fw["lp_cutoff"].value()
            filters[axis] = VelocityFilterConfig(type=filter_type, window=window, cutoff_hz=cutoff, order=fw["bw_order"].value())
        cfg = DashboardConfig(
            geometry=geometry,
            axes=axes,
            home_angle_deg={axis: self.home_fields[axis].value() for axis in (0, 1)},
            spi=spi,
            trajectory=TrajectoryConfig(
                max_vel_deg_s=self.traj_vel.value(),
                max_accel_deg_s2=self.traj_acc.value(),
                max_decel_deg_s2=self.traj_dec.value(),
            ),
            velocity=VelocityControlConfig(**{key: widget.value() for key, widget in self.velocity.items()}),
            filters=filters,
            encoder_noise=EncoderNoiseConfig(
                position_median_window=int(round(self.encoder_noise["position_median_window"].value())),
                motion_window_s=self.encoder_noise["motion_window_s"].value(),
                motion_deadband_turns_s=self.encoder_noise["motion_deadband_turns_s"].value(),
                sync_sample_duration_s=self.encoder_noise["sync_sample_duration_s"].value(),
                sync_sample_hz=self.encoder_noise["sync_sample_hz"].value(),
                sync_max_drift_turns_s=self.encoder_noise["sync_max_drift_turns_s"].value(),
                sync_noise_warning_span_turns=self.encoder_noise["sync_noise_warning_span_turns"].value(),
                sync_hard_span_turns=self.encoder_noise["sync_hard_span_turns"].value(),
            ),
            display=DisplayConfig(
                show_workspace=self.show_workspace.isChecked(),
                px_per_mm=self.px_per_mm.value(),
                auto_fit=self.auto_fit.isChecked(),
                telemetry_window_s=self.telemetry_window.value(),
            ),
        )
        cfg.validate()
        return cfg

    def collect_spi_payload(self) -> dict[str, dict[str, int]]:
        return {
            f"axis{axis}": {
                "mode": int(self.spi_widgets[axis]["mode"].currentData()),
                "cs_gpio": int(self.spi_widgets[axis]["cs"].value()),
            } for axis in (0, 1)
        }

    def set_spi_from_hardware(self, values: dict[str, dict[str, int]]) -> None:
        for axis in (0, 1):
            value = values[f"axis{axis}"]
            combo = self.spi_widgets[axis]["mode"]
            index = combo.findData(value["mode"])
            if index >= 0: combo.setCurrentIndex(index)
            self.spi_widgets[axis]["cs"].setValue(value["cs_gpio"])

    def set_hardware_snapshot(self, values: dict[str, Any]) -> None:
        self.hardware_snapshot.setPlainText(json.dumps(values, indent=2, sort_keys=True))

    def set_software_offsets(self, offsets: dict[int, float]) -> None:
        for axis in (0, 1):
            self.axis_mapping[axis]["offset_turns"].setValue(float(offsets[axis]))

    def set_home_angles(self, homes: dict[int, float]) -> None:
        for axis in (0, 1): self.home_fields[axis].setValue(homes[axis])
