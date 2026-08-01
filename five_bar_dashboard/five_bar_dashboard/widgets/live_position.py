"""Always-visible state-independent encoder position readout."""
from __future__ import annotations

import time

from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import QFrame, QGridLayout, QLabel, QWidget

from ..constants import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, axis_state_name
from ..models import TelemetrySample


class LivePositionWidget(QFrame):
    """Shows raw and noise-filtered encoder position in every ODrive axis state."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("LivePositionBar")
        self._last_update_monotonic: float | None = None

        layout = QGridLayout(self)
        layout.setContentsMargins(12, 6, 12, 6)
        layout.setHorizontalSpacing(18)
        layout.setVerticalSpacing(2)

        title = QLabel("LIVE ENCODER POSITION")
        title.setStyleSheet("font-weight: 800; letter-spacing: 0.5px;")
        self.telemetry_status = QLabel("WAITING FOR CONNECTION")
        self.telemetry_status.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.telemetry_status.setStyleSheet("font-weight: 700; color: #FBBF24;")
        layout.addWidget(title, 0, 0, 1, 2)
        layout.addWidget(self.telemetry_status, 0, 4, 1, 2)

        self.axis_headings: list[QLabel] = []
        self.raw_labels: list[QLabel] = []
        self.angle_labels: list[QLabel] = []
        self.velocity_labels: list[QLabel] = []
        self.error_labels: list[QLabel] = []

        for axis in (0, 1):
            column = axis * 2
            heading = QLabel(f"axis{axis}: --")
            heading.setStyleSheet("font-weight: 700;")
            raw = QLabel("raw: -- turns")
            angle = QLabel("filtered joint: -- °")
            velocity = QLabel("motion: --")
            errors = QLabel("errors: --")
            errors.setWordWrap(True)
            self.axis_headings.append(heading)
            self.raw_labels.append(raw)
            self.angle_labels.append(angle)
            self.velocity_labels.append(velocity)
            self.error_labels.append(errors)
            layout.addWidget(heading, 1, column, 1, 2)
            layout.addWidget(raw, 2, column)
            layout.addWidget(angle, 2, column + 1)
            layout.addWidget(velocity, 3, column)
            layout.addWidget(errors, 3, column + 1)

        self.cartesian_label = QLabel("End effector: waiting for telemetry")
        self.cartesian_label.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)
        self.cartesian_label.setStyleSheet("font-weight: 700;")
        layout.addWidget(self.cartesian_label, 1, 4, 3, 2)

        self.note = QLabel(
            "Joint/FK position uses a median-filtered encoder value. Motion state is derived "
            "from position drift; noisy raw vel_estimate does not mark the robot as moving."
        )
        self.note.setWordWrap(True)
        self.note.setStyleSheet("color: #94A3B8; font-size: 9pt;")
        layout.addWidget(self.note, 4, 0, 1, 6)

        self.stale_timer = QTimer(self)
        self.stale_timer.setInterval(250)
        self.stale_timer.timeout.connect(self._update_freshness)
        self.stale_timer.start()

    def set_waiting(self, text: str = "WAITING FOR TELEMETRY") -> None:
        self._last_update_monotonic = None
        self.telemetry_status.setText(text)
        self.telemetry_status.setStyleSheet("font-weight: 700; color: #FBBF24;")

    def set_disconnected(self) -> None:
        self._last_update_monotonic = None
        self.telemetry_status.setText("DISCONNECTED")
        self.telemetry_status.setStyleSheet("font-weight: 700; color: #F87171;")
        for axis in (0, 1):
            self.axis_headings[axis].setText(f"axis{axis}: --")
            self.raw_labels[axis].setText("raw: -- turns")
            self.angle_labels[axis].setText("filtered joint: -- °")
            self.velocity_labels[axis].setText("motion: --")
            self.error_labels[axis].setText("errors: --")
        self.cartesian_label.setText("End effector: unavailable")

    def set_sample(self, sample: TelemetrySample) -> None:
        self._last_update_monotonic = time.monotonic()
        self.telemetry_status.setText("LIVE")
        self.telemetry_status.setStyleSheet("font-weight: 800; color: #34D399;")

        for axis in (0, 1):
            state = int(sample.axis_state[axis])
            state_name = axis_state_name(state)
            self.axis_headings[axis].setText(f"axis{axis}: {state_name} ({state})")
            if state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                state_colour = "#34D399"
            elif state == AXIS_STATE_IDLE:
                state_colour = "#FBBF24"
            else:
                state_colour = "#60A5FA"
            self.axis_headings[axis].setStyleSheet(
                f"font-weight: 800; color: {state_colour};"
            )
            self.raw_labels[axis].setText(
                f"raw: {sample.raw_pos_turns[axis]:+.7f} t\n"
                f"stable: {sample.filtered_pos_turns[axis]:+.7f} t"
            )
            self.angle_labels[axis].setText(f"filtered joint: {sample.theta_deg[axis]:+.4f} °")
            if bool(sample.stationary[axis]):
                motion_text = "STATIONARY"
                motion_colour = "#34D399"
            else:
                motion_text = "MOVING"
                motion_colour = "#FBBF24"
            self.velocity_labels[axis].setText(
                f"motion: {motion_text}\n"
                f"position drift: {sample.motion_estimate_turns_s[axis]:+.5f} t/s\n"
                f"raw vel sensor: {sample.raw_vel_turns_s[axis]:+.5f} t/s"
            )
            self.velocity_labels[axis].setStyleSheet(
                f"font-weight: 700; color: {motion_colour};"
            )
            errors = (
                int(sample.axis_error[axis]),
                int(sample.motor_error[axis]),
                int(sample.encoder_error[axis]),
            )
            if any(errors):
                self.error_labels[axis].setText(
                    f"errors: A=0x{errors[0]:X} M=0x{errors[1]:X} E=0x{errors[2]:X}"
                )
                self.error_labels[axis].setStyleSheet("font-weight: 700; color: #F87171;")
            else:
                self.error_labels[axis].setText("errors: none")
                self.error_labels[axis].setStyleSheet("color: #94A3B8;")

        if sample.end_effector is None:
            self.cartesian_label.setText(
                "End effector: FK unavailable\n(raw and filtered joint positions remain live)"
            )
            self.cartesian_label.setStyleSheet("font-weight: 700; color: #FBBF24;")
        else:
            x, y = sample.end_effector
            self.cartesian_label.setText(f"End effector\nX = {x:+.3f} mm\nY = {y:+.3f} mm")
            self.cartesian_label.setStyleSheet("font-weight: 800; color: #E5E7EB;")

    def _update_freshness(self) -> None:
        if self._last_update_monotonic is None:
            return
        age = time.monotonic() - self._last_update_monotonic
        if age > 1.0:
            self.telemetry_status.setText(f"STALE  {age:.1f} s")
            self.telemetry_status.setStyleSheet("font-weight: 800; color: #F87171;")
