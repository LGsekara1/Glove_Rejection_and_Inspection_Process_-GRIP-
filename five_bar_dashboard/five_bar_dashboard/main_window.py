"""Main Qt window and worker orchestration."""
from __future__ import annotations

import copy
import threading
import time
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

from PySide6.QtCore import QThread, QTimer, Qt
from PySide6.QtGui import QCloseEvent, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from .backend import ODriveManager
from .config_store import ConfigStore
from .kinematics import KinematicsError, forward_kinematics, inverse_kinematics
from .models import DashboardConfig, TelemetrySample
from .sequence import compile_cartesian_sequence
from .panels import (
    CalibrationPanel,
    CoordinateSequencePanel,
    ConfigPanel,
    ForwardKinematicsPanel,
    InverseKinematicsPanel,
    JointControlPanel,
    PIDTuningGuidePanel,
    PIDTuningPanel,
    VelocityControlPanel,
)
from .widgets.error_dialog import ErrorReportDialog
from .widgets.linkage_view import LinkageView
from .widgets.live_position import LivePositionWidget
from .widgets.telemetry_plots import TelemetryPlots
from .workers import (
    AxisActionWorker,
    CalibrationWorker,
    ConnectWorker,
    CoordinateSequenceWorker,
    FlashConfigWorker,
    MoveWorker,
    PidWorker,
    PollWorker,
    RawNudgeWorker,
    StepResponseWorker,
    SyncZeroWorker,
    VelocityCommandState,
    VelocityWorker,
)


DoneCallback = Callable[[bool, str, object], None]


class MainWindow(QMainWindow):
    def __init__(self, simulate: bool = False, config_path: Path | None = None) -> None:
        super().__init__()
        self.setWindowTitle("Five-Bar Parallel SCARA Control Dashboard" + (" [SIMULATOR]" if simulate else ""))
        self.resize(1560, 980)
        self.setMinimumSize(1160, 760)

        self.store = ConfigStore(config_path)
        try:
            self.config = self.store.load()
        except Exception as exc:
            self.config = DashboardConfig()
            QMessageBox.warning(self, "Config load warning", f"Using defaults because config loading failed:\n{exc}")
        self.manager = ODriveManager(simulate=simulate)
        self.workers: dict[str, tuple[QThread, Any, bool, DoneCallback | None]] = {}
        self.poll_thread: QThread | None = None
        self.poll_worker: PollWorker | None = None
        self.poll_restart_pending = False
        self.last_sample: TelemetrySample | None = None
        self.motion_cancel: threading.Event | None = None
        self.velocity_cancel: threading.Event | None = None
        self.step_cancel: threading.Event | None = None
        self.sequence_cancel: threading.Event | None = None
        self.velocity_commands = VelocityCommandState()
        self.e_stop_active = False
        self.calibrated_clean_this_session = False
        self.error_dialog: ErrorReportDialog | None = None

        self._build_ui()
        self._wire_ui()
        self._apply_config_to_runtime()
        self._install_escape_shortcut()
        self.log("Dashboard started. Hardware access is isolated in QThread workers; Escape triggers E-stop.")
        if simulate:
            self.log("Simulator mode enabled. No physical ODrive commands will be sent.")

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        top = QFrame(); top.setObjectName("TopBar")
        top_layout = QHBoxLayout(top)
        title = QLabel("FIVE-BAR SCARA CONTROL")
        title.setStyleSheet("font-size: 14pt; font-weight: 800; letter-spacing: 1px;")
        self.connect_button = QPushButton("Connect")
        self.connection_label = QLabel("Disconnected")
        self.connection_label.setProperty("state", "error")
        self.stop_button = QPushButton("Stop Motion")
        self.emergency_button = QPushButton("EMERGENCY STOP  [Esc]"); self.emergency_button.setObjectName("EmergencyButton")
        self.resume_button = QPushButton("Resume After E-Stop"); self.resume_button.setObjectName("ResumeButton")
        top_layout.addWidget(title); top_layout.addSpacing(12); top_layout.addWidget(self.connect_button)
        top_layout.addWidget(self.connection_label); top_layout.addStretch(1)
        top_layout.addWidget(self.stop_button); top_layout.addWidget(self.emergency_button); top_layout.addWidget(self.resume_button)
        root.addWidget(top)

        self.live_position = LivePositionWidget()
        root.addWidget(self.live_position)

        self.joint_panel = JointControlPanel()
        self.ik_panel = InverseKinematicsPanel()
        self.coordinate_panel = CoordinateSequencePanel(self.config.trajectory)
        self.fk_panel = ForwardKinematicsPanel()
        self.velocity_panel = VelocityControlPanel()
        self.calibration_panel = CalibrationPanel()
        self.pid_panel = PIDTuningPanel()
        self.pid_guide_panel = PIDTuningGuidePanel()
        self.config_panel = ConfigPanel(self.config)

        self.tabs = QTabWidget()
        self.tabs.addTab(self.joint_panel, "Joint Control")
        self.tabs.addTab(self.ik_panel, "Inverse Kinematics")
        self.tabs.addTab(self.coordinate_panel, "Coordinate Sequence")
        self.tabs.addTab(self.fk_panel, "Forward Kinematics")
        self.tabs.addTab(self.velocity_panel, "Velocity Control")
        self.tabs.addTab(self.calibration_panel, "Calibration / Homing")
        self.tabs.addTab(self.pid_panel, "PID Tuning")
        self.tabs.addTab(self.pid_guide_panel, "PID Guide")
        self.tabs.addTab(self.config_panel, "Config")
        self.tabs.setMinimumWidth(540)

        self.linkage = LinkageView(self.config)
        self.plots = TelemetryPlots(self.config.display.telemetry_window_s)
        right = QSplitter(Qt.Orientation.Vertical)
        right.addWidget(self.linkage); right.addWidget(self.plots)
        right.setSizes([480, 380])

        horizontal = QSplitter(Qt.Orientation.Horizontal)
        horizontal.addWidget(self.tabs); horizontal.addWidget(right)
        horizontal.setSizes([620, 940])

        self.log_panel = QPlainTextEdit()
        self.log_panel.setReadOnly(True)
        self.log_panel.setMaximumBlockCount(5000)
        self.log_panel.setPlaceholderText("Timestamped system log")

        vertical = QSplitter(Qt.Orientation.Vertical)
        vertical.addWidget(horizontal); vertical.addWidget(self.log_panel)
        vertical.setSizes([790, 180])
        root.addWidget(vertical)
        self.setCentralWidget(central)

    def _wire_ui(self) -> None:
        self.connect_button.clicked.connect(self._connect_or_disconnect)
        self.stop_button.clicked.connect(self.stop_motion)
        self.emergency_button.clicked.connect(self.emergency_stop)
        self.resume_button.clicked.connect(self.resume_after_estop)

        self.joint_panel.move_requested.connect(self.start_joint_move)
        self.joint_panel.raw_nudge_requested.connect(self.start_raw_nudge)
        self.ik_panel.compute_requested.connect(self.compute_ik)
        self.coordinate_panel.preview_requested.connect(self.preview_coordinate_sequence)
        self.coordinate_panel.run_requested.connect(self.start_coordinate_sequence)
        self.coordinate_panel.stop_requested.connect(self.stop_coordinate_sequence)
        self.coordinate_panel.global_profile_requested.connect(
            self.set_global_trapezoid_profile
        )
        self.fk_panel.compute_requested.connect(self.compute_fk)
        self.fk_panel.use_current_requested.connect(self.use_current_angles)
        self.velocity_panel.start_requested.connect(self.start_velocity_session)
        self.velocity_panel.stop_requested.connect(self.stop_motion)
        self.velocity_panel.jog_command.connect(self.update_jog_command)
        self.velocity_panel.overlay_changed.connect(self.plots.set_raw_overlay)
        self.calibration_panel.action_requested.connect(self.run_axis_action)
        self.calibration_panel.calibrate_requested.connect(self.run_calibration)
        self.calibration_panel.sync_zero_requested.connect(self.sync_software_zero)
        self.calibration_panel.flash_requested.connect(self.run_flash_action)
        self.pid_panel.read_requested.connect(self.pid_read)
        self.pid_panel.apply_requested.connect(self.pid_apply)
        self.pid_panel.step_requested.connect(self.run_step_response)
        self.pid_panel.save_flash_requested.connect(lambda: self.run_flash_action("save_only", {}))
        self.config_panel.apply_requested.connect(self.apply_config)
        self.config_panel.save_requested.connect(self.save_config_now)
        self.config_panel.reload_requested.connect(self.reload_config)
        self.config_panel.read_spi_requested.connect(lambda: self.run_axis_action("read_spi"))
        self.config_panel.read_hardware_requested.connect(
            lambda: self.run_axis_action("read_hardware")
        )
        self.config_panel.apply_spi_requested.connect(lambda payload: self.run_flash_action("apply_spi", payload))

    def _install_escape_shortcut(self) -> None:
        shortcut = QShortcut(QKeySequence("Esc"), self)
        shortcut.setContext(Qt.ShortcutContext.ApplicationShortcut)
        shortcut.activated.connect(self.emergency_stop)
        self.escape_shortcut = shortcut

    def log(self, message: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        line = f"[{stamp}] {message}"
        self.log_panel.appendPlainText(line)
        print(line, flush=True)

    def set_status(self, message: str, severity: str = "info") -> None:
        self.connection_label.setText(message)
        self.connection_label.setProperty("state", severity)
        self.connection_label.style().unpolish(self.connection_label)
        self.connection_label.style().polish(self.connection_label)
        self.velocity_panel.set_status(message, severity)

    def _connected_guard(self) -> bool:
        if not self.manager.connected:
            QMessageBox.warning(self, "Not connected", "Connect to the ODrive first.")
            return False
        return True

    def _hardware_busy(self) -> bool:
        return any(key != "connect" for key in self.workers)

    def _start_worker(
        self,
        key: str,
        worker: Any,
        *,
        pause_poll: bool,
        done: DoneCallback | None = None,
        exclusive: bool = True,
    ) -> bool:
        if key in self.workers:
            self.log(f"Worker '{key}' is already running.")
            return False
        if exclusive and self._hardware_busy():
            active = ", ".join(self.workers)
            QMessageBox.warning(self, "Hardware busy", f"Another hardware operation is active: {active}")
            return False
        thread = QThread(self)
        worker.moveToThread(thread)
        worker.log.connect(self.log)
        worker.status.connect(self.set_status)
        worker.telemetry.connect(self.on_telemetry)
        worker.progress.connect(lambda payload, k=key: self._worker_progress(k, payload))
        worker.finished.connect(lambda ok, msg, payload, k=key: self._worker_result(k, ok, msg, payload))
        worker.finished.connect(thread.quit)
        thread.started.connect(worker.run)
        thread.finished.connect(lambda k=key: self._worker_cleanup(k))
        thread.finished.connect(worker.deleteLater)
        self.workers[key] = (thread, worker, pause_poll, done)
        self._update_poll_pause()
        thread.start()
        return True

    def _worker_result(self, key: str, ok: bool, message: str, payload: object) -> None:
        meta = self.workers.get(key)
        if meta and meta[3]:
            try:
                meta[3](ok, message, payload)
            except Exception as exc:
                self.log(f"Completion callback for '{key}' failed: {exc}")

    def _worker_progress(self, key: str, payload: object) -> None:
        if key == "sequence":
            self.coordinate_panel.set_progress_payload(payload)

    def _worker_cleanup(self, key: str) -> None:
        self.workers.pop(key, None)
        self._update_poll_pause()

    def _update_poll_pause(self) -> None:
        paused = any(meta[2] for meta in self.workers.values())
        if self.poll_worker:
            self.poll_worker.set_paused(paused)

    def _reset_polling_state_in_place(self, reason: str) -> None:
        """Reset telemetry DSP state without stopping state-independent polling."""
        if self.poll_worker is not None:
            self.poll_worker.request_reset()
            self.log(f"Telemetry remains active; requested in-place reset after {reason}.")
        elif self.manager.connected:
            # Defensive recovery only: a normal axis-state transition must never stop the
            # poller, but start it if it was already absent for some unrelated reason.
            self.log(f"Telemetry poller was absent after {reason}; starting it now.")
            self.start_polling()

    def _connect_or_disconnect(self) -> None:
        if self.manager.connected:
            if self._hardware_busy():
                QMessageBox.warning(self, "Busy", "Stop the active hardware operation before disconnecting.")
                return
            self.stop_polling()
            self.manager.disconnect()
            self.connect_button.setText("Connect")
            self.set_status("Disconnected", "error")
            self.live_position.set_disconnected()
            self.log("Disconnected by user.")
            return
        worker = ConnectWorker(self.manager)
        self.connect_button.setEnabled(False)

        def done(ok: bool, message: str, payload: object) -> None:
            self.connect_button.setEnabled(True)
            if ok:
                self.connect_button.setText("Disconnect")
                self.e_stop_active = False
                self.live_position.set_waiting("WAITING FOR FIRST SAMPLE")
                self.start_polling()
                QTimer.singleShot(250, lambda: self.run_axis_action("read_hardware"))
            else:
                self.connect_button.setText("Connect")

        self._start_worker("connect", worker, pause_poll=False, done=done, exclusive=False)

    def start_polling(self) -> None:
        if not self.manager.connected or self.poll_thread is not None:
            return
        self.poll_restart_pending = False
        worker = PollWorker(self.manager, copy.deepcopy(self.config), hz=50.0)
        thread = QThread(self)
        worker.moveToThread(thread)
        worker.log.connect(self.log)
        worker.status.connect(self.set_status)
        worker.telemetry.connect(self.on_telemetry)
        worker.finished.connect(thread.quit)
        thread.started.connect(worker.run)
        thread.finished.connect(self._poll_finished)
        thread.finished.connect(worker.deleteLater)
        self.poll_worker = worker; self.poll_thread = thread
        self._update_poll_pause()
        thread.start()

    def stop_polling(self, restart: bool = False) -> None:
        self.poll_restart_pending = restart
        if self.poll_worker:
            self.poll_worker.cancel_event.set()
        elif restart and self.manager.connected:
            QTimer.singleShot(0, self.start_polling)

    def _poll_finished(self) -> None:
        thread = self.poll_thread
        self.poll_worker = None; self.poll_thread = None
        if thread:
            thread.deleteLater()
        if self.poll_restart_pending and self.manager.connected:
            self.poll_restart_pending = False
            QTimer.singleShot(50, self.start_polling)

    def on_telemetry(self, sample: TelemetrySample) -> None:
        self.last_sample = sample
        self.live_position.set_sample(sample)
        self.plots.append(sample)
        self.linkage.set_sample(sample)
        if sample.blocked and sample.block_reason:
            self.velocity_panel.set_status(sample.block_reason, "warn")

    def start_joint_move(self, theta0: float, theta1: float) -> None:
        if not self._connected_guard() or self.e_stop_active:
            return
        self.motion_cancel = threading.Event()
        try:
            target = forward_kinematics(theta0, theta1, self.config.geometry).end_effector
            self.linkage.set_targets([target])
        except KinematicsError:
            self.linkage.set_targets([])
        worker = MoveWorker(self.manager, copy.deepcopy(self.config), (theta0, theta1), self.motion_cancel)

        def done(ok: bool, message: str, payload: object) -> None:
            self.linkage.set_targets([])
            self.motion_cancel = None

        self._start_worker("motion", worker, pause_poll=True, done=done)

    def start_raw_nudge(self, delta0: float, delta1: float) -> None:
        if not self._connected_guard() or self.e_stop_active:
            return
        worker = RawNudgeWorker(self.manager, (delta0, delta1))
        self._start_worker("raw_nudge", worker, pause_poll=True)

    def compute_ik(self, x: float, y: float, elbow0: str, elbow1: str, move: bool) -> None:
        geometry = replace(self.config.geometry, elbow1=elbow0, elbow2=elbow1)
        try:
            theta0, theta1 = inverse_kinematics(x, y, geometry)
            self.ik_panel.set_result(f"θ1 = {theta0:.4f}°, θ2 = {theta1:.4f}°", True)
            self.linkage.set_targets([(x, y)])
            if move:
                self.start_joint_move(theta0, theta1)
        except Exception as exc:
            self.ik_panel.set_result(f"IK failed: {exc}", False)
            self.linkage.set_targets([])

    def preview_coordinate_sequence(self, points: object) -> None:
        try:
            compiled = compile_cartesian_sequence(points, self.config.geometry, self.config.trajectory)
            self.coordinate_panel.set_validation(compiled)
            self.linkage.set_targets([(point.x_mm, point.y_mm) for point in compiled])
            self.log(f"Coordinate sequence preview validated: {len(compiled)} reachable points.")
        except Exception as exc:
            self.coordinate_panel.set_validation_error(str(exc))
            self.linkage.set_targets([])
            self.log(f"Coordinate sequence validation failed: {exc}")

    def start_coordinate_sequence(
        self, points: object, repeat_count: int, feedforward_enabled: bool
    ) -> None:
        if not self._connected_guard() or self.e_stop_active:
            return
        try:
            compiled = compile_cartesian_sequence(points, self.config.geometry, self.config.trajectory)
        except Exception as exc:
            self.coordinate_panel.set_validation_error(str(exc))
            self.linkage.set_targets([])
            return
        repeats = max(1, int(repeat_count))
        total = len(compiled) * repeats
        self.coordinate_panel.set_validation(compiled)
        self.coordinate_panel.set_running(True, total)
        self.linkage.set_targets([(point.x_mm, point.y_mm) for point in compiled])
        self.sequence_cancel = threading.Event()
        worker = CoordinateSequenceWorker(
            self.manager,
            copy.deepcopy(self.config),
            points,
            repeats,
            self.sequence_cancel,
            feedforward_enabled=bool(feedforward_enabled),
        )

        def done(ok: bool, message: str, payload: object) -> None:
            self.coordinate_panel.set_running(False, total)
            self.sequence_cancel = None
            if not ok:
                self.coordinate_panel.status.setText(message)

        # Keep the state-independent poller running throughout the sequence. Direct USB
        # transactions remain safe because ODriveManager serialises every short access.
        if not self._start_worker(
            "sequence", worker, pause_poll=False, done=done
        ):
            self.coordinate_panel.set_running(False, total)
            self.sequence_cancel = None

    def set_global_trapezoid_profile(
        self, max_vel_deg_s: float, max_accel_deg_s2: float, max_decel_deg_s2: float
    ) -> None:
        if self._hardware_busy():
            QMessageBox.warning(
                self,
                "Hardware busy",
                "Stop the active operation before changing the global trajectory profile.",
            )
            return
        try:
            from .models import TrajectoryConfig

            profile = TrajectoryConfig(
                max_vel_deg_s=float(max_vel_deg_s),
                max_accel_deg_s2=float(max_accel_deg_s2),
                max_decel_deg_s2=float(max_decel_deg_s2),
            )
            if min(
                profile.max_vel_deg_s,
                profile.max_accel_deg_s2,
                profile.max_decel_deg_s2,
            ) <= 0:
                raise ValueError("All trapezoid limits must be positive.")
            self.config.trajectory = profile
            self.store.save(self.config)
            self.config_panel.traj_vel.setValue(profile.max_vel_deg_s)
            self.config_panel.traj_acc.setValue(profile.max_accel_deg_s2)
            self.config_panel.traj_dec.setValue(profile.max_decel_deg_s2)
            self.log(
                "Global move trapezoid updated: "
                f"V={profile.max_vel_deg_s:.3f} deg/s, "
                f"A={profile.max_accel_deg_s2:.3f} deg/s², "
                f"D={profile.max_decel_deg_s2:.3f} deg/s²."
            )
        except Exception as exc:
            QMessageBox.critical(self, "Trajectory profile error", str(exc))

    def stop_coordinate_sequence(self) -> None:
        if self.sequence_cancel is not None:
            self.sequence_cancel.set()
            self.log("Coordinate sequence stop requested; the worker will hold the current pose.")
        else:
            self.log("Stop Sequence pressed; no coordinate sequence is active.")

    def compute_fk(self, theta0: float, theta1: float) -> None:
        try:
            result = forward_kinematics(theta0, theta1, self.config.geometry)
            e = result.end_effector
            self.fk_panel.set_result(
                f"E = ({e[0]:.3f}, {e[1]:.3f}) mm | P1={result.p1} | P2={result.p2}", True
            )
            self.linkage.set_targets([e])
        except Exception as exc:
            self.fk_panel.set_result(f"FK failed: {exc}", False)

    def use_current_angles(self) -> None:
        if not self.last_sample:
            self.fk_panel.set_result("No telemetry sample is available yet.", False)
            return
        theta0, theta1 = self.last_sample.theta_deg
        self.fk_panel.set_angles(theta0, theta1)
        self.compute_fk(theta0, theta1)

    def start_velocity_session(self, mode: str, target: object) -> None:
        if not self._connected_guard() or self.e_stop_active:
            return
        self.velocity_cancel = threading.Event()
        self.velocity_commands = VelocityCommandState()
        target_xy = target if isinstance(target, tuple) else None
        if mode == "position" and target_xy:
            self.linkage.set_targets([target_xy])
        else:
            self.linkage.set_targets([])
        worker = VelocityWorker(
            self.manager,
            copy.deepcopy(self.config),
            mode,
            self.velocity_commands,
            target_xy,
            self.velocity_cancel,
        )
        self.velocity_panel.set_running(True)

        def done(ok: bool, message: str, payload: object) -> None:
            self.velocity_panel.set_running(False)
            self.linkage.set_targets([])
            self.linkage.set_velocity_vector(None)
            self.velocity_cancel = None

        if not self._start_worker("velocity", worker, pause_poll=True, done=done):
            self.velocity_panel.set_running(False)

    def update_jog_command(self, vx: float, vy: float) -> None:
        self.velocity_commands.set(vx, vy)
        if "velocity" in self.workers and self.velocity_panel.mode.currentIndex() == 0:
            self.linkage.set_velocity_vector((vx, vy))
        else:
            self.linkage.set_velocity_vector(None)

    def stop_motion(self) -> None:
        events = [self.motion_cancel, self.velocity_cancel, self.step_cancel, self.sequence_cancel]
        signalled = False
        for event in events:
            if event is not None:
                event.set(); signalled = True
        if signalled:
            self.log("Stop Motion requested. Active worker will hold current position and exit cleanly.")
        else:
            self.log("Stop Motion pressed; no cancellable motion is active.")

    def emergency_stop(self) -> None:
        for event in (self.motion_cancel, self.velocity_cancel, self.step_cancel, self.sequence_cancel):
            if event is not None:
                event.set()
        ok, message = self.manager.emergency_stop()
        self.e_stop_active = True
        self.velocity_panel.set_running(False)
        self.linkage.set_velocity_vector(None)
        self.set_status("E-STOP ACTIVE", "error")
        self.log(f"EMERGENCY STOP: {message} Both axes are unpowered and will not move until Resume After E-Stop.")
        if not ok and self.manager.connected:
            QMessageBox.critical(self, "E-stop warning", message)

    def resume_after_estop(self) -> None:
        if not self._connected_guard():
            return
        worker = AxisActionWorker(self.manager, "resume")

        def done(ok: bool, message: str, payload: object) -> None:
            if ok:
                self.e_stop_active = False
                self.set_status("Connected / closed loop", "ok")
                self._reset_polling_state_in_place("closed-loop recovery after E-stop")

        # ODriveManager serialises the short state-change transactions, so the dedicated
        # poller can stay enabled and resume immediately between lock acquisitions.
        self._start_worker("resume", worker, pause_poll=False, done=done)

    def _ensure_error_dialog(self) -> ErrorReportDialog:
        if self.error_dialog is None:
            self.error_dialog = ErrorReportDialog(self)
            self.error_dialog.refresh_requested.connect(
                lambda: self.run_axis_action("show_errors")
            )
            self.error_dialog.clear_requested.connect(self._clear_errors_from_dialog)
        return self.error_dialog

    def _clear_errors_from_dialog(self) -> None:
        if not self._connected_guard():
            return
        worker = AxisActionWorker(self.manager, "clear_errors")

        def done(ok: bool, message: str, payload: object) -> None:
            dialog = self._ensure_error_dialog()
            if not ok:
                dialog.set_failure(message)
                return
            self.log("Errors cleared from the error-report window; refreshing registers.")
            # Wait until the clear worker has fully left the worker registry before refreshing.
            QTimer.singleShot(120, lambda: self.run_axis_action("show_errors"))

        if not self._start_worker(
            "action_clear_errors_dialog", worker, pause_poll=False, done=done
        ):
            self._ensure_error_dialog().set_failure(
                "Could not clear errors because another hardware operation is active."
            )

    def run_axis_action(self, action: str) -> None:
        if not self._connected_guard():
            return
        if action == "show_errors":
            dialog = self._ensure_error_dialog()
            dialog.set_loading()
            dialog.show()
        worker = AxisActionWorker(self.manager, action)

        def done(ok: bool, message: str, payload: object) -> None:
            if action == "show_errors":
                dialog = self._ensure_error_dialog()
                if ok:
                    dialog.set_report(payload)
                    dialog.show()
                    dialog.raise_()
                    dialog.activateWindow()
                else:
                    dialog.set_failure(message)
                    dialog.show()
                return
            if not ok:
                return
            if action == "read_flags" and isinstance(payload, dict):
                self.calibration_panel.show_flags(payload)
            elif action == "read_spi" and isinstance(payload, dict):
                self.config_panel.set_spi_from_hardware(payload)
            elif action == "read_hardware" and isinstance(payload, dict):
                self.config_panel.set_hardware_snapshot(payload)
                spi_values: dict[str, dict[str, int]] = {}
                for axis in (0, 1):
                    axis_data = payload.get(f"axis{axis}", {})
                    encoder = axis_data.get("encoder", {})
                    mode = encoder.get("mode")
                    cs_gpio = encoder.get("abs_spi_cs_gpio_pin")
                    if mode is not None and cs_gpio is not None:
                        spi_values[f"axis{axis}"] = {
                            "mode": int(mode),
                            "cs_gpio": int(cs_gpio),
                        }
                if len(spi_values) == 2:
                    self.config_panel.set_spi_from_hardware(spi_values)
                selected_axis = self.pid_panel.axis.currentIndex()
                selected = payload.get(f"axis{selected_axis}", {})
                controller = dict(selected.get("controller", {}))
                motor = selected.get("motor", {})
                if motor.get("current_lim") is not None:
                    controller["current_lim"] = motor["current_lim"]
                self.pid_panel.set_values(
                    {key: value for key, value in controller.items() if value is not None}
                )
                self.log(
                    "Hardware snapshot loaded. Geometry, gear ratio, direction and software "
                    "offset remain dashboard-only and must be entered/calibrated separately."
                )
            elif action == "enable_closed_loop":
                self.e_stop_active = False
                self._reset_polling_state_in_place("entering CLOSED_LOOP_CONTROL")

        # AxisActionWorker operations are short and ODriveManager serialises every USB access.
        # Keep polling enabled so the live encoder readout remains active in IDLE and while
        # the error window is being refreshed.
        started = self._start_worker(
            f"action_{action}", worker, pause_poll=False, done=done
        )
        if not started and action == "show_errors":
            self._ensure_error_dialog().set_failure(
                "Could not refresh errors because another hardware operation is active."
            )

    def run_calibration(self, axes: object) -> None:
        if not self._connected_guard():
            return
        axes_tuple = tuple(axes) if isinstance(axes, tuple) else tuple(axes)
        worker = CalibrationWorker(self.manager, axes_tuple)

        def done(ok: bool, message: str, payload: object) -> None:
            if axes_tuple == (0, 1):
                self.calibrated_clean_this_session = ok

        self._start_worker("calibration", worker, pause_poll=True, done=done)

    def sync_software_zero(self, reference_angles: object) -> None:
        if not self._connected_guard():
            return
        if not isinstance(reference_angles, dict):
            QMessageBox.warning(self, "Invalid reference", "Two physical reference angles are required.")
            return
        refs = {0: float(reference_angles[0]), 1: float(reference_angles[1])}
        if not self._confirm(
            "Sync software offsets",
            "Confirm the robot is stationary and physically positioned at:\n"
            f"axis0 = {refs[0]:.6f}°\naxis1 = {refs[1]:.6f}°\n\n"
            "This changes the dashboard offset_turns values only. The ODrive will not move.\n\n"
            f"The app will sample encoder position for about "
            f"{self.config.encoder_noise.sync_sample_duration_s:.2f} seconds and ignore noisy "
            "raw vel_estimate values. Do not touch the arm during that sampling window.",
        ):
            return
        worker = SyncZeroWorker(self.manager, copy.deepcopy(self.config), refs)

        def done(ok: bool, message: str, payload: object) -> None:
            if not ok or not isinstance(payload, dict):
                return
            offsets = payload["offset_turns"]
            self.config.axes[0].offset_turns = float(offsets[0])
            self.config.axes[1].offset_turns = float(offsets[1])
            self.store.save(self.config)
            self.config_panel.set_software_offsets(
                {0: self.config.axes[0].offset_turns, 1: self.config.axes[1].offset_turns}
            )
            self.log(f"Software-reference offsets saved immediately to {self.store.path}.")
            diagnostics = payload.get("diagnostics")
            if isinstance(diagnostics, dict):
                for axis in (0, 1):
                    data = diagnostics.get(axis, diagnostics.get(str(axis), {}))
                    if isinstance(data, dict):
                        self.log(
                            f"axis{axis} sync diagnostics: samples={data.get('samples')}, "
                            f"position span={float(data.get('robust_span_turns', 0.0)):.7f} turns, "
                            f"drift={float(data.get('drift_turns_s', 0.0)):+.7f} turns/s, "
                            f"raw velocity median={float(data.get('raw_velocity_median_turns_s', 0.0)):+.7f} turns/s."
                        )

            verified = payload.get("verified_angle_deg", refs)
            try:
                theta = (float(verified[0]), float(verified[1]))
                fk = forward_kinematics(theta[0], theta[1], self.config.geometry)
                previous = self.last_sample
                raw_turns_payload = payload.get("raw_turns", (0.0, 0.0))
                raw_velocity_payload = payload.get("velocity_turns_s", (0.0, 0.0))
                sample = TelemetrySample(
                    t=0.0,
                    pos_deg=theta,
                    vel_raw_deg_s=previous.vel_raw_deg_s if previous else (0.0, 0.0),
                    vel_filtered_deg_s=previous.vel_filtered_deg_s if previous else (0.0, 0.0),
                    current_a=previous.current_a if previous else (0.0, 0.0),
                    theta_deg=theta,
                    p1=fk.p1,
                    p2=fk.p2,
                    end_effector=fk.end_effector,
                    raw_pos_turns=(float(raw_turns_payload[0]), float(raw_turns_payload[1])),
                    filtered_pos_turns=(float(raw_turns_payload[0]), float(raw_turns_payload[1])),
                    raw_vel_turns_s=(
                        float(raw_velocity_payload[0]),
                        float(raw_velocity_payload[1]),
                    ),
                    motion_estimate_turns_s=(0.0, 0.0),
                    stationary=(True, True),
                    axis_state=previous.axis_state if previous else (0, 0),
                    axis_error=previous.axis_error if previous else (0, 0),
                    motor_error=previous.motor_error if previous else (0, 0),
                    encoder_error=previous.encoder_error if previous else (0, 0),
                )
                self.linkage.set_sample(sample)
                self.live_position.set_sample(sample)
                self.last_sample = sample
            except Exception as exc:
                self.log(f"Offsets saved, but immediate FK refresh is unreachable: {exc}")

            # PollWorker owns a deep copy of the old mapping. It must be restarted; merely
            # unpausing it would immediately overwrite the synced display with stale angles.
            self.stop_polling(restart=True)

        self._start_worker("sync_zero", worker, pause_poll=True, done=done)

    def _confirm(self, title: str, text: str) -> bool:
        result = QMessageBox.warning(
            self,
            title,
            text,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        return result == QMessageBox.StandardButton.Yes

    def run_flash_action(self, action: str, payload: object) -> None:
        if not self._connected_guard():
            return
        if self._hardware_busy():
            QMessageBox.warning(self, "Hardware busy", "Stop the active operation before saving to flash.")
            return
        if action == "mark_calibrated" and not self.calibrated_clean_this_session:
            QMessageBox.warning(
                self,
                "Calibration required",
                "Run a clean Calibrate Both in this app session immediately before marking the axes calibrated.",
            )
            return
        messages = {
            "mark_calibrated": "This will write pre-calibrated/startup flags and reboot the ODrive. Continue?",
            "clear_precalibrated": "This will clear pre-calibrated flags and reboot the ODrive. Continue?",
            "apply_spi": "This changes SPI encoder mode/CS, saves to flash and reboots. Recalibration may be required. Continue?",
            "save_only": "This saves the current ODrive configuration to flash and reboots. Continue?",
        }
        if not self._confirm("Confirm ODrive flash save", messages.get(action, "Save and reboot?")):
            return
        self.stop_polling()
        worker = FlashConfigWorker(self.manager, action, payload if isinstance(payload, dict) else {})

        def done(ok: bool, message: str, result: object) -> None:
            self.connect_button.setText("Connect")
            self.set_status("Disconnected after reboot", "warn")
            self.calibrated_clean_this_session = False
            if ok:
                QMessageBox.information(
                    self,
                    "ODrive rebooting",
                    "The ODrive configuration was saved and the USB connection was dropped as expected. "
                    "Wait for the controller to reboot, then press Connect.",
                )

        self._start_worker("flash", worker, pause_poll=True, done=done)

    def pid_read(self, axis: int) -> None:
        if not self._connected_guard(): return
        worker = PidWorker(self.manager, axis, "read")
        self._start_worker(
            "pid_read", worker, pause_poll=True,
            done=lambda ok, msg, payload: self.pid_panel.set_values(payload) if ok and isinstance(payload, dict) else None,
        )

    def pid_apply(self, axis: int, values: object) -> None:
        if not self._connected_guard() or not isinstance(values, dict): return
        worker = PidWorker(self.manager, axis, "apply", values)
        self._start_worker("pid_apply", worker, pause_poll=True)

    def run_step_response(self, axis: int, step_turns: float, duration_s: float) -> None:
        if not self._connected_guard() or self.e_stop_active: return
        self.step_cancel = threading.Event()
        worker = StepResponseWorker(
            self.manager, copy.deepcopy(self.config), axis, step_turns, duration_s, self.step_cancel
        )

        def done(ok: bool, message: str, payload: object) -> None:
            self.step_cancel = None
            if ok and payload is not None:
                self.pid_panel.add_step_result(payload)

        self._start_worker("step", worker, pause_poll=True, done=done)

    def apply_config(self, config: object) -> None:
        if not isinstance(config, DashboardConfig):
            return
        if self._hardware_busy():
            QMessageBox.warning(self, "Hardware busy", "Stop the active operation before changing runtime configuration.")
            return
        try:
            config.validate()
            self.config = config
            self.store.save(self.config)
            self._apply_config_to_runtime()
            self.log(f"Dashboard config applied and auto-saved to {self.store.path}.")
            if self.manager.connected:
                self.stop_polling(restart=True)
        except Exception as exc:
            QMessageBox.critical(self, "Config error", str(exc))

    def save_config_now(self) -> None:
        if self._hardware_busy():
            QMessageBox.warning(self, "Hardware busy", "Stop the active operation before changing runtime configuration.")
            return
        try:
            self.config = self.config_panel.collect_config()
            self.store.save(self.config)
            self._apply_config_to_runtime()
            self.log(f"Dashboard config saved to {self.store.path}.")
            if self.manager.connected:
                self.stop_polling(restart=True)
        except Exception as exc:
            QMessageBox.critical(self, "Save failed", str(exc))

    def reload_config(self) -> None:
        if self._hardware_busy():
            QMessageBox.warning(self, "Hardware busy", "Stop the active operation before reloading runtime configuration.")
            return
        try:
            self.config = self.store.load()
            self.config_panel.load_config(self.config)
            self._apply_config_to_runtime()
            self.log(f"Dashboard config reloaded from {self.store.path}.")
            if self.manager.connected:
                self.stop_polling(restart=True)
        except Exception as exc:
            QMessageBox.critical(self, "Reload failed", str(exc))

    def _apply_config_to_runtime(self) -> None:
        self.linkage.set_config(self.config)
        self.ik_panel.set_elbows(self.config.geometry.elbow1, self.config.geometry.elbow2)
        self.coordinate_panel.set_profile_defaults(self.config.trajectory)
        self.plots.set_window(self.config.display.telemetry_window_s)
        f0 = self.config.filters[0]; f1 = self.config.filters[1]
        self.velocity_panel.set_filter_status(
            f"DSP: axis0={f0.type} | axis1={f1.type}; state resets on each session/re-enable"
        )

    def closeEvent(self, event: QCloseEvent) -> None:  # type: ignore[override]
        self.stop_motion()
        self.stop_polling()
        for _, worker, _, _ in list(self.workers.values()):
            if hasattr(worker, "cancel_event"):
                worker.cancel_event.set()
        if self.manager.connected:
            self.manager.emergency_stop()
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            worker_running = any(thread.isRunning() for thread, *_ in self.workers.values())
            poll_running = bool(self.poll_thread and self.poll_thread.isRunning())
            if not worker_running and not poll_running:
                break
            from PySide6.QtWidgets import QApplication

            QApplication.processEvents()
            time.sleep(0.02)
        event.accept()
