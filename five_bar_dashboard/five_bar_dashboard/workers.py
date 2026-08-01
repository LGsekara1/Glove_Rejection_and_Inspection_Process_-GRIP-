"""Qt worker objects for all hardware operations and control loops."""
from __future__ import annotations

import math
import threading
import time
from typing import Any

import numpy as np
from PySide6.QtCore import QObject, Signal, Slot

from .backend import ODriveManager
from .constants import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_IDLE,
    AXIS_STATE_MOTOR_CALIBRATION,
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    INPUT_MODE_TRAP_TRAJ,
)
from .conversion import (
    abs_deg_rate_to_turn_rate,
    angle_deg_to_turns,
    offset_for_reference_angle,
    deg_per_s_to_turns_per_s,
    turns_per_s_to_deg_per_s,
    turns_to_angle_deg,
)
from .filters import make_velocity_filter
from .encoder_noise import PositionMotionEstimator, robust_position_estimate
from .kinematics import (
    KinematicsError,
    cartesian_to_joint_velocity,
    forward_kinematics,
    slew_limit_vector,
)
from .models import DashboardConfig, StepResponseResult, TelemetrySample
from .trajectory import synchronise_two_axes_asymmetric
from .sequence import compile_cartesian_sequence


class BaseWorker(QObject):
    log = Signal(str)
    status = Signal(str, str)  # message, severity
    telemetry = Signal(object)
    progress = Signal(object)
    finished = Signal(bool, str, object)

    def __init__(self, cancel_event: threading.Event | None = None) -> None:
        super().__init__()
        self.cancel_event = cancel_event or threading.Event()

    def cancelled(self) -> bool:
        return self.cancel_event.is_set()

    def emit_finished(self, ok: bool, message: str, payload: object = None) -> None:
        self.finished.emit(ok, message, payload)


class TelemetrySampler:
    def __init__(self, manager: ODriveManager, config: DashboardConfig, sample_rate_hz: float) -> None:
        self.manager = manager
        self.config = config
        self.origin = time.monotonic()
        self.last_time = self.origin
        self.filters = {
            axis: make_velocity_filter(config.filters[axis], sample_rate_hz) for axis in (0, 1)
        }
        noise = config.encoder_noise
        self.position_estimators = {
            axis: PositionMotionEstimator(
                sample_rate_hz=sample_rate_hz,
                median_window=noise.position_median_window,
                motion_window_s=noise.motion_window_s,
                deadband_turns_s=noise.motion_deadband_turns_s,
            )
            for axis in (0, 1)
        }

    def reset(self) -> None:
        self.origin = time.monotonic()
        self.last_time = self.origin
        for filt in self.filters.values():
            filt.reset()
        for estimator in self.position_estimators.values():
            estimator.reset()

    def sample_with_raw(self) -> tuple[TelemetrySample, tuple[Any, Any]]:
        now = time.monotonic()
        dt = max(1e-6, now - self.last_time)
        self.last_time = now
        with self.manager.access() as drive:
            raw = (
                self.manager.read_axis_snapshot_locked(drive, 0),
                self.manager.read_axis_snapshot_locked(drive, 1),
            )

        position_results = tuple(
            self.position_estimators[axis].update(now, raw[axis].pos_turns)
            for axis in (0, 1)
        )
        filtered_position = tuple(result[0] for result in position_results)
        motion_estimate = tuple(result[1] for result in position_results)
        stationary = tuple(result[2] for result in position_results)
        theta = tuple(
            turns_to_angle_deg(
                filtered_position[axis],
                self.config.home_angle_deg[axis],
                self.config.axes[axis],
            )
            for axis in (0, 1)
        )
        vel_raw = tuple(
            turns_per_s_to_deg_per_s(raw[axis].vel_turns_s, self.config.axes[axis])
            for axis in (0, 1)
        )
        vel_filtered = tuple(
            self.filters[axis].update(vel_raw[axis], dt) for axis in (0, 1)
        )
        p1 = p2 = end = None
        try:
            fk = forward_kinematics(theta[0], theta[1], self.config.geometry)
            p1, p2, end = fk.p1, fk.p2, fk.end_effector
        except KinematicsError:
            pass
        sample = TelemetrySample(
            t=now - self.origin,
            pos_deg=theta,
            vel_raw_deg_s=vel_raw,
            vel_filtered_deg_s=vel_filtered,
            current_a=(raw[0].current_a, raw[1].current_a),
            theta_deg=theta,
            p1=p1,
            p2=p2,
            end_effector=end,
            raw_pos_turns=(raw[0].pos_turns, raw[1].pos_turns),
            filtered_pos_turns=(filtered_position[0], filtered_position[1]),
            raw_vel_turns_s=(raw[0].vel_turns_s, raw[1].vel_turns_s),
            motion_estimate_turns_s=(motion_estimate[0], motion_estimate[1]),
            stationary=(stationary[0], stationary[1]),
            axis_state=(raw[0].current_state, raw[1].current_state),
            axis_error=(raw[0].axis_error, raw[1].axis_error),
            motor_error=(raw[0].motor_error, raw[1].motor_error),
            encoder_error=(raw[0].encoder_error, raw[1].encoder_error),
        )
        return sample, raw

    def sample(self) -> TelemetrySample:
        return self.sample_with_raw()[0]


class ConnectWorker(BaseWorker):
    def __init__(self, manager: ODriveManager) -> None:
        super().__init__()
        self.manager = manager

    @Slot()
    def run(self) -> None:
        self.status.emit("Connecting...", "info")
        self.log.emit("Searching for ODrive (6 attempts, 5 s hard timeout each)...")
        try:
            drive = self.manager.connect(attempts=6, hard_timeout_s=5.0)
            label = "simulator" if self.manager.simulate else "ODrive v3.6"
            self.log.emit(f"Connected to {label}.")
            self.status.emit("Connected", "ok")
            self.emit_finished(True, "Connected", drive)
        except Exception as exc:
            self.status.emit("Connection failed", "error")
            self.log.emit(f"Connection failed: {exc}")
            self.emit_finished(False, str(exc))


class PollWorker(BaseWorker):
    def __init__(self, manager: ODriveManager, config: DashboardConfig, hz: float = 50.0) -> None:
        super().__init__()
        self.manager = manager
        self.config = config
        self.hz = max(1.0, hz)
        self.pause_event = threading.Event()
        # Resetting filters/position estimators must not require killing and recreating
        # the polling thread.  A thread-safe Event lets the GUI request an in-place
        # reset while state-independent polling remains alive across axis state changes.
        self.reset_event = threading.Event()

    def set_paused(self, paused: bool) -> None:
        if paused:
            self.pause_event.set()
        else:
            self.pause_event.clear()

    def request_reset(self) -> None:
        """Reset DSP state on the polling thread without stopping telemetry."""
        self.reset_event.set()

    @Slot()
    def run(self) -> None:
        sampler = TelemetrySampler(self.manager, self.config, self.hz)
        period = 1.0 / self.hz
        consecutive_failures = 0
        self.log.emit(
            "State-independent telemetry polling started: encoder position is read in "
            "IDLE, CLOSED_LOOP_CONTROL and after E-stop."
        )
        while not self.cancelled():
            tick = time.monotonic()

            # Apply a requested state reset inside this worker thread.  Crucially, this
            # does not cancel/restart the polling QThread when an axis enters closed loop.
            if self.reset_event.is_set():
                sampler.reset()
                self.reset_event.clear()
                consecutive_failures = 0
                self.log.emit(
                    "Telemetry filter/motion-estimator state reset in place; polling remained active."
                )

            if self.pause_event.is_set():
                self.cancel_event.wait(0.02)
                continue
            try:
                self.telemetry.emit(sampler.sample())
                if consecutive_failures:
                    self.log.emit(
                        f"Telemetry recovered after {consecutive_failures} transient read failure(s)."
                    )
                consecutive_failures = 0
            except Exception as exc:
                consecutive_failures += 1
                # Axis-state transitions and Fibre/libusb contention can produce a burst of
                # read timeouts.  The state-independent poller must remain alive and retry;
                # it is stopped only by its explicit cancel event (disconnect/app shutdown).
                if consecutive_failures in {1, 3, 6} or consecutive_failures % 20 == 0:
                    self.log.emit(
                        f"Telemetry read failed ({consecutive_failures} consecutive); "
                        f"poller remains active and will retry: {exc}"
                    )
                if consecutive_failures == 6:
                    self.status.emit("Telemetry interrupted; retrying", "warn")
                self.cancel_event.wait(min(0.50, max(period * 2.0, 0.02)))
                continue
            delay = period - (time.monotonic() - tick)
            if delay > 0:
                self.cancel_event.wait(delay)
        self.log.emit("State-independent telemetry polling stopped.")
        self.emit_finished(True, "Polling stopped")


class MoveWorker(BaseWorker):
    def __init__(
        self,
        manager: ODriveManager,
        config: DashboardConfig,
        target_angles_deg: tuple[float, float],
        cancel_event: threading.Event,
    ) -> None:
        super().__init__(cancel_event)
        self.manager = manager
        self.config = config
        self.target_angles_deg = target_angles_deg

    def _hold_current(self) -> None:
        with self.manager.access() as drive:
            for axis_index in (0, 1):
                axis = self.manager.axis(drive, axis_index)
                current = float(axis.encoder.pos_estimate)
                axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                axis.controller.input_pos = current

    @Slot()
    def run(self) -> None:
        self.status.emit("Firmware trajectory move running", "info")
        try:
            with self.manager.access() as drive:
                self.manager.require_closed_loop(drive)
                current = tuple(
                    float(self.manager.axis(drive, i).encoder.pos_estimate) for i in (0, 1)
                )
                firmware_caps = tuple(
                    abs(float(self.manager.axis(drive, i).controller.config.vel_limit))
                    for i in (0, 1)
                )
                targets = tuple(
                    angle_deg_to_turns(
                        self.target_angles_deg[i],
                        self.config.home_angle_deg[i],
                        self.config.axes[i],
                    )
                    for i in (0, 1)
                )
                distances = tuple(abs(targets[i] - current[i]) for i in (0, 1))
                velocities = tuple(
                    min(
                        abs_deg_rate_to_turn_rate(
                            self.config.trajectory.max_vel_deg_s, self.config.axes[i]
                        ),
                        0.95 * firmware_caps[i],
                    )
                    for i in (0, 1)
                )
                accelerations = tuple(
                    abs_deg_rate_to_turn_rate(
                        self.config.trajectory.max_accel_deg_s2, self.config.axes[i]
                    )
                    for i in (0, 1)
                )
                decelerations = tuple(
                    abs_deg_rate_to_turn_rate(
                        self.config.trajectory.max_decel_deg_s2, self.config.axes[i]
                    )
                    for i in (0, 1)
                )
                synced_v, synced_a, synced_d, duration = synchronise_two_axes_asymmetric(
                    distances, velocities, accelerations, decelerations
                )
                for i in (0, 1):
                    axis = self.manager.axis(drive, i)
                    axis.trap_traj.config.vel_limit = max(1e-5, synced_v[i])
                    axis.trap_traj.config.accel_limit = max(1e-5, synced_a[i])
                    axis.trap_traj.config.decel_limit = max(1e-5, synced_d[i])
                    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                    axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
                for i in (0, 1):
                    self.manager.axis(drive, i).controller.input_pos = targets[i]

            self.log.emit(
                "Move started: target angles "
                f"({self.target_angles_deg[0]:.3f}°, {self.target_angles_deg[1]:.3f}°), "
                f"synchronised duration {duration:.3f} s."
            )
            sampler = TelemetrySampler(self.manager, self.config, 50.0)
            timeout = 1.6 * duration + 0.6
            start = time.monotonic()
            while True:
                if self.cancelled():
                    self._hold_current()
                    self.log.emit("Move cancelled; both axes are holding current position.")
                    self.status.emit("Move cancelled", "warn")
                    self.emit_finished(False, "Move cancelled")
                    return
                sample, raw = sampler.sample_with_raw()
                self.telemetry.emit(sample)
                position_ok = all(abs(raw[i].pos_turns - targets[i]) <= 0.01 for i in (0, 1))
                velocity_ok = all(
                    abs(
                        deg_per_s_to_turns_per_s(
                            sample.vel_filtered_deg_s[i], self.config.axes[i]
                        )
                    )
                    < 0.02
                    for i in (0, 1)
                )
                if position_ok and velocity_ok:
                    self.log.emit("Firmware trajectory move completed and settled.")
                    self.status.emit("Move complete", "ok")
                    self.emit_finished(True, "Move complete")
                    return
                if time.monotonic() - start > timeout:
                    self._hold_current()
                    raise TimeoutError(
                        f"Move did not settle within {timeout:.2f} s; current pose is being held."
                    )
                time.sleep(0.02)
        except Exception as exc:
            self.status.emit("Move failed", "error")
            self.log.emit(f"Move failed: {exc}")
            self.emit_finished(False, str(exc))

class CoordinateSequenceWorker(BaseWorker):
    """Run Cartesian waypoints as discrete ODrive firmware trap-trajectory moves."""

    def __init__(
        self,
        manager: ODriveManager,
        config: DashboardConfig,
        waypoints: object,
        repeat_count: int,
        cancel_event: threading.Event,
        feedforward_enabled: bool = False,
    ) -> None:
        super().__init__(cancel_event)
        self.manager = manager
        self.config = config
        self.waypoints = waypoints
        self.repeat_count = max(1, int(repeat_count))
        self.feedforward_enabled = bool(feedforward_enabled)

    def _set_feedforward_locked(
        self, axis: Any, velocity_ff_turns_s: float, torque_ff_nm: float
    ) -> tuple[float, float]:
        """Apply bounded additive feedforward while the manager lock is held."""
        velocity_ff_turns_s = float(velocity_ff_turns_s)
        torque_ff_nm = float(torque_ff_nm)
        controller = axis.controller
        if not hasattr(controller, "input_vel") and abs(velocity_ff_turns_s) > 1e-12:
            raise RuntimeError("This ODrive firmware does not expose controller.input_vel feedforward.")
        if not hasattr(controller, "input_torque") and abs(torque_ff_nm) > 1e-12:
            raise RuntimeError("This ODrive firmware does not expose controller.input_torque feedforward.")

        vel_limit = abs(float(controller.config.vel_limit))
        applied_velocity = max(-0.95 * vel_limit, min(0.95 * vel_limit, velocity_ff_turns_s))

        applied_torque = torque_ff_nm
        if abs(torque_ff_nm) > 1e-12:
            motor_cfg = axis.motor.config
            torque_constant = getattr(motor_cfg, "torque_constant", None)
            current_lim = getattr(motor_cfg, "current_lim", None)
            if torque_constant is None or current_lim is None:
                raise RuntimeError(
                    "Cannot safely bound non-zero torque feedforward because current_lim or "
                    "torque_constant is unavailable on this firmware."
                )
            torque_constant = abs(float(torque_constant))
            current_lim = abs(float(current_lim))
            if not math.isfinite(torque_constant) or torque_constant <= 0.0:
                raise RuntimeError("Invalid motor torque_constant; refusing torque feedforward.")
            max_torque = 0.80 * current_lim * torque_constant
            applied_torque = max(-max_torque, min(max_torque, torque_ff_nm))

        if hasattr(controller, "input_vel"):
            controller.input_vel = applied_velocity
        if hasattr(controller, "input_torque"):
            controller.input_torque = applied_torque
        return float(applied_velocity), float(applied_torque)

    def _clear_feedforward(self, *, velocity_only: bool = False) -> None:
        try:
            with self.manager.access() as drive:
                for axis_index in (0, 1):
                    controller = self.manager.axis(drive, axis_index).controller
                    if hasattr(controller, "input_vel"):
                        controller.input_vel = 0.0
                    if not velocity_only and hasattr(controller, "input_torque"):
                        controller.input_torque = 0.0
        except Exception as exc:
            self.log.emit(f"Could not clear sequence feedforward: {exc}")

    def _hold_current(self) -> None:
        try:
            with self.manager.access() as drive:
                for axis_index in (0, 1):
                    axis = self.manager.axis(drive, axis_index)
                    current = float(axis.encoder.pos_estimate)
                    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                    if hasattr(axis.controller, "input_vel"):
                        axis.controller.input_vel = 0.0
                    if hasattr(axis.controller, "input_torque"):
                        axis.controller.input_torque = 0.0
                    axis.controller.input_pos = current
        except Exception as exc:
            self.log.emit(f"Could not hold current position after sequence stop: {exc}")

    def _command_waypoint(
        self, point: object
    ) -> tuple[tuple[float, float], float, dict[str, tuple[float, float]]]:
        target_angles_deg = (
            float(getattr(point, "theta0_deg")),
            float(getattr(point, "theta1_deg")),
        )
        with self.manager.access() as drive:
            self.manager.require_closed_loop(drive)
            current = tuple(
                float(self.manager.axis(drive, i).encoder.pos_estimate) for i in (0, 1)
            )
            firmware_caps = tuple(
                abs(float(self.manager.axis(drive, i).controller.config.vel_limit))
                for i in (0, 1)
            )
            targets = tuple(
                angle_deg_to_turns(
                    target_angles_deg[i],
                    self.config.home_angle_deg[i],
                    self.config.axes[i],
                )
                for i in (0, 1)
            )
            distances = tuple(abs(targets[i] - current[i]) for i in (0, 1))
            requested_vel = float(getattr(point, "max_vel_deg_s"))
            requested_accel = float(getattr(point, "max_accel_deg_s2"))
            requested_decel = float(getattr(point, "max_decel_deg_s2"))
            velocities = tuple(
                min(
                    abs_deg_rate_to_turn_rate(requested_vel, self.config.axes[i]),
                    0.95 * firmware_caps[i],
                )
                for i in (0, 1)
            )
            accelerations = tuple(
                abs_deg_rate_to_turn_rate(requested_accel, self.config.axes[i])
                for i in (0, 1)
            )
            decelerations = tuple(
                abs_deg_rate_to_turn_rate(requested_decel, self.config.axes[i])
                for i in (0, 1)
            )
            synced_v, synced_a, synced_d, duration = synchronise_two_axes_asymmetric(
                distances, velocities, accelerations, decelerations
            )
            requested_velocity_ff = (
                float(getattr(point, "velocity_ff0_turns_s")),
                float(getattr(point, "velocity_ff1_turns_s")),
            ) if self.feedforward_enabled else (0.0, 0.0)
            requested_torque_ff = (
                float(getattr(point, "torque_ff0_nm")),
                float(getattr(point, "torque_ff1_nm")),
            ) if self.feedforward_enabled else (0.0, 0.0)
            applied_ff: list[tuple[float, float]] = []
            for i in (0, 1):
                axis = self.manager.axis(drive, i)
                axis.trap_traj.config.vel_limit = max(1e-5, synced_v[i])
                axis.trap_traj.config.accel_limit = max(1e-5, synced_a[i])
                axis.trap_traj.config.decel_limit = max(1e-5, synced_d[i])
                axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
                applied_ff.append(
                    self._set_feedforward_locked(
                        axis, requested_velocity_ff[i], requested_torque_ff[i]
                    )
                )
            for i in (0, 1):
                self.manager.axis(drive, i).controller.input_pos = targets[i]
        applied = {
            "vel_turns_s": (float(synced_v[0]), float(synced_v[1])),
            "accel_turns_s2": (float(synced_a[0]), float(synced_a[1])),
            "decel_turns_s2": (float(synced_d[0]), float(synced_d[1])),
            "velocity_ff_turns_s": (float(applied_ff[0][0]), float(applied_ff[1][0])),
            "torque_ff_nm": (float(applied_ff[0][1]), float(applied_ff[1][1])),
        }
        return (float(targets[0]), float(targets[1])), float(duration), applied

    def _wait_until_settled(
        self, targets: tuple[float, float], duration_s: float
    ) -> None:
        rate_hz = 50.0
        period = 1.0 / rate_hz
        noise = self.config.encoder_noise
        estimators = {
            axis: PositionMotionEstimator(
                sample_rate_hz=rate_hz,
                median_window=noise.position_median_window,
                motion_window_s=noise.motion_window_s,
                deadband_turns_s=noise.motion_deadband_turns_s,
            )
            for axis in (0, 1)
        }
        timeout = max(0.8, 1.6 * duration_s + 0.8)
        start = time.monotonic()
        stable_since: float | None = None
        velocity_ff_cleared = False
        while True:
            if self.cancelled():
                raise InterruptedError("Coordinate sequence cancelled by user.")
            tick = time.monotonic()
            if not velocity_ff_cleared and tick - start >= max(0.0, duration_s):
                self._clear_feedforward(velocity_only=True)
                velocity_ff_cleared = True
                self.log.emit("Waypoint moving phase ended; velocity feedforward reset to zero before settling.")
            with self.manager.access() as drive:
                snapshots = tuple(
                    self.manager.read_axis_snapshot_locked(drive, i) for i in (0, 1)
                )
            for axis, snapshot in enumerate(snapshots):
                if snapshot.axis_error or snapshot.motor_error or snapshot.encoder_error:
                    raise RuntimeError(
                        f"axis{axis} fault during sequence: axis=0x{snapshot.axis_error:X}, "
                        f"motor=0x{snapshot.motor_error:X}, encoder=0x{snapshot.encoder_error:X}"
                    )
                if snapshot.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                    raise RuntimeError(
                        f"axis{axis} left CLOSED_LOOP_CONTROL during sequence "
                        f"(state={snapshot.current_state})."
                    )
            estimates = tuple(
                estimators[axis].update(tick, snapshots[axis].pos_turns)
                for axis in (0, 1)
            )
            position_ok = all(
                abs(estimates[axis][0] - targets[axis]) <= 0.01 for axis in (0, 1)
            )
            stationary = all(estimates[axis][2] for axis in (0, 1))
            if position_ok and stationary:
                if stable_since is None:
                    stable_since = tick
                elif tick - stable_since >= 0.12:
                    if not velocity_ff_cleared:
                        self._clear_feedforward(velocity_only=True)
                    return
            else:
                stable_since = None
            if tick - start > timeout:
                raise TimeoutError(
                    f"Waypoint did not settle within {timeout:.2f} s. "
                    "Position-derived motion detection was used so raw encoder velocity noise "
                    "did not block completion."
                )
            remaining = period - (time.monotonic() - tick)
            if remaining > 0:
                self.cancel_event.wait(remaining)

    @Slot()
    def run(self) -> None:
        try:
            compiled = compile_cartesian_sequence(
                self.waypoints, self.config.geometry, self.config.trajectory
            )
            if self.repeat_count > 1000:
                raise ValueError("Repeat count is limited to 1000.")
            total = len(compiled) * self.repeat_count
            completed = 0
            self.status.emit("Coordinate sequence running", "info")
            self.log.emit(
                f"Coordinate sequence validated: {len(compiled)} points, "
                f"repeat={self.repeat_count}, total moves={total}, "
                f"feedforward={'ARMED' if self.feedforward_enabled else 'DISABLED'}."
            )
            for cycle in range(1, self.repeat_count + 1):
                for row_index, point in enumerate(compiled):
                    if self.cancelled():
                        raise InterruptedError("Coordinate sequence cancelled by user.")
                    text = (
                        f"Cycle {cycle}/{self.repeat_count}: moving to P{point.index} "
                        f"({point.x_mm:.3f}, {point.y_mm:.3f}) mm"
                    )
                    self.progress.emit(
                        {
                            "completed": completed,
                            "total": total,
                            "row_index": row_index,
                            "phase": "Moving",
                            "text": text,
                        }
                    )
                    targets, duration, applied = self._command_waypoint(point)
                    self.log.emit(
                        f"{text}; joint target=({point.theta0_deg:.3f} deg, "
                        f"{point.theta1_deg:.3f} deg), requested trap="
                        f"V{point.max_vel_deg_s:.3f}/A{point.max_accel_deg_s2:.3f}/"
                        f"D{point.max_decel_deg_s2:.3f} deg units, "
                        f"synchronised duration={duration:.3f} s, "
                        f"applied profiles/feedforward={applied}."
                    )
                    self._wait_until_settled(targets, duration)
                    completed += 1
                    self.progress.emit(
                        {
                            "completed": completed,
                            "total": total,
                            "row_index": row_index,
                            "phase": "Reached",
                            "text": f"Reached P{point.index}; {completed}/{total} moves complete.",
                        }
                    )
                    if point.dwell_s > 0.0:
                        self.progress.emit(
                            {
                                "completed": completed,
                                "total": total,
                                "row_index": row_index,
                                "phase": "Dwelling",
                                "text": f"Dwelling at P{point.index} for {point.dwell_s:.3f} s.",
                            }
                        )
                        if self.cancel_event.wait(point.dwell_s):
                            raise InterruptedError("Coordinate sequence cancelled during dwell.")
            self.progress.emit(
                {
                    "completed": total,
                    "total": total,
                    "row_index": len(compiled) - 1,
                    "phase": "Reached",
                    "text": f"Coordinate sequence complete: {total}/{total} moves.",
                }
            )
            self._clear_feedforward()
            self.status.emit("Coordinate sequence complete", "ok")
            self.log.emit("Coordinate sequence completed successfully; all feedforward commands reset to zero.")
            self.emit_finished(
                True,
                "Coordinate sequence complete",
                {"completed": total, "total": total, "compiled": compiled},
            )
        except InterruptedError as exc:
            self._hold_current()
            self.status.emit("Coordinate sequence stopped", "warn")
            self.log.emit(f"{exc} Both axes are holding current position.")
            self.emit_finished(False, str(exc), None)
        except Exception as exc:
            self._hold_current()
            self.status.emit("Coordinate sequence failed", "error")
            self.log.emit(f"Coordinate sequence failed: {exc}")
            self.emit_finished(False, str(exc), None)


class RawNudgeWorker(BaseWorker):
    def __init__(
        self, manager: ODriveManager, deltas_turns: tuple[float, float]
    ) -> None:
        super().__init__()
        self.manager = manager
        self.deltas_turns = deltas_turns

    @Slot()
    def run(self) -> None:
        try:
            if any(abs(v) > 0.05 for v in self.deltas_turns):
                raise ValueError("Raw-turn nudges are limited to ±0.05 turns per axis.")
            with self.manager.access() as drive:
                self.manager.require_closed_loop(drive)
                for i in (0, 1):
                    axis = self.manager.axis(drive, i)
                    current = float(axis.encoder.pos_estimate)
                    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                    axis.controller.input_pos = current + self.deltas_turns[i]
            self.log.emit(
                f"Raw instant nudge sent: axis0={self.deltas_turns[0]:+.5f}, "
                f"axis1={self.deltas_turns[1]:+.5f} turns."
            )
            self.emit_finished(True, "Raw nudge sent")
        except Exception as exc:
            self.log.emit(f"Raw nudge failed: {exc}")
            self.emit_finished(False, str(exc))


class VelocityCommandState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.vx = 0.0
        self.vy = 0.0
        self.last_update = 0.0

    def set(self, vx: float, vy: float) -> None:
        with self._lock:
            self.vx = float(vx)
            self.vy = float(vy)
            self.last_update = time.monotonic()

    def snapshot(self) -> tuple[float, float, float]:
        with self._lock:
            return self.vx, self.vy, self.last_update


class VelocityWorker(BaseWorker):
    def __init__(
        self,
        manager: ODriveManager,
        config: DashboardConfig,
        mode: str,
        command_state: VelocityCommandState,
        target_xy: tuple[float, float] | None,
        cancel_event: threading.Event,
    ) -> None:
        super().__init__(cancel_event)
        self.manager = manager
        self.config = config
        self.mode = mode
        self.command_state = command_state
        self.target_xy = target_xy

    def _enter_velocity_mode(self) -> None:
        vc = self.config.velocity
        with self.manager.access() as drive:
            self.manager.clear_errors_locked(drive)
            for i in (0, 1):
                axis = self.manager.axis(drive, i)
                axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
                axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                axis.controller.input_vel = 0.0
                axis.config.watchdog_timeout = vc.watchdog_s
                axis.config.enable_watchdog = True
                axis.watchdog_feed()
                axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with self.manager.access() as drive:
                states = []
                errors = []
                for i in (0, 1):
                    axis = self.manager.axis(drive, i)
                    axis.watchdog_feed()
                    states.append(int(axis.current_state))
                    errors.append(int(axis.error))
            if states == [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_CLOSED_LOOP_CONTROL] and errors == [0, 0]:
                return
            time.sleep(0.03)
        raise RuntimeError(f"Failed to enter closed loop: states={states}, errors={errors}")

    def _exit_velocity_mode(self) -> None:
        try:
            with self.manager.access() as drive:
                for i in (0, 1):
                    axis = self.manager.axis(drive, i)
                    axis.controller.input_vel = 0.0
                    try:
                        axis.watchdog_feed()
                    except Exception:
                        pass
                    current = float(axis.encoder.pos_estimate)
                    axis.config.enable_watchdog = False
                    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                    axis.controller.input_pos = current
        except Exception as exc:
            self.log.emit(f"Velocity-mode cleanup warning: {exc}")

    @Slot()
    def run(self) -> None:
        vc = self.config.velocity
        period = 1.0 / vc.loop_hz
        self.status.emit("Starting velocity control...", "info")
        try:
            self._enter_velocity_mode()
            self.log.emit(
                f"Velocity session entered ({self.mode}); watchdog={vc.watchdog_s:.3f} s, "
                f"loop={vc.loop_hz:.1f} Hz."
            )
            self.status.emit("Velocity control active", "ok")
            sampler = TelemetrySampler(self.manager, self.config, vc.loop_hz)
            previous_w = np.zeros(2)
            last_fault_check = 0.0
            completed_position_target = False

            while not self.cancelled():
                tick = time.monotonic()
                sample, raw = sampler.sample_with_raw()
                vx = vy = 0.0
                blocked = False
                block_reason = ""

                if sample.end_effector is None:
                    blocked = True
                    block_reason = "Forward kinematics is unreachable at the measured joint pose."
                elif self.mode == "jog":
                    vx, vy, last_update = self.command_state.snapshot()
                    if tick - last_update > vc.deadman_s:
                        vx = vy = 0.0
                elif self.mode == "position":
                    if self.target_xy is None:
                        raise ValueError("Cartesian position mode requires a target.")
                    ex = self.target_xy[0] - sample.end_effector[0]
                    ey = self.target_xy[1] - sample.end_effector[1]
                    error_mag = math.hypot(ex, ey)
                    if error_mag < vc.pos_tol_mm:
                        vx = vy = 0.0
                        completed_position_target = True
                    else:
                        speed = min(vc.max_cart_speed_mm_s, vc.pos_kp * error_mag)
                        vx = speed * ex / error_mag
                        vy = speed * ey / error_mag
                else:
                    raise ValueError(f"Unknown velocity mode: {self.mode}")

                if blocked:
                    requested_w = np.zeros(2)
                else:
                    try:
                        requested_w, derate, sigma_max = cartesian_to_joint_velocity(
                            vx,
                            vy,
                            sample.end_effector[0],
                            sample.end_effector[1],
                            self.config.geometry,
                            vc.manip_soft_deg_mm,
                            vc.manip_hard_deg_mm,
                            vc.joint_vel_cap_deg_s,
                        )
                        if derate <= 0:
                            blocked = True
                            block_reason = (
                                f"Motion blocked near singularity (σmax={sigma_max:.3f} deg/mm)."
                            )
                    except KinematicsError as exc:
                        requested_w = np.zeros(2)
                        blocked = True
                        block_reason = str(exc)

                w = slew_limit_vector(
                    previous_w,
                    requested_w,
                    vc.joint_accel_cap_deg_s2 * period,
                )
                previous_w = w
                with self.manager.access() as drive:
                    for i in (0, 1):
                        axis = self.manager.axis(drive, i)
                        axis.controller.input_vel = deg_per_s_to_turns_per_s(
                            float(w[i]), self.config.axes[i]
                        )
                        axis.watchdog_feed()

                sample.blocked = blocked
                sample.block_reason = block_reason
                self.telemetry.emit(sample)

                if tick - last_fault_check >= 0.5:
                    last_fault_check = tick
                    faults = []
                    for i in (0, 1):
                        if raw[i].axis_error or raw[i].current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                            faults.append(
                                f"axis{i}: state={raw[i].current_state}, error=0x{raw[i].axis_error:X}"
                            )
                    if faults:
                        raise RuntimeError("Velocity session aborted: " + "; ".join(faults))

                if blocked and block_reason:
                    self.status.emit(block_reason, "warn")
                elif self.mode == "position" and completed_position_target:
                    self.log.emit("Cartesian PC-side position target reached within tolerance.")
                    break

                delay = period - (time.monotonic() - tick)
                if delay > 0:
                    time.sleep(delay)

            self._exit_velocity_mode()
            if self.cancelled():
                self.log.emit("Velocity session stopped cleanly; current position is held.")
                self.status.emit("Velocity session stopped", "info")
                self.emit_finished(True, "Velocity session stopped")
            else:
                self.status.emit("Cartesian target reached", "ok")
                self.emit_finished(True, "Cartesian target reached")
        except Exception as exc:
            self._exit_velocity_mode()
            self.status.emit("Velocity control fault", "error")
            self.log.emit(f"Velocity control fault: {exc}")
            self.emit_finished(False, str(exc))


class AxisActionWorker(BaseWorker):
    def __init__(self, manager: ODriveManager, action: str) -> None:
        super().__init__()
        self.manager = manager
        self.action = action

    def _enable_closed_loop(self, clear_errors: bool) -> dict[str, Any]:
        with self.manager.access() as drive:
            if clear_errors:
                self.manager.clear_errors_locked(drive)
            for i in (0, 1):
                axis = self.manager.axis(drive, i)
                axis.config.enable_watchdog = False
                axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                axis.controller.input_pos = float(axis.encoder.pos_estimate)
                axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.15)
        with self.manager.access() as drive:
            result = {}
            for i in (0, 1):
                axis = self.manager.axis(drive, i)
                result[f"axis{i}"] = {
                    "state": int(axis.current_state),
                    "error": int(axis.error),
                }
            if any(
                item["state"] != AXIS_STATE_CLOSED_LOOP_CONTROL or item["error"] != 0
                for item in result.values()
            ):
                raise RuntimeError(f"Closed-loop verification failed: {result}")
            return result

    @Slot()
    def run(self) -> None:
        try:
            payload: object = None
            if self.action == "clear_errors":
                with self.manager.access() as drive:
                    self.manager.clear_errors_locked(drive)
                message = "ODrive errors cleared."
            elif self.action == "enable_closed_loop":
                payload = self._enable_closed_loop(clear_errors=False)
                message = "Both axes verified in CLOSED_LOOP_CONTROL."
            elif self.action == "resume":
                payload = self._enable_closed_loop(clear_errors=True)
                message = "Recovered from E-stop; both axes verified in closed loop."
            elif self.action == "idle":
                with self.manager.access() as drive:
                    for i in (0, 1):
                        axis = self.manager.axis(drive, i)
                        axis.config.enable_watchdog = False
                        axis.requested_state = AXIS_STATE_IDLE
                message = "Both axes requested IDLE."
            elif self.action == "show_errors":
                with self.manager.access() as drive:
                    payload = self.manager.read_error_report_locked(drive)
                message = "ODrive error report read."
            elif self.action == "read_flags":
                flags: dict[str, dict[str, Any]] = {}
                with self.manager.access() as drive:
                    for i in (0, 1):
                        axis = self.manager.axis(drive, i)
                        flags[f"axis{i}"] = {
                            "motor.pre_calibrated": bool(axis.motor.config.pre_calibrated),
                            "encoder.pre_calibrated": bool(axis.encoder.config.pre_calibrated),
                            "startup_closed_loop_control": bool(
                                axis.config.startup_closed_loop_control
                            ),
                            "startup_homing": bool(axis.config.startup_homing),
                            "startup_motor_calibration": bool(
                                axis.config.startup_motor_calibration
                            ),
                            "startup_encoder_offset_calibration": bool(
                                axis.config.startup_encoder_offset_calibration
                            ),
                        }
                payload = flags
                message = "Startup and calibration flags read."
            elif self.action == "read_hardware":
                def read_value(obj: Any, name: str, cast: Any, default: Any = None) -> Any:
                    try:
                        return cast(getattr(obj, name))
                    except Exception:
                        return default

                values: dict[str, Any] = {}
                with self.manager.access() as drive:
                    values["vbus_voltage"] = read_value(drive, "vbus_voltage", float)
                    for i in (0, 1):
                        axis = self.manager.axis(drive, i)
                        values[f"axis{i}"] = {
                            "runtime": {
                                "state": read_value(axis, "current_state", int),
                                "axis_error": read_value(axis, "error", int),
                                "motor_error": read_value(axis.motor, "error", int),
                                "encoder_error": read_value(axis.encoder, "error", int),
                                "pos_estimate_turns": read_value(axis.encoder, "pos_estimate", float),
                                "vel_estimate_turns_s": read_value(axis.encoder, "vel_estimate", float),
                            },
                            "encoder": {
                                "mode": read_value(axis.encoder.config, "mode", int),
                                "abs_spi_cs_gpio_pin": read_value(
                                    axis.encoder.config, "abs_spi_cs_gpio_pin", int
                                ),
                                "pre_calibrated": read_value(
                                    axis.encoder.config, "pre_calibrated", bool
                                ),
                            },
                            "motor": {
                                "current_lim": read_value(axis.motor.config, "current_lim", float),
                                "pre_calibrated": read_value(
                                    axis.motor.config, "pre_calibrated", bool
                                ),
                            },
                            "controller": {
                                "pos_gain": read_value(axis.controller.config, "pos_gain", float),
                                "vel_gain": read_value(axis.controller.config, "vel_gain", float),
                                "vel_integrator_gain": read_value(
                                    axis.controller.config, "vel_integrator_gain", float
                                ),
                                "vel_limit": read_value(axis.controller.config, "vel_limit", float),
                                "control_mode": read_value(
                                    axis.controller.config, "control_mode", int
                                ),
                                "input_mode": read_value(
                                    axis.controller.config, "input_mode", int
                                ),
                            },
                            "axis_config": {
                                "watchdog_timeout": read_value(
                                    axis.config, "watchdog_timeout", float
                                ),
                                "enable_watchdog": read_value(
                                    axis.config, "enable_watchdog", bool
                                ),
                                "startup_closed_loop_control": read_value(
                                    axis.config, "startup_closed_loop_control", bool
                                ),
                                "startup_homing": read_value(
                                    axis.config, "startup_homing", bool
                                ),
                                "startup_motor_calibration": read_value(
                                    axis.config, "startup_motor_calibration", bool
                                ),
                                "startup_encoder_offset_calibration": read_value(
                                    axis.config, "startup_encoder_offset_calibration", bool
                                ),
                            },
                        }
                payload = values
                message = "Connected ODrive hardware configuration read."
            elif self.action == "read_spi":
                values: dict[str, dict[str, int]] = {}
                with self.manager.access() as drive:
                    for i in (0, 1):
                        axis = self.manager.axis(drive, i)
                        values[f"axis{i}"] = {
                            "mode": int(axis.encoder.config.mode),
                            "cs_gpio": int(axis.encoder.config.abs_spi_cs_gpio_pin),
                        }
                payload = values
                message = "SPI encoder configuration read."
            else:
                raise ValueError(f"Unknown axis action: {self.action}")
            self.log.emit(message)
            self.emit_finished(True, message, payload)
        except Exception as exc:
            self.log.emit(f"Action '{self.action}' failed: {exc}")
            self.emit_finished(False, str(exc))


class CalibrationWorker(BaseWorker):
    def __init__(self, manager: ODriveManager, axes: tuple[int, ...]) -> None:
        super().__init__()
        self.manager = manager
        self.axes = axes

    def _run_state(self, axis_index: int, state: int, label: str) -> None:
        self.log.emit(f"axis{axis_index}: starting {label}...")
        with self.manager.access() as drive:
            axis = self.manager.axis(drive, axis_index)
            axis.requested_state = state
        deadline = time.monotonic() + 30.0
        time.sleep(0.05)
        while time.monotonic() < deadline:
            if self.cancelled():
                raise RuntimeError("Calibration cancelled.")
            with self.manager.access() as drive:
                axis = self.manager.axis(drive, axis_index)
                current_state = int(axis.current_state)
            if current_state == AXIS_STATE_IDLE:
                return
            time.sleep(0.1)
        raise TimeoutError(f"axis{axis_index} {label} exceeded 30 s timeout.")

    @Slot()
    def run(self) -> None:
        clean = True
        details: dict[str, Any] = {}
        try:
            for index in self.axes:
                self._run_state(index, AXIS_STATE_MOTOR_CALIBRATION, "motor calibration")
                self._run_state(
                    index, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, "encoder offset calibration"
                )
                with self.manager.access() as drive:
                    axis = self.manager.axis(drive, index)
                    errors = {
                        "axis": int(axis.error),
                        "motor": int(axis.motor.error),
                        "encoder": int(axis.encoder.error),
                    }
                passed = all(value == 0 for value in errors.values())
                clean = clean and passed
                details[f"axis{index}"] = {"passed": passed, "errors": errors}
                self.log.emit(
                    f"axis{index}: calibration {'PASSED' if passed else 'FAILED'}; errors={errors}"
                )
            if not clean:
                raise RuntimeError(f"Calibration completed with errors: {details}")
            self.status.emit("Calibration complete", "ok")
            self.emit_finished(True, "Calibration complete", details)
        except Exception as exc:
            self.status.emit("Calibration failed", "error")
            self.log.emit(f"Calibration failed: {exc}")
            self.emit_finished(False, str(exc), details)


class SyncZeroWorker(BaseWorker):
    def __init__(
        self,
        manager: ODriveManager,
        config: DashboardConfig,
        reference_angles_deg: dict[int, float],
    ) -> None:
        super().__init__()
        self.manager = manager
        self.config = config
        self.reference_angles_deg = {
            0: float(reference_angles_deg[0]),
            1: float(reference_angles_deg[1]),
        }

    @Slot()
    def run(self) -> None:
        """Synchronise dashboard offsets from a noise-robust position median.

        ``encoder.vel_estimate`` is recorded only for diagnostics.  It is intentionally not
        used to decide whether the robot is moving because SPI absolute encoders can report
        non-zero velocity noise while the axes are IDLE.
        """
        try:
            noise = self.config.encoder_noise
            sample_count = max(12, int(round(noise.sync_sample_duration_s * noise.sync_sample_hz)))
            period = 1.0 / max(1.0, noise.sync_sample_hz)
            times: list[float] = []
            positions: dict[int, list[float]] = {0: [], 1: []}
            raw_velocities: dict[int, list[float]] = {0: [], 1: []}
            states: dict[int, list[int]] = {0: [], 1: []}
            started = time.monotonic()
            self.status.emit("Sampling encoders for robust sync...", "info")
            self.log.emit(
                f"Robust sync sampling {sample_count} position samples over "
                f"~{noise.sync_sample_duration_s:.2f} s. Raw vel_estimate is diagnostic only."
            )

            for index in range(sample_count):
                if self.cancelled():
                    raise RuntimeError("Software-reference sync cancelled.")
                tick = time.monotonic()
                with self.manager.access() as drive:
                    for axis_index in (0, 1):
                        axis = self.manager.axis(drive, axis_index)
                        positions[axis_index].append(float(axis.encoder.pos_estimate))
                        raw_velocities[axis_index].append(float(axis.encoder.vel_estimate))
                        states[axis_index].append(int(axis.current_state))
                times.append(tick - started)
                target = started + (index + 1) * period
                delay = target - time.monotonic()
                if delay > 0:
                    self.cancel_event.wait(delay)

            estimates = {
                axis: robust_position_estimate(
                    times,
                    positions[axis],
                    max_drift_turns_s=noise.sync_max_drift_turns_s,
                    noise_warning_span_turns=noise.sync_noise_warning_span_turns,
                    hard_span_turns=noise.sync_hard_span_turns,
                )
                for axis in (0, 1)
            }
            moving = [axis for axis in (0, 1) if not estimates[axis].stationary]
            if moving:
                details = "; ".join(
                    f"axis{axis}: drift={estimates[axis].drift_turns_s:+.6f} turns/s, "
                    f"90% span={estimates[axis].robust_span_turns:.6f} turns"
                    for axis in moving
                )
                raise RuntimeError(
                    "Refusing sync because sustained position movement was detected. " + details
                )

            raw = tuple(estimates[axis].position_turns for axis in (0, 1))
            raw_velocity_median = tuple(
                float(np.median(raw_velocities[axis])) for axis in (0, 1)
            )
            offsets: dict[int, float] = {}
            verified: dict[int, float] = {}
            for axis in (0, 1):
                mapping = self.config.axes[axis]
                offsets[axis] = offset_for_reference_angle(
                    raw_turns=raw[axis],
                    reference_angle_deg=self.reference_angles_deg[axis],
                    home_angle_deg=self.config.home_angle_deg[axis],
                    cfg=mapping,
                )
                verified[axis] = (
                    (raw[axis] - offsets[axis]) / mapping.gear_ratio
                ) * 360.0 / mapping.direction + self.config.home_angle_deg[axis]

            diagnostics = {
                axis: {
                    "samples": estimates[axis].sample_count,
                    "robust_span_turns": estimates[axis].robust_span_turns,
                    "drift_turns_s": estimates[axis].drift_turns_s,
                    "net_change_turns": estimates[axis].net_change_turns,
                    "high_noise": estimates[axis].high_noise,
                    "raw_velocity_median_turns_s": raw_velocity_median[axis],
                    "states_observed": sorted(set(states[axis])),
                }
                for axis in (0, 1)
            }
            payload = {
                "raw_turns": raw,
                "velocity_turns_s": raw_velocity_median,
                "offset_turns": offsets,
                "reference_angle_deg": self.reference_angles_deg,
                "verified_angle_deg": verified,
                "diagnostics": diagnostics,
            }
            for axis in (0, 1):
                estimate = estimates[axis]
                noise_note = " HIGH-NOISE WARNING" if estimate.high_noise else ""
                self.log.emit(
                    f"axis{axis} robust sync: median={raw[axis]:+.7f} turns, "
                    f"drift={estimate.drift_turns_s:+.7f} turns/s, "
                    f"90% span={estimate.robust_span_turns:.7f} turns, "
                    f"raw vel median={raw_velocity_median[axis]:+.7f} turns/s.{noise_note}"
                )
            self.log.emit(
                "Software offsets synced from robust median positions. "
                "Non-zero raw encoder velocity noise was not treated as motor movement."
            )
            self.status.emit("Software offsets synced", "ok")
            self.emit_finished(True, "Software offsets synced", payload)
        except Exception as exc:
            self.status.emit("Software reference sync failed", "error")
            self.log.emit(f"Software reference sync failed: {exc}")
            self.emit_finished(False, str(exc))


class FlashConfigWorker(BaseWorker):
    def __init__(
        self,
        manager: ODriveManager,
        action: str,
        payload: dict[str, Any] | None = None,
    ) -> None:
        super().__init__()
        self.manager = manager
        self.action = action
        self.payload = payload or {}

    @Slot()
    def run(self) -> None:
        self.status.emit("Saving & rebooting...", "warn")
        expected_drop: Exception | None = None
        try:
            with self.manager.access() as drive:
                if self.action == "mark_calibrated":
                    for i in (0, 1):
                        axis = self.manager.axis(drive, i)
                        errors = (int(axis.error), int(axis.motor.error), int(axis.encoder.error))
                        if any(errors):
                            raise RuntimeError(
                                f"axis{i} has non-zero errors {errors}; refusing to mark calibrated."
                            )
                    auto_closed = bool(self.payload.get("auto_closed_loop", True))
                    for i in (0, 1):
                        axis = self.manager.axis(drive, i)
                        axis.motor.config.pre_calibrated = True
                        axis.encoder.config.pre_calibrated = True
                        axis.config.startup_motor_calibration = False
                        axis.config.startup_encoder_offset_calibration = False
                        axis.config.startup_closed_loop_control = auto_closed
                        axis.config.startup_homing = False
                elif self.action == "clear_precalibrated":
                    for i in (0, 1):
                        axis = self.manager.axis(drive, i)
                        axis.motor.config.pre_calibrated = False
                        axis.encoder.config.pre_calibrated = False
                        axis.config.startup_motor_calibration = False
                        axis.config.startup_encoder_offset_calibration = False
                        axis.config.startup_closed_loop_control = False
                        axis.config.startup_homing = False
                elif self.action == "apply_spi":
                    for i in (0, 1):
                        axis = self.manager.axis(drive, i)
                        settings = self.payload[f"axis{i}"]
                        axis.encoder.config.mode = int(settings["mode"])
                        axis.encoder.config.abs_spi_cs_gpio_pin = int(settings["cs_gpio"])
                elif self.action == "save_only":
                    pass
                else:
                    raise ValueError(f"Unknown flash action: {self.action}")
                try:
                    drive.save_configuration()
                except Exception as exc:
                    expected_drop = exc
            message = "Configuration saved; ODrive reboot initiated. Reconnect when it is back."
            if expected_drop:
                message += f" Connection drop was expected ({expected_drop})."
            self.log.emit(message)
            self.emit_finished(True, message)
        except Exception as exc:
            self.log.emit(f"Flash configuration action failed: {exc}")
            self.emit_finished(False, str(exc))
        finally:
            self.manager.disconnect()
            self.status.emit("Disconnected after reboot", "warn")


class PidWorker(BaseWorker):
    def __init__(
        self,
        manager: ODriveManager,
        axis_index: int,
        action: str,
        values: dict[str, float] | None = None,
    ) -> None:
        super().__init__()
        self.manager = manager
        self.axis_index = axis_index
        self.action = action
        self.values = values or {}

    @Slot()
    def run(self) -> None:
        try:
            with self.manager.access() as drive:
                axis = self.manager.axis(drive, self.axis_index)
                if self.action == "read":
                    payload = {
                        "pos_gain": float(axis.controller.config.pos_gain),
                        "vel_gain": float(axis.controller.config.vel_gain),
                        "vel_integrator_gain": float(
                            axis.controller.config.vel_integrator_gain
                        ),
                        "vel_limit": float(axis.controller.config.vel_limit),
                        "current_lim": float(axis.motor.config.current_lim),
                    }
                    message = f"axis{self.axis_index} PID/current settings read."
                elif self.action == "apply":
                    axis.controller.config.pos_gain = float(self.values["pos_gain"])
                    axis.controller.config.vel_gain = float(self.values["vel_gain"])
                    axis.controller.config.vel_integrator_gain = float(
                        self.values["vel_integrator_gain"]
                    )
                    axis.controller.config.vel_limit = float(self.values["vel_limit"])
                    axis.motor.config.current_lim = float(self.values["current_lim"])
                    payload = dict(self.values)
                    message = f"axis{self.axis_index} PID/current settings applied in RAM."
                else:
                    raise ValueError(f"Unknown PID action: {self.action}")
            self.log.emit(message)
            self.emit_finished(True, message, payload)
        except Exception as exc:
            self.log.emit(f"PID action failed: {exc}")
            self.emit_finished(False, str(exc))


class StepResponseWorker(BaseWorker):
    def __init__(
        self,
        manager: ODriveManager,
        config: DashboardConfig,
        axis_index: int,
        step_turns: float,
        duration_s: float,
        cancel_event: threading.Event,
    ) -> None:
        super().__init__(cancel_event)
        self.manager = manager
        self.config = config
        self.axis_index = axis_index
        self.step_turns = step_turns
        self.duration_s = duration_s

    @Slot()
    def run(self) -> None:
        try:
            with self.manager.access() as drive:
                self.manager.require_closed_loop(drive)
                axis = self.manager.axis(drive, self.axis_index)
                start_pos = float(axis.encoder.pos_estimate)
                target = start_pos + self.step_turns
                gains = {
                    "pos_gain": float(axis.controller.config.pos_gain),
                    "vel_gain": float(axis.controller.config.vel_gain),
                    "vel_integrator_gain": float(
                        axis.controller.config.vel_integrator_gain
                    ),
                }
                axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                axis.controller.input_pos = target

            self.log.emit(
                f"axis{self.axis_index} step response started: {self.step_turns:+.5f} turns, "
                f"duration={self.duration_s:.2f} s."
            )
            sampler = TelemetrySampler(self.manager, self.config, 50.0)
            samples: list[tuple[float, float]] = []
            velocity_samples: list[tuple[float, float, float]] = []
            start_time = time.monotonic()
            while time.monotonic() - start_time <= self.duration_s:
                if self.cancelled():
                    raise RuntimeError("Step-response test cancelled.")
                loop_start = time.monotonic()
                telemetry, raw = sampler.sample_with_raw()
                elapsed = time.monotonic() - start_time
                samples.append((elapsed, raw[self.axis_index].pos_turns))
                filtered_turn_s = deg_per_s_to_turns_per_s(
                    telemetry.vel_filtered_deg_s[self.axis_index],
                    self.config.axes[self.axis_index],
                )
                velocity_samples.append(
                    (elapsed, raw[self.axis_index].vel_turns_s, filtered_turn_s)
                )
                self.telemetry.emit(telemetry)
                delay = 0.02 - (time.monotonic() - loop_start)
                if delay > 0:
                    time.sleep(delay)

            delta = target - start_pos
            positions = [value for _, value in samples]
            if not positions or abs(delta) <= 1e-12:
                overshoot = 0.0
            elif delta > 0:
                overshoot = max(0.0, (max(positions) - target) / delta * 100.0)
            else:
                overshoot = max(0.0, (target - min(positions)) / abs(delta) * 100.0)
            tolerance = 0.02 * abs(delta)
            outside_times = [t for t, value in samples if abs(value - target) > tolerance]
            settling = max(outside_times) if outside_times else 0.0
            if samples and abs(samples[-1][1] - target) > tolerance:
                settling = None
            final_error = samples[-1][1] - target if samples else math.nan
            result = StepResponseResult(
                axis=self.axis_index,
                start_turns=start_pos,
                target_turns=target,
                samples=samples,
                velocity_samples=velocity_samples,
                overshoot_pct=overshoot,
                settling_time_s=settling,
                final_error_turns=final_error,
                gains=gains,
            )
            self.log.emit(
                f"Step response complete: overshoot={overshoot:.2f}%, "
                f"settling={'not settled' if settling is None else f'{settling:.3f} s'}, "
                f"final error={final_error:+.6f} turns."
            )
            self.emit_finished(True, "Step response complete", result)
        except Exception as exc:
            self.log.emit(f"Step-response test failed: {exc}")
            self.emit_finished(False, str(exc))
