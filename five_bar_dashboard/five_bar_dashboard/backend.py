"""Thread-safe ODrive access and a lightweight development simulator."""
from __future__ import annotations

import contextlib
import io
import math
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from types import SimpleNamespace
from typing import Any, Iterator

from .constants import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_IDLE,
    AXIS_STATE_MOTOR_CALIBRATION,
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    INPUT_MODE_TRAP_TRAJ,
    axis_state_name,
)


class ODriveError(RuntimeError):
    pass


class ODriveNotConnectedError(ODriveError):
    pass


class ClosedLoopRequiredError(ODriveError):
    pass


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _decode_error_bits(value: int, prefix: str) -> list[str]:
    """Decode an ODrive 0.5.x error bitmask without depending on one enum layout."""
    value = int(value)
    if value == 0:
        return ["NONE"]
    names: list[tuple[int, str]] = []
    try:
        from odrive import enums as odrive_enums  # type: ignore

        for name in dir(odrive_enums):
            if not name.startswith(prefix):
                continue
            try:
                bit = int(getattr(odrive_enums, name))
            except Exception:
                continue
            if bit and bit & (bit - 1) == 0 and value & bit:
                names.append((bit, name.removeprefix(prefix)))
    except Exception:
        pass
    names.sort(key=lambda item: item[0])
    decoded = [name for _, name in names]
    covered = 0
    for bit, _ in names:
        covered |= bit
    unknown = value & ~covered
    if unknown:
        decoded.append(f"UNKNOWN_BITS_0x{unknown:X}")
    return decoded or [f"UNKNOWN_0x{value:X}"]


def format_error_report(report: dict[str, Any]) -> str:
    lines = [
        "ODrive Error Report",
        f"Captured: {report.get('captured_at', 'unknown')}",
        "",
    ]
    has_errors = bool(report.get("has_errors", False))
    lines.append("Summary: ACTIVE ERROR(S) PRESENT" if has_errors else "Summary: no active errors")
    lines.append("")
    axes = report.get("axes", {})
    for axis_name in ("axis0", "axis1"):
        data = axes.get(axis_name, {})
        state = data.get("state")
        state_text = data.get("state_name", "UNKNOWN")
        lines.append(f"{axis_name}: state={state_text} ({state})")
        for label, key in (
            ("axis", "axis_error"),
            ("motor", "motor_error"),
            ("encoder", "encoder_error"),
            ("controller", "controller_error"),
            ("sensorless", "sensorless_error"),
        ):
            value = data.get(key)
            if value is None:
                lines.append(f"  {label:10s}: unavailable")
                continue
            names = data.get(f"{key}_names", ["NONE"] if int(value) == 0 else [])
            lines.append(f"  {label:10s}: 0x{int(value):08X}  {', '.join(names)}")
        read_failures = data.get("read_failures", [])
        for failure in read_failures:
            lines.append(f"  read warning: {failure}")
        lines.append("")
    dump_text = str(report.get("dump_errors_output", "")).strip()
    if dump_text:
        lines.extend(["odrive.utils.dump_errors output:", dump_text, ""])
    dump_failure = str(report.get("dump_errors_failure", "")).strip()
    if dump_failure:
        lines.extend(
            [
                "Note: odrive.utils.dump_errors was unavailable or failed; ",
                f"the structured register read above was used instead. ({dump_failure})",
            ]
        )
    return "\n".join(lines).rstrip()


@dataclass(slots=True)
class AxisRawSnapshot:
    pos_turns: float
    vel_turns_s: float
    current_a: float
    axis_error: int
    motor_error: int
    encoder_error: int
    current_state: int


class _SimController:
    def __init__(self, axis: "_SimAxis") -> None:
        self.axis = axis
        self.config = SimpleNamespace(
            pos_gain=20.0,
            vel_gain=0.16,
            vel_integrator_gain=0.32,
            vel_limit=10.0,
            control_mode=CONTROL_MODE_POSITION_CONTROL,
            input_mode=INPUT_MODE_PASSTHROUGH,
        )
        self._input_pos = 0.0
        self._input_vel = 0.0
        self._input_torque = 0.0

    @property
    def input_pos(self) -> float:
        return self._input_pos

    @input_pos.setter
    def input_pos(self, value: float) -> None:
        self.axis._update()
        self._input_pos = float(value)

    @property
    def input_vel(self) -> float:
        return self._input_vel

    @input_vel.setter
    def input_vel(self, value: float) -> None:
        self.axis._update()
        self._input_vel = float(value)

    @property
    def input_torque(self) -> float:
        return self._input_torque

    @input_torque.setter
    def input_torque(self, value: float) -> None:
        self.axis._update()
        self._input_torque = float(value)


class _SimAxis:
    def __init__(self, index: int) -> None:
        self.index = index
        self.error = 0
        self._state = AXIS_STATE_IDLE
        self._calibration_end = 0.0
        self._last_update = time.monotonic()
        self._pos = 0.0
        self._vel = 0.0
        self.encoder = SimpleNamespace(
            pos_estimate=0.0,
            vel_estimate=0.0,
            error=0,
            config=SimpleNamespace(
                mode=0x101,
                abs_spi_cs_gpio_pin=4 if index == 0 else 3,
                pre_calibrated=False,
            ),
        )
        self.motor = SimpleNamespace(
            error=0,
            config=SimpleNamespace(current_lim=20.0, torque_constant=0.05, pre_calibrated=False),
            current_control=SimpleNamespace(Iq_measured=0.0),
        )
        self.controller = _SimController(self)
        self.trap_traj = SimpleNamespace(
            config=SimpleNamespace(vel_limit=2.0, accel_limit=4.0, decel_limit=4.0)
        )
        self.config = SimpleNamespace(
            watchdog_timeout=0.15,
            enable_watchdog=False,
            startup_closed_loop_control=False,
            startup_homing=False,
            startup_motor_calibration=False,
            startup_encoder_offset_calibration=False,
        )
        self._last_watchdog_feed = time.monotonic()

    @property
    def current_state(self) -> int:
        self._update()
        return self._state

    @property
    def requested_state(self) -> int:
        return self._state

    @requested_state.setter
    def requested_state(self, value: int) -> None:
        self._update()
        value = int(value)
        if value in {AXIS_STATE_MOTOR_CALIBRATION, AXIS_STATE_ENCODER_OFFSET_CALIBRATION}:
            self._state = value
            self._calibration_end = time.monotonic() + 0.35
        else:
            self._state = value
            if value == AXIS_STATE_IDLE:
                self._vel = 0.0

    def watchdog_feed(self) -> None:
        self._last_watchdog_feed = time.monotonic()

    def _update(self) -> None:
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self._last_update))
        self._last_update = now
        if self._calibration_end and now >= self._calibration_end:
            self._state = AXIS_STATE_IDLE
            self._calibration_end = 0.0
        if self.config.enable_watchdog and now - self._last_watchdog_feed > self.config.watchdog_timeout:
            self.error = 0x800
            self._state = AXIS_STATE_IDLE
            self._vel = 0.0
        if self._state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            mode = self.controller.config.control_mode
            if mode == CONTROL_MODE_VELOCITY_CONTROL:
                desired = max(
                    -self.controller.config.vel_limit,
                    min(self.controller.config.vel_limit, self.controller.input_vel),
                )
                accel = 15.0
                dv = max(-accel * dt, min(accel * dt, desired - self._vel))
                self._vel += dv
                self._pos += self._vel * dt
            else:
                error = self.controller.input_pos - self._pos
                if self.controller.config.input_mode == INPUT_MODE_TRAP_TRAJ:
                    vmax = max(0.01, float(self.trap_traj.config.vel_limit))
                    accel_limit = max(0.01, float(self.trap_traj.config.accel_limit))
                    decel_limit = max(0.01, float(self.trap_traj.config.decel_limit))
                    # Approximate the firmware planner's braking envelope so the simulator
                    # responds to independently edited acceleration/deceleration values.
                    brake_speed = (2.0 * decel_limit * abs(error)) ** 0.5
                    desired_mag = min(vmax, brake_speed)
                    desired = (desired_mag if error >= 0.0 else -desired_mag) + self.controller.input_vel
                    desired = max(-vmax, min(vmax, desired))
                    speeding_up = abs(desired) > abs(self._vel) and desired * self._vel >= 0.0
                    rate_limit = accel_limit if speeding_up else decel_limit
                else:
                    vmax = max(0.01, float(self.controller.config.vel_limit))
                    desired = max(-vmax, min(vmax, error * 6.0 + self.controller.input_vel))
                    rate_limit = 25.0
                dv = max(-rate_limit * dt, min(rate_limit * dt, desired - self._vel))
                self._vel += dv
                if abs(error) < 1e-4 and abs(self._vel) < 1e-3:
                    self._pos = self.controller.input_pos
                    self._vel = 0.0
                else:
                    self._pos += self._vel * dt
        else:
            self._vel = 0.0
        self.encoder.pos_estimate = self._pos
        self.encoder.vel_estimate = self._vel
        self.motor.current_control.Iq_measured = (
            abs(self._vel) * 0.8
            + abs(self.controller.input_pos - self._pos) * 0.2
            + abs(self.controller.input_torque) / max(1e-6, self.motor.config.torque_constant)
        )


class SimulatedODrive:
    def __init__(self) -> None:
        self.axis0 = _SimAxis(0)
        self.axis1 = _SimAxis(1)
        self.vbus_voltage = 24.0

    def clear_errors(self) -> None:
        for axis in (self.axis0, self.axis1):
            axis.error = 0
            axis.motor.error = 0
            axis.encoder.error = 0

    def save_configuration(self) -> None:
        time.sleep(0.15)


class ODriveManager:
    """Owns the ODrive object and serialises every libusb transaction."""

    def __init__(self, simulate: bool = False) -> None:
        self.lock = threading.RLock()
        self._drive: Any | None = None
        self.simulate = simulate

    @property
    def connected(self) -> bool:
        return self._drive is not None

    def set_drive(self, drive: Any) -> None:
        with self.lock:
            self._drive = drive

    def disconnect(self) -> None:
        with self.lock:
            self._drive = None

    def get_drive_unlocked(self) -> Any:
        if self._drive is None:
            raise ODriveNotConnectedError("No ODrive is connected.")
        return self._drive

    @contextlib.contextmanager
    def access(self, timeout: float | None = None) -> Iterator[Any]:
        if timeout is None:
            acquired = self.lock.acquire()
        else:
            acquired = self.lock.acquire(timeout=timeout)
        if not acquired:
            raise TimeoutError("Timed out waiting for ODrive access lock.")
        try:
            yield self.get_drive_unlocked()
        finally:
            self.lock.release()

    def connect(self, attempts: int = 6, hard_timeout_s: float = 5.0) -> Any:
        if self.simulate:
            drive = SimulatedODrive()
            self.set_drive(drive)
            return drive

        last_error: Exception | None = None
        for _ in range(attempts):
            result: list[Any] = []
            error: list[BaseException] = []
            finished = threading.Event()

            def finder() -> None:
                try:
                    import odrive  # type: ignore

                    result.append(odrive.find_any(timeout=hard_timeout_s))
                except BaseException as exc:  # hardware library may raise non-standard errors
                    error.append(exc)
                finally:
                    finished.set()

            thread = threading.Thread(target=finder, daemon=True)
            thread.start()
            if not finished.wait(hard_timeout_s + 0.25):
                last_error = TimeoutError(
                    f"odrive.find_any exceeded the {hard_timeout_s:.1f}s hard timeout"
                )
                continue
            if result and result[0] is not None:
                self.set_drive(result[0])
                return result[0]
            if error:
                last_error = Exception(str(error[0]))
            else:
                last_error = ODriveError("No ODrive found.")
        raise ODriveError(f"Connection failed after {attempts} attempts: {last_error}")

    def axis(self, drive: Any, index: int) -> Any:
        return drive.axis0 if index == 0 else drive.axis1

    def require_closed_loop(self, drive: Any) -> None:
        failures: list[str] = []
        for index in (0, 1):
            axis = self.axis(drive, index)
            if int(axis.current_state) != AXIS_STATE_CLOSED_LOOP_CONTROL:
                failures.append(
                    f"axis{index}: state={int(axis.current_state)} error=0x{int(axis.error):X}"
                )
        if failures:
            raise ClosedLoopRequiredError(
                "Both axes must be in CLOSED_LOOP_CONTROL. Press Resume After E-Stop or "
                "Enable Closed Loop first. " + "; ".join(failures)
            )

    def read_axis_snapshot_locked(self, drive: Any, index: int) -> AxisRawSnapshot:
        axis = self.axis(drive, index)
        if hasattr(axis, "_update"):
            axis._update()
        try:
            current = float(axis.motor.current_control.Iq_measured)
        except Exception:
            current = 0.0
        return AxisRawSnapshot(
            pos_turns=float(axis.encoder.pos_estimate),
            vel_turns_s=float(axis.encoder.vel_estimate),
            current_a=current,
            axis_error=int(axis.error),
            motor_error=int(axis.motor.error),
            encoder_error=int(axis.encoder.error),
            current_state=int(axis.current_state),
        )

    def clear_errors_locked(self, drive: Any) -> None:
        if hasattr(drive, "clear_errors"):
            drive.clear_errors()
            return
        for index in (0, 1):
            axis = self.axis(drive, index)
            try:
                axis.error = 0
                axis.motor.error = 0
                axis.encoder.error = 0
            except Exception:
                pass

    def read_error_report_locked(self, drive: Any) -> dict[str, Any]:
        """Read a structured error snapshot suitable for a reliable GUI error window."""
        dump_text = ""
        dump_failure = ""
        try:
            from odrive.utils import dump_errors  # type: ignore

            output = io.StringIO()
            with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                dump_errors(drive)
            dump_text = output.getvalue().strip()
        except Exception as exc:
            dump_failure = str(exc)

        axes: dict[str, dict[str, Any]] = {}
        has_errors = False
        for index in (0, 1):
            axis = self.axis(drive, index)
            data: dict[str, Any] = {"read_failures": []}

            def read_error(path: str, obj: Any, attr: str, prefix: str) -> None:
                nonlocal has_errors
                key = f"{path}_error"
                try:
                    value = int(getattr(obj, attr))
                    data[key] = value
                    data[f"{key}_names"] = _decode_error_bits(value, prefix)
                    has_errors = has_errors or value != 0
                except Exception as exc:
                    data[key] = None
                    data["read_failures"].append(f"{path}.{attr}: {exc}")

            try:
                state = int(axis.current_state)
                data["state"] = state
                data["state_name"] = axis_state_name(state)
            except Exception as exc:
                data["state"] = None
                data["state_name"] = "UNAVAILABLE"
                data["read_failures"].append(f"current_state: {exc}")

            read_error("axis", axis, "error", "AXIS_ERROR_")
            read_error("motor", axis.motor, "error", "MOTOR_ERROR_")
            read_error("encoder", axis.encoder, "error", "ENCODER_ERROR_")
            if hasattr(axis, "controller") and hasattr(axis.controller, "error"):
                read_error("controller", axis.controller, "error", "CONTROLLER_ERROR_")
            else:
                data["controller_error"] = None
            if hasattr(axis, "sensorless_estimator") and hasattr(
                axis.sensorless_estimator, "error"
            ):
                read_error(
                    "sensorless", axis.sensorless_estimator, "error", "SENSORLESS_ESTIMATOR_ERROR_"
                )
            else:
                data["sensorless_error"] = None
            axes[f"axis{index}"] = data

        report: dict[str, Any] = {
            "captured_at": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
            "has_errors": has_errors,
            "axes": axes,
            "dump_errors_output": dump_text,
            "dump_errors_failure": dump_failure,
        }
        report["formatted_text"] = format_error_report(report)
        return report

    def dump_errors_locked(self, drive: Any) -> str:
        return str(self.read_error_report_locked(drive)["formatted_text"])

    def emergency_stop(self) -> tuple[bool, str]:
        """Immediate safety path. This is the deliberate GUI-thread hardware exception.

        It attempts the global lock briefly, then writes IDLE even without the lock if necessary,
        as required by the safety specification.
        """
        drive = self._drive
        if drive is None:
            return False, "No ODrive connected; local workers were still cancelled."
        acquired = self.lock.acquire(timeout=0.2)
        try:
            failures: list[str] = []
            for index in (0, 1):
                try:
                    axis = self.axis(drive, index)
                    try:
                        axis.config.enable_watchdog = False
                    except Exception as exc:
                        failures.append(f"axis{index} watchdog disable: {exc}")
                    try:
                        axis.requested_state = AXIS_STATE_IDLE
                    except Exception as exc:
                        failures.append(f"axis{index} IDLE request: {exc}")
                except Exception as exc:
                    failures.append(f"axis{index} access: {exc}")
            if failures:
                return False, "E-stop attempted both axes; " + "; ".join(failures)
            return True, "Both axes requested IDLE and watchdogs disabled."
        finally:
            if acquired:
                self.lock.release()
