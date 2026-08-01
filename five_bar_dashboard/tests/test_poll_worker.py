from __future__ import annotations

import sys
import types

# The CI/container used for source validation does not install the GUI runtime.  Stub only
# the tiny QtCore surface needed to import the worker module; hardware/UI deployment still
# uses the real PySide6 package from requirements.txt.
if "PySide6.QtCore" not in sys.modules:
    qtcore = types.ModuleType("PySide6.QtCore")

    class QObject:
        pass

    class Signal:
        def __init__(self, *args, **kwargs) -> None:
            self._callbacks = []

        def connect(self, callback) -> None:
            self._callbacks.append(callback)

        def emit(self, *args) -> None:
            for callback in list(self._callbacks):
                callback(*args)

    def Slot(*args, **kwargs):
        def decorator(function):
            return function
        return decorator

    qtcore.QObject = QObject
    qtcore.Signal = Signal
    qtcore.Slot = Slot
    pyside = types.ModuleType("PySide6")
    pyside.QtCore = qtcore
    sys.modules["PySide6"] = pyside
    sys.modules["PySide6.QtCore"] = qtcore

from five_bar_dashboard.backend import ODriveManager
from five_bar_dashboard.models import DashboardConfig
from five_bar_dashboard.workers import PollWorker
import five_bar_dashboard.workers as workers_module


def test_poll_reset_request_does_not_cancel_worker() -> None:
    manager = ODriveManager(simulate=True)
    manager.connect()
    worker = PollWorker(manager, DashboardConfig(), hz=100.0)

    worker.request_reset()

    assert worker.reset_event.is_set()
    assert not worker.cancel_event.is_set()


def test_poll_worker_survives_more_than_six_transient_failures(monkeypatch) -> None:
    manager = ODriveManager(simulate=True)
    manager.connect()
    worker = PollWorker(manager, DashboardConfig(), hz=1000.0)
    state = {"calls": 0, "resets": 0}

    class FakeSampler:
        def __init__(self, *args, **kwargs) -> None:
            pass

        def reset(self) -> None:
            state["resets"] += 1

        def sample(self) -> object:
            state["calls"] += 1
            if state["calls"] <= 8:
                raise RuntimeError("simulated transient Fibre timeout")
            worker.cancel_event.set()
            return object()

    monkeypatch.setattr(workers_module, "TelemetrySampler", FakeSampler)
    worker.request_reset()
    results: list[tuple[bool, str, object]] = []
    worker.finished.connect(lambda ok, msg, payload: results.append((ok, msg, payload)))

    worker.run()

    assert state["resets"] == 1
    assert state["calls"] == 9
    assert results
    assert results[-1][0] is True
    assert results[-1][1] == "Polling stopped"


def test_coordinate_sequence_command_writes_independent_deceleration() -> None:
    import threading

    from five_bar_dashboard.constants import AXIS_STATE_CLOSED_LOOP_CONTROL
    from five_bar_dashboard.sequence import compile_cartesian_sequence
    from five_bar_dashboard.workers import CoordinateSequenceWorker

    manager = ODriveManager(simulate=True)
    drive = manager.connect()
    drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    cfg = DashboardConfig()
    point = compile_cartesian_sequence(
        [
            {
                "x_mm": 0.0,
                "y_mm": 400.0,
                "max_vel_deg_s": 30.0,
                "max_accel_deg_s2": 80.0,
                "max_decel_deg_s2": 25.0,
            }
        ],
        cfg.geometry,
        cfg.trajectory,
    )[0]
    worker = CoordinateSequenceWorker(manager, cfg, [], 1, threading.Event())

    _, _, applied = worker._command_waypoint(point)

    assert drive.axis0.trap_traj.config.decel_limit == applied["decel_turns_s2"][0]
    assert drive.axis1.trap_traj.config.decel_limit == applied["decel_turns_s2"][1]
    assert drive.axis0.trap_traj.config.accel_limit != drive.axis0.trap_traj.config.decel_limit


def test_coordinate_sequence_feedforward_is_bounded_and_written() -> None:
    import threading

    from five_bar_dashboard.constants import AXIS_STATE_CLOSED_LOOP_CONTROL
    from five_bar_dashboard.sequence import compile_cartesian_sequence
    from five_bar_dashboard.workers import CoordinateSequenceWorker

    manager = ODriveManager(simulate=True)
    drive = manager.connect()
    drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    cfg = DashboardConfig()
    point = compile_cartesian_sequence(
        [
            {
                "x_mm": 0.0,
                "y_mm": 400.0,
                "velocity_ff0_turns_s": 100.0,
                "velocity_ff1_turns_s": -100.0,
                "torque_ff0_nm": 5.0,
                "torque_ff1_nm": -5.0,
            }
        ],
        cfg.geometry,
        cfg.trajectory,
    )[0]
    worker = CoordinateSequenceWorker(
        manager, cfg, [], 1, threading.Event(), feedforward_enabled=True
    )

    _, _, applied = worker._command_waypoint(point)

    assert applied["velocity_ff_turns_s"] == (9.5, -9.5)
    # Simulator torque limit: 80% * 20 A * 0.05 Nm/A = 0.8 Nm.
    assert applied["torque_ff_nm"] == (0.8, -0.8)
    assert drive.axis0.controller.input_vel == 9.5
    assert drive.axis1.controller.input_torque == -0.8


def test_coordinate_sequence_feedforward_disabled_forces_zero() -> None:
    import threading

    from five_bar_dashboard.constants import AXIS_STATE_CLOSED_LOOP_CONTROL
    from five_bar_dashboard.sequence import compile_cartesian_sequence
    from five_bar_dashboard.workers import CoordinateSequenceWorker

    manager = ODriveManager(simulate=True)
    drive = manager.connect()
    drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    cfg = DashboardConfig()
    point = compile_cartesian_sequence(
        [{"x_mm": 0.0, "y_mm": 400.0, "torque_ff0_nm": 0.4}],
        cfg.geometry,
        cfg.trajectory,
    )[0]
    worker = CoordinateSequenceWorker(manager, cfg, [], 1, threading.Event())

    _, _, applied = worker._command_waypoint(point)

    assert applied["velocity_ff_turns_s"] == (0.0, 0.0)
    assert applied["torque_ff_nm"] == (0.0, 0.0)
