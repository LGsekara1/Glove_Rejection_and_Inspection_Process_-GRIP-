"""
state.py
========
Single shared application state object. This dashboard is intended to run
as a local, single-operator tool talking to one physical ODrive, so state is
a module-level singleton rather than per-browser-tab session state -- if you
open the dashboard from two tabs they are looking at the same robot and the
same connection, which is the correct behaviour for this use case.
"""
from __future__ import annotations

import asyncio
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

import odrive_interface as odi
import kinematics as kin
import trajectory as traj


BUFFER_LEN = 1500  # ~ enough for a rolling window at typical poll rates


@dataclass
class AxisConfigState:
    motor: odi.MotorParams = field(default_factory=odi.MotorParams)
    encoder: odi.EncoderParams = field(default_factory=odi.EncoderParams)
    pos_gain: float = 20.0
    vel_gain: float = 0.16
    vel_integrator_gain: float = 0.32
    vel_limit: float = 10.0
    input_filter_bandwidth: float = 2.0
    control_mode_label: str = "Position"
    input_mode_label: str = "Pos Filter (recommended for streamed trajectories)"


@dataclass
class TelemetryBuffer:
    t: deque = field(default_factory=lambda: deque(maxlen=BUFFER_LEN))
    pos: deque = field(default_factory=lambda: deque(maxlen=BUFFER_LEN))
    vel: deque = field(default_factory=lambda: deque(maxlen=BUFFER_LEN))
    iq: deque = field(default_factory=lambda: deque(maxlen=BUFFER_LEN))
    iq_set: deque = field(default_factory=lambda: deque(maxlen=BUFFER_LEN))

    def append(self, t, telem: odi.AxisTelemetry):
        self.t.append(t)
        self.pos.append(telem.pos)
        self.vel.append(telem.vel)
        self.iq.append(telem.iq_measured)
        self.iq_set.append(telem.iq_setpoint)

    def clear(self):
        for d in (self.t, self.pos, self.vel, self.iq, self.iq_set):
            d.clear()


class AppState:
    def __init__(self):
        self.manager = odi.ODriveManager()
        self.t0 = time.time()

        self.axis_cfg = {0: AxisConfigState(), 1: AxisConfigState()}
        self.telemetry = {0: TelemetryBuffer(), 1: TelemetryBuffer()}

        # board level config
        self.brake_resistance = 2.0
        self.enable_brake_resistor = True
        self.dc_bus_overvoltage_trip_level = 59.0
        self.dc_max_negative_current = -1.0

        # graph visibility toggles
        self.show_position = True
        self.show_velocity = True
        self.show_current = True
        self.show_axis0 = True
        self.show_axis1 = True
        self.time_window_s = 20.0

        # kinematics
        self.geo = kin.FiveBarGeometry()
        self._workspace_cache = None
        self._workspace_cache_key = None

        # trajectory
        self.trajectory_mode = "Joint space"          # or "Cartesian (SCARA)"
        self.profile_type = "Trapezoidal (stop-to-stop)"  # or "Cubic spline (smooth)"
        self.waypoints: list[dict] = [
            {"t": 0.0, "a": 0.0, "b": 0.0},
            {"t": 1.0, "a": 1.0, "b": -1.0},
        ]
        self.traj_vel_limit0 = 5.0
        self.traj_accel_limit0 = 10.0
        self.traj_vel_limit1 = 5.0
        self.traj_accel_limit1 = 10.0
        self.traj_dt = 0.01
        self.traj_dwell = 0.2
        self.last_samples: Optional[traj.TrajectorySamples] = None
        self.stream_task: Optional[asyncio.Task] = None
        self.stream_progress = 0.0
        self.stream_running = False
        self.stream_loop = False

        # status/log messages shown in the calibration panel
        self.log_lines: deque = deque(maxlen=200)

    def log(self, msg: str):
        self.log_lines.append(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def elapsed(self) -> float:
        return time.time() - self.t0

    def workspace(self, resolution: int = 220):
        key = (self.geo.d, self.geo.l1a, self.geo.l2a, self.geo.l1b, self.geo.l2b,
               self.geo.elbow_sign_a, self.geo.elbow_sign_b,
               self.geo.theta1_min, self.geo.theta1_max, self.geo.theta2_min, self.geo.theta2_max,
               resolution)
        if self._workspace_cache_key != key:
            self._workspace_cache = kin.workspace_grid(self.geo, resolution=resolution)
            self._workspace_cache_key = key
        return self._workspace_cache


# Single shared instance imported by every UI panel module.
state = AppState()
