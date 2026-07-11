"""
trajectory.py
=============
Trajectory generation for streaming custom motion profiles to the ODrive.

Two profile families:

  * Trapezoidal (stop-to-stop) -- classic trapezoidal velocity profile per
    segment, with the two axes *time-synchronized* so they start and stop
    each segment together (the axis that would finish first has its peak
    velocity reduced to match the slower axis's segment time). Simple,
    predictable, always leaves the tool at rest at every waypoint.

  * Cubic spline -- a natural cubic spline through all waypoints against
    user-supplied waypoint times. Velocity-continuous (no stopping at
    intermediate waypoints), good for smooth continuous paths.

Both families can operate in:

  * Joint space  -- waypoints are (axis0, axis1) targets directly (turns).
  * Cartesian space (SCARA) -- waypoints are (x, y) targets; the sampled
    Cartesian path is converted to joint angles sample-by-sample via inverse
    kinematics, with joint velocities obtained by central-difference of the
    resulting joint-angle samples (accurate at the sample rates used here,
    and avoids needing an explicit Jacobian).

All generators return a `TrajectorySamples` object: parallel numpy arrays of
time, axis0 position/velocity, axis1 position/velocity, ready to be iterated
by the streaming loop in odrive_interface / the trajectory UI panel.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.interpolate import CubicSpline

import kinematics as kin


@dataclass
class TrajectorySamples:
    t: np.ndarray
    pos0: np.ndarray
    vel0: np.ndarray
    pos1: np.ndarray
    vel1: np.ndarray
    # Only populated for Cartesian-mode trajectories, useful for plotting the
    # path over the workspace diagram.
    x: Optional[np.ndarray] = None
    y: Optional[np.ndarray] = None

    @property
    def duration(self) -> float:
        return float(self.t[-1]) if len(self.t) else 0.0


class TrajectoryError(ValueError):
    pass


# ---------------------------------------------------------------------
# Trapezoidal profile primitives
# ---------------------------------------------------------------------
def _time_optimal_duration(distance: float, vmax: float, amax: float) -> float:
    d = abs(distance)
    if d < 1e-12:
        return 0.0
    vmax = max(vmax, 1e-9)
    amax = max(amax, 1e-9)
    d_to_cruise = vmax ** 2 / amax
    if d >= d_to_cruise:
        t_acc = vmax / amax
        t_cruise = (d - d_to_cruise) / vmax
        return 2 * t_acc + t_cruise
    else:
        return 2 * math.sqrt(d / amax)


def _trapezoid_sample(t: np.ndarray, distance: float, T: float, amax: float) -> Tuple[np.ndarray, np.ndarray]:
    """Sample a symmetric-accel/decel trapezoid of fixed total duration T that
    covers `distance` using acceleration magnitude amax (peak velocity is
    derived, and may be below the nominal vmax if T is longer than the
    time-optimal duration -- this is exactly what we want for synchronizing
    axes with a shared segment time)."""
    d = distance
    ad = abs(d)
    sign = 1.0 if d >= 0 else -1.0

    if ad < 1e-12 or T < 1e-9:
        return np.zeros_like(t), np.zeros_like(t)

    amax = max(amax, 1e-9)
    # Solve a*ta^2 - a*T*ta + D = 0 for the acceleration-phase duration ta.
    disc = T ** 2 - 4 * ad / amax
    if disc < 0:
        # Requested T is shorter than physically possible at this amax --
        # clamp to the time-optimal (triangle) case instead of raising, so
        # small floating point shortfalls never crash a run.
        disc = 0.0
    ta = (T - math.sqrt(disc)) / 2.0
    ta = max(0.0, min(ta, T / 2.0))
    vpeak = amax * ta if ta > 0 else ad / T  # pure constant-velocity fallback

    pos = np.zeros_like(t)
    vel = np.zeros_like(t)
    for i, ti in enumerate(t):
        if ti <= ta:
            p = 0.5 * amax * ti ** 2
            v = amax * ti
        elif ti <= T - ta:
            p = 0.5 * amax * ta ** 2 + vpeak * (ti - ta)
            v = vpeak
        else:
            tr = T - ti
            tr = max(0.0, tr)
            p = ad - 0.5 * amax * tr ** 2
            v = amax * tr
        pos[i] = sign * p
        vel[i] = sign * v
    return pos, vel


@dataclass
class AxisLimits:
    vel_limit: float
    accel_limit: float
    decel_limit: float = None  # unused separately here; symmetric accel assumed

    def __post_init__(self):
        if self.decel_limit is None:
            self.decel_limit = self.accel_limit


def trapezoidal_multiaxis(
    waypoints: Sequence[Tuple[float, float]],
    limits0: AxisLimits,
    limits1: AxisLimits,
    dt: float,
    dwell: float = 0.0,
) -> TrajectorySamples:
    """Chain of synchronized 2-axis trapezoidal segments through `waypoints`
    (waypoints[0] is the starting point). Axes start/stop together each
    segment; whichever axis is faster gets its peak velocity reduced to
    match the slower axis for that segment."""
    if len(waypoints) < 2:
        raise TrajectoryError("need at least a start point and one target waypoint")

    all_t, all_p0, all_v0, all_p1, all_v1 = [], [], [], [], []
    t_offset = 0.0

    for seg in range(len(waypoints) - 1):
        (a0_start, a1_start) = waypoints[seg]
        (a0_end, a1_end) = waypoints[seg + 1]
        d0 = a0_end - a0_start
        d1 = a1_end - a1_start

        T0 = _time_optimal_duration(d0, limits0.vel_limit, limits0.accel_limit)
        T1 = _time_optimal_duration(d1, limits1.vel_limit, limits1.accel_limit)
        T = max(T0, T1, 1e-6)

        n = max(2, int(round(T / dt)) + 1)
        t_local = np.linspace(0, T, n)

        p0, v0 = _trapezoid_sample(t_local, d0, T, limits0.accel_limit)
        p1, v1 = _trapezoid_sample(t_local, d1, T, limits1.accel_limit)

        all_t.append(t_local + t_offset)
        all_p0.append(p0 + a0_start)
        all_v0.append(v0)
        all_p1.append(p1 + a1_start)
        all_v1.append(v1)

        t_offset += T
        if dwell > 0 and seg < len(waypoints) - 2:
            all_t.append(np.array([t_offset, t_offset + dwell]))
            all_p0.append(np.array([a0_end, a0_end]))
            all_v0.append(np.array([0.0, 0.0]))
            all_p1.append(np.array([a1_end, a1_end]))
            all_v1.append(np.array([0.0, 0.0]))
            t_offset += dwell

    return TrajectorySamples(
        t=np.concatenate(all_t),
        pos0=np.concatenate(all_p0),
        vel0=np.concatenate(all_v0),
        pos1=np.concatenate(all_p1),
        vel1=np.concatenate(all_v1),
    )


# ---------------------------------------------------------------------
# Cubic spline profile
# ---------------------------------------------------------------------
def spline_multiaxis(
    waypoint_times: Sequence[float],
    waypoints: Sequence[Tuple[float, float]],
    dt: float,
) -> TrajectorySamples:
    """Velocity-continuous natural cubic spline through `waypoints` hitting
    each one at the corresponding `waypoint_times` (strictly increasing,
    same length as waypoints)."""
    if len(waypoints) < 2:
        raise TrajectoryError("need at least 2 waypoints for a spline")
    if len(waypoint_times) != len(waypoints):
        raise TrajectoryError("waypoint_times must match waypoints length")
    wt = np.asarray(waypoint_times, dtype=float)
    if np.any(np.diff(wt) <= 0):
        raise TrajectoryError("waypoint times must be strictly increasing")

    p0 = np.array([w[0] for w in waypoints])
    p1 = np.array([w[1] for w in waypoints])

    cs0 = CubicSpline(wt, p0, bc_type="clamped")
    cs1 = CubicSpline(wt, p1, bc_type="clamped")

    T = wt[-1] - wt[0]
    n = max(2, int(round(T / dt)) + 1)
    t = np.linspace(wt[0], wt[-1], n)

    return TrajectorySamples(
        t=t - wt[0],
        pos0=cs0(t),
        vel0=cs0(t, 1),
        pos1=cs1(t),
        vel1=cs1(t, 1),
    )


# ---------------------------------------------------------------------
# Cartesian (SCARA) wrappers -- generate the path in x/y then convert to
# joint space via inverse kinematics sample-by-sample.
# ---------------------------------------------------------------------
def cartesian_trapezoidal(
    geo: "kin.FiveBarGeometry",
    waypoints_xy: Sequence[Tuple[float, float]],
    limits_x: AxisLimits,
    limits_y: AxisLimits,
    dt: float,
    dwell: float = 0.0,
) -> TrajectorySamples:
    xy_traj = trapezoidal_multiaxis(waypoints_xy, limits_x, limits_y, dt, dwell)
    return _xy_traj_to_joint(geo, xy_traj)


def cartesian_spline(
    geo: "kin.FiveBarGeometry",
    waypoint_times: Sequence[float],
    waypoints_xy: Sequence[Tuple[float, float]],
    dt: float,
) -> TrajectorySamples:
    xy_traj = spline_multiaxis(waypoint_times, waypoints_xy, dt)
    return _xy_traj_to_joint(geo, xy_traj)


def _xy_traj_to_joint(geo: "kin.FiveBarGeometry", xy_traj: TrajectorySamples) -> TrajectorySamples:
    n = len(xy_traj.t)
    theta1 = np.zeros(n)
    theta2 = np.zeros(n)
    for i in range(n):
        x, y = xy_traj.pos0[i], xy_traj.pos1[i]
        try:
            t1, t2 = kin.inverse_kinematics(geo, x, y)
        except kin.KinematicsError as ex:
            raise TrajectoryError(
                f"Cartesian trajectory leaves the workspace at t={xy_traj.t[i]:.3f}s "
                f"(x={x:.4f}, y={y:.4f}): {ex}"
            )
        # keep continuity with the previous sample (avoid +/- 2*pi jumps)
        if i > 0:
            theta1[i] = theta1[i - 1] + _wrap_delta(t1 - theta1[i - 1])
            theta2[i] = theta2[i - 1] + _wrap_delta(t2 - theta2[i - 1])
        else:
            theta1[i] = t1
            theta2[i] = t2

    dt = xy_traj.t[1] - xy_traj.t[0] if n > 1 else 1.0
    vel0_rad = np.gradient(theta1, dt) if n > 1 else np.zeros(n)
    vel1_rad = np.gradient(theta2, dt) if n > 1 else np.zeros(n)

    # IMPORTANT: kin.inverse_kinematics works in radians, but every consumer
    # of TrajectorySamples (the streaming loop, the preview plot, the
    # "turns" waypoint convention used by Joint-space mode) expects ODrive
    # native units of *turns* and *turns/s*. Convert here, once, so pos0/pos1
    # are always turns regardless of which mode generated them.
    TWO_PI = 2 * math.pi
    pos0_turns = theta1 / TWO_PI
    pos1_turns = theta2 / TWO_PI
    vel0_turns = vel0_rad / TWO_PI
    vel1_turns = vel1_rad / TWO_PI

    return TrajectorySamples(
        t=xy_traj.t.copy(),
        pos0=pos0_turns, vel0=vel0_turns,
        pos1=pos1_turns, vel1=vel1_turns,
        x=xy_traj.pos0.copy(), y=xy_traj.pos1.copy(),
    )


def _wrap_delta(d: float) -> float:
    return math.atan2(math.sin(d), math.cos(d))
