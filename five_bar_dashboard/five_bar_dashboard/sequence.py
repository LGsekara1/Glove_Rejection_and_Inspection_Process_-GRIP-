"""Cartesian waypoint sequence validation and compilation.

A coordinate sequence is implemented as discrete point-to-point moves.  Every
segment uses the ODrive firmware trapezoidal planner; this module does not stream
a PC-side interpolated trajectory.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable

from .kinematics import inverse_kinematics
from .models import GeometryConfig, TrajectoryConfig


@dataclass(frozen=True, slots=True)
class CartesianWaypoint:
    x_mm: float
    y_mm: float
    dwell_s: float = 0.0
    max_vel_deg_s: float = 60.0
    max_accel_deg_s2: float = 120.0
    max_decel_deg_s2: float = 120.0
    velocity_ff0_turns_s: float = 0.0
    velocity_ff1_turns_s: float = 0.0
    torque_ff0_nm: float = 0.0
    torque_ff1_nm: float = 0.0


@dataclass(frozen=True, slots=True)
class CompiledWaypoint:
    index: int
    x_mm: float
    y_mm: float
    dwell_s: float
    theta0_deg: float
    theta1_deg: float
    max_vel_deg_s: float
    max_accel_deg_s2: float
    max_decel_deg_s2: float
    velocity_ff0_turns_s: float
    velocity_ff1_turns_s: float
    torque_ff0_nm: float
    torque_ff1_nm: float


def _coerce_waypoint(
    value: Any,
    index: int,
    defaults: TrajectoryConfig,
) -> CartesianWaypoint:
    if isinstance(value, CartesianWaypoint):
        waypoint = value
    elif isinstance(value, dict):
        try:
            x = value["x_mm"] if "x_mm" in value else value["x"]
            y = value["y_mm"] if "y_mm" in value else value["y"]
            dwell = value.get("dwell_s", value.get("dwell", 0.0))
            vmax = value.get(
                "max_vel_deg_s", value.get("vel_deg_s", defaults.max_vel_deg_s)
            )
            accel = value.get(
                "max_accel_deg_s2",
                value.get("accel_deg_s2", defaults.max_accel_deg_s2),
            )
            decel = value.get(
                "max_decel_deg_s2",
                value.get("decel_deg_s2", defaults.max_decel_deg_s2),
            )
            vel_ff0 = value.get("velocity_ff0_turns_s", value.get("vel_ff0", 0.0))
            vel_ff1 = value.get("velocity_ff1_turns_s", value.get("vel_ff1", 0.0))
            torque_ff0 = value.get("torque_ff0_nm", value.get("torque_ff0", 0.0))
            torque_ff1 = value.get("torque_ff1_nm", value.get("torque_ff1", 0.0))
            waypoint = CartesianWaypoint(
                float(x), float(y), float(dwell), float(vmax), float(accel), float(decel),
                float(vel_ff0), float(vel_ff1), float(torque_ff0), float(torque_ff1)
            )
        except Exception as exc:
            raise ValueError(
                f"Point {index}: expected numeric x, y, dwell, velocity, acceleration and deceleration values."
            ) from exc
    else:
        try:
            values = list(value)
            if len(values) < 2:
                raise ValueError
            x, y = values[0], values[1]
            dwell = values[2] if len(values) >= 3 else 0.0
            vmax = values[3] if len(values) >= 4 else defaults.max_vel_deg_s
            accel = values[4] if len(values) >= 5 else defaults.max_accel_deg_s2
            decel = values[5] if len(values) >= 6 else defaults.max_decel_deg_s2
            vel_ff0 = values[6] if len(values) >= 7 else 0.0
            vel_ff1 = values[7] if len(values) >= 8 else 0.0
            torque_ff0 = values[8] if len(values) >= 9 else 0.0
            torque_ff1 = values[9] if len(values) >= 10 else 0.0
            waypoint = CartesianWaypoint(
                float(x), float(y), float(dwell), float(vmax), float(accel), float(decel),
                float(vel_ff0), float(vel_ff1), float(torque_ff0), float(torque_ff1)
            )
        except Exception as exc:
            raise ValueError(
                f"Point {index}: expected (x, y[, dwell_s[, vmax[, accel[, decel]]]])."
            ) from exc

    values = (
        waypoint.x_mm,
        waypoint.y_mm,
        waypoint.dwell_s,
        waypoint.max_vel_deg_s,
        waypoint.max_accel_deg_s2,
        waypoint.max_decel_deg_s2,
        waypoint.velocity_ff0_turns_s,
        waypoint.velocity_ff1_turns_s,
        waypoint.torque_ff0_nm,
        waypoint.torque_ff1_nm,
    )
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"Point {index}: all values must be finite.")
    if waypoint.dwell_s < 0.0:
        raise ValueError(f"Point {index}: dwell time cannot be negative.")
    if min(
        waypoint.max_vel_deg_s,
        waypoint.max_accel_deg_s2,
        waypoint.max_decel_deg_s2,
    ) <= 0.0:
        raise ValueError(
            f"Point {index}: velocity, acceleration and deceleration must be positive."
        )
    return waypoint


def normalise_waypoints(
    values: Iterable[Any], trajectory: TrajectoryConfig | None = None
) -> list[CartesianWaypoint]:
    defaults = trajectory or TrajectoryConfig()
    points = [
        _coerce_waypoint(value, index, defaults)
        for index, value in enumerate(values, start=1)
    ]
    if not points:
        raise ValueError("Add at least one coordinate before running the sequence.")
    return points


def compile_cartesian_sequence(
    values: Iterable[Any],
    geometry: GeometryConfig,
    trajectory: TrajectoryConfig | None = None,
) -> list[CompiledWaypoint]:
    """Validate every point and solve IK before any physical motion starts."""
    points = normalise_waypoints(values, trajectory)
    compiled: list[CompiledWaypoint] = []
    for index, point in enumerate(points, start=1):
        try:
            theta0, theta1 = inverse_kinematics(point.x_mm, point.y_mm, geometry)
        except Exception as exc:
            raise ValueError(
                f"Point {index} ({point.x_mm:.3f}, {point.y_mm:.3f}) mm is unreachable: {exc}"
            ) from exc
        compiled.append(
            CompiledWaypoint(
                index=index,
                x_mm=point.x_mm,
                y_mm=point.y_mm,
                dwell_s=point.dwell_s,
                theta0_deg=float(theta0),
                theta1_deg=float(theta1),
                max_vel_deg_s=point.max_vel_deg_s,
                max_accel_deg_s2=point.max_accel_deg_s2,
                max_decel_deg_s2=point.max_decel_deg_s2,
                velocity_ff0_turns_s=point.velocity_ff0_turns_s,
                velocity_ff1_turns_s=point.velocity_ff1_turns_s,
                torque_ff0_nm=point.torque_ff0_nm,
                torque_ff1_nm=point.torque_ff1_nm,
            )
        )
    return compiled


def waypoints_to_jsonable(
    values: Iterable[Any],
    trajectory: TrajectoryConfig | None = None,
    *,
    include_profile: bool = False,
) -> list[dict[str, float]]:
    output: list[dict[str, float]] = []
    for point in normalise_waypoints(values, trajectory):
        row = {"x_mm": point.x_mm, "y_mm": point.y_mm, "dwell_s": point.dwell_s}
        if include_profile:
            row.update(
                {
                    "max_vel_deg_s": point.max_vel_deg_s,
                    "max_accel_deg_s2": point.max_accel_deg_s2,
                    "max_decel_deg_s2": point.max_decel_deg_s2,
                    "velocity_ff0_turns_s": point.velocity_ff0_turns_s,
                    "velocity_ff1_turns_s": point.velocity_ff1_turns_s,
                    "torque_ff0_nm": point.torque_ff0_nm,
                    "torque_ff1_nm": point.torque_ff1_nm,
                }
            )
        output.append(row)
    return output
