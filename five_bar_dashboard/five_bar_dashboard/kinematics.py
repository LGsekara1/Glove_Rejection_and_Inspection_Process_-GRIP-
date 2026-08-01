"""Pure five-bar linkage kinematics and Jacobian calculations."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .models import GeometryConfig


class KinematicsError(ValueError):
    pass


@dataclass(frozen=True, slots=True)
class ForwardKinematicsResult:
    end_effector: tuple[float, float]
    p1: tuple[float, float]
    p2: tuple[float, float]


def anchors(params: GeometryConfig) -> tuple[tuple[float, float], tuple[float, float]]:
    return (-params.L0 / 2.0, 0.0), (params.L0 / 2.0, 0.0)


def _arm_inverse(
    anchor: tuple[float, float],
    target: tuple[float, float],
    l1: float,
    l2: float,
    elbow: str,
) -> float:
    dx = target[0] - anchor[0]
    dy = target[1] - anchor[1]
    d = math.hypot(dx, dy)
    tol = 1e-9
    if d <= tol:
        raise KinematicsError("Target coincides with a base pivot.")
    if d > l1 + l2 + tol or d < abs(l1 - l2) - tol:
        raise KinematicsError("Target is outside one arm's annular workspace.")
    base_angle = math.atan2(dy, dx)
    cos_val = (l1 * l1 + d * d - l2 * l2) / (2.0 * l1 * d)
    cos_val = max(-1.0, min(1.0, cos_val))
    elbow_angle = math.acos(cos_val)
    if elbow == "up":
        return base_angle + elbow_angle
    if elbow == "down":
        return base_angle - elbow_angle
    raise KinematicsError(f"Unknown elbow selector: {elbow}")


def inverse_kinematics(x: float, y: float, params: GeometryConfig) -> tuple[float, float]:
    a, b = anchors(params)
    t1 = _arm_inverse(a, (x, y), params.l1a, params.l2a, params.elbow1)
    t2 = _arm_inverse(b, (x, y), params.l1b, params.l2b, params.elbow2)
    return math.degrees(t1), math.degrees(t2)


def _circle_intersections(
    c1: tuple[float, float],
    r1: float,
    c2: tuple[float, float],
    r2: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    dx = c2[0] - c1[0]
    dy = c2[1] - c1[1]
    d = math.hypot(dx, dy)
    tol = 1e-9
    if d <= tol:
        raise KinematicsError("Distal-link circle centres coincide.")
    if d > r1 + r2 + tol or d < abs(r1 - r2) - tol:
        raise KinematicsError("Joint angles do not close the five-bar linkage.")

    a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d)
    h_sq = max(0.0, r1 * r1 - a * a)
    h = math.sqrt(h_sq)
    xm = c1[0] + a * dx / d
    ym = c1[1] + a * dy / d
    rx = -dy * h / d
    ry = dx * h / d
    return (xm + rx, ym + ry), (xm - rx, ym - ry)


def forward_kinematics(
    theta1_deg: float,
    theta2_deg: float,
    params: GeometryConfig,
) -> ForwardKinematicsResult:
    a, b = anchors(params)
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    p1 = (a[0] + params.l1a * math.cos(t1), a[1] + params.l1a * math.sin(t1))
    p2 = (b[0] + params.l1b * math.cos(t2), b[1] + params.l1b * math.sin(t2))
    e1, e2 = _circle_intersections(p1, params.l2a, p2, params.l2b)
    if params.fk_branch == "upper":
        e = e1 if e1[1] >= e2[1] else e2
    elif params.fk_branch == "lower":
        e = e1 if e1[1] <= e2[1] else e2
    else:
        raise KinematicsError(f"Unknown FK branch: {params.fk_branch}")
    return ForwardKinematicsResult(e, p1, p2)


def numerical_jacobian(
    x: float,
    y: float,
    params: GeometryConfig,
    eps: float = 0.5,
) -> np.ndarray:
    for step in (eps, eps / 2.0, eps / 5.0, eps / 10.0, eps / 25.0):
        try:
            xp = inverse_kinematics(x + step, y, params)
            xm = inverse_kinematics(x - step, y, params)
            yp = inverse_kinematics(x, y + step, params)
            ym = inverse_kinematics(x, y - step, params)
        except KinematicsError:
            continue
        return np.array(
            [
                [(xp[0] - xm[0]) / (2.0 * step), (yp[0] - ym[0]) / (2.0 * step)],
                [(xp[1] - xm[1]) / (2.0 * step), (yp[1] - ym[1]) / (2.0 * step)],
            ],
            dtype=float,
        )
    raise KinematicsError("Could not evaluate the numerical Jacobian near this pose.")


def singular_values_2x2(matrix: np.ndarray) -> tuple[float, float]:
    if matrix.shape != (2, 2):
        raise ValueError("Expected a 2x2 matrix.")
    a, b = float(matrix[0, 0]), float(matrix[0, 1])
    c, d = float(matrix[1, 0]), float(matrix[1, 1])
    e = (a + d) / 2.0
    f = (a - d) / 2.0
    g = (c + b) / 2.0
    h = (c - b) / 2.0
    q = math.hypot(e, h)
    r = math.hypot(f, g)
    sigma_max = q + r
    sigma_min = abs(q - r)
    return sigma_max, sigma_min


def cartesian_to_joint_velocity(
    vx: float,
    vy: float,
    x: float,
    y: float,
    params: GeometryConfig,
    manip_soft: float,
    manip_hard: float,
    joint_vel_cap_deg_s: float,
) -> tuple[np.ndarray, float, float]:
    jac = numerical_jacobian(x, y, params)
    sigma_max, sigma_min = singular_values_2x2(jac)
    if sigma_max >= manip_hard:
        return np.zeros(2), 0.0, sigma_max
    if sigma_max > manip_soft:
        derate = (manip_hard - sigma_max) / (manip_hard - manip_soft)
    else:
        derate = 1.0
    w = jac @ np.array([vx, vy], dtype=float) * derate
    peak = float(np.max(np.abs(w)))
    if peak > joint_vel_cap_deg_s > 0:
        w *= joint_vel_cap_deg_s / peak
    return w, derate, sigma_max


def slew_limit_vector(previous: np.ndarray, requested: np.ndarray, max_delta: float) -> np.ndarray:
    delta = requested - previous
    peak = float(np.max(np.abs(delta)))
    if peak <= max_delta or peak <= 0:
        return requested
    return previous + delta * (max_delta / peak)
