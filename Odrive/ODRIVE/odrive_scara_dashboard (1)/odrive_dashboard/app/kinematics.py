"""
kinematics.py
=============
Planar 5-bar parallel-linkage SCARA kinematics.

Geometry / naming convention
-----------------------------
Two motors are mounted on a fixed base, separated by distance `d`, symmetric
about the origin:

    O1 = (-d/2, 0)      <- axis0 motor pivot
    O2 = (+d/2, 0)      <- axis1 motor pivot

Each motor drives a proximal ("crank") link:

    L1a : O1 -> A   (driven by axis0, angle theta1 measured from +x axis)
    L1b : O2 -> B   (driven by axis1, angle theta2 measured from +x axis)

Two distal ("coupler") links connect the cranks to the shared end effector P:

    L2a : A -> P
    L2b : B -> P

That closed loop O1-A-P-B-O2-O1 has 5 links total (base counts as one),
hence "5-bar". Only theta1 and theta2 are actuated; the elbow points A and B
are passive.

Inverse kinematics decouples into two independent 2-link (elbow) problems:
for each side, given the fixed pivot O_i and the target P, solve for the
crank angle theta_i that places the elbow so the distal link exactly reaches
P. Which of the two IK solutions (elbow "up"/"out" vs "down"/"in") is
physically correct depends on how your linkage is actually assembled --
set `elbow_sign` per side to flip it if the computed pose looks mirrored
compared to your hardware.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class FiveBarGeometry:
    d: float = 0.10          # distance between the two motor pivots O1-O2 (m)
    l1a: float = 0.10        # axis0 crank length O1->A (m)
    l2a: float = 0.16        # axis0 coupler length A->P (m)
    l1b: float = 0.10        # axis1 crank length O2->B (m)
    l2b: float = 0.16        # axis1 coupler length B->P (m)
    elbow_sign_a: int = 1    # +1 or -1, flip if assembly doesn't match reality
    elbow_sign_b: int = -1
    theta1_min: float = -math.pi   # joint travel limits (radians), default = full range
    theta1_max: float = math.pi
    theta2_min: float = -math.pi
    theta2_max: float = math.pi

    @property
    def O1(self):
        return (-self.d / 2.0, 0.0)

    @property
    def O2(self):
        return (self.d / 2.0, 0.0)


class KinematicsError(ValueError):
    pass


def _elbow_angle(l_prox: float, l_dist: float, r: float) -> float:
    """Interior angle at the base pivot between the O->P line and the O->elbow
    crank, via the law of cosines. Raises KinematicsError if unreachable."""
    if r < 1e-9:
        raise KinematicsError("target coincides with pivot")
    cos_a = (l_prox ** 2 + r ** 2 - l_dist ** 2) / (2 * l_prox * r)
    if cos_a < -1.0 - 1e-6 or cos_a > 1.0 + 1e-6:
        raise KinematicsError("target out of reach for this side")
    cos_a = max(-1.0, min(1.0, cos_a))
    return math.acos(cos_a)


def inverse_kinematics(geo: FiveBarGeometry, x: float, y: float) -> tuple[float, float]:
    """Return (theta1, theta2) in radians required to place the end effector
    at (x, y). Raises KinematicsError if the point is not reachable."""
    O1, O2 = geo.O1, geo.O2

    r1 = math.hypot(x - O1[0], y - O1[1])
    beta1 = math.atan2(y - O1[1], x - O1[0])
    alpha1 = _elbow_angle(geo.l1a, geo.l2a, r1)
    theta1 = beta1 + geo.elbow_sign_a * alpha1

    r2 = math.hypot(x - O2[0], y - O2[1])
    beta2 = math.atan2(y - O2[1], x - O2[0])
    alpha2 = _elbow_angle(geo.l1b, geo.l2b, r2)
    theta2 = beta2 + geo.elbow_sign_b * alpha2

    return theta1, theta2


def forward_kinematics(geo: FiveBarGeometry, theta1: float, theta2: float) -> tuple[float, float]:
    """Return the end-effector (x, y) for given crank angles, by intersecting
    the two coupler-link circles centred at the elbow points. Picks the
    intersection consistent with the configured elbow_sign convention."""
    O1, O2 = geo.O1, geo.O2
    A = (O1[0] + geo.l1a * math.cos(theta1), O1[1] + geo.l1a * math.sin(theta1))
    B = (O2[0] + geo.l1b * math.cos(theta2), O2[1] + geo.l1b * math.sin(theta2))

    dx, dy = B[0] - A[0], B[1] - A[1]
    dAB = math.hypot(dx, dy)
    if dAB < 1e-9:
        raise KinematicsError("elbow points coincide - singular configuration")
    if dAB > geo.l2a + geo.l2b + 1e-9 or dAB < abs(geo.l2a - geo.l2b) - 1e-9:
        raise KinematicsError("coupler links cannot close the loop for these angles")

    a = (geo.l2a ** 2 - geo.l2b ** 2 + dAB ** 2) / (2 * dAB)
    h_sq = geo.l2a ** 2 - a ** 2
    h = math.sqrt(max(0.0, h_sq))

    xm = A[0] + a * dx / dAB
    ym = A[1] + a * dy / dAB

    p_candidates = [
        (xm + h * dy / dAB, ym - h * dx / dAB),
        (xm - h * dy / dAB, ym + h * dx / dAB),
    ]

    # Choose the candidate whose re-computed IK best matches the requested
    # angles (handles the elbow-configuration ambiguity robustly).
    best = min(
        p_candidates,
        key=lambda p: _angle_diff(theta1, theta2, geo, p),
    )
    return best


def _angle_diff(theta1, theta2, geo, p):
    try:
        t1, t2 = inverse_kinematics(geo, p[0], p[1])
    except KinematicsError:
        return float("inf")
    d1 = math.atan2(math.sin(t1 - theta1), math.cos(t1 - theta1))
    d2 = math.atan2(math.sin(t2 - theta2), math.cos(t2 - theta2))
    return abs(d1) + abs(d2)


def elbow_points(geo: FiveBarGeometry, theta1: float, theta2: float):
    O1, O2 = geo.O1, geo.O2
    A = (O1[0] + geo.l1a * math.cos(theta1), O1[1] + geo.l1a * math.sin(theta1))
    B = (O2[0] + geo.l1b * math.cos(theta2), O2[1] + geo.l1b * math.sin(theta2))
    return A, B


def workspace_grid(geo: FiveBarGeometry, resolution: int = 220, margin: float = 1.15):
    """Vectorised reachability sampling over a square grid. Returns
    (xs, ys, mask) where mask[j, i] is True if (xs[i], ys[j]) is reachable
    respecting both link-length limits AND configured joint angle limits.
    This is used both to draw the shaded workspace and to bound trajectory
    / target pickers.
    """
    reach = max(geo.l1a + geo.l2a, geo.l1b + geo.l2b) + geo.d / 2.0
    extent = reach * margin
    xs = np.linspace(-extent, extent, resolution)
    ys = np.linspace(-extent, extent, resolution)
    X, Y = np.meshgrid(xs, ys)

    O1x, O1y = geo.O1
    O2x, O2y = geo.O2

    r1 = np.hypot(X - O1x, Y - O1y)
    r2 = np.hypot(X - O2x, Y - O2y)

    reach1_min, reach1_max = abs(geo.l1a - geo.l2a), geo.l1a + geo.l2a
    reach2_min, reach2_max = abs(geo.l1b - geo.l2b), geo.l1b + geo.l2b

    reachable = (
        (r1 >= reach1_min) & (r1 <= reach1_max) &
        (r2 >= reach2_min) & (r2 <= reach2_max)
    )

    # Respect joint angle limits by checking the required theta1/theta2 fall
    # in range (only where the base annulus condition already holds, to keep
    # the acos() domain safe).
    with np.errstate(invalid="ignore", divide="ignore"):
        beta1 = np.arctan2(Y - O1y, X - O1x)
        cos_a1 = (geo.l1a ** 2 + r1 ** 2 - geo.l2a ** 2) / (2 * geo.l1a * np.clip(r1, 1e-9, None))
        cos_a1 = np.clip(cos_a1, -1.0, 1.0)
        alpha1 = np.arccos(cos_a1)
        theta1 = beta1 + geo.elbow_sign_a * alpha1

        beta2 = np.arctan2(Y - O2y, X - O2x)
        cos_a2 = (geo.l1b ** 2 + r2 ** 2 - geo.l2b ** 2) / (2 * geo.l1b * np.clip(r2, 1e-9, None))
        cos_a2 = np.clip(cos_a2, -1.0, 1.0)
        alpha2 = np.arccos(cos_a2)
        theta2 = beta2 + geo.elbow_sign_b * alpha2

    def in_range(theta, lo, hi):
        # normalise into (-pi, pi] then handle wrapped ranges
        t = np.arctan2(np.sin(theta), np.cos(theta))
        if lo <= hi:
            return (t >= lo) & (t <= hi)
        return (t >= lo) | (t <= hi)

    full_range_a = (geo.theta1_min <= -math.pi + 1e-6) and (geo.theta1_max >= math.pi - 1e-6)
    full_range_b = (geo.theta2_min <= -math.pi + 1e-6) and (geo.theta2_max >= math.pi - 1e-6)

    if not full_range_a:
        reachable &= in_range(theta1, geo.theta1_min, geo.theta1_max)
    if not full_range_b:
        reachable &= in_range(theta2, geo.theta2_min, geo.theta2_max)

    return xs, ys, reachable


def is_point_reachable(geo: FiveBarGeometry, x: float, y: float) -> tuple[bool, str]:
    try:
        t1, t2 = inverse_kinematics(geo, x, y)
    except KinematicsError as ex:
        return False, str(ex)
    if not (geo.theta1_min <= t1 <= geo.theta1_max or geo.theta1_min > geo.theta1_max):
        return False, "outside axis0 joint travel limit"
    if not (geo.theta2_min <= t2 <= geo.theta2_max or geo.theta2_min > geo.theta2_max):
        return False, "outside axis1 joint travel limit"
    return True, "ok"
