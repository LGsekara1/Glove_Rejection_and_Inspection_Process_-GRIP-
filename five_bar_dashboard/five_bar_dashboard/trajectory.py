"""Firmware trapezoid timing and two-axis synchronisation helpers.

The ODrive 0.5.x trap planner exposes independent velocity, acceleration and
 deceleration limits.  The helpers here model an asymmetric trapezoid so the GUI
can edit all three values without silently forcing deceleration=acceleration.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class TrapProfile:
    distance: float
    vel_limit: float
    accel_limit: float
    decel_limit: float
    peak_velocity: float
    total_time: float
    triangular: bool
    accel_time: float
    cruise_time: float
    decel_time: float


def compute_trap_profile(
    distance: float,
    vmax: float,
    amax: float,
    dmax: float | None = None,
) -> TrapProfile:
    """Return the ideal speed profile for one positive move distance.

    ``dmax`` defaults to ``amax`` for compatibility with the original symmetric
    implementation.  Units only need to be internally consistent, for example
    turns / turns-s / turns-s² or degrees / degrees-s / degrees-s².
    """
    distance = abs(float(distance))
    vmax = abs(float(vmax))
    amax = abs(float(amax))
    dmax = amax if dmax is None else abs(float(dmax))
    if vmax <= 0 or amax <= 0 or dmax <= 0:
        raise ValueError("Velocity, acceleration and deceleration limits must be positive.")
    if distance <= 1e-12:
        return TrapProfile(
            distance=0.0,
            vel_limit=vmax,
            accel_limit=amax,
            decel_limit=dmax,
            peak_velocity=0.0,
            total_time=0.0,
            triangular=True,
            accel_time=0.0,
            cruise_time=0.0,
            decel_time=0.0,
        )

    accel_distance_at_vmax = vmax * vmax / (2.0 * amax)
    decel_distance_at_vmax = vmax * vmax / (2.0 * dmax)
    if accel_distance_at_vmax + decel_distance_at_vmax >= distance:
        # No constant-speed section.  Solve
        # distance = vp²/(2a) + vp²/(2d).
        vpeak = math.sqrt(
            2.0 * distance / ((1.0 / amax) + (1.0 / dmax))
        )
        t_acc = vpeak / amax
        t_dec = vpeak / dmax
        return TrapProfile(
            distance=distance,
            vel_limit=vmax,
            accel_limit=amax,
            decel_limit=dmax,
            peak_velocity=vpeak,
            total_time=t_acc + t_dec,
            triangular=True,
            accel_time=t_acc,
            cruise_time=0.0,
            decel_time=t_dec,
        )

    cruise_distance = distance - accel_distance_at_vmax - decel_distance_at_vmax
    t_acc = vmax / amax
    t_cruise = cruise_distance / vmax
    t_dec = vmax / dmax
    return TrapProfile(
        distance=distance,
        vel_limit=vmax,
        accel_limit=amax,
        decel_limit=dmax,
        peak_velocity=vmax,
        total_time=t_acc + t_cruise + t_dec,
        triangular=False,
        accel_time=t_acc,
        cruise_time=t_cruise,
        decel_time=t_dec,
    )


def profile_velocity_samples(
    profile: TrapProfile, sample_count: int = 121
) -> tuple[list[float], list[float]]:
    """Generate time/speed samples for the GUI preview."""
    sample_count = max(2, int(sample_count))
    if profile.total_time <= 0:
        return [0.0, 0.0], [0.0, 0.0]
    times = [profile.total_time * i / (sample_count - 1) for i in range(sample_count)]
    velocities: list[float] = []
    t1 = profile.accel_time
    t2 = t1 + profile.cruise_time
    for t in times:
        if t <= t1:
            velocity = profile.accel_limit * t
        elif t <= t2:
            velocity = profile.peak_velocity
        else:
            velocity = max(
                0.0,
                profile.peak_velocity - profile.decel_limit * (t - t2),
            )
        velocities.append(min(profile.peak_velocity, velocity))
    velocities[-1] = 0.0
    return times, velocities


def scaled_limits_for_duration_asymmetric(
    distance: float,
    vmax: float,
    amax: float,
    dmax: float,
    target_time: float,
) -> tuple[float, float, float]:
    """Scale V/A/D together so the profile duration matches ``target_time``."""
    base = compute_trap_profile(distance, vmax, amax, dmax)
    if base.total_time <= 0 or target_time <= base.total_time + 1e-7:
        return vmax, amax, dmax
    lo, hi = 1e-6, 1.0
    for _ in range(90):
        mid = (lo + hi) / 2.0
        duration = compute_trap_profile(
            distance, vmax * mid, amax * mid, dmax * mid
        ).total_time
        if duration > target_time:
            lo = mid
        else:
            hi = mid
    k = hi
    return vmax * k, amax * k, dmax * k


def scaled_limits_for_duration(
    distance: float,
    vmax: float,
    amax: float,
    target_time: float,
) -> tuple[float, float]:
    """Backward-compatible symmetric wrapper."""
    v, a, _ = scaled_limits_for_duration_asymmetric(
        distance, vmax, amax, amax, target_time
    )
    return v, a


def synchronise_two_axes_asymmetric(
    distances: tuple[float, float],
    velocities: tuple[float, float],
    accelerations: tuple[float, float],
    decelerations: tuple[float, float],
) -> tuple[
    tuple[float, float],
    tuple[float, float],
    tuple[float, float],
    float,
]:
    """Time-synchronise two independently configured asymmetric profiles."""
    p0 = compute_trap_profile(
        distances[0], velocities[0], accelerations[0], decelerations[0]
    )
    p1 = compute_trap_profile(
        distances[1], velocities[1], accelerations[1], decelerations[1]
    )
    target_time = max(p0.total_time, p1.total_time)
    v0, a0, d0 = scaled_limits_for_duration_asymmetric(
        distances[0], velocities[0], accelerations[0], decelerations[0], target_time
    )
    v1, a1, d1 = scaled_limits_for_duration_asymmetric(
        distances[1], velocities[1], accelerations[1], decelerations[1], target_time
    )
    return (v0, v1), (a0, a1), (d0, d1), target_time


def synchronise_two_axes(
    distances: tuple[float, float],
    velocities: tuple[float, float],
    accelerations: tuple[float, float],
) -> tuple[tuple[float, float], tuple[float, float], float]:
    """Backward-compatible symmetric wrapper used by older callers/tests."""
    velocities_out, accelerations_out, _, target_time = (
        synchronise_two_axes_asymmetric(
            distances, velocities, accelerations, accelerations
        )
    )
    return velocities_out, accelerations_out, target_time
