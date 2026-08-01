import math

from five_bar_dashboard.trajectory import compute_trap_profile, synchronise_two_axes


def test_triangular_profile():
    profile = compute_trap_profile(0.1, 10.0, 2.0)
    assert profile.triangular
    assert profile.total_time > 0


def test_two_axis_synchronisation():
    distances = (1.0, 0.25)
    velocities = (2.0, 2.0)
    accelerations = (4.0, 4.0)
    synced_v, synced_a, target = synchronise_two_axes(distances, velocities, accelerations)
    t0 = compute_trap_profile(distances[0], synced_v[0], synced_a[0]).total_time
    t1 = compute_trap_profile(distances[1], synced_v[1], synced_a[1]).total_time
    assert math.isclose(t0, target, rel_tol=1e-5, abs_tol=1e-5)
    assert math.isclose(t1, target, rel_tol=1e-5, abs_tol=1e-5)


def test_asymmetric_trapezoid_uses_independent_deceleration():
    fast_brake = compute_trap_profile(20.0, 10.0, 5.0, 20.0)
    slow_brake = compute_trap_profile(20.0, 10.0, 5.0, 2.0)
    assert fast_brake.decel_time < slow_brake.decel_time
    assert fast_brake.decel_limit == 20.0
    assert slow_brake.decel_limit == 2.0


def test_asymmetric_two_axis_synchronisation():
    from five_bar_dashboard.trajectory import synchronise_two_axes_asymmetric

    distances = (1.0, 0.4)
    velocities = (2.0, 3.0)
    accelerations = (4.0, 6.0)
    decelerations = (8.0, 2.5)
    synced_v, synced_a, synced_d, target = synchronise_two_axes_asymmetric(
        distances, velocities, accelerations, decelerations
    )
    t0 = compute_trap_profile(
        distances[0], synced_v[0], synced_a[0], synced_d[0]
    ).total_time
    t1 = compute_trap_profile(
        distances[1], synced_v[1], synced_a[1], synced_d[1]
    ).total_time
    assert math.isclose(t0, target, rel_tol=1e-5, abs_tol=1e-5)
    assert math.isclose(t1, target, rel_tol=1e-5, abs_tol=1e-5)
