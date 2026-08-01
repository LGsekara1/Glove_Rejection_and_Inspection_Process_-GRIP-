import math

from five_bar_dashboard.kinematics import forward_kinematics, inverse_kinematics, numerical_jacobian
from five_bar_dashboard.models import GeometryConfig


def test_ik_fk_round_trip_default_geometry():
    params = GeometryConfig()
    target = (0.0, 400.0)
    theta = inverse_kinematics(*target, params)
    result = forward_kinematics(*theta, params)
    assert math.isclose(result.end_effector[0], target[0], abs_tol=1e-6)
    assert math.isclose(result.end_effector[1], target[1], abs_tol=1e-6)


def test_numerical_jacobian_shape_and_finiteness():
    jac = numerical_jacobian(0.0, 400.0, GeometryConfig())
    assert jac.shape == (2, 2)
    assert all(math.isfinite(float(v)) for v in jac.flat)
