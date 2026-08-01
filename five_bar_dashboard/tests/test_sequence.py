import pytest

from five_bar_dashboard.models import GeometryConfig
from five_bar_dashboard.sequence import (
    compile_cartesian_sequence,
    normalise_waypoints,
    waypoints_to_jsonable,
)


def test_normalise_waypoints_accepts_dicts_and_tuples():
    points = normalise_waypoints(
        [{"x": 0, "y": 400, "dwell": 0.25}, (25, 425, 0.1)]
    )
    assert points[0].x_mm == 0
    assert points[0].dwell_s == pytest.approx(0.25)
    assert points[1].y_mm == 425


def test_compile_sequence_solves_all_points_before_motion():
    compiled = compile_cartesian_sequence(
        [{"x_mm": 0, "y_mm": 400, "dwell_s": 0.0}, {"x_mm": 20, "y_mm": 420}],
        GeometryConfig(),
    )
    assert len(compiled) == 2
    assert compiled[0].index == 1
    assert isinstance(compiled[0].theta0_deg, float)
    assert isinstance(compiled[0].theta1_deg, float)


def test_compile_sequence_rejects_unreachable_point_with_index():
    with pytest.raises(ValueError, match="Point 2"):
        compile_cartesian_sequence(
            [{"x": 0, "y": 400}, {"x": 0, "y": 5000}], GeometryConfig()
        )


def test_negative_dwell_is_rejected():
    with pytest.raises(ValueError, match="dwell"):
        normalise_waypoints([{"x": 0, "y": 400, "dwell_s": -0.1}])


def test_jsonable_format_is_stable():
    assert waypoints_to_jsonable([(1, 2, 0.5)]) == [
        {"x_mm": 1.0, "y_mm": 2.0, "dwell_s": 0.5}
    ]


def test_sequence_profile_fields_are_preserved():
    from five_bar_dashboard.models import TrajectoryConfig

    compiled = compile_cartesian_sequence(
        [
            {
                "x_mm": 0.0,
                "y_mm": 400.0,
                "max_vel_deg_s": 25.0,
                "max_accel_deg_s2": 80.0,
                "max_decel_deg_s2": 45.0,
            }
        ],
        GeometryConfig(),
        TrajectoryConfig(),
    )
    point = compiled[0]
    assert point.max_vel_deg_s == pytest.approx(25.0)
    assert point.max_accel_deg_s2 == pytest.approx(80.0)
    assert point.max_decel_deg_s2 == pytest.approx(45.0)


def test_old_sequence_uses_configured_profile_defaults():
    from five_bar_dashboard.models import TrajectoryConfig

    defaults = TrajectoryConfig(30.0, 70.0, 55.0)
    point = normalise_waypoints([{"x": 0, "y": 400}], defaults)[0]
    assert point.max_vel_deg_s == pytest.approx(30.0)
    assert point.max_accel_deg_s2 == pytest.approx(70.0)
    assert point.max_decel_deg_s2 == pytest.approx(55.0)


def test_non_positive_profile_is_rejected():
    with pytest.raises(ValueError, match="deceleration"):
        normalise_waypoints(
            [{"x": 0, "y": 400, "max_decel_deg_s2": 0.0}]
        )


def test_sequence_feedforward_fields_are_preserved():
    point = compile_cartesian_sequence(
        [
            {
                "x_mm": 0.0,
                "y_mm": 400.0,
                "velocity_ff0_turns_s": 0.12,
                "velocity_ff1_turns_s": -0.08,
                "torque_ff0_nm": 0.25,
                "torque_ff1_nm": -0.15,
            }
        ],
        GeometryConfig(),
    )[0]
    assert point.velocity_ff0_turns_s == pytest.approx(0.12)
    assert point.velocity_ff1_turns_s == pytest.approx(-0.08)
    assert point.torque_ff0_nm == pytest.approx(0.25)
    assert point.torque_ff1_nm == pytest.approx(-0.15)


def test_legacy_sequence_defaults_feedforward_to_zero():
    point = normalise_waypoints([{"x": 0, "y": 400}])[0]
    assert point.velocity_ff0_turns_s == 0.0
    assert point.velocity_ff1_turns_s == 0.0
    assert point.torque_ff0_nm == 0.0
    assert point.torque_ff1_nm == 0.0


def test_jsonable_profile_includes_feedforward():
    rows = waypoints_to_jsonable(
        [
            {
                "x": 1,
                "y": 2,
                "velocity_ff0_turns_s": 0.1,
                "torque_ff1_nm": -0.2,
            }
        ],
        include_profile=True,
    )
    assert rows[0]["velocity_ff0_turns_s"] == pytest.approx(0.1)
    assert rows[0]["torque_ff1_nm"] == pytest.approx(-0.2)
