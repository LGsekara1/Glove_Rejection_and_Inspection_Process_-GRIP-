from five_bar_dashboard.models import DashboardConfig


def test_known_key_merge_ignores_unknown_fields():
    cfg = DashboardConfig.from_dict(
        {
            "geometry": {"L0": 350.0, "removed_old_field": 42},
            "axes": {"0": {"gear_ratio": 2.0, "unknown": 1}},
            "unknown_root": {"a": 1},
        }
    )
    assert cfg.geometry.L0 == 350.0
    assert cfg.axes[0].gear_ratio == 2.0
    assert not hasattr(cfg.geometry, "removed_old_field")


def test_trajectory_deceleration_defaults_for_old_config():
    cfg = DashboardConfig.from_dict(
        {"trajectory": {"max_vel_deg_s": 50.0, "max_accel_deg_s2": 90.0}}
    )
    assert cfg.trajectory.max_decel_deg_s2 == 120.0


def test_trajectory_deceleration_round_trip():
    cfg = DashboardConfig.from_dict(
        {
            "trajectory": {
                "max_vel_deg_s": 50.0,
                "max_accel_deg_s2": 90.0,
                "max_decel_deg_s2": 70.0,
            }
        }
    )
    assert cfg.to_dict()["trajectory"]["max_decel_deg_s2"] == 70.0
