import math

import pytest

from five_bar_dashboard.conversion import (
    angle_deg_to_turns,
    deg_per_s_to_turns_per_s,
    turns_per_s_to_deg_per_s,
    turns_to_angle_deg,
    offset_for_reference_angle,
)
from five_bar_dashboard.models import AxisMappingConfig


def test_position_round_trip():
    cfg = AxisMappingConfig(gear_ratio=2.5, offset_turns=1.2, direction=-1.0)
    angle = 137.5
    turns = angle_deg_to_turns(angle, 90.0, cfg)
    assert math.isclose(turns_to_angle_deg(turns, 90.0, cfg), angle, abs_tol=1e-12)


def test_rate_round_trip():
    cfg = AxisMappingConfig(gear_ratio=3.0, direction=-1.0)
    rate = 45.0
    turns_rate = deg_per_s_to_turns_per_s(rate, cfg)
    assert math.isclose(turns_per_s_to_deg_per_s(turns_rate, cfg), rate, abs_tol=1e-12)


def test_software_reference_offset_maps_raw_turns_to_known_angle() -> None:
    cfg = AxisMappingConfig(gear_ratio=7.5, offset_turns=0.0, direction=-1.0)
    raw_turns = 12.345678
    home_angle = 90.0
    known_angle = 42.5
    offset = offset_for_reference_angle(raw_turns, known_angle, home_angle, cfg)
    synced = AxisMappingConfig(
        gear_ratio=cfg.gear_ratio, offset_turns=offset, direction=cfg.direction
    )
    assert turns_to_angle_deg(raw_turns, home_angle, synced) == pytest.approx(known_angle)
