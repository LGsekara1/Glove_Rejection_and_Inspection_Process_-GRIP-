"""Motor-turn and joint-angle conversions."""
from __future__ import annotations

from .models import AxisMappingConfig


def angle_deg_to_turns(angle_deg: float, home_angle_deg: float, cfg: AxisMappingConfig) -> float:
    return cfg.offset_turns + cfg.direction * ((angle_deg - home_angle_deg) / 360.0) * cfg.gear_ratio


def turns_to_angle_deg(turns: float, home_angle_deg: float, cfg: AxisMappingConfig) -> float:
    return ((turns - cfg.offset_turns) / cfg.gear_ratio) * 360.0 / cfg.direction + home_angle_deg


def deg_per_s_to_turns_per_s(deg_per_s: float, cfg: AxisMappingConfig) -> float:
    """Differentiate the stated position mapping to obtain a physically consistent rate mapping.

    The supplied rewrite specification prints a rate equation that is dimensionally inconsistent
    with its own position equations. This implementation uses the exact derivative of
    ``turns = offset + direction * ((angle-home)/360) * gear_ratio``.
    """
    return cfg.direction * cfg.gear_ratio * deg_per_s / 360.0


def turns_per_s_to_deg_per_s(turns_per_s: float, cfg: AxisMappingConfig) -> float:
    return turns_per_s * 360.0 / (cfg.direction * cfg.gear_ratio)


def abs_deg_rate_to_turn_rate(deg_rate: float, cfg: AxisMappingConfig) -> float:
    return abs(deg_per_s_to_turns_per_s(deg_rate, cfg))


def offset_for_reference_angle(
    raw_turns: float,
    reference_angle_deg: float,
    home_angle_deg: float,
    cfg: AxisMappingConfig,
) -> float:
    """Return the software offset that maps ``raw_turns`` to a known joint angle.

    This is the correct software-zero operation for an absolute encoder.  It preserves
    the configured gear ratio, direction and home-angle convention, and changes only
    ``offset_turns``.
    """
    return raw_turns - cfg.direction * (
        (reference_angle_deg - home_angle_deg) / 360.0
    ) * cfg.gear_ratio
