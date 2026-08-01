"""Configuration and data-transfer models."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

from .constants import ENCODER_MODE_SPI_ABS_AMS


@dataclass(slots=True)
class GeometryConfig:
    L0: float = 300.0
    l1a: float = 300.0
    l2a: float = 450.0
    l1b: float = 300.0
    l2b: float = 450.0
    elbow1: str = "up"
    elbow2: str = "down"
    fk_branch: str = "upper"


@dataclass(slots=True)
class AxisMappingConfig:
    gear_ratio: float = 1.0
    offset_turns: float = 0.0
    direction: float = -1.0


@dataclass(slots=True)
class EncoderInterfaceConfig:
    mode: int = ENCODER_MODE_SPI_ABS_AMS
    cs_gpio: int = 4


@dataclass(slots=True)
class TrajectoryConfig:
    max_vel_deg_s: float = 60.0
    max_accel_deg_s2: float = 120.0
    max_decel_deg_s2: float = 120.0


@dataclass(slots=True)
class VelocityControlConfig:
    loop_hz: float = 60.0
    joint_vel_cap_deg_s: float = 45.0
    joint_accel_cap_deg_s2: float = 180.0
    max_cart_speed_mm_s: float = 80.0
    pos_kp: float = 3.0
    pos_tol_mm: float = 1.0
    manip_soft_deg_mm: float = 3.0
    manip_hard_deg_mm: float = 8.0
    watchdog_s: float = 0.15
    deadman_s: float = 0.5


@dataclass(slots=True)
class VelocityFilterConfig:
    type: str = "Moving Average"
    window: int = 5
    cutoff_hz: float = 10.0
    order: int = 2


@dataclass(slots=True)
class EncoderNoiseConfig:
    # Live position smoothing and movement detection are derived from position history,
    # not from the noisy ODrive encoder.vel_estimate value.
    position_median_window: int = 7
    motion_window_s: float = 0.40
    motion_deadband_turns_s: float = 0.003
    # Software-reference sync takes many samples and uses a robust median.
    sync_sample_duration_s: float = 1.50
    sync_sample_hz: float = 60.0
    sync_max_drift_turns_s: float = 0.003
    sync_noise_warning_span_turns: float = 0.003
    sync_hard_span_turns: float = 0.015


@dataclass(slots=True)
class DisplayConfig:
    show_workspace: bool = True
    px_per_mm: float = 1.0
    auto_fit: bool = True
    telemetry_window_s: float = 20.0


@dataclass(slots=True)
class DashboardConfig:
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    axes: dict[int, AxisMappingConfig] = field(
        default_factory=lambda: {0: AxisMappingConfig(), 1: AxisMappingConfig()}
    )
    home_angle_deg: dict[int, float] = field(default_factory=lambda: {0: 90.0, 1: 90.0})
    spi: dict[int, EncoderInterfaceConfig] = field(
        default_factory=lambda: {
            0: EncoderInterfaceConfig(mode=ENCODER_MODE_SPI_ABS_AMS, cs_gpio=4),
            1: EncoderInterfaceConfig(mode=ENCODER_MODE_SPI_ABS_AMS, cs_gpio=3),
        }
    )
    trajectory: TrajectoryConfig = field(default_factory=TrajectoryConfig)
    velocity: VelocityControlConfig = field(default_factory=VelocityControlConfig)
    filters: dict[int, VelocityFilterConfig] = field(
        default_factory=lambda: {0: VelocityFilterConfig(), 1: VelocityFilterConfig()}
    )
    encoder_noise: EncoderNoiseConfig = field(default_factory=EncoderNoiseConfig)
    display: DisplayConfig = field(default_factory=DisplayConfig)

    def to_dict(self) -> dict[str, Any]:
        raw = asdict(self)
        raw["axes"] = {str(k): v for k, v in raw["axes"].items()}
        raw["home_angle_deg"] = {str(k): v for k, v in raw["home_angle_deg"].items()}
        raw["spi"] = {str(k): v for k, v in raw["spi"].items()}
        raw["filters"] = {str(k): v for k, v in raw["filters"].items()}
        return raw

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "DashboardConfig":
        """Load only known keys, allowing older or newer JSON files to coexist safely."""
        cfg = cls()

        def merge_dataclass(obj: Any, values: Any) -> None:
            if not isinstance(values, dict):
                return
            for key, value in values.items():
                if hasattr(obj, key):
                    setattr(obj, key, value)

        merge_dataclass(cfg.geometry, data.get("geometry"))
        merge_dataclass(cfg.trajectory, data.get("trajectory"))
        merge_dataclass(cfg.velocity, data.get("velocity"))
        merge_dataclass(cfg.encoder_noise, data.get("encoder_noise"))
        merge_dataclass(cfg.display, data.get("display"))

        for axis in (0, 1):
            merge_dataclass(cfg.axes[axis], (data.get("axes") or {}).get(str(axis), {}))
            merge_dataclass(cfg.spi[axis], (data.get("spi") or {}).get(str(axis), {}))
            merge_dataclass(cfg.filters[axis], (data.get("filters") or {}).get(str(axis), {}))
            if str(axis) in (data.get("home_angle_deg") or {}):
                cfg.home_angle_deg[axis] = float(data["home_angle_deg"][str(axis)])

        cfg.validate()
        return cfg

    def validate(self) -> None:
        g = self.geometry
        if min(g.L0, g.l1a, g.l2a, g.l1b, g.l2b) <= 0:
            raise ValueError("All linkage dimensions must be positive.")
        if g.elbow1 not in {"up", "down"} or g.elbow2 not in {"up", "down"}:
            raise ValueError("Elbow selectors must be 'up' or 'down'.")
        if g.fk_branch not in {"upper", "lower"}:
            raise ValueError("FK branch must be 'upper' or 'lower'.")
        for axis in (0, 1):
            mapping = self.axes[axis]
            if mapping.gear_ratio <= 0:
                raise ValueError(f"Axis {axis} gear ratio must be positive.")
            if mapping.direction not in {-1.0, 1.0}:
                raise ValueError(f"Axis {axis} direction must be +1 or -1.")
            filt = self.filters[axis]
            if filt.window < 1:
                raise ValueError(f"Axis {axis} filter window must be >= 1.")
            if filt.cutoff_hz <= 0:
                raise ValueError(f"Axis {axis} filter cutoff must be positive.")
            if filt.order < 1:
                raise ValueError(f"Axis {axis} Butterworth order must be >= 1.")
        if min(
            self.trajectory.max_vel_deg_s,
            self.trajectory.max_accel_deg_s2,
            self.trajectory.max_decel_deg_s2,
        ) <= 0:
            raise ValueError("Trajectory velocity, acceleration and deceleration limits must be positive.")
        if self.velocity.manip_hard_deg_mm <= self.velocity.manip_soft_deg_mm:
            raise ValueError("manip_hard_deg_mm must be greater than manip_soft_deg_mm.")
        if self.velocity.loop_hz <= 0:
            raise ValueError("Velocity loop frequency must be positive.")
        noise = self.encoder_noise
        if noise.position_median_window < 1:
            raise ValueError("position_median_window must be >= 1.")
        if noise.motion_window_s <= 0 or noise.sync_sample_duration_s <= 0:
            raise ValueError("Encoder noise time windows must be positive.")
        if noise.sync_sample_hz <= 0:
            raise ValueError("sync_sample_hz must be positive.")
        if min(
            noise.motion_deadband_turns_s,
            noise.sync_max_drift_turns_s,
            noise.sync_noise_warning_span_turns,
            noise.sync_hard_span_turns,
        ) < 0:
            raise ValueError("Encoder noise thresholds cannot be negative.")
        if noise.sync_hard_span_turns <= noise.sync_noise_warning_span_turns:
            raise ValueError("sync_hard_span_turns must exceed sync_noise_warning_span_turns.")


@dataclass(slots=True)
class TelemetrySample:
    t: float
    pos_deg: tuple[float, float]
    vel_raw_deg_s: tuple[float, float]
    vel_filtered_deg_s: tuple[float, float]
    current_a: tuple[float, float]
    theta_deg: tuple[float, float]
    p1: tuple[float, float] | None = None
    p2: tuple[float, float] | None = None
    end_effector: tuple[float, float] | None = None
    blocked: bool = False
    block_reason: str = ""
    # State-independent raw encoder telemetry. These values are populated while the
    # axes are IDLE as well as while they are in CLOSED_LOOP_CONTROL.
    raw_pos_turns: tuple[float, float] = (0.0, 0.0)
    filtered_pos_turns: tuple[float, float] = (0.0, 0.0)
    raw_vel_turns_s: tuple[float, float] = (0.0, 0.0)
    motion_estimate_turns_s: tuple[float, float] = (0.0, 0.0)
    stationary: tuple[bool, bool] = (True, True)
    axis_state: tuple[int, int] = (0, 0)
    axis_error: tuple[int, int] = (0, 0)
    motor_error: tuple[int, int] = (0, 0)
    encoder_error: tuple[int, int] = (0, 0)


@dataclass(slots=True)
class StepResponseResult:
    axis: int
    start_turns: float
    target_turns: float
    samples: list[tuple[float, float]]
    velocity_samples: list[tuple[float, float, float]]
    overshoot_pct: float
    settling_time_s: float | None
    final_error_turns: float
    gains: dict[str, float]
