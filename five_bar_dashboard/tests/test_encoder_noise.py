import math

from five_bar_dashboard.encoder_noise import PositionMotionEstimator, robust_position_estimate


def test_sync_accepts_large_velocity_noise_when_position_is_stationary() -> None:
    times = [i / 60.0 for i in range(90)]
    # Several encoder counts of alternating position noise but no sustained drift.
    positions = [1.2345 + (0.00035 if i % 2 else -0.00035) for i in range(90)]
    result = robust_position_estimate(
        times,
        positions,
        max_drift_turns_s=0.003,
        noise_warning_span_turns=0.003,
        hard_span_turns=0.015,
    )
    assert result.stationary
    assert abs(result.position_turns - 1.2345) < 1e-5
    assert abs(result.drift_turns_s) < 0.003


def test_sync_rejects_sustained_position_drift() -> None:
    times = [i / 60.0 for i in range(90)]
    positions = [0.25 + 0.02 * t + 0.0001 * math.sin(30 * t) for t in times]
    result = robust_position_estimate(
        times,
        positions,
        max_drift_turns_s=0.003,
        noise_warning_span_turns=0.003,
        hard_span_turns=0.015,
    )
    assert not result.stationary
    assert result.drift_turns_s > 0.01


def test_live_motion_estimator_deadbands_stationary_jitter() -> None:
    estimator = PositionMotionEstimator(
        sample_rate_hz=50.0,
        median_window=7,
        motion_window_s=0.4,
        deadband_turns_s=0.003,
    )
    stationary = False
    rate = 99.0
    for i in range(50):
        _, rate, stationary = estimator.update(i / 50.0, 0.8 + (0.0002 if i % 2 else -0.0002))
    assert stationary
    assert rate == 0.0


def test_live_motion_estimator_detects_real_motion() -> None:
    estimator = PositionMotionEstimator(
        sample_rate_hz=50.0,
        median_window=5,
        motion_window_s=0.4,
        deadband_turns_s=0.003,
    )
    stationary = True
    rate = 0.0
    for i in range(50):
        _, rate, stationary = estimator.update(i / 50.0, 0.1 + 0.05 * i / 50.0)
    assert not stationary
    assert rate > 0.02
