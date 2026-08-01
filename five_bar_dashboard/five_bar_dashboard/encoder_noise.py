"""Noise-robust encoder position and stationary-state estimation.

The ODrive SPI absolute encoder velocity estimate can be noisy at rest.  Software-zero
synchronisation must therefore be based on a short position history, not on a single
``vel_estimate`` sample.
"""
from __future__ import annotations

from dataclasses import dataclass
from statistics import median
from typing import Sequence

import numpy as np


@dataclass(slots=True)
class RobustPositionEstimate:
    position_turns: float
    robust_span_turns: float
    drift_turns_s: float
    net_change_turns: float
    sample_count: int
    stationary: bool
    high_noise: bool


def unwrap_turn_sequence(values: Sequence[float]) -> list[float]:
    """Unwrap occasional ±1 turn discontinuities while preserving multi-turn position."""
    if not values:
        return []
    output = [float(values[0])]
    for raw in values[1:]:
        value = float(raw)
        previous = output[-1]
        # Absolute SPI encoders can report a wrap around the 0/1-turn boundary.  At the
        # dashboard sample rates the real mechanism cannot move half a turn in one tick.
        while value - previous > 0.5:
            value -= 1.0
        while value - previous < -0.5:
            value += 1.0
        output.append(value)
    return output


def robust_position_estimate(
    timestamps_s: Sequence[float],
    positions_turns: Sequence[float],
    *,
    max_drift_turns_s: float,
    noise_warning_span_turns: float,
    hard_span_turns: float,
) -> RobustPositionEstimate:
    """Estimate stationary position from noisy samples.

    Sustained motion is detected from the difference between the medians of the first and
    final quarters of the sample window.  Random velocity-estimate noise is deliberately
    ignored.  A 5–95 percentile span reports encoder noise and only blocks sync if it is
    extremely large (``hard_span_turns``).
    """
    if len(timestamps_s) != len(positions_turns):
        raise ValueError("timestamps and positions must have the same length")
    if len(positions_turns) < 8:
        raise ValueError("at least 8 encoder samples are required")

    t = np.asarray(timestamps_s, dtype=float)
    x = np.asarray(unwrap_turn_sequence(positions_turns), dtype=float)
    finite = np.isfinite(t) & np.isfinite(x)
    t = t[finite]
    x = x[finite]
    if x.size < 8:
        raise ValueError("not enough finite encoder samples")

    # Median/MAD rejection removes isolated SPI glitches without biasing the reference.
    centre = float(np.median(x))
    abs_dev = np.abs(x - centre)
    mad = float(np.median(abs_dev))
    reject_band = max(1e-5, 8.0 * 1.4826 * mad)
    accepted = abs_dev <= reject_band
    if int(np.count_nonzero(accepted)) >= max(8, int(0.60 * x.size)):
        x_reference = x[accepted]
    else:
        x_reference = x
    position = float(np.median(x_reference))

    q05, q95 = np.quantile(x, [0.05, 0.95])
    robust_span = float(q95 - q05)

    quarter = max(2, int(x.size // 4))
    first_position = float(np.median(x[:quarter]))
    final_position = float(np.median(x[-quarter:]))
    first_time = float(np.median(t[:quarter]))
    final_time = float(np.median(t[-quarter:]))
    elapsed = max(1e-6, final_time - first_time)
    net_change = final_position - first_position
    drift = net_change / elapsed

    high_noise = robust_span > float(noise_warning_span_turns)
    stationary = (
        abs(drift) <= float(max_drift_turns_s)
        and robust_span <= float(hard_span_turns)
    )
    return RobustPositionEstimate(
        position_turns=position,
        robust_span_turns=robust_span,
        drift_turns_s=float(drift),
        net_change_turns=float(net_change),
        sample_count=int(x.size),
        stationary=bool(stationary),
        high_noise=bool(high_noise),
    )


class PositionMotionEstimator:
    """Windowed position filter and drift estimator for the live display."""

    def __init__(
        self,
        sample_rate_hz: float,
        median_window: int,
        motion_window_s: float,
        deadband_turns_s: float,
    ) -> None:
        from collections import deque

        self.median_window = max(1, int(median_window))
        self.history_length = max(8, int(max(0.1, motion_window_s) * sample_rate_hz))
        self.deadband_turns_s = max(0.0, float(deadband_turns_s))
        self._raw = deque(maxlen=self.median_window)
        self._times = deque(maxlen=self.history_length)
        self._filtered = deque(maxlen=self.history_length)
        self._last_unwrapped: float | None = None

    def reset(self) -> None:
        self._raw.clear()
        self._times.clear()
        self._filtered.clear()
        self._last_unwrapped = None

    def update(self, timestamp_s: float, raw_turns: float) -> tuple[float, float, bool]:
        value = float(raw_turns)
        if self._last_unwrapped is not None:
            while value - self._last_unwrapped > 0.5:
                value -= 1.0
            while value - self._last_unwrapped < -0.5:
                value += 1.0
        self._last_unwrapped = value
        self._raw.append(value)
        filtered = float(median(self._raw))
        self._times.append(float(timestamp_s))
        self._filtered.append(filtered)

        if len(self._filtered) < 6:
            return filtered, 0.0, True
        count = len(self._filtered)
        quarter = max(2, count // 4)
        first = float(median(list(self._filtered)[:quarter]))
        last = float(median(list(self._filtered)[-quarter:]))
        first_t = float(median(list(self._times)[:quarter]))
        last_t = float(median(list(self._times)[-quarter:]))
        rate = (last - first) / max(1e-6, last_t - first_t)
        if abs(rate) <= self.deadband_turns_s:
            return filtered, 0.0, True
        return filtered, float(rate), False
