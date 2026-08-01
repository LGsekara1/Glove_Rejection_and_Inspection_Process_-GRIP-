"""Swappable per-axis velocity filters with persistent state."""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from collections import deque
from statistics import median

import numpy as np

from .models import VelocityFilterConfig


class VelocityFilter(ABC):
    @abstractmethod
    def reset(self) -> None: ...

    @abstractmethod
    def update(self, value: float, dt: float) -> float: ...


class PassthroughFilter(VelocityFilter):
    def reset(self) -> None:
        return

    def update(self, value: float, dt: float) -> float:
        return float(value)


class MovingAverageFilter(VelocityFilter):
    def __init__(self, window: int) -> None:
        self.values: deque[float] = deque(maxlen=max(1, int(window)))

    def reset(self) -> None:
        self.values.clear()

    def update(self, value: float, dt: float) -> float:
        self.values.append(float(value))
        return sum(self.values) / len(self.values)


class SinglePoleLowPassFilter(VelocityFilter):
    def __init__(self, cutoff_hz: float) -> None:
        self.cutoff_hz = max(1e-6, float(cutoff_hz))
        self.y: float | None = None

    def reset(self) -> None:
        self.y = None

    def update(self, value: float, dt: float) -> float:
        value = float(value)
        if self.y is None:
            self.y = value
            return value
        dt = max(1e-6, float(dt))
        tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        alpha = dt / (dt + tau)
        self.y = alpha * value + (1.0 - alpha) * self.y
        return self.y


class MedianVelocityFilter(VelocityFilter):
    def __init__(self, window: int) -> None:
        self.values: deque[float] = deque(maxlen=max(1, int(window)))

    def reset(self) -> None:
        self.values.clear()

    def update(self, value: float, dt: float) -> float:
        self.values.append(float(value))
        return float(median(self.values))


class ButterworthVelocityFilter(VelocityFilter):
    def __init__(self, cutoff_hz: float, order: int, sample_rate_hz: float) -> None:
        try:
            from scipy.signal import butter, lfilter, lfilter_zi  # type: ignore
        except Exception as exc:  # pragma: no cover - dependency failure path
            raise RuntimeError("Butterworth filtering requires scipy.") from exc
        nyquist = max(1e-6, sample_rate_hz / 2.0)
        cutoff = min(max(1e-6, float(cutoff_hz)), nyquist * 0.95)
        self.b, self.a = butter(max(1, int(order)), cutoff, btype="low", fs=sample_rate_hz)
        self._lfilter = lfilter
        self._base_zi = lfilter_zi(self.b, self.a)
        self.zi: np.ndarray | None = None

    def reset(self) -> None:
        self.zi = None

    def update(self, value: float, dt: float) -> float:
        x = float(value)
        if self.zi is None:
            self.zi = self._base_zi * x
        y, self.zi = self._lfilter(self.b, self.a, np.array([x]), zi=self.zi)
        return float(y[0])


def make_velocity_filter(config: VelocityFilterConfig, sample_rate_hz: float) -> VelocityFilter:
    if config.type == "None":
        return PassthroughFilter()
    if config.type == "Moving Average":
        return MovingAverageFilter(config.window)
    if config.type == "Low-pass (1-pole)":
        return SinglePoleLowPassFilter(config.cutoff_hz)
    if config.type == "Butterworth":
        return ButterworthVelocityFilter(config.cutoff_hz, config.order, sample_rate_hz)
    if config.type == "Median":
        return MedianVelocityFilter(config.window)
    raise ValueError(f"Unknown velocity filter type: {config.type}")
