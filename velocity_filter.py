"""Velocity estimation from noisy position measurements."""

from __future__ import annotations

import math


class LowPassVelocityEstimator:
    """Differentiate position, then low-pass filter velocity."""

    def __init__(self, tau: float, initial_position: float, initial_velocity: float = 0.0):
        if tau <= 0:
            raise ValueError('tau must be positive')
        self.tau = tau
        self.last_position = initial_position
        self.filtered_velocity = initial_velocity

    def update(self, measured_position: float, dt: float) -> tuple[float, float]:
        if dt <= 0:
            raise ValueError('dt must be positive')
        raw_velocity = (measured_position - self.last_position) / dt
        alpha = 1.0 - math.exp(-dt / self.tau)
        self.filtered_velocity += alpha * (raw_velocity - self.filtered_velocity)
        self.last_position = measured_position
        return raw_velocity, self.filtered_velocity
