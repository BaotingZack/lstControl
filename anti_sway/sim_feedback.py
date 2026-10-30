"""Simulated pendulum sway for closed-loop anti-sway in replay/sim PD runs."""

from __future__ import annotations

import math

from anti_sway.settle import GRAVITY, effective_rope_length


class SimSwayFeedback:
    """Small-angle pendulum excited by trolley/bridge acceleration."""

    def __init__(
        self,
        *,
        sheave_height: float = 4.0,
        grab_offset: float = 0.0,
        damping_ratio: float = 0.015,
    ):
        self.sheave_height = sheave_height
        self.grab_offset = grab_offset
        self.damping_ratio = damping_ratio
        self.theta_x = 0.0
        self.theta_y = 0.0
        self._omega_x = 0.0
        self._omega_y = 0.0
        self._prev_vx = 0.0
        self._prev_vy = 0.0

    def reset(self) -> None:
        self.theta_x = self.theta_y = 0.0
        self._omega_x = self._omega_y = 0.0
        self._prev_vx = self._prev_vy = 0.0

    def advance(self, dt: float, vx: float, vy: float, z: float) -> None:
        dt = max(1e-4, dt)
        L = effective_rope_length(
            z,
            sheave_height=self.sheave_height,
            grab_offset=self.grab_offset,
        )
        omega_n = math.sqrt(GRAVITY / L)
        ax = (vx - self._prev_vx) / dt
        ay = (vy - self._prev_vy) / dt
        self._prev_vx, self._prev_vy = vx, vy

        alpha_x = -(GRAVITY / L) * self.theta_x - ax / L
        alpha_y = -(GRAVITY / L) * self.theta_y - ay / L
        self._omega_x += alpha_x * dt
        self._omega_y += alpha_y * dt
        damp = max(0.0, 1.0 - 2.0 * self.damping_ratio * omega_n * dt)
        self._omega_x *= damp
        self._omega_y *= damp
        self.theta_x += self._omega_x * dt
        self.theta_y += self._omega_y * dt

    def get_sway(self) -> dict[str, float]:
        return {'theta_x': self.theta_x, 'theta_y': self.theta_y}
