"""Open-loop ZVD input shaping on velocity commands."""

from __future__ import annotations

import math
from collections import deque

from anti_sway.settle import GRAVITY


def zvd_impulses(zeta: float) -> tuple[float, float, float]:
    """Three ZVD amplitudes (sum to 1) for pendulum mode with damping ratio zeta."""
    zeta = max(0.0, min(float(zeta), 0.99))
    K = math.exp(-zeta * math.pi / math.sqrt(max(1e-9, 1.0 - zeta * zeta)))
    denom = 1.0 + 2.0 * K + K * K
    return 1.0 / denom, 2.0 * K / denom, K * K / denom


class ZvdAxisShaper:
    """Discrete ZVD: y[n] = Σ Aᵢ·u[n − delayᵢ]."""

    def __init__(self, dt: float, *, max_history: int = 5000):
        if dt <= 0:
            raise ValueError('dt must be positive')
        self.dt = dt
        self._max_history = max_history
        self._history: deque[float] = deque(maxlen=max_history)
        self._delays: tuple[int, int, int] = (0, 1, 2)
        self._amps: tuple[float, float, float] = (0.25, 0.5, 0.25)

    def configure(self, *, omega_n: float, zeta: float = 0.01) -> None:
        omega_n = max(float(omega_n), 1e-3)
        Td = math.pi / omega_n
        a0, a1, a2 = zvd_impulses(zeta)
        d0 = 0
        d1 = max(1, int(round(Td / self.dt)))
        d2 = max(d1 + 1, int(round(2.0 * Td / self.dt)))
        self._amps = (a0, a1, a2)
        self._delays = (d0, d1, d2)
        need = d2 + 1
        if need > self._max_history:
            raise ValueError('ZVD delays exceed max_history; increase dt or max_history')

    def reset(self) -> None:
        self._history.clear()

    def shape(self, u: float) -> float:
        self._history.append(float(u))
        idx = len(self._history) - 1
        y = 0.0
        for delay, amp in zip(self._delays, self._amps):
            j = idx - delay
            if j >= 0:
                y += amp * self._history[j]
        return y


def omega_from_rope_length(L: float, g: float = GRAVITY) -> float:
    return math.sqrt(g / max(float(L), 1e-6))


def zvd_step_gain(t: float, *, omega_n: float, zeta: float = 0.01) -> float:
    """Cumulative ZVD response to a unit step (0 → 1 without residual vibration)."""
    if t <= 0:
        return 0.0
    Td = math.pi / max(float(omega_n), 1e-3)
    a0, a1, a2 = zvd_impulses(zeta)
    gain = 0.0
    if t >= 0:
        gain += a0
    if t >= Td:
        gain += a1
    if t >= 2.0 * Td:
        gain += a2
    return min(1.0, max(0.0, gain))
