"""Effective rope length model and gain tuning helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass

from anti_sway.settle import GRAVITY, effective_rope_length


@dataclass(frozen=True)
class RopeLengthModel:
    sheave_height: float
    grab_offset: float = 0.0
    cable_stretch: float = 0.0
    min_length: float = 0.5

    def compute(self, z_grab: float) -> float:
        return effective_rope_length(
            z_grab,
            sheave_height=self.sheave_height,
            grab_offset=self.grab_offset,
            cable_stretch=self.cable_stretch,
            min_length=self.min_length,
        )


def damping_ratio(sway_gain: float, rope_length: float, g: float = GRAVITY) -> float:
    denom = 2.0 * math.sqrt(g * max(rope_length, 1e-6))
    return sway_gain / denom


def gain_for_damping_ratio(
    zeta: float, rope_length: float, g: float = GRAVITY
) -> float:
    return 2.0 * zeta * math.sqrt(g * max(rope_length, 1e-6))


def envelope_decay_time(
    sway_gain: float,
    rope_length: float,
    theta_from: float,
    theta_to: float,
    g: float = GRAVITY,
) -> float:
    if sway_gain <= 0.0 or theta_to <= 0.0 or theta_from <= theta_to:
        return 0.0
    tau_env = 2.0 * max(rope_length, 1e-6) / sway_gain
    return tau_env * math.log(theta_from / theta_to)
