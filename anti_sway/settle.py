"""Rope length helpers shared by anti-sway control and simulation."""

from __future__ import annotations

GRAVITY = 9.81
HOIST_SHEAVE_HEIGHT = 4.0


def effective_rope_length(
    z_grab: float,
    *,
    sheave_height: float = HOIST_SHEAVE_HEIGHT,
    grab_offset: float = 0.0,
    cable_stretch: float = 0.0,
    min_length: float = 0.5,
) -> float:
    """Effective pendulum length L_eff = sheave_height - z + grab_offset + stretch."""
    L = sheave_height - z_grab + grab_offset + cable_stretch
    return max(L, min_length)
