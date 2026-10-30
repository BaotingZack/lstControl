"""Backward-compatible re-exports — prefer ``anti_sway.incl``."""

from anti_sway.incl import AntiSwayController, build_anti_sway
from anti_sway.rope import (
    RopeLengthModel,
    damping_ratio,
    envelope_decay_time,
    gain_for_damping_ratio,
)

__all__ = [
    'AntiSwayController',
    'RopeLengthModel',
    'build_anti_sway',
    'damping_ratio',
    'envelope_decay_time',
    'gain_for_damping_ratio',
]
