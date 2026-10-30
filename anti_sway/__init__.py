"""Pluggable hook-sway algorithms for crane PD control.

Add a new algorithm:
  1. Create ``anti_sway/my_algo.py`` with MODE_ID, MODE_LABEL, MODE_DESCRIPTION,
     ``config_fields_from_site``, ``build_strategy``.
  2. Register the module in ``anti_sway/registry.py`` ``_ALGORITHM_MODULES``.
  3. Add site_config.json keys and optional PTP UI entry (or use dynamic list).
"""

from anti_sway.base import AntiSwayStepContext, AntiSwayStrategy, VelocityCorrection
from anti_sway.estimator import (
    AxisMapping,
    InclinometerSwayEstimator,
    SwayAxisMapping,
)
from anti_sway.incl import AntiSwayController, build_anti_sway
from anti_sway.gantry import GantryAntiSwayController, build_gantry_anti_sway
from anti_sway.ptp_config import (
    ANTI_SWAY_MODES,
    anti_sway_crane_fields,
    anti_sway_gantry_fields,
    apply_ptp_anti_sway_mode,
    normalize_anti_sway_mode,
)
from anti_sway.registry import ACTIVE_ALGORITHM_IDS, build_strategy, list_algorithm_options
from anti_sway.rope import (
    RopeLengthModel,
    damping_ratio,
    envelope_decay_time,
    gain_for_damping_ratio,
)
from anti_sway.settle import GRAVITY, HOIST_SHEAVE_HEIGHT, effective_rope_length
from anti_sway.sim_feedback import SimSwayFeedback

__all__ = [
    'ACTIVE_ALGORITHM_IDS',
    'ANTI_SWAY_MODES',
    'AntiSwayController',
    'AntiSwayStepContext',
    'AntiSwayStrategy',
    'AxisMapping',
    'GantryAntiSwayController',
    'GRAVITY',
    'HOIST_SHEAVE_HEIGHT',
    'InclinometerSwayEstimator',
    'RopeLengthModel',
    'SimSwayFeedback',
    'SwayAxisMapping',
    'VelocityCorrection',
    'anti_sway_crane_fields',
    'anti_sway_gantry_fields',
    'apply_ptp_anti_sway_mode',
    'build_anti_sway',
    'build_gantry_anti_sway',
    'build_strategy',
    'damping_ratio',
    'effective_rope_length',
    'envelope_decay_time',
    'gain_for_damping_ratio',
    'list_algorithm_options',
    'normalize_anti_sway_mode',
]
