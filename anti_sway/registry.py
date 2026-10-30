"""Register anti-sway algorithms and build runtime strategies."""

from __future__ import annotations

from typing import Any

from anti_sway import (
    baseline,
    crane_remote,
    gantry,
    gantry_settle,
    incl,
    input_shape,
    overhead,
    overhead_settle,
)
from anti_sway.base import AntiSwayStrategy

# Each module: MODE_ID, MODE_LABEL, build_strategy, config_fields_from_site
_ALGORITHM_MODULES = (
    baseline,
    crane_remote,
    gantry_settle,
    overhead_settle,
    incl,
    input_shape,
    gantry,
    overhead,
)
PTP_ALGORITHM_MODULES = (incl, gantry, overhead, input_shape)
SETTLE_ALGORITHM_MODULES = (crane_remote, gantry_settle, overhead_settle, baseline)

OPEN_LOOP_ANTI_SWAY_MODES = frozenset({input_shape.MODE_ID})

MODE_REGISTRY: dict[str, Any] = {m.MODE_ID: m for m in _ALGORITHM_MODULES}

ACTIVE_ALGORITHM_IDS = tuple(MODE_REGISTRY.keys())
PTP_ALGORITHM_IDS = tuple(m.MODE_ID for m in PTP_ALGORITHM_MODULES)
SETTLE_ALGORITHM_IDS = tuple(m.MODE_ID for m in SETTLE_ALGORITHM_MODULES)


def build_strategy(config) -> AntiSwayStrategy | None:
    """Instantiate the strategy for config.anti_sway_mode, or None if off/disabled."""
    if not getattr(config, 'enable_anti_sway', False):
        return None
    mode = getattr(config, 'anti_sway_mode', 'off')
    if mode == 'off':
        return None
    mod = MODE_REGISTRY.get(mode)
    if mod is None:
        return None
    return mod.build_strategy(config)


def list_algorithm_options(*, scope: str = 'ptp') -> list[dict[str, str]]:
    """UI / API: selectable algorithms (excluding off).

    scope: ``ptp`` — 点到点运输防摇; ``settle`` — 消摆测试.
    """
    if scope == 'settle':
        modules = SETTLE_ALGORITHM_MODULES
    elif scope == 'all':
        modules = _ALGORITHM_MODULES
    else:
        modules = PTP_ALGORITHM_MODULES
    return [
        {
            'id': mod.MODE_ID,
            'label': mod.MODE_LABEL,
            'description': mod.MODE_DESCRIPTION,
        }
        for mod in modules
    ]
