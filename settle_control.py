"""Hold-position settle loop (消摆测试) — wraps run_pd_control profile=settle."""

from __future__ import annotations

import math
from typing import Any

from anti_sway.ptp_config import anti_sway_crane_fields, algorithm_fields_from_site
from crane_model import CraneConfig, CraneState, run_pd_control


def apply_settle_anti_sway_mode(
    config: CraneConfig,
    mode: str | None,
    site: dict[str, Any] | None = None,
) -> CraneConfig:
    from dataclasses import replace

    from anti_sway.ptp_config import normalize_anti_sway_mode

    resolved = normalize_anti_sway_mode(mode, legacy_enabled=False)
    if resolved == 'off':
        return replace(config, enable_anti_sway=False, anti_sway_mode='off')
    fields: dict[str, Any] = {
        'enable_anti_sway': True,
        'anti_sway_mode': resolved,
        'anti_sway_travel_damping': True,
        'anti_sway_approach_damping': True,
        **anti_sway_crane_fields(site),
        **algorithm_fields_from_site(resolved, site),
    }
    return replace(config, **fields)


def run_settle_control(
    *,
    source,
    actuator,
    config: CraneConfig,
    anchor_pos: tuple[float, float, float],
    initial_state: CraneState,
    hooks=None,
    sway_source=None,
    is_simulation: bool = False,
    max_time: float | None = None,
    verbose: bool = True,
    initial_theta: tuple[float, float] | None = None,
    settle_theta_tol_rad: float | None = None,
    settle_hold_sec: float | None = 3.5,
    settle_hold_cycles: int | None = None,
) -> tuple[list[dict], list[tuple[float, str]]]:
    if initial_theta and sway_source is not None and hasattr(sway_source, 'theta_x'):
        sway_source.theta_x = float(initial_theta[0])
        sway_source.theta_y = float(initial_theta[1])
        if hasattr(sway_source, '_omega_x'):
            sway_source._omega_x = sway_source._omega_y = 0.0

    history, events = run_pd_control(
        source=source,
        actuator=actuator,
        config=config,
        target_pos=anchor_pos,
        initial_state=initial_state,
        hooks=hooks,
        max_time=max_time,
        verbose=verbose,
        is_simulation=is_simulation,
        sway_source=sway_source,
        profile='settle',
        settle_theta_tol_rad=settle_theta_tol_rad,
        settle_hold_sec=settle_hold_sec,
        settle_hold_cycles=settle_hold_cycles,
    )
    if verbose and history:
        final = history[-1]
        print(
            f'[消摆] 结束 t={final["t"]:.2f}s '
            f'θ=({final.get("theta_x")}, {final.get("theta_y")})'
        )
    return history, events
