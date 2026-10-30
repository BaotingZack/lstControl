"""PTP mode selection and CraneConfig field merge from site_config.json."""

from __future__ import annotations

from typing import Any

from anti_sway.registry import ACTIVE_ALGORITHM_IDS, MODE_REGISTRY

# off + registered algorithms
ANTI_SWAY_MODES = ('off',) + ACTIVE_ALGORITHM_IDS


def normalize_anti_sway_mode(mode: str | None, *, legacy_enabled: bool = False) -> str:
    if mode in ANTI_SWAY_MODES:
        return str(mode)
    if legacy_enabled:
        return 'incl'
    return 'off'


def anti_sway_crane_fields(site: dict[str, Any] | None = None) -> dict[str, Any]:
    """Shared rope geometry + incl-style defaults (used by all algorithms)."""
    site = site or {}
    schedule = site.get('anti_sway_gain_schedule')
    if schedule is not None and not isinstance(schedule, (list, tuple)):
        schedule = ()
    return {
        'anti_sway_travel_damping': bool(site.get('anti_sway_travel_damping', False)),
        'anti_sway_approach_damping': bool(site.get('anti_sway_approach_damping', True)),
        'anti_sway_approach_pos': float(site.get('anti_sway_approach_pos', 0.40)),
        'anti_sway_sway_gain': float(site.get('anti_sway_sway_gain', 2.6)),
        'anti_sway_max_correction': float(site.get('anti_sway_max_correction', 0.2)),
        'rope_length_sheave_height': float(site.get('rope_length_sheave_height', 4.0)),
        'rope_length_grab_offset': float(site.get('rope_length_grab_offset', 0.0)),
        'rope_length_cable_stretch': float(site.get('rope_length_cable_stretch', 0.0)),
        'rope_length_min': float(site.get('rope_length_min', 0.5)),
        'anti_sway_gain_schedule': tuple(schedule or ()),
    }


def algorithm_fields_from_site(mode: str, site: dict[str, Any] | None) -> dict[str, Any]:
    mod = MODE_REGISTRY.get(mode)
    if mod is None:
        return {}
    return mod.config_fields_from_site(site)


def apply_ptp_anti_sway_mode(
    config,
    mode: str | None,
    site: dict[str, Any] | None = None,
    *,
    legacy_enabled: bool = False,
):
    from dataclasses import replace

    resolved = normalize_anti_sway_mode(mode, legacy_enabled=legacy_enabled)
    if resolved == 'off':
        return replace(config, enable_anti_sway=False, anti_sway_mode='off')
    fields: dict[str, Any] = {
        'enable_anti_sway': True,
        'anti_sway_mode': resolved,
        **anti_sway_crane_fields(site),
        **algorithm_fields_from_site(resolved, site),
    }
    return replace(config, **fields)


def anti_sway_gantry_fields(site: dict[str, Any] | None = None) -> dict[str, Any]:
    """Backward-compatible alias."""
    return algorithm_fields_from_site('gantry', site)
