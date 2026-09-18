"""Load site-specific settings (inclinometer calibration, anti-sway) from JSON."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from sway_estimator import AxisMapping, SwayAxisMapping

DEFAULT_SITE_CONFIG_PATH = Path(__file__).resolve().parent / 'site_config.json'

# Keys applied to CraneConfig; keys starting with '_' are metadata only.
SITE_CONFIG_FIELDS = frozenset({
    'enable_anti_sway',
    'anti_sway_travel_damping',
    'anti_sway_approach_damping',
    'anti_sway_approach_pos',
    'anti_sway_approach_vel',
    'anti_sway_sway_gain',
    'anti_sway_angle_deg',
    'anti_sway_roll_bias',
    'anti_sway_pitch_bias',
    'anti_sway_roll_bias_closed',
    'anti_sway_pitch_bias_closed',
    'anti_sway_max_correction',
    'anti_sway_settle_damping',
    'anti_sway_settle_max_shift',
    'anti_sway_theta_x_from',
    'anti_sway_theta_y_from',
    'anti_sway_theta_x_sign',
    'anti_sway_theta_y_sign',
    'hook_sway_angle_tol_deg',
    'hook_sway_angle_tol_approach_deg',
    'anti_sway_locked_max_shift',
    'anti_sway_hold_recenter_gain',
    'anti_sway_settle_gain_scale',
    'hook_settle_window_periods',
    'hook_settle_window_max',
    'hook_settle_max_wait',
    'anti_sway_settle_dc_tau',
    'anti_sway_gain_schedule',
    'rope_length_sheave_height',
    'rope_length_grab_offset',
    'rope_length_structure',
    'rope_length_cable_stretch',
    'rope_length_min',
    'stabilize_delay',
    'gripper_settle_max_wait',
})

_VALID_SWAY_SOURCES = frozenset({'roll', 'pitch'})


def load_site_config(path: str | Path | None = None) -> dict[str, Any]:
    """Load site config JSON. Returns {} if path is missing."""
    cfg_path = Path(path) if path is not None else DEFAULT_SITE_CONFIG_PATH
    if not cfg_path.is_file():
        return {}
    with cfg_path.open(encoding='utf-8') as fh:
        raw = json.load(fh)
    if not isinstance(raw, dict):
        raise ValueError(f'{cfg_path} must contain a JSON object')
    return {
        key: value
        for key, value in raw.items()
        if not key.startswith('_') and key in SITE_CONFIG_FIELDS
    }


def resolve_anti_sway_settings(
    site: dict[str, Any],
    *,
    cli_enable: bool,
    cli_enable_explicit: bool,
    cli_gain: float | None,
    cli_angle_deg: bool,
    cli_angle_deg_explicit: bool,
    cli_roll_bias: float | None,
    cli_pitch_bias: float | None,
) -> dict[str, Any]:
    """Merge site JSON with CLI; explicit CLI flags override file values."""
    enable = bool(site.get('enable_anti_sway', False))
    if cli_enable_explicit:
        enable = cli_enable

    if cli_gain is not None:
        gain = cli_gain
    elif enable:
        gain = float(site.get('anti_sway_sway_gain', 0.5))
    else:
        gain = 0.0

    angle_deg = bool(site.get('anti_sway_angle_deg', False))
    if cli_angle_deg_explicit:
        angle_deg = cli_angle_deg

    out: dict[str, Any] = {
        'enable_anti_sway': enable,
        'anti_sway_sway_gain': gain,
        'anti_sway_angle_deg': angle_deg,
        'anti_sway_roll_bias': (
            cli_roll_bias
            if cli_roll_bias is not None
            else float(site.get('anti_sway_roll_bias', 0.0))
        ),
        'anti_sway_pitch_bias': (
            cli_pitch_bias
            if cli_pitch_bias is not None
            else float(site.get('anti_sway_pitch_bias', 0.0))
        ),
    }
    for key in SITE_CONFIG_FIELDS:
        if key in out or key not in site:
            continue
        out[key] = site[key]
    return out


def build_sway_axis_mapping(site: dict[str, Any] | None = None) -> SwayAxisMapping:
    """Build θx/θy mapping from site_config.

    现场标定 (2026-09-16): 倾角仪 Y→大车 X (θx), 倾角仪 X→小车 Y (θy).
    j1939 vector.y=pitch (Inc-Y), vector.x=roll (Inc-X).
    """
    site = site or {}
    theta_x_from = str(site.get('anti_sway_theta_x_from', 'pitch'))
    theta_y_from = str(site.get('anti_sway_theta_y_from', 'roll'))
    if theta_x_from not in _VALID_SWAY_SOURCES:
        raise ValueError(
            f'anti_sway_theta_x_from must be roll or pitch, got {theta_x_from!r}'
        )
    if theta_y_from not in _VALID_SWAY_SOURCES:
        raise ValueError(
            f'anti_sway_theta_y_from must be roll or pitch, got {theta_y_from!r}'
        )
    return SwayAxisMapping(
        theta_x=AxisMapping(theta_x_from, float(site.get('anti_sway_theta_x_sign', 1.0))),
        theta_y=AxisMapping(theta_y_from, float(site.get('anti_sway_theta_y_sign', 1.0))),
    )
