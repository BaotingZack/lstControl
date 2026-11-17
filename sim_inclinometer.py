"""Simulated hook sway and inclinometer readings for browser replay."""

from __future__ import annotations

import math
from typing import Any

from site_config import build_sway_axis_mapping, load_site_config
from anti_sway.estimator import InclinometerSwayEstimator

GRAVITY = 9.81


def _rope_length(z: float, *, sheave_height: float, grab_offset: float) -> float:
    return max(0.5, sheave_height - z - grab_offset)


def simulate_sway_history(
    history: list[dict],
    *,
    sheave_height: float = 4.0,
    grab_offset: float = 0.0,
    damping_ratio: float = 0.015,
) -> list[dict[str, float]]:
    """Damped pendulum excited by horizontal acceleration (small-angle)."""
    if not history:
        return []

    theta_x = 0.0
    theta_y = 0.0
    omega_x = 0.0
    omega_y = 0.0
    prev_vx = float(history[0].get('vx', 0.0) or 0.0)
    prev_vy = float(history[0].get('vy', 0.0) or 0.0)
    prev_t = float(history[0]['t'])
    out: list[dict[str, float]] = []

    for i, item in enumerate(history):
        t = float(item['t'])
        dt = max(1e-4, t - prev_t) if i > 0 else 0.01
        prev_t = t
        z = float(item['z'])
        L = _rope_length(z, sheave_height=sheave_height, grab_offset=grab_offset)
        omega_n = math.sqrt(GRAVITY / L)
        vx = float(item.get('vx', 0.0) or 0.0)
        vy = float(item.get('vy', 0.0) or 0.0)
        ax = (vx - prev_vx) / dt if i > 0 else 0.0
        ay = (vy - prev_vy) / dt if i > 0 else 0.0
        prev_vx, prev_vy = vx, vy

        alpha_x = -(GRAVITY / L) * theta_x - ax / L
        alpha_y = -(GRAVITY / L) * theta_y - ay / L
        omega_x += alpha_x * dt
        omega_y += alpha_y * dt
        damp = max(0.0, 1.0 - 2.0 * damping_ratio * omega_n * dt)
        omega_x *= damp
        omega_y *= damp
        theta_x += omega_x * dt
        theta_y += omega_y * dt
        out.append({'theta_x': theta_x, 'theta_y': theta_y})

    return out


def _theta_to_raw_deg(
    theta_x: float,
    theta_y: float,
    estimator: InclinometerSwayEstimator,
) -> tuple[float, float]:
    """Invert axis mapping (open-gripper bias) to fake j1939 roll/pitch degrees."""
    scale = estimator.angle_scale if estimator.angle_scale else 1.0
    mx = estimator.mapping.theta_x
    my = estimator.mapping.theta_y
    roll = float(estimator.roll_bias)
    pitch = float(estimator.pitch_bias)
    if mx.angle_source == 'pitch':
        pitch = theta_x / (mx.angle_sign * scale) + estimator.pitch_bias
    elif mx.angle_source == 'roll':
        roll = theta_x / (mx.angle_sign * scale) + estimator.roll_bias
    if my.angle_source == 'roll':
        roll = theta_y / (my.angle_sign * scale) + estimator.roll_bias
    elif my.angle_source == 'pitch':
        pitch = theta_y / (my.angle_sign * scale) + estimator.pitch_bias
    return roll, pitch


def build_sim_inclinometer_estimator(site: dict[str, Any] | None = None) -> InclinometerSwayEstimator:
    site = site if site is not None else load_site_config()
    mapping = build_sway_axis_mapping(site)
    angle_deg = bool(site.get('anti_sway_angle_deg', True))
    scale = math.pi / 180.0 if angle_deg else 1.0
    rb_closed = site.get('anti_sway_roll_bias_closed')
    pb_closed = site.get('anti_sway_pitch_bias_closed')
    return InclinometerSwayEstimator(
        mapping=mapping,
        angle_scale=scale,
        roll_bias=float(site.get('anti_sway_roll_bias', 0.0)),
        pitch_bias=float(site.get('anti_sway_pitch_bias', 0.0)),
        roll_bias_closed=float(rb_closed) if rb_closed is not None else None,
        pitch_bias_closed=float(pb_closed) if pb_closed is not None else None,
    )


def inclinometer_fields_for_sway(
    theta_x: float,
    theta_y: float,
    estimator: InclinometerSwayEstimator,
) -> dict[str, float]:
    roll, pitch = _theta_to_raw_deg(theta_x, theta_y, estimator)
    tx_deg = math.degrees(theta_x)
    ty_deg = math.degrees(theta_y)
    return {
        'incl_roll': roll,
        'incl_pitch': pitch,
        'theta_x_deg': tx_deg,
        'theta_y_deg': ty_deg,
        'theta_peak_deg': max(abs(tx_deg), abs(ty_deg)),
    }


def default_sway_tol_deg(site: dict[str, Any] | None = None) -> float:
    site = site if site is not None else load_site_config()
    return float(site.get('hook_sway_angle_tol_deg', 0.05))
