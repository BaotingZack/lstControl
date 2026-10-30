"""PTP anti-sway toggle and Δv=K·θ wiring."""

from __future__ import annotations

from dataclasses import replace

import pytest

from crane_model import CraneConfig, CraneState
from main import run_simulation, _query_bool
from ros_bridge import SwaySensorSource
from anti_sway.gantry import GantryAntiSwayController, build_gantry_anti_sway
from anti_sway.incl import build_anti_sway
from anti_sway.ptp_config import apply_ptp_anti_sway_mode, normalize_anti_sway_mode
from anti_sway.sim_feedback import SimSwayFeedback


def test_query_bool_anti_sway_param():
    assert _query_bool({'anti_sway': ['1']}, 'anti_sway') is True
    assert _query_bool({'anti_sway': ['0']}, 'anti_sway') is False
    assert _query_bool({}, 'anti_sway', default=False) is False


def test_build_anti_sway_requires_enable_and_gain():
    cfg = CraneConfig(enable_anti_sway=False, anti_sway_sway_gain=2.0)
    assert build_anti_sway(cfg) is None
    cfg2 = replace(cfg, enable_anti_sway=True, anti_sway_mode='incl')
    ctrl = build_anti_sway(cfg2)
    assert ctrl is not None
    dvx, dvy, _ = ctrl.compute(0.05, -0.03, z_grab=3.0)
    assert dvx > 0
    assert dvy < 0


def test_sim_run_records_antisway_when_incl_enabled():
    config = CraneConfig()
    target = (4.0, 3.0, 2.0)
    initial = CraneState(x0=0.0, y0=0.0, z0=0.0)
    history_plain, _ = run_simulation(
        target_pos=target,
        initial_state=initial,
        config=config,
        verbose=False,
        enable_anti_sway=False,
    )
    history_as, _ = run_simulation(
        target_pos=target,
        initial_state=initial,
        config=config,
        verbose=False,
        enable_anti_sway=True,
        anti_sway_mode='incl',
        site={'anti_sway_sway_gain': 2.6, 'anti_sway_approach_pos': 2.0},
    )
    peak_plain = max(
        (abs(s.get('antisway_dvx', 0.0)) + abs(s.get('antisway_dvy', 0.0)) for s in history_plain),
        default=0.0,
    )
    peak_as = max(
        (abs(s.get('antisway_dvx', 0.0)) + abs(s.get('antisway_dvy', 0.0)) for s in history_as),
        default=0.0,
    )
    assert peak_plain == 0.0
    assert peak_as > 0.01


def test_sim_sway_feedback_pendulum_moves_with_accel():
    fb = SimSwayFeedback(sheave_height=4.0, grab_offset=0.0)
    fb.reset()
    for _ in range(50):
        fb.advance(0.1, vx=0.4, vy=0.0, z=2.0)
    sway = fb.get_sway()
    assert abs(sway['theta_x']) > 0.001


def test_sway_sensor_source_delegates(monkeypatch):
    monkeypatch.setattr(
        'ros_bridge.get_sway_state',
        lambda: {'theta_x': 0.01, 'theta_y': -0.02},
    )
    src = SwaySensorSource()
    got = src.get_sway()
    assert got['theta_x'] == pytest.approx(0.01)


def test_normalize_mode_legacy():
    assert normalize_anti_sway_mode(None) == 'off'
    assert normalize_anti_sway_mode('gantry') == 'gantry'
    assert normalize_anti_sway_mode(None, legacy_enabled=True) == 'incl'


def test_gantry_subtracts_from_velocity_command():
    ctrl = GantryAntiSwayController(kp=2.0, kd=0.0, max_correction=1.0, deadband_rad=0.0)
    u_x, _u_y = ctrl.compute(0.05, 0.0, 0.1)
    assert u_x > 0
    assert 0.2 - u_x < 0.2


def test_apply_ptp_gantry_mode_merges_site_fields():
    cfg = apply_ptp_anti_sway_mode(
        CraneConfig(),
        'gantry',
        {'anti_sway_gantry_kp': 1.1, 'anti_sway_gantry_kd': 0.2},
    )
    assert cfg.enable_anti_sway is True
    assert cfg.anti_sway_mode == 'gantry'
    assert cfg.anti_sway_gantry_kp == pytest.approx(1.1)


def test_build_gantry_requires_mode():
    cfg = CraneConfig(enable_anti_sway=True, anti_sway_mode='incl')
    assert build_gantry_anti_sway(cfg) is None
    cfg2 = replace(cfg, anti_sway_mode='gantry')
    assert build_gantry_anti_sway(cfg2) is not None
