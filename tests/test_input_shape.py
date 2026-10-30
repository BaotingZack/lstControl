"""ZVD input shaping (open-loop PTP anti-sway)."""

from __future__ import annotations

import math

import pytest

from anti_sway.input_shape import MODE_ID, build_strategy
from anti_sway.ptp_config import apply_ptp_anti_sway_mode
from anti_sway.registry import OPEN_LOOP_ANTI_SWAY_MODES, build_strategy as reg_build
from anti_sway.shaping import ZvdAxisShaper, omega_from_rope_length, zvd_impulses, zvd_step_gain
from anti_sway.base import AntiSwayStepContext
from crane_model import CraneConfig, CraneState
from main import run_simulation


def test_zvd_impulses_sum_to_one():
    a0, a1, a2 = zvd_impulses(0.0)
    assert a0 + a1 + a2 == pytest.approx(1.0)
    assert a0 == pytest.approx(0.25)
    assert a1 == pytest.approx(0.5)
    assert a2 == pytest.approx(0.25)


def test_zvd_shaper_reproduces_step_with_delay():
    dt = 0.1
    omega = omega_from_rope_length(4.0)
    sh = ZvdAxisShaper(dt)
    sh.configure(omega_n=omega, zeta=0.0)
    outs = [sh.shape(0.0) for _ in range(3)]
    outs.append(sh.shape(1.0))
    assert outs[3] == pytest.approx(0.25)
    for _ in range(50):
        outs.append(sh.shape(1.0))
    assert outs[-1] == pytest.approx(1.0)


def test_zvd_step_gain_reaches_unity():
    omega = omega_from_rope_length(4.0)
    Td = math.pi / omega
    assert zvd_step_gain(0.0, omega_n=omega, zeta=0.0) == pytest.approx(0.0)
    g_mid = zvd_step_gain(Td, omega_n=omega, zeta=0.0)
    assert g_mid == pytest.approx(0.75)
    assert zvd_step_gain(3 * Td, omega_n=omega, zeta=0.0) == pytest.approx(1.0)


def test_input_shape_registered_open_loop():
    assert MODE_ID in OPEN_LOOP_ANTI_SWAY_MODES
    cfg = apply_ptp_anti_sway_mode(CraneConfig(), MODE_ID, {})
    assert cfg.anti_sway_mode == MODE_ID
    strat = reg_build(cfg)
    assert strat is not None
    assert strat.requires_sway_feedback is False


def test_input_shape_strategy_shapes_velocity():
    cfg = apply_ptp_anti_sway_mode(
        CraneConfig(),
        MODE_ID,
        {'anti_sway_input_shape_zeta': 0.01},
    )
    strat = build_strategy(cfg)
    ctx = AntiSwayStepContext(
        theta_x=0.0,
        theta_y=0.0,
        dt=0.1,
        z_grab=2.0,
        dist_xy=5.0,
        locked_x=False,
        locked_y=False,
        config=cfg,
    )
    corr = strat.correct_xy(0.2, 0.0, ctx)
    assert corr.vx_cmd != pytest.approx(0.2) or corr.dvx != 0.0


def test_sim_run_input_shape_without_sway_sensor():
    config = CraneConfig()
    target = (4.0, 3.0, 2.0)
    initial = CraneState(x0=0.0, y0=0.0, z0=0.0)
    history, _ = run_simulation(
        target_pos=target,
        initial_state=initial,
        config=config,
        verbose=False,
        anti_sway_mode=MODE_ID,
    )
    peak = max(
        abs(s.get('antisway_dvx', 0.0)) + abs(s.get('antisway_dvy', 0.0))
        for s in history
    )
    assert peak > 0.001
    assert history[-1]['anti_sway_mode'] == MODE_ID
