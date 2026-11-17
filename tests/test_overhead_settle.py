"""Overhead LQR settle damping (OverheadCraneSwayControl reference)."""

from __future__ import annotations

import math

from anti_sway.overhead import OverheadLqrController
from anti_sway.overhead_settle import MODE_ID, build_strategy
from anti_sway.registry import list_algorithm_options
from anti_sway.rope import RopeLengthModel
from anti_sway.sim_feedback import SimSwayFeedback
from crane_model import CraneConfig, CranePlant, CraneState, PlantActuator, SimPositionSource
from settle_control import apply_settle_anti_sway_mode, run_settle_control


def test_overhead_settle_in_registry():
    settle_ids = [o['id'] for o in list_algorithm_options(scope='settle')]
    assert MODE_ID in settle_ids
    assert 'overhead' not in settle_ids
    ptp_ids = [o['id'] for o in list_algorithm_options(scope='ptp')]
    assert MODE_ID not in ptp_ids
    assert 'overhead' in ptp_ids


def test_overhead_settle_lqr_output():
    rope = RopeLengthModel(sheave_height=4.0)
    ctrl = OverheadLqrController(
        rope,
        load_mass=5.0,
        trolley_mass=250.0,
        max_correction=0.5,
        deadband_rad=0.0,
    )
    u_x, _ = ctrl.compute(0.04, 0.0, 0.01, z_grab=2.5)
    assert u_x > 0


def test_build_strategy_overhead_settle():
    cfg = apply_settle_anti_sway_mode(CraneConfig(), MODE_ID, {})
    assert cfg.anti_sway_mode == MODE_ID
    assert build_strategy(cfg) is not None


def test_sim_settle_overhead_converges():
    cfg = apply_settle_anti_sway_mode(
        CraneConfig(),
        MODE_ID,
        {},
    )
    anchor = (1.0, 1.0, 2.5)
    initial = CraneState(x0=anchor[0], y0=anchor[1], z0=anchor[2])
    plant = CranePlant(cfg)
    source = SimPositionSource(plant, initial, cfg)
    actuator = PlantActuator(plant, initial, cfg)
    sway = SimSwayFeedback(sheave_height=cfg.rope_length_sheave_height)
    history, _ = run_settle_control(
        source=source,
        actuator=actuator,
        config=cfg,
        anchor_pos=anchor,
        initial_state=initial,
        sway_source=sway,
        is_simulation=True,
        verbose=False,
        initial_theta=(0.08, 0.05),
        settle_theta_tol_rad=math.radians(1.0),
        settle_hold_sec=0.05,
        max_time=45.0,
    )
    assert len(history) > 10
    peak_cmd = max(
        abs(h.get('antisway_dvx', 0.0)) + abs(h.get('antisway_dvy', 0.0)) for h in history
    )
    assert peak_cmd > 0.005
    final = history[-1]
    t0 = max(abs(history[5]['theta_x']), abs(history[5]['theta_y']))
    tf = max(abs(final['theta_x']), abs(final['theta_y']))
    assert tf < t0 or tf < math.radians(0.8)
