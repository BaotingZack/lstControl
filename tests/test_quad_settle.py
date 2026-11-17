"""Tests for four-quadrant lead-compensated anti-sway settle (quad_settle & quick_settle)."""

from __future__ import annotations

import math

from anti_sway.quad_settle import (
    MODE_ID,
    QuadLeadAntiSwayController,
    build_strategy,
    classify_sway_quadrant,
)
from anti_sway.registry import list_algorithm_options
from anti_sway.sim_feedback import SimSwayFeedback
from crane_model import CraneConfig, CranePlant, CraneState, PlantActuator, SimPositionSource
import quick_settle
from settle_control import apply_settle_anti_sway_mode, run_settle_control


def test_quadrant_classification():
    db = math.radians(0.1)
    wdb = math.radians(0.2)
    assert classify_sway_quadrant(0.0, 0.0, deadband_rad=db, omega_deadband_rad_s=wdb) == 0
    # Q1: 0 -> +max (forward swing, theta > 0, omega > 0)
    assert classify_sway_quadrant(0.05, 0.08, deadband_rad=db, omega_deadband_rad_s=wdb) == 1
    # Q2: +max -> 0 (returning from forward peak, theta > 0, omega < 0)
    assert classify_sway_quadrant(0.05, -0.08, deadband_rad=db, omega_deadband_rad_s=wdb) == 2
    # Q3: 0 -> -max (backward swing, theta < 0, omega < 0)
    assert classify_sway_quadrant(-0.05, -0.08, deadband_rad=db, omega_deadband_rad_s=wdb) == 3
    # Q4: -max -> 0 (returning from backward peak, theta < 0, omega > 0)
    assert classify_sway_quadrant(-0.05, 0.08, deadband_rad=db, omega_deadband_rad_s=wdb) == 4


def test_lead_compensation_overcomes_velocity_ramp():
    """Verify Q1 boosts v_cmd above v* (to accelerate fast) and Q2 pulls v_cmd below v* (to decelerate early)."""
    ctrl = QuadLeadAntiSwayController(
        kv=2.0,
        tau_lead=0.45,
        max_vel=0.35,
        dc_tau=0.0,
        omega_tau=0.0,
        recenter_gain=0.0,
        sign_x=1.0,
        sign_y=1.0,
    )
    # Step 1 at origin, Step 2 swinging forward (theta: 0 -> +0.04 rad in 0.05s => omega > 0, Q1)
    ctrl.compute(0.0, 0.0, 0.05)
    vx_q1, _ = ctrl.compute(0.04, 0.0, 0.05)
    snap_q1 = ctrl.debug_snapshot()
    assert snap_q1['quad_x'] == 1
    assert snap_q1['v_star_x'] > 0.0
    assert snap_q1['a_star_x'] > 0.0
    # Lead command must exceed steady target velocity v* to overcome drive acceleration lag
    assert vx_q1 > snap_q1['v_star_x']

    # Now returning from peak: theta drops from +0.06 -> +0.04 (theta > 0, omega < 0, Q2)
    ctrl.compute(0.06, 0.0, 0.05)
    vx_q2, _ = ctrl.compute(0.04, 0.0, 0.05)
    snap_q2 = ctrl.debug_snapshot()
    assert snap_q2['quad_x'] == 2
    assert snap_q2['v_star_x'] > 0.0
    assert snap_q2['a_star_x'] < 0.0
    # Early deceleration: v_cmd is pulled well below v* before reaching origin
    assert vx_q2 < snap_q2['v_star_x']


def test_registered_in_settle_only():
    settle_ids = [o['id'] for o in list_algorithm_options(scope='settle')]
    ptp_ids = [o['id'] for o in list_algorithm_options(scope='ptp')]
    assert MODE_ID in settle_ids
    assert MODE_ID not in ptp_ids
    assert settle_ids[0] == 'crane_remote'


def test_sim_settle_quad_converges_fast():
    cfg = apply_settle_anti_sway_mode(CraneConfig(), MODE_ID, {})
    assert build_strategy(cfg) is not None
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
        initial_theta=(0.09, 0.06),
        settle_theta_tol_rad=math.radians(0.35),
        settle_hold_sec=0.05,
        max_time=25.0,
    )
    assert len(history) > 10
    final = history[-1]
    assert max(abs(final['theta_x']), abs(final['theta_y'])) < math.radians(0.4)


def test_quick_settle_cli_sim_mode():
    rc = quick_settle.main([
        '--sim',
        '--sim-init-x-deg', '5.0',
        '--sim-init-y-deg', '3.0',
        '--sim-ramp-tau', '0.50',
        '--tol-deg', '0.30',
        '--hold-sec', '1.5',
        '--max-time', '25.0',
    ])
    assert rc == 0
