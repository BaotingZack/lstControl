"""deepseek-anti: online static-angle estimation and settle-only wiring."""

from __future__ import annotations

import math

from anti_sway.deepseek_anti import (
    MODE_ID,
    DeepseekAntiController,
    _AxisStaticEstimator,
    build_strategy,
)
from anti_sway.registry import list_algorithm_options
from anti_sway.rope import RopeLengthModel
from crane_model import (
    CraneConfig,
    CranePlant,
    CraneState,
    PlantActuator,
    SimPositionSource,
    settle_angle_peak_rad,
)
from settle_control import apply_settle_anti_sway_mode, run_settle_control

G = 9.81


def _feed(est, *, bias_rad, amp_rad, omega, seconds, dt=0.08, phase=0.4, noise=0.0):
    for i in range(int(seconds / dt)):
        t = i * dt
        y = bias_rad + amp_rad * math.cos(omega * t + phase)
        if noise:
            y += noise * math.sin(7.3 * t)
        est.update(y, dt, omega)


def test_estimator_recovers_static_angle_and_amplitude():
    omega = math.sqrt(G / 3.0)
    est = _AxisStaticEstimator(window_periods=3.0)
    bias = math.radians(1.2)
    amp = math.radians(4.0)
    _feed(est, bias_rad=bias, amp_rad=amp, omega=omega, seconds=20.0)

    assert est.ready
    assert math.degrees(abs(est.static - bias)) < 0.02
    assert abs(math.degrees(est.amplitude) - 4.0) < 0.05
    assert math.degrees(est.rms) < 1e-3


def test_estimator_ignores_noise_dc():
    omega = math.sqrt(G / 3.0)
    est = _AxisStaticEstimator(window_periods=3.0)
    bias = math.radians(-0.8)
    _feed(
        est, bias_rad=bias, amp_rad=math.radians(2.5), omega=omega,
        seconds=24.0, noise=math.radians(0.15), phase=1.1,
    )
    assert est.ready
    assert math.degrees(abs(est.static - bias)) < 0.15


def test_estimator_not_ready_before_half_period():
    omega = math.sqrt(G / 3.0)
    est = _AxisStaticEstimator(window_periods=3.0)
    for _ in range(5):
        est.update(0.02, 0.08, omega)
    assert not est.ready


def test_settle_peak_uses_residual_ac_when_ready():
    """大静态倾角时 |θ| 可能很大，但圆点贴近 θ* 后仍应能判稳。"""
    tol = math.radians(0.3)
    debug = {
        'mode': 'deepseek_anti',
        'ready': True,
        'theta_ac_x_deg': 0.12,
        'theta_ac_y_deg': -0.08,
    }
    theta_x = math.radians(2.0)
    theta_y = math.radians(-1.5)
    peak = settle_angle_peak_rad(
        theta_x=theta_x,
        theta_y=theta_y,
        anti_sway_mode='deepseek_anti',
        anti_sway_debug=debug,
    )
    assert peak is not None and peak < tol
    raw_peak = max(abs(theta_x), abs(theta_y))
    assert raw_peak > tol


def test_settle_peak_not_ready_blocks_completion_metric():
    peak = settle_angle_peak_rad(
        theta_x=0.01,
        theta_y=0.01,
        anti_sway_mode='deepseek_anti',
        anti_sway_debug={'mode': 'deepseek_anti', 'ready': False},
    )
    assert peak is None


def test_settle_peak_baseline_uses_raw_theta():
    peak = settle_angle_peak_rad(
        theta_x=math.radians(0.2),
        theta_y=math.radians(0.1),
        anti_sway_mode='baseline',
        anti_sway_debug=None,
    )
    assert peak == math.radians(0.2)


def test_registered_for_settle_only():
    settle_ids = [o['id'] for o in list_algorithm_options(scope='settle')]
    ptp_ids = [o['id'] for o in list_algorithm_options(scope='ptp')]
    assert MODE_ID in settle_ids
    assert MODE_ID not in ptp_ids
    # 既有默认值不被改变：crane_remote 仍是 settle 列表第一项
    assert settle_ids[0] == 'crane_remote'


def test_build_strategy_only_for_its_own_mode():
    cfg = apply_settle_anti_sway_mode(CraneConfig(), MODE_ID, {})
    assert cfg.enable_anti_sway is True
    strat = build_strategy(cfg)
    assert strat is not None
    assert strat.mode_id == MODE_ID

    other = apply_settle_anti_sway_mode(CraneConfig(), 'baseline', {})
    assert build_strategy(other) is None


def test_other_modes_have_no_debug_channel():
    for mode in ('crane_remote', 'baseline', 'gantry_settle', 'overhead_settle'):
        cfg = apply_settle_anti_sway_mode(CraneConfig(), mode, {})
        strat = build_strategy(cfg)
        assert strat is None
        from anti_sway.registry import build_strategy as reg_build
        other_strat = reg_build(cfg)
        assert other_strat is not None
        assert not hasattr(other_strat, 'debug_snapshot')


def test_debug_snapshot_units_and_fields():
    rope = RopeLengthModel(sheave_height=4.0)
    ctrl = DeepseekAntiController(rope, gain=2.6)
    omega = math.sqrt(G / 3.0)
    for i in range(400):
        t = i * 0.08
        ctrl.compute(
            math.radians(1.0) + math.radians(3.0) * math.cos(omega * t),
            math.radians(-0.5) + math.radians(2.0) * math.sin(omega * t),
            0.08,
            1.0,  # z_grab -> L_eff = 3.0 m
        )
    snap = ctrl.debug_snapshot()
    for key in (
        'theta_static_x_deg', 'theta_static_y_deg',
        'theta_static_x_rad', 'theta_static_y_rad',
        'theta_ac_x_deg', 'theta_ac_y_deg',
        'amp_x_deg', 'amp_y_deg', 'rms_x_deg', 'rms_y_deg',
        'coverage_x', 'coverage_y', 'ready_x', 'ready_y', 'ready',
        'window_s', 'period_s', 'rope_m', 'dv_x', 'dv_y', 'gain', 'sign',
        'rate_x_deg_s', 'rate_y_deg_s', 'gain_eff', 'lead_s',
        'sign_x', 'sign_y',
        'dist_xy', 'budget_blocked', 'guard_x', 'guard_y', 'guard_trips',
    ):
        assert key in snap
    assert abs(snap['rope_m'] - 3.0) < 1e-9
    assert abs(snap['period_s'] - 2 * math.pi * math.sqrt(3.0 / G)) < 1e-6
    assert snap['ready'] is True
    assert math.degrees(abs(snap['theta_static_x_rad'] - math.radians(1.0))) < 0.05
    # 度与弧度必须是同一物理量
    assert abs(snap['theta_static_x_deg'] - math.degrees(snap['theta_static_x_rad'])) < 1e-9


def test_sign_flip_changes_correction_direction():
    rope = RopeLengthModel(sheave_height=4.0)
    plus = DeepseekAntiController(rope, gain=1.0, sign=1.0)
    minus = DeepseekAntiController(rope, gain=1.0, sign=-1.0)
    for _ in range(20):
        u_plus, _ = plus.compute(0.05, 0.0, 0.1, 1.0)
        u_minus, _ = minus.compute(0.05, 0.0, 0.1, 1.0)
    assert u_plus > 0
    assert u_minus < 0


def test_per_axis_sign_flips_only_that_axis():
    rope = RopeLengthModel(sheave_height=4.0)
    ctrl = DeepseekAntiController(
        rope, gain=2.6, sign=1.0, sign_x=1.0, sign_y=-1.0,
        guard_enabled=False,
    )
    u_x, u_y = ctrl.compute(0.03, 0.03, 0.1, 1.0, is_simulation=False)
    assert u_x > 0
    assert u_y < 0
    snap = ctrl.debug_snapshot()
    assert snap['sign_x'] == 1.0
    assert snap['sign_y'] == -1.0


def test_global_sign_multiplies_axis_signs():
    rope = RopeLengthModel(sheave_height=4.0)
    ctrl = DeepseekAntiController(
        rope, gain=2.6, sign=-1.0, sign_x=1.0, sign_y=-1.0,
        guard_enabled=False,
    )
    assert ctrl.sign_x == -1.0
    assert ctrl.sign_y == 1.0


def test_growth_guard_isolates_the_growing_axis():
    """只有 Y 的摆幅变大时，应当只掐 Y，X 不受影响。"""
    rope = RopeLengthModel(sheave_height=4.0)
    ctrl = DeepseekAntiController(
        rope, gain=2.0, guard_enabled=True, max_shift_m=0.0,
    )
    omega = math.sqrt(G / 3.0)
    dt = 0.08
    for i in range(int(7.0 / dt)):
        ctrl.compute(math.radians(1.0) * math.cos(omega * (i * dt)),
                     math.radians(1.0) * math.cos(omega * (i * dt)), dt, 1.0)
    tripped_y = False
    for i in range(int(8.0 / dt)):
        t = 7.0 + i * dt
        ctrl.compute(math.radians(1.0) * math.cos(omega * t),
                     math.radians(3.0) * math.cos(omega * t), dt, 1.0)
        snap = ctrl.debug_snapshot()
        if snap['guard_y']:
            tripped_y = True
            assert snap['guard_x'] is False
            break
    assert tripped_y


def test_control_ready_before_settle_ready():
    omega = math.sqrt(G / 3.0)
    est = _AxisStaticEstimator(window_periods=1.1)
    amp = math.radians(3.0)
    dt = 0.08
    period = 2 * math.pi / omega
    t_ctrl = None
    t_ready = None
    for i in range(int(12.0 / dt)):
        est.update(amp * math.cos(omega * (i * dt)), dt, omega)
        if t_ctrl is None and est.control_ready():
            t_ctrl = i * dt
        if t_ready is None and est.ready:
            t_ready = i * dt
    assert t_ctrl is not None and t_ready is not None
    assert t_ctrl < t_ready
    assert t_ctrl < period * 0.75


def test_model_rate_is_analytic_derivative():
    omega = math.sqrt(G / 3.0)
    est = _AxisStaticEstimator(window_periods=2.0)
    amp = math.radians(3.0)
    dt = 0.08
    for i in range(int(14.0 / dt)):
        est.update(amp * math.cos(omega * (i * dt)), dt, omega)
    expected = -amp * omega * math.sin(omega * est._t_last)
    assert abs(est.model_rate() - expected) < 1e-6


def test_lead_term_adds_derivative_action():
    rope = RopeLengthModel(sheave_height=4.0)
    ctrl = DeepseekAntiController(rope, gain=2.6)
    u_p = ctrl._law(0.01, 0.05, 2.6, 0.0, 1.0)
    u_lead = ctrl._law(0.01, 0.05, 2.6, 1.5, 1.0)
    assert u_lead > u_p
    u_neg = ctrl._law(0.01, -0.05, 2.6, 1.5, 1.0)
    assert u_neg < u_p


def test_gain_schedule_only_lowers_for_short_rope():
    rope = RopeLengthModel(sheave_height=4.0)
    ctrl = DeepseekAntiController(rope, gain=2.6, zeta_ref=0.24)
    assert abs(ctrl._effective_gain(3.65) - 2.6) < 1e-9      # 2.87 > 2.6，不上调
    assert ctrl._effective_gain(1.5) < 2.6                   # 短绳下调
    off = DeepseekAntiController(rope, gain=2.6, zeta_ref=0.0)
    assert off._effective_gain(1.5) == 2.6


def test_shift_budget_blocks_correction():
    rope = RopeLengthModel(sheave_height=4.0)
    ctrl = DeepseekAntiController(rope, gain=2.6, max_shift_m=0.06)
    for _ in range(10):
        u, _ = ctrl.compute(0.03, 0.0, 0.1, 1.0, dist_xy=0.0)
    assert u != 0.0
    u, _ = ctrl.compute(0.03, 0.0, 0.1, 1.0, dist_xy=0.10)
    assert u == 0.0
    assert ctrl.debug_snapshot()['budget_blocked'] is True


def test_growth_guard_suspends_growing_axis():
    rope = RopeLengthModel(sheave_height=4.0)
    ctrl = DeepseekAntiController(
        rope, gain=2.6, guard_enabled=True, max_shift_m=0.0,
    )
    omega = math.sqrt(G / 3.0)
    dt = 0.08
    for i in range(int(7.0 / dt)):
        ctrl.compute(math.radians(1.0) * math.cos(omega * (i * dt)), 0.0, dt, 1.0)
    tripped = False
    for i in range(int(8.0 / dt)):
        t = 7.0 + i * dt
        ctrl.compute(math.radians(3.0) * math.cos(omega * t), 0.0, dt, 1.0)
        if ctrl.debug_snapshot()['guard_x']:
            tripped = True
            break
    assert tripped
    assert ctrl.debug_snapshot()['guard_trips'] >= 1


def test_sim_settle_run_emits_debug():
    cfg = apply_settle_anti_sway_mode(CraneConfig(), MODE_ID, {})
    anchor = (1.0, 1.0, 2.5)
    initial = CraneState(x0=anchor[0], y0=anchor[1], z0=anchor[2])
    plant = CranePlant(cfg)
    source = SimPositionSource(plant, initial, cfg)
    actuator = PlantActuator(plant, initial, cfg)

    from anti_sway.sim_feedback import SimSwayFeedback

    sway = SimSwayFeedback(sheave_height=cfg.rope_length_sheave_height)
    history, _ = run_settle_control(
        source=source, actuator=actuator, config=cfg, anchor_pos=anchor,
        initial_state=initial, sway_source=sway, is_simulation=True,
        verbose=False, initial_theta=(0.05, 0.03),
        settle_theta_tol_rad=math.radians(0.5), settle_hold_sec=0.15,
        max_time=30.0,
    )
    assert len(history) > 0
    assert all('anti_sway_debug' in step for step in history)
    debug = history[-1]['anti_sway_debug']
    assert debug['mode'] == MODE_ID
    assert abs(debug['theta_x_deg'] - math.degrees(history[-1]['theta_x'])) < 1e-9
