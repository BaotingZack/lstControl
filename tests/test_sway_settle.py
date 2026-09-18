"""sway_settle 模块单测 — 自适应窗 / 去趋势峰峰值 / θ_dc 跟踪。"""

from __future__ import annotations

import math

import pytest

from sway_settle import (
    GRAVITY,
    SwayAmplitudeTracker,
    SwayLiveAnalyzer,
    ThetaDcTracker,
    adaptive_settle_window,
    effective_rope_length,
    pendulum_period,
    sway_linear_detrend_peak_to_peak,
    sway_raw_peak_to_peak,
    window_samples_full,
    wire_rope_length,
)


def test_physical_hoist_geometry():
    """L_eff = 4 − Z; L_rope = 4 − Z − 1.5。禁止把 1.5 再加进摆长。"""
    assert effective_rope_length(1.5) == pytest.approx(2.5)
    assert effective_rope_length(0.36) == pytest.approx(3.64)
    assert wire_rope_length(1.5) == pytest.approx(1.0)
    assert wire_rope_length(0.36) == pytest.approx(2.14)
    # 旧配法 sheave=4 offset=1.5 会把结构加两次
    assert effective_rope_length(1.5, sheave_height=4.0, grab_offset=1.5) == pytest.approx(4.0)


def test_energy_amplitude_tracks_sine():
    """Â² = θ² + (θ̇/ωn)² 对已知正弦应接近振幅, 不必等半个周期。"""
    L = 2.5
    wn = math.sqrt(GRAVITY / L)
    A = math.radians(1.0)
    tr = SwayAmplitudeTracker(rate_tau=0.15)
    dt = 0.05
    last = 0.0
    for k in range(80):
        t = k * dt
        theta = A * math.sin(wn * t)
        last = tr.update(theta, 0.0, dt, wn)[2]
    assert last == pytest.approx(A, rel=0.25)


def test_energy_amplitude_rate_ignores_static_bias():
    """纯零偏 |θ̇|≈0 → rate_amp 接近 0, 不能当成在摆。"""
    tr = SwayAmplitudeTracker()
    wn = 2.0
    bias = math.radians(0.44)
    for _ in range(30):
        tr.update(bias, 0.0, 0.1, wn)
    assert tr.last_rate_amp < math.radians(0.03)


def test_pendulum_period_increases_with_lower_z():
    T_low = pendulum_period(0.4, sheave_height=8.0, grab_offset=1.5)
    T_high = pendulum_period(1.5, sheave_height=8.0, grab_offset=1.5)
    assert T_low > T_high


def test_adaptive_window_at_least_base():
    w = adaptive_settle_window(
        0.4, base_window=0.6, window_periods=0.55,
        sheave_height=8.0, grab_offset=1.5,
    )
    assert w >= 0.6
    assert w <= 8.0


def test_raw_pp_constant_bias_is_zero():
    """常数偏置不改变峰峰值。"""
    samples = [(float(i), 0.5, -0.3) for i in range(10)]
    pp = sway_raw_peak_to_peak(samples)
    assert pp == pytest.approx((0.0, 0.0))


def test_detrend_pp_removes_ramp():
    """线性斜坡在去趋势后峰峰值应接近 0。"""
    samples = [(float(i), 0.01 * i, 0.0) for i in range(20)]
    raw = sway_raw_peak_to_peak(samples)
    det = sway_linear_detrend_peak_to_peak(samples)
    assert raw is not None and det is not None
    assert max(det) < max(raw) * 0.1


def test_detrend_pp_keeps_sine():
    """正弦摆动去趋势后仍有可观幅值。"""
    samples = [
        (i * 0.1, 0.2 * math.sin(i * 0.5), 0.0)
        for i in range(40)
    ]
    det = sway_linear_detrend_peak_to_peak(samples)
    assert det is not None
    assert math.degrees(max(det)) > 10.0


def test_theta_dc_tracker_converges_to_step():
    dc = ThetaDcTracker(tau_s=0.5)
    ac = dc.update(0.0, 0.0, 0.02)
    assert ac == pytest.approx((0.0, 0.0))
    for _ in range(200):
        ac = dc.update(0.1, 0.0, 0.02)
    assert dc.dc_x == pytest.approx(0.1, abs=0.02)
    assert ac[0] == pytest.approx(0.0, abs=0.02)


def test_sway_live_analyzer_settled_on_constant_bias():
    live = SwayLiveAnalyzer(
        base_window=0.2, window_periods=0.0, tol_deg=0.3,
    )
    for i in range(50):
        live.push(i * 0.02, 0.5, -0.3)
    m = live.metrics(z_grab=1.0)
    assert m['window_full'] is True
    assert m['detrend_pp_max_deg'] == pytest.approx(0.0, abs=0.01)
    assert m['amp_max_deg'] is not None
    assert m['settled'] is True


def test_window_samples_full_uses_span_not_wall_clock():
    samples = [(i * 0.5, 0.0, 0.0) for i in range(8)]
    assert window_samples_full(samples, 3.0) is True
    assert window_samples_full(samples[:2], 3.0) is False


def test_sway_live_analyzer_detects_swing():
    live = SwayLiveAnalyzer(
        base_window=0.4, window_periods=0.0, tol_deg=0.3,
    )
    for i in range(80):
        t = i * 0.05
        live.push(t, 0.15 * math.sin(t * 3.0), 0.0)
    m = live.metrics(z_grab=1.0)
    assert m['detrend_pp_max_deg'] is not None
    assert m['detrend_pp_max_deg'] > 0.2
    assert m['settled'] is False
