"""原地保持消摆验证脚本的纯函数测试 —— 不连 PLC、不连 ROS。

只测离线可判定的部分: 数据活性探针、包络时间常数、窗内峰峰值、判稳时刻、
逐轴三态判定、以及启动计划解析。阶段机本身需要假被控对象, 见脚本的 `_run`。
"""

from __future__ import annotations

import math

import pytest

from script.verify_hold_damping import (
    _jsonable,
    axis_verdict,
    envelope_tau,
    frozen_sensor,
    resolve_plan,
    sample_rate_hz,
    settle_time,
    window_pp,
)
from site_config import DEFAULT_SITE_CONFIG_PATH, load_site_config

_TAU = 3.0
_PERIOD = 3.47


def _decaying(amp_deg=2.0, tau=_TAU, duration=20.0, hz=10.0):
    """θ = A·exp(−t/τ)·sin(2πt/T) 的采样, 返回 (t, θx, θy)。"""
    out = []
    n = int(duration * hz)
    for i in range(n):
        t = i / hz
        theta = math.radians(amp_deg) * math.exp(-t / tau) * math.sin(2 * math.pi * t / _PERIOD)
        out.append((t, theta, 0.0))
    return out


def _constant(amp_deg=1.0, duration=2.0, hz=10.0):
    out = []
    for i in range(int(duration * hz)):
        t = i / hz
        out.append((t, math.radians(amp_deg), 0.0))
    return out


def _plan(args=None):
    import main

    site = load_site_config(DEFAULT_SITE_CONFIG_PATH)
    base = main._build_arg_parser().parse_args([])
    config = main._config_from_args(base, site, argv=[])
    argv = ['--plc-ip', '1.2.3.4'] + (args or [])
    cli = __import__('script.verify_hold_damping', fromlist=['x'])._build_parser().parse_args(argv)
    return resolve_plan(config, 1.5, cli), config


# --------------------------------------------------------------------------
# 数据活性: 这是"倾角仪在重复上一帧"这道静默失败的探针
# --------------------------------------------------------------------------

def test_sample_rate_counts_only_real_changes():
    """读数在重复时不能算作更新 —— 否则 θ̇≈0 的静默失败会被漏掉。

    40 拍、10Hz 采样, 但只有 2 个不同的读数: 真实更新率必须远低于 10Hz。
    """
    samples = [(i * 0.1, 0.5 if i < 20 else 0.6, 0.0) for i in range(40)]
    rate = sample_rate_hz(samples)
    assert rate < 1.0, f'重复帧被当成更新: {rate:.2f} Hz'


def test_sample_rate_matches_true_rate_when_updating():
    samples = _decaying(duration=2.0, hz=10.0)
    rate = sample_rate_hz(samples)
    assert 8.0 < rate < 12.0, rate


def test_sample_rate_zero_with_single_sample():
    assert sample_rate_hz([(0.0, 0.1, 0.0)]) == 0.0
    assert sample_rate_hz([]) == 0.0


def test_frozen_sensor_flags_repeated_readings():
    assert frozen_sensor([(i * 0.1, 0.5, 0.0) for i in range(40)]) is True
    assert frozen_sensor(_decaying(duration=4.0)) is False


def test_frozen_sensor_ignores_short_windows():
    """样本太少时不下结论 —— 否则启动瞬间就会误报冻结。"""
    assert frozen_sensor([(i * 0.1, 0.5, 0.0) for i in range(5)]) is False


# --------------------------------------------------------------------------
# 衰减/残摆指标
# --------------------------------------------------------------------------

def test_envelope_tau_recovers_known_decay():
    tau = envelope_tau(_decaying(tau=2.0, duration=20.0))
    assert tau is not None
    assert abs(tau - 2.0) < 0.6, tau


def test_envelope_tau_none_when_not_decaying():
    """等幅摆动不能编一个衰减时间出来。"""
    out = []
    for i in range(200):
        t = i * 0.1
        out.append((t, math.radians(2.0) * math.sin(2 * math.pi * t / _PERIOD), 0.0))
    assert envelope_tau(out) is None


def test_envelope_tau_none_when_too_few_samples():
    assert envelope_tau(_decaying(duration=4.0)) is None


def test_window_pp_is_peak_to_peak_in_degrees():
    """等幅 ±1° → 峰峰值 2°。原始 PP 是硬上界 (离散采样只会低估);
    去趋势 PP 在窗长不是整数个周期时会略高于原始 PP, 这是该指标本身的特性,
    报告里两个都打出来就是让现场能看到这个偏差。"""
    samples = _decaying(amp_deg=1.0, tau=1e6, duration=8.0)
    metrics = window_pp(samples, _PERIOD)
    assert metrics['raw_pp_max_deg'] == pytest.approx(2.0, abs=0.05)
    assert metrics['detrend_pp_max_deg'] == pytest.approx(2.0, abs=0.3)
    assert metrics['sample_count'] > 0


def test_window_pp_empty_when_no_samples():
    metrics = window_pp([], 3.0)
    assert metrics['sample_count'] == 0
    assert metrics['detrend_pp_max_deg'] is None


def test_settle_time_none_when_never_settles():
    plan, _ = _plan(['--observe', '0'])
    plan['z_grab'] = 1.5
    assert settle_time(_decaying(amp_deg=3.0, tau=1e6, duration=20.0), plan) is None


def test_settle_time_found_when_amplitude_collapses():
    """先大摆一段再安静下来 → 应报出"此后不再超差"的那个时刻, 而不是首个短窗。"""
    plan, _ = _plan(['--observe', '0'])
    plan['z_grab'] = 1.5
    samples = []
    for i in range(200):
        t = i * 0.1
        amp = 2.0 if t < 8.0 else 0.0
        samples.append((t, math.radians(amp) * math.sin(2 * math.pi * t / _PERIOD), 0.0))
    settled = settle_time(samples, plan)
    assert settled is not None
    assert settled >= 8.0, f'把大摆段的巧合短窗当成了判稳: {settled}'


# --------------------------------------------------------------------------
# 逐轴判定: 一轴的改善不得抵消另一轴的恶化
# --------------------------------------------------------------------------

def test_axis_verdict_three_states():
    tol = 0.3
    assert axis_verdict(4.0, 0.5, tol) == '改善'
    assert axis_verdict(4.0, 6.0, tol) == '恶化'
    # 死区 = 半个容差: 比它小的变化只是推钩之间的噪声, 不能叫改善或恶化
    assert axis_verdict(4.0, 4.1, tol) == '未改善'
    assert axis_verdict(4.0, 3.9, tol) == '未改善'


def test_axis_verdict_is_per_axis_not_composite():
    """同样的容差下两轴各自得到独立结论 —— 没有任何合成幅值的入口。"""
    tol = 0.3
    assert axis_verdict(4.0, 0.2, tol) == '改善'
    assert axis_verdict(0.2, 4.0, tol) == '恶化'


# --------------------------------------------------------------------------
# 启动计划解析
# --------------------------------------------------------------------------

def test_resolve_plan_uses_locked_budget_and_tol_over_four_amp_floor():
    """原地验证必须用 locked 预算 (settle 预算是 0, 会让消摆静默失效), 且幅度
    门限取 tol/4 —— 取默认 0.15° 会正好停在容差上, 永远进不了自己的判据。"""
    plan, config = _plan()
    assert plan['budget_m'] == config.anti_sway_locked_max_shift
    assert plan['budget_m'] > 0.0
    assert plan['amp_floor_deg'] == pytest.approx(config.hook_sway_angle_tol_deg / 4.0)
    assert plan['gain_scale'] == config.anti_sway_settle_gain_scale
    assert plan['effective_gain'] == pytest.approx(
        config.anti_sway_sway_gain * config.anti_sway_settle_gain_scale
    )


def test_resolve_plan_rope_geometry():
    """L_eff = H − Z, 不允许把 1.5m 结构再加一次到 grab_offset 里。"""
    plan, config = _plan()
    assert plan['rope_length_m'] == pytest.approx(
        config.rope_length_sheave_height - plan['z_grab']
    )
    assert plan['period_s'] == pytest.approx(2 * math.pi * math.sqrt(plan['rope_length_m'] / 9.81))


def test_terminal_strategy_uses_unit_gain_scale():
    """作业里 terminal_policy 恒用 gain_scale=1.0 (crane_model.py:1189-1190),
    只有非终端的锁轴悬停阻尼才乘 anti_sway_settle_gain_scale。验证脚本必须与作业
    一致, 否则 --strategy terminal 测的不是作业那条通路。"""
    mech, config = _plan(['--strategy', 'mechanism'])
    term, _ = _plan(['--strategy', 'terminal'])
    assert mech['strategy'] == 'mechanism' and term['strategy'] == 'terminal'
    assert mech['gain_scale'] == config.anti_sway_settle_gain_scale
    assert term['gain_scale'] == 1.0
    assert term['effective_gain'] == pytest.approx(config.anti_sway_sway_gain)


def test_resolve_plan_overrides():
    plan, config = _plan(['--max-shift', '0.12', '--recenter-gain', '0',
                          '--amp-floor-deg', '0.05', '--observe', '0'])
    assert plan['budget_m'] == pytest.approx(0.12)
    assert plan['recenter_gain'] == 0.0
    assert plan['amp_floor_deg'] == pytest.approx(0.05)
    assert plan['observe_s'] == 0.0


def test_resolve_plan_default_observe_is_two_periods_plus():
    plan, _ = _plan()
    assert plan['observe_s'] >= 2 * plan['period_s']


def test_jsonable_replaces_non_finite_and_numpy_scalars():
    import numpy as np

    payload = _jsonable({'a': math.inf, 'b': -math.inf, 'c': float('nan'),
                         'd': np.float64(1.5), 'e': [np.int64(2)], 'f': None})
    assert payload == {'a': None, 'b': None, 'c': None, 'd': 1.5, 'e': [2], 'f': None}
