import math

import pytest

from pendulum_model import PendulumAxis


def test_free_swing_period_matches_formula():
    """无阻尼自由摆周期 T = 2π√(L/g)。"""
    L = 5.0
    axis = PendulumAxis(L=L, zeta=0.0)
    axis.reset(theta=0.1, theta_dot=0.0)
    dt = 0.001
    T_expected = 2.0 * math.pi * math.sqrt(L / axis.g)

    crossings = 0
    prev = axis.theta
    steps = int(3.0 * T_expected / dt)
    for _ in range(steps):
        axis.step(cart_accel=0.0, dt=dt)
        if prev > 0.0 and axis.theta <= 0.0:
            crossings += 1
        prev = axis.theta

    # 正→负过零次数 = 周期数；周期 = 总时间 / 过零数
    measured_T = (steps * dt) / max(crossings, 1)
    assert measured_T == pytest.approx(T_expected, rel=0.02)


def test_damping_reduces_swing():
    """ζ>0 时摆幅随时间衰减（对比前/后半段的最大幅值）。"""
    dt = 0.001
    L = 4.0
    axis = PendulumAxis(L=L, zeta=0.1)
    axis.reset(theta=0.2, theta_dot=0.0)
    N = int(8.0 / dt)
    max_first = 0.0
    max_last = 0.0
    for i in range(N):
        axis.step(0.0, dt)
        if i < N // 2:
            max_first = max(max_first, abs(axis.theta))
        else:
            max_last = max(max_last, abs(axis.theta))
    # 前半段仍接近初始幅值，后半段已明显衰减
    assert max_first == pytest.approx(0.2, rel=0.1)
    assert max_last < max_first * 0.7


def test_cart_acceleration_drives_swing_backward():
    """行车加速 +x 时载荷滞后（θ 变负），验证符号约定。

    θ 正 = 载荷朝 +x 偏移；行车朝 +x 加速 → 载荷惯性滞后 → θ 减小(变负)。
    """
    axis = PendulumAxis(L=5.0, zeta=0.0)
    axis.reset(theta=0.0, theta_dot=0.0)
    axis.step(cart_accel=+1.0, dt=0.001)
    assert axis.theta < 0.0


def test_step_returns_state_tuple():
    axis = PendulumAxis(L=5.0)
    axis.reset(theta=0.05, theta_dot=0.01)
    theta, theta_dot, theta_ddot = axis.step(cart_accel=0.0, dt=0.001)
    assert isinstance(theta, float)
    assert isinstance(theta_dot, float)
    assert isinstance(theta_ddot, float)


def test_invalid_arguments_rejected():
    with pytest.raises(ValueError):
        PendulumAxis(L=0.0)
    with pytest.raises(ValueError):
        PendulumAxis(L=5.0, zeta=-0.1)
    axis = PendulumAxis(L=5.0)
    with pytest.raises(ValueError):
        axis.step(cart_accel=0.0, dt=0.0)
    with pytest.raises(ValueError):
        axis.step(cart_accel=0.0, dt=0.01, L=0.0)
