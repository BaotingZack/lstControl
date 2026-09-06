import pytest

from sway_controller import AntiSwayController, RopeLengthModel


def _rope():
    return RopeLengthModel(sheave_height=8.0, grab_offset=1.5, cable_stretch=0.0)


def test_rope_length_formula():
    rope = _rope()
    # L_eff = 8.0 - Z + 1.5 = 9.5 - Z
    assert rope.compute(6.5) == pytest.approx(3.0)
    assert rope.compute(5.5) == pytest.approx(4.0)


def test_rope_length_stretch_and_min_clamp():
    rope = RopeLengthModel(sheave_height=8.0, grab_offset=1.5, cable_stretch=0.2, min_length=0.5)
    assert rope.compute(6.5) == pytest.approx(3.2)
    # 抓钩高于出绳点 → 计算为负, 应被下限保护
    rope_high = RopeLengthModel(sheave_height=1.0, grab_offset=0.0, min_length=0.5)
    assert rope_high.compute(5.0) == pytest.approx(0.5)


def test_fixed_gain_compute_positive_angle_feedback():
    """速度模式阻尼: Δv = +K·θ (摆角正反馈 = 追载荷), 符号必须为正。"""
    ctrl = AntiSwayController(_rope(), sway_gain=1.0, max_correction=1.0)
    dv_x, dv_y, L = ctrl.compute(theta_x=0.1, theta_y=-0.2, z_grab=6.5)
    assert L == pytest.approx(3.0)
    assert dv_x == pytest.approx(+1.0 * 0.1)
    assert dv_y == pytest.approx(+1.0 * -0.2)


def test_max_correction_clamp():
    ctrl = AntiSwayController(_rope(), sway_gain=1.0, max_correction=0.05)
    dv_x, _, _ = ctrl.compute(theta_x=1.0, theta_y=0.0, z_grab=6.5)
    assert dv_x == pytest.approx(0.05)  # +1.0 被限幅到 +0.05


def test_gain_schedule_interpolation():
    schedule = ((2.0, 0.5), (4.0, 1.5))
    ctrl = AntiSwayController(_rope(), sway_gain=0.0, gain_schedule=schedule, max_correction=10.0)

    def gain_at(L):
        # θx=1 → dv_x = +K(L), 直接读回增益
        return ctrl.compute(theta_x=1.0, theta_y=0.0, z_grab=9.5 - L)[0]

    assert gain_at(2.0) == pytest.approx(0.5)
    assert gain_at(4.0) == pytest.approx(1.5)
    # 线性插值中点
    assert gain_at(3.0) == pytest.approx(1.0)
    # 越界钳位
    assert gain_at(1.0) == pytest.approx(0.5)
    assert gain_at(5.0) == pytest.approx(1.5)


def test_gain_schedule_must_be_strictly_increasing():
    with pytest.raises(ValueError):
        AntiSwayController(_rope(), gain_schedule=((2.0, 0.5), (2.0, 0.5)))
