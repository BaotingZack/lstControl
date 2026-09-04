import pytest

from sway_estimator import AxisMapping, ComplementarySwayFilter, SwayAxisMapping


def test_static_converges_to_inclinometer_angle():
    """静态 (陀螺=0) 时, 互补滤波应收敛到倾角仪角度 (低频绝对基准)。

    默认映射: pitch→θx, roll→θy。
    """
    filt = ComplementarySwayFilter(alpha=0.9)
    for _ in range(200):
        state = filt.update(roll=0.10, pitch=0.20, gx=0.0, gy=0.0, gz=0.0, dt=0.01)
    assert state['theta_y'] == pytest.approx(0.10, abs=1e-3)
    assert state['theta_x'] == pytest.approx(0.20, abs=1e-3)


def test_pure_gyro_integration_when_inclinometer_unavailable():
    """倾角仪未就绪 (roll/pitch=None) 时, 退化为纯陀螺积分。

    默认映射: theta_x 的角速度来自 gy, theta_y 的角速度来自 gx。
    """
    filt = ComplementarySwayFilter(alpha=0.98)
    state = None
    for _ in range(5):
        state = filt.update(roll=None, pitch=None, gx=0.1, gy=0.2, gz=0.0, dt=0.1)
    assert state['theta_y'] == pytest.approx(5 * 0.1 * 0.1)   # ∫gx dt
    assert state['theta_x'] == pytest.approx(5 * 0.2 * 0.1)   # ∫gy dt
    assert state['omega_x'] == pytest.approx(0.2)
    assert state['omega_y'] == pytest.approx(0.1)


def test_axis_mapping_sign_flip():
    """符号翻转: 把 roll 映射到 θx 且取反, 验证角度与角速度符号正确翻转。"""
    mapping = SwayAxisMapping(
        theta_x=AxisMapping('roll', -1.0, 'gx', -1.0),
        theta_y=AxisMapping('pitch', 1.0, 'gy', 1.0),
    )
    filt = ComplementarySwayFilter(alpha=0.98, mapping=mapping)
    # 角速度符号: ωx = -gx, ωy = +gy
    state = filt.update(roll=0.05, pitch=0.1, gx=0.2, gy=0.3, gz=0.0, dt=0.1)
    assert state['omega_x'] == pytest.approx(-0.2)
    assert state['omega_y'] == pytest.approx(0.3)
    # 静态收敛 (陀螺=0): θx → angle_sign·roll = -0.05, θy → +pitch = 0.1
    for _ in range(300):
        state = filt.update(roll=0.05, pitch=0.1, gx=0.0, gy=0.0, gz=0.0, dt=0.1)
    assert state['theta_x'] == pytest.approx(-0.05, abs=1e-3)
    assert state['theta_y'] == pytest.approx(0.1, abs=1e-3)


def test_alpha_must_be_in_range():
    with pytest.raises(ValueError):
        ComplementarySwayFilter(alpha=1.5)


def test_reset_clears_state():
    filt = ComplementarySwayFilter(alpha=0.9)
    filt.update(roll=0.3, pitch=0.4, gx=0.0, gy=0.0, gz=0.0, dt=0.01)
    filt.reset()
    assert filt.state == {
        'theta_x': 0.0, 'theta_y': 0.0, 'omega_x': 0.0, 'omega_y': 0.0,
    }
