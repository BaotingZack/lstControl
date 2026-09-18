"""倾角仪轴映射与符号 — 钉死现场安装标定 (2026-09-18)。

为什么值得单独一个文件: 控制律是 Δv = +K·θ, θ 的符号就是阻尼的符号。符号标反
不会报错, 而是安静地变成**负阻尼**, 把摆越推越大。所以这几条用例不是在测实现
细节, 而是在守住一个物理约定:

    θx > 0  ⟺  载荷相对吊点偏向 +X (大航车正向)
    θy > 0  ⟺  载荷相对吊点偏向 +Y (小航车正向)

现场安装 (2026-09-18 实测确认):
  倾角仪 X 轴 ∥ 小航车轴, 正向一致  → θy ← vector.x (roll),  sign +1
  倾角仪 Y 轴 ∥ 大航车轴, 正向相反  → θx ← vector.y (pitch), sign -1
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from site_config import build_sway_axis_mapping, load_site_config
from sway_estimator import InclinometerSwayEstimator, SwayAxisMapping

CONFIG = Path(__file__).resolve().parent.parent / 'site_config.json'

# 现场那次人工推钩的实测原始读数 (Z≈1.5m, 空载, 朝 +X 推):
# 静止 → 峰值, θx 峰峰 1.921°, 同期 θy 仅动 0.213° (6.4:1 定轴归属)
FIELD_REST = (-1.296, -0.541)     # (vector.x, vector.y)
FIELD_PUSH_PLUS_X = (-1.509, -2.462)


def _estimator(mapping=None) -> InclinometerSwayEstimator:
    site = load_site_config(CONFIG)
    return InclinometerSwayEstimator(
        mapping=mapping if mapping is not None else build_sway_axis_mapping(site),
        angle_scale=math.pi / 180.0,
        roll_bias=site['anti_sway_roll_bias'],
        pitch_bias=site['anti_sway_pitch_bias'],
    )


class TestInstalledAxisMapping:

    def test_site_config_matches_installation(self):
        cfg = json.loads(CONFIG.read_text(encoding='utf-8'))
        assert cfg['anti_sway_theta_x_from'] == 'pitch', '大航车的摆在 vector.y 上'
        assert cfg['anti_sway_theta_y_from'] == 'roll', '小航车的摆在 vector.x 上'
        assert cfg['anti_sway_theta_x_sign'] == -1.0, (
            '倾角仪 Y 正向与大车正向相反 → θx 必须取反; 标成 +1 会变成负阻尼'
        )
        assert cfg['anti_sway_theta_y_sign'] == 1.0, '倾角仪 X 与小车正向一致'

    def test_defaults_match_installation(self):
        """site_config 缺键时的默认值也必须是对的 — 不能静默退回错符号。"""
        default = SwayAxisMapping()
        assert (default.theta_x.angle_source, default.theta_x.angle_sign) == ('pitch', -1.0)
        assert (default.theta_y.angle_source, default.theta_y.angle_sign) == ('roll', 1.0)


class TestSignConvention:
    """用现场实测原始读数守住 "θ 的正方向 = 载荷偏移方向" 这个约定。"""

    def test_push_toward_plus_x_gives_positive_theta_x(self):
        est = _estimator()
        rest = est.estimate(*FIELD_REST)
        push = est.estimate(*FIELD_PUSH_PLUS_X)
        d_theta_x = math.degrees(push['theta_x'] - rest['theta_x'])
        assert d_theta_x > 0.0, (
            f'朝 +X 推 → θx 必须变正, 实际 {d_theta_x:+.3f}°。'
            '为负说明符号标反, 防摇会放大摆动'
        )
        assert d_theta_x == pytest.approx(1.921, rel=0.01)

    def test_big_car_sway_lands_on_theta_x_not_theta_y(self):
        """轴归属: 大航车方向的摆必须几乎全部落在 θx 上 (现场实测 6.4:1)。"""
        est = _estimator()
        rest = est.estimate(*FIELD_REST)
        push = est.estimate(*FIELD_PUSH_PLUS_X)
        dx = abs(math.degrees(push['theta_x'] - rest['theta_x']))
        dy = abs(math.degrees(push['theta_y'] - rest['theta_y']))
        assert dx > 5.0 * dy, f'大车方向的摆应主要进 θx, 实际 dx={dx:.3f} dy={dy:.3f}'

    def test_damping_command_chases_the_load(self):
        """端到端: θx>0 (载荷偏 +X) → Δv 必须朝 +X, 即小车追着载荷走。"""
        from sway_controller import AntiSwayController, RopeLengthModel

        ctrl = AntiSwayController(
            RopeLengthModel(4.0, 0.0), sway_gain=2.6, max_correction=0.2,
        )
        est = _estimator()
        rest = est.estimate(*FIELD_REST)
        push = est.estimate(*FIELD_PUSH_PLUS_X)
        theta_x = push['theta_x'] - rest['theta_x']
        dvx, _dvy, _L = ctrl.compute(theta_x=theta_x, theta_y=0.0, z_grab=1.5)
        assert dvx > 0.0, (
            f'载荷偏 +X 时修正速度必须为正 (追载荷), 实际 {dvx:+.4f} m/s — '
            '为负即负阻尼'
        )
