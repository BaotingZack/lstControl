"""摆角提取 — 倾角仪单方案（不融合 IMU）。

控制律 Δv = +K(L)·θ 只需要摆角 θ，倾角仪 roll/pitch 单独即可提供 θ。
本模块只做"倾角仪角度 → 行车摆角"的轴映射、符号与单位换算，不含陀螺、不含滤波。

轴映射:
  抓钩在 X(大车)方向摆动 = 绕 Y 轴旋转 = pitch;
  抓钩在 Y(小车)方向摆动 = 绕 X 轴旋转 = roll。
  具体哪条传感器轴对应哪条行车轴、符号正负, 取决于抓钩上的安装朝向,
  需现场标定后通过 SwayAxisMapping 配置。默认按航天惯例 pitch→θx、roll→θy。

说明: 倾角仪在行车加减速时会受"假倾角"污染 (加速度计混入运动加速度)。这是
在线估计精度问题, 由后续 EKF/加速度补偿解决; 控制律本身只需 θ, 倾角仪单方案
足以验证防摇的有效性。
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class AxisMapping:
    """单条行车摆动轴的倾角仪来源映射。

    angle_source: 倾角仪字段 'roll' | 'pitch' | None (None=该轴无角度)
    angle_sign:   该来源的符号 (现场标定时确定正负)
    """
    angle_source: str | None = None
    angle_sign: float = 1.0


@dataclass(frozen=True)
class SwayAxisMapping:
    """θx 与 θy 的倾角仪映射。默认按航天惯例 (pitch→θx, roll→θy)。"""
    theta_x: AxisMapping = field(default_factory=lambda: AxisMapping('pitch', 1.0))
    theta_y: AxisMapping = field(default_factory=lambda: AxisMapping('roll', 1.0))


class InclinometerSwayEstimator:
    """倾角仪单方案摆角提取: roll/pitch → θx/θy（轴映射 + 符号 + 单位）。"""

    def __init__(self, mapping: SwayAxisMapping | None = None, angle_scale: float = 1.0):
        self.mapping = mapping or SwayAxisMapping()
        self.angle_scale = angle_scale

    @staticmethod
    def _pick(src: str | None, roll, pitch):
        if src == 'roll':
            return roll
        if src == 'pitch':
            return pitch
        return None

    def _axis(self, m: AxisMapping, roll, pitch) -> float:
        angle = self._pick(m.angle_source, roll, pitch)
        if angle is None:
            return 0.0
        return m.angle_sign * angle * self.angle_scale

    def estimate(self, roll, pitch) -> dict:
        """由倾角仪 roll/pitch 计算行车摆角。

        Args:
            roll/pitch: 倾角仪原始角度（可能为 None）

        Returns:
            {'theta_x', 'theta_y'} [rad]
        """
        return {
            'theta_x': self._axis(self.mapping.theta_x, roll, pitch),
            'theta_y': self._axis(self.mapping.theta_y, roll, pitch),
        }
