"""摆角提取 — 倾角仪单方案（不融合 IMU）。

控制律 Δv = +K(L)·θ 只需要摆角 θ，倾角仪 roll/pitch 单独即可提供 θ。
本模块只做"倾角仪角度 → 行车摆角"的轴映射、符号与单位换算，不含陀螺、不含滤波。

轴映射 (现场标定 2026-09-16):
  倾角仪 Y 轴 (j1939 vector.y, 内部称 pitch) → 大车 X 方向 → θx;
  倾角仪 X 轴 (j1939 vector.x, 内部称 roll)  → 小车 Y 方向 → θy。
  符号正负由 SwayAxisMapping / site_config.json 配置。

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
    """θx 与 θy 的倾角仪映射。默认 pitch→θx (倾角仪 Y), roll→θy (倾角仪 X)。"""
    theta_x: AxisMapping = field(default_factory=lambda: AxisMapping('pitch', 1.0))
    theta_y: AxisMapping = field(default_factory=lambda: AxisMapping('roll', 1.0))


class InclinometerSwayEstimator:
    """倾角仪单方案摆角提取: roll/pitch → θx/θy（轴映射 + 符号 + 单位 + 抓钩状态 bias）。"""

    def __init__(
        self,
        mapping: SwayAxisMapping | None = None,
        angle_scale: float = 1.0,
        roll_bias: float = 0.0,
        pitch_bias: float = 0.0,
        *,
        roll_bias_closed: float | None = None,
        pitch_bias_closed: float | None = None,
    ):
        self.mapping = mapping or SwayAxisMapping()
        self.angle_scale = angle_scale
        self.roll_bias = roll_bias
        self.pitch_bias = pitch_bias
        self.roll_bias_closed = (
            roll_bias if roll_bias_closed is None else roll_bias_closed
        )
        self.pitch_bias_closed = (
            pitch_bias if pitch_bias_closed is None else pitch_bias_closed
        )

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

    def _biases(self, gripper_closed: bool | None) -> tuple[float, float]:
        if gripper_closed is True:
            return self.roll_bias_closed, self.pitch_bias_closed
        return self.roll_bias, self.pitch_bias

    def estimate(
        self,
        roll,
        pitch,
        *,
        gripper_closed: bool | None = None,
    ) -> dict:
        """由倾角仪 roll/pitch 计算行车摆角。

        Args:
            roll/pitch: 倾角仪原始角度（可能为 None）
            gripper_closed: True=夹紧 bias, False=张开 bias, None=张开

        Returns:
            {'theta_x', 'theta_y'} [rad]
        """
        rb, pb = self._biases(gripper_closed)
        roll_adj = None if roll is None else roll - rb
        pitch_adj = None if pitch is None else pitch - pb
        return {
            'theta_x': self._axis(self.mapping.theta_x, roll_adj, pitch_adj),
            'theta_y': self._axis(self.mapping.theta_y, roll_adj, pitch_adj),
        }
