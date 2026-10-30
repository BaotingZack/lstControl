"""摆角提取 — 倾角仪单方案（不融合 IMU）。

控制律需要真实摆角 θ；本模块做 roll/pitch → θx/θy 的轴映射、符号与单位换算。
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class AxisMapping:
    angle_source: str | None = None
    angle_sign: float = 1.0


@dataclass(frozen=True)
class SwayAxisMapping:
    theta_x: AxisMapping = field(default_factory=lambda: AxisMapping('pitch', -1.0))
    theta_y: AxisMapping = field(default_factory=lambda: AxisMapping('roll', 1.0))


class InclinometerSwayEstimator:
    """倾角仪单方案摆角提取: roll/pitch → θx/θy（轴映射 + 符号 + 单位 + bias）。"""

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
        rb, pb = self._biases(gripper_closed)
        roll_adj = None if roll is None else roll - rb
        pitch_adj = None if pitch is None else pitch - pb
        return {
            'theta_x': self._axis(self.mapping.theta_x, roll_adj, pitch_adj),
            'theta_y': self._axis(self.mapping.theta_y, roll_adj, pitch_adj),
        }
