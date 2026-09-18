"""摆角提取 — 倾角仪单方案（不融合 IMU）。

控制律 Δv = +K(L)·θ 只需要摆角 θ，倾角仪 roll/pitch 单独即可提供 θ。
本模块只做"倾角仪角度 → 行车摆角"的轴映射、符号与单位换算，不含陀螺、不含滤波。

轴映射与符号 (现场安装确认 2026-09-18):
  倾角仪 Y 轴 (j1939 vector.y, 内部称 pitch) ∥ 大航车移动轴 → θx,
    但倾角仪 Y 正向与大车正向**相反**, 故 sign = -1;
  倾角仪 X 轴 (j1939 vector.x, 内部称 roll)  ∥ 小航车移动轴 → θy,
    正向一致, 故 sign = +1。
  实测值由 site_config.json 的 anti_sway_theta_*_from / _sign 提供。

Δθ

符号为什么必须对: 控制律 Δv = +K·θ 靠 θ 的符号提供阻尼, θ 必须满足
"θx > 0 ⟺ 载荷相对吊点偏向 +X"。符号标反就成了负阻尼, 会把摆越推越大。
下面的默认值已与现场安装一致, 这样即使 site_config 缺键也不会退回错符号。

"roll/pitch" 只是本仓库对两个原始通道的内部称法 (roll=vector.x, pitch=vector.y),
不要按航空"绕轴转"的含义理解 —— 那种解释会把轴映射推反。

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
    """θx 与 θy 的倾角仪映射。

    默认值 = 现场安装实测 (见模块文档): pitch→θx 且取反 (倾角仪 Y 正向与大车
    正向相反), roll→θy 同向。
    """
    theta_x: AxisMapping = field(default_factory=lambda: AxisMapping('pitch', -1.0))
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
