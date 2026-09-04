"""摆角状态估计 — 互补滤波融合倾角仪(低频绝对基准)与 IMU 陀螺(高频角速度)。

原理
----
倾角仪给出抓钩相对竖直的 roll(横滚)/pitch(俯仰), 长期稳定但动态下会被
平移/向心加速度污染(假倾角);
陀螺给出绕各轴的角速度, 高频准确但纯积分有零偏漂移。
互补滤波 = 陀螺高频积分 + 倾角仪低频校正, 单轴形式:

    θ̂[k] = α·( θ̂[k-1] + ω·dt ) + (1-α)·θ_meas

α 越接近 1 越信任陀螺(动态响应好), 越小越信任倾角仪(长期稳定)。

轴映射
------
抓钩在 X(大车)方向摆动 = 绕 Y 轴旋转 = pitch;
抓钩在 Y(小车)方向摆动 = 绕 X 轴旋转 = roll。
具体哪条传感器轴对应哪条行车轴、符号正负, 取决于抓钩上的安装朝向, 需现场
标定后通过 SwayAxisMapping 配置。默认按上述航天惯例 (pitch→θx, roll→θy)。
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class AxisMapping:
    """单条行车摆动轴的传感器来源映射。

    angle_source: 倾角仪字段 'roll' | 'pitch' | None (None=该轴无绝对基准, 纯陀螺积分)
    rate_source:  陀螺轴 'gx' | 'gy' | 'gz' | None
    *_sign:       该来源的符号 (现场标定时确定正负)
    """
    angle_source: str | None = None
    angle_sign: float = 1.0
    rate_source: str | None = None
    rate_sign: float = 1.0


@dataclass(frozen=True)
class SwayAxisMapping:
    """θx 与 θy 的传感器映射。默认按航天惯例 (见模块 docstring)。"""
    theta_x: AxisMapping = field(default_factory=lambda: AxisMapping('pitch', 1.0, 'gy', 1.0))
    theta_y: AxisMapping = field(default_factory=lambda: AxisMapping('roll', 1.0, 'gx', 1.0))


class ComplementarySwayFilter:
    """两轴互补滤波: 融合倾角仪 roll/pitch 与陀螺三轴角速度 → θx,θy,θ̇x,θ̇y。

    设计为在 IMU 回调线程内按 IMU 采样率调用 (通常 ≥100Hz), 以正确积分陀螺;
    倾角仪只提供低频基准, 每次取"最新可用"的 roll/pitch 即可。
    """

    def __init__(self, alpha: float = 0.98, mapping: SwayAxisMapping | None = None):
        if not 0.0 <= alpha <= 1.0:
            raise ValueError('alpha must be in [0, 1]')
        self.alpha = alpha
        self.mapping = mapping or SwayAxisMapping()
        self._theta_x = 0.0
        self._theta_y = 0.0
        self._omega_x = 0.0
        self._omega_y = 0.0

    @staticmethod
    def _pick(src: str | None, roll, pitch, gx, gy, gz):
        if src == 'roll':
            return roll
        if src == 'pitch':
            return pitch
        if src == 'gx':
            return gx
        if src == 'gy':
            return gy
        if src == 'gz':
            return gz
        return None

    def _update_axis(
        self,
        m: AxisMapping,
        roll, pitch, gx, gy, gz,
        dt: float,
        prev: float,
    ) -> tuple[float, float]:
        angle_meas = self._pick(m.angle_source, roll, pitch, gx, gy, gz)
        rate = self._pick(m.rate_source, roll, pitch, gx, gy, gz)
        omega = 0.0 if rate is None else m.rate_sign * rate

        if angle_meas is None:
            # 无倾角仪基准 (尚未就绪) → 退化为纯陀螺积分
            theta = prev + omega * dt
        else:
            theta_meas = m.angle_sign * angle_meas
            gyro_angle = prev + omega * dt
            theta = self.alpha * gyro_angle + (1.0 - self.alpha) * theta_meas
        return theta, omega

    def update(
        self,
        roll: float | None,
        pitch: float | None,
        gx: float | None,
        gy: float | None,
        gz: float | None,
        dt: float | None,
    ) -> dict:
        """用最新倾角仪 roll/pitch 与陀螺三轴角速度更新估计。

        Args:
            roll/pitch: 倾角仪角度 [rad] (可能为 None)
            gx/gy/gz:   陀螺三轴角速度 [rad/s] (可能为 None)
            dt:         与上次调用间隔 [s] (None/非正时按 10ms 兜底)

        Returns:
            {'theta_x', 'theta_y', 'omega_x', 'omega_y'} [rad, rad/s]
        """
        if dt is None or dt <= 0.0:
            dt = 0.01
        self._theta_x, self._omega_x = self._update_axis(
            self.mapping.theta_x, roll, pitch, gx, gy, gz, dt, self._theta_x
        )
        self._theta_y, self._omega_y = self._update_axis(
            self.mapping.theta_y, roll, pitch, gx, gy, gz, dt, self._theta_y
        )
        return self.state

    @property
    def state(self) -> dict:
        return {
            'theta_x': self._theta_x,
            'theta_y': self._theta_y,
            'omega_x': self._omega_x,
            'omega_y': self._omega_y,
        }

    def reset(self) -> None:
        self._theta_x = 0.0
        self._theta_y = 0.0
        self._omega_x = 0.0
        self._omega_y = 0.0
