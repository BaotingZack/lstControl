"""摆动力学模型 — 单轴线性化变绳长摆（供离线闭环仿真与模型标定）。

方程 (小角度、变绳长、含阻尼):

    θ̈ = -(g/L)·θ - 2(L̇/L)·θ̇ - ẍ/L - 2ζ·√(g/L)·θ̇

其中:
    θ      摆角 [rad]，正 = 载荷朝 +x 偏移（与防摇方案 §5.4 约定一致）
    θ̇     摆角速度 [rad/s]
    ẍ      行车加速度 [m/s²]（输入）
    L, L̇  绳长及其变化率 [m, m/s]
    ζ      阻尼比
    g      重力加速度 [m/s²]

用途:
    1. 闭环/影子仿真中当被控对象；
    2. 用录到的摆角 θ 反标定 L_eff、ζ。
"""

from __future__ import annotations

import math


GRAVITY = 9.81


class PendulumAxis:
    """单轴线性化摆，半隐式欧拉积分。"""

    def __init__(self, L: float = 5.0, zeta: float = 0.0, g: float = GRAVITY):
        if L <= 0:
            raise ValueError('L must be positive')
        if zeta < 0:
            raise ValueError('zeta must be non-negative')
        self.L = L
        self.zeta = zeta
        self.g = g
        self.theta = 0.0
        self.theta_dot = 0.0

    @property
    def omega_n(self) -> float:
        return math.sqrt(self.g / self.L)

    @property
    def period(self) -> float:
        """无阻尼自由摆周期 T = 2π√(L/g)。"""
        return 2.0 * math.pi / self.omega_n

    def reset(self, theta: float = 0.0, theta_dot: float = 0.0) -> None:
        self.theta = theta
        self.theta_dot = theta_dot

    def step(
        self,
        cart_accel: float,
        dt: float,
        L: float | None = None,
        L_dot: float = 0.0,
    ) -> tuple[float, float, float]:
        """推进一步，返回 (theta, theta_dot, theta_ddot)。

        Args:
            cart_accel: 行车加速度 ẍ [m/s²]
            dt:         步长 [s]
            L:          本步绳长 [m]（None = 沿用当前值）
            L_dot:      绳长变化率 [m/s]
        """
        if dt <= 0:
            raise ValueError('dt must be positive')
        if L is not None:
            if L <= 0:
                raise ValueError('L must be positive')
            self.L = L
        L = self.L
        omega_n = math.sqrt(self.g / L)
        theta_ddot = (
            -(self.g / L) * self.theta
            - 2.0 * (L_dot / L) * self.theta_dot
            - cart_accel / L
            - 2.0 * self.zeta * omega_n * self.theta_dot
        )
        # 半隐式欧拉：先用新速度更新角度，稳定性更好
        self.theta_dot += theta_ddot * dt
        self.theta += self.theta_dot * dt
        return self.theta, self.theta_dot, theta_ddot
