"""
位置到速度控制器: 位置目标 → 速度指令

控制结构:
  输入: target_position, measured_position, filtered_velocity
  误差: e = target_position - measured_position
  输出: v_cmd = Kp * e - Kd * filtered_velocity
  限幅: ±v_max

这里没有 S 曲线位置/速度前馈。PLC 只知道调度系统发来的最终目标位置，
再根据位置误差和滤波后的差分速度生成速度模式伺服的速度设定值。
"""

from __future__ import annotations


class PositionPDController:
    """位置目标到速度指令的控制器。

    D 项使用速度反馈做阻尼。反馈速度优先使用位置源提供的原生速度
    (如 Odometry twist)，缺失时才退回到位置差分后的低通滤波速度——
    因为对 10Hz 量化定位做差分会引入较大噪声，直接进入 D 项会让
    速度指令在目标附近来回抖动、频繁换向。

    另外提供两项防抖动保护 (默认关闭, 保持纯 PD 语义):
      - position_deadband: 位置误差进入到位窗口后指令直接归零，
        消除锁定前的微幅蠕动与换向脉冲。
      - reverse_tol:       防反向抽动。速度伺服型行车"刹车"应是指令
        归零而非反向脉冲；除非确实越过目标 (|误差| >= reverse_tol)，
        否则禁止朝远离目标方向给速度，避免机械冲击与来回蠕动。
    """

    def __init__(
        self,
        kp_pos: float = 0.5,
        kd_pos: float = 0.35,
        v_max: float = 0.3,
        position_deadband: float = 0.0,
        reverse_tol: float = 0.0,
    ):
        if v_max <= 0:
            raise ValueError('v_max must be positive')
        if kp_pos < 0:
            raise ValueError('kp_pos must be non-negative')
        if kd_pos < 0:
            raise ValueError('kd_pos must be non-negative')
        if position_deadband < 0:
            raise ValueError('position_deadband must be non-negative')
        if reverse_tol < 0:
            raise ValueError('reverse_tol must be non-negative')
        self.kp_pos = kp_pos
        self.kd_pos = kd_pos
        self.v_max = v_max
        self.position_deadband = position_deadband
        self.reverse_tol = reverse_tol

    def reset(self):
        """保留状态重置接口，当前 PD 控制器无内部积分状态。"""
        pass

    def update(
        self,
        target_position: float,
        measured_position: float,
        measured_velocity: float | None = None,
    ) -> float:
        """计算速度指令。

        Args:
            target_position: 当前阶段目标位置 [m]
            measured_position: 编码器反馈位置 [m]
            measured_velocity: 速度反馈 [m/s]（原生速度优先，缺失时用滤波差分速度）

        Returns:
            速度指令 [m/s]
        """
        position_error = target_position - measured_position

        # 位置死区: 已进入到位窗口则不再输出速度，消除锁定前的抖动/换向脉冲。
        if self.position_deadband > 0.0 and abs(position_error) < self.position_deadband:
            return 0.0

        damping = self.kd_pos * measured_velocity if measured_velocity is not None else 0.0
        v_cmd = self.kp_pos * position_error - damping
        v_cmd = max(-self.v_max, min(self.v_max, v_cmd))

        # 防反向抽动: 在尚未越过目标 (|误差| < reverse_tol) 时，禁止给出
        # 与位置误差方向相反的速度指令——此类指令来自 D 项过冲或速度噪声，
        # 会让行车反向抽动。朝目标方向(含真正过冲后的回拉)始终允许。
        if (
            self.reverse_tol > 0.0
            and position_error * v_cmd < 0.0
            and abs(position_error) < self.reverse_tol
        ):
            v_cmd = 0.0

        return v_cmd


# 兼容旧导入名。新代码应使用 PositionPDController。
VelocityController = PositionPDController
