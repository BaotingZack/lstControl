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

    D 项使用滤波速度，不直接使用抖动较大的原始差分速度。
    """

    def __init__(self, kp_pos: float = 0.5, kd_pos: float = 0.35, v_max: float = 0.3):
        if v_max <= 0:
            raise ValueError('v_max must be positive')
        if kp_pos < 0:
            raise ValueError('kp_pos must be non-negative')
        if kd_pos < 0:
            raise ValueError('kd_pos must be non-negative')
        self.kp_pos = kp_pos
        self.kd_pos = kd_pos
        self.v_max = v_max

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
            measured_velocity: 滤波后的速度反馈 [m/s]

        Returns:
            速度指令 [m/s]
        """
        position_error = target_position - measured_position
        damping = self.kd_pos * measured_velocity if measured_velocity is not None else 0.0
        v_cmd = self.kp_pos * position_error - damping
        return max(-self.v_max, min(self.v_max, v_cmd))


# 兼容旧导入名。新代码应使用 PositionPDController。
VelocityController = PositionPDController
