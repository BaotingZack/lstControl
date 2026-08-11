"""位置到速度控制器: 位置目标 → 速度指令

控制结构:
  输入: target_position, measured_position, filtered_velocity
  误差: e = target_position - measured_position
  输出: v_cmd = Kp * e + Ki * ∫e dt - Kd * filtered_velocity
  限幅: ±v_max

这里没有 S 曲线位置/速度前馈。PLC 只知道调度系统发来的最终目标位置，
再根据位置误差和滤波后的差分速度生成速度模式伺服的速度设定值。

积分项 (Ki) 默认关闭 (ki_pos=0), 保持纯 PD 语义与既有现场调参结果兼容;
仅在显式传入 ki_pos>0 时启用, 用于消除纯 PD 在电机死区/机械阻力下的
稳态残余误差 (现场实测: 纯 PD 收敛到目标附近后, 由于 D 项已把速度压
到接近零, 即使残余位置误差仍有 2~3cm, 计算出的速度指令也会小到不足
以克服电机/传动死区, 导致误差无法继续收敛)。
"""

from __future__ import annotations


class PositionPDController:
    """位置目标到速度指令的控制器 (PD, 可选叠加限幅积分项即 PID)。

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

    积分项 (可选, 默认关闭):
      - ki_pos:              积分增益; 0 表示禁用 (纯 PD, 向后兼容)。
      - integral_band:       仅在 |位置误差| 小于该值时才开始积分累加,
                              离目标还远时 (P/D 已饱和限幅) 不积分——
                              避免长距离行程期间积分饱和(windup), 只在
                              精定位阶段用积分"顶开"死区残余误差。
      - integral_output_limit: 积分项对速度指令的贡献上限 [m/s] (0=不限),
                              防止积分项本身造成过冲/反向抽动。
    """

    def __init__(
        self,
        kp_pos: float = 0.5,
        kd_pos: float = 0.35,
        v_max: float = 0.3,
        position_deadband: float = 0.0,
        reverse_tol: float = 0.0,
        ki_pos: float = 0.0,
        integral_band: float = 0.0,
        integral_output_limit: float = 0.0,
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
        if ki_pos < 0:
            raise ValueError('ki_pos must be non-negative')
        if integral_band < 0:
            raise ValueError('integral_band must be non-negative')
        if integral_output_limit < 0:
            raise ValueError('integral_output_limit must be non-negative')
        self.kp_pos = kp_pos
        self.kd_pos = kd_pos
        self.v_max = v_max
        self.position_deadband = position_deadband
        self.reverse_tol = reverse_tol
        self.ki_pos = ki_pos
        self.integral_band = integral_band
        self.integral_output_limit = integral_output_limit
        self._integral = 0.0

    def reset(self):
        """重置积分状态 (每次 run_pd_control 调用都会新建控制器实例,
        这里主要用于同一控制器实例被跨阶段复用的场景)。"""
        self._integral = 0.0

    def update(
        self,
        target_position: float,
        measured_position: float,
        measured_velocity: float | None = None,
        dt: float | None = None,
    ) -> float:
        """计算速度指令。

        Args:
            target_position: 当前阶段目标位置 [m]
            measured_position: 编码器反馈位置 [m]
            measured_velocity: 速度反馈 [m/s]（原生速度优先，缺失时用滤波差分速度）
            dt: 本次调用与上次调用的时间间隔 [s]；仅在启用积分项 (ki_pos>0)
                时需要，用于积分累加。缺省 (None) 时积分项不生效。

        Returns:
            速度指令 [m/s]
        """
        position_error = target_position - measured_position

        # 位置死区: 已进入到位窗口则不再输出速度，消除锁定前的抖动/换向脉冲。
        if self.position_deadband > 0.0 and abs(position_error) < self.position_deadband:
            self._integral = 0.0
            return 0.0

        integral_term = 0.0
        if self.ki_pos > 0.0 and dt is not None and dt > 0.0:
            if self.integral_band > 0.0 and abs(position_error) < self.integral_band:
                self._integral += position_error * dt
                if self.integral_output_limit > 0.0:
                    # 反算积分状态的等效限幅, 避免饱和后的"隐藏 windup"
                    # 在残余误差突然变小时才暴露出一个过大的积分冲量。
                    state_limit = self.integral_output_limit / self.ki_pos
                    self._integral = max(-state_limit, min(state_limit, self._integral))
            else:
                # 误差还远未进入精定位窗口 (P/D 项通常已在限幅饱和):
                # 清空积分, 避免长距离行程期间积分饱和 (windup)。
                self._integral = 0.0
            integral_term = self.ki_pos * self._integral
            if self.integral_output_limit > 0.0:
                integral_term = max(
                    -self.integral_output_limit, min(self.integral_output_limit, integral_term)
                )

        damping = self.kd_pos * measured_velocity if measured_velocity is not None else 0.0
        v_cmd = self.kp_pos * position_error + integral_term - damping
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
