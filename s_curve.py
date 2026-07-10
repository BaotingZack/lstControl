"""
S曲线轨迹生成器 — jerk-limited 7段式速度剖面

S曲线 (S-Curve) 是工业伺服系统中广泛使用的轨迹规划方法,
通过限制 jerk (加加速度, 加速度的变化率) 来产生平滑的启停过程,
减少机械冲击和对吊钩的激励。

7段式剖面结构:
  T1: 加加速段  (jerk > 0, 加速度从 0 线性增大到 a_max)
  T2: 匀加速段  (jerk = 0, 加速度恒定 = a_max)
  T3: 减加速段  (jerk < 0, 加速度从 a_max 线性减小到 0)
  T4: 匀速段    (jerk = 0, 加速度 = 0, 速度恒定 = v_max)
  T5: 加减速段  (jerk < 0, 减速度从 0 线性增大)
  T6: 匀减速段  (jerk = 0, 减速度恒定)
  T7: 减减速段  (jerk > 0, 减速度线性减小到 0)

  速度 ^
  v_max|        _______
       |       /       \\
       |      /         \\
       |     /           \\
       |    /             \\
     0 |___/_______________\\_____> t
       |T1|T2|T3| T4 |T5|T6|T7|

根据目标距离自动判断:
  - 长距离: 完整 7 段 (可达 v_max 和 a_max)
  - 中距离: 无匀速段 (可达 a_max, 达不到 v_max)
  - 短距离: 纯 jerk 加减速 (达不到 a_max)

参考文献:
  - Biagiotti, L. & Melchiorri, C. "Trajectory Planning for Automatic
    Machines and Robots", Springer, 2008.
"""

import math


class SCurveProfile:
    """S曲线速度剖面生成器。

    用法:
        sc = SCurveProfile(v_max=0.3, a_max=0.2, j_max=0.15)
        sc.plan(p0=0.0, pf=8.0)       # 规划从 0 到 8m 的轨迹
        p, v, a = sc.sample(t=2.5)     # 采样 t=2.5s 时的参考值
        T = sc.total_time              # 获取总执行时间
    """

    def __init__(self, max_velocity: float, max_acceleration: float, max_jerk: float):
        """
        Args:
            max_velocity:     最大速度 [m/s]
            max_acceleration: 最大加速度 [m/s²]
            max_jerk:         最大 jerk (加加速度) [m/s³]
        """
        self.configure(max_velocity, max_acceleration, max_jerk)

    def configure(self, max_velocity: float, max_acceleration: float, max_jerk: float):
        """配置运动约束并清空上一条轨迹。"""
        if max_velocity <= 0:
            raise ValueError('max_velocity must be positive')
        if max_acceleration <= 0:
            raise ValueError('max_acceleration must be positive')
        if max_jerk <= 0:
            raise ValueError('max_jerk must be positive')

        self.v_max = max_velocity
        self.a_max = max_acceleration
        self.j_max = max_jerk

        # ---- 规划结果: 各段持续时间 [s] ----
        self._Tj: float = 0.0      # jerk 段时长 (T1 = T3 = T5 = T7)
        self._Ta: float = 0.0      # 恒加速段时长 (T2 = T6)
        self._Tv: float = 0.0      # 匀速段时长 (T4)

        # ---- 实际达到的峰值 ----
        self._a_actual: float = 0.0   # 实际最大加速度 [m/s²] (可能 < a_max)
        self._v_actual: float = 0.0   # 实际最大速度 [m/s] (可能 < v_max)

        # ---- 运动方向和总时间 ----
        self._direction: float = 1.0   # +1 正向, -1 反向
        self._total_time: float = 0.0  # 总执行时间 [s]
        self._p0: float = 0.0          # 起始位置 [m]
        return self

    # ------------------------------------------------------------------
    # 公开方法
    # ------------------------------------------------------------------

    def plan(self, p0: float, pf: float, v0: float = 0.0, vf: float = 0.0):
        """规划从 p0 到 pf 的 S曲线轨迹。

        根据位移大小自动选择剖面类型:
          - 长距离: 完整 7 段 (到达 v_max)
          - 中距离: 6 段, 无 T4 匀速段
          - 短距离: 4 段纯 jerk, 无 T2/T4/T6

        Args:
            p0: 起始位置 [m]
            pf: 目标位置 [m]
            v0: 起始速度 [m/s] (默认 0)
            vf: 终止速度 [m/s] (默认 0)

        Returns:
            self (支持链式调用)
        """
        if abs(v0) > 1e-9 or abs(vf) > 1e-9:
            raise ValueError(
                'SCurveProfile currently supports zero start/end velocity only'
            )

        D = pf - p0                                        # 总位移 [m]
        self._direction = 1.0 if D >= 0 else -1.0          # 运动方向
        D_abs = abs(D)
        self._p0 = p0

        # ---- 零位移: 空轨迹 ----
        if D_abs <= 1e-9:
            self._Tj = 0.0; self._Ta = 0.0; self._Tv = 0.0
            self._a_actual = 0.0; self._v_actual = 0.0
            self._total_time = 0.0
            return self

        # 纯 jerk 加减速 (Tj = a_max / j_max) 能达到的速度:
        #   v = j_max * Tj^2 = a_max^2 / j_max
        # 如果 v_max < 这个值, 说明即使最大加速度, 纯 jerk 加减速
        # 已经超过 v_max —— 即系统达不到 a_max
        v_from_jerk_only = self.a_max ** 2 / self.j_max   # [m/s]

        if self.v_max >= v_from_jerk_only:
            # ============================================
            # 情况 A: 可以达到 a_max
            # ============================================
            self._Tj = self.a_max / self.j_max            # jerk 段时长 [s]
            self._a_actual = self.a_max

            # 加速段终点速度: v = j_max * Tj * (Tj + Ta)
            # 设 v = v_max → Ta = v_max / (j_max * Tj) - Tj
            Ta_for_vmax = max(0.0, self.v_max / (self.j_max * self._Tj) - self._Tj)

            # 加速段总位移 (T1+T2+T3)
            p_accel = self._compute_accel_displacement(self._Tj, Ta_for_vmax)

            if D_abs >= 2 * p_accel:
                # ---- 长距离: 完整 7 段 (有匀速段) ----
                self._Ta = Ta_for_vmax
                self._v_actual = self.v_max
                p_cruise = D_abs - 2 * p_accel             # 匀速段位移
                self._Tv = p_cruise / self._v_actual
            else:
                # ---- 中距离: 无匀速段, 缩小 Ta ----
                self._Tv = 0.0
                # 二分搜索 Ta 使总位移 = D_abs
                self._Ta = self._solve_Ta_for_displacement(D_abs, self._Tj)
                self._v_actual = self.j_max * self._Tj * (self._Tj + self._Ta)

            # ---- 超短距离: Ta=0 但位移仍过大, 需缩减 Tj ----
            # 当前 Tj 的最小位移 = 2 * j_max * Tj³ (Ta=0 时纯 jerk 加减速)
            # 若 D_abs 小于此值, 说明即使用最小加速度也会飞过目标
            if self._Ta == 0.0:
                D_min_tj = 2.0 * self.j_max * self._Tj ** 3
                if D_abs < D_min_tj:
                    self._Tj = (D_abs / (2.0 * self.j_max)) ** (1.0 / 3.0)
                    self._a_actual = self.j_max * self._Tj
                    self._v_actual = self.j_max * self._Tj ** 2

        else:
            # ============================================
            # 情况 B: 达不到 a_max, 纯 jerk 加减速 (Ta=0)
            # ============================================
            self._Ta = 0.0

            # 纯 jerk 能达到的速度峰值: v_peak = j_max * Tj^2
            # 设 v_peak = v_max → Tj = sqrt(v_max / j_max)
            Tj_for_vmax = math.sqrt(self.v_max / self.j_max)

            # 纯 jerk 加减速总位移 (4 段, 无 T2/T4/T6):
            #   D = 2 * j_max * Tj^3
            D_for_vmax = 2 * self.j_max * Tj_for_vmax ** 3

            if D_abs >= D_for_vmax:
                # ---- 中长距离: 可到 v_max 但需匀速段 ----
                self._Tj = Tj_for_vmax
                self._a_actual = self.j_max * self._Tj
                self._v_actual = self.v_max
                # 纯 jerk 加速段位移 = j_max * Tj^3
                p_accel_pure_jerk = self.j_max * self._Tj ** 3
                self._Tv = (D_abs - 2 * p_accel_pure_jerk) / self._v_actual
            else:
                # ---- 短距离: 达不到 v_max ----
                # D = 2 * j_max * Tj^3 → Tj = (D / (2*j_max))^(1/3)
                self._Tj = (D_abs / (2 * self.j_max)) ** (1.0 / 3.0)
                self._a_actual = self.j_max * self._Tj
                self._v_actual = self.j_max * self._Tj ** 2
                self._Tv = 0.0

        # 总时间: 4个jerk段 + 2个恒加速段 + 匀速段
        self._total_time = 4 * self._Tj + 2 * self._Ta + self._Tv
        return self

    def sample(self, t: float) -> tuple[float, float, float]:
        """采样 t 时刻的参考轨迹值。

        Args:
            t: 相对于 plan() 调用时刻的时间 [s]

        Returns:
            (position, velocity, acceleration):
                position:     参考位置 [m] (绝对值, 含起始偏移)
                velocity:     参考速度 [m/s] (带方向符号)
                acceleration: 参考加速度 [m/s²] (带方向符号)
        """
        # ---- 边界处理 ----
        if t <= 0.0:
            return (self._p0, 0.0, 0.0)
        if t >= self._total_time:
            # 轨迹结束后, 返回终点位置, 速度为 0
            pf = self._compute_final_position()
            return (pf, 0.0, 0.0)

        Tj, Ta, Tv = self._Tj, self._Ta, self._Tv

        # jerk 带方向: 正向运动 jerk>0, 反向运动 jerk<0
        j = self.j_max * self._direction

        # ---- 各段时间节点 ----
        t1 = Tj                  # T1 结束 (加加速)
        t2 = Tj + Ta             # T2 结束 (匀加速) = T3 开始
        t3 = 2 * Tj + Ta         # T3 结束 (减加速) = 加速段结束
        t4 = t3 + Tv             # T4 结束 (匀速)
        t5 = t4 + Tj             # T5 结束 (加减速)
        t6 = t4 + Tj + Ta        # T6 结束 (匀减速) = T7 开始
        t7 = t4 + 2 * Tj + Ta    # T7 结束 (减减速) = 总时间

        # ---- 分段计算 ----
        if t <= t1:
            # T1: 加加速段 (加速度从 0 线性增大)
            tau = t
            a = j * tau                         # 线性增大
            v = j * tau ** 2 / 2.0              # 二次增长
            p = j * tau ** 3 / 6.0              # 三次增长

        elif t <= t2:
            # T2: 匀加速段 (加速度恒定 = a_max)
            tau = t - t1
            a1_end = j * Tj                     # T1 终点加速度
            v1_end = j * Tj ** 2 / 2.0          # T1 终点速度
            p1_end = j * Tj ** 3 / 6.0          # T1 终点位移
            a = a1_end
            v = v1_end + a1_end * tau
            p = p1_end + v1_end * tau + a1_end * tau ** 2 / 2.0

        elif t <= t3:
            # T3: 减加速段 (加速度从 a_max 线性减小到 0)
            tau = t - t2
            a1_end = j * Tj
            v1_end = j * Tj ** 2 / 2.0
            p1_end = j * Tj ** 3 / 6.0
            v2_end = v1_end + a1_end * Ta        # T2 终点速度
            p2_end = p1_end + v1_end * Ta + a1_end * Ta ** 2 / 2.0
            a = a1_end - j * tau
            v = v2_end + a1_end * tau - j * tau ** 2 / 2.0
            p = p2_end + v2_end * tau + a1_end * tau ** 2 / 2.0 - j * tau ** 3 / 6.0

        elif t <= t4:
            # T4: 匀速段 (加速度 = 0, 速度恒定 = v_max)
            tau = t - t3
            # 用加速段终点状态
            v_cruise = self._direction * self._v_actual   # 巡航速度 (带符号)
            p3_end = self._compute_position_at_t3()       # 加速段终点位置
            a = 0.0
            v = v_cruise
            p = p3_end + v_cruise * tau

        elif t <= t5:
            # T5: 加减速段 (减速度从 0 线性增大)
            tau = t - t4
            v_cruise = self._direction * self._v_actual
            p_cruise_end = self._compute_accel_end_pos() + v_cruise * Tv
            a = -j * tau                        # 减速方向与运动方向相反
            v = v_cruise - j * tau ** 2 / 2.0
            p = p_cruise_end + v_cruise * tau - j * tau ** 3 / 6.0

        elif t <= t6:
            # T6: 匀减速段 (减速度恒定)
            tau = t - t5
            v_cruise = self._direction * self._v_actual
            p_cruise_end = self._compute_accel_end_pos() + v_cruise * Tv
            a5_end = -j * Tj                   # T5 终点减速度
            v5_end = v_cruise - j * Tj ** 2 / 2.0
            p5_end = p_cruise_end + v_cruise * Tj - j * Tj ** 3 / 6.0
            a = a5_end
            v = v5_end + a5_end * tau
            p = p5_end + v5_end * tau + a5_end * tau ** 2 / 2.0

        else:
            # T7: 减减速段 (减速度线性减小到 0)
            tau = t - t6
            v_cruise = self._direction * self._v_actual
            p_cruise_end = self._compute_accel_end_pos() + v_cruise * Tv
            a5_end = -j * Tj
            v5_end = v_cruise - j * Tj ** 2 / 2.0
            p5_end = p_cruise_end + v_cruise * Tj - j * Tj ** 3 / 6.0
            v6_end = v5_end + a5_end * Ta
            p6_end = p5_end + v5_end * Ta + a5_end * Ta ** 2 / 2.0
            a = a5_end + j * tau                  # 减速度线性归零
            v = v6_end + a5_end * tau + j * tau ** 2 / 2.0
            p = p6_end + v6_end * tau + a5_end * tau ** 2 / 2.0 + j * tau ** 3 / 6.0

        # 返回值 = 起始偏移 + 相对位移 (位移部分已通过 j 的符号带方向)
        return (self._p0 + p, v, a)

    # ------------------------------------------------------------------
    # 属性
    # ------------------------------------------------------------------

    @property
    def total_time(self) -> float:
        """轨迹总执行时间 [s]"""
        return self._total_time

    @property
    def actual_max_velocity(self) -> float:
        """实际达到的最大速度绝对值 [m/s]"""
        return self._v_actual

    # ------------------------------------------------------------------
    # 内部辅助方法
    # ------------------------------------------------------------------

    def _compute_accel_displacement(self, Tj: float, Ta: float) -> float:
        """计算加速段 (T1+T2+T3) 的总位移 (无量纲, 不含符号)。

        Args:
            Tj: jerk 段时长 [s]
            Ta: 恒加速段时长 [s]

        Returns:
            加速段位移 [m] (正值)
        """
        j = self.j_max  # 用正值计算, 方向由外部处理
        # T1: 加加速段
        p1 = j * Tj ** 3 / 6.0                       # 三次项
        v1 = j * Tj ** 2 / 2.0                       # T1 终点速度
        a1 = j * Tj                                   # T1 终点加速度

        # T2: 匀加速段
        p2 = v1 * Ta + a1 * Ta ** 2 / 2.0
        v2 = v1 + a1 * Ta

        # T3: 减加速段
        p3 = v2 * Tj + a1 * Tj ** 2 / 2.0 - j * Tj ** 3 / 6.0

        return p1 + p2 + p3

    def _total_displacement_for_Ta(self, Tj: float, Ta: float) -> float:
        """给定 Tj 和 Ta (无匀速段), 计算总位移 [m] (正值)"""
        return 2 * self._compute_accel_displacement(Tj, Ta)

    def _solve_Ta_for_displacement(self, D_abs: float, Tj: float) -> float:
        """已知 Tj 和目标位移 D_abs (Tv=0), 二分搜索 Ta。

        Args:
            D_abs: 目标位移绝对值 [m]
            Tj:    jerk 段时长 [s]

        Returns:
            所需恒加速段时长 Ta [s]
        """
        # Ta 的上限: 达到 v_max 所需的 Ta
        Ta_max = max(0.0, self.v_max / (self.j_max * Tj) - Tj)

        # 检查 Ta=0 时位移是否已足够
        if self._total_displacement_for_Ta(Tj, 0.0) >= D_abs:
            return 0.0

        # 二分搜索
        lo, hi = 0.0, Ta_max
        for _ in range(50):
            mid = (lo + hi) / 2.0
            if self._total_displacement_for_Ta(Tj, mid) < D_abs:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2.0

    def _compute_accel_end_pos(self) -> float:
        """计算加速段终点 (T3 结束) 的相对位移 [m] (不含 _p0)。

        sample() 返回时统一加 _p0, 此处只返回带方向的相对偏移。
        """
        Tj, Ta = self._Tj, self._Ta
        p_accel = self._compute_accel_displacement(Tj, Ta)
        return self._direction * p_accel

    def _compute_position_at_t3(self) -> float:
        """T3 终点位置 (同 _compute_accel_end_pos, 兼容旧接口)"""
        return self._compute_accel_end_pos()

    def _compute_final_position(self) -> float:
        """计算轨迹终点绝对位置 [m]。

        用于 t >= total_time 时返回正确的终点值。
        """
        Tj, Ta, Tv = self._Tj, self._Ta, self._Tv
        if Tv > 0 or Ta > 0:
            # 有匀速段或恒加速段
            p_accel = self._compute_accel_displacement(Tj, Ta)
            total_disp = 2 * p_accel + self._v_actual * Tv
        else:
            # 纯 jerk, 无 T2/T4/T6
            total_disp = self._total_displacement_for_Ta(Tj, 0.0)
        return self._p0 + self._direction * total_disp
