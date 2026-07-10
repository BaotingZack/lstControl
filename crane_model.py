"""
起重机运动学模型: 轴状态、三轴起重机状态、控制配置参数

坐标系定义 (类似港口集装箱起重机):
  X — 大行车 (bridge),   沿跑道/导轨方向运动
  Y — 小行车 (trolley),  在大行车桥架上横向运动
  Z — 抓钩   (hoist),   升降方向, Z=0 为地面, Z>0 为上方
  X-Y 构成水平面, 抓钩在 Z 轴上下移动
"""

from dataclasses import dataclass
import math
import random


@dataclass
class AxisState:
    """单轴运动学状态

    Attributes:
        position: 当前位置 [m]
        velocity: 当前速度 [m/s], 正负号表示方向
    """
    position: float = 0.0
    velocity: float = 0.0


class CraneState:
    """三轴桥式起重机实时状态

    维护大行车(X)、小行车(Y)、抓钩(Z) 三个独立轴的位置和速度。
    各轴运动学独立——速度正方向:
      X: 大行车前进方向
      Y: 小行车前进方向
      Z: 向上 (远离地面)
    """

    def __init__(self, x0: float = 0.0, y0: float = 0.0, z0: float = 5.0):
        """
        Args:
            x0: 大行车初始位置 [m]
            y0: 小行车初始位置 [m]
            z0: 抓钩初始高度 [m] (默认 5m, 安全高度之上)
        """
        self.x = AxisState(position=x0)
        self.y = AxisState(position=y0)
        self.z = AxisState(position=z0)

    @staticmethod
    def _update_axis(axis: AxisState, accel: float, dt: float):
        """单轴运动学积分: 欧拉法

        v(t+dt) = v(t) + a·dt
        p(t+dt) = p(t) + v(t+dt)·dt

        Args:
            axis:  要更新的轴状态
            accel: 加速度指令 [m/s²]
            dt:    仿真步长 [s]
        """
        axis.velocity += accel * dt
        axis.position += axis.velocity * dt

    def update(self, ax: float, ay: float, az: float, dt: float):
        """三轴同步运动学更新

        Args:
            ax, ay, az: 各轴加速度指令 [m/s²]
            dt:         仿真步长 [s]
        """
        self._update_axis(self.x, ax, dt)
        self._update_axis(self.y, ay, dt)
        self._update_axis(self.z, az, dt)

    def snapshot(self) -> dict:
        """返回当前状态的快照字典，便于日志记录"""
        return {
            'x': self.x.position, 'y': self.y.position, 'z': self.z.position,
            'vx': self.x.velocity, 'vy': self.y.velocity, 'vz': self.z.velocity,
        }


class CranePlant:
    """速度模式伺服 + 机械扰动的简化被控对象。

    PLC 输出速度指令，真实轴不会瞬时跟随；这里用一阶惯性近似伺服速度环，
    再叠加低频外扰和测量噪声，让仿真更接近现场行车。
    """

    def __init__(self, config):
        self.config = config
        self._rng = random.Random(config.disturbance_seed)
        self._phase = {
            'x': self._rng.uniform(0.0, math.tau),
            'y': self._rng.uniform(0.0, math.tau),
            'z': self._rng.uniform(0.0, math.tau),
        }

    def measure_position(self, axis_name: str, position: float) -> float:
        """返回带测量噪声的位置反馈。"""
        noise_std = self._noise_std(axis_name)
        if not self.config.enable_disturbance or noise_std <= 0:
            return position
        return position + self._rng.gauss(0.0, noise_std)

    def update_axis(
        self,
        axis_name: str,
        axis: AxisState,
        velocity_cmd: float,
        target: float,
        locked: bool,
        dt: float,
        t: float,
    ) -> float:
        """根据速度指令更新单轴状态，返回本步外扰速度。"""
        if locked:
            axis.position = target
            axis.velocity = 0.0
            return 0.0

        tau = self._servo_tau(axis_name)
        alpha = 1.0 if tau <= dt else 1.0 - math.exp(-dt / tau)
        servo_velocity = axis.velocity + alpha * (velocity_cmd - axis.velocity)
        disturbance = self.velocity_disturbance(axis_name, t)
        axis.velocity = self._clamp_velocity(axis_name, servo_velocity + disturbance)
        axis.position += axis.velocity * dt
        return disturbance

    def velocity_disturbance(self, axis_name: str, t: float) -> float:
        """低频风载/负载摆动等效速度扰动。"""
        amp = self._disturbance_amp(axis_name)
        if not self.config.enable_disturbance or amp <= 0:
            return 0.0
        phase = self._phase[axis_name]
        gust = (
            0.58 * math.sin(0.37 * t + phase)
            + 0.32 * math.sin(0.11 * t + phase * 0.47)
            + 0.10 * math.sin(1.70 * t + phase * 1.31)
        )
        return amp * gust

    def _servo_tau(self, axis_name: str) -> float:
        return self.config.servo_time_constant_z if axis_name == 'z' else self.config.servo_time_constant_xy

    def _disturbance_amp(self, axis_name: str) -> float:
        return self.config.disturbance_velocity_z if axis_name == 'z' else self.config.disturbance_velocity_xy

    def _noise_std(self, axis_name: str) -> float:
        return self.config.measurement_noise_z if axis_name == 'z' else self.config.measurement_noise_xy

    def _clamp_velocity(self, axis_name: str, velocity: float) -> float:
        v_max = self.config.max_velocity_z if axis_name == 'z' else self.config.max_velocity_xy
        limit = v_max * 1.08
        return max(-limit, min(limit, velocity))


@dataclass
class CraneConfig:
    """起重机控制配置参数 (所有参数均可在构造时覆盖)

    —— S曲线轨迹参数 ——
    - max_velocity_xy: 大小行车最大速度 [m/s], 默认 0.3
    - max_velocity_z:  抓钩最大升降速度 [m/s], 默认 0.2
    - max_acceleration_xy / _z: 最大加速度 [m/s²]
    - max_jerk_xy / _z: 最大 jerk (加加速度) [m/s³]

    —— PLC 速度控制器参数 ——
    - kp_pos: 位置环比例增益 [1/s], 将位置误差转换为速度修正量

    —— 作业参数 ——
    - safe_height_offset: 安全高度偏移 [m], 在目标 Z 之上
    - grab_delay / release_delay: 抓取/释放延时 [s]

    —— 仿真参数 ——
    - dt: 仿真步长 [s]
    - arrival_pos_tol / arrival_vel_tol: 到达判断容差 [m] / [m/s]
    """

    # --- S曲线参数 ---
    max_velocity_xy: float = 0.3       # [m/s] 大小行车最大速度
    max_velocity_z: float = 0.2        # [m/s] 抓钩最大升降速度
    max_acceleration_xy: float = 0.2   # [m/s²]
    max_acceleration_z: float = 0.15   # [m/s²]
    max_jerk_xy: float = 0.15          # [m/s³]
    max_jerk_z: float = 0.1            # [m/s³]

    # --- PLC 速度控制器参数 ---
    kp_pos: float = 0.5                # 位置环比例增益 [1/s]
    kd_pos: float = 0.35               # 速度阻尼增益, 用于 PD 的 D 项
    velocity_filter_tau: float = 0.25  # [s] SLAM/差分速度低通滤波时间常数

    # --- 伺服/扰动模型 ---
    servo_time_constant_xy: float = 0.18   # [s] XY 速度环一阶响应时间常数
    servo_time_constant_z: float = 0.12    # [s] Z 速度环一阶响应时间常数
    enable_disturbance: bool = True        # 是否启用外扰和测量噪声
    disturbance_seed: int = 7              # 扰动随机种子, 保证仿真可复现
    disturbance_velocity_xy: float = 0.006 # [m/s] XY 低频等效速度扰动幅值
    disturbance_velocity_z: float = 0.003  # [m/s] Z 低频等效速度扰动幅值
    measurement_noise_xy: float = 0.0005   # [m] XY 位置反馈测量噪声标准差
    measurement_noise_z: float = 0.0003    # [m] Z 位置反馈测量噪声标准差

    # --- 作业参数 ---
    safe_height_offset: float = 1.0    # [m] 安全高度偏移量
    grab_delay: float = 0.5            # [s] 抓取延时
    release_delay: float = 0.5         # [s] 释放延时

    # --- 仿真参数 ---
    dt: float = 0.01                   # [s] 仿真步长
    arrival_pos_tol: float = 0.01      # [m] 到达判断位置容差
    arrival_vel_tol: float = 0.005     # [m/s] 到达判断速度容差
    velocity_deadband: float = 0.01    # [m/s] 零速阈值, |v|低于此值+到位 → 强制归零
                                        #       模拟伺服驱动 zero-speed 窗口行为
    arrival_capture_pos_tol: float = 0.02  # [m] 到位捕获位置窗口
    arrival_cmd_tol: float = 0.015         # [m/s] 到位捕获速度指令窗口

    def __post_init__(self):
        positive_fields = {
            'max_velocity_xy': self.max_velocity_xy,
            'max_velocity_z': self.max_velocity_z,
            'max_acceleration_xy': self.max_acceleration_xy,
            'max_acceleration_z': self.max_acceleration_z,
            'max_jerk_xy': self.max_jerk_xy,
            'max_jerk_z': self.max_jerk_z,
            'dt': self.dt,
            'servo_time_constant_xy': self.servo_time_constant_xy,
            'servo_time_constant_z': self.servo_time_constant_z,
            'velocity_filter_tau': self.velocity_filter_tau,
        }
        for name, value in positive_fields.items():
            if value <= 0:
                raise ValueError(f'{name} must be positive')

        non_negative_fields = {
            'kp_pos': self.kp_pos,
            'kd_pos': self.kd_pos,
            'safe_height_offset': self.safe_height_offset,
            'grab_delay': self.grab_delay,
            'release_delay': self.release_delay,
            'arrival_pos_tol': self.arrival_pos_tol,
            'arrival_vel_tol': self.arrival_vel_tol,
            'velocity_deadband': self.velocity_deadband,
            'arrival_capture_pos_tol': self.arrival_capture_pos_tol,
            'arrival_cmd_tol': self.arrival_cmd_tol,
            'disturbance_velocity_xy': self.disturbance_velocity_xy,
            'disturbance_velocity_z': self.disturbance_velocity_z,
            'measurement_noise_xy': self.measurement_noise_xy,
            'measurement_noise_z': self.measurement_noise_z,
        }
        for name, value in non_negative_fields.items():
            if value < 0:
                raise ValueError(f'{name} must be non-negative')
