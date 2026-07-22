"""
起重机运动学模型: 轴状态、三轴起重机状态、控制配置参数

坐标系定义 (类似港口集装箱起重机):
  X — 大行车 (bridge),   沿跑道/导轨方向运动
  Y — 小行车 (trolley),  在大行车桥架上横向运动
  Z — 抓钩   (hoist),   升降方向, Z=0 为地面, Z>0 为上方
  X-Y 构成水平面, 抓钩在 Z 轴上下移动
"""

from __future__ import annotations

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
    - max_velocity_xy: 大小行车最大速度 [m/s], 默认 0.2
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
    max_velocity_xy: float = 0.2       # [m/s] 大小行车最大速度
    max_velocity_z: float = 0.2        # [m/s] 抓钩最大升降速度
    max_acceleration_xy: float = 0.2   # [m/s²]
    max_acceleration_z: float = 0.15   # [m/s²]
    max_jerk_xy: float = 0.15          # [m/s³]
    max_jerk_z: float = 0.1            # [m/s³]

    # --- PLC 速度控制器参数 ---
    kp_pos: float = 0.5                # 位置环比例增益 [1/s]
    kd_pos: float = 0.35               # 速度阻尼增益, 用于 PD 的 D 项
    velocity_filter_tau: float = 0.25  # [s] SLAM/差分速度低通滤波时间常数（仿真模式）
    velocity_filter_tau_plc: float = 0.50  # [s] PLC 模式速度滤波时间常数（10Hz 需更大 τ 补偿大 dt）

    # --- 伺服/扰动模型 ---
    servo_time_constant_xy: float = 0.18   # [s] XY 速度环一阶响应时间常数
    servo_time_constant_z: float = 0.12    # [s] Z 速度环一阶响应时间常数
    enable_disturbance: bool = True        # 是否启用外扰和测量噪声
    disturbance_seed: int = 7              # 扰动随机种子, 保证仿真可复现
    disturbance_velocity_xy: float = 0.006  # [m/s] XY 低频等效速度扰动幅值
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
    # 零速阈值；低于此值且到位时，模拟伺服驱动的 zero-speed 窗口行为。
    velocity_deadband: float = 0.01
    arrival_capture_pos_tol: float = 0.02  # [m] 到位捕获位置窗口
    arrival_cmd_tol: float = 0.015         # [m/s] 到位捕获速度指令窗口
    # 到位判定去抖: 需连续 N 个控制周期都满足到位条件才锁轴。防止单帧定位
    # 跳变(异常值)恰好落在目标附近就把轴永久锁死, 表现为"离目标还差十几厘米
    # 就停下、PD 结束"。10Hz 下 3 帧≈0.3s。设为 1 即退化为原单帧判定。
    arrival_debounce_cycles: int = 3
    # 定位跳变门限 [m]: 单个控制周期内位置跳变超过 (max_v*dt + 该余量) 视为
    # 异常帧并丢弃, 避免异常值污染 D 项/差分速度并误触发到位。
    localization_jump_margin: float = 0.30
    # 单帧定位异常 (NaN/越界/dt 异常/跳变) 的容忍预算: 连续异常帧数不超过该值时
    # 直接丢弃并继续控制 (常见于 SLAM/网络抖动的偶发坏帧); 超过则视为真实故障,
    # 按原语义中止并安全停车。10Hz 下默认 10 帧 ≈ 1s。
    max_consecutive_bad_frames: int = 10
    # 防反向抽动保护带 [m]: 在尚未越过目标且 |误差| 小于该值时，禁止 PD 给出
    # 远离目标方向的速度指令（速度伺服型行车"刹车"=指令归零而非反向脉冲）。
    reverse_guard_tol: float = 0.05

    # --- 可选机械工作区 ---
    # 现场行程因设备而异，X/Y 不提供虚假的默认范围；PLC 部署时应显式配置。
    # 坐标零点由现场约定；未配置 workspace_z_bounds 时不擅自限制 Z 的正负。
    workspace_x_bounds: tuple[float, float] | None = None
    workspace_y_bounds: tuple[float, float] | None = None
    workspace_z_bounds: tuple[float, float] | None = None

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
            'velocity_filter_tau_plc': self.velocity_filter_tau_plc,
        }
        for name, value in positive_fields.items():
            if not math.isfinite(value) or value <= 0:
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
            'reverse_guard_tol': self.reverse_guard_tol,
            'localization_jump_margin': self.localization_jump_margin,
            'disturbance_velocity_xy': self.disturbance_velocity_xy,
            'disturbance_velocity_z': self.disturbance_velocity_z,
            'measurement_noise_xy': self.measurement_noise_xy,
            'measurement_noise_z': self.measurement_noise_z,
        }
        for name, value in non_negative_fields.items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be non-negative')

        if int(self.arrival_debounce_cycles) != self.arrival_debounce_cycles \
                or self.arrival_debounce_cycles < 1:
            raise ValueError('arrival_debounce_cycles must be an integer >= 1')
        self.arrival_debounce_cycles = int(self.arrival_debounce_cycles)

        if int(self.max_consecutive_bad_frames) != self.max_consecutive_bad_frames \
                or self.max_consecutive_bad_frames < 0:
            raise ValueError('max_consecutive_bad_frames must be a non-negative integer')
        self.max_consecutive_bad_frames = int(self.max_consecutive_bad_frames)

        for axis_name, bounds in (
            ('X', self.workspace_x_bounds),
            ('Y', self.workspace_y_bounds),
            ('Z', self.workspace_z_bounds),
        ):
            if bounds is None:
                continue
            if len(bounds) != 2:
                raise ValueError(f'{axis_name} workspace bounds must contain min and max')
            lower, upper = bounds
            if not math.isfinite(lower) or not math.isfinite(upper):
                raise ValueError(f'{axis_name} workspace bounds must be finite')
            if lower > upper:
                raise ValueError(f'{axis_name} workspace minimum must not exceed maximum')

    def validate_target(
        self,
        target_pos: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """Validate and normalize a dispatch target before motion is enabled."""
        return self._validate_coordinates(target_pos, label='target')

    def validate_position(
        self,
        position: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """Validate a localization position against physical constraints."""
        return self._validate_coordinates(position, label='position')

    def _validate_coordinates(
        self,
        coordinates: tuple[float, float, float],
        *,
        label: str,
    ) -> tuple[float, float, float]:
        if len(coordinates) != 3:
            raise ValueError(f'{label} must contain exactly X, Y, and Z')

        values = tuple(float(value) for value in coordinates)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f'{label} coordinates must be finite numbers')
        for axis_name, value, bounds in (
            ('X', values[0], self.workspace_x_bounds),
            ('Y', values[1], self.workspace_y_bounds),
            ('Z', values[2], self.workspace_z_bounds),
        ):
            if bounds is not None and not bounds[0] <= value <= bounds[1]:
                raise ValueError(
                    f'{axis_name} {label} {value} is outside workspace '
                    f'[{bounds[0]}, {bounds[1]}]'
                )
        return values


# ============================================================================
# 策略模式接口 — 统一 PD 控制循环的抽象
# ============================================================================

class PositionSource:
    """位置反馈源抽象接口。

    不同模式提供不同实现:
      - SimPositionSource  — 仿真对象模型 (100 Hz 非阻塞)
      - RosPositionSource  — ROS /localization_pose (10 Hz 阻塞等待)

    返回的 dict 必须包含:
      x, y, z:        位置 [m]
      vx, vy, vz:     速度 [m/s] (可能为 None)
      dt:             本次与上次测量的时间间隔 [s]
      t:              时间戳 [s]
      stamp:          唯一标识 (用于检测新数据)
    """

    def get_position(self) -> dict | None:
        """获取最新位置反馈。返回 None 表示超时/不可恢复错误。"""
        raise NotImplementedError

    def reset(self) -> None:
        """重置内部状态 (用于新的控制运行)。"""
        pass


class Actuator:
    """执行器抽象接口。

    不同模式提供不同实现:
      - PlantActuator  — 仿真伺服 + 扰动模型
      - PlcActuator    — 真实 PLC 指令 (libsscarctrl)
    """

    def apply(self, vx: float, vy: float, vz: float, dt: float) -> None:
        """发送速度指令到被控对象。"""
        raise NotImplementedError

    def update_state(self, state: CraneState, position: dict) -> None:
        """从位置反馈更新 CraneState。

        仿真模式: 位置来自 plant model (已在 apply 中更新)
        PLC 模式: 位置来自 /localization_pose
        """
        raise NotImplementedError

    def emergency_stop(self) -> None:
        """安全停止 — 立即发送零速度指令。"""
        pass

    def stop_motion(self) -> None:
        """正常完成后停止运动，同时保持安全的位置设定。"""
        self.emergency_stop()

    def cleanup(self) -> None:
        """控制运行结束后的清理。"""
        pass


class ControlHooks:
    """控制循环回调钩子 — 用于实时推送、日志、外部中断。

    所有方法都是可选的 (默认空实现)。子类只需覆盖需要的方法。
    """

    def on_step(self, step_data: dict) -> None:
        """每步 PD 计算完成后调用。

        step_data 包含: t, x, y, z, vx, vy, vz, vx_cmd, vy_cmd, vz_cmd,
                        vx_raw, vx_filtered, target_x, target_y, target_z
        """
        pass

    def on_arrival(self, axis: str, t: float) -> None:
        """某轴到达目标时调用。"""
        pass

    def should_stop(self) -> bool:
        """返回 True 中止控制循环。每步检查。"""
        return False


# ============================================================================
# 仿真模式实现
# ============================================================================

class SimPositionSource(PositionSource):
    """仿真位置源 — 从 CranePlant 获取带噪声的测量位置。

    100 Hz 非阻塞: get_position() 始终立即返回。
    """

    def __init__(self, plant: CranePlant, state: CraneState, config: CraneConfig):
        self._plant = plant
        self._state = state
        self._config = config
        self._t = 0.0
        self._stamp = 0

    def get_position(self) -> dict:
        self._stamp += 1
        self._t += self._config.dt
        return {
            'x': self._plant.measure_position('x', self._state.x.position),
            'y': self._plant.measure_position('y', self._state.y.position),
            'z': self._plant.measure_position('z', self._state.z.position),
            'vx': None,   # 仿真模式 PD 使用差分速度
            'vy': None,
            'vz': None,
            'dt': self._config.dt,
            't': self._t,
            'stamp': self._stamp,
        }

    def reset(self) -> None:
        self._t = 0.0
        self._stamp = 0


class PlantActuator(Actuator):
    """仿真执行器 — 一阶伺服模型 + 低频扰动。

    调用 apply() 后, CraneState 已更新为最新的伺服响应状态。
    """

    def __init__(self, plant: CranePlant, state: CraneState, config: CraneConfig):
        self._plant = plant
        self._state = state
        self._config = config

    def apply(self, vx: float, vy: float, vz: float, dt: float) -> None:
        """通过仿真 plant model 执行速度指令，更新 state。

        返回的扰动值通过 step_data 传出 (在 run_pd_control 中记录)。
        """
        # 将在 run_pd_control 中调用 plant.update_axis()，这里不用做
        pass

    def update_state(self, state: CraneState, position: dict) -> None:
        """仿真模式: 位置已在 plant.update_axis() 中更新，这里为空。"""
        pass  # state 已通过 plant.update_axis() 更新

    def emergency_stop(self) -> None:
        for axis_name in ('x', 'y', 'z'):
            axis = getattr(self._state, axis_name)
            axis.velocity = 0.0

    def stop_motion(self) -> None:
        self.emergency_stop()


# ============================================================================
# 统一 PD 控制循环
# ============================================================================


class ControlRunError(RuntimeError):
    """Base class for an abnormal control-loop termination."""


class ControlStoppedError(ControlRunError):
    """Raised when an operator or external hook stops a control run."""


class PositionFeedbackTimeout(ControlRunError):
    """Raised when the position source stops delivering fresh measurements."""


class PositionFeedbackError(ControlRunError):
    """Raised when a position measurement is unsafe or malformed."""


def _validate_position_feedback(position: dict, config: CraneConfig) -> dict:
    """Return normalized feedback or fail before it reaches the controller."""
    try:
        normalized = dict(position)
        normalized['x'], normalized['y'], normalized['z'] = config.validate_position(
            (position['x'], position['y'], position['z'])
        )
        normalized['dt'] = float(position['dt'])
        normalized['t'] = float(position['t'])
        if not math.isfinite(normalized['dt']) or normalized['dt'] <= 0.0:
            raise ValueError('dt must be a finite positive number')
        if not math.isfinite(normalized['t']) or normalized['t'] < 0.0:
            raise ValueError('t must be a finite non-negative number')
        for velocity_name in ('vx', 'vy', 'vz'):
            velocity = position.get(velocity_name)
            if velocity is not None:
                velocity = float(velocity)
                if not math.isfinite(velocity):
                    raise ValueError(f'{velocity_name} must be finite')
            normalized[velocity_name] = velocity
        return normalized
    except (KeyError, TypeError, ValueError) as exc:
        raise PositionFeedbackError(f'invalid position feedback: {exc}') from exc


def _axis_arrived(axis, target: float, config: CraneConfig) -> bool:
    """判断单轴是否已到达目标 (位置 + 速度双重判定)。"""
    pos_ok = abs(axis.position - target) < config.arrival_pos_tol
    vel_ok = abs(axis.velocity) < config.arrival_vel_tol
    return pos_ok and vel_ok


def run_pd_control(
    source: PositionSource,
    actuator: Actuator,
    config: CraneConfig,
    target_pos: tuple[float, float, float],
    initial_state: CraneState,
    hooks: ControlHooks | None = None,
    max_time: float | None = 180.0,
    verbose: bool = True,
    is_simulation: bool = True,
) -> tuple[list[dict], list[tuple[float, str]]]:
    """统一 PD 位置控制循环 — 仿真和 PLC 模式共用。

    控制律:
      v_cmd = Kp * (target - measured) - Kd * filtered_velocity

    仿真模式: SimPositionSource (100 Hz 非阻塞) + PlantActuator
    PLC 模式: RosPositionSource (10 Hz 阻塞) + PlcActuator

    Args:
        source:         位置反馈源
        actuator:       执行器
        config:         控制配置参数
        target_pos:     目标位置 (tx, ty, tz) [m]
        initial_state:  初始 CraneState (会被复制, 不修改原对象)
        hooks:          可选回调钩子
        max_time:       最大运行时间 [s]
        verbose:        是否打印日志
        is_simulation:  True=仿真模式 (虚拟时间), False=PLC 模式 (真实时间)

    Returns:
        (history, arrival_events):
          history:        每步状态的 list[dict]
          arrival_events: 各轴到达时间的 list[tuple[float, str]]
    """
    from pd_controller import PositionPDController
    from velocity_filter import LowPassVelocityEstimator

    # 复制初始状态
    crane = CraneState(
        x0=initial_state.x.position,
        y0=initial_state.y.position,
        z0=initial_state.z.position,
    )
    crane.x.velocity = initial_state.x.velocity
    crane.y.velocity = initial_state.y.velocity
    crane.z.velocity = initial_state.z.velocity

    target_x, target_y, target_z = config.validate_target(target_pos)

    # 根据模式选择速度滤波时间常数
    filter_tau = config.velocity_filter_tau if is_simulation else config.velocity_filter_tau_plc

    # PD 控制器 — 两种模式完全共用
    # 到位窗口内指令归零 (position_deadband) + 防反向抽动 (reverse_tol)，
    # 抑制 10Hz 定位噪声在目标附近引起的速度指令换向。
    controllers = {
        'x': PositionPDController(
            config.kp_pos, config.kd_pos, config.max_velocity_xy,
            position_deadband=config.arrival_pos_tol,
            reverse_tol=config.reverse_guard_tol,
        ),
        'y': PositionPDController(
            config.kp_pos, config.kd_pos, config.max_velocity_xy,
            position_deadband=config.arrival_pos_tol,
            reverse_tol=config.reverse_guard_tol,
        ),
        'z': PositionPDController(
            config.kp_pos, config.kd_pos, config.max_velocity_z,
            position_deadband=config.arrival_pos_tol,
            reverse_tol=config.reverse_guard_tol,
        ),
    }

    # 速度滤波器 — 共用, PLC 模式使用更大的 tau
    filters = {
        'x': LowPassVelocityEstimator(filter_tau, crane.x.position, crane.x.velocity),
        'y': LowPassVelocityEstimator(filter_tau, crane.y.position, crane.y.velocity),
        'z': LowPassVelocityEstimator(filter_tau, crane.z.position, crane.z.velocity),
    }

    history: list[dict] = []
    arrival_events: list[tuple[float, str]] = []
    locked = {'x': False, 'y': False, 'z': False}
    # 到位去抖计数: 连续满足到位条件的周期数, 出现一帧不满足即清零。
    arrival_streak = {'x': 0, 'y': 0, 'z': 0}
    # 上一帧被接受的测量位置 (用于定位跳变/异常帧检测)。
    last_accepted_pos: dict[str, float] | None = None
    # 连续被丢弃的异常帧计数 (NaN/越界/dt异常/跳变), 用于区分偶发噪声与真实故障。
    consecutive_bad_frames = 0

    # PD 输出
    vx_cmd = vy_cmd = vz_cmd = 0.0
    vx_raw = vy_raw = vz_raw = 0.0
    vx_filtered = vy_filtered = vz_filtered = 0.0

    # 扰动记录 (仅仿真模式非零)
    disturbance_x = disturbance_y = disturbance_z = 0.0

    # 位置测量值
    x_measured = crane.x.position
    y_measured = crane.y.position
    z_measured = crane.z.position

    mode_label = 'PLC' if not is_simulation else '仿真'
    if verbose:
        print(f"=== 起重机 PD 速度控制{mode_label} ===")
        print(f"初始位置: X={crane.x.position:.2f}, Y={crane.y.position:.2f}, Z={crane.z.position:.2f}")
        print(f"目标位置: X={target_x:.2f}, Y={target_y:.2f}, Z={target_z:.2f}")
        print("=" * 50)

    source.reset()

    # Rebind source/actuator to the copied crane state
    # (SimPositionSource/PlantActuator hold a reference to the original
    # initial_state, but the loop updates crane — the copy.)
    if hasattr(source, '_state'):
        source._state = crane
    if hasattr(actuator, '_state'):
        actuator._state = crane

    # Z 目标高度注入执行器 (抓钩高度参考系)。PLC 的 liftctrl 是绝对位置伺服,
    # 执行器据此让 Z 设定值单调逼近目标, 使 Z 真正由 PD 驱动到位。
    if hasattr(actuator, 'set_z_target'):
        actuator.set_z_target(target_z)

    completed = False
    try:
        while not all(locked.values()):
            # --- 检查钩子中断 ---
            if hooks is not None and hooks.should_stop():
                raise ControlStoppedError(f'{mode_label} control stopped by external request')

            # --- STEP 1: 获取位置反馈 ---
            pos = source.get_position()
            if pos is None:
                raise PositionFeedbackTimeout(
                    f'{mode_label} position feedback timed out'
                )

            # STOP 可能在阻塞等待定位数据期间到达；下发新指令前必须再次检查。
            if hooks is not None and hooks.should_stop():
                raise ControlStoppedError(f'{mode_label} control stopped by external request')

            # --- 单帧异常值容忍 (仅 PLC/真实定位) ---
            # SLAM/网络抖动会偶尔送来一帧 NaN/越界/dt 异常, 或一次幅度离谱的跳变。
            # 若每次都让整段 PD 直接中止, 表现就是"运行过程中 PD 有时会中途提前
            # 结束"——一次偶发坏帧不该终止整个作业。这里统一用 consecutive_bad_frames
            # 预算容忍连续坏帧: 预算内丢弃该帧继续控制; 超出预算才视为真实故障并
            # 按原语义中止 (安全停车)。仿真模式没有真实传感器噪声, 保持严格校验。
            try:
                validated = _validate_position_feedback(pos, config)
            except PositionFeedbackError:
                if (
                    not is_simulation
                    and last_accepted_pos is not None
                    and consecutive_bad_frames < config.max_consecutive_bad_frames
                ):
                    consecutive_bad_frames += 1
                    if verbose:
                        print(f'[定位异常] 丢弃无效帧 (连续第 {consecutive_bad_frames} 帧)')
                    continue
                raise
            pos = validated

            # 跳变帧: 单周期位置变化远超物理可达 (max_v*dt + 余量), 视为定位异常值。
            # 若送入控制会污染差分速度/D 项, 更危险的是恰好落在目标附近时会
            # 误触发到位、把轴永久锁死 → 表现为"离目标十几厘米就停、PD 结束"。
            if not is_simulation and last_accepted_pos is not None:
                dt_guard = pos['dt'] if pos['dt'] and pos['dt'] > 0 else 0.1
                max_step_xy = config.max_velocity_xy * dt_guard + config.localization_jump_margin
                max_step_z = config.max_velocity_z * dt_guard + config.localization_jump_margin
                jumped = (
                    abs(pos['x'] - last_accepted_pos['x']) > max_step_xy
                    or abs(pos['y'] - last_accepted_pos['y']) > max_step_xy
                    or abs(pos['z'] - last_accepted_pos['z']) > max_step_z
                )
                if jumped:
                    if consecutive_bad_frames < config.max_consecutive_bad_frames:
                        consecutive_bad_frames += 1
                        if verbose:
                            print(
                                f"[定位异常] 丢弃跳变帧 (连续第 {consecutive_bad_frames} 帧) dxyz="
                                f"({pos['x'] - last_accepted_pos['x']:+.2f}, "
                                f"{pos['y'] - last_accepted_pos['y']:+.2f}, "
                                f"{pos['z'] - last_accepted_pos['z']:+.2f})m"
                            )
                        continue
                    raise PositionFeedbackError(
                        f'定位连续 {consecutive_bad_frames + 1} 帧跳变异常, '
                        f'疑似传感器/网络故障 (超出容忍预算)'
                    )
            last_accepted_pos = {'x': pos['x'], 'y': pos['y'], 'z': pos['z']}
            consecutive_bad_frames = 0

            x_measured = pos['x']
            y_measured = pos['y']
            z_measured = pos['z']
            dt = pos['dt']
            t = pos['t']

            if max_time is not None and t > max_time:
                raise TimeoutError(
                    f"{mode_label} control did not finish within "
                    f"{max_time:.2f}s (t={t:.2f}s)"
                )

            # --- STEP 2-3: 速度估计 + PD 控制 (共用核心) ---
            vx_raw, vx_filtered = filters['x'].update(x_measured, dt)
            vy_raw, vy_filtered = filters['y'].update(y_measured, dt)
            vz_raw, vz_filtered = filters['z'].update(z_measured, dt)

            # D 项速度反馈: 优先使用位置源提供的原生速度 (如 Odometry twist)，
            # 缺失时才退回到位置差分后的低通滤波速度。对 10Hz 量化定位做差分噪声大，
            # 直接进入 D 项会让速度指令在目标附近抖动、频繁换向。
            vx_damp = pos['vx'] if pos['vx'] is not None else vx_filtered
            vy_damp = pos['vy'] if pos['vy'] is not None else vy_filtered
            vz_damp = pos['vz'] if pos['vz'] is not None else vz_filtered

            vx_cmd = 0.0 if locked['x'] else controllers['x'].update(target_x, x_measured, vx_damp)
            vy_cmd = 0.0 if locked['y'] else controllers['y'].update(target_y, y_measured, vy_damp)
            vz_cmd = 0.0 if locked['z'] else controllers['z'].update(target_z, z_measured, vz_damp)

            # --- STEP 4: 执行 — 委托给 Actuator ---
            if is_simulation:
                # 仿真模式: plant.update_axis() 更新 state
                plant = getattr(source, '_plant', None)
                if plant is not None:
                    disturbance_x = plant.update_axis('x', crane.x, vx_cmd, target_x, locked['x'], dt, t)
                    disturbance_y = plant.update_axis('y', crane.y, vy_cmd, target_y, locked['y'], dt, t)
                    disturbance_z = plant.update_axis('z', crane.z, vz_cmd, target_z, locked['z'], dt, t)
            else:
                # PLC 模式: 发送指令到真实 PLC
                actuator.apply(vx_cmd, vy_cmd, vz_cmd, dt)
                # 从定位数据更新 state
                actuator.update_state(crane, pos)
                # 没有可信原生速度时，统一使用位置差分后的滤波估计。
                for axis_name, filtered_velocity in (
                    ('x', vx_filtered),
                    ('y', vy_filtered),
                    ('z', vz_filtered),
                ):
                    if pos[f'v{axis_name}'] is None:
                        getattr(crane, axis_name).velocity = filtered_velocity
                disturbance_x = disturbance_y = disturbance_z = 0.0

            # --- STEP 5: 到达检测 ---
            axis_data = [
                ('x', crane.x, target_x, vx_cmd),
                ('y', crane.y, target_y, vy_cmd),
                ('z', crane.z, target_z, vz_cmd),
            ]
            for name, axis, target, cmd in axis_data:
                if locked[name]:
                    continue
                in_capture_window = abs(axis.position - target) < config.arrival_capture_pos_tol
                command_settled = abs(cmd) < config.arrival_cmd_tol
                velocity_settled = abs(axis.velocity) < config.velocity_deadband
                arrived_now = (
                    _axis_arrived(axis, target, config)
                    or (in_capture_window and command_settled and velocity_settled)
                )
                # 去抖: 连续满足才累加, 一旦有一帧不满足立即清零。
                arrival_streak[name] = arrival_streak[name] + 1 if arrived_now else 0
                if arrival_streak[name] >= config.arrival_debounce_cycles:
                    axis.position = target
                    axis.velocity = 0.0
                    locked[name] = True
                    arrival_events.append((t, name))
                    if hooks is not None:
                        hooks.on_arrival(name, t)

            # --- STEP 6: 记录历史 ---
            step_data = {
                't': t,
                'x': crane.x.position, 'y': crane.y.position, 'z': crane.z.position,
                'vx': crane.x.velocity, 'vy': crane.y.velocity, 'vz': crane.z.velocity,
                'p_ref_x': target_x, 'p_ref_y': target_y, 'p_ref_z': target_z,
                'v_ref_x': 0.0, 'v_ref_y': 0.0, 'v_ref_z': 0.0,
                'vx_cmd': vx_cmd, 'vy_cmd': vy_cmd, 'vz_cmd': vz_cmd,
                'x_measured': x_measured, 'y_measured': y_measured, 'z_measured': z_measured,
                'vx_raw': vx_raw, 'vy_raw': vy_raw, 'vz_raw': vz_raw,
                'vx_filtered': vx_filtered, 'vy_filtered': vy_filtered, 'vz_filtered': vz_filtered,
                'disturbance_x': disturbance_x, 'disturbance_y': disturbance_y, 'disturbance_z': disturbance_z,
            }
            history.append(step_data)

            if hooks is not None:
                hooks.on_step(step_data)

        completed = True
        if verbose:
            final = history[-1]
            print("=" * 50)
            print(f"{mode_label}完成! 总时间: {final['t']:.2f}s")
            print(f"最终位置: X={final['x']:.3f}, Y={final['y']:.3f}, Z={final['z']:.3f}")
        return history, arrival_events
    finally:
        try:
            if completed:
                actuator.stop_motion()
            else:
                actuator.emergency_stop()
        finally:
            actuator.cleanup()
