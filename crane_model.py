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

    snap_arrivals = True

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

    def update(self, state, commands, targets, locked, dt, t) -> tuple[float, float, float]:
        """Advance all axes; optional physical plants can couple their motion here."""
        return tuple(self.update_axis(name, getattr(state, name), command, target,
                                      locked[name], dt, t)
                     for name, command, target in zip(('x', 'y', 'z'), commands, targets))

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
    # 现场调参说明 (2026-07): 大车停止阶段速度指令频繁正负切换会导致抓钩
    # 摆动。降低 Kp、提高 Kd 与 PLC 速度滤波 τ, 并扩大到位死区/防反向保护带,
    # 让 vx_cmd 更早、更稳地归零, 而不是在 ±小速度之间来回切。
    kp_pos: float = 0.4                # 位置环比例增益 [1/s] (原 0.5~0.6, 降低以减少近目标急刹)
    kd_pos: float = 0.55               # 速度阻尼增益, 用于 PD 的 D 项 (原 0.35~0.45, 提高以抑制过冲回摆)
    velocity_filter_tau: float = 0.25  # [s] SLAM/差分速度低通滤波时间常数（仿真模式）
    velocity_filter_tau_plc: float = 0.80  # [s] PLC 模式速度滤波 (原 0.50, 加大以平滑 D 项、减少换向)

    # 现场调参说明 (2026-08): 纯 PD 收敛到目标附近后, D 项已把速度压得很低,
    # 此时哪怕残余位置误差还有 2~3cm, 算出来的速度指令也小到不足以克服电机/
    # 传动死区与机械摩擦, 导致误差无法继续收敛 (表现为"停在离目标 0.03m 左右
    # 不再靠近")。加一个限幅积分项 (仅在接近目标的精定位窗口内生效, 避免长
    # 距离行程积分饱和) 专门用于顶开这部分残余误差, 把稳态精度从 ~0.03m
    # 收紧到 ~0.01m 量级。ki_pos=0 时完全禁用, 退化为原来的纯 PD。
    ki_pos: float = 0.4                     # [1/s²] 位置环积分增益 (0=禁用积分, 纯 PD)
    integral_band: float = 0.08             # [m] 仅在 |误差| 小于该值时才开始积分累加
    integral_output_limit: float = 0.10     # [m/s] 积分项对速度指令的贡献上限, 防止积分主导过冲

    # PLC 模式下 Z 轴单独的积分增益, 默认 0 (纯 PD)。原因是 Z 与 X/Y 的执行链路
    # 结构不同: X/Y 是速度伺服 (指令速度→车走), 而 Z 的 liftctrl 是绝对位置伺服——
    # PlcActuator 把 PD 输出的 vz 积分成高度设定值再下发, 也就是说"速度指令到
    # 高度"这一步本身已经是一个积分环节, 位置环因此天生具备积分作用 (稳态误差
    # 靠设定值累积消除, 不需要额外积分项)。此时再叠加 ki_pos 就变成双重积分,
    # 低频相位滞后接近 180°; 叠加抓钩高度反馈的纯延迟 (S7 轮询/PLC 扫描, 连接
    # 抖动期间 _LiftHeightSanitizer 还会沿用旧读数) 后必然自激, 正是现场看到的
    # "Z 在目标附近反复上下切换、收不住"。实测复现: 反馈延迟 1.0~2.0s 时旧参数把
    # 高度设定值推离目标 4~15cm 并反复换向, PD 只能超时收场; ki_pos_z=0 后同场景
    # 6~12s 正常收敛。
    # 仿真模式不受影响: PlantActuator 是速度伺服, 环路里没有这层积分, 仍用 ki_pos。
    ki_pos_z: float = 0.0                   # [1/s²] Z 位置环积分增益 (PLC 模式; 0=禁用)

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

    # --- 作业调度安全高度 (绝对高度, 相对于地面) ---
    approach_safe_z: float = 1.0       # [m] 接近取货点 XY 联动时的 Z 安全高度
    lift_cargo_safe_z: float = 1.2     # [m] 夹取后带货上升的安全高度 (Z 轴)
    transport_safe_z: float = 1.5      # [m] 运输阶段安全高度 (Z 轴)
    return_safe_z: float = 1.6         # [m] 作业完成后 Z 归位高度
    stabilize_delay: float = 1.0       # [s] XY 到达后货物判稳等待上限 (自适应: 已平稳时提前放行, 见 hook_settle_*)
    gripper_safety_delay: float = 0.5  # [s] 抓钩动作确认后等待 (机械动作确认延时, 与摆动无关, 固定值)

    # --- 货物平稳判定 (自适应等待, 效率与安全兼顾) ---
    # 货物已经静止 → 尽快放行 (可能远小于等待上限, 提升效率);
    # 货物仍在摆动/回弹 → 持续等待直到真正平稳 (不超过等待上限, 避免无限等待)。
    # 用于替代"盲等固定时长后就不管三七二十一继续抓钩动作"的旧逻辑。
    hook_settle_vel_tol: float = 0.012      # [m/s] 判稳窗口首尾平均漂移速度阈值 (非瞬时速度,
                                             # 对定位测量噪声天然免疫; 用于排除"峰峰值不大但仍
                                             # 持续缓慢漂移"的情况, 如钢丝绳蠕变/缓慢下沉)
    hook_settle_pos_tol: float = 0.02       # [m] 判稳滑动窗口内位置峰峰值容差 (摆动幅度代理;
                                             # 主判据——天然规避"摆动最高点瞬时速度为0"的假阳性)
    hook_settle_window: float = 0.6         # [s] 判稳滑动窗口时长; 需覆盖单摆过零点附近的运动,
                                             # 否则窗口太短可能刚好截在摆动幅值最小处误判"已静止"
    gripper_settle_max_wait: float = 2.0    # [s] Z 下降到位→抓钩动作前的判稳等待上限。
                                             # 原流程中缺失的安全环节: Z 到位后立即执行抓钩夹取/
                                             # 释放, 中间没有任何货物平稳判定。

    # 抓钩完全释放确认后, 停留一小段时间再抬升 (给货物/抓钩一个短暂的
    # 脱离缓冲), 之后直接以正常速度抬升到目标高度, 不做额外分段限速。
    post_release_lift_delay: float = 2.0    # [s] 释放确认后, 开始抬升前的停留时长

    # --- 仿真参数 ---
    dt: float = 0.01                   # [s] 仿真步长
    # 现场调参说明 (2026-08): 原纯 PD (无积分) 组合下, 实测最终残余误差可达
    # 0.03m 以上——原因是纯 PD 在残余误差还有 2~4cm 时, D 项已经把速度压
    # 得很低, 算出来的指令本身就小到落入"到位捕获"的速度窗口, 而这部分
    # 残余恰恰是电机死区/机械摩擦阻力造成的稳态偏差, 单靠比例项永远追不平。
    #
    # 用 ki_pos 精定位积分项顶开这部分残余偏差, 同时小幅收紧到位捕获窗口
    # (arrival_capture_pos_tol/arrival_cmd_tol), 两者需要配套调整——单独收紧
    # 窗口而不加积分项, 在存在持续低频扰动/测量噪声时会让"指令+速度双双
    # 达标"这一条件很难同时满足, 现场实测会让收敛时间从几十秒暴涨到近
    # 200s (欲速不达); 加了积分项后同样的窗口不会再出现这个问题——积分项
    # 提供的持续小幅修正会不断"推"系统穿过卡滞点, 而不是纯 P 项在残余误差
    # 很小时趋于停滞。实测多组目标/扰动种子: 残余误差从 ~0.03m 收紧到
    # ~0.02m 量级, 收敛耗时与调整前基本一致 (无异常拖长)。
    arrival_pos_tol: float = 0.02          # [m] 到达判断位置容差; 同时用作 PD 速度指令死区 (原 0.025)
    arrival_vel_tol: float = 0.005         # [m/s] 到达判断速度容差
    # 零速阈值；低于此值且到位时，模拟伺服驱动的 zero-speed 窗口行为。
    velocity_deadband: float = 0.01

    # Z 轴 (PLC 模式) 单独的速度类容差。PLC 模式下 Z 位置来自抓钩高度
    # GetActualLiftHeight(), 10Hz 且分辨率较粗, 没有原生速度可用, Z 速度只能
    # 由高度差分+低通估计: 高度只要跳一个最小分辨率单位, 差分速度瞬时值就是
    # 0.1m/s 量级, 经 τ=0.8s 低通后仍会留下 0.01m/s 量级的尖峰。这个噪声地板
    # 高于 X/Y 用的 arrival_vel_tol(0.005)/velocity_deadband(0.01), 结果是 Z
    # 明明已经停在目标附近, 到位去抖计数却被噪声一帧帧清零而永远锁不上轴,
    # 控制循环继续给指令 → 表现为"Z 在目标附近反复上下微动、不结束"。
    # 这里给 Z 一组与该噪声地板匹配的容差 (仅 PLC 模式生效, 仿真沿用原值),
    # 配合 arrival_pos_tol(0.02m) 与 3 帧去抖, 锁轴时的真实残余位移
    # 约 0.02m/s × 0.3s ≈ 6mm, 精度上完全可接受。
    arrival_vel_tol_z: float = 0.02         # [m/s] Z 到达判断速度容差 (PLC 模式)
    velocity_deadband_z: float = 0.025      # [m/s] Z 到位捕获零速阈值 (PLC 模式)

    # Z 位置死区的滞环退出阈值 / 到达判断位置容差 (PLC 模式)。进入死区仍用
    # arrival_pos_tol (0.02m, 仍然按 2cm 精度去瞄), 但要重新给速度指令必须误差
    # 超过这个更大的阈值。没有滞环时, 粗分辨率 + 带滞后的高度读数会在 2cm 容差
    # 边界上抖进抖出, 每出去一次就给一次方向不定的速度脉冲, 而 Z 的速度指令会被
    # 积分成绝对高度设定值, 每个脉冲都变成一段真实行程 → 抓钩在目标附近来回换向。
    # 到达判定用同一个阈值: 抓钩高度的可达精度本来就受读数分辨率与受载绳伸长
    # 限制, 判定窗口比死区滞环还严就会出现"停在原地却永远不锁轴", 只能等 PD 超时。
    arrival_pos_tol_z: float = 0.04          # [m] Z 死区滞环退出阈值 / 到达位置容差 (PLC 模式)
    arrival_capture_pos_tol: float = 0.02  # [m] 到位捕获位置窗口 (原 0.04, 需配合 ki_pos>0 一起
                                            # 使用, 否则在持续扰动下可能出现收敛时间异常拖长)
    arrival_cmd_tol: float = 0.018         # [m/s] 到位捕获速度指令窗口 (原 0.025, 理由同上)
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
    # actuator.apply() 瞬时失败 (PLC 连接/心跳抖动) 的容忍预算: 连续失败次数
    # 不超过该值时跳过本周期继续控制 (心跳线程会在背后持续重试自愈);
    # 超过则视为真实故障, 按原语义中止并安全停车。10Hz 下默认 10 次 ≈ 1s,
    # 与心跳自身的失败容忍窗口 (见 plc_interface._max_fails) 保持一致量级。
    max_consecutive_actuator_errors: int = 10
    # 防反向抽动保护带 [m]: 在尚未越过目标且 |误差| 小于该值时，禁止 PD 给出
    # 远离目标方向的速度指令（速度伺服型行车"刹车"=指令归零而非反向脉冲）。
    reverse_guard_tol: float = 0.10    # 原 0.05; 扩大以减少目标附近反向脉冲→抓钩摆动

    # --- 倾角仪 / 抓钩摆角闭环防摇 ---
    # 控制律 Δv = +K(L)·θ_ac 叠加到 PD 速度指令 (见 sway_controller 能量法),
    # ζ = K/(2√(gL)) 仅适用于理想角度/速度执行。新末端策略先以自然支点运动
    # 按轴检查测量一致性，进入末端窗口后渐入；位置到位不等于防摇完成。
    # 调度器仍保留后续独立判稳，但无独立激励的自由摆不能新取得反馈资格。
    enable_anti_sway: bool = False
    anti_sway_terminal_enabled: bool = True
    anti_sway_travel_damping: bool = False
    # 末端按轴窗口同时考虑速度×周期和制动距离；接近目标本身不能证明角度可信。
    anti_sway_approach_damping: bool = True
    anti_sway_approach_pos: float = 0.40   # [m] 末端窗口最小距离；旧机制使用合成距离
    anti_sway_approach_vel: float = 0.10   # [m/s] 兼容旧配置，不作为硬速度门限
    anti_sway_sway_gain: float = 0.0
    # 绳长增益调度断点 ((L, K), ...), L 严格递增; 空=固定增益。现场 L 仅 2.4~3.4m,
    # K_opt 跨度 4.9~5.8, 固定增益足够, 一般不需要调度。
    anti_sway_gain_schedule: tuple = ()
    anti_sway_angle_scale: float = 1.0
    anti_sway_roll_bias: float = 0.0
    anti_sway_pitch_bias: float = 0.0
    anti_sway_roll_bias_closed: float | None = None
    anti_sway_pitch_bias_closed: float | None = None
    anti_sway_max_correction: float = 0.05
    anti_sway_settle_damping: bool = True
    # 位移预算 [m] — 阻尼要求小车追着载荷走, 必须允许偏离锚点一小段行程。
    # 两个值语义不同, 不要设成一样:
    #   locked_max_shift: 高 Z 悬停判稳 (抓钩离地, 横移安全) → 可放宽
    #   settle_max_shift: Z 已下降到取放货高度、抓钩贴近钢卷时判稳 → 必须收紧;
    #                     0 = 只判稳、横移一步都不走 (抓取/放货精度优先)
    anti_sway_locked_max_shift: float = 0.06
    anti_sway_settle_max_shift: float = 0.0
    # 锚点回中增益 [1/s] — 见 AntiSwayRuntime.recenter_gain。必须远慢于摆频
    # (ωn≈1.8 rad/s)，现有 0.3 不在本轮提高；实际效果不能只靠量纲判断。
    anti_sway_hold_recenter_gain: float = 0.3
    # θ_ac 高通时间常数 [s]: 倾角仪零偏随抓钩高度/张合变化 (见 doc/倾角仪初值, 幅度
    # 约 0.3°, 与判稳容差同量级), 残留 DC 会让 Δv=K·θ_dc 变成恒定速度而把车推走。
    # τ=2.5s 对 T≈3.2~3.7s 的摆只衰减约 2.7%, 同时给出约 12° 相位超前。
    anti_sway_settle_dc_tau: float = 2.5
    anti_sway_settle_gain_scale: float = 1.5
    # 绳长几何见 .cursor/rules/hoist-rope-geometry.mdc:
    # 电子葫芦 H=4m, PLC Z=抓钩离地, 结构 h=1.5m → L_rope=H−Z−h, L_eff=H−Z。
    # sheave=4、offset=0, 不要把 1.5 填进 grab_offset。
    rope_length_sheave_height: float = 4.0
    rope_length_grab_offset: float = 0.0
    rope_length_structure: float = 1.5
    rope_length_cable_stretch: float = 0.0
    rope_length_min: float = 0.5
    hook_settle_window_periods: float = 0.55
    hook_settle_window_max: float = 8.0
    hook_sway_angle_tol_deg: float = 0.3
    hook_sway_angle_tol_approach_deg: float = 0.6
    # 倾角判稳等待上限 [s]。末端消摆超时与 settled 分开记录；之后可能仍需
    # PD 恢复位置/速度。调度器后续独立等待不包含在这段控制完成时间内。
    hook_settle_max_wait: float = 12.0

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
            'approach_safe_z': self.approach_safe_z,
            'lift_cargo_safe_z': self.lift_cargo_safe_z,
            'transport_safe_z': self.transport_safe_z,
            'return_safe_z': self.return_safe_z,
            'stabilize_delay': self.stabilize_delay,
            'gripper_safety_delay': self.gripper_safety_delay,
            'ki_pos': self.ki_pos,
            'ki_pos_z': self.ki_pos_z,
            'integral_band': self.integral_band,
            'integral_output_limit': self.integral_output_limit,
            'hook_settle_vel_tol': self.hook_settle_vel_tol,
            'hook_settle_pos_tol': self.hook_settle_pos_tol,
            'hook_settle_window': self.hook_settle_window,
            'hook_settle_max_wait': self.hook_settle_max_wait,
            'hook_sway_angle_tol_deg': self.hook_sway_angle_tol_deg,
            # 增益符号即阻尼符号: K<0 会让 Δv 反向抽能, 把摆越推越大 (见
            # sway_controller 的能量法推导), 必须在构造期就拦住。
            'anti_sway_approach_pos': self.anti_sway_approach_pos,
            'anti_sway_approach_vel': self.anti_sway_approach_vel,
            'anti_sway_sway_gain': self.anti_sway_sway_gain,
            'anti_sway_max_correction': self.anti_sway_max_correction,
            'anti_sway_locked_max_shift': self.anti_sway_locked_max_shift,
            'anti_sway_settle_max_shift': self.anti_sway_settle_max_shift,
            'anti_sway_hold_recenter_gain': self.anti_sway_hold_recenter_gain,
            'gripper_settle_max_wait': self.gripper_settle_max_wait,
            'post_release_lift_delay': self.post_release_lift_delay,
            'arrival_pos_tol': self.arrival_pos_tol,
            'arrival_vel_tol': self.arrival_vel_tol,
            'velocity_deadband': self.velocity_deadband,
            'arrival_vel_tol_z': self.arrival_vel_tol_z,
            'velocity_deadband_z': self.velocity_deadband_z,
            'arrival_pos_tol_z': self.arrival_pos_tol_z,
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

        if int(self.max_consecutive_actuator_errors) != self.max_consecutive_actuator_errors \
                or self.max_consecutive_actuator_errors < 0:
            raise ValueError('max_consecutive_actuator_errors must be a non-negative integer')
        self.max_consecutive_actuator_errors = int(self.max_consecutive_actuator_errors)

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


def _axis_arrived(
    axis,
    target: float,
    config: CraneConfig,
    vel_tol: float | None = None,
    pos_tol: float | None = None,
) -> bool:
    """判断单轴是否已到达目标 (位置 + 速度双重判定)。

    pos_tol / vel_tol 缺省时使用 config.arrival_pos_tol / arrival_vel_tol;
    传入时用于逐轴容差 (PLC 模式的 Z 用抓钩高度读数, 分辨率粗、带滞后、
    速度只能差分估计, 位置与速度容差都需要单独放宽)。
    """
    pos_ok = abs(axis.position - target) < (
        config.arrival_pos_tol if pos_tol is None else pos_tol
    )
    vel_ok = abs(axis.velocity) < (
        config.arrival_vel_tol if vel_tol is None else vel_tol
    )
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
    sway_source: object | None = None,
    simulation_duration: float | None = None,
    sway_guard: object | None = None,
    sway_safe_height: float | None = None,
    simulation_hold: bool = False,
) -> tuple[list[dict], list[tuple[float, str]]]:
    """统一 PD 位置控制循环 — 仿真和 PLC 模式共用。

    控制律:
      v_cmd = Kp * (target - measured) - Kd * filtered_velocity + K(L)·θ_ac

    末项是可选的摆角闭环防摇 (需 config.enable_anti_sway 且传入 sway_source),
    新末端策略先按轴检验自然运动下的测量一致性，再在明确安全高度的末端
    窗口渐入。首次到位与控制完成分开：反馈参与后继续有限时间消摆，完整周期
    角度 PP、位置和速度均合格才记录 settled；缺测/失配退出、超时分别记录。
    OperationScheduler._wait_cargo_settled() 仍保留独立确认等待，不能把本函数
    的完成时间当作整个作业周期。关闭末端策略可用于旧机制对照。

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
        sway_source:    可选摆角源, 需提供 get_sway() -> {'theta_x','theta_y'} | None
                        (断流时返回 None, 防摇自动退化为纯 PD)。按配置在
                        接近/锁轴或全程走行阶段叠加，并遵守取放货附近禁动门限。
        simulation_duration: 仿真固定观测时长 [s]；指定时到位后继续观测。
                        max_time 仍是总超时上限；PLC 模式禁止此选项。
        sway_guard:     可选的作业共享按轴增长保护；阶段切换不清除已撤回状态。
        sway_safe_height: 调度明确的运输安全高度；达到后可在末端窗口阻尼。
                        None 保留原贴货限制，不推断未知货物间隙。
        simulation_hold: 仅机制实验：到位后继续控制，不能称为停车后的残摆。

    Returns:
        (history, arrival_events):
          history:        每步状态的 list[dict]
          arrival_events: 各轴到达时间的 list[tuple[float, str]]
    """
    from pd_controller import PositionPDController
    from velocity_filter import LowPassVelocityEstimator

    if simulation_duration is not None:
        if not is_simulation or not math.isfinite(simulation_duration) or simulation_duration <= 0:
            raise ValueError('simulation_duration requires simulation mode and a positive finite duration')
    if simulation_hold and not is_simulation:
        raise ValueError('simulation_hold requires simulation mode')
    if sway_safe_height is not None and not math.isfinite(sway_safe_height):
        raise ValueError('sway_safe_height must be finite')

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
            ki_pos=config.ki_pos,
            integral_band=config.integral_band,
            integral_output_limit=config.integral_output_limit,
        ),
        'y': PositionPDController(
            config.kp_pos, config.kd_pos, config.max_velocity_xy,
            position_deadband=config.arrival_pos_tol,
            reverse_tol=config.reverse_guard_tol,
            ki_pos=config.ki_pos,
            integral_band=config.integral_band,
            integral_output_limit=config.integral_output_limit,
        ),
        # Z 的积分增益按执行链路区分 (与 velocity_filter_tau 同理):
        # 仿真的 PlantActuator 是速度伺服, 积分项正常有效;
        # PLC 的 Z 是"PD 速度指令 → 积分成绝对高度设定值 → liftctrl 位置伺服",
        # 这条链路本身已含一个积分环节, 再叠加积分项就是双重积分, 在目标附近
        # 会产生上下往复的极限环 → 用 ki_pos_z (默认 0, 纯 PD)。详见 ki_pos_z 注释。
        'z': PositionPDController(
            config.kp_pos, config.kd_pos, config.max_velocity_z,
            position_deadband=config.arrival_pos_tol,
            reverse_tol=config.reverse_guard_tol,
            ki_pos=config.ki_pos if is_simulation else config.ki_pos_z,
            integral_band=config.integral_band,
            integral_output_limit=config.integral_output_limit,
            # 死区滞环只给 PLC 模式的 Z: 它的高度读数分辨率粗且带滞后, 没有滞环
            # 会在容差边界上抖进抖出, 每次都被积分成一段真实的升降行程。
            position_deadband_release=(
                0.0 if is_simulation else config.arrival_pos_tol_z
            ),
        ),
    }

    # 到位判定的速度类容差 (逐轴)。PLC 模式的 Z 用抓钩高度差分估速, 噪声地板
    # 远高于 X/Y 的原生速度, 必须放宽, 否则 Z 会因噪声一直锁不上轴而在目标
    # 附近持续微动。仿真模式 Z 反馈来自对象模型 (分辨率细), 沿用统一容差。
    arrival_vel_tol = {
        'x': config.arrival_vel_tol,
        'y': config.arrival_vel_tol,
        'z': config.arrival_vel_tol if is_simulation else config.arrival_vel_tol_z,
    }
    velocity_deadband = {
        'x': config.velocity_deadband,
        'y': config.velocity_deadband,
        'z': config.velocity_deadband if is_simulation else config.velocity_deadband_z,
    }
    # Z 的到位位置容差必须与死区滞环退出阈值一致, 否则会出现"死区滞环让抓钩
    # 停住了, 到位窗口却比它还严"→ 原地不动却永远不锁轴, 只能等 PD 超时。
    arrival_pos_tol = {
        'x': config.arrival_pos_tol,
        'y': config.arrival_pos_tol,
        'z': config.arrival_pos_tol if is_simulation else config.arrival_pos_tol_z,
    }
    capture_pos_tol = {
        'x': config.arrival_capture_pos_tol,
        'y': config.arrival_capture_pos_tol,
        'z': (
            config.arrival_capture_pos_tol
            if is_simulation
            else max(config.arrival_capture_pos_tol, config.arrival_pos_tol_z)
        ),
    }

    # 摆角闭环防摇。全程走行默认关 (假倾角); 目标附近 / XY 锁轴后按
    # anti_sway_approach_damping 打开。travel_on 仍可强制全程叠加。
    anti_sway_runtime = None
    travel_on = getattr(config, 'anti_sway_travel_damping', False)
    approach_on = getattr(config, 'anti_sway_approach_damping', False)
    if (
        (travel_on or approach_on)
        and sway_source is not None
        and getattr(sway_source, 'get_sway', None) is not None
    ):
        from anti_sway_runtime import AntiSwayRuntime
        from sway_controller import build_anti_sway
        from sway_guard import SwayGrowthGuard

        anti_sway_ctrl = build_anti_sway(config)
        if anti_sway_ctrl is not None:
            anti_sway_runtime = AntiSwayRuntime(
                anti_sway=anti_sway_ctrl,
                dc_tau=config.anti_sway_settle_dc_tau,
                locked_max_shift=config.anti_sway_locked_max_shift,
                recenter_gain=config.anti_sway_hold_recenter_gain,
                terminal_mode=config.anti_sway_terminal_enabled,
                amp_floor=math.radians(config.hook_sway_angle_tol_deg / 4
                                       if config.anti_sway_terminal_enabled else .15),
                guard=sway_guard if sway_guard is not None else SwayGrowthGuard(
                    min_pp_rad=math.radians(2 * config.hook_sway_angle_tol_deg),
                ),
            )
            anti_sway_runtime.reset(crane.x.position, crane.y.position)

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
    # 连续 actuator.apply() 失败计数 (PLC 连接/心跳瞬时抖动), 与上面计数器
    # 分开维护, 避免定位帧恢复正常就重置了执行器故障计数 (二者是不同故障源)。
    consecutive_actuator_errors = 0
    consecutive_feedback_timeouts = 0

    # PD 输出
    vx_cmd = vy_cmd = vz_cmd = 0.0
    vx_raw = vy_raw = vz_raw = 0.0
    vx_filtered = vy_filtered = vz_filtered = 0.0

    # 防摇输出记录 (UI 曲线; 未启用防摇时恒为 0/None)
    antisway_dvx = antisway_dvy = 0.0
    theta_x = theta_y = None
    approach_logged = False
    guard_event_count = len(anti_sway_runtime.guard.events) if anti_sway_runtime else 0
    from sway_terminal import TerminalSwayMonitor, terminal_distance
    terminal_monitor = TerminalSwayMonitor()
    terminal_axes = [False, False]
    terminal_applied = False
    terminal_start = None
    terminal_outcome = 'moving'
    terminal_withdrawn = False
    control_complete = False
    terminal_policy = anti_sway_runtime is not None and config.anti_sway_terminal_enabled

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
    t = 0.0

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
        while (t < simulation_duration - config.dt * 1e-6
               if simulation_duration is not None else not control_complete):
            # --- 检查钩子中断 ---
            if hooks is not None and hooks.should_stop():
                raise ControlStoppedError(f'{mode_label} control stopped by external request')

            # --- STEP 1: 获取位置反馈 ---
            pos = source.get_position()
            if pos is None:
                if anti_sway_runtime is not None:
                    anti_sway_runtime.suspend(0.)
                    terminal_monitor.reset()
                consecutive_feedback_timeouts += 1
                if consecutive_feedback_timeouts < 3:
                    if verbose:
                        print(
                            f'[定位] 反馈短暂断流 '
                            f'(连续第 {consecutive_feedback_timeouts} 次), 继续控制...'
                        )
                    continue
                raise PositionFeedbackTimeout(
                    f'{mode_label} position feedback timed out '
                    f'({consecutive_feedback_timeouts} consecutive misses)'
                )
            consecutive_feedback_timeouts = 0

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
                if anti_sway_runtime is not None:
                    anti_sway_runtime.suspend(0.)
                    terminal_monitor.reset()
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
                    if anti_sway_runtime is not None:
                        anti_sway_runtime.suspend(0.)
                        terminal_monitor.reset()
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

            vx_cmd = 0.0 if locked['x'] else controllers['x'].update(target_x, x_measured, vx_damp, dt)
            vy_cmd = 0.0 if locked['y'] else controllers['y'].update(target_y, y_measured, vy_damp, dt)
            vz_cmd = 0.0 if locked['z'] else controllers['z'].update(target_z, z_measured, vz_damp, dt)
            vx_pd, vy_pd = vx_cmd, vy_cmd

            # --- STEP 3b: 摆角闭环防摇 — Δv = +K(L)·θ_ac 叠加到 XY 指令 ---
            # 已锁轴的 XY 仍在预算内做软阻尼 (典型场景: XY 已到位、Z 还在下降,
            # 此时不阻尼就等于放着摆一路摆到取放货高度)。摆角断流 (get_sway
            # 返回 None) 时不更新 DC 跟踪器也不叠加, 自动退化为纯 PD。
            antisway_dvx = antisway_dvy = 0.0
            theta_x = theta_y = None
            antisway_allowed = False
            terminal_settled = False
            if anti_sway_runtime is not None:
                from anti_sway_runtime import approach_damping_allowed

                sway = sway_source.get_sway()
                if sway is not None and not all(math.isfinite(float(sway[k])) for k in ('theta_x', 'theta_y')):
                    sway = None
                if sway is None:
                    anti_sway_runtime.suspend(dt)
                    terminal_monitor.reset()
                if sway is not None:
                    theta_x = float(sway['theta_x'])
                    theta_y = float(sway['theta_y'])
                    z_remain = abs(target_z - z_measured)
                    near_cargo = (z_remain <= 0.30 if sway_safe_height is None else
                                  z_measured < sway_safe_height - config.arrival_pos_tol_z)
                    # 贴近取放货高度禁止横移 (与 settle_max_shift=0 一致)
                    observe_only = (
                        near_cargo and config.anti_sway_settle_max_shift <= 0.0
                    )
                    allow = (not observe_only) and approach_damping_allowed(
                        err_x=target_x - x_measured,
                        err_y=target_y - y_measured,
                        vx=vx_damp,
                        vy=vy_damp,
                        locked_x=locked['x'],
                        locked_y=locked['y'],
                        travel_on=travel_on,
                        approach_on=approach_on,
                        approach_pos=config.anti_sway_approach_pos,
                        approach_vel=config.anti_sway_approach_vel,
                    )
                    period = 2 * math.pi * math.sqrt(
                        anti_sway_runtime.anti_sway.rope_model.compute(z_measured) / 9.81)
                    if terminal_policy:
                        for a, (error, speed) in enumerate(((target_x - x_measured, vx_filtered),
                                                          (target_y - y_measured, vy_filtered))):
                            distance = terminal_distance(speed, period, config.anti_sway_approach_pos,
                                config.max_velocity_xy, config.max_acceleration_xy)
                            terminal_axes[a] |= bool(travel_on or (approach_on and abs(error) <= distance))
                        allow = not observe_only and any(terminal_axes)
                    allow = allow and not terminal_withdrawn and (not control_complete or simulation_hold)
                    antisway_allowed = allow
                    anti_sway_runtime.locked_max_shift = (
                        config.anti_sway_settle_max_shift
                        if near_cargo
                        else config.anti_sway_locked_max_shift
                    )
                    # 每个有效样本都更新观测；门控只决定能否向 PD 添加修正。
                    vx_cmd, vy_cmd, antisway_dvx, antisway_dvy = anti_sway_runtime.merge_xy_commands(
                        vx_cmd, vy_cmd, theta_x=theta_x, theta_y=theta_y,
                        z_grab=z_measured, x=x_measured, y=y_measured,
                        locked_x=locked['x'], locked_y=locked['y'], dt=dt,
                        max_velocity_xy=config.max_velocity_xy,
                        gain_scale=(config.anti_sway_settle_gain_scale
                                    if not terminal_policy and locked['x'] and locked['y'] else 1.0),
                        feedback_allowed=allow,
                        feedback_axes=tuple(terminal_axes) if terminal_policy else (True, True),
                    )
                    terminal_applied |= abs(vx_cmd - vx_pd) + abs(vy_cmd - vy_pd) > 1e-8
                    terminal_settled = terminal_monitor.update(t=t, theta=(theta_x, theta_y),
                        error=(target_x - x_measured, target_y - y_measured),
                        velocity=(vx_damp, vy_damp), period=period,
                        angle_tol=math.radians(config.hook_sway_angle_tol_deg),
                        pos_tol=config.arrival_pos_tol, vel_tol=config.velocity_deadband)
                    for event in anti_sway_runtime.guard.events[guard_event_count:]:
                        if verbose:
                            print(f"[Anti-sway] {event['axis'].upper()} 轴振荡增长 "
                                  f"{event['baseline_deg']:.3f}° → {event['pp_deg']:.3f}° PP，"
                                  "撤回该轴角度修正，本作业后续阶段保持 PD")
                    guard_event_count = len(anti_sway_runtime.guard.events)
                    if allow:
                        if not approach_logged and verbose:
                            print(
                                f"[Anti-sway] 接近/锁轴阻尼 ON  "
                                f"err_xy={math.hypot(target_x - x_measured, target_y - y_measured):.2f}m "
                                f"z_remain={z_remain:.2f}m "
                                f"θ=({math.degrees(theta_x):+.2f},{math.degrees(theta_y):+.2f})° "
                                f"Â={math.degrees(anti_sway_runtime.last_amp):.2f}° "
                                f"Δv=({antisway_dvx:+.3f},{antisway_dvy:+.3f})"
                            )
                            approach_logged = True
                    elif approach_logged and verbose:
                        print("[Anti-sway] 接近/锁轴阻尼 OFF")
                        approach_logged = False

            # Z 目标附近的微调门控 (仅 PLC 模式)。抓钩高度反馈带滞后 (S7 轮询/PLC
            # 扫描, 抖动期间还会沿用旧读数), 在最后几厘米里若继续按"仍在变化的读数"
            # 去修高度设定值, 等于追一个过时的误差: 误差符号来回翻, 设定值跟着来回
            # 走, 而 liftctrl 是绝对位置伺服, 每次都被忠实执行成一小段升降——这就是
            # 现场看到的"Z 在目标附近来回切换方向"。所以进入目标附近后, 只有当高度
            # 读数确实静止 (滞后已经过去) 时才允许再修设定值; 否则交给 PLC 自己的
            # 位置环把这一步走完。
            if not is_simulation and not locked['z']:
                near_target_z = abs(target_z - z_measured) < config.arrival_pos_tol_z
                z_feedback_moving = abs(vz_damp) >= config.arrival_vel_tol_z
                if near_target_z and z_feedback_moving:
                    vz_cmd = 0.0

            if control_complete and not simulation_hold:
                vx_cmd = vy_cmd = vz_cmd = 0.0
                antisway_dvx = antisway_dvy = 0.0

            # --- STEP 4: 执行 — 委托给 Actuator ---
            if is_simulation:
                # 仿真模式: plant.update_axis() 更新 state
                plant = getattr(source, '_plant', None)
                if plant is not None:
                    disturbance_x, disturbance_y, disturbance_z = plant.update(
                        crane, (vx_cmd, vy_cmd, vz_cmd), (target_x, target_y, target_z), locked, dt, t,
                    )
            else:
                # PLC 模式: 发送指令到真实 PLC。连接/心跳的瞬时抖动 (已在
                # PlcActuator/心跳线程做了容忍与背后自愈) 不应让这一次瞬时
                # 失败直接终止整段作业——否则表现为"距离目标还很远时 PD 就
                # 结束、输出速度为 0"。预算内跳过本周期继续控制; 超出预算
                # 才视为真实故障, 按原语义中止并安全停车。
                try:
                    actuator.apply(vx_cmd, vy_cmd, vz_cmd, dt)
                except RuntimeError as exc:
                    if anti_sway_runtime is not None:
                        anti_sway_runtime.suspend(0.)
                        terminal_monitor.reset()
                    if consecutive_actuator_errors < config.max_consecutive_actuator_errors:
                        consecutive_actuator_errors += 1
                        if verbose:
                            print(
                                f'[PLC异常] 执行器暂时不可用 '
                                f'(连续第 {consecutive_actuator_errors} 次): {exc}'
                            )
                        continue
                    raise
                consecutive_actuator_errors = 0
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
                in_capture_window = abs(axis.position - target) < capture_pos_tol[name]
                command_settled = abs(cmd) < config.arrival_cmd_tol
                velocity_settled = abs(axis.velocity) < velocity_deadband[name]
                arrived_now = (
                    _axis_arrived(
                        axis, target, config, arrival_vel_tol[name], arrival_pos_tol[name]
                    )
                    or (in_capture_window and command_settled and velocity_settled)
                )
                # 去抖: 连续满足才累加, 一旦有一帧不满足立即清零。
                arrival_streak[name] = arrival_streak[name] + 1 if arrived_now else 0
                if arrival_streak[name] >= config.arrival_debounce_cycles:
                    if not is_simulation or getattr(getattr(source, '_plant', None), 'snap_arrivals', True):
                        axis.position = target
                        axis.velocity = 0.0
                    locked[name] = True
                    if not any(axis_name == name for _, axis_name in arrival_events):
                        arrival_events.append((t, name))
                    # 锁轴瞬间把位移预算的锚点搬到锁定点。锚点若还停在出发位置,
                    # |x−anchor| 会是整段行程 (米级) 远超预算, 软阻尼就只剩"朝出发
                    # 点回拉"的半个周期 → 被半波整流成单向漂移 (见 AntiSwayRuntime
                    # .set_anchor)。
                    if anti_sway_runtime is not None and name in ('x', 'y'):
                        anti_sway_runtime.set_anchor(**{name: target})
                    if hooks is not None:
                        hooks.on_arrival(name, t)

            if all(locked.values()) and not control_complete:
                if terminal_start is None:
                    terminal_start = t
                eligible = (terminal_policy and terminal_applied and antisway_allowed
                            and theta_x is not None and any(
                                q and not b for q, b in zip(anti_sway_runtime.qualifier.allowed,
                                                            anti_sway_runtime.guard.blocked)))
                if not eligible:
                    if not terminal_withdrawn:
                        terminal_outcome = 'feedback_withdrawn' if terminal_applied and terminal_policy else 'position_only'
                    control_complete = True
                elif terminal_settled:
                    terminal_outcome, control_complete = 'settled', True
                elif t - terminal_start >= config.hook_settle_max_wait:
                    terminal_outcome, control_complete = 'settle_timeout', True
                if control_complete and terminal_policy and terminal_applied:
                    # Withdrawal stops the angle path, NOT the position contract.
                    # A locked support may have moved during damping or a gust.
                    # Recover under ordinary PD; never blind-recenter with a bad angle.
                    for name, axis, target in (('x', crane.x, target_x), ('y', crane.y, target_y)):
                        # PLC arrival display snaps state to target/zero; it is
                        # not a new measurement. Simulation uses post-step truth.
                        position = axis.position if is_simulation else pos[name]
                        velocity = axis.velocity if is_simulation else (
                            vx_damp if name == 'x' else vy_damp)
                        if (abs(position - target) >= config.arrival_pos_tol
                                or abs(velocity) >= config.velocity_deadband):
                            terminal_withdrawn = True
                            locked[name] = False
                            arrival_streak[name] = 0
                            controllers[name].reset()
                            control_complete = False

            # --- STEP 6: 记录历史 ---
            step_data = {
                't': t,
                'x': crane.x.position, 'y': crane.y.position, 'z': crane.z.position,
                'vx': crane.x.velocity, 'vy': crane.y.velocity, 'vz': crane.z.velocity,
                'p_ref_x': target_x, 'p_ref_y': target_y, 'p_ref_z': target_z,
                'v_ref_x': 0.0, 'v_ref_y': 0.0, 'v_ref_z': 0.0,
                'vx_cmd': vx_cmd, 'vy_cmd': vy_cmd, 'vz_cmd': vz_cmd,
                'vx_pd': vx_pd, 'vy_pd': vy_pd,
                'antisway_applied_dvx': vx_cmd - vx_pd,
                'antisway_applied_dvy': vy_cmd - vy_pd,
                'antisway_allowed': antisway_allowed,
                'control_complete': control_complete,
                'terminal_outcome': terminal_outcome,
                'terminal_active_x': terminal_axes[0],
                'terminal_active_y': terminal_axes[1],
                'x_measured': x_measured, 'y_measured': y_measured, 'z_measured': z_measured,
                'vx_raw': vx_raw, 'vy_raw': vy_raw, 'vz_raw': vz_raw,
                'vx_filtered': vx_filtered, 'vy_filtered': vy_filtered, 'vz_filtered': vz_filtered,
                'disturbance_x': disturbance_x, 'disturbance_y': disturbance_y, 'disturbance_z': disturbance_z,
                'antisway_dvx': antisway_dvx, 'antisway_dvy': antisway_dvy,
                'theta_x': theta_x, 'theta_y': theta_y,
                'antisway_amp_deg': (
                    math.degrees(anti_sway_runtime.last_amp)
                    if anti_sway_runtime is not None else None
                ),
            }
            if anti_sway_runtime is not None:
                step_data.update(anti_sway_runtime.guard.snapshot())
                step_data['theta_ac_x'] = anti_sway_runtime.amp.last_ac_x if theta_x is not None else None
                step_data['theta_ac_y'] = anti_sway_runtime.amp.last_ac_y if theta_y is not None else None
                if anti_sway_runtime.qualifier is not None:
                    step_data.update(anti_sway_runtime.qualifier.snapshot())
            if is_simulation and hasattr(getattr(source, '_plant', None), 'sway_snapshot'):
                step_data.update(source._plant.sway_snapshot(crane))
                step_data['sim_position_arrived'] = all(locked.values())
            history.append(step_data)

            if hooks is not None:
                hooks.on_step(step_data)

        completed = True
        if verbose:
            final = history[-1]
            print("=" * 50)
            outcome = '观测结束' if simulation_duration is not None else '完成'
            print(f"{mode_label}{outcome}! 总时间: {final['t']:.2f}s")
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
