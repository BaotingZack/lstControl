import math
import json
import threading
import time

import pytest

import live_server
import main
from crane_model import ControlHooks, CraneConfig, CraneState, run_pd_control
from live_server import ControlState, CraneLiveServer
from plc_interface import MockPLC, PlcActuator, create_plc
from plc_interface import _LiftHeightSanitizer


class RecordingPLC(MockPLC):
    def __init__(self, *, connected=True, heartbeat_healthy=True):
        super().__init__()
        self._connected = connected
        self._heartbeat_healthy = heartbeat_healthy
        self.big_car_commands = []
        self.small_car_commands = []
        self.lift_commands = []

    def big_car_ctrl(self, velocity):
        self.big_car_commands.append(velocity)

    def small_car_ctrl(self, velocity):
        self.small_car_commands.append(velocity)

    def lift_ctrl(self, height):
        self.lift_commands.append(height)


class StubActuator:
    def __init__(self):
        self.apply_calls = []
        self.stop_calls = 0
        self.emergency_stop_calls = 0
        self.cleanup_calls = 0

    def apply(self, vx, vy, vz, dt):
        self.apply_calls.append((vx, vy, vz, dt))

    def update_state(self, state, position):
        state.x.position = position["x"]
        state.y.position = position["y"]
        state.z.position = position["z"]
        state.x.velocity = position["vx"]
        state.y.velocity = position["vy"]
        state.z.velocity = position["vz"]

    def stop_motion(self):
        self.stop_calls += 1

    def emergency_stop(self):
        self.emergency_stop_calls += 1

    def cleanup(self):
        self.cleanup_calls += 1


class SequenceSource:
    def __init__(self, positions):
        self._positions = iter(positions)

    def reset(self):
        pass

    def get_position(self):
        return next(self._positions)


def position(*, x=0.0, y=0.0, z=0.0, t=0.1, stamp=1):
    return {
        "x": x,
        "y": y,
        "z": z,
        "vx": 0.0,
        "vy": 0.0,
        "vz": 0.0,
        "dt": 0.1,
        "t": t,
        "stamp": stamp,
    }


def test_plc_actuator_sends_updated_height_while_velocity_is_constant():
    plc = RecordingPLC()
    actuator = PlcActuator(plc, initial_z=1.0)

    for _ in range(5):
        actuator.apply(0.0, 0.0, 0.2, 0.1)

    assert plc.lift_commands == pytest.approx([1.02, 1.04, 1.06, 1.08, 1.10])


def test_plc_actuator_resends_velocity_every_cycle_while_constant():
    """Velocity servo needs continuous refresh; a constant command must still be
    re-sent each cycle, otherwise the PLC watchdog stops the axis (stutter)."""
    plc = RecordingPLC()
    actuator = PlcActuator(plc, initial_z=1.0)  # above the 0.35m safety floor

    for _ in range(4):
        actuator.apply(0.2, 0.2, 0.0, 0.1)

    assert plc.big_car_commands == pytest.approx([0.2, 0.2, 0.2, 0.2])
    assert plc.small_car_commands == pytest.approx([0.2, 0.2, 0.2, 0.2])
    # Held height is re-sent every cycle (stays at 1.0, above floor).
    assert plc.lift_commands == pytest.approx([1.0, 1.0, 1.0, 1.0])


def test_plc_actuator_inverts_axis_command_direction():
    """Per-axis sign flips the command when the drive positive direction is
    opposite to the localization axis (a reflection SLAM yaw cannot express)."""
    plc = RecordingPLC()
    actuator = PlcActuator(
        plc,
        initial_z=1.0,  # above safety floor so the inverted Z is observable
        big_car_sign=-1.0,
        small_car_sign=-1.0,
        lift_sign=-1.0,
    )

    actuator.apply(0.2, 0.1, 0.2, 0.1)

    assert plc.big_car_commands == pytest.approx([-0.2])
    assert plc.small_car_commands == pytest.approx([-0.1])
    # vz inverted before integration -> height moves down (1.0 - 0.2*0.1)
    assert plc.lift_commands == pytest.approx([0.98])


def test_plc_actuator_z_setpoint_marches_all_the_way_to_target():
    """liftctrl 是绝对位置伺服: 设 Z 目标后, 高度设定值必须一路走到目标并停住,
    而不是每周期只领先一步 (旧逻辑会因 update_state 重锚导致设定值几乎不动)。"""
    plc = RecordingPLC()
    # z_hold_band=0: 只考察"设定值一路走到目标"这件事, 到达后的有界微调
    # 权限由 test_plc_actuator_z_hold_band_allows_bounded_trim 单独覆盖。
    actuator = PlcActuator(plc, initial_z=5.0, z_hold_band=0.0)  # target 2.0 > floor 0.35
    actuator.set_z_target(2.0)

    for _ in range(300):
        # 模拟控制循环: 先下发速度, 再用实测高度更新状态 (不得重锚设定值)。
        actuator.apply(0.0, 0.0, -0.2, 0.1)
        actuator.update_state(
            CraneState(0.0, 0.0, plc.lift_commands[-1]),
            position(x=0.0, y=0.0, z=plc.lift_commands[-1]),
        )

    # 设定值单调下行、精确停在目标、且从不越过目标。
    assert plc.lift_commands[-1] == pytest.approx(2.0)
    assert min(plc.lift_commands) >= 2.0 - 1e-9
    assert plc.lift_commands[0] == pytest.approx(4.98)  # 5.0 + (-0.2 * 0.1)


def test_plc_actuator_z_setpoint_does_not_overshoot_ascending_target():
    plc = RecordingPLC()
    actuator = PlcActuator(plc, initial_z=1.0, z_hold_band=0.0)
    actuator.set_z_target(3.0)

    for _ in range(300):
        actuator.apply(0.0, 0.0, 0.2, 0.1)

    assert plc.lift_commands[-1] == pytest.approx(3.0)
    assert max(plc.lift_commands) <= 3.0 + 1e-9  # 不越过目标


def test_plc_actuator_confines_z_setpoint_once_inside_hold_band():
    """设定值一旦进入目标 ±z_hold_band 就永久被限死在带内。

    现场故障: Z 在目标附近反复上下升降、收不住。旧逻辑只按逼近方向做单侧钳位,
    另一侧不受限; 而 PD 往往在设定值到达目标之前就进入位置死区、设定值停在目标
    外侧几厘米, 于是反向指令 (来自反馈滞后/量化噪声) 能把它一路推回去。
    """
    plc = RecordingPLC()
    band = PlcActuator._Z_HOLD_BAND_DEFAULT
    target = 1.21
    actuator = PlcActuator(plc, initial_z=1.60)
    actuator.set_z_target(target)

    for _ in range(30):                       # 下放到目标附近
        actuator.apply(0.0, 0.0, -0.2, 0.1)
    for i in range(60):                       # 之后持续换向的速度指令
        actuator.apply(0.0, 0.0, 0.2 if i % 2 else -0.2, 0.1)

    tail = plc.lift_commands[30:]
    assert max(tail) <= target + band + 1e-9
    assert min(tail) >= target - band - 1e-9


def test_plc_actuator_z_setpoint_stops_one_hold_band_past_target():
    """逼近阶段最多走到带的远侧边界 (给受载绳伸长/标定偏差的有界修正权限)。"""
    plc = RecordingPLC()
    actuator = PlcActuator(plc, initial_z=1.0, z_hold_band=0.05)
    actuator.set_z_target(1.1)

    for _ in range(30):
        actuator.apply(0.0, 0.0, 0.2, 0.1)   # 持续上推

    assert max(plc.lift_commands) == pytest.approx(1.15)
    assert all(h <= 1.15 + 1e-9 for h in plc.lift_commands)


def test_plc_actuator_redirects_after_new_z_target():
    """棘轮方向只在本阶段内有效; 注入新目标后按新的高低关系重新定向。"""
    plc = RecordingPLC()
    actuator = PlcActuator(plc, initial_z=1.0, z_hold_band=0.0)
    actuator.set_z_target(1.1)
    for _ in range(10):
        actuator.apply(0.0, 0.0, 0.2, 0.1)
    assert plc.lift_commands[-1] == pytest.approx(1.1)

    actuator.set_z_target(0.6)
    for _ in range(40):
        actuator.apply(0.0, 0.0, -0.2, 0.1)

    assert plc.lift_commands[-1] == pytest.approx(0.6)
    assert min(plc.lift_commands) >= 0.6 - 1e-9


def test_plc_actuator_enforces_minimum_lift_height_floor():
    """The hoist command must never be driven below the safety floor (0.35m),
    keeping the gripper at least 0.35m above ground regardless of PD/target."""
    plc = RecordingPLC()
    actuator = PlcActuator(plc, initial_z=0.42)  # default floor = 0.35m

    # Descend 0.2 m/s: 0.42 -> 0.40 -> ... would pass below 0.35 without the floor.
    for _ in range(7):
        actuator.apply(0.0, 0.0, -0.2, 0.1)

    assert min(plc.lift_commands) == pytest.approx(0.35)
    assert all(h >= 0.35 - 1e-9 for h in plc.lift_commands)
    # Later cycles are pinned to the floor instead of going negative.
    assert plc.lift_commands[-1] == pytest.approx(0.35)


def test_plc_actuator_min_lift_height_is_configurable():
    plc = RecordingPLC()
    actuator = PlcActuator(plc, initial_z=1.0, min_lift_height=0.8)

    for _ in range(20):
        actuator.apply(0.0, 0.0, -0.2, 0.1)

    assert min(plc.lift_commands) == pytest.approx(0.8)


def test_plc_actuator_floor_can_be_disabled_for_full_range():
    """Setting the floor very low restores the un-clamped absolute-height
    behavior for callers that intentionally need the full range."""
    plc = RecordingPLC()
    actuator = PlcActuator(plc, initial_z=-2.0, min_lift_height=float('-inf'))

    actuator.apply(0.0, 0.0, 0.2, 0.1)

    assert plc.lift_commands == pytest.approx([-1.98])


def test_plc_actuator_rejects_motion_when_connection_or_heartbeat_is_unhealthy():
    disconnected = PlcActuator(RecordingPLC(connected=False), initial_z=1.0)
    heartbeat_lost = PlcActuator(
        RecordingPLC(heartbeat_healthy=False),
        initial_z=1.0,
    )

    with pytest.raises(RuntimeError, match="connection"):
        disconnected.apply(0.1, 0.0, 0.0, 0.1)
    with pytest.raises(RuntimeError, match="heartbeat"):
        heartbeat_lost.apply(0.1, 0.0, 0.0, 0.1)


class HoistServoPLC(RecordingPLC):
    """把 liftctrl 当作真实的绝对位置伺服来模拟。

    真实 Z 链路的关键特征 (与 X/Y 完全不同, 也是纯仿真里看不到的部分):
      - liftctrl(h) 收的是绝对目标高度, PLC 内部位置环带惯量/滞后地跟随;
      - 高度反馈 GetActualLiftHeight() 分辨率粗 (默认 1cm) 且只有 10Hz;
      - 没有原生 Z 速度, 速度只能由高度差分估计;
      - feedback_delay: 反馈纯延迟。S7 轮询 + PLC 扫描, 且连接抖动期间
        _LiftHeightSanitizer 会沿用上一次可信读数 (最长 2s), 等效于纯延迟;
      - report_bias: 实测高度与指令参考系的偏差 (受载钢丝绳伸长/标定偏差)。
    """

    def __init__(self, initial_height: float, *, max_rate: float = 0.2,
                 quantum: float = 0.01, loop_gain: float = 2.0, lag: float = 0.5,
                 report_bias: float = 0.0, feedback_delay: int = 0):
        super().__init__()
        self.height = initial_height
        self.setpoint = initial_height
        self.velocity = 0.0
        self.max_rate = max_rate
        self.quantum = quantum
        self.loop_gain = loop_gain
        self.lag = lag
        self.report_bias = report_bias
        self._flicker = False
        self._delay_buffer = [initial_height + report_bias] * feedback_delay

    def lift_ctrl(self, height):
        super().lift_ctrl(height)
        self.setpoint = height

    def step(self, dt: float) -> None:
        want = self.loop_gain * (self.setpoint - self.height)
        want = max(-self.max_rate, min(self.max_rate, want))
        if self.lag > 0.0:
            self.velocity += (want - self.velocity) * min(1.0, dt / self.lag)
        else:
            self.velocity = want
        self.height += self.velocity * dt

    def measured_height(self) -> float:
        """量化到 quantum, 并在相邻两个刻度之间抖动 —— 现场读数的典型表现。

        一次 1cm 抖动对应 0.1m/s 的瞬时差分速度, 经 τ=0.8s 低通后仍有
        0.01m/s 量级, 正是 Z 到位判定被噪声反复清零的来源。
        """
        counts = (self.height + self.report_bias) / self.quantum
        self._flicker = not self._flicker
        value = (math.ceil(counts) if self._flicker else math.floor(counts)) * self.quantum
        if not self._delay_buffer:
            return value
        self._delay_buffer.append(value)
        return self._delay_buffer.pop(0)


class HoistZSource:
    """10Hz 位置源: X/Y 固定 (已在目标), Z 取抓钩量化高度, 无原生 Z 速度。"""

    def __init__(self, plc, xy, dt: float = 0.1):
        self._plc = plc
        self._xy = xy
        self._dt = dt
        self._t = 0.0
        self._stamp = 0

    def reset(self):
        pass

    def get_position(self):
        self._plc.step(self._dt)
        self._t += self._dt
        self._stamp += 1
        return {
            "x": self._xy[0],
            "y": self._xy[1],
            "z": self._plc.measured_height(),
            "vx": 0.0,
            "vy": 0.0,
            "vz": None,          # 抓钩高度没有原生速度, 只能差分估计
            "dt": self._dt,
            "t": self._t,
            "stamp": self._stamp,
        }


def test_z_converges_without_hunting_on_quantized_hoist_feedback():
    """Z 必须在目标处收住并锁轴, 高度指令不得出现上下往复。

    现场故障: Z 有时收不到目标位置, 在目标附近反复上下移动。三处根因:
      1. Z 的 vz 指令被执行器积分成高度设定值 (liftctrl 是位置伺服), 位置环
         本身已含积分环节; 再叠加 ki_pos 就是双重积分 → 目标附近极限环。
      2. 设定值走到目标后仍可被换向指令推着穿越目标 → 被指挥着上下升降。
      3. Z 速度由 1cm/10Hz 高度差分估计, 噪声地板高于 arrival_vel_tol(0.005)
         与 velocity_deadband(0.01) → 到位去抖计数被噪声一帧帧清零, 永远锁不上轴。
    """
    # 2cm 分辨率: 一次刻度跳变的差分速度经低通后仍有 0.02m/s 量级, 正好压过
    # X/Y 用的 arrival_vel_tol(0.005)/velocity_deadband(0.01) —— 用这组容差时,
    # 抓钩明明停在目标 ±1cm 内, Z 也永远锁不上轴, 只能等 PD 超时。
    plc = HoistServoPLC(initial_height=1.60, quantum=0.02)
    actuator = PlcActuator(plc, initial_z=1.60)
    source = HoistZSource(plc, xy=(2.0, 3.0))
    target = (2.0, 3.0, 1.21)

    history, arrivals = run_pd_control(
        source=source,
        actuator=actuator,
        config=CraneConfig(),
        target_pos=target,
        initial_state=CraneState(2.0, 3.0, 1.60),
        max_time=120.0,
        verbose=False,
        is_simulation=False,
    )

    assert {axis for _, axis in arrivals} == {"x", "y", "z"}
    assert abs(plc.height - target[2]) < 0.03

    # 高度指令全程单调下行 (到目标后保持), 没有一次方向反转 = 没有上下往复。
    steps = [
        b - a
        for a, b in zip(plc.lift_commands, plc.lift_commands[1:])
        if abs(b - a) > 1e-9
    ]
    assert steps, "expected the hoist setpoint to actually move"
    assert all(step < 0.0 for step in steps)


def count_swings(series, min_amplitude):
    """统计序列里幅度超过 min_amplitude 的往复段数 (方向反转且走出该幅度)。

    只数"反转次数"会把 1mm 以下、现场根本看不出来的抖动也算进去; 操作员看到的
    "上下移动"是有幅度的往复, 所以按幅度过滤。
    """
    swings = 0
    direction = 0
    extreme = series[0] if series else 0.0
    for previous, current in zip(series, series[1:]):
        step_direction = (current > previous) - (current < previous)
        if step_direction == 0:
            continue
        if direction and step_direction != direction:
            if abs(current - extreme) > min_amplitude:
                swings += 1
                extreme = current
        else:
            extreme = current
        direction = step_direction
    return swings


def test_z_does_not_hunt_when_height_feedback_lags_behind():
    """反馈纯延迟 (S7 轮询/PLC 扫描, 抖动期间还会沿用旧读数) 是 Z 在目标附近
    上下往复的放大器: 环路里已有"速度指令→高度设定值"这一个积分, 再叠加积分项
    就是双重积分, 加上纯延迟必然自激; 而设定值一旦越过目标就不再受单调钳位
    约束, 于是能被推得很远, 抓钩跟着大幅上下跑。

    同场景修复前: PD 跑到 120s 超时都收不住, 高度设定值被推离目标 4~15cm 并反复
    换向 (抓钩肉眼可见地上下跑)。修复后要求: 正常收敛、设定值越过目标不超过微调带、
    进入目标附近后不再出现有幅度的往复。
    """
    target_z = 1.21
    plc = HoistServoPLC(
        initial_height=1.60,
        report_bias=-0.03,   # 受载绳伸长: 实测比指令参考系低 3cm
        feedback_delay=15,   # 1.5s 反馈纯延迟
    )
    actuator = PlcActuator(plc, initial_z=1.60)
    source = HoistZSource(plc, xy=(2.0, 3.0))

    _, arrivals = run_pd_control(
        source=source,
        actuator=actuator,
        config=CraneConfig(),
        target_pos=(2.0, 3.0, target_z),
        initial_state=CraneState(2.0, 3.0, 1.60),
        max_time=120.0,
        verbose=False,
        is_simulation=False,
    )

    assert {axis for _, axis in arrivals} == {"x", "y", "z"}

    commands = plc.lift_commands
    band = PlcActuator._Z_HOLD_BAND_DEFAULT
    assert min(commands) >= target_z - band - 1e-9   # 下降方向不会越过目标跑远

    # 进入目标 ±0.10m 之后, 高度指令不得再出现幅度超过 2mm 的往复。
    near_start = next(
        i for i, height in enumerate(commands) if abs(height - target_z) < 0.10
    )
    assert count_swings(commands[near_start:], 0.002) == 0


def test_timeout_always_emergency_stops_and_cleans_up():
    actuator = StubActuator()
    source = SequenceSource([position(t=1.0)])

    with pytest.raises(TimeoutError, match="did not finish"):
        run_pd_control(
            source=source,
            actuator=actuator,
            config=CraneConfig(),
            target_pos=(1.0, 1.0, 1.0),
            initial_state=CraneState(0.0, 0.0, 0.0),
            max_time=0.5,
            verbose=False,
            is_simulation=False,
        )

    assert actuator.emergency_stop_calls == 1
    assert actuator.cleanup_calls == 1


def test_successful_arrival_explicitly_stops_motion_and_cleans_up():
    actuator = StubActuator()
    config = CraneConfig()
    # 到位需连续 arrival_debounce_cycles 帧满足条件才锁轴 (防单帧异常值误锁)。
    frames = [
        position(x=0.01, y=0.01, z=0.01, t=0.1 * (i + 1), stamp=i + 1)
        for i in range(config.arrival_debounce_cycles)
    ]
    source = SequenceSource(frames)

    _, arrivals = run_pd_control(
        source=source,
        actuator=actuator,
        config=config,
        target_pos=(0.01, 0.01, 0.01),
        initial_state=CraneState(0.01, 0.01, 0.01),
        verbose=False,
        is_simulation=False,
    )

    assert {axis for _, axis in arrivals} == {"x", "y", "z"}
    assert actuator.stop_calls == 1
    assert actuator.emergency_stop_calls == 0
    assert actuator.cleanup_calls == 1


def test_localization_outlier_frame_is_discarded_and_does_not_lock_axis():
    """单帧定位跳变(异常值)必须被丢弃, 不得进入控制/误触发到位锁轴。

    复现"某轴离目标十几厘米就停、PD 结束"的根因: 一个恰好落在目标附近或
    幅值离谱的定位跳变帧, 在旧逻辑里会被单帧判定直接锁轴。现要求既能过滤
    离谱跳变帧, 又需连续多帧才锁轴。
    """
    actuator = StubActuator()
    config = CraneConfig()
    n = config.arrival_debounce_cycles
    frames = [position(x=0.01, y=0.01, z=0.01, t=0.1, stamp=1)]
    # 中间插入一个远超物理可达 (max_v*dt+margin) 的跳变帧, 必须被丢弃。
    frames.append(position(x=10.0, y=10.0, z=10.0, t=0.2, stamp=2))
    for i in range(1, n):
        frames.append(position(x=0.01, y=0.01, z=0.01, t=0.1 * (i + 2), stamp=i + 2))
    source = SequenceSource(frames)

    _, arrivals = run_pd_control(
        source=source,
        actuator=actuator,
        config=config,
        target_pos=(0.01, 0.01, 0.01),
        initial_state=CraneState(0.01, 0.01, 0.01),
        verbose=False,
        is_simulation=False,
    )

    assert {axis for _, axis in arrivals} == {"x", "y", "z"}
    # 跳变帧在进入 apply 之前就被丢弃: apply 次数 == 被接受的帧数 (n)。
    assert len(actuator.apply_calls) == n


def test_single_invalid_frame_mid_run_is_tolerated_not_fatal():
    """一次偶发坏帧 (NaN/dt异常) 不该让整段 PD 直接中止。

    复现"运行过程中 PD 算法控制有时候会中途提前结束"的一个根因: 单帧定位
    异常在旧逻辑里直接 raise PositionFeedbackError, 整个作业跟着结束, 需要
    操作员重新下发目标。现要求预算内的偶发坏帧被丢弃, 控制继续直至到位。
    """
    actuator = StubActuator()
    config = CraneConfig()
    n = config.arrival_debounce_cycles
    frames = [position(x=0.01, y=0.01, z=0.01, t=0.1, stamp=1)]
    # 中间插入一帧 dt=NaN 的畸形数据, 必须被丢弃而不是终止整个 PD。
    frames.append({**position(x=0.01, y=0.01, z=0.01, t=0.2, stamp=2), "dt": math.nan})
    for i in range(1, n):
        frames.append(position(x=0.01, y=0.01, z=0.01, t=0.1 * (i + 2), stamp=i + 2))
    source = SequenceSource(frames)

    _, arrivals = run_pd_control(
        source=source,
        actuator=actuator,
        config=config,
        target_pos=(0.01, 0.01, 0.01),
        initial_state=CraneState(0.01, 0.01, 0.01),
        verbose=False,
        is_simulation=False,
    )

    assert {axis for _, axis in arrivals} == {"x", "y", "z"}


def test_persistent_invalid_frames_beyond_budget_still_abort():
    """坏帧容忍预算不是无限的: 连续异常帧超出预算必须视为真实故障并
    按原语义中止 + 安全停车, 而不是无限期容忍下去。"""
    actuator = StubActuator()
    config = CraneConfig()
    frames = [position(x=0.0, y=0.0, z=0.0, t=0.1, stamp=1)]
    # 预算 + 1 帧连续畸形数据 -> 最后一帧必须真正触发中止。
    for i in range(config.max_consecutive_bad_frames + 1):
        frames.append({**position(t=0.1 * (i + 2), stamp=i + 2), "dt": math.nan})
    source = SequenceSource(frames)

    with pytest.raises(RuntimeError, match="invalid position feedback"):
        run_pd_control(
            source=source,
            actuator=actuator,
            config=config,
            target_pos=(1.0, 1.0, 1.0),
            initial_state=CraneState(0.0, 0.0, 0.0),
            verbose=False,
            is_simulation=False,
        )

    assert actuator.emergency_stop_calls == 1


class FlakyApplyActuator(StubActuator):
    """Simulates actuator.apply() raising RuntimeError for a transient burst
    of cycles (PLC connection/heartbeat blip) before recovering — like the
    _ensure_available() guard in PlcActuator would raise while the heartbeat
    thread is mid-recovery."""

    def __init__(self, fail_calls: int):
        super().__init__()
        self._fail_calls = fail_calls
        self._calls = 0

    def apply(self, vx, vy, vz, dt):
        self._calls += 1
        if self._calls <= self._fail_calls:
            raise RuntimeError('PLC heartbeat is not healthy')
        super().apply(vx, vy, vz, dt)


def test_transient_actuator_error_mid_run_is_tolerated_not_fatal():
    """一次瞬时 actuator.apply() 失败 (PLC 连接/心跳抖动) 不该让整段 PD
    直接中止——否则表现为"距离目标还很远时 PD 就结束、输出速度为 0"。"""
    actuator = FlakyApplyActuator(fail_calls=3)
    config = CraneConfig()
    n = config.arrival_debounce_cycles
    frames = [
        position(x=0.01, y=0.01, z=0.01, t=0.1 * (i + 1), stamp=i + 1)
        for i in range(3 + n)
    ]
    source = SequenceSource(frames)

    _, arrivals = run_pd_control(
        source=source,
        actuator=actuator,
        config=config,
        target_pos=(0.01, 0.01, 0.01),
        initial_state=CraneState(0.01, 0.01, 0.01),
        verbose=False,
        is_simulation=False,
    )

    assert {axis for _, axis in arrivals} == {"x", "y", "z"}
    # The 3 failed calls should not have reached the recording apply().
    assert len(actuator.apply_calls) == n


def test_persistent_actuator_errors_beyond_budget_still_abort():
    """执行器失败容忍预算不是无限的: 连续失败超出预算必须视为真实故障并
    按原语义中止 + 安全停车。"""
    config = CraneConfig()
    actuator = FlakyApplyActuator(fail_calls=config.max_consecutive_actuator_errors + 1)
    frames = [
        position(t=0.1 * (i + 1), stamp=i + 1)
        for i in range(config.max_consecutive_actuator_errors + 1)
    ]
    source = SequenceSource(frames)

    with pytest.raises(RuntimeError, match="PLC heartbeat is not healthy"):
        run_pd_control(
            source=source,
            actuator=actuator,
            config=config,
            target_pos=(1.0, 1.0, 1.0),
            initial_state=CraneState(0.0, 0.0, 0.0),
            verbose=False,
            is_simulation=False,
        )

    assert actuator.emergency_stop_calls == 1


class FlakyHeartbeatPLC(MockPLC):
    """Simulates a transient network blip: heartbeat sends fail for a burst
    of cycles, then start succeeding again — like real S7/TCP jitter."""

    def __init__(self, fail_cycles: int):
        super().__init__()
        self._connected = True
        self._sends = 0
        self._fail_cycles = fail_cycles

    def send_heartbeat(self) -> bool:
        self._sends += 1
        ok = self._sends > self._fail_cycles
        if ok:
            self._fail_count = 0
            self._heartbeat_healthy = True
        return ok


def test_heartbeat_self_heals_after_transient_failure_burst():
    """一次瞬时网络抖动(心跳连续失败超过阈值)必须只是短暂标记不健康,
    绝不能让后台心跳线程退出——线程退出后 heartbeat_healthy 永远无法恢复,
    只能重启整个进程才能继续控制。这是"PD 有时中途提前结束、且之后重新
    下发目标也无法恢复"的根因之一。"""
    plc = FlakyHeartbeatPLC(fail_cycles=15)  # > _max_fails(10), 模拟瞬时抖动
    plc.start_heartbeat()
    try:
        deadline = time.monotonic() + 3.0
        while plc.heartbeat_healthy and time.monotonic() < deadline:
            time.sleep(0.02)
        assert not plc.heartbeat_healthy, 'sustained failures should mark unhealthy'
        assert plc._heartbeat_thread.is_alive(), 'heartbeat thread must keep retrying, not exit'

        deadline = time.monotonic() + 3.0
        while not plc.heartbeat_healthy and time.monotonic() < deadline:
            time.sleep(0.02)
        assert plc.heartbeat_healthy, 'heartbeat must self-heal once sends succeed again'
        assert plc._heartbeat_thread.is_alive()
    finally:
        plc.disconnect()


def test_position_timeout_is_an_error_instead_of_normal_completion():
    actuator = StubActuator()
    source = SequenceSource([None])

    with pytest.raises(RuntimeError, match="position feedback timed out"):
        run_pd_control(
            source=source,
            actuator=actuator,
            config=CraneConfig(),
            target_pos=(1.0, 1.0, 1.0),
            initial_state=CraneState(0.0, 0.0, 0.0),
            verbose=False,
            is_simulation=False,
        )

    assert actuator.emergency_stop_calls == 1
    assert actuator.cleanup_calls == 1


@pytest.mark.parametrize(
    "invalid_position",
    [
        position(x=math.nan),
        {**position(), "dt": math.nan},
        {**position(), "dt": 0.0},
        {**position(), "vx": math.inf},
    ],
)
def test_invalid_ros_feedback_is_rejected_before_motion(invalid_position):
    actuator = StubActuator()
    source = SequenceSource([invalid_position])

    with pytest.raises(RuntimeError, match="invalid position feedback"):
        run_pd_control(
            source=source,
            actuator=actuator,
            config=CraneConfig(),
            target_pos=(1.0, 1.0, 1.0),
            initial_state=CraneState(0.0, 0.0, 0.0),
            verbose=False,
            is_simulation=False,
        )

    assert actuator.apply_calls == []
    assert actuator.emergency_stop_calls == 1


def test_stop_requested_while_waiting_for_position_never_applies_motion():
    actuator = StubActuator()

    class StopDuringReadHooks(ControlHooks):
        def __init__(self):
            self.stopped = False

        def should_stop(self):
            return self.stopped

    hooks = StopDuringReadHooks()

    class StopDuringReadSource(SequenceSource):
        def get_position(self):
            hooks.stopped = True
            return super().get_position()

    source = StopDuringReadSource([position()])

    with pytest.raises(RuntimeError, match="stopped"):
        run_pd_control(
            source=source,
            actuator=actuator,
            config=CraneConfig(),
            target_pos=(1.0, 1.0, 1.0),
            initial_state=CraneState(0.0, 0.0, 0.0),
            hooks=hooks,
            verbose=False,
            is_simulation=False,
        )

    assert actuator.apply_calls == []
    assert actuator.emergency_stop_calls == 1


@pytest.mark.parametrize(
    "target",
    [
        (math.nan, 0.0, 0.0),
        (math.inf, 0.0, 0.0),
    ],
)
def test_config_rejects_non_finite_targets(target):
    with pytest.raises(ValueError):
        CraneConfig().validate_target(target)


def test_negative_z_is_allowed_until_a_workspace_bound_is_configured():
    assert CraneConfig().validate_target((1.0, 2.0, -3.0)) == (1.0, 2.0, -3.0)
    assert CraneConfig().validate_position((1.0, 2.0, -3.0)) == (1.0, 2.0, -3.0)

    config = CraneConfig(workspace_z_bounds=(0.0, 10.0))
    with pytest.raises(ValueError, match="Z target"):
        config.validate_target((1.0, 2.0, -3.0))


def test_missing_native_z_velocity_uses_height_estimate_for_arrival():
    class PreserveMissingVelocityActuator(StubActuator):
        def update_state(self, state, feedback):
            state.x.position = feedback["x"]
            state.y.position = feedback["y"]
            state.z.position = feedback["z"]
            for axis_name in ("x", "y", "z"):
                velocity = feedback[f"v{axis_name}"]
                if velocity is not None:
                    getattr(state, axis_name).velocity = velocity

    class StopAfterFirstStep(ControlHooks):
        def __init__(self):
            self.stop_requested = False
            self.arrivals = []

        def on_step(self, step_data):
            self.stop_requested = True

        def on_arrival(self, axis, t):
            self.arrivals.append(axis)

        def should_stop(self):
            return self.stop_requested

    feedback = position(z=0.01)
    feedback["vz"] = None
    hooks = StopAfterFirstStep()

    with pytest.raises(RuntimeError, match="stopped"):
        run_pd_control(
            source=SequenceSource([feedback]),
            actuator=PreserveMissingVelocityActuator(),
            config=CraneConfig(),
            target_pos=(0.0, 0.0, 0.01),
            initial_state=CraneState(0.0, 0.0, 0.0),
            hooks=hooks,
            verbose=False,
            is_simulation=False,
        )

    assert "z" not in hooks.arrivals


def test_config_enforces_configured_workspace_bounds():
    config = CraneConfig(
        workspace_x_bounds=(0.0, 20.0),
        workspace_y_bounds=(-5.0, 5.0),
        workspace_z_bounds=(0.0, 10.0),
    )

    assert config.validate_target((10.0, 0.0, 5.0)) == (10.0, 0.0, 5.0)
    with pytest.raises(ValueError, match="X target"):
        config.validate_target((21.0, 0.0, 5.0))


def test_control_state_distinguishes_operator_stop_from_success():
    state = ControlState()
    state.set_start(
        pos={"x": 0.0, "y": 0.0, "z": 0.0},
        target={"x": 1.0, "y": 1.0, "z": 1.0},
    )

    state.set_stopped("Stopped by operator")

    snapshot = state.snapshot()
    assert snapshot["running"] is False
    assert snapshot["done"] is False
    assert snapshot["stopped"] is True
    assert snapshot["stop_reason"] == "Stopped by operator"


def test_missing_plc_library_requires_explicit_mock_opt_in(tmp_path):
    missing_library = tmp_path / "missing-libsscarctrl.so"

    with pytest.raises(RuntimeError, match="Cannot load PLC library"):
        create_plc(lib_path=str(missing_library))

    assert isinstance(
        create_plc(lib_path=str(missing_library), allow_mock=True),
        MockPLC,
    )


def test_web_target_parser_uses_control_config_validation():
    config = CraneConfig(workspace_x_bounds=(0.0, 5.0))

    assert live_server._parse_control_target(
        json.dumps({"target_x": 2, "target_y": 1, "target_z": 3}),
        config,
    ) == (2.0, 1.0, 3.0)

    with pytest.raises(ValueError, match="finite"):
        live_server._parse_control_target(
            '{"target_x": NaN, "target_y": 1, "target_z": 3}',
            config,
        )
    with pytest.raises(ValueError, match="X target"):
        live_server._parse_control_target(
            json.dumps({"target_x": 6, "target_y": 1, "target_z": 3}),
            config,
        )


def test_control_run_reservation_allows_only_one_concurrent_starter():
    server = CraneLiveServer(("127.0.0.1", 0), payload=None)
    barrier = threading.Barrier(8)
    reservations = []
    result_lock = threading.Lock()

    def reserve():
        barrier.wait()
        result = server.reserve_control_run()
        with result_lock:
            reservations.append(result)

    threads = [threading.Thread(target=reserve) for _ in range(8)]
    try:
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        assert reservations.count(True) == 1
        assert reservations.count(False) == 7
        server.release_control_run()
        assert server.reserve_control_run() is True
    finally:
        server.release_control_run()
        server.server_close()


def test_cli_builds_workspace_bounds_and_explicit_mock_mode():
    args = main._build_arg_parser().parse_args(
        [
            "--allow-mock-plc",
            "--workspace-x-min", "0",
            "--workspace-x-max", "30",
            "--workspace-y-min", "-10",
            "--workspace-y-max", "10",
            "--workspace-z-min", "0",
            "--workspace-z-max", "15",
        ]
    )

    config = main._config_from_args(args)

    assert args.allow_mock_plc is True
    assert config.workspace_x_bounds == (0.0, 30.0)
    assert config.workspace_y_bounds == (-10.0, 10.0)
    assert config.workspace_z_bounds == (0.0, 15.0)


def test_cli_rejects_half_configured_workspace_bounds():
    args = main._build_arg_parser().parse_args(["--workspace-x-min", "0"])

    with pytest.raises(ValueError, match="workspace X"):
        main._config_from_args(args)


def test_plc_connection_failure_stops_startup(monkeypatch):
    class FailedPLC(RecordingPLC):
        def __init__(self):
            super().__init__(connected=False, heartbeat_healthy=False)
            self.disconnected = False
            self.heartbeat_started = False

        def connect(self, ip):
            return 7

        def start_heartbeat(self):
            self.heartbeat_started = True

        def disconnect(self):
            self.disconnected = True

    plc = FailedPLC()
    monkeypatch.setattr(main, "create_plc", lambda **kwargs: plc)
    args = main._build_arg_parser().parse_args(["--plc-ip", "192.168.0.1"])

    with pytest.raises(ConnectionError, match="ret=7"):
        main._connect_plc(args)

    assert plc.heartbeat_started is False
    assert plc.disconnected is True


# ---------------------------------------------------------------------------
# _LiftHeightSanitizer — 过滤 GetActualLiftHeight() 的异常读数
# ---------------------------------------------------------------------------

def test_lift_height_sanitizer_passes_through_stable_readings():
    """连续稳定的合理读数应原样通过, 不被误判。"""
    s = _LiftHeightSanitizer()
    t = 1000.0
    for raw in (1.60, 1.61, 1.59, 1.62):
        value, reason = s.sanitize(raw, now=t)
        assert value == pytest.approx(raw)
        assert reason is None
        t += 0.1
    assert s.bad_streak == 0


def test_lift_height_sanitizer_rejects_near_zero_sentinel_and_holds_last_good():
    """复现现场日志: 心跳/连接抖动期间 GetActualLiftHeight() 返回 0 (或极小
    的 denormalized 垂圾值) 作兜底, 必须被丢弃并沿用上一次可信读数, 而不是
    让 Z 出现"跳到 0 再跳回正确高度"的抽动。"""
    s = _LiftHeightSanitizer()
    value, reason = s.sanitize(1.59695, now=0.0)
    assert value == pytest.approx(1.59695)
    assert reason is None

    # 0 作兜底值
    value, reason = s.sanitize(0.0, now=0.1)
    assert value == pytest.approx(1.59695)
    assert reason == 'near-zero sentinel'
    assert s.bad_streak == 1

    # 形如日志中 4.49863e-312 的 denormalized 垂圾值, 同样被视为"近零异常"。
    value, reason = s.sanitize(4.49863e-312, now=0.2)
    assert value == pytest.approx(1.59695)
    assert reason == 'near-zero sentinel'
    assert s.bad_streak == 2

    # 恢复正常读数后立即采纳, 计数器清零。
    value, reason = s.sanitize(1.59695, now=0.3)
    assert value == pytest.approx(1.59695)
    assert reason is None
    assert s.bad_streak == 0


def test_lift_height_sanitizer_rejects_implausible_jump_and_nan():
    s = _LiftHeightSanitizer()
    s.sanitize(1.60, now=0.0)

    # 单个 ~0.1s 周期内不可能出现的巨大跳变 (非近零, 走跳变分支)。
    value, reason = s.sanitize(5.0, now=0.1)
    assert value == pytest.approx(1.60)
    assert reason == 'implausible jump'

    # NaN/Inf 一律拒绝。
    value, reason = s.sanitize(math.nan, now=0.2)
    assert value == pytest.approx(1.60)
    assert reason == 'non-finite'
    value, reason = s.sanitize(math.inf, now=0.3)
    assert value == pytest.approx(1.60)
    assert reason == 'non-finite'


def test_lift_height_sanitizer_gives_up_cached_value_after_stale_timeout():
    """异常读数容忍窗口不是无限的: 持续异常超过 STALE_TIMEOUT 后不再假装
    缓存值可信, 返回 None 交给上层 (通常回退 SLAM Z 或判定不可用)。"""
    s = _LiftHeightSanitizer()
    s.sanitize(1.60, now=0.0)

    value, reason = s.sanitize(0.0, now=1.0)
    assert value == pytest.approx(1.60)  # 仍在容忍窗口内
    assert reason == 'near-zero sentinel'

    value, reason = s.sanitize(0.0, now=1.0 + _LiftHeightSanitizer.STALE_TIMEOUT + 0.1)
    assert value is None  # 超出容忍窗口, 放弃缓存值
    assert reason == 'near-zero sentinel'


def test_lift_height_sanitizer_reset_clears_cached_state():
    s = _LiftHeightSanitizer()
    s.sanitize(1.60, now=0.0)
    s.sanitize(0.0, now=0.1)
    assert s.bad_streak == 1

    s.reset()

    # 重置后没有缓存值, 任意有限读数都直接采纳 (不会被当成"近零异常"拒绝)。
    value, reason = s.sanitize(0.0, now=0.2)
    assert value == pytest.approx(0.0)
    assert reason is None
    assert s.bad_streak == 0
