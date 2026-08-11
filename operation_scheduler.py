"""
Operation Scheduler — 完整起重机作业流程编排

根据"lst完整调度流程"文档实现两阶段作业:
  Phase 0 (预检): 取货前确认抓钩已开启, 未开启则打开
  Phase 1 (取货): 初始位置 → 起始位置 → 夹取钢卷
  Phase 2 (运输): 起始位置 → 目标位置 → 释放钢卷
  Phase 3 (归位): Z 轴抬升到安全高度

安全高度约定 (绝对高度, 相对于地面):
  self._config.approach_safe_z = 1.0m  — 取货阶段安全高度
  self._config.transport_safe_z = 1.5m — 运输阶段安全高度
  Z_SAFE_FINAL     = 1.6m — 作业完成后 Z 归位高度

时序约定:
  取货前 → 检查抓钩状态, 关闭 (夹紧) 则打开一次, 不做持续控制/盯守
  XY 到达后 → 自适应判稳 (上限 stabilize_delay) → Z 下降
  Z 下降到位后 → 自适应判稳 (上限 gripper_settle_max_wait) → 抓钩夹取/释放
  抓钩动作确认后安全等待 0.5s (机械动作确认延时, 与摆动无关, 固定值)
  释放完全确认后 → 停留 post_release_lift_delay (~2s) → 直接抬升到目标高度

自适应判稳 (效率与安全兼顾, 替代"盲等固定时长"):
  用实测位置反馈判断货物是否真正静止 (速度 + 滑动窗口位置峰峰值双重判据)。
  已经静止 → 提前放行, 不必等满上限时长 (提升效率);
  仍在摆动/回弹 → 持续等待直到真正平稳, 而不是等满固定时长后就不管
  三七二十一继续抓钩动作 (这正是"货物未放稳/未释放完就抓钩"风险的根源)。
  超时兜底: 与抓钩状态确认超时一致, 打印警告后继续执行 (软失败), 避免
  传感器/反馈异常误伤正常作业。详见 OperationScheduler._wait_cargo_settled()。

抓钩状态检查:
  优先通过 PLC 函数 GetGripperOnStatus/GetGripperOffStatus 读取
  可选通过 ROS /crane/cmd_vel/anguler 的 angular.x 读取
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable

from crane_model import (
    Actuator,
    ControlHooks,
    ControlStoppedError,
    CraneConfig,
    CraneState,
    PositionFeedbackTimeout,
    PositionSource,
    run_pd_control,
)
from plc_interface import PLCInterface


# ---------------------------------------------------------------------------
# 安全高度常量 [m] (绝对高度, 相对于地面) — 默认值, 由 CraneConfig 覆盖
# ---------------------------------------------------------------------------

Z_SAFE_APPROACH = 1.0   # 取货阶段安全高度 (默认, 可被 CraneConfig.approach_safe_z 覆盖)
Z_SAFE_TRANSPORT = 1.5  # 运输阶段安全高度 (默认, 可被 CraneConfig.transport_safe_z 覆盖)
Z_SAFE_FINAL = 1.6      # 作业完成后归位高度 (默认, 可被 CraneConfig.return_safe_z 覆盖)

# 时序常量 [s] — 默认值, 由 CraneConfig 覆盖
STABILIZE_DELAY = 1.0       # XY 到达后抓钩稳定等待
GRIPPER_SAFETY_DELAY = 0.5  # 抓钩动作确认后安全等待
GRIPPER_CHECK_INTERVAL = 0.1  # 抓钩状态轮询间隔 [s]
GRIPPER_CHECK_TIMEOUT = 5.0   # 抓钩状态确认超时 [s]

# 判稳轮询周期上限 [s] — 仅用于仿真模式节流 (SimPositionSource 非阻塞,
# 若不加此 sleep 会以 CPU 全速空转); PLC 模式下 RosPositionSource.get_position()
# 本身以 ~10Hz 阻塞等待新数据, 已有天然节流, 不需要额外 sleep。
HOOK_SETTLE_POLL_INTERVAL = 0.02


# ---------------------------------------------------------------------------
# 作业阶段
# ---------------------------------------------------------------------------

class OperationPhase(Enum):
    """作业阶段枚举 — 对应 UI 显示的进度信息"""
    IDLE = auto()                    # 等待开始
    ENSURE_GRIPPER_OPEN = auto()     # Phase 0: 取货前确认抓钩已开启
    APPROACH_XY = auto()             # Phase 1a: 三轴联动接近取货位置
    APPROACH_Z_DESCEND = auto()      # Phase 1b: Z 下降到取货高度
    GRIPPER_CLAMP = auto()           # Phase 1c: 夹取钢卷
    LIFT_CARGO = auto()              # Phase 2a: 带货上升到 1.0m
    TRANSPORT_XY = auto()            # Phase 2b: 三轴联动运输到目标
    TRANSPORT_Z_DESCEND = auto()     # Phase 2c: Z 下降到卸货高度
    GRIPPER_RELEASE = auto()         # Phase 2d: 释放钢卷
    RETURN_Z = auto()                # Phase 3: Z 归位到 1.6m (含柔性起升)
    DONE = auto()                    # 作业完成
    ERROR = auto()                   # 异常中止
    STOPPED = auto()                 # 操作员停止

    @property
    def label(self) -> str:
        labels = {
            OperationPhase.IDLE:                    "等待开始",
            OperationPhase.ENSURE_GRIPPER_OPEN:     "Phase 0: 确认抓钩已开启",
            OperationPhase.APPROACH_XY:             "Phase 1a: 接近取货位置 (Z→1.0m)",
            OperationPhase.APPROACH_Z_DESCEND:      "Phase 1b: Z 下降到取货高度",
            OperationPhase.GRIPPER_CLAMP:           "Phase 1c: 夹取钢卷",
            OperationPhase.LIFT_CARGO:              "Phase 2a: 带货上升到安全高度",
            OperationPhase.TRANSPORT_XY:            "Phase 2b: 运输到目标位置 (Z→1.5m)",
            OperationPhase.TRANSPORT_Z_DESCEND:     "Phase 2c: Z 下降到卸货高度",
            OperationPhase.GRIPPER_RELEASE:         "Phase 2d: 释放钢卷",
            OperationPhase.RETURN_Z:                "Phase 3: Z 归位到 1.6m",
            OperationPhase.DONE:                    "作业完成",
            OperationPhase.ERROR:                   "异常中止",
            OperationPhase.STOPPED:                 "操作员停止",
        }
        return labels[self]


# ---------------------------------------------------------------------------
# 调度结果
# ---------------------------------------------------------------------------

@dataclass
class OperationResult:
    """一次完整作业的结果"""
    success: bool
    phase: OperationPhase
    message: str = ""
    total_time: float = 0.0
    history: list[dict] | None = None
    phase_history: list[tuple[float, OperationPhase]] | None = None


# ---------------------------------------------------------------------------
# 调度钩子 — 供 UI 轮询进度
# ---------------------------------------------------------------------------

class SchedulerHooks:
    """调度器回调钩子 — 供 UI 轮询作业进度。

    每个阶段切换时更新, 前端通过 /api/control-state 轮询获取。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._stop_flag = threading.Event()
        self.phase: OperationPhase = OperationPhase.IDLE
        self.phase_label: str = OperationPhase.IDLE.label
        self.message: str = ""
        self.step_count: int = 0
        self.done: bool = False
        self.error: str | None = None
        self.stopped: bool = False
        self.stop_reason: str | None = None

    def set_phase(self, phase: OperationPhase, message: str = "") -> None:
        with self._lock:
            self.phase = phase
            self.phase_label = phase.label
            self.message = message
            self.step_count += 1

    def set_done(self) -> None:
        with self._lock:
            self.phase = OperationPhase.DONE
            self.phase_label = OperationPhase.DONE.label
            self.done = True

    def set_error(self, msg: str) -> None:
        with self._lock:
            self.phase = OperationPhase.ERROR
            self.phase_label = OperationPhase.ERROR.label
            self.error = msg
            self.done = True

    def set_stopped(self, reason: str) -> None:
        with self._lock:
            self.phase = OperationPhase.STOPPED
            self.phase_label = OperationPhase.STOPPED.label
            self.stopped = True
            self.stop_reason = reason
            self.done = True

    def should_stop(self) -> bool:
        return self._stop_flag.is_set()

    def stop(self) -> None:
        self._stop_flag.set()

    def snapshot(self) -> dict:
        with self._lock:
            return {
                'phase': self.phase.name,
                'phase_label': self.phase_label,
                'message': self.message,
                'step_count': self.step_count,
                'done': self.done,
                'error': self.error,
                'stopped': self.stopped,
                'stop_reason': self.stop_reason,
            }


# ---------------------------------------------------------------------------
# 调度器
# ---------------------------------------------------------------------------

class OperationScheduler:
    """完整起重机作业调度器。

    对每次 PD 运动阶段调用 run_pd_control(), 在阶段之间处理抓钩动作和
    安全等待。内部维护一个共享的 ControlHooks 适配器, 将每步 PD 状态
    推送到 SchedulerHooks 供前端轮询。

    Usage:
        scheduler = OperationScheduler(plc, ros_source, plc_actuator, config)
        result = scheduler.execute(start_pos=(x1,y1,z1), target_pos=(x2,y2,z2))
    """

    # 最大单次 PD 运行时间 [s] — 单次移动不应超过此值
    _PD_MAX_TIME = 300.0  # 5 min per PD segment

    def __init__(
        self,
        plc: PLCInterface,
        source: PositionSource,
        actuator: Actuator,
        config: CraneConfig,
        *,
        is_simulation: bool = False,
        gripper_provider: Callable[[], tuple[bool | None, bool | None]] | None = None,
        coordinate_transform: object | None = None,
        z_is_hoist_height: bool = False,
    ):
        """
        Args:
            plc:               PLC 接口 (用于抓钩控制和状态读取)
            source:            位置反馈源
            actuator:          执行器
            config:            控制配置参数
            is_simulation:     True=仿真模式, False=PLC 模式
            gripper_provider:  可选的抓钩状态提供者: () -> (clamped, released)
            coordinate_transform: 坐标变换 (crane→map), 用于前端展示
            z_is_hoist_height: Z 是否来自抓钩实测高度
        """
        self._plc = plc
        self._source = source
        self._actuator = actuator
        self._config = config
        self._is_simulation = is_simulation
        self._gripper_provider = gripper_provider
        self._coordinate_transform = coordinate_transform
        self._z_is_hoist_height = z_is_hoist_height

        # 共享调度钩子
        self.hooks = SchedulerHooks()

        # ControlState 引用 — execute() 时注入, _run_pd 中用于转发 PD 步进数据
        self._control_state: object | None = None

        # PD 控制钩子适配器 — 将每步 PD 状态写入 ControlState (供轮询)
        self._pd_hooks: _PdToSchedulerAdapter | None = None

    # ------------------------------------------------------------------
    # 公开接口
    # ------------------------------------------------------------------

    def execute(
        self,
        start_pos: tuple[float, float, float],
        target_pos: tuple[float, float, float],
        *,
        control_state: object | None = None,
    ) -> OperationResult:
        """执行完整作业序列 (阻塞, 应在后台线程中调用)。

        Args:
            start_pos:  起始(取货)位置 (sx, sy, sz) [m] — 起重机坐标系
            target_pos: 目标(卸货)位置 (tx, ty, tz) [m] — 起重机坐标系
            control_state: 可选 ControlState 对象, 用于前端轮询

        Returns:
            OperationResult 包含成功/失败状态和阶段历史
        """
        # 校验位置合法性 (由 config 的 workspace 约束和有限性检查)
        self._config.validate_target(start_pos)
        self._config.validate_target(target_pos)

        sx, sy, sz = start_pos
        tx, ty, tz = target_pos
        t_start = time.monotonic()
        phase_history: list[tuple[float, OperationPhase]] = []
        all_history: list[dict] = []

        # 保存 control_state 引用供 _run_pd 使用
        self._control_state = control_state

        def _record_phase(phase: OperationPhase) -> None:
            elapsed = time.monotonic() - t_start
            phase_history.append((elapsed, phase))
            self.hooks.set_phase(phase)
            if control_state is not None:
                self._update_control_state_phase(control_state, phase)

        try:
            # ================================================================
            # Phase 0: 取货前确认抓钩已开启 (释放状态)
            # 保证接近取货位置前抓钩处于打开状态——避免上次作业异常结束
            # (急停/故障) 后抓钩仍停留在夹紧状态, 这次直接靠上钢卷去"夹"
            # 一个已经夹紧的抓钩, 或带着夹紧状态误碰货物。
            # 只是一次性的状态判断 + 纠正: 关闭则打开一次即可, 不需要
            # 像正式抓取动作那样持续控制/额外安全等待——后续还要走行到
            # 取货位置, 本身就有足够的时间富余。
            # ================================================================
            _record_phase(OperationPhase.ENSURE_GRIPPER_OPEN)
            if self.hooks.should_stop():
                raise ControlStoppedError("操作员停止")

            if not self._check_gripper_released():
                print("[Scheduler] 取货前检测到抓钩未开启, 打开抓钩...")
                self._plc.gripper_release()
                self._wait_gripper_released()
            else:
                print("[Scheduler] 取货前确认抓钩已开启")

            # ================================================================
            # Phase 1a: 三轴联动 → 取货位置 (sx, sy, self._config.approach_safe_z)
            # Z 先到安全高度 1.0m 则等待, XY 到 sx,sy 后继续
            # ================================================================
            _record_phase(OperationPhase.APPROACH_XY)
            if self.hooks.should_stop():
                raise ControlStoppedError("操作员停止")

            current_state = self._get_current_state()
            hist, _ = self._run_pd(
                target=(sx, sy, self._config.approach_safe_z),
                initial_state=current_state,
                phase_label="APPROACH_XY",
            )
            all_history.extend(hist)

            # XYZ 都到位 (Z=safe_approach, XY=sx,sy) → 自适应判稳后才下降 Z
            self._wait_cargo_settled("XY 到取货位置后", self._config.stabilize_delay)

            # ================================================================
            # Phase 1b: Z 从安全高度下降到取货高度 sz
            # ================================================================
            _record_phase(OperationPhase.APPROACH_Z_DESCEND)
            if self.hooks.should_stop():
                raise ControlStoppedError("操作员停止")

            current_state = self._get_current_state()
            hist, _ = self._run_pd(
                target=(sx, sy, sz),
                initial_state=current_state,
                phase_label="APPROACH_Z_DESCEND",
            )
            all_history.extend(hist)

            # ================================================================
            # Phase 1c: 夹取钢卷
            # Z 下降到位后, 先自适应判稳再夹取——避免货物仍在摆动/回弹时就
            # 立即抓钩 (原流程中缺失的安全环节)。
            # ================================================================
            _record_phase(OperationPhase.GRIPPER_CLAMP)
            if self.hooks.should_stop():
                raise ControlStoppedError("操作员停止")

            self._wait_cargo_settled(
                "Z 下降到取货高度后, 夹取前", self._config.gripper_settle_max_wait
            )
            self._plc.gripper_clamp()
            self._wait_gripper_clamped()
            self._safety_sleep(self._config.gripper_safety_delay, "夹取确认后安全等待")

            # ================================================================
            # Phase 2a: 带货上升到 1.0m (初始离地安全高度)
            # ================================================================
            _record_phase(OperationPhase.LIFT_CARGO)
            if self.hooks.should_stop():
                raise ControlStoppedError("操作员停止")

            current_state = self._get_current_state()
            hist, _ = self._run_pd(
                target=(sx, sy, self._config.approach_safe_z),
                initial_state=current_state,
                phase_label="LIFT_CARGO",
            )
            all_history.extend(hist)

            # ================================================================
            # Phase 2b: 三轴联动 → 目标位置 (tx, ty, self._config.transport_safe_z)
            # Z 先到安全高度 1.5m 则等待, XY 到 tx,ty 后继续
            # ================================================================
            _record_phase(OperationPhase.TRANSPORT_XY)
            if self.hooks.should_stop():
                raise ControlStoppedError("操作员停止")

            current_state = self._get_current_state()
            hist, _ = self._run_pd(
                target=(tx, ty, self._config.transport_safe_z),
                initial_state=current_state,
                phase_label="TRANSPORT_XY",
            )
            all_history.extend(hist)

            # XYZ 都到位 (Z=1.5m, XY=tx,ty) → 自适应判稳后才下降 Z
            self._wait_cargo_settled("XY 到目标位置后", self._config.stabilize_delay)

            # ================================================================
            # Phase 2c: Z 从安全高度下降到卸货高度 tz
            # ================================================================
            _record_phase(OperationPhase.TRANSPORT_Z_DESCEND)
            if self.hooks.should_stop():
                raise ControlStoppedError("操作员停止")

            current_state = self._get_current_state()
            hist, _ = self._run_pd(
                target=(tx, ty, tz),
                initial_state=current_state,
                phase_label="TRANSPORT_Z_DESCEND",
            )
            all_history.extend(hist)

            # ================================================================
            # Phase 2d: 释放钢卷
            # Z 下降到位后, 先自适应判稳再释放——这是最关键的风险点: 货物
            # 未放稳/仍在摆动时就释放, 容易造成钢卷偏斜、滑落或磕碰。
            # ================================================================
            _record_phase(OperationPhase.GRIPPER_RELEASE)
            if self.hooks.should_stop():
                raise ControlStoppedError("操作员停止")

            self._wait_cargo_settled(
                "Z 下降到卸货高度后, 释放前", self._config.gripper_settle_max_wait
            )
            self._plc.gripper_release()
            self._wait_gripper_released()
            # 抓钩完全释放确认后, 停留约 2s (post_release_lift_delay) 再抬升,
            # 给货物/抓钩一个短暂的脱离缓冲, 之后直接以正常速度抬升到目标
            # 高度即可, 不需要额外的分段限速。
            self._safety_sleep(
                self._config.post_release_lift_delay, "释放确认后, 抬升前停留"
            )

            # ================================================================
            # Phase 3: Z 归位到 1.6m
            # ================================================================
            _record_phase(OperationPhase.RETURN_Z)
            if self.hooks.should_stop():
                raise ControlStoppedError("操作员停止")

            current_state = self._get_current_state()
            hist, _ = self._run_pd(
                target=(tx, ty, self._config.return_safe_z),
                initial_state=current_state,
                phase_label="RETURN_Z",
            )
            all_history.extend(hist)

            # ================================================================
            # 完成
            # ================================================================
            _record_phase(OperationPhase.DONE)
            self.hooks.set_done()
            if control_state is not None:
                control_state.set_done()

            total_time = time.monotonic() - t_start
            print(f"[Scheduler] 作业完成! 总时间: {total_time:.1f}s")
            for elapsed, phase in phase_history:
                print(f"  {elapsed:6.1f}s  {phase.label}")
            return OperationResult(
                success=True,
                phase=OperationPhase.DONE,
                message=f"作业完成, 总时间 {total_time:.1f}s",
                total_time=total_time,
                history=all_history,
                phase_history=phase_history,
            )

        except ControlStoppedError as exc:
            msg = str(exc)
            print(f"[Scheduler] {msg}")
            self.hooks.set_stopped(msg)
            if control_state is not None:
                control_state.set_stopped(msg)
            self._actuator.emergency_stop()
            return OperationResult(
                success=False,
                phase=OperationPhase.STOPPED,
                message=msg,
                total_time=time.monotonic() - t_start,
                phase_history=phase_history,
            )

        except PositionFeedbackTimeout as exc:
            msg = f"定位超时: {exc}"
            print(f"[Scheduler] {msg}")
            self.hooks.set_error(msg)
            if control_state is not None:
                control_state.set_error(msg)
            self._actuator.emergency_stop()
            return OperationResult(
                success=False,
                phase=OperationPhase.ERROR,
                message=msg,
                total_time=time.monotonic() - t_start,
                phase_history=phase_history,
            )

        except Exception as exc:
            import traceback
            msg = f"{type(exc).__name__}: {exc}"
            print(f"[Scheduler] 异常中止: {msg}")
            traceback.print_exc()
            self.hooks.set_error(msg)
            if control_state is not None:
                control_state.set_error(msg)
            try:
                self._actuator.emergency_stop()
            except Exception:
                pass
            return OperationResult(
                success=False,
                phase=OperationPhase.ERROR,
                message=msg,
                total_time=time.monotonic() - t_start,
                phase_history=phase_history,
            )

    # ------------------------------------------------------------------
    # 内部方法
    # ------------------------------------------------------------------

    def _get_current_state(self) -> CraneState:
        """从位置源获取当前起重机状态 (用于构造 run_pd_control 的初始状态)。

        source.get_position() 内部已有 2s 超时阻塞, 这里的重试仅处理瞬时
        无数据 (返回 None) 的情况。最多重试 2 次 (总计约 4-6s), 同时每步
        检查停止信号。
        """
        for attempt in range(3):
            if self.hooks.should_stop():
                raise ControlStoppedError("操作员在等待定位期间停止")
            pose = self._source.get_position()
            if pose is not None:
                return CraneState(
                    x0=pose['x'],
                    y0=pose['y'],
                    z0=pose['z'],
                )
            if attempt < 2:
                time.sleep(0.5)

        raise RuntimeError("无法获取当前位置反馈 (定位断流)")

    def _run_pd(
        self,
        target: tuple[float, float, float],
        initial_state: CraneState,
        phase_label: str,
    ) -> tuple[list[dict], list[tuple[float, str]]]:
        """运行一次 PD 控制, 封装 run_pd_control()。

        run_pd_control() 内部已调用 actuator.set_z_target() 和
        actuator.set_z_reference(), 这里只做额外的 Z 参考同步以保证
        绝对高度伺服从当前实测位置出发。
        """
        # 同步 Z 参考高度为当前位置 (run_pd_control 内部也会调用,
        # 但此处提前同步可避免第一个周期 PD 对 Z 的误差累积)
        if hasattr(self._actuator, 'set_z_reference'):
            self._actuator.set_z_reference(initial_state.z.position)

        # 创建适配器钩子 — 将 PD 每步数据推给 ControlState (供前端轮询)
        adapter = _PdToSchedulerAdapter(
            self.hooks, self._control_state,
            coordinate_transform=self._coordinate_transform,
            z_is_hoist_height=self._z_is_hoist_height,
        )

        history, arrival_events = run_pd_control(
            source=self._source,
            actuator=self._actuator,
            config=self._config,
            target_pos=target,
            initial_state=initial_state,
            hooks=adapter,
            max_time=self._PD_MAX_TIME,
            verbose=True,
            is_simulation=self._is_simulation,
        )
        return history, arrival_events

    def _safety_sleep(self, duration: float, description: str) -> None:
        """安全等待 — 支持外部中断, 每 100ms 检查一次停止信号。"""
        print(f"[Scheduler] {description} ({duration:.1f}s)...")
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self.hooks.should_stop():
                raise ControlStoppedError("操作员在等待期间停止")
            remaining = deadline - time.monotonic()
            time.sleep(min(0.1, max(0.01, remaining)))

    def _wait_cargo_settled(self, reason: str, max_wait: float) -> None:
        """自适应等待货物平稳 — 用实测位置反馈判断, 代替盲等固定时长。

        设计目标: 效率与安全兼顾。
          - 货物已经静止 → 满足判稳窗口后立即放行, 可能远小于 max_wait (提速)
          - 货物仍在摆动/回弹 → 持续检测直到真正平稳, 不满足就不会放行
            (这正是"Z 到位后立即抓钩/释放, 货物未放稳"这一风险点的根本对策)

        判稳条件 (基于滑动窗口 hook_settle_window 内的整段位置轨迹, 而非
        单帧瞬时读数——单帧瞬时速度并不可靠: 单摆负载在摆动幅值最大处
        瞬时速度恰好为 0, 只判一帧极易在摆动最高点被误判为"已静止"):
          1. 窗口内位置峰峰值 (max-min) < hook_settle_pos_tol —— 直接衡量
             这段时间内摆动/回弹的实际幅度, 天然规避"零速瞬间"假阳性
          2. 窗口首尾的平均漂移速度 < hook_settle_vel_tol —— 排除峰峰值
             恰好很小但仍在持续漂移 (如钢丝绳蠕变、缓慢下沉) 的情况

        窗口需要真正"填满" (覆盖 hook_settle_window 秒) 才参与判定, 避免
        刚开始等待、样本还不够时就被单帧巧合通过。

        超时行为: 与现有抓钩状态确认超时一致 (软失败) —— 打印警告后继续
        执行, 不中止整个作业, 避免传感器/定位反馈异常误伤正常作业。
        """
        cfg = self._config
        window = cfg.hook_settle_window
        vel_tol = cfg.hook_settle_vel_tol
        pos_tol = cfg.hook_settle_pos_tol

        print(f"[Scheduler] {reason} — 等待货物平稳 (自适应判稳, 上限 {max_wait:.1f}s)...")
        t_start = time.monotonic()
        deadline = t_start + max_wait
        samples: list[tuple[float, float, float, float]] = []  # (t, x, y, z)

        while True:
            if self.hooks.should_stop():
                raise ControlStoppedError("操作员在等待货物平稳期间停止")

            pos = self._source.get_position()
            if pos is None:
                raise PositionFeedbackTimeout("等待货物平稳期间定位反馈超时")

            now = time.monotonic()
            x, y, z = pos['x'], pos['y'], pos['z']

            samples.append((now, x, y, z))
            while len(samples) > 1 and now - samples[0][0] > window:
                samples.pop(0)

            # 注意: 用"距等待起点的绝对耗时"判断窗口是否填满, 不能用
            # samples[0] 的年龄——上面的滑动裁剪会把队首年龄压到略小于
            # window (裁剪粒度取决于轮询间隔), 若拿裁剪后的年龄去比较
            # window 本身, 会永远差一点点凑不满, 陷入"永远判不稳"的死锁。
            window_full = (now - t_start) >= window
            if window_full:
                t0, x0, y0, z0 = samples[0]
                span = max(now - t0, 1e-3)
                xs = [s[1] for s in samples]
                ys = [s[2] for s in samples]
                zs = [s[3] for s in samples]

                range_ok = (
                    (max(xs) - min(xs)) < pos_tol
                    and (max(ys) - min(ys)) < pos_tol
                    and (max(zs) - min(zs)) < pos_tol
                )
                drift_ok = (
                    abs(x - x0) / span < vel_tol
                    and abs(y - y0) / span < vel_tol
                    and abs(z - z0) / span < vel_tol
                )
                if range_ok and drift_ok:
                    print(f"[Scheduler] {reason} — 货物已平稳 (用时 {now - t_start:.2f}s)")
                    return

            if now >= deadline:
                print(
                    f"[Scheduler] 警告: {reason} — 判稳超时 ({max_wait:.1f}s), "
                    f"继续执行 (可能仍有轻微摆动)"
                )
                return

            if self._is_simulation:
                time.sleep(HOOK_SETTLE_POLL_INTERVAL)

    def _wait_gripper_clamped(self) -> None:
        """轮询抓钩状态直到确认夹紧完成 (或超时)。

        检查优先级:
          1. gripper_provider (自定义提供者)
          2. PLC get_gripper_clamped()
          3. ROS /crane/cmd_vel/anguler
        """
        print("[Scheduler] 等待抓钩夹紧确认...")
        deadline = time.monotonic() + GRIPPER_CHECK_TIMEOUT
        while time.monotonic() < deadline:
            if self.hooks.should_stop():
                raise ControlStoppedError("操作员在等待夹紧期间停止")

            if self._check_gripper_clamped():
                print("[Scheduler] 抓钩夹紧已确认")
                return
            time.sleep(GRIPPER_CHECK_INTERVAL)

        # 超时 — 不中止, 打印警告 (可能是传感器问题)
        print("[Scheduler] 警告: 抓钩夹紧确认超时, 继续执行 (假定已夹紧)")

    def _wait_gripper_released(self) -> None:
        """轮询抓钩状态直到确认释放完成 (或超时)。"""
        print("[Scheduler] 等待抓钩释放确认...")
        deadline = time.monotonic() + GRIPPER_CHECK_TIMEOUT
        while time.monotonic() < deadline:
            if self.hooks.should_stop():
                raise ControlStoppedError("操作员在等待释放期间停止")

            if self._check_gripper_released():
                print("[Scheduler] 抓钩释放已确认")
                return
            time.sleep(GRIPPER_CHECK_INTERVAL)

        print("[Scheduler] 警告: 抓钩释放确认超时, 继续执行 (假定已释放)")

    def _check_gripper_clamped(self) -> bool:
        """多源检查抓钩是否已夹紧。"""
        # 1. 自定义提供者
        if self._gripper_provider is not None:
            clamped, _ = self._gripper_provider()
            if clamped is True:
                return True

        # 2. PLC 直接读取
        try:
            clamped = self._plc.get_gripper_clamped()
            if clamped is True:
                return True
        except Exception:
            pass

        # 3. ROS 话题 (fallback)
        try:
            from ros_bridge import is_gripper_clamped as ros_is_clamped
            result = ros_is_clamped()
            if result is True:
                return True
        except Exception:
            pass

        return False

    def _check_gripper_released(self) -> bool:
        """多源检查抓钩是否已释放。"""
        # 1. 自定义提供者
        if self._gripper_provider is not None:
            _, released = self._gripper_provider()
            if released is True:
                return True

        # 2. PLC 直接读取
        try:
            released = self._plc.get_gripper_released()
            if released is True:
                return True
        except Exception:
            pass

        # 3. ROS 话题 (fallback)
        try:
            from ros_bridge import is_gripper_clamped as ros_is_clamped
            result = ros_is_clamped()
            if result is False:  # not clamped = released
                return True
        except Exception:
            pass

        return False

    def _update_control_state_phase(self, control_state, phase: OperationPhase) -> None:
        """更新 ControlState 的阶段信息 (供前端轮询)。"""
        try:
            with control_state.lock:
                control_state.scheduler_phase = phase
                control_state.scheduler_phase_label = phase.label
        except Exception:
            pass


# ---------------------------------------------------------------------------
# PD 控制钩子适配器 — 将 run_pd_control 的钩子桥接到 SchedulerHooks
# ---------------------------------------------------------------------------

class _PdToSchedulerAdapter(ControlHooks):
    """将 PD 控制循环的每步状态转发到 SchedulerHooks 和 ControlState。

    不阻塞 PD 循环 (on_step 内不做 IO), 仅更新共享状态。
    ControlState 用于前端 /api/control-state 轮询, 显示实时位置和速度。

    自动将 crane 坐标转换为 map 坐标 (前端展示用), 保持与旧
    LiveControlHooks.on_step() 一致的参考系。
    """

    def __init__(self, scheduler_hooks: SchedulerHooks,
                 control_state: object | None = None,
                 coordinate_transform: object | None = None,
                 z_is_hoist_height: bool = False):
        self._hooks = scheduler_hooks
        self._control_state = control_state
        self._coordinate_transform = coordinate_transform
        self._z_is_hoist_height = z_is_hoist_height

    def on_step(self, step_data: dict) -> None:
        if self._control_state is not None:
            try:
                # 将 crane 坐标转换为 map 坐标 (与 LiveControlHooks.on_step 一致)
                if self._coordinate_transform is not None:
                    display_step = self._coordinate_transform.control_step_to_map(
                        step_data, z_is_hoist_height=self._z_is_hoist_height,
                    )
                else:
                    display_step = dict(step_data)
                with self._control_state.lock:
                    self._control_state.latest = display_step
                    self._control_state.step_count += 1
            except Exception:
                pass

    def on_arrival(self, axis: str, t: float) -> None:
        if self._control_state is not None:
            try:
                with self._control_state.lock:
                    self._control_state.arrivals.append({'axis': axis, 't': t})
            except Exception:
                pass

    def should_stop(self) -> bool:
        return self._hooks.should_stop()


# ---------------------------------------------------------------------------
# 仿真模式便捷函数
# ---------------------------------------------------------------------------

def run_simulation_operation(
    start_pos: tuple[float, float, float],
    target_pos: tuple[float, float, float],
    initial_state: CraneState,
    config: CraneConfig,
    verbose: bool = True,
) -> OperationResult:
    """仿真模式: 使用 plant model 运行完整作业流程。

    用于测试和验证 — 不连接真实 PLC。
    """
    from crane_model import CranePlant, PlantActuator, SimPositionSource

    plant = CranePlant(config)
    source = SimPositionSource(plant, initial_state, config)
    actuator = PlantActuator(plant, initial_state, config)

    # 仿真模式下使用 MockPLC (gripper 动作由 Mock 模拟)
    from plc_interface import MockPLC
    mock_plc = MockPLC(verbose=verbose)

    # 调整 set_z_target / set_z_reference 为仿真兼容
    def _noop_z_target(_h: float) -> None:
        pass

    def _noop_z_ref(_h: float) -> None:
        pass

    actuator.set_z_target = _noop_z_target       # type: ignore[assignment]
    actuator.set_z_reference = _noop_z_ref        # type: ignore[assignment]

    scheduler = OperationScheduler(
        plc=mock_plc,
        source=source,
        actuator=actuator,
        config=config,
        is_simulation=True,
    )

    return scheduler.execute(start_pos=start_pos, target_pos=target_pos)
