"""PLC interface for Siemens S7 bridge crane control.

Provides two implementations sharing the same interface:
  - MockPLC  — prints commands (x86 dev, no ARM library)
  - RealPLC  — ctypes wrapper for libsscarctrl.so (ARM target)

PLC API reference (from ss_car_control.h / demo.cpp):
  BigCarCtrl(velocity, ip, control_flag)   — X bridge, velocity m/s
  SmallcarCtrl(velocity, ip, control_flag) — Y trolley, velocity m/s
  liftctrl(height, ip)                     — Z hoist, absolute height m
  SendPlcHeartbeat(ip)                     — heartbeat pulse
  GetActualLiftHeight()                    — read hoist height
  CheckPlcConnection()                     — connection status
  EmergencyBrake(clamp, ip)                — emergency stop
  ResetControl(clamp, ip)                  — reset fault
"""

from __future__ import annotations

import ctypes
import math
import threading
import time

from crane_model import CraneState


# ---------------------------------------------------------------------------
# GetActualLiftHeight() 异常读数过滤
# ---------------------------------------------------------------------------

class _LiftHeightSanitizer:
    """过滤 GetActualLiftHeight() 的异常读数, 避免抓钩高度反馈"跳到 0/垂圾值
    再跳回正确高度"的抽动, 污染 D 项差分速度或误触发到位判定。

    现场日志显示: 每当底层闭源库打印
      "CheckPlcConnection: Heartbeat timeout/difference too large"
    (S7 连接瞬时抖动) 时, 紧跟着的 GetActualLiftHeight() 读数经常是恰好 0,
    或形如 4.49863e-312 的 denormalized 垂圾值 (典型的连接抖动期间兜底值/
    竞态读取残留)。这是闭源库内部行为, 无法在 Python 侧修复库本身, 只能
    在读数进入控制/展示之前过滤掉——识别为异常时沿用上一次可信读数;
    若异常持续超过容忍窗口才放弃缓存 (返回 None, 交给上层回退 SLAM Z
    或判定为真实故障), 避免无限期相信一个已经过期的缓存值。
    """

    _ZERO_SENTINEL_EPS = 0.05    # [m] 判定"近零"为异常兜底值的门限 (低于安全下限 0.5m 很多)
    _MAX_RATE = 1.0              # [m/s] 认为物理上合理的最大高度变化速率 (含较大余量)
    _JUMP_MARGIN = 0.10          # [m] 跳变判定的固定余量, 补偿调用间隔抖动
    STALE_TIMEOUT = 2.0          # [s] 缓存值信任窗口, 与定位反馈超时保持一致量级

    def __init__(self) -> None:
        self._last_good: float | None = None
        self._last_good_time: float = 0.0
        self.bad_streak = 0

    def reset(self) -> None:
        """PLC (重新) 连接后调用: 旧缓存值可能已过期, 清空重新累积。"""
        self._last_good = None
        self._last_good_time = 0.0
        self.bad_streak = 0

    def sanitize(self, raw: float, now: float | None = None) -> tuple[float | None, str | None]:
        """返回 (采纳的高度或 None, 拒绝原因或 None)。"""
        now = time.time() if now is None else now
        reason = self._reject_reason(raw, now)
        if reason is None:
            self._last_good = raw
            self._last_good_time = now
            self.bad_streak = 0
            return raw, None

        self.bad_streak += 1
        if self._last_good is not None and (now - self._last_good_time) < self.STALE_TIMEOUT:
            return self._last_good, reason
        return None, reason

    def _reject_reason(self, raw: float, now: float) -> str | None:
        if not math.isfinite(raw):
            return 'non-finite'
        if (
            self._last_good is not None
            and abs(raw) < self._ZERO_SENTINEL_EPS
            and abs(self._last_good) >= self._ZERO_SENTINEL_EPS
        ):
            # 库在连接抖动期间常返回 0 (或极小的 denormalized 垂圾值) 作兜底,
            # 而不是保持上一次真实读数。
            return 'near-zero sentinel'
        if self._last_good is not None:
            dt = max(now - self._last_good_time, 0.01)
            max_jump = self._MAX_RATE * dt + self._JUMP_MARGIN
            if abs(raw - self._last_good) > max_jump:
                return 'implausible jump'
        return None


# ---------------------------------------------------------------------------
# Common interface
# ---------------------------------------------------------------------------

class PLCInterface:
    """Abstract interface — MockPLC and RealPLC both implement this."""

    def connect(self, ip: str) -> int:
        raise NotImplementedError

    def disconnect(self) -> None:
        raise NotImplementedError

    def big_car_ctrl(self, velocity: float) -> None:
        raise NotImplementedError

    def small_car_ctrl(self, velocity: float) -> None:
        raise NotImplementedError

    def lift_ctrl(self, height: float) -> None:
        raise NotImplementedError

    def send_heartbeat(self) -> bool:
        raise NotImplementedError

    def get_lift_height(self) -> float | None:
        raise NotImplementedError

    def check_connection(self) -> bool:
        raise NotImplementedError

    def emergency_brake(self, clamp: bool = True) -> None:
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError

    @property
    def heartbeat_healthy(self) -> bool:
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Mock PLC — for local development (x86, no ARM .so)
# ---------------------------------------------------------------------------

class MockPLC(PLCInterface):
    """Print-based PLC stub for local development.

    Each control call prints the command that would be sent to the real PLC.
    Heartbeat runs at 10 Hz but only prints on status changes.
    """

    def __init__(self, verbose: bool = False) -> None:
        self._verbose = verbose
        self._ip = '127.0.0.1'
        self._connected = False
        self._heartbeat_thread: threading.Thread | None = None
        self._heartbeat_running = threading.Event()
        self._heartbeat_healthy = False
        self._fail_count = 0
        # 心跳失败容忍窗口: 连续失败达到该次数(10Hz→约1s)才判定心跳丢失。
        # 工业现场网络(如 WiFi/交换机)偶发 100-300ms 抖动很常见, 阈值太小
        # (旧值 3 次≈300ms) 会把正常抖动误判为心跳丢失, 导致 PD 控制中途
        # 被安全停车提前结束。PLC 侧看门狗超时通常远大于此, 1s 仍在安全范围内。
        self._max_fails = 10
        # Throttle motion prints: only print when value changes > threshold
        # or once per interval, to avoid 100 Hz spam.
        self._last_printed: dict[str, tuple[float, float]] = {}
        self._print_interval = 0.5        # force print at least every 0.5 s
        self._print_threshold_vel = 0.01   # velocity threshold: 0.01 m/s
        self._print_threshold_h = 0.05     # height threshold: 0.05 m
        self.last_vx: float = 0.0          # last-sent X velocity (for UI display)
        self.last_vy: float = 0.0          # last-sent Y velocity
        self.last_hz: float = 0.0          # last-sent Z height
        self.last_vz: float = 0.0          # last-sent Z velocity

    # -- connection ---------------------------------------------------------

    def connect(self, ip: str) -> int:
        self._ip = ip
        self._connected = True
        print(f'[PLC] connect_to_plc({ip}) — OK')
        return 0

    def disconnect(self) -> None:
        self._stop_heartbeat()
        self._connected = False
        print('[PLC] disconnect_plc()')

    def check_connection(self) -> bool:
        return self._connected

    # -- motion control -----------------------------------------------------

    def _should_print(self, key: str, value: float) -> bool:
        """Return True if we should print this value (throttled to reduce spam)."""
        now = time.time()
        prev_val, prev_time = self._last_printed.get(key, (None, 0.0))
        if prev_val is None:
            self._last_printed[key] = (value, now)
            return True
        threshold = self._print_threshold_h if key == 'z' else self._print_threshold_vel
        if abs(value - prev_val) > threshold:
            self._last_printed[key] = (value, now)
            return True
        if now - prev_time > self._print_interval:
            self._last_printed[key] = (value, now)
            return True
        return False

    def big_car_ctrl(self, velocity: float) -> None:
        if self._verbose and self._should_print('x', velocity):
            print(f'[PLC] BigCarCtrl(v={velocity:+.3f} m/s, flag=0x047F)')

    def small_car_ctrl(self, velocity: float) -> None:
        if self._verbose and self._should_print('y', velocity):
            print(f'[PLC] SmallcarCtrl(v={velocity:+.3f} m/s, flag=0x047F)')

    def lift_ctrl(self, height: float) -> None:
        if self._verbose and self._should_print('z', height):
            print(f'[PLC] liftctrl(h={height:.3f} m)')

    # -- heartbeat ----------------------------------------------------------

    def send_heartbeat(self) -> bool:
        if self._verbose:
            self._fail_count = 0
        self._heartbeat_healthy = True
        return True

    def _heartbeat_loop(self) -> None:
        """Background heartbeat at 10 Hz (100 ms interval).

        自愈: 心跳失败达到阈值只是把 heartbeat_healthy 置为 False (供控制
        循环安全停车), 循环本身绝不 break——必须持续重试发送, 一旦网络/PLC
        恢复就立刻把 heartbeat_healthy 重新置 True。旧实现失败达阈值后会
        break 退出线程, 心跳永久停止且再也不会恢复, 只能重启整个进程才能
        继续控制——这是"PD 运行中偶尔提前结束且之后怎么点都无法恢复"的
        根因之一。
        """
        time.sleep(0.5)  # warm-up
        last_print = 0.0
        while self._heartbeat_running.is_set():
            ok = self.send_heartbeat()
            if not ok:
                self._fail_count += 1
                if self._verbose:
                    print(f'[PLC] heartbeat FAIL #{self._fail_count}')
                if self._fail_count >= self._max_fails:
                    self._heartbeat_healthy = False
                    if self._verbose:
                        print('[PLC] Heartbeat lost — will keep retrying to self-heal')
            else:
                self._fail_count = 0
                self._heartbeat_healthy = True
            # Throttle heartbeat prints to ~1 Hz
            now = time.time()
            if self._verbose and now - last_print > 1.0:
                print('[PLC] heartbeat OK')
                last_print = now
            time.sleep(0.1)

    def start_heartbeat(self) -> None:
        self._heartbeat_running.set()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, name='plc-heartbeat', daemon=True,
        )
        self._heartbeat_thread.start()
        print('[PLC] Heartbeat started (10 Hz)')

    def _stop_heartbeat(self) -> None:
        self._heartbeat_running.clear()
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=1.0)

    @property
    def heartbeat_healthy(self) -> bool:
        return self._heartbeat_healthy

    # -- readback -----------------------------------------------------------

    def get_lift_height(self) -> float | None:
        # Mock returns None — real encoder readback not available
        return None

    # -- safety -------------------------------------------------------------

    def emergency_brake(self, clamp: bool = True) -> None:
        action = 'ENGAGE' if clamp else 'RELEASE'
        print(f'[PLC] EmergencyBrake({action})')

    def reset(self) -> None:
        print('[PLC] ResetControl()')


# ---------------------------------------------------------------------------
# Real PLC — ctypes wrapper for libsscarctrl.so (ARM aarch64 target)
# ---------------------------------------------------------------------------

class RealPLC(PLCInterface):
    """ctypes wrapper around libsscarctrl.so.

    Loads the shared library and declares argument/return types for each
    exported C function.  Same interface as MockPLC so the control loop
    can use either transparently.
    """

    def __init__(self, lib_path: str = 'plc_lib/lib/libsscarctrl.so') -> None:
        self._lib = ctypes.CDLL(lib_path)
        self._ip = ctypes.c_char_p(None)
        self._ip_bytes: bytes = b'192.168.0.1'
        self._connected = False
        self._heartbeat_thread: threading.Thread | None = None
        self._heartbeat_running = threading.Event()
        self._heartbeat_healthy = False
        self._fail_count = 0
        # 见 MockPLC._max_fails 注释: 10 次(约1s)容忍网络抖动, 避免误判丢失。
        self._max_fails = 10
        self.last_vx: float = 0.0          # last-sent X velocity (for UI display)
        self.last_vy: float = 0.0          # last-sent Y velocity
        self.last_hz: float = 0.0          # last-sent Z height
        self.last_vz: float = 0.0          # last-sent Z velocity
        # GetActualLiftHeight() 异常读数过滤 (见 _LiftHeightSanitizer 说明)。
        self._lift_height_lock = threading.Lock()
        self._lift_height_sanitizer = _LiftHeightSanitizer()
        self._last_lift_height_warn = 0.0
        self._lib.connect_to_plc.argtypes = [ctypes.c_char_p]
        self._lib.connect_to_plc.restype = ctypes.c_int

        # disconnect_plc
        self._lib.disconnect_plc.argtypes = []
        self._lib.disconnect_plc.restype = None

        # BigCarCtrl / SmallcarCtrl
        self._lib.BigCarCtrl.argtypes = [ctypes.c_double, ctypes.c_char_p, ctypes.c_uint16]
        self._lib.BigCarCtrl.restype = None
        self._lib.SmallcarCtrl.argtypes = [ctypes.c_double, ctypes.c_char_p, ctypes.c_uint16]
        self._lib.SmallcarCtrl.restype = None

        # liftctrl
        self._lib.liftctrl.argtypes = [ctypes.c_double, ctypes.c_char_p]
        self._lib.liftctrl.restype = None

        # heartbeat
        self._lib.SendPlcHeartbeat.argtypes = [ctypes.c_char_p]
        self._lib.SendPlcHeartbeat.restype = ctypes.c_bool

        # readback
        self._lib.GetActualLiftHeight.argtypes = []
        self._lib.GetActualLiftHeight.restype = ctypes.c_double

        # status
        self._lib.CheckPlcConnection.argtypes = []
        self._lib.CheckPlcConnection.restype = ctypes.c_bool

        # safety
        self._lib.EmergencyBrake.argtypes = [ctypes.c_bool, ctypes.c_char_p]
        self._lib.EmergencyBrake.restype = None
        self._lib.ResetControl.argtypes = [ctypes.c_bool, ctypes.c_char_p]
        self._lib.ResetControl.restype = None

        print(f'[PLC] Loaded {lib_path}')

    def _ip_ptr(self) -> ctypes.c_char_p:
        return ctypes.c_char_p(self._ip_bytes)

    # -- connection ---------------------------------------------------------

    def connect(self, ip: str) -> int:
        self._ip_bytes = ip.encode('utf-8')
        ret = self._lib.connect_to_plc(self._ip_ptr())
        self._connected = (ret == 0)
        if ret != 0:
            print(f'[PLC] connect_to_plc({ip}) FAILED, ret={ret}')
        else:
            print(f'[PLC] connect_to_plc({ip}) OK')
        # (重新) 连接后旧的抓钩高度缓存值可能已过期, 清空重新累积。
        with self._lift_height_lock:
            self._lift_height_sanitizer.reset()
        return ret

    def disconnect(self) -> None:
        self._stop_heartbeat()
        self._lib.disconnect_plc()
        self._connected = False
        print('[PLC] disconnect_plc()')

    def check_connection(self) -> bool:
        return bool(self._lib.CheckPlcConnection())

    # -- motion control -----------------------------------------------------

    def big_car_ctrl(self, velocity: float) -> None:
        self._lib.BigCarCtrl(ctypes.c_double(velocity), self._ip_ptr(), 0x047F)

    def small_car_ctrl(self, velocity: float) -> None:
        self._lib.SmallcarCtrl(ctypes.c_double(velocity), self._ip_ptr(), 0x047F)

    def lift_ctrl(self, height: float) -> None:
        self._lib.liftctrl(ctypes.c_double(height), self._ip_ptr())

    # -- heartbeat ----------------------------------------------------------

    def send_heartbeat(self) -> bool:
        ok = bool(self._lib.SendPlcHeartbeat(self._ip_ptr()))
        if ok:
            self._fail_count = 0
            self._heartbeat_healthy = True
        return ok

    def _heartbeat_loop(self) -> None:
        """自愈心跳循环: 见 MockPLC._heartbeat_loop 注释。绝不 break——失败达阈值
        只置 heartbeat_healthy=False 供控制循环安全停车, 循环持续重试发送心跳,
        网络/PLC 恢复后自动把 heartbeat_healthy 重新置 True, 无需重启进程。"""
        time.sleep(0.5)
        while self._heartbeat_running.is_set():
            ok = self.send_heartbeat()
            if not ok:
                self._fail_count += 1
                print(f'[PLC] heartbeat FAIL #{self._fail_count}')
                if self._fail_count >= self._max_fails:
                    self._heartbeat_healthy = False
                    print('[PLC] Heartbeat lost — retrying in background to self-heal')
            else:
                self._fail_count = 0
                self._heartbeat_healthy = True
            time.sleep(0.1)

    def start_heartbeat(self) -> None:
        self._heartbeat_running.set()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, name='plc-heartbeat', daemon=True,
        )
        self._heartbeat_thread.start()
        print('[PLC] Heartbeat thread started (10 Hz)')

    def _stop_heartbeat(self) -> None:
        self._heartbeat_running.clear()
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=1.0)

    @property
    def heartbeat_healthy(self) -> bool:
        return self._heartbeat_healthy

    # -- readback -----------------------------------------------------------

    def get_lift_height(self) -> float | None:
        """读取抓钩实测高度, 过滤连接抖动期间的 0/垂圾值/不可能跳变 (见
        _LiftHeightSanitizer)。持续异常超过其容忍窗口时返回 None, 交给
        上层判定 (通常回退 SLAM Z 或视为定位不可用)。"""
        raw = self._lib.GetActualLiftHeight()
        with self._lift_height_lock:
            accepted, reason = self._lift_height_sanitizer.sanitize(raw)
        if reason is not None:
            # 异常读数通常连续出现 (整段抖动期间), 节流打印避免刷屏。
            now = time.time()
            if now - self._last_lift_height_warn > 1.0:
                if accepted is not None:
                    print(
                        f'[PLC] GetActualLiftHeight 异常读数已丢弃 '
                        f'(raw={raw!r}, {reason}), 沿用上次读数 {accepted:.3f}m'
                    )
                else:
                    print(
                        f'[PLC] GetActualLiftHeight 持续异常超过 '
                        f'{_LiftHeightSanitizer.STALE_TIMEOUT:.0f}s (raw={raw!r}, {reason}), '
                        f'放弃缓存值'
                    )
                self._last_lift_height_warn = now
        return accepted

    # -- safety -------------------------------------------------------------

    def emergency_brake(self, clamp: bool = True) -> None:
        self._lib.EmergencyBrake(ctypes.c_bool(clamp), self._ip_ptr())
        print(f'[PLC] EmergencyBrake({"ENGAGE" if clamp else "RELEASE"})')

    def reset(self) -> None:
        self._lib.ResetControl(ctypes.c_bool(True), self._ip_ptr())
        print('[PLC] ResetControl()')


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def create_plc(
    lib_path: str = 'plc_lib/lib/libsscarctrl.so',
    verbose: bool = False,
    allow_mock: bool = False,
) -> PLCInterface:
    """Create a PLC interface, requiring explicit opt-in before using a mock."""
    try:
        plc = RealPLC(lib_path)
        print('[PLC] Using RealPLC (ctypes → libsscarctrl.so)')
        return plc
    except OSError as exc:
        if allow_mock:
            print(f'[PLC] Cannot load {lib_path}: {exc}')
            print('[PLC] Explicit mock mode enabled — using MockPLC')
            return MockPLC(verbose=verbose)
        raise RuntimeError(f'Cannot load PLC library {lib_path}: {exc}') from exc


# ============================================================================
# PlcActuator — 适配统一 PD 控制循环的 Actuator 接口
# ============================================================================

class PlcActuator:
    """PLC 执行器 — 封装真实 PLC 指令发送。

    实现 Actuator 接口，用于 PLC 模式的 run_pd_control()。

    职责:
      - 每个控制周期都把速度指令发送到 PLC (速度伺服需持续刷新, 否则
        PLC 看门狗会停轴, 表现为运动断断续续)
      - Z 轴: 将速度指令积分为绝对高度, 再调用 liftctrl()
      - 逐轴方向修正: 当驱动器正方向与定位坐标轴相反时翻转指令符号
      - 始终更新 plc.last_vx/vy/vz/hz 供前端轮询
      - 紧急停止

    方向符号 (big_car_sign/small_car_sign/lift_sign):
      +1 表示驱动正方向与定位轴一致; -1 表示相反, 需翻转速度指令。
      定位轴与轨道存在旋转关系时用坐标标定 (yaw) 处理; 但"轴方向整体
      取反"是镜像, 旋转无法表达, 必须用这里的逐轴符号翻转。

    安全高度下限 (min_lift_height):
      下发给 liftctrl 的绝对高度会被钳到 >= min_lift_height (默认 0.5m),
      保证抓钩离地不小于该高度; 无论 PD 输出或目标怎么设, 都不会命令
      抓钩降到该安全高度以下。
    """

    _MIN_LIFT_HEIGHT_DEFAULT = 0.5  # [m] 抓钩离地最小安全高度

    def __init__(
        self,
        plc: PLCInterface,
        initial_z: float = 0.0,
        *,
        big_car_sign: float = 1.0,
        small_car_sign: float = 1.0,
        lift_sign: float = 1.0,
        min_lift_height: float = _MIN_LIFT_HEIGHT_DEFAULT,
    ):
        self._plc = plc
        self._command_lock = threading.RLock()
        self._min_lift_height = float(min_lift_height)   # [m] 抓钩离地安全下限
        self._z_height = initial_z          # Z 轴绝对高度设定值 (下发给 liftctrl)
        # Z 目标高度 (由 PD 控制循环通过 set_z_target 注入)。liftctrl 是绝对位置
        # 伺服, 故 Z 设定值需一路逼近目标, 而非每周期只领先实测一步。
        self._z_target: float | None = None
        self._z_target_descending = False   # 相对设目标时刻的运动方向 (防越过)
        self._last_vx: float | None = None
        self._last_vy: float | None = None
        self._last_z_height: float | None = None
        self._last_log: float = 0.0          # 上次 PD 日志时间
        # 归一化为 ±1, 避免误传入的幅值改变速度大小。
        self._big_car_sign = -1.0 if big_car_sign < 0 else 1.0
        self._small_car_sign = -1.0 if small_car_sign < 0 else 1.0
        self._lift_sign = -1.0 if lift_sign < 0 else 1.0

    _log_interval = 1.0  # [s] PD 输出日志最小间隔

    def _clamp_lift_height(self, height: float) -> float:
        """把下发高度钳到安全下限, 保证抓钩离地 >= min_lift_height。"""
        return height if height >= self._min_lift_height else self._min_lift_height

    def apply(self, vx: float, vy: float, vz: float, dt: float) -> None:
        """发送速度指令到 PLC。

        X/Y 轴: 每周期下发速度指令 (含方向修正)
        Z 轴:   vz * dt 积分 → 绝对高度 → liftctrl()
        """
        with self._command_lock:
            self._ensure_available()

            # 方向修正: 驱动正方向与定位轴相反时翻转指令符号。
            vx_out = self._big_car_sign * vx
            vy_out = self._small_car_sign * vy
            vz_out = self._lift_sign * vz

            # 定期打印 PD 输出 (每秒最多一次, 避免刷屏)
            now = time.time()
            if now - self._last_log > self._log_interval:
                print(f'[PD] v_cmd=(x={vx_out:+.4f}, y={vy_out:+.4f}, z={vz_out:+.4f}) '
                      f'm/s  z_h={self._z_height:.3f}m')
                self._last_log = now

            # 速度伺服模式: 每个控制周期都必须刷新指令 (10Hz)，否则 PLC 看门狗
            # 会在若干周期后停轴, 下一次指令又让它动一下 → 运动断断续续。
            self._plc.big_car_ctrl(vx_out)
            self._plc.last_vx = vx_out
            self._last_vx = vx_out

            self._plc.small_car_ctrl(vy_out)
            self._plc.last_vy = vy_out
            self._last_vy = vy_out

            # --- Z 轴: 绝对高度位置伺服 (liftctrl 接收绝对目标高度) ---
            # 用 PD 速度指令积分出高度设定值, 并单调逼近目标 (不越过)。这样下发
            # 给 liftctrl 的设定值会一路走到目标, PLC 内部位置环随即跟随; 而不是
            # 每周期把设定值重锚到实测、只领先一步 vz*dt——后者遇到伺服/驱动死区
            # 几乎不动, 表现为"Z 不受 PD 控制"。速度仍由 PD 决定 (含限速/阻尼)。
            self._z_height += vz_out * dt
            if self._z_target is not None:
                # 防越过目标: 按"设目标时刻"的方向把设定值钳在目标一侧,
                # 不受单帧速度噪声换向影响。
                if self._z_target_descending:
                    self._z_height = max(self._z_height, self._z_target)
                else:
                    self._z_height = min(self._z_height, self._z_target)
            # 下发前钳到安全下限, 保证抓钩离地不小于 min_lift_height。
            self._z_height = self._clamp_lift_height(self._z_height)
            self._plc.last_vz = vz_out
            self._plc.last_hz = self._z_height
            self._plc.lift_ctrl(self._z_height)
            self._last_z_height = self._z_height

    def _ensure_available(self) -> None:
        if not self._plc.check_connection():
            raise RuntimeError('PLC connection is not available')
        if not self._plc.heartbeat_healthy:
            raise RuntimeError('PLC heartbeat is not healthy')

    def set_z_reference(self, height: float) -> None:
        """Synchronize the Z setpoint with a fresh hoist-height measurement."""
        with self._command_lock:
            self._z_height = float(height)
            self._last_z_height = None

    def set_z_target(self, target_height: float) -> None:
        """注入 Z 目标高度 (抓钩高度参考系), 供绝对高度设定值单调逼近。

        目标同样钳到安全下限; 运动方向按当前设定值与目标的高低关系确定,
        避免下降/上升过程中因单帧速度换向而把设定值提前吸附到目标。
        """
        with self._command_lock:
            target = self._clamp_lift_height(float(target_height))
            self._z_target = target
            self._z_target_descending = target < self._z_height

    def update_state(self, state: CraneState, position: dict) -> None:
        """从 /localization_pose 数据更新 CraneState。

        位置直接来自定位；可用的原生速度逐轴写入，缺失轴由控制循环
        使用位置差分后的滤波速度补齐。
        """
        with self._command_lock:
            state.x.position = position['x']
            state.y.position = position['y']
            state.z.position = position['z']
            # 注意: 不要把 Z 设定值 (self._z_height) 重锚到实测高度。Z 是绝对位置
            # 伺服, 设定值需领先实测一路走到目标; 若每周期重锚, 设定值只会领先一步
            # vz*dt, 遇到伺服死区几乎不动。实测高度用于 PD 反馈 (state.z.position)。

            # 每个轴独立选择原生速度；缺失轴由控制循环填入差分估计值。
            for axis_name in ('x', 'y', 'z'):
                velocity = position.get(f'v{axis_name}')
                if velocity is not None:
                    getattr(state, axis_name).velocity = velocity

    def stop_motion(self) -> None:
        """正常到位：X/Y 归零，Z 保持当前绝对高度 (不低于安全下限)。"""
        with self._command_lock:
            hold_height = self._clamp_lift_height(self._z_height)
            self._z_height = hold_height
            self._plc.big_car_ctrl(0.0)
            self._plc.small_car_ctrl(0.0)
            self._plc.lift_ctrl(hold_height)
            self._plc.last_vx = 0.0
            self._plc.last_vy = 0.0
            self._plc.last_vz = 0.0
            self._plc.last_hz = hold_height
            self._last_vx = 0.0
            self._last_vy = 0.0
            self._last_z_height = hold_height

    def emergency_stop(self) -> None:
        """安全停止 — 匹配 demo.cpp STOP ALL 模式。"""
        with self._command_lock:
            self._plc.big_car_ctrl(0.0)
            self._plc.small_car_ctrl(0.0)
            self._plc.lift_ctrl(0.0)
            self._plc.last_vx = 0.0
            self._plc.last_vy = 0.0
            self._plc.last_vz = 0.0
            self._plc.last_hz = 0.0
            self._last_vx = self._last_vy = self._last_z_height = None

    def cleanup(self) -> None:
        """控制结束后的清理 (PLC 连接保持, 仅重置内部状态)。"""
        pass
