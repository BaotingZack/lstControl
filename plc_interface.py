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
import threading
import time

from crane_model import CraneState


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
        self._max_fails = 3
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
        """Background heartbeat at 10 Hz (100 ms interval)."""
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
                        print('[PLC] Heartbeat lost — PLC disconnected')
                    break
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
        self._max_fails = 3
        self.last_vx: float = 0.0          # last-sent X velocity (for UI display)
        self.last_vy: float = 0.0          # last-sent Y velocity
        self.last_hz: float = 0.0          # last-sent Z height
        self.last_vz: float = 0.0          # last-sent Z velocity
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
        time.sleep(0.5)
        while self._heartbeat_running.is_set():
            ok = self.send_heartbeat()
            if not ok:
                self._fail_count += 1
                print(f'[PLC] heartbeat FAIL #{self._fail_count}')
                if self._fail_count >= self._max_fails:
                    self._heartbeat_healthy = False
                    print('[PLC] Heartbeat lost — PLC disconnected!')
                    break
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
        return self._lib.GetActualLiftHeight()

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
      - 速度指令通过 deadband 过滤后发送到 PLC
      - Z 轴: 将速度指令积分为绝对高度, 再调用 liftctrl()
      - 始终更新 plc.last_vx/vy/vz/hz 供前端轮询
      - 紧急停止
    """

    _DEADBAND = 0.005  # [m/s or m] 最小变化阈值

    def __init__(self, plc: PLCInterface, initial_z: float = 0.0):
        self._plc = plc
        self._command_lock = threading.RLock()
        self._z_height = initial_z          # Z 轴绝对高度积分值
        self._last_vx: float | None = None
        self._last_vy: float | None = None
        self._last_z_height: float | None = None
        self._last_log: float = 0.0          # 上次 PD 日志时间

    _log_interval = 1.0  # [s] PD 输出日志最小间隔

    def apply(self, vx: float, vy: float, vz: float, dt: float) -> None:
        """发送速度指令到 PLC。

        X/Y 轴: 直接发送速度指令 (含 deadband 过滤)
        Z 轴:   vz * dt 积分 → 绝对高度 → liftctrl()
        """
        with self._command_lock:
            self._ensure_available()

            # 定期打印 PD 输出 (每秒最多一次, 避免刷屏)
            now = time.time()
            if now - self._last_log > self._log_interval:
                print(f'[PD] v_cmd=(x={vx:+.4f}, y={vy:+.4f}, z={vz:+.4f}) m/s  z_h={self._z_height:.3f}m')
                self._last_log = now

            # X/Y 的零速指令不能被 deadband 吞掉。
            self._plc.last_vx = vx
            if self._should_send_velocity(vx, self._last_vx):
                self._plc.big_car_ctrl(vx)
                self._last_vx = vx

            self._plc.last_vy = vy
            if self._should_send_velocity(vy, self._last_vy):
                self._plc.small_car_ctrl(vy)
                self._last_vy = vy

            # Z 轴 API 接收绝对高度。节流必须比较高度设定值，而不是 vz。
            self._z_height = max(0.0, self._z_height + vz * dt)
            self._plc.last_vz = vz
            self._plc.last_hz = self._z_height
            if (
                self._last_z_height is None
                or abs(self._z_height - self._last_z_height) >= self._DEADBAND
            ):
                self._plc.lift_ctrl(self._z_height)
                self._last_z_height = self._z_height

    def _ensure_available(self) -> None:
        if not self._plc.check_connection():
            raise RuntimeError('PLC connection is not available')
        if not self._plc.heartbeat_healthy:
            raise RuntimeError('PLC heartbeat is not healthy')

    @staticmethod
    def _should_send_velocity(value: float, previous: float | None) -> bool:
        if previous is None:
            return True
        if value == 0.0 and previous != 0.0:
            return True
        return abs(value - previous) >= PlcActuator._DEADBAND

    def set_z_reference(self, height: float) -> None:
        """Synchronize the Z integrator with a fresh localization measurement."""
        with self._command_lock:
            self._z_height = max(0.0, float(height))
            self._last_z_height = None

    def update_state(self, state: CraneState, position: dict) -> None:
        """从 /localization_pose 数据更新 CraneState。

        位置直接来自定位, 速度使用 Odometry 原生速度 (若可用) 或 PD 指令速度。
        """
        with self._command_lock:
            state.x.position = position['x']
            state.y.position = position['y']
            state.z.position = position['z']
            self._z_height = max(0.0, position['z'])

            # 优先使用 Odometry 原生速度, 若不可用则保留当前状态。
            if position.get('vx') is not None:
                state.x.velocity = position['vx']
                state.y.velocity = position['vy']
                state.z.velocity = position['vz']

    def stop_motion(self) -> None:
        """正常到位：X/Y 归零，Z 保持当前绝对高度。"""
        with self._command_lock:
            self._plc.big_car_ctrl(0.0)
            self._plc.small_car_ctrl(0.0)
            self._plc.lift_ctrl(self._z_height)
            self._plc.last_vx = 0.0
            self._plc.last_vy = 0.0
            self._plc.last_vz = 0.0
            self._plc.last_hz = self._z_height
            self._last_vx = 0.0
            self._last_vy = 0.0
            self._last_z_height = self._z_height

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
