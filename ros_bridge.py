"""ROS 1 bridge: subscribes to /localization_pose (nav_msgs/Odometry) and exposes
the latest pose via a thread-safe accessor for the browser live view.

Also provides RosPositionSource — a PositionSource implementation for the
unified PD control loop that blocks until new localization data arrives.

Target: ROS 1 Noetic (Ubuntu 20.04, ARM domain controller).
Tested in noetic Docker container.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable

from coordinate_transform import CoordinateTransform2D
from sway_estimator import InclinometerSwayEstimator, SwayAxisMapping
from sway_settle import SwayLiveAnalyzer


# ---- raw pose accessor (thread-safe, non-blocking) ----

_latest_pose: dict[str, Any] | None = None
_lock = threading.Lock()
_node_started = False


def _odom_callback(msg: Any) -> None:
    """Store latest odometry data in a thread-safe shared dict."""
    global _latest_pose
    with _lock:
        _latest_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'vz': msg.twist.twist.linear.z,
            # ROS 1: stamp uses .secs / .nsecs
            'stamp_sec': msg.header.stamp.secs,
            'stamp_nsec': msg.header.stamp.nsecs,
        }


def get_latest_pose() -> dict[str, Any] | None:
    """Return a copy of the latest localization pose, or None if no data yet."""
    with _lock:
        if _latest_pose is None:
            return None
        return dict(_latest_pose)


# ---- gripper status from /crane/cmd_vel/anguler (thread-safe) ----

_latest_gripper: dict[str, Any] | None = None
_gripper_lock = threading.Lock()

# Gripper status constants decoded from angular.x:
#   -1 = gripper open/released (开启/释放)
#    1 = gripper closed/clamped (关闭/夹紧)
GRIPPER_OPEN = -1
GRIPPER_CLOSED = 1


def _gripper_callback(msg: Any) -> None:
    """Store latest gripper status from /crane/cmd_vel/anguler.

    Subscribes to geometry_msgs/Twist. The gripper state is in angular.x:
      angular.x = -1  → gripper released (open / 开启)
      angular.x =  1  → gripper clamped (closed / 关闭)
    """
    global _latest_gripper
    with _gripper_lock:
        _latest_gripper = {
            'angular_x': msg.angular.x,
            'angular_y': msg.angular.y,
            'angular_z': msg.angular.z,
        }


def get_gripper_status() -> dict[str, Any] | None:
    """Return a copy of the latest gripper status, or None if no data yet.

    Returns:
        dict with 'angular_x' (-1=open, 1=closed) and other fields, or None.
    """
    with _gripper_lock:
        if _latest_gripper is None:
            return None
        return dict(_latest_gripper)


def is_gripper_clamped() -> bool | None:
    """Convenience: True if gripper is clamped (closed), False if released (open),
    None if no data available or value is ambiguous."""
    status = get_gripper_status()
    if status is None:
        return None
    ax = status.get('angular_x')
    if ax is None:
        return None
    if ax >= 0.5:
        return True   # 1 = closed/clamped
    if ax <= -0.5:
        return False  # -1 = open/released
    return None  # ambiguous value


# ---- sway sensor: /inclination/j1939_msg (thread-safe) ----

_latest_inclination: dict[str, Any] | None = None
_sway_lock = threading.Lock()
_sway_estimator: InclinometerSwayEstimator | None = None
_sway_live: SwayLiveAnalyzer | None = None
_sway_has_data = False
_last_inclination_wall: float | None = None


def _deserialize_any(msg: Any) -> Any | None:
    """把 rospy.AnyMsg 反序列化为实际消息对象 (容错, 失败返回 None)。

    话题 /inclination/j1939_msg 的消息类型可能为设备自定义, 这里不硬依赖具体
    类型: 用 AnyMsg 订阅 + connection header 里的类型串反序列化。
    """
    try:
        from roslib.message import get_message_class
    except ImportError:
        return None
    header = getattr(msg, '_connection_header', None) or {}
    msg_type = header.get('type')
    if not msg_type:
        return None
    klass = get_message_class(msg_type)
    if klass is None and '/' in msg_type:
        pkg, _, tname = msg_type.partition('/')
        try:
            mod = __import__(f'{pkg}.msg', fromlist=[tname])
            klass = getattr(mod, tname, None)
        except Exception:
            klass = None
    if klass is None:
        return None
    try:
        real = klass()
        real.deserialize(getattr(msg, '_buff', b''))
        return real
    except Exception:
        return None


def _inclination_callback(msg: Any) -> None:
    global _latest_inclination, _sway_has_data, _last_inclination_wall
    real = _deserialize_any(msg)
    if real is None:
        return
    # j1939 vector (现场安装确认 2026-09-18):
    #   x=倾角仪 X 轴 ∥ 小航车轴, 正向一致  → θy (sign +1)
    #   y=倾角仪 Y 轴 ∥ 大航车轴, 正向相反  → θx (sign -1)
    # 内部仍称 roll=vector.x, pitch=vector.y; 映射到 θx/θy 见 SwayAxisMapping/site_config。
    vec = getattr(real, 'vector', None)
    roll = getattr(vec, 'x', None) if vec is not None else getattr(real, 'roll', None)
    pitch = getattr(vec, 'y', None) if vec is not None else getattr(real, 'pitch', None)
    if roll is None and pitch is None:
        return
    with _sway_lock:
        _latest_inclination = {'roll': roll, 'pitch': pitch}
        _last_inclination_wall = time.monotonic()
        _sway_has_data = True
        if _sway_estimator is not None and _sway_live is not None:
            sway = _sway_estimator.estimate(roll, pitch)
            _sway_live.push(_last_inclination_wall, sway['theta_x'], sway['theta_y'])


def get_inclination_raw() -> dict[str, float] | None:
    """返回最新倾角仪原始 roll/pitch (调试/启动自检用)。"""
    with _sway_lock:
        if not _sway_has_data or _latest_inclination is None:
            return None
        return dict(_latest_inclination)


def get_sway_state() -> dict[str, Any] | None:
    """返回最新倾角仪摆角状态 {theta_x, theta_y}; 无数据或断流时返回 None。"""
    with _sway_lock:
        if _sway_estimator is None or not _sway_has_data or _last_inclination_wall is None:
            return None
        if time.monotonic() - _last_inclination_wall > 1.0:  # 数据断流 → 防摇安全退出
            return None
        inc = _latest_inclination or {}
        gripper_closed = is_gripper_clamped()
        return _sway_estimator.estimate(
            inc.get('roll'), inc.get('pitch'), gripper_closed=gripper_closed,
        )


def configure_sway_live_analyzer(
    *,
    base_window: float = 0.6,
    window_periods: float = 0.55,
    window_max: float = 8.0,
    sheave_height: float = 4.0,
    grab_offset: float = 0.0,
    tol_deg: float = 0.3,
) -> None:
    """启用实时摆角分析 (与调度器判稳窗/去趋势 PP 一致)。"""
    global _sway_live
    with _sway_lock:
        _sway_live = SwayLiveAnalyzer(
            base_window=base_window,
            window_periods=window_periods,
            window_max=window_max,
            sheave_height=sheave_height,
            grab_offset=grab_offset,
            tol_deg=tol_deg,
        )


def get_sway_live_metrics(z_grab: float) -> dict[str, Any] | None:
    """返回实时判稳指标; 未 configure 或无样本时返回 None。"""
    with _sway_lock:
        if _sway_live is None:
            return None
        return _sway_live.metrics(z_grab)


def get_sway_snapshot(*, stale_after_s: float = 1.0) -> dict[str, Any]:
    """返回 UI/调试用的倾角仪快照 (原始值 + 补偿后摆角, 度制便于展示)。"""
    with _sway_lock:
        if not _sway_has_data or _latest_inclination is None or _last_inclination_wall is None:
            return {'valid': False, 'stale': True}
        age_s = time.monotonic() - _last_inclination_wall
        raw = dict(_latest_inclination)
        stale = age_s > stale_after_s
        sway = None
        if _sway_estimator is not None and not stale:
            gripper_closed = is_gripper_clamped()
            sway = _sway_estimator.estimate(
                raw.get('roll'), raw.get('pitch'), gripper_closed=gripper_closed,
            )
    if stale:
        return {
            'valid': False,
            'stale': True,
            'age_ms': round(age_s * 1000.0),
            'roll': raw.get('roll'),
            'pitch': raw.get('pitch'),
        }
    theta_x = float(sway['theta_x']) if sway else 0.0
    theta_y = float(sway['theta_y']) if sway else 0.0
    theta_x_deg = math.degrees(theta_x)
    theta_y_deg = math.degrees(theta_y)
    return {
        'valid': True,
        'stale': False,
        'age_ms': round(age_s * 1000.0),
        'roll': raw.get('roll'),
        'pitch': raw.get('pitch'),
        'theta_x': theta_x,
        'theta_y': theta_y,
        'theta_x_deg': theta_x_deg,
        'theta_y_deg': theta_y_deg,
        'theta_peak_deg': max(abs(theta_x_deg), abs(theta_y_deg)),
    }


class SwaySensorSource:
    """供 run_pd_control 读取倾角仪摆角状态的轻量源 (类似 PositionSource 接口)。"""

    def get_sway(self) -> dict[str, Any] | None:
        return get_sway_state()


def _ros_spin() -> None:
    """Blocking ROS 1 spin loop — runs in a daemon thread."""
    try:
        import rospy
    except ImportError:
        print('[ros_bridge] rospy not available — localization data will be empty')
        return

    try:
        from nav_msgs.msg import Odometry
    except ImportError:
        print('[ros_bridge] nav_msgs not available — localization data will be empty')
        return

    try:
        from geometry_msgs.msg import Twist
    except ImportError:
        print('[ros_bridge] geometry_msgs not available — gripper status will be unavailable')
        Twist = None

    # Initialize ROS node (safe if already initialized in-process)
    try:
        rospy.init_node('lst_control_localization', anonymous=True, disable_signals=True)
    except rospy.exceptions.ROSException:
        print('[ros_bridge] ROS node already initialized (reusing existing node)')
    except Exception as exc:
        print(f'[ros_bridge] Failed to init ROS node: {exc}')
        return

    rospy.Subscriber('/localization_pose', Odometry, _odom_callback)
    print('[ros_bridge] Subscribed to /localization_pose (nav_msgs/Odometry)')

    if Twist is not None:
        try:
            rospy.Subscriber('/crane/cmd_vel/anguler', Twist, _gripper_callback)
            print('[ros_bridge] Subscribed to /crane/cmd_vel/anguler (geometry_msgs/Twist)')
        except Exception as exc:
            print(f'[ros_bridge] Failed to subscribe to /crane/cmd_vel/anguler: {exc}')
            print('[ros_bridge] Gripper status via ROS will be unavailable')

    # 倾角仪订阅 (best-effort): 话题/类型缺失时不影响主流程。
    # 用 AnyMsg 订阅以兼容自定义 j1939 消息类型, 回调内再容错反序列化。
    try:
        rospy.Subscriber('/inclination/j1939_msg', rospy.AnyMsg, _inclination_callback)
        print('[ros_bridge] Subscribed to /inclination/j1939_msg (AnyMsg)')
    except Exception as exc:
        print(f'[ros_bridge] Failed to subscribe to /inclination/j1939_msg: {exc}')
        print('[ros_bridge] Sway sensing will be unavailable')

    rospy.spin()


def start_ros_bridge(
    angle_scale: float = 1.0,
    roll_bias: float = 0.0,
    pitch_bias: float = 0.0,
    axis_mapping: SwayAxisMapping | None = None,
    *,
    roll_bias_closed: float | None = None,
    pitch_bias_closed: float | None = None,
) -> None:
    """Start the ROS 1 subscriber in a daemon thread. Safe to call multiple times.

    angle_scale: 倾角仪原始值 → 弧度 的换算系数 (角度制填 math.pi/180)。
    roll_bias/pitch_bias: 零点偏置 (roll=倾角仪X, pitch=倾角仪Y), 与原始读数同单位。
    axis_mapping: SwayAxisMapping, 将 roll/pitch 映射到 θx/θy。
    """
    global _node_started, _sway_estimator, _sway_live
    if _node_started:
        return
    _node_started = True
    with _sway_lock:
        if _sway_estimator is None:
            _sway_estimator = InclinometerSwayEstimator(
                mapping=axis_mapping,
                angle_scale=angle_scale,
                roll_bias=roll_bias,
                pitch_bias=pitch_bias,
                roll_bias_closed=roll_bias_closed,
                pitch_bias_closed=pitch_bias_closed,
            )
        if _sway_live is None:
            _sway_live = SwayLiveAnalyzer()
    thread = threading.Thread(target=_ros_spin, name='ros-bridge', daemon=True)
    thread.start()


# ============================================================================
# RosPositionSource — 适配统一 PD 控制循环的 PositionSource 接口
# ============================================================================

class RosPositionSource:
    """10 Hz 阻塞位置源 — 等待新的 /localization_pose 数据。

    用于 PLC 模式的 run_pd_control()。
    每次 get_position() 阻塞直到新数据到达 (stamp 不同于上次)。

    超时保护: 2 秒无新数据 → 返回 None → 触发安全停止。

    Z 轴位置来源:
      SLAM 的 Map Z 在地图倾斜/漂移时不可靠。若提供 lift_height_provider
      (通常是 PLC 的 GetActualLiftHeight)，Z 位置改用抓钩实测高度 (物理 Z,
      Z=0 地面, 向上为正)，不经坐标旋转; 该情况下 Z 速度置 None, 由控制
      循环用高度差分+低通估计。X/Y 仍来自 SLAM 定位。
    """

    _POSITION_TIMEOUT = 2.0  # [s] 定位断流超时

    def __init__(
        self,
        coordinate_transform: CoordinateTransform2D | None = None,
        *,
        use_native_xy_velocity: bool = True,
        use_native_z_velocity: bool = False,
        lift_height_provider: Callable[[], float | None] | None = None,
    ):
        self._coordinate_transform = (
            coordinate_transform or CoordinateTransform2D.identity()
        )
        self._use_native_xy_velocity = use_native_xy_velocity
        self._use_native_z_velocity = use_native_z_velocity
        self._lift_height_provider = lift_height_provider
        self._last_stamp: float | None = None   # ROS stamp (sec + nsec*1e-9)
        self._t0: float | None = None           # 首次数据到达的单调时间
        self._last_wall: float | None = None    # 上次 get_position 的单调时间
        self._t: float = 0.0                    # 累计运行时间 [s]

    def get_position(self) -> dict | None:
        """阻塞等待新的 /localization_pose 数据。

        Returns:
            dict with x, y, z, vx, vy, vz, dt, t, stamp — or None on timeout.
        """
        deadline = time.monotonic() + self._POSITION_TIMEOUT
        while time.monotonic() < deadline:
            pose = get_latest_pose()
            if pose is None:
                time.sleep(0.01)
                continue

            stamp = pose['stamp_sec'] + pose['stamp_nsec'] * 1e-9
            if stamp == self._last_stamp:
                # 同一帧数据, 等待下一个
                time.sleep(0.01)
                continue

            # 新数据到达
            now = time.monotonic()
            if self._t0 is None:
                self._t0 = now
                self._t = 0.0
                dt = 0.1  # 首次, 假设 10 Hz
            else:
                dt = now - self._last_wall if self._last_wall is not None else 0.1
                self._t += dt

            self._last_stamp = stamp
            self._last_wall = now

            crane_x, crane_y, crane_z = (
                self._coordinate_transform.map_to_crane_position(
                    pose['x'], pose['y'], pose['z']
                )
            )

            # Z 位置优先取抓钩实测高度 (物理 Z, 不经坐标旋转); 取到有效值时
            # 覆盖 SLAM 的 Map Z, 并令 Z 速度改由高度差分估计。
            z_from_hoist = False
            if self._lift_height_provider is not None:
                lift_height = self._lift_height_provider()
                if lift_height is not None and math.isfinite(lift_height):
                    crane_z = float(lift_height)
                    z_from_hoist = True

            vx = vy = vz = None
            if self._use_native_xy_velocity:
                map_vx = pose.get('vx')
                map_vy = pose.get('vy')
                map_vz = pose.get('vz')
                if (
                    map_vx is not None
                    and map_vy is not None
                    and self._use_native_z_velocity
                    and map_vz is not None
                ):
                    vx, vy, vz = self._coordinate_transform.map_to_crane_vector3(
                        map_vx,
                        map_vy,
                        map_vz,
                    )
                elif (
                    map_vx is not None
                    and map_vy is not None
                    and self._coordinate_transform.is_planar
                ):
                    vx, vy = self._coordinate_transform.map_to_crane_vector(
                        map_vx,
                        map_vy,
                    )

            # 用抓钩高度做 Z 时, SLAM 的 Map Vz 与该高度无关, 一律弃用,
            # 让控制循环从抓钩高度差分估计 Z 速度。
            if z_from_hoist:
                vz = None

            return {
                'x': crane_x,
                'y': crane_y,
                'z': crane_z,
                'vx': vx,
                'vy': vy,
                'vz': vz,
                'dt': dt,
                't': self._t,
                'stamp': stamp,
            }

        # 超时 — 定位断流
        return None

    def reset(self) -> None:
        """重置时间基准 (新控制运行开始时调用)。"""
        self._last_stamp = None
        self._t0 = None
        self._last_wall = None
        self._t = 0.0
