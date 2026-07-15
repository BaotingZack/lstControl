"""ROS 1 bridge: subscribes to /localization_pose (nav_msgs/Odometry) and exposes
the latest pose via a thread-safe accessor for the browser live view.

Also provides RosPositionSource — a PositionSource implementation for the
unified PD control loop that blocks until new localization data arrives.

Target: ROS 1 Noetic (Ubuntu 20.04, ARM domain controller).
Tested in noetic Docker container.
"""

from __future__ import annotations

import threading
import time
from typing import Any


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
    rospy.spin()


def start_ros_bridge() -> None:
    """Start the ROS 1 subscriber in a daemon thread. Safe to call multiple times."""
    global _node_started
    if _node_started:
        return
    _node_started = True
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
    """

    _POSITION_TIMEOUT = 2.0  # [s] 定位断流超时

    def __init__(self):
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

            return {
                'x': pose['x'],
                'y': pose['y'],
                'z': pose['z'],
                'vx': pose.get('vx'),  # native Odometry velocity (未使用, 保留)
                'vy': pose.get('vy'),
                'vz': pose.get('vz'),
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
