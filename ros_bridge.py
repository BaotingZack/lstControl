"""ROS 1 bridge: subscribes to /localization_pose (nav_msgs/Odometry) and exposes
the latest pose via a thread-safe accessor for the browser live view.

Target: ROS 1 Noetic (Ubuntu 20.04, ARM domain controller).
Tested in noetic Docker container.
"""

from __future__ import annotations

import threading
from typing import Any

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
