"""
桥式起重机位置目标控制仿真。

调度系统只发送一个目标点 (target_x, target_y, target_z)。
PLC 每个周期读取位置反馈，估计并滤波速度，使用 PD 输出速度指令:

  v_cmd = Kp * (target_position - measured_position) - Kd * filtered_velocity

两种运行模式 (通过 --plc-ip 自动切换):

  仿真模式 (无 --plc-ip):
    - 位置反馈来自仿真对象模型 (CranePlant)
    - 100 Hz 固定步长
    - 命令行动目标, 启动后自动运行
    - --live 时浏览器回放已完成的仿真

  PLC 模式 (指定 --plc-ip):
    - 位置反馈来自 ROS /localization_pose (10 Hz)
    - 10 Hz 事件驱动, 跟随定位数据节奏
    - 开 UI 后设置 Target → Apply Target → 开始 PD 控制
    - HTTP 轮询控制状态，SSE 保留为诊断通道
"""

from __future__ import annotations

import argparse
import math
import time

# Must be set before any matplotlib import (headless mode, no tkinter)
import matplotlib
matplotlib.use('Agg')

from coordinate_transform import CoordinateTransform2D  # noqa: E402
from crane_model import (  # noqa: E402
    CraneConfig,
    CranePlant,
    CraneState,
    PlantActuator,
    SimPositionSource,
    run_pd_control,
)
from live_server import build_live_payload, serve_live_view  # noqa: E402
from plc_interface import PlcActuator, create_plc  # noqa: E402
from ros_bridge import RosPositionSource, get_latest_pose, start_ros_bridge  # noqa: E402
from visualizer import CraneVisualizer  # noqa: E402


# ============================================================================
# 兼容旧接口 — 仿真模式一键运行
# ============================================================================

def run_simulation(
    target_pos: tuple[float, float, float],
    initial_state: CraneState,
    config: CraneConfig,
    verbose: bool = True,
    max_time: float | None = 180.0,
) -> tuple[list[dict], list[tuple[float, str]]]:
    """仿真模式: 使用 plant model 运行一次 PD 控制到完成。

    此函数是对 run_pd_control() 的薄封装, 保持与旧代码/测试的兼容性。
    PLC 模式请直接使用 run_pd_control() + RosPositionSource + PlcActuator。
    """
    plant = CranePlant(config)
    source = SimPositionSource(plant, initial_state, config)
    actuator = PlantActuator(plant, initial_state, config)
    return run_pd_control(
        source=source,
        actuator=actuator,
        config=config,
        target_pos=target_pos,
        initial_state=initial_state,
        verbose=verbose,
        max_time=max_time,
        is_simulation=True,
    )


# ============================================================================
# CLI
# ============================================================================

def _build_arg_parser():
    parser = argparse.ArgumentParser(description='Bridge crane target-position control simulation')
    parser.add_argument('--target-x', type=float, default=8.0, help='target X bridge position')
    parser.add_argument('--target-y', type=float, default=6.0, help='target Y trolley position')
    parser.add_argument('--target-z', type=float, default=1.5, help='target Z hoist position')
    parser.add_argument('--live', action='store_true', help='show browser live view after simulation')
    parser.add_argument('--hz', type=float, default=10.0, help='live display refresh rate in Hz')
    parser.add_argument('--speed', type=float, default=1.0, help='live replay speed multiplier')
    parser.add_argument('--host', default='127.0.0.1', help='live web server host')
    parser.add_argument('--port', type=int, default=8000, help='live web server port')
    parser.add_argument('--plc-ip', default='', help='PLC IP (empty = lab simulation; set to e.g. 192.168.0.1 for real PLC)')
    parser.add_argument('--plc-lib', default='plc_lib/lib/libsscarctrl.so',
                        help='path to libsscarctrl.so')
    parser.add_argument('--allow-mock-plc', action='store_true',
                        help='explicitly allow MockPLC when the real PLC library cannot load')
    parser.add_argument('--map-to-crane-origin-x', type=float, default=0.0,
                        help='crane origin X coordinate in the SLAM map')
    parser.add_argument('--map-to-crane-origin-y', type=float, default=0.0,
                        help='crane origin Y coordinate in the SLAM map')
    parser.add_argument('--map-to-crane-origin-z', type=float, default=0.0,
                        help='crane origin Z coordinate in the SLAM map')
    parser.add_argument('--map-to-crane-roll-deg', type=float, default=0.0,
                        help='roll of the physical crane frame in the SLAM map')
    parser.add_argument('--map-to-crane-pitch-deg', type=float, default=0.0,
                        help='pitch of the physical crane frame in the SLAM map')
    parser.add_argument('--map-to-crane-yaw-deg', type=float, default=0.0,
                        help='yaw of the crane +X rail in the SLAM map, in degrees')
    parser.add_argument('--use-native-z-velocity', action='store_true',
                        help='trust Odometry twist.linear.z instead of height-derived velocity')
    parser.add_argument('--invert-big-car', action='store_true',
                        help='flip big-car (X) command sign when the drive positive '
                             'direction is opposite to the localization X axis')
    parser.add_argument('--invert-small-car', action='store_true',
                        help='flip small-car (Y) command sign when the drive positive '
                             'direction is opposite to the localization Y axis')
    parser.add_argument('--invert-lift', action='store_true',
                        help='flip hoist (Z) command sign when the drive positive '
                             'direction is opposite to the localization Z axis')
    parser.add_argument('--min-lift-height', type=float, default=0.5,
                        help='minimum hoist height (m) sent to liftctrl; keeps the '
                             'gripper at least this far above ground (default 0.5)')
    for axis in ('x', 'y', 'z'):
        parser.add_argument(f'--workspace-{axis}-min', type=float, default=None,
                            help=f'minimum allowed {axis.upper()} target in PLC workspace')
        parser.add_argument(f'--workspace-{axis}-max', type=float, default=None,
                            help=f'maximum allowed {axis.upper()} target in PLC workspace')
    return parser


def _workspace_bounds_from_args(args, axis: str) -> tuple[float, float] | None:
    lower = getattr(args, f'workspace_{axis}_min')
    upper = getattr(args, f'workspace_{axis}_max')
    if lower is None and upper is None:
        return None
    if lower is None or upper is None:
        raise ValueError(
            f'workspace {axis.upper()} requires both --workspace-{axis}-min '
            f'and --workspace-{axis}-max'
        )
    return (lower, upper)


def _config_from_args(args) -> CraneConfig:
    return CraneConfig(
        max_velocity_xy=0.2,
        max_velocity_z=0.2,
        kp_pos=0.6,
        kd_pos=0.45,
        dt=0.01,
        workspace_x_bounds=_workspace_bounds_from_args(args, 'x'),
        workspace_y_bounds=_workspace_bounds_from_args(args, 'y'),
        workspace_z_bounds=_workspace_bounds_from_args(args, 'z'),
    )


def _coordinate_transform_from_args(args) -> CoordinateTransform2D:
    return CoordinateTransform2D.from_degrees(
        origin_map_x=args.map_to_crane_origin_x,
        origin_map_y=args.map_to_crane_origin_y,
        origin_map_z=args.map_to_crane_origin_z,
        crane_roll_deg=args.map_to_crane_roll_deg,
        crane_pitch_deg=args.map_to_crane_pitch_deg,
        crane_x_axis_yaw_deg=args.map_to_crane_yaw_deg,
    )


def _connect_plc(args):
    """Create and connect the selected PLC, failing closed on startup errors."""
    plc = create_plc(
        lib_path=args.plc_lib,
        allow_mock=args.allow_mock_plc,
    )
    try:
        result = plc.connect(args.plc_ip)
        if result != 0:
            raise ConnectionError(
                f'Failed to connect PLC at {args.plc_ip}: ret={result}'
            )
        plc.start_heartbeat()
        return plc
    except Exception:
        plc.disconnect()
        raise


def _query_float(query: dict[str, list[str]], names: tuple[str, ...], default: float) -> float:
    for name in names:
        values = query.get(name)
        if values:
            try:
                return float(values[-1])
            except ValueError as exc:
                raise ValueError(f"{name} must be a number") from exc
    return default


def _target_from_query(
    query: dict[str, list[str]],
    default_target: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        _query_float(query, ("target_x", "x"), default_target[0]),
        _query_float(query, ("target_y", "y"), default_target[1]),
        _query_float(query, ("target_z", "z"), default_target[2]),
    )


def main(argv=None):
    args = _build_arg_parser().parse_args(argv)
    config = _config_from_args(args)
    coordinate_transform = _coordinate_transform_from_args(args)
    crane0 = CraneState(x0=0.0, y0=0.0, z0=0.0)
    map_target_pos = (args.target_x, args.target_y, args.target_z)

    # ---- PLC 模式 ----
    if args.plc_ip:
        target_pos = config.validate_target(
            coordinate_transform.map_to_crane_position(*map_target_pos)
        )
        plc = _connect_plc(args)
        try:
            start_ros_bridge()

            # 等待首次定位数据 (超时 5s)
            print('Waiting for /localization_pose...')
            waited = 0.0
            while get_latest_pose() is None and waited < 5.0:
                time.sleep(0.1)
                waited += 0.1
            # Z 位置来自抓钩实测高度 (物理 Z), SLAM 只提供 X/Y。
            hoist_z = plc.get_lift_height()
            has_hoist_z = hoist_z is not None and math.isfinite(hoist_z)

            pose = get_latest_pose()
            if pose is not None:
                px, py, pz = config.validate_position(
                    coordinate_transform.map_to_crane_position(
                        pose['x'], pose['y'], pose['z']
                    )
                )
                # 有抓钩高度时用它作为初始 Z, 否则退回 SLAM Z。
                if has_hoist_z:
                    pz = float(hoist_z)
                crane0 = CraneState(x0=px, y0=py, z0=pz)
                print(
                    f'Localization received: map=({pose["x"]:.2f}, {pose["y"]:.2f}, {pz:.2f}), '
                    f'crane=({px:.2f}, {py:.2f}, {pz:.2f}), '
                    f'Z source={"hoist" if has_hoist_z else "slam"}'
                )
            else:
                print('Warning: /localization_pose timeout, using default initial position')

            fallback_map = coordinate_transform.crane_to_map_position(
                crane0.x.position,
                crane0.y.position,
                crane0.z.position,
            )
            initial_pos = (
                pose['x'] if pose is not None else fallback_map[0],
                pose['y'] if pose is not None else fallback_map[1],
                # Z 用抓钩高度 (映射回 map)；crane0.z 已优先取抓钩高度。
                fallback_map[2],
            )
            ros_source = RosPositionSource(
                coordinate_transform=coordinate_transform,
                use_native_z_velocity=args.use_native_z_velocity,
                lift_height_provider=plc.get_lift_height,
            )
            plc_actuator = PlcActuator(
                plc,
                initial_z=crane0.z.position,
                big_car_sign=-1.0 if args.invert_big_car else 1.0,
                small_car_sign=-1.0 if args.invert_small_car else 1.0,
                lift_sign=-1.0 if args.invert_lift else 1.0,
                min_lift_height=args.min_lift_height,
            )

            if args.live:
                # PLC 实时控制模式: 开 UI, 等用户 Apply Target
                print('Live view starting — set target in browser and click "Apply Target"')
                serve_live_view(
                    payload=None,  # PLC 模式不需要回放数据
                    host=args.host,
                    port=args.port,
                    payload_factory=None,
                    plc=plc,
                    ros_source=ros_source,
                    plc_actuator=plc_actuator,
                    config=config,
                    initial_pos=initial_pos,
                    update_hz=args.hz,
                    speed=args.speed,
                    coordinate_transform=coordinate_transform,
                )
            else:
                # PLC 模式非 live: 使用命令行 target 直接运行
                history, arrival_events = run_pd_control(
                    source=ros_source,
                    actuator=plc_actuator,
                    config=config,
                    target_pos=target_pos,
                    initial_state=crane0,
                    verbose=True,
                    is_simulation=False,
                )
                viz = CraneVisualizer(config)
                viz.plot(history, arrival_events)
                viz.plot_operation_diagram(
                    history=history,
                    phase_boundaries=arrival_events,
                    target_pos=target_pos,
                    initial_pos=initial_pos,
                )
        finally:
            plc.disconnect()
        return

    # ---- 仿真模式 ----
    target_pos = config.validate_target(map_target_pos)
    history, arrival_events = run_simulation(
        target_pos=target_pos,
        initial_state=crane0,
        config=config,
        verbose=True,
    )

    initial_pos = (crane0.x.position, crane0.y.position, crane0.z.position)
    viz = CraneVisualizer(config)
    viz.plot(history, arrival_events)
    viz.plot_operation_diagram(
        history=history,
        phase_boundaries=arrival_events,
        target_pos=target_pos,
        initial_pos=initial_pos,
    )

    if args.live:
        # 仿真回放模式: 直接回放已完成的仿真
        _state = {'target': target_pos, 'initial': initial_pos}

        def live_payload_for_query(query: dict[str, list[str]]):
            if not query:
                return payload
            start_pos = _state['target']
            requested_target = _target_from_query(query, _state['target'])
            live_history, live_events = run_simulation(
                target_pos=requested_target,
                initial_state=CraneState(x0=start_pos[0], y0=start_pos[1], z0=start_pos[2]),
                config=config,
                verbose=False,
            )
            _state['initial'] = start_pos
            _state['target'] = requested_target
            return build_live_payload(
                history=live_history,
                phase_boundaries=live_events,
                target_pos=requested_target,
                initial_pos=start_pos,
                config=config,
                update_hz=args.hz,
                speed=args.speed,
            )

        payload = build_live_payload(
            history=history,
            phase_boundaries=arrival_events,
            target_pos=target_pos,
            initial_pos=initial_pos,
            config=config,
            update_hz=args.hz,
            speed=args.speed,
        )
        serve_live_view(
            payload,
            host=args.host,
            port=args.port,
            payload_factory=live_payload_for_query,
            plc=None,
        )


if __name__ == '__main__':
    main()
