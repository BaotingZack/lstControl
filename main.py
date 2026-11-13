"""
桥式起重机位置目标控制仿真。

调度系统只发送一个目标点 (target_x, target_y, target_z)。
PLC 每个周期读取位置反馈，估计并滤波速度，使用 PD 输出速度指令:

  v_cmd = Kp * (target_position - measured_position) - Kd * filtered_velocity

两种运行方式:

  默认 `python main.py`:
    - 打开浏览器（/ptp/sim 与 /ptp/live 子页）
    - 默认尝试连接 PLC 192.168.0.1；连不上仍可用仿真子页
    - 使用 --no-live 则只跑命令行仿真/控制并保存曲线图

  指定 --plc-ip / --no-plc 等见 --help。
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
    *,
    enable_anti_sway: bool = False,
    anti_sway_mode: str = 'off',
    site: dict | None = None,
) -> tuple[list[dict], list[tuple[float, str]]]:
    """仿真模式: 使用 plant model 运行一次 PD 控制到完成。

    anti_sway_mode: off | incl (K·θ) | gantry (并联角度 PD)。legacy enable_anti_sway=True → incl。
    """
    from anti_sway.sim_feedback import SimSwayFeedback
    from anti_sway.ptp_config import apply_ptp_anti_sway_mode, normalize_anti_sway_mode

    mode = normalize_anti_sway_mode(anti_sway_mode, legacy_enabled=enable_anti_sway)
    run_config = apply_ptp_anti_sway_mode(config, mode, site or {})
    sway_source = None
    if mode != 'off' and mode != 'input_shape':
        sway_source = SimSwayFeedback(
            sheave_height=run_config.rope_length_sheave_height,
            grab_offset=run_config.rope_length_grab_offset,
        )
        sway_source.reset()
    plant = CranePlant(run_config)
    source = SimPositionSource(plant, initial_state, run_config)
    actuator = PlantActuator(plant, initial_state, run_config)
    return run_pd_control(
        source=source,
        actuator=actuator,
        config=run_config,
        target_pos=target_pos,
        initial_state=initial_state,
        verbose=verbose,
        max_time=max_time,
        is_simulation=True,
        sway_source=sway_source,
    )


# ============================================================================
# CLI
# ============================================================================

def _build_arg_parser():
    parser = argparse.ArgumentParser(description='Bridge crane target-position control simulation')
    parser.add_argument('--target-x', type=float, default=8.0, help='target X bridge position')
    parser.add_argument('--target-y', type=float, default=6.0, help='target Y trolley position')
    parser.add_argument('--target-z', type=float, default=1.5, help='target Z hoist position')
    parser.add_argument(
        '--live', dest='live', action='store_true', default=True,
        help='open browser UI (default: on)',
    )
    parser.add_argument(
        '--no-live', dest='live', action='store_false',
        help='batch-only: no browser UI',
    )
    parser.add_argument('--hz', type=float, default=10.0, help='live display refresh rate in Hz')
    parser.add_argument('--speed', type=float, default=1.0, help='live replay speed multiplier')
    parser.add_argument('--host', default='127.0.0.1', help='live web server host')
    parser.add_argument('--port', type=int, default=8000, help='live web server port')
    parser.add_argument('--plc-ip', default='192.168.0.1',
                        help='PLC IP (default 192.168.0.1; use "" or --no-plc to skip PLC)')
    parser.add_argument('--no-plc', action='store_true',
                        help='do not connect PLC; simulation subpage only')
    parser.add_argument('--plc-lib', default='plc_lib/lib/libsscarctrl.so',
                        help='path to libsscarctrl.so')
    parser.add_argument('--allow-mock-plc', action='store_true',
                        help='explicitly allow MockPLC when the real PLC library cannot load')
    from site_config import DEFAULT_SITE_CONFIG_PATH  # noqa: E402
    parser.add_argument('--config', type=str, default=str(DEFAULT_SITE_CONFIG_PATH),
                        help='site config JSON (inclinometer calibration; default site_config.json)')
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
    parser.add_argument('--min-lift-height', type=float, default=0.35,
                        help='minimum hoist height (m) sent to liftctrl; keeps the '
                             'gripper at least this far above ground (default 0.35)')
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
        dt=0.01,
        workspace_x_bounds=_workspace_bounds_from_args(args, 'x'),
        workspace_y_bounds=_workspace_bounds_from_args(args, 'y'),
        workspace_z_bounds=_workspace_bounds_from_args(args, 'z'),
    )


def _coordinate_transform_from_args(args) -> CoordinateTransform2D:
    from dataclasses import replace

    site = _load_site_settings(args)
    transform = CoordinateTransform2D.from_degrees(
        origin_map_x=args.map_to_crane_origin_x,
        origin_map_y=args.map_to_crane_origin_y,
        origin_map_z=args.map_to_crane_origin_z,
        crane_roll_deg=args.map_to_crane_roll_deg,
        crane_pitch_deg=args.map_to_crane_pitch_deg,
        crane_x_axis_yaw_deg=args.map_to_crane_yaw_deg,
    )
    if bool(site.get('localization_negate_xy', False)):
        transform = replace(transform, localization_negate_xy=True)
        print('[PLC] Localization feedback: negate XY after map→crane (site_config)')
    return transform


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


def _query_bool(query: dict[str, list[str]], name: str, default: bool = False) -> bool:
    values = query.get(name)
    if not values:
        return default
    return str(values[-1]).strip().lower() in ('1', 'true', 'yes', 'on')


def _anti_sway_mode_from_query(query: dict[str, list[str]]) -> str:
    from site_config import normalize_anti_sway_mode

    values = query.get('anti_sway_mode')
    mode = str(values[-1]).strip().lower() if values else None
    legacy = _query_bool(query, 'anti_sway', False)
    return normalize_anti_sway_mode(mode, legacy_enabled=legacy)


def _start_live_server(
    *,
    config: CraneConfig,
    args: argparse.Namespace,
    map_target_pos: tuple[float, float, float],
    coordinate_transform: CoordinateTransform2D,
    plc=None,
    ros_source=None,
    plc_actuator=None,
    initial_pos=None,
) -> None:
    """Start HTTP UI with simulation replay (/ptp/sim) and optional PLC (/ptp/live)."""
    from live_server import build_live_payload, serve_live_view

    target_pos = config.validate_target(map_target_pos)
    crane0 = CraneState(x0=0.0, y0=0.0, z0=0.0)
    site = _load_site_settings(args)
    history, arrival_events = run_simulation(
        target_pos=target_pos,
        initial_state=crane0,
        config=config,
        verbose=False,
        enable_anti_sway=False,
        site=site,
    )
    initial_pos_sim = (crane0.x.position, crane0.y.position, crane0.z.position)
    payload = build_live_payload(
        history=history,
        phase_boundaries=arrival_events,
        target_pos=target_pos,
        initial_pos=initial_pos_sim,
        config=config,
        update_hz=args.hz,
        speed=args.speed,
    )
    state = {"target": target_pos, "initial": initial_pos_sim}

    def live_payload_for_query(query: dict[str, list[str]]):
        if not query:
            return payload
        start_pos = state["target"]
        requested_target = _target_from_query(query, state["target"])
        anti_sway = _query_bool(query, 'anti_sway', False)
        mode = _anti_sway_mode_from_query(query)
        live_history, live_events = run_simulation(
            target_pos=requested_target,
            initial_state=CraneState(
                x0=start_pos[0], y0=start_pos[1], z0=start_pos[2]
            ),
            config=config,
            verbose=False,
            enable_anti_sway=anti_sway,
            anti_sway_mode=mode,
            site=site,
        )
        state["initial"] = start_pos
        state["target"] = requested_target
        return build_live_payload(
            history=live_history,
            phase_boundaries=live_events,
            target_pos=requested_target,
            initial_pos=start_pos,
            config=config,
            update_hz=args.hz,
            speed=args.speed,
        )

    sway_tol = float(site.get('hook_sway_angle_tol_deg', 0.3))

    serve_live_view(
        payload,
        host=args.host,
        port=args.port,
        payload_factory=live_payload_for_query,
        plc=plc,
        ros_source=ros_source,
        plc_actuator=plc_actuator,
        config=config,
        initial_pos=initial_pos,
        update_hz=args.hz,
        speed=args.speed,
        coordinate_transform=coordinate_transform,
        hook_sway_angle_tol_deg=sway_tol,
    )


def _effective_plc_ip(args) -> str | None:
    if getattr(args, 'no_plc', False):
        return None
    raw = (args.plc_ip or '').strip()
    if not raw or raw.lower() in ('none', 'off'):
        return None
    return raw


def _load_site_settings(args) -> dict:
    from site_config import DEFAULT_SITE_CONFIG_PATH, load_site_config

    path = getattr(args, 'config', None) or str(DEFAULT_SITE_CONFIG_PATH)
    return load_site_config(path)


def _start_ros_bridge_from_site(args) -> None:
    """Subscribe /localization_pose and /inclination/j1939_msg with site biases."""
    from site_config import build_sway_axis_mapping

    site = _load_site_settings(args)
    mapping = build_sway_axis_mapping(site)
    angle_deg = bool(site.get('anti_sway_angle_deg', True))
    scale = math.pi / 180.0 if angle_deg else 1.0
    rb_closed = site.get('anti_sway_roll_bias_closed')
    pb_closed = site.get('anti_sway_pitch_bias_closed')
    start_ros_bridge(
        angle_scale=scale,
        roll_bias=float(site.get('anti_sway_roll_bias', 0.0)),
        pitch_bias=float(site.get('anti_sway_pitch_bias', 0.0)),
        axis_mapping=mapping,
        roll_bias_closed=float(rb_closed) if rb_closed is not None else None,
        pitch_bias_closed=float(pb_closed) if pb_closed is not None else None,
    )


def _plc_axis_sign(*, cli_invert: bool, site: dict, site_key: str) -> float:
    """+1 normal, -1 when CLI --invert-* or site_config plc_invert_* is true."""
    if cli_invert or bool(site.get(site_key, False)):
        return -1.0
    return 1.0


def _setup_plc_runtime(args, config, coordinate_transform, map_target_pos):
    """Connect PLC + ROS bridge; return (plc, ros_source, plc_actuator, initial_pos)."""
    plc = _connect_plc(args)
    _start_ros_bridge_from_site(args)
    site = _load_site_settings(args)

    print('Waiting for /localization_pose...')
    waited = 0.0
    while get_latest_pose() is None and waited < 5.0:
        time.sleep(0.1)
        waited += 0.1

    hoist_z = plc.get_lift_height()
    has_hoist_z = hoist_z is not None and math.isfinite(hoist_z)

    target_pos = config.validate_target(
        coordinate_transform.map_to_crane_target(
            *map_target_pos, z_is_hoist_height=has_hoist_z,
        )
    )

    crane0 = CraneState(x0=0.0, y0=0.0, z0=0.0)
    pose = get_latest_pose()
    if pose is not None:
        px, py, pz = config.validate_position(
            coordinate_transform.localization_map_to_crane_position(
                pose['x'], pose['y'], pose['z']
            )
        )
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

    fallback_map = coordinate_transform.crane_to_map_display(
        crane0.x.position,
        crane0.y.position,
        crane0.z.position,
        z_is_hoist_height=has_hoist_z,
    )
    initial_pos = (
        pose['x'] if pose is not None else fallback_map[0],
        pose['y'] if pose is not None else fallback_map[1],
        fallback_map[2],
    )
    ros_source = RosPositionSource(
        coordinate_transform=coordinate_transform,
        use_native_z_velocity=args.use_native_z_velocity,
        lift_height_provider=plc.get_lift_height,
    )
    big_sign = _plc_axis_sign(cli_invert=args.invert_big_car, site=site, site_key='plc_invert_big_car')
    small_sign = _plc_axis_sign(cli_invert=args.invert_small_car, site=site, site_key='plc_invert_small_car')
    lift_sign = _plc_axis_sign(cli_invert=args.invert_lift, site=site, site_key='plc_invert_lift')
    if big_sign < 0 or small_sign < 0 or lift_sign < 0:
        print(
            f'[PLC] Axis sign flip: X={big_sign:+.0f} Y={small_sign:+.0f} Z={lift_sign:+.0f} '
            f'(CLI invert-* and/or site_config plc_invert_*)'
        )
    plc_actuator = PlcActuator(
        plc,
        initial_z=crane0.z.position,
        big_car_sign=big_sign,
        small_car_sign=small_sign,
        lift_sign=lift_sign,
        min_lift_height=args.min_lift_height,
    )
    return plc, ros_source, plc_actuator, initial_pos, target_pos, crane0


def _run_live_mode(
    args,
    config: CraneConfig,
    coordinate_transform: CoordinateTransform2D,
    map_target_pos: tuple[float, float, float],
) -> None:
    """Browser UI with /ptp/sim always; /ptp/live when PLC connects."""
    plc_ip = _effective_plc_ip(args)
    plc = None
    ros_source = None
    plc_actuator = None
    initial_pos = None

    if plc_ip:
        args.plc_ip = plc_ip
        try:
            plc, ros_source, plc_actuator, initial_pos, _, _ = _setup_plc_runtime(
                args, config, coordinate_transform, map_target_pos,
            )
            print(f'PLC connected at {plc_ip}')
        except Exception as exc:
            print(f'Warning: PLC at {plc_ip} unavailable ({exc})')
            print('Live UI will run; use /ptp/sim. /ptp/live needs a working PLC.')
            plc = None
            ros_source = None
            plc_actuator = None
            initial_pos = None
            try:
                _start_ros_bridge_from_site(args)
                print('ROS bridge started for inclinometer (PLC not connected).')
            except Exception as ros_exc:
                print(f'Warning: ROS bridge not started ({ros_exc})')

    try:
        _start_live_server(
            config=config,
            args=args,
            map_target_pos=map_target_pos,
            coordinate_transform=coordinate_transform,
            plc=plc,
            ros_source=ros_source,
            plc_actuator=plc_actuator,
            initial_pos=initial_pos,
        )
    finally:
        if plc is not None:
            plc.disconnect()


def main(argv=None):
    from plc_interface import ensure_plc_library_path

    ensure_plc_library_path()
    args = _build_arg_parser().parse_args(argv)
    config = _config_from_args(args)
    coordinate_transform = _coordinate_transform_from_args(args)
    crane0 = CraneState(x0=0.0, y0=0.0, z0=0.0)
    map_target_pos = (args.target_x, args.target_y, args.target_z)

    if args.live:
        _run_live_mode(args, config, coordinate_transform, map_target_pos)
        return

    plc_ip = _effective_plc_ip(args)
    # ---- PLC 模式 (batch) ----
    if plc_ip:
        args.plc_ip = plc_ip
        plc = None
        try:
            plc, ros_source, plc_actuator, initial_pos, target_pos, crane0 = _setup_plc_runtime(
                args, config, coordinate_transform, map_target_pos,
            )
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
            if plc is not None:
                plc.disconnect()
        return

    # ---- 仿真模式 (batch) ----
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


if __name__ == '__main__':
    main()
