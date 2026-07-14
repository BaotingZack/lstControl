"""
桥式起重机位置目标控制仿真。

调度系统只发送一个目标点 (target_x, target_y, target_z)。
PLC 每个周期读取位置反馈，估计并滤波速度，使用 PD 输出速度指令:

  v_cmd = Kp * (target_position - measured_position) - Kd * filtered_velocity

被控对象模拟速度模式伺服的一阶响应，并叠加低频扰动和测量噪声。
"""

from __future__ import annotations

# Must be set before any matplotlib import (headless mode, no tkinter)
import matplotlib
matplotlib.use('Agg')

import argparse
import time

from crane_model import CraneConfig, CranePlant, CraneState
from live_server import build_live_payload, serve_live_view
from pd_controller import PositionPDController
from plc_interface import create_plc, PLCInterface
from ros_bridge import get_latest_pose, start_ros_bridge
from velocity_filter import LowPassVelocityEstimator
from visualizer import CraneVisualizer


def _axis_arrived(axis, target: float, config: CraneConfig) -> bool:
    pos_ok = abs(axis.position - target) < config.arrival_pos_tol
    vel_ok = abs(axis.velocity) < config.arrival_vel_tol
    return pos_ok and vel_ok


def _copy_state(initial_state: CraneState) -> CraneState:
    crane = CraneState(
        x0=initial_state.x.position,
        y0=initial_state.y.position,
        z0=initial_state.z.position,
    )
    crane.x.velocity = initial_state.x.velocity
    crane.y.velocity = initial_state.y.velocity
    crane.z.velocity = initial_state.z.velocity
    return crane


def run_simulation(
    target_pos: tuple[float, float, float],
    initial_state: CraneState,
    config: CraneConfig,
    verbose: bool = True,
    max_time: float | None = 180.0,
    plc: PLCInterface | None = None,
) -> tuple[list[dict], list[tuple[float, str]]]:
    """Run one dispatch target command until all three axes arrive.

    When *plc* is provided, velocity commands are sent to the real PLC
    instead of the simulated plant.  Z-axis velocity is integrated to an
    absolute height for ``liftctrl``.
    """
    dt = config.dt
    crane = _copy_state(initial_state)
    plant = CranePlant(config)
    target_x, target_y, target_z = target_pos

    controllers = {
        'x': PositionPDController(config.kp_pos, config.kd_pos, config.max_velocity_xy),
        'y': PositionPDController(config.kp_pos, config.kd_pos, config.max_velocity_xy),
        'z': PositionPDController(config.kp_pos, config.kd_pos, config.max_velocity_z),
    }
    filters = {
        'x': LowPassVelocityEstimator(config.velocity_filter_tau, crane.x.position, crane.x.velocity),
        'y': LowPassVelocityEstimator(config.velocity_filter_tau, crane.y.position, crane.y.velocity),
        'z': LowPassVelocityEstimator(config.velocity_filter_tau, crane.z.position, crane.z.velocity),
    }

    history: list[dict] = []
    arrival_events: list[tuple[float, str]] = []
    locked = {'x': False, 'y': False, 'z': False}
    t = 0.0

    # Initial PD outputs (updated each cycle, or held when localization unchanged)
    vx_cmd = vy_cmd = vz_cmd = 0.0
    vx_raw = vy_raw = vz_raw = 0.0
    vx_filtered = vy_filtered = vz_filtered = 0.0

    # Z-axis height tracking for liftctrl (which takes absolute height, not velocity)
    z_target_height = crane.z.position  # current hoist height as starting point

    # Per-axis last-sent values — only send when change exceeds deadband.
    _last_vx: float | None = None
    _last_vy: float | None = None
    _last_vz_cmd: float | None = None
    _DEADBAND = 0.005  # m/s or m — minimum change to trigger PLC send

    # Localization tracking — /localization_pose is 10 Hz, only update PD on new data
    _last_pose_stamp: float | None = None

    mode_label = 'PLC' if plc is not None else '仿真'
    if verbose:
        print(f"=== 起重机 PD 速度控制{mode_label} ===")
        print(f"初始位置: X={crane.x.position:.2f}, Y={crane.y.position:.2f}, Z={crane.z.position:.2f}")
        print(f"目标位置: X={target_x:.2f}, Y={target_y:.2f}, Z={target_z:.2f}")
        print("=" * 50)

    while not all(locked.values()):
        if max_time is not None and t > max_time:
            raise TimeoutError(f"Simulation did not finish within {max_time:.2f}s (t={t:.2f}s)")

        # --- STEP 1: Measure positions ---
        has_localization = False
        new_pose_arrived = False
        if plc is not None:
            pose = get_latest_pose()
            if pose is not None:
                stamp = pose['stamp_sec'] + pose['stamp_nsec'] * 1e-9
                if stamp != _last_pose_stamp:
                    x_measured = pose['x']
                    y_measured = pose['y']
                    z_measured = pose['z']
                    _last_pose_stamp = stamp
                    has_localization = True
                    new_pose_arrived = True

        if not has_localization:
            x_measured = plant.measure_position('x', crane.x.position)
            y_measured = plant.measure_position('y', crane.y.position)
            z_measured = plant.measure_position('z', crane.z.position)

        # --- STEP 2-3: PD update — only on new data when using localization ---
        if has_localization and not new_pose_arrived:
            # Localization hasn't updated yet — keep previous velocity command
            pass
        else:
            vx_raw, vx_filtered = filters['x'].update(x_measured, dt)
            vy_raw, vy_filtered = filters['y'].update(y_measured, dt)
            vz_raw, vz_filtered = filters['z'].update(z_measured, dt)

            vx_cmd = 0.0 if locked['x'] else controllers['x'].update(target_x, x_measured, vx_filtered)
            vy_cmd = 0.0 if locked['y'] else controllers['y'].update(target_y, y_measured, vy_filtered)
            vz_cmd = 0.0 if locked['z'] else controllers['z'].update(target_z, z_measured, vz_filtered)

        # --- STEP 4: Send PLC commands (if connected) ---
        if plc is not None:
            if _last_vx is None or abs(vx_cmd - _last_vx) > _DEADBAND:
                plc.big_car_ctrl(vx_cmd)
                plc.last_vx = vx_cmd
                _last_vx = vx_cmd
            if _last_vy is None or abs(vy_cmd - _last_vy) > _DEADBAND:
                plc.small_car_ctrl(vy_cmd)
                plc.last_vy = vy_cmd
                _last_vy = vy_cmd
            if not locked['z']:
                z_target_height += vz_cmd * dt
                z_target_height = max(0.0, min(z_target_height, crane.z.position + 1.0))
            if _last_vz_cmd is None or abs(vz_cmd - _last_vz_cmd) > _DEADBAND:
                plc.lift_ctrl(z_target_height)
                plc.last_hz = z_target_height
                plc.last_vz = vz_cmd
                _last_vz_cmd = vz_cmd

        # --- STEP 4b: Update position ---
        if has_localization:
            # Real localization — position comes from /localization_pose
            crane.x.position = x_measured
            crane.y.position = y_measured
            crane.z.position = z_measured
            crane.x.velocity = vx_cmd
            crane.y.velocity = vy_cmd
            crane.z.velocity = vz_cmd
            disturbance_x = disturbance_y = disturbance_z = 0.0
        else:
            # Simulation plant — position from servo model
            disturbance_x = plant.update_axis('x', crane.x, vx_cmd, target_x, locked['x'], dt, t)
            disturbance_y = plant.update_axis('y', crane.y, vy_cmd, target_y, locked['y'], dt, t)
            disturbance_z = plant.update_axis('z', crane.z, vz_cmd, target_z, locked['z'], dt, t)

        # --- STEP 5: Arrival detection ---
        axis_data = [
            ('x', crane.x, target_x, vx_cmd),
            ('y', crane.y, target_y, vy_cmd),
            ('z', crane.z, target_z, vz_cmd),
        ]
        for name, axis, target, cmd in axis_data:
            in_capture_window = abs(axis.position - target) < config.arrival_capture_pos_tol
            command_settled = abs(cmd) < config.arrival_cmd_tol
            if not locked[name] and (_axis_arrived(axis, target, config) or (in_capture_window and command_settled)):
                axis.position = target
                axis.velocity = 0.0
                locked[name] = True
                arrival_events.append((t, name))

        # --- STEP 6: Record history ---
        history.append({
            't': t,
            'x': crane.x.position, 'y': crane.y.position, 'z': crane.z.position,
            'vx': crane.x.velocity, 'vy': crane.y.velocity, 'vz': crane.z.velocity,
            'p_ref_x': target_x, 'p_ref_y': target_y, 'p_ref_z': target_z,
            'v_ref_x': 0.0, 'v_ref_y': 0.0, 'v_ref_z': 0.0,
            'vx_cmd': vx_cmd, 'vy_cmd': vy_cmd, 'vz_cmd': vz_cmd,
            'x_measured': x_measured, 'y_measured': y_measured, 'z_measured': z_measured,
            'vx_raw': vx_raw, 'vy_raw': vy_raw, 'vz_raw': vz_raw,
            'vx_filtered': vx_filtered, 'vy_filtered': vy_filtered, 'vz_filtered': vz_filtered,
            'disturbance_x': disturbance_x, 'disturbance_y': disturbance_y, 'disturbance_z': disturbance_z,
        })
        t += dt

    if verbose:
        final = history[-1]
        print("=" * 50)
        print(f"{mode_label}完成! 总时间: {final['t']:.2f}s")
        print(f"最终位置: X={final['x']:.3f}, Y={final['y']:.3f}, Z={final['z']:.3f}")

    return history, arrival_events


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
    return parser


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
    config = CraneConfig(
        max_velocity_xy=0.3,
        max_velocity_z=0.2,
        kp_pos=0.6,
        kd_pos=0.45,
        dt=0.01,
    )
    crane0 = CraneState(x0=0.0, y0=0.0, z0=5.0)
    target_pos = (args.target_x, args.target_y, args.target_z)

    # Create PLC interface if PLC IP is specified
    plc = None
    if args.plc_ip:
        plc = create_plc(lib_path=args.plc_lib)
        plc.connect(args.plc_ip)
        plc.start_heartbeat()
        start_ros_bridge()
        # Wait for first localization pose (timeout 5s)
        print('Waiting for /localization_pose...')
        waited = 0.0
        while get_latest_pose() is None and waited < 5.0:
            time.sleep(0.1)
            waited += 0.1
        pose = get_latest_pose()
        if pose is not None:
            crane0 = CraneState(x0=pose['x'], y0=pose['y'], z0=pose['z'])
            px, py, pz = pose['x'], pose['y'], pose['z']
            print(f'Localization received: X={px:.2f}, Y={py:.2f}, Z={pz:.2f}')
        else:
            print('Warning: /localization_pose timeout, using default initial position')

    try:
        history, arrival_events = run_simulation(
            target_pos=target_pos,
            initial_state=crane0,
            config=config,
            verbose=True,
            plc=plc,
        )
    finally:
        # Keep PLC alive during live session (buttons need it)
        if plc is not None and not args.live:
            plc.disconnect()
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
        start_ros_bridge()

        _plc = plc  # capture for closure
        _state = {'target': target_pos, 'initial': initial_pos}  # mutable for re-simulation

        def live_payload_for_query(query: dict[str, list[str]]):
            if not query:
                return payload
            # Use last target as new initial position (continuous path)
            start_pos = _state['target']
            requested_target = _target_from_query(query, _state['target'])
            live_history, live_events = run_simulation(
                target_pos=requested_target,
                initial_state=CraneState(x0=start_pos[0], y0=start_pos[1], z0=start_pos[2]),
                config=config,
                verbose=False,
                plc=_plc,
            )
            # Update state for next re-simulation
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
            plc=plc,
        )
        if plc is not None:
            plc.disconnect()


if __name__ == '__main__':
    main()
