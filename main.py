"""
桥式起重机位置目标控制仿真。

调度系统只发送一个目标点 (target_x, target_y, target_z)。
PLC 每个周期读取位置反馈，估计并滤波速度，使用 PD 输出速度指令:

  v_cmd = Kp * (target_position - measured_position) - Kd * filtered_velocity

被控对象模拟速度模式伺服的一阶响应，并叠加低频扰动和测量噪声。
"""

import argparse

from crane_model import CraneConfig, CranePlant, CraneState
from live_server import build_live_payload, serve_live_view
from pd_controller import PositionPDController
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
) -> tuple[list[dict], list[tuple[float, str]]]:
    """Run one dispatch target command until all three axes arrive."""
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

    if verbose:
        print("=== 起重机 PD 速度控制仿真 ===")
        print(f"初始位置: X={crane.x.position:.2f}, Y={crane.y.position:.2f}, Z={crane.z.position:.2f}")
        print(f"目标位置: X={target_x:.2f}, Y={target_y:.2f}, Z={target_z:.2f}")
        print("=" * 50)

    while not all(locked.values()):
        if max_time is not None and t > max_time:
            raise TimeoutError(f"Simulation did not finish within {max_time:.2f}s (t={t:.2f}s)")

        x_measured = plant.measure_position('x', crane.x.position)
        y_measured = plant.measure_position('y', crane.y.position)
        z_measured = plant.measure_position('z', crane.z.position)

        vx_raw, vx_filtered = filters['x'].update(x_measured, dt)
        vy_raw, vy_filtered = filters['y'].update(y_measured, dt)
        vz_raw, vz_filtered = filters['z'].update(z_measured, dt)

        vx_cmd = 0.0 if locked['x'] else controllers['x'].update(target_x, x_measured, vx_filtered)
        vy_cmd = 0.0 if locked['y'] else controllers['y'].update(target_y, y_measured, vy_filtered)
        vz_cmd = 0.0 if locked['z'] else controllers['z'].update(target_z, z_measured, vz_filtered)

        disturbance_x = plant.update_axis('x', crane.x, vx_cmd, target_x, locked['x'], dt, t)
        disturbance_y = plant.update_axis('y', crane.y, vy_cmd, target_y, locked['y'], dt, t)
        disturbance_z = plant.update_axis('z', crane.z, vz_cmd, target_z, locked['z'], dt, t)

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
        print(f"仿真完成! 总时间: {final['t']:.2f}s")
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
        def live_payload_for_query(query: dict[str, list[str]]):
            if not query:
                return payload
            requested_target = _target_from_query(query, target_pos)
            live_history, live_events = run_simulation(
                target_pos=requested_target,
                initial_state=CraneState(x0=initial_pos[0], y0=initial_pos[1], z0=initial_pos[2]),
                config=config,
                verbose=False,
            )
            return build_live_payload(
                history=live_history,
                phase_boundaries=live_events,
                target_pos=requested_target,
                initial_pos=initial_pos,
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
        )


if __name__ == '__main__':
    main()
