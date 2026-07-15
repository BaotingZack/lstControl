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
    - SSE 实时推送状态到浏览器
"""

from __future__ import annotations

# Must be set before any matplotlib import (headless mode, no tkinter)
import matplotlib
matplotlib.use('Agg')

import argparse
import time

from crane_model import (
    CraneConfig,
    CranePlant,
    CraneState,
    PlantActuator,
    SimPositionSource,
    run_pd_control,
)
from live_server import build_live_payload, serve_live_view
from plc_interface import PlcActuator, create_plc
from ros_bridge import RosPositionSource, get_latest_pose, start_ros_bridge
from visualizer import CraneVisualizer


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
    crane0 = CraneState(x0=0.0, y0=0.0, z0=0.0)
    target_pos = (args.target_x, args.target_y, args.target_z)

    # ---- PLC 模式 ----
    if args.plc_ip:
        plc = create_plc(lib_path=args.plc_lib)
        plc.connect(args.plc_ip)
        plc.start_heartbeat()
        start_ros_bridge()

        # 等待首次定位数据 (超时 5s)
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

        initial_pos = (crane0.x.position, crane0.y.position, crane0.z.position)

        if args.live:
            # PLC 实时控制模式: 开 UI, 等用户 Apply Target
            ros_source = RosPositionSource()
            plc_actuator = PlcActuator(plc, initial_z=crane0.z.position)

            print(f'Live view starting — set target in browser and click "Apply Target"')
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
            )
        else:
            # PLC 模式非 live: 使用命令行 target 直接运行 (兼容旧行为)
            ros_source = RosPositionSource()
            plc_actuator = PlcActuator(plc, initial_z=crane0.z.position)
            try:
                history, arrival_events = run_pd_control(
                    source=ros_source,
                    actuator=plc_actuator,
                    config=config,
                    target_pos=target_pos,
                    initial_state=crane0,
                    verbose=True,
                    is_simulation=False,
                )
            finally:
                plc.disconnect()

            viz = CraneVisualizer(config)
            viz.plot(history, arrival_events)
            viz.plot_operation_diagram(
                history=history,
                phase_boundaries=arrival_events,
                target_pos=target_pos,
                initial_pos=initial_pos,
            )
        return

    # ---- 仿真模式 ----
    try:
        history, arrival_events = run_simulation(
            target_pos=target_pos,
            initial_state=crane0,
            config=config,
            verbose=True,
        )
    except Exception:
        raise

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
        # (PLC 模式不需要 ROS, 但 live_view 的 SSE 需要 start_ros_bridge)
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
