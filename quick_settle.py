#!/usr/bin/env python3
"""基于倾角仪的四象限超前短平快独立消摆脚本 (无需 SLAM 定位，直连倾角仪 + PLC)。

核心原理（四象限加减速消摆 + 驱动斜坡超前补偿）
===============================================
设 θ 为钩子相对原点的前向摆角（向前摆 θ > 0，向后摆 θ < 0），ω = dθ/dt 为摆角速度：

  - 象限 1 (0 → 前向最高点, θ > 0, ω > 0)：向前加速 (车速向前 v* > 0，加速度向前 a* > 0)
  - 象限 2 (前向最高点 → 0, θ > 0, ω < 0)：向前减速 (车速向前 v* > 0，加速度向后 a* < 0)
  - 象限 3 (0 → 后向最高点, θ < 0, ω < 0)：向后加速 (车速向后 v* < 0，加速度向后 a* < 0)
  - 象限 4 (后向最高点 → 0, θ < 0, ω > 0)：向后减速 (车速向后 v* < 0，加速度向前 a* > 0)

令期望速度：
    v*(t) = Kv · θ(t)
对其求导即得期望加速度：
    a*(t) = dv*/dt = Kv · ω(t)
这一个简洁关系在全部 4 个象限中严丝合缝地给出你要求的速度方向与加减速方向！

针对“只能发速度指令、且速度指令有一段加速过程”的补偿办法
========================================================
PLC 速度伺服存在加速斜坡惯性：a_actual ≈ (v_cmd - v_actual) / τ_ramp。
若直接发 v_cmd = v*，车速滞后会导致过零点来不及加速、最高点来不及减速。
本脚本采用【斜坡逆模型超前给定 + 内部速度观测器】：

    v_cmd = v* + τ_lead · a* + k_track · (v* - v_hat) - k_recenter · x_hat

  1. 在 0 → 最高点加速段 (Q1/Q3)：a* 与 v* 同号，τ_lead·a* 把速度指令提前放大（超调给定），
     逼迫驱动器立刻拉起加速度追赶钩子；
  2. 在最高点 → 0 回落段 (Q2/Q4)：钩子刚过最高点，ω 变号，τ_lead·a* 立刻把 v_cmd 提前往下拉，
     抵消驱动器的减速斜坡滞后，使真实车速平滑减速并在钩子回原点时刚好归零，不会过冲或反推！

运行示例
========
1. 本地仿真自检（观察四象限切换、速度指令超前与消摆收敛，无需 PLC/ROS）：
     python3 quick_settle.py --sim

2. 现场真机一键短平快消摆（直读 ROS 倾角仪 + 直发 PLC 速度，完全不依赖 SLAM 定位）：
     python3 quick_settle.py --plc-ip 192.168.0.1

3. 常用现场调参（若某轴方向反了，把对应 --sign-x 或 --sign-y 改成 +1 或 -1 即可）：
     python3 quick_settle.py --kv 2.2 --tau-lead 0.45 --max-vel 0.30 --sign-x -1 --sign-y -1
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time

from anti_sway.quad_settle import QUADRANT_LABELS, QuadLeadAntiSwayController
from plc_interface import create_plc, ensure_plc_library_path
from site_config import DEFAULT_SITE_CONFIG_PATH, build_sway_axis_mapping, load_site_config


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description='基于倾角仪的四象限超前短平快消摆脚本 (支持 --sim 仿真与真机直控)'
    )
    p.add_argument('--sim', action='store_true', help='运行内置物理摆 + 带加速斜坡驱动器的离线仿真')
    p.add_argument('--plc-ip', default='192.168.0.1', help='PLC IP 地址 (默认 192.168.0.1)')
    p.add_argument('--plc-lib', default='plc_lib/lib/libsscarctrl.so', help='PLC 动态库路径')
    p.add_argument('--allow-mock-plc', action='store_true', help='允许无硬件时使用 MockPLC')
    p.add_argument('--config', default=str(DEFAULT_SITE_CONFIG_PATH), help='site_config.json 路径')

    # 核心控制参数（简洁易调）
    p.add_argument('--kv', type=float, default=None, help='四象限速度增益 Kv [m/s/rad] (默认读取配置或 2.2)')
    p.add_argument(
        '--tau-lead',
        type=float,
        default=None,
        help='速度斜坡超前补偿时间常数 [s]：驱动加速越慢该值越大 (默认 0.45s)',
    )
    p.add_argument('--max-vel', type=float, default=None, help='最大输出速度限幅 [m/s] (默认 0.30)')
    p.add_argument('--drive-accel', type=float, default=None, help='估计驱动器最大加速度 [m/s²] (默认 0.20)')
    p.add_argument('--deadband-deg', type=float, default=None, help='静止死区角度 [deg] (默认 0.10°)')
    p.add_argument('--recenter-gain', type=float, default=None, help='内部位移漂移柔顺回中增益 (默认 0.12, 0=关闭)')
    p.add_argument(
        '--sign-x',
        type=float,
        default=None,
        help='X 大车极性 (+1 或 -1，默认真机 -1 / 仿真 +1；若消摆反推改号即可)',
    )
    p.add_argument(
        '--sign-y',
        type=float,
        default=None,
        help='Y 小车极性 (+1 或 -1，默认真机 -1 / 仿真 +1；若消摆反推改号即可)',
    )
    p.add_argument('--axis', choices=('xy', 'x', 'y'), default='xy', help='生效轴选择：xy / 仅 x / 仅 y')

    # 运行循环与判稳参数
    p.add_argument('--hz', type=float, default=30.0, help='控制循环频率 Hz (默认 30Hz，比 SLAM 10Hz 更快响应)')
    p.add_argument('--tol-deg', type=float, default=0.25, help='自动判稳完成角度阈值 [deg] (默认 0.25°)')
    p.add_argument('--hold-sec', type=float, default=2.5, help='角度连续小于阈值多久后判定消摆完成 [s] (默认 2.5s；0=一直运行直到 Ctrl+C)')
    p.add_argument('--max-time', type=float, default=90.0, help='最大运行时间 [s] (默认 90s，0=不限时)')
    p.add_argument('--sim-init-x-deg', type=float, default=6.0, help='仿真初始 X 摆角 [deg]')
    p.add_argument('--sim-init-y-deg', type=float, default=3.5, help='仿真初始 Y 摆角 [deg]')
    p.add_argument('--sim-ramp-tau', type=float, default=0.45, help='仿真被控行车的速度加速滞后时间常数 [s]')
    return p


def _resolve_params(args, site: dict, *, is_sim: bool) -> dict:
    def pick(cli_val, site_key: str, default: float) -> float:
        if cli_val is not None:
            return float(cli_val)
        return float(site.get(site_key, default))

    default_sign = 1.0 if is_sim else float(site.get('anti_sway_quad_sign_x', -1.0))
    default_sign_y = 1.0 if is_sim else float(site.get('anti_sway_quad_sign_y', -1.0))

    # PLC 轴反转配置支持
    plc_big_sign = -1.0 if bool(site.get('plc_invert_big_car', False)) else 1.0
    plc_small_sign = -1.0 if bool(site.get('plc_invert_small_car', False)) else 1.0

    return {
        'kv': pick(args.kv, 'anti_sway_quad_kv', 2.2),
        'tau_lead': pick(args.tau_lead, 'anti_sway_quad_tau_lead', 0.45),
        'max_vel': pick(args.max_vel, 'anti_sway_quad_max_correction', 0.30),
        'drive_accel': pick(args.drive_accel, 'anti_sway_quad_drive_accel', 0.20),
        'deadband_deg': pick(args.deadband_deg, 'anti_sway_quad_deadband_deg', 0.10),
        'omega_tau': float(site.get('anti_sway_quad_omega_tau', 0.06)),
        'dc_tau': float(site.get('anti_sway_quad_dc_tau', 5.0)),
        'recenter_gain': pick(args.recenter_gain, 'anti_sway_quad_recenter_gain', 0.12),
        'sign_x': float(args.sign_x) if args.sign_x is not None else default_sign,
        'sign_y': float(args.sign_y) if args.sign_y is not None else default_sign_y,
        'plc_big_sign': plc_big_sign,
        'plc_small_sign': plc_small_sign,
    }


def _build_controller(params: dict) -> QuadLeadAntiSwayController:
    return QuadLeadAntiSwayController(
        kv=params['kv'],
        tau_lead=params['tau_lead'],
        max_vel=params['max_vel'],
        drive_accel=params['drive_accel'],
        deadband_rad=math.radians(params['deadband_deg']),
        omega_tau=params['omega_tau'],
        dc_tau=params['dc_tau'],
        recenter_gain=params['recenter_gain'],
        sign_x=params['sign_x'],
        sign_y=params['sign_y'],
    )


def run_offline_sim(args, site: dict) -> int:
    """离线物理摆仿真：验证有加速斜坡限制的速度伺服下的四象限消摆效果。"""
    params = _resolve_params(args, site, is_sim=True)
    ctrl = _build_controller(params)

    g = 9.81
    L = 3.5  # 有效摆长 [m]
    dt_sim = 0.01
    ctrl_every = max(1, int(round(1.0 / (max(5.0, args.hz) * dt_sim))))
    dt_ctrl = ctrl_every * dt_sim

    theta_x = math.radians(args.sim_init_x_deg)
    theta_y = math.radians(args.sim_init_y_deg)
    omega_x = omega_y = 0.0
    vx_actual = vy_actual = 0.0
    x_pos = y_pos = 0.0
    vx_cmd = vy_cmd = 0.0

    tau_ramp = max(0.05, float(args.sim_ramp_tau))
    a_max_phys = params['drive_accel']
    v_max_phys = params['max_vel']
    tol_rad = math.radians(args.tol_deg)

    print('=== 四象限超前消摆 · 离线物理仿真 ===')
    print(
        f"初始摆角: θx={args.sim_init_x_deg:+.2f}°, θy={args.sim_init_y_deg:+.2f}° | "
        f"行车加速惯性 τ_ramp={tau_ramp:.2f}s, a_max={a_max_phys:.2f}m/s²"
    )
    print(
        f"控制参数: Kv={params['kv']}, τ_lead={params['tau_lead']}s, "
        f"max_vel={params['max_vel']}m/s, 频率={1.0/dt_ctrl:.0f}Hz"
    )
    print('-' * 88)

    hold_t = 0.0
    max_steps = int((args.max_time if args.max_time > 0 else 40.0) / dt_sim)
    last_print_t = -1.0

    for step in range(max_steps):
        t = step * dt_sim
        if step % ctrl_every == 0:
            vx_cmd, vy_cmd = ctrl.compute(theta_x, theta_y, dt_ctrl)
            if args.axis == 'x':
                vy_cmd = 0.0
            elif args.axis == 'y':
                vx_cmd = 0.0

        # 模拟行车速度伺服的加速过程（一阶滞后 + 最大加速度斜坡限制）：不会瞬时到达 v_cmd！
        ax = max(-a_max_phys, min(a_max_phys, (vx_cmd - vx_actual) / tau_ramp))
        ay = max(-a_max_phys, min(a_max_phys, (vy_cmd - vy_actual) / tau_ramp))
        vx_actual = max(-v_max_phys, min(v_max_phys, vx_actual + ax * dt_sim))
        vy_actual = max(-v_max_phys, min(v_max_phys, vy_actual + ay * dt_sim))
        x_pos += vx_actual * dt_sim
        y_pos += vy_actual * dt_sim

        # 单摆动力学 (钩子前摆 θ>0 时，车向前加速 ax>0 产生 -ax/L 角加速度抑制前摆)
        wn = math.sqrt(g / L)
        alpha_x = -(g / L) * theta_x - ax / L - 2.0 * 0.004 * wn * omega_x
        alpha_y = -(g / L) * theta_y - ay / L - 2.0 * 0.004 * wn * omega_y
        omega_x += alpha_x * dt_sim
        omega_y += alpha_y * dt_sim
        theta_x += omega_x * dt_sim
        theta_y += omega_y * dt_sim

        peak_rad = max(abs(theta_x), abs(theta_y))
        if peak_rad < tol_rad and max(abs(omega_x), abs(omega_y)) < tol_rad * 1.5:
            hold_t += dt_sim
        else:
            hold_t = 0.0

        if t - last_print_t >= 0.4 - 1e-9:
            last_print_t = t
            snap = ctrl.debug_snapshot()
            qx = QUADRANT_LABELS[snap['quad_x']]
            print(
                f"t={t:5.2f}s | θx={math.degrees(theta_x):+6.2f}° θy={math.degrees(theta_y):+6.2f}° | "
                f"{qx:13s} | v*={snap['v_star_x']:+6.3f} -> v_cmd={vx_cmd:+6.3f} "
                f"(实测车速 v={vx_actual:+6.3f}, a={ax:+6.3f}) | 位移=({x_pos:+.3f}, {y_pos:+.3f})m"
            )

        if args.hold_sec > 0 and hold_t >= args.hold_sec:
            print('-' * 88)
            print(
                f"[消摆完成] 用时 {t:.2f}s (含 {args.hold_sec:.1f}s 稳态保持) | "
                f"残余摆角 θ=({math.degrees(theta_x):+.3f}°, {math.degrees(theta_y):+.3f}°) | "
                f"最终位置偏移=({x_pos:+.3f}, {y_pos:+.3f})m"
            )
            return 0

    print('-' * 88)
    print(
        f"[仿真结束] t={max_steps * dt_sim:.2f}s | "
        f"残余摆角 θ=({math.degrees(theta_x):+.3f}°, {math.degrees(theta_y):+.3f}°)"
    )
    return 0


def run_live_plc(args, site: dict) -> int:
    """现场真机短平快消摆：直接读取 ROS 倾角仪并以高频发速度给 PLC（不依赖 SLAM 定位）。"""
    from ros_bridge import get_sway_state, start_ros_bridge

    params = _resolve_params(args, site, is_sim=False)
    ctrl = _build_controller(params)

    # 1. 启动 ROS 倾角仪订阅
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

    # 2. 连接 PLC 并启动心跳
    plc = create_plc(lib_path=args.plc_lib, allow_mock=args.allow_mock_plc)
    ret = plc.connect(args.plc_ip)
    if ret != 0:
        plc.disconnect()
        raise RuntimeError(f'连接 PLC ({args.plc_ip}) 失败, ret={ret}')
    plc.start_heartbeat()

    stop_requested = False

    def _on_signal(_sig, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        print('正在等待 PLC 心跳与倾角仪 (/inclination/j1939_msg) 数据...')
        wait_deadline = time.monotonic() + 6.0
        while time.monotonic() < wait_deadline and not stop_requested:
            if plc.heartbeat_healthy and get_sway_state() is not None:
                break
            time.sleep(0.05)

        if not plc.heartbeat_healthy:
            raise RuntimeError('PLC 心跳未就绪，已取消启动')
        if get_sway_state() is None:
            raise RuntimeError('未收到倾角仪数据 (/inclination/j1939_msg)，已取消启动')

        # 锁定当前起升高度，消摆期间 Z 轴不动
        hold_z = plc.get_lift_height()
        if hold_z is not None and math.isfinite(hold_z):
            hold_z = max(0.35, float(hold_z))
        else:
            hold_z = None

        print('=' * 88)
        print('=== 四象限超前短平快消摆 · 真机运行中 (按 Ctrl+C 随时急停归零) ===')
        print(
            f"参数: Kv={params['kv']}, τ_lead={params['tau_lead']}s, max_vel={params['max_vel']}m/s, "
            f"sign=({params['sign_x']:+.0f}, {params['sign_y']:+.0f}), 轴={args.axis}, 频率={args.hz:.0f}Hz"
        )
        print('=' * 88)

        period = 1.0 / max(5.0, float(args.hz))
        tol_rad = math.radians(args.tol_deg)
        t0 = time.monotonic()
        last_t = t0
        last_print = -1.0
        hold_t = 0.0
        stale_streak = 0

        while not stop_requested:
            now = time.monotonic()
            dt = max(1e-3, now - last_t)
            last_t = now
            elapsed = now - t0

            if args.max_time > 0 and elapsed > args.max_time:
                print(f'\n[超时退出] 已运行 {elapsed:.1f}s，正在归零停车...')
                break

            if not plc.check_connection() or not plc.heartbeat_healthy:
                raise RuntimeError('PLC 连接或心跳中断，触发保护停车')

            sway = get_sway_state()
            if sway is None:
                stale_streak += 1
                plc.big_car_ctrl(0.0)
                plc.small_car_ctrl(0.0)
                if stale_streak > int(max(5.0, args.hz) * 1.0):
                    raise RuntimeError('倾角仪数据超时断流 (>1s)，已归零停车')
                time.sleep(period)
                continue
            stale_streak = 0

            theta_x = float(sway['theta_x'])
            theta_y = float(sway['theta_y'])

            vx_cmd, vy_cmd = ctrl.compute(theta_x, theta_y, dt)
            if args.axis == 'x':
                vy_cmd = 0.0
            elif args.axis == 'y':
                vx_cmd = 0.0

            vx_out = params['plc_big_sign'] * vx_cmd
            vy_out = params['plc_small_sign'] * vy_cmd

            plc.big_car_ctrl(vx_out)
            plc.small_car_ctrl(vy_out)
            if hold_z is not None:
                plc.lift_ctrl(hold_z)

            snap = ctrl.debug_snapshot()
            peak_ac_rad = math.radians(
                max(abs(snap['theta_ac_x_deg']), abs(snap['theta_ac_y_deg']))
            )
            omega_peak_rad = math.radians(
                max(abs(snap['omega_x_deg_s']), abs(snap['omega_y_deg_s']))
            )
            if peak_ac_rad < tol_rad and omega_peak_rad < tol_rad * 1.8:
                hold_t += dt
            else:
                hold_t = 0.0

            if elapsed - last_print >= 0.25:
                last_print = elapsed
                qx = QUADRANT_LABELS[snap['quad_x']]
                qy = QUADRANT_LABELS[snap['quad_y']]
                print(
                    f"t={elapsed:5.1f}s | θac=({snap['theta_ac_x_deg']:+5.2f}°, {snap['theta_ac_y_deg']:+5.2f}°) | "
                    f"X:{qx:13s} Y:{qy:13s} | v_cmd=({vx_out:+.3f}, {vy_out:+.3f}) m/s"
                )

            if args.hold_sec > 0 and hold_t >= args.hold_sec:
                print(
                    f"\n[消摆完成] 用时 {elapsed:.2f}s | "
                    f"残余摆角=({snap['theta_ac_x_deg']:+.2f}°, {snap['theta_ac_y_deg']:+.2f}°)"
                )
                break

            sleep_s = period - (time.monotonic() - now)
            if sleep_s > 0:
                time.sleep(sleep_s)

        return 0
    finally:
        # 无论正常完成、Ctrl+C 还是异常退出，确保大车小车速度指令立即归零
        try:
            for _ in range(3):
                plc.big_car_ctrl(0.0)
                plc.small_car_ctrl(0.0)
                time.sleep(0.05)
            print('[安全停车] 大车/小车速度指令已归零。')
        finally:
            plc.disconnect()


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)
    site = load_site_config(args.config)
    if args.sim:
        return run_offline_sim(args, site)
    ensure_plc_library_path()
    return run_live_plc(args, site)


if __name__ == '__main__':
    sys.exit(main())
