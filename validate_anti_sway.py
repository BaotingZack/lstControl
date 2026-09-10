#!/usr/bin/env python3
"""离线验证防摇算法（倾角仪单方案 + bag 回放）。

用已录制的 bag（定位 + 速度 + 倾角仪）离线验证防摇控制算法的有效性。
不需要 IMU——控制律 Δv=+K(L)·θ 只用摆角 θ，倾角仪 roll/pitch 单独即可提供。

验证内容（三层）:
    1. 数据核对      —— 打印 bag 内话题/消息类型，确认可提取的字段。
    2. 模型标定      —— 用录到的行车加速度 ẍ 驱动摆模型，网格搜索 L_eff、ζ，
                        使模型摆角逼近录到的倾角仪摆角（验证摆模型与 L_eff）。
    3. 影子闭环      —— 防摇 Δv=+K·θ 等价于给摆增加阻尼 ζ_sway=K/(2√(gL))，
                        对比"加防摇 vs 不加"的摆角（验证符号与减摆效果）。

用法（需 ROS 1 Noetic 环境，可放入 noetic 容器运行）:
    python validate_anti_sway.py --bag data.bag [选项]

依赖: rosbag（ROS 1 Noetic）、numpy、matplotlib。
"""

from __future__ import annotations

import argparse
import math
import sys

GRAVITY = 9.81


# ---------------------------------------------------------------------------
# bag 读取
# ---------------------------------------------------------------------------

def load_bag(bag_path, loc_topic, inc_topic):
    """读 ROS1 bag（rosbag），提取定位与倾角仪。

    需在 ROS 1 Noetic 环境（或容器）中运行。消息类型为标准 ROS 1 消息：
    nav_msgs/Odometry（定位）、geometry_msgs/Vector3Stamped（倾角仪 roll/pitch）。

    返回 (loc_t, loc_data, inc_t, inc_data)：
        loc_data: {x, y, z, vx, vy: list}；inc_data: {roll, pitch: list}。
    """
    import rosbag  # ROS 1 Noetic

    loc_t, inc_t = [], []
    loc_data = {'x': [], 'y': [], 'z': [], 'vx': [], 'vy': []}
    inc_data = {'roll': [], 'pitch': []}

    with rosbag.Bag(bag_path, 'r') as bag:
        info = bag.get_type_and_topic_info()[1]
        print('[bag] 话题及消息类型:')
        for topic in sorted(info):
            print(f'        {topic}  ({info[topic].msg_type})')

        for topic, msg, t in bag.read_messages(topics=[loc_topic, inc_topic]):
            ts = t.to_sec()
            if topic == loc_topic:
                p = msg.pose.pose.position
                v = msg.twist.twist.linear
                loc_t.append(ts)
                loc_data['x'].append(p.x)
                loc_data['y'].append(p.y)
                loc_data['z'].append(p.z)
                loc_data['vx'].append(v.x)
                loc_data['vy'].append(v.y)
            elif topic == inc_topic:
                inc_t.append(ts)
                inc_data['roll'].append(msg.vector.x)
                inc_data['pitch'].append(msg.vector.y)

    print(f'[bag] 定位 {len(loc_t)} 帧, 倾角仪 {len(inc_t)} 帧')
    if not loc_t or not inc_t:
        sys.exit('[bag] 定位或倾角仪话题无数据，请检查 --loc-topic / --inc-topic')
    return loc_t, loc_data, inc_t, inc_data


# ---------------------------------------------------------------------------
# 时间对齐
# ---------------------------------------------------------------------------

def _resample(src_t, src_v, grid_t):
    """把 (src_t, src_v) 线性插值到 grid_t 上。"""
    import numpy as np
    return np.interp(grid_t, src_t, src_v)


def align(loc_t, loc_data, inc_t, inc_data, dt=None):
    """对齐到统一时间网格，返回 dict of numpy 数组（含 t、x、y、z、vx、vy、roll、pitch）。"""
    import numpy as np

    t0 = max(loc_t[0], inc_t[0])
    t1 = min(loc_t[-1], inc_t[-1])
    if dt is None:
        dt = float(np.median(np.diff(loc_t))) if len(loc_t) > 1 else 0.1
        dt = max(dt, 1e-3)
    grid_t = np.arange(t0, t1, dt)

    out = {'t': grid_t, 'dt': dt}
    for key in ('x', 'y', 'z', 'vx', 'vy'):
        out[key] = _resample(loc_t, loc_data[key], grid_t)
    for key in ('roll', 'pitch'):
        out[key] = _resample(inc_t, inc_data[key], grid_t)
    return out


# ---------------------------------------------------------------------------
# θ 提取、假倾角校正、加速度
# ---------------------------------------------------------------------------

def extract_theta(data, swap, sign_x, sign_y, angle_scale):
    """roll/pitch → θx/θy（轴映射 + 符号 + 单位）。默认 θx←pitch、θy←roll。"""
    roll = data['roll'] * angle_scale
    pitch = data['pitch'] * angle_scale
    if swap:
        theta_x = sign_x * roll
        theta_y = sign_y * pitch
    else:
        theta_x = sign_x * pitch
        theta_y = sign_y * roll
    return theta_x, theta_y


def cart_acceleration(v, dt):
    """由速度差分得到行车加速度 ẍ（中心差分）。"""
    import numpy as np
    return np.gradient(v, dt)


def correct_false_tilt(theta, accel, sign):
    """假倾角校正：θ_corr = θ − sign·ẍ/g（加速度计混入运动加速度的补偿）。"""
    return theta - sign * accel / GRAVITY


# ---------------------------------------------------------------------------
# 影子闭环与模型标定
# ---------------------------------------------------------------------------

def run_pendulum(accel, L, zeta, dt, g=GRAVITY):
    """用行车加速度 ẍ 驱动摆模型，返回摆角数组。"""
    import numpy as np
    from pendulum_model import PendulumAxis

    axis = PendulumAxis(L=L, zeta=zeta, g=g)
    axis.reset(0.0, 0.0)
    theta = np.zeros(len(accel))
    for i, a in enumerate(accel):
        theta[i] = axis.step(a, dt)[0]
    return theta


def shadow_closed_loop(accel, L, zeta, sway_gain, dt, g=GRAVITY):
    """对比加/不加防摇的摆角响应。

    防摇 Δv=+K·θ ⇒ 等效阻尼 ζ_sway = K/(2√(gL))，直接叠加到 ζ 上。
    返回 (theta_without, theta_with)。
    """
    theta_without = run_pendulum(accel, L, zeta, dt, g)
    zeta_sway = sway_gain / (2.0 * math.sqrt(g * L)) if sway_gain > 0 else 0.0
    theta_with = run_pendulum(accel, L, zeta + zeta_sway, dt, g)
    return theta_without, theta_with


def fit_model(accel, theta_ref, L_grid, zeta_grid, dt, g=GRAVITY):
    """网格搜索 (L, ζ) 使模型摆角逼近录到的 θ_ref。返回 (best_L, best_zeta, rms)。"""
    import numpy as np
    best = None
    for L in L_grid:
        for zeta in zeta_grid:
            theta = run_pendulum(accel, L, zeta, dt, g)
            rms = float(np.sqrt(np.mean((theta - theta_ref) ** 2)))
            if best is None or rms < best[2]:
                best = (L, zeta, rms)
    return best


# ---------------------------------------------------------------------------
# 指标
# ---------------------------------------------------------------------------

def swing_metrics(theta):
    """返回 {max_abs, rms}（摆角，单位 deg）。"""
    import numpy as np
    deg = np.degrees(theta)
    return {'max_abs_deg': float(np.max(np.abs(deg))), 'rms_deg': float(np.sqrt(np.mean(deg ** 2)))}


def residual_swing(theta, dt, settle_window):
    """到位后残余摆动: 最后 settle_window 秒内最大 |θ|（deg）。"""
    import numpy as np
    n = max(1, int(round(settle_window / dt)))
    tail = theta[-n:]
    return float(np.max(np.abs(np.degrees(tail))))


def _zeta_sway(sway_gain, L, g=GRAVITY):
    """防摇 Δv=+K·θ 的等效阻尼比 ζ_sway = K/(2√(gL))。"""
    return sway_gain / (2.0 * math.sqrt(g * L)) if sway_gain > 0 else 0.0


def _find_min_k(accel, L, zeta, dt, settle_window, k_max=10.0, k_step=0.05):
    """扫描 K，找到使残余摆动 ≤0.3° 的最小 K。返回 (k, residual) 或 (None, best)。"""
    import numpy as np
    g = GRAVITY
    best = None
    for k in np.arange(k_step, k_max + 1e-9, k_step):
        zeta_sway = _zeta_sway(k, L, g)
        theta_on = run_pendulum(accel, L, zeta + zeta_sway, dt, g)
        residual = residual_swing(theta_on, dt, settle_window)
        if residual <= 0.3:
            return k, residual
        if best is None or residual < best[1]:
            best = (k, residual)
    return None, best[1]


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def _build_parser():
    p = argparse.ArgumentParser(description='离线验证防摇算法（倾角仪单方案 + bag 回放）')
    p.add_argument('--bag', required=True, help='bag 文件路径')
    p.add_argument('--loc-topic', default='/localization_pose', help='定位话题')
    p.add_argument('--inc-topic', default='/inclination/j1939_msg', help='倾角仪话题')
    # θ 提取
    p.add_argument('--axis-swap', action='store_true', help='θx←roll、θy←pitch（默认 θx←pitch、θy←roll）')
    p.add_argument('--sign-x', type=float, default=1.0, help='θx 符号 (±1)')
    p.add_argument('--sign-y', type=float, default=1.0, help='θy 符号 (±1)')
    p.add_argument('--angle-scale', type=float, default=1.0,
                   help='倾角仪单位→弧度换算系数（角度制填 pi/180）')
    p.add_argument('--correct-false-tilt', action='store_true', help='启用假倾角校正')
    p.add_argument('--tilt-corr-sign', type=float, default=1.0, help='假倾角校正符号 (±1)')
    # 绳长 / 防摇增益
    p.add_argument('--rope-length', type=float, default=5.0, help='固定绳长 L [m]（未用 --rope-sheave-height 时）')
    p.add_argument('--rope-sheave-height', type=float, default=None, help='出绳点高度 H_sheave [m]（从 Z 推 L）')
    p.add_argument('--rope-grab-offset', type=float, default=1.5, help='抓钩质心偏移 h_com [m]')
    p.add_argument('--sway-gain', type=float, default=0.5, help='防摇增益 K [m/s per rad]')
    # 模型标定
    p.add_argument('--L-min', type=float, default=2.0)
    p.add_argument('--L-max', type=float, default=8.0)
    p.add_argument('--L-step', type=float, default=0.5)
    p.add_argument('--zeta-max', type=float, default=0.2)
    p.add_argument('--zeta-step', type=float, default=0.02)
    # 输出
    p.add_argument('--plot', default=None, help='摆角对比图输出路径（png）')
    p.add_argument('--dt', type=float, default=None,
                   help='对齐重采样步长 [s]（默认取定位中位间隔；摆动力学只需 10~20Hz，可设 0.05）')
    p.add_argument('--settle-window', type=float, default=10.0,
                   help='到位后残余摆动统计窗长 [s]（末段）')
    p.add_argument('--find-k', action='store_true',
                   help='扫描 K，找到使残余摆动 ≤0.3° 的最小增益')
    return p


def main(argv=None):
    args = _build_parser().parse_args(argv)

    # 1. 读 bag + 对齐
    loc_t, loc_data, inc_t, inc_data = load_bag(args.bag, args.loc_topic, args.inc_topic)
    data = align(loc_t, loc_data, inc_t, inc_data, dt=args.dt)
    print(f'[align] 对齐后 {len(data["t"])} 帧, dt={data["dt"]:.4f}s, 时长 {data["t"][-1]-data["t"][0]:.1f}s')

    # 2. θ 提取 + 假倾角校正
    theta_x, theta_y = extract_theta(data, args.axis_swap, args.sign_x, args.sign_y, args.angle_scale)
    ax = cart_acceleration(data['vx'], data['dt'])
    ay = cart_acceleration(data['vy'], data['dt'])
    if args.correct_false_tilt:
        theta_x = correct_false_tilt(theta_x, ax, args.tilt_corr_sign)
        theta_y = correct_false_tilt(theta_y, ay, args.tilt_corr_sign)
        print('[θ] 已启用假倾角校正 (θ − sign·ẍ/g)')

    # 绳长 L（常数或由 Z 推导）
    if args.rope_sheave_height is not None:
        from sway_controller import RopeLengthModel
        rope = RopeLengthModel(
            sheave_height=args.rope_sheave_height,
            grab_offset=args.rope_grab_offset,
            min_length=0.5,
        )
        L_arr = rope.compute(data['z'])  # numpy 广播
        L_use = float(L_arr.mean())
        print(f'[绳长] 由 Z 推导, 均值 L_eff={L_use:.2f}m')
    else:
        L_use = args.rope_length
        print(f'[绳长] 固定 L={L_use:.2f}m')

    # 3. 模型标定（用 θx 与 X 轴加速度）
    import numpy as np
    L_grid = np.arange(args.L_min, args.L_max + 1e-9, args.L_step)
    zeta_grid = np.arange(0.0, args.zeta_max + 1e-9, args.zeta_step)
    best_L, best_zeta, best_rms = fit_model(ax, theta_x, L_grid, zeta_grid, data['dt'])
    print(f'[标定] 最佳 L_eff={best_L:.2f}m, ζ={best_zeta:.3f}, RMS={best_rms:.4f}rad '
          f'({np.degrees(best_rms):.2f}°)')

    # 4. 影子闭环：加/不加防摇
    theta_off, theta_on = shadow_closed_loop(
        ax, best_L, best_zeta, args.sway_gain, data['dt']
    )
    m_off = swing_metrics(theta_off)
    m_on = swing_metrics(theta_on)
    r_off = residual_swing(theta_off, data['dt'], args.settle_window)
    r_on = residual_swing(theta_on, data['dt'], args.settle_window)
    print(f'[影子闭环] 不加防摇: 最大摆角 {m_off["max_abs_deg"]:.2f}°, '
          f'残余摆动(末{args.settle_window:.0f}s) {r_off:.2f}°')
    print(f'[影子闭环] 加防摇 K={args.sway_gain}: 最大摆角 {m_on["max_abs_deg"]:.2f}°, '
          f'残余摆动 {r_on:.2f}°')
    red = (m_off['max_abs_deg'] - m_on['max_abs_deg']) / m_off['max_abs_deg'] * 100.0 if m_off['max_abs_deg'] > 1e-9 else 0.0
    print(f'[影子闭环] 最大摆角降低 {red:.1f}%')

    # 4b. 残余摆动达标检查 (≤0.3°) + 可选 K 扫描
    if r_on <= 0.3:
        print(f'[验收] ✅ 加防摇后残余摆动 {r_on:.2f}° ≤ 0.3° 达标')
    else:
        print(f'[验收] ⚠️ 加防摇后残余摆动 {r_on:.2f}° > 0.3°，需增大 K')
        if args.find_k:
            k, r = _find_min_k(ax, best_L, best_zeta, data['dt'], args.settle_window)
            if k is not None:
                print(f'[验收] 使残余摆动 ≤0.3° 的最小 K = {k:.2f}（此时残余 {r:.2f}°）')
            else:
                print(f'[验收] 扫描范围内未找到达标 K（最小残余 {r:.2f}°）')

    # 5. 图
    if args.plot:
        _plot(data, theta_x, theta_off, theta_on, args.plot)

    return 0


def _plot(data, theta_x, theta_off, theta_on, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    t = data['t']
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(t, np.degrees(theta_x), label='θx 录到(倾角仪)', alpha=0.8)
    axes[0].plot(t, data['vx'], label='vx [m/s]', alpha=0.6)
    axes[0].set_ylabel('θx [°] / vx')
    axes[0].legend()
    axes[0].set_title('录到的摆角与行车速度')

    axes[1].plot(t, np.degrees(theta_off), label='不加防摇', alpha=0.8)
    axes[1].plot(t, np.degrees(theta_on), label='加防摇', alpha=0.8)
    axes[1].set_ylabel('θ [°]')
    axes[1].set_xlabel('t [s]')
    axes[1].legend()
    axes[1].set_title('影子闭环：加/不加防摇的摆角对比')

    fig.tight_layout()
    fig.savefig(path, dpi=100)
    print(f'[plot] 已保存 {path}')


if __name__ == '__main__':
    sys.exit(main())
