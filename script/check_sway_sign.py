#!/usr/bin/env python3
"""现场只读诊断 — 倾角仪轴映射/符号核验 + 自然摆动观测。

本脚本**只订阅 ROS, 不连 PLC, 不下发任何指令**, 可在行车通电状态下安全运行。

为什么必须先做符号核验
---------------------
防摇控制律是 Δv = +K·θ (小车追着载荷走), 符号即阻尼的符号:
  θx > 0 必须表示"载荷相对吊点朝 +X 偏移"(X = 大车/定位坐标系 X 正方向)。
若 anti_sway_theta_x_from / _sign 等标定反了, 同一套公式就变成负阻尼,
会把摆越推越大。调度器内置发散保护, 但符号必须先在静态下确认, 不要拿闭环去试。

用法
----
1) 符号核验 (人工推钩, 看读数方向):
       python3 script/check_sway_sign.py
   把空抓钩降到人能安全触及的高度, 停车断开运动, 用手把抓钩**朝 +X 方向**
   推开并保持 → θx 应显示为**正**, 且 θy 基本不变。再朝 +Y 推 → θy 为正。
   任一方向读数反号, 就翻转 site_config.json 里对应的
   anti_sway_theta_x_sign / anti_sway_theta_y_sign;
   若"推 X 却是 θy 在动", 则是 anti_sway_theta_x_from / _y_from 写反了。

2) 零偏核验 (各高度/张合状态静态读数):
       python3 script/check_sway_sign.py --settle 10
   抓钩静止悬停 10s, 报告均值 (应接近 0, 即零偏已扣净) 与去趋势峰峰值
   (测量噪声地板)。在 approach/transport/取放货三个高度、抓钩张开与夹紧
   各做一次; 均值若显著偏离 0, 把它加到 site_config 的对应 bias 上。

3) 自然摆动观测 (标定绳长模型 + 量自然阻尼):
       python3 script/check_sway_sign.py --settle 40
   手动激励一次摆动后静止观测 40s, 报告实测摆周期与包络衰减时间常数。
   实测周期应与模型周期接近 (绳长模型正确); 自然衰减时间常数就是"不做防摇
   要等多久"的现场实证。
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from site_config import (
    DEFAULT_SITE_CONFIG_PATH,
    build_sway_axis_mapping,
    load_site_config,
)
from sway_settle import (
    estimate_period_zero_cross,
    pendulum_period,
    sway_linear_detrend_peak_to_peak,
)


def _start_bridge(site: dict, config_path) -> None:
    from ros_bridge import start_ros_bridge

    angle_scale = math.pi / 180.0 if site.get('anti_sway_angle_deg') else 1.0
    if angle_scale == 1.0:
        print(
            '[警告] site_config 未设 anti_sway_angle_deg — 若倾角仪输出为度制, '
            '读数会被当成弧度 (差 57 倍)'
        )
    start_ros_bridge(
        angle_scale=angle_scale,
        roll_bias=float(site.get('anti_sway_roll_bias', 0.0)),
        pitch_bias=float(site.get('anti_sway_pitch_bias', 0.0)),
        axis_mapping=build_sway_axis_mapping(site),
        roll_bias_closed=site.get('anti_sway_roll_bias_closed'),
        pitch_bias_closed=site.get('anti_sway_pitch_bias_closed'),
    )
    print(f'[配置] {config_path}')
    print(
        f"[配置] θx←{site.get('anti_sway_theta_x_from', 'pitch')}"
        f"×{site.get('anti_sway_theta_x_sign', 1.0):+g}, "
        f"θy←{site.get('anti_sway_theta_y_from', 'roll')}"
        f"×{site.get('anti_sway_theta_y_sign', 1.0):+g}"
    )


def _wait_for_sensor(timeout: float = 5.0) -> bool:
    from ros_bridge import get_inclination_raw

    print(f'等待 /inclination/j1939_msg (最多 {timeout:.0f}s)...')
    waited = 0.0
    while get_inclination_raw() is None and waited < timeout:
        time.sleep(0.1)
        waited += 0.1
    if get_inclination_raw() is None:
        print('[错误] 未收到倾角仪数据 — 检查倾角仪供电/CAN 网关/ROS 话题')
        return False
    return True


def _axis_hint(theta_deg: float, axis: str, positive: str) -> str:
    """把摆角读数翻译成"防摇会往哪边追", 方便和眼睛看到的方向对照。"""
    if abs(theta_deg) < 0.05:
        return f'{axis}: 居中'
    direction = f'+{positive}' if theta_deg > 0 else f'-{positive}'
    return f'{axis}: 载荷偏向 {direction} → 防摇会朝 {direction} 追'


def run_live(hz: float) -> int:
    from ros_bridge import get_sway_snapshot

    print()
    print('实时读数 (Ctrl-C 退出)。用手把空抓钩朝已知方向推开, 核对符号:')
    print('  朝 +X (大车正方向) 推 → θx 应为正;  朝 +Y (小车正方向) 推 → θy 应为正')
    print()
    print('bag 数据分析 (lst_001~005) 给出的预期, 请重点核对:')
    print('  · 大车 X 方向的摆由 vec.y 承载 (= 配置里的 pitch) — 轴归属证据较强')
    print('  · 朝 +X 推时 vec.y 预期读到 **负** 值 ⇒ anti_sway_theta_x_sign 应为 -1')
    print('    (当前配置是 +1。若现场证实为负, 必须改, 否则防摇会把摆推大)')
    print('  · 小车 Y 方向的符号 bag 里激励太弱, 无结论 — 只能靠这次静态试验定')
    print('-' * 78)
    period = 1.0 / max(hz, 0.5)
    # 静止基准 + 活性统计: 倾角仪若被网关冻结, 原始值会逐位不变, 从
    # "变化数/总数" 一眼就能看出来 —— 这比盯绝对值可靠得多。
    base: tuple[float, float] | None = None
    seen: set[tuple[float, float]] = set()
    n = 0
    dx_max = dy_max = 0.0
    try:
        while True:
            snap = get_sway_snapshot()
            if not snap.get('valid'):
                print(f"\r[断流] age={snap.get('age_ms', '?')}ms" + ' ' * 60, end='')
            else:
                tx = snap['theta_x_deg']
                ty = snap['theta_y_deg']
                raw = (snap['roll'], snap['pitch'])
                n += 1
                seen.add(raw)
                if base is None:
                    base = (tx, ty)
                dx, dy = tx - base[0], ty - base[1]
                dx_max = max(dx_max, abs(dx))
                dy_max = max(dy_max, abs(dy))
                # 原始通道按 vector.x / vector.y 显示: "roll/pitch" 的命名约定
                # (绕轴转 vs 沿轴倾) 有两种互相矛盾的解释, 是轴映射搞错的根源
                alive = f'{len(seen)}/{n}'
                print(
                    f"\rvec.x={raw[0]:+7.3f} vec.y={raw[1]:+7.3f} | "
                    f"θx={tx:+6.3f}° θy={ty:+6.3f}° | "
                    f"Δθx={dx:+6.3f}° Δθy={dy:+6.3f}° "
                    f"(峰 {dx_max:.3f}/{dy_max:.3f}) | "
                    f"变化 {alive} age={snap.get('age_ms', '?')}ms"
                    + ' ' * 6,
                    end='',
                )
            sys.stdout.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        print()
        print()
        if n and len(seen) <= max(2, n // 20):
            print(f'[警告] {n} 次采样里原始值只出现 {len(seen)} 种 — 倾角仪读数疑似冻结')
            print('       (网关重复上一帧 / 传感器未真正更新)。先解决数据活性, 再谈符号。')
        elif base is not None:
            print(f'本次 Δθ 峰值: θx {dx_max:.3f}°   θy {dy_max:.3f}°')
            if max(dx_max, dy_max) < 0.1:
                print('[提示] Δθ 峰值 <0.1° — 没有检测到明显推钩动作, 符号未验证。')
        return 0


def run_settle(duration: float, hz: float, z_grab: float, site: dict) -> int:
    """静止观测: 报告均值 (零偏残余)、噪声地板、实测摆周期与自然衰减。"""
    from ros_bridge import get_sway_state

    print()
    print(f'静止观测 {duration:.0f}s — 观测期间请勿操作行车...')
    samples: list[tuple[float, float, float]] = []
    t0 = time.monotonic()
    period = 1.0 / max(hz, 1.0)
    while time.monotonic() - t0 < duration:
        sway = get_sway_state()
        if sway is not None:
            samples.append(
                (time.monotonic(), sway['theta_x'], sway['theta_y'])
            )
        time.sleep(period)

    if len(samples) < 32:
        print(f'[错误] 有效样本太少 ({len(samples)}) — 倾角仪可能断流')
        return 1

    # 活性检查: 冻结的读数会让下面所有统计 (零偏/PP/周期) 都变成假结论
    distinct = len({(s[1], s[2]) for s in samples})
    if distinct <= max(2, len(samples) // 20):
        print(f'[错误] {len(samples)} 次采样里只有 {distinct} 种不同读数 — 倾角仪疑似冻结。')
        print('       网关在重复上一帧, 或传感器未真正更新。此时零偏/摆幅统计全部无效。')
        return 1

    xs_deg = [math.degrees(s[1]) for s in samples]
    ys_deg = [math.degrees(s[2]) for s in samples]
    det = sway_linear_detrend_peak_to_peak(samples)
    T_meas = estimate_period_zero_cross(samples)
    T_model = pendulum_period(
        z_grab,
        sheave_height=float(site.get('rope_length_sheave_height', 4.0)),
        grab_offset=float(site.get('rope_length_grab_offset', 0.0)),
    )
    tol = float(site.get('hook_sway_angle_tol_deg', 0.3))

    print('-' * 78)
    print(f'样本 {len(samples)} 个, 跨度 {samples[-1][0] - samples[0][0]:.1f}s')
    print()
    print('零偏残余 (静止时应接近 0; 显著偏离就把它加到 site_config 的 bias 上):')
    print(f'  θx 均值 = {statistics.fmean(xs_deg):+.3f}°   '
          f'θy 均值 = {statistics.fmean(ys_deg):+.3f}°')
    print()
    print(f'摆动幅度 (判稳容差 = {tol:.2f}° 去趋势峰峰值):')
    if det is not None:
        print(f'  全段去趋势 PP: θx={math.degrees(det[0]):.3f}°  '
              f'θy={math.degrees(det[1]):.3f}°')
    print(f'  |θ|max: θx={max(abs(v) for v in xs_deg):.3f}°  '
          f'θy={max(abs(v) for v in ys_deg):.3f}°')
    print()
    print('绳长模型核验 (实测周期应接近模型周期):')
    print(f'  模型 T={T_model:.2f}s (按 Z={z_grab:.2f}m)', end='')
    if T_meas:
        err = (T_meas - T_model) / T_model * 100.0
        print(f'   实测 T={T_meas:.2f}s   偏差 {err:+.1f}%')
        L_meas = 9.81 * (T_meas / (2 * math.pi)) ** 2
        print(f'  实测周期反推有效摆长 L={L_meas:.2f}m '
              f'(对应 sheave+offset = {L_meas + z_grab:.2f})')
    else:
        print('   实测 T: 摆动太小/样本不足, 无法估计 (静止观测属正常)')

    tau = _envelope_tau(samples)
    if tau is not None:
        print()
        print('自然阻尼 (不做防摇时的实证):')
        print(f'  包络衰减时间常数 τ≈{tau:.1f}s → 1°→{tol:.1f}° 需要 '
              f'{tau * math.log(1.0 / tol):.0f}s')
    return 0


def _envelope_tau(samples: list[tuple[float, float, float]]) -> float | None:
    """用前后半段的去趋势峰峰值比估计包络时间常数 τ [s]。"""
    n = len(samples)
    if n < 64:
        return None
    mid = n // 2
    first = sway_linear_detrend_peak_to_peak(samples[:mid])
    second = sway_linear_detrend_peak_to_peak(samples[mid:])
    if not first or not second:
        return None
    a0, a1 = max(first), max(second)
    if a1 <= 0.0 or a0 <= a1:
        return None
    dt = (samples[-1][0] + samples[mid][0]) / 2 - (
        samples[mid][0] + samples[0][0]
    ) / 2
    if dt <= 0.0:
        return None
    return dt / math.log(a0 / a1)


def main() -> int:
    parser = argparse.ArgumentParser(
        description='倾角仪符号/零偏/摆动的现场只读诊断 (不连 PLC, 不下发指令)',
    )
    parser.add_argument('--config', default=str(DEFAULT_SITE_CONFIG_PATH),
                        help='site_config JSON 路径')
    parser.add_argument('--settle', type=float, default=None, metavar='SECONDS',
                        help='静止观测模式: 采集 N 秒并给出统计 (缺省为实时读数模式)')
    parser.add_argument('--hz', type=float, default=5.0, help='刷新/采样频率')
    parser.add_argument('--z-grab', type=float, default=1.5,
                        help='当前抓钩高度 [m], 用于摆周期模型对照')
    args = parser.parse_args()

    site = load_site_config(args.config)
    if not site:
        print(f'[警告] 未能加载 {args.config} — 将使用默认零偏 0, 读数不可用于零偏核验')

    _start_bridge(site, args.config)
    if not _wait_for_sensor():
        return 1

    if args.settle is not None:
        return run_settle(args.settle, max(args.hz, 10.0), args.z_grab, site)
    return run_live(args.hz)


if __name__ == '__main__':
    sys.exit(main())
