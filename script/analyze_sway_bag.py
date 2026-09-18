#!/usr/bin/env python3
"""离线分析 rosbag 倾角数据 — 为判稳/零偏方案选型提供依据。

读取 /inclination/j1939_msg + /localization_pose + /crane/cmd_vel,
对比 raw 峰峰值 vs 线性去趋势峰峰值 vs |θ|max, 估计摆周期 T,
并给出判稳窗长建议。

用法:
    python3 script/analyze_sway_bag.py /dataSet/lst_003.bag
    python3 script/analyze_sway_bag.py /dataSet/lst_*.bag --config site_config.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import numpy as np

from sway_settle import (
    adaptive_settle_window,
    pendulum_period,
    sway_linear_detrend_peak_to_peak,
    sway_raw_peak_to_peak,
)

GRAVITY = 9.81


def load_site_config(path: Path | None) -> dict:
    if path is None or not path.is_file():
        return {}
    with path.open(encoding='utf-8') as fh:
        raw = json.load(fh)
    return raw if isinstance(raw, dict) else {}


def load_bag(bag_path: Path, inc_topic: str, loc_topic: str, cmd_topic: str):
    import rosbag

    roll_t, pitch_t = [], []
    z_t, vx_t, vy_t = [], [], []
    cmd_t, cmd_mag = [], []

    with rosbag.Bag(str(bag_path), 'r') as bag:
        for topic, msg, t in bag.read_messages(
            topics=[inc_topic, loc_topic, cmd_topic]
        ):
            ts = t.to_sec()
            if topic == inc_topic:
                roll_t.append((ts, float(msg.vector.x)))
                pitch_t.append((ts, float(msg.vector.y)))
            elif topic == loc_topic:
                z_t.append((ts, float(msg.pose.pose.position.z)))
                vx_t.append((ts, float(msg.twist.twist.linear.x)))
                vy_t.append((ts, float(msg.twist.twist.linear.y)))
            elif topic == cmd_topic:
                lx = float(msg.twist.linear.x)
                ly = float(msg.twist.linear.y)
                lz = float(msg.twist.linear.z)
                cmd_t.append(ts)
                cmd_mag.append(math.sqrt(lx * lx + ly * ly + lz * lz))

    def to_arr(pairs):
        if not pairs:
            return np.array([]), np.array([])
        t = np.array([p[0] for p in pairs])
        v = np.array([p[1] for p in pairs])
        return t, v

    t_r, roll = to_arr(roll_t)
    t_p, pitch = to_arr(pitch_t)
    t_z, z = to_arr(z_t)
    t_vx, vx = to_arr(vx_t)
    return {
        't_r': t_r, 'roll': roll, 'pitch': pitch,
        't_z': t_z, 'z': z, 'vx': vx,
        'cmd_t': np.array(cmd_t), 'cmd_mag': np.array(cmd_mag),
    }


def theta_from_raw(roll, pitch, roll_bias, pitch_bias, angle_scale):
    tx = (pitch - pitch_bias) * angle_scale
    ty = (roll - roll_bias) * angle_scale
    return tx, ty


def estimate_period_fft(t: np.ndarray, theta: np.ndarray) -> float | None:
    if len(t) < 128:
        return None
    dt = float(np.median(np.diff(t)))
    if dt <= 0:
        return None
    seg = theta - np.mean(theta)
    n = min(len(seg), 8192)
    seg = seg[:n]
    spec = np.abs(np.fft.rfft(seg))
    freqs = np.fft.rfftfreq(n, dt)
    if len(spec) < 3:
        return None
    i = 1 + int(np.argmax(spec[1: max(32, len(spec) // 8)]))
    f0 = float(freqs[i])
    if f0 <= 1e-3:
        return None
    return 1.0 / f0


def sliding_max_pp(t, theta, t_end, span_s, win_s, detrend: bool):
    """在 [t_end-span_s, t_end] 内滑动窗口, 返回最大峰峰值 [rad]。"""
    if len(t) < 3:
        return float('nan')
    pps = []
    mask = (t >= t_end - span_s) & (t <= t_end)
    idxs = np.where(mask)[0]
    for i in idxs:
        w = (t >= t[i] - win_s) & (t <= t[i])
        if w.sum() < 3:
            continue
        samples = [(float(t[j]), float(theta[j]), 0.0) for j in np.where(w)[0]]
        if detrend:
            pp = sway_linear_detrend_peak_to_peak(samples)
        else:
            pp = sway_raw_peak_to_peak(samples)
        if pp is None:
            continue
        pps.append(max(pp))
    return float(np.degrees(max(pps))) if pps else float('nan')


def analyze_one(bag_path: Path, cfg: dict, args) -> dict:
    roll_bias = float(cfg.get('anti_sway_roll_bias', 0.0))
    pitch_bias = float(cfg.get('anti_sway_pitch_bias', 0.0))
    angle_deg = bool(cfg.get('anti_sway_angle_deg', True))
    angle_scale = math.pi / 180.0 if angle_deg else 1.0
    sheave = float(cfg.get('rope_length_sheave_height', 8.0))
    grab_off = float(cfg.get('rope_length_grab_offset', 1.5))
    base_win = float(cfg.get('hook_settle_window', 0.6))
    win_periods = float(cfg.get('hook_settle_window_periods', 0.55))
    tol_deg = float(cfg.get('hook_sway_angle_tol_deg', 0.3))

    data = load_bag(bag_path, args.inc_topic, args.loc_topic, args.cmd_topic)
    t = data['t_r']
    if len(t) < 10:
        return {'bag': str(bag_path), 'error': '倾角数据不足'}

    tx, ty = theta_from_raw(data['roll'], data['pitch'], roll_bias, pitch_bias, angle_scale)
    peak = np.degrees(np.maximum(np.abs(tx), np.abs(ty)))
    duration = float(t[-1] - t[0])

    z_med = float(np.median(data['z'])) if len(data['z']) else 1.0
    T_model = pendulum_period(
        z_med, sheave_height=sheave, grab_offset=grab_off,
    )
    T_fft_x = estimate_period_fft(t, tx)
    T_fft_y = estimate_period_fft(t, ty)
    T_est = T_fft_x or T_fft_y or T_model

    adapt_win = adaptive_settle_window(
        z_med, base_window=base_win, window_periods=win_periods,
        sheave_height=sheave, grab_offset=grab_off,
    )

    t_end = t[-1]
    windows = sorted(set([
        0.6, 1.0, adapt_win,
        (T_est * 0.55) if T_est else 3.0,
    ]))

    def pp_pair(win, detrend):
        px = sliding_max_pp(t, tx, t_end, 30.0, win, detrend)
        py = sliding_max_pp(t, ty, t_end, 30.0, win, detrend)
        return px, py, max(px, py) if not (math.isnan(px) or math.isnan(py)) else float('nan')

    metrics = {}
    for win in sorted(set(windows)):
        rx, ry, rm = pp_pair(win, False)
        dx, dy, dm = pp_pair(win, True)
        metrics[win] = {
            'raw_pp_x': rx, 'raw_pp_y': ry, 'raw_pp_max': rm,
            'detrend_pp_x': dx, 'detrend_pp_y': dy, 'detrend_pp_max': dm,
        }

    abs_max_30s = float(peak[t >= t_end - 30].max()) if (t >= t_end - 30).any() else float(peak.max())

    # 运动 vs 静止 (cmd_vel)
    move_pp, stop_pp = [], []
    if len(data['cmd_t']) > 0:
        for i, tc in enumerate(data['cmd_t']):
            w = (t >= tc - 1.0) & (t <= tc)
            if w.sum() < 5:
                continue
            samples = [(float(t[j]), float(tx[j]), float(ty[j])) for j in np.where(w)[0]]
            pp = sway_linear_detrend_peak_to_peak(samples)
            if pp is None:
                continue
            val = math.degrees(max(pp))
            (move_pp if data['cmd_mag'][i] > 0.01 else stop_pp).append(val)

    return {
        'bag': bag_path.name,
        'duration_s': duration,
        'inc_hz': len(t) / duration,
        'loc_hz': len(data['t_z']) / duration if duration > 0 else 0,
        'z_min': float(data['z'].min()) if len(data['z']) else None,
        'z_max': float(data['z'].max()) if len(data['z']) else None,
        'T_model_s': T_model,
        'T_fft_x_s': T_fft_x,
        'T_fft_y_s': T_fft_y,
        'adapt_window_s': adapt_win,
        'tol_deg': tol_deg,
        'abs_theta_max_30s_deg': abs_max_30s,
        'metrics': metrics,
        'move_pp_median': float(np.median(move_pp)) if move_pp else None,
        'stop_pp_median': float(np.median(stop_pp)) if stop_pp else None,
        'roll_std_deg': float(data['roll'].std()),
        'pitch_std_deg': float(data['pitch'].std()),
    }


def print_report(result: dict) -> None:
    if 'error' in result:
        print(f"\n=== {result.get('bag', '?')} === ERROR: {result['error']}")
        return

    print(f"\n{'='*60}")
    print(f"Bag: {result['bag']}  ({result['duration_s']:.0f}s)")
    print(f"  倾角 ~{result['inc_hz']:.0f} Hz   定位 ~{result['loc_hz']:.1f} Hz")
    if result['z_min'] is not None:
        print(f"  SLAM Z: {result['z_min']:.2f} ~ {result['z_max']:.2f} m")
    print(f"  摆周期: 模型 T={result['T_model_s']:.2f}s"
          + (f"  FFT θx T={result['T_fft_x_s']:.2f}s" if result['T_fft_x_s'] else "")
          + (f"  FFT θy T={result['T_fft_y_s']:.2f}s" if result['T_fft_y_s'] else ""))
    print(f"  建议判稳窗 (自适应): {result['adapt_window_s']:.2f}s"
          f"  阈值: {result['tol_deg']:.2f}° 峰峰值(去趋势)")
    print(f"  末30s |θ|max={result['abs_theta_max_30s_deg']:.3f}°")
    if result['move_pp_median'] is not None:
        print(f"  1s去趋势PP: 运动段 median={result['move_pp_median']:.3f}°"
              f"  静止段 median={result['stop_pp_median']:.3f}°")
    print(f"  raw roll/pitch std: {result['roll_std_deg']:.3f}° / {result['pitch_std_deg']:.3f}°")
    print("  末30s 滑动峰峰值 [deg] (x/y/max):")
    for win, m in sorted(result['metrics'].items()):
        print(f"    窗 {win:.2f}s  raw=({m['raw_pp_x']:.3f}, {m['raw_pp_y']:.3f}, max={m['raw_pp_max']:.3f})"
              f"  detrend=({m['detrend_pp_x']:.3f}, {m['detrend_pp_y']:.3f}, max={m['detrend_pp_max']:.3f})")

    # 方案建议
    m06 = result['metrics'].get(0.6, {})
    mad = result['metrics'].get(result['adapt_window_s'], {})
    if mad:
        det_max = mad.get('detrend_pp_max', float('nan'))
        raw06 = m06.get('raw_pp_max', float('nan'))
        mad_max = mad.get('raw_pp_max', float('nan'))
        print("  --- 方案提示 ---")
        if mad_max > raw06 * 1.15:
            print("  → 加长判稳窗 (方案 A): 短窗 PP 明显低于长窗, 0.6s 易漏检")
        if result['abs_theta_max_30s_deg'] > result['tol_deg'] * 2 and det_max < result['tol_deg']:
            print("  → |θ| 大但去趋势 PP 小: 固定零偏占主导 (方案 B/C 在线 θ_dc)")
        if det_max > result['tol_deg']:
            print("  → 去趋势 PP 仍超阈值: 真实摆动, 需主动阻尼或更长等待")
        if det_max <= result['tol_deg']:
            print("  → 去趋势 PP 末段已低于阈值: 峰峰值判稳 + 自适应窗可行")


def main(argv=None):
    p = argparse.ArgumentParser(description='分析 bag 倾角数据, 辅助判稳方案选型')
    p.add_argument('bags', nargs='+', help='bag 文件路径 (可多个)')
    p.add_argument('--config', type=Path, default=_REPO_ROOT / 'site_config.json')
    p.add_argument('--inc-topic', default='/inclination/j1939_msg')
    p.add_argument('--loc-topic', default='/localization_pose')
    p.add_argument('--cmd-topic', default='/crane/cmd_vel')
    p.add_argument('--report', type=Path, default=None, help='写入 Markdown 报告')
    args = p.parse_args(argv)

    cfg = load_site_config(args.config)
    results = []
    for bag in args.bags:
        path = Path(bag)
        if not path.is_file():
            print(f'跳过 (不存在): {path}', file=sys.stderr)
            continue
        print(f'分析 {path} ...', file=sys.stderr)
        results.append(analyze_one(path, cfg, args))
        print_report(results[-1])

    if args.report and results:
        lines = ['# 倾角 bag 分析报告\n', f'配置: `{args.config}`\n']
        for r in results:
            if 'error' in r:
                lines.append(f'\n## {r.get("bag")}\n\n错误: {r["error"]}\n')
                continue
            lines.append(f'\n## {r["bag"]}\n')
            lines.append(f'- 时长 {r["duration_s"]:.0f}s, 建议判稳窗 **{r["adapt_window_s"]:.2f}s**\n')
            lines.append(f'- 模型摆周期 {r["T_model_s"]:.2f}s\n')
            lines.append(f'- 末30s |θ|max = {r["abs_theta_max_30s_deg"]:.3f}°\n')
            for win, m in sorted(r['metrics'].items()):
                lines.append(
                    f'- 窗 {win:.2f}s: detrend PP max = **{m["detrend_pp_max"]:.3f}°**\n'
                )
        args.report.write_text(''.join(lines), encoding='utf-8')
        print(f'\n报告已写入 {args.report}', file=sys.stderr)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
