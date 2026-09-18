#!/usr/bin/env python3
"""从 bag 数据判定倾角仪轴映射 — 用实测运动反推 θx/θy 的来源与符号。

判定原理
--------
单摆只在被激励的方向上摆。若大车 (行车 X) 单独走一段然后停车, 停车后载荷只在
X 方向做自由摆; 此时**哪个倾角通道在振荡, 哪个通道就对应 θx**。小车 (行车 Y)
同理。这比依赖"横滚/俯仰"的命名约定可靠 —— 命名约定 (绕轴转 vs 沿轴倾) 有两
种互相矛盾的解释, 而安装又把两轴交叉了, 两次交换可能抵消也可能不抵消。

符号判定
--------
停车瞬间载荷继续前冲: 从 +X 走行停车后, 载荷相对吊点偏向 +X, 即 θx 应为正。
所以对"停车后第一个半周"取符号, 就得到 θx 相对该通道的正负。

用法:
    python3 script/identify_sway_axes.py /tmp/lst_003.npz
    python3 script/identify_sway_axes.py /tmp/lst_*.npz
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np

SENTINEL = 1e3          # 编码器/速度溢出哨兵值过滤阈值
MOVE_SPEED = 0.02       # [m/s] 判定"在动"的速度门限
PURE_RATIO = 4.0        # 主轴速度 / 副轴速度 的最小比值, 才算"单轴运动"
RING_S = 8.0            # 停车后取多长的自由摆窗口 [s]
MIN_MOVE_S = 2.0        # 单轴运动至少持续多久才算有效激励


def _clean(arr: np.ndarray, cols: tuple[int, ...]) -> np.ndarray:
    mask = np.ones(len(arr), dtype=bool)
    for c in cols:
        mask &= np.abs(arr[:, c]) < SENTINEL
    return arr[mask]


def _interp(t_out: np.ndarray, t_in: np.ndarray, v_in: np.ndarray) -> np.ndarray:
    return np.interp(t_out, t_in, v_in)


def _detrend_pp(t: np.ndarray, v: np.ndarray) -> float:
    """线性去趋势后的峰峰值 — 去掉零偏漂移, 只留振荡。"""
    if len(v) < 8:
        return 0.0
    coef = np.polyfit(t, v, 1)
    r = v - np.polyval(coef, t)
    return float(r.max() - r.min())


def _find_single_axis_stops(t: np.ndarray, vx: np.ndarray, vy: np.ndarray):
    """找出"单轴运动 → 停车"事件。返回 [(t_stop, 'x'|'y', 方向符号)]。"""
    events = []
    moving = (np.abs(vx) > MOVE_SPEED) | (np.abs(vy) > MOVE_SPEED)
    i = 0
    n = len(t)
    while i < n:
        if not moving[i]:
            i += 1
            continue
        j = i
        while j < n and moving[j]:
            j += 1
        seg = slice(i, j)
        dur = t[j - 1] - t[i]
        if dur >= MIN_MOVE_S:
            ax_speed = np.abs(vx[seg]).mean()
            ay_speed = np.abs(vy[seg]).mean()
            if ax_speed > PURE_RATIO * max(ay_speed, 1e-6):
                events.append((t[j - 1], 'x', math.copysign(1.0, vx[seg].mean())))
            elif ay_speed > PURE_RATIO * max(ax_speed, 1e-6):
                events.append((t[j - 1], 'y', math.copysign(1.0, vy[seg].mean())))
        i = j
    return events


def analyse(path: str) -> dict:
    """用 /localization_pose 的 XY 位移反推单轴走行事件。

    不用 /crane/encoders: 这批包里小车编码器 88~100% 是溢出哨兵值, 不可用。
    定位 XY 与大车编码器行程一致 (5.97 vs 5.97 m), 说明地图系与行车系无显著
    旋转, 可直接当行车坐标使用。
    """
    d = np.load(path)
    incl = _clean(d['incl'], (1, 2))
    pose = _clean(d['pose'], (1, 2))
    if len(incl) < 100 or len(pose) < 40:
        print(f'{path}: 数据不足, 跳过')
        return {}

    t_p = pose[:, 0]
    dt = np.gradient(t_p)
    vx = np.gradient(pose[:, 1]) / np.maximum(dt, 1e-3)
    vy = np.gradient(pose[:, 2]) / np.maximum(dt, 1e-3)
    k = np.ones(5) / 5.0
    vx = np.convolve(vx, k, mode='same')
    vy = np.convolve(vy, k, mode='same')

    events = _find_single_axis_stops(t_p, vx, vy)
    t_i, ix, iy = incl[:, 0], incl[:, 1], incl[:, 2]

    out = {'x': [], 'y': []}
    for t_stop, axis, direction in events:
        m = (t_i >= t_stop) & (t_i <= t_stop + RING_S)
        if m.sum() < 100:
            continue
        tw, xw, yw = t_i[m], ix[m], iy[m]
        pp_ix = _detrend_pp(tw, xw)
        pp_iy = _detrend_pp(tw, yw)
        # 停车后第一个半周的偏移方向 (去掉窗内均值 = 去零偏)
        head = (tw - tw[0]) < 1.2
        sgn_ix = math.copysign(1.0, (xw[head] - xw.mean()).mean())
        sgn_iy = math.copysign(1.0, (yw[head] - yw.mean()).mean())
        out[axis].append({
            'pp_ix': pp_ix, 'pp_iy': pp_iy, 'dir': direction,
            'sgn_ix': sgn_ix, 'sgn_iy': sgn_iy,
        })
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('npz', nargs='+')
    args = ap.parse_args()

    agg = {'x': [], 'y': []}
    for p in args.npz:
        r = analyse(p)
        for k in ('x', 'y'):
            agg[k].extend(r.get(k, []))

    print()
    print('=' * 74)
    print('倾角仪轴映射判定 — 单轴走行停车后的自由摆响应')
    print('=' * 74)
    for axis, label in (('x', '大车 (行车 X)'), ('y', '小车 (行车 Y)')):
        ev = agg[axis]
        print()
        print(f'── {label} 单独走行 → 停车, 共 {len(ev)} 次事件')
        if not ev:
            print('    无有效事件 (该轴没有足够长的单轴走行)')
            continue
        pp_ix = np.array([e['pp_ix'] for e in ev])
        pp_iy = np.array([e['pp_iy'] for e in ev])
        print(f'    倾角 x 通道振荡 PP: 均值 {pp_ix.mean():.3f}°  中位 {np.median(pp_ix):.3f}°')
        print(f'    倾角 y 通道振荡 PP: 均值 {pp_iy.mean():.3f}°  中位 {np.median(pp_iy):.3f}°')
        ratio = pp_ix.mean() / max(pp_iy.mean(), 1e-9)
        winner = 'x' if ratio > 1 else 'y'
        print(f'    → 响应比 x/y = {ratio:.2f}  ⇒ 该方向的摆由 '
              f'**倾角 {winner} 通道 (vector.{winner})** 承载')
        # 符号: 停车后载荷前冲, θ 应与走行方向同号
        key = f'sgn_i{winner}'
        agree = np.array([e[key] * e['dir'] for e in ev])
        pos = (agree > 0).sum()
        print(f'    符号: 停车后首半周与走行方向同号 {pos}/{len(ev)} 次'
              f'  ⇒ 符号应取 {"+1" if pos * 2 >= len(ev) else "-1"}')
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
