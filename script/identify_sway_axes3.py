#!/usr/bin/env python3
"""倾角仪轴归属 — 受迫单摆正向模型判定 (决定性方法)。

为什么前两种方法不够
--------------------
1) 停车事件法: 作业时 PD 同时驱动 X/Y, 纯单轴走行极少, 样本只有个位数。
2) 加速度静态回归: R²≈0.007。单摆是二阶共振系统, 响应与激励不同相,
   准静态回归被相位差洗掉, 无法分辨。

本方法
------
用实测台车加速度驱动受迫单摆方程, 得到两个方向的**预测摆角**:
    L·θ̈ + g·θ = −ẍ   (加轻阻尼)
θx_pred 由大车加速度 ax 驱动, θy_pred 由小车加速度 ay 驱动。二者相位/频率结构
不同 (因为激励时序不同), 于是把两个倾角通道分别与 θx_pred / θy_pred 做相关,
相关高的即为该方向的承载通道。共振相位由模型显式给出, 所以不会被洗掉。

绳长随起升高度变化, 逐样本用 enc.z (起升高度) 更新 L。

仍然只定轴归属, 不定符号 — 见文件末尾说明。
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

SENTINEL = 1e3
GRAVITY = 9.81


def _clean(arr, cols):
    m = np.ones(len(arr), dtype=bool)
    for c in cols:
        m &= np.abs(arr[:, c]) < SENTINEL
    return arr[m]


def _smooth(v, n):
    if n <= 1:
        return v
    return np.convolve(v, np.ones(n) / n, mode='same')


def _sim_pendulum(t, acc, L, zeta=0.03):
    """受迫单摆: L·θ̈ + 2ζω L·θ̇ + g·θ = −acc。返回 θ(t) [rad]。"""
    th = np.zeros_like(t)
    thd = 0.0
    thv = 0.0
    for i in range(1, len(t)):
        dt = min(max(t[i] - t[i - 1], 1e-3), 0.5)
        Li = L[i] if np.ndim(L) else L
        w = np.sqrt(GRAVITY / Li)
        thdd = (-acc[i] - GRAVITY * thv) / Li - 2.0 * zeta * w * thd
        thd += thdd * dt
        thv += thd * dt
        th[i] = thv
    return th


def _corr(a, b):
    a = a - a.mean()
    b = b - b.mean()
    d = np.sqrt((a * a).sum() * (b * b).sum())
    return float((a * b).sum() / d) if d > 0 else 0.0


def analyse(path, sheave=4.0, offset=0.0):
    d = np.load(path)
    incl = _clean(d['incl'], (1, 2))
    cmd = _clean(d['cmd'], (1, 2, 3))
    enc = _clean(d['enc'], (1, 3))        # 只要 x 与 z, 跳过坏的 y
    if len(incl) < 200 or len(cmd) < 60:
        return None

    t = cmd[:, 0]
    dt = np.gradient(t)
    # 指令速度差分 → 加速度 (比二次差分定位更干净)
    ax = _smooth(np.gradient(_smooth(cmd[:, 1], 3)) / np.maximum(dt, 1e-3), 5)
    ay = _smooth(np.gradient(_smooth(cmd[:, 2], 3)) / np.maximum(dt, 1e-3), 5)

    z = np.interp(t, enc[:, 0], enc[:, 3]) if len(enc) > 10 else np.full_like(t, 1.0)
    L = np.clip(sheave + offset - z, 0.5, 12.0)

    thx = _sim_pendulum(t, ax, L)
    thy = _sim_pendulum(t, ay, L)

    ix = np.interp(t, incl[:, 0], incl[:, 1])
    iy = np.interp(t, incl[:, 0], incl[:, 2])
    # 去掉慢变零偏 (高通): 零偏随高度/抓钩状态变化, 会污染相关
    ix = ix - _smooth(ix, 61)
    iy = iy - _smooth(iy, 61)

    return {
        'n': len(t),
        'ix_thx': _corr(ix, thx), 'ix_thy': _corr(ix, thy),
        'iy_thx': _corr(iy, thx), 'iy_thy': _corr(iy, thy),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('npz', nargs='+')
    ap.add_argument('--sheave', type=float, default=4.0)
    ap.add_argument('--offset', type=float, default=0.0)
    args = ap.parse_args()

    rows = []
    for p in args.npz:
        r = analyse(p, args.sheave, args.offset)
        if r:
            r['bag'] = p.split('/')[-1].replace('.npz', '')
            rows.append(r)
    if not rows:
        print('无可用数据')
        return 1

    print()
    print('=' * 78)
    print('倾角仪轴归属 — 受迫单摆模型相关性')
    print('=' * 78)
    print()
    print('  与"大车激励预测摆角 θx_pred"的相关     与"小车激励 θy_pred"的相关')
    print('  %-10s %-12s %-12s   %-12s %-12s' % (
        'bag', 'incl_x', 'incl_y', 'incl_x', 'incl_y'))
    for r in rows:
        print('  %-10s %+.3f        %+.3f          %+.3f        %+.3f' % (
            r['bag'], r['ix_thx'], r['iy_thx'], r['ix_thy'], r['iy_thy']))

    w = np.array([r['n'] for r in rows], dtype=float)
    w /= w.sum()
    m = {k: float(sum(r[k] * wi for r, wi in zip(rows, w)))
         for k in ('ix_thx', 'iy_thx', 'ix_thy', 'iy_thy')}
    print()
    print('  加权平均   %+.3f        %+.3f          %+.3f        %+.3f' % (
        m['ix_thx'], m['iy_thx'], m['ix_thy'], m['iy_thy']))
    print()

    x_owner = 'y' if abs(m['iy_thx']) > abs(m['ix_thx']) else 'x'
    y_owner = 'x' if abs(m['ix_thy']) > abs(m['iy_thy']) else 'y'
    name = {'x': 'roll', 'y': 'pitch'}
    print(f'  ⇒ 行车 X (大车) 的摆 → vector.{x_owner} = 代码里的 {name[x_owner]}')
    print(f'  ⇒ 行车 Y (小车) 的摆 → vector.{y_owner} = 代码里的 {name[y_owner]}')
    print()
    if x_owner == y_owner:
        print('  [警告] 判到同一通道 — 证据不足, 必须现场静态复核')
    else:
        print(f'  对应配置: anti_sway_theta_x_from={name[x_owner]}, '
              f'anti_sway_theta_y_from={name[y_owner]}')
    margin = min(
        abs(m[f'i{x_owner}_thx']) - abs(m[f'i{"y" if x_owner == "x" else "x"}_thx']),
        abs(m[f'i{y_owner}_thy']) - abs(m[f'i{"y" if y_owner == "x" else "x"}_thy']),
    )
    print(f'  区分度 (相关系数差): {margin:+.3f}', end='')
    print('  — 偏小, 仅作参考' if margin < 0.1 else '  — 可用')
    print()
    print('  注: 只定轴归属, 不定符号。行进段含加速度计假倾角 (符号与真实摆角相反),')
    print('      符号必须用静态人工推钩试验确定。')
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
