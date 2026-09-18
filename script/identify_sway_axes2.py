#!/usr/bin/env python3
"""倾角仪轴归属 — 全样本回归判定 (比停车事件法统计量大得多)。

方法
----
行进中载荷被台车加速度激励。把两个倾角通道分别对行车 X/Y 加速度做多元回归:
    incl_x = a·ax + b·ay + c
    incl_y = d·ax + e·ay + f
若 |d| >> |a| 且 |b| >> |e|, 则 incl_y 承载行车 X 方向的摆 (→θx),
incl_x 承载行车 Y 方向的摆 (→θy)。用 R² 与系数比给出置信度。

这里只判定**轴归属**, 不判定符号:
行进段的读数混有加速度计"假倾角"(比力方向偏转), 其符号与真实摆角相反, 因此
回归系数的正负不能直接用于定符号。符号必须用静态人工推钩试验确定 (静止 ⇒ 无
加速度 ⇒ 无假倾角), 见 script/check_sway_sign.py。
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

SENTINEL = 1e3


def _clean(arr, cols):
    m = np.ones(len(arr), dtype=bool)
    for c in cols:
        m &= np.abs(arr[:, c]) < SENTINEL
    return arr[m]


def _smooth(v, n):
    return np.convolve(v, np.ones(n) / n, mode='same')


def _collect(paths):
    AX, AY, IX, IY = [], [], [], []
    for p in paths:
        d = np.load(p)
        incl = _clean(d['incl'], (1, 2))
        pose = _clean(d['pose'], (1, 2))
        if len(incl) < 200 or len(pose) < 60:
            continue
        t_p = pose[:, 0]
        dt = np.gradient(t_p)
        vx = _smooth(np.gradient(pose[:, 1]) / np.maximum(dt, 1e-3), 5)
        vy = _smooth(np.gradient(pose[:, 2]) / np.maximum(dt, 1e-3), 5)
        ax = _smooth(np.gradient(vx) / np.maximum(dt, 1e-3), 5)
        ay = _smooth(np.gradient(vy) / np.maximum(dt, 1e-3), 5)
        # 倾角 100Hz 降采样到定位时间戳上
        ix = np.interp(t_p, incl[:, 0], incl[:, 1])
        iy = np.interp(t_p, incl[:, 0], incl[:, 2])
        # 只取真正在动的样本: 静止段无激励, 只会稀释回归
        mv = (np.abs(vx) > 0.02) | (np.abs(vy) > 0.02)
        AX.append(ax[mv]); AY.append(ay[mv])
        IX.append(ix[mv]); IY.append(iy[mv])
    return (np.concatenate(AX), np.concatenate(AY),
            np.concatenate(IX), np.concatenate(IY))


def _regress(y, ax, ay):
    A = np.column_stack([ax, ay, np.ones_like(ax)])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ coef
    ss_res = float(((y - pred) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / max(ss_tot, 1e-12)
    return coef, r2


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('npz', nargs='+')
    args = ap.parse_args()

    ax, ay, ix, iy = _collect(args.npz)
    print()
    print('=' * 74)
    print(f'倾角仪轴归属 — 全样本回归 (n={len(ax)} 个运动样本)')
    print('=' * 74)

    (a, b, _), r2x = _regress(ix, ax, ay)
    (dd, e, _), r2y = _regress(iy, ax, ay)
    print()
    print('  incl_x = %+.3f·ax %+.3f·ay    R²=%.3f' % (a, b, r2x))
    print('  incl_y = %+.3f·ax %+.3f·ay    R²=%.3f' % (dd, e, r2y))
    print()
    print('  对大车加速度 ax 的响应:  incl_x %+.3f  vs  incl_y %+.3f'
          '   → 比值 %.2f' % (a, dd, abs(dd) / max(abs(a), 1e-9)))
    print('  对小车加速度 ay 的响应:  incl_x %+.3f  vs  incl_y %+.3f'
          '   → 比值 %.2f' % (b, e, abs(b) / max(abs(e), 1e-9)))
    print()
    x_owner = 'y' if abs(dd) > abs(a) else 'x'
    y_owner = 'x' if abs(b) > abs(e) else 'y'
    print(f'  ⇒ 行车 X (大车) 的摆 → 倾角 vector.{x_owner}  '
          f'(代码里称 {"pitch" if x_owner == "y" else "roll"})')
    print(f'  ⇒ 行车 Y (小车) 的摆 → 倾角 vector.{y_owner}  '
          f'(代码里称 {"pitch" if y_owner == "y" else "roll"})')
    print()
    if x_owner != y_owner:
        print(f'  对应配置: anti_sway_theta_x_from = '
              f'{"pitch" if x_owner == "y" else "roll"}, '
              f'anti_sway_theta_y_from = {"pitch" if y_owner == "y" else "roll"}')
    else:
        print('  [警告] 两轴判到同一通道 — 激励不足或存在强耦合, 需现场复核')
    print()
    print('  注: 本方法只定轴归属, 不定符号。行进段读数含加速度计假倾角,')
    print('      其符号与真实摆角相反, 必须用静态推钩试验定符号。')
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
