#!/usr/bin/env python3
"""从 rosbag 抽取倾角/速度/定位时间序列到 npz — 供轴映射判定使用。

只读 4 个轻量话题, 跳过图像。用法:
    python3 script/extract_bag_axes.py /dataSet/lst_003.bag -o /tmp/lst_003.npz
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import rosbag

TOPICS = [
    '/inclination/j1939_msg',
    '/crane/cmd_vel',
    '/localization_pose',
    '/crane/encoders',
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('-o', '--out', required=True)
    args = ap.parse_args()

    incl: list[tuple[float, float, float, float]] = []
    cmd: list[tuple[float, float, float, float]] = []
    pose: list[tuple[float, float, float, float, float]] = []
    enc: list[tuple[float, float, float, float]] = []

    t_start = time.time()
    with rosbag.Bag(args.bag, 'r') as bag:
        for i, (topic, msg, t) in enumerate(
            bag.read_messages(topics=TOPICS)
        ):
            ts = t.to_sec()
            if topic == '/inclination/j1939_msg':
                v = msg.vector
                incl.append((ts, v.x, v.y, v.z))
            elif topic == '/crane/cmd_vel':
                tw = msg.twist
                cmd.append((ts, tw.linear.x, tw.linear.y, tw.linear.z))
            elif topic == '/localization_pose':
                p = msg.pose.pose.position
                tw = msg.twist.twist.linear
                pose.append((ts, p.x, p.y, tw.x, tw.y))
            elif topic == '/crane/encoders':
                v = msg.vector
                enc.append((ts, v.x, v.y, v.z))
            if i % 5000 == 0:
                print(f'\r  {i} msgs, {time.time() - t_start:.0f}s', end='')
                sys.stdout.flush()

    print(f'\r  done: incl={len(incl)} cmd={len(cmd)} pose={len(pose)} '
          f'enc={len(enc)} in {time.time() - t_start:.0f}s')
    np.savez_compressed(
        args.out,
        incl=np.array(incl), cmd=np.array(cmd),
        pose=np.array(pose), enc=np.array(enc),
    )
    print(f'  -> {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
