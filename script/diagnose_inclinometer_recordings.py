#!/usr/bin/env python3
"""Compare 0918 recordings by PLC height/gripper phase; never edits source data.

Raw ROS1 bags require the optional rosbags package (no running ROS needed).
--cached-dir accepts the explicit NPZ intermediate format to separate reading
bags from numerical analysis. All outputs are derived artifacts.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import tempfile
from xml.etree import ElementTree as ET
from zipfile import ZipFile

import numpy as np

INC = '/inclination/j1939_msg'
ENC = '/crane/encoders'
VEL = '/crane/cmd_vel'
POSE = '/localization_pose'


def read_bag(path):
    from rosbags.highlevel import AnyReader

    values = {}
    with AnyReader([path]) as reader:
        connections = [c for c in reader.connections if c.topic in (INC, ENC, VEL, POSE)]
        for conn, stamp, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, conn.msgtype)
            t = (stamp - reader.start_time) / 1e9
            if conn.topic in (INC, ENC):
                row = [t, msg.vector.x, msg.vector.y, msg.vector.z]
            elif conn.topic == VEL:
                row = [t, msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.x]
            else:
                p, v = msg.pose.pose.position, msg.twist.twist.linear
                row = [t, p.x, p.y, p.z, v.x, v.y, v.z]
            values.setdefault(conn.topic, []).append(row)
    return {key: np.asarray(rows) for key, rows in values.items()}


def static_readings(path):
    ns = {'w': 'http://schemas.openxmlformats.org/wordprocessingml/2006/main'}
    with ZipFile(path) as archive:
        root = ET.fromstring(archive.read('word/document.xml'))
    station, loaded, height = None, False, None
    rows = []
    for paragraph in root.findall('.//w:p', ns):
        text = ''.join(node.text or '' for node in paragraph.findall('.//w:t', ns)).strip()
        heading = re.match(r'^(B1|B2|C1|C2|D1)', text)
        if heading:
            station, loaded, height = heading[1], '载货' in text, None
            continue
        if station is None:
            continue
        numbers = [float(value) for value in re.findall(r'[-+]?\d+(?:\.\d+)?', text)]
        if len(numbers) >= 3 and 'm' in text:
            height, roll, pitch = numbers[:3]
        elif len(numbers) == 2 and height is not None:
            roll, pitch = numbers
        else:
            continue
        rows.append(dict(station=station, loaded=loaded, height_m=height, roll_deg=roll, pitch_deg=pitch))
    return rows


def detrend(t, values):
    design = np.column_stack([np.ones(len(t)), t - t[0]])
    return values - design @ np.linalg.lstsq(design, values, rcond=None)[0]


def intervals(t, mask):
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return [(float(t[a]), float(t[b - 1]))
            for a, b in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))
            if t[b - 1] - t[a] >= 5.]


def angle_metrics(inc, start, end):
    windowed = {}
    for span, remove_trend, key in ((5., False, 'raw_5s_pp_median_deg'), (2., True, 'detrended_2s_pp_median_deg')):
        values = []
        for right in np.arange(start + span, end + 1e-6, .5):
            w = inc[(inc[:, 0] >= right - span) & (inc[:, 0] <= right)]
            if len(w) < span * 75:
                continue
            y = detrend(w[:, 0], w[:, 1:3]) if remove_trend else w[:, 1:3]
            values.append(np.ptp(y, axis=0))
        windowed[key] = np.median(values, axis=0).tolist() if values else None
    return windowed


def phase_metrics(data, start, end):
    inc, enc, vel, pose = (data[key] for key in (INC, ENC, VEL, POSE))
    p = pose[(pose[:, 0] >= start) & (pose[:, 0] <= end)]
    v = vel[(vel[:, 0] >= start) & (vel[:, 0] <= end)]
    e = enc[(enc[:, 0] >= start) & (enc[:, 0] <= end)]
    result = dict(start_s=start, end_s=end, **angle_metrics(inc, start, end))
    result['plc_speed_topic_rms_m_s'] = [float(np.sqrt(np.mean(v[np.abs(v[:, k]) < 1, k]**2))) for k in (1, 2)]
    result['localization_xy_range_m'] = np.ptp(p[:, 1:3], axis=0).tolist()
    result['invalid_y_encoder_samples'] = int(np.sum(np.abs(e[:, 2]) >= 100))
    result['invalid_y_speed_samples'] = int(np.sum(np.abs(v[:, 2]) >= 1))
    # Closed-loop signal relation; do NOT interpret it as an identified delay.
    if result['invalid_y_encoder_samples'] == 0 and result['invalid_y_speed_samples'] == 0:
        t = np.arange(start, end, .1)
        signals = np.column_stack([np.interp(t, inc[:, 0], inc[:, 1]),
                                   np.interp(t, vel[:, 0], vel[:, 2]),
                                   np.interp(t, enc[:, 0], enc[:, 2])])
        signals = detrend(t, signals)
        best = None
        for freq in np.linspace(.15, .5, 1401):
            a = np.column_stack([np.ones(len(t)), t - t.mean(), np.sin(2 * np.pi * freq * t), np.cos(2 * np.pi * freq * t)])
            c = np.linalg.lstsq(a, signals, rcond=None)[0]
            residual = np.sum((signals - a @ c)**2, axis=0)
            if best is None or residual[0] < best[0][0]:
                best = residual, freq, c
        residual, freq, c = best
        phase = np.degrees(np.arctan2(c[3], c[2]))
        result['roll_peak_hz'] = float(freq)
        result['velocity_relative_to_roll_phase_deg'] = float((phase[1] - phase[0] + 180) % 360 - 180)
        result['harmonic_r2_roll_velocity_encoder'] = (1 - residual / np.sum(signals**2, axis=0)).tolist()
    return result


def summarize(name, data):
    inc = data[INC]
    out = dict(recording=name, duration_s=float(inc[-1, 0]), samples={k: len(v) for k, v in data.items() if not k.endswith('_lag')})
    if ENC not in data or VEL not in data:
        out['phase_matching'] = 'Unavailable: PLC height/gripper feedback was not recorded.'
        return out
    enc, vel = data[ENC], data[VEL]
    out['invalid_y_encoder_samples'] = int(np.sum(np.abs(enc[:, 2]) >= 100))
    out['invalid_y_speed_samples'] = int(np.sum(np.abs(vel[:, 2]) >= 1))
    closed = vel[np.maximum(0, np.searchsorted(vel[:, 0], enc[:, 0], side='right') - 1), 4] > .5
    events = np.r_[0, np.flatnonzero(np.diff(vel[:, 4]) != 0) + 1]
    out['gripper_events'] = vel[events][:, [0, 4]].tolist()
    dz = np.gradient(enc[:, 3], enc[:, 0])
    height_band = (enc[:, 3] > .6) & (enc[:, 3] < 1.2)
    phases = {
        'closed_transport_x_3_to_5_5m': closed & (enc[:, 1] > 3.) & (enc[:, 1] < 5.5) & (enc[:, 3] > 1.2),
        'closed_lowering': closed & height_band & (dz < -.01),
        'open_final_lifting': (~closed) & height_band & (dz > .01) & (enc[:, 0] > vel[events[-1], 0]),
    }
    out['phases'] = {name: [phase_metrics(data, start, end) for start, end in intervals(enc[:, 0], mask)]
                     for name, mask in phases.items()}
    return out


def plot_comparison(recordings, summary, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    for mode, color in (('incl', 'tab:red'), ('pd', 'tab:blue')):
        name = f'0918_002_{mode}'
        data = recordings[name]
        phase = summary[name]['phases']['closed_lowering'][0]
        start, end = phase['start_s'], phase['end_s']
        for k, ax, columns, factor in ((INC, axes[0], 1, 1), (INC, axes[1], 2, 1), (VEL, axes[2], 2, 1000), (ENC, axes[3], 3, 1)):
            a = data[k]
            w = a[(a[:, 0] >= start) & (a[:, 0] <= end)]
            y = w[:, columns].copy()
            if k == INC:
                y -= y.mean()
            if k == VEL:
                y[np.abs(y) >= 1] = np.nan
            ax.plot(w[:, 0] - start, y * factor, label=mode, color=color)
    for ax, label in zip(axes, ('Raw X minus mean [deg]', 'Raw Y minus mean [deg]', 'PLC Y speed topic [mm/s]', 'PLC height [m]')):
        ax.set_ylabel(label)
        ax.grid(alpha=.25)
        ax.legend()
    axes[-1].set_xlabel('Time since crossing 1.2 m [s]')
    fig.suptitle('002: gripper closed, descending from 1.2 to 0.6 m')
    fig.tight_layout()
    fig.savefig(output / 'recording_comparison.png', dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('data_dir', type=Path)
    parser.add_argument('--cached-dir', type=Path, help='explicit pre-extracted NPZ intermediates')
    parser.add_argument('--output-dir', type=Path, default=Path('inclinometer_analysis'))
    parser.add_argument('--no-plot', action='store_true', help='extract and analyze without Matplotlib')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    recordings = {}
    if args.cached_dir is not None:
        for path in sorted(args.cached_dir.glob('0918_*.npz')):
            with np.load(path) as arrays:
                recordings[path.stem] = {k: arrays[k] for k in (INC, ENC, VEL, POSE) if k in arrays}
    else:
        with tempfile.TemporaryDirectory(prefix='lst-incl-analysis-') as tmp:
            with ZipFile(args.data_dir / '0918_001_incl.zip') as archive:
                for name in sorted(archive.namelist()):
                    if not name.endswith('.bag'):
                        continue
                    path = Path(tmp) / Path(name).name
                    path.write_bytes(archive.read(name))
                    data = read_bag(path)
                    recordings[path.stem] = data
                    np.savez_compressed(args.output_dir / f'{path.stem}.npz', **data)
    summary = {name: summarize(name, data) for name, data in recordings.items()}
    report = dict(recordings=summary, static_readings=static_readings(args.data_dir / '记录.docx'))
    (args.output_dir / 'recording_metrics.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    for name, row in summary.items():
        print(name, json.dumps(row.get('phases', row.get('phase_matching')), ensure_ascii=False))
    if not args.no_plot:
        plot_comparison(recordings, summary, args.output_dir)


if __name__ == '__main__':
    main()
