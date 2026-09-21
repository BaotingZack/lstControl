#!/usr/bin/env python3
"""Same-input angle-branch replay; never a counterfactual field trajectory.

Uses existing NPZ/phase metrics from diagnose_inclinometer_recordings.py.
Historical runtime is loaded from local, trusted Git revision ffe67ac.
"""

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import types

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from anti_sway_runtime import AntiSwayRuntime
from main import _build_arg_parser, _config_from_args
from site_config import build_sway_axis_mapping, load_site_config
from sway_controller import build_anti_sway
from sway_estimator import InclinometerSwayEstimator


def legacy_runtime():
    source = subprocess.check_output(['git', 'show', 'ffe67ac:anti_sway_runtime.py'], cwd=REPO, text=True)
    module = types.ModuleType('_lst_legacy_runtime_ffe67ac')
    sys.modules[module.__name__] = module
    exec(compile(source, 'ffe67ac:anti_sway_runtime.py', 'exec'), module.__dict__)
    return module.AntiSwayRuntime


def replay(data, phase, config, estimator, runtime_class):
    inc, enc, vel = [data[k] for k in ('/inclination/j1939_msg', '/crane/encoders', '/crane/cmd_vel')]
    start, end = phase['start_s'], phase['end_s']
    times = enc[(enc[:, 0] >= start) & (enc[:, 0] <= end), 0]
    if len(times) < 8:
        raise ValueError('matched phase does not overlap the input recording')
    runtime = runtime_class(build_anti_sway(config), dc_tau=config.anti_sway_settle_dc_tau,
                            locked_max_shift=config.anti_sway_locked_max_shift,
                            recenter_gain=config.anti_sway_hold_recenter_gain)
    runtime.reset(0., 0.)
    out = []
    for index, t in enumerate(times):
        # Causal latest sample, no interpolation using future sensor messages.
        ir, er, vr = [a[max(0, np.searchsorted(a[:, 0], t, side='right') - 1)] for a in (inc, enc, vel)]
        angle = estimator.estimate(ir[1], ir[2], gripper_closed=bool(vr[4] > .5))
        dt = float(t - times[index - 1]) if index else .1
        vx, vy, _, _ = runtime.merge_xy_commands(
            0., 0., **angle, z_grab=float(er[3]), x=0., y=0.,
            locked_x=True, locked_y=True, dt=dt, max_velocity_xy=config.max_velocity_xy,
            gain_scale=config.anti_sway_settle_gain_scale,
        )
        out.append(dict(t=float(t - times[0]), raw_x_deg=float(ir[1]), raw_y_deg=float(ir[2]),
                        correction_x_mm_s=1000 * vx, correction_y_mm_s=1000 * vy))
    events = getattr(getattr(runtime, 'guard', None), 'events', [])
    # Guard's elapsed time includes the assumed first 0.1 s step; align to plot zero.
    events = [dict(event, t=event['t'] - .1) for event in events]
    summary = dict(rms_mm_s=[float(np.sqrt(np.mean([r[f'correction_{a}_mm_s']**2 for r in out])))
                            for a in ('x', 'y')], guard_events=events)
    return out, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cached-dir', type=Path, required=True)
    parser.add_argument('--metrics', type=Path, default=REPO / 'artifacts/inclinometer_0918/recordings/recording_metrics.json')
    parser.add_argument('--output-dir', type=Path, default=REPO / 'artifacts/inclinometer_0918/feedback_revision')
    args = parser.parse_args()
    site = load_site_config(REPO / 'site_config.json')
    config = _config_from_args(_build_arg_parser().parse_args(['--anti-sway']), site, ['--anti-sway'])
    estimator = InclinometerSwayEstimator(
        build_sway_axis_mapping(site), angle_scale=math.pi / 180,
        roll_bias=config.anti_sway_roll_bias, pitch_bias=config.anti_sway_pitch_bias,
        roll_bias_closed=config.anti_sway_roll_bias_closed, pitch_bias_closed=config.anti_sway_pitch_bias_closed,
    )
    metrics = json.loads(args.metrics.read_text(encoding='utf-8'))['recordings']
    legacy = legacy_runtime()
    results = {}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    for name in ('0918_002_incl', '0918_002_pd'):
        with np.load(args.cached_dir / f'{name}.npz') as arrays:
            data = {k: arrays[k] for k in ('/inclination/j1939_msg', '/crane/encoders', '/crane/cmd_vel')}
        for phase_name in ('closed_lowering', 'open_final_lifting'):
            for n, phase in enumerate(metrics[name]['phases'][phase_name]):
                key = f'{name}_{phase_name}_{n}'
                records, result = {}, {'start_s': phase['start_s'], 'end_s': phase['end_s']}
                for label, cls in (('ffe67ac', legacy), ('corrected', AntiSwayRuntime)):
                    records[label], result[label] = replay(data, phase, config, estimator, cls)
                fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
                q = records['corrected']
                axes[0].plot([r['t'] for r in q], [r['raw_x_deg'] for r in q], label='Raw X (trolley)')
                axes[0].plot([r['t'] for r in q], [r['raw_y_deg'] for r in q], label='Raw Y (bridge)')
                for label, rows in records.items():
                    for axis, ax in zip(('x', 'y'), axes[1:]):
                        ax.plot([r['t'] for r in rows], [r[f'correction_{axis}_mm_s'] for r in rows], label=label)
                for event in result['corrected']['guard_events']:
                    axes[1 if event['axis'] == 'x' else 2].axvline(event['t'], color='black', ls=':')
                for ax, ylabel in zip(axes, ('Input [deg]', 'X overlay [mm/s]', 'Y overlay [mm/s]')):
                    ax.set_ylabel(ylabel)
                    ax.legend()
                    ax.grid(alpha=.25)
                axes[-1].set_xlabel('Time in recorded phase [s]')
                fig.suptitle(key + '\nSame recorded input; these are command overlays, not new field sway')
                fig.tight_layout()
                fig.savefig(args.output_dir / f'{key}.png', dpi=150)
                plt.close(fig)
                results[key] = dict(result, histories=records)
                print(key, json.dumps(result, ensure_ascii=False))
    report = dict(
        note='Command-only diagnostic: force feedback allowed, zero PD, both axes locked, support at anchor. '
             'PLC packet times select latest causal inclinometer/height/gripper samples. '
             'Recorded readings do not change when commands change. No new trajectory or sway prediction. '
             '001 excluded from height-matched replay because PLC was not recorded.',
        results=results,
    )
    (args.output_dir / 'replay.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
