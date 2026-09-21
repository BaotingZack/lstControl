#!/usr/bin/env python3
"""Reproduce model sweeps and read-only field qualification diagnostics.

The field branch computes no new motion: encoder X and negative map Y are
explicit support-coordinate proxies, not reconstructed recording-time control
coordinates. The model branch generates new motion from new commands.
"""

import argparse
from dataclasses import asdict, replace
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from sway_qualification import SwayFeedbackQualifier


def qualify_window(data, start, end):
    """Use only fresh, past samples at PLC times; reset evidence across gaps."""
    encoder = data['/crane/encoders']
    angle = data['/inclination/j1939_msg']
    position = data['/localization_pose']
    qualifier = SwayFeedbackQualifier()
    ever = [False, False]
    first = [None, None]
    valid = skipped = 0
    previous = None
    for sample in encoder[(encoder[:, 0] >= start) & (encoder[:, 0] <= end)]:
        t = float(sample[0])
        selected = []
        for source in (angle, position):
            index = np.searchsorted(source[:, 0], t, side='right') - 1
            if index < 0 or t - source[index, 0] > .25:
                break
            selected.append(source[index])
        if len(selected) != 2:
            qualifier.reset(preserve_revoked=True)
            previous = None
            skipped += 1
            continue
        inc, pose = selected
        # Constant calibration biases are nuisance terms in the fit. No slope,
        # gain or sign is changed to obtain a better fit to these recordings.
        values = (-math.radians(inc[2]), math.radians(inc[1]),
                  float(sample[1]), -float(pose[2]), 4. - float(sample[3]))
        dt = .1 if previous is None else t - previous
        if (not all(math.isfinite(v) for v in values) or
                max(abs(values[2]), abs(values[3])) > 100 or values[4] <= 0 or
                not 0 < dt <= .5):
            qualifier.reset(preserve_revoked=True)
            previous = None
            skipped += 1
            continue
        allowed = qualifier.update(*values, dt)
        previous = t
        valid += 1
        for axis in (0, 1):
            if allowed[axis] and not ever[axis]:
                first[axis] = t
            ever[axis] |= allowed[axis]
    return dict(start_s=start, end_s=end, valid_samples=valid, skipped_samples=skipped,
                ever_qualified=ever, first_qualified_s=first, final=qualifier.snapshot())


def field_report(cached_dir, metrics_path):
    phases = json.loads(metrics_path.read_text(encoding='utf-8'))['recordings']
    results = {}
    for name in ('0918_002_pd', '0918_002_incl'):
        with np.load(cached_dir / f'{name}.npz', allow_pickle=False) as arrays:
            data = {key: arrays[key] for key in
                    ('/crane/encoders', '/inclination/j1939_msg', '/localization_pose')}
        windows = {'10_to_60s': (10., 60.), '75_to_135s': (75., 135.)}
        for i, phase in enumerate(phases[name]['phases']['closed_lowering']):
            windows[f'closed_lowering_{i}'] = (phase['start_s'], phase['end_s'])
        results[name] = {key: qualify_window(data, *window) for key, window in windows.items()}
    return dict(note='Fixed recorded inputs; no new commands or counterfactual sway. '
                'Coordinates: encoder X and -map Y proxy, theta X=-raw pitch, theta Y=raw roll; '
                'length=4-PLC Z. Recording-time transform and actuator inversion unknown. '
                'No PLC speed topic used. Missing/stale inputs reset evidence. '
                'Rejection is a model-consistency result under these proxy coordinates, '
                'not identification of a sign or delay and not field safety certification.', results=results)


def model_sweep():
    from crane_model import CraneState
    from main import _build_arg_parser, _config_from_args, run_simulation
    from site_config import load_site_config
    from sway_simulation import SwaySimulationConfig, terminal_metrics

    cfg = replace(_config_from_args(_build_arg_parser().parse_args([]), load_site_config(), []),
                  dt=.1, enable_disturbance=False, anti_sway_terminal_enabled=True)
    results = []
    for height, distance, initial_y in itertools.product((.5, 1., 1.5, 2.), (.3, 1., 3.), (-1., 0., 1.)):
        target = (distance, .65 * distance, height)
        settings = SwaySimulationConfig(duration_s=65., sensor_mode='ideal',
                                       static_bias='none', initial_theta_y_deg=initial_y)
        runs, metrics = {}, {}
        for label, controller in (('pd', replace(cfg, enable_anti_sway=False)), ('terminal', cfg)):
            history, events = run_simulation(target, CraneState(z0=height), controller,
                verbose=False, sway_sim=settings, sway_safe_height=height, max_time=66.)
            runs[label] = history
            metrics[label] = terminal_metrics(history, events, controller)
        stops = [metrics[label]['control_stop_s'] for label in ('pd', 'terminal')]
        pp, passed = None, False
        if all(stop is not None for stop in stops):
            start = max(stops)
            end = start + 3 * max(metrics[label]['stop_period_s'] for label in ('pd', 'terminal'))
            if all(history[-1]['t'] >= end for history in runs.values()):
                pp = {}
                for label, history in runs.items():
                    rows = [r for r in history if start <= r['t'] <= end]
                    pp[label] = [float(np.degrees(np.ptp([r['sim_true_theta_'+a] for r in rows]))) for a in 'xy']
                passed = all(c <= b + 1e-8 for c, b in zip(pp['terminal'], pp['pd']))
        position_ok = all(abs(v) <= cfg.arrival_pos_tol for v in metrics['terminal']['final_error_m'])
        row = dict(height_m=height, distance_x_m=distance, initial_y_deg=initial_y,
                   metrics=metrics, common_stopped_pp_deg=pp,
                   per_axis_residual_pass=passed, position_pass=position_ok)
        results.append(row)
        print('sweep', height, distance, initial_y, passed, position_ok, flush=True)
    return dict(note='36 ideal-transfer model cases, common absolute post-stop window; '
                'not 36 field tests. True damping=.015; see controller and scenario configuration.',
                controller=asdict(cfg), duration_s=65., target_y_over_x=.65,
                cases=len(results), feedback_active_cases=sum(
                    any(t is not None for t in r['metrics']['terminal']['first_feedback_s']) for r in results),
                failed_cases=sum(not (r['per_axis_residual_pass'] and r['position_pass']) for r in results),
                results=results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cached-dir', type=Path)
    parser.add_argument('--metrics', type=Path, default=REPO / 'artifacts/inclinometer_0918/recordings/recording_metrics.json')
    parser.add_argument('--sweep', action='store_true')
    parser.add_argument('--output-dir', type=Path, default=REPO / 'artifacts/inclinometer_0918/terminal')
    args = parser.parse_args()
    if args.cached_dir is None and not args.sweep:
        parser.error('select --cached-dir and/or --sweep')
    report = {}
    if args.cached_dir is not None:
        report['field_qualification'] = field_report(args.cached_dir, args.metrics)
    if args.sweep:
        report['model_sweep'] = model_sweep()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / 'validation.json'
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(output)
    return int(report.get('model_sweep', {}).get('failed_cases', 0) > 0)


if __name__ == '__main__':
    raise SystemExit(main())
