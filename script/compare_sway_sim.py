#!/usr/bin/env python3
"""Legacy continuous-feedback sensitivity experiment, not a stopped-plant test.

Use compare_terminal_sway.py for terminal control and residuals after stopping.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from crane_model import CraneState
from main import _build_arg_parser, _config_from_args, run_simulation
from site_config import DEFAULT_SITE_CONFIG_PATH, load_site_config
from sway_simulation import SwaySimulationConfig, save_simulation_result


def compare_motion(history, baseline, window_end_s, allowed_regression_percent=0.):
    """Use the same PD-defined time window; lower RMS cannot excuse a higher peak."""
    def amplitude(rows):
        return np.degrees(np.hypot([r['sim_true_theta_x'] for r in rows],
                                   [r['sim_true_theta_y'] for r in rows]))

    candidate = [r for r in history if r['t'] <= window_end_s + 1e-8]
    reference = [r for r in baseline if r['t'] <= window_end_s + 1e-8]
    a, b = amplitude(candidate), amplitude(reference)
    rms, peak = float(np.sqrt(np.mean(a**2))), float(a.max())
    pd_rms, pd_peak = float(np.sqrt(np.mean(b**2))), float(b.max())
    factor = 1 + allowed_regression_percent / 100
    per_axis = {}
    for axis in ('x', 'y'):
        candidate_angle = np.degrees([r[f'sim_true_theta_{axis}'] for r in candidate])
        pd_angle = np.degrees([r[f'sim_true_theta_{axis}'] for r in reference])
        axis_rms, axis_peak = float(np.sqrt(np.mean(candidate_angle**2))), float(np.abs(candidate_angle).max())
        reference_rms, reference_peak = float(np.sqrt(np.mean(pd_angle**2))), float(np.abs(pd_angle).max())
        per_axis[axis] = dict(rms_deg=axis_rms, peak_deg=axis_peak, pd_rms_deg=reference_rms, pd_peak_deg=reference_peak,
                              not_worse=axis_rms <= reference_rms * factor + 1e-9 and axis_peak <= reference_peak * factor + 1e-9)
    times, reference_times = [r['t'] for r in candidate], [r['t'] for r in reference]
    delta = [np.asarray([r[axis] for r in candidate]) - np.interp(times, reference_times, [r[axis] for r in reference])
             for axis in ('x', 'y')]
    return {
        'window_end_s': window_end_s,
        'rope_rms_deg': rms, 'rope_peak_deg': peak,
        'pd_rope_rms_deg': pd_rms, 'pd_rope_peak_deg': pd_peak,
        'allowed_regression_percent': allowed_regression_percent,
        'per_axis': per_axis,
        'sway_not_worse_than_pd': (rms <= pd_rms * factor + 1e-9 and peak <= pd_peak * factor + 1e-9
                                   and all(row['not_worse'] for row in per_axis.values())),
        'max_trolley_departure_from_pd_m': float(np.hypot(*delta).max()),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=Path('sway_experiments'))
    parser.add_argument('--config', type=Path, default=DEFAULT_SITE_CONFIG_PATH)
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--height', type=float, default=1.5)
    parser.add_argument('--scenario', choices=('travel', 'hold'), default='travel',
                        help='travel is the acceptance experiment; hold isolates measurement effects')
    parser.add_argument('--target-x', type=float, default=2.)
    parser.add_argument('--target-y', type=float, default=1.)
    parser.add_argument('--allowed-regression-percent', type=float, default=0.,
                        help='permitted increase in BOTH motion RMS and peak relative to PD; default zero')
    parser.add_argument('--delay', type=float, default=1., help='hypothetical measurement delay [s], not a field estimate')
    args = parser.parse_args()
    if not np.isfinite(args.allowed_regression_percent) or args.allowed_regression_percent < 0:
        parser.error('--allowed-regression-percent must be finite and non-negative')
    cli = _build_arg_parser().parse_args(['--anti-sway'])
    site = load_site_config(args.config)
    base = _config_from_args(cli, site, ['--anti-sway'])
    base = replace(base, enable_disturbance=False, anti_sway_terminal_enabled=False)
    current_site = replace(_config_from_args(_build_arg_parser().parse_args([]), site, []), enable_disturbance=False)
    # Explicit simulation overrides enable the requested motion feedback. The
    # site gate otherwise suppresses it whenever current Z equals target Z.
    experimental = replace(base, anti_sway_settle_max_shift=.06,
                           anti_sway_travel_damping=args.scenario == 'travel')
    target = (args.target_x, args.target_y, args.height) if args.scenario == 'travel' else (0., 0., args.height)
    settings = SwaySimulationConfig(duration_s=args.duration, initial_theta_y_deg=1., static_bias='none',
                                    hold_after_arrival=True)
    cases = {
        'pd': (replace(base, enable_anti_sway=False), settings),
        'site_current': (current_site, settings),
        'ideal': (experimental, settings),
        'empty_bias': (experimental, replace(settings, sensor_mode='inclinometer', static_bias='empty')),
        'loaded_bias': (experimental, replace(settings, sensor_mode='inclinometer', static_bias='loaded')),
        'delayed': (experimental, replace(settings, sensor_mode='inclinometer', angle_delay_s=args.delay)),
        'reversed_y': (experimental, replace(settings, sensor_mode='inclinometer', angle_scale_y=-1.)),
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results = {}
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    baseline = None
    window_end = args.duration
    for name, (config, model) in cases.items():
        history, events = run_simulation(
            target, CraneState(z0=args.height), config, verbose=False,
            max_time=args.duration + 1, sway_sim=model,
        )
        xy_events = [t for t, axis in events if axis in ('x', 'y')]
        arrival = max(xy_events) if len(xy_events) == 2 else None
        if name == 'pd':
            baseline = history
            pd_arrival = arrival
            if arrival is not None and args.scenario == 'travel':
                window_end = arrival
        result = save_simulation_result(history, config, model, args.output_dir / name)
        result['motion_comparison'] = compare_motion(history, baseline, window_end, args.allowed_regression_percent)
        result['whole_observation_comparison'] = compare_motion(history, baseline, args.duration, args.allowed_regression_percent)
        result['xy_arrival_s'] = arrival
        result['xy_arrival_delta_from_pd_s'] = arrival - pd_arrival if arrival is not None and pd_arrival is not None else None
        result['sway_not_worse_than_pd'] = (result['motion_comparison']['sway_not_worse_than_pd']
                                          and result['whole_observation_comparison']['sway_not_worse_than_pd'])
        result['comparison_complete'] = args.scenario == 'hold' or (arrival is not None and pd_arrival is not None)
        results[name] = result
        if result['first_5deg_s'] is not None:
            history = [row for row in history if row['t'] <= result['first_5deg_s']]
        t = [r['t'] for r in history]
        axes[0].plot(t, np.degrees(np.hypot([r['sim_true_theta_x'] for r in history], [r['sim_true_theta_y'] for r in history])), label=name)
        reference_times = [r['t'] for r in baseline]
        delta = [np.asarray([r[axis] for r in history]) - np.interp(t, reference_times, [r[axis] for r in baseline]) for axis in ('x', 'y')]
        axes[1].plot(t, 1000 * np.hypot(*delta), label=name)
        axes[2].plot(t, [1000 * r['antisway_dvy'] for r in history], label=name)
        comparison = result['motion_comparison']
        angle_result = (f'reached 5 deg at {result["first_5deg_s"]:.2f} s'
                        if result['first_5deg_s'] is not None
                        else f'motion RMS={comparison["rope_rms_deg"]:.3f} deg, peak={comparison["rope_peak_deg"]:.3f} deg')
        verdict = ('FAIL' if not result['sway_not_worse_than_pd'] else
                   'PASS' if result['comparison_complete'] else 'INCOMPLETE')
        print(f'{name:12s} {verdict}: {angle_result}; XY arrival={arrival}')
    for ax, label in zip(axes, ('Rope angle magnitude [deg]', 'Departure from PD [mm]', 'Requested correction Y [mm/s]')):
        ax.set_ylabel(label)
        ax.grid(alpha=.25)
        ax.axvline(window_end, color='black', ls=':', alpha=.5)
    axes[0].legend(ncol=3)
    axes[-1].set_xlabel('Time [s]')
    fig.suptitle(f'{args.scenario}: same path and initial sway; traces end when either axis reaches 5 deg\nDotted line: end of common PD-defined comparison window')
    fig.tight_layout()
    fig.savefig(args.output_dir / 'comparison.png', dpi=150)
    plt.close(fig)
    (args.output_dir / 'summary.json').write_text(json.dumps({
        'scenario': args.scenario, 'target': target,
        'note': 'Legacy continuous control after position arrival; NOT stopped residuals. '
                'Same plant/path/initial sway, disturbance off. Experimental feedback uses a 0.06 m '
                'target-height movement budget; travel mode enables travel damping. '
                'site_current retains site gating; it uses ideal sensing to isolate controller enablement. '
                'PASS requires each axis and total RMS/peak to match or improve PD in both the motion '
                'window and full observation, and both travel runs to arrive. Arrival time is reported separately. '
                'Delay/sign are assumptions, not identified site parameters. '
                'Bias profiles affect readings only, not physical load dynamics. '
                'Use first_5deg_s for divergent cases; later amplitudes are outside the small-sway comparison.',
        'results': results,
    }, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
