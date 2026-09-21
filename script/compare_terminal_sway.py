#!/usr/bin/env python3
"""Compare closed-loop terminal policies, then observe the genuinely stopped plant.

No recorded angles are changed and no PLC is connected. All angle transfer,
delay, damping and actuator assumptions are listed in the exported JSON.
"""

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from crane_model import CraneState
from main import _build_arg_parser, _config_from_args, run_simulation
from site_config import load_site_config
from sway_simulation import SwaySimulationConfig, terminal_metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=REPO / 'artifacts/inclinometer_0918/terminal')
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--dt', type=float, default=.1, help='default 10 Hz, matching the site position loop')
    args = parser.parse_args()
    config = replace(_config_from_args(_build_arg_parser().parse_args([]), load_site_config(), []),
                     dt=args.dt, enable_disturbance=False)
    scenarios = {
        'ideal': {}, 'empty_bias': dict(static_bias='empty'), 'loaded_bias': dict(static_bias='loaded'),
        'reversed_y': dict(angle_scale_y=-1.), 'delay_1s': dict(angle_delay_s=1.),
        'delay_200ms': dict(angle_delay_s=.2), 'filter_500ms': dict(angle_tau_s=.5),
        'variable_height': {}, 'low_natural_damping': dict(damping_ratio=0.),
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results = {}
    for name, values in scenarios.items():
        settings = replace(SwaySimulationConfig(sensor_mode='inclinometer', static_bias='none',
                                                duration_s=args.duration), **values)
        start_z = .8 if name == 'variable_height' else 1.5
        target = (2., 1., 1.5)
        policies = {
            'pd': replace(config, enable_anti_sway=False),
            # Same corrected runtime without prequalification; earlier legacy
            # guard/DC fixes remain. Not falsely labelled as exact ffe67ac.
            'previous_approach': replace(config, anti_sway_terminal_enabled=False),
            'previous_full': replace(config, anti_sway_terminal_enabled=False, anti_sway_travel_damping=True),
            'terminal': config,
        }
        fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        report = {}
        baseline = None
        for label, cfg in policies.items():
            history, events = run_simulation(target, CraneState(z0=start_z), cfg, verbose=False,
                sway_sim=settings, sway_safe_height=target[2], max_time=args.duration + 1)
            metrics = terminal_metrics(history, events, cfg)
            if label == 'pd':
                baseline = history
            # Equal absolute time after BOTH controllers have stopped prevents
            # natural decay during extra waiting from masquerading as damping.
            own_stop = metrics['control_stop_s']
            pd_stop = next((r['t'] for r in baseline if r['control_complete']), None)
            comparison = None
            if own_stop is not None and pd_stop is not None:
                start = max(own_stop, pd_stop)
                end = start + 3 * metrics['stop_period_s']
                if history[-1]['t'] >= end:
                    def pp(rows):
                        window = [r for r in rows if start <= r['t'] <= end]
                        return [float(np.degrees(np.ptp([r['sim_true_theta_'+a] for r in window]))) for a in 'xy']
                    candidate, reference = pp(history), pp(baseline)
                    comparison = dict(start_s=start, end_s=end, pp_deg=candidate, pd_pp_deg=reference,
                                      per_axis_not_worse=[c <= b + 1e-8 for c, b in zip(candidate, reference)])
            metrics['common_stopped_window'] = comparison
            metrics['position_in_tolerance'] = all(abs(v) <= cfg.arrival_pos_tol for v in metrics['final_error_m'])
            report[label] = metrics
            (args.output_dir / f'{name}_{label}.json').write_text(json.dumps(dict(
                model='driven pendulum, assumed sensor transfer; not field identification',
                simulation=asdict(settings), controller=asdict(cfg), metrics=metrics,
                history=history), indent=2), encoding='utf-8')
            first_large = next((r['t'] for r in history if max(abs(r['sim_true_theta_'+a]) for a in 'xy') >= np.radians(5)), args.duration)
            shown = [r for r in history if r['t'] <= first_large]
            times = [r['t'] for r in shown]
            for a, ax in zip('xy', axes[:2]):
                line, = ax.plot(times, np.degrees([r['sim_true_theta_'+a] for r in shown]), label=label)
                if own_stop is not None and own_stop <= first_large:
                    ax.axvline(own_stop, color=line.get_color(), ls=':', alpha=.4)
            axes[2].plot(times, [1000*np.hypot(r['x']-2, r['y']-1) for r in shown], label=label)
            print(name, label, json.dumps(metrics, ensure_ascii=False), flush=True)
        for ax, ylabel in zip(axes, ('Bridge X true angle [deg]', 'Trolley Y true angle [deg]', 'XY position error [mm]')):
            ax.set_ylabel(ylabel)
            ax.grid(alpha=.25)
        axes[0].legend(ncol=2)
        axes[2].set_yscale('symlog', linthresh=20)
        axes[-1].set_xlabel('Time [s]; dotted lines mark controller stop')
        fig.suptitle(f'{name}: terminal versus PD/previous feedback; curves stop at first 5 deg')
        fig.tight_layout()
        fig.savefig(args.output_dir / f'{name}.png', dpi=150)
        plt.close(fig)
        results[name] = report
    (args.output_dir / 'summary.json').write_text(json.dumps(dict(
        note='All controllers stop before residual scoring; no angle teleports. '
             'Common stopped windows compare equal absolute times. Stable-ready requires '
             'one full post-stop cycle within PP, position and speed tolerances and no later '
             'violation during observation. None means not observed, not zero. '
             'previous_* are prequalification-off versions of the corrected runtime, not exact historical source. '
             'All runs share clearance and plant; no assumptions are claimed as field identification.',
        results=results), indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
