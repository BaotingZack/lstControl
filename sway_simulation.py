"""Optional pendulum truth and gripper-mounted angle sensitivity experiments.

This is a driven pendulum plus a measurement transfer model, not an identified
double pendulum. Scale, filter and delay parameters are hypotheses. Static B2
readings come from lst_data/记录.docx; see doc/inclinometer_diagnosis_0918.md.
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

from crane_model import CraneConfig, CranePlant, CraneState
from sway_controller import RopeLengthModel
from sway_settle import GRAVITY


@dataclass(frozen=True)
class SwaySimulationConfig:
    sensor_mode: str = 'ideal'
    duration_s: float = 30.0
    initial_theta_x_deg: float = 0.0
    initial_theta_y_deg: float = 1.0
    damping_ratio: float = .015
    static_bias: str = 'empty'  # none | empty | loaded; affects measurement only
    angle_scale_x: float = 1.0
    angle_scale_y: float = 1.0
    angle_delay_s: float = 0.0
    angle_tau_s: float = 0.0
    bias_x_deg: float = 0.0  # additional residual, in crane axes
    bias_y_deg: float = 0.0
    hold_after_arrival: bool = False  # diagnostic only; normal runs stop, then observe coast/free sway

    def __post_init__(self):
        if self.sensor_mode not in ('ideal', 'inclinometer'):
            raise ValueError('sensor_mode must be ideal or inclinometer')
        if self.static_bias not in ('none', 'empty', 'loaded'):
            raise ValueError('static_bias must be none, empty or loaded')
        for name, value in asdict(self).items():
            if isinstance(value, (float, int)) and not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if self.duration_s <= 0:
            raise ValueError('duration_s must be positive')
        for name in ('angle_delay_s', 'angle_tau_s', 'damping_ratio'):
            if getattr(self, name) < 0:
                raise ValueError(f'{name} must be non-negative')


# (height m, raw roll deg, raw pitch deg), means of repeated B2 observations.
# Endpoint values are held outside the measured height interval, not extrapolated.
STATIC_B2 = {
    'empty': ((.36, -1.2166666667, -.115), (1., -1.139, -.228), (1.5, -1.101, -.258)),
    'loaded': ((1., -.7085, -.4915), (1.5, -.6743333333, -.4833333333)),
}


def _static_reading(profile: str, z: float) -> tuple[float, float]:
    points = STATIC_B2[profile]
    if z <= points[0][0]:
        return points[0][1:]
    for lower, upper in zip(points, points[1:]):
        if z <= upper[0]:
            fraction = (z - lower[0]) / (upper[0] - lower[0])
            return tuple(lower[k] + fraction * (upper[k] - lower[k]) for k in (1, 2))
    return points[-1][1:]


class SimulatedInclinometer:
    """Separate measurement channel; cannot modify the physical pendulum state.

Angles use crane axes/radians after the installed pitch→-X, roll→Y mapping.
The loaded profile uses the configured closed-gripper calibration. Loading does
not change pendulum mass/inertia in this model. Delay includes sensor/transport
delay only; the existing plant still models the velocity servo separately.
"""

    def __init__(self, config: CraneConfig, settings: SwaySimulationConfig):
        self.config, self.settings = config, settings
        self._history = deque()
        self._filtered = None
        self._time = None

    def update(self, t: float, theta_x: float, theta_y: float, z: float) -> dict:
        s = self.settings
        if s.sensor_mode == 'ideal':
            return {'theta_x': theta_x, 'theta_y': theta_y}
        bx, by = math.radians(s.bias_x_deg), math.radians(s.bias_y_deg)
        if s.static_bias != 'none':
            roll, pitch = _static_reading(s.static_bias, z)
            rb, pb = self.config.anti_sway_roll_bias, self.config.anti_sway_pitch_bias
            if s.static_bias == 'loaded':
                if self.config.anti_sway_roll_bias_closed is not None:
                    rb = self.config.anti_sway_roll_bias_closed
                if self.config.anti_sway_pitch_bias_closed is not None:
                    pb = self.config.anti_sway_pitch_bias_closed
            bx -= math.radians(pitch - pb)
            by += math.radians(roll - rb)
        values = (s.angle_scale_x * theta_x + bx, s.angle_scale_y * theta_y + by)
        if self._filtered is None:
            self._filtered = values
        else:
            dt = max(0., t - self._time)
            alpha = math.exp(-dt / s.angle_tau_s) if s.angle_tau_s > 0 else 0.
            self._filtered = tuple(alpha * old + (1 - alpha) * new
                                   for old, new in zip(self._filtered, values))
        self._time = t
        self._history.append((t, *self._filtered))
        query = t - s.angle_delay_s
        while len(self._history) > 1 and self._history[1][0] <= query:
            self._history.popleft()
        first = self._history[0]
        measured = first[1:]
        if len(self._history) > 1 and query > first[0]:
            second = self._history[1]
            fraction = (query - first[0]) / (second[0] - first[0])
            measured = tuple(first[k] + fraction * (second[k] - first[k]) for k in (1, 2))
        return dict(zip(('theta_x', 'theta_y'), measured))


class SwayCranePlant(CranePlant):
    """Two small-sway planes driven by support acceleration and changing length.

    x/y remain trolley coordinates (the existing position-source contract).
    Payload positions are separate telemetry. Arrival is a controller event;
    it never teleports the support or cancels physical pendulum motion.
    """

    snap_arrivals = False

    def __init__(self, config: CraneConfig, settings: SwaySimulationConfig, state: CraneState):
        super().__init__(config)
        self.settings = settings
        self.rope_model = RopeLengthModel(
            config.rope_length_sheave_height, config.rope_length_grab_offset,
            config.rope_length_cable_stretch, config.rope_length_min,
        )
        self.theta = [math.radians(settings.initial_theta_x_deg), math.radians(settings.initial_theta_y_deg)]
        self.omega = [0., 0.]
        self.sensor = SimulatedInclinometer(config, settings)
        self.measured = self.sensor.update(0., *self.theta, state.z.position)

    def get_sway(self) -> dict:
        return dict(self.measured)

    def update(self, state, commands, targets, locked, dt, t):
        old_velocity = (state.x.velocity, state.y.velocity)
        old_length = self.rope_model.compute(state.z.position)
        # Locked axes still execute the actual command and retain servo inertia.
        disturbances = super().update(state, commands, targets, dict(x=False, y=False, z=False), dt, t)
        new_length = self.rope_model.compute(state.z.position)
        length_rate = (new_length - old_length) / dt
        acceleration = ((state.x.velocity - old_velocity[0]) / dt,
                        (state.y.velocity - old_velocity[1]) / dt)
        steps = max(1, math.ceil(dt / .01))
        h = dt / steps
        for step in range(steps):
            length = old_length + (step + .5) / steps * (new_length - old_length)
            damping = 2 * length_rate / length + 2 * self.settings.damping_ratio * math.sqrt(GRAVITY / length)
            for axis in (0, 1):
                def derivative(angle, rate):
                    return rate, (-GRAVITY * math.sin(angle) - acceleration[axis] * math.cos(angle)) / length - damping * rate

                angle, rate = self.theta[axis], self.omega[axis]
                a1, r1 = derivative(angle, rate)
                a2, r2 = derivative(angle + h * a1 / 2, rate + h * r1 / 2)
                a3, r3 = derivative(angle + h * a2 / 2, rate + h * r2 / 2)
                a4, r4 = derivative(angle + h * a3, rate + h * r3)
                self.theta[axis] += h * (a1 + 2 * a2 + 2 * a3 + a4) / 6
                self.omega[axis] += h * (r1 + 2 * r2 + 2 * r3 + r4) / 6
        self.measured = self.sensor.update(t, *self.theta, state.z.position)
        return disturbances

    def sway_snapshot(self, state: CraneState) -> dict:
        length = self.rope_model.compute(state.z.position)
        return {
            'sim_true_theta_x': self.theta[0], 'sim_true_theta_y': self.theta[1],
            'sim_measured_theta_x': self.measured['theta_x'],
            'sim_measured_theta_y': self.measured['theta_y'],
            'sim_payload_x': state.x.position + length * math.sin(self.theta[0]),
            'sim_payload_y': state.y.position + length * math.sin(self.theta[1]),
            'sim_rope_length': length,
            'sim_sway_energy': sum(.5 * length**2 * rate**2 + GRAVITY * length * (1 - math.cos(angle))
                                   for angle, rate in zip(self.theta, self.omega)),
        }


def simulation_metrics(history: list[dict]) -> dict:
    import numpy as np

    tail = [row for row in history if row['t'] >= history[-1]['t'] - 5.]
    return {
        'duration_s': history[-1]['t'],
        'first_5deg_s': next((row['t'] for row in history
                             if max(abs(row[f'sim_true_theta_{a}']) for a in ('x', 'y')) >= math.radians(5.)), None),
        'tail_true_pp_deg': [float(np.degrees(np.ptp([row[f'sim_true_theta_{a}'] for row in tail]))) for a in ('x', 'y')],
        'tail_measured_pp_deg': [float(np.degrees(np.ptp([row[f'sim_measured_theta_{a}'] for row in tail]))) for a in ('x', 'y')],
        'peak_trolley_error_m': [max(abs(row[a] - row[f'p_ref_{a}']) for row in history) for a in ('x', 'y')],
        'tail_payload_error_m': [max(abs(row[f'sim_payload_{a}'] - row[f'p_ref_{a}']) for row in tail) for a in ('x', 'y')],
        'requested_correction_rms_m_s': [float(np.sqrt(np.mean([row[f'antisway_dv{a}']**2 for row in history]))) for a in ('x', 'y')],
    }


def terminal_metrics(history, events, config):
    """Truth-only offline scoring. A full stopped period is mandatory.

    stable_ready_s is the end of a *post-stop* full-period window satisfying
    angle PP, position and velocity, with no later violation in this record.
    This is stricter than the online settled decision made before stopping.
    """
    import numpy as np
    from sway_terminal import TerminalSwayMonitor

    xy = [t for t, axis in events if axis in ('x', 'y')]
    first = next((r for r in history if r.get('control_complete')), None)
    result = dict(xy_arrival_s=max(xy) if len(xy) == 2 else None,
                  control_stop_s=first['t'] if first else None,
                  terminal_outcome=first['terminal_outcome'] if first else 'incomplete',
                  post_stop_pp_deg=None, post_stop_three_period_pp_deg=None,
                  stable_ready_s=None,
                  final_error_m=[history[-1][a] - history[-1]['p_ref_' + a] for a in 'xy'],
                  first_feedback_s=[next((r['t'] for r in history if abs(r['antisway_applied_dv'+a]) > 1e-8), None) for a in 'xy'],
                  route_peak_deg=[float(np.degrees(max(abs(r['sim_true_theta_'+a]) for r in history))) for a in 'xy'])
    if first is None:
        return result
    period = 2 * math.pi * math.sqrt(first['sim_rope_length'] / GRAVITY)
    result['stop_period_s'] = period
    for periods, key in ((1, 'post_stop_pp_deg'), (3, 'post_stop_three_period_pp_deg')):
        end = first['t'] + periods * period
        if history[-1]['t'] >= end:
            rows = [r for r in history if first['t'] <= r['t'] <= end]
            result[key] = [float(np.degrees(np.ptp([r['sim_true_theta_'+a] for r in rows]))) for a in 'xy']
    monitor = TerminalSwayMonitor()
    stable_start = None
    for row in history:
        if row['t'] < first['t']:
            continue
        good = monitor.update(t=row['t'], theta=tuple(row['sim_true_theta_'+a] for a in 'xy'),
            error=tuple(row[a] - row['p_ref_'+a] for a in 'xy'),
            velocity=tuple(row['v'+a] for a in 'xy'), period=period,
            angle_tol=math.radians(config.hook_sway_angle_tol_deg),
            pos_tol=config.arrival_pos_tol, vel_tol=config.velocity_deadband)
        if not good:
            stable_start = None
        elif stable_start is None:
            stable_start = row['t']
    result['stable_ready_s'] = stable_start
    return result


def save_simulation_result(history, config, settings, output_prefix):
    """Export independently inspectable truth, measurement and command histories."""
    import matplotlib.pyplot as plt
    import numpy as np

    prefix = Path(output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    metrics = simulation_metrics(history)
    prefix.with_suffix('.json').write_text(json.dumps({
        'model': 'driven pendulum + assumed measurement transfer (not field identification)',
        'simulation': asdict(settings), 'controller': asdict(config),
        'metrics': metrics, 'history': history,
    }, ensure_ascii=False, indent=2), encoding='utf-8')
    if metrics['first_5deg_s'] is not None:
        history = [row for row in history if row['t'] <= metrics['first_5deg_s']]
    t = [row['t'] for row in history]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for a, color in (('x', 'tab:blue'), ('y', 'tab:orange')):
        axes[0].plot(t, np.degrees([r[f'sim_true_theta_{a}'] for r in history]), color=color, label=f'{a.upper()} rope truth')
        axes[0].plot(t, np.degrees([r[f'sim_measured_theta_{a}'] for r in history]), color=color, ls='--', alpha=.65, label=f'{a.upper()} measured')
        axes[1].plot(t, [1000 * (r[a] - r[f'p_ref_{a}']) for r in history], color=color, label=f'{a.upper()} trolley')
        axes[1].plot(t, [1000 * (r[f'sim_payload_{a}'] - r[f'p_ref_{a}']) for r in history], color=color, ls='--', label=f'{a.upper()} payload')
        axes[2].plot(t, [1000 * r[f'antisway_dv{a}'] for r in history], color=color, label=f'{a.upper()} correction')
    for ax, label in zip(axes, ('Angle [deg]', 'Target error [mm]', 'Requested correction [mm/s]')):
        ax.set_ylabel(label)
        ax.grid(alpha=.25)
        ax.legend(loc='upper right', ncol=2)
    axes[-1].set_xlabel('Time [s]')
    title = f'Sway sensitivity simulation: {settings.sensor_mode}; feedback={config.enable_anti_sway}'
    if metrics['first_5deg_s'] is not None:
        title += '\nTrace ends at first 5 deg rope angle; later amplitude is outside the small-sway comparison'
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(prefix.with_suffix('.png'), dpi=150)
    plt.close(fig)
    return metrics
