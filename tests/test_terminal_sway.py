"""Terminal behavior is measured on the actual driven plant, including stopping."""

from dataclasses import replace
import math

import numpy as np
import pytest

from crane_model import CraneConfig, CraneState
from main import run_simulation
from sway_simulation import SwaySimulationConfig


def config(**kwargs):
    assert 'anti_sway_terminal_enabled' in CraneConfig.__dataclass_fields__, 'terminal policy is missing'
    return CraneConfig(enable_anti_sway=True, anti_sway_sway_gain=2.6,
        enable_disturbance=False, anti_sway_max_correction=.2,
        anti_sway_terminal_enabled=True, hook_settle_max_wait=15., **kwargs)


def run(cfg, **sensor):
    settings = SwaySimulationConfig(duration_s=45, sensor_mode='inclinometer',
        static_bias='none', initial_theta_y_deg=1., **sensor)
    return run_simulation((2., 1., 1.5), CraneState(z0=1.5), cfg,
        verbose=False, sway_sim=settings, sway_safe_height=1.5)


def post_stop_pp(history):
    stop = next(r['t'] for r in history if r['control_complete'])
    rows = [r for r in history if stop <= r['t'] <= stop + 3.2]
    return np.degrees(np.ptp([[r['sim_true_theta_x'], r['sim_true_theta_y']] for r in rows], axis=0))


def test_correct_angle_terminal_feedback_reduces_stopped_residual_on_each_axis():
    cfg = config()
    pd, _ = run(replace(cfg, enable_anti_sway=False))
    new, _ = run(cfg)
    assert np.all(post_stop_pp(new) < .4 * post_stop_pp(pd))
    assert any(r['antisway_applied_dvx'] != 0 for r in new)
    assert any(r['antisway_applied_dvy'] != 0 for r in new)
    stopped = [r for r in new if r['control_complete']]
    assert stopped and stopped[0]['terminal_outcome'] == 'settled'
    assert all(r['vx_cmd'] == r['vy_cmd'] == r['vz_cmd'] == 0 for r in stopped[1:])
    assert abs(new[-1]['x'] - 2) < cfg.arrival_pos_tol
    assert abs(new[-1]['y'] - 1) < cfg.arrival_pos_tol


def test_reversed_axis_keeps_pd_trajectory_and_does_not_disable_other_axis():
    cfg = config()
    pd, _ = run(replace(cfg, enable_anti_sway=False))
    new, _ = run(cfg, angle_scale_y=-1.)
    assert all(r['antisway_applied_dvy'] == 0 for r in new)
    assert np.allclose([r['y'] for r in new], [r['y'] for r in pd], atol=1e-12)
    assert any(abs(r['antisway_applied_dvx']) > .001 for r in new)


def test_delayed_measurement_cannot_make_either_axis_worse_than_pd():
    cfg = config()
    pd, _ = run(replace(cfg, enable_anti_sway=False))
    new, _ = run(cfg, angle_delay_s=1.)
    assert all(r['antisway_applied_dvx'] == r['antisway_applied_dvy'] == 0 for r in new)
    assert np.allclose(post_stop_pp(new), post_stop_pp(pd), atol=1e-12)


def test_feedback_is_per_axis_and_starts_in_terminal_region_with_a_ramp():
    new, _ = run(config())
    for axis, target in [('x', 2.), ('y', 1.)]:
        active = [r for r in new if abs(r[f'antisway_applied_dv{axis}']) > 1e-7]
        assert active
        assert abs(active[0][f'{axis}_measured'] - target) < .9
        assert abs(active[0][f'antisway_applied_dv{axis}']) < .002
        assert all(r[f'antisway_qualified_{axis}'] for r in active)


def test_unknown_cargo_clearance_still_forbids_equal_height_correction():
    cfg = config()
    h, _ = run_simulation((2, 1, 1.5), CraneState(z0=1.5), cfg, verbose=False,
        sway_sim=SwaySimulationConfig(duration_s=25))
    assert all(r['antisway_applied_dvx'] == r['antisway_applied_dvy'] == 0 for r in h)


def test_zero_crossing_is_not_terminal_settling():
    import importlib.util
    assert importlib.util.find_spec('sway_terminal'), 'terminal settling monitor is missing'
    from sway_terminal import TerminalSwayMonitor
    monitor = TerminalSwayMonitor()
    for k in range(200):
        t = k * .02
        settled = monitor.update(t=t, theta=(math.radians(.7) * math.sin(2*math.pi*t/3.2), 0),
            error=(0., 0.), velocity=(0., 0.), period=3.2,
            angle_tol=math.radians(.3), pos_tol=.02, vel_tol=.01)
        assert not settled


def test_quiet_sway_but_wrong_position_is_not_settled():
    from sway_terminal import TerminalSwayMonitor
    monitor = TerminalSwayMonitor()
    for k in range(200):
        assert not monitor.update(t=k*.02, theta=(0., 0.), error=(.04, 0.),
            velocity=(0., 0.), period=3.2, angle_tol=math.radians(.3), pos_tol=.02, vel_tol=.01)


def test_fixed_duration_and_real_completion_have_the_same_control_until_stop():
    from crane_model import PlantActuator, SimPositionSource, run_pd_control
    from sway_simulation import SwayCranePlant
    cfg = config()
    fixed, _ = run(cfg)
    state = CraneState(z0=1.5)
    plant = SwayCranePlant(cfg, SwaySimulationConfig(sensor_mode='inclinometer', static_bias='none'), state)
    live, _ = run_pd_control(SimPositionSource(plant, state, cfg), PlantActuator(plant, state, cfg),
        cfg, (2, 1, 1.5), state, verbose=False, sway_source=plant, sway_safe_height=1.5)
    first_stop = next(i for i, r in enumerate(fixed) if r['control_complete'])
    assert len(live) == first_stop + 1
    assert np.allclose([[r['vx_cmd'],r['vy_cmd']] for r in live],
                       [[r['vx_cmd'],r['vy_cmd']] for r in fixed[:len(live)]])


def test_invalid_angle_does_not_poison_runtime_observations():
    from anti_sway_runtime import AntiSwayRuntime
    from sway_controller import AntiSwayController, RopeLengthModel
    rt = AntiSwayRuntime(AntiSwayController(RopeLengthModel(4.), sway_gain=2.6), terminal_mode=True)
    for angle in (0., float('nan'), .02):
        vx, vy, _, _ = rt.merge_xy_commands(.1, -.1, theta_x=angle, theta_y=angle,
            x=0., y=0., z_grab=1.5, locked_x=False, locked_y=False, dt=.1, max_velocity_xy=.2)
        assert (vx, vy) == (.1, -.1)
    assert math.isfinite(rt.dc.dc_x) and math.isfinite(rt.dc.dc_y)


def test_residual_metrics_cannot_accept_an_incomplete_post_stop_period():
    import sway_simulation
    assert hasattr(sway_simulation, 'terminal_metrics'), 'stopped residual metrics are missing'
    cfg = config()
    history, events = run(cfg)
    stop = next(r['t'] for r in history if r['control_complete'])
    short = [r for r in history if r['t'] < stop + 1.]
    result = sway_simulation.terminal_metrics(short, events, cfg)
    assert result['post_stop_pp_deg'] is None
    assert result['stable_ready_s'] is None


def test_tolerated_bad_position_frames_discard_angle_qualification():
    from crane_model import run_pd_control
    cfg = config(dt=.1)
    good, _ = run(cfg)

    class Source:
        def __init__(self):
            self.i = 0
        def reset(self):
            pass
        def get_position(self):
            row = good[self.i]
            self.i += 1
            self.row = row
            return dict(x=float('nan') if 12 < row['t'] < 12.65 else row['x_measured'],
                y=row['y_measured'], z=row['z_measured'], vx=row['vx_filtered'],
                vy=row['vy_filtered'], vz=0., dt=.1, t=row['t'], stamp=self.i)
        def get_sway(self):
            return dict(theta_x=self.row['theta_x'], theta_y=self.row['theta_y'])

    class Actuator:
        def apply(self, *args):
            pass
        def update_state(self, state, pos):
            for a in 'xyz':
                getattr(state, a).position = pos[a]
                getattr(state, a).velocity = pos['v'+a]
        def stop_motion(self):
            pass
        def emergency_stop(self):
            pass
        def cleanup(self):
            pass

    source = Source()
    history, _ = run_pd_control(source, Actuator(), cfg, (2,1,1.5), CraneState(z0=1.5),
        verbose=False, is_simulation=False, sway_source=source, sway_safe_height=1.5)
    before = [r for r in history if r['t'] < 12.]
    assert any(r['antisway_qualified_y'] for r in before)
    resumed = next(r for r in history if r['t'] > 12.65)
    assert not resumed['antisway_qualified_y']
    assert resumed['antisway_applied_dvy'] == 0.


@pytest.mark.parametrize('position_jump, speed', [(.035, 0.), (.016, .04)])
def test_feedback_loss_after_lock_does_not_complete_outside_position_tolerance(position_jump, speed):
    from crane_model import PlantActuator, SimPositionSource, run_pd_control
    from sway_simulation import SwayCranePlant
    cfg = config(dt=.1)
    state = CraneState(z0=1.5)
    plant = SwayCranePlant(cfg, SwaySimulationConfig(static_bias='none'), state)
    source = SimPositionSource(plant, state, cfg)

    class Hooks:
        axes = set()
        def should_stop(self):
            return False
        def on_arrival(self, axis, t):
            self.axes.add(axis)
        def on_step(self, row):
            if 'x' in self.axes and 'y' in self.axes and not getattr(self, 'disturbed', False):
                source._state.y.position = 1. + position_jump
                source._state.y.velocity = speed
                self.disturbed = True

    hooks = Hooks()
    class Sensor:
        def get_sway(self):
            return None if getattr(hooks, 'disturbed', False) else plant.get_sway()

    h, _ = run_pd_control(source, PlantActuator(plant,state,cfg), cfg, (2,1,1.5), state,
        verbose=False, hooks=hooks, sway_source=Sensor(), sway_safe_height=1.5, simulation_duration=40.)
    assert hooks.disturbed
    assert abs(h[-1]['y'] - 1) <= cfg.arrival_pos_tol
    done = next(r for r in h if r['control_complete'])
    assert abs(done['y'] - 1) <= cfg.arrival_pos_tol


def test_real_mode_recovery_checks_measurement_not_snapped_arrival_display():
    from crane_model import Actuator, ControlHooks, run_pd_control
    cfg = config(dt=.1, arrival_capture_pos_tol=.04)
    recorded, _ = run(cfg)

    class Hooks(ControlHooks):
        def __init__(self):
            self.axes = set()
        def on_arrival(self, axis, t):
            self.axes.add(axis)

    hooks = Hooks()

    class Source:
        def __init__(self):
            self.i = self.loss_frames = 0
        def reset(self):
            pass
        def get_position(self):
            self.row = row = recorded[self.i]
            self.i += 1
            lost = 'x' in hooks.axes and 'y' in hooks.axes
            if lost:
                self.loss_frames += 1
            # Within the broad capture window but outside final tolerance for
            # long enough to relock. Only later samples actually meet tolerance.
            y = (1.03 if self.loss_frames <= 6 else 1.005) if lost else row['y_measured']
            return dict(x=row['x_measured'], y=y, z=1.5, vx=row['vx_filtered'],
                        vy=0. if lost else row['vy_filtered'], vz=0.,
                        dt=.1, t=row['t'], stamp=self.i)
        def get_sway(self):
            if self.loss_frames:
                return None
            return dict(theta_x=self.row['theta_x'], theta_y=self.row['theta_y'])

    class RecordedActuator(Actuator):
        def apply(self, *args):
            pass
        def update_state(self, state, pos):
            for a in 'xyz':
                getattr(state, a).position = pos[a]
                getattr(state, a).velocity = pos['v'+a]

    source = Source()
    history, _ = run_pd_control(source, RecordedActuator(), cfg, (2,1,1.5), CraneState(z0=1.5),
        hooks=hooks, verbose=False, is_simulation=False, sway_source=source, sway_safe_height=1.5)
    assert any(abs(r['antisway_applied_dvy']) > .001 for r in history)
    done = next(r for r in history if r['control_complete'])
    assert abs(done['y_measured'] - 1.) <= cfg.arrival_pos_tol
