"""Physical checks for the optional sway plant, not controller implementation mirrors."""

import math

import numpy as np
import pytest

from crane_model import CraneConfig, CraneState
from main import _build_arg_parser, _config_from_args, run_simulation


def _classes():
    # Missing optional model is reported as a feature failure, not a collection error.
    import importlib.util

    assert importlib.util.find_spec('sway_simulation') is not None, 'sway simulator is missing'
    from sway_simulation import SwayCranePlant, SwaySimulationConfig, SimulatedInclinometer
    return SwayCranePlant, SwaySimulationConfig, SimulatedInclinometer


def test_free_pendulum_period_and_energy():
    Plant, Settings, _ = _classes()
    cfg = CraneConfig(enable_disturbance=False)
    settings = Settings(initial_theta_y_deg=1.0, damping_ratio=0.0)
    state = CraneState(z0=1.5)
    plant = Plant(cfg, settings, state)
    angles, energies = [], []
    for i in range(1000):
        plant.update(state, (0, 0, 0), (0, 0, 1.5), dict(x=True, y=True, z=True), .01, (i + 1) * .01)
        snap = plant.sway_snapshot(state)
        angles.append(snap['sim_true_theta_y'])
        energies.append(snap['sim_sway_energy'])
    # L=2.5 m: first downward zero crossing is T/4 = 0.793 s.
    crossing = np.flatnonzero(np.asarray(angles) < 0)[0]
    assert (crossing + 1) * .01 == pytest.approx(.793, abs=.015)
    assert max(energies) / min(energies) < 1.001


def test_locked_sway_plant_executes_correction_without_teleporting():
    Plant, Settings, _ = _classes()
    cfg = CraneConfig(enable_disturbance=False)
    state = CraneState(x0=.1, z0=1.5)
    plant = Plant(cfg, Settings(), state)
    plant.update(state, (.03, 0, 0), (0, 0, 1.5), dict(x=True, y=True, z=True), .1, .1)
    assert state.x.position > .1
    assert state.x.velocity > 0


def test_measurement_delay_is_causal_and_does_not_change_truth():
    _, Settings, Sensor = _classes()
    settings = Settings(sensor_mode='inclinometer', static_bias='none', angle_delay_s=.2)
    sensor = Sensor(CraneConfig(), settings)
    for i in range(11):
        reading = sensor.update(i * .1, 0.0, i * .01, 1.5)
    assert reading['theta_y'] == pytest.approx(.08)


def test_loaded_static_readings_are_not_zero_rope_angle():
    _, Settings, Sensor = _classes()
    cfg = CraneConfig(
        anti_sway_roll_bias=-1.159, anti_sway_pitch_bias=-.166,
        anti_sway_roll_bias_closed=-1.139, anti_sway_pitch_bias_closed=-.228,
    )
    sensor = Sensor(cfg, Settings(sensor_mode='inclinometer', static_bias='loaded'))
    reading = sensor.update(0, 0, 0, 1.0)
    # B2 loaded mean roll=-.7085, pitch=-.4915 deg; theta_x=-pitch, theta_y=roll.
    assert math.degrees(reading['theta_y']) == pytest.approx(.4305)
    assert math.degrees(reading['theta_x']) == pytest.approx(.2635)


def test_identical_hold_experiment_distinguishes_damping_from_amplification():
    _, Settings, _ = _classes()
    from dataclasses import replace

    cfg = CraneConfig(
        enable_disturbance=False, enable_anti_sway=True, anti_sway_sway_gain=2.6,
        anti_sway_settle_max_shift=.08, anti_sway_locked_max_shift=.08,
        anti_sway_max_correction=.15, anti_sway_hold_recenter_gain=.3,
        anti_sway_terminal_enabled=False,  # deliberately exercise the legacy feedback law
    )
    settings = Settings(duration_s=20, initial_theta_y_deg=.8, static_bias='none', hold_after_arrival=True)
    histories = []
    for enabled, scale in ((False, 1), (True, 1), (True, -1)):
        history, _ = run_simulation(
            (0, 0, 1.5), CraneState(z0=1.5), replace(cfg, enable_anti_sway=enabled),
            verbose=False, sway_sim=replace(settings, sensor_mode='inclinometer', angle_scale_y=scale),
        )
        assert history[-1]['t'] == pytest.approx(20, abs=.015)
        histories.append(history)
    residual = [max(abs(row['sim_true_theta_y']) for row in h if row['t'] > 15) for h in histories]
    assert residual[1] < residual[0] * .6
    assert residual[2] > residual[0] * 1.5
    assert all(row['antisway_dvy'] == 0 for row in histories[0])


def test_motion_loop_withdraws_growing_axis_and_records_pd_baseline():
    _, Settings, _ = _classes()
    cfg = CraneConfig(
        enable_disturbance=False, enable_anti_sway=True, anti_sway_sway_gain=2.6,
        anti_sway_settle_max_shift=.06, anti_sway_locked_max_shift=.06,
        anti_sway_max_correction=.15,
        anti_sway_terminal_enabled=False,
    )
    settings = Settings(duration_s=20, sensor_mode='inclinometer', static_bias='none', angle_scale_y=-1.,
                        hold_after_arrival=True)
    history, _ = run_simulation((0., 0., 1.5), CraneState(z0=1.5), cfg, verbose=False, sway_sim=settings)
    stopped = [r for r in history if r.get('antisway_blocked_y')]
    assert stopped, '运动控制循环也必须撤回持续放大的角度反馈'
    assert all(r['vy_cmd'] == r['vy_pd'] for r in stopped)
    assert all(r['antisway_applied_dvy'] == 0. for r in stopped)
    assert not any(r['antisway_blocked_x'] for r in history)


@pytest.mark.parametrize('flag', ['--no-anti-sway', '--no-anti'])
def test_cli_can_disable_site_enabled_feedback_for_a_fair_pd_baseline(flag):
    parser = _build_arg_parser()
    argv = [flag]
    args = parser.parse_args(argv)
    cfg = _config_from_args(args, {'enable_anti_sway': True, 'anti_sway_sway_gain': 2.6}, argv)
    assert cfg.enable_anti_sway is False


@pytest.mark.parametrize('kwargs', [dict(angle_delay_s=-.1), dict(duration_s=0), dict(angle_scale_y=float('nan'))])
def test_invalid_simulation_parameters_rejected(kwargs):
    _, Settings, _ = _classes()
    with pytest.raises(ValueError):
        Settings(**kwargs)


def test_cli_fixed_duration_can_exceed_legacy_arrival_timeout(tmp_path, monkeypatch):
    import json
    from main import main

    monkeypatch.chdir(tmp_path)
    main(['--sim-sway', 'ideal', '--sim-duration', '181', '--no-anti-sway',
          '--target-x', '0', '--target-y', '0', '--target-z', '1.5',
          '--sim-output', str(tmp_path / 'long_hold')])
    result = json.loads((tmp_path / 'long_hold.json').read_text())
    assert result['metrics']['duration_s'] == pytest.approx(181, abs=.015)


def test_comparison_enables_gain_when_site_feedback_is_disabled(tmp_path, monkeypatch):
    import json
    import sys
    from script.compare_sway_sim import main

    cfg = tmp_path / 'disabled.json'
    cfg.write_text(json.dumps({'enable_anti_sway': False, 'anti_sway_sway_gain': 2.6,
                               'anti_sway_travel_damping': True, 'anti_sway_settle_max_shift': .06}))
    output = tmp_path / 'comparison'
    monkeypatch.setattr(sys, 'argv', ['compare', '--config', str(cfg), '--duration', '2', '--output-dir', str(output)])
    main()
    result = json.loads((output / 'ideal.json').read_text())
    assert max(abs(row['antisway_dvy']) for row in result['history']) > .001
    current = json.loads((output / 'site_current.json').read_text())
    assert all(row['antisway_dvy'] == 0 for row in current['history'])


def test_simulation_flags_rejected_before_a_plc_connection():
    from main import main

    with pytest.raises(SystemExit) as error:
        main(['--plc-ip', '192.0.2.1', '--sim-sway', 'ideal'])
    assert error.value.code == 2


@pytest.mark.parametrize('angles, passes', [([.5, .5, .5], True), ([1, 1, 1], True), ([0, 0, 1.1], False)])
def test_motion_comparison_requires_both_rms_and_peak_not_to_regress(angles, passes):
    from script import compare_sway_sim

    assert hasattr(compare_sway_sim, 'compare_motion'), 'motion non-regression check is missing'
    baseline = [dict(t=i + 1., sim_true_theta_x=0., sim_true_theta_y=math.radians(1.), x=i, y=0.) for i in range(3)]
    candidate = [dict(row, sim_true_theta_y=math.radians(angle)) for row, angle in zip(baseline, angles)]
    metrics = compare_sway_sim.compare_motion(candidate, baseline, 3.)
    assert metrics['sway_not_worse_than_pd'] is passes


def test_motion_comparison_does_not_hide_a_worse_axis_in_a_lower_total():
    from script.compare_sway_sim import compare_motion

    baseline = [dict(t=1., sim_true_theta_x=math.radians(1.), sim_true_theta_y=math.radians(1.), x=0., y=0.)]
    candidate = [dict(baseline[0], sim_true_theta_x=math.radians(.1), sim_true_theta_y=math.radians(1.2))]
    metrics = compare_motion(candidate, baseline, 1.)
    assert metrics['sway_not_worse_than_pd'] is False
