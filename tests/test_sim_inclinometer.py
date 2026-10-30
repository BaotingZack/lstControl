import math

import pytest

from crane_model import CraneConfig, CraneState
from live_server import build_live_payload
from main import run_simulation
from sim_inclinometer import (
    build_sim_inclinometer_estimator,
    inclinometer_fields_for_sway,
    simulate_sway_history,
)


@pytest.fixture
def default_config():
    return CraneConfig(max_velocity_xy=0.2, max_velocity_z=0.2, dt=0.01)


def test_simulate_sway_moves_with_acceleration(default_config):
    history = [
        {'t': 0.0, 'x': 0, 'y': 0, 'z': 1.5, 'vx': 0.0, 'vy': 0.0, 'vz': 0.0},
        {'t': 0.1, 'x': 0, 'y': 0, 'z': 1.5, 'vx': 0.2, 'vy': 0.0, 'vz': 0.0},
        {'t': 0.2, 'x': 0, 'y': 0, 'z': 1.5, 'vx': 0.2, 'vy': 0.0, 'vz': 0.0},
    ]
    sway = simulate_sway_history(history)
    assert sway[-1]['theta_x'] != pytest.approx(0.0, abs=1e-6)


def test_build_live_payload_includes_sim_inclinometer(default_config):
    target = (3.0, 2.0, 1.2)
    history, events = run_simulation(
        target_pos=target,
        initial_state=CraneState(x0=0.0, y0=0.0, z0=1.5),
        config=default_config,
        verbose=False,
    )
    payload = build_live_payload(
        history=history,
        phase_boundaries=events,
        target_pos=target,
        initial_pos=(0.0, 0.0, 1.5),
        config=default_config,
    )
    assert payload.get('inclinometerSimulated') is True
    assert 'swayTolDeg' in payload
    frame = payload['frames'][len(payload['frames']) // 2]
    assert 'theta_x_deg' in frame
    assert 'incl_roll' in frame
    est = build_sim_inclinometer_estimator()
    back = est.estimate(frame['incl_roll'], frame['incl_pitch'])
    assert math.degrees(back['theta_x']) == pytest.approx(frame['theta_x_deg'], abs=0.05)


def test_inclinometer_fields_round_trip():
    est = build_sim_inclinometer_estimator()
    fields = inclinometer_fields_for_sway(math.radians(0.5), math.radians(-0.3), est)
    out = est.estimate(fields['incl_roll'], fields['incl_pitch'])
    assert math.degrees(out['theta_x']) == pytest.approx(fields['theta_x_deg'], abs=0.02)
