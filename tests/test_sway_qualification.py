"""Feedback polarity needs forced motion; free oscillation alone proves nothing."""

import importlib.util
import math

import pytest

from crane_model import CraneConfig, CraneState
from sway_simulation import SwayCranePlant, SwaySimulationConfig


def qualifier():
    assert importlib.util.find_spec('sway_qualification'), 'natural-motion qualification is missing'
    from sway_qualification import SwayFeedbackQualifier
    return SwayFeedbackQualifier()


def samples(*, scale_y=1., delay=0., moving=True, dt=.05):
    cfg = CraneConfig(enable_disturbance=False)
    state = CraneState(z0=1.5)
    plant = SwayCranePlant(cfg, SwaySimulationConfig(
        sensor_mode='inclinometer', static_bias='none', angle_scale_y=scale_y,
        angle_delay_s=delay, bias_x_deg=.43, bias_y_deg=-.31,
        initial_theta_x_deg=.8, initial_theta_y_deg=-.6,
    ), state)
    for k in range(round(18 / dt)):
        t = k * dt
        angle = plant.get_sway()
        yield t, (angle['theta_x'], angle['theta_y'], state.x.position, state.y.position, 2.5, dt)
        v = (.16 if t < 3 else -.08 if t < 5 else .12 if t < 9 else 0) if moving else 0
        plant.update(state, (v, v * .8, 0), (0, 0, 1.5), dict(x=False, y=False, z=False), dt, t + dt)


@pytest.mark.parametrize('dt', [.01, .1])
def test_natural_excitation_qualifies_correct_axes_with_unknown_initial_sway_and_bias(dt):
    q = qualifier()
    for _, sample in samples(dt=dt):
        allowed = q.update(*sample)
    assert allowed == (True, True)


def test_reversed_axis_never_qualifies_while_other_axis_can():
    q = qualifier()
    results = [q.update(*s) for _, s in samples(scale_y=-1)]
    assert any(x for x, _ in results)
    assert not any(y for _, y in results)


def test_large_delay_does_not_pass_as_an_arbitrary_initial_phase():
    q = qualifier()
    assert not any(any(q.update(*s)) for _, s in samples(delay=1.))


def test_motionless_free_sway_cannot_establish_feedback_sign():
    q = qualifier()
    assert not any(any(q.update(*s)) for _, s in samples(moving=False))


def test_gap_discards_old_qualification():
    q = qualifier()
    for _, s in samples():
        q.update(*s)
    assert q.allowed == (True, True)
    assert q.update(*s[:-1], .8) == (False, False)


def test_changed_measurement_relation_revokes_permission():
    q = qualifier()
    seen = False
    for t, s in samples():
        if t > 12:
            s = (s[0], -s[1], *s[2:])
        allowed = q.update(*s)
        seen |= allowed[1]
    assert seen
    assert allowed == (True, False)


def test_nonfinite_sample_invalidates_evidence():
    q = qualifier()
    for _, s in samples():
        q.update(*s)
    assert q.update(math.nan, *s[1:]) == (False, False)
