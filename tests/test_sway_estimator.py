import math

import pytest

from sway_estimator import AxisMapping, InclinometerSwayEstimator, SwayAxisMapping


def test_default_mapping_pitch_to_theta_x_roll_to_theta_y():
    """默认映射: pitch→θx、roll→θy（航天惯例）。"""
    est = InclinometerSwayEstimator()
    state = est.estimate(roll=0.1, pitch=0.2)
    assert state['theta_x'] == pytest.approx(0.2)  # pitch → θx
    assert state['theta_y'] == pytest.approx(0.1)  # roll → θy


def test_angle_scale_degrees_to_radians():
    est = InclinometerSwayEstimator(angle_scale=math.pi / 180.0)
    state = est.estimate(roll=90.0, pitch=180.0)
    assert state['theta_x'] == pytest.approx(math.pi)        # pitch 180° → π rad
    assert state['theta_y'] == pytest.approx(math.pi / 2.0)  # roll 90° → π/2 rad


def test_sign_flip():
    mapping = SwayAxisMapping(
        theta_x=AxisMapping('roll', -1.0),
        theta_y=AxisMapping('pitch', 1.0),
    )
    est = InclinometerSwayEstimator(mapping=mapping)
    state = est.estimate(roll=0.05, pitch=0.1)
    assert state['theta_x'] == pytest.approx(-0.05)
    assert state['theta_y'] == pytest.approx(0.1)


def test_none_angle_source_gives_zero():
    mapping = SwayAxisMapping(
        theta_x=AxisMapping('pitch', 1.0),
        theta_y=AxisMapping(None, 1.0),  # θy 无来源 → 0
    )
    est = InclinometerSwayEstimator(mapping=mapping)
    state = est.estimate(roll=0.3, pitch=0.4)
    assert state['theta_x'] == pytest.approx(0.4)
    assert state['theta_y'] == pytest.approx(0.0)


def test_none_roll_pitch_input():
    est = InclinometerSwayEstimator()
    state = est.estimate(roll=None, pitch=None)
    assert state['theta_x'] == pytest.approx(0.0)
    assert state['theta_y'] == pytest.approx(0.0)
