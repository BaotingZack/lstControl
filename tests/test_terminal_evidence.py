"""Offline field checks must not invent samples across recording gaps."""

import numpy as np


def recording(sensor_times):
    t = np.arange(1., 2., .1)
    return {
        '/crane/encoders': np.column_stack((t, t, t, np.ones(len(t)))),
        '/inclination/j1939_msg': np.column_stack((sensor_times, np.zeros((len(sensor_times), 3)))),
        '/localization_pose': np.column_stack((t, t, -t, np.zeros((len(t), 4)))),
    }


def test_field_qualification_never_uses_future_sensor_samples():
    from script.validate_terminal_sway import qualify_window
    result = qualify_window(recording(np.array([3.])), 1., 2.)
    assert result['valid_samples'] == 0
    assert result['skipped_samples'] == 10
    assert result['ever_qualified'] == [False, False]


def test_field_qualification_rejects_stale_sensor_samples():
    from script.validate_terminal_sway import qualify_window
    result = qualify_window(recording(np.array([0.])), 1., 2.)
    assert result['valid_samples'] == 0
    assert result['skipped_samples'] == 10


def test_field_qualification_accepts_causal_fresh_samples():
    from script.validate_terminal_sway import qualify_window
    result = qualify_window(recording(np.arange(.95, 2., .1)), 1., 2.)
    assert result['valid_samples'] == 10
    assert result['skipped_samples'] == 0
    assert result['ever_qualified'] == [False, False]  # not two pendulum periods
