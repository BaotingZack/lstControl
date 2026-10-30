import math
import time

import pytest

import ros_bridge as rb
from ros_bridge import get_sway_snapshot
from sway_estimator import InclinometerSwayEstimator


def test_get_sway_snapshot_without_data():
    with rb._sway_lock:
        rb._sway_has_data = False
        rb._latest_inclination = None
        rb._last_inclination_wall = None
    snap = get_sway_snapshot()
    assert snap['valid'] is False


def test_get_sway_snapshot_with_estimator():
    est = InclinometerSwayEstimator(
        angle_scale=math.pi / 180.0,
        roll_bias=-1.159,
        pitch_bias=-0.166,
    )
    with rb._sway_lock:
        rb._sway_estimator = est
        rb._sway_has_data = True
        rb._latest_inclination = {'roll': -1.159, 'pitch': -0.166}
        rb._last_inclination_wall = time.monotonic()
    snap = get_sway_snapshot()
    assert snap['valid'] is True
    assert snap['theta_x_deg'] == pytest.approx(0.0, abs=0.01)
    assert snap['theta_y_deg'] == pytest.approx(0.0, abs=0.01)


def test_ptp_html_inclinometer_tab():
    from live_ptp_html import render_ptp_html

    html = render_ptp_html(plc_mode=True)
    assert 'drawHookSway' in html
    assert 'drawSwayCross' in html
    assert 'Hook Sway / 抓钩摆角' in html
    assert '防摇测试平台' in html
    assert '/settle/sim' in html
    assert 'ptp-dual-viz' in html
    assert 'drawCraneScene' in html
    assert 'function showCommandMsg' in html
    assert 'syncSwayFromSnapshot' in html
    assert 'tab-inclinometer' not in html
