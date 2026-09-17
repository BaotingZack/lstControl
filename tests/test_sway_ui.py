import math
import time

import pytest

import ros_bridge as rb
from crane_model import CraneConfig
from live_server import _enrich_control_state_payload, render_live_html
from ros_bridge import configure_sway_live_analyzer, get_sway_snapshot
from sway_estimator import InclinometerSwayEstimator


class _FakeServer:
    def __init__(self, config):
        self.config = config


def test_get_sway_snapshot_without_data():
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


def test_enrich_control_state_payload():
    configure_sway_live_analyzer(tol_deg=0.3)
    with rb._sway_lock:
        rb._sway_live.push(time.monotonic(), 0.0, 0.0)
    cfg = CraneConfig(enable_anti_sway=True, hook_sway_angle_tol_deg=0.3)
    server = _FakeServer(cfg)
    body = _enrich_control_state_payload(server, {'running': False})
    assert body['anti_sway']['enabled'] is True
    assert body['anti_sway']['tol_deg'] == 0.3
    assert 'sway' in body
    assert 'live' in body['sway']


def test_live_html_includes_sway_panel_when_anti_sway():
    html = render_live_html(plc_mode=True, anti_sway=True, sway_tol_deg=0.3)
    assert 'drawHookSway' in html
    assert 'swayMapOffset' in html
    assert 'Hook Sway' in html
    assert 'const ANTI_SWAY = true' in html
    assert 'const SWAY_TOL_DEG = 0.3' in html


def test_live_html_sway_panel_is_a_single_cross():
    """横滚/俯仰合并到一个十字靶上, 而不是两个独立的单轴仪表条。"""
    html = render_live_html(plc_mode=True, anti_sway=True, sway_tol_deg=0.3)
    assert 'drawSwayCross' in html
    assert 'drawSwayAxisGauge' not in html, '单轴仪表已被十字靶取代'
    assert '横滚 θy' in html and '俯仰 θx' in html
    # 判稳看峰峰值 (与调度器一致), |θ| 绝对值只用于核对倾角仪零点
    assert 'swayPeakToPeak' in html
    assert '峰峰' in html
    assert 'T≈' in html


def test_live_html_includes_whole_job_timer():
    """累计时间区展示"开始作业→完成作业"的整段耗时。"""
    html = render_live_html(plc_mode=True, anti_sway=False, sway_tol_deg=0.3)
    assert 'renderJobTimer' in html
    for element_id in ('jobStart', 'jobElapsed', 'jobEnd'):
        assert f'id="{element_id}"' in html
    assert '开始作业' in html and '累计用时' in html and '完成作业' in html
    # Time 显示整段作业耗时, 不能再用每阶段归零的 PD 局部时间 d.t
    assert "els.time.textContent = (d.t || 0).toFixed(1) + 's';" not in html
