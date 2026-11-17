"""Settle (manual push damping) UI pages."""

from live_ptp_html import render_ptp_html
from live_settle_html import render_settle_html


def test_settle_sim_page():
    html = render_settle_html(plc_mode=False)
    assert '消摆测试' in html
    assert '/settle/sim' in html
    assert 'settle-sim' in html and 'aria-current="page"' in html
    assert 'btnStartSettle' in html
    assert 'settleElapsed' in html
    assert 'settleInitialPeak' in html
    assert 'drawSettleBaselineOverlay' in html
    assert 'drawStaticResidualOverlay' in html
    assert 'settleResidualWidget' in html
    assert 'settleResidualResize' not in html
    assert 'drawResidualMiniCross' in html
    assert 'settle-stage-layout' in html
    assert 'settleStageRail' in html
    assert 'stageHudPeak' in html
    assert 'updateStageHud' in html
    assert 'syncSettleStageLayout' in html
    assert 'drawSettleHookSway' in html
    assert 'deepseekInclinometerExtra' in html
    assert 'staticAngleCard' not in html
    assert '初始最大' in html
    assert 'value="deepseek_anti"' in html and ' selected>' in html.split('deepseek_anti', 1)[1]
    assert 'btnSaveRunLog' not in html
    assert 'value="baseline"' in html
    assert 'value="gantry_settle"' in html
    assert 'value="overhead_settle"' in html
    assert 'antiSwayMode' in html
    assert 'settle-algo-fixed' not in html
    assert 'value="incl"' not in html
    assert 'value="gantry"' not in html
    assert 'value="overhead"' not in html
    assert '纯 PD' not in html
    assert 'const PLC_MODE = false' in html


def test_settle_live_page():
    html = render_settle_html(plc_mode=True)
    assert 'const PLC_MODE = true' in html
    assert 'settle-live' in html


def test_ptp_page_includes_settle_nav():
    html = render_ptp_html(plc_mode=False)
    assert '/settle/sim' in html
    assert '/settle/live' in html
    assert '点到点·仿真' in html
