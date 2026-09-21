"""按轴撤回角度修正；同一作业内的新阶段不得重新开启已撤回的轴。"""

import math

import pytest

from anti_sway_runtime import AntiSwayRuntime
from sway_controller import AntiSwayController, RopeLengthModel


def test_worsening_axis_is_not_hidden_by_larger_improving_axis():
    from sway_guard import SwayGrowthGuard

    guard = SwayGrowthGuard()
    first_trip = None
    for step in range(300):
        t = step * .05
        ax = 3.0 * math.exp(-t / 30)
        ay = .4 * math.exp(t / 15)
        phase = math.cos(2 * math.pi * t / 3.5)
        guard.update(math.radians(ax * phase), math.radians(ay * phase),
                     dt=.05, period_s=3.5, commanded=(True, True))
        if guard.blocked[1]:
            first_trip = (t, ax, ay)
            break
    assert first_trip is not None
    assert first_trip[1] > first_trip[2], '另一轴仍更大，max(X,Y) 会掩盖小车恶化'
    assert guard.blocked == (False, True)


def test_constant_bias_and_decaying_sway_never_trip():
    from sway_guard import SwayGrowthGuard

    guard = SwayGrowthGuard()
    for step in range(400):
        t = step * .05
        theta = math.radians(2 * math.exp(-t / 10) * math.cos(2 * math.pi * t / 3.5))
        guard.update(theta + math.radians(.45), math.radians(-.3),
                     dt=.05, period_s=3.5, commanded=(True, True))
    assert guard.blocked == (False, False)


def test_observation_alone_does_not_latch_feedback_fault():
    from sway_guard import SwayGrowthGuard

    guard = SwayGrowthGuard()
    for step in range(220):
        theta = math.radians(.2 * math.exp(step * .05 / 4) * math.sin(step * .1))
        guard.update(theta, 0., dt=.05, period_s=3.5, commanded=(False, False))
    assert guard.blocked == (False, False)


def test_runtime_observes_while_feedback_gate_is_closed():
    rt = AntiSwayRuntime(AntiSwayController(RopeLengthModel(4.0), sway_gain=2.6))
    for _ in range(100):
        out = rt.merge_xy_commands(
            .07, -.02, theta_x=math.radians(.45), theta_y=math.radians(-.2),
            z_grab=1., x=0., y=0., locked_x=False, locked_y=False,
            dt=.1, max_velocity_xy=.2, feedback_allowed=False,
        )
        assert out == (.07, -.02, 0., 0.)
    assert math.degrees(rt.dc.dc_x) > .4


def test_latched_axis_returns_pd_even_with_recenter_and_after_stage_change():
    from sway_guard import SwayGrowthGuard

    guard = SwayGrowthGuard()
    for step in range(400):
        t = step * .05
        theta = math.radians(.4 * math.exp(t / 8) * math.sin(2 * math.pi * t / 3.5))
        guard.update(0., theta, dt=.05, period_s=3.5, commanded=(False, True))
    assert guard.blocked[1]

    for locked_y in (True, False):
        rt = AntiSwayRuntime(
            AntiSwayController(RopeLengthModel(4.), sway_gain=2.6),
            recenter_gain=.3, guard=guard,
        )
        rt.reset(0., 0.)
        rt.suspend(.2)
        for angle in (0., .05, -.05, .05):
            _, vy, _, dvy = rt.merge_xy_commands(
                0., -.03, theta_x=0., theta_y=angle,
                z_grab=1., x=0., y=.04, locked_x=True, locked_y=locked_y,
                dt=.1, max_velocity_xy=.2,
            )
            assert vy == -.03
            assert dvy == 0.
        assert guard.blocked[1]


@pytest.mark.parametrize('limited_by', ['speed', 'anchor'])
def test_half_wave_limits_cannot_erase_growth_history(limited_by):
    rt = AntiSwayRuntime(AntiSwayController(RopeLengthModel(4.), sway_gain=2.6), recenter_gain=0.)
    rt.reset(0., 0.)
    for step in range(600):
        t = step * .05
        angle = math.radians(.4 * math.exp(t / 15) * math.sin(2 * math.pi * t / 3.5))
        rt.merge_xy_commands(
            .2 if limited_by == 'speed' else 0., 0., theta_x=angle, theta_y=0.,
            z_grab=1., x=.1 if limited_by == 'anchor' else 0., y=0.,
            locked_x=limited_by == 'anchor', locked_y=True, dt=.05, max_velocity_xy=.2,
        )
    assert rt.guard.blocked == (True, False), '半周期限幅不能不断抬高增长基准'


def test_missing_sensor_interval_clears_rate_history():
    rt = AntiSwayRuntime(AntiSwayController(RopeLengthModel(4.), sway_gain=2.6))
    rt.compute_dv(0., 0., z_grab=1., dt=.1)
    assert rt.compute_dv(.02, 0., z_grab=1., dt=.1)[0] > 0.
    rt.suspend(.1)
    assert rt.compute_dv(.1, 0., z_grab=1., dt=.1)[0] == 0.


def test_long_sample_gap_is_not_counted_as_a_full_observed_period():
    from sway_guard import SwayGrowthGuard

    guard = SwayGrowthGuard()
    for step in range(7):
        guard.update(step * .001, 0., dt=.05, period_s=3.5, commanded=(True, False))
    guard.update(.1, 0., dt=5., period_s=3.5, commanded=(True, False))
    for step in range(7):
        guard.update(.1 + step * .001, 0., dt=.05, period_s=3.5, commanded=(True, False))
    assert guard.last_pp == [None, None], '仅恢复 0.35 秒，不能把断流前端点补成一个周期'
