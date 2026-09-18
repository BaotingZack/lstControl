"""anti_sway_runtime — 锁轴软阻尼与 θ_ac 防摇。"""

from __future__ import annotations

import math

import pytest

from anti_sway_runtime import (
    AntiSwayRuntime,
    approach_damping_allowed,
    clamp_damping_axis,
)
from sway_controller import AntiSwayController, RopeLengthModel


def _runtime(**kwargs) -> AntiSwayRuntime:
    rope = RopeLengthModel(sheave_height=8.0, grab_offset=1.5)
    ctrl = AntiSwayController(rope, sway_gain=0.5, max_correction=0.08)
    defaults = dict(
        anti_sway=ctrl,
        dc_tau=0.5,
        locked_max_shift=0.08,
        use_ac=True,
    )
    defaults.update(kwargs)
    return AntiSwayRuntime(**defaults)


def test_approach_gate_blocks_fast_cruise():
    """离目标远不叠; 进入 0.40m 后即使还在走也叠 (车速不再当门限)。"""
    assert not approach_damping_allowed(
        err_x=2.0, err_y=0.0, vx=0.18, vy=0.0,
        locked_x=False, locked_y=False,
        travel_on=False, approach_on=True,
        approach_pos=0.40, approach_vel=0.10,
    )
    assert approach_damping_allowed(
        err_x=0.20, err_y=0.05, vx=0.18, vy=0.02,
        locked_x=False, locked_y=False,
        travel_on=False, approach_on=True,
        approach_pos=0.40, approach_vel=0.10,
    )


def test_approach_gate_allows_locked_xy():
    """XY 锁死、Z 还在走: 必须允许软阻尼, 否则下降段又无防摇。"""
    assert approach_damping_allowed(
        err_x=0.0, err_y=0.0, vx=0.0, vy=0.0,
        locked_x=True, locked_y=True,
        travel_on=False, approach_on=True,
        approach_pos=0.40, approach_vel=0.10,
    )


def test_static_bias_does_not_command():
    """恒定零偏不得变成速度指令。"""
    rt = _runtime()
    rt.reset(0.0, 0.0)
    last = None
    for _ in range(20):
        last = rt.compute_dv(math.radians(0.44), 0.0, z_grab=1.0, dt=0.1)
    assert last[0] == 0.0 and last[1] == 0.0


def test_clamp_damping_blocks_outward_only():
    assert clamp_damping_axis(0.05, 0.09, 0.08) == 0.0
    assert clamp_damping_axis(-0.05, 0.09, 0.08) == -0.05
    assert clamp_damping_axis(0.05, 0.0, 0.0) == 0.0
    assert clamp_damping_axis(-0.05, 0.01, 0.0) == 0.0


def test_locked_axis_applies_soft_damping():
    rt = _runtime(use_ac=False)
    rt.reset(0.0, 0.0)

    def step(theta):
        return rt.merge_xy_commands(
            0.0, 0.0,
            theta_x=theta, theta_y=0.0,
            z_grab=1.0, x=0.0, y=0.0,
            locked_x=True, locked_y=True,
            dt=0.02, max_velocity_xy=0.2,
        )

    step(0.0)   # 建立差分, 下一帧的 θ̇ 才非零
    vx, vy, dvx, dvy = step(0.2)
    assert vx != 0.0 or vy != 0.0
    assert abs(vx) <= 0.08


def test_unlocked_axis_adds_to_pd():
    rt = _runtime()
    rt.reset(0.0, 0.0)
    vx, _, dvx, _ = rt.merge_xy_commands(
        0.1, 0.0,
        theta_x=0.1, theta_y=0.0,
        z_grab=1.0, x=0.0, y=0.0,
        locked_x=False, locked_y=True,
        dt=0.02, max_velocity_xy=0.2,
    )
    assert vx == pytest.approx(0.1 + dvx, abs=0.001)


def test_set_anchor_updates_one_axis_only():
    rt = _runtime()
    rt.reset(1.0, 2.0)
    rt.set_anchor(x=5.0)
    assert rt.anchor_xy == (5.0, 2.0)
    rt.set_anchor(y=7.0)
    assert rt.anchor_xy == (5.0, 7.0)


def test_locked_damping_stays_symmetric_when_anchored_at_lock_point():
    """锁轴时必须把锚点移到锁定点, 否则位移预算早已耗尽, 阻尼被半波整流。

    锚点停在出发位置 (0.0) 而小车锁在 3.0 m 处时, |x−anchor| 远超预算,
    clamp_damping_axis 只放行朝出发点回拉的半个周期 → 只剩单向漂移。
    """
    rt = _runtime(use_ac=False)

    def sweep() -> list[float]:
        rt.merge_xy_commands(
            0.0, 0.0,
            theta_x=0.0, theta_y=0.0,
            z_grab=1.0, x=3.0, y=0.0,
            locked_x=True, locked_y=True,
            dt=0.02, max_velocity_xy=0.2,
        )
        out = []
        for theta in (0.05, -0.05, 0.05, -0.05):
            vx, _, _, _ = rt.merge_xy_commands(
                0.0, 0.0,
                theta_x=theta, theta_y=0.0,
                z_grab=1.0, x=3.0, y=0.0,
                locked_x=True, locked_y=True,
                dt=0.02, max_velocity_xy=0.2,
            )
            out.append(vx)
        return out

    rt.reset(0.0, 0.0)
    stale = sweep()
    assert all(v <= 0.0 for v in stale)
    assert any(v == 0.0 for v in stale)

    rt.reset(0.0, 0.0)
    rt.set_anchor(x=3.0)
    fixed = sweep()
    assert fixed[0] > 0.0 and fixed[1] < 0.0
    assert fixed[0] == pytest.approx(-fixed[1])


def test_ac_tracks_dc_so_locked_dv_small_after_step():
    rt = _runtime()
    rt.reset(0.0, 0.0)
    for _ in range(100):
        vx, _, _, _ = rt.merge_xy_commands(
            0.0, 0.0,
            theta_x=0.15, theta_y=0.0,
            z_grab=1.0, x=0.0, y=0.0,
            locked_x=True, locked_y=True,
            dt=0.02, max_velocity_xy=0.2,
        )
    assert abs(vx) < 0.02


# ---------------------------------------------------------------------------
# 零偏估计的跨阶段接力 — 锚点是任务量, 零偏是器件量, 两者生命周期必须分开
# ---------------------------------------------------------------------------

def test_reset_keep_dc_preserves_the_bias_estimate():
    """换锚点不该丢零偏: 传感器没换, 已学到的零偏仍然有效。"""
    rt = _runtime()
    rt.dc.seed(math.radians(0.44), math.radians(-0.2))

    rt.reset(5.0, 6.0, keep_dc=True)
    assert rt.anchor_xy == (5.0, 6.0)
    assert rt.dc.dc_x == pytest.approx(math.radians(0.44))
    assert rt.dc.dc_y == pytest.approx(math.radians(-0.2))

    rt.reset(0.0, 0.0)          # 默认仍是"整体重置"
    assert rt.dc.dc_x == 0.0 and rt.dc.dc_y == 0.0


def test_shared_tracker_hands_bias_between_runtimes():
    """行进段与悬停段增益不同, 不能共用整个运行时, 但必须共用零偏跟踪器。"""
    travel = _runtime()
    hold = _runtime(gain_scale=2.0, dc_tracker=travel.dc)
    assert hold.dc is travel.dc

    # 行进段"学到"零偏后, 悬停段应当直接看到, 无须重新收敛
    travel.dc.seed(math.radians(0.44), 0.0)
    _, _, ac_x, _ = hold.compute_dv(
        math.radians(0.44), 0.0, z_grab=1.0, dt=0.1,
    )
    assert abs(math.degrees(ac_x)) < 0.01, "接力过来的零偏应当已被扣掉"


def test_unshared_runtimes_start_from_zero_bias():
    """未传 dc_tracker 时各自独立 — 保证老调用方行为不变。"""
    a, b = _runtime(), _runtime()
    assert a.dc is not b.dc
    a.dc.seed(math.radians(0.44), 0.0)
    assert b.dc.dc_x == 0.0
