"""倾角判稳 + 悬停软阻尼 — 调度器侧的防摇集成。

这些用例用假的位置源/执行器和一个可编程的摆角源, 把 _wait_cargo_settled() 的
判据选择、主动阻尼下发、位移预算、发散保护与降级路径都钉住。时间用假时钟推进,
不依赖真实 sleep。
"""

from __future__ import annotations

import math
import time

import pytest

from crane_model import CraneConfig
from operation_scheduler import OperationScheduler
from plc_interface import MockPLC
from sway_controller import (
    AntiSwayController,
    RopeLengthModel,
    build_anti_sway,
    damping_ratio,
    envelope_decay_time,
    gain_for_damping_ratio,
)

GRAVITY = 9.81


class _Clock:
    """假单调时钟 — 每次 tick 前进固定步长 (默认 10Hz, 与现场轮询一致)。"""

    def __init__(self, step: float = 0.1):
        self.t = 1000.0
        self.step = step

    def now(self) -> float:
        return self.t

    def tick(self) -> float:
        self.t += self.step
        return self.t


class _FakeSource:
    """位置源: 每次调用推进时钟并返回当前位置 (可由测试改写 x/y/z)。"""

    def __init__(self, clock: _Clock, x: float = 5.0, y: float = 3.0, z: float = 1.5):
        self.clock = clock
        self.x, self.y, self.z = x, y, z
        self.calls = 0

    def get_position(self) -> dict:
        self.calls += 1
        self.clock.tick()
        return {
            'x': self.x, 'y': self.y, 'z': self.z,
            'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
            'dt': self.clock.step, 't': self.clock.now(), 'stamp': self.clock.now(),
        }

    def reset(self) -> None:
        pass


class _FakeActuator:
    """执行器: 记录每次 apply/stop_motion, 并把 vx/vy 积分成位置回灌给位置源。"""

    def __init__(self, source: _FakeSource | None = None, integrate: bool = False):
        self.applied: list[tuple[float, float, float, float]] = []
        self.stop_calls = 0
        self._source = source
        self._integrate = integrate

    def apply(self, vx: float, vy: float, vz: float, dt: float) -> None:
        self.applied.append((vx, vy, vz, dt))
        if self._integrate and self._source is not None:
            self._source.x += vx * dt
            self._source.y += vy * dt

    def stop_motion(self) -> None:
        self.stop_calls += 1

    def emergency_stop(self) -> None:
        self.stop_calls += 1

    def update_state(self, state, position) -> None:
        pass

    @property
    def vx_commands(self) -> list[float]:
        return [a[0] for a in self.applied]


def _config(**overrides) -> CraneConfig:
    defaults = dict(
        enable_anti_sway=True,
        anti_sway_terminal_enabled=False,  # legacy guard/budget mechanism tests
        anti_sway_sway_gain=2.6,
        anti_sway_angle_scale=1.0,
        anti_sway_max_correction=0.20,
        anti_sway_settle_damping=True,
        anti_sway_settle_gain_scale=1.5,
        anti_sway_settle_dc_tau=2.5,
        anti_sway_locked_max_shift=0.15,
        anti_sway_settle_max_shift=0.05,
        anti_sway_hold_recenter_gain=0.3,
        rope_length_sheave_height=4.0,
        rope_length_grab_offset=0.0,
        hook_sway_angle_tol_deg=0.3,
        hook_settle_window=0.6,
        hook_settle_window_periods=0.55,
        hook_settle_max_wait=12.0,
    )
    defaults.update(overrides)
    return CraneConfig(**defaults)


def _scheduler(config, source, actuator, sway_provider):
    plc = MockPLC(verbose=False)
    plc._connected = True
    plc._heartbeat_healthy = True
    return OperationScheduler(
        plc=plc,
        source=source,
        actuator=actuator,
        config=config,
        is_simulation=False,   # 关掉仿真节流 sleep, 时间由假时钟推进
        sway_provider=sway_provider,
    )


@pytest.fixture
def clock(monkeypatch):
    clk = _Clock()
    monkeypatch.setattr(time, 'monotonic', clk.now)
    return clk


def _decaying_sway(clock: _Clock, amp_deg: float, tau: float, period: float = 3.05):
    """衰减单摆摆角源 (度制输入, 返回弧度)。"""
    t0 = clock.now()

    def provider() -> dict:
        t = clock.now() - t0
        amp = math.radians(amp_deg) * math.exp(-t / tau)
        return {
            'theta_x': amp * math.sin(2 * math.pi * t / period),
            'theta_y': 0.0,
        }

    return provider


def test_terminal_settle_cannot_enable_feedback_from_free_sway_alone(clock):
    source = _FakeSource(clock)
    actuator = _FakeActuator()
    cfg = _config(anti_sway_terminal_enabled=True, hook_settle_max_wait=8.)
    sch = _scheduler(cfg, source, actuator, _decaying_sway(clock, 1., 1e6))
    sch._wait_cargo_settled('terminal qualification', 8.)
    assert actuator.applied
    assert all(vx == vy == 0 for vx, vy, _, _ in actuator.applied)


def test_pd_withdrawal_survives_transition_to_settle(clock):
    """升降段已撤回的小车轴，在随后独立判稳运行时不能重新推车。"""
    t0 = clock.now()

    class HoistSource(_FakeSource):
        lowering = True
        segment_t0 = t0

        def reset(self):
            self.segment_t0 = clock.now()

        def get_position(self):
            pos = super().get_position()
            pos['t'] = clock.now() - self.segment_t0
            if self.lowering:
                self.z = max(.5, 1.5 - (clock.now() - t0) * .05)
                pos['z'] = self.z
            return pos

    class LoopActuator(_FakeActuator):
        def update_state(self, state, position):
            for axis in ('x', 'y', 'z'):
                getattr(state, axis).position = position[axis]
                getattr(state, axis).velocity = position['v' + axis]

        def cleanup(self):
            pass

    source = HoistSource(clock)
    actuator = LoopActuator()
    cfg = _config()

    def sway():
        t = clock.now() - t0
        return {'theta_x': 0., 'theta_y': math.radians(.4 * math.exp(t / 10) * math.sin(2 * math.pi * t / 3.5))}

    sch = _scheduler(cfg, source, actuator, sway)
    from crane_model import CraneState
    sch._run_pd((5., 3., .5), CraneState(5., 3., 1.5), 'lower')
    before = len(actuator.applied)
    assert any(abs(vy) > .001 for _, vy, _, _ in actuator.applied)
    source.lowering = False
    sch._wait_cargo_settled('after lowering', 4., damp_max_shift=.06)
    assert actuator.applied[before:]
    assert all(vy == 0. for _, vy, _, _ in actuator.applied[before:]), '新判稳阶段不得复位撤回状态'


def test_no_shift_settle_observes_without_sending_commands(clock):
    source = _FakeSource(clock)
    actuator = _FakeActuator()
    cfg = _config(hook_settle_max_wait=8.)
    sch = _scheduler(cfg, source, actuator, _decaying_sway(clock, 1., 1e6))
    sch._wait_cargo_settled('observe', 8., damp_max_shift=0.)
    assert not actuator.applied
    assert sch._sway_guard.last_pp[0] is not None
    assert sch._sway_guard.blocked == (False, False)


# ---------------------------------------------------------------------------
# 判据选择与收敛
# ---------------------------------------------------------------------------

class TestSettleCriterion:

    def test_converging_sway_releases_before_timeout(self, clock):
        """摆幅衰减到容差内 → 提前放行, 且全程有主动阻尼指令下发。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config()
        sch = _scheduler(cfg, source, actuator, _decaying_sway(clock, 1.0, 1.0))

        t_start = clock.now()
        sch._wait_cargo_settled("test", cfg.stabilize_delay)
        elapsed = clock.now() - t_start

        assert elapsed < cfg.hook_settle_max_wait, "应在上限前收敛放行"
        # 衰减够快时, 零偏还没解完摆已经进容差 — 此时正确做法是不动车就放行
        assert actuator.stop_calls >= 1, "退出时必须把速度收回"

    def test_persistent_sway_times_out_and_soft_fails(self, clock):
        """摆不收敛 → 等满上限后软失败继续 (不抛异常), 不会提前放行。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config(hook_settle_max_wait=4.0)
        # tau 极大 = 几乎不衰减的 1° 摆
        sch = _scheduler(cfg, source, actuator, _decaying_sway(clock, 1.0, 1e6))

        t_start = clock.now()
        sch._wait_cargo_settled("test", 1.0)
        elapsed = clock.now() - t_start

        assert elapsed >= cfg.hook_settle_max_wait

    def test_car_position_noise_cannot_pass_angle_criterion(self, clock):
        """位置判据的盲区: 车体静止但吊钩在摆, 必须判为未平稳。

        这正是现场"判稳通过了却还在摆 1°"的根因 — 位置判据测的是车体。
        """
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config(hook_settle_max_wait=4.0)
        sch = _scheduler(cfg, source, actuator, _decaying_sway(clock, 1.0, 1e6))

        t_start = clock.now()
        sch._wait_cargo_settled("test", 1.0)

        # 车体一动不动 (source.x/y 从未变), 位置判据会立刻放行; 角度判据不会
        assert clock.now() - t_start >= cfg.hook_settle_max_wait

    def test_observe_only_when_damping_disabled(self, clock):
        """enable_anti_sway=False: 仍用角度判稳, 但不下发任何指令。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config(enable_anti_sway=False, anti_sway_sway_gain=0.0)
        sch = _scheduler(cfg, source, actuator, _decaying_sway(clock, 1.0, 1.0))

        sch._wait_cargo_settled("test", cfg.stabilize_delay)

        assert sch._sway_damper is None
        assert actuator.applied == [], "未启用防摇时不得动车"


# ---------------------------------------------------------------------------
# 安全约束
# ---------------------------------------------------------------------------

class TestSettleSafety:

    def test_velocity_never_exceeds_max_correction(self, clock):
        """悬停期唯一速度来源是防摇修正量, 必须被 max_correction 硬钳住。

        gain_scale 是在 max_correction 钳位之后才乘上去的, 若不再钳一次,
        实际下发速度会达到 1.5×max_correction。
        """
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config(anti_sway_max_correction=0.05, hook_settle_max_wait=4.0)
        # 5° 大摆 → 未钳位时 Δv = 2.6*1.5*0.087 = 0.34 m/s
        sch = _scheduler(cfg, source, actuator, _decaying_sway(clock, 5.0, 1e6))

        sch._wait_cargo_settled("test", 1.0)

        assert actuator.applied
        assert max(abs(v) for v in actuator.vx_commands) <= cfg.anti_sway_max_correction + 1e-9

    def test_budget_blocks_outward_motion(self, clock):
        """位移超出预算后, 只允许朝锚点回拉的方向。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator(source, integrate=True)
        budget = 0.05
        cfg = _config(
            anti_sway_locked_max_shift=budget,
            hook_settle_max_wait=6.0,
            anti_sway_max_correction=0.20,
        )
        # 单一符号的摆角 (斜坡), 高通后仍留同号分量 → 不加预算会一路把车推走
        t0 = clock.now()

        def ramp() -> dict:
            return {'theta_x': math.radians(2.0) * (clock.now() - t0), 'theta_y': 0.0}

        sch = _scheduler(cfg, source, actuator, ramp)
        anchor_x = source.x

        sch._wait_cargo_settled("test", 1.0)

        # 预算是"超出后禁止继续外推", 单周期仍可越界一步 (v*dt), 留一步余量
        overshoot = cfg.anti_sway_max_correction * 0.1
        assert abs(source.x - anchor_x) <= budget + overshoot + 1e-9

    def test_tight_budget_passed_for_near_cargo_settle(self, clock):
        """贴近钢卷的判稳点必须用收紧的预算 (damp_max_shift)。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config(hook_settle_max_wait=1.0, anti_sway_settle_max_shift=0.05)
        sch = _scheduler(cfg, source, actuator, _decaying_sway(clock, 1.0, 1e6))

        sch._wait_cargo_settled(
            "test", 1.0, damp_max_shift=cfg.anti_sway_settle_max_shift,
        )

        assert sch._sway_damper.locked_max_shift == cfg.anti_sway_settle_max_shift

    def test_zero_budget_is_observe_only(self, clock):
        """settle_max_shift=0: 贴近钢卷时只判稳、一步都不许横移。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config(hook_settle_max_wait=4.0)
        sch = _scheduler(cfg, source, actuator, _decaying_sway(clock, 1.0, 1e6))

        sch._wait_cargo_settled("test", 1.0, damp_max_shift=0.0)

        assert actuator.applied == [], "贴近钢卷时不得下发阻尼横移"

    def test_oscillation_damps_before_bias_is_solved(self, clock):
        """真在摆时不必干等 0.6T: rate_amp 门限已能区分摆和纯零偏。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config(hook_settle_max_wait=1.0)
        sch = _scheduler(cfg, source, actuator, _decaying_sway(clock, 1.0, 1e6))

        sch._wait_cargo_settled("test", 1.0)

        assert any(abs(v) > 1e-4 for v in actuator.vx_commands), (
            "1° 摆必须立刻开始阻尼, 不能空等零偏窗"
        )

    def test_stale_travel_dc_is_discarded(self, clock):
        """行进段留下的假零偏必须丢掉, 改用本次悬停的摆动解算。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config(hook_settle_max_wait=8.0)
        true_bias = math.radians(-0.44)
        t0 = clock.now()
        period = 3.05

        def sway() -> dict:
            t = clock.now() - t0
            return {
                'theta_x': true_bias + math.radians(1.0) * math.sin(
                    2 * math.pi * t / period
                ),
                'theta_y': 0.0,
            }

        sch = _scheduler(cfg, source, actuator, sway)
        sch._sway_damper.dc.seed(math.radians(2.0), math.radians(1.0))
        sch._wait_cargo_settled("test", 8.0)

        # reset() 必须清掉 2° 假零偏; 之后的跟踪器会跟着摆动走, 只要不再停在 2°
        assert abs(math.degrees(sch._sway_damper.dc.dc_x) - 2.0) > 1.0
        assert abs(math.degrees(sch._sway_damper.dc.dc_y) - 1.0) > 0.5

    def test_constant_bias_does_not_push_the_car(self, clock):
        """纯零偏不是摆。解出后 θ_ac≈0, 不得把车推离锚点。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator(source, integrate=True)
        cfg = _config(hook_settle_max_wait=6.0)
        x0 = source.x
        sch = _scheduler(
            cfg, source, actuator,
            lambda: {'theta_x': math.radians(1.0), 'theta_y': 0.0},
        )

        sch._wait_cargo_settled("test", 1.0)

        assert abs(source.x - x0) < 0.01, (
            f"纯零偏把车推走了 {abs(source.x - x0) * 1000:.0f}mm"
        )

    def test_divergence_guard_disables_damping(self, clock):
        """摆幅在阻尼下反而放大 (典型原因: 倾角符号标反) → 撤阻尼并停车。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config(hook_settle_max_wait=12.0)
        t0 = clock.now()

        def growing() -> dict:
            # 起始 PP 已 1° (超过容差), 再以 τ=2s 持续放大 — 负阻尼的典型特征
            t = clock.now() - t0
            amp = math.radians(0.5) * math.exp(t / 2.0)
            return {'theta_x': amp * math.sin(2 * math.pi * t / 3.05), 'theta_y': 0.0}

        sch = _scheduler(cfg, source, actuator, growing)
        sch._wait_cargo_settled("test", 1.0)

        assert sch._sway_guard.blocked == (True, False)
        # 逐轴撤回仍允许另一轴工作；检查的是恶化轴保持零速，而非 apply 的调用次数。
        assert actuator.stop_calls >= 1
        assert any(abs(vx) > .001 for vx in actuator.vx_commands)
        assert all(vx == 0. for vx in actuator.vx_commands[-40:])


# ---------------------------------------------------------------------------
# 降级路径
# ---------------------------------------------------------------------------

class TestSettleFallback:

    def test_no_provider_uses_position_criterion(self, clock):
        """未注入摆角源 → 位置判稳, 且绝不动车。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config()
        sch = _scheduler(cfg, source, actuator, None)

        assert sch._sway_analyzer is None and sch._sway_damper is None
        sch._wait_cargo_settled("test", 1.0)
        assert actuator.applied == []

    def test_dead_sensor_falls_back_to_position(self, clock):
        """摆角源恒返回 None (断流) → 降级为位置判稳, 不动车。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config()
        sch = _scheduler(cfg, source, actuator, lambda: None)

        sch._wait_cargo_settled("test", 1.0)
        assert actuator.applied == []

    def test_provider_exception_is_contained(self, clock):
        """传感器侧异常不得掀翻作业 — 降级为位置判稳。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config()

        def boom():
            raise RuntimeError('inclinometer offline')

        sch = _scheduler(cfg, source, actuator, boom)
        sch._wait_cargo_settled("test", 1.0)   # 不应抛出
        assert actuator.applied == []

    def test_nan_sway_is_rejected(self, clock):
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        sch = _scheduler(
            _config(), source, actuator,
            lambda: {'theta_x': float('nan'), 'theta_y': 0.0},
        )
        assert sch._read_sway() is None


# ---------------------------------------------------------------------------
# 闭环消摆 — 带真实单摆动力学, 验证控制律符号与实际消摆能力
# ---------------------------------------------------------------------------

class _PendulumRig:
    """悬停消摆闭环仿真: 速度指令 → 一阶速度环 → 台车加速度 → 单摆。

    单摆受迫方程 L·θ̈ + g·θ = −ẍ (ẍ=台车加速度)。速度模式下 Δv=+K·θ 使
    ẍ=K·θ̇ ⇒ dE/dt=−L·K·θ̇²<0, 即阻尼; 符号反了就是反向注入能量。
    """

    def __init__(
        self,
        cfg: CraneConfig,
        *,
        z: float = 0.36,
        theta0_deg: float = 1.0,
        bias_deg: float = 0.0,
        phase: float = 0.0,
        gain_sign: float = 1.0,
        dc_seed_deg: float | None = None,   # 模拟行进段接力过来的零偏估计
        ctrl_dt: float = 0.1,     # 现场 10Hz
        sim_dt: float = 0.002,
        tau_servo: float = 0.18,  # CraneConfig.servo_time_constant_xy
    ):
        from sway_settle import effective_rope_length

        self.cfg = cfg
        self.z = z
        self.L = effective_rope_length(
            z,
            sheave_height=cfg.rope_length_sheave_height,
            grab_offset=cfg.rope_length_grab_offset,
        )
        self.ctrl_dt, self.sim_dt, self.tau_servo = ctrl_dt, sim_dt, tau_servo
        self.bias = math.radians(bias_deg)
        amp = math.radians(theta0_deg)
        self.theta = amp * math.cos(phase)
        self.theta_d = -amp * math.sqrt(GRAVITY / self.L) * math.sin(phase)
        self.x = self.v = self.v_cmd = 0.0
        self.peak_shift = 0.0

        ctrl = AntiSwayController(
            RopeLengthModel(cfg.rope_length_sheave_height, cfg.rope_length_grab_offset),
            sway_gain=gain_sign * cfg.anti_sway_sway_gain,
            max_correction=cfg.anti_sway_max_correction,
        )
        from anti_sway_runtime import AntiSwayRuntime

        self.rt = AntiSwayRuntime(
            anti_sway=ctrl,
            dc_tau=cfg.anti_sway_settle_dc_tau,
            locked_max_shift=cfg.anti_sway_locked_max_shift,
            gain_scale=cfg.anti_sway_settle_gain_scale,
            recenter_gain=cfg.anti_sway_hold_recenter_gain,
        )
        self.rt.reset(0.0, 0.0)
        if dc_seed_deg is not None:
            self.rt.dc.seed(math.radians(dc_seed_deg), 0.0)

        from sway_settle import SwayLiveAnalyzer

        self.analyzer = SwayLiveAnalyzer(
            base_window=cfg.hook_settle_window,
            window_periods=cfg.hook_settle_window_periods,
            sheave_height=cfg.rope_length_sheave_height,
            grab_offset=cfg.rope_length_grab_offset,
            tol_deg=cfg.hook_sway_angle_tol_deg,
        )

    def run(self, duration: float = 20.0) -> dict:
        t = 0.0
        next_ctrl = 0.0
        settled_at = None
        while t < duration:
            if t >= next_ctrl - 1e-9:
                self.v_cmd, _, _, _ = self.rt.merge_xy_commands(
                    0.0, 0.0,
                    theta_x=self.theta + self.bias, theta_y=0.0,
                    z_grab=self.z, x=self.x, y=0.0,
                    locked_x=True, locked_y=True,
                    dt=self.ctrl_dt,
                    max_velocity_xy=self.cfg.anti_sway_max_correction,
                )
                self.analyzer.push(t, self.theta + self.bias, 0.0)
                if self.analyzer.metrics(self.z)['settled'] and settled_at is None:
                    settled_at = t
                next_ctrl += self.ctrl_dt
            a = (self.v_cmd - self.v) / self.tau_servo
            self.v += a * self.sim_dt
            self.x += self.v * self.sim_dt
            theta_dd = -(GRAVITY / self.L) * self.theta - a / self.L
            self.theta_d += theta_dd * self.sim_dt
            self.theta += self.theta_d * self.sim_dt
            self.peak_shift = max(self.peak_shift, abs(self.x))
            t += self.sim_dt
        return {
            'settled_at': settled_at,
            'peak_shift': self.peak_shift,
            'residual_shift': self.x,
            'final_theta_deg': math.degrees(abs(self.theta)),
        }


class TestClosedLoopDamping:
    """带单摆动力学的闭环验证 — 这是"能否压到 0.3°"的直接证据。"""

    def test_undamped_pendulum_stays_above_tolerance(self):
        """基线: 不防摇时 1° 的摆在 20s 内远远压不到 0.3° (钢丝绳自然阻尼太弱)。"""
        cfg = _config(
            enable_anti_sway=False, anti_sway_sway_gain=0.0,
            anti_sway_hold_recenter_gain=0.0,
        )
        rig = _PendulumRig(cfg, theta0_deg=1.0)
        rig.rt.anti_sway = AntiSwayController(
            RopeLengthModel(4.0, 0.0), sway_gain=0.0, max_correction=0.2,
        )
        out = rig.run(20.0)
        assert out['settled_at'] is None
        # 末帧瞬时 |θ| 可能落在过零点, 用窗内峰峰值判断还在摆
        pp = rig.analyzer.metrics(rig.z)['detrend_pp_max_deg']
        assert pp is not None and pp > 0.6, f'无阻尼 20s 后 PP={pp}'

    def test_damping_drives_one_degree_below_tolerance(self):
        """K=2.6 + gain_scale 1.5 → 1° 摆在判稳上限内收敛到容差以内。"""
        cfg = _config()
        out = _PendulumRig(cfg, theta0_deg=1.0).run(20.0)

        assert out['settled_at'] is not None, "应在 20s 内判稳放行"
        assert out['settled_at'] < cfg.hook_settle_max_wait
        assert out['final_theta_deg'] < cfg.hook_sway_angle_tol_deg

    @pytest.mark.parametrize('z', [0.36, 1.0, 1.5])
    @pytest.mark.parametrize('phase', [0.0, math.pi / 2])
    def test_converges_across_height_and_phase(self, z, phase):
        """全行程绳长 × 起摆相位 (峰值处/过零处) 都要收敛且不超预算。"""
        cfg = _config()
        out = _PendulumRig(cfg, z=z, theta0_deg=1.0, phase=phase).run(20.0)

        assert out['settled_at'] is not None and out['settled_at'] < cfg.hook_settle_max_wait
        assert out['final_theta_deg'] < cfg.hook_sway_angle_tol_deg
        assert out['peak_shift'] <= cfg.anti_sway_locked_max_shift + 1e-6

    def test_wrong_sign_diverges(self):
        """符号标反即负阻尼 — 必须发散, 这正是发散保护要抓的故障。"""
        cfg = _config()
        out = _PendulumRig(cfg, theta0_deg=1.0, gain_sign=-1.0).run(20.0)

        assert out['final_theta_deg'] > 1.0, "负增益却没有放大摆动, 符号推导有误"
        assert out['settled_at'] is None

    @pytest.mark.parametrize('bias_deg', [0.3, 1.0])
    def test_recentering_kills_residual_bias_drift(self, bias_deg):
        """残余零偏不得把车推走 — 回中项是这里的唯一防线。"""
        cfg = _config()
        with_recenter = _PendulumRig(cfg, bias_deg=bias_deg).run(20.0)
        without = _PendulumRig(
            _config(anti_sway_hold_recenter_gain=0.0), bias_deg=bias_deg,
        ).run(20.0)

        assert abs(without['residual_shift']) > abs(
            with_recenter['residual_shift']
        ) * 5, "回中项应显著压低残余偏移"
        assert abs(with_recenter['residual_shift']) < 0.02
        # 过程中的峰值偏移也必须留在预算内
        assert with_recenter['peak_shift'] <= cfg.anti_sway_locked_max_shift + 1e-6

    def test_recentering_does_not_slow_damping(self):
        """回中项必须远慢于摆频: 开启前后的消摆时间应基本一致。"""
        on = _PendulumRig(_config()).run(20.0)
        off = _PendulumRig(_config(anti_sway_hold_recenter_gain=0.0)).run(20.0)
        assert on['settled_at'] == pytest.approx(off['settled_at'], abs=0.5)

    def test_unprimed_hold_must_not_inherit_travel_bias(self):
        """行进段假零偏不得直接拿来消摆 — 那是加减速污染, 不是静态零偏。"""
        cfg = _config(anti_sway_hold_recenter_gain=0.0)
        # 冷启动 (dc=0) 与被注入 2° 行进段污染的对比: 调度器会 reset 掉污染,
        # 运行时本身若被错误 seed, 峰值偏移会明显更大。这里钉住"错误 seed 有害",
        # 防止以后再把行进段 tracker 接回来。
        clean = _PendulumRig(cfg, bias_deg=0.44).run(8.0)
        poisoned = _PendulumRig(
            cfg, bias_deg=0.44, dc_seed_deg=2.0,
        ).run(8.0)
        assert abs(poisoned['peak_shift']) > abs(clean['peak_shift']), (
            "行进段 2° 假零偏应显著加大起步漂移, 否则这个回归抓不住误用"
        )

    def test_prime_dc_matches_the_injected_bias(self):
        """悬停段兜底解算: 没有行进段接力时, 零偏要能从摆动信号本身解出来。"""
        bias_deg = -0.44
        rig = _PendulumRig(_config(), bias_deg=bias_deg, theta0_deg=1.0)
        # 用自由摆样本解算, 不走闭环: 阻尼会把正弦压歪, 拟合误差会变大
        wn = math.sqrt(GRAVITY / rig.L)
        for i in range(50):
            t = i * 0.1
            theta = math.radians(bias_deg) + math.radians(1.0) * math.sin(wn * t)
            rig.analyzer.push(t, theta, 0.0)
        solved = rig.rt.prime_dc(rig.analyzer.samples(), rig.z)
        assert solved is not None, "一个周期以上的样本应当足以解出零偏"
        assert math.degrees(solved[0]) == pytest.approx(bias_deg, abs=0.05)

    def test_dc_tracker_seed_does_not_bias_the_hold(self):
        """θ_dc 初值若取第一帧 (恰好是摆峰), 会积分成米级预算耗尽的单向漂移。"""
        cfg = _config(anti_sway_hold_recenter_gain=0.0)
        rig = _PendulumRig(cfg, theta0_deg=1.0, phase=0.0)   # phase=0 即峰值起摆
        out = rig.run(20.0)
        assert abs(out['residual_shift']) < 0.02, (
            f"零偏初值把车推走了 {out['residual_shift'] * 1000:.0f}mm"
        )


# ---------------------------------------------------------------------------
# 控制律与增益标定
# ---------------------------------------------------------------------------

class TestSwayController:

    def test_damping_sign_chases_the_load(self):
        """Δv 必须与 θ 同号 (小车追着载荷走), 反号即负阻尼。"""
        ctrl = AntiSwayController(
            RopeLengthModel(sheave_height=4.0, grab_offset=0.0),
            sway_gain=2.6,
            max_correction=1.0,
        )
        dvx, dvy, _ = ctrl.compute(0.01, -0.01, 1.0)
        assert dvx > 0 and dvy < 0

    def test_gain_and_damping_ratio_round_trip(self):
        for L in (2.42, 2.81, 3.45):
            for zeta in (0.2, 0.5, 0.7):
                K = gain_for_damping_ratio(zeta, L)
                assert damping_ratio(K, L) == pytest.approx(zeta)

    def test_damping_ratio_matches_closed_loop_formula(self):
        """ζ = K/(2√(gL)) — site_config 的增益标定就是按这个反解的。"""
        K, L = 5.8, 3.42
        assert damping_ratio(K, L) == pytest.approx(
            K / (2 * math.sqrt(GRAVITY * L)), rel=1e-6
        )
        assert damping_ratio(5.79, 3.42) == pytest.approx(0.5, abs=0.01)

    def test_envelope_decay_time_matches_analytic(self):
        K, L = 3.9, 3.45
        t = envelope_decay_time(K, L, 1.0, 0.3)
        assert t == pytest.approx((2 * L / K) * math.log(1.0 / 0.3), rel=1e-6)

    def test_decay_time_fits_settle_budget(self):
        """现场增益必须能在判稳上限内把 1° 压到 0.3° (含判稳窗填满)。"""
        from sway_settle import effective_rope_length, pendulum_period

        cfg = _config()
        K_hold = cfg.anti_sway_sway_gain * cfg.anti_sway_settle_gain_scale
        for z in (0.36, 1.0, 1.5):
            L = effective_rope_length(
                z,
                sheave_height=cfg.rope_length_sheave_height,
                grab_offset=cfg.rope_length_grab_offset,
            )
            window = cfg.hook_settle_window_periods * pendulum_period(
                z,
                sheave_height=cfg.rope_length_sheave_height,
                grab_offset=cfg.rope_length_grab_offset,
            )
            need = envelope_decay_time(K_hold, L, 1.0, cfg.hook_sway_angle_tol_deg) + window
            assert need < cfg.hook_settle_max_wait, f'Z={z}: 需要 {need:.1f}s'

    def test_max_correction_clamps_output(self):
        ctrl = AntiSwayController(
            RopeLengthModel(sheave_height=4.0, grab_offset=0.0),
            sway_gain=100.0,
            max_correction=0.08,
        )
        dvx, dvy, _ = ctrl.compute(0.5, -0.5, 1.0)
        assert dvx == pytest.approx(0.08)
        assert dvy == pytest.approx(-0.08)

    def test_negative_max_correction_rejected(self):
        with pytest.raises(ValueError):
            AntiSwayController(RopeLengthModel(sheave_height=4.0), max_correction=-1.0)

    def test_gain_schedule_interpolates_and_saturates(self):
        rope = RopeLengthModel(sheave_height=10.0, grab_offset=0.0)
        ctrl = AntiSwayController(
            rope, gain_schedule=((2.0, 2.0), (4.0, 6.0)), max_correction=10.0,
        )
        # L=3.0 落在断点中间 → K=4.0
        dvx, _, L = ctrl.compute(1.0, 0.0, 7.0)
        assert L == pytest.approx(3.0)
        assert dvx == pytest.approx(4.0)
        # 超出两端 → 端点饱和
        assert ctrl.compute(1.0, 0.0, 9.0)[0] == pytest.approx(2.0)   # L=1.0 → 下限
        assert ctrl.compute(1.0, 0.0, 4.0)[0] == pytest.approx(6.0)   # L=6.0 → 上限

    def test_gain_schedule_must_be_increasing(self):
        with pytest.raises(ValueError):
            AntiSwayController(
                RopeLengthModel(sheave_height=10.0),
                gain_schedule=((4.0, 1.0), (4.0, 2.0)),
            )

    def test_rope_model_matches_settle_module(self):
        """控制用绳长与判稳窗/摆周期用的绳长必须是同一套几何。"""
        from sway_settle import effective_rope_length

        rope = RopeLengthModel(sheave_height=4.0, grab_offset=0.0, min_length=0.5)
        for z in (0.36, 1.0, 1.5, 2.0):
            assert rope.compute(z) == pytest.approx(
                effective_rope_length(z, sheave_height=4.0, grab_offset=0.0)
            )

    def test_rope_length_floor(self):
        rope = RopeLengthModel(sheave_height=4.0, grab_offset=0.0, min_length=0.5)
        assert rope.compute(100.0) == pytest.approx(0.5)

    def test_build_anti_sway_respects_switches(self):
        assert build_anti_sway(_config(enable_anti_sway=False)) is None
        assert build_anti_sway(_config(anti_sway_sway_gain=0.0)) is None
        assert build_anti_sway(_config()) is not None


# ---------------------------------------------------------------------------
# 现场联调入口
# ---------------------------------------------------------------------------

class TestDampInPlace:
    """damp_in_place() — 现场联调用的原地消摆入口。

    它必须与作业流程里的悬停判稳走同一份代码, 否则现场整定出来的增益不可迁移。
    """

    def test_damps_and_releases_like_the_job_path(self, clock):
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config()
        # 衰减要慢于零偏解算窗, 否则还没开始阻尼就已判稳放行
        sch = _scheduler(cfg, source, actuator, _decaying_sway(clock, 1.0, 8.0))

        t0 = clock.now()
        sch.damp_in_place(10.0)

        assert actuator.applied, "必须真正下发阻尼指令"
        assert actuator.stop_calls >= 1, "退出必须停车"

    def test_respects_explicit_shift_budget(self, clock):
        """传入的位移预算必须被执行 — 现场靠它限制行车动多远。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator(source=source, integrate=True)
        cfg = _config()
        x0 = source.x
        # 恒定大摆角: 阻尼会持续朝一个方向推, 预算是唯一的约束
        sch = _scheduler(
            cfg, source, actuator,
            lambda: {'theta_x': math.radians(2.0), 'theta_y': 0.0},
        )
        sch.damp_in_place(6.0, damp_max_shift=0.03)

        assert abs(source.x - x0) <= 0.03 + 0.02, (
            f"位移 {abs(source.x - x0):.3f}m 超出 3cm 预算"
        )

    def test_observe_only_gain_zero_never_moves(self, clock):
        """增益为 0 = 纯观测模式: 判稳照算, 但一步都不能动车。"""
        source = _FakeSource(clock)
        actuator = _FakeActuator()
        cfg = _config(anti_sway_sway_gain=0.0)
        sch = _scheduler(cfg, source, actuator, _decaying_sway(clock, 1.0, 50.0))

        sch.damp_in_place(3.0)

        assert all(vx == 0.0 and vy == 0.0 for vx, vy, _, _ in actuator.applied), (
            "观测模式下不得下发任何非零速度"
        )
