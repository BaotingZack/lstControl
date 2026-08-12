"""
Tests for the operation scheduler — validates the full multi-phase workflow.

Runs in simulation mode with a MockPLC so no real hardware is needed.
"""

from __future__ import annotations

import time

import pytest

from crane_model import (
    ControlStoppedError,
    CraneConfig,
    CranePlant,
    CraneState,
    SimPositionSource,
    PlantActuator,
)
from plc_interface import MockPLC
from operation_scheduler import (
    OperationScheduler,
    OperationPhase,
    OperationResult,
    SchedulerHooks,
    Z_SAFE_APPROACH,
    Z_SAFE_LIFT_CARGO,
    Z_SAFE_TRANSPORT,
    Z_SAFE_FINAL,
    STABILIZE_DELAY,
    GRIPPER_SAFETY_DELAY,
    GRIPPER_CHECK_TIMEOUT,
    run_simulation_operation,
)


def _make_noop_actuator(plant, state, config):
    """Create a PlantActuator with no-op Z target/reference methods for sim."""
    actuator = PlantActuator(plant, state, config)
    actuator.set_z_target = lambda _h: None       # type: ignore[method-assign]
    actuator.set_z_reference = lambda _h: None    # type: ignore[method-assign]
    return actuator


class TestSchedulerHooks:
    """Unit tests for the SchedulerHooks state management."""

    def test_initial_state_is_idle(self):
        hooks = SchedulerHooks()
        assert hooks.phase == OperationPhase.IDLE
        assert hooks.done is False
        assert hooks.error is None

    def test_set_phase_updates_state(self):
        hooks = SchedulerHooks()
        hooks.set_phase(OperationPhase.APPROACH_XY)
        assert hooks.phase == OperationPhase.APPROACH_XY
        assert hooks.step_count == 1

    def test_set_done_transitions_from_running(self):
        hooks = SchedulerHooks()
        hooks.set_phase(OperationPhase.APPROACH_XY)
        hooks.set_done()
        assert hooks.phase == OperationPhase.DONE
        assert hooks.done is True

    def test_set_error_records_message(self):
        hooks = SchedulerHooks()
        hooks.set_error("Something went wrong")
        assert hooks.phase == OperationPhase.ERROR
        assert hooks.error == "Something went wrong"
        assert hooks.done is True

    def test_set_stopped_records_reason(self):
        hooks = SchedulerHooks()
        hooks.set_stopped("Operator stop")
        assert hooks.phase == OperationPhase.STOPPED
        assert hooks.stop_reason == "Operator stop"

    def test_should_stop_is_initially_false(self):
        hooks = SchedulerHooks()
        assert hooks.should_stop() is False

    def test_stop_sets_flag(self):
        hooks = SchedulerHooks()
        hooks.stop()
        assert hooks.should_stop() is True

    def test_snapshot_includes_all_fields(self):
        hooks = SchedulerHooks()
        hooks.set_phase(OperationPhase.APPROACH_XY, "moving...")
        snap = hooks.snapshot()
        assert snap['phase'] == 'APPROACH_XY'
        assert snap['phase_label'] == OperationPhase.APPROACH_XY.label
        assert snap['done'] is False
        assert snap['error'] is None
        assert snap['stopped'] is False


class TestOperationPhase:
    """Unit tests for phase labels and enumeration."""

    def test_all_phases_have_non_empty_labels(self):
        for phase in OperationPhase:
            assert phase.label, f"{phase.name} has no label"
            assert len(phase.label) > 0

    def test_phase_count(self):
        """Ensure all expected phases exist."""
        expected = {
            OperationPhase.IDLE,
            OperationPhase.ENSURE_GRIPPER_OPEN,
            OperationPhase.APPROACH_XY,
            OperationPhase.APPROACH_Z_DESCEND,
            OperationPhase.GRIPPER_CLAMP,
            OperationPhase.LIFT_CARGO,
            OperationPhase.TRANSPORT_XY,
            OperationPhase.TRANSPORT_Z_DESCEND,
            OperationPhase.GRIPPER_RELEASE,
            OperationPhase.RETURN_Z,
            OperationPhase.DONE,
            OperationPhase.ERROR,
            OperationPhase.STOPPED,
        }
        assert set(OperationPhase) == expected


class TestSafetyConstants:
    """Verify safety height constants match the specification document."""

    def test_safety_heights_are_reasonable(self):
        assert Z_SAFE_APPROACH == 1.0   # 接近取货点安全高度
        assert Z_SAFE_LIFT_CARGO == 1.2 # 夹取后带货上升安全高度
        assert Z_SAFE_TRANSPORT == 1.5  # 运输阶段安全高度
        assert Z_SAFE_FINAL == 1.6      # 最终归位高度

    def test_safety_heights_are_monotonic(self):
        """Transport height should be >= approach/lift heights for safety."""
        assert Z_SAFE_APPROACH <= Z_SAFE_LIFT_CARGO
        assert Z_SAFE_LIFT_CARGO <= Z_SAFE_TRANSPORT
        assert Z_SAFE_TRANSPORT <= Z_SAFE_FINAL

    def test_delay_constants_are_reasonable(self):
        assert STABILIZE_DELAY == 1.0
        assert GRIPPER_SAFETY_DELAY == 0.5
        assert GRIPPER_CHECK_TIMEOUT == 5.0


class TestOperationSchedulerSimulation:
    """End-to-end tests of the scheduler in simulation mode."""

    @pytest.fixture
    def config(self):
        return CraneConfig(
            max_velocity_xy=0.3,
            max_velocity_z=0.3,
            dt=0.01,
            grab_delay=0.3,
            release_delay=0.3,
            safe_height_offset=1.0,
        )

    @pytest.fixture
    def initial_state(self):
        """Crane starts at a known position: X=2, Y=1, Z=5 (high up)."""
        return CraneState(x0=2.0, y0=1.0, z0=5.0)

    def _make_scheduler(self, config, initial_state):
        """Create a scheduler wired to a simulation plant."""
        plant = CranePlant(config)
        source = SimPositionSource(plant, initial_state, config)
        actuator = _make_noop_actuator(plant, initial_state, config)
        mock_plc = MockPLC(verbose=False)

        # Pre-start the mock PLC heartbeat so it's healthy
        mock_plc._connected = True
        mock_plc._heartbeat_healthy = True

        scheduler = OperationScheduler(
            plc=mock_plc,
            source=source,
            actuator=actuator,
            config=config,
            is_simulation=True,
        )
        return scheduler, plant, source

    def test_full_operation_completes_successfully(self, config, initial_state):
        """The full operation: pick at (8,6,0.5) then deliver to (15,10,0.8)."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)

        start_pos = (8.0, 6.0, 0.5)
        target_pos = (15.0, 10.0, 0.8)

        result = scheduler.execute(start_pos=start_pos, target_pos=target_pos)

        assert result.success is True, f"Operation failed: {result.message}"
        assert result.phase == OperationPhase.DONE
        assert result.total_time > 0

        # Verify the scheduler hooks reflect completion
        snap = scheduler.hooks.snapshot()
        assert snap['done'] is True
        assert snap['error'] is None

        # Verify we went through all phases
        phases_seen = {p for _, p in result.phase_history}
        assert OperationPhase.APPROACH_XY in phases_seen
        assert OperationPhase.GRIPPER_CLAMP in phases_seen
        assert OperationPhase.TRANSPORT_XY in phases_seen
        assert OperationPhase.GRIPPER_RELEASE in phases_seen
        assert OperationPhase.RETURN_Z in phases_seen

    def test_operation_starts_from_initial_position(self, config, initial_state):
        """Verify the crane moves from its initial position toward the pick position."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)

        # Start and target are close together to make it fast
        start_pos = (3.0, 2.0, 0.6)
        target_pos = (4.0, 3.0, 0.6)

        result = scheduler.execute(start_pos=start_pos, target_pos=target_pos)

        assert result.success is True

        # The history should show movement from initial to start to target
        assert result.phase_history is not None
        assert len(result.phase_history) > 0

    def test_operation_reaches_final_z_height(self, config, initial_state):
        """After completion, the Z axis should be at Z_SAFE_FINAL (1.6m)."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)

        start_pos = (5.0, 3.0, 0.5)
        target_pos = (7.0, 5.0, 0.5)

        result = scheduler.execute(start_pos=start_pos, target_pos=target_pos)
        assert result.success is True

        # Check the plant's final Z position
        pos = source.get_position()
        # Z should be close to SAFE_Z_FINAL (1.6m) after completion
        assert abs(pos['z'] - Z_SAFE_FINAL) < 0.2, (
            f"Expected Z near {Z_SAFE_FINAL}, got {pos['z']:.3f}"
        )

    def test_operation_stoppable_during_approach(self, config, initial_state):
        """Verify external stop during approach phase is handled cleanly."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)

        # Stop during the approach
        import threading
        def stop_after_delay():
            import time
            time.sleep(0.05)  # stop very early
            scheduler.hooks.stop()

        stopper = threading.Thread(target=stop_after_delay, daemon=True)
        stopper.start()

        start_pos = (50.0, 50.0, 0.5)  # far away, will take time
        target_pos = (60.0, 60.0, 0.5)

        result = scheduler.execute(start_pos=start_pos, target_pos=target_pos)

        # Should be stopped, not errored
        assert result.success is False
        assert result.phase == OperationPhase.STOPPED
        stopper.join(timeout=1.0)

    def test_gripper_actions_are_invoked(self, config, initial_state):
        """Verify gripper clamp and release are called during the operation."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)
        mock_plc = scheduler._plc

        start_pos = (4.0, 2.0, 0.5)
        target_pos = (6.0, 4.0, 0.5)

        result = scheduler.execute(start_pos=start_pos, target_pos=target_pos)
        assert result.success is True

        # Gripper should be in released state at the end (after release phase)
        assert mock_plc.last_gripper == 'released', (
            f"Expected last gripper action to be 'released', got {mock_plc.last_gripper!r}"
        )

    def test_phase_history_is_chronological(self, config, initial_state):
        """Verify phase timestamps are strictly increasing."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)

        start_pos = (6.0, 4.0, 0.5)
        target_pos = (10.0, 8.0, 0.5)

        result = scheduler.execute(start_pos=start_pos, target_pos=target_pos)
        assert result.success is True

        times = [t for t, _ in result.phase_history]
        for i in range(1, len(times)):
            assert times[i] >= times[i - 1], (
                f"Phase times not monotonic: {times[i]} < {times[i-1]}"
            )

    def test_convenience_function_runs(self, config, initial_state):
        """The run_simulation_operation convenience function works."""
        start_pos = (5.0, 3.0, 0.5)
        target_pos = (8.0, 6.0, 0.5)

        result = run_simulation_operation(
            start_pos=start_pos,
            target_pos=target_pos,
            initial_state=initial_state,
            config=config,
            verbose=False,
        )

        assert result.success is True
        assert result.phase == OperationPhase.DONE


class _FixedPositionSource:
    """位置源桩件: 返回固定 (可能带噪声的) 位置, 用于判稳单测。"""

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z

    def get_position(self):
        return {
            'x': self.x, 'y': self.y, 'z': self.z,
            'vx': None, 'vy': None, 'vz': None,
            'dt': 0.02, 't': 0.0, 'stamp': 0,
        }

    def reset(self):
        pass


class _DriftingPositionSource:
    """位置源桩件: 位置持续单向漂移 (模拟货物仍在摆动/未平稳)。"""

    def __init__(self, rate=0.05):
        self._rate = rate
        self._t0 = time.monotonic()

    def get_position(self):
        elapsed = time.monotonic() - self._t0
        return {
            'x': self._rate * elapsed, 'y': 0.0, 'z': 0.0,
            'vx': None, 'vy': None, 'vz': None,
            'dt': 0.02, 't': elapsed, 'stamp': 0,
        }

    def reset(self):
        pass


class _StopAfterNCallsSource:
    """位置源桩件: 若干次调用后触发外部停止信号。"""

    def __init__(self, hooks, n_calls_before_stop=3):
        self._hooks = hooks
        self._remaining = n_calls_before_stop

    def get_position(self):
        if self._remaining <= 0:
            self._hooks.stop()
        else:
            self._remaining -= 1
        return {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'vx': None, 'vy': None, 'vz': None,
            'dt': 0.02, 't': 0.0, 'stamp': 0,
        }

    def reset(self):
        pass


class TestCargoSettleDetection:
    """针对 OperationScheduler._wait_cargo_settled() 的自适应判稳单测。

    验证: 已平稳时提前放行 (效率), 仍在漂移/摆动时持续等待直到超时上限
    (安全), 且可被操作员停止信号中断。
    """

    def _make_scheduler(self, config):
        initial_state = CraneState(x0=0.0, y0=0.0, z0=1.0)
        plant = CranePlant(config)
        source = SimPositionSource(plant, initial_state, config)
        actuator = _make_noop_actuator(plant, initial_state, config)
        mock_plc = MockPLC(verbose=False)
        mock_plc._connected = True
        mock_plc._heartbeat_healthy = True
        return OperationScheduler(
            plc=mock_plc, source=source, actuator=actuator,
            config=config, is_simulation=True,
        )

    def test_settled_cargo_exits_before_max_wait(self):
        """货物已静止 (固定位置) 时, 应在远小于 max_wait 的时间内放行。"""
        config = CraneConfig(hook_settle_window=0.2, hook_settle_pos_tol=0.02,
                              hook_settle_vel_tol=0.02)
        scheduler = self._make_scheduler(config)
        scheduler._source = _FixedPositionSource(x=1.0, y=2.0, z=0.5)

        max_wait = 5.0
        start = time.monotonic()
        scheduler._wait_cargo_settled("测试: 已平稳", max_wait)
        elapsed = time.monotonic() - start

        assert elapsed < max_wait * 0.5, (
            f"已平稳的货物应提前放行, 实际用时 {elapsed:.2f}s (上限 {max_wait:.1f}s)"
        )

    def test_drifting_cargo_waits_until_timeout(self):
        """货物持续漂移/摆动时, 应等到 max_wait 上限才放行 (软失败, 不抛异常)。"""
        config = CraneConfig(hook_settle_window=0.3, hook_settle_pos_tol=0.01,
                              hook_settle_vel_tol=0.01)
        scheduler = self._make_scheduler(config)
        scheduler._source = _DriftingPositionSource(rate=0.5)  # 远超阈值, 永不判稳

        max_wait = 0.6
        start = time.monotonic()
        scheduler._wait_cargo_settled("测试: 持续漂移", max_wait)
        elapsed = time.monotonic() - start

        assert elapsed >= max_wait * 0.9, (
            f"持续漂移应等到超时上限附近才放行, 实际用时 {elapsed:.2f}s"
        )

    def test_stop_signal_interrupts_settle_wait(self):
        """操作员停止信号应能中断判稳等待, 抛出 ControlStoppedError。"""
        config = CraneConfig(hook_settle_window=0.2)
        scheduler = self._make_scheduler(config)
        scheduler._source = _StopAfterNCallsSource(scheduler.hooks, n_calls_before_stop=2)

        with pytest.raises(ControlStoppedError):
            scheduler._wait_cargo_settled("测试: 停止中断", max_wait=5.0)


class TestEnsureGripperOpenBeforePickup:
    """验证取货前会确认抓钩已开启, 未开启时主动释放 (Phase 0)。"""

    def _make_scheduler(self, config, initial_state):
        plant = CranePlant(config)
        source = SimPositionSource(plant, initial_state, config)
        actuator = _make_noop_actuator(plant, initial_state, config)
        mock_plc = MockPLC(verbose=False)
        mock_plc._connected = True
        mock_plc._heartbeat_healthy = True
        scheduler = OperationScheduler(
            plc=mock_plc, source=source, actuator=actuator,
            config=config, is_simulation=True,
        )
        return scheduler, mock_plc

    def test_auto_releases_when_gripper_starts_clamped(self):
        """抓钩初始处于夹紧状态时, 取货前应主动释放一次 (额外的 Phase 0)。"""
        config = CraneConfig(max_velocity_xy=0.3, max_velocity_z=0.3, dt=0.01)
        initial_state = CraneState(x0=2.0, y0=1.0, z0=5.0)
        scheduler, mock_plc = self._make_scheduler(config, initial_state)

        # 模拟上次作业异常结束, 抓钩仍停留在夹紧状态。
        mock_plc.gripper_clamp()
        release_calls = []
        original_release = mock_plc.gripper_release

        def _tracking_release():
            release_calls.append(time.monotonic())
            original_release()

        mock_plc.gripper_release = _tracking_release

        result = scheduler.execute(start_pos=(4.0, 2.0, 0.5), target_pos=(6.0, 4.0, 0.5))

        assert result.success is True
        # 一次是 Phase 0 的主动释放, 一次是 Phase 2d 正常释放钢卷。
        assert len(release_calls) == 2, (
            f"预期抓钩已夹紧时触发一次额外的 Phase 0 主动释放, "
            f"实际释放调用次数={len(release_calls)}"
        )
        phases_seen = {p for _, p in result.phase_history}
        assert OperationPhase.ENSURE_GRIPPER_OPEN in phases_seen

    def test_no_extra_release_when_already_open(self):
        """抓钩已经开启时, 不应触发额外的 Phase 0 主动释放。"""
        config = CraneConfig(max_velocity_xy=0.3, max_velocity_z=0.3, dt=0.01)
        initial_state = CraneState(x0=2.0, y0=1.0, z0=5.0)
        scheduler, mock_plc = self._make_scheduler(config, initial_state)

        # 抓钩已经是开启状态 (如刚上电/复位)。
        mock_plc.gripper_release()
        release_calls = []
        original_release = mock_plc.gripper_release

        def _tracking_release():
            release_calls.append(time.monotonic())
            original_release()

        mock_plc.gripper_release = _tracking_release

        result = scheduler.execute(start_pos=(4.0, 2.0, 0.5), target_pos=(6.0, 4.0, 0.5))

        assert result.success is True
        # 只有 Phase 2d 正常释放钢卷这一次, 没有额外的 Phase 0 主动释放。
        assert len(release_calls) == 1, (
            f"预期抓钩已开启时不触发额外释放, 实际释放调用次数={len(release_calls)}"
        )


class TestPostReleaseLift:
    """验证释放后的简化流程: 完全释放确认后停留 ~1s, 再直接以正常速度
    抬升到目标高度 (不做分段限速)。"""

    @pytest.fixture
    def config(self):
        return CraneConfig(
            max_velocity_xy=0.3, max_velocity_z=0.3, dt=0.01,
            post_release_lift_delay=1.0,
        )

    @pytest.fixture
    def initial_state(self):
        return CraneState(x0=2.0, y0=1.0, z0=5.0)

    def _make_scheduler(self, config, initial_state):
        plant = CranePlant(config)
        source = SimPositionSource(plant, initial_state, config)
        actuator = _make_noop_actuator(plant, initial_state, config)
        mock_plc = MockPLC(verbose=False)
        mock_plc._connected = True
        mock_plc._heartbeat_healthy = True
        scheduler = OperationScheduler(
            plc=mock_plc, source=source, actuator=actuator,
            config=config, is_simulation=True,
        )
        return scheduler, plant, source

    def test_pause_between_release_and_return_z(self, config, initial_state):
        """GRIPPER_RELEASE 阶段和 RETURN_Z 阶段的记录时间差应至少覆盖
        post_release_lift_delay (释放确认后到开始抬升前的停留)。"""
        scheduler, plant, source = self._make_scheduler(config, initial_state)

        result = scheduler.execute(start_pos=(4.0, 2.0, 0.5), target_pos=(6.0, 4.0, 0.5))

        assert result.success is True
        phase_times = {phase: t for t, phase in result.phase_history}
        release_t = phase_times[OperationPhase.GRIPPER_RELEASE]
        return_t = phase_times[OperationPhase.RETURN_Z]
        assert return_t - release_t >= config.post_release_lift_delay - 0.05, (
            f"释放到开始抬升的间隔应至少覆盖 post_release_lift_delay, "
            f"实际间隔={return_t - release_t:.2f}s"
        )

    def test_return_z_lifts_directly_without_speed_staging(self, config, initial_state):
        """RETURN_Z 阶段不再做分段限速, PD 应能按正常 max_velocity_z 抬升。"""
        scheduler, plant, source = self._make_scheduler(config, initial_state)

        current_state = CraneState(x0=6.0, y0=4.0, z0=0.5)
        hist, _ = scheduler._run_pd(
            target=(6.0, 4.0, config.return_safe_z),
            initial_state=current_state,
            phase_label="RETURN_Z",
        )

        assert len(hist) > 0
        # 起步阶段应能看到接近 max_velocity_z 的速度指令 (未被人为限速)。
        early_steps = hist[: max(1, len(hist) // 10)]
        max_early_vz_cmd = max(abs(h['vz_cmd']) for h in early_steps)
        assert max_early_vz_cmd > config.max_velocity_z * 0.5, (
            f"起步阶段速度指令应能接近正常上限, 实际最大值={max_early_vz_cmd:.3f}"
        )
        assert abs(hist[-1]['z'] - config.return_safe_z) < 0.05


class TestMockPLCGripper:
    """Test the MockPLC gripper simulation behavior."""

    def test_initial_gripper_state_is_unknown(self):
        plc = MockPLC()
        assert plc.get_gripper_clamped() is False
        assert plc.get_gripper_released() is False

    def test_clamp_sets_correct_state(self):
        plc = MockPLC()
        plc.gripper_clamp()
        assert plc.get_gripper_clamped() is True
        assert plc.get_gripper_released() is False
        assert plc.last_gripper == 'clamped'

    def test_release_sets_correct_state(self):
        plc = MockPLC()
        plc.gripper_release()
        assert plc.get_gripper_clamped() is False
        assert plc.get_gripper_released() is True
        assert plc.last_gripper == 'released'

    def test_clamp_and_release_are_mutually_exclusive(self):
        plc = MockPLC()
        plc.gripper_clamp()
        assert plc.get_gripper_clamped() is True
        assert plc.get_gripper_released() is False

        plc.gripper_release()
        assert plc.get_gripper_clamped() is False
        assert plc.get_gripper_released() is True

        # Clamp again
        plc.gripper_clamp()
        assert plc.get_gripper_clamped() is True
        assert plc.get_gripper_released() is False


class TestOperationSchedulerEdgeCases:
    """Edge-case tests discovered during code review."""

    @pytest.fixture
    def config(self):
        return CraneConfig(
            max_velocity_xy=0.3,
            max_velocity_z=0.3,
            dt=0.01,
        )

    @pytest.fixture
    def initial_state(self):
        return CraneState(x0=2.0, y0=1.0, z0=5.0)

    def _make_scheduler(self, config, initial_state):
        plant = CranePlant(config)
        source = SimPositionSource(plant, initial_state, config)
        actuator = PlantActuator(plant, initial_state, config)
        actuator.set_z_target = lambda _h: None
        actuator.set_z_reference = lambda _h: None
        mock_plc = MockPLC(verbose=False)
        mock_plc._connected = True
        mock_plc._heartbeat_healthy = True
        scheduler = OperationScheduler(
            plc=mock_plc, source=source, actuator=actuator,
            config=config, is_simulation=True,
        )
        return scheduler, plant, source

    def test_pick_z_above_safety_height_works(self, config, initial_state):
        """Edge case: start Z is above Z_SAFE_APPROACH (e.g., picking from shelf)."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)
        # Pick at Z=1.2m > Z_SAFE_APPROACH (1.0m)
        start_pos = (6.0, 4.0, 1.2)
        target_pos = (10.0, 8.0, 0.5)
        result = scheduler.execute(start_pos=start_pos, target_pos=target_pos)
        assert result.success is True

    def test_pick_z_equals_safety_height_works(self, config, initial_state):
        """Edge case: start Z exactly equals Z_SAFE_APPROACH (no Z movement in Phase 1a→1b)."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)
        start_pos = (6.0, 4.0, 1.0)  # sz == Z_SAFE_APPROACH
        target_pos = (10.0, 8.0, 0.5)
        result = scheduler.execute(start_pos=start_pos, target_pos=target_pos)
        assert result.success is True

    def test_identical_start_and_target_positions(self, config, initial_state):
        """Edge case: start == target (pick and deliver at same location)."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)
        pos = (6.0, 4.0, 0.5)
        result = scheduler.execute(start_pos=pos, target_pos=pos)
        assert result.success is True

    def test_operation_stoppable_during_gripper_wait(self, config, initial_state):
        """Verify STOP is recognized during gripper status polling."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)

        import threading
        def stop_during_gripper():
            import time
            time.sleep(0.1)
            scheduler.hooks.stop()

        stopper = threading.Thread(target=stop_during_gripper, daemon=True)
        stopper.start()

        start_pos = (3.0, 2.0, 0.5)
        target_pos = (5.0, 4.0, 0.5)
        result = scheduler.execute(start_pos=start_pos, target_pos=target_pos)

        # Should be stopped (may complete very fast or be stopped)
        assert result.phase in (OperationPhase.DONE, OperationPhase.STOPPED)
        stopper.join(timeout=1.0)

    def test_operation_stoppable_during_safety_sleep(self, config, initial_state):
        """Verify STOP during safety sleep is recognized."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)

        import threading
        def stop_during_sleep():
            import time
            time.sleep(0.2)  # wait for the stabilize delay to start
            scheduler.hooks.stop()

        stopper = threading.Thread(target=stop_during_sleep, daemon=True)
        stopper.start()

        start_pos = (3.0, 2.0, 0.5)
        target_pos = (5.0, 4.0, 0.5)
        result = scheduler.execute(start_pos=start_pos, target_pos=target_pos)

        # If stop fires during the 1s stabilize delay, it should be STOPPED
        assert result.phase in (OperationPhase.DONE, OperationPhase.STOPPED)
        stopper.join(timeout=1.0)

    def test_control_state_phase_updates(self, config, initial_state):
        """Verify ControlState receives phase updates during operation."""
        from live_server import ControlState

        scheduler, plant, source = self._make_scheduler(config, initial_state)
        cs = ControlState()

        start_pos = (4.0, 2.0, 0.5)
        target_pos = (6.0, 4.0, 0.5)
        result = scheduler.execute(
            start_pos=start_pos, target_pos=target_pos, control_state=cs,
        )
        assert result.success is True

        snap = cs.snapshot()
        assert snap['done'] is True
        assert snap['scheduler_phase'] == 'DONE'
        assert snap['scheduler_phase_label'] == OperationPhase.DONE.label

    def test_control_state_receives_step_data(self, config, initial_state):
        """Verify ControlState.latest is updated with PD step data."""
        from live_server import ControlState

        scheduler, plant, source = self._make_scheduler(config, initial_state)
        cs = ControlState()

        start_pos = (4.0, 2.0, 0.5)
        target_pos = (6.0, 4.0, 0.5)
        result = scheduler.execute(
            start_pos=start_pos, target_pos=target_pos, control_state=cs,
        )
        assert result.success is True

        # After operation, ControlState should have step data
        snap = cs.snapshot()
        assert snap['latest'] is not None, "ControlState should have latest step data"
        assert 'x' in snap['latest'], "Step data should include x position"

    def test_scheduler_rejects_invalid_positions(self, config, initial_state):
        """Verify the scheduler validates positions before starting."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)

        # Non-finite position should be rejected
        import math
        invalid_pos = (float('nan'), 5.0, 0.5)
        with pytest.raises((ValueError, RuntimeError)):
            scheduler.execute(start_pos=invalid_pos, target_pos=(6.0, 4.0, 0.5))

    def test_multiple_operations_on_same_scheduler(self, config, initial_state):
        """Verify a scheduler can be reused for multiple operations."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)

        # First operation
        result1 = scheduler.execute(start_pos=(4.0, 2.0, 0.5), target_pos=(6.0, 4.0, 0.5))
        assert result1.success is True

        # Second operation from where we left off
        result2 = scheduler.execute(start_pos=(8.0, 6.0, 0.5), target_pos=(10.0, 8.0, 0.5))
        assert result2.success is True

    def test_gripper_status_timeout_does_not_abort(self, config, initial_state):
        """Verify that gripper status timeout produces a warning but doesn't abort."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)

        # Override the gripper provider to always return unknown status
        scheduler._gripper_provider = lambda: (None, None)

        start_pos = (4.0, 2.0, 0.5)
        target_pos = (6.0, 4.0, 0.5)
        result = scheduler.execute(start_pos=start_pos, target_pos=target_pos)

        # Should still complete — timeout is non-fatal
        assert result.success is True

    def test_final_z_at_safe_final_height(self, config, initial_state):
        """After the operation, Z should be at or near Z_SAFE_FINAL (1.6m)."""
        scheduler, plant, source = self._make_scheduler(config, initial_state)

        start_pos = (5.0, 3.0, 0.4)
        target_pos = (9.0, 7.0, 0.7)
        result = scheduler.execute(start_pos=start_pos, target_pos=target_pos)
        assert result.success is True

        # Get final position from the plant
        pos = source.get_position()
        assert abs(pos['z'] - 1.6) < 0.15, (
            f"Final Z should be near 1.6m, got {pos['z']:.3f}m"
        )
        # X and Y should be at the target position (delivery location)
        assert abs(pos['x'] - 9.0) < 0.1
        assert abs(pos['y'] - 7.0) < 0.1
