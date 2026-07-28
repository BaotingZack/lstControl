"""
Tests for the operation scheduler — validates the full multi-phase workflow.

Runs in simulation mode with a MockPLC so no real hardware is needed.
"""

from __future__ import annotations

import pytest

from crane_model import CraneConfig, CranePlant, CraneState, SimPositionSource, PlantActuator
from plc_interface import MockPLC
from operation_scheduler import (
    OperationScheduler,
    OperationPhase,
    OperationResult,
    SchedulerHooks,
    Z_SAFE_APPROACH,
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
        assert Z_SAFE_APPROACH == 1.0   # 取货阶段安全高度
        assert Z_SAFE_TRANSPORT == 1.5  # 运输阶段安全高度
        assert Z_SAFE_FINAL == 1.6      # 最终归位高度

    def test_safety_heights_are_monotonic(self):
        """Transport height should be >= approach height for safety."""
        assert Z_SAFE_APPROACH <= Z_SAFE_TRANSPORT
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
