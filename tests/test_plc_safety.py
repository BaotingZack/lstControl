import math
import json
import threading

import pytest

import live_server
import main
from crane_model import ControlHooks, CraneConfig, CraneState, run_pd_control
from live_server import ControlState, CraneLiveServer
from plc_interface import MockPLC, PlcActuator, create_plc


class RecordingPLC(MockPLC):
    def __init__(self, *, connected=True, heartbeat_healthy=True):
        super().__init__()
        self._connected = connected
        self._heartbeat_healthy = heartbeat_healthy
        self.big_car_commands = []
        self.small_car_commands = []
        self.lift_commands = []

    def big_car_ctrl(self, velocity):
        self.big_car_commands.append(velocity)

    def small_car_ctrl(self, velocity):
        self.small_car_commands.append(velocity)

    def lift_ctrl(self, height):
        self.lift_commands.append(height)


class StubActuator:
    def __init__(self):
        self.apply_calls = []
        self.stop_calls = 0
        self.emergency_stop_calls = 0
        self.cleanup_calls = 0

    def apply(self, vx, vy, vz, dt):
        self.apply_calls.append((vx, vy, vz, dt))

    def update_state(self, state, position):
        state.x.position = position["x"]
        state.y.position = position["y"]
        state.z.position = position["z"]
        state.x.velocity = position["vx"]
        state.y.velocity = position["vy"]
        state.z.velocity = position["vz"]

    def stop_motion(self):
        self.stop_calls += 1

    def emergency_stop(self):
        self.emergency_stop_calls += 1

    def cleanup(self):
        self.cleanup_calls += 1


class SequenceSource:
    def __init__(self, positions):
        self._positions = iter(positions)

    def reset(self):
        pass

    def get_position(self):
        return next(self._positions)


def position(*, x=0.0, y=0.0, z=0.0, t=0.1, stamp=1):
    return {
        "x": x,
        "y": y,
        "z": z,
        "vx": 0.0,
        "vy": 0.0,
        "vz": 0.0,
        "dt": 0.1,
        "t": t,
        "stamp": stamp,
    }


def test_plc_actuator_sends_updated_height_while_velocity_is_constant():
    plc = RecordingPLC()
    actuator = PlcActuator(plc, initial_z=1.0)

    for _ in range(5):
        actuator.apply(0.0, 0.0, 0.2, 0.1)

    assert plc.lift_commands == pytest.approx([1.02, 1.04, 1.06, 1.08, 1.10])


def test_plc_actuator_rejects_motion_when_connection_or_heartbeat_is_unhealthy():
    disconnected = PlcActuator(RecordingPLC(connected=False), initial_z=1.0)
    heartbeat_lost = PlcActuator(
        RecordingPLC(heartbeat_healthy=False),
        initial_z=1.0,
    )

    with pytest.raises(RuntimeError, match="connection"):
        disconnected.apply(0.1, 0.0, 0.0, 0.1)
    with pytest.raises(RuntimeError, match="heartbeat"):
        heartbeat_lost.apply(0.1, 0.0, 0.0, 0.1)


def test_timeout_always_emergency_stops_and_cleans_up():
    actuator = StubActuator()
    source = SequenceSource([position(t=1.0)])

    with pytest.raises(TimeoutError, match="did not finish"):
        run_pd_control(
            source=source,
            actuator=actuator,
            config=CraneConfig(),
            target_pos=(1.0, 1.0, 1.0),
            initial_state=CraneState(0.0, 0.0, 0.0),
            max_time=0.5,
            verbose=False,
            is_simulation=False,
        )

    assert actuator.emergency_stop_calls == 1
    assert actuator.cleanup_calls == 1


def test_successful_arrival_explicitly_stops_motion_and_cleans_up():
    actuator = StubActuator()
    source = SequenceSource([position(x=0.01, y=0.01, z=0.01)])

    _, arrivals = run_pd_control(
        source=source,
        actuator=actuator,
        config=CraneConfig(),
        target_pos=(0.01, 0.01, 0.01),
        initial_state=CraneState(0.01, 0.01, 0.01),
        verbose=False,
        is_simulation=False,
    )

    assert {axis for _, axis in arrivals} == {"x", "y", "z"}
    assert actuator.stop_calls == 1
    assert actuator.emergency_stop_calls == 0
    assert actuator.cleanup_calls == 1


def test_position_timeout_is_an_error_instead_of_normal_completion():
    actuator = StubActuator()
    source = SequenceSource([None])

    with pytest.raises(RuntimeError, match="position feedback timed out"):
        run_pd_control(
            source=source,
            actuator=actuator,
            config=CraneConfig(),
            target_pos=(1.0, 1.0, 1.0),
            initial_state=CraneState(0.0, 0.0, 0.0),
            verbose=False,
            is_simulation=False,
        )

    assert actuator.emergency_stop_calls == 1
    assert actuator.cleanup_calls == 1


def test_stop_requested_while_waiting_for_position_never_applies_motion():
    actuator = StubActuator()

    class StopDuringReadHooks(ControlHooks):
        def __init__(self):
            self.stopped = False

        def should_stop(self):
            return self.stopped

    hooks = StopDuringReadHooks()

    class StopDuringReadSource(SequenceSource):
        def get_position(self):
            hooks.stopped = True
            return super().get_position()

    source = StopDuringReadSource([position()])

    with pytest.raises(RuntimeError, match="stopped"):
        run_pd_control(
            source=source,
            actuator=actuator,
            config=CraneConfig(),
            target_pos=(1.0, 1.0, 1.0),
            initial_state=CraneState(0.0, 0.0, 0.0),
            hooks=hooks,
            verbose=False,
            is_simulation=False,
        )

    assert actuator.apply_calls == []
    assert actuator.emergency_stop_calls == 1


@pytest.mark.parametrize(
    "target",
    [
        (math.nan, 0.0, 0.0),
        (math.inf, 0.0, 0.0),
        (0.0, 0.0, -0.01),
    ],
)
def test_config_rejects_non_finite_or_negative_height_targets(target):
    with pytest.raises(ValueError):
        CraneConfig().validate_target(target)


def test_config_enforces_configured_workspace_bounds():
    config = CraneConfig(
        workspace_x_bounds=(0.0, 20.0),
        workspace_y_bounds=(-5.0, 5.0),
        workspace_z_bounds=(0.0, 10.0),
    )

    assert config.validate_target((10.0, 0.0, 5.0)) == (10.0, 0.0, 5.0)
    with pytest.raises(ValueError, match="X target"):
        config.validate_target((21.0, 0.0, 5.0))


def test_control_state_distinguishes_operator_stop_from_success():
    state = ControlState()
    state.set_start(
        pos={"x": 0.0, "y": 0.0, "z": 0.0},
        target={"x": 1.0, "y": 1.0, "z": 1.0},
    )

    state.set_stopped("Stopped by operator")

    snapshot = state.snapshot()
    assert snapshot["running"] is False
    assert snapshot["done"] is False
    assert snapshot["stopped"] is True
    assert snapshot["stop_reason"] == "Stopped by operator"


def test_missing_plc_library_requires_explicit_mock_opt_in(tmp_path):
    missing_library = tmp_path / "missing-libsscarctrl.so"

    with pytest.raises(RuntimeError, match="Cannot load PLC library"):
        create_plc(lib_path=str(missing_library))

    assert isinstance(
        create_plc(lib_path=str(missing_library), allow_mock=True),
        MockPLC,
    )


def test_web_target_parser_uses_control_config_validation():
    config = CraneConfig(workspace_x_bounds=(0.0, 5.0))

    assert live_server._parse_control_target(
        json.dumps({"target_x": 2, "target_y": 1, "target_z": 3}),
        config,
    ) == (2.0, 1.0, 3.0)

    with pytest.raises(ValueError, match="finite"):
        live_server._parse_control_target(
            '{"target_x": NaN, "target_y": 1, "target_z": 3}',
            config,
        )
    with pytest.raises(ValueError, match="X target"):
        live_server._parse_control_target(
            json.dumps({"target_x": 6, "target_y": 1, "target_z": 3}),
            config,
        )


def test_control_run_reservation_allows_only_one_concurrent_starter():
    server = CraneLiveServer(("127.0.0.1", 0), payload=None)
    barrier = threading.Barrier(8)
    reservations = []
    result_lock = threading.Lock()

    def reserve():
        barrier.wait()
        result = server.reserve_control_run()
        with result_lock:
            reservations.append(result)

    threads = [threading.Thread(target=reserve) for _ in range(8)]
    try:
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        assert reservations.count(True) == 1
        assert reservations.count(False) == 7
        server.release_control_run()
        assert server.reserve_control_run() is True
    finally:
        server.release_control_run()
        server.server_close()


def test_cli_builds_workspace_bounds_and_explicit_mock_mode():
    args = main._build_arg_parser().parse_args(
        [
            "--allow-mock-plc",
            "--workspace-x-min", "0",
            "--workspace-x-max", "30",
            "--workspace-y-min", "-10",
            "--workspace-y-max", "10",
            "--workspace-z-min", "0",
            "--workspace-z-max", "15",
        ]
    )

    config = main._config_from_args(args)

    assert args.allow_mock_plc is True
    assert config.workspace_x_bounds == (0.0, 30.0)
    assert config.workspace_y_bounds == (-10.0, 10.0)
    assert config.workspace_z_bounds == (0.0, 15.0)


def test_cli_rejects_half_configured_workspace_bounds():
    args = main._build_arg_parser().parse_args(["--workspace-x-min", "0"])

    with pytest.raises(ValueError, match="workspace X"):
        main._config_from_args(args)


def test_plc_connection_failure_stops_startup(monkeypatch):
    class FailedPLC(RecordingPLC):
        def __init__(self):
            super().__init__(connected=False, heartbeat_healthy=False)
            self.disconnected = False
            self.heartbeat_started = False

        def connect(self, ip):
            return 7

        def start_heartbeat(self):
            self.heartbeat_started = True

        def disconnect(self):
            self.disconnected = True

    plc = FailedPLC()
    monkeypatch.setattr(main, "create_plc", lambda **kwargs: plc)
    args = main._build_arg_parser().parse_args(["--plc-ip", "192.168.0.1"])

    with pytest.raises(ConnectionError, match="ret=7"):
        main._connect_plc(args)

    assert plc.heartbeat_started is False
    assert plc.disconnected is True
