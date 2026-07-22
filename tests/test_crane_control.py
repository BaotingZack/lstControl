import pytest

from crane_model import CraneConfig, CraneState
from live_server import build_live_payload, render_live_html
from main import run_simulation
from pd_controller import PositionPDController
from visualizer import CraneVisualizer


@pytest.fixture
def default_config():
    return CraneConfig(
        max_velocity_xy=0.2,
        max_velocity_z=0.2,
        kp_pos=0.6,
        kd_pos=0.45,
        dt=0.01,
        enable_disturbance=True,
        disturbance_seed=42,
    )


def test_position_controller_uses_filtered_velocity_for_damping():
    controller = PositionPDController(kp_pos=0.6, kd_pos=0.45, v_max=1.0)

    no_velocity = controller.update(target_position=10.0, measured_position=9.0)
    moving_toward_target = controller.update(
        target_position=10.0,
        measured_position=9.0,
        measured_velocity=0.4,
    )
    moving_away_from_target = controller.update(
        target_position=10.0,
        measured_position=9.0,
        measured_velocity=-0.4,
    )
    reverse = controller.update(target_position=10.0, measured_position=11.0)

    assert no_velocity == pytest.approx(0.6)
    assert moving_toward_target == pytest.approx(0.42)
    assert moving_away_from_target == pytest.approx(0.78)
    assert reverse == pytest.approx(-0.6)


def test_dispatch_target_is_reached_by_position_to_velocity_control(default_config):
    target_pos = (8.0, 6.0, 1.5)
    history, events = run_simulation(
        target_pos=target_pos,
        initial_state=CraneState(x0=0.0, y0=0.0, z0=5.0),
        config=default_config,
        verbose=False,
        max_time=220.0,
    )
    final = history[-1]

    assert final["x"] == pytest.approx(target_pos[0])
    assert final["y"] == pytest.approx(target_pos[1])
    assert final["z"] == pytest.approx(target_pos[2])
    assert final["vx"] == pytest.approx(0.0)
    assert final["vy"] == pytest.approx(0.0)
    assert final["vz"] == pytest.approx(0.0)
    assert {round(h["p_ref_x"], 3) for h in history} == {target_pos[0]}
    assert {round(h["p_ref_y"], 3) for h in history} == {target_pos[1]}
    assert {round(h["p_ref_z"], 3) for h in history} == {target_pos[2]}
    assert all(h["v_ref_x"] == pytest.approx(0.0) for h in history)
    assert all(h["v_ref_y"] == pytest.approx(0.0) for h in history)
    assert all(h["v_ref_z"] == pytest.approx(0.0) for h in history)
    assert {event[1] for event in events} == {"x", "y", "z"}


def test_realistic_plant_adds_lag_disturbance_and_measurement_noise(default_config):
    history, _ = run_simulation(
        target_pos=(8.0, 6.0, 1.5),
        initial_state=CraneState(x0=0.0, y0=0.0, z0=5.0),
        config=default_config,
        verbose=False,
        max_time=220.0,
    )

    assert max(abs(h["vx_cmd"] - h["vx"]) for h in history) > 0.005
    assert max(abs(h["disturbance_x"]) for h in history) > 0.001
    assert max(abs(h["x_measured"] - h["x"]) for h in history) > 1e-5


def test_visualizer_writes_control_and_operation_plots(default_config, tmp_path):
    initial_pos = (0.0, 0.0, 5.0)
    target_pos = (8.0, 6.0, 1.5)
    history, events = run_simulation(
        target_pos=target_pos,
        initial_state=CraneState(*initial_pos),
        config=default_config,
        verbose=False,
        max_time=220.0,
    )
    visualizer = CraneVisualizer(default_config)

    control_path = tmp_path / "control.png"
    diagram_path = tmp_path / "operation.png"
    visualizer.plot(history, events, save_path=control_path)
    visualizer.plot_operation_diagram(
        history=history,
        phase_boundaries=events,
        target_pos=target_pos,
        initial_pos=initial_pos,
        save_path=diagram_path,
    )

    for path in (control_path, diagram_path):
        assert path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
        assert path.stat().st_size > 10_000


def test_live_payload_uses_10hz_frames_for_single_target(default_config):
    initial_pos = (0.0, 0.0, 5.0)
    target_pos = (8.0, 6.0, 1.5)
    history, events = run_simulation(
        target_pos=target_pos,
        initial_state=CraneState(*initial_pos),
        config=default_config,
        verbose=False,
        max_time=220.0,
    )

    payload = build_live_payload(
        history=history,
        phase_boundaries=events,
        target_pos=target_pos,
        initial_pos=initial_pos,
        config=default_config,
        update_hz=10.0,
        speed=1.0,
    )

    expected_frames = int(history[-1]["t"] * 10.0) + 2
    assert payload["updateHz"] == 10.0
    assert payload["framePeriodMs"] == pytest.approx(100.0)
    assert payload["target"] == {"x": target_pos[0], "y": target_pos[1], "z": target_pos[2]}
    assert payload["frames"][-1]["x"] == pytest.approx(target_pos[0])
    assert payload["frames"][-1]["y"] == pytest.approx(target_pos[1])
    assert payload["frames"][-1]["z"] == pytest.approx(target_pos[2])
    assert abs(len(payload["frames"]) - expected_frames) <= 2


def test_live_display_rejects_invalid_hz(default_config):
    visualizer = CraneVisualizer(default_config)

    with pytest.raises(ValueError, match="update_hz"):
        visualizer.live_frame_indices([{"t": 0.0}], update_hz=0)


def test_browser_live_html_contains_canvas_bootstrap():
    html = render_live_html()

    assert "<canvas" in html
    assert "simulation.json" in html
    assert "requestAnimationFrame" in html
    assert "Crane Live Control" in html
    assert "drawCraneBay" in html
    assert "drawTrolleyCloseup" in html
    assert "Trolley Movement" in html
    assert "!s.error && !s.stopped" in html


def test_simulation_has_timeout_guard(default_config):
    with pytest.raises(TimeoutError, match="did not finish"):
        run_simulation(
            target_pos=(8.0, 6.0, 1.5),
            initial_state=CraneState(x0=0.0, y0=0.0, z0=5.0),
            config=default_config,
            verbose=False,
            max_time=0.05,
        )
