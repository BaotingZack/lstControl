import json
import math

import pytest

import live_server
from calibration import CalibrationError, CalibrationObservation, calibrate_map_to_crane


def map_point(origin, yaw_deg, crane_x, crane_y):
    yaw = math.radians(yaw_deg)
    return (
        origin[0] + math.cos(yaw) * crane_x - math.sin(yaw) * crane_y,
        origin[1] + math.sin(yaw) * crane_x + math.cos(yaw) * crane_y,
    )


def map_point_3d(origin, roll_deg, pitch_deg, yaw_deg, crane_x, crane_y, crane_z=0.0):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    crane = (crane_x, crane_y, crane_z)
    return tuple(
        origin[row] + sum(rotation[row][column] * crane[column] for column in range(3))
        for row in range(3)
    )


def test_three_dimensional_runs_recover_map_roll_pitch_yaw_and_origin_z():
    origin = (12.5, -4.0, 1.2)
    roll_deg, pitch_deg, yaw_deg = (2.5, -4.0, 17.0)
    observation = CalibrationObservation(
        start_map=origin,
        after_forward_map=map_point_3d(origin, roll_deg, pitch_deg, yaw_deg, 6.0, 0.0),
        after_lateral_map=map_point_3d(origin, roll_deg, pitch_deg, yaw_deg, 6.0, 3.0),
        forward_distance=6.0,
        lateral_distance=3.0,
    )

    result = calibrate_map_to_crane(observation)

    assert result.transform.origin_map_z == pytest.approx(origin[2])
    assert result.transform.crane_roll_deg == pytest.approx(roll_deg)
    assert result.transform.crane_pitch_deg == pytest.approx(pitch_deg)
    assert result.transform.crane_x_axis_yaw_deg == pytest.approx(yaw_deg)
    assert result.ground_tilt_deg > 0.0
    assert result.residual_rms == pytest.approx(0.0, abs=1e-12)


def test_exact_forward_and_lateral_runs_recover_map_to_crane_transform():
    origin = (12.5, -4.0, 0.7)
    yaw_deg = 17.0
    observation = CalibrationObservation(
        start_map=origin,
        after_forward_map=map_point_3d(origin, 0.0, 0.0, yaw_deg, 6.0, 0.0),
        after_lateral_map=map_point_3d(origin, 0.0, 0.0, yaw_deg, 6.0, 3.0),
        forward_distance=6.0,
        lateral_distance=3.0,
    )

    result = calibrate_map_to_crane(observation)

    assert result.transform.origin_map_x == pytest.approx(origin[0])
    assert result.transform.origin_map_y == pytest.approx(origin[1])
    assert result.transform.crane_x_axis_yaw_deg == pytest.approx(yaw_deg)
    assert result.forward_scale == pytest.approx(1.0)
    assert result.lateral_scale == pytest.approx(1.0)
    assert result.orthogonality_error_deg == pytest.approx(0.0)
    assert result.residual_rms == pytest.approx(0.0)


def test_signed_run_distances_preserve_axis_direction():
    origin = (2.0, 3.0, -1.0)
    yaw_deg = -25.0
    observation = CalibrationObservation(
        start_map=origin,
        after_forward_map=map_point_3d(origin, 0.0, 0.0, yaw_deg, -4.0, 0.0),
        after_lateral_map=map_point_3d(origin, 0.0, 0.0, yaw_deg, -4.0, -2.0),
        forward_distance=-4.0,
        lateral_distance=-2.0,
    )

    result = calibrate_map_to_crane(observation)

    assert result.transform.crane_x_axis_yaw_deg == pytest.approx(yaw_deg)
    assert result.forward_scale == pytest.approx(1.0)
    assert result.lateral_scale == pytest.approx(1.0)


def test_zero_distance_or_non_orthogonal_runs_are_rejected():
    with pytest.raises(CalibrationError, match="distance"):
        calibrate_map_to_crane(
            CalibrationObservation((0, 0, 0), (1, 0, 0), (1, 1, 0), 0.0, 1.0)
        )

    with pytest.raises(CalibrationError, match="three-dimensional"):
        calibrate_map_to_crane(
            CalibrationObservation((0, 0), (1, 0), (1, 1), 1.0, 1.0)
        )

    with pytest.raises(CalibrationError, match="orthogonality"):
        calibrate_map_to_crane(
            CalibrationObservation((0, 0, 0), (2, 0, 0), (4, 0.2, 0), 2.0, 2.0),
            max_orthogonality_error_deg=10.0,
        )


def test_calibration_result_serializes_cli_parameters_and_quality_metrics():
    observation = CalibrationObservation(
        start_map=(10.0, 20.0, 1.5),
        after_forward_map=(10.0, 26.0, 1.5),
        after_lateral_map=(7.0, 26.0, 1.5),
        forward_distance=6.0,
        lateral_distance=3.0,
    )

    payload = calibrate_map_to_crane(observation).as_dict()

    assert payload["transform"]["originMapX"] == 10.0
    assert payload["transform"]["originMapY"] == 20.0
    assert payload["transform"]["originMapZ"] == 1.5
    assert payload["transform"]["craneXAxisYawDeg"] == pytest.approx(90.0)
    assert payload["transform"]["craneRollDeg"] == pytest.approx(0.0)
    assert payload["transform"]["cranePitchDeg"] == pytest.approx(0.0)
    assert "--map-to-crane-origin-x 10.000000" in payload["cliArgs"]
    assert "--map-to-crane-origin-z 1.500000" in payload["cliArgs"]
    assert "--map-to-crane-roll-deg 0.000000" in payload["cliArgs"]
    assert "--map-to-crane-pitch-deg 0.000000" in payload["cliArgs"]
    assert "--map-to-crane-yaw-deg 90.000000" in payload["cliArgs"]


def test_calibration_request_parser_validates_json_shape():
    body = json.dumps({
        "start": {"x": 10, "y": 20, "z": 1.5},
        "afterForward": {"x": 10, "y": 26, "z": 1.5},
        "afterLateral": {"x": 7, "y": 26, "z": 1.5},
        "forwardDistance": 6,
        "lateralDistance": 3,
    })

    result = live_server._calibrate_from_request(body)

    assert result.transform.crane_x_axis_yaw_deg == pytest.approx(90.0)
    assert result.transform.origin_map_z == pytest.approx(1.5)
    with pytest.raises(ValueError, match="Invalid calibration"):
        live_server._calibrate_from_request("{}")
    with pytest.raises(ValueError, match="Invalid calibration"):
        live_server._calibrate_from_request(json.dumps({
            "start": {"x": 0, "y": 0},
            "afterForward": {"x": 1, "y": 0},
            "afterLateral": {"x": 1, "y": 1},
            "forwardDistance": 1,
            "lateralDistance": 1,
        }))


def test_calibration_page_contains_simulation_canvas_and_export_flow():
    html = live_server.render_calibration_html()

    assert "Coordinate Calibration Lab" in html
    assert 'id="calibrationCanvas"' in html
    assert "Forward Run" in html
    assert "Lateral Run" in html
    assert "Map Z" in html
    assert "GROUND TILT" in html
    assert "/api/calibrate" in html
    assert "--map-to-crane-roll-deg" in html
    assert "--map-to-crane-pitch-deg" in html
    assert "--map-to-crane-yaw-deg" in html
    assert "requestAnimationFrame" in html
