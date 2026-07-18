import math

import pytest

import main
import ros_bridge
from coordinate_transform import CoordinateTransform2D


def test_identity_transform_preserves_points_and_vectors():
    transform = CoordinateTransform2D.identity()

    assert transform.map_to_crane_point(3.0, -2.0) == pytest.approx((3.0, -2.0))
    assert transform.crane_to_map_point(3.0, -2.0) == pytest.approx((3.0, -2.0))
    assert transform.map_to_crane_vector(0.4, -0.2) == pytest.approx((0.4, -0.2))
    assert transform.crane_to_map_vector(0.4, -0.2) == pytest.approx((0.4, -0.2))


def test_map_points_are_rotated_and_translated_into_crane_axes():
    transform = CoordinateTransform2D.from_degrees(
        origin_map_x=10.0,
        origin_map_y=20.0,
        crane_x_axis_yaw_deg=90.0,
    )

    # In map coordinates, crane +X points north and crane +Y points west.
    assert transform.map_to_crane_point(10.0, 22.0) == pytest.approx((2.0, 0.0))
    assert transform.map_to_crane_point(7.0, 20.0) == pytest.approx((0.0, 3.0))
    assert transform.crane_to_map_point(2.0, 3.0) == pytest.approx((7.0, 22.0))


def test_velocity_vectors_rotate_without_origin_translation():
    transform = CoordinateTransform2D.from_degrees(
        origin_map_x=100.0,
        origin_map_y=-50.0,
        crane_x_axis_yaw_deg=90.0,
    )

    assert transform.map_to_crane_vector(0.0, 1.0) == pytest.approx((1.0, 0.0))
    assert transform.crane_to_map_vector(1.0, 0.0) == pytest.approx((0.0, 1.0))


def test_non_finite_transform_parameters_are_rejected():
    with pytest.raises(ValueError, match="finite"):
        CoordinateTransform2D.from_degrees(math.nan, 0.0, 0.0)
    with pytest.raises(ValueError, match="finite"):
        CoordinateTransform2D.from_degrees(0.0, 0.0, math.inf)


def test_control_step_is_converted_back_to_map_for_browser_display():
    transform = CoordinateTransform2D.from_degrees(10.0, 20.0, 90.0)
    crane_step = {
        "x": 2.0,
        "y": 3.0,
        "z": -1.5,
        "p_ref_x": 4.0,
        "p_ref_y": 5.0,
        "p_ref_z": -2.0,
        "x_measured": 2.5,
        "y_measured": 3.5,
        "vx": 1.0,
        "vy": 0.0,
        "vx_cmd": 0.5,
        "vy_cmd": -0.25,
    }

    map_step = transform.control_step_to_map(crane_step)

    assert (map_step["x"], map_step["y"]) == pytest.approx((7.0, 22.0))
    assert (map_step["p_ref_x"], map_step["p_ref_y"]) == pytest.approx((5.0, 24.0))
    assert (map_step["x_measured"], map_step["y_measured"]) == pytest.approx((6.5, 22.5))
    assert (map_step["vx"], map_step["vy"]) == pytest.approx((0.0, 1.0))
    assert (map_step["vx_cmd"], map_step["vy_cmd"]) == pytest.approx((0.25, 0.5))
    assert map_step["z"] == -1.5


def test_ros_position_source_outputs_crane_coordinates_and_ignores_unavailable_z_velocity(monkeypatch):
    transform = CoordinateTransform2D.from_degrees(10.0, 20.0, 90.0)
    monkeypatch.setattr(
        ros_bridge,
        "get_latest_pose",
        lambda: {
            "x": 7.0,
            "y": 22.0,
            "z": -1.5,
            "vx": 0.0,
            "vy": 1.0,
            "vz": 99.0,
            "stamp_sec": 1,
            "stamp_nsec": 0,
        },
    )
    source = ros_bridge.RosPositionSource(
        coordinate_transform=transform,
        use_native_z_velocity=False,
    )

    position = source.get_position()

    assert (position["x"], position["y"], position["z"]) == pytest.approx((2.0, 3.0, -1.5))
    assert (position["vx"], position["vy"]) == pytest.approx((1.0, 0.0))
    assert position["vz"] is None


def test_cli_builds_map_to_crane_transform():
    args = main._build_arg_parser().parse_args(
        [
            "--map-to-crane-origin-x", "10",
            "--map-to-crane-origin-y", "20",
            "--map-to-crane-yaw-deg", "90",
        ]
    )

    transform = main._coordinate_transform_from_args(args)

    assert transform.map_to_crane_point(10.0, 22.0) == pytest.approx((2.0, 0.0))
