import math

import pytest

import live_server
import main
import ros_bridge
from crane_model import CraneConfig
from coordinate_transform import CoordinateTransform2D


def test_identity_transform_preserves_points_and_vectors():
    transform = CoordinateTransform2D.identity()

    assert transform.map_to_crane_point(3.0, -2.0) == pytest.approx((3.0, -2.0))
    assert transform.crane_to_map_point(3.0, -2.0) == pytest.approx((3.0, -2.0))
    assert transform.map_to_crane_vector(0.4, -0.2) == pytest.approx((0.4, -0.2))
    assert transform.crane_to_map_vector(0.4, -0.2) == pytest.approx((0.4, -0.2))
    assert transform.map_to_crane_position(3.0, -2.0, 5.0) == pytest.approx((3.0, -2.0, 5.0))
    assert transform.crane_to_map_position(3.0, -2.0, 5.0) == pytest.approx((3.0, -2.0, 5.0))


def test_full_3d_transform_corrects_map_roll_and_yaw():
    transform = CoordinateTransform2D.from_degrees(
        origin_map_x=10.0,
        origin_map_y=20.0,
        origin_map_z=30.0,
        crane_x_axis_yaw_deg=90.0,
        crane_roll_deg=90.0,
        crane_pitch_deg=0.0,
    )

    # Rz(90)Rx(90): crane X->map Y, crane Y->map Z, crane Z->map X.
    assert transform.crane_to_map_position(2.0, 3.0, 4.0) == pytest.approx((14.0, 22.0, 33.0))
    assert transform.map_to_crane_position(14.0, 22.0, 33.0) == pytest.approx((2.0, 3.0, 4.0))
    assert transform.crane_to_map_vector3(2.0, 3.0, 4.0) == pytest.approx((4.0, 2.0, 3.0))
    assert transform.map_to_crane_vector3(4.0, 2.0, 3.0) == pytest.approx((2.0, 3.0, 4.0))


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
    transform = CoordinateTransform2D.from_degrees(
        10.0,
        20.0,
        90.0,
        origin_map_z=5.0,
    )
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
        "vz": 0.2,
        "vz_cmd": -0.1,
    }

    map_step = transform.control_step_to_map(crane_step)

    assert (map_step["x"], map_step["y"]) == pytest.approx((7.0, 22.0))
    assert (map_step["p_ref_x"], map_step["p_ref_y"]) == pytest.approx((5.0, 24.0))
    assert (map_step["x_measured"], map_step["y_measured"]) == pytest.approx((6.5, 22.5))
    assert (map_step["vx"], map_step["vy"]) == pytest.approx((0.0, 1.0))
    assert (map_step["vx_cmd"], map_step["vy_cmd"]) == pytest.approx((0.25, 0.5))
    assert map_step["z"] == pytest.approx(3.5)
    assert map_step["p_ref_z"] == pytest.approx(3.0)
    assert map_step["vz"] == pytest.approx(0.2)
    assert map_step["vz_cmd"] == pytest.approx(-0.1)


def test_ros_position_source_outputs_crane_coordinates_and_ignores_unavailable_z_velocity(monkeypatch):
    transform = CoordinateTransform2D.from_degrees(
        origin_map_x=10.0,
        origin_map_y=20.0,
        origin_map_z=30.0,
        crane_x_axis_yaw_deg=90.0,
        crane_roll_deg=90.0,
    )
    monkeypatch.setattr(
        ros_bridge,
        "get_latest_pose",
        lambda: {
            "x": 14.0,
            "y": 22.0,
            "z": 33.0,
            "vx": 4.0,
            "vy": 2.0,
            "vz": 3.0,
            "stamp_sec": 1,
            "stamp_nsec": 0,
        },
    )
    source = ros_bridge.RosPositionSource(
        coordinate_transform=transform,
        use_native_z_velocity=False,
    )

    position = source.get_position()

    assert (position["x"], position["y"], position["z"]) == pytest.approx((2.0, 3.0, 4.0))
    # A tilted 3D frame needs map Vz to rotate velocity safely. Since native
    # Z velocity is disabled, all axes fall back to position-derived velocity.
    assert position["vx"] is None
    assert position["vy"] is None
    assert position["vz"] is None


def test_ros_position_source_uses_hoist_height_for_z_when_provider_given(monkeypatch):
    # 180° heading flip about Z; Z unaffected by yaw.
    transform = CoordinateTransform2D.from_degrees(crane_x_axis_yaw_deg=180.0)
    monkeypatch.setattr(
        ros_bridge,
        "get_latest_pose",
        lambda: {
            "x": 2.0,
            "y": 3.0,
            "z": 99.0,   # unreliable SLAM Z — must be ignored
            "vx": 0.2,
            "vy": -0.1,
            "vz": 0.5,
            "stamp_sec": 1,
            "stamp_nsec": 0,
        },
    )
    source = ros_bridge.RosPositionSource(
        coordinate_transform=transform,
        lift_height_provider=lambda: 1.75,   # actual hoist encoder height
    )

    position = source.get_position()

    # X/Y from SLAM (flipped by yaw 180), Z from hoist height (not SLAM 99.0).
    assert (position["x"], position["y"]) == pytest.approx((-2.0, -3.0))
    assert position["z"] == pytest.approx(1.75)
    # Native XY velocity still rotated; Z velocity dropped (hoist-diff instead).
    assert (position["vx"], position["vy"]) == pytest.approx((-0.2, 0.1))
    assert position["vz"] is None


def test_ros_position_source_falls_back_to_slam_z_when_hoist_height_unavailable(monkeypatch):
    transform = CoordinateTransform2D.identity()
    monkeypatch.setattr(
        ros_bridge,
        "get_latest_pose",
        lambda: {
            "x": 1.0, "y": 2.0, "z": 3.0,
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "stamp_sec": 1, "stamp_nsec": 0,
        },
    )
    # Provider returns None (e.g. MockPLC / no encoder) -> keep SLAM Z.
    source = ros_bridge.RosPositionSource(
        coordinate_transform=transform,
        lift_height_provider=lambda: None,
    )

    assert source.get_position()["z"] == pytest.approx(3.0)


def test_ros_position_source_rotates_native_xyz_velocity_when_explicitly_trusted(monkeypatch):
    transform = CoordinateTransform2D.from_degrees(
        origin_map_x=10.0,
        origin_map_y=20.0,
        origin_map_z=30.0,
        crane_x_axis_yaw_deg=90.0,
        crane_roll_deg=90.0,
    )
    monkeypatch.setattr(
        ros_bridge,
        "get_latest_pose",
        lambda: {
            "x": 14.0,
            "y": 22.0,
            "z": 33.0,
            "vx": 4.0,
            "vy": 2.0,
            "vz": 3.0,
            "stamp_sec": 1,
            "stamp_nsec": 0,
        },
    )

    position = ros_bridge.RosPositionSource(
        coordinate_transform=transform,
        use_native_z_velocity=True,
    ).get_position()

    assert (position["vx"], position["vy"], position["vz"]) == pytest.approx((2.0, 3.0, 4.0))


def test_cli_builds_map_to_crane_transform():
    args = main._build_arg_parser().parse_args(
        [
            "--map-to-crane-origin-x", "10",
            "--map-to-crane-origin-y", "20",
            "--map-to-crane-origin-z", "30",
            "--map-to-crane-roll-deg", "2",
            "--map-to-crane-pitch-deg", "-3",
            "--map-to-crane-yaw-deg", "90",
        ]
    )

    transform = main._coordinate_transform_from_args(args)

    map_position = transform.crane_to_map_position(2.0, 0.0, 0.0)
    assert transform.map_to_crane_position(*map_position) == pytest.approx((2.0, 0.0, 0.0))
    assert transform.origin_map_z == 30.0
    assert transform.crane_roll_deg == pytest.approx(2.0)
    assert transform.crane_pitch_deg == pytest.approx(-3.0)


def test_web_map_target_is_transformed_before_crane_workspace_validation():
    transform = CoordinateTransform2D.from_degrees(
        origin_map_x=10.0,
        origin_map_y=20.0,
        origin_map_z=30.0,
        crane_x_axis_yaw_deg=90.0,
        crane_roll_deg=90.0,
    )
    config = CraneConfig(
        workspace_x_bounds=(0.0, 3.0),
        workspace_y_bounds=(0.0, 4.0),
        workspace_z_bounds=(0.0, 5.0),
    )
    body = '{"target_x": 14, "target_y": 22, "target_z": 33}'

    assert live_server._parse_control_target(
        body,
        config,
        transform,
    ) == pytest.approx((2.0, 3.0, 4.0))
