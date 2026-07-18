"""Calibration of the physical crane frame from 3D SLAM rail movements."""

from __future__ import annotations

from dataclasses import dataclass
import math

from coordinate_transform import CoordinateTransform2D, Matrix3, Vector3


class CalibrationError(ValueError):
    """Raised when a calibration observation cannot define a safe transform."""


@dataclass(frozen=True)
class CalibrationObservation:
    """Three 3D map poses produced by known crane +X then +Y movements."""

    start_map: tuple[float, ...]
    after_forward_map: tuple[float, ...]
    after_lateral_map: tuple[float, ...]
    forward_distance: float
    lateral_distance: float


@dataclass(frozen=True)
class CalibrationResult:
    transform: CoordinateTransform2D
    forward_scale: float
    lateral_scale: float
    orthogonality_error_deg: float
    residual_rms: float
    ground_tilt_deg: float

    def cli_args(self) -> str:
        def _formatted(value: float) -> str:
            return f'{0.0 if abs(value) < 0.5e-6 else value:.6f}'

        return (
            f'--map-to-crane-origin-x {_formatted(self.transform.origin_map_x)} '
            f'--map-to-crane-origin-y {_formatted(self.transform.origin_map_y)} '
            f'--map-to-crane-origin-z {_formatted(self.transform.origin_map_z)} '
            f'--map-to-crane-roll-deg {_formatted(self.transform.crane_roll_deg)} '
            f'--map-to-crane-pitch-deg {_formatted(self.transform.crane_pitch_deg)} '
            f'--map-to-crane-yaw-deg '
            f'{_formatted(self.transform.crane_x_axis_yaw_deg)}'
        )

    def as_dict(self) -> dict:
        return {
            'transform': self.transform.as_dict(),
            'forwardScale': self.forward_scale,
            'lateralScale': self.lateral_scale,
            'orthogonalityErrorDeg': self.orthogonality_error_deg,
            'residualRms': self.residual_rms,
            'groundTiltDeg': self.ground_tilt_deg,
            'cliArgs': self.cli_args(),
        }


def _point3(point: tuple[float, ...], label: str) -> Vector3:
    if len(point) != 3:
        raise CalibrationError(
            f'{label} must contain three-dimensional SLAM x, y, and z'
        )
    result = tuple(float(value) for value in point)
    if not all(math.isfinite(value) for value in result):
        raise CalibrationError('calibration values must be finite')
    return result


def _subtract(end: Vector3, start: Vector3) -> Vector3:
    return tuple(end[index] - start[index] for index in range(3))


def _add(first: Vector3, second: Vector3) -> Vector3:
    return tuple(first[index] + second[index] for index in range(3))


def _scale(vector: Vector3, factor: float) -> Vector3:
    return tuple(value * factor for value in vector)


def _dot(first: Vector3, second: Vector3) -> float:
    return sum(first[index] * second[index] for index in range(3))


def _cross(first: Vector3, second: Vector3) -> Vector3:
    return (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )


def _norm(vector: Vector3) -> float:
    return math.sqrt(_dot(vector, vector))


def _normalize(vector: Vector3, label: str) -> Vector3:
    length = _norm(vector)
    if length < 1e-9:
        raise CalibrationError(f'{label} movement must be non-zero')
    return _scale(vector, 1.0 / length)


def _clamp_unit(value: float) -> float:
    return max(-1.0, min(1.0, value))


def _rotation_to_euler(rotation: Matrix3) -> tuple[float, float, float]:
    """Return ZYX roll, pitch, yaw from a right-handed rotation matrix."""
    pitch = math.asin(_clamp_unit(-rotation[2][0]))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        # At gimbal lock yaw and roll are coupled. Choose roll=0 and retain a
        # stable yaw so the represented rotation remains deterministic.
        roll = 0.0
        yaw = math.atan2(-rotation[0][1], rotation[1][1])
    return roll, pitch, yaw


def calibrate_map_to_crane(
    observation: CalibrationObservation,
    *,
    max_orthogonality_error_deg: float = 15.0,
) -> CalibrationResult:
    """Estimate a full rigid map-to-crane transform from two rail runs.

    The known +X and +Y rail displacement vectors define the physical ground
    plane in the SLAM map. Their right-handed cross product defines physical
    +Z, so no hook movement is required to recover map roll and pitch.
    """
    start = _point3(observation.start_map, 'start')
    after_forward = _point3(observation.after_forward_map, 'afterForward')
    after_lateral = _point3(observation.after_lateral_map, 'afterLateral')
    scalar_values = (
        observation.forward_distance,
        observation.lateral_distance,
        max_orthogonality_error_deg,
    )
    if not all(math.isfinite(float(value)) for value in scalar_values):
        raise CalibrationError('calibration values must be finite')
    if abs(observation.forward_distance) < 1e-9:
        raise CalibrationError('forward distance must be non-zero')
    if abs(observation.lateral_distance) < 1e-9:
        raise CalibrationError('lateral distance must be non-zero')
    if max_orthogonality_error_deg <= 0.0:
        raise CalibrationError('maximum orthogonality error must be positive')

    forward_map = _subtract(after_forward, start)
    lateral_map = _subtract(after_lateral, after_forward)
    forward_length = _norm(forward_map)
    lateral_length = _norm(lateral_map)
    if forward_length < 1e-9 or lateral_length < 1e-9:
        raise CalibrationError('measured axis movement must be non-zero')

    # Signed command distance recovers the positive physical rail direction
    # even when an operator performs the run in -X or -Y.
    forward_axis = _normalize(
        _scale(forward_map, 1.0 / observation.forward_distance),
        'forward axis',
    )
    lateral_axis = _normalize(
        _scale(lateral_map, 1.0 / observation.lateral_distance),
        'lateral axis',
    )
    measured_angle = math.acos(_clamp_unit(_dot(forward_axis, lateral_axis)))
    orthogonality_error = abs(math.pi / 2.0 - measured_angle)
    orthogonality_error_deg = math.degrees(orthogonality_error)
    if orthogonality_error_deg > max_orthogonality_error_deg:
        raise CalibrationError(
            f'axis orthogonality error {orthogonality_error_deg:.3f}° exceeds '
            f'{max_orthogonality_error_deg:.3f}°'
        )

    vertical_axis = _normalize(
        _cross(forward_axis, lateral_axis),
        'cross-axis',
    )
    # Symmetric orthogonal fit: the measured X direction and the X direction
    # implied by measured Y + plane normal receive equal weight.
    x_from_lateral = _cross(lateral_axis, vertical_axis)
    fitted_x = _normalize(_add(forward_axis, x_from_lateral), 'fitted X axis')
    fitted_y = _normalize(_cross(vertical_axis, fitted_x), 'fitted Y axis')
    fitted_z = _normalize(_cross(fitted_x, fitted_y), 'fitted Z axis')

    # Rotation columns are the physical crane axes expressed in map space.
    rotation: Matrix3 = (
        (fitted_x[0], fitted_y[0], fitted_z[0]),
        (fitted_x[1], fitted_y[1], fitted_z[1]),
        (fitted_x[2], fitted_y[2], fitted_z[2]),
    )
    roll, pitch, yaw = _rotation_to_euler(rotation)
    transform = CoordinateTransform2D(
        origin_map_x=start[0],
        origin_map_y=start[1],
        origin_map_z=start[2],
        crane_roll_rad=roll,
        crane_pitch_rad=pitch,
        crane_x_axis_yaw_rad=yaw,
    )

    predicted_forward = transform.crane_to_map_position(
        observation.forward_distance,
        0.0,
        0.0,
    )
    predicted_lateral = transform.crane_to_map_position(
        observation.forward_distance,
        observation.lateral_distance,
        0.0,
    )
    forward_residual = _norm(_subtract(after_forward, predicted_forward))
    lateral_residual = _norm(_subtract(after_lateral, predicted_lateral))
    ground_tilt_deg = math.degrees(
        math.acos(_clamp_unit(fitted_z[2]))
    )

    return CalibrationResult(
        transform=transform,
        forward_scale=forward_length / abs(observation.forward_distance),
        lateral_scale=lateral_length / abs(observation.lateral_distance),
        orthogonality_error_deg=orthogonality_error_deg,
        residual_rms=math.sqrt(
            (forward_residual ** 2 + lateral_residual ** 2) / 2.0
        ),
        ground_tilt_deg=ground_tilt_deg,
    )
