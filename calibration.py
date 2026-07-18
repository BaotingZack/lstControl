"""Calibration of crane rail axes from measured SLAM map movements."""

from __future__ import annotations

from dataclasses import dataclass
import math

from coordinate_transform import CoordinateTransform2D


class CalibrationError(ValueError):
    """Raised when a calibration observation cannot define a safe transform."""


@dataclass(frozen=True)
class CalibrationObservation:
    """Three map poses produced by known crane +X then +Y movements."""

    start_map: tuple[float, float]
    after_forward_map: tuple[float, float]
    after_lateral_map: tuple[float, float]
    forward_distance: float
    lateral_distance: float


@dataclass(frozen=True)
class CalibrationResult:
    transform: CoordinateTransform2D
    forward_scale: float
    lateral_scale: float
    orthogonality_error_deg: float
    residual_rms: float

    def cli_args(self) -> str:
        return (
            f'--map-to-crane-origin-x {self.transform.origin_map_x:.6f} '
            f'--map-to-crane-origin-y {self.transform.origin_map_y:.6f} '
            f'--map-to-crane-yaw-deg '
            f'{self.transform.crane_x_axis_yaw_deg:.6f}'
        )

    def as_dict(self) -> dict:
        return {
            'transform': self.transform.as_dict(),
            'forwardScale': self.forward_scale,
            'lateralScale': self.lateral_scale,
            'orthogonalityErrorDeg': self.orthogonality_error_deg,
            'residualRms': self.residual_rms,
            'cliArgs': self.cli_args(),
        }


def _subtract(
    end: tuple[float, float],
    start: tuple[float, float],
) -> tuple[float, float]:
    return end[0] - start[0], end[1] - start[1]


def _norm(vector: tuple[float, float]) -> float:
    return math.hypot(vector[0], vector[1])


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _circular_mean(first: float, second: float) -> float:
    sum_cos = math.cos(first) + math.cos(second)
    sum_sin = math.sin(first) + math.sin(second)
    if math.hypot(sum_cos, sum_sin) < 1e-12:
        raise CalibrationError('axis directions are inconsistent')
    return math.atan2(sum_sin, sum_cos)


def calibrate_map_to_crane(
    observation: CalibrationObservation,
    *,
    max_orthogonality_error_deg: float = 15.0,
) -> CalibrationResult:
    """Estimate a rigid map-to-crane transform from two known rail runs."""
    values = (
        *observation.start_map,
        *observation.after_forward_map,
        *observation.after_lateral_map,
        observation.forward_distance,
        observation.lateral_distance,
        max_orthogonality_error_deg,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise CalibrationError('calibration values must be finite')
    if abs(observation.forward_distance) < 1e-9:
        raise CalibrationError('forward distance must be non-zero')
    if abs(observation.lateral_distance) < 1e-9:
        raise CalibrationError('lateral distance must be non-zero')
    if max_orthogonality_error_deg <= 0.0:
        raise CalibrationError('maximum orthogonality error must be positive')

    forward_map = _subtract(
        observation.after_forward_map,
        observation.start_map,
    )
    lateral_map = _subtract(
        observation.after_lateral_map,
        observation.after_forward_map,
    )
    forward_map_length = _norm(forward_map)
    lateral_map_length = _norm(lateral_map)
    if forward_map_length < 1e-9 or lateral_map_length < 1e-9:
        raise CalibrationError('measured axis movement must be non-zero')

    # Divide by the signed commanded distance so negative runs still recover
    # the positive physical axis direction.
    forward_axis = (
        forward_map[0] / observation.forward_distance,
        forward_map[1] / observation.forward_distance,
    )
    lateral_axis = (
        lateral_map[0] / observation.lateral_distance,
        lateral_map[1] / observation.lateral_distance,
    )
    forward_yaw = math.atan2(forward_axis[1], forward_axis[0])
    lateral_yaw = math.atan2(lateral_axis[1], lateral_axis[0])
    yaw_from_lateral = _wrap_angle(lateral_yaw - math.pi / 2.0)
    orthogonality_error = abs(
        _wrap_angle(lateral_yaw - forward_yaw - math.pi / 2.0)
    )
    orthogonality_error_deg = math.degrees(orthogonality_error)
    if orthogonality_error_deg > max_orthogonality_error_deg:
        raise CalibrationError(
            f'axis orthogonality error {orthogonality_error_deg:.3f}° exceeds '
            f'{max_orthogonality_error_deg:.3f}°'
        )

    calibrated_yaw = _circular_mean(forward_yaw, yaw_from_lateral)
    transform = CoordinateTransform2D(
        origin_map_x=float(observation.start_map[0]),
        origin_map_y=float(observation.start_map[1]),
        crane_x_axis_yaw_rad=calibrated_yaw,
    )
    predicted_forward = transform.crane_to_map_point(
        observation.forward_distance,
        0.0,
    )
    predicted_lateral = transform.crane_to_map_point(
        observation.forward_distance,
        observation.lateral_distance,
    )
    forward_residual = _norm(
        _subtract(observation.after_forward_map, predicted_forward)
    )
    lateral_residual = _norm(
        _subtract(observation.after_lateral_map, predicted_lateral)
    )

    return CalibrationResult(
        transform=transform,
        forward_scale=forward_map_length / abs(observation.forward_distance),
        lateral_scale=lateral_map_length / abs(observation.lateral_distance),
        orthogonality_error_deg=orthogonality_error_deg,
        residual_rms=math.sqrt(
            (forward_residual ** 2 + lateral_residual ** 2) / 2.0
        ),
    )
