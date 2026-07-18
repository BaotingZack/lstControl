"""Rigid 3D transform between the SLAM map and physical crane axes."""

from __future__ import annotations

from dataclasses import dataclass
import math


Matrix3 = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]
Vector3 = tuple[float, float, float]


def _matrix_vector(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _transpose_vector(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(
        sum(matrix[row][column] * vector[row] for row in range(3))
        for column in range(3)
    )


@dataclass(frozen=True)
class CoordinateTransform2D:
    """Map pose of the physical crane frame.

    The historical class name is retained for API compatibility, but the
    transform is fully three-dimensional. Rotation follows the conventional
    ZYX order: ``R = Rz(yaw) * Ry(pitch) * Rx(roll)``. Points include origin
    translation; velocity and error vectors only rotate.
    """

    # Keep the first three fields in their historical order so existing
    # positional construction ``(origin_x, origin_y, yaw_rad)`` remains valid.
    origin_map_x: float = 0.0
    origin_map_y: float = 0.0
    crane_x_axis_yaw_rad: float = 0.0
    origin_map_z: float = 0.0
    crane_roll_rad: float = 0.0
    crane_pitch_rad: float = 0.0

    def __post_init__(self) -> None:
        values = (
            self.origin_map_x,
            self.origin_map_y,
            self.origin_map_z,
            self.crane_roll_rad,
            self.crane_pitch_rad,
            self.crane_x_axis_yaw_rad,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('coordinate transform parameters must be finite')

    @classmethod
    def identity(cls) -> 'CoordinateTransform2D':
        return cls()

    @classmethod
    def from_degrees(
        cls,
        origin_map_x: float = 0.0,
        origin_map_y: float = 0.0,
        crane_x_axis_yaw_deg: float = 0.0,
        *,
        origin_map_z: float = 0.0,
        crane_roll_deg: float = 0.0,
        crane_pitch_deg: float = 0.0,
    ) -> 'CoordinateTransform2D':
        values = (
            origin_map_x,
            origin_map_y,
            origin_map_z,
            crane_roll_deg,
            crane_pitch_deg,
            crane_x_axis_yaw_deg,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('coordinate transform parameters must be finite')
        return cls(
            origin_map_x=float(origin_map_x),
            origin_map_y=float(origin_map_y),
            origin_map_z=float(origin_map_z),
            crane_roll_rad=math.radians(crane_roll_deg),
            crane_pitch_rad=math.radians(crane_pitch_deg),
            crane_x_axis_yaw_rad=math.radians(crane_x_axis_yaw_deg),
        )

    @property
    def crane_x_axis_yaw_deg(self) -> float:
        return math.degrees(self.crane_x_axis_yaw_rad)

    @property
    def crane_roll_deg(self) -> float:
        return math.degrees(self.crane_roll_rad)

    @property
    def crane_pitch_deg(self) -> float:
        return math.degrees(self.crane_pitch_rad)

    @property
    def is_planar(self) -> bool:
        """Whether map Z is decoupled from crane X/Y velocity."""
        return (
            abs(self.crane_roll_rad) < 1e-12
            and abs(self.crane_pitch_rad) < 1e-12
        )

    @property
    def rotation_matrix(self) -> Matrix3:
        """Return R mapping crane-frame vectors into the SLAM map."""
        cr = math.cos(self.crane_roll_rad)
        sr = math.sin(self.crane_roll_rad)
        cp = math.cos(self.crane_pitch_rad)
        sp = math.sin(self.crane_pitch_rad)
        cy = math.cos(self.crane_x_axis_yaw_rad)
        sy = math.sin(self.crane_x_axis_yaw_rad)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )

    def map_to_crane_position(
        self,
        map_x: float,
        map_y: float,
        map_z: float,
    ) -> Vector3:
        delta = (
            map_x - self.origin_map_x,
            map_y - self.origin_map_y,
            map_z - self.origin_map_z,
        )
        return self.map_to_crane_vector3(*delta)

    def crane_to_map_position(
        self,
        crane_x: float,
        crane_y: float,
        crane_z: float,
    ) -> Vector3:
        delta = self.crane_to_map_vector3(crane_x, crane_y, crane_z)
        return (
            self.origin_map_x + delta[0],
            self.origin_map_y + delta[1],
            self.origin_map_z + delta[2],
        )

    def map_to_crane_vector3(
        self,
        map_x: float,
        map_y: float,
        map_z: float,
    ) -> Vector3:
        """Rotate a map-frame vector into physical crane components (R^T)."""
        return _transpose_vector(self.rotation_matrix, (map_x, map_y, map_z))

    def crane_to_map_vector3(
        self,
        crane_x: float,
        crane_y: float,
        crane_z: float,
    ) -> Vector3:
        """Rotate a physical crane vector into the SLAM map (R)."""
        return _matrix_vector(
            self.rotation_matrix,
            (crane_x, crane_y, crane_z),
        )

    # Legacy planar helpers remain useful for callers that intentionally only
    # have XY data. Full control paths use the explicit 3D methods above.
    def map_to_crane_point(self, map_x: float, map_y: float) -> tuple[float, float]:
        crane = self.map_to_crane_position(map_x, map_y, self.origin_map_z)
        return crane[0], crane[1]

    def crane_to_map_point(self, crane_x: float, crane_y: float) -> tuple[float, float]:
        mapped = self.crane_to_map_position(crane_x, crane_y, 0.0)
        return mapped[0], mapped[1]

    def map_to_crane_vector(self, map_x: float, map_y: float) -> tuple[float, float]:
        crane = self.map_to_crane_vector3(map_x, map_y, 0.0)
        return crane[0], crane[1]

    def crane_to_map_vector(self, crane_x: float, crane_y: float) -> tuple[float, float]:
        mapped = self.crane_to_map_vector3(crane_x, crane_y, 0.0)
        return mapped[0], mapped[1]

    def control_step_to_map(self, crane_step: dict) -> dict:
        """Convert a control-loop snapshot back to map coordinates for the UI."""
        map_step = dict(crane_step)
        for x_key, y_key, z_key in (
            ('x', 'y', 'z'),
            ('p_ref_x', 'p_ref_y', 'p_ref_z'),
            ('x_measured', 'y_measured', 'z_measured'),
        ):
            if all(key in crane_step for key in (x_key, y_key, z_key)):
                map_step[x_key], map_step[y_key], map_step[z_key] = (
                    self.crane_to_map_position(
                        crane_step[x_key],
                        crane_step[y_key],
                        crane_step[z_key],
                    )
                )
            elif x_key in crane_step and y_key in crane_step:
                map_step[x_key], map_step[y_key] = self.crane_to_map_point(
                    crane_step[x_key],
                    crane_step[y_key],
                )

        for x_key, y_key, z_key in (
            ('vx', 'vy', 'vz'),
            ('vx_cmd', 'vy_cmd', 'vz_cmd'),
            ('vx_raw', 'vy_raw', 'vz_raw'),
            ('vx_filtered', 'vy_filtered', 'vz_filtered'),
            ('v_ref_x', 'v_ref_y', 'v_ref_z'),
            ('disturbance_x', 'disturbance_y', 'disturbance_z'),
        ):
            if all(key in crane_step for key in (x_key, y_key, z_key)):
                map_step[x_key], map_step[y_key], map_step[z_key] = (
                    self.crane_to_map_vector3(
                        crane_step[x_key],
                        crane_step[y_key],
                        crane_step[z_key],
                    )
                )
        return map_step

    def as_dict(self) -> dict[str, float]:
        return {
            'originMapX': self.origin_map_x,
            'originMapY': self.origin_map_y,
            'originMapZ': self.origin_map_z,
            'craneRollDeg': self.crane_roll_deg,
            'cranePitchDeg': self.crane_pitch_deg,
            'craneYawDeg': self.crane_x_axis_yaw_deg,
            # Retain the original API field used by deployed clients.
            'craneXAxisYawDeg': self.crane_x_axis_yaw_deg,
        }


# Clearer name for new integrations without breaking existing imports.
CoordinateTransform3D = CoordinateTransform2D
