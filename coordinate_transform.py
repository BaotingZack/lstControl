"""Rigid 2D transform between the SLAM map frame and crane rail axes."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CoordinateTransform2D:
    """Map-frame origin and yaw of the crane +X rail in the SLAM map.

    The crane frame is right-handed: +Y is 90 degrees counter-clockwise from
    +X. Points include origin translation; velocity/error vectors only rotate.
    """

    origin_map_x: float = 0.0
    origin_map_y: float = 0.0
    crane_x_axis_yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        values = (
            self.origin_map_x,
            self.origin_map_y,
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
    ) -> 'CoordinateTransform2D':
        values = (origin_map_x, origin_map_y, crane_x_axis_yaw_deg)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('coordinate transform parameters must be finite')
        return cls(
            origin_map_x=float(origin_map_x),
            origin_map_y=float(origin_map_y),
            crane_x_axis_yaw_rad=math.radians(crane_x_axis_yaw_deg),
        )

    @property
    def crane_x_axis_yaw_deg(self) -> float:
        return math.degrees(self.crane_x_axis_yaw_rad)

    @property
    def _cos_yaw(self) -> float:
        return math.cos(self.crane_x_axis_yaw_rad)

    @property
    def _sin_yaw(self) -> float:
        return math.sin(self.crane_x_axis_yaw_rad)

    def map_to_crane_point(self, map_x: float, map_y: float) -> tuple[float, float]:
        delta_x = map_x - self.origin_map_x
        delta_y = map_y - self.origin_map_y
        return self.map_to_crane_vector(delta_x, delta_y)

    def crane_to_map_point(self, crane_x: float, crane_y: float) -> tuple[float, float]:
        map_delta_x, map_delta_y = self.crane_to_map_vector(crane_x, crane_y)
        return (
            self.origin_map_x + map_delta_x,
            self.origin_map_y + map_delta_y,
        )

    def map_to_crane_vector(self, map_x: float, map_y: float) -> tuple[float, float]:
        """Rotate a map-frame vector into crane rail components (R transpose)."""
        return (
            self._cos_yaw * map_x + self._sin_yaw * map_y,
            -self._sin_yaw * map_x + self._cos_yaw * map_y,
        )

    def crane_to_map_vector(self, crane_x: float, crane_y: float) -> tuple[float, float]:
        """Rotate crane rail components into the SLAM map frame (R)."""
        return (
            self._cos_yaw * crane_x - self._sin_yaw * crane_y,
            self._sin_yaw * crane_x + self._cos_yaw * crane_y,
        )

    def control_step_to_map(self, crane_step: dict) -> dict:
        """Convert a control-loop snapshot back to map coordinates for the UI."""
        map_step = dict(crane_step)
        for x_key, y_key in (
            ('x', 'y'),
            ('p_ref_x', 'p_ref_y'),
            ('x_measured', 'y_measured'),
        ):
            if x_key in crane_step and y_key in crane_step:
                map_step[x_key], map_step[y_key] = self.crane_to_map_point(
                    crane_step[x_key], crane_step[y_key]
                )

        for x_key, y_key in (
            ('vx', 'vy'),
            ('vx_cmd', 'vy_cmd'),
            ('vx_raw', 'vy_raw'),
            ('vx_filtered', 'vy_filtered'),
            ('v_ref_x', 'v_ref_y'),
            ('disturbance_x', 'disturbance_y'),
        ):
            if x_key in crane_step and y_key in crane_step:
                map_step[x_key], map_step[y_key] = self.crane_to_map_vector(
                    crane_step[x_key], crane_step[y_key]
                )
        return map_step

    def as_dict(self) -> dict[str, float]:
        return {
            'originMapX': self.origin_map_x,
            'originMapY': self.origin_map_y,
            'craneXAxisYawDeg': self.crane_x_axis_yaw_deg,
        }
