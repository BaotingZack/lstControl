"""Rigid 3D transform between the SLAM map and physical crane axes."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple


Matrix3 = Tuple[
    Tuple[float, float, float],
    Tuple[float, float, float],
    Tuple[float, float, float],
]
Vector3 = Tuple[float, float, float]


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
    # 定位反馈 X/Y 与行车/PLC 坐标系整体相反时, 在 map→crane 之后对 XY 取负
    # (仅影响 /localization_pose 反馈与由此派生的目标/展示, 不替代逐轴 drive invert)。
    localization_negate_xy: bool = False

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

    def _localization_negate_crane_xy(
        self,
        crane_x: float,
        crane_y: float,
    ) -> tuple[float, float]:
        if self.localization_negate_xy:
            return -crane_x, -crane_y
        return crane_x, crane_y

    def localization_map_to_crane_position(
        self,
        map_x: float,
        map_y: float,
        map_z: float,
    ) -> Vector3:
        """Map→crane for /localization_pose feedback (optional XY sign fix)."""
        crane_x, crane_y, crane_z = self.map_to_crane_position(map_x, map_y, map_z)
        crane_x, crane_y = self._localization_negate_crane_xy(crane_x, crane_y)
        return crane_x, crane_y, crane_z

    def localization_map_to_crane_vector(
        self,
        map_vx: float,
        map_vy: float,
    ) -> tuple[float, float]:
        crane_vx, crane_vy = self.map_to_crane_vector(map_vx, map_vy)
        return self._localization_negate_crane_xy(crane_vx, crane_vy)

    def localization_map_to_crane_vector3(
        self,
        map_vx: float,
        map_vy: float,
        map_vz: float,
    ) -> Vector3:
        crane_vx, crane_vy, crane_vz = self.map_to_crane_vector3(map_vx, map_vy, map_vz)
        crane_vx, crane_vy = self._localization_negate_crane_xy(crane_vx, crane_vy)
        return crane_vx, crane_vy, crane_vz

    def localization_crane_to_map_point(
        self,
        crane_x: float,
        crane_y: float,
    ) -> tuple[float, float]:
        """Inverse of localization_map_to_crane for XY (control loop → map UI)."""
        crane_x, crane_y = self._localization_negate_crane_xy(crane_x, crane_y)
        return self.crane_to_map_point(crane_x, crane_y)

    def localization_crane_to_map_vector(
        self,
        crane_vx: float,
        crane_vy: float,
    ) -> tuple[float, float]:
        crane_vx, crane_vy = self._localization_negate_crane_xy(crane_vx, crane_vy)
        return self.crane_to_map_vector(crane_vx, crane_vy)

    def localization_pose_dict_for_display(self, pose: dict) -> dict:
        """Map-frame /localization_pose → X Bridge / Y Trolley 读数 (与控制环一致)。"""
        out = dict(pose)
        map_z = float(pose['z'])
        crane_x, crane_y, crane_z = self.localization_map_to_crane_position(
            float(pose['x']),
            float(pose['y']),
            map_z,
        )
        out['x'] = crane_x
        out['y'] = crane_y
        # Z 常为抓钩实测高度, 不参与 XY 符号修正
        out['z'] = map_z
        map_vx = pose.get('vx')
        map_vy = pose.get('vy')
        if map_vx is not None and map_vy is not None:
            crane_vx, crane_vy = self.localization_map_to_crane_vector(
                float(map_vx),
                float(map_vy),
            )
            out['vx'] = crane_vx
            out['vy'] = crane_vy
        return out

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

    # ------------------------------------------------------------------
    # 抓钩高度 (物理 Z, 地面=0) 与 SLAM map 3D 旋转/平移变换解耦的辅助方法。
    #
    # 背景: 当 Z 反馈改用 PLC 抓钩实测高度 (GetActualLiftHeight) 时, 该 Z 值
    # 是与 SLAM 地图完全独立的物理量, 不应再套用 map↔crane 的 3D 旋转/平移。
    # 若目标 Z 仍走全量 map_to_crane_position (含 origin_map_z 平移、
    # roll/pitch 旋转), 会与"反馈直接用抓钩高度"产生参考系不一致——现场
    # 只要标定了 origin_map_z 或 roll/pitch, 二者就会出现常数级偏差,
    # 表现为 PD 收敛到一个和真实目标差很远的高度就提前结束 (v=0)。
    # ------------------------------------------------------------------

    def map_to_crane_target(
        self,
        map_x: float,
        map_y: float,
        z_value: float,
        *,
        z_is_hoist_height: bool = False,
    ) -> Vector3:
        """把网页/CLI 输入的目标点转换为 crane 坐标。

        z_is_hoist_height=True 时, z_value 就是抓钩离地高度 (物理量), 直接
        原样返回, 不参与旋转/平移; X/Y 仍走标定变换 (假定目标与标定原点
        位于同一地面高度, 与 is_planar 场景下的行为一致)。

        localization_negate_xy=True 时, 对已按 map 表示的目标 XY 做与反馈相同的取负。
        """
        if z_is_hoist_height:
            crane_x, crane_y = self.map_to_crane_point(map_x, map_y)
            crane_x, crane_y = self._localization_negate_crane_xy(crane_x, crane_y)
            return (crane_x, crane_y, float(z_value))
        crane_x, crane_y, crane_z = self.map_to_crane_position(map_x, map_y, z_value)
        crane_x, crane_y = self._localization_negate_crane_xy(crane_x, crane_y)
        return (crane_x, crane_y, crane_z)

    def ui_bridge_target_to_crane(
        self,
        bridge_x: float,
        bridge_y: float,
        z_value: float,
        *,
        z_is_hoist_height: bool = False,
    ) -> Vector3:
        """网页 Apply Target: X/Y 与侧栏 Bridge/Trolley 读数同系 (非 SLAM map)。"""
        if not self.localization_negate_xy:
            return self.map_to_crane_target(
                bridge_x, bridge_y, z_value, z_is_hoist_height=z_is_hoist_height,
            )
        if z_is_hoist_height:
            return (float(bridge_x), float(bridge_y), float(z_value))
        _, _, crane_z = self.map_to_crane_position(bridge_x, bridge_y, z_value)
        return (float(bridge_x), float(bridge_y), crane_z)

    def crane_to_map_display(
        self,
        crane_x: float,
        crane_y: float,
        z_value: float,
        *,
        z_is_hoist_height: bool = False,
    ) -> Vector3:
        """把 crane 坐标转换回网页展示用的坐标 (map_to_crane_target 的逆操作)。"""
        if z_is_hoist_height:
            map_x, map_y = self.localization_crane_to_map_point(crane_x, crane_y)
            return (map_x, map_y, z_value)
        crane_x, crane_y = self._localization_negate_crane_xy(crane_x, crane_y)
        return self.crane_to_map_position(crane_x, crane_y, z_value)

    def control_step_to_map(
        self,
        crane_step: dict,
        *,
        z_is_hoist_height: bool = False,
    ) -> dict:
        """Convert a control-loop snapshot back to map coordinates for the UI.

        z_is_hoist_height=True 时, 位置/速度的 Z 通道保持原样 (抓钩高度,
        与地图旋转无关)，只转换 X/Y；否则按完整 3D 变换转换 X/Y/Z。
        """
        map_step = dict(crane_step)
        for x_key, y_key, z_key in (
            ('x', 'y', 'z'),
            ('p_ref_x', 'p_ref_y', 'p_ref_z'),
            ('x_measured', 'y_measured', 'z_measured'),
        ):
            if all(key in crane_step for key in (x_key, y_key, z_key)):
                if z_is_hoist_height:
                    map_step[x_key], map_step[y_key] = self.localization_crane_to_map_point(
                        crane_step[x_key],
                        crane_step[y_key],
                    )
                else:
                    cx, cy = self._localization_negate_crane_xy(
                        crane_step[x_key], crane_step[y_key],
                    )
                    map_step[x_key], map_step[y_key], map_step[z_key] = (
                        self.crane_to_map_position(
                            cx,
                            cy,
                            crane_step[z_key],
                        )
                    )
            elif x_key in crane_step and y_key in crane_step:
                map_step[x_key], map_step[y_key] = self.localization_crane_to_map_point(
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
                if z_is_hoist_height:
                    map_step[x_key], map_step[y_key] = self.localization_crane_to_map_vector(
                        crane_step[x_key],
                        crane_step[y_key],
                    )
                else:
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
