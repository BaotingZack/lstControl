"""
运动阶段状态机 — 管理起重机 8 阶段作业序列

完整作业流程:
  Phase 1 (APPROACH):    三轴联动 — X→取货X, Y→取货Y, Z→安全高度
                          Z 先到安全高度则等待 XY
  Phase 2 (DESCEND):     Z 从安全高度降到取货高度
  Phase 3 (GRAB):        抓取延时 (模拟夹具动作)
  Phase 4 (LIFT_CARGO):  Z 带货上升到安全高度
  Phase 5 (TRANSFER):    XY 平移到卸货位置 (Z 保持安全高度)
  Phase 6 (LOWER_CARGO): Z 从安全高度降到卸货高度
  Phase 7 (RELEASE):     释放延时
  Phase 8 (RETURN):      Z 空钩升回初始高度
  DONE:                  作业完成

阶段切换条件:
  - 运动阶段 (P1/P2/P4/P5/P6/P8): 所有运动轴到达目标 (位置+速度容差)
  - 延时阶段 (P3/P7): 当前阶段持续时间 >= 配置延时
"""

from enum import Enum, auto
from crane_model import CraneState, CraneConfig


class MotionPhase(Enum):
    """运动阶段枚举, 每个阶段有中英文标签"""
    APPROACH = auto()      # Phase 1
    DESCEND = auto()       # Phase 2
    GRAB = auto()          # Phase 3
    LIFT_CARGO = auto()    # Phase 4
    TRANSFER = auto()      # Phase 5
    LOWER_CARGO = auto()   # Phase 6
    RELEASE = auto()       # Phase 7
    RETURN = auto()        # Phase 8
    DONE = auto()          # 完成

    @property
    def label(self) -> str:
        """阶段中文标签, 用于控制台输出"""
        labels = {
            MotionPhase.APPROACH:    "Phase 1: 接近取货位置",
            MotionPhase.DESCEND:     "Phase 2: 抓钩下降",
            MotionPhase.GRAB:        "Phase 3: 抓取",
            MotionPhase.LIFT_CARGO:  "Phase 4: 带货上升",
            MotionPhase.TRANSFER:    "Phase 5: 平移至目的地",
            MotionPhase.LOWER_CARGO: "Phase 6: 抓钩下降卸货",
            MotionPhase.RELEASE:     "Phase 7: 释放",
            MotionPhase.RETURN:      "Phase 8: 抓钩归位",
            MotionPhase.DONE:        "完成",
        }
        return labels[self]


class MotionPlanner:
    """运动阶段策划器。

    管理起重机 8 阶段作业序列, 提供:
      - 各阶段各轴的目标位置 get_target()
      - 阶段切换条件判断 check_transition()
      - 安全高度计算
      - 阶段边界时间记录
    """

    def __init__(
        self,
        start_pos: tuple[float, float, float],
        end_pos: tuple[float, float, float],
        initial_state: CraneState,
        config: CraneConfig,
    ):
        """
        Args:
            start_pos:     (sx, sy, sz) 取货位置 [m]
            end_pos:       (ex, ey, ez) 卸货位置 [m]
            initial_state: 起重机初始状态 (用于记录初始 Z 高度)
            config:        控制配置参数
        """
        self.start_pos = start_pos
        self.end_pos = end_pos

        # 安全高度 = max(取货Z, 卸货Z) + 偏移量
        # 确保抓钩带货平移时不碰撞任何障碍物
        self.safe_z = max(start_pos[2], end_pos[2]) + config.safe_height_offset

        # 初始抓钩高度 (Phase 8 作业完成后回到此高度)
        self.initial_z = initial_state.z.position

        self.config = config
        self._arrival_pos_tol = config.arrival_pos_tol   # 到达判断位置容差 [m]
        self._arrival_vel_tol = config.arrival_vel_tol   # 到达判断速度容差 [m/s]

        # 阶段切换时间记录 [(时间, 阶段), ...], 用于可视化和调试
        self.phase_boundaries: list[tuple[float, MotionPhase]] = []

    # ------------------------------------------------------------------
    # 目标位置
    # ------------------------------------------------------------------

    def get_target(self, phase: MotionPhase,
                   current_state: CraneState) -> tuple[float, float, float]:
        """返回当前阶段各轴的目标位置 (tx, ty, tz)。

        不运动的轴返回当前位置 (即保持静止的指令)。

        Args:
            phase:         当前运动阶段
            current_state: 起重机当前状态

        Returns:
            (target_x, target_y, target_z) [m]
        """
        cx = current_state.x.position
        cy = current_state.y.position
        cz = current_state.z.position
        sx, sy, sz = self.start_pos
        ex, ey, ez = self.end_pos

        if phase == MotionPhase.APPROACH:
            # 三轴联动: XY→取货点, Z→安全高度
            return (sx, sy, self.safe_z)
        elif phase == MotionPhase.DESCEND:
            # Z 从安全高度降到取货高度
            return (sx, sy, sz)
        elif phase == MotionPhase.GRAB:
            return (cx, cy, cz)     # 保持
        elif phase == MotionPhase.LIFT_CARGO:
            return (sx, sy, self.safe_z)
        elif phase == MotionPhase.TRANSFER:
            # XY 平移, Z 保持安全高度
            return (ex, ey, self.safe_z)
        elif phase == MotionPhase.LOWER_CARGO:
            # Z 降到卸货高度
            return (ex, ey, ez)
        elif phase == MotionPhase.RELEASE:
            return (cx, cy, cz)     # 保持
        elif phase == MotionPhase.RETURN:
            # Z 升回初始高度
            return (ex, ey, self.initial_z)
        else:
            return (cx, cy, cz)

    # ------------------------------------------------------------------
    # 阶段切换
    # ------------------------------------------------------------------

    def check_transition(
        self,
        phase: MotionPhase,
        state: CraneState,
        current_time: float,
        phase_start_time: float,
    ) -> MotionPhase:
        """检查是否满足阶段切换条件。

        运动阶段: 所有运动轴必须到达目标位置 (位置+速度均在容差内)
        延时阶段: 阶段持续时间 >= 配置延时

        Args:
            phase:            当前阶段
            state:            起重机当前状态
            current_time:     当前仿真时间 [s]
            phase_start_time: 当前阶段开始时间 [s]

        Returns:
            若满足条件返回下一阶段, 否则返回当前阶段
        """
        elapsed = current_time - phase_start_time   # 阶段已用时间 [s]

        if phase == MotionPhase.APPROACH:
            # XY 都到取货点 AND Z 到安全高度 → P2 (下降)
            if (self._axis_arrived(state.z, self.safe_z) and
                    self._axis_arrived(state.x, self.start_pos[0]) and
                    self._axis_arrived(state.y, self.start_pos[1])):
                self.phase_boundaries.append((current_time, MotionPhase.DESCEND))
                return MotionPhase.DESCEND

        elif phase == MotionPhase.DESCEND:
            # Z 到取货高度 → P3 (抓取)
            if self._axis_arrived(state.z, self.start_pos[2]):
                self.phase_boundaries.append((current_time, MotionPhase.GRAB))
                return MotionPhase.GRAB

        elif phase == MotionPhase.GRAB:
            # 延时到 → P4 (带货上升)
            if elapsed >= self.config.grab_delay:
                self.phase_boundaries.append((current_time, MotionPhase.LIFT_CARGO))
                return MotionPhase.LIFT_CARGO

        elif phase == MotionPhase.LIFT_CARGO:
            # Z 到安全高度 → P5 (平移)
            if self._axis_arrived(state.z, self.safe_z):
                self.phase_boundaries.append((current_time, MotionPhase.TRANSFER))
                return MotionPhase.TRANSFER

        elif phase == MotionPhase.TRANSFER:
            # XY 都到卸货点 → P6 (下降卸货)
            if (self._axis_arrived(state.x, self.end_pos[0]) and
                    self._axis_arrived(state.y, self.end_pos[1])):
                self.phase_boundaries.append((current_time, MotionPhase.LOWER_CARGO))
                return MotionPhase.LOWER_CARGO

        elif phase == MotionPhase.LOWER_CARGO:
            # Z 到卸货高度 → P7 (释放)
            if self._axis_arrived(state.z, self.end_pos[2]):
                self.phase_boundaries.append((current_time, MotionPhase.RELEASE))
                return MotionPhase.RELEASE

        elif phase == MotionPhase.RELEASE:
            # 延时到 → P8 (归位)
            if elapsed >= self.config.release_delay:
                self.phase_boundaries.append((current_time, MotionPhase.RETURN))
                return MotionPhase.RETURN

        elif phase == MotionPhase.RETURN:
            # Z 到初始高度 → 完成
            if self._axis_arrived(state.z, self.initial_z):
                self.phase_boundaries.append((current_time, MotionPhase.DONE))
                return MotionPhase.DONE

        return phase  # 条件不满足, 保持当前阶段

    # ------------------------------------------------------------------
    # 辅助方法
    # ------------------------------------------------------------------

    def _axis_arrived(self, axis, target: float) -> bool:
        """判断单轴是否已到达目标位置。

        需同时满足两个条件:
          1. 位置误差 < arrival_pos_tol [m]
          2. 速度绝对值 < arrival_vel_tol [m/s]

        条件 2 防止在位置过零但速度未稳时误判"到达"。

        Args:
            axis:   AxisState 对象 (含 position, velocity)
            target: 目标位置 [m]

        Returns:
            True 表示轴已稳定在目标位置
        """
        pos_ok = abs(axis.position - target) < self._arrival_pos_tol
        vel_ok = abs(axis.velocity) < self._arrival_vel_tol
        return pos_ok and vel_ok
