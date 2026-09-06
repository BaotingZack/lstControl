"""绳长模型 + 速度模式摆角闭环防摇控制器。

闭环防摇核心: 把摆角 θ 反馈成行车速度修正量 Δv, 叠加到现有 PD 速度指令上,
给摆系统增加等效阻尼:

    Δv = +K(L)·θ

物理依据 (能量法): 摆能 E = ½L²θ̇² + ½gLθ², 沿轨迹求导 dE/dt = −L·θ̇·ẍ。
速度模式下 Δv = +K·θ ⇒ ẍ = K·θ̇ ⇒ dE/dt = −L·K·θ̇² < 0 (阻尼)。
故阻尼项必须"速度 ∝ 摆角、符号为正" (追载荷), 不能写成 −K·θ 或 −K·θ̇:

  - −K·θ  → ẍ = −K·θ̇ → dE/dt = +L·K·θ̇² > 0 (失稳);
  - −K·θ̇ → ẍ = −K·θ̈ → 只改等效惯量, 不产生阻尼。

增益 K 随绳长 L 调度 (固有频率 ωn=√(g/L) 时变)。

机械背景 (钢卷抓钩, 2:1 动滑轮 + V 形双绳悬挂)
---------------------------------------------
抓钩经钢缆绕过其顶部动滑轮悬挂, 两根钢缆向上分别连到升降电机(卷筒)与固定
锚点, 构成 V 形悬挂。有效摆长 = 顶部出绳点到钢卷质心的垂直距离:

    L_eff = (H_sheave - Z) + h_com + ΔL_stretch

其中 Z 为抓钩实测高度(GetActualLiftHeight)。2:1 倍率只影响卷筒↔抓钩位移
映射, 因直接用绝对高度 Z, 该倍率已被吸收; L_eff 最终由 V0 自由摆标定兜底。
V 形悬挂还约束了抓钩绕钢缆的转动, 降低了双摆风险。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RopeLengthModel:
    """由抓钩高度 Z 计算有效摆长 L_eff。"""

    sheave_height: float          # 出绳点固定高度 H_sheave [m]
    grab_offset: float = 0.0      # 抓钩挂点→钢卷质心固定偏移 h_com [m]
    cable_stretch: float = 0.0    # 钢缆载荷伸长 ΔL_stretch [m]
    min_length: float = 0.5       # L_eff 下限保护 [m]

    def compute(self, z_grab: float) -> float:
        L = self.sheave_height - z_grab + self.grab_offset + self.cable_stretch
        return max(L, self.min_length)


class AntiSwayController:
    """速度模式摆角闭环防摇: 增益随绳长调度的速度补偿 Δv = +K(L)·θ。

    gain_schedule 为空时使用固定增益 sway_gain; 否则按 ((L, gain), ...) 的断点
    (L 严格递增) 线性插值调度增益, 适配变绳长工况。
    """

    def __init__(
        self,
        rope_model: RopeLengthModel,
        sway_gain: float = 0.0,
        gain_schedule: tuple = (),
        max_correction: float = 0.05,
    ):
        if max_correction < 0:
            raise ValueError('max_correction must be non-negative')
        self.rope_model = rope_model
        self._gain_fixed = sway_gain
        self.max_correction = max_correction

        self._schedule = tuple(sorted(gain_schedule, key=lambda g: g[0]))
        for a, b in zip(self._schedule, self._schedule[1:]):
            if b[0] <= a[0]:
                raise ValueError('gain_schedule rope lengths must be strictly increasing')

    def _gain(self, L: float) -> float:
        if not self._schedule:
            return self._gain_fixed
        if L <= self._schedule[0][0]:
            return self._schedule[0][1]
        if L >= self._schedule[-1][0]:
            return self._schedule[-1][1]
        for (l0, g0), (l1, g1) in zip(self._schedule, self._schedule[1:]):
            if l0 <= L <= l1:
                f = (L - l0) / (l1 - l0)
                return g0 + f * (g1 - g0)
        return self._gain_fixed

    def compute(
        self,
        theta_x: float,
        theta_y: float,
        z_grab: float,
    ) -> tuple[float, float, float]:
        """计算防摇速度修正量。

        Args:
            theta_x/theta_y: 摆角 [rad] (正 = 载荷朝 +x/+y 偏移)
            z_grab: 抓钩实测高度 [m]

        Returns:
            (dv_x, dv_y, L_eff) [m/s, m/s, m]
        """
        L = self.rope_model.compute(z_grab)
        k = self._gain(L)
        dv_x = k * theta_x
        dv_y = k * theta_y
        dv_x = max(-self.max_correction, min(self.max_correction, dv_x))
        dv_y = max(-self.max_correction, min(self.max_correction, dv_y))
        return dv_x, dv_y, L


def build_anti_sway(config) -> AntiSwayController | None:
    """从配置对象 (CraneConfig) 构建摆角闭环防摇控制器。

    未启用 (config.enable_anti_sway 为假) 时返回 None。这里用鸭子类型读取配置
    字段, 避免 sway_controller 反向依赖 crane_model。
    """
    if not getattr(config, 'enable_anti_sway', False):
        return None
    rope = RopeLengthModel(
        sheave_height=config.rope_length_sheave_height,
        grab_offset=config.rope_length_grab_offset,
        cable_stretch=config.rope_length_cable_stretch,
        min_length=config.rope_length_min,
    )
    return AntiSwayController(
        rope,
        sway_gain=config.anti_sway_sway_gain,
        gain_schedule=config.anti_sway_gain_schedule,
        max_correction=config.anti_sway_max_correction,
    )
