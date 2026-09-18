"""绳长模型 + 速度模式摆角闭环防摇控制器。

闭环防摇核心: 把摆角 θ 反馈成行车速度修正量 Δv, 叠加到现有 PD 速度指令上,
给摆系统增加等效阻尼:

    Δv = +K(L)·θ

物理依据 (能量法): 摆能 E = ½L²θ̇² + ½gLθ², 沿轨迹求导 dE/dt = −L·θ̇·ẍ。
速度模式下 Δv = +K·θ ⇒ ẍ = K·θ̇ ⇒ dE/dt = −L·K·θ̇² < 0 (阻尼)。
故阻尼项必须"速度 ∝ 摆角、符号为正" (小车追着载荷走), 不能写成 −K·θ 或 −K·θ̇:

  - −K·θ  → ẍ = −K·θ̇ → dE/dt = +L·K·θ̇² > 0 (失稳);
  - −K·θ̇ → ẍ = −K·θ̈ → 只改等效惯量, 不产生阻尼。

闭环摆动方程与增益标定
---------------------
把 Δv = K·θ 代入单摆 (L·θ̈ + g·θ = −ẍ) 得

    θ̈ + (K/L)·θ̇ + (g/L)·θ = 0   ⇒   ωn = √(g/L),  ζ = K / (2√(gL))

所以增益按目标阻尼比反解: K = 2ζ·√(gL)。摆幅包络按 exp(−ζ·ωn·t) 衰减,
时间常数 τ_env = 1/(ζ·ωn) = 2L/K, 把摆幅从 θ0 压到 θ1 需要 τ_env·ln(θ0/θ1)。

现场绳长 L≈2.0~3.6m (L_eff=4−Z, 见 site_config.json), √(gL)≈4.4~6.0,
故全行程 K_opt 仅 4.9~5.8 量级, 固定增益即可满足, 不必启用 gain_schedule。

机械背景 (电子葫芦 + 抓钩结构)
-----------------------------
现场起升是电子葫芦, 出绳点离地 H=4m; PLC GetActualLiftHeight 给出抓钩
离地高度 Z; 抓钩工作点到钢丝绳连接处 h=1.5m。由此实时反推:

    L_rope = H − Z − h          # 钢丝绳长度
    L_eff  = H − Z = L_rope + h # 出绳点到抓钩的摆长

site_config 取 sheave_height=4.0、grab_offset=0, 即 L_eff = H − Z。
禁止 L = 4 − Z + 1.5 (把 h 加了两次)。Z 升高则绳缩短。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from sway_settle import GRAVITY, effective_rope_length


@dataclass(frozen=True)
class RopeLengthModel:
    """由抓钩高度 Z 计算有效摆长 L_eff。

    与 sway_settle.effective_rope_length() 共用同一套几何, 保证"控制用的绳长"
    与"判稳窗长/摆周期用的绳长"永远一致 — 两边各写一份迟早会调参调飘。
    """

    sheave_height: float          # 出绳点固定高度 H_sheave [m]
    grab_offset: float = 0.0      # 抓钩挂点→钢卷质心固定偏移 h_com [m]
    cable_stretch: float = 0.0    # 钢缆载荷伸长 ΔL_stretch [m]
    min_length: float = 0.5       # L_eff 下限保护 [m]

    def compute(self, z_grab: float) -> float:
        return effective_rope_length(
            z_grab,
            sheave_height=self.sheave_height,
            grab_offset=self.grab_offset,
            cable_stretch=self.cable_stretch,
            min_length=self.min_length,
        )


def damping_ratio(sway_gain: float, rope_length: float, g: float = GRAVITY) -> float:
    """增益 K 在绳长 L 下带来的等效阻尼比 ζ = K/(2√(gL))。"""
    denom = 2.0 * math.sqrt(g * max(rope_length, 1e-6))
    return sway_gain / denom


def gain_for_damping_ratio(
    zeta: float, rope_length: float, g: float = GRAVITY
) -> float:
    """目标阻尼比 ζ 在绳长 L 下需要的增益 K = 2ζ√(gL)。整定时反解用。"""
    return 2.0 * zeta * math.sqrt(g * max(rope_length, 1e-6))


def envelope_decay_time(
    sway_gain: float,
    rope_length: float,
    theta_from: float,
    theta_to: float,
    g: float = GRAVITY,
) -> float:
    """摆幅由 theta_from 衰减到 theta_to 所需时间 [s] (同单位即可, 取比值)。

    τ_env = 1/(ζ·ωn) = 2L/K; 返回 τ_env·ln(θ_from/θ_to)。用于回答"判稳等待
    上限够不够把 1° 压到 0.3°"这类整定问题, 见 CraneConfig.hook_settle_max_wait。
    """
    if sway_gain <= 0.0 or theta_to <= 0.0 or theta_from <= theta_to:
        return 0.0
    tau_env = 2.0 * max(rope_length, 1e-6) / sway_gain
    return tau_env * math.log(theta_from / theta_to)


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

    未启用 (config.enable_anti_sway 为假) 或增益为 0 时返回 None — 调用方据此
    完全跳过防摇通路, 保证"关掉开关就是原来的纯 PD 行为"。这里用鸭子类型读取
    配置字段, 避免 sway_controller 反向依赖 crane_model。
    """
    if not getattr(config, 'enable_anti_sway', False):
        return None
    if not getattr(config, 'anti_sway_sway_gain', 0.0):
        return None
    rope = RopeLengthModel(
        sheave_height=getattr(config, 'rope_length_sheave_height', 4.0),
        grab_offset=getattr(config, 'rope_length_grab_offset', 0.0),
        cable_stretch=getattr(config, 'rope_length_cable_stretch', 0.0),
        min_length=getattr(config, 'rope_length_min', 0.5),
    )
    return AntiSwayController(
        rope,
        sway_gain=config.anti_sway_sway_gain,
        gain_schedule=tuple(getattr(config, 'anti_sway_gain_schedule', ()) or ()),
        max_correction=getattr(config, 'anti_sway_max_correction', 0.05),
    )
