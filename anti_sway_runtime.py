"""防摇运行时 — PD 与判稳共用的 θ_ac 分离 + 锁轴软阻尼。

解决原架构问题:
  - 锁轴后完全关防摇 → 停稳阶段摆幅最大
  - raw θ 含慢变 bias → 用 θ_ac = θ − θ_dc
  - 锁轴后 +K·θ 开环漂移 → 锚点位移预算
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from sway_controller import AntiSwayController
from sway_guard import SwayGrowthGuard
from sway_settle import (
    GRAVITY,
    SwayAmplitudeTracker,
    ThetaDcTracker,
    fit_sway_bias,
    pendulum_period,
)


def approach_damping_allowed(
    *,
    err_x: float,
    err_y: float,
    vx: float,
    vy: float,
    locked_x: bool,
    locked_y: bool,
    travel_on: bool,
    approach_on: bool,
    approach_pos: float,
    approach_vel: float,
) -> bool:
    """目标附近才允许把 θ_ac 叠进 PD — 高速远距离走行时倾角是假倾角。

    行进全程阻尼 (travel_on) 仍可用, 但现场默认关。
    不把 XY 速度当作门限: SLAM 速度噪声经常 >0.10 m/s, 会把整段接近/下降
    的阻尼全部挡掉, 表现为"到了目标附近还在大幅摆"。恒加速假倾角是慢变
    偏置, 由 compute_dv 的 rate_amp 门限拦住, 不靠车速判断。
    XY 都锁死后 (Z 还在走) 也放行, 否则下降段又回到"锁轴无阻尼"。
    approach_vel 保留给调用方做日志, 不再参与门限。
    """
    del vx, vy, approach_vel
    if travel_on:
        return True
    if not approach_on:
        return False
    if locked_x and locked_y:
        return True
    return math.hypot(err_x, err_y) <= approach_pos


def clamp_damping_axis(dv: float, shift: float, max_shift: float) -> float:
    """位移预算: 朝锚点外推禁止, 朝锚点回拉允许。max_shift<=0 为禁止任何横移。"""
    if max_shift <= 0.0:
        return 0.0
    if abs(shift) < max_shift:
        return dv
    return 0.0 if dv * shift > 0 else dv


@dataclass
class AntiSwayRuntime:
    """统一防摇: 运动叠加 + 锁轴软阻尼。"""

    anti_sway: AntiSwayController
    dc_tau: float = 4.0
    locked_max_shift: float = 0.08
    use_ac: bool = True
    gain_scale: float = 1.0
    # 锚点回中增益 [1/s]: 锁轴期间把车拉回锚点, 抵消 θ 残余零偏被积分成的单向漂移。
    # θ_dc 跟踪器要用 dc_tau 才能学到残余零偏, 这段时间里 θ_ac 一直带着它,
    # Δv=K·θ_ac 积分出的漂移可达预算上限 (残余零偏 1° → 150mm, 见闭环仿真)。
    # 必须远慢于摆频才不会激励摆动: 0.3 对应 3.3s 时间常数, ωn≈1.8rad/s 时
    # kp/ωn≈0.16, 实测放行时间与峰值偏移均不受影响。0=关闭回中。
    recenter_gain: float = 0.0
    # 可选外部零偏跟踪器。不要把行进段的 tracker 接过来: 行进段倾角含加减速
    # 假倾角, 学到的不是静态零偏, 接力后会把车推离目标并激励摆。
    dc_tracker: ThetaDcTracker | None = None
    # 静态零偏 / 恒减速假倾角的 |θ̇|≈0, 用 rate_amp 门限避免把偏置当成摆去推车。
    # 0.15°: 挡住 ~0.3° 慢变假倾角的微分残差, 仍放行 0.3° 以上的真摆。
    amp_floor: float = math.radians(0.15)
    guard: SwayGrowthGuard | None = None
    # New terminal policy: natural-motion qualification and gradual engagement.
    # False is retained only for legacy comparisons and isolated mechanism tests.
    terminal_mode: bool = False

    dc: ThetaDcTracker = field(init=False)
    amp: SwayAmplitudeTracker = field(init=False)
    anchor_xy: tuple[float, float] | None = None
    last_amp: float = 0.0

    def __post_init__(self) -> None:
        self.dc = (
            ThetaDcTracker(tau_s=self.dc_tau)
            if self.dc_tracker is None
            else self.dc_tracker
        )
        self.amp = SwayAmplitudeTracker()
        if self.guard is None:
            self.guard = SwayGrowthGuard()
        from sway_qualification import SwayFeedbackQualifier
        self.qualifier = SwayFeedbackQualifier() if self.terminal_mode else None
        self.blend = [0., 0.]

    def reset(
        self,
        anchor_x: float | None = None,
        anchor_y: float | None = None,
        *,
        keep_dc: bool = False,
    ) -> None:
        """重置锚点 (可选) 与零偏估计。

        keep_dc=True 用于"换锚点但传感器没换"的场合 (如 PD 锁轴后切入悬停判稳):
        锚点是任务量, 零偏是器件量, 把后者一起清掉等于每次悬停都从 0 重新学一遍
        残余零偏, 而这段收敛过程正是单向漂移的来源。
        """
        if not keep_dc:
            self.dc.reset()
            self.amp.reset()
            self.last_amp = 0.0
            self.guard.reset_window()
            if self.qualifier is not None:
                self.qualifier.reset()
            self.blend = [0., 0.]
        if anchor_x is not None and anchor_y is not None:
            self.anchor_xy = (anchor_x, anchor_y)

    def prime_dc(
        self,
        sway_samples: list[tuple[float, float, float]],
        z_grab: float,
    ) -> tuple[float, float] | None:
        """用摆动信号本身解出静态零偏并置入高通 — 免掉零偏标定。

        为什么需要: 静态零偏随库位/高度/载荷变化 (现场实测空载 vector.y 跨库位
        跨度 0.78°, 载货态再整体偏 0.3~0.4°), 标定表扣不干净; 残余零偏透过高通
        会积分成 K·b·τ 的一次性漂移 (0.44° → 7.3cm)。

        摆频由绳长已知, 于是零偏可以直接从 Δθ 的形状里解出来, 不依赖任何标定值。
        解不出来 (样本不足/窗不足 0.6 个周期) 返回 None, 此时保持 0 起算。
        """
        period = pendulum_period(
            z_grab,
            sheave_height=self.anti_sway.rope_model.sheave_height,
            grab_offset=self.anti_sway.rope_model.grab_offset,
        )
        bias = fit_sway_bias(sway_samples, period)
        if bias is None:
            return None
        self.dc.seed(*bias)
        return bias

    def suspend(self, dt: float) -> None:
        """缺测时清除差分/增长观测窗，恢复数据后不能把跨断流跳变当成本拍角速度。"""
        self.amp.reset()
        self.last_amp = 0.
        self.guard.pause(dt)
        if self.qualifier is not None:
            self.qualifier.reset(preserve_revoked=True)
        self.blend = [0., 0.]

    def set_anchor(self, x: float | None = None, y: float | None = None) -> None:
        """更新锚点; 只更新传入的分量, 另一轴保持原值。

        轴锁定时必须调用: 位移预算是"相对锚点"的, 若锚点仍停在出发位置, 锁轴后
        |x − anchor| 会是整段行程 (米级) 远超预算, clamp_damping_axis 只放行朝
        锚点回拉的半个周期 → 阻尼被半波整流成"往出发点漂"。
        """
        cur = self.anchor_xy
        new_x = x if x is not None else (cur[0] if cur is not None else None)
        new_y = y if y is not None else (cur[1] if cur is not None else None)
        if new_x is None or new_y is None:
            return
        self.anchor_xy = (new_x, new_y)

    def compute_dv(
        self,
        theta_x: float,
        theta_y: float,
        z_grab: float,
        dt: float,
        *,
        gain_scale: float | None = None,
    ) -> tuple[float, float, float, float]:
        """返回 (dv_x, dv_y, theta_ac_x, theta_ac_y)。"""
        if dt > .5:
            self.amp.reset()
        ac_x, ac_y = self.dc.update(theta_x, theta_y, dt)
        L = self.anti_sway.rope_model.compute(z_grab)
        omega_n = math.sqrt(GRAVITY / max(L, 0.5))
        _, _, self.last_amp = self.amp.update(
            ac_x, ac_y, dt, omega_n,
            rate_sample_x=theta_x, rate_sample_y=theta_y,
        )
        tx = ac_x if self.use_ac else theta_x
        ty = ac_y if self.use_ac else theta_y
        # 两轴独立：一轴在摆不能释放另一轴的静态残差。
        scale = self.gain_scale if gain_scale is None else gain_scale
        dvx, dvy, _ = self.anti_sway.compute(tx, ty, z_grab)
        if self.amp.last_rate_amp_x < self.amp_floor:
            dvx = 0.0
        if self.amp.last_rate_amp_y < self.amp_floor:
            dvy = 0.0
        return dvx * scale, dvy * scale, ac_x, ac_y

    def merge_xy_commands(
        self,
        vx_cmd: float,
        vy_cmd: float,
        *,
        theta_x: float,
        theta_y: float,
        z_grab: float,
        x: float,
        y: float,
        locked_x: bool,
        locked_y: bool,
        dt: float,
        max_velocity_xy: float,
        gain_scale: float | None = None,
        feedback_allowed: bool = True,
        feedback_axes: tuple[bool, bool] = (True, True),
    ) -> tuple[float, float, float, float]:
        """未锁轴: Δv 叠加 PD; 已锁轴: 仅预算内软阻尼 (vx/vy 由 PD 置 0 后替换)。"""
        if not all(math.isfinite(v) for v in (theta_x, theta_y, z_grab, x, y, dt)) or dt <= 0:
            self.suspend(dt if math.isfinite(dt) and dt > 0 else 0.)
            return vx_cmd, vy_cmd, 0., 0.
        dvx, dvy, _, _ = self.compute_dv(
            theta_x, theta_y, z_grab, dt, gain_scale=gain_scale,
        )
        length = self.anti_sway.rope_model.compute(z_grab)
        period = 2 * math.pi * math.sqrt(length / GRAVITY)
        qualified = ((True, True) if self.qualifier is None else
                     self.qualifier.update(theta_x, theta_y, x, y, length, dt))
        allowed = [feedback_allowed and feedback_axes[a] and qualified[a]
                   and not self.guard.blocked[a] for a in (0, 1)]
        for a in (0, 1):
            self.blend[a] = (min(1., self.blend[a] + dt / (.25 * period))
                             if self.terminal_mode and allowed[a] else float(allowed[a]))
        ax, ay = self.anchor_xy if self.anchor_xy is not None else (x, y)

        if locked_x:
            dvx = self._locked_axis_dv(dvx, x - ax)
            dvx *= self.blend[0]
            vx_out = max(-max_velocity_xy, min(max_velocity_xy, dvx))
        else:
            dvx *= self.blend[0]
            vx_out = max(
                -max_velocity_xy,
                min(max_velocity_xy, vx_cmd + dvx),
            )

        if locked_y:
            dvy = self._locked_axis_dv(dvy, y - ay)
            dvy *= self.blend[1]
            vy_out = max(-max_velocity_xy, min(max_velocity_xy, dvy))
        else:
            dvy *= self.blend[1]
            vy_out = max(
                -max_velocity_xy,
                min(max_velocity_xy, vy_cmd + dvy),
            )
        blocked_x, blocked_y = self.guard.update(
            theta_x, theta_y, dt=dt, period_s=period,
            commanded=(feedback_allowed and abs(vx_out - vx_cmd) > 1e-9,
                       feedback_allowed and abs(vy_out - vy_cmd) > 1e-9),
            feedback_allowed=feedback_allowed,
        )
        # 撤回整条附加通路，包括锁轴回中；其余 PD 命令原样保留。
        if blocked_x or not allowed[0]:
            vx_out, dvx = vx_cmd, 0.0
        if blocked_y or not allowed[1]:
            vy_out, dvy = vy_cmd, 0.0
        return vx_out, vy_out, dvx, dvy

    def _locked_axis_dv(self, dv: float, shift: float) -> float:
        """锁轴单轴修正量: 预算内的软阻尼 + 锚点回中。

        位移预算只钳阻尼项: 回中项永远指向锚点, 把它一起钳掉会让"超出预算后
        再也回不来"。预算为 0 (贴近钢卷) 时横移和回中都禁止。
        """
        if self.locked_max_shift <= 0.0:
            return 0.0
        return clamp_damping_axis(dv, shift, self.locked_max_shift) - (
            self.recenter_gain * shift
        )
