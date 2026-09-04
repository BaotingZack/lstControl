"""输入整形 (Input Shaping): ZV / ZVD / EI。

把一条运动命令卷积成若干幅值/时延的子命令, 使各子命令激发的振动互相抵消。
PD 抑制已产生的摆动; 输入整形在命令进入系统前降低对摆动模态的激励, 两者互补。

零阻尼 (ζ=0) 下的标准脉冲序列 (T 为摆周期):

    ZV  : [1/2, 1/2]            @ t = 0, T/2
    ZVD : [1/4, 1/2, 1/4]       @ t = 0, T/2, T
    EI  : 三脉冲, 在允许残余振动内进一步提鲁棒性

含阻尼 ζ 的一般式 (K = exp(-ζπ/√(1-ζ²)), ΔT = π/(ωn√(1-ζ²)) ≈ T/2):

    ZV  : A = [1/(1+K), K/(1+K)]
    ZVD : A = [1/(1+2K+K²), 2K/(1+2K+K²), K²/(1+2K+K²)]   @ t = 0, ΔT, 2ΔT

注意: 输入整形是前馈技术, 作用于参考速度/位置轨迹。当前 run_pd_control 为纯
反馈 PD (无前馈参考), 本模块作为独立可测组件提供; 待 S 曲线前馈接入控制环后
再对 v_ref 整形 (见防摇方案文档 §5.3)。
"""

from __future__ import annotations

import math


def _damping_params(zeta: float) -> tuple[float, float]:
    if not 0.0 <= zeta < 1.0:
        raise ValueError('zeta must be in [0, 1)')
    if zeta == 0.0:
        return 1.0, math.pi
    K = math.exp(-zeta * math.pi / math.sqrt(1.0 - zeta * zeta))
    delta = math.pi / math.sqrt(1.0 - zeta * zeta)
    return K, delta


def _impulse_times(T: float, zeta: float, count: int) -> list[float]:
    """第 i 个脉冲时刻 = i·ΔT, ΔT = T·delta/(2π) (delta = π/√(1-ζ²), ζ=0 时 ΔT=T/2)。"""
    if T < 0:
        raise ValueError('T must be non-negative')
    K, delta = _damping_params(zeta)
    dT = T * delta / (2.0 * math.pi)
    return [i * dT for i in range(count)]


def zv_impulses(T: float, zeta: float = 0.0) -> tuple[list[float], list[float]]:
    """ZV 两脉冲。返回 (amplitudes, times)。"""
    K, _ = _damping_params(zeta)
    times = _impulse_times(T, zeta, 2)
    denom = 1.0 + K
    return [1.0 / denom, K / denom], times


def zvd_impulses(T: float, zeta: float = 0.0) -> tuple[list[float], list[float]]:
    """ZVD 三脉冲。返回 (amplitudes, times)。"""
    K, _ = _damping_params(zeta)
    times = _impulse_times(T, zeta, 3)
    denom = 1.0 + 2.0 * K + K * K
    return [1.0 / denom, 2.0 * K / denom, K * K / denom], times


def ei_impulses(T: float, zeta: float = 0.0, v_tol: float = 0.05) -> tuple[list[float], list[float]]:
    """Extra-Insensitive 整形器 (零阻尼闭式解)。

    在标称频率处允许残余振动 v_tol 的前提下, 最大化对频率失配的鲁棒性。
    ζ=0 时三脉冲闭式解:

        A1 = A3 = (1+V)/4,  A2 = (1-V)/2   @ t = 0, T/2, T

    当 V=0 时退化为 ZVD。含阻尼 (ζ>0) 的 EI 闭式解较复杂, 暂退化为 ZVD。
    """
    if not 0.0 <= v_tol < 1.0:
        raise ValueError('v_tol must be in [0, 1)')
    if zeta != 0.0:
        return zvd_impulses(T, zeta)
    times = _impulse_times(T, 0.0, 3)  # [0, T/2, T]
    a = (1.0 + v_tol) / 4.0
    b = (1.0 - v_tol) / 2.0
    return [a, b, a], times


def shape_command(
    values: list[float],
    dt: float,
    amplitudes: list[float],
    times: list[float],
) -> list[float]:
    """对等间隔采样的命令数组做输入整形 (离散卷积), 返回同长度数组。

    Args:
        values: 等间隔命令采样 (如 S 曲线速度参考)
        dt:     采样周期 [s]
        amplitudes/times: 脉冲幅值与时刻 (来自 zv/zvd/ei_impulses)

    Returns:
        整形后的命令数组 (与 values 等长; 前导时延部分自然为零)
    """
    if dt <= 0:
        raise ValueError('dt must be positive')
    if len(amplitudes) != len(times):
        raise ValueError('amplitudes and times must have equal length')
    n = len(values)
    result = [0.0] * n
    for i in range(n):
        acc = 0.0
        for a, t_i in zip(amplitudes, times):
            k = i - int(round(t_i / dt))
            if 0 <= k < n:
                acc += a * values[k]
        result[i] = acc
    return result
