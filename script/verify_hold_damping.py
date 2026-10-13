#!/usr/bin/env python3
"""原地保持消摆验证 — 抓钩守在一个位置不动, 人为推一次造摆, 看防摇能不能压下去。

为什么单独写一个, 而不是直接用已有的 script/damp_hold_test.py
--------------------------------------------------------------
`run_pd_control()` 在三轴锁定的瞬间就退出并 `stop_motion()`, 而且
`control_complete=True` 会直接关掉防摇叠加通路 (crane_model.py:1176, 1234-1236) ——
摆幅最大的悬停阶段反而没有执行器在作用。调度器的 `damp_in_place()` 补上了悬停阻尼,
是仓库里唯一能在原地阻尼的入口, 但它的 damper 是用
`terminal_mode=cfg.anti_sway_terminal_enabled` 构造的 (operation_scheduler.py:353),
而现场 site_config.json 里该项为 true。

终端通路把全部输出挂在 `SwayFeedbackQualifier` 后面, 而该资格要求"支点被受迫运动
激励"。原地保持时车不动 → 没有受迫激励 → 资格永远不通过 → 修正量恒为 0。
用同一份代码数值核对 (Z=1.0m, 1° 正弦摆, 25s, 锁定双轴):

    terminal_mode=False : max|v_cmd| = 0.07018 m/s , qualifier=None
    terminal_mode=True  : max|v_cmd| = 0.00000 m/s , qualifier=(False, False)

也就是说: **当前配置下, 原地自由摆的消摆输出恒等于零**。所以本程序走机制通路
(terminal_mode=False + gain_scale=anti_sway_settle_gain_scale), 并把终端通路的资格
诊断只打印不参与 —— 机制是能验的, 终端通路在原地能不能介入本身也是要看的结果。

本程序做的事
------------
  · 锚点 = 启动时的当前位置; 全程不动 Z、不动抓钩、不走行 (只驱动大车 X / 小车 Y)
  · A 段 (--observe): 只观测, 每拍显式下发零速, 记录自然衰减作基线
  · B 段 (--damp):   打开 Δv = +K·θ_ac 的锁轴软阻尼, 在位移预算内驱动大小车
  · 全程打印终端通路 (terminal_mode=True) 的资格诊断, 只读不参与控制
  · 结束后给逐轴 A/B 对照, 并落 JSON + PNG

安全
----
  · 运行前必须手握急停。脚本内 Ctrl-C 会立即停车, 但急停是最后防线。
  · 首次测试请空载、抓钩抬到安全高度、周围无人。
  · 符号反了会放大而不是收敛: 数值核对里 2° 推钩在反号下 8~11s 涨到 20.8°。
    本程序在 B 段摆幅超过 max(--abort-pp-deg, --abort-ratio × 进入时摆幅),
    或 guard 撤回任一轴时, 立即停车并判失败 —— 不指望 guard 兜底, 它比发散慢。
  · 建议推钩 1~2° (抓钩约 5~10cm)。超过约 2.5° 就会撞上 6cm 位移预算,
    阻尼被 clamp 成半波, 表现是"消不动还往一边漂"。

用法
----
    python3 script/verify_hold_damping.py --plc-ip 192.168.1.100
    python3 script/verify_hold_damping.py --plc-ip 192.168.1.100 --observe 12 --damp 20
    python3 script/verify_hold_damping.py --plc-ip 192.168.1.100 --dry-run  # 只算不发
    python3 script/verify_hold_damping.py --plc-ip 192.168.1.100 --strategy terminal

--strategy terminal 是给"我不信终端通路在原地真的不出力"准备的: 它把控制运行时
换成作业同款 (terminal_mode=True, gain_scale=1.0), 于是你会亲眼看到 B 段每一拍
"门限开着但 Δv = 0"、车一动不动、残摆等于自然衰减。默认的 mechanism 才是能消摆的。

前置条件 (没做完先别跑, 见 script/check_sway_sign.py):
  1. 倾角仪轴映射与符号已核验 (朝 +X 推抓钩 → θx 为正)
  2. 当前高度的零偏已核验
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import matplotlib  # noqa: E402
matplotlib.use('Agg')

import matplotlib.pyplot as plt  # noqa: E402

from crane_model import ControlStoppedError  # noqa: E402
from plc_interface import PlcActuator, create_plc  # noqa: E402
from ros_bridge import (  # noqa: E402
    RosPositionSource,
    get_inclination_raw,
    get_latest_pose,
    get_sway_snapshot,
    get_sway_state,
    start_ros_bridge,
)
from site_config import (  # noqa: E402
    DEFAULT_SITE_CONFIG_PATH,
    build_sway_axis_mapping,
    load_site_config,
)
from sway_controller import damping_ratio, envelope_decay_time  # noqa: E402
from sway_settle import (  # noqa: E402
    SwayLiveAnalyzer,
    adaptive_settle_window,
    effective_rope_length,
    pendulum_period,
    samples_time_span,
    sway_linear_detrend_peak_to_peak,
    sway_raw_peak_to_peak,
)

# 摆角断流多久就整段中止 [s]。与 operation_scheduler._SWAY_STALE_FALLBACK_S 同语义:
# 宁可不消摆, 也不能拿过期摆角盲目动车。
_SWAY_STALE_ABORT_S = 2.0
# 定位连续断流多少拍就中止。RosPositionSource 单次超时是 2s, 10Hz 下 5 拍 ≈ 10s。
_MAX_POSITION_MISSES = 5
# 周期性摘要/诊断的打印间隔 [s]
_SUMMARY_INTERVAL_S = 2.0
# 启动前等倾角仪/定位的窗口 [s]
_PREFLIGHT_WAIT_S = 8.0
# 冻结检测: 至少这么多样本才开始判"读数是不是一直不变"
_ACTIVITY_MIN_SAMPLES = 32


# ============================================================================
# 纯函数 — 离线可单测, 不依赖 PLC/ROS
# ============================================================================

def frozen_sensor(samples: list[tuple[float, float, float]]) -> bool:
    """倾角仪读数是否疑似冻结 (网关在重复上一帧)。

    冻结时后面所有零偏/摆幅/衰减统计都会变成假结论, 所以必须先判它。
    判据沿用 check_sway_sign.py 的做法: 不同读数的种类数远少于样本数。
    """
    n = len(samples)
    if n < _ACTIVITY_MIN_SAMPLES:
        return False
    distinct = len({(s[1], s[2]) for s in samples})
    return distinct <= max(2, n // 20)


def sample_rate_hz(samples: list[tuple[float, float, float]]) -> float:
    """倾角仪的有效更新率 [Hz] —— 只数"读数真的变了"的样本。

    为什么不能用 get_sway_state() 是否返回 None 来判断: 网关在重复上一帧时它照样
    返回有效值。而 SwayAmplitudeTracker 是对 raw θ 做差分的, 读数一重复 θ̇→0,
    幅度门限就把 Δv 全部归零 —— 现场表现是"防摇毫无反应", 却没有任何报错。
    这个数就是那道静默失败的探针。
    """
    if len(samples) < 2:
        return 0.0
    changes = 1
    for prev, cur in zip(samples, samples[1:]):
        if abs(cur[1] - prev[1]) + abs(cur[2] - prev[2]) > 1e-12:
            changes += 1
    return changes / max(samples_time_span(samples), 1e-3)


def envelope_tau(samples: list[tuple[float, float, float]]) -> float | None:
    """用前后半段去趋势峰峰值之比估计包络时间常数 τ [s]。

    τ = Δt / ln(PP_前 / PP_后)。返回 None 表示样本不足或后半段没有明显变小
    (即没有在衰减) —— 此时不要编一个数出来。

    衰减比门槛取 1.15: 等幅摆动时这个比值也不是 1.0 —— 前后半段装不满整数个
    周期, 采样点落在正弦的不同位置上, 实测有约 8% 的固有偏差。门槛低于它就会
    把等幅摆动算成"τ = 两万秒", 比报 n/a 更容易误导。取 1.15 后仍能报出
    τ 到约 60s 的衰减, 覆盖现场自然阻尼的量级。
    """
    n = len(samples)
    if n < 64:
        return None
    mid = n // 2
    first = sway_linear_detrend_peak_to_peak(samples[:mid])
    second = sway_linear_detrend_peak_to_peak(samples[mid:])
    if not first or not second:
        return None
    a0, a1 = max(first), max(second)
    if a1 <= 0.0 or a0 <= a1 * 1.15:
        return None
    dt = (samples[-1][0] + samples[mid][0]) / 2 - (
        samples[mid][0] + samples[0][0]
    ) / 2
    if dt <= 0.0:
        return None
    return dt / math.log(a0 / a1)


def window_pp(samples: list[tuple[float, float, float]], window_s: float) -> dict:
    """末尾 window_s 窗内的去趋势/原始峰峰值 (度)。样本不足时各字段为 None。"""
    empty = {
        'window_s': window_s,
        'window_full': False,
        'detrend_pp_x_deg': None, 'detrend_pp_y_deg': None, 'detrend_pp_max_deg': None,
        'raw_pp_x_deg': None, 'raw_pp_y_deg': None, 'raw_pp_max_deg': None,
        'sample_count': 0,
    }
    if len(samples) < 3:
        return empty
    now = samples[-1][0]
    win = [s for s in samples if now - s[0] <= window_s]
    if len(win) < 3:
        return empty
    det = sway_linear_detrend_peak_to_peak(win)
    raw = sway_raw_peak_to_peak(win)
    if det is None or raw is None:
        return empty
    return {
        'window_s': round(window_s, 3),
        'window_full': samples_time_span(win) >= window_s * 0.95,
        'detrend_pp_x_deg': round(math.degrees(det[0]), 4),
        'detrend_pp_y_deg': round(math.degrees(det[1]), 4),
        'detrend_pp_max_deg': round(math.degrees(max(det)), 4),
        'raw_pp_x_deg': round(math.degrees(raw[0]), 4),
        'raw_pp_y_deg': round(math.degrees(raw[1]), 4),
        'raw_pp_max_deg': round(math.degrees(max(raw)), 4),
        'sample_count': len(win),
    }


def peak_pp_deg(samples: list[tuple[float, float, float]], axis: int) -> float:
    """整段样本里该轴的最大 |θ| [度] —— 用来回答"这一段到底摆过多大"。"""
    if not samples:
        return 0.0
    return max(abs(math.degrees(s[axis])) for s in samples)


def settle_time(samples: list[tuple[float, float, float]], plan: dict) -> float | None:
    """首次进入判稳容差、且此后不再超差的时刻 [s, 相对 samples[0]]。

    判据用 SwayLiveAnalyzer.metrics()['settled'] (去趋势 PP < 容差且窗已填满),
    与调度器判稳完全同一口径。任何一次评估不 settled 就清零重来, 所以返回值是
    "最后一次超差之后第一次达标"的时刻。未达标记为 None, 绝不写 0。
    """
    analyzer = SwayLiveAnalyzer(
        base_window=plan['window_base_s'],
        window_periods=plan['window_periods'],
        window_max=plan['window_max_s'],
        sheave_height=plan['sheave_height'],
        grab_offset=plan['grab_offset'],
        tol_deg=plan['tol_deg'],
    )
    t0 = samples[0][0] if samples else 0.0
    result = None
    for t, theta_x, theta_y in samples:
        analyzer.push(t, theta_x, theta_y)
        if analyzer.metrics(plan['z_grab'])['settled']:
            if result is None:
                result = t - t0
        else:
            result = None
    return result


def axis_verdict(baseline_pp: float, damped_pp: float, tol_deg: float) -> str:
    """逐轴三态判定。刻意不接受"合成摆角": 一轴的改善不得抵消另一轴的恶化。

    死区取半个容差: 比验收容差一半还小的变化不该被叫作改善或恶化, 那只是这次
    推钩和上一次之间的噪声。死区取 1/10 容差会让 0.05° 的抖动也变成"恶化"。
    """
    deadband = tol_deg / 2.0
    if damped_pp < baseline_pp - deadband:
        return '改善'
    if damped_pp > baseline_pp + deadband:
        return '恶化'
    return '未改善'


def resolve_plan(config, z_grab: float, args) -> dict:
    """把配置 + 抓钩高度 + CLI 解析成一份"这次要干什么"的计划 (启动块的数据源)。"""
    sheave = config.rope_length_sheave_height
    grab_offset = config.rope_length_grab_offset
    L = effective_rope_length(
        z_grab, sheave_height=sheave, grab_offset=grab_offset,
        cable_stretch=config.rope_length_cable_stretch,
        min_length=config.rope_length_min,
    )
    T = pendulum_period(
        z_grab, sheave_height=sheave, grab_offset=grab_offset,
        cable_stretch=config.rope_length_cable_stretch,
        min_length=config.rope_length_min,
    )
    budget = (
        config.anti_sway_locked_max_shift if args.max_shift is None
        else float(args.max_shift)
    )
    amp_floor_deg = (
        config.hook_sway_angle_tol_deg / 4.0 if args.amp_floor_deg is None
        else float(args.amp_floor_deg)
    )
    strategy = str(args.strategy)
    # 作业里 terminal_policy 恒用 gain_scale=1.0 (crane_model.py:1189-1190), 只有
    # 非终端的锁轴悬停阻尼才乘 anti_sway_settle_gain_scale。这里必须一致, 否则
    # 测的就不是作业那条通路。
    gain_scale = (1.0 if strategy == 'terminal'
                  else config.anti_sway_settle_gain_scale)
    return {
        'strategy': strategy,
        'z_grab': z_grab,
        'rope_length_m': L,
        'period_s': T,
        'sheave_height': sheave,
        'grab_offset': grab_offset,
        'gain': config.anti_sway_sway_gain,
        'gain_scale': gain_scale,
        'effective_gain': config.anti_sway_sway_gain * gain_scale,
        'zeta': damping_ratio(config.anti_sway_sway_gain * gain_scale, L),
        'decay_1deg_s': envelope_decay_time(
            config.anti_sway_sway_gain * gain_scale, L,
            1.0, config.hook_sway_angle_tol_deg,
        ),
        'max_correction': config.anti_sway_max_correction,
        'budget_m': budget,
        'recenter_gain': (
            config.anti_sway_hold_recenter_gain if args.recenter_gain is None
            else float(args.recenter_gain)
        ),
        'min_incl_hz': float(args.min_incl_hz),
        'dc_tau_s': config.anti_sway_settle_dc_tau,
        'amp_floor_deg': amp_floor_deg,
        'tol_deg': config.hook_sway_angle_tol_deg,
        'window_base_s': getattr(config, 'hook_settle_window', 0.6),
        'window_periods': config.hook_settle_window_periods,
        'window_max_s': config.hook_settle_window_max,
        'observe_s': 0.0 if args.no_ab else float(args.observe),
        'damp_s': float(args.damp),
        'abort_pp_deg': float(args.abort_pp_deg),
        'abort_ratio': float(args.abort_ratio),
        'dry_run': bool(args.dry_run),
    }


def _jsonable(value):
    """把 numpy 标量/非有限浮点转成可 JSON 化的值 (非有限一律变 null)。"""
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    return number if math.isfinite(number) else None


# ============================================================================
# CLI / 配置
# ============================================================================

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description='原地保持消摆验证 — 只驱动 XY 的锁轴软阻尼, 不动 Z/抓钩/走行',
    )
    p.add_argument('--plc-ip', required=True, help='PLC IP')
    p.add_argument('--plc-lib', default=None, help='PLC 动态库路径')
    p.add_argument('--allow-mock-plc', action='store_true',
                   help='PLC 不可用时退回 MockPLC (仅用于自检脚本本身)')
    p.add_argument('--config', default=str(DEFAULT_SITE_CONFIG_PATH),
                   help='site config JSON')
    p.add_argument('--observe', type=float, default=12.0,
                   help='A 段只观测时长 [s] (默认 12, 至少 2 个摆周期)')
    p.add_argument('--damp', type=float, default=20.0, help='B 段消摆时长 [s]')
    p.add_argument('--anti-sway-gain', type=float, default=None,
                   help='覆盖 K [m/s per rad], 不改 site_config')
    p.add_argument('--max-shift', type=float, default=None,
                   help='覆盖位移预算 [m] (默认 anti_sway_locked_max_shift)')
    p.add_argument('--amp-floor-deg', type=float, default=None,
                   help='覆盖幅度门限 [deg] (默认 hook_sway_angle_tol_deg/4)')
    p.add_argument('--recenter-gain', type=float, default=None,
                   help='覆盖锁轴锚点回中增益 [1/s] (默认 anti_sway_hold_recenter_gain); '
                        '量 τ 时用 0 可去掉这个并行的位置控制混淆')
    p.add_argument('--min-incl-hz', type=float, default=8.0,
                   help='倾角仪有效更新率下限 [Hz]; 低于它就拒绝进入 B 段。'
                        '读数在重复时 θ̇→0 会让幅度门限把 Δv 全部归零, '
                        '表现为"防摇毫无反应"却没有任何报错')
    p.add_argument('--z-grab', type=float, default=None,
                   help='抓钩高度 [m]; 仅在 PLC 无高度回读时用, 且必须配合 --dry-run')
    p.add_argument('--abort-pp-deg', type=float, default=3.0,
                   help='B 段发散判据的绝对下限 [deg]')
    p.add_argument('--abort-ratio', type=float, default=1.5,
                   help='B 段发散判据: 相对进入时摆幅的倍数')
    p.add_argument('--strategy', choices=('mechanism', 'terminal'), default='mechanism',
                   help='消摆通路: mechanism=机制通路(默认, 原地唯一能真正消摆的); '
                        'terminal=作业同款终端通路 —— 原地预期不出力, 用它现场核实这一点')
    p.add_argument('--no-ab', action='store_true',
                   help='跳过 A 段观测, 启动后直接消摆')
    p.add_argument('--dry-run', action='store_true',
                   help='只计算并打印, 不向 PLC 下发任何速度')
    p.add_argument('--invert-big-car', action='store_true')
    p.add_argument('--invert-small-car', action='store_true')
    p.add_argument('--invert-lift', action='store_true',
                   help='本脚本不动 Z, 该项无效; 保留只为与 damp_hold_test.py 命令行一致')
    p.add_argument('--min-lift-height', type=float, default=0.35)
    p.add_argument('--output', default='hold_damping_result',
                   help='JSON/PNG 输出前缀')
    p.add_argument('--yes', action='store_true', help='跳过运行前确认 (慎用)')
    return p


def _make_config(args):
    """复用 main._config_from_args, 保证与作业时的配置解析完全一致。"""
    import main

    site = load_site_config(args.config)
    if args.anti_sway_gain is not None:
        site = dict(site, anti_sway_sway_gain=args.anti_sway_gain)
    if not site.get('enable_anti_sway'):
        print('[错误] site_config 里 enable_anti_sway 不为 true — 防摇通路不会启用')
        return None, None
    base = main._build_arg_parser().parse_args([])
    base.min_lift_height = args.min_lift_height
    base.invert_big_car = args.invert_big_car
    base.invert_small_car = args.invert_small_car
    base.invert_lift = args.invert_lift
    return main._config_from_args(base, site, argv=[]), site


# ============================================================================
# 启动块
# ============================================================================

def _print_plan(plan: dict, site: dict, args) -> None:
    tol = plan['tol_deg']
    print()
    print('=' * 78)
    print('原地保持消摆验证')
    print('=' * 78)
    print(f'  抓钩高度 Z      = {plan["z_grab"]:.2f} m  →  有效摆长 L = {plan["rope_length_m"]:.2f} m')
    print(f'  模型摆周期 T    = {plan["period_s"]:.2f} s')
    print(f'  防摇增益 K      = {plan["gain"]:.2f} m/s/rad  '
          f'(×{plan["gain_scale"]:.2f} = {plan["effective_gain"]:.2f})')
    print(f'  等效阻尼比 ζ    = {plan["zeta"]:.2f}   '
          f'(理论 1°→{tol:.1f}° 约 {plan["decay_1deg_s"]:.1f} s)')
    print(f'  速度上限        = {plan["max_correction"]:.2f} m/s')
    print(f'  位移预算        = ±{plan["budget_m"] * 100:.0f} cm (相对锚点)')
    print(f'  锚点回中增益    = {plan["recenter_gain"]:.2f} 1/s')
    print(f'  θ_ac 高通 τ     = {plan["dc_tau_s"]:.1f} s')
    print(f'  幅度门限        = {plan["amp_floor_deg"]:.3f}° (低于此不阻尼)')
    print(f'  判稳容差        = {tol:.2f}° 去趋势峰峰值')
    print(f'  阶段            = A 观测 {plan["observe_s"]:.0f}s → B 消摆 {plan["damp_s"]:.0f}s'
          + ('   (--no-ab, 无 A 段)' if plan['observe_s'] <= 0 else ''))
    print(f'  发散判据        = PP > max({plan["abort_pp_deg"]:.1f}°, '
          f'{plan["abort_ratio"]:.1f}× 进入 B 段时摆幅)')
    print(f'  运行模式        = {"DRY-RUN (只算不发)" if plan["dry_run"] else "真实下发 (会动车)"}')
    print(f'  消摆通路        = {plan["strategy"]}'
          + ('   (机制通路, 原地能真正消摆)' if plan['strategy'] == 'mechanism'
             else '   (作业同款终端通路, 原地预期不出力)'))
    mapping = build_sway_axis_mapping(site)
    print(f'  轴映射          = θx ← {mapping.theta_x.angle_source}'
          f'(sign={mapping.theta_x.angle_sign:+.0f}), '
          f'θy ← {mapping.theta_y.angle_source}'
          f'(sign={mapping.theta_y.angle_sign:+.0f})')
    print('-' * 78)
    if plan['strategy'] == 'mechanism':
        print('  消摆通路 =【机制通路】Δv = +K·θ_ac + 位移预算 + 增长撤回。')
        print('  作业部署的【终端通路】在原地拿不到资格 (它要求支点受迫运动, 而原地')
        print('  保持时车不动), 程序会把它的资格诊断打印出来但不参与控制。')
    else:
        print('  消摆通路 =【作业同款终端通路】(--strategy terminal)。')
        print('  !! 预期结果就是"几乎不出力": 该通路的输出挂在 SwayFeedbackQualifier')
        print('     后面, 而原地自由摆没有受迫激励, 资格拿不到 → Δv 恒为 0。')
        print('     这一轮是用来看清这件事, 不是用来消摆的; 要消摆请用默认的 mechanism。')
    print('-' * 78)
    print('  请用手把空抓钩朝已知方向推开一次, 幅度建议 1~2°:')
    print('    朝 +X (大车正方向) 推 → θx 应为正;   朝 +Y (小车正方向) 推 → θy 应为正')
    print(f'  推得更大也可以, 但超过约 {plan["budget_m"] * 100 / (plan["effective_gain"] / math.sqrt(9.81 / plan["rope_length_m"])) * 57.3:.1f}° '
          f'就会撞满 {plan["budget_m"] * 100:.0f}cm 位移预算。')
    print()
    print('  !! 握住急停。若摆幅在 B 段放大而非收敛 → 立即急停 (倾角符号仍反)。')
    print('  !! 首次测试请空载、抓钩在安全高度、周围无人。')
    print('=' * 78)
    print()


# ============================================================================
# 主流程
# ============================================================================

def _fmt(value, spec: str = '.3f', dash: str = '  n/a') -> str:
    return dash if value is None else format(value, spec)


def _print_tick(phase: str, t: float, theta, theta_ac, amp_deg, dv,
                drift, note: str) -> None:
    """逐拍状态行 — 现场调试时这一行要能回答"为什么没动/动多少/往哪动"。"""
    th = (f'{_fmt(theta[0])},{_fmt(theta[1])}' if theta else 'n/a,n/a')
    ac = (f'{_fmt(theta_ac[0])},{_fmt(theta_ac[1])}' if theta_ac else 'n/a,n/a')
    line = (
        f'\r[{phase}] t={t:6.1f}s | θ=({th})° | θac=({ac})° | '
        f'Â={_fmt(amp_deg, ".2f")}° | Δv=({dv[0] * 1000:+6.1f},{dv[1] * 1000:+6.1f})mm/s | '
        f'drift=({drift[0] * 1000:+5.0f},{drift[1] * 1000:+5.0f})mm | {note}'
    )
    print(line + ' ' * 8, end='')
    sys.stdout.flush()


def _print_summary(tag: str, t: float, metrics: dict, plan: dict,
                   raw_pp: tuple[float, float] | None, ac_pp: tuple[float, float] | None,
                   period_meas: float | None, diag: dict | None,
                   incl_hz: float) -> None:
    ppx = _fmt(metrics['detrend_pp_x_deg'], '.3f')
    ppy = _fmt(metrics['detrend_pp_y_deg'], '.3f')
    raw_txt = (
        f'{math.degrees(raw_pp[0]):.2f}/{math.degrees(raw_pp[1]):.2f}'
        if raw_pp else 'n/a'
    )
    ac_txt = (
        f'{math.degrees(ac_pp[0]):.2f}/{math.degrees(ac_pp[1]):.2f}'
        if ac_pp else 'n/a'
    )
    # θ_ac 相对 θ 的幅度比: DC 高通 τ 对 3.5s 摆周期约有 2% 衰减 + 12° 相位领先。
    # 这里打出来是为了让现场别把这个正常现象误读成传感器问题。
    if raw_pp and ac_pp and max(raw_pp) > 1e-9:
        ratio = f'{max(ac_pp) / max(raw_pp):.3f}'
    else:
        ratio = 'n/a'
    print()
    print(f'  [{tag}] t={t:6.1f}s  去趋势PP θx={ppx}° θy={ppy}° '
          f'(窗 {metrics["window_s"]:.2f}s, {"满" if metrics["window_full"] else "未满"}, '
          f'{metrics["sample_count"]} 样本)')
    print(f'         原始PP {raw_txt}°   θ_ac PP {ac_txt}° (幅度比 {ratio})   '
          f'Âmax={_fmt(metrics["amp_max_deg"], ".3f")}°   '
          f'倾角仪有效 {incl_hz:.1f}Hz')
    # 实测周期只在摆幅明显高于噪声时才可信, 否则零穿越计数就是在数噪声。
    amp = metrics.get('amp_max_deg')
    model_txt = f'{_fmt(metrics["period_model_s"], ".2f")}s'
    if period_meas is None:
        print(f'         周期 实测 n/a (零穿越不足: 摆幅小或样本不够) / 模型 {model_txt}')
    elif amp is not None and amp <= 3 * plan['tol_deg']:
        print(f'         周期 实测 {period_meas:.2f}s (摆幅仅 {amp:.2f}°, 仅供参考) / '
              f'模型 {model_txt}')
    else:
        print(f'         周期 实测 {period_meas:.2f}s / 模型 {model_txt}')
    if diag is not None:
        print(f'         [终端通路诊断·只读] 资格 x={diag["x_allowed"]} y={diag["y_allowed"]} '
              f'blend=({diag["blend"][0]:.2f},{diag["blend"][1]:.2f})')
        print(f'           x: {diag["x_reason"]}  激励 {_fmt(diag["x_excitation_deg"], ".4f")}°'
              f'  gain {_fmt(diag["x_gain"], ".3f")}  phase {_fmt(diag["x_phase_deg"], ".1f")}°'
              f'  R² {_fmt(diag["x_r2"], ".3f")}')
        print(f'           y: {diag["y_reason"]}  激励 {_fmt(diag["y_excitation_deg"], ".4f")}°'
              f'  gain {_fmt(diag["y_gain"], ".3f")}  phase {_fmt(diag["y_phase_deg"], ".1f")}°'
              f'  R² {_fmt(diag["y_r2"], ".3f")}')


def _diag_snapshot(diag_runtime) -> dict | None:
    """读终端通路诊断运行时的资格状态 (只读, 永不参与控制)。"""
    if diag_runtime is None or diag_runtime.qualifier is None:
        return None
    snap = diag_runtime.qualifier.snapshot()
    out = {
        'x_allowed': snap.get('antisway_qualified_x'),
        'y_allowed': snap.get('antisway_qualified_y'),
        'blend': list(diag_runtime.blend),
    }
    for axis in ('x', 'y'):
        out[f'{axis}_reason'] = snap.get(f'antisway_model_{axis}_reason')
        out[f'{axis}_excitation_deg'] = snap.get(f'antisway_model_{axis}_excitation_deg')
        out[f'{axis}_gain'] = snap.get(f'antisway_model_{axis}_gain')
        out[f'{axis}_phase_deg'] = snap.get(f'antisway_model_{axis}_phase_deg')
        out[f'{axis}_r2'] = snap.get(f'antisway_model_{axis}_forced_r2')
        out[f'{axis}_validation_deg'] = snap.get(f'antisway_model_{axis}_validation_error_deg')
    return out


def _run(args, config, plc, source, actuator, plan: dict) -> tuple[list[dict], dict]:
    """A/B 阶段机。返回 (history, outcome)。"""
    from anti_sway_runtime import AntiSwayRuntime
    from sway_controller import build_anti_sway
    from sway_guard import SwayGrowthGuard

    controller = build_anti_sway(config)
    if controller is None:
        raise RuntimeError('build_anti_sway 返回 None — 防摇未启用或增益为 0')

    # 控制用运行时。默认走机制通路 (terminal_mode=False) —— 这是原地唯一能真正
    # 消摆的通路。--strategy terminal 时改成作业同款终端通路: 它把输出挂在资格
    # 检查后面, 而原地自由摆拿不到资格, 所以预期不出力。保留这个开关是为了让现场
    # 能自己核实"终端通路在原地不介入", 而不是只能听说明。
    guard = SwayGrowthGuard(min_pp_rad=math.radians(2 * plan['tol_deg']))
    runtime = AntiSwayRuntime(
        anti_sway=controller,
        dc_tau=plan['dc_tau_s'],
        locked_max_shift=plan['budget_m'],
        gain_scale=plan['gain_scale'],
        recenter_gain=plan['recenter_gain'],
        amp_floor=math.radians(plan['amp_floor_deg']),
        guard=guard,
        terminal_mode=(plan['strategy'] == 'terminal'),
    )
    # 诊断用运行时: 作业同款终端通路, 只读。输出被丢弃, 只用它的资格诊断。
    diag_runtime = AntiSwayRuntime(
        anti_sway=build_anti_sway(config),
        dc_tau=plan['dc_tau_s'],
        locked_max_shift=plan['budget_m'],
        gain_scale=plan['gain_scale'],
        recenter_gain=plan['recenter_gain'],
        terminal_mode=True,
    )
    analyzer = SwayLiveAnalyzer(
        base_window=plan['window_base_s'],
        window_periods=plan['window_periods'],
        window_max=plan['window_max_s'],
        sheave_height=plan['sheave_height'],
        grab_offset=plan['grab_offset'],
        tol_deg=plan['tol_deg'],
        dc_tau=plan['dc_tau_s'],
    )

    source.reset()
    phase_samples: dict[str, list[tuple[float, float, float]]] = {'a': [], 'b': []}
    ac_samples: list[tuple[float, float, float]] = []
    history: list[dict] = []
    outcome: dict = {
        'aborted': None, 'guard_events': [],
        'bias_primed': None, 'phase_b_entry_pp_deg': None,
        'phase_b_entry_peak_deg': None, 'phase_b_watermark_s': 0.0,
        'dry_run': plan['dry_run'], 'stale_s': 0.0, 'dv_saturated': [0, 0],
        'budget_clamped': [0, 0], 'qualifier_ever_allowed': [False, False],
        'gate_on_zero_dv': 0, 'phase_b_ticks': 0, 'amp_gated': [0, 0],
        'incl_hz': None,
    }

    anchor: tuple[float, float] | None = None
    prime_armed = False
    # 同一窗口内的 (t, θx, θy, θ_ac_x, θ_ac_y)。原始与 θ_ac 的峰峰值必须来自
    # 同一段时间, 否则"幅度比"就是在拿两个不同窗比, 没有意义。
    recent_win: list[tuple[float, float, float, float, float]] = []
    last_summary = -1e9
    last_z: float | None = None
    stale_since: float | None = None
    position_misses = 0
    t_start = None
    t_phase_b: float | None = None
    phase = 'A' if plan['observe_s'] > 0 else 'B'
    guard_event_count = 0
    z_grab = plan['z_grab']

    while True:
        pos = source.get_position()
        now = time.monotonic()
        if pos is None:
            position_misses += 1
            if position_misses >= _MAX_POSITION_MISSES:
                outcome['aborted'] = '定位反馈超时'
                break
            print(f'\n[定位] 断流 ({position_misses}/{_MAX_POSITION_MISSES}), 继续等待...')
            continue
        position_misses = 0
        if t_start is None:
            t_start = now
        t = now - t_start
        dt = pos['dt'] if pos['dt'] and pos['dt'] > 0 else 0.1
        x, y = pos['x'], pos['y']

        # 抓钩高度用 PLC 实测; 读数无效就沿用上一次 (绳长模型不能凭空猜)
        lift = plc.get_lift_height()
        if lift is not None and math.isfinite(lift):
            last_z = float(lift)
        if last_z is None:
            outcome['aborted'] = '抓钩高度读数无效'
            break
        z_grab = last_z
        plan['z_grab'] = z_grab

        if anchor is None:
            # 锚点 = 启动时当前位置。锁定轴的位移预算与回中都以它为准。
            anchor = (x, y)
            runtime.reset(x, y)
            diag_runtime.reset(x, y)
            analyzer.reset()
            if not plan['dry_run']:
                actuator.set_z_reference(z_grab)
            else:
                # dry-run 也要显式发一次零速: PLC 的 X/Y 是速度伺服, 上一条指令
                # 会一直保持有效, 不发等于让历史指令继续执行。
                actuator.stop_motion()
            print(f'锚点 = ({x:.3f}, {y:.3f}) m, Z = {z_grab:.2f} m; '
                  f'预计 T = {pendulum_period(z_grab, sheave_height=plan["sheave_height"], grab_offset=plan["grab_offset"]):.2f} s')
            if plan['observe_s'] > 0:
                print(f'>>> A 段 {plan["observe_s"]:.0f}s: 只观测不动车。'
                      f'现在推抓钩造摆 <<<')
            else:
                print('>>> 直接进入 B 段消摆。现在推抓钩造摆 <<<')

        # 阶段切换: A 段到时 → B 段
        if phase == 'A' and t >= plan['observe_s']:
            # 进 B 段之前先确认倾角仪在真的更新。读数在重复时 θ̇→0, 幅度门限会把
            # Δv 全部归零 —— "防摇毫无反应"却没有任何报错, 这正是要先拦住的那种失败。
            rate = sample_rate_hz(phase_samples['a'])
            outcome['incl_hz'] = round(rate, 2)
            if rate < plan['min_incl_hz']:
                outcome['aborted'] = (
                    f'倾角仪有效更新率 {rate:.1f}Hz < {plan["min_incl_hz"]:.1f}Hz — '
                    f'读数疑似被重复/冻结; 此时 θ̇≈0 会让幅度门限把 Δv 全部归零, '
                    f'消摆不会发生。先解决数据活性, 不要靠这项测试试错'
                )
                print()
                break
            phase = 'B'
            outcome['phase_b_entry_pp_deg'] = window_pp(
                phase_samples['a'], plan['window_base_s'])['detrend_pp_max_deg']
            tail_a = [s for s in phase_samples['a'] if s[0] >= t - plan['period_s']]
            outcome['phase_b_entry_peak_deg'] = round(
                max(peak_pp_deg(tail_a, 1), peak_pp_deg(tail_a, 2)), 4)
            print()
            print(f'>>> B 段: 打开消摆 (K×{plan["gain_scale"]:.2f}, '
                  f'预算 ±{plan["budget_m"] * 100:.0f}cm)。请放手, 不要再碰抓钩 <<<')
            analyzer.reset()
            prime_armed = False
        if phase == 'B' and t_phase_b is None:
            t_phase_b = t

        sway = get_sway_state()
        theta = None
        theta_ac = (0.0, 0.0)
        amp_deg = None
        dv = (0.0, 0.0)
        diag = None

        if sway is None:
            runtime.suspend(dt)
            diag_runtime.suspend(dt)
            # 摆角断流: 立刻停车。宁可不消摆, 也不能拿过期摆角盲目动车。
            if not plan['dry_run']:
                actuator.stop_motion()
            if stale_since is None:
                stale_since = now
            outcome['stale_s'] = now - stale_since
            if outcome['stale_s'] > _SWAY_STALE_ABORT_S:
                outcome['aborted'] = f'摆角持续断流 {outcome["stale_s"]:.1f}s'
                print()
                break
            snap = get_sway_snapshot()
            _print_tick(phase, t, None, None, None, dv,
                        (x - anchor[0], y - anchor[1]),
                        f'[摆角断流 {outcome["stale_s"]:.1f}s '
                        f'age={snap.get("age_ms", "?")}ms — 已停车]')
            continue
        stale_since = None
        outcome['stale_s'] = 0.0

        theta = (float(sway['theta_x']), float(sway['theta_y']))
        analyzer.push(t, theta[0], theta[1])
        phase_samples['a' if phase == 'A' else 'b'].append((t, theta[0], theta[1]))

        # 静态零偏: 用摆动信号本身解出来置入高通, 免得残余零偏透过高通积分成漂移。
        if not prime_armed:
            bias = runtime.prime_dc(analyzer.samples(), z_grab)
            if bias is not None:
                prime_armed = True
                outcome['bias_primed'] = [math.degrees(bias[0]), math.degrees(bias[1])]
                print(f'\n  由摆动信号解出静态零偏 '
                      f'({math.degrees(bias[0]):+.3f}°, {math.degrees(bias[1]):+.3f}°), 已置入高通')

        feedback_allowed = (phase == 'B')
        vx, vy, dvx, dvy = runtime.merge_xy_commands(
            0.0, 0.0,
            theta_x=theta[0], theta_y=theta[1],
            z_grab=z_grab, x=x, y=y,
            locked_x=True, locked_y=True, dt=dt,
            max_velocity_xy=plan['max_correction'],
            gain_scale=plan['gain_scale'],
            feedback_allowed=feedback_allowed,
        )
        # 诊断运行时喂同样的实测输入。feedback_allowed=True 是刻意的: 这里问的是
        # "如果允许它介入, 它会不会拿到资格", 答案本身就是现场要看的结论。
        diag_runtime.merge_xy_commands(
            0.0, 0.0, theta_x=theta[0], theta_y=theta[1], z_grab=z_grab, x=x, y=y,
            locked_x=True, locked_y=True, dt=dt,
            max_velocity_xy=plan['max_correction'],
            gain_scale=plan['gain_scale'], feedback_allowed=True,
        )
        diag = _diag_snapshot(diag_runtime)
        if diag:
            for idx, axis in enumerate('xy'):
                if diag[f'{axis}_allowed']:
                    outcome['qualifier_ever_allowed'][idx] = True

        dv = (dvx, dvy)
        # 实际下发的是 merge_xy_commands 的返回值 (vx, vy); A 段它就是 0 (已数值核对)。
        # 门限开着 ≠ 真的发出了修正: 幅度门限/预算/guard 都可能让 Δv 归零。
        # 这两个状态要分开记, 否则现场会把"gate 开了但没消摆"误读成算法无效。
        applied = feedback_allowed and (abs(vx) + abs(vy) > 1e-12)
        if feedback_allowed:
            outcome['phase_b_ticks'] += 1
            if not applied:
                outcome['gate_on_zero_dv'] += 1
            for idx in (0, 1):
                rate_amp = (runtime.amp.last_rate_amp_x, runtime.amp.last_rate_amp_y)[idx]
                if rate_amp < math.radians(plan['amp_floor_deg']):
                    outcome['amp_gated'][idx] += 1
        ac = runtime.amp.last_ac_x, runtime.amp.last_ac_y
        theta_ac = ac
        ac_samples.append((t, ac[0], ac[1]))
        amp_deg = math.degrees(runtime.last_amp)
        recent_win.append((t, theta[0], theta[1], ac[0], ac[1]))
        while recent_win and recent_win[0][0] < t - 5.0:
            recent_win.pop(0)

        for idx, val in enumerate(dv):
            if abs(val) >= plan['max_correction'] - 1e-9:
                outcome['dv_saturated'][idx] += 1
            if abs(x - anchor[0] if idx == 0 else y - anchor[1]) >= plan['budget_m'] - 1e-9:
                outcome['budget_clamped'][idx] += 1

        # 增长撤回事件: 算法自己发现不对劲的证据, 和操作员看到的发散是两件事。
        for event in guard.events[guard_event_count:]:
            outcome['guard_events'].append(event)
            print()
            print(f'  [增长保护] {event["axis"].upper()} 轴振荡增长 '
                  f'{event["baseline_deg"]:.3f}° → {event["pp_deg"]:.3f}° PP, 已撤回该轴修正')
        guard_event_count = len(guard.events)

        if plan['dry_run']:
            pass
        elif phase == 'B':
            actuator.apply(vx, vy, 0.0, dt)
        else:
            # A 段必须显式下发零速: PLC 的 X/Y 是速度伺服、每周期刷新指令,
            # "什么都不发"等于让上一条指令继续有效。
            actuator.apply(0.0, 0.0, 0.0, dt)

        # ---- B 段发散判据 (相对阈值, 只在 B 段生效) ----
        # 必须是相对的: A 段结束后 guard 的 baseline 就等于进入时的摆幅, 而它的
        # 判据是 pp > baseline×1.25 且 >0.6° 且增加 >0.15°, 连续两次才撤回 ——
        # 推得越大它越迟钝 (推 5° 要等到 6.25° 才动手), 而反号摆 8~11s 就能到 20°。
        # 固定绝对阈值也不行: 取 4° 时推 5° 会在 A 段就误触发。所以用
        # max(绝对下限, 倍数 × 进入 B 段时的摆幅水位), 且只在 B 段生效。
        if phase == 'B':
            recent = [s for s in phase_samples['b'] if s[0] >= t - plan['period_s']]
            cur_peak = max(peak_pp_deg(recent, 1), peak_pp_deg(recent, 2)) if recent else 0.0
            # B 段最初 1.5s 之内允许水位继续抬升 (推钩余波 / 恰好在切换点附近推的
            # 情况), 之后冻结。但抬升必须封顶: 反号发散时摆幅在这 1.5s 里就涨了,
            # 不封顶等于让判据跟着发散一起抬高, 等于没有判据。
            base_entry = outcome['phase_b_entry_peak_deg'] or 0.0
            if t - (t_phase_b or t) <= 1.5:
                ceiling = base_entry * 1.2 if base_entry > 0 else cur_peak
                entry = max(base_entry, min(cur_peak, ceiling))
                outcome['phase_b_entry_peak_deg'] = round(entry, 4)
                outcome['phase_b_watermark_s'] = round(t - (t_phase_b or t), 2)
            else:
                entry = base_entry
            limit = max(plan['abort_pp_deg'], plan['abort_ratio'] * entry)
            if cur_peak > limit:
                outcome['aborted'] = (
                    f'B 段摆幅 {cur_peak:.2f}° 超过发散判据 {limit:.2f}° '
                    f'(进入 B 段时 {entry:.2f}°)'
                )
                print()
                break
            if any(guard.blocked):
                outcome['aborted'] = (
                    '增长保护撤回 '
                    + '/'.join(a.upper() for a, b in zip('xy', guard.blocked) if b)
                    + ' 轴 — 反馈可能在放大摆动'
                )
                break

        note = 'damping' if applied else ('gate-on' if feedback_allowed else 'observing')
        if any(guard.blocked):
            note += f' blocked={"".join(a for a, b in zip("xy", guard.blocked) if b) or "-"}'
        _print_tick(phase, t, theta, theta_ac, amp_deg, dv,
                    (x - anchor[0], y - anchor[1]), note)

        if t - last_summary >= _SUMMARY_INTERVAL_S and len(recent_win) >= 8:
            last_summary = t
            metrics = analyzer.metrics(z_grab)
            # 每个周期都刷新: --no-ab 时没有 A→B 切换那一次赋值, 报告里不能是 None。
            outcome['incl_hz'] = round(
                sample_rate_hz([(w[0], w[1], w[2]) for w in recent_win]), 2)
            # 两条曲线取自同一个 recent_win, 幅度比才有意义。
            _print_summary('A 观测' if phase == 'A' else 'B 消摆', t, metrics, plan,
                           sway_raw_peak_to_peak([(w[0], w[1], w[2]) for w in recent_win]),
                           sway_raw_peak_to_peak([(w[0], w[3], w[4]) for w in recent_win]),
                           metrics['period_s'], diag,
                           sample_rate_hz([(w[0], w[1], w[2]) for w in recent_win]))

        if phase == 'B' and t >= plan['observe_s'] + plan['damp_s']:
            print()
            print('  B 段到时, 结束。')
            break

        entry = {
            't': t, 'phase': phase, 'theta_x': theta[0], 'theta_y': theta[1],
            'theta_ac_x': ac[0], 'theta_ac_y': ac[1], 'amp_deg': amp_deg,
            'dvx': dvx, 'dvy': dvy, 'vx_cmd': vx, 'vy_cmd': vy,
            'x': x, 'y': y, 'z': z_grab,
            'x_anchor': anchor[0], 'y_anchor': anchor[1],
            'antisway_gate': feedback_allowed, 'antisway_applied': applied,
            'dry_run': plan['dry_run'],
            'guard_pp_x_deg': None if guard.last_pp[0] is None else math.degrees(guard.last_pp[0]),
            'guard_pp_y_deg': None if guard.last_pp[1] is None else math.degrees(guard.last_pp[1]),
            'guard_blocked': list(guard.blocked),
            'qual_x_allowed': None if diag is None else diag['x_allowed'],
            'qual_y_allowed': None if diag is None else diag['y_allowed'],
            'qual_x_reason': None if diag is None else diag['x_reason'],
            'qual_y_reason': None if diag is None else diag['y_reason'],
        }
        entry.update(analyzer.metrics(z_grab))
        history.append(entry)

    if not plan['dry_run']:
        actuator.stop_motion()
    outcome['samples'] = phase_samples
    outcome['ac_samples'] = ac_samples
    outcome['anchor'] = anchor
    outcome['guard_blocked'] = list(guard.blocked)
    outcome['plan'] = plan
    return history, outcome


# ============================================================================
# 事后对照报告
# ============================================================================

def _phase_report(tag: str, samples: list[tuple[float, float, float]], plan: dict) -> dict:
    # 头/尾残摆必须用至少一个完整摆周期的窗: 短于一个周期的窗会系统性低估慢摆
    # (site_config 的 _window_note 明确禁止用短窗判稳)。调度器自己的 0.55×T 判稳窗
    # 只用于 settle_time(), 不用来报残摆。
    report_window = max(
        adaptive_settle_window(
            plan['z_grab'], base_window=plan['window_base_s'],
            window_periods=plan['window_periods'],
            sheave_height=plan['sheave_height'], grab_offset=plan['grab_offset'],
            max_window=plan['window_max_s'],
        ),
        plan['period_s'],
    )
    tail = [s for s in samples if samples and s[0] >= samples[-1][0] - report_window]
    head = [s for s in samples if samples and s[0] <= samples[0][0] + report_window]
    enough = len(samples) >= 64
    tau = envelope_tau(samples) if enough else None
    return {
        'label': tag,
        'n_samples': len(samples),
        'span_s': round(samples_time_span(samples), 2),
        'report_window_s': round(report_window, 2),
        'peak_abs_x_deg': round(peak_pp_deg(samples, 1), 3),
        'peak_abs_y_deg': round(peak_pp_deg(samples, 2), 3),
        'head_pp_deg': window_pp(head, report_window) if head else None,
        'tail_pp_deg': window_pp(tail, report_window) if tail else None,
        'envelope_tau_s': None if tau is None else round(tau, 2),
        'envelope_tau_reason': (
            None if tau is not None
            else ('样本不足 (<64)' if not enough else '未在衰减 (后半段没有变小)')
        ),
        'settle_time_s': (
            None if (st := settle_time(samples, plan)) is None else round(st, 2)
        ),
    }


def _print_report(outcome: dict, history: list[dict], plan: dict) -> dict:
    report = {}
    tol = plan['tol_deg']
    print()
    print('=' * 78)
    print('A/B 对照报告')
    print('=' * 78)
    a = _phase_report('A 观测(不消摆)', outcome['samples']['a'], plan) if plan['observe_s'] > 0 else None
    b = _phase_report('B 消摆', outcome['samples']['b'], plan)
    report['a'] = a
    report['b'] = b

    theory_tau = 2.0 * plan['rope_length_m'] / plan['effective_gain'] if plan['effective_gain'] > 0 else None
    for seg in (a, b):
        if seg is None:
            continue
        print(f'\n[{seg["label"]}]  {seg["n_samples"]} 样本, 跨度 {seg["span_s"]}s')
        print(f'  段内最大 |θ| : θx {seg["peak_abs_x_deg"]:.3f}°  θy {seg["peak_abs_y_deg"]:.3f}°')
        head = seg['head_pp_deg']
        tail = seg['tail_pp_deg']
        win = seg['report_window_s']
        if head and head['sample_count'] >= 3:
            print(f'  段首窗去趋势PP ({win:.1f}s): θx {_fmt(head["detrend_pp_x_deg"])}°  '
                  f'θy {_fmt(head["detrend_pp_y_deg"])}°')
        if tail and tail['sample_count'] >= 3:
            print(f'  段尾窗去趋势PP ({win:.1f}s): θx {_fmt(tail["detrend_pp_x_deg"])}°  '
                  f'θy {_fmt(tail["detrend_pp_y_deg"])}°   (容差 {tol:.2f}°)')
            print(f'  段尾窗原始PP          : θx {_fmt(tail["raw_pp_x_deg"])}°  '
                  f'θy {_fmt(tail["raw_pp_y_deg"])}°')
        tau_txt = (f'{seg["envelope_tau_s"]:.2f} s' if seg['envelope_tau_s'] is not None
                   else f'n/a  ({seg["envelope_tau_reason"]})')
        print(f'  包络时间常数 τ: {tau_txt}')
        st = seg['settle_time_s']
        print('  进入容差时刻  : ' + ('未在观测期内收敛' if st is None else f'{st:.1f} s')
              + '   (判据: 去趋势PP < 容差且窗填满, 与调度器判稳同口径)')

    if theory_tau is not None:
        print(f'\n  理论对照: K_eff={plan["effective_gain"]:.2f}, L={plan["rope_length_m"]:.2f}m, '
              f'ζ={plan["zeta"]:.2f} → τ_env = 2L/K = {theory_tau:.2f} s')

    if a is not None:
        if a['peak_abs_x_deg'] < max(3 * tol, 0.5) and a['peak_abs_y_deg'] < max(3 * tol, 0.5):
            print('\n  [警告] A 段没有检测到明显摆动 —— 自然衰减基线无效,')
            print('         本次 A/B 对照不成立 (可能是推钩太轻, 或倾角仪没读到)。')
            report['a_valid'] = False
        else:
            report['a_valid'] = True

    # 逐轴三态: 一轴的改善不得抵消另一轴的恶化 (项目硬规则)。
    # 被增长保护撤回的轴必须单独标注: 那不是"消摆失败", 是保护主动停了它,
    # 拿它的 τ/残摆当性能结论会把两件事混起来。
    guarded = [e['axis'] for e in outcome.get('guard_events', [])]
    print('\n  逐轴结论 (不合成):')
    verdicts = {}
    for axis, idx in (('X', 0), ('Y', 1)):
        note = '   [增长保护已撤回该轴, 非消摆失败]' if axis.lower() in guarded else ''
        if a is not None and a['tail_pp_deg'] and b['tail_pp_deg']:
            key = 'detrend_pp_x_deg' if idx == 0 else 'detrend_pp_y_deg'
            a_pp = a['tail_pp_deg'][key] or 0.0
            b_pp = b['tail_pp_deg'][key] or 0.0
            verdicts[axis] = axis_verdict(a_pp, b_pp, tol)
            print(f'    {axis} 轴: A 段尾残摆 {a_pp:.3f}° → B 段尾残摆 {b_pp:.3f}°  '
                  f'⇒ {verdicts[axis]}{note}')
        else:
            verdicts[axis] = '无法比较(样本不足)'
            print(f'    {axis} 轴: 无法比较 (样本不足){note}')
    report['verdicts'] = verdicts
    report['guarded_axes'] = guarded

    ticks = outcome.get('phase_b_ticks') or 0
    zero = outcome.get('gate_on_zero_dv') or 0
    if ticks and zero > ticks * 0.5:
        print(f'\n  !! B 段 {ticks} 拍里有 {zero} 拍"门限开着但 Δv = 0" —— 消摆没在发生。')
        if plan['strategy'] == 'terminal':
            print('     这是因为你选了 --strategy terminal: 作业同款终端通路在原地')
            print('     拿不到资格, 所以恒为 0 —— 这是本次运行的预期结果, 不是故障。')
            print('     要真正消摆, 去掉 --strategy terminal 用默认的机制通路。')
        print('     其余可能原因, 按可能性排序:')
        rate = outcome.get('incl_hz')
        print('       1. 倾角仪读数在被重复/冻结 → θ̇≈0 → 幅度门限 '
              f'({plan["amp_floor_deg"]:.3f}°) 把 Δv 全部归零。'
              f'本段有效更新率 ' + (f'{rate:.1f}Hz' if rate is not None else 'n/a'))
        print('       2. 摆幅本身就低于幅度门限 (推得太轻)')
        print('       3. 位移预算已用满且只剩朝锚点回拉的半周期')
        print('       4. guard 已撤回该轴')
    report['gate_on_zero_dv'] = zero
    report['phase_b_ticks'] = ticks
    if outcome.get('amp_gated') and ticks:
        gated = outcome['amp_gated']
        # 只提"本来有摆可消"的轴: 全程没摆的轴当然每拍都被门限挡住, 提它只会误导。
        peaks = ((b or {}).get('peak_abs_x_deg') or 0.0, (b or {}).get('peak_abs_y_deg') or 0.0)
        parts = [
            f'{axis} {gated[i]}/{ticks}'
            for i, axis in enumerate('XY')
            if gated[i] > ticks * 0.5 and peaks[i] > 3 * tol
        ]
        if parts:
            print(f'     其中幅度门限挡掉的拍数: {", ".join(parts)} '
                  f'(门限 {plan["amp_floor_deg"]:.3f}°)')
    report['amp_gated'] = outcome.get('amp_gated')

    anchor = outcome.get('anchor')
    if anchor:
        xs = [abs(row['x'] - anchor[0]) for row in history]
        ys = [abs(row['y'] - anchor[1]) for row in history]
        if xs:
            over = max(max(xs), max(ys)) > plan['budget_m']
            print(f'\n  锚点漂移: max |Δx| {max(xs) * 1000:.0f} mm, max |Δy| {max(ys) * 1000:.0f} mm'
                  f'   (控制预算 ±{plan["budget_m"] * 100:.0f} cm)')
            if over:
                print('    注: 实测略超预算是正常的 —— 预算钳的是"发给 PLC 的速度指令", '
                      '车体本身的')
                print('        伺服惯性会让它多滑一小段; 回中项随后把它拉回来。'
                      '持续单向漂下去才是异常。')
            report['anchor_drift_mm'] = [round(max(xs) * 1000, 1), round(max(ys) * 1000, 1)]

    b_rows = [row for row in history if row['phase'] == 'B']
    if b_rows:
        # 分两组报: 实际下发的 (vx_cmd/vy_cmd, 已被 max_correction 钳过) 和算法要求
        # 的 (dvx/dvy, 钳之前)。只报后者会出现"峰值 284mm/s"而速度上限是 200mm/s
        # 这种自相矛盾的数字 —— 那是没发出去的请求量, 不是车真的做了什么。
        sent_rms = [math.sqrt(sum(row[f'v{a}_cmd'] ** 2 for row in b_rows) / len(b_rows)) * 1000
                    for a in ('x', 'y')]
        sent_peak = [max(abs(row[f'v{a}_cmd']) for row in b_rows) * 1000 for a in ('x', 'y')]
        req_peak = [max(abs(row[f'dv{a}']) for row in b_rows) * 1000 for a in ('x', 'y')]
        print(f'  实际下发: RMS ({sent_rms[0]:.1f}, {sent_rms[1]:.1f}) mm/s, '
              f'峰值 ({sent_peak[0]:.1f}, {sent_peak[1]:.1f}) mm/s')
        print(f'  算法要求: 峰值 ({req_peak[0]:.1f}, {req_peak[1]:.1f}) mm/s '
              f'(上限 {plan["max_correction"] * 1000:.0f} mm/s)')
        print(f'  撞速度上限 {outcome["dv_saturated"]} 次, 撞位移预算 {outcome["budget_clamped"]} 次')
        report['sent_rms_mm_s'] = [round(v, 2) for v in sent_rms]
        report['sent_peak_mm_s'] = [round(v, 2) for v in sent_peak]
        report['requested_peak_mm_s'] = [round(v, 2) for v in req_peak]

    if outcome.get('bias_primed'):
        bp = outcome['bias_primed']
        print(f'  解出的静态零偏: θx {bp[0]:+.3f}°, θy {bp[1]:+.3f}° (已置入高通)')
    if outcome['guard_events']:
        print('  增长保护事件:')
        for ev in outcome['guard_events']:
            print(f'    t={ev["t"]:.1f}s {ev["axis"].upper()} 轴 '
                  f'{ev["baseline_deg"]:.3f}° → {ev["pp_deg"]:.3f}° PP')
    else:
        print('  增长保护: 未触发')

    # 终端通路诊断的结论 —— 这是"作业那条通路在原地会不会介入"的直接证据。
    allowed = outcome.get('qualifier_ever_allowed')
    if allowed is not None:
        print(f'\n  终端通路(作业同款)资格诊断: 资格曾在 X={allowed[0]} Y={allowed[1]} 通过')
        print('    原地自由摆没有受迫激励时, 预期一直是 insufficient_excitation ——')
        print('    这正是"终端通路在原地不会介入"的直接证据, 不是推断。')
        print('    (B 段机制通路真的在动车, 支点有了受迫运动, 资格有可能转而通过 ——')
        print('     那说明终端通路在原地是"先有鸡还是先有蛋", 而不代表它现场无效。)')
    report['qualifier_ever_allowed'] = allowed

    if outcome.get('aborted'):
        print(f'\n  !! 提前中止: {outcome["aborted"]}')
        print('     !! 若摆幅是放大而非收敛, 先查 anti_sway_theta_*_sign '
              '(用 script/check_sway_sign.py), 不要靠加增益硬顶。')
    report['aborted'] = outcome.get('aborted')

    print()
    print('  本程序验证的是机制通路 (Δv=+K·θ_ac + 位移预算 + 增长撤回) 在原地是否成立,')
    print('  不是作业终端通路的现场效果, 也不是端到端作业验收。')
    print('=' * 78)
    return report


def _save_artifacts(prefix: str, history: list[dict], report: dict,
                    outcome: dict, plan: dict, args) -> None:
    payload = {
        'model': '现场原地保持消摆验证 — 机制通路 (terminal_mode=False)',
        'notice': ('验证机制通路, 非作业终端通路现场效果; 一次推钩的初始摆幅/相位不可复现, '
                   'A/B 是同一段轨迹的前后窗, 不是两次独立重复'),
        'mode': 'dry-run' if plan['dry_run'] else 'live',
        'cli': vars(args),
        'plan': _jsonable(plan),
        'report': _jsonable(report),
        'outcome': _jsonable({k: v for k, v in outcome.items() if k != 'samples'}),
        'history': _jsonable(history),
    }
    json_path = Path(f'{prefix}.json')
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')

    if not history:
        print(f'[输出] {json_path} (无历史数据, 跳过 PNG)')
        return

    t = [row['t'] for row in history]
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    for idx, (axis, color) in enumerate((('x', 'tab:blue'), ('y', 'tab:orange'))):
        axes[0].plot(t, [math.degrees(row[f'theta_{axis}']) for row in history],
                     color=color, lw=1.2, label=f'θ{axis} 原始')
        axes[0].plot(t, [math.degrees(row[f'theta_ac_{axis}']) for row in history],
                     color=color, lw=0.8, ls='--', alpha=.7, label=f'θ{axis}_ac')
    for sign in (1, -1):
        axes[0].axhline(sign * plan['tol_deg'], color='k', ls=':', lw=0.8,
                        label='判稳容差' if sign > 0 else None)
    axes[0].set_ylabel('摆角 [deg]')
    axes[0].legend(fontsize=8, ncol=4)
    axes[0].set_title(f'原地保持消摆验证  Z={plan["z_grab"]:.2f}m  L={plan["rope_length_m"]:.2f}m  '
                      f'K_eff={plan["effective_gain"]:.2f}  预算±{plan["budget_m"] * 100:.0f}cm')

    anchor = outcome.get('anchor') or (0.0, 0.0)
    for idx, (axis, color) in enumerate((('x', 'tab:blue'), ('y', 'tab:orange'))):
        axes[1].plot(t, [(row[axis] - anchor[idx]) * 1000 for row in history],
                     color=color, lw=1.2, label=f'Δ{axis}')
    for sign in (1, -1):
        axes[1].axhline(sign * plan['budget_m'] * 1000, color='r', ls='--', lw=0.8)
    axes[1].set_ylabel('相对锚点位移 [mm]')
    axes[1].legend(fontsize=8)

    for idx, (axis, color) in enumerate((('x', 'tab:blue'), ('y', 'tab:orange'))):
        axes[2].plot(t, [row[f'dv{axis}'] * 1000 for row in history],
                     color=color, lw=1.2, label=f'Δv{axis}')
    axes[2].set_ylabel('下发修正 [mm/s]')
    axes[2].set_xlabel('t [s]')
    axes[2].legend(fontsize=8)

    if plan['observe_s'] > 0:
        for ax in axes:
            ax.axvline(plan['observe_s'], color='0.4', lw=1.0, ls='-.')
            ax.text(plan['observe_s'], ax.get_ylim()[1], ' A|B', va='top', fontsize=8, color='0.3')

    fig.tight_layout()
    png_path = Path(f'{prefix}.png')
    fig.savefig(png_path, dpi=150)
    plt.close(fig)
    print(f'[输出] {json_path} 和 {png_path}')


# ============================================================================
# 入口
# ============================================================================

def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    if args.z_grab is not None and not args.dry_run:
        # 用一个编出来的高度去驱动真车, 等于用错误的绳长算 K 和摆周期 —— 直接禁掉。
        parser.error('--z-grab 只能配合 --dry-run 使用 (它不是给真车测试用的)')
    config, site = _make_config(args)
    if config is None:
        return 1

    plan_probe = resolve_plan(config, 1.5, args)
    if plan_probe['budget_m'] <= 0.0:
        print(f'[错误] 位移预算 = {plan_probe["budget_m"]:.3f} m ≤ 0 — '
              f'锁轴阻尼会被 _locked_axis_dv 直接返回 0, 车一动不动,')
        print('       看起来像"消摆无效"。这是 anti_sway_settle_max_shift (贴近取放货) 的语义,')
        print('       原地验证请用 anti_sway_locked_max_shift 或显式 --max-shift。')
        return 1
    if config.anti_sway_sway_gain <= 0.0:
        print('[错误] 防摇增益 K ≤ 0 — 没有可验证的修正量。')
        return 1

    start_ros_bridge(
        angle_scale=config.anti_sway_angle_scale,
        roll_bias=config.anti_sway_roll_bias,
        pitch_bias=config.anti_sway_pitch_bias,
        axis_mapping=build_sway_axis_mapping(site),
        roll_bias_closed=config.anti_sway_roll_bias_closed,
        pitch_bias_closed=config.anti_sway_pitch_bias_closed,
    )

    print('等待倾角仪与定位数据...')
    deadline = time.monotonic() + _PREFLIGHT_WAIT_S
    while time.monotonic() < deadline:
        if get_inclination_raw() is not None and get_latest_pose() is not None:
            break
        time.sleep(0.1)
    if get_inclination_raw() is None:
        print('[错误] 无倾角仪数据 — 防摇无法工作, 先用 script/check_sway_sign.py 排查')
        return 1
    if get_latest_pose() is None:
        print('[错误] 无 /localization_pose — 位移预算无法约束, 拒绝动车')
        return 1
    if get_sway_state() is None:
        print('[错误] 倾角仪数据过期 (>1s) — 拒绝动车')
        return 1

    # 活性检查: 冻结的读数会让后面所有零偏/摆幅/衰减统计都变成假结论。
    activity: list[tuple[float, float, float]] = []
    t_act = time.monotonic()
    while time.monotonic() - t_act < 3.0:
        sway = get_sway_state()
        if sway is not None:
            activity.append((time.monotonic(), sway['theta_x'], sway['theta_y']))
        time.sleep(0.05)
    if frozen_sensor(activity):
        print(f'[错误] {len(activity)} 次采样里读数几乎不变 — 倾角仪疑似冻结, 拒绝动车。')
        return 1

    plc_kwargs = {'allow_mock': args.allow_mock_plc}
    if args.plc_lib:
        plc_kwargs['lib_path'] = args.plc_lib
    plc = create_plc(**plc_kwargs)
    try:
        if plc.connect(args.plc_ip) != 0:
            print(f'[错误] PLC 连接失败: {args.plc_ip}')
            return 1
        plc.start_heartbeat()

        z_grab = plc.get_lift_height()
        if z_grab is None or not math.isfinite(z_grab):
            if args.z_grab is None:
                print('[错误] 抓钩高度读数无效 — 绳长模型无法确定, 拒绝动车')
                print('       (dry-run 自检时可用 --z-grab 指定一个高度)')
                return 1
            print(f'[警告] PLC 无抓钩高度回读, 按 --z-grab = {args.z_grab:.2f} m 计算 '
                  f'(dry-run 自检模式, 不下发任何指令)。')
            z_grab = float(args.z_grab)

        plan = resolve_plan(config, z_grab, args)
        _print_plan(plan, site, args)
        if plan['observe_s'] > 0 and plan['observe_s'] < 2 * plan['period_s']:
            print(f'[警告] A 段 {plan["observe_s"]:.0f}s 短于 2 个摆周期 '
                  f'({2 * plan["period_s"]:.1f}s) — 自然衰减基线会不可靠。')

        print('请先把抓钩抬到安全高度、清空周围人员; 本程序不动 Z, 只驱动 XY。')
        if not args.yes and not plan['dry_run'] and not args.allow_mock_plc:
            if input('确认开始? 输入 yes 继续: ').strip().lower() != 'yes':
                print('已取消')
                return 0

        source = RosPositionSource(
            lift_height_provider=plc.get_lift_height,
            use_native_xy_velocity=True,
        )
        actuator = PlcActuator(
            plc,
            initial_z=z_grab,
            big_car_sign=-1.0 if args.invert_big_car else 1.0,
            small_car_sign=-1.0 if args.invert_small_car else 1.0,
            lift_sign=-1.0 if args.invert_lift else 1.0,
            min_lift_height=args.min_lift_height,
        )
        actuator.set_z_reference(z_grab)

        try:
            history, outcome = _run(args, config, plc, source, actuator, plan)
        except KeyboardInterrupt:
            print('\n[中断] 操作员停止 — 立即停车')
            actuator.stop_motion()
            return 130
        except ControlStoppedError as exc:
            print(f'\n[停止] {exc}')
            actuator.stop_motion()
            return 130
        finally:
            actuator.stop_motion()
            print('\n[完成] XY 已归零, Z 保持在当前高度。')

        report = _print_report(outcome, history, plan)
        _save_artifacts(args.output, history, report, outcome, plan, args)
        return 0
    finally:
        plc.disconnect()  # 内部会停心跳


if __name__ == '__main__':
    sys.exit(main())
