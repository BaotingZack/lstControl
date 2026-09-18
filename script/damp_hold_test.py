#!/usr/bin/env python3
"""现场联调 — 原地消摆闭环测试 (第一次让防摇真正动车时用这个)。

与完整作业相比, 本脚本只做一件事: 以当前位置为锚点, 在 XY 上做预算内的软阻尼,
直到摆幅进入容差或等满上限。**不动 Z, 不动抓钩, 不走行**。走的是
OperationScheduler.damp_in_place() → 与作业流程中悬停判稳完全同一份代码, 因此
这里整定出来的增益可以直接用于作业。

为什么要有它: 悬停阻尼只发生在作业流程的判稳阶段, 若不先单独验证, 第一次闭环
就得连着抓放一起跑 —— 出问题时变量太多, 也不安全。

前置条件 (必须先做完, 见 script/check_sway_sign.py):
  1. 倾角仪轴映射与符号已静态核验 (朝 +X 推抓钩 → θx 为正)
  2. 各工作高度的零偏已核验/更新到 site_config

安全要点:
  - 运行前必须手握急停; 脚本内的 Ctrl-C 会立即停车, 但急停是最后防线
  - 首次测试请空载、抓钩抬到安全高度、周围无人
  - 摆幅一旦在阻尼下放大 (而非收敛), 说明符号仍然反了 — 立即急停。调度器内置
    发散保护会自动撤掉阻尼并打印告警, 但不要指望它替代急停

用法:
    # 空载、抓钩在安全高度, 手动激励一次摆动后立刻运行
    python3 script/damp_hold_test.py --plc-ip 192.168.1.100 --duration 15

    # 整定增益 (不改 site_config)
    python3 script/damp_hold_test.py --plc-ip 192.168.1.100 --anti-sway-gain 3.9

    # 只观测不动车 (等价于 gain=0, 用来量自然衰减作对照)
    python3 script/damp_hold_test.py --plc-ip 192.168.1.100 --observe-only
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from crane_model import ControlStoppedError
from operation_scheduler import OperationScheduler
from plc_interface import PlcActuator, create_plc
from ros_bridge import (
    RosPositionSource,
    get_inclination_raw,
    get_latest_pose,
    get_sway_state,
    start_ros_bridge,
)
from site_config import (
    DEFAULT_SITE_CONFIG_PATH,
    build_sway_axis_mapping,
    load_site_config,
)
from sway_controller import damping_ratio, envelope_decay_time
from sway_settle import effective_rope_length


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description='原地消摆闭环测试 — 只在 XY 做软阻尼, 不动 Z/抓钩/走行',
    )
    p.add_argument('--plc-ip', required=True, help='PLC IP')
    p.add_argument('--plc-lib', default=None, help='PLC 动态库路径')
    p.add_argument('--allow-mock-plc', action='store_true',
                   help='PLC 不可用时退回 MockPLC (仅用于自检脚本本身)')
    p.add_argument('--config', default=str(DEFAULT_SITE_CONFIG_PATH))
    p.add_argument('--duration', type=float, default=15.0,
                   help='消摆上限时长 [s] (默认 15)')
    p.add_argument('--anti-sway-gain', type=float, default=None,
                   help='覆盖 K [m/s per rad], 不改 site_config')
    p.add_argument('--max-shift', type=float, default=None,
                   help='覆盖位移预算 [m] (默认用 anti_sway_locked_max_shift)')
    p.add_argument('--observe-only', action='store_true',
                   help='不动车, 只观测摆幅 (用于量自然衰减作对照)')
    p.add_argument('--invert-big-car', action='store_true')
    p.add_argument('--invert-small-car', action='store_true')
    p.add_argument('--invert-lift', action='store_true')
    p.add_argument('--min-lift-height', type=float, default=0.35)
    p.add_argument('--yes', action='store_true', help='跳过运行前确认 (慎用)')
    return p


def _make_config(args):
    """复用 main._config_from_args, 保证与作业时的配置解析完全一致。"""
    import main

    site = load_site_config(args.config)
    if args.observe_only:
        site = dict(site, anti_sway_sway_gain=0.0)
    elif args.anti_sway_gain is not None:
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


def _print_plan(config, z_grab: float, duration: float, max_shift: float) -> None:
    L = effective_rope_length(
        z_grab,
        sheave_height=config.rope_length_sheave_height,
        grab_offset=config.rope_length_grab_offset,
    )
    K = config.anti_sway_sway_gain
    K_hold = K * config.anti_sway_settle_gain_scale
    print()
    print('=' * 72)
    print('原地消摆闭环测试')
    print('=' * 72)
    print(f'  抓钩高度 Z      = {z_grab:.2f} m  → 有效摆长 L = {L:.2f} m')
    if K <= 0.0:
        print('  防摇增益        = 0 (纯观测, 不动车)')
    else:
        print(f'  防摇增益 K      = {K:.2f}  (悬停期 ×{config.anti_sway_settle_gain_scale:.1f} '
              f'= {K_hold:.2f})')
        print(f'  等效阻尼比 ζ    = {damping_ratio(K_hold, L):.2f}')
        print(f'  预期 1°→{config.hook_sway_angle_tol_deg:.1f}°  = '
              f'{envelope_decay_time(K_hold, L, 1.0, config.hook_sway_angle_tol_deg):.1f} s')
        print(f'  速度上限        = {config.anti_sway_max_correction:.2f} m/s')
        print(f'  位移预算        = ±{max_shift * 100:.0f} cm (相对当前位置)')
        print(f'  锚点回中增益    = {config.anti_sway_hold_recenter_gain:.2f} 1/s')
    print(f'  判稳容差        = {config.hook_sway_angle_tol_deg:.2f}° 去趋势峰峰值')
    print(f'  上限时长        = {duration:.1f} s')
    print('=' * 72)
    if K > 0.0:
        print('!! 行车将在 XY 上小幅移动。请手握急停, 确认周围无人。')
        print('!! 若摆幅在阻尼下放大而非收敛 → 立即急停 (倾角符号仍反)。')
    print()


def main() -> int:
    args = _build_parser().parse_args()
    config, site = _make_config(args)
    if config is None:
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
    deadline = time.monotonic() + 8.0
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
            print('[错误] 抓钩高度读数无效 — 绳长模型无法确定, 拒绝动车')
            return 1

        max_shift = (
            config.anti_sway_locked_max_shift if args.max_shift is None
            else args.max_shift
        )
        _print_plan(config, z_grab, args.duration, max_shift)

        if not args.yes and config.anti_sway_sway_gain > 0.0:
            if input('确认开始? 输入 yes 继续: ').strip().lower() != 'yes':
                print('已取消')
                return 0

        source = RosPositionSource(lift_height_provider=plc.get_lift_height)
        actuator = PlcActuator(
            plc,
            initial_z=z_grab,
            big_car_sign=-1.0 if args.invert_big_car else 1.0,
            small_car_sign=-1.0 if args.invert_small_car else 1.0,
            lift_sign=-1.0 if args.invert_lift else 1.0,
            min_lift_height=args.min_lift_height,
        )
        # Z 设定值锚在当前高度: 本测试全程不动 Z
        actuator.set_z_reference(z_grab)

        scheduler = OperationScheduler(
            plc=plc,
            source=source,
            actuator=actuator,
            config=config,
            is_simulation=False,
            sway_provider=get_sway_state,
        )

        t0 = time.monotonic()
        try:
            scheduler.damp_in_place(args.duration, damp_max_shift=max_shift)
        except KeyboardInterrupt:
            print('\n[中断] 操作员停止 — 立即停车')
            return 130
        except ControlStoppedError as exc:
            print(f'[停止] {exc}')
            return 130
        finally:
            actuator.stop_motion()
            print(f'[完成] 耗时 {time.monotonic() - t0:.1f}s, XY 已归零')
        return 0
    finally:
        plc.disconnect()  # 内部会停心跳


if __name__ == '__main__':
    sys.exit(main())
