"""Build and persist PD / settle run logs for debugging."""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

MAX_RECORDED_STEPS = 15_000

RUN_LOG_VERSION = 1
SETTLE_STEPS_SCHEMA = 'settle_slim_v3'
SETTLE_LOG_INTERVAL_S = 1.0
# deepseek 尾段诊断：|θ_ac| 已很小仍振荡时加密采样
SETTLE_TAIL_LOG_INTERVAL_S = 0.25
SETTLE_TAIL_RESIDUAL_DEG = 1.2


def _utc_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace('+00:00', 'Z')


def site_config_debug_subset(site: dict[str, Any] | None) -> dict[str, Any]:
    """Non-secret anti-sway / rope fields useful when reproducing a run."""
    if not site:
        return {}
    keys = (
        'enable_anti_sway',
        'anti_sway_sway_gain',
        'anti_sway_max_correction',
        'anti_sway_approach_pos',
        'anti_sway_gantry_kp',
        'anti_sway_gantry_kd',
        'anti_sway_gantry_max_correction',
        'anti_sway_overhead_trolley_mass',
        'anti_sway_overhead_load_mass',
        'anti_sway_baseline_gain',
        'anti_sway_baseline_max_correction',
        'anti_sway_baseline_deadband_deg',
        'anti_sway_crane_remote_kp',
        'anti_sway_deepseek_gain',
        'anti_sway_deepseek_max_correction',
        'anti_sway_deepseek_deadband_deg',
        'anti_sway_deepseek_window_periods',
        'anti_sway_deepseek_sign',
        'anti_sway_deepseek_sign_x',
        'anti_sway_deepseek_sign_y',
        'anti_sway_deepseek_lead_s',
        'anti_sway_deepseek_lead_ac_cap_ratio',
        'anti_sway_deepseek_lead_ac_cap_min_deg',
        'anti_sway_deepseek_lead_rope_ref_m',
        'anti_sway_deepseek_lead_rope_exp',
        'anti_sway_deepseek_lead_short_min_ratio',
        'anti_sway_deepseek_min_window_periods',
        'anti_sway_deepseek_rope_short_m',
        'anti_sway_deepseek_rope_gain_ref_m',
        'anti_sway_deepseek_rope_gain_max_scale',
        'anti_sway_deepseek_zeta_ref',
        'anti_sway_deepseek_max_shift_m',
        'anti_sway_deepseek_guard',
        'anti_sway_quad_kv',
        'anti_sway_quad_tau_lead',
        'anti_sway_quad_max_correction',
        'anti_sway_quad_drive_accel',
        'anti_sway_quad_dc_tau',
        'anti_sway_quad_recenter_gain',
        'anti_sway_input_shape_zeta',
        'rope_length_sheave_height',
        'rope_length_grab_offset',
        'hook_sway_angle_tol_deg',
        'hook_sway_settle_hold_sec',
    )
    out: dict[str, Any] = {}
    for key in keys:
        if key in site:
            out[key] = site[key]
    return out


def _deepseek_residual_peak_deg(debug: dict[str, Any]) -> float | None:
    ac_x = debug.get('theta_ac_x_deg')
    ac_y = debug.get('theta_ac_y_deg')
    if ac_x is None and ac_y is None:
        return None
    return max(abs(float(ac_x or 0.0)), abs(float(ac_y or 0.0)))


def slim_settle_step(step: dict[str, Any]) -> dict[str, Any]:
    """Sampled settle run log: inclinometer + XY velocity commands (+ anti-sway Δv)."""
    theta_x = step.get('theta_x')
    theta_y = step.get('theta_y')
    sway_valid = step.get('sway_valid')
    if sway_valid is None:
        sway_valid = theta_x is not None and theta_y is not None
    peak_deg = None
    if theta_x is not None and theta_y is not None:
        peak_deg = math.degrees(max(abs(float(theta_x)), abs(float(theta_y))))
    z_pos = step.get('z_measured')
    if z_pos is None:
        z_pos = step.get('z')
    out: dict[str, Any] = {
        't': step.get('t'),
        'roll': step.get('roll'),
        'pitch': step.get('pitch'),
        'theta_x': theta_x,
        'theta_y': theta_y,
        'theta_peak_deg': peak_deg,
        'z_grab': z_pos,
        'sway_valid': bool(sway_valid),
        'vx_cmd': step.get('vx_cmd'),
        'vy_cmd': step.get('vy_cmd'),
        'antisway_dvx': step.get('antisway_dvx', 0.0),
        'antisway_dvy': step.get('antisway_dvy', 0.0),
    }
    # 只有提供 debug_snapshot() 的算法（如 deepseek_anti）才会带这几个键，
    # 其它算法的 settle 日志结构保持不变。
    debug = step.get('anti_sway_debug')
    if isinstance(debug, dict):
        out['static_theta_x_deg'] = debug.get('theta_static_x_deg')
        out['static_theta_y_deg'] = debug.get('theta_static_y_deg')
        out['static_ready'] = debug.get('ready')
        out['ready_x'] = debug.get('ready_x')
        out['ready_y'] = debug.get('ready_y')
        out['theta_ac_x_deg'] = debug.get('theta_ac_x_deg')
        out['theta_ac_y_deg'] = debug.get('theta_ac_y_deg')
        out['theta_ac_peak_deg'] = _deepseek_residual_peak_deg(debug)
        out['amp_x_deg'] = debug.get('amp_x_deg')
        out['amp_y_deg'] = debug.get('amp_y_deg')
        out['coverage_x'] = debug.get('coverage_x')
        out['coverage_y'] = debug.get('coverage_y')
        out['rope_m'] = debug.get('rope_m')
        out['period_s'] = debug.get('period_s')
        out['gain_eff'] = debug.get('gain_eff')
        out['lead_s_eff'] = debug.get('lead_s')
        # 诊断字段：判断"某轴越消越大"到底是反号、增长保护、还是位移预算。
        out['rate_x_deg_s'] = debug.get('rate_x_deg_s')
        out['rate_y_deg_s'] = debug.get('rate_y_deg_s')
        out['guard_x'] = debug.get('guard_x')
        out['guard_y'] = debug.get('guard_y')
        out['guard_trips'] = debug.get('guard_trips')
        out['budget_blocked'] = debug.get('budget_blocked')
        out['tail_recenter_x'] = debug.get('tail_recenter_x')
        out['tail_recenter_y'] = debug.get('tail_recenter_y')
        out['static_frozen_x'] = debug.get('static_frozen_x')
        out['static_frozen_y'] = debug.get('static_frozen_y')
        out['tail_ema_x'] = debug.get('tail_ema_x')
        out['tail_ema_y'] = debug.get('tail_ema_y')
        out['lead_s_x'] = debug.get('lead_s_x')
        out['lead_s_y'] = debug.get('lead_s_y')
    return out


def settle_step_worth_recording(
    step: dict[str, Any],
    *,
    last_log_t: float | None,
) -> bool:
    """Decimate high-rate PD loop (~10 Hz) to ~1 Hz; dense tail for deepseek."""
    if last_log_t is None:
        return True
    t = float(step.get('t') or 0.0)
    if (t - last_log_t) >= SETTLE_LOG_INTERVAL_S:
        return True
    debug = step.get('anti_sway_debug')
    if not isinstance(debug, dict) or debug.get('mode') != 'deepseek_anti':
        return False
    residual = _deepseek_residual_peak_deg(debug)
    if residual is None:
        return False
    if not debug.get('ready'):
        return False
    if residual > SETTLE_TAIL_RESIDUAL_DEG:
        return False
    return (t - last_log_t) >= SETTLE_TAIL_LOG_INTERVAL_S


def analyze_deepseek_settle_steps(
    steps: list[dict[str, Any]],
    *,
    site: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Offline metrics + tuning hints from ``settle_slim_v3`` steps."""
    if not steps:
        return {'ok': False, 'reason': 'no_steps'}
    ds = [s for s in steps if s.get('theta_ac_peak_deg') is not None]
    if not ds:
        return {'ok': False, 'reason': 'not_deepseek_or_missing_debug'}

    def fmax(key: str, subset: list[dict]) -> float:
        return max(float(s.get(key) or 0.0) for s in subset)

    t_end = float(steps[-1].get('t') or 0.0)
    t_ready = next(
        (float(s['t']) for s in ds if s.get('static_ready')),
        None,
    )
    tail = [s for s in ds if t_ready is not None and float(s['t']) >= t_ready]
    tail_late = [
        s for s in tail
        if (float(s['t']) - t_ready) >= max(3.0, 0.5 * (t_end - t_ready))
    ] if t_ready is not None else []

    rope_vals = [float(s['rope_m']) for s in ds if s.get('rope_m') is not None]
    lead_vals = [float(s['lead_s_eff']) for s in ds if s.get('lead_s_eff') is not None]
    rope_m = sum(rope_vals) / len(rope_vals) if rope_vals else None
    lead_med = sorted(lead_vals)[len(lead_vals) // 2] if lead_vals else None

    tol_deg = float((site or {}).get('hook_sway_angle_tol_deg', 0.05))

    peak_theta0 = max(
        float(s.get('theta_peak_deg') or 0.0) for s in steps[: min(20, len(steps))]
    )
    peak_ac0 = fmax('theta_ac_peak_deg', ds[: min(30, len(ds))])
    tail_ac_max = fmax('theta_ac_peak_deg', tail_late) if tail_late else None
    tail_theta_max = (
        max(float(s.get('theta_peak_deg') or 0.0) for s in tail_late)
        if tail_late else None
    )

    def _settle_peak_deg(s: dict) -> float:
        ac = float(s.get('theta_ac_peak_deg') or 0.0)
        return max(
            ac,
            float(s.get('amp_x_deg') or 0.0),
            float(s.get('amp_y_deg') or 0.0),
        )

    tail_settle_peak = (
        max(_settle_peak_deg(s) for s in tail_late) if tail_late else None
    )
    split_count = sum(
        1 for s in tail_late
        if float(s.get('theta_peak_deg') or 0.0) > tol_deg * 4
        and float(s.get('theta_ac_peak_deg') or 0.0) < tol_deg * 2
    )

    stall = 0
    stall_den = 0
    for s in tail:
        rx = abs(float(s.get('rate_x_deg_s') or 0.0))
        ry = abs(float(s.get('rate_y_deg_s') or 0.0))
        dv = abs(float(s.get('antisway_dvx') or 0.0)) + abs(
            float(s.get('antisway_dvy') or 0.0)
        )
        resid = float(s.get('theta_ac_peak_deg') or 0.0)
        if resid > SETTLE_TAIL_RESIDUAL_DEG:
            continue
        if max(rx, ry) < 0.25:
            continue
        stall_den += 1
        if dv < 1e-4:
            stall += 1
    stall_frac = (stall / stall_den) if stall_den else 0.0

    guard_trips = max(int(s.get('guard_trips') or 0) for s in ds)

    hints: list[str] = []
    if rope_m is not None and lead_med is not None:
        ref = float((site or {}).get('anti_sway_deepseek_lead_rope_ref_m', 3.5))
        if rope_m < ref - 0.15 and lead_med > ref / rope_m * 0.38:
            hints.append(
                f'短绳 L≈{rope_m:.2f}m 但现场 lead≈{lead_med:.2f}s 仍偏大，'
                '检查 lead_rope 调度是否生效（应 < 配置 lead_s）。'
            )
    if stall_frac >= 0.25 and tail_ac_max is not None and tail_ac_max > tol_deg:
        hints.append(
            f'尾段「有角速度无 Δv」占比 {stall_frac:.0%}，疑似 P+D 相消或死区；'
            '已启用 D 回退/绳长 lead 调度，若仍高可略降 deadband 或 lead_rope_exp。'
        )
    if guard_trips >= 2:
        hints.append(
            f'增长保护触发 {guard_trips} 次：查 sign_x/y 或某轴反号。'
        )
    if (
        tail_theta_max is not None
        and tail_ac_max is not None
        and tail_theta_max > tol_deg * 3
        and tail_ac_max < tol_deg * 2
    ):
        hints.append(
            f'θ–θ_ac 分裂：末段 |θ|≈{tail_theta_max:.2f}° 但 |θ_ac|≈{tail_ac_max:.2f}°，'
            'θ* 可能锁住包络；新逻辑含 θ* 慢跟踪与尾段弱阻尼，复查 stall/判稳峰值。'
        )
    if split_count >= 8:
        hints.append(
            f'尾段 {split_count} 个采样点 |θ|>|θ_ac| 显著，勿仅凭 θ_ac 验收。'
        )

    return {
        'ok': True,
        'duration_s': t_end,
        't_ready_s': t_ready,
        'rope_m_mean': rope_m,
        'lead_s_median': lead_med,
        'peak_theta_init_deg': peak_theta0,
        'peak_ac_init_deg': peak_ac0,
        'tail_theta_max_deg': tail_theta_max,
        'tail_ac_max_deg': tail_ac_max,
        'tail_settle_peak_deg': tail_settle_peak,
        'tail_theta_ac_split_samples': split_count,
        'tail_stall_frac': round(stall_frac, 3),
        'guard_trips_max': guard_trips,
        'hook_sway_tol_deg': tol_deg,
        'hints': hints,
    }


def run_status(cs) -> str:
    if cs is None:
        return 'idle'
    with cs.lock:
        if cs.running:
            return 'running'
        if cs.done:
            return 'done'
        if cs.stopped:
            return 'stopped'
        if cs.error:
            return 'error'
    return 'idle'


def build_run_log_document(
    cs,
    *,
    server: Any | None = None,
    client_supplement: dict[str, Any] | None = None,
    site: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Assemble export JSON from ControlState (+ optional browser sim replay)."""
    snap = cs.snapshot() if cs is not None else {}
    if cs is not None:
        with cs.lock:
            steps = list(cs.steps)
            events = list(cs.events)
            meta = {
                'kind': cs.run_kind,
                'anti_sway_mode': cs.anti_sway_mode,
                'sim': cs.run_sim,
                'started_at': cs.started_at_iso,
                'finished_at': cs.finished_at_iso,
                'steps_truncated': cs.steps_truncated,
            }
    else:
        steps = []
        events = []
        meta = {
            'kind': None,
            'anti_sway_mode': None,
            'sim': False,
            'started_at': None,
            'finished_at': None,
            'steps_truncated': False,
        }

    latest = snap.get('latest')
    if meta.get('kind') == 'settle' and isinstance(latest, dict):
        latest = slim_settle_step(latest)

    doc: dict[str, Any] = {
        'format': 'lstControl_run_log',
        'version': RUN_LOG_VERSION,
        'exported_at': _utc_iso(),
        'status': run_status(cs),
        'run': {
            **meta,
            **{k: snap.get(k) for k in (
                'start_pos', 'target_pos', 'pick_pos', 'place_pos',
                'planned_route', 'arrivals', 'scheduler_phase',
                'scheduler_phase_label', 'step_count', 'error',
                'stop_reason',
            )},
            'latest': latest,
            'events': events,
            'steps': steps,
        },
        'site_config': site_config_debug_subset(site),
    }
    if meta.get('kind') == 'settle':
        doc['run']['steps_schema'] = SETTLE_STEPS_SCHEMA
        doc['run']['settle_log_interval_s'] = SETTLE_LOG_INTERVAL_S
        doc['run']['settle_tail_log_interval_s'] = SETTLE_TAIL_LOG_INTERVAL_S
        if meta.get('anti_sway_mode') == 'deepseek_anti' and steps:
            doc['run']['deepseek_analysis'] = analyze_deepseek_settle_steps(
                steps, site=site,
            )
    if server is not None:
        doc['environment'] = {
            'plc_mode': getattr(server, 'plc', None) is not None,
            'hook_sway_tol_deg': getattr(server, 'hook_sway_angle_tol_deg', None),
        }
    if client_supplement:
        doc['client'] = client_supplement
    return doc


def default_log_dir(project_root: Path | None = None) -> Path:
    root = project_root or Path(__file__).resolve().parents[1]
    return root / 'logs'


def auto_save_run_log(
    cs,
    *,
    server: Any | None = None,
    site: dict[str, Any] | None = None,
    log_dir: Path | None = None,
) -> Path | None:
    """Write a finished run to logs/ (no-op if run never started or still active)."""
    if cs is None:
        return None
    with cs.lock:
        if not cs.run_kind or cs.finished_at_iso is None:
            return None
        if cs.running:
            return None
    try:
        document = build_run_log_document(cs, server=server, site=site)
        directory = log_dir or default_log_dir()
        path = save_run_log_file(document, directory)
        print(f'[run_log] Auto-saved → {path}')
        return path
    except OSError as exc:
        print(f'[run_log] Auto-save failed: {exc}')
        return None


def save_run_log_file(document: dict[str, Any], directory: Path) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    kind = (document.get('run') or {}).get('kind') or 'run'
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    path = directory / f'run_log_{kind}_{stamp}.json'
    path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2),
        encoding='utf-8',
    )
    return path


def suggested_download_name(document: dict[str, Any]) -> str:
    kind = (document.get('run') or {}).get('kind') or 'run'
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    return f'run_log_{kind}_{stamp}.json'
