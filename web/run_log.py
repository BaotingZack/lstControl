"""Build and persist PD / settle run logs for debugging."""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

MAX_RECORDED_STEPS = 15_000

RUN_LOG_VERSION = 1
SETTLE_STEPS_SCHEMA = 'settle_slim_v2'
SETTLE_LOG_INTERVAL_S = 1.0


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
    )
    out: dict[str, Any] = {}
    for key in keys:
        if key in site:
            out[key] = site[key]
    return out


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
    out = {
        't': step.get('t'),
        'roll': step.get('roll'),
        'pitch': step.get('pitch'),
        'theta_x': theta_x,
        'theta_y': theta_y,
        'theta_peak_deg': peak_deg,
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
        # 诊断字段：判断"某轴越消越大"到底是反号、增长保护、还是位移预算。
        out['rate_x_deg_s'] = debug.get('rate_x_deg_s')
        out['rate_y_deg_s'] = debug.get('rate_y_deg_s')
        out['guard_x'] = debug.get('guard_x')
        out['guard_y'] = debug.get('guard_y')
        out['budget_blocked'] = debug.get('budget_blocked')
    return out


def settle_step_worth_recording(
    step: dict[str, Any],
    *,
    last_log_t: float | None,
) -> bool:
    """Decimate high-rate PD loop (~10 Hz) to ~1 Hz for settle exports."""
    if last_log_t is None:
        return True
    t = float(step.get('t') or 0.0)
    return (t - last_log_t) >= SETTLE_LOG_INTERVAL_S


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
