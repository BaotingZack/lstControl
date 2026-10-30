"""Build and persist PD / settle run logs for debugging."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

MAX_RECORDED_STEPS = 15_000

RUN_LOG_VERSION = 1


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
        'anti_sway_crane_remote_kp',
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
                'stop_reason', 'latest',
            )},
            'events': events,
            'steps': steps,
        },
        'site_config': site_config_debug_subset(site),
    }
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
