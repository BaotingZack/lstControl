#!/usr/bin/env python3
"""Analyze auto-saved settle run logs (deepseek-anti).

Usage:
  python3 tools/analyze_deepseek_settle.py              # newest logs/settle_*.json
  python3 tools/analyze_deepseek_settle.py path/to.json
  python3 tools/analyze_deepseek_settle.py --latest 3   # last 3 settle logs
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from web.run_log import analyze_deepseek_settle_steps, default_log_dir


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def _print_report(path: Path, doc: dict) -> None:
    run = doc.get('run') or {}
    mode = run.get('anti_sway_mode')
    steps = run.get('steps') or []
    pre = run.get('deepseek_analysis')
    if isinstance(pre, dict) and pre.get('ok'):
        analysis = pre
    else:
        analysis = analyze_deepseek_settle_steps(
            steps, site=doc.get('site_config'),
        )
    print('=' * 72)
    print(path.name)
    print(f"  status={doc.get('status')} mode={mode} sim={run.get('sim')} steps={len(steps)}")
    print(f"  schema={run.get('steps_schema')} exported={doc.get('exported_at')}")
    if not analysis.get('ok'):
        print(f"  skip: {analysis.get('reason')}")
        return
    print(f"  duration={analysis['duration_s']:.1f}s  t_ready={analysis.get('t_ready_s')}")
    print(f"  rope_m≈{analysis.get('rope_m_mean')}  lead_eff≈{analysis.get('lead_s_median')}")
    print(
        f"  peak θ_init={analysis.get('peak_theta_init_deg', 0):.2f}°"
        f"  |θ_ac|_init={analysis.get('peak_ac_init_deg', 0):.2f}°"
    )
    print(
        f"  tail max θ={analysis.get('tail_theta_max_deg')}°"
        f"  |θ_ac|={analysis.get('tail_ac_max_deg')}°"
        f"  stall={analysis.get('tail_stall_frac'):.0%}"
    )
    for line in analysis.get('hints') or []:
        print(f"  → {line}")
    if not analysis.get('hints'):
        print('  → 未发现明显异常；可与历史同绳长日志对比 tail_ac_max。')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('paths', nargs='*', type=Path, help='run log JSON files')
    parser.add_argument('--latest', type=int, default=1, help='多少条最新 settle 日志')
    parser.add_argument('--log-dir', type=Path, default=None)
    args = parser.parse_args()

    paths: list[Path] = list(args.paths)
    if not paths:
        log_dir = args.log_dir or default_log_dir(ROOT)
        settle_logs = sorted(
            log_dir.glob('run_log_settle_*.json'),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        paths = settle_logs[: max(1, args.latest)]

    if not paths:
        print(f'No settle logs under {args.log_dir or default_log_dir(ROOT)}', file=sys.stderr)
        return 1

    for path in paths:
        if not path.is_file():
            print(f'missing: {path}', file=sys.stderr)
            continue
        _print_report(path, _load(path))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())