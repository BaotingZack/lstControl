"""One-off helper to split live HTML (run from repo root)."""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
text = (ROOT / "live_server.py").read_text(encoding="utf-8")
m = re.search(r'html = """(.+?)"""\n    if ui_mode ==', text, re.DOTALL)
if not m:
    raise SystemExit("html block not found")
html = m.group(1)
css = html[html.index("  <style>") + 9 : html.index("  </style>")]
start = html.index("    function resize()")
end = html.index("    function applyTargetCommand()")
canvas_js = html[start:end]

shared = ROOT / "live_view_shared.py"
shared.write_text(
    '"""Shared browser assets for crane live views (canvas + layout CSS)."""\n\n'
    "LIVE_VIEW_BASE_CSS = "
    + repr(css)
    + "\n\n"
    "LIVE_CANVAS_JS = "
    + repr(canvas_js)
    + "\n\n"
    "PTP_EXTRA_CSS = "
    + repr(
        "\n    .readonly-grid input { opacity: 0.85; cursor: default; }\n"
    )
    + "\n",
    encoding="utf-8",
)
print(f"Wrote {shared} ({len(canvas_js)} canvas chars)")
