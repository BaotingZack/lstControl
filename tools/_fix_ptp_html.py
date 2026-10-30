"""Rebuild live_ptp_html.py from live_operation_html template."""
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
import sys

sys.path.insert(0, str(ROOT))
import live_operation_html as op
bottom = op._PAGE_BODY[op._PAGE_BODY.index('<section class="bottom-panel">'):]
ptp_body = f"""<body>
  <main class="shell">
    <section class="stage">
      <canvas id="scene"></canvas>
      <div class="topbar">
        <h1>Crane Point-to-Point</h1>
        <div class="top-actions">
          <a class="calibration-link" href="/">Full Operation / 完整作业</a>
          <a class="calibration-link" href="/calibration">Coordinate Calibration</a>
          <div class="badge" id="rate">10 Hz</div>
        </div>
      </div>
    </section>
    <aside class="side">
      <section class="status">
        <div class="metric"><div class="label">Time</div><div class="value" id="time">0.0s</div></div>
        <div class="metric"><div class="label">Frame</div><div class="value" id="frame">0</div></div>
        <div class="metric"><div class="label">Speed</div><div class="value" id="speed">1.0x</div></div>
      </section>
      <section class="readout">
        <div class="phase" id="phase">Loading</div>
        <div class="coords">
          <div class="coord"><span>X Bridge</span><strong id="x">0.00</strong></div>
          <div class="coord"><span>Y Trolley</span><strong id="y">0.00</strong></div>
          <div class="coord"><span>Z Hoist</span><strong id="z">0.00</strong></div>
        </div>
        <div class="velo-label">Velocity</div>
        <div class="coords">
          <div class="coord" id="coordVx"><span>Vx Bridge</span><strong id="vx" style="color:#5ebd72">0.00</strong></div>
          <div class="coord" id="coordVy"><span>Vy Trolley</span><strong id="vy" style="color:#5ebd72">0.00</strong></div>
          <div class="coord" id="coordVz"><span>Vz Hoist</span><strong id="vz" style="color:#5ebd72">0.00</strong></div>
        </div>
      </section>
      <section class="command">
        <div class="command-title">Current Position / 当前位置 (定位)</div>
        <div class="target-grid readonly-grid" style="margin-bottom:12px;">
          <label>X <input id="currX" type="text" readonly tabindex="-1" placeholder="—"></label>
          <label>Y <input id="currY" type="text" readonly tabindex="-1" placeholder="—"></label>
          <label>Z <input id="currZ" type="text" readonly tabindex="-1" placeholder="—"></label>
        </div>
        <div class="command-title">Target Position / 目标位置</div>
        <div class="target-grid" id="targetGroup">
          <label>X <input id="targetX" type="number" step="0.01" autocomplete="off"></label>
          <label>Y <input id="targetY" type="number" step="0.01" autocomplete="off"></label>
          <label>Z <input id="targetZ" type="number" step="0.01" autocomplete="off"></label>
        </div>
        <button class="apply-target" id="applyTarget" type="button">Go to Target / 移动到目标</button>
      </section>
{bottom}"""

text = (ROOT / "live_ptp_html.py").read_text(encoding="utf-8")
boot_start = text.index("_BOOT = ")
boot_end = text.index("\n\ndef render_ptp_html")
boot = text[boot_start + 8 : boot_end]
boot = boot.replace(
    "UI_MODE === 'ptp'\n        ? 'PLC Mode — Enter Target'\n        : 'PLC Mode — Enter Target'",
    "'PLC Mode — Enter Target'",
)
boot = boot.replace(
    "      els.startX.value = '';\n      els.startY.value = '';\n      els.startZ.value = '';\n      ",
    "",
)
boot = boot.replace("if (UI_MODE === 'ptp' && els.currX)", "if (els.currX)")
boot = boot.replace(
    "UI_MODE === 'ptp'\n          ? 'PLC Live — Enter Target'\n          : 'PLC Live — Enter Target'",
    "'PLC Live — Enter Target'",
)
boot = boot.replace(
    "els.ctrlMsg.textContent = UI_MODE === 'ptp'\n                ? 'Target reached'\n                : 'Target reached';",
    "els.ctrlMsg.textContent = 'Target reached';",
)
boot = boot.replace(
    """            if (payload.pick) {
              allX.push(payload.pick.x); allY.push(payload.pick.y); allZ.push(payload.pick.z);
            }""",
    "",
)

out = '''"""Point-to-point (target-only) browser page."""

from live_view_shared import LIVE_CANVAS_JS, LIVE_VIEW_BASE_CSS, PTP_EXTRA_CSS

_PAGE_BODY = '''
out += repr(ptp_body)
out += "\n\n_BOOT = "
out += repr(boot)
out += '''

def render_ptp_html(plc_mode: bool = False) -> str:
    plc = "true" if plc_mode else "false"
    return (
        '<!doctype html>\\n'
        '<html lang="zh-CN">\\n'
        '<head>\\n'
        '  <meta charset="utf-8">\\n'
        '  <meta name="viewport" content="width=device-width, initial-scale=1">\\n'
        '  <title>Crane Point-to-Point</title>\\n'
        f'  <style>{LIVE_VIEW_BASE_CSS}{PTP_EXTRA_CSS}</style>\\n'
        '</head>\\n'
        + _PAGE_BODY
        + '  <script>\\n'
        + f'    const PLC_MODE = {plc};\\n'
        + _BOOT
        + LIVE_CANVAS_JS
        + '  </script>\\n'
        '</body>\\n'
        '</html>'
    )
'''
(ROOT / "live_ptp_html.py").write_text(out, encoding="utf-8")
print("Wrote live_ptp_html.py")
