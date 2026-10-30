"""Split operation vs PTP live HTML out of live_server.py (run once from repo root)."""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _scrub_operation_js(tail: str) -> str:
    """Remove ptp branches from shared tail; keep operation behavior."""
    tail = tail.replace("applyIdleLabel()", "'Start Operation / 开始作业'")
    tail = tail.replace("applyRunningLabel()", "'Running...'")
    # applyTargetCommand: keep only else branch for PLC
    tail = re.sub(
        r"    function applyTargetCommand\(\) \{.*?// Simulation replay mode",
        _operation_apply_target(),
        tail,
        count=1,
        flags=re.DOTALL,
    )
    tail = tail.replace("UI_MODE === 'ptp'\n        ? 'PLC Mode — Enter Target'\n        : ", "")
    tail = tail.replace("'PLC Mode — Enter Target'", "'PLC Mode — Set Target'")
    tail = re.sub(
        r"\n      if \(UI_MODE === 'ptp' && els\.currX\) \{[^}]+\}\n",
        "\n",
        tail,
        flags=re.DOTALL,
    )
    tail = tail.replace(
        "UI_MODE === 'ptp'\n          ? 'PLC Live — Enter Target'\n          : ",
        "",
    )
    tail = tail.replace("'PLC Live — Enter Target'", "'PLC Live — Set Target'")
    tail = re.sub(
        r"els\.ctrlMsg\.textContent = UI_MODE === 'ptp'\s*\? 'Target reached'\s*: 'Operation complete';",
        "els.ctrlMsg.textContent = 'Operation complete';",
        tail,
    )
    return tail


def _operation_apply_target() -> str:
    return r"""    function applyTargetCommand() {
      if (PLC_MODE) {
        var sx = parseFloat(els.startX.value);
        var sy = parseFloat(els.startY.value);
        var sz = parseFloat(els.startZ.value);
        var tx = parseFloat(els.targetX.value);
        var ty = parseFloat(els.targetY.value);
        var tz = parseFloat(els.targetZ.value);
        const command = {
          start_x: sx, start_y: sy, start_z: sz,
          target_x: tx, target_y: ty, target_z: tz,
        };
        for (const key of ['start_x','start_y','start_z','target_x','target_y','target_z']) {
          if (isNaN(command[key])) {
            els.ctrlMsg.textContent = 'Error: ' + key + ' must be a number';
            els.ctrlMsg.style.color = '#e05a47';
            return;
          }
        }
        els.applyTarget.disabled = true;
        els.applyTarget.textContent = 'Running...';
        els.ctrlMsg.textContent = '';
        fetch('/api/start-control', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(command),
        })
          .then(r => r.json())
          .then(data => {
            if (data.ok) {
              els.ctrlMsg.textContent = 'Operation started — ' + (data.phase_label || 'running...');
              els.ctrlMsg.style.color = '#5ebd72';
              _lastStepCount = -1;
              _trajectoryFrozen = false;
              _lastTrailTime = -1;
              _segBoundariesRecorded = 0;
              if (!payload) {
                payload = {
                  updateHz: 10, speed: 1, framePeriodMs: 100,
                  bounds: {xMin: -2, xMax: 10, yMin: -2, yMax: 8, zMin: 0, zMax: 7},
                  velocityLimits: {xy: 0.2, z: 0.2},
                  phaseBoundaries: [],
                  frames: [],
                };
              }
              payload.frames.length = 0;
              payload.segmentIndices = [];
              if (data.planned_route) payload.plannedRoute = data.planned_route;
              if (data.start_pos) payload.initial = data.start_pos;
              if (data.pick_pos) payload.pick = data.pick_pos;
              if (data.place_pos) {
                payload.place = data.place_pos;
                payload.target = data.place_pos;
              }
              if (payload.pick && payload.place) {
                var ix = payload.initial ? payload.initial.x : payload.pick.x;
                var iy = payload.initial ? payload.initial.y : payload.pick.y;
                var iz = payload.initial ? payload.initial.z : payload.pick.z;
                var tx2 = payload.place.x, ty2 = payload.place.y, tz2 = payload.place.z;
                var px = payload.pick.x, py = payload.pick.y, pz = payload.pick.z;
                var allX = [ix, tx2, px], allY = [iy, ty2, py], allZ = [iz, tz2, pz];
                var padX = Math.max(1.0, (Math.max.apply(null, allX) - Math.min.apply(null, allX)) * 0.2);
                var padY = Math.max(1.0, (Math.max.apply(null, allY) - Math.min.apply(null, allY)) * 0.2);
                var padZ = Math.max(1.0, (Math.max.apply(null, allZ) - Math.min.apply(null, allZ)) * 0.2);
                payload.bounds = {
                  xMin: Math.min.apply(null, allX) - padX, xMax: Math.max.apply(null, allX) + padX,
                  yMin: Math.min.apply(null, allY) - padY, yMax: Math.max.apply(null, allY) + padY,
                  zMin: Math.min.apply(null, allZ) - padZ, zMax: Math.max.apply(null, allZ) + padZ,
                };
              }
              draw();
            } else {
              els.ctrlMsg.textContent = 'Error: ' + (data.error || 'unknown');
              els.ctrlMsg.style.color = '#e05a47';
              els.applyTarget.disabled = false;
              els.applyTarget.textContent = 'Start Operation / 开始作业';
            }
          })
          .catch(err => {
            els.ctrlMsg.textContent = 'Failed: ' + err.message;
            els.ctrlMsg.style.color = '#e05a47';
            els.applyTarget.disabled = false;
            els.applyTarget.textContent = 'Start Operation / 开始作业';
          });
        return;
      }
      // Simulation replay mode"""


def _ptp_shell_html(full_html: str) -> str:
    """Body markup for PTP page (stage + side panels)."""
    # Reuse from full_html between <body and command section, patch headings
    start = full_html.index("<body")
    cmd = full_html.index('<section class="command">')
    before_cmd = full_html[start:cmd]
    before_cmd = before_cmd.replace("__BODY_CLASS__", "")
    before_cmd = before_cmd.replace('class=""', "")
    before_cmd = before_cmd.replace("__PAGE_HEADING__", "Crane Point-to-Point")
    before_cmd = before_cmd.replace(
        "__MODE_NAV__",
        '<a class="calibration-link" href="/">Full Operation / 完整作业</a>',
    )
    before_cmd = before_cmd.replace("<body class=\"\">", "<body>")
    ptp_cmd = """
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
"""
    after_cmd = full_html[full_html.index('<section class="bottom-panel">'):]
    return before_cmd + ptp_cmd + after_cmd


def _ptp_boot_js(op_tail: str) -> str:
    """PTP-specific client boot script (no job points)."""
    head = """
    const APPLY_IDLE = 'Go to Target / 移动到目标';
    const canvas = document.getElementById('scene');
    const ctx = canvas.getContext('2d');
    let payload;
    let frame = 0;
    let lastAdvance = 0;
    let _liveControlState = null;
    let _controlActive = false;
    let _lastStepCount = -1;
    let _lastTrailTime = -1;
    let _trajectoryFrozen = false;
    let _segBoundariesRecorded = 0;

    const els = {
      rate: document.getElementById('rate'),
      time: document.getElementById('time'),
      frame: document.getElementById('frame'),
      speed: document.getElementById('speed'),
      phase: document.getElementById('phase'),
      x: document.getElementById('x'),
      y: document.getElementById('y'),
      z: document.getElementById('z'),
      vx: document.getElementById('vx'),
      vy: document.getElementById('vy'),
      vz: document.getElementById('vz'),
      coordVx: document.getElementById('coordVx'),
      coordVy: document.getElementById('coordVy'),
      coordVz: document.getElementById('coordVz'),
      targetX: document.getElementById('targetX'),
      targetY: document.getElementById('targetY'),
      targetZ: document.getElementById('targetZ'),
      currX: document.getElementById('currX'),
      currY: document.getElementById('currY'),
      currZ: document.getElementById('currZ'),
      applyTarget: document.getElementById('applyTarget'),
      progress: document.getElementById('progress'),
      startTime: document.getElementById('startTime'),
      endTime: document.getElementById('endTime'),
      phaseLog: document.getElementById('phaseLog'),
      locoStatus: document.getElementById('locoStatus'),
      locoX: document.getElementById('locoX'),
      locoY: document.getElementById('locoY'),
      locoZ: document.getElementById('locoZ'),
      locoVx: document.getElementById('locoVx'),
      locoVy: document.getElementById('locoVy'),
      locoVz: document.getElementById('locoVz'),
      locoTimestamp: document.getElementById('locoTimestamp'),
      ctrlVx: document.getElementById('ctrlVx'),
      ctrlVy: document.getElementById('ctrlVy'),
      ctrlHz: document.getElementById('ctrlHz'),
      ctrlGrip: document.getElementById('ctrlGrip'),
      ctrlMsg: document.getElementById('ctrlMsg'),
      plcDot: document.getElementById('plcDot'),
      hbDot: document.getElementById('hbDot'),
      plcStatusText: document.getElementById('plcStatusText'),
      hbStatusText: document.getElementById('hbStatusText'),
      btnStop: document.getElementById('btnStop'),
      btnReset: document.getElementById('btnReset'),
    };
"""
    apply = _ptp_apply_target()
    tail = _scrub_ptp_js(op_tail)
    return head + apply + tail


def _ptp_apply_target() -> str:
    return """
    function applyTargetCommand() {
      if (PLC_MODE) {
        var tx = parseFloat(els.targetX.value);
        var ty = parseFloat(els.targetY.value);
        var tz = parseFloat(els.targetZ.value);
        const command = { target_x: tx, target_y: ty, target_z: tz };
        for (const key of ['target_x', 'target_y', 'target_z']) {
          if (isNaN(command[key])) {
            els.ctrlMsg.textContent = 'Error: ' + key + ' must be a number';
            els.ctrlMsg.style.color = '#e05a47';
            return;
          }
        }
        els.applyTarget.disabled = true;
        els.applyTarget.textContent = 'Moving...';
        els.ctrlMsg.textContent = '';
        fetch('/api/start-goto', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(command),
        })
          .then(r => r.json())
          .then(data => {
            if (data.ok) {
              els.ctrlMsg.textContent = 'Point-to-point move started — ' + (data.phase_label || 'running...');
              els.ctrlMsg.style.color = '#5ebd72';
              _lastStepCount = -1;
              _trajectoryFrozen = false;
              _lastTrailTime = -1;
              _segBoundariesRecorded = 0;
              if (!payload) {
                payload = {
                  updateHz: 10, speed: 1, framePeriodMs: 100,
                  bounds: {xMin: -2, xMax: 10, yMin: -2, yMax: 8, zMin: 0, zMax: 7},
                  velocityLimits: {xy: 0.2, z: 0.2},
                  phaseBoundaries: [],
                  frames: [],
                };
              }
              payload.frames.length = 0;
              payload.segmentIndices = [];
              payload.pick = null;
              if (data.planned_route) payload.plannedRoute = data.planned_route;
              if (data.start_pos) payload.initial = data.start_pos;
              if (data.target_pos) {
                payload.target = data.target_pos;
                payload.place = data.target_pos;
              }
              if (payload.initial && payload.target) {
                var ix = payload.initial.x, iy = payload.initial.y, iz = payload.initial.z;
                var tx3 = payload.target.x, ty3 = payload.target.y, tz3 = payload.target.z;
                var padX = Math.max(1.0, Math.abs(tx3 - ix) * 0.3 + 1);
                var padY = Math.max(1.0, Math.abs(ty3 - iy) * 0.3 + 1);
                var padZ = Math.max(1.0, Math.abs(tz3 - iz) * 0.3 + 1);
                payload.bounds = {
                  xMin: Math.min(ix, tx3) - padX, xMax: Math.max(ix, tx3) + padX,
                  yMin: Math.min(iy, ty3) - padY, yMax: Math.max(iy, ty3) + padY,
                  zMin: Math.min(iz, tz3) - padZ, zMax: Math.max(iz, tz3) + padZ,
                };
              }
              draw();
            } else {
              els.ctrlMsg.textContent = 'Error: ' + (data.error || 'unknown');
              els.ctrlMsg.style.color = '#e05a47';
              els.applyTarget.disabled = false;
              els.applyTarget.textContent = APPLY_IDLE;
            }
          })
          .catch(err => {
            els.ctrlMsg.textContent = 'Failed: ' + err.message;
            els.ctrlMsg.style.color = '#e05a47';
            els.applyTarget.disabled = false;
            els.applyTarget.textContent = APPLY_IDLE;
          });
        return;
      }
      const params = new URLSearchParams(window.location.search);
      params.set('target_x', els.targetX.value);
      params.set('target_y', els.targetY.value);
      params.set('target_z', els.targetZ.value);
      window.location.search = params.toString();
    }

"""


def _scrub_ptp_js(tail: str) -> str:
    tail = tail.replace("applyIdleLabel()", "APPLY_IDLE")
    tail = tail.replace("'Start Operation / 开始作业'", "APPLY_IDLE")
    # Remove duplicate const canvas block from tail (starts after applyTarget)
    idx = tail.find("    els.applyTarget.addEventListener")
    if idx == -1:
        return tail
    # drop re-declaration of PLC_MODE if present in tail - keep from original
    tail = re.sub(
        r"const PLC_MODE = __PLC_MODE__;\n",
        "",
        tail,
        count=1,
    )
    tail = tail.replace("'PLC Mode — Set Target'", "'PLC Mode — Enter Target'")
    tail = tail.replace("'PLC Live — Set Target'", "'PLC Live — Enter Target'")
    tail = tail.replace("'Operation complete'", "'Target reached'")
    # curr position update in loco handler
    insert = """
      if (els.currX) {
        els.currX.value = data.x.toFixed(2);
        els.currY.value = data.y.toFixed(2);
        els.currZ.value = data.z.toFixed(2);
      }
"""
    tail = tail.replace(
        "      els.locoStatus.style.color = '#5ebd72';\n\n      // PLC mode",
        "      els.locoStatus.style.color = '#5ebd72';\n" + insert + "\n      // PLC mode",
        1,
    )
    # pollControlState first-step: skip pick for ptp
    tail = tail.replace(
        "            if (s.pick_pos) {\n              payload.pick = {x: s.pick_pos.x, y: s.pick_pos.y, z: s.pick_pos.z};\n            }\n",
        "",
    )
    return tail[idx:]


def _op_body_html(full_html: str) -> str:
    start = full_html.index("<body")
    end = full_html.index("  <script>")
    body = full_html[start:end]
    body = body.replace("__BODY_CLASS__", "")
    body = body.replace('class=""', "")
    body = body.replace("__PAGE_HEADING__", "Crane Live Control")
    body = body.replace(
        "__MODE_NAV__",
        '<a class="calibration-link" href="/goto">Point-to-Point / 点对点</a>',
    )
    body = body.replace("<body class=\"\">", "<body>")
    # Remove ptp-only sections from operation body
    body = re.sub(
        r'        <div class="command-title ui-ptp-only">.*?</div>\n        <div class="target-grid readonly-grid ui-ptp-only".*?</div>\n',
        "",
        body,
        flags=re.DOTALL,
    )
    body = body.replace(" ui-operation-only", "")
    body = body.replace('        <div class="command-title ui-ptp-only">Target Position / 目标位置</div>\n', "")
    body = body.replace("__APPLY_IDLE_LABEL__", "Start Operation / 开始作业")
    return body


def _shared_py(css: str, canvas_js: str) -> str:
    return (
        '"""Shared browser assets for crane live views (canvas + layout CSS)."""\n\n'
        f"LIVE_VIEW_BASE_CSS = {css!r}\n\n"
        f"LIVE_CANVAS_JS = {canvas_js!r}\n\n"
        "PTP_EXTRA_CSS = '\\n    .readonly-grid input { opacity: 0.85; cursor: default; }\\n'\n"
    )


def _operation_py(body: str, boot_head: str, boot_tail: str) -> str:
    boot_head = boot_head.replace("const PLC_MODE = __PLC_MODE__;\n", "")
    boot_head = re.sub(
        r"    function applyIdleLabel\(\) \{[^}]+\}\n    function applyRunningLabel\(\) \{[^}]+\}\n",
        "",
        boot_head,
        flags=re.DOTALL,
    )
    boot_head = boot_head.replace(
        "      currX: document.getElementById('currX'),\n"
        "      currY: document.getElementById('currY'),\n"
        "      currZ: document.getElementById('currZ'),\n",
        "",
    )
    return (
        '"""Full operation (pick/place scheduler) browser page."""\n\n'
        "from live_view_shared import LIVE_CANVAS_JS, LIVE_VIEW_BASE_CSS\n\n"
        f"_PAGE_BODY = {body!r}\n\n"
        f"_BOOT_HEAD = {boot_head!r}\n\n"
        f"_BOOT_TAIL = {boot_tail!r}\n\n"
        "def render_live_html(plc_mode: bool = False) -> str:\n"
        '    plc = "true" if plc_mode else "false"\n'
        "    return (\n"
        '        \'<!doctype html>\\n\'\n'
        '        \'<html lang="en">\\n\'\n'
        '        \'<head>\\n\'\n'
        '        \'  <meta charset="utf-8">\\n\'\n'
        '        \'  <meta name="viewport" content="width=device-width, initial-scale=1">\\n\'\n'
        '        \'  <title>Crane Live Control</title>\\n\'\n'
        '        f\'  <style>{LIVE_VIEW_BASE_CSS}</style>\\n\'\n'
        '        \'</head>\\n\'\n'
        "        + _PAGE_BODY\n"
        '        + \'  <script>\\n\'\n'
        '        + f\'    const PLC_MODE = {plc};\\n\'\n'
        "        + _BOOT_HEAD\n"
        "        + LIVE_CANVAS_JS\n"
        "        + _BOOT_TAIL\n"
        '        + \'  </script>\\n\'\n'
        '        \'</body>\\n\'\n'
        '        \'</html>\'\n'
        "    )\n"
    )


def _ptp_py(body: str, boot: str) -> str:
    return (
        '"""Point-to-point (target-only) browser page."""\n\n'
        "from live_view_shared import LIVE_CANVAS_JS, LIVE_VIEW_BASE_CSS, PTP_EXTRA_CSS\n\n"
        f"_PAGE_BODY = {body!r}\n\n"
        f"_BOOT = {boot!r}\n\n"
        "def render_ptp_html(plc_mode: bool = False) -> str:\n"
        '    plc = "true" if plc_mode else "false"\n'
        "    return (\n"
        '        \'<!doctype html>\\n\'\n'
        '        \'<html lang="zh-CN">\\n\'\n'
        '        \'<head>\\n\'\n'
        '        \'  <meta charset="utf-8">\\n\'\n'
        '        \'  <meta name="viewport" content="width=device-width, initial-scale=1">\\n\'\n'
        '        \'  <title>Crane Point-to-Point</title>\\n\'\n'
        '        f\'  <style>{LIVE_VIEW_BASE_CSS}{PTP_EXTRA_CSS}</style>\\n\'\n'
        '        \'</head>\\n\'\n'
        "        + _PAGE_BODY\n"
        '        + \'  <script>\\n\'\n'
        '        + f\'    const PLC_MODE = {plc};\\n\'\n'
        "        + _BOOT\n"
        "        + LIVE_CANVAS_JS\n"
        '        + \'  </script>\\n\'\n'
        '        \'</body>\\n\'\n'
        '        \'</html>\'\n'
        "    )\n"
    )


def main() -> None:
    text = (ROOT / "live_server.py").read_text(encoding="utf-8")
    m = re.search(
        r"def render_ptp_html.*?def render_live_html\(plc_mode: bool = False, ui_mode: str = 'operation'\).*?"
        r'html = """(.+?)"""\n    if ui_mode ==',
        text,
        re.DOTALL,
    )
    if not m:
        raise SystemExit("Could not locate render_live_html template")
    html = m.group(1)

    css = html[html.index("  <style>") + 9 : html.index("  </style>")]
    for chunk in (
        "    body.ui-ptp .ui-operation-only { display: none !important; }\n",
        "    .ui-ptp-only { display: none; }\n",
        "    body.ui-ptp .ui-ptp-only { display: block; }\n",
        "    .readonly-grid input { opacity: 0.85; cursor: default; }\n",
    ):
        css = css.replace(chunk, "")

    script_start = html.index("  <script>")
    script_end = html.index("  </script>", script_start)
    script = html[script_start + 10 : script_end]

    canvas_start = script.index("    function resize()")
    canvas_end = script.index("    function applyTargetCommand()")
    canvas_js = script[canvas_start:canvas_end]

    op_boot = script[:canvas_start]
    op_boot = op_boot.replace("    const UI_MODE = '__UI_MODE__';\n", "")
    op_boot = re.sub(
        r"    function applyIdleLabel\(\) \{[^}]+\}\n    function applyRunningLabel\(\) \{[^}]+\}\n",
        "",
        op_boot,
        flags=re.DOTALL,
    )
    op_tail = script[canvas_end:]
    op_tail_scrubbed = _scrub_operation_js(op_tail)

    ptp_body = _ptp_shell_html(html)
    ptp_boot = _ptp_boot_js(op_tail)

    (ROOT / "live_view_shared.py").write_text(
        _shared_py(css, canvas_js),
        encoding="utf-8",
    )
    (ROOT / "live_operation_html.py").write_text(
        _operation_py(_op_body_html(html), op_boot, op_tail_scrubbed),
        encoding="utf-8",
    )
    (ROOT / "live_ptp_html.py").write_text(
        _ptp_py(ptp_body, ptp_boot),
        encoding="utf-8",
    )
    print("Wrote live_view_shared.py, live_operation_html.py, live_ptp_html.py")


if __name__ == "__main__":
    main()
