"""Rebuild live_ptp_html.py with valid Python string literals."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import live_operation_html as op

bottom = op._PAGE_BODY[op._PAGE_BODY.index('<section class="bottom-panel">'):]
_PAGE_BODY = f"""<body>
  <main class="shell">
    <section class="stage">
      <canvas id="scene"></canvas>
      <div class="topbar">
        <h1>Crane Point-to-Point</h1>
        <div class="top-actions">
          <a class="calibration-link ptp-subnav" data-sub="sim" href="/ptp/sim">Sim / 仿真</a>
          <a class="calibration-link ptp-subnav" data-sub="live" href="/ptp/live">Live / 真机</a>
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
        <div class="command-title">Target Position / 目标位置</div>
        <div class="target-grid" id="targetGroup">
          <label>X <input id="targetX" type="number" step="0.01" autocomplete="off"></label>
          <label>Y <input id="targetY" type="number" step="0.01" autocomplete="off"></label>
          <label>Z <input id="targetZ" type="number" step="0.01" autocomplete="off"></label>
        </div>
        <div class="command-hint" id="commandMsg"></div>
        <button class="apply-target" id="applyTarget" type="button">Go to Target / 移动到目标</button>
      </section>
{bottom}"""

tail = op._BOOT_TAIL
idx = tail.index("function animate(ts)")
tail = tail[idx:]
tail = tail.replace("'Start Operation / 开始作业'", "APPLY_IDLE")
tail = tail.replace("'PLC Mode — Set Target'", "'PLC Mode — Enter Target'")
tail = tail.replace("'PLC Live — Set Target'", "'PLC Live — Enter Target'")
tail = tail.replace("'Operation complete'", "'Target reached'")
tail = tail.replace(
    """      els.startX.value = '';
      els.startY.value = '';
      els.startZ.value = '';
      """,
    "",
)
tail = tail.replace(
    """      els.locoStatus.style.color = '#5ebd72';

      // PLC mode""",
    """      els.locoStatus.style.color = '#5ebd72';

      // PLC mode""",
    1,
)
tail = tail.replace(
    """            if (s.pick_pos) {
              payload.pick = {x: s.pick_pos.x, y: s.pick_pos.y, z: s.pick_pos.z};
            }
""",
    "",
)
tail = tail.replace(
    """    function renderTargetControls() {
      els.targetX.value = payload.target.x.toFixed(2);
      els.targetY.value = payload.target.y.toFixed(2);
      els.targetZ.value = payload.target.z.toFixed(2);
    }
""",
    """    function renderTargetControls() {
      if (!payload || !payload.target || !els.targetX) return;
      els.targetX.value = payload.target.x.toFixed(2);
      els.targetY.value = payload.target.y.toFixed(2);
      els.targetZ.value = payload.target.z.toFixed(2);
    }
""",
)
tail = tail.replace(
    """    for (const input of [els.targetX, els.targetY, els.targetZ]) {
      input.addEventListener('keydown', event => {
        if (event.key === 'Enter') applyTargetCommand();
      });
    }
""",
    """    for (const input of [els.targetX, els.targetY, els.targetZ]) {
      if (!input) continue;
      input.addEventListener('keydown', event => {
        if (event.key === 'Enter') applyTargetCommand();
      });
    }
""",
)
tail = tail.replace(
    """// Initialize all position inputs empty
els.targetX.value = '';
      els.targetY.value = '';
      els.targetZ.value = '';
""",
    """// Initialize target inputs empty
      if (els.targetX) els.targetX.value = '';
      if (els.targetY) els.targetY.value = '';
      if (els.targetZ) els.targetZ.value = '';
""",
)
tail = tail.replace(
    """          if (!els.targetX.value) {
            els.targetX.value = d.p_ref_x.toFixed(2);
            els.targetY.value = d.p_ref_y.toFixed(2);
            els.targetZ.value = d.p_ref_z.toFixed(2);
          }
""",
    """          if (els.targetX && !els.targetX.value) {
            els.targetX.value = d.p_ref_x.toFixed(2);
            els.targetY.value = d.p_ref_y.toFixed(2);
            els.targetZ.value = d.p_ref_z.toFixed(2);
          }
""",
)
tail = tail.replace(
    """    // SSE stream for real-time localization data
    const locoSource = new EventSource('/localization/stream');""",
    """    // SSE stream for real-time localization data (PLC only — sim uses /simulation.json)
    if (PLC_MODE) {
    const locoSource = new EventSource('/localization/stream');""",
)
tail = tail.replace(
    """    locoSource.onerror = () => {
      els.locoStatus.textContent = 'Disconnected — waiting for data...';
      els.locoStatus.style.color = '#e05a47';
    };

    // PLC mode — poll /api/control-state at 10 Hz for real-time PD state""",
    """    locoSource.onerror = () => {
      els.locoStatus.textContent = 'Disconnected — waiting for data...';
      els.locoStatus.style.color = '#e05a47';
    };
    }

    // PLC mode — poll /api/control-state at 10 Hz for real-time PD state""",
)

_PTP_HEAD = """
    const APPLY_IDLE = 'Go to Target / 移动到目标';
    const canvas = document.getElementById('scene');
    const ctx = canvas.getContext('2d');
    let payload;
    let frame = 0;
    let lastAdvance = 0;
    let _controlActive = false;
    let _lastStepCount = -1;
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
      commandMsg: document.getElementById('commandMsg'),
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

    function showCommandMsg(text, color) {
      const c = color || '#e05a47';
      if (els.commandMsg) {
        els.commandMsg.textContent = text;
        els.commandMsg.style.color = c;
      }
      if (els.ctrlMsg) {
        els.ctrlMsg.textContent = text;
        els.ctrlMsg.style.color = c;
      }
    }

    function readTargetFields() {
      const tx = parseFloat(els.targetX.value);
      const ty = parseFloat(els.targetY.value);
      const tz = parseFloat(els.targetZ.value);
      if ([tx, ty, tz].some(function(n) { return isNaN(n); })) return null;
      return { x: tx, y: ty, z: tz };
    }

    function applyTargetCommand() {
      const parsed = readTargetFields();
      if (!parsed) {
        showCommandMsg('请填写目标 X、Y、Z 三个数字');
        return;
      }
      showCommandMsg('', '#5ebd72');
      if (PLC_MODE) {
        const command = { target_x: parsed.x, target_y: parsed.y, target_z: parsed.z };
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
                payload.place = null;
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
      params.set('target_x', String(parsed.x));
      params.set('target_y', String(parsed.y));
      params.set('target_z', String(parsed.z));
      window.location.search = params.toString();
    }

    (function initTargetFromQuery() {
      const params = new URLSearchParams(window.location.search);
      const tx = params.get('target_x');
      const ty = params.get('target_y');
      const tz = params.get('target_z');
      if (tx && ty && tz) {
        els.targetX.value = tx;
        els.targetY.value = ty;
        els.targetZ.value = tz;
      }
    })();

"""

_BOOT = _PTP_HEAD + tail

content = f'''"""Point-to-point (target-only) browser page."""

from live_view_shared import LIVE_CANVAS_JS, LIVE_VIEW_BASE_CSS, PTP_EXTRA_CSS

_PAGE_BODY = {repr(_PAGE_BODY)}

_BOOT = {repr(_BOOT)}


def render_ptp_html(plc_mode: bool = False) -> str:
    plc = "true" if plc_mode else "false"
    mode_label = "Live PLC / 真机" if plc_mode else "Simulation / 仿真"
    sub = "live" if plc_mode else "sim"
    page_body = _PAGE_BODY.replace(
        'data-sub="' + sub + '"',
        'data-sub="' + sub + '" aria-current="page"',
        1,
    )
    return (
        '<!doctype html>\\n'
        '<html lang="zh-CN">\\n'
        '<head>\\n'
        '  <meta charset="utf-8">\\n'
        '  <meta name="viewport" content="width=device-width, initial-scale=1">\\n'
        f'  <title>Crane Point-to-Point — {{mode_label}}</title>\\n'
        f'  <style>{{LIVE_VIEW_BASE_CSS}}{{PTP_EXTRA_CSS}}</style>\\n'
        '</head>\\n'
        + page_body
        + '  <script>\\n'
        + f'    const PLC_MODE = {{plc}};\\n'
        + _BOOT
        + LIVE_CANVAS_JS
        + '  </script>\\n'
        '</body>\\n'
        '</html>'
    )
'''
(ROOT / "live_ptp_html.py").write_text(content, encoding="utf-8")
print("rebuilt live_ptp_html.py")
