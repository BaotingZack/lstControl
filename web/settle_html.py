"""Manual push + active damping test pages (settle sim / settle live)."""

from web.platform_nav import platform_nav_markup
from web.ptp_html import anti_sway_select_markup
from web.ptp_sway_canvas import PTP_SWAY_STAGE_JS
from web.run_log_ui import (
    RUN_LOG_EXTRA_CSS,
    RUN_LOG_SAVE_JS,
    SETTLE_RUN_LOG_INIT,
)
from web.view_shared import LIVE_VIEW_BASE_CSS, PTP_EXTRA_CSS

_SETTLE_PAGE_BODY = """<body>
  <main class="shell settle-focus">
    <section class="stage">
      <canvas id="scene"></canvas>
      <div class="topbar">
        <h1>消摆测试</h1>
        <div class="top-actions platform-nav">
          __PLATFORM_NAV__
          <div class="badge" id="rate">—</div>
        </div>
      </div>
    </section>
    <aside class="side">
      <section class="status">
        <div class="metric"><div class="label">Mode</div><div class="value" id="modeLabel">—</div></div>
        <div class="metric"><div class="label">Session</div><div class="value" id="sessionState">Idle</div></div>
        <div class="metric"><div class="label">Algorithm</div><div class="value" id="algoLabel">—</div></div>
      </section>
      <section class="readout">
        <div class="phase" id="phase">Loading</div>
        <p class="settle-intro" id="settleIntro">
          <strong>流程：</strong>大车/小车保持不动，人工推动抓钩晃起来；观察摆角衰减。
          选择消摆算法后点击「开始消摆」（CraneRemote / 消摆基准）。
        </p>
        <span class="settle-status-pill" id="backendPill">消摆基准算法已接入（/api/start-settle）</span>
      </section>
      <section class="command">
        <div class="command-title">Anti-sway / 消摆算法</div>
        __ANTI_SWAY_SELECT__
        <div class="settle-actions">
          <button class="btn-settle primary" id="btnStartSettle" type="button">开始消摆 / Start damping</button>
          <button class="btn-settle" id="btnStopSettle" type="button" disabled>停止消摆 / Stop</button>
        </div>
        <button class="btn-save-log" id="btnSaveRunLog" type="button" disabled>保存运行日志 / Save run log</button>
        <div class="command-hint" id="commandMsg"></div>
      </section>
      <section class="bottom-panel">
        <nav class="tab-bar">
          <button class="tab active" data-tab="inclinometer">Inclinometer</button>
          <button class="tab" data-tab="control">Control</button>
        </nav>
        <div class="tab-panel active" id="tab-inclinometer">
          <div class="loco-status" id="swayStatus">Waiting for sway data...</div>
          <div class="loco-table">
            <div class="loco-hdr"></div>
            <div class="loco-hdr">Angle (deg)</div>
            <div class="loco-hdr">Raw inc.</div>
            <div class="loco-axis">θx</div>
            <div class="loco-val" id="thetaX">--</div>
            <div class="loco-val" id="rawPitch">--</div>
            <div class="loco-axis">θy</div>
            <div class="loco-val" id="thetaY">--</div>
            <div class="loco-val" id="rawRoll">--</div>
            <div class="loco-axis">Peak</div>
            <div class="loco-val" id="thetaPeak">--</div>
            <div class="loco-val" id="swayAge">--</div>
          </div>
        </div>
        <div class="tab-panel" id="tab-control">
          <div class="plc-status-bar">
            <span class="plc-dot" id="plcDot"></span>
            <span id="plcStatusText">PLC</span>
            <span class="plc-sep"></span>
            <span class="plc-dot" id="hbDot"></span>
            <span id="hbStatusText">Heartbeat</span>
          </div>
          <div class="ctrl-grid-2x4">
            <div class="ctrl-hdr">X Bridge</div>
            <div class="ctrl-hdr">Y Trolley</div>
            <div class="ctrl-hdr">Z Hoist</div>
            <div class="ctrl-hdr">Gripper</div>
            <div class="ctrl-val" id="ctrlVx">--</div>
            <div class="ctrl-val" id="ctrlVy">--</div>
            <div class="ctrl-val" id="ctrlHz">--</div>
            <div class="ctrl-val gripper" id="ctrlGrip">--</div>
          </div>
          <div class="ctrl-action-msg" id="ctrlMsg"></div>
          <div class="btn-row">
            <button class="btn-stop" id="btnStop">STOP ALL</button>
            <button class="btn-reset" id="btnReset">Reset Control</button>
          </div>
        </div>
      </section>
    </aside>
  </main>
"""

_SETTLE_CANVAS_HELPERS = """
    function drawPanel(box, title) {
      ctx.fillStyle = '#121c25';
      ctx.fillRect(box.x, box.y, box.w, box.h);
      ctx.strokeStyle = '#3b4e60';
      ctx.lineWidth = 1;
      ctx.strokeRect(box.x, box.y, box.w, box.h);
      ctx.fillStyle = '#dce7ee';
      ctx.font = '700 14px system-ui, sans-serif';
      ctx.fillText(title, box.x, box.y - 14);
    }
    function dot(p, color, size) {
      size = size || 8;
      ctx.beginPath();
      ctx.arc(p.x, p.y, size, 0, Math.PI * 2);
      ctx.fillStyle = color;
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#ffffff';
      ctx.stroke();
    }
    function resize() {
      const rect = canvas.getBoundingClientRect();
      const ratio = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.floor(rect.width * ratio));
      canvas.height = Math.max(1, Math.floor(rect.height * ratio));
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      draw();
    }
"""

_SETTLE_DRAW = """
    function draw() {
      var rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);
      ctx.fillStyle = '#060a0e';
      ctx.fillRect(0, 0, rect.width, rect.height);
      var box = {
        x: 14, y: 62,
        w: Math.max(200, rect.width - 28),
        h: Math.max(280, rect.height - 76),
      };
      drawHookSway(box);
    }
"""

_SETTLE_BOOT = """
    const canvas = document.getElementById('scene');
    const ctx = canvas.getContext('2d');
    const ANTI_SWAY = true;
    let SWAY_TOL_DEG = 0.3;
    let _liveSwayState = null;
    let _swayFrames = [];
    let _settleActive = false;

    const els = {
      rate: document.getElementById('rate'),
      modeLabel: document.getElementById('modeLabel'),
      sessionState: document.getElementById('sessionState'),
      algoLabel: document.getElementById('algoLabel'),
      phase: document.getElementById('phase'),
      commandMsg: document.getElementById('commandMsg'),
      btnStartSettle: document.getElementById('btnStartSettle'),
      btnStopSettle: document.getElementById('btnStopSettle'),
      swayStatus: document.getElementById('swayStatus'),
      thetaX: document.getElementById('thetaX'),
      thetaY: document.getElementById('thetaY'),
      thetaPeak: document.getElementById('thetaPeak'),
      rawPitch: document.getElementById('rawPitch'),
      rawRoll: document.getElementById('rawRoll'),
      swayAge: document.getElementById('swayAge'),
      antiSwayMode: document.getElementById('antiSwayMode'),
      antiSwayHint: document.getElementById('antiSwayHint'),
      ctrlMsg: document.getElementById('ctrlMsg'),
      plcDot: document.getElementById('plcDot'),
      hbDot: document.getElementById('hbDot'),
      plcStatusText: document.getElementById('plcStatusText'),
      hbStatusText: document.getElementById('hbStatusText'),
      ctrlVx: document.getElementById('ctrlVx'),
      ctrlVy: document.getElementById('ctrlVy'),
      ctrlHz: document.getElementById('ctrlHz'),
      ctrlGrip: document.getElementById('ctrlGrip'),
      btnStop: document.getElementById('btnStop'),
      btnReset: document.getElementById('btnReset'),
    };

    function updateAntiSwayHint() {
      if (!els.antiSwayHint || !els.antiSwayMode) return;
      const opt = els.antiSwayMode.selectedOptions
        ? els.antiSwayMode.selectedOptions[0]
        : null;
      const desc = opt && opt.dataset ? opt.dataset.description : '';
      els.antiSwayHint.textContent = desc || '';
      if (els.algoLabel && opt) {
        els.algoLabel.textContent = (opt.value || '').slice(0, 14);
      }
    }

    function getAntiSwayMode() {
      if (!els.antiSwayMode) return 'crane_remote';
      return els.antiSwayMode.value || 'crane_remote';
    }

    function setAntiSwayMode(mode) {
      if (!els.antiSwayMode) return;
      const sel = els.antiSwayMode;
      const has = Array.from(sel.options).some(function(o) { return o.value === mode; });
      sel.value = has ? mode : 'crane_remote';
      updateAntiSwayHint();
    }

    if (els.antiSwayMode) {
      els.antiSwayMode.addEventListener('change', function() {
        updateAntiSwayHint();
        syncQueryAntiSwayMode();
      });
      updateAntiSwayHint();
    }

    function syncQueryAntiSwayMode() {
      const params = new URLSearchParams(window.location.search);
      const mode = getAntiSwayMode();
      params.set('anti_sway_mode', mode);
      const q = params.toString();
      history.replaceState(null, '', q ? ('?' + q) : window.location.pathname);
    }

    (function initFromQuery() {
      const params = new URLSearchParams(window.location.search);
      const asm = params.get('anti_sway_mode');
      if (asm) setAntiSwayMode(asm);
      else syncQueryAntiSwayMode();
    })();

    function showCommandMsg(text, color) {
      const c = color || '#91a3b3';
      if (els.commandMsg) {
        els.commandMsg.textContent = text;
        els.commandMsg.style.color = c;
      }
    }

    function syncSwayFromSnapshot(s) {
      if (s && s.tol_deg != null) SWAY_TOL_DEG = s.tol_deg;
      _liveSwayState = s;
      if (s && s.valid) recordSwaySample(s);
      if (s && s.valid) {
        els.swayStatus.textContent = s.stale ? '倾角仪断流' : '倾角仪已连接';
        els.swayStatus.style.color = s.stale ? '#e05a47' : '#5ebd72';
        els.thetaX.textContent = s.theta_x_deg != null ? s.theta_x_deg.toFixed(3) : '--';
        els.thetaY.textContent = s.theta_y_deg != null ? s.theta_y_deg.toFixed(3) : '--';
        els.thetaPeak.textContent = s.theta_peak_deg != null ? s.theta_peak_deg.toFixed(3) : '--';
        els.rawPitch.textContent = s.pitch != null ? s.pitch.toFixed(3) : '--';
        els.rawRoll.textContent = s.roll != null ? s.roll.toFixed(3) : '--';
        els.swayAge.textContent = s.age_ms != null ? (s.age_ms + ' ms') : '--';
      } else {
        els.swayStatus.textContent = s && s.stale ? '倾角仪断流' : '等待倾角仪...';
        els.swayStatus.style.color = '#e05a47';
        els.thetaX.textContent = els.thetaY.textContent = els.thetaPeak.textContent = '--';
      }
      draw();
    }

    function pollInclinometer() {
      if (!PLC_MODE) {
        return;
      }
      fetch('/api/sway-snapshot')
        .then(function(r) { return r.json(); })
        .then(syncSwayFromSnapshot)
        .catch(function() { syncSwayFromSnapshot(null); });
    }

    setInterval(pollInclinometer, 100);

    function pollSettleControl() {
      fetch('/api/control-state')
        .then(function(r) { return r.json(); })
        .then(function(s) {
          if (s.running) {
            setSettleUiActive(true);
            if (s.scheduler_phase_label) {
              els.phase.textContent = s.scheduler_phase_label;
              els.phase.style.color = '#59a7e8';
            }
          }
          if (s.done) {
            setSettleUiActive(false);
            els.phase.textContent = '消摆完成';
            els.phase.style.color = '#5ebd72';
            showCommandMsg('摆角已判稳，消摆结束', '#5ebd72');
            if (typeof markRunLogReady === 'function') markRunLogReady();
          }
          if (s.stopped) {
            setSettleUiActive(false);
            els.phase.textContent = '已停止';
            els.phase.style.color = '#e05a47';
            if (typeof markRunLogReady === 'function') markRunLogReady();
          }
          if (s.error) {
            setSettleUiActive(false);
            showCommandMsg('Error: ' + s.error, '#e05a47');
            if (typeof markRunLogReady === 'function') markRunLogReady();
          }
          var d = s.latest;
          if (!PLC_MODE && d && d.theta_x != null && d.theta_y != null) {
            var k = 180 / Math.PI;
            syncSwayFromSnapshot({
              valid: true,
              stale: false,
              sim: true,
              theta_x_deg: d.theta_x * k,
              theta_y_deg: d.theta_y * k,
              theta_peak_deg: Math.max(Math.abs(d.theta_x), Math.abs(d.theta_y)) * k,
              tol_deg: SWAY_TOL_DEG,
            });
            els.swayStatus.textContent = '仿真消摆运行中';
            els.swayStatus.style.color = '#5ebd72';
          }
        })
        .catch(function() {});
    }

    setInterval(pollSettleControl, 100);

    function setSettleUiActive(active) {
      _settleActive = active;
      els.sessionState.textContent = active ? 'Damping' : 'Idle';
      els.sessionState.style.color = active ? '#5ebd72' : '';
      if (els.btnStartSettle) els.btnStartSettle.disabled = active;
      if (els.btnStopSettle) els.btnStopSettle.disabled = !active;
    }

    els.btnStartSettle.addEventListener('click', function() {
      var mode = getAntiSwayMode();
      if (typeof resetRunLogReady === 'function') resetRunLogReady();
      showCommandMsg('启动中...', '#91a3b3');
      fetch('/api/start-settle', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ anti_sway_mode: mode, sim: !PLC_MODE }),
      })
        .then(function(r) { return r.json(); })
        .then(function(data) {
          if (data.ok) {
            setSettleUiActive(true);
            showCommandMsg('消摆已启动 — ' + (data.phase_label || mode), '#5ebd72');
            if (!PLC_MODE) {
              els.phase.textContent = '仿真消摆运行中（初始扰动已注入）';
            }
          } else {
            showCommandMsg(data.error || '启动失败', '#e05a47');
          }
        })
        .catch(function(err) {
          showCommandMsg('Failed: ' + err.message, '#e05a47');
        });
    });

    els.btnStopSettle.addEventListener('click', function() {
      fetch('/api/stop')
        .then(function() { return fetch('/api/reset'); })
        .then(function() {
          setSettleUiActive(false);
          els.phase.textContent = PLC_MODE ? '就绪 — 可再次手动晃钩' : '仿真就绪';
          showCommandMsg('已停止消摆', '#91a3b3');
        });
    });

    document.querySelectorAll('.tab').forEach(function(tab) {
      tab.addEventListener('click', function() {
        document.querySelectorAll('.tab, .tab-panel').forEach(function(el) {
          el.classList.remove('active');
        });
        tab.classList.add('active');
        document.getElementById('tab-' + tab.dataset.tab).classList.add('active');
      });
    });

    if (PLC_MODE) {
      els.modeLabel.textContent = '真机';
      els.rate.textContent = 'Live';
      els.phase.textContent = '就绪 — 手动晃钩后点开始消摆';
      pollPlcStatus();
      setInterval(pollPlcStatus, 500);
    } else {
      els.modeLabel.textContent = '仿真';
      els.rate.textContent = 'Sim';
      els.phase.textContent = '仿真消摆 — 点击开始（自动注入初始摆角）';
    }

    window.addEventListener('resize', resize);
    resize();

    function pollPlcStatus() {
      fetch('/api/plc-status')
        .then(function(r) { return r.json(); })
        .then(function(s) {
          if (s.connected) {
            els.plcDot.classList.add('on');
            els.plcStatusText.style.color = 'var(--green)';
          } else {
            els.plcDot.classList.remove('on');
            els.plcStatusText.style.color = 'var(--red)';
          }
          if (s.heartbeat) {
            els.hbDot.classList.add('on');
            els.hbStatusText.style.color = 'var(--green)';
          } else {
            els.hbDot.classList.remove('on');
            els.hbStatusText.style.color = 'var(--red)';
          }
          if (els.ctrlVx) els.ctrlVx.textContent = s.last_vx.toFixed(3) + ' m/s';
          if (els.ctrlVy) els.ctrlVy.textContent = s.last_vy.toFixed(3) + ' m/s';
          if (els.ctrlHz) els.ctrlHz.textContent = s.last_hz.toFixed(3) + ' m';
        })
        .catch(function() {});
    }

    if (els.btnStop) {
      els.btnStop.addEventListener('click', function() {
        fetch('/api/stop').then(function() {
          setSettleUiActive(false);
          if (els.ctrlMsg) {
            els.ctrlMsg.textContent = 'STOP 已发送';
            els.ctrlMsg.style.color = '#e05a47';
          }
        });
      });
    }
    if (els.btnReset) {
      els.btnReset.addEventListener('click', function() {
        fetch('/api/reset').then(function() {
          if (els.ctrlMsg) {
            els.ctrlMsg.textContent = 'Control reset';
            els.ctrlMsg.style.color = '#5ebd72';
          }
        });
      });
    }
"""


def render_settle_html(plc_mode: bool = False) -> str:
    mode_label = '真机' if plc_mode else '仿真'
    nav_id = 'settle-live' if plc_mode else 'settle-sim'
    page_body = (
        _SETTLE_PAGE_BODY.replace('__PLATFORM_NAV__', platform_nav_markup(nav_id))
        .replace(
            '__ANTI_SWAY_SELECT__',
            anti_sway_select_markup(default_mode='crane_remote', scope='settle'),
        )
    )
    plc_js = 'true' if plc_mode else 'false'
    return (
        '<!doctype html>\n'
        '<html lang="zh-CN">\n'
        '<head>\n'
        '  <meta charset="utf-8">\n'
        '  <meta name="viewport" content="width=device-width, initial-scale=1">\n'
        f'  <title>消摆测试 — {mode_label}</title>\n'
        f'  <style>{LIVE_VIEW_BASE_CSS}{PTP_EXTRA_CSS}{RUN_LOG_EXTRA_CSS}</style>\n'
        '</head>\n'
        + page_body
        + '  <script>\n'
        + f'    const PLC_MODE = {plc_js};\n'
        + RUN_LOG_SAVE_JS
        + SETTLE_RUN_LOG_INIT
        + _SETTLE_BOOT
        + _SETTLE_CANVAS_HELPERS
        + PTP_SWAY_STAGE_JS
        + _SETTLE_DRAW
        + '  </script>\n'
        '</body>\n'
        '</html>'
    )
