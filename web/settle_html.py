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
        <div class="metric"><div class="label">消摆计时</div><div class="value" id="settleElapsed">—</div></div>
        <div class="metric"><div class="label">初始最大峰</div><div class="value" id="settleInitialPeak">—</div></div>
        <div class="metric"><div class="label">Algorithm</div><div class="value" id="algoLabel">—</div></div>
      </section>
      <section class="readout">
        <div class="phase" id="phase">Loading</div>
        <p class="settle-intro" id="settleIntro">
          保持大车/小车不动，人工晃钩后点「开始消摆」；左侧看摆角靶标，右下角读倾角仪。
        </p>
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
          <div class="inclinometer-deepseek" id="deepseekInclinometerExtra" hidden>
            <div class="inclinometer-deepseek-title">deepseek · θ* 与残余</div>
            <div class="loco-table inclinometer-deepseek-table">
              <div class="loco-hdr"></div>
              <div class="loco-hdr">静态 θ*</div>
              <div class="loco-hdr">θ_ac</div>
              <div class="loco-axis">θx</div>
              <div class="loco-val" id="thetaStaticX">--</div>
              <div class="loco-val" id="thetaAcX">--</div>
              <div class="loco-axis">θy</div>
              <div class="loco-val" id="thetaStaticY">--</div>
              <div class="loco-val" id="thetaAcY">--</div>
              <div class="loco-axis">|θ_ac|</div>
              <div class="loco-val" id="thetaAcPeak">--</div>
              <div class="loco-val" id="deepseekReady">--</div>
            </div>
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

_SETTLE_EXTRA_CSS = """
    .shell.settle-focus .bottom-panel {
      margin-top: auto;
    }
    .inclinometer-deepseek {
      margin-top: 10px;
      padding-top: 8px;
      border-top: 1px solid var(--line);
    }
    .inclinometer-deepseek[hidden] { display: none; }
    .inclinometer-deepseek-title {
      font-size: 11px;
      font-weight: 700;
      color: #8fd3ff;
      margin-bottom: 6px;
    }
    .inclinometer-deepseek-table { margin-top: 0; }
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
    /** 消摆页专用：全高摆角靶标，无画布底部倾角条。 */
    function drawSettleHookSway(box) {
      var swayTitle = box.w < 320 ? 'Hook Sway' : 'Hook Sway / 抓钩摆角';
      drawPanel(box, swayTitle);
      var sway = _liveSwayState;
      var tol = (sway && sway.tol_deg != null) ? sway.tol_deg : SWAY_TOL_DEG;
      var valid = !!(sway && sway.valid);
      var thetaX = valid ? sway.theta_x_deg : null;
      var thetaY = valid ? sway.theta_y_deg : null;
      var pad = 8;
      var crossArea = {
        x: box.x + pad,
        y: box.y + pad,
        w: Math.max(60, box.w - pad * 2),
        h: Math.max(60, box.h - pad * 2),
      };
      drawSwayCross(crossArea, thetaX, thetaY, tol);
    }

    /** 摆角靶标几何（与 drawSettleHookSway 一致）。 */
    function swayCrossGeometry(box) {
      var pad = 8;
      var area = {
        x: box.x + pad,
        y: box.y + pad,
        w: Math.max(60, box.w - pad * 2),
        h: Math.max(60, box.h - pad * 2),
      };
      var leftGutter = Math.min(40, Math.max(30, area.w * 0.14));
      var topGutter = 16;
      var rightGutter = 22;
      var bottomGutter = Math.min(22, Math.max(16, area.h * 0.12));
      var plotW = Math.max(40, area.w - leftGutter - rightGutter);
      var plotH = Math.max(40, area.h - topGutter - bottomGutter);
      return {
        area: area,
        cx: area.x + leftGutter + plotW / 2,
        cy: area.y + topGutter + plotH / 2,
        span: Math.max(18, Math.min(plotW, plotH) / 2 - 8),
      };
    }

    /** deepseek：绿菱形 θ* 标在十字靶标上（展示区右上角计时条不动）。 */
    function drawStaticPoseOverlay(box) {
      if (typeof getAntiSwayMode !== 'function') return;
      if (getAntiSwayMode() !== 'deepseek_anti') return;
      var g = swayCrossGeometry(box);
      var d = (typeof _staticAngleDebug !== 'undefined') ? _staticAngleDebug : null;
      if (!d) return;
      var accent = '#6fe3b4';
      var sxDeg = d.theta_static_y_deg || 0;
      var syDeg = d.theta_static_x_deg || 0;
      var tx = Math.max(-1, Math.min(1, sxDeg));
      var ty = Math.max(-1, Math.min(1, syDeg));
      var px = g.cx + tx * g.span;
      var py = g.cy - ty * g.span;

      ctx.save();
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = 'rgba(111, 227, 180, 0.75)';
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(g.cx, g.cy);
      ctx.lineTo(px, py);
      ctx.stroke();
      ctx.setLineDash([]);

      var r = 7;
      ctx.beginPath();
      ctx.moveTo(px, py - r);
      ctx.lineTo(px + r, py);
      ctx.lineTo(px, py + r);
      ctx.lineTo(px - r, py);
      ctx.closePath();
      ctx.fillStyle = 'rgba(111, 227, 180, 0.22)';
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = accent;
      ctx.stroke();

      ctx.font = '700 11px system-ui, sans-serif';
      ctx.fillStyle = accent;
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      ctx.fillText('θ*', px + 11, py - 10);
      ctx.restore();
    }

    /** 迷你十字靶：中心=θ*，圆点=θ_ac（仅图示，不标数值）。 */
    function drawResidualMiniCross(plot, acXDeg, acYDeg, tol, opts) {
      opts = opts || {};
      var ready = !!opts.ready;
      var hasAc = acXDeg != null && acYDeg != null && isFinite(acXDeg) && isFinite(acYDeg);
      var peak = hasAc ? Math.max(Math.abs(acXDeg), Math.abs(acYDeg)) : 0;
      var fullScaleDeg = Math.max(0.2, Math.min(1.0, Math.max(peak * 1.2, tol * 2, 0.15)));
      var cx = plot.x + plot.w / 2;
      var cy = plot.y + plot.h / 2;
      var span = Math.max(14, Math.min(plot.w, plot.h) / 2 - 6);
      var pxPerDeg = span / fullScaleDeg;
      var toPx = function(deg) {
        return Math.max(-span, Math.min(span, deg * pxPerDeg));
      };

      ctx.fillStyle = '#0c1218';
      ctx.fillRect(plot.x, plot.y, plot.w, plot.h);
      ctx.strokeStyle = '#2a3947';
      ctx.lineWidth = 1;
      ctx.strokeRect(plot.x + 0.5, plot.y + 0.5, plot.w - 1, plot.h - 1);

      ctx.save();
      ctx.beginPath();
      ctx.rect(plot.x + 1, plot.y + 1, plot.w - 2, plot.h - 2);
      ctx.clip();

      var ringDegs = [tol, tol * 2, fullScaleDeg];
      var ringStyles = [
        { stroke: '#5ebd72', dash: [] },
        { stroke: '#f0a83b', dash: [4, 3] },
        { stroke: '#e86850', dash: [3, 3] },
      ];
      for (var ri = 0; ri < ringDegs.length; ri++) {
        var rp = ringDegs[ri] * pxPerDeg;
        if (rp < 2 || rp > span + 1) continue;
        ctx.strokeStyle = ringStyles[ri].stroke;
        ctx.setLineDash(ringStyles[ri].dash);
        ctx.lineWidth = ri === 0 ? 1.5 : 1;
        ctx.beginPath();
        ctx.arc(cx, cy, rp, 0, Math.PI * 2);
        ctx.stroke();
      }
      ctx.setLineDash([]);

      ctx.strokeStyle = 'rgba(142, 163, 184, 0.35)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(cx - span, cy);
      ctx.lineTo(cx + span, cy);
      ctx.moveTo(cx, cy - span);
      ctx.lineTo(cx, cy + span);
      ctx.stroke();

      // θ* 固定在原点
      ctx.fillStyle = 'rgba(111, 227, 180, 0.35)';
      ctx.beginPath();
      ctx.arc(cx, cy, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#6fe3b4';
      ctx.lineWidth = 1.5;
      ctx.stroke();

      if (hasAc) {
        var px = cx + toPx(acYDeg);
        var py = cy - toPx(acXDeg);
        var col = swayAngleColor(peak, tol);
        ctx.setLineDash([3, 3]);
        ctx.strokeStyle = 'rgba(220, 231, 238, 0.35)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(px, py);
        ctx.stroke();
        ctx.setLineDash([]);
        dot({ x: px, y: py }, col, 5);
      } else if (!ready) {
        ctx.fillStyle = 'rgba(145, 163, 179, 0.25)';
        ctx.beginPath();
        ctx.arc(cx, cy, 3, 0, Math.PI * 2);
        ctx.fill();
      }

      ctx.restore();

      // 图例：绿点=θ*，彩点=θ_ac
      var legY = plot.y + plot.h - 5;
      ctx.fillStyle = '#6fe3b4';
      ctx.beginPath();
      ctx.arc(plot.x + 10, legY, 3, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = hasAc ? swayAngleColor(peak, tol) : '#91a3b3';
      ctx.beginPath();
      ctx.arc(plot.x + 28, legY, 3, 0, Math.PI * 2);
      ctx.fill();
    }

    /** 展示区右下角：θ_ac 迷你十字靶（deepseek）。 */
    function drawStaticResidualOverlay(box) {
      if (typeof getAntiSwayMode !== 'function') return;
      if (getAntiSwayMode() !== 'deepseek_anti') return;
      var g = swayCrossGeometry(box);
      var d = (typeof _staticAngleDebug !== 'undefined') ? _staticAngleDebug : null;
      var sway = _liveSwayState;
      var tol = (sway && sway.tol_deg != null) ? sway.tol_deg : SWAY_TOL_DEG;
      var panelSize = Math.min(172, Math.max(132, Math.min(g.area.w, g.area.h) * 0.36));
      var titleH = 16;
      var cx0 = g.area.x + g.area.w - panelSize - 8;
      var cy0 = g.area.y + g.area.h - panelSize - titleH - 8;
      var ready = !!(d && d.ready_x && d.ready_y);
      var frameCol = !d ? '#2f5f80' : (ready ? '#5ebd72' : '#f0a83b');

      ctx.save();
      ctx.fillStyle = 'rgba(6, 14, 20, 0.92)';
      ctx.strokeStyle = frameCol;
      ctx.lineWidth = ready ? 2 : 1;
      ctx.fillRect(cx0, cy0, panelSize, panelSize + titleH);
      ctx.strokeRect(cx0 + 0.5, cy0 + 0.5, panelSize - 1, panelSize + titleH - 1);

      ctx.font = '700 10px system-ui, sans-serif';
      ctx.fillStyle = '#8fd3ff';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      ctx.fillText('θ_ac', cx0 + 8, cy0 + 10);

      var plot = {
        x: cx0 + 6,
        y: cy0 + titleH + 2,
        w: panelSize - 12,
        h: panelSize - 12,
      };

      var acX = null;
      var acY = null;
      if (d) {
        acX = d.theta_ac_x_deg;
        acY = d.theta_ac_y_deg;
        if (acX == null && sway && sway.valid) {
          acX = (sway.theta_x_deg || 0) - (d.theta_static_x_deg || 0);
        }
        if (acY == null && sway && sway.valid) {
          acY = (sway.theta_y_deg || 0) - (d.theta_static_y_deg || 0);
        }
      }
      drawResidualMiniCross(plot, acX, acY, tol, { ready: ready });
      ctx.restore();
    }

    function draw() {
      var rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);
      ctx.fillStyle = '#060a0e';
      ctx.fillRect(0, 0, rect.width, rect.height);
      var box = {
        x: 14, y: 52,
        w: Math.max(200, rect.width - 28),
        h: Math.max(280, rect.height - 66),
      };
      drawSettleHookSway(box);
      drawSettleBaselineOverlay(box);
      drawStaticPoseOverlay(box);
      drawStaticResidualOverlay(box);
    }

    /** 消摆开始头几秒锁定的初始最大摆角：靶心上画参考圈 + 峰值时刻的方向点。 */
    function drawSettleBaselineOverlay(box) {
      var b = (typeof _settleBaseline !== 'undefined') ? _settleBaseline : null;
      if (!b || !b.peakDeg || b.peakDeg <= 0) return;
      var g = swayCrossGeometry(box);
      var pxPerDeg = g.span;
      var r = Math.min(g.span, b.peakDeg * pxPerDeg);

      ctx.save();
      ctx.beginPath();
      ctx.rect(g.area.x, g.area.y, g.area.w, g.area.h);
      ctx.clip();

      ctx.setLineDash([7, 5]);
      ctx.strokeStyle = 'rgba(198, 130, 255, 0.92)';
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      ctx.arc(g.cx, g.cy, r, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);

      if (b.thetaXDeg != null && b.thetaYDeg != null) {
        var ty = Math.max(-1, Math.min(1, b.thetaYDeg));
        var tx = Math.max(-1, Math.min(1, b.thetaXDeg));
        var px = g.cx + ty * g.span;
        var py = g.cy - tx * g.span;
        ctx.beginPath();
        ctx.arc(px, py, 10, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(198, 130, 255, 0.55)';
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(px, py, 3.5, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(198, 130, 255, 0.35)';
        ctx.fill();
      }

      var label = '初始最大 ' + b.peakDeg.toFixed(2) + '°';
      ctx.font = '700 10px system-ui, sans-serif';
      ctx.fillStyle = '#c878ff';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'bottom';
      var ly = g.cy - r - 5;
      if (ly < g.area.y + 4) ly = g.cy + r + 12;
      ctx.fillText(label, Math.max(g.area.x + 6, g.cx - r), ly);

      if (_settleActive && _settleElapsedS != null) {
        ctx.textAlign = 'right';
        ctx.textBaseline = 'top';
        ctx.fillStyle = '#8fd3ff';
        ctx.fillText(
          '消摆 ' + _settleElapsedS.toFixed(1) + ' s',
          g.area.x + g.area.w - 6,
          g.area.y + 4,
        );
      }
      ctx.restore();
    }
"""

_SETTLE_BOOT = """
    const canvas = document.getElementById('scene');
    const ctx = canvas.getContext('2d');
    const ANTI_SWAY = true;
    let SWAY_TOL_DEG = 0.05;
    let _liveSwayState = null;
    let _swayFrames = [];
    let _settleActive = false;
    /** 开始后约 2s 内采样的初始最大峰 [deg]，锁定后用于靶标参考圈。 */
    const SETTLE_BASELINE_CAPTURE_S = 2.0;
    let _settleT0Ms = null;
    let _settleElapsedS = null;
    let _settleBaseline = null;

    const els = {
      rate: document.getElementById('rate'),
      modeLabel: document.getElementById('modeLabel'),
      sessionState: document.getElementById('sessionState'),
      settleElapsed: document.getElementById('settleElapsed'),
      settleInitialPeak: document.getElementById('settleInitialPeak'),
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
      deepseekInclinometerExtra: document.getElementById('deepseekInclinometerExtra'),
      thetaStaticX: document.getElementById('thetaStaticX'),
      thetaStaticY: document.getElementById('thetaStaticY'),
      thetaAcX: document.getElementById('thetaAcX'),
      thetaAcY: document.getElementById('thetaAcY'),
      thetaAcPeak: document.getElementById('thetaAcPeak'),
      deepseekReady: document.getElementById('deepseekReady'),
    };

    let _staticAngleDebug = null;

    function fmtSignedNum(v, digits) {
      if (v == null || !isFinite(v)) return '--';
      var d = (digits == null) ? 3 : digits;
      return (v >= 0 ? '+' : '') + v.toFixed(d);
    }

    function fmtSignedDeg(v) {
      return fmtSignedNum(v, 3) + '°';
    }

    /** deepseek：倾角仪 Tab 右下角补充 θ* / θ_ac。 */
    function updateDeepseekInclinometerPanel() {
      var box = els.deepseekInclinometerExtra;
      if (!box) return;
      var isDeepseek = getAntiSwayMode() === 'deepseek_anti';
      box.hidden = !isDeepseek;
      if (!isDeepseek) return;
      var d = _staticAngleDebug;
      var sway = _liveSwayState;
      var set = function(el, text) { if (el) el.textContent = text; };
      if (!d) {
        set(els.thetaStaticX, '--');
        set(els.thetaStaticY, '--');
        set(els.thetaAcX, '--');
        set(els.thetaAcY, '--');
        set(els.thetaAcPeak, '--');
        set(els.deepseekReady, _settleActive ? '估计中' : '—');
        return;
      }
      set(els.thetaStaticX, fmtSignedDeg(d.theta_static_x_deg));
      set(els.thetaStaticY, fmtSignedDeg(d.theta_static_y_deg));
      var acX = d.theta_ac_x_deg;
      var acY = d.theta_ac_y_deg;
      if (acX == null && sway && sway.valid) {
        acX = (sway.theta_x_deg || 0) - (d.theta_static_x_deg || 0);
      }
      if (acY == null && sway && sway.valid) {
        acY = (sway.theta_y_deg || 0) - (d.theta_static_y_deg || 0);
      }
      var peak = Math.max(Math.abs(acX || 0), Math.abs(acY || 0));
      set(els.thetaAcX, acX == null ? '--' : fmtSignedDeg(acX));
      set(els.thetaAcY, acY == null ? '--' : fmtSignedDeg(acY));
      set(els.thetaAcPeak, peak.toFixed(3) + '°');
      var ready = !!(d.ready_x && d.ready_y);
      var cov = Math.min(1, Math.max(d.coverage_x || 0, d.coverage_y || 0));
      set(els.deepseekReady, ready ? '已收敛' : ('估计 ' + Math.round(cov * 100) + '%'));
    }

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
        updateDeepseekInclinometerPanel();
        draw();
        syncQueryAntiSwayMode();
      });
      updateAntiSwayHint();
      updateDeepseekInclinometerPanel();
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
      updateDeepseekInclinometerPanel();
    })();

    function showCommandMsg(text, color) {
      const c = color || '#91a3b3';
      if (els.commandMsg) {
        els.commandMsg.textContent = text;
        els.commandMsg.style.color = c;
      }
    }

    function resetSettleMetrics() {
      _settleT0Ms = performance.now();
      _settleElapsedS = 0;
      _settleBaseline = {
        peakDeg: 0,
        thetaXDeg: null,
        thetaYDeg: null,
        locked: false,
        captureT0Ms: performance.now(),
      };
      updateSettleMetricsUi();
    }

    function updateSettleBaselineFromSway(sway) {
      if (!_settleActive || !sway || !sway.valid || !_settleBaseline) return;
      if (_settleBaseline.locked) return;
      var elapsed = (performance.now() - _settleBaseline.captureT0Ms) / 1000;
      var peak = sway.theta_peak_deg;
      if (peak != null && isFinite(peak) && peak >= _settleBaseline.peakDeg) {
        _settleBaseline.peakDeg = peak;
        _settleBaseline.thetaXDeg = sway.theta_x_deg;
        _settleBaseline.thetaYDeg = sway.theta_y_deg;
      }
      if (elapsed >= SETTLE_BASELINE_CAPTURE_S) {
        _settleBaseline.locked = true;
        if (_settleBaseline.peakDeg <= 0 && peak != null && isFinite(peak)) {
          _settleBaseline.peakDeg = peak;
          _settleBaseline.thetaXDeg = sway.theta_x_deg;
          _settleBaseline.thetaYDeg = sway.theta_y_deg;
        }
      }
      updateSettleMetricsUi();
    }

    function updateSettleMetricsUi() {
      if (els.settleElapsed) {
        if (_settleElapsedS != null && (_settleActive || _settleT0Ms != null)) {
          els.settleElapsed.textContent = _settleElapsedS.toFixed(1) + ' s';
          els.settleElapsed.style.color = _settleActive ? '#59a7e8' : '#5ebd72';
        } else {
          els.settleElapsed.textContent = '—';
          els.settleElapsed.style.color = '';
        }
      }
      if (els.settleInitialPeak) {
        var p = _settleBaseline && _settleBaseline.peakDeg;
        if (p != null && p > 0) {
          els.settleInitialPeak.textContent = p.toFixed(3) + '°';
          els.settleInitialPeak.style.color = '#c878ff';
        } else {
          els.settleInitialPeak.textContent = _settleActive ? '采样中…' : '—';
          els.settleInitialPeak.style.color = '';
        }
      }
    }

    function syncSettleElapsedFromControl(s) {
      if (s && s.running) {
        if (s.latest && s.latest.t != null && isFinite(s.latest.t)) {
          _settleElapsedS = s.latest.t;
        } else if (_settleT0Ms != null) {
          _settleElapsedS = (performance.now() - _settleT0Ms) / 1000;
        }
      } else if (s && (s.done || s.stopped) && s.latest && s.latest.t != null) {
        _settleElapsedS = s.latest.t;
      }
      updateSettleMetricsUi();
    }

    function syncSwayFromSnapshot(s) {
      if (s && s.tol_deg != null) SWAY_TOL_DEG = s.tol_deg;
      _liveSwayState = s;
      if (s && s.valid) recordSwaySample(s);
      updateSettleBaselineFromSway(s);
      if (s && s.valid) {
        els.swayStatus.textContent = s.stale ? '倾角仪断流' : '倾角仪已连接';
        els.swayStatus.style.color = s.stale ? '#e05a47' : '#5ebd72';
        els.thetaX.textContent = s.theta_x_deg != null ? s.theta_x_deg.toFixed(3) : '--';
        els.thetaY.textContent = s.theta_y_deg != null ? s.theta_y_deg.toFixed(3) : '--';
        els.thetaPeak.textContent = s.theta_peak_deg != null ? s.theta_peak_deg.toFixed(3) : '--';
        els.rawPitch.textContent = s.pitch != null ? s.pitch.toFixed(3) : '--';
        els.rawRoll.textContent = s.roll != null ? s.roll.toFixed(3) : '--';
        els.swayAge.textContent = s.age_ms != null ? (s.age_ms + ' ms') : '--';
        updateDeepseekInclinometerPanel();
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
          syncSettleElapsedFromControl(s);
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
            var doneMsg = (getAntiSwayMode() === 'deepseek_anti')
              ? '抓钩已贴近静态姿态 θ*（靶标圆点接近绿菱形），消摆结束'
              : '摆角已判稳，消摆结束';
            showCommandMsg(doneMsg, '#5ebd72');
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
          if (d && d.anti_sway_debug) {
            _staticAngleDebug = d.anti_sway_debug;
          } else if (!s.running) {
            _staticAngleDebug = null;
          }
          updateDeepseekInclinometerPanel();
          draw();
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
    pollSettleControl();   // 首屏立即拉一次，避免等 100ms 空白

    function setSettleUiActive(active) {
      var was = _settleActive;
      _settleActive = active;
      if (active && !was) {
        if (!_settleT0Ms) resetSettleMetrics();
      }
      els.sessionState.textContent = active ? 'Damping' : 'Idle';
      els.sessionState.style.color = active ? '#5ebd72' : '';
      if (els.btnStartSettle) els.btnStartSettle.disabled = active;
      if (els.btnStopSettle) els.btnStopSettle.disabled = !active;
    }

    els.btnStartSettle.addEventListener('click', function() {
      var mode = getAntiSwayMode();
      if (typeof resetRunLogReady === 'function') resetRunLogReady();
      _staticAngleDebug = null;
      updateDeepseekInclinometerPanel();
      resetSettleMetrics();
      _swayFrames.length = 0;
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
        f'  <style>{LIVE_VIEW_BASE_CSS}{PTP_EXTRA_CSS}{RUN_LOG_EXTRA_CSS}'
        f'{_SETTLE_EXTRA_CSS}</style>\n'
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
