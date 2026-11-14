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
        <div class="topbar-left">
          <h1>消摆测试</h1>
          <div class="deepseek-card" id="staticAngleCard" hidden>
            <span class="deepseek-card-title">静态角 θ*</span>
            <span class="deepseek-stat"><i>θx*</i><b id="staticThetaX">--</b></span>
            <span class="deepseek-stat"><i>θy*</i><b id="staticThetaY">--</b></span>
          </div>
        </div>
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

_SETTLE_EXTRA_CSS = """
    .topbar-left {
      display: flex;
      align-items: center;
      gap: 12px;
      min-width: 0;
      flex: 0 1 auto;
      pointer-events: none;
    }
    .topbar-left h1 {
      flex: 0 0 auto;
      white-space: nowrap;   /* 不能被静态角卡片挤成竖排 */
    }
    .deepseek-card {
      display: flex;
      align-items: baseline;
      gap: 10px;
      padding: 5px 10px;
      border: 1px solid #3f6f9c;
      border-radius: 5px;
      background: rgba(23, 33, 43, 0.92);
      font-size: 12px;
      white-space: nowrap;
      overflow: hidden;
      flex: 0 0 auto;   /* 不许被 platform-nav / h1 挤压没 */
    }
    @media (max-width: 1280px) {
      .deepseek-card .deepseek-card-title { display: none; }
    }
    .deepseek-card[hidden] { display: none; }
    .deepseek-card-title { color: #8fd3ff; font-weight: 700; }
    .deepseek-stat { color: #8a9bab; }
    .deepseek-stat i { font-style: normal; margin-right: 4px; }
    .deepseek-stat b {
      color: #dce7ee;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      font-weight: 700;
    }
    .deepseek-card-meta {
      color: #6b7c8c;
      font-size: 11px;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
    }
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
    /** 与 drawHookSway() 内部布局保持一致，用于把 θ* 标到同一个十字靶标上。 */
    function swayCrossGeometry(box) {
      var pad = 8;
      var infoH = 40 + 6 + 3 * 14 + 10;
      var area = {
        x: box.x + pad,
        y: box.y + pad,
        w: Math.max(60, box.w - pad * 2),
        h: Math.max(60, box.h - pad * 2 - infoH),
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

    /** deepseek-anti：把估算的静态姿态 θ* 画在左侧摆角靶标上 + 数值卡片。 */
    function drawStaticPoseOverlay(box) {
      if (typeof getAntiSwayMode !== 'function') return;
      if (getAntiSwayMode() !== 'deepseek_anti') return;
      var g = swayCrossGeometry(box);
      var d = (typeof _staticAngleDebug !== 'undefined') ? _staticAngleDebug : null;
      var accent = '#6fe3b4';

      if (d) {
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

      var cw = 252;
      var ch = 112;
      var cx0 = g.area.x + g.area.w - cw - 8;
      if (cx0 < g.area.x + 8) cx0 = g.area.x + 8;
      var cy0 = g.area.y + 8;

      ctx.save();
      ctx.fillStyle = 'rgba(6, 14, 20, 0.9)';
      ctx.strokeStyle = '#2f5f80';
      ctx.lineWidth = 1;
      ctx.fillRect(cx0, cy0, cw, ch);
      ctx.strokeRect(cx0 + 0.5, cy0 + 0.5, cw - 1, ch - 1);

      ctx.textAlign = 'left';
      ctx.textBaseline = 'alphabetic';
      ctx.font = '700 11px system-ui, sans-serif';
      ctx.fillStyle = '#8fd3ff';
      ctx.fillText('静态姿态 θ* · deepseek-anti', cx0 + 9, cy0 + 15);

      var colW = (cw - 18) / 2;
      var x1 = cx0 + 9;
      var x2 = cx0 + 9 + colW;
      function cell(label, value, x, y, color) {
        ctx.font = '600 10px system-ui, sans-serif';
        ctx.fillStyle = '#8a9bab';
        ctx.fillText(label, x, y);
        ctx.font = '700 12px ui-monospace, SFMono-Regular, Menlo, monospace';
        ctx.fillStyle = color || '#dce7ee';
        ctx.fillText(value, x + 40, y);
      }

      if (!d) {
        ctx.font = '700 11px system-ui, sans-serif';
        ctx.fillStyle = '#91a3b3';
        ctx.fillText('开始消摆后估计', x1, cy0 + 38);
      } else {
        var ready = !!(d.ready_x && d.ready_y);
        var amp = Math.max(d.amp_x_deg || 0, d.amp_y_deg || 0);
        var cov = Math.min(1, Math.max(d.coverage_x || 0, d.coverage_y || 0));
        cell('θx*', fmtSignedDeg(d.theta_static_x_deg), x1, cy0 + 34);
        cell('θy*', fmtSignedDeg(d.theta_static_y_deg), x2, cy0 + 34);
        cell('ωx', fmtSignedNum(d.rate_x_deg_s, 1) + '°/s', x1, cy0 + 54, '#59a7e8');
        cell('Δvx', fmtSignedNum(d.dv_x, 3), x2, cy0 + 54, '#f0a83b');
        cell('ωy', fmtSignedNum(d.rate_y_deg_s, 1) + '°/s', x1, cy0 + 74, '#59a7e8');
        cell('Δvy', fmtSignedNum(d.dv_y, 3), x2, cy0 + 74, '#f0a83b');
        var status;
        var statusColor;
        if (d.budget_blocked) {
          status = '位移预算暂停 (' + (d.dist_xy || 0).toFixed(2) + ' m)';
          statusColor = '#e05a47';
        } else if (d.guard_x || d.guard_y) {
          status = '增长保护暂停';
          statusColor = '#e05a47';
        } else if (ready) {
          status = '已收敛 · 摆幅 ' + amp.toFixed(2) + '° · K ' + (d.gain_eff || 0).toFixed(2)
            + ' · lead ' + (d.lead_s || 0).toFixed(1) + 's';
          statusColor = '#5ebd72';
        } else {
          status = '估计中 ' + Math.round(cov * 100) + '% · 周期 ' + (d.period_s || 0).toFixed(1) + 's';
          statusColor = '#f0a83b';
        }
        ctx.font = '700 10px system-ui, sans-serif';
        ctx.fillStyle = statusColor;
        ctx.fillText(truncateText(status, cw - 18), x1, cy0 + 98);
      }
      ctx.restore();
    }

    function draw() {
      var rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);
      ctx.fillStyle = '#060a0e';
      ctx.fillRect(0, 0, rect.width, rect.height);
      // 顶部留出 topbar（标题 + 静态角卡片）的高度，避免标题叠在一起
      var box = {
        x: 14, y: 78,
        w: Math.max(200, rect.width - 28),
        h: Math.max(280, rect.height - 92),
      };
      drawHookSway(box);
      drawSettleBaselineOverlay(box);
      drawStaticPoseOverlay(box);
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
    let SWAY_TOL_DEG = 0.3;
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
      staticAngleCard: document.getElementById('staticAngleCard'),
      staticThetaX: document.getElementById('staticThetaX'),
      staticThetaY: document.getElementById('staticThetaY'),
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

    /** deepseek-anti 的静态倾角估计 → 左侧顶栏卡片。其它算法隐藏。 */
    function updateStaticAngleCard() {
      var card = els.staticAngleCard;
      if (!card) return;
      var isDeepseek = getAntiSwayMode() === 'deepseek_anti';
      card.hidden = !isDeepseek;
      if (!isDeepseek) return;
      var d = _staticAngleDebug;
      if (!d) {
        els.staticThetaX.textContent = '--';
        els.staticThetaY.textContent = '--';
        card.title = _settleActive
          ? '估计中…（需要累积约 1 个摆周期的数据）'
          : 'θ* 需要先点「开始消摆」，运行中实时估计';
        return;
      }
      els.staticThetaX.textContent = fmtSignedDeg(d.theta_static_x_deg);
      els.staticThetaY.textContent = fmtSignedDeg(d.theta_static_y_deg);
      var ready = !!(d.ready_x && d.ready_y);
      var cov = Math.min(1, Math.max(d.coverage_x || 0, d.coverage_y || 0));
      var amp = Math.max(d.amp_x_deg || 0, d.amp_y_deg || 0);
      var rms = Math.max(d.rms_x_deg || 0, d.rms_y_deg || 0);
      var period = d.period_s ? d.period_s.toFixed(1) + 's' : '--';
      card.title = ready
        ? ('已收敛 · 摆幅 ' + amp.toFixed(2) + '° · 残差 ' + rms.toFixed(2) + '°')
        : ('估计中 ' + Math.round(cov * 100) + '% · 周期 ' + period);
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
        updateStaticAngleCard();
        syncQueryAntiSwayMode();
      });
      updateAntiSwayHint();
      updateStaticAngleCard();
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
      updateStaticAngleCard();   // 模式可能来自 URL query，需在设置后刷新一次
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
          if (d && d.anti_sway_debug) {
            _staticAngleDebug = d.anti_sway_debug;
          } else if (!s.running) {
            _staticAngleDebug = null;
          }
          updateStaticAngleCard();
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
      updateStaticAngleCard();
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
