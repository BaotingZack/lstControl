"""Manual push + active damping test pages (settle sim / settle live)."""

from web.platform_nav import platform_nav_markup
from web.ptp_html import anti_sway_select_markup
from web.ptp_sway_canvas import PTP_SWAY_STAGE_JS
from web.view_shared import LIVE_VIEW_BASE_CSS, PTP_EXTRA_CSS

_SETTLE_PAGE_BODY = """<body>
  <main class="shell settle-focus">
    <section class="stage settle-stage-layout">
      <canvas id="scene"></canvas>
      <div class="settle-stage-rail" id="settleStageRail" hidden>
        <section class="settle-stage-hud" id="settleStageHud" aria-label="摆角关键指标">
          <div class="settle-stage-hud-head">
            <span class="settle-stage-hud-title">摆角监测</span>
            <span class="settle-stage-hud-phase" id="stageHudReady">θ* 估计中</span>
          </div>
          <div class="settle-stage-hud-peak-block">
            <div class="settle-stage-hud-peak-val" id="stageHudPeak">--</div>
            <div class="settle-stage-hud-peak-lbl">当前峰 · deg</div>
          </div>
          <div class="settle-stage-hud-grid">
            <div class="settle-stage-hud-cell">
              <span class="lbl theta-x">θx</span>
              <span class="val" id="stageHudThetaX">--</span>
            </div>
            <div class="settle-stage-hud-cell">
              <span class="lbl theta-y">θy</span>
              <span class="val" id="stageHudThetaY">--</span>
            </div>
            <div class="settle-stage-hud-cell">
              <span class="lbl ac">|θ_ac|</span>
              <span class="val" id="stageHudAcPeak">--</span>
            </div>
            <div class="settle-stage-hud-cell">
              <span class="lbl init">初始峰</span>
              <span class="val init" id="stageHudInitPeak">--</span>
            </div>
          </div>
          <div class="settle-stage-hud-foot">
            <span id="stageHudElapsed">计时 —</span>
            <span id="stageHudTol">容差 —</span>
          </div>
        </section>
        <div class="settle-residual-widget" id="settleResidualWidget">
          <h2 class="settle-residual-widget-title">相对 θ* 残余</h2>
          <div class="settle-residual-widget-body">
            <canvas id="settleResidualCanvas" aria-label="相对静态姿态残余 θ_ac"></canvas>
          </div>
          <div class="settle-residual-widget-foot" id="settleResidualReadyHint">θ* 估计中</div>
        </div>
      </div>
      <div class="topbar settle-topbar">
        <h1>消摆测试<span class="stage-sub">抓钩摆角</span></h1>
        <div class="top-actions platform-nav">
          __PLATFORM_NAV__
          <div class="badge" id="rate">—</div>
        </div>
      </div>
    </section>
    <aside class="side settle-side">
      <section class="settle-side-top">
        <div class="settle-kpi-row">
          <div class="settle-kpi settle-kpi-elapsed">
            <div class="settle-kpi-label">运行计时</div>
            <div class="settle-kpi-value" id="settleElapsed">—</div>
          </div>
          <div class="settle-kpi settle-kpi-initial">
            <div class="settle-kpi-label">初始峰值</div>
            <div class="settle-kpi-value" id="settleInitialPeak">—</div>
          </div>
        </div>
        <div class="settle-meta-row">
          <div class="settle-meta-item"><span class="settle-meta-label">模式</span><span class="settle-meta-val" id="modeLabel">—</span></div>
          <div class="settle-meta-item"><span class="settle-meta-label">状态</span><span class="settle-meta-val" id="sessionState">Idle</span></div>
        </div>
      </section>
      <section class="readout settle-readout">
        <div class="phase" id="phase">Loading</div>
      </section>
      <section class="command settle-command">
        __ANTI_SWAY_SELECT__
        <div class="settle-actions">
          <button class="btn-settle primary" id="btnStartSettle" type="button">开始消摆</button>
          <button class="btn-settle" id="btnStopSettle" type="button" disabled>停止</button>
        </div>
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
        <div class="tab-panel tab-control-compact" id="tab-control">
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
    .shell.settle-focus .topbar.settle-topbar {
      pointer-events: none;
      padding: 10px 12px;
      gap: 10px;
    }
    .shell.settle-focus .topbar.settle-topbar h1 {
      display: flex;
      align-items: baseline;
      gap: 10px;
      margin: 0;
      font-size: 18px;
      line-height: 1.2;
      white-space: nowrap;
    }
    .shell.settle-focus .stage-sub {
      font-size: 13px;
      font-weight: 600;
      color: var(--muted);
    }
    .shell.settle-focus {
      height: 100vh;
      max-height: 100vh;
      min-height: 0;
      overflow: hidden;
      align-items: stretch;
      padding: 12px;
      gap: 12px;
    }
    .shell.settle-focus .stage.settle-stage-layout {
      position: relative;
      min-height: 0 !important;
      display: grid;
      grid-template-columns: minmax(0, 1fr);
      grid-template-rows: minmax(0, 1fr);
      gap: 0;
    }
    .shell.settle-focus .stage.settle-stage-layout.settle-with-residual {
      grid-template-columns: minmax(0, 1fr) clamp(300px, 36%, 440px);
    }
    .shell.settle-focus .stage.settle-stage-layout #scene {
      grid-column: 1;
      grid-row: 1;
      min-width: 0;
    }
    .settle-stage-rail {
      display: none;
      grid-column: 2;
      grid-row: 1;
      z-index: 2;
      min-width: 0;
      min-height: 0;
      flex-direction: column;
      gap: 10px;
      padding: 10px 10px 12px;
      border-left: 1.5px solid #3b5a73;
      background: rgba(6, 12, 18, 0.96);
      pointer-events: none;
      overflow: hidden;
    }
    .shell.settle-focus .stage.settle-with-residual .settle-stage-rail {
      display: flex;
    }
    .settle-stage-rail[hidden] { display: none !important; }
    .settle-stage-hud {
      flex: 0 0 auto;
      display: flex;
      flex-direction: column;
      gap: 8px;
      padding: 10px 12px 9px;
      height: clamp(248px, 30vh, 302px);
      min-height: 232px;
      max-height: 318px;
      border: 1.5px solid #3b5a73;
      border-radius: 10px;
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.04);
      background: linear-gradient(
        165deg,
        rgba(10, 18, 26, 0.98) 0%,
        rgba(8, 14, 20, 0.88) 100%
      );
    }
    .settle-stage-hud-head {
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      gap: 8px;
    }
    .settle-stage-hud-title {
      font-size: 13px;
      font-weight: 700;
      color: #8a9bab;
    }
    .settle-stage-hud-phase {
      font-size: 11px;
      font-weight: 600;
      color: #f0a83b;
      text-align: right;
    }
    .settle-stage-hud-peak-block { margin-top: 2px; }
    .settle-stage-hud-peak-val {
      font-size: 30px;
      font-weight: 700;
      line-height: 1.05;
      font-variant-numeric: tabular-nums;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      color: #91a3b3;
    }
    .settle-stage-hud-peak-lbl {
      font-size: 11px;
      font-weight: 600;
      color: #6b7c8c;
      margin-top: 2px;
    }
    .settle-stage-hud-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px 12px;
      margin-top: 4px;
    }
    .settle-stage-hud-cell {
      display: flex;
      flex-direction: column;
      gap: 2px;
      min-width: 0;
    }
    .settle-stage-hud-cell .lbl {
      font-size: 11px;
      font-weight: 700;
    }
    .settle-stage-hud-cell .lbl.theta-x { color: #59a7e8; }
    .settle-stage-hud-cell .lbl.theta-y { color: #f0a83b; }
    .settle-stage-hud-cell .lbl.ac { color: #8fd3ff; }
    .settle-stage-hud-cell .lbl.init { color: #c878ff; }
    .settle-stage-hud-cell .val {
      font-size: 16px;
      font-weight: 700;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      font-variant-numeric: tabular-nums;
      color: #dce7ee;
    }
    .settle-stage-hud-cell .val.init { color: #c878ff; }
    .settle-stage-hud-foot {
      margin-top: auto;
      display: flex;
      justify-content: space-between;
      gap: 10px;
      font-size: 11px;
      font-weight: 600;
      color: #6b7c8c;
      padding-top: 6px;
    }
    .settle-residual-widget {
      flex: 1;
      min-height: 0;
      display: flex;
      flex-direction: column;
      width: 100%;
      border: 1.5px solid #3b5a73;
      border-radius: 10px;
      background: rgba(4, 10, 16, 0.55);
      overflow: hidden;
    }
    .settle-residual-widget-title {
      margin: 0;
      padding: 10px 14px 8px;
      font-size: 14px;
      font-weight: 700;
      color: #dce7ee;
      user-select: none;
      border-bottom: 1px solid rgba(59, 90, 115, 0.55);
      flex-shrink: 0;
    }
    .settle-residual-widget-body {
      flex: 1;
      min-height: 0;
      height: 100%;
      padding: 6px 10px 4px;
    }
    .settle-residual-widget-body canvas {
      display: block;
      width: 100%;
      height: 100%;
      border-radius: 6px;
    }
    .settle-residual-widget-foot {
      flex-shrink: 0;
      padding: 3px 12px 8px;
      font-size: 11px;
      font-weight: 600;
      color: #8a9bab;
      text-align: right;
    }
    .shell.settle-focus .stage.settle-stage-layout canvas {
      min-height: 0 !important;
      width: 100%;
      height: 100%;
    }
    .shell.settle-focus .topbar.settle-topbar {
      position: absolute;
      top: 0;
      left: 0;
      right: 0;
      z-index: 8;
    }
    .shell.settle-focus .stage.settle-with-residual .topbar.settle-topbar {
      right: clamp(300px, 36%, 440px);
      border-right: 1px solid rgba(59, 90, 115, 0.35);
    }
    .shell.settle-focus .side.settle-side {
      display: flex;
      flex-direction: column;
      min-height: 0;
      max-height: 100%;
      overflow: hidden;
      padding: 0;
    }
    .shell.settle-focus .settle-side-top {
      flex-shrink: 0;
      border-bottom: 1px solid var(--line);
      background: linear-gradient(180deg, rgba(18, 28, 37, 0.65), rgba(12, 18, 24, 0.2));
    }
    .shell.settle-focus .settle-kpi-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      padding: 12px 12px 10px;
    }
    .shell.settle-focus .settle-kpi {
      padding: 10px 10px 9px;
      border-radius: 8px;
      border: 1px solid var(--line);
      background: rgba(8, 14, 20, 0.55);
      min-width: 0;
    }
    .shell.settle-focus .settle-kpi-elapsed {
      border-color: rgba(89, 167, 232, 0.35);
    }
    .shell.settle-focus .settle-kpi-initial {
      border-color: rgba(200, 120, 255, 0.35);
    }
    .shell.settle-focus .settle-kpi-label {
      font-size: 11px;
      font-weight: 600;
      color: var(--muted);
      margin-bottom: 4px;
    }
    .shell.settle-focus .settle-kpi-value {
      font-size: 26px;
      font-weight: 700;
      line-height: 1.1;
      font-variant-numeric: tabular-nums;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      letter-spacing: -0.02em;
    }
    .shell.settle-focus .settle-kpi-elapsed .settle-kpi-value { color: #59a7e8; }
    .shell.settle-focus .settle-kpi-initial .settle-kpi-value { color: #c878ff; }
    .shell.settle-focus .settle-meta-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      padding: 0 12px 10px;
      font-size: 12px;
    }
    .shell.settle-focus .settle-meta-item {
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      gap: 8px;
      min-width: 0;
    }
    .shell.settle-focus .settle-meta-label {
      color: var(--muted);
      font-size: 11px;
      flex-shrink: 0;
    }
    .shell.settle-focus .settle-meta-val {
      font-weight: 600;
      text-align: right;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .shell.settle-focus .settle-readout {
      padding: 8px 12px;
      flex-shrink: 0;
      border-bottom: 1px solid var(--line);
    }
    .shell.settle-focus .settle-readout .phase {
      font-size: 13px;
      line-height: 1.35;
      margin: 0;
    }
    .shell.settle-focus .settle-command {
      padding: 10px 12px 8px;
      flex-shrink: 0;
      border-bottom: 1px solid var(--line);
    }
    .shell.settle-focus .settle-command .anti-sway-label {
      font-size: 11px;
    }
    .shell.settle-focus .settle-command .anti-sway-select {
      padding: 7px 9px;
      font-size: 12px;
    }
    .shell.settle-focus .settle-command .anti-sway-row {
      margin-bottom: 6px;
    }
    .shell.settle-focus .settle-command .anti-sway-hint {
      font-size: 11px;
      line-height: 1.35;
      max-height: 2.7em;
      overflow: hidden;
      margin-bottom: 6px;
    }
    .shell.settle-focus .settle-actions {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
    }
    .shell.settle-focus .settle-actions .btn-settle {
      margin: 0;
      padding: 9px 8px;
      font-size: 13px;
    }
    .shell.settle-focus .bottom-panel {
      flex: 1;
      min-height: 0;
      margin-top: 0;
      display: flex;
      flex-direction: column;
      overflow: hidden;
      border: none;
      border-radius: 0;
    }
    .shell.settle-focus .tab-bar {
      flex-shrink: 0;
      padding: 0 8px;
    }
    .shell.settle-focus .tab-panel {
      flex: 1;
      min-height: 0;
      overflow: auto;
      padding: 8px 10px 10px;
    }
    .shell.settle-focus .loco-status {
      margin-bottom: 6px;
      font-size: 11px;
    }
    .shell.settle-focus .loco-table {
      gap: 4px 8px;
      font-size: 12px;
    }
    .inclinometer-deepseek {
      margin-top: 8px;
      padding-top: 6px;
      border-top: 1px solid var(--line);
    }
    .inclinometer-deepseek[hidden] { display: none; }
    .inclinometer-deepseek-title {
      font-size: 10px;
      font-weight: 700;
      color: #8fd3ff;
      margin-bottom: 4px;
    }
    .inclinometer-deepseek-table { margin-top: 0; }
    .shell.settle-focus .tab-control-compact .plc-status-bar {
      margin-bottom: 6px;
    }
    .shell.settle-focus .tab-control-compact .ctrl-grid-2x4 {
      gap: 4px;
      font-size: 12px;
    }
    .shell.settle-focus .tab-control-compact .btn-row {
      margin-top: 8px;
      gap: 8px;
    }
    .shell.settle-focus .tab-control-compact .btn-stop,
    .shell.settle-focus .tab-control-compact .btn-reset {
      padding: 8px;
      font-size: 12px;
    }
"""


_SETTLE_CANVAS_HELPERS = """
    function drawSettlePanel(box) {
      ctx.fillStyle = '#121c25';
      ctx.fillRect(box.x, box.y, box.w, box.h);
      ctx.strokeStyle = '#3b4e60';
      ctx.lineWidth = 1;
      ctx.strokeRect(box.x + 0.5, box.y + 0.5, box.w - 1, box.h - 1);
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
    var _settleResidualLayoutOn = null;
    function syncSettleStageLayout() {
      var stage = document.querySelector('.stage.settle-stage-layout');
      if (!stage) return;
      var show = typeof getAntiSwayMode === 'function'
        && getAntiSwayMode() === 'deepseek_anti';
      stage.classList.toggle('settle-with-residual', show);
      if (settleStageRail) settleStageRail.hidden = !show;
      if (_settleResidualLayoutOn !== show) {
        _settleResidualLayoutOn = show;
        requestAnimationFrame(function() { resize(); });
      }
    }

    function getSettleLayout(box) {
      var pad = 10;
      return {
        crossBox: {
          x: box.x + pad,
          y: box.y + 6,
          w: Math.max(120, box.w - pad * 2),
          h: Math.max(120, box.h - 12),
        },
      };
    }

  function residualRingPalette() {
    return [
      { hi: '#3ddc84', lo: 'rgba(61, 220, 132, 0.18)' },
      { hi: '#5ebd72', lo: 'rgba(94, 189, 114, 0.16)' },
      { hi: '#9ccc65', lo: 'rgba(156, 204, 101, 0.16)' },
      { hi: '#f0d04a', lo: 'rgba(240, 208, 74, 0.16)' },
      { hi: '#f0a83b', lo: 'rgba(240, 168, 59, 0.16)' },
      { hi: '#f08a4b', lo: 'rgba(240, 138, 75, 0.16)' },
      { hi: '#e86850', lo: 'rgba(232, 104, 80, 0.16)' },
      { hi: '#e05a47', lo: 'rgba(224, 90, 71, 0.16)' },
    ];
  }

    function drawSettleHookSway(box) {
      drawSettlePanel(box);
      var lay = getSettleLayout(box);
      var sway = _liveSwayState;
      var tol = (sway && sway.tol_deg != null) ? sway.tol_deg : SWAY_TOL_DEG;
      var valid = !!(sway && sway.valid);
      var thetaX = valid ? sway.theta_x_deg : null;
      var thetaY = valid ? sway.theta_y_deg : null;
      var pad = 8;
      var crossArea = {
        x: lay.crossBox.x + pad,
        y: lay.crossBox.y + pad,
        w: Math.max(60, lay.crossBox.w - pad * 2),
        h: Math.max(60, lay.crossBox.h - pad * 2),
      };
      drawSwayCross(crossArea, thetaX, thetaY, tol);
    }

    function swayCrossGeometry(box) {
      var lay = getSettleLayout(box);
      var pad = 8;
      var area = {
        x: lay.crossBox.x + pad,
        y: lay.crossBox.y + pad,
        w: Math.max(60, lay.crossBox.w - pad * 2),
        h: Math.max(60, lay.crossBox.h - pad * 2),
      };
      var leftGutter = Math.min(44, Math.max(32, area.w * 0.12));
      var topGutter = 18;
      var rightGutter = 22;
      var bottomGutter = Math.min(24, Math.max(18, area.h * 0.1));
      var plotW = Math.max(40, area.w - leftGutter - rightGutter);
      var plotH = Math.max(40, area.h - topGutter - bottomGutter);
      return {
        area: area,
        cx: area.x + leftGutter + plotW / 2,
        cy: area.y + topGutter + plotH / 2,
        span: Math.max(20, Math.min(plotW, plotH) / 2 - 6),
      };
    }

    function drawSwayCallout(text, px, py, color, align) {
      align = align || 'left';
      ctx.font = '700 12px system-ui, sans-serif';
      var padX = 6;
      var padY = 4;
      var w = ctx.measureText(text).width + padX * 2;
      var h = 16 + padY * 2;
      var left = align === 'right' ? px - w : px;
      var top = py - h / 2;
      ctx.fillStyle = 'rgba(6, 14, 20, 0.92)';
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.fillRect(left, top, w, h);
      ctx.strokeRect(left + 0.5, top + 0.5, w - 1, h - 1);
      ctx.fillStyle = color;
      ctx.textAlign = align;
      ctx.textBaseline = 'middle';
      ctx.fillText(text, align === 'right' ? px - padX : px + padX, py);
    }

    /** 主靶标：彩色圆点 = 当前摆角；红描边菱形 = 预测静态姿态 θ*。 */
    function drawStaticPoseOverlay(box) {
      if (typeof getAntiSwayMode !== 'function') return;
      if (getAntiSwayMode() !== 'deepseek_anti') return;
      var g = swayCrossGeometry(box);
      var d = (typeof _staticAngleDebug !== 'undefined') ? _staticAngleDebug : null;
      var sway = _liveSwayState;
      if (!d) return;
      var staticAccent = '#e05a47';
      var sxDeg = d.theta_static_y_deg || 0;
      var syDeg = d.theta_static_x_deg || 0;
      var tx = Math.max(-1, Math.min(1, sxDeg));
      var ty = Math.max(-1, Math.min(1, syDeg));
      var px = g.cx + tx * g.span;
      var py = g.cy - ty * g.span;

      ctx.save();
      ctx.beginPath();
      ctx.rect(g.area.x, g.area.y, g.area.w, g.area.h);
      ctx.clip();

      ctx.setLineDash([5, 4]);
      ctx.strokeStyle = 'rgba(224, 90, 71, 0.55)';
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      ctx.moveTo(g.cx, g.cy);
      ctx.lineTo(px, py);
      ctx.stroke();
      ctx.setLineDash([]);

      var r = 9;
      ctx.beginPath();
      ctx.moveTo(px, py - r);
      ctx.lineTo(px + r, py);
      ctx.lineTo(px, py + r);
      ctx.lineTo(px - r, py);
      ctx.closePath();
      ctx.fillStyle = 'rgba(224, 90, 71, 0.28)';
      ctx.fill();
      ctx.lineWidth = 2.5;
      ctx.strokeStyle = staticAccent;
      ctx.stroke();

      var lx = px + 14;
      var ly = py - 12;
      if (lx + 110 > g.area.x + g.area.w) {
        lx = px - 14;
        drawSwayCallout('预测静态姿态 θ*', lx, ly, staticAccent, 'right');
      } else {
        drawSwayCallout('预测静态姿态 θ*', lx, ly, staticAccent, 'left');
      }

      if (sway && sway.valid) {
        var curTy = Math.max(-1, Math.min(1, sway.theta_y_deg));
        var curTx = Math.max(-1, Math.min(1, sway.theta_x_deg));
        var cpx = g.cx + curTy * g.span;
        var cpy = g.cy - curTx * g.span;
        var peak = sway.theta_peak_deg != null ? sway.theta_peak_deg : 0;
        var tol = sway.tol_deg != null ? sway.tol_deg : SWAY_TOL_DEG;
        var curCol = swayAngleColor(peak, tol);
        var clx = cpx + 12;
        var cly = cpy + 14;
        if (clx + 72 > g.area.x + g.area.w) {
          drawSwayCallout('当前摆角', cpx - 12, cly, curCol, 'right');
        } else {
          drawSwayCallout('当前摆角', clx, cly, curCol, 'left');
        }
      }
      ctx.restore();
    }

    function settleResidualAc(d, sway) {
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
      return { acX: acX, acY: acY };
    }

    /** 迷你十字靶：0.1°～0.8° 色环，残余到达的格点亮。 */
    function drawResidualMiniCross(plot, acXDeg, acYDeg, tol, opts) {
      opts = opts || {};
      var ready = !!opts.ready;
      var drawCtx = opts.drawCtx || ctx;
      var dotFn = opts.dotFn || dot;
      if (opts.squarePlot) {
        var side = Math.min(plot.w, plot.h);
        plot = {
          x: plot.x + (plot.w - side) / 2,
          y: plot.y + (plot.h - side) / 2,
          w: side,
          h: side,
        };
      }
      var hasAc = acXDeg != null && acYDeg != null && isFinite(acXDeg) && isFinite(acYDeg);
      var peak = hasAc ? Math.max(Math.abs(acXDeg), Math.abs(acYDeg)) : 0;
      var reach = hasAc ? Math.hypot(acXDeg, acYDeg) : 0;
      var fullScaleDeg = opts.fullScaleDeg != null ? opts.fullScaleDeg : 0.8;
      if (!opts.fullScaleDeg) {
        var peakNeed = Math.max(peak, tol * 2, 0.12);
        fullScaleDeg = 0.3;
        if (peakNeed > 0.28) fullScaleDeg = 0.5;
        if (peakNeed > 0.45) fullScaleDeg = 0.8;
        if (peakNeed > 0.72) fullScaleDeg = 1.0;
      }

      var labelColW = Math.max(48, Math.min(56, plot.w * 0.16));
      var topGutter = 20;
      var bottomGutter = 16;
      var rightGutter = 12;
      var plotW = Math.max(80, plot.w - labelColW - rightGutter);
      var plotH = Math.max(80, plot.h - topGutter - bottomGutter);
      var span = Math.max(32, Math.min(plotW, plotH) / 2 - 8);
      var plotCx = plot.x + labelColW + plotW / 2;
      var plotCy = plot.y + topGutter + plotH / 2;
      var pxPerDeg = span / fullScaleDeg;
      var toPx = function(deg) {
        return Math.max(-span, Math.min(span, deg * pxPerDeg));
      };
      var palette = residualRingPalette();

      drawCtx.fillStyle = '#0a1016';
      drawCtx.fillRect(plot.x, plot.y, plot.w, plot.h);

      drawCtx.save();
      drawCtx.beginPath();
      drawCtx.rect(plot.x, plot.y, plot.w, plot.h);
      drawCtx.clip();

      var ringDegs = [];
      var maxRing = Math.min(fullScaleDeg, 0.8);
      for (var rd = 0.1; rd <= maxRing + 1e-6; rd += 0.1) {
        ringDegs.push(Math.round(rd * 10) / 10);
      }

      if (hasAc && reach > 0.02) {
        for (var fi = 0; fi < ringDegs.length; fi++) {
          var fd = ringDegs[fi];
          var prevD = fi === 0 ? 0 : ringDegs[fi - 1];
          if (reach <= prevD + 1e-6) break;
          var bandEnd = Math.min(reach, fd);
          var pal = palette[Math.min(fi, palette.length - 1)];
          var r0 = prevD * pxPerDeg;
          var r1 = bandEnd * pxPerDeg;
          if (r1 - r0 < 0.5) continue;
          drawCtx.fillStyle = pal.hi + '28';
          drawCtx.beginPath();
          drawCtx.arc(plotCx, plotCy, r1, 0, Math.PI * 2);
          drawCtx.arc(plotCx, plotCy, r0, 0, Math.PI * 2, true);
          drawCtx.fill();
          if (reach <= fd + 1e-6) break;
        }
      }

      for (var ri = 0; ri < ringDegs.length; ri++) {
        var rd2 = ringDegs[ri];
        var rp = rd2 * pxPerDeg;
        if (rp < 2) continue;
        var pal2 = palette[Math.min(ri, palette.length - 1)];
        var lit = hasAc && reach + 1e-6 >= rd2;
        drawCtx.strokeStyle = lit ? pal2.hi : pal2.lo;
        drawCtx.setLineDash(lit ? [] : [3, 4]);
        drawCtx.lineWidth = lit ? 2.2 : 1.1;
        if (lit && ri === ringDegs.length - 1) drawCtx.lineWidth = 2.4;
        drawCtx.beginPath();
        drawCtx.arc(plotCx, plotCy, rp, 0, Math.PI * 2);
        drawCtx.stroke();
      }
      drawCtx.setLineDash([]);

      drawCtx.strokeStyle = 'rgba(142, 163, 184, 0.45)';
      drawCtx.lineWidth = 1.2;
      drawCtx.beginPath();
      drawCtx.moveTo(plotCx - span - 2, plotCy);
      drawCtx.lineTo(plotCx + span + 2, plotCy);
      drawCtx.moveTo(plotCx, plotCy + span + 2);
      drawCtx.lineTo(plotCx, plotCy - span - 2);
      drawCtx.stroke();

      var labelLaneR = plot.x + labelColW - 6;
      var minLabelGap = 14;
      var lastLabelY = 1e9;
      for (var ti = 0; ti < ringDegs.length; ti++) {
        var td = ringDegs[ti];
        var tp = td * pxPerDeg;
        if (tp < 3 || tp > span + 1) continue;
        var pal3 = palette[Math.min(ti, palette.length - 1)];
        var lit2 = hasAc && reach + 1e-6 >= td;
        drawCtx.strokeStyle = lit2 ? pal3.hi : pal3.lo;
        drawCtx.lineWidth = lit2 ? 1.5 : 0.9;
        drawCtx.beginPath();
        drawCtx.moveTo(plotCx - tp, plotCy - 4);
        drawCtx.lineTo(plotCx - tp, plotCy + 4);
        drawCtx.moveTo(plotCx + tp, plotCy - 4);
        drawCtx.lineTo(plotCx + tp, plotCy + 4);
        drawCtx.moveTo(plotCx - 4, plotCy - tp);
        drawCtx.lineTo(plotCx + 4, plotCy - tp);
        drawCtx.stroke();

        var labelY = plotCy - tp;
        var major = Math.abs(Math.round(td * 10) % 2) === 0 || td >= 0.79;
        var showLabel = major || lit2;
        if (!showLabel || Math.abs(labelY - lastLabelY) < minLabelGap) continue;

        var tcol = lit2 ? pal3.hi : '#6a7c8d';
        var ttext = td.toFixed(1) + '°';
        drawCtx.font = lit2 ? '700 11px ui-monospace, Menlo, monospace'
          : '600 10px ui-monospace, Menlo, monospace';
        drawCtx.textAlign = 'right';
        drawCtx.textBaseline = 'middle';
        var tw = drawCtx.measureText(ttext).width;
        drawCtx.fillStyle = 'rgba(6, 10, 14, 0.92)';
        drawCtx.fillRect(labelLaneR - tw - 8, labelY - 8, tw + 8, 16);
        drawCtx.fillStyle = tcol;
        drawCtx.fillText(ttext, labelLaneR, labelY);
        lastLabelY = labelY;
      }

      drawCtx.font = '600 9px system-ui, sans-serif';
      drawCtx.fillStyle = '#6b7c8c';
      drawCtx.textAlign = 'right';
      drawCtx.textBaseline = 'bottom';
      drawCtx.fillText('|θ|°', labelLaneR, plot.y + plot.h - 4);

      drawCtx.fillStyle = 'rgba(224, 90, 71, 0.38)';
      drawCtx.beginPath();
      drawCtx.arc(plotCx, plotCy, 5, 0, Math.PI * 2);
      drawCtx.fill();
      drawCtx.strokeStyle = '#e05a47';
      drawCtx.lineWidth = 1.8;
      drawCtx.stroke();

      if (hasAc) {
        var px = plotCx + toPx(acYDeg);
        var py = plotCy - toPx(acXDeg);
        var litIdx = Math.min(
          ringDegs.length - 1,
          Math.max(0, Math.ceil(reach * 10) - 1),
        );
        var col = palette[litIdx] ? palette[litIdx].hi : swayAngleColor(peak, tol);
        drawCtx.setLineDash([4, 3]);
        drawCtx.strokeStyle = col;
        drawCtx.globalAlpha = 0.72;
        drawCtx.lineWidth = 1.4;
        drawCtx.beginPath();
        drawCtx.moveTo(plotCx, plotCy);
        drawCtx.lineTo(px, py);
        drawCtx.stroke();
        drawCtx.globalAlpha = 1;
        drawCtx.setLineDash([]);
        dotFn({ x: px, y: py }, col, 7, drawCtx);
      }

      drawCtx.font = '700 11px system-ui, sans-serif';
      drawCtx.fillStyle = '#59a7e8';
      drawCtx.textAlign = 'center';
      drawCtx.textBaseline = 'bottom';
      drawCtx.fillText('θx', plotCx, plotCy - span - 6);
      drawCtx.fillStyle = '#f0a83b';
      drawCtx.textAlign = 'left';
      drawCtx.fillText('θy', plotCx + span + 6, plotCy + 4);

      if (hasAc) {
        drawCtx.textAlign = 'right';
        drawCtx.textBaseline = 'top';
        drawCtx.font = '700 12px ui-monospace, Menlo, monospace';
        drawCtx.fillStyle = '#dce7ee';
        drawCtx.fillText(
          '|θ_ac| ' + reach.toFixed(2) + '°',
          plot.x + plot.w - 6,
          plot.y + 4,
        );
      } else if (!ready) {
        drawCtx.textAlign = 'center';
        drawCtx.textBaseline = 'middle';
        drawCtx.font = '600 11px system-ui, sans-serif';
        drawCtx.fillStyle = '#6b7c8c';
        drawCtx.fillText('等待 θ_ac', plotCx, plotCy);
      }

      drawCtx.restore();
    }

    function resizeResidualWidgetCanvas() {
      if (!residualCanvas || !residualCtx) return;
      var rect = residualCanvas.getBoundingClientRect();
      var ratio = window.devicePixelRatio || 1;
      var w = Math.max(1, Math.floor(rect.width * ratio));
      var h = Math.max(1, Math.floor(rect.height * ratio));
      if (residualCanvas.width !== w || residualCanvas.height !== h) {
        residualCanvas.width = w;
        residualCanvas.height = h;
        residualCtx.setTransform(ratio, 0, 0, ratio, 0, 0);
      }
    }

    function dotResidual(p, color, size, drawCtx) {
      size = size || 8;
      drawCtx.beginPath();
      drawCtx.arc(p.x, p.y, size, 0, Math.PI * 2);
      drawCtx.fillStyle = color;
      drawCtx.fill();
      drawCtx.lineWidth = 2;
      drawCtx.strokeStyle = '#ffffff';
      drawCtx.stroke();
    }

    function drawStaticResidualOverlay(box) {
      syncSettleStageLayout();
      if (typeof updateStageHud === 'function') updateStageHud();
      if (typeof getAntiSwayMode !== 'function') return;
      var isDeepseek = getAntiSwayMode() === 'deepseek_anti';
      if (!isDeepseek || !residualCanvas || !residualCtx) return;

      var sway = _liveSwayState;
      var tol = (sway && sway.tol_deg != null) ? sway.tol_deg : SWAY_TOL_DEG;
      var d = (typeof _staticAngleDebug !== 'undefined') ? _staticAngleDebug : null;
      var ready = !!(d && d.ready_x && d.ready_y);
      var ac = settleResidualAc(d, sway);
      var peakReach = (ac.acX != null && ac.acY != null)
        ? Math.hypot(ac.acX, ac.acY) : 0;

      if (residualReadyHint) {
        var hint = !d ? '等待 deepseek 调试数据'
          : (ready ? 'θ* 已收敛' : 'θ* 估计中');
        if (ac.acX != null && ac.acY != null) {
          hint += ' · |θ_ac| ' + peakReach.toFixed(2) + '°';
        }
        residualReadyHint.textContent = hint;
        residualReadyHint.style.color = ready ? '#5ebd72' : '#f0a83b';
      }

      resizeResidualWidgetCanvas();
      var rect = residualCanvas.getBoundingClientRect();
      residualCtx.clearRect(0, 0, rect.width, rect.height);
      drawResidualMiniCross(
        { x: 0, y: 0, w: rect.width, h: rect.height },
        ac.acX,
        ac.acY,
        tol,
        {
          ready: ready,
          fullScaleDeg: 0.8,
          drawCtx: residualCtx,
          dotFn: dotResidual,
          squarePlot: true,
        },
      );
    }

    function draw() {
      var rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);
      ctx.fillStyle = '#060a0e';
      ctx.fillRect(0, 0, rect.width, rect.height);
      var topbarClear = 56;
      var box = {
        x: 14,
        y: topbarClear,
        w: Math.max(200, rect.width - 28),
        h: Math.max(280, rect.height - topbarClear - 10),
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

      ctx.restore();
    }
"""

_SETTLE_BOOT = """
    function markRunLogReady() {}
    function resetRunLogReady() {}

    const canvas = document.getElementById('scene');
    const ctx = canvas.getContext('2d');
    const settleStageRail = document.getElementById('settleStageRail');
    const residualWidget = document.getElementById('settleResidualWidget');
    const residualCanvas = document.getElementById('settleResidualCanvas');
    const residualCtx = residualCanvas ? residualCanvas.getContext('2d') : null;
    const residualReadyHint = document.getElementById('settleResidualReadyHint');
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
      stageHudPeak: document.getElementById('stageHudPeak'),
      stageHudThetaX: document.getElementById('stageHudThetaX'),
      stageHudThetaY: document.getElementById('stageHudThetaY'),
      stageHudAcPeak: document.getElementById('stageHudAcPeak'),
      stageHudInitPeak: document.getElementById('stageHudInitPeak'),
      stageHudReady: document.getElementById('stageHudReady'),
      stageHudElapsed: document.getElementById('stageHudElapsed'),
      stageHudTol: document.getElementById('stageHudTol'),
    };

    let _staticAngleDebug = null;

  function updateStageHud() {
    if (typeof getAntiSwayMode !== 'function' || getAntiSwayMode() !== 'deepseek_anti') {
      return;
    }
    var sway = _liveSwayState;
    var tol = (sway && sway.tol_deg != null) ? sway.tol_deg : SWAY_TOL_DEG;
    var d = _staticAngleDebug;
    var ready = !!(d && d.ready_x && d.ready_y);
    var peakNow = (sway && sway.valid && sway.theta_peak_deg != null)
      ? sway.theta_peak_deg : null;
    if (els.stageHudPeak) {
      els.stageHudPeak.textContent = peakNow != null ? peakNow.toFixed(2) : '--';
      els.stageHudPeak.style.color = peakNow != null
        ? swayAngleColor(peakNow, tol) : '#91a3b3';
    }
    if (els.stageHudThetaX) {
      els.stageHudThetaX.textContent = (sway && sway.valid && sway.theta_x_deg != null)
        ? sway.theta_x_deg.toFixed(2) + '°' : '--';
    }
    if (els.stageHudThetaY) {
      els.stageHudThetaY.textContent = (sway && sway.valid && sway.theta_y_deg != null)
        ? sway.theta_y_deg.toFixed(2) + '°' : '--';
    }
    var acPeak = null;
    if (d && (d.theta_ac_x_deg != null || d.theta_ac_y_deg != null)) {
      var ax = d.theta_ac_x_deg;
      var ay = d.theta_ac_y_deg;
      if (sway && sway.valid) {
        if (ax == null) ax = (sway.theta_x_deg || 0) - (d.theta_static_x_deg || 0);
        if (ay == null) ay = (sway.theta_y_deg || 0) - (d.theta_static_y_deg || 0);
      }
      if (ax != null && ay != null && isFinite(ax) && isFinite(ay)) {
        acPeak = Math.hypot(ax, ay);
      }
    }
    if (els.stageHudAcPeak) {
      els.stageHudAcPeak.textContent = acPeak != null ? acPeak.toFixed(3) + '°' : '--';
      els.stageHudAcPeak.style.color = acPeak != null
        ? swayAngleColor(acPeak, tol) : '#dce7ee';
    }
    if (els.stageHudInitPeak) {
      var b = (typeof _settleBaseline !== 'undefined') ? _settleBaseline : null;
      if (b && b.peakDeg > 0) {
        els.stageHudInitPeak.textContent = b.peakDeg.toFixed(2) + '°';
      } else if (_settleActive) {
        els.stageHudInitPeak.textContent = '采样中';
      } else {
        els.stageHudInitPeak.textContent = '--';
      }
    }
    if (els.stageHudReady) {
      if (!d) {
        els.stageHudReady.textContent = '等待数据';
        els.stageHudReady.style.color = '#8a9bab';
      } else if (ready) {
        els.stageHudReady.textContent = 'θ* 已收敛';
        els.stageHudReady.style.color = '#5ebd72';
      } else {
        els.stageHudReady.textContent = 'θ* 估计中';
        els.stageHudReady.style.color = '#f0a83b';
      }
    }
    if (els.stageHudElapsed) {
      if (_settleElapsedS != null && (_settleActive || _settleT0Ms != null)) {
        els.stageHudElapsed.textContent = '计时 ' + _settleElapsedS.toFixed(1) + ' s';
        els.stageHudElapsed.style.color = _settleActive ? '#59a7e8' : '#5ebd72';
      } else {
        els.stageHudElapsed.textContent = '计时 —';
        els.stageHudElapsed.style.color = '#6b7c8c';
      }
    }
    if (els.stageHudTol) {
      els.stageHudTol.textContent = '容差 ' + tol.toFixed(2) + '°';
    }
    }

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
    }

    function getAntiSwayMode() {
      if (!els.antiSwayMode) return 'deepseek_anti';
      return els.antiSwayMode.value || 'deepseek_anti';
    }

    function setAntiSwayMode(mode) {
      if (!els.antiSwayMode) return;
      const sel = els.antiSwayMode;
      const has = Array.from(sel.options).some(function(o) { return o.value === mode; });
      sel.value = has ? mode : 'deepseek_anti';
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
      syncSettleStageLayout();
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
      updateStageHud();
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
            anti_sway_select_markup(default_mode='deepseek_anti', scope='settle'),
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
        f'  <style>{LIVE_VIEW_BASE_CSS}{PTP_EXTRA_CSS}'
        f'{_SETTLE_EXTRA_CSS}</style>\n'
        '</head>\n'
        + page_body
        + '  <script>\n'
        + f'    const PLC_MODE = {plc_js};\n'
        + _SETTLE_BOOT
        + _SETTLE_CANVAS_HELPERS
        + PTP_SWAY_STAGE_JS
        + _SETTLE_DRAW
        + '  </script>\n'
        '</body>\n'
        '</html>'
    )
