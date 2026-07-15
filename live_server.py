"""Browser-based 10 Hz live view for the crane simulation.

Supports two rendering modes:

  Simulation (replay):
    Simulation runs to completion first, then the browser replays the
    recorded history at 10 Hz.  URL query parameters trigger new runs.

  PLC (real-time):
    The browser shows live /localization_pose data on open.  The user
    sets a target and clicks "Apply Target" to start PD control.
    A background thread runs the control loop while an SSE endpoint
    pushes real-time state to the browser Canvas.
"""

from __future__ import annotations

import json
import queue
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse

from crane_model import (
    ControlHooks,
    CraneConfig,
    CraneState,
    run_pd_control,
)
from ros_bridge import get_latest_pose, RosPositionSource
from plc_interface import PlcActuator
from visualizer import CraneVisualizer


def _point_tuple(point: tuple[float, float, float]) -> dict[str, float]:
    return {'x': point[0], 'y': point[1], 'z': point[2]}


def build_live_payload(
    history: list[dict],
    phase_boundaries: list[tuple[float, str]],
    target_pos: tuple[float, float, float],
    initial_pos: tuple[float, float, float],
    config,
    update_hz: float = 10.0,
    speed: float = 1.0,
) -> dict[str, Any]:
    """Build JSON-serializable data consumed by the browser live view."""
    if update_hz <= 0:
        raise ValueError('update_hz must be positive')
    if speed <= 0:
        raise ValueError('speed must be positive')
    if not history:
        raise ValueError('history must not be empty')

    frame_indices = CraneVisualizer(config).live_frame_indices(history, update_hz)
    frames = []
    for idx in frame_indices:
        item = history[idx]
        frames.append({
            't': round(item['t'], 3),
            'x': item['x'],
            'y': item['y'],
            'z': item['z'],
            'vx': item['vx'],
            'vy': item['vy'],
            'vz': item['vz'],
            'vxCmd': item.get('vx_cmd', item['vx']),
            'vyCmd': item.get('vy_cmd', item['vy']),
            'vzCmd': item.get('vz_cmd', item['vz']),
            'disturbanceX': item.get('disturbance_x', 0.0),
            'disturbanceY': item.get('disturbance_y', 0.0),
            'disturbanceZ': item.get('disturbance_z', 0.0),
            'phase': 'MOVE_TO_TARGET',
            'phaseLabel': 'Move to target',
        })

    xs = [item['x'] for item in history] + [target_pos[0], initial_pos[0]]
    ys = [item['y'] for item in history] + [target_pos[1], initial_pos[1]]
    zs = [item['z'] for item in history] + [target_pos[2], initial_pos[2]]

    return {
        'updateHz': float(update_hz),
        'speed': float(speed),
        'framePeriodMs': 1000.0 / float(update_hz) / float(speed),
        'target': _point_tuple(target_pos),
        'initial': _point_tuple(initial_pos),
        'bounds': {
            'xMin': min(xs),
            'xMax': max(xs),
            'yMin': min(ys),
            'yMax': max(ys),
            'zMin': min(zs),
            'zMax': max(zs),
        },
        'velocityLimits': {
            'xy': config.max_velocity_xy,
            'z': config.max_velocity_z,
        },
        'phaseBoundaries': [
            {'t': round(t, 3), 'axis': axis, 'label': f'{axis.upper()} arrived'}
            for t, axis in phase_boundaries
        ],
        'frames': frames,
    }


# ============================================================================
# PLC 实时控制钩子 — 通过队列将 PD 状态推送到 SSE
# ============================================================================

class LiveControlHooks(ControlHooks):
    """ControlHooks 实现 — 将每步 PD 状态写入 ControlState (供轮询) + 入队 (供 SSE)。"""

    def __init__(self, control_state: ControlState | None = None):
        self._queue: queue.Queue = queue.Queue()
        self._event = threading.Event()    # 信号: 队列有新数据, 唤醒 SSE 消费者
        self._stop_flag = threading.Event()
        self._control_state = control_state  # 轮询 API 用的共享状态

    def on_step(self, step_data: dict) -> None:
        """每步: 写 ControlState (供轮询) + 入队 (供 SSE)。"""
        # 写入轮询状态 (主要数据通道)
        if self._control_state is not None:
            self._control_state.set_step(step_data)
        # 入队供 SSE (诊断通道)
        try:
            self._queue.put_nowait({'type': 'step', 'data': step_data})
            self._event.set()
        except queue.Full:
            pass

    def on_arrival(self, axis: str, t: float) -> None:
        """到达事件: 写 ControlState + 入队。"""
        if self._control_state is not None:
            self._control_state.set_arrival(axis, t)
        try:
            self._queue.put_nowait({'type': 'arrival', 'axis': axis, 't': t})
            self._event.set()
        except queue.Full:
            pass

    def should_stop(self) -> bool:
        """检查外部停止信号。"""
        return self._stop_flag.is_set()

    def stop(self) -> None:
        """请求停止控制循环。"""
        self._stop_flag.set()

    def done(self) -> None:
        """发送完成信号。"""
        try:
            self._queue.put_nowait({'type': 'done'})
            self._event.set()
        except queue.Full:
            pass

    def send_error(self, message: str) -> None:
        """发送错误信号。"""
        try:
            self._queue.put_nowait({'type': 'error', 'message': message})
            self._event.set()
        except queue.Full:
            pass

    def get_event(self, timeout: float = 0.5) -> dict | None:
        """SSE 端点调用 — 阻塞等待下一个事件（Event 驱动，无轮询延迟）。

        与 queue.Queue.get(timeout) 不同，此方法用 threading.Event
        实现即时唤醒：一旦 on_step() 入队数据，get_event() 立即返回，
        而非等待最多 timeout 秒。
        """
        if self._event.wait(timeout=timeout):
            # 有数据到达
            try:
                event = self._queue.get_nowait()
                if self._queue.empty():
                    self._event.clear()
                return event
            except queue.Empty:
                return None
        return None


def render_live_html(plc_mode: bool = False) -> str:
    """Return the browser live view HTML.

    Args:
        plc_mode: True → PLC 实时控制模式, False → 仿真回放模式
    """
    html = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Crane Live Control</title>
  <style>
    :root {
      --bg: #111820;
      --panel: #17212b;
      --panel-2: #1e2a36;
      --line: #344657;
      --text: #eef4f8;
      --muted: #91a3b3;
      --green: #5ebd72;
      --blue: #59a7e8;
      --amber: #f0a83b;
      --red: #e05a47;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--text);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    .shell {
      display: grid;
      grid-template-columns: minmax(520px, 1.35fr) minmax(360px, 0.65fr);
      gap: 16px;
      min-height: 100vh;
      padding: 16px;
    }
    .stage, .side {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 6px;
      overflow: hidden;
    }
    .stage { position: relative; }
    canvas { display: block; width: 100%; height: 100%; min-height: 640px; }
    .topbar {
      position: absolute;
      top: 14px;
      left: 14px;
      right: 14px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      pointer-events: none;
    }
    h1 {
      margin: 0;
      font-size: 18px;
      font-weight: 700;
      letter-spacing: 0;
    }
    .badge {
      padding: 6px 9px;
      border: 1px solid #3e566a;
      border-radius: 4px;
      background: rgba(23, 33, 43, 0.86);
      color: var(--muted);
      font-size: 12px;
      white-space: nowrap;
    }
    .side {
      display: grid;
      grid-template-rows: auto auto auto 1fr;
      gap: 0;
    }
    .status {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      border-bottom: 1px solid var(--line);
    }
    .metric {
      padding: 14px;
      border-right: 1px solid var(--line);
      min-width: 0;
    }
    .metric:last-child { border-right: 0; }
    .metric .label {
      color: var(--muted);
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }
    .metric .value {
      margin-top: 5px;
      font-size: 24px;
      font-variant-numeric: tabular-nums;
    }
    .readout {
      padding: 14px;
      border-bottom: 1px solid var(--line);
      background: var(--panel-2);
    }
    .phase {
      font-size: 16px;
      font-weight: 700;
      color: var(--green);
    }
    .coords {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
      margin-top: 12px;
    }
    .coord {
      padding: 10px;
      border: 1px solid var(--line);
      border-radius: 4px;
      background: #131c25;
    }
    .coord span {
      display: block;
      color: var(--muted);
      font-size: 11px;
      margin-bottom: 4px;
    }
    .coord strong {
      font-size: 18px;
      font-variant-numeric: tabular-nums;
    }
    .velo-label {
      color: var(--muted);
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      margin-top: 14px;
      margin-bottom: 8px;
    }
    .command {
      padding: 14px;
      border-bottom: 1px solid var(--line);
      background: #141e27;
    }
    .command-title {
      color: var(--text);
      font-size: 13px;
      font-weight: 700;
      margin-bottom: 10px;
    }
    .target-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 8px;
    }
    .target-grid label {
      display: grid;
      gap: 4px;
      color: var(--muted);
      font-size: 11px;
    }
    .target-grid input {
      width: 100%;
      min-width: 0;
      border: 1px solid var(--line);
      border-radius: 4px;
      background: #0f161d;
      color: var(--text);
      padding: 8px 7px;
      font: inherit;
      font-variant-numeric: tabular-nums;
    }
    .target-grid input:focus {
      outline: 1px solid var(--blue);
      border-color: var(--blue);
    }
    .apply-target {
      width: 100%;
      margin-top: 10px;
      border: 1px solid #45637a;
      border-radius: 4px;
      background: #203040;
      color: var(--text);
      padding: 9px 10px;
      font: inherit;
      font-size: 13px;
      font-weight: 700;
      cursor: pointer;
    }
    .apply-target:hover { background: #283b4d; }
    .bottom-panel {
      padding: 0;
      display: grid;
      grid-template-rows: auto 1fr;
      overflow: hidden;
    }
    .tab-bar {
      display: flex;
      border-bottom: 1px solid var(--line);
      background: #0f161d;
    }
    .tab {
      padding: 10px 18px;
      border: 0;
      border-bottom: 2px solid transparent;
      background: transparent;
      color: var(--muted);
      font: inherit;
      font-size: 12px;
      font-weight: 600;
      cursor: pointer;
      transition: color 0.15s, border-color 0.15s;
    }
    .tab:hover { color: var(--text); }
    .tab.active {
      color: var(--amber);
      border-bottom-color: var(--amber);
    }
    .tab-panel { display: none; padding: 14px; overflow-y: auto; }
    .tab-panel.active { display: block; }
    .track {
      height: 16px;
      border-radius: 3px;
      background: #0f161d;
      border: 1px solid var(--line);
      overflow: hidden;
      position: relative;
    }
    .progress {
      height: 100%;
      width: 0;
      background: linear-gradient(90deg, var(--green), var(--amber));
    }
    .ticks {
      margin-top: 12px;
      display: grid;
      grid-template-columns: 1fr 1fr;
      color: var(--muted);
      font-size: 12px;
      font-variant-numeric: tabular-nums;
    }
    .ticks span:last-child { text-align: right; }
    .log {
      margin-top: 18px;
      display: grid;
      gap: 7px;
      color: var(--muted);
      font-size: 12px;
    }
    .log div {
      display: flex;
      justify-content: space-between;
      gap: 10px;
      border-bottom: 1px solid rgba(52, 70, 87, 0.55);
      padding-bottom: 7px;
    }
    .loco-status {
      font-size: 12px;
      font-weight: 600;
      margin-bottom: 12px;
      color: var(--muted);
    }
    .loco-table {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 1px;
      border: 1px solid var(--line);
      border-radius: 4px;
      overflow: hidden;
      background: var(--line);
    }
    .loco-table > div {
      padding: 8px 10px;
      background: #131c25;
    }
    .loco-hdr {
      color: var(--muted);
      font-size: 10px;
      text-transform: uppercase;
      letter-spacing: 0.06em;
      background: #0f161d;
    }
    .loco-axis {
      color: var(--amber);
      font-size: 12px;
      font-weight: 700;
      text-align: center;
    }
    .loco-val {
      font-size: 15px;
      font-variant-numeric: tabular-nums;
      color: var(--text);
      text-align: right;
    }
    .loco-timestamp {
      margin-top: 12px;
      color: var(--muted);
      font-size: 11px;
      font-variant-numeric: tabular-nums;
    }
    .plc-status-bar {
      display: flex;
      align-items: center;
      gap: 6px;
      margin-bottom: 12px;
      font-size: 12px;
      font-weight: 600;
      color: var(--text);
    }
    .plc-dot {
      width: 8px; height: 8px;
      border-radius: 50%;
      display: inline-block;
      background: var(--red);
      flex-shrink: 0;
    }
    .plc-dot.on { background: var(--green); animation: plc-blink 1.2s ease-in-out infinite; }
    @keyframes plc-blink {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.25; }
    }
    .plc-sep { color: var(--line); margin: 0 4px; font-weight: 400; }
    .ctrl-grid-2x4 {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 1px;
      border: 1px solid var(--line);
      border-radius: 4px;
      overflow: hidden;
      background: var(--line);
      margin-bottom: 10px;
    }
    .ctrl-grid-2x4 > div {
      padding: 8px 6px;
      background: #131c25;
      text-align: center;
    }
    .ctrl-hdr {
      font-size: 10px;
      text-transform: uppercase;
      letter-spacing: 0.06em;
      color: var(--amber);
      font-weight: 700;
      background: #0f161d !important;
    }
    .ctrl-val {
      font-size: 18px;
      font-variant-numeric: tabular-nums;
      font-weight: 700;
      color: var(--text);
      transition: color 0.3s, background 0.3s;
    }
    .ctrl-val.flash { color: #000; background: var(--amber); }
    .ctrl-val.gripper { font-size: 12px; color: var(--muted); font-style: italic; }
    .btn-row {
      display: flex;
      gap: 8px;
    }
    .btn-row .btn-stop { flex: 1; }
    .btn-row .btn-reset { flex: 1; }
    .ctrl-action-msg {
      margin-top: 12px;
      font-size: 12px;
      font-weight: 600;
      min-height: 18px;
    }
    .btn-stop {
      margin-top: 10px;
      padding: 12px 10px;
      border: 2px solid var(--red);
      border-radius: 4px;
      background: rgba(224, 90, 71, 0.18);
      color: var(--red);
      font: inherit;
      font-size: 14px;
      font-weight: 700;
      letter-spacing: 0.06em;
      cursor: pointer;
      transition: background 0.15s;
    }
    .btn-stop:hover { background: rgba(224, 90, 71, 0.35); }
    .btn-reset {
      margin-top: 10px;
      padding: 10px;
      border: 1px solid #45637a;
      border-radius: 4px;
      background: #203040;
      color: var(--text);
      font: inherit;
      font-size: 13px;
      font-weight: 600;
      cursor: pointer;
      transition: background 0.15s;
    }
    .btn-reset:hover { background: #283b4d; }
    @media (max-width: 980px) {
      .shell { grid-template-columns: 1fr; }
      canvas { min-height: 520px; }
    }
  </style>
</head>
<body>
  <main class="shell">
    <section class="stage">
      <canvas id="scene"></canvas>
      <div class="topbar">
        <h1>Crane Live Control</h1>
        <div class="badge" id="rate">10 Hz</div>
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
        <div class="command-title">Target Command</div>
        <div class="target-grid">
          <label>X <input id="targetX" type="number" step="0.1"></label>
          <label>Y <input id="targetY" type="number" step="0.1"></label>
          <label>Z <input id="targetZ" type="number" step="0.1"></label>
        </div>
        <button class="apply-target" id="applyTarget" type="button">Apply Target</button>
      </section>
      <section class="bottom-panel">
        <nav class="tab-bar">
          <button class="tab active" data-tab="progress">Progress</button>
          <button class="tab" data-tab="localization">Localization</button>
          <button class="tab" data-tab="control">Control</button>
        </nav>
        <div class="tab-panel active" id="tab-progress">
          <div class="track"><div class="progress" id="progress"></div></div>
          <div class="ticks"><span id="startTime">0.0s</span><span id="endTime">0.0s</span></div>
          <div class="log" id="phaseLog"></div>
        </div>
        <div class="tab-panel" id="tab-localization">
          <div class="loco-status" id="locoStatus">Waiting for /localization_pose...</div>
          <div class="loco-table">
            <div class="loco-hdr"></div>
            <div class="loco-hdr">Position (m)</div>
            <div class="loco-hdr">Velocity (m/s)</div>
            <div class="loco-axis">X</div>
            <div class="loco-val" id="locoX">--</div>
            <div class="loco-val" id="locoVx">--</div>
            <div class="loco-axis">Y</div>
            <div class="loco-val" id="locoY">--</div>
            <div class="loco-val" id="locoVy">--</div>
            <div class="loco-axis">Z</div>
            <div class="loco-val" id="locoZ">--</div>
            <div class="loco-val" id="locoVz">--</div>
          </div>
          <div class="loco-timestamp" id="locoTimestamp">--</div>
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
  <script>
    const PLC_MODE = __PLC_MODE__;
    const canvas = document.getElementById('scene');
    const ctx = canvas.getContext('2d');
    let payload;
    let frame = 0;
    let lastAdvance = 0;
    // PLC real-time state (updated from /api/control-state polling)
    let _liveControlState = null;
    let _controlActive = false;
    let _lastStepCount = -1;
    let _lastTrailTime = -1;  // 1Hz trail decimation

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

    function resize() {
      const rect = canvas.getBoundingClientRect();
      const ratio = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.floor(rect.width * ratio));
      canvas.height = Math.max(1, Math.floor(rect.height * ratio));
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      draw();
    }

    function ranges() {
      const b = payload.bounds;
      const pad = 0.9;
      return {
        xMin: b.xMin - pad,
        xMax: b.xMax + pad,
        yMin: b.yMin - pad,
        yMax: b.yMax + pad,
        zMin: Math.min(b.zMin, payload.target.z) - 0.35,
        zMax: Math.max(b.zMax, payload.initial.z, payload.target.z) + 0.35,
      };
    }

    function mapPlan(p, box, r) {
      const w = box.w;
      const h = box.h;
      const scale = Math.min(w / (r.xMax - r.xMin), h / (r.yMax - r.yMin));
      const usedW = (r.xMax - r.xMin) * scale;
      const usedH = (r.yMax - r.yMin) * scale;
      return {
        x: box.x + (w - usedW) / 2 + (p.x - r.xMin) * scale,
        y: box.y + (h + usedH) / 2 - (p.y - r.yMin) * scale,
      };
    }

    function mapZ(p, box, r) {
      return {
        x: box.x + (p.t / payload.frames[payload.frames.length - 1].t) * box.w,
        y: box.y + box.h - ((p.z - r.zMin) / (r.zMax - r.zMin)) * box.h,
      };
    }

    function label(text, x, y, color) {
      ctx.fillStyle = color;
      ctx.font = '700 13px system-ui, sans-serif';
      ctx.fillText(text, x, y);
    }

    function dot(p, color, size = 8) {
      ctx.beginPath();
      ctx.arc(p.x, p.y, size, 0, Math.PI * 2);
      ctx.fillStyle = color;
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#ffffff';
      ctx.stroke();
    }

    function drawGrid(box) {
      ctx.strokeStyle = '#2b3a47';
      ctx.lineWidth = 1;
      for (let i = 0; i <= 8; i++) {
        const x = box.x + (box.w * i) / 8;
        const y = box.y + (box.h * i) / 8;
        ctx.beginPath(); ctx.moveTo(x, box.y); ctx.lineTo(x, box.y + box.h); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(box.x, y); ctx.lineTo(box.x + box.w, y); ctx.stroke();
      }
    }

    function polyline(points, color, width) {
      if (points.length < 2) return;
      ctx.beginPath();
      ctx.moveTo(points[0].x, points[0].y);
      for (const p of points.slice(1)) ctx.lineTo(p.x, p.y);
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      ctx.stroke();
    }

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

    function drawBridgeBeam(p0, p1, current) {
      ctx.strokeStyle = 'rgba(89, 167, 232, 0.22)';
      ctx.lineWidth = 22;
      ctx.beginPath(); ctx.moveTo(p0.x, p0.y); ctx.lineTo(p1.x, p1.y); ctx.stroke();
      ctx.strokeStyle = '#59a7e8';
      ctx.lineWidth = 5;
      ctx.beginPath(); ctx.moveTo(p0.x, p0.y); ctx.lineTo(p1.x, p1.y); ctx.stroke();
      const cap = 12;
      ctx.fillStyle = '#89bce8';
      ctx.fillRect(p0.x - cap / 2, p0.y - 16, cap, 32);
      ctx.fillRect(p1.x - cap / 2, p1.y - 16, cap, 32);
      const speedMark = Math.min(1, Math.abs(current.vx) / 0.3);
      ctx.fillStyle = `rgba(240, 168, 59, ${0.25 + speedMark * 0.45})`;
      ctx.fillRect(p0.x - 4, p0.y - 23, 8, 46);
      ctx.fillRect(p1.x - 4, p1.y - 23, 8, 46);
    }

    function drawTrolley(p, current) {
      const sway = Math.sin(current.t * 3.5) * Math.min(10, Math.abs(current.vy) * 28);
      ctx.fillStyle = 'rgba(0, 0, 0, 0.28)';
      ctx.beginPath();
      ctx.ellipse(p.x + 4, p.y + 18, 33, 10, 0, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = '#f0a83b';
      ctx.strokeStyle = '#ffd18a';
      ctx.lineWidth = 2;
      ctx.fillRect(p.x - 34, p.y - 20, 68, 40);
      ctx.strokeRect(p.x - 34, p.y - 20, 68, 40);
      ctx.fillStyle = '#203040';
      ctx.fillRect(p.x - 18, p.y - 9, 36, 18);

      ctx.strokeStyle = '#c8d2dc';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(p.x, p.y + 20);
      ctx.lineTo(p.x + sway, p.y + 92);
      ctx.stroke();
      ctx.fillStyle = '#e05a47';
      ctx.beginPath();
      ctx.arc(p.x + sway, p.y + 103, 12, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    function drawCraneBay(box, r, current) {
      drawPanel(box, 'Bridge Crane Bay');
      drawGrid(box);
      // Axis tick labels
      ctx.fillStyle = '#91a3b3';
      ctx.font = '10px system-ui, sans-serif';
      ctx.textAlign = 'center';
      var xRange = r.xMax - r.xMin;
      var yRange = r.yMax - r.yMin;
      var xStep = Math.pow(10, Math.floor(Math.log10(xRange))) / 2;
      if (xRange / xStep > 8) xStep *= 2;
      var yStep = Math.pow(10, Math.floor(Math.log10(yRange))) / 2;
      if (yRange / yStep > 8) yStep *= 2;
      // X-axis ticks (bottom)
      for (var xv = Math.ceil(r.xMin / xStep) * xStep; xv <= r.xMax; xv += xStep) {
        var sx = box.x + ((xv - r.xMin) / xRange) * box.w;
        ctx.fillText(xv.toFixed(1), sx, box.y + box.h + 16);
      }
      // Y-axis ticks (left)
      ctx.textAlign = 'right';
      for (var yv = Math.ceil(r.yMin / yStep) * yStep; yv <= r.yMax; yv += yStep) {
        var sy = box.y + box.h - ((yv - r.yMin) / yRange) * box.h;
        ctx.fillText(yv.toFixed(1), box.x - 6, sy + 4);
      }
      ctx.textAlign = 'start';
      const railTop = box.y + box.h * 0.11;
      const railBottom = box.y + box.h * 0.89;
      ctx.strokeStyle = '#7c8790';
      ctx.lineWidth = 5;
      ctx.beginPath(); ctx.moveTo(box.x + 18, railTop); ctx.lineTo(box.x + box.w - 18, railTop); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(box.x + 18, railBottom); ctx.lineTo(box.x + box.w - 18, railBottom); ctx.stroke();

      const path = payload.frames.map(f => mapPlan(f, box, r));
      const traveled = payload.frames.slice(0, frame + 1).map(f => mapPlan(f, box, r));
      polyline(path, '#53697a', 2);
      polyline(traveled, '#5ebd72', 4);

      const xTop = mapPlan({x: current.x, y: r.yMax}, box, r);
      const xBottom = mapPlan({x: current.x, y: r.yMin}, box, r);
      const now = mapPlan(current, box, r);
      drawBridgeBeam(xTop, xBottom, current);
      drawTrolley(now, current);

      const initial = mapPlan(payload.initial, box, r);
      const target = mapPlan(payload.target, box, r);
      dot(initial, '#7c8790', 6); label('Initial', initial.x + 10, initial.y - 8, '#aab5bd');
      dot(target, '#f0a83b', 8); label('Target', target.x + 10, target.y - 8, '#ffc263');
      label(`X bridge ${current.x.toFixed(2)} m`, box.x + 16, box.y + 28, '#9fc9ed');
      label(`Y trolley ${current.y.toFixed(2)} m`, box.x + 16, box.y + 48, '#ffe0a8');
    }

    function drawTrolleyCloseup(box, r, current) {
      drawPanel(box, 'Trolley Movement');
      const centerX = box.x + box.w * 0.5;
      const railTop = box.y + 40;
      const railBottom = box.y + box.h - 28;
      const yToScreen = y => railBottom - ((y - r.yMin) / (r.yMax - r.yMin)) * (railBottom - railTop);
      const cartY = yToScreen(current.y);
      const targetY = yToScreen(payload.target.y);

      ctx.strokeStyle = '#596b7a';
      ctx.lineWidth = 14;
      ctx.beginPath(); ctx.moveTo(centerX, railTop); ctx.lineTo(centerX, railBottom); ctx.stroke();
      ctx.strokeStyle = '#b7c2cc';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(centerX, railTop); ctx.lineTo(centerX, railBottom); ctx.stroke();

      ctx.setLineDash([5, 5]);
      ctx.strokeStyle = '#f0a83b';
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(box.x + 20, targetY); ctx.lineTo(box.x + box.w - 20, targetY); ctx.stroke();
      ctx.setLineDash([]);
      label('target Y', box.x + 22, targetY - 8, '#ffc263');

      ctx.fillStyle = '#f0a83b';
      ctx.strokeStyle = '#ffd18a';
      ctx.lineWidth = 2;
      ctx.fillRect(centerX - 58, cartY - 28, 116, 56);
      ctx.strokeRect(centerX - 58, cartY - 28, 116, 56);
      ctx.fillStyle = '#203040';
      ctx.fillRect(centerX - 33, cartY - 13, 66, 26);
      label(`${current.y.toFixed(2)} m`, centerX - 28, cartY + 48, '#ffe0a8');
    }

    function drawHoistProfile(box, r, current) {
      drawPanel(box, 'Hoist Height');
      const zPath = payload.frames.map(f => mapZ(f, box, r));
      const zTravel = payload.frames.slice(0, frame + 1).map(f => mapZ(f, box, r));
      polyline(zPath, '#6f5835', 2);
      polyline(zTravel, '#f0a83b', 4);
      const targetY = mapZ({t: 0, z: payload.target.z}, box, r).y;
      const now = mapZ(current, box, r);
      ctx.setLineDash([5, 5]);
      ctx.beginPath(); ctx.moveTo(box.x, targetY); ctx.lineTo(box.x + box.w, targetY);
      ctx.strokeStyle = '#5ebd72'; ctx.lineWidth = 1.5; ctx.stroke();
      ctx.setLineDash([]);
      dot(now, '#e05a47', 6);
      label(`Z ${current.z.toFixed(2)} m`, box.x + 12, box.y + 28, '#ffc263');
    }

    function layout(rect) {
      if (rect.width > 900) {
        const sideW = Math.max(270, rect.width * 0.28);
        const bay = { x: 38, y: 82, w: rect.width - sideW - 58, h: rect.height - 124 };
        const closeup = { x: bay.x + bay.w + 28, y: 82, w: sideW - 18, h: (rect.height - 154) * 0.58 };
        const zbox = { x: closeup.x, y: closeup.y + closeup.h + 52, w: closeup.w, h: rect.height - (closeup.y + closeup.h + 90) };
        return { bay, closeup, zbox };
      }
      const bay = { x: 28, y: 82, w: rect.width - 56, h: rect.height * 0.52 };
      const closeup = { x: 28, y: bay.y + bay.h + 50, w: rect.width - 56, h: rect.height * 0.20 };
      const zbox = { x: 28, y: closeup.y + closeup.h + 48, w: rect.width - 56, h: rect.height - (closeup.y + closeup.h + 86) };
      return { bay, closeup, zbox };
    }

    function draw() {
      const rect = canvas.getBoundingClientRect();
      if (!payload) {
        // PLC mode waiting for data — show placeholder
        ctx.clearRect(0, 0, rect.width, rect.height);
        ctx.fillStyle = '#91a3b3';
        ctx.font = '700 18px system-ui, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('Waiting for localization data...', rect.width / 2, rect.height / 2);
        ctx.textAlign = 'start';
        return;
      }
      ctx.clearRect(0, 0, rect.width, rect.height);
      const r = ranges();
      const current = payload.frames[frame];
      const boxes = layout(rect);
      drawCraneBay(boxes.bay, r, current);
      drawTrolleyCloseup(boxes.closeup, r, current);
      drawHoistProfile(boxes.zbox, r, current);
    }

    function speedColor(absV, limit) {
      const ratio = absV / (limit || 1);
      if (ratio < 0.33) return '#5ebd72';
      if (ratio < 0.66) return '#f0a83b';
      return '#e05a47';
    }

    function renderStatus() {
      const f = payload.frames[frame];
      const total = payload.frames[payload.frames.length - 1].t;
      els.time.textContent = `${f.t.toFixed(1)}s`;
      els.frame.textContent = `${frame + 1}`;
      els.speed.textContent = `${payload.speed.toFixed(1)}x`;
      els.phase.textContent = f.phaseLabel;
      els.x.textContent = f.x.toFixed(2);
      els.y.textContent = f.y.toFixed(2);
      els.z.textContent = f.z.toFixed(2);
      els.progress.style.width = `${Math.min(100, (f.t / total) * 100)}%`;

      const xyLimit = payload.velocityLimits.xy;
      const zLimit = payload.velocityLimits.z;
      els.vx.textContent = f.vx.toFixed(2);
      els.vy.textContent = f.vy.toFixed(2);
      els.vz.textContent = f.vz.toFixed(2);

      const vxColor = speedColor(Math.abs(f.vx), xyLimit);
      const vyColor = speedColor(Math.abs(f.vy), xyLimit);
      const vzColor = speedColor(Math.abs(f.vz), zLimit);
      els.coordVx.style.borderColor = vxColor;
      els.coordVy.style.borderColor = vyColor;
      els.coordVz.style.borderColor = vzColor;
      els.vx.style.color = vxColor;
      els.vy.style.color = vyColor;
      els.vz.style.color = vzColor;

      // Control tab values are updated exclusively by pollPlcStatus()
      // to avoid racing between replay frames and live PLC data.
    }

    function renderTargetControls() {
      els.targetX.value = payload.target.x.toFixed(2);
      els.targetY.value = payload.target.y.toFixed(2);
      els.targetZ.value = payload.target.z.toFixed(2);
    }

    function applyTargetCommand() {
      if (PLC_MODE) {
        // PLC real-time mode: POST target → start control
        const target = {
          target_x: parseFloat(els.targetX.value),
          target_y: parseFloat(els.targetY.value),
          target_z: parseFloat(els.targetZ.value),
        };
        els.applyTarget.disabled = true;
        els.applyTarget.textContent = 'Starting...';
        els.ctrlMsg.textContent = '';
        fetch('/api/start-control', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(target),
        })
          .then(r => r.json())
          .then(data => {
            if (data.ok) {
              els.ctrlMsg.textContent = 'Control started — polling...';
              els.ctrlMsg.style.color = '#5ebd72';
              _lastStepCount = -1;  // reset so first step triggers Start marker
            } else {
              els.ctrlMsg.textContent = 'Error: ' + (data.error || 'unknown');
              els.ctrlMsg.style.color = '#e05a47';
              els.applyTarget.disabled = false;
              els.applyTarget.textContent = 'Apply Target';
            }
          })
          .catch(err => {
            els.ctrlMsg.textContent = 'Failed: ' + err.message;
            els.ctrlMsg.style.color = '#e05a47';
            els.applyTarget.disabled = false;
            els.applyTarget.textContent = 'Apply Target';
          });
        return;
      }
      // Simulation replay mode: URL-based re-simulation
      const params = new URLSearchParams(window.location.search);
      params.set('target_x', els.targetX.value);
      params.set('target_y', els.targetY.value);
      params.set('target_z', els.targetZ.value);
      window.location.search = params.toString();
    }

    function animate(ts) {
      if (!payload) return;
      if (!lastAdvance) lastAdvance = ts;
      if (ts - lastAdvance >= payload.framePeriodMs) {
        frame = Math.min(frame + 1, payload.frames.length - 1);
        lastAdvance = ts;
        renderStatus();
      }
      draw();
      requestAnimationFrame(animate);
    }

    els.applyTarget.addEventListener('click', applyTargetCommand);
    for (const input of [els.targetX, els.targetY, els.targetZ]) {
      input.addEventListener('keydown', event => {
        if (event.key === 'Enter') applyTargetCommand();
      });
    }

    if (PLC_MODE) {
      // PLC mode — wait for control stream, show initial state
      els.phase.textContent = 'PLC Mode — Set Target';
      els.phaseLog.innerHTML = '<div>Waiting for target...</div>';
      els.rate.textContent = 'Live';
      els.time.textContent = '0.0s';
      els.frame.textContent = '—';
      els.speed.textContent = 'Live';
      // Initialize target inputs empty
      els.targetX.value = '';
      els.targetY.value = '';
      els.targetZ.value = '';
      // Create default payload so Canvas shows crane immediately
      const defaultPos = {x: 0, y: 0, z: 5};
      payload = {
        updateHz: 10, speed: 1, framePeriodMs: 100,
        target: defaultPos,
        initial: defaultPos,
        bounds: { xMin: -2, xMax: 10, yMin: -2, yMax: 8, zMin: 0, zMax: 7 },
        velocityLimits: {xy: 0.3, z: 0.2},
        phaseBoundaries: [],
        frames: [{t: 0, x: 0, y: 0, z: 5, vx: 0, vy: 0, vz: 0,
                  vxCmd: 0, vyCmd: 0, vzCmd: 0, phaseLabel: 'Waiting for localization...'}],
      };
      frame = 0;
      els.rate.textContent = 'Live';
      els.startTime.textContent = '—';
      els.endTime.textContent = '—';
      resize();
      draw();
      requestAnimationFrame(draw);
    } else {
      fetch('/simulation.json' + window.location.search)
        .then(async r => {
          if (!r.ok) throw new Error(await r.text());
          return r.json();
        })
        .then(data => {
          payload = data;
          frame = 0;
          lastAdvance = 0;
          els.rate.textContent = `${payload.updateHz} Hz`;
          els.startTime.textContent = '0.0s';
          els.endTime.textContent = `${payload.frames[payload.frames.length - 1].t.toFixed(1)}s`;
          els.phaseLog.innerHTML = payload.phaseBoundaries.map(p =>
            `<div><span>${p.label}</span><span>${p.t.toFixed(1)}s</span></div>`
          ).join('');
          renderTargetControls();
          renderStatus();
          resize();
          requestAnimationFrame(animate);
        })
        .catch(error => {
          els.phase.textContent = 'Invalid target';
          els.phaseLog.textContent = error.message;
        });
    }

    window.addEventListener('resize', resize);

    // Tab switching
    document.querySelectorAll('.tab').forEach(tab => {
      tab.addEventListener('click', () => {
        document.querySelectorAll('.tab, .tab-panel').forEach(el => el.classList.remove('active'));
        tab.classList.add('active');
        document.getElementById('tab-' + tab.dataset.tab).classList.add('active');
      });
    });

    // SSE stream for real-time localization data
    const locoSource = new EventSource('/localization/stream');
    locoSource.onmessage = (event) => {
      const data = JSON.parse(event.data);
      els.locoX.textContent = data.x.toFixed(3);
      els.locoY.textContent = data.y.toFixed(3);
      els.locoZ.textContent = data.z.toFixed(3);
      els.locoVx.textContent = data.vx.toFixed(3);
      els.locoVy.textContent = data.vy.toFixed(3);
      els.locoVz.textContent = data.vz.toFixed(3);
      const stamp = data.stamp_sec + data.stamp_nsec * 1e-9;
      els.locoTimestamp.textContent = 'Stamp: ' + new Date(stamp * 1000).toLocaleTimeString();
      els.locoStatus.textContent = 'Connected — /localization_pose';
      els.locoStatus.style.color = '#5ebd72';

      // PLC mode — use localization data to drive Canvas when no control is active
      if (PLC_MODE && !_controlActive) {
        const pos = {x: data.x, y: data.y, z: data.z};
        const vel = {vx: data.vx, vy: data.vy, vz: data.vz};
        if (!payload) {
          payload = {
            updateHz: 10, speed: 1, framePeriodMs: 100,
            target: pos,
            initial: pos,
            bounds: {
              xMin: data.x - 2, xMax: data.x + 2,
              yMin: data.y - 2, yMax: data.y + 2,
              zMin: data.z - 2, zMax: data.z + 2,
            },
            velocityLimits: {xy: 0.3, z: 0.2},
            phaseBoundaries: [],
            frames: [],
          };
        }
        // Keep a sliding window of recent positions for the trail
        const f = {t: stamp, x: data.x, y: data.y, z: data.z,
                   vx: data.vx, vy: data.vy, vz: data.vz,
                   vxCmd: 0, vyCmd: 0, vzCmd: 0,
                   phaseLabel: 'Localization Live'};
        payload.frames.push(f);
        if (payload.frames.length > 30) payload.frames.shift();
        payload.target = pos;  // no target yet, show current position
        frame = payload.frames.length - 1;

        // Update readout
        els.time.textContent = new Date(stamp * 1000).toLocaleTimeString();
        els.phase.textContent = 'PLC Live — Set Target';
        els.x.textContent = data.x.toFixed(2);
        els.y.textContent = data.y.toFixed(2);
        els.z.textContent = data.z.toFixed(2);
        els.vx.textContent = data.vx.toFixed(2);
        els.vy.textContent = data.vy.toFixed(2);
        els.vz.textContent = data.vz.toFixed(2);

        // Keep bounds updated
        payload.bounds.xMin = Math.min(payload.bounds.xMin, data.x - 2);
        payload.bounds.xMax = Math.max(payload.bounds.xMax, data.x + 2);
        payload.bounds.yMin = Math.min(payload.bounds.yMin, data.y - 2);
        payload.bounds.yMax = Math.max(payload.bounds.yMax, data.y + 2);

        draw();
      }
    };
    locoSource.onerror = () => {
      els.locoStatus.textContent = 'Disconnected — waiting for data...';
      els.locoStatus.style.color = '#e05a47';
    };

    // PLC mode — poll /api/control-state at 10 Hz for real-time PD state
    function pollControlState() {
      fetch('/api/control-state')
        .then(r => r.json())
        .then(s => {
          if (!s.running && !s.done && !s.error) return; // control not started yet
          if (s.error) {
            _controlActive = false;
            els.ctrlMsg.textContent = 'Error: ' + s.error;
            els.ctrlMsg.style.color = '#e05a47';
            els.applyTarget.disabled = false;
            els.applyTarget.textContent = 'Apply Target';
            return;
          }
          if (s.done) {
            if (_controlActive) {
              _controlActive = false;
              els.phase.textContent = 'Target Reached';
              els.ctrlMsg.textContent = 'Control complete — all axes arrived';
              els.ctrlMsg.style.color = '#5ebd72';
              els.applyTarget.disabled = false;
              els.applyTarget.textContent = 'Apply Target';
            }
            return;
          }
          if (!s.latest) return;
          const d = s.latest;
          // Only process new steps (avoid duplicate rendering)
          if (s.step_count <= _lastStepCount) return;
          _lastStepCount = s.step_count;

          // On first step, set Start marker + reset trajectory with auto-fit bounds
          if (!_controlActive) {
            payload.initial = s.start_pos
              ? {x: s.start_pos.x, y: s.start_pos.y, z: s.start_pos.z}
              : {x: d.x, y: d.y, z: d.z};
            payload.target = {x: d.p_ref_x, y: d.p_ref_y, z: d.p_ref_z};
            // Keep existing frames for continuous trajectory, mark PD start
            payload.frames.push({t: d.t, x: d.x, y: d.y, z: d.z,
                                 vx: 0, vy: 0, vz: 0, vxCmd: 0, vyCmd: 0, vzCmd: 0,
                                 phaseLabel: 'PD START'});
            // Fit bounds around initial→target with 20% padding
            var ix = payload.initial.x, iy = payload.initial.y, iz = payload.initial.z;
            var tx = d.p_ref_x, ty = d.p_ref_y, tz = d.p_ref_z;
            var padX = Math.max(1.0, Math.abs(tx - ix) * 0.2);
            var padY = Math.max(1.0, Math.abs(ty - iy) * 0.2);
            var padZ = Math.max(1.0, Math.abs(tz - iz) * 0.2);
            payload.bounds = {
              xMin: Math.min(ix, tx) - padX, xMax: Math.max(ix, tx) + padX,
              yMin: Math.min(iy, ty) - padY, yMax: Math.max(iy, ty) + padY,
              zMin: Math.min(iz, tz) - padZ, zMax: Math.max(iz, tz) + padZ,
            };
            els.ctrlMsg.textContent = 'Start(' + ix.toFixed(1) + ',' + iy.toFixed(1) + ') Target(' + tx.toFixed(1) + ',' + ty.toFixed(1) + ')';
            els.ctrlMsg.style.color = '#5ebd72';
          }
          _controlActive = true;
          // Expand bounds if position moves outside (trajectory auto-follow)
          var b = payload.bounds;
          if (d.x < b.xMin + 0.5) b.xMin = d.x - 2;
          if (d.x > b.xMax - 0.5) b.xMax = d.x + 2;
          if (d.y < b.yMin + 0.5) b.yMin = d.y - 2;
          if (d.y > b.yMax - 0.5) b.yMax = d.y + 2;
          if (d.z < b.zMin + 0.5) b.zMin = d.z - 2;
          if (d.z > b.zMax - 0.5) b.zMax = d.z + 2;
          // Update readout — show PD state: error + position
          var ex = ((d.p_ref_x||0) - (d.x||0)).toFixed(2);
          var ey = ((d.p_ref_y||0) - (d.y||0)).toFixed(2);
          var ez = ((d.p_ref_z||0) - (d.z||0)).toFixed(2);
          els.time.textContent = (d.t || 0).toFixed(1) + 's';
          els.phase.textContent = 'PD#' + s.step_count
            + ' err=(' + ex + ',' + ey + ',' + ez + ')m'
            + ' cmd=(' + (d.vx_cmd||0).toFixed(2) + ',' + (d.vy_cmd||0).toFixed(2) + ',' + (d.vz_cmd||0).toFixed(2) + ')m/s';
          els.phase.style.color = '#f0a83b';
          els.ctrlMsg.textContent = 'PD step #' + s.step_count;
          els.ctrlMsg.style.color = '#5ebd72';
          els.x.textContent = (d.x || 0).toFixed(2);
          els.y.textContent = (d.y || 0).toFixed(2);
          els.z.textContent = (d.z || 0).toFixed(2);
          els.vx.textContent = (d.vx_cmd || 0).toFixed(2);
          els.vy.textContent = (d.vy_cmd || 0).toFixed(2);
          els.vz.textContent = (d.vz_cmd || 0).toFixed(2);
          const xyLimit = 0.3, zLimit = 0.2;
          els.coordVx.style.borderColor = speedColor(Math.abs(d.vx_cmd), xyLimit);
          els.coordVy.style.borderColor = speedColor(Math.abs(d.vy_cmd), xyLimit);
          els.coordVz.style.borderColor = speedColor(Math.abs(d.vz_cmd), zLimit);
          els.vx.style.color = speedColor(Math.abs(d.vx_cmd), xyLimit);
          els.vy.style.color = speedColor(Math.abs(d.vy_cmd), xyLimit);
          els.vz.style.color = speedColor(Math.abs(d.vz_cmd), zLimit);
          if (!els.targetX.value) {
            els.targetX.value = d.p_ref_x.toFixed(2);
            els.targetY.value = d.p_ref_y.toFixed(2);
            els.targetZ.value = d.p_ref_z.toFixed(2);
          }
          const f = {
            t: d.t, x: d.x, y: d.y, z: d.z,
            vx: d.vx, vy: d.vy, vz: d.vz,
            vxCmd: d.vx_cmd, vyCmd: d.vy_cmd, vzCmd: d.vz_cmd,
            phaseLabel: 'PD Control',
          };
          // 2 Hz trail sampling: 3000 frames = 25 min @ 2 Hz
          if (_lastTrailTime < 0 || d.t - _lastTrailTime >= 0.45) {
            payload.frames.push(f);
            if (payload.frames.length > 3000) payload.frames.shift();
            _lastTrailTime = d.t;
          }
          frame = Math.max(0, payload.frames.length - 1);
          draw();
        })
        .catch(function(err) {
          els.ctrlMsg.textContent = 'Poll error: ' + (err.message || err);
          els.ctrlMsg.style.color = '#e05a47';
        });
    }
    if (PLC_MODE) {
      setInterval(pollControlState, 100); // 10 Hz polling
    }

    // Control tab — PLC status polling (2 Hz), flash on PLC command change
    let _lastPlcVx = null, _lastPlcVy = null, _lastPlcHz = null;
    function pollPlcStatus() {
      fetch('/api/plc-status')
        .then(r => r.json())
        .then(s => {
          // PLC connection — green when connected, red when not
          if (s.connected) {
            els.plcDot.classList.add('on');
            els.plcStatusText.textContent = 'PLC';
            els.plcStatusText.style.color = 'var(--green)';
          } else {
            els.plcDot.classList.remove('on');
            els.plcStatusText.textContent = 'PLC';
            els.plcStatusText.style.color = 'var(--red)';
          }
          // Heartbeat — green when healthy, red when not
          if (s.heartbeat) {
            els.hbDot.classList.add('on');
            els.hbStatusText.textContent = 'Heartbeat';
            els.hbStatusText.style.color = 'var(--green)';
          } else {
            els.hbDot.classList.remove('on');
            els.hbStatusText.textContent = 'Heartbeat';
            els.hbStatusText.style.color = 'var(--red)';
          }
          // Update Control tab cells — always show latest value, flash on change
          function updateCell(el, newVal, lastVal, unit) {
            const txt = newVal.toFixed(3) + ' ' + unit;
            // Always update text (first call + every poll)
            if (lastVal === null) {
              el.textContent = txt;
              return newVal;
            }
            // Flash on significant change
            if (Math.abs(newVal - lastVal) > 0.001) {
              el.textContent = txt;
              el.classList.add('flash');
              setTimeout(() => el.classList.remove('flash'), 400);
              return newVal;
            }
            // Small change: update text without flash
            el.textContent = txt;
            return newVal;
          }
          _lastPlcVx = updateCell(els.ctrlVx, s.last_vx, _lastPlcVx, 'm/s');
          _lastPlcVy = updateCell(els.ctrlVy, s.last_vy, _lastPlcVy, 'm/s');
          // Z height (m) — always show latest
          {
            const txt = s.last_hz.toFixed(3) + ' m';
            if (_lastPlcHz !== null && Math.abs(s.last_hz - _lastPlcHz) > 0.001) {
              els.ctrlHz.classList.add('flash');
              setTimeout(() => els.ctrlHz.classList.remove('flash'), 400);
            }
            els.ctrlHz.textContent = txt;
            _lastPlcHz = s.last_hz;
          }
        })
        .catch(() => {
          els.plcDot.classList.remove('on');
          els.plcStatusText.style.color = 'var(--red)';
          els.hbDot.classList.remove('on');
          els.hbStatusText.style.color = 'var(--red)';
        });
    }
    pollPlcStatus();
    setInterval(pollPlcStatus, 500);

    // Control tab — STOP ALL / Reset buttons
    els.btnStop.addEventListener('click', () => {
      els.btnStop.disabled = true;
      els.btnStop.textContent = 'STOPPING...';
      fetch('/api/stop')
        .then(r => r.json())
        .then(() => {
          els.ctrlMsg.textContent = 'STOPPED — All axes set to zero';
          els.ctrlMsg.style.color = '#e05a47';
          els.btnStop.textContent = 'STOP ALL';
          els.btnStop.disabled = false;
        })
        .catch(() => {
          els.ctrlMsg.textContent = 'Failed to send stop command';
          els.ctrlMsg.style.color = '#e05a47';
          els.btnStop.textContent = 'STOP ALL';
          els.btnStop.disabled = false;
        });
    });

    els.btnReset.addEventListener('click', () => {
      els.btnReset.disabled = true;
      els.btnReset.textContent = 'RESETTING...';
      fetch('/api/reset')
        .then(r => r.json())
        .then(() => {
          els.ctrlMsg.textContent = 'RESET — Control restored';
          els.ctrlMsg.style.color = '#5ebd72';
          els.btnReset.textContent = 'Reset Control';
          els.btnReset.disabled = false;
        });
    });
  </script>
</body>
</html>"""
    return html.replace('__PLC_MODE__', 'true' if plc_mode else 'false')


class _LiveRequestHandler(BaseHTTPRequestHandler):
    def _write(self, status: int, content_type: str, body: bytes):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/start-control':
            self._handle_start_control()
        else:
            self._write(404, 'text/plain; charset=utf-8', b'not found')

    def do_GET(self):
        parsed = urlparse(self.path)
        plc_mode = self.server.plc_actuator is not None
        if parsed.path in ('/', '/index.html'):
            self._write(200, 'text/html; charset=utf-8', render_live_html(plc_mode).encode('utf-8'))
        elif parsed.path == '/simulation.json':
            if plc_mode:
                self._write(400, 'application/json; charset=utf-8',
                            json.dumps({'error': 'Not available in PLC mode'}).encode('utf-8'))
                return
            try:
                payload = self.server.build_payload(parse_qs(parsed.query))
            except ValueError as exc:
                body = json.dumps({'error': str(exc)}, ensure_ascii=False).encode('utf-8')
                self._write(400, 'application/json; charset=utf-8', body)
                return
            body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
            self._write(200, 'application/json; charset=utf-8', body)
        elif parsed.path == '/localization/stream':
            self._stream_localization()
        elif parsed.path == '/control/stream':
            self._stream_control()
        elif parsed.path == '/api/stop':
            self._handle_stop()
        elif parsed.path == '/api/reset':
            self._handle_reset()
        elif parsed.path == '/api/plc-status':
            self._handle_plc_status()
        elif parsed.path == '/api/control-state':
            self._handle_control_state()
        else:
            self._write(404, 'text/plain; charset=utf-8', b'not found')

    def _stream_localization(self):
        """SSE endpoint that streams the latest /localization_pose data at ~10 Hz."""
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        try:
            while True:
                pose = get_latest_pose()
                if pose is not None:
                    data = json.dumps(pose, ensure_ascii=False)
                    self.wfile.write(f'data: {data}\n\n'.encode('utf-8'))
                    self.wfile.flush()
                else:
                    # Heartbeat comment to keep connection alive
                    self.wfile.write(': waiting for /localization_pose\n\n'.encode('utf-8'))
                    self.wfile.flush()
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _stream_control(self):
        """SSE endpoint — stream real-time PD control state to browser."""
        server = self.server
        if server.plc_actuator is None:
            self._write(400, 'text/plain; charset=utf-8', b'PLC mode not active')
            return

        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        print('[control SSE] Client connected, waiting for control hooks...')
        try:
            while True:
                hooks = server.control_hooks  # re-read each iteration — hooks set later by /api/start-control
                event = hooks.get_event(timeout=0.5) if hooks else None
                if event is not None:
                    data = json.dumps(event, ensure_ascii=False)
                    self.wfile.write(f'data: {data}\n\n'.encode('utf-8'))
                    self.wfile.flush()
                    if event.get('type') in ('done', 'error'):
                        print(f'[control SSE] Sent {event.get("type")}, closing')
                        break
                else:
                    if hooks is None:
                        pass  # no hooks yet, heartbeat only
                    # Keep-alive heartbeat
                    self.wfile.write(': heartbeat\n\n'.encode('utf-8'))
                    self.wfile.flush()
                    time.sleep(0.25)  # 防止 busy-loop, 4Hz 心跳足够
        except (BrokenPipeError, ConnectionResetError):
            print('[control SSE] Client disconnected')
        except Exception as exc:
            print(f'[control SSE] Unexpected error: {exc}')
            import traceback; traceback.print_exc()

    def _handle_start_control(self):
        """POST /api/start-control — parse target and start PLC PD control thread."""
        server = self.server
        if server.plc_actuator is None or server.ros_source is None or server.config is None:
            self._write(400, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'PLC mode not active'}).encode('utf-8'))
            return

        # Read request body
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length).decode('utf-8')
        try:
            target = json.loads(body)
            target_x = float(target['target_x'])
            target_y = float(target['target_y'])
            target_z = float(target['target_z'])
        except (ValueError, KeyError, TypeError) as exc:
            self._write(400, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': f'Invalid target: {exc}'}).encode('utf-8'))
            return

        # Check if control is already running
        if server.control_thread is not None and server.control_thread.is_alive():
            self._write(409, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'Control already running'}).encode('utf-8'))
            return

        # Get current position from localization
        pose = get_latest_pose()
        if pose is None:
            self._write(503, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'No localization data — cannot start control'}).encode('utf-8'))
            return

        # Create control state for frontend polling
        cs = ControlState()
        cs.set_start(
            pos={'x': pose['x'], 'y': pose['y'], 'z': pose['z']},
            target={'x': target_x, 'y': target_y, 'z': target_z},
        )
        server.control_state = cs

        # Create control hooks — writes to ControlState (polling) + queue (SSE diagnostic)
        hooks = LiveControlHooks(control_state=cs)
        server.control_hooks = hooks

        initial_state = CraneState(x0=pose['x'], y0=pose['y'], z0=pose['z'])
        # Update PlcActuator Z height to match current position
        server.plc_actuator._z_height = pose['z']

        target_pos = (target_x, target_y, target_z)

        # Start control in background thread
        def _run():
            print(f'[PLC control] Starting PD: target=({target_x:.2f}, {target_y:.2f}, {target_z:.2f}), '
                  f'start=({initial_state.x.position:.2f}, {initial_state.y.position:.2f}, {initial_state.z.position:.2f})')
            try:
                history, events = run_pd_control(
                    source=server.ros_source,
                    actuator=server.plc_actuator,
                    config=server.config,
                    target_pos=target_pos,
                    initial_state=initial_state,
                    hooks=hooks,
                    verbose=True,
                    is_simulation=False,
                    max_time=600.0,  # 10min max — same as expected max operation time
                )
                print(f'[PLC control] PD complete — {len(history)} steps, arrivals: {[(t,a) for t,a in events]}')
                hooks.done()
                cs.set_done()
            except TimeoutError as exc:
                msg = (f'Timeout after 20 min — axes did not all arrive. '
                       f'Check PLC connection or localization. Use STOP ALL to abort sooner.')
                print(f'[PLC control] {msg}')
                hooks.send_error(msg)
                cs.set_error(msg)
            except Exception as exc:
                import traceback
                print(f'[PLC control] Error: {exc}')
                traceback.print_exc()
                hooks.send_error(str(exc))
                cs.set_error(str(exc))
            finally:
                server.control_thread = None
                print(f'[PLC control] Thread exiting')

        server.control_thread = threading.Thread(target=_run, name='plc-control', daemon=True)
        server.control_thread.start()
        print(f'[PLC control] Thread started')

        self._write(200, 'application/json; charset=utf-8',
                    json.dumps({'ok': True, 'message': 'Control started'}).encode('utf-8'))

    def _handle_stop(self):
        """STOP ALL: send zero velocity to all three axes (matches demo.cpp pattern)."""
        plc = self.server.plc
        if plc is not None:
            plc.big_car_ctrl(0.0)
            plc.small_car_ctrl(0.0)
            plc.lift_ctrl(0.0)
        # Also stop PLC control loop if running
        server = self.server
        if server.control_hooks is not None:
            server.control_hooks.stop()
        self._write(200, 'application/json; charset=utf-8', b'{"ok":true}')

    def _handle_reset(self):
        """Reset PLC control after emergency stop."""
        plc = self.server.plc
        if plc is not None:
            plc.reset()
        self._write(200, 'application/json; charset=utf-8', b'{"ok":true}')

    def _handle_plc_status(self):
        """Return PLC connection, heartbeat, and last-sent command values."""
        plc = self.server.plc
        if plc is None:
            body = json.dumps({
                'connected': False, 'heartbeat': False, 'mode': 'none',
                'last_vx': 0.0, 'last_vy': 0.0, 'last_hz': 0.0, 'last_vz': 0.0,
            })
        else:
            body = json.dumps({
                'connected': plc.check_connection(),
                'heartbeat': plc.heartbeat_healthy,
                'mode': 'mock' if type(plc).__name__ == 'MockPLC' else 'real',
                'last_vx': getattr(plc, 'last_vx', 0.0),
                'last_vy': getattr(plc, 'last_vy', 0.0),
                'last_hz': getattr(plc, 'last_hz', 0.0),
                'last_vz': getattr(plc, 'last_vz', 0.0),
            })
        self._write(200, 'application/json; charset=utf-8', body.encode('utf-8'))

    def _handle_control_state(self):
        """Return latest PD control state for frontend polling (10 Hz)."""
        cs = self.server.control_state
        if cs is None:
            self._write(200, 'application/json; charset=utf-8',
                        json.dumps({'running': False, 'latest': None}).encode('utf-8'))
            return
        self._write(200, 'application/json; charset=utf-8',
                    json.dumps(cs.snapshot(), ensure_ascii=False).encode('utf-8'))

    def log_message(self, format, *args):
        return


class ControlState:
    """Thread-safe shared state for the active PD control run.

    Written by the control thread, read by /api/control-state polling.
    This replaces the SSE + hooks queue architecture with simple polling.
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.latest: dict | None = None   # most recent step_data
        self.running: bool = False
        self.start_pos: dict | None = None  # {'x','y','z'} at Apply Target time
        self.target_pos: dict | None = None # {'x','y','z'} target
        self.step_count: int = 0
        self.arrivals: list = []            # [{'axis': 'x', 't': 1.23}, ...]
        self.done: bool = False
        self.error: str | None = None

    def set_start(self, pos: dict, target: dict):
        with self.lock:
            self.start_pos = dict(pos)
            self.target_pos = dict(target)
            self.running = True
            self.done = False
            self.error = None
            self.step_count = 0
            self.arrivals = []

    def set_step(self, step_data: dict):
        with self.lock:
            self.latest = dict(step_data)
            self.step_count += 1

    def set_arrival(self, axis: str, t: float):
        with self.lock:
            self.arrivals.append({'axis': axis, 't': t})

    def set_done(self):
        with self.lock:
            self.done = True
            self.running = False

    def set_error(self, msg: str):
        with self.lock:
            self.error = msg
            self.running = False

    def snapshot(self) -> dict:
        with self.lock:
            return {
                'running': self.running,
                'done': self.done,
                'error': self.error,
                'step_count': self.step_count,
                'start_pos': self.start_pos,
                'target_pos': self.target_pos,
                'arrivals': list(self.arrivals),
                'latest': dict(self.latest) if self.latest else None,
            }


class CraneLiveServer(ThreadingHTTPServer):
    def __init__(self, server_address, payload, payload_factory=None, plc=None,
                 ros_source=None, plc_actuator=None, config=None,
                 initial_pos=None, update_hz=10.0, speed=1.0):
        super().__init__(server_address, _LiveRequestHandler)
        self.payload = payload
        self.payload_factory = payload_factory
        self.plc = plc  # PLC instance for interactive control (stop/reset)
        # PLC real-time control mode
        self.ros_source: RosPositionSource | None = ros_source
        self.plc_actuator: PlcActuator | None = plc_actuator
        self.config: CraneConfig | None = config
        self.initial_pos: tuple | None = initial_pos
        self.update_hz: float = update_hz
        self.speed: float = speed
        self.control_hooks: LiveControlHooks | None = None
        self.control_thread: threading.Thread | None = None
        self.control_state: ControlState | None = None

    def build_payload(self, query: dict[str, list[str]]) -> dict[str, Any]:
        if self.payload_factory is None:
            return self.payload
        return self.payload_factory(query)


def _find_available_port(host: str, preferred_port: int) -> int:
    if preferred_port == 0:
        return 0
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        if sock.connect_ex((host, preferred_port)) != 0:
            return preferred_port
    return 0


def serve_live_view(
    payload: dict[str, Any] | None,
    host: str = '127.0.0.1',
    port: int = 8000,
    payload_factory=None,
    plc=None,
    ros_source=None,
    plc_actuator=None,
    config=None,
    initial_pos=None,
    update_hz: float = 10.0,
    speed: float = 1.0,
):
    """Start a blocking browser live-view server.

    Simulation mode (ros_source=None, plc_actuator=None):
      payload and payload_factory control what the browser replays.

    PLC mode (ros_source and plc_actuator provided):
      payload should be None.  The browser shows live /localization_pose
      data and the user starts control via the Apply Target button.
    """
    selected_port = _find_available_port(host, port)
    server = CraneLiveServer(
        (host, selected_port), payload,
        payload_factory=payload_factory, plc=plc,
        ros_source=ros_source, plc_actuator=plc_actuator,
        config=config, initial_pos=initial_pos,
        update_hz=update_hz, speed=speed,
    )
    actual_host, actual_port = server.server_address
    url = f'http://{actual_host}:{actual_port}'
    print(f'Live view: {url}')
    print('Press Ctrl+C to stop.')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nLive view stopped.')
    finally:
        server.server_close()
