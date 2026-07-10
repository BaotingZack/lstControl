"""Browser-based 10 Hz live view for the crane simulation."""

from __future__ import annotations

import json
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse

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
        'phaseBoundaries': [
            {'t': round(t, 3), 'axis': axis, 'label': f'{axis.upper()} arrived'}
            for t, axis in phase_boundaries
        ],
        'frames': frames,
    }


def render_live_html() -> str:
    """Return the browser live view HTML."""
    return """<!doctype html>
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
    .timeline {
      padding: 14px;
    }
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
      <section class="timeline">
        <div class="track"><div class="progress" id="progress"></div></div>
        <div class="ticks"><span id="startTime">0.0s</span><span id="endTime">0.0s</span></div>
        <div class="log" id="phaseLog"></div>
      </section>
    </aside>
  </main>
  <script>
    const canvas = document.getElementById('scene');
    const ctx = canvas.getContext('2d');
    let payload;
    let frame = 0;
    let lastAdvance = 0;

    const els = {
      rate: document.getElementById('rate'),
      time: document.getElementById('time'),
      frame: document.getElementById('frame'),
      speed: document.getElementById('speed'),
      phase: document.getElementById('phase'),
      x: document.getElementById('x'),
      y: document.getElementById('y'),
      z: document.getElementById('z'),
      targetX: document.getElementById('targetX'),
      targetY: document.getElementById('targetY'),
      targetZ: document.getElementById('targetZ'),
      applyTarget: document.getElementById('applyTarget'),
      progress: document.getElementById('progress'),
      startTime: document.getElementById('startTime'),
      endTime: document.getElementById('endTime'),
      phaseLog: document.getElementById('phaseLog'),
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
      if (!payload) return;
      const rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);
      const r = ranges();
      const current = payload.frames[frame];
      const boxes = layout(rect);
      drawCraneBay(boxes.bay, r, current);
      drawTrolleyCloseup(boxes.closeup, r, current);
      drawHoistProfile(boxes.zbox, r, current);
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
    }

    function renderTargetControls() {
      els.targetX.value = payload.target.x.toFixed(2);
      els.targetY.value = payload.target.y.toFixed(2);
      els.targetZ.value = payload.target.z.toFixed(2);
    }

    function applyTargetCommand() {
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

    window.addEventListener('resize', resize);
  </script>
</body>
</html>"""


class _LiveRequestHandler(BaseHTTPRequestHandler):
    def _write(self, status: int, content_type: str, body: bytes):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ('/', '/index.html'):
            self._write(200, 'text/html; charset=utf-8', render_live_html().encode('utf-8'))
        elif parsed.path == '/simulation.json':
            try:
                payload = self.server.build_payload(parse_qs(parsed.query))
            except ValueError as exc:
                body = json.dumps({'error': str(exc)}, ensure_ascii=False).encode('utf-8')
                self._write(400, 'application/json; charset=utf-8', body)
                return
            body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
            self._write(200, 'application/json; charset=utf-8', body)
        else:
            self._write(404, 'text/plain; charset=utf-8', b'not found')

    def log_message(self, format, *args):
        return


class CraneLiveServer(ThreadingHTTPServer):
    def __init__(self, server_address, payload, payload_factory=None):
        super().__init__(server_address, _LiveRequestHandler)
        self.payload = payload
        self.payload_factory = payload_factory

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
    payload: dict[str, Any],
    host: str = '127.0.0.1',
    port: int = 8000,
    payload_factory=None,
):
    """Start a blocking browser live-view server."""
    selected_port = _find_available_port(host, port)
    server = CraneLiveServer((host, selected_port), payload, payload_factory=payload_factory)
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
