"""Browser-based 10 Hz live view for the crane simulation.

Supports two rendering modes:

  Simulation (replay):
    Simulation runs to completion first, then the browser replays the
    recorded history at 10 Hz.  URL query parameters trigger new runs.

  PLC (real-time):
    The browser shows live /localization_pose data on open.  The user
    sets a target and clicks "Apply Target" to start PD control.
    A background thread runs the control loop while the browser polls
    the latest thread-safe state; SSE remains available for diagnostics.
"""

from __future__ import annotations

import json
import math
import queue
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse

from calibration import (
    CalibrationObservation,
    CalibrationResult,
    calibrate_map_to_crane,
)
from coordinate_transform import CoordinateTransform2D
from crane_model import (
    ControlHooks,
    ControlStoppedError,
    CraneConfig,
    CraneState,
    PositionFeedbackTimeout,
    run_pd_control,
)
from ros_bridge import get_latest_pose, RosPositionSource
from plc_interface import PlcActuator
from visualizer import CraneVisualizer


_MAX_CONTROL_BODY_BYTES = 4096
_MAX_CALIBRATION_BODY_BYTES = 8192


def _calibrate_from_request(body: str) -> CalibrationResult:
    """Parse browser observations and calculate a map-to-crane transform."""
    try:
        payload = json.loads(body)
        if not isinstance(payload, dict):
            raise TypeError('payload must be a JSON object')

        def _point(name: str) -> tuple[float, float, float]:
            point = payload[name]
            if not isinstance(point, dict):
                raise TypeError(f'{name} must be an object with x, y, and z')
            return float(point['x']), float(point['y']), float(point['z'])

        observation = CalibrationObservation(
            start_map=_point('start'),
            after_forward_map=_point('afterForward'),
            after_lateral_map=_point('afterLateral'),
            forward_distance=float(payload['forwardDistance']),
            lateral_distance=float(payload['lateralDistance']),
        )
        max_error = float(payload.get('maxOrthogonalityErrorDeg', 15.0))
        return calibrate_map_to_crane(
            observation,
            max_orthogonality_error_deg=max_error,
        )
    except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
        raise ValueError(f'Invalid calibration: {exc}') from exc


def render_calibration_html() -> str:
    """Return the interactive SLAM-map to crane-rail calibration lab."""
    return """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Coordinate Calibration Lab</title>
  <style>
    :root {
      --ink: #eaf2f5;
      --muted: #8ea2ad;
      --void: #091014;
      --deck: #101a20;
      --panel: #142128;
      --raised: #1a2a32;
      --line: #2c414b;
      --faint: #1c3038;
      --amber: #f4b942;
      --cyan: #49d6d0;
      --green: #79d987;
      --red: #ef705d;
      --blue: #5caee8;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      color: var(--ink);
      background:
        linear-gradient(rgba(73, 214, 208, 0.025) 1px, transparent 1px),
        linear-gradient(90deg, rgba(73, 214, 208, 0.025) 1px, transparent 1px),
        radial-gradient(circle at 74% 8%, #18303a 0, transparent 34%),
        var(--void);
      background-size: 24px 24px, 24px 24px, auto, auto;
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
    }
    button, input { font: inherit; }
    .shell { min-height: 100vh; display: grid; grid-template-rows: auto 1fr; }
    header {
      min-height: 68px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 20px;
      padding: 12px 22px;
      border-bottom: 1px solid var(--line);
      background: rgba(9, 16, 20, 0.9);
    }
    .brand { display: flex; align-items: center; gap: 14px; }
    .mark {
      width: 36px; height: 36px; position: relative;
      border: 1px solid var(--cyan); transform: rotate(45deg);
      box-shadow: inset 0 0 0 7px var(--void), inset 0 0 0 8px var(--amber);
    }
    .eyebrow { color: var(--cyan); font-size: 10px; letter-spacing: .18em; text-transform: uppercase; }
    h1 { margin: 3px 0 0; font-family: "Arial Narrow", sans-serif; font-size: 22px; letter-spacing: .04em; }
    header nav { display: flex; align-items: center; gap: 14px; }
    header a { color: var(--muted); text-decoration: none; font-size: 12px; }
    header a:hover { color: var(--ink); }
    .live-dot { display: inline-flex; align-items: center; gap: 7px; color: var(--green); font-size: 11px; }
    .live-dot::before { content: ""; width: 7px; height: 7px; border-radius: 50%; background: currentColor; box-shadow: 0 0 10px currentColor; }
    main { display: grid; grid-template-columns: minmax(560px, 1.5fr) minmax(360px, .75fr); gap: 14px; padding: 14px; min-height: 0; }
    .visual, .controls { border: 1px solid var(--line); background: rgba(16, 26, 32, .96); }
    .visual { display: grid; grid-template-rows: auto 1fr auto; min-height: 760px; }
    .section-head {
      display: flex; align-items: center; justify-content: space-between; gap: 12px;
      padding: 12px 15px; border-bottom: 1px solid var(--line); background: var(--panel);
    }
    .section-title { display: flex; align-items: center; gap: 10px; font-size: 12px; letter-spacing: .09em; text-transform: uppercase; }
    .section-title::before { content: "//"; color: var(--amber); }
    .mode { color: var(--muted); font-size: 10px; }
    .canvas-wrap { position: relative; min-height: 620px; }
    canvas { display: block; width: 100%; height: 100%; min-height: 620px; }
    .legend {
      display: flex; flex-wrap: wrap; gap: 18px; padding: 10px 15px;
      color: var(--muted); font-size: 10px; border-top: 1px solid var(--line);
    }
    .legend span { display: inline-flex; align-items: center; gap: 7px; }
    .legend i { width: 18px; height: 2px; background: var(--amber); }
    .legend .measured { background: var(--green); }
    .legend .corrected { background: var(--cyan); }
    .controls { overflow: auto; }
    .block { padding: 14px; border-bottom: 1px solid var(--line); }
    .block:last-child { border-bottom: 0; }
    .block-title { display: flex; align-items: center; justify-content: space-between; margin-bottom: 12px; }
    .block-title strong { font-size: 11px; letter-spacing: .1em; text-transform: uppercase; }
    .step-no { color: var(--amber); font-size: 10px; }
    .form-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 9px; }
    label { display: grid; gap: 5px; color: var(--muted); font-size: 10px; }
    label.wide { grid-column: 1 / -1; }
    input {
      min-width: 0; width: 100%; padding: 9px 10px; color: var(--ink);
      border: 1px solid var(--line); border-radius: 2px; background: #0a1318;
      font-variant-numeric: tabular-nums;
    }
    input:focus { outline: none; border-color: var(--cyan); box-shadow: 0 0 0 1px rgba(73,214,208,.25); }
    .run-label { grid-column: 1 / -1; color: var(--blue); margin-top: 4px; font-size: 10px; letter-spacing: .09em; text-transform: uppercase; }
    .point-row { grid-column: 1 / -1; display: grid; grid-template-columns: 74px repeat(3, 1fr); gap: 8px; align-items: end; }
    .point-row > span { align-self: center; color: var(--ink); font-size: 10px; }
    .actions { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    button {
      min-height: 39px; border: 1px solid var(--line); border-radius: 2px;
      color: var(--ink); background: var(--raised); cursor: pointer; transition: .15s ease;
      font-size: 11px; font-weight: 700; letter-spacing: .04em;
    }
    button:hover { border-color: var(--cyan); transform: translateY(-1px); }
    button.primary { color: #071114; border-color: var(--amber); background: var(--amber); }
    button.primary:hover { filter: brightness(1.08); }
    button.calibrate { color: #071114; border-color: var(--cyan); background: var(--cyan); }
    button:disabled { opacity: .55; cursor: wait; transform: none; }
    .status-line {
      display: flex; align-items: center; justify-content: space-between; gap: 10px;
      margin-bottom: 11px; padding: 9px 10px; border: 1px solid var(--line); background: #0b151a;
    }
    .status-line span:first-child { color: var(--muted); font-size: 10px; }
    .quality { color: var(--muted); font-size: 11px; font-weight: 700; }
    .quality.pass { color: var(--green); }
    .quality.warn, .quality.error { color: var(--red); }
    .metrics { display: grid; grid-template-columns: 1fr 1fr; border: 1px solid var(--line); }
    .metric { min-width: 0; padding: 10px; border-right: 1px solid var(--line); border-bottom: 1px solid var(--line); }
    .metric:nth-child(2n) { border-right: 0; }
    .metric:nth-last-child(-n + 2) { border-bottom: 0; }
    .metric span { display: block; color: var(--muted); font-size: 9px; letter-spacing: .06em; text-transform: uppercase; }
    .metric strong { display: block; margin-top: 6px; color: var(--ink); font-size: 16px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
    .cli {
      min-height: 76px; margin-top: 10px; padding: 10px; color: var(--cyan); background: #071014;
      border: 1px dashed #35515c; font-size: 10px; line-height: 1.55; word-break: break-all; user-select: all;
    }
    .note { margin: 10px 0 0; color: var(--muted); font-family: sans-serif; font-size: 11px; line-height: 1.55; }
    @media (max-width: 980px) {
      main { grid-template-columns: 1fr; }
      .visual { min-height: 620px; }
    }
    @media (max-width: 600px) {
      header { align-items: flex-start; }
      header nav { flex-direction: column; align-items: flex-end; gap: 6px; }
      main { padding: 8px; }
      .visual { min-height: 560px; }
      .canvas-wrap, canvas { min-height: 460px; }
      .point-row { grid-template-columns: 58px repeat(3, 1fr); }
    }
  </style>
</head>
<body>
  <div class="shell">
    <header>
      <div class="brand">
        <div class="mark" aria-hidden="true"></div>
        <div><div class="eyebrow">SLAM / Rail frame alignment</div><h1>Coordinate Calibration Lab</h1></div>
      </div>
      <nav><span class="live-dot">SIM READY</span><a href="/">返回起重机控制台 →</a></nav>
    </header>
    <main>
      <section class="visual">
        <div class="section-head">
          <div class="section-title">Map deviation &amp; correction</div>
          <div class="mode" id="motionPhase">IDLE / 等待仿真</div>
        </div>
        <div class="canvas-wrap"><canvas id="calibrationCanvas" aria-label="标定过程仿真画布"></canvas></div>
        <div class="legend">
          <span><i></i>物理行车轨道</span><span><i class="measured"></i>SLAM 测量轨迹</span><span><i class="corrected"></i>标定后坐标</span>
        </div>
      </section>
      <aside class="controls">
        <section class="block">
          <div class="block-title"><strong>仿真工况</strong><span class="step-no">SETUP</span></div>
          <div class="form-grid">
            <label>地图偏转角 / DEG<input id="simYaw" type="number" step="0.1" value="17"></label>
            <label>测量噪声 / M<input id="simNoise" type="number" min="0" step="0.001" value="0.008"></label>
            <label>地图横滚 Roll / DEG<input id="simRoll" type="number" step="0.1" value="2.5"></label>
            <label>地图俯仰 Pitch / DEG<input id="simPitch" type="number" step="0.1" value="-4"></label>
            <label>地图原点 X / M<input id="originX" type="number" step="0.1" value="12.5"></label>
            <label>地图原点 Y / M<input id="originY" type="number" step="0.1" value="-4"></label>
            <label>地图原点 Z / M<input id="originZ" type="number" step="0.1" value="1.2"></label>
            <label>Forward Run / 大车 X / M<input id="forwardDistance" type="number" step="0.1" value="6"></label>
            <label>Lateral Run / 小车 Y / M<input id="lateralDistance" type="number" step="0.1" value="3"></label>
          </div>
        </section>
        <section class="block">
          <div class="block-title"><strong>SLAM 观测点</strong><span class="step-no">01—03</span></div>
          <div class="form-grid">
            <div class="point-row"><span>START</span><label>Map X<input id="startX" type="number" step="0.001"></label><label>Map Y<input id="startY" type="number" step="0.001"></label><label>Map Z<input id="startZ" type="number" step="0.001"></label></div>
            <div class="run-label">Forward Run / 沿大车轨道移动已知距离</div>
            <div class="point-row"><span>AFTER X</span><label>Map X<input id="forwardX" type="number" step="0.001"></label><label>Map Y<input id="forwardY" type="number" step="0.001"></label><label>Map Z<input id="forwardZ" type="number" step="0.001"></label></div>
            <div class="run-label">Lateral Run / 沿小车轨道移动已知距离</div>
            <div class="point-row"><span>AFTER Y</span><label>Map X<input id="lateralX" type="number" step="0.001"></label><label>Map Y<input id="lateralY" type="number" step="0.001"></label><label>Map Z<input id="lateralZ" type="number" step="0.001"></label></div>
          </div>
        </section>
        <section class="block">
          <div class="actions">
            <button class="primary" id="simulateBtn" type="button">▶ SIMULATE RUN</button>
            <button class="calibrate" id="calibrateBtn" type="button">CALIBRATE / 标定</button>
            <button id="resetBtn" type="button">RESET</button>
            <button id="copyBtn" type="button">COPY CLI</button>
          </div>
        </section>
        <section class="block">
          <div class="block-title"><strong>标定结果</strong><span class="step-no">RESULT</span></div>
          <div class="status-line"><span>CALIBRATION QUALITY</span><span class="quality" id="quality">NOT CALIBRATED</span></div>
          <div class="metrics">
            <div class="metric"><span>Origin map X</span><strong id="resultOriginX">—</strong></div>
            <div class="metric"><span>Origin map Y</span><strong id="resultOriginY">—</strong></div>
            <div class="metric"><span>Origin map Z</span><strong id="resultOriginZ">—</strong></div>
            <div class="metric"><span>Rail yaw</span><strong id="resultYaw">—</strong></div>
            <div class="metric"><span>Map roll</span><strong id="resultRoll">—</strong></div>
            <div class="metric"><span>Map pitch</span><strong id="resultPitch">—</strong></div>
            <div class="metric"><span>GROUND TILT</span><strong id="resultTilt">—</strong></div>
            <div class="metric"><span>Orthogonality</span><strong id="resultOrth">—</strong></div>
            <div class="metric"><span>X / Y scale</span><strong id="resultScale">—</strong></div>
            <div class="metric"><span>Residual RMS</span><strong id="resultRms">—</strong></div>
          </div>
          <div class="cli" id="cliOutput">--map-to-crane-origin-x … --map-to-crane-origin-y … --map-to-crane-origin-z … --map-to-crane-roll-deg … --map-to-crane-pitch-deg … --map-to-crane-yaw-deg …</div>
          <p class="note">两段三维 SLAM 轨迹确定物理 +X/+Y 轨道，叉乘得到真实地面法向 +Z。控制内部会同时修正地图的 roll、pitch 和 yaw；不需要额外移动吊钩。</p>
        </section>
      </aside>
    </main>
  </div>
  <script>
    const canvas = document.getElementById('calibrationCanvas');
    const ctx = canvas.getContext('2d');
    const ids = [
      'simYaw', 'simRoll', 'simPitch', 'simNoise', 'originX', 'originY', 'originZ',
      'forwardDistance', 'lateralDistance', 'startX', 'startY', 'startZ',
      'forwardX', 'forwardY', 'forwardZ', 'lateralX', 'lateralY', 'lateralZ'
    ];
    const fields = Object.fromEntries(ids.map(id => [id, document.getElementById(id)]));
    const ui = {
      phase: document.getElementById('motionPhase'), quality: document.getElementById('quality'),
      originX: document.getElementById('resultOriginX'), originY: document.getElementById('resultOriginY'),
      originZ: document.getElementById('resultOriginZ'), roll: document.getElementById('resultRoll'),
      pitch: document.getElementById('resultPitch'), yaw: document.getElementById('resultYaw'),
      tilt: document.getElementById('resultTilt'), orth: document.getElementById('resultOrth'),
      scale: document.getElementById('resultScale'), rms: document.getElementById('resultRms'),
      cli: document.getElementById('cliOutput'), simulate: document.getElementById('simulateBtn'),
      calibrate: document.getElementById('calibrateBtn'), copy: document.getElementById('copyBtn')
    };
    let measured = null;
    let calibration = null;
    let animation = { active: false, start: 0, progress: 1 };

    const number = id => Number(fields[id].value);
    const rotate = (x, y, yaw) => ({x: Math.cos(yaw) * x - Math.sin(yaw) * y, y: Math.sin(yaw) * x + Math.cos(yaw) * y});
    const add = (a, b) => ({x: a.x + b.x, y: a.y + b.y});
    const add3 = (a, b) => ({x: a.x + b.x, y: a.y + b.y, z: a.z + b.z});
    const fmt = value => Number(value).toFixed(3);

    function rotate3(point, roll, pitch, yaw) {
      const cr = Math.cos(roll), sr = Math.sin(roll);
      const cp = Math.cos(pitch), sp = Math.sin(pitch);
      const cy = Math.cos(yaw), sy = Math.sin(yaw);
      return {
        x: cy * cp * point.x + (cy * sp * sr - sy * cr) * point.y + (cy * sp * cr + sy * sr) * point.z,
        y: sy * cp * point.x + (sy * sp * sr + cy * cr) * point.y + (sy * sp * cr - cy * sr) * point.z,
        z: -sp * point.x + cp * sr * point.y + cp * cr * point.z
      };
    }

    function randomNoise(amplitude) {
      return amplitude ? (Math.random() * 2 - 1) * amplitude : 0;
    }

    function writePoint(prefix, point) {
      fields[prefix + 'X'].value = fmt(point.x);
      fields[prefix + 'Y'].value = fmt(point.y);
      fields[prefix + 'Z'].value = fmt(point.z);
    }

    function readMeasured() {
      return {
        start: {x: number('startX'), y: number('startY'), z: number('startZ')},
        forward: {x: number('forwardX'), y: number('forwardY'), z: number('forwardZ')},
        lateral: {x: number('lateralX'), y: number('lateralY'), z: number('lateralZ')}
      };
    }

    function simulate() {
      const yaw = number('simYaw') * Math.PI / 180;
      const roll = number('simRoll') * Math.PI / 180;
      const pitch = number('simPitch') * Math.PI / 180;
      const noise = Math.max(0, number('simNoise'));
      const origin = {x: number('originX'), y: number('originY'), z: number('originZ')};
      const perturb = point => ({x: point.x + randomNoise(noise), y: point.y + randomNoise(noise), z: point.z + randomNoise(noise)});
      const start = perturb(origin);
      const idealForward = add3(origin, rotate3({x: number('forwardDistance'), y: 0, z: 0}, roll, pitch, yaw));
      const idealLateral = add3(idealForward, rotate3({x: 0, y: number('lateralDistance'), z: 0}, roll, pitch, yaw));
      measured = {start, forward: perturb(idealForward), lateral: perturb(idealLateral)};
      writePoint('start', measured.start);
      writePoint('forward', measured.forward);
      writePoint('lateral', measured.lateral);
      calibration = null;
      clearResult();
      animation = {active: true, start: performance.now(), progress: 0};
      ui.phase.textContent = 'FORWARD RUN / 大车移动';
    }

    function clearResult() {
      ui.quality.className = 'quality';
      ui.quality.textContent = 'NOT CALIBRATED';
      [ui.originX, ui.originY, ui.originZ, ui.roll, ui.pitch, ui.yaw, ui.tilt, ui.orth, ui.scale, ui.rms].forEach(el => el.textContent = '—');
      ui.cli.textContent = '--map-to-crane-origin-x … --map-to-crane-origin-y … --map-to-crane-origin-z … --map-to-crane-roll-deg … --map-to-crane-pitch-deg … --map-to-crane-yaw-deg …';
    }

    async function calibrate() {
      measured = readMeasured();
      ui.calibrate.disabled = true;
      ui.quality.className = 'quality';
      ui.quality.textContent = 'CALCULATING…';
      try {
        const response = await fetch('/api/calibrate', {
          method: 'POST', headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({
            start: measured.start,
            afterForward: measured.forward,
            afterLateral: measured.lateral,
            forwardDistance: number('forwardDistance'),
            lateralDistance: number('lateralDistance')
          })
        });
        const result = await response.json();
        if (!response.ok) throw new Error(result.error || 'Calibration failed');
        calibration = result;
        showResult(result);
        ui.phase.textContent = 'ALIGNED / 标定完成';
      } catch (error) {
        calibration = null;
        ui.quality.className = 'quality error';
        ui.quality.textContent = error.message;
        ui.phase.textContent = 'INVALID OBSERVATION';
      } finally {
        ui.calibrate.disabled = false;
      }
    }

    function showResult(result) {
      const scaleError = Math.max(Math.abs(result.forwardScale - 1), Math.abs(result.lateralScale - 1));
      const pass = result.orthogonalityErrorDeg <= 3 && scaleError <= .03 && result.residualRms <= .05;
      ui.quality.className = 'quality ' + (pass ? 'pass' : 'warn');
      ui.quality.textContent = pass ? 'PASS / 可用' : 'CHECK / 建议复测';
      ui.originX.textContent = fmt(result.transform.originMapX) + ' m';
      ui.originY.textContent = fmt(result.transform.originMapY) + ' m';
      ui.originZ.textContent = fmt(result.transform.originMapZ) + ' m';
      ui.roll.textContent = fmt(result.transform.craneRollDeg) + '°';
      ui.pitch.textContent = fmt(result.transform.cranePitchDeg) + '°';
      ui.yaw.textContent = fmt(result.transform.craneXAxisYawDeg) + '°';
      ui.tilt.textContent = fmt(result.groundTiltDeg) + '°';
      ui.orth.textContent = fmt(result.orthogonalityErrorDeg) + '°';
      ui.scale.textContent = result.forwardScale.toFixed(4) + ' / ' + result.lateralScale.toFixed(4);
      ui.rms.textContent = fmt(result.residualRms) + ' m';
      ui.cli.textContent = result.cliArgs;
    }

    function boundsFor(points) {
      const xs = points.map(p => p.x), ys = points.map(p => p.y);
      const dx = Math.max(2, Math.max(...xs) - Math.min(...xs));
      const dy = Math.max(2, Math.max(...ys) - Math.min(...ys));
      const pad = Math.max(dx, dy) * .28;
      return {minX: Math.min(...xs) - pad, maxX: Math.max(...xs) + pad, minY: Math.min(...ys) - pad, maxY: Math.max(...ys) + pad};
    }

    function mapper(box, bounds) {
      const sx = box.w / (bounds.maxX - bounds.minX);
      const sy = box.h / (bounds.maxY - bounds.minY);
      const scale = Math.min(sx, sy);
      const usedW = (bounds.maxX - bounds.minX) * scale;
      const usedH = (bounds.maxY - bounds.minY) * scale;
      return point => ({
        x: box.x + (box.w - usedW) / 2 + (point.x - bounds.minX) * scale,
        y: box.y + (box.h + usedH) / 2 - (point.y - bounds.minY) * scale
      });
    }

    function grid(box, map, bounds, label) {
      ctx.save(); ctx.beginPath(); ctx.rect(box.x, box.y, box.w, box.h); ctx.clip();
      ctx.fillStyle = '#0b151a'; ctx.fillRect(box.x, box.y, box.w, box.h);
      ctx.strokeStyle = '#1d3038'; ctx.lineWidth = 1;
      const step = Math.max(1, Math.ceil(Math.max(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY) / 8));
      for (let x = Math.floor(bounds.minX / step) * step; x <= bounds.maxX; x += step) {
        const p = map({x, y: 0}); ctx.beginPath(); ctx.moveTo(p.x, box.y); ctx.lineTo(p.x, box.y + box.h); ctx.stroke();
      }
      for (let y = Math.floor(bounds.minY / step) * step; y <= bounds.maxY; y += step) {
        const p = map({x: 0, y}); ctx.beginPath(); ctx.moveTo(box.x, p.y); ctx.lineTo(box.x + box.w, p.y); ctx.stroke();
      }
      ctx.restore();
      ctx.strokeStyle = '#2c414b'; ctx.strokeRect(box.x + .5, box.y + .5, box.w - 1, box.h - 1);
      ctx.fillStyle = '#8ea2ad'; ctx.font = '700 10px monospace'; ctx.fillText(label, box.x + 12, box.y + 20);
    }

    function line(a, b, color, width = 2, dash = []) {
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y);
      ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash); ctx.stroke(); ctx.setLineDash([]);
    }

    function axis(origin, yaw, length, map, alpha = 1) {
      const xEnd = add(origin, rotate(length, 0, yaw));
      const yEnd = add(origin, rotate(0, length, yaw));
      line(map(origin), map(xEnd), `rgba(244,185,66,${alpha})`, 3);
      line(map(origin), map(yEnd), `rgba(92,174,232,${alpha})`, 3);
      const xo = map(xEnd), yo = map(yEnd);
      ctx.fillStyle = '#f4b942'; ctx.fillText('+X RAIL', xo.x + 5, xo.y - 4);
      ctx.fillStyle = '#5caee8'; ctx.fillText('+Y RAIL', yo.x + 5, yo.y - 4);
    }

    function axis3(origin, roll, pitch, yaw, length, map, alpha = 1) {
      const xEnd = add3(origin, rotate3({x: length, y: 0, z: 0}, roll, pitch, yaw));
      const yEnd = add3(origin, rotate3({x: 0, y: length, z: 0}, roll, pitch, yaw));
      line(map(origin), map(xEnd), `rgba(244,185,66,${alpha})`, 3);
      line(map(origin), map(yEnd), `rgba(92,174,232,${alpha})`, 3);
      const xo = map(xEnd), yo = map(yEnd);
      ctx.fillStyle = '#f4b942'; ctx.fillText('+X RAIL', xo.x + 5, xo.y - 4);
      ctx.fillStyle = '#5caee8'; ctx.fillText('+Y RAIL', yo.x + 5, yo.y - 4);
    }

    function dot(point, map, color, label) {
      const p = map(point); ctx.beginPath(); ctx.arc(p.x, p.y, 5, 0, Math.PI * 2);
      ctx.fillStyle = color; ctx.fill(); ctx.strokeStyle = '#f4f8fa'; ctx.lineWidth = 1.5; ctx.stroke();
      ctx.fillStyle = color; ctx.font = '700 9px monospace'; ctx.fillText(label, p.x + 8, p.y - 7);
    }

    function interpolatePath(progress) {
      if (!measured) return null;
      if (progress < .58) {
        const t = progress / .58;
        return {
          x: measured.start.x + (measured.forward.x - measured.start.x) * t,
          y: measured.start.y + (measured.forward.y - measured.start.y) * t,
          z: measured.start.z + (measured.forward.z - measured.start.z) * t
        };
      }
      const t = (progress - .58) / .42;
      return {
        x: measured.forward.x + (measured.lateral.x - measured.forward.x) * t,
        y: measured.forward.y + (measured.lateral.y - measured.forward.y) * t,
        z: measured.forward.z + (measured.lateral.z - measured.forward.z) * t
      };
    }

    function drawMapPanel(box) {
      const yaw = number('simYaw') * Math.PI / 180;
      const roll = number('simRoll') * Math.PI / 180;
      const pitch = number('simPitch') * Math.PI / 180;
      const origin = {x: number('originX'), y: number('originY'), z: number('originZ')};
      const idealForward = add3(origin, rotate3({x: number('forwardDistance'), y: 0, z: 0}, roll, pitch, yaw));
      const fallback = [origin, idealForward, add3(idealForward, rotate3({x: 0, y: number('lateralDistance'), z: 0}, roll, pitch, yaw))];
      const points = measured ? [measured.start, measured.forward, measured.lateral] : fallback;
      const rail = add3(origin, rotate3({x: Math.max(Math.abs(number('forwardDistance')), Math.abs(number('lateralDistance'))) * 1.25, y: 0, z: 0}, roll, pitch, yaw));
      const bounds = boundsFor([...points, origin, rail]);
      const map = mapper({x: box.x + 10, y: box.y + 32, w: box.w - 20, h: box.h - 42}, bounds);
      grid(box, map, bounds, 'RAW SLAM MAP / 地图坐标未对齐');
      axis3(origin, roll, pitch, yaw, Math.max(3, Math.abs(number('forwardDistance')) * 1.1), map, .85);
      if (measured) {
        line(map(measured.start), map(measured.forward), '#79d987', 3);
        if (animation.progress > .58) line(map(measured.forward), map(measured.lateral), '#79d987', 3);
        dot(measured.start, map, '#eaf2f5', 'START');
        if (animation.progress >= .58) dot(measured.forward, map, '#79d987', 'X');
        if (animation.progress >= 1) dot(measured.lateral, map, '#79d987', 'Y');
        const car = interpolatePath(animation.progress);
        if (car) dot(car, map, '#ef705d', 'CAR');
      }
      if (calibration) {
        const estimatedOrigin = {x: calibration.transform.originMapX, y: calibration.transform.originMapY, z: calibration.transform.originMapZ};
        axis3(
          estimatedOrigin,
          calibration.transform.craneRollDeg * Math.PI / 180,
          calibration.transform.cranePitchDeg * Math.PI / 180,
          calibration.transform.craneXAxisYawDeg * Math.PI / 180,
          Math.max(3, Math.abs(number('forwardDistance'))), map, .6
        );
      }
    }

    function mapToCrane(point) {
      if (!calibration) return point;
      const roll = calibration.transform.craneRollDeg * Math.PI / 180;
      const pitch = calibration.transform.cranePitchDeg * Math.PI / 180;
      const yaw = calibration.transform.craneXAxisYawDeg * Math.PI / 180;
      const cr = Math.cos(roll), sr = Math.sin(roll);
      const cp = Math.cos(pitch), sp = Math.sin(pitch);
      const cy = Math.cos(yaw), sy = Math.sin(yaw);
      const dx = point.x - calibration.transform.originMapX;
      const dy = point.y - calibration.transform.originMapY;
      const dz = point.z - calibration.transform.originMapZ;
      return {
        x: cy * cp * dx + sy * cp * dy - sp * dz,
        y: (cy * sp * sr - sy * cr) * dx + (sy * sp * sr + cy * cr) * dy + cp * sr * dz,
        z: (cy * sp * cr + sy * sr) * dx + (sy * sp * cr - cy * sr) * dy + cp * cr * dz
      };
    }

    function drawCorrectedPanel(box) {
      const corrected = measured && calibration ? [mapToCrane(measured.start), mapToCrane(measured.forward), mapToCrane(measured.lateral)] : [{x:0,y:0,z:0}, {x:number('forwardDistance'),y:0,z:0}, {x:number('forwardDistance'),y:number('lateralDistance'),z:0}];
      const bounds = boundsFor(corrected);
      const map = mapper({x: box.x + 10, y: box.y + 32, w: box.w - 20, h: box.h - 42}, bounds);
      grid(box, map, bounds, calibration ? 'CALIBRATED CRANE FRAME / 轨道坐标已对正' : 'CALIBRATED CRANE FRAME / WAITING');
      line(map({x: bounds.minX, y: 0}), map({x: bounds.maxX, y: 0}), '#f4b942', 2);
      line(map({x: 0, y: bounds.minY}), map({x: 0, y: bounds.maxY}), '#5caee8', 2);
      ctx.fillStyle = '#f4b942'; ctx.fillText('CRANE +X →', box.x + box.w - 105, box.y + box.h - 12);
      ctx.fillStyle = '#5caee8'; ctx.fillText('CRANE +Y ↑', box.x + 12, box.y + 48);
      if (measured && calibration) {
        line(map(corrected[0]), map(corrected[1]), '#49d6d0', 3);
        line(map(corrected[1]), map(corrected[2]), '#49d6d0', 3);
        dot(corrected[0], map, '#eaf2f5', '(0,0)'); dot(corrected[1], map, '#49d6d0', 'X'); dot(corrected[2], map, '#49d6d0', 'Y');
      } else {
        ctx.fillStyle = '#607681'; ctx.font = '11px monospace'; ctx.fillText('运行仿真并点击 CALIBRATE', box.x + 20, box.y + box.h / 2);
      }
    }

    function drawElevationPanel(box) {
      ctx.fillStyle = '#0b151a'; ctx.fillRect(box.x, box.y, box.w, box.h);
      ctx.strokeStyle = '#2c414b'; ctx.strokeRect(box.x + .5, box.y + .5, box.w - 1, box.h - 1);
      ctx.fillStyle = '#8ea2ad'; ctx.font = '700 10px monospace';
      ctx.fillText('GROUND PLANE Z / 水平移动时的地图高程漂移', box.x + 12, box.y + 20);
      if (!measured) return;

      const raw = [measured.start, measured.forward, measured.lateral];
      const corrected = calibration ? raw.map(mapToCrane) : null;
      const values = raw.map(point => point.z).concat(corrected ? corrected.map(point => point.z) : [0]);
      let minZ = Math.min(...values), maxZ = Math.max(...values);
      const range = Math.max(.08, maxZ - minZ);
      minZ -= range * .25; maxZ += range * .25;
      const left = box.x + 46, right = box.x + box.w - 22;
      const top = box.y + 35, bottom = box.y + box.h - 28;
      const sx = phase => left + phase * (right - left) / 2;
      const sy = z => bottom - (z - minZ) * (bottom - top) / (maxZ - minZ);

      ctx.strokeStyle = '#1d3038'; ctx.lineWidth = 1;
      for (let index = 0; index <= 4; index++) {
        const y = top + (bottom - top) * index / 4;
        ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(right, y); ctx.stroke();
      }
      ['START', 'AFTER X', 'AFTER Y'].forEach((label, index) => {
        const x = sx(index);
        ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, bottom); ctx.stroke();
        ctx.fillStyle = '#607681'; ctx.fillText(label, x - 18, bottom + 17);
      });
      const rawScreen = raw.map((point, index) => ({x: sx(index), y: sy(point.z)}));
      line(rawScreen[0], rawScreen[1], '#79d987', 3);
      line(rawScreen[1], rawScreen[2], '#79d987', 3);
      ctx.fillStyle = '#79d987'; ctx.fillText('RAW MAP Z', left + 8, rawScreen[0].y - 8);

      if (corrected) {
        const correctedScreen = corrected.map((point, index) => ({x: sx(index), y: sy(point.z)}));
        line(correctedScreen[0], correctedScreen[1], '#49d6d0', 3);
        line(correctedScreen[1], correctedScreen[2], '#49d6d0', 3);
        ctx.fillStyle = '#49d6d0'; ctx.fillText('CALIBRATED GROUND Z ≈ 0', left + 8, correctedScreen[0].y + 14);
      }

      const phase = animation.progress < .58
        ? animation.progress / .58
        : 1 + (animation.progress - .58) / .42;
      const car = interpolatePath(animation.progress);
      const marker = {x: sx(phase), y: sy(car.z)};
      ctx.beginPath(); ctx.arc(marker.x, marker.y, 5, 0, Math.PI * 2);
      ctx.fillStyle = '#ef705d'; ctx.fill();
      ctx.fillStyle = '#8ea2ad';
      ctx.fillText(`Δ map Z ${(raw[2].z - raw[0].z).toFixed(3)} m`, right - 145, top + 12);
    }

    function resize() {
      const rect = canvas.getBoundingClientRect();
      const ratio = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.floor(rect.width * ratio));
      canvas.height = Math.max(1, Math.floor(rect.height * ratio));
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    }

    function draw(now) {
      if (animation.active) {
        animation.progress = Math.min(1, (now - animation.start) / 2400);
        if (animation.progress < .58) ui.phase.textContent = 'FORWARD RUN / 大车移动';
        else if (animation.progress < 1) ui.phase.textContent = 'LATERAL RUN / 小车移动';
        else { animation.active = false; ui.phase.textContent = 'OBSERVATION READY / 可开始标定'; }
      }
      const width = canvas.clientWidth, height = canvas.clientHeight;
      ctx.clearRect(0, 0, width, height);
      if (width < 720) {
        const panelHeight = (height - 40) / 3;
        drawMapPanel({x: 10, y: 10, w: width - 20, h: panelHeight});
        drawCorrectedPanel({x: 10, y: 20 + panelHeight, w: width - 20, h: panelHeight});
        drawElevationPanel({x: 10, y: 30 + panelHeight * 2, w: width - 20, h: panelHeight});
      } else {
        const planHeight = Math.max(280, height * .64);
        drawMapPanel({x: 10, y: 10, w: (width - 30) / 2, h: planHeight - 15});
        drawCorrectedPanel({x: 20 + (width - 30) / 2, y: 10, w: (width - 30) / 2, h: planHeight - 15});
        drawElevationPanel({x: 10, y: planHeight + 5, w: width - 20, h: height - planHeight - 15});
      }
      requestAnimationFrame(draw);
    }

    document.getElementById('simulateBtn').addEventListener('click', simulate);
    document.getElementById('calibrateBtn').addEventListener('click', calibrate);
    document.getElementById('resetBtn').addEventListener('click', () => { measured = null; calibration = null; animation = {active:false,start:0,progress:1}; clearResult(); ui.phase.textContent = 'IDLE / 等待仿真'; });
    document.getElementById('copyBtn').addEventListener('click', async () => {
      if (!calibration) return;
      try { await navigator.clipboard.writeText(calibration.cliArgs); ui.copy.textContent = 'COPIED ✓'; }
      catch (_) { ui.copy.textContent = 'SELECT CLI MANUALLY'; }
      setTimeout(() => { ui.copy.textContent = 'COPY CLI'; }, 1600);
    });
    window.addEventListener('resize', resize);
    resize(); simulate(); requestAnimationFrame(draw);
  </script>
</body>
</html>"""


def _parse_control_target(
    body: str,
    config: CraneConfig,
    coordinate_transform: CoordinateTransform2D | None = None,
    *,
    z_is_hoist_height: bool = False,
) -> tuple[float, float, float]:
    """Parse a map-frame web target and return validated crane coordinates.

    z_is_hoist_height: 当前 Z 反馈是否来自 PLC 抓钩实测高度 (物理量, 与
    SLAM 地图旋转无关)。为 True 时 target_z 被当作直接输入的抓钩高度,
    不经过 map↔crane 的 3D 旋转/平移——否则一旦标定了 origin_map_z 或
    roll/pitch, 目标 Z 与反馈 Z 就会出现参考系不一致的常数级偏差, 导致
    PD 收敛到一个远离真实目标的高度就提前结束 (v=0)。
    """
    try:
        payload = json.loads(body)
    except json.JSONDecodeError as exc:
        raise ValueError(f'invalid JSON: {exc.msg}') from exc
    if not isinstance(payload, dict):
        raise ValueError('target payload must be a JSON object')
    try:
        map_target_x = float(payload['target_x'])
        map_target_y = float(payload['target_y'])
        map_target_z = float(payload['target_z'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f'target_x, target_y, and target_z are required numbers: {exc}') from exc
    transform = coordinate_transform or CoordinateTransform2D.identity()
    crane_target = transform.map_to_crane_target(
        map_target_x, map_target_y, map_target_z,
        z_is_hoist_height=z_is_hoist_height,
    )
    return config.validate_target(crane_target)


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

    def __init__(
        self,
        control_state: ControlState | None = None,
        coordinate_transform: CoordinateTransform2D | None = None,
        *,
        z_is_hoist_height: bool = False,
    ):
        self._queue: queue.Queue = queue.Queue()
        self._event = threading.Event()    # 信号: 队列有新数据, 唤醒 SSE 消费者
        self._stop_flag = threading.Event()
        self._control_state = control_state  # 轮询 API 用的共享状态
        self._coordinate_transform = (
            coordinate_transform or CoordinateTransform2D.identity()
        )
        # Z 反馈来自抓钩实测高度时, 展示给前端的 Z 也不应被地图旋转/平移
        # 改写——否则实时轮询显示的 Z 会和 Apply Target 输入框里的抓钩高度
        # 不是同一个参考系, 让人误以为"Z 没有用抓钩高度"。
        self._z_is_hoist_height = z_is_hoist_height

    def on_step(self, step_data: dict) -> None:
        """每步: 写 ControlState (供轮询) + 入队 (供 SSE)。"""
        display_step = self._coordinate_transform.control_step_to_map(
            step_data, z_is_hoist_height=self._z_is_hoist_height,
        )
        # 写入轮询状态 (主要数据通道)
        if self._control_state is not None:
            self._control_state.set_step(display_step)
        # 入队供 SSE (诊断通道)
        try:
            self._queue.put_nowait({'type': 'step', 'data': display_step})
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
    .top-actions { display: flex; align-items: center; gap: 8px; pointer-events: auto; }
    .calibration-link {
      padding: 6px 9px;
      border: 1px solid #4b716f;
      border-radius: 4px;
      background: rgba(23, 33, 43, 0.92);
      color: var(--green);
      font-size: 12px;
      text-decoration: none;
      white-space: nowrap;
    }
    .calibration-link:hover { border-color: var(--green); color: var(--text); }
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
        <div class="top-actions">
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
    // PD 结束后冻结轨迹: 保留起点→目标点的完整轨迹在画面上, 阻止实时定位
    // 直播用其 30 帧滑动窗口覆盖/裁掉这条轨迹, 方便展示成果。
    let _trajectoryFrozen = false;

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
      const xyLimit = payload && payload.velocityLimits ? payload.velocityLimits.xy : 0.2;
      const speedMark = Math.min(1, Math.abs(current.vx) / xyLimit);
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
              // 新一轮控制: 解冻并清空上一次的轨迹, 重新开始记录。
              _trajectoryFrozen = false;
              _lastTrailTime = -1;
              if (payload && payload.frames) payload.frames.length = 0;
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
        velocityLimits: {xy: 0.2, z: 0.2},
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

      // PLC mode — use localization data to drive Canvas when no control is active.
      // 冻结时 (PD 刚结束) 不再往 payload.frames 里推流并裁剪, 以保留完整轨迹。
      if (PLC_MODE && !_controlActive && !_trajectoryFrozen) {
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
            velocityLimits: {xy: 0.2, z: 0.2},
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
          if (!s.running && !s.done && !s.error && !s.stopped) return; // control not started yet
          if (s.error) {
            _controlActive = false;
            els.ctrlMsg.textContent = 'Error: ' + s.error;
            els.ctrlMsg.style.color = '#e05a47';
            els.applyTarget.disabled = false;
            els.applyTarget.textContent = 'Apply Target';
            return;
          }
          if (s.stopped) {
            if (_controlActive || !_trajectoryFrozen) {
              // 冻结并重绘: 保留已走过的部分轨迹, 便于查看停止位置。
              _trajectoryFrozen = true;
              frame = Math.max(0, payload.frames.length - 1);
              draw();
            }
            _controlActive = false;
            els.phase.textContent = 'Control Stopped';
            els.ctrlMsg.textContent = s.stop_reason || 'Stopped by operator';
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
              // 冻结并重绘: 把起点→目标点的完整轨迹定格在画面上。
              _trajectoryFrozen = true;
              frame = Math.max(0, payload.frames.length - 1);
              draw();
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
          const xyLimit = (payload && payload.velocityLimits) ? payload.velocityLimits.xy : 0.2;
          const zLimit = (payload && payload.velocityLimits) ? payload.velocityLimits.z : 0.2;
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
        elif parsed.path == '/api/calibrate':
            self._handle_calibrate()
        else:
            self._write(404, 'text/plain; charset=utf-8', b'not found')

    def do_GET(self):
        parsed = urlparse(self.path)
        plc_mode = self.server.plc_actuator is not None
        if parsed.path in ('/', '/index.html'):
            self._write(200, 'text/html; charset=utf-8', render_live_html(plc_mode).encode('utf-8'))
        elif parsed.path in ('/calibration', '/calibration.html'):
            self._write(
                200,
                'text/html; charset=utf-8',
                render_calibration_html().encode('utf-8'),
            )
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

    def _handle_calibrate(self):
        """POST /api/calibrate — estimate transform from browser observations."""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
        except ValueError:
            content_length = -1
        if content_length <= 0 or content_length > _MAX_CALIBRATION_BODY_BYTES:
            body = json.dumps({
                'ok': False,
                'error': 'Invalid calibration request size',
            }).encode('utf-8')
            self._write(413, 'application/json; charset=utf-8', body)
            return

        try:
            request_body = self.rfile.read(content_length).decode('utf-8')
            result = _calibrate_from_request(request_body)
        except (UnicodeDecodeError, ValueError) as exc:
            body = json.dumps({
                'ok': False,
                'error': str(exc),
            }, ensure_ascii=False).encode('utf-8')
            self._write(400, 'application/json; charset=utf-8', body)
            return

        body = json.dumps(result.as_dict(), ensure_ascii=False).encode('utf-8')
        self._write(200, 'application/json; charset=utf-8', body)

    def _stream_localization(self):
        """SSE endpoint that streams the latest /localization_pose data at ~10 Hz.

        Z 轴一致性: 控制环的 Z 反馈用的是 PLC 抓钩实测高度 (物理 Z), 而非
        SLAM 的 Map Z。若这里仍推送 SLAM Z, 前端监视面板与预填的目标 Z 就会
        落在与控制不同的参考系, 导致 (1) 看起来"Z 没用抓钩高度", (2) 操作员
        据此设定的目标 Z 与实际物理高度差出几十公分。因此 PLC 模式下这里也把
        Z (及 Z 速度) 替换为抓钩高度, 使显示/预填/控制三者同一参考系。
        """
        server = self.server
        plc = getattr(server, 'plc', None)
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        last_hoist_z: float | None = None
        last_hoist_wall: float | None = None
        try:
            while True:
                pose = get_latest_pose()
                if pose is not None:
                    # PLC 模式: 用抓钩实测高度覆盖 Z, 并由高度差分估计 Z 速度,
                    # 与控制环 (RosPositionSource + lift_height_provider) 完全一致。
                    if plc is not None:
                        try:
                            hoist_z = plc.get_lift_height()
                        except Exception:
                            hoist_z = None
                        if hoist_z is not None and math.isfinite(hoist_z):
                            pose = dict(pose)
                            hoist_z = float(hoist_z)
                            now = time.monotonic()
                            if last_hoist_z is not None and last_hoist_wall is not None:
                                dt = now - last_hoist_wall
                                pose['vz'] = (hoist_z - last_hoist_z) / dt if dt > 1e-6 else 0.0
                            else:
                                pose['vz'] = 0.0
                            pose['z'] = hoist_z
                            last_hoist_z = hoist_z
                            last_hoist_wall = now
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
            import traceback
            traceback.print_exc()

    def _handle_start_control(self):
        """POST /api/start-control — parse target and start PLC PD control thread."""
        server = self.server
        if server.plc_actuator is None or server.ros_source is None or server.config is None:
            self._write(400, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'PLC mode not active'}).encode('utf-8'))
            return

        try:
            content_length = int(self.headers.get('Content-Length', 0))
        except ValueError:
            self._write(400, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'Invalid Content-Length'}).encode('utf-8'))
            return
        if content_length <= 0 or content_length > _MAX_CONTROL_BODY_BYTES:
            self._write(413, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'Invalid control request size'}).encode('utf-8'))
            return

        plc = server.plc
        if plc is None or not plc.check_connection():
            self._write(503, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'PLC connection is not available'}).encode('utf-8'))
            return
        if not plc.heartbeat_healthy:
            self._write(503, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'PLC heartbeat is not healthy'}).encode('utf-8'))
            return

        # Z 反馈是否用抓钩实测高度 (物理量, 与 SLAM 地图旋转无关)。必须在解析
        # 目标之前就确定这个标志, 否则 target_z 和当前 z_measured 会落在不同
        # 参考系——一旦标定了 origin_map_z 或 roll/pitch, PD 就会朝着一个和
        # 真实目标差着常数偏移的高度收敛, 表现为"距离目标很远时就提前结束"。
        hoist_z = plc.get_lift_height()
        has_hoist_z = hoist_z is not None and math.isfinite(hoist_z)

        try:
            body = self.rfile.read(content_length).decode('utf-8')
            target_x, target_y, target_z = _parse_control_target(
                body,
                server.config,
                server.coordinate_transform,
                z_is_hoist_height=has_hoist_z,
            )
        except (UnicodeDecodeError, ValueError) as exc:
            self._write(400, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': f'Invalid target: {exc}'}).encode('utf-8'))
            return

        # Get current position from localization
        pose = get_latest_pose()
        if pose is None:
            self._write(503, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'No localization data — cannot start control'}).encode('utf-8'))
            return
        try:
            pose_x, pose_y, pose_z = server.config.validate_position(
                server.coordinate_transform.map_to_crane_position(
                    pose['x'], pose['y'], pose['z']
                )
            )
        except (KeyError, TypeError, ValueError) as exc:
            self._write(503, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': f'Invalid localization data: {exc}'}).encode('utf-8'))
            return
        map_pose = dict(pose)
        crane_pose = {**pose, 'x': pose_x, 'y': pose_y, 'z': pose_z}
        # Z 位置优先取抓钩实测高度 (物理 Z); 复用上面已经读取的 hoist_z,
        # 与 target_z 的解析共用同一次读数、同一参考系。
        if has_hoist_z:
            crane_pose['z'] = float(hoist_z)
            map_pose['z'] = float(hoist_z)
        map_target_x, map_target_y, map_target_z = (
            server.coordinate_transform.crane_to_map_display(
                target_x,
                target_y,
                target_z,
                z_is_hoist_height=has_hoist_z,
            )
        )

        if not server.reserve_control_run():
            self._write(409, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'Control already running'}).encode('utf-8'))
            return

        # Create control state for frontend polling
        cs = ControlState()
        cs.set_start(
            pos={'x': map_pose['x'], 'y': map_pose['y'], 'z': map_pose['z']},
            target={
                'x': map_target_x,
                'y': map_target_y,
                'z': map_target_z,
            },
        )
        server.control_state = cs

        # Create control hooks — writes to ControlState (polling) + queue (SSE diagnostic)
        hooks = LiveControlHooks(
            control_state=cs,
            coordinate_transform=server.coordinate_transform,
            z_is_hoist_height=has_hoist_z,
        )
        server.control_hooks = hooks

        initial_state = CraneState(
            x0=crane_pose['x'],
            y0=crane_pose['y'],
            z0=crane_pose['z'],
        )
        # Update PlcActuator Z height to match current position
        server.plc_actuator.set_z_reference(crane_pose['z'])

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
                print(f'[PLC control] PD complete — {len(history)} steps, arrivals: {[(t, a) for t, a in events]}')
                if hooks.should_stop():
                    cs.set_stopped('Stopped by operator')
                else:
                    hooks.done()
                    cs.set_done()
            except ControlStoppedError as exc:
                msg = str(exc)
                print(f'[PLC control] {msg}')
                cs.set_stopped(msg)
            except PositionFeedbackTimeout as exc:
                msg = f'{exc}. Check ROS /localization_pose.'
                print(f'[PLC control] {msg}')
                hooks.send_error(msg)
                cs.set_error(msg)
            except TimeoutError as exc:
                msg = (f'Timeout after 10 min — axes did not all arrive. '
                       f'Check PLC connection or localization: {exc}')
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
                server.release_control_run()
                print('[PLC control] Thread exiting')

        control_thread = threading.Thread(target=_run, name='plc-control', daemon=True)
        server.set_control_thread(control_thread)
        try:
            control_thread.start()
        except Exception as exc:
            server.release_control_run()
            self._write(500, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': f'Failed to start control: {exc}'}).encode('utf-8'))
            return
        print('[PLC control] Thread started')

        self._write(200, 'application/json; charset=utf-8',
                    json.dumps({'ok': True, 'message': 'Control started'}).encode('utf-8'))

    def _handle_stop(self):
        """STOP ALL: send zero velocity to all three axes (matches demo.cpp pattern)."""
        server = self.server
        # 先置停止标志，再通过执行器锁下发最终 STOP，保证 STOP 后没有运动指令穿插。
        if server.control_hooks is not None:
            server.control_hooks.stop()
        if server.control_state is not None:
            server.control_state.set_stopped('Stopped by operator')
        if server.plc_actuator is not None:
            server.plc_actuator.emergency_stop()
        elif server.plc is not None:
            server.plc.big_car_ctrl(0.0)
            server.plc.small_car_ctrl(0.0)
            server.plc.lift_ctrl(0.0)
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
        self.target_pos: dict | None = None  # {'x','y','z'} target
        self.step_count: int = 0
        self.arrivals: list = []            # [{'axis': 'x', 't': 1.23}, ...]
        self.done: bool = False
        self.error: str | None = None
        self.stopped: bool = False
        self.stop_reason: str | None = None

    def set_start(self, pos: dict, target: dict):
        with self.lock:
            self.start_pos = dict(pos)
            self.target_pos = dict(target)
            self.running = True
            self.done = False
            self.error = None
            self.stopped = False
            self.stop_reason = None
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
            self.stopped = False
            self.stop_reason = None

    def set_error(self, msg: str):
        with self.lock:
            self.error = msg
            self.running = False
            self.done = False
            self.stopped = False
            self.stop_reason = None

    def set_stopped(self, reason: str):
        with self.lock:
            self.running = False
            self.done = False
            self.error = None
            self.stopped = True
            self.stop_reason = reason

    def snapshot(self) -> dict:
        with self.lock:
            return {
                'running': self.running,
                'done': self.done,
                'error': self.error,
                'stopped': self.stopped,
                'stop_reason': self.stop_reason,
                'step_count': self.step_count,
                'start_pos': self.start_pos,
                'target_pos': self.target_pos,
                'arrivals': list(self.arrivals),
                'latest': dict(self.latest) if self.latest else None,
            }


class CraneLiveServer(ThreadingHTTPServer):
    def __init__(self, server_address, payload, payload_factory=None, plc=None,
                 ros_source=None, plc_actuator=None, config=None,
                 initial_pos=None, update_hz=10.0, speed=1.0,
                 coordinate_transform=None):
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
        self.coordinate_transform: CoordinateTransform2D = (
            coordinate_transform or CoordinateTransform2D.identity()
        )
        self.control_hooks: LiveControlHooks | None = None
        self.control_thread: threading.Thread | None = None
        self.control_state: ControlState | None = None
        self._control_run_lock = threading.Lock()
        self._control_run_reserved = False

    def build_payload(self, query: dict[str, list[str]]) -> dict[str, Any]:
        if self.payload_factory is None:
            return self.payload
        return self.payload_factory(query)

    def reserve_control_run(self) -> bool:
        """Atomically reserve the single PLC control slot."""
        with self._control_run_lock:
            if self._control_run_reserved:
                return False
            self._control_run_reserved = True
            return True

    def set_control_thread(self, thread: threading.Thread) -> None:
        with self._control_run_lock:
            if not self._control_run_reserved:
                raise RuntimeError('control run has not been reserved')
            self.control_thread = thread

    def release_control_run(self) -> None:
        with self._control_run_lock:
            self.control_thread = None
            self._control_run_reserved = False


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
    coordinate_transform: CoordinateTransform2D | None = None,
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
        coordinate_transform=coordinate_transform,
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
        if server.control_hooks is not None:
            server.control_hooks.stop()
        if server.plc_actuator is not None:
            try:
                server.plc_actuator.emergency_stop()
            except Exception as exc:
                print(f'[PLC] Failed to send shutdown stop command: {exc}')
        control_thread = server.control_thread
        if control_thread is not None and control_thread.is_alive():
            control_thread.join(timeout=3.0)
        server.server_close()
