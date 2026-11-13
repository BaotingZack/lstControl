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
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
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
from operation_scheduler import OperationScheduler, OperationPhase
from anti_sway.ptp_config import ANTI_SWAY_MODES, apply_ptp_anti_sway_mode
from anti_sway.registry import SETTLE_ALGORITHM_IDS
from site_config import load_site_config
from web.run_log import (
    MAX_RECORDED_STEPS,
    auto_save_run_log,
    build_run_log_document,
    default_log_dir,
    save_run_log_file,
    suggested_download_name,
)
from ros_bridge import get_latest_pose, get_sway_snapshot, RosPositionSource
from sim_inclinometer import (
    build_sim_inclinometer_estimator,
    default_sway_tol_deg,
    inclinometer_fields_for_sway,
    simulate_sway_history,
)
from plc_interface import PlcActuator
from visualizer import CraneVisualizer


_MAX_CONTROL_BODY_BYTES = 4096
_MAX_CALIBRATION_BODY_BYTES = 8192
# Sim replay may attach full `client.frames` (much larger than control POST bodies).
_MAX_RUN_LOG_BODY_BYTES = 32 * 1024 * 1024


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
    crane_target = transform.ui_bridge_target_to_crane(
        map_target_x, map_target_y, map_target_z,
        z_is_hoist_height=z_is_hoist_height,
    )
    return config.validate_target(crane_target)


def _parse_operation_positions(
    body: str,
    config: CraneConfig,
    coordinate_transform: CoordinateTransform2D | None = None,
    *,
    z_is_hoist_height: bool = False,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Parse map-frame start and target positions from a browser request.

    Returns ((start_x, start_y, start_z), (target_x, target_y, target_z))
    in validated crane coordinates.
    """
    try:
        payload = json.loads(body)
    except json.JSONDecodeError as exc:
        raise ValueError(f'invalid JSON: {exc.msg}') from exc
    if not isinstance(payload, dict):
        raise ValueError('payload must be a JSON object')

    transform = coordinate_transform or CoordinateTransform2D.identity()

    def _parse_pos(key_prefix: str) -> tuple[float, float, float]:
        try:
            x = float(payload[f'{key_prefix}_x'])
            y = float(payload[f'{key_prefix}_y'])
            z = float(payload[f'{key_prefix}_z'])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f'{key_prefix}_x, {key_prefix}_y, and {key_prefix}_z '
                f'are required numbers: {exc}'
            ) from exc
        crane_pos = transform.ui_bridge_target_to_crane(
            x, y, z, z_is_hoist_height=z_is_hoist_height,
        )
        return config.validate_target(crane_pos)

    start_pos = _parse_pos('start')
    target_pos = _parse_pos('target')
    return start_pos, target_pos


def _point_tuple(point: tuple[float, float, float]) -> dict[str, float]:
    return {'x': point[0], 'y': point[1], 'z': point[2]}


def _build_ptp_planned_route(
    initial: tuple[float, float, float],
    target: tuple[float, float, float],
) -> dict[str, list[dict[str, float]]]:
    """Straight-line planned overlay for point-to-point moves."""
    return {
        'seg1': [_point_tuple(initial), _point_tuple(target)],
        'seg2': [],
    }


def _build_planned_route(
    initial: tuple[float, float, float],
    pick: tuple[float, float, float],
    place: tuple[float, float, float],
    config,
) -> dict[str, list[dict[str, float]]]:
    """Ideal scheduler waypoints for UI overlay (Start→Pick, Pick→Place)."""
    return {
        'seg1': [
            _point_tuple(initial),
            {'x': pick[0], 'y': pick[1], 'z': config.approach_safe_z},
            _point_tuple(pick),
        ],
        'seg2': [
            {'x': pick[0], 'y': pick[1], 'z': config.lift_cargo_safe_z},
            {'x': place[0], 'y': place[1], 'z': config.transport_safe_z},
            _point_tuple(place),
        ],
    }


def build_live_payload(
    history: list[dict],
    phase_boundaries: list[tuple[float, str]],
    target_pos: tuple[float, float, float],
    initial_pos: tuple[float, float, float],
    config,
    update_hz: float = 10.0,
    speed: float = 1.0,
    pick_pos: tuple[float, float, float] | None = None,
    segment_indices: list[int] | None = None,
    simulate_inclinometer: bool = True,
) -> dict[str, Any]:
    """Build JSON-serializable data consumed by the browser live view.

    Args:
        pick_pos: Optional pick/loading position for two-segment display.
        segment_indices: Optional [seg1_end, seg2_end] indices into history
                         marking segment boundaries for color-coded rendering.
    """
    if update_hz <= 0:
        raise ValueError('update_hz must be positive')
    if speed <= 0:
        raise ValueError('speed must be positive')
    if not history:
        raise ValueError('history must not be empty')

    frame_indices = CraneVisualizer(config).live_frame_indices(history, update_hz)

    # Map segment boundary history indices to frame (display) indices
    display_segment_indices: list[int] | None = None
    if segment_indices and len(segment_indices) >= 2:
        display_segment_indices = []
        for seg_idx in segment_indices[:2]:
            # Find the closest frame_indices entry that covers this history index
            mapped = 0
            for fi_idx, hi in enumerate(frame_indices):
                if hi >= seg_idx:
                    mapped = fi_idx
                    break
            else:
                mapped = len(frame_indices) - 1
            display_segment_indices.append(mapped)

    sway_series: list[dict[str, float]] = []
    incl_estimator = None
    site_cfg: dict = {}
    if simulate_inclinometer:
        site_cfg = load_site_config()
        incl_estimator = build_sim_inclinometer_estimator(site_cfg)
        sheave = float(site_cfg.get('rope_length_sheave_height', 4.0))
        grab_off = float(site_cfg.get('rope_length_grab_offset', 0.0))
        sway_series = simulate_sway_history(
            history,
            sheave_height=sheave,
            grab_offset=grab_off,
        )

    frames = []
    for idx in frame_indices:
        item = history[idx]
        frame_dict = {
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
        }
        if sway_series and incl_estimator is not None:
            s = sway_series[idx]
            frame_dict.update(
                inclinometer_fields_for_sway(
                    s['theta_x'], s['theta_y'], incl_estimator,
                )
            )
        frames.append(frame_dict)

    xs = [item['x'] for item in history] + [target_pos[0], initial_pos[0]]
    ys = [item['y'] for item in history] + [target_pos[1], initial_pos[1]]
    zs = [item['z'] for item in history] + [target_pos[2], initial_pos[2]]
    if pick_pos is not None:
        xs.extend([pick_pos[0]])
        ys.extend([pick_pos[1]])
        zs.extend([pick_pos[2]])

    result = {
        'updateHz': float(update_hz),
        'speed': float(speed),
        'framePeriodMs': 1000.0 / float(update_hz) / float(speed),
        'target': _point_tuple(target_pos),   # deprecated alias for place
        'place': _point_tuple(target_pos),    # place/unload position
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
        'swayTolDeg': default_sway_tol_deg(site_cfg) if simulate_inclinometer else 0.3,
        'inclinometerSimulated': bool(simulate_inclinometer),
    }
    if pick_pos is not None:
        result['pick'] = _point_tuple(pick_pos)
        result['plannedRoute'] = _build_planned_route(
            initial_pos, pick_pos, target_pos, config,
        )
    if display_segment_indices is not None:
        result['segmentIndices'] = display_segment_indices
    return result


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
        # 侧边栏 X Bridge / Y Trolley 与控制环共用行车坐标 (含 localization 取负)
        display_step['crane_x'] = step_data['x']
        display_step['crane_y'] = step_data['y']
        display_step['crane_z'] = step_data['z']
        if step_data.get('vx') is not None:
            display_step['crane_vx'] = step_data['vx']
            display_step['crane_vy'] = step_data['vy']
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



from web.operation_html import render_live_html
from web.ptp_html import render_ptp_html
from web.settle_html import render_settle_html



class _LiveRequestHandler(BaseHTTPRequestHandler):
    def handle(self):
        try:
            super().handle()
        except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
            pass

    def log_message(self, format, *args):
        # Quiet default access log; connection aborts are normal on refresh.
        if args and isinstance(args[-1], str) and 'WinError 10053' in args[-1]:
            return
        super().log_message(format, *args)

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
        elif parsed.path == '/api/start-goto':
            self._handle_start_goto()
        elif parsed.path == '/api/start-settle':
            self._handle_start_settle()
        elif parsed.path == '/api/calibrate':
            self._handle_calibrate()
        elif parsed.path == '/api/run-log/save':
            self._handle_run_log_save()
        else:
            self._write(404, 'text/plain; charset=utf-8', b'not found')

    def do_GET(self):
        parsed = urlparse(self.path)
        plc_connected = self.server.plc_actuator is not None
        if parsed.path in ('/', '/index.html'):
            self.send_response(302)
            self.send_header('Location', '/ptp/sim')
            self.end_headers()
            return
        if parsed.path in ('/goto', '/goto.html', '/ptp', '/ptp.html'):
            self.send_response(302)
            self.send_header('Location', '/ptp/sim')
            self.end_headers()
            return
        if parsed.path in ('/ptp/sim', '/ptp/sim.html'):
            self._write(
                200,
                'text/html; charset=utf-8',
                render_ptp_html(plc_mode=False).encode('utf-8'),
            )
        elif parsed.path in ('/ptp/live', '/ptp/live.html', '/ptp/plc', '/ptp/plc.html'):
            self._write(
                200,
                'text/html; charset=utf-8',
                render_ptp_html(plc_mode=True).encode('utf-8'),
            )
        elif parsed.path in ('/settle/sim', '/settle/sim.html'):
            self._write(
                200,
                'text/html; charset=utf-8',
                render_settle_html(plc_mode=False).encode('utf-8'),
            )
        elif parsed.path in ('/settle/live', '/settle/live.html'):
            self._write(
                200,
                'text/html; charset=utf-8',
                render_settle_html(plc_mode=True).encode('utf-8'),
            )
        elif parsed.path in ('/operation', '/operation.html'):
            self._write(
                200,
                'text/html; charset=utf-8',
                render_live_html(plc_connected).encode('utf-8'),
            )
        elif parsed.path in ('/calibration', '/calibration.html'):
            self._write(
                200,
                'text/html; charset=utf-8',
                render_calibration_html().encode('utf-8'),
            )
        elif parsed.path == '/simulation.json':
            if self.server.payload_factory is None and self.server.payload is None:
                self._write(400, 'application/json; charset=utf-8',
                            json.dumps({'error': 'Simulation replay not configured'}).encode('utf-8'))
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
        elif parsed.path == '/api/sway-snapshot':
            self._handle_sway_snapshot()
        elif parsed.path == '/api/anti-sway-algorithms':
            self._handle_anti_sway_algorithms()
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
                    transform = getattr(
                        server,
                        'coordinate_transform',
                        None,
                    ) or CoordinateTransform2D.identity()
                    pose = transform.localization_pose_dict_for_display(pose)
                    data = json.dumps(pose, ensure_ascii=False)
                    self.wfile.write(f'data: {data}\n\n'.encode('utf-8'))
                    self.wfile.flush()
                else:
                    # Heartbeat comment to keep connection alive
                    self.wfile.write(': waiting for /localization_pose\n\n'.encode('utf-8'))
                    self.wfile.flush()
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
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
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            print('[control SSE] Client disconnected')
        except Exception as exc:
            print(f'[control SSE] Unexpected error: {exc}')
            import traceback
            traceback.print_exc()

    def _handle_start_control(self):
        """POST /api/start-control — parse start+target positions and run the
        full multi-phase operation scheduler."""
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

        # Z 反馈是否用抓钩实测高度 (物理量, 与 SLAM 地图旋转无关)。
        hoist_z = plc.get_lift_height()
        has_hoist_z = hoist_z is not None and math.isfinite(hoist_z)

        try:
            body = self.rfile.read(content_length).decode('utf-8')
            (start_x, start_y, start_z), (target_x, target_y, target_z) = \
                _parse_operation_positions(
                    body,
                    server.config,
                    server.coordinate_transform,
                    z_is_hoist_height=has_hoist_z,
                )
        except (UnicodeDecodeError, ValueError) as exc:
            self._write(400, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': f'Invalid positions: {exc}'}).encode('utf-8'))
            return

        # Get current position from localization
        pose = get_latest_pose()
        if pose is None:
            self._write(503, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'No localization data — cannot start control'}).encode('utf-8'))
            return
        try:
            pose_x, pose_y, pose_z = server.config.validate_position(
                server.coordinate_transform.localization_map_to_crane_position(
                    pose['x'], pose['y'], pose['z']
                )
            )
        except (KeyError, TypeError, ValueError) as exc:
            self._write(503, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': f'Invalid localization data: {exc}'}).encode('utf-8'))
            return
        map_pose = dict(pose)
        crane_pose = {**pose, 'x': pose_x, 'y': pose_y, 'z': pose_z}
        if has_hoist_z:
            crane_pose['z'] = float(hoist_z)
            map_pose['z'] = float(hoist_z)

        # Map target back to map frame for display
        map_start_x, map_start_y, map_start_z = (
            server.coordinate_transform.crane_to_map_display(
                start_x, start_y, start_z,
                z_is_hoist_height=has_hoist_z,
            )
        )
        map_target_x, map_target_y, map_target_z = (
            server.coordinate_transform.crane_to_map_display(
                target_x, target_y, target_z,
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
        # Store the start position as well for display
        cs.start_pos = {
            'x': map_pose['x'], 'y': map_pose['y'], 'z': map_pose['z'],
        }
        # Also store pick position
        cs.pick_pos = {
            'x': map_start_x, 'y': map_start_y, 'z': map_start_z,
        }
        cs.planned_route = _build_planned_route(
            (map_pose['x'], map_pose['y'], map_pose['z']),
            (map_start_x, map_start_y, map_start_z),
            (map_target_x, map_target_y, map_target_z),
            server.config,
        )
        cs.scheduler_phase = None
        cs.scheduler_phase_label = OperationPhase.IDLE.label
        server.control_state = cs

        # Create control hooks — for SSE diagnostic channel
        hooks = LiveControlHooks(
            control_state=cs,
            coordinate_transform=server.coordinate_transform,
            z_is_hoist_height=has_hoist_z,
        )
        server.control_hooks = hooks

        # Sync PlcActuator Z with current position
        server.plc_actuator.set_z_reference(crane_pose['z'])

        start_pos = (start_x, start_y, start_z)
        target_pos = (target_x, target_y, target_z)

        # Create scheduler with coordinate transform for proper frontend display
        scheduler = OperationScheduler(
            plc=plc,
            source=server.ros_source,
            actuator=server.plc_actuator,
            config=server.config,
            is_simulation=False,
            coordinate_transform=server.coordinate_transform,
            z_is_hoist_height=has_hoist_z,
        )

        # Store scheduler reference for stop/reset
        server._scheduler = scheduler

        # Start operation in background thread
        def _run():
            print(f'[Scheduler] Starting operation: '
                  f'pick=({start_x:.2f}, {start_y:.2f}, {start_z:.2f}), '
                  f'place=({target_x:.2f}, {target_y:.2f}, {target_z:.2f}), '
                  f'from=({crane_pose["x"]:.2f}, {crane_pose["y"]:.2f}, {crane_pose["z"]:.2f})')
            try:
                result = scheduler.execute(
                    start_pos=start_pos,
                    target_pos=target_pos,
                    control_state=cs,
                )
                print(f'[Scheduler] Operation result: {result.phase.name} — {result.message}')
                # Terminal state was already set by scheduler.execute() internally.
                # Only send the SSE completion event — on success only.
                if result.success:
                    hooks.done()
                elif result.phase == OperationPhase.STOPPED:
                    pass  # scheduler already set cs.set_stopped + emergency_stop
                else:
                    hooks.send_error(result.message)
            except Exception as exc:
                import traceback
                print(f'[Scheduler] Fatal error: {exc}')
                traceback.print_exc()
                cs.set_error(str(exc))
                hooks.send_error(str(exc))
            finally:
                server.persist_run_log(cs)
                server.release_control_run()
                print('[Scheduler] Thread exiting')

        control_thread = threading.Thread(target=_run, name='scheduler-control', daemon=True)
        server.set_control_thread(control_thread)
        try:
            control_thread.start()
        except Exception as exc:
            server.release_control_run()
            self._write(500, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': f'Failed to start control: {exc}'}).encode('utf-8'))
            return
        print('[Scheduler] Thread started')

        self._write(200, 'application/json; charset=utf-8',
                    json.dumps({
                        'ok': True,
                        'message': 'Operation started',
                        'phase': OperationPhase.APPROACH_XY.name,
                        'phase_label': OperationPhase.APPROACH_XY.label,
                        'start_pos': cs.start_pos,
                        'pick_pos': cs.pick_pos,
                        'place_pos': cs.place_pos,
                        'planned_route': cs.planned_route,
                    }).encode('utf-8'))

    def _handle_start_goto(self):
        """POST /api/start-goto — move from current localization to a single target via PD."""
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

        hoist_z = plc.get_lift_height()
        has_hoist_z = hoist_z is not None and math.isfinite(hoist_z)

        try:
            body = self.rfile.read(content_length).decode('utf-8')
            anti_sway = False
            anti_sway_mode = 'off'
            try:
                req_payload = json.loads(body)
                if isinstance(req_payload, dict):
                    raw_mode = req_payload.get('anti_sway_mode')
                    if isinstance(raw_mode, str) and raw_mode.strip().lower() in ANTI_SWAY_MODES:
                        anti_sway_mode = raw_mode.strip().lower()
                    elif bool(req_payload.get('anti_sway', False)):
                        anti_sway_mode = 'incl'
                    anti_sway = anti_sway_mode != 'off'
            except json.JSONDecodeError:
                pass
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

        pose = get_latest_pose()
        if pose is None:
            self._write(503, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'No localization data — cannot start control'}).encode('utf-8'))
            return
        try:
            pose_x, pose_y, pose_z = server.config.validate_position(
                server.coordinate_transform.localization_map_to_crane_position(
                    pose['x'], pose['y'], pose['z']
                )
            )
        except (KeyError, TypeError, ValueError) as exc:
            self._write(503, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': f'Invalid localization data: {exc}'}).encode('utf-8'))
            return
        map_pose = dict(pose)
        if has_hoist_z:
            map_pose['z'] = float(hoist_z)

        map_target_x, map_target_y, map_target_z = (
            server.coordinate_transform.crane_to_map_display(
                target_x, target_y, target_z,
                z_is_hoist_height=has_hoist_z,
            )
        )

        if not server.reserve_control_run():
            self._write(409, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'Control already running'}).encode('utf-8'))
            return

        cs = ControlState()
        cs.set_start(
            pos={'x': map_pose['x'], 'y': map_pose['y'], 'z': map_pose['z']},
            target={
                'x': map_target_x,
                'y': map_target_y,
                'z': map_target_z,
            },
        )
        cs.begin_run(kind='ptp_goto', anti_sway_mode=anti_sway_mode, sim=False)
        with cs.lock:
            cs.scheduler_phase_label = 'Move to target / 移动到目标'
        cs.planned_route = _build_ptp_planned_route(
            (map_pose['x'], map_pose['y'], map_pose['z']),
            (map_target_x, map_target_y, map_target_z),
        )
        cs.pick_pos = None
        server.control_state = cs

        hooks = LiveControlHooks(
            control_state=cs,
            coordinate_transform=server.coordinate_transform,
            z_is_hoist_height=has_hoist_z,
        )
        server.control_hooks = hooks
        server._scheduler = None

        target_pos = (target_x, target_y, target_z)
        ros_source = server.ros_source
        plc_actuator = server.plc_actuator
        config = server.config

        def _run():
            print(
                f'[PD goto] Target=({target_x:.2f}, {target_y:.2f}, {target_z:.2f}), '
                f'from map=({map_pose["x"]:.2f}, {map_pose["y"]:.2f}, {map_pose["z"]:.2f})'
            )
            try:
                initial_state: CraneState | None = None
                for attempt in range(3):
                    if hooks.should_stop():
                        raise ControlStoppedError('Stopped by operator before start')
                    pose_fb = ros_source.get_position()
                    if pose_fb is not None:
                        initial_state = CraneState(
                            x0=pose_fb['x'],
                            y0=pose_fb['y'],
                            z0=pose_fb['z'],
                        )
                        break
                    if attempt < 2:
                        time.sleep(0.5)
                if initial_state is None:
                    raise RuntimeError('无法获取当前位置反馈 (定位断流)')

                plc_actuator.set_z_reference(initial_state.z.position)
                from ros_bridge import SwaySensorSource

                run_config = apply_ptp_anti_sway_mode(
                    config, anti_sway_mode, load_site_config(),
                    legacy_enabled=anti_sway,
                )
                sway_source = None
                if anti_sway_mode != 'off' and anti_sway_mode != 'input_shape':
                    sway_source = SwaySensorSource()
                run_pd_control(
                    source=ros_source,
                    actuator=plc_actuator,
                    config=run_config,
                    target_pos=target_pos,
                    initial_state=initial_state,
                    hooks=hooks,
                    verbose=True,
                    is_simulation=False,
                    sway_source=sway_source,
                )
                cs.set_done()
                hooks.done()
                print('[PD goto] Target reached')
            except ControlStoppedError as exc:
                cs.set_stopped(str(exc))
                print(f'[PD goto] Stopped: {exc}')
            except Exception as exc:
                import traceback
                print(f'[PD goto] Fatal error: {exc}')
                traceback.print_exc()
                cs.set_error(str(exc))
                hooks.send_error(str(exc))
            finally:
                server.persist_run_log(cs)
                server.release_control_run()
                print('[PD goto] Thread exiting')

        control_thread = threading.Thread(target=_run, name='pd-goto-control', daemon=True)
        server.set_control_thread(control_thread)
        try:
            control_thread.start()
        except Exception as exc:
            server.release_control_run()
            self._write(500, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': f'Failed to start control: {exc}'}).encode('utf-8'))
            return
        print('[PD goto] Thread started')

        self._write(200, 'application/json; charset=utf-8',
                    json.dumps({
                        'ok': True,
                        'message': 'Point-to-point move started',
                        'phase_label': 'Move to target / 移动到目标',
                        'start_pos': cs.start_pos,
                        'target_pos': cs.target_pos,
                        'planned_route': cs.planned_route,
                    }).encode('utf-8'))

    def _handle_start_settle(self):
        """POST /api/start-settle — hold position and damp hook sway (消摆测试)."""
        server = self.server
        try:
            content_length = int(self.headers.get('Content-Length', 0))
        except ValueError:
            self._write(400, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'Invalid Content-Length'}).encode('utf-8'))
            return
        if content_length <= 0 or content_length > _MAX_CONTROL_BODY_BYTES:
            self._write(413, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'Invalid request size'}).encode('utf-8'))
            return

        try:
            body = self.rfile.read(content_length).decode('utf-8')
            req = json.loads(body) if body.strip() else {}
        except (UnicodeDecodeError, json.JSONDecodeError):
            req = {}

        anti_sway_mode = 'baseline'
        if isinstance(req, dict):
            raw = req.get('anti_sway_mode')
            if isinstance(raw, str) and raw.strip().lower() in SETTLE_ALGORITHM_IDS:
                anti_sway_mode = raw.strip().lower()
        use_sim = bool(req.get('sim', False)) if isinstance(req, dict) else False

        if anti_sway_mode not in SETTLE_ALGORITHM_IDS:
            self._write(400, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'Invalid settle algorithm'}).encode('utf-8'))
            return

        base_config = server.config or CraneConfig()
        site = load_site_config()
        from settle_control import apply_settle_anti_sway_mode, run_settle_control

        run_config = apply_settle_anti_sway_mode(base_config, anti_sway_mode, site)
        tol_rad = math.radians(getattr(server, 'hook_sway_angle_tol_deg', 0.3))

        if use_sim or (server.plc_actuator is None and not server.ros_source):
            use_sim = True
        else:
            plc = server.plc
            if plc is None or not plc.check_connection() or not plc.heartbeat_healthy:
                self._write(503, 'application/json; charset=utf-8',
                            json.dumps({'ok': False, 'error': 'PLC not ready for live settle'}).encode('utf-8'))
                return

        if not server.reserve_control_run():
            self._write(409, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'Control already running'}).encode('utf-8'))
            return

        cs = ControlState()
        cs.begin_run(kind='settle', anti_sway_mode=anti_sway_mode, sim=use_sim)
        hooks = LiveControlHooks(
            control_state=cs,
            coordinate_transform=server.coordinate_transform,
            z_is_hoist_height=False,
        )
        server.control_state = cs
        server.control_hooks = hooks
        server._scheduler = None

        def _run_sim():
            from anti_sway.sim_feedback import SimSwayFeedback
            from crane_model import CranePlant, PlantActuator, SimPositionSource

            anchor = server.initial_pos or (2.0, 2.0, 3.0)
            initial = CraneState(x0=anchor[0], y0=anchor[1], z0=anchor[2])
            plant = CranePlant(run_config)
            source = SimPositionSource(plant, initial, run_config)
            actuator = PlantActuator(plant, initial, run_config)
            sway = SimSwayFeedback(
                sheave_height=run_config.rope_length_sheave_height,
                grab_offset=run_config.rope_length_grab_offset,
            )
            sway.reset()
            cs.set_start(
                pos={'x': anchor[0], 'y': anchor[1], 'z': anchor[2]},
                target={'x': anchor[0], 'y': anchor[1], 'z': anchor[2]},
            )
            with cs.lock:
                cs.scheduler_phase_label = '消摆仿真 / Sim settle'
            try:
                run_settle_control(
                    source=source,
                    actuator=actuator,
                    config=run_config,
                    anchor_pos=anchor,
                    initial_state=initial,
                    hooks=hooks,
                    sway_source=sway,
                    is_simulation=True,
                    verbose=True,
                    initial_theta=(0.055, 0.035),
                    settle_theta_tol_rad=tol_rad,
                )
                cs.set_done()
                hooks.done()
            except ControlStoppedError as exc:
                cs.set_stopped(str(exc))
            except Exception as exc:
                cs.set_error(str(exc))
                hooks.send_error(str(exc))
            finally:
                server.persist_run_log(cs)
                server.release_control_run()

        def _run_live():
            from ros_bridge import SwaySensorSource

            ros_source = server.ros_source
            plc_actuator = server.plc_actuator
            pose = get_latest_pose()
            if pose is None or ros_source is None or plc_actuator is None:
                cs.set_error('No localization / PLC for live settle')
                server.persist_run_log(cs)
                server.release_control_run()
                return
            try:
                px, py, pz = base_config.validate_position(
                    server.coordinate_transform.localization_map_to_crane_position(
                        pose['x'], pose['y'], pose['z']
                    )
                )
            except (KeyError, TypeError, ValueError) as exc:
                cs.set_error(f'Invalid localization: {exc}')
                server.persist_run_log(cs)
                server.release_control_run()
                return
            map_pose = dict(pose)
            cs.set_start(
                pos={'x': map_pose['x'], 'y': map_pose['y'], 'z': map_pose['z']},
                target={'x': map_pose['x'], 'y': map_pose['y'], 'z': map_pose['z']},
            )
            with cs.lock:
                cs.scheduler_phase_label = '消摆真机 / Live settle — 请手动晃钩'
            anchor = (px, py, pz)
            initial_state = CraneState(x0=px, y0=py, z0=pz)
            plc_actuator.set_z_reference(pz)
            sway_source = SwaySensorSource()
            try:
                run_settle_control(
                    source=ros_source,
                    actuator=plc_actuator,
                    config=run_config,
                    anchor_pos=anchor,
                    initial_state=initial_state,
                    hooks=hooks,
                    sway_source=sway_source,
                    is_simulation=False,
                    verbose=True,
                    settle_theta_tol_rad=tol_rad,
                )
                cs.set_done()
                hooks.done()
            except ControlStoppedError as exc:
                cs.set_stopped(str(exc))
            except Exception as exc:
                import traceback
                traceback.print_exc()
                cs.set_error(str(exc))
                hooks.send_error(str(exc))
            finally:
                server.persist_run_log(cs)
                server.release_control_run()

        target_fn = _run_sim if use_sim else _run_live
        thread = threading.Thread(target=target_fn, name='settle-control', daemon=True)
        server.set_control_thread(thread)
        try:
            thread.start()
        except Exception as exc:
            server.release_control_run()
            self._write(500, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': str(exc)}).encode('utf-8'))
            return

        self._write(200, 'application/json; charset=utf-8',
                    json.dumps({
                        'ok': True,
                        'sim': use_sim,
                        'anti_sway_mode': anti_sway_mode,
                        'phase_label': '消摆运行中',
                    }).encode('utf-8'))

    def _handle_stop(self):
        """STOP ALL: stop scheduler + send zero velocity to all axes."""
        server = self.server
        # Stop the scheduler first — it will set the final stop_reason on
        # ControlState internally via scheduler.hooks.set_stopped().
        scheduler = getattr(server, '_scheduler', None)
        if scheduler is not None:
            scheduler.hooks.stop()
        if server.control_hooks is not None:
            server.control_hooks.stop()
        # Only set stopped when there's no scheduler running (idle stop);
        # otherwise the scheduler thread owns the terminal state transition.
        if server.control_state is not None and scheduler is None:
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

    def _handle_anti_sway_algorithms(self):
        """List selectable anti-sway modes for PTP UI (off + registry)."""
        from anti_sway.registry import list_algorithm_options
        from urllib.parse import parse_qs, urlparse

        qs = parse_qs(urlparse(self.path).query)
        scope_raw = (qs.get('scope') or ['ptp'])[0]
        scope = scope_raw.strip().lower() if isinstance(scope_raw, str) else 'ptp'
        if scope not in ('ptp', 'settle', 'all'):
            scope = 'ptp'

        payload = {
            'scope': scope,
            'options': [
                {
                    'id': 'off',
                    'label': '纯 PD / Plain PD',
                    'description': '仅位置 PD，不叠加防摇修正。',
                },
                *list_algorithm_options(scope=scope),
            ],
        }
        if scope == 'settle':
            payload['options'] = list_algorithm_options(scope='settle')
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self._write(200, 'application/json; charset=utf-8', body)

    def _handle_run_log_save(self):
        """POST /api/run-log/save — export run log JSON (download + server logs/ copy)."""
        server = self.server
        client_supplement = None
        if self.command == 'POST':
            try:
                content_length = int(self.headers.get('Content-Length', 0))
            except ValueError:
                content_length = 0
            if content_length <= 0:
                pass
            elif content_length > _MAX_RUN_LOG_BODY_BYTES:
                self._write(413, 'application/json; charset=utf-8',
                            json.dumps({
                                'ok': False,
                                'error': (
                                    f'Run log request too large ({content_length} bytes, '
                                    f'max {_MAX_RUN_LOG_BODY_BYTES})'
                                ),
                            }).encode('utf-8'))
                return
            else:
                try:
                    raw = self.rfile.read(content_length).decode('utf-8')
                    parsed = json.loads(raw) if raw.strip() else {}
                    if isinstance(parsed, dict):
                        client_supplement = parsed.get('client') or parsed
                except (UnicodeDecodeError, json.JSONDecodeError):
                    pass

        cs = server.control_state
        status = 'idle'
        if cs is not None:
            with cs.lock:
                if cs.running:
                    status = 'running'
                elif cs.done:
                    status = 'done'
                elif cs.stopped:
                    status = 'stopped'
                elif cs.error:
                    status = 'error'

        if cs is None and not client_supplement:
            self._write(400, 'application/json; charset=utf-8',
                        json.dumps({
                            'ok': False,
                            'error': (
                                'No run data — complete a move or settle first '
                                '(simulation: wait for replay to finish, then save)'
                            ),
                        }).encode('utf-8'))
            return
        if status == 'running':
            self._write(409, 'application/json; charset=utf-8',
                        json.dumps({'ok': False, 'error': 'Run still in progress'}).encode('utf-8'))
            return

        site = load_site_config()
        document = build_run_log_document(
            cs,
            server=server,
            client_supplement=client_supplement,
            site=site,
        )
        log_dir = default_log_dir()
        try:
            saved_path = save_run_log_file(document, log_dir)
            document['saved_server_path'] = str(saved_path)
        except OSError as exc:
            document['saved_server_path'] = None
            document['save_error'] = str(exc)

        filename = suggested_download_name(document)
        body = json.dumps(document, ensure_ascii=False, indent=2).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_sway_snapshot(self):
        """Return inclinometer / sway angles for PTP live UI."""
        snap = dict(get_sway_snapshot())
        snap['tol_deg'] = getattr(self.server, 'hook_sway_angle_tol_deg', 0.3)
        body = json.dumps(snap, ensure_ascii=False).encode('utf-8')
        self._write(200, 'application/json; charset=utf-8', body)

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
        self.target_pos: dict | None = None  # {'x','y','z'} target (place position)
        self.pick_pos: dict | None = None  # {'x','y','z'} pick position (start of operation)
        self.place_pos: dict | None = None  # {'x','y','z'} place/unload position (alias for target_pos)
        self.planned_route: dict | None = None  # ideal waypoints for UI overlay
        self.segment_boundaries: list = []  # frame indices marking segment transitions
        self.scheduler_phase: str | None = None
        self.scheduler_phase_label: str | None = None
        self.step_count: int = 0
        self.arrivals: list = []            # [{'axis': 'x', 't': 1.23}, ...]
        self.done: bool = False
        self.error: str | None = None
        self.stopped: bool = False
        self.stop_reason: str | None = None
        # Run log (debug export)
        self.run_kind: str | None = None
        self.anti_sway_mode: str | None = None
        self.run_sim: bool = False
        self.started_at_iso: str | None = None
        self.finished_at_iso: str | None = None
        self.steps: list[dict] = []
        self.events: list[dict] = []
        self.steps_truncated: bool = False

    def begin_run(
        self,
        *,
        kind: str,
        anti_sway_mode: str = 'off',
        sim: bool = False,
    ) -> None:
        with self.lock:
            self.run_kind = kind
            self.anti_sway_mode = anti_sway_mode
            self.run_sim = sim
            self.started_at_iso = datetime.now(timezone.utc).replace(
                microsecond=0,
            ).isoformat().replace('+00:00', 'Z')
            self.finished_at_iso = None
            self.steps = []
            self.events = []
            self.steps_truncated = False

    def _finalize_run(self) -> None:
        if self.finished_at_iso is None:
            self.finished_at_iso = datetime.now(timezone.utc).replace(
                microsecond=0,
            ).isoformat().replace('+00:00', 'Z')

    def set_start(self, pos: dict, target: dict):
        with self.lock:
            self.start_pos = dict(pos)
            self.target_pos = dict(target)
            self.place_pos = dict(target)  # semantic alias for place/unload position
            self.running = True
            self.done = False
            self.error = None
            self.stopped = False
            self.stop_reason = None
            self.step_count = 0
            self.arrivals = []
            self.segment_boundaries = []
            self.scheduler_phase = None
            self.scheduler_phase_label = None

    def set_step(self, step_data: dict):
        with self.lock:
            self.latest = dict(step_data)
            self.step_count += 1
            if not self.steps_truncated:
                if len(self.steps) < MAX_RECORDED_STEPS:
                    self.steps.append(dict(step_data))
                else:
                    self.steps_truncated = True

    def set_arrival(self, axis: str, t: float):
        with self.lock:
            self.arrivals.append({'axis': axis, 't': t})
            self.events.append({'type': 'arrival', 'axis': axis, 't': t})

    def set_done(self):
        with self.lock:
            self.done = True
            self.running = False
            self.stopped = False
            self.stop_reason = None
            self._finalize_run()

    def set_error(self, msg: str):
        with self.lock:
            self.error = msg
            self.running = False
            self.done = False
            self.stopped = False
            self.stop_reason = None
            self.events.append({'type': 'error', 'message': msg})
            self._finalize_run()

    def set_stopped(self, reason: str):
        with self.lock:
            self.running = False
            self.done = False
            self.error = None
            self.stopped = True
            self.stop_reason = reason
            self.events.append({'type': 'stopped', 'reason': reason})
            self._finalize_run()

    def snapshot(self) -> dict:
        with self.lock:
            phase = self.scheduler_phase
            phase_label = self.scheduler_phase_label
            pick_pos = self.pick_pos
            # scheduler_phase may be an OperationPhase enum or None
            phase_name = phase.name if hasattr(phase, 'name') else None
            return {
                'running': self.running,
                'done': self.done,
                'error': self.error,
                'stopped': self.stopped,
                'stop_reason': self.stop_reason,
                'step_count': self.step_count,
                'start_pos': self.start_pos,
                'target_pos': self.target_pos,
                'pick_pos': pick_pos,
                'place_pos': self.place_pos,
                'planned_route': self.planned_route,
                'segment_boundaries': list(self.segment_boundaries),
                'scheduler_phase': phase_name,
                'scheduler_phase_label': phase_label,
                'arrivals': list(self.arrivals),
                'latest': dict(self.latest) if self.latest else None,
            }


class CraneLiveServer(ThreadingHTTPServer):
    def handle_error(self, request, client_address):
        exc = sys.exc_info()[1]
        if isinstance(exc, (ConnectionAbortedError, ConnectionResetError, BrokenPipeError)):
            return
        super().handle_error(request, client_address)

    def __init__(self, server_address, payload, payload_factory=None, plc=None,
                 ros_source=None, plc_actuator=None, config=None,
                 initial_pos=None, update_hz=10.0, speed=1.0,
                 coordinate_transform=None, hook_sway_angle_tol_deg=0.3):
        super().__init__(server_address, _LiveRequestHandler)
        self.payload = payload
        self.payload_factory = payload_factory
        self.plc = plc  # PLC instance for interactive control (stop/reset)
        self.hook_sway_angle_tol_deg = hook_sway_angle_tol_deg
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

    def persist_run_log(self, cs: ControlState | None) -> Path | None:
        """Save completed run JSON under project logs/ (done, stopped, or error)."""
        return auto_save_run_log(
            cs,
            server=self,
            site=load_site_config(),
        )


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
    hook_sway_angle_tol_deg: float = 0.3,
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
        hook_sway_angle_tol_deg=hook_sway_angle_tol_deg,
    )
    actual_host, actual_port = server.server_address
    url = f'http://{actual_host}:{actual_port}'
    print(f'防摇测试平台 (点到点·仿真):  {url}/ptp/sim')
    print(f'防摇测试平台 (点到点·真机):  {url}/ptp/live')
    print(f'消摆测试 (仿真):          {url}/settle/sim')
    print(f'消摆测试 (真机):          {url}/settle/live')
    print(f'Full operation:  {url}/operation')
    if plc_actuator is not None:
        print('PLC connected — live PTP and /operation control are available.')
    else:
        print('PLC not connected — use --plc-ip with --live for /ptp/live and /operation.')
    print('Press Ctrl+C to stop.')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nLive view stopped.')
    finally:
        # Stop scheduler first (signals nested PD loops)
        scheduler = getattr(server, '_scheduler', None)
        if scheduler is not None:
            scheduler.hooks.stop()
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
