"""Shared run-log save button (JS + CSS) for PTP / settle pages."""

RUN_LOG_EXTRA_CSS = """
    .btn-save-log {
      width: 100%;
      margin-top: 8px;
      border: 1px solid #4b716f;
      border-radius: 4px;
      background: #1a2a32;
      color: var(--green);
      padding: 9px 10px;
      font: inherit;
      font-size: 13px;
      font-weight: 600;
      cursor: pointer;
    }
    .btn-save-log:hover:not(:disabled) { background: #223840; }
    .btn-save-log:disabled {
      opacity: 0.45;
      cursor: not-allowed;
      color: var(--muted);
      border-color: var(--line);
    }
"""

RUN_LOG_SAVE_JS = """
    function initRunLogSave(opts) {
      opts = opts || {};
      var btn = document.getElementById('btnSaveRunLog');
      if (!btn) return { setReady: function() {}, reset: function() {} };
      var ready = false;
      function setReady(on) {
        ready = !!on;
        btn.disabled = !ready;
      }
      function reset() { setReady(false); }
      btn.addEventListener('click', function() {
        btn.disabled = true;
        var client = { page: opts.page || 'unknown', plc_mode: !!opts.plcMode };
        if (typeof opts.buildClientSupplement === 'function') {
          try {
            Object.assign(client, opts.buildClientSupplement());
          } catch (e) { client.client_error = String(e); }
        }
        fetch('/api/run-log/save', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({ client: client }),
        })
          .then(function(r) {
            if (!r.ok) {
              return r.json().catch(function() { return {}; }).then(function(j) {
                throw new Error(j.error || ('HTTP ' + r.status));
              });
            }
            var disp = r.headers.get('Content-Disposition') || '';
            var m = /filename=\"([^\"]+)\"/.exec(disp);
            var name = m ? m[1] : 'run_log.json';
            return r.blob().then(function(blob) { return { blob: blob, name: name }; });
          })
          .then(function(o) {
            var a = document.createElement('a');
            a.href = URL.createObjectURL(o.blob);
            a.download = o.name;
            document.body.appendChild(a);
            a.click();
            a.remove();
            URL.revokeObjectURL(a.href);
            setReady(true);
            if (opts.onSaved) opts.onSaved(o.name);
          })
          .catch(function(err) {
            setReady(ready);
            alert('保存运行日志失败: ' + (err.message || err));
          });
      });
      return { setReady: setReady, reset: reset };
    }
"""

PTP_RUN_LOG_INIT = """
    var _runLogUi = initRunLogSave({
      page: 'ptp',
      plcMode: PLC_MODE,
      buildClientSupplement: function() {
        var mode = typeof getAntiSwayMode === 'function' ? getAntiSwayMode() : 'off';
        var sup = { anti_sway_mode: mode, url: window.location.href };
        if (!payload || !payload.frames) return sup;
        if (PLC_MODE) {
          sup.ui_trajectory_frames = payload.frames.slice(-800);
          return sup;
        }
        sup.sim_replay = true;
        sup.frames = payload.frames;
        sup.initial = payload.initial;
        sup.target = payload.target;
        sup.phaseBoundaries = payload.phaseBoundaries;
        sup.plannedRoute = payload.plannedRoute || null;
        return sup;
      }
    });
    function markRunLogReady() { _runLogUi.setReady(true); }
    function resetRunLogReady() { _runLogUi.reset(); }
"""

SETTLE_RUN_LOG_INIT = """
    var _runLogUi = initRunLogSave({
      page: 'settle',
      plcMode: PLC_MODE,
      buildClientSupplement: function() {
        var mode = typeof getAntiSwayMode === 'function' ? getAntiSwayMode() : 'baseline';
        return { anti_sway_mode: mode, url: window.location.href };
      }
    });
    function markRunLogReady() { _runLogUi.setReady(true); }
    function resetRunLogReady() { _runLogUi.reset(); }
"""
