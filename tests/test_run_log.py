"""Run log export (ControlState + /api/run-log/save payload builder)."""

import json
import threading
from http.client import HTTPConnection
from pathlib import Path

import pytest

from live_server import ControlState, CraneLiveServer
from web.run_log import (
    auto_save_run_log,
    build_run_log_document,
    run_status,
    save_run_log_file,
    suggested_download_name,
)


def test_control_state_begin_run_and_steps():
    cs = ControlState()
    cs.begin_run(kind='ptp_goto', anti_sway_mode='incl', sim=False)
    cs.set_start({'x': 0, 'y': 0, 'z': 5}, {'x': 3, 'y': 2, 'z': 4})
    cs.set_step({'t': 0.1, 'x': 0.1, 'y': 0, 'z': 5, 'vx_cmd': 0.02})
    cs.set_done()

    snap = cs.snapshot()
    assert snap['done'] is True
    assert run_status(cs) == 'done'

    with cs.lock:
        assert cs.run_kind == 'ptp_goto'
        assert cs.anti_sway_mode == 'incl'
        assert len(cs.steps) == 1
        assert cs.started_at_iso
        assert cs.finished_at_iso


def test_build_run_log_document_merges_client():
    cs = ControlState()
    cs.begin_run(kind='settle', anti_sway_mode='crane_remote', sim=True)
    cs.set_step({'t': 0, 'theta_x': 0.01})
    cs.set_done()

    doc = build_run_log_document(
        cs,
        client_supplement={'page': 'settle', 'anti_sway_mode': 'crane_remote'},
        site={'hook_sway_angle_tol_deg': 0.3, 'secret': 'skip'},
    )
    assert doc['format'] == 'lstControl_run_log'
    assert doc['run']['kind'] == 'settle'
    assert doc['run']['steps'][0]['theta_x'] == 0.01
    assert doc['client']['page'] == 'settle'
    assert 'secret' not in doc['site_config']
    assert doc['site_config']['hook_sway_angle_tol_deg'] == 0.3


def test_build_run_log_client_only_when_no_control_state():
    doc = build_run_log_document(
        None,
        client_supplement={'page': 'ptp', 'sim_replay': True, 'frames': []},
    )
    assert doc['status'] == 'idle'
    assert doc['client']['sim_replay'] is True
    assert doc['run']['steps'] == []


def test_auto_save_run_log_after_terminal_state(tmp_path: Path):
    cs = ControlState()
    cs.begin_run(kind='ptp_goto', anti_sway_mode='off', sim=False)
    cs.set_step({'t': 0.1, 'x': 1.0, 'vx_cmd': 0.1})
    assert auto_save_run_log(cs, log_dir=tmp_path) is None
    cs.set_done()
    path = auto_save_run_log(cs, log_dir=tmp_path)
    assert path is not None
    assert path.parent == tmp_path
    assert path.name.startswith('run_log_ptp_goto_')


def test_save_run_log_file_writes_json(tmp_path: Path):
    doc = build_run_log_document(
        None,
        client_supplement={'page': 'ptp'},
    )
    path = save_run_log_file(doc, tmp_path)
    assert path.exists()
    loaded = json.loads(path.read_text(encoding='utf-8'))
    assert loaded['format'] == 'lstControl_run_log'
    assert suggested_download_name(doc).startswith('run_log_')


def test_run_log_save_post_accepts_client_only_body_over_4k():
    """Regression: save used 4 KiB cap so sim `client.frames` never parsed."""
    server = CraneLiveServer(('127.0.0.1', 0), payload={})
    host, port = server.server_address
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        big_frames = [{'t': i, 'x': i * 0.01, 'vxCmd': 0.1} for i in range(200)]
        body = json.dumps({
            'client': {
                'page': 'ptp',
                'sim_replay': True,
                'frames': big_frames,
            },
        }).encode('utf-8')
        assert len(body) > 4096
        conn = HTTPConnection(host, port, timeout=5)
        conn.request(
            'POST',
            '/api/run-log/save',
            body=body,
            headers={'Content-Type': 'application/json'},
        )
        resp = conn.getresponse()
        assert resp.status == 200
        payload = json.loads(resp.read().decode('utf-8'))
        assert payload['client']['frames'][0]['t'] == 0
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
