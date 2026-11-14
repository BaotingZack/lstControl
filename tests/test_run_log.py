"""Run log export (ControlState + /api/run-log/save payload builder)."""

import json
import threading
from http.client import HTTPConnection
from pathlib import Path

import pytest

from live_server import ControlState, CraneLiveServer
from web.run_log import (
    SETTLE_LOG_INTERVAL_S,
    SETTLE_STEPS_SCHEMA,
    auto_save_run_log,
    build_run_log_document,
    run_status,
    save_run_log_file,
    settle_step_worth_recording,
    slim_settle_step,
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
    cs.set_step({
        't': 0.1,
        'roll': 1.2,
        'pitch': -0.8,
        'theta_x': 0.01,
        'theta_y': -0.02,
        'vx_cmd': 0.03,
        'vy_cmd': -0.01,
        'antisway_dvx': 0.04,
        'antisway_dvy': 0.0,
        'x': 99.0,
        'y': 88.0,
    })
    cs.set_done()

    doc = build_run_log_document(
        cs,
        client_supplement={'page': 'settle', 'anti_sway_mode': 'crane_remote'},
        site={'hook_sway_angle_tol_deg': 0.3, 'secret': 'skip'},
    )
    assert doc['format'] == 'lstControl_run_log'
    assert doc['run']['kind'] == 'settle'
    assert doc['run']['steps_schema'] == SETTLE_STEPS_SCHEMA
    step = doc['run']['steps'][0]
    assert step['theta_x'] == 0.01
    assert 'x' not in step
    assert doc['run']['latest']['vx_cmd'] == 0.03
    assert doc['client']['page'] == 'settle'
    assert 'secret' not in doc['site_config']
    assert doc['site_config']['hook_sway_angle_tol_deg'] == 0.3


def test_slim_settle_step_drops_nonessential_fields():
    slim = slim_settle_step({
        't': 1.0,
        'theta_x': 0.1,
        'theta_y': None,
        'vx_cmd': 0.2,
        'x_measured': 3.0,
    })
    assert slim['sway_valid'] is False
    assert 'x_measured' not in slim
    assert 'plc_vx' not in slim
    assert 'vz_cmd' not in slim


def test_settle_step_decimation_in_control_state():
    cs = ControlState()
    cs.begin_run(kind='settle', anti_sway_mode='baseline', sim=False)
    base = {
        'theta_x': 0.01, 'theta_y': 0.0, 'vx_cmd': 0.1, 'vy_cmd': 0.0,
    }
    for t in (0.0, 0.05, 0.1, 0.5, 1.0, 1.05, 2.0):
        cs.set_step({**base, 't': t})
    cs.set_done()
    with cs.lock:
        times = [s['t'] for s in cs.steps]
    assert times == [0.0, 1.0, 2.0]


def test_settle_step_worth_recording_interval():
    assert settle_step_worth_recording({'t': 0.0}, last_log_t=None) is True
    assert settle_step_worth_recording({'t': 0.5}, last_log_t=0.0) is False
    assert settle_step_worth_recording(
        {'t': SETTLE_LOG_INTERVAL_S}, last_log_t=0.0,
    ) is True


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


def test_auto_save_settle_run_log_slim_steps(tmp_path: Path):
    cs = ControlState()
    cs.begin_run(kind='settle', anti_sway_mode='baseline', sim=False)
    cs.set_step({'t': 0.0, 'theta_x': 0.05, 'theta_y': 0.01, 'vx_cmd': 0.1, 'x': 1.0})
    cs.set_done()
    path = auto_save_run_log(cs, log_dir=tmp_path)
    assert path is not None
    assert path.name.startswith('run_log_settle_')
    doc = json.loads(path.read_text(encoding='utf-8'))
    assert doc['run']['steps_schema'] == SETTLE_STEPS_SCHEMA
    assert 'x' not in doc['run']['steps'][0]


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
