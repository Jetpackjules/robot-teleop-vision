from contextlib import contextmanager
import http.client
import json
import threading
import pytest
from robot_teleop.trial_protocol import LEGACY_TASKS, STUDY_TASKS, load_protocol, save_protocol, validate_protocol
from tools.lan_remote_view_server import LanRemoteHandler, LanRemoteServer


@contextmanager
def host(root):
    (root/'web').mkdir(exist_ok=True)
    server = LanRemoteServer(('127.0.0.1', 0), LanRemoteHandler, root/'web',
        '127.0.0.1', 0, '127.0.0.1', 0, 'test-only', robot_module='disabled',
        robot_status_port=0, robot_calibration_status_port=0, site_settings_path=root/'site.json')
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        server.shutdown();server.server_close();thread.join(timeout=2)


def request(server, method='GET', value=None, origin=True, authenticated=True):
    address = f'127.0.0.1:{server.server_port}'
    headers = {'Content-Type': 'application/json'}
    if authenticated: headers['Cookie'] = 'godot_remote_session='+server.session_cookie
    if origin: headers['Origin'] = 'http://'+address
    connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=5)
    connection.request(method, '/api/v1/trial-protocol', None if value is None else json.dumps(value), headers)
    response = connection.getresponse();body=response.read();status=response.status;connection.close()
    return status, body


def test_trial_protocol_survives_server_restart_and_is_separate_from_calibration(tmp_path):
    with host(tmp_path) as server:
        status, body = request(server)
        value = json.loads(body)
        assert status == 200 and value['time_limit_seconds'] == 75
        assert value['version'] == 4 and value['practice_time_limit_seconds'] == 60
        assert value['tasks']['tray_transfer_v1']['time_limit_seconds'] == 60
        assert value['tasks']['single_block']['success_metric'] == 'practice'
        assert {key: value['tasks'][key] for key in STUDY_TASKS} == STUDY_TASKS
        assert 'Balance test' in value['tasks']['custom']['instruction']
        value['time_limit_seconds'] = 90
        value['practice_time_limit_seconds'] = 120
        value['tasks']['custom']['success_metric'] = 'block_count'
        status, body = request(server, 'POST', value)
        assert status == 200 and json.loads(body) == value
    with host(tmp_path) as restarted:
        status, body = request(restarted)
        assert status == 200 and json.loads(body) == value
    assert list((tmp_path/'.teleop').iterdir()) == [tmp_path/'.teleop/trial-protocol.json']


def test_trial_protocol_requires_authentication_same_origin_and_valid_bounds(tmp_path):
    with host(tmp_path) as server:
        value = load_protocol(server.trial_protocol_path)
        assert request(server, authenticated=False)[0] == 303
        assert request(server, 'POST', value, authenticated=False)[0] == 401
        assert request(server, 'POST', value, origin=False)[0] == 403
        for limit in (True, 0, 29, 7201, 75.5):
            assert request(server, 'POST', {**value, 'time_limit_seconds':limit})[0] == 409
            assert request(server, 'POST', {**value, 'practice_time_limit_seconds':limit})[0] == 409
        assert not server.trial_protocol_path.exists()
        save_protocol(server.trial_protocol_path, value)
        with pytest.raises(ValueError):save_protocol(server.trial_protocol_path, {**value, 'version':True})
        assert load_protocol(server.trial_protocol_path) == value


def test_legacy_protocol_upgrades_future_task1_only_without_rewriting_original(tmp_path):
    value = load_protocol(tmp_path/'absent.json')
    legacy = {key: value[key] for key in ('version', 'time_limit_seconds', 'tasks')}
    legacy['version'] = 1
    legacy['tasks'] = {key: task for key, task in legacy['tasks'].items() if key in LEGACY_TASKS}
    legacy['tasks']['single_block'] = {'instruction': 'Move the object from one plate to the other', 'success_metric':'pass_fail'}
    legacy['tasks']['custom']['instruction'] = 'Keep this lab-specific balance instruction'
    path = tmp_path/'legacy.json'
    original = json.dumps(legacy).encode()
    path.write_bytes(original)
    migrated = load_protocol(path)
    assert path.read_bytes() == original
    assert migrated['version'] == 4 and migrated['practice_time_limit_seconds'] == 60
    assert migrated['tasks']['single_block']['success_metric'] == 'practice'
    assert migrated['tasks']['custom'] == legacy['tasks']['custom']
    assert migrated['tasks']['count_blocks'] == legacy['tasks']['count_blocks']
    with pytest.raises(ValueError):
        validate_protocol({**migrated, 'tasks':{**migrated['tasks'],'single_block':legacy['tasks']['single_block']}})


def test_task1_upgrade_and_legacy_save_preserve_other_tasks_and_transfer_settings(tmp_path):
    path = tmp_path/'protocol.json'
    current = load_protocol(path)
    legacy = {**current, 'version': 2, 'tasks': {k:v for k,v in current['tasks'].items() if k in LEGACY_TASKS}}
    path.write_text(json.dumps(legacy))
    original = path.read_bytes()
    upgraded = load_protocol(path)
    assert path.read_bytes() == original
    assert upgraded['time_limit_seconds'] == legacy['time_limit_seconds']
    for key in legacy['tasks']: assert upgraded['tasks'][key] == legacy['tasks'][key]
    upgraded['tasks']['tray_transfer_v1']['time_limit_seconds'] = 90
    save_protocol(path, upgraded)
    assert save_protocol(path, legacy)['tasks']['tray_transfer_v1']['time_limit_seconds'] == 90
    for seconds in (True, 0, 29, 7201, 60.5):
        invalid = json.loads(json.dumps(upgraded))
        invalid['tasks']['tray_transfer_v1']['time_limit_seconds'] = seconds
        with pytest.raises(ValueError):validate_protocol(invalid)
