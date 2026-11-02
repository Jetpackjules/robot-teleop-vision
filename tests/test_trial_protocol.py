from contextlib import contextmanager
import http.client
import json
import threading
import pytest
from robot_teleop.trial_protocol import load_protocol, save_protocol
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
        assert 'Balance test' in value['tasks']['custom']['instruction']
        value['time_limit_seconds'] = 90
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
        assert not server.trial_protocol_path.exists()
        save_protocol(server.trial_protocol_path, value)
        with pytest.raises(ValueError):save_protocol(server.trial_protocol_path, {**value, 'version':True})
        assert load_protocol(server.trial_protocol_path) == value
