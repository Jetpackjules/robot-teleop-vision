from types import SimpleNamespace
import pytest
from robot_teleop.operator import load_robot_operator
from tools.lan_remote_view_server import LanRemoteHandler


@pytest.mark.parametrize('detail', [0, -2, -1, 20000, 40000])
def test_point_detail_is_a_valid_persistent_site_setting(detail):
    handler = LanRemoteHandler.__new__(LanRemoteHandler)
    handler.server = SimpleNamespace(robot_operator=load_robot_operator('disabled'))
    value = handler.validate_site_settings({'point_preview_detail':detail, 'stream_preset':'latency'})
    assert value['point_preview_detail'] == detail
    assert value['stream_preset'] == 'latency'


@pytest.mark.parametrize('detail', [True, '40000', 40.5, 10000, -3, None])
def test_point_detail_rejects_unsupported_values(detail):
    handler = LanRemoteHandler.__new__(LanRemoteHandler)
    handler.server = SimpleNamespace(robot_operator=load_robot_operator('disabled'))
    with pytest.raises(ValueError):
        handler.validate_site_settings({'point_preview_detail':detail})
