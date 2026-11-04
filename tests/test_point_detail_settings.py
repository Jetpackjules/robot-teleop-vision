from types import SimpleNamespace
import pytest
from robot_teleop.operator import load_robot_operator
from tools.lan_remote_view_server import LanRemoteHandler


def test_arm_body_opacity_persists_and_rejects_nonfinite_values():
    handler = LanRemoteHandler.__new__(LanRemoteHandler)
    handler.server = SimpleNamespace(robot_operator=load_robot_operator('disabled'))
    for value, expected in [(0, 0), (.25, .25), (1, 1), (-1, 0), (2, 1)]:
        assert handler.validate_site_settings({'robot_overlay_body_opacity':value})['robot_overlay_body_opacity'] == expected
    for value in [True, '0.5', float('nan'), float('inf')]:
        with pytest.raises(ValueError):handler.validate_site_settings({'robot_overlay_body_opacity':value})


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


def test_cleanup_controls_persist_on_host_and_reject_fractional_support():
    handler = LanRemoteHandler.__new__(LanRemoteHandler)
    handler.server = SimpleNamespace(robot_operator=load_robot_operator('disabled'))
    value = handler.validate_site_settings({'point_cleanup_enabled': True, 'point_cleanup_support': 3, 'point_cleanup_radius_mm': 15})
    assert value['point_cleanup_support'] == 3
    assert value['point_cleanup_radius_mm'] == 15
    assert value['point_cleanup_enabled'] is True
    with pytest.raises(ValueError):
        handler.validate_site_settings({'point_cleanup_support': 2.5})


def test_arm_mask_buffer_persists_with_finite_bounds():
    handler = LanRemoteHandler.__new__(LanRemoteHandler)
    handler.server = SimpleNamespace(robot_operator=load_robot_operator('disabled'))
    for value, expected in [(0, 0), (25, 25), (50, 50), (-1, 0), (75, 50)]:
        assert handler.validate_site_settings({'robot_overlay_body_buffer_mm': value})['robot_overlay_body_buffer_mm'] == expected
    for value in [True, '25', float('nan'), float('inf')]:
        with pytest.raises(ValueError):
            handler.validate_site_settings({'robot_overlay_body_buffer_mm': value})


@pytest.mark.parametrize('key,maximum', [('robot_overlay_claw_opacity', 1), ('robot_overlay_claw_buffer_mm', 50)])
def test_claw_settings_persist_with_independent_bounds(key, maximum):
    handler = LanRemoteHandler.__new__(LanRemoteHandler)
    handler.server = SimpleNamespace(robot_operator=load_robot_operator('disabled'))
    for value, expected in [(0, 0), (maximum / 2, maximum / 2), (maximum * 2, maximum), (-1, 0)]:
        assert handler.validate_site_settings({key: value})[key] == expected
    for value in [True, '25', float('nan'), float('inf')]:
        with pytest.raises(ValueError):
            handler.validate_site_settings({key: value})


def test_squares_fusion_is_an_optional_persistent_boolean():
    handler=LanRemoteHandler.__new__(LanRemoteHandler)
    handler.server=SimpleNamespace(robot_operator=load_robot_operator('disabled'))
    assert 'squares_fusion_enabled' not in handler.validate_site_settings({})
    for enabled in [False,True]:
        assert handler.validate_site_settings({'squares_fusion_enabled':enabled})['squares_fusion_enabled'] is enabled
    for bad in [1,'true',None]:
        with pytest.raises(ValueError):handler.validate_site_settings({'squares_fusion_enabled':bad})
