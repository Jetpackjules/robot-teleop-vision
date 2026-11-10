import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from robot_teleop.realsense_depth_recovery import DepthRecovery


@pytest.fixture
def recovery(tmp_path, monkeypatch):
    serial = "262522075787"
    device = Mock()
    device.get_info.side_effect = {"serial": serial, "name": "Intel RealSense D435"}.__getitem__
    control = SimpleNamespace(deepSeaSecondPeakThreshold=325, lrAgreeThreshold=24)
    advanced = Mock()
    advanced.get_depth_control.side_effect = lambda: SimpleNamespace(**vars(control))
    advanced.set_depth_control.side_effect = lambda value: vars(control).update(vars(value))
    sdk = SimpleNamespace(
        context=lambda: SimpleNamespace(query_devices=lambda: [device]),
        camera_info=SimpleNamespace(serial_number="serial", name="name"),
        rs400_advanced_mode=lambda device: advanced,
    )
    monkeypatch.setattr("robot_teleop.realsense_depth_recovery.find_spec", lambda name: True)
    monkeypatch.setitem(sys.modules, "pyrealsense2", sdk)
    return DepthRecovery(tmp_path / "recovery.json"), serial, control


def test_restore_original_threshold_across_restart(recovery):
    settings, serial, control = recovery
    settings.set_enabled(serial, True)
    assert control.deepSeaSecondPeakThreshold == 0
    assert control.lrAgreeThreshold == 24
    # Simulate a camera power cycle as well as a server restart.
    control.deepSeaSecondPeakThreshold = 325
    new = DepthRecovery(settings.path)
    new._apply_saved()
    assert control.deepSeaSecondPeakThreshold == 0
    assert new.set_enabled(serial, False)["enabled"] == []
    assert control.deepSeaSecondPeakThreshold == 325
    assert control.lrAgreeThreshold == 24


def test_undo_respects_a_later_external_adjustment(recovery):
    settings, serial, control = recovery
    settings.set_enabled(serial, True)
    control.deepSeaSecondPeakThreshold = 180
    settings.set_enabled(serial, False)
    assert control.deepSeaSecondPeakThreshold == 180


def test_reject_unknown_camera_or_invalid_setting(recovery):
    settings, serial, control = recovery
    with pytest.raises(RuntimeError, match="not connected"):
        settings.set_enabled("1234567890", True)
    with pytest.raises(ValueError, match="on/off"):
        settings.set_enabled(serial, 1)
    assert settings.status()["enabled"] == []
    assert control.deepSeaSecondPeakThreshold == 325
