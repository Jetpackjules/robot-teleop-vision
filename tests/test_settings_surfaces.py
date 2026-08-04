from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def source(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_dormant_oakd_and_launcher_wiring_are_not_inspector_controls():
    view = source("godot/runtime/point_cloud/unified_point_cloud_view.gd")

    assert '@export_group("OAK-D Camera")' not in view
    assert "DORMANT_OAKD_PROPERTIES" in view
    assert '"oakd_enabled"' in view
    assert "CORE_DEVELOPER_PROPERTIES" in view
    assert '"tracker_control_port"' in view
    assert '"sync_fps_to_slowest"' in view
    assert '"camera_diagnostic_view"' in view


def test_realsense_camera_exposes_only_wired_expert_depth_controls():
    camera = source("godot/runtime/point_cloud/realsense_camera_node.gd")

    assert '@export_group("Expert Depth Tuning")' in camera
    assert '@export_subgroup("RealSense SDK Filter Chain")' in camera
    assert '@export_subgroup("FastFoundation Depth")' in camera
    assert "@export_storage var filters_for_geometry" in camera
    assert "@export_storage var geometry_edge_guard_m" in camera
    assert "@export_storage var stabilization_enabled" in camera
    assert "PROPERTY_USAGE_READ_ONLY" in camera


def test_stream_preset_is_saved_and_restored_without_fov_side_channel():
    controller = source("web/controller_hybrid.js")
    module = source("robot_modules/so101/web/module.js")

    assert "stream_preset: streamPresetSelect.value" in controller
    assert 'saved.stream_preset === "latency"' in controller
    assert "feedbackMask" not in module
    assert "fov: Number(payload.fov) +" not in module


def test_stream_server_and_gateway_wiring_use_storage_only_properties():
    stream = source("godot/runtime/streaming/viewport_stream_server.gd")
    gateway = source("godot/runtime/remote_control_gateway.gd")

    assert "@export var" not in stream
    assert "@export_range" not in stream
    assert "@export_multiline" not in stream
    assert "@export_storage var target_fps" in stream
    assert "@export_storage var listen_port" in gateway


def test_so101_automatic_feedback_defaults_and_recovery_grouping():
    web = source("robot_modules/so101/web/module.js")

    assert 'id="so101-measured-feedback-enabled"' not in web
    assert 'id="so101-freeze-overlay-on-stale-enabled"' not in web
    assert "arm_measured_feedback_enabled: true" in web
    assert "arm_freeze_overlay_on_stale_enabled: true" in web
    assert "Experimental safeguards" in web
    assert "Recovery tools" in web


def test_browser_tracking_has_no_unused_launcher_adapter_config():
    config = source("robot_teleop/config.py")
    registry = source("robot_teleop/registry.py")
    examples = "\n".join(
        source(path)
        for path in (
            "config/examples/vision_only.toml",
            "config/examples/so101_realsense.toml",
        )
    )

    assert "TrackingConfig" not in config
    assert '"tracking"' not in registry
    assert "[tracking]" not in examples
