from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def source(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_dormant_oakd_and_launcher_wiring_are_not_inspector_controls():
    view = source("godot/runtime/point_cloud/unified_point_cloud_view.gd")
    scene = source("godot/Main.tscn")

    assert '@export_group("OAK-D Camera")' not in view
    assert "DORMANT_OAKD_PROPERTIES" in view
    assert '"oakd_enabled"' in view
    assert "CORE_DEVELOPER_PROPERTIES" in view
    assert '"tracker_control_port"' in view
    assert '"sync_fps_to_slowest"' in view
    assert '"camera_diagnostic_view"' in view
    assert "if oakd_enabled and CAMERA_OAKD not in ids:" in view
    assert "_remove_empty_dormant_oakd_anchor()" in view
    assert "oakd_enabled = false" not in scene
    assert '[node name="CalibrationPairs"' not in scene
    ensure_anchors = view.split("func _ensure_scene_anchors() -> void:", 1)[1].split(
        "func _remove_empty_dormant_oakd_anchor() -> void:", 1
    )[0]
    assert "_calibration_pairs_node(true)" not in ensure_anchors


def test_realsense_camera_exposes_only_wired_expert_depth_controls():
    camera = source("godot/runtime/point_cloud/realsense_camera_node.gd")
    native = source(
        "native/realsense_shared_memory/src/realsense_direct_frame_source.cpp"
    )

    assert '@export_group("Expert Depth Tuning")' in camera
    assert '@export_subgroup("RealSense SDK Filter Chain")' in camera
    assert '@export_subgroup("FastFoundation Depth")' not in camera
    assert "@export_storage var depth_source" in camera
    assert "@export_storage var fast_backend" in camera
    assert "@export_storage var fast_profile" in camera
    assert "@export_storage var filters_for_geometry" in camera
    assert "@export_storage var geometry_edge_guard_m" in camera
    assert "@export_storage var stabilization_enabled" in camera
    assert "PROPERTY_USAGE_READ_ONLY" in camera
    assert "FAST_FOUNDATIONSTEREO_ONNX" in native
    assert "experiments/oakd_head_tracker_demo" not in native


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


def test_runtime_stream_owner_heartbeat_is_atomic_and_quiet():
    view = source("godot/runtime/point_cloud/unified_point_cloud_view.gd")

    owner_code = view.split("func _read_runtime_stream_owner_active() -> bool:", 1)[1].split(
        "func _camera_stride(camera_id: String) -> int:", 1
    )[0]
    assert "_parse_json_dictionary_quietly" in owner_code
    assert "JSON.parse_string" not in owner_code
    assert "DirAccess.rename_absolute" in owner_code


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
