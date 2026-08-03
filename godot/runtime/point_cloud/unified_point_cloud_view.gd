@tool
extends Node3D

const CAMERA_REALSENSE := "realsense"
const CAMERA_REALSENSE_PREFIX := "realsense:"
const CAMERA_OAKD := "oakd"
const CAMERA_IDS := [CAMERA_REALSENSE, CAMERA_OAKD]
const BIG_ARUCO_MARKER_SIZE_M := 0.15
const BIG_ARUCO_DICTIONARY := "4x4_50"
const WORLD_LEVEL_ANCHOR_NAME := "WorldLevelAnchor"
const CAMERA_CLOUDS_NAME := "CameraClouds"
const CAMERA_CLOUDS_NODE := "WorldLevelAnchor/CameraClouds"
const CALIBRATION_PAIRS_NAME := "CalibrationPairs"
const CALIBRATION_PAIRS_NODE := "WorldLevelAnchor/CalibrationPairs"
const CALIBRATION_REFERENCE_NODE := "Reference"
const CALIBRATION_TARGET_NODE := "Target"
const DEBUG_PANEL_ANCHOR_NODE := "DebugPanelAnchor"
const ROBOT_OVERLAY_NAME := "RobotOverlay"
const ROBOT_OVERLAY_NODE := "WorldLevelAnchor/RobotOverlay"
const REALSENSE_VIEWER_MESH_DEPTH_DELTA := 0.05
const REALSENSE_VIEWER_MESH_EDGE_M := 0.08
const RUNTIME_STREAM_OWNER_PATH := "user://godot_realsense_runtime_owner.json"
const RUNTIME_STREAM_OWNER_TIMEOUT_MSEC := 2000
const RUNTIME_STREAM_OWNER_WRITE_INTERVAL_MSEC := 250
const RUNTIME_STREAM_OWNER_CHECK_INTERVAL_MSEC := 150
const REALSENSE_SCAN_INTERVAL_MSEC := 2000
const REALSENSE_DUPLICATE_SUFFIXES := ["A", "B", "C", "D", "E", "F", "G", "H"]
const CAMERA_ALIGNMENT_REGISTRY_PATH := "user://camera_alignment_registry.json"
const REALSENSE_ALIGNMENT_GROUND_TRUTH_PATH := "user://realsense_alignment_ground_truth.json"
const REALSENSE_CLOUD_ALIGNMENT_RESULT_PATH := "user://realsense_cloud_alignment_result.json"
const REALSENSE_CLOUD_ALIGNMENT_SCRIPT_PATH := "res://tools/realsense_cloud_align.py"
const REALSENSE_CAMERA_NODE_SCRIPT_PATH := "res://godot/runtime/point_cloud/realsense_camera_node.gd"
const REALSENSE_NATIVE_CALIBRATION_MODEL_PATH := "res://native/realsense_shared_memory/models/superpoint_lightglue_pipeline.onnx"
const WORLD_LEVEL_PATH := "user://unified_world_level.json"
const LEGACY_CALIBRATION_PROPERTIES := [
	"request_realsense_cloud_auto_align_now",
	"request_realsense_cloud_refine_alignment_now",
	"request_realsense_cloud_guarded_refine_now",
	"request_realsense_cloud_color_refine_now",
	"request_realsense_rgb_feature_align_now",
	"request_realsense_rgb_feature_rigid_now",
	"request_realsense_plane_align_now",
	"request_realsense_axis_search_refine_now",
	"save_realsense_alignment_ground_truth_now",
	"apply_saved_realsense_alignment_now",
	"solve_realsense_point_pairs_now",
	"request_realsense_alignment_benchmark_now",
	"realsense_cloud_alignment_frames",
	"realsense_cloud_alignment_voxel_m",
	"realsense_cloud_alignment_max_correspondence_m",
	"realsense_manual_nudge_target_serial",
	"realsense_manual_nudge_translation_step_m",
	"realsense_manual_nudge_rotation_step_deg",
	"nudge_realsense_x_neg",
	"nudge_realsense_x_pos",
	"nudge_realsense_y_neg",
	"nudge_realsense_y_pos",
	"nudge_realsense_z_neg",
	"nudge_realsense_z_pos",
	"nudge_realsense_pitch_neg",
	"nudge_realsense_pitch_pos",
	"nudge_realsense_yaw_neg",
	"nudge_realsense_yaw_pos",
	"nudge_realsense_roll_neg",
	"nudge_realsense_roll_pos",
	"reset_realsense_manual_alignment_now",
	"request_big_aruco_alignment_now",
	"reload_alignment_now",
	"big_aruco_marker_ids",
	"big_aruco_auto_depth_refine",
]

@export_group("Workflow")
## UDP control port used by launch_web_stack.py. Performance impact: none unless changed to the wrong port.
@export var tracker_control_port: int = 4244
## Starts or stops both point-cloud streams for this editor view. Performance impact: high when enabled because both cameras publish live SHM frames.
@export var editor_stream_enabled: bool = false:
	set(value):
		if editor_stream_enabled == value:
			return
		editor_stream_enabled = value
		_update_camera_renderers()
		_send_all_stream_commands()
## Shows the in-world FPS table. Performance impact: low; it updates a small viewport texture four times per second.
@export var show_debug_panel: bool = true:
	set(value):
		show_debug_panel = value
		_update_debug_panel(true)
## Loads the last OAK-D to RealSense alignment JSON automatically. Performance impact: none; it only reads a small file.
@export var auto_apply_alignment_file: bool = true:
	set(value):
		auto_apply_alignment_file = value
		if value:
			_poll_alignment_result(true)
## Restores the intended simple defaults for this new view. Performance impact: applies settings that favor clarity and stable FPS over maximum mesh detail.
@export var apply_clean_defaults_now: bool = false:
	set(value):
		apply_clean_defaults_now = false
		if value:
			_apply_clean_defaults()

@export_group("Universal Point Cloud")
## Rejects points closer than this distance. Performance impact: low; changing it updates the renderer and stream clipping.
@export_range(0.05, 2.0, 0.01, "suffix:m") var min_depth_m: float = 0.20:
	set(value):
		min_depth_m = value
		_send_all_stream_commands()
		_update_camera_renderers()
## Rejects points farther than this distance. Performance impact: low; lower values can reduce rendered points.
@export_range(0.25, 10.0, 0.05, "suffix:m") var max_depth_m: float = 4.50:
	set(value):
		max_depth_m = value
		_send_all_stream_commands()
		_update_camera_renderers()
## Caps faster camera publishing to the slower live camera. Performance impact: moderate; smoother sync, lower maximum publish FPS.
@export var sync_fps_to_slowest: bool = false:
	set(value):
		sync_fps_to_slowest = value
		_send_all_stream_commands()
## independent_mesh draws both cameras untouched and is the project default. shader_mesh shares the reference camera's color in overlap. prefer_d455/prefer_d435 remove only the losing mesh where depth agrees. unified_mesh is the legacy D455-priority mode.
@export_enum("shader_mesh", "prefer_d455", "prefer_d435", "independent_mesh", "unified_mesh", "points", "point_splats", "cpu_mesh") var render_mode: String = "independent_mesh":
	set(value):
		if value == "gpu_mesh":
			value = "independent_mesh"
		if value not in ["points", "point_splats", "unified_mesh", "shader_mesh", "prefer_d455", "prefer_d435", "independent_mesh", "cpu_mesh"]:
			value = "point_splats"
		render_mode = value
		_send_all_stream_commands()
		_update_camera_renderers()
## Base screen size for points. Splat mode multiplies this slightly so splats are visibly different. Performance impact: moderate if very large.
@export_range(1.0, 12.0, 0.25) var point_pixel_size: float = 2.5:
	set(value):
		point_pixel_size = value
		_update_camera_renderers()
## Depth jump tolerance used by mesh edge rejection and feathering. 0.05 matches RealSense Viewer's filled point-cloud cutoff.
@export_range(0.01, 0.30, 0.005) var mesh_max_depth_delta_m: float = REALSENSE_VIEWER_MESH_DEPTH_DELTA:
	set(value):
		mesh_max_depth_delta_m = value
		_update_camera_renderers()
## Maximum connected edge length sent to mesh-aware publisher paths. Performance impact: low in SHM GPU mesh, moderate in CPU/TCP mesh paths.
@export_range(0.01, 0.40, 0.005, "suffix:m") var mesh_max_edge_m: float = REALSENSE_VIEWER_MESH_EDGE_M:
	set(value):
		mesh_max_edge_m = value
		_update_camera_renderers()
## Minimum triangle area for native mesh mode. Keep this at 0 if mesh disappears into shards. Performance impact: low.
@export_range(0.0, 0.001, 0.000001, "suffix:m2") var mesh_min_triangle_area_m2: float = 0.0:
	set(value):
		mesh_min_triangle_area_m2 = maxf(0.0, value)
		_update_camera_renderers()
## Uses the camera color texture on connected mesh mode instead of per-vertex colors. Usually looks richer but can reveal color/depth mismatch. Performance impact: low to moderate.
@export var texture_map_mesh: bool = true:
	set(value):
		texture_map_mesh = value
		_update_camera_renderers()
## Maximum surface disagreement still treated as one shared surface. Lower values preserve detail; higher values hide more double surfaces.
@export_range(0.005, 0.15, 0.005, "suffix:m") var mesh_fusion_depth_tolerance_m: float = 0.035:
	set(value):
		mesh_fusion_depth_tolerance_m = value
		_update_camera_renderers()

@export_group("RealSense Color Match")
## Optional reference serial. When blank, the darker D435 is preferred so the match does not wash out the wood surface.
@export var realsense_color_reference_serial: String = ""
## Fits the other live RealSense camera to the reference using depth-verified overlap samples.
@export_tool_button("Match RealSense Colors Now") var match_realsense_colors_action: Callable = _match_realsense_colors_now
@export_tool_button("Clear RealSense Color Matching") var clear_realsense_color_matching_action: Callable = _clear_realsense_color_matching
@export_multiline var realsense_color_match_status: String = "RealSense color matching has not been run."

@export_group("Robot Module")
## Shows the selected module's telemetry-driven overlay when that capability is available.
@export var robot_overlay_enabled: bool = true:
	set(value):
		robot_overlay_enabled = value
		_update_robot_overlay_settings()
## Diagnostic comparison view. Increase this to see the live scan through the
## telemetry-driven model without hiding either layer.
@export_range(0.0, 0.95, 0.05) var robot_overlay_transparency: float = 0.0:
	set(value):
		robot_overlay_transparency = clampf(value, 0.0, 0.95)
		_update_robot_overlay_settings()
## Hides point-cloud samples inside the animated robot links when the native renderer supports masks.
@export var robot_overlay_mask_scanned_robot: bool = true:
	set(value):
		robot_overlay_mask_scanned_robot = value
		_update_robot_overlay_settings()
## Runs the selected module's full calibration workflow, when supported.
@export_tool_button("Calibrate Robot") var calibrate_robot_position_action: Callable = _start_robot_position_calibration
## Runs the selected module's optional refinement workflow.
@export_tool_button("Refine Robot Calibration") var refine_robot_joint_alignment_action: Callable = _start_robot_joint_refinement
## Sends the selected module's guarded rest request, when supported.
@export_tool_button("Return Robot to Rest Pose") var return_robot_to_rest_action: Callable = _return_robot_to_rest_pose
## Saves the current calibrated position as an explicit user checkpoint. This checkpoint never constrains a new calibration.
@export_tool_button("Save Robot Position Calibration") var save_robot_position_action: Callable = _save_robot_position_checkpoint
## Replaces the current robot position with the checkpoint most recently saved above.
@export_tool_button("Restore Saved Robot Position") var restore_robot_anchor_action: Callable = _restore_saved_robot_position
## Removes the saved automatic base registration so the scene transform can be used again.
@export_tool_button("Clear Robot Position Calibration") var clear_robot_position_calibration_action: Callable = _clear_robot_position_calibration
@export_multiline var robot_position_calibration_status: String = "Robot position calibration is idle."

@export_group("World Level")
## Uses the registered robot base as gravity-up. Keep this off unless the physical base is known to be level.
@export var auto_level_from_robot_base: bool = false:
	set(value):
		auto_level_from_robot_base = value
		if not value and is_inside_tree():
			_clear_world_level(true)
## Re-applies world leveling from the current saved robot registration without recalibrating either camera.
@export_tool_button("Level World From Robot") var level_world_from_robot_action: Callable = level_world_from_robot_base
## Removes only the shared world-level correction. Camera-to-camera and robot registrations remain intact.
@export_tool_button("Clear World Level") var clear_world_level_action: Callable = _clear_world_level.bind(true)
@export_multiline var world_level_status: String = "World leveling has not been applied."

@export_group("Universal Point Cloud")
@export_subgroup("Cleanup")
## Shader-side cleanup for isolated depth pixels. Performance impact: low to moderate; effect is strongest on lone speckles, subtle on dense noisy surfaces.
@export var cleanup_enabled: bool = false:
	set(value):
		cleanup_enabled = value
		_update_camera_renderers()
## Neighbor depths within this range count as connected. Performance impact: low. Lower values remove more floaters but can bite real edges.
@export_range(0.005, 0.25, 0.005, "suffix:m") var cleanup_depth_delta_m: float = 0.055:
	set(value):
		cleanup_depth_delta_m = value
		_update_camera_renderers()
## How many of the four direct neighbors must be close in depth. Performance impact: low. 2 is visible cleanup; 3-4 can erase thin details.
@export_range(0.0, 4.0, 1.0) var cleanup_min_close_neighbors: float = 2.0:
	set(value):
		cleanup_min_close_neighbors = value
		_update_camera_renderers()
## Fades mesh edges near depth jumps instead of hard clipping. Performance impact: low; only affects GPU mesh.
@export var edge_feather_enabled: bool = true:
	set(value):
		edge_feather_enabled = value
		_update_camera_renderers()
## Width of the mesh edge fade zone. Performance impact: low. Higher values soften harsh mesh cuts.
@export_range(0.001, 0.20, 0.001, "suffix:m") var edge_feather_width_m: float = 0.035:
	set(value):
		edge_feather_width_m = value
		_update_camera_renderers()
## Lowest alpha used at feathered mesh edges. Performance impact: low. Lower is cleaner but can make edges vanish.
@export_range(0.0, 1.0, 0.01) var edge_feather_min_alpha: float = 0.20:
	set(value):
		edge_feather_min_alpha = value
		_update_camera_renderers()

@export_group("RealSense Camera")
## Auto-detects RealSense devices by serial and creates one renderer per discovered camera. Settings are stored per serial in Realsense Device Registry.
@export var realsense_auto_discover: bool = true:
	set(value):
		realsense_auto_discover = value
		_refresh_realsense_devices(true)
		_update_camera_renderers()
## Closes and reopens the native RealSense capture pipelines without restarting the editor.
@export var restart_realsense_now: bool = false:
	set(value):
		restart_realsense_now = false
		if value:
			_restart_realsense_direct_renderers()
## Applies the recommended 30-40 cm two-camera robot rig: D435 detail, D455 context, one active projector, and color-valid crops.
@export_tool_button("Apply Close Robot Rig Defaults") var apply_close_robot_rig_defaults_action: Callable = _apply_close_robot_rig_defaults
## Serial-keyed RealSense settings. Hidden backing store; edit each RealSense camera node instead.
@export_storage var realsense_device_registry: Dictionary = {}
@export_multiline var realsense_devices_status: String = ""
## Enables the RealSense camera in this view. Performance impact: high when on because it captures and publishes a live depth grid.
@export_storage var realsense_enabled: bool = true:
	set(value):
		realsense_enabled = value
		for camera_id in _realsense_camera_ids():
			_set_realsense_setting(camera_id, "enabled", value, false)
		_update_camera_renderers()
		_send_all_stream_commands()
@export_multiline var realsense_status: String = ""
## Temporarily isolates one depth source without stopping either capture pipeline.
@export_enum("all", "d455_only", "d435_only") var camera_diagnostic_view: String = "all":
	set(value):
		camera_diagnostic_view = value if value in ["all", "d455_only", "d435_only"] else "all"
		_update_camera_renderers()
## Disables RGB capture while diagnosing geometry, exposing invalid depth without texture hiding it.
@export var diagnostic_depth_only: bool = false:
	set(value):
		diagnostic_depth_only = value
		_update_camera_renderers()
## RealSense depth source. sdk_depth keeps the native RealSense SDK path; fast_foundation_native runs ONNX in the native extension on Windows builds; fast_foundation keeps the Python publisher fallback. Performance impact: high when FastFoundation is selected.
@export_storage var realsense_depth_source: String = "sdk_depth":
	set(value):
		if value not in ["sdk_depth", "fast_foundation_native", "fast_foundation"]:
			value = "sdk_depth"
		if realsense_depth_source == value:
			return
		var previous_source := realsense_depth_source
		realsense_depth_source = value
		if previous_source == "fast_foundation" and _realsense_direct_capture_active():
			_send_camera_stream_command(CAMERA_REALSENSE, false)
		_update_camera_renderers()
		_request_realsense_restart()
		_send_camera_stream_command(CAMERA_REALSENSE, _streams_enabled() and realsense_enabled)
## RealSense stream profile. Changing this restarts the RealSense pipeline because resolution/FPS are fixed at stream start.
@export_storage var realsense_stream_profile: String = "fast60":
	set(value):
		if value not in ["fast60", "viewer30", "highres30"]:
			value = "fast60"
		if realsense_stream_profile == value:
			return
		realsense_stream_profile = value
		_request_realsense_restart()
		_update_camera_renderers()
		_send_camera_stream_command(CAMERA_REALSENSE, _streams_enabled() and realsense_enabled)
## RealSense grid sampling stride. 1 keeps maximum RealSense detail; 2 halves each axis and is much faster. Performance impact: high.
@export_storage var realsense_stride: int = 1:
	set(value):
		realsense_stride = maxi(1, value)
		_update_camera_renderers()
		_send_camera_stream_command(CAMERA_REALSENSE, _streams_enabled() and realsense_enabled)
## Shows RealSense RGB on its point cloud. Off renders RealSense as grayscale for easier depth debugging. Performance impact: low.
@export_storage var realsense_color_enabled: bool = true:
	set(value):
		realsense_color_enabled = value
		_update_camera_renderers()
## Intel RealSense SDK post filters. Performance impact: high on this stack; often cuts publish FPS hard while looking subtle in the final cloud.
@export_storage var realsense_depth_filters_enabled: bool = false:
	set(value):
		realsense_depth_filters_enabled = value
		_update_camera_renderers()
		_send_camera_stream_command(CAMERA_REALSENSE, _streams_enabled() and realsense_enabled)
@export_subgroup("Post Processing")
@export_storage var realsense_decimation_filter_enabled: bool = true:
	set(value):
		realsense_decimation_filter_enabled = value
		_update_camera_renderers()
@export_storage var realsense_decimation_magnitude: int = 2:
	set(value):
		realsense_decimation_magnitude = clampi(value, 2, 8)
		_update_camera_renderers()
@export_storage var realsense_rotation_filter_enabled: bool = false:
	set(value):
		realsense_rotation_filter_enabled = value
		_update_camera_renderers()
@export_storage var realsense_hdr_merge_filter_enabled: bool = true:
	set(value):
		realsense_hdr_merge_filter_enabled = value
		_update_camera_renderers()
@export_storage var realsense_sequence_id_filter_enabled: bool = false:
	set(value):
		realsense_sequence_id_filter_enabled = value
		_update_camera_renderers()
@export_storage var realsense_threshold_filter_enabled: bool = false:
	set(value):
		realsense_threshold_filter_enabled = value
		_update_camera_renderers()
@export_storage var realsense_depth_to_disparity_filter_enabled: bool = true:
	set(value):
		realsense_depth_to_disparity_filter_enabled = value
		_update_camera_renderers()
@export_storage var realsense_spatial_filter_enabled: bool = true:
	set(value):
		realsense_spatial_filter_enabled = value
		_update_camera_renderers()
@export_storage var realsense_temporal_filter_enabled: bool = true:
	set(value):
		realsense_temporal_filter_enabled = value
		_update_camera_renderers()
@export_storage var realsense_hole_filling_filter_enabled: bool = false:
	set(value):
		realsense_hole_filling_filter_enabled = value
		_update_camera_renderers()
@export_storage var realsense_disparity_to_depth_filter_enabled: bool = true:
	set(value):
		realsense_disparity_to_depth_filter_enabled = value
		_update_camera_renderers()
@export_subgroup("")
## Uses filtered depth for geometry, not only preview. Performance impact: high when combined with SDK filters; can stabilize depth but costs FPS.
@export_storage var realsense_filters_for_geometry: bool = false:
	set(value):
		realsense_filters_for_geometry = value
		_update_camera_renderers()
		_send_camera_stream_command(CAMERA_REALSENSE, _streams_enabled() and realsense_enabled)
## Drops points near filtered RealSense edge changes. Performance impact: moderate; higher values clean edges but remove real geometry.
@export_storage var realsense_geometry_edge_guard_m: float = 0.04:
	set(value):
		realsense_geometry_edge_guard_m = value
		_send_camera_stream_command(CAMERA_REALSENSE, _streams_enabled() and realsense_enabled)
## RealSense SDK hole filling mode. Performance impact: low to moderate; visual impact depends on the scene.
@export_storage var realsense_hole_filling: int = 1:
	set(value):
		realsense_hole_filling = value
		_update_camera_renderers()
		_send_camera_stream_command(CAMERA_REALSENSE, _streams_enabled() and realsense_enabled)
## Freezes tiny RealSense depth changes in the published grid. Performance impact: low; reduces floor shimmer without SDK filter cost.
@export_storage var realsense_stabilization_enabled: bool = false:
	set(value):
		realsense_stabilization_enabled = value
		_send_camera_stream_command(CAMERA_REALSENSE, _streams_enabled() and realsense_enabled)
## RealSense depth changes smaller than this are treated as noise. Higher is steadier; lower follows motion more eagerly.
@export_storage var realsense_stabilization_deadband_m: float = 0.012:
	set(value):
		realsense_stabilization_deadband_m = maxf(0.0, value)
		_send_camera_stream_command(CAMERA_REALSENSE, _streams_enabled() and realsense_enabled)
## Holds briefly-missing RealSense grid cells for this many published frames. 0 disables hole hold.
@export_storage var realsense_stabilization_hold_frames: int = 1:
	set(value):
		realsense_stabilization_hold_frames = maxi(0, value)
		_send_camera_stream_command(CAMERA_REALSENSE, _streams_enabled() and realsense_enabled)
@export_subgroup("FastFoundation")
## RealSense FastFoundation backend. Matches the OAK-D options; sdk_depth ignores this setting.
@export_storage var realsense_fast_backend: String = "onnx_cuda":
	set(value):
		if value not in ["pytorch", "onnx_trt", "onnx_cuda", "trt_engine"]:
			value = "onnx_cuda"
		realsense_fast_backend = value
		_send_camera_stream_command(CAMERA_REALSENSE, _streams_enabled() and realsense_enabled)
## RealSense FastFoundation model profile. Matches the OAK-D FastFoundation profiles.
@export_storage var realsense_fast_profile: String = "fast_192x384_i2":
	set(value):
		if value not in ["full_320x736_i4", "rt_256x512_i2", "fast_192x384_i2"]:
			value = "fast_192x384_i2"
		realsense_fast_profile = value
		_send_camera_stream_command(CAMERA_REALSENSE, _streams_enabled() and realsense_enabled)
## RealSense FastFoundation solver iterations. sdk_depth ignores this setting.
@export_storage var realsense_fast_iters: int = 4:
	set(value):
		realsense_fast_iters = clampi(value, 1, 32)
		_send_camera_stream_command(CAMERA_REALSENSE, _streams_enabled() and realsense_enabled)
## RealSense FastFoundation input scaling. sdk_depth ignores this setting.
@export_storage var realsense_fast_scale: float = 0.5:
	set(value):
		realsense_fast_scale = clampf(value, 0.25, 1.0)
		_send_camera_stream_command(CAMERA_REALSENSE, _streams_enabled() and realsense_enabled)
@export_subgroup("")

@export_group("OAK-D Camera")
## Enables the OAK-D camera in this view. Performance impact: high when on, especially with FastFoundation depth.
@export var oakd_enabled: bool = true:
	set(value):
		oakd_enabled = value
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and value)
		_update_camera_renderers()
## Reopens only the OAK-D capture pipeline using the current OAK-D settings. Use this after changing OAK-D preset/resolution/FPS/source settings.
@export var restart_oakd_now: bool = false:
	set(value):
		restart_oakd_now = false
		if value:
			_request_oakd_restart()
## Live OAK-D status from the tracker. If this says restart required, press Restart Oakd Now to apply deferred pipeline settings.
@export_multiline var oakd_status: String = ""
## OAK-D grid sampling stride. 1 keeps maximum OAK-D detail; 2 halves each axis and is much faster. Performance impact: high.
@export_range(1, 8, 1) var oakd_stride: int = 1:
	set(value):
		oakd_stride = maxi(1, value)
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and oakd_enabled)
## Fixed OAK-D capture preset. 30fps_low_latency is the normal low-delay 400p stereo path; 60fps_stable is available for experiments.
@export_enum("30fps_low_latency", "60fps_stable", "30fps_quality") var oakd_capture_preset: String = "30fps_low_latency":
	set(value):
		if value not in ["30fps_low_latency", "60fps_stable", "30fps_quality"]:
			value = "30fps_low_latency"
		oakd_capture_preset = value
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and oakd_enabled)
## OAK-D depth source. FastFoundation is best-looking but GPU-heavy; DepthAI is device-side and simpler. Performance impact: high.
@export_enum("depthai", "fast_foundation", "host_sgbm") var oakd_depth_source: String = "fast_foundation":
	set(value):
		if value not in ["depthai", "fast_foundation", "host_sgbm"]:
			value = "fast_foundation"
		oakd_depth_source = value
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and oakd_enabled)
## Adds OAK-D RGB color to host/FastFoundation depth. Off uses grayscale and avoids color/depth alignment artifacts. Performance impact: moderate.
@export var oakd_color_enabled: bool = false:
	set(value):
		oakd_color_enabled = value
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and oakd_enabled)
		_update_camera_renderers()
## OAK-D color path used only when OAK-D Color Enabled is on. rgb_projected_stable damps projection flicker; rgb_projected is raw; rgb_preview is cheap and less aligned.
@export_enum("rgb_projected_stable", "rgb_projected", "rgb_preview") var oakd_color_mode: String = "rgb_projected_stable":
	set(value):
		if value not in ["rgb_projected_stable", "rgb_projected", "rgb_preview"]:
			value = "rgb_projected_stable"
		oakd_color_mode = value
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and oakd_enabled)
## Tiny render-only local Z offset for OAK-D to reduce z-fighting when aligned on top of RealSense. Performance impact: none. Set to 0 for exact visual transform.
@export_range(-0.03, 0.03, 0.001, "suffix:m") var oakd_render_depth_bias_m: float = 0.004:
	set(value):
		oakd_render_depth_bias_m = value
		_update_camera_renderers()
## Drops OAK-D geometry near local depth jumps before publishing the point cloud. Cleans fringe/flying pixels across points, splats, and meshes. Set 0 to disable.
@export_range(0.0, 0.30, 0.005, "suffix:m") var oakd_geometry_edge_guard_m: float = 0.06:
	set(value):
		oakd_geometry_edge_guard_m = maxf(0.0, value)
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and oakd_enabled)
## Drops a small raw-pixel border from the OAK-D depth image before publishing. Removes sensor-edge strips across every render mode.
@export_range(0, 64, 1, "suffix:px") var oakd_border_crop_px: int = 12:
	set(value):
		oakd_border_crop_px = maxi(0, value)
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and oakd_enabled)
## Freezes tiny OAK-D depth changes in the published grid. Performance impact: low; reduces floor shimmer without changing depth source quality.
@export var oakd_stabilization_enabled: bool = true:
	set(value):
		oakd_stabilization_enabled = value
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and oakd_enabled)
## OAK-D depth changes smaller than this are treated as noise. FastFoundation usually benefits from a slightly larger value than RealSense.
@export_range(0.0, 0.08, 0.001, "suffix:m") var oakd_stabilization_deadband_m: float = 0.018:
	set(value):
		oakd_stabilization_deadband_m = maxf(0.0, value)
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and oakd_enabled)
## Holds briefly-missing OAK-D grid cells for this many published frames. 0 disables hole hold.
@export_range(0, 4, 1) var oakd_stabilization_hold_frames: int = 1:
	set(value):
		oakd_stabilization_hold_frames = maxi(0, value)
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and oakd_enabled)
## FastFoundation backend. onnx_cuda avoids TensorRT ScatterND console errors; onnx_trt may be faster but can spam parser warnings. Performance impact: high.
@export_enum("onnx_cuda", "onnx_trt", "pytorch", "trt_engine") var oakd_fast_backend: String = "onnx_cuda":
	set(value):
		if value not in ["pytorch", "onnx_trt", "onnx_cuda", "trt_engine"]:
			value = "onnx_cuda"
		oakd_fast_backend = value
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and oakd_enabled)
## FastFoundation model profile. Realtime is fastest; full profile is slower and can look better. Performance impact: high.
@export_enum("rt_256x512_i2", "fast_192x384_i2", "full_320x736_i4") var oakd_fast_profile: String = "rt_256x512_i2":
	set(value):
		if value not in ["full_320x736_i4", "rt_256x512_i2", "fast_192x384_i2"]:
			value = "rt_256x512_i2"
		oakd_fast_profile = value
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and oakd_enabled)
## FastFoundation solver iterations. Performance impact: high; more iterations usually improve depth but reduce FPS.
@export_range(1, 32, 1) var oakd_fast_iters: int = 4:
	set(value):
		oakd_fast_iters = clampi(value, 1, 32)
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and oakd_enabled)
## FastFoundation input scaling. Performance impact: high; lower scale is faster and blurrier.
@export_range(0.25, 1.0, 0.05) var oakd_fast_scale: float = 0.5:
	set(value):
		oakd_fast_scale = clampf(value, 0.25, 1.0)
		_send_camera_stream_command(CAMERA_OAKD, _streams_enabled() and oakd_enabled)

@export_group("Calibration")
@export_subgroup("Core Actions")
## Recommended one-button calibration. Uses native SuperPoint + LightGlue RGB-D matching and guarded cloud refinement; no marker or Python process is used.
@export var auto_align_realsense_without_marker_now: bool = false:
	set(value):
		auto_align_realsense_without_marker_now = false
		if value:
			_request_native_realsense_calibration("markerless")
## Establishes a high-confidence reference from a shared DICT_4X4_100 marker and saves it as the calibration ground truth.
@export var calibrate_realsense_with_aruco_now: bool = false:
	set(value):
		calibrate_realsense_with_aruco_now = false
		if value:
			_request_native_realsense_calibration("aruco")
## Tries a local native geometry refinement from the current transform and keeps the current transform unless whole-cloud overlap improves.
@export var refine_current_realsense_alignment_now: bool = false:
	set(value):
		refine_current_realsense_alignment_now = false
		if value:
			_request_native_realsense_calibration("refine")
@export_subgroup("Native Setup")
@export_range(0, 999, 1) var realsense_calibration_marker_id: int = 49
@export_range(0.02, 1.0, 0.005, "suffix:m") var realsense_calibration_marker_size_m: float = 0.15
@export_range(0.5, 6.0, 0.1, "suffix:m") var realsense_calibration_max_depth_m: float = 3.0
@export_subgroup("")
## Depth-only RealSense-to-RealSense alignment. It tries coarse geometry registration and then ICP; no ArUco, ChArUco, color, or texture is used.
@export var request_realsense_cloud_auto_align_now: bool = false:
	set(value):
		request_realsense_cloud_auto_align_now = false
		if value:
			_request_realsense_cloud_alignment("auto")
## Depth-only RealSense-to-RealSense ICP refinement. Starts from the current remembered transform and tightens it.
@export var request_realsense_cloud_refine_alignment_now: bool = false:
	set(value):
		request_realsense_cloud_refine_alignment_now = false
		if value:
			_request_realsense_cloud_alignment("refine")
## Static-camera-safe ICP refinement. Keeps the current transform if ICP tries to jump to a bad local overlap.
@export var request_realsense_cloud_guarded_refine_now: bool = false:
	set(value):
		request_realsense_cloud_guarded_refine_now = false
		if value:
			_request_realsense_cloud_alignment("guarded_refine")
## Texture-aware RealSense-to-RealSense refinement. Starts from the current transform and uses both geometry and RGB color.
@export var request_realsense_cloud_color_refine_now: bool = false:
	set(value):
		request_realsense_cloud_color_refine_now = false
		if value:
			_request_realsense_cloud_alignment("color_refine")
## RGB feature RealSense-to-RealSense alignment. Matches image features, lifts matches into 3D, then refines with cloud ICP.
@export var request_realsense_rgb_feature_align_now: bool = false:
	set(value):
		request_realsense_rgb_feature_align_now = false
		if value:
			_request_realsense_cloud_alignment("rgb_feature")
## RGB-D feature rigid alignment. Uses visual matches directly and skips ICP so it cannot drift into the wrong surface.
@export var request_realsense_rgb_feature_rigid_now: bool = false:
	set(value):
		request_realsense_rgb_feature_rigid_now = false
		if value:
			_request_realsense_cloud_alignment("rgb_feature_rigid")
## Plane-based RealSense alignment. Tries dominant room planes as the initial transform, then refines with ICP.
@export var request_realsense_plane_align_now: bool = false:
	set(value):
		request_realsense_plane_align_now = false
		if value:
			_request_realsense_cloud_alignment("plane_align")
## Bounded automatic nudge. Searches pitch/yaw/roll and X/Y/Z one axis at a time near the current transform.
@export var request_realsense_axis_search_refine_now: bool = false:
	set(value):
		request_realsense_axis_search_refine_now = false
		if value:
			_request_realsense_cloud_alignment("axis_search")
## Saves the current target RealSense transform as a benchmark truth after you manually align it.
@export var save_realsense_alignment_ground_truth_now: bool = false:
	set(value):
		save_realsense_alignment_ground_truth_now = false
		if value:
			_save_realsense_alignment_ground_truth()
## Applies the saved static RealSense alignment truth directly. Best for fixed cameras after one good manual calibration.
@export var apply_saved_realsense_alignment_now: bool = false:
	set(value):
		apply_saved_realsense_alignment_now = false
		if value:
			_apply_saved_realsense_alignment_ground_truth(true)
## Solves RealSense alignment from matching 3D marker points under CalibrationPairs/Reference and CalibrationPairs/Target.
@export var solve_realsense_point_pairs_now: bool = false:
	set(value):
		solve_realsense_point_pairs_now = false
		if value:
			_solve_realsense_point_pair_alignment()
## Locks fixed RealSense cameras to the saved alignment truth on scene load. Disable only when deliberately recalibrating.
@export var lock_realsense_to_saved_alignment: bool = true
## Runs multiple alignment strategies and ranks them by closeness to the saved manual ground truth.
@export var request_realsense_alignment_benchmark_now: bool = false:
	set(value):
		request_realsense_alignment_benchmark_now = false
		if value:
			_request_realsense_cloud_alignment("benchmark")
## Serial used as the alignment reference. Leave blank to use the first discovered enabled RealSense.
@export var realsense_cloud_alignment_reference_serial: String = ""
## Serial transformed into the reference camera. Leave blank to use the second discovered enabled RealSense.
@export var realsense_cloud_alignment_target_serial: String = ""
## Number of depth frames to capture per camera for the pure-cloud alignment worker.
@export_range(1, 30, 1) var realsense_cloud_alignment_frames: int = 8
## Point-cloud downsample size for Open3D registration. Larger is faster and coarser.
@export_range(0.01, 0.15, 0.005, "suffix:m") var realsense_cloud_alignment_voxel_m: float = 0.035
## Maximum point match distance during registration. 0 chooses about three voxels.
@export_range(0.0, 0.50, 0.005, "suffix:m") var realsense_cloud_alignment_max_correspondence_m: float = 0.0
@export_subgroup("Manual RealSense Nudge")
## Serial to nudge manually. Leave blank to use the current alignment target, usually the second enabled RealSense.
@export var realsense_manual_nudge_target_serial: String = ""
## Translation amount used by each nudge button.
@export_range(0.001, 0.25, 0.001, "suffix:m") var realsense_manual_nudge_translation_step_m: float = 0.01
## Rotation amount used by each nudge button.
@export_range(0.05, 10.0, 0.05, "suffix:deg") var realsense_manual_nudge_rotation_step_deg: float = 0.5
@export var nudge_realsense_x_neg: bool = false:
	set(value):
		nudge_realsense_x_neg = false
		if value:
			_apply_manual_realsense_alignment_delta(Vector3(-realsense_manual_nudge_translation_step_m, 0.0, 0.0), Vector3.ZERO)
@export var nudge_realsense_x_pos: bool = false:
	set(value):
		nudge_realsense_x_pos = false
		if value:
			_apply_manual_realsense_alignment_delta(Vector3(realsense_manual_nudge_translation_step_m, 0.0, 0.0), Vector3.ZERO)
@export var nudge_realsense_y_neg: bool = false:
	set(value):
		nudge_realsense_y_neg = false
		if value:
			_apply_manual_realsense_alignment_delta(Vector3(0.0, -realsense_manual_nudge_translation_step_m, 0.0), Vector3.ZERO)
@export var nudge_realsense_y_pos: bool = false:
	set(value):
		nudge_realsense_y_pos = false
		if value:
			_apply_manual_realsense_alignment_delta(Vector3(0.0, realsense_manual_nudge_translation_step_m, 0.0), Vector3.ZERO)
@export var nudge_realsense_z_neg: bool = false:
	set(value):
		nudge_realsense_z_neg = false
		if value:
			_apply_manual_realsense_alignment_delta(Vector3(0.0, 0.0, -realsense_manual_nudge_translation_step_m), Vector3.ZERO)
@export var nudge_realsense_z_pos: bool = false:
	set(value):
		nudge_realsense_z_pos = false
		if value:
			_apply_manual_realsense_alignment_delta(Vector3(0.0, 0.0, realsense_manual_nudge_translation_step_m), Vector3.ZERO)
@export var nudge_realsense_pitch_neg: bool = false:
	set(value):
		nudge_realsense_pitch_neg = false
		if value:
			_apply_manual_realsense_alignment_delta(Vector3.ZERO, Vector3(-realsense_manual_nudge_rotation_step_deg, 0.0, 0.0))
@export var nudge_realsense_pitch_pos: bool = false:
	set(value):
		nudge_realsense_pitch_pos = false
		if value:
			_apply_manual_realsense_alignment_delta(Vector3.ZERO, Vector3(realsense_manual_nudge_rotation_step_deg, 0.0, 0.0))
@export var nudge_realsense_yaw_neg: bool = false:
	set(value):
		nudge_realsense_yaw_neg = false
		if value:
			_apply_manual_realsense_alignment_delta(Vector3.ZERO, Vector3(0.0, -realsense_manual_nudge_rotation_step_deg, 0.0))
@export var nudge_realsense_yaw_pos: bool = false:
	set(value):
		nudge_realsense_yaw_pos = false
		if value:
			_apply_manual_realsense_alignment_delta(Vector3.ZERO, Vector3(0.0, realsense_manual_nudge_rotation_step_deg, 0.0))
@export var nudge_realsense_roll_neg: bool = false:
	set(value):
		nudge_realsense_roll_neg = false
		if value:
			_apply_manual_realsense_alignment_delta(Vector3.ZERO, Vector3(0.0, 0.0, -realsense_manual_nudge_rotation_step_deg))
@export var nudge_realsense_roll_pos: bool = false:
	set(value):
		nudge_realsense_roll_pos = false
		if value:
			_apply_manual_realsense_alignment_delta(Vector3.ZERO, Vector3(0.0, 0.0, realsense_manual_nudge_rotation_step_deg))
@export var reset_realsense_manual_alignment_now: bool = false:
	set(value):
		reset_realsense_manual_alignment_now = false
		if value:
			_reset_manual_realsense_alignment()
@export_subgroup("")
## Requests multi-marker big ArUco OAK-D to RealSense alignment. It uses every shared marker ID visible in both cameras. Performance impact: temporary background CPU/GPU work.
@export var request_big_aruco_alignment_now: bool = false:
	set(value):
		request_big_aruco_alignment_now = false
		if value:
			_request_big_aruco_alignment()
## Reloads the last saved alignment JSON. Performance impact: none.
@export var reload_alignment_now: bool = false:
	set(value):
		reload_alignment_now = false
		if value:
			_poll_alignment_result(true)
## Comma-separated big ArUco IDs. Calibration only uses IDs seen by both cameras in the same capture.
@export var big_aruco_marker_ids: String = "45,46,47,48,49"
## Runs depth ICP refinement after marker alignment. Performance impact: temporary high cost during calibration; can improve final transform.
@export var big_aruco_auto_depth_refine: bool = true
## Compact calibration result. If it says no shared markers, both cameras saw markers but not the same marker ID.
@export_multiline var calibration_status: String = ""

@export_group("Debug")
## Fallback world position for DebugPanelAnchor when the child node is missing. Move DebugPanelAnchor directly for normal layout work. Performance impact: none.
@export var debug_panel_position: Vector3 = Vector3(-1.35, 1.15, -1.0):
	set(value):
		debug_panel_position = value
		var anchor := _debug_panel_anchor(false)
		if anchor != null:
			anchor.position = debug_panel_position
## Pixel size scale for the in-world FPS table. Performance impact: none.
@export_range(0.0005, 0.01, 0.0001) var debug_panel_pixel_size: float = 0.0022:
	set(value):
		debug_panel_pixel_size = value
		if _debug_sprite != null:
			_debug_sprite.pixel_size = debug_panel_pixel_size
## Font size used inside the FPS table texture. Performance impact: low.
@export_range(10, 64, 1) var debug_font_size: int = 22:
	set(value):
		debug_font_size = value
		_rebuild_debug_table()
## Last generated debug table text for inspector readback only. Performance impact: none.
@export_multiline var debug_text: String = ""

var _command_udp: PacketPeerUDP = PacketPeerUDP.new()
var _camera_nodes: Dictionary = {}
var _point_cloud_stats_path: String = ""
var _point_cloud_stats_token: String = ""
var _point_cloud_stats: Dictionary = {}
var _alignment_result_path: String = ""
var _alignment_result_token: String = ""
var _alignment_transforms: Dictionary = {CAMERA_REALSENSE: Transform3D.IDENTITY, CAMERA_OAKD: Transform3D.IDENTITY}
var _camera_alignment_registry_path: String = ""
var _camera_alignment_registry_token: String = ""
var _realsense_alignment_ground_truth_path: String = ""
var _realsense_cloud_alignment_result_path: String = ""
var _realsense_cloud_alignment_result_token: String = ""
var _realsense_cloud_alignment_pid: int = -1
var _realsense_cloud_alignment_started_msec: int = 0
var _realsense_cloud_alignment_pending: bool = false
var _native_realsense_calibrator: Object = null
var _native_realsense_calibration_pending := false
var _native_realsense_calibration_mode := ""
var _native_realsense_calibration_reference_id := ""
var _native_realsense_calibration_target_id := ""
var _debug_viewport: SubViewport
var _debug_sprite: Sprite3D
var _debug_root: PanelContainer
var _debug_cells: Dictionary = {}
var _last_debug_update_msec: int = 0
var _native_missing_warned := false
var _stream_commands_active := false
var _runtime_stream_owner_path: String = ""
var _last_runtime_owner_write_msec: int = 0
var _last_runtime_owner_check_msec: int = -1000000
var _runtime_stream_owner_blocks_editor_cached := false
var _realsense_devices: Dictionary = {}
var _realsense_camera_order: Array[String] = []
var _realsense_labels: Dictionary = {}
var _last_realsense_scan_msec: int = -1000000
var _realsense_scan_helper: Object = null
var _world_level_path: String = ""
var _last_world_level_registration_token: float = 0.0

func _ready() -> void:
	add_to_group("unified_point_cloud_view")
	_point_cloud_stats_path = ProjectSettings.globalize_path("user://point_cloud_stream_stats.json")
	_alignment_result_path = ProjectSettings.globalize_path("user://oakd_realsense_alignment.json")
	_camera_alignment_registry_path = ProjectSettings.globalize_path(CAMERA_ALIGNMENT_REGISTRY_PATH)
	_realsense_alignment_ground_truth_path = ProjectSettings.globalize_path(REALSENSE_ALIGNMENT_GROUND_TRUTH_PATH)
	_realsense_cloud_alignment_result_path = ProjectSettings.globalize_path(REALSENSE_CLOUD_ALIGNMENT_RESULT_PATH)
	_runtime_stream_owner_path = ProjectSettings.globalize_path(RUNTIME_STREAM_OWNER_PATH)
	_world_level_path = ProjectSettings.globalize_path(WORLD_LEVEL_PATH)
	_refresh_realsense_devices(true)
	_poll_realsense_cloud_alignment_result(true)
	if Engine.is_editor_hint():
		_update_runtime_stream_owner_cache(true)
	else:
		_write_runtime_stream_owner(true)
	_ensure_scene_anchors()
	# Scene files can retain an editor-preview transform. Begin from the true
	# unlevelled state, then apply only an explicitly saved world-level file.
	_clear_world_level(false)
	if auto_level_from_robot_base:
		_load_world_level()
	else:
		_clear_world_level(true)
	if auto_apply_alignment_file:
		_poll_alignment_result(true)
	# The per-camera registry is authoritative over older one-off result files.
	_poll_camera_alignment_registry(true)
	if lock_realsense_to_saved_alignment:
		_apply_saved_realsense_alignment_ground_truth(false)
	_update_camera_renderers()
	_update_robot_overlay_settings()
	call_deferred("_maybe_auto_level_from_robot_registration")
	if editor_stream_enabled and not _editor_live_stream_allowed():
		realsense_status = _editor_live_stream_block_reason()
		oakd_status = realsense_status
	if _streams_enabled():
		_send_all_stream_commands()
	_update_debug_panel(true)

func _validate_property(property: Dictionary) -> void:
	var property_name := str(property.get("name", ""))
	if property_name in LEGACY_CALIBRATION_PROPERTIES:
		property["usage"] = PROPERTY_USAGE_STORAGE

func _exit_tree() -> void:
	if _native_realsense_calibrator != null and _native_realsense_calibrator.has_method("cancel"):
		_native_realsense_calibrator.cancel()
	if not Engine.is_editor_hint():
		_clear_runtime_stream_owner()
	if _stream_commands_active:
		_send_all_stream_commands(false)
	_free_debug_panel()

func _process(_delta: float) -> void:
	_update_runtime_stream_owner_state()
	_refresh_realsense_devices(false)
	_poll_point_cloud_stats()
	_poll_alignment_result(false)
	_poll_realsense_cloud_alignment_result(false)
	_poll_camera_alignment_registry(false)
	_update_realsense_cloud_alignment_process()
	_update_native_realsense_calibration()
	_update_robot_position_calibration_status()
	_update_camera_renderers()
	_update_realsense_direct_status()
	_update_debug_panel(false)

func _update_robot_overlay_settings() -> void:
	if not is_inside_tree():
		return
	var overlay := get_node_or_null(ROBOT_OVERLAY_NODE)
	if overlay == null:
		return
	if overlay.has_method("set_overlay_enabled"):
		overlay.call("set_overlay_enabled", robot_overlay_enabled)
	if overlay.has_method("set_overlay_transparency"):
		overlay.call(
			"set_overlay_transparency",
			robot_overlay_transparency,
		)
	if overlay.has_method("set_mask_scanned_robot"):
		overlay.call("set_mask_scanned_robot", robot_overlay_mask_scanned_robot)


func _active_robot_module() -> Node:
	if not is_inside_tree():
		return null
	var modules := get_tree().get_nodes_in_group("robot_module")
	return modules[0] if not modules.is_empty() else null

func _start_robot_position_calibration() -> void:
	var module := _active_robot_module()
	if module == null or not module.has_method("start_full_calibration"):
		robot_position_calibration_status = "The selected robot has no full calibration capability."
		if Engine.is_editor_hint():
			notify_property_list_changed()
		return
	module.call("start_full_calibration", Engine.is_editor_hint())
	_update_robot_position_calibration_status()


func _start_robot_joint_refinement() -> void:
	var module := _active_robot_module()
	if module == null or not module.has_method("refine_calibration"):
		robot_position_calibration_status = "The selected robot has no refinement capability."
		if Engine.is_editor_hint():
			notify_property_list_changed()
		return
	module.call("refine_calibration", Engine.is_editor_hint())
	_update_robot_position_calibration_status()


func _return_robot_to_rest_pose() -> void:
	var module := _active_robot_module()
	if module == null or not module.has_method("return_to_rest"):
		robot_position_calibration_status = "The selected robot has no rest-pose action."
		if Engine.is_editor_hint():
			notify_property_list_changed()
		return
	if not bool(module.call("return_to_rest")):
		robot_position_calibration_status = "Could not send the return-to-rest request."
	else:
		robot_position_calibration_status = "Return-to-rest requested; Hold can interrupt it."
	if Engine.is_editor_hint():
		notify_property_list_changed()


func _clear_robot_position_calibration() -> void:
	var module := _active_robot_module()
	if module != null and module.has_method("clear_calibration"):
		module.call("clear_calibration")
		_clear_world_level(true)
		_update_robot_position_calibration_status()
		return
	_clear_world_level(true)
	robot_position_calibration_status = "The selected robot has no clear-calibration action."

func _save_robot_position_checkpoint() -> void:
	var module := _active_robot_module()
	if module == null or not module.has_method("save_calibration_checkpoint"):
		robot_position_calibration_status = "The selected robot cannot save a calibration checkpoint."
		return
	if not bool(module.call("save_calibration_checkpoint")):
		robot_position_calibration_status = "Could not save the robot calibration checkpoint."
		return
	robot_position_calibration_status = "Current calibrated robot position saved as checkpoint."
	if Engine.is_editor_hint():
		notify_property_list_changed()


func _restore_saved_robot_position() -> void:
	var module := _active_robot_module()
	if module == null or not module.has_method("restore_calibration_checkpoint"):
		robot_position_calibration_status = "The selected robot cannot restore a calibration checkpoint."
		return
	if not bool(module.call("restore_calibration_checkpoint")):
		robot_position_calibration_status = "Could not restore the robot calibration checkpoint."
		return
	robot_position_calibration_status = "Saved robot position checkpoint restored."
	_maybe_auto_level_from_robot_registration()

func _update_robot_position_calibration_status() -> void:
	var module := _active_robot_module()
	if module == null or not module.has_method("get_calibration_status"):
		return
	var status: Dictionary = module.call("get_calibration_status")
	var next_status := "%s | frames=%d | confidence=%.0f%%" % [
		str(status.get("message", "Robot position calibration is idle.")),
		int(status.get("frames", 0)),
		float(status.get("confidence", 0.0)) * 100.0,
	]
	if next_status != robot_position_calibration_status:
		robot_position_calibration_status = next_status
		if Engine.is_editor_hint():
			notify_property_list_changed()
	if str(status.get("state", "")) == "complete":
		_maybe_auto_level_from_robot_registration()

func level_world_from_robot_base(persist: bool = true) -> bool:
	if not is_inside_tree():
		world_level_status = "World leveling is waiting for the unified scene to load."
		return false
	var anchor := _world_level_anchor(false)
	var overlay := get_node_or_null(ROBOT_OVERLAY_NODE) as Node3D
	if anchor == null or overlay == null:
		world_level_status = "World leveling needs WorldLevelAnchor and a registered robot overlay."
		return false
	var registration: Dictionary = {}
	if overlay.has_method("get_registration_status"):
		registration = overlay.call("get_registration_status")
	if not bool(registration.get("registered", false)):
		world_level_status = "Calibrate the robot position before leveling the merged point cloud."
		return false
	var current_up := (overlay.global_transform.basis.orthonormalized() * Vector3.UP).normalized()
	if not current_up.is_finite() or current_up.length_squared() < 0.99:
		world_level_status = "The saved robot registration has an invalid up direction."
		return false
	var tilt_radians := current_up.angle_to(Vector3.UP)
	var tilt_degrees := rad_to_deg(tilt_radians)
	if tilt_radians > 0.00001:
		var correction := Basis(Quaternion(current_up, Vector3.UP)).orthonormalized()
		var previous := anchor.global_transform
		var pivot := overlay.global_position
		var corrected_origin := pivot + correction * (previous.origin - pivot)
		anchor.global_transform = Transform3D(
			(correction * previous.basis).orthonormalized(),
			corrected_origin
		)
	var registration_token := float(registration.get("saved_unix_ms", 0.0))
	_last_world_level_registration_token = registration_token if registration_token > 0.0 else -1.0
	var remaining_degrees := rad_to_deg(
		(overlay.global_transform.basis.orthonormalized() * Vector3.UP).normalized().angle_to(Vector3.UP)
	)
	world_level_status = "Shared clouds and robot leveled by %.2f deg | remaining %.3f deg" % [
		tilt_degrees,
		remaining_degrees,
	]
	if persist:
		_save_world_level(anchor.transform, tilt_degrees)
	return true

func _maybe_auto_level_from_robot_registration() -> void:
	if not auto_level_from_robot_base or not is_inside_tree():
		return
	var overlay := get_node_or_null(ROBOT_OVERLAY_NODE)
	if overlay == null or not overlay.has_method("get_registration_status"):
		return
	var registration: Dictionary = overlay.call("get_registration_status")
	if not bool(registration.get("registered", false)):
		return
	var token := float(registration.get("saved_unix_ms", 0.0))
	if token > 0.0 and absf(token - _last_world_level_registration_token) < 0.5:
		return
	if token <= 0.0 and _last_world_level_registration_token < 0.0:
		return
	level_world_from_robot_base(true)

func _save_world_level(value: Transform3D, correction_degrees: float) -> bool:
	if _world_level_path.is_empty():
		_world_level_path = ProjectSettings.globalize_path(WORLD_LEVEL_PATH)
	var payload := _world_level_transform_to_dictionary(value)
	payload["type"] = "unified_world_level"
	payload["source"] = "robot_base_up"
	payload["correction_degrees"] = correction_degrees
	payload["registration_saved_unix_ms"] = _last_world_level_registration_token
	payload["saved_unix_ms"] = Time.get_unix_time_from_system() * 1000.0
	var file := FileAccess.open(_world_level_path, FileAccess.WRITE)
	if file == null:
		world_level_status += " | warning: could not save world level"
		return false
	file.store_string(JSON.stringify(payload, "\t"))
	return true

func _load_world_level() -> bool:
	if _world_level_path.is_empty():
		_world_level_path = ProjectSettings.globalize_path(WORLD_LEVEL_PATH)
	if not FileAccess.file_exists(_world_level_path):
		return false
	var file := FileAccess.open(_world_level_path, FileAccess.READ)
	if file == null:
		return false
	var parsed = JSON.parse_string(file.get_as_text())
	if not parsed is Dictionary:
		world_level_status = "Saved world-level file is invalid JSON."
		return false
	var loaded := _world_level_transform_from_dictionary(parsed as Dictionary)
	if not loaded.is_finite():
		world_level_status = "Saved world-level transform is invalid."
		return false
	var anchor := _world_level_anchor(false)
	if anchor == null:
		return false
	anchor.transform = Transform3D(loaded.basis.orthonormalized(), loaded.origin)
	var registration_token := float((parsed as Dictionary).get("registration_saved_unix_ms", 0.0))
	_last_world_level_registration_token = registration_token if registration_token > 0.0 else -1.0
	world_level_status = "Saved robot world level loaded | correction %.2f deg" % float(
		(parsed as Dictionary).get("correction_degrees", 0.0)
	)
	return true

func _clear_world_level(remove_saved: bool) -> void:
	var anchor := _world_level_anchor(false)
	if anchor != null:
		anchor.transform = Transform3D.IDENTITY
	_last_world_level_registration_token = 0.0
	if remove_saved:
		if _world_level_path.is_empty():
			_world_level_path = ProjectSettings.globalize_path(WORLD_LEVEL_PATH)
		if FileAccess.file_exists(_world_level_path):
			DirAccess.remove_absolute(_world_level_path)
	world_level_status = "World-level correction cleared; camera-to-camera alignment was preserved."

func _world_level_transform_to_dictionary(value: Transform3D) -> Dictionary:
	return {
		"basis_x": [value.basis.x.x, value.basis.x.y, value.basis.x.z],
		"basis_y": [value.basis.y.x, value.basis.y.y, value.basis.y.z],
		"basis_z": [value.basis.z.x, value.basis.z.y, value.basis.z.z],
		"origin": [value.origin.x, value.origin.y, value.origin.z],
	}

func _world_level_transform_from_dictionary(payload: Dictionary) -> Transform3D:
	for key in ["basis_x", "basis_y", "basis_z", "origin"]:
		if not payload.get(key, null) is Array or (payload[key] as Array).size() < 3:
			return Transform3D(Basis(Vector3.INF, Vector3.INF, Vector3.INF), Vector3.INF)
	var bx: Array = payload["basis_x"]
	var by: Array = payload["basis_y"]
	var bz: Array = payload["basis_z"]
	var origin_values: Array = payload["origin"]
	return Transform3D(
		Basis(
			Vector3(float(bx[0]), float(bx[1]), float(bx[2])),
			Vector3(float(by[0]), float(by[1]), float(by[2])),
			Vector3(float(bz[0]), float(bz[1]), float(bz[2]))
		),
		Vector3(float(origin_values[0]), float(origin_values[1]), float(origin_values[2]))
	)

func _apply_clean_defaults() -> void:
	render_mode = "independent_mesh"
	point_pixel_size = 2.5
	cleanup_enabled = false
	cleanup_depth_delta_m = 0.055
	cleanup_min_close_neighbors = 2.0
	edge_feather_enabled = true
	mesh_max_depth_delta_m = REALSENSE_VIEWER_MESH_DEPTH_DELTA
	mesh_max_edge_m = REALSENSE_VIEWER_MESH_EDGE_M
	texture_map_mesh = true
	realsense_depth_source = "sdk_depth"
	realsense_stream_profile = "fast60"
	realsense_stride = 1
	oakd_stride = 1
	realsense_depth_filters_enabled = false
	realsense_decimation_filter_enabled = true
	realsense_decimation_magnitude = 2
	realsense_rotation_filter_enabled = false
	realsense_hdr_merge_filter_enabled = true
	realsense_sequence_id_filter_enabled = false
	realsense_threshold_filter_enabled = false
	realsense_depth_to_disparity_filter_enabled = true
	realsense_spatial_filter_enabled = true
	realsense_temporal_filter_enabled = true
	realsense_hole_filling_filter_enabled = false
	realsense_disparity_to_depth_filter_enabled = true
	realsense_filters_for_geometry = false
	realsense_stabilization_enabled = false
	realsense_stabilization_deadband_m = 0.0
	realsense_stabilization_hold_frames = 0
	realsense_fast_backend = "onnx_cuda"
	realsense_fast_profile = "fast_192x384_i2"
	realsense_fast_iters = 1
	realsense_fast_scale = 0.25
	oakd_capture_preset = "30fps_low_latency"
	oakd_depth_source = "fast_foundation"
	oakd_color_enabled = false
	oakd_color_mode = "rgb_projected_stable"
	oakd_render_depth_bias_m = 0.004
	oakd_geometry_edge_guard_m = 0.06
	oakd_border_crop_px = 12
	oakd_stabilization_enabled = true
	oakd_stabilization_deadband_m = 0.018
	oakd_stabilization_hold_frames = 1
	oakd_fast_backend = "onnx_cuda"
	oakd_fast_profile = "rt_256x512_i2"
	oakd_fast_iters = 4
	oakd_fast_scale = 0.5
	_send_all_stream_commands()
	_update_camera_renderers()

func _apply_close_robot_rig_defaults() -> void:
	render_mode = "independent_mesh"
	min_depth_m = 0.18
	max_depth_m = 1.5
	for camera_id in _realsense_camera_ids():
		var serial := _realsense_serial(camera_id)
		if serial.is_empty():
			continue
		var settings := _realsense_settings(camera_id).duplicate(true)
		var model := str(settings.get("model", _realsense_setting(camera_id, "model", ""))).to_lower()
		settings["enabled"] = true
		settings["depth_source"] = "sdk_depth"
		settings["stride"] = 1
		settings["color_enabled"] = true
		settings["depth_filters_enabled"] = false
		settings["decimation_filter_enabled"] = false
		settings["filters_for_geometry"] = false
		settings["use_custom_depth_range"] = true
		if model.contains("435"):
			settings["stream_profile"] = "highres30"
			settings["min_depth_m"] = 0.18
			settings["max_depth_m"] = 1.2
			settings["emitter_enabled"] = true
			settings["laser_power_percent"] = 100.0
			settings["workspace_crop_enabled"] = true
			settings["crop_left_percent"] = 11.0
			settings["crop_right_percent"] = 11.0
			settings["crop_top_percent"] = 14.0
			settings["crop_bottom_percent"] = 14.0
		elif model.contains("455"):
			settings["stream_profile"] = "fast60"
			settings["min_depth_m"] = 0.30
			settings["max_depth_m"] = 1.5
			settings["emitter_enabled"] = false
			settings["laser_power_percent"] = 70.0
			settings["workspace_crop_enabled"] = false
		realsense_device_registry[serial] = settings
		var anchor := _find_realsense_camera_anchor_existing(camera_id)
		if anchor != null and anchor.has_method("apply_settings"):
			anchor.call("apply_settings", settings)
	camera_diagnostic_view = "all"
	diagnostic_depth_only = false
	_restart_realsense_direct_renderers()
	_update_camera_renderers()
	_send_all_stream_commands()

func _refresh_realsense_devices(force: bool) -> void:
	if not realsense_auto_discover:
		_realsense_devices.clear()
		_realsense_camera_order.clear()
		_realsense_labels.clear()
		realsense_devices_status = "RealSense auto-discovery is disabled."
		return
	var now_msec := Time.get_ticks_msec()
	if not force and now_msec - _last_realsense_scan_msec < REALSENSE_SCAN_INTERVAL_MSEC:
		return
	_last_realsense_scan_msec = now_msec
	var discovered := _query_realsense_devices()
	var next_devices: Dictionary = {}
	var next_order: Array[String] = []
	for device in discovered:
		if not (device is Dictionary):
			continue
		var info := device as Dictionary
		var serial := str(info.get("serial", "")).strip_edges()
		if serial.is_empty():
			continue
		var camera_id := _realsense_camera_id(serial)
		_ensure_realsense_registry_entry(serial, info)
		next_devices[camera_id] = info
		next_order.append(camera_id)
	next_order.sort()
	var order_changed := next_order != _realsense_camera_order
	_realsense_devices = next_devices
	_realsense_camera_order = next_order
	_refresh_realsense_labels()
	_update_realsense_devices_status(discovered)
	if order_changed:
		_poll_camera_alignment_registry(true)
		notify_property_list_changed()
		_update_camera_renderers()
		_update_debug_panel(true)

func _query_realsense_devices() -> Array:
	if not ClassDB.class_exists("RealSenseDirectFrameSource"):
		return []
	if _realsense_scan_helper == null or not is_instance_valid(_realsense_scan_helper):
		_realsense_scan_helper = ClassDB.instantiate("RealSenseDirectFrameSource")
	if _realsense_scan_helper == null or not _realsense_scan_helper.has_method("list_connected_devices"):
		return []
	var devices = _realsense_scan_helper.call("list_connected_devices")
	return devices if devices is Array else []

func _ensure_realsense_registry_entry(serial: String, info: Dictionary) -> void:
	var settings: Dictionary = {}
	if realsense_device_registry.has(serial) and realsense_device_registry[serial] is Dictionary:
		settings = (realsense_device_registry[serial] as Dictionary).duplicate(true)
	settings["serial"] = serial
	settings["name"] = str(info.get("name", settings.get("name", "")))
	settings["product_id"] = str(info.get("product_id", settings.get("product_id", "")))
	settings["firmware"] = str(info.get("firmware", settings.get("firmware", "")))
	settings["physical_port"] = str(info.get("physical_port", settings.get("physical_port", "")))
	settings["model"] = _realsense_model_name(settings)
	_realsense_default_setting(settings, "enabled", realsense_enabled)
	_realsense_default_setting(settings, "depth_source", realsense_depth_source)
	_realsense_default_setting(settings, "stream_profile", realsense_stream_profile)
	_realsense_default_setting(settings, "stride", realsense_stride)
	_realsense_default_setting(settings, "color_enabled", realsense_color_enabled)
	var model_lower := str(settings.get("model", "")).to_lower()
	_realsense_default_setting(settings, "emitter_enabled", not model_lower.contains("455"))
	_realsense_default_setting(settings, "laser_power_percent", 100.0)
	_realsense_default_setting(settings, "workspace_crop_enabled", false)
	_realsense_default_setting(settings, "crop_left_percent", 0.0)
	_realsense_default_setting(settings, "crop_right_percent", 0.0)
	_realsense_default_setting(settings, "crop_top_percent", 0.0)
	_realsense_default_setting(settings, "crop_bottom_percent", 0.0)
	if not settings.has("max_depth_enabled"):
		settings["max_depth_enabled"] = bool(settings.get("use_custom_depth_range", false))
	_realsense_default_setting(settings, "use_custom_depth_range", false)
	_realsense_default_setting(settings, "min_depth_m", min_depth_m)
	_realsense_default_setting(settings, "max_depth_m", max_depth_m)
	_realsense_default_setting(settings, "depth_filters_enabled", realsense_depth_filters_enabled)
	_realsense_default_setting(settings, "decimation_filter_enabled", realsense_decimation_filter_enabled)
	_realsense_default_setting(settings, "decimation_magnitude", realsense_decimation_magnitude)
	_realsense_default_setting(settings, "rotation_filter_enabled", realsense_rotation_filter_enabled)
	_realsense_default_setting(settings, "hdr_merge_filter_enabled", realsense_hdr_merge_filter_enabled)
	_realsense_default_setting(settings, "sequence_id_filter_enabled", realsense_sequence_id_filter_enabled)
	_realsense_default_setting(settings, "threshold_filter_enabled", realsense_threshold_filter_enabled)
	_realsense_default_setting(settings, "depth_to_disparity_filter_enabled", realsense_depth_to_disparity_filter_enabled)
	_realsense_default_setting(settings, "spatial_filter_enabled", realsense_spatial_filter_enabled)
	_realsense_default_setting(settings, "temporal_filter_enabled", realsense_temporal_filter_enabled)
	_realsense_default_setting(settings, "hole_filling_filter_enabled", realsense_hole_filling_filter_enabled)
	_realsense_default_setting(settings, "disparity_to_depth_filter_enabled", realsense_disparity_to_depth_filter_enabled)
	_realsense_default_setting(settings, "filters_for_geometry", realsense_filters_for_geometry)
	_realsense_default_setting(settings, "geometry_edge_guard_m", realsense_geometry_edge_guard_m)
	_realsense_default_setting(settings, "hole_filling", realsense_hole_filling)
	_realsense_default_setting(settings, "stabilization_enabled", realsense_stabilization_enabled)
	_realsense_default_setting(settings, "stabilization_deadband_m", realsense_stabilization_deadband_m)
	_realsense_default_setting(settings, "stabilization_hold_frames", realsense_stabilization_hold_frames)
	_realsense_default_setting(settings, "fast_backend", realsense_fast_backend)
	_realsense_default_setting(settings, "fast_profile", realsense_fast_profile)
	_realsense_default_setting(settings, "fast_iters", realsense_fast_iters)
	_realsense_default_setting(settings, "fast_scale", realsense_fast_scale)
	realsense_device_registry[serial] = settings

func _realsense_default_setting(settings: Dictionary, key: String, value) -> void:
	if not settings.has(key):
		settings[key] = value

func _realsense_model_name(settings: Dictionary) -> String:
	var name := str(settings.get("name", ""))
	var upper_name := name.to_upper()
	var product_id := str(settings.get("product_id", "")).to_upper()
	if product_id == "0B5C" or upper_name.find("455") >= 0:
		return "D455"
	if upper_name.find("435I") >= 0:
		return "D435i"
	if product_id == "0B07" or upper_name.find("435") >= 0:
		return "D435"
	if upper_name.find("415") >= 0:
		return "D415"
	if upper_name.find("405") >= 0:
		return "D405"
	var compact := name.replace("Intel(R)", "").replace("RealSense(TM)", "").replace("Depth Camera", "").replace("with RGB Module", "").strip_edges()
	return compact if not compact.is_empty() else "Camera"

func _refresh_realsense_labels() -> void:
	_realsense_labels.clear()
	var groups: Dictionary = {}
	for camera_id in _realsense_camera_order:
		var settings := _realsense_settings(camera_id)
		var base_label := "RealSense %s" % str(settings.get("model", "Camera"))
		if not groups.has(base_label):
			groups[base_label] = []
		(groups[base_label] as Array).append(camera_id)
	for base_label in groups.keys():
		var ids := groups[base_label] as Array
		ids.sort()
		for i in ids.size():
			var suffix := ""
			if ids.size() > 1:
				suffix = " %s" % REALSENSE_DUPLICATE_SUFFIXES[mini(i, REALSENSE_DUPLICATE_SUFFIXES.size() - 1)]
			_realsense_labels[str(ids[i])] = "%s%s" % [base_label, suffix]

func _update_realsense_devices_status(raw_devices: Array) -> void:
	if _realsense_camera_order.is_empty():
		realsense_devices_status = "No RealSense devices discovered by librealsense."
		realsense_status = realsense_devices_status
		return
	var lines: Array[String] = []
	for camera_id in _realsense_camera_order:
		var settings := _realsense_settings(camera_id)
		lines.append("%s | serial=%s | firmware=%s" % [
			_camera_label(camera_id),
			str(settings.get("serial", _realsense_serial(camera_id))),
			str(settings.get("firmware", "")),
		])
	realsense_devices_status = "\n".join(lines)

func _realsense_camera_id(serial: String) -> String:
	return "%s%s" % [CAMERA_REALSENSE_PREFIX, serial]

func _is_realsense_camera(camera_id: String) -> bool:
	return camera_id == CAMERA_REALSENSE or camera_id.begins_with(CAMERA_REALSENSE_PREFIX)

func _realsense_serial(camera_id: String) -> String:
	if camera_id.begins_with(CAMERA_REALSENSE_PREFIX):
		return camera_id.substr(CAMERA_REALSENSE_PREFIX.length())
	return ""

func _realsense_camera_ids() -> Array[String]:
	var ids := _realsense_camera_order.duplicate()
	var serials: Array[String] = []
	for key in realsense_device_registry.keys():
		var serial := str(key).strip_edges()
		if not serial.is_empty():
			serials.append(serial)
	serials.sort()
	for serial in serials:
		var camera_id := _realsense_camera_id(serial)
		if camera_id not in ids:
			ids.append(camera_id)
	return ids

func _known_camera_ids() -> Array[String]:
	var ids: Array[String] = []
	for camera_id in _realsense_camera_order:
		ids.append(camera_id)
	for camera_id in _camera_nodes.keys():
		if camera_id is String and camera_id not in ids:
			ids.append(camera_id)
	if CAMERA_OAKD not in ids:
		ids.append(CAMERA_OAKD)
	return ids

func _debug_camera_ids() -> Array[String]:
	var ids := _known_camera_ids()
	if CAMERA_OAKD in ids and not oakd_enabled and not _camera_nodes.has(CAMERA_OAKD):
		ids.erase(CAMERA_OAKD)
	return ids

func _realsense_settings(camera_id: String) -> Dictionary:
	var serial := _realsense_serial(camera_id)
	if serial.is_empty() or not realsense_device_registry.has(serial) or not (realsense_device_registry[serial] is Dictionary):
		return {}
	return realsense_device_registry[serial] as Dictionary

func _realsense_setting(camera_id: String, key: String, fallback):
	var node_settings := _realsense_camera_node_settings(camera_id)
	if node_settings.has(key):
		return node_settings.get(key, fallback)
	var settings := _realsense_settings(camera_id)
	return settings.get(key, fallback)

func _realsense_camera_node_settings(camera_id: String) -> Dictionary:
	var anchor := _find_realsense_camera_anchor_existing(camera_id)
	if anchor != null and anchor.has_method("get_settings"):
		var settings = anchor.call("get_settings")
		if settings is Dictionary:
			return settings
	return {}

func _find_realsense_camera_anchor_existing(camera_id: String) -> Node3D:
	var clouds := _camera_clouds_node(false)
	if clouds == null:
		return null
	var anchor := clouds.get_node_or_null(_camera_anchor_name(camera_id)) as Node3D
	if anchor != null:
		return anchor
	for child in clouds.get_children():
		if child is Node3D and child.has_method("get_settings"):
			var settings = child.call("get_settings")
			if settings is Dictionary and str((settings as Dictionary).get("serial", "")) == _realsense_serial(camera_id):
				return child as Node3D
	return clouds.get_node_or_null("RealSense%sCloudAnchor" % _realsense_serial(camera_id)) as Node3D

func _set_realsense_setting(camera_id: String, key: String, value, refresh_renderers: bool = true) -> void:
	var serial := _realsense_serial(camera_id)
	if serial.is_empty():
		return
	var anchor := _camera_anchor(camera_id, false)
	if anchor != null and key == "enabled":
		anchor.set(key, value)
	var settings := _realsense_settings(camera_id).duplicate(true)
	settings[key] = value
	realsense_device_registry[serial] = settings
	if refresh_renderers:
		_update_camera_renderers()

func _get_property_list() -> Array[Dictionary]:
	var properties: Array[Dictionary] = []
	var realsense_ids := _realsense_camera_ids()
	if realsense_ids.is_empty():
		return properties
	properties.append({
		"name": "Registered RealSense Cameras",
		"type": TYPE_NIL,
		"usage": PROPERTY_USAGE_GROUP,
	})
	for camera_id in realsense_ids:
		var serial := _realsense_serial(camera_id)
		var prefix := "realsense/%s/" % serial
		properties.append({
			"name": "%slabel" % prefix,
			"type": TYPE_STRING,
			"usage": PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_READ_ONLY,
		})
		properties.append({
			"name": "%senabled" % prefix,
			"type": TYPE_BOOL,
			"usage": PROPERTY_USAGE_DEFAULT,
		})
	return properties

func _get(property: StringName):
	var parts := _realsense_dynamic_property_parts(str(property))
	if parts.is_empty():
		return null
	var camera_id := _realsense_camera_id(str(parts["serial"]))
	var key := str(parts["key"])
	if key == "label":
		return _camera_label(camera_id)
	return _realsense_setting(camera_id, key, null)

func _set(property: StringName, value) -> bool:
	var parts := _realsense_dynamic_property_parts(str(property))
	if parts.is_empty():
		return false
	var key := str(parts["key"])
	if key == "label":
		return true
	var camera_id := _realsense_camera_id(str(parts["serial"]))
	match key:
		"enabled":
			value = bool(value)
		_:
			return false
	_set_realsense_setting(camera_id, key, value)
	_send_camera_stream_command(camera_id, _streams_enabled() and _camera_enabled(camera_id))
	return true

func _realsense_dynamic_property_parts(property: String) -> Dictionary:
	if not property.begins_with("realsense/"):
		return {}
	var parts := property.split("/")
	if parts.size() != 3:
		return {}
	var serial := str(parts[1]).strip_edges()
	var key := str(parts[2]).strip_edges()
	if serial.is_empty() or key.is_empty():
		return {}
	return {
		"serial": serial,
		"key": key,
	}

func _camera_enabled(camera_id: String) -> bool:
	if _is_realsense_camera(camera_id):
		return bool(_realsense_setting(camera_id, "enabled", realsense_enabled))
	if camera_id == CAMERA_OAKD:
		return oakd_enabled
	return false

func _editor_live_stream_allowed() -> bool:
	if not Engine.is_editor_hint():
		return true
	if _runtime_stream_owner_blocks_editor():
		return false
	if not is_inside_tree() or get_tree() == null:
		return false
	var edited_root := get_tree().edited_scene_root
	if edited_root == self:
		return true
	return _is_nested_editor_preview_under_view_switcher(edited_root)

func _editor_live_stream_block_reason() -> String:
	if Engine.is_editor_hint() and _runtime_stream_owner_blocks_editor():
		return "Editor live streaming is paused because the running game owns the Godot RealSense stream."
	return "Editor live streaming is suppressed while Unified Point Clouds is nested inside another edited scene. Open the Unified Point Clouds scene directly to preview live cameras."

func _is_nested_editor_preview_under_view_switcher(edited_root: Node) -> bool:
	if edited_root == null:
		return false
	var node := get_parent()
	while node != null:
		if _node_is_view_switcher(node):
			return node == edited_root or edited_root.is_ancestor_of(node)
		if node == edited_root:
			break
		node = node.get_parent()
	return false

func _node_is_view_switcher(node: Node) -> bool:
	if node == null:
		return false
	var script := node.get_script() as Script
	return script != null and script.resource_path == "res://view_switcher.gd"

func _streams_enabled() -> bool:
	return editor_stream_enabled and _editor_live_stream_allowed()

func _update_runtime_stream_owner_state() -> void:
	if Engine.is_editor_hint():
		var was_blocked := _runtime_stream_owner_blocks_editor_cached
		_update_runtime_stream_owner_cache(false)
		if _runtime_stream_owner_blocks_editor_cached == was_blocked:
			return
		_update_camera_renderers()
		if _runtime_stream_owner_blocks_editor_cached:
			_send_all_stream_commands(false)
			var blocked_status := _editor_live_stream_block_reason()
			realsense_status = blocked_status
			oakd_status = blocked_status
		elif _streams_enabled():
			_send_all_stream_commands()
		return

	_write_runtime_stream_owner(false)

func _runtime_stream_owner_blocks_editor() -> bool:
	if not Engine.is_editor_hint():
		return false
	_update_runtime_stream_owner_cache(false)
	return _runtime_stream_owner_blocks_editor_cached

func _update_runtime_stream_owner_cache(force: bool) -> void:
	if not Engine.is_editor_hint():
		_runtime_stream_owner_blocks_editor_cached = false
		return
	var now_msec := Time.get_ticks_msec()
	if not force and now_msec - _last_runtime_owner_check_msec < RUNTIME_STREAM_OWNER_CHECK_INTERVAL_MSEC:
		return
	_last_runtime_owner_check_msec = now_msec
	_runtime_stream_owner_blocks_editor_cached = _read_runtime_stream_owner_active()

func _read_runtime_stream_owner_active() -> bool:
	if _runtime_stream_owner_path.is_empty():
		_runtime_stream_owner_path = ProjectSettings.globalize_path(RUNTIME_STREAM_OWNER_PATH)
	if _runtime_stream_owner_path.is_empty() or not FileAccess.file_exists(_runtime_stream_owner_path):
		return false
	var file := FileAccess.open(_runtime_stream_owner_path, FileAccess.READ)
	if file == null:
		return false
	var parsed = JSON.parse_string(file.get_as_text())
	if not (parsed is Dictionary):
		return false
	var owner := parsed as Dictionary
	if str(owner.get("owner", "")) != "runtime":
		return false
	var timestamp_msec := int(owner.get("timestamp_msec", 0))
	var now_unix_msec := int(Time.get_unix_time_from_system() * 1000.0)
	return timestamp_msec > 0 and now_unix_msec - timestamp_msec <= RUNTIME_STREAM_OWNER_TIMEOUT_MSEC

func _write_runtime_stream_owner(force: bool) -> void:
	if Engine.is_editor_hint():
		return
	var now_msec := Time.get_ticks_msec()
	if not force and now_msec - _last_runtime_owner_write_msec < RUNTIME_STREAM_OWNER_WRITE_INTERVAL_MSEC:
		return
	_last_runtime_owner_write_msec = now_msec
	if _runtime_stream_owner_path.is_empty():
		_runtime_stream_owner_path = ProjectSettings.globalize_path(RUNTIME_STREAM_OWNER_PATH)
	var file := FileAccess.open(_runtime_stream_owner_path, FileAccess.WRITE)
	if file == null:
		return
	file.store_string(JSON.stringify({
		"owner": "runtime",
		"pid": OS.get_process_id(),
		"timestamp_msec": int(Time.get_unix_time_from_system() * 1000.0),
		"scene": scene_file_path,
	}))

func _clear_runtime_stream_owner() -> void:
	if _runtime_stream_owner_path.is_empty():
		_runtime_stream_owner_path = ProjectSettings.globalize_path(RUNTIME_STREAM_OWNER_PATH)
	if _runtime_stream_owner_path.is_empty() or not FileAccess.file_exists(_runtime_stream_owner_path):
		return
	var file := FileAccess.open(_runtime_stream_owner_path, FileAccess.READ)
	if file != null:
		var parsed = JSON.parse_string(file.get_as_text())
		if parsed is Dictionary and int((parsed as Dictionary).get("pid", -1)) != OS.get_process_id():
			return
	DirAccess.remove_absolute(_runtime_stream_owner_path)

func _camera_stride(camera_id: String) -> int:
	if _is_realsense_camera(camera_id):
		return maxi(1, int(_realsense_setting(camera_id, "stride", realsense_stride)))
	if camera_id == CAMERA_OAKD:
		return maxi(1, oakd_stride)
	return 1

func _camera_color_enabled(camera_id: String) -> bool:
	if diagnostic_depth_only:
		return false
	if _is_realsense_camera(camera_id):
		return bool(_realsense_setting(camera_id, "color_enabled", realsense_color_enabled))
	if camera_id == CAMERA_OAKD:
		return oakd_color_enabled
	return true

func _camera_diagnostic_visible(camera_id: String) -> bool:
	if camera_diagnostic_view == "all" or not _is_realsense_camera(camera_id):
		return true
	var model := str(_realsense_setting(camera_id, "model", "")).to_lower()
	if camera_diagnostic_view == "d455_only":
		return model.contains("455")
	if camera_diagnostic_view == "d435_only":
		return model.contains("435")
	return true

func _value_enabled(value) -> bool:
	if value is bool:
		return value
	if value is int or value is float:
		return float(value) != 0.0
	if value is String:
		var text := (value as String).strip_edges().to_lower()
		return text in ["true", "on", "yes", "1"]
	return false

func _camera_uses_custom_depth_range(camera_id: String) -> bool:
	if not _is_realsense_camera(camera_id):
		return false
	if _value_enabled(_realsense_setting(camera_id, "max_depth_enabled", false)):
		return true
	if _value_enabled(_realsense_setting(camera_id, "use_custom_depth_range", false)):
		return true
	var camera_max := float(_realsense_setting(camera_id, "max_depth_m", max_depth_m))
	return not is_equal_approx(camera_max, max_depth_m)

func _camera_min_depth(camera_id: String) -> float:
	return min_depth_m

func _camera_max_depth(camera_id: String) -> float:
	if _camera_uses_custom_depth_range(camera_id):
		return maxf(float(_realsense_setting(camera_id, "max_depth_m", max_depth_m)), 0.25)
	return max_depth_m

func _camera_label(camera_id: String) -> String:
	if camera_id.begins_with(CAMERA_REALSENSE_PREFIX):
		if _realsense_labels.has(camera_id):
			return str(_realsense_labels[camera_id])
		var settings := _realsense_settings(camera_id)
		var model := str(settings.get("model", "")).strip_edges()
		if not model.is_empty():
			return "RealSense %s" % model
		var saved_name := str(settings.get("name", "")).strip_edges()
		if not saved_name.is_empty():
			return saved_name.replace("Intel ", "")
		return "RealSense %s" % _realsense_serial(camera_id)
	if camera_id == CAMERA_REALSENSE:
		return "RealSense"
	if camera_id == CAMERA_OAKD:
		return "OAK-D"
	return camera_id

func _camera_stats_key(camera_id: String) -> String:
	return camera_id

func _camera_shm_name(camera_id: String) -> String:
	if camera_id.begins_with(CAMERA_REALSENSE_PREFIX):
		return "realsense_%s_point_cloud_grid" % _realsense_serial(camera_id)
	if camera_id == CAMERA_REALSENSE:
		return "realsense_point_cloud_grid"
	if camera_id == CAMERA_OAKD:
		return "oakd_point_cloud_grid"
	return "%s_point_cloud_grid" % camera_id

func _camera_node_name(camera_id: String) -> String:
	if camera_id.begins_with(CAMERA_REALSENSE_PREFIX):
		return "PointCloudRenderer"
	var safe_id := camera_id.replace(":", "_").replace("-", "_").replace(" ", "_")
	return "%sUnifiedPointCloud" % safe_id.to_pascal_case()

func _camera_anchor_name(camera_id: String) -> String:
	if camera_id.begins_with(CAMERA_REALSENSE_PREFIX):
		return "%s %s" % [_camera_label(camera_id), _realsense_serial(camera_id)]
	if camera_id == CAMERA_REALSENSE:
		return "RealSenseCloudAnchor"
	if camera_id == CAMERA_OAKD:
		return "OAKDCloudAnchor"
	return "%sCloudAnchor" % _camera_label(camera_id).replace("-", "").replace(" ", "")

func _camera_transform(camera_id: String) -> Transform3D:
	var camera_transform: Transform3D = _alignment_transforms.get(camera_id, Transform3D.IDENTITY)
	if camera_id == CAMERA_OAKD and absf(oakd_render_depth_bias_m) > 0.000001:
		camera_transform = camera_transform * Transform3D(Basis.IDENTITY, Vector3(0.0, 0.0, oakd_render_depth_bias_m))
	return camera_transform

func _mesh_enabled() -> bool:
	return render_mode in ["unified_mesh", "shader_mesh", "prefer_d455", "prefer_d435", "independent_mesh", "cpu_mesh"]

func _gpu_mesh_enabled() -> bool:
	return render_mode in ["unified_mesh", "shader_mesh", "prefer_d455", "prefer_d435", "independent_mesh"]

func _gpu_mesh_compute_enabled() -> bool:
	return false

func _gpu_mesh_static_shader_enabled() -> bool:
	return render_mode in ["unified_mesh", "shader_mesh", "prefer_d455", "prefer_d435", "independent_mesh"]

func _tracker_mesh_mode() -> String:
	if render_mode in ["unified_mesh", "shader_mesh", "prefer_d455", "prefer_d435", "independent_mesh"]:
		return "stereo_gpu"
	if render_mode == "cpu_mesh":
		return "stereo_cpu"
	return "gpu_points"

func _direct_realsense_filter_config(camera_id: String = CAMERA_REALSENSE) -> Dictionary:
	var depth_filters_enabled := bool(_realsense_setting(camera_id, "depth_filters_enabled", realsense_depth_filters_enabled))
	var filters_for_geometry := bool(_realsense_setting(camera_id, "filters_for_geometry", realsense_filters_for_geometry))
	var native_filters_enabled := depth_filters_enabled or filters_for_geometry
	return {
		"post_processing_enabled": native_filters_enabled,
		"decimation_filter_enabled": bool(_realsense_setting(camera_id, "decimation_filter_enabled", realsense_decimation_filter_enabled)),
		"decimation_magnitude": int(_realsense_setting(camera_id, "decimation_magnitude", realsense_decimation_magnitude)),
		"rotation_filter_enabled": bool(_realsense_setting(camera_id, "rotation_filter_enabled", realsense_rotation_filter_enabled)),
		"hdr_merge_filter_enabled": bool(_realsense_setting(camera_id, "hdr_merge_filter_enabled", realsense_hdr_merge_filter_enabled)),
		"sequence_id_filter_enabled": bool(_realsense_setting(camera_id, "sequence_id_filter_enabled", realsense_sequence_id_filter_enabled)),
		"threshold_filter_enabled": bool(_realsense_setting(camera_id, "threshold_filter_enabled", realsense_threshold_filter_enabled)),
		"depth_to_disparity_filter_enabled": bool(_realsense_setting(camera_id, "depth_to_disparity_filter_enabled", realsense_depth_to_disparity_filter_enabled)),
		"spatial_filter_enabled": bool(_realsense_setting(camera_id, "spatial_filter_enabled", realsense_spatial_filter_enabled)),
		"temporal_filter_enabled": bool(_realsense_setting(camera_id, "temporal_filter_enabled", realsense_temporal_filter_enabled)),
		"hole_filling_filter_enabled": bool(_realsense_setting(camera_id, "hole_filling_filter_enabled", realsense_hole_filling_filter_enabled)),
		"disparity_to_depth_filter_enabled": bool(_realsense_setting(camera_id, "disparity_to_depth_filter_enabled", realsense_disparity_to_depth_filter_enabled)),
		"hole_filling_mode": int(_realsense_setting(camera_id, "hole_filling", realsense_hole_filling)),
	}

func _realsense_fast_foundation_native_supported() -> bool:
	return OS.get_name() in ["Windows", "Linux", "FreeBSD", "NetBSD", "OpenBSD", "BSD"]

func _native_realsense_depth_source(requested_source: String) -> String:
	if requested_source == "fast_foundation_native" and _realsense_fast_foundation_native_supported():
		return "fast_foundation_native"
	return "sdk_depth"

func _effective_oakd_color_mode() -> String:
	return oakd_color_mode if oakd_color_enabled else "gray"

func _oakd_capture_settings() -> Dictionary:
	if oakd_capture_preset == "60fps_stable":
		return {
			"width": 640,
			"height": 400,
			"fps": 60.0,
			"rgb_res": "720p",
			"mono_res": "400p",
		}
	if oakd_capture_preset == "30fps_quality":
		return {
			"width": 1280,
			"height": 800,
			"fps": 30.0,
			"rgb_res": "1080p",
			"mono_res": "800p",
		}
	return {
		"width": 640,
		"height": 360,
		"fps": 30.0,
		"rgb_res": "1080p",
		"mono_res": "400p",
	}

func _effective_point_pixel_size() -> float:
	return point_pixel_size * 3.0 if render_mode == "point_splats" else point_pixel_size

func _camera_point_pixel_size(camera_id: String) -> float:
	var base := _effective_point_pixel_size()
	if not _is_realsense_camera(camera_id):
		return base
	# Keep roughly constant physical point coverage across profiles. Without this,
	# highres30 has more samples but each sample occupies the old large screen pixel.
	var profile := str(_realsense_setting(camera_id, "stream_profile", realsense_stream_profile))
	return base * (848.0 / 1280.0) if profile == "highres30" else base

func _send_udp(payload: Dictionary) -> void:
	if OS.has_feature("web"):
		return
	_command_udp.set_dest_address("127.0.0.1", tracker_control_port)
	_command_udp.put_packet(JSON.stringify(payload).to_utf8_buffer())

func _send_all_stream_commands(force_enabled = null) -> void:
	var active := _streams_enabled() if force_enabled == null else bool(force_enabled)
	if not active and not _stream_commands_active:
		return
	for camera_id in _known_camera_ids():
		_send_camera_stream_command(camera_id, active and _camera_enabled(camera_id))
	_stream_commands_active = active

func _send_camera_stream_command(camera_id: String, enabled: bool) -> void:
	if not enabled and not _stream_commands_active and not _streams_enabled():
		return
	if not is_inside_tree() and _point_cloud_stats_path.is_empty():
		return
	if _point_cloud_stats_path.is_empty():
		_point_cloud_stats_path = ProjectSettings.globalize_path("user://point_cloud_stream_stats.json")
	var payload := {
		"enabled": enabled,
		"stats_path": _point_cloud_stats_path,
		"console_stats": false,
		"sync_to_slowest": sync_fps_to_slowest,
		"stride": _camera_stride(camera_id),
		"min_depth": min_depth_m,
		"max_depth": max_depth_m,
		"transport": "shm",
		"shm_color_format": "bgr",
	}
	if camera_id.begins_with(CAMERA_REALSENSE_PREFIX):
		return
	if camera_id == CAMERA_REALSENSE:
		payload.merge({
			"type": "realsense_point_cloud",
			"depth_source": realsense_depth_source,
			"stream_profile": realsense_stream_profile,
			"max_points": 0,
			"mesh_enabled": _mesh_enabled(),
			"mesh_mode": _tracker_mesh_mode(),
			"mesh_max_edge": mesh_max_edge_m,
			"rs_depth_filters_enabled": realsense_depth_filters_enabled,
			"rs_filters_for_point_cloud_geometry": realsense_filters_for_geometry,
			"rs_filter_geometry_edge_guard_m": realsense_geometry_edge_guard_m,
			"rs_disparity_filters_enabled": true,
			"rs_hole_filling": realsense_hole_filling,
			"rs_stabilization_enabled": realsense_stabilization_enabled,
			"rs_stabilization_deadband_m": realsense_stabilization_deadband_m,
			"rs_stabilization_hold_frames": realsense_stabilization_hold_frames,
			"rs_fast_stereo_backend": realsense_fast_backend,
			"rs_fast_stereo_model_profile": realsense_fast_profile,
			"rs_fast_stereo_iters": realsense_fast_iters,
			"rs_fast_stereo_scale": realsense_fast_scale,
			"rs_fast_stereo_torch_compile": false,
		}, true)
		if _realsense_direct_capture_active():
			payload["enabled"] = false
			_send_udp(payload)
			return
	elif camera_id == CAMERA_OAKD:
		var oakd_capture := _oakd_capture_settings()
		payload.merge({
			"type": "oakd_point_cloud",
			"oakd_width": int(oakd_capture["width"]),
			"oakd_height": int(oakd_capture["height"]),
			"oakd_fps": float(oakd_capture["fps"]),
			"oakd_rgb_res": str(oakd_capture["rgb_res"]),
			"oakd_mono_res": str(oakd_capture["mono_res"]),
			"oakd_stereo_preset": "fast_density",
			"oakd_lr_check": true,
			"oakd_subpixel": false,
			"oakd_subpixel_bits": 3,
			"oakd_confidence_threshold": 160,
			"oakd_median_filter": "off",
			"oakd_speckle_filter": false,
			"oakd_speckle_range": 0,
			"oakd_depth_source": oakd_depth_source,
			"oakd_use_rgb_color_for_host_depth": oakd_color_enabled,
			"oakd_host_depth_color_mode": _effective_oakd_color_mode(),
			"oakd_geometry_edge_guard_m": oakd_geometry_edge_guard_m,
			"oakd_border_crop_px": oakd_border_crop_px,
			"oakd_stabilization_enabled": oakd_stabilization_enabled,
			"oakd_stabilization_deadband_m": oakd_stabilization_deadband_m,
			"oakd_stabilization_hold_frames": oakd_stabilization_hold_frames,
			"oakd_fast_stereo_enabled": oakd_depth_source == "fast_foundation",
			"oakd_fast_stereo_backend": oakd_fast_backend,
			"oakd_fast_stereo_model_profile": oakd_fast_profile,
			"oakd_fast_stereo_iters": oakd_fast_iters,
			"oakd_fast_stereo_scale": oakd_fast_scale,
			"oakd_fast_stereo_torch_compile": false,
		}, true)
	else:
		return
	_send_udp(payload)

func _realsense_direct_renderer_available() -> bool:
	return not _realsense_camera_order.is_empty()

func _realsense_direct_renderer_available_for(camera_id: String) -> bool:
	var node := _camera_nodes.get(camera_id) as Node
	if node == null or not is_instance_valid(node):
		node = _find_camera_renderer(camera_id)
	return node != null and node.has_method("set_direct_realsense_enabled")

func _realsense_direct_capture_active(camera_id: String = CAMERA_REALSENSE) -> bool:
	if camera_id.begins_with(CAMERA_REALSENSE_PREFIX):
		var source := str(_realsense_setting(camera_id, "depth_source", realsense_depth_source))
		return source in ["sdk_depth", "fast_foundation_native"] and _realsense_direct_renderer_available_for(camera_id)
	return realsense_depth_source in ["sdk_depth", "fast_foundation_native"] and _realsense_direct_renderer_available()

func _update_realsense_direct_status() -> void:
	var lines: Array[String] = []
	for camera_id in _realsense_camera_order:
		var node := _camera_nodes.get(camera_id) as Node
		if node == null or not is_instance_valid(node):
			node = _find_camera_renderer(camera_id)
		if node != null and node.has_method("get_direct_realsense_status"):
			lines.append("%s: %s" % [_camera_label(camera_id), str(node.call("get_direct_realsense_status"))])
	if not lines.is_empty():
		realsense_status = "\n\n".join(lines)

func _request_oakd_restart() -> void:
	if _point_cloud_stats_path.is_empty():
		_point_cloud_stats_path = ProjectSettings.globalize_path("user://point_cloud_stream_stats.json")
	var oakd_capture := _oakd_capture_settings()
	var payload := {
		"type": "oakd_restart",
		"enabled": _streams_enabled() and oakd_enabled,
		"stats_path": _point_cloud_stats_path,
		"oakd_width": int(oakd_capture["width"]),
		"oakd_height": int(oakd_capture["height"]),
		"oakd_fps": float(oakd_capture["fps"]),
		"oakd_rgb_res": str(oakd_capture["rgb_res"]),
		"oakd_mono_res": str(oakd_capture["mono_res"]),
		"oakd_stereo_preset": "fast_density",
		"oakd_lr_check": true,
		"oakd_subpixel": false,
		"oakd_subpixel_bits": 3,
		"oakd_confidence_threshold": 160,
		"oakd_median_filter": "off",
		"oakd_speckle_filter": false,
		"oakd_speckle_range": 0,
		"oakd_depth_source": oakd_depth_source,
		"oakd_use_rgb_color_for_host_depth": oakd_color_enabled,
		"oakd_host_depth_color_mode": _effective_oakd_color_mode(),
		"oakd_fast_stereo_enabled": oakd_depth_source == "fast_foundation",
		"oakd_fast_stereo_backend": oakd_fast_backend,
		"oakd_fast_stereo_model_profile": oakd_fast_profile,
		"oakd_fast_stereo_iters": oakd_fast_iters,
		"oakd_fast_stereo_scale": oakd_fast_scale,
		"oakd_fast_stereo_torch_compile": false,
	}
	oakd_status = "OAK-D restart requested..."
	_send_udp(payload)

func _request_realsense_restart() -> void:
	if _realsense_direct_capture_active():
		return
	var payload := {
		"type": "realsense_restart",
		"enabled": _streams_enabled() and realsense_enabled,
		"stream_profile": realsense_stream_profile,
		"depth_source": realsense_depth_source,
		"rs_fast_stereo_backend": realsense_fast_backend,
		"rs_fast_stereo_model_profile": realsense_fast_profile,
		"rs_fast_stereo_iters": realsense_fast_iters,
		"rs_fast_stereo_scale": realsense_fast_scale,
		"rs_fast_stereo_torch_compile": false,
	}
	_send_udp(payload)

func _realsense_cloud_alignment_running() -> bool:
	return (
		_native_realsense_calibration_running()
		or _realsense_cloud_alignment_pending
		or (_realsense_cloud_alignment_pid > 0 and OS.is_process_running(_realsense_cloud_alignment_pid))
	)

func _native_realsense_calibration_running() -> bool:
	return (
		_native_realsense_calibration_pending
		or (
			_native_realsense_calibrator != null
			and _native_realsense_calibrator.has_method("is_running")
			and bool(_native_realsense_calibrator.is_running())
		)
	)

func _enabled_realsense_serials() -> Array[String]:
	var serials: Array[String] = []
	for camera_id in _realsense_camera_order:
		if _camera_enabled(camera_id):
			serials.append(_realsense_serial(camera_id))
	return serials

func _cloud_alignment_serial_pair() -> Dictionary:
	var serials := _enabled_realsense_serials()
	if serials.size() < 2:
		return {
			"ok": false,
			"status": "RealSense cloud alignment needs at least two enabled discovered RealSense cameras.",
		}
	var reference_serial := realsense_cloud_alignment_reference_serial.strip_edges()
	var target_serial := realsense_cloud_alignment_target_serial.strip_edges()
	if reference_serial.is_empty():
		reference_serial = serials[0]
	if target_serial.is_empty():
		for serial in serials:
			if serial != reference_serial:
				target_serial = serial
				break
	if reference_serial == target_serial:
		return {
			"ok": false,
			"status": "RealSense cloud alignment reference and target serials must be different.",
		}
	if reference_serial not in serials:
		return {
			"ok": false,
			"status": "Reference RealSense serial %s is not currently discovered/enabled." % reference_serial,
		}
	if target_serial not in serials:
		return {
			"ok": false,
			"status": "Target RealSense serial %s is not currently discovered/enabled." % target_serial,
		}
	return {
		"ok": true,
		"reference_serial": reference_serial,
		"target_serial": target_serial,
	}

func _python_executable() -> String:
	return "python" if OS.has_feature("windows") else "python3"

func _transform_to_payload(transform: Transform3D) -> Dictionary:
	var b := transform.basis
	return {
		"R": [
			[b.x.x, b.y.x, b.z.x],
			[b.x.y, b.y.y, b.z.y],
			[b.x.z, b.y.z, b.z.z],
		],
		"T": [transform.origin.x, transform.origin.y, transform.origin.z],
	}

func _manual_realsense_alignment_target_id() -> String:
	var serial := realsense_manual_nudge_target_serial.strip_edges()
	if serial.is_empty():
		serial = realsense_cloud_alignment_target_serial.strip_edges()
	if serial.is_empty():
		var pair := _cloud_alignment_serial_pair()
		if _value_enabled(pair.get("ok", false)):
			serial = str(pair.get("target_serial", ""))
	if serial.is_empty() and _realsense_camera_order.size() > 1:
		serial = _realsense_serial(_realsense_camera_order[1])
	if serial.is_empty() and _realsense_camera_order.size() == 1:
		serial = _realsense_serial(_realsense_camera_order[0])
	if serial.is_empty():
		calibration_status = "Manual RealSense nudge needs a target serial."
		return ""
	var camera_id := _realsense_camera_id(serial)
	if camera_id not in _realsense_camera_order:
		calibration_status = "Manual RealSense nudge target is not currently discovered: %s" % serial
		return ""
	return camera_id

func _manual_realsense_reference_id(target_id: String) -> String:
	var reference_serial := realsense_cloud_alignment_reference_serial.strip_edges()
	if not reference_serial.is_empty() and reference_serial != _realsense_serial(target_id):
		return _realsense_camera_id(reference_serial)
	for camera_id in _realsense_camera_order:
		if camera_id != target_id and _camera_enabled(camera_id):
			return camera_id
	return CAMERA_REALSENSE

func _save_realsense_alignment_ground_truth() -> void:
	var target_id := _manual_realsense_alignment_target_id()
	if target_id.is_empty():
		return
	if _realsense_alignment_ground_truth_path.is_empty():
		_realsense_alignment_ground_truth_path = ProjectSettings.globalize_path(REALSENSE_ALIGNMENT_GROUND_TRUTH_PATH)
	var transform: Transform3D = _alignment_transforms.get(target_id, Transform3D.IDENTITY)
	var payload := _transform_to_payload(transform)
	payload["type"] = "realsense_alignment_ground_truth"
	payload["target_camera"] = target_id
	payload["reference_camera"] = _manual_realsense_reference_id(target_id)
	payload["saved_at"] = Time.get_unix_time_from_system()
	var out := FileAccess.open(_realsense_alignment_ground_truth_path, FileAccess.WRITE)
	if out == null:
		calibration_status = "Could not save RealSense alignment ground truth: %s" % _realsense_alignment_ground_truth_path
		return
	out.store_string(JSON.stringify(payload, "\t"))
	calibration_status = "Saved RealSense benchmark ground truth for %s" % _realsense_serial(target_id)
	_apply_saved_realsense_alignment_ground_truth(false)

func _apply_saved_realsense_alignment_ground_truth(show_missing_status: bool) -> bool:
	if _realsense_alignment_ground_truth_path.is_empty():
		_realsense_alignment_ground_truth_path = ProjectSettings.globalize_path(REALSENSE_ALIGNMENT_GROUND_TRUTH_PATH)
	if not FileAccess.file_exists(_realsense_alignment_ground_truth_path):
		if show_missing_status:
			calibration_status = "No saved RealSense alignment truth yet."
		return false
	var file := FileAccess.open(_realsense_alignment_ground_truth_path, FileAccess.READ)
	if file == null:
		if show_missing_status:
			calibration_status = "Could not read saved RealSense alignment truth."
		return false
	var parsed = JSON.parse_string(file.get_as_text())
	if typeof(parsed) != TYPE_DICTIONARY:
		if show_missing_status:
			calibration_status = "Saved RealSense alignment truth is invalid JSON."
		return false
	var payload := parsed as Dictionary
	var target_id := str(payload.get("target_camera", ""))
	if target_id.is_empty():
		target_id = _manual_realsense_alignment_target_id()
	if target_id.is_empty() or not target_id.begins_with(CAMERA_REALSENSE_PREFIX):
		if show_missing_status:
			calibration_status = "Saved RealSense alignment truth has no valid target camera."
		return false
	_alignment_transforms[target_id] = _transform_from_payload(payload)
	var status := "Applied locked RealSense alignment truth for %s" % _realsense_serial(target_id)
	calibration_status = status
	_persist_realsense_alignment_transform(target_id, "saved_ground_truth_lock", status)
	_update_camera_renderers()
	return true

func _persist_realsense_alignment_transform(target_id: String, method: String, status: String) -> void:
	if target_id.is_empty():
		return
	if _camera_alignment_registry_path.is_empty():
		_camera_alignment_registry_path = ProjectSettings.globalize_path(CAMERA_ALIGNMENT_REGISTRY_PATH)
	var registry: Dictionary = {}
	if FileAccess.file_exists(_camera_alignment_registry_path):
		var file := FileAccess.open(_camera_alignment_registry_path, FileAccess.READ)
		if file != null:
			var parsed = JSON.parse_string(file.get_as_text())
			if parsed is Dictionary:
				registry = parsed as Dictionary
	var reference_id := _manual_realsense_reference_id(target_id)
	registry["type"] = "camera_alignment_registry"
	registry["reference_camera"] = reference_id
	registry["updated_at"] = Time.get_unix_time_from_system()
	var transforms = registry.get("transforms", {})
	if typeof(transforms) != TYPE_DICTIONARY:
		transforms = {}
	var transforms_dict := transforms as Dictionary
	var payload := _transform_to_payload(_alignment_transforms.get(target_id, Transform3D.IDENTITY))
	payload["relative_to"] = reference_id
	payload["method"] = method
	payload["status"] = status
	payload["updated_at"] = Time.get_unix_time_from_system()
	transforms_dict[target_id] = payload
	registry["transforms"] = transforms_dict
	var out := FileAccess.open(_camera_alignment_registry_path, FileAccess.WRITE)
	if out != null:
		out.store_string(JSON.stringify(registry, "\t"))
		_camera_alignment_registry_token = ""

func _apply_manual_realsense_alignment_delta(translation_delta: Vector3, rotation_delta_deg: Vector3) -> void:
	var target_id := _manual_realsense_alignment_target_id()
	if target_id.is_empty():
		return
	var transform: Transform3D = _alignment_transforms.get(target_id, Transform3D.IDENTITY)
	if not rotation_delta_deg.is_zero_approx():
		var rotation := Basis.IDENTITY
		if absf(rotation_delta_deg.x) > 0.000001:
			rotation = Basis(Vector3.RIGHT, deg_to_rad(rotation_delta_deg.x)) * rotation
		if absf(rotation_delta_deg.y) > 0.000001:
			rotation = Basis(Vector3.UP, deg_to_rad(rotation_delta_deg.y)) * rotation
		if absf(rotation_delta_deg.z) > 0.000001:
			rotation = Basis(Vector3.BACK, deg_to_rad(rotation_delta_deg.z)) * rotation
		transform.basis = (rotation * transform.basis).orthonormalized()
	transform.origin += translation_delta
	_alignment_transforms[target_id] = transform
	var status := "Manual RealSense nudge %s | move=(%.3f, %.3f, %.3f)m rotate=(%.2f, %.2f, %.2f)deg" % [
		_realsense_serial(target_id),
		translation_delta.x,
		translation_delta.y,
		translation_delta.z,
		rotation_delta_deg.x,
		rotation_delta_deg.y,
		rotation_delta_deg.z,
	]
	calibration_status = status
	_persist_realsense_alignment_transform(target_id, "manual_nudge", status)
	_update_camera_renderers()

func _reset_manual_realsense_alignment() -> void:
	var target_id := _manual_realsense_alignment_target_id()
	if target_id.is_empty():
		return
	_alignment_transforms[target_id] = Transform3D.IDENTITY
	var status := "Manual RealSense alignment reset for %s" % _realsense_serial(target_id)
	calibration_status = status
	_persist_realsense_alignment_transform(target_id, "manual_reset", status)
	_update_camera_renderers()

func _point_pair_basis(points: Array[Vector3]) -> Basis:
	if points.size() < 3:
		return Basis.IDENTITY
	var origin := points[0]
	var x_axis := (points[1] - origin).normalized()
	var y_seed := points[2] - origin
	y_seed -= x_axis * y_seed.dot(x_axis)
	if x_axis.length_squared() <= 0.000001 or y_seed.length_squared() <= 0.000001:
		return Basis.IDENTITY
	var y_axis := y_seed.normalized()
	var z_axis := x_axis.cross(y_axis).normalized()
	y_axis = z_axis.cross(x_axis).normalized()
	return Basis(x_axis, y_axis, z_axis).orthonormalized()

func _sorted_node3d_children(node: Node) -> Array[Node3D]:
	var children: Array[Node3D] = []
	for child in node.get_children():
		if child is Node3D:
			children.append(child as Node3D)
	children.sort_custom(func(a: Node3D, b: Node3D) -> bool:
		return String(a.name).naturalnocasecmp_to(String(b.name)) < 0
	)
	return children

func _solve_realsense_point_pair_alignment() -> void:
	var target_id := _manual_realsense_alignment_target_id()
	if target_id.is_empty():
		return
	var root_node := _calibration_pairs_node(true)
	if root_node == null:
		calibration_status = "Could not create RealSense calibration pair folders."
		return
	var reference_node := root_node.get_node_or_null(CALIBRATION_REFERENCE_NODE)
	var target_node := root_node.get_node_or_null(CALIBRATION_TARGET_NODE)
	if reference_node == null or target_node == null:
		calibration_status = "CalibrationPairs needs Reference and Target folders."
		return
	var reference_children := _sorted_node3d_children(reference_node)
	var target_children := _sorted_node3d_children(target_node)
	var target_by_name: Dictionary = {}
	for child in target_children:
		target_by_name[String(child.name)] = child
	var reference_points: Array[Vector3] = []
	var target_points: Array[Vector3] = []
	var target_anchor := _camera_anchor(target_id, false)
	if target_anchor == null:
		calibration_status = "Point-pair solve needs target camera anchor."
		return
	var reference_space := _world_level_anchor(false)
	var world_to_self := reference_space.global_transform.affine_inverse() if reference_space != null else global_transform.affine_inverse()
	var target_world_to_local := target_anchor.global_transform.affine_inverse()
	for index in range(reference_children.size()):
		var reference_marker := reference_children[index]
		var target_marker: Node3D = null
		if target_by_name.has(String(reference_marker.name)):
			target_marker = target_by_name[String(reference_marker.name)] as Node3D
		elif index < target_children.size():
			target_marker = target_children[index]
		if target_marker == null:
			continue
		reference_points.append(world_to_self * reference_marker.global_position)
		target_points.append(target_world_to_local * target_marker.global_position)
	if reference_points.size() < 3 or target_points.size() < 3:
		calibration_status = "Point-pair solve needs at least 3 matching markers in CalibrationPairs/Reference and Target."
		return
	var reference_basis := _point_pair_basis(reference_points)
	var target_basis := _point_pair_basis(target_points)
	var solved_basis := (reference_basis * target_basis.inverse()).orthonormalized()
	var solved_origin := reference_points[0] - solved_basis * target_points[0]
	var solved := Transform3D(solved_basis, solved_origin)
	_alignment_transforms[target_id] = solved
	var max_error := 0.0
	var sum_sq := 0.0
	for i in reference_points.size():
		var error := (solved * target_points[i]).distance_to(reference_points[i])
		max_error = maxf(max_error, error)
		sum_sq += error * error
	var rmse := sqrt(sum_sq / float(reference_points.size()))
	var status := "Solved RealSense point-pair alignment %s | pairs=%d rmse=%.4fm max=%.4fm" % [
		_realsense_serial(target_id),
		reference_points.size(),
		rmse,
		max_error,
	]
	calibration_status = status
	_persist_realsense_alignment_transform(target_id, "point_pair_solve", status)
	_update_camera_renderers()

func _set_realsense_direct_renderers_enabled(enabled: bool) -> void:
	for camera_id in _realsense_camera_order:
		var node := _camera_nodes.get(camera_id) as Node
		if node == null or not is_instance_valid(node):
			node = _find_camera_renderer(camera_id)
		if node != null and node.has_method("set_direct_realsense_enabled"):
			if not enabled:
				node.set_process(false)
			node.call("set_direct_realsense_enabled", enabled and _streams_enabled() and _camera_enabled(camera_id))
			if enabled:
				node.set_process(true)

func _packed_realsense_transform(transform: Transform3D) -> PackedFloat64Array:
	var basis := transform.basis
	return PackedFloat64Array([
		basis.x.x, basis.y.x, basis.z.x, transform.origin.x,
		basis.x.y, basis.y.y, basis.z.y, transform.origin.y,
		basis.x.z, basis.y.z, basis.z.z, transform.origin.z,
	])

func _native_realsense_saved_prior(reference_id: String, target_id: String) -> PackedFloat64Array:
	if _realsense_alignment_ground_truth_path.is_empty():
		_realsense_alignment_ground_truth_path = ProjectSettings.globalize_path(
			REALSENSE_ALIGNMENT_GROUND_TRUTH_PATH
		)
	if not FileAccess.file_exists(_realsense_alignment_ground_truth_path):
		return PackedFloat64Array()
	var file := FileAccess.open(_realsense_alignment_ground_truth_path, FileAccess.READ)
	if file == null:
		return PackedFloat64Array()
	var parsed = JSON.parse_string(file.get_as_text())
	if not parsed is Dictionary:
		return PackedFloat64Array()
	var payload := parsed as Dictionary
	if str(payload.get("reference_camera", "")) != reference_id:
		return PackedFloat64Array()
	if str(payload.get("target_camera", "")) != target_id:
		return PackedFloat64Array()
	return _packed_realsense_transform(_transform_from_payload(payload))

func _request_native_realsense_calibration(mode: String) -> void:
	mode = mode.strip_edges().to_lower()
	if mode not in ["aruco", "markerless", "refine"]:
		mode = "markerless"
	if not is_inside_tree():
		calibration_status = "Open the Unified Point Clouds scene before starting calibration."
		return
	if _realsense_cloud_alignment_running():
		calibration_status = "A RealSense calibration is already running."
		return
	var pair := _cloud_alignment_serial_pair()
	if not bool(pair.get("ok", false)):
		calibration_status = str(pair.get("status", "RealSense calibration could not start."))
		return
	if not ClassDB.class_exists("RealSensePairCalibrator"):
		calibration_status = "Native RealSense calibration is unavailable in this extension build."
		return
	var reference_serial := str(pair["reference_serial"])
	var target_serial := str(pair["target_serial"])
	var reference_id := _realsense_camera_id(reference_serial)
	var target_id := _realsense_camera_id(target_serial)
	_native_realsense_calibrator = ClassDB.instantiate("RealSensePairCalibrator")
	if _native_realsense_calibrator == null:
		calibration_status = "Could not create the native RealSense calibrator."
		return
	_native_realsense_calibration_mode = mode
	_native_realsense_calibration_reference_id = reference_id
	_native_realsense_calibration_target_id = target_id
	_native_realsense_calibration_pending = true
	_set_realsense_direct_renderers_enabled(false)
	calibration_status = "Native %s calibration is releasing the live camera pipelines..." % mode
	await get_tree().create_timer(1.25).timeout
	if not is_inside_tree() or _native_realsense_calibrator == null:
		_native_realsense_calibration_pending = false
		return
	var calibration_min_depth := maxf(_camera_min_depth(reference_id), _camera_min_depth(target_id))
	var calibration_max_depth := minf(
		realsense_calibration_max_depth_m,
		minf(_camera_max_depth(reference_id), _camera_max_depth(target_id))
	)
	calibration_max_depth = maxf(calibration_min_depth + 0.25, calibration_max_depth)
	var options := {
		"mode": mode,
		"reference_serial": reference_serial,
		"target_serial": target_serial,
		"profile": "highres30",
		"model_path": ProjectSettings.globalize_path(REALSENSE_NATIVE_CALIBRATION_MODEL_PATH),
		"marker_id": realsense_calibration_marker_id,
		"marker_size_m": realsense_calibration_marker_size_m,
		"capture_frames": 24 if mode == "aruco" else 48,
		"marker_frames": 24,
		"min_depth_m": calibration_min_depth,
		"max_depth_m": calibration_max_depth,
	}
	if mode == "refine":
		options["initial_transform"] = _packed_realsense_transform(
			_alignment_transforms.get(target_id, Transform3D.IDENTITY)
		)
	elif mode == "markerless":
		var prior := _native_realsense_saved_prior(reference_id, target_id)
		if prior.size() >= 12:
			options["prior_transform"] = prior
	var started := bool(_native_realsense_calibrator.start(options))
	_native_realsense_calibration_pending = false
	if not started:
		calibration_status = str(_native_realsense_calibrator.get_status())
		_native_realsense_calibrator = null
		_native_realsense_calibration_mode = ""
		_native_realsense_calibration_reference_id = ""
		_native_realsense_calibration_target_id = ""
		_restart_realsense_direct_renderers()
		return
	calibration_status = "Native %s calibration started: %s -> %s" % [mode, target_serial, reference_serial]

func _update_native_realsense_calibration() -> void:
	if _native_realsense_calibrator == null or _native_realsense_calibration_pending:
		return
	if bool(_native_realsense_calibrator.is_running()):
		calibration_status = str(_native_realsense_calibrator.get_status())
		return
	var result: Dictionary = _native_realsense_calibrator.get_result()
	var mode := _native_realsense_calibration_mode
	var reference_id := _native_realsense_calibration_reference_id
	var target_id := _native_realsense_calibration_target_id
	_native_realsense_calibrator = null
	_native_realsense_calibration_mode = ""
	_native_realsense_calibration_reference_id = ""
	_native_realsense_calibration_target_id = ""
	if bool(result.get("ok", false)):
		result["reference_camera"] = reference_id
		result["target_camera"] = target_id
		if _apply_realsense_cloud_alignment_payload(result):
			var status := str(result.get("status", "Native RealSense calibration applied."))
			_persist_realsense_alignment_transform(target_id, str(result.get("method", mode)), status)
			if mode == "aruco":
				_save_native_realsense_ground_truth(result, reference_id, target_id)
	else:
		calibration_status = str(result.get("status", "Native RealSense calibration failed."))
	_restart_realsense_direct_renderers()
	_update_camera_renderers()

func _save_native_realsense_ground_truth(result: Dictionary, reference_id: String, target_id: String) -> void:
	if _realsense_alignment_ground_truth_path.is_empty():
		_realsense_alignment_ground_truth_path = ProjectSettings.globalize_path(REALSENSE_ALIGNMENT_GROUND_TRUTH_PATH)
	var payload := result.duplicate(true)
	payload["type"] = "realsense_alignment_ground_truth"
	payload["reference_camera"] = reference_id
	payload["target_camera"] = target_id
	payload["saved_at"] = Time.get_unix_time_from_system()
	var out := FileAccess.open(_realsense_alignment_ground_truth_path, FileAccess.WRITE)
	if out == null:
		calibration_status += " | could not save ArUco ground truth"
		return
	out.store_string(JSON.stringify(payload, "\t"))
	calibration_status += " | ArUco ground truth saved"

func _restart_realsense_direct_renderers() -> void:
	for camera_id in _realsense_camera_order:
		var node := _camera_nodes.get(camera_id) as Node
		if node == null or not is_instance_valid(node):
			node = _find_camera_renderer(camera_id)
		if node != null:
			node.set_process(true)
			if node.has_method("restart_direct_realsense"):
				node.call("restart_direct_realsense")
			if node.has_method("set_direct_realsense_enabled"):
				node.call("set_direct_realsense_enabled", _streams_enabled() and _camera_enabled(camera_id))
	_update_camera_renderers()

func _request_realsense_cloud_alignment(mode: String) -> void:
	mode = mode.strip_edges().to_lower()
	if mode not in ["auto", "refine", "guarded_refine", "color_refine", "rgb_feature", "rgb_feature_rigid", "plane_align", "axis_search", "benchmark"]:
		mode = "refine"
	if _realsense_cloud_alignment_running():
		calibration_status = "RealSense cloud alignment is already running."
		return
	var pair := _cloud_alignment_serial_pair()
	if not bool(pair.get("ok", false)):
		calibration_status = str(pair.get("status", "RealSense cloud alignment could not start."))
		return
	var reference_serial := str(pair["reference_serial"])
	var target_serial := str(pair["target_serial"])
	var target_id := _realsense_camera_id(target_serial)
	var script_path := ProjectSettings.globalize_path(REALSENSE_CLOUD_ALIGNMENT_SCRIPT_PATH)
	if script_path.is_empty() or not FileAccess.file_exists(script_path):
		calibration_status = "RealSense cloud alignment script not found: %s" % script_path
		return
	if _realsense_cloud_alignment_result_path.is_empty():
		_realsense_cloud_alignment_result_path = ProjectSettings.globalize_path(REALSENSE_CLOUD_ALIGNMENT_RESULT_PATH)
	if _camera_alignment_registry_path.is_empty():
		_camera_alignment_registry_path = ProjectSettings.globalize_path(CAMERA_ALIGNMENT_REGISTRY_PATH)
	if _realsense_alignment_ground_truth_path.is_empty():
		_realsense_alignment_ground_truth_path = ProjectSettings.globalize_path(REALSENSE_ALIGNMENT_GROUND_TRUTH_PATH)
	if mode == "benchmark" and not FileAccess.file_exists(_realsense_alignment_ground_truth_path):
		calibration_status = "Save RealSense alignment ground truth after manual alignment before running benchmark."
		return
	var current_transform: Transform3D = _alignment_transforms.get(target_id, Transform3D.IDENTITY)
	var initial_json := JSON.stringify(_transform_to_payload(current_transform))
	if FileAccess.file_exists(_realsense_cloud_alignment_result_path):
		DirAccess.remove_absolute(_realsense_cloud_alignment_result_path)
		_realsense_cloud_alignment_result_token = ""
	_set_realsense_direct_renderers_enabled(false)
	_realsense_cloud_alignment_pending = true
	var args := PackedStringArray([
		script_path,
		"--reference-serial", reference_serial,
		"--target-serial", target_serial,
		"--reference-profile", str(_realsense_setting(_realsense_camera_id(reference_serial), "stream_profile", realsense_stream_profile)),
		"--target-profile", str(_realsense_setting(target_id, "stream_profile", realsense_stream_profile)),
		"--mode", mode,
		"--initial-transform", initial_json,
		"--result-path", _realsense_cloud_alignment_result_path,
		"--registry-path", _camera_alignment_registry_path,
		"--reference-min-depth", str(_camera_min_depth(_realsense_camera_id(reference_serial))),
		"--reference-max-depth", str(_camera_max_depth(_realsense_camera_id(reference_serial))),
		"--target-min-depth", str(_camera_min_depth(target_id)),
		"--target-max-depth", str(_camera_max_depth(target_id)),
		"--stride", str(_camera_stride(target_id)),
		"--frames", str(mini(realsense_cloud_alignment_frames, 4) if mode == "benchmark" else realsense_cloud_alignment_frames),
		"--voxel", str(realsense_cloud_alignment_voxel_m),
		"--max-correspondence", str(realsense_cloud_alignment_max_correspondence_m),
	])
	if mode == "benchmark":
		args.append("--timeout")
		args.append("60")
		args.append("--benchmark-ground-truth")
		args.append(_realsense_alignment_ground_truth_path)
	calibration_status = "RealSense cloud %s alignment waiting for camera release: target %s -> reference %s" % [mode, target_serial, reference_serial]
	await get_tree().create_timer(2.0).timeout
	if not is_inside_tree():
		_realsense_cloud_alignment_pending = false
		return
	_realsense_cloud_alignment_pid = OS.create_process(_python_executable(), args, false)
	_realsense_cloud_alignment_pending = false
	_realsense_cloud_alignment_started_msec = Time.get_ticks_msec()
	if _realsense_cloud_alignment_pid <= 0:
		_set_realsense_direct_renderers_enabled(true)
		calibration_status = "RealSense cloud alignment failed to launch Python worker."
		return
	calibration_status = "RealSense cloud %s alignment started: target %s -> reference %s" % [mode, target_serial, reference_serial]

func _update_realsense_cloud_alignment_process() -> void:
	if _realsense_cloud_alignment_pid <= 0:
		return
	if OS.is_process_running(_realsense_cloud_alignment_pid):
		return
	_realsense_cloud_alignment_pid = -1
	_poll_realsense_cloud_alignment_result(true)
	_set_realsense_direct_renderers_enabled(true)
	_restart_realsense_direct_renderers()
	_update_camera_renderers()

func _ensure_scene_anchors() -> void:
	_world_level_anchor(true)
	_camera_clouds_node(true)
	for camera_id in _known_camera_ids():
		_camera_anchor(camera_id, true)
	_calibration_pairs_node(true)
	_debug_panel_anchor(true)

func _world_level_anchor(create: bool) -> Node3D:
	var anchor := get_node_or_null(WORLD_LEVEL_ANCHOR_NAME) as Node3D
	if anchor == null and create:
		anchor = Node3D.new()
		anchor.name = WORLD_LEVEL_ANCHOR_NAME
		add_child(anchor)
		_assign_editor_owner(anchor)
	if anchor == null:
		return null
	# Migrate scenes saved before the shared anchor existed without changing their
	# visible transforms. The motion calibrator remains a root sibling by design.
	for child_name in [CAMERA_CLOUDS_NAME, CALIBRATION_PAIRS_NAME, ROBOT_OVERLAY_NAME]:
		var direct_child := get_node_or_null(child_name) as Node3D
		if direct_child == null or direct_child.get_parent() != self:
			continue
		var previous_global := direct_child.global_transform
		direct_child.reparent(anchor)
		direct_child.global_transform = previous_global
	return anchor

func _camera_clouds_node(create: bool) -> Node3D:
	var node := get_node_or_null(CAMERA_CLOUDS_NODE) as Node3D
	if node == null and create:
		var anchor := _world_level_anchor(true)
		if anchor == null:
			return null
		node = Node3D.new()
		node.name = CAMERA_CLOUDS_NAME
		anchor.add_child(node)
		_assign_editor_owner(node)
	return node

func _calibration_pairs_node(create: bool) -> Node3D:
	var root_node := get_node_or_null(CALIBRATION_PAIRS_NODE) as Node3D
	if root_node == null and create:
		var anchor := _world_level_anchor(true)
		if anchor == null:
			return null
		root_node = Node3D.new()
		root_node.name = CALIBRATION_PAIRS_NAME
		anchor.add_child(root_node)
		_assign_editor_owner(root_node)
	if root_node != null:
		for child_name in [CALIBRATION_REFERENCE_NODE, CALIBRATION_TARGET_NODE]:
			var child := root_node.get_node_or_null(child_name) as Node3D
			if child == null and create:
				child = Node3D.new()
				child.name = child_name
				root_node.add_child(child)
				_assign_editor_owner(child)
	return root_node

func _camera_anchor(camera_id: String, create: bool) -> Node3D:
	var clouds := _camera_clouds_node(create)
	if clouds == null:
		return null
	var anchor := clouds.get_node_or_null(_camera_anchor_name(camera_id)) as Node3D
	if anchor == null and camera_id.begins_with(CAMERA_REALSENSE_PREFIX):
		for child in clouds.get_children():
			if child is Node3D and child.has_method("get_settings"):
				var settings = child.call("get_settings")
				if settings is Dictionary and str((settings as Dictionary).get("serial", "")) == _realsense_serial(camera_id):
					anchor = child as Node3D
					break
		if anchor == null:
			anchor = clouds.get_node_or_null("RealSense%sCloudAnchor" % _realsense_serial(camera_id)) as Node3D
	if anchor == null and create:
		anchor = Node3D.new()
		anchor.name = _camera_anchor_name(camera_id)
		clouds.add_child(anchor)
		_assign_editor_owner(anchor)
	if anchor != null and camera_id.begins_with(CAMERA_REALSENSE_PREFIX):
		_configure_realsense_camera_anchor(anchor, camera_id)
	return anchor

func _debug_panel_anchor(create: bool) -> Node3D:
	var anchor := get_node_or_null(DEBUG_PANEL_ANCHOR_NODE) as Node3D
	if anchor == null and create:
		anchor = Node3D.new()
		anchor.name = DEBUG_PANEL_ANCHOR_NODE
		anchor.position = debug_panel_position
		add_child(anchor)
		_assign_editor_owner(anchor)
	return anchor

func _assign_editor_owner(node: Node) -> void:
	if not Engine.is_editor_hint() or not is_inside_tree() or get_tree() == null:
		return
	var scene_root := get_tree().edited_scene_root
	if scene_root == self and (node == scene_root or scene_root.is_ancestor_of(node)):
		node.owner = scene_root

func _configure_realsense_camera_anchor(anchor: Node3D, camera_id: String) -> void:
	if anchor == null:
		return
	var serial := _realsense_serial(camera_id)
	var had_settings_script := anchor.has_method("get_settings")
	if not had_settings_script:
		var script := load(REALSENSE_CAMERA_NODE_SCRIPT_PATH) as Script
		if script != null:
			anchor.set_script(script)
	var info: Dictionary = _realsense_devices.get(camera_id, {})
	var defaults := _realsense_settings(camera_id).duplicate(true)
	defaults["model"] = str(defaults.get("model", _realsense_model_name(info)))
	var apply_defaults := not had_settings_script
	if anchor.has_method("configure"):
		anchor.call("configure", camera_id, serial, _camera_label(camera_id), info, defaults, apply_defaults)
	_sync_realsense_registry_from_node(camera_id)

func _sync_realsense_registry_from_node(camera_id: String) -> void:
	var serial := _realsense_serial(camera_id)
	if serial.is_empty():
		return
	var node_settings := _realsense_camera_node_settings(camera_id)
	if node_settings.is_empty():
		return
	var settings := _realsense_settings(camera_id).duplicate(true)
	settings.merge(node_settings, true)
	realsense_device_registry[serial] = settings

func _on_realsense_camera_settings_changed(camera_id: String) -> void:
	if not _is_realsense_camera(camera_id):
		return
	_sync_realsense_registry_from_node(camera_id)
	_update_camera_renderers()
	_send_camera_stream_command(camera_id, _streams_enabled() and _camera_enabled(camera_id))
	_update_debug_panel(true)

func _update_camera_renderers() -> void:
	if not is_inside_tree():
		return
	if OS.has_feature("web"):
		return
	if not ClassDB.class_exists("RealSenseSharedMemoryPointCloud"):
		if not _native_missing_warned:
			_native_missing_warned = true
			push_warning("RealSenseSharedMemoryPointCloud native extension is unavailable; unified point-cloud view needs the native SHM renderer.")
		return
	for camera_id in _known_camera_ids():
		if _camera_enabled(camera_id):
			_ensure_camera_renderer(camera_id)
		else:
			_free_camera_renderer(camera_id)
	_update_unified_mesh_fusion()
	_update_robot_renderer_masks()

func _update_unified_mesh_fusion() -> void:
	var renderers: Array[MeshInstance3D] = []
	var fusion_camera_ids := _realsense_camera_ids()
	fusion_camera_ids.sort_custom(_fusion_camera_precedes)
	for camera_id in fusion_camera_ids:
		var renderer := _camera_nodes.get(camera_id) as MeshInstance3D
		if renderer != null and is_instance_valid(renderer) and _camera_enabled(camera_id) and _camera_diagnostic_visible(camera_id):
			renderers.append(renderer)
	for renderer in renderers:
		if renderer.has_method("set_fusion_enabled"):
			renderer.call("set_fusion_enabled", false)
		if renderer.has_method("set_fusion_color_only"):
			renderer.call("set_fusion_color_only", false)
	if render_mode == "shader_mesh" and renderers.size() >= 2:
		var reference_id := _realsense_camera_id(realsense_color_reference_serial.strip_edges())
		if not _camera_nodes.has(reference_id):
			for camera_id in fusion_camera_ids:
				if str(_realsense_setting(camera_id, "model", "")).to_upper().contains("D435"):
					reference_id = camera_id
					break
		var reference := _camera_nodes.get(reference_id) as MeshInstance3D
		if reference == null or not is_instance_valid(reference):
			return
		var reference_size: Vector2 = reference.call("get_current_grid_size")
		var reference_depth = reference.call("get_depth_texture")
		var reference_color = reference.call("get_color_texture")
		var reference_ready := reference_depth != null and reference_color != null and reference_size.x > 1.0 and reference_size.y > 1.0
		if not reference_ready:
			return
		for camera_id in fusion_camera_ids:
			if camera_id == reference_id:
				continue
			var renderer := _camera_nodes.get(camera_id) as MeshInstance3D
			if renderer == null or not is_instance_valid(renderer) or not renderer.has_method("set_fusion_color_only"):
				continue
			renderer.call("set_fusion_peer_depth_texture", reference_depth)
			renderer.call("set_fusion_peer_color_texture", reference_color)
			renderer.call("set_fusion_local_to_peer", reference.global_transform.affine_inverse() * renderer.global_transform)
			renderer.call("set_fusion_peer_intrinsics", reference.call("get_current_intrinsics"))
			renderer.call("set_fusion_peer_grid_size", reference_size)
			renderer.call("set_fusion_peer_grid_stride", int(reference.call("get_current_grid_stride")))
			renderer.call("set_fusion_depth_tolerance", maxf(mesh_fusion_depth_tolerance_m, 0.05))
			renderer.call("set_fusion_color_only", true)
			renderer.call("set_fusion_enabled", true)
		return
	if render_mode not in ["unified_mesh", "prefer_d455", "prefer_d435"] or renderers.size() < 2:
		return
	# The current view is a calibrated two-camera rig. Extra devices remain visible,
	# but only the first pair participates until multi-peer texture arrays are added.
	for index in range(2):
		var renderer := renderers[index]
		var peer := renderers[1 - index]
		if not renderer.has_method("set_fusion_enabled"):
			continue
		var peer_size: Vector2 = peer.call("get_current_grid_size") if peer.has_method("get_current_grid_size") else Vector2.ZERO
		var peer_depth = peer.call("get_depth_texture") if peer.has_method("get_depth_texture") else null
		var ready := peer_depth != null and peer_size.x > 1.0 and peer_size.y > 1.0
		renderer.call("set_fusion_enabled", ready)
		if not ready:
			continue
		renderer.call("set_fusion_peer_depth_texture", peer_depth)
		if peer.has_method("get_color_texture"):
			renderer.call("set_fusion_peer_color_texture", peer.call("get_color_texture"))
		renderer.call("set_fusion_local_to_peer", peer.global_transform.affine_inverse() * renderer.global_transform)
		renderer.call("set_fusion_peer_intrinsics", peer.call("get_current_intrinsics"))
		renderer.call("set_fusion_peer_grid_size", peer_size)
		renderer.call("set_fusion_peer_grid_stride", int(peer.call("get_current_grid_stride")))
		# renderers are ordered D455, then D435 by _fusion_camera_precedes().
		var loses_overlap := index == (1 if render_mode in ["unified_mesh", "prefer_d455"] else 0)
		renderer.call("set_fusion_priority", 1 if loses_overlap else 0)
		renderer.call("set_fusion_depth_tolerance", mesh_fusion_depth_tolerance_m)

func _fusion_camera_precedes(left_camera_id: String, right_camera_id: String) -> bool:
	return _fusion_camera_rank(left_camera_id) < _fusion_camera_rank(right_camera_id)

func _fusion_camera_rank(camera_id: String) -> int:
	var model := str(_realsense_setting(camera_id, "model", "")).to_lower()
	if model.contains("455"):
		return 0
	if model.contains("435"):
		return 1
	return 10

func _update_robot_renderer_masks() -> void:
	var overlay := get_node_or_null(ROBOT_OVERLAY_NODE)
	var world_capsules := PackedVector4Array()
	if overlay != null and overlay.has_method("get_mask_capsules_world"):
		world_capsules = overlay.call("get_mask_capsules_world")
	for renderer_variant in _camera_nodes.values():
		var renderer := renderer_variant as MeshInstance3D
		if renderer == null or not is_instance_valid(renderer) or not renderer.has_method("set_robot_mask_enabled"):
			continue
		var local_capsules := PackedVector4Array()
		var inverse := renderer.global_transform.affine_inverse()
		for capsule in world_capsules:
			var world_point := Vector3(capsule.x, capsule.y, capsule.z)
			var local_point := inverse * world_point
			local_capsules.append(Vector4(local_point.x, local_point.y, local_point.z, capsule.w))
		renderer.call("set_robot_mask_capsules", local_capsules)
		# The overlay node is the live authority after scene initialization. An
		# empty capsule set means either the model or scanned-robot masking is off.
		renderer.call("set_robot_mask_enabled", not local_capsules.is_empty())

func pick_point_from_world_ray(ray_origin: Vector3, ray_direction: Vector3) -> Dictionary:
	var direction := ray_direction.normalized()
	if not ray_origin.is_finite() or not direction.is_finite() or direction.length_squared() < 0.99:
		return {"hit": false}
	var best := {"hit": false}
	var best_distance := INF
	for camera_id_variant in _camera_nodes.keys():
		var camera_id := str(camera_id_variant)
		var renderer := _camera_nodes.get(camera_id) as MeshInstance3D
		if renderer == null or not is_instance_valid(renderer) or not renderer.visible:
			continue
		if not renderer.has_method("pick_world_point_near_ray"):
			continue
		var candidate = renderer.call("pick_world_point_near_ray", ray_origin, direction, 0.006, 2)
		if candidate is not Dictionary or not bool(candidate.get("hit", false)):
			continue
		var distance := float(candidate.get("distance", INF))
		var position = candidate.get("position", Vector3.INF)
		if distance <= 0.0 or distance >= best_distance or not position is Vector3 or not (position as Vector3).is_finite():
			continue
		best_distance = distance
		best = (candidate as Dictionary).duplicate(true)
		best["camera_id"] = camera_id
	if bool(best.get("hit", false)):
		print("Point-cloud focus hit %s at %s (%.3f m)" % [
			str(best.get("camera_id", "camera")),
			str(best.get("position", Vector3.ZERO)),
			float(best.get("distance", 0.0)),
		])
	else:
		print("Point-cloud focus ray missed the live depth grids.")
	return best

func _ensure_camera_renderer(camera_id: String) -> void:
	var node := _camera_nodes.get(camera_id) as MeshInstance3D
	var anchor := _camera_anchor(camera_id, true)
	if node == null or not is_instance_valid(node):
		node = _find_camera_renderer(camera_id)
	if node == null:
		node = ClassDB.instantiate("RealSenseSharedMemoryPointCloud") as MeshInstance3D
		node.name = _camera_node_name(camera_id)
		(anchor if anchor != null else self).add_child(node)
	elif anchor != null and node.get_parent() != anchor:
		var old_parent := node.get_parent()
		if old_parent != null:
			old_parent.remove_child(node)
		anchor.add_child(node)
	_camera_nodes[camera_id] = node
	_apply_camera_renderer_settings(camera_id, node)

func _free_camera_renderer(camera_id: String) -> void:
	var node := _camera_nodes.get(camera_id) as Node
	if node == null or not is_instance_valid(node):
		node = _find_camera_renderer(camera_id)
	if node != null:
		if _is_realsense_camera(camera_id) and node.has_method("set_direct_realsense_enabled"):
			node.call("set_direct_realsense_enabled", false)
		node.queue_free()
	_camera_nodes.erase(camera_id)

func _find_camera_renderer(camera_id: String) -> MeshInstance3D:
	var node_name := _camera_node_name(camera_id)
	var anchor := _camera_anchor(camera_id, false)
	if anchor != null:
		var anchored := anchor.get_node_or_null(node_name) as MeshInstance3D
		if anchored != null:
			return anchored
	return get_node_or_null(node_name) as MeshInstance3D

func _apply_camera_renderer_settings(camera_id: String, node: MeshInstance3D) -> void:
	node.visible = _streams_enabled() and _camera_enabled(camera_id) and _camera_diagnostic_visible(camera_id)
	node.transform = _camera_transform(camera_id)
	node.call("set_shared_memory_name", _camera_shm_name(camera_id))
	node.call("set_point_pixel_size", _camera_point_pixel_size(camera_id))
	node.call("set_min_depth", _camera_min_depth(camera_id))
	node.call("set_max_depth", _camera_max_depth(camera_id))
	node.call("set_render_connected_mesh", _mesh_enabled())
	if node.has_method("set_gpu_connected_mesh"):
		node.call("set_gpu_connected_mesh", _gpu_mesh_enabled())
	if node.has_method("set_cpu_project_points"):
		node.call("set_cpu_project_points", false)
	if node.has_method("set_color_enabled"):
		node.call("set_color_enabled", _camera_color_enabled(camera_id))
	if node.has_method("set_color_match_enabled"):
		node.call("set_color_match_enabled", bool(_realsense_setting(camera_id, "color_match_enabled", false)))
		node.call("set_color_gain", _realsense_setting(camera_id, "color_gain", Vector3.ONE))
		node.call("set_color_bias", _realsense_setting(camera_id, "color_bias", Vector3.ZERO))
	if node.has_method("set_circular_point_splats"):
		node.call("set_circular_point_splats", render_mode == "point_splats")
	if node.has_method("set_point_cleanup_enabled"):
		node.call("set_point_cleanup_enabled", cleanup_enabled)
		node.call("set_point_cleanup_depth_delta", cleanup_depth_delta_m)
		node.call("set_point_cleanup_min_neighbors", cleanup_min_close_neighbors)
	if node.has_method("set_edge_feather_enabled"):
		node.call("set_edge_feather_enabled", edge_feather_enabled)
		node.call("set_edge_feather_width", edge_feather_width_m)
		node.call("set_edge_feather_min_alpha", edge_feather_min_alpha)
	node.call("set_mesh_max_edge", mesh_max_edge_m)
	node.call("set_mesh_max_depth_delta", mesh_max_depth_delta_m)
	if node.has_method("set_mesh_min_triangle_area"):
		node.call("set_mesh_min_triangle_area", mesh_min_triangle_area_m2)
	node.call("set_texture_map_mesh", texture_map_mesh)
	if node.has_method("set_gpu_mesh_compute_indices"):
		node.call("set_gpu_mesh_compute_indices", _gpu_mesh_compute_enabled())
	if node.has_method("set_gpu_mesh_static_shader"):
		node.call("set_gpu_mesh_static_shader", _gpu_mesh_static_shader_enabled())
	if _is_realsense_camera(camera_id) and node.has_method("set_direct_realsense_enabled"):
		var depth_source := str(_realsense_setting(camera_id, "depth_source", realsense_depth_source))
		var native_depth_source := _native_realsense_depth_source(depth_source)
		if node.has_method("set_direct_realsense_serial"):
			node.call("set_direct_realsense_serial", _realsense_serial(camera_id))
		node.call("set_direct_realsense_stream_profile", str(_realsense_setting(camera_id, "stream_profile", realsense_stream_profile)))
		if node.has_method("set_direct_realsense_depth_source"):
			node.call("set_direct_realsense_depth_source", native_depth_source)
		if node.has_method("set_direct_realsense_fast_foundation_backend"):
			node.call("set_direct_realsense_fast_foundation_backend", str(_realsense_setting(camera_id, "fast_backend", realsense_fast_backend)))
		if node.has_method("set_direct_realsense_fast_foundation_profile"):
			node.call("set_direct_realsense_fast_foundation_profile", str(_realsense_setting(camera_id, "fast_profile", realsense_fast_profile)))
		node.call("set_direct_realsense_stride", _camera_stride(camera_id))
		if node.has_method("set_direct_realsense_emitter_enabled"):
			node.call("set_direct_realsense_emitter_enabled", bool(_realsense_setting(camera_id, "emitter_enabled", true)))
		if node.has_method("set_direct_realsense_laser_power_percent"):
			node.call("set_direct_realsense_laser_power_percent", float(_realsense_setting(camera_id, "laser_power_percent", 100.0)))
		if node.has_method("set_direct_realsense_crop_rect"):
			var crop_enabled := bool(_realsense_setting(camera_id, "workspace_crop_enabled", false))
			var crop_rect := Vector4(0.0, 0.0, 1.0, 1.0)
			if crop_enabled:
				crop_rect = Vector4(
					float(_realsense_setting(camera_id, "crop_left_percent", 0.0)) * 0.01,
					float(_realsense_setting(camera_id, "crop_top_percent", 0.0)) * 0.01,
					1.0 - float(_realsense_setting(camera_id, "crop_right_percent", 0.0)) * 0.01,
					1.0 - float(_realsense_setting(camera_id, "crop_bottom_percent", 0.0)) * 0.01
				)
			node.call("set_direct_realsense_crop_rect", crop_rect)
		if node.has_method("set_direct_realsense_filter_config"):
			node.call("set_direct_realsense_filter_config", _direct_realsense_filter_config(camera_id))
		node.call("set_direct_realsense_enabled", _streams_enabled() and _camera_enabled(camera_id) and not _realsense_cloud_alignment_running() and (depth_source in ["sdk_depth", "fast_foundation_native"]))
		if node.has_method("get_direct_realsense_status"):
			realsense_status = "%s: %s" % [_camera_label(camera_id), str(node.call("get_direct_realsense_status"))]

func _match_realsense_colors_now() -> void:
	var live_ids: Array[String] = []
	for camera_id in _realsense_camera_ids():
		var renderer := _camera_nodes.get(camera_id) as MeshInstance3D
		if renderer != null and is_instance_valid(renderer) and renderer.has_method("get_depth_image") and renderer.has_method("get_color_image"):
			live_ids.append(camera_id)
	if live_ids.size() < 2:
		realsense_color_match_status = "Color match needs two live RealSense renderers."
		return

	var reference_id := ""
	var requested_serial := realsense_color_reference_serial.strip_edges()
	if not requested_serial.is_empty():
		var requested_id := _realsense_camera_id(requested_serial)
		if requested_id in live_ids:
			reference_id = requested_id
	if reference_id.is_empty():
		for camera_id in live_ids:
			if str(_realsense_setting(camera_id, "model", "")).to_upper().contains("D435"):
				reference_id = camera_id
				break
	if reference_id.is_empty():
		reference_id = live_ids[0]
	var target_id := ""
	for camera_id in live_ids:
		if camera_id != reference_id and str(_realsense_setting(camera_id, "model", "")).to_upper().contains("D455"):
			target_id = camera_id
			break
	if target_id.is_empty():
		for camera_id in live_ids:
			if camera_id != reference_id:
				target_id = camera_id
				break
	if target_id.is_empty():
		realsense_color_match_status = "Could not choose a non-reference RealSense camera."
		return
	var target_anchor := _find_realsense_camera_anchor_existing(target_id)
	if target_anchor == null:
		realsense_color_match_status = "Color match could not find the target camera settings node."
		return
	# The reference must be raw too: an earlier failed run may have corrected the
	# wrong camera. Always restore every live RealSense camera before measuring.
	for camera_id in live_ids:
		var camera_anchor := _find_realsense_camera_anchor_existing(camera_id)
		if camera_anchor != null and camera_anchor.has_method("reset_color_match"):
			camera_anchor.call("reset_color_match")
			_sync_realsense_registry_from_node(camera_id)
	_update_camera_renderers()

	var reference := _camera_nodes.get(reference_id) as MeshInstance3D
	var target := _camera_nodes.get(target_id) as MeshInstance3D
	var reference_depth := reference.call("get_depth_image") as Image
	var reference_color := reference.call("get_color_image") as Image
	var target_depth := target.call("get_depth_image") as Image
	var target_color := target.call("get_color_image") as Image
	if _color_match_image_missing(reference_depth) or _color_match_image_missing(reference_color) or _color_match_image_missing(target_depth) or _color_match_image_missing(target_color):
		realsense_color_match_status = "Color match is waiting for complete depth and RGB frames from both cameras."
		return

	var reference_intrinsics: Vector4 = reference.call("get_current_intrinsics")
	var target_intrinsics: Vector4 = target.call("get_current_intrinsics")
	if reference_intrinsics.x <= 0.0 or reference_intrinsics.y <= 0.0 or target_intrinsics.x <= 0.0 or target_intrinsics.y <= 0.0:
		realsense_color_match_status = "Color match could not read valid camera intrinsics."
		return

	var target_samples := PackedVector3Array()
	var reference_samples := PackedVector3Array()
	var target_to_world := target.global_transform
	var world_to_reference := reference.global_transform.affine_inverse()
	var sample_step := maxi(3, int(ceil(sqrt(float(target_depth.get_width() * target_depth.get_height()) / 45000.0))))
	for y in range(sample_step, target_depth.get_height() - sample_step, sample_step):
		for x in range(sample_step, target_depth.get_width() - sample_step, sample_step):
			var target_depth_m := target_depth.get_pixel(x, y).r
			if target_depth_m < _camera_min_depth(target_id) or target_depth_m > _camera_max_depth(target_id):
				continue
			if _color_match_depth_edge(target_depth, x, y, target_depth_m, sample_step):
				continue
			var target_local := Vector3(
				(float(x) - target_intrinsics.z) * target_depth_m / target_intrinsics.x,
				-(float(y) - target_intrinsics.w) * target_depth_m / target_intrinsics.y,
				-target_depth_m
			)
			var reference_local := world_to_reference * (target_to_world * target_local)
			var projected_depth := -reference_local.z
			if projected_depth < _camera_min_depth(reference_id) or projected_depth > _camera_max_depth(reference_id):
				continue
			var reference_x := int(round(reference_local.x * reference_intrinsics.x / projected_depth + reference_intrinsics.z))
			var reference_y := int(round(-reference_local.y * reference_intrinsics.y / projected_depth + reference_intrinsics.w))
			if reference_x < sample_step or reference_y < sample_step or reference_x >= reference_depth.get_width() - sample_step or reference_y >= reference_depth.get_height() - sample_step:
				continue
			var reference_depth_m := reference_depth.get_pixel(reference_x, reference_y).r
			var depth_tolerance := maxf(0.025, projected_depth * 0.02)
			if reference_depth_m <= 0.0 or absf(reference_depth_m - projected_depth) > depth_tolerance:
				continue
			if _color_match_depth_edge(reference_depth, reference_x, reference_y, reference_depth_m, sample_step):
				continue
			var target_rgb := _color_match_rgb_at_depth_pixel(target_color, target_depth, x, y)
			var reference_rgb := _color_match_rgb_at_depth_pixel(reference_color, reference_depth, reference_x, reference_y)
			if not _color_match_rgb_valid(target_rgb) or not _color_match_rgb_valid(reference_rgb):
				continue
			target_samples.append(target_rgb)
			reference_samples.append(reference_rgb)

	if target_samples.size() < 300:
		realsense_color_match_status = "Color match rejected: only %d valid overlap samples (need 300)." % target_samples.size()
		return
	var fit := _fit_realsense_color_affine(target_samples, reference_samples)
	if fit.is_empty():
		realsense_color_match_status = "Color match rejected: overlap samples did not produce a stable fit."
		return
	var before_error := float(fit.get("before_error", INF))
	var after_error := float(fit.get("after_error", INF))
	if not is_finite(before_error) or not is_finite(after_error) or after_error >= before_error * 0.98 or before_error - after_error < 0.001:
		realsense_color_match_status = "Color match kept existing settings: %d samples, residual %.4f -> %.4f (insufficient improvement)." % [target_samples.size(), before_error, after_error]
		return

	var gain: Vector3 = fit["gain"]
	var bias: Vector3 = fit["bias"]
	target_anchor.set("color_gain", gain)
	target_anchor.set("color_bias", bias)
	target_anchor.set("color_match_enabled", true)
	_sync_realsense_registry_from_node(target_id)
	_update_camera_renderers()
	realsense_color_match_status = "%s matched to %s: %d samples, residual %.4f -> %.4f, gain=%s bias=%s" % [
		_realsense_serial(target_id),
		_realsense_serial(reference_id),
		target_samples.size(),
		before_error,
		after_error,
		str(gain),
		str(bias),
	]

func _clear_realsense_color_matching() -> void:
	var cleared := 0
	for camera_id in _realsense_camera_ids():
		var camera_anchor := _find_realsense_camera_anchor_existing(camera_id)
		if camera_anchor == null or not camera_anchor.has_method("reset_color_match"):
			continue
		camera_anchor.call("reset_color_match")
		_sync_realsense_registry_from_node(camera_id)
		cleared += 1
	_update_camera_renderers()
	realsense_color_match_status = "Cleared RealSense color matching for %d camera(s); gain=(1, 1, 1), bias=(0, 0, 0)." % cleared

func _color_match_image_missing(image: Image) -> bool:
	return image == null or image.is_empty() or image.get_width() < 2 or image.get_height() < 2

func _color_match_depth_edge(image: Image, x: int, y: int, center: float, offset: int) -> bool:
	var edge_limit := maxf(0.025, center * 0.025)
	for neighbor in [image.get_pixel(x - offset, y).r, image.get_pixel(x + offset, y).r, image.get_pixel(x, y - offset).r, image.get_pixel(x, y + offset).r]:
		if neighbor <= 0.0 or absf(neighbor - center) > edge_limit:
			return true
	return false

func _color_match_rgb_at_depth_pixel(color_image: Image, depth_image: Image, depth_x: int, depth_y: int) -> Vector3:
	var color_x := clampi(int(round((float(depth_x) + 0.5) * color_image.get_width() / depth_image.get_width() - 0.5)), 0, color_image.get_width() - 1)
	var color_y := clampi(int(round((float(depth_y) + 0.5) * color_image.get_height() / depth_image.get_height() - 0.5)), 0, color_image.get_height() - 1)
	var color := color_image.get_pixel(color_x, color_y)
	return Vector3(color.r, color.g, color.b)

func _color_match_rgb_valid(rgb: Vector3) -> bool:
	return rgb.is_finite() and minf(rgb.x, minf(rgb.y, rgb.z)) > 0.03 and maxf(rgb.x, maxf(rgb.y, rgb.z)) < 0.97

func _fit_realsense_color_affine(source: PackedVector3Array, reference: PackedVector3Array) -> Dictionary:
	if source.size() != reference.size() or source.size() < 2:
		return {}
	var all_indices := PackedInt32Array()
	all_indices.resize(source.size())
	for index in source.size():
		all_indices[index] = index
	# Fit robust color distributions instead of individual RGB pairs. Slight
	# geometric misregistration can pair neighboring pixels from different objects,
	# while percentiles remain stable for the same depth-verified shared surface.
	var initial_gain := Vector3.ONE
	var initial_bias := Vector3.ZERO
	for channel in 3:
		var channel_fit := _fit_realsense_color_distribution(source, reference, all_indices, channel)
		initial_gain[channel] = channel_fit.x
		initial_bias[channel] = channel_fit.y
	var residuals := PackedFloat32Array()
	residuals.resize(source.size())
	for index in source.size():
		residuals[index] = _color_match_residual(source[index], reference[index], initial_gain, initial_bias)
	var sorted_residuals := residuals.duplicate()
	sorted_residuals.sort()
	var trim_threshold := sorted_residuals[clampi(int(floor(sorted_residuals.size() * 0.80)), 0, sorted_residuals.size() - 1)]
	var kept_indices := PackedInt32Array()
	for index in source.size():
		if residuals[index] <= trim_threshold:
			kept_indices.append(index)
	if kept_indices.size() < 200:
		return {}
	var gain := Vector3.ONE
	var bias := Vector3.ZERO
	for channel in 3:
		var channel_fit := _fit_realsense_color_distribution(source, reference, kept_indices, channel)
		gain[channel] = channel_fit.x
		bias[channel] = channel_fit.y
	var before_error := 0.0
	var after_error := 0.0
	for index in kept_indices:
		before_error += _color_match_residual(source[index], reference[index], Vector3.ONE, Vector3.ZERO)
		after_error += _color_match_residual(source[index], reference[index], gain, bias)
	before_error /= kept_indices.size()
	after_error /= kept_indices.size()
	return {"gain": gain, "bias": bias, "before_error": before_error, "after_error": after_error, "kept_samples": kept_indices.size()}

func _fit_realsense_color_distribution(source: PackedVector3Array, reference: PackedVector3Array, indices: PackedInt32Array, channel: int) -> Vector2:
	var source_values := PackedFloat32Array()
	var reference_values := PackedFloat32Array()
	for index in indices:
		var source_value := source[index][channel]
		var reference_value := reference[index][channel]
		if source_value > 0.03 and source_value < 0.97 and reference_value > 0.03 and reference_value < 0.97:
			source_values.append(source_value)
			reference_values.append(reference_value)
	if source_values.size() < 100 or reference_values.size() < 100:
		return Vector2(1.0, 0.0)
	source_values.sort()
	reference_values.sort()
	var source_low := _sorted_color_quantile(source_values, 0.15)
	var source_mid := _sorted_color_quantile(source_values, 0.50)
	var source_high := _sorted_color_quantile(source_values, 0.85)
	var reference_low := _sorted_color_quantile(reference_values, 0.15)
	var reference_mid := _sorted_color_quantile(reference_values, 0.50)
	var reference_high := _sorted_color_quantile(reference_values, 0.85)
	var gain := clampf((reference_high - reference_low) / maxf(source_high - source_low, 0.04), 0.75, 1.25)
	var bias := clampf(reference_mid - gain * source_mid, -0.08, 0.08)
	# Preserve usable shadows and highlights after the affine transform.
	if source_high * gain + bias > 0.95:
		bias -= source_high * gain + bias - 0.95
	if source_low * gain + bias < 0.03:
		bias += 0.03 - (source_low * gain + bias)
	return Vector2(gain, clampf(bias, -0.08, 0.08))

func _sorted_color_quantile(values: PackedFloat32Array, quantile: float) -> float:
	return values[clampi(int(round((values.size() - 1) * quantile)), 0, values.size() - 1)]

func _color_match_residual(source: Vector3, reference: Vector3, gain: Vector3, bias: Vector3) -> float:
	var corrected := source * gain + bias
	return (absf(corrected.x - reference.x) + absf(corrected.y - reference.y) + absf(corrected.z - reference.z)) / 3.0

func _request_big_aruco_alignment() -> void:
	if _alignment_result_path.is_empty():
		_alignment_result_path = ProjectSettings.globalize_path("user://oakd_realsense_alignment.json")
	var payload := {
		"type": "oakd_realsense_align",
		"method": "big_aruco",
		"min_depth": min_depth_m,
		"max_depth": max_depth_m,
		"stride": mini(_camera_stride(CAMERA_REALSENSE), _camera_stride(CAMERA_OAKD)),
		"marker_size_m": BIG_ARUCO_MARKER_SIZE_M,
		"aruco_dictionary": BIG_ARUCO_DICTIONARY,
		"aruco_marker_id": -1,
		"aruco_marker_ids": big_aruco_marker_ids,
		"auto_depth_refine": big_aruco_auto_depth_refine,
		"result_path": _alignment_result_path,
	}
	calibration_status = "Big ArUco alignment requested..."
	_send_udp(payload)

func _transform_from_payload(payload: Dictionary) -> Transform3D:
	if not payload.has("R") or not payload.has("T"):
		return Transform3D.IDENTITY
	var r: Array = payload["R"]
	var t: Array = payload["T"]
	if r.size() < 3 or t.size() < 3:
		return Transform3D.IDENTITY
	var basis := Basis(
		Vector3(float(r[0][0]), float(r[1][0]), float(r[2][0])),
		Vector3(float(r[0][1]), float(r[1][1]), float(r[2][1])),
		Vector3(float(r[0][2]), float(r[1][2]), float(r[2][2]))
	).orthonormalized()
	return Transform3D(basis, Vector3(float(t[0]), float(t[1]), float(t[2])))

func _apply_realsense_cloud_alignment_payload(payload: Dictionary) -> bool:
	if not bool(payload.get("ok", false)):
		calibration_status = str(payload.get("status", "RealSense cloud alignment failed."))
		return false
	if str(payload.get("method", "")) == "cloud_benchmark":
		calibration_status = str(payload.get("status", "RealSense alignment benchmark complete."))
		return true
	var target_camera := str(payload.get("target_camera", ""))
	if not target_camera.begins_with(CAMERA_REALSENSE_PREFIX):
		calibration_status = "RealSense cloud alignment result has invalid target camera: %s" % target_camera
		return false
	_alignment_transforms[target_camera] = _transform_from_payload(payload)
	calibration_status = str(payload.get("status", "RealSense cloud alignment applied."))
	_update_camera_renderers()
	return true

func _poll_realsense_cloud_alignment_result(force: bool) -> void:
	if _realsense_cloud_alignment_result_path.is_empty():
		_realsense_cloud_alignment_result_path = ProjectSettings.globalize_path(REALSENSE_CLOUD_ALIGNMENT_RESULT_PATH)
	if not FileAccess.file_exists(_realsense_cloud_alignment_result_path):
		return
	var modified := int(FileAccess.get_modified_time(_realsense_cloud_alignment_result_path))
	if modified <= 0:
		return
	var file := FileAccess.open(_realsense_cloud_alignment_result_path, FileAccess.READ)
	if file == null:
		return
	var text := file.get_as_text()
	var token := "%d:%d" % [modified, text.hash()]
	if not force and token == _realsense_cloud_alignment_result_token:
		return
	_realsense_cloud_alignment_result_token = token
	var parsed = JSON.parse_string(text)
	if typeof(parsed) != TYPE_DICTIONARY:
		return
	_apply_realsense_cloud_alignment_payload(parsed as Dictionary)

func _poll_camera_alignment_registry(force: bool) -> void:
	if _camera_alignment_registry_path.is_empty():
		_camera_alignment_registry_path = ProjectSettings.globalize_path(CAMERA_ALIGNMENT_REGISTRY_PATH)
	if not FileAccess.file_exists(_camera_alignment_registry_path):
		return
	var modified := int(FileAccess.get_modified_time(_camera_alignment_registry_path))
	if modified <= 0:
		return
	var file := FileAccess.open(_camera_alignment_registry_path, FileAccess.READ)
	if file == null:
		return
	var text := file.get_as_text()
	var token := "%d:%d" % [modified, text.hash()]
	if not force and token == _camera_alignment_registry_token:
		return
	_camera_alignment_registry_token = token
	var parsed = JSON.parse_string(text)
	if typeof(parsed) != TYPE_DICTIONARY:
		return
	var registry := parsed as Dictionary
	var transforms = registry.get("transforms", {})
	if typeof(transforms) != TYPE_DICTIONARY:
		return
	var loaded_status := ""
	for target_camera in (transforms as Dictionary).keys():
		var entry = (transforms as Dictionary)[target_camera]
		if typeof(entry) != TYPE_DICTIONARY:
			continue
		var target_id := str(target_camera)
		if not target_id.begins_with(CAMERA_REALSENSE_PREFIX):
			continue
		var entry_dict := entry as Dictionary
		if entry_dict.has("R") and entry_dict.has("T"):
			_alignment_transforms[target_id] = _transform_from_payload(entry_dict)
			var method := str(entry_dict.get("method", "saved"))
			loaded_status = "Loaded saved RealSense alignment for %s (%s)." % [
				_realsense_serial(target_id),
				method,
			]
	if force and not loaded_status.is_empty() and not _realsense_cloud_alignment_running() and _native_realsense_calibrator == null:
		calibration_status = loaded_status
	_update_camera_renderers()

func _poll_alignment_result(force: bool) -> void:
	if _alignment_result_path.is_empty():
		_alignment_result_path = ProjectSettings.globalize_path("user://oakd_realsense_alignment.json")
	if not FileAccess.file_exists(_alignment_result_path):
		return
	var modified := int(FileAccess.get_modified_time(_alignment_result_path))
	if modified <= 0:
		return
	var file := FileAccess.open(_alignment_result_path, FileAccess.READ)
	if file == null:
		return
	var text := file.get_as_text()
	var token := "%d:%d" % [modified, text.hash()]
	if not force and token == _alignment_result_token:
		return
	_alignment_result_token = token
	var parsed = JSON.parse_string(text)
	if typeof(parsed) != TYPE_DICTIONARY:
		return
	var payload: Dictionary = parsed
	calibration_status = _compact_alignment_status(payload)
	if not bool(payload.get("ok", false)):
		return
	if not payload.has("R") or not payload.has("T"):
		return
	var r: Array = payload["R"]
	var t: Array = payload["T"]
	if r.size() < 3 or t.size() < 3:
		return
	var alignment_basis := Basis(
		Vector3(float(r[0][0]), float(r[1][0]), float(r[2][0])),
		Vector3(float(r[0][1]), float(r[1][1]), float(r[2][1])),
		Vector3(float(r[0][2]), float(r[1][2]), float(r[2][2]))
	).orthonormalized()
	_alignment_transforms[CAMERA_OAKD] = Transform3D(alignment_basis, Vector3(float(t[0]), float(t[1]), float(t[2])))
	_update_camera_renderers()

func _poll_point_cloud_stats() -> void:
	if _point_cloud_stats_path.is_empty():
		_point_cloud_stats_path = ProjectSettings.globalize_path("user://point_cloud_stream_stats.json")
	if not FileAccess.file_exists(_point_cloud_stats_path):
		return
	var modified := int(FileAccess.get_modified_time(_point_cloud_stats_path))
	if modified <= 0:
		return
	var file := FileAccess.open(_point_cloud_stats_path, FileAccess.READ)
	if file == null:
		return
	var text := file.get_as_text()
	var token := "%d:%d" % [modified, text.hash()]
	if token == _point_cloud_stats_token:
		return
	_point_cloud_stats_token = token
	var parsed = JSON.parse_string(text)
	if typeof(parsed) == TYPE_DICTIONARY:
		_point_cloud_stats = parsed
		_update_oakd_status_from_stats()

func _update_oakd_status_from_stats() -> void:
	var stats: Dictionary = _point_cloud_stats.get(CAMERA_OAKD, {})
	if stats.is_empty():
		oakd_status = "OAK-D status unavailable"
		return
	var source := str(stats.get("source", "unknown"))
	var active_source := str(stats.get("active_source", source))
	var restart_required := bool(stats.get("restart_required", false))
	var active_size := "%dx%d@%.0f" % [
		int(stats.get("active_width", stats.get("width", 0))),
		int(stats.get("active_height", stats.get("height", 0))),
		float(stats.get("active_fps", 0.0)),
	]
	var requested_size := "%dx%d@%.0f" % [
		int(stats.get("requested_width", 0)),
		int(stats.get("requested_height", 0)),
		float(stats.get("requested_fps", 0.0)),
	]
	var status := "OAK-D active %s %s" % [active_source, active_size]
	if source != active_source:
		status += " | requested source %s" % source
	if requested_size != "0x0@0" and requested_size != active_size:
		status += " | requested %s" % requested_size
	if restart_required:
		status += "\nRestart required for deferred pipeline settings. Press Restart Oakd Now."
	oakd_status = status

func _native_stat(camera_id: String, method: String) -> float:
	var node := _camera_nodes.get(camera_id) as Object
	if node == null or not is_instance_valid(node) or not node.has_method(method):
		return 0.0
	return float(node.call(method))

func _compact_alignment_status(payload: Dictionary) -> String:
	var raw_status := str(payload.get("status", "")).replace("single ArUco", "big ArUco")
	var ok := bool(payload.get("ok", false))
	var details: Dictionary = payload.get("details", {}) if typeof(payload.get("details", {})) == TYPE_DICTIONARY else {}
	var shared_count := int(details.get("shared_marker_count", -1))
	if ok:
		var rmse := float(details.get("depth_refine_rmse", 0.0))
		if shared_count >= 0 and rmse > 0.0:
			return "Big ArUco OK: %d shared marker(s), depth rmse %.3fm" % [shared_count, rmse]
		if shared_count >= 0:
			return "Big ArUco OK: %d shared marker(s)" % shared_count
		return "Big ArUco OK"
	if raw_status.contains("markers=0") or raw_status.contains("no usable shared"):
		return "Big ArUco failed: no shared marker IDs visible to both cameras. Put the same ID in both views."
	if raw_status.length() > 180:
		return raw_status.substr(0, 177) + "..."
	return raw_status

func _free_debug_panel() -> void:
	if _debug_sprite != null and is_instance_valid(_debug_sprite):
		_debug_sprite.queue_free()
	if _debug_viewport != null and is_instance_valid(_debug_viewport):
		_debug_viewport.queue_free()
	_debug_sprite = null
	_debug_viewport = null
	_debug_root = null
	_debug_cells.clear()

func _ensure_debug_panel() -> void:
	if not show_debug_panel:
		_free_debug_panel()
		return
	if _debug_sprite != null and is_instance_valid(_debug_sprite) and _debug_viewport != null and is_instance_valid(_debug_viewport):
		return
	var anchor := _debug_panel_anchor(true)
	_debug_viewport = SubViewport.new()
	_debug_viewport.name = "UnifiedPointCloudDebugViewport"
	_debug_viewport.size = Vector2i(860, 190)
	_debug_viewport.transparent_bg = true
	_debug_viewport.disable_3d = true
	_debug_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	(anchor if anchor != null else self).add_child(_debug_viewport)
	_rebuild_debug_table()
	_debug_sprite = Sprite3D.new()
	_debug_sprite.name = "UnifiedPointCloudDebugPlane"
	_debug_sprite.texture = _debug_viewport.get_texture()
	_debug_sprite.position = Vector3.ZERO
	_debug_sprite.pixel_size = debug_panel_pixel_size
	_debug_sprite.fixed_size = false
	_debug_sprite.no_depth_test = false
	_debug_sprite.billboard = BaseMaterial3D.BILLBOARD_DISABLED
	(anchor if anchor != null else self).add_child(_debug_sprite)

func _rebuild_debug_table() -> void:
	if _debug_viewport == null or not is_instance_valid(_debug_viewport):
		return
	for child in _debug_viewport.get_children():
		child.queue_free()
	_debug_cells.clear()
	_debug_root = PanelContainer.new()
	_debug_root.set_anchors_preset(Control.PRESET_FULL_RECT)
	var panel_style := StyleBoxFlat.new()
	panel_style.bg_color = Color(0.025, 0.030, 0.035, 0.86)
	panel_style.border_color = Color(0.35, 0.55, 0.70, 0.95)
	panel_style.set_border_width_all(2)
	panel_style.set_corner_radius_all(8)
	panel_style.content_margin_left = 16
	panel_style.content_margin_right = 16
	panel_style.content_margin_top = 12
	panel_style.content_margin_bottom = 12
	_debug_root.add_theme_stylebox_override("panel", panel_style)
	_debug_viewport.add_child(_debug_root)

	var vbox := VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 8)
	_debug_root.add_child(vbox)

	var title := Label.new()
	title.text = "Point Cloud FPS"
	title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	title.add_theme_font_size_override("font_size", debug_font_size + 4)
	title.add_theme_color_override("font_color", Color(0.83, 0.94, 1.0, 1.0))
	vbox.add_child(title)

	var grid := GridContainer.new()
	grid.columns = 10
	grid.add_theme_constant_override("h_separation", 6)
	grid.add_theme_constant_override("v_separation", 5)
	vbox.add_child(grid)
	for header in ["Camera", "Cap", "Pub", "Disp", "Dev", "Host", "Work", "Frame", "Held", "Pts/Tri"]:
		grid.add_child(_debug_cell(header, "", true))
	for camera_id in _debug_camera_ids():
		grid.add_child(_debug_cell(_camera_label(camera_id), "", false, true))
		for metric in ["cap", "pub", "render", "device_age", "host_age", "work", "frame", "held", "points_tris"]:
			grid.add_child(_debug_cell("--", "%s_%s" % [camera_id, metric]))
	grid.add_child(_debug_cell("Total", "", false, true))
	grid.add_child(_debug_cell("", "total_cap"))
	grid.add_child(_debug_cell("", "total_pub"))
	grid.add_child(_debug_cell("", "total_render"))
	grid.add_child(_debug_cell("--", "total_device_age"))
	grid.add_child(_debug_cell("--", "total_host_age"))
	grid.add_child(_debug_cell("--", "total_work"))
	grid.add_child(_debug_cell("--", "total_frame"))
	grid.add_child(_debug_cell("--", "total_held"))
	grid.add_child(_debug_cell("--", "total_points_tris"))

func _debug_cell(text: String, key: String = "", header: bool = false, row_label: bool = false) -> Label:
	var label := Label.new()
	label.text = text
	label.custom_minimum_size = Vector2(74 if not row_label else 118, 28)
	label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	label.add_theme_font_size_override("font_size", debug_font_size if not header else debug_font_size - 1)
	var color := Color(0.78, 0.88, 0.95, 1.0)
	if header:
		color = Color(0.50, 0.68, 0.82, 1.0)
	elif row_label:
		color = Color(0.93, 0.95, 0.88, 1.0)
	label.add_theme_color_override("font_color", color)
	if key != "":
		_debug_cells[key] = label
	return label

func _update_debug_panel(force: bool) -> void:
	if not show_debug_panel:
		_free_debug_panel()
		return
	_ensure_debug_panel()
	var now_msec := Time.get_ticks_msec()
	if not force and now_msec - _last_debug_update_msec < 250:
		return
	_last_debug_update_msec = now_msec
	var total_render_points := 0
	var total_tris := 0
	var cap_values := []
	var pub_values := []
	var render_values := []
	var device_age_values := []
	var host_age_values := []
	var work_values := []
	var frame_age_values := []
	var held_age_values := []
	var debug_ids := _debug_camera_ids()
	for camera_id in debug_ids:
		var values := _camera_debug_values(camera_id)
		_set_debug_cell("%s_cap" % camera_id, _format_fps(float(values["cap"])))
		_set_debug_cell("%s_pub" % camera_id, _format_fps(float(values["pub"])))
		_set_debug_cell("%s_render" % camera_id, _format_fps(float(values["render"])))
		_set_debug_cell("%s_device_age" % camera_id, _format_device_age(values))
		_set_debug_cell("%s_host_age" % camera_id, _format_host_age(values))
		_set_debug_cell("%s_work" % camera_id, _format_work_age(values))
		_set_debug_cell("%s_frame" % camera_id, _format_ms(float(values["frame_age"])))
		_set_debug_cell("%s_held" % camera_id, _format_ms(float(values["held_age"])))
		_set_debug_cell("%s_points_tris" % camera_id, "%s/%s" % [_format_points(int(values["render_points"])), _format_points(int(values["render_tris"]))])
		total_render_points += int(values["render_points"])
		total_tris += int(values["render_tris"])
		cap_values.append(float(values["cap"]))
		pub_values.append(float(values["pub"]))
		render_values.append(float(values["render"]))
		device_age_values.append(float(values["sensor_age"]))
		device_age_values.append(float(values["color_age"]))
		host_age_values.append(float(values["sensor_host_age"]))
		host_age_values.append(float(values["color_host_age"]))
		work_values.append(float(values["work_age"]))
		frame_age_values.append(float(values["frame_age"]))
		held_age_values.append(float(values["held_age"]))
	_set_debug_cell("total_cap", _format_fps(_average_nonzero(cap_values)))
	_set_debug_cell("total_pub", _format_fps(_average_nonzero(pub_values)))
	_set_debug_cell("total_render", _format_fps(_average_nonzero(render_values)))
	_set_debug_cell("total_device_age", _format_ms(_max_nonzero(device_age_values)))
	_set_debug_cell("total_host_age", _format_ms(_max_nonzero(host_age_values)))
	_set_debug_cell("total_work", _format_ms(_max_nonzero(work_values)))
	_set_debug_cell("total_frame", _format_ms(_max_nonzero(frame_age_values)))
	_set_debug_cell("total_held", _format_ms(_max_nonzero(held_age_values)))
	_set_debug_cell("total_points_tris", "%s/%s" % [_format_points(total_render_points), _format_points(total_tris)])
	var summaries: Array[String] = []
	for camera_id in debug_ids:
		var values := _camera_debug_values(camera_id)
		summaries.append("%s %.1f/%.1f/%.1f" % [
			_camera_label(camera_id),
			float(values["cap"]),
			float(values["pub"]),
			float(values["render"]),
		])
	debug_text = ", ".join(summaries)

func _camera_debug_values(camera_id: String) -> Dictionary:
	var stats: Dictionary = _point_cloud_stats.get(_camera_stats_key(camera_id), {})
	var fast_timing: Dictionary = stats.get("fast_timing_ms", {})
	var work_age := 0.0
	var model_age := float(fast_timing.get("model", 0.0))
	for key in fast_timing.keys():
		work_age += float(fast_timing.get(key, 0.0))
	var native_render_fps := _native_stat(camera_id, "get_render_fps")
	var native_direct_capture_fps := 0.0
	if _is_realsense_camera(camera_id) and _realsense_direct_capture_active(camera_id):
		native_direct_capture_fps = _native_stat(camera_id, "get_direct_realsense_capture_fps")
		var direct_cap := native_direct_capture_fps if native_direct_capture_fps > 0.01 else native_render_fps
		return {
			"cap": direct_cap,
			"pub": direct_cap,
			"render": native_render_fps,
			"points": int(_native_stat(camera_id, "get_last_point_count")),
			"render_points": int(_native_stat(camera_id, "get_last_point_count")),
			"render_tris": int(_native_stat(camera_id, "get_last_triangle_count")),
			"held_age": float(_native_stat(camera_id, "get_display_frame_age_ms")),
			"sensor_age": 0.0,
			"color_age": 0.0,
			"sensor_host_age": 0.0,
			"color_host_age": 0.0,
			"work_age": 0.0,
			"model_age": 0.0,
			"frame_age": 0.0,
		}
	return {
		"cap": float(stats.get("capture_fps", 0.0)),
		"pub": float(stats.get("publish_fps", 0.0)),
		"render": native_render_fps,
		"points": int(stats.get("points", 0)),
		"render_points": int(_native_stat(camera_id, "get_last_point_count")),
		"render_tris": int(_native_stat(camera_id, "get_last_triangle_count")),
		"held_age": float(_native_stat(camera_id, "get_display_frame_age_ms")),
		"sensor_age": float(stats.get("sensor_age_ms", 0.0)),
		"color_age": float(stats.get("color_age_ms", 0.0)),
		"sensor_host_age": float(stats.get("sensor_host_age_ms", 0.0)),
		"color_host_age": float(stats.get("color_host_age_ms", 0.0)),
		"work_age": work_age,
		"model_age": model_age,
		"frame_age": float(stats.get("frame_age_ms", 0.0)),
	}

func _set_debug_cell(key: String, text: String) -> void:
	var label := _debug_cells.get(key) as Label
	if label != null:
		label.text = text

func _format_fps(value: float) -> String:
	return "--" if value <= 0.01 else "%.1f" % value

func _format_points(value: int) -> String:
	if value >= 1000000:
		return "%.1fM" % (float(value) / 1000000.0)
	if value >= 1000:
		return "%.0fk" % (float(value) / 1000.0)
	return str(value)

func _format_ms(value: float) -> String:
	return "--" if value <= 0.01 else "%.0f" % value

func _format_device_age(values: Dictionary) -> String:
	var sensor := float(values.get("sensor_age", 0.0))
	var color := float(values.get("color_age", 0.0))
	if sensor > 0.01 or color > 0.01:
		return "S%s C%s" % [_format_ms(sensor), _format_ms(color)]
	return "--"

func _format_host_age(values: Dictionary) -> String:
	var sensor := float(values.get("sensor_host_age", 0.0))
	var color := float(values.get("color_host_age", 0.0))
	if sensor > 0.01 or color > 0.01:
		return "S%s C%s" % [_format_ms(sensor), _format_ms(color)]
	return "--"

func _format_work_age(values: Dictionary) -> String:
	var model := float(values.get("model_age", 0.0))
	var total := float(values.get("work_age", 0.0))
	if model > 0.01 or total > 0.01:
		return "M%s T%s" % [_format_ms(model), _format_ms(total)]
	return "--"

func _average_nonzero(values: Array) -> float:
	var total := 0.0
	var count := 0
	for value in values:
		var f := float(value)
		if f > 0.01:
			total += f
			count += 1
	return total / max(1, count)

func _max_nonzero(values: Array) -> float:
	var result := 0.0
	for value in values:
		result = maxf(result, float(value))
	return result
