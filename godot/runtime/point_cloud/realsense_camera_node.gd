@tool
extends Node3D

@export_group("Identity")
@export var camera_id: String = ""
@export var serial: String = ""
@export var camera_label: String = ""
@export var model: String = ""

@export_group("Capture")
@export var enabled: bool = true:
	set(value):
		enabled = value
		_notify_settings_changed()
## viewer30: 848x480 depth + 1280x720 color at 30 Hz. fast60: 848x480 depth/color at 60 Hz. highres30: 1280x720 depth/color at 30 Hz.
@export_enum("viewer30", "fast60", "highres30") var stream_profile: String = "fast60":
	set(value):
		stream_profile = value if value in ["viewer30", "fast60", "highres30"] else "fast60"
		_notify_settings_changed()
@export_enum("sdk_depth", "fast_foundation_native") var depth_source: String = "sdk_depth":
	set(value):
		depth_source = value if value in ["sdk_depth", "fast_foundation_native"] else "sdk_depth"
		_notify_settings_changed()
@export_range(1, 8, 1) var stride: int = 1:
	set(value):
		stride = maxi(1, value)
		_notify_settings_changed()
@export var color_enabled: bool = true:
	set(value):
		color_enabled = value
		_notify_settings_changed()

@export_group("Color Match")
## Applies a lightweight per-channel affine correction after capture. Identity is gain 1 and bias 0.
@export var color_match_enabled: bool = false:
	set(value):
		color_match_enabled = value
		_notify_settings_changed()
@export var color_gain: Vector3 = Vector3.ONE:
	set(value):
		color_gain = Vector3(
			clampf(value.x, 0.5, 2.0),
			clampf(value.y, 0.5, 2.0),
			clampf(value.z, 0.5, 2.0)
		)
		_notify_settings_changed()
@export var color_bias: Vector3 = Vector3.ZERO:
	set(value):
		color_bias = Vector3(
			clampf(value.x, -0.25, 0.25),
			clampf(value.y, -0.25, 0.25),
			clampf(value.z, -0.25, 0.25)
		)
		_notify_settings_changed()
@export_tool_button("Reset Color Match") var reset_color_match_action: Callable = reset_color_match

@export_group("Infrared Projector")
## Keep only one overlapping camera projector enabled unless the cameras are hardware synchronized.
@export var emitter_enabled: bool = true:
	set(value):
		emitter_enabled = value
		_notify_settings_changed()
@export_range(0.0, 100.0, 1.0, "suffix:%") var laser_power_percent: float = 100.0:
	set(value):
		laser_power_percent = clampf(value, 0.0, 100.0)
		_notify_settings_changed()

@export_group("Workspace Crop")
## Image-space crop applied before rendering, picking, and calibration point sampling.
@export var workspace_crop_enabled: bool = false:
	set(value):
		workspace_crop_enabled = value
		_notify_settings_changed()
@export_range(0.0, 45.0, 1.0, "suffix:%") var crop_left_percent: float = 0.0:
	set(value):
		crop_left_percent = clampf(value, 0.0, 45.0)
		_notify_settings_changed()
@export_range(0.0, 45.0, 1.0, "suffix:%") var crop_right_percent: float = 0.0:
	set(value):
		crop_right_percent = clampf(value, 0.0, 45.0)
		_notify_settings_changed()
@export_range(0.0, 45.0, 1.0, "suffix:%") var crop_top_percent: float = 0.0:
	set(value):
		crop_top_percent = clampf(value, 0.0, 45.0)
		_notify_settings_changed()
@export_range(0.0, 45.0, 1.0, "suffix:%") var crop_bottom_percent: float = 0.0:
	set(value):
		crop_bottom_percent = clampf(value, 0.0, 45.0)
		_notify_settings_changed()

@export_group("Maximum Depth")
@export var max_depth_enabled: bool = false:
	set(value):
		max_depth_enabled = value
		use_custom_depth_range = value
		_notify_settings_changed()
@export_range(0.25, 10.0, 0.05, "suffix:m") var max_depth_m: float = 4.50:
	set(value):
		max_depth_m = maxf(value, 0.25)
		if not _suppress_notifications:
			max_depth_enabled = true
		_notify_settings_changed()

# Retained in saved scenes/settings for compatibility; the per-camera UI only
# exposes a far cutoff. The universal near cutoff remains unchanged.
@export_storage var use_custom_depth_range: bool = false
@export_storage var min_depth_m: float = 0.20

@export_group("Depth Filters")
@export var depth_filters_enabled: bool = false:
	set(value):
		depth_filters_enabled = value
		_notify_settings_changed()
@export var filters_for_geometry: bool = false:
	set(value):
		filters_for_geometry = value
		_notify_settings_changed()
@export_range(0.0, 0.30, 0.005, "suffix:m") var geometry_edge_guard_m: float = 0.04:
	set(value):
		geometry_edge_guard_m = maxf(0.0, value)
		_notify_settings_changed()
@export var decimation_filter_enabled: bool = true:
	set(value):
		decimation_filter_enabled = value
		_notify_settings_changed()
@export_range(2, 8, 1) var decimation_magnitude: int = 2:
	set(value):
		decimation_magnitude = clampi(value, 2, 8)
		_notify_settings_changed()
@export var rotation_filter_enabled: bool = false:
	set(value):
		rotation_filter_enabled = value
		_notify_settings_changed()
@export var hdr_merge_filter_enabled: bool = true:
	set(value):
		hdr_merge_filter_enabled = value
		_notify_settings_changed()
@export var sequence_id_filter_enabled: bool = false:
	set(value):
		sequence_id_filter_enabled = value
		_notify_settings_changed()
@export var threshold_filter_enabled: bool = false:
	set(value):
		threshold_filter_enabled = value
		_notify_settings_changed()
@export var depth_to_disparity_filter_enabled: bool = true:
	set(value):
		depth_to_disparity_filter_enabled = value
		_notify_settings_changed()
@export var spatial_filter_enabled: bool = true:
	set(value):
		spatial_filter_enabled = value
		_notify_settings_changed()
@export var temporal_filter_enabled: bool = true:
	set(value):
		temporal_filter_enabled = value
		_notify_settings_changed()
@export var hole_filling_filter_enabled: bool = false:
	set(value):
		hole_filling_filter_enabled = value
		_notify_settings_changed()
@export var disparity_to_depth_filter_enabled: bool = true:
	set(value):
		disparity_to_depth_filter_enabled = value
		_notify_settings_changed()
@export_range(0, 2, 1) var hole_filling: int = 1:
	set(value):
		hole_filling = clampi(value, 0, 2)
		_notify_settings_changed()

@export_group("Stabilization")
@export var stabilization_enabled: bool = false:
	set(value):
		stabilization_enabled = value
		_notify_settings_changed()
@export_range(0.0, 0.06, 0.001, "suffix:m") var stabilization_deadband_m: float = 0.012:
	set(value):
		stabilization_deadband_m = maxf(0.0, value)
		_notify_settings_changed()
@export_range(0, 4, 1) var stabilization_hold_frames: int = 1:
	set(value):
		stabilization_hold_frames = maxi(0, value)
		_notify_settings_changed()

@export_group("FastFoundation")
@export_enum("onnx_cuda", "onnx_trt", "pytorch", "trt_engine") var fast_backend: String = "onnx_cuda":
	set(value):
		fast_backend = value if value in ["onnx_cuda", "onnx_trt", "pytorch", "trt_engine"] else "onnx_cuda"
		_notify_settings_changed()
@export_enum("fast_192x384_i2", "rt_256x512_i2", "full_320x736_i4") var fast_profile: String = "fast_192x384_i2":
	set(value):
		fast_profile = value if value in ["fast_192x384_i2", "rt_256x512_i2", "full_320x736_i4"] else "fast_192x384_i2"
		_notify_settings_changed()
@export_range(1, 32, 1) var fast_iters: int = 4:
	set(value):
		fast_iters = clampi(value, 1, 32)
		_notify_settings_changed()
@export_range(0.25, 1.0, 0.05) var fast_scale: float = 0.5:
	set(value):
		fast_scale = clampf(value, 0.25, 1.0)
		_notify_settings_changed()

var _suppress_notifications := false

func configure(next_camera_id: String, next_serial: String, next_label: String, info: Dictionary, defaults: Dictionary, apply_defaults: bool) -> void:
	_suppress_notifications = true
	camera_id = next_camera_id
	serial = next_serial
	camera_label = next_label
	model = str(info.get("model", defaults.get("model", model)))
	name = _display_node_name(next_label, next_serial)
	if apply_defaults:
		apply_settings(defaults)
	_suppress_notifications = false

func apply_settings(settings: Dictionary) -> void:
	var notifications_were_suppressed := _suppress_notifications
	_suppress_notifications = true
	enabled = bool(settings.get("enabled", enabled))
	stream_profile = str(settings.get("stream_profile", stream_profile))
	depth_source = str(settings.get("depth_source", depth_source))
	stride = int(settings.get("stride", stride))
	color_enabled = bool(settings.get("color_enabled", color_enabled))
	color_match_enabled = bool(settings.get("color_match_enabled", color_match_enabled))
	color_gain = settings.get("color_gain", color_gain) as Vector3
	color_bias = settings.get("color_bias", color_bias) as Vector3
	emitter_enabled = bool(settings.get("emitter_enabled", emitter_enabled))
	laser_power_percent = float(settings.get("laser_power_percent", laser_power_percent))
	workspace_crop_enabled = bool(settings.get("workspace_crop_enabled", workspace_crop_enabled))
	crop_left_percent = float(settings.get("crop_left_percent", crop_left_percent))
	crop_right_percent = float(settings.get("crop_right_percent", crop_right_percent))
	crop_top_percent = float(settings.get("crop_top_percent", crop_top_percent))
	crop_bottom_percent = float(settings.get("crop_bottom_percent", crop_bottom_percent))
	max_depth_enabled = bool(settings.get("max_depth_enabled", settings.get("use_custom_depth_range", max_depth_enabled)))
	use_custom_depth_range = max_depth_enabled
	min_depth_m = float(settings.get("min_depth_m", min_depth_m))
	max_depth_m = float(settings.get("max_depth_m", max_depth_m))
	depth_filters_enabled = bool(settings.get("depth_filters_enabled", depth_filters_enabled))
	decimation_filter_enabled = bool(settings.get("decimation_filter_enabled", decimation_filter_enabled))
	decimation_magnitude = int(settings.get("decimation_magnitude", decimation_magnitude))
	rotation_filter_enabled = bool(settings.get("rotation_filter_enabled", rotation_filter_enabled))
	hdr_merge_filter_enabled = bool(settings.get("hdr_merge_filter_enabled", hdr_merge_filter_enabled))
	sequence_id_filter_enabled = bool(settings.get("sequence_id_filter_enabled", sequence_id_filter_enabled))
	threshold_filter_enabled = bool(settings.get("threshold_filter_enabled", threshold_filter_enabled))
	depth_to_disparity_filter_enabled = bool(settings.get("depth_to_disparity_filter_enabled", depth_to_disparity_filter_enabled))
	spatial_filter_enabled = bool(settings.get("spatial_filter_enabled", spatial_filter_enabled))
	temporal_filter_enabled = bool(settings.get("temporal_filter_enabled", temporal_filter_enabled))
	hole_filling_filter_enabled = bool(settings.get("hole_filling_filter_enabled", hole_filling_filter_enabled))
	disparity_to_depth_filter_enabled = bool(settings.get("disparity_to_depth_filter_enabled", disparity_to_depth_filter_enabled))
	filters_for_geometry = bool(settings.get("filters_for_geometry", filters_for_geometry))
	geometry_edge_guard_m = float(settings.get("geometry_edge_guard_m", geometry_edge_guard_m))
	hole_filling = int(settings.get("hole_filling", hole_filling))
	stabilization_enabled = bool(settings.get("stabilization_enabled", stabilization_enabled))
	stabilization_deadband_m = float(settings.get("stabilization_deadband_m", stabilization_deadband_m))
	stabilization_hold_frames = int(settings.get("stabilization_hold_frames", stabilization_hold_frames))
	fast_backend = str(settings.get("fast_backend", fast_backend))
	fast_profile = str(settings.get("fast_profile", fast_profile))
	fast_iters = int(settings.get("fast_iters", fast_iters))
	fast_scale = float(settings.get("fast_scale", fast_scale))
	_suppress_notifications = notifications_were_suppressed
	if not notifications_were_suppressed:
		_notify_settings_changed()

func get_settings() -> Dictionary:
	return {
		"serial": serial,
		"name": camera_label,
		"model": model,
		"enabled": enabled,
		"stream_profile": stream_profile,
		"depth_source": depth_source,
		"stride": stride,
		"color_enabled": color_enabled,
		"color_match_enabled": color_match_enabled,
		"color_gain": color_gain,
		"color_bias": color_bias,
		"emitter_enabled": emitter_enabled,
		"laser_power_percent": laser_power_percent,
		"workspace_crop_enabled": workspace_crop_enabled,
		"crop_left_percent": crop_left_percent,
		"crop_right_percent": crop_right_percent,
		"crop_top_percent": crop_top_percent,
		"crop_bottom_percent": crop_bottom_percent,
		"max_depth_enabled": max_depth_enabled,
		"use_custom_depth_range": max_depth_enabled,
		"min_depth_m": min_depth_m,
		"max_depth_m": max_depth_m,
		"depth_filters_enabled": depth_filters_enabled,
		"decimation_filter_enabled": decimation_filter_enabled,
		"decimation_magnitude": decimation_magnitude,
		"rotation_filter_enabled": rotation_filter_enabled,
		"hdr_merge_filter_enabled": hdr_merge_filter_enabled,
		"sequence_id_filter_enabled": sequence_id_filter_enabled,
		"threshold_filter_enabled": threshold_filter_enabled,
		"depth_to_disparity_filter_enabled": depth_to_disparity_filter_enabled,
		"spatial_filter_enabled": spatial_filter_enabled,
		"temporal_filter_enabled": temporal_filter_enabled,
		"hole_filling_filter_enabled": hole_filling_filter_enabled,
		"disparity_to_depth_filter_enabled": disparity_to_depth_filter_enabled,
		"filters_for_geometry": filters_for_geometry,
		"geometry_edge_guard_m": geometry_edge_guard_m,
		"hole_filling": hole_filling,
		"stabilization_enabled": stabilization_enabled,
		"stabilization_deadband_m": stabilization_deadband_m,
		"stabilization_hold_frames": stabilization_hold_frames,
		"fast_backend": fast_backend,
		"fast_profile": fast_profile,
		"fast_iters": fast_iters,
		"fast_scale": fast_scale,
	}

func reset_color_match() -> void:
	_suppress_notifications = true
	color_match_enabled = false
	color_gain = Vector3.ONE
	color_bias = Vector3.ZERO
	_suppress_notifications = false
	_notify_settings_changed()

func _notify_settings_changed() -> void:
	if _suppress_notifications or camera_id.is_empty():
		return
	var node := get_parent()
	while node != null:
		if node.has_method("_on_realsense_camera_settings_changed"):
			node.call_deferred("_on_realsense_camera_settings_changed", camera_id)
			return
		node = node.get_parent()

func _display_node_name(label: String, serial_value: String) -> String:
	var display := label.strip_edges()
	if display.is_empty():
		display = "RealSense"
	if not serial_value.is_empty() and display.find(serial_value) < 0:
		display = "%s %s" % [display, serial_value]
	return display
