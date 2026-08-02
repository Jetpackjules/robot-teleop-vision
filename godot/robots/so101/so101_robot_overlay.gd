@tool
extends Node3D

const MODEL_ROOT_NAME := "SO101ModelRoot"
const TARGET_GHOST_ROOT_NAME := "SO101TargetGhostRoot"
const REGISTRATION_PATH := "user://so101_robot_registration.json"
# Keep the legacy filename so existing depth-validated anchors remain usable as
# the initial user checkpoint after this feature's rename.
const ROBOT_POSITION_CHECKPOINT_PATH := "user://so101_robot_registration.depth_validated_anchor.json"
var _registration_path := REGISTRATION_PATH
var _robot_position_checkpoint_path := ROBOT_POSITION_CHECKPOINT_PATH
const ROBOT_KINEMATICS_VERSION := 10
const BASE_TELEMETRY_DIRECTION := 1.0
# Measured with the physical shoulder pan aligned straight ahead. The servo's
# normalized encoder reports -40.4296875 degrees at that pose.
const BASE_TELEMETRY_OFFSET_DEGREES := 40.4296875
const ROS_TO_GODOT := Basis(
	Vector3(0.0, 0.0, -1.0),
	Vector3(-1.0, 0.0, 0.0),
	Vector3(0.0, 1.0, 0.0)
)
const LINK_MODELS := {
	"base_link": "res://assets/robots/so101/base_link.glb",
	"shoulder_link": "res://assets/robots/so101/shoulder_link.glb",
	"upper_arm_link": "res://assets/robots/so101/upper_arm_link.glb",
	"lower_arm_link": "res://assets/robots/so101/lower_arm_link.glb",
	"wrist_link": "res://assets/robots/so101/wrist_link.glb",
	"gripper_link": "res://assets/robots/so101/gripper_link.glb",
	"moving_jaw_so101_v1_link": "res://assets/robots/so101/moving_jaw_so101_v1_link.glb",
}
const GRIPPER_ANGLE_OFFSET_DEFAULT_DEGREES := -16.17708418
const GRIPPER_ANGLE_SCALE_DEFAULT_DEGREES := 1.14931457
const GRIPPER_ANGLE_CURVATURE_DEFAULT_DEGREES := 0.15235159
const GRIPPER_CLOSED_ENDPOINT_TOLERANCE_DEGREES := 5.0
# The unmodelled wrist-camera bracket previously contaminated an image-only fit
# and produced a 31 mm gripper-frame translation.  The exposed D455 tip check
# supports only a small assembly correction, so reject another large jump.
const GRIPPER_MOUNT_CORRECTION_MAXIMUM_METERS := 0.015
const GRIPPER_VISUAL_CORRECTION_MAXIMUM_METERS := 0.025
const GRIPPER_VISUAL_CORRECTION_MAXIMUM_DEGREES := 30.0
# The D455/world alignment is gravity-up. A table-mounted SO-101 can change X/Z
# placement and heading, but its base axis may never point into the table.
const MINIMUM_REGISTRATION_WORLD_UP_DOT := 0.25
const JOINTS := [
	{
		"name": "shoulder_pan",
		"parent": "base_link",
		"child": "shoulder_link",
		"xyz": Vector3(0.0388353, -0.000000009, 0.0624),
		"rpy": Vector3(PI, 0.0, -PI),
		"limits": Vector2(-1.91986, 1.91986),
	},
	{
		"name": "shoulder_lift",
		"parent": "shoulder_link",
		"child": "upper_arm_link",
		"xyz": Vector3(-0.0303992, -0.0182778, -0.0542),
		"rpy": Vector3(-PI * 0.5, -PI * 0.5, 0.0),
		"limits": Vector2(-1.74533, 1.74533),
	},
	{
		"name": "elbow_flex",
		"parent": "upper_arm_link",
		"child": "lower_arm_link",
		"xyz": Vector3(-0.11257, -0.028, 0.0),
		"rpy": Vector3(0.0, 0.0, PI * 0.5),
		"limits": Vector2(-1.69, 1.69),
	},
	{
		"name": "wrist_flex",
		"parent": "lower_arm_link",
		"child": "wrist_link",
		"xyz": Vector3(-0.1349, 0.0052, 0.0),
		"rpy": Vector3(0.0, 0.0, -PI * 0.5),
		"limits": Vector2(-PI, PI),
	},
	{
		"name": "wrist_roll",
		"parent": "wrist_link",
		"child": "gripper_link",
		"xyz": Vector3(0.0, -0.0611, 0.0181),
		"rpy": Vector3(PI * 0.5, 0.0486795, PI),
		"limits": Vector2(-2.74385, 2.84121),
	},
	{
		"name": "gripper",
		"parent": "gripper_link",
		"child": "moving_jaw_so101_v1_link",
		"xyz": Vector3(0.0202, 0.0188, -0.0234),
		"rpy": Vector3(PI * 0.5, -0.000000052, 0.0),
		# Stock SO-101 joint limits. The earlier -20-degree visual workaround
		# hid a rigid gripper-frame tilt and distorted every intermediate open
		# state; the multi-state D455 fit keeps the official -10-degree stop.
		"limits": Vector2(-0.174533, 1.74533),
	},
]

@export_group("Robot Overlay")
@export var overlay_enabled: bool = false:
	set(value):
		overlay_enabled = value
		visible = value
## Makes the telemetry model see-through for visually comparing its links and
## claw against the live point cloud. Zero is the normal solid rendering.
@export_range(0.0, 0.95, 0.05) var overlay_transparency: float = 0.0:
	set(value):
		overlay_transparency = clampf(value, 0.0, 0.95)
		_apply_overlay_transparency()
@export var mask_scanned_robot: bool = true
@export var load_saved_registration: bool = true
@export_range(0.25, 2.0, 0.01) var model_scale: float = 1.0:
	set(value):
		model_scale = value
		_apply_model_scale()
# Encoder feedback arrives at a bounded 10 Hz so it cannot starve commands on
# the serial servo bus. Spread each measured correction over most of the 100 ms
# sample interval, avoiding visible pose steps without substituting commanded
# angles for the physical measurement.
@export_range(0.0, 1.0, 0.01) var telemetry_smoothing: float = 0.52
@export var telemetry_port: int = 4250
@export var editor_telemetry_port: int = 4252
@export var joint_angle_directions := PackedFloat32Array([1.0, -1.0, 1.0, 1.0, 1.0, 1.0])
# The old -1 direction and 17.40234375-degree offset agreed only at the
# marker-ground-truth capture pose. The follower telemetry itself is +1.
@export var joint_angle_offsets_degrees := PackedFloat32Array([40.4296875, 80.0, 0.0, -70.0, 0.0, 0.0])
## Command limits protect motion, but clamping encoder-derived visualization to
## nominal URDF limits can freeze a link while its real servo continues moving
## after its zero is calibrated. Leave visual telemetry unbounded and wrapped.
@export var clamp_visual_joint_limits: bool = false
## Diagnostic override only. Production calibration must follow the measured
## wrist-roll encoder; forcing this straight creates a centimetre-scale claw-tip
## error whenever the physical wrist is rolled.
@export var force_wrist_roll_straight: bool = false
@export_range(0.0, 20.0, 0.1) var gripper_closed_normalized: float = 2.5
## Empirical linkage map from normalized encoder value to moving-jaw angle.
## Curvature captures the small nonlinearity visible across the settled D455
## closed/25/50/75/open sweep without changing the physical mesh dimensions.
@export var gripper_angle_offset_degrees: float = GRIPPER_ANGLE_OFFSET_DEFAULT_DEGREES
@export var gripper_angle_scale_degrees_per_normalized: float = GRIPPER_ANGLE_SCALE_DEFAULT_DEGREES
@export var gripper_angle_curvature_degrees: float = GRIPPER_ANGLE_CURVATURE_DEFAULT_DEGREES
## Settled physical encoder/angle correspondences from the terminal-finger-only
## temporal fit. Piecewise interpolation follows the linkage more accurately
## than forcing its visibly nonlinear closed end into a single polynomial.
@export var gripper_angle_samples_normalized := PackedFloat32Array([
	4.05932865,
	24.43397713,
	49.41446199,
	74.62915576,
	88.29039856,
])
@export var gripper_angle_samples_degrees := PackedFloat32Array([
	-10.0,
	15.5,
	41.5,
	66.25,
	79.5,
])
## Small rigid correction at the wrist-roll child frame.  This calibrates the
## real printed gripper mount without moving or scaling the upstream links.
@export var gripper_mount_correction_local := Vector3.ZERO
## D455-validated post-joint rigid correction shared by the fixed gripper and
## moving jaw. This is deliberately separate from the legacy parent-axis
## translation so existing base and upstream joint calibration remains intact.
@export var gripper_visual_correction_rpy_degrees := Vector3.ZERO
@export var gripper_visual_correction_translation_local := Vector3.ZERO
## The installed wrist-roll cradle is mounted 180 degrees around the roll
## servo axis relative to the published mesh convention. Correct only the
## wrist housing visual about the real output-axis pivot; upstream links and
## the downstream claw kinematics remain rigid and unscaled.
@export var wrist_housing_mount_flipped: bool = true:
	set(value):
		wrist_housing_mount_flipped = value
		_apply_wrist_housing_mount_visual()
## Native-D455 multiview calibration for physical grippers whose printed jaw
## pivot differs from the stock URDF. This overrides only the moving jaw; the
## fixed gripper and every upstream link keep their normal calibrated chain.
@export var moving_jaw_calibration_enabled: bool = false
@export var moving_jaw_pivot_parent := Vector3(0.0202, 0.0188, -0.0234)
@export var moving_jaw_axis_parent := Vector3(0.0, -1.0, 0.0)
@export var moving_jaw_closed_basis_parent := Basis.IDENTITY
@export var moving_jaw_opening_samples_degrees := PackedFloat32Array([
	0.0, 25.5, 51.5, 76.25, 89.5,
])
@export_range(0.75, 1.35, 0.001) var moving_jaw_visual_radial_scale := 1.0
@export_range(-0.03, 0.03, 0.0001) var moving_jaw_visual_axial_translation_local := 0.0
@export_group("Manual Wrist Roll Alignment")
## Visual-only trim for identifying the physical wrist-roll zero in the editor.
## It rotates the complete rigid claw assembly and never commands the arm.
@export_range(0.0, 360.0, 0.5, "suffix:deg") var wrist_roll_alignment_preview_degrees: float = 0.0:
	set(value):
		wrist_roll_alignment_preview_degrees = clampf(value, 0.0, 360.0)
		_refresh_wrist_roll_alignment_preview()
## Converts the visual preview into the saved wrist-roll encoder offset. The
## effective displayed pose does not jump when the preview returns to zero.
@export_tool_button("Save Wrist Rotation Alignment") var save_wrist_rotation_alignment_action: Callable = _save_wrist_rotation_alignment
@export_tool_button("Reset Wrist Rotation Preview") var reset_wrist_rotation_preview_action: Callable = reset_wrist_roll_alignment_preview
@export var preview_pose_degrees := PackedFloat32Array([0.0, 0.0, 0.0, 0.0, 0.0, 25.0])
@export_group("Manual Claw Calibration")
## Website-driven, visualization-only calibration. These values are deltas on
## top of the saved registration and never command a physical servo.
var manual_claw_calibration_enabled: bool = false
var manual_wrist_flex_trim_degrees: float = 0.0
var manual_wrist_roll_trim_degrees: float = 0.0
var manual_wrist_roll_direction: float = 1.0
var manual_tool_rpy_degrees := Vector3.ZERO
var manual_tool_translation_local := Vector3.ZERO
var manual_opening_offset_degrees: float = 0.0
var manual_opening_scale: float = 1.0
@export_group("Measured Feedback")
@export var measured_feedback_enabled: bool = true
@export var target_ghost_enabled: bool = true
@export var freeze_overlay_on_stale_enabled: bool = true
@export var d455_visual_correction_enabled: bool = false

var _model_root: Node3D
var _links: Dictionary = {}
var _link_surface_points: Dictionary = {}
var _link_surface_points_maximum_per_link := 0
var _joint_nodes: Array[Node3D] = []
var _joint_origin_transforms: Array[Transform3D] = []
var _target_angles := PackedFloat32Array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
var _display_angles := PackedFloat32Array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
var _target_ghost_root: Node3D
var _target_ghost_links: Dictionary = {}
var _target_ghost_joint_nodes: Array[Node3D] = []
var _target_ghost_joint_origins: Array[Transform3D] = []
var _udp := PacketPeerUDP.new()
var _udp_bound := false
var _last_status: Dictionary = {}
var _registration_status: Dictionary = {}
var _registration_modified_time: int = 0
var _next_registration_poll_msec: int = 0
var _scene_fallback_transform := Transform3D.IDENTITY
var _scene_fallback_mapping: Dictionary = {}
var _d455_distal_correction_local := Vector3.ZERO
var _d455_distal_correction_confidence := 0.0
var _d455_distal_correction_msec := 0

func _ready() -> void:
	add_to_group("so101_robot_overlay")
	_capture_scene_fallback()
	_build_model()
	_build_target_ghost()
	visible = overlay_enabled
	if load_saved_registration:
		_load_registration()
	_registration_modified_time = _registration_file_modified_time()
	if Engine.is_editor_hint():
		_set_target_from_normalized(preview_pose_degrees)
	var selected_port := editor_telemetry_port if Engine.is_editor_hint() else telemetry_port
	_udp_bound = _udp.bind(selected_port, "127.0.0.1") == OK
	if not _udp_bound:
		push_warning("SO-101 overlay could not bind telemetry UDP %d" % selected_port)
	set_process(true)

func _exit_tree() -> void:
	if _udp_bound:
		_udp.close()
	_udp_bound = false

func _process(delta: float) -> void:
	_poll_saved_registration()
	_poll_telemetry()
	var blend := 1.0 - pow(clampf(telemetry_smoothing, 0.0, 0.98), maxf(delta, 0.0001) * 60.0)
	for index in range(_joint_nodes.size()):
		_display_angles[index] = lerp_angle(_display_angles[index], _target_angles[index], blend)
		_apply_joint_angle(index, _display_angles[index])

func set_overlay_enabled(enabled: bool) -> void:
	overlay_enabled = enabled
	_update_target_ghost_visibility()

func set_overlay_transparency(value: float) -> void:
	overlay_transparency = value

func set_mask_scanned_robot(enabled: bool) -> void:
	mask_scanned_robot = enabled

func configure_feedback_features(settings: Dictionary) -> void:
	measured_feedback_enabled = bool(settings.get(
		"measured_feedback_enabled",
		measured_feedback_enabled,
	))
	target_ghost_enabled = bool(settings.get(
		"target_ghost_enabled",
		target_ghost_enabled,
	))
	freeze_overlay_on_stale_enabled = bool(settings.get(
		"freeze_overlay_on_stale_enabled",
		freeze_overlay_on_stale_enabled,
	))
	d455_visual_correction_enabled = bool(settings.get(
		"d455_visual_correction_enabled",
		d455_visual_correction_enabled,
	))
	_update_target_ghost_visibility()


func begin_manual_claw_calibration() -> void:
	manual_claw_calibration_enabled = true
	manual_wrist_flex_trim_degrees = 0.0
	manual_wrist_roll_trim_degrees = 0.0
	manual_wrist_roll_direction = _joint_direction(4)
	manual_tool_rpy_degrees = Vector3.ZERO
	manual_tool_translation_local = Vector3.ZERO
	manual_opening_offset_degrees = 0.0
	manual_opening_scale = 1.0
	_refresh_manual_claw_calibration()


func update_manual_claw_calibration(settings: Dictionary) -> void:
	if not manual_claw_calibration_enabled:
		begin_manual_claw_calibration()
	manual_wrist_flex_trim_degrees = clampf(float(settings.get(
		"manual_wrist_flex_trim_degrees", manual_wrist_flex_trim_degrees
	)), -90.0, 90.0)
	manual_wrist_roll_trim_degrees = clampf(float(settings.get(
		"manual_wrist_roll_trim_degrees", manual_wrist_roll_trim_degrees
	)), -180.0, 180.0)
	var requested_direction := float(settings.get(
		"manual_wrist_roll_direction", manual_wrist_roll_direction
	))
	if absf(requested_direction) >= 0.5:
		manual_wrist_roll_direction = -1.0 if requested_direction < 0.0 else 1.0
	manual_tool_translation_local = Vector3(
		clampf(float(settings.get("manual_tool_x", manual_tool_translation_local.x)), -0.05, 0.05),
		clampf(float(settings.get("manual_tool_y", manual_tool_translation_local.y)), -0.05, 0.05),
		clampf(float(settings.get("manual_tool_z", manual_tool_translation_local.z)), -0.05, 0.05),
	)
	manual_tool_rpy_degrees = Vector3(
		clampf(float(settings.get("manual_tool_roll", manual_tool_rpy_degrees.x)), -180.0, 180.0),
		clampf(float(settings.get("manual_tool_pitch", manual_tool_rpy_degrees.y)), -180.0, 180.0),
		clampf(float(settings.get("manual_tool_yaw", manual_tool_rpy_degrees.z)), -180.0, 180.0),
	)
	manual_opening_offset_degrees = clampf(float(settings.get(
		"manual_opening_offset_degrees", manual_opening_offset_degrees
	)), -35.0, 35.0)
	manual_opening_scale = clampf(float(settings.get(
		"manual_opening_scale", manual_opening_scale
	)), 0.5, 1.5)
	_refresh_manual_claw_calibration()


func reset_manual_claw_calibration() -> void:
	begin_manual_claw_calibration()


func cancel_manual_claw_calibration() -> void:
	manual_claw_calibration_enabled = false
	manual_wrist_flex_trim_degrees = 0.0
	manual_wrist_roll_trim_degrees = 0.0
	manual_tool_rpy_degrees = Vector3.ZERO
	manual_tool_translation_local = Vector3.ZERO
	manual_opening_offset_degrees = 0.0
	manual_opening_scale = 1.0
	_refresh_manual_claw_calibration()


func save_manual_claw_calibration() -> bool:
	if (
		not manual_claw_calibration_enabled
		or joint_angle_offsets_degrees.size() < 5
		or joint_angle_directions.size() < 5
	):
		return false
	var manual_state := get_manual_claw_calibration_state().duplicate(true)
	var previous_directions := joint_angle_directions.duplicate()
	var previous_offsets := joint_angle_offsets_degrees.duplicate()
	var previous_rpy := gripper_visual_correction_rpy_degrees
	var previous_translation := gripper_visual_correction_translation_local
	var previous_gripper_angles := gripper_angle_samples_degrees.duplicate()
	var previous_jaw_angles := moving_jaw_opening_samples_degrees.duplicate()
	var previous_status := _registration_status.duplicate(true)
	joint_angle_offsets_degrees[3] += manual_wrist_flex_trim_degrees
	joint_angle_directions[4] = manual_wrist_roll_direction
	joint_angle_offsets_degrees[4] = wrapf(
		joint_angle_offsets_degrees[4] + manual_wrist_roll_trim_degrees + 180.0,
		0.0, 360.0
	) - 180.0
	gripper_visual_correction_rpy_degrees += manual_tool_rpy_degrees
	gripper_visual_correction_translation_local += manual_tool_translation_local
	for index in range(gripper_angle_samples_degrees.size()):
		gripper_angle_samples_degrees[index] = (
			float(gripper_angle_samples_degrees[index]) * manual_opening_scale
			+ manual_opening_offset_degrees
		)
	for index in range(moving_jaw_opening_samples_degrees.size()):
		moving_jaw_opening_samples_degrees[index] = (
			float(moving_jaw_opening_samples_degrees[index]) * manual_opening_scale
			+ manual_opening_offset_degrees
		)
	_registration_status["registered"] = true
	_registration_status["saved_unix_ms"] = Time.get_unix_time_from_system() * 1000.0
	_registration_status["manual_claw_calibration"] = manual_state
	var existing_refinement = _registration_status.get("joint_refinement", {})
	var refinement := (
		(existing_refinement as Dictionary).duplicate(true)
		if existing_refinement is Dictionary
		else {}
	)
	manual_state["method"] = "user_visual_d455_rgbd_rigid_claw_fit"
	manual_state["saved_unix_ms"] = _registration_status["saved_unix_ms"]
	refinement["manual_claw_calibration"] = manual_state
	_registration_status["joint_refinement"] = refinement
	manual_claw_calibration_enabled = false
	if _save_registration():
		cancel_manual_claw_calibration()
		return true
	joint_angle_directions = previous_directions
	joint_angle_offsets_degrees = previous_offsets
	gripper_visual_correction_rpy_degrees = previous_rpy
	gripper_visual_correction_translation_local = previous_translation
	gripper_angle_samples_degrees = previous_gripper_angles
	moving_jaw_opening_samples_degrees = previous_jaw_angles
	_registration_status = previous_status
	manual_claw_calibration_enabled = true
	_refresh_manual_claw_calibration()
	return false


func get_manual_claw_calibration_state() -> Dictionary:
	return {
		"enabled": manual_claw_calibration_enabled,
		"wrist_flex_trim_degrees": manual_wrist_flex_trim_degrees,
		"wrist_roll_trim_degrees": manual_wrist_roll_trim_degrees,
		"wrist_roll_direction": manual_wrist_roll_direction,
		"tool_translation_local": [manual_tool_translation_local.x, manual_tool_translation_local.y, manual_tool_translation_local.z],
		"tool_rpy_degrees": [manual_tool_rpy_degrees.x, manual_tool_rpy_degrees.y, manual_tool_rpy_degrees.z],
		"opening_offset_degrees": manual_opening_offset_degrees,
		"opening_scale": manual_opening_scale,
	}


func _refresh_manual_claw_calibration() -> void:
	if not is_inside_tree():
		return
	var values := get_best_available_normalized_pose()
	if values.size() < 6 and preview_pose_degrees.size() >= 6:
		values = Array(preview_pose_degrees)
	if values.size() >= 6:
		_set_target_from_normalized(values)
		for index in range(_joint_nodes.size()):
			_display_angles[index] = _target_angles[index]
			_apply_joint_angle(index, _display_angles[index])


func set_wrist_roll_alignment_preview(value_degrees: float) -> void:
	wrist_roll_alignment_preview_degrees = value_degrees


func reset_wrist_roll_alignment_preview() -> void:
	wrist_roll_alignment_preview_degrees = 0.0


func _save_wrist_rotation_alignment() -> void:
	if not commit_wrist_roll_alignment_preview():
		push_error("Could not save the manual SO-101 wrist rotation alignment.")


func commit_wrist_roll_alignment_preview() -> bool:
	var preview_degrees := _signed_wrist_roll_preview_degrees()
	if absf(preview_degrees) < 0.001:
		return true
	if joint_angle_offsets_degrees.size() <= 4:
		return false
	var previous_offset := float(joint_angle_offsets_degrees[4])
	var previous_force_straight := force_wrist_roll_straight
	var previous_status := _registration_status.duplicate(true)
	var previous_preview := wrist_roll_alignment_preview_degrees
	var saved_offset := wrapf(
		previous_offset + preview_degrees + 180.0,
		0.0,
		360.0,
	) - 180.0
	joint_angle_offsets_degrees[4] = saved_offset
	force_wrist_roll_straight = false
	wrist_roll_alignment_preview_degrees = 0.0
	_registration_status["registered"] = true
	_registration_status["saved_unix_ms"] = (
		Time.get_unix_time_from_system() * 1000.0
	)
	var existing := _read_registration_payload()
	var existing_refinement = _registration_status.get(
		"joint_refinement",
		existing.get("joint_refinement", {}),
	)
	var refinement := (
		(existing_refinement as Dictionary).duplicate(true)
		if existing_refinement is Dictionary
		else {}
	)
	refinement["wrist_roll_manual_alignment"] = {
		"method": "user_visual_rigid_claw_roll_trim",
		"previous_offset_degrees": previous_offset,
		"preview_correction_degrees": preview_degrees,
		"saved_offset_degrees": saved_offset,
		"saved_unix_ms": _registration_status["saved_unix_ms"],
	}
	_registration_status["joint_refinement"] = refinement
	_refresh_wrist_roll_alignment_preview()
	if _save_registration():
		print(
			"Saved SO-101 wrist rotation alignment: %.1f deg preview, %.1f deg offset"
			% [preview_degrees, saved_offset]
		)
		return true
	joint_angle_offsets_degrees[4] = previous_offset
	force_wrist_roll_straight = previous_force_straight
	_registration_status = previous_status
	wrist_roll_alignment_preview_degrees = previous_preview
	_refresh_wrist_roll_alignment_preview()
	return false


func _signed_wrist_roll_preview_degrees() -> float:
	return wrapf(
		wrist_roll_alignment_preview_degrees + 180.0,
		0.0,
		360.0,
	) - 180.0


func _refresh_wrist_roll_alignment_preview() -> void:
	if not is_inside_tree():
		return
	var values := get_best_available_normalized_pose()
	if values.size() < 6 and preview_pose_degrees.size() >= 6:
		values = Array(preview_pose_degrees)
	if values.size() >= 6:
		_set_target_from_normalized(values)
	if _joint_nodes.size() > 4:
		_display_angles[4] = _target_angles[4]
		_apply_joint_angle(4, _display_angles[4])
	var applied = _last_status.get("applied_normalized", null)
	if applied is Array and (applied as Array).size() >= 6:
		_set_target_ghost_from_normalized(applied)

func get_link_surface_points_world(
	link_name: String,
	maximum_points: int = 180,
) -> PackedVector3Array:
	_ensure_link_surface_points(maxi(200, maximum_points * 4))
	var link := _links.get(link_name, null) as Node3D
	var local_points = _link_surface_points.get(link_name, PackedVector3Array())
	var result := PackedVector3Array()
	if link == null or not local_points is PackedVector3Array:
		return result
	var step := maxi(1, int(ceil(float(local_points.size()) / float(maxi(1, maximum_points)))))
	for index in range(0, local_points.size(), step):
		result.append(link.to_global(local_points[index]))
	return result

func apply_d455_distal_visual_correction(
	residual_global: Vector3,
	confidence: float,
) -> void:
	if not d455_visual_correction_enabled or not residual_global.is_finite():
		return
	var wrist := _joint_nodes[3] if _joint_nodes.size() > 3 else null
	if wrist == null or wrist.get_parent() is not Node3D:
		return
	var parent := wrist.get_parent() as Node3D
	var residual_local := parent.global_transform.basis.inverse() * residual_global
	var maximum_step := 0.004
	if residual_local.length() > maximum_step:
		residual_local = residual_local.normalized() * maximum_step
	_d455_distal_correction_local += residual_local
	if _d455_distal_correction_local.length() > 0.03:
		_d455_distal_correction_local = (
			_d455_distal_correction_local.normalized() * 0.03
		)
	_d455_distal_correction_confidence = clampf(confidence, 0.0, 1.0)
	_d455_distal_correction_msec = Time.get_ticks_msec()

func apply_claw_visual_calibration(
	wrist_roll_offset_degrees: float,
	jaw_angle_offset_degrees: float,
	jaw_angle_scale_degrees: float,
	mount_correction_local: Vector3,
	metrics: Dictionary = {},
	jaw_angle_curvature_degrees: float = 0.0,
	visual_correction_rpy_degrees: Vector3 = Vector3.ZERO,
	visual_correction_translation_local: Vector3 = Vector3.ZERO,
) -> bool:
	if (
		not is_finite(wrist_roll_offset_degrees)
		or absf(wrist_roll_offset_degrees) > 360.0
		or not is_finite(jaw_angle_offset_degrees)
		or absf(jaw_angle_offset_degrees) > 360.0
		or not is_finite(jaw_angle_scale_degrees)
		or absf(jaw_angle_scale_degrees) > 5.0
		or not is_finite(jaw_angle_curvature_degrees)
		or absf(jaw_angle_curvature_degrees) > 5.0
		or not _gripper_calibration_matches_closed_endpoint(
			jaw_angle_offset_degrees,
			jaw_angle_scale_degrees,
			jaw_angle_curvature_degrees,
		)
		or not mount_correction_local.is_finite()
		or (
			mount_correction_local.length()
			> GRIPPER_MOUNT_CORRECTION_MAXIMUM_METERS
		)
		or not visual_correction_rpy_degrees.is_finite()
		or (
			visual_correction_rpy_degrees.length()
			> GRIPPER_VISUAL_CORRECTION_MAXIMUM_DEGREES
		)
		or not visual_correction_translation_local.is_finite()
		or (
			visual_correction_translation_local.length()
			> GRIPPER_VISUAL_CORRECTION_MAXIMUM_METERS
		)
	):
		return false
	var previous_roll := joint_angle_offsets_degrees[4]
	var previous_angle_offset := gripper_angle_offset_degrees
	var previous_angle_scale := gripper_angle_scale_degrees_per_normalized
	var previous_angle_curvature := gripper_angle_curvature_degrees
	var previous_mount := gripper_mount_correction_local
	var previous_visual_rpy := gripper_visual_correction_rpy_degrees
	var previous_visual_translation := (
		gripper_visual_correction_translation_local
	)
	joint_angle_offsets_degrees[4] = wrist_roll_offset_degrees
	gripper_angle_offset_degrees = jaw_angle_offset_degrees
	gripper_angle_scale_degrees_per_normalized = jaw_angle_scale_degrees
	gripper_angle_curvature_degrees = jaw_angle_curvature_degrees
	gripper_mount_correction_local = mount_correction_local
	gripper_visual_correction_rpy_degrees = (
		visual_correction_rpy_degrees
	)
	gripper_visual_correction_translation_local = (
		visual_correction_translation_local
	)
	_registration_status["confidence"] = float(metrics.get("confidence", 0.90))
	_registration_status["saved_unix_ms"] = (
		Time.get_unix_time_from_system() * 1000.0
	)
	var existing_refinement = _registration_status.get(
		"joint_refinement",
		{},
	)
	var refinement := (
		(existing_refinement as Dictionary).duplicate(true)
		if existing_refinement is Dictionary
		else {}
	)
	refinement["claw_calibration"] = metrics.duplicate(true)
	_registration_status["joint_refinement"] = refinement
	if _save_registration():
		return true
	joint_angle_offsets_degrees[4] = previous_roll
	gripper_angle_offset_degrees = previous_angle_offset
	gripper_angle_scale_degrees_per_normalized = previous_angle_scale
	gripper_angle_curvature_degrees = previous_angle_curvature
	gripper_mount_correction_local = previous_mount
	gripper_visual_correction_rpy_degrees = previous_visual_rpy
	gripper_visual_correction_translation_local = (
		previous_visual_translation
	)
	return false

func get_d455_visual_correction_status() -> Dictionary:
	return {
		"enabled": d455_visual_correction_enabled,
		"confidence": _d455_distal_correction_confidence,
		"age_ms": (
			Time.get_ticks_msec() - _d455_distal_correction_msec
			if _d455_distal_correction_msec > 0
			else -1
		),
		"correction_m": _d455_distal_correction_local.length(),
	}

func get_robot_base_global_position() -> Vector3:
	var base := _links.get("base_link") as Node3D
	return base.global_position if base != null else global_position

func get_latest_status() -> Dictionary:
	return _last_status.duplicate(true)


func get_measured_normalized_pose() -> Array:
	# follower_normalized is calculated from the latest raw servo encoder read by
	# the follower service.
	var values = _last_status.get("follower_normalized", null)
	if not values is Array or (values as Array).size() < 6:
		return []
	return (values as Array).slice(0, 6)


func get_best_available_normalized_pose() -> Array:
	# With measured feedback enabled, never silently turn a commanded goal into
	# the depicted physical pose. A stale read either freezes the last measured
	# model pose or, only when explicitly requested, falls back to the goal.
	var measured := get_measured_normalized_pose()
	var read_age = _last_status.get("last_read_age_ms", null)
	if measured_feedback_enabled and not measured.is_empty() and read_age != null and float(read_age) <= 350.0:
		return measured
	var applied = _last_status.get("applied_normalized", null)
	if (
		(not measured_feedback_enabled or not freeze_overlay_on_stale_enabled)
		and applied is Array
		and (applied as Array).size() >= 6
	):
		return (applied as Array).slice(0, 6)
	return [] if freeze_overlay_on_stale_enabled else measured


func get_hybrid_render_state() -> Dictionary:
	var link_transforms: Dictionary = {}
	for link_name in LINK_MODELS:
		var link := _links.get(link_name, null) as Node3D
		if link != null:
			link_transforms[link_name] = _transform_to_dictionary(link.global_transform)
	return {
		"available": not link_transforms.is_empty(),
		"links": link_transforms,
		"manual_claw_calibration": get_manual_claw_calibration_state(),
	}

func get_registration_status() -> Dictionary:
	var result := _registration_status.duplicate(true)
	# Report the live visualization invariant, not merely the historical file
	# field, so acceptance tests can prove that measured wrist roll is active.
	result["force_wrist_roll_straight"] = force_wrist_roll_straight
	result["joint_angle_directions"] = joint_angle_directions.duplicate()
	result["joint_angle_offsets_degrees"] = (
		joint_angle_offsets_degrees.duplicate()
	)
	result["wrist_housing_mount_flipped"] = wrist_housing_mount_flipped
	return result


func _calibration_transaction_is_stale(
	calibration_started_unix_ms: float,
	existing_started_unix_ms: float,
) -> bool:
	# JSON serialization can round these epoch-millisecond floats by a few
	# thousandths. Treat sub-millisecond differences as the same transaction;
	# otherwise the second outward checkpoint can reject its own first save.
	return (
		calibration_started_unix_ms > 0.0
		and existing_started_unix_ms
			> calibration_started_unix_ms + 1.0
	)


func apply_calibrated_global_transform(
	value: Transform3D,
	confidence: float = 0.0,
	calibration_started_unix_ms: float = 0.0,
	registration_source: String = "markerless_motion_fit",
) -> bool:
	if not value.is_finite() or not _registration_is_upright(value):
		return false
	var existing := _read_registration_payload()
	var existing_started := float(existing.get("calibration_started_unix_ms", 0.0))
	if _calibration_transaction_is_stale(
		calibration_started_unix_ms,
		existing_started,
	):
		return false
	global_transform = Transform3D(value.basis.orthonormalized(), value.origin)
	_registration_status = {
		"registered": true,
		"confidence": clampf(confidence, 0.0, 1.0),
		"calibration_started_unix_ms": calibration_started_unix_ms,
		"saved_unix_ms": Time.get_unix_time_from_system() * 1000.0,
		"registration_source": registration_source,
	}
	return _save_registration()


func _capture_scene_fallback() -> void:
	_scene_fallback_transform = transform
	_scene_fallback_mapping = {
		"model_scale": model_scale,
		"joint_angle_directions": joint_angle_directions.duplicate(),
		"joint_angle_offsets_degrees": joint_angle_offsets_degrees.duplicate(),
		"force_wrist_roll_straight": force_wrist_roll_straight,
		"gripper_angle_offset_degrees": gripper_angle_offset_degrees,
		"gripper_angle_scale_degrees_per_normalized": (
			gripper_angle_scale_degrees_per_normalized
		),
		"gripper_angle_curvature_degrees": gripper_angle_curvature_degrees,
		"gripper_angle_samples_normalized": (
			gripper_angle_samples_normalized.duplicate()
		),
		"gripper_angle_samples_degrees": (
			gripper_angle_samples_degrees.duplicate()
		),
		"gripper_mount_correction_local": gripper_mount_correction_local,
		"gripper_visual_correction_rpy_degrees": (
			gripper_visual_correction_rpy_degrees
		),
		"gripper_visual_correction_translation_local": (
			gripper_visual_correction_translation_local
		),
		"moving_jaw_calibration_enabled": moving_jaw_calibration_enabled,
		"moving_jaw_pivot_parent": moving_jaw_pivot_parent,
		"moving_jaw_axis_parent": moving_jaw_axis_parent,
		"moving_jaw_closed_basis_parent": moving_jaw_closed_basis_parent,
		"moving_jaw_opening_samples_degrees": moving_jaw_opening_samples_degrees.duplicate(),
		"moving_jaw_visual_radial_scale": moving_jaw_visual_radial_scale,
		"moving_jaw_visual_axial_translation_local": moving_jaw_visual_axial_translation_local,
	}


func _reset_to_scene_fallback() -> void:
	transform = _scene_fallback_transform
	if _scene_fallback_mapping.is_empty():
		return
	model_scale = float(_scene_fallback_mapping["model_scale"])
	joint_angle_directions = (
		_scene_fallback_mapping["joint_angle_directions"] as PackedFloat32Array
	).duplicate()
	joint_angle_offsets_degrees = (
		_scene_fallback_mapping["joint_angle_offsets_degrees"] as PackedFloat32Array
	).duplicate()
	force_wrist_roll_straight = bool(
		_scene_fallback_mapping["force_wrist_roll_straight"]
	)
	gripper_angle_offset_degrees = float(
		_scene_fallback_mapping["gripper_angle_offset_degrees"]
	)
	gripper_angle_scale_degrees_per_normalized = float(
		_scene_fallback_mapping["gripper_angle_scale_degrees_per_normalized"]
	)
	gripper_angle_curvature_degrees = float(
		_scene_fallback_mapping["gripper_angle_curvature_degrees"]
	)
	gripper_angle_samples_normalized = (
		_scene_fallback_mapping["gripper_angle_samples_normalized"] as PackedFloat32Array
	).duplicate()
	gripper_angle_samples_degrees = (
		_scene_fallback_mapping["gripper_angle_samples_degrees"] as PackedFloat32Array
	).duplicate()
	gripper_mount_correction_local = Vector3(
		_scene_fallback_mapping["gripper_mount_correction_local"]
	)
	gripper_visual_correction_rpy_degrees = Vector3(
		_scene_fallback_mapping["gripper_visual_correction_rpy_degrees"]
	)
	gripper_visual_correction_translation_local = Vector3(
		_scene_fallback_mapping[
			"gripper_visual_correction_translation_local"
		]
	)
	moving_jaw_calibration_enabled = bool(
		_scene_fallback_mapping["moving_jaw_calibration_enabled"]
	)
	moving_jaw_pivot_parent = Vector3(_scene_fallback_mapping["moving_jaw_pivot_parent"])
	moving_jaw_axis_parent = Vector3(_scene_fallback_mapping["moving_jaw_axis_parent"])
	moving_jaw_closed_basis_parent = Basis(
		_scene_fallback_mapping["moving_jaw_closed_basis_parent"]
	)
	moving_jaw_opening_samples_degrees = (
		_scene_fallback_mapping["moving_jaw_opening_samples_degrees"] as PackedFloat32Array
	).duplicate()
	moving_jaw_visual_radial_scale = float(
		_scene_fallback_mapping["moving_jaw_visual_radial_scale"]
	)
	moving_jaw_visual_axial_translation_local = float(
		_scene_fallback_mapping["moving_jaw_visual_axial_translation_local"]
	)
	_apply_moving_jaw_visual_geometry()


func _registration_is_upright(value: Transform3D) -> bool:
	if not value.is_finite() or value.basis.determinant() <= 0.0:
		return false
	var world_up := value.basis.orthonormalized().y.normalized()
	var parent := get_parent() as Node3D
	if parent != null:
		world_up = (
			parent.global_transform.basis.orthonormalized() * world_up
		).normalized()
	return world_up.dot(Vector3.UP) >= MINIMUM_REGISTRATION_WORLD_UP_DOT


func clear_saved_registration() -> void:
	if FileAccess.file_exists(_registration_path):
		DirAccess.remove_absolute(ProjectSettings.globalize_path(_registration_path))
	_reset_to_scene_fallback()
	_registration_modified_time = 0
	_registration_status = {"registered": false}

func save_robot_position_checkpoint() -> bool:
	if not bool(_registration_status.get("registered", false)):
		_registration_status["fault"] = "there is no calibrated robot position to save"
		return false
	if not transform.is_finite():
		_registration_status["fault"] = "current robot position is invalid"
		return false
	var payload := _read_registration_payload()
	payload.merge(_transform_to_dictionary(transform), true)
	payload["type"] = "so101_robot_position_checkpoint"
	payload["kinematics_version"] = ROBOT_KINEMATICS_VERSION
	payload["confidence"] = float(_registration_status.get("confidence", 0.0))
	payload["calibration_started_unix_ms"] = float(
		_registration_status.get("calibration_started_unix_ms", 0.0)
	)
	payload["checkpoint_saved_unix_ms"] = Time.get_unix_time_from_system() * 1000.0
	payload["registration_source"] = "saved_robot_position_checkpoint"
	var file := FileAccess.open(_robot_position_checkpoint_path, FileAccess.WRITE)
	if file == null:
		_registration_status["fault"] = "could not save robot position checkpoint"
		return false
	file.store_string(JSON.stringify(payload, "\t"))
	_registration_status.erase("fault")
	return true


func restore_saved_robot_position() -> bool:
	var parsed := _read_registration_payload_from(_robot_position_checkpoint_path)
	if parsed.is_empty():
		_registration_status["fault"] = "saved robot position checkpoint is missing"
		return false
	if int(parsed.get("kinematics_version", 0)) != ROBOT_KINEMATICS_VERSION:
		_registration_status["fault"] = "saved robot position uses an incompatible joint mapping"
		return false
	var restored := _transform_from_dictionary(parsed)
	if not restored.is_finite():
		_registration_status["fault"] = "saved robot position is invalid"
		return false
	transform = Transform3D(restored.basis.orthonormalized(), restored.origin)
	_registration_status = {
		"registered": true,
		"confidence": float(parsed.get("confidence", 1.0)),
		"calibration_started_unix_ms": float(parsed.get("calibration_started_unix_ms", 0.0)),
		"saved_unix_ms": Time.get_unix_time_from_system() * 1000.0,
		"registration_source": "saved_robot_position_checkpoint",
	}
	return _save_registration()


# Compatibility for callers outside this scene that still use the old name.
func restore_depth_validated_anchor() -> bool:
	return restore_saved_robot_position()

func get_pose_capsules_local(values: Variant) -> PackedVector4Array:
	var points := PackedVector3Array([Vector3.ZERO])
	var chain := Transform3D.IDENTITY
	for index in range(JOINTS.size()):
		var entry: Dictionary = JOINTS[index]
		var origin := Transform3D(_basis_from_urdf_rpy(entry["rpy"]), entry["xyz"])
		var angle := _normalized_joint_angle(index, float(values[index]))
		chain = chain * origin * Transform3D(Basis(Vector3(0.0, 0.0, 1.0), angle), Vector3.ZERO)
		points.append(ROS_TO_GODOT * (chain.origin * model_scale))
	var capsules := PackedVector4Array()
	var radii := [0.065, 0.060, 0.050, 0.047, 0.043, 0.040]
	for index in range(radii.size()):
		var radius: float = radii[index] * model_scale
		var a := points[index]
		var b := points[index + 1]
		capsules.append(Vector4(a.x, a.y, a.z, radius))
		capsules.append(Vector4(b.x, b.y, b.z, radius))
	return capsules


func get_pose_joint_frames_local(values: Variant) -> Array:
	var result: Array = []
	var chain := Transform3D.IDENTITY
	for index in range(JOINTS.size()):
		if index == 5 and moving_jaw_calibration_enabled:
			var opening := _normalized_joint_angle(index, float(values[index]))
			var jaw_transform := _moving_jaw_transform(opening)
			var pivot := ROS_TO_GODOT * ((chain * jaw_transform).origin * model_scale)
			var axis := (
				ROS_TO_GODOT * (chain.basis * moving_jaw_axis_parent.normalized())
			).normalized()
			result.append({"pivot": pivot, "axis": axis})
			chain = chain * jaw_transform
			continue
		var entry: Dictionary = JOINTS[index]
		var origin := Transform3D(_basis_from_urdf_rpy(entry["rpy"]), entry["xyz"])
		chain = chain * origin
		var pivot := ROS_TO_GODOT * (chain.origin * model_scale)
		var axis := (ROS_TO_GODOT * (chain.basis * Vector3(0.0, 0.0, 1.0))).normalized()
		result.append({"pivot": pivot, "axis": axis})
		var angle := _normalized_joint_angle(index, float(values[index]))
		chain = chain * Transform3D(Basis(Vector3(0.0, 0.0, 1.0), angle), Vector3.ZERO)
		if index == 4:
			chain.origin += gripper_mount_correction_local
			chain = chain * _gripper_visual_correction_transform()
	return result


func get_pose_mesh_points_local(values: Variant, maximum_points_per_link: int = 220) -> Dictionary:
	_ensure_link_surface_points(maximum_points_per_link)
	var result := {}
	var chain := Transform3D.IDENTITY
	var link_names := ["base_link"]
	var link_transforms: Array[Transform3D] = [Transform3D.IDENTITY]
	for index in range(JOINTS.size()):
		var entry: Dictionary = JOINTS[index]
		var angle := _normalized_joint_angle(index, float(values[index]))
		if index == 5 and moving_jaw_calibration_enabled:
			chain = chain * _moving_jaw_transform(angle)
		else:
			var origin := Transform3D(_basis_from_urdf_rpy(entry["rpy"]), entry["xyz"])
			chain = chain * origin * Transform3D(Basis(Vector3(0.0, 0.0, 1.0), angle), Vector3.ZERO)
		if index == 4:
			chain.origin += gripper_mount_correction_local
			chain = chain * _gripper_visual_correction_transform()
		link_names.append(str(entry["child"]))
		link_transforms.append(chain)
	for index in range(link_names.size()):
		var raw: PackedVector3Array = _link_surface_points.get(link_names[index], PackedVector3Array())
		var posed := PackedVector3Array()
		posed.resize(raw.size())
		for point_index in range(raw.size()):
			var point := raw[point_index]
			if link_names[index] == "moving_jaw_so101_v1_link" and moving_jaw_calibration_enabled:
				point.x *= moving_jaw_visual_radial_scale
				point.y *= moving_jaw_visual_radial_scale
				point.z += moving_jaw_visual_axial_translation_local
			posed[point_index] = ROS_TO_GODOT * ((link_transforms[index] * point) * model_scale)
		result[link_names[index]] = posed
	return result


func get_pose_claw_tip_positions_local(
	values: Variant,
	maximum_points_per_link: int = 20000,
) -> PackedVector3Array:
	var result := PackedVector3Array()
	if not values is Array or (values as Array).size() < JOINTS.size():
		return result
	_ensure_link_surface_points(maximum_points_per_link)
	var chain := Transform3D.IDENTITY
	for index in range(JOINTS.size()):
		var entry: Dictionary = JOINTS[index]
		var angle := _normalized_joint_angle(index, float((values as Array)[index]))
		if index == 5 and moving_jaw_calibration_enabled:
			chain = chain * _moving_jaw_transform(angle)
		else:
			var origin := Transform3D(_basis_from_urdf_rpy(entry["rpy"]), entry["xyz"])
			chain = chain * origin * Transform3D(Basis(Vector3(0.0, 0.0, 1.0), angle), Vector3.ZERO)
		if index == 4:
			chain.origin += gripper_mount_correction_local
			chain = chain * _gripper_visual_correction_transform()
			var fixed_tip := _raw_claw_terminal_center("gripper_link")
			if fixed_tip.is_finite():
				result.append(ROS_TO_GODOT * ((chain * fixed_tip) * model_scale))
		elif index == 5:
			var moving_tip := _raw_claw_terminal_center("moving_jaw_so101_v1_link")
			if moving_tip.is_finite():
				if moving_jaw_calibration_enabled:
					moving_tip.x *= moving_jaw_visual_radial_scale
					moving_tip.y *= moving_jaw_visual_radial_scale
					moving_tip.z += moving_jaw_visual_axial_translation_local
				result.append(ROS_TO_GODOT * ((chain * moving_tip) * model_scale))
	return result


func get_pose_claw_calibration_metadata_local(values: Variant) -> Dictionary:
	if not values is Array or (values as Array).size() < JOINTS.size():
		return {}
	var chain := Transform3D.IDENTITY
	for index in range(5):
		var entry: Dictionary = JOINTS[index]
		var origin := Transform3D(_basis_from_urdf_rpy(entry["rpy"]), entry["xyz"])
		var angle := _normalized_joint_angle(index, float(values[index]))
		chain = chain * origin * Transform3D(
			Basis(Vector3(0.0, 0.0, 1.0), angle),
			Vector3.ZERO,
		)
	var pre_visual_chain := chain
	pre_visual_chain.origin += gripper_mount_correction_local
	var pre_visual_local := Transform3D(
		ROS_TO_GODOT * pre_visual_chain.basis,
		ROS_TO_GODOT * (pre_visual_chain.origin * model_scale),
	)
	var corrected_chain := pre_visual_chain * _gripper_visual_correction_transform()
	var corrected_local := Transform3D(
		ROS_TO_GODOT * corrected_chain.basis,
		ROS_TO_GODOT * (corrected_chain.origin * model_scale),
	)
	var metadata := {
		"pre_visual_transform_local": pre_visual_local,
		"corrected_transform_local": corrected_local,
		"model_gripper_angle_degrees": _calibrated_gripper_angle_degrees(
			float((values as Array)[5])
		),
		"mount_correction_local": gripper_mount_correction_local,
		"visual_correction_rpy_degrees": gripper_visual_correction_rpy_degrees,
		"visual_correction_translation_local": gripper_visual_correction_translation_local,
		"validated_angle_samples_normalized": gripper_angle_samples_normalized,
		"validated_angle_samples_degrees": gripper_angle_samples_degrees,
	}
	if moving_jaw_calibration_enabled:
		metadata["moving_jaw_calibration_enabled"] = true
		metadata["moving_jaw_pivot_parent"] = [
			moving_jaw_pivot_parent.x,
			moving_jaw_pivot_parent.y,
			moving_jaw_pivot_parent.z,
		]
		metadata["moving_jaw_axis_parent"] = [
			moving_jaw_axis_parent.x,
			moving_jaw_axis_parent.y,
			moving_jaw_axis_parent.z,
		]
		metadata["moving_jaw_closed_basis_parent_row_major"] = [
			moving_jaw_closed_basis_parent.x.x,
			moving_jaw_closed_basis_parent.y.x,
			moving_jaw_closed_basis_parent.z.x,
			moving_jaw_closed_basis_parent.x.y,
			moving_jaw_closed_basis_parent.y.y,
			moving_jaw_closed_basis_parent.z.y,
			moving_jaw_closed_basis_parent.x.z,
			moving_jaw_closed_basis_parent.y.z,
			moving_jaw_closed_basis_parent.z.z,
		]
		metadata["moving_jaw_opening_degrees"] = (
			_calibrated_moving_jaw_opening_degrees(float((values as Array)[5]))
		)
		metadata["moving_jaw_opening_samples_degrees"] = (
			moving_jaw_opening_samples_degrees
		)
		metadata["moving_jaw_visual_radial_scale"] = moving_jaw_visual_radial_scale
		metadata["moving_jaw_visual_axial_translation_local"] = (
			moving_jaw_visual_axial_translation_local
		)
	else:
		metadata["moving_jaw_calibration_enabled"] = false
	return metadata


func _ensure_link_surface_points(maximum_points_per_link: int) -> void:
	if (
		not _link_surface_points.is_empty()
		and maximum_points_per_link <= _link_surface_points_maximum_per_link
	):
		return
	_link_surface_points.clear()
	_link_surface_points_maximum_per_link = maximum_points_per_link
	for link_name in LINK_MODELS:
		var link := _links.get(link_name, null) as Node3D
		if link == null:
			continue
		var vertices := PackedVector3Array()
		for child in link.get_children():
			_collect_visual_vertices(child, Transform3D.IDENTITY, vertices)
		if vertices.is_empty():
			continue
		var sampled := PackedVector3Array()
		var step := maxi(1, int(ceil(float(vertices.size()) / float(maxi(1, maximum_points_per_link)))))
		for index in range(0, vertices.size(), step):
			sampled.append(vertices[index])
		_link_surface_points[link_name] = sampled


func _raw_claw_terminal_center(link_name: String) -> Vector3:
	var points: PackedVector3Array = _link_surface_points.get(
		link_name,
		PackedVector3Array(),
	)
	if points.is_empty():
		return Vector3(INF, INF, INF)
	# These are stock-link coordinates, before any joint transform. The fixed
	# finger extends along -Z in the imported gripper link; the moving finger
	# extends along -Y in its own link. Selecting that terminal slab excludes
	# the servo housing and cannot jump as the jaw articulates.
	var terminal_coordinate := INF
	for point in points:
		terminal_coordinate = minf(
			terminal_coordinate,
			point.z if link_name == "gripper_link" else point.y,
		)
	var sum := Vector3.ZERO
	var count := 0
	for point in points:
		var coordinate := point.z if link_name == "gripper_link" else point.y
		if coordinate <= terminal_coordinate + 0.0015:
			sum += point
			count += 1
	return sum / float(count) if count > 0 else Vector3(INF, INF, INF)


func _collect_visual_vertices(node: Node, parent_transform: Transform3D, output: PackedVector3Array) -> void:
	var local_transform := parent_transform
	if node is Node3D:
		local_transform = parent_transform * (node as Node3D).transform
	if node is MeshInstance3D:
		var mesh := (node as MeshInstance3D).mesh
		if mesh != null:
			for surface_index in range(mesh.get_surface_count()):
				var arrays := mesh.surface_get_arrays(surface_index)
				if arrays.size() <= Mesh.ARRAY_VERTEX:
					continue
				var vertices: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
				for vertex in vertices:
					output.append(local_transform * vertex)
	for child in node.get_children():
		_collect_visual_vertices(child, local_transform, output)

func get_mask_capsules_world() -> PackedVector4Array:
	var capsules := PackedVector4Array()
	if not overlay_enabled or not mask_scanned_robot or _links.is_empty():
		return capsules
	var names := ["base_link", "shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link", "moving_jaw_so101_v1_link"]
	var radii := [0.065, 0.060, 0.050, 0.047, 0.043, 0.040]
	for index in range(radii.size()):
		var a := (_links[names[index]] as Node3D).global_position
		var b := (_links[names[index + 1]] as Node3D).global_position
		var radius: float = radii[index] * model_scale
		capsules.append(Vector4(a.x, a.y, a.z, radius))
		capsules.append(Vector4(b.x, b.y, b.z, radius))
	return capsules


func get_claw_tip_positions_world() -> PackedVector3Array:
	# Return stock-link terminal slabs, never a radial extreme around the moving
	# jaw hinge (which can select the servo housing or an interior mesh corner).
	_ensure_link_surface_points(20000)
	var result := PackedVector3Array()
	for link_name in ["gripper_link", "moving_jaw_so101_v1_link"]:
		var link := _links.get(link_name, null) as Node3D
		if link == null:
			continue
		var tip := _raw_claw_terminal_center(link_name)
		if tip.is_finite():
			result.append(link.to_global(tip))
	return result

func _build_model() -> void:
	_model_root = get_node_or_null(MODEL_ROOT_NAME) as Node3D
	if _model_root != null:
		_index_existing_model()
		_apply_model_scale()
		return
	_model_root = Node3D.new()
	_model_root.name = MODEL_ROOT_NAME
	_model_root.basis = ROS_TO_GODOT
	add_child(_model_root)
	var base := Node3D.new()
	base.name = "base_link"
	_model_root.add_child(base)
	_links[base.name] = base
	_add_link_visual(base, base.name)
	for entry in JOINTS:
		var parent := _links[str(entry["parent"])] as Node3D
		var child := Node3D.new()
		child.name = str(entry["child"])
		parent.add_child(child)
		_links[child.name] = child
		_joint_nodes.append(child)
		var origin := Transform3D(_basis_from_urdf_rpy(entry["rpy"]), entry["xyz"])
		_joint_origin_transforms.append(origin)
		child.transform = origin
		_add_link_visual(child, child.name)
	_apply_model_scale()
	_apply_moving_jaw_visual_geometry()

func _build_target_ghost() -> void:
	_target_ghost_root = get_node_or_null(TARGET_GHOST_ROOT_NAME) as Node3D
	if _target_ghost_root != null:
		_target_ghost_root.queue_free()
	_target_ghost_links.clear()
	_target_ghost_joint_nodes.clear()
	_target_ghost_joint_origins.clear()
	_target_ghost_root = Node3D.new()
	_target_ghost_root.name = TARGET_GHOST_ROOT_NAME
	_target_ghost_root.basis = ROS_TO_GODOT
	_target_ghost_root.scale = Vector3.ONE * model_scale
	add_child(_target_ghost_root)
	var base := Node3D.new()
	base.name = "base_link"
	_target_ghost_root.add_child(base)
	_target_ghost_links[base.name] = base
	_add_target_ghost_visual(base, base.name)
	for entry in JOINTS:
		var parent := _target_ghost_links[str(entry["parent"])] as Node3D
		var child := Node3D.new()
		child.name = str(entry["child"])
		parent.add_child(child)
		_target_ghost_links[child.name] = child
		_target_ghost_joint_nodes.append(child)
		var origin := Transform3D(_basis_from_urdf_rpy(entry["rpy"]), entry["xyz"])
		_target_ghost_joint_origins.append(origin)
		child.transform = origin
		_add_target_ghost_visual(child, child.name)
	_update_target_ghost_visibility()
	_apply_moving_jaw_visual_geometry()

func _add_target_ghost_visual(link: Node3D, link_name: String) -> void:
	var path := str(LINK_MODELS.get(link_name, ""))
	var scene := load(path) as PackedScene
	if scene == null:
		return
	var visual := scene.instantiate()
	visual.name = "TargetGhostVisual"
	link.add_child(visual)
	_apply_link_mount_visual(visual, link_name)
	_configure_target_ghost_visual(visual)

func _configure_target_ghost_visual(node: Node) -> void:
	if node is GeometryInstance3D:
		var geometry := node as GeometryInstance3D
		geometry.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		geometry.transparency = 0.68
		var material := StandardMaterial3D.new()
		material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		material.albedo_color = Color(0.12, 0.72, 1.0, 0.32)
		material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		geometry.material_override = material
	for child in node.get_children():
		_configure_target_ghost_visual(child)

func _set_target_ghost_from_normalized(values: Variant) -> void:
	if values is not Array or (values as Array).size() < 6:
		return
	for index in range(mini(6, _target_ghost_joint_nodes.size())):
		var angle := _live_visual_joint_angle(index, float(values[index]))
		var rotation := Transform3D(
			Basis(Vector3(0.0, 0.0, 1.0), angle),
			Vector3.ZERO,
		)
		if index == 5 and moving_jaw_calibration_enabled:
			_target_ghost_joint_nodes[index].transform = _moving_jaw_transform(angle)
		else:
			_target_ghost_joint_nodes[index].transform = (
				_target_ghost_joint_origins[index] * rotation
			)
		if index == 4:
			_target_ghost_joint_nodes[index].position += (
				gripper_mount_correction_local
			)
			_apply_gripper_visual_correction(
				_target_ghost_joint_nodes[index],
			)

func _update_target_ghost_visibility() -> void:
	if _target_ghost_root == null:
		return
	var applied = _last_status.get("applied_normalized", null)
	_target_ghost_root.visible = (
		overlay_enabled
		and target_ghost_enabled
		and applied is Array
		and (applied as Array).size() >= 6
	)

func _index_existing_model() -> void:
	_links.clear()
	_joint_nodes.clear()
	_joint_origin_transforms.clear()
	var base := _model_root.get_node_or_null("base_link") as Node3D
	if base == null:
		_model_root.queue_free()
		_model_root = null
		_build_model()
		return
	_links[base.name] = base
	for entry in JOINTS:
		var parent := _links[str(entry["parent"])] as Node3D
		var child := parent.get_node_or_null(str(entry["child"])) as Node3D
		if child == null:
			return
		_links[child.name] = child
		_joint_nodes.append(child)
		_joint_origin_transforms.append(Transform3D(_basis_from_urdf_rpy(entry["rpy"]), entry["xyz"]))

func _add_link_visual(link: Node3D, link_name: String) -> void:
	var path := str(LINK_MODELS.get(link_name, ""))
	if path.is_empty():
		return
	var scene := load(path) as PackedScene
	if scene == null:
		push_warning("SO-101 overlay model is not imported yet: %s" % path)
		return
	var visual := scene.instantiate()
	visual.name = "Visual"
	link.add_child(visual)
	_apply_link_mount_visual(visual, link_name)
	_configure_visual(visual)


func _apply_link_mount_visual(visual: Node3D, link_name: String) -> void:
	visual.transform = (
		_wrist_housing_mount_visual_transform()
		if link_name == "wrist_link" and wrist_housing_mount_flipped
		else Transform3D.IDENTITY
	)


func _wrist_housing_mount_visual_transform() -> Transform3D:
	# The wrist-roll joint axis expressed in wrist_link is +Y. Its published
	# origin is also the physical servo-output pivot, so this changes handedness
	# without translating the housing or rotating any child joint frame.
	var pivot: Vector3 = JOINTS[4]["xyz"]
	return (
		Transform3D(Basis.IDENTITY, pivot)
		* Transform3D(Basis(Vector3.UP, PI), Vector3.ZERO)
		* Transform3D(Basis.IDENTITY, -pivot)
	)


func _apply_wrist_housing_mount_visual() -> void:
	for links in [_links, _target_ghost_links]:
		var wrist := (links as Dictionary).get("wrist_link", null) as Node3D
		if wrist == null:
			continue
		var visual_name := (
			"TargetGhostVisual" if links == _target_ghost_links else "Visual"
		)
		var visual := wrist.get_node_or_null(visual_name) as Node3D
		if visual != null:
			_apply_link_mount_visual(visual, "wrist_link")
	# Calibration surface points include the visual-node transform. Force them
	# to be rebuilt so automatic fitting sees the same handedness as the editor.
	_link_surface_points.clear()
	_link_surface_points_maximum_per_link = 0

func _configure_visual(node: Node) -> void:
	if node is GeometryInstance3D:
		(node as GeometryInstance3D).cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_ON
		(node as GeometryInstance3D).transparency = overlay_transparency
	for child in node.get_children():
		_configure_visual(child)

func _apply_overlay_transparency() -> void:
	if _model_root == null:
		return
	_set_overlay_transparency_recursive(_model_root)

func _set_overlay_transparency_recursive(node: Node) -> void:
	if node is GeometryInstance3D:
		(node as GeometryInstance3D).transparency = overlay_transparency
	for child in node.get_children():
		_set_overlay_transparency_recursive(child)

func _basis_from_urdf_rpy(rpy: Vector3) -> Basis:
	return Basis(Vector3(0.0, 0.0, 1.0), rpy.z) * Basis(Vector3(0.0, 1.0, 0.0), rpy.y) * Basis(Vector3(1.0, 0.0, 0.0), rpy.x)

func _apply_model_scale() -> void:
	if _model_root:
		_model_root.scale = Vector3.ONE * model_scale
	if _target_ghost_root:
		_target_ghost_root.scale = Vector3.ONE * model_scale

func _poll_telemetry() -> void:
	if not _udp_bound:
		return
	var latest: Dictionary = {}
	while _udp.get_available_packet_count() > 0:
		var parsed = JSON.parse_string(_udp.get_packet().get_string_from_utf8())
		if parsed is Dictionary and str((parsed as Dictionary).get("type", "")) == "arm_status":
			latest = parsed as Dictionary
	if latest.is_empty():
		return
	_last_status = latest
	var service_settings = latest.get("feedback_settings", null)
	if service_settings is Dictionary:
		configure_feedback_features(service_settings as Dictionary)
	var values := get_best_available_normalized_pose()
	if values.size() >= 6:
		_set_target_from_normalized(values)
	var applied = latest.get("applied_normalized", null)
	if applied is Array and (applied as Array).size() >= 6:
		_set_target_ghost_from_normalized(applied)
	_update_target_ghost_visibility()

func _set_target_from_normalized(values: Variant) -> void:
	for index in range(5):
		_target_angles[index] = _live_visual_joint_angle(index, float(values[index]))
	_target_angles[5] = _normalized_joint_angle(5, float(values[5]))


func _live_visual_joint_angle(index: int, normalized: float) -> float:
	var angle := _normalized_joint_angle(index, normalized)
	if index == 4:
		angle = wrapf(
			angle + deg_to_rad(_signed_wrist_roll_preview_degrees()) + PI,
			0.0,
			TAU,
		) - PI
	return angle

func _normalized_joint_angle(index: int, normalized: float) -> float:
	var limits: Vector2 = JOINTS[index]["limits"]
	if index == 5:
		if moving_jaw_calibration_enabled:
			var jaw_degrees := _calibrated_moving_jaw_opening_degrees(normalized)
			if manual_claw_calibration_enabled:
				jaw_degrees = jaw_degrees * manual_opening_scale + manual_opening_offset_degrees
			return deg_to_rad(jaw_degrees)
		var calibrated := deg_to_rad(
			_calibrated_gripper_angle_degrees(normalized)
		)
		if manual_claw_calibration_enabled:
			calibrated = calibrated * manual_opening_scale + deg_to_rad(manual_opening_offset_degrees)
		return clampf(calibrated, limits.x, limits.y)
	if index == 4 and force_wrist_roll_straight:
		return 0.0
	var direction := manual_wrist_roll_direction if manual_claw_calibration_enabled and index == 4 else _joint_direction(index)
	var trim := 0.0
	if manual_claw_calibration_enabled:
		trim = manual_wrist_flex_trim_degrees if index == 3 else manual_wrist_roll_trim_degrees if index == 4 else 0.0
	var angle := deg_to_rad(normalized * direction + _joint_offset(index) + trim)
	angle = wrapf(angle + PI, 0.0, TAU) - PI
	return clampf(angle, limits.x, limits.y) if clamp_visual_joint_limits else angle


func _calibrated_gripper_angle_degrees(normalized: float) -> float:
	var count := mini(
		gripper_angle_samples_normalized.size(),
		gripper_angle_samples_degrees.size(),
	)
	if count >= 2:
		for index in range(count - 1):
			var left := float(gripper_angle_samples_normalized[index])
			var right := float(gripper_angle_samples_normalized[index + 1])
			if right <= left:
				break
			if normalized <= right:
				var fraction := inverse_lerp(left, right, normalized)
				return lerpf(
					float(gripper_angle_samples_degrees[index]),
					float(gripper_angle_samples_degrees[index + 1]),
					fraction,
				)
		var previous_normalized := float(
			gripper_angle_samples_normalized[count - 2]
		)
		var last_normalized := float(
			gripper_angle_samples_normalized[count - 1]
		)
		if last_normalized > previous_normalized:
			var previous_angle := float(
				gripper_angle_samples_degrees[count - 2]
			)
			var last_angle := float(
				gripper_angle_samples_degrees[count - 1]
			)
			return last_angle + (
				(normalized - last_normalized)
				* (last_angle - previous_angle)
				/ (last_normalized - previous_normalized)
			)
	return (
		gripper_angle_offset_degrees
		+ gripper_angle_scale_degrees_per_normalized * normalized
		+ gripper_angle_curvature_degrees
			* normalized * (100.0 - normalized) / 100.0
	)

func _gripper_calibration_matches_closed_endpoint(
	angle_offset_degrees: float,
	angle_scale_degrees: float,
	angle_curvature_degrees: float = 0.0,
) -> bool:
	var closed_angle := (
		angle_offset_degrees
		+ angle_scale_degrees * gripper_closed_normalized
		+ angle_curvature_degrees
			* gripper_closed_normalized
			* (100.0 - gripper_closed_normalized) / 100.0
	)
	var expected_closed_angle := rad_to_deg(
		(JOINTS[5]["limits"] as Vector2).x
	)
	return (
		absf(closed_angle - expected_closed_angle)
		<= GRIPPER_CLOSED_ENDPOINT_TOLERANCE_DEGREES
	)

func _joint_offset(index: int) -> float:
	if index == 0:
		return BASE_TELEMETRY_OFFSET_DEGREES
	return float(joint_angle_offsets_degrees[index]) if index < joint_angle_offsets_degrees.size() else 0.0


func _joint_direction(index: int) -> float:
	if index == 0:
		return BASE_TELEMETRY_DIRECTION
	return float(joint_angle_directions[index]) if index < joint_angle_directions.size() else 1.0

func _apply_joint_angle(index: int, angle: float) -> void:
	if index >= _joint_nodes.size():
		return
	var rotation := Transform3D(Basis(Vector3(0.0, 0.0, 1.0), angle), Vector3.ZERO)
	if index == 5 and moving_jaw_calibration_enabled:
		_joint_nodes[index].transform = _moving_jaw_transform(angle)
	else:
		_joint_nodes[index].transform = _joint_origin_transforms[index] * rotation
	if index == 4:
		_joint_nodes[index].position += gripper_mount_correction_local
		_apply_gripper_visual_correction(_joint_nodes[index])
	if index == 3 and d455_visual_correction_enabled:
		_joint_nodes[index].position += _d455_distal_correction_local


func _apply_gripper_visual_correction(node: Node3D) -> void:
	node.transform = node.transform * _gripper_visual_correction_transform()


func _gripper_visual_correction_transform() -> Transform3D:
	var preview_rpy := manual_tool_rpy_degrees if manual_claw_calibration_enabled else Vector3.ZERO
	var preview_translation := manual_tool_translation_local if manual_claw_calibration_enabled else Vector3.ZERO
	return Transform3D(
		Basis.from_euler(
			Vector3(
				deg_to_rad(gripper_visual_correction_rpy_degrees.x + preview_rpy.x),
				deg_to_rad(gripper_visual_correction_rpy_degrees.y + preview_rpy.y),
				deg_to_rad(gripper_visual_correction_rpy_degrees.z + preview_rpy.z),
			),
			EULER_ORDER_XYZ,
		).orthonormalized(),
		gripper_visual_correction_translation_local + preview_translation,
	)


func _calibrated_moving_jaw_opening_degrees(normalized: float) -> float:
	var count := mini(
		gripper_angle_samples_normalized.size(),
		moving_jaw_opening_samples_degrees.size(),
	)
	if count < 2:
		return _calibrated_gripper_angle_degrees(normalized)
	for index in range(count - 1):
		var left := float(gripper_angle_samples_normalized[index])
		var right := float(gripper_angle_samples_normalized[index + 1])
		if normalized <= right:
			return lerpf(
				float(moving_jaw_opening_samples_degrees[index]),
				float(moving_jaw_opening_samples_degrees[index + 1]),
				inverse_lerp(left, right, normalized),
			)
	var left := float(gripper_angle_samples_normalized[count - 2])
	var right := float(gripper_angle_samples_normalized[count - 1])
	return lerpf(
		float(moving_jaw_opening_samples_degrees[count - 2]),
		float(moving_jaw_opening_samples_degrees[count - 1]),
		inverse_lerp(left, right, normalized),
	)


func _moving_jaw_transform(opening_radians: float) -> Transform3D:
	var axis := moving_jaw_axis_parent.normalized()
	return Transform3D(
		Basis(axis, opening_radians) * moving_jaw_closed_basis_parent,
		moving_jaw_pivot_parent,
	)


func _apply_moving_jaw_visual_geometry() -> void:
	for links in [_links, _target_ghost_links]:
		var jaw := (links as Dictionary).get("moving_jaw_so101_v1_link", null) as Node3D
		if jaw == null:
			continue
		var visual_name := "TargetGhostVisual" if links == _target_ghost_links else "Visual"
		var visual := jaw.get_node_or_null(visual_name) as Node3D
		if visual == null:
			continue
		visual.scale = (
			Vector3(
				moving_jaw_visual_radial_scale,
				moving_jaw_visual_radial_scale,
				1.0,
			)
			if moving_jaw_calibration_enabled
			else Vector3.ONE
		)
		visual.position = (
			Vector3(0.0, 0.0, moving_jaw_visual_axial_translation_local)
			if moving_jaw_calibration_enabled
			else Vector3.ZERO
		)

func _save_registration() -> bool:
	var payload := _read_registration_payload()
	payload.merge(_transform_to_dictionary(transform), true)
	payload["type"] = "so101_robot_registration"
	payload["kinematics_version"] = ROBOT_KINEMATICS_VERSION
	payload["confidence"] = float(_registration_status.get("confidence", 0.0))
	payload["calibration_started_unix_ms"] = float(_registration_status.get("calibration_started_unix_ms", 0.0))
	payload["saved_unix_ms"] = _registration_status.get("saved_unix_ms", Time.get_unix_time_from_system() * 1000.0)
	payload["registration_source"] = str(_registration_status.get("registration_source", "markerless_motion_fit"))
	payload["joint_angle_directions"] = _packed_float_array_to_array(joint_angle_directions)
	payload["joint_angle_offsets_degrees"] = _packed_float_array_to_array(joint_angle_offsets_degrees)
	payload["force_wrist_roll_straight"] = force_wrist_roll_straight
	payload["gripper_angle_offset_degrees"] = gripper_angle_offset_degrees
	payload["gripper_angle_scale_degrees_per_normalized"] = (
		gripper_angle_scale_degrees_per_normalized
	)
	payload["gripper_angle_curvature_degrees"] = (
		gripper_angle_curvature_degrees
	)
	payload["gripper_angle_samples_normalized"] = (
		_packed_float_array_to_array(gripper_angle_samples_normalized)
	)
	payload["gripper_angle_samples_degrees"] = (
		_packed_float_array_to_array(gripper_angle_samples_degrees)
	)
	payload["gripper_mount_correction_local"] = [
		gripper_mount_correction_local.x,
		gripper_mount_correction_local.y,
		gripper_mount_correction_local.z,
	]
	payload["gripper_visual_correction_rpy_degrees"] = [
		gripper_visual_correction_rpy_degrees.x,
		gripper_visual_correction_rpy_degrees.y,
		gripper_visual_correction_rpy_degrees.z,
	]
	payload["gripper_visual_correction_translation_local"] = [
		gripper_visual_correction_translation_local.x,
		gripper_visual_correction_translation_local.y,
		gripper_visual_correction_translation_local.z,
	]
	payload["moving_jaw_calibration_enabled"] = moving_jaw_calibration_enabled
	payload["moving_jaw_pivot_parent"] = [
		moving_jaw_pivot_parent.x, moving_jaw_pivot_parent.y, moving_jaw_pivot_parent.z,
	]
	payload["moving_jaw_axis_parent"] = [
		moving_jaw_axis_parent.x, moving_jaw_axis_parent.y, moving_jaw_axis_parent.z,
	]
	payload["moving_jaw_closed_basis_parent_row_major"] = [
		moving_jaw_closed_basis_parent.x.x,
		moving_jaw_closed_basis_parent.y.x,
		moving_jaw_closed_basis_parent.z.x,
		moving_jaw_closed_basis_parent.x.y,
		moving_jaw_closed_basis_parent.y.y,
		moving_jaw_closed_basis_parent.z.y,
		moving_jaw_closed_basis_parent.x.z,
		moving_jaw_closed_basis_parent.y.z,
		moving_jaw_closed_basis_parent.z.z,
	]
	payload["moving_jaw_opening_samples_degrees"] = (
		_packed_float_array_to_array(moving_jaw_opening_samples_degrees)
	)
	payload["moving_jaw_visual_radial_scale"] = moving_jaw_visual_radial_scale
	payload["moving_jaw_visual_axial_translation_local"] = (
		moving_jaw_visual_axial_translation_local
	)
	if _registration_status.has("joint_refinement"):
		payload["joint_refinement"] = _registration_status["joint_refinement"]
	if _registration_status.has("base_registration_source"):
		payload["base_registration_source"] = _registration_status["base_registration_source"]
	if _registration_status.has("base_axis_fit"):
		payload["base_axis_fit"] = _registration_status["base_axis_fit"]
	if _registration_status.has("calibrated_through_joint"):
		payload["calibrated_through_joint"] = int(
			_registration_status["calibrated_through_joint"]
		)
	var target_path := ProjectSettings.globalize_path(_registration_path)
	var temporary_path := target_path + ".pending"
	if FileAccess.file_exists(temporary_path):
		DirAccess.remove_absolute(temporary_path)
	var file := FileAccess.open(temporary_path, FileAccess.WRITE)
	if file == null:
		_registration_status["fault"] = "could not save robot registration"
		return false
	file.store_string(JSON.stringify(payload, "\t"))
	file.flush()
	file.close()
	# Keep the prior path untouched until a complete pending file exists. On
	# supported desktop platforms rename is the single atomic commit point and
	# replaces the old file without exposing a partial JSON document.
	_backup_registration_before_write()
	var rename_error := DirAccess.rename_absolute(temporary_path, target_path)
	if rename_error != OK:
		DirAccess.remove_absolute(temporary_path)
		_registration_status["fault"] = (
			"could not atomically commit robot registration (%d)" % rename_error
		)
		return false
	_registration_modified_time = _registration_file_modified_time()
	return true


func _backup_registration_before_write() -> void:
	if not FileAccess.file_exists(_registration_path):
		return
	var source_path := ProjectSettings.globalize_path(_registration_path)
	var stamp := Time.get_datetime_string_from_system().replace(":", "").replace("-", "").replace("T", "_")
	var backup_path := "%s.before_markerless_%s" % [source_path, stamp]
	var source := FileAccess.open(source_path, FileAccess.READ)
	if source == null:
		return
	var backup := FileAccess.open(backup_path, FileAccess.WRITE)
	if backup != null:
		backup.store_buffer(source.get_buffer(source.get_length()))

func _registration_file_modified_time() -> int:
	if not FileAccess.file_exists(_registration_path):
		return 0
	return int(FileAccess.get_modified_time(_registration_path))

func _poll_saved_registration() -> void:
	if not load_saved_registration:
		return
	var now := Time.get_ticks_msec()
	if now < _next_registration_poll_msec:
		return
	_next_registration_poll_msec = now + 500
	var modified := _registration_file_modified_time()
	if modified <= 0:
		if (
			_registration_modified_time > 0
			or bool(_registration_status.get("registered", false))
		):
			_reset_to_scene_fallback()
			_registration_status = {"registered": false}
		_registration_modified_time = 0
		return
	if modified == _registration_modified_time:
		return
	_registration_modified_time = modified
	_load_registration()

func _load_registration() -> void:
	var parsed := _read_registration_payload()
	if parsed.is_empty():
		_reset_to_scene_fallback()
		_registration_status = {"registered": false}
		return
	if int(parsed.get("kinematics_version", 0)) != ROBOT_KINEMATICS_VERSION:
		_registration_status = {
			"registered": false,
			"fault": "Robot joint mapping changed; recalibrate the robot position.",
		}
		return
	var loaded := _transform_from_dictionary(parsed)
	if not loaded.is_finite():
		return
	if not _registration_is_upright(loaded):
		_reset_to_scene_fallback()
		_registration_status = {
			"registered": false,
			"fault": "Rejected saved robot registration: base up axis points toward the table.",
		}
		return
	_apply_saved_joint_mapping(parsed)
	transform = loaded
	_registration_status = {
		"registered": true,
		"confidence": float(parsed.get("confidence", 0.0)),
		"calibration_started_unix_ms": float(parsed.get("calibration_started_unix_ms", 0.0)),
		"saved_unix_ms": float(parsed.get("saved_unix_ms", 0.0)),
		"registration_source": str(parsed.get("registration_source", "legacy")),
	}
	if parsed.has("joint_refinement"):
		_registration_status["joint_refinement"] = parsed["joint_refinement"]
	if parsed.has("base_registration_source"):
		_registration_status["base_registration_source"] = parsed["base_registration_source"]
	if parsed.has("base_axis_fit"):
		_registration_status["base_axis_fit"] = parsed["base_axis_fit"]
	if parsed.has("calibrated_through_joint"):
		_registration_status["calibrated_through_joint"] = int(
			parsed["calibrated_through_joint"]
		)


func apply_joint_offset_refinement(
	offset_deltas_degrees: Array,
	confidence: float,
	calibration_started_unix_ms: float,
	metrics: Dictionary = {},
) -> bool:
	if offset_deltas_degrees.size() < JOINTS.size():
		return false
	var existing := _read_registration_payload()
	var existing_started := float(existing.get("calibration_started_unix_ms", 0.0))
	if _calibration_transaction_is_stale(
		calibration_started_unix_ms,
		existing_started,
	):
		return false
	for index in [1, 2, 3]:
		if not is_finite(float(offset_deltas_degrees[index])):
			return false
		joint_angle_offsets_degrees[index] += float(offset_deltas_degrees[index])
	_registration_status["registered"] = true
	_registration_status["confidence"] = clampf(confidence, 0.0, 1.0)
	_registration_status["calibration_started_unix_ms"] = calibration_started_unix_ms
	_registration_status["saved_unix_ms"] = Time.get_unix_time_from_system() * 1000.0
	_registration_status["registration_source"] = "staged_attachment_safe_joint_refinement"
	var base_source := str(existing.get(
		"base_registration_source",
		existing.get("registration_source", ""),
	))
	# Registrations written by the first staged-refinement revision did not yet
	# retain this field. Its stock-mesh method was only allowed after the settled
	# shoulder-pan axis registration, so migrate that one known case safely.
	if (
		base_source == "staged_attachment_safe_joint_refinement"
		and str(existing.get("joint_refinement", {}).get("method", ""))
			== "base_locked_revolute_axis_stock_mesh"
	):
		base_source = "settled_shoulder_pan_revolute_axis"
	_registration_status["base_registration_source"] = base_source
	_registration_status["joint_refinement"] = metrics
	var values := get_best_available_normalized_pose()
	if values.size() >= 6:
		_set_target_from_normalized(values)
	return _save_registration()


func apply_wrist_roll_calibration(
	absolute_direction: float,
	absolute_offset_degrees: float,
	confidence: float,
	calibration_started_unix_ms: float,
	metrics: Dictionary = {},
) -> bool:
	if (
		absf(absf(absolute_direction) - 1.0) > 0.01
		or not is_finite(absolute_offset_degrees)
	):
		return false
	var existing := _read_registration_payload()
	var existing_started := float(existing.get("calibration_started_unix_ms", 0.0))
	if _calibration_transaction_is_stale(
		calibration_started_unix_ms,
		existing_started,
	):
		return false
	var previous_direction := joint_angle_directions[4]
	var previous_offset := joint_angle_offsets_degrees[4]
	var previous_status := _registration_status.duplicate(true)
	joint_angle_directions[4] = absolute_direction
	joint_angle_offsets_degrees[4] = absolute_offset_degrees
	force_wrist_roll_straight = false
	_registration_status["registered"] = true
	# A distal-only correction must not rewrite the confidence or transaction
	# timestamp of the already validated base/upstream registration. Its own
	# confidence and evidence live under joint_refinement.wrist_roll_zero.
	_registration_status["confidence"] = float(existing.get(
		"confidence",
		clampf(confidence, 0.0, 1.0),
	))
	_registration_status["calibration_started_unix_ms"] = float(existing.get(
		"calibration_started_unix_ms",
		calibration_started_unix_ms,
	))
	_registration_status["saved_unix_ms"] = Time.get_unix_time_from_system() * 1000.0
	_registration_status["joint_refinement"] = (
		existing.get("joint_refinement", {}) as Dictionary
	).duplicate(true)
	_registration_status["joint_refinement"]["wrist_roll_zero"] = metrics.duplicate(true)
	var values := get_best_available_normalized_pose()
	if values.size() >= 6:
		_set_target_from_normalized(values)
	if _save_registration():
		return true
	joint_angle_directions[4] = previous_direction
	joint_angle_offsets_degrees[4] = previous_offset
	_registration_status = previous_status
	if values.size() >= 6:
		_set_target_from_normalized(values)
	return false


func apply_complete_automated_calibration(
	value: Transform3D,
	absolute_directions: Array,
	absolute_offsets_degrees: Array,
	confidence: float,
	calibration_started_unix_ms: float,
	metrics: Dictionary = {},
) -> bool:
	if (
		not value.is_finite()
		or not _registration_is_upright(value)
		or absolute_directions.size() < JOINTS.size()
		or absolute_offsets_degrees.size() < JOINTS.size()
	):
		return false
	for index in range(JOINTS.size()):
		var direction := float(absolute_directions[index])
		if not is_finite(direction) or absf(absf(direction) - 1.0) > 0.01:
			return false
		if not is_finite(float(absolute_offsets_degrees[index])):
			return false
	var existing := _read_registration_payload()
	var existing_started := float(existing.get("calibration_started_unix_ms", 0.0))
	if _calibration_transaction_is_stale(
		calibration_started_unix_ms,
		existing_started,
	):
		return false
	var previous_transform := transform
	var previous_directions := joint_angle_directions.duplicate()
	var previous_offsets := joint_angle_offsets_degrees.duplicate()
	var previous_force_wrist_roll_straight := force_wrist_roll_straight
	var previous_status := _registration_status.duplicate(true)
	transform = Transform3D(value.basis.orthonormalized(), value.origin)
	for index in range(JOINTS.size()):
		joint_angle_directions[index] = float(absolute_directions[index])
		joint_angle_offsets_degrees[index] = float(absolute_offsets_degrees[index])
	# A complete automated result always uses all measured joint encoders.
	force_wrist_roll_straight = false
	_registration_status = {
		"registered": true,
		"confidence": clampf(confidence, 0.0, 1.0),
		"calibration_started_unix_ms": calibration_started_unix_ms,
		"saved_unix_ms": Time.get_unix_time_from_system() * 1000.0,
		"registration_source": "full_automated_motion_axis_calibration",
		"base_registration_source": "settled_shoulder_pan_revolute_axis",
		"calibrated_through_joint": 4,
		"joint_refinement": metrics,
	}
	if metrics.get("base_axis_fit", null) is Dictionary:
		_registration_status["base_axis_fit"] = (
			metrics["base_axis_fit"] as Dictionary
		).duplicate(true)
	var values := get_best_available_normalized_pose()
	if values.size() >= 6:
		_set_target_from_normalized(values)
	if _save_registration():
		return true
	transform = previous_transform
	joint_angle_directions = previous_directions
	joint_angle_offsets_degrees = previous_offsets
	force_wrist_roll_straight = previous_force_wrist_roll_straight
	_registration_status = previous_status
	if values.size() >= 6:
		_set_target_from_normalized(values)
	return false


func apply_automated_claw_calibration(
	result: Dictionary,
	calibration_started_unix_ms: float,
) -> bool:
	var method := str(result.get("method", ""))
	if (
		str(result.get("type", "")) != "so101_claw_visual_fit"
		or method not in [
			"d455_five_state_independent_mesh_fit",
			"d455_native_rgb_multiview_tip_fit",
			"d455_native_rgb_joint_frame_arc_fit",
		]
	):
		return false
	var normalized = result.get("gripper_angle_samples_normalized", null)
	var degrees = result.get("gripper_angle_samples_degrees", null)
	var mount_values = result.get("gripper_mount_correction_local", null)
	var rpy_values = result.get("gripper_visual_correction_rpy_degrees", null)
	var translation_values = result.get(
		"gripper_visual_correction_translation_local",
		null,
	)
	var has_moving_jaw_arc := method == "d455_native_rgb_joint_frame_arc_fit"
	var jaw_pivot_values = result.get("moving_jaw_pivot_parent", null)
	var jaw_axis_values = result.get("moving_jaw_axis_parent", null)
	var jaw_basis_values = result.get("moving_jaw_closed_basis_parent_row_major", null)
	var jaw_opening_values = result.get("moving_jaw_opening_samples_degrees", null)
	if (
		not normalized is Array
		or not degrees is Array
		or (normalized as Array).size() != 5
		or (degrees as Array).size() != 5
		or not mount_values is Array
		or (mount_values as Array).size() < 3
		or not rpy_values is Array
		or (rpy_values as Array).size() < 3
		or not translation_values is Array
		or (translation_values as Array).size() < 3
	):
		return false
	if has_moving_jaw_arc and (
		not jaw_pivot_values is Array
		or (jaw_pivot_values as Array).size() != 3
		or not jaw_axis_values is Array
		or (jaw_axis_values as Array).size() != 3
		or not jaw_basis_values is Array
		or (jaw_basis_values as Array).size() != 9
		or not jaw_opening_values is Array
		or (jaw_opening_values as Array).size() != 5
	):
		return false
	var fitted_normalized := PackedFloat32Array()
	var fitted_degrees := PackedFloat32Array()
	for index in range(5):
		var normalized_value := float((normalized as Array)[index])
		var degree_value := float((degrees as Array)[index])
		if (
			not is_finite(normalized_value)
			or not is_finite(degree_value)
			or (
				index > 0
				and (
					normalized_value <= fitted_normalized[index - 1]
					or degree_value <= fitted_degrees[index - 1]
				)
			)
		):
			return false
		fitted_normalized.append(normalized_value)
		fitted_degrees.append(degree_value)
	if (
		absf(fitted_degrees[0] + 10.0)
			> GRIPPER_CLOSED_ENDPOINT_TOLERANCE_DEGREES
		or fitted_degrees[-1] < 68.0
	):
		return false
	var jaw_pivot := moving_jaw_pivot_parent
	var jaw_axis := moving_jaw_axis_parent
	var jaw_basis := moving_jaw_closed_basis_parent
	var jaw_openings := moving_jaw_opening_samples_degrees.duplicate()
	var jaw_radial_scale := moving_jaw_visual_radial_scale
	var jaw_axial_translation := moving_jaw_visual_axial_translation_local
	if has_moving_jaw_arc:
		jaw_pivot = Vector3(
			float((jaw_pivot_values as Array)[0]), float((jaw_pivot_values as Array)[1]), float((jaw_pivot_values as Array)[2])
		)
		jaw_axis = Vector3(
			float((jaw_axis_values as Array)[0]), float((jaw_axis_values as Array)[1]), float((jaw_axis_values as Array)[2])
		)
		jaw_basis = Basis(
			Vector3(float((jaw_basis_values as Array)[0]), float((jaw_basis_values as Array)[3]), float((jaw_basis_values as Array)[6])),
			Vector3(float((jaw_basis_values as Array)[1]), float((jaw_basis_values as Array)[4]), float((jaw_basis_values as Array)[7])),
			Vector3(float((jaw_basis_values as Array)[2]), float((jaw_basis_values as Array)[5]), float((jaw_basis_values as Array)[8])),
		)
		jaw_openings = PackedFloat32Array()
		var previous_opening := -INF
		for value in jaw_opening_values as Array:
			var opening := float(value)
			if not is_finite(opening) or opening < previous_opening or opening < -5.0 or opening > 140.0:
				return false
			jaw_openings.append(opening)
			previous_opening = opening
		jaw_radial_scale = float(result.get("moving_jaw_visual_radial_scale", 1.0))
		jaw_axial_translation = float(result.get("moving_jaw_visual_axial_translation_local", 0.0))
		if (
			not jaw_pivot.is_finite()
			or jaw_pivot.length() > 0.12
			or not jaw_axis.is_finite()
			or jaw_axis.length() < 0.99
			or jaw_axis.length() > 1.01
			or not jaw_basis.is_finite()
			or absf(jaw_basis.determinant() - 1.0) > 0.02
			or jaw_radial_scale < 0.75
			or jaw_radial_scale > 1.35
			or absf(jaw_axial_translation) > 0.03
		):
			return false
	var mount := Vector3(
		float((mount_values as Array)[0]),
		float((mount_values as Array)[1]),
		float((mount_values as Array)[2]),
	)
	var visual_rpy := Vector3(
		float((rpy_values as Array)[0]),
		float((rpy_values as Array)[1]),
		float((rpy_values as Array)[2]),
	)
	var visual_translation := Vector3(
		float((translation_values as Array)[0]),
		float((translation_values as Array)[1]),
		float((translation_values as Array)[2]),
	)
	if (
		not mount.is_finite()
		or mount.length() > GRIPPER_MOUNT_CORRECTION_MAXIMUM_METERS
		or not visual_rpy.is_finite()
		or visual_rpy.length() > GRIPPER_VISUAL_CORRECTION_MAXIMUM_DEGREES
		or not visual_translation.is_finite()
		or (
			visual_translation.length()
			> GRIPPER_VISUAL_CORRECTION_MAXIMUM_METERS
		)
	):
		return false
	var existing := _read_registration_payload()
	if _calibration_transaction_is_stale(
		calibration_started_unix_ms,
		float(existing.get("calibration_started_unix_ms", 0.0)),
	):
		return false
	var previous_normalized := gripper_angle_samples_normalized.duplicate()
	var previous_degrees := gripper_angle_samples_degrees.duplicate()
	var previous_mount := gripper_mount_correction_local
	var previous_rpy := gripper_visual_correction_rpy_degrees
	var previous_translation := gripper_visual_correction_translation_local
	var previous_jaw_enabled := moving_jaw_calibration_enabled
	var previous_jaw_pivot := moving_jaw_pivot_parent
	var previous_jaw_axis := moving_jaw_axis_parent
	var previous_jaw_basis := moving_jaw_closed_basis_parent
	var previous_jaw_openings := moving_jaw_opening_samples_degrees.duplicate()
	var previous_jaw_scale := moving_jaw_visual_radial_scale
	var previous_jaw_translation := moving_jaw_visual_axial_translation_local
	var previous_status := _registration_status.duplicate(true)
	gripper_angle_samples_normalized = fitted_normalized
	gripper_angle_samples_degrees = fitted_degrees
	gripper_mount_correction_local = mount
	gripper_visual_correction_rpy_degrees = visual_rpy
	gripper_visual_correction_translation_local = visual_translation
	if has_moving_jaw_arc:
		moving_jaw_pivot_parent = jaw_pivot
		moving_jaw_axis_parent = jaw_axis.normalized()
		moving_jaw_closed_basis_parent = jaw_basis.orthonormalized()
		moving_jaw_opening_samples_degrees = jaw_openings
		moving_jaw_visual_radial_scale = jaw_radial_scale
		moving_jaw_visual_axial_translation_local = jaw_axial_translation
		moving_jaw_calibration_enabled = true
		_apply_moving_jaw_visual_geometry()
	var refinement = _registration_status.get("joint_refinement", {})
	var refined := (
		(refinement as Dictionary).duplicate(true)
		if refinement is Dictionary
		else {}
	)
	refined.erase("claw_calibration_pending")
	refined["claw_calibration"] = result.duplicate(true)
	_registration_status["joint_refinement"] = refined
	_registration_status["confidence"] = minf(
		float(_registration_status.get("confidence", 1.0)),
		float(result.get("confidence", 0.90)),
	)
	_registration_status["saved_unix_ms"] = (
		Time.get_unix_time_from_system() * 1000.0
	)
	var values := get_best_available_normalized_pose()
	if values.size() >= 6:
		_set_target_from_normalized(values)
	if _save_registration():
		return true
	gripper_angle_samples_normalized = previous_normalized
	gripper_angle_samples_degrees = previous_degrees
	gripper_mount_correction_local = previous_mount
	gripper_visual_correction_rpy_degrees = previous_rpy
	gripper_visual_correction_translation_local = previous_translation
	moving_jaw_calibration_enabled = previous_jaw_enabled
	moving_jaw_pivot_parent = previous_jaw_pivot
	moving_jaw_axis_parent = previous_jaw_axis
	moving_jaw_closed_basis_parent = previous_jaw_basis
	moving_jaw_opening_samples_degrees = previous_jaw_openings
	moving_jaw_visual_radial_scale = previous_jaw_scale
	moving_jaw_visual_axial_translation_local = previous_jaw_translation
	_apply_moving_jaw_visual_geometry()
	_registration_status = previous_status
	if values.size() >= 6:
		_set_target_from_normalized(values)
	return false


## Completes a full automatic transaction when all three D455 claw views are
## optically ambiguous.  The arm through wrist has already been freshly
## validated at this point.  Never replace a known-good jaw mapping with a
## non-monotonic image fit: validate the currently installed mapping and retain
## every one of its values unchanged while recording the explicit fallback.
func finalize_automated_claw_with_validated_prior(
	calibration_started_unix_ms: float,
	optical_failure: Dictionary = {},
) -> bool:
	if (
		str(_registration_status.get("registration_source", ""))
			!= "full_automated_motion_axis_calibration"
		or int(_registration_status.get("calibrated_through_joint", -1)) < 4
		or not _registration_is_upright(transform)
	):
		return false
	var refinement = _registration_status.get("joint_refinement", null)
	if (
		not refinement is Dictionary
		or not bool((refinement as Dictionary).get(
			"claw_calibration_pending",
			false,
		))
	):
		return false
	if (
		gripper_angle_samples_normalized.size() != 5
		or gripper_angle_samples_degrees.size() != 5
	):
		return false
	for index in range(5):
		var normalized := float(gripper_angle_samples_normalized[index])
		var degrees := float(gripper_angle_samples_degrees[index])
		if (
			not is_finite(normalized)
			or not is_finite(degrees)
			or (
				index > 0
				and (
					normalized
						<= float(gripper_angle_samples_normalized[index - 1])
					or degrees
						<= float(gripper_angle_samples_degrees[index - 1])
				)
			)
		):
			return false
	if (
		absf(float(gripper_angle_samples_degrees[0]) + 10.0)
			> GRIPPER_CLOSED_ENDPOINT_TOLERANCE_DEGREES
		or float(gripper_angle_samples_degrees[-1]) < 68.0
		or not gripper_mount_correction_local.is_finite()
		or gripper_mount_correction_local.length()
			> GRIPPER_MOUNT_CORRECTION_MAXIMUM_METERS
		or not gripper_visual_correction_rpy_degrees.is_finite()
		or gripper_visual_correction_rpy_degrees.length()
			> GRIPPER_VISUAL_CORRECTION_MAXIMUM_DEGREES
		or not gripper_visual_correction_translation_local.is_finite()
		or gripper_visual_correction_translation_local.length()
			> GRIPPER_VISUAL_CORRECTION_MAXIMUM_METERS
	):
		return false
	var existing := _read_registration_payload()
	if _calibration_transaction_is_stale(
		calibration_started_unix_ms,
		float(existing.get("calibration_started_unix_ms", 0.0)),
	):
		return false
	var previous_status := _registration_status.duplicate(true)
	var refined := (refinement as Dictionary).duplicate(true)
	refined.erase("claw_calibration_pending")
	refined["claw_calibration"] = {
		"method": (
			"preserved_validated_prior_claw_curve_after_three_ambiguous_d455_views"
		),
		"fallback": true,
		"optical_attempts": 3,
		"reason": str(optical_failure.get(
			"status",
			optical_failure.get("reason", "D455 moving-jaw evidence was ambiguous"),
		)),
	}
	_registration_status["joint_refinement"] = refined
	_registration_status["saved_unix_ms"] = (
		Time.get_unix_time_from_system() * 1000.0
	)
	if _save_registration():
		return true
	_registration_status = previous_status
	return false


## Atomically commits the fresh base and only the contiguous, D455-validated
## servo prefix. Distal mappings remain byte-for-byte unchanged until their own
## outward stage passes, so a wrist failure cannot discard a valid base,
## shoulder, or elbow registration.
func apply_validated_automated_prefix(
	value: Transform3D,
	absolute_directions: Array,
	absolute_offsets_degrees: Array,
	calibrated_through_joint: int,
	confidence: float,
	calibration_started_unix_ms: float,
	metrics: Dictionary = {},
) -> bool:
	if (
		not value.is_finite()
		or not _registration_is_upright(value)
		or calibrated_through_joint < 0
		or calibrated_through_joint > 2
		or absolute_directions.size() < JOINTS.size()
		or absolute_offsets_degrees.size() < JOINTS.size()
	):
		return false
	for index in range(JOINTS.size()):
		var direction := float(absolute_directions[index])
		if not is_finite(direction) or absf(absf(direction) - 1.0) > 0.01:
			return false
		if not is_finite(float(absolute_offsets_degrees[index])):
			return false
	var existing := _read_registration_payload()
	var existing_started := float(existing.get("calibration_started_unix_ms", 0.0))
	if _calibration_transaction_is_stale(
		calibration_started_unix_ms,
		existing_started,
	):
		return false
	var previous_transform := transform
	var previous_directions := joint_angle_directions.duplicate()
	var previous_offsets := joint_angle_offsets_degrees.duplicate()
	var previous_force_wrist_roll_straight := force_wrist_roll_straight
	var previous_status := _registration_status.duplicate(true)
	transform = Transform3D(value.basis.orthonormalized(), value.origin)
	for index in range(1, calibrated_through_joint + 1):
		joint_angle_directions[index] = float(absolute_directions[index])
		joint_angle_offsets_degrees[index] = float(
			absolute_offsets_degrees[index]
		)
	# This checkpoint intentionally preserves the unvalidated wrist-roll policy.
	# The complete transaction disables the legacy straight-wrist workaround
	# only after fresh wrist evidence passes.
	_registration_status = {
		"registered": true,
		"confidence": clampf(confidence, 0.0, 1.0),
		"calibration_started_unix_ms": calibration_started_unix_ms,
		"saved_unix_ms": Time.get_unix_time_from_system() * 1000.0,
		"registration_source": "partial_automated_motion_axis_calibration",
		"base_registration_source": "settled_shoulder_pan_revolute_axis",
		"calibrated_through_joint": calibrated_through_joint,
		"joint_refinement": metrics.duplicate(true),
	}
	if metrics.get("base_axis_fit", null) is Dictionary:
		_registration_status["base_axis_fit"] = (
			metrics["base_axis_fit"] as Dictionary
		).duplicate(true)
	var values := get_best_available_normalized_pose()
	if values.size() >= 6:
		_set_target_from_normalized(values)
	if _save_registration():
		return true
	transform = previous_transform
	joint_angle_directions = previous_directions
	joint_angle_offsets_degrees = previous_offsets
	force_wrist_roll_straight = previous_force_wrist_roll_straight
	_registration_status = previous_status
	if values.size() >= 6:
		_set_target_from_normalized(values)
	return false


func _apply_saved_joint_mapping(payload: Dictionary) -> void:
	var saved_directions = payload.get("joint_angle_directions", null)
	var saved_offsets = payload.get("joint_angle_offsets_degrees", null)
	if saved_directions is Array and (saved_directions as Array).size() >= JOINTS.size():
		for index in range(1, JOINTS.size()):
			var direction := float((saved_directions as Array)[index])
			if is_finite(direction) and absf(absf(direction) - 1.0) <= 0.01:
				joint_angle_directions[index] = direction
	if saved_offsets is Array and (saved_offsets as Array).size() >= JOINTS.size():
		for index in range(1, JOINTS.size()):
			var offset := float((saved_offsets as Array)[index])
			if is_finite(offset) and absf(offset) <= 360.0:
				joint_angle_offsets_degrees[index] = offset
	if payload.has("force_wrist_roll_straight"):
		var saved_force_straight := bool(payload["force_wrist_roll_straight"])
		# Migrate the old full-auto visual workaround in memory. It discarded
		# valid wrist telemetry and could displace the claw tip by about 30 mm.
		force_wrist_roll_straight = (
			saved_force_straight
			and str(payload.get("registration_source", ""))
				!= "full_automated_motion_axis_calibration"
		)
	var saved_gripper_offset := float(payload.get(
		"gripper_angle_offset_degrees",
		gripper_angle_offset_degrees,
	))
	var saved_gripper_scale := float(payload.get(
		"gripper_angle_scale_degrees_per_normalized",
		gripper_angle_scale_degrees_per_normalized,
	))
	var saved_gripper_curvature := float(payload.get(
		"gripper_angle_curvature_degrees",
		GRIPPER_ANGLE_CURVATURE_DEFAULT_DEGREES,
	))
	if (
		is_finite(saved_gripper_offset)
		and absf(saved_gripper_offset) <= 360.0
		and is_finite(saved_gripper_scale)
		and absf(saved_gripper_scale) <= 5.0
		and is_finite(saved_gripper_curvature)
		and absf(saved_gripper_curvature) <= 5.0
		and _gripper_calibration_matches_closed_endpoint(
			saved_gripper_offset,
			saved_gripper_scale,
			saved_gripper_curvature,
		)
	):
		gripper_angle_offset_degrees = saved_gripper_offset
		gripper_angle_scale_degrees_per_normalized = saved_gripper_scale
		gripper_angle_curvature_degrees = saved_gripper_curvature
	else:
		# A prior image-only fit extrapolated outside its sampled open poses and
		# mapped the physical closed endpoint to a 66-degree-open digital jaw.
		# Keep the endpoint-preserving URDF linkage map instead.
		gripper_angle_offset_degrees = GRIPPER_ANGLE_OFFSET_DEFAULT_DEGREES
		gripper_angle_scale_degrees_per_normalized = (
			GRIPPER_ANGLE_SCALE_DEFAULT_DEGREES
		)
		gripper_angle_curvature_degrees = (
			GRIPPER_ANGLE_CURVATURE_DEFAULT_DEGREES
		)
	var saved_mount = payload.get("gripper_mount_correction_local", null)
	if saved_mount is Array and (saved_mount as Array).size() >= 3:
		var correction := Vector3(
			float((saved_mount as Array)[0]),
			float((saved_mount as Array)[1]),
			float((saved_mount as Array)[2])
		)
		if (
			correction.is_finite()
			and correction.length()
				<= GRIPPER_MOUNT_CORRECTION_MAXIMUM_METERS
		):
			gripper_mount_correction_local = correction
	_apply_saved_gripper_angle_samples(payload)
	_apply_saved_gripper_visual_correction(payload)
	_apply_saved_moving_jaw_calibration(payload)


func _apply_saved_gripper_angle_samples(payload: Dictionary) -> void:
	var saved_normalized = payload.get(
		"gripper_angle_samples_normalized",
		null,
	)
	var saved_degrees = payload.get(
		"gripper_angle_samples_degrees",
		null,
	)
	if (
		not saved_normalized is Array
		or not saved_degrees is Array
		or (saved_normalized as Array).size() < 2
		or (saved_normalized as Array).size()
			!= (saved_degrees as Array).size()
	):
		return
	var normalized := PackedFloat32Array()
	var degrees := PackedFloat32Array()
	var previous := -INF
	for index in range((saved_normalized as Array).size()):
		var sample_normalized := float(
			(saved_normalized as Array)[index]
		)
		var sample_degrees := float((saved_degrees as Array)[index])
		if (
			not is_finite(sample_normalized)
			or not is_finite(sample_degrees)
			or sample_normalized <= previous
			or sample_normalized < -20.0
			or sample_normalized > 120.0
			or absf(sample_degrees) > 180.0
		):
			return
		normalized.append(sample_normalized)
		degrees.append(sample_degrees)
		previous = sample_normalized
	gripper_angle_samples_normalized = normalized
	gripper_angle_samples_degrees = degrees


func _apply_saved_gripper_visual_correction(payload: Dictionary) -> void:
	var saved_rpy = payload.get(
		"gripper_visual_correction_rpy_degrees",
		null,
	)
	if saved_rpy is Array and (saved_rpy as Array).size() >= 3:
		var rpy := Vector3(
			float((saved_rpy as Array)[0]),
			float((saved_rpy as Array)[1]),
			float((saved_rpy as Array)[2]),
		)
		if (
			rpy.is_finite()
			and rpy.length()
				<= GRIPPER_VISUAL_CORRECTION_MAXIMUM_DEGREES
		):
			gripper_visual_correction_rpy_degrees = rpy
	var saved_translation = payload.get(
		"gripper_visual_correction_translation_local",
		null,
	)
	if (
		saved_translation is Array
		and (saved_translation as Array).size() >= 3
	):
		var translation := Vector3(
			float((saved_translation as Array)[0]),
			float((saved_translation as Array)[1]),
			float((saved_translation as Array)[2]),
		)
		if (
			translation.is_finite()
			and translation.length()
				<= GRIPPER_VISUAL_CORRECTION_MAXIMUM_METERS
		):
			gripper_visual_correction_translation_local = translation


func _apply_saved_moving_jaw_calibration(payload: Dictionary) -> void:
	if not bool(payload.get("moving_jaw_calibration_enabled", false)):
		moving_jaw_calibration_enabled = false
		_apply_moving_jaw_visual_geometry()
		return
	var pivot_values = payload.get("moving_jaw_pivot_parent", null)
	var axis_values = payload.get("moving_jaw_axis_parent", null)
	var basis_values = payload.get("moving_jaw_closed_basis_parent_row_major", null)
	var opening_values = payload.get("moving_jaw_opening_samples_degrees", null)
	if (
		not pivot_values is Array
		or (pivot_values as Array).size() != 3
		or not axis_values is Array
		or (axis_values as Array).size() != 3
		or not basis_values is Array
		or (basis_values as Array).size() != 9
		or not opening_values is Array
		or (opening_values as Array).size() != 5
	):
		return
	var pivot := Vector3(
		float((pivot_values as Array)[0]),
		float((pivot_values as Array)[1]),
		float((pivot_values as Array)[2]),
	)
	var axis := Vector3(
		float((axis_values as Array)[0]),
		float((axis_values as Array)[1]),
		float((axis_values as Array)[2]),
	)
	var basis := Basis(
		Vector3(float((basis_values as Array)[0]), float((basis_values as Array)[3]), float((basis_values as Array)[6])),
		Vector3(float((basis_values as Array)[1]), float((basis_values as Array)[4]), float((basis_values as Array)[7])),
		Vector3(float((basis_values as Array)[2]), float((basis_values as Array)[5]), float((basis_values as Array)[8])),
	)
	var openings := PackedFloat32Array()
	var previous := -INF
	for value in opening_values as Array:
		var opening := float(value)
		if not is_finite(opening) or opening < previous or opening < -5.0 or opening > 140.0:
			return
		openings.append(opening)
		previous = opening
	var radial_scale := float(payload.get("moving_jaw_visual_radial_scale", 1.0))
	var axial_translation := float(payload.get("moving_jaw_visual_axial_translation_local", 0.0))
	if (
		not pivot.is_finite()
		or pivot.length() > 0.12
		or not axis.is_finite()
		or axis.length() < 0.99
		or axis.length() > 1.01
		or not basis.is_finite()
		or absf(basis.determinant() - 1.0) > 0.02
		or not is_finite(radial_scale)
		or radial_scale < 0.75
		or radial_scale > 1.35
		or not is_finite(axial_translation)
		or absf(axial_translation) > 0.03
	):
		return
	moving_jaw_pivot_parent = pivot
	moving_jaw_axis_parent = axis.normalized()
	moving_jaw_closed_basis_parent = basis.orthonormalized()
	moving_jaw_opening_samples_degrees = openings
	moving_jaw_visual_radial_scale = radial_scale
	moving_jaw_visual_axial_translation_local = axial_translation
	moving_jaw_calibration_enabled = true
	_apply_moving_jaw_visual_geometry()


func _packed_float_array_to_array(values: PackedFloat32Array) -> Array:
	var result: Array = []
	for value in values:
		result.append(float(value))
	return result

func _read_registration_payload() -> Dictionary:
	return _read_registration_payload_from(_registration_path)

func _read_registration_payload_from(path: String) -> Dictionary:
	if not FileAccess.file_exists(path):
		return {}
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		return {}
	var parsed = JSON.parse_string(file.get_as_text())
	return parsed as Dictionary if parsed is Dictionary else {}

func _transform_to_dictionary(value: Transform3D) -> Dictionary:
	return {
		"basis_x": [value.basis.x.x, value.basis.x.y, value.basis.x.z],
		"basis_y": [value.basis.y.x, value.basis.y.y, value.basis.y.z],
		"basis_z": [value.basis.z.x, value.basis.z.y, value.basis.z.z],
		"origin": [value.origin.x, value.origin.y, value.origin.z],
	}

func _transform_from_dictionary(payload: Dictionary) -> Transform3D:
	for key in ["basis_x", "basis_y", "basis_z", "origin"]:
		if not payload.get(key, null) is Array or (payload[key] as Array).size() < 3:
			return Transform3D(Basis(Vector3.INF, Vector3.INF, Vector3.INF), Vector3.INF)
	var bx: Array = payload["basis_x"]
	var by: Array = payload["basis_y"]
	var bz: Array = payload["basis_z"]
	var o: Array = payload["origin"]
	return Transform3D(
		Basis(
			Vector3(float(bx[0]), float(bx[1]), float(bx[2])),
			Vector3(float(by[0]), float(by[1]), float(by[2])),
			Vector3(float(bz[0]), float(bz[1]), float(bz[2]))
		),
		Vector3(float(o[0]), float(o[1]), float(o[2]))
	)
