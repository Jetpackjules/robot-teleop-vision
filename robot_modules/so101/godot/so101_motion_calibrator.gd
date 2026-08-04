@tool
extends Node

const OVERLAY_PATH := "../WorldLevelAnchor/RobotOverlay"
const CAMERA_ROOT_PATH := "../WorldLevelAnchor/CameraClouds"
const REALSENSE_ARUCO_GROUND_TRUTH_PATH := "user://realsense_alignment_ground_truth.json"
const STATUS_TYPE := "robot_calibration_status"
const MAX_SOLVE_FRAMES := 24
const TRUSTED_HEADING_LIMIT_DEGREES := 8.0
const TRUSTED_TRANSLATION_LIMIT_M := 0.02
const TRUSTED_LIMIT_SATURATION_RATIO := 0.975
const DEPTH_VALIDATION_SAMPLE_STEP := 4
const DEPTH_VALIDATION_KEEP_FRACTION := 0.38
const DEPTH_VALIDATION_MAX_FRAMES := 4
const MINIMUM_SEMANTIC_MOTION_CONFIDENCE := 0.72
const MAXIMUM_SEMANTIC_MOTION_LOSS_M := 0.035
const MINIMUM_SECONDARY_CONFIRMATION_CONFIDENCE := 0.35
const MAXIMUM_SECONDARY_CONFIRMATION_LOSS_M := 0.075
const ATTACHMENT_SAFE_BASE_LINKS := [
	"base_link",
	"shoulder_link",
	"upper_arm_link",
	"lower_arm_link",
	"wrist_link",
]
const REFINABLE_JOINT_NAMES := {
	1: "shoulder_lift",
	2: "elbow_flex",
	3: "wrist_flex",
}
const JOINT_REFINEMENT_LIMIT_DEGREES := 15.0
const MINIMUM_BASE_PAN_COVERAGE_DEGREES := 52.0
const AUTOMATED_BASE_CAPTURE_PATH := "user://so101_automated_base_axis_capture.json"
const AUTOMATED_BASE_RESULT_PATH := "user://so101_automated_base_axis_fit.json"
const AUTOMATED_JOINT_CAPTURE_PATH := "user://so101_automated_joint_capture.json"
const AUTOMATED_JOINT_RESULT_PATH := "user://so101_automated_joint_fit.json"
const AUTOMATED_CLAW_CAPTURE_PATH := "user://so101_automated_claw_capture.json"
const AUTOMATED_CLAW_RESULT_PATH := "user://so101_automated_claw_fit.json"
const AUTOMATED_CLAW_RGB_DIRECTORY := "user://so101_automated_claw_rgb"
const AUTOMATED_CANDIDATE_REGISTRATION_PATH := "user://so101_automated_candidate_registration.json"
const WRIST_CAPTURE_PATH := "user://so101_wrist_roll_capture.json"
const WRIST_RESULT_PATH := "user://so101_wrist_roll_fit.json"
const SAVED_REGISTRATION_PATH := "user://so101_robot_registration.json"
const BASE_AXIS_SOLVER_PATH := "res://robot_modules/so101/tools/solve_so101_base_axis.py"
const STAGED_JOINT_SOLVER_PATH := "res://robot_modules/so101/tools/solve_so101_staged_joints.py"
const CLAW_VISUAL_SOLVER_PATH := "res://robot_modules/so101/tools/solve_so101_claw_visual.py"
const CLAW_RGB_TIP_SOLVER_PATH := "res://robot_modules/so101/tools/solve_so101_claw_rgb_tips.py"
const DEVELOPER_PROPERTY_NAMES := [
	"capture_seconds",
	"maximum_capture_seconds",
	"sample_interval_seconds",
	"depth_sample_step",
	"motion_threshold_m",
	"maximum_points_per_frame",
	"minimum_moving_points",
	"minimum_pose_frames",
	"maximum_encoder_settle_error_degrees",
	"editor_auto_move_follower",
	"follower_command_port",
	"editor_sweep_delay_seconds",
	"editor_sweep_minimum_seconds",
	"diagnostic_only",
	"base_axis_full_points_per_camera",
	"solver_revision",
	"minimum_confidence",
	"inlier_distance_m",
	"calibration_primary_camera_match",
	"lock_base_up_to_current",
	"constrain_base_to_aruco_plane",
	"aruco_base_clearance_m",
	"minimum_base_sweep_for_heading_fit_degrees",
	"maximum_ambiguous_heading_change_degrees",
	"required_loss_improvement_for_heading_change",
	"claw_tip_fit_weight",
	"maximum_joint_offset_adjustment_degrees",
	"joint_offset_regularization_m_per_degree",
	"calibration_status_port",
	"debug_capture_path",
	"passive_d455_visual_correction_enabled",
	"passive_d455_interval_seconds",
	"passive_d455_pair_distance_m",
	"passive_d455_minimum_pairs",
]

@export_group("Capture")
@export_range(4.0, 24.0, 0.5, "suffix:s") var capture_seconds: float = 15.0
@export_range(8.0, 180.0, 0.5, "suffix:s") var maximum_capture_seconds: float = 150.0
@export_range(0.08, 0.5, 0.01, "suffix:s") var sample_interval_seconds: float = 0.12
@export_range(2, 32, 1) var depth_sample_step: int = 4
@export_range(0.004, 0.08, 0.001, "suffix:m") var motion_threshold_m: float = 0.012
@export_range(100, 2400, 25) var maximum_points_per_frame: int = 1200
@export_range(20, 500, 10) var minimum_moving_points: int = 70
@export_range(5, 30, 1) var minimum_pose_frames: int = 9
# This is only a gross command-tracking sanity gate. Calibration geometry uses
# the fresh physical encoder pose below, so normal gravity sag/backlash must not
# discard an otherwise stationary measured sample.
@export_range(1.0, 8.0, 0.25, "suffix:deg") var maximum_encoder_settle_error_degrees: float = 8.0
## In the editor, explicitly starting calibration requests the follower's bounded, tested motion sweep after capturing a clean baseline.
@export var editor_auto_move_follower: bool = true
@export var follower_command_port: int = 4248
@export_range(0.25, 2.0, 0.05, "suffix:s") var editor_sweep_delay_seconds: float = 0.75
@export_range(10.0, 24.0, 0.5, "suffix:s") var editor_sweep_minimum_seconds: float = 15.0

@export_group("Diagnostics")
## Solves and reports the capture without applying or saving the result.
@export var diagnostic_only: bool = false
@export_range(1000, 60000, 1000) var base_axis_full_points_per_camera: int = 30000

@export_group("Fit")
@export var solver_revision: String = "semantic_motion_v2_no_depth_adjustment"
@export_range(0.25, 0.95, 0.01) var minimum_confidence: float = 0.50
@export_range(0.02, 0.15, 0.005, "suffix:m") var inlier_distance_m: float = 0.065
## Optional model/name fragment for the primary calibration camera. If no
## matching camera is live, calibration chooses the camera with the broadest
## point coverage instead of depending on a device serial.
@export var calibration_primary_camera_match: String = "D455"
## Restricts fitting to yaw around the currently loaded up axis. Leave this off
## for portable camera-only calibration from an unknown camera/environment pose.
@export var lock_base_up_to_current: bool = false
## When the latest native ArUco calibration retained the marker pose, use its
## physical plane as the robot's base plane. Motion still solves X/Z and yaw.
@export var constrain_base_to_aruco_plane: bool = false
@export_range(0.0, 0.02, 0.0005, "suffix:m") var aruco_base_clearance_m: float = 0.0025
## A deliberate base-joint sweep makes yaw observable even while base up remains locked.
@export_range(5.0, 45.0, 1.0, "suffix:deg") var minimum_base_sweep_for_heading_fit_degrees: float = 12.0
## Sparse motion silhouettes can be rotationally ambiguous. Keep a good saved heading unless depth wins decisively.
@export_range(5.0, 45.0, 1.0, "suffix:deg") var maximum_ambiguous_heading_change_degrees: float = 18.0
@export_range(0.05, 0.5, 0.01) var required_loss_improvement_for_heading_change: float = 0.30
## Keep this at zero for a customized wrist-camera assembly. The gripper and
## moving jaw are intentionally excluded from initial base registration.
@export_range(0.0, 0.85, 0.05) var claw_tip_fit_weight: float = 0.0
@export_group("Staged Joint Refinement")
@export_range(0.0, 20.0, 0.5, "suffix:deg") var maximum_joint_offset_adjustment_degrees: float = 15.0
@export_range(0.0, 0.001, 0.00001, "suffix:m") var joint_offset_regularization_m_per_degree: float = 0.00001
@export var calibration_status_port: int = 4251
@export_file("*.json") var debug_capture_path: String = "user://so101_motion_capture_latest.json"
@export_group("Passive D455 Distal Tracking")
@export var passive_d455_visual_correction_enabled: bool = false
@export_range(0.1, 1.0, 0.05, "suffix:s") var passive_d455_interval_seconds: float = 0.25
@export_range(0.005, 0.04, 0.001, "suffix:m") var passive_d455_pair_distance_m: float = 0.025
@export_range(8, 80, 1) var passive_d455_minimum_pairs: int = 18

var _state := "idle"
var _status: Dictionary = {
	"type": STATUS_TYPE,
	"state": "idle",
	"progress": 0.0,
	"frames": 0,
	"confidence": 0.0,
	"message": "Arm position is not calibrated in this session.",
}
var _frames: Array = []
var _camera_previous: Dictionary = {}
var _camera_sequences: Dictionary = {}
var _last_accepted_pose: Array = []
var _started_msec := 0
var _started_unix_ms := 0.0
var _next_sample_msec := 0
var _last_status_send_msec := 0
var _solve_thread := Thread.new()
var _status_udp := PacketPeerUDP.new()
var _arm_command_udp := PacketPeerUDP.new()
var _capture_diagnostic := "Waiting to sample telemetry and depth."
var _capture_renderer_count := 0
var _capture_moving_points := 0
var _editor_sweep_due_msec := 0
var _editor_sweep_requested := false
var _editor_sweep_requested_msec := 0
var _editor_sweep_acknowledged := false
var _preflight_sequences: Dictionary = {}
var _preflight_ready_samples := 0
var _next_preflight_msec := 0
var _auto_move_this_capture := false
var _editor_status_until_msec := 0
var _baseline_capsules := PackedVector4Array()
var _baseline_joint_frames: Array = []
var _capture_mode := "base"
var _last_solution: Dictionary = {}
var _automation_active := false
var _automation_stage := ""
var _automation_force_editor_auto_move := false
var _automation_previous_diagnostic_only := false
var _automation_previous_debug_capture_path := ""
var _automation_base_result: Dictionary = {}
var _automation_base_capture_attempt := 0
var _automation_base_accumulated_frames: Array = []
var _automation_joint_capture_attempt := 0
var _automation_joint_accumulated_frames: Array = []
var _automation_solver_through_joint := 0
var _automation_next_solver_through_joint := 1
## Highest freshly validated stage already committed by this button press.
## -1 means the pre-run registration is still active, 0 means base only, and
## 1/2 mean the corresponding proximal joint prefix is also fresh.
var _automation_applied_through_joint := -1
var _automation_latest_joint_result: Dictionary = {}
var _automation_claw_result: Dictionary = {}
var _automation_claw_capture_attempt := 0
var _automation_claw_accumulated_frames: Array = []
var _next_passive_d455_msec := 0
var _passive_d455_contact_frames := 0
var _passive_d455_status: Dictionary = {
	"enabled": true,
	"tracking": false,
	"confidence": 0.0,
	"diagnostic": "waiting for D455 depth",
}
const MAXIMUM_AUTOMATED_BASE_CAPTURE_ATTEMPTS := 3
const MAXIMUM_AUTOMATED_JOINT_CAPTURE_ATTEMPTS := 3
const MAXIMUM_AUTOMATED_CLAW_CAPTURE_ATTEMPTS := 5
# A complete camera-aware base/joint solve can itself take more than 30 minutes
# on the laptop. Allow one hour to resume only its explicitly pending claw
# stage; upright/source/transaction checks still reject unrelated registrations.
const PENDING_CLAW_RESUME_MAXIMUM_AGE_MSEC := 60.0 * 60.0 * 1000.0


func _validate_property(property: Dictionary) -> void:
	if str(property.get("name", "")) in DEVELOPER_PROPERTY_NAMES:
		property["usage"] = PROPERTY_USAGE_STORAGE


func _ready() -> void:
	add_to_group("robot_calibrator")
	_status_udp.connect_to_host("127.0.0.1", calibration_status_port)
	_connect_arm_command_peer()
	set_process(true)
	var overlay := get_node_or_null(OVERLAY_PATH)
	if overlay != null and overlay.has_method("get_registration_status"):
		var registration: Dictionary = overlay.call("get_registration_status")
		if bool(registration.get("registered", false)):
			var source := str(registration.get("registration_source", ""))
			var unvalidated_marker_guess := (
				source == "marker_grounded_latest_native_aruco+base_zero_v10"
				or source.begins_with("unvalidated_marker_relationship_guess")
			)
			_state = "failed" if unvalidated_marker_guess else "complete"
			_status = {
				"type": STATUS_TYPE,
				"state": _state,
				"progress": 0.0 if unvalidated_marker_guess else 1.0,
				"frames": 0,
				"confidence": 0.0 if unvalidated_marker_guess else float(registration.get("confidence", 0.0)),
				"message": (
					"Marker relationship is only a placement guess; run Locate Robot Arm."
					if unvalidated_marker_guess
					else "Saved arm position calibration loaded."
				),
				"calibration_mode": "base",
			}
	if not Engine.is_editor_hint():
		_emit_status(true)


func _exit_tree() -> void:
	if _solve_thread.is_started():
		_solve_thread.wait_to_finish()
	_stop_editor_sweep("scene closed")
	_status_udp.close()
	_arm_command_udp.close()


func _process(_delta: float) -> void:
	if _editor_sweep_due_msec > 0 and Time.get_ticks_msec() >= _editor_sweep_due_msec:
		_update_editor_sweep_preflight()
	if _state == "capturing":
		_update_capture()
	elif _state == "solving" and _solve_thread.is_started() and not _solve_thread.is_alive():
		var result: Dictionary = _solve_thread.wait_to_finish()
		if _automation_active:
			_finish_automated_stage(result)
		else:
			_finish_solution(result)
	elif Time.get_ticks_msec() >= _next_passive_d455_msec:
		_update_passive_d455_visual_correction()
	if Time.get_ticks_msec() - _last_status_send_msec >= 500:
		_emit_status(false)

func set_passive_d455_visual_correction_enabled(enabled: bool) -> void:
	passive_d455_visual_correction_enabled = enabled
	_passive_d455_status["enabled"] = enabled
	if not enabled:
		_passive_d455_contact_frames = 0
		_passive_d455_status["tracking"] = false

func apply_remote_feedback_settings(settings: Dictionary) -> void:
	set_passive_d455_visual_correction_enabled(bool(settings.get(
		"d455_visual_correction_enabled",
		false,
	)))
	_connect_arm_command_peer()
	_arm_command_udp.put_packet(JSON.stringify({
		"type": "arm_feedback_settings",
		"measured_feedback_enabled": bool(settings.get("measured_feedback_enabled", true)),
		"target_ghost_enabled": bool(settings.get("target_ghost_enabled", true)),
		"following_error_safety_enabled": bool(settings.get("following_error_safety_enabled", true)),
		"freeze_overlay_on_stale_enabled": bool(settings.get("freeze_overlay_on_stale_enabled", true)),
		"d455_visual_correction_enabled": bool(settings.get("d455_visual_correction_enabled", false)),
		"control_session": "godot-remote-feedback-settings",
	}).to_utf8_buffer())

func get_passive_d455_visual_correction_status() -> Dictionary:
	return _passive_d455_status.duplicate(true)

func _update_passive_d455_visual_correction() -> void:
	_next_passive_d455_msec = (
		Time.get_ticks_msec()
		+ maxi(100, int(passive_d455_interval_seconds * 1000.0))
	)
	if not passive_d455_visual_correction_enabled:
		return
	var overlay := get_node_or_null(OVERLAY_PATH)
	if (
		overlay == null
		or not overlay.has_method("get_link_surface_points_world")
		or not overlay.has_method("apply_d455_distal_visual_correction")
	):
		_passive_d455_status["tracking"] = false
		_passive_d455_status["diagnostic"] = "robot overlay unavailable"
		return
	var reference: Node3D
	for renderer_variant in _active_depth_renderers():
		var renderer := renderer_variant as Node3D
		var renderer_name := _depth_renderer_name(renderer)
		if "D455" in renderer_name:
			reference = renderer
			break
	if reference == null:
		_passive_d455_status["tracking"] = false
		_passive_d455_status["diagnostic"] = "D455 depth unavailable"
		return
	var model_points: PackedVector3Array = overlay.call(
		"get_link_surface_points_world",
		"wrist_link",
		140,
	)
	if model_points.size() < 12:
		return
	var model_center := _points_centroid(model_points)
	var observed_all := _full_points_for_renderer(reference)
	var observed := PackedVector3Array()
	for point in observed_all:
		if point.distance_to(model_center) <= 0.12:
			observed.append(point)
	if observed.size() < passive_d455_minimum_pairs:
		_passive_d455_status["tracking"] = false
		_passive_d455_status["diagnostic"] = "too few D455 points near wrist"
		return
	var deltas := PackedVector3Array()
	var pair_limit_squared := passive_d455_pair_distance_m * passive_d455_pair_distance_m
	for model_point in model_points:
		var nearest := Vector3.ZERO
		var nearest_squared := pair_limit_squared
		var found := false
		for observed_point in observed:
			var distance_squared := model_point.distance_squared_to(observed_point)
			if distance_squared < nearest_squared:
				nearest_squared = distance_squared
				nearest = observed_point
				found = true
		if found:
			deltas.append(nearest - model_point)
	if deltas.size() < passive_d455_minimum_pairs:
		_passive_d455_status["tracking"] = false
		_passive_d455_status["diagnostic"] = "D455/model wrist overlap is ambiguous"
		return
	var residual := _points_coordinate_median(deltas)
	var residual_distances: Array[float] = []
	for delta in deltas:
		residual_distances.append(delta.distance_to(residual))
	var scatter := _median(residual_distances)
	var confidence := clampf(
		float(deltas.size()) / 55.0,
		0.0,
		1.0,
	) * clampf(1.0 - scatter / 0.014, 0.0, 1.0)
	if scatter > 0.010 or confidence < 0.42 or residual.length() > 0.03:
		_passive_d455_status["tracking"] = false
		_passive_d455_status["confidence"] = confidence
		_passive_d455_status["diagnostic"] = "D455 wrist fit failed confidence gate"
		return
	overlay.call("apply_d455_distal_visual_correction", residual, confidence)
	_passive_d455_status = {
		"enabled": true,
		"tracking": true,
		"confidence": confidence,
		"pairs": deltas.size(),
		"scatter_m": scatter,
		"residual_m": residual.length(),
		"diagnostic": "D455 wrist correction active",
	}
	var follower := _latest_follower_status()
	var feedback_settings = follower.get("feedback_settings", {})
	var contact_guard_enabled := (
		feedback_settings is Dictionary
		and bool((feedback_settings as Dictionary).get(
			"following_error_safety_enabled",
			true,
		))
	)
	if (
		str(follower.get("state", "")) == "armed"
		and contact_guard_enabled
		and confidence >= 0.65
		and residual.length() >= 0.014
	):
		_passive_d455_contact_frames += 1
	else:
		_passive_d455_contact_frames = 0
	if _passive_d455_contact_frames >= 4:
		_connect_arm_command_peer()
		_arm_command_udp.put_packet(JSON.stringify({
			"type": "arm_visual_contact_stop",
			"source": "d455_passive_wrist_tracker",
		}).to_utf8_buffer())
		_passive_d455_contact_frames = 0


func start_arm_position_calibration(
	force_editor_auto_move: bool = false,
	capture_mode: String = "base",
) -> void:
	if _state in ["capturing", "solving"]:
		return
	_capture_mode = capture_mode if capture_mode in ["base", "joints", "axis", "wrist", "claw"] else "base"
	var auto_move := force_editor_auto_move or (Engine.is_editor_hint() and editor_auto_move_follower)
	var overlay := get_node_or_null(OVERLAY_PATH)
	if overlay == null or not overlay.has_method("get_pose_capsules_local"):
		_fail("SO-101 overlay is unavailable.")
		return
	if _capture_mode in ["joints", "wrist", "claw"]:
		if not auto_move:
			_fail("Staged joint refinement requires the automatic one-servo-at-a-time sweep.")
			return
		if not diagnostic_only:
			if not overlay.has_method("get_registration_status"):
				_fail("Calibrate the robot base before refining individual servos.")
				return
			var registration: Dictionary = overlay.call("get_registration_status")
			if not bool(registration.get("registered", false)):
				_fail("Calibrate the robot base before refining individual servos.")
				return
			var registration_source := str(registration.get("registration_source", ""))
			var base_registration_source := str(
				registration.get("base_registration_source", "")
			)
			if (
				not registration_source.ends_with("motion_fit")
				and registration_source != "staged_attachment_safe_joint_refinement"
				and registration_source != "full_automated_motion_axis_calibration"
				and registration_source != "partial_automated_motion_axis_calibration"
				and base_registration_source != "settled_shoulder_pan_revolute_axis"
			):
				_fail(
					"Run Calibrate Robot Position successfully in this setup before refining servos; "
					+ "marker guesses and old checkpoints are not sufficient."
				)
				return
	if auto_move and overlay.has_method("get_latest_status"):
		var follower_status: Dictionary = overlay.call("get_latest_status")
		if not bool(follower_status.get("follower_connected", false)):
			_fail("Editor calibration needs the follower service on telemetry UDP 4252.")
			return
		if str(follower_status.get("state", "")) == "fault":
			_fail("Follower is faulted: %s. Use Reconnect Arm Hardware first." % str(follower_status.get("fault", "unknown fault")))
			return
	# Fresh containers are safe when this @tool script is hot-reloaded onto an
	# existing editor node. Mutating a newly-added stale member can crash Godot.
	_frames = []
	_baseline_capsules = PackedVector4Array()
	_baseline_joint_frames = []
	if overlay.has_method("get_best_available_normalized_pose"):
		var initial_pose = overlay.call("get_best_available_normalized_pose")
		if initial_pose is Array and (initial_pose as Array).size() >= 6:
			_baseline_capsules = overlay.call("get_pose_capsules_local", initial_pose)
			if overlay.has_method("get_pose_joint_frames_local"):
				_baseline_joint_frames = overlay.call("get_pose_joint_frames_local", initial_pose)
	_camera_previous = {}
	_camera_sequences = {}
	_preflight_sequences = {}
	_preflight_ready_samples = 0
	_next_preflight_msec = 0
	# A completed sweep can hand control directly to a bounded automatic retry.
	# Never let the prior request/timeout state leak into that fresh capture.
	_editor_sweep_due_msec = 0
	_editor_sweep_requested = false
	_editor_sweep_acknowledged = false
	_editor_sweep_requested_msec = 0
	_last_accepted_pose = []
	_capture_diagnostic = "Waiting to sample telemetry and depth."
	_capture_renderer_count = 0
	_capture_moving_points = 0
	_started_msec = Time.get_ticks_msec()
	_started_unix_ms = Time.get_unix_time_from_system() * 1000.0
	_next_sample_msec = _started_msec
	_state = "capturing"
	_auto_move_this_capture = auto_move
	if auto_move:
		var sweep_delay := editor_sweep_delay_seconds if editor_sweep_delay_seconds > 0.0 else 0.75
		_editor_sweep_due_msec = _started_msec + int(sweep_delay * 1000.0)
		_editor_status_until_msec = _started_msec + int((maximum_capture_seconds + 5.0) * 1000.0)
		_set_status(
			"capturing",
			(
				"Capturing a depth baseline before the one-servo-at-a-time refinement sweep."
				if _capture_mode == "joints"
				else "Preparing the five-state D455 claw fit."
				if _capture_mode == "claw"
				else (
					"Capturing settled clouds for the shoulder-pan base-axis sweep."
					if _capture_mode == "axis"
					else "Capturing a depth baseline before the bounded automatic base-registration sweep."
				)
			),
			0.0,
			0.0,
		)
	else:
		_set_status("capturing", "Move several arm joints slowly through distinct poses.", 0.0, 0.0)


func start_joint_alignment_calibration(force_editor_auto_move: bool = false) -> void:
	start_arm_position_calibration(force_editor_auto_move, "joints")


func start_base_axis_capture(force_editor_auto_move: bool = false) -> void:
	start_arm_position_calibration(force_editor_auto_move, "axis")


func start_claw_visual_calibration(force_editor_auto_move: bool = false) -> void:
	start_arm_position_calibration(force_editor_auto_move, "claw")


## Rechecks only the customized distal assembly. The saved base, shoulder,
## elbow, wrist-flex, wrist-roll, and validated gripper opening curve remain
## active unless the independently visible native-D455 RGB views approve a
## better claw frame across the five 45-degree wrist orientations.
func start_distal_claw_tip_calibration(force_editor_auto_move: bool = false) -> void:
	if _state in ["capturing", "solving"] or _automation_active:
		return
	var overlay := get_node_or_null(OVERLAY_PATH)
	if overlay == null or not overlay.has_method("get_registration_status"):
		_fail("SO-101 overlay registration is unavailable.")
		return
	var registration: Dictionary = overlay.call("get_registration_status")
	if (
		str(registration.get("registration_source", ""))
			!= "full_automated_motion_axis_calibration"
		or int(registration.get("calibrated_through_joint", -1)) < 4
	):
		_fail("Claw-only calibration requires a validated base-through-wrist registration.")
		return
	_automation_active = true
	_automation_stage = "claw_capture"
	_automation_force_editor_auto_move = force_editor_auto_move
	_automation_previous_diagnostic_only = diagnostic_only
	_automation_previous_debug_capture_path = debug_capture_path
	_automation_applied_through_joint = 4
	_automation_claw_result = {}
	_automation_claw_capture_attempt = 1
	_automation_claw_accumulated_frames = []
	diagnostic_only = true
	debug_capture_path = AUTOMATED_CLAW_CAPTURE_PATH
	_set_status(
		"capturing",
		"Preserving the validated arm chain; collecting five native-D455 RGB claw views...",
		0.94,
		0.85,
	)
	start_claw_visual_calibration(force_editor_auto_move)
	if _state == "failed":
		_end_automation(false)


func start_wrist_roll_calibration(force_editor_auto_move: bool = false) -> void:
	var previous_diagnostic_only := diagnostic_only
	var previous_debug_path := debug_capture_path
	diagnostic_only = false
	debug_capture_path = WRIST_CAPTURE_PATH
	start_arm_position_calibration(force_editor_auto_move, "wrist")
	if _state == "failed":
		diagnostic_only = previous_diagnostic_only
		debug_capture_path = previous_debug_path


func start_automated_arm_calibration(force_editor_auto_move: bool = false) -> void:
	if _state in ["capturing", "solving"] or _automation_active:
		return
	var overlay := get_node_or_null(OVERLAY_PATH)
	if overlay == null or not overlay.has_method("get_registration_status"):
		_fail("SO-101 overlay registration is unavailable.")
		return
	if not force_editor_auto_move and not Engine.is_editor_hint():
		_fail("Full automatic calibration requires the bounded follower sweep.")
		return
	var registration: Dictionary = overlay.call("get_registration_status")
	if _can_resume_recent_pending_claw(registration, overlay as Node3D):
		_resume_recent_pending_claw_calibration(force_editor_auto_move)
		return
	_automation_active = true
	_automation_stage = "base_capture"
	_automation_force_editor_auto_move = force_editor_auto_move
	_automation_previous_diagnostic_only = diagnostic_only
	_automation_previous_debug_capture_path = debug_capture_path
	_automation_base_result = {}
	_automation_base_capture_attempt = 1
	_automation_base_accumulated_frames = []
	_automation_joint_capture_attempt = 0
	_automation_joint_accumulated_frames = []
	_automation_solver_through_joint = 0
	_automation_applied_through_joint = -1
	_automation_latest_joint_result = {}
	_automation_claw_result = {}
	_automation_claw_capture_attempt = 0
	_automation_claw_accumulated_frames = []
	diagnostic_only = true
	debug_capture_path = AUTOMATED_BASE_CAPTURE_PATH
	start_base_axis_capture(force_editor_auto_move)
	if _state == "failed":
		_end_automation(false)


func _can_resume_recent_pending_claw(
	registration: Dictionary,
	overlay: Node3D,
) -> bool:
	if overlay == null:
		return false
	var refinement = registration.get("joint_refinement", null)
	var saved_unix_ms := float(registration.get("saved_unix_ms", 0.0))
	var age_msec := Time.get_unix_time_from_system() * 1000.0 - saved_unix_ms
	var transform := overlay.global_transform
	return (
		str(registration.get("registration_source", ""))
			== "full_automated_motion_axis_calibration"
		and int(registration.get("calibrated_through_joint", -1)) >= 4
		and refinement is Dictionary
		and bool((refinement as Dictionary).get("claw_calibration_pending", false))
		and saved_unix_ms > 0.0
		and age_msec >= 0.0
		and age_msec <= PENDING_CLAW_RESUME_MAXIMUM_AGE_MSEC
		and transform.is_finite()
		and transform.basis.determinant() > 0.0
		and transform.basis.y.normalized().dot(Vector3.UP) >= 0.25
	)


func _resume_recent_pending_claw_calibration(force_editor_auto_move: bool) -> void:
	_automation_active = true
	_automation_stage = "claw_capture"
	_automation_force_editor_auto_move = force_editor_auto_move
	_automation_previous_diagnostic_only = diagnostic_only
	_automation_previous_debug_capture_path = debug_capture_path
	_automation_applied_through_joint = 4
	_automation_claw_result = {}
	_automation_claw_capture_attempt = 1
	_automation_claw_accumulated_frames = []
	diagnostic_only = true
	debug_capture_path = AUTOMATED_CLAW_CAPTURE_PATH
	_set_status(
		"capturing",
		"Recent upright arm transaction is valid; resuming its pending five-state claw fit...",
		0.94,
		0.85,
	)
	start_claw_visual_calibration(force_editor_auto_move)
	if _state == "failed":
		_end_automation(false)


func _sweep_command_type() -> String:
	if _capture_mode == "claw":
		return "arm_claw_calibration_sweep"
	if _capture_mode == "wrist":
		return "arm_wrist_calibration_sweep"
	if _capture_mode == "joints":
		return "arm_joint_calibration_sweep"
	if _capture_mode == "axis":
		return "arm_base_axis_sweep"
	return "arm_calibration_sweep"


func cancel_arm_position_calibration() -> void:
	if _state == "capturing":
		_stop_editor_sweep("calibration cancelled")
		_fail("Arm position calibration cancelled.")
func _start_editor_sweep() -> void:
	if not _auto_move_this_capture or _state != "capturing":
		return
	_connect_arm_command_peer()
	_arm_command_udp.put_packet(JSON.stringify({
		"type": _sweep_command_type(),
		"action": "start",
		"control_session": "godot-editor-calibration",
		"view_strategy_attempt": (
			(
				_automation_claw_capture_attempt
				if _capture_mode == "claw"
				else _automation_joint_capture_attempt
			)
			if _automation_active and _capture_mode in ["joints", "wrist", "claw"]
			else _automation_base_capture_attempt
			if _automation_active and _capture_mode == "axis"
			else 1
		),
	}).to_utf8_buffer())
	_editor_sweep_requested = true
	_editor_sweep_requested_msec = Time.get_ticks_msec()
	_editor_sweep_acknowledged = false
	_capture_diagnostic = "Automatic follower sweep requested; waiting for visible arm motion."


func _update_editor_sweep_preflight() -> void:
	var now := Time.get_ticks_msec()
	if now < _next_preflight_msec:
		return
	_next_preflight_msec = now + maxi(80, int(sample_interval_seconds * 1000.0))
	var follower := _latest_follower_status()
	if not bool(follower.get("follower_connected", false)):
		_preflight_ready_samples = 0
		_capture_diagnostic = "Preflight blocked: follower telemetry is disconnected."
		return
	var sent_unix_ms := float(follower.get("sent_unix_ms", 0.0))
	if sent_unix_ms <= 0.0 or Time.get_unix_time_from_system() * 1000.0 - sent_unix_ms > 1000.0:
		_preflight_ready_samples = 0
		_capture_diagnostic = "Preflight blocked: follower telemetry is stale."
		return
	var renderers := _active_depth_renderers()
	if renderers.is_empty():
		_preflight_ready_samples = 0
		_capture_diagnostic = "Preflight blocked: no live RealSense depth image. The arm will not move."
		return
	var every_camera_advanced := true
	for renderer_variant in renderers:
		var renderer := renderer_variant as Node3D
		if renderer == null:
			every_camera_advanced = false
			continue
		var image := renderer.call("get_depth_image") as Image
		var intrinsics: Vector4 = renderer.call("get_current_intrinsics")
		if image == null or image.is_empty() or image.get_format() != Image.FORMAT_RF or intrinsics.x <= 0.0 or intrinsics.y <= 0.0:
			every_camera_advanced = false
			continue
		var key := str(renderer.get_instance_id())
		var sequence := int(renderer.call("get_frame_sequence")) if renderer.has_method("get_frame_sequence") else now
		if _preflight_sequences.has(key) and int(_preflight_sequences[key]) == sequence:
			every_camera_advanced = false
		_preflight_sequences[key] = sequence
		if not _camera_previous.has(key):
			_moving_points_for_renderer(renderer)
	if not every_camera_advanced:
		_preflight_ready_samples = 0
		_capture_diagnostic = "Preflight waiting for fresh depth frames from every active camera."
		return
	_preflight_ready_samples += 1
	_capture_diagnostic = "Preflight: live telemetry and depth %d/3." % _preflight_ready_samples
	if _preflight_ready_samples >= 3:
		_editor_sweep_due_msec = 0
		_start_editor_sweep()


func _stop_editor_sweep(reason: String) -> void:
	_editor_sweep_due_msec = 0
	_auto_move_this_capture = false
	if not _editor_sweep_requested:
		return
	_arm_command_udp.put_packet(JSON.stringify({
		"type": _sweep_command_type(),
		"action": "stop",
		"reason": reason,
		"control_session": "godot-editor-calibration",
	}).to_utf8_buffer())
	_editor_sweep_requested = false
	_editor_sweep_requested_msec = 0
	_editor_sweep_acknowledged = false


func _connect_arm_command_peer() -> void:
	_arm_command_udp.close()
	var selected_port := follower_command_port if follower_command_port > 0 else 4248
	_arm_command_udp.connect_to_host("127.0.0.1", selected_port)


func return_arm_to_rest_pose() -> bool:
	if not _arm_command_udp.is_socket_connected():
		_connect_arm_command_peer()
	var error := _arm_command_udp.put_packet(JSON.stringify({
		"type": "arm_return_to_rest",
		"control_session": "godot-editor-rest-return",
	}).to_utf8_buffer())
	return error == OK


func get_calibration_status() -> Dictionary:
	var status := _status.duplicate(true)
	if _state != "complete":
		return status
	var overlay := get_node_or_null(OVERLAY_PATH)
	if overlay == null or not overlay.has_method("get_registration_status"):
		return status
	var registration: Dictionary = overlay.call("get_registration_status")
	if not bool(registration.get("registered", false)):
		return status
	status["confidence"] = float(registration.get("confidence", status.get("confidence", 0.0)))
	status["saved_unix_ms"] = float(registration.get("saved_unix_ms", 0.0))
	return status


func get_last_solution() -> Dictionary:
	return _last_solution.duplicate(true)


func report_external_registration(message: String, confidence: float = 1.0) -> void:
	_state = "complete"
	_set_status("complete", message, 1.0, confidence)


func solve_frames_for_test(frames: Array, current_transform: Transform3D = Transform3D.IDENTITY) -> Dictionary:
	return _solve_frames(frames.duplicate(true), current_transform, false)


func score_transform_for_test(frames: Array, value: Transform3D) -> Dictionary:
	return _score_transform(value, frames, false)


func _update_capture() -> void:
	var now := Time.get_ticks_msec()
	var elapsed := float(now - _started_msec) / 1000.0
	var follower_status := _latest_follower_status()
	if _auto_move_this_capture and _editor_sweep_due_msec > 0 and elapsed > 6.0:
		_fail("Calibration preflight timed out without three fresh depth frames. The follower was not moved.")
		return
	if _editor_sweep_requested:
		if bool(follower_status.get("calibration_sweep_active", false)):
			_editor_sweep_acknowledged = true
		elif not _editor_sweep_acknowledged:
			var rejection := str(follower_status.get("calibration_rejection", ""))
			if not rejection.is_empty():
				_fail(
					"Follower rejected the calibration sweep without moving: %s "
					% rejection
					+ "No solve was attempted."
				)
				return
			elif now - _editor_sweep_requested_msec > 2500:
				_fail("Follower did not acknowledge the calibration sweep. No solve was attempted.")
				return
	var required_seconds := (
		maxf(capture_seconds, editor_sweep_minimum_seconds)
		if _auto_move_this_capture
		else capture_seconds
	)
	var progress := clampf(elapsed / required_seconds, 0.0, 1.0)
	var sweep_fraction := -1.0
	if _auto_move_this_capture:
		sweep_fraction = clampf(
			float(follower_status.get("calibration_sweep_progress", 0.0)),
			0.0,
			1.0,
		)
		progress = sweep_fraction
	if _automation_active:
		if _automation_stage == "base_capture":
			progress = (
				(
					float(maxi(0, _automation_base_capture_attempt - 1))
					+ progress
				)
				/ float(MAXIMUM_AUTOMATED_BASE_CAPTURE_ATTEMPTS)
			) * 0.30
		elif _automation_stage == "joint_capture":
			progress = 0.45 + (
				(
					float(maxi(0, _automation_joint_capture_attempt - 1))
					+ progress
				)
				/ float(MAXIMUM_AUTOMATED_JOINT_CAPTURE_ATTEMPTS)
			) * 0.30
		elif _automation_stage == "claw_capture":
			progress = 0.94 + (
				(
					float(maxi(0, _automation_claw_capture_attempt - 1))
					+ progress
				)
				/ float(MAXIMUM_AUTOMATED_CLAW_CAPTURE_ATTEMPTS)
			) * 0.05
	if now >= _next_sample_msec:
		_next_sample_msec = now + int(sample_interval_seconds * 1000.0)
		_capture_frame()
	var excitation := _pose_excitation()
	var axis_ready := (
		_capture_mode == "axis"
		and _frames.size() >= 5
		and float(excitation.get("pan_range", 0.0)) >= MINIMUM_BASE_PAN_COVERAGE_DEGREES
	)
	var ready := axis_ready or (
		_frames.size() >= minimum_pose_frames and bool(excitation.get("ready", false))
	)
	var wrist_coverage := {}
	if _capture_mode == "wrist":
		wrist_coverage = _wrist_roll_capture_coverage(_frames)
		ready = bool(wrist_coverage.get("ready", false))
	var claw_coverage := {}
	if _capture_mode == "claw":
		claw_coverage = _claw_capture_coverage(_frames)
		ready = bool(claw_coverage.get("ready", false))
	var message := "Move several arm joints slowly through distinct poses."
	if _auto_move_this_capture:
		var sweep_progress := maxf(sweep_fraction, 0.0) * 100.0
		message = "%s sweep %.0f%% | %d accepted poses | %s" % [
			(
				"Wrist-roll"
				if _capture_mode == "wrist"
				else "Five-state claw"
				if _capture_mode == "claw"
				else "One-servo"
				if _capture_mode == "joints"
				else "Base-axis" if _capture_mode == "axis" else "Base-registration"
			),
			sweep_progress,
			_frames.size(),
			_capture_diagnostic,
		]
	if _frames.is_empty():
		message = ("Automatic sweep: %s" % _capture_diagnostic) if _auto_move_this_capture else _capture_diagnostic
	elif _capture_mode == "wrist" and not ready:
		message = "Wrist-roll coverage incomplete: %s | %s" % [
			str(wrist_coverage.get("summary", "unknown")),
			_capture_diagnostic,
		]
	elif _capture_mode == "claw" and not ready:
		message = "Claw-state coverage incomplete: %s | %s" % [
			str(claw_coverage.get("summary", "unknown")),
			_capture_diagnostic,
		]
	elif not (axis_ready or bool(excitation.get("ready", false))):
		message = "Waiting for distinct joint motion. %s" % _capture_diagnostic
	_set_status("capturing", message, progress, 0.0)
	var automatic_sweep_complete := (
		not _auto_move_this_capture
		or (
			(
				_editor_sweep_requested
				or not _frames.is_empty()
			)
			and (
				_editor_sweep_acknowledged
				or not _frames.is_empty()
			)
			and not bool(follower_status.get("calibration_sweep_active", false))
			and elapsed >= required_seconds
		)
	)
	if (
		elapsed >= required_seconds
		and automatic_sweep_complete
		and _automation_active
		and _automation_stage == "base_capture"
		and _capture_mode == "axis"
	):
		_merge_automated_base_capture_frames(_frames)
		var base_coverage := _base_axis_capture_coverage(
			_automation_base_accumulated_frames
		)
		if bool(base_coverage.get("ready", false)):
			_frames = _automation_base_accumulated_frames.duplicate(true)
			_start_solver()
		elif (
			_automation_base_capture_attempt
			< MAXIMUM_AUTOMATED_BASE_CAPTURE_ATTEMPTS
		):
			_stop_editor_sweep("base-axis coverage retry")
			_state = "complete"
			_set_status(
				"capturing",
				(
					"Base-axis depth coverage incomplete (%s); automatically "
					+ "retrying sweep %d/%d..."
				) % [
					str(base_coverage.get("summary", "unknown")),
					_automation_base_capture_attempt + 1,
					MAXIMUM_AUTOMATED_BASE_CAPTURE_ATTEMPTS,
				],
				progress,
				0.0,
			)
			call_deferred("_continue_automated_base_capture")
		else:
			_fail_automation(
				"Base-axis coverage remained incomplete after %d sweeps (%s)."
				% [
					MAXIMUM_AUTOMATED_BASE_CAPTURE_ATTEMPTS,
					str(base_coverage.get("summary", "unknown")),
				]
			)
		return
	if (
		elapsed >= required_seconds
		and automatic_sweep_complete
		and _automation_active
		and _automation_stage == "claw_capture"
		and _capture_mode == "claw"
	):
		_merge_automated_claw_capture_frames(_frames)
		var current_claw_coverage := _claw_capture_coverage(_frames)
		var complete_view_count := _complete_automated_claw_view_count(
			_automation_claw_accumulated_frames
		)
		if _automation_claw_capture_attempt < MAXIMUM_AUTOMATED_CLAW_CAPTURE_ATTEMPTS:
			_stop_editor_sweep("claw coverage retry")
			_state = "complete"
			_set_status(
				"capturing",
				(
					"Claw RGB view %d/%d captured (%s); automatically "
					+ "rotating only the wrist for the next validation view..."
				) % [
					_automation_claw_capture_attempt,
					MAXIMUM_AUTOMATED_CLAW_CAPTURE_ATTEMPTS,
					str(current_claw_coverage.get("summary", "unknown")),
				],
				progress,
				0.85,
			)
			call_deferred("_continue_automated_claw_capture")
		elif complete_view_count >= 2:
			_frames = _automation_claw_accumulated_frames.duplicate(true)
			_start_solver()
		else:
			_fail_automation(
				"D455 claw coverage produced only %d/2 complete RGB views after %d sweeps (%s)."
				% [
					complete_view_count,
					MAXIMUM_AUTOMATED_CLAW_CAPTURE_ATTEMPTS,
					str(current_claw_coverage.get("summary", "unknown")),
				]
			)
		return
	if elapsed >= required_seconds and ready and automatic_sweep_complete:
		if (
			_automation_active
			and _automation_stage == "joint_capture"
			and _capture_mode == "joints"
		):
			_merge_automated_joint_capture_frames(_frames)
			var coverage := _joint_axis_capture_coverage(
				_automation_joint_accumulated_frames
			)
			if bool(coverage.get("ready", false)):
				_frames = _automation_joint_accumulated_frames.duplicate(true)
				_start_solver()
			elif (
				_automation_joint_capture_attempt
				< MAXIMUM_AUTOMATED_JOINT_CAPTURE_ATTEMPTS
			):
				_stop_editor_sweep("joint coverage retry")
				_state = "complete"
				_set_status(
					"capturing",
					(
						"Joint depth coverage incomplete (%s); automatically "
						+ "retrying sweep %d/%d..."
					) % [
						str(coverage.get("summary", "unknown")),
						_automation_joint_capture_attempt + 1,
						MAXIMUM_AUTOMATED_JOINT_CAPTURE_ATTEMPTS,
					],
					progress,
					0.0,
				)
				call_deferred("_continue_automated_joint_capture")
			else:
				_fail_automation(
					"Joint depth coverage remained incomplete after %d sweeps (%s)."
					% [
						MAXIMUM_AUTOMATED_JOINT_CAPTURE_ATTEMPTS,
						str(coverage.get("summary", "unknown")),
					]
				)
		else:
			_start_solver()
	elif elapsed >= maximum_capture_seconds:
		_fail(
			"Not enough distinct shoulder-pan poses were observed."
			if _capture_mode == "axis"
			else "Lowered D455 wrist-roll coverage was incomplete (%s)." % [
				str(wrist_coverage.get("summary", "unknown"))
			]
			if _capture_mode == "wrist"
			else "D455 five-state claw coverage was incomplete (%s)." % [
				str(claw_coverage.get("summary", "unknown"))
			]
			if _capture_mode == "claw"
			else "Not enough distinct robot motion was observed. Try again and move 3-4 joints slowly."
		)


func _capture_frame() -> void:
	var overlay := get_node_or_null(OVERLAY_PATH)
	if overlay == null:
		_capture_diagnostic = "Robot overlay is unavailable."
		return
	var telemetry: Dictionary = overlay.call("get_latest_status")
	if _auto_move_this_capture and not bool(telemetry.get("calibration_pose_settled", false)):
		_capture_diagnostic = "Automatic sweep is moving to the next measured pose."
		return
	# Prefer a fresh encoder-derived pose. During the automatic sweep the follower
	# pauses Feetech reads to avoid torqued-chain stalls, so use its applied pose
	# until direct encoder sampling resumes.
	var pose_variant = (
		overlay.call("get_best_available_normalized_pose")
		if overlay.has_method("get_best_available_normalized_pose")
		else telemetry.get("applied_normalized", telemetry.get("follower_normalized", null))
	)
	if _auto_move_this_capture:
		var measured = telemetry.get("follower_normalized", null)
		var applied = telemetry.get("applied_normalized", null)
		var read_age = telemetry.get("last_read_age_ms", null)
		if (
			measured is Array
			and applied is Array
			and (measured as Array).size() >= 6
			and (applied as Array).size() >= 6
			and read_age != null
			and float(read_age) <= 250.0
		):
			var maximum_tracking_error := 0.0
			var maximum_tracking_joint := -1
			for index in range(6 if _capture_mode == "claw" else 5):
				var tracking_error := absf(
					float((measured as Array)[index])
					- float((applied as Array)[index])
				)
				if tracking_error > maximum_tracking_error:
					maximum_tracking_error = tracking_error
					maximum_tracking_joint = index
			var tracking_limit := maximum_encoder_settle_error_degrees
			# This real shoulder-pan trails a positive viewpoint by 6--8
			# degrees. Wrist fitting uses the fresh measured pan angle, not the
			# request, so retain the strict roll gate while allowing that
			# harmless viewpoint lag.
			if _capture_mode == "wrist" and maximum_tracking_joint == 0:
				tracking_limit = maxf(tracking_limit, 12.0)
			# The gripper linkage commonly settles several normalized degrees
			# away from its request under load.  Its measured encoder value is
			# what the visual solver consumes, so applying a pitch-joint gate
			# here only discards valid D455 opening states.
			if _capture_mode == "claw" and maximum_tracking_joint == 5:
				tracking_limit = maxf(tracking_limit, 12.0)
			if maximum_tracking_error > tracking_limit:
				_capture_diagnostic = (
					"Waiting for physical servo %d encoder (error %.2f deg) "
					+ "to settle within %.2f degrees of the commanded pose."
				) % [
					maximum_tracking_joint,
					maximum_tracking_error,
					tracking_limit,
				]
				return
			pose_variant = measured
	if not pose_variant is Array or (pose_variant as Array).size() < 6:
		_capture_diagnostic = "Waiting for follower joint telemetry on UDP 4250/4252."
		return
	var sent_unix_ms := float(telemetry.get("sent_unix_ms", 0.0))
	if sent_unix_ms > 0.0 and Time.get_unix_time_from_system() * 1000.0 - sent_unix_ms > 1000.0:
		_capture_diagnostic = "Follower telemetry is stale."
		return
	var pose: Array = []
	for index in range(6):
		pose.append(float((pose_variant as Array)[index]))
	var distinct_pose_delta := INF
	# Generic arm excitation intentionally ignores the gripper, but the
	# five-state claw sweep moves servo 5 and locks servos 0--4. Excluding it
	# here labels every real jaw opening after the first one a duplicate.
	if not _last_accepted_pose.is_empty():
		distinct_pose_delta = _pose_delta_degrees(_last_accepted_pose, pose)
		if _capture_mode == "claw":
			distinct_pose_delta += absf(
				float(_last_accepted_pose[5]) - float(pose[5])
			)
	if not _last_accepted_pose.is_empty() and distinct_pose_delta < 2.0:
		_capture_diagnostic = "Depth is live; move the arm farther from its last accepted pose."
		return
	var capsules: PackedVector4Array = overlay.call("get_pose_capsules_local", pose)
	if capsules.size() < 12:
		_capture_diagnostic = "Robot kinematic model did not produce all arm links."
		return
	var frame_baseline_capsules := (
		_baseline_capsules
		if _baseline_capsules.size() == capsules.size()
		else capsules
	)
	var joint_frames: Array = (
		overlay.call("get_pose_joint_frames_local", pose)
		if overlay.has_method("get_pose_joint_frames_local")
		else []
	)
	var frame_baseline_joint_frames: Array = (
		_baseline_joint_frames
		if _baseline_joint_frames.size() == joint_frames.size()
		else joint_frames
	)
	var points_by_camera: Array = []
	var full_points_by_camera: Array = []
	var depth_snapshots: Array = []
	var rgb_snapshots: Array = []
	var renderers := _active_depth_renderers()
	_capture_renderer_count = renderers.size()
	if renderers.is_empty():
		_capture_diagnostic = "No live RealSense depth images are available."
		return
	for renderer in renderers:
		var camera_points := _limit_points(_moving_points_for_renderer(renderer), maximum_points_per_frame)
		points_by_camera.append({"name": _depth_renderer_name(renderer), "points": camera_points})
		if _capture_mode in ["axis", "joints", "wrist", "claw"]:
			full_points_by_camera.append({
				"name": _depth_renderer_name(renderer),
				"points": _limit_points(
					_full_points_for_renderer(renderer),
					base_axis_full_points_per_camera,
				),
			})
		var snapshot := _depth_snapshot_for_renderer(renderer)
		if not snapshot.is_empty():
			depth_snapshots.append(snapshot)
		# Wrist roll must be validated against the real jaw silhouette, not only
		# noisy depth or the custom camera bracket.  Retain native D455 RGB for
		# both distal calibration modes so the wrist solver can lock the rigid
		# fixed jaw before the articulated opening curve is considered.
		if _capture_mode in ["wrist", "claw"]:
			var rgb_snapshot := _rgb_snapshot_for_renderer(renderer, pose)
			if not rgb_snapshot.is_empty():
				rgb_snapshots.append(rgb_snapshot)
	var points := _balanced_camera_points(points_by_camera, maximum_points_per_frame)
	_capture_moving_points = points.size()
	# Depth differencing advanced to this pose, so advance the corresponding
	# model baseline too—even when this transition has too few usable points.
	_baseline_capsules = capsules
	_baseline_joint_frames = joint_frames
	var required_moving_points := (
		20 if _capture_mode in ["wrist", "claw"] else minimum_moving_points
	)
	if points.size() < required_moving_points:
		_capture_diagnostic = "Depth is live (%d camera%s), but only %d/%d moving points were found." % [
			_capture_renderer_count,
			"s" if _capture_renderer_count != 1 else "",
			points.size(),
			required_moving_points,
		]
		return
	var mesh_points := {}
	if overlay.has_method("get_pose_mesh_points_local"):
		mesh_points = overlay.call("get_pose_mesh_points_local", pose, 140)
	var calibration_joint_index := int(telemetry.get("calibration_joint_index", -1))
	var claw_metadata: Dictionary = {}
	if overlay.has_method("get_pose_claw_calibration_metadata_local"):
		var claw_metadata_variant = overlay.call(
			"get_pose_claw_calibration_metadata_local",
			pose,
		)
		if claw_metadata_variant is Dictionary:
			claw_metadata = (claw_metadata_variant as Dictionary).duplicate(true)
	var claw_tip_positions_local := PackedVector3Array()
	if overlay.has_method("get_pose_claw_tip_positions_local"):
		claw_tip_positions_local = overlay.call(
			"get_pose_claw_tip_positions_local",
			pose,
			20000,
		)
	elif overlay.has_method("get_claw_tip_positions_world"):
		var model_inverse: Transform3D = overlay.global_transform.affine_inverse()
		for tip in overlay.call("get_claw_tip_positions_world"):
			claw_tip_positions_local.append(model_inverse * tip)
	_frames.append({
		"pose": pose,
		"points": points,
		"camera_points": points_by_camera,
		"full_camera_points": full_points_by_camera,
		"capsules": capsules,
		"baseline_capsules": frame_baseline_capsules,
		"observed_centroid": _points_centroid(points),
		"observed_robust_center": _points_coordinate_median(points),
		"model_centroid": _capsules_centroid(capsules),
		"mesh_points": mesh_points,
		"model_transform": overlay.global_transform,
		"claw_metadata": claw_metadata,
		"depth_snapshots": depth_snapshots,
		"rgb_snapshots": rgb_snapshots,
		"claw_tip_positions_local": claw_tip_positions_local,
		"calibration_mode": str(telemetry.get("calibration_sweep_mode", _capture_mode)),
		"calibration_joint_index": calibration_joint_index,
		"joint_frames": joint_frames,
		"baseline_joint_frames": frame_baseline_joint_frames,
	})
	_last_accepted_pose = pose
	_capture_diagnostic = "Captured pose %d from %d moving depth points." % [_frames.size(), points.size()]


func _capture_frame_motion_quality(frame: Dictionary) -> int:
	var quality := 0
	for camera in frame.get("camera_points", []):
		if camera is Dictionary:
			quality += (camera as Dictionary).get(
				"points",
				PackedVector3Array(),
			).size()
	return quality


func _merge_automated_base_capture_frames(incoming: Array) -> void:
	for candidate_variant in incoming:
		if not candidate_variant is Dictionary:
			continue
		var candidate := candidate_variant as Dictionary
		var pose = candidate.get("pose", null)
		if (
			not pose is Array
			or (pose as Array).size() < 6
			or int(candidate.get("calibration_joint_index", -2)) != 0
		):
			continue
		var duplicate_index := -1
		for existing_index in range(
			_automation_base_accumulated_frames.size()
		):
			var existing = _automation_base_accumulated_frames[existing_index]
			if (
				existing is Dictionary
				and absf(
					float(existing["pose"][0])
					- float((pose as Array)[0])
				) < 3.0
			):
				duplicate_index = existing_index
				break
		if duplicate_index < 0:
			_automation_base_accumulated_frames.append(
				candidate.duplicate(true)
			)
		elif (
			_capture_frame_motion_quality(candidate)
			> _capture_frame_motion_quality(
				_automation_base_accumulated_frames[duplicate_index]
			)
		):
			_automation_base_accumulated_frames[duplicate_index] = (
				candidate.duplicate(true)
			)


func _base_axis_capture_coverage(frames: Array) -> Dictionary:
	var pan_values: Array[float] = []
	var reference_camera_frames := 0
	for frame in frames:
		if (
			frame is Dictionary
			and frame.get("pose", null) is Array
			and (frame["pose"] as Array).size() >= 1
		):
			pan_values.append(float(frame["pose"][0]))
			if _automated_has_required_d455(
				frame.get("full_camera_points", null)
			):
				reference_camera_frames += 1
	pan_values.sort()
	var distinct: Array[float] = []
	for value in pan_values:
		if distinct.is_empty() or value - distinct[-1] >= 2.0:
			distinct.append(value)
	var pan_range := (
		distinct[-1] - distinct[0]
		if distinct.size() >= 2
		else 0.0
	)
	return {
		"ready": (
			distinct.size() >= 5
			and pan_range >= MINIMUM_BASE_PAN_COVERAGE_DEGREES
			and reference_camera_frames >= 5
		),
		"summary": "%d/5 poses, %.1f/%.0f deg, %d/5 D455 frames" % [
			distinct.size(),
			pan_range,
			MINIMUM_BASE_PAN_COVERAGE_DEGREES,
			reference_camera_frames,
		],
		"distinct_poses": distinct.size(),
		"pan_range_degrees": pan_range,
		"reference_camera_frames": reference_camera_frames,
		"frames": frames.size(),
	}


func _merge_automated_joint_capture_frames(incoming: Array) -> void:
	for candidate_variant in incoming:
		if not candidate_variant is Dictionary:
			continue
		var candidate := candidate_variant as Dictionary
		var joint_index := int(
			candidate.get("calibration_joint_index", -2)
		)
		var pose = candidate.get("pose", null)
		if not pose is Array or (pose as Array).size() < 6:
			continue
		# Keep all stationary baselines: a later retry can contain the one view
		# where the fixed jaw is not occluded by the custom camera bracket.
		if joint_index == -1:
			_automation_joint_accumulated_frames.append(
				candidate.duplicate(true)
			)
			continue
		var duplicate_index := -1
		for existing_index in range(
			_automation_joint_accumulated_frames.size()
		):
			var existing = _automation_joint_accumulated_frames[existing_index]
			if (
				not existing is Dictionary
				or int(existing.get("calibration_joint_index", -3))
					!= joint_index
			):
				continue
			var existing_pose = existing.get("pose", null)
			if (
				existing_pose is Array
				and (existing_pose as Array).size() >= 6
				and absf(
					float((existing_pose as Array)[0])
					- float((pose as Array)[0])
				) < 3.0
				and absf(
					float((existing_pose as Array)[joint_index])
					- float((pose as Array)[joint_index])
				) < 1.5
			):
				duplicate_index = existing_index
				break
		if duplicate_index < 0:
			_automation_joint_accumulated_frames.append(
				candidate.duplicate(true)
			)
		elif (
			_capture_frame_motion_quality(candidate)
			> _capture_frame_motion_quality(
				_automation_joint_accumulated_frames[duplicate_index]
			)
		):
			_automation_joint_accumulated_frames[duplicate_index] = (
				candidate.duplicate(true)
			)


func _merge_automated_claw_capture_frames(incoming: Array) -> void:
	for candidate_variant in incoming:
		if not candidate_variant is Dictionary:
			continue
		var candidate := candidate_variant as Dictionary
		if int(candidate.get("calibration_joint_index", -1)) != 5:
			continue
		var pose = candidate.get("pose", null)
		if not pose is Array or (pose as Array).size() < 6:
			continue
		var duplicate_index := -1
		for existing_index in range(_automation_claw_accumulated_frames.size()):
			var existing = _automation_claw_accumulated_frames[existing_index]
			if (
				existing is Dictionary
				and existing.get("pose", null) is Array
				and (existing["pose"] as Array).size() >= 6
				and absf(
					float((existing["pose"] as Array)[5])
					- float((pose as Array)[5])
				) < 3.0
				and absf(
					float((existing["pose"] as Array)[4])
					- float((pose as Array)[4])
				) < 3.0
			):
				duplicate_index = existing_index
				break
		if duplicate_index < 0:
			_automation_claw_accumulated_frames.append(candidate.duplicate(true))
		elif (
			_capture_frame_motion_quality(candidate)
			> _capture_frame_motion_quality(
				_automation_claw_accumulated_frames[duplicate_index]
			)
		):
			_automation_claw_accumulated_frames[duplicate_index] = candidate.duplicate(true)


func _best_complete_automated_claw_view(frames: Array) -> Array:
	var groups: Array = []
	for frame_variant in frames:
		if not frame_variant is Dictionary:
			continue
		var frame := frame_variant as Dictionary
		var pose = frame.get("pose", null)
		if (
			int(frame.get("calibration_joint_index", -1)) != 5
			or not pose is Array
			or (pose as Array).size() < 6
		):
			continue
		var group_index := -1
		for index in range(groups.size()):
			var first: Dictionary = groups[index][0]
			if absf(float(first["pose"][4]) - float((pose as Array)[4])) < 3.0:
				group_index = index
				break
		if group_index < 0:
			groups.append([frame])
		else:
			groups[group_index].append(frame)
	var best: Array = []
	var best_quality := -1
	for group_variant in groups:
		var group := group_variant as Array
		if not bool(_claw_capture_coverage(group).get("ready", false)):
			continue
		var quality := 0
		for frame in group:
			quality += _capture_frame_motion_quality(frame)
		if quality > best_quality:
			best = group.duplicate(true)
			best_quality = quality
	return best


func _complete_automated_claw_view_count(frames: Array) -> int:
	var groups: Array = []
	for frame_variant in frames:
		if not frame_variant is Dictionary:
			continue
		var frame := frame_variant as Dictionary
		var pose = frame.get("pose", null)
		if (
			int(frame.get("calibration_joint_index", -1)) != 5
			or not pose is Array
			or (pose as Array).size() < 6
		):
			continue
		var group_index := -1
		for index in range(groups.size()):
			var first := groups[index][0] as Dictionary
			if absf(float(first["pose"][4]) - float((pose as Array)[4])) < 3.0:
				group_index = index
				break
		if group_index < 0:
			groups.append([frame])
		else:
			groups[group_index].append(frame)
	var complete := 0
	for group_variant in groups:
		var group := group_variant as Array
		if not bool(_claw_capture_coverage(group).get("ready", false)):
			continue
		var rgb_states := 0
		for frame_variant in group:
			if not frame_variant is Dictionary:
				continue
			for snapshot_variant in (frame_variant as Dictionary).get("rgb_snapshots", []):
				if (
					snapshot_variant is Dictionary
					and "d455" in str((snapshot_variant as Dictionary).get("name", "")).to_lower()
					and FileAccess.file_exists(str((snapshot_variant as Dictionary).get("path", "")))
				):
					rgb_states += 1
					break
		if rgb_states >= 5:
			complete += 1
	return complete


func _joint_axis_capture_coverage(frames: Array) -> Dictionary:
	var stage_summaries: Array[String] = []
	var all_ready := true
	for joint_index in [1, 2, 3, 4]:
		var staged: Array = []
		for frame in frames:
			if (
				frame is Dictionary
				and int(frame.get("calibration_joint_index", -1))
					== joint_index
			):
				staged.append(frame)
		staged.sort_custom(
			func(left: Dictionary, right: Dictionary) -> bool:
				return float(left["pose"][0]) < float(right["pose"][0])
		)
		var groups: Array = []
		for frame in staged:
			var pan := float(frame["pose"][0])
			if groups.is_empty():
				groups.append([frame])
				continue
			var latest: Array = groups[-1]
			var pan_sum := 0.0
			for grouped in latest:
				pan_sum += float(grouped["pose"][0])
			if absf(pan - pan_sum / float(latest.size())) > 3.0:
				groups.append([frame])
			else:
				latest.append(frame)
		var complete_groups := 0
		for group in groups:
			var values: Array[float] = []
			for frame in group:
				values.append(float(frame["pose"][joint_index]))
			values.sort()
			var distinct: Array[float] = []
			for value in values:
				if (
					distinct.is_empty()
					or value - distinct[-1] >= 1.0
				):
					distinct.append(value)
			# Wrist-roll zero is fitted from all five purpose-built angles in
			# both shoulder-pan viewpoints.  Treating a three-angle wrist group
			# as complete let capture finish successfully only for the external
			# solver to reject the same evidence several minutes later.
			var minimum_distinct := 5 if joint_index == 4 else 3
			if (
				distinct.size() >= minimum_distinct
				and distinct[-1] - distinct[0] >= 24.0
			):
				complete_groups += 1
		if complete_groups < 2:
			all_ready = false
		stage_summaries.append(
			"%s:%d/2" % [
				str(REFINABLE_JOINT_NAMES.get(
					joint_index,
					"wrist_roll" if joint_index == 4 else joint_index,
				)),
				complete_groups,
			]
		)
	var baseline_count := 0
	for frame in frames:
		if (
			frame is Dictionary
			and int(frame.get("calibration_joint_index", -2)) == -1
		):
			baseline_count += 1
	if baseline_count < 1:
		all_ready = false
	return {
		"ready": all_ready,
		"summary": ", ".join(stage_summaries),
		"stationary_baselines": baseline_count,
		"frames": frames.size(),
	}


func _wrist_roll_capture_coverage(frames: Array) -> Dictionary:
	var staged: Array = []
	for frame in frames:
		if (
			frame is Dictionary
			and (
				str(frame.get("calibration_mode", "")) == "wrist"
				or int(frame.get("calibration_joint_index", -1)) == 4
			)
			and frame.get("pose", null) is Array
			and (frame["pose"] as Array).size() >= 5
		):
			staged.append(frame)
	staged.sort_custom(
		func(left: Dictionary, right: Dictionary) -> bool:
			return float(left["pose"][0]) < float(right["pose"][0])
	)
	var groups: Array = []
	for frame in staged:
		var pan := float(frame["pose"][0])
		if groups.is_empty():
			groups.append([frame])
			continue
		var latest: Array = groups[-1]
		var reference_pan := float(latest[0]["pose"][0])
		if absf(pan - reference_pan) > 3.0:
			groups.append([frame])
		else:
			latest.append(frame)
	var complete_groups := 0
	var viewpoint_summaries: Array[String] = []
	for group in groups:
		var d455_rolls: Array[float] = []
		for frame in group:
			if _automated_has_required_d455(
				frame.get("full_camera_points", null)
			):
				d455_rolls.append(float(frame["pose"][4]))
		d455_rolls.sort()
		var distinct: Array[float] = []
		for roll in d455_rolls:
			if distinct.is_empty() or roll - distinct[-1] >= 5.0:
				distinct.append(roll)
		var roll_span := (
			distinct[-1] - distinct[0]
			if distinct.size() >= 2
			else 0.0
		)
		viewpoint_summaries.append("%d poses/%.0f°" % [
			distinct.size(),
			roll_span,
		])
		if (
			distinct.size() >= 5
			and roll_span >= 48.0
		):
			complete_groups += 1
	return {
		"ready": complete_groups >= 2,
		"summary": "%d/2 viewpoints (%s)" % [
			complete_groups,
			", ".join(viewpoint_summaries),
		],
		"complete_viewpoints": complete_groups,
		"viewpoint_count": groups.size(),
		"frames": staged.size(),
	}


func _claw_capture_coverage(frames: Array) -> Dictionary:
	var openings: Array[float] = []
	var d455_frames := 0
	for frame in frames:
		if (
			frame is Dictionary
			and int(frame.get("calibration_joint_index", -1)) == 5
			and frame.get("pose", null) is Array
			and (frame["pose"] as Array).size() >= 6
		):
			if _automated_has_required_d455(
				frame.get("full_camera_points", null)
			):
				openings.append(float(frame["pose"][5]))
				d455_frames += 1
	openings.sort()
	var distinct: Array[float] = []
	for opening in openings:
		if distinct.is_empty() or opening - distinct[-1] >= 5.0:
			distinct.append(opening)
	var span := (
		distinct[-1] - distinct[0]
		if distinct.size() >= 2
		else 0.0
	)
	return {
		"ready": distinct.size() >= 5 and span >= 70.0 and d455_frames >= 5,
		"summary": "%d/5 D455 states, %.1f/70 deg span" % [
			distinct.size(),
			span,
		],
		"distinct_states": distinct.size(),
		"span_degrees": span,
		"d455_frames": d455_frames,
	}


func _depth_snapshot_for_renderer(renderer: Node3D) -> Dictionary:
	var image := renderer.call("get_depth_image") as Image
	if image == null or image.is_empty() or image.get_format() != Image.FORMAT_RF:
		return {}
	var intrinsics: Vector4 = renderer.call("get_current_intrinsics")
	if intrinsics.x <= 0.0 or intrinsics.y <= 0.0:
		return {}
	var width := image.get_width()
	var height := image.get_height()
	var step := DEPTH_VALIDATION_SAMPLE_STEP
	var cols := int(ceil(float(width) / float(step)))
	var rows := int(ceil(float(height) / float(step)))
	var sampled := PackedFloat32Array()
	var valid := PackedByteArray()
	sampled.resize(cols * rows)
	valid.resize(cols * rows)
	var bytes := image.get_data()
	# Display crop is a presentation setting. Calibration must retain the whole
	# sensor image or a valid arm segment can disappear from one camera before
	# the independent cross-camera gates ever see it.
	var limits := _renderer_capture_limits(renderer, true)
	var crop: Vector4 = limits["crop"]
	var maximum_depth := float(limits["max_depth_m"])
	for row in range(rows):
		var y := mini(row * step + step / 2, height - 1)
		var normalized_y := float(y) / float(height)
		if normalized_y < crop.y or normalized_y >= crop.w:
			continue
		for col in range(cols):
			var x := mini(col * step + step / 2, width - 1)
			var normalized_x := float(x) / float(width)
			if normalized_x < crop.x or normalized_x >= crop.z:
				continue
			var depth := bytes.decode_float((y * width + x) * 4)
			var index := row * cols + col
			if is_finite(depth) and depth >= 0.15 and depth <= maximum_depth:
				sampled[index] = depth
				valid[index] = 1
	return {
		"name": _depth_renderer_name(renderer),
		"depths": sampled,
		"valid": valid,
		"cols": cols,
		"rows": rows,
		"sample_step": step,
		"width": width,
		"height": height,
		"intrinsics": intrinsics,
		"grid_stride": int(renderer.call("get_current_grid_stride")) if renderer.has_method("get_current_grid_stride") else 1,
		"camera_inverse": renderer.global_transform.affine_inverse(),
	}


func _rgb_snapshot_for_renderer(renderer: Node3D, pose: Array) -> Dictionary:
	var camera_name := _depth_renderer_name(renderer)
	if "d455" not in camera_name.to_lower():
		return {}
	var image: Image = null
	var raw_intrinsics := Vector4.ZERO
	var raw_extrinsics := PackedFloat32Array()
	var uses_raw_color := false
	if renderer.has_method("get_raw_color_image"):
		image = renderer.call("get_raw_color_image") as Image
		if image != null and not image.is_empty():
			uses_raw_color = true
			raw_intrinsics = renderer.call("get_raw_color_intrinsics") as Vector4
			raw_extrinsics = renderer.call("get_depth_to_raw_color_extrinsics") as PackedFloat32Array
	if image == null or image.is_empty():
		if renderer.has_method("get_color_image"):
			image = renderer.call("get_color_image") as Image
		elif renderer.has_method("get_color_texture"):
			var texture = renderer.call("get_color_texture")
			if texture is Texture2D:
				image = (texture as Texture2D).get_image()
	if image == null or image.is_empty():
		return {}
	var intrinsics: Vector4 = raw_intrinsics if uses_raw_color else renderer.call("get_current_intrinsics")
	if intrinsics.x <= 0.0 or intrinsics.y <= 0.0:
		return {}
	if uses_raw_color and raw_extrinsics.size() != 12:
		return {}
	var directory := ProjectSettings.globalize_path(AUTOMATED_CLAW_RGB_DIRECTORY)
	if DirAccess.make_dir_recursive_absolute(directory) != OK:
		return {}
	var opening := float(pose[5]) if pose.size() >= 6 else -1.0
	var roll := float(pose[4]) if pose.size() >= 5 else -999.0
	var filename := "claw_%d_view%02d_roll%+04d_open%03d.png" % [
		int(_started_unix_ms),
		maxi(1, _automation_claw_capture_attempt),
		int(round(roll)),
		int(round(opening)),
	]
	var path := directory.path_join(filename)
	if image.save_png(path) != OK:
		return {}
	return {
		"name": camera_name,
		"path": path,
		"width": image.get_width(),
		"height": image.get_height(),
		"intrinsics": intrinsics,
		"camera_inverse": renderer.global_transform.affine_inverse(),
		"uses_raw_color": uses_raw_color,
		"depth_to_raw_color_extrinsics": raw_extrinsics,
	}

func _latest_follower_status() -> Dictionary:
	var overlay := get_node_or_null(OVERLAY_PATH)
	if overlay != null and overlay.has_method("get_latest_status"):
		return overlay.call("get_latest_status")
	return {}


func _active_depth_renderers() -> Array:
	var found: Array = []
	var camera_root := get_node_or_null(CAMERA_ROOT_PATH)
	if camera_root != null:
		_collect_depth_renderers(camera_root, found)
	return found


func _load_aruco_base_plane() -> Dictionary:
	if not constrain_base_to_aruco_plane or not FileAccess.file_exists(REALSENSE_ARUCO_GROUND_TRUTH_PATH):
		return {}
	var file := FileAccess.open(REALSENSE_ARUCO_GROUND_TRUTH_PATH, FileAccess.READ)
	if file == null:
		return {}
	var parsed = JSON.parse_string(file.get_as_text())
	if not parsed is Dictionary:
		return {}
	var payload := parsed as Dictionary
	if str(payload.get("method", "")) != "native_aruco":
		return {}
	var details = payload.get("details", {})
	if not details is Dictionary:
		return {}
	var rotation = (details as Dictionary).get("reference_marker_to_depth_godot_R", null)
	var translation = (details as Dictionary).get("reference_marker_to_depth_godot_T", null)
	if not rotation is Array or (rotation as Array).size() != 3:
		return {}
	if not translation is Array or (translation as Array).size() != 3:
		return {}
	for row in rotation:
		if not row is Array or (row as Array).size() != 3:
			return {}
	var marker_local := Transform3D(
		Basis(
			Vector3(float(rotation[0][0]), float(rotation[1][0]), float(rotation[2][0])),
			Vector3(float(rotation[0][1]), float(rotation[1][1]), float(rotation[2][1])),
			Vector3(float(rotation[0][2]), float(rotation[1][2]), float(rotation[2][2])),
		).orthonormalized(),
		Vector3(float(translation[0]), float(translation[1]), float(translation[2])),
	)
	var reference_id := str(payload.get("reference_camera", ""))
	var reference_node := _find_camera_node_by_id(get_node_or_null(CAMERA_ROOT_PATH), reference_id)
	if reference_node == null:
		return {}
	var marker_global: Transform3D = (reference_node as Node3D).global_transform * marker_local
	# cv_to_godot converts both sides of the marker->camera transform. Its local
	# marker Z is therefore the negative of OpenCV's camera-facing plane normal.
	var normal := -marker_global.basis.z.normalized()
	return {
		"ok": true,
		"origin": marker_global.origin + normal * aruco_base_clearance_m,
		"normal": normal,
		"marker_id": int(payload.get("marker_id", 49)),
	}


func _find_camera_node_by_id(node: Node, camera_id: String) -> Node3D:
	if node == null:
		return null
	if node is Node3D:
		if _node_has_property(node, &"camera_id") and str(node.get("camera_id")) == camera_id:
			return node as Node3D
		if not camera_id.is_empty() and str(node.name).contains(camera_id.split(":")[-1]):
			return node as Node3D
	for child in node.get_children():
		var found := _find_camera_node_by_id(child, camera_id)
		if found != null:
			return found
	return null


func _collect_depth_renderers(node: Node, found: Array) -> void:
	if node.has_method("get_depth_image") and node.has_method("get_current_intrinsics") and node is Node3D:
		var image = node.call("get_depth_image")
		if image is Image and not (image as Image).is_empty():
			found.append(node)
	for child in node.get_children():
		_collect_depth_renderers(child, found)


func _depth_renderer_name(renderer: Node) -> String:
	var current := renderer
	while current != null:
		if current.name.begins_with("RealSense "):
			return current.name
		current = current.get_parent()
	return renderer.name


func _moving_points_for_renderer(renderer: Node3D) -> PackedVector3Array:
	var sequence := int(renderer.call("get_frame_sequence")) if renderer.has_method("get_frame_sequence") else Time.get_ticks_msec()
	var key := str(renderer.get_instance_id())
	if int(_camera_sequences.get(key, -1)) == sequence:
		return PackedVector3Array()
	_camera_sequences[key] = sequence
	var image := renderer.call("get_depth_image") as Image
	if image == null or image.is_empty() or image.get_format() != Image.FORMAT_RF:
		return PackedVector3Array()
	var width := image.get_width()
	var height := image.get_height()
	# Motion evidence is calibration data, not rendered workspace geometry.
	# Ignore the display crop while retaining the camera's physical far cutoff.
	var capture_limits := _renderer_capture_limits(renderer, true)
	var crop: Vector4 = capture_limits["crop"]
	var minimum_x := clampi(int(floor(crop.x * float(width))), 0, width - 1)
	var maximum_x := clampi(int(ceil(crop.z * float(width))), minimum_x + 1, width)
	var minimum_y := clampi(int(floor(crop.y * float(height))), 0, height - 1)
	var maximum_y := clampi(int(ceil(crop.w * float(height))), minimum_y + 1, height)
	var maximum_depth := float(capture_limits["max_depth_m"])
	var step := maxi(2, depth_sample_step)
	var x_start := minimum_x + step / 2
	var y_start := minimum_y + step / 2
	var cols := int(ceil(float(maximum_x - x_start) / float(step)))
	var rows := int(ceil(float(maximum_y - y_start) / float(step)))
	if cols <= 0 or rows <= 0:
		return PackedVector3Array()
	var bytes := image.get_data()
	var depths := PackedFloat32Array()
	depths.resize(cols * rows)
	var valid := PackedByteArray()
	valid.resize(cols * rows)
	for row in range(rows):
		var y := y_start + row * step
		for col in range(cols):
			var x := x_start + col * step
			var index := row * cols + col
			var depth := bytes.decode_float((y * width + x) * 4)
			if is_finite(depth) and depth >= 0.15 and depth <= maximum_depth:
				depths[index] = depth
				valid[index] = 1
	var baseline: Dictionary = _camera_previous.get(key, {})
	if int(baseline.get("cols", 0)) != cols or int(baseline.get("rows", 0)) != rows:
		_camera_previous[key] = {"depths": depths, "valid": valid, "cols": cols, "rows": rows}
		return PackedVector3Array()
	var baseline_depths: PackedFloat32Array = baseline["depths"]
	var baseline_valid: PackedByteArray = baseline["valid"]
	var changed := PackedByteArray()
	changed.resize(cols * rows)
	for index in range(depths.size()):
		if valid[index] == 0 or baseline_valid[index] == 0:
			continue
		var adaptive_threshold := motion_threshold_m * maxf(1.0, depths[index] * 0.65)
		# Keep only new geometry that moved toward the camera relative to the
		# Keep coherent motion in either direction. Limit away-motion so deep
		# background disocclusions do not become part of the robot silhouette.
		var depth_delta := baseline_depths[index] - depths[index]
		if depth_delta >= adaptive_threshold or (-depth_delta >= adaptive_threshold and -depth_delta <= 0.12):
			changed[index] = 1
	var intrinsics: Vector4 = renderer.call("get_current_intrinsics")
	if intrinsics.x <= 0.0 or intrinsics.y <= 0.0:
		return PackedVector3Array()
	var grid_stride := int(renderer.call("get_current_grid_stride")) if renderer.has_method("get_current_grid_stride") else 1
	var points := PackedVector3Array()
	for row in range(rows):
		var y := y_start + row * step
		for col in range(cols):
			var x := x_start + col * step
			var index := row * cols + col
			if valid[index] == 0 or not _motion_or_neighbor(changed, depths, valid, cols, rows, col, row):
				continue
			var depth := depths[index]
			var source_x := float(x * grid_stride)
			var source_y := float(y * grid_stride)
			var local_point := Vector3(
				(source_x - intrinsics.z) * depth / intrinsics.x,
				-(source_y - intrinsics.w) * depth / intrinsics.y,
				-depth
			)
			points.append(renderer.global_transform * local_point)
	return points


func _full_points_for_renderer(renderer: Node3D) -> PackedVector3Array:
	var image := renderer.call("get_depth_image") as Image
	if image == null or image.is_empty() or image.get_format() != Image.FORMAT_RF:
		return PackedVector3Array()
	var intrinsics: Vector4 = renderer.call("get_current_intrinsics")
	if intrinsics.x <= 0.0 or intrinsics.y <= 0.0:
		return PackedVector3Array()
	var width := image.get_width()
	var height := image.get_height()
	# "Full" means the complete sensor image even when the user has cropped the
	# displayed point cloud. Otherwise reference-camera wrist validation can silently
	# become one-camera validation.
	var capture_limits := _renderer_capture_limits(renderer, true)
	var crop: Vector4 = capture_limits["crop"]
	var minimum_x := clampi(int(floor(crop.x * float(width))), 0, width - 1)
	var maximum_x := clampi(int(ceil(crop.z * float(width))), minimum_x + 1, width)
	var minimum_y := clampi(int(floor(crop.y * float(height))), 0, height - 1)
	var maximum_y := clampi(int(ceil(crop.w * float(height))), minimum_y + 1, height)
	var maximum_depth := float(capture_limits["max_depth_m"])
	var step := maxi(3, depth_sample_step)
	var bytes := image.get_data()
	var grid_stride := (
		int(renderer.call("get_current_grid_stride"))
		if renderer.has_method("get_current_grid_stride")
		else 1
	)
	var points := PackedVector3Array()
	for y in range(minimum_y + step / 2, maximum_y, step):
		for x in range(minimum_x + step / 2, maximum_x, step):
			var depth := bytes.decode_float((y * width + x) * 4)
			if not is_finite(depth) or depth < 0.15 or depth > maximum_depth:
				continue
			var source_x := float(x * grid_stride)
			var source_y := float(y * grid_stride)
			var local_point := Vector3(
				(source_x - intrinsics.z) * depth / intrinsics.x,
				-(source_y - intrinsics.w) * depth / intrinsics.y,
				-depth
			)
			points.append(renderer.global_transform * local_point)
	return points


func _renderer_capture_limits(
	renderer: Node,
	ignore_workspace_crop: bool = false,
) -> Dictionary:
	var crop := Vector4(0.0, 0.0, 1.0, 1.0)
	var maximum_depth := 8.0
	var current := renderer
	while current != null:
		if _node_has_property(current, &"max_depth_enabled"):
			if bool(current.get("max_depth_enabled")):
				maximum_depth = clampf(float(current.get("max_depth_m")), 0.25, 8.0)
			if (
				not ignore_workspace_crop
				and bool(current.get("workspace_crop_enabled"))
			):
				crop = Vector4(
					clampf(float(current.get("crop_left_percent")) * 0.01, 0.0, 0.45),
					clampf(float(current.get("crop_top_percent")) * 0.01, 0.0, 0.45),
					1.0 - clampf(float(current.get("crop_right_percent")) * 0.01, 0.0, 0.45),
					1.0 - clampf(float(current.get("crop_bottom_percent")) * 0.01, 0.0, 0.45),
				)
			break
		current = current.get_parent()
	return {"crop": crop, "max_depth_m": maximum_depth}


func _node_has_property(node: Object, property_name: StringName) -> bool:
	for property in node.get_property_list():
		if StringName(property.get("name", "")) == property_name:
			return true
	return false


func _motion_or_neighbor(changed: PackedByteArray, depths: PackedFloat32Array, valid: PackedByteArray, cols: int, rows: int, col: int, row: int) -> bool:
	var index := row * cols + col
	if valid[index] == 0:
		return false
	var support := 0
	for y_offset in range(-1, 2):
		for x_offset in range(-1, 2):
			var nc: int = col + x_offset
			var nr: int = row + y_offset
			if nc < 0 or nc >= cols or nr < 0 or nr >= rows:
				continue
			var neighbor: int = nr * cols + nc
			if changed[neighbor] == 0 or valid[neighbor] == 0:
				continue
			if absf(depths[index] - depths[neighbor]) <= 0.10:
				support += 1
	# Three nearby changed samples preserve coherent moving surfaces while
	# rejecting isolated temporal-depth noise.
	return support >= 3


func _limit_points(points: PackedVector3Array, maximum: int) -> PackedVector3Array:
	if points.size() <= maximum:
		return points
	var limited := PackedVector3Array()
	limited.resize(maximum)
	var step := float(points.size()) / float(maximum)
	for index in range(maximum):
		limited[index] = points[mini(points.size() - 1, int(floor(float(index) * step)))]
	return limited


func _balanced_camera_points(cameras: Array, maximum: int) -> PackedVector3Array:
	var balanced := PackedVector3Array()
	if cameras.is_empty() or maximum <= 0:
		return balanced
	var offsets: Array[int] = []
	offsets.resize(cameras.size())
	var active := true
	while balanced.size() < maximum and active:
		active = false
		for camera_index in range(cameras.size()):
			var camera_points: PackedVector3Array = cameras[camera_index].get(
				"points",
				PackedVector3Array(),
			)
			var point_index := offsets[camera_index]
			if point_index >= camera_points.size():
				continue
			balanced.append(camera_points[point_index])
			offsets[camera_index] = point_index + 1
			active = true
			if balanced.size() >= maximum:
				break
	return balanced


func _pose_delta_degrees(a: Array, b: Array) -> float:
	var total := 0.0
	for index in range(5):
		total += absf(float(a[index]) - float(b[index]))
	return total


func _pose_excitation() -> Dictionary:
	if _frames.size() < 2:
		return {"ready": false, "joint_count": 0, "total_range": 0.0, "pan_range": 0.0}
	var minimums := [INF, INF, INF, INF, INF]
	var maximums := [-INF, -INF, -INF, -INF, -INF]
	for frame in _frames:
		var pose: Array = frame["pose"]
		for index in range(5):
			minimums[index] = minf(minimums[index], float(pose[index]))
			maximums[index] = maxf(maximums[index], float(pose[index]))
	var joint_count := 0
	var total_range := 0.0
	for index in range(5):
		var joint_range: float = maximums[index] - minimums[index]
		total_range += joint_range
		if joint_range >= 7.0:
			joint_count += 1
	return {
		"ready": joint_count >= 2 and total_range >= 35.0,
		"joint_count": joint_count,
		"total_range": total_range,
		"pan_range": maximums[0] - minimums[0],
	}


func _start_solver() -> void:
	_stop_editor_sweep("motion capture complete")
	var overlay := get_node_or_null(OVERLAY_PATH) as Node3D
	if overlay == null:
		_fail("SO-101 overlay disappeared before fitting.")
		return
	# External motion-axis diagnostics need every settled angle from every
	# shoulder-pan viewpoint. The legacy generic solver's 24-frame cap can
	# remove an endpoint and make an otherwise complete five-pose group
	# unsolvable.
	var preserve_all_axis_frames := (
		_capture_mode in ["wrist", "claw"]
		or (diagnostic_only and _capture_mode in ["axis", "joints"])
	)
	var solve_frames := (
		_frames.duplicate(true)
		if preserve_all_axis_frames
		else _select_solve_frames(_frames, MAX_SOLVE_FRAMES)
	)
	if not debug_capture_path.is_empty():
		_save_debug_capture(solve_frames)
	if _capture_mode in ["axis", "joints"] and diagnostic_only:
		_last_solution = {
			"ok": true,
			"calibration_mode": _capture_mode,
			"frames": solve_frames.size(),
			"message": "Settled motion-axis clouds captured; no registration was applied.",
		}
		if _automation_active:
			_start_automated_external_solver()
			return
		_state = "complete"
		_set_status(
			"complete",
			(
				"Settled one-servo-at-a-time clouds captured for external joint-axis fitting."
				if _capture_mode == "joints"
				else "Settled shoulder-pan clouds captured for external base-axis fitting."
			),
			1.0,
			1.0,
		)
		return
	if _capture_mode == "claw":
		if not _automation_active or _automation_stage != "claw_capture":
			_fail("Five-state claw fitting is available through Calibrate Full Arm.")
			return
		_start_automated_external_solver()
		return
	if _capture_mode == "wrist":
		_state = "solving"
		var wrist_result_path := ProjectSettings.globalize_path(WRIST_RESULT_PATH)
		var wrist_arguments := PackedStringArray([
			ProjectSettings.globalize_path(WRIST_CAPTURE_PATH),
			ProjectSettings.globalize_path(SAVED_REGISTRATION_PATH),
			"--wrist-only",
			"--output",
			wrist_result_path,
		])
		_set_status(
			"solving",
			"Fitting wrist-roll direction and zero from lowered D455 observations...",
			1.0,
			0.0,
		)
		var wrist_error := _solve_thread.start(
			_execute_external_json_solver.bind(
				ProjectSettings.globalize_path(STAGED_JOINT_SOLVER_PATH),
				wrist_arguments,
				wrist_result_path,
				"wrist_solve",
			)
		)
		if wrist_error != OK:
			_fail("Could not start the wrist-roll solver.")
		return
	_state = "solving"
	if _capture_mode == "joints":
		_set_status(
			"solving",
			"Base held fixed; fitting attachment-safe shoulder, elbow, and wrist-flex servo zeroes...",
			1.0,
			0.0,
		)
		var joint_error := _solve_thread.start(
			_solve_joint_offsets.bind(solve_frames, overlay.global_transform)
		)
		if joint_error != OK:
			_fail("Could not start the staged joint-refinement solver thread.")
		return
		return
	_set_status("solving", "Arm located in motion; fitting the fixed base transform...", 1.0, 0.0)
	# The current overlay pose is included as one ordinary hypothesis, but no
	# marker-derived limits or acceptance rules are applied to the solve.
	var base_plane := _load_aruco_base_plane()
	if bool(base_plane.get("ok", false)):
		_set_status("solving", "Arm located; fitting position and yaw on the saved ArUco base plane...", 1.0, 0.0)
	var error := _solve_thread.start(_solve_frames.bind(solve_frames, overlay.global_transform, false, base_plane))
	if error != OK:
		_fail("Could not start the arm-position solver thread.")


func _save_debug_capture(frames: Array) -> void:
	var serialized_frames: Array = []
	for frame in frames:
		var serialized_points: Array = []
		for point in frame["points"]:
			serialized_points.append([point.x, point.y, point.z])
		var serialized_capsules: Array = []
		for endpoint in frame["capsules"]:
			serialized_capsules.append([endpoint.x, endpoint.y, endpoint.z, endpoint.w])
		var serialized_baseline_capsules: Array = []
		for endpoint in frame.get("baseline_capsules", PackedVector4Array()):
			serialized_baseline_capsules.append([endpoint.x, endpoint.y, endpoint.z, endpoint.w])
		var serialized_cameras: Array = []
		for camera in frame.get("camera_points", []):
			var camera_points: Array = []
			for point in camera["points"]:
				camera_points.append([point.x, point.y, point.z])
			serialized_cameras.append({"name": camera["name"], "points": camera_points})
		var serialized_full_cameras: Array = []
		for camera in frame.get("full_camera_points", []):
			var camera_points: Array = []
			for point in camera["points"]:
				camera_points.append([point.x, point.y, point.z])
			serialized_full_cameras.append({"name": camera["name"], "points": camera_points})
		var serialized_joint_frames: Array = []
		for joint_frame in frame.get("joint_frames", []):
			serialized_joint_frames.append({
				"pivot": _vector3_to_array(joint_frame.get("pivot", Vector3.ZERO)),
				"axis": _vector3_to_array(joint_frame.get("axis", Vector3.UP)),
			})
		var serialized_baseline_joint_frames: Array = []
		for joint_frame in frame.get("baseline_joint_frames", []):
			serialized_baseline_joint_frames.append({
				"pivot": _vector3_to_array(joint_frame.get("pivot", Vector3.ZERO)),
				"axis": _vector3_to_array(joint_frame.get("axis", Vector3.UP)),
			})
		var serialized_mesh_points := {}
		for link_name in frame.get("mesh_points", {}):
			var link_points: Array = []
			for point in frame["mesh_points"][link_name]:
				link_points.append([point.x, point.y, point.z])
			serialized_mesh_points[str(link_name)] = link_points
		var serialized_claw_metadata := {}
		var claw_metadata = frame.get("claw_metadata", {})
		if claw_metadata is Dictionary:
			for key in claw_metadata:
				var value = claw_metadata[key]
				if value is Transform3D:
					serialized_claw_metadata[str(key)] = _transform_to_external_dictionary(value)
				elif value is Vector3:
					serialized_claw_metadata[str(key)] = _vector3_to_array(value)
				else:
					serialized_claw_metadata[str(key)] = value
		var serialized_rgb_snapshots: Array = []
		for snapshot_variant in frame.get("rgb_snapshots", []):
			if not snapshot_variant is Dictionary:
				continue
			var snapshot := snapshot_variant as Dictionary
			var snapshot_intrinsics: Vector4 = snapshot.get("intrinsics", Vector4.ZERO)
			serialized_rgb_snapshots.append({
				"name": str(snapshot.get("name", "")),
				"path": str(snapshot.get("path", "")),
				"width": int(snapshot.get("width", 0)),
				"height": int(snapshot.get("height", 0)),
				"intrinsics": [
					snapshot_intrinsics.x,
					snapshot_intrinsics.y,
					snapshot_intrinsics.z,
					snapshot_intrinsics.w,
				],
				"camera_inverse": _transform_to_external_dictionary(
					snapshot.get("camera_inverse", Transform3D.IDENTITY)
				),
				"uses_raw_color": bool(snapshot.get("uses_raw_color", false)),
				"depth_to_raw_color_extrinsics": Array(
					snapshot.get("depth_to_raw_color_extrinsics", PackedFloat32Array())
				),
			})
		var serialized_claw_tips: Array = []
		for tip in frame.get("claw_tip_positions_local", PackedVector3Array()):
			serialized_claw_tips.append(_vector3_to_array(tip))
		serialized_frames.append({
			"pose": frame["pose"],
			"points": serialized_points,
			"capsules": serialized_capsules,
			"baseline_capsules": serialized_baseline_capsules,
			"camera_points": serialized_cameras,
			"full_camera_points": serialized_full_cameras,
			"mesh_points": serialized_mesh_points,
			"model_transform": _transform_to_external_dictionary(
				frame.get("model_transform", Transform3D.IDENTITY)
			),
			"claw_metadata": serialized_claw_metadata,
			"rgb_snapshots": serialized_rgb_snapshots,
			"claw_tip_positions_local": serialized_claw_tips,
			"observed_centroid": _vector3_to_array(frame["observed_centroid"]),
			"model_centroid": _vector3_to_array(frame["model_centroid"]),
			"calibration_mode": str(frame.get("calibration_mode", _capture_mode)),
			"calibration_joint_index": int(frame.get("calibration_joint_index", -1)),
			"joint_frames": serialized_joint_frames,
			"baseline_joint_frames": serialized_baseline_joint_frames,
		})
	var file := FileAccess.open(debug_capture_path, FileAccess.WRITE)
	if file == null:
		push_warning("Could not save SO-101 motion capture: %s" % debug_capture_path)
		return
	file.store_string(JSON.stringify({
		"type": "so101_motion_capture",
		"calibration_mode": _capture_mode,
		"capture_started_unix_ms": _started_unix_ms,
		"reference_camera": _reference_camera_for_frames(frames),
		"frames": serialized_frames,
	}))
	print("SO101_MOTION_CAPTURE_SAVED ", ProjectSettings.globalize_path(debug_capture_path))


func _reference_camera_for_frames(frames: Array) -> String:
	var requested := calibration_primary_camera_match.strip_edges().to_lower()
	var coverage := {}
	for frame_variant in frames:
		if not frame_variant is Dictionary:
			continue
		for camera_variant in (frame_variant as Dictionary).get(
			"full_camera_points",
			[],
		):
			if not camera_variant is Dictionary:
				continue
			var camera: Dictionary = camera_variant
			var name := str(camera.get("name", ""))
			if name.is_empty():
				continue
			coverage[name] = int(coverage.get(name, 0)) + camera.get(
				"points",
				PackedVector3Array(),
			).size()
	if coverage.is_empty():
		return ""
	if not requested.is_empty():
		for name in coverage:
			if requested in str(name).to_lower():
				return str(name)
	var best_name := ""
	var best_coverage := -1
	for name in coverage:
		if int(coverage[name]) > best_coverage:
			best_name = str(name)
			best_coverage = int(coverage[name])
	return best_name


func _transform_to_external_dictionary(value: Transform3D) -> Dictionary:
	return {
		"basis_x": _vector3_to_array(value.basis.x),
		"basis_y": _vector3_to_array(value.basis.y),
		"basis_z": _vector3_to_array(value.basis.z),
		"origin": _vector3_to_array(value.origin),
	}


func _vector3_to_array(value: Vector3) -> Array:
	return [value.x, value.y, value.z]


func _select_solve_frames(frames: Array, maximum: int) -> Array:
	if frames.size() <= maximum:
		return frames.duplicate(true)
	var selected: Array = []
	var step := float(frames.size() - 1) / float(maximum - 1)
	for index in range(maximum):
		selected.append(frames[int(round(float(index) * step))])
	return selected


func _solve_frames(
	frames: Array,
	current_transform: Transform3D,
	trusted_existing: bool = false,
	base_plane: Dictionary = {},
) -> Dictionary:
	var camera_sets := _camera_frame_sets(frames)
	var fit_frames := frames
	var fit_camera := "pooled"
	var requested_camera := calibration_primary_camera_match.strip_edges().to_lower()
	if not requested_camera.is_empty():
		for camera_name in camera_sets:
			var isolated_frames: Array = camera_sets[camera_name]
			if (
				requested_camera in str(camera_name).to_lower()
				and isolated_frames.size() >= minimum_pose_frames
			):
				fit_frames = isolated_frames
				fit_camera = str(camera_name)
				break
	var pooled: Dictionary
	if bool(base_plane.get("ok", false)):
		pooled = _solve_frame_set(fit_frames, current_transform, trusted_existing, base_plane)
	else:
		var unconstrained := _solve_frame_set(fit_frames, current_transform, trusted_existing)
		var approximate: Transform3D = unconstrained.get("transform", current_transform)
		var depth_plane := _estimate_depth_base_plane(fit_frames, approximate)
		if bool(depth_plane.get("ok", false)):
			var constrained := _solve_frame_set(
				fit_frames,
				current_transform,
				trusted_existing,
				depth_plane,
			)
			# A generic scene plane can be a wall, box, or shelf rather than the
			# robot's support surface.  It may only constrain the motion fit when
			# it preserves the independently observed robot-motion solution.
			var unconstrained_motion_loss := float(unconstrained.get("motion_loss_m", INF))
			var constrained_motion_loss := float(constrained.get("motion_loss_m", INF))
			var unconstrained_motion_confidence := float(
				unconstrained.get("motion_confidence", 0.0)
			)
			var constrained_motion_confidence := float(
				constrained.get("motion_confidence", 0.0)
			)
			var plane_preserves_motion := (
				bool(constrained.get("motion_valid", false))
				and constrained_motion_loss <= unconstrained_motion_loss + 0.004
				and constrained_motion_confidence
					>= unconstrained_motion_confidence - 0.06
			)
			if plane_preserves_motion:
				pooled = constrained
				pooled["status"] = "Depth-table plane constrained | %s" % str(
					pooled.get("status", "Arm fit complete.")
				)
			else:
				pooled = unconstrained
				pooled["depth_plane_rejected"] = true
				pooled["status"] = "Unreliable scene-plane constraint ignored | %s" % str(
					pooled.get("status", "Arm fit complete.")
				)
		else:
			pooled = unconstrained
	pooled["camera"] = fit_camera
	pooled["camera_frames"] = fit_frames.size()
	pooled["camera_count"] = camera_sets.size()
	var candidate = pooled.get("transform", null)
	var camera_metrics := {}
	var secondary_eligible := 0
	var secondary_confirmations := 0
	if candidate is Transform3D:
		for camera_name in camera_sets:
			var isolated_frames: Array = camera_sets[camera_name]
			var metrics := _score_transform(candidate, isolated_frames, false)
			camera_metrics[camera_name] = metrics
			if str(camera_name) == fit_camera or isolated_frames.size() < 5:
				continue
			secondary_eligible += 1
			if (
				float(metrics.get("confidence", 0.0))
					>= MINIMUM_SECONDARY_CONFIRMATION_CONFIDENCE
				and float(metrics.get("loss", INF))
					<= MAXIMUM_SECONDARY_CONFIRMATION_LOSS_M
			):
				secondary_confirmations += 1
	var cross_camera_confirmed := (
		secondary_eligible == 0
		or secondary_confirmations > 0
	)
	pooled["camera_motion_metrics"] = camera_metrics
	pooled["secondary_camera_checks"] = secondary_eligible
	pooled["secondary_camera_confirmations"] = secondary_confirmations
	pooled["cross_camera_confirmed"] = cross_camera_confirmed
	if bool(pooled.get("ok", false)) and not cross_camera_confirmed:
		pooled["ok"] = false
		pooled["status"] = (
			"Rejected: %s found a candidate, but no second camera saw matching "
			+ "robot motion. Clear the arm's view and repeat."
		) % fit_camera
	return pooled


func _solve_joint_offsets(frames: Array, fixed_transform: Transform3D) -> Dictionary:
	var deltas: Array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	var metrics := {}
	var confidence := 1.0
	var adjustment_limit := minf(
		maximum_joint_offset_adjustment_degrees,
		JOINT_REFINEMENT_LIMIT_DEGREES,
	)
	for joint_index in [1, 2, 3]:
		var staged_frames: Array = []
		for frame in frames:
			if int(frame.get("calibration_joint_index", -1)) == joint_index:
				staged_frames.append(frame)
		if staged_frames.size() < 3:
			return {
				"ok": false,
				"calibration_mode": "joints",
				"confidence": 0.0,
				"status": "Servo %s had only %d usable settled transitions; need at least 3."
					% [str(REFINABLE_JOINT_NAMES[joint_index]), staged_frames.size()],
			}
		var before_loss := _score_joint_deltas(
			staged_frames,
			fixed_transform,
			deltas,
			joint_index,
		)
		var best_loss := before_loss
		for step_degrees in [4.0, 1.5, 0.5, 0.15]:
			for pass_index in range(4):
				var improved := false
				for direction in [-1.0, 1.0]:
					var candidate: Array = deltas.duplicate()
					candidate[joint_index] = clampf(
						float(candidate[joint_index]) + float(step_degrees) * direction,
						-adjustment_limit,
						adjustment_limit,
					)
					if is_equal_approx(
						float(candidate[joint_index]),
						float(deltas[joint_index]),
					):
						continue
					var candidate_loss := _score_joint_deltas(
						staged_frames,
						fixed_transform,
						candidate,
						joint_index,
					)
					candidate_loss += (
						absf(float(candidate[joint_index]))
						* joint_offset_regularization_m_per_degree
					)
					if candidate_loss + 0.00001 < best_loss:
						deltas = candidate
						best_loss = candidate_loss
						improved = true
				if not improved:
					break
		var after_loss := _score_joint_deltas(
			staged_frames,
			fixed_transform,
			deltas,
			joint_index,
		)
		var improvement := before_loss - after_loss
		# Gains below 0.15 mm are not repeatable at this range.
		if improvement < 0.00015:
			deltas[joint_index] = 0.0
			after_loss = before_loss
			improvement = 0.0
		var at_limit := absf(float(deltas[joint_index])) >= adjustment_limit - 0.05
		if at_limit:
			return {
				"ok": false,
				"calibration_mode": "joints",
				"confidence": 0.0,
				"status": (
					"Servo %s refinement reached the safe %.1f-degree limit; "
					+ "the base/model must be corrected instead of forcing a servo offset."
				) % [str(REFINABLE_JOINT_NAMES[joint_index]), adjustment_limit],
			}
		var joint_confidence := clampf(1.0 - after_loss / 0.055, 0.0, 1.0)
		joint_confidence *= clampf(float(staged_frames.size()) / 4.0, 0.0, 1.0)
		confidence = minf(confidence, joint_confidence)
		metrics[str(REFINABLE_JOINT_NAMES[joint_index])] = {
			"frames": staged_frames.size(),
			"offset_delta_degrees": float(deltas[joint_index]),
			"before_residual_mm": before_loss * 1000.0,
			"after_residual_mm": after_loss * 1000.0,
			"improvement_mm": improvement * 1000.0,
		}
	var maximum_after_loss := 0.0
	for joint_name in metrics:
		maximum_after_loss = maxf(
			maximum_after_loss,
			float(metrics[joint_name].get("after_residual_mm", INF)) / 1000.0,
		)
	return {
		"ok": maximum_after_loss <= 0.05 and confidence >= 0.10,
		"calibration_mode": "joints",
		"confidence": confidence,
		"offset_deltas_degrees": deltas,
		"joint_metrics": metrics,
		"loss_m": maximum_after_loss,
		"status": (
			"Attachment-safe servo refinement complete: shoulder %.2f°, elbow %.2f°, wrist flex %.2f°."
			% [float(deltas[1]), float(deltas[2]), float(deltas[3])]
			if maximum_after_loss <= 0.05
			else "Joint refinement residual %.1f mm is too high; no servo mapping was changed."
				% (maximum_after_loss * 1000.0)
		),
	}


func _score_joint_deltas(
	frames: Array,
	fixed_transform: Transform3D,
	deltas: Array,
	joint_index: int,
) -> float:
	var frame_losses: Array[float] = []
	for frame in frames:
		var adjusted := _adjust_capsules_for_joint_deltas(
			frame.get("capsules", PackedVector4Array()),
			frame.get("joint_frames", []),
			deltas,
		)
		var adjusted_baseline := _adjust_capsules_for_joint_deltas(
			frame.get("baseline_capsules", PackedVector4Array()),
			frame.get("baseline_joint_frames", []),
			deltas,
		)
		var moved := _attachment_safe_moved_capsules(
			adjusted,
			adjusted_baseline,
			joint_index,
		)
		if moved.size() < 2:
			continue
		var transformed := _transform_capsules(moved, fixed_transform)
		var points: PackedVector3Array = frame.get("points", PackedVector3Array())
		if points.size() < 24:
			continue
		var point_step := maxi(1, int(ceil(float(points.size()) / 420.0)))
		var residuals: Array[float] = []
		for point_index in range(0, points.size(), point_step):
			residuals.append(
				minf(_point_capsule_residual(points[point_index], transformed), 0.12)
			)
		if residuals.size() < 16:
			continue
		residuals.sort()
		var keep := maxi(16, int(floor(float(residuals.size()) * 0.35)))
		keep = mini(keep, residuals.size())
		var observed_sum := 0.0
		for index in range(keep):
			observed_sum += residuals[index]
		var coverage_sum := 0.0
		var coverage_count := 0
		for capsule_index in range(0, transformed.size(), 2):
			var a4 := transformed[capsule_index]
			var b4 := transformed[capsule_index + 1]
			var a := Vector3(a4.x, a4.y, a4.z)
			var b := Vector3(b4.x, b4.y, b4.z)
			var radius := maxf(a4.w, b4.w)
			for fraction in [0.25, 0.5, 0.75]:
				var center := a.lerp(b, float(fraction))
				var nearest := 0.12
				for point_index in range(0, points.size(), point_step):
					nearest = minf(
						nearest,
						absf(center.distance_to(points[point_index]) - radius),
					)
				coverage_sum += nearest
				coverage_count += 1
		var observed_loss := observed_sum / float(keep)
		var coverage_loss := coverage_sum / maxf(float(coverage_count), 1.0)
		frame_losses.append(observed_loss * 0.75 + coverage_loss * 0.25)
	if frame_losses.size() < 3:
		return INF
	frame_losses.sort()
	return frame_losses[frame_losses.size() / 2]


func _adjust_capsules_for_joint_deltas(
	source: PackedVector4Array,
	source_joint_frames: Array,
	deltas: Array,
) -> PackedVector4Array:
	if source_joint_frames.size() < 4 or source.size() < 10:
		return source
	var result := source.duplicate()
	var joint_frames: Array = source_joint_frames.duplicate(true)
	for joint_index in [1, 2, 3]:
		var angle_degrees := float(deltas[joint_index])
		if absf(angle_degrees) <= 0.00001:
			continue
		var pivot: Vector3 = joint_frames[joint_index].get("pivot", Vector3.ZERO)
		var axis: Vector3 = joint_frames[joint_index].get("axis", Vector3.UP)
		if axis.length_squared() < 0.5:
			continue
		var rotation := Basis(axis.normalized(), deg_to_rad(angle_degrees))
		for capsule_index: int in range(joint_index + 1, int(result.size() / 2)):
			for endpoint_offset: int in [0, 1]:
				var endpoint_index: int = capsule_index * 2 + endpoint_offset
				var endpoint := result[endpoint_index]
				var point := Vector3(endpoint.x, endpoint.y, endpoint.z)
				point = pivot + rotation * (point - pivot)
				result[endpoint_index] = Vector4(point.x, point.y, point.z, endpoint.w)
		for later_joint_index in range(joint_index + 1, joint_frames.size()):
			var later_pivot: Vector3 = joint_frames[later_joint_index].get("pivot", Vector3.ZERO)
			var later_axis: Vector3 = joint_frames[later_joint_index].get("axis", Vector3.UP)
			joint_frames[later_joint_index]["pivot"] = (
				pivot + rotation * (later_pivot - pivot)
			)
			joint_frames[later_joint_index]["axis"] = (rotation * later_axis).normalized()
	return result


func _attachment_safe_moved_capsules(
	capsules: PackedVector4Array,
	baseline: PackedVector4Array,
	joint_index: int,
) -> PackedVector4Array:
	if capsules.size() != baseline.size():
		return PackedVector4Array()
	var moved := PackedVector4Array()
	# Fit exactly one immediate stock segment. Downstream servo zeroes and the
	# customized gripper/camera assembly therefore cannot pull this joint.
	var first_capsule_index := joint_index + 1
	var last_capsule_index := mini(first_capsule_index + 1, int(capsules.size() / 2))
	for capsule_index in range(first_capsule_index, last_capsule_index):
		var endpoint_index := capsule_index * 2
		var a := Vector3(
			capsules[endpoint_index].x,
			capsules[endpoint_index].y,
			capsules[endpoint_index].z,
		)
		var b := Vector3(
			capsules[endpoint_index + 1].x,
			capsules[endpoint_index + 1].y,
			capsules[endpoint_index + 1].z,
		)
		var baseline_a := Vector3(
			baseline[endpoint_index].x,
			baseline[endpoint_index].y,
			baseline[endpoint_index].z,
		)
		var baseline_b := Vector3(
			baseline[endpoint_index + 1].x,
			baseline[endpoint_index + 1].y,
			baseline[endpoint_index + 1].z,
		)
		if maxf(a.distance_to(baseline_a), b.distance_to(baseline_b)) < 0.006:
			continue
		moved.append(capsules[endpoint_index])
		moved.append(capsules[endpoint_index + 1])
	return moved


func _estimate_depth_base_plane(frames: Array, approximate: Transform3D) -> Dictionary:
	var scene_points := PackedVector3Array()
	var plane_camera_name := ""
	var selected_frames := _select_solve_frames(frames, mini(3, frames.size()))
	for frame in selected_frames:
		for snapshot_variant in frame.get("depth_snapshots", []):
			var snapshot: Dictionary = snapshot_variant
			var snapshot_name := str(snapshot.get("name", ""))
			if plane_camera_name.is_empty():
				plane_camera_name = snapshot_name
			if snapshot_name != plane_camera_name:
				continue
			var depths: PackedFloat32Array = snapshot.get("depths", PackedFloat32Array())
			var valid: PackedByteArray = snapshot.get("valid", PackedByteArray())
			var cols := int(snapshot.get("cols", 0))
			var rows := int(snapshot.get("rows", 0))
			var sample_step := int(snapshot.get("sample_step", DEPTH_VALIDATION_SAMPLE_STEP))
			var grid_stride := int(snapshot.get("grid_stride", 1))
			var intrinsics: Vector4 = snapshot.get("intrinsics", Vector4.ZERO)
			var camera_to_global: Transform3D = (snapshot.get("camera_inverse", Transform3D.IDENTITY) as Transform3D).affine_inverse()
			if cols <= 0 or rows <= 0 or depths.size() != cols * rows or intrinsics.x <= 0.0:
				continue
			for row in range(0, rows, 3):
				for col in range(0, cols, 3):
					var index := row * cols + col
					if valid[index] == 0:
						continue
					var depth := float(depths[index])
					var source_x := float((col * sample_step + sample_step / 2) * grid_stride)
					var source_y := float((row * sample_step + sample_step / 2) * grid_stride)
					var local := Vector3(
						(source_x - intrinsics.z) * depth / intrinsics.x,
						-(source_y - intrinsics.w) * depth / intrinsics.y,
						-depth,
					)
					var point := camera_to_global * local
					if point.distance_to(approximate.origin) <= 0.75:
						scene_points.append(point)
	if scene_points.size() < 400:
		return {}
	if scene_points.size() > 9000:
		scene_points = _limit_points(scene_points, 9000)
	var motion_points := PackedVector3Array()
	for frame in frames:
		for point in frame.get("points", PackedVector3Array()):
			motion_points.append(point)
	if motion_points.size() < 100:
		return {}
	motion_points = _limit_points(motion_points, 1200)
	var rng := RandomNumberGenerator.new()
	rng.seed = 101
	var best_normal := Vector3.ZERO
	var best_offset := 0.0
	var best_score := -INF
	var approximate_up := approximate.basis.orthonormalized().y.normalized()
	for iteration in range(360):
		var a := scene_points[rng.randi_range(0, scene_points.size() - 1)]
		var b := scene_points[rng.randi_range(0, scene_points.size() - 1)]
		var c := scene_points[rng.randi_range(0, scene_points.size() - 1)]
		var cross := (b - a).cross(c - a)
		if cross.length_squared() < 0.0025:
			continue
		var normal := cross.normalized()
		if normal.dot(approximate_up) < 0.0:
			normal = -normal
		if normal.dot(approximate_up) < cos(deg_to_rad(65.0)):
			continue
		var offset := normal.dot(a)
		var base_gap := absf(normal.dot(approximate.origin) - offset)
		if base_gap > 0.18:
			continue
		var motion_distances: Array[float] = []
		for point in motion_points:
			motion_distances.append(normal.dot(point) - offset)
		motion_distances.sort()
		var q10 := motion_distances[int(float(motion_distances.size() - 1) * 0.10)]
		var median := motion_distances[motion_distances.size() / 2]
		var q90 := motion_distances[int(float(motion_distances.size() - 1) * 0.90)]
		if q10 < -0.045 or median < 0.035 or median > 0.32 or q90 > 0.55:
			continue
		var inliers := 0
		for point in scene_points:
			if absf(normal.dot(point) - offset) <= 0.007:
				inliers += 1
		var score := float(inliers) - base_gap * 1800.0 + normal.dot(approximate_up) * 120.0
		if score > best_score:
			best_score = score
			best_normal = normal
			best_offset = offset
	if best_normal == Vector3.ZERO:
		return {}
	var centroid := Vector3.ZERO
	var centroid_count := 0
	for point in scene_points:
		if absf(best_normal.dot(point) - best_offset) <= 0.008:
			centroid += point
			centroid_count += 1
	if centroid_count < 80:
		return {}
	centroid /= float(centroid_count)
	# Least-squares plane refinement: the smallest covariance eigenvector is the
	# table normal. Inverse iteration is compact and stable for this 3x3 case.
	var covariance := Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)
	for point in scene_points:
		if absf(best_normal.dot(point) - best_offset) > 0.008:
			continue
		var delta := point - centroid
		covariance.x += delta.x * delta
		covariance.y += delta.y * delta
		covariance.z += delta.z * delta
	covariance.x.x += 0.0000001
	covariance.y.y += 0.0000001
	covariance.z.z += 0.0000001
	var inverse_covariance := covariance.inverse()
	var refined_normal := best_normal
	for iteration in range(12):
		refined_normal = (inverse_covariance * refined_normal).normalized()
	if refined_normal.dot(approximate_up) < 0.0:
		refined_normal = -refined_normal
	best_normal = refined_normal
	var plane_origin := approximate.origin - best_normal * best_normal.dot(approximate.origin - centroid)
	return {
		"ok": true,
		"origin": plane_origin + best_normal * aruco_base_clearance_m,
		"normal": best_normal,
		"source": "depth_table_plane",
		"inliers": centroid_count,
	}


func _solve_frame_set(
	frames: Array,
	current_transform: Transform3D,
	trusted_existing: bool = false,
	base_plane: Dictionary = {},
) -> Dictionary:
	if frames.size() < 3:
		return {"ok": false, "status": "Too few motion frames."}
	var base_sweep_degrees := _pose_joint_range_degrees(frames, 0)
	var heading_observed := base_sweep_degrees >= minimum_base_sweep_for_heading_fit_degrees
	var heading_limit := TRUSTED_HEADING_LIMIT_DEGREES if trusted_existing else maximum_ambiguous_heading_change_degrees
	var translation_limit := TRUSTED_TRANSLATION_LIMIT_M if trusted_existing else INF
	var has_base_plane := bool(base_plane.get("ok", false))
	var plane_up: Vector3 = base_plane.get("normal", Vector3.UP)
	var plane_origin: Vector3 = base_plane.get("origin", Vector3.ZERO)
	var candidate_bases := (
		_coarse_orientation_bases_on_up(current_transform.basis, plane_up)
		if has_base_plane
		else _coarse_orientation_bases(current_transform.basis, heading_observed, heading_limit)
	)
	var trusted_up := current_transform.basis.orthonormalized().y.normalized()
	var hypotheses: Array = []
	for basis in candidate_bases:
		var estimated_origin := _estimate_origin(frames, basis)
		if has_base_plane:
			estimated_origin = _project_origin_to_anchor_plane(estimated_origin, plane_origin, plane_up)
		elif trusted_existing:
			estimated_origin = _limit_origin_on_plane(
				estimated_origin,
				current_transform.origin,
				trusted_up,
				translation_limit,
			)
		var candidate := Transform3D(basis.orthonormalized(), estimated_origin)
		var metrics := _score_transform(candidate, frames, true)
		var loss := float(metrics.get("loss", INF))
		hypotheses.append({"transform": candidate, "loss": loss})
	hypotheses.sort_custom(_candidate_loss_less)
	# A folded SO-101 has several nearly mirrored coarse silhouettes. Preserve a
	# wider orientation shortlist, then use the bidirectional fine score before
	# spending the expensive local refinement budget.
	if hypotheses.size() > 32:
		hypotheses.resize(32)
	for hypothesis in hypotheses:
		hypothesis["loss"] = float(_score_transform(hypothesis["transform"], frames, false).get("loss", INF))
	hypotheses.sort_custom(_candidate_loss_less)
	if hypotheses.size() > 6:
		hypotheses.resize(6)
	var refined_hypotheses: Array = []
	for hypothesis in hypotheses:
		var refined := _refine_transform(
			hypothesis["transform"],
			frames,
			heading_observed,
			(hypothesis["transform"] as Transform3D).basis if has_base_plane else current_transform.basis,
			180.0 if has_base_plane else heading_limit,
			plane_origin if has_base_plane else current_transform.origin,
			INF if has_base_plane else translation_limit,
			has_base_plane or trusted_existing,
		)
		var refined_loss := float(_score_transform(refined, frames, false).get("loss", INF))
		refined_hypotheses.append({"transform": refined, "loss": refined_loss, "motion_loss": refined_loss})
	# The exact loaded pose is an important control sample. Previous versions only
	# kept its orientation and re-estimated its origin, which could discard an
	# already-correct anchor before the decisive depth comparison.
	if not has_base_plane:
		var current_motion_loss := float(_score_transform(current_transform, frames, false).get("loss", INF))
		refined_hypotheses.append({
			"transform": current_transform,
			"loss": current_motion_loss,
			"motion_loss": current_motion_loss,
		})
	refined_hypotheses.sort_custom(_candidate_loss_less)
	if refined_hypotheses.is_empty():
		return {"ok": false, "status": "No finite robot-base hypothesis could be fitted."}
	var depth_hypotheses: Array = []
	# Motion identifies the arm. Full-scene depth may refine that winner, but may
	# not rerank it onto a table, wall, or other background surface.
	for hypothesis in [refined_hypotheses[0]]:
		var depth_metrics := _score_depth_transform(hypothesis["transform"], frames)
		if int(depth_metrics.get("comparisons", 0)) < 80:
			continue
		depth_hypotheses.append({
			"transform": hypothesis["transform"],
			"loss": float(depth_metrics.get("loss", INF)),
			"motion_loss": float(hypothesis.get("motion_loss", hypothesis.get("loss", INF))),
			"depth_metrics": depth_metrics,
		})
	depth_hypotheses.sort_custom(_candidate_loss_less)
	if not depth_hypotheses.is_empty():
		# Full-scene depth is only a sanity check. It is not semantic: a table,
		# wall, or attachment can look deceptively close to projected robot mesh
		# vertices. Never move the motion-identified robot to improve this score.
		refined_hypotheses = depth_hypotheses
	var best: Transform3D = refined_hypotheses[0]["transform"]
	var best_loss := float(refined_hypotheses[0]["loss"])
	var distinct_alternative_loss := INF
	for alternative in refined_hypotheses:
		var difference := _transform_difference(best, alternative["transform"])
		if difference.x >= 0.03 or difference.y >= 12.0:
			distinct_alternative_loss = float(alternative["loss"])
			break
	var ambiguity_margin := (
		1.0
		if distinct_alternative_loss == INF
		else (distinct_alternative_loss - best_loss) / maxf(best_loss, 0.001)
	)
	var final_metrics := _score_transform(best, frames, false)
	var depth_metrics: Dictionary = refined_hypotheses[0].get("depth_metrics", {})
	var anchor_metrics := _score_transform(current_transform, frames, false)
	var depth_valid := int(depth_metrics.get("comparisons", 0)) >= 80
	var loss := float(depth_metrics.get("loss", final_metrics.get("loss", INF)))
	var confidence := (
		float(depth_metrics.get("confidence", 0.0))
		if depth_valid
		else float(final_metrics.get("confidence", 0.0))
	)
	var anchor_loss := float(anchor_metrics.get("loss", INF))
	var motion_loss := float(final_metrics.get("loss", INF))
	var motion_confidence := float(final_metrics.get("confidence", 0.0))
	var motion_valid := (
		motion_confidence >= maxf(minimum_confidence, MINIMUM_SEMANTIC_MOTION_CONFIDENCE)
		and motion_loss <= MAXIMUM_SEMANTIC_MOTION_LOSS_M
	)
	var accepted := _candidate_fit_is_acceptable(
		motion_confidence,
		motion_loss,
		confidence,
		loss,
		depth_valid,
		ambiguity_margin,
	)
	var result_status := (
		"Arm motion fit is ambiguous; repeat the one-joint sweep with clearer depth."
		if ambiguity_margin < 0.05
		else (
			"Attachment-safe depth fit confidence %.0f%%, residual %.1f mm, wrist %.1f mm; D455 motion %.0f%% / %.1f mm (%d samples)."
			% [
				confidence * 100.0,
				loss * 1000.0,
				float(depth_metrics.get("tip_residual_loss", loss)) * 1000.0,
				motion_confidence * 100.0,
				motion_loss * 1000.0,
				int(depth_metrics.get("comparisons", 0)),
			]
			if depth_valid
			else "Arm fit confidence %.0f%%, residual %.1f mm." % [
				motion_confidence * 100.0,
				motion_loss * 1000.0,
			]
		)
	)
	if not motion_valid:
		result_status = (
			"Rejected false alignment: robot-motion fit was only %.0f%% / %.1f mm. "
			+ "Table/background depth cannot promote this result."
		) % [motion_confidence * 100.0, motion_loss * 1000.0]
	return {
		"ok": accepted,
		"transform": best,
		"confidence": confidence,
		"loss_m": loss,
		"motion_confidence": motion_confidence,
		"motion_loss_m": motion_loss,
		"motion_valid": motion_valid,
		"depth_loss_m": depth_metrics.get("loss", INF),
		"depth_comparisons": depth_metrics.get("comparisons", 0),
		"depth_coverage_ratio": depth_metrics.get("coverage_ratio", 0.0),
		"anchor_loss_m": anchor_loss,
		"loss_improvement_ratio": (anchor_loss - loss) / maxf(anchor_loss, 0.0001),
		"inlier_ratio": final_metrics.get("inlier_ratio", 0.0),
		"coverage_ratio": final_metrics.get("coverage_ratio", 0.0),
		"heading_observed": heading_observed,
		"base_sweep_degrees": base_sweep_degrees,
		"base_plane_constrained": has_base_plane,
		"base_plane_source": str(base_plane.get("source", "aruco_marker_plane")) if has_base_plane else "",
		"ambiguity_margin": ambiguity_margin,
		"status": result_status,
	}


func _candidate_fit_is_acceptable(
	motion_confidence: float,
	motion_loss: float,
	final_confidence: float,
	final_loss: float,
	depth_valid: bool,
	ambiguity_margin: float,
) -> bool:
	return (
		motion_confidence >= maxf(minimum_confidence, MINIMUM_SEMANTIC_MOTION_CONFIDENCE)
		and motion_loss <= MAXIMUM_SEMANTIC_MOTION_LOSS_M
		and (
			(final_loss <= 0.05)
			if depth_valid
			else (
				final_confidence >= minimum_confidence
				and final_loss <= 0.09
			)
		)
		and ambiguity_margin >= 0.05
	)


func candidate_fit_is_acceptable_for_test(
	motion_confidence: float,
	motion_loss: float,
	final_confidence: float,
	final_loss: float,
	depth_valid: bool,
	ambiguity_margin: float,
) -> bool:
	return _candidate_fit_is_acceptable(
		motion_confidence,
		motion_loss,
		final_confidence,
		final_loss,
		depth_valid,
		ambiguity_margin,
	)


func _score_depth_transform(candidate: Transform3D, frames: Array) -> Dictionary:
	var validation_frames := _select_solve_frames(frames, mini(DEPTH_VALIDATION_MAX_FRAMES, frames.size()))
	var robust_sum := 0.0
	var robust_count := 0
	var tip_robust_sum := 0.0
	var tip_robust_count := 0
	var projected_count := 0
	var sampled_count := 0
	var snapshot_count := 0
	for frame in validation_frames:
		var mesh_points: Dictionary = frame.get("mesh_points", {})
		if mesh_points.is_empty():
			continue
		for snapshot_variant in frame.get("depth_snapshots", []):
			var snapshot: Dictionary = snapshot_variant
			var depths: PackedFloat32Array = snapshot.get("depths", PackedFloat32Array())
			var valid: PackedByteArray = snapshot.get("valid", PackedByteArray())
			var cols := int(snapshot.get("cols", 0))
			var rows := int(snapshot.get("rows", 0))
			var sample_step := maxi(1, int(snapshot.get("sample_step", 1)))
			var grid_stride := maxi(1, int(snapshot.get("grid_stride", 1)))
			var intrinsics: Vector4 = snapshot.get("intrinsics", Vector4.ZERO)
			var camera_inverse: Transform3D = snapshot.get("camera_inverse", Transform3D.IDENTITY)
			if cols <= 0 or rows <= 0 or depths.size() != cols * rows or valid.size() != depths.size():
				continue
			var residuals: Array[float] = []
			var tip_residuals: Array[float] = []
			var snapshot_projected := 0
			for link_name in mesh_points:
				if str(link_name) not in ATTACHMENT_SAFE_BASE_LINKS:
					continue
				var points: PackedVector3Array = mesh_points[link_name]
				for point in points:
					var camera_point := camera_inverse * (candidate * point)
					var predicted_depth := -camera_point.z
					if predicted_depth <= 0.15:
						continue
					var render_x := (camera_point.x * intrinsics.x / predicted_depth + intrinsics.z) / float(grid_stride)
					var render_y := (-camera_point.y * intrinsics.y / predicted_depth + intrinsics.w) / float(grid_stride)
					var center_col := int(floor(render_x / float(sample_step)))
					var center_row := int(floor(render_y / float(sample_step)))
					if center_col < 0 or center_col >= cols or center_row < 0 or center_row >= rows:
						continue
					snapshot_projected += 1
					var best_residual := INF
					for row_offset in range(-1, 2):
						var row := center_row + row_offset
						if row < 0 or row >= rows:
							continue
						for col_offset in range(-1, 2):
							var col := center_col + col_offset
							if col < 0 or col >= cols:
								continue
							var index := row * cols + col
							if valid[index] == 0:
								continue
							best_residual = minf(best_residual, absf(float(depths[index]) - predicted_depth))
					if best_residual < INF:
						var clipped_residual := minf(best_residual, 0.15)
						residuals.append(clipped_residual)
						if str(link_name) == "wrist_link":
							tip_residuals.append(clipped_residual)
			projected_count += snapshot_projected
			sampled_count += residuals.size()
			if residuals.size() < 20:
				continue
			residuals.sort()
			var keep := maxi(20, int(floor(float(residuals.size()) * DEPTH_VALIDATION_KEEP_FRACTION)))
			keep = mini(keep, residuals.size())
			for index in range(keep):
				robust_sum += residuals[index]
				robust_count += 1
			if not tip_residuals.is_empty():
				tip_residuals.sort()
				var tip_keep := mini(tip_residuals.size(), maxi(8, int(floor(float(tip_residuals.size()) * 0.55))))
				for index in range(tip_keep):
					tip_robust_sum += tip_residuals[index]
					tip_robust_count += 1
			snapshot_count += 1
	if robust_count == 0 or snapshot_count == 0:
		return {"loss": INF, "comparisons": 0, "coverage_ratio": 0.0, "confidence": 0.0}
	var residual_loss := robust_sum / float(robust_count)
	var tip_residual_loss := tip_robust_sum / float(tip_robust_count) if tip_robust_count >= 16 else residual_loss
	var coverage_ratio := float(sampled_count) / maxf(float(projected_count), 1.0)
	# A candidate cannot win by projecting most of the robot outside valid depth.
	var coverage_penalty := maxf(0.0, 0.18 - coverage_ratio) * 0.08
	var active_tip_weight := claw_tip_fit_weight if tip_robust_count >= 16 else 0.0
	var loss := lerpf(residual_loss, tip_residual_loss, active_tip_weight) + coverage_penalty
	var confidence := clampf(1.0 - loss / 0.035, 0.0, 1.0)
	confidence *= clampf(float(robust_count) / 350.0, 0.0, 1.0)
	confidence *= clampf(coverage_ratio / 0.18, 0.0, 1.0)
	return {
		"loss": loss,
		"residual_loss": residual_loss,
		"tip_residual_loss": tip_residual_loss,
		"tip_comparisons": tip_robust_count,
		"comparisons": robust_count,
		"coverage_ratio": coverage_ratio,
		"confidence": confidence,
		"snapshots": snapshot_count,
	}


func _refine_depth_transform(
	initial: Transform3D,
	frames: Array,
	anchor: Transform3D,
	trusted_existing: bool,
	base_plane: Dictionary = {},
) -> Transform3D:
	var best := initial
	var has_base_plane := bool(base_plane.get("ok", false))
	var plane_up: Vector3 = base_plane.get("normal", Vector3.UP)
	var plane_origin: Vector3 = base_plane.get("origin", Vector3.ZERO)
	var best_loss := float(_score_depth_transform(best, frames).get("loss", INF))
	var translation_steps := [0.02, 0.008, 0.003, 0.0015]
	var rotation_steps_degrees := [5.0, 2.0, 0.8, 0.35]
	for level in range(translation_steps.size()):
		for pass_index in range(1):
			var improved := false
			var translation_axes: Array = (
				[best.basis.x.normalized(), best.basis.z.normalized()]
				if has_base_plane
				else [Vector3.RIGHT, Vector3.UP, Vector3.BACK]
			)
			for axis in translation_axes:
				for direction in [-1.0, 1.0]:
					var candidate := best
					candidate.origin += axis * float(translation_steps[level]) * direction
					if has_base_plane:
						candidate.origin = _project_origin_to_anchor_plane(candidate.origin, plane_origin, plane_up)
					if trusted_existing and candidate.origin.distance_to(anchor.origin) > TRUSTED_TRANSLATION_LIMIT_M:
						continue
					var loss := float(_score_depth_transform(candidate, frames).get("loss", INF))
					if loss < best_loss:
						best = candidate
						best_loss = loss
						improved = true
			var rotation_axes: Array = [plane_up] if has_base_plane else [Vector3.RIGHT, Vector3.UP, Vector3.BACK]
			for axis in rotation_axes:
				for direction in [-1.0, 1.0]:
					var candidate := Transform3D(
						(Basis(axis, deg_to_rad(float(rotation_steps_degrees[level])) * direction) * best.basis).orthonormalized(),
						best.origin,
					)
					if (
						trusted_existing
						and rad_to_deg(candidate.basis.get_rotation_quaternion().angle_to(anchor.basis.get_rotation_quaternion()))
						> TRUSTED_HEADING_LIMIT_DEGREES
					):
						continue
					var loss := float(_score_depth_transform(candidate, frames).get("loss", INF))
					if loss < best_loss:
						best = candidate
						best_loss = loss
						improved = true
			if not improved:
				break
	return best


func score_depth_transform_for_test(candidate: Transform3D, frames: Array) -> Dictionary:
	return _score_depth_transform(candidate, frames)


func _transform_difference(a: Transform3D, b: Transform3D) -> Vector2:
	return Vector2(
		a.origin.distance_to(b.origin),
		rad_to_deg(a.basis.get_rotation_quaternion().angle_to(b.basis.get_rotation_quaternion())),
	)


func _camera_frame_sets(frames: Array) -> Dictionary:
	var result := {}
	for frame in frames:
		for camera in frame.get("camera_points", []):
			var camera_name := str(camera.get("name", "unknown"))
			var points: PackedVector3Array = camera.get("points", PackedVector3Array())
			if points.size() < 24:
				continue
			var isolated: Dictionary = frame.duplicate(true)
			isolated["points"] = points
			isolated["camera_points"] = [camera]
			var isolated_snapshots: Array = []
			for snapshot in frame.get("depth_snapshots", []):
				if str(snapshot.get("name", "")) == camera_name:
					isolated_snapshots.append(snapshot)
			isolated["depth_snapshots"] = isolated_snapshots
			isolated["observed_centroid"] = _points_centroid(points)
			isolated["observed_robust_center"] = _points_coordinate_median(points)
			if not result.has(camera_name):
				result[camera_name] = []
			(result[camera_name] as Array).append(isolated)
	return result


func _result_quality_less(a: Dictionary, b: Dictionary) -> bool:
	var a_loss := float(a.get("loss_m", INF))
	var b_loss := float(b.get("loss_m", INF))
	if absf(a_loss - b_loss) > 0.001:
		return a_loss < b_loss
	return float(a.get("confidence", 0.0)) > float(b.get("confidence", 0.0))


func _candidate_loss_less(a: Dictionary, b: Dictionary) -> bool:
	return float(a.get("loss", INF)) < float(b.get("loss", INF))


func _coarse_orientation_bases_on_up(current_basis: Basis, requested_up: Vector3) -> Array[Basis]:
	var up := requested_up.normalized()
	var x_axis := current_basis.orthonormalized().x
	x_axis -= up * x_axis.dot(up)
	if x_axis.length_squared() < 0.001:
		x_axis = Vector3.RIGHT - up * Vector3.RIGHT.dot(up)
	if x_axis.length_squared() < 0.001:
		x_axis = Vector3.BACK - up * Vector3.BACK.dot(up)
	x_axis = x_axis.normalized()
	var seed := Basis(x_axis, up, x_axis.cross(up).normalized()).orthonormalized()
	var candidates: Array[Basis] = []
	for yaw_index in range(24):
		candidates.append((Basis(up, deg_to_rad(float(yaw_index) * 15.0)) * seed).orthonormalized())
	return candidates


func _coarse_orientation_bases(
	current_basis: Basis,
	allow_heading_fit: bool = false,
	heading_limit_degrees: float = -1.0,
) -> Array[Basis]:
	var candidates: Array[Basis] = [current_basis.orthonormalized()]
	if lock_base_up_to_current:
		if allow_heading_fit:
			var up_axis := current_basis.orthonormalized().y.normalized()
			var requested_limit := heading_limit_degrees if heading_limit_degrees > 0.0 else maximum_ambiguous_heading_change_degrees
			var heading_limit := int(floor(requested_limit))
			for yaw_degrees in range(-heading_limit, heading_limit + 1, 6):
				if yaw_degrees == 0:
					continue
				candidates.append((Basis(up_axis, deg_to_rad(float(yaw_degrees))) * current_basis).orthonormalized())
		return candidates
	# Most installations are approximately upright. Seed that common case more
	# densely so the local fitter starts within a few degrees instead of merely
	# within the nearest cube orientation.
	for yaw_index in range(24):
		var yaw := deg_to_rad(float(yaw_index) * 15.0)
		for pitch_degrees in [-20.0, 0.0, 20.0]:
			for roll_degrees in [-20.0, 0.0, 20.0]:
				candidates.append(
					Basis(Vector3.UP, yaw)
					* Basis(Vector3.RIGHT, deg_to_rad(pitch_degrees))
					* Basis(Vector3.BACK, deg_to_rad(roll_degrees))
				)
	var directions := [
		Vector3.RIGHT,
		Vector3.LEFT,
		Vector3.UP,
		Vector3.DOWN,
		Vector3.BACK,
		Vector3.FORWARD,
	]
	# The 24 proper axis-aligned rotations cover SO(3) closely enough that the
	# local 15-degree refinement below can converge from any installation pose.
	for x_axis in directions:
		for y_axis in directions:
			if absf(x_axis.dot(y_axis)) > 0.001:
				continue
			var z_axis: Vector3 = x_axis.cross(y_axis)
			candidates.append(Basis(x_axis, y_axis, z_axis))
	return candidates


func _refine_transform(
	initial: Transform3D,
	frames: Array,
	allow_heading_fit: bool = false,
	heading_anchor_basis: Basis = Basis.IDENTITY,
	heading_limit_degrees: float = -1.0,
	origin_anchor: Vector3 = Vector3.ZERO,
	translation_limit_m: float = INF,
	constrain_origin_to_anchor_plane: bool = false,
) -> Transform3D:
	var best := initial
	var heading_anchor := (
		heading_anchor_basis.orthonormalized()
		if allow_heading_fit
		else initial.basis.orthonormalized()
	)
	var best_loss := float(_score_transform(best, frames, true).get("loss", INF))
	var requested_heading_limit := heading_limit_degrees if heading_limit_degrees > 0.0 else maximum_ambiguous_heading_change_degrees
	var anchor_up := heading_anchor_basis.orthonormalized().y.normalized()
	if constrain_origin_to_anchor_plane:
		best.origin = _limit_origin_on_plane(best.origin, origin_anchor, anchor_up, translation_limit_m)
	var translation_axes: Array[Vector3] = []
	if constrain_origin_to_anchor_plane:
		translation_axes.assign([
			heading_anchor_basis.orthonormalized().x.normalized(),
			heading_anchor_basis.orthonormalized().z.normalized(),
		])
	else:
		translation_axes.assign([Vector3.RIGHT, Vector3.UP, Vector3.BACK])
	var translation_step := 0.16
	var rotation_step := deg_to_rad(20.0)
	for level in range(7):
		var coarse := level < 3
		best_loss = float(_score_transform(best, frames, coarse).get("loss", INF))
		for pass_index in range(2):
			var improved := false
			for axis in translation_axes:
				for direction in [-1.0, 1.0]:
					var candidate := best
					candidate.origin += axis * translation_step * direction
					if constrain_origin_to_anchor_plane:
						candidate.origin = _project_origin_to_anchor_plane(candidate.origin, origin_anchor, anchor_up)
					if _origin_offset_distance(candidate.origin, origin_anchor, anchor_up, constrain_origin_to_anchor_plane) > translation_limit_m:
						continue
					var loss := float(_score_transform(candidate, frames, coarse).get("loss", INF))
					if loss < best_loss:
						best = candidate
						best_loss = loss
						improved = true
			var lock_up_axis := lock_base_up_to_current or constrain_origin_to_anchor_plane
			var rotation_axes: Array = [best.basis.y.normalized()] if lock_up_axis and allow_heading_fit else ([] if lock_up_axis else [Vector3.RIGHT, Vector3.UP, Vector3.BACK])
			for axis in rotation_axes:
				for direction in [-1.0, 1.0]:
					var new_basis := (Basis(axis, rotation_step * direction) * best.basis).orthonormalized()
					if (
						lock_up_axis
						and allow_heading_fit
						and rad_to_deg(heading_anchor.get_rotation_quaternion().angle_to(new_basis.get_rotation_quaternion()))
						> requested_heading_limit
					):
						continue
					var centroid_origin := _estimate_origin(frames, best.basis)
					var offset := best.origin - centroid_origin
					var new_origin := _estimate_origin(frames, new_basis) + offset
					if constrain_origin_to_anchor_plane:
						new_origin = _project_origin_to_anchor_plane(new_origin, origin_anchor, anchor_up)
					elif translation_limit_m < INF:
						new_origin = (
							_limit_origin_on_plane(new_origin, origin_anchor, anchor_up, translation_limit_m)
							if constrain_origin_to_anchor_plane
							else _limit_origin(new_origin, origin_anchor, translation_limit_m)
						)
					var candidate := Transform3D(new_basis, new_origin)
					var loss := float(_score_transform(candidate, frames, coarse).get("loss", INF))
					if loss < best_loss:
						best = candidate
						best_loss = loss
						improved = true
			if not improved:
				break
		translation_step *= 0.5
		rotation_step *= 0.5
	return best


func _limit_origin(value: Vector3, anchor: Vector3, maximum_distance: float) -> Vector3:
	var offset := value - anchor
	if offset.length() <= maximum_distance:
		return value
	return anchor + offset.normalized() * maximum_distance


func _project_origin_to_anchor_plane(value: Vector3, anchor: Vector3, up_axis: Vector3) -> Vector3:
	var normalized_up := up_axis.normalized()
	var offset := value - anchor
	return anchor + offset - normalized_up * offset.dot(normalized_up)


func _limit_origin_on_plane(
	value: Vector3,
	anchor: Vector3,
	up_axis: Vector3,
	maximum_distance: float,
) -> Vector3:
	var projected := _project_origin_to_anchor_plane(value, anchor, up_axis)
	return _limit_origin(projected, anchor, maximum_distance)


func _origin_offset_distance(
	value: Vector3,
	anchor: Vector3,
	up_axis: Vector3,
	planar: bool,
) -> float:
	if not planar:
		return value.distance_to(anchor)
	return _project_origin_to_anchor_plane(value, anchor, up_axis).distance_to(anchor)


func _pose_joint_range_degrees(frames: Array, joint_index: int) -> float:
	var minimum := INF
	var maximum := -INF
	for frame in frames:
		var pose: Array = frame.get("pose", [])
		if joint_index < 0 or joint_index >= pose.size():
			continue
		var value := float(pose[joint_index])
		minimum = minf(minimum, value)
		maximum = maxf(maximum, value)
	return maximum - minimum if minimum != INF else 0.0


func _estimate_origin(frames: Array, basis: Basis) -> Vector3:
	var xs: Array[float] = []
	var ys: Array[float] = []
	var zs: Array[float] = []
	for frame in frames:
		var observed_center: Vector3 = frame.get("observed_robust_center", _points_coordinate_median(frame["points"]))
		var baseline_capsules: PackedVector4Array = frame.get(
			"baseline_capsules",
			frames[0].get("capsules", PackedVector4Array()),
		)
		var moved_capsules := _capsules_moved_from_baseline(
			frame.get("capsules", PackedVector4Array()),
			baseline_capsules,
			0.008,
		)
		if moved_capsules.size() < 2:
			continue
		var model_center := _capsules_centroid(moved_capsules)
		var difference: Vector3 = observed_center - basis * model_center
		xs.append(difference.x)
		ys.append(difference.y)
		zs.append(difference.z)
	if xs.is_empty():
		return Vector3.ZERO
	return Vector3(_median(xs), _median(ys), _median(zs))


func _score_transform(candidate: Transform3D, frames: Array, coarse: bool) -> Dictionary:
	var robust_sum := 0.0
	var robust_count := 0
	var all_inliers := 0
	var all_points := 0
	var coverage_hits := 0
	var coverage_count := 0
	var coverage_sum := 0.0
	var point_limit := 120 if coarse else 320
	var scored_frames := 0
	for frame in frames:
		var points: PackedVector3Array = frame["points"]
		var baseline_capsules: PackedVector4Array = frame.get(
			"baseline_capsules",
			frames[0].get("capsules", PackedVector4Array()),
		)
		var capsules := _capsules_moved_from_baseline(
			frame["capsules"],
			baseline_capsules,
			0.008,
		)
		# Motion masks contain newly exposed geometry, not a scan of the complete
		# robot. Comparing those points against every link makes mirrored poses
		# deceptively cheap. Frames with no meaningfully moved link carry no pose
		# information and should not participate in the fit.
		if capsules.size() < 2:
			continue
		scored_frames += 1
		var transformed := _transform_capsules(capsules, candidate)
		var point_step := maxi(1, int(ceil(float(points.size()) / float(point_limit))))
		var residuals: Array[float] = []
		for point_index in range(0, points.size(), point_step):
			var residual := _point_capsule_residual(points[point_index], transformed)
			residuals.append(residual)
			all_points += 1
			if residual <= inlier_distance_m:
				all_inliers += 1
		residuals.sort()
		# Baseline subtraction now rejects room noise, so require the candidate to
		# explain most of every frame. A small trim still tolerates depth outliers.
		var keep := mini(residuals.size(), maxi(12, int(ceil(float(residuals.size()) * 0.82))))
		for index in range(keep):
			robust_sum += minf(residuals[index], 0.15)
			robust_count += 1
		if not coarse:
			for capsule_index in range(0, transformed.size(), 2):
				var a4 := transformed[capsule_index]
				var b4 := transformed[capsule_index + 1]
				var a := Vector3(a4.x, a4.y, a4.z)
				var b := Vector3(b4.x, b4.y, b4.z)
				var radius := maxf(a4.w, b4.w)
				for fraction in [0.2, 0.5, 0.8]:
					var center := a.lerp(b, float(fraction))
					var nearest := 0.15
					for point_index in range(0, points.size(), point_step):
						nearest = minf(nearest, absf(center.distance_to(points[point_index]) - radius))
					coverage_sum += nearest
					coverage_count += 1
					if nearest <= inlier_distance_m:
						coverage_hits += 1
	if scored_frames < 3 or robust_count == 0:
		return {"loss": INF, "inlier_ratio": 0.0, "coverage_ratio": 0.0, "confidence": 0.0}
	var observed_loss := robust_sum / float(robust_count)
	var coverage_loss := coverage_sum / maxf(float(coverage_count), 1.0) if not coarse else observed_loss
	# Motion masks contain only the portions of the arm newly visible against
	# the initial background. Their centroid is not the full robot centroid, so
	# centroid-trajectory penalties reject otherwise millimeter-accurate fits.
	var loss := observed_loss * 0.65 + coverage_loss * 0.35
	var inlier_ratio := float(all_inliers) / maxf(float(all_points), 1.0)
	var coverage_ratio := float(coverage_hits) / maxf(float(coverage_count), 1.0) if not coarse else 0.0
	var confidence := 0.0
	if not coarse:
		confidence = 0.62 * coverage_ratio + 0.38 * clampf(inlier_ratio / 0.38, 0.0, 1.0)
		confidence *= clampf(1.0 - maxf(0.0, loss - 0.025) / 0.09, 0.0, 1.0)
	return {
		"loss": loss,
		"inlier_ratio": inlier_ratio,
		"coverage_ratio": coverage_ratio,
		"confidence": confidence,
	}


func _capsules_moved_from_baseline(
	capsules: PackedVector4Array,
	baseline: PackedVector4Array,
	minimum_displacement_m: float,
) -> PackedVector4Array:
	if baseline.size() != capsules.size():
		return capsules
	var moved := PackedVector4Array()
	for index in range(0, capsules.size(), 2):
		var a := Vector3(capsules[index].x, capsules[index].y, capsules[index].z)
		var b := Vector3(capsules[index + 1].x, capsules[index + 1].y, capsules[index + 1].z)
		var baseline_a := Vector3(baseline[index].x, baseline[index].y, baseline[index].z)
		var baseline_b := Vector3(baseline[index + 1].x, baseline[index + 1].y, baseline[index + 1].z)
		if maxf(a.distance_to(baseline_a), b.distance_to(baseline_b)) < minimum_displacement_m:
			continue
		moved.append(capsules[index])
		moved.append(capsules[index + 1])
	return moved


func _transform_capsules(capsules: PackedVector4Array, value: Transform3D) -> PackedVector4Array:
	var transformed := PackedVector4Array()
	transformed.resize(capsules.size())
	for index in range(capsules.size()):
		var endpoint := capsules[index]
		var point := value * Vector3(endpoint.x, endpoint.y, endpoint.z)
		transformed[index] = Vector4(point.x, point.y, point.z, endpoint.w)
	return transformed


func _point_capsule_residual(point: Vector3, capsules: PackedVector4Array) -> float:
	var best := INF
	for index in range(0, capsules.size(), 2):
		var a4 := capsules[index]
		var b4 := capsules[index + 1]
		var a := Vector3(a4.x, a4.y, a4.z)
		var b := Vector3(b4.x, b4.y, b4.z)
		var segment := b - a
		var along := clampf((point - a).dot(segment) / maxf(segment.length_squared(), 0.0000001), 0.0, 1.0)
		var radius := maxf(a4.w, b4.w)
		best = minf(best, absf(point.distance_to(a + segment * along) - radius))
	return best


func _points_centroid(points: PackedVector3Array) -> Vector3:
	var sum := Vector3.ZERO
	for point in points:
		sum += point
	return sum / maxf(float(points.size()), 1.0)


func _points_coordinate_median(points: PackedVector3Array) -> Vector3:
	var xs: Array[float] = []
	var ys: Array[float] = []
	var zs: Array[float] = []
	for point in points:
		xs.append(point.x)
		ys.append(point.y)
		zs.append(point.z)
	return Vector3(_median(xs), _median(ys), _median(zs))


func _capsules_centroid(capsules: PackedVector4Array) -> Vector3:
	var sum := Vector3.ZERO
	for endpoint in capsules:
		sum += Vector3(endpoint.x, endpoint.y, endpoint.z)
	return sum / maxf(float(capsules.size()), 1.0)


func _median(values: Array[float]) -> float:
	if values.is_empty():
		return 0.0
	values.sort()
	var middle := values.size() / 2
	if values.size() % 2 == 0:
		return (values[middle - 1] + values[middle]) * 0.5
	return values[middle]


func _start_automated_external_solver() -> void:
	var script_path := ""
	var arguments: PackedStringArray = []
	var result_path := ""
	if _automation_stage == "base_capture" and _capture_mode == "axis":
		_automation_stage = "base_solve"
		script_path = ProjectSettings.globalize_path(BASE_AXIS_SOLVER_PATH)
		result_path = ProjectSettings.globalize_path(AUTOMATED_BASE_RESULT_PATH)
		arguments = PackedStringArray([
			ProjectSettings.globalize_path(AUTOMATED_BASE_CAPTURE_PATH),
			"--output",
			result_path,
		])
		_set_status(
			"solving",
			"Base sweep complete; fitting the fixed shoulder-pan axis from the reference D455...",
			0.32,
			0.0,
		)
	elif _automation_stage == "joint_capture" and _capture_mode == "joints":
		# Solve and checkpoint the chain one proximal prefix at a time. If a
		# distal stage is uncertain, the fresh base and every already-validated
		# parent joint remain useful instead of reverting to a stale placement.
		var next_prefix := _automation_next_solver_through_joint
		_automation_next_solver_through_joint = 1
		_launch_automated_joint_solver(next_prefix)
		return
	elif _automation_stage == "claw_capture" and _capture_mode == "claw":
		_automation_stage = "claw_solve"
		script_path = ProjectSettings.globalize_path(CLAW_RGB_TIP_SOLVER_PATH)
		result_path = ProjectSettings.globalize_path(AUTOMATED_CLAW_RESULT_PATH)
		arguments = PackedStringArray([
			ProjectSettings.globalize_path(AUTOMATED_CLAW_CAPTURE_PATH),
			"--output",
			result_path,
		])
		_set_status(
			"solving",
			"Comparing real and overlay fingertips across five native-D455 RGB views...",
			0.99,
			0.0,
		)
	else:
		_fail_automation("Automatic calibration entered an invalid capture stage.")
		return
	_state = "solving"
	var error := _solve_thread.start(
		_execute_external_json_solver.bind(script_path, arguments, result_path, _automation_stage)
	)
	if error != OK:
		_fail_automation("Could not start the automatic calibration solver.")


func _launch_automated_joint_solver(through_joint: int) -> void:
	if (
		not _automation_active
		or through_joint < 1
		or through_joint > 3
	):
		_fail_automation("Automatic calibration requested an invalid joint prefix.")
		return
	_automation_stage = "joint_solve"
	_automation_solver_through_joint = through_joint
	_state = "solving"
	var result_path := ProjectSettings.globalize_path(AUTOMATED_JOINT_RESULT_PATH)
	var arguments := PackedStringArray([
		ProjectSettings.globalize_path(AUTOMATED_JOINT_CAPTURE_PATH),
		ProjectSettings.globalize_path(AUTOMATED_CANDIDATE_REGISTRATION_PATH),
		"--through-joint",
		str(through_joint),
		"--output",
		result_path,
	])
	var joint_label: String = [
		"",
		"shoulder",
		"shoulder and elbow",
		"shoulder, elbow, wrist-flex, and wrist-roll",
	][through_joint]
	_set_status(
		"solving",
		(
			"Fresh base locked; validating %s from the D455 motion axes..."
			% joint_label
		),
		0.76 + float(through_joint - 1) * 0.08,
		0.0,
	)
	var error := _solve_thread.start(
		_execute_external_json_solver.bind(
			ProjectSettings.globalize_path(STAGED_JOINT_SOLVER_PATH),
			arguments,
			result_path,
			"joint_solve_%d" % through_joint,
		)
	)
	if error != OK:
		_fail_automation(
			"Could not start the automatic joint-%d solver." % through_joint
		)


func _execute_external_json_solver(
	script_path: String,
	arguments: PackedStringArray,
	result_path: String,
	stage: String,
) -> Dictionary:
	if FileAccess.file_exists(result_path):
		DirAccess.remove_absolute(result_path)
	var command_arguments := PackedStringArray([script_path])
	command_arguments.append_array(arguments)
	var output: Array = []
	var exit_code := OS.execute("/usr/bin/python3", command_arguments, output, true)
	if exit_code != 0:
		return {
			"ok": false,
			"automation_stage": stage,
			"status": "External %s rejected its evidence (exit %d): %s" % [
				stage,
				exit_code,
				"\n".join(output).strip_edges(),
			],
		}
	var file := FileAccess.open(result_path, FileAccess.READ)
	if file == null:
		return {
			"ok": false,
			"automation_stage": stage,
			"status": "External %s produced no result file." % stage,
		}
	var parsed = JSON.parse_string(file.get_as_text())
	if not parsed is Dictionary:
		return {
			"ok": false,
			"automation_stage": stage,
			"status": "External %s produced invalid JSON." % stage,
		}
	var result := (parsed as Dictionary).duplicate(true)
	result["ok"] = true
	result["automation_stage"] = stage
	return result


func _finish_automated_stage(result: Dictionary) -> void:
	_last_solution = result.duplicate(true)
	if not bool(result.get("ok", false)):
		if (
			_automation_active
			and _automation_stage == "claw_solve"
			and _automation_claw_capture_attempt
				< MAXIMUM_AUTOMATED_CLAW_CAPTURE_ATTEMPTS
		):
			# Dark jaws or the rigid camera bracket can make one wrist orientation
			# optically ambiguous. Discard that view as a unit and resample the
			# same five measured openings after a 90-degree wrist roll.
			_automation_stage = "claw_capture"
			_automation_claw_accumulated_frames = []
			_state = "complete"
			_set_status(
				"capturing",
				"Claw surface was ambiguous; automatically rotating to an alternate D455 view (%d/%d)..." % [
					_automation_claw_capture_attempt + 1,
					MAXIMUM_AUTOMATED_CLAW_CAPTURE_ATTEMPTS,
				],
				0.96,
				0.85,
			)
			call_deferred("_continue_automated_claw_capture")
			return
		if (
			_automation_active
			and _automation_stage == "joint_solve"
			and _automation_solver_through_joint == 3
			and _automation_joint_capture_attempt
				< MAXIMUM_AUTOMATED_JOINT_CAPTURE_ATTEMPTS
		):
			# A complete capture can still be optically ambiguous when the
			# custom wrist camera hides the stock jaw.  Keep the validated
			# proximal frames, request a shorter alternate wrist strategy, and
			# retry only the distal prefix instead of terminating the button.
			_automation_stage = "joint_capture"
			_automation_next_solver_through_joint = 3
			_state = "complete"
			_set_status(
				"capturing",
				(
					"Wrist views disagreed; automatically collecting alternate "
					+ "D455 pan/flex views (%d/%d)..."
				) % [
					_automation_joint_capture_attempt + 1,
					MAXIMUM_AUTOMATED_JOINT_CAPTURE_ATTEMPTS,
				],
				0.90,
				0.82,
			)
			call_deferred("_continue_automated_joint_capture")
			return
		if (
			_automation_active
			and _automation_stage == "claw_solve"
			and _automation_claw_capture_attempt
				>= MAXIMUM_AUTOMATED_CLAW_CAPTURE_ATTEMPTS
			and _finish_with_validated_prior_claw(result)
		):
			return
		_fail_automation(str(result.get(
			"status",
			"Automatic calibration stopped because one stage was uncertain.",
		)))
		return
	if _automation_stage == "base_solve":
		var base_validation := _validate_automated_base_result(result)
		if not bool(base_validation.get("ok", false)):
			_fail_automation(
				"Base-axis evidence was rejected: %s"
				% str(base_validation.get("reason", "incomplete result"))
			)
			return
		if not _write_automated_candidate_registration(result):
			_fail_automation(
				"The validated base candidate could not be staged; the prior registration was preserved."
			)
			return
		_automation_base_result = result.duplicate(true)
		if not _apply_automated_prefix_checkpoint(0, {}, 0.72):
			_fail_automation(
				"The validated fresh base could not be saved atomically; the prior registration was restored."
			)
			return
		_automation_applied_through_joint = 0
		_automation_stage = "joint_capture"
		_state = "complete"
		debug_capture_path = AUTOMATED_JOINT_CAPTURE_PATH
		_set_status(
			"capturing",
			"Fresh base rediscovered and saved; preparing multi-view servo refinement...",
			0.45,
			0.72,
		)
		call_deferred("_continue_automated_joint_capture")
		return
	if _automation_stage == "joint_solve":
		var through_joint := _automation_solver_through_joint
		var prefix_validation := _validate_automated_joint_prefix_result(
			result,
			through_joint,
		)
		if not bool(prefix_validation.get("ok", false)):
			_fail_automation(
				"Outward joint-%d evidence was rejected: %s"
				% [
					through_joint,
					str(prefix_validation.get("reason", "incomplete result")),
				]
			)
			return
		if through_joint < 3:
			var checkpoint_confidence := 0.72 + float(through_joint) * 0.05
			if not _apply_automated_prefix_checkpoint(
				through_joint,
				result,
				checkpoint_confidence,
			):
				_fail_automation(
					"The validated joint-%d prefix could not be saved atomically."
					% through_joint
				)
				return
			if not _update_automated_candidate_mapping(result):
				_fail_automation(
					"The validated joint-%d mapping could not be staged for the next outward solve."
					% through_joint
				)
				return
			_automation_applied_through_joint = through_joint
			_automation_latest_joint_result = result.duplicate(true)
			_state = "complete"
			_set_status(
				"solving",
				(
					"Fresh base and %s validated and saved; continuing outward..."
					% (
						"shoulder"
						if through_joint == 1
						else "shoulder/elbow"
					)
				),
				0.80 + float(through_joint - 1) * 0.08,
				checkpoint_confidence,
			)
			call_deferred(
				"_launch_automated_joint_solver",
				through_joint + 1,
			)
			return
		var joint_validation := _validate_automated_joint_result(result)
		if not bool(joint_validation.get("ok", false)):
			_fail_automation(
				"Outward joint evidence was rejected: %s"
				% str(joint_validation.get("reason", "incomplete result"))
			)
			return
		var fitted_offsets = result.get("fitted_offsets_degrees", [])
		var fitted_directions = result.get("fitted_directions", [])
		if (
			not fitted_offsets is Array
			or (fitted_offsets as Array).size() < 6
			or not fitted_directions is Array
			or (fitted_directions as Array).size() < 6
		):
			_fail_automation("The staged solver returned an invalid servo mapping.")
			return
		var preview: Dictionary = _automation_base_result.get("preview_transform", {})
		var candidate_global_transform := _transform_from_external_preview(preview)
		var overlay := get_node_or_null(OVERLAY_PATH)
		var candidate_local_transform := candidate_global_transform
		if overlay != null and overlay.get_parent() is Node3D:
			candidate_local_transform = (
				(overlay.get_parent() as Node3D).global_transform.affine_inverse()
				* candidate_global_transform
			)
		if (
			not candidate_local_transform.is_finite()
			or overlay == null
			or not overlay.has_method("apply_complete_automated_calibration")
		):
			_fail_automation("The overlay could not accept the completed calibration transaction.")
			return
		var metrics := {
			"method": str(result.get("method", "")),
			"base_method": str(_automation_base_result.get("method", "")),
			"base_camera_axis_disagreement_m": float(
				_automation_base_result.get("camera_axis_disagreement_m", INF)
			),
			"base_axis_fit": _automation_base_result.duplicate(true),
			"joints": result.get("joints", []),
			"wrist_roll_zero": result.get("wrist_roll_zero", {}),
			"convergence": result.get("convergence", {}),
			"fitted_directions": fitted_directions,
			"claw_calibration_pending": true,
		}
		var applied := bool(overlay.call(
			"apply_complete_automated_calibration",
			candidate_local_transform,
			fitted_directions,
			fitted_offsets,
			0.85,
			_started_unix_ms,
			metrics,
		))
		if not applied:
			_fail_automation("The completed calibration could not be saved atomically; the prior registration was restored.")
			return
		_automation_applied_through_joint = 4
		_automation_latest_joint_result = result.duplicate(true)
		_automation_stage = "claw_capture"
		_state = "complete"
		debug_capture_path = AUTOMATED_CLAW_CAPTURE_PATH
		_set_status(
			"capturing",
			"Base and arm joints validated and saved; preparing the final five-state claw fit...",
			0.94,
			0.85,
		)
		call_deferred("_continue_automated_claw_capture")
		return
	if _automation_stage == "claw_solve":
		var claw_validation := _validate_automated_claw_result(result)
		if not bool(claw_validation.get("ok", false)):
			var fallback_failure := result.duplicate(true)
			fallback_failure["reason"] = str(claw_validation.get(
				"reason",
				"incomplete result",
			))
			if (
				_automation_claw_capture_attempt
					>= MAXIMUM_AUTOMATED_CLAW_CAPTURE_ATTEMPTS
				and _finish_with_validated_prior_claw(fallback_failure)
			):
				return
			_fail_automation(
				"Five-state claw evidence was rejected: %s"
				% str(claw_validation.get("reason", "incomplete result"))
			)
			return
		var overlay := get_node_or_null(OVERLAY_PATH)
		if (
			overlay == null
			or not overlay.has_method("apply_automated_claw_calibration")
		):
			_fail_automation("The overlay could not accept the claw calibration transaction.")
			return
		if not bool(overlay.call(
			"apply_automated_claw_calibration",
			result,
			_started_unix_ms,
		)):
			_fail_automation(
				"The claw calibration could not be saved atomically; the validated arm-joint calibration remains saved."
			)
			return
		_automation_claw_result = result.duplicate(true)
		_state = "complete"
		_set_status(
			"complete",
			"Full automatic base, arm-joint, wrist-roll, and five-state claw calibration validated and saved.",
			1.0,
			float(result.get("confidence", 0.90)),
		)
		_end_automation(true)
		return
	_fail_automation("Automatic calibration returned from an unknown solver stage.")


func _finish_with_validated_prior_claw(optical_failure: Dictionary) -> bool:
	var overlay := get_node_or_null(OVERLAY_PATH)
	if (
		overlay == null
		or not overlay.has_method(
			"finalize_automated_claw_with_validated_prior"
		)
		or not bool(overlay.call(
			"finalize_automated_claw_with_validated_prior",
			_started_unix_ms,
			optical_failure,
		))
	):
		return false
	_automation_claw_result = {
		"fallback": true,
		"optical_failure": optical_failure.duplicate(true),
	}
	_state = "complete"
	_set_status(
		"complete",
		(
			"Automatic base and arm calibration through wrist validated and saved. "
			+ "The existing validated five-state claw curve was retained because "
			+ "all five D455 jaw views were optically ambiguous."
		),
		1.0,
		0.85,
	)
	_end_automation(true)
	return true


func _validate_automated_base_result(result: Dictionary) -> Dictionary:
	if (
		str(result.get("type", "")) != "so101_base_axis_fit"
		or str(result.get("method", ""))
			!= "settled_shoulder_pan_revolute_axis"
	):
		return {"ok": false, "reason": "wrong base fit type or method"}
	var reference_camera := str(result.get(
		"reference_camera",
		result.get("reference_camera_serial", ""),
	))
	if not _automated_has_required_d455(
		result.get("camera_fits", null),
		reference_camera,
	):
		return {
			"ok": false,
			"reason": "the fixed base axis was not fitted from the capture-selected reference camera",
		}
	var disagreement := float(result.get("camera_axis_disagreement_m", INF))
	if not is_finite(disagreement) or disagreement > 0.025:
		return {
			"ok": false,
			"reason": "camera base-axis disagreement exceeds 25 mm",
		}
	var preview = result.get("preview_transform", null)
	if (
		not preview is Dictionary
		or not _transform_from_external_preview(preview as Dictionary).is_finite()
	):
		return {"ok": false, "reason": "base preview transform is missing or non-finite"}
	return {"ok": true}


func _validate_automated_claw_result(result: Dictionary) -> Dictionary:
	var method := str(result.get("method", ""))
	if (
		str(result.get("type", "")) != "so101_claw_visual_fit"
		or method not in [
			"d455_five_state_independent_mesh_fit",
			"d455_native_rgb_multiview_tip_fit",
		]
	):
		return {"ok": false, "reason": "wrong claw fit type or method"}
	if "d455" not in str(result.get("reference_camera", "")).to_lower():
		return {"ok": false, "reason": "the fit did not use the reference D455"}
	var normalized = result.get("gripper_angle_samples_normalized", null)
	var degrees = result.get("gripper_angle_samples_degrees", null)
	if (
		not normalized is Array
		or not degrees is Array
		or (normalized as Array).size() != 5
		or (degrees as Array).size() != 5
	):
		return {"ok": false, "reason": "exactly five fitted claw states are required"}
	for index in range(5):
		if (
			not is_finite(float((normalized as Array)[index]))
			or not is_finite(float((degrees as Array)[index]))
			or (
				index > 0
				and (
					float((normalized as Array)[index])
						<= float((normalized as Array)[index - 1])
					or float((degrees as Array)[index])
						<= float((degrees as Array)[index - 1])
				)
			)
		):
			return {"ok": false, "reason": "claw samples are non-finite or non-monotonic"}
	if absf(float((degrees as Array)[0]) + 10.0) > 5.0:
		return {"ok": false, "reason": "the physical closed state missed the stock -10 degree stop"}
	if float((degrees as Array)[-1]) < 68.0:
		return {"ok": false, "reason": "the fitted open-state span is too small"}
	var confidence := float(result.get("confidence", 0.0))
	if method in [
		"d455_native_rgb_multiview_tip_fit",
	]:
		if int(result.get("validation_view_count", 0)) < 2:
			return {"ok": false, "reason": "fewer than two native-RGB wrist views validated"}
		if not bool(result.get("all_views_improved", false)):
			return {"ok": false, "reason": "the candidate did not improve every usable wrist view"}
		var pixel_residual := float(result.get("median_tip_residual_px", INF))
		if not is_finite(pixel_residual) or pixel_residual > 8.0:
			return {"ok": false, "reason": "native-RGB fingertip residual exceeds 8 pixels"}
		if not is_finite(confidence) or confidence < 0.72:
			return {"ok": false, "reason": "native-RGB claw confidence is below 72%"}
		return {"ok": true}
	var residual := float(result.get("median_residual_m", INF))
	if not is_finite(confidence) or confidence < 0.72:
		return {"ok": false, "reason": "claw confidence is below 72%"}
	if not is_finite(residual) or residual > 0.018:
		return {"ok": false, "reason": "claw surface residual exceeds 18 mm"}
	return {"ok": true}


func _validate_automated_joint_prefix_result(
	result: Dictionary,
	through_joint: int,
) -> Dictionary:
	if through_joint < 1 or through_joint > 3:
		return {"ok": false, "reason": "joint prefix is outside 1..3"}
	if (
		str(result.get("type", "")) != "so101_staged_joint_fit"
		or str(result.get("method", ""))
			!= "outward_next_servo_revolute_axis_chain"
		or str(result.get("registration_source", ""))
			!= "settled_shoulder_pan_revolute_axis"
	):
		return {
			"ok": false,
			"reason": "joint prefix is not tied to the freshly rediscovered base",
		}
	var fitted_offsets = result.get("fitted_offsets_degrees", null)
	var fitted_directions = result.get("fitted_directions", null)
	if (
		not fitted_offsets is Array
		or (fitted_offsets as Array).size() < 6
		or not fitted_directions is Array
		or (fitted_directions as Array).size() < 6
	):
		return {"ok": false, "reason": "complete six-servo mapping is missing"}
	for index in range(6):
		var direction := float((fitted_directions as Array)[index])
		if (
			not is_finite(direction)
			or absf(absf(direction) - 1.0) > 0.01
			or not is_finite(float((fitted_offsets as Array)[index]))
		):
			return {"ok": false, "reason": "servo mapping contains an invalid value"}
	var convergence = result.get("convergence", null)
	if not convergence is Dictionary or not bool(convergence.get("converged", false)):
		return {"ok": false, "reason": "joint prefix did not converge"}
	var history = (convergence as Dictionary).get("pass_history", null)
	if not history is Array or (history as Array).is_empty():
		return {"ok": false, "reason": "joint prefix has no fixed-point history"}
	var final_pass = (history as Array)[-1]
	if (
		not final_pass is Dictionary
		or absf(float((final_pass as Dictionary).get(
			"maximum_correction_degrees",
			INF,
		))) > 2.0
	):
		return {
			"ok": false,
			"reason": "final joint-prefix pass still moves by more than 2 degrees",
		}
	var expected_names := ["shoulder_lift", "elbow_flex", "wrist_flex"]
	var joints = result.get("joints", null)
	if not joints is Array or (joints as Array).size() != through_joint:
		return {
			"ok": false,
			"reason": "joint prefix does not contain exactly %d stages" % through_joint,
		}
	for index in range(through_joint):
		var joint = (joints as Array)[index]
		if (
			not joint is Dictionary
			or int((joint as Dictionary).get("joint_index", -1)) != index + 1
			or str((joint as Dictionary).get("joint", "")) != expected_names[index]
			or absf(float((joint as Dictionary).get(
				"fixed_point_delta_degrees",
				INF,
			))) > 2.0
			or not _automated_has_required_d455(
				(joint as Dictionary).get("camera_results", null)
			)
		):
			return {
				"ok": false,
				"reason": "%s lacks a D455 fixed-point stage" % expected_names[index],
			}
	if through_joint == 3:
		var wrist_flex: Dictionary = (joints as Array)[2]
		if (
			str(wrist_flex.get("method", ""))
			!= "direct_dynamic_next_servo_axis_line_collapse"
			or wrist_flex.has("mapped_link")
		):
			return {
				"ok": false,
				"reason": (
					"wrist-flex stage did not isolate motion from the "
					+ "rigid camera bracket"
				),
			}
	return {"ok": true}


func _validate_automated_joint_result(result: Dictionary) -> Dictionary:
	var prefix_validation := _validate_automated_joint_prefix_result(result, 3)
	if not bool(prefix_validation.get("ok", false)):
		return prefix_validation
	if (
		str(result.get("type", "")) != "so101_staged_joint_fit"
		or str(result.get("method", ""))
			!= "outward_next_servo_revolute_axis_chain"
		or str(result.get("registration_source", ""))
			!= "settled_shoulder_pan_revolute_axis"
	):
		return {
			"ok": false,
			"reason": "joint result is not tied to the freshly locked base candidate",
		}
	var fitted_offsets = result.get("fitted_offsets_degrees", null)
	var fitted_directions = result.get("fitted_directions", null)
	if (
		not fitted_offsets is Array
		or (fitted_offsets as Array).size() < 6
		or not fitted_directions is Array
		or (fitted_directions as Array).size() < 6
	):
		return {"ok": false, "reason": "complete six-servo mapping is missing"}
	for index in range(6):
		var direction := float((fitted_directions as Array)[index])
		if (
			not is_finite(direction)
			or absf(absf(direction) - 1.0) > 0.01
			or not is_finite(float((fitted_offsets as Array)[index]))
		):
			return {"ok": false, "reason": "servo mapping contains an invalid value"}
	var convergence = result.get("convergence", null)
	if not convergence is Dictionary or not bool(convergence.get("converged", false)):
		return {"ok": false, "reason": "outward chain did not converge"}
	var history = (convergence as Dictionary).get("pass_history", null)
	if not history is Array or (history as Array).is_empty():
		return {"ok": false, "reason": "outward chain has no fixed-point pass history"}
	var final_pass = (history as Array)[-1]
	if (
		not final_pass is Dictionary
		or absf(float((final_pass as Dictionary).get(
			"maximum_correction_degrees",
			INF,
		))) > 2.0
	):
		return {"ok": false, "reason": "final outward pass still moves by more than 2 degrees"}
	var joints = result.get("joints", null)
	var expected_names := ["shoulder_lift", "elbow_flex", "wrist_flex"]
	if not joints is Array or (joints as Array).size() != expected_names.size():
		return {"ok": false, "reason": "exactly three outward joint stages are required"}
	for index in range(expected_names.size()):
		var joint = (joints as Array)[index]
		if (
			not joint is Dictionary
			or int((joint as Dictionary).get("joint_index", -1)) != index + 1
			or str((joint as Dictionary).get("joint", "")) != expected_names[index]
			or absf(float((joint as Dictionary).get(
				"fixed_point_delta_degrees",
				INF,
			))) > 2.0
			or not _automated_has_required_d455(
				(joint as Dictionary).get("camera_results", null)
			)
		):
			return {
				"ok": false,
				"reason": "%s lacks a D455 fixed-point stage" % expected_names[index],
			}
	var wrist_flex: Dictionary = (joints as Array)[2]
	if (
		str(wrist_flex.get("method", ""))
			!= "direct_dynamic_next_servo_axis_line_collapse"
		or wrist_flex.has("mapped_link")
	):
		return {
			"ok": false,
			"reason": "wrist-flex stage did not exclude the unmapped camera attachment",
		}
	if bool(wrist_flex.get("fixed_point_only_shallow_basin", false)):
		var decisive_prior_wrist_pass := false
		for pass_index in range((history as Array).size() - 1):
			var pass_variant = (history as Array)[pass_index]
			if not pass_variant is Dictionary:
				continue
			var pass_data: Dictionary = pass_variant
			var corrections = pass_data.get("corrections_degrees", [])
			for evidence_variant in pass_data.get("joint_evidence", []):
				if not evidence_variant is Dictionary:
					continue
				var evidence: Dictionary = evidence_variant
				if (
					str(evidence.get("joint", "")) == "wrist_flex"
					and corrections is Array
					and (corrections as Array).size() >= 3
					and absf(float((corrections as Array)[2])) > 2.0
					and float(evidence.get("loss_peak_ratio", 0.0)) >= 1.05
					and float(evidence.get(
						"wrong_direction_loss_ratio",
						0.0,
					)) >= 1.12
					and int(evidence.get(
						"minimum_dynamic_points_per_pose",
						0,
					)) >= 100
				):
					decisive_prior_wrist_pass = true
		if not decisive_prior_wrist_pass:
			return {
				"ok": false,
				"reason": (
					"shallow wrist fixed point lacks an earlier decisive "
					+ "D455 correction pass"
				),
			}
	var wrist = result.get("wrist_roll_zero", null)
	if not wrist is Dictionary:
		return {"ok": false, "reason": "wrist-roll sign and zero evidence is missing"}
	var wrist_metrics: Dictionary = wrist
	var wrist_method := str(wrist_metrics.get("method", ""))
	var preserved_prior_wrist := (
		wrist_method
		== "preserved_validated_prior_wrist_zero_after_d455_motion_sign_check"
	)
	if (
		wrist_method not in [
			"multiangle_distal_stock_gripper_sign_and_zero",
			"preserved_validated_prior_wrist_zero_after_d455_motion_sign_check",
		]
		or str(wrist_metrics.get("mapped_link", "")) != "gripper_link"
		or str(wrist_metrics.get("mapped_region", "")) != "distal_fixed_jaw"
		or not bool(wrist_metrics.get("unmapped_camera_attachment_excluded", false))
		or str(wrist_metrics.get(
			"camera_attachment_topology",
			"single_rigid_piece",
		)) != "single_rigid_piece"
		or not bool(wrist_metrics.get("moving_jaw_orientation_check", false))
		or not bool(wrist_metrics.get("model_convention_orientation_check", false))
		or not bool(wrist_metrics.get("coupling_converged", false))
		or not bool(wrist_metrics.get(
			"direction_locked_from_reference_camera_motion",
			false,
		))
		or absf(float(wrist_metrics.get("fixed_point_delta_degrees", INF))) > 2.0
		or not bool(wrist_metrics.get("reference_camera_validation_required", false))
		or str(wrist_metrics.get("reference_camera_serial", "")).is_empty()
		or int(wrist_metrics.get("validating_camera_count", 0)) < 1
		or not _automated_has_required_d455(
			wrist_metrics.get("camera_results", null),
			str(wrist_metrics.get("reference_camera_serial", "")),
		)
	):
		return {
			"ok": false,
			"reason": "wrist-roll lacks attachment-safe D455 orientation and fixed-point proof",
		}
	if preserved_prior_wrist:
		var prior = wrist_metrics.get("prior_validation", null)
		var preliminary = wrist_metrics.get("preliminary_direction_fit", null)
		if (
			not bool(wrist_metrics.get("fresh_orientation_ambiguous", false))
			or not bool(wrist_metrics.get(
				"orientation_preserved_not_reestimated",
				false,
			))
			or not _automated_validated_prior_wrist_zero(prior)
			or not preliminary is Dictionary
			or str((preliminary as Dictionary).get("method", ""))
				!= "reference_camera_repeated_motion_collapse_direction"
		):
			return {
				"ok": false,
				"reason": "ambiguous D455 wrist zero lacks a validated prior zero and fresh motion-sign proof",
			}
	else:
		if (
			int(wrist_metrics.get("multiangle_pose_count", 0)) < 10
			or int(wrist_metrics.get("multiangle_viewpoint_count", 0)) < 2
			or float(wrist_metrics.get(
				"visible_view_offset_disagreement_degrees",
				INF,
			)) > 12.0
		):
			return {
				"ok": false,
				"reason": "fresh D455 wrist orientation lacks multi-angle agreement",
			}
	if absf(
		float((fitted_directions as Array)[4])
		- float(wrist_metrics.get("fitted_direction", INF))
	) > 0.01:
		return {"ok": false, "reason": "saved wrist direction disagrees with measured wrist evidence"}
	for camera_variant in wrist_metrics.get("camera_results", []):
		if not camera_variant is Dictionary:
			return {"ok": false, "reason": "wrist camera evidence is malformed"}
		var camera: Dictionary = camera_variant
		if preserved_prior_wrist:
			var dynamic_counts = camera.get("dynamic_points_per_pose", null)
			if (
				not dynamic_counts is Array
				or (dynamic_counts as Array).is_empty()
				or float(camera.get("collapse_loss_m", INF)) > 0.035
				or float(camera.get("wrong_direction_loss_ratio", 0.0)) < 1.12
				or int((dynamic_counts as Array).min()) < 100
			):
				return {
					"ok": false,
					"reason": "fresh D455 wrist motion-sign evidence failed its quality gate",
				}
			continue
		var evidence_class := str(camera.get("evidence_class", ""))
		var strong_quality := (
			evidence_class == "strong_full_surface"
			and float(camera.get("mesh_loss_m", INF)) <= 0.018
			and float(camera.get("mapped_surface_support_fraction", 0.0)) >= 0.55
		)
		var partial_quality := (
			evidence_class == "unique_partial_surface"
			and float(camera.get("mesh_loss_m", INF)) <= 0.025
			and float(camera.get("partial_surface_support_distance_m", INF)) <= 0.025
			and float(camera.get("partial_surface_support_fraction", 0.0)) >= 0.40
		)
		if (
			not bool(camera.get("visible_stock_gripper", false))
			or int(camera.get("moving_jaw_visible_pose_count", 0)) < 2
			or not (strong_quality or partial_quality)
			or float(camera.get("mesh_peak_ratio", 0.0)) < 1.06
			or float(camera.get("local_orientation_basin_width_degrees", INF)) > 25.0
		):
			return {
				"ok": false,
				"reason": "one wrist camera view failed its mapped-stock-gripper quality gate",
			}
	return {"ok": true}


func _automated_has_required_d455(
	camera_results,
	reference_camera: String = "",
) -> bool:
	if not camera_results is Array:
		return false
	for camera_variant in camera_results:
		if not camera_variant is Dictionary:
			continue
		var camera_name := str((camera_variant as Dictionary).get(
			"camera",
			(camera_variant as Dictionary).get("name", ""),
		))
		if camera_name.is_empty():
			continue
		if (
			reference_camera.is_empty()
			or reference_camera == camera_name
			or reference_camera.to_lower() in camera_name.to_lower()
		):
			return true
	return false


func _automated_validated_prior_wrist_zero(prior_variant) -> bool:
	if not prior_variant is Dictionary:
		return false
	var prior: Dictionary = prior_variant
	if (
		absf(float(prior.get("fixed_point_delta_degrees", INF))) > 2.0
		or not bool(prior.get("moving_jaw_orientation_check", false))
	):
		return false
	var method := str(prior.get("method", ""))
	if method == "dual_realsense_native_rgb_tip_projection":
		return int(prior.get("validating_camera_count", 0)) >= 2
	if (
		method
		== "preserved_validated_prior_wrist_zero_after_d455_motion_sign_check"
	):
		return (
			bool(prior.get("orientation_preserved_not_reestimated", false))
			and _automated_validated_prior_wrist_zero(
				prior.get("prior_validation", null)
			)
		)
	if (
		method != "multiangle_distal_stock_gripper_sign_and_zero"
		or not bool(prior.get("model_convention_orientation_check", false))
		or not bool(prior.get("coupling_converged", false))
		or not bool(prior.get("reference_camera_validation_required", false))
		or str(prior.get("reference_camera_serial", "")).is_empty()
		or int(prior.get("validating_camera_count", 0)) < 1
		or int(prior.get("multiangle_pose_count", 0)) < 10
		or int(prior.get("multiangle_viewpoint_count", 0)) < 2
	):
		return false
	for camera_variant in prior.get("camera_results", []):
		if not camera_variant is Dictionary:
			continue
		var camera: Dictionary = camera_variant
		if (
			str(prior.get("reference_camera_serial", "")).to_lower()
			not in str(camera.get("camera", "")).to_lower()
		):
			continue
		var evidence_class := str(camera.get("evidence_class", ""))
		var strong := (
			evidence_class == "strong_full_surface"
			and float(camera.get("mesh_loss_m", INF)) <= 0.018
			and float(camera.get("mapped_surface_support_fraction", 0.0)) >= 0.55
		)
		var partial := (
			evidence_class == "unique_partial_surface"
			and float(camera.get("mesh_loss_m", INF)) <= 0.025
			and float(camera.get("partial_surface_support_distance_m", INF)) <= 0.025
			and float(camera.get("partial_surface_support_fraction", 0.0)) >= 0.40
		)
		if (
			bool(camera.get("visible_stock_gripper", false))
			and int(camera.get("moving_jaw_visible_pose_count", 0)) >= 2
			and (strong or partial)
			and float(camera.get("mesh_peak_ratio", 0.0)) >= 1.06
			and float(camera.get(
				"local_orientation_basin_width_degrees",
				INF,
			)) <= 25.0
		):
			return true
	return false


func _continue_automated_joint_capture() -> void:
	if not _automation_active or _automation_stage != "joint_capture":
		return
	_automation_joint_capture_attempt += 1
	start_joint_alignment_calibration(_automation_force_editor_auto_move)
	if _state == "failed":
		_end_automation(false)


func _continue_automated_claw_capture() -> void:
	if not _automation_active or _automation_stage != "claw_capture":
		return
	_automation_claw_capture_attempt += 1
	start_claw_visual_calibration(_automation_force_editor_auto_move)
	if _state == "failed":
		_end_automation(false)


func _continue_automated_base_capture() -> void:
	if not _automation_active or _automation_stage != "base_capture":
		return
	_automation_base_capture_attempt += 1
	start_base_axis_capture(_automation_force_editor_auto_move)
	if _state == "failed":
		_end_automation(false)


func _automated_candidate_local_transform() -> Transform3D:
	var preview = _automation_base_result.get("preview_transform", null)
	if not preview is Dictionary:
		return Transform3D(Basis(), Vector3(INF, INF, INF))
	var candidate_global := _transform_from_external_preview(preview)
	if not candidate_global.is_finite():
		return candidate_global
	var overlay := get_node_or_null(OVERLAY_PATH)
	if overlay != null and overlay.get_parent() is Node3D:
		return (
			(overlay.get_parent() as Node3D).global_transform.affine_inverse()
			* candidate_global
		)
	return candidate_global


func _apply_automated_prefix_checkpoint(
	through_joint: int,
	joint_result: Dictionary,
	confidence: float,
) -> bool:
	var overlay := get_node_or_null(OVERLAY_PATH)
	if (
		overlay == null
		or not overlay.has_method("apply_validated_automated_prefix")
	):
		return false
	var directions = joint_result.get(
		"fitted_directions",
		overlay.get("joint_angle_directions"),
	)
	var offsets = joint_result.get(
		"fitted_offsets_degrees",
		overlay.get("joint_angle_offsets_degrees"),
	)
	if directions is PackedFloat32Array:
		directions = Array(directions)
	if offsets is PackedFloat32Array:
		offsets = Array(offsets)
	if not directions is Array or not offsets is Array:
		return false
	var metrics := {
		"method": (
			"settled_shoulder_pan_revolute_axis"
			if through_joint == 0
			else "outward_next_servo_revolute_axis_chain"
		),
		"partial": true,
		"calibrated_through_joint": through_joint,
		"base_axis_fit": _automation_base_result.duplicate(true),
		"joints": joint_result.get("joints", []),
		"convergence": joint_result.get("convergence", {}),
		"camera_attachment_fit_policy": "unmodelled_and_excluded_from_stock_gripper_fit",
	}
	return bool(overlay.call(
		"apply_validated_automated_prefix",
		_automated_candidate_local_transform(),
		directions,
		offsets,
		through_joint,
		confidence,
		_started_unix_ms,
		metrics,
	))


func _update_automated_candidate_mapping(result: Dictionary) -> bool:
	var path := ProjectSettings.globalize_path(
		AUTOMATED_CANDIDATE_REGISTRATION_PATH
	)
	var source := FileAccess.open(path, FileAccess.READ)
	if source == null:
		return false
	var parsed = JSON.parse_string(source.get_as_text())
	if not parsed is Dictionary:
		return false
	var directions = result.get("fitted_directions", null)
	var offsets = result.get("fitted_offsets_degrees", null)
	if (
		not directions is Array
		or (directions as Array).size() < 6
		or not offsets is Array
		or (offsets as Array).size() < 6
	):
		return false
	var payload: Dictionary = (parsed as Dictionary).duplicate(true)
	payload["joint_angle_directions"] = (directions as Array).duplicate()
	payload["joint_angle_offsets_degrees"] = (offsets as Array).duplicate()
	payload["calibrated_through_joint"] = _automation_solver_through_joint
	var pending_path := path + ".pending"
	if FileAccess.file_exists(pending_path):
		DirAccess.remove_absolute(pending_path)
	var pending := FileAccess.open(pending_path, FileAccess.WRITE)
	if pending == null:
		return false
	pending.store_string(JSON.stringify(payload, "\t"))
	pending.flush()
	pending.close()
	var rename_error := DirAccess.rename_absolute(pending_path, path)
	if rename_error != OK:
		DirAccess.remove_absolute(pending_path)
		return false
	return true


func _write_automated_candidate_registration(base_result: Dictionary) -> bool:
	var preview = base_result.get("preview_transform", null)
	if not preview is Dictionary:
		return false
	var transform := _transform_from_external_preview(preview)
	if not transform.is_finite():
		return false
	var overlay := get_node_or_null(OVERLAY_PATH)
	if overlay == null:
		return false
	var directions = overlay.get("joint_angle_directions")
	var offsets = overlay.get("joint_angle_offsets_degrees")
	if not directions is PackedFloat32Array or not offsets is PackedFloat32Array:
		return false
	var payload := {
		"type": "so101_robot_registration_candidate",
		"basis_x": _vector3_to_array(transform.basis.x),
		"basis_y": _vector3_to_array(transform.basis.y),
		"basis_z": _vector3_to_array(transform.basis.z),
		"origin": _vector3_to_array(transform.origin),
		"joint_angle_directions": Array(directions),
		"joint_angle_offsets_degrees": Array(offsets),
		"registration_source": "settled_shoulder_pan_revolute_axis",
		"camera_attachment_fit_policy": "physically_kept_opposite_d455",
		"saved": false,
	}
	if overlay.has_method("get_registration_status"):
		var saved_registration = overlay.call("get_registration_status")
		if saved_registration is Dictionary:
			var refinement = saved_registration.get("joint_refinement", {})
			if refinement is Dictionary:
				var prior_wrist = refinement.get("wrist_roll_zero", {})
				if prior_wrist is Dictionary and not prior_wrist.is_empty():
					payload["prior_wrist_roll_zero_evidence"] = (
						prior_wrist.duplicate(true)
					)
	var file := FileAccess.open(AUTOMATED_CANDIDATE_REGISTRATION_PATH, FileAccess.WRITE)
	if file == null:
		return false
	file.store_string(JSON.stringify(payload, "\t"))
	return true


func _transform_from_external_preview(preview: Dictionary) -> Transform3D:
	for key in ["basis_x", "basis_y", "basis_z", "origin"]:
		var values = preview.get(key, null)
		if not values is Array or (values as Array).size() < 3:
			return Transform3D(Basis(), Vector3(INF, INF, INF))
	var basis := Basis(
		_array_to_vector3(preview["basis_x"]),
		_array_to_vector3(preview["basis_y"]),
		_array_to_vector3(preview["basis_z"]),
	).orthonormalized()
	return Transform3D(basis, _array_to_vector3(preview["origin"]))


func _array_to_vector3(values: Array) -> Vector3:
	return Vector3(float(values[0]), float(values[1]), float(values[2]))


func _fail_automation(message: String) -> void:
	var preserved_message := "Existing robot registration was not changed."
	if _automation_applied_through_joint == 0:
		preserved_message = (
			"The fresh validated base registration remains saved; "
			+ "unvalidated joint mappings were left unchanged."
		)
	elif _automation_applied_through_joint > 0:
		var joint_name := str(REFINABLE_JOINT_NAMES.get(
			_automation_applied_through_joint,
			"joint %d" % _automation_applied_through_joint,
		))
		preserved_message = (
			"The fresh base and validated joints through %s remain saved; "
			% joint_name
			+ "unvalidated distal mappings were left unchanged."
		)
	_end_automation(false)
	_fail("%s %s" % [message, preserved_message])


func _end_automation(_success: bool) -> void:
	_automation_active = false
	_automation_stage = ""
	_automation_base_result = {}
	_automation_base_capture_attempt = 0
	_automation_base_accumulated_frames = []
	_automation_joint_capture_attempt = 0
	_automation_joint_accumulated_frames = []
	_automation_solver_through_joint = 0
	_automation_next_solver_through_joint = 1
	_automation_applied_through_joint = -1
	_automation_latest_joint_result = {}
	_automation_claw_result = {}
	_automation_claw_capture_attempt = 0
	_automation_claw_accumulated_frames = []
	diagnostic_only = _automation_previous_diagnostic_only
	debug_capture_path = _automation_previous_debug_capture_path
	var candidate_path := ProjectSettings.globalize_path(AUTOMATED_CANDIDATE_REGISTRATION_PATH)
	if FileAccess.file_exists(candidate_path):
		DirAccess.remove_absolute(candidate_path)


func _finish_solution(result: Dictionary) -> void:
	_stop_editor_sweep("calibration fit complete")
	_last_solution = result.duplicate(true)
	if not bool(result.get("ok", false)):
		_fail(str(result.get("status", "Arm motion fit was not confident enough.")), float(result.get("confidence", 0.0)))
		return
	if _capture_mode == "wrist":
		var wrist_metrics = result.get("wrist_roll_zero", null)
		var fitted_directions = result.get("fitted_directions", null)
		var fitted_offsets = result.get("fitted_offsets_degrees", null)
		if (
			not wrist_metrics is Dictionary
			or not fitted_directions is Array
			or (fitted_directions as Array).size() < 6
			or not fitted_offsets is Array
			or (fitted_offsets as Array).size() < 6
			or (
				not bool(
					(wrist_metrics as Dictionary).get(
						"moving_jaw_orientation_check",
						false,
					)
				)
				and not bool(
					(wrist_metrics as Dictionary).get(
						"model_convention_orientation_check",
						false,
					)
				)
			)
			or not bool(
				(wrist_metrics as Dictionary).get(
					"reference_camera_validation_required",
					false,
				)
			)
			or str(
				(wrist_metrics as Dictionary).get(
					"reference_camera_serial",
					"",
				)
			).is_empty()
			or int((wrist_metrics as Dictionary).get("validating_camera_count", 0)) < 1
			or not _automated_has_required_d455(
				(wrist_metrics as Dictionary).get("camera_results", null),
				str((wrist_metrics as Dictionary).get(
					"reference_camera_serial",
					"",
				)),
			)
				or not bool(
					(wrist_metrics as Dictionary).get(
						"unmapped_camera_attachment_excluded",
						false,
					)
				)
				or str(
					(wrist_metrics as Dictionary).get(
						"camera_attachment_topology",
						"single_rigid_piece",
					)
				) != "single_rigid_piece"
			or int((wrist_metrics as Dictionary).get("multiangle_viewpoint_count", 0)) < 2
		):
			_fail("The lowered wrist fit did not pass its D455 orientation checks.")
			return
		var wrist_overlay := get_node_or_null(OVERLAY_PATH)
		if wrist_overlay == null or not wrist_overlay.has_method("apply_wrist_roll_calibration"):
			_fail("SO-101 overlay could not accept the wrist-roll fit.")
			return
		var wrist_applied := bool(wrist_overlay.call(
			"apply_wrist_roll_calibration",
			float((fitted_directions as Array)[4]),
			float((fitted_offsets as Array)[4]),
			0.85,
			_started_unix_ms,
			(wrist_metrics as Dictionary),
		))
		if not wrist_applied:
			_fail("The wrist-roll result could not be saved atomically.")
			return
		_state = "complete"
		_set_status(
			"complete",
			"Lowered D455 wrist-roll calibration validated and saved.",
			1.0,
			0.85,
		)
		return
	if diagnostic_only:
		var diagnostic_confidence := float(result.get("confidence", 0.0))
		_state = "complete"
		_set_status(
			"complete",
			"Diagnostic only; nothing saved. %s" % str(result.get("status", "Fit complete.")),
			1.0,
			diagnostic_confidence,
		)
		return
	if str(result.get("calibration_mode", "base")) == "joints":
		var joint_overlay := get_node_or_null(OVERLAY_PATH)
		if joint_overlay == null or not joint_overlay.has_method("apply_joint_offset_refinement"):
			_fail("SO-101 overlay could not accept the solved servo offsets.")
			return
		var joint_confidence := float(result.get("confidence", 0.0))
		var joint_applied := bool(joint_overlay.call(
			"apply_joint_offset_refinement",
			result.get("offset_deltas_degrees", []),
			joint_confidence,
			_started_unix_ms,
			result.get("joint_metrics", {}),
		))
		if not joint_applied:
			_fail("A newer robot calibration exists or the servo offsets were invalid.", joint_confidence)
			return
		_state = "complete"
		_set_status(
			"complete",
			str(result.get("status", "Staged servo alignment refined and saved.")),
			1.0,
			joint_confidence,
		)
		return
	var overlay := get_node_or_null(OVERLAY_PATH)
	if overlay == null or not overlay.has_method("apply_calibrated_global_transform"):
		_fail("SO-101 overlay could not accept the solved transform.")
		return
	var transform: Transform3D = result["transform"]
	var confidence := float(result.get("confidence", 0.0))
	var message := str(result.get("status", "Arm position calibrated and saved."))
	var plane_source := str(result.get("base_plane_source", ""))
	var source := "markerless_motion_fit"
	if plane_source == "depth_table_plane":
		source = "depth_plane_constrained_motion_fit"
	elif bool(result.get("base_plane_constrained", false)):
		source = "aruco_plane_constrained_motion_fit"
	var applied := bool(overlay.call("apply_calibrated_global_transform", transform, confidence, _started_unix_ms, source))
	if not applied:
		_fail("A newer robot-position calibration already exists; this older result was discarded.", confidence)
		return
	_state = "complete"
	_set_status("complete", message, 1.0, confidence)


func _trusted_search_limit_reached(rotation_degrees: float, translation_m: float) -> bool:
	return (
		rotation_degrees >= TRUSTED_HEADING_LIMIT_DEGREES * TRUSTED_LIMIT_SATURATION_RATIO
		or translation_m >= TRUSTED_TRANSLATION_LIMIT_M * TRUSTED_LIMIT_SATURATION_RATIO
	)


func _set_status(state: String, message: String, progress: float, confidence: float) -> void:
	_status = {
		"type": STATUS_TYPE,
		"state": state,
		"progress": clampf(progress, 0.0, 1.0),
		"frames": _frames.size(),
		"confidence": clampf(confidence, 0.0, 1.0),
		"message": message,
		"calibration_mode": _capture_mode,
		"depth_renderers": _capture_renderer_count,
		"moving_points": _capture_moving_points,
		"sent_unix_ms": int(Time.get_unix_time_from_system() * 1000.0),
	}
	_emit_status(true)


func _fail(message: String, confidence: float = 0.0) -> void:
	_stop_editor_sweep("calibration failed")
	_state = "failed"
	_editor_status_until_msec = Time.get_ticks_msec() + 30000
	_set_status("failed", message, float(_frames.size()) / maxf(float(minimum_pose_frames), 1.0), confidence)
	push_warning("SO-101 robot calibration rejected: %s" % message)


func _emit_status(force: bool) -> void:
	var now := Time.get_ticks_msec()
	if Engine.is_editor_hint() and _state not in ["capturing", "solving"] and now > _editor_status_until_msec:
		return
	if not force and now - _last_status_send_msec < 200:
		return
	_last_status_send_msec = now
	_status_udp.put_packet(JSON.stringify(_status).to_utf8_buffer())
