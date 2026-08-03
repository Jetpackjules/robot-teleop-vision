from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_web_full_arm_button_uses_progressive_automated_pipeline() -> None:
    source = (
        ROOT / "robot_modules" / "so101" / "godot" / "so101_module.gd"
    ).read_text(encoding="utf-8")
    marker = "func start_full_calibration(from_editor: bool = false) -> void:"
    block = source.split(marker, 1)[1].split("func refine_calibration", 1)[0]

    assert '"start_automated_arm_calibration"' in block
    assert '"start_arm_position_calibration"' not in block


def test_distal_disagreement_triggers_alternate_d455_recapture() -> None:
    source = (
        ROOT / "robot_modules/so101/godot/so101_motion_calibrator.gd"
    ).read_text(encoding="utf-8")

    assert 'and _automation_solver_through_joint == 3' in source
    assert '_automation_next_solver_through_joint = 3' in source
    assert '"view_strategy_attempt"' in source
    assert "automatically collecting alternate" in source


def test_claw_stage_uses_measured_gripper_tolerance_and_bounded_retry() -> None:
    source = (
        ROOT / "robot_modules/so101/godot/so101_motion_calibrator.gd"
    ).read_text(encoding="utf-8")

    assert 'if _capture_mode == "claw" and maximum_tracking_joint == 5:' in source
    assert "MAXIMUM_AUTOMATED_CLAW_CAPTURE_ATTEMPTS := 5" in source
    assert "_merge_automated_claw_capture_frames(_frames)" in source
    assert 'call_deferred("_continue_automated_claw_capture")' in source
    assert 'if _capture_mode == "claw":' in source
    assert "float(_last_accepted_pose[5]) - float(pose[5])" in source
    assert "_can_resume_recent_pending_claw" in source
    assert "PENDING_CLAW_RESUME_MAXIMUM_AGE_MSEC" in source
    assert '== "full_automated_motion_axis_calibration"' in source
    assert "transform.basis.y.normalized().dot(Vector3.UP) >= 0.25" in source
    assert '_automation_stage == "claw_solve"' in source
    assert "automatically rotating to an alternate D455 view" in source
    assert "_best_complete_automated_claw_view" in source
    assert "_finish_with_validated_prior_claw" in source
    assert "finalize_automated_claw_with_validated_prior" in source
    assert "all five D455 jaw views were optically ambiguous" in source
