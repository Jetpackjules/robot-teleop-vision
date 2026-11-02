# Robot Teleop Vision

<!-- impeccable:product-schema 1 -->

## Platform

web

## Users

The immediate audience is participants in supervised robot teleoperation studies, using the research team's laptops or larger screens. Participants perform physical manipulation tasks through the browser.

Facilitators configure the installation, prepare and calibrate the workspace, manage recordings, and advance trials. General remote operators remain a supported audience; simplifying the participant experience must preserve their existing controls and configuration options.

## Product Purpose

The current priority is to conduct supervised trials comparing fixed RGB cameras, mouse-controlled 3D viewing, and head-tracked 3D viewing. Participants need to understand their task and current condition while concentrating on the robot workspace. Facilitators need a consistent trial sequence and identifiable recordings for later review and training-data export.

General remote robot operation remains part of the product. The study workflow builds on the working teleoperation stack rather than replacing it.

## Positioning

The browser combines live RealSense RGB-D reconstruction, navigable 3D viewing, optional head tracking, and a separately rendered robot-model overlay. Robot integrations supply their own hardware controls, geometry, calibration, and feedback through a shared module boundary. SO-101 is the included reference integration.

These mechanisms describe the implementation. Improved task performance, presence, graphical fidelity, bandwidth, and frame rate require appropriate measurements; the study has not established those benefits merely because the interface or a synthetic rehearsal works.

## Operating Context

- Testing is supervised and takes place on team devices. Laptop and larger screens are the current scope; phone layouts are outside it.
- The browser is the operational interface. Host-side tools and Godot support installation, capture, rendering, calibration, diagnostics, and service management.
- The installation has one active browser operator. Stream connection, robot readiness or enablement, head tracking, and recording are distinct states.
- Facilitators configure the system through Settings on the same page. A separate setup screen or mandatory setup wizard is unnecessary for this workflow.
- Camera alignment, motor calibration, and visual registration of the full robot model are separate operations. An attractive model overlay does not establish physical alignment.
- Recordings are written on the robot host. Browser viewpoint, table-region filtering, and artificial scene geometry do not replace the original RGB observations saved for training.

## Capabilities and Constraints

### Trial workflow

- Show one active view at a time, filling the available viewport.
- Keep the task, condition, trial progress, timer, recording state, and appropriate next action clear.
- The compact top trial bar shows the task, time and current action; during recording the action is Done. Configure the session name in Settings. Setup connects the physical leader arm if needed, waits for fresh readings from all six encoders and follower readiness, and enables leader control without recording. The next action is Start first trial. A held follower can be enabled again by the next explicit Start trial action.
- Append a Europe/Berlin date, time, and UTC offset to the chosen session name at setup; persist the full name in every trial's host metadata. Keep the internal unique session identifier separate. Refresh preserves the session name, order, and progress. After the last trial is saved and its required block count is entered, the top bar offers Restart session; restarting creates a new identifier, timestamp, and shuffled plan without deleting recordings.
- Keep the task first. Omit routine explanatory sentences from the trial HUD; show actionable warnings and errors only when needed. Avoid decorative subheadings and repeated idle-state text.
- Task 1 is unscored free exploration, always first: one 60-second practice recording per view, with prompts to engage camera switching, mouse navigation and head movement. Tasks 2 and 3 retain two separately shuffled rounds of all three views (12 measured trials); their task order remains randomized. Practice is explicitly labeled in recording metadata. Restored plans and historical recordings retain their original instructions, scoring, times and progress until New session.
- Measured tasks default to a common 75-second active-time limit; practice has a separate 60-second default. Instructions, measured scoring and both time limits are editable in Session settings and saved on the host across browser and host restarts. Done or timeout saves a measured episode before showing a small result prompt; practice saves without pass/fail or a result prompt. A counting task accepts an integer including zero. Pass/fail defaults to pass for an early finish and fail for timeout; the facilitator can review the result. Save & Next stores that result before advancing and never starts the next recording automatically.
- Trial time limits run on the host, exclude pauses, stop/save the recording, and hold the arm. The progress bar and counter reflect the full configured plan. Saved recordings remain intact when starting a new session.
- Fixed-camera trials support selecting individual RGB cameras using visible previous/next controls and left/right arrow keys.
- Mouse 3D trials use the existing 3D scene with head tracking off. Head-tracked trials use that scene with head tracking on and an accessible reset action. Keep the selected 3D renderer consistent between these two conditions.
- Facilitators can configure task instructions, current-activity time limits, activity order, and per-activity view order in Settings → Session.
- The facilitator explicitly clicks Next after the host has finished saving a complete episode. Do not automatically advance conditions. Discarded or incomplete recordings do not qualify as a completed trial.
- Pause/Resume remains available in Settings recording controls during a trial. Pause holds arm control and freezes task time while original RGB and robot data continue recording with pause annotations. Resume is explicit and only restores previously armed control after module readiness checks. Done while paused saves without resuming the arm.
- Preserve free configuration and recording outside guided trials.

### Controls and configuration

- Auto-return to the saved rest pose is enabled with a five-minute inactivity timer. Meaningful arm or gripper control resets the timer; zero-motion keyboard heartbeats do not. Trial Pause suspends auto-return until Resume or Finish. The timer uses the existing host rest-return route and does not release torque.

- Preserve access to the existing robot controls, recording controls, stream and renderer choices, calibration, workspace tools, installation defaults, and diagnostics.
- Group configuration into the same-page Session, Arm, View, and Diagnostics settings tabs.
- Hold Arm belongs in Settings → Arm alongside the other arm controls.
- Show arm notifications sits at the top of Arm settings. It toggles safeguard and connection pop-ups, clears visible notices immediately when disabled, and remembers the preference in this browser. Muting notices leaves arm safeguards and control behavior unchanged; re-enabling shows only new notices.
- Keep the main connection indicator compact. Its disclosure provides technical stream, tracking, controller, and follower information on demand.
- Remove the persistent head-tracking status and XYZ debug text from the participant viewport.
- Stream quality and rendering-method configuration belong with facilitator settings.

### Scene and data integrity

- Preserve the existing RGB-D rendering options, including BlendPCR, and the original RGB camera view.
- The artificial table and floor, scene lighting, and robot overlay remain reversible options. Preserve manual placement and calibration controls.
- The robot overlay is a separate model driven by robot data. Modeled surroundings must not be represented as reconstructed sensor observations.
- Host recording preserves original RGB, measured joint state, applied actions, timing, task, outcome, and trial context. Recording itself does not enable or move the arm.
- Incomplete and discarded captures remain recoverable on the host and are excluded from complete-episode export. Export uses the official LeRobot writer through the optional recording environment.
- Physical robot commands and safeguards remain owned by robot integrations. Keep robot-specific behavior outside the shared application.
- Protect local credentials, motor profiles, calibration files, recordings, and physical-device identities from accidental publication. Preserve a reviewable rollback path for substantial changes.

## Brand Commitments

The existing project name is Robot Teleop Vision. Its existing icon is `assets/branding/robot_teleop_vision_icon.png`.

The user prefers a restrained, functional interface with clear controls and strongly rejects decorative game-like or obviously generated button styling. They approved the single-view layout and the timer/trial information at the top center. RoboCade's trial/timer arrangement is a reference for that organization.

These are supplied constraints, not authorization to invent a new visual identity.

## Evidence on Hand

- `web/controller.html`, `web/controller_ui.css`, and `web/trial_controller.js`: the current supervised-study interface and condition workflow.
- `web/hybrid_rgbd_renderer.js` and `godot/runtime/point_cloud/`: the browser and Godot rendering implementations.
- `robot_modules/so101/`: the reference integration, model assets, controls, and calibration tools. Asset licenses remain applicable.
- `robot_teleop/recording.py`, `robot_teleop/recording_export.py`, and `docs/RECORDING.md`: host capture and export behavior. Current source governs when older documentation uses previous UI labels.
- `tests/serve_recording_fixture.py` and `tests/tester_ui_browser.cjs`: camera-free workflow and layout rehearsals using synthetic inputs and actual SO-101 assets. Synthetic head-tracking signals exercise UI state changes, not tracking accuracy.
- `docs/ARCHITECTURE.md`, `docs/ROBOT_MODULES.md`, `docs/CALIBRATION.md`, and `docs/SAFETY.md`: implementation boundaries and operating requirements.

Synthetic previews and passing software tests do not establish live camera quality, calibrated alignment, motion behavior, or study results.

## Product Principles

1. Let participants concentrate on the workspace and their task; expose configuration when the facilitator needs it.
2. Simplify access without losing working capabilities or introducing a separate setup journey.
3. Make trial transitions and host save state explicit, with facilitator-controlled advancement.
4. Preserve the distinction between sensor observations, modeled geometry, measured feedback, and commanded actions.
5. Keep experiments reversible and distinguish implemented, tested, deployed, and physically validated behavior.

## Accessibility & Inclusion

Preserve keyboard access, visible focus, meaningful control labels, and clear status text. Camera arrow shortcuts must not interfere with editing fields. Mouse navigation remains available in the condition designed to operate without head tracking. Status and recording information must be understandable without relying on color alone.

## Open Decisions

- The final task set, participant population, counterbalancing beyond the configured random order, evaluation measures, and study success criteria.
- The final responsibility for assigning success/failure outcomes. The current supervised implementation makes those controls available to the facilitator.
- Any future self-service onboarding or mobile support. Neither is a current requirement.
