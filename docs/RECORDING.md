# Host-side demonstration recording

The browser's **Controller → Demonstration recording** controls an episode
recorder on the robot host. It subscribes to the already-running capture service
and the existing follower feedback listener. Starting, stopping or discarding a
recording sends no motor commands and does not enable the arm.

The implementation follows RoboCade's capture-episodes-then-export workflow.
Capture uses the normal lightweight teleoperation environment. Export uses the
official LeRobot dataset writer in a separate optional environment.

## Record an episode

1. Start the normal supervised stack with the SO-101 module. Confirm live original
   RGB and fresh measured robot feedback. The recorder needs an applied position
   target as well as measured positions; the module supplies their definitions.
2. The study task is preconfigured as **Place the block in the cup** (placeholder).
   A facilitator can override it in **Settings → Session**. Select **Start trial**;
   preparation checks the RGB streams before creating an episode. The status must
   say **Recording** and its host frame count must advance. The preset carries
   across all three conditions; participants do not retype it.
3. Perform the task with the existing controls. Select **Success**, **Failure**, or
   leave **Not rated**, then select **Finish trial**. Wait for the saved status,
   then select **Next trial** explicitly. Outside guided trials, the recording
   controls in Settings offer **Start recording** and **Stop and save**.
4. **Discard** marks the episode as discarded. Its files remain on the host and
   are excluded from export. A new episode always starts with an unrated outcome.

The default rate is 15 Hz. All RGB cameras reported by the capture service are
selected, together with any already-open auxiliary wrist stream. Open the wrist
view before recording if it should be included. The recorder never starts a
second wrist-camera process. The API also accepts an explicit camera-ID list
and a rate of 10, 15, 20 or 30 Hz.

**Pause** holds the arm through its robot module, freezes the trial timer and
elapsed-time bar, and blocks arm commands until **Resume**. Raw RGB, measured
state and applied targets continue recording. Resume restores control only if
the arm was armed before Pause, using the module's normal readiness checks; it
does not enable a previously unarmed robot. Finish and Discard work while paused
and never resume the arm. Reconnecting or reloading the page does not re-enable
the arm. Physical Hold/Resume behavior still needs supervised hardware validation.

Recording uses full original sensor RGB JPEGs, independent of browser viewpoint,
workspace crop, modeled table and point-cloud mode. It preserves the incoming
JPEG bytes; the capture service encodes original RGB at quality 95 or the higher
quality another active RGB consumer requests. RGB subscribers share the highest
requested image width and quality. No depth or rendered point-cloud view is
required for these training observations.

Changing a camera profile or restarting the robot service during an episode
ends that capture as incomplete. Keep camera selection and capture profiles
consistent across demonstrations that will form one dataset.

## What is saved

Each episode lives in an ignored host folder:

```text
.teleop/recordings/<UTC-time>_<unique-id>/
  episode.json
  frames.jsonl
  episode.npz
  camera_1/0000000.jpg
  camera_2/0000000.jpg
  ...
```

The manifest includes the task, outcome, frame rate, camera identities/sizes,
robot feature names, units and coordinate conventions. SO-101 state means
measured follower positions. Actions are the follower's applied absolute joint
position targets, including held targets. These are Robot Teleop's internal
normalized coordinates: degrees for the first five joints and percent for the
gripper. They must not be treated as an unadapted stock LeRobot motor profile.

The numeric archive contains measured state, applied action, original camera
timestamps and frame counters, robot feedback receipt/measurement times, applied
target write times, command
sequence numbers and relative JPEG paths. New captures include `trial_paused`
and `trial_elapsed_seconds` per sample, plus pause intervals and
`active_duration_seconds` in the manifest. Capture timestamps keep real time;
trial time excludes pauses. It loads with `allow_pickle=False`.
The incremental JSONL journal and individual JPEGs preserve evidence even if the
process ends before finalization.

Camera capture Unix timestamps are mapped to the host monotonic clock. Each
sample uses nearby measured feedback, with at most 100 ms of camera-to-camera
or camera-to-robot skew. Auxiliary wrist timestamps identify host JPEG receipt,
not a hardware exposure timestamp. Repeated source frames remain identifiable
by their sequence numbers. Inputs older than 350 ms, invalid feedback, calibration
sweeps, rest-return motion, profile changes and slow disk/capture stop the episode
instead of silently filling it with fabricated samples. Browser takeover and
host shutdown preserve an interrupted capture as incomplete. Limits are ten
minutes of wall-clock capture (including pauses), 4 GiB of JPEGs per episode and
at least 512 MiB free disk space. The HUD's visual time target is separate from
these capture bounds.

Only **complete** episodes are eligible for export. A process crash leaves an
unfinalized episode, which is excluded. Discarded and incomplete captures are
retained for recovery; there is no automatic deletion or upload.

## Export with LeRobot

Use a separate environment so training dependencies do not alter the live
camera/control environment. The supported writer is pinned to LeRobot 0.4.3
and produces its v3 dataset format.

```bash
python -m venv .teleop/lerobot-export
# Linux:
source .teleop/lerobot-export/bin/activate
# Windows PowerShell instead:
# .\.teleop\lerobot-export\Scripts\Activate.ps1
python -m pip install -e ".[recording]"
python -m robot_teleop.recording_export \
  --source .teleop/recordings \
  --output .teleop/datasets/pick-place-01 \
  --repo-id local/pick-place-01
```

The default export includes completed **successes**. To inspect all completed
captures, including failures and unrated episodes, explicitly add `--outcome all`.
Their outcome annotation remains 1, 0 or -1 respectively. Incomplete/discarded
episodes are always excluded. An existing output directory is never overwritten.
Paused frames are retained with `annotation.trial_paused` equal to 1 (otherwise
0); older captures without pause data export as 0. A training pipeline can use
that annotation to choose whether to include paused intervals.

Export resamples captured observations to a uniform frame grid, choosing the
nearest source sample within 100 ms and retaining its original timestamps.
All episodes must share camera identities, resolution, recording rate and robot
schema. LeRobot image features retain captured resolution and losslessly store
the decoded RGB; resize observations in a training configuration if required.

The result loads through `LeRobotDataset` and a normal training `DataLoader`.
`capture_export.json` maps exported episodes back to source episodes and outcomes.
Nothing is pushed to Hugging Face or another service. Training and autonomous
policy execution are separate operations; recording never starts either.

## Validation without hardware

`tests/test_recording.py` checks capture integrity, incomplete/discard recovery,
owner/authentication boundaries and the real HTTP/UDP/multipart path using
synthetic camera and follower services. Set `LEROBOT_TEST_PYTHON` to the isolated
export environment's Python to include the official writer/readback/DataLoader
test.

`tests/episode_recording_browser.cjs` exercises the real controller controls on
desktop and mobile with that synthetic host. A manual loopback-only rehearsal
is available with:

```bash
python tests/serve_recording_fixture.py
```

Open `http://127.0.0.1:18768/controller.html`. Every frame and robot reading in
that fixture is synthetic; it opens no hardware. Synthetic validation does not
establish actual sensor latency, alignment or physical task performance.

Deploy the updated Python gateway, Godot stream script and browser files together.
Restart the gateway to load the recorder. Reload the Godot runtime for the RGB
request aggregation and original-size headers. The recorder checks those headers
and waits for a new native-size frame before starting, so an older capture service
cannot silently record a downscaled browser preview. Do not replace Chen's local motor
profile or camera/arm calibrations when updating the code.
