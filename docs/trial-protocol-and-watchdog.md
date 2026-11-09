# Trial HUD, results and durable task settings

The participant HUD contains the task, view/trial position, elapsed time and
the current action. During a trial that action is **Done**. Session names are
configured in Session settings and retained in recording metadata; they do not
occupy a HUD row. Pause/Resume remains in the recording controls in Settings.

The host ends each trial at its active-time limit, excluding pauses. New
sessions start with Task 1, free exploration for 60 seconds per view condition,
one round of all three modes. Practice prompts ask participants to switch RGB
cameras, orbit/pan/zoom with the mouse, or move their head to look around objects,
while trying arm control. There is no object-placement goal or pass/fail score.
Task 1 always precedes the measured tasks, even with randomized activity order.
Tasks 2 (count in cup) and 3 (Balance test) each retain two rounds of all three
view conditions with the shared 75-second default: 3 practice + 12 measured
recordings. Their activity order and each round's mode order remain randomized.
Existing restored plans retain their recorded instructions, times and progress
until a new session is created.

The participant's arm-control session is independent of recording and viewing
mode. Done and recording deadlines stop recording only; they do not send Hold,
Enable, or replace the leader reference. Trial starts verify fresh, already-armed
control rather than implicitly enabling a held robot. The facilitator uses Hold
before resetting the physical workspace. Explicit Pause/Hold, disconnects, stale
control input and hardware safeguards retain their stopping behavior. A held
leader must match its retained target before an explicit Enable can resume it.

After Done or timeout, the host first finishes saving. The participant then
sees a small result prompt for a measured trial. Practice instead offers Next
without a score prompt or automatic recording start. A count task accepts an integer including zero;
a pass/fail task presents the result for review. Finish before the limit
defaults to pass and timeout defaults to fail. Incomplete/discarded recordings
are never automatically passed. The facilitator can correct the result in
the prompt. **Save & Next** saves the result before advancing, without starting
the next recording. The last trial uses **Save & Finish**. Closing the prompt
leaves Review result available in the HUD.

Session settings expose editable instructions, measured scoring mode, a common
measured time limit, and a separate practice duration in seconds. Task 1 is
always unscored. **Save task settings on host** writes the validated protocol
to `.teleop/trial-protocol.json`, separately from calibration and camera state.
Both browser reloads and gateway/host restarts load that file. The API requires
the existing authenticated session and same-origin JSON writes. A refreshed
active controller can review the saved result only for its matching study
session.

Protocol version 2 migrates legacy host configuration for future sessions only.
Every new episode has `trial.phase` (`practice` or `measured`); practice also
uses `success_metric: practice`, `outcome_rule: manual`, `result_prompt: false`
and outcome `unrated`. Filter `trial.phase == practice` out of outcome analyses.
The original camera/action recording pipeline and recording rate are unchanged.
Legacy plans and recordings without a phase keep their original semantics;
reloading never converts today's scored Task 1 recordings into practice.

## Watchdog recovery

The default command watchdog and browser input age thresholds are 500 ms. An explicit `watchdog_ms` in the arm-pair profile overrides the follower default. The original recovery code treated
keyboard timeouts specially but left leader input in a latched Hold. The
leader path now also freezes and keeps writing the last applied target when
commands expire. Fresh input from the same control session resumes leader
control, re-clutching the leader to that retained target. Movement made during
the gap is not replayed. Explicit Hold, controller replacement, stale packets,
torque release and hardware faults do not automatically enable motion.

Leader timing diagnostics include command gaps, serial-read duration, encoder
sample age, queued socket bytes and foreground state. Follower status retains
the most recent watchdog event and its input/transport timings. These separate
the confirmed recovery defect from the still-unattributed source of occasional
command gaps.

Validation used synthetic browser/recording fixtures and FakeFollowerBus,
including timeout, sag, stale input, manual Hold and controller-handoff cases.
The deployed process was checked for connected hardware and holding torque;
no movement command was sent as part of verification. Calibration hashes,
capture process and existing recording metadata were preserved. Detailed
activation and browser evidence are in
`.teleop/compact-trials-watchdog-20261101/`.
