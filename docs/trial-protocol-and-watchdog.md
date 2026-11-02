# Trial HUD, results and durable task settings

The participant HUD contains the task, view/trial position, elapsed time and
the current action. During a trial that action is **Done**. Session names are
configured in Session settings and retained in recording metadata; they do not
occupy a HUD row. Pause/Resume remains in the recording controls in Settings.

The host ends each trial at its active-time limit, excluding pauses. New
sessions default to 75 seconds and three activities: pick/place, count in cup,
and Balance test. Every activity has two rounds of all three view conditions.
Existing restored plans retain their recorded instructions, times and progress
until a new session is created.

After Done or timeout, the host first finishes saving. The participant then
sees a small result prompt. A count task accepts an integer including zero;
a pass/fail task presents the result for review. Finish before the limit
defaults to pass and timeout defaults to fail. Incomplete/discarded recordings
are never automatically passed. The facilitator can correct the result in
the prompt. **Save & Next** saves the result before advancing, without starting
the next recording. The last trial uses **Save & Finish**. Closing the prompt
leaves Review result available in the HUD.

Session settings expose editable instructions, scoring mode and a common time
limit in seconds. **Save task settings on host** writes the validated protocol
to `.teleop/trial-protocol.json`, separately from calibration and camera state.
Both browser reloads and gateway/host restarts load that file. The API requires
the existing authenticated session and same-origin JSON writes. A refreshed
active controller can review the saved result only for its matching study
session.

## Watchdog recovery

The freshness threshold remains 250 ms. The original recovery code treated
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
