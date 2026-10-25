// Served only by the loopback rehearsal. Never imported by the application.
import '/godot_webcam_tracker_bridge.js?v=20261025_trials';
window.trialFixture = {starts: 0, resets: 0};
window.startGodotWebcamTracker = async () => {
  window.trialFixture.starts++;
  window.godotWebcamTrackerLatest = {active: true, status: 'synthetic tracking', x: 0, y: 0, z: 0};
  return true;
};
window.stopGodotWebcamTracker = () => { window.godotWebcamTrackerLatest.active = false; };
window.recenterGodotWebcamTracker = () => { window.trialFixture.resets++; };
