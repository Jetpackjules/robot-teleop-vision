const MODEL_ROOT = "/robot-modules/so101/assets";
const NOTIFICATIONS_STORAGE_KEY = "robotTeleopArmNotificationsEnabled";

const RENDERER = {
  modelUrls: {
    base_link: `${MODEL_ROOT}/base_link.glb`,
    shoulder_link: `${MODEL_ROOT}/shoulder_link.glb`,
    upper_arm_link: `${MODEL_ROOT}/upper_arm_link.glb`,
    lower_arm_link: `${MODEL_ROOT}/lower_arm_link.glb`,
    wrist_link: `${MODEL_ROOT}/wrist_link.glb`,
    gripper_link: `${MODEL_ROOT}/gripper_link.glb`,
    moving_jaw_so101_v1_link: `${MODEL_ROOT}/moving_jaw_so101_v1_link.glb`,
  },
  colors: {
    base_link: [0.91, 0.76, 0.18, 1],
    shoulder_link: [0.78, 0.64, 0.12, 1],
    upper_arm_link: [0.95, 0.81, 0.24, 1],
    lower_arm_link: [0.78, 0.64, 0.12, 1],
    wrist_link: [0.95, 0.81, 0.24, 1],
    gripper_link: [0.78, 0.64, 0.12, 1],
    moving_jaw_so101_v1_link: [0.95, 0.81, 0.24, 1],
  },
  baseLink: "base_link",
  materialPalette: [
    {sourceColor:[1,209/255,31/255], color:[.19,.20,.215,1], roughness:.78, metalness:0, detail:1},
    {sourceColor:[28/255,31/255,36/255], color:[.105,.12,.14,1], roughness:.34, metalness:.03, detail:.2},
  ],
  accentLinks: ["moving_jaw_so101_v1_link"],
  calibrationLinks: ["wrist_link", "gripper_link", "moving_jaw_so101_v1_link"],
  calibrationBoundsLinks: ["gripper_link", "moving_jaw_so101_v1_link"],
  endEffectorLinks: ["gripper_link", "moving_jaw_so101_v1_link"],
  maskChain: {
    names: ["base_link", "shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link", "moving_jaw_so101_v1_link"],
    radii: [0.065, 0.060, 0.050, 0.047, 0.043, 0.040],
  },
  manualView: {
    rgbCanvas: "manual-rgb-canvas",
    cropCanvas: "manual-crop-canvas",
  },
  // Visualization only. The hardware service remains authoritative for joint
  // limits, path clearance, and contact handling.
};

const MODULE_STYLE = `
  #so101-wrist-camera-panel {
    position: fixed; right: 16px; bottom: 16px; z-index: 2;
    width: min(30vw, 420px); overflow: hidden;
    border: 1px solid var(--line); border-radius: 10px;
    background: #050607; box-shadow: 0 18px 50px rgba(0,0,0,.48);
  }
  #so101-wrist-camera-view { display:block; width:100%; max-height:32vh; object-fit:contain; }
  #so101-wrist-camera-status { padding:12px; color:var(--muted); font:12px/1.4 ui-monospace,monospace; }
  #so101-manual-workspace {
    position:fixed; right:16px; bottom:16px; z-index:5;
    width:min(440px,calc(100vw - 32px)); max-height:calc(100vh - 32px);
    overflow:auto; padding:12px; border:1px solid var(--line); border-radius:10px;
    background:rgba(8,10,12,.94); box-shadow:0 22px 70px rgba(0,0,0,.58); backdrop-filter:blur(14px);
  }
  .so101-manual-preview-grid { display:grid; grid-template-columns:minmax(0,1fr); gap:10px; }
  .so101-manual-preview-grid figure { margin:0; }
  .so101-manual-preview-grid figcaption { margin-bottom:5px; color:var(--muted); font-size:12px; }
  .so101-manual-preview-grid canvas { display:block; width:100%; max-height:43vh; object-fit:contain; border:1px solid var(--line); border-radius:6px; background:#111; }
  .so101-manual-controls { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:9px; margin-top:10px; }
  .so101-joint-row { display:grid; grid-template-columns:1fr 84px; align-items:center; gap:6px 12px; margin:12px 0; }
  .so101-joint-row label { margin:0; }
  .so101-joint-row input[type=range] { grid-column:1 / -1; width:100%; height:22px; margin:0; padding:0; }
  .so101-joint-value { display:flex; align-items:center; gap:4px; }
  .so101-joint-value input[type=number] { width:72px; min-width:0; height:34px; padding:6px; border:1px solid var(--line); border-radius:5px; background:#171b1f; color:var(--text); font:inherit; }
  #so101-import-dialog { max-width:min(430px,calc(100vw - 48px)); color:var(--text); background:#15191d; border:1px solid var(--line); border-radius:10px; padding:20px; }
  #so101-import-dialog::backdrop { background:rgba(0,0,0,.6); }
  .so101-manual-actions { display:grid; grid-template-columns:repeat(3,1fr); gap:8px; margin-top:10px; }
  .so101-manual-note { margin:8px 0 0; color:var(--muted); font-size:12px; }
  @media (max-width:760px) { #so101-wrist-camera-panel { width:min(46vw,320px); } }
`;

const SETTINGS_MARKUP = `
  <label class="toggle-row" title="Show safeguard warnings and input/connection notices."><span>Show arm notifications</span><input id="so101-notifications-enabled" type="checkbox" checked></label>
  <label class="toggle-row"><span>Wrist camera</span><input id="so101-wrist-camera-enabled" type="checkbox"></label>
  <div class="button-row">
    <button id="so101-connect" type="button">Connect Leader</button>
    <button id="so101-keyboard" type="button" title="WASD translates the claw, R/F raises or lowers it, I/K pitches and J/L rolls the wrist, Q/E opens or closes it, and Shift enables precision motion.">Keyboard Control</button>
  </div>
  <button id="so101-enable" class="primary" type="button" disabled>Enable Arm</button>
  <button id="so101-hold" class="hold" type="button">Hold Arm</button>
  <button id="so101-return-rest" type="button">Return Arm to Rest Pose</button>
  <p id="so101-rest-status" class="helper-text" role="status"></p>
`;

const SETUP_MARKUP = `
  <button id="so101-save-rest" type="button">Save Current Pose as Rest</button>
  <p class="helper-text">Place the arm in your chosen rest pose, press Hold, then save. This records the physical pose without moving the arm.</p>
  <p id="so101-setup-warnings" class="helper-text" role="status"></p>
  <label class="toggle-row" title="Returns to the saved rest pose after five minutes without arm-control input."><span>Auto-return to rest after 5 min</span><input id="so101-idle-return-enabled" type="checkbox" checked></label>
  <button id="so101-restart" type="button">Reconnect Arm Hardware</button>
  <details class="advanced-disclosure">
    <summary>Experimental safeguards</summary>
    <div class="disclosure-body">
      <label class="toggle-row"><span>Stop on following error / contact</span><input id="so101-following-error-safety-enabled" type="checkbox"></label>
      <p class="helper-text">This modeled safeguard can false-trigger on gravity sag or backlash. The hardware watchdog and joint limits remain authoritative.</p>
    </div>
  </details>
  <div id="so101-health" class="network-stats">Follower status unavailable</div>
`;

const VIEW_MARKUP = `
  <label class="toggle-row"><span>Mirror view left/right</span><input id="so101-mirror-view" type="checkbox"></label>
  <p class="helper-text">Mirrors RGB, point clouds and the arm overlay together. Physical arm controls stay unchanged. Remembered in this browser.</p>
  <label class="toggle-row"><span>Robot overlay</span><input id="so101-overlay-enabled" type="checkbox" checked></label>
  <label>Show
    <select id="so101-overlay-scope"><option value="full">Full arm</option><option value="claw">Claw only</option></select>
  </label>
  <label>Robot overlay style
    <select id="so101-overlay-style"><option value="realistic" selected>Realistic materials</option><option value="alignment">Alignment (transparent + outlines)</option><option value="solid">Solid yellow</option></select>
  </label>
  <label class="toggle-row"><span>Overlay occludes scanned arm</span><input id="so101-overlay-occlusion-enabled" type="checkbox" checked></label>
  <label>Arm occlusion buffer <output id="so101-mask-buffer-value">0 mm</output><input id="so101-mask-buffer" type="range" min="0" max="50" step="1" value="0"></label>
  <p class="helper-text">Extra clearance around the base and arm to hide wires and scan edges. Claw masking stays unchanged. Save Current Setup as Default to share this setting.</p>
  <label>Claw opacity <output id="so101-claw-opacity-value">100%</output><input id="so101-claw-opacity" type="range" min="0" max="1" step="0.05" value="1"></label>
  <label>Claw occlusion buffer <output id="so101-claw-buffer-value">0 mm</output><input id="so101-claw-buffer" type="range" min="0" max="50" step="1" value="0"></label>
  <p class="helper-text">Claw opacity lets you see through the model. Extra claw clearance hides nearby scan points too; keep it small near objects.</p>
  <label>Rest of arm opacity <output id="so101-body-opacity-value">100%</output><input id="so101-body-opacity" type="range" min="0" max="1" step="0.05" value="1"></label>
  <p class="helper-text">Independent of claw opacity. Choose Full arm to show the rest as a ghost.</p>
  <label class="toggle-row"><span>Show commanded target ghost</span><input id="so101-target-ghost-enabled" type="checkbox" checked></label>
  <button id="so101-calibrate" class="primary" type="button">Align Arm</button>
  <p class="helper-text">Uses a measured base sweep to align the whole arm. Existing joint settings and claw geometry stay fixed.</p>
  <button id="so101-claw-adjust" type="button">Adjust Claw Translation</button>
  <button id="so101-base-adjust" type="button">Adjust Base in 3D</button>
  <p id="so101-base-status" class="helper-text" role="status"></p>
  <div class="calibration-progress"><progress id="so101-calibration-progress" max="1" value="0"></progress><span id="so101-calibration-progress-value" class="value-label">0%</span></div>
  <div id="so101-calibration-status" class="network-stats">Arm position calibration idle</div>
  <details class="advanced-disclosure">
    <summary>Recovery tools</summary>
    <div class="disclosure-body">
      <button id="so101-manual-open" type="button">Manual Joint Tuning</button>
      <div class="button-row"><button id="so101-calibration-export" type="button">Export JSON</button><button id="so101-calibration-import" type="button">Import JSON</button></div>
      <input id="so101-calibration-file" type="file" accept=".json,application/json" hidden>
      <p id="so101-calibration-file-status" class="helper-text" role="status"></p>
      <label class="toggle-row"><span>D455 wrist / claw visual correction</span><input id="so101-d455-visual-correction-enabled" type="checkbox"></label>
      <p class="helper-text">Tune remaining joint mismatches after the base is aligned. Export a saved calibration to reuse it with this arm and motor profile.</p>
    </div>
  </details>
`;

const STATUS_MARKUP = `
  <div id="keyboard-controls-guide" class="keyboard-guide" hidden>
    <strong>SO-101 keyboard control</strong><br>
    <kbd>W / S</kbd> forward / back<br><kbd>A / D</kbd> left / right<br>
    <kbd>R / F</kbd> up / down<br><kbd>Q / E</kbd> open / close claw<br>
    <kbd>I / K</kbd> wrist up / down<br><kbd>J / L</kbd> wrist rotate left / right<br>
    <kbd>Shift</kbd> precision speed
  </div>
`;

const JOINT_TUNING_FIELDS = [
  ["manual_shoulder_lift_trim_degrees", "Shoulder lift (J2)", -180, 180],
  ["manual_elbow_flex_trim_degrees", "Elbow bend (J3)", -180, 180],
  ["manual_wrist_flex_trim_degrees", "Wrist bend (J4)", -90, 90],
  ["manual_wrist_roll_trim_degrees", "Wrist rotation (J5)", -180, 180],
];
const JOINT_TUNING_MARKUP = JOINT_TUNING_FIELDS.map(([key, label, min, max]) => `
  <div class="so101-joint-row">
    <label for="so101-number-${key}">${label}</label>
    <div class="so101-joint-value"><input id="so101-number-${key}" data-manual-number="${key}" type="number" min="${min}" max="${max}" step="0.1" value="0" aria-label="${label} offset in degrees"><span aria-hidden="true">°</span></div>
    <input data-manual-key="${key}" type="range" min="${min}" max="${max}" step="0.1" value="0" aria-label="${label} offset slider">
  </div>`).join("");

const FLOATING_MARKUP = `
  <div id="so101-wrist-camera-panel" hidden><img id="so101-wrist-camera-view" alt="SO-101 wrist camera" hidden><div id="so101-wrist-camera-status">Connecting to wrist camera…</div></div>
  <section id="so101-manual-workspace" hidden aria-label="Manual joint tuning">
    <strong>Manual joint tuning</strong>
    <p class="so101-manual-note">Match the overlay to the real arm. These are adjustments to the saved angles; base placement stays fixed.</p>
    <fieldset><legend>Claw translation only</legend>
      <p class="helper-text">Local claw axes, in millimetres (0.25 cm = 2.5 mm). Moves the claw overlay only; base and joint angles stay fixed. Preview, then Save Calibration.</p>
      <div class="so101-manual-controls"><label>Claw X (mm)<input data-manual-number="manual_tool_x" data-manual-scale="1000" type="number" min="-50" max="50" step="0.1" value="0" aria-label="Claw X translation in millimetres"><input data-manual-key="manual_tool_x" type="range" min="-0.05" max="0.05" step="0.0001" value="0" aria-label="Claw X translation"></label>
<label>Claw Y (mm)<input data-manual-number="manual_tool_y" data-manual-scale="1000" type="number" min="-50" max="50" step="0.1" value="0" aria-label="Claw Y translation in millimetres"><input data-manual-key="manual_tool_y" type="range" min="-0.05" max="0.05" step="0.0001" value="0" aria-label="Claw Y translation"></label>
<label>Claw Z (mm)<input data-manual-number="manual_tool_z" data-manual-scale="1000" type="number" min="-50" max="50" step="0.1" value="0" aria-label="Claw Z translation in millimetres"><input data-manual-key="manual_tool_z" type="range" min="-0.05" max="0.05" step="0.0001" value="0" aria-label="Claw Z translation"></label>
    </div></fieldset>
    ${JOINT_TUNING_MARKUP}
    <details class="advanced-disclosure"><summary>Wrist / claw close-up</summary>
    <div class="so101-manual-preview-grid">
      <figure><figcaption>Aligned RGB + exact projected 3D geometry</figcaption><canvas id="manual-rgb-canvas"></canvas></figure>
      <figure><figcaption>Automatic claw crop</figcaption><canvas id="manual-crop-canvas"></canvas></figure>
    </div>
    </details>
    <details class="advanced-disclosure"><summary>Claw and tool adjustments</summary>
    <div class="so101-manual-controls">
      <label>Wrist direction<select id="so101-manual-wrist-direction"><option value="0" selected>Keep saved direction</option><option value="1">Normal (+)</option><option value="-1">Reversed (−)</option></select></label>
      <label>Overlay opacity <span data-manual-value="manual_overlay_opacity" class="value-label">30%</span><input data-manual-key="manual_overlay_opacity" type="range" min="0.05" max="0.9" step="0.05" value="0.3"></label>
      <label>Tool roll <span data-manual-value="manual_tool_roll" class="value-label">0.0°</span><input data-manual-key="manual_tool_roll" type="range" min="-180" max="180" step="0.5" value="0"></label>
      <label>Tool pitch <span data-manual-value="manual_tool_pitch" class="value-label">0.0°</span><input data-manual-key="manual_tool_pitch" type="range" min="-180" max="180" step="0.5" value="0"></label>
      <label>Tool yaw <span data-manual-value="manual_tool_yaw" class="value-label">0.0°</span><input data-manual-key="manual_tool_yaw" type="range" min="-180" max="180" step="0.5" value="0"></label>
      <label>Opening offset <span data-manual-value="manual_opening_offset_degrees" class="value-label">0.0°</span><input data-manual-key="manual_opening_offset_degrees" type="range" min="-35" max="35" step="0.25" value="0"></label>
      <label>Opening scale <span data-manual-value="manual_opening_scale" class="value-label">1.000×</span><input data-manual-key="manual_opening_scale" type="range" min="0.5" max="1.5" step="0.005" value="1"></label>
    </div>
    </details>
    <p class="so101-manual-note">Preview only. This does not move the arm or modify saved registration until Save is pressed.</p>
    <p id="so101-manual-status" class="helper-text" role="status">Check several arm poses before saving.</p>
    <div class="so101-manual-actions"><button id="so101-manual-reset" type="button">Reset Preview</button><button id="so101-manual-cancel" type="button">Cancel</button><button id="so101-manual-save" class="primary" type="button">Save Calibration</button></div>
  </section>
  <dialog id="so101-import-dialog" aria-labelledby="so101-import-title">
    <h3 id="so101-import-title">Import visual calibration</h3>
    <p id="so101-import-name"></p>
    <p>Use this file with the same physical arm and motor profile. By default, only joint calibration is imported and the current base is kept.</p>
    <label class="toggle-row"><span>Also restore saved base placement</span><input id="so101-import-base" type="checkbox"></label>
    <p class="helper-text">Restore the base only when camera alignment and physical placement match the saved setup.</p>
    <p id="so101-import-status" class="helper-text" role="status"></p>
    <div class="button-row"><button id="so101-import-cancel" type="button">Cancel</button><button id="so101-import-apply" class="primary" type="button">Import and Save</button></div>
  </dialog>
`;

function element(id) { return document.getElementById(id); }

class So101WebModule {
  constructor(manifest) {
    this.mirrorView = false;
    try { this.mirrorView = localStorage.getItem('robotTeleopMirrorView') === 'true'; } catch {}
    this.notificationsEnabled = true;
    try { this.notificationsEnabled = localStorage.getItem(NOTIFICATIONS_STORAGE_KEY) !== "false"; } catch { /* Storage is optional. */ }
    this.manifest = manifest;
    this.renderer = RENDERER;
    this.sendViewSettings = () => false;
    this.updateStatus = () => {};
    this.manualInputs = [];
    this.manualPending = null;
    this.manualReady = false;
    this.manualPreviewTimer = null;
    this.importFile = null;
    this.calibrationRequest = null;
    this.calibrationError = "";
    this.baseEditor = null;
    this.baseSession = null;
    this.basePreviewTimer = null;
    this.baseHeartbeat = null;
    this.baseOffsets = { offset_x: 0, offset_y: 0, offset_z: 0, yaw_degrees: 0 };
    this.previewTimer = null;
    this.previewDirty = null;
    this.previewInFlight = 0;
    this.previewPollTimer = null;
  }

  async mount({ settingsRoot, setupRoot, viewRoot, statusRoot, floatingRoot }) {
    const style = document.createElement("style");
    style.dataset.robotModule = "so101";
    style.textContent = MODULE_STYLE;
    document.head.append(style);
    if (settingsRoot) {
      settingsRoot.innerHTML = SETTINGS_MARKUP;
      element("so101-notifications-enabled").checked = this.notificationsEnabled;
    }
    if (setupRoot) setupRoot.innerHTML = SETUP_MARKUP;
    if (viewRoot) viewRoot.innerHTML = VIEW_MARKUP;
    if (statusRoot) statusRoot.innerHTML = STATUS_MARKUP;
    if (floatingRoot) floatingRoot.innerHTML = FLOATING_MARKUP;
    import('./legacy_visual_corrections.js?v=20261103').then(({mountLegacyCorrections}) =>
      mountLegacyCorrections(this, element('so101-manual-open'))).catch(error => {
      element('so101-calibration-file-status').textContent = error.message;
    });
    this.manualInputs = Array.from(document.querySelectorAll("[data-manual-key]"));
    await import("./so101_web_serial.js?v=20261101_watchdog_diagnostics");
    const {ArmToastStack} = await import("./safeguard_toasts.js?v=20261028_toggle");
    this.armNotices?.dispose();
    this.armNotices = new ArmToastStack({enabled: this.notificationsEnabled});
    await import("./wrist_camera_view.js");
  }

  extendViewSettings(payload) {
    return {
      ...payload,
      robot_overlay_enabled: element("so101-overlay-enabled")?.checked !== false,
      robot_overlay_scope: element("so101-overlay-scope")?.value || "full",
      robot_overlay_style: element("so101-overlay-style")?.value || "realistic",
      view_mirror_left_right: this.mirrorView,
      robot_overlay_claw_opacity: Number(element("so101-claw-opacity")?.value ?? 1),
      robot_overlay_claw_buffer_mm: Number(element("so101-claw-buffer")?.value ?? 0),
      robot_overlay_body_buffer_mm: Number(element("so101-mask-buffer")?.value ?? 0),
      robot_overlay_body_opacity: Number(element("so101-body-opacity")?.value ?? 1),
      robot_overlay_mask_scanned_robot: element("so101-overlay-occlusion-enabled")?.checked !== false,
      arm_measured_feedback_enabled: true,
      arm_target_ghost_enabled: element("so101-target-ghost-enabled")?.checked !== false,
      arm_following_error_safety_enabled: element("so101-following-error-safety-enabled")?.checked === true,
      arm_freeze_overlay_on_stale_enabled: true,
      arm_d455_visual_correction_enabled: element("so101-d455-visual-correction-enabled")?.checked === true,
      arm_idle_return_enabled: element("so101-idle-return-enabled")?.checked === true,
    };
  }

  restoreViewSettings(saved) {
    if (!saved) return;
    this.applyMirrorView(this.mirrorView);
    element("so101-overlay-enabled").checked = saved.robot_overlay_enabled !== false;
    element("so101-overlay-scope").value = saved.robot_overlay_scope === "claw" ? "claw" : "full";
    element("so101-overlay-style").value = ["realistic", "solid", "alignment"].includes(saved.robot_overlay_style) ? saved.robot_overlay_style : "realistic";
    element("so101-claw-opacity").value = String(Number.isFinite(saved.robot_overlay_claw_opacity) ? Math.max(0,Math.min(1,saved.robot_overlay_claw_opacity)) : 1);
    element("so101-claw-buffer").value = String(Number.isFinite(saved.robot_overlay_claw_buffer_mm) ? Math.max(0,Math.min(50,saved.robot_overlay_claw_buffer_mm)) : 0);
    element("so101-mask-buffer").value = String(Number.isFinite(saved.robot_overlay_body_buffer_mm) ? Math.max(0,Math.min(50,saved.robot_overlay_body_buffer_mm)) : 0);
    element("so101-body-opacity").value = String(Number.isFinite(saved.robot_overlay_body_opacity) ? Math.max(0,Math.min(1,saved.robot_overlay_body_opacity)) : 1);
    element("so101-overlay-occlusion-enabled").checked = (saved.robot_overlay_mask_scanned_robot ?? saved.robot_overlay_mask_scanned_arm) !== false;
    element("so101-target-ghost-enabled").checked = saved.arm_target_ghost_enabled !== false;
    element("so101-following-error-safety-enabled").checked = saved.arm_following_error_safety_enabled === true;
    element("so101-d455-visual-correction-enabled").checked = saved.arm_d455_visual_correction_enabled === true;
    element("so101-idle-return-enabled").checked = saved.arm_idle_return_enabled !== false;
    this.syncOverlayScopeControls();
  }

  applyMirrorView(enabled) {
    this.mirrorView = enabled === true;
    document.documentElement.classList.toggle('teleop-mirror-view',this.mirrorView);
    if(element('so101-mirror-view'))element('so101-mirror-view').checked=this.mirrorView;
    window.setGodotHybridViewSettings?.({view_mirror_left_right:this.mirrorView});
  }

  syncOverlayScopeControls() {
    element("so101-overlay-occlusion-enabled").disabled = false;
    element("so101-claw-buffer").disabled = !element("so101-overlay-occlusion-enabled").checked;
    element("so101-claw-buffer-value").textContent = `${Number(element("so101-claw-buffer").value)} mm`;
    element("so101-claw-opacity-value").textContent = `${Math.round(Number(element("so101-claw-opacity").value)*100)}%`;
    element("so101-mask-buffer").disabled = !element("so101-overlay-occlusion-enabled").checked;
    element("so101-mask-buffer-value").textContent = `${Number(element("so101-mask-buffer").value)} mm`;
    element("so101-body-opacity").disabled = element("so101-overlay-scope").value === "claw";
    element("so101-body-opacity-value").textContent = `${Math.round(Number(element("so101-body-opacity").value)*100)}%`;
  }

  manualPayload() {
    const payload = { manual_wrist_roll_direction: Number(element("so101-manual-wrist-direction").value) };
    for (const input of this.manualInputs) payload[input.dataset.manualKey] = Number(input.value);
    return payload;
  }

  updateManualLabels() {
    for (const input of this.manualInputs) {
      const key = input.dataset.manualKey;
      const output = document.querySelector(`[data-manual-value="${key}"]`);
      const value = Number(input.value);
      const numeric = document.querySelector(`[data-manual-number="${key}"]`);
      if (numeric && document.activeElement !== numeric) numeric.value = String(Number(input.value) * Number(numeric.dataset.manualScale || 1));
      if (!output) continue;
      output.textContent = key === "manual_overlay_opacity" ? `${Math.round(value * 100)}%`
        : key === "manual_opening_scale" ? `${value.toFixed(3)}×`
        : ["manual_tool_x", "manual_tool_y", "manual_tool_z"].includes(key) ? `${(value * 1000).toFixed(1)} mm`
        : `${value.toFixed(1)}°`;
    }
  }

  resetManualControls() {
    for (const input of this.manualInputs) {
      input.value = input.dataset.manualKey === "manual_opening_scale" ? "1"
        : input.dataset.manualKey === "manual_overlay_opacity" ? "0.3" : "0";
    }
    for (const input of document.querySelectorAll("[data-manual-number]")) input.value = "0";
    element("so101-manual-wrist-direction").value = "0";
    this.updateManualLabels();
  }

  manualMessage(message) {
    element("so101-manual-status").textContent = message;
    element("so101-calibration-file-status").textContent = message;
    element("so101-import-status").textContent = message;
  }

  calibrationAction(operation, extra = {}) {
    if (!["preview", "base_preview"].includes(operation)) {
      window.clearTimeout(this.previewTimer);
      this.previewTimer = null;
      this.previewDirty = null;
    }
    if (operation === "save" && Array.from(document.querySelectorAll("[data-manual-number]")).some(input => !input.checkValidity() || !Number.isFinite(input.valueAsNumber))) {
      return Promise.reject(new Error("Enter a valid angle in each joint field before saving."));
    }
    const request_id = globalThis.crypto?.randomUUID?.() || `cal-${Date.now()}-${Math.random().toString(36).slice(2)}`;
    const action = { operation, request_id, ...extra };
    const settings = ["save", "preview"].includes(operation) ? this.manualPayload() : {};
    if (["preview", "base_preview"].includes(operation)) {
      return window.so101ArmController.requestViewSettings({ ...settings, robot_visual_calibration_action: action });
    }
    if (this.manualPending) return Promise.reject(new Error("Wait for the previous calibration action."));
    return new Promise((resolve, reject) => {
      const timer = window.setTimeout(() => {
        if (this.manualPending?.request_id !== request_id) return;
        this.manualPending = null;
        reject(new Error("No confirmation from Godot. The action may not have completed; check the connection before retrying."));
      }, 12000);
      this.manualPending = { request_id, resolve, reject, timer };
      this.manualMessage("Waiting for Godot…");
      window.so101ArmController.requestViewSettings({ ...settings, robot_visual_calibration_action: action }).catch(error => {
        if (this.manualPending?.request_id !== request_id) return;
        window.clearTimeout(timer);
        this.manualPending = null;
        reject(error);
      });
    });
  }

  baseMessage(message) {
    element("so101-base-status").textContent = message;
    const help = this.baseEditor?.root.querySelector("#workspace-edit-help");
    if (help) help.textContent = message;
  }

  queueBasePreview() {
    this.previewBaseLocally();
    this.queuePreview("base");
  }

  previewBaseLocally() {
    if (!this.baseOrigin) return;
    const o=this.baseOrigin,d=this.baseOffsets;
    window.previewGodotRobotBase?.({origin:[o[0]+d.offset_x,o[1]+d.offset_y,o[2]+d.offset_z],yaw:d.yaw_degrees});
  }

  queuePreview(kind) {
    // Coalesce continuous input instead of waiting for it to stop. Bound the
    // transport work and keep only the newest pending values under load.
    this.previewDirty = kind;
    if (this.previewTimer || this.previewInFlight >= 3) return;
    this.previewTimer = window.setTimeout(() => this.flushPreview(), 33);
  }

  flushPreview() {
    this.previewTimer = null;
    const kind = this.previewDirty;
    if (!kind || this.manualPending || (kind === "base" ? !this.baseSession || this.baseEditor?.finishing : !this.manualReady)) return;
    this.previewDirty = null;
    this.previewInFlight += 1;
    const action = kind === "base" ? this.calibrationAction("base_preview", {session_id:this.baseSession,...this.baseOffsets}) : this.calibrationAction("preview");
    action.catch(error => kind === "base" ? this.baseMessage(error.message) : this.manualMessage(error.message)).finally(() => {
      this.previewInFlight -= 1;
      if (this.previewDirty) this.queuePreview(this.previewDirty);
    });
  }

  startPreviewPolling() {
    if (this.previewPollTimer) return;
    this.previewPollTimer = window.setInterval(() => {
      if (this.manualReady || this.baseSession) window.so101ArmController?.send?.({type:"arm_status_request"});
    }, 80);
  }

  async openBaseEditor() {
    if (this.baseEditor || this.manualPending) return;
    if (typeof window.editGodotPlanarPose !== "function") {
      this.baseMessage("Open a connected 3D view before adjusting the base.");
      return;
    }
    try {
      const reply = await this.calibrationAction("base_begin");
      this.baseSession = reply.session_id;
      const origin = reply.origin;
      this.baseOrigin = origin;
      this.baseOffsets = { offset_x: 0, offset_y: 0, offset_z: 0, yaw_degrees: 0 };
      const editor = window.editGodotPlanarPose({title: "Adjust arm base", saveLabel: "Save Base", verticalTranslation:true,
        getPose: () => ({ origin: [origin[0] + this.baseOffsets.offset_x, origin[1] + this.baseOffsets.offset_y, origin[2] + this.baseOffsets.offset_z], yaw: this.baseOffsets.yaw_degrees }),
        onChange: pose => {
          this.baseOffsets = { offset_x: Math.max(-2, Math.min(2, pose.origin[0] - origin[0])), offset_y: Math.max(-2, Math.min(2, pose.origin[1] - origin[1])), offset_z: Math.max(-2, Math.min(2, pose.origin[2] - origin[2])), yaw_degrees: pose.yaw };
          this.queueBasePreview();
        },
        readout: () => `X ${(this.baseOffsets.offset_x * 1000).toFixed(0)} mm · Height ${(this.baseOffsets.offset_y * 1000).toFixed(0)} mm · Z ${(this.baseOffsets.offset_z * 1000).toFixed(0)} mm · Yaw ${this.baseOffsets.yaw_degrees.toFixed(1)}°`,
        onReset: async () => {
          window.clearTimeout(this.basePreviewTimer);
          await this.calibrationAction("base_reset", { session_id: this.baseSession });
          this.baseOffsets = { offset_x: 0, offset_y: 0, offset_z: 0, yaw_degrees: 0 };
          this.previewBaseLocally();
        },
        onRequestFinish: async commit => {
          window.clearTimeout(this.basePreviewTimer);
          const result = await this.calibrationAction(commit ? "base_save" : "base_cancel", { session_id: this.baseSession, ...(commit ? this.baseOffsets : {}) });
          this.baseMessage(result.message || "Base placement updated.");
          return true;
        },
        onFinish: () => this.closeBaseEditor(),
      });
      if (!editor) {
        await this.calibrationAction("base_cancel", { session_id: this.baseSession });
        this.baseSession = null;
        this.baseMessage("Open a connected 3D view and close other placement tools, then try again.");
        return;
      }
      this.baseEditor = editor;
      this.previewBaseLocally();
      this.startPreviewPolling();
      this.baseViewBefore = { enabled: element("so101-overlay-enabled").checked, occlusion: element("so101-overlay-occlusion-enabled").checked, scope: element("so101-overlay-scope").value };
      element("so101-overlay-scope").value = "full";
      this.syncOverlayScopeControls();
      element("so101-overlay-enabled").checked = true;
      element("so101-overlay-occlusion-enabled").checked = false;
      this.sendViewSettings();
      window.so101ArmController?.clearKeyboardMotion?.();
      document.getElementById("advanced-setup")?.setAttribute("hidden", "");
      document.body.classList.add("workspace-editing");
      this.baseHeartbeat = window.setInterval(() => this.queueBasePreview(), 4000);
      this.baseMessage("");
    } catch (error) { this.baseMessage(error.message || String(error)); }
  }

  closeBaseEditor() {
    window.clearTimeout(this.basePreviewTimer);
    window.clearInterval(this.baseHeartbeat);
    this.baseEditor = null;
    this.baseSession = null;
    this.baseOrigin = null;
    window.previewGodotRobotBase?.(null);
    window.clearInterval(this.previewPollTimer);
    this.previewPollTimer = null;
    if (this.baseViewBefore) {
      element("so101-overlay-enabled").checked = this.baseViewBefore.enabled;
      element("so101-overlay-occlusion-enabled").checked = this.baseViewBefore.occlusion;
      element("so101-overlay-scope").value = this.baseViewBefore.scope;
      this.syncOverlayScopeControls();
      this.baseViewBefore = null;
      this.sendViewSettings();
    }
    document.body.classList.remove("workspace-editing");
    document.getElementById("open-advanced")?.focus({ preventScroll: true });
  }

  queueManualPreview() {
    this.updateManualLabels();
    this.queuePreview("joint");
  }

  finishManualPreview() {
    this.manualReady = false;
    window.clearTimeout(this.manualPreviewTimer);
    window.clearTimeout(this.previewTimer);
    this.previewTimer = null;
    this.previewDirty = null;
    window.clearInterval(this.previewPollTimer);
    this.previewPollTimer = null;
    element("so101-manual-workspace").hidden = true;
    this.resetManualControls();
  }

  async performManualAction(operation, extra = {}) {
    try {
      const result = await this.calibrationAction(operation, extra);
      this.manualMessage(result.message || "Calibration updated.");
      return result;
    } catch (error) {
      this.manualMessage(String(error.message || error));
      return null;
    }
  }

  async exportCalibration() {
    const result = await this.performManualAction("export");
    if (!result?.file) return;
    const blob = new Blob([JSON.stringify(result.file, null, 2) + "\n"], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = `so101-visual-calibration-${new Date().toISOString().slice(0, 10)}.json`;
    document.body.append(link);
    link.click();
    link.remove();
    window.setTimeout(() => URL.revokeObjectURL(url), 1000);
    this.manualMessage("Exported the saved calibration as JSON.");
  }

  async chooseCalibrationFile(event) {
    const file = event.target.files?.[0];
    event.target.value = "";
    if (!file) return;
    try {
      if (file.size > 16384) throw new Error("Choose a calibration JSON file smaller than 16 KB.");
      const value = JSON.parse(await file.text());
      if (value?.type !== "so101_visual_calibration" || value.schema_version !== 1) throw new Error("This is not a supported visual calibration file.");
      this.importFile = value;
      element("so101-import-name").textContent = file.name;
      element("so101-import-base").checked = false;
      element("so101-import-status").textContent = "";
      element("so101-import-dialog").showModal();
    } catch (error) { this.manualMessage(String(error.message || error)); }
  }

  controllerStatus() {
    return window.getSo101ArmStatus?.() || {};
  }

  calibrationStatus() {
    const arm = this.controllerStatus();
    return arm.status?.robot_calibration || {};
  }

  calibrationUnavailable() {
    const arm = this.controllerStatus(), calibration = this.calibrationStatus();
    if (!arm.serverConnected) return "Robot server disconnected. Reconnect before calibration.";
    if (!arm.followerConnected) return "Robot offline. Start the follower service before calibration.";
    if (!arm.status?.measured_feedback_fresh) return "Waiting for fresh robot joint feedback.";
    if (!calibration.state || ["offline", "loading"].includes(calibration.state)) return "Godot calibration is offline. Open the robot scene or start the runtime.";
    return "";
  }

  async requestCalibration(settings) {
    this.calibrationError = "";
    this.calibrationRequest = {before:JSON.stringify(this.calibrationStatus()), deadline:Date.now()+10000};
    this.updateStatus();
    try { await window.so101ArmController.requestViewSettings(settings); }
    catch (error) { this.calibrationRequest = null; this.calibrationError = error.message; }
    this.updateStatus();
  }

  statusSummary() {
    const arm = this.controllerStatus();
    const input = arm.keyboardConnected ? "keyboard on" : arm.leaderConnected ? `leader ${arm.leaderHz || 0} Hz` : "controller off";
    return `SO-101 ${arm.state || "loading"} | ${input} | follower ${arm.followerConnected ? "on" : "off"}`;
  }

  renderStatus() {
    const arm = this.controllerStatus();
    this.armNotices?.update(arm);
    const status = arm.status || {};
    const calibration = status.robot_calibration || {};
    const manual = calibration.manual_calibration || {};
    const base = manual.base_adjustment || {};
    window.setGodotRobotPreviewActive?.(manual.enabled === true || base.enabled === true);
    if (manual.render_state) window.updateGodotRobotOverlay?.(manual.render_state);
    if (this.manualPending && manual.reply?.request_id === this.manualPending.request_id) {
      const pending = this.manualPending;
      this.manualPending = null;
      window.clearTimeout(pending.timer);
      if (manual.reply.ok) pending.resolve(manual.reply);
      else pending.reject(new Error(manual.reply.message || "Godot rejected this calibration action."));
    }
    const manualOffline = calibration.state === "offline" || manual.supported !== true;
    const manualBusy = ["capturing", "solving"].includes(calibration.state) || Boolean(this.manualPending);
    element("so101-base-adjust").disabled = manualOffline || manualBusy || manual.enabled === true || base.enabled === true || base.supported !== true;
    element("so101-manual-open").disabled = manualOffline || manualBusy || !manual.registered || base.enabled === true;
    element("so101-claw-adjust").disabled = element("so101-manual-open").disabled;
    element("so101-calibration-export").disabled = manualOffline || manualBusy || !manual.registered || manual.enabled === true || base.enabled === true;
    element("so101-calibration-import").disabled = manualOffline || manualBusy || manual.enabled === true || base.enabled === true;
    if (this.baseEditor?.active && !this.baseEditor.finishing && !this.manualPending && base.enabled !== true) {
      this.baseEditor.close(false);
      this.baseMessage("Base preview ended. Open Adjust Base to continue.");
    }
    for (const id of ["so101-manual-save", "so101-manual-reset", "so101-manual-cancel", "so101-import-apply"]) element(id).disabled = manualOffline || manualBusy;
    for (const input of document.querySelectorAll("#so101-manual-workspace input, #so101-manual-workspace select")) input.disabled = manualOffline || manualBusy;
    element("so101-connect").textContent = arm.leaderConnected ? "Leader Connected" : "Connect Leader";
    element("so101-keyboard").textContent = arm.keyboardConnected ? "Keyboard Enabled" : "Keyboard Control";
    element("so101-keyboard").classList.toggle("primary", arm.keyboardConnected === true);
    element("keyboard-controls-guide").hidden = arm.keyboardConnected !== true;
    const blocksEnable = ["fault", "restarting", "calibrating", "unresponsive"].includes(arm.state);
    element("so101-enable").disabled = !((arm.leaderConnected || arm.keyboardConnected) && arm.serverConnected && arm.followerConnected) || arm.armed || blocksEnable;
    element("so101-enable").textContent = arm.armed ? "Arm Enabled" : "Enable Arm";
    element("so101-restart").disabled = !arm.serverConnected || arm.restartRequested;
    element("so101-restart").textContent = arm.restartRequested ? "Reconnecting Arm…" : "Reconnect Arm Hardware";
    const returningRest = status.rest_return_active === true;
    element("so101-return-rest").disabled = !arm.serverConnected || !arm.followerConnected || status.rest_pose_available === false || returningRest || ["fault", "restarting", "calibrating", "unresponsive"].includes(arm.state);
    element("so101-return-rest").textContent = returningRest ? `Returning to Rest ${Math.round(Number(status.rest_return_progress || 0) * 100)}%` : "Return Arm to Rest Pose";
    const pendingSave = window.so101ArmController?.restSaveRequestId;
    const waitingSave = pendingSave && status.rest_save_request_id !== pendingSave && Date.now() < window.so101ArmController.restSavePendingUntil;
    element("so101-save-rest").disabled = !arm.serverConnected || !arm.followerConnected || status.rest_pose_save_supported !== true || returningRest || waitingSave || !["ready", "hold"].includes(arm.state);
    const saveMessage = pendingSave && status.rest_save_request_id === pendingSave ? status.rest_save_message : "";
    element("so101-rest-status").textContent = saveMessage || window.so101ArmController?.restSaveError || (waitingSave
      ? "Saving current pose; waiting for the follower response." : status.rest_pose_available === false
      ? `Rest pose unavailable. Use Save Current Pose as Rest in Setup. ${status.rest_pose_fault || ""}`
      : status.rest_pose_available === true ? "Rest pose saved for this arm." : "Waiting for rest-pose status.");
    const warnings = Array.isArray(status.setup_warnings) ? status.setup_warnings : [];
    element("so101-setup-warnings").textContent = [
      ...warnings,
      ...(arm.followerConnected && status.rest_pose_save_supported !== true ? ["Restart the updated Python launcher to save a rest pose here."] : []),
    ].join("\n");
    const writing = status.state === "armed" || status.state === "calibrating";
    const lastWrite = writing ? (status.last_write_age_ms == null ? "--" : `${Math.round(status.last_write_age_ms)}ms ago`) : `${status.state || "idle"} (motion writes paused)`;
    element("so101-health").textContent = [
      `Follower ${status.follower_responsive === false ? "UNRESPONSIVE" : status.follower_connected ? "connected" : "offline"} | ${status.state || arm.state || "--"}`,
      `Commands ${status.command_rate_hz || 0}/s | writes ${status.write_rate_hz || 0}/s | status age ${status.status_age_ms == null ? "--" : `${Math.round(status.status_age_ms)}ms`}`,
      `Measured ${status.measured_feedback_fresh ? "fresh" : "STALE"} | reads ${status.read_rate_hz || 0}/s | age ${status.last_read_age_ms == null ? "--" : `${Math.round(status.last_read_age_ms)}ms`}`,
      `Last write ${lastWrite} | safety stops ${status.following_error_trip_count || 0} | restart #${status.restart_count || 0}`,
      `Status ${status.message || "--"}`,
      `Fault ${arm.fault || "none"}`,
      `Profile ${status.profile_path || "--"}`,
      `Coordinates: leader ${status.leader_coordinate_system || "--"} / follower ${status.follower_coordinate_system || "--"}`,
      `Leader directions (motors 1–6): ${(status.leader_joint_directions || []).join(", ") || "--"}`,
    ].join("\n");
    const capturing = calibration.state === "capturing";
    const solving = calibration.state === "solving";
    const offline = calibration.state === "offline";
    const mode = calibration.calibration_mode || "base";
    const unavailable = this.calibrationUnavailable();
    if (this.calibrationRequest) {
      if (JSON.stringify(calibration) !== this.calibrationRequest.before && ["capturing","solving","failed"].includes(calibration.state)) {
        this.calibrationRequest = null;
      } else if (Date.now() > this.calibrationRequest.deadline) {
        this.calibrationRequest = null;
        this.calibrationError = "Godot did not start calibration. Check the runtime connection before retrying.";
      }
    }
    if (capturing || solving) this.calibrationError = "";
    const progress = Math.max(0, Math.min(1, Number(calibration.progress || 0)));
    element("so101-calibration-progress").value = progress;
    element("so101-calibration-progress-value").textContent = `${Math.round(progress * 100)}%`;
    const placingArm = mode === "registration" || mode === "base";
    element("so101-calibrate").textContent = capturing && placingArm ? "Cancel Arm Alignment" : solving && placingArm ? "Aligning Arm" : offline ? "Godot Calibration Offline" : "Align Arm";
    element("so101-calibrate").disabled = manual.enabled === true || base.enabled === true || solving || Boolean(this.calibrationRequest) || (!capturing && Boolean(unavailable)) || (capturing && !placingArm);
    const message = this.calibrationError || (this.calibrationRequest ? "Request sent. Waiting for Godot calibration…" : !capturing && !solving && unavailable ? unavailable : calibration.message || "Arm position calibration ready");
    element("so101-calibration-status").textContent = `${message}\nFrames ${calibration.frames || 0} | confidence ${Math.round((calibration.confidence || 0) * 100)}%`;
  }

  sendFeedbackSettings() {
    return window.so101ArmController?.setFeedbackSettings({
      measured_feedback_enabled: true,
      target_ghost_enabled: element("so101-target-ghost-enabled").checked,
      following_error_safety_enabled: element("so101-following-error-safety-enabled").checked,
      freeze_overlay_on_stale_enabled: true,
      d455_visual_correction_enabled: element("so101-d455-visual-correction-enabled").checked,
    });
  }

  reportError(error) {
    if (!window.so101ArmController) return;
    window.so101ArmController.latest.fault = String(error?.message || error);
    window.so101ArmController.emitStatus();
  }

  confirmAction(message, label = "Continue") {
    if (this.confirmationDialog) return Promise.resolve(false);
    const dialog = document.createElement("dialog"), form = document.createElement("form");
    const text = document.createElement("p"), actions = document.createElement("div");
    const cancel = document.createElement("button"), accept = document.createElement("button");
    dialog.setAttribute("aria-label", "Confirm robot action");
    dialog.style.cssText = "max-width:30rem;border:1px solid var(--line,#666);border-radius:10px;padding:24px;background:var(--panel,#18202b);color:var(--text,#fff)";
    text.textContent = message; text.style.whiteSpace = "pre-line";
    actions.style.cssText = "display:flex;gap:12px;justify-content:flex-end";
    form.method = "dialog";
    cancel.textContent = "Cancel"; cancel.value = "cancel"; cancel.autofocus = true;
    accept.textContent = label; accept.value = "confirm";
    actions.append(cancel, accept); form.append(text, actions); dialog.append(form);
    document.body.append(dialog); this.confirmationDialog = dialog;
    const controller = window.so101ArmController;
    if (controller) { controller.keyboardModalOpen = true; controller.clearKeyboardMotion(); }
    return new Promise(resolve => {
      const finish = () => {
        if (controller) controller.keyboardModalOpen = false;
        this.confirmationDialog = null; dialog.remove();
        resolve(dialog.returnValue === "confirm");
      };
      dialog.addEventListener("close", finish, {once:true});
      try { dialog.showModal(); cancel.focus(); }
      catch (error) { dialog.returnValue = "cancel"; finish(); this.reportError(error); }
    });
  }

  bind({ sendViewSettings, updateStatus }) {
    this.sendViewSettings = sendViewSettings;
    this.applyMirrorView(this.mirrorView);
    element('so101-mirror-view').addEventListener('change',event=>{
      this.applyMirrorView(event.currentTarget.checked);
      try{localStorage.setItem('robotTeleopMirrorView',String(this.mirrorView));}catch{}
      sendViewSettings();
    });
    this.updateStatus = updateStatus;
    element("so101-notifications-enabled").addEventListener("change", (event) => {
      this.notificationsEnabled = event.currentTarget.checked;
      this.armNotices?.setEnabled(this.notificationsEnabled);
      try { localStorage.setItem(NOTIFICATIONS_STORAGE_KEY, String(this.notificationsEnabled)); } catch { /* Storage is optional. */ }
      event.currentTarget.blur();
    });
    element("so101-base-adjust").addEventListener("click", () => this.openBaseEditor());
    element("so101-connect").addEventListener("click", async () => { try { await window.so101ArmController.connectLeader(); } catch (error) { this.reportError(error); } });
    element("so101-keyboard").addEventListener("click", async () => {
      try {
        if (window.so101ArmController.latest.keyboardConnected) window.so101ArmController.disconnectKeyboard(true);
        else await window.so101ArmController.connectKeyboard();
      } catch (error) { this.reportError(error); }
      updateStatus();
    });
    element("so101-enable").addEventListener("click", () => { try { window.so101ArmController.enableArm(); } catch (error) { this.reportError(error); } });
    element("so101-hold").addEventListener("click", () => window.so101ArmController?.hold());
    element("so101-save-rest").addEventListener("click", async () => {
      if (!await this.confirmAction("Save the arm's current physical pose as its rest pose? Any previous saved rest pose will be backed up.", "Save Rest Pose")) return;
      try { window.so101ArmController.saveCurrentRestPose(); } catch (error) { this.reportError(error); }
    });
    element("so101-return-rest").addEventListener("click", async () => {
      try { window.so101ArmController.returnToRest(); } catch (error) { this.reportError(error); }
    });
    element("so101-restart").addEventListener("click", () => { try { window.so101ArmController.restartFollower(); } catch (error) { this.reportError(error); } });
    element("so101-idle-return-enabled").addEventListener("change", async (event) => {
      const input = event.currentTarget;
      if (input.checked && !await this.confirmAction("Enable unattended return after 5 minutes without meaningful control input?", "Enable Idle Return")) input.checked = false;
      sendViewSettings();
      window.so101ArmController?.setIdleReturn(input.checked, 300);
      input.blur();
    });
    for (const id of ["so101-target-ghost-enabled", "so101-following-error-safety-enabled", "so101-d455-visual-correction-enabled"]) {
      element(id).addEventListener("change", (event) => { sendViewSettings(); this.sendFeedbackSettings(); event.currentTarget.blur(); });
    }
    for (const id of ["so101-overlay-enabled", "so101-overlay-occlusion-enabled", "so101-overlay-style", "so101-overlay-scope"]) {
      element(id).addEventListener("change", (event) => { this.syncOverlayScopeControls(); sendViewSettings(); event.currentTarget.blur(); });
    }
    for (const id of ["so101-claw-opacity", "so101-claw-buffer"]) element(id).addEventListener("input", () => { this.syncOverlayScopeControls(); sendViewSettings(); });
    element("so101-mask-buffer").addEventListener("input", () => { this.syncOverlayScopeControls(); sendViewSettings(); });
    element("so101-body-opacity").addEventListener("input", () => { this.syncOverlayScopeControls(); sendViewSettings(); });
    element("so101-calibrate").addEventListener("click", async () => {
      const calibration = this.calibrationStatus();
      const capturing = calibration.state === "capturing";
      if (!capturing && this.calibrationUnavailable()) return;
      if (!capturing && !await this.confirmAction("The follower will run a measured base sweep to align the whole arm. Joint settings and claw geometry will stay fixed.\n\nClear its workspace and stay ready to remove physical power.", "Start Alignment")) return;
      this.requestCalibration(capturing ? { cancel_robot_position_calibration: true } : { calibrate_robot_position: true });
    });
    const openManualPreview = async () => {
      if (await this.performManualAction("begin")) {
        this.resetManualControls();
        this.manualReady = true;
        this.startPreviewPolling();
        element("so101-manual-workspace").hidden = false;
        element("so101-manual-workspace").querySelector("input[type=number]").focus();
      }
    };
    element("so101-manual-open").addEventListener("click", openManualPreview);
    element("so101-claw-adjust").addEventListener("click", openManualPreview);
    for (const input of this.manualInputs) input.addEventListener("input", () => this.queueManualPreview());
    for (const input of document.querySelectorAll("[data-manual-number]")) input.addEventListener("input", () => {
      if (!input.checkValidity() || !Number.isFinite(input.valueAsNumber)) return;
      const slider = this.manualInputs.find((candidate) => candidate.dataset.manualKey === input.dataset.manualNumber);
      slider.value = String(input.valueAsNumber / Number(input.dataset.manualScale || 1));
      this.queueManualPreview();
    });
    element("so101-manual-wrist-direction").addEventListener("change", () => this.queueManualPreview());
    element("so101-manual-reset").addEventListener("click", async () => {
      if (await this.performManualAction("reset")) this.resetManualControls();
    });
    element("so101-manual-cancel").addEventListener("click", async () => {
      if (await this.performManualAction("cancel")) this.finishManualPreview();
    });
    element("so101-manual-save").addEventListener("click", async () => {
      if (await this.performManualAction("save")) this.finishManualPreview();
    });
    element("so101-calibration-export").addEventListener("click", () => this.exportCalibration());
    element("so101-calibration-import").addEventListener("click", () => element("so101-calibration-file").click());
    element("so101-calibration-file").addEventListener("change", (event) => this.chooseCalibrationFile(event));
    element("so101-import-cancel").addEventListener("click", () => element("so101-import-dialog").close());
    element("so101-import-apply").addEventListener("click", async () => {
      if (await this.performManualAction("import", { file: this.importFile, restore_base: element("so101-import-base").checked })) element("so101-import-dialog").close();
    });
    window.addEventListener("so101-arm-status", updateStatus);
    window.addEventListener("pagehide", () => {
      if (this.baseSession) window.so101ArmController.requestViewSettings({ robot_visual_calibration_action: { operation: "base_cancel", session_id: this.baseSession, request_id: `base-close-${Date.now()}` } }).catch(() => {});
      if (this.manualReady) window.so101ArmController.requestViewSettings({ robot_visual_calibration_action: { operation: "cancel", request_id: `close-${Date.now()}` } }).catch(() => {});
    });
  }

  start() {
    this.sendFeedbackSettings();
    window.so101ArmController?.setIdleReturn(element("so101-idle-return-enabled").checked, 300);
    this.renderStatus();
  }

  prepareSession() { return window.so101ArmController.prepareTrialControl({connect: true}); }
  prepareTrial() { return window.so101ArmController.prepareTrialControl(); }
  sessionConnected() {
    const arm = window.so101ArmController?.latest;
    return arm?.serverConnected === true && arm.followerConnected === true
      && (arm.leaderConnected === true || arm.keyboardConnected === true);
  }
  pauseTrial() { return window.so101ArmController.pauseTrial(); }
  resumeTrial() { return window.so101ArmController.resumeTrial(); }
  finishTrialPause() { window.so101ArmController.finishTrialPause(); }

  stop() {
    this.armNotices?.dispose();
    this.confirmationDialog?.close("cancel");
    window.clearInterval(this.previewPollTimer);
    window.clearTimeout(this.previewTimer);
    window.so101ArmController?.shutdown();
  }
}

export function createRobotModule(manifest) {
  return new So101WebModule(manifest);
}
