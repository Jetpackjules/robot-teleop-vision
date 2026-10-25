const MODEL_ROOT = "/robot-modules/so101/assets";

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
  accentLinks: ["moving_jaw_so101_v1_link"],
  calibrationLinks: ["wrist_link", "gripper_link", "moving_jaw_so101_v1_link"],
  calibrationBoundsLinks: ["gripper_link", "moving_jaw_so101_v1_link"],
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
  workspaceBoundary: {
    shape: "cylinder",
    anchorLink: "base_link",
    innerRadius: 0.07,
    outerRadius: 0.43,
    minimumHeight: 0.015,
    maximumHeight: 0.48,
  },
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
  <label class="toggle-row" title="Opt-in: modeled checks cannot detect a person or loose object in the path."><span>Auto-return to rest after 10 min</span><input id="so101-idle-return-enabled" type="checkbox"></label>
  <button id="so101-restart" type="button">Reconnect Arm Hardware</button>
  <details class="advanced-disclosure">
    <summary>Experimental safeguards</summary>
    <div class="disclosure-body">
      <label class="toggle-row"><span>Stop on following error / contact</span><input id="so101-following-error-safety-enabled" type="checkbox" checked></label>
      <p class="helper-text">This modeled safeguard can false-trigger on gravity sag or backlash. The hardware watchdog and joint limits remain authoritative.</p>
    </div>
  </details>
  <div id="so101-health" class="network-stats">Follower status unavailable</div>
`;

const VIEW_MARKUP = `
  <label class="toggle-row"><span>Robot overlay</span><input id="so101-overlay-enabled" type="checkbox" checked></label>
  <label>Robot overlay style
    <select id="so101-overlay-style"><option value="alignment" selected>Alignment (transparent + outlines)</option><option value="solid">Solid yellow</option></select>
  </label>
  <label class="toggle-row"><span>Overlay occludes scanned arm</span><input id="so101-overlay-occlusion-enabled" type="checkbox" checked></label>
  <label class="toggle-row"><span>Show commanded target ghost</span><input id="so101-target-ghost-enabled" type="checkbox" checked></label>
  <button id="so101-calibrate" class="primary" type="button">Calibrate Full Arm</button>
  <div class="calibration-progress"><progress id="so101-calibration-progress" max="1" value="0"></progress><span id="so101-calibration-progress-value" class="value-label">0%</span></div>
  <div id="so101-calibration-status" class="network-stats">Arm position calibration idle</div>
  <details class="advanced-disclosure">
    <summary>Recovery tools</summary>
    <div class="disclosure-body">
      <button id="so101-refine" type="button">Refine Arm Servos</button>
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
      <label>Tool X <span data-manual-value="manual_tool_x" class="value-label">0.0 mm</span><input data-manual-key="manual_tool_x" type="range" min="-0.05" max="0.05" step="0.0005" value="0"></label>
      <label>Tool Y <span data-manual-value="manual_tool_y" class="value-label">0.0 mm</span><input data-manual-key="manual_tool_y" type="range" min="-0.05" max="0.05" step="0.0005" value="0"></label>
      <label>Tool Z <span data-manual-value="manual_tool_z" class="value-label">0.0 mm</span><input data-manual-key="manual_tool_z" type="range" min="-0.05" max="0.05" step="0.0005" value="0"></label>
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
    this.manifest = manifest;
    this.renderer = RENDERER;
    this.sendViewSettings = () => false;
    this.updateStatus = () => {};
    this.manualInputs = [];
    this.manualPending = null;
    this.manualReady = false;
    this.manualPreviewTimer = null;
    this.importFile = null;
  }

  async mount({ settingsRoot, setupRoot, viewRoot, statusRoot, floatingRoot }) {
    const style = document.createElement("style");
    style.dataset.robotModule = "so101";
    style.textContent = MODULE_STYLE;
    document.head.append(style);
    if (settingsRoot) settingsRoot.innerHTML = SETTINGS_MARKUP;
    if (setupRoot) setupRoot.innerHTML = SETUP_MARKUP;
    if (viewRoot) viewRoot.innerHTML = VIEW_MARKUP;
    if (statusRoot) statusRoot.innerHTML = STATUS_MARKUP;
    if (floatingRoot) floatingRoot.innerHTML = FLOATING_MARKUP;
    this.manualInputs = Array.from(document.querySelectorAll("[data-manual-key]"));
    await import("./so101_web_serial.js");
    await import("./wrist_camera_view.js");
  }

  extendViewSettings(payload) {
    return {
      ...payload,
      robot_overlay_enabled: element("so101-overlay-enabled")?.checked !== false,
      robot_overlay_style: element("so101-overlay-style")?.value || "alignment",
      robot_overlay_mask_scanned_robot: element("so101-overlay-occlusion-enabled")?.checked !== false,
      arm_measured_feedback_enabled: true,
      arm_target_ghost_enabled: element("so101-target-ghost-enabled")?.checked !== false,
      arm_following_error_safety_enabled: element("so101-following-error-safety-enabled")?.checked !== false,
      arm_freeze_overlay_on_stale_enabled: true,
      arm_d455_visual_correction_enabled: element("so101-d455-visual-correction-enabled")?.checked === true,
      arm_idle_return_enabled: element("so101-idle-return-enabled")?.checked === true,
    };
  }

  restoreViewSettings(saved) {
    if (!saved) return;
    element("so101-overlay-enabled").checked = saved.robot_overlay_enabled !== false;
    element("so101-overlay-style").value = saved.robot_overlay_style === "solid" ? "solid" : "alignment";
    element("so101-overlay-occlusion-enabled").checked = (saved.robot_overlay_mask_scanned_robot ?? saved.robot_overlay_mask_scanned_arm) !== false;
    element("so101-target-ghost-enabled").checked = saved.arm_target_ghost_enabled !== false;
    element("so101-following-error-safety-enabled").checked = saved.arm_following_error_safety_enabled !== false;
    element("so101-d455-visual-correction-enabled").checked = saved.arm_d455_visual_correction_enabled === true;
    element("so101-idle-return-enabled").checked = saved.arm_idle_return_enabled === true;
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
      if (numeric && document.activeElement !== numeric) numeric.value = input.value;
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
    if (operation === "save" && Array.from(document.querySelectorAll("[data-manual-number]")).some(input => !input.checkValidity() || !Number.isFinite(input.valueAsNumber))) {
      return Promise.reject(new Error("Enter a valid angle in each joint field before saving."));
    }
    const request_id = globalThis.crypto?.randomUUID?.() || `cal-${Date.now()}-${Math.random().toString(36).slice(2)}`;
    const action = { operation, request_id, ...extra };
    const settings = ["save", "preview"].includes(operation) ? this.manualPayload() : {};
    if (operation === "preview") {
      return this.sendViewSettings({ ...settings, robot_visual_calibration_action: action });
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
      if (!this.sendViewSettings({ ...settings, robot_visual_calibration_action: action })) {
        window.clearTimeout(timer);
        this.manualPending = null;
        reject(new Error("Connect the controller to Godot before changing calibration."));
      }
    });
  }

  queueManualPreview() {
    this.updateManualLabels();
    window.clearTimeout(this.manualPreviewTimer);
    this.manualPreviewTimer = window.setTimeout(() => {
      if (!this.manualReady || this.manualPending) return;
      if (!this.calibrationAction("preview")) this.manualMessage("Preview could not reach Godot; check the connection.");
    }, 60);
  }

  finishManualPreview() {
    this.manualReady = false;
    window.clearTimeout(this.manualPreviewTimer);
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

  statusSummary() {
    const arm = this.controllerStatus();
    const input = arm.keyboardConnected ? "keyboard on" : arm.leaderConnected ? `leader ${arm.leaderHz || 0} Hz` : "controller off";
    return `SO-101 ${arm.state || "loading"} | ${input} | follower ${arm.followerConnected ? "on" : "off"}`;
  }

  renderStatus() {
    const arm = this.controllerStatus();
    const status = arm.status || {};
    const calibration = status.robot_calibration || {};
    const manual = calibration.manual_calibration || {};
    if (this.manualPending && manual.reply?.request_id === this.manualPending.request_id) {
      const pending = this.manualPending;
      this.manualPending = null;
      window.clearTimeout(pending.timer);
      if (manual.reply.ok) pending.resolve(manual.reply);
      else pending.reject(new Error(manual.reply.message || "Godot rejected this calibration action."));
    }
    const manualOffline = calibration.state === "offline" || manual.supported !== true;
    const manualBusy = ["capturing", "solving"].includes(calibration.state) || Boolean(this.manualPending);
    element("so101-manual-open").disabled = manualOffline || manualBusy || !manual.registered;
    element("so101-calibration-export").disabled = manualOffline || manualBusy || !manual.registered || manual.enabled === true;
    element("so101-calibration-import").disabled = manualOffline || manualBusy || manual.enabled === true;
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
    const progress = Math.max(0, Math.min(1, Number(calibration.progress || 0)));
    element("so101-calibration-progress").value = progress;
    element("so101-calibration-progress-value").textContent = `${Math.round(progress * 100)}%`;
    element("so101-calibrate").textContent = capturing && mode === "base" ? "Cancel Full Arm Calibration" : solving && mode === "base" ? "Fitting Full Arm" : offline ? "Godot Calibration Offline" : "Calibrate Full Arm";
    element("so101-calibrate").disabled = manual.enabled === true || solving || offline || (capturing && mode !== "base");
    element("so101-refine").textContent = capturing && mode === "joints" ? "Cancel Servo Refinement" : solving && mode === "joints" ? "Fitting Arm Servos" : offline ? "Godot Calibration Offline" : "Refine Arm Servos";
    element("so101-refine").disabled = manual.enabled === true || solving || offline || (capturing && mode !== "joints");
    element("so101-calibration-status").textContent = `${calibration.message || "Arm position calibration idle"}\nFrames ${calibration.frames || 0} | confidence ${Math.round((calibration.confidence || 0) * 100)}%`;
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

  bind({ sendViewSettings, updateStatus }) {
    this.sendViewSettings = sendViewSettings;
    this.updateStatus = updateStatus;
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
    element("so101-save-rest").addEventListener("click", () => {
      if (!window.confirm("Save the arm's current physical pose as its rest pose? Any previous saved rest pose will be backed up.")) return;
      try { window.so101ArmController.saveCurrentRestPose(); } catch (error) { this.reportError(error); }
    });
    element("so101-return-rest").addEventListener("click", () => {
      if (!window.confirm("Return the physical arm to its saved rest pose now?\n\nClear the workspace and stay ready to press Hold Arm or remove power.")) return;
      try { window.so101ArmController.returnToRest(); } catch (error) { this.reportError(error); }
    });
    element("so101-restart").addEventListener("click", () => { try { window.so101ArmController.restartFollower(); } catch (error) { this.reportError(error); } });
    element("so101-idle-return-enabled").addEventListener("change", (event) => {
      if (event.currentTarget.checked && !window.confirm("Enable unattended return after 10 minutes without meaningful control input?")) event.currentTarget.checked = false;
      sendViewSettings();
      window.so101ArmController?.setIdleReturn(event.currentTarget.checked, 600);
      event.currentTarget.blur();
    });
    for (const id of ["so101-target-ghost-enabled", "so101-following-error-safety-enabled", "so101-d455-visual-correction-enabled"]) {
      element(id).addEventListener("change", (event) => { sendViewSettings(); this.sendFeedbackSettings(); event.currentTarget.blur(); });
    }
    for (const id of ["so101-overlay-enabled", "so101-overlay-occlusion-enabled", "so101-overlay-style"]) {
      element(id).addEventListener("change", (event) => { sendViewSettings(); event.currentTarget.blur(); });
    }
    element("so101-calibrate").addEventListener("click", () => {
      const calibration = this.calibrationStatus();
      const capturing = calibration.state === "capturing";
      if (!capturing && calibration.state === "offline") return void (element("so101-calibration-status").textContent = "Godot calibration module is offline. Start the runtime and retry.");
      if (!capturing && !window.confirm("The follower will locate its base, calibrate outward through the arm, and sample claw openings.\n\nClear its workspace and stay ready to remove physical power.")) return;
      if (!sendViewSettings(capturing ? { cancel_robot_position_calibration: true } : { calibrate_robot_position: true })) element("so101-calibration-status").textContent = "Start the controller so Godot can receive calibration commands.";
    });
    element("so101-refine").addEventListener("click", () => {
      const calibration = this.calibrationStatus();
      const capturing = calibration.state === "capturing" && calibration.calibration_mode === "joints";
      if (!capturing && calibration.state === "offline") return void (element("so101-calibration-status").textContent = "Godot calibration module is offline. Start the runtime and retry.");
      if (!capturing && !window.confirm("Refine shoulder, elbow, and wrist servos now? Clear the workspace and stay ready to remove power.")) return;
      if (!sendViewSettings(capturing ? { cancel_robot_position_calibration: true } : { refine_robot_joint_alignment: true })) element("so101-calibration-status").textContent = "Start the controller so Godot can receive calibration commands.";
    });
    element("so101-manual-open").addEventListener("click", async () => {
      if (await this.performManualAction("begin")) {
        this.resetManualControls();
        this.manualReady = true;
        element("so101-manual-workspace").hidden = false;
        element("so101-manual-workspace").querySelector("input[type=number]").focus();
      }
    });
    for (const input of this.manualInputs) input.addEventListener("input", () => this.queueManualPreview());
    for (const input of document.querySelectorAll("[data-manual-number]")) input.addEventListener("input", () => {
      if (!input.checkValidity() || !Number.isFinite(input.valueAsNumber)) return;
      const slider = this.manualInputs.find((candidate) => candidate.dataset.manualKey === input.dataset.manualNumber);
      slider.value = input.value;
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
      if (this.manualReady) sendViewSettings({ robot_visual_calibration_action: { operation: "cancel", request_id: `close-${Date.now()}` } });
    });
  }

  start() {
    this.sendFeedbackSettings();
    window.so101ArmController?.setIdleReturn(element("so101-idle-return-enabled").checked, 600);
    this.renderStatus();
  }

  stop() { window.so101ArmController?.shutdown(); }
}

export function createRobotModule(manifest) {
  return new So101WebModule(manifest);
}
