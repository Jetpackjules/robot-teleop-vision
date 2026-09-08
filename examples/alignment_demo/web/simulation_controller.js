// Isolated simulation input. Deliberately does not import the production arm
// controller: that controller automatically opens its robot-control socket.
const MOTOR_IDS = [1, 2, 3, 4, 5, 6];
const TARGET_HZ = 30;
const START_POSE = [-40.4296875, 130, 38, 78, 0, 25];

export function packetChecksum(bytes, end = bytes.length - 1) {
  let sum = 0;
  for (let index = 2; index < end; index += 1) sum = (sum + bytes[index]) & 255;
  return (~sum) & 255;
}

export function makeLeaderReadPacket() {
  // Feetech sync-read (0x82), Present_Position at address 56, two bytes.
  // No other serial instruction is produced anywhere in this module.
  const packet = new Uint8Array([255, 255, 254, 10, 0x82, 56, 2, ...MOTOR_IDS, 0]);
  packet[packet.length - 1] = packetChecksum(packet);
  return packet;
}

export class LeaderPacketDecoder {
  constructor() { this.buffer = new Uint8Array(); }

  push(chunk) {
    const joined = new Uint8Array(this.buffer.length + chunk.length);
    joined.set(this.buffer);
    joined.set(chunk, this.buffer.length);
    this.buffer = joined.slice(-4096);
    const readings = [];
    while (this.buffer.length >= 6) {
      let header = -1;
      for (let index = 0; index < this.buffer.length - 1; index += 1) {
        if (this.buffer[index] === 255 && this.buffer[index + 1] === 255) { header = index; break; }
      }
      if (header < 0) { this.buffer = this.buffer.slice(-1); break; }
      this.buffer = this.buffer.slice(header);
      if (this.buffer.length < 4) break;
      const size = this.buffer[3] + 4;
      if (size !== 8) { this.buffer = this.buffer.slice(1); continue; }
      if (this.buffer.length < size) break;
      const packet = this.buffer.slice(0, size);
      this.buffer = this.buffer.slice(size);
      if (packetChecksum(packet) !== packet[7] || packet[4] !== 0 || !MOTOR_IDS.includes(packet[2])) continue;
      const position = packet[5] | (packet[6] << 8);
      if (position <= 4095) readings.push([packet[2], position]);
    }
    return readings;
  }
}

export function parseLeaderCalibration(data) {
  const raw = data?.leader?.calibration || data?.calibration || data;
  const calibration = {};
  for (const field of ["homing_offset", "drive_mode", "start_pos", "end_pos", "calib_mode"]) {
    if (!Array.isArray(raw?.[field]) || raw[field].length !== 6) throw new Error(`Calibration needs six ${field} values`);
    calibration[field] = raw[field].map((value) => {
      if (field === "calib_mode") {
        const mode = String(value).toUpperCase();
        if (!["DEGREE", "LINEAR"].includes(mode)) throw new Error("Unsupported calibration mode");
        return mode;
      }
      if (!Number.isInteger(value) || Math.abs(value) > 1000000) throw new Error("Calibration values must be finite integers");
      if (field === "drive_mode" && ![0, 1].includes(value)) throw new Error("Drive mode must be 0 or 1");
      return value;
    });
  }
  calibration.calib_mode.forEach((mode, index) => {
    if (mode === "LINEAR" && calibration.start_pos[index] === calibration.end_pos[index]) throw new Error("Calibration has a zero linear span");
  });
  return calibration;
}

export function normalizeLeaderPositions(positions, calibration) {
  if (!Array.isArray(positions) || positions.length !== 6
      || positions.some((value) => !Number.isInteger(value) || value < 0 || value > 4095)) {
    throw new Error("Expected six raw encoder values from 0 to 4095");
  }
  // Same equations as ArmCalibration.raw_to_normalized in so101_arm_common.py.
  const values = positions.map((raw, index) => {
    if (calibration.calib_mode[index] === "LINEAR") {
      return (raw - calibration.start_pos[index]) / (calibration.end_pos[index] - calibration.start_pos[index]) * 100;
    }
    const directed = calibration.drive_mode[index] ? -raw : raw;
    return (directed + calibration.homing_offset[index]) / 2048 * 180;
  });
  if (values.some((value) => !Number.isFinite(value)) || values.slice(0, 5).some((value) => Math.abs(value) > 360)) {
    throw new Error("Leader calibration produces a pose outside the simulation range");
  }
  values[5] = Math.min(100, Math.max(0, values[5]));
  return values;
}

export class ReadOnlyLeader {
  constructor(onPose, onStatus) {
    this.onPose = onPose;
    this.onStatus = onStatus;
    this.port = null;
    this.connecting = false;
    this.connectEpoch = 0;
    this.reader = null;
    this.running = false;
    this.stopping = null;
    this.responses = new Map();
    this.decoder = new LeaderPacketDecoder();
    this.pumpPromise = null;
    this.loopPromise = null;
  }

  async connect(serial = navigator.serial) {
    if (!serial) throw new Error("Use desktop Chrome or Edge for Web Serial");
    if (this.port || this.stopping || this.connecting) throw new Error("Disconnect the previous leader first");
    const epoch = ++this.connectEpoch;
    this.connecting = true;
    let candidate = null;
    let opened = false;
    // Explicit user gesture only. No getPorts(), reconnect timer or torque setup.
    try {
      candidate = await serial.requestPort({ filters: [{ usbVendorId: 0x1a86, usbProductId: 0x55d3 }] });
      if (epoch !== this.connectEpoch) throw new Error("Leader connection cancelled");
      await candidate.open({ baudRate: 1000000, bufferSize: 4096 });
      opened = true;
      // Stop/pagehide can run while either the chooser or port.open() is pending.
      // A late completion must release its port before any polling begins.
      if (epoch !== this.connectEpoch) throw new Error("Leader connection cancelled");
      this.port = candidate;
      this.reader = candidate.readable.getReader();
      this.running = true;
      this.decoder = new LeaderPacketDecoder();
      this.onStatus("Reading leader encoders; no actuator writes");
      this.pumpPromise = this.readPump().catch((error) => this.failed(error));
      this.loopPromise = this.pollLoop().catch((error) => this.failed(error));
    } catch (error) {
      if (opened && this.port !== candidate) {
        try { await candidate.close(); } catch (_) { /* USB already disconnected */ }
      }
      throw error;
    } finally {
      this.connecting = false;
    }
  }

  failed(error) {
    this.running = false;
    this.onStatus(`Leader stopped: ${error?.message || error}`);
    // Do not await from inside a task that disconnect() itself joins.
    void this.disconnect();
  }

  async readPump() {
    const reader = this.reader;
    while (this.running) {
      const { value, done } = await reader.read();
      if (done) {
        if (this.running) throw new Error("Leader serial stream ended");
        break;
      }
      if (value) for (const [id, position] of this.decoder.push(value)) this.responses.set(id, position);
    }
  }

  async pollLoop() {
    while (this.running) {
      const began = performance.now();
      this.responses.clear();
      const writer = this.port.writable.getWriter();
      try { await writer.write(makeLeaderReadPacket()); } finally { writer.releaseLock(); }
      while (this.running && this.responses.size < 6 && performance.now() - began < 100) {
        await new Promise((resolve) => setTimeout(resolve, 2));
      }
      if (!this.running) break;
      if (this.responses.size === 6) this.onPose(MOTOR_IDS.map((id) => this.responses.get(id)));
      else this.onStatus(`Waiting for all encoders (${this.responses.size}/6); virtual input will pause`);
      await new Promise((resolve) => setTimeout(resolve, Math.max(0, 1000 / TARGET_HZ - (performance.now() - began))));
    }
  }

  async disconnect() {
    this.connectEpoch += 1;
    if (this.stopping) return this.stopping;
    this.running = false;
    this.stopping = (async () => {
      try { if (this.reader) await this.reader.cancel(); } catch (_) { /* disconnected USB */ }
      await Promise.allSettled([this.pumpPromise, this.loopPromise]);
      try { this.reader?.releaseLock(); } catch (_) { /* already released */ }
      try { if (this.port) await this.port.close(); } catch (_) { /* disconnected USB */ }
      this.reader = null;
      this.port = null;
      this.pumpPromise = null;
      this.loopPromise = null;
    })();
    try { await this.stopping; } finally { this.stopping = null; }
  }
}

export class RelayConnection {
  constructor({url, onState, releaseInputs, onMessage = () => {},
    createSocket = (address) => new WebSocket(address),
    setTimer = (callback, delay) => setTimeout(callback, delay),
    clearTimer = (timer) => clearTimeout(timer)}) {
    Object.assign(this, {url, onState, releaseInputs, onMessage, createSocket, setTimer, clearTimer});
    this.socket = null;
    this.state = "disconnected";
    this.retry = 0;
    this.timer = null;
  }

  get connected() { return this.state === "owner" && this.socket?.readyState === 1; }
  get terminal() { return this.state === "superseded" || this.state === "stopped"; }

  setState(state) { this.state = state; this.onState(state); }

  clearRetry() { if (this.timer !== null) this.clearTimer(this.timer); this.timer = null; }

  scheduleRetry() {
    if (this.terminal || this.timer !== null) return;
    const delay = Math.min(5000, 250 * 2 ** Math.min(this.retry++, 5));
    this.timer = this.setTimer(() => { this.timer = null; this.connect(); }, delay);
  }

  connect() {
    if (this.terminal || this.socket?.readyState === 0 || this.socket?.readyState === 1) return;
    this.clearRetry();
    this.setState("connecting");
    let socket;
    try { socket = this.createSocket(this.url); } catch (_) {
      this.setState("disconnected"); this.scheduleRetry(); return;
    }
    this.socket = socket;
    socket.addEventListener("message", (event) => {
      if (socket !== this.socket || this.terminal) return;
      try {
        const message = JSON.parse(event.data);
        if (message.type === "controller_status" && message.state === "superseded") {
          this.supersede();
        } else if (message.type === "controller_status" && message.state === "owner") {
          this.retry = 0; this.setState("owner");
        } else this.onMessage(message);
      } catch (_) { /* Ignore non-JSON relay messages. */ }
    });
    socket.addEventListener("close", (event) => {
      if (socket !== this.socket || this.terminal) return;
      if (event.code === 4001) { this.supersede(); return; }
      this.socket = null;
      this.releaseInputs("disconnected");
      this.setState("disconnected");
      this.scheduleRetry();
    });
    socket.addEventListener("error", () => {
      // Browser WebSockets deliver close after an error; close owns retry.
      if (socket === this.socket) socket.close();
    });
  }

  supersede() {
    if (this.terminal) return;
    this.clearRetry();
    this.setState("superseded");
    this.releaseInputs("superseded");
    this.socket?.close();
  }

  recover() { if (!this.terminal && !this.connected) this.connect(); }

  send(packet) {
    if (!this.connected || this.socket.bufferedAmount > 8192) return false;
    try { this.socket.send(JSON.stringify(packet)); return true; } catch (_) {
      this.socket.close(); return false;
    }
  }

  stop() {
    this.clearRetry();
    this.setState("stopped");
    this.releaseInputs("stopped");
    this.socket?.close();
  }
}

export function sceneInputReady(state, udpPort) {
  return state?.running === true && state.input_bound === true
    && Number.isInteger(state.input_port) && state.input_port === udpPort;
}

async function startController() {
  const byId = (id) => document.getElementById(id);
  let config = {udp_port: 14861};
  let sceneConnected = false;
  let seq = 0;
  const source = `browser-${crypto.randomUUID()}`;
  let calibration = null;
  let leaderPose = [...START_POSE];
  let leaderSampleAt = 0;
  let manualEnabled = false;
  let webcamEnabled = false;
  let webcamPending = false;
  let headGeneration = 0;
  let manualPose = [...START_POSE];
  let pendingAction = null;
  const sliderInputs = [];
  const jointDefinitions = [
    ["Base", -200, 180], ["Shoulder", -80, 260], ["Elbow", -180, 180],
    ["Wrist pitch", -180, 260], ["Wrist roll", -180, 180], ["Gripper", 0, 100],
  ];
  const leader = new ReadOnlyLeader((raw) => {
    try {
      leaderPose = normalizeLeaderPositions(raw, calibration);
      leaderSampleAt = performance.now();
      byId("leader-status").textContent = `Reading all 6 encoders → virtual arm\n${leaderPose.map((value) => value.toFixed(1)).join("  ")}`;
    } catch (error) {
      leaderSampleAt = 0;
      byId("leader-status").textContent = error.message;
    }
  }, (message) => { byId("leader-status").textContent = message; });
  const relay = new RelayConnection({
    url: `ws://${location.host}/input?client=${encodeURIComponent(source)}&opened=${performance.timeOrigin}`,
    releaseInputs: () => {
      manualEnabled = false;
      leaderSampleAt = 0;
      pendingAction = null;
      void stopWebcam();
      void leader.disconnect().then(() => {
        byId("leader-status").textContent = "Leader disconnected";
        refreshControls();
      });
      byId("manual-status").textContent = "Slider input off";
    },
    onState: (state) => {
      const messages = {
        owner: `This tab controls the local simulation · UDP ${config.udp_port}`,
        connecting: "Connecting to the local input relay…",
        disconnected: "Relay connection lost. Reconnecting automatically; webcam and leader are off.",
        superseded: "A newer controller tab has taken over. Webcam and leader released. This tab will not reconnect; reload it only to take control again.",
        stopped: "Controller stopped. Webcam and leader released.",
      };
      byId("relay-status").textContent = messages[state];
      refreshControls();
      if (state === "owner") {
        void fetch("/config").then((response) => response.json()).then((value) => {
          config = value; byId("udp-target").textContent = `127.0.0.1:${config.udp_port}`;
        }).catch(() => {});
      }
    },
    onMessage: (message) => { if (message.error) byId("relay-status").textContent = message.error; },
  });

  function refreshControls() {
    const connected = relay.connected && sceneConnected;
    const serialBusy = Boolean(leader.port || leader.stopping || leader.connecting);
    byId("leader-connect").disabled = !connected || !calibration || serialBusy || !("serial" in navigator);
    byId("leader-disconnect").disabled = !serialBusy;
    byId("leader-profile").disabled = serialBusy;
    byId("manual-enable").disabled = !connected || serialBusy || manualEnabled;
    byId("manual-stop").disabled = !manualEnabled;
    for (const input of sliderInputs) input.disabled = !connected || !manualEnabled || serialBusy;
    byId("head-start").disabled = !connected || webcamEnabled || webcamPending;
    byId("head-stop").disabled = !webcamEnabled && !webcamPending;
    byId("webcam-device").disabled = webcamEnabled || webcamPending;
    for (const id of ["head-recenter", "scene-reset", "scene-replay", "render-solid", "render-cloud", "view-front", "view-side"]) {
      if (byId(id)) byId(id).disabled = !connected;
    }
  }

  function sendPacket() {
    if (!relay.connected || !sceneConnected) return;
    const head = window.godotWebcamTrackerLatest || {};
    const headActive = webcamEnabled && head.active === true && Date.now() - Number(head.sent_unix_ms || 0) < 250;
    const leaderActive = leader.running && leaderSampleAt > 0 && performance.now() - leaderSampleAt < 250;
    const pose = leader.port ? leaderPose : manualPose;
    const packet = {
      type: "alignment_demo", version: 1, seq: seq++, timestamp_ms: Date.now(), source,
      head: {
        active: headActive, units: "cm",
        x: headActive ? Math.max(-100, Math.min(100, head.x)) : 0,
        y: headActive ? Math.max(-100, Math.min(100, head.y)) : 0,
        z: headActive ? Math.max(0, Math.min(500, head.z)) : 35,
      },
      arm: { active: leader.port ? leaderActive : manualEnabled, source: leader.port ? "leader" : "manual", normalized: pose },
    };
    if (pendingAction) packet.action = pendingAction;
    if (relay.send(packet)) pendingAction = null;
    if (webcamEnabled) byId("head-status").textContent = headActive
      ? `Tracking · x ${head.x.toFixed(1)} / y ${head.y.toFixed(1)} / z ${head.z.toFixed(1)} cm`
      : head.status || "Looking for your face…";
  }

  async function stopWebcam() {
    headGeneration += 1;
    const wasPending = webcamPending;
    webcamPending = false;
    webcamEnabled = false;
    window.stopGodotWebcamTracker?.();
    byId("head-status").textContent = wasPending
      ? "Stopping pending webcam request… close any camera prompt if it is still open."
      : "Webcam off";
    refreshControls();
  }

  byId("head-start").addEventListener("click", async () => {
    if (!relay.connected || !sceneConnected) return;
    const generation = ++headGeneration;
    webcamPending = true;
    byId("head-status").textContent = "Requesting webcam; loading face tracker…";
    refreshControls();
    try {
      await import("/example_webcam_tracker_bridge.js");
      if (generation !== headGeneration) return;
      await window.startGodotWebcamTracker({
        backend: "mediapipe", remoteEnabled: false, deviceId: byId("webcam-device").value,
        preferredLabel: "", preview: false, debug: false, showVideo: true,
        videoContainerId: "webcam-preview", showFaceOverlay: false,
        neutralZCm: 35, smoothing: 0.35, trackingFps: 30, holdLastPoseMs: 200,
      });
      if (generation !== headGeneration) return;
      webcamEnabled = true;
      pendingAction = "recenter";
      const devices = await window.listGodotWebcamTrackerDevices();
      if (generation !== headGeneration) return;
      const selected = byId("webcam-device").value;
      byId("webcam-device").replaceChildren(new Option("Default user-facing webcam", ""));
      for (const device of devices) byId("webcam-device").add(new Option(device.label || "Webcam", device.deviceId));
      byId("webcam-device").value = selected;
    } catch (error) {
      if (generation === headGeneration) {
        window.stopGodotWebcamTracker?.();
        webcamEnabled = false;
        byId("head-status").textContent = `Webcam stopped: ${error.message}`;
      }
    } finally {
      if (generation === headGeneration) webcamPending = false;
      refreshControls();
    }
  });
  byId("head-stop").addEventListener("click", stopWebcam);
  byId("head-recenter").addEventListener("click", () => { pendingAction = "recenter"; sendPacket(); });
  byId("leader-profile").addEventListener("change", async (event) => {
    calibration = null;
    try {
      const file = event.target.files[0];
      if (!file || file.size > 128000) throw new Error("Choose a small calibration JSON file");
      calibration = parseLeaderCalibration(JSON.parse(await file.text()));
      byId("profile-status").textContent = "Leader calibration loaded locally. Device identity and follower settings are not sent.";
    } catch (error) { byId("profile-status").textContent = error.message; }
    refreshControls();
  });
  byId("leader-connect").addEventListener("click", async () => {
    if (!relay.connected || !sceneConnected) return;
    manualEnabled = false;
    leaderSampleAt = 0;
    byId("leader-connect").disabled = true;
    try { await leader.connect(); } catch (error) {
      await leader.disconnect();
      byId("leader-status").textContent = `Not connected: ${error.message}`;
    }
    refreshControls();
  });
  byId("leader-disconnect").addEventListener("click", async () => {
    leaderSampleAt = 0;
    await leader.disconnect();
    byId("leader-status").textContent = "Leader disconnected";
    refreshControls();
  });
  jointDefinitions.forEach(([name, minimum, maximum], index) => {
    const label = document.createElement("label"); label.className = "slider";
    const text = document.createElement("span"); text.textContent = name;
    const input = document.createElement("input"); input.type = "range"; input.min = minimum; input.max = maximum; input.step = 0.5; input.value = manualPose[index]; input.disabled = true;
    const output = document.createElement("output"); output.textContent = `${manualPose[index].toFixed(1)}${index === 5 ? "%" : "°"}`;
    input.addEventListener("input", () => { manualPose[index] = Number(input.value); output.textContent = `${manualPose[index].toFixed(1)}${index === 5 ? "%" : "°"}`; });
    label.append(text, input, output); byId("sliders").append(label); sliderInputs.push(input);
  });
  byId("manual-enable").addEventListener("click", () => { if (!relay.connected || !sceneConnected) return; manualEnabled = true; byId("manual-status").textContent = "Sliders controlling virtual joints"; refreshControls(); });
  byId("manual-stop").addEventListener("click", () => { manualEnabled = false; byId("manual-status").textContent = "Slider input off"; refreshControls(); });
  byId("scene-reset").addEventListener("click", () => { pendingAction = "reset"; sendPacket(); });
  byId("scene-replay").addEventListener("click", async () => {
    manualEnabled = false;
    leaderSampleAt = 0;
    if (leader.port) {
      await leader.disconnect();
      byId("leader-status").textContent = "Leader disconnected for replay";
    }
    pendingAction = "replay";
    sendPacket(); refreshControls();
    byId("manual-status").textContent = "Replay requested in Godot";
  });
  const timer = setInterval(sendPacket, 1000 / TARGET_HZ);
  const refreshTimer = setInterval(refreshControls, 500);
  let previewBusy = false;
  let pageStopped = false;
  let previewUrl = null;
  async function refreshPreview() {
    if (previewBusy || document.hidden || pageStopped || !byId("simulation-preview")) return;
    previewBusy = true;
    try {
      const response = await fetch("/preview-state.json", {signal: AbortSignal.timeout(2000)});
      if (!response.ok) throw new Error("Scene preview is not running");
      const state = await response.json();
      if (!state.running || Date.now() - state.timestamp_ms > 2000) throw new Error("Scene preview is stale");
      sceneConnected = sceneInputReady(state, config.udp_port);
      refreshControls();
      if (pageStopped) return;
      const frame = await fetch(`/preview.jpg?frame=${state.timestamp_ms}`, {signal: AbortSignal.timeout(2000)});
      if (!frame.ok) throw new Error("Preview frame unavailable");
      const nextUrl = URL.createObjectURL(await frame.blob());
      const decoded = new Image();
      decoded.src = nextUrl;
      try { await decoded.decode(); } catch (error) { URL.revokeObjectURL(nextUrl); throw error; }
      if (pageStopped) { URL.revokeObjectURL(nextUrl); return; }
      const image = byId("simulation-preview");
      image.src = nextUrl; image.hidden = false;
      if (previewUrl) URL.revokeObjectURL(previewUrl);
      previewUrl = nextUrl;
      byId("render-solid")?.setAttribute("aria-pressed", String(state.render_mode === "solid"));
      byId("render-cloud")?.setAttribute("aria-pressed", String(state.render_mode === "point_cloud"));
      byId("simulation-status").textContent = sceneConnected ? "Live Godot simulation" : "Godot is running; controller input is not connected";
      byId("simulation-detail").textContent = sceneConnected
        ? `${state.render_mode === "point_cloud" ? "Point cloud" : "Solid geometry"} · ${state.replay ? "replay" : state.input_mode || "manual"} · ${state.held ? "holding object" : state.completed ? "task complete" : "object released"}${state.head_active ? " · head tracking active" : ""}`
        : `The scene must listen on local UDP ${config.udp_port}. Restart it with the Windows launcher to enable these controls.`;
    } catch (_) {
      if (!pageStopped) {
        sceneConnected = false;
        refreshControls();
        byId("simulation-preview").hidden = true;
        byId("simulation-status").textContent = "Scene preview unavailable";
        byId("simulation-detail").textContent = "Start the standalone demo with the Windows launcher. Input relay connectivity alone does not confirm a running scene.";
      }
    } finally { previewBusy = false; }
  }
  for (const [id, action] of [["render-solid", "render_solid"], ["render-cloud", "render_cloud"], ["view-front", "view_front"], ["view-side", "view_side"]]) {
    byId(id)?.addEventListener("click", () => { if (relay.connected && sceneConnected) { pendingAction = action; sendPacket(); } });
  }
  const previewTimer = setInterval(refreshPreview, 100);
  const recover = () => { if (!document.hidden) { relay.recover(); void refreshPreview(); } };
  window.addEventListener("focus", recover);
  window.addEventListener("online", recover);
  document.addEventListener("visibilitychange", recover);
  window.addEventListener("pagehide", () => {
    pageStopped = true;
    if (previewUrl) URL.revokeObjectURL(previewUrl);
    clearInterval(timer); clearInterval(refreshTimer); clearInterval(previewTimer);
    manualEnabled = false; webcamEnabled = false; leaderSampleAt = 0;
    pendingAction = null; sendPacket(); relay.stop();
  });
  window.addEventListener("pageshow", (event) => {
    // A browser-back restore has already released its devices on pagehide.
    // Treat the user's navigation as a fresh page, with no device autostart.
    if (event.persisted) location.reload();
  });
  if (!("serial" in navigator)) byId("leader-status").textContent = "Web Serial unavailable. Open this localhost page in desktop Chrome or Edge.";
  refreshControls(); relay.connect(); void refreshPreview();
}

if (typeof document !== "undefined") {
  startController().catch((error) => {
    document.getElementById("relay-status").textContent = `Controller unavailable: ${error.message}`;
  });
}
