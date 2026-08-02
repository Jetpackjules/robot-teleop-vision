const USB_VENDOR_ID = 0x1a86;
const USB_PRODUCT_ID = 0x55d3;
const BAUD_RATE = 1_000_000;
const MOTOR_IDS = [1, 2, 3, 4, 5, 6];
const PRESENT_POSITION_ADDRESS = 56;
const POSITION_BYTES = 2;
const TARGET_HZ = 30;
const SERIAL_RESPONSE_TIMEOUT_MS = 100;
const KEYBOARD_CODES = new Set([
  "KeyW", "KeyA", "KeyS", "KeyD", "KeyR", "KeyF", "KeyQ", "KeyE",
  "KeyI", "KeyJ", "KeyK", "KeyL",
  "ShiftLeft", "ShiftRight",
]);
const ARM_MOTION_CODES = new Set([
  "KeyW", "KeyA", "KeyS", "KeyD", "KeyR", "KeyF",
  "KeyI", "KeyJ", "KeyK", "KeyL",
]);

function isTextEditingTarget(target) {
  if (!(target instanceof HTMLElement)) return false;
  if (target.isContentEditable || target.tagName === "TEXTAREA") return true;
  if (target.tagName !== "INPUT") return false;
  return ["text", "search", "email", "password", "url", "tel", "number"].includes(target.type);
}

function armSocketUrl() {
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  const client = encodeURIComponent(String(window.robotTeleopPageId || ""));
  return `${scheme}://${window.location.host}/arm-control?client=${client}`;
}

function packetChecksum(bytes, endExclusive) {
  let sum = 0;
  for (let i = 2; i < endExclusive; i += 1) sum = (sum + bytes[i]) & 0xff;
  return (~sum) & 0xff;
}

function makeSyncReadPacket() {
  const params = [PRESENT_POSITION_ADDRESS, POSITION_BYTES, ...MOTOR_IDS];
  const packet = new Uint8Array(6 + params.length);
  packet[0] = 0xff;
  packet[1] = 0xff;
  packet[2] = 0xfe;
  packet[3] = params.length + 2;
  packet[4] = 0x82;
  packet.set(params, 5);
  packet[packet.length - 1] = packetChecksum(packet, packet.length - 1);
  return packet;
}

function appendBytes(first, second) {
  const joined = new Uint8Array(first.length + second.length);
  joined.set(first, 0);
  joined.set(second, first.length);
  return joined;
}

export class So101ArmController extends EventTarget {
  constructor() {
    super();
    this.port = null;
    this.reader = null;
    this.reading = false;
    this.rxBuffer = new Uint8Array(0);
    this.responses = new Map();
    this.lastLeaderPositions = null;
    this.socket = null;
    this.socketReconnectTimer = 0;
    this.socketHeartbeatTimer = 0;
    this.serialReconnectTimer = 0;
    this.loopTimer = 0;
    this.keyboardTimer = 0;
    this.keyboardKeys = new Set();
    this.keyboardKeyOrder = new Map();
    this.keyboardKeySequence = 0;
    this.resumeKeyboardAfterTransportRecovery = false;
    this.gripperContactLatched = false;
    this.recoveringGripperContact = false;
    this.armContactLatched = false;
    this.recoveringArmContact = false;
    this.seq = 0;
    this.lastLoopMs = 0;
    this.sendTimes = [];
    this.latest = {
      supported: "serial" in navigator,
      leaderConnected: false,
      leaderState: "not connected",
      leaderPositions: null,
      leaderHz: 0,
      keyboardConnected: false,
      keyboardDeadman: false,
      keyboardFrame: "base",
      controlMode: "none",
      serverConnected: false,
      followerConnected: false,
      armed: false,
      torqueEnabled: false,
      state: "offline",
      fault: "",
      restartRequested: false,
      status: null,
    };
    this.feedbackSettings = {
      measured_feedback_enabled: true,
      target_ghost_enabled: true,
      following_error_safety_enabled: true,
      freeze_overlay_on_stale_enabled: true,
      d455_visual_correction_enabled: false,
    };
    this.idleReturnEnabled = false;
    this.connectArmSocket();
    this.onKeyboardDown = (event) => this.handleKeyboardEvent(event, true);
    this.onKeyboardUp = (event) => this.handleKeyboardEvent(event, false);
    window.addEventListener("keydown", this.onKeyboardDown, { capture: true });
    window.addEventListener("keyup", this.onKeyboardUp, { capture: true });
    window.addEventListener("blur", () => this.clearKeyboardMotion());
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) this.clearKeyboardMotion();
    });
    if ("serial" in navigator) {
      navigator.serial.addEventListener("connect", () => this.scheduleApprovedLeaderReconnect(150));
      navigator.serial.addEventListener("disconnect", (event) => {
        if (event.target === this.port) this.serialFailed(new Error("leader disconnected"));
      });
    }
  }

  emitStatus() {
    const guide = document.getElementById("keyboard-controls-guide");
    if (guide) {
      guide.hidden = this.latest.keyboardConnected !== true;
      guide.style.display = this.latest.keyboardConnected === true ? "block" : "none";
    }
    this.dispatchEvent(new CustomEvent("status", { detail: { ...this.latest } }));
    window.dispatchEvent(new CustomEvent("so101-arm-status", { detail: { ...this.latest } }));
  }

  connectArmSocket() {
    if (this.socket && [WebSocket.CONNECTING, WebSocket.OPEN].includes(this.socket.readyState)) return;
    window.clearTimeout(this.socketReconnectTimer);
    const socket = new WebSocket(armSocketUrl());
    this.socket = socket;
    socket.addEventListener("open", () => {
      if (this.socket !== socket) return;
      this.latest.serverConnected = true;
      this.latest.fault = "";
      const resumeKeyboard = this.resumeKeyboardAfterTransportRecovery
        && this.latest.keyboardConnected;
      this.resumeKeyboardAfterTransportRecovery = false;
      this.emitStatus();
      this.send({ type: "arm_status_request" });
      this.sendFeedbackSettings();
      this.sendIdleReturnSettings();
      if (resumeKeyboard) {
        // Establish a fresh browser/bridge clock baseline, then restore the
        // same keyboard deadman session after the old bridge's latency gate
        // forced a transient Hold.
        this.sendKeyboardIntent();
        this.send({ type: "arm_enable", source: "keyboard" });
      }
      window.clearInterval(this.socketHeartbeatTimer);
      this.socketHeartbeatTimer = window.setInterval(() => this.send({ type: "arm_status_request" }), 500);
    });
    socket.addEventListener("message", (event) => {
      if (this.socket !== socket) return;
      try {
        const status = JSON.parse(event.data);
        if (status.type !== "arm_status") return;
        if (
          this.latest.keyboardConnected
          && String(status.fault || "").includes("arm command transport delay is older than 250 ms")
        ) {
          this.resumeKeyboardAfterTransportRecovery = true;
          this.latest.fault = "Recovering keyboard control after a transient network delay…";
          this.emitStatus();
          if (socket.readyState <= WebSocket.OPEN) {
            socket.close(1012, "refreshing arm transport clock");
          }
          return;
        }
        const oldFollowerGripperContactHold =
          this.latest.keyboardConnected
          && status.state === "hold"
          && String(status.message || "").includes(
            "Contact / following error stopped motion at servo 6",
          );
        if (oldFollowerGripperContactHold && !this.recoveringGripperContact) {
          // Compatibility recovery for a follower process that predates the
          // gripper-only contact behavior.  Suppress a still-held close key
          // before re-enabling so contact cannot create a re-arm loop.
          this.recoveringGripperContact = true;
          this.gripperContactLatched = this.keyboardKeys.has("KeyE");
          this.sendKeyboardIntent();
          this.send({ type: "arm_enable", source: "keyboard" });
        } else if (status.state === "armed") {
          this.recoveringGripperContact = false;
        }
        const oldFollowerArmContactHold =
          this.latest.keyboardConnected
          && status.state === "hold"
          && /Contact \/ following error stopped motion at servo [1-5]\b/.test(
            String(status.message || ""),
          );
        if (oldFollowerArmContactHold && !this.recoveringArmContact) {
          // Stop remains fail-safe, but a fixed-threshold contact false
          // positive must not cancel the browser's control session.  Re-arm
          // with zero arm motion and require release/repress before moving.
          this.recoveringArmContact = true;
          this.armContactLatched = true;
          this.sendKeyboardIntent();
          this.send({ type: "arm_enable", source: "keyboard" });
        } else if (status.state === "armed") {
          this.recoveringArmContact = false;
        }
        const oldFollowerIkGuardHold =
          this.latest.keyboardConnected
          && status.state === "hold"
          && (
            String(status.message || "").startsWith("IK workspace boundary stopped motion")
            || String(status.message || "").startsWith("Modeled arm clearance stopped motion")
          );
        if (oldFollowerIkGuardHold && !this.recoveringArmContact) {
          this.recoveringArmContact = true;
          this.armContactLatched = true;
          this.sendKeyboardIntent();
          this.send({ type: "arm_enable", source: "keyboard" });
        }
        this.latest.status = status;
        this.latest.followerConnected = status.follower_connected === true;
        this.latest.armed = status.armed === true;
        this.latest.torqueEnabled = status.torque_enabled === true;
        this.latest.state = status.state || "unknown";
        this.latest.fault = status.fault || "";
        this.latest.restartRequested = status.state === "restarting";
        this.emitStatus();
      } catch (_) {}
    });
    let disconnectedHandled = false;
    const disconnected = (event) => {
      if (disconnectedHandled) return;
      disconnectedHandled = true;
      if (this.socket !== socket) return;
      window.clearInterval(this.socketHeartbeatTimer);
      this.socketHeartbeatTimer = 0;
      this.latest.serverConnected = false;
      this.latest.armed = false;
      this.latest.state = "disconnected";
      if (event instanceof CloseEvent && event.code === 4001) {
        this.latest.fault = "This viewer was superseded by a newer browser.";
        this.emitStatus();
        return;
      }
      this.emitStatus();
      const reconnectDelay = this.resumeKeyboardAfterTransportRecovery ? 100 : 1500;
      this.socketReconnectTimer = window.setTimeout(
        () => this.connectArmSocket(),
        reconnectDelay,
      );
    };
    socket.addEventListener("close", disconnected);
    socket.addEventListener("error", () => {
      // A WebSocket error is followed by close; let close decide whether this
      // page may reconnect (superseded pages must remain disconnected).
    }, { once: true });
  }

  send(message) {
    if (!this.socket || this.socket.readyState !== WebSocket.OPEN) return false;
    if (this.socket.bufferedAmount > 32 * 1024) return false;
    this.socket.send(JSON.stringify(message));
    return true;
  }

  setFeedbackSettings(settings = {}) {
    for (const key of Object.keys(this.feedbackSettings)) {
      if (typeof settings[key] === "boolean") this.feedbackSettings[key] = settings[key];
    }
    return this.sendFeedbackSettings();
  }

  sendFeedbackSettings() {
    return this.send({ type: "arm_feedback_settings", ...this.feedbackSettings });
  }

  async autoConnectApprovedLeader() {
    if (!("serial" in navigator) || this.port || this.latest.keyboardConnected) return false;
    const ports = await navigator.serial.getPorts();
    const approved = ports.filter((port) => {
      const info = port.getInfo();
      return info.usbVendorId === USB_VENDOR_ID && info.usbProductId === USB_PRODUCT_ID;
    });
    if (approved.length !== 1) return false;
    await this.openLeader(approved[0]);
    return true;
  }

  scheduleApprovedLeaderReconnect(delayMs = 500) {
    if (this.latest.leaderConnected || this.latest.keyboardConnected || this.port || !("serial" in navigator)) return;
    window.clearTimeout(this.serialReconnectTimer);
    this.serialReconnectTimer = window.setTimeout(async () => {
      try {
        const connected = await this.autoConnectApprovedLeader();
        if (!connected) this.scheduleApprovedLeaderReconnect(1500);
      } catch (error) {
        this.latest.leaderState = "waiting for serial port release";
        this.latest.fault = `leader reopen: ${error && error.message ? error.message : error}`;
        this.emitStatus();
        this.scheduleApprovedLeaderReconnect(1500);
      }
    }, delayMs);
  }

  async connectLeader() {
    if (!("serial" in navigator)) throw new Error("Web Serial is unavailable; use desktop Chrome or Edge over HTTPS");
    this.disconnectKeyboard(false);
    const port = await navigator.serial.requestPort({
      filters: [{ usbVendorId: USB_VENDOR_ID, usbProductId: USB_PRODUCT_ID }],
    });
    await this.openLeader(port);
  }

  async openLeader(port) {
    if (this.port === port && this.latest.leaderConnected) return;
    await this.disconnectLeader(false);
    this.latest.leaderState = "opening";
    this.emitStatus();
    await port.open({ baudRate: BAUD_RATE, bufferSize: 4096 });
    window.clearTimeout(this.serialReconnectTimer);
    this.serialReconnectTimer = 0;
    this.port = port;
    this.reading = true;
    this.latest.leaderConnected = true;
    this.latest.controlMode = "leader";
    this.latest.leaderState = "reading motors 1-6";
    this.readPump().catch((error) => this.serialFailed(error));
    this.scheduleLoop(0);
    this.emitStatus();
  }

  async disconnectLeader(sendHold = true) {
    window.clearTimeout(this.loopTimer);
    this.loopTimer = 0;
    this.reading = false;
    if (sendHold) this.hold();
    if (this.reader) {
      try { await this.reader.cancel(); } catch (_) {}
      try { this.reader.releaseLock(); } catch (_) {}
      this.reader = null;
    }
    if (this.port) {
      try { await this.port.close(); } catch (_) {}
    }
    this.port = null;
    this.latest.leaderConnected = false;
    if (this.latest.controlMode === "leader") this.latest.controlMode = "none";
    this.latest.leaderState = "not connected";
    this.emitStatus();
  }

  async readPump() {
    if (!this.port || !this.port.readable) throw new Error("leader serial stream is unavailable");
    const reader = this.port.readable.getReader();
    this.reader = reader;
    try {
      while (this.reading) {
        const { value, done } = await reader.read();
        if (done) break;
        if (!value || !value.length) continue;
        this.rxBuffer = appendBytes(this.rxBuffer, value);
        this.parsePackets();
      }
    } finally {
      try { reader.releaseLock(); } catch (_) {}
      if (this.reader === reader) this.reader = null;
    }
  }

  parsePackets() {
    while (this.rxBuffer.length >= 6) {
      let header = -1;
      for (let i = 0; i < this.rxBuffer.length - 1; i += 1) {
        if (this.rxBuffer[i] === 0xff && this.rxBuffer[i + 1] === 0xff) {
          header = i;
          break;
        }
      }
      if (header < 0) {
        this.rxBuffer = this.rxBuffer.slice(-1);
        return;
      }
      if (header > 0) this.rxBuffer = this.rxBuffer.slice(header);
      if (this.rxBuffer.length < 4) return;
      const total = this.rxBuffer[3] + 4;
      if (total < 6 || total > 64) {
        this.rxBuffer = this.rxBuffer.slice(1);
        continue;
      }
      if (this.rxBuffer.length < total) return;
      const packet = this.rxBuffer.slice(0, total);
      this.rxBuffer = this.rxBuffer.slice(total);
      if (packetChecksum(packet, total - 1) !== packet[total - 1]) continue;
      const id = packet[2];
      if (!MOTOR_IDS.includes(id) || packet[3] < 4 || packet[4] !== 0) continue;
      this.responses.set(id, packet[5] | (packet[6] << 8));
    }
  }

  scheduleLoop(delayMs) {
    window.clearTimeout(this.loopTimer);
    this.loopTimer = window.setTimeout(() => this.pollLeader(), delayMs);
  }

  async pollLeader() {
    if (!this.port || !this.latest.leaderConnected) return;
    const started = performance.now();
    this.responses.clear();
    try {
      const writer = this.port.writable.getWriter();
      try {
        await writer.write(makeSyncReadPacket());
      } finally {
        writer.releaseLock();
      }
      // MediaPipe inference shares the browser main thread. Bytes still arrive while
      // inference runs, so allow its callback to drain them before declaring a partial read.
      const deadline = performance.now() + SERIAL_RESPONSE_TIMEOUT_MS;
      while (this.responses.size < MOTOR_IDS.length && performance.now() < deadline) {
        await new Promise((resolve) => setTimeout(resolve, 1));
      }
      if (this.responses.size === MOTOR_IDS.length || (this.responses.size >= 5 && this.lastLeaderPositions)) {
        const positions = MOTOR_IDS.map((id, index) => (
          this.responses.has(id) ? this.responses.get(id) : this.lastLeaderPositions[index]
        ));
        this.lastLeaderPositions = positions;
        const now = Date.now();
        this.latest.leaderPositions = positions;
        this.latest.leaderState = this.responses.size === MOTOR_IDS.length
          ? "streaming"
          : `streaming degraded ${this.responses.size}/6`;
        this.sendTimes.push(now);
        while (this.sendTimes.length && now - this.sendTimes[0] > 1000) this.sendTimes.shift();
        this.latest.leaderHz = this.sendTimes.length;
        this.send({ type: "arm_command", seq: this.seq++, sent_unix_ms: now, positions });
      } else {
        this.latest.leaderState = `partial read ${this.responses.size}/6`;
      }
      this.emitStatus();
    } catch (error) {
      await this.serialFailed(error);
      return;
    }
    const period = 1000 / TARGET_HZ;
    this.lastLoopMs = performance.now() - started;
    this.scheduleLoop(Math.max(0, period - this.lastLoopMs));
  }

  enableArm() {
    const source = this.latest.keyboardConnected ? "keyboard" : "leader";
    if (source === "leader" && !this.latest.leaderConnected) throw new Error("Connect the leader arm or enable keyboard control first");
    if (!this.latest.serverConnected) throw new Error("Arm control server is not connected");
    if (source === "keyboard") this.sendKeyboardIntent();
    this.send({ type: "arm_enable", source });
  }

  hold() {
    this.keyboardKeys.clear();
    this.keyboardKeyOrder.clear();
    this.latest.keyboardDeadman = false;
    this.send({ type: "arm_hold" });
    this.emitStatus();
  }

  returnToRest() {
    if (!this.latest.serverConnected) throw new Error("Arm control server is not connected");
    this.keyboardKeys.clear();
    this.keyboardKeyOrder.clear();
    this.latest.keyboardDeadman = false;
    if (!this.send({ type: "arm_return_to_rest" })) {
      throw new Error("Could not send return-to-rest request");
    }
    this.emitStatus();
  }

  setIdleReturn(enabled, timeoutSeconds = 600) {
    this.idleReturnEnabled = enabled === true;
    this.idleReturnTimeoutSeconds = timeoutSeconds;
    return this.sendIdleReturnSettings();
  }

  sendIdleReturnSettings() {
    if (!this.latest.serverConnected) return false;
    return this.send({
      type: "arm_idle_return_settings",
      enabled: this.idleReturnEnabled === true,
      timeout_seconds: this.idleReturnTimeoutSeconds || 600,
    });
  }

  async connectKeyboard() {
    if (this.latest.leaderConnected || this.port) await this.disconnectLeader(true);
    if (document.activeElement instanceof HTMLElement) document.activeElement.blur();
    this.latest.keyboardConnected = true;
    this.latest.keyboardDeadman = true;
    this.latest.controlMode = "keyboard";
    this.latest.leaderState = "keyboard Cartesian claw control";
    window.clearInterval(this.keyboardTimer);
    this.keyboardTimer = window.setInterval(() => this.sendKeyboardIntent(), 1000 / TARGET_HZ);
    this.sendKeyboardIntent();
    this.emitStatus();
  }

  disconnectKeyboard(sendHold = true) {
    window.clearInterval(this.keyboardTimer);
    this.keyboardTimer = 0;
    this.keyboardKeys.clear();
    this.keyboardKeyOrder.clear();
    this.latest.keyboardConnected = false;
    this.latest.keyboardDeadman = false;
    if (this.latest.controlMode === "keyboard") this.latest.controlMode = "none";
    if (sendHold) this.hold();
    this.emitStatus();
  }

  handleKeyboardEvent(event, pressed) {
    if (!this.latest.keyboardConnected || !KEYBOARD_CODES.has(event.code)) return;
    if (isTextEditingTarget(event.target)) return;
    event.preventDefault();
    event.stopPropagation();
    event.stopImmediatePropagation();
    if (pressed) {
      this.keyboardKeys.add(event.code);
      if (!event.repeat) this.keyboardKeyOrder.set(event.code, ++this.keyboardKeySequence);
    } else {
      this.keyboardKeys.delete(event.code);
      this.keyboardKeyOrder.delete(event.code);
      if (event.code === "KeyE") this.gripperContactLatched = false;
      if (![...ARM_MOTION_CODES].some((code) => this.keyboardKeys.has(code))) {
        this.armContactLatched = false;
      }
    }
    this.latest.keyboardDeadman = true;
    this.emitStatus();
  }

  clearKeyboardMotion() {
    if (!this.latest.keyboardConnected) return;
    this.keyboardKeys.clear();
    this.keyboardKeyOrder.clear();
    this.armContactLatched = false;
    this.gripperContactLatched = false;
    this.latest.keyboardDeadman = false;
    this.sendKeyboardIntent(true);
    this.emitStatus();
  }

  keyboardAxis(positive, negative) {
    const positivePressed = this.keyboardKeys.has(positive);
    const negativePressed = this.keyboardKeys.has(negative);
    if (positivePressed && negativePressed) {
      return (this.keyboardKeyOrder.get(positive) || 0) >= (this.keyboardKeyOrder.get(negative) || 0) ? 1 : -1;
    }
    return (positivePressed ? 1 : 0) - (negativePressed ? 1 : 0);
  }

  sendKeyboardIntent(forceIdle = false) {
    if (!this.latest.keyboardConnected) return false;
    const inputEnabled = !forceIdle;
    this.latest.keyboardDeadman = inputEnabled;
    const armInputEnabled = inputEnabled && !this.armContactLatched;
    const right = armInputEnabled ? this.keyboardAxis("KeyD", "KeyA") : 0;
    const forward = armInputEnabled ? this.keyboardAxis("KeyW", "KeyS") : 0;
    const up = armInputEnabled ? this.keyboardAxis("KeyR", "KeyF") : 0;
    const requestedGripper = inputEnabled ? this.keyboardAxis("KeyQ", "KeyE") : 0;
    const gripper = this.gripperContactLatched && requestedGripper < 0
      ? 0
      : requestedGripper;
    const wristPitch = armInputEnabled ? this.keyboardAxis("KeyI", "KeyK") : 0;
    const wristRoll = armInputEnabled ? this.keyboardAxis("KeyL", "KeyJ") : 0;
    const common = {
      seq: this.seq++,
      sent_unix_ms: Date.now(),
      precision: this.keyboardKeys.has("ShiftLeft") || this.keyboardKeys.has("ShiftRight"),
      deadman: inputEnabled,
    };
    return this.send({
      ...common,
      type: "arm_cartesian_velocity",
      // The service captures a horizontal plane from the claw's actual radial
      // direction at enable time. This stays intuitive even when base yaw or
      // visual registration is not aligned with fixed robot X/Y.
      frame: "plane",
      linear: [right, forward, up],
      angular: [0, 0, 0],
      wrist: [wristPitch, wristRoll],
      gripper,
    });
  }

  restartFollower() {
    if (!this.latest.serverConnected) throw new Error("Arm control server is not connected");
    this.keyboardKeys.clear();
    this.keyboardKeyOrder.clear();
    this.latest.keyboardDeadman = this.latest.keyboardConnected;
    this.latest.restartRequested = true;
    this.latest.state = "restarting";
    this.latest.fault = "Follower hardware restart requested";
    if (!this.send({ type: "arm_restart" })) {
      this.latest.restartRequested = false;
      throw new Error("Could not send follower restart request");
    }
    this.emitStatus();
  }

  releaseTorque() {
    this.send({ type: "arm_release_torque" });
  }

  async serialFailed(error) {
    this.latest.fault = `leader serial: ${error && error.message ? error.message : error}`;
    await this.disconnectLeader(true);
    this.scheduleApprovedLeaderReconnect(1000);
  }

  shutdown() {
    window.clearTimeout(this.serialReconnectTimer);
    window.clearTimeout(this.socketReconnectTimer);
    window.clearInterval(this.socketHeartbeatTimer);
    this.hold();
    this.disconnectKeyboard(false);
    if (this.socket && this.socket.readyState <= WebSocket.OPEN) this.socket.close(1000, "page closing");
    this.disconnectLeader(false).catch(() => {});
  }
}

window.so101ArmController = window.so101ArmController || new So101ArmController();
window.getSo101ArmStatus = () => ({ ...window.so101ArmController.latest });

window.addEventListener("load", () => {
  window.so101ArmController.scheduleApprovedLeaderReconnect(100);
});
window.addEventListener("pagehide", () => window.so101ArmController.shutdown());
