const COUNTERS = ["ik_guard_trip_count", "following_error_trip_count", "gripper_contact_trip_count", "watchdog_hold_count"];

export class ArmNoticeDetector {
  constructor({connectionNotices = true} = {}) {
    this.connectionNotices = connectionNotices;
    this.previous = null;
    this.connected = false;
    this.seenConnection = false;
    this.previousFault = "";
  }
  update(arm = {}) {
    const status = arm.status || {}, previous = this.previous;
    const notices = [];
    const add = (id, title, detail, type = "safeguard", count = 1) => {
      if (type !== "connection" || this.connectionNotices) notices.push({id, title, detail, type, count});
    };
    const delta = name => previous && Number.isFinite(status[name]) && Number.isFinite(previous[name])
      ? Math.max(0, status[name] - previous[name]) : 0;
    const message = String(status.message || "");
    const floor = status.ik_clearance_limited === true || /Modeled (arm clearance|floor limit)/i.test(message);
    const ik = delta("ik_guard_trip_count");
    if (ik || (status.ik_clearance_limited && !previous?.ik_clearance_limited)) {
      add("clearance", floor ? "Claw/base clearance limit" : "IK workspace limit",
        floor ? "Claw motion below the base-model underside clipped; valid directions remain enabled."
          : "Requested motion reached the modeled workspace boundary.", "safeguard", ik || 1);
    }
    const following = delta("following_error_trip_count");
    if (following || (status.arm_contact_latched && !previous?.arm_contact_latched)) {
      add("following", "Following-error / contact stop",
        "A joint lagged its commanded target. Release movement keys before trying again.", "safeguard", following || 1);
    }
    const gripper = delta("gripper_contact_trip_count");
    if (gripper || (status.gripper_contact_latched && !previous?.gripper_contact_latched)) {
      add("gripper", "Gripper contact stop", "Claw closing stopped at contact; release close or command open.", "safeguard", gripper || 1);
    }
    const watchdog = delta("watchdog_hold_count");
    const timedOut = status.watchdog_holding === true || /watchdog expired|older than watchdog limit/i.test(message);
    const previouslyTimedOut = previous?.watchdog_holding === true || /watchdog expired|older than watchdog limit/i.test(previous?.message || "");
    if (watchdog || (timedOut && !previouslyTimedOut && status.state === "hold")) {
      add("watchdog", "Input timeout · watchdog", "Fresh control commands stopped arriving; holding the last target.", "connection", watchdog || 1);
    }
    const joints = Array.isArray(status.leader_limited_joints) ? status.leader_limited_joints : [];
    if (joints.length && joints.join() !== (previous?.leader_limited_joints || []).join()) {
      add("joint-limit", "Calibrated joint limit", "Joint travel limited. Reverse away from the boundary; other joints remain enabled.");
    }
    if (status.state === "hold" && message !== previous?.message && !ik && !following && !timedOut) {
      if (/measured servo telemetry became stale/i.test(message))
        add("servo-feedback", "Servo feedback safety stop", "Measured joint feedback became stale; motion stopped.");
      else if (/D455.*divergence/i.test(message))
        add("visual-contact", "Visual contact safeguard", "Camera and arm-model motion diverged; motion stopped.");
      else if (/missing arm target|bad local command/i.test(message))
        add("invalid-target", "Invalid control target", message.slice(0, 200));
      else if (!previous && /Modeled arm clearance|IK workspace boundary/.test(message))
        add("clearance", floor ? "Floor clearance limit" : "IK workspace limit", "Motion was limited by the modeled workspace boundary.");
    }
    const fault = String(status.hardware_fault || status.fault || arm.fault || "");
    if ((status.state === "fault" || arm.state === "fault") && fault && fault !== this.previousFault)
      add("arm-fault", "Arm controller fault", fault.slice(0, 200));
    if (status.state === "unresponsive" && previous?.state !== "unresponsive")
      add("feedback-link", "Arm status connection stalled", "Controller status stopped arriving. This is a connection notice.", "connection");
    const connected = arm.serverConnected === true;
    if (this.seenConnection && this.connected && !connected)
      add("connection", "Arm connection interrupted", "The browser lost its arm-control connection.", "connection");
    this.seenConnection ||= connected;
    this.connected = connected;
    this.previousFault = fault;
    if (status.follower_connected === true || COUNTERS.some(name => Number.isFinite(status[name])))
      this.previous = {...status, leader_limited_joints: joints.slice()};
    return notices;
  }
}

const STYLE = `
.so101-notices { position:fixed;right:20px;bottom:20px;z-index:50;display:flex;flex-direction:column;
  gap:8px;width:min(340px,calc(100vw - 40px));max-height:calc(100vh - 100px);pointer-events:none; }
.so101-notices[hidden] { display:none; }
.so101-notice { padding:12px 14px;background:var(--hold,#43292c);color:var(--text,#f0f3f6);
  border:1px solid var(--bad,#fa9b9b);border-radius:8px;
  opacity:1;transform:translateY(0);transition:opacity 180ms ease,transform 180ms ease; }
.so101-notice[data-type="connection"] { background:var(--panel,#14171b);border-color:var(--warn,#e6be71); }
.so101-notice-title { display:flex;justify-content:space-between;gap:12px;font:600 14px/1.35 system-ui,sans-serif; }
.so101-notice-kind { display:block;margin-top:4px;color:var(--bad,#fa9b9b);font:500 11px/1.4 system-ui,sans-serif; }
.so101-notice[data-type="connection"] .so101-notice-kind { color:var(--warn,#e6be71); }
.so101-notice-detail { margin:4px 0 0;font:400 12px/1.45 system-ui,sans-serif;overflow-wrap:anywhere; }
.so101-notice.leaving { opacity:0;transform:translateY(4px); }
body:has(#so101-wrist-camera-panel:not([hidden])) .so101-notices { bottom:calc(32vh + 72px); }
@media (prefers-reduced-motion:reduce) { .so101-notice { transition:none; } }
`;

export class ArmToastStack {
  constructor({document: doc = document, timers = globalThis, lifetime = 5000, maxVisible = 4, connectionNotices = true, enabled = true} = {}) {
    this.doc = doc; this.timers = timers; this.lifetime = lifetime; this.maxVisible = maxVisible;
    this.detector = new ArmNoticeDetector({connectionNotices}); this.items = new Map();
    this.style = doc.createElement("style"); this.style.textContent = STYLE; doc.head.append(this.style);
    this.root = doc.createElement("div"); this.root.className = "so101-notices";
    this.root.setAttribute("role", "log"); this.root.setAttribute("aria-label", "Robot safeguard and connection notices");
    this.root.setAttribute("aria-live", "polite"); this.root.setAttribute("aria-relevant", "additions");
    doc.body.append(this.root);
    this.setEnabled(enabled);
  }
  setEnabled(enabled) {
    if (this.disposed) return;
    this.enabled = enabled !== false;
    this.root.hidden = !this.enabled;
    this.root.setAttribute("aria-live", this.enabled ? "polite" : "off");
    if (!this.enabled) for (const id of [...this.items.keys()]) this.remove(id);
  }
  update(arm) {
    if (this.disposed) return;
    // Keep tracking the baseline while muted so old warnings are not replayed.
    const notices = this.detector.update(arm);
    if (this.enabled) for (const notice of notices) this.show(notice);
  }
  show(notice) {
    if (this.disposed || !this.enabled) return;
    let item = this.items.get(notice.id);
    if (!item) {
      while (this.items.size >= this.maxVisible) this.remove(this.items.keys().next().value);
      const element = this.doc.createElement("div"), title = this.doc.createElement("div");
      const label = this.doc.createElement("strong"), count = this.doc.createElement("span");
      const kind = this.doc.createElement("span"), detail = this.doc.createElement("p");
      element.className = "so101-notice"; element.dataset.type = notice.type;
      title.className = "so101-notice-title"; kind.className = "so101-notice-kind"; detail.className = "so101-notice-detail";
      title.append(label, count); element.append(title, kind, detail); this.root.append(element);
      item = {element, label, count, kind, detail, total: 0}; this.items.set(notice.id, item);
    }
    this.timers.clearTimeout(item.fade); this.timers.clearTimeout(item.remove);
    item.total += notice.count || 1; item.label.textContent = notice.title;
    item.count.textContent = item.total > 1 ? `×${item.total}` : "";
    item.kind.textContent = notice.type === "connection" ? "Input / connection" : "Safeguard";
    item.detail.textContent = notice.detail; item.element.classList.remove("leaving");
    item.fade = this.timers.setTimeout(() => {
      item.element.classList.add("leaving");
      item.remove = this.timers.setTimeout(() => this.remove(notice.id), 180);
    }, this.lifetime);
  }
  remove(id) {
    const item = this.items.get(id); if (!item) return;
    this.timers.clearTimeout(item.fade); this.timers.clearTimeout(item.remove);
    item.element.remove(); this.items.delete(id);
  }
  dispose() {
    this.disposed = true;
    for (const id of [...this.items.keys()]) this.remove(id);
    this.root.remove(); this.style.remove();
  }
}
