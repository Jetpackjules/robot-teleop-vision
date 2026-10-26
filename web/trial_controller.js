const CONDITIONS = Object.freeze({fixed_rgb: 'Fixed cameras', mouse_3d: 'Mouse 3D', head_3d: 'Head tracking'});
const DEFAULT_ORDER = Object.keys(CONDITIONS);
const RENDERERS = new Set(['point_cloud', 'shader_mesh', 'independent_mesh', 'unified_mesh', 'blendpcr']);
const STORAGE_KEY = 'robotTeleopTrialSession.v1';
const ACTIVE = new Set(['preparing', 'recording', 'saving']);
const byId = id => document.getElementById(id);
const formatTime = seconds => `${Math.floor(seconds / 60).toString().padStart(2, '0')}:${Math.floor(seconds % 60).toString().padStart(2, '0')}`;

/** A small facilitator layer over the existing view and host-recording controls. */
export class TrialController {
  constructor({recording, applyCondition, resetView, resetTracking, viewReady = () => true}) {
    this.recording = recording;
    this.applyView = applyCondition;
    this.resetView = resetView;
    this.resetTracking = resetTracking;
    this.viewReady = viewReady;
    this.display = byId('display-mode');
    this.head = byId('head-tracking-enabled');
    this.cameras = byId('rgb-camera-select');
    this.duration = byId('trial-duration');
    this.taskInput = byId('trial-task-input');
    this.panel = byId('advanced-setup');
    this.state = this.restore();
    this.pending = false;
    this.lastElapsed = 0;
    if (!Number.isSafeInteger(this.state.timeTargetSeconds) || this.state.timeTargetSeconds < 30 || this.state.timeTargetSeconds > 7200)
      this.state.timeTargetSeconds = 600;
    this.duration.value = String(this.state.timeTargetSeconds / 60);
    if (!RENDERERS.has(this.state.renderer)) this.state.renderer = this.display.value === 'rgb_camera' ? 'independent_mesh' : this.display.value;
    byId('guided-trials').checked = this.state.guided;
    byId('trial-order').value = this.state.order.join(',');
    if (this.state.task) recording.task.value = this.state.task;
    recording.context = () => {
      if (!this.state.guided) return null;
      if (!this.matchesView) throw new Error('Restore the trial view in Settings before recording.');
      if (this.state.finished || this.saved) throw new Error('This trial is saved. Continue with Next trial.');
      if (this.condition === 'head_3d' && !window.godotWebcamTrackerLatest?.active)
        throw new Error('Wait for head tracking before starting this trial.');
      if (!this.viewReady()) throw new Error('Wait for the live view before starting this trial.');
      return {session_id: this.state.id, index: this.state.index + 1, total: this.state.order.length,
        condition: this.condition, display_mode: this.display.value};
    };
    recording.root.addEventListener('recordingchange', () => this.render());
    recording.task.addEventListener('input', () => {this.state.task = recording.task.value; this.persist();});
    recording.outcome.addEventListener('change', () => this.render());
    byId('trial-outcome').addEventListener('change', event => {recording.outcome.value = event.target.value;});
    byId('trial-action').addEventListener('click', () => this.act());
    byId('trial-task-cancel').addEventListener('click', () => this.cancelTaskEdit());
    this.taskInput.addEventListener('keydown', event => {
      if (event.isComposing) return;
      if (event.key === 'Enter') { event.preventDefault(); this.saveTask(); }
      if (event.key === 'Escape') { event.preventDefault(); event.stopPropagation(); this.cancelTaskEdit(); }
    });
    byId('guided-trials').addEventListener('change', event => {
      this.state.guided = event.target.checked;
      this.persist();
      if (this.state.guided) this.apply(); else this.render();
    });
    byId('trial-order').addEventListener('change', event => {
      this.state.order = event.target.value.split(',');
      this.newSession();
    });
    this.duration.addEventListener('change', () => {
      if (!this.duration.reportValidity()) {
        this.duration.value = String(this.state.timeTargetSeconds / 60);
        return;
      }
      this.state.timeTargetSeconds = Math.round(this.duration.valueAsNumber * 60);
      this.persist(); this.render();
    });
    byId('new-trial-session').addEventListener('click', () => this.newSession());
    byId('restore-trial-view').addEventListener('click', () => this.apply());
    this.display.addEventListener('change', () => {
      if (this.display.value !== 'rgb_camera') this.state.renderer = this.display.value;
      this.persist(); this.render();
    });
    this.head.addEventListener('change', () => this.render());
    byId('previous-camera').addEventListener('click', () => this.switchCamera(-1));
    byId('next-camera').addEventListener('click', () => this.switchCamera(1));
    byId('reset-view').addEventListener('click', resetView);
    byId('reset-head-tracking').addEventListener('click', resetTracking);
    this.bindSettings();
    document.addEventListener('keydown', event => {
      if (!['ArrowLeft', 'ArrowRight'].includes(event.key) || event.defaultPrevented || event.isComposing
        || event.ctrlKey || event.metaKey || event.altKey || event.shiftKey || !this.panel.hidden
        || event.target.closest?.('input, textarea, select, [contenteditable="true"], [role="tab"]')
        || this.display.value !== 'rgb_camera') return;
      event.preventDefault();
      this.switchCamera(event.key === 'ArrowRight' ? 1 : -1);
    });
    this.timer = setInterval(() => this.render(), 250);
    window.addEventListener('pagehide', () => clearInterval(this.timer), {once: true});
    this.render();
  }

  get condition() { return this.state.order[this.state.index]; }
  get active() { return ACTIVE.has(this.recording.status?.state); }
  get matchesView() {
    return this.condition === 'fixed_rgb' ? this.display.value === 'rgb_camera' && !this.head.checked
      : this.display.value !== 'rgb_camera' && this.head.checked === (this.condition === 'head_3d');
  }
  get saved() {
    const saved = this.recording.status?.last_saved;
    return saved?.status === 'complete' && saved.trial?.session_id === this.state.id
      && saved.trial.index === this.state.index + 1 && saved.trial.condition === this.condition ? saved : null;
  }

  restore() {
    try {
      const state = JSON.parse(sessionStorage.getItem(STORAGE_KEY));
      if (state?.version === 1 && typeof state.id === 'string' && /^[A-Za-z0-9_-]{1,80}$/.test(state.id)
        && Array.isArray(state.order) && state.order.length === 3 && new Set(state.order).size === 3
        && state.order.every(key => Object.hasOwn(CONDITIONS, key)) && Number.isInteger(state.index) && state.index >= 0 && state.index < 3)
        return {...state, guided: state.guided !== false, completed: Array.isArray(state.completed) ? state.completed.slice(0, 3) : []};
    } catch { /* Storage may be disabled; the session can still run. */ }
    return {version: 1, id: this.sessionId(), index: 0, order: [...DEFAULT_ORDER], completed: [], guided: true, task: ''};
  }
  sessionId() { return window.crypto?.randomUUID?.() || `session-${Date.now()}-${Math.random().toString(16).slice(2)}`; }
  persist() { try { sessionStorage.setItem(STORAGE_KEY, JSON.stringify(this.state)); } catch { /* Optional. */ } }

  async initialize() { if (this.state.guided && !this.active) await this.apply(); }
  async apply() {
    if (this.active || this.recording.busy || this.pending) return;
    this.pending = true; this.viewError = ''; this.render();
    try { await this.applyView(this.condition, this.state.renderer); }
    catch (error) { this.viewError = error.message || 'The view could not start. Reconnect in Settings.'; }
    finally { this.pending = false; this.render(); }
  }
  async newSession() {
    if (this.active || this.recording.busy || this.pending) return;
    this.state.id = this.sessionId(); this.state.index = 0; this.state.completed = []; this.state.finished = false;
    this.lastElapsed = 0; this.persist();
    await this.apply();
  }
  async act() {
    if (this.recording.busy || this.pending) return;
    if (this.taskEditing) { this.saveTask(); return; }
    if (this.active) {
      if (this.recording.status?.state !== 'saving' && this.recording.status?.is_owner !== false)
        await this.recording.command('stop');
      return;
    }
    if (this.state.guided && this.saved) {
      if (this.recording.offline) return;
      this.state.completed[this.state.index] = this.saved.episode;
      if (this.state.index === this.state.order.length - 1) this.state.finished = true;
      else { this.state.index++; this.lastElapsed = 0; }
      this.persist();
      if (!this.state.finished) await this.apply(); else this.render();
      return;
    }
    if (!this.recording.task.value.trim()) { this.editTask(); return; }
    if (this.state.guided && (!this.matchesView || this.state.finished)) return;
    if (this.condition === 'head_3d' && this.state.guided && !window.godotWebcamTrackerLatest?.active) return;
    await this.recording.command('start');
  }

  editTask() {
    if (this.active || this.recording.busy || this.pending) return;
    if (!this.panel.hidden) this.closeSettings();
    this.taskEditing = true;
    this.taskInput.value = this.recording.task.value;
    this.render(); this.taskInput.focus(); this.taskInput.select();
  }
  saveTask() {
    if (!this.taskEditing || this.active || this.recording.busy || this.pending) return;
    this.taskInput.value = this.taskInput.value.trim();
    if (!this.taskInput.reportValidity()) return;
    this.taskEditing = false;
    this.recording.task.value = this.taskInput.value;
    this.recording.task.dispatchEvent(new Event('input', {bubbles: true}));
    this.render();
    const action = byId('trial-action');
    (action.disabled ? byId('open-advanced') : action).focus();
  }
  cancelTaskEdit(focus = true) {
    if (!this.taskEditing) return;
    this.taskEditing = false;
    this.render();
    if (focus) byId('trial-action').focus();
  }

  switchCamera(direction) {
    const options = Array.from(this.cameras.options).filter(option => option.value && !option.disabled);
    if (this.display.value !== 'rgb_camera' || options.length < 2) return;
    const current = options.findIndex(option => option.value === this.cameras.value);
    this.cameras.value = options[(Math.max(0, current) + direction + options.length) % options.length].value;
    this.cameras.dispatchEvent(new Event('change', {bubbles: true}));
    this.render();
  }

  render() {
    const recorder = this.recording, status = recorder.status || {};
    const active = this.active, locked = active || recorder.busy || this.pending;
    const saved = !active && this.saved;
    const guided = this.state.guided;
    const done = guided && this.state.finished;
    const needsTask = !recorder.task.value.trim();
    const mismatch = guided && !this.matchesView;
    const needsTracking = this.head.checked && this.display.value !== 'rgb_camera' && !window.godotWebcamTrackerLatest?.active;
    const needsView = guided && !this.viewReady();
    if (locked) this.taskEditing = false;
    const editing = !!this.taskEditing;
    byId('trial-task-editor').hidden = !editing;
    byId('trial-task').hidden = editing;
    const action = byId('trial-action');
    byId('trial-hud').dataset.state = status.state;
    byId('trial-position').textContent = guided ? `${this.state.index + 1}/${this.state.order.length}` : 'Free session';
    byId('trial-position').setAttribute('aria-label', guided ? `Trial ${this.state.index + 1} of ${this.state.order.length}` : 'Free session');
    byId('trial-condition').textContent = guided ? CONDITIONS[this.condition]
      : this.display.value === 'rgb_camera' ? CONDITIONS.fixed_rgb : this.head.checked ? CONDITIONS.head_3d : CONDITIONS.mouse_3d;
    const task = recorder.task.value.trim() || 'Task not set';
    byId('trial-task').textContent = task; byId('trial-task').title = task;
    if (active) this.lastElapsed = Number(status.elapsed_seconds) || 0;
    else if (saved) this.lastElapsed = Number(saved.duration_seconds) || this.lastElapsed;
    byId('trial-timer').textContent = formatTime(this.lastElapsed);
    const label = this.pending ? 'Switching…' : status.state === 'preparing' ? 'Preparing' : status.state === 'saving' ? 'Saving…'
      : active ? 'Recording' : done ? 'Complete' : saved ? 'Saved' : status.state === 'error' ? 'Interrupted' : 'Ready';
    byId('trial-recording-state').textContent = label;
    byId('trial-recording-state').hidden = label === 'Ready';
    action.textContent = editing ? 'Save task' : this.pending ? 'Switching…' : recorder.busy ? 'Please wait…' : active ? status.state === 'saving' ? 'Saving…' : 'Finish trial'
      : done ? 'Complete' : saved ? this.state.index === 2 ? 'End session' : 'Next trial' : needsTask ? 'Set task' : guided ? 'Start trial' : 'Start recording';
    action.disabled = this.pending || recorder.busy || status.state === 'saving' || done
      || (active ? status.is_owner === false : saved ? !!recorder.offline
        : !needsTask && (!!recorder.offline || !status.supported || !status.ready || !status.controller_active || mismatch || needsTracking || needsView));
    recorder.startButton.disabled ||= this.pending || (guided && (mismatch || needsTracking || needsView || !!saved || done));
    const progress = byId('trial-progress');
    progress.hidden = !guided;
    progress.max = this.state.timeTargetSeconds;
    progress.value = Math.min(this.lastElapsed, this.state.timeTargetSeconds);
    const timeTarget = `Time target ${formatTime(this.state.timeTargetSeconds)}`;
    progress.title = timeTarget;
    progress.setAttribute('aria-valuetext', `${formatTime(this.lastElapsed)} elapsed; ${timeTarget.toLowerCase()}`);
    byId('trial-timer').title = timeTarget;
    byId('trial-hud').dataset.timeReached = String(this.lastElapsed >= this.state.timeTargetSeconds);
    const completed = done ? this.state.order.length : Math.max(this.state.completed.length, saved ? this.state.index + 1 : this.state.index);
    const message = recorder.error || this.viewError || (done || saved ? ''
      : status.state === 'saving' ? 'Finishing the recording on the host. Keep this page open.'
      : active ? status.is_owner === false ? 'This recording belongs to another controller page.'
        : needsView ? 'Live view interrupted. Hold the arm if needed; the host is still recording.' : ''
      : needsTask ? ''
      : mismatch ? 'The view was changed in Settings. Restore the trial view, or turn off guided trials.'
      : needsTracking ? 'Waiting for head tracking. Look toward the laptop camera, then reset head tracking.'
      : needsView ? 'Waiting for the live view. Reconnect in Settings → View if it does not return.'
      : !status.supported ? 'Host recording is unavailable for this robot module.'
      : !status.controller_active ? 'Reconnect this controller in Settings → View before recording.'
      : !status.ready ? 'Waiting for fresh robot feedback. Check the arm in Settings.'
      : status.state === 'error' ? `${status.message} Start this trial again when ready.`
      : '');
    const messageNode = byId('trial-message');
    if (messageNode.textContent !== message) messageNode.textContent = message;
    messageNode.hidden = !message;
    const messageRole = recorder.error || this.viewError || status.state === 'error' ? 'alert' : 'status';
    if (messageNode.getAttribute('role') !== messageRole) messageNode.setAttribute('role', messageRole);
    byId('trial-outcome-control').hidden = !active || status.is_owner === false;
    byId('trial-outcome').value = recorder.outcome.value;
    byId('trial-outcome').disabled = status.state === 'saving' || recorder.busy;
    for (const id of ['guided-trials', 'trial-order', 'new-trial-session', 'restore-trial-view']) byId(id).disabled = locked;
    byId('trial-order').disabled ||= !guided || this.state.index > 0 || !!saved || done;
    byId('restore-trial-view').hidden = !guided || !mismatch;
    byId('trial-session-status').textContent = `Session ${this.state.id.slice(0, 8)} · ${completed} of 3 saved. New session keeps all host recordings.`;
    this.duration.disabled = locked;
    this.display.disabled = locked; this.head.disabled = locked;
    const rgb = this.display.value === 'rgb_camera';
    byId('camera-navigation').hidden = !rgb;
    const options = Array.from(this.cameras.options).filter(option => option.value);
    const selected = options.find(option => option.value === this.cameras.value);
    byId('camera-name').textContent = selected ? `${selected.textContent} · ${options.indexOf(selected) + 1} / ${options.length}` : 'Waiting for cameras';
    for (const id of ['previous-camera', 'next-camera']) byId(id).disabled = options.length < 2;
    byId('view-hint').hidden = rgb;
    byId('view-hint').textContent = this.head.checked ? 'Move your head to look around' : 'Drag to orbit · scroll to zoom';
    byId('reset-view').hidden = rgb;
    byId('reset-head-tracking').hidden = rgb || !this.head.checked;
  }

  bindSettings() {
    byId('open-advanced').addEventListener('click', () => {
      if (this.panel.hidden) this.openSettings(); else this.closeSettings();
    });
    byId('close-advanced').addEventListener('click', () => this.closeSettings());
    const tabs = Array.from(document.querySelectorAll('.settings-tabs [role="tab"]'));
    for (const [index, tab] of tabs.entries()) {
      tab.addEventListener('click', () => this.selectTab(tab.id.replace('tab-', '')));
      tab.addEventListener('keydown', event => {
        let next = index;
        if (event.key === 'ArrowRight') next = (index + 1) % tabs.length;
        else if (event.key === 'ArrowLeft') next = (index + tabs.length - 1) % tabs.length;
        else if (event.key === 'Home') next = 0;
        else if (event.key === 'End') next = tabs.length - 1;
        else return;
        event.preventDefault(); this.selectTab(tabs[next].id.replace('tab-', '')); tabs[next].focus();
      });
    }
    document.addEventListener('keydown', event => {
      if (event.key === 'Escape') {
        if (!this.panel.hidden) { event.preventDefault(); this.closeSettings(); }
        const connection = byId('connection-details');
        if (connection.open) { connection.open = false; connection.querySelector('summary').focus(); }
      }
    });
    // Workspace editing closes Settings directly; reflect it in the trigger too.
    new MutationObserver(() => byId('open-advanced').setAttribute('aria-expanded', String(!this.panel.hidden)))
      .observe(this.panel, {attributes: true, attributeFilter: ['hidden']});
    const positionPanel = () => {
      const hud = byId('trial-hud').getBoundingClientRect();
      this.panel.style.top = `${hud.left < 532 ? Math.ceil(hud.bottom + 14) : 70}px`;
    };
    new ResizeObserver(positionPanel).observe(byId('trial-hud'));
    window.addEventListener('resize', positionPanel);
    positionPanel();
  }
  selectTab(name) {
    for (const tab of document.querySelectorAll('.settings-tabs [role="tab"]')) {
      const selected = tab.id === `tab-${name}`;
      tab.setAttribute('aria-selected', String(selected)); tab.tabIndex = selected ? 0 : -1;
      byId(tab.getAttribute('aria-controls')).hidden = !selected;
    }
    document.querySelector('.advanced-body').scrollTop = 0;
  }
  openSettings(tab) {
    this.cancelTaskEdit(false);
    this.panel.hidden = false;
    if (tab) this.selectTab(tab);
    byId('open-advanced').setAttribute('aria-expanded', 'true');
    this.panel.querySelector('[role="tab"][aria-selected="true"]').focus();
  }
  closeSettings() {
    this.panel.hidden = true; byId('open-advanced').setAttribute('aria-expanded', 'false'); byId('open-advanced').focus();
  }
}
