const CONDITIONS = Object.freeze({fixed_rgb: 'Fixed cameras', mouse_3d: 'Mouse 3D', head_3d: 'Head tracking'});
const DEFAULT_ORDER = Object.keys(CONDITIONS);
const DEFAULT_TASK = 'Free exploration: practise the view and arm controls';
const PRACTICE_HINTS = {
  fixed_rgb: 'Try switching cameras while moving the arm. Use the camera arrows or left/right arrow keys.',
  mouse_3d: 'Try orbiting, panning and zooming to inspect objects from different angles while moving the arm.',
  head_3d: 'Move your head side to side and up/down to look around objects while moving the arm.',
};
const TASKS = {single_block: {instruction: DEFAULT_TASK, seconds: 75, success_metric: 'practice'},
  count_blocks: {instruction: 'Put as many blocks in the cup as possible', seconds: 75, success_metric: 'block_count'},
  custom: {instruction: 'Balance test: move the objects into the cup', seconds: 75, success_metric: 'pass_fail'}};
const PROTOCOL_KEY = 'robotTeleopTrialProtocol.v1';
const defaultMetric = id => id === 'count_blocks' ? 'block_count' : 'pass_fail';
function configuredTasks() {
  const tasks = structuredClone(TASKS);
  try {
    const saved = JSON.parse(globalThis.localStorage?.getItem(PROTOCOL_KEY) || 'null');
    for (const id of Object.keys(tasks)) {
      const value = saved?.[id];
      if (value && typeof value.instruction === 'string' && value.instruction.trim() && value.instruction.length <= 500
          && Number.isInteger(value.seconds) && value.seconds >= 30 && value.seconds <= 7200
          && (id === 'single_block' ? value.success_metric === 'practice' : ['pass_fail', 'block_count'].includes(value.success_metric))) tasks[id] = {...value};
    }
  } catch { /* Browser storage is optional. */ }
  return tasks;
}
export function randomizedConditions(random = Math.random) {
  const order = [...DEFAULT_ORDER];
  for (let i = order.length - 1; i > 0; i--) { const j = Math.floor(random() * (i + 1)); [order[i], order[j]] = [order[j], order[i]]; }
  return order;
}
const RENDERERS = new Set(['point_cloud', 'shader_mesh', 'independent_mesh', 'unified_mesh', 'blendpcr']);
const STORAGE_KEY = 'robotTeleopTrialSession.v1';
const ACTIVE = new Set(['preparing', 'recording', 'paused', 'saving']);
const byId = id => document.getElementById(id);
const formatTime = seconds => `${Math.floor(seconds / 60).toString().padStart(2, '0')}:${Math.floor(seconds % 60).toString().padStart(2, '0')}`;
export function namedSession(base, date = new Date()) {
  const parts = Object.fromEntries(new Intl.DateTimeFormat('en-GB', {timeZone: 'Europe/Berlin',
    year: 'numeric', month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit', second: '2-digit',
    hourCycle: 'h23', timeZoneName: 'shortOffset'}).formatToParts(date).map(p => [p.type, p.value]));
  return `${base.trim()} — ${parts.year}-${parts.month}-${parts.day} ${parts.hour}:${parts.minute}:${parts.second} ${parts.timeZoneName}`;
}

export function makeTrialPlan(state, random = Math.random) {
  let activities = state.activitySelection === 'all' ? ['single_block', 'count_blocks', 'custom'] : [state.activitySelection];
  activities = activities.filter(id => state.tasks[id]?.instruction?.trim());
  const practice = activities.includes('single_block');
  activities = activities.filter(id => id !== 'single_block');
  if (state.randomActivities !== false) {
    for (let i = activities.length - 1; i > 0; i--) { const j = Math.floor(random() * (i + 1)); [activities[i], activities[j]] = [activities[j], activities[i]]; }
  }
  const order = () => state.randomize !== false ? randomizedConditions(random) : state.order;
  const training = practice ? order().map(condition => ({task_id: 'single_block', task: state.tasks.single_block.instruction,
    time_limit_seconds: state.practiceSeconds || 60, condition, phase: 'practice', success_metric: 'practice', outcome_rule: 'manual'})) : [];
  return training.concat(activities.flatMap(task_id => Array.from({length: 2}, () =>
    order().map(condition => ({
      task_id, task: state.tasks[task_id].instruction, time_limit_seconds: state.tasks[task_id].seconds, condition,
      success_metric: state.tasks[task_id].success_metric || defaultMetric(task_id),
      phase: 'measured',
      outcome_rule: (state.tasks[task_id].success_metric || defaultMetric(task_id)) === 'pass_fail' ? 'finish_before_limit' : 'manual',
    }))).flat()));
}

/** A small facilitator layer over the existing view and host-recording controls. */
export class TrialController {
  constructor({recording, applyCondition, resetView, resetTracking, pauseRobot, resumeRobot, finishRobotPause,
    prepareSession = async () => {}, prepareTrial = async () => {}, sessionConnected = () => true, viewReady = () => true}) {
    this.recording = recording;
    this.applyView = applyCondition;
    this.resetView = resetView;
    this.resetTracking = resetTracking;
    this.viewReady = viewReady;
    this.pauseRobot = pauseRobot;
    this.resumeRobot = resumeRobot;
    this.finishRobotPause = finishRobotPause;
    this.prepareSession = prepareSession;
    this.prepareTrial = prepareTrial;
    this.sessionConnected = sessionConnected;
    this.display = byId('display-mode');
    this.head = byId('head-tracking-enabled');
    this.cameras = byId('rgb-camera-select');
    this.duration = byId('trial-duration');
    this.panel = byId('advanced-setup');
    this.state = this.restore();
    this.restoredPlan = !!this.state.plan?.length;
    this.pending = false; this.lastElapsed = 0;
    if (this.state.version !== 3 || !this.state.tasks) {
      this.state = {...this.state, version: 3, id: this.sessionId(), index: 0, completed: [], finished: false,
        randomize: true, randomActivities: true, activitySelection: 'all', plan: [],
        tasks: configuredTasks(), protocolVersion: 2, practiceSeconds: 60};
    }
    // Upgrade future-session configuration only. A restored plan keeps its
    // original instructions, limits, scoring and progress until New session.
    if (!this.state.protocolVersion) {
      for (const [id, defaults] of Object.entries(TASKS)) {
        const task = this.state.tasks[id] || {};
        this.state.tasks[id] = {...defaults, ...task,
          instruction: task.instruction?.trim() || defaults.instruction,
          seconds: [60, 120].includes(task.seconds) ? 75 : task.seconds || 75,
          success_metric: task.success_metric || defaults.success_metric};
      }
      this.state.protocolVersion = 1;
    }
    if (this.state.protocolVersion < 2) {
      this.state.tasks.single_block = {...TASKS.single_block};
      this.state.protocolVersion = 2;
    }
    this.state.practiceSeconds ??= 60;
    byId('trial-practice-duration').value = String(this.state.practiceSeconds);
    if (!this.state.plan?.length) this.state.plan = makeTrialPlan(this.state);
    this.state.prepared ??= this.state.index > 0 || this.state.completed.length > 0 || this.state.finished === true;
    this.state.nameBase = typeof this.state.nameBase === 'string' ? this.state.nameBase.slice(0, 120) : 'Session';
    this.state.name = typeof this.state.name === 'string' ? this.state.name : this.state.prepared ? namedSession(this.state.nameBase) : '';
    byId('trial-session-input').value = this.state.nameBase;
    this.syncTask();
    byId('trial-task-kind').value = this.state.activitySelection;
    byId('random-activity-order').checked = this.state.randomActivities !== false;
    for (const [id, number] of [['single_block', 1], ['count_blocks', 2], ['custom', 3]]) {
      byId(`trial-task${number}-instruction`).value = this.state.tasks[id].instruction;
      byId(`trial-task${number}-metric`).value = this.state.tasks[id].success_metric || defaultMetric(id);
    }
    if (!RENDERERS.has(this.state.renderer)) this.state.renderer = this.display.value === 'rgb_camera' ? 'independent_mesh' : this.display.value;
    byId('guided-trials').checked = this.state.guided;
    byId('trial-order').value = this.state.randomize !== false ? 'random' : this.state.order.join(',');
    this.persist();
    recording.context = () => {
      if (!this.state.guided) return null;
      if (!this.state.prepared) throw new Error('Set up the session before starting a trial.');
      if (!this.matchesView) throw new Error('Restore the trial view in Settings before recording.');
      if (this.state.finished || this.saved) throw new Error('This trial is saved. Continue with Next trial.');
      if (this.condition === 'head_3d' && !window.godotWebcamTrackerLatest?.active)
        throw new Error('Wait for head tracking before starting this trial.');
      if (!this.viewReady()) throw new Error('Wait for the live view before starting this trial.');
      return {session_id: this.state.id, session_name: this.state.name, index: this.state.index + 1, total: this.state.plan.length,
        condition: this.condition, display_mode: this.display.value,
        task_id: this.state.taskId, time_limit_seconds: this.state.timeTargetSeconds,
        success_metric: this.state.plan[this.state.index].success_metric || defaultMetric(this.state.taskId),
        outcome_rule: this.state.plan[this.state.index].outcome_rule ||
          (defaultMetric(this.state.taskId) === 'pass_fail' ? 'finish_before_limit' : 'manual'), result_prompt: !this.practice,
        ...(this.state.plan[this.state.index].phase ? {phase: this.state.plan[this.state.index].phase} : {})};
    };
    recording.root.addEventListener('recordingchange', () => this.render());
    recording.task.addEventListener('input', () => {
      this.state.tasks[this.state.taskId].instruction = recording.task.value;
      for (const entry of this.state.plan) if (entry.task_id === this.state.taskId) entry.task = recording.task.value;
      this.state.task = recording.task.value; this.persist();
    });
    recording.outcome.addEventListener('change', () => this.render());
    byId('trial-action').addEventListener('click', () => this.act());
    byId('trial-session-input').addEventListener('input', event => {
      this.state.nameBase = event.target.value; this.persist(); this.render();
    });
    byId('trial-pause').addEventListener('click', () => this.togglePause());
    byId('guided-trials').addEventListener('change', event => {
      this.state.guided = event.target.checked;
      this.persist();
      if (this.state.guided) this.apply(); else this.render();
    });
    byId('trial-order').addEventListener('change', event => {
      this.state.randomize = event.target.value === 'random';
      if (!this.state.randomize) this.state.order = event.target.value.split(',');
      this.newSession();
    });
    byId('trial-task-kind').addEventListener('change', event => {
      this.state.activitySelection = event.target.value; this.newSession();
    });
    byId('random-activity-order').addEventListener('change', event => {
      this.state.randomActivities = event.target.checked; this.newSession();
    });
    for (const [id, number] of [['single_block', 1], ['count_blocks', 2], ['custom', 3]]) {
      byId(`trial-task${number}-instruction`).addEventListener('change', event => {
        if (!event.target.reportValidity()) return;
        this.state.tasks[id].instruction = event.target.value.trim();
        this.protocolMessage = 'Changed in this browser. Save task settings on host to keep them.'; this.newSession();
      });
      byId(`trial-task${number}-metric`).addEventListener('change', event => {
        this.state.tasks[id].success_metric = event.target.value;
        this.protocolMessage = 'Changed in this browser. Save task settings on host to keep them.'; this.newSession();
      });
    }
    byId('trial-save-block-count').addEventListener('click', () => this.saveResult());
    byId('trial-result-form').addEventListener('submit', event => {event.preventDefault();this.saveResult();});
    byId('trial-result-close').addEventListener('click', () => byId('trial-result-dialog').close());
    byId('trial-save-protocol').addEventListener('click', () => this.saveProtocol());
    this.duration.addEventListener('change', () => {
      if (!this.duration.reportValidity()) {
        this.duration.value = String(this.state.timeTargetSeconds);
        return;
      }
      for (const task of Object.values(this.state.tasks)) task.seconds = this.duration.valueAsNumber;
      this.protocolMessage = 'Changed in this browser. Save task settings on host to keep them.';
      this.newSession();
    });
    byId('trial-practice-duration').addEventListener('change', event => {
      if (!event.target.reportValidity()) { event.target.value = String(this.state.practiceSeconds); return; }
      this.state.practiceSeconds = event.target.valueAsNumber;
      this.protocolMessage = 'Changed in this browser. Save task settings on host to keep them.';
      this.newSession();
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

  get condition() { return this.state.plan[this.state.index].condition; }
  get practice() { return this.state.plan[this.state.index].phase === 'practice'; }
  syncTask() {
    const entry = this.state.plan[this.state.index];
    this.state.taskId = entry.task_id; this.state.task = entry.task; this.state.timeTargetSeconds = entry.time_limit_seconds;
    this.recording.task.value = entry.task; this.duration.value = String(this.state.tasks.single_block.seconds);
  }
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
      if (state?.version === 3 && typeof state.id === 'string' && /^[A-Za-z0-9_-]{1,80}$/.test(state.id)
          && Array.isArray(state.plan) && state.plan.length > 0 && state.plan.length <= 18
          && state.plan.every(p => Object.hasOwn(CONDITIONS, p.condition) && Object.hasOwn(TASKS, p.task_id)
            && typeof p.task === 'string' && p.task.trim() && p.task.length <= 500
            && Number.isInteger(p.time_limit_seconds) && p.time_limit_seconds >= 30 && p.time_limit_seconds <= 7200)
          && Number.isInteger(state.index) && state.index >= 0 && state.index < state.plan.length && state.tasks)
        return {...state, completed: Array.isArray(state.completed) ? state.completed : []};
      if ([1, 2].includes(state?.version)) return {...state, completed: []};
    } catch { /* Storage is optional. */ }
    return {version: 1, id: this.sessionId(), index: 0, order: [...DEFAULT_ORDER], completed: [], guided: true, task: DEFAULT_TASK};
  }
  sessionId() { return window.crypto?.randomUUID?.() || `session-${Date.now()}-${Math.random().toString(16).slice(2)}`; }
  persist() {
    try { sessionStorage.setItem(STORAGE_KEY, JSON.stringify(this.state)); } catch { /* Optional. */ }
    try { globalThis.localStorage?.setItem(PROTOCOL_KEY, JSON.stringify(this.state.tasks)); } catch { /* Optional. */ }
  }

  async initialize() {
    if (typeof window.fetch === 'function') {
      try {
        const response = await fetch('/api/v1/trial-protocol', {cache: 'no-store'});
        if (!response.ok) throw new Error('Host task settings are unavailable.');
        const value = await response.json();
        this.state.practiceSeconds = value.practice_time_limit_seconds || 60;
        byId('trial-practice-duration').value = String(this.state.practiceSeconds);
        for (const [id, task] of Object.entries(value.tasks))
          this.state.tasks[id] = {...task, seconds: value.time_limit_seconds};
        if (!this.restoredPlan) this.state.plan = makeTrialPlan(this.state);
        this.protocolMessage = this.restoredPlan ? 'Host settings loaded. New session uses these settings; the restored plan is unchanged.' : 'Task settings loaded from host.';
        this.syncTask(); this.persist();
      } catch (error) { this.protocolMessage = error.message; }
    }
    if (this.state.guided && !this.active) await this.apply();
  }
  async saveProtocol() {
    if (this.active || this.protocolSaving || !this.duration.reportValidity() || !byId('trial-practice-duration').reportValidity()) return;
    const tasks = {};
    for (const [id, number] of [['single_block', 1], ['count_blocks', 2], ['custom', 3]]) {
      const input = byId(`trial-task${number}-instruction`);
      if (!input.reportValidity() || !input.value.trim()) return;
      tasks[id] = {instruction: input.value.trim(), success_metric: byId(`trial-task${number}-metric`).value};
    }
    this.protocolSaving = true; this.render();
    try {
      const response = await fetch('/api/v1/trial-protocol', {method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({version: 2, time_limit_seconds: this.duration.valueAsNumber,
          practice_time_limit_seconds: byId('trial-practice-duration').valueAsNumber, tasks})});
      const value = await response.json();
      if (!response.ok) throw new Error(value.error || 'Task settings could not be saved.');
      this.state.practiceSeconds = value.practice_time_limit_seconds;
      for (const [id, task] of Object.entries(value.tasks)) this.state.tasks[id] = {...task, seconds: value.time_limit_seconds};
      this.persist(); this.protocolMessage = 'Saved on host. New sessions use these settings after browser or host restarts.';
    } catch (error) { this.protocolMessage = error.message; }
    finally { this.protocolSaving = false; this.render(); }
  }
  resultNeedsReview(saved = this.saved) {
    if (saved?.trial?.phase === 'practice') return false;
    return !!saved && (saved.trial?.result_prompt ? saved.result_reviewed !== true
      : (saved.trial?.success_metric || defaultMetric(saved.trial?.task_id)) === 'block_count' && saved.block_count == null);
  }
  openResult() {
    const saved = this.saved;
    if (!saved || saved.trial?.phase === 'practice') return;
    const dialog = byId('trial-result-dialog');
    if (this.resultEpisode !== saved.episode) {
      byId('trial-outcome').value = ['success', 'failure'].includes(saved.outcome) ? saved.outcome : saved.timed_out ? 'failure' : 'success';
      byId('trial-block-count').value = saved.block_count == null ? '' : String(saved.block_count);
      this.resultEpisode = saved.episode;
    }
    byId('trial-result-title').textContent = saved.timed_out ? 'Time’s up' : 'Trial complete';
    byId('trial-result-summary').textContent = `${this.state.task} · ${formatTime(this.lastElapsed)}`;
    byId('trial-result-error').hidden = true;
    this.promptedEpisode = saved.episode;
    if (!dialog.open) dialog.showModal?.();
  }
  async saveResult() {
    const saved = this.saved;
    if (!saved || saved.trial?.phase === 'practice' || this.recording.busy) return;
    const counting = (saved.trial?.success_metric || defaultMetric(saved.trial?.task_id)) === 'block_count';
    const input = byId('trial-block-count');
    if (counting && !input.reportValidity()) return;
    const last = this.state.index === this.state.plan.length - 1;
    const ok = counting ? await this.recording.command('score', {block_count: input.valueAsNumber, session_id: this.state.id})
      : await this.recording.command('rate', {outcome: byId('trial-outcome').value, session_id: this.state.id});
    if (!ok) {
      byId('trial-result-error').textContent = this.recording.error || 'Result was not saved. Try again.';
      byId('trial-result-error').hidden = false; return;
    }
    byId('trial-result-dialog').close?.();
    this.render();
    if (!last) await this.act();
  }
  async apply() {
    if (this.active || this.recording.busy || this.pending) return;
    this.syncTask();
    this.pending = true; this.viewError = ''; this.render();
    try { await this.applyView(this.condition, this.state.renderer); }
    catch (error) { this.viewError = error.message || 'The view could not start. Reconnect in Settings.'; }
    finally { this.pending = false; this.render(); }
  }
  async newSession({applyView = true} = {}) {
    if (this.active || this.recording.busy || this.pending) return;
    const plan = makeTrialPlan(this.state);
    if (!plan.length) { this.viewError = "Name task 3 before selecting it."; this.render(); return; }
    this.state.plan = plan;
    this.state.id = this.sessionId(); this.state.index = 0; this.state.completed = []; this.state.finished = false;
    this.state.prepared = false; this.state.name = '';
    this.lastElapsed = 0; this.persist();
    if (applyView) await this.apply();
    else { this.syncTask(); this.render(); }
  }
  async act() {
    if (this.recording.busy || this.pending || this.pausePending || this.setupPending) return;
    if (this.active) {
      if (this.recording.status?.state !== 'saving' && this.recording.status?.is_owner !== false)
        await this.recording.command('stop');
      return;
    }
    if (this.state.guided && this.state.finished) {
      if (!byId('trial-session-input').reportValidity() || !byId('trial-session-input').value.trim()) return;
      await this.newSession({applyView: false});
      await this.setupSession();
      await this.apply();
      return;
    }
    if (this.state.guided && this.saved) {
      if (this.resultNeedsReview()) { this.openResult(); return; }
      if ((this.saved.trial?.success_metric || defaultMetric(this.saved.trial?.task_id)) === 'block_count'
          && this.saved.block_count == null) return;
      if (this.recording.offline) return;
      this.state.completed[this.state.index] = this.saved.episode;
      if (this.state.index === this.state.plan.length - 1) this.state.finished = true;
      else { this.state.index++; this.lastElapsed = 0; }
      this.persist();
      if (!this.state.finished) await this.apply(); else this.render();
      return;
    }
    if (!this.recording.task.value.trim()) return;
    if (this.state.guided && (!this.state.prepared || !this.sessionConnected())) {
      await this.setupSession(); return;
    }
    if (this.state.guided && (!this.matchesView || this.state.finished)) return;
    if (this.condition === 'head_3d' && this.state.guided && !window.godotWebcamTrackerLatest?.active) return;
    this.setupPending = true; this.viewError = ''; this.render();
    try {
      if (this.state.guided) await this.prepareTrial();
      await this.recording.command('start');
    } catch (error) { this.viewError = String(error.message || error); }
    finally { this.setupPending = false; this.render(); }
  }

  async setupSession() {
    const input = byId('trial-session-input');
    if (!input.reportValidity() || !input.value.trim()) return;
    this.setupPending = true; this.viewError = ''; this.render();
    try {
      // Invoke immediately from the click so the browser's USB chooser is allowed.
      await this.prepareSession();
      if (!this.state.name) this.state.name = namedSession(input.value);
      this.state.nameBase = input.value.trim(); this.state.prepared = true;
      this.persist();
    } catch (error) { this.viewError = String(error.message || error); }
    finally { this.setupPending = false; this.render(); }
  }

  async togglePause() {
    const recorder = this.recording, status = recorder.status || {};
    if (this.pending || this.pausePending || recorder.busy || recorder.offline || status.is_owner === false
      || status.pause_supported !== true || !['recording', 'paused'].includes(status.state)
      || typeof this.pauseRobot !== 'function' || typeof this.resumeRobot !== 'function') return;
    const resume = status.state === 'paused';
    this.pausePending = resume ? 'resuming' : 'pausing';
    this.viewError = ''; this.render();
    try {
      if (resume) {
        if (!await recorder.command('resume')) return;
        try {
          await this.resumeRobot();
          this.robotPaused = false;
        } catch (error) {
          await this.pauseRobot().catch(() => {});
          await recorder.command('pause');
          throw new Error(`${error.message || error} Arm remains held.`);
        }
      } else {
        this.robotPaused = true;
        await this.pauseRobot();
        await recorder.command('pause');
      }
    } catch (error) {
      this.viewError = String(error.message || error);
    } finally {
      this.pausePending = null; this.render();
    }
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
    const active = this.active, locked = active || recorder.busy || this.pending || !!this.pausePending || !!this.setupPending;
    if (this.robotPaused && !this.pausePending && !['recording', 'paused'].includes(status.state)) {
      this.robotPaused = false;
      this.finishRobotPause?.();
    }
    const saved = !active && this.saved;
    const counting = !!saved && (saved.trial?.success_metric || defaultMetric(saved.trial?.task_id)) === 'block_count';
    const needsCount = counting && saved.block_count == null;
    const needsReview = this.resultNeedsReview(saved);
    const guided = this.state.guided;
    if (guided && saved && !needsCount && !needsReview && this.state.index === this.state.plan.length - 1 && !this.state.finished) {
      this.state.completed[this.state.index] = saved.episode;
      this.state.finished = true; this.persist();
    }
    const done = guided && this.state.finished;
    const needsSetup = guided && (!this.state.prepared || !this.sessionConnected());
    const needsTask = !recorder.task.value.trim();
    const mismatch = guided && !this.matchesView;
    const needsTracking = this.head.checked && this.display.value !== 'rgb_camera' && !window.godotWebcamTrackerLatest?.active;
    const needsView = guided && !this.viewReady();
    const action = byId('trial-action');
    byId('trial-hud').dataset.state = status.state;
    const sessionInput = byId('trial-session-input');
    byId('trial-session-edit').hidden = !guided || (this.state.prepared && !done);
    sessionInput.disabled = locked;
    const practiceCount = this.state.plan.filter(entry => entry.phase === 'practice').length;
    const position = this.practice ? `Practice ${this.state.index + 1}/${practiceCount}`
      : `${this.state.index + 1 - practiceCount}/${this.state.plan.length - practiceCount}`;
    byId('trial-position').textContent = guided ? position : 'Free session';
    byId('trial-position').setAttribute('aria-label', guided ? position : 'Free session');
    byId('trial-condition').textContent = guided ? CONDITIONS[this.condition]
      : this.display.value === 'rgb_camera' ? CONDITIONS.fixed_rgb : this.head.checked ? CONDITIONS.head_3d : CONDITIONS.mouse_3d;
    const task = recorder.task.value.trim() || 'Task not configured';
    byId('trial-task').textContent = task; byId('trial-task').title = task;
    if (active) this.lastElapsed = Number(status.elapsed_seconds) || 0;
    else if (saved) this.lastElapsed = Number(saved.active_duration_seconds ?? saved.duration_seconds) || this.lastElapsed;
    byId('trial-timer').textContent = formatTime(this.lastElapsed);
    const label = this.pending ? 'Switching…' : status.state === 'preparing' ? 'Preparing' : status.state === 'saving' ? 'Saving…'
      : status.state === 'paused' ? 'Paused' : active ? 'Recording' : done ? 'Complete' : saved ? 'Saved' : status.state === 'error' ? 'Interrupted' : 'Ready';
    byId('trial-recording-state').textContent = label;
    byId('trial-recording-state').hidden = ['Ready', 'Recording', 'Saved', 'Complete'].includes(label);
    action.textContent = this.pending ? 'Switching…' : this.setupPending ? 'Setting up…' : recorder.busy ? 'Please wait…' : active ? status.state === 'saving' ? 'Saving…' : 'Done'
      : done ? 'Restart session' : saved ? needsReview ? 'Review result' : this.state.index === this.state.plan.length - 1 ? 'End session' : 'Next trial'
        : needsSetup ? 'Set up session' : guided ? this.practice ? 'Start practice' : this.state.index === 0 ? 'Start first trial' : 'Start trial' : 'Start recording';
    action.disabled = this.pending || !!this.pausePending || !!this.setupPending || recorder.busy || status.state === 'saving'
      || (active ? status.is_owner === false : done ? !!recorder.offline || !sessionInput.value.trim()
        : saved ? !!recorder.offline
        : needsTask || !!recorder.offline || !status.supported || !status.controller_active
          || (needsSetup ? !sessionInput.value.trim() : !status.ready || mismatch || needsTracking || needsView));
    recorder.startButton.disabled ||= this.pending || !!this.setupPending || (guided && (needsSetup || mismatch || needsTracking || needsView || !!saved || done));
    const pause = byId('trial-pause');
    pause.hidden = !['recording', 'paused'].includes(status.state) || status.pause_supported !== true
      || typeof this.pauseRobot !== 'function' || typeof this.resumeRobot !== 'function' || status.is_owner === false;
    pause.disabled = !!this.pausePending || recorder.busy || !!recorder.offline || !!this.pending;
    pause.textContent = this.pausePending === 'pausing' ? 'Pausing…' : this.pausePending === 'resuming' ? 'Resuming…'
      : status.state === 'paused' ? 'Resume' : 'Pause';
    const progress = byId('trial-progress');
    progress.hidden = !guided;
    progress.max = this.state.timeTargetSeconds;
    progress.value = Math.min(this.lastElapsed, this.state.timeTargetSeconds);
    const timeTarget = `Time limit ${formatTime(this.state.timeTargetSeconds)}`;
    progress.title = timeTarget;
    progress.setAttribute('aria-valuetext', `${formatTime(this.lastElapsed)} elapsed; ${timeTarget.toLowerCase()}`);
    byId('trial-timer').title = timeTarget;
    byId('trial-hud').dataset.timeReached = String(this.lastElapsed >= this.state.timeTargetSeconds);
    const completed = done ? this.state.plan.length : Math.max(this.state.completed.length, saved ? this.state.index + 1 : this.state.index);
    const message = recorder.error || this.viewError || (saved || done ? ''
      : status.state === 'saving' ? 'Finishing the recording on the host. Keep this page open.'
      : active ? status.is_owner === false ? 'This recording belongs to another controller page.'
        : needsView ? 'Live view interrupted. Hold the arm if needed; the host is still recording.'
          : guided && this.practice ? PRACTICE_HINTS[this.condition] : ''
      : needsSetup ? `Set up connects the physical leader arm and enables control. Recording starts with ${this.practice ? 'Start practice' : 'Start trial'}.`
      : needsTask ? ''
      : mismatch ? 'The view was changed in Settings. Restore the trial view, or turn off guided trials.'
      : needsTracking ? 'Waiting for head tracking. Look toward the laptop camera, then reset head tracking.'
      : needsView ? 'Waiting for the live view. Reconnect in Settings → View if it does not return.'
      : !status.supported ? 'Host recording is unavailable for this robot module.'
      : !status.controller_active ? 'Reconnect this controller in Settings → View before recording.'
      : !status.ready ? 'Waiting for fresh robot feedback. Check the arm in Settings.'
      : status.state === 'error' ? `${status.message} Start this trial again when ready.`
      : guided && this.practice && !saved ? PRACTICE_HINTS[this.condition] : '');
    const messageNode = byId('trial-message');
    if (messageNode.textContent !== message) messageNode.textContent = message;
    messageNode.hidden = !message;
    const messageRole = recorder.error || this.viewError || status.state === 'error' ? 'alert' : 'status';
    if (messageNode.getAttribute('role') !== messageRole) messageNode.setAttribute('role', messageRole);
    const result = byId('trial-block-result'); result.hidden = !counting;
    byId('trial-block-count').required = counting;
    byId('trial-block-score').textContent = counting && saved.block_count != null ? `${saved.block_count} blocks in cup` : '';
    byId('trial-save-block-count').disabled = recorder.busy || !!recorder.offline;
    if (counting && this.countEpisode !== saved.episode) { byId('trial-block-count').value = saved.block_count == null ? '' : String(saved.block_count); this.countEpisode = saved.episode; }
    byId('trial-outcome-control').hidden = counting || this.practice;
    byId('trial-outcome').disabled = recorder.busy;
    byId('trial-save-block-count').textContent = this.state.index === this.state.plan.length - 1 ? 'Save & Finish' : 'Save & Next';
    if (saved && needsReview && this.promptedEpisode !== saved.episode) this.openResult();
    for (const id of ['guided-trials', 'trial-task-kind', 'random-activity-order', 'trial-task3-instruction', 'trial-order', 'new-trial-session', 'restore-trial-view']) byId(id).disabled = locked;
    byId('trial-order').disabled ||= !guided || this.state.index > 0 || !!saved || done;
    byId('restore-trial-view').hidden = !guided || !mismatch;
    byId('trial-session-status').textContent = `${this.state.name || 'Session not started'} · ${completed} of ${this.state.plan.length} saved. New session keeps all host recordings.`;
    byId('trial-task3-option').disabled = !this.state.tasks.custom.instruction.trim();
    this.duration.disabled = locked;
    byId('trial-practice-duration').disabled = locked;
    for (const [id, number] of [['single_block', 1], ['count_blocks', 2], ['custom', 3]]) for (const field of ['instruction', 'metric']) {
      const element = byId(`trial-task${number}-${field}`); element.disabled = locked;
      const value = field === 'instruction' ? this.state.tasks[id].instruction : this.state.tasks[id].success_metric || defaultMetric(id);
      if (element !== document.activeElement && element.value !== value) element.value = value;
    }
    byId('trial-save-protocol').disabled = locked || this.protocolSaving;
    byId('trial-protocol-status').textContent = this.protocolMessage || 'Save task settings on host to share them across restarts.';
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
    this.panel.hidden = false;
    if (tab) this.selectTab(tab);
    byId('open-advanced').setAttribute('aria-expanded', 'true');
    this.panel.querySelector('[role="tab"][aria-selected="true"]').focus();
  }
  closeSettings() {
    this.panel.hidden = true; byId('open-advanced').setAttribute('aria-expanded', 'false'); byId('open-advanced').focus();
  }
}
