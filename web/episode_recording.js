// The browser controls capture; RGB/state/action files are written on the host.
export class EpisodeRecordingControl {
  constructor(root, {client, fetch: request = (...args) => fetch(...args), interval = 1000} = {}) {
    this.root = root;
    this.client = client;
    this.request = request;
    this.interval = interval;
    this.busy = false;
    this.revision = 0;
    this.status = null;
    this.message = root.querySelector('[data-recording-status]');
    this.task = root.querySelector('[data-recording-task]');
    this.outcome = root.querySelector('[data-recording-outcome]');
    this.startButton = root.querySelector('[data-recording-start]');
    this.stopButton = root.querySelector('[data-recording-stop]');
    this.discardButton = root.querySelector('[data-recording-discard]');
    this.startButton.addEventListener('click', () => this.command('start'));
    this.stopButton.addEventListener('click', () => this.command('stop'));
    this.discardButton.addEventListener('click', () => this.command('discard'));
    this.task.addEventListener('input', () => this.render());
    this.poll();
  }

  async poll() {
    if (this.closed) return;
    const revision = this.revision;
    try {
      const response = await this.request('/api/v1/recording?'+new URLSearchParams({client: this.client}), {cache: 'no-store'});
      if (!response.ok) throw new Error('Host recording is unavailable. Update the gateway and reconnect.');
      const status = await response.json();
      if (revision === this.revision && !this.busy) {
        this.status = status;
        this.offline = false;
        this.root.hidden = this.status.supported === false;
        this.error = this.commandError || '';
      }
    } catch (error) {
      if (revision === this.revision && !this.busy) {
        this.offline = true;
        this.error = error.message;
      }
    }
    this.render();
    if (!this.closed) this.timer = setTimeout(() => this.poll(), this.interval);
  }

  async command(command) {
    if (this.busy) return false;
    if (command === 'start' && !this.task.reportValidity()) return false;
    this.revision++;
    this.busy = true;
    this.error = this.commandError = '';
    this.render();
    try {
      const payload = {command, client: this.client};
      if (command === 'start') {
        payload.task = this.task.value.trim();
        const context = this.context?.();
        if (context) payload.trial = context;
      }
      else payload.outcome = this.outcome.value;
      const response = await this.request('/api/v1/recording', {method: 'POST',
        headers: {'Content-Type': 'application/json'}, body: JSON.stringify(payload)});
      const result = await response.json();
      if (!response.ok) throw new Error(result.error || 'The host could not change recording state.');
      this.status = result;
      this.offline = false;
      if (command === 'start') this.outcome.value = 'unrated';
      return true;
    } catch (error) {
      this.error = this.commandError = error.message;
      return false;
    } finally {
      this.busy = false;
      this.render();
    }
  }

  render() {
    const status = this.status;
    const active = ['preparing', 'recording', 'paused', 'saving'].includes(status?.state);
    const finishing = status?.state === 'saving';
    const owner = status?.is_owner !== false;
    this.startButton.disabled = this.busy || active || !status?.ready || !status?.controller_active || !this.task.value.trim();
    this.stopButton.disabled = this.busy || !active || finishing || !owner;
    this.discardButton.disabled = this.stopButton.disabled;
    this.task.disabled = active || this.busy;
    this.outcome.disabled = !active || finishing || this.busy || !owner;
    this.startButton.textContent = status?.state === 'preparing' ? 'Preparing…' : 'Start recording';
    this.stopButton.textContent = finishing ? 'Saving…' : 'Stop and save';
    this.root.dataset.recordingState = status?.state || 'unavailable';
    let message = this.error || status?.message || 'Checking host recording…';
    if (!this.error && status?.state === 'recording')
      message = `Recording on host · ${status.frames} frames · ${status.elapsed_seconds.toFixed(1)} s`;
    else if (!this.error && status && !active && !status.ready && status.state !== 'error')
      message = 'Waiting for measured feedback and applied targets. Recording does not enable or move the arm.';
    if (!this.error && status?.last_saved && !active) {
      const saved = status.last_saved;
      message += ` ${saved.frames} frames · ${saved.status} · ${saved.outcome}. Host folder: ${saved.host_folder}`;
    }
    if (active && !owner) message = 'Another controller page owns this recording.';
    this.message.textContent = message;
    this.message.setAttribute('role', this.error || status?.state === 'error' ? 'alert' : 'status');
    this.root.dispatchEvent(new CustomEvent('recordingchange'));
  }

  close() {
    this.closed = true;
    clearTimeout(this.timer);
  }
}
