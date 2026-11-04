export class CameraAlignmentControl {
  constructor(root, {request = (...args) => fetch(...args), confirm = message => window.confirm(message)} = {}) {
    this.root = root;
    this.request = request;
    this.confirm = confirm;
    this.closed = false;
    this.sending = false;
    this.worldLevelToken = null;
    this.status = root.querySelector('#camera-alignment-status');
    this.backup = root.querySelector('#camera-alignment-backup');
    this.buttons = [root.querySelector('#camera-align-auto'), root.querySelector('#camera-align-refine'), root.querySelector('#camera-align-aruco'), root.querySelector('#camera-align-charuco')];
    this.buttons[0].addEventListener('click', () => this.start('markerless'));
    this.buttons[1].addEventListener('click', () => this.start('refine'));
    this.buttons[2].addEventListener('click', () => this.start('aruco'));
    this.buttons[3].addEventListener('click', () => this.start('charuco'));
    this.topOnlyButton = root.querySelector("#camera-align-top-only");
    if(this.topOnlyButton){this.buttons.push(this.topOnlyButton);this.topOnlyButton.addEventListener("click",()=>this.start("charuco","top_only"));}
    this.poll();
  }
  render(value) {
    const level = value.world_level;
    if (level?.token > 0) {
      if (this.worldLevelToken !== level.token) {
        window.dispatchEvent(new CustomEvent('camera-board-world-level', {detail: level}));
      }
      this.worldLevelToken = level.token;
    } else if (this.worldLevelToken === null) this.worldLevelToken = 0;
    const correction = this.root.querySelector('#camera-depth-correction-status');
    if (correction) correction.textContent = value.active_depth_corrections?.length
      ? `Saved depth alignment correction active: ${value.active_depth_corrections.join(', ')}.` : '';
    this.status.textContent = value.message || 'Camera alignment is ready.';
    this.backup.textContent = value.backup ? `Backup saved on host: ${value.backup}` : '';
    for (const button of this.buttons) button.disabled = this.sending || !value.online || value.busy || (button === this.topOnlyButton && value.top_only_available !== true);
  }
  async poll() {
    if (this.closed) return;
    try {
      const response = await this.request('/api/v1/camera-alignment', {cache: 'no-store'});
      if (!response.ok) throw Error('Camera alignment status unavailable.');
      const value = await response.json();
      if (!this.sending) this.render(value);
    } catch (error) { if (!this.sending) this.render({message: error.message, online: false}); }
    if (!this.closed) this.timer = setTimeout(() => this.poll(), 2000);
  }
  async start(mode, scope = "rig") {
    const payload = scope === "rig" ? {mode} : {mode, scope};
    if (mode === 'aruco' || mode === 'charuco') payload.board_is_tabletop = scope !== 'top_only' && this.root.querySelector('#camera-board-tabletop').checked;
    if (mode === 'charuco') {
      payload.board = {};
      for (const [key,id] of Object.entries({squares_x:'x',squares_y:'y',square_mm:'square',marker_mm:'marker',dictionary:'dictionary'})) {
        const input = this.root.querySelector(`#camera-charuco-${id}`);
        if (!input.reportValidity()) return;
        payload.board[key] = Number(input.value);
      }
      if (payload.board.marker_mm >= payload.board.square_mm) {
        this.status.textContent = 'The marker must be smaller than its checker square.';
        return;
      }
    }
    if (mode === 'aruco') {
      const size = this.root.querySelector('#camera-aruco-size'), id = this.root.querySelector('#camera-aruco-id');
      if (!size.reportValidity() || !id.reportValidity()) return;
      payload.marker_size_mm = Number(size.value);
      payload.marker_id = Number(id.value);
    }
    const confirmation = scope === 'top_only'
      ? 'Put the arm in Hold. Keep the board flat, still and visible to all three cameras. Only A will be saved; B/C, tabletop and arm calibration stay fixed. Existing calibration will be backed up. Continue?'
      : 'Put the arm in Hold and keep all cameras and objects still. The preview will pause while Godot aligns the cameras. Camera and arm calibration will be backed up first. Continue?';
    if (this.sending || !await this.confirm(confirmation)) return;
    this.sending = true;
    this.render({message:'Backing up calibration and requesting Godot alignment…'});
    try {
      const response = await this.request('/api/v1/camera-alignment', {
        method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload)
      });
      if (!response.ok) {
        const text = await response.text();
        const paragraphs = [...new DOMParser().parseFromString(text, 'text/html').querySelectorAll('p')];
        const message = paragraphs.find(p => p.textContent.startsWith('Message:'))?.textContent;
        throw Error(message || 'Camera alignment request failed. Check that recording is stopped and the arm is in Hold.');
      }
      const value = await response.json();
      this.sending = false;
      this.render(value);
    } catch (error) {
      this.sending = false;
      this.render({message: error.message, online: false});
    }
  }
  close() {this.closed = true; clearTimeout(this.timer);}
}
