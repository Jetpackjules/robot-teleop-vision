export class CameraAlignmentControl {
  constructor(root, {request = (...args) => fetch(...args), confirm = message => window.confirm(message)} = {}) {
    this.root = root;
    this.request = request;
    this.confirm = confirm;
    this.closed = false;
    this.sending = false;
    this.status = root.querySelector('#camera-alignment-status');
    this.backup = root.querySelector('#camera-alignment-backup');
    this.buttons = [root.querySelector('#camera-align-auto'), root.querySelector('#camera-align-refine')];
    this.buttons[0].addEventListener('click', () => this.start('markerless'));
    this.buttons[1].addEventListener('click', () => this.start('refine'));
    this.poll();
  }
  render(value) {
    this.status.textContent = value.message || 'Camera alignment is ready.';
    this.backup.textContent = value.backup ? `Backup saved on host: ${value.backup}` : '';
    for (const button of this.buttons) button.disabled = this.sending || !value.online || value.busy;
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
  async start(mode) {
    if (this.sending || !await this.confirm('Put the arm in Hold and keep all cameras and objects still. The preview will pause while Godot aligns the cameras. Camera and arm calibration will be backed up first. Continue?')) return;
    this.sending = true;
    this.render({message:'Backing up calibration and requesting Godot alignment…'});
    try {
      const response = await this.request('/api/v1/camera-alignment', {
        method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({mode})
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
