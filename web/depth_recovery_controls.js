const DEPTH_RECOVERY_ROUTE = '/api/v1/realsense-depth-recovery';

async function recoveryRequest(url, options = {}) {
  const response = await fetch(url, {cache:'no-store', ...options});
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(data.error || 'Depth recovery is unavailable.');
  return data;
}

export class DepthRecoveryControl {
  constructor(root) {
    this.camera = root.querySelector('select');
    this.toggle = root.querySelector('input');
    this.status = root.querySelector('[role="status"]');
    this.enabled = [];
    this.camera.addEventListener('change', () => this.refresh());
    this.toggle.addEventListener('change', () => this.save());
    this.load();
  }

  async load() {
    try {
      const [cameraList, settings] = await Promise.all([
        recoveryRequest('/api/v1/cameras'), recoveryRequest(DEPTH_RECOVERY_ROUTE),
      ]);
      if (!settings.available) throw new Error('The host has no RealSense Python SDK.');
      const cameras = cameraList.cameras.filter(camera =>
        /^\d{8,16}$/.test(camera.id) && /\bD435\b/i.test(camera.model || camera.name));
      if (!cameras.length) throw new Error('No D435 cameras are available.');
      for (const camera of cameras) {
        const option = document.createElement('option');
        option.value = camera.id;
        option.textContent = camera.name || camera.id;
        this.camera.append(option);
      }
      this.enabled = settings.enabled;
      this.camera.value = cameras.find(camera => this.enabled.includes(camera.id))?.id || cameras[0].id;
      this.camera.disabled = this.toggle.disabled = false;
      this.refresh();
    } catch (error) {
      this.status.textContent = error.message;
    }
  }

  refresh() {
    this.toggle.checked = this.enabled.includes(this.camera.value);
    this.status.textContent = this.toggle.checked ? 'On for this camera.' : 'Off for this camera.';
  }

  async save() {
    this.camera.disabled = this.toggle.disabled = true;
    this.status.textContent = 'Applying camera setting…';
    try {
      const settings = await recoveryRequest(DEPTH_RECOVERY_ROUTE, {
        method:'POST', headers:{'Content-Type':'application/json'},
        body:JSON.stringify({camera_id:this.camera.value, enabled:this.toggle.checked}),
      });
      this.enabled = settings.enabled;
      this.refresh();
    } catch (error) {
      this.refresh();
      this.status.textContent = error.message;
    } finally {
      this.camera.disabled = this.toggle.disabled = false;
    }
  }
}
