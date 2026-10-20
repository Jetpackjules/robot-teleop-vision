// Original sensor RGB. This deliberately bypasses depth-aligned RGB-D images.
export class CameraRgbViewer {
  constructor(canvas, options = {}) {
    this.canvas = canvas;
    this.options = { ...options };
    this.running = false;
    this.controllers = new Set();
    this.generation = 0;
    this.lastPacketMs = 0;
    this.lastSequence = -1;
    this.stats = { displaySource: "original_rgb", connected: false, width: 0, height: 0,
      received: 0, decoded: 0, dropped: 0, bytes: 0, sourceFps: 0, renderFps: 0,
      bitrateKbps: 0, cameras: 1, cameraFps: {}, captureUnixMs: 0,
      transport: "Original RGB over HTTPS", status: "Original RGB connecting", renderWarning: "" };
    this.sample = { time: performance.now(), decoded: 0, bytes: 0 };
    this.socket = null;
    this.pending = null;
    this.decoding = false;
  }

  start() {
    if (this.running) return;
    this.running = true;
    this.canvas.getContext("2d", { alpha: false }).clearRect(0, 0, this.canvas.width, this.canvas.height);
    const generation = ++this.generation;
    this.visibilityHandler = () => {
      this.closeSocket();
      if (!document.hidden && this.running) this.connect(this.generation);
    };
    document.addEventListener("visibilitychange", this.visibilityHandler);
    this.connect(generation);
  }

  stop() {
    this.running = false;
    this.generation += 1;
    for (const controller of this.controllers) controller.abort();
    this.controllers.clear();
    this.closeSocket();
    this.pending = null;
    clearTimeout(this.retryTimer);
    if (this.visibilityHandler) document.removeEventListener("visibilitychange", this.visibilityHandler);
  }

  closeSocket() {
    if (!this.socket) return;
    this.socket.onmessage = this.socket.onclose = this.socket.onerror = null;
    this.socket.close();
    this.socket = null;
  }

  setCamera(id) {
    if (id === (this.options.camera || '')) return;
    const running = this.running;
    this.stop();
    this.options = { ...this.options, camera: id };
    this.cameraId = '';this.captureEpoch = '';this.lastSequence = -1;
    this.stats.captureUnixMs = 0;this.stats.connected = false;
    if (running) this.start();
  }

  connect(generation) {
    if (!this.running || generation !== this.generation || document.hidden) return;
    if (typeof WebSocket === "undefined") { this.poll(generation); this.poll(generation); return; }
    const query = new URLSearchParams({ w: String(this.options.width || 3840), q: String(this.options.quality || 95), camera: this.options.camera || '' });
    const scheme = location.protocol === "https:" ? "wss" : "ws";
    const socket = new WebSocket(`${scheme}://${location.host}/camera-rgb-stream?${query}`);
    this.socket = socket;
    socket.binaryType = "arraybuffer";
    this.stats.transport = "Original RGB WebSocket";
    socket.onmessage = event => {
      if (this.socket !== socket || !this.running || generation !== this.generation || document.hidden) return;
      if (!(event.data instanceof ArrayBuffer) || event.data.byteLength < 5) return;
      const bytes = new Uint8Array(event.data), length = new DataView(event.data).getUint32(0, true);
      if (length > 4096 || length + 4 >= bytes.length || bytes.length > 16 * 1024 * 1024 + 4100) return;
      try {
        const metadata = JSON.parse(new TextDecoder().decode(bytes.subarray(4, length + 4)));
        if (this.pending) this.stats.dropped++;
        this.pending = { blob: new Blob([bytes.subarray(length + 4)], { type: "image/jpeg" }), metadata };
        this.drain(generation);
      } catch (_) { this.stats.renderWarning = "Invalid original RGB packet"; }
    };
    socket.onerror = () => { this.stats.status = "Original RGB reconnecting"; };
    socket.onclose = () => {
      if (this.socket !== socket || !this.running || generation !== this.generation) return;
      this.socket = null;
      this.stats.connected = false;
      this.retryTimer = setTimeout(() => this.connect(generation), 500);
    };
  }

  async drain(generation) {
    if (this.decoding) return;
    this.decoding = true;
    try {
      while (this.pending && this.running && generation === this.generation) {
        const { blob, metadata } = this.pending;
        this.pending = null;
        await this.displayFrame(blob, metadata, generation);
      }
    } catch (error) { this.stats.renderWarning = error.message; }
    finally {
      this.decoding = false;
      if (this.pending && this.running) this.drain(this.generation);
    }
  }

  async displayFrame(blob, metadata, generation) {
    const id = metadata["x-camera-id"] || "camera";
    if (this.options.camera && id !== this.options.camera) return;
    const sequence = Number(metadata["x-frame-sequence"] ?? -1);
    const capture = Number(metadata["x-capture-unix-ms"] || 0);
    const epoch = metadata["x-capture-epoch"] || "";
    const stale = () => id === this.cameraId && epoch === this.captureEpoch && sequence >= 0 && sequence <= this.lastSequence;
    if (stale() || (epoch !== this.captureEpoch && capture < this.stats.captureUnixMs)) return;
    this.stats.received++;
    this.stats.bytes += blob.size;
    const bitmap = await createImageBitmap(blob);
    try {
      if (!this.running || generation !== this.generation || stale()) return;
      if (this.canvas.width !== bitmap.width || this.canvas.height !== bitmap.height) {
        this.canvas.width = bitmap.width; this.canvas.height = bitmap.height;
      }
      this.canvas.getContext("2d", { alpha: false }).drawImage(bitmap, 0, 0);
      this.lastPacketMs = performance.now();
      this.lastSequence = sequence; this.cameraId = id; this.captureEpoch = epoch;
      Object.assign(this.stats, { connected: true, status: "Original camera RGB", renderWarning: "",
        width: bitmap.width, height: bitmap.height, decoded: this.stats.decoded + 1, captureUnixMs: capture });
    } finally { bitmap.close(); }
  }

  async poll(generation) {
    while (this.running && generation === this.generation) {
      const started = performance.now();
      if (!document.hidden) await this.fetchFrame(generation);
      await new Promise((resolve) => setTimeout(resolve, Math.max(20, 1000 / 30 - (performance.now() - started))));
    }
  }

  async fetchFrame(generation) {
    const controller = new AbortController();
    this.controllers.add(controller);
    const timeout = setTimeout(() => controller.abort(), 10000);
    try {
      const query = new URLSearchParams({ w: String(this.options.width || 3840), q: String(this.options.quality || 95), camera: this.options.camera || '' });
      const response = await fetch(`/camera-rgb?${query}`, { cache: "no-store", signal: controller.signal });
      if (!response.ok || !response.headers.get("Content-Type")?.startsWith("image/jpeg")) {
        throw new Error(response.status === 503 ? "Original camera RGB not ready" : `Original RGB HTTP ${response.status}`);
      }
      const blob = await response.blob();
      if (!this.running || generation !== this.generation) return;
      const metadata = Object.fromEntries(["X-Camera-Id", "X-Frame-Sequence", "X-Capture-Unix-Ms", "X-Capture-Epoch"]
        .map(key => [key.toLowerCase(), response.headers.get(key)]));
      await this.displayFrame(blob, metadata, generation);
    } catch (error) {
      if (!this.running || generation !== this.generation) return;
      this.stats.status = error.name === "AbortError" ? "Original RGB timed out" : error.message;
      this.stats.renderWarning = this.stats.status;
      await new Promise((resolve) => setTimeout(resolve, 500));
    } finally {
      clearTimeout(timeout);
      this.controllers.delete(controller);
    }
  }

  status() {
    const now = performance.now();
    const elapsed = now - this.sample.time;
    if (elapsed >= 1000) {
      this.stats.sourceFps = (this.stats.decoded - this.sample.decoded) * 1000 / elapsed;
      this.stats.renderFps = this.stats.sourceFps;
      this.stats.bitrateKbps = (this.stats.bytes - this.sample.bytes) * 8 / elapsed;
      this.stats.cameraFps = { [this.cameraId || "camera"]: this.stats.sourceFps };
      this.sample = { time: now, decoded: this.stats.decoded, bytes: this.stats.bytes };
    }
    const packetAgeMs = this.lastPacketMs ? now - this.lastPacketMs : -1;
    return { ...this.stats, packetAgeMs, stalled: this.stats.connected && packetAgeMs > 3000,
      captureAgeMs: this.stats.captureUnixMs ? Math.max(0, Date.now() - this.stats.captureUnixMs) : -1 };
  }
}
