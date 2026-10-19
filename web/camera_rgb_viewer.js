// Original sensor RGB. This deliberately bypasses depth-aligned RGB-D images.
export class CameraRgbViewer {
  constructor(canvas, options = {}) {
    this.canvas = canvas;
    this.options = options;
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
  }

  start() {
    if (this.running) return;
    this.running = true;
    this.canvas.getContext("2d", { alpha: false }).clearRect(0, 0, this.canvas.width, this.canvas.height);
    const generation = ++this.generation;
    // Two bounded requests cover WAN round trips without queuing old frames.
    this.poll(generation);
    this.poll(generation);
  }

  stop() {
    this.running = false;
    this.generation += 1;
    for (const controller of this.controllers) controller.abort();
    this.controllers.clear();
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
    let bitmap = null;
    try {
      const query = new URLSearchParams({ w: String(this.options.width || 3840), q: String(this.options.quality || 95) });
      const response = await fetch(`/camera-rgb?${query}`, { cache: "no-store", signal: controller.signal });
      if (!response.ok || !response.headers.get("Content-Type")?.startsWith("image/jpeg")) {
        throw new Error(response.status === 503 ? "Original camera RGB not ready" : `Original RGB HTTP ${response.status}`);
      }
      const blob = await response.blob();
      if (!this.running || generation !== this.generation) return;
      this.stats.received += 1;
      this.stats.bytes += blob.size;
      const id = response.headers.get("X-Camera-Id") || "camera";
      const sequence = Number(response.headers.get("X-Frame-Sequence") ?? -1);
      if (id === this.cameraId && sequence >= 0 && sequence <= this.lastSequence) return;
      bitmap = await createImageBitmap(blob);
      if (!this.running || generation !== this.generation) return;
      // An earlier response may finish decoding after a newer response.
      if (id === this.cameraId && sequence >= 0 && sequence <= this.lastSequence) return;
      if (this.canvas.width !== bitmap.width || this.canvas.height !== bitmap.height) {
        this.canvas.width = bitmap.width;
        this.canvas.height = bitmap.height;
      }
      this.canvas.getContext("2d", { alpha: false }).drawImage(bitmap, 0, 0);
      this.lastPacketMs = performance.now();
      this.lastSequence = sequence;
      this.cameraId = id;
      Object.assign(this.stats, { connected: true, status: "Original camera RGB", renderWarning: "",
        width: bitmap.width, height: bitmap.height, decoded: this.stats.decoded + 1,
        captureUnixMs: Number(response.headers.get("X-Capture-Unix-Ms") || 0) });
    } catch (error) {
      if (!this.running || generation !== this.generation) return;
      this.stats.status = error.name === "AbortError" ? "Original RGB timed out" : error.message;
      this.stats.renderWarning = this.stats.status;
      await new Promise((resolve) => setTimeout(resolve, 500));
    } finally {
      bitmap?.close();
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
