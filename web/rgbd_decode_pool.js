export function isIndependentNativeFrame(metadata) {
  return (!metadata.depth_encoding || ["u16_mm_deflate", "u16_mm_vpredict_shuffle_deflate"].includes(metadata.depth_encoding))
    && !metadata.depth_temporal_keyframe && !metadata.depth_temporal_reference_mode
    && !metadata.color_temporal_keyframe && !metadata.color_temporal_reference_mode
    && metadata.color_encoding !== "rgb8_keyframe_tiles_jpeg";
}

export function takeIndependentFrameBatch(queue) {
  const independent = frame => frame && frame.metadata.temporal_reference_mode !== "persistent_v1"
    && frame.payloads.every(payload => isIndependentNativeFrame(payload.metadata));
  const first = queue.shift();
  if (!independent(first)) return first;
  const latest = new Map();
  let frame = first, packets = 0;
  do {
    packets++;
    for (const payload of frame.payloads) {
      const id = String(payload.metadata.id || payload.metadata.serial || payload.metadata.model || "camera");
      latest.set(id, {payload, receivedAtMs: frame.receivedAtMs});
    }
    if (!independent(queue[0])) break;
    frame = queue.shift();
  } while (frame);
  const entries = [...latest.values()], payloads = entries.map(entry => entry.payload);
  return {...frame, payloads, metadata: {...frame.metadata, cameras: payloads.map(payload => payload.metadata)},
    receivedAtMs: Math.min(...entries.map(entry => entry.receivedAtMs)), coalescedPackets: packets - 1};
}

export class NativeRgbdDecoderPool {
  constructor({rgbaPixels = true} = {}) {
    this.workers = [];
    this.pending = new Map();
    this.nextId = 0;
    this.disabled = typeof Worker !== "function";
    this.rgbaPixels=rgbaPixels;
  }

  decode(payload) {
    return new Promise((resolve, reject) => {
      if (this.disabled) {reject(new Error("native decoder is unavailable"));return;}
      const id = ++this.nextId, index = (id - 1) % 3;
      let worker = this.workers[index];
      try {
        if (!worker) {
          worker = new Worker(new URL("./rgbd_decode_worker.js?v=20261020_native_pixels", import.meta.url), {type: "module"});
          worker.onmessage = ({data}) => {
            const job = this.pending.get(data.id);
            if (!job) {data.bitmap?.close();return;}
            this.pending.delete(data.id);clearTimeout(job.timer);
            if (data.error) job.reject(new Error(data.error));
            else job.resolve({depth:new Uint16Array(data.depth),bitmap:data.bitmap,
              rgbaPixels:data.rgba?new Uint8Array(data.rgba):null,workerMs:data.workerMs||0,
              dispatchMs:Math.max(0,performance.now()-job.started-(data.workerMs||0))});
          };
          worker.onerror = () => this.close(new Error("native decoder worker failed"));
          worker.onmessageerror = () => this.close(new Error("native decoder transfer failed"));
          this.workers[index] = worker;
        }
        const timer = setTimeout(() => this.close(new Error("native decoder timed out")), 5000);
        this.pending.set(id, {resolve, reject, timer,started:performance.now()});
        worker.postMessage({id, ...payload,rgbaPixels:this.rgbaPixels}, [...new Set([payload.color.buffer, payload.depth.buffer])]);
      } catch (error) {
        // Transferred inputs may already be detached. Drop this frame and use
        // the main-thread compatibility decoder for subsequent frames.
        this.close(error);reject(error);
      }
    });
  }

  close(error = new Error("native decoder stopped")) {
    this.disabled = true;
    for (const worker of this.workers) worker?.terminate();
    this.workers.length = 0;
    for (const job of this.pending.values()) {clearTimeout(job.timer);job.reject(error);}
    this.pending.clear();
  }
}
