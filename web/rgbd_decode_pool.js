export function isIndependentNativeFrame(metadata) {
  return (!metadata.depth_encoding || ["u16_mm_deflate", "u16_mm_vpredict_shuffle_deflate", "u16_mm_sparse_points_deflate"].includes(metadata.depth_encoding))
    && !metadata.depth_temporal_keyframe && !metadata.depth_temporal_reference_mode
    && !metadata.color_temporal_keyframe && !metadata.color_temporal_reference_mode
    && metadata.color_encoding !== "rgb8_keyframe_tiles_jpeg";
}

export function takeIndependentFrameBatch(queue) {
  const independent = frame => frame && frame.metadata.temporal_reference_mode !== "persistent_v1"
    && frame.payloads.every(payload => isIndependentNativeFrame(payload.metadata));
  const first = queue.shift();
  if (!independent(first)) return first;
  const compact = frame => independent(frame) && frame.payloads.length === 1
    && frame.payloads[0].metadata.depth_encoding === 'u16_mm_sparse_points_deflate';
  if(compact(first)) {
    const frames=[first];while(compact(queue[0]))frames.push(queue.shift());
    const cameras=new Map(),now=performance.now();
    for(const frame of frames) {
      const p=frame.payloads[0],id=String(p.metadata.id||p.metadata.serial||p.metadata.model||'camera');
      const entries=cameras.get(id)||[];entries.push(frame);cameras.set(id,entries);
    }
    const selected=[],remaining=new Set();
    for(const entries of cameras.values()) {
      while(entries.length>3)entries.shift();
      while(entries.length>1&&now-entries[0].receivedAtMs>50)entries.shift();
      selected.push(entries.shift());for(const frame of entries)remaining.add(frame);
    }
    queue.unshift(...frames.filter(frame=>remaining.has(frame)));
    const payloads=selected.map(frame=>({...frame.payloads[0],receivedAtMs:frame.receivedAtMs}));
    const newest=selected.reduce((a,b)=>a.receivedAtMs>b.receivedAtMs?a:b);
    return {...newest,payloads,metadata:{...newest.metadata,cameras:payloads.map(p=>p.metadata)},
      receivedAtMs:Math.min(...selected.map(frame=>frame.receivedAtMs)),
      coalescedPackets:frames.length-selected.length-remaining.size};
  }
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
  constructor({rgbaPixels = true,softwareColor=true} = {}) {
    this.workers = [];
    this.pending = new Map();
    this.nextId = 0;
    this.disabled = typeof Worker !== "function";
    this.rgbaPixels=rgbaPixels;
    this.softwareColor=softwareColor;
    this.retryAt=Infinity;
    this.recoveryAttempts=0;
    this.restarts=0;
    this.lastError='';
  }

  decode(payload) {
    return new Promise((resolve, reject) => {
      if (this.disabled) {reject(new Error("native decoder is unavailable"));return;}
      const sparse=payload.metadata.depth_encoding==="u16_mm_sparse_points_deflate";
      const id = ++this.nextId, index = (id - 1) % 3+(sparse?3:0);
      let worker = this.workers[index];
      try {
        if (!worker) {
          worker = new Worker(new URL(sparse?"./sparse_point_decode_worker.js?v=20261028_near60":"./rgbd_decode_worker.js?v=20261028_near60", import.meta.url), {type: "module"});
          worker.onmessage = ({data}) => {
            const job = this.pending.get(data.id);
            if (!job) {data.bitmap?.close();return;}
            this.pending.delete(data.id);clearTimeout(job.timer);
            if(!data.error)this.recoveryAttempts=0;
            if (data.error) job.reject(new Error(data.error));
            else if(data.previewPacked){
              const count=data.previewCount,buffer=data.previewPacked;
              job.resolve({depth:null,bitmap:null,rgbaPixels:null,depthTiles:null,validPoints:data.previewValid,
                previewSamples:{indices:new Uint32Array(buffer,0,count),depths:new Uint16Array(buffer,count*4,count),colors:new Uint8Array(buffer,count*6,count*3),valid:data.previewValid,budget:data.previewBudget},
                workerMs:data.workerMs||0,dispatchMs:Math.max(0,performance.now()-job.started-(data.workerMs||0))});
            }
            else job.resolve({depth:new Uint16Array(data.depth),bitmap:data.bitmap,
              depthTiles:data.depthTiles?new Uint16Array(data.depthTiles):null,
              validPoints:data.previewValid,previewSamples:data.previewIndices?{indices:new Uint32Array(data.previewIndices),valid:data.previewValid,budget:data.previewBudget}:null,
              rgbaPixels:data.rgba?new Uint8Array(data.rgba):null,workerMs:data.workerMs||0,
              dispatchMs:Math.max(0,performance.now()-job.started-(data.workerMs||0))});
          };
          worker.onerror = () => {if(this.workers[index]===worker)this.fail(new Error("native decoder worker failed"));};
          worker.onmessageerror = () => {if(this.workers[index]===worker)this.fail(new Error("native decoder transfer failed"));};
          this.workers[index] = worker;
        }
        const timer = setTimeout(() => {if(this.pending.has(id))this.fail(new Error("native decoder timed out"));}, 5000);
        this.pending.set(id, {resolve, reject, timer,started:performance.now()});
        worker.postMessage({id, ...payload,rgbaPixels:this.rgbaPixels,softwareColor:this.softwareColor}, [...new Set([payload.color.buffer, payload.depth.buffer])]);
      } catch (error) {
        // Transferred inputs may already be detached. Drop this frame and use
        // the main-thread compatibility decoder for subsequent frames.
        this.fail(error);reject(error);
      }
    });
  }

  fail(error) {
    const retry=!this.disabled&&typeof Worker==='function'&&this.recoveryAttempts<3;
    const attempt=this.recoveryAttempts;
    this.close(error);
    this.lastError=String(error?.message||error);
    if(retry){this.retryAt=performance.now()+[1000,5000,15000][attempt];this.recoveryAttempts=attempt+1;}
  }

  tryRecover(now=performance.now()) {
    if(!this.disabled||!Number.isFinite(this.retryAt)||now<this.retryAt)return false;
    this.disabled=false;this.retryAt=Infinity;this.restarts++;
    return true;
  }

  close(error = new Error("native decoder stopped")) {
    this.disabled = true;
    this.retryAt=Infinity;
    for (const worker of this.workers) worker?.terminate();
    this.workers.length = 0;
    for (const job of this.pending.values()) {clearTimeout(job.timer);job.reject(error);}
    this.pending.clear();
  }
}
