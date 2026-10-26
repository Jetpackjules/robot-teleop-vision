export function nativeStreamPreset(preset) {
  const fps = preset === "latency" ? 60 : 30;
  return {width:3840,fps,contextFps:fps,detailFps:fps,quality:95,splatFill:1.32,exactDepth:true};
}

// Workspace filtering must also work when head tracking and WebRTC are off.
// Only viewing bounds enter this reconnect queue, never robot/calibration actions.
const WORKSPACE_KEYS = ['workspace_stream_crop_enabled', 'display_mode', 'workspace_crop_enabled',
  'workspace_crop_min', 'workspace_crop_max', 'workspace_crop_rotation', 'modeled_workspace_enabled',
  'modeled_workspace_center', 'modeled_workspace_width', 'modeled_workspace_depth',
  'modeled_workspace_height', 'modeled_workspace_heading', 'modeled_workspace_clearance'];

export class WorkspaceStreamControl {
  constructor(url) { this.url = url; this.socket = null; this.latest = null; this.retry = null; this.connectTimer = null; this.closed = false; }

  update(settings) {
    if (this.closed || !/^[A-Za-z0-9_-]{1,64}$/.test(settings.rgbd_client_id || '')) return false;
    const payload = {type: 'view_settings', rgbd_client_id: settings.rgbd_client_id};
    for (const key of WORKSPACE_KEYS) if (Object.hasOwn(settings, key)) payload[key] = settings[key];
    const latest = JSON.stringify(payload);
    if (latest === this.latest) return true;
    this.latest = latest;
    this.flush();
    return true;
  }

  schedule() {
    if (this.closed || this.retry !== null) return;
    this.retry = setTimeout(() => { this.retry = null; this.flush(); }, 500);
  }

  flush() {
    if (this.closed || !this.latest) return;
    if (this.socket?.readyState === 1) {
      if (this.socket.bufferedAmount > 16384) { this.schedule(); return; }
      try { this.socket.send(this.latest); }
      catch (_) { this.socket.close(); this.schedule(); }
      return;
    }
    if (this.socket?.readyState === 0) return;
    clearTimeout(this.retry); this.retry = null;
    const socket = new WebSocket(this.url);
    this.socket = socket;
    socket.onopen = () => {
      if (this.socket !== socket) return;
      clearTimeout(this.connectTimer); this.connectTimer = null;
      this.flush();
    };
    const lost = () => {
      if (this.socket !== socket) return;
      clearTimeout(this.connectTimer); this.connectTimer = null;
      this.socket = null;
      socket.onopen = socket.onclose = socket.onerror = null;
      socket.close();
      this.schedule();
    };
    socket.onclose = socket.onerror = lost;
    this.connectTimer = setTimeout(lost,5000);
  }

  close() {
    this.closed = true;
    clearTimeout(this.retry); this.retry = null;
    clearTimeout(this.connectTimer); this.connectTimer = null;
    if (this.socket) {
      this.socket.onopen = this.socket.onclose = this.socket.onerror = null;
      this.socket.close(); this.socket = null;
    }
    this.latest = null;
  }
}

// Reuse the authenticated view-settings protocol without depending on head
// tracking being enabled. Only one bounded mode request can be pending.
export class CapturePresetControl {
  constructor(url) {this.url=url;this.pending=null;}

  cancel() {this.pending?.finish(false);}

  request(preset) {
    this.cancel();
    if(!["quality","latency"].includes(preset))return Promise.reject(new Error("Unknown camera preset"));
    return new Promise((resolve,reject)=>{
      const socket=new WebSocket(this.url);
      let done=false,timer;
      const finish=(applied,error=null)=>{
        if(done)return;
        done=true;clearTimeout(timer);
        socket.onopen=socket.onerror=socket.onclose=null;
        if(this.pending?.socket===socket)this.pending=null;
        socket.close();
        if(error)reject(error);else resolve(applied);
      };
      this.pending={socket,finish};
      timer=setTimeout(()=>finish(false,new Error("Camera mode request timed out; reconnect and retry")),5000);
      socket.onopen=()=>{
        if(done)return;
        try {socket.send(JSON.stringify({type:"view_settings",stream_preset:preset}));finish(true);}
        catch(error){finish(false,error);}
      };
      socket.onerror=socket.onclose=()=>finish(false,new Error("Could not change camera capture mode; reconnect and retry"));
    });
  }
}
