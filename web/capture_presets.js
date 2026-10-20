export function nativeStreamPreset(preset) {
  const fps = preset === "latency" ? 60 : 30;
  return {width:3840,fps,contextFps:fps,detailFps:fps,quality:95,splatFill:1.32,exactDepth:true};
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
