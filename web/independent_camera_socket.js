// Independent reliable camera streams; selected points stay byte-exact.
export class IndependentCameraSocket {
  constructor(url,{Socket=WebSocket,fetcher=(...args)=>fetch(...args)}={}) {
    this.url=url;this.Socket=Socket;this.fetcher=fetcher;this.readyState=0;this.binaryType='arraybuffer';
    this.children=new Map();this.timers=new Set();this.closed=false;this.opened=false;
    this.lanes=1;this.latest=new Map();this.decoder=new TextDecoder();
    this.initialize();
  }
  later(fn,ms){const timer=setTimeout(()=>{this.timers.delete(timer);if(!this.closed)fn();},ms);this.timers.add(timer);}
  async initialize(){
    let ids=[''];
    try{
      const response=await this.fetcher('/api/v1/cameras',{signal:AbortSignal.timeout(3000)});
      if(!response.ok)throw Error('camera discovery unavailable');
      const data=await response.json();
      if(data.independent_native_preview!==true)throw Error('host uses a single preview connection');
      const found=[...new Set(data.cameras.map(c=>c.id))];
      if(!found.length||found.length>8||found.some(id=>typeof id!=='string'||!/^[A-Za-z0-9_-]{1,64}$/.test(id)))throw Error('invalid cameras');
      ids=found;
      const requested=Number(new URL(this.url).searchParams.get('camera_lanes')),supported=Number(data.independent_native_preview_lanes||1);
      if(requested===2&&supported===2)this.lanes=2;
    }catch(_error){this.fallbackReason=String(_error.message);/* Older hosts retain the original single connection. */}
    if(!this.closed)for(const id of ids)for(let lane=0;lane<this.lanes;lane++)this.connect(id,lane);
  }
  acceptCapture(id,buffer){
    try{
      const bytes=new Uint8Array(buffer),view=new DataView(buffer);
      if(bytes.length<9||this.decoder.decode(bytes.subarray(0,5))!=='RGBD1')return false;
      const length=view.getUint32(5,true);
      if(length>65536||length>bytes.length-9)return false;
      const metadata=JSON.parse(this.decoder.decode(bytes.subarray(9,9+length)));
      const camera=metadata.cameras?.[0];
      if(metadata.cameras.length!==1||camera.id!==id||!/^\d{1,20}$/.test(camera.preview_publication)||!/^\d{1,20}$/.test(camera.preview_epoch))return false;
      const publication=BigInt(camera.preview_publication),epoch=camera.preview_epoch;
      let prior=this.latest.get(id);
      if(!prior){prior={epoch,publication,retired:new Set()};this.latest.set(id,prior);return true;}
      if(prior.epoch!==epoch){
        if(prior.retired.has(epoch))return false;
        prior.retired.add(prior.epoch);if(prior.retired.size>8)prior.retired.delete(prior.retired.values().next().value);
        prior.epoch=epoch;prior.publication=publication;return true;
      }
      if(publication<=prior.publication)return false;
      prior.publication=publication;return true;
    }catch(_error){return false;}
  }
  connect(id,lane=0){
    if(this.closed)return;
    const key=id+'/'+lane,url=new URL(this.url);if(id)url.searchParams.set('camera_id',id);
    url.searchParams.delete('camera_lanes');url.searchParams.delete('camera_lane');
    if(this.lanes>1){url.searchParams.set('camera_lanes',String(this.lanes));url.searchParams.set('camera_lane',String(lane));}
    const socket=new this.Socket(url.href);socket.binaryType='arraybuffer';this.children.set(key,socket);
    socket.onopen=()=>{if(this.closed||this.children.get(key)!==socket)return;this.readyState=1;if(!this.opened){this.opened=true;this.onopen?.({});}};
    socket.onmessage=event=>{
      if(this.closed||this.children.get(key)!==socket)return;
      if(typeof event.data==='string'){
        try{const data=JSON.parse(event.data);if(data.type==='rgbd-sequence'&&Number.isSafeInteger(data.sequence)&&data.sequence>0)socket.sequence=data.sequence;}catch(_error){}
        return;
      }
      const accepted=this.lanes===1||this.acceptCapture(id,event.data);
      if(accepted)this.onmessage?.(event);
      if(socket.sequence&&socket.readyState===1){socket.send(JSON.stringify({type:'rgbd-ack',sequence:socket.sequence}));socket.sequence=0;}
    };
    socket.onerror=event=>this.onerror?.(event);
    socket.onclose=event=>{
      if(this.closed||this.children.get(key)!==socket)return;
      this.children.delete(key);
      if(event.code===4001){this.close(1000,'superseded');this.onclose?.(event);return;}
      this.later(()=>this.connect(id,lane),500);
    };
  }
  send(){/* Each child acknowledges its own sequence immediately after delivery. */}
  close(code=1000,reason='preview closed'){
    if(this.closed)return;this.closed=true;this.readyState=3;
    for(const timer of this.timers)clearTimeout(timer);this.timers.clear();
    for(const socket of this.children.values()){socket.onopen=socket.onmessage=socket.onclose=socket.onerror=null;socket.close(code,reason);}
    this.children.clear();
    this.latest.clear();
  }
}
