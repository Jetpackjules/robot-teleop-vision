const assert=require('node:assert/strict'),fs=require('node:fs');
(async()=>{
 const {IndependentCameraSocket}=await import('data:text/javascript;base64,'+Buffer.from(fs.readFileSync('web/independent_camera_socket.js')).toString('base64'));
 const sockets=[];class Socket{constructor(url){this.url=url;this.readyState=0;this.sent=[];sockets.push(this);}send(x){this.sent.push(JSON.parse(x));}close(){this.readyState=3;this.closed=true;}}
 const group=new IndependentCameraSocket('wss://example.test/rgbd-stream?client=test',{Socket,fetcher:async()=>({ok:true,json:async()=>({independent_native_preview:true,cameras:[{id:'a'},{id:'b'},{id:'c'}]})})});
 const received=[];group.onmessage=e=>received.push(e.data);let opens=0;group.onopen=()=>opens++;
 await new Promise(r=>setTimeout(r,0));assert.equal(sockets.length,3);
 for(const s of sockets){s.readyState=1;s.onopen({});}assert.equal(opens,1);
 // Camera a is stalled: b and c must continue, with independent acknowledgements.
 for(const s of sockets.slice(1)){s.onmessage({data:JSON.stringify({type:'rgbd-sequence',sequence:7})});const bytes=new Uint8Array([1,2,3]).buffer;s.onmessage({data:bytes});assert.equal(received.at(-1),bytes);assert.deepEqual(s.sent,[{type:'rgbd-ack',sequence:7}]);}
 assert.equal(sockets[0].sent.length,0);assert.equal(received.length,2);
 sockets[0].onclose({code:1006});await new Promise(r=>setTimeout(r,550));assert.equal(sockets.length,4);assert.equal(group.readyState,1);
 group.close();await new Promise(r=>setTimeout(r,550));assert.equal(sockets.length,4);assert(sockets.slice(1).every(s=>s.closed));assert.equal(group.readyState,3);
 const second=new IndependentCameraSocket('wss://example.test/rgbd-stream',{Socket,fetcher:async()=>({ok:true,json:async()=>({independent_native_preview:true,cameras:[{id:'a'},{id:'b'}]})})});await new Promise(r=>setTimeout(r,0));let code;second.onclose=e=>code=e.code;sockets.at(-1).onclose({code:4001});assert.equal(second.readyState,3);assert.equal(code,4001);
 const beforeFallback=sockets.length;
 const fallback=new IndependentCameraSocket('wss://example.test/rgbd-stream?camera_lanes=2',{Socket,fetcher:async()=>({ok:true,json:async()=>({cameras:[{id:'a'}]})})});await new Promise(r=>setTimeout(r,0));assert.equal(sockets.length,beforeFallback+1);assert(!new URL(sockets.at(-1).url).searchParams.has('camera_id'));assert(!new URL(sockets.at(-1).url).searchParams.has('camera_lanes'));fallback.close();
 let finish;const pending=new IndependentCameraSocket('wss://example.test/rgbd-stream',{Socket,fetcher:()=>new Promise(r=>finish=r)});pending.close();const count=sockets.length;finish({ok:true,json:async()=>({independent_native_preview:true,cameras:[{id:'a'}]})});await new Promise(r=>setTimeout(r,0));assert.equal(sockets.length,count);
 const striped=new IndependentCameraSocket('wss://example.test/rgbd-stream?camera_lanes=2',{Socket,fetcher:async()=>({ok:true,json:async()=>({independent_native_preview:true,independent_native_preview_lanes:2,cameras:[{id:'a'}]})})});await new Promise(r=>setTimeout(r,0));assert.equal(striped.children.size,2);const lanes=[...striped.children.values()];for(const socket of lanes)socket.readyState=1;
 const seen=[];striped.onmessage=e=>seen.push(e.data);
 function packet(epoch,publication,id='a'){const meta=new TextEncoder().encode(JSON.stringify({cameras:[{id,preview_epoch:String(epoch),preview_publication:String(publication)}]}));const data=new Uint8Array(9+meta.length+3);data.set(new TextEncoder().encode('RGBD1'));new DataView(data.buffer).setUint32(5,meta.length,true);data.set(meta,9);data.set([7,8,9],9+meta.length);return data.buffer;}
 function deliver(lane,sequence,data){lanes[lane].onmessage({data:JSON.stringify({type:'rgbd-sequence',sequence})});lanes[lane].onmessage({data});}
 const first=packet(5,2),newer=packet(5,6);deliver(0,1,first);deliver(1,1,newer);deliver(0,2,packet(5,4));assert.deepEqual(seen,[first,newer]);assert.equal(lanes[0].sent.at(-1).sequence,2,'late capture is acknowledged without rewinding the scene');
 const restarted=packet(6,2);deliver(1,2,restarted);deliver(0,3,packet(5,8));deliver(1,3,packet(6,4,'wrong-camera'));assert.deepEqual(seen,[first,newer,restarted]);assert.equal(lanes[0].sent.at(-1).sequence,3);striped.close();
 const selectSockets=[];class SelectSocket extends Socket{constructor(url){super(url);selectSockets.push(this);}}
 const selected=new IndependentCameraSocket('wss://example.test/rgbd-stream?camera_lanes=2',{Socket:SelectSocket,excludedIds:['top'],fetcher:async()=>({ok:true,json:async()=>({independent_native_preview:true,independent_native_preview_lanes:2,cameras:[{id:'top'},{id:'side'}]})})});
 await new Promise(r=>setTimeout(r,0));assert.equal(selectSockets.length,2);assert(selectSockets.every(s=>new URL(s.url).searchParams.get('camera_id')==='side'));
 selected.setExcluded([]);assert.equal(selectSockets.length,4);const top=selectSockets.slice(2);selected.setExcluded(['top']);assert(top.every(s=>s.closed));assert(selectSockets.slice(0,2).every(s=>!s.closed));selected.setExcluded([]);assert.equal(selectSockets.length,6);selected.close();
 console.log('Independent camera lifecycle, byte identity, ACK routing, isolated reconnect and supersession checks passed');
})().catch(e=>{console.error(e);process.exitCode=1;});
