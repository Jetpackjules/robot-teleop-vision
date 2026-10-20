const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(require('node:path').join(__dirname,'../web/camera_rgb_viewer.js'),'utf8').replace(/^export /gm,'');
const requests=[],painted=[],closed=[];
const context=vm.createContext({performance,AbortController,URLSearchParams,setTimeout,clearTimeout,
  Blob,TextDecoder,ArrayBuffer,Uint8Array,DataView,location:{protocol:'https:',host:'viewer.example'},
  document:{hidden:false,addEventListener(){},removeEventListener(){}},fetch:(_,options)=>new Promise(resolve=>requests.push({resolve,options})),
  createImageBitmap:async blob=>({width:1280,height:720,id:blob.id,close(){closed.push(this.id);}})});
vm.runInContext(source+'\nthis.Viewer=CameraRgbViewer;',context);
const canvas={width:0,height:0,getContext:()=>({drawImage:b=>painted.push(b.id),clearRect(){}})};
const viewer=new context.Viewer(canvas);viewer.running=true;viewer.generation=1;
const response=id=>({ok:true,headers:{get:name=>({'Content-Type':'image/jpeg','X-Camera-Id':'1234',
  'X-Frame-Sequence':String(id),'X-Capture-Unix-Ms':String(Date.now())}[name]||null)},blob:async()=>({size:100,id})});
(async()=>{
  const older=viewer.fetchFrame(1),newer=viewer.fetchFrame(1);
  requests[1].resolve(response(20));await newer;
  requests[0].resolve(response(10));await older;
  assert.deepEqual(painted,[20],'out-of-order response must not overwrite newest image');
  assert.equal(canvas.width,1280);assert.equal(canvas.height,720);
  assert.equal(viewer.status().decoded,1,'duplicate/old captures do not inflate FPS');
  const pending=viewer.fetchFrame(1);viewer.stop();
  assert.ok(requests[2].options.signal.aborted,'mode exit aborts requests');
  requests[2].resolve(response(30));await pending;
  assert.deepEqual(painted,[20],'late responses cannot repaint after mode exit');
  assert.deepEqual(closed,[20],'decoded image is released');
  const sockets=[],decodes=[];
  context.WebSocket=class {
    constructor(url){this.url=url;sockets.push(this);}
    close(){this.closed=true;}
  };
  context.createImageBitmap=blob=>new Promise(resolve=>decodes.push({blob,resolve}));
  const stream=new context.Viewer(canvas);stream.start();
  assert.match(sockets[0].url,/^wss:\/\/viewer.example\/camera-rgb-stream\?/,'persistent connection uses the authenticated same origin');
  const packet=(sequence,epoch='ownerA')=>{
    const metadata=Buffer.from(JSON.stringify({'x-camera-id':'1234','x-frame-sequence':String(sequence),
      'x-capture-unix-ms':String((epoch==='ownerB'?20000:10000)+sequence),'x-capture-epoch':epoch}));
    const bytes=new Uint8Array(4+metadata.length+1);new DataView(bytes.buffer).setUint32(0,metadata.length,true);
    bytes.set(metadata,4);bytes[bytes.length-1]=255;return {data:bytes.buffer};
  };
  sockets[0].onmessage(packet(100));sockets[0].onmessage(packet(101));sockets[0].onmessage(packet(102));
  assert.equal(decodes.length,1,'only one image decode is active');
  assert.equal(stream.stats.dropped,1,'intermediate queued frame is replaced rather than accumulated');
  const settle=()=>new Promise(resolve=>setImmediate(resolve));
  decodes[0].resolve({width:960,height:540,id:100,close(){closed.push(100);}});await settle();
  assert.equal(decodes.length,2,'newest pending image follows the in-flight decode');
  decodes[1].resolve({width:960,height:540,id:102,close(){closed.push(102);}});await settle();
  assert.deepEqual(painted,[20,100,102]);
  assert.equal(stream.stats.decoded,2,'fresh decoded frames drive FPS');
  sockets[0].onmessage(packet(1,'ownerB'));
  decodes[2].resolve({width:960,height:540,id:1,close(){closed.push(1);}});await settle();
  assert.equal(stream.lastSequence,1,'capture-owner epoch permits a restarted sequence');
  sockets[0].onmessage(packet(2,'ownerB'));stream.stop();
  decodes[3].resolve({width:960,height:540,id:2,close(){closed.push(2);}});await settle();
  assert.equal(painted.at(-1),1,'stopped stream cannot paint a late decode');
  assert.ok(sockets[0].closed,'mode exit closes the persistent stream');
  const sharedOptions={camera:'1234'};
  const selected=new context.Viewer(canvas,sharedOptions);selected.start();
  assert.equal(new URL(sockets.at(-1).url).searchParams.get('camera'),'1234','selected camera is sent to the stream endpoint');
  sockets.at(-1).onmessage(packet(4,'ownerB'));
  const obsolete=decodes.at(-1);
  sharedOptions.camera='5678';selected.setCamera('5678');
  obsolete.resolve({width:960,height:540,id:4,close(){}});await settle();
  assert.equal(painted.at(-1),1,'an old camera decode cannot repaint after selecting another camera');
  assert.equal(new URL(sockets.at(-1).url).searchParams.get('camera'),'5678','camera selection reconnects to the new sensor');
  const decodeCount=decodes.length;
  sockets.at(-1).onmessage(packet(5,'ownerB'));await settle();
  assert.equal(decodes.length,decodeCount,'mismatched camera frames are rejected before decoding');
  selected.stop();
  let now=100,timerId=0;
  const timers=new Map(),quietSockets=[];
  const quietContext=vm.createContext({performance:{now:()=>now},AbortController,URLSearchParams,
    setTimeout:(fn,ms)=>{const id=++timerId;timers.set(id,{fn,at:now+ms});return id;},
    clearTimeout:id=>timers.delete(id),Blob,TextDecoder,ArrayBuffer,Uint8Array,DataView,
    location:{protocol:'https:',host:'viewer.example'},document:context.document,
    WebSocket:class {constructor(){quietSockets.push(this);}close(){this.closed=true;}}});
  vm.runInContext(source+'\nthis.Viewer=CameraRgbViewer;',quietContext);
  const quiet=new quietContext.Viewer(canvas);quiet.start();
  const tick=ms=>{now+=ms;for(const [id,t] of [...timers])if(t.at<=now){timers.delete(id);t.fn();}};
  tick(21000);assert.ok(quietSockets[0].closed,'socket with no first frame must recover automatically');
  tick(500);assert.equal(quietSockets.length,2,'watchdog retries without reloading the page');
  quiet.lastPacketMs=now;tick(1000);
  assert.equal(quietSockets.length,2,'a fresh frame keeps the socket');
  tick(8001);assert.ok(quietSockets[1].closed,'a previously live but silent socket must also recover');
  quiet.visibilityHandler();
  const countAfterVisible=quietSockets.length;
  tick(600);
  assert.equal(quietSockets.length,countAfterVisible,'visibility recovery cancels the already scheduled retry');
  quiet.stop();assert.equal(timers.size,0,'mode exit cancels watchdog and reconnect timers');
  console.log('Original RGB viewer: ordering, native size, FPS, cancellation and disposal passed');
})().catch(error=>{console.error(error);process.exitCode=1;});
