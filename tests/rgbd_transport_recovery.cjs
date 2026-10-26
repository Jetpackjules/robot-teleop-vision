const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const timers = new Map(), sockets = [];
let nextTimer = 1;
const setTimer = (fn, delay) => { const id = nextTimer++; timers.set(id,{fn,delay}); return id; };
const clearTimer = id => timers.delete(id);
class Socket {
  static OPEN = 1;
  constructor(url) { this.url=url; this.readyState=0; sockets.push(this); }
  close() { this.readyState=3; }
  send(text) { (this.messages ||= []).push(JSON.parse(text)); }
}
const context = vm.createContext({
  window:{location:{protocol:'https:',host:'example.test'},setTimeout:setTimer,clearTimeout:clearTimer},
  document:{hidden:false}, WebSocket:Socket, setTimeout:setTimer, clearTimeout:clearTimer,
  performance:{now:()=>100}, URLSearchParams, FUSION_FRAGMENT_SHADER:'', COLOR_PROJECTION_SHADER:'',
  DEPTH_SHADING_SHADER:'', SCENE_LIGHTING_SHADER:'', MODELED_WORKSPACE_DEFAULTS:{},
  WORKSPACE_CROP_SHADER:'', MODELED_WORKSPACE_SHADER:'',
});
const source = fs.readFileSync(path.join(__dirname,'../web/hybrid_rgbd_renderer.js'),'utf8')
  .replace(/^import .*;\r?\n/gm,'').replace(/^export /gm,'');
vm.runInContext(source+'\nwindow.probe={state,startRgbdWebSocket,closeRgbdSocket,gpuFrameReady,markGpuFrame};',context);
const {state,startRgbdWebSocket,closeRgbdSocket} = context.window.probe;
let httpStarts=0;
context.httpSpy=()=>httpStarts++;
vm.runInContext('startRgbdHttpLatest=httpSpy',context);
const options={transport:'websocket',temporalDepth:false,temporalColor:false};
state.running=true; state.rgbdClientId='test'; state.socketOptions=options;
function tick() {
  assert.equal(timers.size,1,'only one bounded reconnect/deadline is pending');
  const [id,{fn}] = [...timers][0];timers.delete(id);fn();
}
function open() { const s=sockets.at(-1);s.readyState=1;s.onopen();return s; }
startRgbdWebSocket(options);
assert.equal(timers.size,1,'a connection that never opens has a deadline');
let first=open();assert.equal(timers.size,0);
assert.ok(first.url.includes('flow_control=1'),'independent native packets request bounded WAN flow control');
vm.runInContext('handleRgbdPacket=()=>{}',context);
first.onmessage({data:JSON.stringify({type:'rgbd-sequence',sequence:7})});
first.onmessage({data:new ArrayBuffer(0)});
assert.deepEqual(first.messages,[{type:'rgbd-ack',sequence:7}],'consumed native packet releases its exact server credit');
first.onclose({code:1006});tick();
assert.equal(sockets.length,2,'a transient close retries WebSocket before HTTPS');
assert.equal(httpStarts,0); open();
// Three consecutive failures are retried; the fourth uses the fallback.
for (let i=0;i<2;i++) {sockets.at(-1).onclose({code:1006});tick();open();}
sockets.at(-1).onclose({code:1006});tick();
assert.equal(httpStarts,1,'retries are bounded when WebSocket remains unavailable');
state.reconnectAttempt=0;
startRgbdWebSocket(options);first=open();first.onclose({code:4001});
assert.equal(timers.size,0,'a superseded page must never take the stream back');
startRgbdWebSocket(options);first=open();first.onclose({code:1006});
state.running=false;tick();
assert.equal(sockets.at(-1),first,'stopping during backoff cannot reconnect');
state.running=true;state.reconnectAttempt=0;
startRgbdWebSocket(options);const stalled=sockets.at(-1);tick();
assert.equal(stalled.readyState,3,'a stuck connecting socket is closed');
closeRgbdSocket();assert.equal(timers.size,0,'explicit reconnect/stop cancels the pending retry');
startRgbdWebSocket(options);first=open();first.onclose({code:1006});
context.document.hidden=true;tick();
assert.equal(sockets.at(-1),first,'hidden tabs cannot resurrect their stream');
console.log('RGB-D transport: transient recovery, bounded fallback, deadline, stop/hidden/supersession passed');
let polls=0,deletes=0,flushes=0,ready=false;
state.gl={TIMEOUT_EXPIRED:1,WAIT_FAILED:2,SYNC_GPU_COMMANDS_COMPLETE:3,
  clientWaitSync(_sync,flags,timeout){assert.equal(flags,0);assert.equal(timeout,0);polls++;return ready?4:1;},
  deleteSync(){deletes++;},fenceSync(){return {};},flush(){flushes++;}};
context.window.probe.markGpuFrame();
assert.equal(context.window.probe.gpuFrameReady(),true,'one queued frame retains CPU/GPU overlap');
context.window.probe.markGpuFrame();
assert.equal(context.window.probe.gpuFrameReady(),false,'a busy GPU must not receive another obsolete view');
assert.equal(deletes,0);ready=true;
assert.equal(context.window.probe.gpuFrameReady(),true,'completed GPU work releases the newest capture');
assert.equal(deletes,2);assert.equal(state.renderFences.length,0);assert.equal(flushes,2);assert.equal(polls,4);
console.log('GPU pacing: nonblocking fence polling, completion and cleanup passed');
