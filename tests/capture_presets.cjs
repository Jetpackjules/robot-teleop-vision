const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, '../web/capture_presets.js'), 'utf8').replace(/^export /gm, '');
const sockets = [], timers = new Map();
class Socket {
  constructor(url) {this.url=url;this.sent=[];this.closed=0;sockets.push(this);}
  send(text) {this.sent.push(JSON.parse(text));}
  close() {this.closed++;}
}
const context = vm.createContext({WebSocket:Socket,setTimeout:fn=>{const id=Symbol();timers.set(id,fn);return id;},clearTimeout:id=>timers.delete(id)});
vm.runInContext(source+'\nglobalThis.control=new CapturePresetControl("wss://example.test/head-tracking");',context);
(async()=>{
  const first=context.control.request('quality'), original=sockets.at(-1), lateOpen=original.onopen;
  const second=context.control.request('latency'), current=sockets.at(-1);
  assert.equal(await first,false);
  lateOpen();assert.equal(original.sent.length,0);assert.equal(original.closed,1);
  current.onopen();assert.equal(await second,true);
  assert.deepEqual(current.sent,[{type:'view_settings',stream_preset:'latency'}]);
  assert.equal(timers.size,0);assert.equal(context.control.pending,null);
  let failed=context.control.request('quality');sockets.at(-1).onerror();
  await assert.rejects(failed,/Could not change/);
  failed=context.control.request('quality');sockets.at(-1).onclose();
  await assert.rejects(failed,/Could not change/);
  failed=context.control.request('quality');[...timers.values()][0]();
  await assert.rejects(failed,/timed out/);
  failed=context.control.request('latency');context.control.cancel();
  assert.equal(await failed,false);assert.equal(timers.size,0);
  await assert.rejects(context.control.request('unknown'),/Unknown camera preset/);
  Socket.prototype.readyState=0;
  vm.runInContext('globalThis.workspace=new WorkspaceStreamControl("wss://example.test/head-tracking");',context);
  const settings={rgbd_client_id:'page-a',workspace_stream_crop_enabled:true,display_mode:'unified_mesh',
    modeled_workspace_enabled:true,modeled_workspace_center:[0,0,0],modeled_workspace_width:1.2,
    calibrate_robot_position:true,robot_visual_calibration_action:{operation:'save'},stream_preset:'latency'};
  assert.equal(context.workspace.update(settings),true,'view settings connect without a tracker or WebRTC');
  const connecting=sockets.at(-1);
  context.workspace.update({...settings,modeled_workspace_width:1.4});
  assert.equal(sockets.at(-1),connecting,'rapid edits share one connection');
  assert.equal(connecting.sent.length,0);
  connecting.readyState=1;connecting.onopen();
  assert.equal(connecting.sent.length,1);
  assert.equal(connecting.sent[0].modeled_workspace_width,1.4,'newest bounds replace pending edits');
  assert.equal(connecting.sent[0].calibrate_robot_position,undefined,'calibration actions are never queued');
  assert.equal(connecting.sent[0].robot_visual_calibration_action,undefined);
  assert.equal(connecting.sent[0].stream_preset,undefined,'workspace edits never restart cameras');
  assert.equal(connecting.sent[0].type,'view_settings');
  context.workspace.update({...settings,modeled_workspace_width:1.4});
  assert.equal(connecting.sent.length,1,'unchanged bounds do not resend');
  connecting.onclose();assert.equal(timers.size,1);
  const retry=[...timers.entries()][0];timers.delete(retry[0]);retry[1]();
  const reconnected=sockets.at(-1);reconnected.readyState=1;reconnected.onopen();
  assert.deepEqual(reconnected.sent,connecting.sent,'reconnect restores only viewing bounds');
  context.workspace.update({...settings,modeled_workspace_enabled:false});
  assert.equal(reconnected.sent.at(-1).modeled_workspace_enabled,false,'off restores uncropped data');
  assert.equal(context.workspace.update({...settings,rgbd_client_id:'../bad'}),false);
  const staleOpen=reconnected.onopen;
  context.workspace.close();staleOpen();
  assert.equal(timers.size,0);assert.equal(context.workspace.socket,null);
  assert.equal(context.workspace.update(settings),false,'closed pages cannot reconnect');
  vm.runInContext('globalThis.timeoutWorkspace=new WorkspaceStreamControl("wss://example.test/head-tracking");',context);
  context.timeoutWorkspace.update(settings);
  const quiet=sockets.at(-1),deadline=[...timers.entries()][0];
  timers.delete(deadline[0]);deadline[1]();
  assert.equal(quiet.closed,1,'a socket stuck connecting is closed');
  assert.equal(timers.size,1,'a bounded retry is scheduled after connection timeout');
  context.timeoutWorkspace.close();assert.equal(timers.size,0);
  console.log('capture/workspace transport: replacement, bounded reconnect, tracker-independent bounds and action isolation passed');
})().catch(error=>{console.error(error);process.exitCode=1;});
