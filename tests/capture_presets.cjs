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
  console.log('capture preset transport: replacement, cancellation, failure and timeout passed');
})().catch(error=>{console.error(error);process.exitCode=1;});
