// Exercise the real session setup against a simulated leader and follower. No hardware.
const assert = require('node:assert/strict'), fs = require('node:fs'), path = require('node:path');
(async () => {
  global.window = {setTimeout: (fn, ms) => setTimeout(fn, Math.min(ms, 30)), clearTimeout,
    so101ArmController: {}, addEventListener() {}};
  global.WebSocket = {OPEN: 1};
  const source = fs.readFileSync(path.join(__dirname, '../robot_modules/so101/web/so101_web_serial.js'), 'utf8');
  const {So101ArmController} = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));
  const make = () => {
    const arm = Object.setPrototypeOf(new EventTarget(), So101ArmController.prototype);
    Object.assign(arm, {latest: {serverConnected: true, followerConnected: true, leaderConnected: false,
      keyboardConnected: false, armed: false, controlMode: 'leader', status: {control_source: 'leader'}},
      lastLeaderPositions: [1, 2, 3, 4, 5, 6], lastLeaderReadTimes: new Map(), leaderReadFault: false,
      sent: [], connections: 0, emitStatus() {this.dispatchEvent(new Event('status'));}});
    arm.fresh = () => {for (let id = 1; id <= 6; id++) arm.lastLeaderReadTimes.set(id, performance.now());};
    arm.connectLeader = async () => {arm.connections++; arm.latest.leaderConnected = true; arm.fresh(); arm.emitStatus();};
    arm.socket = {readyState: 1, bufferedAmount: 0, send(raw) {
      const message = JSON.parse(raw); arm.sent.push(message);
      if (message.type === 'arm_enable') queueMicrotask(() => {
        arm.latest.armed = true; arm.latest.status.control_source = message.source; arm.emitStatus();
      });
    }};
    return arm;
  };
  const arm = make(), setup = arm.prepareLeaderTrial({connect: true});
  assert.equal(arm.connections, 1, 'USB selection starts directly from the setup click');
  await setup;
  const chooser = make();
  chooser.connectKeyboard = async()=>{throw Error('Keyboard must never be silently selected');};
  await chooser.prepareTrialControl({connect:true,keyboardFallback:true});
  assert.equal(chooser.connections,1,'generic setup opens the leader chooser even for a legacy keyboard fallback caller');
  assert.deepEqual(arm.sent.filter(m=>m.type!=='arm_status_request'), [{type: 'arm_enable', source: 'leader'}]);
  await arm.prepareLeaderTrial(); assert.equal(arm.sent.filter(m=>m.type==='arm_enable').length, 1, 'ready leader is reused');
  arm.latest.armed = false; await arm.prepareLeaderTrial();
  assert.equal(arm.sent.filter(m=>m.type==='arm_enable').length, 2, 'Start trial re-enables a held follower after timeout');
  arm.latest.status.control_source = 'keyboard'; await arm.prepareLeaderTrial();
  assert.equal(arm.sent.filter(m=>m.type==='arm_enable').length, 3); assert.equal(arm.latest.status.control_source, 'leader');
  assert.equal(arm.sent.some(m => /record|restart|return/.test(m.type)), false);
  const disconnected = make(); await assert.rejects(disconnected.prepareLeaderTrial(), /Reconnect the physical leader/);
  assert.equal(disconnected.connections, 0); assert.equal(disconnected.sent.length, 0);
  const cancelled = make(); cancelled.connectLeader = async () => {throw new Error('USB chooser cancelled');};
  await assert.rejects(cancelled.prepareLeaderTrial({connect: true}), /cancelled/); assert.equal(cancelled.sent.length, 0);
  const stale = make(); stale.latest.leaderConnected = true; stale.fresh(); stale.lastLeaderReadTimes.delete(6);
  await assert.rejects(stale.prepareLeaderTrial(), /did not confirm/); assert.equal(stale.sent.filter(m=>m.type!=='arm_status_request').length, 0, 'all six encoders must be fresh before Enable');
  const paused = make(); paused.trialSuspended = true;
  await assert.rejects(paused.prepareLeaderTrial({connect: true}), /paused trial/); assert.equal(paused.connections, 0);
  console.log('PASS physical-leader setup, ready reuse, held-arm re-enable, source selection, USB cancellation, stale encoder and Pause gates');
})().catch(error => {console.error(error.message); process.exitCode = 1;});
