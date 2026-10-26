// Exercise the real module methods with a simulated arm transport. No hardware.
const assert = require('node:assert/strict');
const {readFileSync} = require('node:fs');
const {resolve} = require('node:path');

(async () => {
  // Keep the module's page bootstrap inert; construct only the transport fixture.
  global.window = {setTimeout, clearTimeout, so101ArmController: {}, addEventListener() {}};
  global.WebSocket = {OPEN: 1};
  const source = readFileSync(resolve(__dirname, '../robot_modules/so101/web/so101_web_serial.js'), 'utf8');
  const {So101ArmController} = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));
  function simulatedArm(armed = true) {
    const arm = Object.setPrototypeOf(new EventTarget(), So101ArmController.prototype);
    Object.assign(arm, {
      latest: {serverConnected: true, followerConnected: true, keyboardConnected: true,
        armed, state: armed ? 'armed' : 'ready', controlMode: 'keyboard'},
      keyboardKeys: new Set(['KeyW']), keyboardKeyOrder: new Map([['KeyW', 1]]), seq: 0,
      idleReturnEnabled: true, idleReturnTimeoutSeconds: 120, sent: [], acknowledge: true,
      emitStatus() { this.dispatchEvent(new Event('status')); },
    });
    arm.socket = {readyState: 1, bufferedAmount: 0, send(json) {
      const message = JSON.parse(json);
      arm.sent.push(message);
      if (arm.acknowledge && ['arm_hold', 'arm_enable'].includes(message.type)) queueMicrotask(() => {
        arm.latest.armed = message.type === 'arm_enable';
        arm.latest.state = arm.latest.armed ? 'armed' : 'hold';
        arm.emitStatus();
      });
    }};
    return arm;
  }

  const arm = simulatedArm();
  await arm.pauseTrial();
  assert.equal(arm.latest.armed, false);
  assert.equal(arm.latest.state, 'hold');
  assert.equal(arm.trialSuspended, true);
  assert.equal(arm.keyboardKeys.size, 0, 'keys held before Pause must be released');
  assert.equal(arm.sent[0].type, 'arm_idle_return_settings');
  assert.equal(arm.sent[0].enabled, false, 'optional idle return cannot move a paused arm');
  const pauseCount = arm.sent.length;
  arm.enableArm();
  arm.send({type: 'arm_return_to_rest'});
  arm.sendKeyboardIntent();
  arm.handleKeyboardEvent({code: 'KeyW'}, true);
  assert.equal(arm.sent.length, pauseCount, 'commands and keyboard recovery remain blocked during Pause');
  assert.equal(arm.keyboardKeys.size, 0);
  await arm.pauseTrial();  // A retry must retain the original armed state.
  await arm.resumeTrial();
  assert.equal(arm.latest.armed, true);
  assert.equal(arm.trialSuspended, false);
  const velocity = arm.sent.find(m => m.type === 'arm_cartesian_velocity');
  assert.deepEqual(velocity.linear, [0, 0, 0], 'Resume starts with fresh keyboard input');
  assert.equal(arm.sent.filter(m => m.type === 'arm_enable').length, 1);
  assert.equal(arm.sent.at(-1).enabled, true, 'resume restores configured idle return');

  const unarmed = simulatedArm(false);
  await unarmed.pauseTrial();
  await unarmed.resumeTrial();
  assert.equal(unarmed.sent.some(m => m.type === 'arm_enable'), false, 'an unarmed robot stays unarmed');

  const finish = simulatedArm();
  await finish.pauseTrial();
  finish.finishTrialPause();
  assert.equal(finish.trialSuspended, false);
  assert.equal(finish.sent.some(m => m.type === 'arm_enable'), false, 'Finish never resumes the arm');

  const modeChange = simulatedArm();
  await modeChange.pauseTrial();
  modeChange.latest.controlMode = 'leader';
  await assert.rejects(modeChange.resumeTrial(), /Restore the arm control mode/);
  assert.equal(modeChange.trialSuspended, true);

  const staleLeader = simulatedArm();
  staleLeader.latest.keyboardConnected = false;
  staleLeader.latest.leaderConnected = true;
  staleLeader.latest.controlMode = 'leader';
  staleLeader.lastLeaderPositions = [1, 2, 3, 4, 5, 6];
  staleLeader.lastLeaderReadTimes = new Map();
  await staleLeader.pauseTrial();
  await assert.rejects(staleLeader.resumeTrial(), /fresh readings from all six/);
  assert.equal(staleLeader.trialSuspended, true);
  assert.equal(staleLeader.sent.some(m => m.type === 'arm_enable'), false);

  const lost = simulatedArm();
  lost.acknowledge = false;
  const pending = lost.pauseTrial();
  lost.latest.serverConnected = false;
  lost.emitStatus();
  await assert.rejects(pending, /connection was lost/);
  assert.equal(lost.trialSuspended, true);

  // Exercise the real acknowledgement timeout without a three-second test delay.
  window.setTimeout = callback => setTimeout(callback, 10);
  const timeout = simulatedArm();
  timeout.acknowledge = false;
  await assert.rejects(timeout.pauseTrial(), /did not confirm/);
  assert.equal(timeout.trialSuspended, true);
  const backpressure = simulatedArm();
  backpressure.socket.bufferedAmount = 65536;
  await assert.rejects(backpressure.pauseTrial(), /Could not send Hold/);
  assert.equal(backpressure.trialSuspended, true);
  console.log('SO-101 Pause/Resume: held input gate, armed/unarmed resume, finish, mode/freshness and transport failures passed (synthetic).');
})().catch(error => {console.error(error.name + ': ' + error.message); process.exitCode = 1;});
