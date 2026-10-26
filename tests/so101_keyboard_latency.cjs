// Real event handler, simulated transport only. No robot connection.
const assert = require('node:assert/strict');
const {readFileSync} = require('node:fs');
const {resolve} = require('node:path');
(async () => {
  global.window = {so101ArmController: {}, addEventListener() {}};
  global.HTMLElement = class {};
  const source = readFileSync(resolve(__dirname, '../robot_modules/so101/web/so101_web_serial.js'), 'utf8');
  const {So101ArmController} = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));
  const arm = Object.create(So101ArmController.prototype), sent = [];
  Object.assign(arm, {latest: {keyboardConnected: true}, keyboardKeys: new Set(),
    keyboardKeyOrder: new Map(), keyboardKeySequence: 0, seq: 0, emitStatus() {},
    send(packet) {sent.push(packet);return true;}});
  const event = code => ({code, preventDefault() {}, stopPropagation() {}, stopImmediatePropagation() {}});
  arm.handleKeyboardEvent(event('KeyW'), true);
  assert.equal(sent.length, 1, 'press sends before returning from event handler');
  assert.equal(sent[0].linear[1], 1);
  arm.handleKeyboardEvent({...event('KeyW'), repeat: true}, true);
  assert.equal(sent.length, 1, 'OS repeat must not flood commands');
  arm.handleKeyboardEvent(event('KeyW'), false);
  assert.equal(sent.length, 2, 'release immediately sends zero velocity');
  assert.deepEqual(sent[1].linear, [0, 0, 0]);
  arm.trialSuspended = true;
  arm.handleKeyboardEvent(event('KeyW'), true);
  assert.equal(sent.length, 2, 'paused trial cannot send input');
  arm.trialSuspended = false;
  const input = new HTMLElement();input.tagName = 'INPUT';input.type = 'text';
  arm.handleKeyboardEvent({...event('KeyW'), target: input}, true);
  assert.equal(sent.length, 2, 'typing must never command hardware');
  console.log('PASS immediate key transitions, repeat suppression, Pause and editing isolation');
})().catch(error => {console.error(error);process.exitCode = 1;});
