const assert = require('node:assert/strict');
const {readFileSync} = require('node:fs');
const source = readFileSync(require('node:path').join(__dirname, '../web/robot_model_loader.js'), 'utf8');
(async () => {
  const {fetchRobotModel} = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));
  const original = global.fetch;
  const options = {retryDelayMs: 1, timeoutMs: 20};
  try {
    let calls = 0, retries = 0;
    global.fetch = async () => ++calls === 1
      ? {ok: false, status: 503}
      : {ok: true, arrayBuffer: async () => new Uint8Array([7]).buffer};
    const result = await fetchRobotModel('/forearm.glb', {...options, onRetry: () => retries++, decode: b => new Uint8Array(b)[0]});
    assert.equal(result, 7); assert.equal(calls, 2); assert.equal(retries, 1);

    calls = 0;
    global.fetch = async () => {calls++; return {ok: true, arrayBuffer: async () => new ArrayBuffer(0)};};
    await assert.rejects(fetchRobotModel('/corrupt.glb', {...options, decode: () => {throw Error('invalid GLB');}}), /invalid GLB/);
    assert.equal(calls, 3, 'invalid model bodies have bounded retries');

    calls = 0;
    global.fetch = async (_url, {signal}) => {
      calls++;
      return {ok: true, arrayBuffer: () => new Promise((_resolve, reject) => signal.addEventListener('abort', () => reject(Error('aborted')), {once: true}))};
    };
    await assert.rejects(fetchRobotModel('/body-stalled.glb', options), /timed out/);
    assert.equal(calls, 3, 'the response body shares the download deadline');

    const controller = new AbortController(); calls = 0;
    global.fetch = (_url, {signal}) => {calls++; return new Promise((_resolve, reject) => signal.addEventListener('abort', () => reject(Error('aborted')), {once: true}));};
    const stopped = fetchRobotModel('/cancel.glb', {...options, signal: controller.signal});
    controller.abort(); assert.equal(await stopped, null); assert.equal(calls, 1);
    assert.equal(await fetchRobotModel('/already-stopped.glb', {signal: controller.signal}), null);
    assert.equal(calls, 1, 'stopped renderers cannot start another asset download');

    const backoff = new AbortController(); calls = 0;
    global.fetch = async () => {calls++; throw Error('offline');};
    assert.equal(await fetchRobotModel('/offline.glb', {signal: backoff.signal, onRetry: () => backoff.abort()}), null);
    assert.equal(calls, 1, 'stop during backoff prevents retries');
    console.log('PASS model retry, corrupt content, body deadline, cancellation and bounded recovery');
  } finally {global.fetch = original;}
})().catch(error => {console.error(error); process.exitCode = 1;});
