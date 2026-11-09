const assert = require("node:assert/strict");
const { readFileSync } = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

const bridgePath = path.join(__dirname, "../web/godot_webcam_tracker_bridge.js");
const source = readFileSync(bridgePath, "utf8").replace("import.meta.url", JSON.stringify(bridgePath));

function deferred() {
  let resolve;
  const promise = new Promise(done => { resolve = done; });
  return { promise, resolve };
}

function webcamFixture(getUserMedia, loadModel) {
  const video = { srcObject: null, play: async () => {}, pause() {} };
  const context = vm.createContext({
    window: { cancelAnimationFrame() {} },
    console: { warn() {} },
    navigator: { mediaDevices: { getUserMedia, enumerateDevices: async () => [] } },
    video,
    loadModel,
    loopStarts: 0,
  });
  vm.runInContext(source, context);
  vm.runInContext([
    "state.video = video;",
    "state.webglInfo = { webgl1: true };",
    "placeVideoElement = () => video;",
    "initLandmarker = loadModel;",
    "loop = () => { loopStarts++; };",
    "updatePreview = () => {};",
    "updateFaceOverlay = () => {};",
  ].join("\n"), context);
  return {
    video,
    start: () => context.window.startGodotWebcamTracker({ remoteEnabled: false, preview: false }),
    stop: () => context.window.stopGodotWebcamTracker(),
    loops: () => context.loopStarts,
  };
}

async function checkWebcamLifecycle() {
  let stops = 0;
  const stream = { getTracks: () => [{ stop() { stops++; } }] };
  const requested = deferred();
  const permission = deferred();
  const pending = webcamFixture(() => {
    requested.resolve();
    return permission.promise;
  }, async () => {});
  const start = pending.start();
  await requested.promise;
  pending.stop();
  permission.resolve(stream);
  assert.equal(await start, false);
  assert.equal(pending.video.srcObject, null);
  assert.equal(pending.loops(), 0);
  assert.equal(stops, 1);

  const failed = webcamFixture(async () => stream, async () => {
    throw new Error("model load failed");
  });
  await assert.rejects(failed.start(), /model load failed/);
  assert.equal(failed.video.srcObject, null);
  assert.equal(failed.loops(), 0);
  assert.equal(stops, 2);
}

checkWebcamLifecycle().catch(error => {
  console.error(error);
  process.exitCode = 1;
});
