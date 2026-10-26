// Isolated browser and synthetic image stream. Never connects to the robot.
const assert = require('node:assert/strict');
const {readFileSync, mkdirSync, writeFileSync} = require('node:fs');
const {resolve} = require('node:path');
const http = require('node:http');
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = resolve(__dirname, '..');
const fixturePath = process.env.PREVIEW_FIXTURE || resolve(root, '.teleop/camera-preview-tests/fixture.json');
const server = http.createServer((req, res) => {
  res.setHeader('Cache-Control', 'no-store');
  if (req.url === '/camera_rgb_viewer.js') {
    res.setHeader('Content-Type', 'text/javascript');res.end(readFileSync(resolve(root, 'web/camera_rgb_viewer.js')));
  } else if (req.url === '/fixture.json') {
    res.setHeader('Content-Type', 'application/json');res.end(readFileSync(fixturePath));
  } else {res.setHeader('Content-Type', 'text/html');res.end('<!doctype html><canvas id="view"></canvas>');}
});
(async () => {
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  const browser = await chromium.launch({headless: true, ...(process.env.TEST_BROWSER_CHANNEL ? {channel: process.env.TEST_BROWSER_CHANNEL} : {})});
  try {
    const page = await browser.newPage(), errors = [];
    page.on('pageerror', error => errors.push(error.message));
    await page.goto(`http://127.0.0.1:${server.address().port}/`);
    const result = await page.evaluate(async () => {
      const fixture = await (await fetch('/fixture.json')).json();
      const checks = [];
      const check = (value, text) => {if (!value) throw new Error(text);checks.push(text);};
      const sockets = [];
      class Socket {
        static OPEN = 1;
        constructor(url) {this.url = url;this.readyState = 1;this.sent = [];sockets.push(this);}
        send(text) {this.sent.push(JSON.parse(text));}
        close() {this.readyState = 3;}
      }
      window.WebSocket = Socket;
      const {CameraRgbViewer} = await import('/camera_rgb_viewer.js');
      const canvas = document.getElementById('view');
      const viewer = new CameraRgbViewer(canvas, {camera: 'fixture'});
      const waitFor = async predicate => {for (let n = 0; n < 300; n++) {if (predicate()) return;await new Promise(r => setTimeout(r, 10));}throw new Error('Timed out waiting for decoded camera image');};
      function deliver(socket, packet, camera = 'fixture') {
        const metadata = {...packet.metadata, 'x-camera-id': camera};
        const head = new TextEncoder().encode(JSON.stringify(metadata));
        const bytes = Uint8Array.from(atob(packet.data), c => c.charCodeAt(0));
        const buffer = new Uint8Array(4 + head.length + bytes.length);
        new DataView(buffer.buffer).setUint32(0, head.length, true);
        buffer.set(head, 4);buffer.set(bytes, head.length + 4);
        socket.onmessage?.({data: buffer.buffer});
      }
      viewer.start();
      check(sockets[0].url.includes('/camera-preview-stream?'), 'WebCodecs selects compressed preview');
      for (const packet of fixture.packets) {
        const decoded = viewer.stats.decoded;deliver(sockets[0], packet);
        await waitFor(() => viewer.stats.decoded > decoded);
      }
      check(canvas.width === fixture.width && canvas.height === fixture.height, 'native image dimensions preserved');
      check(sockets[0].sent.filter(m => m.type === 'preview-ack').length === fixture.packets.length, 'decoded images release server flow-control credits');
      const context = canvas.getContext('2d'), pixels = context.getImageData(0, 0, canvas.width, canvas.height).data;
      for (const sample of fixture.colors) {
        const index = (sample.y * canvas.width + sample.x) * 4;
        check(sample.rgb.every((v, i) => Math.abs(v - pixels[index + i]) < 10), `camera color range and channels preserved: expected ${sample.rgb}, painted ${[...pixels.slice(index,index+3)]}`);
      }
      const painted = viewer.stats.decoded;
      viewer.setCamera('other');
      check(sockets[0].readyState === 3 && viewer.stats.decoded === painted, 'camera handoff closes old stream without clearing its painted image');
      deliver(sockets.at(-1), fixture.packets[0], 'other');
      await waitFor(() => viewer.stats.decoded > painted);
      check(viewer.cameraId === 'other', 'next camera replaces preserved image');
      viewer.stop();
      check(sockets.at(-1).readyState === 3 && viewer.decoder === null, 'leaving RGB mode closes network and decoder');
      const fallback = new CameraRgbViewer(canvas, {camera: 'fixture'});
      const RealDecoder = window.VideoDecoder;
      window.VideoDecoder = class {constructor() {throw new Error('decoder unavailable');}};
      fallback.start();deliver(sockets.at(-1), fixture.packets[0]);
      await waitFor(() => sockets.at(-1).url.includes('/camera-rgb-stream?'));
      check(fallback.previewFailed && sockets.at(-1).url.includes('/camera-rgb-stream?'), 'unsupported hardware returns to original JPEG stream');
      fallback.stop();window.VideoDecoder = RealDecoder;
      return {checks, success: true};
    });
    assert.deepEqual(errors, []);assert.equal(result.success, true);
    const output = resolve(root, '.teleop/camera-preview-tests');mkdirSync(output, {recursive: true});
    writeFileSync(resolve(output, 'browser-result.json'), JSON.stringify(result, null, 2));
    console.log(JSON.stringify({success: true, checks: result.checks.length}));
  } finally {await browser.close();await new Promise(resolve => server.close(resolve));}
})().catch(error => {console.error(error);process.exitCode = 1;server.close();});
