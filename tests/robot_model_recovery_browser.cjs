const assert = require('node:assert/strict');
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const {createTestServer} = require('./serve_web_rendering.cjs');
const server = createTestServer();
(async () => {
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  const browser = await chromium.launch({headless: true, ...(process.env.TEST_BROWSER_CHANNEL ? {channel: process.env.TEST_BROWSER_CHANNEL} : {})});
  try {
    const page = await browser.newPage();
    let requests = 0, release;
    const resumed = new Promise(resolve => {release = resolve;});
    await page.route('**/lower_arm_link.glb', async route => {
      if (++requests === 1) await route.fulfill({status: 503, body: 'transient model download failure'});
      else {await resumed; await route.continue();}
    });
    await page.goto(`http://127.0.0.1:${server.address().port}/`);
    await page.evaluate(async () => {
      await import('/hybrid_rgbd_renderer.js');
      const fixture = await import('/web_rendering_fixture.js');
      const {state, gl} = fixture.setup();
      state.robotRenderer = await (await fetch('/so101-renderer.json')).json();
      for (const [i, name] of Object.keys(state.robotRenderer.modelUrls).entries())
        state.robotTransforms[name] = {basis_x:[1,0,0], basis_y:[0,1,0], basis_z:[0,0,1], origin:[0,i*.03,-1]};
      Object.assign(state.settings, {robot_overlay_enabled:true, robot_overlay_scope:'full'});
      window.savedTransforms = JSON.stringify(state.robotTransforms);
      window.modelLoad = window.probe.loadRobotMeshes(gl, state.robotLoadToken);
    });
    await page.waitForFunction(() => window.probe.state.stats.robotModelsLoaded === 6 && window.probe.state.stats.robotLoadError.includes('lower_arm_link'));
    const missing = await page.evaluate(() => ({
      loaded: window.probe.state.stats.robotModelsLoaded,
      warning: window.getGodotHybridRendererStatus().robotModelWarning,
      mask: window.probe.robotMaskCapsules(),
      unchanged: JSON.stringify(window.probe.state.robotTransforms) === window.savedTransforms,
    }));
    assert.equal(missing.loaded, 6); assert.match(missing.warning, /6\/7.*lower_arm_link/);
    assert.equal(missing.mask, null, 'missing CAD link must not erase the real camera arm');
    assert.equal(missing.unchanged, true);
    release();
    const recovered = await page.evaluate(async () => {
      await window.modelLoad;
      const {state} = window.probe;
      return {loaded:state.stats.robotModelsLoaded, error:state.stats.robotLoadError,
        warning:window.getGodotHybridRendererStatus().robotModelWarning,
        mask:Boolean(window.probe.robotMaskCapsules()),
        unchanged:JSON.stringify(state.robotTransforms) === window.savedTransforms,
        glError:state.gl.getError()};
    });
    assert.equal(requests, 2); assert.equal(recovered.loaded, 7);
    assert.equal(recovered.error, ''); assert.equal(recovered.warning, '');
    assert.equal(recovered.mask, true); assert.equal(recovered.unchanged, true); assert.equal(recovered.glError, 0);
    console.log(JSON.stringify({missing, recovered, requests}));
  } finally {await browser.close(); server.close();}
})().catch(error => {console.error(error); server.close(); process.exitCode = 1;});
