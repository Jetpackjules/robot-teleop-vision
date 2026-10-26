// Exercise the real controller, recorder, RGB transport and CAD with fake sensors.
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const {spawn} = require('node:child_process');
const {resolve} = require('node:path');
const {mkdirSync, readFileSync, writeFileSync} = require('node:fs');
const assert = require('node:assert/strict');
const root = resolve(__dirname, '..'), output = resolve(root, '.teleop/tester-ui-20261025');
mkdirSync(output, {recursive: true});
const episodeRoot = resolve(output, 'episodes-' + Date.now());
const server = spawn(process.env.TEST_PYTHON || resolve(root, '.venv/Scripts/python.exe'),
  [resolve(__dirname, 'serve_recording_fixture.py'), '--port', '0', '--output', episodeRoot],
  {cwd: root, stdio: ['ignore', 'pipe', 'pipe'], windowsHide: true});

(async () => {
  let browser;
  const diagnostics = [], errors = [];
  server.stderr.on('data', chunk => diagnostics.push(chunk.toString()));
  try {
    const port = await new Promise((accept, reject) => {
      let data = '';
      const timer = setTimeout(() => reject(new Error('Synthetic host did not start: ' + diagnostics.join('').slice(-1600))), 20000);
      server.stdout.on('data', chunk => {
        data += chunk.toString();
        for (const line of data.split('\n')) try {
          const result = JSON.parse(line);
          if (result.port) { clearTimeout(timer); accept(result.port); return; }
        } catch {}
      });
      server.on('exit', () => reject(new Error('Synthetic host exited: ' + diagnostics.join('').slice(-1600))));
    });
    browser = await chromium.launch({headless: true, ...(process.env.TEST_BROWSER_CHANNEL ? {channel: process.env.TEST_BROWSER_CHANNEL} : {})});
    const page = await browser.newPage({viewport: {width: 1366, height: 768}});
    // A developer's Live toolbar must not cover the app controls under test.
    await page.route('http://localhost:*/live.js*', route => route.abort());
    const capture = path => process.env.TEST_UI_SCREENSHOTS === '0' ? Promise.resolve() : page.screenshot({path});
    page.setDefaultTimeout(15000);
    page.setDefaultNavigationTimeout(20000);
    page.on('pageerror', error => errors.push(error.message));
    console.log('Fixture ready on loopback port', port);
    await page.goto(`http://127.0.0.1:${port}/controller.html?rgbd_transport=websocket`, {waitUntil: 'domcontentloaded'});
    await page.waitForFunction(() => document.querySelector('#so101-hold')
      && document.getElementById('episode-recording').dataset.recordingState === 'idle');
    console.log('Controller initialized');
    assert.equal(await page.locator('#advanced-setup').isVisible(), false, 'Settings begins closed');
    assert.equal(await page.locator('#connection-details').getAttribute('open'), null);
    assert.equal(await page.locator('#display-mode').inputValue(), 'rgb_camera');
    assert.equal(await page.locator('#head-tracking-enabled').isChecked(), false);
    assert.equal(await page.locator('#trial-progress').getAttribute('max'), '600', 'default time target matches RoboCade');
    assert.equal(await page.locator('#so101-hold').isVisible(), false, 'Hold stays off the participant view');
    assert.equal(await page.locator('#settings-controls #so101-hold').count(), 1);

    await page.locator('#trial-action').click();
    const taskInput = page.locator('#trial-task-input');
    assert.equal(await page.locator('#advanced-setup').isVisible(), false, 'Set task edits inline rather than opening Settings');
    assert.equal(await taskInput.evaluate(el => el === document.activeElement), true);
    await page.locator('#trial-action').click();
    assert.equal(await page.locator('#trial-task-editor').isVisible(), true, 'empty task cannot be saved');
    await taskInput.fill('Unsaved task');
    await taskInput.press('Escape');
    assert.equal(await page.locator('#trial-task-editor').isVisible(), false);
    assert.equal(await page.locator('[data-recording-task]').inputValue(), '', 'Escape leaves the saved task unchanged');
    await page.locator('#trial-action').click();
    await taskInput.fill('Another draft');
    await page.locator('#trial-task-cancel').click();
    assert.equal(await page.locator('[data-recording-task]').inputValue(), '', 'Cancel leaves the saved task unchanged');
    await page.locator('#trial-action').click();
    await taskInput.fill('Place the block in the cup');
    await page.locator('#trial-action').click();
    assert.equal(await page.locator('#trial-task-editor').isVisible(), false);
    assert.equal(await page.locator('#advanced-setup').isVisible(), false);
    assert.equal(await page.locator('#trial-task').textContent(), 'Place the block in the cup');
    assert.equal(await page.locator('#episode-recording').getAttribute('data-recording-state'), 'idle', 'saving a task never starts recording');
    await page.locator('#open-advanced').click();
    assert.equal(await page.locator('[data-recording-task]').inputValue(), 'Place the block in the cup', 'Settings shares the same saved task');
    await page.getByRole('spinbutton', {name: 'Time target (minutes)', exact: true}).fill('0.5');
    await page.locator('#tab-session').focus();
    assert.equal(await page.locator('#trial-progress').getAttribute('max'), '30');
    await page.keyboard.press('ArrowRight');
    assert.equal(await page.locator('#tab-controls').getAttribute('aria-selected'), 'true');
    for (const id of ['so101-connect', 'so101-enable', 'so101-hold', 'so101-keyboard', 'so101-return-rest'])
      assert.equal(await page.locator('#' + id).isVisible(), true, id + ' remains available');
    await page.locator('#tab-view').click();
    await page.getByRole('combobox', {name: 'View', exact: true}).selectOption('blendpcr');
    await page.locator('#tab-session').click();
    await page.locator('#restore-trial-view').click();
    await page.keyboard.press('Escape');
    assert.equal(await page.locator('#open-advanced').evaluate(el => el === document.activeElement), true);
    assert.equal(await page.locator('#open-advanced').getAttribute('aria-expanded'), 'false');

    await page.waitForFunction(() => document.querySelector('#rgb-camera-select').options.length === 2);
    await page.locator('#next-camera').click();
    assert.equal(await page.locator('#rgb-camera-select').inputValue(), 'camera-right');
    await page.locator('body').click({position: {x: 700, y: 500}});
    await page.keyboard.press('ArrowRight');
    assert.equal(await page.locator('#rgb-camera-select').inputValue(), 'camera-test');
    await page.waitForFunction(() => !document.getElementById('trial-action').disabled);
    assert.equal(await page.locator('#trial-message').isVisible(), false, 'ready HUD has no instruction line');
    assert.equal(await page.locator('#trial-recording-state').isVisible(), false, 'ready is not repeated below the timer');

    const episodes = [];
    for (const [index, condition] of ['fixed_rgb', 'mouse_3d', 'head_3d'].entries()) {
      assert.equal(await page.locator('#trial-position').textContent(), `${index + 1}/3`);
      assert.equal(await page.locator('#trial-progress').getAttribute('value'), '0', 'each trial resets its own time bar');
      await page.locator('#trial-action').click();
      await page.waitForFunction(() => document.getElementById('episode-recording').dataset.recordingState === 'recording');
      assert.equal(await page.locator('#trial-action').textContent(), 'Finish trial');
      assert.equal(await page.locator('#trial-message').isVisible(), false, 'normal recording has no explanatory subtext');
      assert.equal(await page.locator('#trial-recording-state').textContent(), 'Recording');
      await page.locator('#trial-outcome').selectOption(index === 1 ? 'failure' : 'success');
      await page.waitForFunction(() => document.getElementById('trial-timer').textContent !== '00:00');
      assert.ok(Number(await page.locator('#trial-progress').getAttribute('value')) > 0, 'time bar advances during the trial');
      assert.equal(await page.locator('#trial-duration').isDisabled(), true);
      assert.equal(await page.locator('#display-mode').isDisabled(), true, 'condition cannot change during a trial');
      if (index === 1) {
        await page.waitForFunction(() => window.getGodotHybridRendererStatus?.().robotModelsLoaded === 7);
        const recordingURL = '**/api/v1/recording?*';
        await page.route(recordingURL, async route => {
          const response = await route.fetch();
          const status = await response.json();
          await route.fulfill({response, json: {...status, ...(status.state === 'recording' ? {elapsed_seconds: 31} : {})}});
        });
        await page.waitForFunction(() => document.getElementById('trial-hud').dataset.timeReached === 'true');
        assert.equal(await page.locator('#trial-progress').getAttribute('value'), '30', 'bar caps at its target');
        assert.equal(await page.locator('#trial-timer').textContent(), '00:31', 'elapsed timer keeps counting past the target');
        assert.equal(await page.locator('#trial-action').textContent(), 'Finish trial', 'time target never stops the recording');
        assert.equal(await page.locator('#trial-position').textContent(), '2/3', 'time target never advances conditions');
        await page.unroute(recordingURL);
        await capture(resolve(output, 'trial-laptop.png'));
      }
      if (index === 2) {
        assert.equal(await page.locator('#reset-head-tracking').isVisible(), true);
        const count = await page.evaluate(() => window.trialFixture.resets);
        await page.locator('#reset-head-tracking').click();
        assert.equal(await page.evaluate(() => window.trialFixture.resets), count + 1);
      }
      await page.locator('#trial-action').click();
      await page.waitForFunction(() => document.getElementById('episode-recording').dataset.recordingState === 'saving');
      assert.equal(await page.locator('#trial-action').isDisabled(), true, 'cannot advance while host saves');
      assert.equal(await page.locator('#trial-message').isVisible(), true, 'host save warning remains visible');
      assert.equal(await page.locator('#trial-position').textContent(), `${index + 1}/3`);
      await page.waitForFunction(() => ['Next trial', 'End session'].includes(document.getElementById('trial-action').textContent));
      assert.equal(await page.locator('#trial-message').isVisible(), false, 'saved HUD has no redundant explanation');
      assert.equal(await page.locator('#trial-position').textContent(), `${index + 1}/3`, 'saving never auto-advances');
      const status = await page.evaluate(async () => await (await fetch('/api/v1/recording?client=recording-fixture')).json());
      const manifest = JSON.parse(readFileSync(resolve(episodeRoot, status.last_saved.episode, 'episode.json'), 'utf8'));
      assert.equal(manifest.status, 'complete'); assert.equal(manifest.synthetic, true);
      assert.equal(manifest.trial.condition, condition); assert.equal(manifest.trial.index, index + 1);
      assert.equal(manifest.outcome, index === 1 ? 'failure' : 'success');
      assert.equal(manifest.trial.display_mode, index ? 'blendpcr' : 'rgb_camera');
      assert.equal(manifest.cameras.length, 2, 'host records all cameras, independent of selected view');
      episodes.push(manifest.episode_id);
      await page.locator('#trial-action').click();
      if (index < 2) {
        await page.waitForFunction(() => document.getElementById('trial-action').textContent === 'Start trial');
        assert.equal(await page.locator('#display-mode').inputValue(), 'blendpcr');
        assert.equal(await page.locator('#head-tracking-enabled').isChecked(), index === 1);
      }
    }
    assert.equal(await page.locator('#trial-action').textContent(), 'Complete');
    assert.equal(await page.locator('#trial-position').textContent(), '3/3');
    assert.equal(await page.locator('#trial-progress').getAttribute('max'), '30');

    await page.locator('#open-advanced').click();
    await page.locator('#guided-trials').uncheck();
    await page.locator('#episode-recording summary').click();
    const start = page.locator('[data-recording-start]');
    await start.click();
    await page.waitForFunction(() => document.getElementById('episode-recording').dataset.recordingState === 'recording');
    assert.equal(await page.locator('[data-recording-outcome]').inputValue(), 'unrated');
    await page.locator('[data-recording-discard]').click();
    await page.waitForFunction(() => document.querySelector('[data-recording-status]').textContent.includes('discarded'));
    assert.equal(await page.locator('#trial-position').textContent(), 'Free session');
    await page.locator('#new-trial-session').click();
    await page.locator('#guided-trials').check();
    await page.locator('#trial-order').selectOption('mouse_3d,fixed_rgb,head_3d');
    await page.keyboard.press('Escape');
    await page.waitForFunction(() => document.getElementById('trial-condition').textContent === 'Mouse 3D');

    const layouts = [];
    for (const [width, height] of [[1366, 768], [1920, 1080]]) {
      await page.setViewportSize({width, height});
      const geometry = await page.evaluate(() => {
        const rect = id => {const r = document.getElementById(id).getBoundingClientRect(); return {left: r.left, right: r.right, top: r.top, bottom: r.bottom};};
        return {hud: rect('trial-hud'), settings: rect('open-advanced'), connection: rect('connection-details'), overflow: document.documentElement.scrollWidth > innerWidth};
      });
      assert.equal(geometry.overflow, false);
      assert.ok(geometry.hud.left > geometry.settings.right && geometry.hud.right < geometry.connection.left);
      await capture(resolve(output, `scene-${width}.png`));
      await page.locator('#open-advanced').click();
      const panel = await page.locator('#advanced-setup').boundingBox();
      assert.ok(panel.x + panel.width < geometry.hud.left || panel.y > geometry.hud.bottom, 'Settings does not cover the trial bar');
      for (const tab of ['session', 'controls', 'view', 'diagnostics']) {
        await page.locator('#tab-' + tab).click();
        assert.equal(await page.locator('.advanced-body').evaluate(el => el.scrollWidth <= el.clientWidth + 1), true, tab + ' fits panel');
      }
      await page.locator('#tab-session').click();
      await capture(resolve(output, `settings-${width}.png`));
      await page.keyboard.press('Escape');
      layouts.push({width, height, ...geometry});
    }
    // Position can change while the HUD's width stays constant. Starting wide
    // and then shrinking must move Settings below the HUD too.
    await page.reload({waitUntil: 'domcontentloaded'});
    await page.waitForFunction(() => !document.getElementById('trial-action').disabled);
    assert.equal(await page.locator('#trial-duration').inputValue(), '0.5', 'time target survives reload');
    await page.locator('#open-advanced').click();
    assert.equal((await page.locator('#advanced-setup').boundingBox()).y, 70);
    await page.setViewportSize({width: 1366, height: 768});
    await page.waitForFunction(() => document.getElementById('advanced-setup').getBoundingClientRect().top
      > document.getElementById('trial-hud').getBoundingClientRect().bottom);
    await page.keyboard.press('Escape');
    assert.deepEqual(errors, []);
    const result = {synthetic: true, conditions: 3, episodes, hostSaveGate: true, manualNext: true,
      outcomes: true, cameraArrows: true, tabsAndFocus: true, holdInSettings: true, freeConfiguration: true,
      elapsedTimeProgress: true, timeTargetNoAutoStop: true,
      actualCadModels: 7, layouts, episodeRoot};
    writeFileSync(resolve(output, 'browser-results.json'), JSON.stringify(result, null, 2));
    console.log(JSON.stringify(result));
  } catch (error) {
    console.error('Fixture diagnostics:', diagnostics.join('').slice(-2400)); throw error;
  } finally { await browser?.close(); server.kill(); }
})().catch(error => { console.error(error); server.kill(); process.exitCode = 1; });
