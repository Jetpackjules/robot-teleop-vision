// Exercise production module discovery/mounting without hardware or camera access.
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const {readFileSync, mkdirSync} = require('node:fs');
const {resolve} = require('node:path');
const http = require('node:http');
const assert = require('node:assert/strict');

const root = resolve(__dirname, '..');
const sourceRoot = process.env.ROBOT_MODULE_TEST_SOURCE_ROOT || root;
const output = resolve(root, '.teleop/robot-module-browser');
mkdirSync(output, {recursive:true});
const css = readFileSync(resolve(sourceRoot, 'web/controller.html'), 'utf8').split('<style>')[1].split('</style>')[0];
const manifest = JSON.parse(readFileSync(resolve(sourceRoot, 'robot_modules/so101/robot.json'), 'utf8'));
let scenario = 'disabled';
const server = http.createServer((req,res) => {
  const path = new URL(req.url, 'http://localhost').pathname;
  res.setHeader('Cache-Control', 'no-store');
  if(path === '/api/v1/robot-module') {
    res.setHeader('Content-Type','application/json');
    if(scenario === 'discovery-error') {res.writeHead(503);res.end('{}');return;}
    const value = scenario === 'disabled' ? {id:'disabled',label:'Vision only',entrypoints:{}}
      : scenario === 'no-web' ? {id:'testbot',label:'Test robot',entrypoints:{}}
      : manifest;
    res.end(JSON.stringify(value));
  } else if(path === '/robot_module_host.js') {
    res.setHeader('Content-Type','text/javascript');
    res.end(readFileSync(resolve(sourceRoot,'web/robot_module_host.js')));
  } else if(path === manifest.entrypoints.web) {
    res.setHeader('Content-Type','text/javascript');
    if(scenario === 'import-error') {res.writeHead(404);res.end('');return;}
    let source = readFileSync(resolve(sourceRoot,'robot_modules/so101/web/module.js'),'utf8');
    if(scenario === 'mount-error') source = source.replace('await import("./so101_web_serial.js");',
      'throw new Error("<img src=x onerror=alert(1)> mount failed");');
    res.end(source);
  } else if(path.endsWith('.js')) {
    // The real mount runs, but serial and auxiliary-camera startup stay inert.
    res.setHeader('Content-Type','text/javascript');res.end('export {};');
  } else {
    res.setHeader('Content-Type','text/html');
    res.end(`<!doctype html><meta charset="utf-8"><style>${css}</style>
      <div id="robot-settings-root" hidden></div><div id="robot-status-root"></div>
      <div class="advanced-shell" style="margin:20px"><section class="advanced-section wide">
      <h3>Robot setup and calibration</h3><div id="robot-setup-root"></div><div id="robot-view-root"></div>
      </section></div><div id="robot-floating-root" hidden></div>
      <script type="module">import {robotModuleReady} from '/robot_module_host.js';
      window.testModule = await robotModuleReady;</script>`);
  }
});

(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  let browser;
  try {
    browser = await chromium.launch({headless:true,
      ...(process.env.TEST_BROWSER_CHANNEL ? {channel:process.env.TEST_BROWSER_CHANNEL} : {})});
    const page = await browser.newPage({viewport:{width:1050,height:850}});
    const errors=[];page.on('pageerror', error=>errors.push(error.message));
    const scenarios = process.env.ROBOT_MODULE_TEST_SCENARIOS?.split(',') || ['disabled','so101','no-web','discovery-error','import-error','mount-error'];
    for(const name of scenarios) {
      scenario=name;
      await page.goto(`http://127.0.0.1:${server.address().port}/?case=${name}`);
      await page.waitForFunction(()=>window.testModule);
      const controls = page.locator('#robot-view-root');
      assert((await controls.innerText()).trim(),`${name}: setup must never be blank`);
      if(name === 'so101') {
        assert.equal(await controls.getByRole('button',{name:'Calibrate Full Arm',exact:true}).count(),1);
        assert.equal(await controls.getByRole('checkbox',{name:'Robot overlay',exact:true}).count(),1);
        await page.screenshot({path:resolve(output,'so101.png'),fullPage:true});
      } else {
        assert.equal(await page.locator('#so101-calibrate').count(),0,`${name}: no stale calibration controls`);
        assert.equal(await page.locator('#so101-enable').count(),0,`${name}: no stale hardware controls`);
        if(name === 'disabled') {
          assert((await controls.innerText()).includes('Vision only mode is active'));
          await page.screenshot({path:resolve(output,'vision-only.png')});
        } else if(name === 'no-web') {
          assert((await controls.innerText()).includes('Test robot does not provide web controls'));
        } else {
          assert.equal(await page.evaluate(()=>window.testModule.statusSummary()),'Robot module unavailable');
          assert((await controls.innerText()).includes('Robot controls could not load'));
        }
        if(name === 'mount-error') {
          assert((await controls.innerText()).includes('<img src=x onerror=alert(1)>'));
          assert.equal(await controls.locator('img').count(),0,'error text is never interpreted as markup');
          assert.equal(await page.locator('#so101-manual-workspace').count(),0,'partial floating controls are cleared');
        }
      }
      console.log(`PASS ${name}`);
    }
    assert.deepEqual(errors,[]);
    console.log(scenarios.length+' module discovery/mount scenarios passed; screenshots: '+output);
  } finally {
    await browser?.close();
    await new Promise(resolve=>server.close(resolve));
  }
})().catch(error=>{console.error(error);process.exitCode=1;});
