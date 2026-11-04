const assert=require('node:assert/strict');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');
const server=createTestServer();
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const browser=await chromium.launch({channel:'msedge',headless:true});
 try{
  const context=await browser.newContext();let site={settings_version:16,display_mode:'blendpcr',point_preview_detail:-1,stream_preset:'latency'};
  await context.route('**/api/v1/site-settings',async route=>{
   if(route.request().method()==='POST')site=route.request().postDataJSON();
   await route.fulfill({json:site});
  });
  await context.routeWebSocket('**/*',ws=>ws.close());
  const page=await context.newPage();const errors=[];page.on('pageerror',e=>errors.push(e.message));
  const url=`http://127.0.0.1:${server.address().port}/controller.html`;
  await page.goto(url);await page.waitForFunction(()=>document.getElementById('point-preview-enabled').value==='-1');
  assert.equal(await page.locator('#squares-fusion-enabled').isChecked(),false,'new mode is opt-in');
  await page.evaluate(()=>{const e=document.getElementById('squares-fusion-enabled');e.checked=true;e.dispatchEvent(new Event('change'));});
  await page.waitForTimeout(800);assert.notEqual(site.squares_fusion_enabled,true,'a local toggle does not silently overwrite host defaults');
  await page.evaluate(()=>document.getElementById('save-site-defaults').click());
  await page.waitForFunction(()=>document.getElementById('site-settings-status').textContent.startsWith('Saved.'));
  assert.equal(site.squares_fusion_enabled,true,'explicit Save publishes the toggle');
  await page.evaluate(()=>localStorage.clear());await page.reload();
  await page.waitForFunction(()=>document.getElementById('squares-fusion-enabled').checked);
  assert.equal(site.point_preview_detail,-1);assert.equal(site.stream_preset,'latency');assert.deepEqual(errors,[]);
  console.log('Squares fusion: opt-in, preview-only changes, explicit shared Save and fresh-browser restore passed.');
 }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
