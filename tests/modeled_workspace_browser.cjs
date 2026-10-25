const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {mkdirSync}=require('node:fs');
const {resolve}=require('node:path');
const assert=require('node:assert/strict');
const {createTestServer}=require('./serve_web_rendering.cjs');
const server=createTestServer();
(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  let browser;
  try {
    browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
    const page=await browser.newPage({viewport:{width:1200,height:900},deviceScaleFactor:1});
    const errors=[];page.on('pageerror',error=>errors.push(error.message));
    const diagnostics=[];page.on('console',message=>{if(['warning','error'].includes(message.type()))diagnostics.push(message.text());});
    const url=`http://127.0.0.1:${server.address().port}`;
    await page.goto(url+'/?autorun=1&modeled=1',{waitUntil:'commit'});
    await page.waitForFunction(()=>window.testResult||window.testError,null,{timeout:90000});
    const result=await page.evaluate(()=>({result:window.testResult,error:window.testError}));
    assert.ok(!result.error,[result.error,...diagnostics].join('\n'));
    const output=resolve(__dirname,'../.teleop/modeled-workspace-tests');mkdirSync(output,{recursive:true});
    await page.screenshot({path:resolve(output,'gpu.png'),fullPage:true});
    // This isolated settings check has no webcam or capture service.
    await page.addInitScript(()=>localStorage.setItem('robotTeleopViewSettings',JSON.stringify({settings_version:15,head_tracking_enabled:false})));
    await page.goto(url+'/controller.html?rgbd_transport=websocket');
    await page.locator('#start-overlay').waitFor({state:'visible'});
    // Capture APIs intentionally return 404 in this fixture; dismiss their
    // handled startup message so the real settings controls can be exercised.
    await page.evaluate(()=>{document.getElementById('start-overlay').hidden=true;});
    await page.getByRole('button',{name:'Advanced Setup',exact:true}).click();
    const toggle=page.getByRole('checkbox',{name:'Modeled table and floor',exact:true});
    assert.equal(await toggle.isChecked(),false,'new browser starts in the original full scene');
    // Mock only the picked world point; run the actual controls/persistence.
    await page.evaluate(()=>{window.alignGodotModeledWorkspace=()=>({modeled_workspace_center:[.12,.34,-.56]});});
    await toggle.check();
    await page.getByRole('button',{name:'Place Below Arm Base',exact:true}).click();
    await page.getByText('Exact placement and lighting',{exact:true}).click();
    assert.equal(await page.getByRole('spinbutton',{name:'Table width (m)',exact:true}).inputValue(),'1.2');
    await page.getByRole('spinbutton',{name:'Table width (m)',exact:true}).fill('0.9');
    await page.getByRole('spinbutton',{name:'Table depth (m)',exact:true}).click();
    await page.waitForFunction(()=>JSON.parse(localStorage.getItem('robotTeleopViewSettings')||'{}').modeled_workspace_width===.9);
    const saved=await page.evaluate(()=>JSON.parse(localStorage.getItem('robotTeleopViewSettings')));
    assert.deepEqual(saved.modeled_workspace_center,[.12,.34,-.56]);
    assert.equal(saved.modeled_workspace_enabled,true);
    await page.locator('#modeled-workspace-options').screenshot({path:resolve(output,'desktop-controls.png')});
    await page.setViewportSize({width:390,height:844});
    await toggle.scrollIntoViewIfNeeded();
    const overflow=await page.locator('#modeled-workspace-options').evaluate(el=>el.scrollWidth>el.clientWidth+1);
    assert.equal(overflow,false,'alignment controls fit phone width');
    await page.screenshot({path:resolve(output,'mobile-controls.png')});
    await toggle.uncheck();
    assert.equal(await page.locator('#modeled-workspace-options').isVisible(),false);
    const off=await page.evaluate(()=>JSON.parse(localStorage.getItem('robotTeleopViewSettings')));
    assert.equal(off.modeled_workspace_enabled,false);
    assert.deepEqual(off.modeled_workspace_center,saved.modeled_workspace_center,'off switch retains placement for the next comparison');
    assert.equal(off.display_mode,saved.display_mode,'off switch preserves rendering mode');
    assert.deepEqual(errors,[]);
    console.log(JSON.stringify({...result.result,ui:'toggle, alignment, persistence and mobile checks passed',output}));
  } finally {await browser?.close();await new Promise(resolve=>server.close(resolve));}
})().catch(error=>{console.error(error);process.exitCode=1;});
