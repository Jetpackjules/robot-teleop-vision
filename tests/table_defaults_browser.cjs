const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const assert=require('node:assert/strict');
const {createTestServer}=require('./serve_web_rendering.cjs');
const server=createTestServer();
(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  let browser;
  try {
    browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
    const context=await browser.newContext();
    const errors=[],writes=[];
    const current={modeled_workspace_enabled:true,modeled_workspace_center:[.12,-.34,-.56],
      modeled_workspace_width:.93,modeled_workspace_depth:.67,modeled_workspace_table_height:.81,
      modeled_workspace_heading:37,workspace_crop_enabled:true,workspace_crop_show_box:true,
      workspace_crop_min:[0,0,0],workspace_crop_max:[.1,.1,.1],workspace_surface_enabled:true,workspace_boundary_enabled:true};
    let site={settings_version:1793108252126,display_mode:'blendpcr',robot_overlay_scope:'claw',
      ...current,point_preview_detail:-1,stream_preset:'latency',robot_overlay_body_buffer_mm:10,
      workspace_crop_enabled:true,workspace_surface_enabled:true};
    await context.route('**/api/v1/site-settings',async route=>{
      if(route.request().method()==='POST') {site=route.request().postDataJSON();writes.push(site);}
      await route.fulfill({json:site});
    });
    const page=await context.newPage();page.on('pageerror',e=>errors.push(e.message));
    const url=`http://127.0.0.1:${server.address().port}/controller.html?rgbd_transport=websocket`;
    await page.goto(url);
    await page.evaluate(current=>localStorage.setItem('robotTeleopViewSettings',JSON.stringify({settings_version:16,head_tracking_enabled:false,...current,modeled_workspace_width:1.8,modeled_workspace_center:[9,9,9],point_preview_detail:20000})),current);
    await page.evaluate(()=>{localStorage.setItem('robotTeleopPointDetail','20000');localStorage.setItem('robotTeleopPointDetailHostDefault',JSON.stringify([1793108252126,-1]));});
    await page.reload();
    await page.waitForFunction(()=>document.querySelector('#modeled-workspace-enabled').checked);
    assert.equal(await page.locator('[id^="workspace-crop-"], [id^="workspace-surface-"], [id^="workspace-boundary-"]').count(),0);
    const read=()=>page.evaluate(()=>Object.fromEntries(['enabled','width','depth','table-height','heading','x','y','z'].map(key=>{
      const input=document.getElementById(`modeled-workspace-${key}`);return [key,key==='enabled'?input.checked:input.valueAsNumber];
    })));
    assert.equal(await page.locator('#point-preview-enabled').inputValue(),'-1','saved host detail overrides a stale browser choice');
    assert.deepEqual(await read(),{enabled:true,width:.93,depth:.67,'table-height':.81,heading:37,x:.12,y:-.34,z:-.56});
    await assertEventually(()=>site.modeled_workspace_width===.93);
    assert.equal(site.display_mode,'blendpcr');assert.equal(site.robot_overlay_scope,'claw');
    // Changes in the editor only become installation defaults after Done/Cancel.
    const baseline=writes.length;
    await page.evaluate(()=>{
      document.body.classList.add('workspace-editing');
      window.dispatchEvent(new CustomEvent('workspace-edit-change',{detail:{phase:'preview',settings:{modeled_workspace_center:[.2,-.4,-.7],modeled_workspace_width:1.07}}}));
    });
    await page.waitForTimeout(600);assert.equal(writes.length,baseline);
    await page.evaluate(()=>window.dispatchEvent(new CustomEvent('workspace-edit-finish',{detail:{}})));
    await assertEventually(()=>site.modeled_workspace_width===1.07);
    assert.equal(Object.keys(site).some(key=>/^(workspace_crop_|workspace_surface_|workspace_boundary_)/.test(key)),false,'obsolete site preferences are retired');

    // A fresh browser uses the shared last placement, including its enabled state.
    const fresh=await context.newPage();
    await page.evaluate(()=>localStorage.clear());
    await fresh.goto(url);await fresh.waitForFunction(()=>document.querySelector('#modeled-workspace-width').value==='1.07');
    assert.equal(await fresh.locator('#modeled-workspace-enabled').isChecked(),true);
    await fresh.evaluate(()=>{
      const toggle=document.querySelector('#modeled-workspace-enabled');toggle.checked=false;toggle.dispatchEvent(new Event('change'));
    });
    await assertEventually(()=>site.modeled_workspace_enabled===false);
    assert.deepEqual(site.modeled_workspace_center,[.2,-.4,-.7]);
    await fresh.reload();await fresh.waitForFunction(()=>document.querySelector('#modeled-workspace-width').value==='1.07');
    assert.equal(await fresh.locator('#modeled-workspace-enabled').isChecked(),false);
    assert.deepEqual(errors,[]);
    console.log('Table defaults: existing placement, legacy version, automatic save, editor finish, fresh browser, and off-state restore passed.');
  } finally {await browser?.close();await new Promise(resolve=>server.close(resolve));}
})().catch(error=>{console.error(error);process.exitCode=1;});
async function assertEventually(predicate){for(let i=0;i<60;i++){if(predicate())return;await new Promise(r=>setTimeout(r,100));}assert.ok(predicate(),'table defaults saved');}
