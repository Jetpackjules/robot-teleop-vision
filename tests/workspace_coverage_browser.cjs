const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {spawn}=require('node:child_process'),{resolve}=require('node:path');
const {mkdirSync,readFileSync,writeFileSync}=require('node:fs'),assert=require('node:assert/strict');
const root=resolve(__dirname,'..'),output=resolve(root,'.teleop/table-coverage-20261026');mkdirSync(output,{recursive:true});
const fixture=spawn(process.env.TEST_PYTHON||resolve(root,'.venv/Scripts/python.exe'),
  [resolve(__dirname,'serve_recording_fixture.py'),'--port','0','--output',resolve(output,'ui-fixture')],
  {cwd:root,stdio:['ignore','pipe','pipe'],windowsHide:true});
(async()=>{let browser;const logs=[],errors=[];fixture.stderr.on('data',d=>logs.push(d.toString()));
try{
  const port=await new Promise((accept,reject)=>{let text='';const timer=setTimeout(()=>reject(Error(logs.join('').slice(-2000))),20000);
    fixture.stdout.on('data',d=>{text+=d;for(const line of text.split('\n'))try{const value=JSON.parse(line);if(value.port){clearTimeout(timer);accept(value.port);}}catch{}});
    fixture.on('exit',()=>reject(Error('Fixture exited: '+logs.join('').slice(-1000))));});
  browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
  const page=await browser.newPage({viewport:{width:1366,height:900}});page.on('pageerror',e=>errors.push(e.message));
  await page.route('http://localhost:*/live.js*',r=>r.abort());
  await page.route('**/hybrid_rgbd_renderer.js*',async route=>{const response=await route.fetch();await route.fulfill({response,body:(await response.text())+'\nwindow.tableCoverageProbe={state,render};'});});
  await page.goto('http://127.0.0.1:'+port+'/controller.html?rgbd_transport=websocket',{waitUntil:'domcontentloaded'});
  await page.waitForFunction(()=>document.querySelector('#so101-hold')&&document.querySelector('#episode-recording').dataset.recordingState==='idle');
  await page.locator('#open-advanced').click();await page.locator('#tab-view').click();
  await page.locator('#display-mode').selectOption('blendpcr');await page.locator('#modeled-workspace-enabled').check();
  await page.waitForFunction(()=>window.tableCoverageProbe?.state.cameras.length>=2);
  const before=await page.evaluate(()=>{
    const settings={modeled_workspace_center:[0,-.5,-1],modeled_workspace_width:4,modeled_workspace_depth:4,modeled_workspace_height:3,modeled_workspace_clearance:0};
    window.setGodotHybridViewSettings(settings);return settings;
  });
  await page.locator('#modeled-workspace-edit').click();
  const coverage=page.locator('#workspace-edit-coverage');
  await page.waitForFunction(()=>document.querySelector('#workspace-edit-coverage').dataset.state==='ready');
  const initial=await coverage.evaluate(el=>({text:el.textContent,percent:+el.dataset.percent,total:+el.dataset.total,kept:+el.dataset.kept}));
  assert.ok(initial.total>0&&initial.kept>0,JSON.stringify(initial));
  assert.equal(await coverage.isVisible(),true);
  const depthBytes=await page.evaluate(()=>window.tableCoverageProbe.state.cameras.map(c=>c.depth.byteLength));
  assert.ok(depthBytes.every(v=>v>0),'worker counting does not detach rendering arrays');
  await page.evaluate(()=>window.setGodotHybridViewSettings({modeled_workspace_center:[0,10,-1]}));
  await page.waitForFunction(()=>document.querySelector('#workspace-edit-coverage').dataset.state==='ready'&&+document.querySelector('#workspace-edit-coverage').dataset.percent===0);
  await page.evaluate(()=>window.setGodotHybridViewSettings({modeled_workspace_center:[0,-.5,-1]}));
  await page.waitForFunction(()=>+document.querySelector('#workspace-edit-coverage').dataset.kept>0);
  // Exercise the editor's own keyboard handle path, not just the settings API.
  const y=page.getByRole('slider',{name:'Move Y',exact:true});await y.focus();await page.keyboard.press('ArrowUp');
  await page.waitForFunction(()=>Math.abs(window.tableCoverageProbe.state.settings.modeled_workspace_center[1]+.49)<1e-6);
  const actualY=await page.evaluate(()=>window.tableCoverageProbe.state.settings.modeled_workspace_center[1]);assert.ok(Math.abs(actualY+.49)<1e-6);
  // Cropped input cannot be used as an apparent full-scene denominator.
  const waiting=await page.evaluate(()=>{
    const {state,render}=window.tableCoverageProbe;for(const c of state.cameras)c.metadata.stream_depth_origin=[0,0];render();
    const text=document.querySelector('#workspace-edit-coverage').textContent;
    for(const c of state.cameras)delete c.metadata.stream_depth_origin;render();return text;
  });assert.match(waiting,/waiting for full camera frames/);
  await page.waitForFunction(()=>document.querySelector('#workspace-edit-coverage').dataset.state==='ready');
  const layouts=[];
  for(const [width,height]of [[1366,900],[1920,1080]]){
    await page.setViewportSize({width,height});
    const size=await page.locator('.workspace-edit-toolbar').evaluate(el=>({overflow:el.scrollWidth>el.clientWidth+1,bottom:el.getBoundingClientRect().bottom,height:innerHeight}));
    assert.equal(size.overflow,false);assert.ok(size.bottom<=size.height);
    await page.screenshot({path:resolve(output,'editor-synthetic-'+width+'.png')});layouts.push({width,height,...size});
  }
  await page.getByRole('button',{name:'Cancel',exact:true}).click();
  const cancelled=await page.evaluate(()=>({settings:window.tableCoverageProbe.state.settings,closed:window.tableCoverageProbe.state.workspaceEditor.coverage===null}));
  assert.deepEqual(cancelled.settings.modeled_workspace_center,before.modeled_workspace_center);assert.equal(cancelled.closed,true);
  await page.evaluate(()=>window.editGodotWorkspace('crop'));assert.equal(await coverage.isVisible(),false,'ordinary box and robot pose editing do not start the table counter');
  await page.getByRole('button',{name:'Cancel',exact:true}).click();
  assert.deepEqual(errors,[]);
  // Measure the actual worker on the captured three-camera native Quality frame.
  const captured=readFileSync(resolve(root,'.teleop/table-performance-20261026/quality.rgbd'));
  await page.route('**/coverage-profile.rgbd',route=>route.fulfill({contentType:'application/octet-stream',body:captured}));
  await page.goto('http://127.0.0.1:'+port+'/coverage-profile-page');
  const performanceResult=await page.evaluate(async()=>{
    const packet=await(await fetch('/coverage-profile.rgbd')).arrayBuffer(),n=new DataView(packet).getUint32(5,true),metadata=JSON.parse(new TextDecoder().decode(new Uint8Array(packet,9,n)));
    const {decodeIndependentDepth}=await import('/rgbd_decode_worker.js');let offset=9+n;const cameras=[];
    for(const c of metadata.cameras){offset+=c.color_length;const depth=await decodeIndependentDepth(new Uint8Array(packet.slice(offset,offset+c.depth_length)),c);offset+=c.depth_length;
      cameras.push({key:c.id,width:c.width,height:c.height,depth,modelMatrix:c.transform,intrinsics:c.intrinsics});}
    const {modeledWorkspace}=await import('/modeled_workspace.js');const center=metadata.robot.links.base_link.origin.slice();center[1]=.05796116099327442;center[2]=-.6605590408100735;
    const table=modeledWorkspace({modeled_workspace_enabled:true,modeled_workspace_center:center,modeled_workspace_heading:5.600540419595404});
    const worker=new Worker('/workspace_coverage_worker.js',{type:'module'}),rows=[];
    try{for(let i=0;i<6;i++){const start=performance.now();const response=await new Promise((accept,reject)=>{worker.onmessage=({data})=>accept(data);worker.onerror=e=>reject(Error(e.message));worker.postMessage({id:i,keys:cameras.map(c=>c.key),updates:i===0?cameras:[],table:{...table,center:[center[0],center[1]+i*.001,center[2]]}});});
      if(response.error)throw Error(response.error);rows.push({...response,roundTripMs:performance.now()-start});}
      return {depthSamples:cameras.reduce((n,c)=>n+c.depth.length,0),rows,renderingArraysPreserved:cameras.every(c=>c.depth.byteLength>0)};
    }finally{worker.terminate();}
  });
  writeFileSync(resolve(output,'browser-results.json'),JSON.stringify({syntheticUi:true,initial,layouts,errors,cancelRestoresPlacement:true,workerReleasedOnExit:true,waitsForUncroppedFrames:true,performance:performanceResult},null,2));
  console.log(JSON.stringify({success:true,initial,performance:performanceResult}));
}finally{await browser?.close();fixture.kill();}})().catch(e=>{console.error(e);fixture.kill();process.exitCode=1;});
