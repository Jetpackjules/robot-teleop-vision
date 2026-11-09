const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {spawn}=require('node:child_process'),{resolve}=require('node:path');
const {mkdirSync,writeFileSync}=require('node:fs'),assert=require('node:assert/strict');
const root=resolve(__dirname,'..'),output=resolve(root,'.teleop/recording-audit-20261026');mkdirSync(output,{recursive:true});
const fixture=spawn(process.env.TEST_PYTHON||resolve(root,'.venv/Scripts/python.exe'),
  [resolve(__dirname,'serve_recording_fixture.py'),'--port','0','--output',resolve(output,'crop-ui-fixture')],
  {cwd:root,stdio:['ignore','pipe','pipe'],windowsHide:true});
(async()=>{let browser;const logs=[],errors=[];fixture.stderr.on('data',d=>logs.push(d.toString()));
try{
  const port=await new Promise((accept,reject)=>{let text='';const timer=setTimeout(()=>reject(Error(logs.join('').slice(-2000))),20000);
    fixture.stdout.on('data',d=>{text+=d;for(const line of text.split('\n'))try{const value=JSON.parse(line);if(value.port){clearTimeout(timer);accept(value.port);}}catch{}});
    fixture.on('exit',()=>reject(Error('Fixture exited: '+logs.join('').slice(-1000))));});
  browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
  const page=await browser.newPage({viewport:{width:1366,height:900}});page.on('pageerror',e=>errors.push(e.message));
  await page.route('http://localhost:*/live.js*',r=>r.abort());
  await page.goto('http://127.0.0.1:'+port+'/controller.html?rgbd_transport=websocket',{waitUntil:'domcontentloaded'});
  await page.waitForFunction(()=>document.querySelector('#so101-hold')&&document.querySelector('#episode-recording').dataset.recordingState==='idle');
  await page.locator('#open-advanced').click();await page.locator('#tab-view').click();
  await page.locator('#display-mode').selectOption('blendpcr');await page.locator('#modeled-workspace-enabled').check();
  await page.evaluate(()=>{
    const original=window.getGodotHybridRendererStatus;
    window.getGodotHybridRendererStatus=()=>({...original(),displaySource:'rgbd',streamCrop:{applied:true,input_bytes:1000000,output_bytes:687000,source_samples:100000,retained_samples:12000,source_color_pixels:2000000,color_pixels:2000000},workspaceCrop:{sourceCells:10000,submittedCells:4200}});
  });
  const status=page.locator('#modeled-workspace-status');
  await page.waitForFunction(()=>document.querySelector('#modeled-workspace-status').textContent.includes('68.7% sent'));
  assert.match(await status.textContent(),/68\.7% sent.*12\.0% kept/s);
  await status.locator('summary').click();await status.locator('summary').focus();
  await page.waitForTimeout(1300);assert.equal(await status.locator('details').getAttribute('open'),'');
  assert.equal(await status.locator('summary').evaluate(el=>el===document.activeElement),true,'polling preserves help disclosure and keyboard focus');
  assert.match(await status.textContent(),/do not measure visible screen area/);
  const layouts=[];
  for(const [width,height]of [[1366,900],[1920,1080]]){
    await page.setViewportSize({width,height});await status.scrollIntoViewIfNeeded();
    const overflow=await status.evaluate(el=>el.scrollWidth>el.clientWidth+1);assert.equal(overflow,false);
    await page.screenshot({path:resolve(output,'crop-statistics-synthetic-'+width+'.png')});layouts.push({width,height,overflow});
  }
  await page.locator('#display-mode').selectOption('rgb_camera');
  await page.waitForFunction(()=>document.querySelector('#modeled-workspace-status').textContent.includes('Table statistics apply to 3D views'));
  assert.equal(await status.locator('dl').count(),0);assert.deepEqual(errors,[]);
  writeFileSync(resolve(output,'crop-browser-results.json'),JSON.stringify({synthetic:true,layouts,errors,distinctDenominators:true,disclosureAndFocusPreserved:true,rgbModeClearsStaleCounts:true},null,2));
  console.log('Crop-statistics browser checks passed');
}finally{await browser?.close();fixture.kill();}})().catch(e=>{console.error(e);fixture.kill();process.exitCode=1;});
