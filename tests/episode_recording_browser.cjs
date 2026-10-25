const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {spawn}=require('node:child_process');
const {resolve}=require('node:path');
const {mkdirSync,readFileSync,writeFileSync}=require('node:fs');
const assert=require('node:assert/strict');
const root=resolve(__dirname,'..'),output=resolve(root,'.teleop/recording-20261025');mkdirSync(output,{recursive:true});
const episodeRoot=resolve(output,'browser-episodes-'+Date.now());
const server=spawn(process.env.TEST_PYTHON||resolve(root,'.venv/Scripts/python.exe'),
  [resolve(__dirname,'serve_recording_fixture.py'),'--port','0','--output',episodeRoot],
  {cwd:root,stdio:['ignore','pipe','pipe'],windowsHide:true});
(async()=>{
  let browser;const diagnostics=[];server.stderr.on('data',d=>diagnostics.push(d.toString()));
  try {
    const port=await new Promise((resolve,reject)=>{
      let data='';const timer=setTimeout(()=>reject(new Error('Synthetic host did not start: '+diagnostics.join('').slice(-1200))),20000);
      server.stdout.on('data',chunk=>{data+=chunk.toString();for(const line of data.split('\n')){try{const result=JSON.parse(line);if(result.port){clearTimeout(timer);resolve(result.port);return;}}catch{}}});
      server.on('exit',()=>reject(new Error('Synthetic host exited: '+diagnostics.join('').slice(-1200))));
    });
    browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
    const page=await browser.newPage({viewport:{width:1200,height:950}}),errors=[];
    page.on('pageerror',error=>errors.push(error.message));
    await page.goto(`http://127.0.0.1:${port}/controller.html`,{waitUntil:'domcontentloaded'});
    // The fixture disables webcam startup; the actual recording controls and
    // host API remain unchanged.
    const start=page.getByRole('button',{name:'Start recording',exact:true});
    await page.getByRole('textbox',{name:'Task instruction',exact:true}).fill('Place the synthetic block in the cup');
    await start.waitFor({state:'visible'});
    await page.waitForFunction(()=>!document.querySelector('[data-recording-start]').disabled);
    await start.click();
    await page.waitForFunction(()=>document.getElementById('episode-recording').dataset.recordingState==='recording');
    await page.getByRole('combobox',{name:'Episode outcome',exact:true}).selectOption('success');
    await page.waitForFunction(()=>document.querySelector('[data-recording-status]').textContent.includes('frames'));
    await page.locator('#episode-recording').screenshot({path:resolve(output,'recording-desktop.png')});
    await page.getByRole('button',{name:'Stop and save',exact:true}).click();
    await page.waitForFunction(()=>document.querySelector('[data-recording-status]').textContent.includes('complete · success'));
    const saved=await page.evaluate(async()=>await(await fetch('/api/v1/recording?client=recording-fixture')).json());
    assert.ok(saved.last_saved.frames>=2);
    const manifest=resolve(episodeRoot,saved.last_saved.episode,'episode.json');
    const record=JSON.parse(readFileSync(manifest,'utf8'));record.synthetic=true;writeFileSync(manifest,JSON.stringify(record,null,2));
    await page.setViewportSize({width:390,height:844});
    await start.scrollIntoViewIfNeeded();await start.click();
    await page.waitForFunction(()=>document.getElementById('episode-recording').dataset.recordingState==='recording');
    assert.equal(await page.getByRole('combobox',{name:'Episode outcome',exact:true}).inputValue(),'unrated','new episode resets its outcome');
    await page.screenshot({path:resolve(output,'recording-mobile.png'),fullPage:true});
    assert.ok(await page.locator('#episode-recording').evaluate(el=>el.scrollWidth<=el.clientWidth+1),'recording controls fit mobile');
    await page.getByRole('button',{name:'Discard',exact:true}).click();
    await page.waitForFunction(()=>document.querySelector('[data-recording-status]').textContent.includes('discarded'));
    assert.deepEqual(errors,[]);
    const result={desktop:true,mobile:true,start_stop:true,outcomes:true,discard:true,synthetic:true,
      savedFrames:saved.last_saved.frames,episode:saved.last_saved.episode,episodeRoot};
    writeFileSync(resolve(output,'browser-results.json'),JSON.stringify(result,null,2));console.log(JSON.stringify(result));
  } finally {await browser?.close();server.kill();}
})().catch(error=>{console.error(error);server.kill();process.exitCode=1;});
