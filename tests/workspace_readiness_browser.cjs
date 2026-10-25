const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {resolve}=require('node:path');const {mkdirSync,writeFileSync}=require('node:fs');
const {execFileSync}=require('node:child_process');const assert=require('node:assert/strict');
const {createTestServer}=require('./serve_web_rendering.cjs');
const root=resolve(__dirname,'..'),output=resolve(root,'.teleop/offline-ready-20261021');mkdirSync(output,{recursive:true});
const python=process.env.TEST_PYTHON||resolve(root,'.venv/Scripts/python.exe');
execFileSync(python,[resolve(__dirname,'workspace_readiness_fixture.py'),'--output',resolve(output,'fixture.json')]);
const server=createTestServer();
(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));let browser;
  try {
    browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
    const page=await browser.newPage({viewport:{width:1200,height:950},deviceScaleFactor:1});
    const errors=[],diagnostics=[];
    page.on('pageerror',error=>errors.push(error.message));
    page.on('console',message=>{if(['error','warning'].includes(message.type()))diagnostics.push(message.text());});
    await page.goto(`http://127.0.0.1:${server.address().port}/?ready=1&autorun=1`,{waitUntil:'commit'});
    await page.waitForFunction(()=>window.testResult||window.testError,null,{timeout:120000});
    const result=await page.evaluate(()=>({result:window.testResult,error:window.testError,captures:window.demoCaptures}));
    await page.screenshot({path:resolve(output,'readiness-page.png'),fullPage:true});
    assert.ok(!result.error,[result.error,...errors,...diagnostics.slice(-12)].join('\n'));
    assert.deepEqual(errors,[]);
    for(const [name,data]of Object.entries(result.captures||{}))writeFileSync(resolve(output,`so101-${name}.png`),Buffer.from(data.split(',')[1],'base64'));
    writeFileSync(resolve(output,'results.json'),JSON.stringify(result.result,null,2));
    console.log(JSON.stringify({...result.result,output}));
  } finally{await browser?.close();await new Promise(resolve=>server.close(resolve));}
})().catch(error=>{console.error(error);process.exitCode=1;});
