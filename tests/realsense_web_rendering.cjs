// CI wrapper for the same camera-free test page used during visual checks.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {mkdirSync}=require('node:fs');
const {resolve}=require('node:path');
const assert=require('node:assert/strict');
const {createTestServer}=require('./serve_web_rendering.cjs');
const server=createTestServer();
(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  const browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
  try {
    const page=await browser.newPage({viewport:{width:1200,height:900},deviceScaleFactor:1});
    const errors=[];page.on('pageerror',error=>errors.push(error.message));
    await page.goto(`http://127.0.0.1:${server.address().port}/?autorun=1`);
    await page.waitForFunction(()=>window.testResult||window.testError,null,{timeout:60000});
    const result=await page.evaluate(()=>({result:window.testResult,error:window.testError}));
    assert.ok(!result.error,result.error);assert.deepEqual(errors,[]);
    const output=resolve(__dirname,'../.teleop/multicamera-web-tests');mkdirSync(output,{recursive:true});
    await page.screenshot({path:resolve(output,'blendpcr-webgl.png')});
    console.log(JSON.stringify(result.result));
  } finally {await browser.close();server.close();}
})().catch(error=>{console.error(error);server.close();process.exitCode=1;});
