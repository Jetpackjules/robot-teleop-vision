// Run the camera-free visibility shader regressions on a real WebGL2 context.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const assert=require('node:assert/strict');
const {mkdirSync,writeFileSync}=require('node:fs');
const {resolve}=require('node:path');
const {createTestServer}=require('./serve_web_rendering.cjs');
const server=createTestServer();
(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  let browser;
  try {
    browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
    const page=await browser.newPage();
    const errors=[];page.on('pageerror',error=>errors.push(error.message));
    await page.goto(`http://127.0.0.1:${server.address().port}/rgb-visibility`);
    await page.waitForFunction(()=>document.getElementById('result').textContent.trim().startsWith('{'),null,{timeout:60000});
    const result=JSON.parse(await page.locator('#result').textContent());
    const output=resolve(__dirname,'../.teleop/rgb-visibility-tests');mkdirSync(output,{recursive:true});
    writeFileSync(resolve(output,'result.json'),JSON.stringify({...result,pageErrors:errors},null,2)+'\n');
    assert.deepEqual(errors,[]);assert.equal(result.failures,0,result.error);
    console.log(JSON.stringify({checks:result.passed.length,failures:result.failures}));
    await page.goto(`http://127.0.0.1:${server.address().port}/?autorun=1&raw=1`,{waitUntil:'commit'});
    await page.waitForFunction(()=>window.testResult||window.testError,null,{timeout:300000});
    const integration=await page.evaluate(()=>({result:window.testResult,error:window.testError}));
    writeFileSync(resolve(output,'original-rgb-result.json'),JSON.stringify({...integration,pageErrors:errors},null,2)+'\n');
    assert.deepEqual(errors,[]);assert.ok(!integration.error,integration.error);assert.ok(integration.result.success);
    console.log(JSON.stringify({originalRgbChecks:integration.result.checks,success:integration.result.success}));
  } finally {if(browser)await browser.close();await new Promise(resolve=>server.close(resolve));}
})().catch(error=>{console.error(error);process.exitCode=1;});
