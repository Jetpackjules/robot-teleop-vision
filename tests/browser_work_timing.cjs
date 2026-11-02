const assert=require('node:assert/strict'),fs=require('node:fs');
(async()=>{
  let now=0;Object.defineProperty(globalThis,'performance',{value:{now:()=>now},configurable:true});
  const source=fs.readFileSync(require('node:path').join(__dirname,'../web/browser_work_timing.js'),'utf8');
  const p=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
  const gl={drawArrays(x){assert.equal(this,gl);now+=100;return x;},readPixels(){now+=90;throw Error('driver');}};
  p.profileBrowserGL(gl);const wrapped=gl.drawArrays;p.profileBrowserGL(gl);assert.equal(gl.drawArrays,wrapped);
  assert.equal(gl.drawArrays(7),7);assert.throws(()=>gl.readPixels(),/driver/);
  assert.deepEqual(p.browserWorkTrace().map(x=>x.label),['WebGL.drawArrays','WebGL.readPixels']);
  assert.equal(p.timeBrowserWork('fast',()=>42),42);assert.equal(p.browserWorkTrace().length,2);
  for(let i=0;i<40;i++)p.timeBrowserWork('slow',()=>{now+=100;});
  assert.equal(p.browserWorkTrace().length,6);now+=60001;assert.equal(p.browserWorkTrace().length,0);
  console.log('PASS bounded attribution, native return/exception/context, idempotence and expiry');
})().catch(e=>{console.error(e);process.exitCode=1;});
