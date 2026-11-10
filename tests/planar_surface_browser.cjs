const assert = require('node:assert/strict');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');

(async () => {
  const server = createTestServer();
  await new Promise(resolve => server.listen(0,'127.0.0.1',resolve));
  const browser=await chromium.launch({headless:true,channel:'msedge',args:['--enable-unsafe-swiftshader']});
  try {
    const page=await browser.newPage();
    await page.goto('http://127.0.0.1:'+server.address().port+'/rgb-visibility');
    const result=await page.evaluate(async()=>{
      const {SquareSurfacePreview}=await import('/square_surface_preview.js');
      const canvas = document.createElement('canvas');
      canvas.width=canvas.height=160;
      const gl=canvas.getContext('webgl2',{preserveDrawingBuffer:true,antialias:false});
      const preview = new SquareSurfacePreview(gl);
      const viewer=new Float32Array([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]);
      const projection = viewer.slice();
      projection[0]=projection[5]=12;
      projection[10]=-1.001; projection[14]=-.10005;
      const zBuffer = z => .5*(1.-projection[10]+projection[14]/z);
      const ink = new Float32Array([.2,.45,.8,1]);
      const clearDepth=new Float32Array(1);
      const center=[80,80];
      const pixel=(x,y)=>{const rgba=new Uint8Array(4);gl.readPixels(x,y,1,1,gl.RGBA,gl.UNSIGNED_BYTE,rgba);return [...rgba];};
      function run(enabled, badDepth=false, tablePlane=undefined) {
        preview.begin(160,160);
        gl.enable(gl.SCISSOR_TEST);
        // Four samples around a 2D hole plus a diagonal witness.
        for(const [x,y] of [[76,80],[84,80],[80,76],[80,84],[83,83]]){
          gl.scissor(x,y,1,1);
          gl.clearBufferfv(gl.COLOR,0,ink);
          clearDepth[0]=zBuffer(badDepth&&x===84?.46:.40);
          gl.clearBufferfv(gl.DEPTH,0,clearDepth);
        }
        gl.disable(gl.SCISSOR_TEST);
        gl.bindFramebuffer(gl.FRAMEBUFFER,null);
        gl.clearColor(0,0,0,0);
        gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
        preview.draw({projection,viewer},tablePlane,
          {squares_fill_surface_gaps:false,squares_plane_completion:enabled});
        return {center:pixel(...center),anchor:pixel(76,80),unsupported:pixel(65,80)};
      }
      return {off:run(false),on:run(true),step:run(true,true),table:run(true,false,0)};
    });
    assert.equal(result.off.center[0],0);
    assert.ok(result.on.center[2]>150,'supported plane fills the gap');
    assert.deepEqual(result.on.anchor,result.off.anchor,'measured color stays unchanged');
    assert.equal(result.on.unsupported[0],0,'unsupported silhouette stays empty');
    assert.equal(result.step.center[0],0,'reject contradictory measured depth');
    assert.equal(result.table.center[0],0,'preserve the tabletop cutoff');
    console.log(result);
  } finally {
    await browser.close();
    server.close();
  }
})().catch(error=>{console.error(error);process.exitCode=1;});
