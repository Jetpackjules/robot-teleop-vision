// Actual WebGL2 rendering, synthetic frames only. No network camera or robot.
const { chromium } = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const { readFileSync, mkdirSync } = require('node:fs');
const { resolve } = require('node:path');
const http = require('node:http');
const assert = require('node:assert/strict');
const root = resolve(__dirname, '..');
const output = resolve(root, '.teleop/multicamera-web-tests');
mkdirSync(output, {recursive:true});
const server = http.createServer((req,res) => {
  const path = new URL(req.url,'http://localhost').pathname;
  if (['/hybrid_rgbd_renderer.js','/hybrid_camera_fusion.js'].includes(path)) {
    res.setHeader('Content-Type','text/javascript');
    let text = readFileSync(resolve(root,'web',path.slice(1)),'utf8');
    if (path === '/hybrid_rgbd_renderer.js') text += '\nwindow.probe={state,createProgram,findUniforms,render,uploadCamera,identityMatrix,inverseRigidTransform};';
    res.end(text);
  } else {
    res.setHeader('Content-Type','text/html');
    res.end('<canvas id="view" style="width:128px;height:128px"></canvas><script type="module" src="/hybrid_rgbd_renderer.js"></script>');
  }
});
(async()=>{
  await new Promise(r=>server.listen(0,'127.0.0.1',r));
  const browser = await chromium.launch({headless:true, ...(process.env.TEST_BROWSER_CHANNEL ? {channel:process.env.TEST_BROWSER_CHANNEL} : {})});
  try {
    const page = await browser.newPage({viewport:{width:600,height:450},deviceScaleFactor:1});
    const errors=[];
    page.on('pageerror',e=>errors.push(e.message));
    await page.goto(`http://127.0.0.1:${server.address().port}/`);
    await page.waitForFunction(()=>window.probe);
    const result = await page.evaluate(async()=>{
      const {MultiCameraFusion,chooseColorReference,orderedCameras} = await import('/hybrid_camera_fusion.js');
      const {state,createProgram,findUniforms,render,uploadCamera,identityMatrix} = window.probe;
      const canvas = document.getElementById('view');
      const gl = canvas.getContext('webgl2',{antialias:false,preserveDrawingBuffer:true});
      if (!gl) throw Error('WebGL2 unavailable');
      Object.assign(state,{canvas,gl,running:true,program:createProgram(gl),cameraFusion:new MultiCameraFusion(gl),
        baseViewer:{transform:identityMatrix(),fov:55,near:0.01,far:10}});
      state.uniforms=findUniforms(gl,state.program);
      Object.assign(state.settings,{inspect_enabled:false,robot_overlay_enabled:false,workspace_surface_enabled:false,workspace_boundary_enabled:false});
      gl.useProgram(state.program);
      for(const [key,unit] of [['depth',0],['color',1],['fusionDepth',2],['fusionColor',3],['rigDepth',4],['rigData',5]]) gl.uniform1i(state.uniforms[key],unit);
      gl.uniform1f(state.uniforms.maxPointSize,32);
      let checks=0;
      function check(ok,label) { checks++; if(!ok) throw Error(label); }
      const palette=[[255,0,0],[0,255,0],[0,0,255],[255,255,0]];
      function fixture(depths=[1008,1006,1004,1002],sizes=[64,64,64,64]) {
        for(const c of state.cameras){gl.deleteTexture(c.depthTexture);gl.deleteTexture(c.colorTexture);}
        state.cameras=[];
        depths.forEach((z,i)=>{
          const n=sizes[i], pixels=new Uint8Array(n*n*3);
          for(let p=0;p<n*n;p++) pixels.set(palette[i],p*3);
          const depth=new Uint16Array(n*n);depth.fill(z);
          const c=uploadCamera({metadata:{id:String(i),model:'D455',priority:0,intrinsics:[n,n,(n-1)/2,(n-1)/2]},
            width:n,height:n,depth,colorPixels:pixels,modelMatrix:identityMatrix()});
          state.cameras.push(c);
        });
        state.cameraFusion.clear();
      }
      function sample(mode,label,expected){
        window.setGodotHybridViewSettings({display_mode:mode});
        render();
        const rgba=new Uint8Array(4);gl.readPixels(65,65,1,1,gl.RGBA,gl.UNSIGNED_BYTE,rgba);
        check(gl.getError()===gl.NO_ERROR,label+' GL error');
        check(expected.every((v,i)=>Math.abs(v-rgba[i])<8),label+': '+rgba);
      }
      fixture();
      sample('independent_mesh','independent keeps closest fourth surface',palette[3]);
      sample('unified_mesh','four-camera overlap uses first priority',palette[0]);
      check(state.cameraFusion.depth!==null,'texture layers allocated');
      const texture=state.cameraFusion.depth;
      render();check(state.cameraFusion.depth===texture,'unchanged frames reuse layers');
      for(let missing=0;missing<3;missing++){
        state.cameras[missing].depth.fill(0);state.cameras[missing].lastUpdateMs=performance.now();
        gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,state.cameras[missing].depthTexture);
        gl.texSubImage2D(gl.TEXTURE_2D,0,0,0,64,64,gl.RED_INTEGER,gl.UNSIGNED_SHORT,state.cameras[missing].depth);
        sample('unified_mesh','missing views fall through to '+(missing+1),palette[missing+1]);
      }
      fixture([1000,1000,1000,850]);sample('unified_mesh','distinct foreground remains visible',palette[3]);
      fixture();state.cameras[0].lastUpdateMs=performance.now()-1500;
      sample('unified_mesh','stale first camera does not own surface',palette[1]);
      fixture();state.cameras[0].modelMatrix[12]=2;
      sample('unified_mesh','translated first camera does not cover center',palette[1]);
      fixture([1008,1006,1004,1002],[64,48,80,64]);sample('unified_mesh','mixed resolutions',palette[0]);
      fixture();sample('shader_mesh','shared color works without D435',palette[0]);
      check(chooseColorReference(state.cameras).metadata.id==='0','D455 fallback reference');
      check(orderedCameras([...state.cameras].reverse())[0].metadata.id==='0','stable priority independent of arrival order');
      state.cameras[0].metadata.model='D435';state.cameras[0].lastUpdateMs=performance.now()-1500;
      check(chooseColorReference(state.cameras).metadata.id==='1','stale D435 excluded');
      fixture([1000],[16]);
      state.cameras[0].depth.fill(0);state.cameras[0].depth[8*16+8]=1000;
      gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,state.cameras[0].depthTexture);
      gl.texSubImage2D(gl.TEXTURE_2D,0,0,0,16,16,gl.RED_INTEGER,gl.UNSIGNED_SHORT,state.cameras[0].depth);
      window.setGodotHybridViewSettings({display_mode:'point_cloud',camera_color_fusion_enabled:false});
      render();
      function coverage(){const pixels=new Uint8Array(128*128*4);gl.readPixels(0,0,128,128,gl.RGBA,gl.UNSIGNED_BYTE,pixels);let n=0;for(let i=0;i<pixels.length;i+=4)if(pixels[i]>200)n++;return n;}
      const far=coverage();
      state.baseViewer.transform[14]=-0.4;render();
      check(coverage()>far*1.2,'points enlarge as viewer approaches');
      check(gl.getError()===gl.NO_ERROR,'point size rendering clean');
      fixture();state.baseViewer.transform=identityMatrix();sample('unified_mesh','final render',palette[0]);
      return {checks,renderer:gl.getParameter(gl.RENDERER)};
    });
    await page.screenshot({path:resolve(output,'four-camera-webgl.png')});
    assert.deepEqual(errors,[]);
    console.log(JSON.stringify(result));
  } finally {await browser.close();server.close();}
})().catch(error=>{console.error(error);server.close();process.exitCode=1;});
