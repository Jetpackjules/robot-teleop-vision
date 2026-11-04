const assert=require('node:assert/strict');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');
const server=createTestServer();
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const browser=await chromium.launch({headless:true,channel:'msedge'});
 try{
  const page=await browser.newPage();await page.goto(`http://127.0.0.1:${server.address().port}/`);
  const result=await page.evaluate(async()=>{
   await import('/hybrid_rgbd_renderer.js');const f=await import('/web_rendering_fixture.js'),{gl}=f.setup(),p=window.probe;
   const m=await import('/claw_mesh_mask.js'),{NativeRobotMask}=await import('/native_robot_mask.js');
   if(!await m.loadClawMeshMask(gl))throw Error('SDF download failed');
   let checks=0;const check=(value,label)=>{checks++;if(!value)throw Error(label);};
   // Independent CAD nearest-surface measurements: each query is 5/20/60mm
   // past the actual distal extremum, not past a joint origin.
   const tips=[[-.0088999998,-.0002179344,-.1044252933],[-.0113000004,-.0820000023,.0189]];
   const axes=[2,1],native=new NativeRobotMask(gl),depth=gl.createTexture();
   gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,depth);
   gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);
   gl.texImage2D(gl.TEXTURE_2D,0,gl.R16UI,1,1,0,gl.RED_INTEGER,gl.UNSIGNED_SHORT,new Uint16Array([1000]));
   const mask={starts:new Float32Array(24).fill(100),ends:new Float32Array(24).fill(100)};
   for(let i=0;i<6;i++)mask.starts[4*i+3]=mask.ends[4*i+3]=0;
   const camera={width:1,height:1,depthTexture:depth,intrinsicsArray:new Float32Array([1,1,0,0]),modelMatrix:p.identityMatrix(),uploadRevision:1};
   const query=(point,links,buffer,enabled=true)=>{
    mask.clawSignature=m.updateClawMeshMask(gl,links.map(p.inverseRigidTransform),buffer,enabled);
    camera.modelMatrix=p.identityMatrix();camera.modelMatrix.set([point[0],point[1],point[2]+1],12);
    native.update([camera],mask,[0,0,1,1]);
    gl.bindFramebuffer(gl.FRAMEBUFFER,native.framebuffer);gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,native.texture,0,0);
    const value=new Uint8Array(1);gl.readPixels(0,0,1,1,gl.RED_INTEGER,gl.UNSIGNED_BYTE,value);
    gl.bindFramebuffer(gl.FRAMEBUFFER,null);return value[0];
   };
   for(let link=0;link<2;link++)for(const angle of [0,.9]){
    const own=p.identityMatrix(),other=p.identityMatrix();other[12]=2;
    own[0]=own[5]=Math.cos(angle);own[1]=Math.sin(angle);own[4]=-Math.sin(angle);
    own.set([.2,-.1,-1],12);
    const links=link===0?[own,other]:[other,own];
    for(const distance of [.005,.02,.06]){
     const local=[...tips[link]];local[axes[link]]-=distance;
     const point=[own[0]*local[0]+own[4]*local[1]+local[2]*own[8]+own[12],own[1]*local[0]+own[5]*local[1]+local[2]*own[9]+own[13],local[2]+own[14]];
     for(const buffer of [0,.01,.03,.05])check(query(point,links,buffer)===(distance<buffer?1:0),`link ${link}, angle ${angle}, distance ${distance}, buffer ${buffer}`);
    }
   }
   const links=[p.identityMatrix(),p.identityMatrix()];links[1][12]=2;
   const point=[...tips[0]];point[2]-=.005;
   check(query(point,links,.01)===1,'near-tip surface masked');
   const before=native.rebuilds;query(point,links,.01);check(native.rebuilds===before,'unchanged pose reuses mask');
   links[0][0]=links[0][5]=0;links[0][1]=1;links[0][4]=-1;
   check(query(point,links,.01)===0&&native.rebuilds===before+1,'rotation around an unchanged joint pivot updates the mask');
   check(query(point,links,.05,false)===0,'disabled mesh mask keeps objects visible');
   check(gl.getError()===gl.NO_ERROR,'valid GL state');
   return {checks,grid:96,links:2};
  });
  assert.ok(result.checks>=53);console.log(JSON.stringify(result));
 }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
