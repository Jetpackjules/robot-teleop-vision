const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const assert=require('node:assert/strict');
const {createTestServer}=require('./serve_web_rendering.cjs');
const server=createTestServer();
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
 try{
  const page=await browser.newPage({viewport:{width:1200,height:900}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.route('**/hybrid_rgbd_renderer.js*',async route=>{
   const response=await route.fetch();await route.fulfill({response,body:await response.text()+'\nwindow.attachPickingProbe=attachPointerNavigation;'});
  });
  await page.goto(`http://127.0.0.1:${server.address().port}/`);
  await page.addStyleTag({content:'canvas{width:900px;height:650px;border:0}'});
  await page.evaluate(async()=>{
   await import('/hybrid_rgbd_renderer.js');const f=await import('/web_rendering_fixture.js');f.setup();
   const p=window.probe,s=p.state;
   window.focusEvents=0;window.attachPickingProbe({onFocus:()=>window.focusEvents++});
   const unit=a=>a.map(v=>v/Math.hypot(...a)),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
   window.pickView=({center=[0,0,0],heading=0,eye=[1.3,1.2,2],target=[0,-.15,0]}={})=>{
    s.cameras=[];s.manualPan=[0,0,0];s.focusPicked=false;s.focusWorld=null;
    Object.assign(s.settings,{modeled_workspace_enabled:true,modeled_workspace_center:center,modeled_workspace_heading:heading,
     modeled_workspace_width:1.2,modeled_workspace_depth:.8,modeled_workspace_table_height:.72,
     inspect_enabled:false,robot_overlay_enabled:false,display_mode:'independent_mesh',fov:55});
    const z=unit(eye.map((v,i)=>v-target[i])),x=unit(cross([0,1,0],z)),y=cross(z,x);
    s.baseViewer={transform:new Float32Array([...x,0,...y,0,...z,0,...eye,1]),fov:55,near:.02,far:20};p.render();
   };
   window.projectPick=point=>{
    const m=s.viewProjection,v=[...point,1],q=[0,1,2,3].map(r=>v.reduce((sum,x,c)=>sum+m[c*4+r]*x,0));
    const rect=s.canvas.getBoundingClientRect();return [rect.left+(q[0]/q[3]+1)*rect.width/2,rect.top+(1-q[1]/q[3])*rect.height/2];
   };
   window.fakePickSample=point=>{
    const matrix=p.identityMatrix();matrix[12]=point[0];matrix[13]=point[1];matrix[14]=point[2]+1;
    s.cameras=[{width:1,height:1,metadata:{intrinsics:[1,1,0,0]},depth:new Uint16Array([1000]),modelMatrix:matrix}];
   };
  });
  // Native pointer coordinates are integer CSS pixels; allow that projected footprint.
  const near=(actual,expected)=>assert.ok(actual.every((v,i)=>Math.abs(v-expected[i])<.008),`${actual} should match ${expected}`);
  const click=async point=>{const xy=await page.evaluate(p=>window.projectPick(p),point);await page.mouse.dblclick(...xy);return page.evaluate(()=>window.probe.state.focusWorld);};
  await page.evaluate(()=>window.pickView());
  near(await click([.12,0,.08]),[.12,0,.08]);
  await page.evaluate(()=>window.pickView());
  assert.equal(await click([1,-.72,.6]),null,'floor clicks leave the pivot unchanged');
  await page.evaluate(()=>window.pickView());
  assert.equal(await click([.545,-.4,.345]),null,'table legs cannot become orbit targets');
  const center=[.2,.12,-.4],heading=37,a=heading*Math.PI/180;
  const target=[center[0]+Math.cos(a)*.12+Math.sin(a)*.08,center[1],center[2]-Math.sin(a)*.12+Math.cos(a)*.08];
  await page.evaluate(({center,heading})=>window.pickView({center,heading}),{center,heading});
  near(await click(target),target);
  // The sensor surface in front of the modeled tabletop remains selectable.
  await page.evaluate(()=>{window.pickView();window.fakePickSample([0,.15,0]);});
  near(await click([0,.15,0]),[0,.15,0]);
  // From below, an otherwise valid sample hidden behind the modeled slab cannot steal the pick.
  await page.evaluate(()=>{window.pickView({eye:[0,-.2,1],target:[0,.15,-.3]});window.fakePickSample([0,.15,-.3]);});
  assert.equal(await click([0,.15,-.3]),null,'a table edge cannot focus a hidden sample or become the target itself');
  // Rendered arm triangles are selectable, including when the table is disabled.
  await page.evaluate(()=>{
    window.pickView();const {state:s,createProgram,findRobotUniforms,ROBOT_VERTEX_SHADER,ROBOT_FRAGMENT_SHADER,identityMatrix}=window.probe;
    s.robotProgram=createProgram(s.gl,ROBOT_VERTEX_SHADER,ROBOT_FRAGMENT_SHADER);s.robotVao=s.gl.createVertexArray();
    s.robotUniforms=findRobotUniforms(s.gl,s.robotProgram);
    s.robotMeshes={arm:[{cpuPositions:new Float32Array([-.3,0,0,.3,0,0,0,.4,0]),cpuIndices:null,mode:4,localMatrix:identityMatrix()}]};
    s.robotTransforms={arm:{basis_x:[1,0,0],basis_y:[0,1,0],basis_z:[0,0,1],origin:[0,.1,0]}};
    Object.assign(s.settings,{robot_overlay_enabled:true,robot_overlay_scope:'full',modeled_workspace_enabled:true});
  });
  near(await click([0,.25,0]),[0,.25,0]);
  await page.evaluate(()=>{window.probe.state.focusWorld=null;window.probe.state.settings.modeled_workspace_enabled=false;});
  near(await click([0,.25,0]),[0,.25,0]);
  await page.evaluate(()=>{window.probe.state.focusWorld=null;window.probe.state.settings.robot_overlay_scope='claw';});
  assert.equal(await click([0,.25,0]),null,'hidden arm links cannot be picked in claw-only mode');
  await page.evaluate(()=>{window.probe.state.settings.robot_overlay_scope='full';window.probe.state.settings.robot_overlay_enabled=false;});
  assert.equal(await click([0,.25,0]),null,'disabled arm overlays cannot be picked');
  await page.evaluate(()=>{window.pickView();window.probe.state.settings.modeled_workspace_enabled=false;});
  assert.equal(await page.evaluate(()=>window.focusGodotHybridRendererAt(450,400)),false,'disabled table cannot be picked');
  const state=await page.evaluate(()=>({events:window.focusEvents,pan:window.probe.state.manualPan,heading:window.probe.state.settings.modeled_workspace_heading}));
  assert.equal(state.events,5);assert.deepEqual(state.pan,[0,0,0]);assert.deepEqual(errors,[]);
  console.log('Orbit targets: tabletop, visible camera objects and arm overlay; floor, legs, edges, hidden/disabled links rejected.');
 }finally{await browser.close();await new Promise(r=>server.close(r));}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
