const assert = require('node:assert/strict');
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const {createTestServer} = require('./serve_web_rendering.cjs');
const server = createTestServer();
(async () => {
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  const browser = await chromium.launch({headless:true, ...(process.env.TEST_BROWSER_CHANNEL ? {channel:process.env.TEST_BROWSER_CHANNEL} : {})});
  try {
    const page = await browser.newPage({viewport:{width:1280,height:900}});
    await page.goto(`http://127.0.0.1:${server.address().port}/`);
    const result = await page.evaluate(async () => {
      await import('/hybrid_rgbd_renderer.js');
      const fixture = await import('/web_rendering_fixture.js');
      const {state,gl} = fixture.setup(), p = window.probe;
      state.robotRenderer = await (await fetch('/so101-renderer.json')).json();
      state.robotProgram = p.createProgram(gl,p.ROBOT_VERTEX_SHADER,p.ROBOT_FRAGMENT_SHADER);
      state.robotUniforms = p.findRobotUniforms(gl,state.robotProgram);
      state.robotVao = gl.createVertexArray();
      await p.loadRobotMeshes(gl,state.robotLoadToken);
      const names=Object.keys(state.robotMeshes);
      if(names.length!==7)throw Error('All seven real model links must load');
      for(const name of names)state.robotTransforms[name]={basis_x:[1,0,0],basis_y:[0,1,0],basis_z:[0,0,1],origin:[0,0,0]};
      for(const name of state.robotRenderer.maskChain.names)state.robotTransforms[name]??={basis_x:[1,0,0],basis_y:[0,1,0],basis_z:[0,0,1],origin:[0,0,0]};
      const initial=JSON.stringify(state.robotTransforms);
      let draws=0;
      for(const method of ['drawArrays','drawElements']){
        const original=gl[method].bind(gl);gl[method]=(...args)=>{draws++;return original(...args);};
      }
      const count=links=>links.reduce((sum,name)=>sum+state.robotMeshes[name].length,0);
      const all=count(names),claw=count(state.robotRenderer.endEffectorLinks);
      const render=scope=>{
        window.setGodotHybridViewSettings({robot_overlay_enabled:true,robot_overlay_style:'realistic',robot_overlay_scope:scope});
        gl.clearColor(0,0,0,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);draws=0;
        p.drawRobotOverlay(p.identityMatrix());return draws;
      };
      if(render('full')!==all)throw Error('Full arm lost model primitives');
      const fullMask=p.robotMaskCapsules();
      if(!fullMask)throw Error('Full arm mask was unavailable with every model loaded');
      if(render('claw')!==claw||claw>=all)throw Error('Claw mode must render only the two end-effector links');
      const clawMask=p.robotMaskCapsules();
      if(!clawMask||JSON.stringify([...clawMask.starts,...clawMask.ends])!==JSON.stringify([...fullMask.starts,...fullMask.ends]))throw Error('Claw-only visibility must keep exactly the full-arm occlusion mask');
      window.setGodotHybridViewSettings({robot_overlay_body_buffer_mm:50});
      const buffered=p.robotMaskCapsules();
      for(let i=0;i<state.robotRenderer.maskChain.radii.length;i++){
        const isClaw=state.robotRenderer.endEffectorLinks.includes(state.robotRenderer.maskChain.names[i]);
        for(const field of ['starts','ends']){
          for(let axis=0;axis<3;axis++)if(buffered[field][4*i+axis]!==fullMask[field][4*i+axis])throw Error('Buffer moved a calibrated endpoint');
          if(Math.abs(buffered[field][4*i+3]-fullMask[field][4*i+3]-(isClaw?0:.05))>1e-6)throw Error('Buffer must expand body only');
        }
      }
      window.setGodotHybridViewSettings({robot_overlay_body_buffer_mm:0});
      if(JSON.stringify([...p.robotMaskCapsules().starts])!==JSON.stringify([...fullMask.starts]))throw Error('Zero buffer changed existing mask');
      const missing=names.find(name=>!state.robotRenderer.endEffectorLinks.includes(name)),saved=state.robotMeshes[missing];
      delete state.robotMeshes[missing];
      if(p.robotMaskCapsules()!==null)throw Error('An unavailable hidden arm model must still prevent masking');
      state.robotMeshes[missing]=saved;
      gl.useProgram(state.robotProgram);draws=0;p.drawRobotShadow(state.robotProgram,state.robotUniforms.model);
      if(draws!==claw)throw Error('Hidden arm links must not cast model shadows');
      window.setGodotHybridViewSettings({robot_overlay_enabled:false});draws=0;p.drawRobotOverlay(p.identityMatrix());
      if(draws!==0)throw Error('Off mode still renders an overlay');
      if(render('full')!==all)throw Error('Switching back must restore the complete arm');
      const alphas=[],uniform4=gl.uniform4fv.bind(gl);
      gl.uniform4fv=(location,value)=>{if(location===state.robotUniforms.color)alphas.push(value[3]);return uniform4(location,value);};
      window.setGodotHybridViewSettings({robot_overlay_body_opacity:.25});
      if(render('full')!==all)throw Error('Ghost view lost links');
      if(!alphas.some(a=>Math.abs(a-.25)<1e-5)||!alphas.some(a=>a===1))throw Error('Ghost body and solid claw must use separate alpha values');
      if(!gl.getParameter(gl.DEPTH_WRITEMASK)||gl.isEnabled(gl.BLEND))throw Error('Ghost pass leaked render state');
      window.setGodotHybridViewSettings({robot_overlay_body_opacity:0});
      if(render('full')!==claw)throw Error('Zero body opacity must keep only the solid claw');
      if(!p.robotMaskCapsules())throw Error('Invisible body lost its full-arm point mask');
      window.setGodotHybridViewSettings({robot_overlay_body_opacity:1});
      window.setGodotHybridViewSettings({robot_overlay_scope:'invalid'});
      if(state.settings.robot_overlay_scope!=='full')throw Error('Invalid scope replaced a valid mode');
      if(JSON.stringify(state.robotTransforms)!==initial)throw Error('Rendering scope changed calibrated transforms');
      if(gl.getError()!==gl.NO_ERROR)throw Error('Overlay scope left a WebGL error');
      return {models:names.length,fullPrimitives:all,clawPrimitives:claw,clawLinks:state.robotRenderer.endEffectorLinks};
    });
    assert.equal(result.clawLinks.length,2);
    console.log(JSON.stringify(result));
  } finally {await browser.close();server.close();}
})().catch(error=>{console.error(error);server.close();process.exitCode=1;});
