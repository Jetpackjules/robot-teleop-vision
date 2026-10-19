// Actual production WebGL shaders, checked with synthetic asymmetric scenes.
import {MultiCameraFusion} from './hybrid_camera_fusion.js?v=20261020_native_quality';

function setup() {
  const {state,createProgram,findUniforms,identityMatrix}=window.probe;
  const canvas=document.getElementById('view');
  const gl=canvas.getContext('webgl2',{antialias:false,preserveDrawingBuffer:true});
  if(!gl)throw new Error('WebGL2 unavailable');
  state.blendPCR?.dispose();state.cameraFusion?.clear();
  for(const camera of state.cameras)window.probe.deleteCameraTextures(camera);
  if(state.program)gl.deleteProgram(state.program);
  Object.assign(state,{canvas,gl,running:true,cameras:[],blendPCR:null,program:createProgram(gl),cameraFusion:new MultiCameraFusion(gl),
    baseViewer:{transform:identityMatrix(),fov:55,near:0.01,far:10}});
  state.uniforms=findUniforms(gl,state.program);
  Object.assign(state.settings,{inspect_enabled:false,robot_overlay_enabled:false,workspace_surface_enabled:false,workspace_boundary_enabled:false,
    camera_locations_enabled:false,white_background_enabled:false,mesh_depth_delta:0.035,mesh_max_edge:0.08});
  gl.useProgram(state.program);
  for(const [name,unit] of [['depth',0],['color',1],['fusionDepth',2],['fusionColor',3],['rigDepth',4],['rigData',5],['rigQuality',6],['rigColor',7]])
    gl.uniform1i(state.uniforms[name],unit);
  gl.uniform1f(state.uniforms.maxPointSize,32);
  return {state,gl,canvas};
}

const palette=[[255,0,0],[0,255,0],[0,0,255],[255,255,0],[255,0,255],[0,255,255]];
function fixture(depths=[1000],sizes=depths.map(()=>64),custom=null) {
  const {state,uploadCamera,identityMatrix,deleteCameraTextures}=window.probe;
  for(const c of state.cameras)deleteCameraTextures(c);
  state.cameras=[];
  depths.forEach((z,i)=>{
    const [width,height]=Array.isArray(sizes[i])?sizes[i]:[sizes[i],sizes[i]];
    const pixels=new Uint8Array(width*height*3),depth=new Uint16Array(width*height);
    for(let y=0;y<height;y++)for(let x=0;x<width;x++){
      let value={z,color:palette[i%palette.length]};
      if(custom)value=custom(x,y,i,width,height,value);
      depth[y*width+x]=value.z;pixels.set(value.color,(y*width+x)*3);
    }
    state.cameras.push(uploadCamera({metadata:{id:String(i),model:'D435',priority:0,intrinsics:[width,width,(width-1)/2,(height-1)/2]},
      width,height,depth,colorPixels:pixels,modelMatrix:identityMatrix()}));
  });
  state.cameraFusion.clear();
}
function draw(mode,fresh=true) {
  const {state,render}=window.probe;
  if(fresh)for(const c of state.cameras)c.lastUpdateMs=performance.now();
  window.setGodotHybridViewSettings({display_mode:mode});render();
  if(state.stats.renderWarning)throw new Error(state.stats.renderWarning);
  const error=state.gl.getError();if(error)throw new Error('WebGL error '+error+' in '+mode);
}
function image() {
  const {gl,canvas}=window.probe.state;
  const pixels=new Uint8Array(canvas.width*canvas.height*4);
  gl.readPixels(0,0,canvas.width,canvas.height,gl.RGBA,gl.UNSIGNED_BYTE,pixels);
  return pixels;
}
function pixel() {
  const {gl,canvas}=window.probe.state,p=new Uint8Array(4);
  gl.readPixels(Math.floor(canvas.width/2)+1,Math.floor(canvas.height/2)+1,1,1,gl.RGBA,gl.UNSIGNED_BYTE,p);return p;
}
function snapshot(label) {
  const figure=document.createElement('figure'),caption=document.createElement('figcaption'),img=document.createElement('img');
  caption.textContent=label;img.src=window.probe.state.canvas.toDataURL();figure.append(caption,img);document.getElementById('gallery').append(figure);
}

export async function runRenderingChecks() {
  const {state,gl,canvas}=setup();let checks=0;const passed=[];
  const check=(ok,label)=>{checks++;if(!ok)throw new Error(label);passed.push(label);};
  const expect=(color,label)=>check(color.every((v,i)=>Math.abs(pixel()[i]-v)<4),label+': '+pixel());
  document.getElementById('gallery').replaceChildren();
  const sourceDepth=new Uint16Array(25).fill(1000);sourceDepth[12]=0;
  const compressed=await new Response(new Blob([sourceDepth]).stream().pipeThrough(new CompressionStream('deflate'))).arrayBuffer();
  const rgbCanvas=document.createElement('canvas');rgbCanvas.width=5;rgbCanvas.height=5;
  const rgb=await new Promise(resolve=>rgbCanvas.toBlob(resolve,'image/jpeg'));
  const payload={color:new Uint8Array(await rgb.arrayBuffer()),depth:new Uint8Array(compressed),metadata:{id:'exact-depth',width:5,height:5,
    transform:window.probe.identityMatrix(),depth_encoding:'u16_mm_deflate',depth_exact:true}};
  let decoded=await window.probe.decodeCamera(payload);
  check(decoded.depth[12]===0,'Quality decoder retains measured depth holes');decoded.bitmap?.close();
  payload.metadata.depth_exact=false;decoded=await window.probe.decodeCamera(payload);
  check(decoded.depth[12]===1000,'Low latency retains its optional small-hole repair');decoded.bitmap?.close();
  state.socketOptions={exactDepth:true};decoded=await window.probe.decodeCamera(payload);
  check(decoded.depth[12]===0,'Quality remains exact on HTTPS and direct transports without temporal metadata');decoded.bitmap?.close();state.socketOptions=null;
  fixture([1000,1000,1000,1000],[64,64,64,64]);
  draw('independent_mesh');expect(palette[3],'independent keeps nearest surface');
  draw('unified_mesh');expect(palette[0],'stable near-equal ownership with four cameras');
  const depthTexture=state.cameraFusion.depth,rebuilds=state.cameraFusion.rebuilds;
  draw('unified_mesh');check(state.cameraFusion.depth===depthTexture&&state.cameraFusion.rebuilds===rebuilds,'unchanged capture reuses validation and textures');
  fixture([1000,1000,1000],[64,48,96]);draw('unified_mesh');expect(palette[2],'higher pixel density wins locally');
  draw('shader_mesh');expect(palette[2],'shared-color mesh uses the best local RGB view');
  fixture([1000,1000,850]);draw('unified_mesh');expect(palette[2],'distinct foreground remains visible');
  state.cameras[2].lastUpdateMs=performance.now()-2000;
  draw('unified_mesh',false);expect(palette[0],'stale foreground does not cover live geometry');
  draw('independent_mesh',false);expect(palette[1],'independent excludes stale foreground too');
  fixture([1000,1000,1000,1000,1000]);draw('blendpcr');
  check(state.stats.renderMethod==='blendpcr','five cameras use BlendPCR without truncation');
  expect(palette[0],'equal-quality views use stable ownership rather than tinting the whole image');
  snapshot('Five-camera BlendPCR');
  // Compare the actual two-pass GPU filter against all 49 samples of the
  // original square kernel, including border clamping and camera group two.
  const blend=state.blendPCR,sw=blend.smallWidth,sh=blend.smallHeight;
  blend.attach(blend.owner,null);
  const owners=new Uint32Array(sw*sh);
  gl.readPixels(0,0,sw,sh,gl.RED_INTEGER,gl.UNSIGNED_INT,owners);
  let filterErrors=0;
  for(let group=0;group<blend.groups;group++) {
    blend.attach(blend.weights,group);
    const weights=new Uint8Array(sw*sh*4);gl.readPixels(0,0,sw,sh,gl.RGBA,gl.UNSIGNED_BYTE,weights);
    for(let y=0;y<sh;y++)for(let x=0;x<sw;x++)for(let c=0;c<4;c++) {
      let count=0;
      for(let dy=-3;dy<=3;dy++)for(let dx=-3;dx<=3;dx++)
        if(owners[Math.max(0,Math.min(sh-1,y+dy))*sw+Math.max(0,Math.min(sw-1,x+dx))]===group*4+c+1)count++;
      if(Math.abs(Math.round(count*255/49)-weights[(y*sw+x)*4+c])>1)filterErrors++;
    }
  }
  gl.bindFramebuffer(gl.FRAMEBUFFER,null);
  check(filterErrors===0,'separable filter matches all 49 reference taps for every camera and border pixel');
  const beforeDepthEdit=state.cameraFusion.rebuilds;
  state.cameras[4].depth.fill(700);state.cameras[4].uploadRevision++;
  gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,state.cameras[4].depthTexture);
  gl.texSubImage2D(gl.TEXTURE_2D,0,0,0,64,64,gl.RED_INTEGER,gl.UNSIGNED_SHORT,state.cameras[4].depth);
  draw('blendpcr');expect(palette[4],'fifth-camera foreground keeps its own color and depth');
  check(state.cameraFusion.rebuilds===beforeDepthEdit+1,'in-place depth revision regenerates only the changed camera');
  fixture([1000,1000],[64,64],(x,y,i,w,h,value)=>({z:(i===0&&x>34)||(i===1&&x<26)?0:1000,color:palette[i]}));
  draw('blendpcr');const seam=image();let mixed=0;
  for(let i=0;i<seam.length;i+=4)if(seam[i]>20&&seam[i+1]>20&&seam[i+2]<10)mixed++;
  check(mixed>0,'overlapping view boundaries blend colors without blending depth');
  snapshot('Smoothed camera seam');

  // Rectangular non-power-of-two grid, holes, checkerboard colors, variable
  // depth and an offset/rotated camera expose primitive-ID and UV mistakes.
  fixture([1000],[[79,53]],(x,y,i,w,h,value)=>({z:(x>28&&x<32)||(y>15&&y<18&&x<20)?0:920+2*x+y,
    color:[(x*19)%256,(y*23)%256,((x+y)%2)*255]}));
  const m=state.cameras[0].modelMatrix,angle=0.19;
  m[0]=Math.cos(angle);m[2]=-Math.sin(angle);m[8]=Math.sin(angle);m[10]=Math.cos(angle);m[12]=0.09;
  draw('independent_mesh');const cached=image();snapshot('Native mesh reference');
  // Draw the original uncached production vertex path with identical uniforms.
  gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.useProgram(state.program);
  gl.uniform1i(state.uniforms.qualityEnabled,0);gl.uniform1i(state.uniforms.rigEnabled,0);
  gl.drawArrays(gl.TRIANGLES,0,(79-1)*(53-1)*6);
  const original=image();
  check(cached.every((v,i)=>v===original[i]),'cached validation is pixel-identical to original mesh rules');
  draw('blendpcr');const blended=image();snapshot('BlendPCR exact reconstruction');
  let maximumError=0,badPixels=0;
  for(let i=0;i<cached.length;i+=4){let error=0;for(let c=0;c<3;c++)error=Math.max(error,Math.abs(cached[i+c]-blended[i+c]));maximumError=Math.max(maximumError,error);if(error>3)badPixels++;}
  // Nearest sampling is discontinuous exactly at texel boundaries. Raster
  // interpolation and ray reconstruction may round a half-texel differently.
  // Verify every mismatch lies on such a boundary, and selects an adjacent
  // original texel, rather than tolerating arbitrary color/coverage errors.
  let nonBoundaryErrors=0;const mismatchDetails=[];
  if(badPixels) {
    const savedProgram=state.program,savedUniforms=state.uniforms;
    const uvProgram=window.probe.createProgram(gl,undefined,`#version 300 es
      precision highp float;in float v_valid;in vec2 v_source_pixel;out vec4 color;
      void main(){if(v_valid<0.9999)discard;vec2 uv=floor((v_source_pixel+vec2(.5))/vec2(79,53)*65535.);
        color=vec4(mod(uv.x,256.),floor(uv.x/256.),mod(uv.y,256.),floor(uv.y/256.))/255.;}`);
    state.program=uvProgram;state.uniforms=window.probe.findUniforms(gl,uvProgram);
    gl.useProgram(uvProgram);
    for(const [name,unit] of [['depth',0],['color',1],['fusionDepth',2],['fusionColor',3],['rigDepth',4],['rigData',5],['rigQuality',6],['rigColor',7]])gl.uniform1i(state.uniforms[name],unit);
    draw('independent_mesh');const uv=image();
    state.program=savedProgram;state.uniforms=savedUniforms;gl.deleteProgram(uvProgram);
    const texel=(x,y)=>[(x*19)%256,(y*23)%256,((x+y)%2)*255];
    for(let i=0;i<cached.length;i+=4) {
      if(![0,1,2].some(c=>Math.abs(cached[i+c]-blended[i+c])>3))continue;
      const x=(uv[i]+256*uv[i+1])/65535*79-.5,y=(uv[i+2]+256*uv[i+3])/65535*53-.5;
      const nearX=Math.abs(x+.5-Math.round(x+.5))<.002;
      const nearY=Math.abs(y+.5-Math.round(y+.5))<.002;
      const xs=nearX?[Math.floor(x),Math.ceil(x)]:[Math.round(x)];
      const ys=nearY?[Math.floor(y),Math.ceil(y)]:[Math.round(y)];
      const matches=rgb=>xs.some(tx=>ys.some(ty=>texel(tx,ty).every((c,j)=>Math.abs(c-rgb[j])<=3)));
      if(!(nearX||nearY)||!matches(cached.subarray(i,i+3))||!matches(blended.subarray(i,i+3))){nonBoundaryErrors++;if(mismatchDetails.length<3)mismatchDetails.push({x,y,reference:Array.from(cached.subarray(i,i+3)),blend:Array.from(blended.subarray(i,i+3)),uv:Array.from(uv.subarray(i,i+4))});}
    }
  }
  if(badPixels)draw('blendpcr');
  check(nonBoundaryErrors===0,'single-camera coverage and RGB match native mesh away from exact nearest-texel ties ('+badPixels+' ties, '+nonBoundaryErrors+' errors) '+JSON.stringify(mismatchDetails));
  const compositions=state.blendPCR.compositions;
  draw('blendpcr');
  check(state.blendPCR.compositions===compositions&&image().every((v,i)=>v===blended[i]),'unchanged input reuses exact full-resolution color and depth');
  state.baseViewer.transform[12]=.02;draw('blendpcr');
  check(state.blendPCR.compositions===compositions+1,'mouse/viewpoint motion invalidates the composite immediately');
  state.baseViewer.transform=window.probe.identityMatrix();

  fixture([1000],[[379,233]],(x,y,i,w,h,value)=>({z:980+Math.floor(x/12)+Math.floor(y/11),color:[x%256,y%256,(x+y)%256]}));
  draw('independent_mesh');const largeReference=image();draw('blendpcr');const largeBlend=image();
  check(largeReference.every((v,i)=>Math.abs(v-largeBlend[i])<=3),'primitive IDs remain exact beyond the 16-bit boundary');

  const {RIG_SAMPLING_SHADER,FULLSCREEN_VERTEX_SHADER}=await import('./hybrid_camera_quality.js?v=20261020_native_quality');
  const qualityProbe=(await import('./hybrid_camera_quality.js?v=20261020_native_quality')).compileGpuProgram(gl,FULLSCREEN_VERTEX_SHADER,
    `#version 300 es\nprecision highp float;precision highp int;${RIG_SAMPLING_SHADER}\nout vec4 color;void main(){color=rig_veto(0,vec3(0,0,-1))?vec4(1,0,0,1):vec4(0,1,0,1);}`);
  function vetoProbe(peerDepth) {
    fixture([1000,peerDepth],[64,128],(x,y,i,w,h,value)=>({z:i===0?1000+(x-32)*10:peerDepth,color:palette[i]}));
    draw('independent_mesh');gl.useProgram(qualityProbe);state.cameraFusion.bind(qualityProbe);
    gl.uniform1f(gl.getUniformLocation(qualityProbe,'u_fusion_depth_tolerance'),.035);
    gl.disable(gl.DEPTH_TEST);gl.drawArrays(gl.TRIANGLES,0,3);gl.enable(gl.DEPTH_TEST);
  }
  vetoProbe(1150);expect(palette[0],'coherent better-resolved background can veto an uncertain ramp in observed free space');
  vetoProbe(850);expect(palette[1],'a nearer peer is occlusion and never vetoes a surface behind it');
  vetoProbe(0);expect(palette[1],'missing peer depth never removes a surface');
  gl.deleteProgram(qualityProbe);

  // Read depth through a normal overlay: a plane just behind the measured
  // source must remain hidden; one just in front must cover it.
  fixture([1000]);draw('blendpcr');
  const {compileGpuProgram}=await import('./hybrid_camera_quality.js?v=20261020_native_quality');
  const overlay=compileGpuProgram(gl,`#version 300 es\nprecision highp float;uniform float z;void main(){vec2 p=vec2(float((gl_VertexID<<1)&2),float(gl_VertexID&2));gl_Position=vec4(p*2.-1.,z,1.);}`,
    '#version 300 es\nprecision highp float;out vec4 color;void main(){color=vec4(0,1,0,1);}');
  const p=window.probe.perspectiveMatrix(55,canvas.width/canvas.height,.01,10);
  gl.useProgram(overlay);gl.uniform1f(gl.getUniformLocation(overlay,'z'),-p[10]+p[14]/1.01);gl.drawArrays(gl.TRIANGLES,0,3);
  expect(palette[0],'overlay behind source is depth-occluded');
  gl.uniform1f(gl.getUniformLocation(overlay,'z'),-p[10]+p[14]/.99);gl.drawArrays(gl.TRIANGLES,0,3);
  expect(palette[1],'overlay in front remains visible');gl.deleteProgram(overlay);

  fixture([1000],[[5,64]],(x,y,i,w,h,value)=>({z:1000,color:[255,80,180]}));
  state.cameras[0].metadata.intrinsics=[200,200,2,31.5];state.cameras[0].intrinsicsArray=new Float32Array([200,200,2,31.5]);
  draw('independent_mesh');const thin=image();draw('blendpcr');const thinBlend=image();
  check(thin.every((v,i)=>Math.abs(v-thinBlend[i])<=2),'thin geometry retains full-resolution coverage');
  fixture([1000]);draw('independent_mesh');
  const mask={starts:new Float32Array(24),ends:new Float32Array(24)};
  mask.starts.set([.0183,.0207,-1,.133]);mask.ends.set([.0183,.0207,-1,.133]);
  gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.useProgram(state.program);
  gl.uniform1i(state.uniforms.robotMaskEnabled,1);
  gl.uniform4fv(state.uniforms.maskCapsuleA,mask.starts);gl.uniform4fv(state.uniforms.maskCapsuleB,mask.ends);
  gl.drawArrays(gl.TRIANGLES,0,63*63*6);const maskedReference=image();
  const view={width:canvas.width,height:canvas.height,now:performance.now(),viewer:state.currentViewer,
    inverseViewer:window.probe.inverseRigidTransform(state.currentViewer),viewProjection:state.viewProjection,
    projection:window.probe.perspectiveMatrix(55,canvas.width/canvas.height,.01,10)};
  state.blendPCR.render(state.cameraFusion,state.cameras,view,mask,state.settings);
  const maskedBlend=image();
  check(maskedReference.every((v,i)=>Math.abs(v-maskedBlend[i])<=3),'robot masking preserves the original per-triangle coverage');
  const maskedCompositions=state.blendPCR.compositions;
  state.blendPCR.render(state.cameraFusion,state.cameras,view,null,state.settings);
  check(state.blendPCR.compositions===maskedCompositions+1,'changing robot masks invalidates cached color and depth');
  fixture([1000,1000,1000]);
  canvas.style.width='303px';canvas.style.height='197px';draw('blendpcr');
  check(state.blendPCR.width===canvas.width&&state.blendPCR.height===canvas.height,'non-power-of-two resize keeps full canvas resolution');
  draw('independent_mesh');check(!state.blendPCR.metadata,'switching mode releases private targets');
  draw('blendpcr');check(!!state.blendPCR.metadata,'BlendPCR re-entry recreates private targets');
  state.blendPCR.failedSignature='not-current';
  state.blendPCR.clearTargets();state.blendPCR.failedSignature=`${canvas.width}:${canvas.height}:3`;
  window.setGodotHybridViewSettings({display_mode:'blendpcr'});window.probe.render();
  check(state.stats.renderWarning.includes('using independent')&&state.stats.renderMethod==='independent_mesh','unsupported target falls back visibly instead of a black frame');
  check(gl.getError()===gl.NO_ERROR,'fallback leaves valid WebGL state');
  state.blendPCR.dispose();state.blendPCR=null;
  state.cameraFusion.clear();
  state.cameraFusion.failedSignature=JSON.stringify(state.cameras.map(c=>[c.metadata.id,c.width,c.height]));
  window.setGodotHybridViewSettings({display_mode:'independent_mesh'});window.probe.render();
  expect(palette[2],'GPU cache failure preserves the original uncached rendering path');
  check(state.stats.renderWarning.includes('independent')&&gl.getError()===gl.NO_ERROR,'cache capacity fallback is explicit and leaves valid GL state');
  state.cameraFusion.clear();
  fixture([1000],[16],(x,y,i,w,h,value)=>({z:x===8&&y===8?1000:0,color:palette[0]}));
  canvas.style.width='256px';canvas.style.height='256px';draw('point_cloud');
  const coverage=()=>{const values=image();let count=0;for(let i=0;i<values.length;i+=4)if(values[i]>200&&values[i+1]<10)count++;return count;};
  const farCoverage=coverage();state.baseViewer.transform[14]=-.4;draw('point_cloud');
  check(farCoverage>0&&coverage()>farCoverage*1.2,'point splats scale with viewer distance instead of leaving fixed-size holes');
  state.baseViewer.transform=window.probe.identityMatrix();
  canvas.style.width='256px';canvas.style.height='256px';
  fixture([1000,1000,1000]);draw('blendpcr');
  const extension=gl.getExtension('WEBGL_debug_renderer_info');
  return {checks,passed,renderer:extension?gl.getParameter(extension.UNMASKED_RENDERER_WEBGL):gl.getParameter(gl.RENDERER),
    canvas:[canvas.width,canvas.height],maximumSingleCameraColorError:maximumError};
}

export async function benchmark() {
  const {state,gl,canvas}=setup();
  canvas.style.width='960px';canvas.style.height='600px';fixture([1000,1000,1000],[320,320,320]);
  const results={sensorGrid:[320,320],canvas:null};
  for(const mode of ['independent_mesh','unified_mesh','blendpcr']) {
    draw(mode);gl.finish();let started=performance.now();
    for(let i=0;i<12;i++){draw(mode);gl.finish();}
    results[mode]={unchangedGpuAndSubmitMs:(performance.now()-started)/12};
    started=performance.now();
    for(let i=0;i<12;i++){state.baseViewer.transform[12]=i*.002;draw(mode);gl.finish();}
    results[mode].movingViewGpuAndSubmitMs=(performance.now()-started)/12;
    state.baseViewer.transform=window.probe.identityMatrix();
    started=performance.now();
    for(let i=0;i<12;i++){for(const c of state.cameras)c.uploadRevision++;draw(mode);gl.finish();}
    results[mode].newCaptureGpuAndSubmitMs=(performance.now()-started)/12;
    await new Promise(resolve=>requestAnimationFrame(resolve));
  }
  results.canvas=[canvas.width,canvas.height];return results;
}
