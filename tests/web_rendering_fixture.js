// Actual production WebGL shaders, checked with synthetic asymmetric scenes.
import {MultiCameraFusion} from './hybrid_camera_fusion.js?v=20261020_capture60';

export function setup() {
  const {state,createProgram,findUniforms,identityMatrix}=window.probe;
  const canvas=document.getElementById('view');
  const gl=canvas.getContext('webgl2',{antialias:false,preserveDrawingBuffer:true});
  if(!gl)throw new Error('WebGL2 unavailable');
  state.blendPCR?.dispose();state.cameraFusion?.clear();
  for(const camera of state.cameras)window.probe.deleteCameraTextures(camera);
  if(state.program)gl.deleteProgram(state.program);
  if(state.indexedProgram)gl.deleteProgram(state.indexedProgram);
  state.indexedProgram=state.indexedUniforms=null;state.indexedUnavailable=false;
  for(const key of ['original','originalIndexed','originalPoints']) {
    if(state[key+'Program'])gl.deleteProgram(state[key+'Program']);state[key+'Program']=state[key+'Uniforms']=null;state[key+'Unavailable']=false;
  }
  Object.assign(state,{canvas,gl,running:true,cameras:[],blendPCR:null,program:createProgram(gl),cameraFusion:new MultiCameraFusion(gl),
    baseViewer:{transform:identityMatrix(),fov:55,near:0.01,far:10}});
  state.uniforms=findUniforms(gl,state.program);
  Object.assign(state.settings,{inspect_enabled:false,robot_overlay_enabled:false,workspace_surface_enabled:false,workspace_boundary_enabled:false,
    modeled_workspace_enabled:false,
    workspace_crop_enabled:false,workspace_crop_show_box:false,workspace_crop_rotation:[0,0,0],
    camera_locations_enabled:false,white_background_enabled:false,depth_only_enabled:false,mesh_depth_delta:0.035,mesh_max_edge:0.08});
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
export function draw(mode,fresh=true) {
  const {state,render}=window.probe;
  if(fresh)for(const c of state.cameras)c.lastUpdateMs=performance.now();
  window.setGodotHybridViewSettings({display_mode:mode});render();
  if(state.stats.renderWarning)throw new Error(state.stats.renderWarning);
  const error=state.gl.getError();if(error)throw new Error('WebGL error '+error+' in '+mode);
}
export function image() {
  const {gl,canvas}=window.probe.state;
  const pixels=new Uint8Array(canvas.width*canvas.height*4);
  gl.readPixels(0,0,canvas.width,canvas.height,gl.RGBA,gl.UNSIGNED_BYTE,pixels);
  return pixels;
}
function pixel() {
  const {gl,canvas}=window.probe.state,p=new Uint8Array(4);
  gl.readPixels(Math.floor(canvas.width/2)+1,Math.floor(canvas.height/2)+1,1,1,gl.RGBA,gl.UNSIGNED_BYTE,p);return p;
}
export function snapshot(label) {
  const figure=document.createElement('figure'),caption=document.createElement('figcaption'),img=document.createElement('img');
  caption.textContent=label;img.src=window.probe.state.canvas.toDataURL();figure.append(caption,img);document.getElementById('gallery').append(figure);
}

export async function runRenderingChecks() {
  const {state,gl,canvas}=setup();let checks=0;const passed=[];
  const check=(ok,label)=>{window.testProgress=label;checks++;if(!ok)throw new Error(label);passed.push(label);};
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
  const qualityAt=(camera,x,y)=>{
    const previous=gl.getParameter(gl.FRAMEBUFFER_BINDING),target=gl.createFramebuffer(),value=new Uint8Array(4);
    gl.bindFramebuffer(gl.FRAMEBUFFER,target);
    gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,state.cameraFusion.quality,0,camera);
    gl.readPixels(x,y,1,1,gl.RGBA,gl.UNSIGNED_BYTE,value);
    gl.bindFramebuffer(gl.FRAMEBUFFER,previous);gl.deleteFramebuffer(target);return value;
  };
  fixture([1000],[64],(x,y,i,w,h,value)=>({...value,z:x<32?1000:1030}));
  state.cameras[0].intrinsicsArray=new Float32Array([600,600,31.5,31.5]);
  state.settings.mesh_depth_delta=.05;draw('independent_mesh');
  check(qualityAt(0,31,31)[0]===0&&qualityAt(0,31,31)[1]===0,'3 cm foreground/background step is never triangulated into a ramp at 1 m');
  check(qualityAt(0,12,31)[0]===255&&qualityAt(0,12,31)[1]===255,'foreground interior remains full resolution beside a rejected edge');
  fixture([1000],[64],(x,y,i,w,h,value)=>({...value,z:900+3*x+2*y}));
  state.cameras[0].intrinsicsArray=new Float32Array([600,600,31.5,31.5]);draw('independent_mesh');
  check(qualityAt(0,31,31)[0]===255&&qualityAt(0,31,31)[1]===255,'continuous oblique surface survives the range and pixel footprint guard');
  fixture([1000,1000,1000],[64,64,64],(x,y,i,w,h,value)=>({...value,z:x>=29&&x<=35&&y>=29&&y<=35?0:1000}));
  draw('blendpcr');
  check([0,1,2].every(i=>qualityAt(i,31,31).every(v=>v===0)),'all-camera blind spot remains invalid in every geometry cache');
  expect(Array.from(gl.getParameter(gl.COLOR_CLEAR_VALUE)).slice(0,3).map(v=>Math.round(v*255)),
    'BlendPCR leaves an all-camera blind spot empty');
  state.settings.mesh_depth_delta=.035;
  // Compare optimized submission with an identical world crop rendered from
  // the full source grid. This catches missing boundary triangles, wrong
  // primitive IDs, source-row offsets, and stale BlendPCR composite caches.
  fixture([1000,1000,1000],[128,128,128],(x,y,i,w,h,value)=>({...value,
    z:x>90&&y>90?0:950+Math.floor(x/12),color:[120+x%80,80+y%80,60+i*40]}));
  const cropSettings={workspace_crop_enabled:true,workspace_crop_min:[-.14,-.16,-1.05],workspace_crop_max:[.15,.13,-.85]};
  for(const mode of ['point_cloud','shader_mesh','independent_mesh','unified_mesh','blendpcr']) {
    Object.assign(state.settings,{workspace_crop_enabled:false,depth_only_enabled:false});
    draw(mode);const original=image();
    Object.assign(state.settings,cropSettings);
    state.useWorkspaceCulling=false;if(state.blendPCR)state.blendPCR.useWorkspaceCulling=false;
    draw(mode);if(state.blendPCR)state.blendPCR.useWorkspaceCulling=false;draw(mode);
    const reference=image();
    state.useWorkspaceCulling=true;if(state.blendPCR)state.blendPCR.useWorkspaceCulling=true;
    draw(mode);const clipped=image();
    check(reference.every((v,i)=>v===clipped[i]),`${mode}: early crop matches exact full-grid crop at every pixel`);
    for(const rotation of [[0,37,0],[18,-29,12]]) {
      state.settings.workspace_crop_rotation=rotation;state.useWorkspaceCulling=false;if(state.blendPCR)state.blendPCR.useWorkspaceCulling=false;draw(mode);const rotatedReference=image();
      state.useWorkspaceCulling=true;if(state.blendPCR)state.blendPCR.useWorkspaceCulling=true;draw(mode);const rotatedCrop=image();
      check(rotatedReference.every((v,i)=>v===rotatedCrop[i]),`${mode}: rotated crop preserves every interior pixel at ${rotation}`);
    }
    state.settings.workspace_crop_rotation=[0,0,0];draw(mode);

    let removed=0,kept=0;
    for(let i=0;i<original.length;i+=4) {
      const bg=[8,9,10],outside=bg.every((v,c)=>clipped[i+c]===v),wasBg=bg.every((v,c)=>original[i+c]===v);
      if(!wasBg&&outside)removed++;if(!outside)kept++;
    }
    check(removed>1000&&kept>100,`${mode}: workspace crop removes surroundings and retains its interior`);
    check(state.stats.workspaceCrop.submittedCells<state.stats.workspaceCrop.sourceCells*.7,
      `${mode}: outside source geometry is skipped before shading`);
    Object.assign(state.settings,{workspace_crop_enabled:false});draw(mode);const restored=image();
    check(original.every((v,i)=>v===restored[i]),`${mode}: disabling the crop restores the original full-detail image`);
  }
  state.useWorkspaceCulling=true;
  // Changing the material must not change geometry, depth occlusion or RGB
  // restoration, including BlendPCR's cached composite and sparse points.
  fixture([1000,1000,1000],[64,64,64],(x,y,i,w,h,value)=>({
    ...value,z:x>27&&x<34&&y>22&&y<40?0:920+x+y}));
  const occluder=window.probe.createProgram(gl,`#version 300 es
    void main(){vec2 p=vec2(float((gl_VertexID<<1)&2),float(gl_VertexID&2));gl_Position=vec4(p*2.-1.,0,1);}`,
    `#version 300 es
    precision highp float;out vec4 color;
    void main(){color=vec4(1,0,1,1);gl_FragDepth=0.97+gl_FragCoord.x/float(${canvas.width})*0.03;}`);
  const occlusion=()=>{
    gl.useProgram(occluder);gl.bindVertexArray(null);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);
    gl.drawArrays(gl.TRIANGLES,0,3);
    const pixels=image(),mask=[];
    for(let i=0;i<pixels.length;i+=4)mask.push(pixels[i]===255&&pixels[i+1]===0&&pixels[i+2]===255);
    return mask;
  };
  const background=[8,9,10],isBackground=(pixels,i)=>background.every((v,c)=>pixels[i+c]===v);
  for(const mode of ['point_cloud','shader_mesh','independent_mesh','unified_mesh','blendpcr']) {
    state.settings.depth_only_enabled=false;draw(mode);const textured=image(),depthMask=occlusion();
    const rebuilds=state.cameraFusion.rebuilds,metadataPasses=state.blendPCR?.metadataPasses;
    window.setGodotHybridViewSettings({depth_only_enabled:true});draw(mode);const gray=image();
    let grayPixels=0,coverageErrors=0,colorErrors=0;
    for(let i=0;i<gray.length;i+=4) {
      if(isBackground(textured,i)!==isBackground(gray,i))coverageErrors++;
      if(!isBackground(gray,i)) {grayPixels++;if(gray[i]!==gray[i+1]||gray[i]!==gray[i+2])colorErrors++;}
    }
    check(grayPixels>100&&colorErrors===0,'depth-only has visible neutral gray geometry in '+mode);
    check(coverageErrors===0,'depth-only preserves every covered pixel and depth hole in '+mode);
    check(occlusion().every((v,i)=>v===depthMask[i]),'depth-only preserves exact depth occlusion in '+mode);
    check(state.cameraFusion.rebuilds===rebuilds,'material toggle reuses the measured geometry in '+mode);
    if(mode==='blendpcr')check(state.blendPCR.metadataPasses===metadataPasses,'BlendPCR changes its material without rasterizing the cameras again');
    window.setGodotHybridViewSettings({depth_only_enabled:false});draw(mode);
    check(image().every((v,i)=>v===textured[i]),'disabling depth-only restores identical RGB output in '+mode);
  }
  snapshot('Depth-only material checks passed');
  gl.deleteProgram(occluder);
  fixture([1000,1000,1040],[[79,53],[79,53],[65,47]],(x,y,i,w,h,value)=>({
    z:(x>25&&x<29)||(y>17&&y<20&&x<20)?0:value.z+2*x+y,
    color:[(x*19+i*31)%256,(y*23)%256,((x+y)%2)*255]}));
  for(const mode of ['independent_mesh'])for(const gray of [false,true]) {
    state.settings.depth_only_enabled=gray;state.useIndexedMeshes=false;draw(mode);const reference=image();
    state.useIndexedMeshes=true;draw(mode);
    check(image().every((v,i)=>v===reference[i]),'indexed geometry preserves exact RGB, gray, holes and ownership in '+mode+' / '+gray);
  }
  state.settings.depth_only_enabled=false;
  fixture([1000],[64]);
  const slots=new Set(),native=state.cameras[0];
  for(let frame=0;frame<7;frame++) {
    const color=palette[frame%palette.length],pixels=new Uint8Array(64*64*3);
    for(let i=0;i<64*64;i++)pixels.set(color,i*3);
    window.probe.uploadCamera({metadata:native.metadata,width:64,height:64,
      modelMatrix:native.modelMatrix,depth:new Uint16Array(64*64).fill(900+frame),colorPixels:pixels});
    slots.add(native.depthTexture);draw('independent_mesh');expect(color,'texture ring presents the newest complete frame '+frame);
  }
  check(slots.size===3&&native.textureSlots.length===3,'complete frames reuse three bounded texture slots');
  const patch=new Uint8Array(8*8*3);
  for(let i=0;i<64;i++)patch.set([255,0,255],i*3);
  const activeDepthTexture=native.depthTexture,activeColorTexture=native.colorTexture;
  window.probe.uploadCamera({metadata:native.metadata,width:64,height:64,modelMatrix:native.modelMatrix,
    depth:native.depth,colorTileUpdates:[{x:28,y:28,width:8,height:8,pixels:patch}]});
  draw('independent_mesh');expect([255,0,255],'partial RGB updates keep the current texture reference');
  check(native.depthTexture===activeDepthTexture&&native.colorTexture===activeColorTexture,'partial updates never rotate into an old full-frame slot');
  const colorReadTarget=gl.createFramebuffer(),outsidePatch=new Uint8Array(4);
  gl.bindFramebuffer(gl.FRAMEBUFFER,colorReadTarget);
  gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,native.colorTexture,0);
  gl.readPixels(8,8,1,1,gl.RGBA,gl.UNSIGNED_BYTE,outsidePatch);
  gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.deleteFramebuffer(colorReadTarget);
  check(outsidePatch[0]===255&&outsidePatch[1]===0&&outsidePatch[2]===0,'partial RGB updates preserve pixels outside the patch');
  const readTarget=gl.createFramebuffer(),rawPixel=new Uint32Array(4);
  gl.bindFramebuffer(gl.FRAMEBUFFER,readTarget);
  gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,state.cameraFusion.depth,0,0);
  gl.readPixels(32,32,1,1,gl.RGBA_INTEGER,gl.UNSIGNED_INT,rawPixel);
  gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.deleteFramebuffer(readTarget);
  check(rawPixel[0]===906,'GPU depth copy retains exact native integer millimeters');
  const oldSlots=[...native.textureSlots];
  window.probe.uploadCamera({metadata:{...native.metadata,intrinsics:[32,32,15.5,15.5]},width:32,height:32,
    modelMatrix:native.modelMatrix,depth:new Uint16Array(32*32).fill(1000),colorPixels:new Uint8Array(32*32*3)});
  check(oldSlots.every(slot=>!gl.isTexture(slot.depthTexture)&&!gl.isTexture(slot.colorTexture)),
    'profile resolution changes release every old texture slot');
  fixture(Array(9).fill(1000),[32,32,32,32,32,32,32,32,64]);
  draw('unified_mesh');expect(palette[2],'camera nine retains geometry ownership');
  draw('shader_mesh');expect(palette[2],'camera nine retains RGB selection');
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

  // Calibrated RGB coverage is independent of depth ownership. An 8 mm
  // planar stereo mismatch must not paint a gray camera-shaped wedge.
  const outsideColor={model:'pinhole',size:[64,64],intrinsics:[64,64,-640,32],extrinsics:[1,0,0,0,1,0,0,0,1,0,0,0]};
  for(const mode of ['point_cloud','shader_mesh','independent_mesh','unified_mesh','blendpcr']) {
    fixture([1000,1008]);state.cameras[0].metadata.color_projection=outsideColor;
    draw(mode);expect([0,255,0],'missing RGB borrows the agreeing planar camera in '+mode);
    fixture([1000,1008],[64,64],(x,y,i,w,h,value)=>({z:value.z,color:i===0?[0,0,0]:value.color}));
    draw(mode);expect([0,0,0],'a genuinely black observed surface remains black in '+mode);
    fixture([1000,1008],[64,64],(x,y,i,w,h,value)=>({z:i===1?1008+(x-31.5)*10:value.z,color:value.color}));
    state.cameras[0].metadata.color_projection=outsideColor;
    draw(mode);const edge=pixel();check(edge[0]===edge[1]&&edge[1]===edge[2], 'RGB fallback never crosses a different surface angle in '+mode);
  }

  // Rectangular non-power-of-two grid, holes, checkerboard colors, variable
  // depth and an offset/rotated camera expose primitive-ID and UV mistakes.
  fixture([1000],[[79,53]],(x,y,i,w,h,value)=>({z:(x>28&&x<32)||(y>15&&y<18&&x<20)?0:920+2*x+y,
    color:[(x*19)%256,(y*23)%256,((x+y)%2)*255]}));
  const m=state.cameras[0].modelMatrix,angle=0.19;
  m[0]=Math.cos(angle);m[2]=-Math.sin(angle);m[8]=Math.sin(angle);m[10]=Math.cos(angle);m[12]=0.09;
  draw('independent_mesh');const cached=image();snapshot('Native mesh reference');
  // Draw the original uncached production vertex path with identical uniforms.
  state.useIndexedMeshes=false;draw('independent_mesh');state.useIndexedMeshes=true;
  gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.useProgram(state.program);
  gl.uniform1i(state.uniforms.qualityEnabled,0);gl.uniform1i(state.uniforms.rigEnabled,0);
  gl.uniform1i(state.uniforms.sparseTiles,0);gl.uniform2i(state.uniforms.sourceOrigin,0,0);gl.uniform2i(state.uniforms.sourceSize,79,53);
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

  fixture([1000,1000,1000]);draw('blendpcr');
  const passes=state.blendPCR.metadataPasses;
  state.cameras[0].uploadRevision++;draw('blendpcr');
  const partial=image();
  check(state.blendPCR.metadataPasses===passes+1,'a partial camera update rasterizes only its own full-resolution layer');
  state.blendPCR.metadataLayerKeys=[];state.blendPCR.cacheKey='';draw('blendpcr');
  check(image().every((v,i)=>v===partial[i]),'partial-layer reuse is pixel-identical to rebuilding every camera');
  const viewPasses=state.blendPCR.metadataPasses;
  state.baseViewer.transform[12]=.02;draw('blendpcr');
  check(state.blendPCR.metadataPasses===viewPasses+3,'viewpoint changes invalidate every camera layer');
  state.baseViewer.transform=window.probe.identityMatrix();

  // Smooth gradients keep nearest-texel ties at wrap boundaries from looking
  // like primitive-ID failures; a wrong large ID still changes position/color.
  fixture([1000],[[379,233]],(x,y,i,w,h,value)=>({z:980+Math.floor(x/12)+Math.floor(y/11),color:[Math.floor(x*255/(w-1)),Math.floor(y*255/(h-1)),128]}));
  draw('independent_mesh');const largeReference=image();draw('blendpcr');const largeBlend=image();
  const largeErrors=[];
  for(let i=0;i<largeReference.length;i+=4)if([0,1,2,3].some(c=>Math.abs(largeReference[i+c]-largeBlend[i+c])>3))
    largeErrors.push({pixel:i/4,reference:Array.from(largeReference.subarray(i,i+4)),blend:Array.from(largeBlend.subarray(i,i+4))});
  check(largeErrors.length===0,'primitive IDs remain exact beyond the 16-bit boundary: '+JSON.stringify({count:largeErrors.length,first:largeErrors.slice(0,5)}));
  const instancedBlend=state.blendPCR;
  instancedBlend.instancedMetadata=false;instancedBlend.cacheKey='';instancedBlend.metadataLayerKeys=[];
  draw('blendpcr');
  check(image().every((v,i)=>v===largeBlend[i]),'reusable 16-bit tiles preserve exact output and global primitive IDs');
  instancedBlend.instancedMetadata=true;instancedBlend.cacheKey='';instancedBlend.metadataLayerKeys=[];

  const {RIG_SAMPLING_SHADER,FULLSCREEN_VERTEX_SHADER}=await import('./hybrid_camera_quality.js?v=20261020_capture60');
  const qualityProbe=(await import('./hybrid_camera_quality.js?v=20261020_capture60')).compileGpuProgram(gl,FULLSCREEN_VERTEX_SHADER,
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
  const {compileGpuProgram}=await import('./hybrid_camera_quality.js?v=20261020_capture60');
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
  state.useIndexedMeshes=false;draw('independent_mesh');state.useIndexedMeshes=true;
  const mask={starts:new Float32Array(24),ends:new Float32Array(24)};
  mask.starts.set([.0183,.0207,-1,.133]);mask.ends.set([.0183,.0207,-1,.133]);
  gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.useProgram(state.program);
  gl.uniform1i(state.uniforms.robotMaskEnabled,1);
  gl.uniform1i(state.uniforms.sparseTiles,0);gl.uniform2i(state.uniforms.sourceOrigin,0,0);gl.uniform2i(state.uniforms.sourceSize,64,64);
  gl.uniform4fv(state.uniforms.maskCapsuleA,mask.starts);gl.uniform4fv(state.uniforms.maskCapsuleB,mask.ends);
  gl.drawArrays(gl.TRIANGLES,0,63*63*6);const maskedReference=image();
  const view={width:canvas.width,height:canvas.height,now:performance.now(),viewer:state.currentViewer,
    inverseViewer:window.probe.inverseRigidTransform(state.currentViewer),viewProjection:state.viewProjection,
    projection:window.probe.perspectiveMatrix(55,canvas.width/canvas.height,.01,10)};
  state.blendPCR.render(state.cameraFusion,state.cameras,view,mask,state.settings);
  const maskedBlend=image();
  const maskErrors=[];
  for(let i=0;i<maskedReference.length;i+=4)if([0,1,2,3].some(c=>Math.abs(maskedReference[i+c]-maskedBlend[i+c])>3))
    maskErrors.push({pixel:i/4,reference:Array.from(maskedReference.subarray(i,i+4)),blend:Array.from(maskedBlend.subarray(i,i+4))});
  check(maskErrors.length===0,'robot masking preserves the original per-triangle coverage: '+JSON.stringify({count:maskErrors.length,first:maskErrors.slice(0,5)}));
  fixture([1000],[[128,128]]);draw('blendpcr');
  Object.assign(state.settings,{workspace_crop_enabled:true,workspace_crop_min:[-.12,-.15,-1.01],workspace_crop_max:[.12,.15,-.99]});
  const maskedView={...view,now:performance.now()};
  state.blendPCR.indexedMetadata=false;
  state.blendPCR.useWorkspaceCulling=false;
  state.blendPCR.render(state.cameraFusion,state.cameras,maskedView,mask,state.settings);
  const maskedCropReference=image();
  state.blendPCR.indexedMetadata=true;
  state.blendPCR.useWorkspaceCulling=true;
  state.blendPCR.render(state.cameraFusion,state.cameras,maskedView,mask,state.settings);
  const maskedCrop=image();
  check(maskedCrop.every((v,i)=>Math.abs(v-maskedCropReference[i])<=3),'tiled arm-masked crop preserves original full-grid pixels');
  check(state.blendPCR.cropStats.submittedCells<state.blendPCR.cropStats.sourceCells*.7,'arm masking retains workspace tile culling instead of submitting the full grid');
  state.settings.workspace_crop_enabled=false;
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
  state.cameraFusion.failedSignature=JSON.stringify(state.cameras.map(c=>[c.metadata.id,c.width,c.height,c.colorWidth,c.colorHeight,c.metadata.color_space]));
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
  if(new URLSearchParams(location.search).has("rawperf"))return benchmarkOriginalRgb();
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

export async function runModeledWorkspaceChecks() {
  const {state,gl,canvas}=setup();let checks=0;const passed=[];
  const check=(ok,label)=>{checks++;if(!ok)throw new Error(label);passed.push(label);};
  const redCount=()=>{const pixels=image();let count=0;for(let i=0;i<pixels.length;i+=4)if(pixels[i]>200&&pixels[i+1]<30&&pixels[i+2]<30)count++;return count;};
  fixture([1000,1000],[128,128],()=>({z:1000,color:[255,0,0]}));
  const original=state.cameras.map(c=>({depth:c.depth.slice(),matrix:c.modelMatrix.slice()}));
  const settings={modeled_workspace_enabled:true,modeled_workspace_center:[0,-.18,-1],
    modeled_workspace_width:.38,modeled_workspace_depth:.35,modeled_workspace_clearance:.006};
  const submissions={};
  for(const mode of ['point_cloud','independent_mesh','unified_mesh','blendpcr']) {
    window.setGodotHybridViewSettings({modeled_workspace_enabled:false});draw(mode);
    const reference=image(),fullCount=redCount();
    window.setGodotHybridViewSettings(settings);draw(mode);
    check(redCount()>0&&redCount()<fullCount*.65,mode+': object samples remain while outside/table samples disappear');
    check(pixel()[0]>240&&pixel()[1]<10,mode+': foreground object occludes the artificial environment');
    check(state.stats.workspaceCrop.submittedCells<state.stats.workspaceCrop.sourceCells*.85,mode+': bounded source submission skips geometry');
    submissions[mode]={...state.stats.workspaceCrop};
    if(mode==='blendpcr')snapshot('Modeled table + live measured surface');
    window.setGodotHybridViewSettings({modeled_workspace_enabled:false});draw(mode);
    const restored=image();
    check(reference.every((v,i)=>v===restored[i]),mode+': switching off restores the exact original frame');
  }
  window.setGodotHybridViewSettings({...settings,modeled_workspace_center:[NaN,0,0]});draw('blendpcr');
  const invalid=image();window.setGodotHybridViewSettings({modeled_workspace_enabled:false});draw('blendpcr');
  const restored=image();
  check(invalid.every((v,i)=>v===restored[i]),'invalid placement safely preserves the full scene');
  check(state.cameras.every((c,i)=>c.depth.every((v,j)=>v===original[i].depth[j])&&c.modelMatrix.every((v,j)=>v===original[i].matrix[j])),
    'modeled scene never alters depth measurements or camera calibration');
  window.setGodotHybridViewSettings({...settings,modeled_workspace_center:[0,.8,-1]});draw('blendpcr');
  check(redCount()===0,'samples below the tabletop exclusion are removed');
  check(gl.getError()===gl.NO_ERROR,'stage rendering preserves valid WebGL state');
  snapshot('Higher tabletop excludes the measured surface');
  window.setGodotHybridViewSettings(settings);draw('blendpcr');
  return {checks,passed,submissions};
}

export async function runOriginalRgbChecks() {
  const {state,gl,canvas}=setup();const checks=[];
  const check=(ok,label)=>{if(!ok)throw new Error(label);checks.push(label);};
  canvas.style.width='256px';canvas.style.height='256px';
  const image=()=>{const p=new Uint8Array(canvas.width*canvas.height*4);gl.readPixels(0,0,canvas.width,canvas.height,gl.RGBA,gl.UNSIGNED_BYTE,p);return p;};
  const center=()=>{const p=image(),i=(Math.floor(canvas.height/2)*canvas.width+Math.floor(canvas.width/2))*4;return [...p.slice(i,i+3)];};
  const close=(a,b)=>a.every((v,i)=>Math.abs(v-b[i])<=3);
  function scene(specs){
    for(const c of state.cameras)window.probe.deleteCameraTextures(c);state.cameras=[];state.cameraFusion.clear();
    for(let i=0;i<specs.length;i++){
      const spec=specs[i],width=64,height=64,cw=spec.aligned?64:(spec.cw||256),ch=spec.aligned?64:(spec.ch||128);
      const pixels=new Uint8Array(cw*ch*3),depth=new Uint16Array(width*height);
      for(let y=0;y<height;y++)for(let x=0;x<width;x++)depth[y*width+x]=typeof spec.depth==='function'?spec.depth(x,y):(spec.depth??1000);
      for(let y=0;y<ch;y++)for(let x=0;x<cw;x++)pixels.set(typeof spec.rgb==='function'?spec.rgb(x,y):spec.rgb,(y*cw+x)*3);
      const transform=window.probe.identityMatrix();transform[12]=spec.x||0;
      const projection={model:'pinhole',size:[cw,ch],intrinsics:spec.k||[128,128,(cw-1)/2,(ch-1)/2],extrinsics:spec.extrinsics||[1,0,0,0,1,0,0,0,1,spec.tx||0,0,0]};
      state.cameras.push(window.probe.uploadCamera({metadata:{id:String(i),model:'D435',intrinsics:[64,64,31.5,31.5],color_space:spec.aligned?'aligned_depth':'original_rgb',color_width:cw,color_height:ch,color_projection:projection},width,height,depth,colorPixels:pixels,modelMatrix:transform}));
    }
  }
  const modes=['point_cloud','shader_mesh','independent_mesh','unified_mesh','blendpcr'];
  for(const mode of modes){
    scene([{rgb:[255,0,0],tx:3},{depth:0,rgb:[0,0,255]}]);draw(mode);
    check(close(center(),[0,0,255]),`${mode}: RGB donor with no depth samples colors the independently measured surface`);
    const passes=state.cameraFusion.visibility.passes;draw(mode);
    check(state.cameraFusion.visibility.passes===passes,`${mode}: unchanged geometry reuses RGB visibility when orbit renders again`);
    scene([{rgb:[255,0,0],tx:3},{depth:0,rgb:[0,0,0]}]);draw(mode);
    check(close(center(),[0,0,0]),`${mode}: genuinely black RGB stays black`);
    scene([{rgb:[255,0,0],tx:3},{depth:0,rgb:[0,0,255],tx:3}]);draw(mode);
    check(close(center(),[87,87,87]),`${mode}: no RGB coverage retains neutral geometry`);
    scene([{rgb:[255,0,0],tx:3},{depth:0,rgb:[0,0,255],x:.2},
      {depth:(x,y)=>x>=18&&x<=21&&y>=27&&y<=36?500:0,rgb:[0,255,0],tx:3,x:.2}]);draw(mode);
    check(close(center(),[87,87,87]),`${mode}: a third camera's geometry blocks an occluded RGB donor`);
    scene([{rgb:[255,0,0],tx:3},{depth:0,rgb:[0,0,255],x:.2}]);draw(mode);
    check(close(center(),[0,0,255]),`${mode}: removing the occluder restores color without changing the receiving geometry`);
    state.cameras[0].lastUpdateMs=performance.now();state.cameras[1].lastUpdateMs=performance.now()-2000;draw(mode,false);
    check(close(center(),[87,87,87]),`${mode}: stale RGB cameras are excluded (${center()})`);
    check(gl.getError()===gl.NO_ERROR,`${mode}: original RGB rendering leaves valid GL state`);
  }
  for(const mode of modes){
    scene([{rgb:x=>x>=128?[0,0,255]:[255,0,0],tx:.2}]);draw(mode);
    check(close(center(),[0,0,255]),`${mode}: color uses calibrated RGB optical translation`);
    scene([{rgb:(x,y)=>y>=64?[0,0,255]:[255,0,0],extrinsics:[0,1,0,-1,0,0,0,0,1,0,0,0]}]);
    state.baseViewer.transform[12]=.25;draw(mode);
    check(close(center(),[0,0,255]),`${mode}: color uses calibrated optical rotation and image axes`);
    state.baseViewer.transform=window.probe.identityMatrix();
  }
  scene([{aligned:true,rgb:[255,0,0]},{rgb:[0,0,255],tx:3}]);draw('blendpcr');
  check(close(center(),[255,0,0]),'mixed original and legacy aligned packets retain valid legacy colors');
  scene([{rgb:(x,y)=>x%4<2?[255,255,255]:[0,0,0],cw:256,ch:256,k:[256,256,127.5,127.5]}]);draw('blendpcr');
  const pixels=image();let light=0,dark=0;for(let y=95;y<160;y++)for(let x=95;x<160;x++){
    const value=pixels[(y*canvas.width+x)*4];if(value>200)light++;if(value<55)dark++;
  }
  check(light>200&&dark>200,'original RGB preserves texture detail finer than the native depth grid');
  const depthImage=image(),visibilityPasses=state.cameraFusion.visibility.passes;
  state.settings.depth_only_enabled=true;state.cameras[0].uploadRevision++;draw('blendpcr');
  check(state.cameraFusion.visibility.passes===visibilityPasses,'gray mode skips all RGB visibility work on fresh captures');
  check(center().every(v=>v>0),'depth-only material still renders original RGB geometry');
  state.settings.depth_only_enabled=false;draw('blendpcr');check(image().every((v,i)=>v===depthImage[i]),'depth-only toggle restores the exact original-RGB image');
  const original=state.cameras[0],depth=original.depth;
  const p={...original.metadata,color_width:192,color_height:96,color_projection:{...original.metadata.color_projection,size:[192,96],intrinsics:[128,128,95.5,47.5]}};
  const changed=window.probe.uploadCamera({metadata:p,width:64,height:64,depth,colorPixels:new Uint8Array(192*96*3).fill(255),modelMatrix:original.modelMatrix});state.cameras=[changed];draw('blendpcr');
  check(close(center(),[255,255,255])&&changed.colorWidth===192,'RGB-only resolution changes reallocate textures without changing depth resolution');
  check(gl.getError()===gl.NO_ERROR,'mixed color/depth dimensions leave valid WebGL state');
  return {success:true,checks:checks.length,details:checks};
}

async function benchmarkOriginalRgb(){
  const {state,gl,canvas}=setup();canvas.style.width='960px';canvas.style.height='600px';
  const results={sensorDepth:[1280,720],originalRgb:[1920,1080]};
  fixture([1000,1000,1000],[[1280,720],[1280,720],[1280,720]]);
  async function measure(label){
    draw('blendpcr');gl.finish();const values={};
    for(const kind of ['orbit','capture']){
      const times=[];for(let n=0;n<6;n++){
        if(kind==='capture')for(const c of state.cameras)c.uploadRevision++;
        state.baseViewer.transform[12]=n*.001;
        const t=performance.now();draw('blendpcr');gl.finish();times.push(performance.now()-t);
        await new Promise(resolve=>requestAnimationFrame(resolve));
      }
      values[kind+'MedianMs']=times.sort((a,b)=>a-b)[Math.floor(times.length/2)];
    }
    const pixels=new Uint8Array(canvas.width*canvas.height*4);gl.readPixels(0,0,canvas.width,canvas.height,gl.RGBA,gl.UNSIGNED_BYTE,pixels);
    const mid=(Math.floor(canvas.height/2)*canvas.width+Math.floor(canvas.width/2))*4;
    values.center=[...pixels.slice(mid,mid+3)];values.nonBackground=0;
    for(let i=0;i<pixels.length;i+=4)if(pixels[i]!==8||pixels[i+1]!==9||pixels[i+2]!==10)values.nonBackground++;
    values.method=state.stats.renderMethod;values.visibleCameras=state.cameras.length;
    results[label]=values;
  }
  await measure('aligned');
  state.cameras=state.cameras.map(c=>window.probe.uploadCamera({metadata:{...c.metadata,color_space:'original_rgb',color_width:1920,color_height:1080,
    color_projection:{model:'pinhole',size:[1920,1080],intrinsics:[1300,1300,959.5,539.5],extrinsics:[1,0,0,0,1,0,0,0,1,0,0,0]}},
    width:c.width,height:c.height,depth:c.depth,modelMatrix:c.modelMatrix,colorPixels:new Uint8Array(1920*1080*3).fill(180)}));
  await measure('originalRgb');results.canvas=[canvas.width,canvas.height];return results;
}
