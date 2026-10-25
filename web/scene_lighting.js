// Shared CAD lighting, matching RoboCade's ambient, white key and blue fill.
// Camera RGB is deliberately never passed through this material shader.
export const SCENE_LIGHTING_SHADER = `
uniform int u_scene_lighting_enabled;
uniform vec3 u_scene_key;
uniform vec3 u_scene_eye;
uniform int u_scene_shadow_enabled;
uniform sampler2D u_scene_shadow;
uniform mat4 u_scene_light_matrix;
const float SCENE_PI=3.14159265359;
vec3 scene_linear(vec3 c){return mix(c/12.92,pow((c+.055)/1.055,vec3(2.4)),step(vec3(.04045),c));}
vec3 scene_srgb(vec3 c){c=max(c,vec3(0));return mix(c*12.92,1.055*pow(c,vec3(1./2.4))-.055,step(vec3(.0031308),c));}
float scene_shadow(vec3 world,vec3 normal){
  if(u_scene_shadow_enabled!=1)return 1.;
  vec4 clip=u_scene_light_matrix*vec4(world,1.);vec3 p=clip.xyz/clip.w*.5+.5;
  if(any(lessThan(p,vec3(0)))||any(greaterThan(p,vec3(1))))return 1.;
  // PCF samples neighboring texels on sloping surfaces. A slope-aware receiver
  // bias plus caster polygon offset avoids self-shadow stripes on the table.
  float bias=max(.002*(1.-max(dot(normal,u_scene_key),0.)),.0005),sum=0.;
  vec2 texel=1./vec2(textureSize(u_scene_shadow,0));
  for(int y=-1;y<=1;y++)for(int x=-1;x<=1;x++)
    sum+=p.z-bias<=texture(u_scene_shadow,p.xy+vec2(x,y)*texel).r?1.:0.;
  return sum/9.;
}
vec3 scene_brdf(vec3 base,vec3 n,vec3 v,vec3 l,float roughness,float metalness){
  float nl=max(dot(n,l),0.),nv=max(dot(n,v),.0001);
  vec3 h=normalize(v+l);float nh=max(dot(n,h),0.),vh=max(dot(v,h),0.);
  float a=roughness*roughness,a2=a*a,d=nh*nh*(a2-1.)+1.;
  float distribution=a2/max(SCENE_PI*d*d,.00001),k=(roughness+1.)*(roughness+1.)/8.;
  float geometry=nv/(nv*(1.-k)+k)*nl/max(nl*(1.-k)+k,.0001);
  vec3 f0=mix(vec3(.04),base,metalness),f=f0+(1.-f0)*pow(1.-vh,5.);
  return ((1.-f)*(1.-metalness)*base/SCENE_PI+distribution*geometry*f/max(4.*nv*nl,.0001))*nl;
}
vec3 scene_material(vec3 color,vec3 normal,vec3 world,vec2 material){
  if(u_scene_lighting_enabled!=1)return color;
  vec3 n=dot(normal,normal)>1e-10?normalize(normal):vec3(0,1,0);
  vec3 delta=u_scene_eye-world,v=dot(delta,delta)>1e-10?normalize(delta):n,base=scene_linear(color);
  float roughness=clamp(material.x,.12,1.),metalness=clamp(material.y,0.,1.);
  vec3 ambient=base*scene_linear(vec3(64./255.))*2.;
  vec3 key=scene_brdf(base,n,v,u_scene_key,roughness,metalness)*3.*scene_shadow(world,n);
  vec3 fill=scene_brdf(base,n,v,normalize(vec3(-5,10,-5)),roughness,metalness)*scene_linear(vec3(135,206,235)/255.);
  return scene_srgb(ambient+key+fill);
}`;

export function sceneKeyDirection(settings={}) {
  const angle=Number.isFinite(settings.modeled_workspace_light_angle)?settings.modeled_workspace_light_angle:35;
  const a=angle*Math.PI/180,r=Math.hypot(5,7),length=Math.hypot(r,10);
  return [Math.cos(a)*r/length,10/length,Math.sin(a)*r/length];
}
const bindings=new WeakMap();
const fallbackTextures=new WeakMap();
export function bindSceneLighting(gl,program,settings,eye,shadow=null,enabled=true) {
  let u=bindings.get(program);
  if(!u){u=Object.fromEntries(['lighting_enabled','key','eye','shadow_enabled','shadow','light_matrix'].map(name=>[name,gl.getUniformLocation(program,'u_scene_'+name)]));bindings.set(program,u);}
  const lit=enabled&&settings.scene_lighting_enabled!==false;
  gl.uniform1i(u.lighting_enabled,lit?1:0);gl.uniform3fv(u.key,sceneKeyDirection(settings));
  gl.uniform3fv(u.eye,eye||[0,0,0]);
  gl.uniform1i(u.shadow_enabled,lit&&shadow?.ready&&settings.scene_shadows_enabled!==false?1:0);
  // Bind a complete texture even with shadows disabled; WebGL validates every
  // active sampler. The shadow renderer supplies its initialized depth map.
  gl.uniform1i(u.shadow,0);
  let texture=shadow?.texture;
  if(!texture){texture=fallbackTextures.get(gl);if(!texture){texture=gl.createTexture();gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,texture);gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA,1,1,0,gl.RGBA,gl.UNSIGNED_BYTE,new Uint8Array([255,255,255,255]));gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);fallbackTextures.set(gl,texture);}}
  gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,texture);
  if(shadow?.matrix)gl.uniformMatrix4fv(u.light_matrix,false,shadow.matrix);
}

const dot=(a,b)=>a.reduce((sum,v,i)=>sum+v*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const unit=a=>a.map(v=>v/Math.max(1e-9,Math.hypot(...a)));
function lightMatrix(center,key,extent) {
  const back=unit(key),right=unit(cross([0,1,0],back)),up=cross(back,right);
  const eye=center.map((v,i)=>v+back[i]*extent*3),near=.01,far=extent*6;
  const view=[right[0],up[0],back[0],0,right[1],up[1],back[1],0,right[2],up[2],back[2],0,-dot(right,eye),-dot(up,eye),-dot(back,eye),1];
  const projection=[1/extent,0,0,0,0,1/extent,0,0,0,0,-2/(far-near),0,0,0,-(far+near)/(far-near),1];
  return Float32Array.from({length:16},(_,i)=>[0,1,2,3].reduce((sum,k)=>sum+projection[k*4+i%4]*view[Math.floor(i/4)*4+k],0));
}

export class SceneShadowRenderer {
  constructor(gl) {
    this.gl=gl;this.size=1024;this.ready=false;
    const shaders=[];this.program=gl.createProgram();
    try {
      for(const [type,source] of [[gl.VERTEX_SHADER,`#version 300 es\nprecision highp float;layout(location=0)in vec3 a_position;uniform mat4 u_model;uniform mat4 u_light_matrix;void main(){gl_Position=u_light_matrix*u_model*vec4(a_position,1);}`],
        [gl.FRAGMENT_SHADER,'#version 300 es\nprecision highp float;void main(){}']]) {
        const shader=gl.createShader(type);shaders.push(shader);gl.shaderSource(shader,source);gl.compileShader(shader);
        if(!gl.getShaderParameter(shader,gl.COMPILE_STATUS))throw new Error(gl.getShaderInfoLog(shader));gl.attachShader(this.program,shader);
      }
      gl.linkProgram(this.program);if(!gl.getProgramParameter(this.program,gl.LINK_STATUS))throw new Error(gl.getProgramInfoLog(this.program));
      this.model=gl.getUniformLocation(this.program,'u_model');this.light=gl.getUniformLocation(this.program,'u_light_matrix');
      this.texture=gl.createTexture();gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,this.texture);
      gl.texImage2D(gl.TEXTURE_2D,0,gl.DEPTH_COMPONENT24,this.size,this.size,0,gl.DEPTH_COMPONENT,gl.UNSIGNED_INT,null);
      for(const name of [gl.TEXTURE_MIN_FILTER,gl.TEXTURE_MAG_FILTER])gl.texParameteri(gl.TEXTURE_2D,name,gl.NEAREST);
      for(const name of [gl.TEXTURE_WRAP_S,gl.TEXTURE_WRAP_T])gl.texParameteri(gl.TEXTURE_2D,name,gl.CLAMP_TO_EDGE);
      this.framebuffer=gl.createFramebuffer();gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);
      gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.DEPTH_ATTACHMENT,gl.TEXTURE_2D,this.texture,0);
      gl.drawBuffers([gl.NONE]);gl.readBuffer(gl.NONE);
      if(gl.checkFramebufferStatus(gl.FRAMEBUFFER)!==gl.FRAMEBUFFER_COMPLETE)throw new Error('CAD shadow framebuffer unavailable');
      gl.clearDepth(1);gl.clear(gl.DEPTH_BUFFER_BIT);
      this.matrix=lightMatrix([0,0,0],[0,1,.1],1);
    } catch(error){this.dispose();throw error;}
    finally{for(const shader of shaders)gl.deleteShader(shader);gl.bindFramebuffer(gl.FRAMEBUFFER,null);}
  }
  update(settings,center,extent,signature,draw) {
    if(this.signature===signature)return;
    const gl=this.gl,framebuffer=gl.getParameter(gl.FRAMEBUFFER_BINDING),viewport=gl.getParameter(gl.VIEWPORT),vao=gl.getParameter(gl.VERTEX_ARRAY_BINDING);
    const offset=gl.isEnabled(gl.POLYGON_OFFSET_FILL),factor=gl.getParameter(gl.POLYGON_OFFSET_FACTOR),units=gl.getParameter(gl.POLYGON_OFFSET_UNITS);
    this.matrix=lightMatrix(center,sceneKeyDirection(settings),extent);
    try {
      gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);gl.viewport(0,0,this.size,this.size);
      gl.colorMask(false,false,false,false);gl.depthMask(true);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);
      gl.enable(gl.POLYGON_OFFSET_FILL);gl.polygonOffset(1.5,4);
      gl.clearDepth(1);gl.clear(gl.DEPTH_BUFFER_BIT);gl.useProgram(this.program);gl.uniformMatrix4fv(this.light,false,this.matrix);
      draw(this.program,this.model);this.signature=signature;this.ready=true;
    } finally{gl.polygonOffset(factor,units);if(!offset)gl.disable(gl.POLYGON_OFFSET_FILL);gl.colorMask(true,true,true,true);gl.bindFramebuffer(gl.FRAMEBUFFER,framebuffer);gl.viewport(...viewport);gl.bindVertexArray(vao);}
  }
  dispose(){const gl=this.gl;if(this.texture)gl.deleteTexture(this.texture);if(this.framebuffer)gl.deleteFramebuffer(this.framebuffer);if(this.program)gl.deleteProgram(this.program);this.ready=false;}
}
