import {SCENE_LIGHTING_SHADER,bindSceneLighting} from './scene_lighting.js?v=20261021_lit_workspace';
// Optional modeled stage. Camera samples, calibration and robot poses are
// never changed: this controls draw submission and the displayed environment.
export const MODELED_WORKSPACE_DEFAULTS = Object.freeze({
  modeled_workspace_enabled: false,
  modeled_workspace_center: [0, 0, 0],
  modeled_workspace_width: 1.2,
  modeled_workspace_depth: 0.8,
  modeled_workspace_heading: 0,
  modeled_workspace_clearance: 0.006,
  modeled_workspace_height: 0.8,
  modeled_workspace_table_height: 0.72,
  modeled_workspace_light_angle: 35,
  scene_lighting_enabled: true,
  scene_shadows_enabled: true,
});

export function modeledWorkspace(settings) {
  if (settings.modeled_workspace_enabled !== true) return null;
  const value = {...MODELED_WORKSPACE_DEFAULTS, ...settings};
  const center = value.modeled_workspace_center;
  if (!Array.isArray(center) || center.length !== 3 || !center.every(v => Number.isFinite(v) && Math.abs(v) <= 20)) return null;
  const limits = {width:[0.1,4],depth:[0.1,4],table_height:[0.1,4],heading:[-180,180],clearance:[0,0.03],height:[0.05,3],light_angle:[-180,180]};
  for (const [key,[lo,hi]] of Object.entries(limits)) {
    const v=value[`modeled_workspace_${key}`];
    if (!Number.isFinite(v) || v<lo || v>hi) return null;
  }
  const heading=value.modeled_workspace_heading*Math.PI/180;
  const result={center:center.slice(),width:value.modeled_workspace_width,depth:value.modeled_workspace_depth,
    heading,cos:Math.cos(heading),sin:Math.sin(heading),clearance:value.modeled_workspace_clearance,
    height:value.modeled_workspace_height,tableHeight:value.modeled_workspace_table_height,lightAngle:value.modeled_workspace_light_angle*Math.PI/180};
  result.key=JSON.stringify(result);
  return result;
}

export function modeledWorkspaceOutside(point, stage) {
  if (!stage) return false;
  const x=point[0]-stage.center[0],y=point[1]-stage.center[1],z=point[2]-stage.center[2];
  return Math.abs(stage.cos*x-stage.sin*z)>stage.width/2
    || Math.abs(stage.sin*x+stage.cos*z)>stage.depth/2 || y<stage.clearance || y>stage.height;
}

// The oriented tabletop's AABB is only a conservative submission bound. The
// shader below makes the exact cut, retaining full native detail inside it.
export function modeledWorkspaceBounds(settings, crop=null, margin=0) {
  const stage=modeledWorkspace(settings);
  if (!stage) return crop;
  const x=(Math.abs(stage.cos)*stage.width+Math.abs(stage.sin)*stage.depth)/2;
  const z=(Math.abs(stage.sin)*stage.width+Math.abs(stage.cos)*stage.depth)/2;
  const min=[stage.center[0]-x-margin,stage.center[1]+stage.clearance-margin,stage.center[2]-z-margin];
  const max=[stage.center[0]+x+margin,stage.center[1]+stage.height+margin,stage.center[2]+z+margin];
  if (crop) for(let axis=0;axis<3;axis++) {min[axis]=Math.max(min[axis],crop.min[axis]);max[axis]=Math.min(max[axis],crop.max[axis]);}
  return {min,max,key:`${crop?.key||''}:${stage.key}:${margin}`};
}

export const MODELED_WORKSPACE_SHADER = `
uniform highp int u_modeled_workspace_enabled;
uniform highp vec3 u_modeled_workspace_center;
uniform highp vec4 u_modeled_workspace_extent;
uniform highp vec2 u_modeled_workspace_rotation;
bool modeled_workspace_outside(vec3 world) {
  if(u_modeled_workspace_enabled!=1)return false;
  vec3 p=world-u_modeled_workspace_center;
  vec2 q=vec2(u_modeled_workspace_rotation.x*p.x-u_modeled_workspace_rotation.y*p.z,
    u_modeled_workspace_rotation.y*p.x+u_modeled_workspace_rotation.x*p.z);
  return any(greaterThan(abs(q),u_modeled_workspace_extent.xy))
    ||p.y<u_modeled_workspace_extent.z||p.y>u_modeled_workspace_extent.w;
}
`;

export function bindModeledWorkspace(gl, program, settings, margin=0) {
  const stage=modeledWorkspace(settings);
  gl.uniform1i(gl.getUniformLocation(program,'u_modeled_workspace_enabled'),stage?1:0);
  if (!stage) return;
  gl.uniform3fv(gl.getUniformLocation(program,'u_modeled_workspace_center'),stage.center);
  gl.uniform4f(gl.getUniformLocation(program,'u_modeled_workspace_extent'),stage.width/2+margin,stage.depth/2+margin,
    stage.clearance-margin,stage.height+margin);
  gl.uniform2f(gl.getUniformLocation(program,'u_modeled_workspace_rotation'),stage.cos,stage.sin);
}

export function modeledWorkspaceGeometry(stage) {
  const vertices=[];
  const face=(points,normal,color,material)=>{
    for(const i of [0,1,2,0,2,3]) vertices.push(...points[i],...normal,...color,...material);
  };
  const box=(x0,y0,z0,x1,y1,z1,color,material)=>{
    face([[x0,y1,z0],[x0,y1,z1],[x1,y1,z1],[x1,y1,z0]],[0,1,0],color,material);
    face([[x0,y0,z1],[x0,y0,z0],[x1,y0,z0],[x1,y0,z1]],[0,-1,0],color,material);
    face([[x0,y0,z0],[x0,y0,z1],[x0,y1,z1],[x0,y1,z0]],[-1,0,0],color,material);
    face([[x1,y0,z1],[x1,y0,z0],[x1,y1,z0],[x1,y1,z1]],[1,0,0],color,material);
    face([[x1,y0,z0],[x0,y0,z0],[x0,y1,z0],[x1,y1,z0]],[0,0,-1],color,material);
    face([[x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]],[0,0,1],color,material);
  };
  const w=stage.width/2,d=stage.depth/2,h=stage.tableHeight??.72,leg=Math.min(0.055,stage.width/8,stage.depth/8);
  box(-w,-0.045,-d,w,0,d,[0.19,0.22,0.25],[.7,.1]);
  for(const x of [-w+leg,w-leg])for(const z of [-d+leg,d-leg])
    box(x-leg/2,-h,z-leg/2,x+leg/2,-0.045,z+leg/2,[0.31,0.34,0.37],[.6,.8]);
  const extent=Math.max(4,stage.width*3,stage.depth*3);
  face([[-extent,-h,-extent],[-extent,-h,extent],[extent,-h,extent],[extent,-h,-extent]],
    [0,1,0],[0.105,0.13,0.15],[.8,.1]);
  return new Float32Array(vertices);
}

const VERTEX = `#version 300 es
precision highp float;
layout(location=0)in vec3 a_position;
layout(location=1)in vec3 a_normal;
layout(location=2)in vec3 a_color;
layout(location=3)in vec2 a_material;
uniform mat4 u_view_projection;
uniform vec3 u_center;
uniform vec2 u_rotation;
out vec3 v_position;out vec3 v_normal;out vec3 v_color;out vec3 v_world;out vec2 v_material;
void main(){
  vec3 p=vec3(u_rotation.x*a_position.x+u_rotation.y*a_position.z,a_position.y,
    -u_rotation.y*a_position.x+u_rotation.x*a_position.z);
  v_position=a_position;v_normal=vec3(u_rotation.x*a_normal.x+u_rotation.y*a_normal.z,a_normal.y,-u_rotation.y*a_normal.x+u_rotation.x*a_normal.z);
  v_color=a_color;v_world=p+u_center;v_material=a_material;
  gl_Position=u_view_projection*vec4(p+u_center,1);
}`;
const FRAGMENT = `#version 300 es
precision highp float;
in vec3 v_position;in vec3 v_normal;in vec3 v_color;in vec3 v_world;in vec2 v_material;
uniform vec3 u_light;
uniform vec2 u_half_size;
uniform float u_table_height;
out vec4 out_color;
${SCENE_LIGHTING_SHADER}
void main(){
  vec3 color=scene_material(v_color,v_normal,v_world,v_material);
  if(u_scene_shadow_enabled!=1&&v_position.y<-u_table_height+.001){
    vec2 p=v_position.xz+u_light.xz*u_table_height/max(u_light.y,.2);
    vec2 q=abs(p)-u_half_size;
    float distance=length(max(q,vec2(0)))+min(max(q.x,q.y),0.0);
    color*=mix(.62,1.,smoothstep(-.02,.14,distance));
  }
  out_color=vec4(color,1);
}`;

export class ModeledWorkspaceRenderer {
  constructor(gl) {
    this.gl=gl;
    const compile=(type,source)=>{
      const shader=gl.createShader(type);gl.shaderSource(shader,source);gl.compileShader(shader);
      if(!gl.getShaderParameter(shader,gl.COMPILE_STATUS)){const error=gl.getShaderInfoLog(shader);gl.deleteShader(shader);throw new Error(error);}
      return shader;
    };
    const vertex=compile(gl.VERTEX_SHADER,VERTEX),fragment=compile(gl.FRAGMENT_SHADER,FRAGMENT);
    this.program=gl.createProgram();gl.attachShader(this.program,vertex);gl.attachShader(this.program,fragment);gl.linkProgram(this.program);
    gl.deleteShader(vertex);gl.deleteShader(fragment);
    if(!gl.getProgramParameter(this.program,gl.LINK_STATUS)){const error=gl.getProgramInfoLog(this.program);gl.deleteProgram(this.program);throw new Error(error);}
    this.vao=gl.createVertexArray();this.buffer=gl.createBuffer();
  }
  prepare(stage) {
    const gl=this.gl;
    gl.bindVertexArray(this.vao);gl.bindBuffer(gl.ARRAY_BUFFER,this.buffer);
    const key=`${stage.width}:${stage.depth}:${stage.tableHeight}`;
    if(this.key!==key){const vertices=modeledWorkspaceGeometry(stage);gl.bufferData(gl.ARRAY_BUFFER,vertices,gl.STATIC_DRAW);this.count=vertices.length/11;this.key=key;}
    for(let i=0;i<4;i++){gl.enableVertexAttribArray(i);gl.vertexAttribPointer(i,i===3?2:3,gl.FLOAT,false,44,i*12);}
  }
  drawDepth(settings,program,modelUniform) {
    const stage=modeledWorkspace(settings);if(!stage)return;
    this.prepare(stage);const gl=this.gl;
    const c=stage.cos,s=stage.sin,p=stage.center;
    gl.uniformMatrix4fv(modelUniform,false,[c,0,-s,0,0,1,0,0,s,0,c,0,...p,1]);
    gl.drawArrays(gl.TRIANGLES,0,this.count);gl.bindVertexArray(null);
  }
  render(settings,viewProjection,eye,shadow) {
    const stage=modeledWorkspace(settings);if(!stage)return;
    const gl=this.gl;this.prepare(stage);
    gl.useProgram(this.program);
    bindSceneLighting(gl,this.program,settings,eye,shadow);
    gl.uniformMatrix4fv(gl.getUniformLocation(this.program,'u_view_projection'),false,viewProjection);
    gl.uniform3fv(gl.getUniformLocation(this.program,'u_center'),stage.center);
    gl.uniform2f(gl.getUniformLocation(this.program,'u_rotation'),stage.cos,stage.sin);
    const angle=stage.lightAngle-stage.heading;
    gl.uniform3f(gl.getUniformLocation(this.program,'u_light'),Math.cos(angle)*.5,.866,Math.sin(angle)*.5);
    gl.uniform2f(gl.getUniformLocation(this.program,'u_half_size'),stage.width/2,stage.depth/2);
    gl.uniform1f(gl.getUniformLocation(this.program,'u_table_height'),stage.tableHeight);
    gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);gl.depthMask(true);gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);
    gl.drawArrays(gl.TRIANGLES,0,this.count);gl.bindVertexArray(null);
  }
  dispose(){const gl=this.gl;gl.deleteBuffer(this.buffer);gl.deleteVertexArray(this.vao);gl.deleteProgram(this.program);}
}
