// Shared, unit-tested geometry for the crop volume and its direct-manipulation UI.
export const identity3 = () => [1,0,0,0,1,0,0,0,1];
export const vadd = (a,b) => a.map((v,i)=>v+b[i]);
export const vsub = (a,b) => a.map((v,i)=>v-b[i]);
export const vmul = (a,s) => a.map(v=>v*s);
export const vdot = (a,b) => a.reduce((s,v,i)=>s+v*b[i],0);
export const vcross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
export const vunit = a => vmul(a,1/Math.max(1e-12,Math.hypot(...a)));
export const transform3 = (m,p) => [0,1,2].map(i=>m[i]*p[0]+m[i+3]*p[1]+m[i+6]*p[2]);
export const transpose3 = m => [m[0],m[3],m[6],m[1],m[4],m[7],m[2],m[5],m[8]];
export const multiply3 = (a,b) => [0,1,2].flatMap(i=>transform3(a,b.slice(i*3,i*3+3)));

// Column-major Rz * Ry * Rx, degrees. Deliberately independent of engine Euler defaults.
export function rotation3(degrees=[0,0,0]) {
  const [x,y,z]=degrees.map(v=>v*Math.PI/180),cx=Math.cos(x),sx=Math.sin(x),cy=Math.cos(y),sy=Math.sin(y),cz=Math.cos(z),sz=Math.sin(z);
  return [cz*cy,sz*cy,-sy,cz*sy*sx-sz*cx,sz*sy*sx+cz*cx,cy*sx,cz*sy*cx+sz*sx,sz*sy*cx-cz*sx,cy*cx];
}
export function rotationDegrees(m) {
  const y=Math.asin(Math.max(-1,Math.min(1,-m[2]))),singular=Math.abs(Math.cos(y))<1e-7;
  return [singular?Math.atan2(-m[7],m[4]):Math.atan2(m[5],m[8]),y,singular?0:Math.atan2(m[1],m[0])].map(v=>v*180/Math.PI);
}
export function orientedBox(center,size,rotation=[0,0,0]) {
  const matrix=rotation3(rotation),half=size.map(v=>v/2);
  const corners=Array.from({length:8},(_,i)=>vadd(center,transform3(matrix,half.map((v,a)=>(i&(1<<a))?v:-v))));
  return {center:center.slice(),size:size.slice(),half,rotation:rotation.slice(),matrix,inverse:transpose3(matrix),corners,
    min:[0,1,2].map(a=>Math.min(...corners.map(p=>p[a]))),max:[0,1,2].map(a=>Math.max(...corners.map(p=>p[a])))};
}
export function rayPlane(ray,point,normal) {
  const denominator=vdot(ray.direction,normal);
  if(Math.abs(denominator)<1e-7)return null;
  const t=vdot(vsub(point,ray.origin),normal)/denominator;
  return t>0?vadd(ray.origin,vmul(ray.direction,t)):null;
}
export function rayAxis(ray,point,axis) {
  const a=vdot(ray.direction,axis),denominator=1-a*a;
  if(denominator<1e-5)return null;
  const delta=vsub(ray.origin,point);
  return (vdot(delta,axis)-a*vdot(delta,ray.direction))/denominator;
}
export function projectWorld(point,view) {
  const m=view.viewProjection,p=[...point,1],q=[0,1,2,3].map(i=>p.reduce((s,v,j)=>s+m[j*4+i]*v,0));
  if(q[3]<=0.001)return null;
  return [(q[0]/q[3]+1)*view.width/2,(1-q[1]/q[3])*view.height/2,q[2]/q[3]];
}
export function screenRay(x,y,view) {
  const f=Math.tan(view.fov*Math.PI/360),m=view.viewer;
  const local=[(2*x/view.width-1)*f*view.width/view.height,(1-2*y/view.height)*f,-1];
  return {origin:Array.from(m.slice(12,15)),direction:vunit(transform3([m[0],m[1],m[2],m[4],m[5],m[6],m[8],m[9],m[10]],local))};
}

export function workspaceEditBox(settings,target) {
  if(target==='table') {
    const h=settings.modeled_workspace_table_height??.72,c=settings.modeled_workspace_center||[0,0,0];
    return orientedBox([c[0],c[1]-h/2,c[2]],[settings.modeled_workspace_width||1.2,h,settings.modeled_workspace_depth||.8],[0,settings.modeled_workspace_heading||0,0]);
  }
  const lo=settings.workspace_crop_min||[-.6,-.45,-.6],hi=settings.workspace_crop_max||[.6,.45,.6];
  return orientedBox(lo.map((v,i)=>(v+hi[i])/2),hi.map((v,i)=>v-lo[i]),settings.workspace_crop_rotation||[0,0,0]);
}
export function workspaceEditSettings(box,target) {
  if(target==='table')return {modeled_workspace_center:[box.center[0],box.center[1]+box.size[1]/2,box.center[2]],
    modeled_workspace_width:box.size[0],modeled_workspace_table_height:box.size[1],modeled_workspace_depth:box.size[2],modeled_workspace_heading:box.rotation[1]};
  return {workspace_crop_min:box.center.map((v,i)=>v-box.size[i]/2),workspace_crop_max:box.center.map((v,i)=>v+box.size[i]/2),workspace_crop_rotation:box.rotation.slice()};
}
// Opposite face/corner stays fixed. No snapping or change to scene detail.
export function resizeWorkspaceBox(start,signs,localDelta,target) {
  const minimum=target==='table'?.1:.02,maximum=target==='table'?4:20;
  const size=start.size.map((v,i)=>signs[i]?Math.min(maximum,Math.max(minimum,v+signs[i]*localDelta[i])):v);
  const shift=size.map((v,i)=>(v-start.size[i])*signs[i]/2);
  return orientedBox(vadd(start.center,transform3(start.matrix,shift)),size,start.rotation);
}
