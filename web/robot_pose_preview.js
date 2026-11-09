// Interpolate only manual visual previews. Normal measured robot updates keep
// their existing timing. Work in parent space so links stay connected.
const identity=()=>[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
const multiply=(a,b)=>Array.from({length:16},(_,k)=>{
  const row=k%4,column=Math.floor(k/4);
  return a[row]*b[column*4]+a[row+4]*b[column*4+1]+a[row+8]*b[column*4+2]+a[row+12]*b[column*4+3];
});
function matrix(value){
  if(!value||![value.basis_x,value.basis_y,value.basis_z,value.origin].every(a=>Array.isArray(a)&&a.length===3&&a.every(Number.isFinite)))return null;
  const m=[...value.basis_x,0,...value.basis_y,0,...value.basis_z,0,...value.origin,1];
  return Math.abs(m[0]*(m[5]*m[10]-m[9]*m[6])-m[4]*(m[1]*m[10]-m[9]*m[2])+m[8]*(m[1]*m[6]-m[5]*m[2]))>1e-12?m:null;
}
function dictionary(m){return {basis_x:m.slice(0,3),basis_y:m.slice(4,7),basis_z:m.slice(8,11),origin:m.slice(12,15)};}
function inverse(m){
  const [a,b,c,d,e,f,g,h,i]=[m[0],m[4],m[8],m[1],m[5],m[9],m[2],m[6],m[10]];
  const det=a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g);
  if(Math.abs(det)<1e-12)return null;
  const n=[(e*i-f*h)/det,(f*g-d*i)/det,(d*h-e*g)/det,0,(c*h-b*i)/det,(a*i-c*g)/det,(b*g-a*h)/det,0,(b*f-c*e)/det,(c*d-a*f)/det,(a*e-b*d)/det,0,0,0,0,1];
  for(let r=0;r<3;r++)n[12+r]=-(n[r]*m[12]+n[r+4]*m[13]+n[r+8]*m[14]);
  return n;
}
const normalize=q=>{const n=Math.hypot(...q)||1;return q.map(v=>v/n);};
function decompose(m){
  const s=[Math.hypot(...m.slice(0,3)),Math.hypot(...m.slice(4,7)),Math.hypot(...m.slice(8,11))];
  if(m[0]*(m[5]*m[10]-m[9]*m[6])-m[4]*(m[1]*m[10]-m[9]*m[2])+m[8]*(m[1]*m[6]-m[5]*m[2])<0)s[0]=-s[0];
  const r=[m[0]/s[0],m[4]/s[1],m[8]/s[2],m[1]/s[0],m[5]/s[1],m[9]/s[2],m[2]/s[0],m[6]/s[1],m[10]/s[2]];
  const trace=r[0]+r[4]+r[8];let q;
  if(trace>0){const k=Math.sqrt(trace+1)*2;q=[(r[7]-r[5])/k,(r[2]-r[6])/k,(r[3]-r[1])/k,k/4];}
  else if(r[0]>r[4]&&r[0]>r[8]){const k=Math.sqrt(1+r[0]-r[4]-r[8])*2;q=[k/4,(r[1]+r[3])/k,(r[2]+r[6])/k,(r[7]-r[5])/k];}
  else if(r[4]>r[8]){const k=Math.sqrt(1+r[4]-r[0]-r[8])*2;q=[(r[1]+r[3])/k,k/4,(r[5]+r[7])/k,(r[2]-r[6])/k];}
  else {const k=Math.sqrt(1+r[8]-r[0]-r[4])*2;q=[(r[2]+r[6])/k,(r[5]+r[7])/k,k/4,(r[3]-r[1])/k];}
  return {p:m.slice(12,15),s,q:normalize(q)};
}
function slerp(a,b,t){
  let dot=a.reduce((v,x,i)=>v+x*b[i],0);if(dot<0){b=b.map(x=>-x);dot=-dot;}
  if(dot>.9995)return normalize(a.map((x,i)=>x+(b[i]-x)*t));
  const angle=Math.acos(Math.min(1,dot)),sin=Math.sin(angle);
  return a.map((x,i)=>(x*Math.sin((1-t)*angle)+b[i]*Math.sin(t*angle))/sin);
}
function compose(a,b,t){
  const [x,y,z,w]=slerp(a.q,b.q,t),s=a.s.map((v,i)=>v+(b.s[i]-v)*t),p=a.p.map((v,i)=>v+(b.p[i]-v)*t);
  return [(1-2*(y*y+z*z))*s[0],2*(x*y+z*w)*s[0],2*(x*z-y*w)*s[0],0,
    2*(x*y-z*w)*s[1],(1-2*(x*x+z*z))*s[1],2*(y*z+x*w)*s[1],0,
    2*(x*z+y*w)*s[2],2*(y*z-x*w)*s[2],(1-2*(x*x+y*y))*s[2],0,...p,1];
}

export class RobotPosePreview {
  constructor(durationMs=70){this.durationMs=durationMs;this.reset();}
  reset(){this.target={};this.parents={};this.from={};this.to={};this.cached={};this.started=-Infinity;this.latestUnixMs=0;this.manual=false;this.base=null;}
  setManual(enabled){this.manual=Boolean(enabled);}
  setBaseTarget(pose,baseLink){
    if(!pose){this.base=null;return;}
    if(!Array.isArray(pose.origin)||!pose.origin.every(Number.isFinite)||!Number.isFinite(pose.yaw))return;
    const reference=this.base?.reference||this.target[baseLink];if(!reference)return;
    this.base={reference:reference.slice(),pose:{origin:pose.origin.slice(),yaw:pose.yaw},baseLink};
  }
  accept(robot,now){
    if(!robot||robot.available===false||!robot.links)return false;
    const stamp=Number(robot.pose_unix_ms||0);
    if(stamp&&stamp<this.latestUnixMs)return false;
    // Once a timestamped pose is available, legacy camera packets must not
    // roll the preview back to an older model while a manual edit is active.
    if(!stamp&&this.latestUnixMs&&(this.manual||this.base))return false;
    const target={};for(const [name,value]of Object.entries(robot.links)){const m=matrix(value);if(m)target[name]=m;}
    if(!Object.keys(target).length)return false;
    const current=this.sample(now),same=Object.keys(target).length===Object.keys(this.target).length&&Object.entries(target).every(([name,m])=>this.target[name]&&m.every((v,i)=>Math.abs(v-this.target[name][i])<1e-8));
    this.latestUnixMs=Math.max(stamp,this.latestUnixMs);
    if(same)return true;
    this.parents=robot.parents||{};this.from={};this.to={};
    for(const [name,m]of Object.entries(target)){
      const parent=this.parents[name],parentInverse=target[parent]&&inverse(target[parent]);
      this.to[name]=decompose(parentInverse?multiply(parentInverse,m):m);
      const old=matrix(current[name])||m,oldParent=matrix(current[parent]),oldInverse=oldParent&&inverse(oldParent);
      this.from[name]=decompose(oldInverse?multiply(oldInverse,old):old);
    }
    const hadPose=Object.keys(this.target).length>0;
    this.target=target;this.started=(this.manual||this.base)&&hadPose?now:now-this.durationMs;
    this.cached=Object.fromEntries(Object.entries(target).map(([name,m])=>[name,dictionary(m)]));
    return true;
  }
  sample(now){
    const progress=Math.max(0,Math.min(1,(now-this.started)/this.durationMs));
    if(progress===1&&!this.base)return this.cached;
    const t=1-Math.pow(1-progress,3),world={},visiting=new Set();
    const resolve=name=>{
      if(world[name])return world[name];
      if(visiting.has(name))return this.target[name]||identity();
      visiting.add(name);const parent=this.parents[name],local=compose(this.from[name],this.to[name],t);
      world[name]=parent&&this.target[parent]?multiply(resolve(parent),local):local;
      visiting.delete(name);return world[name];
    };
    for(const name of Object.keys(this.target))resolve(name);
    if(this.base&&world[this.base.baseLink]){
      const {reference,pose,baseLink}=this.base,a=pose.yaw*Math.PI/180,c=Math.cos(a),s=Math.sin(a);
      const desired=multiply([c,0,-s,0,0,1,0,0,s,0,c,0,0,0,0,1],reference);
      desired.splice(12,3,...pose.origin);const oldInverse=inverse(world[baseLink]);
      if(oldInverse){const delta=multiply(desired,oldInverse);for(const name of Object.keys(world))world[name]=multiply(delta,world[name]);}
    }
    return Object.fromEntries(Object.entries(world).map(([name,m])=>[name,dictionary(m)]));
  }
}
