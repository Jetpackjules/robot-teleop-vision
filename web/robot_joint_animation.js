// Measured encoder FK in the camera clock domain. No commanded targets,
// extrapolation, local-clock rebasing, or calibration fitting.
const radians = degrees => degrees * Math.PI / 180;
const wrap = x => ((x + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
function matrix(t) {
  if (!t || !['basis_x','basis_y','basis_z','origin'].every(k => Array.isArray(t[k]) && t[k].length === 3 && t[k].every(Number.isFinite))) throw Error('Invalid calibrated transform');
  return [...t.basis_x,0,...t.basis_y,0,...t.basis_z,0,...t.origin,1];
}
function multiply(a,b) {
  return Array.from({length:16}, (_,i) => {
    const r=i%4,c=Math.floor(i/4)*4;
    return a[r]*b[c]+a[r+4]*b[c+1]+a[r+8]*b[c+2]+a[r+12]*b[c+3];
  });
}
function rotation(axis, angle) {
  const n=Math.hypot(...axis); if (!(n>0)) throw Error('Invalid joint axis');
  const [x,y,z]=axis.map(v=>v/n),c=Math.cos(angle),s=Math.sin(angle),t=1-c;
  return [t*x*x+c,t*x*y+s*z,t*x*z-s*y,0,
    t*x*y-s*z,t*y*y+c,t*y*z+s*x,0,t*x*z+s*y,t*y*z-s*x,t*z*z+c,0,0,0,0,1];
}
function dictionary(m) { return {basis_x:m.slice(0,3),basis_y:m.slice(4,7),basis_z:m.slice(8,11),origin:m.slice(12,15)}; }
function curve(x, xs, ys) {
  const count=Math.min(xs?.length||0,ys?.length||0);
  if (count<2) return null;
  let i=0; while(i<count-2 && x>xs[i+1]) i++;
  if (!(xs[i+1]>xs[i])) throw Error('Invalid saved opening curve');
  return ys[i]+(ys[i+1]-ys[i])*(x-xs[i])/(xs[i+1]-xs[i]);
}
export function measuredJointTransforms(model, normalized) {
  if (model?.schema!=='so101_measured_fk_v1' || model.manual_active || model.joints?.length!==6 || normalized?.length!==6 || !normalized.every(Number.isFinite)) return null;
  const world={[model.base_link]:matrix(model.base_transform)};
  for(let i=0;i<6;i++) {
    const j=model.joints[i]; let angle;
    if(i===5) {
      const g=model.gripper;
      let degrees=curve(normalized[i],g.samples_normalized,g.samples_degrees);
      if(degrees===null && g.moving_jaw) degrees=curve(normalized[i],g.samples_normalized,g.fallback_samples_degrees);
      if(degrees===null) degrees=g.offset+g.scale*normalized[i]+g.curvature*normalized[i]*(100-normalized[i])/100;
      angle=radians(degrees);
      // Old hosts always clipped the stock jaw. New hosts export its visual
      // clamp explicitly so a validated opening curve can use its full range.
      if(!g.moving_jaw && g.clamp_limits!==false) angle=clamp(angle,...j.limits);
    } else {
      angle=j.force_zero?0:wrap(radians(normalized[i]*j.direction+j.offset_degrees));
      if(!j.force_zero && model.clamp_limits) angle=clamp(angle,...j.limits);
      if(i===4) angle=wrap(angle+radians(j.extra_degrees||0));
    }
    let local=multiply(matrix(j.origin),rotation(j.axis,angle));
    // Mount offsets are in the joint parent's frame, before the local tool correction.
    if(i===4) for(let k=0;k<3;k++) local[12+k]+=j.parent_translation[k];
    local=multiply(local,matrix(j.post));
    if(i===3) for(let k=0;k<3;k++) local[12+k]+=j.parent_translation[k];
    if(!world[j.parent] || !local.every(Number.isFinite)) throw Error('Invalid calibrated chain');
    world[j.child]=multiply(world[j.parent],local);
  }
  return Object.fromEntries(Object.entries(world).map(([name,m])=>[name,dictionary(m)]));
}

export class MeasuredJointAnimation {
  constructor(){this.reset();}
  reset(){this.model=null;this.samples=[];this.status={mode:'legacy'};this.last=null;}
  accept(robot) {
    const model=robot?.kinematics, history=robot?.measured_joint_history;
    if(model?.schema!=='so101_measured_fk_v1' || !Array.isArray(history)) return false;
    this.model=model;
    // Each packet carries a bounded authoritative snapshot, so out-of-order
    // arrival cannot discard newer measurements already received.
    const all=[...this.samples,...history].filter(s=>Number.isFinite(s.unix_ms)&&Array.isArray(s.normalized)&&s.normalized.length===6&&s.normalized.every(Number.isFinite));
    all.sort((a,b)=>a.unix_ms-b.unix_ms);
    this.samples=all.filter((s,i)=>i===0||s.unix_ms>all[i-1].unix_ms).slice(-160);
    return true;
  }
  sample(captureUnixMs) {
    if(!this.model || this.model.manual_active || !Number.isFinite(captureUnixMs) || captureUnixMs<=0 || !this.samples.length) return null;
    let right=this.samples.findIndex(s=>s.unix_ms>=captureUnixMs);
    let a,b,f=0,mode='interpolated';
    if(right===0){a=b=this.samples[0];mode=captureUnixMs===a.unix_ms?'measured':'held_early';}
    else if(right<0){a=b=this.samples.at(-1);mode='held_latest';}
    else {a=this.samples[right-1];b=this.samples[right];f=(captureUnixMs-a.unix_ms)/(b.unix_ms-a.unix_ms);}
    const gap=b.unix_ms-a.unix_ms;
    if(gap>350){b=a;f=0;mode='held_gap';}
    const normalized=a.normalized.map((v,i)=>{
      const delta=i===5?b.normalized[i]-v:wrap(radians(b.normalized[i]-v))*180/Math.PI;
      return v+delta*f;
    });
    try {
      const links=measuredJointTransforms(this.model,normalized);
      if(!links)return null;
      this.status={mode,captureUnixMs,sampleBeforeMs:a.unix_ms,sampleAfterMs:b.unix_ms,
        errorMs:mode==='interpolated'?0:Math.abs(captureUnixMs-a.unix_ms),gapMs:gap,
        synchronized:mode==='interpolated'||mode==='measured',samples:this.samples.length};
      this.last=links;return links;
    }catch(_error){this.status={mode:'invalid_model'};return null;}
  }
}
