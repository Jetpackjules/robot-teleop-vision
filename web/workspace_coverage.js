// Count native sample locations, not rendered pixels or projected triangles.
// The renderer supplies the same validated table used by its shaders.
export function countWorkspacePoints(cameras,{table=null}={}) {
  let total=0,kept=0;
  for(const camera of cameras) {
    const {width,height,depth,modelMatrix:m,intrinsics:k}=camera;
    const xs=new Float64Array(width);
    for(let x=0;x<width;x++)xs[x]=(x-k[2])/k[0];
    for(let y=0;y<height;y++) {
      const v=-(y-k[3])/k[1],ay=m[5]*v-m[9],ax=m[4]*v-m[8],az=m[6]*v-m[10];
      for(let x=0;x<width;x++) {
        const mm=depth[y*width+x];if(!mm)continue;
        total++;
        const z=mm*.001,u=xs[x],wy=m[13]+z*(m[1]*u+ay);
        // Most removed tabletop samples can be rejected before X/Z work.
        if(table&&(wy<table.center[1]+table.clearance||wy>table.center[1]+table.height))continue;
        const wx=m[12]+z*(m[0]*u+ax),wz=m[14]+z*(m[2]*u+az);
        if(table) {
          const dx=wx-table.center[0],dz=wz-table.center[2];
          if(Math.abs(table.cos*dx-table.sin*dz)>table.width/2
              ||Math.abs(table.sin*dx+table.cos*dz)>table.depth/2)continue;
        }
        kept++;
      }
    }
  }
  return {total,kept,percent:total?100*kept/total:null};
}

export function coverageText(result) {
  if(result.state==='waiting')return `Point cloud kept: ${result.message}`;
  if(result.state==='error')return 'Point cloud kept: count unavailable';
  if(!result.total)return 'Point cloud kept: no measured points';
  return `Point cloud kept: ${result.percent.toFixed(1)}% · ${result.kept.toLocaleString('en-US')} of ${result.total.toLocaleString('en-US')} measured points`;
}

export class WorkspaceCoverage {
  constructor({onResult,interval=100,createWorker=()=>new Worker(new URL('./workspace_coverage_worker.js',import.meta.url),{type:'module'})}) {
    Object.assign(this,{onResult,interval,createWorker});this.sent=new Map();this.lastSent=-Infinity;this.nextId=0;
  }
  update({cameras=[],table=null},now=performance.now()) {
    if(this.closed||this.failed)return;
    const valid=cameras.filter(c=>c.depth?.length===c.width*c.height&&c.modelMatrix&&c.intrinsicsArray);
    let message='';
    if(!valid.length)message='waiting for camera frames…';
    else if(valid.some(c=>c.metadata?.stream_depth_origin||c.metadata?.stream_source_size))message='waiting for full camera frames…';
    else if(valid.some(c=>Number.isFinite(c.lastUpdateMs)&&now-c.lastUpdateMs>=1000))message='waiting for fresh camera frames…';
    if(message) {
      this.latest=null;this.lastSignature='';
      if(message!==this.waiting){this.waiting=message;this.onResult({state:'waiting',message});}
      return;
    }
    this.waiting='';
    const signature=JSON.stringify([table,valid.map((c,i)=>[c.metadata?.id??i,c.uploadRevision??c.lastUpdateMs,c.width,c.height,Array.from(c.modelMatrix),Array.from(c.intrinsicsArray)])]);
    this.latest={cameras:valid,table,signature};this.schedule();
  }
  schedule() {
    if(this.closed||this.failed||this.busy||!this.latest||this.latest.signature===this.lastSignature||this.timer)return;
    this.timer=setTimeout(()=>{this.timer=null;this.flush();},Math.max(0,this.interval-(performance.now()-this.lastSent)));
  }
  flush() {
    if(this.closed||this.failed||this.busy||!this.latest||this.latest.signature===this.lastSignature)return;
    try {
      if(!this.worker) {
        const worker=this.worker=this.createWorker();
        worker.onmessage=({data})=>{
          if(this.closed||this.worker!==worker||data.id!==this.pendingId)return;
          this.busy=false;
          if(data.error){this.fail();return;}
          if(this.latest)this.onResult({state:'ready',...data});
          this.schedule();
        };
        worker.onerror=worker.onmessageerror=()=>this.fail();
      }
      const {cameras,table,signature}=this.latest,updates=[],keys=[];
      cameras.forEach((camera,index)=>{
        const key=String(camera.metadata?.id??index),revision=`${camera.uploadRevision??camera.lastUpdateMs}:${camera.width}:${camera.height}:${camera.modelMatrix}:${camera.intrinsicsArray}`;
        keys.push(key);const previous=this.sent.get(key);
        if(previous?.camera===camera&&previous.revision===revision)return;
        updates.push({key,width:camera.width,height:camera.height,depth:camera.depth,
          modelMatrix:Array.from(camera.modelMatrix),intrinsics:Array.from(camera.intrinsicsArray)});
        this.sent.set(key,{camera,revision});
      });
      for(const key of this.sent.keys())if(!keys.includes(key))this.sent.delete(key);
      this.busy=true;this.pendingId=++this.nextId;this.lastSignature=signature;this.lastSent=performance.now();
      // Clone only changed depth frames; never transfer/detach rendering data.
      // One in-flight job and one latest request bound both memory and latency.
      this.worker.postMessage({id:this.pendingId,updates,keys,table});
    } catch(_){this.fail();}
  }
  fail(){if(this.closed||this.failed)return;this.failed=true;this.busy=false;clearTimeout(this.timer);this.timer=null;this.worker?.terminate();this.worker=null;this.onResult({state:'error'});}
  close(){this.closed=true;clearTimeout(this.timer);this.timer=null;this.worker?.terminate();this.worker=null;this.sent.clear();this.latest=null;}
}
