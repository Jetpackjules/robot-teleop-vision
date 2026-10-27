const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const url=source=>'data:text/javascript;base64,'+Buffer.from(source).toString('base64');
(async()=>{
 const web=path.join(__dirname,'../web'),transform=url(fs.readFileSync(path.join(web,'workspace_transform.js'),'utf8'));
 const crop=url(fs.readFileSync(path.join(web,'workspace_crop.js'),'utf8').replace(/'\.\/workspace_transform\.js[^']*'/,JSON.stringify(transform)));
 const source=fs.readFileSync(path.join(web,'rgb_visibility_region.js'),'utf8').replace(/'\.\/workspace_crop\.js[^']*'/,JSON.stringify(crop));
 const {rgbVisibilityRegion,regionContains,rgbRegionClip}=await import(url(source));
 const width=640,height=480,matrix=new Float32Array([1.25,0,0,0,0,1.65,0,0,0,0,1,1,0,0,-.02,0]);
 const full={x:0,y:0,width,height},project=(m,p)=>[0,1,2,3].map(r=>m[r]*p[0]+m[r+4]*p[1]+m[r+8]*p[2]+m[r+12]);
 assert.deepEqual(rgbVisibilityRegion(null,matrix,width,height),full);
 assert.deepEqual(rgbVisibilityRegion({min:[-.2,-.2,-.1],max:[.2,.2,.1]},matrix,width,height),full,'near-plane crossings retain complete RGB coverage');
 assert.equal(rgbVisibilityRegion({min:[10,-.2,1],max:[11,.2,2]},matrix,width,height).width,0,'a disjoint world box needs no RGB samples');
 assert.equal(rgbRegionClip(matrix,full,width,height),matrix,'full coverage keeps the original float32 matrix');
 assert(regionContains(full,{x:1,y:1,width:0,height:0}));assert(!regionContains({x:10,y:10,width:10,height:10},{x:9,y:10,width:2,height:2}));
 let seed=17,checks=6;const random=()=>((seed=(Math.imul(seed,1664525)+1013904223)>>>0)/4294967296);
 for(let box=0;box<100;box++){
  const lo=[random()-.5,random()-.5,.1+random()*2],hi=lo.map(v=>v+.02+random()*.5),region=rgbVisibilityRegion({min:lo,max:hi},matrix,width,height);
  for(let n=0;n<100;n++){
   const p=lo.map((v,i)=>v+random()*(hi[i]-v)),clip=project(matrix,p),x=(clip[0]/clip[3]+1)*width/2,y=(clip[1]/clip[3]+1)*height/2;
   if(x<0||x>width-1||y<0||y>height-1)continue;
   assert(x>=region.x&&Math.floor(x)+1<region.x+region.width);assert(y>=region.y&&Math.floor(y)+1<region.y+region.height);checks+=2;
  }
 }
 const region={x:100,y:110,width:200,height:90},clipMatrix=rgbRegionClip(matrix,region,width,height);
 for(const a of [0,1])for(const b of [0,1]){
  const x=2*(region.x+region.width*a)/width-1,y=2*(region.y+region.height*b)/height-1,p=[x/matrix[0],y/matrix[5],1],clip=project(clipMatrix,p);
  assert(Math.abs(clip[0]/clip[3]-(a*2-1))<1e-6);assert(Math.abs(clip[1]/clip[3]-(b*2-1))<1e-6);checks+=2;
 }
 console.log(JSON.stringify({checks}));
})().catch(e=>{console.error(e);process.exitCode=1;});
