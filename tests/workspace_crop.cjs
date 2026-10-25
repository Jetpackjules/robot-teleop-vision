const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
(async()=>{
  const geometry='data:text/javascript;base64,'+Buffer.from(fs.readFileSync(path.join(__dirname,'../web/workspace_transform.js'),'utf8')).toString('base64');
  const source=fs.readFileSync(path.join(__dirname,'../web/workspace_crop.js'),'utf8').replace(/'\.\/workspace_transform.js[^']*'/,JSON.stringify(geometry));
  const {workspaceCropBounds,workspaceSourceWindow,workspaceCropLines,workspaceDepthTiles,workspaceVisibleTiles}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
  let checks=0;const check=(test,message)=>{assert.ok(test,message);checks++;};
  const settings={workspace_crop_enabled:true,workspace_crop_min:[-.1,-.2,-1.1],workspace_crop_max:[.1,.2,-.9]};
  const bounds=workspaceCropBounds(settings);
  check(bounds&&workspaceCropLines(bounds).length===72,'world box has all twelve edges');
  check(!workspaceCropBounds({...settings,workspace_crop_enabled:false}),'disabled crop preserves the full scene');
  for(const value of [[0,0],['0',0,-1],[0,NaN,-1],[0,Infinity,-1],[0,0,-2]])
    check(!workspaceCropBounds({...settings,workspace_crop_max:value}),'invalid or inverted bounds never crop');
  const identity=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
  const camera={width:1280,height:720,modelMatrix:identity,intrinsicsArray:[640,640,639.5,359.5]};
  const window=workspaceSourceWindow(camera,bounds);
  check(window.width<200&&window.height<320,'small projected box skips most native source geometry');
  check(workspaceSourceWindow(camera,null).width===1280,'no box submits the original full width');
  check(workspaceSourceWindow(camera,{min:[-.1,-.1,.1],max:[.1,.1,.3]}).width===0,'a box wholly behind a camera submits no geometry');
  check(workspaceSourceWindow(camera,{min:[-.1,-.1,-.1],max:[.1,.1,.1]}).width===1280,'a box crossing the near plane safely keeps the full grid');
  const scaled=identity.slice();scaled[0]=2;
  check(workspaceSourceWindow({...camera,modelMatrix:scaled},bounds).width===1280,'non-rigid third-party poses fall back without deleting points');
  const depth=new Uint16Array(128*64).fill(5000);
  for(let y=20;y<40;y++)for(let x=50;x<78;x++)depth[y*128+x]=700;
  const inside={width:128,height:64,intrinsicsArray:[100,100,63.5,31.5],modelMatrix:identity,depth,uploadRevision:1};
  const volume={min:[-.3,-.3,-1],max:[.3,.3,.1],key:'inside'};
  const tiles=workspaceVisibleTiles(inside,volume);
  check(tiles.length>0&&tiles.cellCount<127*63*.6,'depth tiles cull distant geometry when the camera is inside the box');
  check(workspaceVisibleTiles(inside,volume)===tiles,'orbiting reuses conservative source visibility without a new depth scan');
  const ranges=workspaceDepthTiles(new Uint16Array(33*9),33,9);
  check(ranges.every(v=>v===0),'invalid depth never creates a retained tile');
  for(let y=20;y<40;y++)for(let x=50;x<78;x++) {
    let found=false;
    for(let i=0;i<tiles.length;i+=2)if(x>=tiles[i]&&x<=tiles[i]+32&&y>=tiles[i+1]&&y<=tiles[i+1]+8)found=true;
    check(found,'every in-box native depth sample remains in a submitted tile');
  }
  // A deterministic collection of real source pixels and rigid camera poses.
  // Every in-box sample must remain in the conservative native window.
  for(let pose=0;pose<12;pose++) {
    const a=pose*.39,c=Math.cos(a),s=Math.sin(a),m=[c,0,-s,0,0,1,0,0,s,0,c,0,pose*.03,-.2,.1,1];
    const sensor={...camera,modelMatrix:m};
    for(let sample=0;sample<45;sample++) {
      const x=(sample*127+pose*47)%1280,y=(sample*71+pose*13)%720,z=.3+(sample%11)*.12;
      const local=[(x-639.5)*z/640,-(y-359.5)*z/640,-z];
      const world=[m[0]*local[0]+m[8]*local[2]+m[12],local[1]+m[13],m[2]*local[0]+m[10]*local[2]+m[14]];
      const box={min:world.map(v=>v-.015),max:world.map(v=>v+.015)};
      const r=workspaceSourceWindow(sensor,box);
      check(x>=r.x&&x<r.x+r.width&&y>=r.y&&y<r.y+r.height,'rotated camera retains every in-box native sample');
    }
  }
  process.stdout.write(`${checks} workspace crop checks passed\n`);
})().catch(error=>{console.error(error);process.exitCode=1;});
