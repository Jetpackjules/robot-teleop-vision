// Conservative tile rejection only: no point sampling, simplification or LOD.
import {workspaceDepthTiles,workspaceVisibleTiles} from './workspace_crop.js?v=20261021_original_rgb';

const cache=new WeakMap();
export function visibleNativeTiles(camera,viewProjection,bounds=null,pointPadding=[0,0]){
  if(!camera.depth||!camera.modelMatrix||!camera.intrinsicsArray)return null;
  const revision=camera.uploadRevision??camera.lastUpdateMs;
  let entry=cache.get(camera);
  if(!entry||entry.depth!==camera.depth||entry.revision!==revision){
    entry={depth:camera.depth,revision,ranges:camera.depthTiles||workspaceDepthTiles(camera.depth,camera.width,camera.height)};
    cache.set(camera,entry);
  }
  const key=`${viewProjection}:${camera.modelMatrix}:${camera.intrinsicsArray}:${bounds?.key||''}:${pointPadding}`;
  if(entry.key===key)return entry.tiles;
  const m=camera.modelMatrix,p=viewProjection,clip=new Float64Array(16);
  for(let c=0;c<4;c++)for(let r=0;r<4;r++)for(let k=0;k<4;k++)clip[c*4+r]+=p[k*4+r]*m[c*4+k];
  // A tile's native vertices lie inside its pixel cone and measured z range.
  // Transform all eight corners, then reject only when every corner is beyond
  // the same clip plane. Shared triangle edges and point-sprite radii survive.
  const candidates=bounds?workspaceVisibleTiles(camera,bounds):null;
  const columns=Math.ceil(camera.width/32),rows=Math.ceil(camera.height/8),k=camera.intrinsicsArray;
  const values=new Uint16Array((candidates?candidates.length:columns*rows*2));
  let count=0,cellCount=0;
  const visit=(x,y)=>{
    const index=(Math.floor(y/8)*columns+Math.floor(x/32))*2;
    const lo=entry.ranges[index]*.001,hi=entry.ranges[index+1]*.001;if(!hi)return;
    const xs=[(x-k[2])/k[0],(Math.min(camera.width-1,x+32)-k[2])/k[0]];
    const ys=[-(Math.min(camera.height-1,y+8)-k[3])/k[1],-(y-k[3])/k[1]];
    let outside=63;
    for(const z of [lo,hi])for(const a of xs)for(const b of ys){
      const qx=a*z,qy=b*z,qz=-z;
      const cx=clip[0]*qx+clip[4]*qy+clip[8]*qz+clip[12],cy=clip[1]*qx+clip[5]*qy+clip[9]*qz+clip[13];
      const cz=clip[2]*qx+clip[6]*qy+clip[10]*qz+clip[14],cw=clip[3]*qx+clip[7]*qy+clip[11]*qz+clip[15];
      const px=cw*(1+pointPadding[0]),py=cw*(1+pointPadding[1]),epsilon=1e-5;
      let mask=0;if(cx < -px-epsilon)mask|=1;if(cx > px+epsilon)mask|=2;
      if(cy < -py-epsilon)mask|=4;if(cy > py+epsilon)mask|=8;
      if(cz < -cw-epsilon)mask|=16;if(cz > cw+epsilon)mask|=32;
      outside &= mask;
    }
    if(outside)return;
    values[count++]=x;values[count++]=y;
    cellCount+=Math.max(0,Math.min(32,camera.width-1-x))*Math.max(0,Math.min(8,camera.height-1-y));
  };
  if(candidates)for(let i=0;i<candidates.length;i+=2)visit(candidates[i],candidates[i+1]);
  else for(let y=0;y<rows;y++)for(let x=0;x<columns;x++)visit(x*32,y*8);
  // Tiny rejection gains do not repay instancing overhead on integrated GPUs.
  // Retain the ordinary native grid when most cells are still in view.
  entry.key=key;entry.tiles=!bounds&&cellCount>(camera.width-1)*(camera.height-1)*.65?null:values.slice(0,count);
  if(entry.tiles)entry.tiles.cellCount=cellCount;
  return entry.tiles;
}
