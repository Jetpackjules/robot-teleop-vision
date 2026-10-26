// Conservative tile rejection only: no point sampling, simplification or LOD.
import {workspaceDepthTiles,workspaceVisibleTiles} from './workspace_crop.js?v=20261021_original_rgb';

const cache=new WeakMap();
const sourceTiles=new WeakMap();
export function nativeSourceTiles(camera){
  let entry=sourceTiles.get(camera);
  const ranges=camera.depthTiles,revision=ranges?(camera.uploadRevision??camera.lastUpdateMs):null;
  if(entry?.width===camera.width&&entry.height===camera.height&&entry.ranges===ranges&&entry.revision===revision)return entry.tiles;
  const columns=Math.ceil(camera.width/32),values=[];
  for(let y=0;y<camera.height-1;y+=8)for(let x=0;x<camera.width-1;x+=32){
    if(ranges&&!ranges[(Math.floor(y/8)*columns+Math.floor(x/32))*2+1])continue;
    values.push(x,y);
  }
  entry={width:camera.width,height:camera.height,ranges,revision,tiles:new Uint16Array(values)};sourceTiles.set(camera,entry);return entry.tiles;
}
export function visibleNativeTiles(camera,viewProjection,bounds=null,pointPadding=[0,0],sourceClip=false){
  if(!camera.depth||!camera.modelMatrix||!camera.intrinsicsArray)return null;
  const revision=camera.uploadRevision??camera.lastUpdateMs;
  let entry=cache.get(camera);
  if(!entry){entry={optical:new Map()};cache.set(camera,entry);}
  if(entry.depth!==camera.depth||entry.revision!==revision){
    entry.depth=camera.depth;entry.revision=revision;
    entry.ranges=camera.depthTiles||workspaceDepthTiles(camera.depth,camera.width,camera.height);entry.key=null;
  }
  const key=`${viewProjection}:${camera.modelMatrix}:${camera.intrinsicsArray}:${bounds?.key||''}:${pointPadding}:${sourceClip}`;
  if(entry.key===key)return entry.tiles;
  const m=camera.modelMatrix,p=viewProjection,clip=new Float64Array(16);
  if(sourceClip)clip.set(p);
  else for(let c=0;c<4;c++)for(let r=0;r<4;r++)for(let k=0;k<4;k++)clip[c*4+r]+=p[k*4+r]*m[c*4+k];
  // A tile's native vertices lie inside its pixel cone and measured z range.
  // Reject only when every corner is beyond the same clip plane. Shared
  // triangle edges and point-sprite radii survive the interval calculation.
  const candidates=bounds?workspaceVisibleTiles(camera,bounds):null;
  const columns=Math.ceil(camera.width/32),rows=Math.ceil(camera.height/8),k=camera.intrinsicsArray;
  const coneKey=`${camera.width}:${camera.height}:${k}`;
  if(entry.coneKey!==coneKey){
    entry.cones=new Float64Array(columns*rows*4);entry.optical.clear();entry.coneKey=coneKey;
    for(let by=0;by<rows;by++)for(let bx=0;bx<columns;bx++){
      const x=bx*32,y=by*8,index=(by*columns+bx)*4;
      entry.cones.set([(x-k[2])/k[0],(Math.min(camera.width-1,x+32)-k[2])/k[0],
        -(Math.min(camera.height-1,y+8)-k[3])/k[1],-(y-k[3])/k[1]],index);
    }
  }
  const planes=[];
  for(let axis=0;axis<3;axis++)for(const sign of [-1,1]){
    const margin=1+(axis<2?pointPadding[axis]:0);
    planes.push([margin*clip[3]+sign*clip[axis],margin*clip[7]+sign*clip[4+axis],
      margin*clip[11]+sign*clip[8+axis],margin*clip[15]+sign*clip[12+axis]]);
  }
  // Optical projections stay fixed across captures. Their plane slopes
  // depend only on calibration and pixel cones, never on measured depth.
  let slopes=sourceClip?entry.optical.get(key):null;
  if(sourceClip&&!slopes){
    slopes=new Float64Array(columns*rows*6);
    for(let tile=0;tile<columns*rows;tile++)for(let i=0;i<6;i++){
      const plane=planes[i],offset=tile*4,c=entry.cones;
      slopes[tile*6+i]=plane[0]*c[offset+(plane[0]>=0?1:0)]+plane[1]*c[offset+(plane[1]>=0?3:2)]-plane[2];
    }
    if(entry.optical.size>=6)entry.optical.delete(entry.optical.keys().next().value);
    entry.optical.set(key,slopes);
  }
  const values=new Uint16Array((candidates?candidates.length:columns*rows*2));
  let count=0,cellCount=0;
  const visit=(x,y)=>{
    const tile=Math.floor(y/8)*columns+Math.floor(x/32),index=tile*2;
    const lo=entry.ranges[index]*.001,hi=entry.ranges[index+1]*.001;if(!hi)return;
    const offset=tile*4,c=entry.cones;
    // Maximize each homogeneous clip-plane distance over the tile cone.
    // Positive measured depth makes this the exact eight-corner interval,
    // without transforming or allocating eight vectors per tile and target.
    for(let i=0;i<6;i++){
      const plane=planes[i],slope=slopes?slopes[tile*6+i]:
        plane[0]*c[offset+(plane[0]>=0?1:0)]+plane[1]*c[offset+(plane[1]>=0?3:2)]-plane[2];
      if(slope*(slope>=0?hi:lo)+plane[3]<-1e-5)return;
    }
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
