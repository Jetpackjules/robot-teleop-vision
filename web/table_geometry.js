// Conservative geometry submission for the modeled table, at native detail.
export const TABLE_CONTEXT_MARGIN = 0.05;
export function volumeCorners(bounds) {
  if(bounds.corners)return bounds.corners;
  return Array.from({length: 8}, (_, corner) => [0, 1, 2].map(axis =>
    (corner & (1 << axis)) ? bounds.max[axis] : bounds.min[axis]));
}

// Conservative source rectangle from the eight box corners. A near-plane
// crossing falls back to the full image rather than dropping valid geometry.
// No depth scan, per-point CPU work, or per-frame buffer rebuild is needed.
export function workspaceSourceWindow(camera, bounds) {
  const full = {x: 0, y: 0, width: camera.width, height: camera.height};
  if (!bounds) return full;
  const m = camera.modelMatrix, f = camera.intrinsicsArray || camera.metadata.intrinsics;
  // Rig poses are rigid. A scaled/sheared third-party transform needs its
  // general inverse; retain the full grid rather than making an unsafe crop.
  if ([0, 4, 8].some(i => Math.abs(m[i] ** 2 + m[i+1] ** 2 + m[i+2] ** 2 - 1) > 0.0001)
      || Math.abs(m[0]*m[4]+m[1]*m[5]+m[2]*m[6]) > 0.0001
      || Math.abs(m[0]*m[8]+m[1]*m[9]+m[2]*m[10]) > 0.0001
      || Math.abs(m[4]*m[8]+m[5]*m[9]+m[6]*m[10]) > 0.0001) return full;
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity, front = 0;
  for (const p of volumeCorners(bounds)) {
    const x = p[0] - m[12], y = p[1] - m[13], z = p[2] - m[14];
    const depth = -(m[8] * x + m[9] * y + m[10] * z);
    if (depth <= 0.001) continue;
    front++;
    const u = (m[0] * x + m[1] * y + m[2] * z) * f[0] / depth + f[2];
    const v = -(m[4] * x + m[5] * y + m[6] * z) * f[1] / depth + f[3];
    minX = Math.min(minX, u); maxX = Math.max(maxX, u);
    minY = Math.min(minY, v); maxY = Math.max(maxY, v);
  }
  if (!front) return {...full, width: 0, height: 0};
  if (front !== 8 || ![minX, maxX, minY, maxY].every(Number.isFinite)) return full;
  // Include triangles crossing the box even when their source corner lies
  // just outside it. Two source pixels also cover floating point boundaries.
  const x = Math.max(0, Math.min(camera.width, Math.floor(minX) - 2));
  const y = Math.max(0, Math.min(camera.height, Math.floor(minY) - 2));
  const right = Math.max(0, Math.min(camera.width, Math.ceil(maxX) + 3));
  const bottom = Math.max(0, Math.min(camera.height, Math.ceil(maxY) + 3));
  return {x, y, width: Math.max(0, right - x), height: Math.max(0, bottom - y)};
}

// Exact min/max of every native vertex in a tile, including the shared edge.
// This can run in the existing decode worker; zero samples remain invalid.
export function workspaceDepthTiles(depth, width, height) {
  const columns=Math.ceil(width/32), rows=Math.ceil(height/8), ranges=new Uint16Array(columns*rows*2);
  for(let by=0;by<rows;by++) for(let bx=0;bx<columns;bx++) {
    let lo=65535,hi=0;
    const right=Math.min(width,bx*32+33),bottom=Math.min(height,by*8+9);
    for(let y=by*8;y<bottom;y++) for(let x=bx*32;x<right;x++) {
      const value=depth[y*width+x];
      if(value) {if(value<lo)lo=value;if(value>hi)hi=value;}
    }
    const i=(by*columns+bx)*2;ranges[i]=hi?lo:0;ranges[i+1]=hi;
  }
  return ranges;
}

const tileCache=new WeakMap();
const denseTileCache=new WeakMap();
function cameraTileCache(camera) {
  const revision=camera.uploadRevision??camera.lastUpdateMs;
  let cache=tileCache.get(camera);
  if(!cache||cache.depth!==camera.depth||cache.revision!==revision) {
    cache={depth:camera.depth,revision,ranges:camera.depthTiles||workspaceDepthTiles(camera.depth,camera.width,camera.height)};
    tileCache.set(camera,cache);
  }
  return cache;
}

// Visibility must retain occluders outside the displayed workspace. Only skip
// tiles with no measured corner at all, including their shared triangle edge.
export function nonemptySourceTiles(camera) {
  // Uncropped frames need no synchronous scan on the UI thread. Quality
  // workers supply ranges; without them use the exact regular tile grid.
  if(!camera.depthTiles&&!camera.metadata?.stream_depth_origin) {
    let dense=denseTileCache.get(camera);
    if(dense&&dense.width===camera.width&&dense.height===camera.height)return dense.tiles;
    const values=[];
    for(let y=0;y<camera.height-1;y+=8)for(let x=0;x<camera.width-1;x+=32)values.push(x,y);
    dense={width:camera.width,height:camera.height,tiles:new Uint16Array(values)};
    denseTileCache.set(camera,dense);return dense.tiles;
  }
  const cache=camera.depth?cameraTileCache(camera):null;
  if(cache?.nonempty)return cache.nonempty;
  const columns=Math.ceil(camera.width/32),rows=Math.ceil(camera.height/8);
  const values=new Uint16Array(columns*rows*2);let count=0;
  for(let y=0;y<rows;y++)for(let x=0;x<columns;x++) {
    if(cache&&!cache.ranges[(y*columns+x)*2+1])continue;
    values[count++]=x*32;values[count++]=y*8;
  }
  const result=values.slice(0,count);
  if(cache)cache.nonempty=result;
  return result;
}

export function workspaceVisibleTiles(camera,bounds) {
  if(!bounds || !camera.depth) return null;
  const cache=cameraTileCache(camera);
  const key=`${bounds.key||`${bounds.min}:${bounds.max}`}:${camera.modelMatrix}:${camera.intrinsicsArray}`;
  if(cache.key===key)return cache.visible;
  const columns=Math.ceil(camera.width/32),rows=Math.ceil(camera.height/8);
  const values=new Uint16Array(columns*rows*2),m=camera.modelMatrix,k=camera.intrinsicsArray;
  let count=0,cellCount=0;
  for(let by=0;by<rows;by++) for(let bx=0;bx<columns;bx++) {
    const index=(by*columns+bx)*2,zlo=cache.ranges[index]*.001,zhi=cache.ranges[index+1]*.001;
    if(!zhi)continue;
    const x=bx*32,y=by*8;
    const xmin=(x-k[2])/k[0],xmax=(Math.min(camera.width-1,x+32)-k[2])/k[0];
    const ymin=-(Math.min(camera.height-1,y+8)-k[3])/k[1],ymax=-(y-k[3])/k[1];
    let keep=true;
    // The cone's world AABB is conservative for all native points and every
    // triangle between them. Test the interval; never infer a surface or fill.
    for(let axis=0;axis<3;axis++) {
      const a=m[axis],b=m[axis+4];
      const lo=(a>=0?a*xmin:a*xmax)+(b>=0?b*ymin:b*ymax)-m[axis+8];
      const hi=(a>=0?a*xmax:a*xmin)+(b>=0?b*ymax:b*ymin)-m[axis+8];
      const worldLo=m[axis+12]+lo*(lo>=0?zlo:zhi),worldHi=m[axis+12]+hi*(hi>=0?zhi:zlo);
      if(worldLo>bounds.max[axis]||worldHi<bounds.min[axis]){keep=false;break;}
    }
    if(keep){values[count++]=x;values[count++]=y;cellCount+=Math.max(0,Math.min(32,camera.width-1-x))*Math.max(0,Math.min(8,camera.height-1-y));}
  }
  cache.key=key;cache.visible=values.slice(0,count);
  cache.visible.cellCount=cellCount;
  return cache.visible;
}

export function bindWorkspaceTiles(gl,tiles,entry={}) {
  // WebGL requires an active per-vertex attribute alongside the tile's
  // instanced attribute. The tiny identity stream supplies the exact native
  // vertex ID for both indexed and non-indexed tiles (no geometry reduction).
  if(!entry.vertexBuffer) {
    entry.vertexBuffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,entry.vertexBuffer);
    gl.bufferData(gl.ARRAY_BUFFER,Uint16Array.from({length:32*8*6},(_,i)=>i),gl.STATIC_DRAW);
  } else gl.bindBuffer(gl.ARRAY_BUFFER,entry.vertexBuffer);
  gl.enableVertexAttribArray(1);gl.vertexAttribIPointer(1,1,gl.UNSIGNED_SHORT,0,0);gl.vertexAttribDivisor(1,0);
  if(!entry.buffer)entry.buffer=gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER,entry.buffer);
  if(entry.tiles!==tiles){gl.bufferData(gl.ARRAY_BUFFER,tiles,gl.DYNAMIC_DRAW);entry.tiles=tiles;}
  gl.enableVertexAttribArray(0);gl.vertexAttribIPointer(0,2,gl.UNSIGNED_SHORT,0,0);gl.vertexAttribDivisor(0,1);
  return entry;
}

export function unbindWorkspaceTiles(gl) {
  gl.vertexAttribDivisor(0,0);gl.disableVertexAttribArray(0);
  gl.disableVertexAttribArray(1);
}
