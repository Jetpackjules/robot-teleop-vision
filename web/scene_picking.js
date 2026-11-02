// CPU intersection on focus gestures only. The unnormalized local ray retains
// world-distance parameters through affine model transforms, including scale.
export function raycastTriangles(ray,positions,indices=null,{stride=3,mode=4,limit=Infinity}={}) {
  if(!ray||!positions||![4,5,6].includes(mode))return null;
  const count=indices?.length??Math.floor(positions.length/stride);
  const index=i=>(indices?indices[i]:i)*stride;
  const [ox,oy,oz]=ray.origin,[dx,dy,dz]=ray.direction;
  let nearest=limit;
  for(let triangle=0;triangle<count-2;triangle+=mode===4?3:1) {
    const a=index(mode===6?0:triangle),b=index(triangle+1),c=index(triangle+2);
    const ex=positions[b]-positions[a],ey=positions[b+1]-positions[a+1],ez=positions[b+2]-positions[a+2];
    const fx=positions[c]-positions[a],fy=positions[c+1]-positions[a+1],fz=positions[c+2]-positions[a+2];
    const px=dy*fz-dz*fy,py=dz*fx-dx*fz,pz=dx*fy-dy*fx,det=ex*px+ey*py+ez*pz;
    if(Math.abs(det)<1e-12)continue;
    const tx=ox-positions[a],ty=oy-positions[a+1],tz=oz-positions[a+2],u=(tx*px+ty*py+tz*pz)/det;
    if(u < -1e-8 || u > 1+1e-8)continue;
    const qx=ty*ez-tz*ey,qy=tz*ex-tx*ez,qz=tx*ey-ty*ex,v=(dx*qx+dy*qy+dz*qz)/det;
    if(v < -1e-8 || u+v > 1+1e-8)continue;
    const distance=(fx*qx+fy*qy+fz*qz)/det;
    if(distance>1e-6&&distance<nearest)nearest=distance;
  }
  return nearest<limit?{distance:nearest,point:ray.origin.map((v,i)=>v+ray.direction[i]*nearest)}:null;
}

export function raycastRobotPrimitive(ray,primitive,matrix,limit=Infinity) {
  const positions=primitive.cpuPositions;
  if(!ray||!positions)return null;
  const a=matrix.slice(0,3),b=matrix.slice(4,7),c=matrix.slice(8,11);
  const cross=(u,v)=>[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
  const rows=[cross(b,c),cross(c,a),cross(a,b)],det=a.reduce((s,v,i)=>s+v*rows[0][i],0);
  if(Math.abs(det)<1e-12)return null;
  const inverse=v=>rows.map(row=>row.reduce((s,x,i)=>s+x*v[i],0)/det);
  const local={origin:inverse(ray.origin.map((v,i)=>v-matrix[12+i])),direction:inverse(ray.direction)};
  // A broad-phase bound avoids scanning triangles for links the ray misses.
  if(!primitive.pickBounds){
    const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
    for(let i=0;i<positions.length;i++) {const axis=i%3;min[axis]=Math.min(min[axis],positions[i]);max[axis]=Math.max(max[axis],positions[i]);}
    primitive.pickBounds={min,max};
  }
  let near=0,far=limit;
  for(let axis=0;axis<3;axis++) {
    const o=local.origin[axis],d=local.direction[axis],{min,max}=primitive.pickBounds;
    if(Math.abs(d)<1e-12){if(o<min[axis]||o>max[axis])return null;continue;}
    const p=(min[axis]-o)/d,q=(max[axis]-o)/d;
    near=Math.max(near,Math.min(p,q));far=Math.min(far,Math.max(p,q));if(near>far)return null;
  }
  const hit=raycastTriangles(local,positions,primitive.cpuIndices,{mode:primitive.mode??4,limit});
  return hit?{distance:hit.distance,point:ray.origin.map((v,i)=>v+ray.direction[i]*hit.distance)}:null;
}
