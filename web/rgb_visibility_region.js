// Restrict color-visibility work to RGB pixels that retained world points can
// sample. Occluders outside the world crop still participate in this region.
// SPDX-License-Identifier: EUPL-1.2
import {workspaceCropCorners} from './workspace_crop.js?v=20261021_original_rgb';
export function rgbVisibilityRegion(bounds,matrix,width,height){
  const full={x:0,y:0,width,height};if(!bounds)return full;
  if(bounds.min?.some((v,i)=>v>=bounds.max[i]))return {...full,width:0,height:0};
  let loX=Infinity,loY=Infinity,hiX=-Infinity,hiY=-Infinity,front=0;
  for(const p of workspaceCropCorners(bounds)){
    const clip=[0,1,2,3].map(r=>matrix[r]*p[0]+matrix[r+4]*p[1]+matrix[r+8]*p[2]+matrix[r+12]);
    if(!clip.every(Number.isFinite))return full;
    if(clip[3]<=.05)continue;front++;
    const x=(clip[0]/clip[3]+1)*width/2,y=(clip[1]/clip[3]+1)*height/2;
    loX=Math.min(loX,x);hiX=Math.max(hiX,x);loY=Math.min(loY,y);hiY=Math.max(hiY,y);
  }
  // A crossing near the optical plane needs full coverage.
  if(front!==8)return full;
  const x=Math.max(0,Math.min(width,Math.floor(loX)-3)),y=Math.max(0,Math.min(height,Math.floor(loY)-3));
  const right=Math.max(0,Math.min(width,Math.ceil(hiX)+4)),top=Math.max(0,Math.min(height,Math.ceil(hiY)+4));
  return {x,y,width:Math.max(0,right-x),height:Math.max(0,top-y)};
}
export function regionContains(outer,inner){
  return !inner.width||!inner.height||!outer||outer.x<=inner.x&&outer.y<=inner.y
    &&outer.x+outer.width>=inner.x+inner.width&&outer.y+outer.height>=inner.y+inner.height;
}
export function rgbRegionClip(matrix,region,width,height){
  if(region.x===0&&region.y===0&&region.width===width&&region.height===height)return matrix;
  const result=new Float32Array(matrix),sx=region.width/width,sy=region.height/height;
  const cx=(region.x*2+region.width)/width-1,cy=(region.y*2+region.height)/height-1;
  for(let c=0;c<4;c++){
    result[c*4]=(matrix[c*4]-cx*matrix[c*4+3])/sx;
    result[c*4+1]=(matrix[c*4+1]-cy*matrix[c*4+3])/sy;
  }return result;
}
