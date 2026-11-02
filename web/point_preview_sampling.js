// Optional lossy preview: choose existing nonzero measurements without changing
// their values. Evenly spaced valid ranks avoid fresh random flicker per frame.
export function sampleDepthPoints(depth, budget = 20000) {
  if (!Number.isInteger(budget) || budget < 0 || budget > 40000) throw new Error('invalid point preview budget');
  let valid = 0;
  for (let i = 0; i < depth.length; i++) if (depth[i]) valid++;
  const count = budget===0?valid:Math.min(valid, budget), indices = new Uint32Array(count);
  if (!count) return {indices, valid};
  let rank = 0, selected = 0, target = Math.floor(valid / count * .5);
  for (let i = 0; i < depth.length && selected < count; i++) {
    if (!depth[i]) continue;
    if (rank === target) {
      indices[selected++] = i;
      target = Math.floor((selected + .5) * valid / count);
    }
    rank++;
  }
  return {indices, valid};
}

// Preserve the original picker's grid sampling without rebuilding a depth image.
export function* cameraDepthSamples(camera,stride=1){
  const compact=camera.previewSamples;
  if(compact?.depths){
    for(let i=0;i<compact.indices.length;i++){
      const index=compact.indices[i],x=index%camera.width,y=Math.floor(index/camera.width);
      if(x%stride===0&&y%stride===0)yield [x,y,compact.depths[i]];
    }
  }else if(camera.depth){
    for(let y=0;y<camera.height;y+=stride)for(let x=0;x<camera.width;x+=stride){
      const z=camera.depth[y*camera.width+x];if(z)yield [x,y,z];
    }
  }
}
