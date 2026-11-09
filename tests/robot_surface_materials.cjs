const assert = require('node:assert/strict');
const {readFileSync} = require('node:fs');
const {resolve} = require('node:path');
(async()=>{
  const source=readFileSync(resolve(__dirname,'../web/robot_surface_materials.js'),'utf8');
  const {triangleNormals,robotSurfaceMaterial}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
  const positions=Float32Array.from([0,0,0, 1,0,0, 0,1,0]),original=positions.slice();
  assert.deepEqual(Array.from(triangleNormals(positions)),[0,0,1,0,0,1,0,0,1]);
  assert.deepEqual(Array.from(triangleNormals(positions,Uint32Array.from([0,2,1]))),[0,0,-1,0,0,-1,0,0,-1]);
  assert.deepEqual(positions,original,'normal generation does not modify calibrated geometry');
  assert(triangleNormals(new Float32Array(9)).every(Number.isFinite),'degenerate facets remain finite');
  const material={color:[1,209/255,31/255,1],roughness:.6,metalness:0};
  const palette=[{sourceColor:material.color.slice(0,3),color:[.19,.2,.215,1],roughness:.78,detail:1}];
  const mapped=robotSurfaceMaterial(material,palette);
  assert.deepEqual(mapped.color,palette[0].color);assert.equal(mapped.roughness,.78);
  assert.equal(material.color[0],1,'source material remains untouched');
  assert.equal(robotSurfaceMaterial(material),material,'other robots preserve their imported material');
  console.log('PASS robot surface normals, palette and unchanged geometry');
})().catch(error=>{console.error(error);process.exitCode=1;});
