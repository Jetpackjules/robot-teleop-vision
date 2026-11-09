const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
(async()=>{
  const source=fs.readFileSync(path.join(__dirname,'../web/point_preview_sampling.js'),'utf8');
  const {sampleDepthPoints}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
  for(const length of [0,1,19999,20000,20001,921600]){
    const depth=new Uint16Array(length);for(let i=0;i<length;i++)depth[i]=i%7?1000+i%250:0;
    const before=depth.slice(),result=sampleDepthPoints(depth),again=sampleDepthPoints(depth);
    const valid=depth.reduce((n,z)=>n+Boolean(z),0);
    assert.equal(result.valid,valid);assert.equal(result.indices.length,Math.min(valid,20000));
    assert.equal(new Set(result.indices).size,result.indices.length);
    assert(result.indices.every(i=>i<length&&depth[i]>0));
    assert.deepEqual(result.indices,again.indices,'fixed measurements have stable selection');
    assert.deepEqual(depth,before,'sampling cannot modify source depth');
    if(result.indices.length>2)assert(result.indices.at(-1)>length*.9,'samples span the source image');
  }
  assert.equal(sampleDepthPoints(new Uint16Array(50000)).indices.length,0);
  assert.equal(sampleDepthPoints(new Uint16Array(100000).fill(1000),40000).indices.length,40000);
  const allDepth=new Uint16Array([0,9,10,0,13,7]);
  assert.deepEqual(sampleDepthPoints(allDepth,0),{indices:new Uint32Array([1,2,4,5]),valid:4},'zero budget selects every valid native sample');
  assert.equal(sampleDepthPoints(new Uint16Array(100000).fill(1000),0).indices.length,100000,'All has no 40k cap');
  assert.throws(()=>sampleDepthPoints(allDepth,-1));
  assert.throws(()=>sampleDepthPoints(new Uint16Array(2),40001));
  console.log('PASS stable capped selection, sparse/empty frames, unique valid indices and unchanged depth');
})().catch(e=>{console.error(e);process.exitCode=1;});
