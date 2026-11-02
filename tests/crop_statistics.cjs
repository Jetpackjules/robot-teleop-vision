const assert=require('node:assert/strict'),fs=require('node:fs');
(async()=>{
  const source=fs.readFileSync(require('node:path').resolve(__dirname,'../web/crop_statistics.js'),'utf8');
  const {cropStatistics}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
  const status={streamCrop:{applied:true,input_bytes:1000000,output_bytes:687000,
    source_samples:100000,retained_samples:12000,source_color_pixels:2000000,color_pixels:2000000},
    workspaceCrop:{sourceCells:10000,submittedCells:4200}};
  const rows=Object.fromEntries(cropStatistics(status).rows);
  assert.match(rows['Network bytes'],/68\.7% sent.*31\.3% saved/);
  assert.match(rows['Depth samples'],/12\.0% kept/);
  assert.match(rows['RGB pixel area'],/100\.0% kept/);
  assert.match(rows['3D grid submission'],/42\.0% submitted.*4,200 of 10,000 received grid cells/);
  assert.equal(cropStatistics(status).message,'');
  assert.equal(cropStatistics(status,{is3D:false}).rows.length,0,'RGB mode must not show stale crop counters');
  assert.match(cropStatistics({}).rows[0][1],/Not active/);
  assert.match(cropStatistics({}).rows[1][1],/Waiting/);
  assert.match(cropStatistics({...status,streamCrop:{...status.streamCrop,retained_samples:undefined}}).rows[1][1],/Not reported/);
  assert.match(cropStatistics({...status,streamCrop:{...status.streamCrop,source_samples:0,retained_samples:0}}).rows[1][1],/No valid depth samples/);
  assert.match(cropStatistics({...status,streamCrop:{...status.streamCrop,output_bytes:1100000}}).rows[0][1],/10\.0% more bytes/);
  assert.doesNotMatch(rows['Network bytes'],/visible|screen/);
  console.log('12 crop-statistics checks passed');
})().catch(error=>{console.error(error);process.exitCode=1;});
