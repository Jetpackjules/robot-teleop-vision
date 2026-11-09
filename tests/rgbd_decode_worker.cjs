const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {pathToFileURL} = require('node:url');
const zlib = require('node:zlib');
const load = async name => {
  const file = path.join(__dirname, '../web', name);
  const geomUrl='data:text/javascript;base64,'+Buffer.from(fs.readFileSync(path.join(__dirname,'../web/workspace_transform.js'),'utf8')).toString('base64');
  const cropSource=fs.readFileSync(path.join(__dirname,'../web/table_geometry.js'),'utf8').replace(/'\.\/workspace_transform.js[^']*'/,JSON.stringify(geomUrl));
  const cropUrl='data:text/javascript;base64,'+Buffer.from(cropSource).toString('base64');
  const projectionUrl='data:text/javascript;base64,'+Buffer.from(fs.readFileSync(path.join(__dirname,'../web/hybrid_color_projection.js'),'utf8')).toString('base64');
  const jpegUrl='data:text/javascript;base64,'+Buffer.from(fs.readFileSync(path.join(__dirname,'../web/jpeg_rgba_decoder.js'),'utf8')
    .replaceAll('import.meta.url',JSON.stringify(pathToFileURL(path.join(__dirname,'../web/jpeg_rgba_decoder.js')).href))).toString('base64');
  const samplingUrl='data:text/javascript;base64,'+Buffer.from(fs.readFileSync(path.join(__dirname,'../web/point_preview_sampling.js'),'utf8')).toString('base64');
  const source = fs.readFileSync(file, 'utf8').replaceAll('import.meta.url', JSON.stringify(pathToFileURL(file).href))
    .replace(/'\.\/hybrid_color_projection\.js\?[^']+'/,JSON.stringify(projectionUrl))
    .replace(/'\.\/jpeg_rgba_decoder\.js\?[^']+'/,JSON.stringify(jpegUrl))
    .replace(/'\.\/table_geometry\.js\?[^']+'/,JSON.stringify(cropUrl))
    .replace(/'\.\/point_preview_sampling\.js\?[^']+'/,JSON.stringify(samplingUrl));
  return import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));
};
(async () => {
  const {colorFrameDimensions}=await load('hybrid_color_projection.js');
  assert.deepEqual(colorFrameDimensions({width:1280,height:720}),[1280,720]);
  const projection={model:'pinhole',size:[1920,1080],intrinsics:[1300,1300,959.5,539.5],extrinsics:[1,0,0,0,1,0,0,0,1,.015,0,0]};
  assert.deepEqual(colorFrameDimensions({width:1280,height:720,color_space:'original_rgb',color_width:1920,color_height:1080,color_projection:projection}),[1920,1080]);
  assert.throws(()=>colorFrameDimensions({width:1280,height:720,color_space:'original_rgb',color_width:1280,color_height:720,color_projection:projection}),/matching calibrated/);
  assert.throws(()=>colorFrameDimensions({width:1280,height:720,color_space:'original_rgb',color_width:1920,color_height:1080}),/matching calibrated/);
  const {decodeIndependentDepth} = await load('rgbd_decode_worker.js');
  const {decodeSparsePoints}=await load('sparse_point_decode_worker.js');
  const allCount=45000,allBlock=new ArrayBuffer(allCount*9),allIndices=new Uint32Array(allBlock,0,allCount),allDepths=new Uint16Array(allBlock,allCount*4,allCount),allColors=new Uint8Array(allBlock,allCount*6,allCount*3);
  for(let i=0;i<allCount;i++){allIndices[i]=i*2;allDepths[i]=400+i%1000;allColors.set([i%256,77,133],i*3);}
  const allMetadata={width:300,height:300,preview_count:allCount,preview_source_points:allCount,preview_budget:0};
  const allCompressed=zlib.deflateSync(new Uint8Array(allBlock)),allDecoded=await decodeSparsePoints(allCompressed,allMetadata);
  assert.deepEqual(allDecoded.previewSamples.indices,allIndices,'All preserves every selected native index beyond 40k');
  assert.deepEqual(allDecoded.previewSamples.depths,allDepths);
  assert.deepEqual(allDecoded.previewSamples.colors,allColors);
  await assert.rejects(decodeSparsePoints(allCompressed,{...allMetadata,preview_source_points:allCount+1}),/dimensions/,'All cannot silently omit source points');
  const cleanedMetadata={...allMetadata,preview_source_points:allCount+20,preview_cleanup_enabled:true,
    preview_cleanup_kind:'selected_radius_v1',preview_cleanup_input_points:allCount+20,preview_cleanup_removed:20};
  const cleanedDecoded=await decodeSparsePoints(allCompressed,cleanedMetadata);
  assert.deepEqual(cleanedDecoded.previewSamples.indices,allIndices,'explicit post-selection cleanup is accepted in All mode');
  await assert.rejects(decodeSparsePoints(allCompressed,{...cleanedMetadata,preview_cleanup_removed:19}),/dimensions/);
  allDepths.set([65535,1,32768,65534]);
  const shuffled=new Uint8Array(allCount*9);
  let previousIndex=0,previousZ=0,previousRedGreen=0,previousGreen=0,previousBlueGreen=0;
  for(let i=0;i<allCount;i++){
    const delta=(allIndices[i]-previousIndex)>>>0,signed=(allDepths[i]-previousZ)<<16>>16,zigzag=((signed<<1)^(signed>>15))&65535;
    for(let b=0;b<4;b++)shuffled[allCount*b+i]=delta>>>(8*b);
    shuffled[allCount*4+i]=zigzag;shuffled[allCount*5+i]=zigzag>>>8;
    const green=allColors[i*3+1],redGreen=(allColors[i*3]-green)&255,blueGreen=(allColors[i*3+2]-green)&255;
    shuffled[allCount*6+i]=redGreen-previousRedGreen;shuffled[allCount*7+i]=green-previousGreen;shuffled[allCount*8+i]=blueGreen-previousBlueGreen;
    previousIndex=allIndices[i];previousZ=allDepths[i];previousRedGreen=redGreen;previousGreen=green;previousBlueGreen=blueGreen;
  }
  const shuffledDecoded=await decodeSparsePoints(zlib.deflateSync(shuffled),{...allMetadata,preview_predictor:'shuffle_delta_v2'});
  assert.deepEqual(shuffledDecoded.previewSamples.indices,allIndices);
  assert.deepEqual(shuffledDecoded.previewSamples.depths,allDepths,'signed depth deltas preserve wraparound and full millimetre precision');
  assert.deepEqual(shuffledDecoded.previewSamples.colors,allColors,'reversible green differences preserve all RGB8 bytes');
  const {NativeRgbdDecoderPool, isIndependentNativeFrame, takeIndependentFrameBatch} = await load('rgbd_decode_pool.js');
  const width = 848, height = 480, values = new Uint16Array(width * height);
  for (let i = 0; i < values.length; i++) values[i] = i % 17 === 0 ? 0 : (i * 137) & 65535;
  const predicted = new Uint8Array(values.byteLength);
  for (let i = 0; i < values.length; i++) {
    const difference = (values[i] - (i >= width ? values[i - width] : 0)) << 16 >> 16;
    const zigzag = ((difference << 1) ^ (difference >> 31)) & 65535;
    predicted[i] = zigzag & 255;predicted[i + values.length] = zigzag >> 8;
  }
  for (const [encoding, raw] of [['u16_mm_deflate', new Uint8Array(values.buffer)],
      ['u16_mm_vpredict_shuffle_deflate', predicted]]) {
    const metadata = {width, height, depth_encoding: encoding};
    const compressed = zlib.deflateSync(raw);
    const result = await decodeIndependentDepth(compressed, metadata);
    assert.deepEqual(result, values, 'full native grid including zeros and modulo boundaries is lossless');
    await assert.rejects(decodeIndependentDepth(zlib.deflateSync(raw.subarray(2)), metadata), /length/);
  }
  await assert.rejects(decodeIndependentDepth(new Uint8Array(), {width: -1, height}), /dimensions/);
  assert.equal(isIndependentNativeFrame({depth_encoding: 'u16_mm_keyframe_tiles_deflate'}), false);
  assert.equal(isIndependentNativeFrame({depth_temporal_keyframe: true}), false);
  assert.equal(isIndependentNativeFrame({color_temporal_reference_mode: 'persistent_v1'}), false);
  assert.equal(isIndependentNativeFrame({depth_encoding: 'u16_mm_vpredict_shuffle_deflate'}), true);
  const frame = (id, sequence) => ({receivedAtMs: sequence, metadata: {cameras: [{id, sequence}]},
    payloads: [{metadata: {id, sequence, depth_encoding: 'u16_mm_vpredict_shuffle_deflate'}}]});
  const queue = [frame('B', 1), frame('A', 2), frame('C', 3), frame('A', 4)];
  const batch = takeIndependentFrameBatch(queue);
  assert.deepEqual(batch.payloads.map(p => [p.metadata.id, p.metadata.sequence]), [['B', 1], ['A', 4], ['C', 3]],
    'coalescing keeps the newest capture of every camera, including asynchronous B/C updates');
  assert.equal(batch.coalescedPackets, 3);assert.equal(queue.length, 0);
  const keyframe = frame('A', 5);keyframe.payloads[0].metadata.depth_temporal_keyframe = true;
  const dependentQueue = [frame('C', 4), keyframe, frame('B', 6)];
  assert.equal(takeIndependentFrameBatch(dependentQueue).payloads.length, 1);
  assert.equal(takeIndependentFrameBatch(dependentQueue), keyframe, 'named keyframes never coalesce');
  const compactFrame=(id,sequence,age=0)=>{
    const value=frame(id,sequence);value.receivedAtMs=performance.now()-age;
    value.payloads[0].metadata.depth_encoding='u16_mm_sparse_points_deflate';return value;
  };
  const burst=[compactFrame('A',1),compactFrame('A',2),compactFrame('A',3),compactFrame('B',1),compactFrame('C',1)];
  assert.deepEqual(takeIndependentFrameBatch(burst).payloads.map(p=>[p.metadata.id,p.metadata.sequence]),[['A',1],['B',1],['C',1]],'a short burst preserves each camera for separate draws');
  assert.equal(takeIndependentFrameBatch(burst).payloads[0].metadata.sequence,2);
  assert.equal(takeIndependentFrameBatch(burst).payloads[0].metadata.sequence,3);
  const stale=[compactFrame('A',1,100),compactFrame('A',2)];
  assert.equal(takeIndependentFrameBatch(stale).payloads[0].metadata.sequence,2,'old buffered frames cannot accumulate latency');
  const overfull=Array.from({length:10},(_,i)=>compactFrame('A',i+1));
  assert.equal(takeIndependentFrameBatch(overfull).payloads[0].metadata.sequence,8,'only three captures per camera remain buffered');
  assert.equal(overfull.length,2);
  let workers = [], closed = 0;
  global.Worker = class {
    constructor() {workers.push(this);}
    postMessage(message, transfers) {this.message = structuredClone(message, {transfer: transfers});}
    terminate() {this.terminated = true;}
  };
  const pool = new NativeRgbdDecoderPool();
  const payload = () => ({metadata: {width: 2, height: 1}, color: new Uint8Array(2), depth: new Uint8Array(4)});
  const first = payload(), result = pool.decode(first);
  assert.equal(first.depth.byteLength, 0, 'compressed inputs transfer without a main-thread copy');
  workers[0].onmessage({data: {id: 1, depth: new Uint16Array([0, 65535]).buffer,
    depthTiles:new Uint16Array([65535,65535]).buffer, bitmap: {close() {closed++;}}}});
  const transferred=await result;
  assert.deepEqual(transferred.depth, new Uint16Array([0, 65535]));
  assert.deepEqual(transferred.depthTiles,new Uint16Array([65535,65535]),'conservative native tile ranges survive the worker handoff');
  const stopped = pool.decode(payload());
  pool.close();
  await assert.rejects(stopped, /stopped/);
  assert.equal(pool.pending.size, 0);
  assert(workers.every(worker => worker.terminated));
  workers[0].onmessage({data: {id: 1, bitmap: {close() {closed++;}}}});
  assert.equal(closed, 1, 'late results release their bitmap after shutdown');
  assert.equal(pool.tryRecover(Infinity),false,'an explicitly closed pool must stay closed');
  const recovering=new NativeRgbdDecoderPool();
  const pending=recovering.decode(payload());
  const failedWorker=workers.at(-1);failedWorker.onerror();
  await assert.rejects(pending,/worker failed/);
  assert.equal(recovering.tryRecover(recovering.retryAt-1),false,'failure cooldown keeps fallback available');
  assert.equal(recovering.tryRecover(recovering.retryAt),true,'a transient worker failure can recover');
  const recovered=recovering.decode(payload()),replacement=workers.at(-1);
  failedWorker.onerror();
  assert.equal(recovering.disabled,false,'late failure events from a replaced worker do not kill its replacement');
  replacement.onmessage({data:{id:replacement.message.id,depth:new Uint16Array([9,12]).buffer}});
  assert.deepEqual((await recovered).depth,new Uint16Array([9,12]),'fresh frames decode after recovery');
  assert.equal(recovering.restarts,1);
  assert.equal(recovering.recoveryAttempts,0,'successful decoding clears consecutive failure backoff');
  recovering.close();
  const bounded=new NativeRgbdDecoderPool();
  for(let index=0;index<3;index++){
    bounded.fail(new Error('repeated failure'));
    assert.equal(bounded.tryRecover(bounded.retryAt),true);
  }
  bounded.fail(new Error('permanent failure'));
  assert.equal(bounded.tryRecover(Infinity),false,'repeated failures have a bounded retry count');
  const renderer = fs.readFileSync(path.join(__dirname, '../web/hybrid_rgbd_renderer.js'), 'utf8');
  const prepareSource = renderer.slice(renderer.indexOf('function beginFrameDecode('),renderer.indexOf('async function decodeNewestPackets('));
  let finishDecode, released = 0;
  const stageState = {running:true,robotLoadToken:1};
  const prepare = new Function('state','decodeCamera',prepareSource+';return beginFrameDecode;')(
    stageState,()=>new Promise(resolve=>{finishDecode=()=>resolve({bitmap:{close(){released++;}}});}));
  const stagedFrame = {payloads:[{}],receivedAtMs:performance.now()};
  const canceled = prepare(stagedFrame);canceled.cancel();finishDecode();await canceled.promise;
  assert.equal(released,1,'canceling an in-flight prefetched frame releases its eventual bitmap');
  const completed = prepare(stagedFrame);finishDecode();await completed.promise;
  assert.equal(released,1,'completed current-generation data remains available for upload');
  completed.cancel();assert.equal(released,2,'stopping also releases an already-completed prefetched frame');
  const oldGeneration = prepare(stagedFrame);stageState.robotLoadToken++;finishDecode();await oldGeneration.promise;
  assert.equal(released,3,'a restarted renderer discards old-generation prefetched resources');
  console.log('Native depth losslessness, validation, transfer and worker lifecycle checks passed');
})().catch(error => {console.error(error);process.exitCode = 1;});
