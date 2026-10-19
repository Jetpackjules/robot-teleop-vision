const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(require('node:path').join(__dirname,'../web/camera_rgb_viewer.js'),'utf8').replace(/^export /gm,'');
const requests=[],painted=[],closed=[];
const context=vm.createContext({performance,AbortController,URLSearchParams,setTimeout,clearTimeout,
  document:{hidden:false},fetch:(_,options)=>new Promise(resolve=>requests.push({resolve,options})),
  createImageBitmap:async blob=>({width:1280,height:720,id:blob.id,close(){closed.push(this.id);}})});
vm.runInContext(source+'\nthis.Viewer=CameraRgbViewer;',context);
const canvas={width:0,height:0,getContext:()=>({drawImage:b=>painted.push(b.id)})};
const viewer=new context.Viewer(canvas);viewer.running=true;viewer.generation=1;
const response=id=>({ok:true,headers:{get:name=>({'Content-Type':'image/jpeg','X-Camera-Id':'1234',
  'X-Frame-Sequence':String(id),'X-Capture-Unix-Ms':String(Date.now())}[name]||null)},blob:async()=>({size:100,id})});
(async()=>{
  const older=viewer.fetchFrame(1),newer=viewer.fetchFrame(1);
  requests[1].resolve(response(20));await newer;
  requests[0].resolve(response(10));await older;
  assert.deepEqual(painted,[20],'out-of-order response must not overwrite newest image');
  assert.equal(canvas.width,1280);assert.equal(canvas.height,720);
  assert.equal(viewer.status().decoded,1,'duplicate/old captures do not inflate FPS');
  const pending=viewer.fetchFrame(1);viewer.stop();
  assert.ok(requests[2].options.signal.aborted,'mode exit aborts requests');
  requests[2].resolve(response(30));await pending;
  assert.deepEqual(painted,[20],'late responses cannot repaint after mode exit');
  assert.deepEqual(closed,[20],'decoded image is released');
  console.log('Original RGB viewer: ordering, native size, FPS, cancellation and disposal passed');
})().catch(error=>{console.error(error);process.exitCode=1;});
