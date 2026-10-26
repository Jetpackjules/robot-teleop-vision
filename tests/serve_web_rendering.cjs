// Camera-free browser test page. The probe is injected only by this local server.
const http = require('node:http');
const {readFileSync,existsSync} = require('node:fs');
const {resolve} = require('node:path');
const vm=require('node:vm');
const root = resolve(__dirname,'..');
const modules = new Set(['hybrid_rgbd_renderer.js','hybrid_camera_fusion.js','hybrid_camera_quality.js','hybrid_blendpcr.js','camera_rgb_viewer.js','hybrid_color_projection.js','depth_shading.js','workspace_crop.js','modeled_workspace.js','rgbd_decode_pool.js','rgbd_decode_worker.js']);
function createTestServer() {
  return http.createServer((req,res) => {
    const path = new URL(req.url,'http://localhost').pathname;
    res.setHeader('Cache-Control','no-store');
    if(path==='/workspace-readiness-fixture.json') {
      res.setHeader('Content-Type','application/json');res.end(readFileSync(resolve(root,'.teleop/offline-ready-20261021/fixture.json')));
    } else if(path==='/jpeg_i420_rgba.wasm') {
      res.setHeader('Content-Type','application/wasm');res.end(readFileSync(resolve(root,'web/jpeg_i420_rgba.wasm')));
    } else if(path==='/so101-renderer.json') {
      const source=readFileSync(resolve(root,'robot_modules/so101/web/module.js'),'utf8').split('const MODULE_STYLE')[0];
      res.setHeader('Content-Type','application/json');res.end(JSON.stringify(vm.runInNewContext(source+'\nRENDERER')));
    } else if(/^\/robot-modules\/so101\/assets\/[a-z0-9_]+\.glb$/.test(path)) {
      res.setHeader('Content-Type','model/gltf-binary');res.end(readFileSync(resolve(root,'robot_modules/so101/assets',path.split('/').pop())));
    } else if(path==='/api/v1/robot-module') {
      res.setHeader('Content-Type','application/json');res.end(JSON.stringify({id:'disabled',label:'Vision only',entrypoints:{}}));
    } else if(path==='/api/v1/site-settings') {
      res.setHeader('Content-Type','application/json');res.end('{}');
    } else if(path.startsWith('/api/')||path.startsWith('/capture-')) {
      res.setHeader('Content-Type','application/json');res.writeHead(404);res.end('{}');
    } else if(path==='/rgb-visibility') {
      res.setHeader('Content-Type','text/html');res.end(readFileSync(resolve(__dirname,'browser_rgb_visibility_probe.html')));
    } else if(path==='/renderer.js') {
      res.setHeader('Content-Type','text/javascript');
      res.end(readFileSync(resolve(root,'web/hybrid_rgbd_renderer.js'),'utf8')+
        '\nexport {state,uploadCamera,identityMatrix,inverseRigidTransform};');
    } else if(path==='/controller.html') {
      res.setHeader('Content-Type','text/html');res.end(readFileSync(resolve(root,'web/controller.html')));
    } else if(modules.has(path.slice(1))||(/^\/[\w-]+\.js$/.test(path)&&existsSync(resolve(root,'web',path.slice(1))))) {
      res.setHeader('Content-Type','text/javascript');
      let source=readFileSync(resolve(root,'web',path.slice(1)),'utf8');
      if(path==='/hybrid_rgbd_renderer.js')source+='\nwindow.probe={state,createProgram,findUniforms,render,uploadCamera,decodeCamera,identityMatrix,inverseRigidTransform,perspectiveMatrix,multiplyMatrix,deleteCameraTextures,loadRobotMeshes,findRobotUniforms,ROBOT_VERTEX_SHADER,ROBOT_FRAGMENT_SHADER,parsePacket};';
      res.end(source);
    } else if(path==='/web_rendering_fixture.js') {
      res.setHeader('Content-Type','text/javascript');res.end(readFileSync(resolve(__dirname,'web_rendering_fixture.js')));
    } else if(path==='/workspace_readiness_fixture.js') {
      res.setHeader('Content-Type','text/javascript');res.end(readFileSync(resolve(__dirname,'workspace_readiness_fixture.js')));
    } else if(path==='/') {
      res.setHeader('Content-Type','text/html');
      res.end(`<!doctype html><meta charset="utf-8"><title>RGB-D browser GPU checks</title>
        <style>body{background:#14191f;color:#e9f1f8;font:15px system-ui;padding:20px}canvas{width:256px;height:256px;border:1px solid #586}button{margin:8px;padding:10px}pre{white-space:pre-wrap}#gallery{display:flex;gap:18px}figure{margin:0}img{width:256px}</style>
        <h1>RGB-D browser GPU checks</h1><p>Synthetic cameras only. No robot or camera connection.</p>
        <button id="run">Run GPU checks</button><button id="benchmark">Benchmark 3 cameras</button>
        <canvas id="view"></canvas><div id="gallery"></div><pre id="result">Ready</pre>
        <script type="module">import './hybrid_rgbd_renderer.js';import {runRenderingChecks,runModeledWorkspaceChecks,runOriginalRgbChecks,benchmark} from './web_rendering_fixture.js';import {runWorkspaceReadinessChecks} from './workspace_readiness_fixture.js';
        const result=document.getElementById('result');
        const run=async()=>{try{result.textContent='Running';const data=await (new URLSearchParams(location.search).has('ready')?runWorkspaceReadinessChecks():new URLSearchParams(location.search).has('raw')?runOriginalRgbChecks():new URLSearchParams(location.search).has('modeled')?runModeledWorkspaceChecks():runRenderingChecks());window.testResult=data;result.textContent=JSON.stringify(data,null,2);}catch(e){window.testError=e.stack;result.textContent='FAILED: '+e.stack;console.error(e);}};
        document.getElementById('run').onclick=run;
        document.getElementById('benchmark').onclick=async()=>{try{result.textContent='Benchmarking';result.textContent=JSON.stringify(await benchmark(),null,2);}catch(e){result.textContent=e.stack;}};
        if(new URLSearchParams(location.search).has('autorun'))run();</script>`);
    } else {res.writeHead(404);res.end();}
  });
}
module.exports={createTestServer};
if(require.main===module){const server=createTestServer();server.listen(Number(process.env.PORT||18766),'127.0.0.1',()=>process.stdout.write('GPU fixture: http://127.0.0.1:'+server.address().port+'/\n'));}
