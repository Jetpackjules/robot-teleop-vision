// Camera-free browser test page. The probe is injected only by this local server.
const http = require('node:http');
const {readFileSync} = require('node:fs');
const {resolve} = require('node:path');
const root = resolve(__dirname,'..');
const modules = new Set(['hybrid_rgbd_renderer.js','hybrid_camera_fusion.js','hybrid_camera_quality.js','hybrid_blendpcr.js','camera_rgb_viewer.js']);
function createTestServer() {
  return http.createServer((req,res) => {
    const path = new URL(req.url,'http://localhost').pathname;
    res.setHeader('Cache-Control','no-store');
    if(modules.has(path.slice(1))) {
      res.setHeader('Content-Type','text/javascript');
      let source=readFileSync(resolve(root,'web',path.slice(1)),'utf8');
      if(path==='/hybrid_rgbd_renderer.js')source+='\nwindow.probe={state,createProgram,findUniforms,render,uploadCamera,decodeCamera,identityMatrix,inverseRigidTransform,perspectiveMatrix,multiplyMatrix,deleteCameraTextures};';
      res.end(source);
    } else if(path==='/web_rendering_fixture.js') {
      res.setHeader('Content-Type','text/javascript');res.end(readFileSync(resolve(__dirname,'web_rendering_fixture.js')));
    } else if(path==='/') {
      res.setHeader('Content-Type','text/html');
      res.end(`<!doctype html><meta charset="utf-8"><title>RGB-D browser GPU checks</title>
        <style>body{background:#14191f;color:#e9f1f8;font:15px system-ui;padding:20px}canvas{width:256px;height:256px;border:1px solid #586}button{margin:8px;padding:10px}pre{white-space:pre-wrap}#gallery{display:flex;gap:18px}figure{margin:0}img{width:256px}</style>
        <h1>RGB-D browser GPU checks</h1><p>Synthetic cameras only. No robot or camera connection.</p>
        <button id="run">Run GPU checks</button><button id="benchmark">Benchmark 3 cameras</button>
        <canvas id="view"></canvas><div id="gallery"></div><pre id="result">Ready</pre>
        <script type="module">import './hybrid_rgbd_renderer.js';import {runRenderingChecks,benchmark} from './web_rendering_fixture.js';
        const result=document.getElementById('result');
        const run=async()=>{try{result.textContent='Running';const data=await runRenderingChecks();window.testResult=data;result.textContent=JSON.stringify(data,null,2);}catch(e){window.testError=e.stack;result.textContent='FAILED: '+e.stack;console.error(e);}};
        document.getElementById('run').onclick=run;
        document.getElementById('benchmark').onclick=async()=>{try{result.textContent='Benchmarking';result.textContent=JSON.stringify(await benchmark(),null,2);}catch(e){result.textContent=e.stack;}};
        if(new URLSearchParams(location.search).has('autorun'))run();</script>`);
    } else {res.writeHead(404);res.end();}
  });
}
module.exports={createTestServer};
if(require.main===module){const server=createTestServer();server.listen(Number(process.env.PORT||18766),'127.0.0.1',()=>process.stdout.write('GPU fixture: http://127.0.0.1:'+server.address().port+'/\n'));}
