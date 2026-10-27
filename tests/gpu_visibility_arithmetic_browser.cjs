// Exact binary32 arithmetic oracle for the finite normal-range projection path.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');
function fusedBits(a,b,c){
  const unpack=bits=>{const e=(bits>>>23)&255;return {m:BigInt((bits&0x7fffff)|(e?0x800000:0))*((bits>>>31)?-1n:1n),e:e?e-150:-149};};
  const x=unpack(a),y=unpack(b),z=unpack(c),e=Math.min(x.e+y.e,z.e);
  let n=(x.m*y.m<<BigInt(x.e+y.e-e))+(z.m<<BigInt(z.e-e));
  if(!n)return ((a^b)&c&0x80000000)>>>0;
  const sign=n<0n?0x80000000:0;n=n<0n?-n:n;
  let shift=n.toString(2).length-24;
  if(shift>0){const low=n&((1n<<BigInt(shift))-1n),half=1n<<BigInt(shift-1);n>>=BigInt(shift);if(low>half||low===half&&(n&1n))n++;}
  else n<<=BigInt(-shift);
  if(n===0x1000000n){n>>=1n;shift++;}
  const exponent=e+shift+150;assert(exponent>0&&exponent<255,'oracle inputs stay in normal range');
  return (sign|(exponent<<23)|Number(n&0x7fffffn))>>>0;
}
function cases(){
  let seed=0x6d2b79f5;const random=()=>{seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;return seed>>>0;};
  const bits=(lo,hi)=>(((random()&0x80000000)|((127+lo+random()%(hi-lo+1))<<23)|(random()&0x7fffff))>>>0);
  const out=[],f=new Float32Array(1),u=new Uint32Array(f.buffer);
  for(let i=0;i<60000;i++){
    const a=bits(-40,8),b=bits(-20,8);let c=bits(-40,16);
    if(i%3===0){u[0]=a;const af=f[0];u[0]=b;f[0]=-Math.fround(af*f[0]);c=u[0];}
    out.push([a,b,c]);
  }
  for(const a of [0,0x80000000,0x3f800000,0xbf800000,0x3f800001,0x3f7fffff,0x3f801000])
    for(const b of [0,0x80000000,0x3f800000,0xbf800000,0x3f800001,0x3f7fffff])
      for(const c of [0,0x80000000,0x3f800000,0xbf800000,0x33800000,0xb3800000])out.push([a,b,c]);
  return out;
}
(async()=>{
 const f32=new Float32Array(1),u32=new Uint32Array(f32.buffer);
 for(let n=0;n<16777216;n++){f32[0]=n/16777216;const decoded=u32[0]+(n?1:0);f32[0]=n/16777215;assert.equal(decoded,u32[0],'exact 24-bit depth normalization');}
 const input=cases(),expected=Uint32Array.from(input,v=>fusedBits(...v));
 const server=createTestServer();await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
 try{
  const page=await browser.newPage();await page.route('**/experimental_gpu_visibility.js',route=>route.fulfill({contentType:'text/javascript',body:fs.readFileSync(path.join(__dirname,'../web/experimental_gpu_visibility.js'),'utf8')+'\nexport {GL_MATCHED_MATH};'}));
  await page.goto(`http://127.0.0.1:${server.address().port}/`);
  const result=await page.evaluate(async({input,expected})=>{
   const {GL_MATCHED_MATH}=await import('/experimental_gpu_visibility.js'),adapter=await navigator.gpu.requestAdapter(),device=await adapter.requestDevice(),errors=[];device.addEventListener('uncapturederror',e=>errors.push(e.error.message));
   const count=input.length,data=new Uint32Array(count*4);input.forEach((v,i)=>data.set(v,i*4));
   const make=(size,usage)=>device.createBuffer({size,usage});
   const params=make(96,GPUBufferUsage.UNIFORM|GPUBufferUsage.COPY_DST),source=make(data.byteLength,GPUBufferUsage.STORAGE|GPUBufferUsage.COPY_DST),output=make(count*4,GPUBufferUsage.STORAGE|GPUBufferUsage.COPY_SRC),read=make(count*4,GPUBufferUsage.COPY_DST|GPUBufferUsage.MAP_READ);
   try{
    device.queue.writeBuffer(source,0,data);device.queue.writeBuffer(params,0,new Uint32Array(24));
    const code=`struct Params {matrix:mat4x4f,intrinsics:vec4f,size:vec2u,half:u32,padding:u32};@group(0) @binding(0) var<uniform> p:Params;@group(0) @binding(1) var<storage,read> values:array<vec4u>;@group(0) @binding(2) var<storage,read_write> results:array<u32>;${GL_MATCHED_MATH}
    @compute @workgroup_size(64) fn main(@builtin(global_invocation_id) id:vec3u){if(id.x>=arrayLength(&values)){return;}let v=values[id.x];let value=fused(vec4f(bitcast<f32>(v.x)),vec4f(bitcast<f32>(v.y)),vec4f(bitcast<f32>(v.z)));results[id.x]=bitcast<u32>(value.x);}`;
    const pipeline=await device.createComputePipelineAsync({layout:'auto',compute:{module:device.createShaderModule({code}),entryPoint:'main'}}),group=device.createBindGroup({layout:pipeline.getBindGroupLayout(0),entries:[{binding:0,resource:{buffer:params}},{binding:1,resource:{buffer:source}},{binding:2,resource:{buffer:output}}]}),encoder=device.createCommandEncoder(),pass=encoder.beginComputePass();
    pass.setPipeline(pipeline);pass.setBindGroup(0,group);pass.dispatchWorkgroups(Math.ceil(count/64));pass.end();encoder.copyBufferToBuffer(output,0,read,0,count*4);device.queue.submit([encoder.finish()]);await read.mapAsync(GPUMapMode.READ);
    const actual=new Uint32Array(read.getMappedRange()),different=[];let changed=0;for(let i=0;i<count;i++)if(actual[i]!==expected[i]){changed++;if(different.length<12)different.push({input:input[i],expected:expected[i],actual:actual[i]});}
    return {count,changed,different,errors,depthValuesChecked:16777216};
   }finally{if(read.mapState==='mapped')read.unmap();for(const b of [params,source,output,read])b.destroy();device.destroy();}
  },{input,expected:Array.from(expected)});
  if(process.env.RESULT_PATH)fs.writeFileSync(process.env.RESULT_PATH,JSON.stringify(result,null,2));console.log(JSON.stringify(result));assert.deepEqual(result.errors,[]);assert.equal(result.changed,0);
 }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
