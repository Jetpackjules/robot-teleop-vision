const assert=require('node:assert/strict');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');
const server=createTestServer();
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));const browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
 try{
  const page=await browser.newPage();await page.goto(`http://127.0.0.1:${server.address().port}/`);
  const result=await page.evaluate(async()=>{
   const {decodeExactJpegRgba,exactJpegDecoderStatus}=await import('/jpeg_rgba_decoder.js');let checks=0;
   const check=(v,label)=>{checks++;if(!v)throw Error(label);},samples=[];
   for(const [width,height]of [[6,2],[10,6],[24,18],[26,18],[32,32],[640,360],[1920,1080]]){
    const canvas=new OffscreenCanvas(width,height),ctx=canvas.getContext('2d',{willReadFrequently:true,alpha:false}),source=new ImageData(width,height);
    let seed=width*height;for(let i=0;i<source.data.length;i+=4){seed=(Math.imul(seed,1664525)+1013904223)>>>0;source.data.set([seed&255,(seed>>>8)&255,(seed>>>16)&255,255],i);}
    ctx.putImageData(source,0,0);const encoded=new Uint8Array(await(await canvas.convertToBlob({type:'image/jpeg',quality:.8})).arrayBuffer());
    const parts=[encoded.subarray(0,2)];let offset=2;while(offset<encoded.length){const marker=encoded[offset+1];if(marker===218){parts.push(encoded.subarray(offset));break;}const end=offset+2+((encoded[offset+2]<<8)|encoded[offset+3]);if(marker!==226)parts.push(encoded.subarray(offset,end));offset=end;}const jpeg=new Uint8Array(parts.reduce((n,p)=>n+p.length,0));let position=0;for(const part of parts){jpeg.set(part,position);position+=part.length;}
    const bitmap=await createImageBitmap(new Blob([jpeg],{type:'image/jpeg'}));ctx.drawImage(bitmap,0,0);bitmap.close();const reference=ctx.getImageData(0,0,width,height).data;
    for(let n=0;n<2;n++){const exact=await decodeExactJpegRgba(jpeg,width,height);check(Boolean(exact),'native JPEG is supported '+width+'x'+height);check(exact.every((v,i)=>v===reference[i]),'decoded RGB bytes and alpha match the original canvas path');}
    samples.push({jpeg,width,height,reference:new Uint8Array(reference)});
   }
   const concurrent=await Promise.all(samples.slice(-3).map(async s=>({sample:s,rgba:await decodeExactJpegRgba(s.jpeg,s.width,s.height)})));
   for(const {sample,rgba}of concurrent)check(rgba.every((v,i)=>v===sample.reference[i]),'concurrent calls retain independent conversion memory');
   const s=samples[2],tagged=new Uint8Array(s.jpeg.length+8);tagged.set(s.jpeg.subarray(0,2));tagged.set([255,225,0,6,69,120,105,102],2);tagged.set(s.jpeg.subarray(2),10);
   check(await decodeExactJpegRgba(tagged,s.width,s.height)===null,'EXIF images retain the browser fallback');
   check(await decodeExactJpegRgba(s.jpeg,25,s.height)===null,'odd dimensions retain the browser fallback');
   check(await decodeExactJpegRgba(new Uint8Array([137,80,78,71]),32,32)===null,'non-JPEG images retain the browser fallback');
   const status=exactJpegDecoderStatus();check(status.disabled.length===0,'self-verification did not hide a conversion mismatch');check(status.verified.length===samples.length,'every size exercised the optimized path');
   const {NativeRgbdDecoderPool}=await import('/rgbd_decode_pool.js');
   for(const softwareColor of [true,false]){
    const pool=new NativeRgbdDecoderPool({softwareColor});
    try{
     await Promise.all(samples.slice(-3).map(async sample=>{
      const {width,height}=sample,values=Uint16Array.from({length:width*height},(_,i)=>(i*137)&65535);
      const stream=new Blob([values]).stream().pipeThrough(new CompressionStream('deflate'));
      const depth=new Uint8Array(await new Response(stream).arrayBuffer()),color=sample.jpeg.slice();
      const decoded=await pool.decode({metadata:{width,height,depth_encoding:'u16_mm_deflate'},color,depth});
      check(color.byteLength===0&&depth.byteLength===0,'worker inputs transfer without copies');
      check(decoded.depth.every((v,i)=>v===values[i]),'worker preserves every native depth value');
      check(decoded.rgbaPixels.every((v,i)=>v===sample.reference[i]),'worker color matches Canvas with optimized decoder '+softwareColor);
     }));
    }finally{pool.close();}
   }
   return {checks,verified:status.verified};
  });assert.ok(result.checks>=35);console.log(JSON.stringify(result));
 }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);process.exitCode=1;server.close();});
