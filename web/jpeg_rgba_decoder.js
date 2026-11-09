// Decode native JPEGs without a GPU-to-canvas readback. The integer chroma
// reconstruction matches the existing browser JPEG path, including rounding.
// SPDX-License-Identifier: EUPL-1.2
let converterPromise;
let memoryTail=Promise.resolve();
const verified=new Set(),disabled=new Set();
const preferred=new Map();
export function preferExactJpegDecode(width,height){return preferred.get(`${width}:${height}`)!==false;}
export function exactJpegDecoderStatus(){return {verified:[...verified],disabled:[...disabled],preferred:[...preferred]};}
function plainJpeg(bytes){
  if(bytes[0]!==255||bytes[1]!==216)return false;
  let p=2;
  while(p+3<bytes.length){
    if(bytes[p++]!==255)return false;
    while(bytes[p]===255)p++;
    const marker=bytes[p++];if(marker===218)return true;
    // Keep profiles and EXIF orientation on the original browser path.
    if(marker===225||marker===226||marker===238)return false;
    const length=(bytes[p]<<8)|bytes[p+1];if(length<2||p+length>bytes.length)return false;p+=length;
  }
  return false;
}
async function converter(){
  converterPromise ||= (async()=>{
    const response=await fetch(new URL('./jpeg_i420_rgba.wasm?v=20261026_exact_color',import.meta.url));
    if(!response.ok)return null;
    return (await WebAssembly.instantiate(await response.arrayBuffer())).instance.exports;
  })().catch(()=>null);
  return converterPromise;
}
export async function decodeExactJpegRgba(bytes,width,height){
  const key=`${width}:${height}`;
  if(typeof ImageDecoder!=='function'||typeof WebAssembly!=='object'||disabled.has(key)
      ||width<6||height<2||width%2||height%2||width*height>16*1024*1024||!plainJpeg(bytes))return null;
  let decoder,frame;
  try{
    const wasm=await converter();if(!wasm)return null;
    const started=performance.now();
    decoder=new ImageDecoder({data:bytes,type:'image/jpeg'});frame=(await decoder.decode()).image;
    if(frame.format!=='I420'||frame.displayWidth!==width||frame.displayHeight!==height)return null;
    const previous=memoryTail;let release,rgba;
    memoryTail=new Promise(resolve=>{release=resolve;});await previous;
    try{
      const size=frame.allocationSize(),offset=Math.ceil(size/16)*16,rgbaSize=width*height*4,total=offset+rgbaSize;
      if(wasm.memory.buffer.byteLength<total)wasm.memory.grow(Math.ceil((total-wasm.memory.buffer.byteLength)/65536));
      const layout=await frame.copyTo(new Uint8Array(wasm.memory.buffer,0,size));
      wasm.convert(layout[0].offset,layout[1].offset,layout[2].offset,layout[0].stride,layout[1].stride,layout[2].stride,offset,width,height);
      rgba=new Uint8Array(wasm.memory.buffer,offset,rgbaSize).slice();
    }finally{release();}
    if(!verified.has(key)){
      const decodeMs=performance.now()-started,referenceStarted=performance.now();
      // Verify the current browser implementation before enabling this path.
      // A mismatch retains its original pixels and disables the optimization.
      const bitmap=await createImageBitmap(new Blob([bytes],{type:'image/jpeg'}));
      const canvas=new OffscreenCanvas(width,height),ctx=canvas.getContext('2d',{willReadFrequently:true,alpha:false});
      let reference;
      try{ctx.drawImage(bitmap,0,0);reference=ctx.getImageData(0,0,width,height).data;}
      finally{bitmap.close();canvas.width=canvas.height=1;}
      const referenceMs=performance.now()-referenceStarted;
      if(!rgba.every((v,i)=>v===reference[i])){disabled.add(key);return new Uint8Array(reference.buffer,reference.byteOffset,reference.byteLength);}
      verified.add(key);
      // GPU/CPU contention differs by laptop. Enable only with a clear
      // measured win on the same image; otherwise retain Canvas decoding.
      preferred.set(key,decodeMs<referenceMs*.9);
    }
    return rgba;
  }catch(_error){disabled.add(key);return null;}
  finally{frame?.close();decoder?.close();}
}
