// Compact native samples only: no image-decoder or geometry-module startup cost.
export async function decodeSparsePoints(depth,metadata){
  const width=Number(metadata.width),height=Number(metadata.height),count=Number(metadata.preview_count),budget=Number(metadata.preview_budget);
  if(!Number.isInteger(width)||!Number.isInteger(height)||width<1||height<1||width*height>16*1024*1024||![0,20000,40000].includes(budget)||!Number.isInteger(count)||count<0||(budget===0?count!==metadata.preview_source_points:count>budget)||count>width*height)throw Error('invalid sparse point dimensions');
  let packed=await new Response(new Blob([depth]).stream().pipeThrough(new DecompressionStream('deflate'))).arrayBuffer();
  if(packed.byteLength!==count*9)throw Error('invalid sparse point payload');
  if(metadata.preview_predictor==='shuffle_delta_v2'){
    const bytes=new Uint8Array(packed),restored=new ArrayBuffer(count*9);
    const indices=new Uint32Array(restored,0,count),depths=new Uint16Array(restored,count*4,count),colors=new Uint8Array(restored,count*6,count*3);
    let index=0,z=0,redGreen=0,green=0,blueGreen=0;
    for(let i=0;i<count;i++){
      index=(index+(bytes[i]|bytes[count+i]<<8|bytes[count*2+i]<<16|bytes[count*3+i]<<24))>>>0;
      const zigzag=bytes[count*4+i]|bytes[count*5+i]<<8;
      z=(z+((zigzag>>>1)^-(zigzag&1)))&65535;
      redGreen=(redGreen+bytes[count*6+i])&255;green=(green+bytes[count*7+i])&255;blueGreen=(blueGreen+bytes[count*8+i])&255;
      indices[i]=index;depths[i]=z;colors[i*3]=(redGreen+green)&255;colors[i*3+1]=green;colors[i*3+2]=(blueGreen+green)&255;
    }
    packed=restored;
  }
  const indices=new Uint32Array(packed,0,count),depths=new Uint16Array(packed,count*4,count),colors=new Uint8Array(packed,count*6,count*3);
  if(metadata.preview_predictor==='delta_v1'){
    for(let i=1;i<count;i++){indices[i]+=indices[i-1];depths[i]+=depths[i-1];}
    for(let i=3;i<colors.length;i++)colors[i]+=colors[i-3];
  }else if(metadata.preview_predictor&&metadata.preview_predictor!=='shuffle_delta_v2')throw Error('unknown sparse point predictor');
  let previous=-1;
  for(let i=0;i<count;i++){
    const index=indices[i];
    if(index<=previous||index>=width*height||!depths[i])throw Error('invalid sparse point sample');
    previous=index;
  }
  return {depth:null,rgbaPixels:null,previewSamples:{indices,depths,colors,valid:metadata.preview_source_points,budget},validPoints:metadata.preview_source_points};
}
if(typeof WorkerGlobalScope!=='undefined'&&self instanceof WorkerGlobalScope)self.onmessage=async({data})=>{
 const started=performance.now(),{id,depth,metadata}=data;
 try{
  const result=await decodeSparsePoints(depth,metadata),preview=result.previewSamples;
  self.postMessage({id,previewPacked:preview.indices.buffer,previewCount:preview.indices.length,previewValid:preview.valid,previewBudget:preview.budget,workerMs:performance.now()-started},[preview.indices.buffer]);
 }catch(error){self.postMessage({id,error:String(error?.message||error)});}
};
