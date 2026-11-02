// Stateless native Quality frames: preserve every depth sample and JPEG pixel.
import {colorFrameDimensions} from './hybrid_color_projection.js?v=20261021_original_rgb';
import {workspaceDepthTiles} from './table_geometry.js?v=20261021_original_rgb';
import {sampleDepthPoints} from './point_preview_sampling.js?v=20261027_point_options';
import {decodeExactJpegRgba,preferExactJpegDecode} from './jpeg_rgba_decoder.js?v=20261026_exact_color';
export async function decodeIndependentDepth(bytes, metadata) {
  const width = Number(metadata.width), height = Number(metadata.height);
  const count = width * height;
  if (!Number.isInteger(width) || !Number.isInteger(height) || width < 1 || height < 1
      || count > 16 * 1024 * 1024) throw new Error("invalid depth dimensions");
  const stream = new Blob([bytes]).stream().pipeThrough(new DecompressionStream("deflate"));
  const buffer = await new Response(stream).arrayBuffer();
  if (buffer.byteLength !== count * 2) throw new Error("invalid native depth length");
  if (metadata.depth_encoding !== "u16_mm_vpredict_shuffle_deflate") {
    if (metadata.depth_encoding && metadata.depth_encoding !== "u16_mm_deflate")
      throw new Error("unsupported independent depth encoding");
    return new Uint16Array(buffer);
  }
  const predicted = new Uint8Array(buffer), depth = new Uint16Array(count);
  for (let index = 0; index < count; index++) {
    const zigzag = predicted[index] | (predicted[index + count] << 8);
    const delta = (zigzag >>> 1) ^ -(zigzag & 1);
    depth[index] = ((index >= width ? depth[index - width] : 0) + delta) & 65535;
  }
  return depth;
}

let rgbaCanvas, rgbaContext;
if (typeof self !== "undefined") self.onmessage = async ({data}) => {
  const started=performance.now();
  const {id, metadata, color, depth, rgbaPixels, workspaceTiles,pointPreview=false,pointPreviewBudget=20000,softwareColor=true} = data;
  if(metadata.depth_encoding==='u16_mm_sparse_points_deflate'){
    try{
      const width=Number(metadata.width),height=Number(metadata.height),count=Number(metadata.preview_count),budget=Number(metadata.preview_budget);
      if(!Number.isInteger(width)||!Number.isInteger(height)||width<1||height<1||width*height>16*1024*1024||![0,20000,40000].includes(budget)||!Number.isInteger(count)||count<0||(budget===0?count!==metadata.preview_source_points:count>budget)||count>width*height)throw Error('invalid sparse point dimensions');
      const packed=await new Response(new Blob([depth]).stream().pipeThrough(new DecompressionStream('deflate'))).arrayBuffer();
      if(packed.byteLength!==count*9)throw Error('invalid sparse point payload');
      const view=new DataView(packed),values=new Uint16Array(width*height),rgba=new Uint8Array(width*height*4),indices=new Uint32Array(count);
      let previous=-1;
      for(let i=0;i<count;i++){
        const index=view.getUint32(i*4,true),z=view.getUint16(count*4+i*2,true);
        if(index<=previous||index>=values.length||!z)throw Error('invalid sparse point sample');
        previous=index;indices[i]=index;values[index]=z;
        const color=count*6+i*3;rgba.set([view.getUint8(color),view.getUint8(color+1),view.getUint8(color+2),255],index*4);
      }
      self.postMessage({id,depth:values.buffer,rgba:rgba.buffer,previewIndices:indices.buffer,previewValid:metadata.preview_source_points,previewBudget:budget,workerMs:performance.now()-started},[values.buffer,rgba.buffer,indices.buffer]);
    }catch(error){self.postMessage({id,error:String(error?.message||error)});}
    return;
  }
  let colorWidth,colorHeight;
  try {[colorWidth,colorHeight]=colorFrameDimensions(metadata);}catch(error){self.postMessage({id,error:error.message});return;}
  const [depthResult, colorResult] = await Promise.allSettled([
    decodeIndependentDepth(depth, metadata),
    (async()=>{
      if(rgbaPixels&&softwareColor&&metadata.color_encoding!=='png'&&preferExactJpegDecode(colorWidth,colorHeight)){
        const rgba=await decodeExactJpegRgba(color,colorWidth,colorHeight);if(rgba)return {rgba};
      }
      return {bitmap:await createImageBitmap(new Blob([color], {type: metadata.color_encoding==='png'?'image/png':'image/jpeg'}))};
    })(),
  ]);
  if (depthResult.status === "rejected" || colorResult.status === "rejected") {
    if (colorResult.status === "fulfilled") colorResult.value.bitmap?.close();
    const error = depthResult.status === "rejected" ? depthResult.reason : colorResult.reason;
    self.postMessage({id, error: String(error?.message || error)});
    return;
  }
  const {bitmap,rgba}=colorResult.value,values = depthResult.value;
  const width=Number(metadata.width),height=Number(metadata.height);
  if (bitmap&&(bitmap.width !== colorWidth || bitmap.height !== colorHeight)) {
    bitmap.close();self.postMessage({id, error: "color dimensions do not match metadata"});return;
  }
  try {
    const tiles=workspaceTiles?workspaceDepthTiles(values,width,height):null;
    const preview=pointPreview?sampleDepthPoints(values,pointPreviewBudget):null;
    let validPoints=preview?.valid;if(validPoints===undefined){validPoints=0;for(let i=0;i<values.length;i++)if(values[i])validPoints++;}
    const previewData={previewValid:validPoints,...(preview?{previewIndices:preview.indices.buffer,previewBudget:pointPreviewBudget}:{})};
    const transfer=[values.buffer];if(tiles)transfer.push(tiles.buffer);
    if(preview)transfer.push(preview.indices.buffer);
    if(rgba){
      self.postMessage({id,...previewData,depth:values.buffer,depthTiles:tiles?.buffer,rgba:rgba.buffer,workerMs:performance.now()-started},[...transfer,rgba.buffer]);
    }else if(rgbaPixels && typeof OffscreenCanvas === "function") {
      if(!rgbaCanvas) {
        rgbaCanvas=new OffscreenCanvas(colorWidth,colorHeight);
        rgbaContext=rgbaCanvas.getContext("2d",{willReadFrequently:true,alpha:false});
      }
      if(rgbaCanvas.width!==colorWidth||rgbaCanvas.height!==colorHeight) {
        rgbaCanvas.width=colorWidth;rgbaCanvas.height=colorHeight;
      }
      rgbaContext.drawImage(bitmap,0,0);
      const pixels=rgbaContext.getImageData(0,0,colorWidth,colorHeight).data;
      bitmap.close();
      self.postMessage({id,...previewData,depth:values.buffer,depthTiles:tiles?.buffer,rgba:pixels.buffer,workerMs:performance.now()-started},[...transfer,pixels.buffer]);
    } else self.postMessage({id,...previewData,depth: values.buffer,depthTiles:tiles?.buffer, bitmap,workerMs:performance.now()-started}, [...transfer, bitmap]);
  } catch (error) {
    bitmap?.close();self.postMessage({id, error: String(error?.message || error)});
  }
};
