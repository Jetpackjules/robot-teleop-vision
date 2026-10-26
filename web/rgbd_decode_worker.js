// Stateless native Quality frames: preserve every depth sample and JPEG pixel.
import {colorFrameDimensions} from './hybrid_color_projection.js?v=20261021_original_rgb';
import {workspaceDepthTiles} from './workspace_crop.js?v=20261021_original_rgb';
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
  const {id, metadata, color, depth, rgbaPixels, workspaceTiles,softwareColor=true} = data;
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
    const transfer=[values.buffer];if(tiles)transfer.push(tiles.buffer);
    if(rgba){
      self.postMessage({id,depth:values.buffer,depthTiles:tiles?.buffer,rgba:rgba.buffer,workerMs:performance.now()-started},[...transfer,rgba.buffer]);
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
      self.postMessage({id,depth:values.buffer,depthTiles:tiles?.buffer,rgba:pixels.buffer,workerMs:performance.now()-started},[...transfer,pixels.buffer]);
    } else self.postMessage({id, depth: values.buffer,depthTiles:tiles?.buffer, bitmap,workerMs:performance.now()-started}, [...transfer, bitmap]);
  } catch (error) {
    bitmap?.close();self.postMessage({id, error: String(error?.message || error)});
  }
};
