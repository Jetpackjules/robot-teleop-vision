// Stateless native Quality frames: preserve every depth sample and JPEG pixel.
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
  const {id, metadata, color, depth, rgbaPixels} = data;
  const [depthResult, colorResult] = await Promise.allSettled([
    decodeIndependentDepth(depth, metadata),
    createImageBitmap(new Blob([color], {type: "image/jpeg"})),
  ]);
  if (depthResult.status === "rejected" || colorResult.status === "rejected") {
    if (colorResult.status === "fulfilled") colorResult.value.close();
    const error = depthResult.status === "rejected" ? depthResult.reason : colorResult.reason;
    self.postMessage({id, error: String(error?.message || error)});
    return;
  }
  const bitmap = colorResult.value, values = depthResult.value;
  if (bitmap.width !== Number(metadata.width) || bitmap.height !== Number(metadata.height)) {
    bitmap.close();self.postMessage({id, error: "native color dimensions do not match depth"});return;
  }
  try {
    if(rgbaPixels && typeof OffscreenCanvas === "function") {
      if(!rgbaCanvas) {
        rgbaCanvas=new OffscreenCanvas(bitmap.width,bitmap.height);
        rgbaContext=rgbaCanvas.getContext("2d",{willReadFrequently:true,alpha:false});
      }
      if(rgbaCanvas.width!==bitmap.width||rgbaCanvas.height!==bitmap.height) {
        rgbaCanvas.width=bitmap.width;rgbaCanvas.height=bitmap.height;
      }
      rgbaContext.drawImage(bitmap,0,0);
      const pixels=rgbaContext.getImageData(0,0,bitmap.width,bitmap.height).data;
      bitmap.close();
      self.postMessage({id,depth:values.buffer,rgba:pixels.buffer,workerMs:performance.now()-started},[values.buffer,pixels.buffer]);
    } else self.postMessage({id, depth: values.buffer, bitmap,workerMs:performance.now()-started}, [values.buffer, bitmap]);
  } catch (error) {
    bitmap.close();self.postMessage({id, error: String(error?.message || error)});
  }
};
