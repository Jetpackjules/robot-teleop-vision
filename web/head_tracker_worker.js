let assetsPromise;
let landmarker;

self.addEventListener("message", async ({ data }) => {
  const { id, type, frame, timestamp } = data;
  try {
    if (type === "initialize") {
      assetsPromise ??= import(data.moduleUrl).then(async ({ FaceLandmarker, FilesetResolver }) => ({
        FaceLandmarker,
        fileset: await FilesetResolver.forVisionTasks(data.wasmUrl),
      }));
      const { FaceLandmarker, fileset } = await assetsPromise;
      landmarker?.close();
      landmarker = await FaceLandmarker.createFromOptions(fileset, {
        ...data.options,
        canvas: new OffscreenCanvas(320, 240),
      });
      self.postMessage({ id, value: true });
    } else if (type === "detect") {
      const result = landmarker.detectForVideo(frame, timestamp);
      self.postMessage({ id, value: { faceLandmarks: result.faceLandmarks } });
    }
  } catch (error) {
    self.postMessage({ id, error: String(error.message || error) });
  } finally {
    frame?.close();
  }
});
