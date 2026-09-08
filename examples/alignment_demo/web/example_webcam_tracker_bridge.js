// Example-local copy of web/godot_webcam_tracker_bridge.js. Startup cancellation
// is scoped here so late camera permission cannot outlive this page's ownership.
// Keep the pose mapping and model configuration aligned with the shared tracker.
const MODEL_URL = "https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/latest/face_landmarker.task";
const MEDIAPIPE_URL = "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.35/vision_bundle.mjs";
const MEDIAPIPE_WASM_URL = "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.35/wasm";
const OPENCV_URL = "https://cdn.jsdelivr.net/npm/@techstark/opencv-js@4.10.0-release.1/dist/opencv.js";
const FACE_CASCADE_URL = "https://raw.githubusercontent.com/opencv/opencv/4.x/data/haarcascades/haarcascade_frontalface_default.xml";
const TRACKING_JS_URL = "https://cdn.jsdelivr.net/npm/tracking@1.1.3/build/tracking-min.js";
const TRACKING_FACE_URL = "https://cdn.jsdelivr.net/npm/tracking@1.1.3/build/data/face-min.js";

const state = {
  startGeneration: 0,
  running: false,
  backend: "mediapipe",
  mediapipeModule: null,
  landmarker: null,
  mediapipeDelegate: "CPU",
  mediapipeNoFaceSinceMs: 0,
  mediapipeRecoveryAttempted: false,
  mediapipeLastDetectMs: 0,
  cvReady: null,
  cvClassifier: null,
  cvCanvas: null,
  cvLastDetectMs: 0,
  trackingReady: null,
  trackingTask: null,
  trackingTracker: null,
  trackingRaf: 0,
  trackingCanvas: null,
  trackingContext: null,
  trackingRuns: [],
  trackingHits: [],
  remoteRuns: [],
  remoteWs: null,
  remoteTransportSend: null,
  remoteTransportReady: null,
  remoteTransportLabel: "WebRTC data",
  remoteReconnectTimer: 0,
  remoteUrl: "",
  remoteEnabled: false,
  remoteStatus: "remote off",
  remoteLastSendMs: 0,
  lastFaceCount: 0,
  webglInfo: null,
  lastPose: null,
  lastPoseMs: 0,
  lastFaceRect: null,
  lastFaceRectMs: 0,
  video: null,
  preview: null,
  overlay: null,
  neutral: null,
  smoothed: { x: 0, y: 0, z: 0 },
  options: {
    neutralZCm: 35,
    xScale: 1,
    zScale: 1,
    maxZM: 5,
    smoothing: 0.65,
    preview: true,
    debug: false,
    showVideo: false,
    videoContainerId: "",
    showFaceOverlay: false,
    holdLastPoseMs: 1200,
    trackingWidth: 320,
    trackingHeight: 240,
    trackingFps: 20,
    trackingInitialScale: 3,
    trackingStepSize: 1.7,
    trackingEdgesDensity: 0.08,
    remoteEnabled: false,
    remoteUrl: "ws://127.0.0.1:8766",
    remoteMaxHz: 90,
    deviceId: "",
    preferredLabel: "ACER HD",
    backend: "mediapipe",
  },
};

window.godotWebcamTrackerLatest = {
  type: "tracking",
  source: "browser_trackingjs",
  active: false,
  status: "not started",
  x: 0,
  y: 0,
  z: 0,
};

function ensureVideoElement() {
  if (state.video) return state.video;
  const video = document.createElement("video");
  video.autoplay = true;
  video.playsInline = true;
  video.muted = true;
  video.width = 640;
  video.height = 480;
  video.style.cssText = "position:fixed;left:-10000px;top:0;width:640px;height:480px;opacity:0;pointer-events:none;";
  document.body.appendChild(video);
  state.video = video;
  return video;
}

function placeVideoElement() {
  const video = ensureVideoElement();
  if (state.options.showVideo) {
    const parent = state.options.videoContainerId ? document.getElementById(state.options.videoContainerId) : null;
    if (parent && video.parentElement !== parent) {
      parent.appendChild(video);
    }
    if (parent) {
      parent.style.position = "relative";
      ensureOverlayElement(parent);
    }
    video.style.cssText = "display:block;width:100%;max-width:640px;aspect-ratio:4/3;background:#050505;border:1px solid #333;object-fit:cover;";
  } else {
    if (video.parentElement !== document.body) {
      document.body.appendChild(video);
    }
    video.style.cssText = "position:fixed;left:-10000px;top:0;width:640px;height:480px;opacity:0;pointer-events:none;";
  }
  return video;
}

function ensureOverlayElement(parent) {
  if (!state.options.showFaceOverlay) return null;
  if (state.overlay && state.overlay.parentElement === parent) return state.overlay;
  const overlay = document.createElement("div");
  overlay.style.cssText = "position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none;box-sizing:border-box;";
  parent.appendChild(overlay);
  state.overlay = overlay;
  return overlay;
}

function ensurePreviewElement() {
  if (!state.options.preview) return null;
  if (state.preview) return state.preview;
  const preview = document.createElement("div");
  preview.style.cssText = [
    "position:fixed",
    "right:12px",
    "bottom:12px",
    "z-index:9999",
    "padding:8px 10px",
    "background:rgba(0,0,0,.65)",
    "color:white",
    "font:12px/1.35 monospace",
    "border-radius:6px",
    "pointer-events:none",
    "white-space:pre",
  ].join(";");
  document.body.appendChild(preview);
  state.preview = preview;
  return preview;
}

function landmarksToPoseCm(landmarks) {
  const nose = landmarks[1] || landmarks[0];
  const leftEye = landmarks[33];
  const rightEye = landmarks[263];
  if (!nose || !leftEye || !rightEye) return null;
  const faceWidth = Math.max(0.001, Math.abs(rightEye.x - leftEye.x));
  const x = -(nose.x - 0.5) * 42.0;
  const y = -(nose.y - 0.5) * 30.0;
  const z = 6.4 / faceWidth;
  return { x, y, z };
}

function rectToPoseCm(rect, width, height) {
  const cx = rect.x + rect.width * 0.5;
  const cy = rect.y + rect.height * 0.5;
  const x = -((cx / Math.max(1, width)) - 0.5) * 42.0;
  const y = -((cy / Math.max(1, height)) - 0.5) * 30.0;
  const z = 1250.0 / Math.max(1, rect.width);
  return { x, y, z };
}

function rememberFaceRect(rect, frameWidth, frameHeight) {
  state.lastFaceRect = {
    x: rect.x,
    y: rect.y,
    width: rect.width,
    height: rect.height,
    frameWidth,
    frameHeight,
  };
  state.lastFaceRectMs = Date.now();
}

function rememberLandmarkRect(landmarks, frameWidth, frameHeight) {
  if (!landmarks || !landmarks.length) return;
  let minX = 1;
  let minY = 1;
  let maxX = 0;
  let maxY = 0;
  for (const point of landmarks) {
    minX = Math.min(minX, point.x);
    minY = Math.min(minY, point.y);
    maxX = Math.max(maxX, point.x);
    maxY = Math.max(maxY, point.y);
  }
  const padX = (maxX - minX) * 0.08;
  const padY = (maxY - minY) * 0.12;
  minX = Math.max(0, minX - padX);
  minY = Math.max(0, minY - padY);
  maxX = Math.min(1, maxX + padX);
  maxY = Math.min(1, maxY + padY);
  rememberFaceRect({
    x: minX * frameWidth,
    y: minY * frameHeight,
    width: (maxX - minX) * frameWidth,
    height: (maxY - minY) * frameHeight,
  }, frameWidth, frameHeight);
}

function rememberTrackingRun(hit, faceCount) {
  const now = Date.now();
  state.trackingRuns.push(now);
  if (hit) state.trackingHits.push(now);
  state.lastFaceCount = faceCount;
  const cutoff = now - 1000;
  while (state.trackingRuns.length && state.trackingRuns[0] < cutoff) state.trackingRuns.shift();
  while (state.trackingHits.length && state.trackingHits[0] < cutoff) state.trackingHits.shift();
}

function getWebGlInfo() {
  if (state.webglInfo) return state.webglInfo;
  const canvas = document.createElement("canvas");
  const info = {
    webgl2: false,
    webgl1: false,
    vendor: "",
    renderer: "",
  };
  const gl = canvas.getContext("webgl2") || canvas.getContext("webgl") || canvas.getContext("experimental-webgl");
  if (gl) {
    info.webgl1 = true;
    info.webgl2 = typeof WebGL2RenderingContext !== "undefined" && gl instanceof WebGL2RenderingContext;
    const debugInfo = gl.getExtension("WEBGL_debug_renderer_info");
    if (debugInfo) {
      info.vendor = gl.getParameter(debugInfo.UNMASKED_VENDOR_WEBGL) || "";
      info.renderer = gl.getParameter(debugInfo.UNMASKED_RENDERER_WEBGL) || "";
    } else {
      info.vendor = gl.getParameter(gl.VENDOR) || "";
      info.renderer = gl.getParameter(gl.RENDERER) || "";
    }
  }
  state.webglInfo = info;
  return info;
}

function emit(active, pose, status) {
  const now = Date.now();
  const debugInfo = buildDebugInfo(now);
  if (!active || !pose) {
    window.godotWebcamTrackerLatest = {
      type: "tracking",
      source: `browser_${state.backend}`,
      active: false,
      status,
      x: 0,
      y: 0,
      z: 0,
      debug: debugInfo,
      sent_unix_ms: now,
    };
    sendRemoteTrackingPacket(window.godotWebcamTrackerLatest);
    return;
  }
  state.lastPose = pose;
  state.lastPoseMs = now;
  if (!state.neutral) {
    state.neutral = pose;
  }
  const raw = {
    x: (pose.x - state.neutral.x) * state.options.xScale,
    y: pose.y - state.neutral.y,
    z: ((pose.z - state.neutral.z) * state.options.zScale) + state.options.neutralZCm,
  };
  raw.z = Math.min(raw.z, Math.max(state.options.neutralZCm, state.options.maxZM * 100.0));
  const a = Math.min(Math.max(state.options.smoothing, 0), 0.98);
  state.smoothed.x = state.smoothed.x * a + raw.x * (1 - a);
  state.smoothed.y = state.smoothed.y * a + raw.y * (1 - a);
  state.smoothed.z = state.smoothed.z * a + raw.z * (1 - a);
  window.godotWebcamTrackerLatest = {
    type: "tracking",
    source: `browser_${state.backend}`,
    active: true,
    status,
    x: state.smoothed.x,
    y: state.smoothed.y,
    z: state.smoothed.z,
    debug: debugInfo,
    sent_unix_ms: now,
  };
  sendRemoteTrackingPacket(window.godotWebcamTrackerLatest);
}

function buildDebugInfo(now = Date.now()) {
  const rect = state.lastFaceRect;
  return {
    runs_per_sec: state.trackingRuns.length,
    hits_per_sec: state.trackingHits.length,
    last_face_count: state.lastFaceCount,
    last_face_age_ms: state.lastFaceRectMs ? now - state.lastFaceRectMs : -1,
    scan_width: state.trackingCanvas ? state.trackingCanvas.width : 0,
    scan_height: state.trackingCanvas ? state.trackingCanvas.height : 0,
    target_fps: state.options.trackingFps,
    x_scale: state.options.xScale,
    webgl: state.webglInfo || getWebGlInfo(),
    remote: {
      enabled: state.remoteEnabled,
      status: state.remoteStatus,
      url: state.remoteUrl,
      sends_per_sec: state.remoteRuns.length,
    },
    box: rect ? {
      x: Math.round(rect.x),
      y: Math.round(rect.y),
      width: Math.round(rect.width),
      height: Math.round(rect.height),
    } : null,
  };
}

function rememberRemoteSend(now) {
  state.remoteRuns.push(now);
  const cutoff = now - 1000;
  while (state.remoteRuns.length && state.remoteRuns[0] < cutoff) state.remoteRuns.shift();
}

function remoteSocketOpen() {
  return state.remoteWs && state.remoteWs.readyState === WebSocket.OPEN;
}

function remoteTransportOpen() {
  if (typeof state.remoteTransportSend !== "function") return false;
  return typeof state.remoteTransportReady !== "function" || state.remoteTransportReady() === true;
}

function remoteConnectionOpen() {
  return remoteTransportOpen() || remoteSocketOpen();
}

function setRemoteStatus(status) {
  state.remoteStatus = status;
}

function scheduleRemoteReconnect() {
  if (!state.remoteEnabled || remoteTransportOpen() || state.remoteReconnectTimer) return;
  state.remoteReconnectTimer = window.setTimeout(() => {
    state.remoteReconnectTimer = 0;
    connectRemoteHeadTracking(state.remoteUrl).catch((err) => {
      setRemoteStatus(`reconnect failed: ${err && err.message ? err.message : err}`);
      scheduleRemoteReconnect();
    });
  }, 1000);
}

async function connectRemoteHeadTracking(url = state.options.remoteUrl || "ws://127.0.0.1:8766") {
  state.remoteEnabled = true;
  state.remoteUrl = url;
  state.options.remoteUrl = url;
  state.options.remoteEnabled = true;
  if (state.remoteReconnectTimer) {
    window.clearTimeout(state.remoteReconnectTimer);
    state.remoteReconnectTimer = 0;
  }
  if (remoteTransportOpen()) {
    setRemoteStatus(state.remoteTransportLabel);
    return true;
  }
  if (remoteSocketOpen()) return true;
  if (state.remoteWs) {
    try { state.remoteWs.close(); } catch (_) {}
  }
  setRemoteStatus(`connecting ${url}`);
  await new Promise((resolve, reject) => {
    const ws = new WebSocket(url);
    state.remoteWs = ws;
    const timeout = window.setTimeout(() => {
      reject(new Error(`timed out connecting to ${url}`));
      try { ws.close(); } catch (_) {}
    }, 2500);
    ws.addEventListener("open", () => {
      window.clearTimeout(timeout);
      setRemoteStatus("connected");
      resolve(true);
    }, { once: true });
    ws.addEventListener("error", () => {
      setRemoteStatus("socket error");
    });
    ws.addEventListener("close", () => {
      if (state.remoteWs === ws) {
        setRemoteStatus(state.remoteEnabled ? "disconnected; retrying" : "remote off");
        scheduleRemoteReconnect();
      }
    });
  });
  return true;
}

function disconnectRemoteHeadTracking() {
  state.remoteEnabled = false;
  state.options.remoteEnabled = false;
  if (state.remoteReconnectTimer) {
    window.clearTimeout(state.remoteReconnectTimer);
    state.remoteReconnectTimer = 0;
  }
  if (state.remoteWs) {
    try { state.remoteWs.close(); } catch (_) {}
  }
  state.remoteWs = null;
  setRemoteStatus("remote off");
}

function refreshRemoteTransport() {
  if (remoteTransportOpen()) {
    if (state.remoteReconnectTimer) {
      window.clearTimeout(state.remoteReconnectTimer);
      state.remoteReconnectTimer = 0;
    }
    if (state.remoteWs) {
      try { state.remoteWs.close(); } catch (_) {}
      state.remoteWs = null;
    }
    if (state.remoteEnabled) setRemoteStatus(state.remoteTransportLabel);
  } else if (state.remoteEnabled && !remoteSocketOpen()) {
    scheduleRemoteReconnect();
  }
}

function sendRemotePayload(payload) {
  if (remoteTransportOpen()) {
    try {
      if (state.remoteTransportSend(payload) !== false) {
        setRemoteStatus(state.remoteTransportLabel);
        return true;
      }
    } catch (_) {}
  }
  if (!remoteSocketOpen() || state.remoteWs.bufferedAmount > 65536) return false;
  state.remoteWs.send(JSON.stringify(payload));
  setRemoteStatus("WebSocket fallback");
  return true;
}

function sendRemoteTrackingPacket(packet) {
  if (!state.remoteEnabled || !remoteConnectionOpen()) return;
  const now = Date.now();
  const minMs = 1000 / Math.max(1, state.options.remoteMaxHz || 90);
  if (now - state.remoteLastSendMs < minMs) return;
  state.remoteLastSendMs = now;
  const forwarded = Object.assign({}, packet, {
    type: "tracking",
    transport: remoteTransportOpen() ? "webrtc_datachannel" : "remote_websocket",
    remote_sent_unix_ms: now,
  });
  if (!sendRemotePayload(forwarded)) {
    setRemoteStatus("backpressure");
    scheduleRemoteReconnect();
    return;
  }
  rememberRemoteSend(now);
}

function emitHeldOrSearching(status) {
  const now = Date.now();
  if (state.lastPose && now - state.lastPoseMs <= state.options.holdLastPoseMs) {
    emit(true, state.lastPose, `${status} holding`);
  } else {
    emit(false, null, `${status} searching`);
  }
}

async function initLandmarker(generation = state.startGeneration) {
  if (state.landmarker) return state.landmarker;
  const webgl = getWebGlInfo();
  window.godotWebcamTrackerLatest.status = webgl.webgl1 ? "loading mediapipe" : "mediapipe blocked: no webgl";
  if (!webgl.webgl1) {
    throw new Error("MediaPipe browser FaceLandmarker needs WebGL, but this browser could not create a WebGL context.");
  }
  if (!state.mediapipeModule) {
    state.mediapipeModule = await import(MEDIAPIPE_URL);
  }
  if (generation !== state.startGeneration) throw new DOMException("Webcam startup cancelled", "AbortError");
  const { FaceLandmarker, FilesetResolver } = state.mediapipeModule;
  const fileset = await FilesetResolver.forVisionTasks(MEDIAPIPE_WASM_URL);
  if (generation !== state.startGeneration) throw new DOMException("Webcam startup cancelled", "AbortError");
  const landmarker = await FaceLandmarker.createFromOptions(fileset, {
    baseOptions: {
      modelAssetPath: MODEL_URL,
      delegate: state.mediapipeDelegate,
    },
    runningMode: "VIDEO",
    numFaces: 1,
    outputFaceBlendshapes: false,
    outputFacialTransformationMatrixes: false,
    minFaceDetectionConfidence: 0.2,
    minFacePresenceConfidence: 0.2,
    minTrackingConfidence: 0.2,
  });
  if (generation !== state.startGeneration) {
    landmarker.close();
    throw new DOMException("Webcam startup cancelled", "AbortError");
  }
  state.landmarker = landmarker;
  return state.landmarker;
}

async function loadScriptOnce(url) {
  const existing = document.querySelector(`script[data-tracker-src="${url}"]`);
  if (existing) return;
  await new Promise((resolve, reject) => {
    const script = document.createElement("script");
    script.src = url;
    script.async = true;
    script.dataset.trackerSrc = url;
    script.onload = resolve;
    script.onerror = () => reject(new Error(`Failed to load ${url}`));
    document.head.appendChild(script);
  });
}

async function initTrackingJsTracker(video) {
  if (state.trackingReady) return await state.trackingReady;
  state.trackingReady = (async () => {
    window.godotWebcamTrackerLatest.status = "loading tracking.js";
    await loadScriptOnce(TRACKING_JS_URL);
    await loadScriptOnce(TRACKING_FACE_URL);
    if (!window.tracking || !window.tracking.ObjectTracker) {
      throw new Error("tracking.js did not expose ObjectTracker");
    }
    const tracker = new window.tracking.ObjectTracker("face");
    tracker.setInitialScale(state.options.trackingInitialScale || 3);
    tracker.setStepSize(state.options.trackingStepSize || 1.7);
    tracker.setEdgesDensity(state.options.trackingEdgesDensity || 0.08);
    tracker.on("track", (event) => {
	      if (!state.running || state.backend !== "trackingjs") return;
	      const faces = event && event.data ? event.data : [];
	      rememberTrackingRun(faces.length > 0, faces.length);
	      if (!faces.length) {
        emitHeldOrSearching("tracking.js");
        updatePreview();
        updateFaceOverlay();
        return;
      }
      let best = faces[0];
      for (let i = 1; i < faces.length; i += 1) {
        const candidate = faces[i];
        if (candidate.width * candidate.height > best.width * best.height) {
          best = candidate;
        }
      }
      const frameWidth = state.trackingCanvas ? state.trackingCanvas.width : (video.videoWidth || video.clientWidth || 640);
      const frameHeight = state.trackingCanvas ? state.trackingCanvas.height : (video.videoHeight || video.clientHeight || 480);
      rememberFaceRect(best, frameWidth, frameHeight);
      emit(true, rectToPoseCm(best, frameWidth, frameHeight), "tracking.js tracking");
      updatePreview();
      updateFaceOverlay();
    });
    state.trackingTracker = tracker;
    startTrackingJsLoop(video, tracker);
    window.godotWebcamTrackerLatest.status = "tracking.js ready";
    return tracker;
  })();
  return await state.trackingReady;
}

function startTrackingJsLoop(video, tracker) {
  if (state.trackingRaf) {
    window.cancelAnimationFrame(state.trackingRaf);
    state.trackingRaf = 0;
  }
  const canvas = document.createElement("canvas");
  canvas.width = Math.max(80, Math.floor(state.options.trackingWidth || 320));
  canvas.height = Math.max(60, Math.floor(state.options.trackingHeight || 240));
  const context = canvas.getContext("2d", { willReadFrequently: true });
  state.trackingCanvas = canvas;
  state.trackingContext = context;
  let lastRunMs = 0;
  const minFrameMs = 1000 / Math.max(1, state.options.trackingFps || 20);
  const run = (now) => {
    if (!state.running || state.backend !== "trackingjs") {
      state.trackingRaf = 0;
      return;
    }
    if (video.readyState >= 2 && now - lastRunMs >= minFrameMs) {
      lastRunMs = now;
      context.drawImage(video, 0, 0, canvas.width, canvas.height);
      const imageData = context.getImageData(0, 0, canvas.width, canvas.height);
      tracker.track(imageData.data, canvas.width, canvas.height);
    }
    state.trackingRaf = window.requestAnimationFrame(run);
  };
  state.trackingRaf = window.requestAnimationFrame(run);
}

async function waitForOpenCvRuntime() {
  if (window.cv && window.cv.Mat) return window.cv;
  window.godotWebcamTrackerLatest.status = "loading opencv";
  await loadScriptOnce(OPENCV_URL);
  if (window.cv && typeof window.cv.then === "function") {
    await Promise.race([
      window.cv,
      new Promise((_, reject) => setTimeout(() => reject(new Error("OpenCV.js runtime timed out")), 15000)),
    ]);
    if (window.cv && window.cv.Mat) return window.cv;
  }
  return await new Promise((resolve, reject) => {
    const started = Date.now();
    const previous = window.cv && window.cv.onRuntimeInitialized;
    const finish = () => resolve(window.cv);
    if (window.cv && window.cv.Mat) {
      finish();
      return;
    }
    if (window.cv) {
      window.cv.onRuntimeInitialized = () => {
        if (typeof previous === "function") previous();
        finish();
      };
    }
    const poll = () => {
      if (window.cv && window.cv.Mat) {
        finish();
      } else if (Date.now() - started > 15000) {
        reject(new Error("OpenCV.js runtime timed out"));
      } else {
        setTimeout(poll, 50);
      }
    };
    poll();
  });
}

async function initOpenCvTracker() {
  if (state.cvReady) return await state.cvReady;
  state.cvReady = (async () => {
    const cv = await waitForOpenCvRuntime();
    window.godotWebcamTrackerLatest.status = "loading face cascade";
    const response = await fetch(FACE_CASCADE_URL);
    if (!response.ok) throw new Error(`Failed to load face cascade: ${response.status}`);
    const data = new Uint8Array(await response.arrayBuffer());
    cv.FS_createDataFile("/", "haarcascade_frontalface_default.xml", data, true, false, false);
    const classifier = new cv.CascadeClassifier();
    if (!classifier.load("haarcascade_frontalface_default.xml")) {
      throw new Error("OpenCV could not load face cascade");
    }
    state.cvClassifier = classifier;
    state.cvCanvas = document.createElement("canvas");
    state.cvCanvas.width = 320;
    state.cvCanvas.height = 240;
    window.godotWebcamTrackerLatest.status = "opencv ready";
    return cv;
  })();
  return await state.cvReady;
}

function detectOpenCvPose(video) {
  const cv = window.cv;
  const canvas = state.cvCanvas;
  if (!cv || !state.cvClassifier || !canvas) return null;
  const ctx = canvas.getContext("2d", { willReadFrequently: true });
  ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
  const src = cv.imread(canvas);
  const gray = new cv.Mat();
  const faces = new cv.RectVector();
  const minSize = new cv.Size(32, 32);
  try {
    cv.cvtColor(src, gray, cv.COLOR_RGBA2GRAY, 0);
    cv.equalizeHist(gray, gray);
    state.cvClassifier.detectMultiScale(gray, faces, 1.12, 4, 0, minSize);
    if (faces.size() <= 0) return null;
    let best = faces.get(0);
    for (let i = 1; i < faces.size(); i += 1) {
      const candidate = faces.get(i);
      if (candidate.width * candidate.height > best.width * best.height) {
        best = candidate;
      }
    }
    return rectToPoseCm(best, canvas.width, canvas.height);
  } finally {
    src.delete();
    gray.delete();
    faces.delete();
    minSize.delete();
  }
}

async function listVideoDevices() {
  if (!navigator.mediaDevices || !navigator.mediaDevices.enumerateDevices) {
    return [];
  }
  const devices = await navigator.mediaDevices.enumerateDevices();
  return devices.filter((device) => device.kind === "videoinput");
}

function chooseVideoDevice(devices) {
  if (state.options.deviceId) {
    return state.options.deviceId;
  }
  const preferred = String(state.options.preferredLabel || "").toLowerCase();
  if (preferred) {
    const match = devices.find((device) => String(device.label || "").toLowerCase().includes(preferred));
    if (match) return match.deviceId;
  }
  const normalWebcam = devices.find((device) => {
    const label = String(device.label || "").toLowerCase();
    return label && !label.includes("realsense") && !label.includes("depth") && !label.includes("oak");
  });
  return normalWebcam ? normalWebcam.deviceId : "";
}

function describeVideoDevice(devices, deviceId) {
  const device = devices.find((entry) => entry.deviceId === deviceId);
  return device ? (device.label || device.deviceId || "selected camera") : "auto camera";
}

function shouldFallbackToOpenCv(err) {
  const text = String((err && (err.stack || err.message)) || err || "");
  return /activeTexture|webgl|gl_context|emscripten_gl/i.test(text);
}

function updatePreview() {
  const preview = ensurePreviewElement();
  if (!preview) return;
  const p = window.godotWebcamTrackerLatest;
  const lines = [
    p.status,
    `x=${p.x.toFixed(1)}cm y=${p.y.toFixed(1)}cm z=${p.z.toFixed(1)}cm`,
  ];
  if (state.options.debug) {
    const ageMs = state.lastFaceRectMs ? Date.now() - state.lastFaceRectMs : -1;
    const rect = state.lastFaceRect;
    lines.push(`runs=${state.trackingRuns.length}/s hits=${state.trackingHits.length}/s faces=${state.lastFaceCount}`);
    lines.push(`scan=${state.trackingCanvas ? `${state.trackingCanvas.width}x${state.trackingCanvas.height}` : "--"} target=${state.options.trackingFps}fps`);
    lines.push(`last=${ageMs >= 0 ? `${ageMs}ms` : "--"} box=${rect ? `${Math.round(rect.x)},${Math.round(rect.y)} ${Math.round(rect.width)}x${Math.round(rect.height)}` : "--"}`);
    lines.push(`remote=${state.remoteStatus} sent=${state.remoteRuns.length}/s`);
  }
  preview.textContent = lines.join("\n");
}

function updateFaceOverlay() {
  if (!state.overlay) return;
  state.overlay.replaceChildren();
  const rect = state.lastFaceRect;
  const fresh = rect && Date.now() - state.lastFaceRectMs <= state.options.holdLastPoseMs;
  if (!fresh) return;
  const box = document.createElement("div");
  box.style.cssText = [
    "position:absolute",
    `left:${(rect.x / Math.max(1, rect.frameWidth)) * 100}%`,
    `top:${(rect.y / Math.max(1, rect.frameHeight)) * 100}%`,
    `width:${(rect.width / Math.max(1, rect.frameWidth)) * 100}%`,
    `height:${(rect.height / Math.max(1, rect.frameHeight)) * 100}%`,
    "border:3px solid #28e6ff",
    "box-shadow:0 0 0 1px rgba(0,0,0,.7),0 0 18px rgba(40,230,255,.55)",
    "box-sizing:border-box",
  ].join(";");
  const label = document.createElement("div");
  label.textContent = window.godotWebcamTrackerLatest.status || "tracking";
  label.style.cssText = "position:absolute;left:0;top:-26px;padding:3px 6px;background:rgba(0,0,0,.72);color:#bdf7ff;font:12px/1.2 monospace;white-space:nowrap;";
  box.appendChild(label);
  state.overlay.appendChild(box);
}

async function loop() {
  if (!state.running) return;
  try {
    const video = state.video;
    if (video && video.readyState >= 2) {
      if (state.backend === "opencv") {
        await initOpenCvTracker();
        const pose = detectOpenCvPose(video);
        if (pose) {
          emit(true, pose, "opencv tracking");
        } else {
          emit(false, null, "opencv searching");
        }
      } else {
        const detectNow = performance.now();
        const detectInterval = 1000 / Math.max(1, state.options.trackingFps || 20);
        if (state.mediapipeLastDetectMs && detectNow - state.mediapipeLastDetectMs < detectInterval) {
          requestAnimationFrame(loop);
          return;
        }
        state.mediapipeLastDetectMs = detectNow;
        const result = state.landmarker.detectForVideo(video, detectNow);
	        const landmarks = result.faceLandmarks && result.faceLandmarks[0];
	        rememberTrackingRun(Boolean(landmarks), result.faceLandmarks ? result.faceLandmarks.length : 0);
	        if (landmarks) {
	          state.mediapipeNoFaceSinceMs = 0;
	          rememberLandmarkRect(landmarks, video.videoWidth || video.clientWidth || 640, video.videoHeight || video.clientHeight || 480);
	          emit(true, landmarksToPoseCm(landmarks), "mediapipe tracking");
	        } else {
          const now = performance.now();
          if (!state.mediapipeNoFaceSinceMs) state.mediapipeNoFaceSinceMs = now;
          const searchingMs = now - state.mediapipeNoFaceSinceMs;
          if ((state.options.backend === "auto" || state.options.backend === "mediapipe") && searchingMs >= 4000) {
            if (!state.mediapipeRecoveryAttempted && getWebGlInfo().webgl2) {
              state.mediapipeRecoveryAttempted = true;
              state.mediapipeDelegate = "GPU";
              if (state.landmarker && typeof state.landmarker.close === "function") state.landmarker.close();
              state.landmarker = null;
              state.mediapipeNoFaceSinceMs = now;
              emit(false, null, "mediapipe CPU found no face; retrying GPU");
              await initLandmarker();
            } else {
              emit(false, null, `mediapipe searching ${Math.round(searchingMs / 1000)}s (GPU retry complete)`);
            }
          } else {
            emit(false, null, `mediapipe searching ${Math.round(searchingMs / 1000)}s`);
          }
        }
      }
      updatePreview();
      updateFaceOverlay();
    }
  } catch (err) {
    console.warn("Browser MediaPipe tracker failed:", err);
    const detail = err && err.message ? err.message : String(err);
    emit(false, null, `mediapipe error: ${detail}`.slice(0, 180));
    state.running = false;
  }
  requestAnimationFrame(loop);
}

window.startGodotWebcamTracker = async function startGodotWebcamTracker(options = {}) {
  state.options = Object.assign(state.options, options || {});
  if (state.options.remoteEnabled) {
    connectRemoteHeadTracking(state.options.remoteUrl).catch((err) => {
      setRemoteStatus(`connect failed: ${err && err.message ? err.message : err}`);
      scheduleRemoteReconnect();
    });
  }
  if (state.running) return true;
  const generation = ++state.startGeneration;
  const checkCurrent = () => {
    if (generation !== state.startGeneration) {
      throw new DOMException("Webcam startup cancelled", "AbortError");
    }
  };
  state.lastPose = null;
  state.lastPoseMs = 0;
  state.mediapipeNoFaceSinceMs = 0;
  state.mediapipeRecoveryAttempted = false;
  state.mediapipeLastDetectMs = 0;
  state.mediapipeDelegate = "CPU";
  const video = placeVideoElement();
  window.godotWebcamTrackerLatest.status = "requesting camera";
  let devices = await listVideoDevices();
  checkCurrent();
  let deviceId = chooseVideoDevice(devices);
  const openCamera = async (selectedDeviceId) => {
    checkCurrent();
    const videoConstraints = {
      width: { ideal: 640 },
      height: { ideal: 480 },
    };
    if (selectedDeviceId) {
      videoConstraints.deviceId = { exact: selectedDeviceId };
    } else {
      videoConstraints.facingMode = "user";
    }
    const stream = await navigator.mediaDevices.getUserMedia({
      video: videoConstraints,
      audio: false,
    });
    if (generation !== state.startGeneration) {
      for (const track of stream.getTracks()) track.stop();
      checkCurrent();
    }
    return stream;
  };
  try {
    video.srcObject = await openCamera(deviceId);
  } catch (err) {
    checkCurrent();
    if (state.options.deviceId) throw err;
    devices = await listVideoDevices();
    checkCurrent();
    deviceId = chooseVideoDevice(devices);
    if (!deviceId) throw err;
    video.srcObject = await openCamera(deviceId);
  }
  window.godotWebcamTrackerLatest.status = `camera: ${describeVideoDevice(devices, deviceId)}`;
  await video.play();
  checkCurrent();
  if (state.trackingTask && typeof state.trackingTask.stop === "function") {
    state.trackingTask.stop();
    state.trackingTask = null;
  }
  if (state.trackingRaf) {
    window.cancelAnimationFrame(state.trackingRaf);
    state.trackingRaf = 0;
  }
  state.backend = state.options.backend;
  if (state.backend === "auto") {
    state.backend = "mediapipe";
  }
  if (state.backend === "opencv") {
    await initOpenCvTracker();
    checkCurrent();
    state.running = true;
    state.neutral = null;
    state.smoothed = { x: 0, y: 0, z: state.options.neutralZCm };
    loop();
  } else if (state.backend === "mediapipe") {
    try {
      await initLandmarker();
      checkCurrent();
      state.running = true;
      state.neutral = null;
      state.smoothed = { x: 0, y: 0, z: state.options.neutralZCm };
      loop();
    } catch (err) {
      checkCurrent();
      console.warn("MediaPipe initialization failed:", err);
      state.landmarker = null;
      const detail = err && err.message ? err.message : String(err);
      emit(false, null, `mediapipe init error: ${detail}`.slice(0, 180));
      throw err;
    }
  } else {
    state.backend = "trackingjs";
    state.running = true;
    state.neutral = null;
    state.smoothed = { x: 0, y: 0, z: state.options.neutralZCm };
    await initTrackingJsTracker(video);
    checkCurrent();
  }
  return true;
};

window.stopGodotWebcamTracker = function stopGodotWebcamTracker() {
  state.startGeneration += 1;
  state.running = false;
  if (state.trackingTask && typeof state.trackingTask.stop === "function") {
    state.trackingTask.stop();
  }
  state.trackingTask = null;
  if (state.trackingRaf) {
    window.cancelAnimationFrame(state.trackingRaf);
    state.trackingRaf = 0;
  }
  if (state.video?.srcObject) {
    for (const track of state.video.srcObject.getTracks()) track.stop();
    state.video.srcObject = null;
    state.video.pause();
  }
  disconnectRemoteHeadTracking();
  state.lastPose = null;
  state.lastPoseMs = 0;
  state.neutral = null;
  emit(false, null, "head tracking disabled");
  updatePreview();
  updateFaceOverlay();
  return true;
};

window.listGodotWebcamTrackerDevices = listVideoDevices;

window.connectGodotRemoteHeadTracking = connectRemoteHeadTracking;

window.disconnectGodotRemoteHeadTracking = disconnectRemoteHeadTracking;

window.setGodotRemoteHeadTrackingTransport = function setGodotRemoteHeadTrackingTransport(send, ready, label = "WebRTC data") {
  state.remoteTransportSend = typeof send === "function" ? send : null;
  state.remoteTransportReady = typeof ready === "function" ? ready : null;
  state.remoteTransportLabel = String(label || "WebRTC data");
  refreshRemoteTransport();
};

window.refreshGodotRemoteHeadTrackingTransport = refreshRemoteTransport;

window.sendGodotViewSettings = function sendGodotViewSettings(settings) {
  if (!settings || typeof settings !== "object") return false;
  return sendRemotePayload({ ...settings, type: "view_settings" });
};

window.getGodotRemoteHeadTrackingStatus = function getGodotRemoteHeadTrackingStatus() {
  return {
    enabled: state.remoteEnabled,
    connected: remoteConnectionOpen(),
    transport: remoteTransportOpen() ? "webrtc_datachannel" : remoteSocketOpen() ? "websocket" : "none",
    status: state.remoteStatus,
    url: state.remoteUrl,
    sends_per_sec: state.remoteRuns.length,
    buffered_amount: state.remoteWs ? state.remoteWs.bufferedAmount : 0,
  };
};

window.recenterGodotWebcamTracker = function recenterGodotWebcamTracker() {
  state.neutral = null;
  state.smoothed = { x: 0, y: 0, z: state.options.neutralZCm };
};

// Recenter remains available from the UI button. The R key belongs exclusively
// to Cartesian arm-up motion while keyboard arm control is enabled.
