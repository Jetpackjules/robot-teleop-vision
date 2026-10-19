import RFB from "/godot/core/rfb.js";

const video = document.getElementById("video");
const status = document.getElementById("status");
const notice = document.getElementById("notice");
let aborter, rfb, mediaURL, source, statsTimer, inputTimer, controlHeartbeat, generation = 0;
let controlsConnected = false;

// noVNC handles input coordinates, keyboard layouts, and desktop resizing. It
// requests one initial image, then video takes over. Avoid sending the desktop
// twice. This adapter is limited to this page and falls back visibly if the
// installed noVNC version no longer exposes the two message writers.
const messages = RFB.messages;
if (!messages?.fbUpdateRequest || !messages?.enableContinuousUpdates) {
  throw new Error("This noVNC version needs the standard desktop view");
}
const requestUpdate = messages.fbUpdateRequest;
messages.fbUpdateRequest = (socket, incremental, ...area) => {
  if (!incremental) requestUpdate(socket, false, ...area);
};
const continuousUpdates = messages.enableContinuousUpdates;
messages.enableContinuousUpdates = (socket, enabled, ...area) => {
  if (!enabled) continuousUpdates(socket, false, ...area);
};

function showError(error) {
  status.textContent = "Video unavailable";
  notice.hidden = false;
  notice.textContent = `${error.message || error}. Reconnect or use Standard desktop.`;
}

async function connect() {
  const current = ++generation;
  aborter?.abort();
  clearInterval(statsTimer);
  clearTimeout(inputTimer);
  clearInterval(controlHeartbeat);
  controlsConnected=false;
  rfb?.disconnect();
  if (mediaURL) URL.revokeObjectURL(mediaURL);
  aborter = new AbortController();
  const signal = aborter.signal;
  notice.hidden = true;
  status.textContent = "Connecting…";
  try {
    const response = await fetch("/godot/desktop-status.json", {signal, cache:"no-store"});
    if (!response.ok) throw new Error(`Desktop setup returned ${response.status}`);
    const config = await response.json();
    statsTimer=setInterval(async () => {
      try {
        const refreshed=await fetch("/godot/desktop-status.json",{signal,cache:"no-store"});
        if (refreshed.ok && current===generation) Object.assign(config,await refreshed.json());
      } catch (_) { /* The video connection handles disconnect reporting. */ }
    },2000);
    let lastVideoTime = performance.now(), lastPresented = 0;
    const videoStats = (now, frame) => {
      if (current !== generation) return;
      if (now-lastVideoTime >= 1000) {
        const fps = Math.round((frame.presentedFrames-lastPresented)*1000/(now-lastVideoTime));
        const end = video.buffered.length ? video.buffered.end(video.buffered.length-1) : video.currentTime;
        const scene=config.scene_fps == null ? "" : ` · scene ${config.scene_fps} fps`;
        status.textContent = `${config.size} · GPU H.264 · video ${fps}/${config.fps} fps${scene} · ${Math.max(0,end-video.currentTime).toFixed(2)} s buffer${controlsConnected ? "" : " · controls reconnecting"}`;
        lastVideoTime=now;
        lastPresented=frame.presentedFrames;
      }
      video.requestVideoFrameCallback(videoStats);
    };
    if (video.requestVideoFrameCallback) video.requestVideoFrameCallback(videoStats);
    const mime = `video/mp4; codecs="${config.codec}"`;
    if (!window.MediaSource || !MediaSource.isTypeSupported(mime)) throw new Error("H.264 playback is unavailable in this browser");
    const controls = document.getElementById("controls");
    const scheme = location.protocol === "https:" ? "wss:" : "ws:";
    const connectControls = () => {
      if (current !== generation) return;
      controls.replaceChildren();
      const input = new RFB(controls, `${scheme}//${location.host}/godot/websockify`);
      rfb=input;
      input.scaleViewport=true;
      input.resizeSession=false;
      input.viewOnly=false;
      input.addEventListener("connect",() => {
        controlsConnected=true;
        clearInterval(controlHeartbeat);
        // An idle control-only WebSocket needs a heartbeat through proxies.
        // One pixel every 15 s avoids restarting full desktop image updates.
        controlHeartbeat=setInterval(() => {
          if (controlsConnected && current===generation) requestUpdate(input._sock,false,0,0,1,1);
        },15000);
      });
      input.addEventListener("disconnect",() => {
        if (current!==generation || rfb!==input) return;
        controlsConnected=false;
        clearInterval(controlHeartbeat);
        inputTimer=setTimeout(connectControls,1500);
      });
      input.addEventListener("securityfailure",() => showError(new Error("Desktop control connection was rejected")));
    };
    connectControls();
    source = new MediaSource();
    mediaURL = URL.createObjectURL(source);
    video.src = mediaURL;
    await new Promise(resolve => source.addEventListener("sourceopen", resolve, {once:true}));
    if (current !== generation) return;
    const buffer = source.addSourceBuffer(mime);
    const queue = [];
    let queuedBytes = 0;
    const pump = () => {
      if (buffer.updating || current !== generation) return;
      if (video.buffered.length) {
        const end = video.buffered.end(video.buffered.length-1);
        const lag = end-video.currentTime;
        if (lag > 0.45) video.currentTime = Math.max(video.buffered.start(0),end-0.12);
        if (video.currentTime > 4 && buffer.buffered.length && buffer.buffered.start(0)<video.currentTime-3) {
          buffer.remove(buffer.buffered.start(0),video.currentTime-2);
          return;
        }
      }
      if (queue.length) {
        const next = queue.shift();
        queuedBytes -= next.byteLength;
        buffer.appendBuffer(next);
      }
    };
    buffer.addEventListener("updateend",pump);
    buffer.addEventListener("error",() => { aborter.abort(); showError(new Error("Video decoder error")); });
    const stream = await fetch("/godot/desktop.mp4", {signal,cache:"no-store"});
    if (!stream.ok) throw new Error(`Desktop encoder returned ${stream.status}`);
    const reader = stream.body.getReader();
    status.textContent = `${config.size} · GPU H.264 · ${config.fps} fps video · mouse and keyboard enabled`;
    video.play().catch(() => {});
    while (current === generation) {
      const {value,done} = await reader.read();
      if (done) throw new Error("Desktop stream ended");
      queue.push(value);
      queuedBytes += value.byteLength;
      if (queuedBytes > 4*1024*1024) throw new Error("Connection is falling behind");
      pump();
    }
  } catch (error) {
    if (current === generation && error.name !== "AbortError") {
      aborter.abort();
      showError(error);
    }
  }
}
document.getElementById("reconnect").addEventListener("click",connect);
window.addEventListener("pagehide",() => {
  generation++;
  clearInterval(statsTimer); clearInterval(controlHeartbeat); clearTimeout(inputTimer);
  aborter?.abort(); rfb?.disconnect();
});
connect();
