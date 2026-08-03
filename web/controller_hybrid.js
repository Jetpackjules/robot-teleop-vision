    import { robotModuleReady } from "./robot_module_host.js";

    const robotModule = await robotModuleReady;
    const viewportVideo = document.getElementById("viewport-video");
    const viewportImage = document.getElementById("viewport-image");
    const hybridCanvas = document.getElementById("hybrid-canvas");
    const streamPresetSelect = document.getElementById("stream-preset");
    const statusDot = document.getElementById("status-dot");
    const statusTitle = document.getElementById("status-title");
    const statusLines = document.getElementById("status-lines");
    const startOverlay = document.getElementById("start-overlay");
    const startMessage = document.getElementById("start-message");
    const startButton = document.getElementById("start-button");
    const networkStats = document.getElementById("network-stats");
    const rgbdLatencyGraph = document.getElementById("rgbd-latency-graph");
    const rgbdLatencyValue = document.getElementById("rgbd-latency-value");
    const transportLink = document.getElementById("transport-link");
    const viewControlIds = ["orbit-span", "orbit-response", "orbit-distance", "dolly-gain", "inspect-fov", "head-height", "orbit-elevation"];
    // v9 delegates robot-specific settings to the active module.
    const viewSettingsVersion = 9;

    let peer = null;
    let trackingDataChannel = null;
    let viewSettingsDataChannel = null;
    let rgbdDataChannel = null;
    let av1Socket = null;
    let av1MediaSource = null;
    let av1SourceBuffer = null;
    let av1Queue = [];
    let av1ObjectUrl = "";
    let av1Watchdog = 0;
    let av1BytesReceived = 0;
    let av1StatsPrevious = null;
    let imageUrl = "";
    let streamMode = "quality";
    let streamState = "idle";
    let trackingState = "idle";
    let activeViewportMode = "hybrid";
    let startupRecenterGeneration = 0;
    let stats = { fps: 0, bitrateKbps: 0, framesDecoded: 0, framesDropped: 0, packetsLost: 0, lossPercent: 0, jitterMs: 0, rttMs: 0, connection: "--" };
    let previousStats = null;
    const rgbdLatencySamples = [];
    let lastRgbdLatencySampleMs = 0;
    let userStarted = false;
    const hybridMode = !window.location.pathname.startsWith("/frame-stream");
    const quickTunnel = window.location.hostname.endsWith(".trycloudflare.com");
    if (!hybridMode && quickTunnel) {
      // WebRTC media cannot reliably traverse a TCP-only Quick Tunnel. Avoid
      // leaving public viewers on its black, indefinitely-connecting route.
      window.location.replace("/controller.html");
    }
    transportLink.href = quickTunnel ? "/controller.html" : (hybridMode ? "/frame-stream" : "/");
    transportLink.textContent = quickTunnel ? "Reload Point Cloud" : (hybridMode ? "Frame Stream Fallback" : "Hybrid RGB-D View");

    function viewSettingsPayload(extra = {}) {
      const span = Number(document.getElementById("orbit-span").value);
      const response = Number(document.getElementById("orbit-response").value);
      const orbitDistance = Number(document.getElementById("orbit-distance").value);
      const payload = {
        inspect_enabled: document.getElementById("inspect-enabled").checked,
        yaw_gain: response,
        pitch_gain: response * 0.73,
        max_yaw: span * 0.5,
        max_pitch: Math.min(89, span * 0.28),
        focus_distance: 0.45,
        orbit_distance: orbitDistance,
        focus_vertical_offset: Number(document.getElementById("head-height").value),
        orbit_pitch_offset: Number(document.getElementById("orbit-elevation").value),
        dolly_enabled: document.getElementById("dolly-enabled").checked,
        dolly_gain: Number(document.getElementById("dolly-gain").value),
        min_distance: 0.2,
        max_distance: 3.0,
        fov: Number(document.getElementById("inspect-fov").value),
        white_background_enabled: document.getElementById("white-background-enabled").checked,
        persistent_temporal_reference_enabled: document.getElementById("persistent-temporal-reference-enabled").checked,
        full_rgb_frame_updates_enabled: document.getElementById("full-rgb-frame-updates-enabled").checked,
        settings_version: viewSettingsVersion,
        ...extra,
      };
      return robotModule.extendViewSettings?.(payload) || payload;
    }

    function updateViewLabels() {
      document.getElementById("orbit-span-value").textContent = `${document.getElementById("orbit-span").value} deg`;
      document.getElementById("orbit-response-value").textContent = document.getElementById("orbit-response").value;
      document.getElementById("orbit-distance-value").textContent = `${Number(document.getElementById("orbit-distance").value).toFixed(2)} m`;
      document.getElementById("dolly-gain-value").textContent = `${Number(document.getElementById("dolly-gain").value).toFixed(3)} m/cm`;
      document.getElementById("inspect-fov-value").textContent = `${document.getElementById("inspect-fov").value} deg`;
      document.getElementById("head-height-value").textContent = `${Number(document.getElementById("head-height").value).toFixed(2)} m`;
      document.getElementById("orbit-elevation-value").textContent = `${Number(document.getElementById("orbit-elevation").value).toFixed(0)} deg`;
    }

    function sendViewSettings(extra = {}) {
      updateViewLabels();
      const payload = viewSettingsPayload(extra);
      localStorage.setItem("robotTeleopViewSettings", JSON.stringify(payload));
      window.setGodotHybridViewSettings && window.setGodotHybridViewSettings(payload);
      return window.sendGodotViewSettings && window.sendGodotViewSettings(payload);
    }

    function restoreViewSettings() {
      let saved = null;
      try {
        saved = JSON.parse(localStorage.getItem("robotTeleopViewSettings") || localStorage.getItem("digitalWindowViewSettings") || "null");
      } catch (_) {}
      if (!saved || ![6, 7, 8, viewSettingsVersion].includes(saved.settings_version)) return updateViewLabels();
      document.getElementById("inspect-enabled").checked = saved.inspect_enabled === true;
      document.getElementById("orbit-span").value = String(Math.max(30, Math.min(360, (saved.max_yaw || 80) * 2)));
      document.getElementById("orbit-response").value = String(saved.yaw_gain || 2.5);
      document.getElementById("orbit-distance").value = String(saved.orbit_distance || 0.35);
      document.getElementById("dolly-enabled").checked = saved.dolly_enabled !== false;
      document.getElementById("dolly-gain").value = String(saved.dolly_gain ?? 0.008);
      document.getElementById("inspect-fov").value = String(Math.round(saved.fov || 55));
      document.getElementById("white-background-enabled").checked = saved.white_background_enabled === true;
      document.getElementById("head-height").value = String(saved.focus_vertical_offset ?? 0.35);
      document.getElementById("orbit-elevation").value = String(saved.orbit_pitch_offset ?? 0);
      document.getElementById("persistent-temporal-reference-enabled").checked = saved.persistent_temporal_reference_enabled !== false;
      document.getElementById("full-rgb-frame-updates-enabled").checked = saved.full_rgb_frame_updates_enabled === true;
      robotModule.restoreViewSettings?.(saved);
      updateViewLabels();
    }

    function normalizedStreamClick(event, element) {
      const mediaWidth = element.videoWidth || element.naturalWidth || 0;
      const mediaHeight = element.videoHeight || element.naturalHeight || 0;
      const rect = element.getBoundingClientRect();
      if (!mediaWidth || !mediaHeight || rect.width <= 0 || rect.height <= 0) return null;
      const scale = Math.min(rect.width / mediaWidth, rect.height / mediaHeight);
      const displayedWidth = mediaWidth * scale;
      const displayedHeight = mediaHeight * scale;
      const left = rect.left + (rect.width - displayedWidth) * 0.5;
      const top = rect.top + (rect.height - displayedHeight) * 0.5;
      const x = event.clientX - left;
      const y = event.clientY - top;
      if (x < 0 || y < 0 || x > displayedWidth || y > displayedHeight) return null;
      return [x / displayedWidth, y / displayedHeight];
    }

    function focusInspectAtStreamClick(event) {
      const coordinates = normalizedStreamClick(event, event.currentTarget);
      if (!coordinates) return;
      document.getElementById("inspect-enabled").checked = true;
      sendViewSettings({
        inspect_enabled: true,
        focus_pick_uv: coordinates,
        focus_pick_sent_unix_ms: Date.now(),
      });
    }

    const streamPresets = {
      latency: {
        label: "Latency",
        width: 960,
        fps: 30,
        jpegQuality: 60,
        source: "raw",
        webrtcBitrateKbps: 7000,
        videoBitrate: "7000k",
        av1Bitrate: "2200k",
        webmBitrate: "6000k",
      },
      quality: {
        label: "Quality",
        width: 1920,
        fps: 30,
        jpegQuality: 95,
        source: "raw",
        webrtcBitrateKbps: 24000,
        videoBitrate: "24000k",
        av1Bitrate: "6000k",
        webmBitrate: "18000k",
      },
    };

    const trackingPreset = {
      trackingWidth: 480,
      trackingHeight: 360,
      trackingFps: 20,
      trackingInitialScale: 3.2,
      trackingStepSize: 2,
      trackingEdgesDensity: 0.09,
    };

    function pageHost() {
      return window.location.hostname || "127.0.0.1";
    }

    function remoteUrl() {
      if (window.location.protocol === "https:") {
        return `wss://${window.location.host}/head-tracking`;
      }
      return `ws://${pageHost()}:8766`;
    }

    function streamBaseUrl() {
      const preset = currentPreset();
      if (window.location.protocol === "https:") {
        return `${window.location.origin}/stream?w=${preset.width}&q=${preset.jpegQuality}&fps=${preset.fps}`;
      }
      return `http://${pageHost()}:8780/stream?w=${preset.width}&q=${preset.jpegQuality}&fps=${preset.fps}`;
    }

    function videoUrl(codec = "h264") {
      const preset = currentPreset();
      const path = codec === "av1" ? "video-av1.webm" : "video.mp4";
      const bitrate = codec === "av1" ? preset.av1Bitrate : preset.videoBitrate;
      if (window.location.protocol === "https:") {
        return `${window.location.origin}/${path}?w=${preset.width}&q=${preset.jpegQuality}&fps=${preset.fps}&bitrate=${bitrate}&encoder=auto`;
      }
      return `https://${pageHost()}:8765/${path}?w=${preset.width}&q=${preset.jpegQuality}&fps=${preset.fps}&bitrate=${bitrate}&encoder=auto`;
    }

    function av1SocketUrl() {
      const preset = currentPreset();
      const scheme = window.location.protocol === "https:" ? "wss" : "ws";
      return `${scheme}://${window.location.host}/av1-stream?w=${preset.width}&fps=${preset.fps}&bitrate=${preset.av1Bitrate}`;
    }

    function browserSupportsAv1() {
      const type = 'video/webm; codecs="av01.0.05M.08"';
      return (typeof MediaSource !== "undefined" && MediaSource.isTypeSupported(type))
        || viewportVideo.canPlayType(type) !== "";
    }

    function currentPreset() {
      return streamPresets[streamPresetSelect.value] || streamPresets.quality;
    }

    function selectedViewportMode() {
      return hybridMode ? "hybrid" : "runtime";
    }

    async function waitForBridge() {
      for (let i = 0; i < 80; i += 1) {
        if (typeof window.startGodotWebcamTracker === "function") return true;
        await new Promise((resolve) => setTimeout(resolve, 100));
      }
      return false;
    }

    function showVideo() {
      hybridCanvas.hidden = true;
      viewportImage.hidden = true;
      viewportVideo.hidden = false;
    }

    function showImage() {
      hybridCanvas.hidden = true;
      viewportVideo.hidden = true;
      viewportImage.hidden = false;
    }

    function showHybrid() {
      viewportVideo.hidden = true;
      viewportImage.hidden = true;
      hybridCanvas.hidden = false;
    }

    function stopViewport() {
      window.stopGodotHybridRenderer && window.stopGodotHybridRenderer();
      if (peer) {
        peer.close();
        peer = null;
      }
      trackingDataChannel = null;
      viewSettingsDataChannel = null;
      rgbdDataChannel = null;
      window.refreshGodotRemoteHeadTrackingTransport && window.refreshGodotRemoteHeadTrackingTransport();
      if (av1Watchdog) {
        clearTimeout(av1Watchdog);
        av1Watchdog = 0;
      }
      if (av1Socket) {
        av1Socket.onopen = null;
        av1Socket.onmessage = null;
        av1Socket.onerror = null;
        av1Socket.onclose = null;
        av1Socket.close();
        av1Socket = null;
      }
      av1Queue = [];
      av1SourceBuffer = null;
      av1MediaSource = null;
      av1BytesReceived = 0;
      av1StatsPrevious = null;
      viewportVideo.pause();
      viewportVideo.removeAttribute("src");
      viewportVideo.srcObject = null;
      viewportVideo.load();
      if (av1ObjectUrl) {
        URL.revokeObjectURL(av1ObjectUrl);
        av1ObjectUrl = "";
      }
      if (imageUrl) {
        URL.revokeObjectURL(imageUrl);
        imageUrl = "";
      }
      viewportImage.removeAttribute("src");
      hybridCanvas.hidden = true;
      previousStats = null;
      stats = { fps: 0, bitrateKbps: 0, framesDecoded: 0, framesDropped: 0, packetsLost: 0, lossPercent: 0, jitterMs: 0, rttMs: 0, connection: "--" };
    }

    async function waitForIceGatheringComplete(pc) {
      if (pc.iceGatheringState === "complete") return;
      await new Promise((resolve) => {
        const check = () => {
          if (pc.iceGatheringState === "complete") {
            pc.removeEventListener("icegatheringstatechange", check);
            resolve();
          }
        };
        pc.addEventListener("icegatheringstatechange", check);
        setTimeout(resolve, 1500);
      });
    }

    function configureControlDataChannels(pc) {
      trackingDataChannel = pc.createDataChannel("head-tracking", { ordered: false, maxRetransmits: 0 });
      viewSettingsDataChannel = pc.createDataChannel("view-settings", { ordered: true });
      trackingDataChannel.bufferedAmountLowThreshold = 0;
      viewSettingsDataChannel.bufferedAmountLowThreshold = 0;
      const dataChannelReady = () => trackingDataChannel && trackingDataChannel.readyState === "open";
      const sendPeerControl = (payload) => {
        const channel = payload && payload.type === "view_settings" ? viewSettingsDataChannel : trackingDataChannel;
        if (!channel || channel.readyState !== "open" || channel.bufferedAmount > 16384) return false;
        channel.send(JSON.stringify(payload));
        return true;
      };
      window.setGodotRemoteHeadTrackingTransport && window.setGodotRemoteHeadTrackingTransport(
        sendPeerControl,
        dataChannelReady,
        "WebRTC data"
      );
      for (const channel of [trackingDataChannel, viewSettingsDataChannel]) {
        channel.addEventListener("open", () => {
          window.refreshGodotRemoteHeadTrackingTransport && window.refreshGodotRemoteHeadTrackingTransport();
          if (channel === viewSettingsDataChannel) sendViewSettings();
          updateStatus();
        });
        channel.addEventListener("close", () => {
          window.refreshGodotRemoteHeadTrackingTransport && window.refreshGodotRemoteHeadTrackingTransport();
          updateStatus();
        });
      }
    }

    async function startWebRtcViewport() {
      stopViewport();
      activeViewportMode = "runtime";
      if (!window.RTCPeerConnection) {
        startVideoFallback();
        return;
      }
      const preset = currentPreset();
      streamMode = `${preset.label} Native`;
      streamState = "connecting";
      updateStatus();
      const pc = new RTCPeerConnection({ iceServers: [{ urls: "stun:stun.l.google.com:19302" }] });
      peer = pc;
      showVideo();
      viewportVideo.muted = true;
      viewportVideo.playsInline = true;
      viewportVideo.autoplay = true;

      pc.addTransceiver("video", { direction: "recvonly" });
      configureControlDataChannels(pc);
      pc.ontrack = (event) => {
        const receiver = pc.getReceivers().find((candidate) => candidate.track === event.track);
        if (receiver && "playoutDelayHint" in receiver) receiver.playoutDelayHint = 0;
        if (receiver && "jitterBufferTarget" in receiver) receiver.jitterBufferTarget = 0;
        viewportVideo.srcObject = event.streams[0] || new MediaStream([event.track]);
        const play = viewportVideo.play();
        if (play && typeof play.catch === "function") play.catch(() => {});
      };
      pc.onconnectionstatechange = () => {
        streamState = `webrtc ${pc.connectionState}`;
        updateStatus();
        if (["failed", "closed", "disconnected"].includes(pc.connectionState)) {
          startVideoFallback();
        }
      };

      try {
        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);
        await waitForIceGatheringComplete(pc);
        const response = await fetch("/webrtc-offer", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            sdp: pc.localDescription.sdp,
            type: pc.localDescription.type,
            width: preset.width,
            fps: preset.fps,
            quality: preset.jpegQuality,
            source: preset.source,
            bitrateKbps: preset.webrtcBitrateKbps,
          }),
        });
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        await pc.setRemoteDescription(await response.json());
      } catch (err) {
        streamState = `webrtc failed: ${err && err.message ? err.message : err}`;
        updateStatus();
        startVideoFallback();
      }
    }

    async function startHybridControlPeer(rgbdPreset = null) {
      if (!window.RTCPeerConnection) return false;
      const pc = new RTCPeerConnection({ iceServers: [{ urls: "stun:stun.l.google.com:19302" }] });
      peer = pc;
      configureControlDataChannels(pc);
      if (rgbdPreset) {
        rgbdDataChannel = pc.createDataChannel("rgbd-stream", {
          ordered: false,
          maxRetransmits: 0,
        });
      }
      pc.onconnectionstatechange = () => {
        window.refreshGodotRemoteHeadTrackingTransport && window.refreshGodotRemoteHeadTrackingTransport();
        updateStatus();
      };
      try {
        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);
        await waitForIceGatheringComplete(pc);
        const response = await fetch("/webrtc-offer", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            sdp: pc.localDescription.sdp,
            type: pc.localDescription.type,
            source: rgbdPreset ? "rgbd_data" : "control_only",
            width: rgbdPreset ? rgbdPreset.width : undefined,
            fps: rgbdPreset ? rgbdPreset.fps : undefined,
            contextFps: rgbdPreset ? rgbdPreset.contextFps : undefined,
            detailFps: rgbdPreset ? rgbdPreset.detailFps : undefined,
            quality: rgbdPreset ? rgbdPreset.quality : undefined,
          }),
        });
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        await pc.setRemoteDescription(await response.json());
        return true;
      } catch (error) {
        if (peer === pc) peer = null;
        pc.close();
        window.refreshGodotRemoteHeadTrackingTransport && window.refreshGodotRemoteHeadTrackingTransport();
        console.warn("WebRTC control channel unavailable; using WebSocket tracking fallback", error);
        return false;
      }
    }

    async function startHybridViewport() {
      stopViewport();
      activeViewportMode = "hybrid";
      showHybrid();
      const forceHttpsRgbd = new URLSearchParams(window.location.search)
        .get("rgbd_transport") === "https";
      const forceWebSocketRgbd = new URLSearchParams(window.location.search)
        .get("rgbd_transport") === "websocket";
      const forceRelayedRgbd = forceHttpsRgbd || forceWebSocketRgbd;
      const preset = currentPreset();
      streamMode = `${preset.label} Hybrid`;
      streamState = "RGB-D connecting";
      updateStatus();
      if (typeof window.startGodotHybridRenderer !== "function") {
        throw new Error("hybrid RGB-D renderer did not load");
      }
      const hybridPreset = streamPresetSelect.value === "latency"
        ? { width: 384, fps: 60, contextFps: 30, detailFps: 60, quality: 74, geometryMode: "points", splatFill: 1.18 }
        : (quickTunnel
          // Plane-aware, keyframe-relative depth tiles carry the complete
          // 666x526 workspace crop at a full 30 Hz source cadence. Requesting a
          // larger image would only upscale the cropped sensor samples.
          // Whole-scene motion automatically falls back to a fresh lossless
          // keyframe, so the stream cannot accumulate temporal errors.
          ? { width: 666, fps: 30, contextFps: 36, detailFps: 36, quality: 68, geometryMode: "points", splatFill: 1.32 }
          : { width: 720, fps: 30, contextFps: 15, detailFps: 36, quality: 90, geometryMode: "points", splatFill: 1.32 });
      const persistentReference = document.getElementById(
        "persistent-temporal-reference-enabled"
      ).checked;
      const fullRgbFrames = document.getElementById(
        "full-rgb-frame-updates-enabled"
      ).checked;
      window.setGodotHybridViewSettings && window.setGodotHybridViewSettings(viewSettingsPayload({
        geometry_mode: hybridPreset.geometryMode,
        splat_fill: hybridPreset.splatFill,
        mesh_depth_delta: 0.035,
        mesh_max_edge: 0.055,
      }));
      // This async call creates all channels synchronously before its first
      // await, allowing the renderer to attach while ICE is negotiating.
      // The diagnostics-only query flag keeps control on WebRTC but sends
      // RGB-D over the public latest-frame endpoint. This lets a local browser
      // exercise the same Cloudflare path used when direct peer connectivity
      // is unavailable on a remote network.
      const peerPromise = startHybridControlPeer(
        forceRelayedRgbd || persistentReference ? null : hybridPreset
      );
      await Promise.all([
        window.startGodotHybridRenderer({
          canvas: hybridCanvas,
          ...hybridPreset,
          dataChannel: forceRelayedRgbd || persistentReference ? null : rgbdDataChannel,
          // Prefer direct unordered WebRTC when peer connectivity is possible.
          // A continuous latest-only WebSocket is the free public fallback;
          // HTTPS polling remains available as an explicit diagnostic mode.
          transport: forceHttpsRgbd ? "https" : "websocket",
          persistentReference,
          temporalColor: !fullRgbFrames,
        }),
        peerPromise,
      ]);
      streamState = "RGB-D rendering";
      updateStatus();
    }

    function startSelectedViewport() {
      return selectedViewportMode() === "hybrid" ? startHybridViewport() : startWebRtcViewport();
    }

    function recenterWhenTrackingStarts() {
      const generation = ++startupRecenterGeneration;
      const deadline = performance.now() + 5000;
      const attempt = () => {
        if (generation !== startupRecenterGeneration) return;
        const latest = window.godotWebcamTrackerLatest || {};
        if (latest.active === true) {
          window.recenterGodotWebcamTracker && window.recenterGodotWebcamTracker();
          window.recenterGodotHybridRenderer && window.recenterGodotHybridRenderer();
          window.setTimeout(() => sendViewSettings({ recenter: true }), 120);
          return;
        }
        if (performance.now() < deadline) window.setTimeout(attempt, 100);
      };
      attempt();
    }

    function startVideoFallback() {
      const useAv1 = browserSupportsAv1();
      if (useAv1) {
        startAv1Fallback();
        return;
      }
      startH264Fallback();
    }

    function startAv1Fallback() {
      stopViewport();
      showVideo();
      streamState = "AV1 GPU fallback";
      viewportVideo.muted = true;
      viewportVideo.playsInline = true;
      viewportVideo.autoplay = true;
      viewportVideo.onerror = () => startH264Fallback();

      const mimeType = 'video/webm; codecs="av01.0.05M.08"';
      av1MediaSource = new MediaSource();
      av1ObjectUrl = URL.createObjectURL(av1MediaSource);
      viewportVideo.src = av1ObjectUrl;
      const socket = new WebSocket(av1SocketUrl());
      av1Socket = socket;
      socket.binaryType = "arraybuffer";
      let receivedVideo = false;

      const failToH264 = () => {
        if (av1Socket !== socket) return;
        startH264Fallback();
      };
      const pump = () => {
        if (!av1SourceBuffer || av1SourceBuffer.updating || av1Queue.length === 0) return;
        try {
          av1SourceBuffer.appendBuffer(av1Queue.shift());
        } catch (_) {
          failToH264();
        }
      };
      av1MediaSource.addEventListener("sourceopen", () => {
        if (av1Socket !== socket || !av1MediaSource) return;
        try {
          av1SourceBuffer = av1MediaSource.addSourceBuffer(mimeType);
          av1SourceBuffer.addEventListener("updateend", () => {
            if (viewportVideo.buffered.length > 0) {
              const liveEdge = viewportVideo.buffered.end(viewportVideo.buffered.length - 1);
              if (liveEdge - viewportVideo.currentTime > 0.25) {
                viewportVideo.currentTime = Math.max(0, liveEdge - 0.16);
              }
            }
            pump();
          });
          pump();
        } catch (_) {
          failToH264();
        }
      }, { once: true });
      socket.onmessage = (event) => {
        if (!(event.data instanceof ArrayBuffer)) return;
        receivedVideo = true;
        av1BytesReceived += event.data.byteLength;
        if (av1Watchdog) {
          clearTimeout(av1Watchdog);
          av1Watchdog = 0;
        }
        av1Queue.push(event.data);
        pump();
      };
      socket.onerror = failToH264;
      socket.onclose = failToH264;
      av1Watchdog = setTimeout(() => {
        if (!receivedVideo) failToH264();
      }, 5000);
      const play = viewportVideo.play();
      if (play && typeof play.catch === "function") play.catch(() => {});
    }

    function startH264Fallback() {
      stopViewport();
      showVideo();
      streamState = "H.264 fallback";
      viewportVideo.muted = true;
      viewportVideo.playsInline = true;
      viewportVideo.autoplay = true;
      viewportVideo.src = `${videoUrl("h264")}&t=${Date.now()}`;
      viewportVideo.onerror = () => startDirectFallback();
      const play = viewportVideo.play();
      if (play && typeof play.catch === "function") play.catch(() => {});
    }

    function startDirectFallback() {
      stopViewport();
      showImage();
      streamState = "direct fallback";
      viewportImage.onerror = () => {
        streamState = "stream unavailable";
        updateStatus();
      };
      viewportImage.src = `${streamBaseUrl()}&t=${Date.now()}`;
    }

    async function startTracking() {
      trackingState = "starting";
      updateStatus();
      if (!await waitForBridge()) throw new Error("tracker bridge did not load");
      await window.startGodotWebcamTracker({
        ...trackingPreset,
        neutralZCm: 35,
        xScale: -1,
        zScale: 1,
        maxZM: 5,
        smoothing: 0.35,
        preview: false,
        debug: false,
        showVideo: false,
        videoContainerId: "video-preview",
        showFaceOverlay: false,
        holdLastPoseMs: 1200,
        preferredLabel: "ACER HD",
        backend: "mediapipe",
        remoteEnabled: true,
        remoteUrl: remoteUrl(),
        remoteMaxHz: 90,
      });
      trackingState = "tracking";
      window.setTimeout(() => sendViewSettings(), 150);
      recenterWhenTrackingStarts();
      updateStatus();
    }

    async function startController() {
      userStarted = true;
      startOverlay.hidden = true;
      try {
        await Promise.all([startSelectedViewport(), startTracking()]);
      } catch (err) {
        trackingState = `needs start: ${err && err.message ? err.message : err}`;
        startMessage.textContent = "Camera permission needs a tap.";
        startOverlay.hidden = false;
        updateStatus();
      }
    }

    async function collectStats() {
      if (activeViewportMode === "hybrid") {
        const hybrid = window.getGodotHybridRendererStatus ? window.getGodotHybridRendererStatus() : {};
        stats = {
          ...stats,
          fps: hybrid.sourceFps || 0,
          bitrateKbps: hybrid.bitrateKbps || 0,
          framesDecoded: hybrid.decoded || 0,
          framesDropped: hybrid.dropped || 0,
          connection: hybrid.transport || (
            peer && peer.connectionState === "connected"
              ? "WebRTC control + RGB-D WebSocket fallback"
              : "RGB-D WebSocket + tracking fallback"
          ),
        };
        if (!peer) return;
        const controlReport = await peer.getStats();
        controlReport.forEach((item) => {
          if (item.type !== "candidate-pair" || item.state !== "succeeded" || !(item.selected || item.nominated)) return;
          stats.rttMs = Math.max(0, (item.currentRoundTripTime || 0) * 1000);
        });
        return;
      }
      if (!peer) {
        if (av1Socket && av1Socket.readyState === WebSocket.OPEN) {
          const now = performance.now();
          const quality = viewportVideo.getVideoPlaybackQuality ? viewportVideo.getVideoPlaybackQuality() : null;
          const targetFps = currentPreset().fps;
          const mediaTime = viewportVideo.currentTime;
          if (av1StatsPrevious && now > av1StatsPrevious.timestamp) {
            const elapsed = now - av1StatsPrevious.timestamp;
            stats.bitrateKbps = ((av1BytesReceived - av1StatsPrevious.bytes) * 8) / elapsed;
            stats.fps = Math.min(targetFps, Math.max(0, (mediaTime - av1StatsPrevious.mediaTime) * targetFps * 1000 / elapsed));
          }
          stats.framesDecoded = Math.max(0, Math.round(mediaTime * targetFps));
          stats.framesDropped = quality ? quality.droppedVideoFrames : 0;
          stats.connection = "Cloudflare WebSocket AV1";
          av1StatsPrevious = { timestamp: now, bytes: av1BytesReceived, mediaTime };
        }
        return;
      }
      const report = await peer.getStats();
      let inbound = null;
      let selectedPair = null;
      report.forEach((item) => {
        if (item.type === "inbound-rtp" && item.kind === "video") inbound = item;
        if (item.type === "candidate-pair" && item.state === "succeeded" && (item.selected || item.nominated)) selectedPair = item;
      });
      if (inbound) {
        const now = inbound.timestamp;
        let bitrateKbps = 0;
        let lossPercent = stats.lossPercent || 0;
        if (previousStats && now > previousStats.timestamp) {
          bitrateKbps = ((inbound.bytesReceived - previousStats.bytesReceived) * 8) / (now - previousStats.timestamp);
          const receivedDelta = Math.max(0, (inbound.packetsReceived || 0) - previousStats.packetsReceived);
          const lostDelta = Math.max(0, (inbound.packetsLost || 0) - previousStats.packetsLost);
          const packetDelta = receivedDelta + lostDelta;
          lossPercent = packetDelta ? (lostDelta * 100) / packetDelta : 0;
        }
        stats = {
          ...stats,
          fps: inbound.framesPerSecond || stats.fps || 0,
          bitrateKbps: Math.max(0, bitrateKbps),
          framesDecoded: inbound.framesDecoded || 0,
          framesDropped: inbound.framesDropped || 0,
          packetsLost: inbound.packetsLost || 0,
          lossPercent,
          jitterMs: Math.max(0, (inbound.jitter || 0) * 1000),
        };
        previousStats = {
          timestamp: now,
          bytesReceived: inbound.bytesReceived,
          packetsReceived: inbound.packetsReceived || 0,
          packetsLost: inbound.packetsLost || 0,
        };
      }
      if (selectedPair) {
        const local = report.get(selectedPair.localCandidateId);
        const remoteCandidate = report.get(selectedPair.remoteCandidateId);
        stats.rttMs = Math.max(0, (selectedPair.currentRoundTripTime || 0) * 1000);
        const localType = local && local.candidateType ? local.candidateType : "unknown";
        const remoteType = remoteCandidate && remoteCandidate.candidateType ? remoteCandidate.candidateType : "unknown";
        const protocol = (local && local.protocol) || "udp";
        stats.connection = `${localType}->${remoteType} ${protocol}`;
      }
    }

    function updateRgbdLatencyGraph(hybrid) {
      const now = performance.now();
      if (now - lastRgbdLatencySampleMs >= 500) {
        lastRgbdLatencySampleMs = now;
        rgbdLatencySamples.push({
          timestamp: now,
          latency: hybrid.captureAgeMs == null || hybrid.captureAgeMs < 0 ? null : Number(hybrid.captureAgeMs),
          packetAge: hybrid.packetAgeMs == null || hybrid.packetAgeMs < 0 ? null : Number(hybrid.packetAgeMs),
          stalled: hybrid.stalled === true,
        });
      }
      const cutoff = now - 120000;
      while (rgbdLatencySamples.length && rgbdLatencySamples[0].timestamp < cutoff) rgbdLatencySamples.shift();

      const context = rgbdLatencyGraph.getContext("2d");
      const width = rgbdLatencyGraph.width;
      const height = rgbdLatencyGraph.height;
      context.clearRect(0, 0, width, height);
      context.fillStyle = "#101010";
      context.fillRect(0, 0, width, height);

      const values = rgbdLatencySamples
        .map((sample) => sample.latency)
        .filter((value) => Number.isFinite(value));
      const peak = values.length ? Math.max(...values) : 0;
      const scales = [250, 500, 1000, 2000, 5000, 10000, 30000];
      const scaleMax = scales.find((value) => value >= peak) || Math.max(30000, peak);
      context.strokeStyle = "#292929";
      context.lineWidth = 1;
      for (let row = 1; row < 4; row += 1) {
        const y = Math.round((height * row) / 4) + 0.5;
        context.beginPath();
        context.moveTo(0, y);
        context.lineTo(width, y);
        context.stroke();
      }

      for (const sample of rgbdLatencySamples) {
        if (!sample.stalled) continue;
        const x = ((sample.timestamp - cutoff) / 120000) * width;
        context.fillStyle = "rgba(255, 77, 77, 0.22)";
        context.fillRect(Math.max(0, x - 1), 0, 3, height);
      }

      context.strokeStyle = "#72d6ff";
      context.lineWidth = 1.5;
      context.beginPath();
      let drawing = false;
      for (const sample of rgbdLatencySamples) {
        if (!Number.isFinite(sample.latency)) {
          drawing = false;
          continue;
        }
        const x = ((sample.timestamp - cutoff) / 120000) * width;
        const y = height - Math.min(height - 1, (sample.latency / scaleMax) * (height - 2)) - 1;
        if (!drawing) {
          context.moveTo(x, y);
          drawing = true;
        } else {
          context.lineTo(x, y);
        }
      }
      context.stroke();
      context.fillStyle = "#888";
      context.font = "9px ui-monospace, monospace";
      context.fillText(`${scaleMax}ms`, 3, 10);
      context.fillText("0", 3, height - 3);

      if (hybrid.stalled) {
        rgbdLatencyValue.textContent = `STALLED · packet ${Math.round(hybrid.packetAgeMs)}ms`;
        rgbdLatencyValue.style.color = "#ff6b6b";
      } else if (hybrid.captureAgeMs != null && hybrid.captureAgeMs >= 0) {
        rgbdLatencyValue.textContent = `${Math.round(hybrid.captureAgeMs)}ms · packet ${Math.round(Math.max(0, hybrid.packetAgeMs || 0))}ms`;
        rgbdLatencyValue.style.color = "";
      } else {
        rgbdLatencyValue.textContent = "—";
        rgbdLatencyValue.style.color = "";
      }
    }

    function updateStatus() {
      const latest = window.godotWebcamTrackerLatest || {};
      const remote = window.getGodotRemoteHeadTrackingStatus ? window.getGodotRemoteHeadTrackingStatus() : {};
      const active = latest.active === true;
      const av1Connected = av1Socket && av1Socket.readyState === WebSocket.OPEN && viewportVideo.currentTime > 0;
      const hybrid = window.getGodotHybridRendererStatus ? window.getGodotHybridRendererStatus() : {};
      const connected = activeViewportMode === "hybrid" ? hybrid.connected === true : (peer && peer.connectionState === "connected") || av1Connected;
      const hybridStalled = activeViewportMode === "hybrid" && hybrid.stalled === true;
      statusDot.className = "dot " + (hybridStalled ? "bad" : connected && active ? "ok" : connected ? "" : "bad");
      statusTitle.textContent = hybridStalled ? "Stream stalled" : connected && active ? "Connected" : connected ? "Stream connected" : "Connecting";
      const size = activeViewportMode === "hybrid"
        ? (hybrid.width > 0 ? `${hybrid.width}x${hybrid.height}` : "--")
        : (viewportVideo.videoWidth > 0 ? `${viewportVideo.videoWidth}x${viewportVideo.videoHeight}` : "--");
      const hasStreamMeasurement = activeViewportMode === "hybrid"
        ? hybrid.received > 0 || hybrid.connected === true
        : stats.framesDecoded > 0;
      const fps = hasStreamMeasurement ? `${Math.round(Math.max(0, stats.fps || 0))} fps` : "-- fps";
      const bitrate = hasStreamMeasurement ? `${Math.round(Math.max(0, stats.bitrateKbps || 0))} kbps` : "-- kbps";
      networkStats.textContent = [
        `Resolution  ${size} @ ${fps}`,
        `Bitrate     ${bitrate}`,
        `RTT / jitter ${stats.rttMs.toFixed(1)} / ${stats.jitterMs.toFixed(1)} ms`,
        `Packet loss ${stats.lossPercent.toFixed(2)}% (${stats.packetsLost} total)`,
        `Frames      ${stats.framesDecoded} decoded / ${stats.framesDropped} dropped`,
        `Route       ${stats.connection}`,
        ...(activeViewportMode === "hybrid" ? [
          `RGB-D age   ${hybrid.captureAgeMs == null || hybrid.captureAgeMs < 0 ? "--" : `${Math.round(hybrid.captureAgeMs)}ms`} | packet ${hybrid.packetAgeMs == null || hybrid.packetAgeMs < 0 ? "--" : `${Math.round(hybrid.packetAgeMs)}ms`} | cameras ${hybrid.cameras || 0}`,
          `Decode ${hybrid.decodeMs ? `${hybrid.decodeMs.toFixed(1)}ms` : "--"} | render ${hybrid.renderFps ? `${Math.round(hybrid.renderFps)} fps` : "--"}`,
          `Camera FPS  ${Object.entries(hybrid.cameraFps || {}).map(([id, rate]) => `${id.slice(-4)}:${Math.round(rate)}`).join("  ") || "--"}`,
        ] : []),
      ].join("\n");
      statusLines.textContent = [
        `Stream ${streamMode} | ${size} | ${fps} | ${bitrate}`,
        `Tracking ${latest.status || trackingState} | ${remote.connected ? "bridge on" : "bridge ..."}`,
        robotModule.statusSummary?.(),
      ].filter(Boolean).join("\n");
      robotModule.renderStatus?.();
      if (activeViewportMode === "hybrid") updateRgbdLatencyGraph(hybrid);
    }

    streamPresetSelect.addEventListener("change", () => {
      if (userStarted) startSelectedViewport();
      updateStatus();
    });
    document.getElementById("persistent-temporal-reference-enabled").addEventListener("change", (event) => {
      sendViewSettings();
      if (userStarted) startSelectedViewport();
      event.currentTarget.blur();
    });
    document.getElementById("full-rgb-frame-updates-enabled").addEventListener("change", (event) => {
      sendViewSettings();
      if (userStarted) startSelectedViewport();
      event.currentTarget.blur();
    });
    document.getElementById("restart").addEventListener("click", startController);
    document.getElementById("recenter").addEventListener("click", () => {
      window.recenterGodotWebcamTracker && window.recenterGodotWebcamTracker();
      window.recenterGodotHybridRenderer && window.recenterGodotHybridRenderer();
    });
    for (const id of viewControlIds) {
      const control = document.getElementById(id);
      control.addEventListener("input", () => sendViewSettings());
      control.addEventListener("change", () => control.blur());
    }
    for (const id of ["inspect-enabled", "dolly-enabled", "white-background-enabled"]) {
      const control = document.getElementById(id);
      control.addEventListener("change", () => {
        sendViewSettings();
        control.blur();
      });
    }
    document.getElementById("inspect-recenter").addEventListener("click", () => sendViewSettings({ recenter: true }));
    viewportVideo.addEventListener("click", focusInspectAtStreamClick);
    viewportImage.addEventListener("click", focusInspectAtStreamClick);
    hybridCanvas.addEventListener("click", (event) => {
      if (!window.focusGodotHybridRendererAt || !window.focusGodotHybridRendererAt(event.clientX, event.clientY)) return;
      document.getElementById("inspect-enabled").checked = true;
      sendViewSettings({ inspect_enabled: true });
    });
    startButton.addEventListener("click", startController);

    setInterval(() => {
      collectStats().catch(() => {});
      updateStatus();
    }, 500);

    restoreViewSettings();
    robotModule.bind?.({ sendViewSettings, updateStatus });
    robotModule.start?.({ sendViewSettings, updateStatus });
    startController();
