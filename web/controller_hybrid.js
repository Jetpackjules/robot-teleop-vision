    import { robotModuleReady } from "./robot_module_host.js";
    import { EpisodeRecordingControl } from './episode_recording.js?v=20261025_trials';
    import { TrialController } from './trial_controller.js?v=20261025_trials';
    import { CapturePresetControl, WorkspaceStreamControl, nativeStreamPreset } from "./capture_presets.js?v=20261026_transport_recovery";
    import { workspaceCropBounds } from './workspace_crop.js?v=20261021_original_rgb';
    import { MODELED_WORKSPACE_DEFAULTS, modeledWorkspace } from './modeled_workspace.js?v=20261021_original_rgb';

    const robotModule = await robotModuleReady;
    const recordingRoot = document.getElementById('episode-recording');
    const episodeRecording = recordingRoot ? new EpisodeRecordingControl(recordingRoot,
      {client: window.robotTeleopPageId}) : null;
    window.addEventListener('pagehide', () => episodeRecording?.close());
    const workspaceStreamControl = new WorkspaceStreamControl(remoteUrl());
    window.addEventListener('pagehide', () => workspaceStreamControl.close());
    const viewportVideo = document.getElementById("viewport-video");
    const viewportImage = document.getElementById("viewport-image");
    const hybridCanvas = document.getElementById("hybrid-canvas");
    const rgbCameraCanvas = document.getElementById("rgb-camera-canvas");
    const displayModeSelect = document.getElementById("display-mode");
    const streamPresetSelect = document.getElementById("stream-preset");
    const rgbCameraControl = document.getElementById("rgb-camera-control");
    const rgbCameraSelect = document.getElementById("rgb-camera-select");
    let selectedRgbCamera = '';
    let rgbCameraListLoading = false;
    let rgbCameraListUpdated = 0;
    let rgbCameraListSignature = '';
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
    const advancedSetup = document.getElementById("advanced-setup");
    const siteSettingsStatus = document.getElementById("site-settings-status");
    const viewControlIds = [
      "orbit-span", "orbit-response", "orbit-distance", "dolly-gain", "inspect-fov",
      "head-height", "orbit-elevation", "workspace-surface-offset", "workspace-surface-size",
      "workspace-surface-opacity",
    ];
    // v13 adds GPU mesh views while preserving older saved camera views.
    const viewSettingsVersion = 16;
    let workspaceCropConfigured = false;
    let lastValidCrop = {workspace_crop_min: [-0.6, -0.45, -0.6], workspace_crop_max: [0.6, 0.45, 0.6]};
    const modeledFields = ['width','depth','table-height','heading','clearance','height','light-angle','x','y','z'];
    let lastValidModeled = {...MODELED_WORKSPACE_DEFAULTS};
    let modeledWorkspaceConfigured = false;

    function modeledViewSettings() {
      const value = name => document.getElementById(`modeled-workspace-${name}`).valueAsNumber;
      return {modeled_workspace_enabled: document.getElementById('modeled-workspace-enabled').checked,
        modeled_workspace_center: ['x','y','z'].map(value),modeled_workspace_width:value('width'),
        modeled_workspace_depth:value('depth'),modeled_workspace_table_height:value('table-height'),modeled_workspace_heading:value('heading'),
        modeled_workspace_clearance:value('clearance')*.001,modeled_workspace_height:value('height'),
        modeled_workspace_light_angle:value('light-angle'),
        scene_lighting_enabled:document.getElementById('scene-lighting-enabled').checked,
        scene_shadows_enabled:document.getElementById('scene-shadows-enabled').checked};
    }

    function restoreModeledSettings(saved = {}) {
      const candidate = {...MODELED_WORKSPACE_DEFAULTS,...saved};
      const valid = modeledWorkspace({...candidate,modeled_workspace_enabled:true});
      const settings = valid ? candidate : {...MODELED_WORKSPACE_DEFAULTS};
      document.getElementById('modeled-workspace-enabled').checked = valid && saved.modeled_workspace_enabled === true;
      document.getElementById('scene-lighting-enabled').checked=settings.scene_lighting_enabled!==false;
      document.getElementById('scene-shadows-enabled').checked=settings.scene_shadows_enabled!==false;
      for (const name of modeledFields) {
        const value = ['x','y','z'].includes(name) ? settings.modeled_workspace_center[['x','y','z'].indexOf(name)]
          : settings[`modeled_workspace_${name.replaceAll('-','_')}`] * (name === 'clearance' ? 1000 : 1);
        document.getElementById(`modeled-workspace-${name}`).value = String(value);
      }
      modeledWorkspaceConfigured = Array.isArray(saved.modeled_workspace_center);
      lastValidModeled = modeledViewSettings();
      updateModeledControls();
    }

    function updateModeledControls() {
      const settings = modeledViewSettings(),valid = modeledWorkspace({...settings,modeled_workspace_enabled:true});
      document.getElementById('modeled-workspace-options').hidden = !settings.modeled_workspace_enabled;
      for (const name of modeledFields) {
        const input = document.getElementById(`modeled-workspace-${name}`),n=input.valueAsNumber;
        input.setCustomValidity(Number.isFinite(n) && n >= Number(input.min) && n <= Number(input.max)
          ? '' : 'Enter a value within the displayed range.');
      }
      document.getElementById('modeled-workspace-status').textContent = !settings.modeled_workspace_enabled ? ''
        : valid ? 'Artificial table and floor; live objects retain native detail. Server cropping is requested for this view.'
          : 'Enter valid dimensions; the last valid placement stays active.';
      return Boolean(valid);
    }

    function alignModeledTable(source='auto') {
      const placement = window.alignGodotModeledWorkspace?.({source});
      if (!placement) {
        document.getElementById('modeled-workspace-status').textContent = source==='robot'
          ? 'A calibrated arm base is not available. Use Edit Table in 3D or place the table at the orbit point.'
          : 'Open a connected 3D view, then double-click the real tabletop.';
        return false;
      }
      restoreModeledSettings({...modeledViewSettings(),...placement});
      modeledWorkspaceConfigured = true;
      return true;
    }

    function cropViewSettings() {
      const values = bound => ['x', 'y', 'z'].map(axis =>
        document.getElementById(`workspace-crop-${bound}-${axis}`).valueAsNumber);
      return {workspace_crop_enabled: document.getElementById('workspace-crop-enabled').checked,
        workspace_crop_show_box: document.getElementById('workspace-crop-show-box').checked,
        workspace_crop_min: values('min'), workspace_crop_max: values('max'),
        workspace_crop_rotation:['x','y','z'].map(axis=>document.getElementById(`workspace-crop-rotation-${axis}`).valueAsNumber)};
    }

    function restoreCropSettings(saved = {}) {
      const valid = workspaceCropBounds({...saved, workspace_crop_enabled: true});
      if (valid) for (const bound of ['min', 'max']) ['x', 'y', 'z'].forEach((axis, i) => {
        document.getElementById(`workspace-crop-${bound}-${axis}`).value = String(saved[`workspace_crop_${bound}`][i]);
      });
      if(valid) ['x','y','z'].forEach((axis,i)=>document.getElementById(`workspace-crop-rotation-${axis}`).value=String((saved.workspace_crop_rotation||[0,0,0])[i]));
      workspaceCropConfigured = Boolean(valid);
      document.getElementById('workspace-crop-enabled').checked = Boolean(valid && saved.workspace_crop_enabled);
      document.getElementById('workspace-crop-show-box').checked = saved.workspace_crop_show_box === true;
      updateCropControls();
    }

    function updateCropControls() {
      const settings = cropViewSettings(), enabled = settings.workspace_crop_enabled;
      document.getElementById('workspace-crop-options').hidden = !enabled;
      const valid = workspaceCropBounds({...settings, workspace_crop_enabled: true});
      for (const bound of ['min', 'max']) ['x', 'y', 'z'].forEach((axis, i) => {
        const control = document.getElementById(`workspace-crop-${bound}-${axis}`);
        const lo = settings.workspace_crop_min[i], hi = settings.workspace_crop_max[i];
        control.setCustomValidity(Number.isFinite(lo) && Number.isFinite(hi) && lo < hi
          && Math.abs(lo) <= 20 && Math.abs(hi) <= 20 ? '' : 'Enter a minimum below the maximum, between -20 and 20 metres.');
      });
      document.getElementById('workspace-crop-status').textContent = enabled
        ? valid ? 'Native detail retained inside the box. Save site defaults to reuse it in other browsers.'
          : 'Enter valid bounds; the last valid crop stays active.' : '';
      return !enabled || Boolean(valid);
    }

    function centerCropOnOrbit() {
      const bounds = window.centerGodotHybridWorkspaceCrop?.();
      if (!bounds) {
        document.getElementById('workspace-crop-status').textContent = 'Open a connected 3D view to center the box.';
        return false;
      }
      restoreCropSettings({...cropViewSettings(), ...bounds});
      return true;
    }

    let peer = null;
    let capturePresetControl = null;
    let viewportGeneration = 0;
    let retainedNavigation = null;
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
    let startupNavigation = null;
    let startupNavigationPending = false;
    const hybridMode = !window.location.pathname.startsWith("/frame-stream");
    const displayModes = hybridMode
      ? ["point_cloud", "shader_mesh", "independent_mesh", "unified_mesh", "blendpcr", "rgb_camera"]
      : ["point_cloud", "rgb_camera"];
    const defaultDisplayMode = hybridMode ? "independent_mesh" : "point_cloud";
    for (const option of displayModeSelect.options) {
      option.hidden = option.disabled = !displayModes.includes(option.value);
    }
    displayModeSelect.value = defaultDisplayMode;
    const quickTunnel = window.location.hostname.endsWith(".trycloudflare.com");
    if (!hybridMode && quickTunnel) {
      // WebRTC media cannot reliably traverse a TCP-only Quick Tunnel. Avoid
      // leaving public viewers on its black, indefinitely-connecting route.
      window.location.replace("/controller.html");
    }
    transportLink.href = quickTunnel ? "/controller.html" : (hybridMode ? "/frame-stream" : "/");
    transportLink.textContent = quickTunnel ? "Reload Point Cloud" : (hybridMode ? "Frame Stream Fallback" : "Hybrid RGB-D View");

    function viewSettingsPayload(extra = {}) {
      const crop = cropViewSettings();
      if (workspaceCropBounds({...crop, workspace_crop_enabled: true})) {
        lastValidCrop = {workspace_crop_min: crop.workspace_crop_min, workspace_crop_max: crop.workspace_crop_max,workspace_crop_rotation:crop.workspace_crop_rotation};
      } else Object.assign(crop, lastValidCrop);
      const modeled = modeledViewSettings();
      if (modeledWorkspace({...modeled,modeled_workspace_enabled:true})) lastValidModeled = modeled;
      else Object.assign(modeled,lastValidModeled,{modeled_workspace_enabled:modeled.modeled_workspace_enabled});
      const span = Number(document.getElementById("orbit-span").value);
      const response = Number(document.getElementById("orbit-response").value);
      const orbitDistance = Number(document.getElementById("orbit-distance").value);
      const payload = {
        rgbd_client_id:window.robotTeleopPageId,
        workspace_stream_crop_enabled:!document.body.classList.contains('workspace-editing'),
        // Mouse navigation and click-to-focus are always available. Head
        // tracking layers onto this orbit rather than selecting another mode.
        inspect_enabled: true,
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
        depth_only_enabled: document.getElementById("depth-only-enabled").checked,
        // Retire the old live-view default, including previously saved true values.
        persistent_temporal_reference_enabled: false,
        full_rgb_frame_updates_enabled: true,
        camera_locations_enabled: document.getElementById("camera-locations-enabled").checked,
        head_tracking_enabled: document.getElementById("head-tracking-enabled").checked,
        workspace_surface_enabled: document.getElementById("workspace-surface-enabled").checked,
        workspace_boundary_enabled: document.getElementById("workspace-boundary-enabled").checked,
        workspace_surface_offset: Number(document.getElementById("workspace-surface-offset").value),
        workspace_surface_size: Number(document.getElementById("workspace-surface-size").value),
        workspace_surface_opacity: Number(document.getElementById("workspace-surface-opacity").value),
        ...crop,
        ...modeled,
        display_mode: displayModeSelect.value,
        stream_preset: streamPresetSelect.value,
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
      document.getElementById("workspace-surface-offset-value").textContent = `${Math.round(Number(document.getElementById("workspace-surface-offset").value) * 1000)} mm`;
      document.getElementById("workspace-surface-size-value").textContent = `${Number(document.getElementById("workspace-surface-size").value).toFixed(2)} m`;
      document.getElementById("workspace-surface-opacity-value").textContent = `${Math.round(Number(document.getElementById("workspace-surface-opacity").value) * 100)}%`;
    }

    function updateHeadTrackingOptions() {
      const enabled = document.getElementById("head-tracking-enabled").checked;
      const options = document.getElementById("head-tracking-options");
      options.hidden = !enabled;
      for (const control of options.querySelectorAll("input")) control.disabled = !enabled;
    }

    function sendViewSettings(extra = {}) {
      if (!updateCropControls()) return false;
      updateViewLabels();
      const payload = viewSettingsPayload(extra);
      workspaceStreamControl.update(payload);
      // Calibration actions and uploaded files are transient, never startup defaults.
      localStorage.setItem("robotTeleopViewSettings", JSON.stringify(viewSettingsPayload()));
      window.setGodotHybridViewSettings && window.setGodotHybridViewSettings(payload);
      const remotePayload = {...payload};
      // The dedicated socket owns page crop state, avoiding reordering with
      // the optional tracking/WebRTC transport.
      delete remotePayload.rgbd_client_id;
      return window.sendGodotViewSettings && window.sendGodotViewSettings(remotePayload);
    }

    function localSavedViewSettings() {
      try {
        return JSON.parse(localStorage.getItem("robotTeleopViewSettings") || localStorage.getItem("digitalWindowViewSettings") || "null");
      } catch (_) {
        return null;
      }
    }

    function restoreViewSettings(savedInput = null) {
      const localSaved = localSavedViewSettings();
      const saved = savedInput && Object.keys(savedInput).length ? { ...localSaved, ...savedInput } : localSaved;
      if (!saved || ![6, 7, 8, 9, 10, 11, 12, 13, 14, 15, viewSettingsVersion].includes(saved.settings_version)) {
        updateHeadTrackingOptions();
        return updateViewLabels();
      }
      document.getElementById("orbit-span").value = String(Math.max(30, Math.min(360, (saved.max_yaw || 80) * 2)));
      document.getElementById("orbit-response").value = String(saved.yaw_gain || 2.5);
      document.getElementById("orbit-distance").value = String(saved.orbit_distance || 0.35);
      document.getElementById("dolly-enabled").checked = saved.dolly_enabled !== false;
      document.getElementById("dolly-gain").value = String(saved.dolly_gain ?? 0.008);
      document.getElementById("inspect-fov").value = String(Math.round(saved.fov || 55));
      document.getElementById("white-background-enabled").checked = saved.white_background_enabled === true;
      document.getElementById("depth-only-enabled").checked = saved.depth_only_enabled === true;
      document.getElementById("head-height").value = String(saved.focus_vertical_offset ?? 0.35);
      document.getElementById("orbit-elevation").value = String(saved.orbit_pitch_offset ?? 0);
      document.getElementById("head-tracking-enabled").checked = saved.head_tracking_enabled !== false;
      document.getElementById("camera-locations-enabled").checked = saved.camera_locations_enabled === true;
      document.getElementById("workspace-surface-enabled").checked = saved.workspace_surface_enabled === true;
      document.getElementById("workspace-boundary-enabled").checked = saved.workspace_boundary_enabled === true;
      document.getElementById("workspace-surface-offset").value = String(saved.workspace_surface_offset ?? 0);
      document.getElementById("workspace-surface-size").value = String(saved.workspace_surface_size ?? 1.2);
      document.getElementById("workspace-surface-opacity").value = String(saved.workspace_surface_opacity ?? 0.18);
      restoreCropSettings(saved);
      if ('modeled_workspace_enabled' in saved || 'modeled_workspace_center' in saved) restoreModeledSettings(saved);
      displayModeSelect.value = displayModes.includes(saved.display_mode) ? saved.display_mode : defaultDisplayMode;
      streamPresetSelect.value = saved.stream_preset === "latency" ? "latency" : "quality";
      startupNavigation = saved.default_navigation || null;
      startupNavigationPending = startupNavigation !== null;
      robotModule.restoreViewSettings?.(saved);
      updateHeadTrackingOptions();
      updateViewLabels();
    }

    async function fetchSiteSettings() {
      const response = await fetch("/api/v1/site-settings", { cache: "no-store" });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      return await response.json();
    }

    async function loadSiteSettings({ apply = false } = {}) {
      try {
        const saved = await fetchSiteSettings();
        if (apply) {
          restoreViewSettings(saved);
          sendViewSettings();
          if (startupNavigationPending && window.setGodotHybridNavigationState) {
            window.setGodotHybridNavigationState(startupNavigation);
            startupNavigationPending = false;
          }
          applyDisplayMode();
          if (userStarted) await startSelectedViewport();
        }
        siteSettingsStatus.textContent = Object.keys(saved).length
          ? "Saved installation defaults loaded."
          : "No installation defaults saved yet; using this browser's settings.";
        return saved;
      } catch (error) {
        siteSettingsStatus.textContent = `Could not load defaults: ${error && error.message ? error.message : error}`;
        return {};
      }
    }

    async function saveSiteSettings() {
      siteSettingsStatus.textContent = "Saving...";
      const payload = {
        ...viewSettingsPayload(),
        default_navigation: window.getGodotHybridNavigationState?.() || startupNavigation || undefined,
      };
      try {
        const response = await fetch("/api/v1/site-settings", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload),
        });
        if (!response.ok) throw new Error((await response.text()).trim() || `HTTP ${response.status}`);
        siteSettingsStatus.textContent = "Saved. New browsers will start with this setup.";
      } catch (error) {
        siteSettingsStatus.textContent = `Save failed: ${error && error.message ? error.message : error}`;
      }
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
      return `ws://${pageHost()}:8766/head-tracking`;
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
      rgbCameraCanvas.hidden = true;
      viewportImage.hidden = true;
      viewportVideo.hidden = false;
    }

    function showImage() {
      hybridCanvas.hidden = true;
      rgbCameraCanvas.hidden = true;
      viewportVideo.hidden = true;
      viewportImage.hidden = false;
    }

    function applyDisplayMode() {
      if (activeViewportMode !== "hybrid") return;
      const showRgb = displayModeSelect.value === "rgb_camera";
      hybridCanvas.hidden = showRgb;
      rgbCameraCanvas.hidden = !showRgb;
      viewportVideo.hidden = true;
      viewportImage.hidden = true;
      rgbCameraControl.hidden = !showRgb || rgbCameraSelect.options.length < 2;
      if (showRgb) refreshRgbCameras();
    }

    async function refreshRgbCameras() {
      if (rgbCameraListLoading || Date.now() - rgbCameraListUpdated < 5000) return;
      rgbCameraListLoading = true;
      const controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), 5000);
      try {
        const response = await fetch('/api/v1/cameras', {cache:'no-store',signal:controller.signal});
        if (!response.ok) return;
        const { cameras } = await response.json();
        if (!Array.isArray(cameras)) return;
        const signature = JSON.stringify(cameras.map(({id,name,model}) => ({id,name,model})));
        if (signature !== rgbCameraListSignature) {
          rgbCameraListSignature = signature;
          rgbCameraSelect.replaceChildren(...cameras.map(camera => new Option(
            `${camera.name || camera.model || 'Camera'} · ${camera.id.slice(-4)}`, camera.id)));
          if (!cameras.some(camera => camera.id === selectedRgbCamera)) selectedRgbCamera = cameras[0]?.id || '';
          rgbCameraSelect.value = selectedRgbCamera;
          window.setGodotHybridRgbCamera?.(selectedRgbCamera);
        }
        rgbCameraListUpdated = Date.now();
        const levelsDisplay = document.getElementById('camera-image-levels');
        if (levelsDisplay) levelsDisplay.textContent = cameras.map(camera => {
          const name = `${camera.name || camera.model || 'Camera'} · ${camera.id.slice(-4)}`;
          const levels = camera.image_levels;
          if (!levels || !Number.isFinite(levels.mean_luma)) return `${name}: image levels unavailable`;
          const age = Date.now() - levels.sample_unix_ms;
          const wb = levels.white_balance_state === 'warming up' ? 'warming up' :
            levels.auto_white_balance === 1 ? 'auto' : levels.auto_white_balance === 0 ? 'manual' : 'unavailable';
          return `${name}${age > 10000 ? ' (stale)' : ''}\n` +
            `Brightness ${Math.round(levels.mean_luma / 255 * 100)}% | bright areas ${Math.round(levels.p95_luma / 255 * 100)}% | dark pixels ${Math.round(levels.dark_percent)}% | white balance ${wb}`;
        }).join('\n\n') || 'No cameras available.';
        rgbCameraControl.hidden = displayModeSelect.value !== 'rgb_camera' || cameras.length < 2;
      } catch (_) { /* Keep the last usable camera list during reconnection. */ }
      finally {clearTimeout(timeout);rgbCameraListLoading = false;}
    }

    function showHybrid() {
      viewportVideo.hidden = true;
      viewportImage.hidden = true;
      applyDisplayMode();
    }

    function stopViewport() {
      viewportGeneration += 1;
      capturePresetControl?.cancel();
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
      rgbCameraCanvas.hidden = true;
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
            client:window.robotTeleopPageId,
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
      const navigation = window.getGodotHybridNavigationState?.({ includeView: true });
      if (navigation?.view) retainedNavigation = navigation;
      stopViewport();
      const generation = viewportGeneration;
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
      capturePresetControl ||= new CapturePresetControl(remoteUrl());
      if (!await capturePresetControl.request(streamPresetSelect.value) || generation !== viewportGeneration) return;
      // Both profiles preserve full native samples. The capture owner selects
      // each device's largest jointly supported depth/RGB pair at this cadence.
      const hybridPreset = nativeStreamPreset(streamPresetSelect.value);
      workspaceStreamControl.update(viewSettingsPayload());
      window.setGodotHybridViewSettings && window.setGodotHybridViewSettings(viewSettingsPayload({
        splat_fill: hybridPreset.splatFill,
        mesh_depth_delta: 0.05,
        mesh_max_edge: 0.08,
      }));
      // This async call creates all channels synchronously before its first
      // await, allowing the renderer to attach while ICE is negotiating.
      // The diagnostics-only query flag keeps control on WebRTC but sends
      // RGB-D over the public latest-frame endpoint. This lets a local browser
      // exercise the same Cloudflare path used when direct peer connectivity
      // is unavailable on a remote network.
      const peerPromise = startHybridControlPeer(
        forceRelayedRgbd ? null : hybridPreset
      );
      await Promise.all([
        window.startGodotHybridRenderer({
          canvas: hybridCanvas,
          rgbDisplayCanvas: rgbCameraCanvas,
          camera: selectedRgbCamera,
          ...hybridPreset,
          dataChannel: forceRelayedRgbd ? null : rgbdDataChannel,
          // Prefer direct unordered WebRTC when peer connectivity is possible.
          // A continuous latest-only WebSocket is the free public fallback;
          // HTTPS polling remains available as an explicit diagnostic mode.
          transport: forceHttpsRgbd ? "https" : "websocket",
          // Persistent recompression is experimental and too slow for live
          // capture. Old saved settings must not silently re-enable it.
          persistentReference: false,
          // Native JPEG/depth are already compressed; avoid a second codec.
          temporalDepth: false,
          temporalColor: false,
          onFocus: () => sendViewSettings({ inspect_enabled: true }),
        }),
        peerPromise,
      ]);
      if (generation !== viewportGeneration) return;
      if (retainedNavigation && window.setGodotHybridNavigationState) {
        window.setGodotHybridNavigationState(retainedNavigation);
      } else if (startupNavigationPending && startupNavigation && window.setGodotHybridNavigationState) {
        window.setGodotHybridNavigationState(startupNavigation);
        startupNavigationPending = false;
      }
      applyDisplayMode();
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
      const enabled = document.getElementById("head-tracking-enabled").checked;
      if (!enabled) {
        startupRecenterGeneration += 1;
        window.stopGodotWebcamTracker && window.stopGodotWebcamTracker();
        trackingState = "off · mouse navigation enabled";
        updateStatus();
        return true;
      }
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
        startMessage.textContent = err?.message || "Could not start the viewer. Please retry.";
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
      const geometry = hybrid.workspaceCrop;
      const modeled=modeledWorkspace(modeledViewSettings()),crop=cropViewSettings().workspace_crop_enabled;
      if ((modeled||crop) && geometry?.sourceCells > 0) {
        const percent = Math.min(100,100 * geometry.submittedCells / geometry.sourceCells);
        const network=hybrid.streamCrop;
        document.getElementById(modeled?'modeled-workspace-status':'workspace-crop-status').textContent = network?.applied&&network.input_bytes>0
          ? `Server stream: ${(100*network.output_bytes/network.input_bytes).toFixed(1)}% of original RGB-D bytes. Geometry submitted: ${percent.toFixed(1)}% of source cells.`
          : `Geometry submitted: ${percent.toFixed(1)}% of source cells. Server crop not active; local cropping still applies.`;
      }
      const hybridStalled = activeViewportMode === "hybrid" && hybrid.stalled === true;
      const headTrackingEnabled = document.getElementById("head-tracking-enabled").checked;
      statusDot.className = "dot " + (hybridStalled ? "bad" : connected ? "ok" : "bad");
      statusTitle.textContent = hybridStalled ? "Stream stalled" : connected ? "Connected" : "Connecting";
      const size = activeViewportMode === "hybrid"
        ? (hybrid.width > 0 ? `${hybrid.width}x${hybrid.height}` : "--")
        : (viewportVideo.videoWidth > 0 ? `${viewportVideo.videoWidth}x${viewportVideo.videoHeight}` : "--");
      const hasStreamMeasurement = activeViewportMode === "hybrid"
        ? hybrid.received > 0 || hybrid.connected === true
        : stats.framesDecoded > 0;
      let fps = hasStreamMeasurement ? `${Math.round(Math.max(0, stats.fps || 0))} fps` : "-- fps";
      const cameraRates = Object.values(hybrid.cameraFps || {}).map(Number).filter(Number.isFinite);
      if (activeViewportMode === "hybrid" && hybrid.displaySource !== "original_rgb" && cameraRates.length) {
        const minimum = Math.round(Math.min(...cameraRates)), maximum = Math.round(Math.max(...cameraRates));
        fps = `${minimum === maximum ? minimum : `${minimum}-${maximum}`} updates/s/camera`;
      }
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
          ...(hybrid.displaySource === "original_rgb" ? [] : [
            `Packets ${Number(hybrid.received || 0)} received | ${Number(hybrid.decoded || 0)} decoded batches | ${Number(hybrid.coalesced || 0)} coalesced`,
            `Arrival ${Number(hybrid.receiveFps || 0).toFixed(1)}/s | decoded ${Number(hybrid.sourceFps || 0).toFixed(1)}/s | queue ${Number(hybrid.queueDepth || 0)}${hybrid.decodeBusy ? " + 1 in progress" : ""}`,
            `Codec ${Number(hybrid.codecMs || 0).toFixed(1)}ms | upload submission ${Number(hybrid.uploadMs || 0).toFixed(1)}ms | queue wait ${Number(hybrid.queueMs || 0).toFixed(1)}ms`,
            `Worker ${Number(hybrid.workerMs || 0).toFixed(1)}ms | return scheduling ${Number(hybrid.dispatchMs || 0).toFixed(1)}ms`,
          ]),
          `Decode+upload ${hybrid.decodeMs ? `${hybrid.decodeMs.toFixed(1)}ms` : "--"} | render ${hybrid.renderFps ? `${Math.round(hybrid.renderFps)} fps` : "--"}`,
          ...(hybrid.decodeErrors ? [`Decode errors ${hybrid.decodeErrors} | ${hybrid.status}`] : []),
          `Camera FPS  ${Object.entries(hybrid.cameraFps || {}).map(([id, rate]) => `${id.slice(-4)}:${Math.round(rate)}`).join("  ") || "--"}`,
          `Camera frames ${Object.entries(hybrid.cameraFrames || {}).map(([id, count]) => `${id.slice(-4)}:${count}`).join("  ") || "--"}`,
          ...(hybrid.workspaceCrop?.enabled && hybrid.workspaceCrop.sourceCells ? [
            `Workspace crop: ${Math.round(100 * (1 - hybrid.workspaceCrop.submittedCells / hybrid.workspaceCrop.sourceCells))}% of source geometry skipped; native detail inside`,
          ] : []),
        ] : []),
      ].join("\n");
      statusLines.textContent = [
        `Stream ${hybrid.displaySource === "original_rgb" ? `${currentPreset().label} RGB` : streamMode} | ${size} | ${fps} | ${bitrate}`,
        ...(activeViewportMode === 'hybrid' && hybrid.displaySource !== 'original_rgb' && hasStreamMeasurement
          ? [`Viewer drawing ${Math.round(hybrid.renderFps || 0)} fps`] : []),
        !connected ? streamState : "",
        activeViewportMode === "hybrid" ? hybrid.renderWarning : "",
        headTrackingEnabled
          ? `Tracking ${latest.status || trackingState} | ${remote.connected ? "bridge on" : "bridge ..."}`
          : "Tracking off | mouse orbit/pan/zoom enabled",
        robotModule.statusSummary?.(),
      ].filter(Boolean).join("\n");
      robotModule.renderStatus?.();
      if (activeViewportMode === "hybrid") updateRgbdLatencyGraph(hybrid);
    }

    streamPresetSelect.addEventListener("change", () => {
      sendViewSettings();
      if (userStarted) startSelectedViewport().catch(error => {
        streamState = error?.message || "Camera mode change failed";
        updateStatus();
      });
      streamPresetSelect.blur();
      updateStatus();
    });
    displayModeSelect.addEventListener("change", () => {
      applyDisplayMode();
      sendViewSettings();
      displayModeSelect.blur();
    });
    rgbCameraSelect.addEventListener('change', () => {
      selectedRgbCamera = rgbCameraSelect.value;
      window.setGodotHybridRgbCamera?.(selectedRgbCamera);
      rgbCameraSelect.blur();
    });
    document.getElementById("restart").addEventListener("click", startController);
    document.getElementById("recenter").addEventListener("click", () => {
      retainedNavigation = null;
      window.recenterGodotWebcamTracker && window.recenterGodotWebcamTracker();
      window.recenterGodotHybridRenderer && window.recenterGodotHybridRenderer();
    });
    for (const id of viewControlIds) {
      const control = document.getElementById(id);
      control.addEventListener("input", () => sendViewSettings());
      control.addEventListener("change", () => control.blur());
    }
    for (const id of [
      "dolly-enabled", "white-background-enabled", "depth-only-enabled",
      "workspace-surface-enabled", "workspace-boundary-enabled",
      "camera-locations-enabled",
      "workspace-crop-show-box",
      "scene-lighting-enabled", "scene-shadows-enabled",
    ]) {
      const control = document.getElementById(id);
      control.addEventListener("change", () => {
        sendViewSettings();
        control.blur();
      });
    }
    document.getElementById('workspace-crop-enabled').addEventListener('change', event => {
      if (event.currentTarget.checked && !workspaceCropConfigured && !centerCropOnOrbit()) {
        event.currentTarget.checked = false;
        updateCropControls();
        document.getElementById('workspace-crop-status').textContent = 'Open a connected 3D view to center the box.';
        return;
      }
      sendViewSettings();
    });
    document.getElementById('modeled-workspace-enabled').addEventListener('change', event => {
      if (event.currentTarget.checked && !modeledWorkspaceConfigured && !alignModeledTable()) {
        event.currentTarget.checked = false;
        return;
      }
      updateModeledControls();sendViewSettings();
    });
    document.getElementById('modeled-workspace-align').addEventListener('click', () => {
      if (alignModeledTable('robot')) sendViewSettings();
    });
    document.getElementById('modeled-workspace-orbit').addEventListener('click',()=>{if(alignModeledTable('orbit'))sendViewSettings();});
    for (const name of modeledFields) {
      const input = document.getElementById(`modeled-workspace-${name}`);
      input.addEventListener(name === 'light-angle' ? 'input' : 'change', () => {
        if (updateModeledControls()) {modeledWorkspaceConfigured=true;sendViewSettings();}
      });
    }
    document.getElementById('workspace-crop-center').addEventListener('click', () => {
      if (centerCropOnOrbit()) sendViewSettings();
    });
    for (const bound of ['min', 'max']) for (const axis of ['x', 'y', 'z']) {
      document.getElementById(`workspace-crop-${bound}-${axis}`).addEventListener('change', () => {
        if (updateCropControls()) {workspaceCropConfigured = true; sendViewSettings();}
      });
    }
    document.getElementById("head-tracking-enabled").addEventListener("change", async (event) => {
      const control = event.currentTarget;
      updateHeadTrackingOptions();
      sendViewSettings();
      if (userStarted) {
        try {
          await startTracking();
        } catch (error) {
          trackingState = `needs start: ${error && error.message ? error.message : error}`;
          startMessage.textContent = "Camera permission needs a tap.";
          startOverlay.hidden = false;
        }
      }
      control.blur();
      updateStatus();
    });
    let workspaceEditTimer=0;
    const flushWorkspaceEdit=()=>{clearTimeout(workspaceEditTimer);workspaceEditTimer=0;sendViewSettings();};
    window.addEventListener('workspace-edit-change',event=>{
      const {settings,phase}=event.detail;
      if('workspace_crop_min' in settings)restoreCropSettings({...cropViewSettings(),...settings});
      if('modeled_workspace_center' in settings)restoreModeledSettings({...modeledViewSettings(),...settings});
      if(phase==='preview') {if(!workspaceEditTimer)workspaceEditTimer=setTimeout(flushWorkspaceEdit,100);}
      else flushWorkspaceEdit();
    });
    window.addEventListener('workspace-edit-finish',event=>{
      document.body.classList.remove('workspace-editing');
      sendViewSettings();
      document.getElementById('open-advanced').focus({preventScroll:true});
    });
    for(const target of ['crop','table'])document.getElementById(target==='table'?'modeled-workspace-edit':'workspace-crop-edit').addEventListener('click',()=>{
      if(displayModeSelect.value==='rgb_camera') {displayModeSelect.value='independent_mesh';applyDisplayMode();sendViewSettings();}
      if(window.editGodotWorkspace?.(target)) {advancedSetup.hidden=true;document.body.classList.add('workspace-editing');sendViewSettings();}
      else document.getElementById(target==='table'?'modeled-workspace-status':'workspace-crop-status').textContent='Open a connected 3D view to edit this workspace.';
    });
    for(const axis of ['x','y','z'])document.getElementById(`workspace-crop-rotation-${axis}`).addEventListener('change',()=>{if(updateCropControls())sendViewSettings();});
    function handleDepthOnlyShortcut(event) {
      if (event.key !== "Backspace" || event.defaultPrevented || event.isComposing
        || event.ctrlKey || event.altKey || event.metaKey || event.shiftKey) return;
      const target = event.target;
      if (target?.isContentEditable || target?.closest?.("input, textarea, select, [role='textbox']")) return;
      if (!hybridMode || displayModeSelect.value === "rgb_camera") return;
      event.preventDefault();
      if (event.repeat) return;
      const control = document.getElementById("depth-only-enabled");
      control.checked = !control.checked;
      sendViewSettings();
    }
    document.addEventListener("keydown", handleDepthOnlyShortcut);
    document.getElementById("save-site-defaults").addEventListener("click", saveSiteSettings);
    document.getElementById("reload-site-defaults").addEventListener("click", async () => {
      await loadSiteSettings({ apply: true });
      if (userStarted) await startSelectedViewport();
    });
    for (const element of [viewportVideo, viewportImage]) {
      element.addEventListener("dblclick", focusInspectAtStreamClick);
      element.addEventListener("auxclick", (event) => {
        if (event.button !== 1) return;
        event.preventDefault();
        focusInspectAtStreamClick(event);
      });
      element.addEventListener("pointerdown", (event) => {
        if (event.button === 1) event.preventDefault();
      });
    }
    startButton.addEventListener("click", startController);

    setInterval(() => {
      if (displayModeSelect.value === 'rgb_camera' || !advancedSetup.hidden) refreshRgbCameras();
      collectStats().catch(() => {});
      updateStatus();
    }, 500);

    const installationSettings = await loadSiteSettings();
    restoreViewSettings(installationSettings);
    robotModule.bind?.({ sendViewSettings, updateStatus });
    robotModule.start?.({ sendViewSettings, updateStatus });
    const trialController = new TrialController({recording: episodeRecording,
      pauseRobot: typeof robotModule.pauseTrial === 'function' ? () => robotModule.pauseTrial() : null,
      resumeRobot: typeof robotModule.resumeTrial === 'function' ? () => robotModule.resumeTrial() : null,
      finishRobotPause: () => robotModule.finishTrialPause?.(),
      applyCondition: async (condition, renderer) => {
        const mode = condition === 'fixed_rgb' ? 'rgb_camera' : renderer;
        displayModeSelect.value = displayModes.includes(mode) ? mode : defaultDisplayMode;
        document.getElementById('head-tracking-enabled').checked = condition === 'head_3d';
        updateHeadTrackingOptions();
        applyDisplayMode();
        sendViewSettings();
        if (userStarted) await startTracking();
      },
      resetView: () => {retainedNavigation = null; window.recenterGodotHybridRenderer?.();},
      resetTracking: () => window.recenterGodotWebcamTracker?.(),
      viewReady: () => {
        if (activeViewportMode !== 'hybrid') return viewportVideo.currentTime > 0;
        const view = window.getGodotHybridRendererStatus?.();
        return view?.connected === true && view.stalled !== true && Number(view.decoded) > 0;
      },
    });
    await trialController.initialize();
    startController();
