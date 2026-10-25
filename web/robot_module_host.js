const MODULE_ID = /^[a-z][a-z0-9_-]{0,63}$/;
const MODULE_ENTRYPOINT = /^\/robot-modules\/[a-z][a-z0-9_-]{0,63}\/web\/[A-Za-z0-9_./-]+\.js$/;

function visionOnlyModule(manifest = {}, error = null) {
  const label = manifest.label || "Vision only";
  return {
    manifest: {
      id: "disabled",
      label: "Vision only",
      capabilities: {},
      ...manifest,
    },
    renderer: null,
    mount(points) {
      // Failed mounts can leave partly initialized robot controls behind.
      for (const root of Object.values(points)) root?.replaceChildren();
      const root = points.viewRoot || points.setupRoot || points.statusRoot;
      if (!root) return;
      const notice = document.createElement("p");
      notice.className = "helper-text";
      notice.setAttribute("role", "status");
      notice.textContent = error
        ? `Robot controls could not load. Reload the page to retry. ${String(error.message || error)}`
        : manifest.id && manifest.id !== "disabled"
          ? `${label} does not provide web controls.`
          : "Vision only mode is active. Select a robot module on the host and restart the controller to use its overlay and calibration controls.";
      root.append(notice);
    },
    extendViewSettings(payload) { return payload; },
    restoreViewSettings() {},
    statusSummary() { return error ? "Robot module unavailable" : `Robot ${label}`; },
    renderStatus() {},
    bind() {},
    start() {},
    stop() {},
  };
}

function validateManifest(value) {
  if (!value || typeof value !== "object" || !MODULE_ID.test(String(value.id || ""))) {
    throw new Error("operator server returned an invalid robot module manifest");
  }
  const web = String(value.entrypoints?.web || "");
  if (web && !MODULE_ENTRYPOINT.test(web)) {
    throw new Error(`robot module ${value.id} has an unsafe web entrypoint`);
  }
  return value;
}

function mountPoints() {
  return {
    settingsRoot: document.getElementById("robot-settings-root"),
    setupRoot: document.getElementById("robot-setup-root"),
    viewRoot: document.getElementById("robot-view-root"),
    statusRoot: document.getElementById("robot-status-root"),
    floatingRoot: document.getElementById("robot-floating-root"),
    quickControlsRoot: document.getElementById("robot-quick-controls-root"),
  };
}

async function loadActiveModule() {
  const response = await fetch("/api/v1/robot-module", { cache: "no-store" });
  if (!response.ok) throw new Error(`robot module discovery failed (${response.status})`);
  const manifest = validateManifest(await response.json());
  if (manifest.id === "disabled" || !manifest.entrypoints?.web) {
    const module = visionOnlyModule(manifest);
    module.mount(mountPoints());
    return module;
  }
  const implementation = await import(manifest.entrypoints.web);
  if (typeof implementation.createRobotModule !== "function") {
    throw new Error(`robot module ${manifest.id} does not export createRobotModule()`);
  }
  const module = await implementation.createRobotModule(manifest);
  if (!module || typeof module !== "object") {
    throw new Error(`robot module ${manifest.id} did not create a module object`);
  }
  module.manifest = manifest;
  await module.mount?.(mountPoints());
  return module;
}

export const robotModuleReady = loadActiveModule().catch((error) => {
  console.error("Robot module failed to load", error);
  const module = visionOnlyModule({
    id: "disabled",
    label: "Robot module unavailable",
  }, error);
  const points = mountPoints();
  module.mount(points);
  return module;
});

window.robotTeleopModuleReady = robotModuleReady;
robotModuleReady.then((module) => {
  window.robotTeleopActiveModule = module;
  window.dispatchEvent(new CustomEvent("robot-teleop-module-ready", { detail: module.manifest }));
});
