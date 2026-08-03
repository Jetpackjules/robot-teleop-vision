const toggle = document.getElementById("so101-wrist-camera-enabled");
const image = document.getElementById("so101-wrist-camera-view");
const panel = document.getElementById("so101-wrist-camera-panel");
const status = document.getElementById("so101-wrist-camera-status");
const storageKey = "robotTeleop.so101.wristCameraEnabled";

if (toggle) {
  toggle.checked = localStorage.getItem(storageKey) !== "false";
}

let retryTimer = 0;
let generation = 0;

function streamUrl() {
  return `${window.location.origin}/robot-view/wrist_rgb?w=1280&h=720&fps=30&v=${Date.now()}`;
}

function stopClawCamera() {
  generation += 1;
  window.clearTimeout(retryTimer);
  retryTimer = 0;
  if (panel) panel.hidden = true;
  if (!image) return;
  image.hidden = true;
  image.onload = null;
  image.onerror = null;
  // Replacing src actively aborts a multipart response; removeAttribute alone
  // can leave the old HTTP request owning a server stream until TCP timeout.
  image.src = "data:image/gif;base64,R0lGODlhAQABAAD/ACwAAAAAAQABAAACADs=";
  image.removeAttribute("src");
}

function startClawCamera() {
  if (!toggle?.checked || !image) return;
  const currentGeneration = ++generation;
  window.clearTimeout(retryTimer);
  if (panel) panel.hidden = false;
  image.hidden = true;
  if (status) {
    status.hidden = false;
    status.textContent = "Connecting to claw camera…";
  }
  image.src = streamUrl();
  image.onerror = () => {
    if (!toggle.checked || currentGeneration !== generation) return;
    image.hidden = true;
    if (status) {
      status.hidden = false;
      status.textContent = "Reconnecting live claw camera…";
    }
    retryTimer = window.setTimeout(startClawCamera, 750);
  };
  image.onload = () => {
    if (currentGeneration === generation) {
      image.hidden = false;
      if (status) status.hidden = true;
    }
  };
}

toggle?.addEventListener("change", () => {
  localStorage.setItem(storageKey, String(toggle.checked));
  if (toggle.checked) startClawCamera();
  else stopClawCamera();
  toggle.blur();
});

if (toggle?.checked) startClawCamera();

window.addEventListener("beforeunload", stopClawCamera);
window.addEventListener("pagehide", stopClawCamera);
window.so101WristCameraView = { start: startClawCamera, stop: stopClawCamera };
