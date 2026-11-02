// Bounded slow-call attribution; no polling, pixel readbacks or CPU sampling.
const events = [];
const profiled = new WeakSet();
function record(label, ms, stack = "") {
  if (!(ms >= 80)) return;
  events.push({label: String(label).slice(0, 160), ms: Math.round(ms * 10) / 10,
    at: performance.now(), stack: String(stack).slice(0, 400)});
  if (events.length > 32) events.shift();
}
export function timeBrowserWork(label, call) {
  const start = performance.now();
  try { return call(); }
  finally { record(label, performance.now() - start); }
}
export function profileBrowserGL(gl) {
  if (profiled.has(gl)) return;
  profiled.add(gl);
  for (const name of ["getShaderParameter", "getProgramParameter", "texSubImage2D",
    "texImage2D", "bufferData", "readPixels", "drawArrays", "drawElements",
    "drawArraysInstanced", "drawElementsInstanced"]) {
    const original = gl[name];
    if (typeof original !== "function") continue;
    try {
      gl[name] = function (...args) {
        const start = performance.now();
        try { return original.apply(this, args); }
        finally {
          const ms = performance.now() - start;
          if (ms >= 80) record(`WebGL.${name}`, ms, new Error().stack?.split("\n").slice(2, 5).join("\n"));
        }
      };
    } catch (_) { /* Restricted contexts retain their original methods. */ }
  }
}
export function browserWorkTrace() {
  const now = performance.now();
  return events.filter(event => now - event.at < 60000).slice(-6)
    .map(({at, ...event}) => ({...event, ageMs: Math.round(now - at)}));
}
if (typeof window !== "undefined") {
  window.getRobotTeleopBrowserWorkTrace = browserWorkTrace;
  if (typeof PerformanceObserver !== "undefined"
      && PerformanceObserver.supportedEntryTypes?.includes("long-animation-frame")) {
    const observer = new PerformanceObserver(list => {
      for (const frame of list.getEntries()) {
        for (const script of frame.scripts || []) {
          let source = "unknown";
          try { source = new URL(script.sourceURL, location.href).pathname.split("/").pop(); } catch (_) {}
          record(`${source}:${script.sourceFunctionName || script.invokerType}`, script.duration,
            `position=${script.sourceCharPosition};layoutMs=${script.forcedStyleAndLayoutDuration || 0};pauseMs=${script.pauseDuration || 0}`);
        }
      }
    });
    observer.observe({type: "long-animation-frame"});
  }
}
