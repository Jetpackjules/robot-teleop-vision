const DETAILS = new Set(['0', '-2', '-1', '20000', '40000']);

// An explicit last-used browser choice wins; new tunnel origins use the host.
export function resolvePointDetail(saved = {}, browserValue = null, preferBrowser = true) {
  const hostValue = String(saved?.point_preview_detail);
  const choices = preferBrowser ? [browserValue, hostValue] : [hostValue, browserValue];
  return choices.find(value => DETAILS.has(value)) ?? '-1';
}
