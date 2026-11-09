// Surface rejection limits for the production BlendPCR renderer.
export function blendPcrSurfaceSettings(settings) {
  const bounded=(value,min,max,fallback)=>Number.isFinite(Number(value))?Math.max(min,Math.min(max,Number(value))):fallback;
  const active=settings.blendpcr_enabled===true;
  const relaxed=active&&settings.blendpcr_relaxed===true;
  return {
    relaxed,
    tolerance:active?bounded(settings.blendpcr_depth_tolerance,.5,6,1):1,
    support:active?bounded(settings.blendpcr_support,0,4,relaxed?1:2):2,
    edge:active?bounded(settings.blendpcr_max_edge,.001,1,Number(settings.mesh_max_edge||.08)):Number(settings.mesh_max_edge||.08),
  };
}
