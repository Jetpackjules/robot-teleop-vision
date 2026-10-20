import { RIG_SAMPLING_SHADER } from "./hybrid_camera_quality.js?v=20261020_depth_quality";
export { GpuCameraRig as MultiCameraFusion } from "./hybrid_camera_quality.js?v=20261020_depth_quality";

// Shared by the live renderer and the camera-free GPU regression test.
export function cameraIsFresh(camera, now = performance.now()) {
  return camera.width > 1 && camera.height > 1 && now - camera.lastUpdateMs < 1000;
}

export function orderedCameras(cameras) {
  return [...cameras].sort((a, b) => Number(a.metadata.priority || 0) - Number(b.metadata.priority || 0)
    || String(a.metadata.id).localeCompare(String(b.metadata.id)));
}

export function chooseColorReference(cameras, now = performance.now()) {
  const live = orderedCameras(cameras).filter(camera => cameraIsFresh(camera, now));
  return live.find(camera => String(camera.metadata.model || "").includes("435")) || live[0] || null;
}

export function pointPixelScale(height, fov, intrinsics, fill) {
  const focal = height / (2 * Math.tan(fov * Math.PI / 360));
  return focal * fill / Math.max(1, Math.min(intrinsics[0], intrinsics[1]));
}

// Local geometry ownership and color selection share the same camera score.
export const FUSION_FRAGMENT_SHADER = `
${RIG_SAMPLING_SHADER}
uniform highp int u_rig_enabled;
uniform highp int u_rig_color_enabled;
uniform highp int u_rig_index;
in vec3 v_world_position;
bool rig_overlap() {
  if(u_rig_enabled==0)return false;
  RigHit own;
  if(!rig_surface(u_rig_index,v_world_position,own))return false;
  for(int camera=0;camera<u_rig_count;camera++) {
    if(camera==u_rig_index)continue;
    RigHit peer;
    if(!rig_surface(camera,v_world_position,peer))continue;
    float error=-peer.local.z+rig_local(camera,v_world_position).z;
    if(rig_free_space(own,camera,v_world_position,peer))return true;
    if(abs(error)<=rig_tolerance(-peer.local.z)
      &&rig_better(peer.score,camera,own.score,u_rig_index))return true;
  }
  return false;
}
vec3 rig_color(vec3 fallback) {
  if(u_rig_color_enabled==0)return fallback;
  RigHit own;
  bool has_own=rig_surface(u_rig_index,v_world_position,own);
  float quality=has_own?own.score:-1.0;
  int owner=u_rig_index;
  vec3 color=fallback;
  for(int camera=0;camera<u_rig_count;camera++) {
    if(camera==u_rig_index)continue;
    RigHit peer;
    if(!rig_surface(camera,v_world_position,peer))continue;
    float error=-peer.local.z+rig_local(camera,v_world_position).z;
    if(abs(error)<=rig_tolerance(-peer.local.z)
      &&(quality<0.0||rig_better(peer.score,camera,quality,owner))) {
      color=peer.color;quality=peer.score;owner=camera;
    }
  }
  return color;
}
`;
