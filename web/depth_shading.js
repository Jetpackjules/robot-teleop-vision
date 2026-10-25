// Neutral material lit from measured geometry, independent of camera RGB.
export const DEPTH_SHADING_SHADER = `
vec3 depth_surface_gray(vec3 normal) {
  float length_squared = dot(normal, normal);
  float diffuse = length_squared > 1e-20
    ? abs(dot(normal * inversesqrt(length_squared), normalize(vec3(0.35, 0.8, 0.45))))
    : 0.6;
  return vec3(0.28 + 0.52 * diffuse);
}
`;
