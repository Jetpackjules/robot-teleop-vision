// Stable model materials; no camera colour, geometry edits or pose corrections.
export function triangleNormals(positions, indices = null, mode = 4) {
  const normals = new Float32Array(positions.length);
  if (mode !== 4) return normals;
  const count = indices ? indices.length : positions.length / 3;
  for (let i = 0; i + 2 < count; i += 3) {
    const a = (indices ? indices[i] : i) * 3;
    const b = (indices ? indices[i + 1] : i + 1) * 3;
    const c = (indices ? indices[i + 2] : i + 2) * 3;
    const ux = positions[b] - positions[a], uy = positions[b + 1] - positions[a + 1], uz = positions[b + 2] - positions[a + 2];
    const vx = positions[c] - positions[a], vy = positions[c + 1] - positions[a + 1], vz = positions[c + 2] - positions[a + 2];
    const x = uy * vz - uz * vy, y = uz * vx - ux * vz, z = ux * vy - uy * vx;
    for (const index of [a, b, c]) {
      normals[index] += x; normals[index + 1] += y; normals[index + 2] += z;
    }
  }
  for (let i = 0; i < normals.length; i += 3) {
    const length = Math.hypot(normals[i], normals[i + 1], normals[i + 2]) || 1;
    normals[i] /= length; normals[i + 1] /= length; normals[i + 2] /= length;
  }
  return normals;
}

export function robotSurfaceMaterial(source, palette = []) {
  // The module supplies its own palette. Unknown models retain their material.
  const match = palette.find(entry => entry.sourceColor?.length >= 3
    && Math.hypot(...entry.sourceColor.slice(0, 3).map((v, i) => v - source.color[i])) < .035);
  return match ? {...source, ...match, color: [...match.color]} : source;
}
