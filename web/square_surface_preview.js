import { compileGpuProgram, prepareGpuProgram, FULLSCREEN_VERTEX_SHADER } from "./hybrid_camera_quality.js?v=20261109_background_startup";

const FRAGMENT = `#version 300 es
precision highp float;
precision highp int;
uniform sampler2D u_points;
uniform sampler2D u_depth;
uniform ivec2 u_size;
uniform int u_coverage_level;
uniform vec3 u_projection;
uniform vec2 u_inverse_focal;
uniform vec4 u_viewer_y;
uniform float u_table_plane;
uniform int u_guard_table;
uniform int u_small_gaps;
uniform int u_plane_completion;
out vec4 out_color;

// Camera depth in meters, computed from the exact projection used for points.
float view_depth(float depth) {
  return u_projection.y / (depth * 2. - 1. + u_projection.x);
}
float window_depth(float z) {
  return .5 * (1. - u_projection.x + u_projection.y / z);
}
bool point_at(ivec2 cell, out vec3 rgb, out float z) {
  if(any(lessThan(cell,ivec2(0))) || any(greaterThanEqual(cell,u_size)))return false;
  vec4 color=texelFetch(u_points,cell,0);
  if(color.a<=0.)return false;
  rgb=color.rgb;
  z=view_depth(texelFetch(u_depth,cell,0).r);
  return z>0.;
}
float chroma(vec3 rgb) {
  return max(rgb.r,max(rgb.g,rgb.b))-min(rgb.r,min(rgb.g,rgb.b));
}
vec3 hue(vec3 rgb) {
  return (rgb-min(rgb.r,min(rgb.g,rgb.b)))/max(.001,chroma(rgb));
}
bool is_pigment(vec3 rgb) {
  return chroma(rgb)>=max(.055,.30*max(rgb.r,max(rgb.g,rgb.b)));
}
float color_delta(vec3 a,vec3 b) {
  vec3 d=abs(a-b);
  return max(d.r,max(d.g,d.b));
}
float nearby_coverage(ivec2 cell) {
  vec2 uv=(vec2(cell)+.5)/vec2(u_size);
  return textureLod(u_points,uv,float(u_coverage_level)).a;
}
bool supported_pair(ivec2 cell,ivec2 offset,vec4 original,float z0,out vec4 sample_pair) {
  vec3 a,b;float za,zb;
  if(!point_at(cell-offset,a,za) || !point_at(cell+offset,b,zb))return false;
  float z=(za+zb)*.5;
  float span=2.*length(vec2(offset))*z/u_projection.z;
  if(span>.008 || abs(za-zb)>min(.018,max(.005,span*1.8)) || color_delta(a,b)>.141)return false;
  vec3 color=(a+b)*.5;
  if(original.a>0.){
    // Only points with a native depth discontinuity can have ambiguous color.
    // The two sides must supply the same stronger pigment at the same depth.
    if(abs(z-z0)>.006 || !is_pigment(a) || !is_pigment(b) || distance(hue(a),hue(b))>.43 ||
       chroma(color)<chroma(original.rgb)+.024)return false;
    if(chroma(original.rgb)>=.086 && distance(hue(original.rgb),hue(color))>.49)return false;
  }
  sample_pair=vec4(color,z);
  return true;
}
bool above_table(ivec2 cell,vec3 rgb,float z){
  if(u_guard_table==0 || chroma(rgb)>.08)return true;
  vec2 ndc=2.*(vec2(cell)+.5)/vec2(u_size)-1.;
  vec3 ray=vec3(ndc*u_inverse_focal,-1.);
  float height=u_viewer_y.w+z*dot(u_viewer_y.xyz,ray)-u_table_plane;
  return height>.013;
}

// Extrapolate an observed depth slope a few millimeters across an empty pixel.
// Three samples must agree along each side of the slit before the sides can meet.
bool seam_face(ivec2 cell, ivec2 axis, int radius, vec3 edge, float ze,
               out vec3 pigment, out float predicted, out float depth_slope) {
  int step=int(clamp(round(.0025*u_projection.z/ze),3.,16.));
  vec3 middle,farther;
  float zm,zf;
  if(!point_at(cell+axis*(radius+step),middle,zm) ||
     !point_at(cell+axis*(radius+2*step),farther,zf))return false;
  float slope=(1./zf-1./ze)/float(2*step);
  float estimated_mid=1./(1./ze+float(step)*slope);
  if(abs(estimated_mid-zm)>.0018 || max(abs(zm-ze),abs(zf-zm))>.008)return false;
  pigment=is_pigment(middle)?middle:farther;
  if(!is_pigment(pigment))return false;
  if(is_pigment(farther) && distance(hue(pigment),hue(farther))>.35)return false;
  if(is_pigment(edge) && distance(hue(pigment),hue(edge))>.35)return false;
  float inverse=1./ze-float(radius)*slope;
  if(inverse<=0.)return false;
  predicted=1./inverse;
  depth_slope=(zf-ze)/float(2*step);
  return abs(predicted-ze)<=.008;
}
bool close_supported_seam(ivec2 cell, out vec4 repaired){
  const ivec2 axes[2]=ivec2[2](ivec2(1,0),ivec2(0,1));
  const int radii[5]=int[5](7,11,16,21,26);
  float scale=clamp(float(u_size.y)/1280.,.45,1.5);
  for(int i=0;i<5;i++)for(int axis=0;axis<2;axis++){
    int r=max(1,int(round(float(radii[i])*scale)));
    ivec2 dir=axes[axis];
    vec3 left,right;
    float zl,zr;
    if(!point_at(cell-r*dir,left,zl) || !point_at(cell+r*dir,right,zr))continue;
    if(float(2*r)*max(zl,zr)/u_projection.z>.011 || abs(zl-zr)>.008)continue;
    vec3 a,b;
    float za,zb,slope_a,slope_b;
    if(!seam_face(cell,-dir,r,left,zl,a,za,slope_a) || !seam_face(cell,dir,r,right,zr,b,zb,slope_b))continue;
    // Parallel surfaces can have a real gap even when their colors match.
    if(abs(slope_a+slope_b)*u_projection.z/max(za,zb)<.12)continue;
    if(distance(hue(a),hue(b))>.32 || abs(za-zb)>.0035)continue;
    repaired=vec4(za<=zb?a:b,min(za,zb));
    return true;
  }
  return false;
}


struct SeamFace {
  vec3 color;
  float inverse_depth;
  float inverse_slope;
  float edge_depth;
  float depth_slope;
  int radius;
};

// Sample a face along a ray and extrapolate its inverse depth to the target.
bool fit_seam_face(ivec2 cell, ivec2 axis, int radius, out SeamFace face) {
  vec3 edge, middle, farther;
  float ze, zm, zf;
  if (!point_at(cell + radius * axis, edge, ze)) return false;
  int step = int(clamp(round(.0025 * u_projection.z / ze), 3., 16.));
  if (!point_at(cell + (radius + step) * axis, middle, zm) ||
      !point_at(cell + (radius + 2 * step) * axis, farther, zf)) return false;
  if (max(abs(zm - ze), abs(zf - zm)) > .008) return false;

  float inverse_slope = (1. / zf - 1. / ze) / float(2 * step);
  float estimated_mid = 1. / (1. / ze + float(step) * inverse_slope);
  float inverse_depth = 1. / ze - float(radius) * inverse_slope;
  if (abs(estimated_mid - zm) > .0018 || inverse_depth <= 0. ||
      abs(1. / inverse_depth - ze) > .010) return false;

  vec3 color = is_pigment(middle) ? middle : farther;
  if (!is_pigment(color)) return false;
  vec3 color_hue = hue(color);
  if ((is_pigment(farther) && distance(color_hue, hue(farther)) > .35) ||
      (is_pigment(edge) && distance(color_hue, hue(edge)) > .35)) return false;

  face = SeamFace(color, inverse_depth, inverse_slope, ze, (zf - ze) / float(2 * step), radius);
  return true;
}

// A near and a farther fit cover asymmetric breaks without pairing every radius.
int seam_faces(ivec2 cell, ivec2 axis, float scale, out SeamFace near_face, out SeamFace far_face) {
  const int radii[5] = int[5](7, 11, 16, 21, 26);
  int count = 0;
  for (int i = 0; i < 5; i++) {
    int radius = max(1, int(round(float(radii[i]) * scale)));
    SeamFace face;
    if (!fit_seam_face(cell, axis, radius, face)) continue;
    if (count > 0 && radius != near_face.radius) {
      far_face = face;
      return 2;
    }
    near_face = face;
    count = 1;
  }
  return count;
}

bool improves_edge_color(vec3 original, vec3 candidate) {
  return chroma(candidate) > chroma(original) + .018 &&
    (!is_pigment(original) || distance(hue(candidate), hue(original)) <= .50);
}

bool join_seam_faces(SeamFace left, SeamFace right, float axis_length, vec4 point,
                     float original_z, out vec4 repaired) {
  float span = float(left.radius + right.radius) * axis_length;
  if (span * max(left.edge_depth, right.edge_depth) > .012 * u_projection.z) return false;
  float za = 1. / left.inverse_depth, zb = 1. / right.inverse_depth;
  float z = max(za, zb);
  // At a convex edge the visible surface is the farther of the two planes.
  // Their crossing must lie between the measured samples.
  if ((left.depth_slope + right.depth_slope) * u_projection.z <= .12 * z * axis_length ||
      abs(za - zb) > .008) return false;
  float slope = left.inverse_slope + right.inverse_slope;
  if (slope >= 0.) return false;
  float intersection = (left.inverse_depth - right.inverse_depth) / slope;
  if (intersection < -float(left.radius) - 1. || intersection > float(right.radius) + 1.) return false;

  vec3 color = za >= zb ? left.color : right.color;
  float error = abs(original_z - z);
  float other_error = abs(original_z - min(za, zb));
  if (distance(hue(left.color), hue(right.color)) > .32) {
    // A measured pixel can use the visible face's color when the other face
    // has a different color and is farther from the measured depth.
    if (point.a <= 0. || error > .0025 || other_error < error + .0015) return false;
  }
  if (point.a > 0. && (min(error, other_error) > .0035 || error > .009 ||
      !improves_edge_color(point.rgb, color))) return false;
  repaired = vec4(color, z);
  return true;
}

bool repair_supported_corner(ivec2 cell, vec4 point, float z, float scale, out vec4 repaired) {
  const ivec2 axes[4] = ivec2[4](ivec2(1, 0), ivec2(0, 1), ivec2(1, 1), ivec2(1, -1));
  for (int axis = 0; axis < 4; axis++) {
    ivec2 direction = axes[axis];
    SeamFace near_left, far_left, near_right, far_right;
    int left = seam_faces(cell, -direction, scale, near_left, far_left);
    if (left == 0) continue;
    int right = seam_faces(cell, direction, scale, near_right, far_right);
    if (right == 0) continue;
    float span_scale = axis < 2 ? 1. : 1.414214;
    if (join_seam_faces(near_left, near_right, span_scale, point, z, repaired) ||
        (left == 2 && join_seam_faces(far_left, near_right, span_scale, point, z, repaired)) ||
        (right == 2 && join_seam_faces(near_left, far_right, span_scale, point, z, repaired)) ||
        (left == 2 && right == 2 && join_seam_faces(far_left, far_right, span_scale, point, z, repaired))) return true;
  }
  return false;
}


struct FaceColor {
  vec3 color;
  float depth;
  float cost;
};

// For a flagged measured pixel, look into the same face from adjacent
// directions. Require both depth predictions and both pigments to agree.
FaceColor face_color(ivec2 cell, ivec2 direction, float scale, vec3 original, float measured_z) {
  const int radii[5] = int[5](7, 11, 16, 21, 26);
  FaceColor best = FaceColor(vec3(0.), 0., 1.);
  float pixel_span = length(vec2(direction)) * measured_z / u_projection.z;
  for (int i = 0; i < 5; i++) {
    int radius = max(1, int(round(float(radii[i]) * scale)));
    float span = float(radius) * pixel_span;
    if (span > .006 || span >= best.cost) break;
    SeamFace face;
    if (!fit_seam_face(cell, direction, radius, face)) continue;
    float z = 1. / face.inverse_depth;
    float error = abs(measured_z - z);
    if (error <= .0022 && error + span < best.cost && improves_edge_color(original, face.color)) {
      best = FaceColor(face.color, z, error + span);
    }
  }
  return best;
}

bool repair_face_color(ivec2 cell, vec3 original, float measured_z, float scale, out vec3 color) {
  const ivec2 directions[8] = ivec2[8](ivec2(1, 0), ivec2(1, 1), ivec2(0, 1), ivec2(-1, 1),
    ivec2(-1, 0), ivec2(-1, -1), ivec2(0, -1), ivec2(1, -1));
  FaceColor first = face_color(cell, directions[0], scale, original, measured_z);
  FaceColor previous = first;
  float best = 1.;
  for (int i = 1; i <= 8; i++) {
    FaceColor current = first;
    if (i < 8) current = face_color(cell, directions[i], scale, original, measured_z);
    float cost = previous.cost + current.cost;
    if (cost < best && abs(previous.depth - current.depth) <= .002 &&
        distance(hue(previous.color), hue(current.color)) <= .32) {
      best = cost;
      color = previous.cost <= current.cost ? previous.color : current.color;
    }
    previous = current;
  }
  return best < 1.;
}

vec3 view_point(ivec2 cell, float z) {
  vec2 ndc = 2. * (vec2(cell) + .5) / vec2(u_size) - 1.;
  return z * vec3(ndc * u_inverse_focal, -1.);
}

float height_over_table(vec3 view_position) {
  return dot(u_viewer_y.xyz, view_position) + u_viewer_y.w - u_table_plane;
}

// Inverse depth of a plane is affine in screen coordinates. Fit both
// slopes from measured samples and check a fifth point off those axes.
struct PlaneSpan {
  vec3 color;
  float inverse_depth;
  float slope;
  int extrapolated;
};

bool plane_span(ivec2 cell, ivec2 dir, int radius, out PlaneSpan span) {
  vec3 a, b;
  float za, zb;
  int p = -radius, q = radius;
  bool left = point_at(cell + dir * p, a, za);
  bool right = point_at(cell + dir * q, b, zb);
  if (!left && !right) return false;
  int extrapolated = 0;
  if (!left || !right) {
    // Extend just past a measured patch, and only if the orthogonal
    // span and a diagonal sample can still bound the gap.
    extrapolated = left ? 1 : -1;
    if (left) {
      q = p; p -= radius;
      b = a; zb = za;
      if (!point_at(cell + dir * p, a, za)) return false;
    } else {
      p = q; q += radius;
      a = b; za = zb;
      if (!point_at(cell + dir * q, b, zb)) return false;
    }
    if (float(radius) * max(za, zb) / u_projection.z > .004 ||
        !is_pigment(a) || !is_pigment(b)) return false;
  }
  float width = float(q - p) * max(za, zb) / u_projection.z;
  if (width > .026 || abs(za-zb) > .004 + width * 1.65 || color_delta(a,b) > .085) return false;
  float slope = (1. / zb - 1. / za) / float(q - p);
  float center = 1. / za - slope * float(p);
  if (center <= 0. || abs(1. / center - (za + zb) * .5) > .018) return false;
  span = PlaneSpan((a + b) * .5, center, slope, extrapolated);
  return true;
}

bool complete_plane(ivec2 cell, float scale, out vec4 filled) {
  const int radii[4] = int[4](8, 13, 20, 29);
  for (int i = 0; i < 4; i++) {
    int radius = max(2, int(round(float(radii[i]) * scale)));
    PlaneSpan x, y;
    if (!plane_span(cell, ivec2(1,0), radius, x) ||
        !plane_span(cell, ivec2(0,1), radius, y) ||
        (x.extrapolated != 0 && y.extrapolated != 0)) continue;
    vec3 rgb = (x.color + y.color) * .5;
    if (color_delta(x.color,y.color) > .085 ||
        abs(1. / x.inverse_depth - 1. / y.inverse_depth) > .0025) continue;
    float inverse = (x.inverse_depth + y.inverse_depth) * .5;
    float z = 1. / inverse;
    // Keep modeled table cutouts intact even if only the plane toggle is on.
    if (u_guard_table == 1 && height_over_table(view_point(cell,z)) <= .004) continue;
    int diagonal = max(1, radius * 3 / 4);
    for (int j = 0; j < 4; j++) {
      ivec2 sign_offset = ivec2(j < 2 ? -1 : 1, (j % 2) == 0 ? -1 : 1);
      if ((x.extrapolated != 0 && x.extrapolated != sign_offset.x) ||
          (y.extrapolated != 0 && y.extrapolated != sign_offset.y)) continue;
      ivec2 offset = diagonal * sign_offset;
      vec3 color;
      float measured;
      if (!point_at(cell + offset,color,measured) || color_delta(color,rgb) > .085) continue;
      float expected = inverse + dot(vec2(offset),vec2(x.slope,y.slope));
      if (expected > 0. && abs(1. / expected - measured) <= .0022) {
        filled = vec4(rgb,z);
        return true;
      }
    }
  }
  return false;
}


void main(){
  ivec2 cell=ivec2(gl_FragCoord.xy);
  vec4 point=texelFetch(u_points,cell,0);
  float original_depth=texelFetch(u_depth,cell,0).r;
  bool has_point=point.a>0.;
  if(point.a>.75 || (has_point && u_small_gaps==0)){
    out_color=vec4(point.rgb,1.);
    gl_FragDepth=original_depth;
    return;
  }
  float coverage = has_point ? 1. : nearby_coverage(cell);
  if (coverage <= 0.) discard;
  if (u_small_gaps == 0) {
    vec4 filled;
    if (u_plane_completion == 0 ||
        !complete_plane(cell,clamp(float(u_size.y)/1280.,.45,1.5),filled)) discard;
    out_color=vec4(filled.rgb,1.);
    gl_FragDepth=window_depth(filled.w);
    return;
  }
  float z0=has_point?view_depth(original_depth):0.;
  const ivec2 axes[4]=ivec2[4](ivec2(1,0),ivec2(0,1),ivec2(1,1),ivec2(1,-1));
  const int radii[5]=int[5](2,4,7,10,15);
  vec4 evidence[4];
  for(int i=0;i<4;i++)evidence[i]=vec4(0.);
  float scale=clamp(float(u_size.y)/1280.,.45,1.5);
  for(int radius=0;radius<5;radius++){
    int step=max(1,int(round(float(radii[radius])*scale)));
    for(int axis=0;axis<4;axis++){
      if(evidence[axis].w>0.)continue;
      vec4 hit;
      if(!supported_pair(cell,axes[axis]*step,point,z0,hit))continue;
      for(int i=0;i<4;i++){
        vec4 other=evidence[i];
        if(other.w<=0. || abs(other.w-hit.w)>.005 || color_delta(other.rgb,hit.rgb)>.114)continue;
        vec3 recovered=(hit.rgb+other.rgb)*.5;
        float recovered_z=(hit.w+other.w)*.5;
        if(!has_point && !above_table(cell,recovered,recovered_z))continue;
        out_color=vec4(recovered,1.);
        gl_FragDepth=has_point?original_depth:window_depth(recovered_z);
        return;
      }
      evidence[axis]=hit;
    }
  }
  vec4 repaired;
  if (coverage >= .22 && repair_supported_corner(cell, point, z0, scale, repaired) &&
      (has_point || above_table(cell, repaired.rgb, repaired.w))) {
    out_color = vec4(repaired.rgb, 1.);
    gl_FragDepth = has_point ? original_depth : window_depth(repaired.w);
    return;
  }
  if(!has_point){
    vec4 repaired;
    if((!close_supported_seam(cell,repaired) || !above_table(cell,repaired.rgb,repaired.w)) &&
       (u_plane_completion==0 || !complete_plane(cell,scale,repaired))) discard;
    out_color=vec4(repaired.rgb,1.);
    gl_FragDepth=window_depth(repaired.w);
    return;
  }
  vec3 color;
  out_color = vec4(repair_face_color(cell, point.rgb, z0, scale, color) ? color : point.rgb, 1.);
  gl_FragDepth = original_depth;
}
`;

export class SquareSurfacePreview {
  static prepare(gl) {
    return prepareGpuProgram(gl, FULLSCREEN_VERTEX_SHADER, FRAGMENT);
  }

  constructor(gl, program = compileGpuProgram(gl, FULLSCREEN_VERTEX_SHADER, FRAGMENT)) {
    this.gl = gl;
    this.program = program;
    this.vao = gl.createVertexArray();
    this.uniforms = Object.fromEntries(
      ["u_points", "u_depth", "u_size", "u_coverage_level", "u_projection", "u_inverse_focal", "u_viewer_y", "u_table_plane", "u_guard_table", "u_small_gaps", "u_plane_completion"]
        .map((name) => [name, gl.getUniformLocation(this.program, name)]),
    );
    this.clearColor = new Float32Array(4);
    this.clearDepth = new Float32Array([1]);
  }

  releaseTargets() {
    const gl = this.gl;
    if (this.framebuffer) gl.deleteFramebuffer(this.framebuffer);
    if (this.color) gl.deleteTexture(this.color);
    if (this.depth) gl.deleteTexture(this.depth);
    this.framebuffer = this.color = this.depth = null;
  }

  texture(internalFormat, levels, width, height) {
    const gl = this.gl;
    const texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.texStorage2D(gl.TEXTURE_2D, levels, internalFormat, width, height);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, levels > 1 ? gl.LINEAR_MIPMAP_NEAREST : gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    return texture;
  }

  begin(width, height) {
    const gl = this.gl;
    if (!this.framebuffer || width !== this.width || height !== this.height) {
      this.releaseTargets();
      this.width = width;
      this.height = height;
      this.coverageLevel = Math.min(5, Math.floor(Math.log2(Math.min(width, height))));
      gl.activeTexture(gl.TEXTURE0);
      this.color = this.texture(gl.RGBA8, this.coverageLevel + 1, width, height);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAX_LEVEL, this.coverageLevel);
      this.depth = this.texture(gl.DEPTH_COMPONENT24, 1, width, height);
      this.framebuffer = gl.createFramebuffer();
      gl.bindFramebuffer(gl.FRAMEBUFFER, this.framebuffer);
      gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, this.color, 0);
      gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.TEXTURE_2D, this.depth, 0);
      if (gl.checkFramebufferStatus(gl.FRAMEBUFFER) !== gl.FRAMEBUFFER_COMPLETE) {
        gl.bindFramebuffer(gl.FRAMEBUFFER, null);
        this.releaseTargets();
        throw new Error("Surface preview framebuffer unavailable");
      }
    } else {
      gl.bindFramebuffer(gl.FRAMEBUFFER, this.framebuffer);
    }
    gl.viewport(0, 0, width, height);
    gl.depthMask(true);
    gl.clearBufferfv(gl.COLOR, 0, this.clearColor);
    gl.clearBufferfv(gl.DEPTH, 0, this.clearDepth);
  }

  draw({ projection, viewer }, tablePlane, settings = {}) {
    const gl = this.gl;
    const u = this.uniforms;
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.viewport(0, 0, this.width, this.height);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.color);
    if (this.coverageLevel > 0) gl.generateMipmap(gl.TEXTURE_2D);
    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, this.depth);
    gl.bindVertexArray(this.vao);
    gl.useProgram(this.program);
    gl.uniform1i(u.u_points, 0);
    gl.uniform1i(u.u_depth, 1);
    gl.uniform2i(u.u_size, this.width, this.height);
    gl.uniform1i(u.u_coverage_level, this.coverageLevel);
    gl.uniform3f(u.u_projection, projection[10], projection[14], projection[5] * this.height / 2);
    gl.uniform2f(u.u_inverse_focal, 1 / projection[0], 1 / projection[5]);
    gl.uniform4f(u.u_viewer_y, viewer[1], viewer[5], viewer[9], viewer[13]);
    gl.uniform1i(u.u_small_gaps, settings.squares_fill_surface_gaps === false ? 0 : 1);
    gl.uniform1i(u.u_plane_completion, settings.squares_plane_completion === true ? 1 : 0);
    gl.uniform1i(u.u_guard_table, Number.isFinite(tablePlane) ? 1 : 0);
    gl.uniform1f(u.u_table_plane, tablePlane ?? 0);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
    gl.bindVertexArray(null);
    gl.activeTexture(gl.TEXTURE0);
  }

  dispose() {
    this.releaseTargets();
    this.gl.deleteProgram(this.program);
    this.gl.deleteVertexArray(this.vao);
  }
}
