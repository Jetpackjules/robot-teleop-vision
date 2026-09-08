shader_type spatial;
render_mode unshaded;

// Neutral, deterministic studio shading for the explanatory geometry view.
// Using a bounded light term also keeps white props readable on low-end GPUs.
uniform vec4 base_color : source_color = vec4(0.7, 0.7, 0.7, 1.0);

void fragment() {
	vec3 world_normal = normalize((INV_VIEW_MATRIX * vec4(NORMAL, 0.0)).xyz);
	float shade = 0.54 + 0.46 * max(0.0, dot(world_normal, normalize(vec3(-0.4, 0.8, 0.4))));
	ALBEDO = base_color.rgb * shade;
}
