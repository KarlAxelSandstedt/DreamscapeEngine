#version 330 core

in vec3	FragPos;
in vec4 color;
in vec3 normal;

uniform vec3 light_position;

out vec4 fragColor;

void main()
{
	/* a zero normal is unlit: lines and degenerate triangles draw their colour as it is */
	vec3 light_dir = normalize(FragPos - light_position);
	float diff = max(-dot(light_dir, normal), 0.0);
	float lit = (dot(normal, normal) > 0.0) ? 0.8 * diff + 0.2 : 1.0;
	fragColor = vec4(lit * color.xyz, color.w);
}
