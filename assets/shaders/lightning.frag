#version 330 core

in vec3	FragPos;
in vec4 color;
in vec3 normal;

uniform vec3 light_position;

out vec4 fragColor;

void main()
{
	vec3 light_dir = normalize(FragPos - light_position);
	float diff = max(-dot(light_dir, normal), 0.0);
	fragColor = vec4(0.8 * diff * color.xyz + 0.2 * color.xyz, color.w);
}
