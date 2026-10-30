#version 300 es

layout(location = 0) in vec3 a_position;		/* local:  vertex position 	*/
layout(location = 1) in vec4 a_color;			/* local: instance color	*/
layout(location = 2) in vec3 a_normal;			/* local:  vertex normal	*/

uniform float 	aspect_ratio;
uniform mat4 	view;
uniform mat4 	perspective;

out vec3 	FragPos;
out vec4 	color;
out vec3 	normal;

void main()
{
	color = a_color;
	gl_Position = perspective * view * vec4(a_position, 1.0);
	normal = a_normal;
	FragPos = a_position;
}
