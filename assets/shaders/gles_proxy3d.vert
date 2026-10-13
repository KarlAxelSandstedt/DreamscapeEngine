#version 300 es

layout(location = 0) in vec4 a_translation_blend;	/* shared: instance translation[3] and color blend[1] 	*/
layout(location = 1) in vec4 a_rotation;		/* shared: instance rotation quaternion  		*/
layout(location = 2) in vec4 a_color;			/* shared: instance color		 		*/
layout(location = 3) in vec3 a_position;		/* local:  vertex position		 		*/
layout(location = 4) in vec3 a_normal;			/* local:  vertex normal		 		*/

uniform float 	aspect_ratio;
uniform mat4 	view;
uniform mat4 	perspective;

out vec3 	FragPos;
out vec4 	color;
out vec3 	normal;
out float	blend;

void main()
{
	float tr_part = 2.0*a_rotation[3]*a_rotation[3] - 1.0;
	float q12 = 2.0*a_rotation[0]*a_rotation[1];
	float q13 = 2.0*a_rotation[0]*a_rotation[2];
	float q10 = 2.0*a_rotation[0]*a_rotation[3];
	float q23 = 2.0*a_rotation[1]*a_rotation[2];
	float q20 = 2.0*a_rotation[1]*a_rotation[3];
	float q30 = 2.0*a_rotation[2]*a_rotation[3];
	mat4 transform = mat4(tr_part + 2.0*a_rotation[0]*a_rotation[0], q12 + q30, q13 - q20, 0.0,
		      q12 - q30, tr_part + 2.0*a_rotation[1]*a_rotation[1], q23 + q10, 0.0,
		      q13 + q20, q23 - q10, tr_part + 2.0*a_rotation[2]*a_rotation[2], 0.0, 
		      a_translation_blend[0], a_translation_blend[1], a_translation_blend[2], 1.0);
	color = a_color;
	blend = a_translation_blend[3];
	gl_Position = perspective * view * transform * vec4(a_position, 1.0);
	normal = (transform * vec4(a_normal, 0.0)).xyz;
	FragPos = (transform * vec4(a_position, 1.0)).xyz;
}
