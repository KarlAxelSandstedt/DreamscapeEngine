#version 300 es

/* Note: input sizes are in pixels */

uniform vec2	resolution;

layout(location = 0) in vec4 node_rect;		/* shared: center[2], size[2] 	*/
layout(location = 1) in vec4 visible_rect;	/* shared: center[2], size[2] 	*/
layout(location = 2) in vec4 uv_rect;		/* shared: center[2], size[2] 	*/
layout(location = 3) in vec4 background_color;	/* shared			*/
layout(location = 4) in vec4 border_color;	/* shared			*/
layout(location = 5) in vec4 sprite_color;	/* shared			*/
layout(location = 6) in vec3 extra;		/* shared: border_size[1]
					   	   shared: corner_radius[1]
					   	   shared: edge_softness[1] 	*/
layout(location = 7) in mat4 gradient_color; 	/* shared                       */

out vec2 	uv;
out vec2 	n_rect_center;
out vec2 	v_rect_center;
out vec2 	n_rect_halfsize;
out vec2 	v_rect_halfsize;
out vec4 	bg_color;
out vec4 	br_color;
out vec4 	sp_color;
out vec4 	gr_color;
out float 	border_size;
out float 	corner_radius;
out float 	edge_softness;

/* br - tr - tl - bl */
const vec4 x_sign = vec4( 1.0, 1.0, -1.0, -1.0);
const vec4 y_sign = vec4(-1.0, 1.0,  1.0, -1.0);

void main()
{
	n_rect_center = node_rect.xy;
	v_rect_center = visible_rect.xy;
	n_rect_halfsize = node_rect.zw;
	v_rect_halfsize = visible_rect.zw;
	bg_color = background_color;
	br_color = border_color;
	sp_color = sprite_color;
	border_size = extra.x;
	corner_radius = extra.y;
	edge_softness = extra.z;

	vec2 _sign = vec2(x_sign[gl_VertexID], y_sign[gl_VertexID]);
	uv = uv_rect.xy + _sign*uv_rect.zw;
	gl_Position = vec4(2.0*((node_rect.xy + _sign*node_rect.zw) / resolution) - 1.0, 0.0, 1.0);
	gr_color = gradient_color[gl_VertexID];
}
