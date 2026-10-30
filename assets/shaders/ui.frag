#version 330 core

uniform vec2		resolution;

uniform sampler2D	texture_atlas;

in vec2 	uv;
in vec2 	n_rect_center;
in vec2 	v_rect_center;
in vec2 	n_rect_halfsize;
in vec2 	v_rect_halfsize;
in vec4 	bg_color;
in vec4 	br_color;
in vec4 	sp_color;
in vec4 	gr_color;
in float 	border_size;
in float 	corner_radius;
in float 	edge_softness;

out vec4	fragColor;

/*
 *	returns distance of point from inner rectangle formed by corners of a given corner radius, minus radius
 */
float rounded_rect_sdf(in vec2 position, in vec2 center, in vec2 halfsize, in float radius)
{
	vec2 zero = vec2( 0.0, 0.0 );
	vec2 diff = abs(position - center);
	return length(max(diff - halfsize + corner_radius, zero)) - radius;
}

/*
 *	returns distance of point from rectangle boundary (negative inside, positive outside) 
 */
float rect_sdf(in vec2 position, in vec2 center, in vec2 halfsize)
{
	vec2 diff = abs(position - center) - halfsize;
	return max(diff[0], diff[1]);
}

void main()
{
	vec2 fragment_pixel = 2.0*(gl_FragCoord.xy / resolution) - vec2(1.0, 1.0);
	if (	   gl_FragCoord.x < v_rect_center.x - v_rect_halfsize.x 
		|| gl_FragCoord.x > v_rect_center.x + v_rect_halfsize.x
		|| gl_FragCoord.y < v_rect_center.y - v_rect_halfsize.y 
		|| gl_FragCoord.y > v_rect_center.y + v_rect_halfsize.y
		)
	{
		fragColor = vec4(0.0, 0.0, 0.0, 0.0);
	}
	else
	{
		float distance;
		float softness_padding = max(0.0, 2.0 * edge_softness-1.0); 
		if (corner_radius > 0.0)
		{
			distance = rounded_rect_sdf(gl_FragCoord.xy, n_rect_center, n_rect_halfsize - softness_padding, corner_radius);
		}
		else
		{
			distance = rect_sdf(gl_FragCoord.xy, n_rect_center, n_rect_halfsize - softness_padding);
		}

		if (distance > -border_size + softness_padding) 
		{
			fragColor = br_color;
		} else 
		{
			fragColor = mix(bg_color, gr_color, gr_color.w);
		}

		/* undefined if second parameter is <= first parameter, so we force it to be positive .*/
		const float epsilon = 0.001;
		float sdf_factor = 1.0 - smoothstep(0.0, 2.0*edge_softness + epsilon, distance);
		fragColor = fragColor * sdf_factor;

		vec4 sampled = texture(texture_atlas, uv);
		sampled = vec4(mix(sampled.xyz, sp_color.xyz, sp_color.w), sampled.w);
		fragColor = mix(fragColor, sampled, sampled.w);
	}
}
