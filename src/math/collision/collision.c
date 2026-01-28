/*
==========================================================================
    Copyright (C) 2025 Axel Sandstedt 

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
==========================================================================
*/

#include <float.h>
#include "float32.h"
#include "collision.h"
#include "dynamics.h"

DEFINE_STACK(visual_segment);

dsThreadLocal struct collision_debug *debug;

struct visual_segment visual_segment_construct(const struct segment segment, const vec4 color)
{
	struct visual_segment visual =
	{
		.segment = segment,
	};
	vec4_copy(visual.color, color);
	return visual;
}

/********************************** Contact Manifold helpers **********************************/

void contact_manifold_debug_print(FILE *file, const struct contact_manifold *cm)
{
	fprintf(stderr, "Contact Manifold:\n{\n");

	fprintf(stderr, "\t.i1 = %u\n", cm->i1);
	fprintf(stderr, "\t.i2 = %u\n", cm->i2);
	fprintf(stderr, "\t.v_count = %u\n", cm->v_count);
	for (u32 i = 0; i < cm->v_count; ++i)
	{
		fprintf(stderr, "\t.v[%u] = { %f, %f, %f }\n", i, cm->v[i][0], cm->v[i][1], cm->v[i][2]);
	}
	fprintf(stderr, "\t.n = { %f, %f, %f }\n", cm->n[0], cm->n[1], cm->n[2]);
	fprintf(stderr, "}\n");
}

/********************************** GJK INTERNALS **********************************/

/**
 * Gilbert-Johnson-Keerthi intersection algorithm in 3D. Based on the original paper. 
 *
 * For understanding, see [ Collision Detection in Interactive 3D environments, chapter 4.3.1 - 4.3.8 ]
 */
struct simplex
{
	vec3 p[4];
	u64 id[4];
	f32 dot[4];
	u32 type;
};

#define SIMPLEX_0	0
#define SIMPLEX_1	1
#define SIMPLEX_2	2
#define SIMPLEX_3	3

static struct simplex gjk_internal_simplex_init(void)
{
	struct simplex simplex = 
	{
		.id = {UINT64_MAX, UINT64_MAX, UINT64_MAX, UINT64_MAX},
		.dot = { -1.0f, -1.0f, -1.0f, -1.0f },
		.type = UINT32_MAX,
	};

	return simplex;
}

static u32 gjk_internal_johnsons_algorithm(struct simplex *simplex, vec3 c_v, vec4 lambda)
{
	vec3 a;

	if (simplex->type == 0)
	{
		vec3_copy(c_v, simplex->p[0]);
	}
	else if (simplex->type == 1)
	{
		vec3_sub(a, simplex->p[0], simplex->p[1]);
		const f32 delta_01_1 = vec3_dot(a, simplex->p[0]);

		if (delta_01_1 > 0.0f)
		{
			vec3_sub(a, simplex->p[1], simplex->p[0]);
			const f32 delta_01_0 = vec3_dot(a, simplex->p[1]);
			if (delta_01_0 > 0.0f)
			{
				const f32 delta = delta_01_0 + delta_01_1;
				lambda[0] = delta_01_0 / delta;
				lambda[1] = delta_01_1 / delta;
				vec3_set(c_v,
				       	(lambda[0]*(simplex->p[0])[0] + lambda[1]*(simplex->p[1])[0]),
				       	(lambda[0]*(simplex->p[0])[1] + lambda[1]*(simplex->p[1])[1]),
				       	(lambda[0]*(simplex->p[0])[2] + lambda[1]*(simplex->p[1])[2]));
			}
			else
			{
				simplex->type = 0;
				vec3_copy(c_v, simplex->p[1]);
				vec3_copy(simplex->p[0], simplex->p[1]);
			}
		}
		else
		{
			/* 
			 * numerical issues, new simplex should always contain newly added point
			 * of simplex, terminate next iteration. Let c_v stay the same as in the
			 * previous iteration.
			 */
			return 1;
		}
	}
	else if (simplex->type == 2)
	{
		vec3_sub(a, simplex->p[1], simplex->p[0]);
		const f32 delta_01_0 = vec3_dot(a, simplex->p[1]);
		vec3_sub(a, simplex->p[0], simplex->p[1]);
		const f32 delta_01_1 = vec3_dot(a, simplex->p[0]);
		vec3_sub(a, simplex->p[0], simplex->p[2]);
		const f32 delta_012_2 = delta_01_0 * vec3_dot(a, simplex->p[0]) + delta_01_1 * vec3_dot(a, simplex->p[1]);
		if (delta_012_2 > 0.0f)
		{
			vec3_sub(a, simplex->p[2], simplex->p[0]);
			const f32 delta_02_0 = vec3_dot(a, simplex->p[2]);
			vec3_sub(a, simplex->p[0], simplex->p[2]);
			const f32 delta_02_2 = vec3_dot(a, simplex->p[0]);
			vec3_sub(a, simplex->p[0], simplex->p[1]);
			const f32 delta_012_1 = delta_02_0 * vec3_dot(a, simplex->p[0]) + delta_02_2 * vec3_dot(a, simplex->p[2]);
			if (delta_012_1 > 0.0f)
			{
				vec3_sub(a, simplex->p[2], simplex->p[1]);
				const f32 delta_12_1 = vec3_dot(a, simplex->p[2]);
				vec3_sub(a, simplex->p[1], simplex->p[2]);
				const f32 delta_12_2 = vec3_dot(a, simplex->p[1]);
				vec3_sub(a, simplex->p[1], simplex->p[0]);
				const f32 delta_012_0 = delta_12_1 * vec3_dot(a, simplex->p[1]) + delta_12_2 * vec3_dot(a, simplex->p[2]);
				if (delta_012_0 > 0.0f)
				{
					const f32 delta = delta_012_0 + delta_012_1 + delta_012_2;
					lambda[0] = delta_012_0 / delta;
					lambda[1] = delta_012_1 / delta;
					lambda[2] = delta_012_2 / delta;
					vec3_set(c_v,
						(lambda[0]*(simplex->p[0])[0] + lambda[1]*(simplex->p[1])[0] + lambda[2]*(simplex->p[2])[0]),
						(lambda[0]*(simplex->p[0])[1] + lambda[1]*(simplex->p[1])[1] + lambda[2]*(simplex->p[2])[1]),
						(lambda[0]*(simplex->p[0])[2] + lambda[1]*(simplex->p[1])[2] + lambda[2]*(simplex->p[2])[2]));
				}
				else
				{
					if (delta_12_2 > 0.0f)
					{
						if (delta_12_1 > 0.0f)
						{
							const f32 delta = delta_12_1 + delta_12_2;
							lambda[0] = delta_12_1 / delta;
							lambda[1] = delta_12_2 / delta;
							vec3_set(c_v,
							       	(lambda[0]*(simplex->p[1])[0] + lambda[1]*(simplex->p[2])[0]),
							       	(lambda[0]*(simplex->p[1])[1] + lambda[1]*(simplex->p[2])[1]),
							       	(lambda[0]*(simplex->p[1])[2] + lambda[1]*(simplex->p[2])[2]));
							simplex->type = 1;
							vec3_copy(simplex->p[0], simplex->p[1]);
							vec3_copy(simplex->p[1], simplex->p[2]);
							simplex->id[0] = simplex->id[1];
							simplex->dot[0] = simplex->dot[1];
						}
						else
						{
							simplex->type = 0;
							vec3_copy(c_v, simplex->p[2]);
							vec3_copy(simplex->p[0], simplex->p[2]);
							simplex->id[1] = UINT32_MAX;
							simplex->dot[1] = -1.0f;
						}


					}
					else
					{
						return 1;
					}
				}

			}
			else
			{
				if (delta_02_2 > 0.0f)
				{
					if (delta_02_0 > 0.0f)
					{
						const f32 delta = delta_02_0 + delta_02_2;
						lambda[0] = delta_02_0 / delta;
						lambda[1] = delta_02_2 / delta;
						vec3_set(c_v,
						       	(lambda[0]*(simplex->p[0])[0] + lambda[1]*(simplex->p[2])[0]),
						       	(lambda[0]*(simplex->p[0])[1] + lambda[1]*(simplex->p[2])[1]),
						       	(lambda[0]*(simplex->p[0])[2] + lambda[1]*(simplex->p[2])[2]));
						simplex->type = 1;
						vec3_copy(simplex->p[1], simplex->p[2]);
					}
					else
					{
						simplex->type = 0;
						vec3_copy(c_v, simplex->p[2]);
						vec3_copy(simplex->p[0], simplex->p[2]);
						simplex->id[1] = UINT32_MAX;
						simplex->dot[1] = -1.0f;
					}
				}
			}
		}
		else
		{
			return 1;
		}
	}
	else
	{
		vec3_sub(a, simplex->p[1], simplex->p[0]);
		const f32 delta_01_0 = vec3_dot(a, simplex->p[1]);
		vec3_sub(a, simplex->p[0], simplex->p[1]);
		const f32 delta_01_1 = vec3_dot(a, simplex->p[0]);
		vec3_sub(a, simplex->p[0], simplex->p[2]);
		const f32 delta_012_2 = delta_01_0 * vec3_dot(a, simplex->p[0]) + delta_01_1 * vec3_dot(a, simplex->p[1]);

		vec3_sub(a, simplex->p[2], simplex->p[0]);
		const f32 delta_02_0 = vec3_dot(a, simplex->p[2]);
		vec3_sub(a, simplex->p[0], simplex->p[2]);
		const f32 delta_02_2 = vec3_dot(a, simplex->p[0]);
		vec3_sub(a, simplex->p[0], simplex->p[1]);
		const f32 delta_012_1 = delta_02_0 * vec3_dot(a, simplex->p[0]) + delta_02_2 * vec3_dot(a, simplex->p[2]);

		vec3_sub(a, simplex->p[2], simplex->p[1]);
		const f32 delta_12_1 = vec3_dot(a, simplex->p[2]);
		vec3_sub(a, simplex->p[1], simplex->p[2]);
		const f32 delta_12_2 = vec3_dot(a, simplex->p[1]);
		vec3_sub(a, simplex->p[1], simplex->p[0]);
		const f32 delta_012_0 = delta_12_1 * vec3_dot(a, simplex->p[1]) + delta_12_2 * vec3_dot(a, simplex->p[2]);

		vec3_sub(a, simplex->p[0], simplex->p[3]);
		const f32 delta_0123_3 = delta_012_0 * vec3_dot(a, simplex->p[0]) + delta_012_1 * vec3_dot(a, simplex->p[1]) + delta_012_2 * vec3_dot(a, simplex->p[2]);

		if (delta_0123_3 > 0.0f)
		{
			vec3_sub(a, simplex->p[0], simplex->p[3]);
			const f32 delta_013_3 = delta_01_0 * vec3_dot(a, simplex->p[0]) + delta_01_1 * vec3_dot(a, simplex->p[1]);

			vec3_sub(a, simplex->p[3], simplex->p[0]);
			const f32 delta_03_0 = vec3_dot(a, simplex->p[3]);
			vec3_sub(a, simplex->p[0], simplex->p[3]);
			const f32 delta_03_3 = vec3_dot(a, simplex->p[0]);
			vec3_sub(a, simplex->p[0], simplex->p[1]);
			const f32 delta_013_1 = delta_03_0 * vec3_dot(a, simplex->p[0]) + delta_03_3 * vec3_dot(a, simplex->p[3]);

			vec3_sub(a, simplex->p[3], simplex->p[1]);
			const f32 delta_13_1 = vec3_dot(a, simplex->p[3]);
			vec3_sub(a, simplex->p[1], simplex->p[3]);
			const f32 delta_13_3 = vec3_dot(a, simplex->p[1]);
			vec3_sub(a, simplex->p[1], simplex->p[0]);
			const f32 delta_013_0 = delta_13_1 * vec3_dot(a, simplex->p[1]) + delta_13_3 * vec3_dot(a, simplex->p[3]);

			vec3_sub(a, simplex->p[0], simplex->p[2]);
			const f32 delta_0123_2 = delta_013_0 * vec3_dot(a, simplex->p[0]) + delta_013_1 * vec3_dot(a, simplex->p[1]) + delta_013_3 * vec3_dot(a, simplex->p[3]);

			if (delta_0123_2 > 0.0f)
			{
				vec3_sub(a, simplex->p[0], simplex->p[3]);
				const f32 delta_023_3 = delta_02_0 * vec3_dot(a, simplex->p[0]) + delta_02_2 * vec3_dot(a, simplex->p[2]);

				vec3_sub(a, simplex->p[0], simplex->p[2]);
				const f32 delta_023_2 = delta_03_0 * vec3_dot(a, simplex->p[0]) + delta_03_3 * vec3_dot(a, simplex->p[3]);

				vec3_sub(a, simplex->p[3], simplex->p[2]);
				const f32 delta_23_2 = vec3_dot(a, simplex->p[3]);
				vec3_sub(a, simplex->p[2], simplex->p[3]);
				const f32 delta_23_3 = vec3_dot(a, simplex->p[2]);
				vec3_sub(a, simplex->p[2], simplex->p[0]);
				const f32 delta_023_0 = delta_23_2 * vec3_dot(a, simplex->p[2]) + delta_23_3 * vec3_dot(a, simplex->p[3]);

				vec3_sub(a, simplex->p[0], simplex->p[1]);
				const f32 delta_0123_1 = delta_023_0 * vec3_dot(a, simplex->p[0]) + delta_023_2 * vec3_dot(a, simplex->p[2]) + delta_023_3 * vec3_dot(a, simplex->p[3]);

				if (delta_0123_1 > 0.0f)
				{
					vec3_sub(a, simplex->p[3], simplex->p[1]);
					const f32 delta_123_1 = delta_23_2 * vec3_dot(a, simplex->p[2]) + delta_23_3 * vec3_dot(a, simplex->p[3]);

					vec3_sub(a, simplex->p[3], simplex->p[2]);
					const f32 delta_123_2 = delta_13_1 * vec3_dot(a, simplex->p[1]) + delta_13_3 * vec3_dot(a, simplex->p[3]);

					vec3_sub(a, simplex->p[1], simplex->p[3]);
					const f32 delta_123_3 = delta_12_1 * vec3_dot(a, simplex->p[1]) + delta_12_2 * vec3_dot(a, simplex->p[2]);

					vec3_sub(a, simplex->p[3], simplex->p[0]);
					const f32 delta_0123_0 = delta_123_1 * vec3_dot(a, simplex->p[1]) + delta_123_2 * vec3_dot(a, simplex->p[2]) + delta_123_3 * vec3_dot(a, simplex->p[3]);

					if (delta_0123_0 > 0.0f)
					{
						/* intersection */
						const f32 delta = delta_0123_0 + delta_0123_1 + delta_0123_2 + delta_0123_3;
						lambda[0] = delta_0123_0 / delta;
						lambda[1] = delta_0123_1 / delta;
						lambda[2] = delta_0123_2 / delta;
						lambda[3] = delta_0123_3 / delta;
						vec3_set(c_v,
							(lambda[0]*(simplex->p[0])[0] + lambda[1]*(simplex->p[1])[0] + lambda[2]*(simplex->p[2])[0] + lambda[3]*(simplex->p[3])[0]),
							(lambda[0]*(simplex->p[0])[1] + lambda[1]*(simplex->p[1])[1] + lambda[2]*(simplex->p[2])[1] + lambda[3]*(simplex->p[3])[1]),
							(lambda[0]*(simplex->p[0])[2] + lambda[1]*(simplex->p[1])[2] + lambda[2]*(simplex->p[2])[2] + lambda[3]*(simplex->p[3])[2]));
					}
					else
					{
						/* check 123 subset */
						if (delta_123_3 > 0.0f)
						{
							if (delta_123_2 > 0.0f)
							{
								if (delta_123_1 > 0.0f)
								{
									const f32 delta = delta_123_1 + delta_123_2 + delta_123_3;
									lambda[0] = delta_123_1 / delta;
									lambda[1] = delta_123_2 / delta;
									lambda[2] = delta_123_3 / delta;
									vec3_set(c_v,
										(lambda[0]*(simplex->p[1])[0] + lambda[1]*(simplex->p[2])[0] + lambda[2]*(simplex->p[3])[0]),
										(lambda[0]*(simplex->p[1])[1] + lambda[1]*(simplex->p[2])[1] + lambda[2]*(simplex->p[3])[1]),
										(lambda[0]*(simplex->p[1])[2] + lambda[1]*(simplex->p[2])[2] + lambda[2]*(simplex->p[3])[2]));
									simplex->type = 2;
									vec3_copy(simplex->p[0], simplex->p[1]);		
									vec3_copy(simplex->p[1], simplex->p[2]);		
									vec3_copy(simplex->p[2], simplex->p[3]);		
									simplex->dot[0] = simplex->dot[1];
									simplex->dot[1] = simplex->dot[2];
									simplex->id[0] = simplex->id[1];
									simplex->id[1] = simplex->id[2];
								}
								else
								{
									/* check 23 */
									if (delta_23_3 > 0.0f)
									{
										if (delta_23_2 > 0.0f)
										{
											const f32 delta = delta_23_2 + delta_23_3;
											lambda[0] = delta_23_2 / delta;
											lambda[1] = delta_23_3 / delta;
											vec3_set(c_v,
												(lambda[0]*(simplex->p[2])[0] + lambda[1]*(simplex->p[3])[0]),
												(lambda[0]*(simplex->p[2])[1] + lambda[1]*(simplex->p[3])[1]),
												(lambda[0]*(simplex->p[2])[2] + lambda[1]*(simplex->p[3])[2]));
											simplex->type = 1;
											vec3_copy(simplex->p[0], simplex->p[2]);		
											vec3_copy(simplex->p[1], simplex->p[3]);		
											simplex->dot[0] = simplex->dot[2];
											simplex->dot[2] = -1.0f;
											simplex->id[0] = simplex->id[2];
											simplex->id[2] = UINT32_MAX;
										}
										else
										{
											vec3_copy(c_v, simplex->p[3]);
											simplex->type = 0;
											vec3_copy(simplex->p[0], simplex->p[3]);
											simplex->dot[1] = -1.0f;
											simplex->dot[2] = -1.0f;
											simplex->id[1] = UINT32_MAX;
											simplex->id[2] = UINT32_MAX;
										}
									}
									else
									{
										return 1;
									}
								}
							}
							else
							{
								/* check 13 subset */
								if (delta_13_3 > 0.0f)
								{
									if (delta_13_1 > 0.0f)
									{
										const f32 delta = delta_13_1 + delta_13_3;
										lambda[0] = delta_13_1 / delta;
										lambda[1] = delta_13_3 / delta;
										vec3_set(c_v,
											(lambda[0]*(simplex->p[1])[0] + lambda[1]*(simplex->p[3])[0]),
											(lambda[0]*(simplex->p[1])[1] + lambda[1]*(simplex->p[3])[1]),
											(lambda[0]*(simplex->p[1])[2] + lambda[1]*(simplex->p[3])[2]));
										simplex->type = 1;
										vec3_copy(simplex->p[0], simplex->p[1]);
										vec3_copy(simplex->p[1], simplex->p[3]);		
										simplex->dot[0] = simplex->dot[1];
										simplex->dot[2] = -1.0f;
										simplex->id[0] = simplex->id[1];
										simplex->id[2] = UINT32_MAX;
									}
									else
									{
										vec3_copy(c_v, simplex->p[3]);
										simplex->type = 0;
										vec3_copy(simplex->p[0], simplex->p[3]);
										simplex->dot[1] = -1.0f;
										simplex->dot[2] = -1.0f;
										simplex->id[1] = UINT32_MAX;
										simplex->id[2] = UINT32_MAX;
									}
								}
								else
								{
									return 1;
								}
							}	
						}
						else
						{
							return 1;
						}
					}
				}
				else
				{
					/* check 023 subset */
					if (delta_023_3 > 0.0f)
					{
						if (delta_023_2 > 0.0f)
						{
							if (delta_023_0 > 0.0f)
							{
								const f32 delta = delta_023_0 + delta_023_2 + delta_023_3;
								lambda[0] = delta_023_0 / delta;
								lambda[1] = delta_023_2 / delta;
								lambda[2] = delta_023_3 / delta;
								vec3_set(c_v,
									(lambda[0]*(simplex->p[0])[0] + lambda[1]*(simplex->p[2])[0] + lambda[2]*(simplex->p[3])[0]),
									(lambda[0]*(simplex->p[0])[1] + lambda[1]*(simplex->p[2])[1] + lambda[2]*(simplex->p[3])[1]),
									(lambda[0]*(simplex->p[0])[2] + lambda[1]*(simplex->p[2])[2] + lambda[2]*(simplex->p[3])[2]));
								simplex->type = 2;
								vec3_copy(simplex->p[1], simplex->p[2]);		
								vec3_copy(simplex->p[2], simplex->p[3]);		
								simplex->dot[1] = simplex->dot[2];
								simplex->id[1] = simplex->id[2];
							}
							else
							{
								/* check 23 subset */
								if (delta_23_3 > 0.0f)
								{
									if (delta_23_2 > 0.0f)
									{
										const f32 delta = delta_23_2 + delta_23_3;
										lambda[0] = delta_23_2 / delta;
										lambda[1] = delta_23_3 / delta;
										vec3_set(c_v,
											(lambda[0]*(simplex->p[2])[0] + lambda[1]*(simplex->p[3])[0]),
											(lambda[0]*(simplex->p[2])[1] + lambda[1]*(simplex->p[3])[1]),
											(lambda[0]*(simplex->p[2])[2] + lambda[1]*(simplex->p[3])[2]));
										simplex->type = 1;
										vec3_copy(simplex->p[0], simplex->p[2]);
										vec3_copy(simplex->p[1], simplex->p[3]);
										simplex->dot[0] = simplex->dot[2];
										simplex->dot[2] = -1.0f;
										simplex->id[0] = simplex->id[2];
										simplex->id[2] = UINT32_MAX;
									}
									else
									{
										vec3_copy(c_v, simplex->p[3]);
										simplex->type = 0;
										vec3_copy(simplex->p[0], simplex->p[3]);
										simplex->dot[1] = -1.0f;
										simplex->dot[2] = -1.0f;
										simplex->id[1] = UINT32_MAX;
										simplex->id[2] = UINT32_MAX;
									}
								}
								else
								{
									return 1;
								}
							}
						}
						else
						{
							/* check 03 subset */
							if (delta_03_3 > 0.0f)
							{
								if (delta_03_0 > 0.0f)
								{
									const f32 delta = delta_03_0 + delta_03_3;
									lambda[0] = delta_03_0 / delta;
									lambda[1] = delta_03_3 / delta;
									vec3_set(c_v,
										(lambda[0]*(simplex->p[0])[0] + lambda[1]*(simplex->p[3])[0]),
										(lambda[0]*(simplex->p[0])[1] + lambda[1]*(simplex->p[3])[1]),
										(lambda[0]*(simplex->p[0])[2] + lambda[1]*(simplex->p[3])[2]));
									simplex->type = 1;
									vec3_copy(simplex->p[1], simplex->p[3]);
									simplex->dot[2] = -1.0f;
									simplex->id[2] = UINT32_MAX;
								}
								else
								{
									vec3_copy(c_v, simplex->p[3]);
									simplex->type = 0;
									vec3_copy(simplex->p[0], simplex->p[3]);
									simplex->dot[1] = -1.0f;
									simplex->dot[2] = -1.0f;
									simplex->id[1] = UINT32_MAX;
									simplex->id[2] = UINT32_MAX;
								}
							}
							else
							{
								return 1;
							}
						}
					}
					else
					{
						return 1;
					}
				}
			}
			else
			{
				/* check 013 subset */
				if (delta_013_3 > 0.0f)
				{
					if (delta_013_1 > 0.0f)
					{
						if (delta_013_0 > 0.0f)
						{
							const f32 delta = delta_013_0 + delta_013_1 + delta_013_3;
							lambda[0] = delta_013_0 / delta;
							lambda[1] = delta_013_1 / delta;
							lambda[2] = delta_013_3 / delta;
							vec3_set(c_v,
								(lambda[0]*(simplex->p[0])[0] + lambda[1]*(simplex->p[1])[0] + lambda[2]*(simplex->p[3])[0]),
								(lambda[0]*(simplex->p[0])[1] + lambda[1]*(simplex->p[1])[1] + lambda[2]*(simplex->p[3])[1]),
								(lambda[0]*(simplex->p[0])[2] + lambda[1]*(simplex->p[1])[2] + lambda[2]*(simplex->p[3])[2]));
							simplex->type = 2;
							vec3_copy(simplex->p[2], simplex->p[3]);
						}
						else
						{
							/* check 13 subset */
							if (delta_13_3 > 0.0f)
							{
								if (delta_13_1 > 0.0f)
								{
									const f32 delta = delta_13_1 + delta_13_3;
									lambda[0] = delta_13_1 / delta;
									lambda[1] = delta_13_3 / delta;
									vec3_set(c_v,
										(lambda[0]*(simplex->p[1])[0] + lambda[1]*(simplex->p[3])[0]),
										(lambda[0]*(simplex->p[1])[1] + lambda[1]*(simplex->p[3])[1]),
										(lambda[0]*(simplex->p[1])[2] + lambda[1]*(simplex->p[3])[2]));
									simplex->type = 1;
									vec3_copy(simplex->p[0], simplex->p[1]);
									vec3_copy(simplex->p[1], simplex->p[3]);
									simplex->dot[2] = -1.0f;
									simplex->id[2] = UINT32_MAX;
								}
								else
								{
									vec3_copy(c_v, simplex->p[3]);
									simplex->type = 0;
									vec3_copy(simplex->p[0], simplex->p[3]);
									simplex->dot[1] = -1.0f;
									simplex->dot[2] = -1.0f;
									simplex->id[1] = UINT32_MAX;
									simplex->id[2] = UINT32_MAX;
								}
							}
							else
							{
								return 1;
							}
						}	
					}
					else
					{
						/* check 03 subset */
						if (delta_03_3 > 0.0f)
						{
							if (delta_03_0 > 0.0f)
							{
								const f32 delta = delta_03_0 + delta_03_3;
								lambda[0] = delta_03_0 / delta;
								lambda[1] = delta_03_3 / delta;
								vec3_set(c_v,
									(lambda[0]*(simplex->p[0])[0] + lambda[1]*(simplex->p[3])[0]),
									(lambda[0]*(simplex->p[0])[1] + lambda[1]*(simplex->p[3])[1]),
									(lambda[0]*(simplex->p[0])[2] + lambda[1]*(simplex->p[3])[2]));
								simplex->type = 1;
								vec3_copy(simplex->p[1], simplex->p[3]);
								simplex->dot[2] = -1.0f;
								simplex->id[2] = UINT32_MAX;
							}
							else
							{
								vec3_copy(c_v, simplex->p[3]);
								simplex->type = 0;
								vec3_copy(simplex->p[0], simplex->p[3]);
								simplex->dot[1] = -1.0f;
								simplex->dot[2] = -1.0f;
								simplex->id[1] = UINT32_MAX;
								simplex->id[2] = UINT32_MAX;
							}
						}
						else
						{
							return 1;
						}
					}
				}
				else
				{
					return 1;
				}
			}
		}
		else
		{
			return 1;
		}
	}

	return 0;
}

struct gjk_input
{
	vec3ptr v;
	vec3 pos;
	mat3 rot;
	u32 v_count;
};

static void gjk_internal_closest_points(vec3 c1, vec3 c2, struct gjk_input *in1, struct simplex *simplex, const vec4 lambda)
{
	if (simplex->type == 0)
	{
		mat3_vec_mul(c1, in1->rot, in1->v[simplex->id[0] >> 32]);
		vec3_translate(c1, in1->pos);
		vec3_sub(c2, c1, simplex->p[0]);
	}
	else
	{
		vec3 tmp1, tmp2;
		vec3_set(c1, 0.0f, 0.0f, 0.0f);
		vec3_set(c2, 0.0f, 0.0f, 0.0f);
		for (u32 i = 0; i <= simplex->type; ++i)
		{
			mat3_vec_mul(tmp1, in1->rot, in1->v[simplex->id[i] >> 32]);
			vec3_translate(tmp1, in1->pos);
			vec3_sub(tmp2, tmp1, simplex->p[i]);
			vec3_translate_scaled(c1, tmp1, lambda[i]);
			vec3_translate_scaled(c2, tmp2, lambda[i]);
		}
	}

}	

static u32 gjk_internal_support(vec3 support, const vec3 dir, struct gjk_input *in)
{
	f32 max = -FLT_MAX;
	u32 max_index = 0;
	vec3 p;
	for (u32 i = 0; i < in->v_count; ++i)
	{
		mat3_vec_mul(p, in->rot, in->v[i]);
		const f32 dot = vec3_dot(p, dir);
		if (max < dot)
		{
			max_index = i;
			max = dot; 
		}
	}

	mat3_vec_mul(support, in->rot, in->v[max_index]);
	vec3_translate(support,in->pos);
	return max_index;

}

static f32 gjk_distance_sq(vec3 c1, vec3 c2, struct gjk_input *in1, struct gjk_input *in2)
{
	ds_Assert(in1->v_count > 0);
	ds_Assert(in2->v_count > 0);
	
	const f32 abs_tol = 100.0f * F32_EPSILON;
	const f32 tol = 100.0f * F32_EPSILON;

	struct simplex simplex = gjk_internal_simplex_init();
	vec3 dir, c_v, tmp, s1, s2;
	vec4 lambda;
	u64 support_id;
	f32 ma; /* max dot product of current simplex */
	f32 dist_sq = FLT_MAX; 
	const f32 rel = tol * tol;

	/* arbitrary starting search direction */
	vec3_set(c_v, 1.0f, 0.0f, 0.0f);
	u64 old_support = UINT64_MAX;

	//TODO
	const u32 max_iter = 128;
	for (u32 i = 0; i < max_iter; ++i)
	{
		simplex.type += 1;
		vec3_scale(dir, c_v, -1.0f);

		const u32 i1 = gjk_internal_support(s1, dir, in1);
		vec3_negative_to(tmp, dir);
		const u32 i2 = gjk_internal_support(s2, tmp, in2);
		vec3_sub(simplex.p[simplex.type], s1, s2);
		support_id = ((u64) i1 << 32) | (u64) i2;

		if (dist_sq - vec3_dot(simplex.p[simplex.type], c_v) <= rel * dist_sq + abs_tol
				|| simplex.id[0] == support_id || simplex.id[1] == support_id 
				|| simplex.id[2] == support_id || simplex.id[3] == support_id)
		{
			ds_Assert(dist_sq != FLT_MAX);
			simplex.type -= 1;
			gjk_internal_closest_points(c1, c2, in1, &simplex, lambda);
			return dist_sq;
		}

		/* find closest point v to origin using naive Johnson's algorithm, update simplex data 
		 * Degenerate Case: due to numerical issues, determinant signs may flip, which may result
		 * either in wrong sub-simplex being chosen, or no valid simplex at all. In that case c_v
		 * stays the same, and we terminate the algorithm. [See page 142].
		 */
		if (gjk_internal_johnsons_algorithm(&simplex, c_v, lambda))
		{
			ds_Assert(dist_sq != FLT_MAX);
			simplex.type -= 1;
			gjk_internal_closest_points(c1, c2, in1, &simplex, lambda);
			return dist_sq;
		}

		simplex.id[simplex.type] = support_id;
		simplex.dot[simplex.type] = vec3_dot(simplex.p[simplex.type], simplex.p[simplex.type]);

		/* 
		 * If the simplex is of type 3, or a tetrahedron, we have encapsulated 0, or, if v is sufficiently
		 * close to the origin, within a margin of error, return an intersection.
		 */
		if (simplex.type == 3)
		{
			return 0.0f;
		}
		else
		{
			ma = simplex.dot[0];
			ma = f32_max(ma, simplex.dot[1]);
			ma = f32_max(ma, simplex.dot[2]);
			ma = f32_max(ma, simplex.dot[3]);

			/* For error bound discussion, see sections 4.3.5, 4.3.6 */
			dist_sq = vec3_dot(c_v, c_v);
			if (dist_sq <= abs_tol * ma)
			{
				return 0.0f;
			}
		}
	}

	return 0.0f;
}

/********************************** DISTANCE METHODS **********************************/

static f32 sphere_distance(vec3 c1, vec3 c2, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_SPHERE && b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collision_shape *shape1 = strdb_Address(pipeline->shape_db, b1->shape_handle);
	const struct collision_shape *shape2 = strdb_Address(pipeline->shape_db, b2->shape_handle);

	f32 dist_sq = 0.0f;

	const f32 r_sum = shape1->sphere.radius + shape2->sphere.radius + 2.0f * margin;
	if (vec3_distance_squared(b1->position, b2->position) > r_sum*r_sum)
	{
		vec3 dir;
		vec3_sub(dir, b2->position, b1->position);
		vec3_mul_constant(dir, 1.0f/vec3_length(dir));
		vec3_copy(c1, b1->position);
		vec3_copy(c2, b2->position);
		vec3_translate_scaled(c1, dir,  shape1->sphere.radius);
		vec3_translate_scaled(c2, dir, -shape2->sphere.radius);
		dist_sq = vec3_distance_squared(c1, c2);
	}

	return f32_sqrt(dist_sq);
}

static f32 capsule_sphere_distance(vec3 c1, vec3 c2, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CAPSULE && b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collision_shape *shape2 = strdb_Address(pipeline->shape_db, b2->shape_handle);

	const struct capsule *cap = &((struct collision_shape *) strdb_Address(pipeline->shape_db, b1->shape_handle))->capsule;
	f32 r_sum = cap->radius + shape2->sphere.radius + 2.0f * margin;

	mat3 rot;
	quat_to_mat3(rot, b1->rotation);

	vec3 s_p1, s_p2, diff;
	vec3_sub(c2, b2->position, b1->position);
	s_p1[0] = rot[1][0] * cap->half_height;	
	s_p1[1] = rot[1][1] * cap->half_height;	
	s_p1[2] = rot[1][2] * cap->half_height;	
	vec3_negative_to(s_p2, s_p1);
	struct segment s = segment_construct(s_p1, s_p2);

	f32 dist = 0.0f;
	if (segment_point_distance_sq(c1, &s, c2) > r_sum*r_sum)
	{
		vec3_translate(c1, b1->position);
		vec3_translate(c2, b1->position);
		vec3_sub(diff, c2, c1);
		vec3_mul_constant(diff, 1.0f / vec3_length(diff));
		vec3_translate_scaled(c1, diff, cap->radius);
		vec3_translate_scaled(c2, diff, -shape2->sphere.radius);

		dist = f32_sqrt(vec3_distance_squared(c1, c2));
	}

	return dist;
}

static f32 capsule_distance(vec3 c1, vec3 c2, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CAPSULE && b2->shape_type == COLLISION_SHAPE_CAPSULE);


	struct capsule *cap1 = &((struct collision_shape *) strdb_Address(pipeline->shape_db, b1->shape_handle))->capsule;
	struct capsule *cap2 = &((struct collision_shape *) strdb_Address(pipeline->shape_db, b2->shape_handle))->capsule;
	f32 r_sum = cap1->radius + cap2->radius + 2.0f * margin;

	mat3 rot;
	vec3 p0, p1; /* line points */

	quat_to_mat3(rot, b1->rotation);
	p0[0] = rot[1][0] * cap1->half_height,	
	p0[1] = rot[1][1] * cap1->half_height,	
	p0[2] = rot[1][2] * cap1->half_height,	
	vec3_negative_to(p1, p0);
	vec3_translate(p0, b1->position);
	vec3_translate(p1, b1->position);
	struct segment s1 = segment_construct(p0, p1);
	
	quat_to_mat3(rot, b2->rotation);
	p0[0] = rot[1][0] * cap2->half_height,	
	p0[1] = rot[1][1] * cap2->half_height,	
	p0[2] = rot[1][2] * cap2->half_height,	
	vec3_negative_to(p1, p0);
	vec3_translate(p0, b2->position);
	vec3_translate(p1, b2->position);
	struct segment s2 = segment_construct(p0, p1);

	f32 dist = 0.0f;
	if (segment_distance_sq(c1, c2, &s1, &s2) > r_sum*r_sum)
	{
		vec3_sub(p0, c2, c1);
		vec3_normalize(p1, p0);
		vec3_translate_scaled(c1, p1, cap1->radius);
		vec3_translate_scaled(c2, p1, -cap2->radius);
		dist = f32_sqrt(vec3_distance_squared(c1, c2));
	}

	return dist;
}

static f32 hull_sphere_distance(vec3 c1, vec3 c2, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CONVEX_HULL);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collision_shape *shape1 = strdb_Address(pipeline->shape_db, b1->shape_handle);
	const struct collision_shape *shape2 = strdb_Address(pipeline->shape_db, b2->shape_handle);

	struct gjk_input g1 = { .v = shape1->hull.v, .v_count = shape1->hull.v_count, };
	vec3_copy(g1.pos, b1->position);
	quat_to_mat3(g1.rot, b1->rotation);

	vec3 n = VEC3_ZERO;
	struct gjk_input g2 = { .v = &n, .v_count = 1, };
	vec3_copy(g2.pos, b2->position);
	mat3_identity(g2.rot);

	f32 dist_sq = gjk_distance_sq(c1, c2, &g1, &g2);
	const f32 r_sum = shape2->sphere.radius + 2.0f * margin;

	if (dist_sq <= r_sum*r_sum)
	{  
		dist_sq = 0.0f;
	}
	else
	{
		vec3_sub(n, c2, c1);
		vec3_mul_constant(n, 1.0f / vec3_length(n));
		vec3_translate_scaled(c1, n, margin);
		vec3_translate_scaled(c2, n, -(shape2->sphere.radius + margin));
	}

	return f32_sqrt(dist_sq);
}

static f32 hull_capsule_distance(vec3 c1, vec3 c2, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CONVEX_HULL);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_CAPSULE);

	const struct collision_shape *shape1 = strdb_Address(pipeline->shape_db, b1->shape_handle);
	const struct collision_shape *shape2 = strdb_Address(pipeline->shape_db, b2->shape_handle);

	struct gjk_input g1 = { .v = shape1->hull.v, .v_count = shape1->hull.v_count, };
	vec3_copy(g1.pos, b1->position);
	quat_to_mat3(g1.rot, b1->rotation);

	vec3 segment[2];
	vec3_set(segment[0], 0.0f, shape2->capsule.half_height, 0.0f);
	vec3_set(segment[1], 0.0f, -shape2->capsule.half_height, 0.0f);
	struct gjk_input g2 = { .v = segment, .v_count = 2, };
	vec3_copy(g2.pos, b2->position);
	mat3_identity(g2.rot);

	f32 dist_sq = gjk_distance_sq(c1, c2, &g1, &g2);
	const f32 r_sum = shape2->capsule.radius + 2.0f * margin;

	if (dist_sq <= r_sum*r_sum)
	{
		dist_sq = 0.0f;
	}
	else
	{
		vec3 n;
		vec3_sub(n, c2, c1);
		vec3_mul_constant(n, 1.0f / vec3_length(n));
		vec3_translate_scaled(c1, n, margin);
		vec3_translate_scaled(c2, n, -(shape2->sphere.radius + margin));
	}

	return f32_sqrt(dist_sq);

}

static f32 hull_distance(vec3 c1, vec3 c2, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert (b1->shape_type == COLLISION_SHAPE_CONVEX_HULL);
	ds_Assert (b2->shape_type == COLLISION_SHAPE_CONVEX_HULL);

	const struct collision_shape *shape1 = strdb_Address(pipeline->shape_db, b1->shape_handle);
	const struct collision_shape *shape2 = strdb_Address(pipeline->shape_db, b2->shape_handle);

	struct gjk_input g1 = { .v = shape1->hull.v, .v_count = shape1->hull.v_count, };
	vec3_copy(g1.pos, b1->position);
	quat_to_mat3(g1.rot, b1->rotation);

	struct gjk_input g2 = { .v = shape2->hull.v, .v_count = shape2->hull.v_count, };
	vec3_copy(g2.pos, b2->position);
	quat_to_mat3(g2.rot, b2->rotation);

	f32 dist_sq = gjk_distance_sq(c1, c2, &g1, &g2);
	if (dist_sq <= 4.0f*margin*margin)
	{
		dist_sq = 0.0f;
		vec3 n;
		vec3_sub(n, c2, c1);
		vec3_mul_constant(n, 1.0f / vec3_length(n));
		vec3_translate_scaled(c1, n, margin);
		vec3_translate_scaled(c2, n, margin);
	}

	return f32_sqrt(dist_sq);
}

static f32 tri_mesh_bvh_sphere_distance(vec3 c1, vec3 c2, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_AssertString(0, "implement");
	return 0.0f;
}

static f32 tri_mesh_bvh_capsule_distance(vec3 c1, vec3 c2, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_AssertString(0, "implement");
	return 0.0f;
}

static f32 tri_mesh_bvh_hull_distance(vec3 c1, vec3 c2, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_AssertString(0, "implement");
	return 0.0f;
}

/********************************** INTERSECTION TESTS **********************************/

static u32 sphere_test(const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_SPHERE && b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collision_shape *shape1 = strdb_Address(pipeline->shape_db, b1->shape_handle);
	const struct collision_shape *shape2 = strdb_Address(pipeline->shape_db, b2->shape_handle);
	
	const f32 r_sum = shape1->sphere.radius + shape2->sphere.radius + 2.0f * margin;
	return vec3_distance_squared(b1->position, b2->position) <= r_sum*r_sum;
}

static u32 capsule_sphere_test(const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CAPSULE && b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collision_shape *shape1 = strdb_Address(pipeline->shape_db, b1->shape_handle);
	const struct collision_shape *shape2 = strdb_Address(pipeline->shape_db, b2->shape_handle);

	const struct capsule *cap = &shape1->capsule;
	f32 r_sum = cap->radius + shape2->sphere.radius + 2.0f * margin;

	mat3 rot;
	quat_to_mat3(rot, b1->rotation);

	vec3 c1, c2, s_p1, s_p2;
	vec3_sub(c2, b2->position, b1->position);
	s_p1[0] = rot[1][0] * cap->half_height;	
	s_p1[1] = rot[1][1] * cap->half_height;	
	s_p1[2] = rot[1][2] * cap->half_height;	
	vec3_negative_to(s_p2, s_p1);
	struct segment s = segment_construct(s_p1, s_p2);

	return segment_point_distance_sq(c1, &s, c2) <= r_sum*r_sum;
}

static u32 capsule_test(const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	vec3 c1, c2;
	return capsule_distance(c1, c2, pipeline, b1, b2, margin) == 0.0f;
}

static u32 hull_sphere_test(const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	vec3 c1, c2;
	return hull_sphere_distance(c1, c2, pipeline, b1, b2, margin) == 0.0f;
}

static u32 hull_capsule_test(const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	vec3 c1, c2;
	return hull_capsule_distance(c1, c2, pipeline, b1, b2, margin) == 0.0f;
}

static u32 hull_test(const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	vec3 c1, c2;
	return hull_distance(c1, c2, pipeline, b1, b2, margin) == 0.0f;
}

static u32 tri_mesh_bvh_sphere_test(const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	vec3 c1, c2;
	return tri_mesh_bvh_sphere_distance(c1, c2, pipeline, b1, b2, margin) == 0.0f;
}

static u32 tri_mesh_bvh_capsule_test(const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	vec3 c1, c2;
	return tri_mesh_bvh_capsule_distance(c1, c2, pipeline, b1, b2, margin) == 0.0f;
}

static u32 tri_mesh_bvh_hull_test(const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	vec3 c1, c2;
	return tri_mesh_bvh_hull_distance(c1, c2, pipeline, b1, b2, margin) == 0.0f;
}

/********************************** CONTACT MANIFOLD METHODS **********************************/

static u32 sphere_contact(struct arena *garbage, struct collision_result *result, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_SPHERE);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collision_shape *shape1 = strdb_Address(pipeline->shape_db, b1->shape_handle);
	const struct collision_shape *shape2 = strdb_Address(pipeline->shape_db, b2->shape_handle);

	result->type = COLLISION_NONE;
	u32 contact_generated = 0;

	const f32 r_sum = shape1->sphere.radius + shape2->sphere.radius + 2.0f * margin;
	const f32 dist_sq = vec3_distance_squared(b1->position, b2->position);
	if (dist_sq <= r_sum*r_sum)
	{
		result->type = COLLISION_CONTACT;
		contact_generated = 1;
		result->manifold.v_count = 1;
		if (dist_sq <= COLLISION_POINT_DIST_SQ)
		{
			//TODO(Degenerate): spheres have same center => normal returned should depend on the context.
			vec3_set(result->manifold.n, 0.0f, 1.0f, 0.0f);
		}
		else
		{
			vec3_sub(result->manifold.n, b2->position, b1->position);
			vec3_mul_constant(result->manifold.n, 1.0f/vec3_length(result->manifold.n));
		}

		vec3 c1, c2;
		vec3_copy(c1, b1->position);
		vec3_copy(c2, b2->position);
		vec3_translate_scaled(c1, result->manifold.n, shape1->sphere.radius + margin);
		vec3_translate_scaled(c2, result->manifold.n, -(shape2->sphere.radius + margin));
		result->manifold.depth[0] = vec3_dot(c1, result->manifold.n) - vec3_dot(c2, result->manifold.n);
		vec3_interpolate(result->manifold.v[0], c1, c2, 0.5f);
	}

	return contact_generated;
}

static u32 capsule_sphere_contact(struct arena *garbage, struct collision_result *result, const struct physics_pipeline *pipeline,  const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CAPSULE);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collision_shape *shape1 = strdb_Address(pipeline->shape_db, b1->shape_handle);
	const struct collision_shape *shape2 = strdb_Address(pipeline->shape_db, b2->shape_handle);

	result->type = COLLISION_NONE;
	u32 contact_generated = 0;

	const struct capsule *cap = &shape1->capsule;
	const f32 r_sum = cap->radius + shape2->sphere.radius + 2.0f * margin;

	mat3 rot;
	quat_to_mat3(rot, b1->rotation);

	vec3 c1, c2, s_p1, s_p2, diff;
	vec3_sub(c2, b2->position, b1->position);
	s_p1[0] = rot[1][0] * cap->half_height;	
	s_p1[1] = rot[1][1] * cap->half_height;	
	s_p1[2] = rot[1][2] * cap->half_height;	
	vec3_negative_to(s_p2, s_p1);
	struct segment s = segment_construct(s_p1, s_p2);
	const f32 dist_sq = segment_point_distance_sq(c1, &s, c2);

	if (dist_sq <= r_sum*r_sum)
	{
		result->type = COLLISION_CONTACT;
		contact_generated = 1;
		result->manifold.v_count = 1;
		if (dist_sq <= COLLISION_POINT_DIST_SQ)
		{
			//TODO Degerate case: normal should be context dependent
			vec3_copy(result->manifold.v[0], b1->position);
			if (s.dir[0]*s.dir[0] < s.dir[1]*s.dir[1])
			{
				if (s.dir[0]*s.dir[0] < s.dir[2]*s.dir[2]) { vec3_set(result->manifold.v[2], 1.0f, 0.0f, 0.0f); }
				else { vec3_set(result->manifold.v[2], 0.0f, 0.0f, 1.0f); }
			}
			else
			{
				if (s.dir[1]*s.dir[1] < s.dir[2]*s.dir[2]) { vec3_set(result->manifold.v[0], 0.0f, 1.0f, 0.0f); }
				else { vec3_set(result->manifold.v[2], 0.0f, 0.0f, 1.0f); }
			}
				
			vec3_set(result->manifold.v[2], 1.0f, 0.0f, 0.0f);
			vec3_cross(diff, result->manifold.v[2], s.dir);
			vec3_normalize(result->manifold.n, diff);
			result->manifold.depth[0] = r_sum;
		}
		else
		{
			vec3_sub(diff, c2, c1);
			vec3_normalize(result->manifold.n, diff);
			vec3_translate_scaled(c1, result->manifold.n, cap->radius + margin);
			vec3_translate_scaled(c2, result->manifold.n, -(shape2->sphere.radius + margin));
			result->manifold.depth[0] = vec3_dot(c1, result->manifold.n) - vec3_dot(c2, result->manifold.n);
			vec3_interpolate(result->manifold.v[0], c1, c2, 0.5f);
			vec3_translate(result->manifold.v[0], b1->position);
		}	
	}

	return contact_generated;
}

static u32 capsule_contact(struct arena *garbage, struct collision_result *result, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CAPSULE);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_CAPSULE);

	u32 contact_generated = 0;
	result->type = COLLISION_NONE;

	const struct capsule *cap1 = &((struct collision_shape *) strdb_Address(pipeline->shape_db, b1->shape_handle))->capsule;
	const struct capsule *cap2 = &((struct collision_shape *) strdb_Address(pipeline->shape_db, b2->shape_handle))->capsule;
	f32 r_sum = cap1->radius + cap2->radius + 2.0f * margin;

	mat3 rot;
	vec3 c1, c2, p0, p1; /* line points */

	quat_to_mat3(rot, b1->rotation);
	p0[0] = rot[1][0] * cap1->half_height;	
	p0[1] = rot[1][1] * cap1->half_height;	
	p0[2] = rot[1][2] * cap1->half_height;	
	vec3_negative_to(p1, p0);
	vec3_translate(p0, b1->position);
	vec3_translate(p1, b1->position);
	struct segment s1 = segment_construct(p0, p1);
	
	quat_to_mat3(rot, b2->rotation);
	p0[0] = rot[1][0] * cap2->half_height;	
	p0[1] = rot[1][1] * cap2->half_height;	
	p0[2] = rot[1][2] * cap2->half_height;	
	vec3_negative_to(p1, p0);
	vec3_translate(p0, b2->position);
	vec3_translate(p1, b2->position);
	struct segment s2 = segment_construct(p0, p1);

	const f32 dist_sq = segment_distance_sq(c1, c2, &s1, &s2);
	if (dist_sq <= r_sum*r_sum)
	{
		result->type = COLLISION_CONTACT;
		contact_generated = 1;
		vec3 cross;
		vec3_cross(cross, s1.dir, s2.dir);
		const f32 cross_dist_sq = vec3_length_squared(cross);
		if (dist_sq <= COLLISION_POINT_DIST_SQ)
		{
			/* Degenerate Case 1: Parallel capsules,*/
			result->manifold.depth[0] = r_sum;
			vec3_copy(result->manifold.v[0], b1->position);
			if (cross_dist_sq <= COLLISION_POINT_DIST_SQ)
			{
				result->manifold.v_count = 1;

				//TODO Normal should be context dependent
				if (s1.dir[0]*s1.dir[0] < s1.dir[1]*s1.dir[1])
				{
					if (s1.dir[0]*s1.dir[0] < s1.dir[2]*s1.dir[2]) { vec3_set(result->manifold.n, 1.0f, 0.0f, 0.0f); }
					else { vec3_set(result->manifold.n, 0.0f, 0.0f, 1.0f); }
				}
				else
				{
					if (s1.dir[1]*s1.dir[1] < s1.dir[2]*s1.dir[2]) { vec3_set(result->manifold.n, 0.0f, 1.0f, 0.0f); }
					else { vec3_set(result->manifold.n, 0.0f, 0.0f, 1.0f); }
				}
				vec3_cross(p0, s1.dir, result->manifold.n);
				vec3_normalize(result->manifold.n, p0);
			}
			/* Degenerate Case 2: Non-Parallel capsules, */
			else
			{
				result->manifold.v_count = 1;
				vec3_normalize(result->manifold.n, cross);
			}
		}
		else
		{
			vec3_sub(result->manifold.n, c2, c1);
			vec3_mul_constant(result->manifold.n, 1.0f / vec3_length(result->manifold.n));
			vec3_translate_scaled(c1, result->manifold.n, cap1->radius + margin);
			vec3_translate_scaled(c2, result->manifold.n, -(cap2->radius + margin));
			const f32 d = vec3_dot(c1, result->manifold.n) - vec3_dot(c2, result->manifold.n);
			result->manifold.depth[0] = d;
			if (cross_dist_sq <= COLLISION_POINT_DIST_SQ)
			{
				const f32 t1 = segment_point_closest_bc_parameter(&s1, s2.p0);
				const f32 t2 = segment_point_closest_bc_parameter(&s1, s2.p1);

				if (t1 != t2)
				{
					result->manifold.v_count = 2;
					result->manifold.depth[1] = d;
					segment_bc(result->manifold.v[0], &s1, t1);
					segment_bc(result->manifold.v[1], &s1, t2);
				}
				/* end-point contact point */
				else
				{
					result->manifold.v_count = 1;
					vec3_interpolate(result->manifold.v[0], c1, c2, 0.5f);
				}
			}
			else
			{
				result->manifold.v_count = 1;
				vec3_interpolate(result->manifold.v[0], c1, c2, 0.5f);
			}
		}
	}

	return contact_generated;
}

static u32 hull_sphere_contact(struct arena *garbage, struct collision_result *result, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert (b1->shape_type == COLLISION_SHAPE_CONVEX_HULL);
	ds_Assert (b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collision_shape *shape1 = strdb_Address(pipeline->shape_db, b1->shape_handle);
	const struct collision_shape *shape2 = strdb_Address(pipeline->shape_db, b2->shape_handle);

	result->type = COLLISION_NONE;
	u32 contact_generated = 0;

	struct gjk_input g1 = { .v = shape1->hull.v, .v_count = shape1->hull.v_count, };
	vec3_copy(g1.pos, b1->position);
	quat_to_mat3(g1.rot, b1->rotation);

	vec3 zero = VEC3_ZERO;
	struct gjk_input g2 = { .v = &zero, .v_count = 1, };
	vec3_copy(g2.pos, b2->position);
	mat3_identity(g2.rot);

	vec3 c1, c2;
	const f32 dist_sq = gjk_distance_sq(c1, c2, &g1, &g2);
	const f32 r_sum = shape2->sphere.radius + 2.0f * margin;

	/* Deep Penetration */
	if (dist_sq <= margin*margin)
	{
		result->type = COLLISION_CONTACT;
		contact_generated = 1;
		result->manifold.v_count = 1;

		vec3 n;	
		const struct dcel *h = &shape1->hull;
		f32 min_depth = FLT_MAX;
		vec3 diff;
		vec3 p, best_p;
		for (u32 fi = 0; fi < h->f_count; ++fi)
		{
			dcel_face_normal(p, h, fi);
			mat3_vec_mul(n, g1.rot, p);
			mat3_vec_mul(p, g1.rot, h->v[h->e[h->f[fi].first].origin]);
			vec3_translate(p, b1->position);
			vec3_sub(diff, p, b2->position);
			const f32 depth = vec3_dot(n, diff);
			if (depth < min_depth)
			{
				vec3_copy(best_p, p);
				vec3_copy(result->manifold.n, n);
				min_depth = depth;
			}
		}

		vec3_sub(diff, best_p, b2->position);
		result->manifold.depth[0] = vec3_dot(result->manifold.n, diff) + shape2->sphere.radius + 2.0f * margin;

		vec3_copy(result->manifold.v[0], b2->position);
		vec3_translate_scaled(result->manifold.v[0], result->manifold.n, margin + min_depth);
	}
	/* Shallow Penetration */
	else if (dist_sq <= r_sum*r_sum)
	{
		result->type = COLLISION_CONTACT;
		contact_generated = 1;
		result->manifold.v_count = 1;

		vec3_sub(result->manifold.n, c2, c1);
		vec3_mul_constant(result->manifold.n, 1.0f / vec3_length(result->manifold.n));

		vec3_translate_scaled(c1, result->manifold.n, margin);
		vec3_translate_scaled(c2, result->manifold.n, -(shape2->sphere.radius + margin));
		result->manifold.depth[0] = vec3_dot(c1, result->manifold.n) - vec3_dot(c2, result->manifold.n);

		vec3_interpolate(result->manifold.v[0], c1, c2, 0.5f);
	}

	return contact_generated;
}

static u32 hull_capsule_contact(struct arena *garbage, struct collision_result *result, const struct physics_pipeline *pipeline,  const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CONVEX_HULL);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_CAPSULE);

	result->type = COLLISION_NONE;
	u32 contact_generated = 0;

	const struct collision_shape *shape1 = strdb_Address(pipeline->shape_db, b1->shape_handle);
	const struct collision_shape *shape2 = strdb_Address(pipeline->shape_db, b2->shape_handle);

	const struct dcel *h = &shape1->hull;
	struct gjk_input g1 = { .v = h->v, .v_count = h->v_count, };
	vec3_copy(g1.pos, b1->position);
	quat_to_mat3(g1.rot, b1->rotation);

	vec3 segment[2];
	vec3_set(segment[0], 0.0f, shape2->capsule.half_height, 0.0f);
	vec3_set(segment[1], 0.0f, -shape2->capsule.half_height, 0.0f);
	vec3_negative_to(segment[1], segment[0]);
	struct gjk_input g2 = { .v = segment, .v_count = 2, };
	vec3_copy(g2.pos, b2->position);
	//mat3_identity(g2.rot);
	quat_to_mat3(g2.rot, b2->rotation);

	vec3 c1, c2;
	const f32 dist_sq = gjk_distance_sq(c1, c2, &g1, &g2);
	const f32 r_sum = shape2->capsule.radius + 2.0f * margin;
	if (dist_sq <= r_sum*r_sum)
	{
		result->type = COLLISION_CONTACT;
		contact_generated = 1;

		vec3 p1, p2, tmp;
		mat3_vec_mul(p1, g2.rot, g2.v[0]);
		mat3_vec_mul(p2, g2.rot, g2.v[1]);
		vec3_translate(p1, g2.pos);
		vec3_translate(p2, g2.pos);
		struct segment cap_s = segment_construct(p1, p2);

		g2.v_count = 1;
		const u32 cap_p0_inside = (gjk_distance_sq(p1, tmp, &g1, &g2) == 0.0f) ? 1 : 0;
		vec3_copy(g2.v[0], g2.v[1]);
		const u32 cap_p1_inside = (gjk_distance_sq(p2, tmp, &g1, &g2) == 0.0f) ? 1 : 0;

		/* Deep Penetration */
		if (dist_sq <= margin*margin)
		{
			u32 edge_best = 0; 
			u32 best_index = 0;

			f32 max_d0 = -FLT_MAX;
			f32 max_d1 = -FLT_MAX;
			f32 max_signed_depth = -FLT_MAX;

			for (u32 fi = 0; fi < h->f_count; ++fi)
			{
				struct plane pl = dcel_face_plane(h, g1.rot, b1->position, fi);

				const f32 d0 = plane_point_signed_distance(&pl, cap_s.p0);
				const f32 d1 = plane_point_signed_distance(&pl, cap_s.p1);
				const f32 d = f32_min(d0, d1);
				if (max_signed_depth < d)
				{
					best_index = fi;
					max_signed_depth = d;
					max_d0 = d0;
					max_d1 = d1;
				}
			}

			/* For an edge to define seperating axis, either both or no end-points of the capsule must be inside */
			if (cap_p0_inside == cap_p1_inside)
			{
				for (u32 ei = 0; ei < h->e_count; ++ei)
				{
					struct segment edge_s = dcel_edge_segment(h, g1.rot, g1.pos, best_index);
					
					const f32 d = -f32_sqrt(segment_distance_sq(c1, c2, &edge_s, &cap_s));
					if (max_signed_depth < d)
					{
						edge_best = 1;
						best_index = ei;
						max_signed_depth = d;
						max_d0 = d;
					}
				}
			}

			//TODO Is this correct?
			result->manifold.depth[0] = f32_max(-max_d0, 0.0f);
			result->manifold.depth[1] = f32_max(-max_d1, 0.0f);
			if (edge_best)
			{
				result->manifold.v_count = 1;
				struct segment edge_s = dcel_edge_segment(h, g1.rot, g1.pos, best_index);
				segment_distance_sq(c1, c2, &edge_s, &cap_s);
				vec3_sub(result->manifold.n, c1, c2);
				vec3_mul_constant(result->manifold.n, 1.0f / vec3_length(result->manifold.n));
				vec3_copy(result->manifold.v[0], c1);
			}
			else
			{
				result->manifold.v_count = 2;
				dcel_face_normal(c1, h, best_index);
				mat3_vec_mul(result->manifold.n, g1.rot, c1);
				struct segment s = dcel_face_clip_segment(h, g1.rot, g1.pos, best_index, &cap_s);
				const struct plane pl = dcel_face_plane(h, g1.rot, g1.pos, best_index);

				//COLLISION_DEBUG_ADD_SEGMENT(cap_s);
				//COLLISION_DEBUG_ADD_SEGMENT(s);

				if (cap_p0_inside == 1 && cap_p1_inside == 0)
				{
					vec3_copy(result->manifold.v[0], s.p0);
					plane_segment_clip(result->manifold.v[1], &pl, &s);
				}
				else if (cap_p0_inside == 0 && cap_p1_inside == 1)
				{
					plane_segment_clip(result->manifold.v[0], &pl, &s);
					vec3_copy(result->manifold.v[1], s.p1);
				}
				else
				{
					vec3_copy(result->manifold.v[0], s.p0);
					vec3_copy(result->manifold.v[1], s.p1);
				}
				
				vec3_translate_scaled(result->manifold.v[0], result->manifold.n, -plane_point_signed_distance(&pl, result->manifold.v[0]));
				vec3_translate_scaled(result->manifold.v[1], result->manifold.n, -plane_point_signed_distance(&pl, result->manifold.v[1]));
			}
		}
		/* Shallow Penetration */
		else
		{
			//COLLISION_DEBUG_ADD_SEGMENT(segment_construct(cap_s.p0, cap_s.p1));
			//COLLISION_DEBUG_ADD_SEGMENT(segment_construct(c1, c2));

			vec3_sub(result->manifold.n, c2, c1);
			vec3_mul_constant(result->manifold.n, 1.0f / vec3_length(result->manifold.n));

			const struct dcel *h = &shape1->hull;

			/* (1) compute closest face points for end-point segement */
			vec3 s_dir, diff;
			vec3_normalize(s_dir, cap_s.dir);

			struct segment s = segment_construct(p1, p2);
			u32 fi;
			vec3 n1;
			u32 parallel = 0;
		
			/* If projected segment is not a point */
			if (vec3_dot(s.dir, s.dir) > COLLISION_POINT_DIST_SQ)
			{
				/* (2) Check if capsule is infront of some parallel plane   */
				/* find parallel face with vec3_dot(face_normal, segment_points) > 0.0f */
				struct dcel_face *f;
				for (fi = 0; fi < h->f_count; ++fi)
				{
					f = h->f + fi;
					dcel_face_normal(n1, h, fi);

					const f32 d1d1 = vec3_dot(n1, n1);
					const f32 d2d2 = vec3_dot(s_dir, s_dir);
					const f32 d1d2 = vec3_dot(n1, s_dir);
					const f32 denom = d1d1*d2d2 - d1d2*d1d2;

					/* denom = (1-cos(theta)^2) == 1.0f <=> capsule and face normal orthogonal */
					//if (d1d2*d1d2 <= COLLISION_POINT_DIST_SQ)
					if (denom >= 1.0f - COLLISION_POINT_DIST_SQ)
					{	
						mat3_vec_mul(p2, g2.rot, g2.v[0]);
						vec3_translate(p2, g2.pos);
						mat3_vec_mul(p1, g1.rot, h->v[h->e[f->first].origin]);
						vec3_translate(p1, g1.pos);
						vec3_sub(diff, p2, p1);
						
						/* is capsule infront of face? */
						if (vec3_dot(diff, n1) > 0.0f)
						{
							vec3 center;
							vec3_interpolate(center, s.p0, s.p1, 0.5f);
							vec3_translate(n1, center);
							parallel = 1;
							break;
						}
					}
				}
			}

			if (parallel)
			{
				result->manifold.v_count = 2;
				dcel_face_normal(result->manifold.n, h, fi);
				vec3_translate_scaled(c1, result->manifold.n, margin);
				vec3_translate_scaled(c2, result->manifold.n, -(shape2->capsule.radius + margin));
				result->manifold.depth[0] = vec3_dot(result->manifold.n, c1) - vec3_dot(result->manifold.n, c2);
				result->manifold.depth[1] = result->manifold.depth[0];
				struct segment s = dcel_face_clip_segment(h, g1.rot, g1.pos, fi, &cap_s);
				vec3_copy(result->manifold.v[0], s.p0);
				vec3_copy(result->manifold.v[1], s.p1);
				vec3_translate_scaled(result->manifold.v[0], result->manifold.n, -(shape2->capsule.radius + 2.0f*margin -result->manifold.depth[0]));
				vec3_translate_scaled(result->manifold.v[1], result->manifold.n, -(shape2->capsule.radius + 2.0f*margin -result->manifold.depth[1]));
			}
			else
			{
				result->manifold.v_count = 1;
				vec3_sub(result->manifold.n, c2, c1);
				vec3_mul_constant(result->manifold.n, 1.0f / vec3_length(result->manifold.n));
				vec3_translate_scaled(c1, result->manifold.n, margin);
				vec3_translate_scaled(c2, result->manifold.n, -(shape2->capsule.radius + margin));
				result->manifold.depth[0] = vec3_dot(result->manifold.n, c1) - vec3_dot(result->manifold.n, c2);
				vec3_copy(result->manifold.v[0], c1);
			}
		}
	}

	return contact_generated;
}

struct sat_face_query
{
	constvec3ptr v;
	vec3 normal;
	u32 fi;
	f32 depth;
};

struct sat_edge_query
{
	struct segment s1;
	struct segment s2;
	u32	e1;
	u32	e2;
	vec3 normal;
	f32 depth;
};

static u32 hull_contact_internal_face_contact(struct arena *mem_tmp, struct contact_manifold *cm, const vec3 cm_n, const struct dcel *ref_dcel, const vec3 n_ref, const u32 ref_face_index, constvec3ptr v_ref, const struct dcel *inc_dcel, constvec3ptr v_inc)
{
	vec3 tmp1, tmp2, n;

	/* (1) determine incident_face */
	u32 inc_fi = 0;
	f32 min_dot = 1.0f;
	for (u32 fi = 0; fi < inc_dcel->f_count; ++fi)
	{
		const u32 i0  = inc_dcel->e[inc_dcel->f[fi].first + 0].origin;
		const u32 i1  = inc_dcel->e[inc_dcel->f[fi].first + 1].origin;
		const u32 i2  = inc_dcel->e[inc_dcel->f[fi].first + 2].origin;

		vec3_sub(tmp1, v_inc[i1], v_inc[i0]);
		vec3_sub(tmp2, v_inc[i2], v_inc[i0]);
		vec3_cross(n, tmp1, tmp2);
		vec3_mul_constant(n, 1.0f / vec3_length(n));

		const f32 dot = vec3_dot(n_ref, n);
		if (dot < min_dot)
		{
			min_dot = dot;
			inc_fi = fi;
		}
	}
	
	struct dcel_face *ref_face = ref_dcel->f + ref_face_index;
	struct dcel_face *inc_face = inc_dcel->f + inc_fi;

	/* (2) Setup world polygons */
	stack_vec3 clip_stack[2];
	clip_stack[0] = stack_vec3_alloc(mem_tmp, 2*inc_face->count + ref_face->count, NOT_GROWABLE);
	clip_stack[1] = stack_vec3_alloc(mem_tmp, 2*inc_face->count + ref_face->count, NOT_GROWABLE);
	u32 cur = 0;
	vec3ptr ref_v = ArenaPush(mem_tmp, ref_face->count * sizeof(vec3));
	vec3ptr cp = ArenaPush(mem_tmp, (2*inc_face->count + ref_face->count) * sizeof(vec3));

	for (u32 i = 0; i < ref_face->count; ++i)
	{
		const u32 vi = ref_dcel->e[ref_face->first + i].origin;
		vec3_copy(ref_v[i], v_ref[vi]);
	}

	for (u32 i = 0; i < inc_face->count; ++i)
	{
		const u32 vi = inc_dcel->e[inc_face->first + i].origin;
		stack_vec3_push(clip_stack + cur, v_inc[vi]);
	}

	/* (4) clip incident_face to reference_face */
	f32 *depth = ArenaPush(mem_tmp, (inc_face->count * 2 + ref_face->count) * sizeof(f32));

	/*
	 * Sutherland-Hodgman 3D polygon clipping
	 */
	for (u32 j = 0; j < ref_face->count; ++j)
	{
		const u32 prev = cur;
		cur = 1 - cur;
		stack_vec3_flush(clip_stack + cur);

		vec3_sub(tmp1, ref_v[(j+1) % ref_face->count], ref_v[j]);
		vec3_cross(n, tmp1, n_ref);
		vec3_mul_constant(n, 1.0f / vec3_length(n));
		struct plane clip_plane = plane_construct(n, ref_v[j]);

		for (u32 i = 0; i < clip_stack[prev].next; ++i)
		{
			const struct segment clip_edge = segment_construct(clip_stack[prev].arr[i], clip_stack[prev].arr[(i+1) % clip_stack[prev].next]);
			const f32 t = plane_segment_clip_parameter(&clip_plane, &clip_edge);

			vec3 inter;
			vec3_interpolate(inter, clip_edge.p1, clip_edge.p0, t);

			if (plane_point_is_behind(&clip_plane, clip_edge.p0))
			{
				stack_vec3_push(clip_stack + cur, clip_edge.p0);
				if (0.0f < t && t < 1.0f)
				{
					stack_vec3_push(clip_stack + cur, inter);
				}
			}
			else if (plane_point_is_behind(&clip_plane, clip_edge.p1))
			{
				stack_vec3_push(clip_stack + cur, inter);
			}
		}
	}

	f32 max_depth = -F32_INFINITY;
	u32 deepest_point = 0;
	u32 cp_count = 0;
	
	for (u32 i = 0; i < clip_stack[cur].next; ++i)
	{
		vec3_copy(cp[cp_count], clip_stack[cur].arr[i]);
		vec3_sub(tmp1, cp[cp_count], ref_v[0]);
		depth[cp_count] = -vec3_dot(tmp1, n_ref);
		if (depth[cp_count] >= 0.0f)
		{
			vec3_translate_scaled(cp[cp_count], n_ref, depth[cp_count]);
			if (max_depth < depth[cp_count])
			{
				max_depth = depth[cp_count];
				deepest_point = cp_count;
			}
			cp_count += 1;
		}
	}

	for (u32 i = 0; i < cp_count; ++i)
	{
		COLLISION_DEBUG_ADD_SEGMENT(segment_construct(cp[i], cp[(i+1) % cp_count]), vec4_inline(0.8f, 0.6, 0.1f, 1.0f));
	}

	u32 is_colliding = 1;
	vec3_copy(cm->n, cm_n);
	switch (cp_count)
	{
		case 0:
		{
			is_colliding = 0;
		} break;

		case 1:
		{
			cm->v_count = 1;
			vec3_copy(cm->v[0], cp[0]);
			cm->depth[0] = depth[0];
		} break;

		case 2:
		{
			cm->v_count = 2;
			vec3_copy(cm->v[0], cp[0]);
			vec3_copy(cm->v[1], cp[1]);
			cm->depth[0] = depth[0];
			cm->depth[1] = depth[1];

		} break;

		case 3:
		{
			cm->v_count = 3;
			vec3_sub(tmp1, cp[1], cp[0]);	
			vec3_sub(tmp2, cp[2], cp[0]);	
			vec3_cross(n, tmp1, tmp2);
			if (vec3_dot(n, cm->n) >= 0.0f)
			{
				vec3_copy(cm->v[0], cp[0]);
				vec3_copy(cm->v[1], cp[1]);
				vec3_copy(cm->v[2], cp[2]);
				cm->depth[0] = depth[0];
				cm->depth[1] = depth[1];
				cm->depth[2] = depth[2];
			}
			else
			{
				vec3_copy(cm->v[0], cp[0]);
				vec3_copy(cm->v[2], cp[1]);
				vec3_copy(cm->v[1], cp[2]);
				cm->depth[0] = depth[0];
				cm->depth[2] = depth[1];
				cm->depth[1] = depth[2];
			}
		} break;

		default:
		{
			/* (1) First point is deepest point */
			cm->v_count = 4;
			vec3_copy(cm->v[0], cp[deepest_point]);
			cm->depth[0] = depth[deepest_point];

			/* (2) Third point is point furthest away from deepest point */
			f32 max_dist = 0.0f;
			u32 max_i = (deepest_point + 2) % cp_count;
			for (u32 i = 0; i < cp_count; ++i)
			{
				if (i == (deepest_point + 1) % cp_count || (i+1) % cp_count == deepest_point)
				{
					continue;
				}

				const f32 dist = vec3_distance_squared(cp[deepest_point], cp[i]);
				if (max_dist < dist)
				{
					max_dist = dist;
					max_i = i;
				}
			}
			vec3_copy(cm->v[2], cp[max_i]);
			cm->depth[2] = depth[max_i];

			/* (3, 4) Second point and forth is point that gives largest (in magnitude) 
			 * areas with the previous points on each side of the previous segment 
			 */
			u32 max_pos_i = (deepest_point + 1) % cp_count;
			u32 max_neg_i = (max_i + 1) % cp_count;
			f32 max_neg = 0.0f;
			f32 max_pos = 0.0f;

			for (u32 i = (deepest_point + 1) % cp_count; i != max_i; i = (i+1) % cp_count)
			{
				vec3_sub(tmp1, cm->v[0], cp[i]);
				vec3_sub(tmp2, cm->v[2], cp[i]);
				vec3_cross(n, tmp1, tmp2);
				const f32 d = vec3_length_squared(n);
				if (max_pos < d)
				{
					max_pos = d;
					max_pos_i = i;
				}
			}

			for (u32 i = (max_i + 1) % cp_count; i != deepest_point; i = (i+1) % cp_count)
			{
				vec3_sub(tmp1, cm->v[0], cp[i]);
				vec3_sub(tmp2, cm->v[2], cp[i]);
				vec3_cross(n, tmp1, tmp2);
				const f32 d = vec3_length_squared(n);
				if (max_neg < d)
				{
					max_neg = d;
					max_neg_i = i;
				}
			}

			ds_Assert(deepest_point != max_i);
			ds_Assert(deepest_point != max_pos_i);
			ds_Assert(deepest_point != max_neg_i);
			ds_Assert(max_i != max_pos_i);
			ds_Assert(max_i != max_neg_i);
			ds_Assert(max_pos_i != max_neg_i);
	
			vec3 dir;
			tri_ccw_direction(dir, cm->v[0], cp[max_pos_i], cm->v[2]);
			if (vec3_dot(dir, cm->n) < 0.0f)
			{
				vec3_copy(cm->v[3], cp[max_pos_i]);
				vec3_copy(cm->v[1], cp[max_neg_i]);
				cm->depth[3] = depth[max_pos_i];
				cm->depth[1] = depth[max_neg_i];
			}
			else
			{
				vec3_copy(cm->v[3], cp[max_neg_i]);
				vec3_copy(cm->v[1], cp[max_pos_i]);
				cm->depth[3] = depth[max_neg_i];
				cm->depth[1] = depth[max_pos_i];
			}

		} break;
	}

	return is_colliding;
}

static u32 hull_contact_internal_fv_separation(struct sat_face_query *query, const struct dcel *h1, constvec3ptr v1_world, const struct dcel *h2, constvec3ptr v2_world)
{
	for (u32 fi = 0; fi < h1->f_count; ++fi)
	{
		const u32 f_v0 = h1->e[h1->f[fi].first + 0].origin;
		const u32 f_v1 = h1->e[h1->f[fi].first + 1].origin;
		const u32 f_v2 = h1->e[h1->f[fi].first + 2].origin;
		const struct plane sep_plane = plane_construct_from_ccw_triangle(v1_world[f_v0], v1_world[f_v1], v1_world[f_v2]);
		f32 min_dist = FLT_MAX;
		for (u32 i = 0; i < h2->v_count; ++i)
		{
			const f32 dist = plane_point_signed_distance(&sep_plane, v2_world[i]);
			if (dist < min_dist)
			{
				min_dist = dist;
			}
		}

		if (min_dist > 0.0f) 
		{ 
			query->fi = fi;
			query->depth = min_dist;
			vec3_copy(query->normal, sep_plane.normal);
			return 1; 
		}

		if (query->depth < min_dist)
		{
			query->fi = fi;
			query->depth = min_dist;
			/* We switch the sign of the normal outside the function, if need be */
			vec3_copy(query->normal, sep_plane.normal);
		}
	}

	return 0;
}

static u32 internal_ee_is_minkowski_face(const vec3 n1_1, const vec3 n1_2, const vec3 n2_1, const vec3 n2_2, const vec3 arc_n1, const vec3 arc_n2)
{
	const f32 n1_1d = vec3_dot(n1_1, arc_n2);
	const f32 n1_2d = vec3_dot(n1_2, arc_n2);
	const f32 n2_1d = vec3_dot(n2_1, arc_n1);
	const f32 n2_2d = vec3_dot(n2_2, arc_n1);

	/*
	 * last check is the hemisphere test: arc plane normals points "to the left" of the arc 1->2. 
	 * Thus, given the fact that the two first tests passes, which tells us that the two arcs 
	 * cross each others planes, the hemisphere test finally tells us if the arcs cross each other.
	 *
	 * If n2_1 lies in the positive half-space defined by arc_n1, and we know that n2_2 lies in the
	 * negative half-space, then the two arcs cross each other iff n2_1->n2_2 CCW relative to n1_2.
	 * This holds since from the first two check and n2_1->n2_2 CCW relative to n1_2, it must hold
	 * that arc_n2*n1_1 < 0.0f. If the arc is CW to n1_2, arc_n2*n1_1 > 0.0f.
	 *
	 * Similarly, if n2_1 lies in the negative half-space, then the two arcs cross each other iff 
	 * n2_1->n2_2 CW relative to n1_2 <=> arc_n2*n1_1 > 0.0f.
	 *
	 * It follows that intersection <=> (arc_n1*n2_1 > 0 && arc_n2*n1_2 > 0) || 
	 * 				    (arc_n1*n2_1 < 0 && arc_n2*n1_2 < 0)
	 *				<=>  arc_n1*n2_1 * arc_n2*n1_2) > 0
	 *				<=>  n2_1d * n1_2d > 0
	 */
	return (n1_1d*n1_2d < 0.0f && n2_1d*n2_2d < 0.0f && n1_2d*n2_1d > 0.0f) ? 1 : 0;
}

static void hull_contact_internal_ee_check(struct sat_edge_query *query, const struct dcel *h1, constvec3ptr v1_world, const u32 e1_1, const struct dcel *h2, constvec3ptr v2_world, const u32 e2_1, const vec3 h1_world_center)
{
	vec3 n1_1, n1_2, n2_1, n2_2, e1, e2;
	const u32 e1_2 = h1->e[e1_1].twin;
	const u32 e2_2 = h2->e[e2_1].twin;

	const u32 f1_1 = h1->e[e1_1].face_ccw;
	const u32 f1_2 = h1->e[e1_2].face_ccw;
	const u32 f2_1 = h2->e[e2_1].face_ccw;
	const u32 f2_2 = h2->e[e2_2].face_ccw;
	tri_ccw_direction(n1_1, v1_world[h1->e[h1->f[f1_1].first + 0].origin],  v1_world[h1->e[h1->f[f1_1].first + 1].origin], v1_world[h1->e[h1->f[f1_1].first + 2].origin]);
	tri_ccw_direction(n1_2, v1_world[h1->e[h1->f[f1_2].first + 0].origin],  v1_world[h1->e[h1->f[f1_2].first + 1].origin], v1_world[h1->e[h1->f[f1_2].first + 2].origin]);
	tri_ccw_direction(n2_1, v2_world[h2->e[h2->f[f2_1].first + 0].origin],  v2_world[h2->e[h2->f[f2_1].first + 1].origin], v2_world[h2->e[h2->f[f2_1].first + 2].origin]);
	tri_ccw_direction(n2_2, v2_world[h2->e[h2->f[f2_2].first + 0].origin],  v2_world[h2->e[h2->f[f2_2].first + 1].origin], v2_world[h2->e[h2->f[f2_2].first + 2].origin]);

	///* we are working with minkowski difference A - B, so gauss map of B is (-B). n2_1, n2_2 cross product stays the same. */
	vec3_negative(n2_1);	
	vec3_negative(n2_2);

	const struct segment s1 = segment_construct(v1_world[h1->e[e1_1].origin], v1_world[h1->e[e1_2].origin]);
	const struct segment s2 = segment_construct(v2_world[h2->e[e2_1].origin], v2_world[h2->e[e2_2].origin]);

	/* 
	 * test if A, -B edges intersect on gauss map, only if they do, 
	 * they are a candidate for collision
	 */
	if (internal_ee_is_minkowski_face(n1_1, n1_2, n2_1, n2_2, s1.dir, s2.dir))
	{
		const f32 d1d1 = vec3_dot(s1.dir, s1.dir);
		const f32 d2d2 = vec3_dot(s2.dir, s2.dir);
		const f32 d1d2 = vec3_dot(s1.dir, s2.dir);
		/* Skip parallel edge pairs  */
		if (d1d1*d2d2 - d1d2*d1d2 > F32_EPSILON*100.0f) 
		{
			vec3_cross(e1, s1.dir, s2.dir);
			vec3_mul_constant(e1, 1.0f / vec3_length(e1));
			vec3_sub(e2, s1.p0, h1_world_center);
			/* plane normal points from A -> B */
			if (vec3_dot(e1, e2) < 0.0f)
			{
				vec3_negative(e1);
			}
			
			/* check segmente-segment distance interval signed plane distance, > 0.0f => we have found a seperating axis */
			vec3_sub(e2, s2.p0, s1.p0);
			const f32 dist = vec3_dot(e1, e2);

			if (query->depth < dist)
			{
				query->depth = dist;
				vec3_copy(query->normal, e1);
				query->s1 = s1;
				query->s2 = s2;
				query->e1 = e1_1;
				query->e2 = e2_1;
			}
		}
	}
}

/*
 * For full algorithm: see GDC talk by Dirk Gregorius - 
 * 	Physics for Game Programmers: The Separating Axis Test between Convex Polyhedra
 */
static u32 hull_contact_internal_ee_separation(struct sat_edge_query *query, const struct dcel *h1, constvec3ptr v1_world, const struct dcel *h2, constvec3ptr v2_world, const vec3 h1_world_center)
{
	for (u32 e1_1 = 0; e1_1 < h1->e_count; ++e1_1)
	{
		if (h1->e[e1_1].twin < e1_1) { continue; }

		for (u32 e2_1 = 0; e2_1 < h2->e_count; ++e2_1) 
		{
			if (h2->e[e2_1].twin < e2_1) { continue; }

			hull_contact_internal_ee_check(query, h1, v1_world, e1_1, h2, v2_world, e2_1, h1_world_center);
			if (query->depth > 0.0f)
			{
				return 1;
			}
		}
	}

	return 0;
}

void sat_edge_query_collision_result(struct contact_manifold *manifold, struct sat_cache *sat_cache, const struct sat_edge_query *query)
{
	vec3 c1, c2;
	segment_distance_sq(c1, c2, &query->s1, &query->s2);
	COLLISION_DEBUG_ADD_SEGMENT(segment_construct(c1,c2), vec4_inline(0.0f, 0.8, 0.8f, 1.0f));
	COLLISION_DEBUG_ADD_SEGMENT(query->s1, vec4_inline(0.0f, 1.0, 0.1f, 1.0f));
	COLLISION_DEBUG_ADD_SEGMENT(query->s2, vec4_inline(0.0f, 0.1, 1.0f, 1.0f));

	manifold->v_count = 1;
	manifold->depth[0] = -query->depth;
	vec3_interpolate(manifold->v[0], c1, c2, 0.5f);
	vec3_copy(manifold->n, query->normal);


	sat_cache->edge1 = query->e1;
	sat_cache->edge2 = query->e2;
	sat_cache->type = SAT_CACHE_CONTACT_EE;
	ds_Assert(1.0f - 1000.0f * F32_EPSILON < vec3_length(manifold->n));
	ds_Assert(vec3_length(manifold->n) < 1.0f + 1000.0f * F32_EPSILON);
}

/*
 * For the Algorithm, see
 * 	(Game Physics Pearls, Chapter 4)
 *	(GDC 2013 Dirk Gregorius, https://www.gdcvault.com/play/1017646/Physics-for-Game-Programmers-The)
 */
static u32 hull_contact(struct arena *tmp, struct collision_result *result, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CONVEX_HULL);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_CONVEX_HULL);

	/* 
	 * we want penetration depth d and direction normal n (b1->b2), 
	 * i.e. A - n*d just touches B,  or B + n*d just touches A.
	 */

	/*
	 * n = separation normal from A to B
	 * Plane PA = plane n*x - dA denotes the plane with normal n that just touches A, pointing towards B 
	 * Plane PB = plane (-n)*x - dB denotes the plane with normal (-n) that just touches B, pointing towards A
	 *
	 * We seek * (n,d) = sup_{s on unit-sphere}(d : (s,d)). If we find a seperating axis, no contact manifold
	 * is generated and we get an early exit, returning 0;
	 */

	//TODO: Margins??
	ArenaPushRecord(tmp);

	mat3 rot1, rot2;
	quat_to_mat3(rot1, b1->rotation);
	quat_to_mat3(rot2, b2->rotation);

	struct dcel *h1 = &((struct collision_shape *) strdb_Address(pipeline->shape_db, b1->shape_handle))->hull;
	struct dcel *h2 = &((struct collision_shape *) strdb_Address(pipeline->shape_db, b2->shape_handle))->hull;

	vec3ptr v1_world = ArenaPush(tmp, h1->v_count * sizeof(vec3));
	vec3ptr v2_world = ArenaPush(tmp, h2->v_count * sizeof(vec3));

	for (u32 i = 0; i < h1->v_count; ++i)
	{
		mat3_vec_mul(v1_world[i], rot1, h1->v[i]);
		vec3_translate(v1_world[i], b1->position);
	}

	for (u32 i = 0; i < h2->v_count; ++i)
	{
		mat3_vec_mul(v2_world[i], rot2, h2->v[i]);
		vec3_translate(v2_world[i], b2->position);
	}

	struct sat_face_query f_query[2] = { { .depth = -F32_INFINITY }, { .depth = -F32_INFINITY } };
	struct sat_edge_query e_query = { .depth = -F32_INFINITY };

	u32 colliding = 1;
	u32 calculate = 1;
	struct sat_cache *sat_cache = NULL;

	const u32 bi1 = PoolIndex(&pipeline->body_pool, b1);
	const u32 bi2 = PoolIndex(&pipeline->body_pool, b2);
	ds_AssertString(bi1 < bi2, "Having these requirements spread all over the pipeline is bad, should\
			standardize some place where we enforce this rule, if at all. Furthermore, we should\
			consider better ways of creating body pair keys");

	u32 cache_found = 1;	
	if ((sat_cache = sat_cache_lookup(&pipeline->c_db, bi1, bi2)) == NULL)
	{
		cache_found = 0;	
		sat_cache = &result->sat_cache;
	}
	else
	{
		if (sat_cache->type == SAT_CACHE_SEPARATION)
		{
			vec3 support1, support2, tmp;
			vec3_negative_to(tmp, sat_cache->separation_axis);

			vertex_support(support1, sat_cache->separation_axis, v1_world, h1->v_count);
			vertex_support(support2, tmp, v2_world, h2->v_count);

			const f32 dot1 = vec3_dot(support1, sat_cache->separation_axis);
			const f32 dot2 = vec3_dot(support2, sat_cache->separation_axis);
			const f32 separation = dot2 - dot1;
			if (separation > 0.0f)
			{
				calculate = 0;
				colliding = 0;
				sat_cache->separation = separation;
			}
		}
		else if (sat_cache->type == SAT_CACHE_CONTACT_EE)
		{
			hull_contact_internal_ee_check(&e_query, h1, v1_world, sat_cache->edge1, h2, v2_world, sat_cache->edge2, b1->position);
			if (-F32_INFINITY < e_query.depth && e_query.depth < 0.0f)
			{
				calculate = 0;
				sat_edge_query_collision_result(&result->manifold, sat_cache, &e_query);
			}
			else
			{
				colliding = 0;
				e_query.depth = -F32_INFINITY;
			}
		}
		else 
		{
			//TODO BUG to fix: when removing body's all contacts, ALSO remove any sat_cache; otherwise 
			// it may be wrongfully alised the next frame by new indices.
			//TODO Should we check that the manifold is still stable? if not, we throw it away.
			vec3 ref_n, cm_n;

			if (sat_cache->body == 0)
			{
				dcel_face_normal(cm_n, h1, sat_cache->face);
				mat3_vec_mul(ref_n, rot1, cm_n);
				colliding = hull_contact_internal_face_contact(tmp, &result->manifold, ref_n, h1, ref_n, sat_cache->face, v1_world, h2, v2_world);
			}
			else
			{
				dcel_face_normal(cm_n, h2, sat_cache->face);
				mat3_vec_mul(ref_n, rot2, cm_n);
				vec3_negative_to(cm_n, ref_n);
				colliding = hull_contact_internal_face_contact(tmp, &result->manifold, cm_n, h2, ref_n, sat_cache->face, v2_world, h1, v1_world);
			}

			calculate = !colliding;
		}
	}

	if (calculate)
	{
		if (hull_contact_internal_fv_separation(&f_query[0], h1, v1_world, h2, v2_world))
		{
			vec3_copy(sat_cache->separation_axis, f_query[0].normal);
			sat_cache->separation = f_query[0].depth;
			sat_cache->type = SAT_CACHE_SEPARATION;
			colliding = 0;
			goto sat_cleanup;
		}

		if (hull_contact_internal_fv_separation(&f_query[1], h2, v2_world, h1, v1_world))
		{
			vec3_negative_to(sat_cache->separation_axis, f_query[1].normal);
			sat_cache->separation = f_query[1].depth;
			sat_cache->type = SAT_CACHE_SEPARATION;
			colliding = 0;
			goto sat_cleanup;
		}

		if (hull_contact_internal_ee_separation(&e_query, h1, v1_world, h2, v2_world, b1->position))
		{
			vec3_copy(sat_cache->separation_axis, e_query.normal);
			sat_cache->separation = e_query.depth;
			sat_cache->type = SAT_CACHE_SEPARATION;
			colliding = 0;
			goto sat_cleanup;
		}

		colliding = 1;
		if (0.99f*f_query[0].depth >= e_query.depth || 0.99f*f_query[1].depth >= e_query.depth)
		{
			if (f_query[0].depth > f_query[1].depth)
			{
				sat_cache->body = 0;
				sat_cache->face = f_query[0].fi;
				colliding = hull_contact_internal_face_contact(tmp, &result->manifold, f_query[0].normal, h1, f_query[0].normal, f_query[0].fi, v1_world, h2, v2_world);
			}
			else
			{
				vec3 cm_n;
				sat_cache->body = 1;
				sat_cache->face = f_query[1].fi;
				vec3_negative_to(cm_n, f_query[1].normal);
				colliding = hull_contact_internal_face_contact(tmp, &result->manifold, cm_n, h2, f_query[1].normal, f_query[1].fi, v2_world, h1, v1_world);
			}

			if (colliding)
			{
				sat_cache->type = SAT_CACHE_CONTACT_FV;
			}
			else
			{
				if (sat_cache->body == 0)
				{
					vec3_copy(sat_cache->separation_axis, f_query[0].normal);
				}
				else
				{
					vec3_negative_to(sat_cache->separation_axis, f_query[1].normal);
				}
				sat_cache->separation = 0.0f;
				sat_cache->type = SAT_CACHE_SEPARATION;
			}
		}
		/* edge_contact */
		else
		{
			sat_edge_query_collision_result(&result->manifold, sat_cache, &e_query);
		}
	}

sat_cleanup:
	if (!cache_found)
	{
		sat_cache->key = KeyGenU32U32(bi1, bi2);
		result->type = COLLISION_SAT_CACHE;
		ds_Assert(result->sat_cache.type < SAT_CACHE_COUNT);
	}
	else
	{
		sat_cache->touched = 1;
		result->type = (colliding)
			? COLLISION_CONTACT
			: COLLISION_NONE;
	}
	
	ArenaPopRecord(tmp);
	return colliding;
}

static u32 tri_mesh_bvh_sphere_contact(struct arena *tmp, struct collision_result *result, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_TRI_MESH);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_SPHERE);

	struct tri_mesh_bvh *mesh_bvh = &((struct collision_shape *) strdb_Address(pipeline->shape_db, b1->shape_handle))->mesh_bvh;
	struct sphere *sph = &((struct collision_shape *) strdb_Address(pipeline->shape_db, b2->shape_handle))->sphere;

	struct AABB bbox_transform;
	vec3_sub(bbox_transform.center, b2->position, b1->position);
	vec3_set(bbox_transform.hw, sph->radius, sph->radius, sph->radius);

	ArenaPushRecord(tmp);

	const struct bvh *bvh = &mesh_bvh->bvh;
	const struct bvh_node *node = (struct bvh_node *) bvh->tree.pool.buf;
	struct memArray arr = ArenaPushAlignedAll(tmp, sizeof(struct bvh_node *), sizeof(struct bvh_node *));
	const struct bvh_node **node_stack = arr.addr;

	if (arr.len == 0)
	{
		Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
		FatalCleanupAndExit(ds_ThreadSelfTid());
	}

	u32 sc = 0;
	if (AABB_test(&bbox_transform, &node[bvh->tree.root].bbox))
	{
		node_stack[sc++] = node + bvh->tree.root;
	}

	while (sc--)
	{
		if (bt_LeafCheck(node_stack[sc]))
		{
			fprintf(stderr, "sphere hits triangle bbox\n");	
		}
		else
		{
			const struct bvh_node *left = node + node_stack[sc]->bt_left;
			const struct bvh_node *right = node + node_stack[sc]->bt_right;
			if (AABB_test(&bbox_transform, &right->bbox))
			{
				node_stack[sc++] = right;
			}

			if (AABB_test(&bbox_transform, &left->bbox))
			{
				if (sc >= arr.len)
				{
					Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
					FatalCleanupAndExit(ds_ThreadSelfTid());
				}
				node_stack[sc++] = left;
			}
		}
	}

	ArenaPopRecord(tmp);

	result->type = COLLISION_NONE;
	return 0;
}

static u32 tri_mesh_bvh_capsule_contact(struct arena *tmp, struct collision_result *result, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_TRI_MESH);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_CAPSULE);

	result->type = COLLISION_NONE;
	return 0;
}

static u32 tri_mesh_bvh_hull_contact(struct arena *tmp, struct collision_result *result, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_TRI_MESH);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_CONVEX_HULL);

	result->type = COLLISION_NONE;
	return 0;
}

/********************************** RAYCAST **********************************/

f32 _sphere_raycast_parameter(const struct physics_pipeline *pipeline, const struct rigid_body *b, const struct ray *ray)
{
	ds_Assert(b->shape_type == COLLISION_SHAPE_SPHERE);
	const struct collision_shape *shape = strdb_Address(pipeline->shape_db, b->shape_handle);
	struct sphere sph = sphere_construct(b->position, shape->sphere.radius);
	return sphere_raycast_parameter(&sph, ray);	
}

f32 capsule_raycast_parameter(const struct physics_pipeline *pipeline, const struct rigid_body *b, const struct ray *ray)
{
	ds_Assert(b->shape_type == COLLISION_SHAPE_CAPSULE);

	const struct collision_shape *shape = strdb_Address(pipeline->shape_db, b->shape_handle);
	mat3 rot;
	vec3 p0, p1;
	quat_to_mat3(rot, b->rotation);
	p0[0] = rot[1][0] * shape->capsule.half_height;	
	p0[1] = rot[1][1] * shape->capsule.half_height;	
	p0[2] = rot[1][2] * shape->capsule.half_height;	
	vec3_negative_to(p1, p0);
	vec3_translate(p0, b->position);
	vec3_translate(p1, b->position);
	struct segment s = segment_construct(p0, p1);

	const f32 r = shape->capsule.radius;
	const f32 dist_sq = ray_segment_distance_sq(p0, p1, ray, &s);
	if (dist_sq > r*r) { return F32_INFINITY; }

	struct sphere sph = sphere_construct(p1, r);
	return sphere_raycast_parameter(&sph, ray);
}

f32 hull_raycast_parameter(const struct physics_pipeline *pipeline, const struct rigid_body *b, const struct ray *ray)
{
	ds_Assert(b->shape_type == COLLISION_SHAPE_CONVEX_HULL);

	vec3 n, p;
	mat3 rot;
	quat_to_mat3(rot, b->rotation);
	const struct dcel *h = &((struct collision_shape *) strdb_Address(pipeline->shape_db, b->shape_handle))->hull;
	f32 t_best = F32_INFINITY;

	for (u32 fi = 0; fi < h->f_count; ++fi)
	{
		dcel_face_normal(p, h, fi);
		mat3_vec_mul(n, rot, p);
		vec3_translate(p, b->position);

		struct plane pl = dcel_face_plane(h, rot, b->position, fi);
		const f32 t = plane_raycast_parameter(&pl, ray);
		if (t < t_best && t >= 0.0f)
		{
			ray_point(p, ray, t);
			if (dcel_face_projected_point_test(h, rot, b->position, fi, p))
			{
				t_best = t;
			}
		}
	}

	return t_best;
}

f32 tri_mesh_bvh_raycast_parameter(const struct physics_pipeline *pipeline, const struct rigid_body *b, const struct ray *ray)
{
	//TODO should cache frame/longer lived data (obviously true for tri_mesh_bvh...)
	quat inv_quat;
	mat3 inv_rot;
	vec3 tmp;
	quat_inv(inv_quat, b->rotation);
	quat_to_mat3(inv_rot, inv_quat);

	const struct tri_mesh_bvh *mesh_bvh = &((struct collision_shape *) strdb_Address(pipeline->shape_db, b->shape_handle))->mesh_bvh;
	struct ray rotated_ray;
	vec3_sub(tmp, ray->origin, b->position);
	mat3_vec_mul(rotated_ray.origin, inv_rot, tmp);
	mat3_vec_mul(rotated_ray.dir, inv_rot, ray->dir);

	return tri_mesh_bvh_raycast((struct arena *) &pipeline->frame, mesh_bvh, &rotated_ray).f;
}

/********************************** LOOKUP TABLES FOR SHAPES **********************************/

u32 (*shape_tests[COLLISION_SHAPE_COUNT][COLLISION_SHAPE_COUNT])(const struct physics_pipeline *, const struct rigid_body *, const struct rigid_body *, const f32 margin) =
{
	{ sphere_test, 			0, 				0, 				0, },
	{ capsule_sphere_test,		capsule_test, 			0, 				0, },
	{ hull_sphere_test, 		hull_capsule_test,		hull_test,			0, },
	{ tri_mesh_bvh_sphere_test, 	tri_mesh_bvh_capsule_test, 	tri_mesh_bvh_hull_test,		0, },
};

f32 (*distance_methods[COLLISION_SHAPE_COUNT][COLLISION_SHAPE_COUNT])(vec3 c1, vec3 c2, const struct physics_pipeline *, const struct rigid_body *, const struct rigid_body *, const f32) =
{
	{ sphere_distance,	 	0,				0, 				0, },
	{ capsule_sphere_distance,	capsule_distance, 		0, 				0, },
	{ hull_sphere_distance, 	hull_capsule_distance, 		hull_distance,			0, },
	{ tri_mesh_bvh_sphere_distance,	tri_mesh_bvh_capsule_distance, 	tri_mesh_bvh_hull_distance,	0, },
};

u32 (*contact_methods[COLLISION_SHAPE_COUNT][COLLISION_SHAPE_COUNT])(struct arena *, struct collision_result *, const struct physics_pipeline *, const struct rigid_body *, const struct rigid_body *, const f32) =
{
	{ sphere_contact,	 	0, 				0,				0, },
	{ capsule_sphere_contact, 	capsule_contact,		0, 				0, },
	{ hull_sphere_contact, 	  	hull_capsule_contact,		hull_contact, 			0, },
	{ tri_mesh_bvh_sphere_contact,	tri_mesh_bvh_capsule_contact, 	tri_mesh_bvh_hull_contact,	0, },
};

f32 (*shape_raycast_parameter_methods[COLLISION_SHAPE_COUNT])(const struct physics_pipeline *, const struct rigid_body *, const struct ray *) =
{
	_sphere_raycast_parameter,
	capsule_raycast_parameter,
	hull_raycast_parameter,
	tri_mesh_bvh_raycast_parameter,
};

u32 body_body_test(const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(margin >= 0.0f);
	return (b1->shape_type >= b2->shape_type)  
		? shape_tests[b1->shape_type][b2->shape_type](pipeline, b1, b2, margin)
		: shape_tests[b2->shape_type][b1->shape_type](pipeline, b2, b1, margin);
}

f32 body_body_distance(vec3 c1, vec3 c2, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(margin >= 0.0f);
	return (b1->shape_type >= b2->shape_type)  
		? distance_methods[b1->shape_type][b2->shape_type](c1, c2, pipeline, b1, b2, margin)
		: distance_methods[b2->shape_type][b1->shape_type](c2, c1, pipeline, b2, b1, margin);
}

u32 body_body_contact_manifold(struct arena *tmp, struct collision_result *result, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin)
{
	ds_Assert(margin >= 0.0f);

	/* TODO: Cannot do as above, we must make sure that CM is in correct A->B order,  maybe push this issue up? */
	u32 collision;	
	if (b1->shape_type >= b2->shape_type)  
	{
		collision = contact_methods[b1->shape_type][b2->shape_type](tmp, result, pipeline, b1, b2, margin);
	}
	else
	{
		collision = contact_methods[b2->shape_type][b1->shape_type](tmp, result, pipeline, b2, b1, margin);
		vec3_mul_constant(result->manifold.n, -1.0f);
	}

	return collision;
}

f32 body_raycast_parameter(const struct physics_pipeline *pipeline, const struct rigid_body *b, const struct ray *ray)
{
	return shape_raycast_parameter_methods[b->shape_type](pipeline, b, ray);
}

u32 body_raycast(vec3 intersection, const struct physics_pipeline *pipeline, const struct rigid_body *b, const struct ray *ray)
{
	const f32 t = body_raycast_parameter(pipeline, b, ray);
	if (t == F32_INFINITY) return 0;

	vec3_copy(intersection, ray->origin);
	vec3_translate_scaled(intersection, ray->dir, t);
	return 1;
}
