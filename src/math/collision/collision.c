/*
==========================================================================
    Copyright (C) 2025, 2026 Axel Sandstedt 

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

#include "dynamics.h"
#include "collision.h"

DEFINE_STACK(visualSegment);

dsThreadLocal struct collisionDebug *debug;

struct visualSegment VisualSegmentConstruct(const struct segment segment, const vec4 color)
{
	struct visualSegment visual =
	{
		.segment = segment,
	};
	Vec4Copy(visual.color, color);
	return visual;
}

/********************************** Contact Manifold helpers **********************************/

void ContactManifoldDebugPrint(FILE *file, const struct contactManifold *cm)
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

/********************************** Collision Shape Mass Properties **********************************/

#define VOL	0 
#define T_X 	1
#define T_Y 	2
#define T_Z 	3
#define T_XX	4
#define T_YY	5
#define T_ZZ	6
#define T_XY	7
#define T_YZ	8
#define T_ZX	9
	    
//TODO: REPLACE using table
static u32 Comb(const u32 o, const u32 u)
{
	ds_Assert(u <= o);

	u32 v1 = 1;
	u32 v2 = 1;
	u32 rep = (u <= o-u) ? u : o-u;

	for (u32 i = 0; i < rep; ++i)
	{
		v1 *= (o-i);
		v2 *= (i+1);
	}

	ds_Assert(v1 % v2 == 0);

	return v1 / v2;
}

static f32 StaticsInternalLineIntegrals(const vec2 v0, const vec2 v1, const vec2 v2, const u32 p, const u32 q, const vec3 int_scalars)
{
       ds_Assert(p <= 4 && q <= 4);
       
       f32 sum = 0.0f;
       for (u32 i = 0; i <= p; ++i)
       {
               for (u32 j = 0; j <= q; ++j)
               {
                       sum += int_scalars[0] * Comb(p, i) * Comb(q, j) * f32_pow(v1[0], (f32) i) * f32_pow(v0[0], (f32) (p-i)) * f32_pow(v1[1], (f32) j) * f32_pow(v0[1], (f32) (q-j)) / Comb(p+q, i+j);
                       sum += int_scalars[1] * Comb(p, i) * Comb(q, j) * f32_pow(v2[0], (f32) i) * f32_pow(v1[0], (f32) (p-i)) * f32_pow(v2[1], (f32) j) * f32_pow(v1[1], (f32) (q-j)) / Comb(p+q, i+j);
                       sum += int_scalars[2] * Comb(p, i) * Comb(q, j) * f32_pow(v0[0], (f32) i) * f32_pow(v2[0], (f32) (p-i)) * f32_pow(v0[1], (f32) j) * f32_pow(v2[1], (f32) (q-j)) / Comb(p+q, i+j);
               }
       }

       return sum / (p+q+1);
}

/*
 *  alpha beta gamma CCW
 */ 
static void StaticsInternalCalculateFaceIntegrals(f32 integrals[10], const struct collisionShape *shape, const u32 fi)
{
	f32 P_1   = 0.0f;
	f32 P_a   = 0.0f;
	f32 P_aa  = 0.0f;
	f32 P_aaa = 0.0f;
	f32 P_b   = 0.0f;
	f32 P_bb  = 0.0f;
	f32 P_bbb = 0.0f;
	f32 P_ab  = 0.0f;
	f32 P_aab = 0.0f;
	f32 P_abb = 0.0f;

	vec3 n, a, b;
	vec2 v0, v1, v2;

	vec3ptr v = shape->hull.v;
	struct dcelFace *f = shape->hull.f + fi;
	struct dcelEdge *e0 = shape->hull.e + f->first;
	struct dcelEdge *e1 = shape->hull.e + f->first + 1;
	struct dcelEdge *e2 = shape->hull.e + f->first + 2;

	Vec3Sub(a, v[e1->origin], v[e0->origin]);
	Vec3Sub(b, v[e2->origin], v[e0->origin]);
	Vec3Cross(n, a, b);
	Vec3ScaleSelf(n, 1.0f / Vec3Length(n));
	const f32 d = -Vec3Dot(n, v[e0->origin]);

	u32 max_index = 0;
	if (n[max_index]*n[max_index] < n[1]*n[1]) { max_index = 1; }
	if (n[max_index]*n[max_index] < n[2]*n[2]) { max_index = 2; }

	/* maxized normal direction determines projected surface integral axes (we maximse the projected surface area) */
	
	const u32 a_i = (1+max_index) % 3;
	const u32 b_i = (2+max_index) % 3;
	const u32 y_i = max_index % 3;

	//Vec3Set(n, n[a_i], n[b_i], n[y_i]);

	/* TODO: REPLACE */
	union { f32 f; u32 bits; } val = { .f = n[y_i] };
	const f32 n_sign = (val.bits >> 31) ? -1.0f : 1.0f;

	const u32 tri_count = f->count - 2;
	for (u32 i = 0; i < tri_count; ++i)
	{
		e0 = shape->hull.e + f->first;
		e1 = shape->hull.e + f->first + 1 + i;
		e2 = shape->hull.e + f->first + 2 + i;

		Vec2Set(v0, v[e0->origin][a_i], v[e0->origin][b_i]);
		Vec2Set(v1, v[e1->origin][a_i], v[e1->origin][b_i]);
		Vec2Set(v2, v[e2->origin][a_i], v[e2->origin][b_i]);
		
		const vec3 delta_a =
		{
			v1[0] - v0[0],
			v2[0] - v1[0],
			v0[0] - v2[0],
		};
		
		const vec3 delta_b = 
		{
			v1[1] - v0[1],
			v2[1] - v1[1],
			v0[1] - v2[1],
		};

		/* simplify cross product of v1-v0, v2-v0 to get this */
		P_1   += ((v0[0] + v1[0])*delta_b[0] + (v1[0] + v2[0])*delta_b[1] + (v0[0] + v2[0])*delta_b[2]) / 2.0f;
		P_a   +=  StaticsInternalLineIntegrals(v0, v1, v2, 2, 0, delta_b);
		P_aa  +=  StaticsInternalLineIntegrals(v0, v1, v2, 3, 0, delta_b);
		P_aaa +=  StaticsInternalLineIntegrals(v0, v1, v2, 4, 0, delta_b);
		P_b   += -StaticsInternalLineIntegrals(v0, v1, v2, 0, 2, delta_a);
		P_bb  += -StaticsInternalLineIntegrals(v0, v1, v2, 0, 3, delta_a);
		P_bbb += -StaticsInternalLineIntegrals(v0, v1, v2, 0, 4, delta_a);
		P_ab  +=  StaticsInternalLineIntegrals(v0, v1, v2, 2, 1, delta_b);
		P_aab +=  StaticsInternalLineIntegrals(v0, v1, v2, 3, 1, delta_b);
		P_abb +=  StaticsInternalLineIntegrals(v0, v1, v2, 1, 3, delta_b);
	}

	P_1   *= n_sign;
	P_a   *= (n_sign / 2.0f); 
	P_aa  *= (n_sign / 3.0f); 
	P_aaa *= (n_sign / 4.0f); 
	P_b   *= (n_sign / 2.0f); 
	P_bb  *= (n_sign / 3.0f); 
	P_bbb *= (n_sign / 4.0f); 
	P_ab  *= (n_sign / 2.0f); 
	P_aab *= (n_sign / 3.0f); 
	P_abb *= (n_sign / 3.0f); 

	const f32 a_y_div = n_sign / n[y_i];
	const f32 n_y_div = 1.0f / n[y_i];

	/* surface integrals */
	const f32 S_a 	= a_y_div * P_a;
	const f32 S_aa 	= a_y_div * P_aa;
	const f32 S_aaa = a_y_div * P_aaa;
	const f32 S_aab = a_y_div * P_aab;
	const f32 S_b 	= a_y_div * P_b;
	const f32 S_bb 	= a_y_div * P_bb;
	const f32 S_bbb = a_y_div * P_bbb;
	const f32 S_bby = -a_y_div * n_y_div * (n[a_i]*P_abb + n[b_i]*P_bbb + d*P_bb);
	const f32 S_y 	= -a_y_div * n_y_div * (n[a_i]*P_a + n[b_i]*P_b + d*P_1);
	const f32 S_yy 	= a_y_div * n_y_div * n_y_div * (n[a_i]*n[a_i]*P_aa + 2.0f*n[a_i]*n[b_i]*P_ab + n[b_i]*n[b_i]*P_bb 
			+ 2.0f*d*n[a_i]*P_a + 2.0f*d*n[b_i]*P_b + d*d*P_1);	
	const f32 S_yyy = -a_y_div * n_y_div * n_y_div * n_y_div * (n[a_i]*n[a_i]*n[a_i]*P_aaa + 3.0f*n[a_i]*n[a_i]*n[b_i]*P_aab
			+ 3.0f*n[a_i]*n[b_i]*n[b_i]*P_abb + n[b_i]*n[b_i]*n[b_i]*P_bbb + 3.0f*d*n[a_i]*n[a_i]*P_aa 
			+ 6.0f*d*n[a_i]*n[b_i]*P_ab + 3.0f*d*n[b_i]*n[b_i]*P_bb + 3.0f*d*d*n[a_i]*P_a
		       	+ 3.0f*d*d*n[b_i]*P_b + d*d*d*P_1);
	const f32 S_yya = a_y_div * n_y_div * n_y_div * (n[a_i]*n[a_i]*P_aaa + 2.0f*n[a_i]*n[b_i]*P_aab + n[b_i]*n[b_i]*P_abb 
			+ 2.0f*d*n[a_i]*P_aa + 2.0f*d*n[b_i]*P_ab + d*d*P_a);	

	if (max_index == 2)
	{
		integrals[VOL] += S_a * n[0];
	}
	else if (max_index == 1)
	{
		integrals[VOL] += S_b * n[0];
	}
	else
	{
		integrals[VOL] += S_y * n[0];
	}

	integrals[T_X + a_i] += S_aa * n[a_i] / 2.0f;
	integrals[T_X + b_i] += S_bb * n[b_i] / 2.0f;
	integrals[T_X + y_i] += S_yy * n[y_i] / 2.0f;

	integrals[T_XX + a_i] += S_aaa * n[a_i] / 3.0f;
	integrals[T_XX + b_i] += S_bbb * n[b_i] / 3.0f;
	integrals[T_XX + y_i] += S_yyy * n[y_i] / 3.0f;

	integrals[T_XY + a_i] += S_aab * n[a_i] / 2.0f;
	integrals[T_XY + b_i] += S_bby * n[b_i] / 2.0f;
	integrals[T_XY + y_i] += S_yya * n[y_i] / 2.0f;
}

void CollisionShapeUpdateMassProperties(struct collisionShape *shape)
{
	ds_Assert(shape->type != COLLISION_SHAPE_TRI_MESH);

	f32 I_xx = 0.0f;
	f32 I_yy = 0.0f;
	f32 I_zz = 0.0f;
	f32 I_xy = 0.0f;
	f32 I_xz = 0.0f;
	f32 I_yz = 0.0f;
	vec3 com = VEC3_ZERO;

	if (shape->type == COLLISION_SHAPE_CONVEX_HULL)
	{
		f32 integrals[10] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }; 
		for (u32 fi = 0; fi < shape->hull.f_count; ++fi)
		{
			StaticsInternalCalculateFaceIntegrals(integrals, shape, fi);
		}

		//		fprintf(stderr, "c_hull Volume integrals: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
		//				integrals[VOL ],
		//				integrals[T_X ],
		//				integrals[T_Y ],
		//				integrals[T_Z ],
		//				integrals[T_XX],
		//				integrals[T_YY],
		//				integrals[T_ZZ],
		//      	                integrals[T_XY],
		//      	                integrals[T_YZ],
		//      	                integrals[T_ZX]);

		shape->volume = integrals[VOL];
		ds_Assert(shape->volume > 0.0f);

		/* center of mass */
		Vec3Set(shape->center_of_mass,
			integrals[T_X] / shape->volume,
		       	integrals[T_Y] / shape->volume,
		       	integrals[T_Z] / shape->volume
		);
		vec3 com;
		Vec3Copy(com, shape->center_of_mass);


		I_xx = integrals[T_YY] + integrals[T_ZZ] - shape->volume*(com[1]*com[1] + com[2]*com[2]);
		I_yy = integrals[T_XX] + integrals[T_ZZ] - shape->volume*(com[0]*com[0] + com[2]*com[2]);
		I_zz = integrals[T_XX] + integrals[T_YY] - shape->volume*(com[0]*com[0] + com[1]*com[1]);
		I_xy = integrals[T_XY] - shape->volume*com[0]*com[1];
		I_xz = integrals[T_ZX] - shape->volume*com[0]*com[2];
		I_yz = integrals[T_YZ] - shape->volume*com[1]*com[2];
		Mat3Set(shape->inertia_tensor, I_xx, -I_xy, -I_xz,
			       		 	 -I_xy,  I_yy, -I_yz,
						 -I_xz, -I_yz, I_zz);
	}
	else if (shape->type == COLLISION_SHAPE_SPHERE)
	{
		Vec3Set(shape->center_of_mass, 0.0f, 0.0f, 0.0f);
		const f32 r = shape->sphere.radius;
		const f32 rr = r*r;
		const f32 rrr = rr*r;
		shape->volume =  4.0f * F32_PI * rrr / 3.0f;
		I_xx = 2.0f * shape->volume * rr / 5.0f;
		I_yy = I_xx;
		I_zz = I_xx;
		I_xy = 0.0f;
		I_yz = 0.0f;
		I_xz = 0.0f;

		Mat3Set(shape->inertia_tensor, I_xx, -I_xy, -I_xz,
			       		 	 -I_xy,  I_yy, -I_yz,
						 -I_xz, -I_yz, I_zz);
	}
	else if (shape->type == COLLISION_SHAPE_CAPSULE)
	{
		Vec3Set(shape->center_of_mass, 0.0f, 0.0f, 0.0f);
		const f32 r = shape->capsule.radius;
		const f32 h = shape->capsule.half_height;
		const f32 hpr = h+r;
		const f32 hmr = h-r;

		shape->volume = 4.0f * F32_PI * r*r*r / 3.0f + 2.0f * h * F32_PI * r*r;

		const f32 I_xx_cap_up = (4.0f * F32_PI * r*r * h*h*h + 3.0f * F32_PI * r*r*r*r * h) / 6.0f;
		const f32 I_xx_sph_up = 2.0f * F32_PI * r*r * (hpr*hpr*hpr - hmr*hmr*hmr) / 3.0f + F32_PI * r*r*r*r*r;
		const f32 I_xx_up = I_xx_sph_up + I_xx_cap_up;
		const f32 I_zz_up = I_xx_up;

		const f32 I_yy_cap_up = F32_PI * r*r*r*r * h;
		const f32 I_yy_sph_up = 2.0f * F32_PI * r*r*r*r*r;
		const f32 I_yy_up = I_yy_cap_up + I_yy_sph_up;

		const f32 I_xy_up = 0;
		const f32 I_yz_up = 0;
		const f32 I_xz_up = 0;

		/* Derive */
		Mat3Set(shape->inertia_tensor, I_xx_up, -I_xy_up, -I_xz_up,
			       		 	 -I_xy_up,  I_yy_up, -I_yz_up,
						 -I_xz_up, -I_yz_up,  I_zz_up);
	}
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

static struct simplex GjkInternalSimplexInit(void)
{
	struct simplex simplex = 
	{
		.id = {UINT64_MAX, UINT64_MAX, UINT64_MAX, UINT64_MAX},
		.dot = { -1.0f, -1.0f, -1.0f, -1.0f },
		.type = UINT32_MAX,
	};

	return simplex;
}

static u32 GjkInternalJohnsonsAlgorithm(struct simplex *simplex, vec3 c_v, vec4 lambda)
{
	vec3 a;

	if (simplex->type == 0)
	{
		Vec3Copy(c_v, simplex->p[0]);
	}
	else if (simplex->type == 1)
	{
		Vec3Sub(a, simplex->p[0], simplex->p[1]);
		const f32 delta_01_1 = Vec3Dot(a, simplex->p[0]);

		if (delta_01_1 > 0.0f)
		{
			Vec3Sub(a, simplex->p[1], simplex->p[0]);
			const f32 delta_01_0 = Vec3Dot(a, simplex->p[1]);
			if (delta_01_0 > 0.0f)
			{
				const f32 delta = delta_01_0 + delta_01_1;
				lambda[0] = delta_01_0 / delta;
				lambda[1] = delta_01_1 / delta;
				Vec3Set(c_v,
				       	(lambda[0]*(simplex->p[0])[0] + lambda[1]*(simplex->p[1])[0]),
				       	(lambda[0]*(simplex->p[0])[1] + lambda[1]*(simplex->p[1])[1]),
				       	(lambda[0]*(simplex->p[0])[2] + lambda[1]*(simplex->p[1])[2]));
			}
			else
			{
				simplex->type = 0;
				Vec3Copy(c_v, simplex->p[1]);
				Vec3Copy(simplex->p[0], simplex->p[1]);
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
		Vec3Sub(a, simplex->p[1], simplex->p[0]);
		const f32 delta_01_0 = Vec3Dot(a, simplex->p[1]);
		Vec3Sub(a, simplex->p[0], simplex->p[1]);
		const f32 delta_01_1 = Vec3Dot(a, simplex->p[0]);
		Vec3Sub(a, simplex->p[0], simplex->p[2]);
		const f32 delta_012_2 = delta_01_0 * Vec3Dot(a, simplex->p[0]) + delta_01_1 * Vec3Dot(a, simplex->p[1]);
		if (delta_012_2 > 0.0f)
		{
			Vec3Sub(a, simplex->p[2], simplex->p[0]);
			const f32 delta_02_0 = Vec3Dot(a, simplex->p[2]);
			Vec3Sub(a, simplex->p[0], simplex->p[2]);
			const f32 delta_02_2 = Vec3Dot(a, simplex->p[0]);
			Vec3Sub(a, simplex->p[0], simplex->p[1]);
			const f32 delta_012_1 = delta_02_0 * Vec3Dot(a, simplex->p[0]) + delta_02_2 * Vec3Dot(a, simplex->p[2]);
			if (delta_012_1 > 0.0f)
			{
				Vec3Sub(a, simplex->p[2], simplex->p[1]);
				const f32 delta_12_1 = Vec3Dot(a, simplex->p[2]);
				Vec3Sub(a, simplex->p[1], simplex->p[2]);
				const f32 delta_12_2 = Vec3Dot(a, simplex->p[1]);
				Vec3Sub(a, simplex->p[1], simplex->p[0]);
				const f32 delta_012_0 = delta_12_1 * Vec3Dot(a, simplex->p[1]) + delta_12_2 * Vec3Dot(a, simplex->p[2]);
				if (delta_012_0 > 0.0f)
				{
					const f32 delta = delta_012_0 + delta_012_1 + delta_012_2;
					lambda[0] = delta_012_0 / delta;
					lambda[1] = delta_012_1 / delta;
					lambda[2] = delta_012_2 / delta;
					Vec3Set(c_v,
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
							Vec3Set(c_v,
							       	(lambda[0]*(simplex->p[1])[0] + lambda[1]*(simplex->p[2])[0]),
							       	(lambda[0]*(simplex->p[1])[1] + lambda[1]*(simplex->p[2])[1]),
							       	(lambda[0]*(simplex->p[1])[2] + lambda[1]*(simplex->p[2])[2]));
							simplex->type = 1;
							Vec3Copy(simplex->p[0], simplex->p[1]);
							Vec3Copy(simplex->p[1], simplex->p[2]);
							simplex->id[0] = simplex->id[1];
							simplex->dot[0] = simplex->dot[1];
						}
						else
						{
							simplex->type = 0;
							Vec3Copy(c_v, simplex->p[2]);
							Vec3Copy(simplex->p[0], simplex->p[2]);
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
						Vec3Set(c_v,
						       	(lambda[0]*(simplex->p[0])[0] + lambda[1]*(simplex->p[2])[0]),
						       	(lambda[0]*(simplex->p[0])[1] + lambda[1]*(simplex->p[2])[1]),
						       	(lambda[0]*(simplex->p[0])[2] + lambda[1]*(simplex->p[2])[2]));
						simplex->type = 1;
						Vec3Copy(simplex->p[1], simplex->p[2]);
					}
					else
					{
						simplex->type = 0;
						Vec3Copy(c_v, simplex->p[2]);
						Vec3Copy(simplex->p[0], simplex->p[2]);
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
		Vec3Sub(a, simplex->p[1], simplex->p[0]);
		const f32 delta_01_0 = Vec3Dot(a, simplex->p[1]);
		Vec3Sub(a, simplex->p[0], simplex->p[1]);
		const f32 delta_01_1 = Vec3Dot(a, simplex->p[0]);
		Vec3Sub(a, simplex->p[0], simplex->p[2]);
		const f32 delta_012_2 = delta_01_0 * Vec3Dot(a, simplex->p[0]) + delta_01_1 * Vec3Dot(a, simplex->p[1]);

		Vec3Sub(a, simplex->p[2], simplex->p[0]);
		const f32 delta_02_0 = Vec3Dot(a, simplex->p[2]);
		Vec3Sub(a, simplex->p[0], simplex->p[2]);
		const f32 delta_02_2 = Vec3Dot(a, simplex->p[0]);
		Vec3Sub(a, simplex->p[0], simplex->p[1]);
		const f32 delta_012_1 = delta_02_0 * Vec3Dot(a, simplex->p[0]) + delta_02_2 * Vec3Dot(a, simplex->p[2]);

		Vec3Sub(a, simplex->p[2], simplex->p[1]);
		const f32 delta_12_1 = Vec3Dot(a, simplex->p[2]);
		Vec3Sub(a, simplex->p[1], simplex->p[2]);
		const f32 delta_12_2 = Vec3Dot(a, simplex->p[1]);
		Vec3Sub(a, simplex->p[1], simplex->p[0]);
		const f32 delta_012_0 = delta_12_1 * Vec3Dot(a, simplex->p[1]) + delta_12_2 * Vec3Dot(a, simplex->p[2]);

		Vec3Sub(a, simplex->p[0], simplex->p[3]);
		const f32 delta_0123_3 = delta_012_0 * Vec3Dot(a, simplex->p[0]) + delta_012_1 * Vec3Dot(a, simplex->p[1]) + delta_012_2 * Vec3Dot(a, simplex->p[2]);

		if (delta_0123_3 > 0.0f)
		{
			Vec3Sub(a, simplex->p[0], simplex->p[3]);
			const f32 delta_013_3 = delta_01_0 * Vec3Dot(a, simplex->p[0]) + delta_01_1 * Vec3Dot(a, simplex->p[1]);

			Vec3Sub(a, simplex->p[3], simplex->p[0]);
			const f32 delta_03_0 = Vec3Dot(a, simplex->p[3]);
			Vec3Sub(a, simplex->p[0], simplex->p[3]);
			const f32 delta_03_3 = Vec3Dot(a, simplex->p[0]);
			Vec3Sub(a, simplex->p[0], simplex->p[1]);
			const f32 delta_013_1 = delta_03_0 * Vec3Dot(a, simplex->p[0]) + delta_03_3 * Vec3Dot(a, simplex->p[3]);

			Vec3Sub(a, simplex->p[3], simplex->p[1]);
			const f32 delta_13_1 = Vec3Dot(a, simplex->p[3]);
			Vec3Sub(a, simplex->p[1], simplex->p[3]);
			const f32 delta_13_3 = Vec3Dot(a, simplex->p[1]);
			Vec3Sub(a, simplex->p[1], simplex->p[0]);
			const f32 delta_013_0 = delta_13_1 * Vec3Dot(a, simplex->p[1]) + delta_13_3 * Vec3Dot(a, simplex->p[3]);

			Vec3Sub(a, simplex->p[0], simplex->p[2]);
			const f32 delta_0123_2 = delta_013_0 * Vec3Dot(a, simplex->p[0]) + delta_013_1 * Vec3Dot(a, simplex->p[1]) + delta_013_3 * Vec3Dot(a, simplex->p[3]);

			if (delta_0123_2 > 0.0f)
			{
				Vec3Sub(a, simplex->p[0], simplex->p[3]);
				const f32 delta_023_3 = delta_02_0 * Vec3Dot(a, simplex->p[0]) + delta_02_2 * Vec3Dot(a, simplex->p[2]);

				Vec3Sub(a, simplex->p[0], simplex->p[2]);
				const f32 delta_023_2 = delta_03_0 * Vec3Dot(a, simplex->p[0]) + delta_03_3 * Vec3Dot(a, simplex->p[3]);

				Vec3Sub(a, simplex->p[3], simplex->p[2]);
				const f32 delta_23_2 = Vec3Dot(a, simplex->p[3]);
				Vec3Sub(a, simplex->p[2], simplex->p[3]);
				const f32 delta_23_3 = Vec3Dot(a, simplex->p[2]);
				Vec3Sub(a, simplex->p[2], simplex->p[0]);
				const f32 delta_023_0 = delta_23_2 * Vec3Dot(a, simplex->p[2]) + delta_23_3 * Vec3Dot(a, simplex->p[3]);

				Vec3Sub(a, simplex->p[0], simplex->p[1]);
				const f32 delta_0123_1 = delta_023_0 * Vec3Dot(a, simplex->p[0]) + delta_023_2 * Vec3Dot(a, simplex->p[2]) + delta_023_3 * Vec3Dot(a, simplex->p[3]);

				if (delta_0123_1 > 0.0f)
				{
					Vec3Sub(a, simplex->p[3], simplex->p[1]);
					const f32 delta_123_1 = delta_23_2 * Vec3Dot(a, simplex->p[2]) + delta_23_3 * Vec3Dot(a, simplex->p[3]);

					Vec3Sub(a, simplex->p[3], simplex->p[2]);
					const f32 delta_123_2 = delta_13_1 * Vec3Dot(a, simplex->p[1]) + delta_13_3 * Vec3Dot(a, simplex->p[3]);

					Vec3Sub(a, simplex->p[1], simplex->p[3]);
					const f32 delta_123_3 = delta_12_1 * Vec3Dot(a, simplex->p[1]) + delta_12_2 * Vec3Dot(a, simplex->p[2]);

					Vec3Sub(a, simplex->p[3], simplex->p[0]);
					const f32 delta_0123_0 = delta_123_1 * Vec3Dot(a, simplex->p[1]) + delta_123_2 * Vec3Dot(a, simplex->p[2]) + delta_123_3 * Vec3Dot(a, simplex->p[3]);

					if (delta_0123_0 > 0.0f)
					{
						/* intersection */
						const f32 delta = delta_0123_0 + delta_0123_1 + delta_0123_2 + delta_0123_3;
						lambda[0] = delta_0123_0 / delta;
						lambda[1] = delta_0123_1 / delta;
						lambda[2] = delta_0123_2 / delta;
						lambda[3] = delta_0123_3 / delta;
						Vec3Set(c_v,
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
									Vec3Set(c_v,
										(lambda[0]*(simplex->p[1])[0] + lambda[1]*(simplex->p[2])[0] + lambda[2]*(simplex->p[3])[0]),
										(lambda[0]*(simplex->p[1])[1] + lambda[1]*(simplex->p[2])[1] + lambda[2]*(simplex->p[3])[1]),
										(lambda[0]*(simplex->p[1])[2] + lambda[1]*(simplex->p[2])[2] + lambda[2]*(simplex->p[3])[2]));
									simplex->type = 2;
									Vec3Copy(simplex->p[0], simplex->p[1]);		
									Vec3Copy(simplex->p[1], simplex->p[2]);		
									Vec3Copy(simplex->p[2], simplex->p[3]);		
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
											Vec3Set(c_v,
												(lambda[0]*(simplex->p[2])[0] + lambda[1]*(simplex->p[3])[0]),
												(lambda[0]*(simplex->p[2])[1] + lambda[1]*(simplex->p[3])[1]),
												(lambda[0]*(simplex->p[2])[2] + lambda[1]*(simplex->p[3])[2]));
											simplex->type = 1;
											Vec3Copy(simplex->p[0], simplex->p[2]);		
											Vec3Copy(simplex->p[1], simplex->p[3]);		
											simplex->dot[0] = simplex->dot[2];
											simplex->dot[2] = -1.0f;
											simplex->id[0] = simplex->id[2];
											simplex->id[2] = UINT32_MAX;
										}
										else
										{
											Vec3Copy(c_v, simplex->p[3]);
											simplex->type = 0;
											Vec3Copy(simplex->p[0], simplex->p[3]);
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
										Vec3Set(c_v,
											(lambda[0]*(simplex->p[1])[0] + lambda[1]*(simplex->p[3])[0]),
											(lambda[0]*(simplex->p[1])[1] + lambda[1]*(simplex->p[3])[1]),
											(lambda[0]*(simplex->p[1])[2] + lambda[1]*(simplex->p[3])[2]));
										simplex->type = 1;
										Vec3Copy(simplex->p[0], simplex->p[1]);
										Vec3Copy(simplex->p[1], simplex->p[3]);		
										simplex->dot[0] = simplex->dot[1];
										simplex->dot[2] = -1.0f;
										simplex->id[0] = simplex->id[1];
										simplex->id[2] = UINT32_MAX;
									}
									else
									{
										Vec3Copy(c_v, simplex->p[3]);
										simplex->type = 0;
										Vec3Copy(simplex->p[0], simplex->p[3]);
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
								Vec3Set(c_v,
									(lambda[0]*(simplex->p[0])[0] + lambda[1]*(simplex->p[2])[0] + lambda[2]*(simplex->p[3])[0]),
									(lambda[0]*(simplex->p[0])[1] + lambda[1]*(simplex->p[2])[1] + lambda[2]*(simplex->p[3])[1]),
									(lambda[0]*(simplex->p[0])[2] + lambda[1]*(simplex->p[2])[2] + lambda[2]*(simplex->p[3])[2]));
								simplex->type = 2;
								Vec3Copy(simplex->p[1], simplex->p[2]);		
								Vec3Copy(simplex->p[2], simplex->p[3]);		
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
										Vec3Set(c_v,
											(lambda[0]*(simplex->p[2])[0] + lambda[1]*(simplex->p[3])[0]),
											(lambda[0]*(simplex->p[2])[1] + lambda[1]*(simplex->p[3])[1]),
											(lambda[0]*(simplex->p[2])[2] + lambda[1]*(simplex->p[3])[2]));
										simplex->type = 1;
										Vec3Copy(simplex->p[0], simplex->p[2]);
										Vec3Copy(simplex->p[1], simplex->p[3]);
										simplex->dot[0] = simplex->dot[2];
										simplex->dot[2] = -1.0f;
										simplex->id[0] = simplex->id[2];
										simplex->id[2] = UINT32_MAX;
									}
									else
									{
										Vec3Copy(c_v, simplex->p[3]);
										simplex->type = 0;
										Vec3Copy(simplex->p[0], simplex->p[3]);
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
									Vec3Set(c_v,
										(lambda[0]*(simplex->p[0])[0] + lambda[1]*(simplex->p[3])[0]),
										(lambda[0]*(simplex->p[0])[1] + lambda[1]*(simplex->p[3])[1]),
										(lambda[0]*(simplex->p[0])[2] + lambda[1]*(simplex->p[3])[2]));
									simplex->type = 1;
									Vec3Copy(simplex->p[1], simplex->p[3]);
									simplex->dot[2] = -1.0f;
									simplex->id[2] = UINT32_MAX;
								}
								else
								{
									Vec3Copy(c_v, simplex->p[3]);
									simplex->type = 0;
									Vec3Copy(simplex->p[0], simplex->p[3]);
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
							Vec3Set(c_v,
								(lambda[0]*(simplex->p[0])[0] + lambda[1]*(simplex->p[1])[0] + lambda[2]*(simplex->p[3])[0]),
								(lambda[0]*(simplex->p[0])[1] + lambda[1]*(simplex->p[1])[1] + lambda[2]*(simplex->p[3])[1]),
								(lambda[0]*(simplex->p[0])[2] + lambda[1]*(simplex->p[1])[2] + lambda[2]*(simplex->p[3])[2]));
							simplex->type = 2;
							Vec3Copy(simplex->p[2], simplex->p[3]);
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
									Vec3Set(c_v,
										(lambda[0]*(simplex->p[1])[0] + lambda[1]*(simplex->p[3])[0]),
										(lambda[0]*(simplex->p[1])[1] + lambda[1]*(simplex->p[3])[1]),
										(lambda[0]*(simplex->p[1])[2] + lambda[1]*(simplex->p[3])[2]));
									simplex->type = 1;
									Vec3Copy(simplex->p[0], simplex->p[1]);
									Vec3Copy(simplex->p[1], simplex->p[3]);
									simplex->dot[2] = -1.0f;
									simplex->id[2] = UINT32_MAX;
								}
								else
								{
									Vec3Copy(c_v, simplex->p[3]);
									simplex->type = 0;
									Vec3Copy(simplex->p[0], simplex->p[3]);
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
								Vec3Set(c_v,
									(lambda[0]*(simplex->p[0])[0] + lambda[1]*(simplex->p[3])[0]),
									(lambda[0]*(simplex->p[0])[1] + lambda[1]*(simplex->p[3])[1]),
									(lambda[0]*(simplex->p[0])[2] + lambda[1]*(simplex->p[3])[2]));
								simplex->type = 1;
								Vec3Copy(simplex->p[1], simplex->p[3]);
								simplex->dot[2] = -1.0f;
								simplex->id[2] = UINT32_MAX;
							}
							else
							{
								Vec3Copy(c_v, simplex->p[3]);
								simplex->type = 0;
								Vec3Copy(simplex->p[0], simplex->p[3]);
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

struct gjkInput
{
	vec3ptr v;
	vec3 pos;
	mat3 rot;
	u32 v_count;
};

static void GjkInternalClosestPoints(vec3 c1, vec3 c2, struct gjkInput *in1, struct simplex *simplex, const vec4 lambda)
{
	if (simplex->type == 0)
	{
		Mat3VecMul(c1, in1->rot, in1->v[simplex->id[0] >> 32]);
		Vec3Translate(c1, in1->pos);
		Vec3Sub(c2, c1, simplex->p[0]);
	}
	else
	{
		vec3 tmp1, tmp2;
		Vec3Set(c1, 0.0f, 0.0f, 0.0f);
		Vec3Set(c2, 0.0f, 0.0f, 0.0f);
		for (u32 i = 0; i <= simplex->type; ++i)
		{
			Mat3VecMul(tmp1, in1->rot, in1->v[simplex->id[i] >> 32]);
			Vec3Translate(tmp1, in1->pos);
			Vec3Sub(tmp2, tmp1, simplex->p[i]);
			Vec3TranslateScaled(c1, tmp1, lambda[i]);
			Vec3TranslateScaled(c2, tmp2, lambda[i]);
		}
	}

}	

static u32 GjkInternalSupport(vec3 support, const vec3 dir, struct gjkInput *in)
{
	f32 max = -F32_INFINITY;
	u32 max_index = 0;
	vec3 p;
	for (u32 i = 0; i < in->v_count; ++i)
	{
		Mat3VecMul(p, in->rot, in->v[i]);
		const f32 dot = Vec3Dot(p, dir);
		if (max < dot)
		{
			max_index = i;
			max = dot; 
		}
	}

	Mat3VecMul(support, in->rot, in->v[max_index]);
	Vec3Translate(support,in->pos);
	return max_index;

}

static f32 GjkDistanceSquared(vec3 c1, vec3 c2, struct gjkInput *in1, struct gjkInput *in2)
{
	ds_Assert(in1->v_count > 0);
	ds_Assert(in2->v_count > 0);
	
	const f32 abs_tol = 100.0f * F32_EPSILON;
	const f32 tol = 100.0f * F32_EPSILON;

	struct simplex simplex = GjkInternalSimplexInit();
	vec3 dir, c_v, tmp, s1, s2;
	vec4 lambda;
	u64 support_id;
	f32 ma; /* max dot product of current simplex */
	f32 dist_sq = F32_MAX_POSITIVE_NORMAL; 
	const f32 rel = tol * tol;

	/* arbitrary starting search direction */
	Vec3Set(c_v, 1.0f, 0.0f, 0.0f);
	u64 old_support = UINT64_MAX;

	//TODO
	const u32 max_iter = 128;
	for (u32 i = 0; i < max_iter; ++i)
	{
		simplex.type += 1;
		Vec3Scale(dir, c_v, -1.0f);

		const u32 i1 = GjkInternalSupport(s1, dir, in1);
		Vec3Negate(tmp, dir);
		const u32 i2 = GjkInternalSupport(s2, tmp, in2);
		Vec3Sub(simplex.p[simplex.type], s1, s2);
		support_id = ((u64) i1 << 32) | (u64) i2;

		if (dist_sq - Vec3Dot(simplex.p[simplex.type], c_v) <= rel * dist_sq + abs_tol
				|| simplex.id[0] == support_id || simplex.id[1] == support_id 
				|| simplex.id[2] == support_id || simplex.id[3] == support_id)
		{
			ds_Assert(dist_sq != F32_INFINITY);
			simplex.type -= 1;
			GjkInternalClosestPoints(c1, c2, in1, &simplex, lambda);
			return dist_sq;
		}

		/* find closest point v to origin using naive Johnson's algorithm, update simplex data 
		 * Degenerate Case: due to numerical issues, determinant signs may flip, which may result
		 * either in wrong sub-simplex being chosen, or no valid simplex at all. In that case c_v
		 * stays the same, and we terminate the algorithm. [See page 142].
		 */
		if (GjkInternalJohnsonsAlgorithm(&simplex, c_v, lambda))
		{
			ds_Assert(dist_sq != F32_INFINITY);
			simplex.type -= 1;
			GjkInternalClosestPoints(c1, c2, in1, &simplex, lambda);
			return dist_sq;
		}

		simplex.id[simplex.type] = support_id;
		simplex.dot[simplex.type] = Vec3Dot(simplex.p[simplex.type], simplex.p[simplex.type]);

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
			dist_sq = Vec3Dot(c_v, c_v);
			if (dist_sq <= abs_tol * ma)
			{
				return 0.0f;
			}
		}
	}

	return 0.0f;
}

/********************************** DISTANCE METHODS **********************************/

static f32 SphereDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_SPHERE && b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collisionShape *shape1 = strdb_Address(pipeline->cshape_db, b1->shape_handle);
	const struct collisionShape *shape2 = strdb_Address(pipeline->cshape_db, b2->shape_handle);

	f32 dist_sq = 0.0f;

	const f32 r_sum = shape1->sphere.radius + shape2->sphere.radius + 2.0f * margin;
	if (Vec3DistanceSquared(b1->position, b2->position) > r_sum*r_sum)
	{
		vec3 dir;
		Vec3Sub(dir, b2->position, b1->position);
		Vec3ScaleSelf(dir, 1.0f/Vec3Length(dir));
		Vec3Copy(c1, b1->position);
		Vec3Copy(c2, b2->position);
		Vec3TranslateScaled(c1, dir,  shape1->sphere.radius);
		Vec3TranslateScaled(c2, dir, -shape2->sphere.radius);
		dist_sq = Vec3DistanceSquared(c1, c2);
	}

	return f32_sqrt(dist_sq);
}

static f32 CapsuleSphereDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CAPSULE && b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collisionShape *shape2 = strdb_Address(pipeline->cshape_db, b2->shape_handle);

	const struct capsule *cap = &((struct collisionShape *) strdb_Address(pipeline->cshape_db, b1->shape_handle))->capsule;
	f32 r_sum = cap->radius + shape2->sphere.radius + 2.0f * margin;

	mat3 rot;
	Mat3Quat(rot, b1->rotation);

	vec3 s_p1, s_p2, diff;
	Vec3Sub(c2, b2->position, b1->position);
	s_p1[0] = rot[1][0] * cap->half_height;	
	s_p1[1] = rot[1][1] * cap->half_height;	
	s_p1[2] = rot[1][2] * cap->half_height;	
	Vec3Negate(s_p2, s_p1);
	struct segment s = SegmentConstruct(s_p1, s_p2);

	f32 dist = 0.0f;
	if (SegmentPointDistanceSquared(c1, &s, c2) > r_sum*r_sum)
	{
		Vec3Translate(c1, b1->position);
		Vec3Translate(c2, b1->position);
		Vec3Sub(diff, c2, c1);
		Vec3ScaleSelf(diff, 1.0f / Vec3Length(diff));
		Vec3TranslateScaled(c1, diff, cap->radius);
		Vec3TranslateScaled(c2, diff, -shape2->sphere.radius);

		dist = f32_sqrt(Vec3DistanceSquared(c1, c2));
	}

	return dist;
}

static f32 CapsuleDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CAPSULE && b2->shape_type == COLLISION_SHAPE_CAPSULE);


	struct capsule *cap1 = &((struct collisionShape *) strdb_Address(pipeline->cshape_db, b1->shape_handle))->capsule;
	struct capsule *cap2 = &((struct collisionShape *) strdb_Address(pipeline->cshape_db, b2->shape_handle))->capsule;
	f32 r_sum = cap1->radius + cap2->radius + 2.0f * margin;

	mat3 rot;
	vec3 p0, p1; /* line points */

	Mat3Quat(rot, b1->rotation);
	p0[0] = rot[1][0] * cap1->half_height,	
	p0[1] = rot[1][1] * cap1->half_height,	
	p0[2] = rot[1][2] * cap1->half_height,	
	Vec3Negate(p1, p0);
	Vec3Translate(p0, b1->position);
	Vec3Translate(p1, b1->position);
	struct segment s1 = SegmentConstruct(p0, p1);
	
	Mat3Quat(rot, b2->rotation);
	p0[0] = rot[1][0] * cap2->half_height,	
	p0[1] = rot[1][1] * cap2->half_height,	
	p0[2] = rot[1][2] * cap2->half_height,	
	Vec3Negate(p1, p0);
	Vec3Translate(p0, b2->position);
	Vec3Translate(p1, b2->position);
	struct segment s2 = SegmentConstruct(p0, p1);

	f32 dist = 0.0f;
	if (SegmentDistanceSquared(c1, c2, &s1, &s2) > r_sum*r_sum)
	{
		Vec3Sub(p0, c2, c1);
		Vec3Normalize(p1, p0);
		Vec3TranslateScaled(c1, p1, cap1->radius);
		Vec3TranslateScaled(c2, p1, -cap2->radius);
		dist = f32_sqrt(Vec3DistanceSquared(c1, c2));
	}

	return dist;
}

static f32 HullSphereDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CONVEX_HULL);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collisionShape *shape1 = strdb_Address(pipeline->cshape_db, b1->shape_handle);
	const struct collisionShape *shape2 = strdb_Address(pipeline->cshape_db, b2->shape_handle);

	struct gjkInput g1 = { .v = shape1->hull.v, .v_count = shape1->hull.v_count, };
	Vec3Copy(g1.pos, b1->position);
	Mat3Quat(g1.rot, b1->rotation);

	vec3 n = VEC3_ZERO;
	struct gjkInput g2 = { .v = &n, .v_count = 1, };
	Vec3Copy(g2.pos, b2->position);
	Mat3Identity(g2.rot);

	f32 dist_sq = GjkDistanceSquared(c1, c2, &g1, &g2);
	const f32 r_sum = shape2->sphere.radius + 2.0f * margin;

	if (dist_sq <= r_sum*r_sum)
	{  
		dist_sq = 0.0f;
	}
	else
	{
		Vec3Sub(n, c2, c1);
		Vec3ScaleSelf(n, 1.0f / Vec3Length(n));
		Vec3TranslateScaled(c1, n, margin);
		Vec3TranslateScaled(c2, n, -(shape2->sphere.radius + margin));
	}

	return f32_sqrt(dist_sq);
}

static f32 HullCapsuleDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CONVEX_HULL);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_CAPSULE);

	const struct collisionShape *shape1 = strdb_Address(pipeline->cshape_db, b1->shape_handle);
	const struct collisionShape *shape2 = strdb_Address(pipeline->cshape_db, b2->shape_handle);

	struct gjkInput g1 = { .v = shape1->hull.v, .v_count = shape1->hull.v_count, };
	Vec3Copy(g1.pos, b1->position);
	Mat3Quat(g1.rot, b1->rotation);

	vec3 segment[2];
	Vec3Set(segment[0], 0.0f, shape2->capsule.half_height, 0.0f);
	Vec3Set(segment[1], 0.0f, -shape2->capsule.half_height, 0.0f);
	struct gjkInput g2 = { .v = segment, .v_count = 2, };
	Vec3Copy(g2.pos, b2->position);
	Mat3Identity(g2.rot);

	f32 dist_sq = GjkDistanceSquared(c1, c2, &g1, &g2);
	const f32 r_sum = shape2->capsule.radius + 2.0f * margin;

	if (dist_sq <= r_sum*r_sum)
	{
		dist_sq = 0.0f;
	}
	else
	{
		vec3 n;
		Vec3Sub(n, c2, c1);
		Vec3ScaleSelf(n, 1.0f / Vec3Length(n));
		Vec3TranslateScaled(c1, n, margin);
		Vec3TranslateScaled(c2, n, -(shape2->sphere.radius + margin));
	}

	return f32_sqrt(dist_sq);

}

static f32 HullDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert (b1->shape_type == COLLISION_SHAPE_CONVEX_HULL);
	ds_Assert (b2->shape_type == COLLISION_SHAPE_CONVEX_HULL);

	const struct collisionShape *shape1 = strdb_Address(pipeline->cshape_db, b1->shape_handle);
	const struct collisionShape *shape2 = strdb_Address(pipeline->cshape_db, b2->shape_handle);

	struct gjkInput g1 = { .v = shape1->hull.v, .v_count = shape1->hull.v_count, };
	Vec3Copy(g1.pos, b1->position);
	Mat3Quat(g1.rot, b1->rotation);

	struct gjkInput g2 = { .v = shape2->hull.v, .v_count = shape2->hull.v_count, };
	Vec3Copy(g2.pos, b2->position);
	Mat3Quat(g2.rot, b2->rotation);

	f32 dist_sq = GjkDistanceSquared(c1, c2, &g1, &g2);
	if (dist_sq <= 4.0f*margin*margin)
	{
		dist_sq = 0.0f;
		vec3 n;
		Vec3Sub(n, c2, c1);
		Vec3ScaleSelf(n, 1.0f / Vec3Length(n));
		Vec3TranslateScaled(c1, n, margin);
		Vec3TranslateScaled(c2, n, margin);
	}

	return f32_sqrt(dist_sq);
}

static f32 TriMeshBvhSphereDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_AssertString(0, "implement");
	return 0.0f;
}

static f32 TriMeshBvhCapsuleDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_AssertString(0, "implement");
	return 0.0f;
}

static f32 TriMeshBvhHullDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_AssertString(0, "implement");
	return 0.0f;
}

/********************************** INTERSECTION TESTS **********************************/

static u32 SphereTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_SPHERE && b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collisionShape *shape1 = strdb_Address(pipeline->cshape_db, b1->shape_handle);
	const struct collisionShape *shape2 = strdb_Address(pipeline->cshape_db, b2->shape_handle);
	
	const f32 r_sum = shape1->sphere.radius + shape2->sphere.radius + 2.0f * margin;
	return Vec3DistanceSquared(b1->position, b2->position) <= r_sum*r_sum;
}

static u32 CapsuleSphereTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CAPSULE && b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collisionShape *shape1 = strdb_Address(pipeline->cshape_db, b1->shape_handle);
	const struct collisionShape *shape2 = strdb_Address(pipeline->cshape_db, b2->shape_handle);

	const struct capsule *cap = &shape1->capsule;
	f32 r_sum = cap->radius + shape2->sphere.radius + 2.0f * margin;

	mat3 rot;
	Mat3Quat(rot, b1->rotation);

	vec3 c1, c2, s_p1, s_p2;
	Vec3Sub(c2, b2->position, b1->position);
	s_p1[0] = rot[1][0] * cap->half_height;	
	s_p1[1] = rot[1][1] * cap->half_height;	
	s_p1[2] = rot[1][2] * cap->half_height;	
	Vec3Negate(s_p2, s_p1);
	struct segment s = SegmentConstruct(s_p1, s_p2);

	return SegmentPointDistanceSquared(c1, &s, c2) <= r_sum*r_sum;
}

static u32 CapsuleTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	vec3 c1, c2;
	return CapsuleDistance(c1, c2, pipeline, b1, b2, margin) == 0.0f;
}

static u32 HullSphereTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	vec3 c1, c2;
	return HullSphereDistance(c1, c2, pipeline, b1, b2, margin) == 0.0f;
}

static u32 HullCapsuleTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	vec3 c1, c2;
	return HullCapsuleDistance(c1, c2, pipeline, b1, b2, margin) == 0.0f;
}

static u32 HullTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	vec3 c1, c2;
	return HullDistance(c1, c2, pipeline, b1, b2, margin) == 0.0f;
}

static u32 TriMeshBvhSphereTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	vec3 c1, c2;
	return TriMeshBvhSphereDistance(c1, c2, pipeline, b1, b2, margin) == 0.0f;
}

static u32 TriMeshBvhCapsuleTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	vec3 c1, c2;
	return TriMeshBvhCapsuleDistance(c1, c2, pipeline, b1, b2, margin) == 0.0f;
}

static u32 TriMeshBvhHullTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	vec3 c1, c2;
	return TriMeshBvhHullDistance(c1, c2, pipeline, b1, b2, margin) == 0.0f;
}

/********************************** CONTACT MANIFOLD METHODS **********************************/

static u32 SphereContact(struct arena *garbage, struct collisionResult *result, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_SPHERE);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collisionShape *shape1 = strdb_Address(pipeline->cshape_db, b1->shape_handle);
	const struct collisionShape *shape2 = strdb_Address(pipeline->cshape_db, b2->shape_handle);

	result->type = COLLISION_NONE;
	u32 contact_generated = 0;

	const f32 r_sum = shape1->sphere.radius + shape2->sphere.radius + 2.0f * margin;
	const f32 dist_sq = Vec3DistanceSquared(b1->position, b2->position);
	if (dist_sq <= r_sum*r_sum)
	{
		result->type = COLLISION_CONTACT;
		contact_generated = 1;
		result->manifold.v_count = 1;
		if (dist_sq <= COLLISION_POINT_DIST_SQ)
		{
			//TODO(Degenerate): spheres have same center => normal returned should depend on the context.
			Vec3Set(result->manifold.n, 0.0f, 1.0f, 0.0f);
		}
		else
		{
			Vec3Sub(result->manifold.n, b2->position, b1->position);
			Vec3ScaleSelf(result->manifold.n, 1.0f/Vec3Length(result->manifold.n));
		}

		vec3 c1, c2;
		Vec3Copy(c1, b1->position);
		Vec3Copy(c2, b2->position);
		Vec3TranslateScaled(c1, result->manifold.n, shape1->sphere.radius + margin);
		Vec3TranslateScaled(c2, result->manifold.n, -(shape2->sphere.radius + margin));
		result->manifold.depth[0] = Vec3Dot(c1, result->manifold.n) - Vec3Dot(c2, result->manifold.n);
		Vec3Interpolate(result->manifold.v[0], c1, c2, 0.5f);
	}

	return contact_generated;
}

static u32 CapsuleSphereContact(struct arena *garbage, struct collisionResult *result, const struct ds_RigidBodyPipeline *pipeline,  const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CAPSULE);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collisionShape *shape1 = strdb_Address(pipeline->cshape_db, b1->shape_handle);
	const struct collisionShape *shape2 = strdb_Address(pipeline->cshape_db, b2->shape_handle);

	result->type = COLLISION_NONE;
	u32 contact_generated = 0;

	const struct capsule *cap = &shape1->capsule;
	const f32 r_sum = cap->radius + shape2->sphere.radius + 2.0f * margin;

	mat3 rot;
	Mat3Quat(rot, b1->rotation);

	vec3 c1, c2, s_p1, s_p2, diff;
	Vec3Sub(c2, b2->position, b1->position);
	s_p1[0] = rot[1][0] * cap->half_height;	
	s_p1[1] = rot[1][1] * cap->half_height;	
	s_p1[2] = rot[1][2] * cap->half_height;	
	Vec3Negate(s_p2, s_p1);
	struct segment s = SegmentConstruct(s_p1, s_p2);
	const f32 dist_sq = SegmentPointDistanceSquared(c1, &s, c2);

	if (dist_sq <= r_sum*r_sum)
	{
		result->type = COLLISION_CONTACT;
		contact_generated = 1;
		result->manifold.v_count = 1;
		if (dist_sq <= COLLISION_POINT_DIST_SQ)
		{
			//TODO Degerate case: normal should be context dependent
			Vec3Copy(result->manifold.v[0], b1->position);
			if (s.dir[0]*s.dir[0] < s.dir[1]*s.dir[1])
			{
				if (s.dir[0]*s.dir[0] < s.dir[2]*s.dir[2]) { Vec3Set(result->manifold.v[2], 1.0f, 0.0f, 0.0f); }
				else { Vec3Set(result->manifold.v[2], 0.0f, 0.0f, 1.0f); }
			}
			else
			{
				if (s.dir[1]*s.dir[1] < s.dir[2]*s.dir[2]) { Vec3Set(result->manifold.v[0], 0.0f, 1.0f, 0.0f); }
				else { Vec3Set(result->manifold.v[2], 0.0f, 0.0f, 1.0f); }
			}
				
			Vec3Set(result->manifold.v[2], 1.0f, 0.0f, 0.0f);
			Vec3Cross(diff, result->manifold.v[2], s.dir);
			Vec3Normalize(result->manifold.n, diff);
			result->manifold.depth[0] = r_sum;
		}
		else
		{
			Vec3Sub(diff, c2, c1);
			Vec3Normalize(result->manifold.n, diff);
			Vec3TranslateScaled(c1, result->manifold.n, cap->radius + margin);
			Vec3TranslateScaled(c2, result->manifold.n, -(shape2->sphere.radius + margin));
			result->manifold.depth[0] = Vec3Dot(c1, result->manifold.n) - Vec3Dot(c2, result->manifold.n);
			Vec3Interpolate(result->manifold.v[0], c1, c2, 0.5f);
			Vec3Translate(result->manifold.v[0], b1->position);
		}	
	}

	return contact_generated;
}

static u32 CapsuleContact(struct arena *garbage, struct collisionResult *result, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CAPSULE);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_CAPSULE);

	u32 contact_generated = 0;
	result->type = COLLISION_NONE;

	const struct capsule *cap1 = &((struct collisionShape *) strdb_Address(pipeline->cshape_db, b1->shape_handle))->capsule;
	const struct capsule *cap2 = &((struct collisionShape *) strdb_Address(pipeline->cshape_db, b2->shape_handle))->capsule;
	f32 r_sum = cap1->radius + cap2->radius + 2.0f * margin;

	mat3 rot;
	vec3 c1, c2, p0, p1; /* line points */

	Mat3Quat(rot, b1->rotation);
	p0[0] = rot[1][0] * cap1->half_height;	
	p0[1] = rot[1][1] * cap1->half_height;	
	p0[2] = rot[1][2] * cap1->half_height;	
	Vec3Negate(p1, p0);
	Vec3Translate(p0, b1->position);
	Vec3Translate(p1, b1->position);
	struct segment s1 = SegmentConstruct(p0, p1);
	
	Mat3Quat(rot, b2->rotation);
	p0[0] = rot[1][0] * cap2->half_height;	
	p0[1] = rot[1][1] * cap2->half_height;	
	p0[2] = rot[1][2] * cap2->half_height;	
	Vec3Negate(p1, p0);
	Vec3Translate(p0, b2->position);
	Vec3Translate(p1, b2->position);
	struct segment s2 = SegmentConstruct(p0, p1);

	const f32 dist_sq = SegmentDistanceSquared(c1, c2, &s1, &s2);
	if (dist_sq <= r_sum*r_sum)
	{
		result->type = COLLISION_CONTACT;
		contact_generated = 1;
		vec3 cross;
		Vec3Cross(cross, s1.dir, s2.dir);
		const f32 cross_dist_sq = Vec3LengthSquared(cross);
		if (dist_sq <= COLLISION_POINT_DIST_SQ)
		{
			/* Degenerate Case 1: Parallel capsules,*/
			result->manifold.depth[0] = r_sum;
			Vec3Copy(result->manifold.v[0], b1->position);
			if (cross_dist_sq <= COLLISION_POINT_DIST_SQ)
			{
				result->manifold.v_count = 1;

				//TODO Normal should be context dependent
				if (s1.dir[0]*s1.dir[0] < s1.dir[1]*s1.dir[1])
				{
					if (s1.dir[0]*s1.dir[0] < s1.dir[2]*s1.dir[2]) { Vec3Set(result->manifold.n, 1.0f, 0.0f, 0.0f); }
					else { Vec3Set(result->manifold.n, 0.0f, 0.0f, 1.0f); }
				}
				else
				{
					if (s1.dir[1]*s1.dir[1] < s1.dir[2]*s1.dir[2]) { Vec3Set(result->manifold.n, 0.0f, 1.0f, 0.0f); }
					else { Vec3Set(result->manifold.n, 0.0f, 0.0f, 1.0f); }
				}
				Vec3Cross(p0, s1.dir, result->manifold.n);
				Vec3Normalize(result->manifold.n, p0);
			}
			/* Degenerate Case 2: Non-Parallel capsules, */
			else
			{
				result->manifold.v_count = 1;
				Vec3Normalize(result->manifold.n, cross);
			}
		}
		else
		{
			Vec3Sub(result->manifold.n, c2, c1);
			Vec3ScaleSelf(result->manifold.n, 1.0f / Vec3Length(result->manifold.n));
			Vec3TranslateScaled(c1, result->manifold.n, cap1->radius + margin);
			Vec3TranslateScaled(c2, result->manifold.n, -(cap2->radius + margin));
			const f32 d = Vec3Dot(c1, result->manifold.n) - Vec3Dot(c2, result->manifold.n);
			result->manifold.depth[0] = d;
			if (cross_dist_sq <= COLLISION_POINT_DIST_SQ)
			{
				const f32 t1 = SegmentPointClosestBcParameter(&s1, s2.p0);
				const f32 t2 = SegmentPointClosestBcParameter(&s1, s2.p1);

				if (t1 != t2)
				{
					result->manifold.v_count = 2;
					result->manifold.depth[1] = d;
					SegmentBc(result->manifold.v[0], &s1, t1);
					SegmentBc(result->manifold.v[1], &s1, t2);
				}
				/* end-point contact point */
				else
				{
					result->manifold.v_count = 1;
					Vec3Interpolate(result->manifold.v[0], c1, c2, 0.5f);
				}
			}
			else
			{
				result->manifold.v_count = 1;
				Vec3Interpolate(result->manifold.v[0], c1, c2, 0.5f);
			}
		}
	}

	return contact_generated;
}

static u32 HullSphereContact(struct arena *garbage, struct collisionResult *result, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert (b1->shape_type == COLLISION_SHAPE_CONVEX_HULL);
	ds_Assert (b2->shape_type == COLLISION_SHAPE_SPHERE);

	const struct collisionShape *shape1 = strdb_Address(pipeline->cshape_db, b1->shape_handle);
	const struct collisionShape *shape2 = strdb_Address(pipeline->cshape_db, b2->shape_handle);

	result->type = COLLISION_NONE;
	u32 contact_generated = 0;

	struct gjkInput g1 = { .v = shape1->hull.v, .v_count = shape1->hull.v_count, };
	Vec3Copy(g1.pos, b1->position);
	Mat3Quat(g1.rot, b1->rotation);

	vec3 zero = VEC3_ZERO;
	struct gjkInput g2 = { .v = &zero, .v_count = 1, };
	Vec3Copy(g2.pos, b2->position);
	Mat3Identity(g2.rot);

	vec3 c1, c2;
	const f32 dist_sq = GjkDistanceSquared(c1, c2, &g1, &g2);
	const f32 r_sum = shape2->sphere.radius + 2.0f * margin;

	/* Deep Penetration */
	if (dist_sq <= margin*margin)
	{
		result->type = COLLISION_CONTACT;
		contact_generated = 1;
		result->manifold.v_count = 1;

		vec3 n;	
		const struct dcel *h = &shape1->hull;
		f32 min_depth = F32_INFINITY;
		vec3 diff;
		vec3 p, best_p;
		for (u32 fi = 0; fi < h->f_count; ++fi)
		{
			DcelFaceNormal(p, h, fi);
			Mat3VecMul(n, g1.rot, p);
			Mat3VecMul(p, g1.rot, h->v[h->e[h->f[fi].first].origin]);
			Vec3Translate(p, b1->position);
			Vec3Sub(diff, p, b2->position);
			const f32 depth = Vec3Dot(n, diff);
			if (depth < min_depth)
			{
				Vec3Copy(best_p, p);
				Vec3Copy(result->manifold.n, n);
				min_depth = depth;
			}
		}

		Vec3Sub(diff, best_p, b2->position);
		result->manifold.depth[0] = Vec3Dot(result->manifold.n, diff) + shape2->sphere.radius + 2.0f * margin;

		Vec3Copy(result->manifold.v[0], b2->position);
		Vec3TranslateScaled(result->manifold.v[0], result->manifold.n, margin + min_depth);
	}
	/* Shallow Penetration */
	else if (dist_sq <= r_sum*r_sum)
	{
		result->type = COLLISION_CONTACT;
		contact_generated = 1;
		result->manifold.v_count = 1;

		Vec3Sub(result->manifold.n, c2, c1);
		Vec3ScaleSelf(result->manifold.n, 1.0f / Vec3Length(result->manifold.n));

		Vec3TranslateScaled(c1, result->manifold.n, margin);
		Vec3TranslateScaled(c2, result->manifold.n, -(shape2->sphere.radius + margin));
		result->manifold.depth[0] = Vec3Dot(c1, result->manifold.n) - Vec3Dot(c2, result->manifold.n);

		Vec3Interpolate(result->manifold.v[0], c1, c2, 0.5f);
	}

	return contact_generated;
}

static u32 HullCapsuleContact(struct arena *garbage, struct collisionResult *result, const struct ds_RigidBodyPipeline *pipeline,  const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_CONVEX_HULL);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_CAPSULE);

	result->type = COLLISION_NONE;
	u32 contact_generated = 0;

	const struct collisionShape *shape1 = strdb_Address(pipeline->cshape_db, b1->shape_handle);
	const struct collisionShape *shape2 = strdb_Address(pipeline->cshape_db, b2->shape_handle);

	const struct dcel *h = &shape1->hull;
	struct gjkInput g1 = { .v = h->v, .v_count = h->v_count, };
	Vec3Copy(g1.pos, b1->position);
	Mat3Quat(g1.rot, b1->rotation);

	vec3 segment[2];
	Vec3Set(segment[0], 0.0f, shape2->capsule.half_height, 0.0f);
	Vec3Set(segment[1], 0.0f, -shape2->capsule.half_height, 0.0f);
	Vec3Negate(segment[1], segment[0]);
	struct gjkInput g2 = { .v = segment, .v_count = 2, };
	Vec3Copy(g2.pos, b2->position);
	//Mat3Identity(g2.rot);
	Mat3Quat(g2.rot, b2->rotation);

	vec3 c1, c2;
	const f32 dist_sq = GjkDistanceSquared(c1, c2, &g1, &g2);
	const f32 r_sum = shape2->capsule.radius + 2.0f * margin;
	if (dist_sq <= r_sum*r_sum)
	{
		result->type = COLLISION_CONTACT;
		contact_generated = 1;

		vec3 p1, p2, tmp;
		Mat3VecMul(p1, g2.rot, g2.v[0]);
		Mat3VecMul(p2, g2.rot, g2.v[1]);
		Vec3Translate(p1, g2.pos);
		Vec3Translate(p2, g2.pos);
		struct segment cap_s = SegmentConstruct(p1, p2);

		g2.v_count = 1;
		const u32 cap_p0_inside = (GjkDistanceSquared(p1, tmp, &g1, &g2) == 0.0f) ? 1 : 0;
		Vec3Copy(g2.v[0], g2.v[1]);
		const u32 cap_p1_inside = (GjkDistanceSquared(p2, tmp, &g1, &g2) == 0.0f) ? 1 : 0;

		/* Deep Penetration */
		if (dist_sq <= margin*margin)
		{
			u32 edge_best = 0; 
			u32 best_index = 0;

			f32 max_d0 = -F32_INFINITY;
			f32 max_d1 = -F32_INFINITY;
			f32 max_signed_depth = -F32_INFINITY;

			for (u32 fi = 0; fi < h->f_count; ++fi)
			{
				struct plane pl = DcelFacePlane(h, g1.rot, b1->position, fi);

				const f32 d0 = PlanePointSignedDistance(&pl, cap_s.p0);
				const f32 d1 = PlanePointSignedDistance(&pl, cap_s.p1);
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
					struct segment edge_s = DcelEdgeSegment(h, g1.rot, g1.pos, best_index);
					
					const f32 d = -f32_sqrt(SegmentDistanceSquared(c1, c2, &edge_s, &cap_s));
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
				struct segment edge_s = DcelEdgeSegment(h, g1.rot, g1.pos, best_index);
				SegmentDistanceSquared(c1, c2, &edge_s, &cap_s);
				Vec3Sub(result->manifold.n, c1, c2);
				Vec3ScaleSelf(result->manifold.n, 1.0f / Vec3Length(result->manifold.n));
				Vec3Copy(result->manifold.v[0], c1);
			}
			else
			{
				result->manifold.v_count = 2;
				DcelFaceNormal(c1, h, best_index);
				Mat3VecMul(result->manifold.n, g1.rot, c1);
				struct segment s = DcelFaceClipSegment(h, g1.rot, g1.pos, best_index, &cap_s);
				const struct plane pl = DcelFacePlane(h, g1.rot, g1.pos, best_index);

				//COLLISION_DEBUG_ADD_SEGMENT(cap_s);
				//COLLISION_DEBUG_ADD_SEGMENT(s);

				if (cap_p0_inside == 1 && cap_p1_inside == 0)
				{
					Vec3Copy(result->manifold.v[0], s.p0);
					PlaneSegmentClip(result->manifold.v[1], &pl, &s);
				}
				else if (cap_p0_inside == 0 && cap_p1_inside == 1)
				{
					PlaneSegmentClip(result->manifold.v[0], &pl, &s);
					Vec3Copy(result->manifold.v[1], s.p1);
				}
				else
				{
					Vec3Copy(result->manifold.v[0], s.p0);
					Vec3Copy(result->manifold.v[1], s.p1);
				}
				
				Vec3TranslateScaled(result->manifold.v[0], result->manifold.n, -PlanePointSignedDistance(&pl, result->manifold.v[0]));
				Vec3TranslateScaled(result->manifold.v[1], result->manifold.n, -PlanePointSignedDistance(&pl, result->manifold.v[1]));
			}
		}
		/* Shallow Penetration */
		else
		{
			//COLLISION_DEBUG_ADD_SEGMENT(SegmentConstruct(cap_s.p0, cap_s.p1));
			//COLLISION_DEBUG_ADD_SEGMENT(SegmentConstruct(c1, c2));

			Vec3Sub(result->manifold.n, c2, c1);
			Vec3ScaleSelf(result->manifold.n, 1.0f / Vec3Length(result->manifold.n));

			const struct dcel *h = &shape1->hull;

			/* (1) compute closest face points for end-point segement */
			vec3 s_dir, diff;
			Vec3Normalize(s_dir, cap_s.dir);

			struct segment s = SegmentConstruct(p1, p2);
			u32 fi;
			vec3 n1;
			u32 parallel = 0;
		
			/* If projected segment is not a point */
			if (Vec3Dot(s.dir, s.dir) > COLLISION_POINT_DIST_SQ)
			{
				/* (2) Check if capsule is infront of some parallel plane   */
				/* find parallel face with Vec3Dot(face_normal, segment_points) > 0.0f */
				struct dcelFace *f;
				for (fi = 0; fi < h->f_count; ++fi)
				{
					f = h->f + fi;
					DcelFaceNormal(n1, h, fi);

					const f32 d1d1 = Vec3Dot(n1, n1);
					const f32 d2d2 = Vec3Dot(s_dir, s_dir);
					const f32 d1d2 = Vec3Dot(n1, s_dir);
					const f32 denom = d1d1*d2d2 - d1d2*d1d2;

					/* denom = (1-cos(theta)^2) == 1.0f <=> capsule and face normal orthogonal */
					//if (d1d2*d1d2 <= COLLISION_POINT_DIST_SQ)
					if (denom >= 1.0f - COLLISION_POINT_DIST_SQ)
					{	
						Mat3VecMul(p2, g2.rot, g2.v[0]);
						Vec3Translate(p2, g2.pos);
						Mat3VecMul(p1, g1.rot, h->v[h->e[f->first].origin]);
						Vec3Translate(p1, g1.pos);
						Vec3Sub(diff, p2, p1);
						
						/* is capsule infront of face? */
						if (Vec3Dot(diff, n1) > 0.0f)
						{
							vec3 center;
							Vec3Interpolate(center, s.p0, s.p1, 0.5f);
							Vec3Translate(n1, center);
							parallel = 1;
							break;
						}
					}
				}
			}

			if (parallel)
			{
				result->manifold.v_count = 2;
				DcelFaceNormal(result->manifold.n, h, fi);
				Vec3TranslateScaled(c1, result->manifold.n, margin);
				Vec3TranslateScaled(c2, result->manifold.n, -(shape2->capsule.radius + margin));
				result->manifold.depth[0] = Vec3Dot(result->manifold.n, c1) - Vec3Dot(result->manifold.n, c2);
				result->manifold.depth[1] = result->manifold.depth[0];
				struct segment s = DcelFaceClipSegment(h, g1.rot, g1.pos, fi, &cap_s);
				Vec3Copy(result->manifold.v[0], s.p0);
				Vec3Copy(result->manifold.v[1], s.p1);
				Vec3TranslateScaled(result->manifold.v[0], result->manifold.n, -(shape2->capsule.radius + 2.0f*margin -result->manifold.depth[0]));
				Vec3TranslateScaled(result->manifold.v[1], result->manifold.n, -(shape2->capsule.radius + 2.0f*margin -result->manifold.depth[1]));
			}
			else
			{
				result->manifold.v_count = 1;
				Vec3Sub(result->manifold.n, c2, c1);
				Vec3ScaleSelf(result->manifold.n, 1.0f / Vec3Length(result->manifold.n));
				Vec3TranslateScaled(c1, result->manifold.n, margin);
				Vec3TranslateScaled(c2, result->manifold.n, -(shape2->capsule.radius + margin));
				result->manifold.depth[0] = Vec3Dot(result->manifold.n, c1) - Vec3Dot(result->manifold.n, c2);
				Vec3Copy(result->manifold.v[0], c1);
			}
		}
	}

	return contact_generated;
}

struct satFaceQuery
{
	constvec3ptr v;
	vec3 normal;
	u32 fi;
	f32 depth;
};

struct satEdgeQuery
{
	struct segment s1;
	struct segment s2;
	u32	e1;
	u32	e2;
	vec3 normal;
	f32 depth;
};

static u32 HullContactInternalFaceContact(struct arena *mem_tmp, struct contactManifold *cm, const vec3 cm_n, const struct dcel *ref_dcel, const vec3 n_ref, const u32 ref_face_index, constvec3ptr v_ref, const struct dcel *inc_dcel, constvec3ptr v_inc)
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

		Vec3Sub(tmp1, v_inc[i1], v_inc[i0]);
		Vec3Sub(tmp2, v_inc[i2], v_inc[i0]);
		Vec3Cross(n, tmp1, tmp2);
		Vec3ScaleSelf(n, 1.0f / Vec3Length(n));

		const f32 dot = Vec3Dot(n_ref, n);
		if (dot < min_dot)
		{
			min_dot = dot;
			inc_fi = fi;
		}
	}
	
	struct dcelFace *ref_face = ref_dcel->f + ref_face_index;
	struct dcelFace *inc_face = inc_dcel->f + inc_fi;

	/* (2) Setup world polygons */
	struct stackVec3 clip_stack[2];
	clip_stack[0] = stackVec3Alloc(mem_tmp, 2*inc_face->count + ref_face->count, NOT_GROWABLE);
	clip_stack[1] = stackVec3Alloc(mem_tmp, 2*inc_face->count + ref_face->count, NOT_GROWABLE);
	u32 cur = 0;
	vec3ptr ref_v = ArenaPush(mem_tmp, ref_face->count * sizeof(vec3));
	vec3ptr cp = ArenaPush(mem_tmp, (2*inc_face->count + ref_face->count) * sizeof(vec3));

	for (u32 i = 0; i < ref_face->count; ++i)
	{
		const u32 vi = ref_dcel->e[ref_face->first + i].origin;
		Vec3Copy(ref_v[i], v_ref[vi]);
	}

	for (u32 i = 0; i < inc_face->count; ++i)
	{
		const u32 vi = inc_dcel->e[inc_face->first + i].origin;
		stackVec3Push(clip_stack + cur, v_inc[vi]);
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
		stackVec3Flush(clip_stack + cur);

		Vec3Sub(tmp1, ref_v[(j+1) % ref_face->count], ref_v[j]);
		Vec3Cross(n, tmp1, n_ref);
		Vec3ScaleSelf(n, 1.0f / Vec3Length(n));
		struct plane clip_plane = PlaneConstruct(n, ref_v[j]);

		for (u32 i = 0; i < clip_stack[prev].next; ++i)
		{
			const struct segment clip_edge = SegmentConstruct(clip_stack[prev].arr[i], clip_stack[prev].arr[(i+1) % clip_stack[prev].next]);
			const f32 t = PlaneSegmentClipParameter(&clip_plane, &clip_edge);

			vec3 inter;
			Vec3Interpolate(inter, clip_edge.p1, clip_edge.p0, t);

			if (PlanePointBehindCheck(&clip_plane, clip_edge.p0))
			{
				stackVec3Push(clip_stack + cur, clip_edge.p0);
				if (0.0f < t && t < 1.0f)
				{
					stackVec3Push(clip_stack + cur, inter);
				}
			}
			else if (PlanePointBehindCheck(&clip_plane, clip_edge.p1))
			{
				stackVec3Push(clip_stack + cur, inter);
			}
		}
	}

	f32 max_depth = -F32_INFINITY;
	u32 deepest_point = 0;
	u32 cp_count = 0;
	
	for (u32 i = 0; i < clip_stack[cur].next; ++i)
	{
		Vec3Copy(cp[cp_count], clip_stack[cur].arr[i]);
		Vec3Sub(tmp1, cp[cp_count], ref_v[0]);
		depth[cp_count] = -Vec3Dot(tmp1, n_ref);
		if (depth[cp_count] >= 0.0f)
		{
			Vec3TranslateScaled(cp[cp_count], n_ref, depth[cp_count]);
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
		COLLISION_DEBUG_ADD_SEGMENT(SegmentConstruct(cp[i], cp[(i+1) % cp_count]), Vec4Inline(0.8f, 0.6, 0.1f, 1.0f));
	}

	u32 is_colliding = 1;
	Vec3Copy(cm->n, cm_n);
	switch (cp_count)
	{
		case 0:
		{
			is_colliding = 0;
		} break;

		case 1:
		{
			cm->v_count = 1;
			Vec3Copy(cm->v[0], cp[0]);
			cm->depth[0] = depth[0];
		} break;

		case 2:
		{
			cm->v_count = 2;
			Vec3Copy(cm->v[0], cp[0]);
			Vec3Copy(cm->v[1], cp[1]);
			cm->depth[0] = depth[0];
			cm->depth[1] = depth[1];

		} break;

		case 3:
		{
			cm->v_count = 3;
			Vec3Sub(tmp1, cp[1], cp[0]);	
			Vec3Sub(tmp2, cp[2], cp[0]);	
			Vec3Cross(n, tmp1, tmp2);
			if (Vec3Dot(n, cm->n) >= 0.0f)
			{
				Vec3Copy(cm->v[0], cp[0]);
				Vec3Copy(cm->v[1], cp[1]);
				Vec3Copy(cm->v[2], cp[2]);
				cm->depth[0] = depth[0];
				cm->depth[1] = depth[1];
				cm->depth[2] = depth[2];
			}
			else
			{
				Vec3Copy(cm->v[0], cp[0]);
				Vec3Copy(cm->v[2], cp[1]);
				Vec3Copy(cm->v[1], cp[2]);
				cm->depth[0] = depth[0];
				cm->depth[2] = depth[1];
				cm->depth[1] = depth[2];
			}
		} break;

		default:
		{
			/* (1) First point is deepest point */
			cm->v_count = 4;
			Vec3Copy(cm->v[0], cp[deepest_point]);
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

				const f32 dist = Vec3DistanceSquared(cp[deepest_point], cp[i]);
				if (max_dist < dist)
				{
					max_dist = dist;
					max_i = i;
				}
			}
			Vec3Copy(cm->v[2], cp[max_i]);
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
				Vec3Sub(tmp1, cm->v[0], cp[i]);
				Vec3Sub(tmp2, cm->v[2], cp[i]);
				Vec3Cross(n, tmp1, tmp2);
				const f32 d = Vec3LengthSquared(n);
				if (max_pos < d)
				{
					max_pos = d;
					max_pos_i = i;
				}
			}

			for (u32 i = (max_i + 1) % cp_count; i != deepest_point; i = (i+1) % cp_count)
			{
				Vec3Sub(tmp1, cm->v[0], cp[i]);
				Vec3Sub(tmp2, cm->v[2], cp[i]);
				Vec3Cross(n, tmp1, tmp2);
				const f32 d = Vec3LengthSquared(n);
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
			TriCcwDirection(dir, cm->v[0], cp[max_pos_i], cm->v[2]);
			if (Vec3Dot(dir, cm->n) < 0.0f)
			{
				Vec3Copy(cm->v[3], cp[max_pos_i]);
				Vec3Copy(cm->v[1], cp[max_neg_i]);
				cm->depth[3] = depth[max_pos_i];
				cm->depth[1] = depth[max_neg_i];
			}
			else
			{
				Vec3Copy(cm->v[3], cp[max_neg_i]);
				Vec3Copy(cm->v[1], cp[max_pos_i]);
				cm->depth[3] = depth[max_neg_i];
				cm->depth[1] = depth[max_pos_i];
			}

		} break;
	}

	return is_colliding;
}

static u32 HullContactInternalFVSeparation(struct satFaceQuery *query, const struct dcel *h1, constvec3ptr v1_world, const struct dcel *h2, constvec3ptr v2_world)
{
	for (u32 fi = 0; fi < h1->f_count; ++fi)
	{
		const u32 f_v0 = h1->e[h1->f[fi].first + 0].origin;
		const u32 f_v1 = h1->e[h1->f[fi].first + 1].origin;
		const u32 f_v2 = h1->e[h1->f[fi].first + 2].origin;
		const struct plane sep_plane = PlaneConstructFromCcwTriangle(v1_world[f_v0], v1_world[f_v1], v1_world[f_v2]);
		f32 min_dist = F32_INFINITY;
		for (u32 i = 0; i < h2->v_count; ++i)
		{
			const f32 dist = PlanePointSignedDistance(&sep_plane, v2_world[i]);
			if (dist < min_dist)
			{
				min_dist = dist;
			}
		}

		if (min_dist > 0.0f) 
		{ 
			query->fi = fi;
			query->depth = min_dist;
			Vec3Copy(query->normal, sep_plane.normal);
			return 1; 
		}

		if (query->depth < min_dist)
		{
			query->fi = fi;
			query->depth = min_dist;
			/* We switch the sign of the normal outside the function, if need be */
			Vec3Copy(query->normal, sep_plane.normal);
		}
	}

	return 0;
}

static u32 InternalEeIsMinkowskiFace(const vec3 n1_1, const vec3 n1_2, const vec3 n2_1, const vec3 n2_2, const vec3 arc_n1, const vec3 arc_n2)
{
	const f32 n1_1d = Vec3Dot(n1_1, arc_n2);
	const f32 n1_2d = Vec3Dot(n1_2, arc_n2);
	const f32 n2_1d = Vec3Dot(n2_1, arc_n1);
	const f32 n2_2d = Vec3Dot(n2_2, arc_n1);

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

static void HullContactInternalEECheck(struct satEdgeQuery *query, const struct dcel *h1, constvec3ptr v1_world, const u32 e1_1, const struct dcel *h2, constvec3ptr v2_world, const u32 e2_1, const vec3 h1_world_center)
{
	vec3 n1_1, n1_2, n2_1, n2_2, e1, e2;
	const u32 e1_2 = h1->e[e1_1].twin;
	const u32 e2_2 = h2->e[e2_1].twin;

	const u32 f1_1 = h1->e[e1_1].face_ccw;
	const u32 f1_2 = h1->e[e1_2].face_ccw;
	const u32 f2_1 = h2->e[e2_1].face_ccw;
	const u32 f2_2 = h2->e[e2_2].face_ccw;
	TriCcwDirection(n1_1, v1_world[h1->e[h1->f[f1_1].first + 0].origin],  v1_world[h1->e[h1->f[f1_1].first + 1].origin], v1_world[h1->e[h1->f[f1_1].first + 2].origin]);
	TriCcwDirection(n1_2, v1_world[h1->e[h1->f[f1_2].first + 0].origin],  v1_world[h1->e[h1->f[f1_2].first + 1].origin], v1_world[h1->e[h1->f[f1_2].first + 2].origin]);
	TriCcwDirection(n2_1, v2_world[h2->e[h2->f[f2_1].first + 0].origin],  v2_world[h2->e[h2->f[f2_1].first + 1].origin], v2_world[h2->e[h2->f[f2_1].first + 2].origin]);
	TriCcwDirection(n2_2, v2_world[h2->e[h2->f[f2_2].first + 0].origin],  v2_world[h2->e[h2->f[f2_2].first + 1].origin], v2_world[h2->e[h2->f[f2_2].first + 2].origin]);

	///* we are working with minkowski difference A - B, so gauss map of B is (-B). n2_1, n2_2 cross product stays the same. */
	Vec3NegateSelf(n2_1);	
	Vec3NegateSelf(n2_2);

	const struct segment s1 = SegmentConstruct(v1_world[h1->e[e1_1].origin], v1_world[h1->e[e1_2].origin]);
	const struct segment s2 = SegmentConstruct(v2_world[h2->e[e2_1].origin], v2_world[h2->e[e2_2].origin]);

	/* 
	 * test if A, -B edges intersect on gauss map, only if they do, 
	 * they are a candidate for collision
	 */
	if (InternalEeIsMinkowskiFace(n1_1, n1_2, n2_1, n2_2, s1.dir, s2.dir))
	{
		const f32 d1d1 = Vec3Dot(s1.dir, s1.dir);
		const f32 d2d2 = Vec3Dot(s2.dir, s2.dir);
		const f32 d1d2 = Vec3Dot(s1.dir, s2.dir);
		/* Skip parallel edge pairs  */
		if (d1d1*d2d2 - d1d2*d1d2 > F32_EPSILON*100.0f) 
		{
			Vec3Cross(e1, s1.dir, s2.dir);
			Vec3ScaleSelf(e1, 1.0f / Vec3Length(e1));
			Vec3Sub(e2, s1.p0, h1_world_center);
			/* plane normal points from A -> B */
			if (Vec3Dot(e1, e2) < 0.0f)
			{
				Vec3NegateSelf(e1);
			}
			
			/* check segmente-segment distance interval signed plane distance, > 0.0f => we have found a seperating axis */
			Vec3Sub(e2, s2.p0, s1.p0);
			const f32 dist = Vec3Dot(e1, e2);

			if (query->depth < dist)
			{
				query->depth = dist;
				Vec3Copy(query->normal, e1);
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
static u32 HullContactInternalEESeparation(struct satEdgeQuery *query, const struct dcel *h1, constvec3ptr v1_world, const struct dcel *h2, constvec3ptr v2_world, const vec3 h1_world_center)
{
	for (u32 e1_1 = 0; e1_1 < h1->e_count; ++e1_1)
	{
		if (h1->e[e1_1].twin < e1_1) { continue; }

		for (u32 e2_1 = 0; e2_1 < h2->e_count; ++e2_1) 
		{
			if (h2->e[e2_1].twin < e2_1) { continue; }

			HullContactInternalEECheck(query, h1, v1_world, e1_1, h2, v2_world, e2_1, h1_world_center);
			if (query->depth > 0.0f)
			{
				return 1;
			}
		}
	}

	return 0;
}

void SatEdgeQueryCollisionResult(struct contactManifold *manifold, struct satCache *sat_cache, const struct satEdgeQuery *query)
{
	vec3 c1, c2;
	SegmentDistanceSquared(c1, c2, &query->s1, &query->s2);
	COLLISION_DEBUG_ADD_SEGMENT(SegmentConstruct(c1,c2), Vec4Inline(0.0f, 0.8, 0.8f, 1.0f));
	COLLISION_DEBUG_ADD_SEGMENT(query->s1, Vec4Inline(0.0f, 1.0, 0.1f, 1.0f));
	COLLISION_DEBUG_ADD_SEGMENT(query->s2, Vec4Inline(0.0f, 0.1, 1.0f, 1.0f));

	manifold->v_count = 1;
	manifold->depth[0] = -query->depth;
	Vec3Interpolate(manifold->v[0], c1, c2, 0.5f);
	Vec3Copy(manifold->n, query->normal);


	sat_cache->edge1 = query->e1;
	sat_cache->edge2 = query->e2;
	sat_cache->type = SAT_CACHE_CONTACT_EE;
	ds_Assert(1.0f - 1000.0f * F32_EPSILON < Vec3Length(manifold->n));
	ds_Assert(Vec3Length(manifold->n) < 1.0f + 1000.0f * F32_EPSILON);
}

/*
 * For the Algorithm, see
 * 	(Game Physics Pearls, Chapter 4)
 *	(GDC 2013 Dirk Gregorius, https://www.gdcvault.com/play/1017646/Physics-for-Game-Programmers-The)
 */
static u32 HullContact(struct arena *tmp, struct collisionResult *result, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
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
	Mat3Quat(rot1, b1->rotation);
	Mat3Quat(rot2, b2->rotation);

	struct dcel *h1 = &((struct collisionShape *) strdb_Address(pipeline->cshape_db, b1->shape_handle))->hull;
	struct dcel *h2 = &((struct collisionShape *) strdb_Address(pipeline->cshape_db, b2->shape_handle))->hull;

	vec3ptr v1_world = ArenaPush(tmp, h1->v_count * sizeof(vec3));
	vec3ptr v2_world = ArenaPush(tmp, h2->v_count * sizeof(vec3));

	for (u32 i = 0; i < h1->v_count; ++i)
	{
		Mat3VecMul(v1_world[i], rot1, h1->v[i]);
		Vec3Translate(v1_world[i], b1->position);
	}

	for (u32 i = 0; i < h2->v_count; ++i)
	{
		Mat3VecMul(v2_world[i], rot2, h2->v[i]);
		Vec3Translate(v2_world[i], b2->position);
	}

	struct satFaceQuery f_query[2] = { { .depth = -F32_INFINITY }, { .depth = -F32_INFINITY } };
	struct satEdgeQuery e_query = { .depth = -F32_INFINITY };

	u32 colliding = 1;
	u32 calculate = 1;
	struct satCache *sat_cache = NULL;

	const u32 bi1 = PoolIndex(&pipeline->body_pool, b1);
	const u32 bi2 = PoolIndex(&pipeline->body_pool, b2);
	ds_AssertString(bi1 < bi2, "Having these requirements spread all over the pipeline is bad, should\
			standardize some place where we enforce this rule, if at all. Furthermore, we should\
			consider better ways of creating body pair keys");

	u32 cache_found = 1;	
	if ((sat_cache = SatCacheLookup(&pipeline->c_db, bi1, bi2)) == NULL)
	{
		cache_found = 0;	
		sat_cache = &result->sat_cache;
	}
	else
	{
		if (sat_cache->type == SAT_CACHE_SEPARATION)
		{
			vec3 support1, support2, tmp;
			Vec3Negate(tmp, sat_cache->separation_axis);

			VertexSupport(support1, sat_cache->separation_axis, v1_world, h1->v_count);
			VertexSupport(support2, tmp, v2_world, h2->v_count);

			const f32 dot1 = Vec3Dot(support1, sat_cache->separation_axis);
			const f32 dot2 = Vec3Dot(support2, sat_cache->separation_axis);
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
			HullContactInternalEECheck(&e_query, h1, v1_world, sat_cache->edge1, h2, v2_world, sat_cache->edge2, b1->position);
			if (-F32_INFINITY < e_query.depth && e_query.depth < 0.0f)
			{
				calculate = 0;
				SatEdgeQueryCollisionResult(&result->manifold, sat_cache, &e_query);
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
				DcelFaceNormal(cm_n, h1, sat_cache->face);
				Mat3VecMul(ref_n, rot1, cm_n);
				colliding = HullContactInternalFaceContact(tmp, &result->manifold, ref_n, h1, ref_n, sat_cache->face, v1_world, h2, v2_world);
			}
			else
			{
				DcelFaceNormal(cm_n, h2, sat_cache->face);
				Mat3VecMul(ref_n, rot2, cm_n);
				Vec3Negate(cm_n, ref_n);
				colliding = HullContactInternalFaceContact(tmp, &result->manifold, cm_n, h2, ref_n, sat_cache->face, v2_world, h1, v1_world);
			}

			calculate = !colliding;
		}
	}

	if (calculate)
	{
		if (HullContactInternalFVSeparation(&f_query[0], h1, v1_world, h2, v2_world))
		{
			Vec3Copy(sat_cache->separation_axis, f_query[0].normal);
			sat_cache->separation = f_query[0].depth;
			sat_cache->type = SAT_CACHE_SEPARATION;
			colliding = 0;
			goto sat_cleanup;
		}

		if (HullContactInternalFVSeparation(&f_query[1], h2, v2_world, h1, v1_world))
		{
			Vec3Negate(sat_cache->separation_axis, f_query[1].normal);
			sat_cache->separation = f_query[1].depth;
			sat_cache->type = SAT_CACHE_SEPARATION;
			colliding = 0;
			goto sat_cleanup;
		}

		if (HullContactInternalEESeparation(&e_query, h1, v1_world, h2, v2_world, b1->position))
		{
			Vec3Copy(sat_cache->separation_axis, e_query.normal);
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
				colliding = HullContactInternalFaceContact(tmp, &result->manifold, f_query[0].normal, h1, f_query[0].normal, f_query[0].fi, v1_world, h2, v2_world);
			}
			else
			{
				vec3 cm_n;
				sat_cache->body = 1;
				sat_cache->face = f_query[1].fi;
				Vec3Negate(cm_n, f_query[1].normal);
				colliding = HullContactInternalFaceContact(tmp, &result->manifold, cm_n, h2, f_query[1].normal, f_query[1].fi, v2_world, h1, v1_world);
			}

			if (colliding)
			{
				sat_cache->type = SAT_CACHE_CONTACT_FV;
			}
			else
			{
				if (sat_cache->body == 0)
				{
					Vec3Copy(sat_cache->separation_axis, f_query[0].normal);
				}
				else
				{
					Vec3Negate(sat_cache->separation_axis, f_query[1].normal);
				}
				sat_cache->separation = 0.0f;
				sat_cache->type = SAT_CACHE_SEPARATION;
			}
		}
		/* edgeContact */
		else
		{
			SatEdgeQueryCollisionResult(&result->manifold, sat_cache, &e_query);
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

static u32 TriMeshBvhSphereContact(struct arena *tmp, struct collisionResult *result, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_TRI_MESH);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_SPHERE);

	struct triMeshBvh *mesh_bvh = &((struct collisionShape *) strdb_Address(pipeline->cshape_db, b1->shape_handle))->mesh_bvh;
	struct sphere *sph = &((struct collisionShape *) strdb_Address(pipeline->cshape_db, b2->shape_handle))->sphere;

	struct aabb bbox_transform;
	Vec3Sub(bbox_transform.center, b2->position, b1->position);
	Vec3Set(bbox_transform.hw, sph->radius, sph->radius, sph->radius);

	ArenaPushRecord(tmp);

	const struct bvh *bvh = &mesh_bvh->bvh;
	const struct bvhNode *node = (struct bvhNode *) bvh->tree.pool.buf;
	struct memArray arr = ArenaPushAlignedAll(tmp, sizeof(struct bvhNode *), sizeof(struct bvhNode *));
	const struct bvhNode **node_stack = arr.addr;

	if (arr.len == 0)
	{
		Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
		FatalCleanupAndExit();
	}

	u32 sc = 0;
	if (AabbTest(&bbox_transform, &node[bvh->tree.root].bbox))
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
			const struct bvhNode *left = node + node_stack[sc]->bt_left;
			const struct bvhNode *right = node + node_stack[sc]->bt_right;
			if (AabbTest(&bbox_transform, &right->bbox))
			{
				node_stack[sc++] = right;
			}

			if (AabbTest(&bbox_transform, &left->bbox))
			{
				if (sc >= arr.len)
				{
					Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
					FatalCleanupAndExit();
				}
				node_stack[sc++] = left;
			}
		}
	}

	ArenaPopRecord(tmp);

	result->type = COLLISION_NONE;
	return 0;
}

static u32 TriMeshBvhCapsuleContact(struct arena *tmp, struct collisionResult *result, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_TRI_MESH);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_CAPSULE);

	result->type = COLLISION_NONE;
	return 0;
}

static u32 TriMeshBvhHullContact(struct arena *tmp, struct collisionResult *result, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(b1->shape_type == COLLISION_SHAPE_TRI_MESH);
	ds_Assert(b2->shape_type == COLLISION_SHAPE_CONVEX_HULL);

	result->type = COLLISION_NONE;
	return 0;
}

/********************************** RAYCAST **********************************/

static f32 _SphereRaycastParameter(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b, const struct ray *ray)
{
	ds_Assert(b->shape_type == COLLISION_SHAPE_SPHERE);
	const struct collisionShape *shape = strdb_Address(pipeline->cshape_db, b->shape_handle);
	struct sphere sph = SphereConstruct(b->position, shape->sphere.radius);
	return SphereRaycastParameter(&sph, ray);	
}

static f32 CapsuleRaycastParameter(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b, const struct ray *ray)
{
	ds_Assert(b->shape_type == COLLISION_SHAPE_CAPSULE);

	const struct collisionShape *shape = strdb_Address(pipeline->cshape_db, b->shape_handle);
	mat3 rot;
	vec3 p0, p1;
	Mat3Quat(rot, b->rotation);
	p0[0] = rot[1][0] * shape->capsule.half_height;	
	p0[1] = rot[1][1] * shape->capsule.half_height;	
	p0[2] = rot[1][2] * shape->capsule.half_height;	
	Vec3Negate(p1, p0);
	Vec3Translate(p0, b->position);
	Vec3Translate(p1, b->position);
	struct segment s = SegmentConstruct(p0, p1);

	const f32 r = shape->capsule.radius;
	const f32 dist_sq = RaySegmentDistanceSquared(p0, p1, ray, &s);
	if (dist_sq > r*r) { return F32_INFINITY; }

	struct sphere sph = SphereConstruct(p1, r);
	return SphereRaycastParameter(&sph, ray);
}

static f32 HullRaycastParameter(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b, const struct ray *ray)
{
	ds_Assert(b->shape_type == COLLISION_SHAPE_CONVEX_HULL);

	vec3 n, p;
	mat3 rot;
	Mat3Quat(rot, b->rotation);
	const struct dcel *h = &((struct collisionShape *) strdb_Address(pipeline->cshape_db, b->shape_handle))->hull;
	f32 t_best = F32_INFINITY;

	for (u32 fi = 0; fi < h->f_count; ++fi)
	{
		DcelFaceNormal(p, h, fi);
		Mat3VecMul(n, rot, p);
		Vec3Translate(p, b->position);

		struct plane pl = DcelFacePlane(h, rot, b->position, fi);
		const f32 t = PlaneRaycastParameter(&pl, ray);
		if (t < t_best && t >= 0.0f)
		{
			RayPoint(p, ray, t);
			if (DcelFaceProjectedPointTest(h, rot, b->position, fi, p))
			{
				t_best = t;
			}
		}
	}

	return t_best;
}

static f32 TriMeshBvhRaycastParameter(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b, const struct ray *ray)
{
	//TODO should cache frame/longer lived data (obviously true for tri_mesh_bvh...)
	quat inv_quat;
	mat3 inv_rot;
	vec3 tmp;
	QuatInverse(inv_quat, b->rotation);
	Mat3Quat(inv_rot, inv_quat);

	const struct triMeshBvh *mesh_bvh = &((struct collisionShape *) strdb_Address(pipeline->cshape_db, b->shape_handle))->mesh_bvh;
	struct ray rotated_ray;
	Vec3Sub(tmp, ray->origin, b->position);
	Mat3VecMul(rotated_ray.origin, inv_rot, tmp);
	Mat3VecMul(rotated_ray.dir, inv_rot, ray->dir);

	return TriMeshBvhRaycast((struct arena *) &pipeline->frame, mesh_bvh, &rotated_ray).f;
}

/********************************** LOOKUP TABLES FOR SHAPES **********************************/

u32 (*shape_tests[COLLISION_SHAPE_COUNT][COLLISION_SHAPE_COUNT])(const struct ds_RigidBodyPipeline *, const struct ds_RigidBody *, const struct ds_RigidBody *, const f32 margin) =
{
	{ SphereTest, 			0, 				0, 			0, },
	{ CapsuleSphereTest,		CapsuleTest, 			0, 			0, },
	{ HullSphereTest, 		HullCapsuleTest,		HullTest,		0, },
	{ TriMeshBvhSphereTest, 	TriMeshBvhCapsuleTest, 	TriMeshBvhHullTest,		0, },
};

f32 (*distance_methods[COLLISION_SHAPE_COUNT][COLLISION_SHAPE_COUNT])(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *, const struct ds_RigidBody *, const struct ds_RigidBody *, const f32) =
{
	{ SphereDistance,	 	0,				0, 			0, },
	{ CapsuleSphereDistance,	CapsuleDistance, 		0, 			0, },
	{ HullSphereDistance, 		HullCapsuleDistance, 		HullDistance,		0, },
	{ TriMeshBvhSphereDistance,	TriMeshBvhCapsuleDistance, 	TriMeshBvhHullDistance,	0, },
};

u32 (*contact_methods[COLLISION_SHAPE_COUNT][COLLISION_SHAPE_COUNT])(struct arena *, struct collisionResult *, const struct ds_RigidBodyPipeline *, const struct ds_RigidBody *, const struct ds_RigidBody *, const f32) =
{
	{ SphereContact,	 	0, 				0,			0, },
	{ CapsuleSphereContact, 	CapsuleContact,			0,			0, },
	{ HullSphereContact, 	  	HullCapsuleContact,		HullContact, 		0, },
	{ TriMeshBvhSphereContact,	TriMeshBvhCapsuleContact, 	TriMeshBvhHullContact,	0, },
};

f32 (*raycast_parameter_methods[COLLISION_SHAPE_COUNT])(const struct ds_RigidBodyPipeline *, const struct ds_RigidBody *, const struct ray *) =
{
	_SphereRaycastParameter,
	CapsuleRaycastParameter,
	HullRaycastParameter,
	TriMeshBvhRaycastParameter,
};

u32 BodyBodyTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(margin >= 0.0f);
	return (b1->shape_type >= b2->shape_type)  
		? shape_tests[b1->shape_type][b2->shape_type](pipeline, b1, b2, margin)
		: shape_tests[b2->shape_type][b1->shape_type](pipeline, b2, b1, margin);
}

f32 BodyBodyDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
{
	ds_Assert(margin >= 0.0f);
	return (b1->shape_type >= b2->shape_type)  
		? distance_methods[b1->shape_type][b2->shape_type](c1, c2, pipeline, b1, b2, margin)
		: distance_methods[b2->shape_type][b1->shape_type](c2, c1, pipeline, b2, b1, margin);
}

u32 BodyBodyContactManifold(struct arena *tmp, struct collisionResult *result, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin)
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
		Vec3ScaleSelf(result->manifold.n, -1.0f);
	}

	return collision;
}

f32 BodyRaycastParameter(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b, const struct ray *ray)
{
	return raycast_parameter_methods[b->shape_type](pipeline, b, ray);
}

u32 BodyRaycast(vec3 intersection, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b, const struct ray *ray)
{
	const f32 t = BodyRaycastParameter(pipeline, b, ray);
	if (t == F32_INFINITY) return 0;

	Vec3Copy(intersection, ray->origin);
	Vec3TranslateScaled(intersection, ray->dir, t);
	return 1;
}
