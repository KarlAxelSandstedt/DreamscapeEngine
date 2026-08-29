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

#include <string.h>

#include "ds_dynamics.h"
#include "collision.h"

ds_ThreadLocal struct collisionDebug *debug;

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

void c_ManifoldDebugPrint(const struct c_Manifold *cm)
{
	fprintf(stderr, "Contact Manifold:\n{\n");
	fprintf(stderr, "\t.v_count = %u\n", cm->v_count);
	for (u32 i = 0; i < cm->v_count; ++i)
	{
		fprintf(stderr, "\t.v[%u] = { %f, %f, %f }\n", i, cm->v[i][0], cm->v[i][1], cm->v[i][2]);
	}
	fprintf(stderr, "\t.n = { %f, %f, %f }\n", cm->n[0], cm->n[1], cm->n[2]);
	fprintf(stderr, "}\n");
}

u32 c_ManifoldCheck(const struct c_Manifold *cm)
{
    u32 valid = 1;
    const u32 bad_normal = Vec3Length(cm->n) == 0.0f;
    for (u32 i = 0; i < cm->v_count; ++i)
    {
        const u32 bad_collision = bad_normal 
            || Vec3Length(cm->v[i]) > 10000.0f 
            || f32_test_nan(cm->v[i][0]) 
            || f32_test_nan(cm->v[i][1]) 
            || f32_test_nan(cm->v[i][2])
            //|| !(cm->depth[i] >= -0.005f)
                    ;

        if (bad_collision)
        {
            fprintf(stderr, "=== Bad Collision ===\n");
            Vec3Print("\tNormal", cm->n);
            Vec3Print("\tv[0]", cm->v[0]);
            fprintf(stderr, "\tdepth[0]: %f\n", cm->depth[0]);

            if (2 <= cm->v_count)
            {
                Vec3Print("\tv[1]", cm->v[1]);
                fprintf(stderr, "\tdepth[1]: %f\n", cm->depth[1]);
            }

            if (3 <= cm->v_count)
            {
                Vec3Print("\tv[2]", cm->v[2]);
                fprintf(stderr, "\tdepth[2]: %f\n", cm->depth[2]);
            }

            if (4 <= cm->v_count)
            {
                Vec3Print("\tv[3]", cm->v[3]);
                fprintf(stderr, "\tdepth[3]: %f\n", cm->depth[3]);
            }

            valid = 0;
            break;
        }
    }

    return valid;
}

void c_ManifoldTransform(struct c_Manifold *dst, const struct c_Manifold *src, mat3 rot, const vec3 translation)
{
    dst->v_count = src->v_count;
    Mat3VecMul(dst->n, rot, src->n);
    for (u32 vi = 0; vi < dst->v_count; ++vi)
    {
        dst->depth[vi] = src->depth[vi];
        Mat3VecMul(dst->v[vi], rot, src->v[vi]);
        Vec3Translate(dst->v[vi], translation);
    }
}

/********************************** Collision Shape Mass Properties **********************************/

#define VOL	    0 
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

static f32 StaticsLineIntegrals(const vec2 v0, const vec2 v1, const vec2 v2, const u32 p, const u32 q, const vec3 int_scalars)
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
static void StaticsCalculateFaceIntegrals(f32 integrals[10], const struct c_Shape *shape, const u32 fi)
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
		P_a   +=  StaticsLineIntegrals(v0, v1, v2, 2, 0, delta_b);
		P_aa  +=  StaticsLineIntegrals(v0, v1, v2, 3, 0, delta_b);
		P_aaa +=  StaticsLineIntegrals(v0, v1, v2, 4, 0, delta_b);
		P_b   += -StaticsLineIntegrals(v0, v1, v2, 0, 2, delta_a);
		P_bb  += -StaticsLineIntegrals(v0, v1, v2, 0, 3, delta_a);
		P_bbb += -StaticsLineIntegrals(v0, v1, v2, 0, 4, delta_a);
		P_ab  +=  StaticsLineIntegrals(v0, v1, v2, 2, 1, delta_b);
		P_aab +=  StaticsLineIntegrals(v0, v1, v2, 3, 1, delta_b);
		P_abb +=  StaticsLineIntegrals(v0, v1, v2, 1, 3, delta_b);
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

void c_ShapeUpdateMassProperties(struct c_Shape *shape)
{
	ds_Assert(shape->type != C_SHAPE_TRI_MESH);

	f32 I_xx = 0.0f;
	f32 I_yy = 0.0f;
	f32 I_zz = 0.0f;
	f32 I_xy = 0.0f;
	f32 I_xz = 0.0f;
	f32 I_yz = 0.0f;
	vec3 com = VEC3_ZERO;

	if (shape->type == C_SHAPE_CONVEX_HULL)
	{
		f32 integrals[10] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }; 
		for (u32 fi = 0; fi < shape->hull.f_count; ++fi)
		{
			StaticsCalculateFaceIntegrals(integrals, shape, fi);
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
	else if (shape->type == C_SHAPE_SPHERE)
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
	else if (shape->type == C_SHAPE_CAPSULE)
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
struct gjk_Simplex
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

static struct gjk_Simplex gjk_SimplexInit(void)
{
	struct gjk_Simplex simplex = 
	{
		.id = {UINT64_MAX, UINT64_MAX, UINT64_MAX, UINT64_MAX},
		.dot = { -1.0f, -1.0f, -1.0f, -1.0f },
		.type = UINT32_MAX,
	};

	return simplex;
}

static u32 gjk_JohnsonsAlgorithm(struct gjk_Simplex *simplex, vec3 c_v, vec4 lambda)
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

struct gjk_Input
{
	vec3ptr v;
	vec3 pos;
	mat3 rot;
	u32 v_count;
};

static void gjk_ClosestPoints(vec3 c1, vec3 c2, struct gjk_Input *in1, struct gjk_Simplex *simplex, const vec4 lambda)
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

static u32 gjk_Support(vec3 support, const vec3 dir, struct gjk_Input *in)
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

static f32 gjk_DistanceSquared(vec3 c1, vec3 c2, struct gjk_Simplex *simplex, struct gjk_Input *in1, struct gjk_Input *in2)
{
	ds_Assert(in1->v_count > 0);
	ds_Assert(in2->v_count > 0);
	
	const f32 abs_tol = 100.0f * F32_EPSILON;
	const f32 tol = 100.0f * F32_EPSILON;

	*simplex = gjk_SimplexInit();
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
		simplex->type += 1;
		Vec3Scale(dir, c_v, -1.0f);

		const u32 i1 = gjk_Support(s1, dir, in1);
		Vec3Negate(tmp, dir);
		const u32 i2 = gjk_Support(s2, tmp, in2);
		Vec3Sub(simplex->p[simplex->type], s1, s2);
		support_id = ((u64) i1 << 32) | (u64) i2;

		if (dist_sq - Vec3Dot(simplex->p[simplex->type], c_v) <= rel * dist_sq + abs_tol
				|| simplex->id[0] == support_id || simplex->id[1] == support_id 
				|| simplex->id[2] == support_id || simplex->id[3] == support_id)
		{
			ds_Assert(dist_sq != F32_INFINITY);
			simplex->type -= 1;
			gjk_ClosestPoints(c1, c2, in1, simplex, lambda);
			return dist_sq;
		}

		/* find closest point v to origin using naive Johnson's algorithm, update simplex data 
		 * Degenerate Case: due to numerical issues, determinant signs may flip, which may result
		 * either in wrong sub-simplex being chosen, or no valid simplex at all. In that case c_v
		 * stays the same, and we terminate the algorithm. [See page 142].
		 */
		if (gjk_JohnsonsAlgorithm(simplex, c_v, lambda))
		{
			ds_Assert(dist_sq != F32_INFINITY);
			simplex->type -= 1;
			gjk_ClosestPoints(c1, c2, in1, simplex, lambda);
			return dist_sq;
		}

		simplex->id[simplex->type] = support_id;
		simplex->dot[simplex->type] = Vec3Dot(simplex->p[simplex->type], simplex->p[simplex->type]);

		/* 
		 * If the simplex is of type 3, or a tetrahedron, we have encapsulated 0, or, if v is sufficiently
		 * close to the origin, within a margin of error, return an intersection.
		 */
		if (simplex->type == 3)
		{
			gjk_ClosestPoints(c1, c2, in1, simplex, lambda);
			return 0.0f;
		}
		else
		{
			ma = simplex->dot[0];
			ma = f32_max(ma, simplex->dot[1]);
			ma = f32_max(ma, simplex->dot[2]);
			ma = f32_max(ma, simplex->dot[3]);

			/* For error bound discussion, see sections 4.3.5, 4.3.6 */
			dist_sq = Vec3Dot(c_v, c_v);
			if (dist_sq <= abs_tol * ma)
			{
			    gjk_ClosestPoints(c1, c2, in1, simplex, lambda);
				return 0.0f;
			}
		}
	}

	ds_Assert(dist_sq != F32_INFINITY);
	gjk_ClosestPoints(c1, c2, in1, simplex, lambda);
	return dist_sq;
}

/********************************** INTERSECTION TESTS **********************************/

u32 c_SphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert(s1->type == C_SHAPE_SPHERE && s2->type == C_SHAPE_SPHERE);
	const f32 r_sum = s1->sphere.radius + s2->sphere.radius;
	return Vec3DistanceSquared(t1->position, t2->position) <= r_sum*r_sum;
}

u32 c_CapsuleSphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert(s1->type == C_SHAPE_CAPSULE);
    ds_Assert(s2->type == C_SHAPE_SPHERE);

	vec3 c1, c2, s_p1, s_p2;
	Vec3Sub(c2, t2->position, t1->position);

    const struct capsule *cap = &s1->capsule;
    Vec3Set(s_p1, 0.0f, cap->half_height, 0.0f);
    QuatVec3RotateSelf(s_p1, t1->rotation);
	Vec3Negate(s_p2, s_p1);
	struct segment s = SegmentConstruct(s_p1, s_p2);

	const f32 r_sum = s1->capsule.radius + s2->sphere.radius;
	return SegmentPointDistanceSquared(c1, &s, c2) <= r_sum*r_sum;
}

u32 c_CapsuleTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	vec3 c1, c2;
	return c_CapsuleDistance(c1, c2, s1, t1, s2, t2) == 0.0f;
}

u32 c_HullSphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	vec3 c1, c2;
	return c_HullSphereDistance(c1, c2, s1, t1, s2, t2) == 0.0f;
}

u32 c_HullCapsuleTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	vec3 c1, c2;
	return c_HullCapsuleDistance(c1, c2, s1, t1, s2, t2) == 0.0f;
}

u32 c_HullTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	vec3 c1, c2;
	return c_HullDistance(c1, c2, s1, t1, s2, t2) == 0.0f;
}

u32 c_TriMeshBvhSphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	vec3 c1, c2;
	return c_TriMeshBvhSphereDistance(c1, c2, s1, t1, s2, t2) == 0.0f;
}

u32 c_TriMeshBvhCapsuleTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	vec3 c1, c2;
	return c_TriMeshBvhCapsuleDistance(c1, c2, s1, t1, s2, t2) == 0.0f;
}

u32 c_TriMeshBvhHullTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	vec3 c1, c2;
	return c_TriMeshBvhHullDistance(c1, c2, s1, t1, s2, t2) == 0.0f;
}

/********************************** DISTANCE METHODS **********************************/

f32 c_SphereDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert(s1->type == C_SHAPE_SPHERE);
    ds_Assert(s2->type == C_SHAPE_SPHERE);

	f32 dist_sq = 0.0f;

	const f32 r_sum = s1->sphere.radius + s2->sphere.radius;
	if (Vec3DistanceSquared(t1->position, t2->position) > r_sum*r_sum)
	{
		vec3 dir;
		Vec3Sub(dir, t2->position, t1->position);
		Vec3ScaleSelf(dir, 1.0f/Vec3Length(dir));
		Vec3Copy(c1, t1->position);
		Vec3Copy(c2, t2->position);
		Vec3TranslateScaled(c1, dir,  s1->sphere.radius);
		Vec3TranslateScaled(c2, dir, -s2->sphere.radius);
		dist_sq = Vec3DistanceSquared(c1, c2);
	}

	return f32_sqrt(dist_sq);
}

f32 c_CapsuleSphereDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert(s1->type == C_SHAPE_CAPSULE);
    ds_Assert(s2->type == C_SHAPE_SPHERE);

	const struct capsule *cap = &s1->capsule;
	const f32 r_sum = cap->radius + s2->sphere.radius;
	struct segment s = SegmentCapsuleTransform(cap, t1);

	f32 dist = 0.0f;
	if (SegmentPointDistanceSquared(c1, &s, c2) > r_sum*r_sum)
	{
        vec3 diff;
		Vec3Translate(c1, t1->position);
		Vec3Translate(c2, t1->position);
		Vec3Sub(diff, c2, c1);
		Vec3ScaleSelf(diff, 1.0f / Vec3Length(diff));
		Vec3TranslateScaled(c1, diff, cap->radius);
		Vec3TranslateScaled(c2, diff, -s2->sphere.radius);

		dist = f32_sqrt(Vec3DistanceSquared(c1, c2));
	}

	return dist;
}

f32 c_CapsuleDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert(s1->type == C_SHAPE_CAPSULE);
    ds_Assert(s2->type == C_SHAPE_CAPSULE);

	const struct capsule *cap1 = &s1->capsule;
	const struct capsule *cap2 = &s2->capsule;
	const f32 r_sum = cap1->radius + cap2->radius;

	struct segment seg1 = SegmentCapsuleTransform(cap1, t1);
	struct segment seg2 = SegmentCapsuleTransform(cap2, t2);

	f32 dist = 0.0f;
	if (SegmentDistanceSquared(c1, c2, &seg1, &seg2) > r_sum*r_sum)
	{
        vec3 p0, p1;
		Vec3Sub(p0, c2, c1);
		Vec3Normalize(p1, p0);
		Vec3TranslateScaled(c1, p1, cap1->radius);
		Vec3TranslateScaled(c2, p1, -cap2->radius);
		dist = f32_sqrt(Vec3DistanceSquared(c1, c2));
	}

	return dist;
}

f32 c_HullSphereDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert(s1->type == C_SHAPE_CONVEX_HULL);
	ds_Assert(s2->type == C_SHAPE_SPHERE);

	struct gjk_Input g1 = { .v = s1->hull.v, .v_count = s1->hull.v_count, };
	Vec3Copy(g1.pos, t1->position);
	Mat3Quat(g1.rot, t1->rotation);

	vec3 n = VEC3_ZERO;
	struct gjk_Input g2 = { .v = &n, .v_count = 1, };
	Vec3Copy(g2.pos, t2->position);
	Mat3Identity(g2.rot);

    struct gjk_Simplex simplex;
	f32 dist_sq = gjk_DistanceSquared(c1, c2, &simplex, &g1, &g2);
	const f32 r_sum = s2->sphere.radius;

	if (dist_sq <= r_sum*r_sum)
	{  
		dist_sq = 0.0f;
	}
	else
	{
		Vec3Sub(n, c2, c1);
		Vec3ScaleSelf(n, 1.0f / Vec3Length(n));
		Vec3TranslateScaled(c2, n, -s2->sphere.radius);
	}

	return f32_sqrt(dist_sq);
}

f32 c_HullCapsuleDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert(s1->type == C_SHAPE_CONVEX_HULL);
	ds_Assert(s2->type == C_SHAPE_CAPSULE);

	struct gjk_Input g1 = { .v = s1->hull.v, .v_count = s1->hull.v_count, };
	Vec3Copy(g1.pos, t1->position);
	Mat3Quat(g1.rot, t1->rotation);

	vec3 segment[2];
	Vec3Set(segment[0], 0.0f,  s2->capsule.half_height, 0.0f);
	Vec3Set(segment[1], 0.0f, -s2->capsule.half_height, 0.0f);
	struct gjk_Input g2 = { .v = segment, .v_count = 2, };
	Vec3Copy(g2.pos, t2->position);
	Mat3Quat(g1.rot, t2->rotation);

    struct gjk_Simplex simplex;
	f32 dist_sq = gjk_DistanceSquared(c1, c2, &simplex, &g1, &g2);
	const f32 r_sum = s2->capsule.radius;

	if (dist_sq <= r_sum*r_sum)
	{
		dist_sq = 0.0f;
	}
	else
	{
		vec3 n;
		Vec3Sub(n, c2, c1);
		Vec3ScaleSelf(n, 1.0f / Vec3Length(n));
		Vec3TranslateScaled(c2, n, -s2->capsule.radius);
	}

	return f32_sqrt(dist_sq);
}

f32 c_HullDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_Assert (s1->type == C_SHAPE_CONVEX_HULL);
	ds_Assert (s2->type == C_SHAPE_CONVEX_HULL);

	struct gjk_Input g1 = { .v = s1->hull.v, .v_count = s1->hull.v_count, };
	Vec3Copy(g1.pos, t1->position);
	Mat3Quat(g1.rot, t1->rotation);

	struct gjk_Input g2 = { .v = s2->hull.v, .v_count = s2->hull.v_count, };
	Vec3Copy(g2.pos, t2->position);
	Mat3Quat(g2.rot, t2->rotation);

    struct gjk_Simplex simplex;
	const f32 dist_sq = gjk_DistanceSquared(c1, c2, &simplex, &g1, &g2);
	return f32_sqrt(dist_sq);
}

f32 c_TriMeshBvhSphereDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_AssertString(0, "implement");
	return 0.0f;
}

f32 c_TriMeshBvhCapsuleDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_AssertString(0, "implement");
	return 0.0f;
}

f32 c_TriMeshBvhHullDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2)
{
	ds_AssertString(0, "implement");
	return 0.0f;
}

/********************************** CONTACT MANIFOLD METHODS **********************************/

struct c_ContactResult c_SphereContact(struct arena *frame, const struct c_ContactResult *not_used, const struct c_Shape *s[2], const ds_Transform t[2], const u32 ref)
{
	ds_Assert(s[0]->type == C_SHAPE_SPHERE);
	ds_Assert(s[1]->type == C_SHAPE_SPHERE);

    struct c_ContactResult result = { 0 };
    const u32 inc = 1 - ref;

	const f32 r_sum = s[0]->sphere.radius + s[1]->sphere.radius;
	const f32 dist_sq = Vec3DistanceSquared(t[0].position, t[1].position);
	if (dist_sq <= r_sum*r_sum)
	{
        result.manifold_count = 1;
        result.manifold = ArenaPushPacked(frame, sizeof(struct c_Manifold));
        struct c_Manifold *manifold = result.manifold;

		manifold->v_count = 1;
		if (dist_sq <= COLLISION_POINT_DIST_SQ)
		{
			//TODO(Degenerate): spheres have same center => normal returned should depend on the context.
			Vec3Set(manifold->n, 0.0f, 1.0f, 0.0f);
		}
		else
		{
			Vec3Sub(manifold->n, t[inc].position, t[ref].position);
			Vec3ScaleSelf(manifold->n, 1.0f/Vec3Length(manifold->n));
		}

		vec3 c2;
        Vec3Copy(manifold->v[0], t[ref].position);
		Vec3Copy(c2, t[inc].position);
		Vec3TranslateScaled(manifold->v[0], manifold->n, s[ref]->sphere.radius);
		Vec3TranslateScaled(c2, manifold->n, -s[inc]->sphere.radius);
		manifold->depth[0] = Vec3Dot(manifold->v[0], manifold->n) - Vec3Dot(c2, manifold->n);
	}

    return result;
}

struct c_ContactResult c_CapsuleSphereContact(struct arena *frame, const struct c_ContactResult *not_used, const struct c_Shape *s[2], const ds_Transform t[2], const u32 ref)
{
	ds_Assert(s[0]->type == C_SHAPE_CAPSULE);
	ds_Assert(s[1]->type == C_SHAPE_SPHERE);

    struct c_ContactResult result = { 0 };

	const struct capsule *cap = &s[0]->capsule;
	const f32 r_sum = cap->radius + s[1]->sphere.radius;

	vec3 c[2], s_p1, s_p2, diff;

    Vec3Set(s_p1, 0.0f, cap->half_height, 0.0f);
    QuatVec3RotateSelf(s_p1, t[0].rotation);
	Vec3Negate(s_p2, s_p1);
	struct segment seg = SegmentConstruct(s_p1, s_p2);

	Vec3Sub(c[1], t[1].position, t[0].position);
	const f32 dist_sq = SegmentPointDistanceSquared(c[0], &seg, c[1]);

	if (dist_sq <= r_sum*r_sum)
	{
        result.manifold_count = 1;
        result.manifold = ArenaPushPacked(frame, sizeof(struct c_Manifold));
        struct c_Manifold *manifold = result.manifold;

		manifold->v_count = 1;
		if (dist_sq <= COLLISION_POINT_DIST_SQ)
		{
			//TODO Degerate case: normal should be context dependent
			Vec3CreateBasis(manifold->n, diff, seg.dir);
            Vec3Copy(manifold->v[0], t[0].position);
			manifold->depth[0] = r_sum;
            ds_AssertString(0, "Implement Degenerate CapsuleSphere contact case properly");
		}
		else
		{
            /* segment point -> sphere center local to capsule position */
			Vec3Sub(diff, c[1], c[0]);
			Vec3ScaleSelf(diff, 1.0f/Vec3Length(diff));
			Vec3TranslateScaled(c[0], diff, cap->radius);
			Vec3TranslateScaled(c[1], diff, -s[1]->sphere.radius);

			manifold->depth[0] = Vec3Dot(c[0], diff) - Vec3Dot(c[1], diff);
            Vec3Add(manifold->v[0], t[0].position, c[ref]);
            Vec3Scale(manifold->n, diff, 1.0f - 2.0f*((f32) ref));
		}	
	}

    return result;
}

struct c_ContactResult c_CapsuleContact(struct arena *frame, const struct c_ContactResult *not_used, const struct c_Shape *s[2], const ds_Transform t[2], const u32 ref)
{
	ds_Assert(s[0]->type == C_SHAPE_CAPSULE);
	ds_Assert(s[1]->type == C_SHAPE_CAPSULE);

    struct c_ContactResult result = { 0 };
    const u32 inc = 1 - ref;

    const struct capsule *cap[2] =
    {
	    &s[0]->capsule,
	    &s[1]->capsule,
    };

    const struct segment seg[2] = 
    { 
        SegmentCapsuleTransform(cap[0], t + 0), 
        SegmentCapsuleTransform(cap[1], t + 1),
    };

	vec3 c[2], p0, p1; /* line points */
	const f32 r_sum = cap[0]->radius + cap[1]->radius;
	const f32 dist_sq = SegmentDistanceSquared(c[0], c[1], &seg[0], &seg[1]);
	if (dist_sq <= r_sum*r_sum)
	{
        result.manifold_count = 1;
        result.manifold = ArenaPushPacked(frame, sizeof(struct c_Manifold));
        struct c_Manifold *manifold = result.manifold;

		vec3 cross;
		Vec3Cross(cross, seg[ref].dir, seg[inc].dir);
		const f32 cross_dist_sq = Vec3LengthSquared(cross);
		if (dist_sq <= COLLISION_POINT_DIST_SQ)
		{
			/* Degenerate Case 1: Parallel capsules,*/
			manifold->v_count = 1;
			manifold->depth[0] = r_sum;
			Vec3Copy(manifold->v[0], t[ref].position);
			if (cross_dist_sq <= COLLISION_POINT_DIST_SQ)
			{
				//TODO Normal should be context dependent
                Vec3CreateBasis(manifold->n, p0, seg[ref].dir);
			}
			/* Degenerate Case 2: Non-Parallel capsules, */
			else
			{
				manifold->v_count = 1;
				Vec3Normalize(manifold->n, cross);
			}
            Vec3TranslateScaled(manifold->v[0], manifold->n, cap[ref]->radius);
		}
		else
		{   
            Vec3Sub(manifold->n, c[inc], c[ref]);
            Vec3ScaleSelf(manifold->n, 1.0f/Vec3Length(manifold->n));
            Vec3TranslateScaled(c[ref], manifold->n, cap[ref]->radius);
            Vec3TranslateScaled(c[inc], manifold->n, -cap[inc]->radius);
            manifold->depth[0] = Vec3Dot(c[ref], manifold->n) - Vec3Dot(c[inc], manifold->n);
            if (cross_dist_sq <= COLLISION_POINT_DIST_SQ)
			{
                const f32 t[2] =
                {
				    SegmentPointClosestBcParameter(&seg[ref], seg[inc].p[0]),
				    SegmentPointClosestBcParameter(&seg[ref], seg[inc].p[1]),
                };

				if (t[0] != t[1])
				{
					manifold->v_count = 2;
					manifold->depth[1] = manifold->depth[0];
					SegmentBc(manifold->v[0], &seg[ref], t[0]);
					SegmentBc(manifold->v[1], &seg[ref], t[1]);
                    Vec3TranslateScaled(manifold->v[0], manifold->n, cap[ref]->radius);
                    Vec3TranslateScaled(manifold->v[1], manifold->n, cap[ref]->radius);
				}
				/* end-point contact point */
				else
				{
					manifold->v_count = 1;
					Vec3Copy(manifold->v[0], c[ref]);
				}
			}
			else
			{
				manifold->v_count = 1;
                Vec3Copy(manifold->v[0], c[ref]);
			}
		}
	}

    return result;
}

static void c_HullSphereShallowManifold(struct c_Manifold *manifold, const f32 radius, const vec3 c[2], const u32 ref)
{
    manifold->v_count = 1;
    Vec3Sub(manifold->n, c[1], c[0]);
    Vec3ScaleSelf(manifold->n, 1.0f / Vec3Length(manifold->n));
    manifold->depth[0] = f32_max(0.0f, radius - (Vec3Dot(c[1], manifold->n) - Vec3Dot(c[0], manifold->n)));
    Vec3Copy(manifold->v[0], c[ref]);
    if (ref == 1)
    {
        Vec3ScaleSelf(manifold->n, -1.0f);
    	Vec3TranslateScaled(manifold->v[0], manifold->n, radius);
    }   
}

struct c_ContactResult c_HullSphereContact(struct arena *frame, const struct c_ContactResult *not_used, const struct c_Shape *s[2], const ds_Transform t[2], const u32 ref)
{
	ds_Assert(s[0]->type == C_SHAPE_CONVEX_HULL);
	ds_Assert(s[1]->type == C_SHAPE_SPHERE);

    struct c_ContactResult result = { 0 };
    const u32 inc = 1 - ref;

	struct gjk_Input g1 = { .v = s[0]->hull.v, .v_count = s[0]->hull.v_count, };
	Vec3Copy(g1.pos, t[0].position);
	Mat3Quat(g1.rot, t[0].rotation);

	vec3 zero = VEC3_ZERO;
	struct gjk_Input g2 = { .v = &zero, .v_count = 1, };
	Vec3Copy(g2.pos, t[1].position);
	Mat3Identity(g2.rot);

	vec3 c[2];
    struct gjk_Simplex simplex;
	const f32 dist_sq = gjk_DistanceSquared(c[0], c[1], &simplex, &g1, &g2);
	const f32 r_sum = s[1]->sphere.radius;

	/* Deep Penetration */
	if (dist_sq <= 0.0f)
	{
        result.manifold_count = 1;
        result.manifold = ArenaPushPacked(frame, sizeof(struct c_Manifold));
        struct c_Manifold *manifold = result.manifold;

		manifold->v_count = 1;
		vec3 n;	
		const struct dcel *h = &s[0]->hull;
		f32 min_depth = F32_INFINITY;
		vec3 diff;
		vec3 p;
		for (u32 fi = 0; fi < h->f_count; ++fi)
		{
			DcelFaceNormal(p, h, g1.rot, fi);
			Mat3VecMul(p, g1.rot, h->v[h->e[h->f[fi].first].origin]);
			Vec3Translate(p, t[0].position);
			Vec3Sub(diff, t[1].position, p);
			const f32 depth = f32_max(0.0f, -Vec3Dot(n, diff));
			if (depth < min_depth)
			{
				Vec3Copy(manifold->n, n);
				min_depth = depth;
			}
		}

        COLLISION_DEBUG_ADD_SEGMENT(SegmentConstruct(c[0], c[1]), Vec4Inline(0.1f, 0.6f, 0.9f, 1.0f));
            
        //ds_Assert(min_depth > 0.0f);
		manifold->depth[0] = min_depth + s[1]->sphere.radius;
		Vec3Copy(manifold->v[0], t[1].position);
        if (ref == 0)
        {
		    Vec3TranslateScaled(manifold->v[0], manifold->n, min_depth);
        }
        else
        {
            Vec3ScaleSelf(manifold->n, -1.0f);
		    Vec3TranslateScaled(manifold->v[0], manifold->n, s[1]->sphere.radius);
        }
	}
	/* Shallow Penetration */
	else if (dist_sq <= r_sum*r_sum)
	{
        result.manifold_count = 1;
        result.manifold = ArenaPushPacked(frame, sizeof(struct c_Manifold));
        struct c_Manifold *manifold = result.manifold;

        c_HullSphereShallowManifold(manifold, s[1]->sphere.radius, c, ref);
	}

    return result;
}

struct c_ContactResult c_HullCapsuleContact(struct arena *frame, const struct c_ContactResult *not_used, const struct c_Shape *s[2], const ds_Transform t[2], const u32 ref)
{
	ds_Assert(s[0]->type == C_SHAPE_CONVEX_HULL);
	ds_Assert(s[1]->type == C_SHAPE_CAPSULE);

    struct c_ContactResult result = { 0 };
    const u32 inc = 1 - ref;

	const struct dcel *h = &s[0]->hull;
	struct gjk_Input g1 = { .v = h->v, .v_count = h->v_count, };
	Vec3Copy(g1.pos, t[0].position);
	Mat3Quat(g1.rot, t[0].rotation);

	vec3 segment[2];
	Vec3Set(segment[0], 0.0f, s[1]->capsule.half_height, 0.0f);
	Vec3Negate(segment[1], segment[0]);

	struct gjk_Input g2 = { .v = segment, .v_count = 2, };
	Vec3Copy(g2.pos, t[1].position);
	Mat3Quat(g2.rot, t[1].rotation);

	vec3 c[2];
    struct gjk_Simplex simplex;
	const f32 dist_sq = gjk_DistanceSquared(c[0], c[1], &simplex, &g1, &g2);
	if (dist_sq <= s[1]->capsule.radius*s[1]->capsule.radius)
	{
        result.manifold_count = 1;
        result.manifold = ArenaPushPacked(frame, sizeof(struct c_Manifold));
        struct c_Manifold *manifold = result.manifold;

		vec3 p1, p2, tmp;
		Mat3VecMul(p1, g2.rot, g2.v[0]);
		Mat3VecMul(p2, g2.rot, g2.v[1]);
		Vec3Translate(p1, g2.pos);
		Vec3Translate(p2, g2.pos);
		const struct segment cap_s = SegmentConstruct(p1, p2);
		/* Deep Penetration */
		if (dist_sq == 0.0f)
		{
            /* TODO: if p0 is outside, then p1 must be inside, can prob skip check ??? */
		    g2.v_count = 1;
		    const u32 cap_p0_inside = (gjk_DistanceSquared(p1, tmp, &simplex, &g1, &g2) == 0.0f) ? 1 : 0;
		    Vec3Copy(g2.v[0], g2.v[1]);
		    const u32 cap_p1_inside = (gjk_DistanceSquared(p2, tmp, &simplex, &g1, &g2) == 0.0f) ? 1 : 0;

			u32 edge_best = 0; 
			u32 best_index = 0;

			f32 max_signed_depth = -F32_INFINITY;

			for (u32 fi = 0; fi < h->f_count; ++fi)
			{
				struct plane pl = DcelFacePlane(h, g1.rot, t[0].position, fi);

				const f32 d0 = PlanePointSignedDistance(&pl, cap_s.p[0]);
				const f32 d1 = PlanePointSignedDistance(&pl, cap_s.p[1]);
				const f32 d = f32_min(d0, d1);
				if (max_signed_depth < d)
				{
					best_index = fi;
					max_signed_depth = d;
				}
			}

			/* For an edge to define seperating axis, either both or no end-points of the capsule must be inside */
			if (cap_p0_inside == cap_p1_inside)
			{
				for (u32 ei = 0; ei < h->e_count; ++ei)
				{
					struct segment edge_s = DcelEdgeSegment(h, g1.rot, g1.pos, ei);
					
					const f32 d = -f32_sqrt(SegmentDistanceSquared(c[0], c[1], &edge_s, &cap_s));
					if (max_signed_depth < d)
					{
						edge_best = 1;
						best_index = ei;
						max_signed_depth = d;
					}
				}
			}

			if (edge_best)
			{
				manifold->v_count = 1;
			    manifold->depth[0] = f32_max(0.0f, -max_signed_depth);
				struct segment edge_s = DcelEdgeSegment(h, g1.rot, g1.pos, best_index);
				SegmentDistanceSquared(c[0], c[1], &edge_s, &cap_s);
				Vec3Sub(manifold->n, c[ref], c[inc]);
				Vec3ScaleSelf(manifold->n, 1.0f/Vec3Length(manifold->n));
				Vec3Copy(manifold->v[0], c[ref]);
                if (ref == 1)
                {
                    Vec3TranslateScaled(manifold->v[0], manifold->n, s[1]->capsule.radius);
                }
			}
			else
			{
				manifold->v_count = 2;
				struct segment seg = DcelFaceClipSegment(h, g1.rot, g1.pos, best_index, &cap_s);
				const struct plane pl = DcelFacePlane(h, g1.rot, g1.pos, best_index);

				if (cap_p0_inside == 1 && cap_p1_inside == 0)
				{
					Vec3Copy(manifold->v[0], seg.p[0]);
                    if (!PlaneSegmentClip(manifold->v[1], &pl, &seg))
                    {
				        manifold->v_count = 1;
                    }
				}
				else if (cap_p0_inside == 0 && cap_p1_inside == 1)
				{
					Vec3Copy(manifold->v[1], seg.p[1]);
					if (!PlaneSegmentClip(manifold->v[0], &pl, &seg))
                    {
				        manifold->v_count = 1;
					    Vec3Copy(manifold->v[0], seg.p[1]);
                    }
				}
				else
				{
					Vec3Copy(manifold->v[0], seg.p[0]);
					Vec3Copy(manifold->v[1], seg.p[1]);
				}
				
                if (ref == 0)
                {
                    Vec3Copy(manifold->n, pl.normal);
                    manifold->depth[0] = -PlanePointSignedDistance(&pl, manifold->v[0]);
                    manifold->depth[1] = -PlanePointSignedDistance(&pl, manifold->v[1]);
				    Vec3TranslateScaled(manifold->v[0], manifold->n, manifold->depth[0]);
				    Vec3TranslateScaled(manifold->v[1], manifold->n, manifold->depth[1]);
                    manifold->depth[0] += s[1]->capsule.radius;
                    manifold->depth[1] += s[1]->capsule.radius;
                }
                else
                {
                    Vec3Scale(manifold->n, pl.normal, -1.0f);
                    manifold->depth[0] = -PlanePointSignedDistance(&pl, manifold->v[0]) + s[1]->capsule.radius;
                    manifold->depth[1] = -PlanePointSignedDistance(&pl, manifold->v[1]) + s[1]->capsule.radius;
				    Vec3TranslateScaled(manifold->v[0], manifold->n, s[1]->capsule.radius);
				    Vec3TranslateScaled(manifold->v[1], manifold->n, s[1]->capsule.radius);
                }
			}
		}
		/* Shallow Penetration */
		else
		{
			/* (1) compute closest face points for end-point segement */
			vec3 diff;

            u32 best_face = h->f_count;
			const struct dcel *h = &s[0]->hull;
			/* If capsule is not a point, check if it lies parallel on a face */
			if (Vec3Dot(cap_s.dir, cap_s.dir) > COLLISION_POINT_DIST_SQ)
			{
				/* find parallel face with Vec3Dot(face_normal, segment_points) > 0.0f */
				for (u32 fi = 0; fi < h->f_count; ++fi)
				{
				    struct plane pl = DcelFacePlane(h, g1.rot, g1.pos, fi);
                    if (PlaneSegmentParallelCheck(&pl, &cap_s))
					{	
                        const f32 depth = PlanePointSignedDistance(&pl, c[1]);
                        if (f32_abs(dist_sq - depth*depth) <= COLLISION_POINT_DIST_SQ)
                        {
                            Vec3Copy(manifold->n, pl.normal);
                            best_face = fi;
                            break;
                        }
					}
				}
			}

			if (best_face != h->f_count)
			{
				manifold->v_count = 2;
				manifold->depth[0] = s[1]->capsule.radius + Vec3Dot(manifold->n, c[0]) - Vec3Dot(manifold->n, c[1]);
				manifold->depth[1] = manifold->depth[0];
				const struct segment cap_clip = DcelFaceClipSegment(h, g1.rot, g1.pos, best_face, &cap_s);
				Vec3Copy(manifold->v[0], cap_clip.p[0]);
				Vec3Copy(manifold->v[1], cap_clip.p[1]);
                if (ref == 0)
                {
				    Vec3TranslateScaled(manifold->v[0], manifold->n, -s[1]->capsule.radius + manifold->depth[0]);
				    Vec3TranslateScaled(manifold->v[1], manifold->n, -s[1]->capsule.radius + manifold->depth[1]);
                }
                else
                {
                    Vec3ScaleSelf(manifold->n, -1.0f);
				    Vec3TranslateScaled(manifold->v[0], manifold->n, s[1]->capsule.radius);
				    Vec3TranslateScaled(manifold->v[1], manifold->n, s[1]->capsule.radius);
                }
                COLLISION_DEBUG_ADD_SEGMENT(SegmentConstruct(manifold->v[0], manifold->v[1]), Vec4Inline(0.1f, 0.6f, 0.9f, 1.0f));
			}
			else
			{
				manifold->v_count = 1;
				Vec3Sub(manifold->n, c[inc], c[ref]);
				Vec3ScaleSelf(manifold->n, 1.0f / Vec3Length(manifold->n));
				manifold->depth[0] = s[1]->capsule.radius + Vec3Dot(manifold->n, c[ref]) - Vec3Dot(manifold->n, c[inc]);
                Vec3Copy(manifold->v[0], c[ref]);
                if (ref == 1)
                {
                    Vec3TranslateScaled(manifold->v[0], manifold->n, s[1]->capsule.radius);
                }
			}
		}
	}

    return result;
}

typedef struct PolygonClipPoint PolygonClipPoint;
struct PolygonClipPoint
{
    u32             index;   /* index == 0 => vertex, index == 1 => edge */
    u32             feature[2];
    vec3            v;
};

DEFINE_CPOOL_STRUCT(PolygonClipPoint);

/*
 * Sutherland-Hodgman 3D polygon clipping + negative face voronoi region projection: 
 *
 *  Clip polygon against orthogonal side-planes of the input face. Any clip points 
 *  infront of the face is discarded, after which the remaining points are projected
 *  onto the face and stored in cp, with their corresponding positive depth in cp_depth.
 *  
 * clip_stack[0]: Expected to be filled with polygon vertices in counter-clockwise order.
 *            fv: Expected to be filled with face vertices in counter-clockwise order.
 */
static void PolygonCcwClipNegativeFaceAndProject(struct arena *mem, sat_FeatureId **features, vec3ptr *cp, f32 **cp_depth, u32 *cp_deepest, u32 *cp_count, f32 *max_depth, u32 *deepest_feature, const struct dcel *h[2], constvec3ptr v[2], const u32 f[2], const u32 b_f, const vec3 face_normal)
{
    const u32 b_v = 1 - b_f;
	struct dcelFace *ref_face = h[b_f]->f + f[b_f];
	struct dcelFace *inc_face = h[b_v]->f + f[b_v];

    const u32 count = 2*inc_face->count + ref_face->count;
    ds_CPool(PolygonClipPoint) clip_stack[2];
	ds_CPoolAlloc(mem, clip_stack[0], count, NOT_GROWABLE);
	ds_CPoolAlloc(mem, clip_stack[1], count, NOT_GROWABLE);
	vec3ptr ref_v = ArenaPush(mem, ref_face->count * sizeof(vec3));
	*cp = ArenaPush(mem, count*sizeof(vec3));
	*cp_depth = ArenaPush(mem, count*sizeof(f32));
	*features = ArenaPush(mem, count*sizeof(sat_FeatureId));

	for (u32 i = 0; i < ref_face->count; ++i)
	{
		const u32 vi = h[b_f]->e[ref_face->first + i].origin;
		Vec3Copy(ref_v[i], v[b_f][vi]);
	}
    struct plane ref_plane = PlaneConstruct(face_normal, ref_v[0]);

	u32 cur = 0;
	for (u32 i = 0; i < inc_face->count; ++i)
	{
		const u32 vi = h[b_v]->e[inc_face->first + i].origin;
        struct PolygonClipPoint pcp = 
        { 
            .index = 0,
            .feature[0] = sat_FeatureIdConstruct(vi, SAT_FEATURE_TYPE_VERTEX),
            .feature[1] = sat_FeatureIdConstruct(inc_face->first + i, SAT_FEATURE_TYPE_EDGE),
        };
        Vec3Copy(pcp.v, v[b_v][vi]);
		ds_CPoolPushMemcpy(clip_stack[cur], &pcp);
	}

    /* Clip polygon against voronoi face region */
    vec3 tmp, side_plane_direction;
	for (u32 j = 0; j < ref_face->count; ++j)
	{
		const u32 prev = cur;
		cur = 1 - cur;
		ds_CPoolFlush(clip_stack[cur]);

		Vec3Sub(tmp, ref_v[(j+1) % ref_face->count], ref_v[j]);
		Vec3Cross(side_plane_direction, tmp, ref_plane.normal);
		struct plane side_plane = PlaneConstruct(side_plane_direction, ref_v[j]);

		for (u32 i = 0; i < clip_stack[prev].count; ++i)
		{
			const struct segment clip_edge = SegmentConstruct(clip_stack[prev].buf[i].v, clip_stack[prev].buf[(i+1) % clip_stack[prev].count].v);
			const f32 t = PlaneSegmentClipParameter(&side_plane, &clip_edge);
            struct PolygonClipPoint pcp = clip_stack[prev].buf[i];
            SegmentBc(pcp.v, &clip_edge, t);
            pcp.index = 1;

			if (PlanePointBehindCheck(&side_plane, clip_edge.p[0]))
			{
				ds_CPoolPushMemcpy(clip_stack[cur], &clip_stack[prev].buf[i]);
				if (0.0f < t && t < 1.0f)
				{
				    ds_CPoolPushMemcpy(clip_stack[cur], &pcp);
				}
			}
			else if (PlanePointBehindCheck(&side_plane, clip_edge.p[1]))
			{
				ds_CPoolPushMemcpy(clip_stack[cur], &pcp);
			}
		}
	}

	*max_depth = -F32_INFINITY;
    *deepest_feature = 0;
    *cp_deepest = 0;
    *cp_count = 0;
	for (u32 i = 0; i < clip_stack[cur].count; ++i)
	{
        const struct PolygonClipPoint *pcp = clip_stack[cur].buf + i;
		Vec3Copy((*cp)[*cp_count], pcp->v);
        (*cp_depth)[ *cp_count ] = -PlanePointSignedDistance(&ref_plane, (*cp)[ *cp_count ]);

		if ((*cp_depth)[*cp_count] >= 0.0f)
		{
            (*features)[*cp_count] = pcp->feature[ pcp->index ];
			Vec3TranslateScaled((*cp)[*cp_count], ref_plane.normal, (*cp_depth)[*cp_count]);
			if (*max_depth < (*cp_depth)[ *cp_count ])
			{
                *max_depth = (*cp_depth)[ *cp_count ];
				*cp_deepest = *cp_count;
                *deepest_feature = pcp->feature[ pcp->index ];
			}
			*cp_count += 1;
		}
	}

    ds_Assert(*cp_count <= count); 
}

static u32 PolygonCcwContact(struct c_Manifold *cm, sat_FeatureId cm_features[4], const vec3 cm_n, const sat_FeatureId *features, constvec3ptr cp, const f32 *depth, const u32 cp_deepest, const u32 cp_count)
{
    vec3 n, tmp1, tmp2;

	u32 collision = 1;
	Vec3Copy(cm->n, cm_n);
	switch (cp_count)
	{
		case 0:
		{
            cm->v_count = 0;
			collision = 0;
		} break;

		case 1:
		{
			cm->v_count = 1;
			Vec3Copy(cm->v[0], cp[0]);
			cm->depth[0] = depth[0];
            cm_features[0] = features[0];
		} break;

		case 2:
		{
			cm->v_count = 2;
			Vec3Copy(cm->v[0], cp[0]);
			Vec3Copy(cm->v[1], cp[1]);
			cm->depth[0] = depth[0];
			cm->depth[1] = depth[1];
            cm_features[0] = features[0];
            cm_features[1] = features[1];
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
                cm_features[0] = features[0];
                cm_features[1] = features[1];
                cm_features[2] = features[2];
			}
			else
			{
				Vec3Copy(cm->v[0], cp[0]);
				Vec3Copy(cm->v[2], cp[1]);
				Vec3Copy(cm->v[1], cp[2]);
				cm->depth[0] = depth[0];
				cm->depth[2] = depth[1];
				cm->depth[1] = depth[2];
                cm_features[0] = features[0];
                cm_features[2] = features[1];
                cm_features[1] = features[2];
			}
		} break;

		default:
		{
			/* (1) First point is deepest point */
			cm->v_count = 4;
			Vec3Copy(cm->v[0], cp[cp_deepest]);
			cm->depth[0] = depth[cp_deepest];
            cm_features[0] = features[cp_deepest];
			/* (2) Third point is point furthest away from deepest point */
			f32 max_dist = 0.0f;
			u32 max_i = (cp_deepest + 2) % cp_count;
			for (u32 i = 0; i < cp_count; ++i)
			{
				if (i == (cp_deepest + 1) % cp_count || (i+1) % cp_count == cp_deepest)
				{
					continue;
				}

				const f32 dist = Vec3DistanceSquared(cp[cp_deepest], cp[i]);
				if (max_dist < dist)
				{
					max_dist = dist;
					max_i = i;
				}
			}
			Vec3Copy(cm->v[2], cp[max_i]);
			cm->depth[2] = depth[max_i];
            cm_features[2] = features[max_i];

			/* (3, 4) Second point and forth is point that gives largest (in magnitude) 
			 * areas with the previous points on each side of the previous segment 
			 */
			u32 max_pos_i = (cp_deepest + 1) % cp_count;
			u32 max_neg_i = (max_i + 1) % cp_count;
			f32 max_neg = 0.0f;
			f32 max_pos = 0.0f;

			for (u32 i = (cp_deepest + 1) % cp_count; i != max_i; i = (i+1) % cp_count)
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

			for (u32 i = (max_i + 1) % cp_count; i != cp_deepest; i = (i+1) % cp_count)
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

			ds_Assert(cp_deepest != max_i);
			ds_Assert(cp_deepest != max_pos_i);
			ds_Assert(cp_deepest != max_neg_i);
			ds_Assert(max_i != max_pos_i);
			ds_Assert(max_i != max_neg_i);
			ds_Assert(max_pos_i != max_neg_i);
	
			vec3 dir;
			TriCcwNormalDirection(dir, cm->v[0], cp[max_pos_i], cm->v[2]);
			if (Vec3Dot(dir, cm->n) < 0.0f)
			{
				Vec3Copy(cm->v[3], cp[max_pos_i]);
				Vec3Copy(cm->v[1], cp[max_neg_i]);
				cm->depth[3] = depth[max_pos_i];
				cm->depth[1] = depth[max_neg_i];
                cm_features[3] = features[max_pos_i];
                cm_features[1] = features[max_neg_i];
			}
			else
			{
				Vec3Copy(cm->v[3], cp[max_neg_i]);
				Vec3Copy(cm->v[1], cp[max_pos_i]);
				cm->depth[3] = depth[max_neg_i];
				cm->depth[1] = depth[max_pos_i];
                cm_features[3] = features[max_neg_i];
                cm_features[1] = features[max_pos_i];
			}
		} break;
	}

    return collision;
}

struct sat_FaceQuery
{
	vec3 normal;
	u32 fi;
	f32 depth;
};

struct sat_EdgeQuery
{
	struct segment s1;
	struct segment s2;
	u32	e1;
	u32	e2;
	vec3 normal;
	f32 depth;
};

static u32 HullFaceContact(struct arena *mem_tmp, struct c_Manifold *cm, sat_FeatureId cm_features[4], f32 *max_depth, sat_FeatureId *max_feature, const struct sat_FaceQuery *query, const struct dcel *h[2], constvec3ptr v[2], const u32 b_f, const u32 ref)
{
	vec3 tmp1, tmp2, n;

	/* (1) determine incident_face */
    const u32 b_v = 1 - b_f;
    u32 f[2];
    f[b_f] = query->fi;
    f[b_v] = 0;
	f32 min_cos_sq = 1.0f;
    //f32 min_dot = 1.0f;
	for (u32 fi = 0; fi < h[b_v]->f_count; ++fi)
	{
		const u32 i0  = h[b_v]->e[h[b_v]->f[fi].first + 0].origin;
		const u32 i1  = h[b_v]->e[h[b_v]->f[fi].first + 1].origin;
		const u32 i2  = h[b_v]->e[h[b_v]->f[fi].first + 2].origin;
        TriCcwNormalDirection(n, v[b_v][i0], v[b_v][i1], v[b_v][i2]);

        /* Dot(R_n, I_n_dir) * Dot(R_n, I_n_dir) / Dot(I_n_dir, I_n_dir) = cos(theta)^2 */
        const f32 dot_RI = Vec3Dot(n, query->normal);
        const f32 signed_cos_sq = f32_abs(dot_RI) * dot_RI / Vec3Dot(n, n);
		if (signed_cos_sq < min_cos_sq)
		{
            min_cos_sq = signed_cos_sq;
			f[b_v] = fi;
		}
	}
	
	u32 deepest_point = 0;
	u32 cp_count = 0;
    vec3ptr cp = NULL;
    f32 *depth = NULL;
    sat_FeatureId *features;
    PolygonCcwClipNegativeFaceAndProject(mem_tmp, &features, &cp, &depth, &deepest_point, &cp_count, max_depth, max_feature, h, v, f, b_f, query->normal);

	//for (u32 i = 0; i < cp_count; ++i)
	//{
	//	COLLISION_DEBUG_ADD_SEGMENT(SegmentConstruct(cp[i], cp[(i+1) % cp_count]), Vec4Inline(0.8f, 0.6f, 0.1f, 1.0f));
	//}

    u32 collision;
    if (b_f == ref)
    {
        collision = PolygonCcwContact(cm, cm_features, query->normal, features, cp, depth, deepest_point, cp_count);
    }
    else
    {
        vec3 cm_n;
        Vec3Scale(cm_n, query->normal, -1.0f);
        collision = PolygonCcwContact(cm, cm_features, cm_n, features, cp, depth, deepest_point, cp_count);
        Vec3TranslateScaled(cm->v[0], cm->n, cm->depth[0]);
        Vec3TranslateScaled(cm->v[1], cm->n, cm->depth[1]);
        Vec3TranslateScaled(cm->v[2], cm->n, cm->depth[2]);
        Vec3TranslateScaled(cm->v[3], cm->n, cm->depth[3]);
    }

    return collision;
}

static u32 HullContactFVSeparation(struct sat_FaceQuery *query, const struct dcel *h1, constvec3ptr v1_world, const struct dcel *h2, constvec3ptr v2_world)
{
	for (u32 fi = 0; fi < h1->f_count; ++fi)
	{
		const u32 f_v0 = h1->e[h1->f[fi].first + 0].origin;
		const u32 f_v1 = h1->e[h1->f[fi].first + 1].origin;
		const u32 f_v2 = h1->e[h1->f[fi].first + 2].origin;
		const struct plane sep_plane = PlaneConstructNormalizedFromCcwTriangle(v1_world[f_v0], v1_world[f_v1], v1_world[f_v2]);
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

static u32 EEIsMinkowskiFace(const vec3 n1_1, const vec3 n1_2, const vec3 n2_1, const vec3 n2_2, const vec3 arc_n1, const vec3 arc_n2)
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

static void HullContactEECheck(struct sat_EdgeQuery *query, const struct dcel *h1, constvec3ptr v1_world, const u32 e1_1, const struct dcel *h2, constvec3ptr v2_world, const u32 e2_1, const vec3 h1_world_center, const vec3 n1_1, const vec3 n1_2, const vec3 n2_1, const vec3 n2_2, const struct segment *s1, const struct segment *s2, const f32 s1_len_sq)
{
    /* 
	 * test if A, -B edges intersect on gauss map, only if they do, 
	 * they are a candidate for collision
	 */
    if (EEIsMinkowskiFace(n1_1, n1_2, n2_1, n2_2, s1->dir, s2->dir))
    {
        /* Inlined SegmentParallelCheck */
    	const f32 d2d2 = Vec3Dot(s2->dir, s2->dir);
    	const f32 d1d2 = Vec3Dot(s1->dir, s2->dir);
        const f32 d1d1_d2d2 = s1_len_sq*d2d2;
        /* 0.5 degrees cut-off */
        const f32 eps = 7.62e-5;    
    	/* Skip parallel edge pairs  */
    	if (d1d1_d2d2 - d1d2*d1d2 >= eps*d1d1_d2d2) 
    	{
            vec3 p1, p2;
    		Vec3Cross(p1, s1->dir, s2->dir);
    		Vec3ScaleSelf(p1, 1.0f / Vec3Length(p1));
    		Vec3Sub(p2, s1->p[0], h1_world_center);
    		/* plane normal points from A -> B */
    		if (Vec3Dot(p1, p2) < 0.0f)
    		{
    			Vec3NegateSelf(p1);
    		}
    		
    		/* check segmente-segment distance interval signed plane distance, > 0.0f => we have found a seperating axis */
    		Vec3Sub(p2, s2->p[0], s1->p[0]);
    		const f32 dist = Vec3Dot(p1, p2);

    		if (query->depth < dist)
    		{
    			query->depth = dist;
    			Vec3Copy(query->normal, p1);
    			query->s1 = *s1;
    			query->s2 = *s2;
    			query->e1 = e1_1;
    			query->e2 = e2_1;
    		}
    	}
    }
}
static void HullContactEECheckRecompute(struct sat_EdgeQuery *query, const struct dcel *h1, constvec3ptr v1_world, const u32 e1_1, const struct dcel *h2, constvec3ptr v2_world, const u32 e2_1, const vec3 h1_world_center)
{
	vec3 n1_1, n1_2, n2_1, n2_2, e1, e2;
	const u32 e1_2 = h1->e[e1_1].twin;
	const u32 e2_2 = h2->e[e2_1].twin;

	const u32 f1_1 = h1->e[e1_1].face_ccw;
	const u32 f1_2 = h1->e[e1_2].face_ccw;
	const u32 f2_1 = h2->e[e2_1].face_ccw;
	const u32 f2_2 = h2->e[e2_2].face_ccw;
	TriCcwNormalDirection(n1_1, v1_world[h1->e[h1->f[f1_1].first + 0].origin],  v1_world[h1->e[h1->f[f1_1].first + 1].origin], v1_world[h1->e[h1->f[f1_1].first + 2].origin]);
	TriCcwNormalDirection(n1_2, v1_world[h1->e[h1->f[f1_2].first + 0].origin],  v1_world[h1->e[h1->f[f1_2].first + 1].origin], v1_world[h1->e[h1->f[f1_2].first + 2].origin]);
	TriCcwNormalDirection(n2_1, v2_world[h2->e[h2->f[f2_1].first + 0].origin],  v2_world[h2->e[h2->f[f2_1].first + 1].origin], v2_world[h2->e[h2->f[f2_1].first + 2].origin]);
	TriCcwNormalDirection(n2_2, v2_world[h2->e[h2->f[f2_2].first + 0].origin],  v2_world[h2->e[h2->f[f2_2].first + 1].origin], v2_world[h2->e[h2->f[f2_2].first + 2].origin]);

	///* we are working with minkowski difference A - B, so gauss map of B is (-B). n2_1, n2_2 cross product stays the same. */
	Vec3NegateSelf(n2_1);	
	Vec3NegateSelf(n2_2);

	const struct segment s1 = SegmentConstruct(v1_world[h1->e[e1_1].origin], v1_world[h1->e[e1_2].origin]);
	const struct segment s2 = SegmentConstruct(v2_world[h2->e[e2_1].origin], v2_world[h2->e[e2_2].origin]);

    HullContactEECheck(query, h1, v1_world, e1_1, h2, v2_world, e2_1, h1_world_center, n1_1, n1_2, n2_1, n2_2, &s1, &s2, Vec3Dot(s1.dir, s1.dir));
}


/*
 * For full algorithm: see GDC talk by Dirk Gregorius - 
 * 	Physics for Game Programmers: The Separating Axis Test between Convex Polyhedra
 */
static u32 HullContactEESeparation(struct sat_EdgeQuery *query, const struct dcel *h1, constvec3ptr v1_world, const struct dcel *h2, constvec3ptr v2_world, const vec3 h1_world_center)
{
	vec3 n1_1, n1_2, n2_1, n2_2;
    for (u32 f1 = 0; f1 < h1->f_count; ++f1)
    {
        TriCcwNormalDirection(n1_1, v1_world[ h1->e[h1->f[f1].first + 0].origin ] , v1_world[ h1->e[h1->f[f1].first + 1].origin ] , v1_world[ h1->e[h1->f[f1].first + 2].origin ]);
        const u32 end_1 = h1->f[f1].first + h1->f[f1].count;
        for (u32 e1_1 = h1->f[f1].first; e1_1 < end_1; ++e1_1)
	    {
	        const u32 e1_2 = h1->e[e1_1].twin;
	    	if (e1_2 < e1_1) { continue; }

	        const struct segment s1 = SegmentConstruct(v1_world[h1->e[e1_1].origin], v1_world[h1->e[e1_2].origin]);
            const f32 s1s1_d = Vec3Dot(s1.dir, s1.dir);
            const u32 f1_2 = h1->e[e1_2].face_ccw;
            TriCcwNormalDirection(n1_2 , v1_world[ h1->e[h1->f[f1_2].first + 0].origin ] , v1_world[ h1->e[h1->f[f1_2].first + 1].origin ] , v1_world[ h1->e[h1->f[f1_2].first + 2].origin ]);
            
            for (u32 f2 = 0; f2 < h2->f_count; ++f2)
            {
                TriCcwNormalDirection(n2_1 , v2_world[ h2->e[h2->f[f2].first + 0].origin ] , v2_world[ h2->e[h2->f[f2].first + 2].origin ] , v2_world[ h2->e[h2->f[f2].first + 1].origin ]);
                const u32 end_2 = h2->f[f2].first + h2->f[f2].count;
            	for (u32 e2_1 = h2->f[f2].first; e2_1 < end_2; ++e2_1) 
	        	{
	                const u32 e2_2 = h2->e[e2_1].twin;
	        		if (e2_2 < e2_1) { continue; }
    
	                const struct segment s2 = SegmentConstruct(v2_world[h2->e[e2_1].origin], v2_world[h2->e[e2_2].origin]);
                    const u32 f2_2 = h2->e[e2_2].face_ccw;
                    TriCcwNormalDirection(n2_2 , v2_world[ h2->e[h2->f[f2_2].first + 0].origin ] , v2_world[ h2->e[h2->f[f2_2].first + 2].origin ] , v2_world[ h2->e[h2->f[f2_2].first + 1].origin ]);
	        		HullContactEECheck(query, h1, v1_world, e1_1, h2, v2_world, e2_1, h1_world_center, n1_1, n1_2, n2_1, n2_2, &s1, &s2, s1s1_d);
	        		if (query->depth > 0.0f)
	        		{
	        			return 1;
	        		}
	        	}
	        }
        }
    }

	return 0;
}

void sat_EdgeQueryCollisionResult(struct c_Manifold *manifold, struct c_SatCache *sat_cache, const struct sat_EdgeQuery *query, const u32 ref)
{
	vec3 c[2];
	SegmentDistanceSquared(c[0], c[1], &query->s1, &query->s2);

	manifold->v_count = 1;
	manifold->depth[0] = -query->depth;
	Vec3Copy(manifold->v[0], c[ref]);
    (ref == 0)
        ? Vec3Copy(manifold->n, query->normal)
        : Vec3Scale(manifold->n, query->normal, -1.0f);

	sat_cache->type = SAT_CACHE_CONTACT_EE;
    sat_cache->depth = manifold->depth[0];
    Vec3Copy(sat_cache->normal, manifold->n);
	sat_cache->feature[0] = sat_FeatureIdConstruct(query->e1, SAT_FEATURE_TYPE_EDGE);
	sat_cache->feature[1] = sat_FeatureIdConstruct(query->e2, SAT_FEATURE_TYPE_EDGE);
}

/*
 * For the Algorithm, see
 * 	(Game Physics Pearls, Chapter 4)
 *	(GDC 2013 Dirk Gregorius, https://www.gdcvault.com/play/1017646/Physics-for-Game-Programmers-The)
 */
struct c_ContactResult c_HullContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s[2], const ds_Transform t[2], const u32 ref)
{
    //ProfZone;
	ds_Assert(s[0]->type == C_SHAPE_CONVEX_HULL);
	ds_Assert(s[1]->type == C_SHAPE_CONVEX_HULL);

    u32 colliding = 0;
    struct c_ContactResult result = { 0 };
    result.cache_count = 1;
    result.cache = ArenaPushPacked(frame, sizeof(struct c_SatCache));
    result.manifold_count = 1;
    result.manifold = ArenaPushPacked(frame, sizeof(struct c_Manifold));

    struct arena *mem_tmp = ArenaPushScratch();

    mat3 rot[2];
	Mat3Quat(rot[0], t[0].rotation);
	Mat3Quat(rot[1], t[1].rotation);
	
    const struct dcel *h[2] = { &s[0]->hull, &s[1]->hull };
    vec3ptr v_world[2] = { ArenaPush(mem_tmp, h[0]->v_count * sizeof(vec3)), ArenaPush(mem_tmp, h[1]->v_count * sizeof(vec3)) };

	for (u32 i = 0; i < h[0]->v_count; ++i)
	{
		Mat3VecMul(v_world[0][i], rot[0], h[0]->v[i]);
		Vec3Translate(v_world[0][i], t[0].position);
	}

	for (u32 i = 0; i < h[1]->v_count; ++i)
	{
		Mat3VecMul(v_world[1][i], rot[1], h[1]->v[i]);
		Vec3Translate(v_world[1][i], t[1].position);
	}

    if (cached_result->cache_count)
    {
        const struct c_SatCache *cache = cached_result->cache;
        switch (cache->type)
        {
            case SAT_CACHE_SEPARATION:
	        {
	        	vec3 support1, support2, tmp;
	        	Vec3Negate(tmp, cache->normal);

	        	VertexSupport(support1, cache->normal, v_world[0], h[0]->v_count);
	        	VertexSupport(support2, tmp, v_world[1], h[1]->v_count);

	        	const f32 dot1 = Vec3Dot(support1, cache->normal);
	        	const f32 dot2 = Vec3Dot(support2, cache->normal);
	        	const f32 separation = dot2 - dot1;
	        	if (separation > 0.0f)
	        	{
                    result.cache->type = SAT_CACHE_SEPARATION;
	        		result.cache->depth = separation;
                    Vec3Copy(result.cache->normal, cache->normal);
                    goto sat_cleanup;
	        	}
	        } break;
             
            case SAT_CACHE_CONTACT_EE:
	        {
	            struct sat_EdgeQuery e_query = { .depth = -F32_INFINITY };
	        	HullContactEECheckRecompute(&e_query, h[0], v_world[0], sat_FeatureIdIndex(cache->feature[0]), h[1], v_world[1], sat_FeatureIdIndex(cache->feature[1]), t[0].position);
	        	sat_EdgeQueryCollisionResult(result.manifold, result.cache, &e_query, ref);

                colliding = (e_query.depth < 0.0f);
                if (!colliding
                        || f32_abs(e_query.depth - result.cache->depth) >= g_numerics_config->manifold_cache_depth_max_diff_allowed
                        || Vec3Dot(result.cache->normal, cache->normal) < g_numerics_config->manifold_cache_normal_parallel_check_eps)
                {
                    break;
                }

                goto sat_cleanup;
	        } break;

            case SAT_CACHE_CONTACT_FV:
	        {
                /* b_f = body with reference/contact face, b_v = incident body with penetrating vertices */
                const u32 b_f = sat_FeatureIdFaceCheck(cache->feature[1]);
                const u32 b_v = 1 - b_f;
                const u32 face = sat_FeatureIdIndex(cache->feature[b_f]);
	        	DcelFaceNormal(result.cache->normal, h[b_f], rot[b_f], face);
                if (Vec3Dot(result.cache->normal, cache->normal) < g_numerics_config->manifold_cache_normal_parallel_check_eps) 
                { 
                    break; 
                }

                struct sat_FaceQuery q = { .fi = face };
                Vec3Copy(q.normal, result.cache->normal);

                sat_FeatureId manifold_features[4];
	        	colliding = HullFaceContact(mem_tmp, result.manifold, manifold_features, &result.cache->depth, result.cache->feature + b_v, &q, h, (constvec3ptr *)v_world, b_f, ref);
                if (!colliding 
                        || result.cache->feature[b_v] != cache->feature[b_v]
                        || f32_abs(result.cache->depth - cache->depth) >= g_numerics_config->manifold_cache_depth_max_diff_allowed)
                { 
                    break; 
                }
                goto sat_cleanup;
	        } break;

            default:
            {
                ds_AssertString(0, "Shouldn't be reachable");
            } break;
	    }
    }

	struct sat_FaceQuery f_query[2] = { { .depth = -F32_INFINITY }, { .depth = -F32_INFINITY } };
	struct sat_EdgeQuery e_query = { .depth = -F32_INFINITY };

	if (HullContactFVSeparation(&f_query[0], h[0], v_world[0], h[1], v_world[1]))
	{
		Vec3Copy(result.cache->normal, f_query[0].normal);
		result.cache->depth = f_query[0].depth;
		result.cache->type = SAT_CACHE_SEPARATION;
		goto sat_cleanup;
	}

	if (HullContactFVSeparation(&f_query[1], h[1], v_world[1], h[0], v_world[0]))
	{
		Vec3Negate(result.cache->normal, f_query[1].normal);
		result.cache->depth = f_query[1].depth;
		result.cache->type = SAT_CACHE_SEPARATION;
		goto sat_cleanup;
	}

	if (HullContactEESeparation(&e_query, h[0], v_world[0], h[1], v_world[1], t[0].position))
	{
		Vec3Copy(result.cache->normal, e_query.normal);
		result.cache->depth = e_query.depth;
		result.cache->type = SAT_CACHE_SEPARATION;
		goto sat_cleanup;
    }


    colliding = 1;
	if (0.99f*f_query[0].depth >= e_query.depth || 0.99f*f_query[1].depth >= e_query.depth)
	{
        /* b_f = body with reference/contact face, b_v = incident body with penetrating vertices */
        const u32 b_f = (f_query[0].depth < f_query[1].depth);
        const u32 b_v = 1 - b_f;

        f32 max_depth;
        sat_FeatureId manifold_features[4];
		colliding = HullFaceContact(mem_tmp, result.manifold, manifold_features, &max_depth, result.cache->feature + b_v, f_query + b_f, h, (constvec3ptr *)v_world, b_f, ref);

		if (colliding)
		{
			result.cache->type = SAT_CACHE_CONTACT_FV;
            result.cache->feature[b_f] = sat_FeatureIdConstruct(f_query[b_f].fi, SAT_FEATURE_TYPE_FACE);
            result.cache->depth = max_depth;
            Vec3Copy(result.cache->normal, f_query[b_f].normal);
		}
		else
		{
			result.cache->type = SAT_CACHE_SEPARATION;
			result.cache->depth = 0.0f;
			(b_f == ref)
				? Vec3Copy(result.cache->normal, f_query[0].normal)
				: Vec3Negate(result.cache->normal, f_query[1].normal);
		}
	}
	/* edgeContact */
	else
	{
		sat_EdgeQueryCollisionResult(result.manifold, result.cache, &e_query, ref);
	}

sat_cleanup:
    if (!colliding)
    {
        ArenaPopPacked(frame, sizeof(struct c_Manifold));
        result.manifold_count = 0;
        result.manifold = NULL;
    }
    ArenaPopScratch();
    //ProfZoneEnd;
    return result;
}

struct DelayedFeature
{
    u32 index;
    f32 dist_sq;
};

struct c_TriMeshBvhContact
{
    u32 tri;
    union
    {
        f32 dist_sq;
        f32 priority;
    };
    union
    {
        struct
        {
            enum TriVoronoiRegion   region;
            vec3                    c[2];
            struct TriVoronoi       tv;
        };

        struct
        {
            struct c_Manifold       manifold;
            struct c_SatCache       cache;
            u32                     delayed_set[2];
            u32                     delayed_count;
        };
    };
};

struct c_TriMeshBvhIterator
{
    struct arena *              tmp1;
    struct arena *              tmp2;

    struct c_TriMeshBvhContact *contact;
    u32                         contact_count;
    u32                         contact_len;

    u32                         delayed_count;
    struct DelayedFeature *     delayed_set;
    struct ds_BitSet               void_bitset;

    const struct triMeshBvh *   mesh_bvh;
    const struct triMesh *      mesh;
    const struct bvh *          bvh;

    const struct bvhNode *      node;
    const struct bvhNode **     node_stack;
    u32                         sc;

    /* Bounding box of transformed shape in BVH's local coordinate system */
    const struct aabb *         bvh_local_bbox;
    const struct aabb *         bvh_bbox;
};

static void c_TriMeshBvhIteratorAlloc(struct c_TriMeshBvhIterator *it, const struct triMeshBvh *mesh_bvh, struct aabb *bvh_local_bbox)
{
    it->tmp1 = ArenaPushScratch();
    it->tmp2 = ArenaPushScratch();

    struct memArray contact_arr = ArenaPushAlignedAll(it->tmp1, sizeof(struct c_TriMeshBvhContact), 8);
    it->contact = contact_arr.addr;
    it->contact_count = 0;
    it->contact_len = contact_arr.len;

    it->void_bitset = ds_BitSetAlloc(it->tmp2, mesh_bvh->mesh->v_count, 0, NOT_GROWABLE);

    it->mesh_bvh = mesh_bvh;
    it->mesh = mesh_bvh->mesh;
	it->bvh = &mesh_bvh->bvh;

	it->node = (struct bvhNode *) mesh_bvh->bvh.tree.pool.buf;
	it->node_stack = ArenaPush(it->tmp2, (mesh_bvh->depth+1)*sizeof(struct bvhNode **));
    it->sc = 0;

    it->bvh_local_bbox = bvh_local_bbox;
    it->bvh_bbox = &it->node[it->bvh->tree.root].bbox;
    if (AabbTest(it->bvh_local_bbox, it->bvh_bbox))
	{
		it->node_stack[it->sc++] = it->node + it->bvh->tree.root;
	}
}

static void c_TriMeshBvhIteratorDealloc(struct c_TriMeshBvhIterator *it)
{
    ArenaPopScratch();
    ArenaPopScratch();
}

static void c_TriMeshBvhIteratorPushChildren(struct c_TriMeshBvhIterator *it)
{
	const struct bvhNode *left = it->node + it->node_stack[it->sc]->bt_left;
	const struct bvhNode *right = it->node + it->node_stack[it->sc]->bt_right;
	if (AabbTest(it->bvh_local_bbox, &right->bbox))
	{
		it->node_stack[it->sc++] = right;
	}

	if (AabbTest(it->bvh_local_bbox, &left->bbox))
    {
        ds_Assert(it->sc < it->mesh_bvh->depth+1);
		it->node_stack[it->sc++] = left;
    }
}

static void c_TriMeshBvhIteratorDelayedSetAlloc(struct c_TriMeshBvhIterator *it)
{
    it->delayed_count = 0;
    it->delayed_set = ArenaPush(it->tmp2, it->contact_count*sizeof(struct DelayedFeature));
}

static void c_TriMeshBvhIteratorDelayedSetPush(struct c_TriMeshBvhIterator *it, const u32 contact, const f32 dist_sq)
{
    it->delayed_set[it->delayed_count].index = contact;
    it->delayed_set[it->delayed_count].dist_sq = dist_sq;
    for (u32 d = it->delayed_count; d; --d)
    {
        if (it->delayed_set[d-1].dist_sq <= it->delayed_set[d].dist_sq)
        {
            break;
        }
        const struct DelayedFeature tmp = it->delayed_set[d];
        it->delayed_set[d] = it->delayed_set[d-1];
        it->delayed_set[d-1] = tmp;
    }

    it->delayed_count += 1;
    ds_AssertString(it->delayed_count <= 64, "delayed phase exceeded 64 triangles, consider moving from insertion sort to something more proper.");
}

void c_ContactResultSortTriangles(struct arena *frame, struct c_ContactResult *result)
{
    if (!result->manifold_count)
    {
        return;
    }

    //TODO Shit sort for now
    ProfZoneNamed("Bad Triangle Sorting");

    result->tri_manifold = ArenaPushPacked(frame, result->manifold_count*sizeof(u32));
    if (!result->tri_manifold)
    {
	    Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
	    FatalCleanupAndExit();
    }

    u32 *t = result->tri;
    u32 *m = result->tri_manifold;
    for (u32 h = 0; h < result->manifold_count; ++h)
    {
        m[h] = h;
        for (u32 i = h; i; --i)
        {
            if (t[i-1] < t[i])
            {
                break;
            }

            const u32 t_tmp = t[i-1];
            t[i-1] = t[i];
            t[i] = t_tmp;

            const u32 m_tmp = m[i-1];
            m[i-1] = m[i];
            m[i] = m_tmp;
        }
    }

    ProfZoneEnd;
}

static const u32 delayed_vertex_map[TRI_VORONOI_COUNT][2] =
{
    { 0, 0 },
    { 1, 1 },
    { 2, 2 },
    { 0, 1 },
    { 1, 2 },
    { 2, 0 },
    { U32_MAX, U32_MAX },
};

struct c_ContactResult c_TriMeshBvhSphereContact(struct arena *frame, const struct c_ContactResult *not_used, const struct c_Shape *s[2], const ds_Transform tf[2], const u32 ref)
{
    //ProfZone;

	ds_Assert(s[0]->type == C_SHAPE_TRI_MESH);
	ds_Assert(s[1]->type == C_SHAPE_SPHERE);

    struct c_ContactResult result = { 0 };

    const struct triMeshBvh *mesh_bvh = &s[0]->mesh_bvh;
	const struct sphere *sph = &s[1]->sphere;

    quat q_inv;
	struct aabb bbox_transform;
    QuatInverse(q_inv, tf[0].rotation);
	Vec3Sub(bbox_transform.center, tf[1].position, tf[0].position);
    QuatVec3RotateSelf(bbox_transform.center, q_inv);
	Vec3Set(bbox_transform.hw, sph->radius, sph->radius, sph->radius);

    struct c_TriMeshBvhIterator it;
    c_TriMeshBvhIteratorAlloc(&it, mesh_bvh, &bbox_transform);

    mat3 bvh_rotation;
    Mat3Quat(bvh_rotation, tf[0].rotation);
    
    vec3 v_sphere;
    Vec3Copy(v_sphere, tf[1].position);

	{
        //ProfZoneNamed("Calculate Triangles");
	    while (it.sc--)
	    {
	    	if (!bt_LeafCheck(it.node_stack[it.sc]))
	    	{
                c_TriMeshBvhIteratorPushChildren(&it);
	    	}
	    	else
            { 
                const u32 tri_first = it.node_stack[it.sc]->bt_left;
                const u32 tri_last = tri_first + it.node_stack[it.sc]->bt_right - 1;
                for (u32 index = tri_first; index <= tri_last; ++index)
                {
                    struct c_TriMeshBvhContact *c = it.contact + it.contact_count;
                    c->tri = mesh_bvh->tri[index];

                    vec3 tri[3];
                    Mat3VecMul(tri[0], bvh_rotation, it.mesh->v[it.mesh->tri[c->tri][0]]);
                    Mat3VecMul(tri[1], bvh_rotation, it.mesh->v[it.mesh->tri[c->tri][1]]);
                    Mat3VecMul(tri[2], bvh_rotation, it.mesh->v[it.mesh->tri[c->tri][2]]);
                    Vec3Translate(tri[0], tf[0].position); 
                    Vec3Translate(tri[1], tf[0].position); 
                    Vec3Translate(tri[2], tf[0].position); 
                    Vec3Copy(c->c[1], v_sphere);

                    //ProfZoneNamed("TriCcwPointDistanceSquared");
                    const u32 robust = TriVoronoiInitCcw(&c->tv, tri);
                    ds_Assert(robust);

                    c->dist_sq = TriCcwPointDistanceSquared(c->c[0], &c->region, c->c[1], &c->tv);
                    //ProfZoneEnd;

                    if (c->dist_sq <= sph->radius*sph->radius)
                    {
                        it.contact_count += 1;
                        if (it.contact_count == it.contact_len)
                        {
	    			    	Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
	    			    	FatalCleanupAndExit();
                        }
                    }
                    else
                    {
                        COLLISION_DEBUG_ADD_SEGMENT(SegmentConstruct(c->tv.t[0], c->c[1]), Vec4Inline(0.8f, 0.8f, 0.4f, 1.0f));
                        COLLISION_DEBUG_ADD_SEGMENT(SegmentConstruct(c->tv.t[1], c->c[1]), Vec4Inline(0.8f, 0.8f, 0.4f, 1.0f));
                        COLLISION_DEBUG_ADD_SEGMENT(SegmentConstruct(c->tv.t[2], c->c[1]), Vec4Inline(0.8f, 0.8f, 0.4f, 1.0f));
                    }
                }
	    	}
	    }
        //ProfZoneEnd;
    }

    result.tri = ArenaPushPacked(frame, it.contact_count*sizeof(u32));
    result.manifold = ArenaPushPacked(frame, it.contact_count*sizeof(struct c_Manifold));
    c_TriMeshBvhIteratorDelayedSetAlloc(&it);
    if (it.contact_count && (!result.manifold || !result.tri || !it.delayed_set))
    {
		Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
		FatalCleanupAndExit();
	}

    for (u32 i = 0; i < it.contact_count; ++i)
    {
        /* face contact */
        struct c_TriMeshBvhContact *c = it.contact + i;
        if (c->region == TRI_VORONOI_FACE)
        {
            struct c_Manifold *m = result.manifold + result.manifold_count;
            u32 *t = result.tri + result.manifold_count;
            result.manifold_count += 1;
            c_HullSphereShallowManifold(m, s[1]->sphere.radius, c->c, ref);
            
            *t = c->tri;
            ds_BitSetSet(&it.void_bitset, it.mesh->tri[*t][0], 1);
            ds_BitSetSet(&it.void_bitset, it.mesh->tri[*t][1], 1);
            ds_BitSetSet(&it.void_bitset, it.mesh->tri[*t][2], 1);
        }
        else
        {
            c_TriMeshBvhIteratorDelayedSetPush(&it, i, c->dist_sq);
        }
    }

    for (u32 i = 0; i < it.delayed_count; ++i)
    {
        const struct c_TriMeshBvhContact *c = it.contact + it.delayed_set[i].index;
        const u32 *tri_id = it.mesh->tri[c->tri];
        const u32 v0 = tri_id[ delayed_vertex_map[c->region][0] ];
        const u32 v1 = tri_id[ delayed_vertex_map[c->region][1] ];
        /* if vertex_contact not in void, or edge contact and not fully in void */
        if (!ds_BitSetGet(&it.void_bitset, v0) || !ds_BitSetGet(&it.void_bitset, v1))
        {
            struct c_Manifold *m = result.manifold + result.manifold_count;
            u32 *t = result.tri + result.manifold_count;
            *t = c->tri;
            result.manifold_count += 1;

            c_HullSphereShallowManifold(m, s[1]->sphere.radius, c->c, ref);
        }

        ds_BitSetSet(&it.void_bitset, tri_id[0], 1);
        ds_BitSetSet(&it.void_bitset, tri_id[1], 1);
        ds_BitSetSet(&it.void_bitset, tri_id[2], 1);
    }

    c_TriMeshBvhIteratorDealloc(&it);

    c_ContactResultSortTriangles(frame, &result);

    //ProfZoneEnd;

    return result;
}

static void c_TriCapsuleManifold(struct c_Manifold *m, const struct c_TriMeshBvhContact *c, const struct capsule *cap, const struct segment *s, const u32 ref)
{
	if (c->dist_sq == 0.0f)
	{
        struct plane pl = PlaneConstructNormalized(c->tv.face_plane.normal_direction, c->c[0]);
        f32 d[2] = 
        {
            PlanePointSignedDistance(&pl, s->p[0]),
            PlanePointSignedDistance(&pl, s->p[1]),
        };

        vec3 c_p[2][3];

        vec3 tmp1, tmp2;
        f32 dist_sq[5] = 
        {
            d[0]*d[0],
            d[1]*d[1],
            SegmentDistanceSquared(c_p[0][0], c_p[1][0], c->tv.s + 0, s),
            SegmentDistanceSquared(c_p[0][1], c_p[1][1], c->tv.s + 1, s),
            SegmentDistanceSquared(c_p[0][2], c_p[1][2], c->tv.s + 2, s),
        };

        u32 best = 0;
        for (u32 i = 1; i < 5; ++i)
        {
            if (dist_sq[i] < dist_sq[best])
            {
                best = i;
            }
        }
		
        const u32 inc = 1 - ref;
		if (best < 2)
        {
			m->v_count = 2;

            f32 flip_sign;
            struct segment seg = TriCcwSegmentSideClip(s, &c->tv);
			if (best == 0)
			{
				Vec3Copy(m->v[0], seg.p[0]);
				PlaneSegmentClip(m->v[1], &pl, &seg);
                flip_sign = (d[0] <= 0.0f)
                          ?  1.0f
                          : -1.0f;
                d[0] = PlanePointSignedDistance(&pl, m->v[0]);
                d[1] = 0.0f;
			}
			else if (best == 1)
			{
				PlaneSegmentClip(m->v[0], &pl, &seg);
				Vec3Copy(m->v[1], seg.p[1]);
                flip_sign = (d[1] <= 0.0f)
                          ?  1.0f
                          : -1.0f;
                d[0] = 0.0f;
                d[1] = PlanePointSignedDistance(&pl, m->v[1]);
			}

			Vec3Copy(m->v[0], seg.p[0]);
			Vec3Copy(m->v[1], seg.p[1]);
            m->depth[0] = f32_abs(d[0]);
            m->depth[1] = f32_abs(d[1]);
			
            if (ref == 0)
            {
                Vec3Scale(m->n, pl.normal, flip_sign);
			    Vec3TranslateScaled(m->v[0], m->n, m->depth[0]);
			    Vec3TranslateScaled(m->v[1], m->n, m->depth[1]);
            }
            else
            {
                Vec3Scale(m->n, pl.normal, -1.0f*flip_sign);
			    Vec3TranslateScaled(m->v[0], m->n, cap->radius);
			    Vec3TranslateScaled(m->v[1], m->n, cap->radius);
            }
            m->depth[0] += cap->radius;
            m->depth[1] += cap->radius;
        }
        else
		{
            const u32 v = best - 2;
			m->v_count = 1;
		    m->depth[0] = f32_sqrt(dist_sq[best]);
			Vec3Sub(m->n, c_p[ref][v], c_p[inc][v]);
			Vec3ScaleSelf(m->n, 1.0f/Vec3Length(m->n));
			Vec3Copy(m->v[0], c_p[ref][v]);
            if (ref == 1)
            {
                Vec3TranslateScaled(m->v[0], m->n, cap->radius);
            }
		}	
	}
	/* Shallow Penetration */
	else
	{
        Vec3Sub(m->n, c->c[1-ref], c->c[ref]);
	    Vec3ScaleSelf(m->n, 1.0f / Vec3Length(m->n));
        m->depth[0] = cap->radius - f32_sqrt(c->dist_sq);
        if (SegmentPointCheck(s, (100.0f*F32_EPSILON)*(100.0f*F32_EPSILON)) || PlaneSegmentParallelCheck(&c->tv.face_plane, s))
        {
			m->v_count = 2;
			m->depth[1] = m->depth[0];

            struct segment cap_clip = TriCcwSegmentSideClip(s, &c->tv);
			Vec3Copy(m->v[0], cap_clip.p[0]);
			Vec3Copy(m->v[1], cap_clip.p[1]);

            if (ref == 0)
            {
		        Vec3TranslateScaled(m->v[0], m->n, -cap->radius + m->depth[0]);
		        Vec3TranslateScaled(m->v[1], m->n, -cap->radius + m->depth[1]);
            }
            else
            {
		        Vec3TranslateScaled(m->v[0], m->n, cap->radius);
		        Vec3TranslateScaled(m->v[1], m->n, cap->radius);
            }
        }
        else
        {
            m->v_count = 1;
            Vec3Copy(m->v[0], c->c[ref]);
            if (ref == 1)
            {
		        Vec3TranslateScaled(m->v[0], m->n, cap->radius);
            }
        }
	}
}

struct c_ContactResult c_TriMeshBvhCapsuleContact(struct arena *frame, const struct c_ContactResult *not_used, const struct c_Shape *s[2], const ds_Transform tf[2], const u32 reference_index)
{
	ds_Assert(s[0]->type == C_SHAPE_TRI_MESH);
	ds_Assert(s[1]->type == C_SHAPE_CAPSULE);

    struct c_ContactResult result = { 0 };

    //ProfZone;

	const struct capsule *cap = &s[1]->capsule;
    const struct segment cap_s = SegmentCapsuleTransform(cap, tf +1);
    vec3 cap_v[2];
    Vec3Copy(cap_v[0], cap_s.p[0]);
    Vec3Copy(cap_v[1], cap_s.p[1]);

    const struct triMeshBvh *mesh_bvh = &s[0]->mesh_bvh;
    /* bvh local-space capsule segment */
    quat q_inv;
    QuatInverse(q_inv, tf[0].rotation);
    Vec3TranslateScaled(cap_v[0], tf[0].position, -1.0f);
    Vec3TranslateScaled(cap_v[1], tf[0].position, -1.0f);
    QuatVec3RotateSelf(cap_v[0], q_inv);
    QuatVec3RotateSelf(cap_v[1], q_inv);
    const struct segment cap_mesh_space_s = SegmentConstruct(cap_v[0], cap_v[1]);
	struct aabb bbox_transform = BboxSegment(&cap_mesh_space_s);
    const vec3 radius = { cap->radius, cap->radius, cap->radius };
	Vec3Translate(bbox_transform.hw, radius);

    mat3 bvh_rotation;
    Mat3Quat(bvh_rotation, tf[0].rotation);

    struct c_TriMeshBvhIterator it;
    c_TriMeshBvhIteratorAlloc(&it, mesh_bvh, &bbox_transform);

	{
        //ProfZoneNamed("Calculate Triangles");
	    while (it.sc--)
	    {
	    	if (!bt_LeafCheck(it.node_stack[it.sc]))
	    	{
                c_TriMeshBvhIteratorPushChildren(&it);
	    	}
	    	else
            { 
                const u32 tri_first = it.node_stack[it.sc]->bt_left;
                const u32 tri_last = tri_first + it.node_stack[it.sc]->bt_right - 1;
                for (u32 index = tri_first; index <= tri_last; ++index)
                {
                    struct c_TriMeshBvhContact *c = it.contact + it.contact_count;
                    c->tri = mesh_bvh->tri[index];

                    vec3 tri[3];
                    Mat3VecMul(tri[0], bvh_rotation, it.mesh->v[it.mesh->tri[c->tri][0]]);
                    Mat3VecMul(tri[1], bvh_rotation, it.mesh->v[it.mesh->tri[c->tri][1]]);
                    Mat3VecMul(tri[2], bvh_rotation, it.mesh->v[it.mesh->tri[c->tri][2]]);
                    Vec3Translate(tri[0], tf[0].position); 
                    Vec3Translate(tri[1], tf[0].position); 
                    Vec3Translate(tri[2], tf[0].position); 

                    //ProfZoneNamed("TriCcwSegmentDistanceSquared");
                    const u32 robust = TriVoronoiInitCcw(&c->tv, tri);
                    ds_Assert(robust);

                    c->dist_sq = TriCcwSegmentDistanceSquared(c->c[0], c->c[1], &c->region, &cap_s, &c->tv);
                    //ProfZoneEnd;

                    if (c->dist_sq <= cap->radius*cap->radius)
                    {
                        it.contact_count += 1;
                        if (it.contact_count == it.contact_len)
                        {
	    			    	Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
	    			    	FatalCleanupAndExit();
                        }
                    }
                    else
                    {
                        COLLISION_DEBUG_ADD_SEGMENT(SegmentConstruct(c->tv.t[0], c->c[1]), Vec4Inline(0.8f, 0.8f, 0.4f, 1.0f));
                        COLLISION_DEBUG_ADD_SEGMENT(SegmentConstruct(c->tv.t[1], c->c[1]), Vec4Inline(0.8f, 0.8f, 0.4f, 1.0f));
                        COLLISION_DEBUG_ADD_SEGMENT(SegmentConstruct(c->tv.t[2], c->c[1]), Vec4Inline(0.8f, 0.8f, 0.4f, 1.0f));
                    }
                }
	    	}
	    }
        //ProfZoneEnd;
    }

    result.manifold = ArenaPushPacked(frame, it.contact_count*sizeof(struct c_Manifold));
    result.tri = ArenaPushPacked(frame, it.contact_count*sizeof(u32));
    c_TriMeshBvhIteratorDelayedSetAlloc(&it);
    if (it.contact_count && (!result.manifold || !result.tri || !it.delayed_set))
    {
		Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
		FatalCleanupAndExit();
	} 
    
    for (u32 i = 0; i < it.contact_count; ++i)
    {
        /* face contact */
        struct c_TriMeshBvhContact *c = it.contact + i;
        if (c->region == TRI_VORONOI_FACE)
        {
            struct c_Manifold *m = result.manifold + result.manifold_count;
            u32 *t = result.tri + result.manifold_count;
            result.manifold_count += 1;
            c_TriCapsuleManifold(m, c, cap, &cap_s, reference_index);

            *t = c->tri;
            ds_BitSetSet(&it.void_bitset, it.mesh->tri[*t][0], 1);
            ds_BitSetSet(&it.void_bitset, it.mesh->tri[*t][1], 1);
            ds_BitSetSet(&it.void_bitset, it.mesh->tri[*t][2], 1);
        }
        else
        {
            c_TriMeshBvhIteratorDelayedSetPush(&it, i, c->dist_sq);
        }
    }

    for (u32 i = 0; i < it.delayed_count; ++i)
    {
        const struct c_TriMeshBvhContact *c = it.contact + it.delayed_set[i].index;
        const u32 *tri_id = it.mesh->tri[c->tri];
        const u32 v0 = tri_id[ delayed_vertex_map[c->region][0] ];
        const u32 v1 = tri_id[ delayed_vertex_map[c->region][1] ];
        /* if vertex_contact not in void, or edge contact and not fully in void */
        if (!ds_BitSetGet(&it.void_bitset, v0) || !ds_BitSetGet(&it.void_bitset, v1))
        {
            struct c_Manifold *m = result.manifold + result.manifold_count;
            u32 *t = result.tri + result.manifold_count;
            *t = c->tri;

            result.manifold_count += 1;
            c_TriCapsuleManifold(m, c, cap, &cap_s, reference_index);
        }

        ds_BitSetSet(&it.void_bitset, tri_id[0], 1);
        ds_BitSetSet(&it.void_bitset, tri_id[1], 1);
        ds_BitSetSet(&it.void_bitset, tri_id[2], 1);
    }

    c_TriMeshBvhIteratorDealloc(&it);

    c_ContactResultSortTriangles(frame, &result);

    //ProfZoneEnd;

    return result;
}

static u32 TriCcwHullEEIsMinkowskiFace(const vec3 n_tri, const vec3 n2_1, const vec3 n2_2, const vec3 arc_n1, const vec3 arc_n2)
{
	const f32 n1_1d = Vec3Dot(n_tri, arc_n2);
	const f32 n1_2d = -n1_1d;
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

static u32 TriCcwHullEECheck(struct sat_EdgeQuery *query, const struct plane *tri_plane, const struct segment tri_s[3], const f32 tri_s_dist_sq[3], const vec3 tri_center, const struct dcel *hull, const vec3 n2_1, const u32 e2_1)
{
	vec3 n2_2, e1, e2;
    
	/* we are working with minkowski difference A - B, so gauss map of B is (-B). n2_1, n2_2 cross product stays the same. */
    const u32 e2_2 = hull->e[e2_1].twin;
	const u32 f2_1 = hull->e[e2_1].face_ccw;
	const u32 f2_2 = hull->e[e2_2].face_ccw;
	const struct segment hull_s = SegmentConstruct(hull->v[hull->e[e2_1].origin], hull->v[hull->e[e2_2].origin]);
    DcelFaceDirectionLocal(n2_2, hull, f2_2);
	Vec3NegateSelf(n2_2);

	/* 
	 * test if A, -B edges intersect on gauss map, only if they do, 
	 * they are a candidate for collision
	 */
    for (u32 si = 0; si < 3; ++si)
    {
	    if (TriCcwHullEEIsMinkowskiFace(tri_plane->normal, n2_1, n2_2, tri_s[si].dir, hull_s.dir))
	    {
            // Inlined SegmentParallelCheck, 
	    	const f32 d2d2 = Vec3Dot(hull_s.dir, hull_s.dir);
	    	const f32 d1d2 = Vec3Dot(tri_s[si].dir, hull_s.dir);
            const f32 d1d1_d2d2 = tri_s_dist_sq[si]*d2d2;
	    	/* Skip parallel edge pairs  */
	    	if (d1d1_d2d2 - d1d2*d1d2 >= g_numerics_config->vec3_parallel_check_eps*d1d1_d2d2) 
	    	{
	    		Vec3Cross(e1, tri_s[si].dir, hull_s.dir);
	    		Vec3ScaleSelf(e1, 1.0f / Vec3Length(e1));
	    		Vec3Sub(e2, tri_s[si].p[0], tri_center);
	    		/* plane normal points from A -> B */
	    		if (Vec3Dot(e1, e2) < 0.0f)
	    		{
	    			Vec3NegateSelf(e1);
	    		}
	    		
	    		/* check segmente-segment distance interval signed plane distance, > 0.0f => we have found a seperating axis */
	    		Vec3Sub(e2, hull_s.p[0], tri_s[si].p[0]);
	    		const f32 dist = Vec3Dot(e1, e2);

	    		if (query->depth < dist)
	    		{
	    			query->depth = dist;
	    			Vec3Copy(query->normal, e1);
	    			query->s1 = tri_s[si];
	    			query->s2 = hull_s;
	    			query->e1 = si;
	    			query->e2 = e2_1;
                    if (query->depth > 0)
                    {
                        return 1;
                    }
	    		}
	    	}
	    }
    }

    return 0;
}

static void TriCcwHullEdgeQueryCollisionResult(struct c_Manifold *manifold, struct c_SatCache *cache, u32 delayed_set[2], u32 *delayed_count, const struct sat_EdgeQuery *query, const u32 ref)
{
	sat_EdgeQueryCollisionResult(manifold, cache, query, ref);

    ds_Assert(query->e1 <= 2);
    *delayed_count = 2;
    delayed_set[0] = query->e1;
    delayed_set[1] = (query->e1+1) % 3;
}

static void TriCcwHullDelayedSet(u32 delayed_set[2], u32 *delayed_count, const struct dcel *hull_tri, const sat_FeatureId feature[4], const u32 feature_count, const u32 b_f)
{
    *delayed_count = 0;
    if (b_f == 0)
    {
        return;
    }

    /* Setup delayed vertices (if any) */
    if (feature_count == 1)
    {
        const u32 index = sat_FeatureIdIndex(feature[0]);
        if (sat_FeatureIdVertexCheck(feature[0]))
        {
            *delayed_count = 1;
            delayed_set[0] = index;
        }
        else
        {
            *delayed_count = 2;
            delayed_set[0] = hull_tri->e[index].origin;
            delayed_set[1] = hull_tri->e[ hull_tri->e[index].twin ].origin;
        }
    }
    else if (feature_count == 2)
    {
        const u32 index0 = sat_FeatureIdIndex(feature[0]);
        const u32 index1 = sat_FeatureIdIndex(feature[1]);
        u32 delayed_point[3] = { 0, 0, 0 };
        if (sat_FeatureIdVertexCheck(feature[0]))
        {
            delayed_point[index0] = 1;
        }
        else
        {
            const u32 v0 = hull_tri->e[index0].origin;
            const u32 v1 = hull_tri->e[ hull_tri->e[index0].twin ].origin;
            delayed_point[v0] = 1;
            delayed_point[v1] = 1;
        }

        if (sat_FeatureIdVertexCheck(feature[1]))
        {
            delayed_point[index1] = 1;
        }
        else
        {
            const u32 v0 = hull_tri->e[index1].origin;
            const u32 v1 = hull_tri->e[ hull_tri->e[index1].twin ].origin;
            delayed_point[v0] = 1;
            delayed_point[v1] = 1;
        }

        const u32 sum = delayed_point[0] + delayed_point[1] + delayed_point[2];
        if (sum < 3)
        {
            if (sum == 1)
            {
                delayed_set[0] = 0 + delayed_point[1] + 2*delayed_point[2];
            }
            else
            {
                if (delayed_point[0])
                {
                    delayed_set[0] = 0;
                    delayed_set[1] = delayed_point[1] + 2*delayed_point[2];
                }
                else
                {
                    delayed_set[0] = 1;
                    delayed_set[1] = 2;
                }
            }
        }
    }
}

static u32 TriCcwHullContact(struct c_Manifold *manifold, struct c_SatCache *new_cache, u32 delayed_set[2], u32 *delayed_count, const struct c_SatCache *old_cache, const vec3 tri[3], const struct dcel *hull, const u32 ref)
{ 
    struct arena *tmp = ArenaPushScratch();
    
    struct dcel hull_tri = DcelTriStub();
    hull_tri.v = (vec3ptr) tri;
    const struct dcel *h[2] = { &hull_tri, hull };
    constvec3ptr v[2] = { tri, hull->v };

    const vec3 tri_center =
    {
        (tri[0][0] + tri[1][0] + tri[2][0]) / 3.0f,
        (tri[0][1] + tri[1][1] + tri[2][1]) / 3.0f,
        (tri[0][2] + tri[1][2] + tri[2][2]) / 3.0f,
    };

    //TODO Move this to ds_DynamicsStatistics
    static ds_ThreadLocal u32 cache_count = 0;
    static ds_ThreadLocal u32 eviction_count = 0;
    static ds_ThreadLocal u32 call_count = 0;
    call_count += 1;
	u32 colliding = 0;
    if (old_cache)
    {
        //ProfZoneNamed("Cache-Ops");
        switch (old_cache->type)
        {
            case SAT_CACHE_SEPARATION:
	        {
                cache_count += 1;
	        	vec3 support1, support2, tmp;
	        	Vec3Negate(tmp, old_cache->normal);

	        	VertexSupport(support1, old_cache->normal, v[0], h[0]->v_count);
	        	VertexSupport(support2, tmp, v[1], h[1]->v_count);

	        	const f32 dot1 = Vec3Dot(support1, old_cache->normal);
	        	const f32 dot2 = Vec3Dot(support2, old_cache->normal);
	        	const f32 depth = dot2 - dot1;
	        	if (depth <= 0.0f)
	        	{
                    eviction_count += 1;
                    break;
	        	}

                Vec3Copy(new_cache->normal, old_cache->normal);
	        	new_cache->depth = depth;
                new_cache->type = SAT_CACHE_SEPARATION;
                //ProfZoneEnd;
                goto sat_cleanup;
	        } break;
             
            case SAT_CACHE_CONTACT_EE:
	        {
                cache_count += 1;
	            struct sat_EdgeQuery e_query = { .depth = -F32_INFINITY };
	        	HullContactEECheckRecompute(&e_query, h[0], v[0], sat_FeatureIdIndex(old_cache->feature[0]), h[1], v[1], sat_FeatureIdIndex(old_cache->feature[1]), tri_center);
                TriCcwHullEdgeQueryCollisionResult(manifold, new_cache, delayed_set, delayed_count, &e_query, ref);

                colliding = (e_query.depth < 0.0f);
                if (!colliding
                        || f32_abs(new_cache->depth - old_cache->depth) >= g_numerics_config->manifold_cache_depth_max_diff_allowed
                        || Vec3Dot(new_cache->normal, old_cache->normal) < g_numerics_config->manifold_cache_normal_parallel_check_eps)
                {
                    eviction_count += 1;
                    break;
                }

                //ProfZoneEnd;
                goto sat_cleanup;
	        } break;

            case SAT_CACHE_CONTACT_FV:
            {
                cache_count += 1;
                const u32 b_f = sat_FeatureIdFaceCheck(old_cache->feature[1]);
                const u32 b_v = 1 - b_f;
                const u32 face = sat_FeatureIdIndex(old_cache->feature[b_f]);
	        	DcelFaceNormalLocal(new_cache->normal, h[b_f], face);
                if (Vec3Dot(new_cache->normal, old_cache->normal) < g_numerics_config->manifold_cache_normal_parallel_check_eps) 
                { 
                    eviction_count += 1;
                    break; 
                }

                struct sat_FaceQuery q = { .fi = face };
                Vec3Copy(q.normal, new_cache->normal);

                sat_FeatureId manifold_features[4];
	        	colliding = HullFaceContact(tmp, manifold, manifold_features, &new_cache->depth, new_cache->feature + b_v, &q, h, (constvec3ptr *) v, b_f, ref);
                if (!colliding 
                        || new_cache->feature[b_v] != old_cache->feature[b_v]
                        || f32_abs(new_cache->depth - old_cache->depth) >= g_numerics_config->manifold_cache_depth_max_diff_allowed)
                { 
                    eviction_count += 1;
                    break; 
                }

                new_cache->type = SAT_CACHE_CONTACT_FV;
                new_cache->feature[b_f] = old_cache->feature[b_f];

                TriCcwHullDelayedSet(delayed_set, delayed_count, &hull_tri, manifold_features, manifold->v_count, b_f);
        
                //ProfZoneEnd;
                goto sat_cleanup;
	        } break;

            default:
            {

            } break;
	    }
        //ProfZoneEnd;
    }

    //TODO
    if ((cache_count % 1000) == 1)
        fprintf(stderr, "EvictionRate, SetRate: %f, %f\n", (f32) eviction_count / cache_count, (f32) cache_count / call_count);
        
    colliding = 0;
	struct sat_FaceQuery f_query[2] = { { .depth = -F32_INFINITY }, { .depth = -F32_INFINITY } };
	struct sat_EdgeQuery e_query = { .depth = -F32_INFINITY };

    /* tri-plane vs. hull */
    const struct plane tri_plane = PlaneConstructNormalizedFromCcwTriangle(tri[0], tri[1], tri[2]);
    {
        //ProfZoneNamed("TriPlane vs. Hull");
        f32 min_dist[2] = { F32_INFINITY, F32_INFINITY};
        for (u32 i  = 0; i < hull->v_count; ++i)
        {
            const f32 dist = PlanePointSignedDistance(&tri_plane, hull->v[i]);
            min_dist[0] = f32_min(min_dist[0], dist);
            min_dist[1] = f32_min(min_dist[1], -dist);
        }
        //ProfZoneEnd;

        static const f32 f_query_sign[2] = { 1.0f, -1.0f };
        f_query[0].fi = (min_dist[0] < min_dist[1]);
        f_query[0].depth = min_dist[ f_query[0].fi ];
        Vec3Scale(f_query[0].normal, tri_plane.normal, f_query_sign[ f_query[0].fi ]);
        if (f_query[0].depth > 0.0f)
        {
		    Vec3Copy(new_cache->normal, f_query[0].normal);
		    new_cache->depth = f_query[0].depth;
		    new_cache->type = SAT_CACHE_SEPARATION;
            goto sat_cleanup;
        }
    }

    /* hull-planes vs. triangle */
    struct plane min_plane;
    {
        //ProfZoneNamed("HullPlane vs. Tri");
        u32 separation_axis_found = 0;
    	for (u32 fi = 0; fi < hull->f_count; ++fi)
    	{
            const struct plane sep_plane = DcelFacePlaneLocal(hull, fi);

    		const f32 dist0 = PlanePointSignedDistance(&sep_plane, tri[0]);
    		const f32 dist1 = PlanePointSignedDistance(&sep_plane, tri[1]);
    		const f32 dist2 = PlanePointSignedDistance(&sep_plane, tri[2]);
    		const f32 min_dist = f32_min(f32_min(dist0, dist1), dist2);

    		if (f_query[1].depth < min_dist*Vec3LengthSquared(sep_plane.normal_direction))
    		{
                f_query[1].fi = fi;
    			f_query[1].depth = min_dist;
                min_plane = sep_plane;
                if (f_query[1].depth > 0.0f)
                {
                    separation_axis_found = 1;
                    break;
                }
    		}
    	}
        //ProfZoneEnd;

        const f32 n_dir_len = Vec3Length(min_plane.normal_direction);
        Vec3ScaleSelf(min_plane.normal_direction, 1.0f/n_dir_len);
        min_plane.signed_distance /= n_dir_len;
    	Vec3Copy(f_query[1].normal, min_plane.normal);
        f_query[1].depth /= n_dir_len;

        if (separation_axis_found)
        {
		    Vec3Copy(new_cache->normal, f_query[1].normal);
		    new_cache->depth = f_query[1].depth;
		    new_cache->type = SAT_CACHE_SEPARATION;
            goto sat_cleanup;
        }
    }

    /* Edge vs. Edge */
    {
        //ProfZoneNamed("Edge vs. Edge");
        const struct segment tri_s[3] = 
        {
            SegmentConstruct(tri[0], tri[1]),
            SegmentConstruct(tri[1], tri[2]),
            SegmentConstruct(tri[2], tri[0]),
        };

        const f32 tri_s_dist_sq[3] =
        {
            Vec3LengthSquared(tri_s[0].dir),
            Vec3LengthSquared(tri_s[1].dir),
            Vec3LengthSquared(tri_s[2].dir),
        };

        for (u32 fi = 0; fi < hull->f_count; ++fi)
        {
            vec3 f_dir;
            DcelFaceDirectionLocal(f_dir, hull, fi);
	        Vec3NegateSelf(f_dir);	
            const u32 ei_end = hull->f[fi].first + hull->f[fi].count;
            for (u32 ei = hull->f[fi].first; ei < ei_end; ++ei)
            {
                if (ei > hull->e[ei].twin) { continue; }

                if (TriCcwHullEECheck(&e_query, &tri_plane, tri_s, tri_s_dist_sq, tri_center, hull, f_dir, ei))
                {
                    //ProfZoneEnd;
		            Vec3Copy(new_cache->normal, e_query.normal);
		            new_cache->depth = e_query.depth;
		            new_cache->type = SAT_CACHE_SEPARATION;
                    goto sat_cleanup;
                }
            } 
        }
        //ProfZoneEnd;
    }

	colliding = 1;
	if (0.99f*f_query[0].depth >= e_query.depth || 0.99f*f_query[1].depth >= e_query.depth)
	{
        //ProfZoneNamed("FaceContact");

        const u32 b_f = (f_query[1].depth > f_query[0].depth);
        const u32 b_v = 1 - b_f;

        new_cache->type = SAT_CACHE_CONTACT_FV;
        new_cache->feature[b_f] = sat_FeatureIdConstruct(f_query[b_f].fi, SAT_FEATURE_TYPE_FACE);
        Vec3Copy(new_cache->normal, f_query[b_f].normal);
        sat_FeatureId manifold_features[4];
        colliding = HullFaceContact(tmp, manifold, manifold_features, &new_cache->depth, new_cache->feature + b_v, f_query + b_f, h, v, b_f, ref);

        TriCcwHullDelayedSet(delayed_set, delayed_count, &hull_tri, manifold_features, manifold->v_count, b_f);

        if (!colliding)
        {
			new_cache->type = SAT_CACHE_SEPARATION;
			new_cache->depth = 0.0f;
			(b_f == ref)
				? Vec3Copy(new_cache->normal, f_query[0].normal)
				: Vec3Negate(new_cache->normal, f_query[1].normal);
        }

        //ProfZoneEnd;
	}
	/* edgeContact */
	else
    {
        TriCcwHullEdgeQueryCollisionResult(manifold, new_cache, delayed_set, delayed_count, &e_query, ref);
	}

sat_cleanup:
    ArenaPopScratch();
	return colliding;
}

struct c_ContactResult c_TriMeshBvhHullContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s[2], const ds_Transform tf[2], const u32 reference_index)
{
	ds_Assert(s[0]->type == C_SHAPE_TRI_MESH);
	ds_Assert(s[1]->type == C_SHAPE_CONVEX_HULL);

    struct c_ContactResult result = { 0 };

    //ProfZone;

    const struct triMeshBvh *mesh_bvh = &s[0]->mesh_bvh;
	const struct dcel *hull = &s[1]->hull;

    /*
     *  v_bvh_local_space = R_INV_bvh*((R_hull*v_hull_local_space + T_hull) - T_bvh)
     *                    = R_INV_bvh*R_hull*v_hull_local_space + R_INV_bvh*(T_hull - T_bvh)
     *                    = R*v_hull + T
     */
    quat q_R, q_inv;
    QuatInverse(q_inv, tf[0].rotation);
    QuatMul(q_R, q_inv, tf[1].rotation);

    vec3 T;
    Vec3Sub(T, tf[1].position, tf[0].position);
    QuatVec3RotateSelf(T, q_inv);

    mat3 R;
    Mat3Quat(R, q_R);

    struct arena *tmp = ArenaPushScratch();
    struct dcel hull_bvh_local_space = *hull;
    hull_bvh_local_space.v = ArenaPush(tmp, hull->v_count*sizeof(vec3));

    struct aabb bbox_transform;
	vec3 min = { F32_INFINITY, F32_INFINITY, F32_INFINITY };
	vec3 max = { -F32_INFINITY, -F32_INFINITY, -F32_INFINITY };

    for (u32 i = 0; i < hull->v_count; ++i)
    {
        Mat3VecMul(hull_bvh_local_space.v[i], R, hull->v[i]);
        Vec3Translate(hull_bvh_local_space.v[i], T);

        Vec3MinSelf(min, hull_bvh_local_space.v[i]);
        Vec3MaxSelf(max, hull_bvh_local_space.v[i]);
    }

	Vec3Sub(bbox_transform.hw, max, min);
	Vec3ScaleSelf(bbox_transform.hw, 0.5f);
	Vec3Add(bbox_transform.center, min, bbox_transform.hw);

    mat3 bvh_rotation;
    Mat3Quat(bvh_rotation, tf[0].rotation);

    /* it.contact_count in this case counts real contacts + separation axes, so need an additional counter here */
    u32 true_contact_count = 0;
    struct c_TriMeshBvhIterator it;
    c_TriMeshBvhIteratorAlloc(&it, mesh_bvh, &bbox_transform);

	{
        //ProfZoneNamed("Calculate Triangles");
	    while (it.sc--)
	    {
	    	if (!bt_LeafCheck(it.node_stack[it.sc]))
	    	{
                c_TriMeshBvhIteratorPushChildren(&it);
	    	}
	    	else
            { 
                const u32 tri_first = it.node_stack[it.sc]->bt_left;
                const u32 tri_last = tri_first + it.node_stack[it.sc]->bt_right - 1;
                for (u32 index = tri_first; index <= tri_last; ++index)
                {
                    struct c_TriMeshBvhContact *c = it.contact + it.contact_count;
                    c->tri = mesh_bvh->tri[index];
                    c->cache.tri = mesh_bvh->tri[index];

                    vec3 tri[3];
                    Vec3Copy(tri[0], it.mesh->v[it.mesh->tri[c->tri][0]]);
                    Vec3Copy(tri[1], it.mesh->v[it.mesh->tri[c->tri][1]]);
                    Vec3Copy(tri[2], it.mesh->v[it.mesh->tri[c->tri][2]]);

                    //ProfZoneNamed("TriCcwHullContact");
                    const struct c_SatCache *old_cache = NULL;
                    for (u32 ci = 0; ci < cached_result->cache_count; ++ci)
                    {
                        if (c->tri == cached_result->cache[ci].tri)
                        {
                            old_cache = cached_result->cache + ci;
                            break;
                        }
                    }

                    true_contact_count += TriCcwHullContact(&c->manifold, &c->cache, c->delayed_set, &c->delayed_count, old_cache, tri, &hull_bvh_local_space, reference_index);
                    //ProfZoneEnd;

                    /* delayed set processed from deepest to shallowest contact. */
                    c->priority = -c->cache.depth * c->cache.depth;

                    it.contact_count += 1;
                    if (it.contact_count == it.contact_len)
                    {
	    				Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
	    				FatalCleanupAndExit();
                    }
                }
	    	}
	    }
        //ProfZoneEnd;
    }

    result.cache_count = 0;
    result.cache = (it.contact_count < g_numerics_config->cache_count_max) 
                     ? ArenaPushPacked(frame, it.contact_count*sizeof(struct c_SatCache))
                     : ArenaPushPacked(frame, g_numerics_config->cache_count_max*sizeof(struct c_SatCache));

    result.manifold = ArenaPush(frame, true_contact_count*sizeof(struct c_Manifold));
    result.tri = ArenaPush(frame, true_contact_count*sizeof(u32));
    c_TriMeshBvhIteratorDelayedSetAlloc(&it);
    if (it.contact_count && (!result.manifold || !result.tri || !it.delayed_set || !result.cache))
    {
		Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
		FatalCleanupAndExit();
	} 

    for (u32 i = 0; i < it.contact_count; ++i)
    {
        struct c_TriMeshBvhContact *c = it.contact + i;
        if (c->cache.type == SAT_CACHE_SEPARATION)
        {
            if (result.cache_count < g_numerics_config->cache_count_max)
            {
                memcpy(result.cache + result.cache_count, &c->cache, sizeof(struct c_SatCache));
                result.cache_count += 1;
            }
            continue;
        }

        if (c->delayed_count == 0)
        {
            struct c_Manifold *m = result.manifold + result.manifold_count;
            u32 *t = result.tri + result.manifold_count;
            *t = c->tri;
            result.manifold_count += 1;
            c_ManifoldTransform(m, &c->manifold, bvh_rotation, tf[0].position);

            if (result.cache_count < g_numerics_config->cache_count_max)
            {
                memcpy(result.cache + result.cache_count, &c->cache, sizeof(struct c_SatCache));
                result.cache_count += 1;
            }

            ds_BitSetSet(&it.void_bitset, it.mesh->tri[*t][0], 1);
            ds_BitSetSet(&it.void_bitset, it.mesh->tri[*t][1], 1);
            ds_BitSetSet(&it.void_bitset, it.mesh->tri[*t][2], 1);
        }
        else
        {
            c_TriMeshBvhIteratorDelayedSetPush(&it, i, c->priority);
        }
    }

    for (u32 i = 0; i < it.delayed_count; ++i)
    {
        const struct c_TriMeshBvhContact *c = it.contact + it.delayed_set[i].index;
        const u32 *tri_id = it.mesh->tri[c->tri];
        
        const u32 voided = (c->delayed_count == 1)
            ? ds_BitSetGet(&it.void_bitset, tri_id[ c->delayed_set[0] ])
            : ds_BitSetGet(&it.void_bitset, tri_id[ c->delayed_set[0] ]) && ds_BitSetGet(&it.void_bitset, tri_id[ c->delayed_set[1] ]);

        if (!voided)
        {
            struct c_Manifold *m = result.manifold + result.manifold_count;
            u32 *t = result.tri + result.manifold_count;
            *t = c->tri;
            result.manifold_count += 1;

            ds_Assert(c->manifold.v_count <= 2);
            c_ManifoldTransform(m, &c->manifold, bvh_rotation, tf[0].position);

            if (result.cache_count < g_numerics_config->cache_count_max)
            {
                memcpy(result.cache + result.cache_count, &c->cache, sizeof(struct c_SatCache));
                result.cache_count += 1;
            }
        }

        ds_BitSetSet(&it.void_bitset, tri_id[0], 1);
        ds_BitSetSet(&it.void_bitset, tri_id[1], 1);
        ds_BitSetSet(&it.void_bitset, tri_id[2], 1);
    }

    c_TriMeshBvhIteratorDealloc(&it);

    ArenaPopScratch();

    c_ContactResultSortTriangles(frame, &result);

    //ProfZoneEnd;
    
    return result;
}

/********************************** RAYCAST **********************************/

f32 c_SphereRaycastParameter(const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray)
{
	ds_Assert(shape->type == C_SHAPE_SPHERE);
	struct sphere sph = SphereConstruct(transform->position, shape->sphere.radius);
	return SphereRaycastParameter(&sph, ray);
}

f32 c_CapsuleRaycastParameter(const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray)
{
	ds_Assert(shape->type == C_SHAPE_CAPSULE);

	const f32 r = shape->capsule.radius;
	struct segment s = SegmentCapsuleTransform(&shape->capsule, transform);

	vec3 p0, p1;
	const f32 dist_sq = RaySegmentDistanceSquared(p0, p1, ray, &s);
	if (dist_sq > r*r) { return F32_INFINITY; }

	struct sphere sph = SphereConstruct(p1, r);
	return SphereRaycastParameter(&sph, ray);
}

f32 c_HullRaycastParameter(const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray)
{
	ds_Assert(shape->type == C_SHAPE_CONVEX_HULL);

	vec3 p;
	mat3 rot;
	Mat3Quat(rot, transform->rotation);
	const struct dcel *h = &shape->hull;
	f32 t_best = F32_INFINITY;

	for (u32 fi = 0; fi < h->f_count; ++fi)
	{
		struct plane pl = DcelFacePlane(h, rot, transform->position, fi);
		const f32 t = PlaneRaycastParameter(&pl, ray);
		if (t < t_best && t >= 0.0f)
		{
			RayPoint(p, ray, t);
			if (DcelFaceProjectedPointTest(h, rot, transform->position, fi, p))
			{
				t_best = t;
			}
		}
	}

	return t_best;
}

f32 c_TriMeshBvhRaycastParameter(const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray)
{
	quat inv_quat;
	mat3 inv_rot;
	vec3 tmp;
	QuatInverse(inv_quat, transform->rotation);
	Mat3Quat(inv_rot, inv_quat);

	const struct triMeshBvh *mesh_bvh = &shape->mesh_bvh;
	struct ray rotated_ray;
	Vec3Sub(tmp, ray->origin, transform->position);
	Mat3VecMul(rotated_ray.origin, inv_rot, tmp);
	Mat3VecMul(rotated_ray.dir, inv_rot, ray->dir);

    struct arena *mem_tmp = ArenaPushScratch();
	const f32 t = TriMeshBvhRaycast(mem_tmp, mesh_bvh, &rotated_ray).f;
    ArenaPopScratch();

    return t;
}
