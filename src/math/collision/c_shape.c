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

#include "ds_dynamics.h"
#include "collision.h"
#include "ds_float.h"
#include "ds_vector.h"
#include "ds_matrix.h"

SDB_DEFINE(c_Shape);

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

static f32 StaticsLineIntegrals(const v2 p0, const v2 p1, const v2 p2, const u32 p, const u32 q, const v3 int_scalars)
{
       ds_Assert(p <= 4 && q <= 4);

       f32 sum = 0.0f;
       for (u32 i = 0; i <= p; ++i)
       {
               for (u32 j = 0; j <= q; ++j)
               {
                       sum += int_scalars.x * Comb(p, i) * Comb(q, j) * F32Pow(p1.x, (f32) i) * F32Pow(p0.x, (f32) (p-i)) * F32Pow(p1.y, (f32) j) * F32Pow(p0.y, (f32) (q-j)) / Comb(p+q, i+j);
                       sum += int_scalars.y * Comb(p, i) * Comb(q, j) * F32Pow(p2.x, (f32) i) * F32Pow(p1.x, (f32) (p-i)) * F32Pow(p2.y, (f32) j) * F32Pow(p1.y, (f32) (q-j)) / Comb(p+q, i+j);
                       sum += int_scalars.z * Comb(p, i) * Comb(q, j) * F32Pow(p0.x, (f32) i) * F32Pow(p2.x, (f32) (p-i)) * F32Pow(p0.y, (f32) j) * F32Pow(p2.y, (f32) (q-j)) / Comb(p+q, i+j);
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

	const v3 *v = shape->hull.v;
	struct dcelFace *f = shape->hull.f + fi;
	struct dcelEdge *e0 = shape->hull.e + f->first;
	struct dcelEdge *e1 = shape->hull.e + f->first + 1;
	struct dcelEdge *e2 = shape->hull.e + f->first + 2;

	const v3 a = V3Sub(v[e1->origin], v[e0->origin]);
	const v3 b = V3Sub(v[e2->origin], v[e0->origin]);
	v3 n = V3Cross(a, b);
	n = V3Scale(n, 1.0f / V3Length(n));
	const f32 d = -V3Dot(n, v[e0->origin]);

	u32 max_index = 0;
	if (n.buf[max_index]*n.buf[max_index] < n.y*n.y) { max_index = 1; }
	if (n.buf[max_index]*n.buf[max_index] < n.z*n.z) { max_index = 2; }

	/* maxized normal direction determines projected surface integral axes (we maximse the projected surface area) */
	
	const u32 a_i = (1+max_index) % 3;
	const u32 b_i = (2+max_index) % 3;
	const u32 y_i = max_index % 3;

	//Vec3Set(n, n[a_i], n[b_i], n[y_i]);

	/* TODO: REPLACE */
	union { f32 f; u32 bits; } val = { .f = n.buf[y_i] };
	const f32 n_sign = (val.bits >> 31) ? -1.0f : 1.0f;

	const u32 tri_count = f->count - 2;
	for (u32 i = 0; i < tri_count; ++i)
	{
		e0 = shape->hull.e + f->first;
		e1 = shape->hull.e + f->first + 1 + i;
		e2 = shape->hull.e + f->first + 2 + i;

		const v2 p0 = V2(v[e0->origin].buf[a_i], v[e0->origin].buf[b_i]);
		const v2 p1 = V2(v[e1->origin].buf[a_i], v[e1->origin].buf[b_i]);
		const v2 p2 = V2(v[e2->origin].buf[a_i], v[e2->origin].buf[b_i]);
		
		const v3 delta_a = V3(p1.x - p0.x,
				      p2.x - p1.x,
				      p0.x - p2.x);
		
		const v3 delta_b = V3(p1.y - p0.y,
				      p2.y - p1.y,
				      p0.y - p2.y);

		/* simplify cross product of v1-v0, v2-v0 to get this */
		P_1   += ((p0.x + p1.x)*delta_b.x + (p1.x + p2.x)*delta_b.y + (p0.x + p2.x)*delta_b.z) / 2.0f;
		P_a   +=  StaticsLineIntegrals(p0, p1, p2, 2, 0, delta_b);
		P_aa  +=  StaticsLineIntegrals(p0, p1, p2, 3, 0, delta_b);
		P_aaa +=  StaticsLineIntegrals(p0, p1, p2, 4, 0, delta_b);
		P_b   += -StaticsLineIntegrals(p0, p1, p2, 0, 2, delta_a);
		P_bb  += -StaticsLineIntegrals(p0, p1, p2, 0, 3, delta_a);
		P_bbb += -StaticsLineIntegrals(p0, p1, p2, 0, 4, delta_a);
		P_ab  +=  StaticsLineIntegrals(p0, p1, p2, 2, 1, delta_b);
		P_aab +=  StaticsLineIntegrals(p0, p1, p2, 3, 1, delta_b);
		P_abb +=  StaticsLineIntegrals(p0, p1, p2, 1, 3, delta_b);
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

	const f32 a_y_div = n_sign / n.buf[y_i];
	const f32 n_y_div = 1.0f / n.buf[y_i];

	/* surface integrals */
	const f32 S_a 	= a_y_div * P_a;
	const f32 S_aa 	= a_y_div * P_aa;
	const f32 S_aaa = a_y_div * P_aaa;
	const f32 S_aab = a_y_div * P_aab;
	const f32 S_b 	= a_y_div * P_b;
	const f32 S_bb 	= a_y_div * P_bb;
	const f32 S_bbb = a_y_div * P_bbb;
	const f32 S_bby = -a_y_div * n_y_div * (n.buf[a_i]*P_abb + n.buf[b_i]*P_bbb + d*P_bb);
	const f32 S_y 	= -a_y_div * n_y_div * (n.buf[a_i]*P_a + n.buf[b_i]*P_b + d*P_1);
	const f32 S_yy 	= a_y_div * n_y_div * n_y_div * (n.buf[a_i]*n.buf[a_i]*P_aa + 2.0f*n.buf[a_i]*n.buf[b_i]*P_ab + n.buf[b_i]*n.buf[b_i]*P_bb 
			+ 2.0f*d*n.buf[a_i]*P_a + 2.0f*d*n.buf[b_i]*P_b + d*d*P_1);	
	const f32 S_yyy = -a_y_div * n_y_div * n_y_div * n_y_div * (n.buf[a_i]*n.buf[a_i]*n.buf[a_i]*P_aaa + 3.0f*n.buf[a_i]*n.buf[a_i]*n.buf[b_i]*P_aab
			+ 3.0f*n.buf[a_i]*n.buf[b_i]*n.buf[b_i]*P_abb + n.buf[b_i]*n.buf[b_i]*n.buf[b_i]*P_bbb + 3.0f*d*n.buf[a_i]*n.buf[a_i]*P_aa 
			+ 6.0f*d*n.buf[a_i]*n.buf[b_i]*P_ab + 3.0f*d*n.buf[b_i]*n.buf[b_i]*P_bb + 3.0f*d*d*n.buf[a_i]*P_a
		       	+ 3.0f*d*d*n.buf[b_i]*P_b + d*d*d*P_1);
	const f32 S_yya = a_y_div * n_y_div * n_y_div * (n.buf[a_i]*n.buf[a_i]*P_aaa + 2.0f*n.buf[a_i]*n.buf[b_i]*P_aab + n.buf[b_i]*n.buf[b_i]*P_abb 
			+ 2.0f*d*n.buf[a_i]*P_aa + 2.0f*d*n.buf[b_i]*P_ab + d*d*P_a);	

	if (max_index == 2)
	{
		integrals[VOL] += S_a * n.x;
	}
	else if (max_index == 1)
	{
		integrals[VOL] += S_b * n.x;
	}
	else
	{
		integrals[VOL] += S_y * n.x;
	}

	integrals[T_X + a_i] += S_aa * n.buf[a_i] / 2.0f;
	integrals[T_X + b_i] += S_bb * n.buf[b_i] / 2.0f;
	integrals[T_X + y_i] += S_yy * n.buf[y_i] / 2.0f;

	integrals[T_XX + a_i] += S_aaa * n.buf[a_i] / 3.0f;
	integrals[T_XX + b_i] += S_bbb * n.buf[b_i] / 3.0f;
	integrals[T_XX + y_i] += S_yyy * n.buf[y_i] / 3.0f;

	integrals[T_XY + a_i] += S_aab * n.buf[a_i] / 2.0f;
	integrals[T_XY + b_i] += S_bby * n.buf[b_i] / 2.0f;
	integrals[T_XY + y_i] += S_yya * n.buf[y_i] / 2.0f;
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
	v3 com = V3Zero();

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
		com = V3(integrals[T_X] / shape->volume,
			 integrals[T_Y] / shape->volume,
			 integrals[T_Z] / shape->volume);
		shape->center_of_mass = com;

		I_xx = integrals[T_YY] + integrals[T_ZZ] - shape->volume*(com.y*com.y + com.z*com.z);
		I_yy = integrals[T_XX] + integrals[T_ZZ] - shape->volume*(com.x*com.x + com.z*com.z);
		I_zz = integrals[T_XX] + integrals[T_YY] - shape->volume*(com.x*com.x + com.y*com.y);
		I_xy = integrals[T_XY] - shape->volume*com.x*com.y;
		I_xz = integrals[T_ZX] - shape->volume*com.x*com.z;
		I_yz = integrals[T_YZ] - shape->volume*com.y*com.z;
		shape->inertia_tensor = M3(I_xx, -I_xy, -I_xz,
			       		 	 -I_xy,  I_yy, -I_yz,
						 -I_xz, -I_yz, I_zz);
	}
	else if (shape->type == C_SHAPE_SPHERE)
	{
		shape->center_of_mass = V3Zero();
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

		shape->inertia_tensor = M3(I_xx, -I_xy, -I_xz,
			       		 	 -I_xy,  I_yy, -I_yz,
						 -I_xz, -I_yz, I_zz);
	}
	else if (shape->type == C_SHAPE_CAPSULE)
	{
		shape->center_of_mass = V3Zero();
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
		shape->inertia_tensor = M3(I_xx_up, -I_xy_up, -I_xz_up,
			       		 	 -I_xy_up,  I_yy_up, -I_yz_up,
						 -I_xz_up, -I_yz_up,  I_zz_up);
	}
}
