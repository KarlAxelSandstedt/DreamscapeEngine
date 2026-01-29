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

#include <string.h>
#include "ds_math.h"
#include "geometry.h"
#include "hash_map.h"
#include "array_list.h"
#include "queue.h"
#include "float32.h"
#include "sys_public.h"
#include "ds_random.h"

//TODO what to do with this?
#define MIN_SEGMENT_LENGTH_SQ	(100.0f*F32_EPSILON)

struct ray ray_construct(const vec3 origin, const vec3 dir)
{
	assert(Vec3LengthSquared(dir) > 0.0f);

	struct ray r;
	Vec3Copy(r.origin, origin);
	Vec3Copy(r.dir, dir);
	return r;
}

struct segment ray_construct_segment(const struct ray *r, const f32 t)
{
	vec3 p;
	Vec3Copy(p, r->origin);
	Vec3TranslateScaled(p, r->dir, t);
	return segment_construct(r->origin, p);
}


void ray_point(vec3 ray_point, const struct ray *ray, const f32 t)
{
	Vec3Copy(ray_point, ray->origin);
	Vec3TranslateScaled(ray_point, ray->dir, t);
}

struct sphere sphere_construct(const vec3 center, const f32 radius)
{
	struct sphere sph = { .radius = radius };
	Vec3Copy(sph.center, center);
	return sph;	
}

/*
 * | r.o + t*r.dir - s.c |^2 = s.r^2 => solve quadratic formula give the following method.
 */
f32 sphere_raycast_parameter(const struct sphere *sph, const struct ray *ray)
{
	vec3 diff;
	Vec3Sub(diff, ray->origin, sph->center);

	const f32 a = Vec3Dot(ray->dir, ray->dir);
	const f32 b = 2.0f * Vec3Dot(ray->dir, diff);
	const f32 c = Vec3Dot(diff, diff) - sph->radius*sph->radius;

	const f32 square = (b*b - 4.0f*a*c);
	if (square < 0.0f) { return F32_INFINITY; }

	const f32 root = f32_sqrt(square);

	const f32 t2 = -b + root;
	if (t2 < 0.0f) { return F32_INFINITY; }

	const f32 t1 = -b - root;
	return (t1 >= 0.0f) ? t1 / (2.0f * a) : t2 / (2.0f * a);
}

u32 sphere_raycast(vec3 intersection, const struct sphere *sph, const struct ray *ray)
{
	const f32 t = sphere_raycast_parameter(sph, ray);
	if (t < 0.0f || t == F32_INFINITY) { return 0; }

	Vec3Copy(intersection, ray->origin);
	Vec3TranslateScaled(intersection, ray->dir, t);
	return 1;
}

f32 ray_point_closest_point_parameter(const struct ray *ray, const vec3 p)
{ 
	vec3 diff;
	Vec3Sub(diff, p, ray->origin);
	const f32 tr = Vec3Dot(diff, ray->dir) / Vec3Dot(ray->dir, ray->dir);
	return (tr >= 0.0f) ? tr : 0.0f;
}

f32 ray_point_distance_sq(vec3 r_c, const struct ray *ray, const vec3 p)
{
	const f32 t = ray_point_closest_point_parameter(ray, p);
	ray_point(r_c, ray, t);
	return Vec3DistanceSquared(r_c, p);
}

f32 ray_segment_distance_sq(vec3 r_c, vec3 s_c, const struct ray *ray, const struct segment *s)
{
	vec3 diff;
	Vec3Sub(diff, s->p0, ray->origin);
	const f32 drdr = Vec3Dot(ray->dir, ray->dir);
	const f32 dsds = Vec3Dot(s->dir, s->dir);

	f32 tr = 0.0f;
	f32 ts = 0.0f;

	if (dsds >= MIN_SEGMENT_LENGTH_SQ)
	{
		const f32 drds = Vec3Dot(ray->dir, s->dir);
		const f32 diffdr = Vec3Dot(diff, ray->dir);
		const f32 diffds = Vec3Dot(diff, s->dir);
		const f32 denom = drdr*dsds - drds*drds;
		/* Check that the ray and segment are not parallel */
		if (denom > 0.0f)
		{
			tr = (diffdr*dsds - diffds*drds) / denom;
			tr = (tr >= 0.0f) ? tr : 0.0f;
		}

		ts = f32_clamp(tr*drds - diffds, 0.0f, dsds);
		if (ts == 0.0f)
		{
			tr = diffdr / drdr;
			tr = (tr >= 0.0f) ? tr : 0.0f;
		}
		else if (ts == dsds)
		{
			ts = 1.0f;
			tr = (diffdr + drds) / drdr;
			tr = (tr >= 0.0f) ? tr : 0.0f;
		}	
		else
		{
			ts /= dsds;
		}
	}
	else
	{
		tr = f32_clamp(Vec3Dot(diff, ray->dir) / drdr, 0.0f, 1.0f);
	}
	
	assert(0.0f <= tr);
	assert(0.0f <= ts && ts <= 1.0f);

	ray_point(r_c, ray, tr);
	segment_bc(s_c, s, ts);
	return Vec3DistanceSquared(r_c, s_c);
}

struct segment segment_construct(const vec3 p0, const vec3 p1)
{
	struct segment s;
	Vec3Copy(s.p0, p0);
	Vec3Copy(s.p1, p1);
	Vec3Sub(s.dir, p1, p0);
	return s;
}

f32 segment_distance_sq(vec3 c1, vec3 c2, const struct segment *s1, const struct segment *s2)
{
	vec3 diff;
	Vec3Sub(diff, s2->p0, s1->p0);
	const f32 d1d1 = Vec3LengthSquared(s1->dir);
	const f32 d2d2 = Vec3LengthSquared(s2->dir);

	f32 t1 = 0.0f;
	f32 t2 = 0.0f;

	if (d1d1 >= MIN_SEGMENT_LENGTH_SQ && d2d2 >= MIN_SEGMENT_LENGTH_SQ)
	{
		const f32 d1d2 = Vec3Dot(s1->dir, s2->dir);
		const f32 diffd1 = Vec3Dot(diff, s1->dir);
		const f32 diffd2 = Vec3Dot(diff, s2->dir);
		const f32 denom = d1d1*d2d2 - d1d2*d1d2;
		/* Check that the segments are not parallel */
		if (denom > 0.0f)
		{
			t1 = f32_clamp((diffd1*d2d2 - diffd2*d1d2) / denom, 0.0f, 1.0f);
		}

		/*
		 *  t2 = (L1_P1*(1-t1) + L1_P2*t1 - L2_P1) * DIR2 / DIR2*DIR2
		 *     = (-DIFF + DIR1*t1) * DIR2 / DIR2*DIR2
		 *     = (-DIFF*DIR2 + DIR1*DIR2*t1) / DIR2*DIR2
		 */
		t2 = f32_clamp(t1*d1d2 - diffd2, 0.0f, d2d2);

		if (t2 == 0.0f)
		{
			/*
			 *  t1 = (L2_P1*(1-t2) + L2_P2*t2 - L1_P1) * DIR1 / DIR1*DIR1
			 *     = (DIFF + DIR2*t2) * DIR1 / DIR1*DIR1
			 *     = DIFF*DIR1 / DIR1*DIR1
			 */
			t1 = f32_clamp(diffd1 / d1d1, 0.0f, 1.0f);
		}
		else if (t2 == d2d2)
		{
			t2 = 1.0f;
			t1 = f32_clamp((diffd1 + d1d2) / d1d1, 0.0f, 1.0f);
		}	
		else
		{
			t2 /= d2d2;
		}
	}
	/* S2 is point */
	else if (d1d1 >= MIN_SEGMENT_LENGTH_SQ)
	{
		/* 
		 * SIGNED PROJECTED LENGTH 
		 * 	= (L2_P1 - L1_P1) * DIR1 / |DIR1| 
		 * 	= t1*|DIR1|
		 * => t = DIFF*DIR1 / (DIR1*DIR1) 
		 */
		t1 = f32_clamp(Vec3Dot(diff, s1->dir) / d1d1, 0.0f, 1.0f);
	}
	else if (d2d2 >= MIN_SEGMENT_LENGTH_SQ)
	{
		t2 = f32_clamp(-Vec3Dot(diff, s2->dir) / d2d2, 0.0f, 1.0f);
	}

	assert(0.0f <= t1 && t1 <= 1.0f);
	assert(0.0f <= t2 && t2 <= 1.0f);

	segment_bc(c1, s1, t1);
	segment_bc(c2, s2, t2);
	return Vec3DistanceSquared(c1, c2);
}

f32 segment_point_distance_sq(vec3 c, const struct segment *s, const vec3 p)
{
	f32 t = 0.0f;
	
	if (Vec3LengthSquared(s->dir) >= MIN_SEGMENT_LENGTH_SQ)
	{
		vec3 diff;
		Vec3Sub(diff, p, s->p0);
		t = f32_clamp(Vec3Dot(diff, s->dir) / Vec3Dot(s->dir, s->dir), 0.0f, 1.0f);
	}

	segment_bc(c, s, t);
	return Vec3DistanceSquared(c, p);
}

void segment_bc(vec3 bc_p, const struct segment *s, const f32 t)
{
	Vec3Interpolate(bc_p, s->p1, s->p0, t);
}

f32 segment_point_projected_bc_parameter(const struct segment *s, const vec3 p)
{	
	vec3 diff;
	Vec3Sub(diff, p, s->p0);
	return Vec3Dot(diff, s->dir) / Vec3Dot(s->dir, s->dir);
}

f32 segment_point_closest_bc_parameter(const struct segment *s, const vec3 p)
{	
	vec3 diff;
	Vec3Sub(diff, p, s->p0);
	return f32_clamp(Vec3Dot(diff, s->dir) / Vec3Dot(s->dir, s->dir), 0.0f, 1.0f);
}

struct plane plane_construct(const vec3 n, const vec3 p)
{
	struct plane pl;
	Vec3Copy(pl.normal, n);
	pl.signed_distance = Vec3Dot(n, p);
	return pl;
}

struct plane plane_construct_from_ccw_triangle(const vec3 a, const vec3 b, const vec3 c)
{
	vec3 ab, ac, cross;
	Vec3Sub(ab, b, a);
	Vec3Sub(ac, c, a);
	Vec3Cross(cross, ab, ac);
	Vec3ScaleSelf(cross, 1.0f/Vec3Length(cross));
	return plane_construct(cross, a);
}

u32 plane_point_is_infront(const struct plane *pl, const vec3 p)
{
	return (plane_point_signed_distance(pl, p) > 0.0f) ? 1 : 0;
}

u32 plane_point_is_behind(const struct plane *pl, const vec3 p)
{
	return (plane_point_signed_distance(pl, p) < 0.0f) ? 1 : 0;
}

f32 plane_segment_clip_parameter(const struct plane *pl, const struct segment *s)
{
	/*
	 * 	GIVEN: pl.normal and segment direction not orthogonal.
	 *
	 * 	s.p0 + t*s.dir = PLANE POINT
	 * =>   DOT(s.p0 + t*s.dir - pl.normal*pl.signed_distance, pl.normal) = 0
	 * =>   DOT(t*s.dir pl.normal) = DOT(pl.normal*pl.signed_distance - s.p0, pl.normal)
	 * =>   t = [pl.signed_distance - DOT(s.p0, pl.normal)] / DOT(s.dir, pl.normal)
	 *
	 * degenerate case: segment parallel to plane gives t = +-infinity, which is okay!
	 */
	return (pl->signed_distance - Vec3Dot(pl->normal, s->p0)) / Vec3Dot(pl->normal, s->dir);
}

u32 plane_segment_clip(vec3 clip, const struct plane *pl, const struct segment *s)
{
	const f32 t = plane_segment_clip_parameter(pl, s);
	u32 intersection = 0;

	if (0.0f <= t && t <= 1.0f)
	{
		intersection = 0;
		segment_bc(clip, s, t);
	} 
	
	return intersection;
}

u32 plane_segment_test(const struct plane *pl, const struct segment *s)
{
	const f32 t = plane_segment_clip_parameter(pl, s);
	return (0.0f <= t && t <= 1.0f) ? 1 : 0;
}

f32 plane_point_signed_distance(const struct plane *pl, const vec3 p)
{
	return Vec3Dot(pl->normal, p) - pl->signed_distance;
}

f32 plane_point_distance(const struct plane *pl, const vec3 p)
{
	return f32_abs(Vec3Dot(pl->normal, p) - pl->signed_distance);
}

f32 plane_raycast_parameter(const struct plane *plane, const struct ray *ray)
{
	const f32 dot = Vec3Dot(ray->dir, plane->normal);
	if (dot == 0.0f) { return F32_INFINITY; }

	return (plane->signed_distance - Vec3Dot(ray->origin, plane->normal)) / dot;
}

u32 plane_raycast(vec3 intersection, const struct plane *plane, const struct ray *ray)
{
	const f32 t = plane_raycast_parameter(plane, ray);
	if (t < 0.0f || t == F32_INFINITY) { return 0; }

	Vec3Copy(intersection, ray->origin);
	Vec3TranslateScaled(intersection, ray->dir, t);
	return 1;
}

void AABB_vertex(struct AABB *dst, const vec3ptr v, const u32 v_count, const f32 margin)
{
	vec3 min = { F32_INFINITY, F32_INFINITY, F32_INFINITY };
	vec3 max = { -F32_INFINITY, -F32_INFINITY, -F32_INFINITY };
	for (u32 i = 0; i < v_count; ++i)
	{
		min[0] = f32_min(min[0], v[i][0]); 
		min[1] = f32_min(min[1], v[i][1]);			
		min[2] = f32_min(min[2], v[i][2]);			

		max[0] = f32_max(max[0], v[i][0]);			
		max[1] = f32_max(max[1], v[i][1]);			
		max[2] = f32_max(max[2], v[i][2]);			
	}

	Vec3Sub(dst->hw, max, min);
	Vec3ScaleSelf(dst->hw, 0.5f);
	Vec3Add(dst->center, min, dst->hw);

	dst->hw[0] += margin;
	dst->hw[1] += margin;
	dst->hw[2] += margin;
}

void AABB_union(struct AABB *box_union, const struct AABB *a, const struct AABB *b)
{
	vec3 min, max;
	
	min[0] = f32_min(a->center[0] - a->hw[0], b->center[0] - b->hw[0]);
	min[1] = f32_min(a->center[1] - a->hw[1], b->center[1] - b->hw[1]);
	min[2] = f32_min(a->center[2] - a->hw[2], b->center[2] - b->hw[2]);
                                                                      
	max[0] = f32_max(a->center[0] + a->hw[0], b->center[0] + b->hw[0]);
	max[1] = f32_max(a->center[1] + a->hw[1], b->center[1] + b->hw[1]);
	max[2] = f32_max(a->center[2] + a->hw[2], b->center[2] + b->hw[2]);
	
	Vec3Sub(box_union->hw, max, min);
	Vec3ScaleSelf(box_union->hw, 0.5f);
	Vec3Add(box_union->center, box_union->hw, min);
}

void AABB_rotate(struct AABB *dst, const struct AABB *src, mat3 rotation)
{
	/*
	 * Since we may pick any sign for hw[k], and the support point in any direction for an AABB is
	 * one of its corners, we derive new hw as hw_new[k] = Vec3AbsSelf(rot.row[k])*hw_old;
	 */

	const vec3 x = { f32_abs(rotation[0][0]), f32_abs(rotation[1][0]), f32_abs(rotation[2][0]), };
	const vec3 y = { f32_abs(rotation[0][1]), f32_abs(rotation[1][1]), f32_abs(rotation[2][1]), };
	const vec3 z = { f32_abs(rotation[0][2]), f32_abs(rotation[1][2]), f32_abs(rotation[2][2]), };

	dst->hw[0] = Vec3Dot(x, src->hw);
	dst->hw[1] = Vec3Dot(y, src->hw);
	dst->hw[2] = Vec3Dot(z, src->hw);

	Vec3Copy(dst->center, src->center);
}

u32 AABB_test(const struct AABB *a, const struct AABB *b)
{
	if (b->center[0] - b->hw[0] - (a->center[0] + a->hw[0]) > 0.0f 
			|| a->center[0] - a->hw[0] - (b->center[0] + b->hw[0]) > 0.0f) { return 0; }
	if (b->center[1] - b->hw[1] - (a->center[1] + a->hw[1]) > 0.0f 
			|| a->center[1] - a->hw[1] - (b->center[1] + b->hw[1]) > 0.0f) { return 0; }
	if (b->center[2] - b->hw[2] - (a->center[2] + a->hw[2]) > 0.0f 
			|| a->center[2] - a->hw[2] - (b->center[2] + b->hw[2]) > 0.0f) { return 0; }

	return 1;
}

u32 AABB_contains(const struct AABB *a, const struct AABB *b)
{
	if (b->center[0] - b->hw[0] < a->center[0] - a->hw[0]) { return 0; }
	if (b->center[1] - b->hw[1] < a->center[1] - a->hw[1]) { return 0; }
	if (b->center[2] - b->hw[2] < a->center[2] - a->hw[2]) { return 0; }
	
	if (b->center[0] + b->hw[0] > a->center[0] + a->hw[0]) { return 0; }
	if (b->center[1] + b->hw[1] > a->center[1] + a->hw[1]) { return 0; }
	if (b->center[2] + b->hw[2] > a->center[2] + a->hw[2]) { return 0; }

	return 1;
}

u32 AABB_contains_margin(const struct AABB *a, const struct AABB *b, const f32 margin)
{
	if (b->center[0] - b->hw[0] < a->center[0] - a->hw[0] - margin) { return 0; }
	if (b->center[1] - b->hw[1] < a->center[1] - a->hw[1] - margin) { return 0; }
	if (b->center[2] - b->hw[2] < a->center[2] - a->hw[2] - margin) { return 0; }
	
	if (b->center[0] + b->hw[0] > a->center[0] + a->hw[0] + margin) { return 0; }
	if (b->center[1] + b->hw[1] > a->center[1] + a->hw[1] + margin) { return 0; }
	if (b->center[2] + b->hw[2] > a->center[2] + a->hw[2] + margin) { return 0; }

	return 1;
}
	
void AABB_raycast_parameter_ex_setup(vec3 multiplier, vec3u32 dir_sign_bit, const struct ray *ray)
{
	multiplier[0] = 1.0f / (ray->dir[0]);
	multiplier[1] = 1.0f / (ray->dir[1]);
	multiplier[2] = 1.0f / (ray->dir[2]);

	dir_sign_bit[0] = (u32) f32_sign_bit(ray->dir[0]);
	dir_sign_bit[1] = (u32) f32_sign_bit(ray->dir[1]);
	dir_sign_bit[2] = (u32) f32_sign_bit(ray->dir[2]);
}

/*
	Quick Derivation: (For more information, see Christer Ericsson, Real-time collision detection ch 5.3)
	X = P + tD 
	Xn = dist

	aabb axis aligned =>

	find t such that 
			Xn = dist
		<=>	X[axis] * n[axis] = dist
		<=>	(P[axis] + t*D[axis]) * n[axis] = dist
		<=>	t = (dist - P[axis]*n[axis]) / (D[axis]*n[axis])	
		<=>	t = (dist - P[axis]) / D[axis]				(we assume n[axis] == 1.0f always)

		Note (1):
	
			If ray is perpendicular, do point_slab test of origin instead	

		optimization (1):

			1.0f / D[axis]*n[axis] precomputed.
*/
f32 AABB_raycast_parameter_ex(const struct AABB *aabb, const struct ray *ray, const vec3 multiplier, const vec3u32 dir_sign_bit)
{
	vec3 box_min, box_max;
	Vec3Sub(box_min, aabb->center, aabb->hw);
	Vec3Add(box_max, aabb->center, aabb->hw);

	f32 t_min = 0.0f;
	f32 t_max = F32_INFINITY;

	for (u32 axis = 0; axis < 3; ++axis)
	{
		/* If parallel to slab, point_slab test */
		if (f32_abs(ray->dir[axis]) < 10.0f * F32_EPSILON)
		{
			if (ray->origin[axis] < box_min[axis] || ray->origin[axis] > box_max[axis]) { return F32_INFINITY; }
		}
		else
		{
			const f32 t_1 = (box_min[axis] - ray->origin[axis]) * multiplier[axis];
			const f32 t_2 = (box_max[axis] - ray->origin[axis]) * multiplier[axis];

			/* if sign bit, we hit min_plane last, max_plane first => t_1 > t_2, else t_2 > t_1 */
			const f32 t_min_axis = (1-dir_sign_bit[axis])*t_1 + dir_sign_bit[axis]*t_2;
			const f32 t_max_axis = (1-dir_sign_bit[axis])*t_2 + dir_sign_bit[axis]*t_1;

			ds_Assert(t_min_axis <= t_max_axis);
			t_min = f32_max(t_min, t_min_axis);
			t_max = f32_min(t_max, t_max_axis);

			if (t_min > t_max)
			{
				return F32_INFINITY;
			}
		}
	}

	return t_min;
}

f32 AABB_raycast_parameter(const struct AABB *aabb, const struct ray *ray)
{
	vec3 multiplier;
       	vec3u32	dir_sign_bit;
	AABB_raycast_parameter_ex_setup(multiplier, dir_sign_bit, ray);
	return AABB_raycast_parameter_ex(aabb, ray, multiplier, dir_sign_bit);
}

u32 AABB_raycast_ex(vec3 intersection, const struct AABB *aabb, const struct ray *ray, const vec3 multiplier, const vec3u32 dir_sign_bit)
{
	const f32 t = AABB_raycast_parameter_ex(aabb, ray, multiplier, dir_sign_bit);
	if (t == F32_INFINITY) { return 0; }

	Vec3Copy(intersection, ray->origin);
	Vec3TranslateScaled(intersection, ray->dir, t);
	return 1;
}

u32 AABB_raycast(vec3 intersection, const struct AABB *aabb, const struct ray *ray)
{
	vec3 multiplier; 
	vec3u32 dir_sign_bit;
	AABB_raycast_parameter_ex_setup(multiplier, dir_sign_bit, ray);
	return AABB_raycast_ex(intersection, aabb, ray, multiplier, dir_sign_bit);
}

u64 AABB_push_lines_buffered(u8 *buf, const u64 bufsize, const struct AABB *box, const vec4 color)
{
	const u64 bytes_written = 3*8*(sizeof(vec3)+sizeof(vec4));
	if (bufsize < bytes_written)
	{
		return 0;
	}

	vec3 end;
	Vec3Sub(end, box->center, box->hw);

	f32 *v = (f32*) buf;
	Vec3Set(v+7*0, end[0], 		   end[1], 		     end[2]);
	Vec3Set(v+7*1, end[0] + 2.0f*box->hw[0], end[1], 		     end[2]);
	Vec3Set(v+7*2, end[0], 		   end[1], 		     end[2]);
	Vec3Set(v+7*3, end[0], 		   end[1] + 2.0f*box->hw[1], end[2]);
	Vec3Set(v+7*4, end[0], 		   end[1], 		     end[2]);
	Vec3Set(v+7*5, end[0], 		   end[1], 	             end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*6, end[0] + 2.0f*box->hw[0], end[1], 		     end[2]);
	Vec3Set(v+7*7, end[0] + 2.0f*box->hw[0], end[1] + 2.0f*box->hw[1], end[2]);
	Vec3Set(v+7*8, end[0] + 2.0f*box->hw[0], end[1], 		     end[2]);
	Vec3Set(v+7*9, end[0] + 2.0f*box->hw[0], end[1]                  , end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*10, end[0], 		   end[1] + 2.0f*box->hw[1], end[2]);
	Vec3Set(v+7*11, end[0], 		   end[1] + 2.0f*box->hw[1], end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*12, end[0], 		   end[1] + 2.0f*box->hw[1], end[2]);
	Vec3Set(v+7*13, end[0] + 2.0f*box->hw[0], end[1] + 2.0f*box->hw[1], end[2]);

	Vec3Set(v+7*14, end[0], 		   end[1], 	             end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*15, end[0], 		   end[1] + 2.0f*box->hw[1], end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*16, end[0], 		   end[1], 	             end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*17, end[0] + 2.0f*box->hw[0], end[1], 	             end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*18, end[0] + 2.0f*box->hw[0], end[1] + 2.0f*box->hw[1], end[2]);
	Vec3Set(v+7*19, end[0] + 2.0f*box->hw[0], end[1] + 2.0f*box->hw[1], end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*20, end[0], 		   end[1] + 2.0f*box->hw[1], end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*21, end[0] + 2.0f*box->hw[0], end[1] + 2.0f*box->hw[1], end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*22, end[0] + 2.0f*box->hw[0], end[1], 	             end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*23, end[0] + 2.0f*box->hw[0], end[1] + 2.0f*box->hw[1], end[2] + 2.0f*box->hw[2]);

	for (u32 i = 0; i < 24; ++i)
	{
		Vec4Copy(v + 7*i + 3, color);
	}

	return bytes_written;
}

u64 AABB_transform_push_lines_buffered(u8 *buf, const u64 bufsize, const struct AABB *box, const vec3 translation, mat3 rotation, const vec4 color)
{
	const u64 bytes_written = 3*8*(sizeof(vec3)+sizeof(vec4));
	if (bufsize < bytes_written)
	{
		return 0;
	}

	vec3 end;
	Vec3Sub(end, box->center, box->hw);

	f32 *v = (f32*) buf;
	Vec3Set(v+7*0, end[0], 		   end[1], 		     end[2]);
	Vec3Set(v+7*1, end[0] + 2.0f*box->hw[0], end[1], 		     end[2]);
	Vec3Set(v+7*2, end[0], 		   end[1], 		     end[2]);
	Vec3Set(v+7*3, end[0], 		   end[1] + 2.0f*box->hw[1], end[2]);
	Vec3Set(v+7*4, end[0], 		   end[1], 		     end[2]);
	Vec3Set(v+7*5, end[0], 		   end[1], 	             end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*6, end[0] + 2.0f*box->hw[0], end[1], 		     end[2]);
	Vec3Set(v+7*7, end[0] + 2.0f*box->hw[0], end[1] + 2.0f*box->hw[1], end[2]);
	Vec3Set(v+7*8, end[0] + 2.0f*box->hw[0], end[1], 		     end[2]);
	Vec3Set(v+7*9, end[0] + 2.0f*box->hw[0], end[1]                  , end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*10, end[0], 		   end[1] + 2.0f*box->hw[1], end[2]);
	Vec3Set(v+7*11, end[0], 		   end[1] + 2.0f*box->hw[1], end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*12, end[0], 		   end[1] + 2.0f*box->hw[1], end[2]);
	Vec3Set(v+7*13, end[0] + 2.0f*box->hw[0], end[1] + 2.0f*box->hw[1], end[2]);

	Vec3Set(v+7*14, end[0], 		   end[1], 	             end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*15, end[0], 		   end[1] + 2.0f*box->hw[1], end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*16, end[0], 		   end[1], 	             end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*17, end[0] + 2.0f*box->hw[0], end[1], 	             end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*18, end[0] + 2.0f*box->hw[0], end[1] + 2.0f*box->hw[1], end[2]);
	Vec3Set(v+7*19, end[0] + 2.0f*box->hw[0], end[1] + 2.0f*box->hw[1], end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*20, end[0], 		   end[1] + 2.0f*box->hw[1], end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*21, end[0] + 2.0f*box->hw[0], end[1] + 2.0f*box->hw[1], end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*22, end[0] + 2.0f*box->hw[0], end[1], 	             end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*23, end[0] + 2.0f*box->hw[0], end[1] + 2.0f*box->hw[1], end[2] + 2.0f*box->hw[2]);

	for (u32 i = 0; i < 24; ++i)
	{
		vec3 tmp1, tmp2;
		//Vec3Sub(tmp1, v + 7*i, box->center);
		//mat3_vec_mul(tmp2, rotation, tmp1);
		//Vec3Add(v + 7*i, tmp2, box->center);
		//Vec3Translate(v + 7*i, translation);

		mat3_vec_mul(tmp1, rotation, v + 7*i);
		Vec3Add(v + 7*i, tmp1, translation);


		Vec4Copy(v + 7*i + 3, color);
	}

	return bytes_written;
}

struct AABB bbox_triangle(const vec3 p0, const vec3 p1, const vec3 p2)
{
	struct AABB bbox;

	vec3 min = { p0[0], p0[1], p0[2] };
	vec3 max = { p0[0], p0[1], p0[2] };

	min[0] = f32_min(min[0], p1[0]); 
	min[1] = f32_min(min[1], p1[1]); 
	min[2] = f32_min(min[2], p1[2]); 

	max[0] = f32_max(max[0], p1[0]); 
	max[1] = f32_max(max[1], p1[1]); 
	max[2] = f32_max(max[2], p1[2]); 

	min[0] = f32_min(min[0], p2[0]); 
	min[1] = f32_min(min[1], p2[1]); 
	min[2] = f32_min(min[2], p2[2]); 

	max[0] = f32_max(max[0], p2[0]); 
	max[1] = f32_max(max[1], p2[1]); 
	max[2] = f32_max(max[2], p2[2]); 

	Vec3Sub(bbox.hw, max, min);
	Vec3ScaleSelf(bbox.hw, 0.5f);
	Vec3Add(bbox.center, min, bbox.hw);

	return bbox;
}

struct AABB bbox_union(const struct AABB a, const struct AABB b)
{
	struct AABB bbox;
	vec3 min, max;
	
	min[0] = f32_min(a.center[0] - a.hw[0], b.center[0] - b.hw[0]);
	min[1] = f32_min(a.center[1] - a.hw[1], b.center[1] - b.hw[1]);
	min[2] = f32_min(a.center[2] - a.hw[2], b.center[2] - b.hw[2]);
                                                                      
	max[0] = f32_max(a.center[0] + a.hw[0], b.center[0] + b.hw[0]);
	max[1] = f32_max(a.center[1] + a.hw[1], b.center[1] + b.hw[1]);
	max[2] = f32_max(a.center[2] + a.hw[2], b.center[2] + b.hw[2]);
	
	Vec3Sub(bbox.hw, max, min);
	Vec3ScaleSelf(bbox.hw, 0.5f);
	Vec3Add(bbox.center, bbox.hw, min);

	return bbox;
}

u32 vertex_support(vec3 support, const vec3 dir, const vec3ptr v, const u32 v_count)
{
	u32 best = U32_MAX;
	f32 max_dist = -F32_INFINITY;
	for (u32 i = 0; i < v_count; ++i)
	{
		const f32 dist = Vec3Dot(dir, v[i]);

		if (max_dist < dist)
		{
			best = i;
			max_dist = dist;
		}
	}

	Vec3Copy(support, v[best]);
	return best;
}

void vertex_centroid(vec3 centroid, const vec3ptr vs, const u32 n)
{
	Vec3Set(centroid, 0.0f, 0.0f, 0.0f);
	for (u32 i = 0; i < n; ++i)
	{
		Vec3Translate(centroid, vs[i]);
	}
	Vec3ScaleSelf(centroid, 1.0f / n);
}

void tri_ccw_normal(vec3 normal, const vec3 p0, const vec3 p1, const vec3 p2)
{
	vec3 A, B, C;
	Vec3Sub(A, p1, p0);
	Vec3Sub(B, p2, p0);
	Vec3Cross(C, A, B);
	Vec3Normalize(normal, C);
}

void tri_ccw_direction(vec3 dir, const vec3 p0, const vec3 p1, const vec3 p2)
{
	vec3 A, B;
	Vec3Sub(A, p1, p0);
	Vec3Sub(B, p2, p0);
	Vec3Cross(dir, A, B);
}

vec3 box_stub_vertex[8] =
{
	{  0.5f,  0.5f,  0.5f }, 
	{  0.5f,  0.5f, -0.5f },	
	{ -0.5f,  0.5f, -0.5f },	
	{ -0.5f,  0.5f,  0.5f },	
	{  0.5f, -0.5f,  0.5f },
	{  0.5f, -0.5f, -0.5f },	
	{ -0.5f, -0.5f, -0.5f },	
	{ -0.5f, -0.5f,  0.5f },	
};


static struct dcel_face box_face[] =
{
	{ .first  =  0, .count = 4 },
	{ .first  =  4, .count = 4 },
	{ .first  =  8, .count = 4 },
	{ .first  = 12, .count = 4 },
	{ .first  = 16, .count = 4 },
	{ .first  = 20, .count = 4 },
};

static struct dcel_edge box_edge[] =
{
	{ .origin = 0, .twin =  7,  .face_ccw = 0, },
	{ .origin = 1, .twin = 11,  .face_ccw = 0, },
	{ .origin = 2, .twin = 15,  .face_ccw = 0, },
	{ .origin = 3, .twin = 19,  .face_ccw = 0, },

	{ .origin = 0, .twin = 18,  .face_ccw = 1, },
	{ .origin = 4, .twin = 21,  .face_ccw = 1, },
	{ .origin = 5, .twin =  8,  .face_ccw = 1, },
	{ .origin = 1, .twin =  0,  .face_ccw = 1, },

	{ .origin = 1, .twin =  6,  .face_ccw = 2, },
	{ .origin = 5, .twin = 20,  .face_ccw = 2, },
	{ .origin = 6, .twin = 12,  .face_ccw = 2, },
	{ .origin = 2, .twin =  1,  .face_ccw = 2, },

	{ .origin = 2, .twin = 10,  .face_ccw = 3, },
	{ .origin = 6, .twin = 23,  .face_ccw = 3, },
	{ .origin = 7, .twin = 16,  .face_ccw = 3, },
	{ .origin = 3, .twin =  2,  .face_ccw = 3, },

	{ .origin = 3, .twin = 14,  .face_ccw = 4, },
	{ .origin = 7, .twin = 22,  .face_ccw = 4, },
	{ .origin = 4, .twin =  4,  .face_ccw = 4, },
	{ .origin = 0, .twin =  3,  .face_ccw = 4, },

	{ .origin = 6, .twin =  9,  .face_ccw = 5, },
	{ .origin = 5, .twin =  5,  .face_ccw = 5, },
	{ .origin = 4, .twin = 17,  .face_ccw = 5, },
	{ .origin = 7, .twin = 13,  .face_ccw = 5, },
};

struct dcel dcel_box_stub(void)
{
	struct dcel box = 
	{
		.v = box_stub_vertex,
		.e = box_edge,
		.f = box_face,
		.e_count = 24,
		.v_count = 8,
		.f_count = 6,
	};

	return box; 
}

struct dcel dcel_box(struct arena *mem, const vec3 hw)
{
	vec3ptr box_vertex = ArenaPush(mem, 8*sizeof(vec3));

	Vec3Set(box_vertex[0],  hw[0],  hw[1],  hw[2]); 
	Vec3Set(box_vertex[1],  hw[0],  hw[1], -hw[2]);	
	Vec3Set(box_vertex[2], -hw[0],  hw[1], -hw[2]);	
	Vec3Set(box_vertex[3], -hw[0],  hw[1],  hw[2]);	
	Vec3Set(box_vertex[4],  hw[0], -hw[1],  hw[2]);
	Vec3Set(box_vertex[5],  hw[0], -hw[1], -hw[2]);	
	Vec3Set(box_vertex[6], -hw[0], -hw[1], -hw[2]);	
	Vec3Set(box_vertex[7], -hw[0], -hw[1],  hw[2]);	

	struct dcel box = 
	{
		.v = box_vertex,
		.e = box_edge,
		.f = box_face,
		.e_count = 24,
		.v_count = 8,
		.f_count = 6,
	};

	return box; 
}

void dcel_face_direction(vec3 dir, const struct dcel *h, const u32 fi)
{
	vec3 a, b;
	struct dcel_edge *e0 = h->e + h->f[fi].first;
	struct dcel_edge *e1 = h->e + h->f[fi].first + 1;
	struct dcel_edge *e2 = h->e + h->f[fi].first + 2;
	Vec3Sub(a, h->v[e1->origin], h->v[e0->origin]);
	Vec3Sub(b, h->v[e2->origin], h->v[e0->origin]);
	Vec3Cross(dir, a, b);
}

void dcel_face_normal(vec3 normal, const struct dcel *h, const u32 fi)
{
	dcel_face_direction(normal, h, fi);
	Vec3ScaleSelf(normal, 1.0f/Vec3Length(normal));	
}

struct plane dcel_face_plane(const struct dcel *h, mat3 rot, const vec3 pos, const u32 fi)
{
	vec3 n, p;
	dcel_face_normal(p, h, fi);
	mat3_vec_mul(n, rot, p);
	mat3_vec_mul(p, rot, h->v[h->e[h->f[fi].first].origin]);
	Vec3Translate(p, pos);
	return plane_construct(n, p);
}

struct segment dcel_face_clip_segment(const struct dcel *h, mat3 rot, const vec3 pos, const u32 fi, const struct segment *s)
{
	vec3 f_n, p_n, p_p0, p_p1;

	dcel_face_normal(p_n, h, fi);
	mat3_vec_mul(f_n, rot, p_n);

	f32 min_p = 0.0f;
	f32 max_p = 1.0f;

	struct dcel_face *f = h->f + fi;
	for (u32 i = 0; i < f->count; ++i)
	{
		const u32 e0 = f->first + i;
		const u32 e1 = f->first + ((i + 1) % f->count);
		struct plane clip_plane = dcel_face_clip_plane(h, rot, pos, f_n, e0, e1);

		const f32 bc_c = plane_segment_clip_parameter(&clip_plane, s);
		if (min_p <= bc_c && bc_c <= max_p)
		{
			if (Vec3Dot(s->dir, clip_plane.normal) >= 0.0f)
			{
				max_p = bc_c;
			}
			else
			{
				min_p = bc_c;
			}
		}
	}	

	segment_bc(p_p0, s, min_p);
	segment_bc(p_p1, s, max_p);
	return segment_construct(p_p0, p_p1);
}

struct plane dcel_face_clip_plane(const struct dcel *h, mat3 rot, const vec3 pos, const vec3 face_normal, const u32 e0, const u32 e1)
{
	vec3 diff, p0, p1;
	struct dcel_edge *edge0 = h->e + e0; 
	struct dcel_edge *edge1 = h->e + e1; 

	mat3_vec_mul(p0, rot, h->v[edge0->origin]);
	mat3_vec_mul(p1, rot, h->v[edge1->origin]);
	Vec3Translate(p0, pos);
	Vec3Translate(p1, pos);
	Vec3Sub(diff, p1, p0);
	Vec3Cross(p1, diff, face_normal);
	Vec3ScaleSelf(p1, 1.0f/Vec3Length(p1));

	return plane_construct(p1, p0);
}

u32 dcel_face_projected_point_test(const struct dcel *h, mat3 rot, const vec3 pos, const u32 fi, const vec3 p)
{
	vec3 f_n, p_n;

	dcel_face_normal(p_n, h, fi);
	mat3_vec_mul(f_n, rot, p_n);

	f32 min_p = 0.0f;
	f32 max_p = 1.0f;

	struct dcel_face *f = h->f + fi;
	for (u32 i = 0; i < f->count; ++i)
	{
		const u32 e0 = f->first + i;
		const u32 e1 = f->first + ((i + 1) % f->count);
		struct plane clip_plane = dcel_face_clip_plane(h, rot, pos, f_n, e0, e1);
		if (Vec3Dot(clip_plane.normal, p) > clip_plane.signed_distance)
		{
			return 0;
		}
	}	

	return 1;
}

void dcel_edge_direction(vec3 dir, const struct dcel *h, const u32 ei)
{
	struct dcel_edge *e0 = h->e + ei;
	struct dcel_face *f = h->f + e0->face_ccw;
	const u32 next = f->first + ((ei - f->first + 1) % f->count);
	struct dcel_edge *e1 = h->e + next;
	Vec3Sub(dir, h->v[e1->origin], h->v[e0->origin]);
}

void dcel_edge_normal(vec3 dir, const struct dcel *h, const u32 ei)
{
	dcel_edge_direction(dir, h, ei);
	Vec3ScaleSelf(dir, 1.0f / Vec3Length(dir));
}

struct segment dcel_edge_segment(const struct dcel *h, mat3 rot, const vec3 pos, const u32 ei)
{
	vec3 p0, p1;
	const u32 first = h->f[h->e[ei].face_ccw].first;
	const u32 count = h->f[h->e[ei].face_ccw].count;
	const u32 e0 = ei;
	const u32 e1 = first + ((ei - first + 1) % count); 

	mat3_vec_mul(p0, rot, h->v[h->e[e0].origin]);
	mat3_vec_mul(p1, rot, h->v[h->e[e1].origin]);
	Vec3Translate(p0, pos);
	Vec3Translate(p1, pos);

	return segment_construct(p0, p1);
}

void sphere_support(vec3 support, const vec3 dir, const struct sphere *sph, const vec3 pos)
{
	Vec3Scale(support, dir, sph->radius / Vec3Length(dir));
	Vec3Translate(support, pos);
}

void capsule_support(vec3 support, const vec3 dir, const struct capsule *cap, mat3 rot, const vec3 pos)
{
	vec3 p1, p2;
	p1[0] = rot[1][0] * cap->half_height,	
	p1[1] = rot[1][1] * cap->half_height,	
	p1[2] = rot[1][2] * cap->half_height,	
	Vec3Negate(p2, p1);

	Vec3Scale(support, dir, cap->radius / Vec3Length(dir));
	Vec3Translate(support, pos);
	(Vec3Dot(dir, p1) > Vec3Dot(dir, p2))
		? Vec3Translate(support, p1) 
		: Vec3Translate(support, p2);
}

u32 dcel_support(vec3 support, const vec3 dir, const struct dcel *dcel, mat3 rot, const vec3 pos)
{
	f32 max = -F32_INFINITY;
	u32 max_index = 0;
	vec3 p;
	for (u32 i = 0; i < dcel->v_count; ++i)
	{
		mat3_vec_mul(p, rot, dcel->v[i]);
		const f32 dot = Vec3Dot(p, dir);
		if (max < dot)
		{
			max_index = i;
			max = dot; 
		}
	}

	mat3_vec_mul(support, rot, dcel->v[max_index]);
	Vec3Translate(support, pos);
	return max_index;
}

struct dcel dcel_empty(void)
{
	struct dcel dcel = { 0 };

	return dcel;
}

void dcel_print(const struct dcel *dcel)
{	
	fprintf(stderr, "dcel[%p]\n{\n", dcel);	

	fprintf(stderr, "\tv[%u]\n\t{\n", dcel->v_count);
	for (u32 i = 0; i < dcel->v_count; ++i)
	{
		fprintf(stderr, "\t\t{ %f, %f, %f }\n"
				, dcel->v[i][0]
				, dcel->v[i][1]
				, dcel->v[i][2]);
	}
	fprintf(stderr, "\t}\n");

	fprintf(stderr, "\tf[%u]\n\t{\n", dcel->f_count);
	for (u32 i = 0; i < dcel->f_count; ++i)
	{

		fprintf(stderr, "\t\tf(%u)\n\t\t{\n", i);
		fprintf(stderr, "\t\t\te[%u]\n\t\t\t{\n", dcel->f[i].count);
		for (u32 ei = 0; ei < dcel->f[i].count; ++ei)
		{
			fprintf(stderr, "\t\t\t\te(%u) = { origin : %u, twin : %u, ccw : %u }\n" 
					, dcel->f[i].first + ei
					, dcel->e[dcel->f[i].first + ei].origin
					, dcel->e[dcel->f[i].first + ei].twin
					, dcel->e[dcel->f[i].first + ei].face_ccw);
		}
		fprintf(stderr, "\t\t\t}\n");
		fprintf(stderr, "\t\t}\n");
	}
	fprintf(stderr, "\t}\n");

	fprintf(stderr, "}\n");	
}

void dcel_assert_topology(struct dcel *dcel)
{
	struct dcel_face *f;
	struct dcel_edge *e;
	for (u32 i = 0; i < dcel->f_count; ++i)
	{
		f = dcel->f + i;
		e = dcel->e + f->first;
		for (u32 j = 0; j < f->count; ++j)
		{
			assert(e->face_ccw == i);
 			e = dcel->e + f->first + j + 1;
		}

		if (f->first + f->count < dcel->e_count)
		{
			assert(e->face_ccw != i);
		}
	}

	for (u32 i = 0; i < dcel->e_count; ++i)
	{
		e = dcel->e + i;
		assert(i == (dcel->e + e->twin)->twin);
	}
}

struct ddcel_face
{
	POOL_SLOT_STATE;
	struct dll	ce_list;
	vec3		normal;
	u32 		first;	/* first half edge */
	u32 		count;	/* edge count */
};

struct ddcel_edge
{
	POOL_SLOT_STATE;
	u32 		origin;		/* vertex index origin */
	u32 		twin; 		/* twin half edge */
	u32 		next;		/* next ccw edge */
	u32 		prev;		/* prev ccw edge */
	u32 		face_ccw; 	/* face to the left of half edge */
	u32		horizon; /* used in horizon derivation step */
};

struct conflict_edge
{
	DLL_SLOT_STATE;
	DLL2_SLOT_STATE;
	u32 vertex;
	u32 face;
	POOL_SLOT_STATE;
};

struct conflict_vertex
{
	struct dll	ce_list;
	u32		index;
	/* Needed in last step of iteration */
	u32		last_iter;	/* last iteration it was added to a face's conflict list*/
	u32		last_face;	/* last iteration it was added to a face's conflict list*/
};

struct horizon_vertex
{
	u32	edge1;			/* new edge; going INTO conflict vertex */
	u32	edge2;			/* new edge; going OUT OF  conflict vertex */
	u32 	edge_in;		/* edge going in to vertex */
	u32 	edge_out;		/* edge going out from vertex */
	u32 	edge_out_twin_face;	/* face of edge_out's twin */
	u32	next;			/* next horizon vertex */
	u32	colinear;		/* is twin face colinear */
};

/*
 * (Computational Geometry Algorithms and Applications, Section 2.2) 
 * ddcel - **dynamic** doubly-connected edge list. similar to a dcel but contains extra information in order to
 * construct itself iteratively. 
 */
struct ddcel
{
	struct pool 		face_pool;
	struct pool		edge_pool;
	/* pools are not growable, so safe to use these */
	struct ddcel_face *	f;		
	struct ddcel_edge *	e;
	const vec3ptr		v;
	u32 			v_count;

	/* internal */
	struct arena		tmp1;
	struct pool 		ce_pool;
	struct conflict_edge *	ce;
	struct conflict_vertex *cv;
	struct horizon_vertex * hv;
};

static void ddcel_face_set(struct ddcel_face *face, const u32 first, const u32 count)
{
	face->first = first;
	face->count = count;
	face->ce_list = dll2_Init(struct conflict_edge);
}

static void ddcel_edge_set(struct ddcel_edge *edge, const u32 origin, const u32 twin, const u32 prev, const u32 next, const u32 face_ccw)
{
	edge->origin = origin;
	edge->twin = twin;
	edge->next = next;
	edge->prev = prev;
	edge->face_ccw = face_ccw;
	edge->horizon = 0;
}

static void ddcel_assert_topology(const struct ddcel *ddcel)
{
	struct arena tmp = ArenaAlloc1MB();

	u32 face_count = 0;
	u32 vertex_count = 0;

	u32 *vertex_check = ArenaPushZero(&tmp, ddcel->v_count * sizeof(u32));
	u32 *edge_check = ArenaPushZero(&tmp, ddcel->edge_pool.count * sizeof(u32));
	u32 *face_check = ArenaPushZero(&tmp, 3*ddcel->v_count * sizeof(u32));

	for (u32 i = 0; i < ddcel->edge_pool.count; ++i)
	{
		if (PoolSlotAllocated(ddcel->e + i) && !edge_check[i])
		{
			face_count += 1;
			u32 next;
		        u32 prev; 
			u32 current = i;
			u32 edge_count = 0;
			do
			{
				edge_count += 1;
				const struct ddcel_edge *c = ddcel->e + current;
				const struct ddcel_edge *p = ddcel->e + c->prev;
				const struct ddcel_edge *n = ddcel->e + c->next;
				const struct ddcel_edge *t = ddcel->e + c->twin;

				ds_Assert(c->horizon == 0);
				ds_Assert(c->origin < ddcel->v_count);
				ds_Assert(p->next == current);
				ds_Assert(n->prev == current);
				ds_Assert(t->twin == current);
				ds_Assert(t->origin == n->origin);

				edge_check[current] = 1;
				vertex_count += (1 - vertex_check[c->origin]);
				vertex_check[c->origin] = 1;

				current = c->next;
			} while (current != i);

			ds_Assert(edge_count >= 3);
		}
	}

	vec3 center = { 0 };
	for (u32 i = 0; i < ddcel->v_count; ++i)
	{
		if (vertex_check[i])
		{
			Vec3Translate(center, ddcel->v[i]);
		}
	}
	Vec3ScaleSelf(center, 1.0f/vertex_count);

	for (u32 i = 0; i < ddcel->face_pool.count_max; ++i)
	{
		if (PoolSlotAllocated(ddcel->f + i))
		{
			vec3 diff;
			Vec3Sub(diff, center, ddcel->v[ddcel->e[ddcel->f[i].first].origin]);
			ds_Assert(Vec3Dot(diff, ddcel->f[i].normal) < 0.0f);
		}
	}

	ArenaFree1MB(&tmp);

	ds_Assert(face_count >= 4);
}

u32 internal_convex_hull_tetrahedron_indices(struct ddcel *ddcel, const f32 tol)
{
	vec3 a, b, n;

	const f32 tol_sq = tol*tol;
	u32 indices[4] = { 0 };
	u32 i = 1;
	/* Find two points not to close to each other */
	for (; i < ddcel->v_count; ++i)
	{
		Vec3Sub(a, ddcel->v[ddcel->cv[i].index], ddcel->v[ddcel->cv[0].index]);
		const f32 dist_sq = Vec3Dot(a, a);
		if (dist_sq > tol_sq)
		{
			//Vec3ScaleSelf(a, 1.0f / len);
			indices[1] = i;
			i += 1;
			break;
		}
	}	

	/* Find non-collinear point */
	for (; i < ddcel->v_count; ++i)
	{
		Vec3Sub(b, ddcel->v[ddcel->cv[i].index], ddcel->v[ddcel->cv[0].index]);
		Vec3Cross(n, a, b);
		const f32 dist = Vec3Length(n);
		const f32 area = dist / 2.0f;
		if (area > tol_sq)
		{
			indices[2] = i;
			i += 1;
			Vec3ScaleSelf(n, 1.0f / dist);
			break;
		}
	}

	/* Find non-coplanar point */
	for (; i < ddcel->v_count; ++i)
	{
		Vec3Sub(a, ddcel->v[ddcel->cv[i].index], ddcel->v[ddcel->cv[0].index]);
		const f32 height = Vec3Dot(a, n);
		if (f32_abs(height) > tol)
		{
			indices[3] = i;
			break;
		}
	}

	for (u32 j = 0; j < 4; ++j)
	{
		struct conflict_vertex tmp = ddcel->cv[j];
		ddcel->cv[j] = ddcel->cv[indices[j]];
		ddcel->cv[indices[j]] = tmp;
	}

	return i < ddcel->v_count;
}

static void internal_convex_hull_tetrahedron_ddcel(struct ddcel *ddcel, const f32 tol)
{
	const vec3ptr v = ddcel->v;
	const u32 v_count = ddcel->v_count;
	vec3 a, b, c, cr;
	Vec3Sub(a, v[ddcel->cv[1].index], v[ddcel->cv[0].index]);
	Vec3Sub(b, v[ddcel->cv[2].index], v[ddcel->cv[0].index]);
	Vec3Sub(c, v[ddcel->cv[3].index], v[ddcel->cv[0].index]);
	Vec3Cross(cr, a, b);

	/* CCW == inside gives negative dot product for any polygon on a convex polyhedron */
	if (Vec3Dot(cr, c) > 0.0f)
	{
		/* Make 0->1->2->0 CCW */
		const u32 tmp = ddcel->cv[1].index;
		ddcel->cv[1].index = ddcel->cv[2].index;
		ddcel->cv[2].index = tmp;
	}

	struct ddcel_face *f0 = PoolAdd(&ddcel->face_pool).address;
	struct ddcel_face *f1 = PoolAdd(&ddcel->face_pool).address;
	struct ddcel_face *f2 = PoolAdd(&ddcel->face_pool).address;
	struct ddcel_face *f3 = PoolAdd(&ddcel->face_pool).address;

	struct ddcel_edge *e0 = PoolAdd(&ddcel->edge_pool).address;
	struct ddcel_edge *e1 = PoolAdd(&ddcel->edge_pool).address;
	struct ddcel_edge *e2 = PoolAdd(&ddcel->edge_pool).address;
	struct ddcel_edge *e3 = PoolAdd(&ddcel->edge_pool).address;
	struct ddcel_edge *e4 = PoolAdd(&ddcel->edge_pool).address;
	struct ddcel_edge *e5 = PoolAdd(&ddcel->edge_pool).address;
	struct ddcel_edge *e6 = PoolAdd(&ddcel->edge_pool).address;
	struct ddcel_edge *e7 = PoolAdd(&ddcel->edge_pool).address;
	struct ddcel_edge *e8 = PoolAdd(&ddcel->edge_pool).address;
	struct ddcel_edge *e9 = PoolAdd(&ddcel->edge_pool).address;
	struct ddcel_edge *e10 = PoolAdd(&ddcel->edge_pool).address;
	struct ddcel_edge *e11 = PoolAdd(&ddcel->edge_pool).address;

	ddcel_face_set(f0, 0, 3);
	ddcel_face_set(f1, 3, 3);
	ddcel_face_set(f2, 6, 3);
	ddcel_face_set(f3, 9, 3);

	ddcel_edge_set(e0,  ddcel->cv[0].index,  4,  2,  1, 0);
	ddcel_edge_set(e1,  ddcel->cv[1].index, 10,  0,  2, 0);
	ddcel_edge_set(e2,  ddcel->cv[2].index,  7,  1,  0, 0);

	ddcel_edge_set(e3,  ddcel->cv[3].index, 11,  5,  4, 1);
	ddcel_edge_set(e4,  ddcel->cv[1].index,  0,  3,  5, 1);
	ddcel_edge_set(e5,  ddcel->cv[0].index,  6,  4,  3, 1);

	ddcel_edge_set(e6,  ddcel->cv[3].index,  5,  8,  7, 2);
	ddcel_edge_set(e7,  ddcel->cv[0].index,  2,  6,  8, 2);
	ddcel_edge_set(e8,  ddcel->cv[2].index,  9,  7,  6, 2);

	ddcel_edge_set(e9,  ddcel->cv[3].index,  8, 11, 10, 3);
	ddcel_edge_set(e10, ddcel->cv[2].index,  1,  9, 11, 3);
	ddcel_edge_set(e11, ddcel->cv[1].index,  3, 10,  9, 3);

	Vec3Sub(a, ddcel->v[e1->origin], ddcel->v[e0->origin]);
	Vec3Sub(b, ddcel->v[e2->origin], ddcel->v[e0->origin]);
	Vec3Cross(cr, a, b);
	Vec3Normalize(ddcel->f[0].normal, cr);

	Vec3Sub(a, ddcel->v[e4->origin], ddcel->v[e3->origin]);
	Vec3Sub(b, ddcel->v[e5->origin], ddcel->v[e3->origin]);
	Vec3Cross(cr, a, b);
	Vec3Normalize(ddcel->f[1].normal, cr);

	Vec3Sub(a, ddcel->v[e7->origin], ddcel->v[e6->origin]);
	Vec3Sub(b, ddcel->v[e8->origin], ddcel->v[e6->origin]);
	Vec3Cross(cr, a, b);
	Vec3Normalize(ddcel->f[2].normal, cr);

	Vec3Sub(a, ddcel->v[e10->origin], ddcel->v[e9->origin]);
	Vec3Sub(b, ddcel->v[e11->origin], ddcel->v[e9->origin]);
	Vec3Cross(cr, a, b);
	Vec3Normalize(ddcel->f[3].normal, cr);

	ddcel_assert_topology(ddcel);
}

static void internal_convex_hull_tetrahedron_conflicts(struct ddcel *ddcel, const f32 tol)
{
	const vec3ptr v = ddcel->v;
	const u32 v_count = ddcel->v_count;
	vec3 b;
	for (u32 cv_i = 4; cv_i < v_count; ++cv_i)
	{
		struct conflict_vertex *cv = ddcel->cv + cv_i;
		for (u32 f_i = 0; f_i < 4; ++f_i)
		{
			const u32 v0_i = ddcel->e[ddcel->f[f_i].first].origin;
			Vec3Sub(b, v[cv->index], v[v0_i]);
			/* If point is "in front" of face, we have a conflict */
			if (Vec3Dot(ddcel->f[f_i].normal, b) > tol)
			{
				struct slot slot = PoolAdd(&ddcel->ce_pool);
				dll_Append(&cv->ce_list, ddcel->ce_pool.buf, slot.index);
				dll_Append(&ddcel->f[f_i].ce_list, ddcel->ce_pool.buf, slot.index);

				struct conflict_edge *edge = slot.address;
				edge->vertex = cv_i;
				edge->face = f_i;
			}
		}
	}
}

void convex_hull_iteration(struct ddcel *ddcel, const u32 cvi, const f32 tol)
{
	if (ddcel->cv[cvi].ce_list.count == 0) { return; }

	struct conflict_vertex *cv = ddcel->cv + cvi;
	struct conflict_edge *ce = NULL;

	for (u32 i = cv->ce_list.first; i != DLL_NULL; i = dll_Next(ce))
	{
		ce = ddcel->ce + i;
		const u32 fi = ce->face;
		struct ddcel_edge *e = ddcel->e + ddcel->f[fi].first;
		for (u32 j = 0; j < ddcel->f[fi].count; ++j)
		{
			struct ddcel_edge *twin = ddcel->e + e->twin;
			ds_Assert(e->horizon == 0);
			ds_Assert(twin->horizon == 0);
		}
	}
	
	/* (5) Get horizon edges:
	 * At this point, all edges has horizon set to 0. Whenever we visit an edge, 
	 * we flip the edge and its twin's horizon value. After the loop the horizon
	 * will consist of the edges with value 1.  */
	for (u32 i = cv->ce_list.first; i != DLL_NULL; i = dll_Next(ce))
	{
		ce = ddcel->ce + i;
		const u32 fi = ce->face;
		struct ddcel_edge *e = ddcel->e + ddcel->f[fi].first;
		for (u32 j = 0; j < ddcel->f[fi].count; ++j)
		{
			struct ddcel_edge *twin = ddcel->e + e->twin;
			//fprintf(stderr, "horizon edge par (%u,%u)\n", PoolIndex(&ddcel->edge_pool, e), e->twin);
			e->horizon += 1;
			twin->horizon += 1;
			ds_Assert(e->horizon <= 2 && twin->horizon <= 2);
			ds_Assert(twin->twin == PoolIndex(&ddcel->edge_pool, e));
			e = ddcel->e + e->next;
		}
	}

	/* (6) loop through and remove all faces and non-horizon edges. Sort horizon edges in a 
	 * ad-hoc dll structure horizon vertex and cache additional data.
	 */
	u32 horizon = 0;
	u32 horizon_count = 0;
	for (u32 i = cv->ce_list.first; i != DLL_NULL; i = dll_Next(ce))
	{
		ce = ddcel->ce + i;
		const u32 fi = ce->face;
		struct ddcel_face *f = ddcel->f + fi;

		u32 ej = f->first;
		for (u32 j = 0; j < f->count; ++j)
		{
			struct ddcel_edge *e = ddcel->e + ej;
			const u32 next = e->next;
			if (e->horizon == 2)
			{
				PoolRemove(&ddcel->edge_pool, ej);	
			}
			else
			{
				//fprintf(stderr, "horizon edge par (%u,%u)\n", ej, e->twin);
				struct ddcel_edge *e_twin = ddcel->e + e->twin;
				struct ddcel_face *f_twin = ddcel->f + e_twin->face_ccw;

				ds_Assert(e->horizon == 1);
				ds_Assert(e_twin->horizon == 1);
				e->horizon = 0;
				e_twin->horizon = 0;

				horizon_count += 1;
				horizon = e->origin;

				ddcel->hv[e->origin].edge_out_twin_face = e_twin->face_ccw;
				ddcel->hv[e->origin].edge_out = ej;
				ddcel->hv[e->origin].next = e_twin->origin;
				ddcel->hv[e_twin->origin].edge_in = ej;

				vec3 diff;
				Vec3Sub(diff, ddcel->v[cv->index], ddcel->v[e->origin]);
				ddcel->hv[e->origin].colinear = (f32_abs(Vec3Dot(f_twin->normal, diff)) < tol)
					? 1
					: 0;
			}
			ej = next;
		}
	}

	/* (7) We don't want to start in the middle of a colinear_face, so we traverse the horizon until we find a 
	 * new triangle.
	 */
	const u32 face = ddcel->hv[horizon].edge_out_twin_face;
	u32 prev_horizon = U32_MAX;
	for (u32 i = 0; i < horizon_count; ++i)
	{
		prev_horizon = horizon;
		horizon = ddcel->hv[horizon].next;
		if (ddcel->hv[horizon].edge_out_twin_face != ddcel->hv[prev_horizon].edge_out_twin_face)
		{
			break;
		}
	}

	/* (8) Traverse the horizon creaing new faces and removing any colinear horizon
	 * twin edges. 
	 */
	for (u32 i = 0; i < horizon_count; ++i)
	{	
		const u32 twin_face = ddcel->hv[horizon].edge_out_twin_face;
		const u32 ccw_face = ddcel->e[ddcel->hv[horizon].edge_out].face_ccw;
		struct ddcel_face *f_ccw = ddcel->f + ccw_face;
		struct ddcel_face *f_twin = ddcel->f + twin_face;

		struct slot se1 = PoolAdd(&ddcel->edge_pool);
		struct slot se2 = PoolAdd(&ddcel->edge_pool);

		struct ddcel_edge *e1 = se1.address;
		struct ddcel_edge *e2 = se2.address;

		ddcel->hv[horizon].edge1 = se1.index;
		ddcel->hv[horizon].edge2 = se2.index;

		u32 prev_horizon_edge1 = U32_MAX;
		if (i != 0)
		{ 
			prev_horizon_edge1 = ddcel->hv[prev_horizon].edge1;
			ddcel->e[ddcel->hv[prev_horizon].edge1].twin = se2.index;
		}

		if (ddcel->hv[horizon].colinear)
		{
			u32 edge = ddcel->hv[horizon].edge_out;
			const u32 twin_start_next = ddcel->e[
					            ddcel->e[edge].twin]
							          .next;
			//fprintf(stderr, "extending face %u\n", twin_face);
			u32 twin_end_prev;
			f_twin->first = se1.index;
			f_twin->count += 2;
			i -= 1;
			do
			{
				f_twin->count -= 1;
				i += 1;
				twin_end_prev = ddcel->e[
					        ddcel->e[edge].twin]
					                      .prev;
				prev_horizon = horizon;
				horizon = ddcel->hv[horizon].next;

				ddcel->hv[prev_horizon].edge1 = se1.index;
				ddcel->hv[prev_horizon].edge2 = se2.index;

				PoolRemove(&ddcel->edge_pool, ddcel->e[edge].twin);
				PoolRemove(&ddcel->edge_pool, edge);
				edge = ddcel->hv[horizon].edge_out;
			} while (ddcel->hv[horizon].edge_out_twin_face == twin_face);

			ddcel_edge_set(e1, horizon, U32_MAX, twin_end_prev, se2.index, twin_face);
			ddcel_edge_set(e2, cv->index, prev_horizon_edge1, se1.index, twin_start_next, twin_face);
			ddcel->e[twin_end_prev].next = se1.index;
			ddcel->e[twin_start_next].prev = se2.index;

			/* Note: Since we reuse the twin face, we also reuse its list of conflicts, so we are done. */
		}
		else
		{
			struct slot sf = PoolAdd(&ddcel->face_pool);
			//fprintf(stderr, "added face %u\n", sf.index);

			const u32 e0i = ddcel->hv[horizon].edge_out;

			struct ddcel_face *f = sf.address;
			struct ddcel_edge *e0 = ddcel->e + e0i;

			ddcel_edge_set(e2, cv->index, ddcel->hv[prev_horizon].edge1, se1.index, e0i, sf.index);
			ddcel_edge_set(e1, ddcel->e[e0->twin].origin, U32_MAX, e0i, se2.index, sf.index);
			ddcel_edge_set(e0, e0->origin, e0->twin, se2.index, se1.index, sf.index);

			ddcel_face_set(f, e0i, 3);
			vec3 a, b, cr;
			Vec3Sub(a, ddcel->v[e1->origin], ddcel->v[e0->origin]);
			Vec3Sub(b, ddcel->v[e2->origin], ddcel->v[e0->origin]);
			Vec3Cross(cr, a, b);
			Vec3Normalize(f->normal, cr);
		
			/*TODO: We may add same point twich here, need to add a "has_been_mapped" thingy to not add again*/
			ce = NULL;
			for (u32 j = f_ccw->ce_list.first; j != DLL_NULL; j = dll2_Next(ce))
			{
				ce = ddcel->ce + j;
				if (ce->vertex != cvi && (ddcel->cv[ce->vertex].last_face != sf.index || ddcel->cv[ce->vertex].last_iter != cvi))
				{
					ddcel->cv[ce->vertex].last_iter = cvi;
					ddcel->cv[ce->vertex].last_face = sf.index;
					vec3 diff;
					Vec3Sub(diff, ddcel->v[ddcel->cv[ce->vertex].index], ddcel->v[e0->origin]);
					if (Vec3Dot(f->normal, diff) > tol)
					{
						struct slot slot = PoolAdd(&ddcel->ce_pool);
						dll_Append(&f->ce_list, ddcel->ce_pool.buf, slot.index);
						dll_Append(&ddcel->cv[ce->vertex].ce_list, ddcel->ce_pool.buf, slot.index);

						struct conflict_edge *new = slot.address;
						new->vertex = ce->vertex;
						new->face = sf.index;
					}
				}
			}

			ce = NULL;
			for (u32 j = f_twin->ce_list.first; j != DLL_NULL; j = dll2_Next(ce))
			{
				ce = ddcel->ce + j;
				vec3 diff;
				ds_Assert(ce->vertex != cvi)
				if (ddcel->cv[ce->vertex].last_face != sf.index || ddcel->cv[ce->vertex].last_iter != cvi)
				{
					ddcel->cv[ce->vertex].last_iter = cvi;
					ddcel->cv[ce->vertex].last_face = sf.index;
					Vec3Sub(diff, ddcel->v[ddcel->cv[ce->vertex].index], ddcel->v[e0->origin]);
					if (Vec3Dot(f->normal, diff) > tol)
					{
						struct slot slot = PoolAdd(&ddcel->ce_pool);
						dll_Append(&f->ce_list, ddcel->ce_pool.buf, slot.index);
						dll_Append(&ddcel->cv[ce->vertex].ce_list, ddcel->ce_pool.buf, slot.index);

						struct conflict_edge *new = slot.address;
						new->vertex = ce->vertex;
						new->face = sf.index;
					}
				}
			}

			prev_horizon = horizon;
			horizon = ddcel->hv[horizon].next;
		}
	}
	ddcel->e[ddcel->hv[prev_horizon].edge1].twin = ddcel->hv[horizon].edge2;
	ddcel->e[ddcel->hv[horizon].edge2].twin = ddcel->hv[prev_horizon].edge1;

	/* (9) Update all conflict_edge lists of vertices conflicting with face,
	 * and remove all conflicting edges to face.  */
	for (u32 i = cv->ce_list.first; i != DLL_NULL;)
	{
		ce = ddcel->ce + i;
		i = dll_Next(ce);
		const u32 fi = ce->face;
		struct ddcel_face *f = ddcel->f + fi;

		ce = NULL;
		for (u32 j = f->ce_list.first; j != DLL_NULL; )
		{
			ce = ddcel->ce + j;
			const u32 next = dll2_Next(ce);

			struct conflict_vertex *cvj = ddcel->cv + ce->vertex;
			dll_Remove(&cvj->ce_list, ddcel->ce_pool.buf, j);
			PoolRemove(&ddcel->ce_pool, j);

			j = next;
		}
		PoolRemove(&ddcel->face_pool, fi);
		//fprintf(stderr, "removed face %u\n", fi);
	}
}

struct dcel dcel_ddcel(struct arena *mem, const struct ddcel *ddcel)
{
	ArenaPushRecord(mem);
	struct dcel cpy =
	{
		.v = ArenaPushMemcpy(mem, ddcel->v, ddcel->v_count*sizeof(vec3)),
		.e = ArenaPush(mem, ddcel->edge_pool.count*sizeof(struct dcel_edge)),
		.f = ArenaPush(mem, ddcel->face_pool.count*sizeof(struct dcel_face)),
		.v_count = ddcel->v_count,
		.e_count = ddcel->edge_pool.count,
		.f_count = ddcel->face_pool.count,
	};


	if (cpy.v && cpy.e && cpy.f)
	{
		ArenaPushRecord(mem);
		u32 *emap = ArenaPush(mem, sizeof(u32) * ddcel->edge_pool.count_max);
		u32 off = 0;
		/* set dcel edge index into ddcel->edge.prev temporarily */
		for (u32 fj = 0; fj < ddcel->face_pool.count_max; ++fj)
		{
			if (PoolSlotAllocated(ddcel->f + fj))
			{
				u32 next = ddcel->f[fj].first;
				for (u32 ei = 0; ei < ddcel->f[fj].count; ++ei)
				{
					emap[next] = off + ei;
					next = ddcel->e[next].next;
				}
				off += ddcel->f[fj].count;
			}
		}

		off = 0;
		for (u32 fi = 0, fj = 0; fi < cpy.f_count; fj += 1)
		{
			ds_Assert(fj < ddcel->face_pool.count_max);
			if (PoolSlotAllocated(ddcel->f + fj))
			{
				cpy.f[fi].count = ddcel->f[fj].count;
				cpy.f[fi].first = off;
				u32 next = ddcel->f[fj].first;
				for (u32 ei = 0; ei < ddcel->f[fj].count; ++ei)
				{
					cpy.e[off + ei].origin = ddcel->e[next].origin;
					cpy.e[off + ei].face_ccw = fi;
					cpy.e[off + ei].twin = emap[ddcel->e[next].twin];
					next = ddcel->e[next].next;
				}
				off += ddcel->f[fj].count;
				fi += 1;
			}
		}

		//dcel_print(&cpy);
		//dcel_assert_topology(&cpy);
		ArenaPopRecord(mem);
		ArenaRemoveRecord(mem);
	}
	else
	{
		ArenaPopRecord(mem);
		cpy = dcel_empty();
	}

	return cpy;
}

struct dcel dcel_convex_hull(struct arena *mem, const vec3ptr v, const u32 v_count, const f32 tol)
{
	struct dcel dcel = dcel_empty();
	if (v_count < 4) { goto end; }	

	struct arena tmp1 = ArenaAlloc1MB();
	struct arena tmp2 = ArenaAlloc1MB();

	const u32 edge_count_upper_bound = 6*v_count - 12;
	const u32 face_count_upper_bound = 2*v_count - 4;
	struct ddcel ddcel =
	{
		.face_pool = PoolAlloc(&tmp1, 2*face_count_upper_bound, struct ddcel_face, NOT_GROWABLE),	/* add additional space for easier memory management */
		.edge_pool = PoolAlloc(&tmp1, 2+edge_count_upper_bound, struct ddcel_edge, NOT_GROWABLE),	/* add additional space for easier memory management */
		.v = v,
		.v_count = v_count,
		.ce_pool = PoolAlloc(&tmp2, tmp2.mem_size / sizeof(struct conflict_edge), struct conflict_edge, NOT_GROWABLE),
		.cv = ArenaPush(&tmp1, v_count * sizeof(struct conflict_vertex)),
		.hv = ArenaPush(&tmp1, v_count * sizeof(struct horizon_vertex)),
	};

	ddcel.tmp1 = tmp1;
	ddcel.e = (struct ddcel_edge *) ddcel.edge_pool.buf;
	ddcel.f = (struct ddcel_face *) ddcel.face_pool.buf;
	ddcel.ce = (struct conflict_edge *) ddcel.ce_pool.buf;

	/* (1) permutation - Random permutation of remaining points */
	for (u32 i = 0; i < v_count; ++i)
	{
		ddcel.cv[i].ce_list = dll_Init(struct conflict_edge);
		ddcel.cv[i].index = i;
		ddcel.cv[i].last_iter = U32_MAX;
		ddcel.cv[i].last_face = U32_MAX;
	}
	for (u32 i = 0; i < v_count; ++i)
	{
		const u32 rng = (u32) rng_u64_range(i, v_count-1);
		const u32 tmp = ddcel.cv[i].index;
		ddcel.cv[i].index = ddcel.cv[rng].index;
		ddcel.cv[rng].index = tmp;
	}

	/* (2) Get inital points for tetrahedron */
	if (internal_convex_hull_tetrahedron_indices(&ddcel, tol) == 0) { goto end; }

	/* (3) initiate DCEL from points */
	internal_convex_hull_tetrahedron_ddcel(&ddcel, tol);

	/* (4) setup conflict graph */
	internal_convex_hull_tetrahedron_conflicts(&ddcel, tol);

	/* iteratetively solve and add conflicts until no vertices left */
	for (u32 i = 4; i < v_count; ++i)
	{
		convex_hull_iteration(&ddcel, i, tol);
		ddcel_assert_topology(&ddcel);
	}

	dcel = dcel_ddcel(mem, &ddcel);	
end:
	ArenaFree1MB(&tmp1);
	ArenaFree1MB(&tmp2);

	return dcel;
}

struct AABB tri_mesh_bbox(const struct tri_mesh *mesh)
{
	struct AABB bbox = { 0 };	
	AABB_vertex(&bbox, mesh->v, mesh->v_count, 0.0f);	
	return bbox;
}

f32 tri_ccw_raycast_parameter(const struct tri_mesh *mesh, const u32 tri, const struct ray *ray)
{
	vec3 intersection;
	f32 t = F32_INFINITY;
	if (tri_ccw_raycast(intersection, mesh, tri, ray))
	{
		t = ray_point_closest_point_parameter(ray, intersection);
	}
	return t;
}

u32 tri_ccw_raycast(vec3 intersection, const struct tri_mesh *mesh, const u32 tri, const struct ray *ray)
{
	/* TODO(Optimization): 
	 * By precomputation and extending our tri_mesh structure, we can avoid these branches;
	 * so if it becomes relevant, we need to precompute the edge sorting or something...  
	 */
	/* canonicalize edges: We require consistency between triangles sharing edges; if the 
	 * test determining which side of the edge the ray intersects the triangle planes are
	 * not done in the same way, we may miss collision with both triangles despite hitting
	 * one of them. See (Real Time Collision Detection, 5.3.4 and 11.3.3) for algorithm and
	 * robustness discussion. */

	vec3 p0, p1, p2, c;

	Vec3Sub(p0, mesh->v[mesh->tri[tri][1]],mesh->v[mesh->tri[tri][0]]);
	Vec3Sub(p1, mesh->v[mesh->tri[tri][2]],mesh->v[mesh->tri[tri][0]]);
	Vec3Cross(p2, p0, p1);
	ds_Assert(p2[1] > 0.0f);

	Vec3Sub(p0, mesh->v[mesh->tri[tri][0]], ray->origin);
	Vec3Sub(p1, mesh->v[mesh->tri[tri][1]], ray->origin);
	Vec3Sub(p2, mesh->v[mesh->tri[tri][2]], ray->origin);

	f32 u;
	if (mesh->tri[tri][0] < mesh->tri[tri][1])
	{
		Vec3Cross(c, p1, p0);
		u = Vec3Dot(ray->dir, c);
	}
	else
	{
		Vec3Cross(c, p0, p1);
		u = -Vec3Dot(ray->dir, c);
	}
	if (u < 0.0f) { return 0; }

	f32 v;
	if (mesh->tri[tri][1] < mesh->tri[tri][2])
	{
		Vec3Cross(c, p2, p1);
		v = Vec3Dot(ray->dir, c);
	}
	else
	{
		Vec3Cross(c, p1, p2);
		v = -Vec3Dot(ray->dir, c);
	}
	if (v < 0.0f) { return 0; }

	f32 w;
	if (mesh->tri[tri][2] < mesh->tri[tri][0])
	{
		Vec3Cross(c, p0, p2);
		w = Vec3Dot(ray->dir, c);
	}
	else
	{
		Vec3Cross(c, p2, p0);
		w = -Vec3Dot(ray->dir, c);
	}
	if (w < 0.0f) { return 0; }

	/* TODO: Prob bad, we can go back to this later */
	if (u + v + w < 100.0f * F32_EPSILON)
	{
		Vec3Copy(intersection, ray->origin);
	}
	else
	{
		const f32 denom = 1.0f / (u + v + w);
		u *= denom;
		v *= denom;
		w *= 1.0f - u - v;

		Vec3Scale(intersection, mesh->v[mesh->tri[tri][0]], v);
		Vec3TranslateScaled(intersection, mesh->v[mesh->tri[tri][1]], w);
		Vec3TranslateScaled(intersection, mesh->v[mesh->tri[tri][2]], u);
	}

	return 1;
}
