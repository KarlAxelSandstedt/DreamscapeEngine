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
#include "ds_base.h"
#include "ds_math.h"
#include "geometry.h"
#include "ds_hash_map.h"
#include "list.h"
#include "queue.h"
#include "float32.h"

//TODO what to do with this?
#define MIN_SEGMENT_LENGTH_SQ	(100.0f*F32_EPSILON)

struct ray RayConstruct(const vec3 origin, const vec3 dir)
{
	ds_Assert(Vec3LengthSquared(dir) > 0.0f);

	struct ray r;
	Vec3Copy(r.origin, origin);
	Vec3Copy(r.dir, dir);
	return r;
}

struct segment RayConstructSegment(const struct ray *r, const f32 t)
{
	vec3 p;
	Vec3Copy(p, r->origin);
	Vec3TranslateScaled(p, r->dir, t);
	return SegmentConstruct(r->origin, p);
}


void RayPoint(vec3 RayPoint, const struct ray *ray, const f32 t)
{
	Vec3Copy(RayPoint, ray->origin);
	Vec3TranslateScaled(RayPoint, ray->dir, t);
}

struct sphere SphereConstruct(const vec3 center, const f32 radius)
{
	struct sphere sph = { .radius = radius };
	Vec3Copy(sph.center, center);
	return sph;	
}

/*
 * | r.o + t*r.dir - s.c |^2 = s.r^2 => solve quadratic formula give the following method.
 */
f32 SphereRaycastParameter(const struct sphere *sph, const struct ray *ray)
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

u32 SphereRaycast(vec3 intersection, const struct sphere *sph, const struct ray *ray)
{
	const f32 t = SphereRaycastParameter(sph, ray);
	if (t < 0.0f || t == F32_INFINITY) { return 0; }

	Vec3Copy(intersection, ray->origin);
	Vec3TranslateScaled(intersection, ray->dir, t);
	return 1;
}

f32 RayPointClosestPointParameter(const struct ray *ray, const vec3 p)
{ 
	vec3 diff;
	Vec3Sub(diff, p, ray->origin);
	const f32 tr = Vec3Dot(diff, ray->dir) / Vec3Dot(ray->dir, ray->dir);
	return (tr >= 0.0f) ? tr : 0.0f;
}

f32 RayPointDistanceSquared(vec3 r_c, const struct ray *ray, const vec3 p)
{
	const f32 t = RayPointClosestPointParameter(ray, p);
	RayPoint(r_c, ray, t);
	return Vec3DistanceSquared(r_c, p);
}

f32 RaySegmentDistanceSquared(vec3 r_c, vec3 s_c, const struct ray *ray, const struct segment *s)
{
	vec3 diff;
	Vec3Sub(diff, s->p[0], ray->origin);
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
	
	ds_Assert(0.0f <= tr);
	ds_Assert(0.0f <= ts && ts <= 1.0f);

	RayPoint(r_c, ray, tr);
	SegmentBc(s_c, s, ts);
	return Vec3DistanceSquared(r_c, s_c);
}

struct segment SegmentConstruct(const vec3 p0, const vec3 p1)
{
	struct segment s;
	Vec3Copy(s.p[0], p0);
	Vec3Copy(s.p[1], p1);
	Vec3Sub(s.dir, p1, p0);
	return s;
}

u32 SegmentPointCheck(const struct segment *s, const f32 min_dist_sq)
{
    return (Vec3Dot(s->dir, s->dir) <= min_dist_sq);
}

u32 SegmentParallelCheck(const struct segment *s1, const struct segment *s2, const f32 eps)
{
    return Vec3ParallelCheck(s1->dir, s2->dir, eps);
}

void SegmentClosestParameter(f32 *t1, f32 *t2, const struct segment *s1, const struct segment *s2)
{
	vec3 diff;
	Vec3Sub(diff, s2->p[0], s1->p[0]);
	const f32 d1d1 = Vec3LengthSquared(s1->dir);
	const f32 d2d2 = Vec3LengthSquared(s2->dir);

	*t1 = 0.0f;
	*t2 = 0.0f;

	if (d1d1 >= MIN_SEGMENT_LENGTH_SQ && d2d2 >= MIN_SEGMENT_LENGTH_SQ)
	{
		const f32 d1d2 = Vec3Dot(s1->dir, s2->dir);
		const f32 diffd1 = Vec3Dot(diff, s1->dir);
		const f32 diffd2 = Vec3Dot(diff, s2->dir);
		const f32 denom = d1d1*d2d2 - d1d2*d1d2;
		/* Check that the segments are not parallel */
        //TODO is 0.0f good here, or should we use degree test as in SegmentParallelCheck?
		if (denom > 0.0f)
		{
			*t1 = f32_clamp((diffd1*d2d2 - diffd2*d1d2) / denom, 0.0f, 1.0f);
		}

		/*
		 *  t2 = (L1_P1*(1-t1) + L1_P2*t1 - L2_P1) * DIR2 / DIR2*DIR2
		 *     = (-DIFF + DIR1*t1) * DIR2 / DIR2*DIR2
		 *     = (-DIFF*DIR2 + DIR1*DIR2*t1) / DIR2*DIR2
		 */
		*t2 = f32_clamp(*t1*d1d2 - diffd2, 0.0f, d2d2);

		if (*t2 == 0.0f)
		{
			/*
			 *  t1 = (L2_P1*(1-t2) + L2_P2*t2 - L1_P1) * DIR1 / DIR1*DIR1
			 *     = (DIFF + DIR2*t2) * DIR1 / DIR1*DIR1
			 *     = DIFF*DIR1 / DIR1*DIR1
			 */
            *t1 = f32_clamp(diffd1 / d1d1, 0.0f, 1.0f);
		}
		else if (*t2 == d2d2)
		{
			*t2 = 1.0f;
			*t1 = f32_clamp((diffd1 + d1d2) / d1d1, 0.0f, 1.0f);
		}	
		else
		{
			*t2 /= d2d2;
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
        *t1 = f32_clamp(Vec3Dot(diff, s1->dir) / d1d1, 0.0f, 1.0f);
	}
	else if (d2d2 >= MIN_SEGMENT_LENGTH_SQ)
	{
		*t2 = f32_clamp(-Vec3Dot(diff, s2->dir) / d2d2, 0.0f, 1.0f);
	}

	ds_Assert(0.0f <= *t1 && *t1 <= 1.0f);
	ds_Assert(0.0f <= *t2 && *t2 <= 1.0f);
}

f32 SegmentDistanceSquared(vec3 c1, vec3 c2, const struct segment *s1, const struct segment *s2)
{
    f32 t1, t2;
    SegmentClosestParameter(&t1, &t2, s1, s2);
	SegmentBc(c1, s1, t1);
	SegmentBc(c2, s2, t2);
	return Vec3DistanceSquared(c1, c2);
}

f32 SegmentPointDistanceSquared(vec3 c, const struct segment *s, const vec3 p)
{
	f32 t = 0.0f;
	
	if (Vec3LengthSquared(s->dir) >= MIN_SEGMENT_LENGTH_SQ)
	{
		vec3 diff;
		Vec3Sub(diff, p, s->p[0]);
		t = f32_clamp(Vec3Dot(diff, s->dir) / Vec3Dot(s->dir, s->dir), 0.0f, 1.0f);
	}

	SegmentBc(c, s, t);
	return Vec3DistanceSquared(c, p);
}

void SegmentBc(vec3 bc_p, const struct segment *s, const f32 t)
{
	Vec3Interpolate(bc_p, s->p[1], s->p[0], t);
}
                                                                        
struct segment SegmentCapsuleTransform(const struct capsule *cap, const ds_Transform *t)
{
    vec3 p1, p0 = { 0.0f, cap->half_height, 0.0 };
    QuatVec3RotateSelf(p0, t->rotation);
	Vec3Negate(p1, p0);
	Vec3Translate(p0, t->position);
	Vec3Translate(p1, t->position);
	return SegmentConstruct(p0, p1);
}

struct aabb BboxSegment(const struct segment *s)
{
	struct aabb bbox;

	vec3 min = { s->p[0][0], s->p[0][1], s->p[0][2] };
	vec3 max = { s->p[0][0], s->p[0][1], s->p[0][2] };

    Vec3MinSelf(min, s->p[1]);
    Vec3MaxSelf(max, s->p[1]);

	Vec3Sub(bbox.hw, max, min);
	Vec3ScaleSelf(bbox.hw, 0.5f);
	Vec3Add(bbox.center, min, bbox.hw);

    return bbox;
}

f32 SegmentPointProjectedBcParameter(const struct segment *s, const vec3 p)
{	
	vec3 diff;
	Vec3Sub(diff, p, s->p[0]);
	return Vec3Dot(diff, s->dir) / Vec3Dot(s->dir, s->dir);
}

f32 SegmentPointClosestBcParameter(const struct segment *s, const vec3 p)
{	
	vec3 diff;
	Vec3Sub(diff, p, s->p[0]);
	return f32_clamp(Vec3Dot(diff, s->dir) / Vec3Dot(s->dir, s->dir), 0.0f, 1.0f);
}

struct plane PlaneConstruct(const vec3 n, const vec3 p)
{
	struct plane pl;
	Vec3Copy(pl.normal_direction, n);
    pl.inv_dot_nn = 1.0f / Vec3Dot(n,n);
	pl.signed_distance = Vec3Dot(n, p);
	return pl;
}

struct plane PlaneConstructNormalized(const vec3 n, const vec3 p)
{
	struct plane pl;
	Vec3Normalize(pl.normal, n);
    pl.inv_dot_nn = 1.0f;
	pl.signed_distance = Vec3Dot(pl.normal, p);
	return pl;
}

struct plane PlaneConstructFromCcwTriangle(const vec3 a, const vec3 b, const vec3 c)
{
	vec3 ab, ac, cross;
	Vec3Sub(ab, b, a);
	Vec3Sub(ac, c, a);
	Vec3Cross(cross, ab, ac);
	return PlaneConstruct(cross, a);
}

struct plane PlaneConstructNormalizedFromCcwTriangle(const vec3 a, const vec3 b, const vec3 c)
{
	vec3 ab, ac, cross;
	Vec3Sub(ab, b, a);
	Vec3Sub(ac, c, a);
	Vec3Cross(cross, ab, ac);
	Vec3ScaleSelf(cross, 1.0f/Vec3Length(cross));
	return PlaneConstruct(cross, a);
}

void PlaneNormalize(struct plane *pl)
{
    const f32 n_dir_len = Vec3Length(pl->normal_direction);
    Vec3ScaleSelf(pl->normal_direction, 1.0f/n_dir_len);
    pl->signed_distance /= n_dir_len;
    pl->inv_dot_nn = 1.0f;
}

u32 PlanePointInfrontCheck(const struct plane *pl, const vec3 p)
{
	return (PlanePointSignedDistance(pl, p) > 0.0f);
}

u32 PlanePointBehindCheck(const struct plane *pl, const vec3 p)
{
	return (PlanePointSignedDistance(pl, p) < 0.0f);
}

u32 PlaneSegmentParallelCheck(const struct plane *pl, const struct segment *s)
{
    const f32 d1d1 = Vec3Dot(pl->normal_direction, pl->normal_direction);
    const f32 d2d2 = Vec3Dot(s->dir, s->dir);
	const f32 d1d2 = Vec3Dot(pl->normal_direction, s->dir);
	const f32 denom = d1d1*d2d2 - d1d2*d1d2;
	/* 
     * denom = |n|^2 * |s->dir|^2 * (1-cos(theta)^2) == 1.0f 
     *  <=> segment is orthogonal to normal
     *  <=> segment is parallel to face
     */
    //TODO what is a reasonable limit here?
	return (denom >= (1.0f - 100.0f*F32_EPSILON) * d1d1 * d2d2);
}

f32 PlaneSegmentClipParameter(const struct plane *pl, const struct segment *s)
{
	/*
	 * 	s.p0 + t*s.dir = PLANE POINT
	 * =>   DOT(s.p0 + t*s.dir - n*pl.signed_distance/DOT(n,n), pl.n) = 0
	 * =>   DOT(t*s.dir n) = DOT(n*pl.signed_distance/DOT(n,n) - s.p0, n)
	 * =>   t = [pl.signed_distance - DOT(s.p0, n)] / DOT(s.dir, n)
	 *
	 * degenerate case: segment parallel to plane gives t = +-infinity, which is okay!
	 */
    const f32 dot_pn = Vec3Dot(pl->normal_direction, s->p[0]);
    const f32 dot_dn = Vec3Dot(pl->normal_direction, s->dir);
	return (pl->signed_distance - dot_pn) / dot_dn;
}

u32 PlaneSegmentClip(vec3 clip, const struct plane *pl, const struct segment *s)
{
	const f32 t = PlaneSegmentClipParameter(pl, s);
	SegmentBc(clip, s, t);
	return (0.0f <= t && t <= 1.0f);
}

u32 PlaneSegmentTest(const struct plane *pl, const struct segment *s)
{
	const f32 t = PlaneSegmentClipParameter(pl, s);
	return (0.0f <= t && t <= 1.0f);
}

f32 PlanePointSignedDistance(const struct plane *pl, const vec3 p)
{
	return Vec3Dot(pl->normal_direction, p) - pl->signed_distance;
}

f32 PlanePointDistance(const struct plane *pl, const vec3 p)
{
	return f32_abs(PlanePointSignedDistance(pl, p));
}

f32 PlanePointProjection(vec3 proj, const struct plane *pl, const vec3 p)
{
	const f32 n_units = PlanePointSignedDistance(pl, p) * pl->inv_dot_nn;
    Vec3Copy(proj, p);
    Vec3TranslateScaled(proj, pl->normal_direction, -n_units);
    return n_units;
}

f32 PlaneRaycastParameter(const struct plane *plane, const struct ray *ray)
{
	const f32 dot = Vec3Dot(ray->dir, plane->normal);
	if (dot == 0.0f) { return F32_INFINITY; }

	return (plane->signed_distance - Vec3Dot(ray->origin, plane->normal)) / dot;
}

u32 PlaneRaycast(vec3 intersection, const struct plane *plane, const struct ray *ray)
{
	const f32 t = PlaneRaycastParameter(plane, ray);
	if (t < 0.0f || t == F32_INFINITY) { return 0; }

	Vec3Copy(intersection, ray->origin);
	Vec3TranslateScaled(intersection, ray->dir, t);
	return 1;
}

void AabbVertex(struct aabb *dst, constvec3ptr v, const u32 v_count, const f32 margin)
{
	vec3 min = { F32_INFINITY, F32_INFINITY, F32_INFINITY };
	vec3 max = { -F32_INFINITY, -F32_INFINITY, -F32_INFINITY };
	for (u32 i = 0; i < v_count; ++i)
	{
        Vec3MinSelf(min, v[i]);
        Vec3MaxSelf(max, v[i]);
	}

	Vec3Sub(dst->hw, max, min);
	Vec3ScaleSelf(dst->hw, 0.5f);
	Vec3Add(dst->center, min, dst->hw);

	dst->hw[0] += margin;
	dst->hw[1] += margin;
	dst->hw[2] += margin;
}

void AabbUnion(struct aabb *box_union, const struct aabb *a, const struct aabb *b)
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

void AabbRotate(struct aabb *dst, const struct aabb *src, mat3 rotation)
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

u32 AabbTest(const struct aabb *a, const struct aabb *b)
{
	if (b->center[0] - b->hw[0] - (a->center[0] + a->hw[0]) > 0.0f 
			|| a->center[0] - a->hw[0] - (b->center[0] + b->hw[0]) > 0.0f) { return 0; }
	if (b->center[1] - b->hw[1] - (a->center[1] + a->hw[1]) > 0.0f 
			|| a->center[1] - a->hw[1] - (b->center[1] + b->hw[1]) > 0.0f) { return 0; }
	if (b->center[2] - b->hw[2] - (a->center[2] + a->hw[2]) > 0.0f 
			|| a->center[2] - a->hw[2] - (b->center[2] + b->hw[2]) > 0.0f) { return 0; }

	return 1;
}

u32 AabbContains(const struct aabb *a, const struct aabb *b)
{
	if (b->center[0] - b->hw[0] < a->center[0] - a->hw[0]) { return 0; }
	if (b->center[1] - b->hw[1] < a->center[1] - a->hw[1]) { return 0; }
	if (b->center[2] - b->hw[2] < a->center[2] - a->hw[2]) { return 0; }
	
	if (b->center[0] + b->hw[0] > a->center[0] + a->hw[0]) { return 0; }
	if (b->center[1] + b->hw[1] > a->center[1] + a->hw[1]) { return 0; }
	if (b->center[2] + b->hw[2] > a->center[2] + a->hw[2]) { return 0; }

	return 1;
}

u32 AabbContainsMargin(const struct aabb *a, const struct aabb *b, const f32 margin)
{
	if (b->center[0] - b->hw[0] < a->center[0] - a->hw[0] - margin) { return 0; }
	if (b->center[1] - b->hw[1] < a->center[1] - a->hw[1] - margin) { return 0; }
	if (b->center[2] - b->hw[2] < a->center[2] - a->hw[2] - margin) { return 0; }
	
	if (b->center[0] + b->hw[0] > a->center[0] + a->hw[0] + margin) { return 0; }
	if (b->center[1] + b->hw[1] > a->center[1] + a->hw[1] + margin) { return 0; }
	if (b->center[2] + b->hw[2] > a->center[2] + a->hw[2] + margin) { return 0; }

	return 1;
}
	
void AabbRaycastParameterExSetup(vec3 multiplier, vec3u32 dir_sign_bit, const struct ray *ray)
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
f32 AabbRaycastParameterEx(const struct aabb *aabb, const struct ray *ray, const vec3 multiplier, const vec3u32 dir_sign_bit)
{
	vec3 box_min, box_max;
	Vec3Sub(box_min, aabb->center, aabb->hw);
	Vec3Add(box_max, aabb->center, aabb->hw);

	f32 t_min = 0.0f;
	f32 t_max = F32_INFINITY;

	for (u32 axis = 0; axis < 3; ++axis)
	{
		/* If parallel to slab, point_slab test */
        //TODO fix hardcoded values here
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

f32 AabbRaycastParameter(const struct aabb *aabb, const struct ray *ray)
{
	vec3 multiplier;
       	vec3u32	dir_sign_bit;
	AabbRaycastParameterExSetup(multiplier, dir_sign_bit, ray);
	return AabbRaycastParameterEx(aabb, ray, multiplier, dir_sign_bit);
}

u32 AabbRaycastEx(vec3 intersection, const struct aabb *aabb, const struct ray *ray, const vec3 multiplier, const vec3u32 dir_sign_bit)
{
	const f32 t = AabbRaycastParameterEx(aabb, ray, multiplier, dir_sign_bit);
	if (t == F32_INFINITY) { return 0; }

	Vec3Copy(intersection, ray->origin);
	Vec3TranslateScaled(intersection, ray->dir, t);
	return 1;
}

u32 AabbRaycast(vec3 intersection, const struct aabb *aabb, const struct ray *ray)
{
	vec3 multiplier; 
	vec3u32 dir_sign_bit;
	AabbRaycastParameterExSetup(multiplier, dir_sign_bit, ray);
	return AabbRaycastEx(intersection, aabb, ray, multiplier, dir_sign_bit);
}

u64 AabbPushLinesBuffered(u8 *buf, const u64 bufsize, const struct aabb *box, const vec4 color)
{
    mat3 identity;
    Mat3Identity(identity);
    const vec3 translation = VEC3_ZERO;
	return AabbTransformPushLinesBuffered(buf, bufsize, box, translation, identity, color);
}

u64 AabbTransformPushLinesBuffered(u8 *buf, const u64 bufsize, const struct aabb *box, const vec3 translation, mat3 rotation, const vec4 color)
{
	const u64 bytes_written = 3*8*(sizeof(vec3)+sizeof(vec4));
	if (bufsize < bytes_written)
	{
		return 0;
	}

	vec3 end;
	Vec3Sub(end, box->center, box->hw);

	f32 *v = (f32*) buf;
	Vec3Set(v+7*0, end[0], 		                end[1], 		            end[2]);
	Vec3Set(v+7*1, end[0] + 2.0f*box->hw[0],    end[1], 		            end[2]);
	Vec3Set(v+7*2, end[0], 		                end[1], 		            end[2]);
	Vec3Set(v+7*3, end[0], 		                end[1] + 2.0f*box->hw[1],   end[2]);
	Vec3Set(v+7*4, end[0], 		                end[1], 		            end[2]);
	Vec3Set(v+7*5, end[0], 		                end[1], 	                end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*6, end[0] + 2.0f*box->hw[0],    end[1], 		            end[2]);
	Vec3Set(v+7*7, end[0] + 2.0f*box->hw[0],    end[1] + 2.0f*box->hw[1],   end[2]);
	Vec3Set(v+7*8, end[0] + 2.0f*box->hw[0],    end[1], 		            end[2]);
	Vec3Set(v+7*9, end[0] + 2.0f*box->hw[0],    end[1],                     end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*10, end[0], 		            end[1] + 2.0f*box->hw[1],   end[2]);
	Vec3Set(v+7*11, end[0], 		            end[1] + 2.0f*box->hw[1],   end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*12, end[0], 		            end[1] + 2.0f*box->hw[1],   end[2]);
	Vec3Set(v+7*13, end[0] + 2.0f*box->hw[0],   end[1] + 2.0f*box->hw[1],   end[2]);

	Vec3Set(v+7*14, end[0], 		            end[1], 	                end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*15, end[0], 		            end[1] + 2.0f*box->hw[1],   end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*16, end[0], 		            end[1], 	                end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*17, end[0] + 2.0f*box->hw[0],   end[1], 	                end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*18, end[0] + 2.0f*box->hw[0],   end[1] + 2.0f*box->hw[1],   end[2]);
	Vec3Set(v+7*19, end[0] + 2.0f*box->hw[0],   end[1] + 2.0f*box->hw[1],   end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*20, end[0], 		            end[1] + 2.0f*box->hw[1],   end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*21, end[0] + 2.0f*box->hw[0],   end[1] + 2.0f*box->hw[1],   end[2] + 2.0f*box->hw[2]);

	Vec3Set(v+7*22, end[0] + 2.0f*box->hw[0],   end[1], 	                end[2] + 2.0f*box->hw[2]);
	Vec3Set(v+7*23, end[0] + 2.0f*box->hw[0],   end[1] + 2.0f*box->hw[1],   end[2] + 2.0f*box->hw[2]);

	for (u32 i = 0; i < 24; ++i)
	{
		vec3 tmp1, tmp2;
		Mat3VecMul(tmp1, rotation, v + 7*i);
		Vec3Add(v + 7*i, tmp1, translation);
		Vec4Copy(v + 7*i + 3, color);
	}

	return bytes_written;
}

struct aabb BboxTriangle(const vec3 p0, const vec3 p1, const vec3 p2)
{
	struct aabb bbox;

	vec3 min = { p0[0], p0[1], p0[2] };
	vec3 max = { p0[0], p0[1], p0[2] };
    Vec3MinSelf(min, p1);
    Vec3MinSelf(min, p2);
    Vec3MaxSelf(max, p1);
    Vec3MaxSelf(max, p2);

	Vec3Sub(bbox.hw, max, min);
	Vec3ScaleSelf(bbox.hw, 0.5f);
	Vec3Add(bbox.center, min, bbox.hw);

	return bbox;
}

struct aabb BboxUnion(const struct aabb a, const struct aabb b)
{
	struct aabb bbox;
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

u32 VertexSupport(vec3 support, const vec3 dir, constvec3ptr v, const u32 v_count)
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

    ds_Assert(best != U32_MAX);
	Vec3Copy(support, v[best]);
	return best;
}

void VertexCentroid(vec3 centroid, constvec3ptr vs, const u32 n)
{
	Vec3Set(centroid, 0.0f, 0.0f, 0.0f);
	for (u32 i = 0; i < n; ++i)
	{
		Vec3Translate(centroid, vs[i]);
	}
	Vec3ScaleSelf(centroid, 1.0f / n);
}

void TriCcwNormal(vec3 normal, const vec3 p0, const vec3 p1, const vec3 p2)
{
	vec3 A, B, C;
	Vec3Sub(A, p1, p0);
	Vec3Sub(B, p2, p0);
	Vec3Cross(C, A, B);
	Vec3Normalize(normal, C);
}

void TriCcwNormalDirection(vec3 dir, const vec3 p0, const vec3 p1, const vec3 p2)
{
	vec3 A, B;
	Vec3Sub(A, p1, p0);
	Vec3Sub(B, p2, p0);
	Vec3Cross(dir, A, B);
}

static void TriVoronoiStaticAssert(void)
{
    ds_StaticAssert(TRI_VORONOI_VERTEX0 == 0, "");
    ds_StaticAssert(TRI_VORONOI_VERTEX1 == 1, "");
    ds_StaticAssert(TRI_VORONOI_VERTEX2 == 2, "");
    ds_StaticAssert(TRI_VORONOI_EDGE01 == 3, "");
    ds_StaticAssert(TRI_VORONOI_EDGE12 == 4, "");
    ds_StaticAssert(TRI_VORONOI_EDGE20 == 5, "");
    ds_StaticAssert(TRI_VORONOI_FACE == 6, "");
}

const char *g_table_tri_voronoi_region_string[TRI_VORONOI_COUNT] =
{
    "TRI_VORONOI_VERTEX0",
    "TRI_VORONOI_VERTEX1",
    "TRI_VORONOI_VERTEX2",
    "TRI_VORONOI_EDGE01",
    "TRI_VORONOI_EDGE12",
    "TRI_VORONOI_EDGE20",
    "TRI_VORONOI_FACE",
};

u32 TriVoronoiInitCcw(struct TriVoronoi *tv, const vec3 t[3])
{
    vec3 face_normal_dir, edge_normal_dir[3];

    Vec3Copy(tv->t[0], t[0]);
    Vec3Copy(tv->t[1], t[1]);
    Vec3Copy(tv->t[2], t[2]);

    tv->s[0] = SegmentConstruct(t[0], t[1]);
    tv->s[1] = SegmentConstruct(t[1], t[2]);
    tv->s[2] = SegmentConstruct(t[2], t[0]);

    Vec3Cross(face_normal_dir, tv->s[0].dir, tv->s[2].dir);
    Vec3ScaleSelf(face_normal_dir, -1.0f);
    tv->face_plane = PlaneConstruct(face_normal_dir, t[0]);

    Vec3Cross(edge_normal_dir[0], tv->s[0].dir, tv->face_plane.normal_direction);
    Vec3Cross(edge_normal_dir[1], tv->s[1].dir, tv->face_plane.normal_direction);
    Vec3Cross(edge_normal_dir[2], tv->s[2].dir, tv->face_plane.normal_direction);

    tv->edge_plane[0] = PlaneConstruct(edge_normal_dir[0], t[0]);
    tv->edge_plane[1] = PlaneConstruct(edge_normal_dir[1], t[1]);
    tv->edge_plane[2] = PlaneConstruct(edge_normal_dir[2], t[2]);

    const f32 n_dir_len_sq = Vec3Dot(tv->face_plane.normal_direction, tv->face_plane.normal_direction);
    const f32 s0_len_sq = Vec3Dot(tv->s[0].dir, tv->s[0].dir);
    const f32 s1_len_sq = Vec3Dot(tv->s[1].dir, tv->s[1].dir);
    const f32 s2_len_sq = Vec3Dot(tv->s[2].dir, tv->s[2].dir);
    const f32 s_max_len_sq = f32_max(f32_max(s0_len_sq, s1_len_sq), s2_len_sq);

    /* 
     * We want |n| to be bound from below proportionally to the size of the triangle's
     * sides.
     *
     *                                     |n|^2  >= EPSILON^2 * |s_max|^2
     *
     *  This test simply enforces a lower limit of |n|, but how does it affect the
     *  minimum angle within the triangle? From the cross product n = Cross(si,sj)
     *  we have for all i,j:
     *
     *                  |si| * |sj| sin(theta_ij) = |n|
     *                              sin(theta_ij) = |n| / (|si| * |sj|)
     *                                           >= |n| / |s_max|^2
     *
     *  In particular, for i,j such that theta_ij = theta_min:
     *
     *                            sin(theta_min) >= |n| / |s_max|^2
     *
     *  Assuming a fixed EPSILON, and the use of our test, we get:
     *
     *                                     |n|^2 >= EPSILON^2 * |s_max|^2
     *                                     |n|   >= EPSILON * |s_max|
     *       =>                   sin(theta_min) >= EPSILON / |s_max|
     *
     *  Note that this yields an ever decreasing minimum angle as the sides of our triangles grow:
     *
     *                      MinAngle(EPSILON) = ArcSin( EPSILON / |s_max| )
     *
     *  This may be a problem for other scenarios, but hopefully our requirement of |n| being 
     *  lower-bound by EPSILON*|s_max| is good enough for Voronoi region calculations.
     *
     *  Table for EPSILON = 10^-6:
     *
     *  |s_max|     |   Approx. minimum angle
     * -------------+---------------------------
     *  0.0100      |   0.00573
     *  0.1000      |   0.000573 
     *  1.0000      |   0.0000573
     *  10.000      |   0.00000573
     *  100.00      |   0.000000573
     */
    const u32 robust = (n_dir_len_sq >= s_max_len_sq * 1e-6 * 1e-6);
    return robust;
}

static const enum TriVoronoiRegion table_tri_voronoi[TRI_VORONOI_COUNT + 1] =
{
    TRI_VORONOI_FACE,
    TRI_VORONOI_EDGE01,
    TRI_VORONOI_EDGE12,
    TRI_VORONOI_VERTEX1,
    TRI_VORONOI_EDGE20,
    TRI_VORONOI_VERTEX0,
    TRI_VORONOI_VERTEX2,
    TRI_VORONOI_VERTEX0 /* TRI_VORONOI_COUNT is invalid, so we just map it to something */
};

static const u32 table_tri_voronoi_edge_check[TRI_VORONOI_COUNT + 1] = { 0, 0, 0, 1, 1, 1, 0, 0 };
static const u32 table_tri_voronoi_vertex_check[TRI_VORONOI_COUNT + 1] = { 1, 1, 1, 0, 0, 0, 0, 0 };

static const u32 table_add_1_mod_3[6] = { 1, 2, 0, 1, 2, 0 };
static const u32 table_sub_1_mod_3[6] = { 2, 0, 1, 2, 0, 1 };

f32 TriCcwPointDistanceSquared(vec3 c, enum TriVoronoiRegion *region, const vec3 point, const struct TriVoronoi *tv)
{
    const u32 index = ((PlanePointInfrontCheck(&tv->edge_plane[0], point)) << 0) 
                    | ((PlanePointInfrontCheck(&tv->edge_plane[1], point)) << 1)
                    | ((PlanePointInfrontCheck(&tv->edge_plane[2], point)) << 2);

    ds_Assert(index != TRI_VORONOI_COUNT);
    *region = table_tri_voronoi[index];

    if (*region == TRI_VORONOI_FACE)
    {
        PlanePointProjection(c, &tv->face_plane, point);
    }
    else if (table_tri_voronoi_edge_check[*region])
    {
        const enum TriVoronoiRegion v = *region - TRI_VORONOI_EDGE01;
        const u32 next = table_add_1_mod_3[v];
        const f32 param = SegmentPointClosestBcParameter(tv->s + v, point); 
        SegmentBc(c, tv->s, param);
        if (1.0f == param)
        {
            *region = v;
        }
        else if (0.0f == param)
        {
            *region = next;
        }
    }
    else if (table_tri_voronoi_vertex_check[*region])
    {
        const u32 ij = table_sub_1_mod_3[*region];
        const u32 jk = *region;
        f32 param; 
        if (0.0f < (param = SegmentPointClosestBcParameter(tv->s + ij, point)) && param < 1.0f)
        {
            SegmentBc(c, tv->s + ij, param);
            *region = TRI_VORONOI_EDGE01 + ij;
        }
        else if (0.0f < (param = SegmentPointClosestBcParameter(tv->s + jk, point)) && param < 1.0f)
        {
            SegmentBc(c, tv->s + jk, param);
            *region = TRI_VORONOI_EDGE01 + jk;
        }
        else
        {
            Vec3Copy(c, tv->t[*region]);
        }
    }

    return Vec3DistanceSquared(point, c);
}

f32 TriCcwSegmentClipParameter(vec3 clip, const struct segment *s, const struct TriVoronoi *tv)
{
    const f32 param = PlaneSegmentClipParameter(&tv->face_plane, s);
    if (0.0f <= param && param <= 1.0f)
    {
        SegmentBc(clip, s, param);
        if (PlanePointBehindCheck(tv->edge_plane + 0, clip) && 
            PlanePointBehindCheck(tv->edge_plane + 1, clip) && 
            PlanePointBehindCheck(tv->edge_plane + 2, clip))
        {
            return param;
        }
    }

    return F32_INFINITY;
}

u32 TriCcwSegmentClip(vec3 clip, const struct segment *s, const struct TriVoronoi *tv)
{
    return (TriCcwSegmentClipParameter(clip, s, tv) < F32_INFINITY);
}

struct segment TriCcwSegmentSideClip(const struct segment *s, const struct TriVoronoi *tv)
{
	f32 min_p = 0.0f;
	f32 max_p = 1.0f;

	for (u32 i = 0; i < 3; ++i)
	{
		const f32 bc_c = PlaneSegmentClipParameter(tv->edge_plane + i, s);
        if (min_p <= bc_c && bc_c <= max_p)
		{
			if (Vec3Dot(s->dir, tv->edge_plane[i].normal_direction) >= 0.0f)
			{
				max_p = bc_c;
			}
			else
			{
				min_p = bc_c;
			}
		}
    }

    vec3 p0, p1;
	SegmentBc(p0, s, min_p);
	SegmentBc(p1, s, max_p);
	return SegmentConstruct(p0, p1);
}

static f32 TriCcwSegmentEdgeCheck(vec3 c_t, vec3 c_s, enum TriVoronoiRegion *region, const struct segment *s, const struct TriVoronoi *tv, const u32 start)
{
    /* Last case: segment-edge generates closest point */
    f32 s_param, t_param;
    SegmentClosestParameter(&s_param, &t_param, s, tv->s + start);
    SegmentBc(c_s, s, s_param);
    SegmentBc(c_t, tv->s + start, t_param);
    
    if (t_param == 0.0f)
    {
        *region = TRI_VORONOI_VERTEX0 + start;
    }
    else if (t_param == 1.0f)
    {
        *region = TRI_VORONOI_VERTEX0 + table_add_1_mod_3[start];
    }
    else
    {
        *region = TRI_VORONOI_EDGE01 + start;
    }
    
    return Vec3DistanceSquared(c_s, c_t);
}

static f32 TriCcwSegmentDoubleEdgeCheck(vec3 c_t, vec3 c_s, enum TriVoronoiRegion *segment_region, const struct segment *s, const struct TriVoronoi *tv, const u32 j)
{
    const u32 i = table_sub_1_mod_3[j];
    const u32 k = table_sub_1_mod_3[j];

    f32 dist_sq, s_param_ij, s_param_jk, t_param_ij, t_param_jk;
    SegmentClosestParameter(&s_param_ij, &t_param_ij, s, tv->s + i);
    SegmentClosestParameter(&s_param_jk, &t_param_jk, s, tv->s + j);

    vec3 c_s_ij, c_s_jk, c_t_ij, c_t_jk;
    SegmentBc(c_s_ij, s, s_param_ij);
    SegmentBc(c_s_jk, s, s_param_jk);
    SegmentBc(c_t_ij, tv->s + i, t_param_ij);
    SegmentBc(c_t_jk, tv->s + j, t_param_jk);

    const f32 dist_sq_ij = Vec3DistanceSquared(c_s_ij, c_t_ij);
    const f32 dist_sq_jk = Vec3DistanceSquared(c_s_jk, c_t_jk);
    if (dist_sq_ij < dist_sq_jk)
    {
        Vec3Copy(c_s, c_s_ij);
        Vec3Copy(c_t, c_t_ij);
        dist_sq = dist_sq_ij;
        if (t_param_ij == 0.0f)
        {
            *segment_region = TRI_VORONOI_VERTEX0 + i;
        }
        else if (t_param_ij == 1.0f)
        {
            *segment_region = TRI_VORONOI_VERTEX0 + j;
        }
        else
        {
            *segment_region = TRI_VORONOI_EDGE01 + i;
        }
    }
    else
    {
        Vec3Copy(c_s, c_s_jk);
        Vec3Copy(c_t, c_t_jk);
        dist_sq = dist_sq_jk;
        if (t_param_jk == 0.0f)
        {
            *segment_region = TRI_VORONOI_VERTEX0 + j;
        }
        else if (t_param_jk == 1.0f)
        {
            *segment_region = TRI_VORONOI_VERTEX0 + k;
        }
        else
        {
            *segment_region = TRI_VORONOI_EDGE01 + j;
        }
    }

    return dist_sq;
}

static f32 TriCcwSegmentTripleEdgeCheck(vec3 c_t, vec3 c_s, enum TriVoronoiRegion *segment_region, const struct segment *s, const struct TriVoronoi *tv)
{
    f32 dist_sq = F32_INFINITY;
    u32 min_i = 0;
    f32 s_param[3], t_param[3];
    vec3 c_s_local[3], c_t_local[3];
    for (u32 i = 0; i < 3; ++i)
    {
        SegmentClosestParameter(s_param + i, t_param + i, s, tv->s + i);
        SegmentBc(c_s_local[i], s, s_param[i]);
        SegmentBc(c_t_local[i], tv->s + i, t_param[i]);
        const f32 dist_sq_local = Vec3DistanceSquared(c_s_local[i], c_t_local[i]);
        if (dist_sq_local < dist_sq)
        {
            dist_sq = dist_sq_local;
            min_i = i;
        }
    }

    Vec3Copy(c_s, c_s_local[min_i]);
    Vec3Copy(c_t, c_t_local[min_i]);
    if (t_param[min_i] == 0.0f)
    {
        *segment_region = TRI_VORONOI_VERTEX0 + min_i;
    }
    else if (t_param[min_i] == 1.0f)
    {
        *segment_region = TRI_VORONOI_VERTEX0 + table_add_1_mod_3[min_i];
    }
    else
    {
        *segment_region = TRI_VORONOI_EDGE01 + min_i;
    }

    return dist_sq;
}

f32 TriCcwSegmentDistanceSquared(vec3 c_t, vec3 c_s, enum TriVoronoiRegion *segment_region, const struct segment *s, const struct TriVoronoi *tv)
{
    f32 dist_sq = F32_INFINITY;

    u32 index[2];
    index[0] = (PlanePointInfrontCheck(tv->edge_plane + 0, s->p[0]) << 0) 
             | (PlanePointInfrontCheck(tv->edge_plane + 1, s->p[0]) << 1)
             | (PlanePointInfrontCheck(tv->edge_plane + 2, s->p[0]) << 2);

    index[1] = (PlanePointInfrontCheck(tv->edge_plane + 0, s->p[1]) << 0) 
             | (PlanePointInfrontCheck(tv->edge_plane + 1, s->p[1]) << 1)
             | (PlanePointInfrontCheck(tv->edge_plane + 2, s->p[1]) << 2);

    ds_Assert(index[0] != TRI_VORONOI_COUNT);
    ds_Assert(index[1] != TRI_VORONOI_COUNT);

    const enum TriVoronoiRegion region[2] = { table_tri_voronoi[index[0]], table_tri_voronoi[index[1]] };

    const u32 high = (region[0] <= region[1])
        ? 1
        : 0;
    const u32 low = 1 - high;
    const struct segment s_canon = SegmentConstruct(s->p[high], s->p[low]);

    if (region[low] == TRI_VORONOI_FACE)
    {
        const f32 param = f32_clamp(PlaneSegmentClipParameter(&tv->face_plane, &s_canon), 0.0f, 1.0f);

        *segment_region = TRI_VORONOI_FACE;
        SegmentBc(c_s, &s_canon, param);
        PlanePointProjection(c_t, &tv->face_plane, c_s);
        dist_sq = (param == 0.0f || param == 1.0f)
                ? Vec3DistanceSquared(c_t, c_s)
                : 0.0f;
    }
    else if (region[high] == TRI_VORONOI_FACE)
    {
        const u32 infront_face = PlanePointInfrontCheck(&tv->face_plane, s_canon.p[0]);
        const f32 dot_dir = Vec3Dot(tv->face_plane.normal_direction, s_canon.dir);
        /* First case: end-point in FACE is closest */
        if ((infront_face && dot_dir >= 0.0f) || (!infront_face && dot_dir <= 0.0f))
        {
            *segment_region = TRI_VORONOI_FACE;
            Vec3Copy(c_s, s_canon.p[0]);
            PlanePointProjection(c_t, &tv->face_plane, c_s);
            dist_sq = Vec3DistanceSquared(c_t, c_s);
        }
        /* Face-Edge: May yield closest point on s within FACE, EDGE_ij, VERTEX_i or VERTEX_j */
        else if (table_tri_voronoi_edge_check[region[low]])
        {
            const u32 start = region[low] - TRI_VORONOI_EDGE01;
            const u32 end = table_add_1_mod_3[start];
            const struct plane side_pl = (infront_face)
                               ? PlaneConstructFromCcwTriangle(s_canon.p[0], tv->t[start], tv->t[end])
                               : PlaneConstructFromCcwTriangle(s_canon.p[0], tv->t[end], tv->t[start]);
            /* Second case: segment clips triangle, point on FACE is closest */
            if (Vec3Dot(side_pl.normal, s_canon.dir) < 0.0f)
            {
                *segment_region = TRI_VORONOI_FACE;
                PlaneSegmentClip(c_s, &tv->face_plane, s);
                Vec3Copy(c_t, c_s);
                dist_sq = 0.0f;
            }    
            /* Last case: segment-edge generates closest point */
            else
            {
                dist_sq = TriCcwSegmentEdgeCheck(c_t, c_s, segment_region, s, tv, start); 
            }
        }
        /* Face-Vertex: May yield closest point on s within FACE, VERTEX_k, EDGE_jk or EDGE_ki */
        else
        {
            const u32 i  = table_sub_1_mod_3[region[low]];
            const u32 j = region[low];
            const u32 k  = table_add_1_mod_3[region[low]];

            const struct plane pl_ij = (infront_face)
                                       ? PlaneConstructFromCcwTriangle(s_canon.p[0], tv->t[i], tv->t[j])
                                       : PlaneConstructFromCcwTriangle(s_canon.p[0], tv->t[j], tv->t[i]);
            const struct plane pl_jk = (infront_face)                                              
                                       ? PlaneConstructFromCcwTriangle(s_canon.p[0], tv->t[j], tv->t[k])
                                       : PlaneConstructFromCcwTriangle(s_canon.p[0], tv->t[k], tv->t[j]);

            const f32 dot_ij = Vec3Dot(pl_ij.normal, s_canon.dir);
            const f32 dot_jk = Vec3Dot(pl_jk.normal, s_canon.dir);

            /* Segment clips face */
            if (dot_ij < 0.0f && dot_jk < 0.0f)
            {
                *segment_region = TRI_VORONOI_FACE;
                PlaneSegmentClip(c_s, &tv->face_plane, &s_canon);
                Vec3Copy(c_t, c_s);
                dist_sq = 0.0f;
            }
            /* Segment is closest to some point on either edges */
            else if (dot_ij >= 0.0f && dot_jk < 0.0f)
            {
                dist_sq = TriCcwSegmentEdgeCheck(c_t, c_s, segment_region, s, tv, i); 
            }
            else if (dot_ij < 0.0f && dot_jk >= 0.0f)
            {
                dist_sq = TriCcwSegmentEdgeCheck(c_t, c_s, segment_region, s, tv, j); 
            }
            else
            {
                //TODO Can we reduce this to a single check?
                dist_sq = TriCcwSegmentDoubleEdgeCheck(c_t, c_s, segment_region, s, tv, j); 
            }
        }
    }
    else if (table_tri_voronoi_edge_check[region[low]])
    {
        /* Edge_ij-Edge_ij: Both points on positive side of EDGE_ij, may yield EDGE_ij, VERTEX_i, VERTEX_j */
        if (region[high] == region[low])
        {
            const u32 start = region[low] - TRI_VORONOI_EDGE01;
            dist_sq = TriCcwSegmentEdgeCheck(c_t, c_s, segment_region, s, tv, start); 
        }
        /* Edge_ij-Edge_jk (/jk): Point points on positive side of EDGE_ij, and EDGE_jk (/kj), may yield, Face, EDGE_ij, EDGE_jk (/kj), VERTEX_j */
        else
        {
            if (TriCcwSegmentClip(c_s, s, tv))
            {
                    Vec3Copy(c_t, c_s);
                    *segment_region = TRI_VORONOI_FACE;
                    dist_sq = 0.0f;
            }
            else
            {
                static const u32 j_map[4] = { U32_MAX, 1, 0, 2 };
                const u32 j = j_map[region[high] + region[low] - 2*TRI_VORONOI_EDGE01];
                dist_sq = TriCcwSegmentDoubleEdgeCheck(c_t, c_s, segment_region, s, tv, j); 
            }
        }
    }
    /* Edge-Vertex: */
    else if (table_tri_voronoi_edge_check[region[high]])
    {
        const u32 start = region[high] - TRI_VORONOI_EDGE01;
        const u32 end = table_add_1_mod_3[start];
        /* Edge-Vertex shared, we can do a single segment-edge test */
        if (start == region[low] || end == region[low])
        {
            dist_sq = TriCcwSegmentDoubleEdgeCheck(c_t, c_s, segment_region, s, tv, region[low]); 
        }
        /* Edge-Vertex not shared, we can get a face intersection, or any edge. */
        else
        {
            if (TriCcwSegmentClip(c_s, s, tv))
            {
                    Vec3Copy(c_t, c_s);
                    *segment_region = TRI_VORONOI_FACE;
                    dist_sq = 0.0f;
            }
            else
            {
                dist_sq = TriCcwSegmentTripleEdgeCheck(c_t, c_s, segment_region, s, tv); 
            }
        }
    }
    else
    {
        /* Vertex_i-Vertex_i */
        if (region[high] == region[low])
        {
            dist_sq = TriCcwSegmentDoubleEdgeCheck(c_t, c_s, segment_region, s, tv, region[high]); 
        }
        /* Vertex_i-Vertex_j */
        else
        {
            dist_sq = TriCcwSegmentTripleEdgeCheck(c_t, c_s, segment_region, s, tv); 
        }
    }

    return dist_sq;
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

static struct dcelFace box_face[] =
{
	{ .first  =  0, .count = 4 },
	{ .first  =  4, .count = 4 },
	{ .first  =  8, .count = 4 },
	{ .first  = 12, .count = 4 },
	{ .first  = 16, .count = 4 },
	{ .first  = 20, .count = 4 },
};

static struct dcelEdge box_edge[] =
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

struct dcel DcelBoxStub(void)
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

static struct dcelFace tri_face[2] =
{
	{ .first  =  0, .count = 3 },
	{ .first  =  3, .count = 3 },
};
                                                        
static struct dcelEdge tri_edge[6] =                    
{
	{ .origin = 0, .twin = 5,  .face_ccw = 0, },     
	{ .origin = 1, .twin = 4,  .face_ccw = 0, },    
	{ .origin = 2, .twin = 3,  .face_ccw = 0, },

	{ .origin = 0, .twin = 2,  .face_ccw = 1, },
	{ .origin = 2, .twin = 1,  .face_ccw = 1, },
	{ .origin = 1, .twin = 0,  .face_ccw = 1, },
};

struct dcel DcelTriStub(void)
{
	struct dcel tri = 
	{
		.v = NULL,
		.e = tri_edge,
		.f = tri_face,
		.e_count = 6,
		.v_count = 3,
		.f_count = 2,
	};

	return tri;
}

struct dcel DcelBox(struct arena *mem, const vec3 hw)
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

void DcelFaceNormal(vec3 normal, const struct dcel *h, mat3 rot, const u32 fi)
{
    vec3 local;
    DcelFaceNormalLocal(local, h, fi);
    Mat3VecMul(normal, rot, local);
}

void DcelFaceDirection(vec3 normal_direction, const struct dcel *h, mat3 rot, const u32 fi)
{
    vec3 local;
    DcelFaceDirectionLocal(local, h, fi);
    Mat3VecMul(normal_direction, rot, local);
}

void DcelFaceDirectionLocal(vec3 dir, const struct dcel *h, const u32 fi)
{
	vec3 a, b;
	struct dcelEdge *e0 = h->e + h->f[fi].first;
	struct dcelEdge *e1 = h->e + h->f[fi].first + 1;
	struct dcelEdge *e2 = h->e + h->f[fi].first + 2;
    TriCcwNormalDirection(dir, h->v[e0->origin], h->v[e1->origin], h->v[e2->origin]);
}

void DcelFaceNormalLocal(vec3 normal, const struct dcel *h, const u32 fi)
{
	DcelFaceDirectionLocal(normal, h, fi);
	Vec3ScaleSelf(normal, 1.0f/Vec3Length(normal));	
}

struct plane DcelFacePlane(const struct dcel *h, mat3 rot, const vec3 pos, const u32 fi)
{
	vec3 n, p;
	DcelFaceNormalLocal(p, h, fi);
	Mat3VecMul(n, rot, p);
	Mat3VecMul(p, rot, h->v[h->e[h->f[fi].first].origin]);
	Vec3Translate(p, pos);
	return PlaneConstruct(n, p);
}

struct plane DcelFacePlaneLocal(const struct dcel *h, const u32 fi)
{
    const u32 i0  = h->e[h->f[fi].first + 0].origin;
    const u32 i1  = h->e[h->f[fi].first + 1].origin;
    const u32 i2  = h->e[h->f[fi].first + 2].origin;
    return PlaneConstructFromCcwTriangle(h->v[i0], h->v[i1], h->v[i2]);
}

struct segment DcelFaceClipSegment(const struct dcel *h, mat3 rot, const vec3 pos, const u32 fi, const struct segment *s)
{
	vec3 f_n, p_n, p_p0, p_p1;

	DcelFaceNormal(p_n, h, rot, fi);

	f32 min_p = 0.0f;
	f32 max_p = 1.0f;

	struct dcelFace *f = h->f + fi;
	for (u32 i = 0; i < f->count; ++i)
	{
		const u32 e0 = f->first + i;
		const u32 e1 = f->first + ((i + 1) % f->count);
		struct plane clip_plane = DcelFaceClipPlane(h, rot, pos, f_n, e0, e1);

		const f32 bc_c = PlaneSegmentClipParameter(&clip_plane, s);
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

	SegmentBc(p_p0, s, min_p);
	SegmentBc(p_p1, s, max_p);
	return SegmentConstruct(p_p0, p_p1);
}

struct plane DcelFaceClipPlane(const struct dcel *h, mat3 rot, const vec3 pos, const vec3 face_normal, const u32 e0, const u32 e1)
{
	vec3 diff, p0, p1;
	struct dcelEdge *edge0 = h->e + e0; 
	struct dcelEdge *edge1 = h->e + e1; 

	Mat3VecMul(p0, rot, h->v[edge0->origin]);
	Mat3VecMul(p1, rot, h->v[edge1->origin]);
	Vec3Translate(p0, pos);
	Vec3Translate(p1, pos);
	Vec3Sub(diff, p1, p0);
	Vec3Cross(p1, diff, face_normal);
	Vec3ScaleSelf(p1, 1.0f/Vec3Length(p1));

	return PlaneConstruct(p1, p0);
}

u32 DcelFaceProjectedPointTest(const struct dcel *h, mat3 rot, const vec3 pos, const u32 fi, const vec3 p)
{
	vec3 f_n, p_n;

	DcelFaceNormal(f_n, h, rot, fi);

	f32 min_p = 0.0f;
	f32 max_p = 1.0f;

	struct dcelFace *f = h->f + fi;
	for (u32 i = 0; i < f->count; ++i)
	{
		const u32 e0 = f->first + i;
		const u32 e1 = f->first + ((i + 1) % f->count);
		struct plane clip_plane = DcelFaceClipPlane(h, rot, pos, f_n, e0, e1);
		if (Vec3Dot(clip_plane.normal, p) > clip_plane.signed_distance)
		{
			return 0;
		}
	}	

	return 1;
}

void DcelEdgeDirection(vec3 dir, const struct dcel *h, const u32 ei)
{
	struct dcelEdge *e0 = h->e + ei;
	struct dcelFace *f = h->f + e0->face_ccw;
	const u32 next = f->first + ((ei - f->first + 1) % f->count);
	struct dcelEdge *e1 = h->e + next;
	Vec3Sub(dir, h->v[e1->origin], h->v[e0->origin]);
}

void DcelEdgeNormal(vec3 dir, const struct dcel *h, const u32 ei)
{
	DcelEdgeDirection(dir, h, ei);
	Vec3ScaleSelf(dir, 1.0f / Vec3Length(dir));
}

struct segment DcelEdgeSegment(const struct dcel *h, mat3 rot, const vec3 pos, const u32 ei)
{
	vec3 p0, p1;
	const u32 first = h->f[h->e[ei].face_ccw].first;
	const u32 count = h->f[h->e[ei].face_ccw].count;
	const u32 e0 = ei;
	const u32 e1 = first + ((ei - first + 1) % count); 

	Mat3VecMul(p0, rot, h->v[h->e[e0].origin]);
	Mat3VecMul(p1, rot, h->v[h->e[e1].origin]);
	Vec3Translate(p0, pos);
	Vec3Translate(p1, pos);

	return SegmentConstruct(p0, p1);
}

void SphereSupport(vec3 support, const vec3 dir, const struct sphere *sph, const vec3 pos)
{
	Vec3Scale(support, dir, sph->radius / Vec3Length(dir));
	Vec3Translate(support, pos);
}

void CapsuleSupport(vec3 support, const vec3 dir, const struct capsule *cap, mat3 rot, const vec3 pos)
{
	vec3 p1, p2;
    Vec3Scale(p1, rot[1], cap->half_height);
	Vec3Negate(p2, p1);

	Vec3Scale(support, dir, cap->radius / Vec3Length(dir));
	Vec3Translate(support, pos);
	(Vec3Dot(dir, p1) > Vec3Dot(dir, p2))
		? Vec3Translate(support, p1) 
		: Vec3Translate(support, p2);
}

u32 DcelSupport(vec3 support, const vec3 dir, const struct dcel *dcel, mat3 rot, const vec3 pos)
{
	f32 max = -F32_INFINITY;
	u32 max_index = 0;
	vec3 p;
	for (u32 i = 0; i < dcel->v_count; ++i)
	{
		Mat3VecMul(p, rot, dcel->v[i]);
		const f32 dot = Vec3Dot(p, dir);
		if (max < dot)
		{
			max_index = i;
			max = dot; 
		}
	}

	Mat3VecMul(support, rot, dcel->v[max_index]);
	Vec3Translate(support, pos);
	return max_index;
}

struct dcel DcelEmpty(void)
{
	struct dcel dcel = { 0 };

	return dcel;
}

void DcelPrint(const struct dcel *dcel)
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

void DcelAssertTopology(struct dcel *dcel)
{
	struct dcelFace *f;
	struct dcelEdge *e;
	for (u32 i = 0; i < dcel->f_count; ++i)
	{
		f = dcel->f + i;
		e = dcel->e + f->first;
		for (u32 j = 0; j < f->count; ++j)
		{
			ds_Assert(e->face_ccw == i);
 			e = dcel->e + f->first + j + 1;
		}

		if (f->first + f->count < dcel->e_count)
		{
			ds_Assert(e->face_ccw != i);
		}
	}

	for (u32 i = 0; i < dcel->e_count; ++i)
	{
		e = dcel->e + i;
		ds_Assert(i == (dcel->e + e->twin)->twin);
	}
}

struct ddcelFace
{
	POOL_NODE;
	struct ds_DLL	ce_list;
	vec3		    normal;
	u32 		    first;	/* first half edge */
	u32 		    count;	/* edge count */
};

struct ddcelEdge
{
    POOL_NODE;
	u32 		origin;		/* vertex index origin */
	u32 		twin; 		/* twin half edge */
	u32 		next;		/* next ccw edge */
	u32 		prev;		/* prev ccw edge */
	u32 		face_ccw; 	/* face to the left of half edge */
	u32		horizon; /* used in horizon derivation step */
};

struct conflictEdge
{
    struct ds_DLLNode   face_edge;
    struct ds_DLLNode   vertex_edge;
	u32                 vertex;
	u32                 face;
    POOL_NODE;
};

POOL_DECLARE(ddcelFace);
POOL_DECLARE(ddcelEdge);
POOL_DECLARE(conflictEdge);

POOL_DEFINE(ddcelFace);
POOL_DEFINE(ddcelEdge);
POOL_DEFINE(conflictEdge);

struct conflictVertex
{
	struct ds_DLL	ce_list;
	u32		index;
	/* Needed in last step of iteration */
	u32		last_iter;	/* last iteration it was added to a face's conflict list*/
	u32		last_face;	/* last iteration it was added to a face's conflict list*/
};

struct horizionVertex
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
	struct ddcelFacePool 		face_pool;
	struct ddcelEdgePool		edge_pool;
	/* pools are not growable, so safe to use these */
	struct ddcelFace *	f;		
	struct ddcelEdge *	e;
	constvec3ptr		v;
	u32 			v_count;

	/* internal */
	struct arena		tmp1;
	struct conflictEdgePool 	ce_pool;
	struct conflictEdge *	ce;
	struct conflictVertex *cv;
	struct horizionVertex * hv;
};

static void DdcelFaceSet(struct ddcelFace *face, const u32 first, const u32 count)
{
	face->first = first;
	face->count = count;
	ds_DLLFlush(face->ce_list);
}

static void DdcelEdgeSet(struct ddcelEdge *edge, const u32 origin, const u32 twin, const u32 prev, const u32 next, const u32 face_ccw)
{
	edge->origin = origin;
	edge->twin = twin;
	edge->next = next;
	edge->prev = prev;
	edge->face_ccw = face_ccw;
	edge->horizon = 0;
}

static void DdcelAssertTopology(const struct ddcel *ddcel)
{
	struct arena *tmp = ArenaPushScratch();

	u32 face_count = 0;
	u32 vertex_count = 0;

	u32 *vertex_check = ArenaPushZero(tmp, ddcel->v_count * sizeof(u32));
	u32 *edge_check = ArenaPushZero(tmp, ddcel->edge_pool.count * sizeof(u32));
	u32 *face_check = ArenaPushZero(tmp, 3*ddcel->v_count * sizeof(u32));

	for (u32 i = 0; i < ddcel->edge_pool.count; ++i)
	{
		if (ds_PoolSlotAllocated(ddcel->e + i) && !edge_check[i])
		{
			face_count += 1;
			u32 next;
		        u32 prev; 
			u32 current = i;
			u32 edge_count = 0;
			do
			{
				edge_count += 1;
				const struct ddcelEdge *c = ddcel->e + current;
				const struct ddcelEdge *p = ddcel->e + c->prev;
				const struct ddcelEdge *n = ddcel->e + c->next;
				const struct ddcelEdge *t = ddcel->e + c->twin;

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
		if (ds_PoolSlotAllocated(ddcel->f + i))
		{
			vec3 diff;
			Vec3Sub(diff, center, ddcel->v[ddcel->e[ddcel->f[i].first].origin]);
			ds_Assert(Vec3Dot(diff, ddcel->f[i].normal) < 0.0f);
		}
	}

    ArenaPopScratch();
	ds_Assert(face_count >= 4);
}

u32 InternalConvexHullTetrahedronIndices(struct ddcel *ddcel, const f32 tol)
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
		struct conflictVertex tmp = ddcel->cv[j];
		ddcel->cv[j] = ddcel->cv[indices[j]];
		ddcel->cv[indices[j]] = tmp;
	}

	return i < ddcel->v_count;
}

static void InternalConvexHullTetrahedronDdcel(struct ddcel *ddcel, const f32 tol)
{
	constvec3ptr v = ddcel->v;
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

	struct ddcelFace *f0 =  ddcelFacePoolAdd(&ddcel->face_pool).address;
	struct ddcelFace *f1 =  ddcelFacePoolAdd(&ddcel->face_pool).address;
	struct ddcelFace *f2 =  ddcelFacePoolAdd(&ddcel->face_pool).address;
	struct ddcelFace *f3 =  ddcelFacePoolAdd(&ddcel->face_pool).address;

	struct ddcelEdge *e0 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e1 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e2 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e3 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e4 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e5 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e6 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e7 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e8 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e9 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e10 = ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e11 = ddcelEdgePoolAdd(&ddcel->edge_pool).address;

	DdcelFaceSet(f0, 0, 3);
	DdcelFaceSet(f1, 3, 3);
	DdcelFaceSet(f2, 6, 3);
	DdcelFaceSet(f3, 9, 3);

	DdcelEdgeSet(e0,  ddcel->cv[0].index,  4,  2,  1, 0);
	DdcelEdgeSet(e1,  ddcel->cv[1].index, 10,  0,  2, 0);
	DdcelEdgeSet(e2,  ddcel->cv[2].index,  7,  1,  0, 0);

	DdcelEdgeSet(e3,  ddcel->cv[3].index, 11,  5,  4, 1);
	DdcelEdgeSet(e4,  ddcel->cv[1].index,  0,  3,  5, 1);
	DdcelEdgeSet(e5,  ddcel->cv[0].index,  6,  4,  3, 1);

	DdcelEdgeSet(e6,  ddcel->cv[3].index,  5,  8,  7, 2);
	DdcelEdgeSet(e7,  ddcel->cv[0].index,  2,  6,  8, 2);
	DdcelEdgeSet(e8,  ddcel->cv[2].index,  9,  7,  6, 2);

	DdcelEdgeSet(e9,  ddcel->cv[3].index,  8, 11, 10, 3);
	DdcelEdgeSet(e10, ddcel->cv[2].index,  1,  9, 11, 3);
	DdcelEdgeSet(e11, ddcel->cv[1].index,  3, 10,  9, 3);

    TriCcwNormal(ddcel->f[0].normal, ddcel->v[e0->origin], ddcel->v[e1->origin], ddcel->v[e2->origin]);
    TriCcwNormal(ddcel->f[1].normal, ddcel->v[e3->origin], ddcel->v[e4->origin], ddcel->v[e5->origin]);
    TriCcwNormal(ddcel->f[2].normal, ddcel->v[e6->origin], ddcel->v[e7->origin], ddcel->v[e8->origin]);
    TriCcwNormal(ddcel->f[3].normal, ddcel->v[e9->origin], ddcel->v[e10->origin], ddcel->v[e11->origin]);

	DdcelAssertTopology(ddcel);
}

static void InternalConvexHullTetrahedronConflicts(struct ddcel *ddcel, const f32 tol)
{
	constvec3ptr v = ddcel->v;
	const u32 v_count = ddcel->v_count;
	vec3 b;
	for (u32 cv_i = 4; cv_i < v_count; ++cv_i)
	{
		struct conflictVertex *cv = ddcel->cv + cv_i;
		for (u32 f_i = 0; f_i < 4; ++f_i)
		{
			const u32 v0_i = ddcel->e[ddcel->f[f_i].first].origin;
			Vec3Sub(b, v[cv->index], v[v0_i]);
			/* If point is "in front" of face, we have a conflict */
			if (Vec3Dot(ddcel->f[f_i].normal, b) > tol)
			{
				struct slot slot = conflictEdgePoolAdd(&ddcel->ce_pool);
				ds_DLLAppend(cv->ce_list, ddcel->ce_pool.buf, slot.index, vertex_edge);
				ds_DLLAppend(ddcel->f[f_i].ce_list, ddcel->ce_pool.buf, slot.index, face_edge);

				struct conflictEdge *edge = slot.address;
				edge->vertex = cv_i;
				edge->face = f_i;
			}
		}
	}
}

void ConvexHullIteration(struct ddcel *ddcel, const u32 cvi, const f32 tol)
{
	if (ddcel->cv[cvi].ce_list.count == 0) { return; }

	struct conflictVertex *cv = ddcel->cv + cvi;
	struct conflictEdge *ce = NULL;
	for (i32 i = cv->ce_list.first; i != DLL_SENTINEL; i = ce->vertex_edge.next)
	{
		ce = ddcel->ce + i;
		const u32 fi = ce->face;
		struct ddcelEdge *e = ddcel->e + ddcel->f[fi].first;
		for (u32 j = 0; j < ddcel->f[fi].count; ++j)
		{
			struct ddcelEdge *twin = ddcel->e + e->twin;
			ds_Assert(e->horizon == 0);
			ds_Assert(twin->horizon == 0);
			e = ddcel->e + e->next;
		}
	}
	
	/* (5) Get horizon edges:
	 * At this point, all edges has horizon set to 0. Whenever we visit an edge, 
	 * we flip the edge and its twin's horizon value. After the loop the horizon
	 * will consist of the edges with value 1.  */
	for (i32 i = cv->ce_list.first; i != DLL_SENTINEL; i = ce->vertex_edge.next)
	{
		ce = ddcel->ce + i;
		const u32 fi = ce->face;
		struct ddcelEdge *e = ddcel->e + ddcel->f[fi].first;
		for (u32 j = 0; j < ddcel->f[fi].count; ++j)
		{
			struct ddcelEdge *twin = ddcel->e + e->twin;
			//fprintf(stderr, "horizon edge par (%u,%u)\n", ds_PoolIndex(&ddcel->edge_pool, e), e->twin);
			e->horizon += 1;
			twin->horizon += 1;
			ds_Assert(e->horizon <= 2 && twin->horizon <= 2);
			ds_Assert(twin->twin == ddcelEdgePoolIndex(&ddcel->edge_pool, e));
			e = ddcel->e + e->next;
		}
	}

	/* (6) loop through and remove all faces and non-horizon edges. Sort horizon edges in a 
	 * ad-hoc dll structure horizon vertex and cache additional data.
	 */
	u32 horizon = 0;
	u32 horizon_count = 0;
	for (i32 i = cv->ce_list.first; i != DLL_SENTINEL; i = ce->vertex_edge.next)
	{
		ce = ddcel->ce + i;
		const u32 fi = ce->face;
		struct ddcelFace *f = ddcel->f + fi;

		u32 ej = f->first;
		for (u32 j = 0; j < f->count; ++j)
		{
			struct ddcelEdge *e = ddcel->e + ej;
			const u32 next = e->next;
			if (e->horizon == 2)
			{
				ddcelEdgePoolRemove(&ddcel->edge_pool, ej);	
			}
			else
			{
				//fprintf(stderr, "horizon edge par (%u,%u)\n", ej, e->twin);
				struct ddcelEdge *e_twin = ddcel->e + e->twin;
				struct ddcelFace *f_twin = ddcel->f + e_twin->face_ccw;

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
		struct ddcelFace *f_ccw = ddcel->f + ccw_face;
		struct ddcelFace *f_twin = ddcel->f + twin_face;

		struct slot se1 = ddcelEdgePoolAdd(&ddcel->edge_pool);
		struct slot se2 = ddcelEdgePoolAdd(&ddcel->edge_pool);

		struct ddcelEdge *e1 = se1.address;
		struct ddcelEdge *e2 = se2.address;

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

				ddcelEdgePoolRemove(&ddcel->edge_pool, ddcel->e[edge].twin);
				ddcelEdgePoolRemove(&ddcel->edge_pool, edge);
				edge = ddcel->hv[horizon].edge_out;
			} while (ddcel->hv[horizon].edge_out_twin_face == twin_face);

			DdcelEdgeSet(e1, horizon, U32_MAX, twin_end_prev, se2.index, twin_face);
			DdcelEdgeSet(e2, cv->index, prev_horizon_edge1, se1.index, twin_start_next, twin_face);
			ddcel->e[twin_end_prev].next = se1.index;
			ddcel->e[twin_start_next].prev = se2.index;

			/* Note: Since we reuse the twin face, we also reuse its list of conflicts, so we are done. */
		}
		else
		{
			struct slot sf = ddcelFacePoolAdd(&ddcel->face_pool);
			//fprintf(stderr, "added face %u\n", sf.index);

			const u32 e0i = ddcel->hv[horizon].edge_out;

			struct ddcelFace *f = sf.address;
			struct ddcelEdge *e0 = ddcel->e + e0i;

			DdcelEdgeSet(e2, cv->index, ddcel->hv[prev_horizon].edge1, se1.index, e0i, sf.index);
			DdcelEdgeSet(e1, ddcel->e[e0->twin].origin, U32_MAX, e0i, se2.index, sf.index);
			DdcelEdgeSet(e0, e0->origin, e0->twin, se2.index, se1.index, sf.index);

			DdcelFaceSet(f, e0i, 3);
            TriCcwNormal(f->normal, ddcel->v[e0->origin], ddcel->v[e1->origin], ddcel->v[e2->origin]);
		
			/*TODO: We may add same point twich here, need to add a "has_been_mapped" thingy to not add again*/
			ce = NULL;
			for (i32 j = f_ccw->ce_list.first; j != DLL_SENTINEL; j = ce->face_edge.next)
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
						struct slot slot = conflictEdgePoolAdd(&ddcel->ce_pool);
						ds_DLLAppend(f->ce_list, ddcel->ce_pool.buf, slot.index, face_edge);
						ds_DLLAppend(ddcel->cv[ce->vertex].ce_list, ddcel->ce_pool.buf, slot.index, vertex_edge);

						struct conflictEdge *new = slot.address;
						new->vertex = ce->vertex;
						new->face = sf.index;
					}
				}
			}

			ce = NULL;
			for (i32 j = f_twin->ce_list.first; j != DLL_SENTINEL; j = ce->face_edge.next)
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
						struct slot slot = conflictEdgePoolAdd(&ddcel->ce_pool);
						ds_DLLAppend(f->ce_list, ddcel->ce_pool.buf, slot.index, face_edge);
						ds_DLLAppend(ddcel->cv[ce->vertex].ce_list, ddcel->ce_pool.buf, slot.index, vertex_edge);

						struct conflictEdge *new = slot.address;
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
	while (cv->ce_list.first != DLL_SENTINEL)
	{
		ce = ddcel->ce + cv->ce_list.first;
		const u32 fi = ce->face;
		struct ddcelFace *f = ddcel->f + fi;

        i32 next;
        for (i32 cei = f->ce_list.first; cei != DLL_SENTINEL; cei = next)
        {
			ce = ddcel->ce + cei;
            next = ce->face_edge.next;
			struct conflictVertex *cvj = ddcel->cv + ce->vertex;
			ds_DLLRemove(cvj->ce_list, ddcel->ce_pool.buf, cei, vertex_edge);
			conflictEdgePoolRemove(&ddcel->ce_pool, cei);
        }
		ddcelFacePoolRemove(&ddcel->face_pool, fi);
		//fprintf(stderr, "removed face %u\n", fi);
	}
}

struct dcel DcelDdcel(struct arena *mem, const struct ddcel *ddcel)
{
	ArenaPushRecord(mem);
	struct dcel cpy =
	{
		.v = ArenaPushMemcpy(mem, ddcel->v, ddcel->v_count*sizeof(vec3)),
		.e = ArenaPush(mem, ddcel->edge_pool.count*sizeof(struct dcelEdge)),
		.f = ArenaPush(mem, ddcel->face_pool.count*sizeof(struct dcelFace)),
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
			if (ds_PoolSlotAllocated(ddcel->f + fj))
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
			if (ds_PoolSlotAllocated(ddcel->f + fj))
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

		//DcelPrint(&cpy);
		//DcelAssertTopology(&cpy);
		ArenaPopRecord(mem);
		ArenaRemoveRecord(mem);
	}
	else
	{
		ArenaPopRecord(mem);
		cpy = DcelEmpty();
	}

	return cpy;
}

struct dcel DcelConvexHull(struct arena *mem, constvec3ptr v, const u32 v_count, const f32 tol)
{
	struct dcel dcel = DcelEmpty();
	if (v_count < 4) { goto end; }	

	struct arena *tmp1 = ArenaPushScratch();
	struct arena *tmp2 = ArenaPushScratch();

	const u32 edge_count_upper_bound = 6*v_count - 12;
	const u32 face_count_upper_bound = 2*v_count - 4;
	struct ddcel ddcel =
	{
		.face_pool = ddcelFacePoolAlloc(tmp1, 2*face_count_upper_bound, NOT_GROWABLE),	/* add additional space for easier memory management */
		.edge_pool = ddcelEdgePoolAlloc(tmp1, 2+edge_count_upper_bound, NOT_GROWABLE),	/* add additional space for easier memory management */
		.v = v,
		.v_count = v_count,
		.ce_pool = conflictEdgePoolAlloc(tmp2, (tmp2->mem_size / sizeof(struct conflictEdge))-1, NOT_GROWABLE),
		.cv = ArenaPush(tmp1, v_count * sizeof(struct conflictVertex)),
		.hv = ArenaPush(tmp1, v_count * sizeof(struct horizionVertex)),
	};

	ddcel.tmp1 = *ArenaPushScratch();
	ddcel.e = (struct ddcelEdge *) ddcel.edge_pool.buf;
	ddcel.f = (struct ddcelFace *) ddcel.face_pool.buf;
	ddcel.ce = (struct conflictEdge *) ddcel.ce_pool.buf;

	/* (1) permutation - Random permutation of remaining points */
	for (u32 i = 0; i < v_count; ++i)
	{
		ds_DLLFlush(ddcel.cv[i].ce_list);
		ddcel.cv[i].index = i;
		ddcel.cv[i].last_iter = U32_MAX;
		ddcel.cv[i].last_face = U32_MAX;
	}
	for (u32 i = 0; i < v_count; ++i)
	{
		const u32 rng = (u32) RngU64Range(i, v_count-1);
		const u32 tmp = ddcel.cv[i].index;
		ddcel.cv[i].index = ddcel.cv[rng].index;
		ddcel.cv[rng].index = tmp;
	}

	/* (2) Get inital points for tetrahedron */
	if (InternalConvexHullTetrahedronIndices(&ddcel, tol) == 0) { goto end; }

	/* (3) initiate DCEL from points */
	InternalConvexHullTetrahedronDdcel(&ddcel, tol);

	/* (4) setup conflict graph */
	InternalConvexHullTetrahedronConflicts(&ddcel, tol);

	/* iteratetively solve and add conflicts until no vertices left */
	for (u32 i = 4; i < v_count; ++i)
	{
		ConvexHullIteration(&ddcel, i, tol);
		DdcelAssertTopology(&ddcel);
	}

	dcel = DcelDdcel(mem, &ddcel);	
end:
	ArenaPopScratch();
	ArenaPopScratch();
	ArenaPopScratch();

	return dcel;
}

struct aabb TriMeshBbox(const struct triMesh *mesh)
{
	struct aabb bbox = { 0 };	
	AabbVertex(&bbox, mesh->v, mesh->v_count, 0.0f);	
	return bbox;
}

f32 TriMeshRaycastParameter(const struct triMesh *mesh, const u32 tri, const struct ray *ray)
{
	vec3 intersection;
	f32 t = F32_INFINITY;
	if (TriMeshRaycast(intersection, mesh, tri, ray))
	{
		t = RayPointClosestPointParameter(ray, intersection);
	}
	return t;
}

u32 TriMeshRaycast(vec3 intersection, const struct triMesh *mesh, const u32 tri, const struct ray *ray)
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
