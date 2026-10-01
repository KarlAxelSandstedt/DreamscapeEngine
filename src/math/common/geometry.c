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
#include "ds_float.h"
#include "ds_vector.h"
#include "ds_matrix.h"
#include "ds_hash_map.h"
#include "list.h"
#include "queue.h"
#include "ds_float.h"

//TODO what to do with this?
#define MIN_SEGMENT_LENGTH_SQ	(100.0f*F32_EPSILON)

struct ray RayConstruct(const v3 origin, const v3 dir)
{
	ds_Assert(V3LengthSquared(dir) > 0.0f);

	const struct ray r = { .origin = origin, .dir = dir };
	return r;
}

struct segment RayConstructSegment(const struct ray *r, const f32 t)
{
	const v3 p = V3AddScaled(r->origin, r->dir, t);
	return SegmentConstruct(r->origin, p);
}


v3 RayPoint(const struct ray *ray, const f32 t)
{
	return V3AddScaled(ray->origin, ray->dir, t);
}

struct sphere SphereConstruct(const v3 center, const f32 radius)
{
	const struct sphere sph = { .center = center, .radius = radius };
	return sph;	
}

/*
 * | r.o + t*r.dir - s.c |^2 = s.r^2 => solve quadratic formula give the following method.
 */
f32 SphereRaycastParameter(const struct sphere *sph, const struct ray *ray)
{
	const v3 diff = V3Sub(ray->origin, sph->center);

	const f32 a = V3Dot(ray->dir, ray->dir);
	const f32 b = 2.0f * V3Dot(ray->dir, diff);
	const f32 c = V3Dot(diff, diff) - sph->radius*sph->radius;

	const f32 square = (b*b - 4.0f*a*c);
	if (square < 0.0f) { return F32_INFINITY; }

	const f32 root = F32Sqrt(square);

	const f32 t2 = -b + root;
	if (t2 < 0.0f) { return F32_INFINITY; }

	const f32 t1 = -b - root;
	return (t1 >= 0.0f) ? t1 / (2.0f * a) : t2 / (2.0f * a);
}

u32 SphereRaycast(v3 *intersection, const struct sphere *sph, const struct ray *ray)
{
	const f32 t = SphereRaycastParameter(sph, ray);
	if (t < 0.0f || t == F32_INFINITY) { return 0; }

	*intersection = V3AddScaled(ray->origin, ray->dir, t);
	return 1;
}

f32 RayPointClosestPointParameter(const struct ray *ray, const v3 p)
{ 
	const v3 diff = V3Sub(p, ray->origin);
	const f32 tr = V3Dot(diff, ray->dir) / V3Dot(ray->dir, ray->dir);
	return (tr >= 0.0f) ? tr : 0.0f;
}

f32 RayPointDistanceSquared(v3 *r_c, const struct ray *ray, const v3 p)
{
	const f32 t = RayPointClosestPointParameter(ray, p);
	*r_c = RayPoint(ray, t);
	return V3DistanceSquared(*r_c, p);
}

f32 RaySegmentDistanceSquared(v3 *r_c, v3 *s_c, const struct ray *ray, const struct segment *s)
{
	const v3 diff = V3Sub(s->p[0], ray->origin);
	const f32 drdr = V3Dot(ray->dir, ray->dir);
	const f32 dsds = V3Dot(s->dir, s->dir);

	f32 tr = 0.0f;
	f32 ts = 0.0f;

	if (dsds >= MIN_SEGMENT_LENGTH_SQ)
	{
		const f32 drds = V3Dot(ray->dir, s->dir);
		const f32 diffdr = V3Dot(diff, ray->dir);
		const f32 diffds = V3Dot(diff, s->dir);
		const f32 denom = drdr*dsds - drds*drds;
		/* Check that the ray and segment are not parallel */
		if (denom > 0.0f)
		{
			tr = (diffdr*dsds - diffds*drds) / denom;
			tr = (tr >= 0.0f) ? tr : 0.0f;
		}

		ts = F32Clamp(tr*drds - diffds, 0.0f, dsds);
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
		tr = F32Clamp(V3Dot(diff, ray->dir) / drdr, 0.0f, 1.0f);
	}
	
	ds_Assert(0.0f <= tr);
	ds_Assert(0.0f <= ts && ts <= 1.0f);

	*r_c = RayPoint(ray, tr);
	*s_c = SegmentBc(s, ts);
	return V3DistanceSquared(*r_c, *s_c);
}

struct segment SegmentConstruct(const v3 p0, const v3 p1)
{
	const struct segment s = { .p = { p0, p1 }, .dir = V3Sub(p1, p0) };
	return s;
}

u32 SegmentPointCheck(const struct segment *s, const f32 min_dist_sq)
{
    return (V3Dot(s->dir, s->dir) <= min_dist_sq);
}

u32 SegmentParallelCheck(const struct segment *s1, const struct segment *s2, const f32 eps)
{
    return V3ParallelCheck(s1->dir, s2->dir, eps);
}

void SegmentClosestParameter(f32 *t1, f32 *t2, const struct segment *s1, const struct segment *s2)
{
	const v3 diff = V3Sub(s2->p[0], s1->p[0]);
	const f32 d1d1 = V3LengthSquared(s1->dir);
	const f32 d2d2 = V3LengthSquared(s2->dir);

	*t1 = 0.0f;
	*t2 = 0.0f;

	if (d1d1 >= MIN_SEGMENT_LENGTH_SQ && d2d2 >= MIN_SEGMENT_LENGTH_SQ)
	{
		const f32 d1d2 = V3Dot(s1->dir, s2->dir);
		const f32 diffd1 = V3Dot(diff, s1->dir);
		const f32 diffd2 = V3Dot(diff, s2->dir);
		const f32 denom = d1d1*d2d2 - d1d2*d1d2;
		/* Check that the segments are not parallel */
        //TODO is 0.0f good here, or should we use degree test as in SegmentParallelCheck?
		if (denom > 0.0f)
		{
			*t1 = F32Clamp((diffd1*d2d2 - diffd2*d1d2) / denom, 0.0f, 1.0f);
		}

		/*
		 *  t2 = (L1_P1*(1-t1) + L1_P2*t1 - L2_P1) * DIR2 / DIR2*DIR2
		 *     = (-DIFF + DIR1*t1) * DIR2 / DIR2*DIR2
		 *     = (-DIFF*DIR2 + DIR1*DIR2*t1) / DIR2*DIR2
		 */
		*t2 = F32Clamp(*t1*d1d2 - diffd2, 0.0f, d2d2);

		if (*t2 == 0.0f)
		{
			/*
			 *  t1 = (L2_P1*(1-t2) + L2_P2*t2 - L1_P1) * DIR1 / DIR1*DIR1
			 *     = (DIFF + DIR2*t2) * DIR1 / DIR1*DIR1
			 *     = DIFF*DIR1 / DIR1*DIR1
			 */
            *t1 = F32Clamp(diffd1 / d1d1, 0.0f, 1.0f);
		}
		else if (*t2 == d2d2)
		{
			*t2 = 1.0f;
			*t1 = F32Clamp((diffd1 + d1d2) / d1d1, 0.0f, 1.0f);
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
        *t1 = F32Clamp(V3Dot(diff, s1->dir) / d1d1, 0.0f, 1.0f);
	}
	else if (d2d2 >= MIN_SEGMENT_LENGTH_SQ)
	{
		*t2 = F32Clamp(-V3Dot(diff, s2->dir) / d2d2, 0.0f, 1.0f);
	}

	ds_Assert(0.0f <= *t1 && *t1 <= 1.0f);
	ds_Assert(0.0f <= *t2 && *t2 <= 1.0f);
}

f32 SegmentDistanceSquared(v3 *c1, v3 *c2, const struct segment *s1, const struct segment *s2)
{
    f32 t1, t2;
    SegmentClosestParameter(&t1, &t2, s1, s2);
	*c1 = SegmentBc(s1, t1);
	*c2 = SegmentBc(s2, t2);
	return V3DistanceSquared(*c1, *c2);
}

f32 SegmentPointDistanceSquared(v3 *c, const struct segment *s, const v3 p)
{
	f32 t = 0.0f;
	
	if (V3LengthSquared(s->dir) >= MIN_SEGMENT_LENGTH_SQ)
	{
		const v3 diff = V3Sub(p, s->p[0]);
		t = F32Clamp(V3Dot(diff, s->dir) / V3Dot(s->dir, s->dir), 0.0f, 1.0f);
	}

	*c = SegmentBc(s, t);
	return V3DistanceSquared(*c, p);
}

v3 SegmentBc(const struct segment *s, const f32 t)
{
	return V3Interpolate(s->p[1], s->p[0], t);
}
                                                                        
struct segment SegmentCapsuleTransform(const struct capsule *cap, const ds_Transform *t)
{
    const v3 p = QVec3Rotate(t->rotation, V3(0.0f, cap->half_height, 0.0f));
	return SegmentConstruct(V3Add(p, t->position), V3Add(V3Negate(p), t->position));
}

struct aabb BboxSegment(const struct segment *s)
{
	struct aabb bbox;

	const v3 min = V3Min(s->p[0], s->p[1]);
	const v3 max = V3Max(s->p[0], s->p[1]);

	bbox.hw = V3Scale(V3Sub(max, min), 0.5f);
	bbox.center = V3Add(min, bbox.hw);

    return bbox;
}

f32 SegmentPointProjectedBcParameter(const struct segment *s, const v3 p)
{	
	const v3 diff = V3Sub(p, s->p[0]);
	return V3Dot(diff, s->dir) / V3Dot(s->dir, s->dir);
}

f32 SegmentPointClosestBcParameter(const struct segment *s, const v3 p)
{	
	const v3 diff = V3Sub(p, s->p[0]);
	return F32Clamp(V3Dot(diff, s->dir) / V3Dot(s->dir, s->dir), 0.0f, 1.0f);
}

struct plane PlaneConstruct(const v3 n, const v3 p)
{
	struct plane pl;
	pl.normal_direction = n;
    pl.inv_dot_nn = 1.0f / V3Dot(n,n);
	pl.signed_distance = V3Dot(n, p);
	return pl;
}

struct plane PlaneConstructNormalized(const v3 n, const v3 p)
{
	struct plane pl;
	pl.normal = V3Normalize(n);
    pl.inv_dot_nn = 1.0f;
	pl.signed_distance = V3Dot(pl.normal, p);
	return pl;
}

struct plane PlaneConstructFromCcwTriangle(const v3 a, const v3 b, const v3 c)
{
	return PlaneConstruct(V3Cross(V3Sub(b, a), V3Sub(c, a)), a);
}

struct plane PlaneConstructNormalizedFromCcwTriangle(const v3 a, const v3 b, const v3 c)
{
	const v3 cross = V3Cross(V3Sub(b, a), V3Sub(c, a));
	return PlaneConstruct(V3Scale(cross, 1.0f/V3Length(cross)), a);
}

void PlaneNormalize(struct plane *pl)
{
    const f32 n_dir_len = V3Length(pl->normal_direction);
    pl->normal_direction = V3Scale(pl->normal_direction, 1.0f/n_dir_len);
    pl->signed_distance /= n_dir_len;
    pl->inv_dot_nn = 1.0f;
}

u32 PlanePointInfrontCheck(const struct plane *pl, const v3 p)
{
	return (PlanePointSignedDistance(pl, p) > 0.0f);
}

u32 PlanePointBehindCheck(const struct plane *pl, const v3 p)
{
	return (PlanePointSignedDistance(pl, p) < 0.0f);
}

u32 PlaneSegmentParallelCheck(const struct plane *pl, const struct segment *s)
{
    const f32 d1d1 = V3Dot(pl->normal_direction, pl->normal_direction);
    const f32 d2d2 = V3Dot(s->dir, s->dir);
	const f32 d1d2 = V3Dot(pl->normal_direction, s->dir);
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
    const f32 dot_pn = V3Dot(pl->normal_direction, s->p[0]);
    const f32 dot_dn = V3Dot(pl->normal_direction, s->dir);
	return (pl->signed_distance - dot_pn) / dot_dn;
}

u32 PlaneSegmentClip(v3 *clip, const struct plane *pl, const struct segment *s)
{
	const f32 t = PlaneSegmentClipParameter(pl, s);
	*clip = SegmentBc(s, t);
	return (0.0f <= t && t <= 1.0f);
}

u32 PlaneSegmentTest(const struct plane *pl, const struct segment *s)
{
	const f32 t = PlaneSegmentClipParameter(pl, s);
	return (0.0f <= t && t <= 1.0f);
}

f32 PlanePointSignedDistance(const struct plane *pl, const v3 p)
{
	return V3Dot(pl->normal_direction, p) - pl->signed_distance;
}

f32 PlanePointDistance(const struct plane *pl, const v3 p)
{
	return F32Abs(PlanePointSignedDistance(pl, p));
}

f32 PlanePointProjection(v3 *proj, const struct plane *pl, const v3 p)
{
	const f32 n_units = PlanePointSignedDistance(pl, p) * pl->inv_dot_nn;
    *proj = V3AddScaled(p, pl->normal_direction, -n_units);
    return n_units;
}

f32 PlaneRaycastParameter(const struct plane *plane, const struct ray *ray)
{
	const f32 dot = V3Dot(ray->dir, plane->normal);
	if (dot == 0.0f) { return F32_INFINITY; }

	return (plane->signed_distance - V3Dot(ray->origin, plane->normal)) / dot;
}

u32 PlaneRaycast(v3 *intersection, const struct plane *plane, const struct ray *ray)
{
	const f32 t = PlaneRaycastParameter(plane, ray);
	if (t < 0.0f || t == F32_INFINITY) { return 0; }

	*intersection = V3AddScaled(ray->origin, ray->dir, t);
	return 1;
}

u32 AabbMaxAxis(const struct aabb a)
{
    u32 axis = 0;
    if (a.hw.x < a.hw.y)
    {
        axis = 1;
    }

    if (a.hw.buf[axis] < a.hw.z)
    {
        axis = 2;
    }

    return axis;
}

struct aabb BboxVertexSet(const v3 *v, const u32 count)
{
    if (count == 0)
    {
        return (struct aabb) { 0 };
    }

	v3 min = V3(F32_INFINITY, F32_INFINITY, F32_INFINITY);
	v3 max = V3(-F32_INFINITY, -F32_INFINITY, -F32_INFINITY);
	for (u32 i = 0; i < count; ++i)
	{
        min = V3Min(min, v[i]);
        max = V3Max(max, v[i]);
	}

    struct aabb bbox;
	bbox.hw = V3Scale(V3Sub(max, min), 0.5f);
	bbox.center = V3Add(min, bbox.hw);
    return bbox;
}

void AabbUnion(struct aabb *box_union, const struct aabb *a, const struct aabb *b)
{
	const v3 min = V3Min(V3Sub(a->center, a->hw), V3Sub(b->center, b->hw));
	const v3 max = V3Max(V3Add(a->center, a->hw), V3Add(b->center, b->hw));
	
	box_union->hw = V3Scale(V3Sub(max, min), 0.5f);
	box_union->center = V3Add(box_union->hw, min);
}

void AabbRotate(struct aabb *dst, const struct aabb *src, const m3 rotation)
{
	/*
	 * Since we may pick any sign for hw[k], and the support point in any direction for an AABB is
	 * one of its corners, we derive new hw as hw_new[k] = V3Abs(rot.row[k])*hw_old;
	 */

	const v3 x = V3Abs(V3(rotation.a11, rotation.a12, rotation.a13));
	const v3 y = V3Abs(V3(rotation.a21, rotation.a22, rotation.a23));
	const v3 z = V3Abs(V3(rotation.a31, rotation.a32, rotation.a33));

	dst->hw = V3(V3Dot(x, src->hw), V3Dot(y, src->hw), V3Dot(z, src->hw));
	dst->center = src->center;
}

u32 AabbTest(const struct aabb *a, const struct aabb *b)
{
	if (b->center.x - b->hw.x - (a->center.x + a->hw.x) > 0.0f 
			|| a->center.x - a->hw.x - (b->center.x + b->hw.x) > 0.0f) { return 0; }
	if (b->center.y - b->hw.y - (a->center.y + a->hw.y) > 0.0f 
			|| a->center.y - a->hw.y - (b->center.y + b->hw.y) > 0.0f) { return 0; }
	if (b->center.z - b->hw.z - (a->center.z + a->hw.z) > 0.0f 
			|| a->center.z - a->hw.z - (b->center.z + b->hw.z) > 0.0f) { return 0; }

	return 1;
}

u32 AabbContains(const struct aabb *a, const struct aabb *b)
{
	if (b->center.x - b->hw.x < a->center.x - a->hw.x) { return 0; }
	if (b->center.y - b->hw.y < a->center.y - a->hw.y) { return 0; }
	if (b->center.z - b->hw.z < a->center.z - a->hw.z) { return 0; }
	
	if (b->center.x + b->hw.x > a->center.x + a->hw.x) { return 0; }
	if (b->center.y + b->hw.y > a->center.y + a->hw.y) { return 0; }
	if (b->center.z + b->hw.z > a->center.z + a->hw.z) { return 0; }

	return 1;
}

u32 AabbContainsMargin(const struct aabb *a, const struct aabb *b, const f32 margin)
{
	if (b->center.x - b->hw.x < a->center.x - a->hw.x - margin) { return 0; }
	if (b->center.y - b->hw.y < a->center.y - a->hw.y - margin) { return 0; }
	if (b->center.z - b->hw.z < a->center.z - a->hw.z - margin) { return 0; }
	
	if (b->center.x + b->hw.x > a->center.x + a->hw.x + margin) { return 0; }
	if (b->center.y + b->hw.y > a->center.y + a->hw.y + margin) { return 0; }
	if (b->center.z + b->hw.z > a->center.z + a->hw.z + margin) { return 0; }

	return 1;
}
	
void AabbRaycastParameterExSetup(v3 *multiplier, v3u32 *dir_sign_bit, const struct ray *ray)
{
	*multiplier = V3(1.0f / (ray->dir.x),
			 1.0f / (ray->dir.y),
			 1.0f / (ray->dir.z));

	*dir_sign_bit = V3U32((u32) F32SignBit(ray->dir.x),
			      (u32) F32SignBit(ray->dir.y),
			      (u32) F32SignBit(ray->dir.z));
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
f32 AabbRaycastParameterEx(const struct aabb *aabb, const struct ray *ray, const v3 multiplier, const v3u32 dir_sign_bit)
{
	const v3 box_min = V3Sub(aabb->center, aabb->hw);
	const v3 box_max = V3Add(aabb->center, aabb->hw);

	f32 t_min = 0.0f;
	f32 t_max = F32_INFINITY;

	for (u32 axis = 0; axis < 3; ++axis)
	{
		/* If parallel to slab, point_slab test */
        //TODO fix hardcoded values here
		if (F32Abs(ray->dir.buf[axis]) < 10.0f * F32_EPSILON)
		{
			if (ray->origin.buf[axis] < box_min.buf[axis] || ray->origin.buf[axis] > box_max.buf[axis]) { return F32_INFINITY; }
		}
		else
		{
			const f32 t_1 = (box_min.buf[axis] - ray->origin.buf[axis]) * multiplier.buf[axis];
			const f32 t_2 = (box_max.buf[axis] - ray->origin.buf[axis]) * multiplier.buf[axis];

			/* if sign bit, we hit min_plane last, max_plane first => t_1 > t_2, else t_2 > t_1 */
			const f32 t_min_axis = (1-dir_sign_bit.buf[axis])*t_1 + dir_sign_bit.buf[axis]*t_2;
			const f32 t_max_axis = (1-dir_sign_bit.buf[axis])*t_2 + dir_sign_bit.buf[axis]*t_1;

			ds_Assert(t_min_axis <= t_max_axis);
			t_min = F32Max(t_min, t_min_axis);
			t_max = F32Min(t_max, t_max_axis);

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
	v3 multiplier;
	v3u32 dir_sign_bit;
	AabbRaycastParameterExSetup(&multiplier, &dir_sign_bit, ray);
	return AabbRaycastParameterEx(aabb, ray, multiplier, dir_sign_bit);
}

u32 AabbRaycastEx(v3 *intersection, const struct aabb *aabb, const struct ray *ray, const v3 multiplier, const v3u32 dir_sign_bit)
{
	const f32 t = AabbRaycastParameterEx(aabb, ray, multiplier, dir_sign_bit);
	if (t == F32_INFINITY) { return 0; }

	*intersection = V3AddScaled(ray->origin, ray->dir, t);
	return 1;
}

u32 AabbRaycast(v3 *intersection, const struct aabb *aabb, const struct ray *ray)
{
	v3 multiplier; 
	v3u32 dir_sign_bit;
	AabbRaycastParameterExSetup(&multiplier, &dir_sign_bit, ray);
	return AabbRaycastEx(intersection, aabb, ray, multiplier, dir_sign_bit);
}

u64 AabbPushLinesBuffered(u8 *buf, const u64 bufsize, const struct aabb *box, const v4 color)
{
	return AabbTransformPushLinesBuffered(buf, bufsize, box, V3Zero(), M3Identity(), color);
}

u64 AabbTransformPushLinesBuffered(u8 *buf, const u64 bufsize, const struct aabb *box, const v3 translation, const m3 rotation, const v4 color)
{
	const u64 bytes_written = 3*8*(sizeof(v3)+sizeof(v4));
	if (bufsize < bytes_written)
	{
		return 0;
	}

	const v3 end = V3Sub(box->center, box->hw);
	const v3 line[24] =
	{
		V3(end.x, 		                end.y, 		            end.z),
		V3(end.x + 2.0f*box->hw.x,    end.y, 		            end.z),
		V3(end.x, 		                end.y, 		            end.z),
		V3(end.x, 		                end.y + 2.0f*box->hw.y,   end.z),
		V3(end.x, 		                end.y, 		            end.z),
		V3(end.x, 		                end.y, 	                end.z + 2.0f*box->hw.z),

		V3(end.x + 2.0f*box->hw.x,    end.y, 		            end.z),
		V3(end.x + 2.0f*box->hw.x,    end.y + 2.0f*box->hw.y,   end.z),
		V3(end.x + 2.0f*box->hw.x,    end.y, 		            end.z),
		V3(end.x + 2.0f*box->hw.x,    end.y,                     end.z + 2.0f*box->hw.z),

		V3(end.x, 		            end.y + 2.0f*box->hw.y,   end.z),
		V3(end.x, 		            end.y + 2.0f*box->hw.y,   end.z + 2.0f*box->hw.z),
		V3(end.x, 		            end.y + 2.0f*box->hw.y,   end.z),
		V3(end.x + 2.0f*box->hw.x,   end.y + 2.0f*box->hw.y,   end.z),

		V3(end.x, 		            end.y, 	                end.z + 2.0f*box->hw.z),
		V3(end.x, 		            end.y + 2.0f*box->hw.y,   end.z + 2.0f*box->hw.z),
		V3(end.x, 		            end.y, 	                end.z + 2.0f*box->hw.z),
		V3(end.x + 2.0f*box->hw.x,   end.y, 	                end.z + 2.0f*box->hw.z),

		V3(end.x + 2.0f*box->hw.x,   end.y + 2.0f*box->hw.y,   end.z),
		V3(end.x + 2.0f*box->hw.x,   end.y + 2.0f*box->hw.y,   end.z + 2.0f*box->hw.z),

		V3(end.x, 		            end.y + 2.0f*box->hw.y,   end.z + 2.0f*box->hw.z),
		V3(end.x + 2.0f*box->hw.x,   end.y + 2.0f*box->hw.y,   end.z + 2.0f*box->hw.z),

		V3(end.x + 2.0f*box->hw.x,   end.y, 	                end.z + 2.0f*box->hw.z),
		V3(end.x + 2.0f*box->hw.x,   end.y + 2.0f*box->hw.y,   end.z + 2.0f*box->hw.z),
	};

	f32 *v = (f32*) buf;
	for (u32 i = 0; i < 24; ++i)
	{
		const v3 p = V3Add(M3V3Mul(rotation, line[i]), translation);
		memcpy(v + 7*i + 0, &p, sizeof(v3));
		memcpy(v + 7*i + 3, &color, sizeof(v4));
	}

	return bytes_written;
}

struct aabb BboxTriangle(const v3 p0, const v3 p1, const v3 p2)
{
	struct aabb bbox;

	const v3 min = V3Min(V3Min(p0, p1), p2);
	const v3 max = V3Max(V3Max(p0, p1), p2);

	bbox.hw = V3Scale(V3Sub(max, min), 0.5f);
	bbox.center = V3Add(min, bbox.hw);

	return bbox;
}

struct aabb BboxUnion(const struct aabb a, const struct aabb b)
{
	struct aabb bbox;
	const v3 min = V3Min(V3Sub(a.center, a.hw), V3Sub(b.center, b.hw));
	const v3 max = V3Max(V3Add(a.center, a.hw), V3Add(b.center, b.hw));
	
	bbox.hw = V3Scale(V3Sub(max, min), 0.5f);
	bbox.center = V3Add(bbox.hw, min);

	return bbox;
}

struct aabb	BboxPointUnion(const struct aabb a, const v3 p)
{
    const v3 diff = V3Sub(p, a.center);
    const v3 diff_abs = V3Abs(diff);

    struct aabb bbox = a;
    for (u32 i = 0; i < 3; ++i)
    {
        const f32 diff_hw = (diff_abs.buf[i] - a.hw.buf[i]) / 2.0f;
        if (diff_hw > 0.0f)
        {
            bbox.hw.buf[i] += diff_hw;
            if (diff.buf[i] < 0.0f)
            {
                bbox.center.buf[i] -= diff_hw;
            }
            else
            {
                bbox.center.buf[i] += diff_hw;
            }
        }
    }

    return bbox;
}

u32 VertexSupport(v3 *support, const v3 dir, const v3 *v, const u32 v_count)
{
	u32 best = U32_MAX;
	f32 max_dist = -F32_INFINITY;
	for (u32 i = 0; i < v_count; ++i)
	{
		const f32 dist = V3Dot(dir, v[i]);

		if (max_dist < dist)
		{
			best = i;
			max_dist = dist;
		}
	}

    ds_Assert(best != U32_MAX);
	*support = v[best];
	return best;
}

v3 VertexCentroid(const v3 *vs, const u32 n)
{
	v3 centroid = V3Zero();
	for (u32 i = 0; i < n; ++i)
	{
		centroid = V3Add(centroid, vs[i]);
	}
	return V3Scale(centroid, 1.0f / n);
}

v3 TriCcwNormal(const v3 p0, const v3 p1, const v3 p2)
{
	return V3Normalize(V3Cross(V3Sub(p1, p0), V3Sub(p2, p0)));
}

v3 TriCcwNormalDirection(const v3 p0, const v3 p1, const v3 p2)
{
	return V3Cross(V3Sub(p1, p0), V3Sub(p2, p0));
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

u32 TriVoronoiInitCcw(struct TriVoronoi *tv, const v3 t[3])
{
    tv->t[0] = t[0];
    tv->t[1] = t[1];
    tv->t[2] = t[2];

    tv->s[0] = SegmentConstruct(t[0], t[1]);
    tv->s[1] = SegmentConstruct(t[1], t[2]);
    tv->s[2] = SegmentConstruct(t[2], t[0]);

    const v3 face_normal_dir = V3Scale(V3Cross(tv->s[0].dir, tv->s[2].dir), -1.0f);
    tv->face_plane = PlaneConstruct(face_normal_dir, t[0]);

    tv->edge_plane[0] = PlaneConstruct(V3Cross(tv->s[0].dir, tv->face_plane.normal_direction), t[0]);
    tv->edge_plane[1] = PlaneConstruct(V3Cross(tv->s[1].dir, tv->face_plane.normal_direction), t[1]);
    tv->edge_plane[2] = PlaneConstruct(V3Cross(tv->s[2].dir, tv->face_plane.normal_direction), t[2]);

    const f32 n_dir_len_sq = V3Dot(tv->face_plane.normal_direction, tv->face_plane.normal_direction);
    const f32 s0_len_sq = V3Dot(tv->s[0].dir, tv->s[0].dir);
    const f32 s1_len_sq = V3Dot(tv->s[1].dir, tv->s[1].dir);
    const f32 s2_len_sq = V3Dot(tv->s[2].dir, tv->s[2].dir);
    const f32 s_max_len_sq = F32Max(F32Max(s0_len_sq, s1_len_sq), s2_len_sq);

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

f32 TriCcwPointDistanceSquared(v3 *c, enum TriVoronoiRegion *region, const v3 point, const struct TriVoronoi *tv)
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
        *c = SegmentBc(tv->s + v, param);
        if (1.0f == param)
        {
            *region = next;
        }
        else if (0.0f == param)
        {
            *region = v;
        }
    }
    else if (table_tri_voronoi_vertex_check[*region])
    {
        const u32 ij = table_sub_1_mod_3[*region];
        const u32 jk = *region;
        f32 param; 
        if (0.0f < (param = SegmentPointClosestBcParameter(tv->s + ij, point)) && param < 1.0f)
        {
            *c = SegmentBc(tv->s + ij, param);
            *region = TRI_VORONOI_EDGE01 + ij;
        }
        else if (0.0f < (param = SegmentPointClosestBcParameter(tv->s + jk, point)) && param < 1.0f)
        {
            *c = SegmentBc(tv->s + jk, param);
            *region = TRI_VORONOI_EDGE01 + jk;
        }
        else
        {
            *c = tv->t[*region];
        }
    }

    return V3DistanceSquared(point, *c);
}

f32 TriCcwSegmentClipParameter(v3 *clip, const struct segment *s, const struct TriVoronoi *tv)
{
    const f32 param = PlaneSegmentClipParameter(&tv->face_plane, s);
    if (0.0f <= param && param <= 1.0f)
    {
        *clip = SegmentBc(s, param);
        if (PlanePointBehindCheck(tv->edge_plane + 0, *clip) && 
            PlanePointBehindCheck(tv->edge_plane + 1, *clip) && 
            PlanePointBehindCheck(tv->edge_plane + 2, *clip))
        {
            return param;
        }
    }

    return F32_INFINITY;
}

u32 TriCcwSegmentClip(v3 *clip, const struct segment *s, const struct TriVoronoi *tv)
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
			if (V3Dot(s->dir, tv->edge_plane[i].normal_direction) >= 0.0f)
			{
				max_p = bc_c;
			}
			else
			{
				min_p = bc_c;
			}
		}
    }

	return SegmentConstruct(SegmentBc(s, min_p), SegmentBc(s, max_p));
}

static f32 TriCcwSegmentEdgeCheck(v3 *c_t, v3 *c_s, enum TriVoronoiRegion *region, const struct segment *s, const struct TriVoronoi *tv, const u32 start)
{
    /* Last case: segment-edge generates closest point */
    f32 s_param, t_param;
    SegmentClosestParameter(&s_param, &t_param, s, tv->s + start);
    *c_s = SegmentBc(s, s_param);
    *c_t = SegmentBc(tv->s + start, t_param);
    
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
    
    return V3DistanceSquared(*c_s, *c_t);
}

static f32 TriCcwSegmentDoubleEdgeCheck(v3 *c_t, v3 *c_s, enum TriVoronoiRegion *segment_region, const struct segment *s, const struct TriVoronoi *tv, const u32 j)
{
    const u32 i = table_sub_1_mod_3[j];
    const u32 k = table_add_1_mod_3[j];

    f32 dist_sq, s_param_ij, s_param_jk, t_param_ij, t_param_jk;
    SegmentClosestParameter(&s_param_ij, &t_param_ij, s, tv->s + i);
    SegmentClosestParameter(&s_param_jk, &t_param_jk, s, tv->s + j);

    const v3 c_s_ij = SegmentBc(s, s_param_ij);
    const v3 c_s_jk = SegmentBc(s, s_param_jk);
    const v3 c_t_ij = SegmentBc(tv->s + i, t_param_ij);
    const v3 c_t_jk = SegmentBc(tv->s + j, t_param_jk);

    const f32 dist_sq_ij = V3DistanceSquared(c_s_ij, c_t_ij);
    const f32 dist_sq_jk = V3DistanceSquared(c_s_jk, c_t_jk);
    if (dist_sq_ij < dist_sq_jk)
    {
        *c_s = c_s_ij;
        *c_t = c_t_ij;
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
        *c_s = c_s_jk;
        *c_t = c_t_jk;
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

static f32 TriCcwSegmentTripleEdgeCheck(v3 *c_t, v3 *c_s, enum TriVoronoiRegion *segment_region, const struct segment *s, const struct TriVoronoi *tv)
{
    f32 dist_sq = F32_INFINITY;
    u32 min_i = 0;
    f32 s_param[3], t_param[3];
    v3 c_s_local[3], c_t_local[3];
    for (u32 i = 0; i < 3; ++i)
    {
        SegmentClosestParameter(s_param + i, t_param + i, s, tv->s + i);
        c_s_local[i] = SegmentBc(s, s_param[i]);
        c_t_local[i] = SegmentBc(tv->s + i, t_param[i]);
        const f32 dist_sq_local = V3DistanceSquared(c_s_local[i], c_t_local[i]);
        if (dist_sq_local < dist_sq)
        {
            dist_sq = dist_sq_local;
            min_i = i;
        }
    }

    *c_s = c_s_local[min_i];
    *c_t = c_t_local[min_i];
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

f32 TriCcwSegmentDistanceSquared(v3 *c_t, v3 *c_s, enum TriVoronoiRegion *segment_region, const struct segment *s, const struct TriVoronoi *tv)
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
        const f32 param = F32Clamp(PlaneSegmentClipParameter(&tv->face_plane, &s_canon), 0.0f, 1.0f);

        *segment_region = TRI_VORONOI_FACE;
        *c_s = SegmentBc(&s_canon, param);
        PlanePointProjection(c_t, &tv->face_plane, *c_s);
        dist_sq = (param == 0.0f || param == 1.0f)
                ? V3DistanceSquared(*c_t, *c_s)
                : 0.0f;
    }
    else if (region[high] == TRI_VORONOI_FACE)
    {
        const u32 infront_face = PlanePointInfrontCheck(&tv->face_plane, s_canon.p[0]);
        const f32 dot_dir = V3Dot(tv->face_plane.normal_direction, s_canon.dir);
        /* First case: end-point in FACE is closest */
        if ((infront_face && dot_dir >= 0.0f) || (!infront_face && dot_dir <= 0.0f))
        {
            *segment_region = TRI_VORONOI_FACE;
            *c_s = s_canon.p[0];
            PlanePointProjection(c_t, &tv->face_plane, *c_s);
            dist_sq = V3DistanceSquared(*c_t, *c_s);
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
            if (V3Dot(side_pl.normal, s_canon.dir) < 0.0f)
            {
                *segment_region = TRI_VORONOI_FACE;
                PlaneSegmentClip(c_s, &tv->face_plane, s);
                *c_t = *c_s;
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

            const f32 dot_ij = V3Dot(pl_ij.normal, s_canon.dir);
            const f32 dot_jk = V3Dot(pl_jk.normal, s_canon.dir);

            /* Segment clips face */
            if (dot_ij < 0.0f && dot_jk < 0.0f)
            {
                *segment_region = TRI_VORONOI_FACE;
                PlaneSegmentClip(c_s, &tv->face_plane, &s_canon);
                *c_t = *c_s;
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
                    *c_t = *c_s;
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
                    *c_t = *c_s;
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

v3 box_stub_vertex[8] =
{
	{ .x = 0.5f, .y = 0.5f, .z = 0.5f },
	{ .x = 0.5f, .y = 0.5f, .z = -0.5f },
	{ .x = -0.5f, .y = 0.5f, .z = -0.5f },
	{ .x = -0.5f, .y = 0.5f, .z = 0.5f },
	{ .x = 0.5f, .y = -0.5f, .z = 0.5f },
	{ .x = 0.5f, .y = -0.5f, .z = -0.5f },
	{ .x = -0.5f, .y = -0.5f, .z = -0.5f },
	{ .x = -0.5f, .y = -0.5f, .z = 0.5f },
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

struct dcel DcelBox(struct arena *mem, const v3 hw)
{
	v3 *box_vertex = ArenaPush(mem, 8*sizeof(v3));

	box_vertex[0] = V3(hw.x,  hw.y,  hw.z); 
	box_vertex[1] = V3(hw.x,  hw.y, -hw.z);	
	box_vertex[2] = V3(-hw.x,  hw.y, -hw.z);	
	box_vertex[3] = V3(-hw.x,  hw.y,  hw.z);	
	box_vertex[4] = V3(hw.x, -hw.y,  hw.z);
	box_vertex[5] = V3(hw.x, -hw.y, -hw.z);	
	box_vertex[6] = V3(-hw.x, -hw.y, -hw.z);	
	box_vertex[7] = V3(-hw.x, -hw.y,  hw.z);	

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

v3 DcelFaceNormal(const struct dcel *h, const m3 rot, const u32 fi)
{
    return M3V3Mul(rot, DcelFaceNormalLocal(h, fi));
}

v3 DcelFaceDirection(const struct dcel *h, const m3 rot, const u32 fi)
{
    return M3V3Mul(rot, DcelFaceDirectionLocal(h, fi));
}

v3 DcelFaceDirectionLocal(const struct dcel *h, const u32 fi)
{
	struct dcelEdge *e0 = h->e + h->f[fi].first;
	struct dcelEdge *e1 = h->e + h->f[fi].first + 1;
	struct dcelEdge *e2 = h->e + h->f[fi].first + 2;
    return TriCcwNormalDirection(h->v[e0->origin], h->v[e1->origin], h->v[e2->origin]);
}

v3 DcelFaceNormalLocal(const struct dcel *h, const u32 fi)
{
	const v3 normal = DcelFaceDirectionLocal(h, fi);
	return V3Scale(normal, 1.0f/V3Length(normal));
}

struct plane DcelFacePlane(const struct dcel *h, const m3 rot, const v3 pos, const u32 fi)
{
	const v3 n = M3V3Mul(rot, DcelFaceNormalLocal(h, fi));
	const v3 p = V3Add(M3V3Mul(rot, h->v[h->e[h->f[fi].first].origin]), pos);
	return PlaneConstruct(n, p);
}

struct plane DcelFacePlaneLocal(const struct dcel *h, const u32 fi)
{
    const u32 i0  = h->e[h->f[fi].first + 0].origin;
    const u32 i1  = h->e[h->f[fi].first + 1].origin;
    const u32 i2  = h->e[h->f[fi].first + 2].origin;
    return PlaneConstructFromCcwTriangle(h->v[i0], h->v[i1], h->v[i2]);
}

struct segment DcelFaceClipSegment(const struct dcel *h, const m3 rot, const v3 pos, const u32 fi, const struct segment *s)
{
	const v3 f_n = DcelFaceNormal(h, rot, fi);

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
			if (V3Dot(s->dir, clip_plane.normal) >= 0.0f)
			{
				max_p = bc_c;
			}
			else
			{
				min_p = bc_c;
			}
		}
	}	

	return SegmentConstruct(SegmentBc(s, min_p), SegmentBc(s, max_p));
}

struct plane DcelFaceClipPlane(const struct dcel *h, const m3 rot, const v3 pos, const v3 face_normal, const u32 e0, const u32 e1)
{
	struct dcelEdge *edge0 = h->e + e0; 
	struct dcelEdge *edge1 = h->e + e1; 

	const v3 p0 = V3Add(M3V3Mul(rot, h->v[edge0->origin]), pos);
	const v3 p1 = V3Add(M3V3Mul(rot, h->v[edge1->origin]), pos);
	const v3 n = V3Cross(V3Sub(p1, p0), face_normal);

	return PlaneConstruct(V3Scale(n, 1.0f/V3Length(n)), p0);
}

u32 DcelFaceProjectedPointTest(const struct dcel *h, const m3 rot, const v3 pos, const u32 fi, const v3 p)
{
	const v3 f_n = DcelFaceNormal(h, rot, fi);

	struct dcelFace *f = h->f + fi;
	for (u32 i = 0; i < f->count; ++i)
	{
		const u32 e0 = f->first + i;
		const u32 e1 = f->first + ((i + 1) % f->count);
		struct plane clip_plane = DcelFaceClipPlane(h, rot, pos, f_n, e0, e1);
		if (V3Dot(clip_plane.normal, p) > clip_plane.signed_distance)
		{
			return 0;
		}
	}	

	return 1;
}

v3 DcelEdgeDirection(const struct dcel *h, const u32 ei)
{
	struct dcelEdge *e0 = h->e + ei;
	struct dcelFace *f = h->f + e0->face_ccw;
	const u32 next = f->first + ((ei - f->first + 1) % f->count);
	struct dcelEdge *e1 = h->e + next;
	return V3Sub(h->v[e1->origin], h->v[e0->origin]);
}

v3 DcelEdgeNormal(const struct dcel *h, const u32 ei)
{
	const v3 dir = DcelEdgeDirection(h, ei);
	return V3Scale(dir, 1.0f / V3Length(dir));
}

struct segment DcelEdgeSegment(const struct dcel *h, const m3 rot, const v3 pos, const u32 ei)
{
	const u32 first = h->f[h->e[ei].face_ccw].first;
	const u32 count = h->f[h->e[ei].face_ccw].count;
	const u32 e0 = ei;
	const u32 e1 = first + ((ei - first + 1) % count); 

	const v3 p0 = V3Add(M3V3Mul(rot, h->v[h->e[e0].origin]), pos);
	const v3 p1 = V3Add(M3V3Mul(rot, h->v[h->e[e1].origin]), pos);

	return SegmentConstruct(p0, p1);
}

v3 SphereSupport(const v3 dir, const struct sphere *sph, const v3 pos)
{
	return V3Add(V3Scale(dir, sph->radius / V3Length(dir)), pos);
}

v3 CapsuleSupport(const v3 dir, const struct capsule *cap, const m3 rot, const v3 pos)
{
    const v3 p1 = V3Scale(rot.col[1], cap->half_height);
	const v3 p2 = V3Negate(p1);

	const v3 support = V3Add(V3Scale(dir, cap->radius / V3Length(dir)), pos);
	return (V3Dot(dir, p1) > V3Dot(dir, p2))
		? V3Add(support, p1) 
		: V3Add(support, p2);
}

u32 DcelSupport(v3 *support, const v3 dir, const struct dcel *dcel, const m3 rot, const v3 pos)
{
	f32 max = -F32_INFINITY;
	u32 max_index = 0;
	for (u32 i = 0; i < dcel->v_count; ++i)
	{
		const v3 p = M3V3Mul(rot, dcel->v[i]);
		const f32 dot = V3Dot(p, dir);
		if (max < dot)
		{
			max_index = i;
			max = dot; 
		}
	}

	*support = V3Add(M3V3Mul(rot, dcel->v[max_index]), pos);
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
				, dcel->v[i].buf[0]
				, dcel->v[i].buf[1]
				, dcel->v[i].buf[2]);
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
	v3		        normal;
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
	const v3 *		v;
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

	v3 center = V3Zero();
	for (u32 i = 0; i < ddcel->v_count; ++i)
	{
		if (vertex_check[i])
		{
			center = V3Add(center, ddcel->v[i]);
		}
	}
	center = V3Scale(center, 1.0f/vertex_count);

	for (u32 i = 0; i < ddcel->face_pool.count_max; ++i)
	{
		if (ds_PoolSlotAllocated(ddcel->f + i))
		{
			v3 diff;
			diff = V3Sub(center, ddcel->v[ddcel->e[ddcel->f[i].first].origin]);
			ds_Assert(V3Dot(diff, ddcel->f[i].normal) < 0.0f);
		}
	}

    ArenaPopScratch();
	ds_Assert(face_count >= 4);
}

u32 InternalConvexHullTetrahedronIndices(struct ddcel *ddcel, const f32 tol)
{
	v3 a, b, n;

	const f32 tol_sq = tol*tol;
	u32 indices[4] = { 0 };
	u32 i = 1;
	/* Find two points not to close to each other */
	for (; i < ddcel->v_count; ++i)
	{
		a = V3Sub(ddcel->v[ddcel->cv[i].index], ddcel->v[ddcel->cv[0].index]);
		const f32 dist_sq = V3Dot(a, a);
		if (dist_sq > tol_sq)
		{
			//a = V3Scale(a, 1.0f / len);
			indices[1] = i;
			i += 1;
			break;
		}
	}	

	/* Find non-collinear point */
	for (; i < ddcel->v_count; ++i)
	{
		b = V3Sub(ddcel->v[ddcel->cv[i].index], ddcel->v[ddcel->cv[0].index]);
		n = V3Cross(a, b);
		const f32 dist = V3Length(n);
		const f32 area = dist / 2.0f;
		if (area > tol_sq)
		{
			indices[2] = i;
			i += 1;
			n = V3Scale(n, 1.0f / dist);
			break;
		}
	}

	/* Find non-coplanar point */
	for (; i < ddcel->v_count; ++i)
	{
		a = V3Sub(ddcel->v[ddcel->cv[i].index], ddcel->v[ddcel->cv[0].index]);
		const f32 height = V3Dot(a, n);
		if (F32Abs(height) > tol)
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
	const v3 *v = ddcel->v;
	const u32 v_count = ddcel->v_count;
	v3 a, b, c, cr;
	a = V3Sub(v[ddcel->cv[1].index], v[ddcel->cv[0].index]);
	b = V3Sub(v[ddcel->cv[2].index], v[ddcel->cv[0].index]);
	c = V3Sub(v[ddcel->cv[3].index], v[ddcel->cv[0].index]);
	cr = V3Cross(a, b);

	/* CCW == inside gives negative dot product for any polygon on a convex polyhedron */
	if (V3Dot(cr, c) > 0.0f)
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

    ddcel->f[0].normal = TriCcwNormal(ddcel->v[e0->origin], ddcel->v[e1->origin], ddcel->v[e2->origin]);
    ddcel->f[1].normal = TriCcwNormal(ddcel->v[e3->origin], ddcel->v[e4->origin], ddcel->v[e5->origin]);
    ddcel->f[2].normal = TriCcwNormal(ddcel->v[e6->origin], ddcel->v[e7->origin], ddcel->v[e8->origin]);
    ddcel->f[3].normal = TriCcwNormal(ddcel->v[e9->origin], ddcel->v[e10->origin], ddcel->v[e11->origin]);

	DdcelAssertTopology(ddcel);
}

static void InternalConvexHullTetrahedronConflicts(struct ddcel *ddcel, const f32 tol)
{
	const v3 *v = ddcel->v;
	const u32 v_count = ddcel->v_count;
	v3 b;
	for (u32 cv_i = 4; cv_i < v_count; ++cv_i)
	{
		struct conflictVertex *cv = ddcel->cv + cv_i;
		for (u32 f_i = 0; f_i < 4; ++f_i)
		{
			const u32 v0_i = ddcel->e[ddcel->f[f_i].first].origin;
			b = V3Sub(v[cv->index], v[v0_i]);
			/* If point is "in front" of face, we have a conflict */
			if (V3Dot(ddcel->f[f_i].normal, b) > tol)
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

				v3 diff;
				diff = V3Sub(ddcel->v[cv->index], ddcel->v[e->origin]);
				ddcel->hv[e->origin].colinear = (F32Abs(V3Dot(f_twin->normal, diff)) < tol)
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
            f->normal = TriCcwNormal(ddcel->v[e0->origin], ddcel->v[e1->origin], ddcel->v[e2->origin]);
		
			/*TODO: We may add same point twich here, need to add a "has_been_mapped" thingy to not add again*/
			ce = NULL;
			for (i32 j = f_ccw->ce_list.first; j != DLL_SENTINEL; j = ce->face_edge.next)
			{
				ce = ddcel->ce + j;
				if (ce->vertex != cvi && (ddcel->cv[ce->vertex].last_face != sf.index || ddcel->cv[ce->vertex].last_iter != cvi))
				{
					ddcel->cv[ce->vertex].last_iter = cvi;
					ddcel->cv[ce->vertex].last_face = sf.index;
					v3 diff;
					diff = V3Sub(ddcel->v[ddcel->cv[ce->vertex].index], ddcel->v[e0->origin]);
					if (V3Dot(f->normal, diff) > tol)
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
				v3 diff;
				ds_Assert(ce->vertex != cvi)
				if (ddcel->cv[ce->vertex].last_face != sf.index || ddcel->cv[ce->vertex].last_iter != cvi)
				{
					ddcel->cv[ce->vertex].last_iter = cvi;
					ddcel->cv[ce->vertex].last_face = sf.index;
					diff = V3Sub(ddcel->v[ddcel->cv[ce->vertex].index], ddcel->v[e0->origin]);
					if (V3Dot(f->normal, diff) > tol)
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
		.v = ArenaPushMemcpy(mem, ddcel->v, ddcel->v_count*sizeof(v3)),
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

struct dcel DcelConvexHull(struct arena *mem, const v3 *v, const u32 v_count, const f32 tol)
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
	return BboxVertexSet(mesh->v, mesh->v_count);	
}

f32 TriMeshRaycastParameter(const struct triMesh *mesh, const u32 tri, const struct ray *ray)
{
	v3 intersection;
	f32 t = F32_INFINITY;
	if (TriMeshRaycast(&intersection, mesh, tri, ray))
	{
		t = RayPointClosestPointParameter(ray, intersection);
	}
	return t;
}

u32 TriMeshRaycast(v3 *intersection, const struct triMesh *mesh, const u32 tri, const struct ray *ray)
{
	/* TODO(Research): watertight raycasting.
	 * Shared edges are consistent (canonicalized edge tests below), but rays passing exactly through a
	 * shared vertex can still slip between all triangles of the fan (~0.1% of exactly vertex-aimed rays
	 * in tests), since the signs of the float triple products are not exact. Woop, Benthin, Wald 2013,
	 * "Watertight Ray/Triangle Intersection" (JCGT 2(1)), makes every edge test sign exact: per-ray shear
	 * to ray space, 2D edge functions on per-vertex transformed coordinates, double precision recompute
	 * when an edge function is exactly 0.0. It also needs no edge canonicalization and no face normal.
	 */

	/* TODO(Optimization):
	 * By precomputation and extending our tri_mesh structure, we can avoid these branches;
	 * so if it becomes relevant, we need to precompute the edge sorting or something...  
	 */

	/* canonicalize edges: We require consistency between triangles sharing edges; if the 
	 * test determining which side of the edge the ray intersects the triangle planes are
	 * not done in the same way, we may miss collision with both triangles despite hitting
	 * one of them. See (Real Time Collision Detection, 5.3.4 and 11.3.3) for algorithm and
	 * robustness discussion. */

	const v3 oa = V3Sub(mesh->v[mesh->tri[tri].x], ray->origin);
	const v3 ob = V3Sub(mesh->v[mesh->tri[tri].y], ray->origin);
	const v3 oc = V3Sub(mesh->v[mesh->tri[tri].z], ray->origin);
    const v3 n = TriCcwNormalDirection(mesh->v[mesh->tri[tri].x], mesh->v[mesh->tri[tri].y], mesh->v[mesh->tri[tri].z]);

    const u32 infront = (V3Dot(n, oa) < 0.0f); 
    const u32 towards = (V3Dot(n, ray->dir) < 0.0f);

    /* Exit if infront and along OR behind and towards */
    if (infront != towards)
    {
        return 0;
    }

    const f32 side_sign = (infront)
        ? 1.0f
        : -1.0f;

	f32 u, v, w;
	if (mesh->tri[tri].x < mesh->tri[tri].y)
	{
		const v3 c = V3Cross(ob, oa);
		u = side_sign*V3Dot(ray->dir, c);
	}
	else
	{
		const v3 c = V3Cross(oa, ob);
		u = -side_sign*V3Dot(ray->dir, c);
	}
	if (u < 0.0f) { return 0; }

	if (mesh->tri[tri].y < mesh->tri[tri].z)
	{
		const v3 c = V3Cross(oc, ob);
		v = side_sign*V3Dot(ray->dir, c);
	}
	else
	{
		const v3 c = V3Cross(ob, oc);
		v = -side_sign*V3Dot(ray->dir, c);
	}
	if (v < 0.0f) { return 0; }

	if (mesh->tri[tri].z < mesh->tri[tri].x)
	{
		const v3 c = V3Cross(oa, oc);
		w = side_sign*V3Dot(ray->dir, c);
	}
	else
	{
		const v3 c = V3Cross(oc, oa);
		w = -side_sign*V3Dot(ray->dir, c);
	}
	if (w < 0.0f) { return 0; }

	/* TODO: Prob bad, we can go back to this later
	 * KNOWN ISSUE: rays lying in (or parallel to) the triangle plane may report a false hit at t = 0. For such
	 * rays n.d ~ 0 and n.oa ~ 0, so infront/towards are decided by rounding noise, and u, v, w are (noisy) zeros
	 * that can all pass >= 0 (always for exactly axis-aligned planes); we then end up here and return the ray
	 * origin as the intersection. Measured: 100% of in-plane rays on horizontal triangles, ~6% on tilted ones.
	 * Returning 0 here is not enough on its own (the noise in u, v, w scales with |oa||ob|, so far away origins
	 * pass the absolute threshold with garbage weights). Tested fix: before the infront/towards test, reject
	 * rays parallel to the plane relative to scale, (n.d)^2 <= (100*F32_EPSILON)^2 * |n|^2 * |d|^2, and return 0
	 * here instead of the origin. See TODO(Research) above for the robust alternative.
	 */
	if (u + v + w < 100.0f * F32_EPSILON)
	{
		*intersection = ray->origin;
	}
	else
	{
		const f32 denom = 1.0f / (u + v + w);
		u *= denom;
		v *= denom;
		w *= denom;

		*intersection = V3AddScaled(
                            V3AddScaled(
                                V3Scale(mesh->v[mesh->tri[tri].x], v), 
                                mesh->v[mesh->tri[tri].y], w),
					        mesh->v[mesh->tri[tri].z], u);
	}

	return 1;
}
