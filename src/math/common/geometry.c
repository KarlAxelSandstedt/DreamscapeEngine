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
#include "ds_list.h"
#include "queue.h"
#include "ds_float.h"

//TODO what to do with this?

struct ray RayConstruct(const v3 origin, const v3 dir)
{
	ds_Assert(V3LengthSquared(dir) > 0.0f);

	const struct ray r = { .origin = origin, .dir = dir };
	return r;
}

struct segment SegmentConstruct(const v3 p0, const v3 p1)
{
	const struct segment s = { .p = { p0, p1 }, .dir = V3Sub(p1, p0) };
	return s;
}

void SegmentClosestParameter(f32 *t1, f32 *t2, const struct segment *s1, const struct segment *s2)
{
	const v3 diff = V3Sub(s2->p[0], s1->p[0]);
	const f32 d1d1 = V3LengthSquared(s1->dir);
	const f32 d2d2 = V3LengthSquared(s2->dir);

	*t1 = 0.0f;
	*t2 = 0.0f;

	/*
	 * A segment is a point only if |dir|^2 == 0: every quotient below is clamped (Inf included) or already in
	 * [0, divisor], so short segments need no length threshold.
	 */
	if (d1d1 > 0.0f && d2d2 > 0.0f)
	{
		const f32 d1d2 = V3Dot(s1->dir, s2->dir);
		const f32 diffd1 = V3Dot(diff, s1->dir);
		const f32 diffd2 = V3Dot(diff, s2->dir);
		const f32 denom = d1d1*d2d2 - d1d2*d1d2;
		/* Check that the segments are not parallel */
        //TODO is 0.0f good here, or should we use a degree test (V3ParallelCheck)?
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
	else if (d1d1 > 0.0f)
	{
		/* 
		 * SIGNED PROJECTED LENGTH 
		 * 	= (L2_P1 - L1_P1) * DIR1 / |DIR1| 
		 * 	= t1*|DIR1|
		 * => t = DIFF*DIR1 / (DIR1*DIR1) 
		 */
        *t1 = F32Clamp(V3Dot(diff, s1->dir) / d1d1, 0.0f, 1.0f);
	}
	else if (d2d2 > 0.0f)
	{
		*t2 = F32Clamp(-V3Dot(diff, s2->dir) / d2d2, 0.0f, 1.0f);
	}

	ds_Assert(0.0f <= *t1 && *t1 <= 1.0f);
	ds_Assert(0.0f <= *t2 && *t2 <= 1.0f);
}

u32 SegmentLineClosestParameterCheck(f32 *t1, f32 *t2, const struct segment *s1, const struct segment *s2)
{
	/*
	 * Lines s1.p0 + t1 d1, s2.p0 + t2 d2, with w = s1.p0 - s2.p0: the closest pair is where the difference is
	 * perpendicular to both directions,
	 *
	 *	t1 = (d1d2 d2w - d1w d2d2) / den,  t2 = (d1d1 d2w - d1d2 d1w) / den
	 *	den = d1d1 d2d2 - d1d2^2 = |d1 x d2|^2
	 *
	 * den from the cross product: no cancellation for nearly parallel lines.
	 */
	const v3 d1 = s1->dir;
	const v3 d2 = s2->dir;
	const v3 w = V3Sub(s1->p[0], s2->p[0]);
	const f32 d1d1 = V3Dot(d1, d1);
	const f32 d1d2 = V3Dot(d1, d2);
	const f32 d2d2 = V3Dot(d2, d2);
	const f32 d1w = V3Dot(d1, w);
	const f32 d2w = V3Dot(d2, w);
	const v3 cross = V3Cross(d1, d2);
	const f32 den = V3Dot(cross, cross);
	ds_Assert(den > 0.0f);

	*t1 = (d1d2*d2w - d1w*d2d2) / den;
	*t2 = (d1d1*d2w - d1d2*d1w) / den;
	return (0.0f <= *t1 && *t1 <= 1.0f && 0.0f <= *t2 && *t2 <= 1.0f);
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
	
	if (V3LengthSquared(s->dir) > 0.0f)
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
    const v3 p = QUnitRotate(t->rotation, V3(0.0f, cap->half_height, 0.0f));
	return SegmentConstruct(V3Add(p, t->position), V3Add(V3Negate(p), t->position));
}

struct aabb BboxSegment(const struct segment *s)
{
	const struct aabb bbox = { .min = V3Min(s->p[0], s->p[1]), .max = V3Max(s->p[0], s->p[1]) };
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
	pl.signed_distance = V3Dot(n, p);
	return pl;
}

struct plane PlaneConstructNormalized(const v3 n, const v3 p)
{
	struct plane pl;
	pl.normal = V3Normalize(n);
	pl.signed_distance = V3Dot(pl.normal, p);
	return pl;
}

struct plane PlaneConstructFromCcwTriangle(const v3 a, const v3 b, const v3 c)
{
	return PlaneConstruct(V3Cross(V3Sub(b, a), V3Sub(c, a)), a);
}

void PlaneNormalize(struct plane *pl)
{
    const f32 n_dir_len = V3Length(pl->normal_direction);
    pl->normal_direction = V3Scale(pl->normal_direction, 1.0f/n_dir_len);
    pl->signed_distance /= n_dir_len;
}

u32 PlanePointInfrontCheck(const struct plane *pl, const v3 p)
{
	return (PlanePointSignedDistance(pl, p) > 0.0f);
}

u32 PlanePointBehindCheck(const struct plane *pl, const v3 p)
{
	return (PlanePointSignedDistance(pl, p) < 0.0f);
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
	 * degenerate case: segment parallel and lies on plane gives t = NaN, not okay!
	 */
    const f32 dot_pn = V3Dot(pl->normal_direction, s->p[0]);
    const f32 dot_dn = V3Dot(pl->normal_direction, s->dir);
    const f32 t = (pl->signed_distance - dot_pn) / dot_dn;
	return t;
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

struct plane PlaneTransform(const struct plane *pl, const m3 rot, const v3 pos)
{
	/*
	 * A point x on pl (dot(n, x) = d) moves to rot x + pos. With n' = rot n, rotations keep dot products:
	 *      dot(n', rot x + pos) = dot(n, x) + dot(n', pos) = d + dot(n', pos)
	 * so the moved plane is (n', d + dot(n', pos)).
	 */
	struct plane moved;
	moved.normal_direction = M3V3Mul(rot, pl->normal_direction);
	moved.signed_distance = pl->signed_distance + V3Dot(moved.normal_direction, pos);
	return moved;
}

f32 PlanePointProjection(v3 *proj, const struct plane *pl, const v3 p)
{
	/*
	 * x = p - t n lies on the plane when dot(n, p - t n) = signed_distance, i.e.
	 *      t = (dot(n, p) - signed_distance) / dot(n, n) = PlanePointSignedDistance / |n|^2
	 */
	const f32 inv_dot_nn = 1.0f / V3Dot(pl->normal_direction, pl->normal_direction);
	const f32 n_units = PlanePointSignedDistance(pl, p) * inv_dot_nn;
    *proj = V3AddScaled(p, pl->normal_direction, -n_units);
    return n_units;
}

u32 AabbMaxAxis(const struct aabb a)
{
    const v3 size = V3Sub(a.max, a.min);
    u32 axis = 0;
    if (size.x < size.y)
    {
        axis = 1;
    }

    if (size.buf[axis] < size.z)
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

    const struct aabb bbox = { .min = min, .max = max };
    return bbox;
}

void AabbUnion(struct aabb *box_union, const struct aabb *a, const struct aabb *b)
{
	box_union->min = V3Min(a->min, b->min);
	box_union->max = V3Max(a->max, b->max);
}

void AabbRotate(struct aabb *dst, const struct aabb *src, const m3 rotation)
{
	/*
	 * The box is {c + hw*s : s in [-1, 1]^3}; rotated, {R c + R (hw*s)}. The support point of a box in any
	 * direction is a corner, so the rotated box's half widths are hw_new[k] = |R row k| . hw, around R c.
	 */

	const v3 x = V3Abs(V3(rotation.a11, rotation.a12, rotation.a13));
	const v3 y = V3Abs(V3(rotation.a21, rotation.a22, rotation.a23));
	const v3 z = V3Abs(V3(rotation.a31, rotation.a32, rotation.a33));

	const v3 center = V3Scale(V3Add(src->min, src->max), 0.5f);
	const v3 hw_src = V3Scale(V3Sub(src->max, src->min), 0.5f);
	const v3 hw = V3(V3Dot(x, hw_src), V3Dot(y, hw_src), V3Dot(z, hw_src));
	const v3 center_rotated = M3V3Mul(rotation, center);
	dst->min = V3Sub(center_rotated, hw);
	dst->max = V3Add(center_rotated, hw);
}

void AabbTranslateOutward(struct aabb *dst, const struct aabb *local, const v3f64 origin)
{
    dst->min = V3F64DemoteDownwards(V3F64Add(origin, V3Promote(local->min)));
    dst->max = V3F64DemoteUpwards(V3F64Add(origin, V3Promote(local->max)));
}

u32 AabbTest(const struct aabb *a, const struct aabb *b)
{
	if (b->min.x > a->max.x || a->min.x > b->max.x) { return 0; }
	if (b->min.y > a->max.y || a->min.y > b->max.y) { return 0; }
	if (b->min.z > a->max.z || a->min.z > b->max.z) { return 0; }

	return 1;
}

u32 AabbContains(const struct aabb *a, const struct aabb *b)
{
	if (b->min.x < a->min.x) { return 0; }
	if (b->min.y < a->min.y) { return 0; }
	if (b->min.z < a->min.z) { return 0; }
	
	if (b->max.x > a->max.x) { return 0; }
	if (b->max.y > a->max.y) { return 0; }
	if (b->max.z > a->max.z) { return 0; }

	return 1;
}

u32 AabbContainsMargin(const struct aabb *a, const struct aabb *b, const f32 margin)
{
	if (b->min.x < a->min.x - margin) { return 0; }
	if (b->min.y < a->min.y - margin) { return 0; }
	if (b->min.z < a->min.z - margin) { return 0; }
	
	if (b->max.x > a->max.x + margin) { return 0; }
	if (b->max.y > a->max.y + margin) { return 0; }
	if (b->max.z > a->max.z + margin) { return 0; }

	return 1;
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

	const v3 end = box->min;
	const v3 size = V3Sub(box->max, box->min);
	const v3 line[24] =
	{
		V3(end.x, 		                end.y, 		            end.z),
		V3(end.x + size.x,    end.y, 		            end.z),
		V3(end.x, 		                end.y, 		            end.z),
		V3(end.x, 		                end.y + size.y,   end.z),
		V3(end.x, 		                end.y, 		            end.z),
		V3(end.x, 		                end.y, 	                end.z + size.z),

		V3(end.x + size.x,    end.y, 		            end.z),
		V3(end.x + size.x,    end.y + size.y,   end.z),
		V3(end.x + size.x,    end.y, 		            end.z),
		V3(end.x + size.x,    end.y,                     end.z + size.z),

		V3(end.x, 		            end.y + size.y,   end.z),
		V3(end.x, 		            end.y + size.y,   end.z + size.z),
		V3(end.x, 		            end.y + size.y,   end.z),
		V3(end.x + size.x,   end.y + size.y,   end.z),

		V3(end.x, 		            end.y, 	                end.z + size.z),
		V3(end.x, 		            end.y + size.y,   end.z + size.z),
		V3(end.x, 		            end.y, 	                end.z + size.z),
		V3(end.x + size.x,   end.y, 	                end.z + size.z),

		V3(end.x + size.x,   end.y + size.y,   end.z),
		V3(end.x + size.x,   end.y + size.y,   end.z + size.z),

		V3(end.x, 		            end.y + size.y,   end.z + size.z),
		V3(end.x + size.x,   end.y + size.y,   end.z + size.z),

		V3(end.x + size.x,   end.y, 	                end.z + size.z),
		V3(end.x + size.x,   end.y + size.y,   end.z + size.z),
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
	const struct aabb bbox = { .min = V3Min(V3Min(p0, p1), p2), .max = V3Max(V3Max(p0, p1), p2) };
	return bbox;
}

struct aabb BboxUnion(const struct aabb a, const struct aabb b)
{
	const struct aabb bbox = { .min = V3Min(a.min, b.min), .max = V3Max(a.max, b.max) };
	return bbox;
}

struct aabb	BboxPointUnion(const struct aabb a, const v3 p)
{
	const struct aabb bbox = { .min = V3Min(a.min, p), .max = V3Max(a.max, p) };
    return bbox;
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

enum voronoi TriSegmentVoronoi(f32 w[3], f32 *s, const v3 t0, const v3 t1, const v3 t2, const v3 s0, const v3 s1)
{
    /*
     * Ericson 5.1.10: the segment pierces the triangle, or the closest pair involves a segment end point (point vs
     * triangle) or a triangle edge (segment vs segment). Each candidate is exact up to rounding; the closest wins.
     *
     * Piercing is decided from the signs of the end points' plane distances h_i (unnormalized normal): strictly
     * opposite signs bound lambda = h0 / (h0 - h1) to (0, 1). A crossing point just outside the face by rounding
     * falls to the edge candidates, which then give distance ~0.
     */
    const v3 t[3] = { t0, t1, t2 };
    const v3 n = V3Cross(V3Sub(t1, t0), V3Sub(t2, t0));
    const f32 h0 = V3Dot(n, V3Sub(s0, t0));
    const f32 h1 = V3Dot(n, V3Sub(s1, t0));
    if ((h0 < 0.0f && 0.0f < h1) || (h1 < 0.0f && 0.0f < h0))
    {
        const f32 lambda = h0 / (h0 - h1);
        const v3 x = V3Add(s0, V3Scale(V3Sub(s1, s0), lambda));
        if (TriVoronoi(w, t0, t1, t2, x) == VORONOI_F012)
        {
            *s = lambda;
            return VORONOI_F012;
        }
    }

    enum voronoi best = VORONOI_INVALID;
    f32 best_dist_sq = F32_INFINITY;
    f32 best_w[3] = { 0.0f, 0.0f, 0.0f };
    f32 best_s = 0.0f;

    /* end points vs the triangle */
    const v3 end[2] = { s0, s1 };
    for (u32 i = 0; i < 2; ++i)
    {
        f32 w_i[3];
        const enum voronoi r = TriVoronoi(w_i, t0, t1, t2, end[i]);
        if (r == VORONOI_INVALID)
        {
            continue;
        }
        const v3 c = V3Add(V3Add(V3Scale(V3Sub(t0, end[i]), w_i[0]), V3Scale(V3Sub(t1, end[i]), w_i[1])), V3Scale(V3Sub(t2, end[i]), w_i[2]));
        const f32 dist_sq = V3Dot(c, c);
        if (dist_sq < best_dist_sq)
        {
            best_dist_sq = dist_sq;
            best = r;
            best_w[0] = w_i[0];
            best_w[1] = w_i[1];
            best_w[2] = w_i[2];
            best_s = (f32) i;
        }
    }

    /* the edges vs the segment */
    const struct segment seg = SegmentConstruct(s0, s1);
    for (u32 a = 0; a < 3; ++a)
    {
        const u32 b = (a + 1) % 3;
        const struct segment edge = SegmentConstruct(t[a], t[b]);
        f32 t_edge, t_seg;
        SegmentClosestParameter(&t_edge, &t_seg, &edge, &seg);
        const v3 c = V3Sub(SegmentBc(&edge, t_edge), SegmentBc(&seg, t_seg));
        const f32 dist_sq = V3Dot(c, c);
        if (dist_sq < best_dist_sq)
        {
            best_dist_sq = dist_sq;
            best_w[0] = 0.0f;
            best_w[1] = 0.0f;
            best_w[2] = 0.0f;
            best_w[a] = 1.0f - t_edge;
            best_w[b] = t_edge;
            best = (enum voronoi) (((t_edge < 1.0f) << a) | ((0.0f < t_edge) << b));
            best_s = t_seg;
        }
    }

    w[0] = best_w[0];
    w[1] = best_w[1];
    w[2] = best_w[2];
    *s = best_s;
    return best;
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

/* box_face's planes for box_stub_vertex (what DcelPlanesTryCompute returns for them) */
static struct plane box_stub_plane[6] =
{
	{ .normal = {{  0.0f,  1.0f,  0.0f }}, .signed_distance = 0.5f },
	{ .normal = {{  1.0f,  0.0f,  0.0f }}, .signed_distance = 0.5f },
	{ .normal = {{  0.0f,  0.0f, -1.0f }}, .signed_distance = 0.5f },
	{ .normal = {{ -1.0f,  0.0f,  0.0f }}, .signed_distance = 0.5f },
	{ .normal = {{  0.0f,  0.0f,  1.0f }}, .signed_distance = 0.5f },
	{ .normal = {{  0.0f, -1.0f,  0.0f }}, .signed_distance = 0.5f },
};

struct dcel DcelBoxStub(void)
{
	struct dcel box = 
	{
		.v = box_stub_vertex,
		.bbox = { .min = {{ -0.5f, -0.5f, -0.5f }}, .max = {{ 0.5f, 0.5f, 0.5f }} },
		.e = box_edge,
		.f = box_face,
		.plane = box_stub_plane,
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
		.plane = NULL,
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
		.plane = ArenaPush(mem, 6*sizeof(struct plane)),
		.e = box_edge,
		.f = box_face,
		.e_count = 24,
		.v_count = 8,
		.f_count = 6,
	};

	if (!DcelPlanesTryCompute(&box))
	{
		Log(T_PHYSICS, S_WARNING, "DcelBox: degenerate box (half widths %f, %f, %f), using the box stub", hw.x, hw.y, hw.z);
		ds_AssertString(0, "DcelBox: degenerate box");
		box = DcelBoxStub();
	}
	else
	{
		DcelBboxCompute(&box);
	}

	return box; 
}

void DcelBboxCompute(struct dcel *h)
{
	h->bbox.min = V3(F32_INFINITY, F32_INFINITY, F32_INFINITY);
	h->bbox.max = V3(-F32_INFINITY, -F32_INFINITY, -F32_INFINITY);
	for (u32 i = 0; i < h->v_count; ++i)
	{
		h->bbox.min = V3Min(h->bbox.min, h->v[i]);
		h->bbox.max = V3Max(h->bbox.max, h->v[i]);
	}
}

u32 DcelPlanesTryCompute(struct dcel *h)
{
	u32 valid = 1;
	for (u32 fi = 0; fi < h->f_count; ++fi)
	{
		/*
		 * Newell's method. Every edge (v1, v2) of the face adds the doubled signed area of its shadow on the
		 * yz, zx and xy planes (trapezoid rule); summed over the closed polygon these are the components of
		 * twice the area vector, area * unit normal, pointing to the side from which the face is CCW. For a
		 * triangle it equals the cross product (v1 - v0) x (v2 - v0). Coordinates are taken relative to the
		 * face's first vertex o to keep the products small (less rounding).
		 */
		const struct dcelFace *f = h->f + fi;
		const v3 o = h->v[h->e[f->first].origin];
		v3 n = V3Zero();
		v3 sum = V3Zero();
		for (u32 i = 0; i < f->count; ++i)
		{
			const u32 i1 = h->e[f->first + i].origin;
			const u32 i2 = h->e[f->first + ((i + 1) % f->count)].origin;
			const v3 v1 = V3Sub(h->v[i1], o);
			const v3 v2 = V3Sub(h->v[i2], o);
			n.x += (v1.y - v2.y) * (v1.z + v2.z);
			n.y += (v1.z - v2.z) * (v1.x + v2.x);
			n.z += (v1.x - v2.x) * (v1.y + v2.y);
			sum = V3Add(sum, v1);
		}

		/* |n| = 2 * area; 0: the face has no area, so no normal (normalizing would give Inf/NaN) */
		const f32 length = V3Length(n);
		if (length > 0.0f)
		{
			/* the plane passes through the average of the face's vertices, spreading their deviation */
			const v3 centroid = V3Add(o, V3Scale(sum, 1.0f / (f32) f->count));
			h->plane[fi].normal = V3Scale(n, 1.0f / length);
			h->plane[fi].signed_distance = V3Dot(h->plane[fi].normal, centroid);
		}
		else
		{
			valid = 0;
		}
	}

	return valid;
}

u32 DcelSupportFace(const struct dcel *h, const v3 dir)
{
	u32 best = 0;
	f32 best_dot = V3Dot(h->plane[0].normal, dir);
	for (u32 fi = 1; fi < h->f_count; ++fi)
	{
		const f32 dot = V3Dot(h->plane[fi].normal, dir);
		if (best_dot < dot)
		{
			best = fi;
			best_dot = dot;
		}
	}

	return best;
}

/* Clip [t_min, t_max] of s to the outward side plane pl (keep signed distance <= 0); return 0 if s lies wholly in front */
static u32 DcelSegmentSideClip(f32 *t_min, f32 *t_max, const struct plane *pl, const struct segment *s)
{
	const f32 d0 = PlanePointSignedDistance(pl, s->p[0]);
	const f32 d1 = PlanePointSignedDistance(pl, s->p[1]);
	if (0.0f < d0 && 0.0f < d1)
	{
		return 0;
	}

	/* signs differ: d0 - d1 != 0 and t = d0 / (d0 - d1) is in [0, 1] */
	if (0.0f < d0)
	{
		*t_min = F32Max(*t_min, d0 / (d0 - d1));
	}
	else if (0.0f < d1)
	{
		*t_max = F32Min(*t_max, d0 / (d0 - d1));
	}

	return 1;
}

u32 DcelFaceSegmentClip(struct segment *clip, const struct dcel *h, const u32 fi, const struct segment *s)
{
	const v3 f_n = h->plane[fi].normal;

	f32 t_min = 0.0f;
	f32 t_max = 1.0f;

	struct dcelFace *f = h->f + fi;
	for (u32 i = 0; i < f->count; ++i)
	{
		const v3 p0 = h->v[h->e[f->first + i].origin];
		const v3 p1 = h->v[h->e[f->first + ((i + 1) % f->count)].origin];
		const v3 n = V3Cross(V3Sub(p1, p0), f_n);
		const struct plane clip_plane = PlaneConstruct(V3Scale(n, 1.0f/V3Length(n)), p0);
		if (!DcelSegmentSideClip(&t_min, &t_max, &clip_plane, s))
		{
			return 0;
		}
	}

	if (t_max < t_min)
	{
		return 0;
	}

	*clip = SegmentConstruct(SegmentBc(s, t_min), SegmentBc(s, t_max));
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
	struct DLL	ce_list;
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
    struct DLLNode   face_edge;
    struct DLLNode   vertex_edge;
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
	struct DLL	ce_list;
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
	DLLFlush(face->ce_list);
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
	u32 *edge_check = ArenaPushZero(tmp, ddcel->edge_pool.count_max * sizeof(u32));
	u32 *face_check = ArenaPushZero(tmp, 3*ddcel->v_count * sizeof(u32));

	for (u32 i = 0; i < ddcel->edge_pool.count_max; ++i)
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
				DLLAppend(cv->ce_list, ddcel->ce_pool.buf, slot.index, vertex_edge);
				DLLAppend(ddcel->f[f_i].ce_list, ddcel->ce_pool.buf, slot.index, face_edge);

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
						DLLAppend(f->ce_list, ddcel->ce_pool.buf, slot.index, face_edge);
						DLLAppend(ddcel->cv[ce->vertex].ce_list, ddcel->ce_pool.buf, slot.index, vertex_edge);

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
						DLLAppend(f->ce_list, ddcel->ce_pool.buf, slot.index, face_edge);
						DLLAppend(ddcel->cv[ce->vertex].ce_list, ddcel->ce_pool.buf, slot.index, vertex_edge);

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
			DLLRemove(cvj->ce_list, ddcel->ce_pool.buf, cei, vertex_edge);
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
		.plane = ArenaPush(mem, ddcel->face_pool.count*sizeof(struct plane)),
		.v_count = ddcel->v_count,
		.e_count = ddcel->edge_pool.count,
		.f_count = ddcel->face_pool.count,
	};


	if (cpy.v && cpy.e && cpy.f && cpy.plane)
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
		DLLFlush(ddcel.cv[i].ce_list);
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

	ArenaPushRecord(mem);
	dcel = DcelDdcel(mem, &ddcel);	
	if (dcel.f_count && !DcelPlanesTryCompute(&dcel))
	{
		Log(T_PHYSICS, S_WARNING, "DcelConvexHull: hull of %u points has a degenerate face, using the box stub", v_count);
		ds_AssertString(0, "DcelConvexHull: degenerate face");
		ArenaPopRecord(mem);
		dcel = DcelBoxStub();
	}
	else
	{
		DcelBboxCompute(&dcel);
		ArenaRemoveRecord(mem);
	}
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

