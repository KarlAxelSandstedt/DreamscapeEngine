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

#ifndef __DS_GEOMETRY_H__
#define __DS_GEOMETRY_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"
#include "ds_math.h"
#include "ds_voronoi.h"

/****************** GEOMETRIC PRIMITIVES ******************/

/*
 * aabb: geometrical primitive; axis aligned bounding box, the points p with min <= p <= max. Stored as bounds so
 * that boxes built from points, and unions, are exact (no rounding from a center/half width round trip). The
 * functions here take and return boxes with min <= max on every axis.
 */
struct aabb 
{
	v3 min;
	v3 max;
};

/*
 * plane: the points x with dot(n, x) = signed_distance, where n is the stored normal (16 B).
 *
 * normal / normal_direction: the same storage under two names; code reads .normal when the plane is known to
 * be normalized (|n| = 1) and .normal_direction when it may have any length > 0. PlaneConstruct* keep the
 * given direction's length, the *Normalized constructors and PlaneNormalize make it 1.
 *
 * signed_distance: dot(n, p) for any point p on the plane. For a unit normal it is the plane's signed
 * distance from the origin (signed_distance * normal is the point of the plane closest to the origin); for a
 * direction of any length it is that distance times |n|.
 *
 * Point distances follow the same rule: PlanePointSignedDistance = dot(n, x) - signed_distance is the true
 * signed distance times |n| (exact distance only for a unit normal). Functions that need 1/|n|^2 compute it
 * (PlanePointProjection).
 */
struct plane 
{
    union
    {
	    v3 normal;
	    v3 normal_direction;
    };
    f32 signed_distance;
};

/*
 * ray: geometrical primitive. dir is not normalized; a ray parameter t names the point origin + t*dir, so t is
 * measured in units of |dir|.
 */
struct ray
{
	v3 origin;
	v3 dir;
};

/* ray in world space: f64 origin (precise far from the origin), f32 direction */
struct rayWorld
{
	v3f64   origin;
	v3      dir;
};

/*
 * segment: geometrical primitive; the directed segment p[0] -> p[1]. dir = p[1] - p[0] (not normalized) must hold:
 * build segments with SegmentConstruct (or copy one), never by writing the fields one by one.
 */
struct segment
{
	v3 p[2];
	v3 dir;
};

/* sphere: geometrical primitive, centered at its frame's origin; radius > 0 */
struct sphere
{
	f32 radius;
};

/*
 * capsule: geometrical primitive; the points within radius of the segment from (0, -half_height, 0) to
 * (0, half_height, 0) in its frame. half_height > 0, radius > 0.
 */
struct capsule
{
	f32 half_height;
	f32 radius;
};

/*********************************** ray ************************************/

/* Return the ray origin + t dir. Precondition: |dir|^2 > 0 (asserted) */
static ds_ForceInline struct ray Ray(const v3 origin, const v3 dir);

/********************************* segment **********************************/

/*
 * A segment's parameter t names the point p0 (1 - t) + p1 t (SegmentBc). Point segments (p0 == p1) are valid
 * input to every function here, unless a precondition says otherwise.
 */

/* Return the segment p0 -> p1 (dir = p1 - p0) */
static ds_ForceInline struct segment SegmentConstruct(const v3 p0, const v3 p1);
/* Return p0 (1 - t) + p1 t; exactly p0 at t = 0 and p1 at t = 1 */
static ds_ForceInline v3 SegmentBc(const struct segment *s, const f32 t);
/* Return the capsule's core segment, transformed by t: from (0, half_height, 0) to (0, -half_height, 0) */
static ds_ForceInline struct segment SegmentCapsuleTransform(const struct capsule *cap, const ds_Transform *t);
/* Set t1, t2 in [0, 1] to the parameters of a closest pair of points of s1 and s2 */
void		    SegmentClosestParameter(f32 *t1, f32 *t2, const struct segment *s1, const struct segment *s2);
/* Return the squared distance between s1 and s2; set c1, c2 to a closest pair of points on s1, s2 */
static ds_ForceInline f32 SegmentDistanceSquared(v3 *c1, v3 *c2, const struct segment *s1, const struct segment *s2);
/*
 * Closest points of the lines through s1 and s2, s1.p0 + t1 s1.dir and s2.p0 + t2 s2.dir: return 1 and t1, t2 if both
 * lie in [0, 1] (the lines' closest points lie within the segments), else 0. Precondition: the segments are not
 * parallel, |s1.dir x s2.dir|^2 > 0 (callers' parallel tests guarantee it; asserted in debug).
 */
static ds_ForceInline u32 SegmentLineClosestParameterCheck(f32 *t1, f32 *t2, const struct segment *s1, const struct segment *s2);
/* Return the parameter t in [0, 1] of the point of s closest to p (a point segment: 1, the same point as 0) */
static ds_ForceInline f32 SegmentPointClosestBcParameter(const struct segment *s, const v3 p);
/* Return the squared distance between s and p; set c to the point of s closest to p */
static ds_ForceInline f32 SegmentPointDistanceSquared(v3 *c, const struct segment *s, const v3 p);
/* Return the parameter t of p projected onto the line through s (any real t). Precondition: |s.dir|^2 > 0 (asserted) */
static ds_ForceInline f32 SegmentPointProjectedBcParameter(const struct segment *s, const v3 p);

/********************************** plane ***********************************/

/* Construct plane with given normal direction n containing point p */
static ds_ForceInline struct plane PlaneConstruct(const v3 n, const v3 p);
/* Construct normalized plane with given normal direction n containing point p. Precondition: 0 < |n|^2 < inf (asserted) */
static ds_ForceInline struct plane PlaneConstructNormalized(const v3 n, const v3 p);
/* Construct plane from CCW triangle abc (normal direction (b - a) x (c - a)). Precondition: abc not degenerate (asserted) */
static ds_ForceInline struct plane PlaneConstructFromCcwTriangle(const v3 a, const v3 b, const v3 c);
/*
 * Normalize the plane's normal direction and rescale signed_distance by the same factor. Precondition:
 * 0 < |n|^2 < inf (asserted)
 */
static ds_ForceInline void PlaneNormalize(struct plane *pl);
/*
 * Return pl moved by the rigid transform x -> rot x + pos (rot a rotation matrix). Keeps |n|, so unit planes stay
 * unit. Not for scaling or shearing maps: their normals transform by the inverse transpose.
 */
static ds_ForceInline struct plane PlaneTransform(const struct plane *pl, const m3 rot, const v3 pos);
/* return signed distance multiplied by |normal_direction| between plane and point (infront of plane == positive) */
static ds_ForceInline f32 PlanePointSignedDistance(const struct plane *pl, const v3 p);
/* return absolute distance multiplied by |normal_direction| between plane and point */
static ds_ForceInline f32 PlanePointDistance(const struct plane *pl, const v3 p);
/* Return 1 if p is infront of plane, i.e. a positive signed distance, otherwise 0 */
static ds_ForceInline u32 PlanePointInfrontCheck(const struct plane *pl, const v3 p);
/* Return 1 if p is behind plane, i.e. a negative signed distance, otherwise 0 */
static ds_ForceInline u32 PlanePointBehindCheck(const struct plane *pl, const v3 p);
/*
 * Set proj to the projection of p onto pl and return t = PlanePointSignedDistance / |n|^2, the signed distance
 * measured in multiples of n = pl->normal_direction: proj = p - t n. For a unit normal t is the signed distance.
 * Precondition: 0 < |n|^2 < inf (asserted)
 */
static ds_ForceInline f32 PlanePointProjection(v3 *proj, const struct plane *pl, const v3 p);
/*
 * Return t such that s.p0 + t*s.dir lies on the plane: t = d0 / (d0 - d1), from the end points' signed distances.
 * End points strictly on opposite sides give t in [0, 1]. Parallel to the plane (d0 == d1): F32_INFINITY, or 0 if
 * the segment lies in the plane.
 */
static ds_ForceInline f32 PlaneSegmentClipParameter(const struct plane *pl, const struct segment *s);
/* Return 1 if the segment touches or crosses the plane (its end points are not strictly on one side), otherwise 0 */
static ds_ForceInline u32 PlaneSegmentTest(const struct plane *pl, const struct segment *s);

/********************************** AABB ************************************/

/*
 * Bbox* return a box by value; Aabb* test or translate boxes. Boxes are closed: they contain their boundary, and
 * boxes that touch intersect.
 */

/* Return the smallest box containing a and b */
static ds_ForceInline struct aabb BboxUnion(const struct aabb a, const struct aabb b);
/* Return the smallest box containing a and the point p */
static ds_ForceInline struct aabb BboxPointUnion(const struct aabb a, const v3 p);
/* Return the bounding box of the triangle p0 p1 p2 */
static ds_ForceInline struct aabb BboxTriangle(const v3 p0, const v3 p1, const v3 p2);
/* Return the bounding box of the segment */
static ds_ForceInline struct aabb BboxSegment(const struct segment *s);
/* Return the bounding box of p[0 .. count); count == 0: the zero box at the origin */
struct aabb	BboxVertexSet(const v3 *p, const u32 count);
/*
 * Return a box that contains R x for every x in box (R = rotation): conservative, it covers this computation's
 * rounding, so it bounds any shape bounded by box, rotated by R.
 */
static ds_ForceInline struct aabb BboxRotate(const struct aabb box, const m3 rotation);
/* dst = local translated by origin (f64), rounded outward to f32: dst contains the f64 box at any distance */
static ds_ForceInline void AabbTranslateOutward(struct aabb *dst, const struct aabb *local, const v3f64 origin);
/* Return 1 if a and b intersect (touching included), otherwise 0 */
static ds_ForceInline u32 AabbTest(const struct aabb *a, const struct aabb *b);
/* Return 1 if a contains b, otherwise 0 */
static ds_ForceInline u32 AabbContains(const struct aabb *a, const struct aabb *b);
/* Return 1 if a, extended by margin on every side, contains b, otherwise 0 */
static ds_ForceInline u32 AabbContainsMargin(const struct aabb *a, const struct aabb *b, const f32 margin);
/* Return the axis with the largest side length (ties: the lowest axis) */
static ds_ForceInline u32 AabbMaxAxis(const struct aabb a);

/********************************* tri_mesh **********************************/

/*
 * triangle mesh: tri_count triangles, each three indices into the v_count vertices v; no adjacency. Precondition:
 * every triangle has nonzero area (asserted by the mesh contacts). Collision is two-sided; the CCW winding only
 * orients render normals.
 */
struct triMesh
{
	v3 *		v;
	v3u32 *	tri;
	u32 		v_count;	
	u32 		tri_count;
};

/* Return the bounding box of all v_count vertices, unreferenced ones included; no vertices: the zero box */
static ds_ForceInline struct aabb TriMeshBbox(const struct triMesh *mesh);

/*
 * Return 1 and the normal direction of the CCW triangle p0 p1 p2 in *dir (|dir| = twice the area); 0 for a degenerate
 * triangle (collinear or coincident points, or |dir|^2 outside the f32 range), *dir = 0.
 */
static ds_ForceInline u32 TriCcwNormalDirectionTryCompute(v3 *dir, const v3 p0, const v3 p1, const v3 p2);
/* Return 1 and the unit normal of the CCW triangle p0 p1 p2 in *normal; 0 for a degenerate triangle, *normal = 0 */
static ds_ForceInline u32 TriCcwNormalTryCompute(v3 *normal, const v3 p0, const v3 p1, const v3 p2);

/********************************** dcel ************************************/

struct dcelFace
{
	u32 first;	/* first half edge */
	u32 count;	/* edge count */
};

struct dcelEdge
{
	u32 origin;	/* vertex index origin */
	u32 twin; 	/* twin half edge */
	u32 face_ccw; 	/* face to the left of half edge */
};

/*
 * (Computational Geometry Algorithms and Applications, Section 2.2) 
 * dcel - doubly-connected edge list. Can represent convex 3d bodies (with no holes in polygons)
 * 	  and 2d planar graphs. A polygon in the data structure are implicitly defined by its 
 * 	  first half edge. 
 */
struct dcel
{
	struct dcelFace *f;		/* f[i] = half-edge of face i */
	struct plane *  plane;	/* plane[i] = face i's plane: unit normal pointing out of the hull (DcelPlanesTryCompute) */
	struct aabb     bbox;   /* local bounding box of v (DcelBboxCompute) */
	struct dcelEdge *e;
	v3 *	v;
	u32 f_count;
	u32 e_count;
	u32 v_count;
};

/* return dcel { 0 } */
struct dcel 	DcelEmpty(void);
/*
 * Return the triangle stub: two faces (front 0-1-2, back 0-2-1) without vertices, planes or box. The caller sets
 * v (3 vertices), plane (2 planes, e.g. through DcelPlanesTryCompute) and bbox (DcelBboxCompute; flat).
 */
struct dcel     DcelTriStub(void);
/* return dcel box stub: unit cube centered at the origin, static memory (planes and box included) */
struct dcel 	DcelBoxStub(void);
/*
 * return arena allocated dcel box with given half widths. Degenerate (a half width of 0): asserts in debug;
 * in release logs a warning and returns DcelBoxStub().
 */
struct dcel 	DcelBox(struct arena *mem, const v3 hw);
/*
 * return arena allocated dcel convex hull of input points. On failure (fewer than 4 points, or no
 * tetrahedron within tol), an empty dcel is returned. A hull with a degenerate face (zero area) asserts in
 * debug; in release it is discarded with a warning and DcelBoxStub() is returned.
 */
struct dcel 	DcelConvexHull(struct arena *mem, const v3 *v, const u32 v_count, const f32 tol);

/*
 * Compute h->plane[0 .. f_count) (memory set by the caller) with Newell's method: per face, the summed
 * vector over all of its edges is twice the face's area times its unit normal, and the
 * plane goes through the average of the face's vertices. Using every vertex keeps the normal accurate for
 * polygons whose first triangle is thin. Return 1 if every face has a nonzero area, otherwise 0 (the
 * planes of degenerate faces are then garbage).
 */
u32             DcelPlanesTryCompute(struct dcel *h);
/* Compute h->bbox: the bounding box of h->v (DcelConvexHull and DcelBox call it; stubs: see DcelTriStub) */
void            DcelBboxCompute(struct dcel *h);

/*
 * Return the index of the face whose outward unit normal has the largest dot product with dir (the face most
 * facing dir). dir need not be unit. Ties go to the lowest index; dir == 0 returns 0.
 */
u32             DcelSupportFace(const struct dcel *h, const v3 dir);

/*
 * Clip s (in the hull's frame) to the side planes of face fi (planes through its edges, orthogonal to the face).
 * Return 1 and the clipped part in clip (not projected onto the face), or 0 if nothing of s lies within the
 * face's sides (clip unwritten). A degenerate s (p[0] == p[1]) gives 0 or itself.
 */
u32         	DcelFaceSegmentClip(struct segment *clip, const struct dcel *h, const u32 fi, const struct segment *s);

v3 		DcelEdgeNormal(const struct dcel *h, const u32 ei);
v3 		DcelEdgeDirection(const struct dcel *h, const u32 ei);

void 		DcelAssertTopology(struct dcel *dcel);

#ifdef DS_DEBUG
#define COLLISION_HULL_ASSERT(dcel)	DcelAssertTopology(dcel)
#else
#define COLLISION_HULL_ASSERT(dcel)	
#endif

/****************************** ray: inline definitions *******************************/

static ds_ForceInline struct ray Ray(const v3 origin, const v3 dir)
{
	ds_Assert(V3Dot(dir, dir) > 0.0f);
	const struct ray r = { .origin = origin, .dir = dir };
	return r;
}

/****************************** segment: inline definitions *******************************/

static ds_ForceInline struct segment SegmentConstruct(const v3 p0, const v3 p1)
{
	const struct segment s = { .p = { p0, p1 }, .dir = V3Sub(p1, p0) };
	return s;
}

static ds_ForceInline v3 SegmentBc(const struct segment *s, const f32 t)
{
	return V3Interpolate(s->p[1], s->p[0], t);
}

static ds_ForceInline struct segment SegmentCapsuleTransform(const struct capsule *cap, const ds_Transform *t)
{
	const v3 p = QUnitRotate(t->rotation, V3(0.0f, cap->half_height, 0.0f));
	return SegmentConstruct(V3Add(p, t->position), V3Add(V3Negate(p), t->position));
}

static ds_ForceInline f32 SegmentDistanceSquared(v3 *c1, v3 *c2, const struct segment *s1, const struct segment *s2)
{
	f32 t1, t2;
	SegmentClosestParameter(&t1, &t2, s1, s2);
	*c1 = SegmentBc(s1, t1);
	*c2 = SegmentBc(s2, t2);
	return V3DistanceSquared(*c1, *c2);
}

static ds_ForceInline u32 SegmentLineClosestParameterCheck(f32 *t1, f32 *t2, const struct segment *s1, const struct segment *s2)
{
	/*
	 * Lines s1.p0 + t1 d1, s2.p0 + t2 d2, r = s2.p0 - s1.p0, n = d1 x d2: the closest pair has
	 *
	 *	t1 = (r x d2) . n / |n|^2,  t2 = (r x d1) . n / |n|^2
	 *
	 * (Lagrange's identity turns the dot product forms into these); the cross products keep the relative error
	 * near eps / sin(angle) where the dot forms cancel to eps / sin^2(angle).
	 */
	const v3 r = V3Sub(s2->p[0], s1->p[0]);
	const v3 n = V3Cross(s1->dir, s2->dir);
	const f32 den = V3Dot(n, n);
	ds_Assert(den > 0.0f);

	*t1 = V3Dot(V3Cross(r, s2->dir), n) / den;
	*t2 = V3Dot(V3Cross(r, s1->dir), n) / den;
	return (0.0f <= *t1 && *t1 <= 1.0f && 0.0f <= *t2 && *t2 <= 1.0f);
}

static ds_ForceInline f32 SegmentPointClosestBcParameter(const struct segment *s, const v3 p)
{
	/* the weight of p1; a point segment gives VORONOI_V1 (t = 1, the same point); VORONOI_INVALID (rounding): p0 */
	f32 w[2];
	return (SegmentVoronoi(w, s->p[0], s->p[1], p) == VORONOI_INVALID)
		? 0.0f
		: w[1];
}

static ds_ForceInline f32 SegmentPointDistanceSquared(v3 *c, const struct segment *s, const v3 p)
{
	*c = SegmentBc(s, SegmentPointClosestBcParameter(s, p));
	return V3DistanceSquared(*c, p);
}

static ds_ForceInline f32 SegmentPointProjectedBcParameter(const struct segment *s, const v3 p)
{
	f32 bc[2], divisor;
	SegmentBCUnnormalized(bc, &divisor, s->p[0], s->p[1], p);
	ds_Assert(divisor > 0.0f);
	return bc[1] / divisor;
}

/****************************** plane: inline definitions *******************************/

static ds_ForceInline struct plane PlaneConstruct(const v3 n, const v3 p)
{
	struct plane pl;
	pl.normal_direction = n;
	pl.signed_distance = V3Dot(n, p);
	return pl;
}

static ds_ForceInline struct plane PlaneConstructNormalized(const v3 n, const v3 p)
{
	const f32 len_sq = V3Dot(n, n);
	ds_Assert(0.0f < len_sq && len_sq < F32_INFINITY);
	struct plane pl;
	pl.normal = V3Scale(n, 1.0f / F32Sqrt(len_sq));
	pl.signed_distance = V3Dot(pl.normal, p);
	return pl;
}

static ds_ForceInline struct plane PlaneConstructFromCcwTriangle(const v3 a, const v3 b, const v3 c)
{
	v3 n;
	const u32 valid = TriCcwNormalDirectionTryCompute(&n, a, b, c);
	ds_Assert(valid);
	return PlaneConstruct(n, a);
}

static ds_ForceInline void PlaneNormalize(struct plane *pl)
{
	const f32 len_sq = V3Dot(pl->normal_direction, pl->normal_direction);
	ds_Assert(0.0f < len_sq && len_sq < F32_INFINITY);
	const f32 inv_len = 1.0f / F32Sqrt(len_sq);
	pl->normal = V3Scale(pl->normal_direction, inv_len);
	pl->signed_distance *= inv_len;
}

static ds_ForceInline struct plane PlaneTransform(const struct plane *pl, const m3 rot, const v3 pos)
{
	/*
	 * A point x on pl (dot(n, x) = d) moves to rot x + pos. With n' = rot n, rotations keep dot products:
	 *      dot(n', rot x + pos) = dot(n, x) + dot(n', pos) = d + dot(n', pos)
	 */
	struct plane moved;
	moved.normal_direction = M3V3Mul(rot, pl->normal_direction);
	moved.signed_distance = pl->signed_distance + V3Dot(moved.normal_direction, pos);
	return moved;
}

static ds_ForceInline f32 PlanePointSignedDistance(const struct plane *pl, const v3 p)
{
	return V3Dot(pl->normal_direction, p) - pl->signed_distance;
}

static ds_ForceInline f32 PlanePointDistance(const struct plane *pl, const v3 p)
{
	return F32Abs(PlanePointSignedDistance(pl, p));
}

static ds_ForceInline u32 PlanePointInfrontCheck(const struct plane *pl, const v3 p)
{
	return (PlanePointSignedDistance(pl, p) > 0.0f);
}

static ds_ForceInline u32 PlanePointBehindCheck(const struct plane *pl, const v3 p)
{
	return (PlanePointSignedDistance(pl, p) < 0.0f);
}

static ds_ForceInline f32 PlanePointProjection(v3 *proj, const struct plane *pl, const v3 p)
{
	const f32 len_sq = V3Dot(pl->normal_direction, pl->normal_direction);
	ds_Assert(0.0f < len_sq && len_sq < F32_INFINITY);
	const f32 n_units = PlanePointSignedDistance(pl, p) / len_sq;
	*proj = V3AddScaled(p, pl->normal_direction, -n_units);
	return n_units;
}

static ds_ForceInline f32 PlaneSegmentClipParameter(const struct plane *pl, const struct segment *s)
{
	const f32 d0 = PlanePointSignedDistance(pl, s->p[0]);
	const f32 d1 = PlanePointSignedDistance(pl, s->p[1]);
	if (d0 == d1)
	{
		return (d0 == 0.0f) ? 0.0f : F32_INFINITY;
	}
	return d0 / (d0 - d1);
}

static ds_ForceInline u32 PlaneSegmentTest(const struct plane *pl, const struct segment *s)
{
	const f32 d0 = PlanePointSignedDistance(pl, s->p[0]);
	const f32 d1 = PlanePointSignedDistance(pl, s->p[1]);
	return !((0.0f < d0 && 0.0f < d1) || (d0 < 0.0f && d1 < 0.0f));
}

/****************************** AABB: inline definitions *******************************/

static ds_ForceInline struct aabb BboxUnion(const struct aabb a, const struct aabb b)
{
	const struct aabb bbox = { .min = V3Min(a.min, b.min), .max = V3Max(a.max, b.max) };
	return bbox;
}

static ds_ForceInline struct aabb BboxPointUnion(const struct aabb a, const v3 p)
{
	const struct aabb bbox = { .min = V3Min(a.min, p), .max = V3Max(a.max, p) };
	return bbox;
}

static ds_ForceInline struct aabb BboxTriangle(const v3 p0, const v3 p1, const v3 p2)
{
	const struct aabb bbox = { .min = V3Min(V3Min(p0, p1), p2), .max = V3Max(V3Max(p0, p1), p2) };
	return bbox;
}

static ds_ForceInline struct aabb BboxSegment(const struct segment *s)
{
	const struct aabb bbox = { .min = V3Min(s->p[0], s->p[1]), .max = V3Max(s->p[0], s->p[1]) };
	return bbox;
}

static ds_ForceInline struct aabb BboxRotate(const struct aabb box, const m3 rotation)
{
	/*
	 * box = {c + h*s : s in [-1, 1]^3}; rotated, {R c + R (h*s)}: half widths |R| h around R c (|R| entrywise).
	 * With u = eps/2, the roundings of c, h and the 3-term dot products move each component by less than about
	 * 4u (|R| (|c| + h))_k; the slack is 16u of that, which also covers the last two roundings.
	 */
	const v3 x = V3Abs(V3(rotation.a11, rotation.a12, rotation.a13));
	const v3 y = V3Abs(V3(rotation.a21, rotation.a22, rotation.a23));
	const v3 z = V3Abs(V3(rotation.a31, rotation.a32, rotation.a33));

	const v3 c = V3Scale(V3Add(box.min, box.max), 0.5f);
	const v3 h = V3Scale(V3Sub(box.max, box.min), 0.5f);
	const v3 a = V3Add(V3Abs(c), h);
	const v3 hw = V3(V3Dot(x, h), V3Dot(y, h), V3Dot(z, h));
	const v3 slack = V3Scale(V3(V3Dot(x, a), V3Dot(y, a), V3Dot(z, a)), 8.0f * F32_EPSILON);
	const v3 r = V3Add(hw, slack);
	const v3 c_rotated = M3V3Mul(rotation, c);
	const struct aabb rotated = { .min = V3Sub(c_rotated, r), .max = V3Add(c_rotated, r) };
	return rotated;
}

static ds_ForceInline void AabbTranslateOutward(struct aabb *dst, const struct aabb *local, const v3f64 origin)
{
	dst->min = V3F64DemoteDownwards(V3F64Add(origin, V3Promote(local->min)));
	dst->max = V3F64DemoteUpwards(V3F64Add(origin, V3Promote(local->max)));
}

static ds_ForceInline u32 AabbTest(const struct aabb *a, const struct aabb *b)
{
	if (b->min.x > a->max.x || a->min.x > b->max.x) { return 0; }
	if (b->min.y > a->max.y || a->min.y > b->max.y) { return 0; }
	if (b->min.z > a->max.z || a->min.z > b->max.z) { return 0; }

	return 1;
}

static ds_ForceInline u32 AabbContains(const struct aabb *a, const struct aabb *b)
{
	if (b->min.x < a->min.x) { return 0; }
	if (b->min.y < a->min.y) { return 0; }
	if (b->min.z < a->min.z) { return 0; }

	if (b->max.x > a->max.x) { return 0; }
	if (b->max.y > a->max.y) { return 0; }
	if (b->max.z > a->max.z) { return 0; }

	return 1;
}

static ds_ForceInline u32 AabbContainsMargin(const struct aabb *a, const struct aabb *b, const f32 margin)
{
	if (b->min.x < a->min.x - margin) { return 0; }
	if (b->min.y < a->min.y - margin) { return 0; }
	if (b->min.z < a->min.z - margin) { return 0; }

	if (b->max.x > a->max.x + margin) { return 0; }
	if (b->max.y > a->max.y + margin) { return 0; }
	if (b->max.z > a->max.z + margin) { return 0; }

	return 1;
}

static ds_ForceInline u32 AabbMaxAxis(const struct aabb a)
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

/****************************** tri_mesh: inline definitions *******************************/

static ds_ForceInline struct aabb TriMeshBbox(const struct triMesh *mesh)
{
	return BboxVertexSet(mesh->v, mesh->v_count);
}

static ds_ForceInline u32 TriCcwNormalDirectionTryCompute(v3 *dir, const v3 p0, const v3 p1, const v3 p2)
{
	/*
	 * The cross product of the two shortest edges, at the vertex opposite the longest edge: the same vector in
	 * exact arithmetic at every vertex, the smallest rounding error in f32 (it scales with the two edges' lengths).
	 */
	const v3 e0 = V3Sub(p1, p0);
	const v3 e1 = V3Sub(p2, p1);
	const v3 e2 = V3Sub(p0, p2);
	const f32 l0 = V3Dot(e0, e0);
	const f32 l1 = V3Dot(e1, e1);
	const f32 l2 = V3Dot(e2, e2);
	v3 n;
	if (l1 <= l0 && l2 <= l0)
	{
		n = V3Cross(e2, V3Negate(e1));	/* p0p1 longest: (p0 - p2) x (p1 - p2) */
	}
	else if (l2 <= l1)
	{
		n = V3Cross(e0, V3Negate(e2));	/* p1p2 longest: (p1 - p0) x (p2 - p0) */
	}
	else
	{
		n = V3Cross(e1, V3Negate(e0));	/* p2p0 longest: (p2 - p1) x (p0 - p1) */
	}

	const f32 len_sq = V3Dot(n, n);
	ds_Assert(len_sq == len_sq);
	const u32 valid = (0.0f < len_sq && len_sq < F32_INFINITY);
	*dir = (valid) ? n : V3Zero();
	return valid;
}

static ds_ForceInline u32 TriCcwNormalTryCompute(v3 *normal, const v3 p0, const v3 p1, const v3 p2)
{
	v3 dir;
	if (!TriCcwNormalDirectionTryCompute(&dir, p0, p1, p2))
	{
		*normal = V3Zero();
		return 0;
	}

	*normal = V3Scale(dir, 1.0f / F32Sqrt(V3Dot(dir, dir)));
	return 1;
}

#ifdef __cplusplus
} 
#endif

#endif
