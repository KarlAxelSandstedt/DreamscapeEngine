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
 * that boxes built from points, and unions, are exact (no rounding from a center/half width round trip).
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

/**
 * segment: geometrical primitive; directional two point segment
 * p0: segment start
 * p1: segment end 
 * dir: non-normalized direction vector
 */
struct segment
{
	v3 p[2];	
	v3 dir;	/* p[1]-p[0] */
};

/* sphere: geometrical primitive, centered at its frame's origin */
struct sphere
{
	f32 radius;
};

/**
 * capsule: geometrical primitive
 */
struct capsule
{
	f32 half_height;	/* capsule extends from [0, -half_height, 0] x [0, half_height, 0] */
	f32 radius;	
};

/*********************************** ray ************************************/

/* return constructed ray */
struct ray 	RayConstruct(const v3 origin, const v3 dir);

/********************************* segment **********************************/

/* construct segment */
struct segment 	SegmentConstruct(const v3 p0, const v3 p1);
/* return squared distance between s1 and s2; set c1, c2 to closest point on s1, s2 respectively  */
f32 		    SegmentDistanceSquared(v3 *c1, v3 *c2, const struct segment *s1, const struct segment *s2);
/* return parameters t1,t2 of closest points c1,c2 on s1,s2 such that ci = si.p0(1-ti) + s1.p1*ti  */
void		    SegmentClosestParameter(f32 *t1, f32 *t2, const struct segment *s1, const struct segment *s2);
/*
 * Closest points of the lines through s1 and s2, s1.p0 + t1 s1.dir and s2.p0 + t2 s2.dir: return 1 and t1, t2 if both
 * lie in [0, 1] (the lines' closest points lie within the segments), else 0. Precondition: the segments are not
 * parallel, |s1.dir x s2.dir|^2 > 0 (callers' parallel tests guarantee it; asserted in debug).
 */
u32             SegmentLineClosestParameterCheck(f32 *t1, f32 *t2, const struct segment *s1, const struct segment *s2);
/* return squared distance between s and p; set c to the closest point on s to p */
f32 		    SegmentPointDistanceSquared(v3 *c, const struct segment *s, const v3 p);
/* Return parameter t of projected barycentric point p to segment s: PROJECTION_ON_LINE(p) = s.p0*(1-t) + s.p1*t */
f32		        SegmentPointProjectedBcParameter(const struct segment *s, const v3 p);
/* Return parameter g of closest barycentric point p to segment s: PROJECTION_ON_SEGMENT(p) = s.p0*(1-t) + s.p1*t, 0.0f <= t <= 1.0f */
f32 		    SegmentPointClosestBcParameter(const struct segment *s, const v3 p);
/* set bc_p = s.p0*(1-t) + s.p1*t */
v3 		    SegmentBc(const struct segment *s, const f32 t); 	


/* Return the segment resulting from transforming the given capsule. */
struct segment  SegmentCapsuleTransform(const struct capsule *cap, const ds_Transform *t);

/* Return the bounding box of the segment  */
struct aabb     BboxSegment(const struct segment *s);

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

/* Return smallest AABB that contains both a and b  */
void		AabbUnion(struct aabb *box_union, const struct aabb *a, const struct aabb *b);
/* Return AABB of rotated AABB. */
void		AabbRotate(struct aabb *dst, const struct aabb *src, const m3 rotation);
/* dst = local translated by origin (f64), rounded outward to f32: dst contains the f64 box at any distance */
void		AabbTranslateOutward(struct aabb *dst, const struct aabb *local, const v3f64 origin);
/* Return 1 if a and b intersect, 0 otherwise  */
u32 		AabbTest(const struct aabb *a, const struct aabb *b);
/* Return 1 if a fully contains b, 0 otherwise  */
u32 		AabbContains(const struct aabb *a, const struct aabb *b);
/* Return 1 if a (extended with given margin) fully contains b, 0 otherwise  */
u32 		AabbContainsMargin(const struct aabb *a, const struct aabb *b, const f32 margin);
/* sets up vertex buffer to use with glDrawArrays. Returns number of bytes written. */
u64 		AabbPushLinesBuffered(u8 *buf, const u64 bufsize, const struct aabb *box, const v4 color);
/* sets up vertex buffer to use with glDrawArrays. Returns number of bytes written. */
u64 		AabbTransformPushLinesBuffered(u8 *buf, const u64 bufsize, const struct aabb *box, const v3 translation, const m3 rotation, const v4 color);
/* Return the smallest index of the aabb with the maximum side length */
u32         AabbMaxAxis(const struct aabb a);

/* Return bounding box of a and b  */
struct aabb	BboxUnion(const struct aabb a, const struct aabb b);
/* Return bounding box of aabb a and point p  */
struct aabb	BboxPointUnion(const struct aabb a, const v3 p);
/* Return bounding box of triangle */
struct aabb	BboxTriangle(const v3 p0, const v3 p1, const v3 p2);
/* Return bounding box of the vertex set */
struct aabb	BboxVertexSet(const v3 *p, const u32 count);

/********************************* tri_mesh **********************************/

/*
 * triangle mesh (CCW) - set of ungrouped triangles.
 */
struct triMesh
{
	v3 *		v;
	v3u32 *	tri;
	u32 		v_count;	
	u32 		tri_count;
};

/* return bounding box of triMesh */
struct aabb	TriMeshBbox(const struct triMesh *mesh);

/* Get normal of ccw triangle */
v3 		TriCcwNormal(const v3 p0, const v3 p1, const v3 p2);
/* Get normal direction of ccw triangle */
v3 		TriCcwNormalDirection(const v3 p0, const v3 p1, const v3 p2);


/*
 * Return the feature of triangle (t0, t1, t2) closest to segment (s0, s1), its barycentric weights (as TriVoronoi)
 * and *s, the segment's parameter: the closest points are sum w[i] t_i and s0 + *s (s1 - s0), *s in [0, 1].
 * A segment piercing the triangle gives VORONOI_F012 at the crossing (distance 0). Any winding.
 *
 * Degenerate input: a segment in the triangle's plane, a zero-length segment, or a collinear triangle is handled
 * by the end point and edge candidates. Returns VORONOI_INVALID only for NaN input or distances beyond f32 (~1e19).
 */
enum voronoi    TriSegmentVoronoi(f32 w[3], f32 *s, const v3 t0, const v3 t1, const v3 t2, const v3 s0, const v3 s1);

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

/********************************* vertex operations ***********************************/

v3 	VertexCentroid(const v3 *vs, const u32 n);

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
	const struct plane pl = PlaneConstruct(V3Cross(V3Sub(b, a), V3Sub(c, a)), a);
	ds_Assert(V3Dot(pl.normal_direction, pl.normal_direction) > 0.0f);
	return pl;
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

#ifdef __cplusplus
} 
#endif

#endif
