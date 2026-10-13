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
    union
    {
	    f32 signed_distance;
        f32 signed_normal_unit_distance;
    };
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
/* Return 1 if the end-points of s are within a distance of sqrt(min_dist_sq) of each other, otherwise return 0. */
u32             SegmentPointCheck(const struct segment *s, const f32 min_dist_sq);
/* Return 1 if s1 and s2 are parallel, otherwise return 0. */
u32             SegmentParallelCheck(const struct segment *s1, const struct segment *s2, const f32 eps);
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
struct plane 	PlaneConstruct(const v3 n, const v3 p); 
/* Construct normalized plane with given normal direction n containing point p */
struct plane 	PlaneConstructNormalized(const v3 n, const v3 p); 
/* Construct plane from CCW triangle abc */
struct plane 	PlaneConstructFromCcwTriangle(const v3 a, const v3 b, const v3 c);
/* Construct normalized plane from CCW triangle abc */
struct plane 	PlaneConstructNormalizedFromCcwTriangle(const v3 a, const v3 b, const v3 c);
/* Normalize the plane's normal direction and rescale signed_distance accordingly */
void            PlaneNormalize(struct plane *pl);
/*
 * Return pl moved by the rigid transform x -> rot x + pos (rot a rotation matrix). Keeps |n|, so unit planes stay
 * unit. Not for scaling or shearing maps: their normals transform by the inverse transpose.
 */
struct plane    PlaneTransform(const struct plane *pl, const m3 rot, const v3 pos);
/* Return 1 if p is infront of plane, i.e. a positive signed distance, otherwise 0 */
u32 		    PlanePointInfrontCheck(const struct plane *pl, const v3 p);
/* Return 1 if p is behind plane, i.e. a negative signed distance, otherwise 0 */
u32 		    PlanePointBehindCheck(const struct plane *pl, const v3 p);
/* Return 1 if segment is parallel to plane, otherwise return 0. */
u32             PlaneSegmentParallelCheck(const struct plane *pl, const struct segment *s);
/* 
 * return t: s.p0 + t*s.dir is point on plane 
 *
 * WARNING: if the segment lies on the plane, NaN is returned.
 */
f32 		    PlaneSegmentClipParameter(const struct plane *pl, const struct segment *s);
/* 
 * return 1 if clip happened, otherwise 0. If 1, return valid clip point 
 *
 * WARNING: if the segment lies on the plane, 0 is returned and clip is set to NaN.
 */
u32 		    PlaneSegmentClip(v3 *clip, const struct plane *pl, const struct segment *s);
/* 
 * return 1 if clip happened, otherwise 0 
 *
 * WARNING: if the segment lies on the plane, 0 is returned.
 */
u32 		    PlaneSegmentTest(const struct plane *pl, const struct segment *s); 
/* return signed distance multiplied by |normal_direction| between plane and point (infront of plane == positive) */
f32 		    PlanePointSignedDistance(const struct plane *pl, const v3 p);
/* return absolute distance multiplied by |normal_direction| between plane and point */
f32 		    PlanePointDistance(const struct plane *pl, const v3 p);
/*
 * Set proj to the projection of p onto pl and return t = PlanePointSignedDistance / |n|^2, the signed distance
 * measured in multiples of n = pl->normal_direction: proj = p - t n. For a unit normal t is the signed distance.
 */
f32 		    PlanePointProjection(v3 *proj, const struct plane *pl, const v3 p);

/********************************** AABB ************************************/

/* Return smallest AABB that contains both a and b  */
void		AabbUnion(struct aabb *box_union, const struct aabb *a, const struct aabb *b);
/* Return AABB of rotated AABB. */
void		AabbRotate(struct aabb *dst, const struct aabb *src, const m3 rotation);
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

/*
TriVoronoi
==========
Shared data for triangle voronoi calculation 
*/

/* WARNING: Do not change ordering! */
enum TriVoronoiRegion
{
    TRI_VORONOI_VERTEX0,
    TRI_VORONOI_VERTEX1,
    TRI_VORONOI_VERTEX2,
    TRI_VORONOI_EDGE01,
    TRI_VORONOI_EDGE12,
    TRI_VORONOI_EDGE20,
    TRI_VORONOI_FACE,
    TRI_VORONOI_COUNT
};

extern const char *g_table_tri_voronoi_region_string[TRI_VORONOI_COUNT];

struct TriVoronoiTMP
{
    struct segment  s[3];           /* edge segment */
    struct plane    edge_plane[3];  /* edge plane orthogonal to face plane */
    struct plane    face_plane;     /* triangle plane (CCW) */
    v3            t[3];
};


/* Get normal of ccw triangle */
v3 		TriCcwNormal(const v3 p0, const v3 p1, const v3 p2);
/* Get normal direction of ccw triangle */
v3 		TriCcwNormalDirection(const v3 p0, const v3 p1, const v3 p2);


/* Setup a TriVoronoi struct corresponding to the CCW triangle t and return true if t is robust, false otherwise.  */
u32         TriVoronoiInitCcw(struct TriVoronoiTMP *tv, const v3 t[3]);

/*
 * Return the feature of triangle (t0, t1, t2) closest to segment (s0, s1), its barycentric weights (as TriVoronoi)
 * and *s, the segment's parameter: the closest points are sum w[i] t_i and s0 + *s (s1 - s0), *s in [0, 1].
 * A segment piercing the triangle gives VORONOI_F012 at the crossing (distance 0). Any winding.
 *
 * Degenerate input: a segment in the triangle's plane, a zero-length segment, or a collinear triangle is handled
 * by the end point and edge candidates. Returns VORONOI_INVALID only for NaN input or distances beyond f32 (~1e19).
 */
enum voronoi    TriSegmentVoronoi(f32 w[3], f32 *s, const v3 t0, const v3 t1, const v3 t2, const v3 s0, const v3 s1);

/* 
 * Return squared distance from segment s to triangle t, and set c_s to be the closest point on s, and c_t to be 
 * the closest point on the triangle.
 *
 * NOTE: If the returned distance is 0.0f, c_t is not necessarily c_s, but instead c_t ~= c_s. Use one of the points
 * for consistency if needed.
 */
f32 		TriCcwSegmentDistanceSquared(v3 *c_t, v3 *c_s, enum TriVoronoiRegion *region, const struct segment *s, const struct TriVoronoiTMP *tv);

/* 
 * Return squared distance from point p to triangle t, and set c to be the closest point on the triangle. 
 * lambda_count is set to indicate the number of non-zero lambda components, and lambda is set to the 
 * barocentric coordinates:
 */
f32         TriCcwPointDistanceSquared(v3 *c, enum TriVoronoiRegion *region, const v3 point, const struct TriVoronoiTMP *tv);

/* 
 * Return t in [0,1] such that clip = s.p0*(1-t) + s.p1*t is a point on the given plane. If no such t exist, 
 * return F32_INFINITY. 
 */
f32         TriCcwSegmentClipParameter(v3 *clip, const struct segment *s, const struct TriVoronoiTMP *tv);

/* 
 * Return 1 if segment clips triangle, 0 otherwise. If clip, set the clip point.
 */
u32         TriCcwSegmentClip(v3 *clip, const struct segment *s, const struct TriVoronoiTMP *tv);

/* 
 * Return the remaining segment when clipping s against all side-planes of the triangle. WARNING: Assumes s in
 * at least partially within the voronoi face region.
 */
struct segment  TriCcwSegmentSideClip(const struct segment *s, const struct TriVoronoiTMP *tv);


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
 * vector over all of its edges is twice the face's area times its unit normal (Box3D b3NewellPlane), and the
 * plane goes through the average of the face's vertices. Using every vertex keeps the normal accurate for
 * polygons whose first triangle is thin. Return 1 if every face has a nonzero area, otherwise 0 (the
 * planes of degenerate faces are then garbage).
 */
u32             DcelPlanesTryCompute(struct dcel *h);
/* Compute h->bbox: the bounding box of h->v (DcelConvexHull and DcelBox call it; stubs: see DcelTriStub) */
void            DcelBboxCompute(struct dcel *h);

/* Return face fi's plane transformed by p -> rot p + pos (local planes: h->plane[fi]) */
struct plane 	DcelFacePlaneWorld(const struct dcel *h, const m3 rot, const v3 pos, const u32 fi);
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
/* DcelFaceSegmentClip with s in the frame where the hull is moved by p -> rot p + pos (rot a rotation) */
u32         	DcelFaceSegmentClipWorld(struct segment *clip, const struct dcel *h, const m3 rot, const v3 pos, const u32 fi, const struct segment *s);

v3 		DcelEdgeNormal(const struct dcel *h, const u32 ei);
v3 		DcelEdgeDirection(const struct dcel *h, const u32 ei);
struct segment 	DcelEdgeSegment(const struct dcel *h, const m3 rot, const v3 pos, const u32 ei);

void 		DcelAssertTopology(struct dcel *dcel);

#ifdef DS_DEBUG
#define COLLISION_HULL_ASSERT(dcel)	DcelAssertTopology(dcel)
#else
#define COLLISION_HULL_ASSERT(dcel)	
#endif

/********************************* vertex operations ***********************************/

v3 	VertexCentroid(const v3 *vs, const u32 n);

#ifdef __cplusplus
} 
#endif

#endif
