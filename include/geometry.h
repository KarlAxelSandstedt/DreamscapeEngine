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

/****************** GEOMETRIC PRIMITIVES ******************/

/**
 * AABB: geomtrical primitive; Axis aligned bounding box
 * center: center of the box
 * hw: half width of the box in each dimension
 */
struct aabb 
{
	v3 center;
	v3 hw;
};

/**
 * plane: geomtrical primitive
 * normal: normal of plane 
 * OR normal_dir: normal direction of plane (To make it explicit in code when we choose not to normalize the direction)
 * signed_distance: Signed distance (in |normal_direction| units) to plane; plane.normal * signed_distance is point on plane.
 *
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
    f32 inv_dot_nn; /* 1.0f / Dot(normal,normal) */
};

/**
 * ray: geomtrical primitive
 * origin: Origin of ray
 * dir.direction with length. 
 *
 * NOTE: When deriving parameters t using rays, it is always in the context origin + t * dir. For example,
 * 	If for a given ray we get a collision at a point p with parameter t, The follow will always hold:
 *
 * 	p = ray.origin + t * ray.dir.
 */
struct ray
{
	v3 origin;
	v3 dir;
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

/**
 * segment: geometrical primitive
 * p0: segment start
 * p1: segment end 
 * dir: non-normalized direction vector
 */
struct sphere
{
	v3 center;
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

/********************************** sphere **********************************/

/* constructed sphere */
struct sphere 	SphereConstruct(const v3 center, const f32 radius);
/* return t: smallest t >= 0 such that p = origin + t*dir is a point on the sphere, or F32_INF if no such t exist */
f32 		SphereRaycastParameter(const struct sphere *sph, const struct ray *ray);
/* Return 1 if raycast hit sphere, 0 otherwise. If hit, set intersection  */
u32 		SphereRaycast(v3 *intersection, const struct sphere *sph, const struct ray *ray);
/* Return support of sphere in given direction. sph->position is ignored here, so use pos as the real position */
v3		SphereSupport(const v3 dir, const struct sphere *sph, const v3 pos);

/*********************************** ray ************************************/

/* return constructed ray */
struct ray 	RayConstruct(const v3 origin, const v3 dir);
/* return segment: s.p0 = r.origin, s.p1 = r.origin + t * r.dir */
struct segment	RayConstructSegment(const struct ray *r, const f32 t);
/* return ray.origin + t * ray.dir */
v3		RayPoint(const struct ray *ray, const f32 t);
/* return t: closest point on ray to p = origin + t * dir */
f32 		RayPointClosestPointParameter(const struct ray *ray, const v3 p);
/* return squared distance from p to ray, and set RayPoint to the closest point on the ray */
f32 		RayPointDistanceSquared(v3 *r_c, const struct ray *ray, const v3 p);
/* return squared distance from s to ray, and set r_c and s_c to the closest points on the primitives */
f32 		RaySegmentDistanceSquared(v3 *r_c, v3 *s_c, const struct ray *ray, const struct segment *s);

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
/* Normalize the plane's normal direction (and update affected internals) */
void            PlaneNormalize(struct plane *pl);
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
/* return the signed distance (measured in |plane.normal_direction| units) of point p to plane pl, and set the projection of p onto pl. */
f32 		    PlanePointProjection(v3 *proj, const struct plane *pl, const v3 p);

/* Return t such that ray->origin + t*ray->dir is a point on the given plane. If no such t exist, return F32_INFINITY. */
f32 		PlaneRaycastParameter(const struct plane *plane, const struct ray *ray);
/* Return 1 if raycast hit plane, 0 otherwise. If hit, set intersection  */
u32 		PlaneRaycast(v3 *intersection, const struct plane *plane, const struct ray *ray);

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

/* Setup parameters for extended raycasting functions. */
void 		AabbRaycastParameterExSetup(v3 *multiplier, v3u32 *dir_sign_bit, const struct ray *ray);
/* Extended AabbRaycastParameter optimized for multiple raycasts against AABBs using same ray. 
 * return t: smallest t >= 0 such that p = origin + t*dir is a point in the AABB volume, or F32_INF if no such t exist */
f32 		AabbRaycastParameterEx(const struct aabb *aabb, const struct ray *ray, const v3 multiplier, const v3u32 dir_sign_bit);
/* return t: smallest t >= 0 such that p = origin + t*dir is a point in the AABB volume, or F32_INF if no such t exist */
f32 		AabbRaycastParameter(const struct aabb *a, const struct ray *ray);
/* Extended AabbRaycast, optimized for multiple raycasts against AABBs using same ray. 
 * If the ray hits aabb, return 1 and set intersection. otherwise return 0. */
u32 		AabbRaycastEx(v3 *intersection, const struct aabb *aabb, const struct ray *ray, const v3 multiplier, const v3u32 dir_sign_bit);
/* If the ray hits aabb, return 1 and set intersection. otherwise return 0. */
u32 		AabbRaycast(v3 *intersection, const struct aabb *aabb, const struct ray *ray);

/********************************* capsule **********************************/

/* Return support of capsule in given direction. */
v3		CapsuleSupport(const v3 dir, const struct capsule *cap, const m3 rot, const v3 pos);

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
/* return t: smallest t >= 0 such that p = origin + t*dir is a point on the triangle, or F32_INF if no such t exist */
f32 		TriMeshRaycastParameter(const struct triMesh *mesh, const u32 tri, const struct ray *ray);
/* If the ray hits triangle, return 1 and set intersection. otherwise return 0. */
u32 		TriMeshRaycast(v3 *intersection, const struct triMesh *mesh, const u32 tri, const struct ray *ray);

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
	struct dcelEdge *e;
	v3 *	v;
	u32 f_count;
	u32 e_count;
	u32 v_count;
};

/* return dcel { 0 } */
struct dcel 	DcelEmpty(void);
/* return dcel tri stub */
struct dcel     DcelTriStub(void);
/* return dcel box stub */
struct dcel 	DcelBoxStub(void);
/* return arena allocated dcel box with given half widths */
struct dcel 	DcelBox(struct arena *mem, const v3 hw);
/* return arena allocated dcel convex hull of input points. On failure, an empty dcel is returned. */
struct dcel 	DcelConvexHull(struct arena *mem, const v3 *v, const u32 v_count, const f32 tol);
/* Return support of dcel in given direction, and return supporting vertex index */
u32		        DcelSupport(v3 *support, const v3 dir, const struct dcel *hull, const m3 rot, const v3 pos);

/* Return the transformed plane defined by the given face */
struct plane 	DcelFacePlane(const struct dcel *h, const m3 rot, const v3 pos, const u32 fi);
/* Return the plane defined by the given face */
struct plane 	DcelFacePlaneLocal(const struct dcel *h, const u32 fi);

/* Return the transformed normal defined by the given face */
v3 		    DcelFaceNormal(const struct dcel *h, const m3 rot, const u32 fi);
/* Return the transformed normal drirection defined by the given face */
v3 		    DcelFaceDirection(const struct dcel *h, const m3 rot, const u32 fi);
/* Return the normal defined by the given face */
v3 		    DcelFaceNormalLocal(const struct dcel *h, const u32 fi);
/* Return the normal drirection defined by the given face */
v3 		    DcelFaceDirectionLocal(const struct dcel *h, const u32 fi);

struct plane 	DcelFaceClipPlane(const struct dcel *h, const m3 rot, const v3 pos, const v3 face_normal, const u32 e0, const u32 e1); /* Return clip plane of face containing edge e0e1, orthogonal to the face normal */

/* TODO: document, go through ... */
struct segment 	DcelFaceClipSegment(const struct dcel *h, const m3 rot, const v3 pos, const u32 fi, const struct segment *s); /* clip segment against face fi's edge-planes (No projection onto face plane!) */
u32 		DcelFaceProjectedPointTest(const struct dcel *h, const m3 rot, const v3 pos, const u32 fi, const v3 p); /* Project p onto face plane and test if it is on the face */

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

/* Return: support of vertex set given the direction, and supporting vertex index */
u32 	VertexSupport(v3 *support, const v3 dir, const v3 *v, const u32 v_count);
v3 	VertexCentroid(const v3 *vs, const u32 n);

#ifdef __cplusplus
} 
#endif

#endif
