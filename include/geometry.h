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

/****************** GEOMETRIC PRIMITIVES ******************/

/**
 * AABB: geomtrical primitive; Axis aligned bounding box
 * center: center of the box
 * hw: half width of the box in each dimension
 */
struct aabb 
{
	vec3 center;
	vec3 hw;
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
	    vec3 normal;
	    vec3 normal_direction;
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
	vec3 origin;
	vec3 dir;
};

/**
 * segment: geometrical primitive; directional two point segment
 * p0: segment start
 * p1: segment end 
 * dir: non-normalized direction vector
 */
struct segment
{
	vec3 p[2];	
	vec3 dir;	/* p[1]-p[0] */
};

/**
 * segment: geometrical primitive
 * p0: segment start
 * p1: segment end 
 * dir: non-normalized direction vector
 */
struct sphere
{
	vec3 center;
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
struct sphere 	SphereConstruct(const vec3 center, const f32 radius);
/* return t: smallest t >= 0 such that p = origin + t*dir is a point on the sphere, or F32_INF if no such t exist */
f32 		SphereRaycastParameter(const struct sphere *sph, const struct ray *ray);
/* Return 1 if raycast hit sphere, 0 otherwise. If hit, set intersection  */
u32 		SphereRaycast(vec3 intersection, const struct sphere *sph, const struct ray *ray);
/* Return support of sphere in given direction. sph->position is ignored here, so use pos as the real position */
void		SphereSupport(vec3 support, const vec3 dir, const struct sphere *sph, const vec3 pos);

/*********************************** ray ************************************/

/* return constructed ray */
struct ray 	RayConstruct(const vec3 origin, const vec3 dir);
/* return segment: s.p0 = r.origin, s.p1 = r.origin + t * r.dir */
struct segment	RayConstructSegment(const struct ray *r, const f32 t);
/* set r_c = ray.origin + t * ray.dir */
void		RayPoint(vec3 r_c, const struct ray *ray, const f32 t);
/* return t: closest point on ray to p = origin + t * dir */
f32 		RayPointClosestPointParameter(const struct ray *ray, const vec3 p);
/* return squared distance from p to ray, and set RayPoint to the closest point on the ray */
f32 		RayPointDistanceSquared(vec3 RayPoint, const struct ray *ray, const vec3 p);
/* return squared distance from s to ray, and set r_c and s_c to the closest points on the primitives */
f32 		RaySegmentDistanceSquared(vec3 r_c, vec3 s_c, const struct ray *ray, const struct segment *s);

/********************************* segment **********************************/

/* construct segment */
struct segment 	SegmentConstruct(const vec3 p0, const vec3 p1);
/* Return 1 if the end-points of s are within a distance of sqrt(min_dist_sq) of each other, otherwise return 0. */
u32             SegmentPointCheck(const struct segment *s, const f32 min_dist_sq);
/* return squared distance between s1 and s2; set c1, c2 to closest point on s1, s2 respectively  */
f32 		    SegmentDistanceSquared(vec3 c1, vec3 c2, const struct segment *s1, const struct segment *s2);
/* return parameters t1,t2 of closest points c1,c2 on s1,s2 such that ci = si.p0(1-ti) + s1.p1*ti  */
void		    SegmentClosestParameter(f32 *t1, f32 *t2, const struct segment *s1, const struct segment *s2);
/* return squared distance between s and p; set c to the closest point on s to p */
f32 		    SegmentPointDistanceSquared(vec3 c, const struct segment *s, const vec3 p);
/* Return parameter t of projected barycentric point p to segment s: PROJECTION_ON_LINE(p) = s.p0*(1-t) + s.p1*t */
f32		        SegmentPointProjectedBcParameter(const struct segment *s, const vec3 p);
/* Return parameter g of closest barycentric point p to segment s: PROJECTION_ON_SEGMENT(p) = s.p0*(1-t) + s.p1*t, 0.0f <= t <= 1.0f */
f32 		    SegmentPointClosestBcParameter(const struct segment *s, const vec3 p);
/* set bc_p = s.p0*(1-t) + s.p1*t */
void 		    SegmentBc(vec3 bc_p, const struct segment *s, const f32 t); 	

/* Return the bounding box of the segment  */
struct aabb     BboxSegment(const struct segment *s);

/********************************** plane ***********************************/

/* construct plane with given normal direction n containing point p */
struct plane 	PlaneConstruct(const vec3 n, const vec3 p); 
/* construct normalized plane with given normal direction n containing point p */
struct plane 	PlaneConstructNormalized(const vec3 n, const vec3 p); 
/* construct plane from CCW triangle abc */
struct plane 	PlaneConstructFromCcwTriangle(const vec3 a, const vec3 b, const vec3 c);
/* construct normalized plane from CCW triangle abc */
struct plane 	PlaneConstructNormalizedFromCcwTriangle(const vec3 a, const vec3 b, const vec3 c);
/* Return 1 if p is infront of plane, i.e. a positive signed distance, otherwise 0 */
u32 		    PlanePointInfrontCheck(const struct plane *pl, const vec3 p);
/* Return 1 if p is behind plane, i.e. a negative signed distance, otherwise 0 */
u32 		    PlanePointBehindCheck(const struct plane *pl, const vec3 p);
/* Return 1 if segment is parallel to plane, otherwise return 0. */
u32             PlaneSegmentParallelCheck(const struct plane *pl, const struct segment *s);
/* return t: s.p0 + t*s.dir is point on plane */
f32 		    PlaneSegmentClipParameter(const struct plane *pl, const struct segment *s);
/* return 1 if clip happened, otherwise 0. If 1, return valid clip point */
u32 		    PlaneSegmentClip(vec3 clip, const struct plane *pl, const struct segment *s);
/* return 1 if clip happened, otherwise 0 */
u32 		    PlaneSegmentTest(const struct plane *pl, const struct segment *s); 
/* return signed distance (measured in |plane.normal_direction| units) between plane and point (infront of plane == positive) */
f32 		    PlanePointSignedDistance(const struct plane *pl, const vec3 p);
/* return absolute distance (measured in |plane.normal_direction| units) between plane and point */
f32 		    PlanePointDistance(const struct plane *pl, const vec3 p);
/* return the signed distance (measured in |plane.normal_direction| units) of point p to plane pl, and set the projection of p onto pl. */
f32 		    PlanePointProjection(vec3 proj, const struct plane *pl, const vec3 p);

/* Return t such that ray->origin + t*ray->dir is a point on the given plane. If no such t exist, return F32_INFINITY. */
f32 		PlaneRaycastParameter(const struct plane *plane, const struct ray *ray);
/* Return 1 if raycast hit plane, 0 otherwise. If hit, set intersection  */
u32 		PlaneRaycast(vec3 intersection, const struct plane *plane, const struct ray *ray);

/********************************** AABB ************************************/

/* Return smallest AABB with a given margin of the input vertex set,   */
void		AabbVertex(struct aabb *dst, const vec3ptr v, const u32 v_count, const f32 margin);
/* Return smallest AABB that contains both a and b  */
void		AabbUnion(struct aabb *box_union, const struct aabb *a, const struct aabb *b);
/* Return AABB of rotated AABB. */
void		AabbRotate(struct aabb *dst, const struct aabb *src, mat3 rotation);
/* Return 1 if a and b intersect, 0 otherwise  */
u32 		AabbTest(const struct aabb *a, const struct aabb *b);
/* Return 1 if a fully contains b, 0 otherwise  */
u32 		AabbContains(const struct aabb *a, const struct aabb *b);
/* Return 1 if a (extended with given margin) fully contains b, 0 otherwise  */
u32 		AabbContainsMargin(const struct aabb *a, const struct aabb *b, const f32 margin);
/* sets up vertex buffer to use with glDrawArrays. Returns number of bytes written. */
u64 		AabbPushLinesBuffered(u8 *buf, const u64 bufsize, const struct aabb *box, const vec4 color);
/* sets up vertex buffer to use with glDrawArrays. Returns number of bytes written. */
u64 		AabbTransformPushLinesBuffered(u8 *buf, const u64 bufsize, const struct aabb *box, const vec3 translation, mat3 rotation, const vec4 color);

/* return AABB bounding box of triangle */
struct aabb	BboxTriangle(const vec3 p0, const vec3 p1, const vec3 p2);
/* Return smallest AABB that contains both a and b  */
struct aabb	BboxUnion(const struct aabb a, const struct aabb b);

/* Setup parameters for extended raycasting functions. */
void 		AabbRaycastParameterExSetup(vec3 multiplier, vec3u32 dir_sign_bit, const struct ray *ray);
/* Extended AabbRaycastParameter optimized for multiple raycasts against AABBs using same ray. 
 * return t: smallest t >= 0 such that p = origin + t*dir is a point in the AABB volume, or F32_INF if no such t exist */
f32 		AabbRaycastParameterEx(const struct aabb *aabb, const struct ray *ray, const vec3 multiplier, const vec3u32 dir_sign_bit);
/* return t: smallest t >= 0 such that p = origin + t*dir is a point in the AABB volume, or F32_INF if no such t exist */
f32 		AabbRaycastParameter(const struct aabb *a, const struct ray *ray);
/* Extended AabbRaycast, optimized for multiple raycasts against AABBs using same ray. 
 * If the ray hits aabb, return 1 and set intersection. otherwise return 0. */
u32 		AabbRaycastEx(vec3 intersection, const struct aabb *aabb, const struct ray *ray, const vec3 multiplier, const vec3u32 dir_sign_bit);
/* If the ray hits aabb, return 1 and set intersection. otherwise return 0. */
u32 		AabbRaycast(vec3 intersection, const struct aabb *aabb, const struct ray *ray);

/********************************* capsule **********************************/

/* Return support of capsule in given direction. */
void		CapsuleSupport(vec3 support, const vec3 dir, const struct capsule *cap, mat3 rot, const vec3 pos);

/********************************* tri_mesh **********************************/

/*
 * triangle mesh (CCW) - set of ungrouped triangles.
 */
struct triMesh
{
	vec3ptr		v;
	vec3u32ptr	tri;
	u32 		v_count;	
	u32 		tri_count;
};

/* return bounding box of triMesh */
struct aabb	TriMeshBbox(const struct triMesh *mesh);
/* return t: smallest t >= 0 such that p = origin + t*dir is a point on the triangle, or F32_INF if no such t exist */
f32 		TriMeshRaycastParameter(const struct triMesh *mesh, const u32 tri, const struct ray *ray);
/* If the ray hits triangle (ccw), return 1 and set intersection. otherwise return 0. */
u32 		TriMeshRaycast(vec3 intersection, const struct triMesh *mesh, const u32 tri, const struct ray *ray);

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

struct TriVoronoi
{
    struct segment  s[3];           /* edge segment */
    struct plane    edge_plane[3];  /* edge plane orthogonal to face plane */
    struct plane    face_plane;     /* triangle plane (CCW) */
    vec3            t[3];
};


/* Get normal of ccw triangle */
void 		TriCcwNormal(vec3 normal, const vec3 p0, const vec3 p1, const vec3 p2);
/* Get direction of ccw triangle */
void 		TriCcwDirection(vec3 dir, const vec3 p0, const vec3 p1, const vec3 p2);


/* Setup a TriVoronoi struct corresponding to the CCW triangle t and return true if t is robust, false otherwise.  */
u32         TriVoronoiInitCcw(struct TriVoronoi *tv, const vec3 t[3]);

/* 
 * Return squared distance from segment s to triangle t, and set c_s to be the closest point on s, and c_t to be 
 * the closest point on the triangle.
 *
 * NOTE: If the returned distance is 0.0f, c_t is not necessarily c_s, but instead c_t ~= c_s. Use one of the points
 * for consistency if needed.
 */
f32 		TriCcwSegmentDistanceSquared(vec3 c_t, vec3 c_s, enum TriVoronoiRegion *region, const struct segment *s, const struct TriVoronoi *tv);

/* 
 * Return squared distance from point p to triangle t, and set c to be the closest point on the triangle. 
 * lambda_count is set to indicate the number of non-zero lambda components, and lambda is set to the 
 * barocentric coordinates:
 */
f32         TriCcwPointDistanceSquared(vec3 c, enum TriVoronoiRegion *region, const vec3 point, const struct TriVoronoi *tv);

/* 
 * Return t in [0,1] such that clip = s.p0*(1-t) + s.p1*t is a point on the given plane. If no such t exist, 
 * return F32_INFINITY. 
 */
f32         TriCcwSegmentClipParameter(vec3 clip, const struct segment *s, const struct TriVoronoi *tv);

/* 
 * Return 1 if segment clips triangle, 0 otherwise. If clip, set the clip point.
 */
u32         TriCcwSegmentClip(vec3 clip, const struct segment *s, const struct TriVoronoi *tv);

/* 
 * Return the remaining segment when clipping s against all side-planes of the triangle. WARNING: Assumes s in
 * at least partially within the voronoi face region.
 */
struct segment  TriCcwSegmentSideClip(const struct segment *s, const struct TriVoronoi *tv);


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
	vec3ptr	v;
	u32 f_count;
	u32 e_count;
	u32 v_count;
};

/* return dcel { 0 } */
struct dcel 	DcelEmpty(void);
/* return dcel box stub */
struct dcel 	DcelBoxStub(void);
/* return arena allocated dcel box with given half widths */
struct dcel 	DcelBox(struct arena *mem, const vec3 hw);
/* return arena allocated dcel convex hull of input points. On failure, an empty dcel is returned. */
struct dcel 	DcelConvexHull(struct arena *mem, const vec3ptr v, const u32 v_count, const f32 tol);
/* Return support of dcel in given direction, and return supporting vertex index */
u32		DcelSupport(vec3 support, const vec3 dir, const struct dcel *hull, mat3 rot, const vec3 pos);

/* TODO: document, go through ... */
void 		DcelFaceDirection(vec3 dir, const struct dcel *h, const u32 fi); /* not normalized */
void 		DcelFaceNormal(vec3 normal, const struct dcel *h, const u32 fi); /* normalized */
struct plane 	DcelFacePlane(const struct dcel *h, mat3 rot, const vec3 pos, const u32 fi);
struct plane 	DcelFaceClipPlane(const struct dcel *h, mat3 rot, const vec3 pos, const vec3 face_normal, const u32 e0, const u32 e1); /* Return clip plane of face containing edge e0e1, orthogonal to the face normal */
struct segment 	DcelFaceClipSegment(const struct dcel *h, mat3 rot, const vec3 pos, const u32 fi, const struct segment *s); /* clip segment against face fi's edge-planes (No projection onto face plane!) */
u32 		DcelFaceProjectedPointTest(const struct dcel *h, mat3 rot, const vec3 pos, const u32 fi, const vec3 p); /* Project p onto face plane and test if it is on the face */

void 		DcelEdgeNormal(vec3 dir, const struct dcel *h, const u32 ei);
void 		DcelEdgeDirection(vec3 dir, const struct dcel *h, const u32 ei);
struct segment 	DcelEdgeSegment(const struct dcel *h, mat3 rot, const vec3 pos, const u32 ei);

void 		DcelAssertTopology(struct dcel *dcel);

#ifdef DS_DEBUG
#define COLLISION_HULL_ASSERT(dcel)	DcelAssertTopology(dcel)
#else
#define COLLISION_HULL_ASSERT(dcel)	
#endif

/********************************* vertex operations ***********************************/

/* Return: support of vertex set given the direction, and supporting vertex index */
u32 	VertexSupport(vec3 support, const vec3 dir, const vec3ptr v, const u32 v_count);
void 	VertexCentroid(vec3 centroid, const vec3ptr vs, const u32 n);

#ifdef __cplusplus
} 
#endif

#endif
