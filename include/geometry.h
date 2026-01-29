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
struct AABB 
{
	vec3 center;
	vec3 hw;
};

/**
 * plane: geomtrical primitive
 * normal: normal of plane
 * signed_distance: Signed distance to plane; plane_normal * signed_distance is point on plane.
 */
struct plane 
{
	vec3 normal;
	f32 signed_distance;
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
	vec3 p0;	
	vec3 p1;	
	vec3 dir;	/* p1-p0 */
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
struct sphere 	sphere_construct(const vec3 center, const f32 radius);
/* return t: smallest t >= 0 such that p = origin + t*dir is a point on the sphere, or F32_INF if no such t exist */
f32 		sphere_raycast_parameter(const struct sphere *sph, const struct ray *ray);
/* Return 1 if raycast hit sphere, 0 otherwise. If hit, set intersection  */
u32 		sphere_raycast(vec3 intersection, const struct sphere *sph, const struct ray *ray);
/* Return support of sphere in given direction. sph->position is ignored here, so use pos as the real position */
void		sphere_support(vec3 support, const vec3 dir, const struct sphere *sph, const vec3 pos);

/*********************************** ray ************************************/

/* return constructed ray */
struct ray 	ray_construct(const vec3 origin, const vec3 dir);
/* return segment: s.p0 = r.origin, s.p1 = r.origin + t * r.dir */
struct segment	ray_construct_segment(const struct ray *r, const f32 t);
/* set r_c = ray.origin + t * ray.dir */
void		ray_point(vec3 r_c, const struct ray *ray, const f32 t);
/* return t: closest point on ray to p = origin + t * dir */
f32 		ray_point_closest_point_parameter(const struct ray *ray, const vec3 p);
/* return squared distance from p to ray, and set ray_point to the closest point on the ray */
f32 		ray_point_distance_sq(vec3 ray_point, const struct ray *ray, const vec3 p);
/* return squared distance from s to ray, and set r_c and s_c to the closest points on the primitives */
f32 		ray_segment_distance_sq(vec3 r_c, vec3 s_c, const struct ray *ray, const struct segment *s);

/********************************* segment **********************************/

/* construct segment */
struct segment 	segment_construct(const vec3 p0, const vec3 p1);
/* return squared distance between s1 and s2; set c1, c2 to closest point on s1, s2 respectively  */
f32 		segment_distance_sq(vec3 c1, vec3 c2, const struct segment *s1, const struct segment *s2);
/* return squared distance between s and p; set c to the closest point on s to p */
f32 		segment_point_distance_sq(vec3 c, const struct segment *s, const vec3 p);
/* Return parameter t of projected barycentric point p to segment s: PROJECTION_ON_LINE(p) = s.p0*(1-t) + s.p1*t */
f32		segment_point_projected_bc_parameter(const struct segment *s, const vec3 p);
/* Return parameter g of closest barycentric point p to segment s: PROJECTION_ON_SEGMENT(p) = s.p0*(1-t) + s.p1*t, 0.0f <= t <= 1.0f */
f32 		segment_point_closest_bc_parameter(const struct segment *s, const vec3 p);
/* set bc_p = s.p0*(1-t) + s.p1*t */
void 		segment_bc(vec3 bc_p, const struct segment *s, const f32 t); 	

/********************************** plane ***********************************/

/* construct plane with given normal n containing point p */
struct plane 	plane_construct(const vec3 n, const vec3 p); 
/* construct plane from CCW triangle abc */
struct plane 	plane_construct_from_ccw_triangle(const vec3 a, const vec3 b, const vec3 c);
/* return 1: If p is infront of plane, i.e. a positive signed distance, otherwise 0*/
u32 		plane_point_is_infront(const struct plane *pl, const vec3 p);
/* return 1: If p is behind plane, i.e. a negative signed distance, otherwise 0*/
u32 		plane_point_is_behind(const struct plane *pl, const vec3 p);
 /* return t: s.p0 + t*s.dir is point on plane */
f32 		plane_segment_clip_parameter(const struct plane *pl, const struct segment *s);
 /* return 1 if clip happened, otherwise 0. If 1, return valid clip point */
u32 		plane_segment_clip(vec3 clip, const struct plane *pl, const struct segment *s);
/* return 1 if clip happened, otherwise 0 */
u32 		plane_segment_test(const struct plane *pl, const struct segment *s); 
 /* return signed distance between plane and point (infront of plane == positive) */
f32 		plane_point_signed_distance(const struct plane *pl, const vec3 p);
 /* return absolute distance between plane and point */
f32 		plane_point_distance(const struct plane *pl, const vec3 p);
/* Return t such that ray->origin + t*ray->dir is a point on the given plane. If no such t exist, return F32_INFINITY. */
f32 		plane_raycast_parameter(const struct plane *plane, const struct ray *ray);
/* Return 1 if raycast hit plane, 0 otherwise. If hit, set intersection  */
u32 		plane_raycast(vec3 intersection, const struct plane *plane, const struct ray *ray);

/********************************** AABB ************************************/

/* Return smallest AABB with a given margin of the input vertex set,   */
void		AABB_vertex(struct AABB *dst, const vec3ptr v, const u32 v_count, const f32 margin);
/* Return smallest AABB that contains both a and b  */
void		AABB_union(struct AABB *box_union, const struct AABB *a, const struct AABB *b);
/* Return AABB of rotated AABB. */
void		AABB_rotate(struct AABB *dst, const struct AABB *src, mat3 rotation);
/* Return 1 if a and b intersect, 0 otherwise  */
u32 		AABB_test(const struct AABB *a, const struct AABB *b);
/* Return 1 if a fully contains b, 0 otherwise  */
u32 		AABB_contains(const struct AABB *a, const struct AABB *b);
/* Return 1 if a (extended with given margin) fully contains b, 0 otherwise  */
u32 		AABB_contains_margin(const struct AABB *a, const struct AABB *b, const f32 margin);
/* sets up vertex buffer to use with glDrawArrays. Returns number of bytes written. */
u64 		AABB_push_lines_buffered(u8 *buf, const u64 bufsize, const struct AABB *box, const vec4 color);
/* sets up vertex buffer to use with glDrawArrays. Returns number of bytes written. */
u64 		AABB_transform_push_lines_buffered(u8 *buf, const u64 bufsize, const struct AABB *box, const vec3 translation, mat3 rotation, const vec4 color);

/* return AABB bounding box of triangle */
struct AABB	bbox_triangle(const vec3 p0, const vec3 p1, const vec3 p2);
/* Return smallest AABB that contains both a and b  */
struct AABB	bbox_union(const struct AABB a, const struct AABB b);



/* AABB raycasting */

/* Setup parameters for extended raycasting functions. */
void 		AABB_raycast_parameter_ex_setup(vec3 multiplier, vec3u32 dir_sign_bit, const struct ray *ray);
/* Extended AABB_raycast_parameter optimized for multiple raycasts against AABBs using same ray. 
 * return t: smallest t >= 0 such that p = origin + t*dir is a point in the AABB volume, or F32_INF if no such t exist */
f32 		AABB_raycast_parameter_ex(const struct AABB *aabb, const struct ray *ray, const vec3 multiplier, const vec3u32 dir_sign_bit);
/* return t: smallest t >= 0 such that p = origin + t*dir is a point in the AABB volume, or F32_INF if no such t exist */
f32 		AABB_raycast_parameter(const struct AABB *a, const struct ray *ray);
/* Extended AABB_raycast, optimized for multiple raycasts against AABBs using same ray. 
 * If the ray hits aabb, return 1 and set intersection. otherwise return 0. */
u32 		AABB_raycast_ex(vec3 intersection, const struct AABB *aabb, const struct ray *ray, const vec3 multiplier, const vec3u32 dir_sign_bit);
/* If the ray hits aabb, return 1 and set intersection. otherwise return 0. */
u32 		AABB_raycast(vec3 intersection, const struct AABB *aabb, const struct ray *ray);

/********************************* capsule **********************************/

/* Return support of capsule in given direction. */
void	capsule_support(vec3 support, const vec3 dir, const struct capsule *cap, mat3 rot, const vec3 pos);

/********************************* tri_mesh **********************************/

/*
 * triangle mesh (CCW) - set of ungrouped triangles.
 */
struct tri_mesh
{
	vec3ptr		v;
	vec3u32ptr	tri;
	u32 		v_count;	
	u32 		tri_count;
};

struct AABB	tri_mesh_bbox(const struct tri_mesh *mesh);

/* return t: smallest t >= 0 such that p = origin + t*dir is a point on the triangle, or F32_INF if no such t exist */
f32 	tri_ccw_raycast_parameter(const struct tri_mesh *mesh, const u32 tri, const struct ray *ray);
/* If the ray hits triangle (ccw), return 1 and set intersection. otherwise return 0. */
u32 	tri_ccw_raycast(vec3 intersection, const struct tri_mesh *mesh, const u32 tri, const struct ray *ray);

/* get normal of ccw triangle */
void 	tri_ccw_normal(vec3 normal, const vec3 p0, const vec3 p1, const vec3 p2);
/* get direction of ccw triangle */
void 	tri_ccw_direction(vec3 dir, const vec3 p0, const vec3 p1, const vec3 p2);



/********************************** dcel ************************************/

struct dcel_face
{
	u32 first;	/* first half edge */
	u32 count;	/* edge count */
};

struct dcel_edge
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
	struct dcel_face *f;		/* f[i] = half-edge of face i */
	struct dcel_edge *e;
	vec3ptr	v;
	u32 f_count;
	u32 e_count;
	u32 v_count;
};

/* return dcel { 0 } */
struct dcel 	dcel_empty(void);
/* return dcel box stub */
struct dcel 	dcel_box_stub(void);
/* return arena allocated dcel box with given half widths */
struct dcel 	dcel_box(struct arena *mem, const vec3 hw);
/* return arena allocated dcel convex hull of input points. On failure, an empty dcel is returned. */
struct dcel 	dcel_convex_hull(struct arena *mem, const vec3ptr v, const u32 v_count, const f32 tol);
/* Return support of dcel in given direction, and return supporting vertex index */
u32		dcel_support(vec3 support, const vec3 dir, const struct dcel *hull, mat3 rot, const vec3 pos);

/* TODO: document, go through ... */
void 		dcel_face_direction(vec3 dir, const struct dcel *h, const u32 fi); /* not normalized */
void 		dcel_face_normal(vec3 normal, const struct dcel *h, const u32 fi); /* normalized */
struct plane 	dcel_face_plane(const struct dcel *h, mat3 rot, const vec3 pos, const u32 fi);
struct plane 	dcel_face_clip_plane(const struct dcel *h, mat3 rot, const vec3 pos, const vec3 face_normal, const u32 e0, const u32 e1); /* Return clip plane of face containing edge e0e1, orthogonal to the face normal */
struct segment 	dcel_face_clip_segment(const struct dcel *h, mat3 rot, const vec3 pos, const u32 fi, const struct segment *s); /* clip segment against face fi's edge-planes (No projection onto face plane!) */
u32 		dcel_face_projected_point_test(const struct dcel *h, mat3 rot, const vec3 pos, const u32 fi, const vec3 p); /* Project p onto face plane and test if it is on the face */

void 		dcel_edge_normal(vec3 dir, const struct dcel *h, const u32 ei);
void 		dcel_edge_direction(vec3 dir, const struct dcel *h, const u32 ei);
struct segment 	dcel_edge_segment(const struct dcel *h, mat3 rot, const vec3 pos, const u32 ei);

/* TODO: merge with newer commented out dcel_assert_topology... */
void 		dcel_assert_topology(struct dcel *dcel);

#ifdef DS_DEBUG
#define COLLISION_HULL_ASSERT(dcel)	dcel_assert_topology(dcel)
#else
#define COLLISION_HULL_ASSERT(dcel)	
#endif

/********************************* vertex operations ***********************************/

/* Return: support of vertex set given the direction, and supporting vertex index */
u32 	vertex_support(vec3 support, const vec3 dir, const vec3ptr v, const u32 v_count);
void 	vertex_centroid(vec3 centroid, const vec3ptr vs, const u32 n);

#ifdef __cplusplus
} 
#endif

#endif
