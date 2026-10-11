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

#ifndef __DS_COLLISION_H__
#define __DS_COLLISION_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_base.h"
#include "ds_math.h"
#include "string_database.h"
#include "queue.h"
#include "tree.h"
#include "ds_bitset.h"

#define COLLISION_DEFAULT_MARGIN	(100.0f * F32_EPSILON)
#define COLLISION_POINT_DIST_SQ		(10000.0f * F32_EPSILON)

/*
bounding volume hierarchy
=========================
*/

struct bvhNode
{
    /* If leaf, bt_right == body_index, bt_left == shape index */
	BT_NODE;
    POOL_NODE;
	struct aabb bbox;
};
POOL_DECLARE(bvhNode);

struct bvh
{
    struct ds_BT        bt;  
    struct ds_BitSet    leaf_set;       /* dynamic specific */
    struct ds_BitSet    internal_set;   /* dynamic specific */
    struct bvhNodePool  pool;
	struct minQueue	    cost_queue;	    /* dynamic specific */
	u32			        heap_allocated;
};

/* free allocated resources */
void 		        BvhFree(struct bvh *tree);
/* Return the required size when serializing the bvh. */
u64                 BvhSerializeSize(const struct bvh *bvh);
/* Serialize the bvh; the cost queue is empty between insertions and not serialized. WARNING: Assumes bvh fits in the stream. */
void                BvhSerialize(struct ss *ss, const struct bvh *bvh);
/* Returns 1 on success and 0 on failure. The cost queue is always heap allocated: free the bvh with BvhFree. */
u32                 BvhTryDeserialize(struct arena *mem, struct ss *ss, struct bvh *bvh, const u32 growable);
/* Derive all internal bounding boxes from the leaves' bounding boxes */
void                BvhPropagateBoundingBoxesFromLeaves(struct bvh *bvh);
/* validate (ds_Assert) internal coherence of bvh */
void 		        BvhValidate(const struct bvh *bvh);
/* return total cost of bvh */
f32 		        BvhCost(const struct bvh *bvh);
/* Query all overlapping shapes */
struct bvh_QuerySet BvhQuery(struct arena *mem, const struct bvh *bvh, const struct bvhNode *node);
/* Query overlapping shapes with a body index > node->body (bt_right)  */
struct bvh_QuerySet BvhQueryAndFilterOnBody(struct arena *mem, const struct bvh *bvh, const struct bvhNode *node);

#define COST_QUEUE_INITIAL_COUNT 	64 

struct bvh_Query
{
    u32 tmp[2];
};

struct bvh_QuerySet
{
    u32     count;
    u32 *   shape;
};


struct bvh		    DbvhAlloc(struct arena *mem, const u32 initial_length, const u32 growable);
/* flush / reset the hierarchy  */
void 			    DbvhFlush(struct bvh *bvh);
/* id is an integer identifier from the outside, return index of added value */
u32 			    DbvhInsert(struct bvh *bvh, const u32 body, const u32 shape, const struct aabb *bbox);
/* remove leaf corresponding to index from tree */
void 			    DbvhRemove(struct bvh *bvh, const u32 index);

struct triMeshBvh
{
	const struct triMesh *	mesh;		
	struct bvh		        bvh;
	u32 *			        tri;		
	u32			            tri_count;	
    u32                     depth;      /* root=0, */
};

/* Return non-empty tri_mesh_bvh on success. */
struct triMeshBvh   TriMeshBvhConstruct(struct arena *mem, const struct triMesh *mesh, const u32 bin_count);


/*
bvh raycasting
==============
Nodes are visited in order of their box's entry t (the smallest t >= 0 in the box's volume, 0 if the origin is
inside it), so the search stops once the best hit is nearer than the next box. BvhRaycastInit takes all of mem
for the node queue: pass a scratch arena and push/pop (or flush) around each traversal. Raycasting your own
primitives in the leaves (see TriMeshBvhRaycastParameter, ds_DynamicsRaycastParameter):

	struct arena *mem = ArenaPushScratch();
	struct bvhRaycastInfo info = BvhRaycastInit(mem, bvh, ray);
	while (info.hit_queue.count)
	{
		const u32f32 tuple = MinQueueFixedPop(&info.hit_queue);
		if (info.hit.f < tuple.f)
		{
			break;
		}

		if (ds_BTLeafCheck(info.node + tuple.u))
		{
			// raycast the leaf's primitive(s), e.g. primitive = info.node[tuple.u].bt_child[0]
			const f32 t = primitive_raycast_parameter(primitive, ray);
			if (t < info.hit.f)
			{
				info.hit = u32f32_inline(primitive, t);
			}
		}
		else
		{
			BvhRaycastTestAndPushChildren(&info, tuple);
		}
	}
	ArenaPopScratch();
*/

/* the ray prepared for repeated box tests */
struct rayAabb
{
	v3	    origin;
	v3	    inv_dir;	/* 1 / dir, +-inf on an axis where dir is 0 */
	v3u32	sign;		/* sign bit of dir: 1 -> the near slab plane is the box's max */
};

struct rayAabb  RayAabb(const struct ray *ray);
/*
 * Return the interval [low, high] of t for which origin + t*dir is in the box (the whole line, t < 0 included);
 * empty if high < low. Conservative: rounding never shortens it, so a BVH traversal never skips a box the ray
 * touches (it may keep one the ray misses by an ulp). Raycasts apply the first surface hit rule to it
 * (AabbRaycastParameter); a BVH orders boxes by max(low, 0), the entry into the box's volume.
 */
intv            AabbRaycastInterval(const struct rayAabb *ray, const struct aabb *aabb);

struct bvhRaycastInfo
{
	u32f32			hit;
	struct rayAabb	ray;
	struct minQueueFixed	hit_queue;
	const struct bvh *	bvh;
	const struct bvhNode *	node;
};

/* Return the traversal state with the root queued if the ray hits its box; uses all of mem for the queue */
struct bvhRaycastInfo	BvhRaycastInit(struct arena *mem, const struct bvh *bvh, const struct ray *ray);
/* Queue the popped node's children whose boxes the ray hits, keyed by their entry t */
void 			        BvhRaycastTestAndPushChildren(struct bvhRaycastInfo *info, const u32f32 popped_tuple);


/********************************** COLLISION SHAPES **********************************/

enum c_ShapeType
{
	C_SHAPE_SPHERE,
	C_SHAPE_CAPSULE,
	C_SHAPE_CONVEX_HULL,
	C_SHAPE_TRI_MESH,	
	C_SHAPE_COUNT,
};

#define C_SHAPE_ID_SIZE     128
struct c_Shape
{
    u8                      id_buf[C_SHAPE_ID_SIZE];
    SDB_NODE;
	
	m3	                    inertia_tensor;		/* local shape frame intertia tensor (Assumes density=1.0, 
			                		                to get the interia tensor given a density, just multiply
			                		                the matrix with the given density. */
	v3	                    center_of_mass;		/* local shape frame center of mass */
	f32	                    volume;

	enum c_ShapeType        type;
	union
	{
		struct sphere 		sphere;
		struct capsule 		capsule;
		struct dcel		    hull;
		struct triMeshBvh 	mesh_bvh;
	};
};
SDB_DECLARE(c_Shape);

void	c_ShapeUpdateMassProperties(struct c_Shape *shape);


/*
c_Manifold
==========
a c_Manifold (contact manifold) contains the required information about how two shapes
are colliding for our physics solvers to solve them. One of the shapes are viewed as 
the reference shape. 
*/
struct c_Manifold
{
	v3 	    v[4];       /* contact point on the reference shape surface     */
	f32 	depth[4];   /* Contact point penetration depth (0.0f, INFINITY) */
	v3 	    n;		    /* Contact normal: Points away from reference       */
	u32 	v_count;    /* Contact point count                              */
};

/* Print manifold to file stderr */
void 	c_ManifoldDebugPrint(const struct c_Manifold *cm);
/* Sanity tests for debugging. Return 1 if valid, 0 otherwise. */
u32     c_ManifoldCheck(const struct c_Manifold *cm);
/* Transform the manifold */
void	c_ManifoldTransform(struct c_Manifold *dst, const struct c_Manifold *src, const m3 rot, const v3 translation);

/********************************** INTERSECTION TESTS **********************************/

/* 1 if the shapes overlap or touch (hull pairs: within GJK's touch band), else 0 */

u32     c_SphereTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
u32     c_CapsuleSphereTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
u32     c_CapsuleTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
u32     c_HullSphereTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
u32     c_HullCapsuleTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
u32     c_HullTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
u32     c_TriMeshBvhSphereTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
u32     c_TriMeshBvhCapsuleTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
u32     c_TriMeshBvhHullTest(const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);

/********************************** DISTANCE METHODS **********************************/

/*
GJK distance
------------
Description:
    Distance, closest points and normal of two convex shapes, computed in A's frame. Shapes closer than
    the rounding noise band GJK_TOUCH_TOLERANCE * eps * L (L ~ the shapes' extent in A's frame) count as
    touching. Uses the pushed numerics config (gjk_max_iterations).

Usage:
    struct GJKCache cache;
    v3f64 c_a, c_b;
    v3 n;
    const f32 d = GJK(&c_a, &c_b, &n, &cache, cache_in,
                      shape_a, &transform_a, shape_b, &transform_b, cutoff_distance);

    d == 0.0f           overlap or touch; c_a == c_b, n = 0
    d == F32_INFINITY   a separating plane proves distance > cutoff_distance; c_a, c_b, n garbage
    otherwise           the distance; c_a, c_b the closest points and n the unit normal from A to B (world
                        space), n accurate to ~6 eps * L / distance radians. Only an upper bound if the
                        iteration limit was reached

    cache_in            NULL or the pair's previous cache_out; warm-starts the search unless the cached
                        simplex changed size by more than 2x. cache_out is always written.
    cutoff_distance     distances above it are of no interest (contacts: radii + speculative margin);
                        F32_INFINITY for distance queries. Not every stop tests it, so a finite result
                        may still exceed it; callers compare.

Internals:
    c_distance.c: the termination cases and the derivation of the touch band.
*/

/*
 * GJK warm start between frames on the same pair: the support indices of the last simplex. A cache that
 * exists is valid: count is 1..4 and the indices belong to the pair's shapes. No cache is NULL, never an
 * empty cache.
 */
struct GJKCache
{
    f32 metric;             /* length/area/volume of the cached simplex; flushed if it changes > 2x */
    u16 count;              /* 1..4 */
    u16 index_a[4];
    u16 index_b[4];
};

/* touch band width in units of the rounding noise eps * L (see c_distance.c) */
#define GJK_TOUCH_TOLERANCE 100.0f

f32     GJK(v3f64 *c_a, v3f64 *c_b, v3 *n, struct GJKCache *cache_out, const struct GJKCache *cache_in, const struct c_Shape *shape_a, const ds_TransformWorld *t_a, const struct c_Shape *shape_b, const ds_TransformWorld *t_b, const f32 cutoff_distance);

f32     c_SphereDistance(v3f64 *c1, v3f64 *c2, const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
f32     c_CapsuleSphereDistance(v3f64 *c1, v3f64 *c2, const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
f32     c_CapsuleDistance(v3f64 *c1, v3f64 *c2, const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
f32     c_HullSphereDistance(v3f64 *c1, v3f64 *c2, const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
f32     c_HullCapsuleDistance(v3f64 *c1, v3f64 *c2, const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
f32     c_HullDistance(v3f64 *c1, v3f64 *c2, const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
f32     c_TriMeshBvhSphereDistance(v3f64 *c1, v3f64 *c2, const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
f32     c_TriMeshBvhCapsuleDistance(v3f64 *c1, v3f64 *c2, const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);
f32     c_TriMeshBvhHullDistance(v3f64 *c1, v3f64 *c2, const struct c_Shape *s1, const ds_TransformWorld *t1, const struct c_Shape *s2, const ds_TransformWorld *t2);

/********************************** CONTACT MANIFOLD METHODS **********************************/

struct c_ContactResult
{
    u32                         manifold_count;     /* Number of stored manifolds               */
    u32                         cache_count;        /* Number of stored caches                  */
    struct c_Manifold *         manifold;           /* Manifolds (if any)                       */
    struct c_SatCache *         cache;              /* Caches (if any)                          */
    struct GJKCache *           gjk_cache;          /* GJK cache, at most one (NULL if none)    */
    u32 *                       tri;                /* Sorted triangles, low-to-high (if any)   */
    u32 *                       tri_manifold;       /* Sorted triangle manifold indices (if any)
                                                       m = tri_manifold[T] => manifold if tri[T]
                                                       is manifold[m].                          */
};

struct c_ContactResult  c_SphereContact(struct arena *frame, const struct c_ContactResult *not_used, const struct c_Shape *s[2], const ds_Transform t[2], const u32 reference_index);
struct c_ContactResult  c_CapsuleSphereContact(struct arena *frame, const struct c_ContactResult *not_used, const struct c_Shape *s[2], const ds_Transform t[2], const u32 reference_index);
struct c_ContactResult  c_CapsuleContact(struct arena *frame, const struct c_ContactResult *not_used, const struct c_Shape *s[2], const ds_Transform t[2], const u32 reference_index);
struct c_ContactResult  c_HullSphereContact(struct arena *frame, const struct c_ContactResult *not_used, const struct c_Shape *s[2], const ds_Transform t[2], const u32 reference_index);
struct c_ContactResult  c_HullCapsuleContact(struct arena *frame, const struct c_ContactResult *not_used, const struct c_Shape *s[2], const ds_Transform t[2], const u32 reference_index);
struct c_ContactResult  c_HullContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s[2], const ds_Transform t[2], const u32 reference_index);
struct c_ContactResult  c_TriMeshBvhSphereContact(struct arena *frame, const struct c_ContactResult *not_used, const struct c_Shape *s[2], const ds_Transform t[2], const u32 reference_index);
struct c_ContactResult  c_TriMeshBvhCapsuleContact(struct arena *frame, const struct c_ContactResult *not_used, const struct c_Shape *s[2], const ds_Transform t[2], const u32 reference_index);
struct c_ContactResult  c_TriMeshBvhHullContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s[2], const ds_Transform t[2], const u32 reference_index);

/********************************** RAYCAST **********************************/

/*
 * The ray sheared into ray space, where it is the +z axis from the origin (Woop, Benthin, Wald 2013). Built once
 * per ray and shared by every triangle test, so a vertex transforms identically in all its triangles (what makes
 * the tests watertight).
 */
struct rayWoop
{
	v3	origin;
	u32	kx;		/* kz: the axis of dir's largest |component|; kx = (kz + 1) % 3, ky = (kx + 1) % 3 */
	u32	ky;
	u32	kz;
	f32	sx;		/* dir[kx] / dir[kz] */
	f32	sy;		/* dir[ky] / dir[kz] */
	f32	sz;		/* 1 / dir[kz] */
};

/*
 * Return the smallest t >= 0 at which origin + t*dir is on the shape's surface (the first surface hit), or
 * F32_INFINITY. Two-sided: surfaces count from either side, so an origin inside a solid returns its exit.
 * The ray is in the shape's local frame.
 * precondition: |ray->dir|^2 > 0 in f32 (asserted; rejects |dir| below ~4e-23, whose square underflows); callers check once per ray,
 * not per shape
 */
f32 SphereRaycastParameter(const struct sphere *sphere, const struct ray *ray);
f32 CapsuleRaycastParameter(const struct capsule *capsule, const struct ray *ray);
/* hull: a closed convex polyhedron with planar faces */
f32 HullRaycastParameter(const struct dcel *hull, const struct ray *ray);
f32 AabbRaycastParameter(const struct aabb *aabb, const struct ray *ray);
f32 TriMeshBvhRaycastParameter(const struct triMeshBvh *mesh_bvh, const struct ray *ray);
/*
 * triangle abc, any winding (no culling). A ray in the triangle's plane misses; an origin on the triangle with
 * dir out of its plane returns 0.
 */
f32 TriRaycastParameter(const v3 a, const v3 b, const v3 c, const struct ray *ray);
/* TriRaycastParameter with the ray set up once (RayWoop) for many triangles, e.g. a mesh */
struct rayWoop  RayWoop(const struct ray *ray);
f32 TriRaycastParameterEx(const struct rayWoop *ray, const v3 a, const v3 b, const v3 c);

/*
 * The local functions above, with the ray in world space and the shape placed by transform. t is the
 * same in both frames (the rotation keeps |dir|).
 */
f32 SphereRaycastWorldParameter(const struct sphere *sphere, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 CapsuleRaycastWorldParameter(const struct capsule *capsule, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 HullRaycastWorldParameter(const struct dcel *hull, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 AabbRaycastWorldParameter(const struct aabb *aabb, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 TriMeshBvhRaycastWorldParameter(const struct triMeshBvh *mesh_bvh, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 TriRaycastWorldParameter(const v3 a, const v3 b, const v3 c, const ds_TransformWorld *transform, const struct rayWorld *ray);

/* The world functions above on a c_Shape of the matching type (the physics engine's shape table) */
f32 c_SphereRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 c_CapsuleRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 c_HullRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 c_TriMeshBvhRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray);

#ifdef __cplusplus
} 
#endif

#endif
