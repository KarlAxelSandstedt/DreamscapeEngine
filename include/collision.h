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
/* Return (index, ray hit parameter) on closest hit, or (U32_MAX, F32_INFINITY) on no hit */
u32f32 			    TriMeshBvhRaycast(struct arena *tmp, const struct triMeshBvh *mesh_bvh, const struct ray *ray);


/*
bvh raycasting
==============
To implement raycast using external primitives, one can use the following code:

	ArenaPushRecord(mem);

	struct bvhRaycastInfo info = BvhRaycastInit(mem, bvh, ray);
	while (info.hit_queue.count)
	{
		const u32f32 tuple = MinQueueFixedPop(&info.hit_queue);
		if (info.hit.f < tuple.f)
		{
			break;	
		}

		if (bt_LeafCheck(info.node + tuple.u))
		{
			//TODO: Here you implement raycasting against your external primitive.
			const f32 t = external_primitive_raycast(...);
			if (t < info.hit.f)
			{
				info.hit = u32f32_inline(tuple.u, t);
			}
		}
		else
		{
			BvhRaycastTestAndPushChildren(&info, tuple);
		}
	}

	ArenaPopRecord(mem);
*/
struct bvhRaycastInfo
{
	u32f32			hit;
	v3 			    multiplier;
	v3u32 		    dir_sign_bit;
	struct minQueueFixed	hit_queue;
	const struct ray *	ray;
	const struct bvh *	bvh;
	const struct bvhNode *	node;
};

/* Initiate raycast information */
struct bvhRaycastInfo	BvhRaycastInit(struct arena *mem, const struct bvh *bvh, const struct ray *ray);
/* test Raycasting against child nodes and push hit children onto queue */
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

u32     c_SphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
u32     c_CapsuleSphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
u32     c_CapsuleTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
u32     c_HullSphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
u32     c_HullCapsuleTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
u32     c_HullTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
u32     c_TriMeshBvhSphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
u32     c_TriMeshBvhCapsuleTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
u32     c_TriMeshBvhHullTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);

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
    v3 c_a, c_b, n;
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
    ds_distance.c: the termination cases and the derivation of the touch band.
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

/* touch band width in units of the rounding noise eps * L (see ds_distance.c) */
#define GJK_TOUCH_TOLERANCE 100.0f

f32     GJK(v3 *c_a, v3 *c_b, v3 *n, struct GJKCache *cache_out, const struct GJKCache *cache_in, const struct c_Shape *shape_a, const ds_Transform *t_a, const struct c_Shape *shape_b, const ds_Transform *t_b, const f32 cutoff_distance);

f32     c_SphereDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_CapsuleSphereDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_CapsuleDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_HullSphereDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_HullCapsuleDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_HullDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_TriMeshBvhSphereDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_TriMeshBvhCapsuleDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_TriMeshBvhHullDistance(v3 *c1, v3 *c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);

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

f32 c_SphereRaycastParameter(const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray);
f32 c_CapsuleRaycastParameter(const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray);
f32 c_HullRaycastParameter(const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray);
f32 c_TriMeshBvhRaycastParameter(const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray);

#ifdef __cplusplus
} 
#endif

#endif
