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

#define COLLISION_DEFAULT_MARGIN	(100.0f * F32_EPSILON)
#define COLLISION_POINT_DIST_SQ		(10000.0f * F32_EPSILON)

/*
bounding volume hierarchy
=========================
*/

struct bvhNode
{
    /* If leaf, bt_right == body_index, bt_left == shape index */
	BT_SLOT_STATE;
	struct aabb bbox;
};

struct bvh
{
	struct bt		tree;
	struct minQueue	cost_queue;	/* dynamic specific */
	u32			    heap_allocated;
};

/* free allocated resources */
void 		        BvhFree(struct bvh *tree);
/* validate (ds_Assert) internal coherence of bvh */
void 		        BvhValidate(struct arena *tmp, const struct bvh *bvh);
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


//TODO remove
struct dbvhOverlap
{
	u32 id1;
	u32 id2;	
};
/* (DEPRECATED): Return overlapping ids ptr, set to NULL if no overlap. if overlap, count is set */
struct dbvhOverlap *DbvhPushOverlapPairs(struct arena *mem, u32 *count, const struct bvh *bvh);


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
	vec3 			multiplier;
	vec3u32 		dir_sign_bit;
	struct minQueueFixed	hit_queue;
	const struct ray *	ray;
	const struct bvh *	bvh;
	const struct bvhNode *	node;
};

/* Initiate raycast information */
struct bvhRaycastInfo	BvhRaycastInit(struct arena *mem, const struct bvh *bvh, const struct ray *ray);
/* test Raycasting against child nodes and push hit children onto queue */
void 			        BvhRaycastTestAndPushChildren(struct bvhRaycastInfo *info, const u32f32 popped_tuple);


/********************************** COLLISION DEBUG **********************************/

typedef struct visualSegment
{
	struct segment	segment;
	vec4		color;
} visualSegment;
DEFINE_CPOOL_STRUCT(visualSegment);

struct visualSegment	VisualSegmentConstruct(const struct segment segment, const vec4 color);

struct collisionDebug
{
	ds_CPool(visualSegment)	stack_segment;
	u8			            pad[64];
};

extern struct collisionDebug *g_collision_debug;

#ifdef DS_PHYSICS_DEBUG

#define COLLISION_DEBUG_ADD_SEGMENT(segment, color)							\
	ds_CPoolPushValue(g_collision_debug[ds_ThreadSelfIndex()].stack_segment,  VisualSegmentConstruct(segment, color))

#else

#define COLLISION_DEBUG_ADD_SEGMENT(segment, color)

#endif

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
	
	mat3	                inertia_tensor;		/* local shape frame intertia tensor (Assumes density=1.0, 
			                		                to get the interia tensor given a density, just multiply
			                		                the matrix with the given density. */
	vec3	                center_of_mass;		/* local shape frame center of mass */
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
	vec3 	v[4];       /* contact point on the reference shape surface     */
	f32 	depth[4];   /* Contact point penetration depth (0.0f, INFINITY) */
	vec3 	n;		    /* Contact normal: Points away from reference       */
	u32 	v_count;    /* Contact point count                              */
};

/* Print manifold to file stderr */
void 	c_ManifoldDebugPrint(const struct c_Manifold *cm);
/* Sanity tests for debugging. Return 1 if valid, 0 otherwise. */
u32     c_ManifoldCheck(const struct c_Manifold *cm);
/* Transform the manifold */
void    c_ManifoldTransform(struct c_Manifold *dst, const struct c_Manifold *src, mat3 rot, const vec3 translation);

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

f32     c_SphereDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_CapsuleSphereDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_CapsuleDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_HullSphereDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_HullCapsuleDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_HullDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_TriMeshBvhSphereDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_TriMeshBvhCapsuleDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);
f32     c_TriMeshBvhHullDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2);

/********************************** CONTACT MANIFOLD METHODS **********************************/

struct c_ContactResult
{
    u32                         manifold_count;     /* Number of stored manifolds               */
    u32                         cache_count;        /* Number of stored caches                  */
    struct c_Manifold *         manifold;           /* Manifolds (if any)                       */
    struct c_SatCache *         cache;              /* Caches (if any)                          */
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
