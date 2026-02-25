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
#include "ds_vector.h"
#include "queue.h"
#include "tree.h"

struct ds_RigidBodyPipeline;

#define COLLISION_DEFAULT_MARGIN	(100.0f * F32_EPSILON)
#define COLLISION_POINT_DIST_SQ		(10000.0f * F32_EPSILON)

/*
bounding volume hierarchy
=========================
*/

struct bvhNode
{
	BT_SLOT_STATE;
	struct aabb	bbox;
};

struct bvh
{
	struct bt		tree;
	struct minQueue	cost_queue;	/* dynamic specific */
	u32			heap_allocated;
};

/* free allocated resources */
void 		BvhFree(struct bvh *tree);
/* validate (ds_Assert) internal coherence of bvh */
void 		BvhValidate(struct arena *tmp, const struct bvh *bvh);
/* return total cost of bvh */
f32 		BvhCost(const struct bvh *bvh);

#define COST_QUEUE_INITIAL_COUNT 	64 

//TODO remove
struct dbvhOverlap
{
	u32 id1;
	u32 id2;	
};

struct bvh		DbvhAlloc(struct arena *mem, const u32 initial_length, const u32 growable);
/* flush / reset the hierarchy  */
void 			DbvhFlush(struct bvh *bvh);
/* id is an integer identifier from the outside, return index of added value */
u32 			DbvhInsert(struct bvh *bvh, const u32 id, const struct aabb *bbox);
/* remove leaf corresponding to index from tree */
void 			DbvhRemove(struct bvh *bvh, const u32 index);
/* Return overlapping ids ptr, set to NULL if no overlap. if overlap, count is set */
struct dbvhOverlap *	DbvhPushOverlapPairs(struct arena *mem, u32 *count, const struct bvh *bvh);
/* push	id:s of leaves hit by raycast. returns number of hits. -1 == out of memory */

struct triMeshBvh
{
	const struct triMesh *	mesh;		
	struct bvh		bvh;
	u32 *			tri;		
	u32			tri_count;	
};

/* Return non-empty tri_mesh_bvh on success. */
struct triMeshBvh 	TriMeshBvhConstruct(struct arena *mem, const struct triMesh *mesh, const u32 bin_count);
/* Return (index, ray hit parameter) on closest hit, or (U32_MAX, F32_INFINITY) on no hit */
u32f32 			TriMeshBvhRaycast(struct arena *tmp, const struct triMeshBvh *mesh_bvh, const struct ray *ray);


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
void 			BvhRaycastTestAndPushChildren(struct bvhRaycastInfo *info, const u32f32 popped_tuple);


/********************************** COLLISION DEBUG **********************************/

typedef struct visualSegment
{
	struct segment	segment;
	vec4		color;
} visualSegment;
DECLARE_STACK(visualSegment);

struct visualSegment	VisualSegmentConstruct(const struct segment segment, const vec4 color);

struct collisionDebug
{
	stack_visualSegment	stack_segment;
	u8			pad[64];
};

extern dsThreadLocal struct collisionDebug *tl_debug;

#ifdef DS_PHYSICS_DEBUG

#define COLLISION_DEBUG_ADD_SEGMENT(segment, color)							\
	stack_visualSegmentPush(&tl_debug->stack_segment,  VisualSegmentConstruct(segment, color))

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

struct c_Shape
{
	STRING_DATABASE_SLOT_STATE;
	
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

void	CollisionShapeUpdateMassProperties(struct c_Shape *shape);


enum collisionResultType
{
	COLLISION_NONE,		/* No collision, no sat cache stored */
	COLLISION_SAT_CACHE,	/* No collision, sat cache stored    */
	COLLISION_CONTACT,	/* Collision, sat cache stored       */
	COLLISION_COUNT
};

struct contactManifold
{
	vec3 	v[4];
	f32 	depth[4];
	vec3 	n;		/* B1 -> B2 */
	u32 	v_count;
	u32 	i1;
	u32 	i2;
};

enum sat_CacheType
{
	SAT_CACHE_SEPARATION,
	SAT_CACHE_CONTACT_FV,
	SAT_CACHE_CONTACT_EE,
	SAT_CACHE_COUNT,
};

struct sat_Cache
{
	POOL_SLOT_STATE;
	u32	touched;
	DLL_SLOT_STATE;

	enum sat_CacheType	type;
	union
	{
		struct
		{
			u32	body;	/* body (0,1) containing face */
			u32	face;	/* reference face 	      */
		};

		struct
		{
			u32	edge1;	/* body0 edge, body0 < body1 */
			u32	edge2;	/* body1 edge                */
		};

		struct
		{
			vec3    separation_axis;
			f32	    separation;
		};
	};

	u64	key;
};

struct c_Result
{
	enum collisionResultType	type;
	struct sat_Cache			sat_cache;
	struct contactManifold 		manifold;
};

void 	ContactManifoldDebugPrint(FILE *file, const struct contactManifold *cm);

/********************************** INTERSECTION TESTS **********************************/

u32     c_SphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_CapsuleSphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_CapsuleTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_HullSphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_HullCapsuleTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_HullTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_TriMeshBvhSphereTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_TriMeshBvhCapsuleTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_TriMeshBvhHullTest(const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);

/********************************** DISTANCE METHODS **********************************/

f32     c_SphereDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
f32     c_CapsuleSphereDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
f32     c_CapsuleDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
f32     c_HullSphereDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
f32     c_HullCapsuleDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
f32     c_HullDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
f32     c_TriMeshBvhSphereDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
f32     c_TriMeshBvhCapsuleDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
f32     c_TriMeshBvhHullDistance(vec3 c1, vec3 c2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);

/********************************** CONTACT MANIFOLD METHODS **********************************/

u32     c_SphereContact(struct arena *not_used1, struct c_Result *result, const struct ds_RigidBodyPipeline *not_used2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_CapsuleSphereContact(struct arena *not_used1, struct c_Result *result, const struct ds_RigidBodyPipeline *not_used2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_CapsuleContact(struct arena *not_used1, struct c_Result *result, const struct ds_RigidBodyPipeline *not_used2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_HullSphereContact(struct arena *not_used1, struct c_Result *result, const struct ds_RigidBodyPipeline *not_used2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_HullCapsuleContact(struct arena *not_used1, struct c_Result *result, const struct ds_RigidBodyPipeline *not_used2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_HullContact(struct arena *tmp, struct c_Result *result, const struct ds_RigidBodyPipeline *pipeline, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_TriMeshBvhSphereContact(struct arena *tmp, struct c_Result *result, const struct ds_RigidBodyPipeline *not_used2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_TriMeshBvhCapsuleContact(struct arena *tmp, struct c_Result *result, const struct ds_RigidBodyPipeline *not_used2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);
u32     c_TriMeshBvhHullContact(struct arena *tmp, struct c_Result *result, const struct ds_RigidBodyPipeline *not_used2, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 margin);

/********************************** RAYCAST **********************************/

f32 c_SphereRaycastParameter(struct arena *not_used, const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray);
f32 c_CapsuleRaycastParameter(struct arena *not_used, const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray);
f32 c_HullRaycastParameter(struct arena *not_used, const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray);
f32 c_TriMeshBvhRaycastParameter(struct arena *mem_tmp, const struct c_Shape *shape, const ds_Transform *transform, const struct ray *ray);




struct ds_RigidBodyPipeline;
struct ds_RigidBody;

/* Test for intersection between shapes, with each shape having the given margin. returns 1 if intersecting, else 0 */
u32	    BodyBodyTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin);
/* Return, if no intersection was found, the distance between shapes s1 and s2 (with no margin) and their 
 * respective closest points c1 and c2. If the shapes are intersecting, return 0.0f. */
f32 	BodyBodyDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin);
/* Returns 1 if the shapes are colliding, 0 otherwise. If a collision is found, return a contact manifold
 * or sat cache pointing from s1 to s2 */
u32 	BodyBodyContactManifold(struct arena *tmp, struct c_Result *result, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b1, const struct ds_RigidBody *b2, const f32 margin);
/* Return t such that ray.origin + t*ray.dir == closest point on shape */
f32 	BodyRaycastParameter(const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b, const struct ray *ray);
/* Return 1 if ray hit shape, 0 otherwise. If hit, we return the closest intersection point */
u32 	BodyRaycast(vec3 intersection, const struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBody *b, const struct ray *ray);




#ifdef __cplusplus
} 
#endif

#endif
