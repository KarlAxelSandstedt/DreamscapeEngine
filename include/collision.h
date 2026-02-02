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

enum collisionShapeType
{
	COLLISION_SHAPE_SPHERE,
	COLLISION_SHAPE_CAPSULE,
	COLLISION_SHAPE_CONVEX_HULL,
	COLLISION_SHAPE_TRI_MESH,	
	COLLISION_SHAPE_COUNT,
};

struct collisionShape
{
	STRING_DATABASE_SLOT_STATE;
	u32 			center_of_mass_localized; /* Has the shape translated its vertices into COM space? */
	enum collisionShapeType type;
	union
	{
		struct sphere 		sphere;
		struct capsule 		capsule;
		struct dcel		hull;
		struct triMeshBvh 	mesh_bvh;
	};
};

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

enum satCacheType
{
	SAT_CACHE_SEPARATION,
	SAT_CACHE_CONTACT_FV,
	SAT_CACHE_CONTACT_EE,
	SAT_CACHE_COUNT,
};

struct satCache
{
	POOL_SLOT_STATE;
	u32	touched;
	DLL_SLOT_STATE;

	enum satCacheType	type;
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
			vec3	separation_axis;
			f32	separation;
		};
	};

	u64	key;
};

struct collisionResult
{
	enum collisionResultType	type;
	struct satCache			sat_cache;
	struct contactManifold 		manifold;
};

void 	ContactManifoldDebugPrint(FILE *file, const struct contactManifold *cm);

/********************************** RIGID BODY METHODS  **********************************/

struct physicsPipeline;
struct rigidBody;

/* test for intersection between bodies, with each body having the given margin. returns 1 if intersection. */
u32	BodyBodyTest(const struct physicsPipeline *pipeline, const struct rigidBody *b1, const struct rigidBody *b2, const f32 margin);
/* return closest points c1 and c2 on bodies b1 and b2 (with no margin), respectively, given no intersection */
f32 	BodyBodyDistance(vec3 c1, vec3 c2, const struct physicsPipeline *pipeline, const struct rigidBody *b1, const struct rigidBody *b2, const f32 margin);
/* returns contact manifold or sat cache pointing from b1 to b2, given that the bodies are colliding  */
u32 	BodyBodyContactManifold(struct arena *tmp, struct collisionResult *result, const struct physicsPipeline *pipeline, const struct rigidBody *b1, const struct rigidBody *b2, const f32 margin);
/* Return t such that ray.origin + t*ray.dir == closest point on rigid body */
f32 	BodyRaycastParameter(const struct physicsPipeline *pipeline, const struct rigidBody *b, const struct ray *ray);
/* Return 1 if ray hit body, 0 otherwise. If hit, we return the closest intersection point */
u32 	BodyRaycast(vec3 intersection, const struct physicsPipeline *pipeline, const struct rigidBody *b, const struct ray *ray);

#ifdef __cplusplus
} 
#endif

#endif
