/*
==========================================================================
    Copyright (C) 2025 Axel Sandstedt 

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

#ifndef __COLLISION_H__
#define __COLLISION_H__

#include "ds_common.h"
#include "allocator.h"
#include "ds_math.h"
#include "geometry.h"
#include "sys_public.h"
#include "string_database.h"
#include "float32.h"
#include "queue.h"
#include "tree.h"

#define COLLISION_DEFAULT_MARGIN	(100.0f * F32_EPSILON)
#define COLLISION_POINT_DIST_SQ		(10000.0f * F32_EPSILON)

/*
bounding volume hierarchy
=========================
*/

struct bvh_node
{
	BT_SLOT_STATE;
	struct AABB	bbox;
};

struct bvh
{
	struct bt		tree;
	struct minQueue	cost_queue;	/* dynamic specific */
	u32			heap_allocated;
};

/* free allocated resources */
void 		bvh_free(struct bvh *tree);
/* validate (ds_Assert) internal coherence of bvh */
void 		bvh_validate(struct arena *tmp, const struct bvh *bvh);
/* return total cost of bvh */
f32 		bvh_cost(const struct bvh *bvh);

#define COST_QUEUE_INITIAL_COUNT 	64 

//TODO remove
struct dbvh_overlap
{
	u32 id1;
	u32 id2;	
};

struct bvh		dbvh_alloc(struct arena *mem, const u32 initial_length, const u32 growable);
/* flush / reset the hierarchy  */
void 			dbvh_flush(struct bvh *bvh);
/* id is an integer identifier from the outside, return index of added value */
u32 			dbvh_insert(struct bvh *bvh, const u32 id, const struct AABB *bbox);
/* remove leaf corresponding to index from tree */
void 			dbvh_remove(struct bvh *bvh, const u32 index);
/* Return overlapping ids ptr, set to NULL if no overlap. if overlap, count is set */
struct dbvh_overlap *	dbvh_push_overlap_pairs(struct arena *mem, u32 *count, const struct bvh *bvh);
/* push	id:s of leaves hit by raycast. returns number of hits. -1 == out of memory */

struct tri_mesh_bvh
{
	const struct tri_mesh *	mesh;		
	struct bvh		bvh;
	u32 *			tri;		
	u32			tri_count;	
};

/* Return non-empty tri_mesh_bvh on success. */
struct tri_mesh_bvh 	tri_mesh_bvh_construct(struct arena *mem, const struct tri_mesh *mesh, const u32 bin_count);
/* Return (index, ray hit parameter) on closest hit, or (U32_MAX, F32_INFINITY) on no hit */
u32f32 			tri_mesh_bvh_raycast(struct arena *tmp, const struct tri_mesh_bvh *mesh_bvh, const struct ray *ray);


/*
bvh raycasting
==============
To implement raycast using external primitives, one can use the following code:

	ArenaPushRecord(mem);

	struct bvh_raycast_info info = bvh_raycast_init(mem, bvh, ray);
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
			bvh_raycast_test_and_push_children(&info, tuple);
		}
	}

	ArenaPopRecord(mem);
*/
struct bvh_raycast_info
{
	u32f32			hit;
	vec3 			multiplier;
	vec3u32 		dir_sign_bit;
	struct minQueueFixed	hit_queue;
	const struct ray *	ray;
	const struct bvh *	bvh;
	const struct bvh_node *	node;
};

/* Initiate raycast information */
struct bvh_raycast_info	bvh_raycast_init(struct arena *mem, const struct bvh *bvh, const struct ray *ray);
/* test Raycasting against child nodes and push hit children onto queue */
void 			bvh_raycast_test_and_push_children(struct bvh_raycast_info *info, const u32f32 popped_tuple);


/********************************** COLLISION DEBUG **********************************/

typedef struct visual_segment
{
	struct segment	segment;
	vec4		color;
} visual_segment;
DECLARE_STACK(visual_segment);

struct visual_segment	visual_segment_construct(const struct segment segment, const vec4 color);

struct collision_debug
{
	stack_visual_segment	stack_segment;

	u8			pad[64];
};

extern dsThreadLocal struct collision_debug *tl_debug;

#ifdef DS_PHYSICS_DEBUG

#define COLLISION_DEBUG_ADD_SEGMENT(segment, color)							\
	stack_visual_segment_push(&tl_debug->stack_segment,  visual_segment_construct(segment, color))

#else

#define COLLISION_DEBUG_ADD_SEGMENT(segment, color)

#endif

/********************************** COLLISION SHAPES **********************************/

enum collision_shape_type
{
	COLLISION_SHAPE_SPHERE,
	COLLISION_SHAPE_CAPSULE,
	COLLISION_SHAPE_CONVEX_HULL,
	COLLISION_SHAPE_TRI_MESH,	
	COLLISION_SHAPE_COUNT,
};

struct collision_shape
{
	STRING_DATABASE_SLOT_STATE;
	u32 			center_of_mass_localized; /* Has the shape translated its vertices into COM space? */
	enum collision_shape_type type;
	union
	{
		struct sphere 		sphere;
		struct capsule 		capsule;
		struct dcel		hull;
		struct tri_mesh_bvh 	mesh_bvh;
	};
};

enum collision_result_type
{
	COLLISION_NONE,		/* No collision, no sat cache stored */
	COLLISION_SAT_CACHE,	/* No collision, sat cache stored    */
	COLLISION_CONTACT,	/* Collision, sat cache stored       */
	COLLISION_COUNT
};

struct contact_manifold
{
	vec3 	v[4];
	f32 	depth[4];
	vec3 	n;		/* B1 -> B2 */
	u32 	v_count;
	u32 	i1;
	u32 	i2;
};

enum sat_cache_type
{
	SAT_CACHE_SEPARATION,
	SAT_CACHE_CONTACT_FV,
	SAT_CACHE_CONTACT_EE,
	SAT_CACHE_COUNT,
};

struct sat_cache
{
	POOL_SLOT_STATE;
	u32	touched;
	DLL_SLOT_STATE;

	enum sat_cache_type	type;
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

struct collision_result
{
	enum collision_result_type	type;
	struct sat_cache		sat_cache;
	struct contact_manifold 	manifold;
};

void 	contact_manifold_debug_print(FILE *file, const struct contact_manifold *cm);

/********************************** RIGID BODY METHODS  **********************************/

struct physics_pipeline;
struct rigid_body;

/* test for intersection between bodies, with each body having the given margin. returns 1 if intersection. */
u32	body_body_test(const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin);
/* return closest points c1 and c2 on bodies b1 and b2 (with no margin), respectively, given no intersection */
f32 	body_body_distance(vec3 c1, vec3 c2, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin);
/* returns contact manifold or sat cache pointing from b1 to b2, given that the bodies are colliding  */
u32 	body_body_contact_manifold(struct arena *tmp, struct collision_result *result, const struct physics_pipeline *pipeline, const struct rigid_body *b1, const struct rigid_body *b2, const f32 margin);
/* Return t such that ray.origin + t*ray.dir == closest point on rigid body */
f32 	body_raycast_parameter(const struct physics_pipeline *pipeline, const struct rigid_body *b, const struct ray *ray);
/* Return 1 if ray hit body, 0 otherwise. If hit, we return the closest intersection point */
u32 	body_raycast(vec3 intersection, const struct physics_pipeline *pipeline, const struct rigid_body *b, const struct ray *ray);

#endif
