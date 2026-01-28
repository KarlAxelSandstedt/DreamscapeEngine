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

#ifndef __DS_DYNAMICS_H__
#define __DS_DYNAMICS_H__

#include "ds_common.h"
#include "allocator.h"
#include "list.h"
#include "collision.h"
#include "hash_map.h"
#include "bitVector.h"
#include "array_list.h"
#include "ds_math.h"

struct rigid_body;
struct physics_pipeline;

/*
=================================================================================================================
|						Contact Database						|
=================================================================================================================
*/

struct contact
{
	NLL_SLOT_STATE;
	struct contact_manifold cm;
	u64 			key;

	vec3 			normal_cache;
	vec3 			tangent_cache[2];
	vec3 			v_cache[4];			/* previous contact manifold vertices, 
								   or { F32_MAX, F32_MAX, F32_MAX }  */
	f32 			tangent_impulse_cache[4][2];
	f32 			normal_impulse_cache[4];	/* contact_solver solution to contact constraint, 
								   or 0.0f */
	u32 			cached_count;			/* number of vertices in cache */
};

/*
contact_database
================
Database for last and current frame contacts. Any rigid body can lookup its cached
and current contacts, and if necessary, invalidate any contact data.

Frame layout:
	1. generate_contacts
 	2. c_db_new_frame(contact_count)	// alloc memory for frame contacts
 	3. c_db_add_contact(i1, i2, contact)	// add all new contacts 
 	4. solve
 	5. invalidate any contacts before caching them.
 	6. switch frame and cache 
 	7. reset frame
*/

struct contact_database
{
	/*
	 * contact net list nodes are owned as follows:
	 *
	 * contact->key & (0xffffffff00000000) >> 32 identifier owns slot 0
	 * contact->key & (0x00000000ffffffff) >>  0 identifier owns slot 1
	 *
	 * i.e. the smaller index owns slot 0 and the larger index owns slot 1.
	 */
	struct nll	contact_net;
	struct hashMap *contact_map;		

	/*
	 * frame-cached separation axes 
	 */
	struct pool	sat_cache_pool;
	struct dll	sat_cache_list;
	struct hashMap *sat_cache_map;		

	/* PERSISTENT DATA, GROWABLE, keeps track of which slots in contacts are currently being used. */
	struct bitVec 	contacts_persistent_usage; /* At end of frame, is set to contacts_frame_usage + any 
						     new appended contacts resulting in appending the 
						     contacts array.  */

	/* FRAME DATA, NOT GROWABLE, keeps track of which slots in contacts are currently being used. */
	struct bitVec 	contacts_frame_usage;	/* bit-array showing which of the previous frame link indices
						   are reused. Thus, all links in the current frame are the
						   ones in the bit array + any appended contacts which resu-
						   -lted in growing the array. */
};

#define CONTACT_KEY_TO_BODY_0(key) 	((key) >> 32)
#define CONTACT_KEY_TO_BODY_1(key) 	((key) & U32_MAX)

struct contact_database c_db_alloc(struct arena *mem_persistent, const u32 initial_size);
void 			c_db_free(struct contact_database *c_db);
void			c_db_flush(struct contact_database *c_db);
void			c_db_validate(const struct physics_pipeline *pipeline);
void			c_db_clear_frame(struct contact_database *c_db);
/* Update or add new contact depending on if the contact persisted from prevous frame. */
struct contact *	c_db_add_contact(struct physics_pipeline *pipeline, const struct contact_manifold *cm, const u32 i1, const u32 i2);
void 			c_db_remove_contact(struct physics_pipeline *pipeline, const u64 key, const u32 index);
/* Remove all contacts associated with the given body */
void			c_db_remove_body_contacts(struct physics_pipeline *pipeline, const u32 body_index);
/* Remove all contacts associated with the given static body and store affected islands */
u32 *			c_db_remove_static_contacts_and_store_affected_islands(struct arena *mem, u32 *count, struct physics_pipeline *pipeline, const u32 static_index);
struct contact *	c_db_lookup_contact(const struct contact_database *c_db, const u32 b1, const u32 b2);
u32 			c_db_lookup_contact_index(const struct contact_database *c_db, const u32 i1, const u32 i2);
void 			c_db_update_persistent_contacts_usage(struct contact_database *c_db);

/* add sat_cache to pipeline; if it already exists, reset the cache. */
void 			sat_cache_add(struct contact_database *c_db, const struct sat_cache *sat_cache);
/* lookup sat_cache to pipeline; if it does't exist, return NULL. */
struct sat_cache *	sat_cache_lookup(const struct contact_database *c_db, const u32 b1, const u32 b2);

/*
=================================================================================================================
|						Persistent Islands						|
=================================================================================================================

island
======
Persistent island over several frames. Justification is that island information may possibly not change much from
frame to frame, so storing persistent island data may work as an optimization.  It would also be of help in storing
cached collision/body data between frames.

Operations:
	(1) island_initialize(body)	- Initalize new island from a body (valid for being in an island)
	(2) island_split()		- We must be able to split an island no longer fully connected
	(3) island_merge() 		- We must be able to merge two islands now connected

Auxilliary Operation:
	(1)	contact_cache_get_persistent_contacts()	
	(2)	contact_cache_get_new_contacts() 
  	(3)	contact_cache_get_deleted_contacts()


----- Island Consistency: Knowing when to split, and when to merge -----

In order to know that we should split an island, or merge two islands, we must have ways to reason about
the connectivity of islands. The physics pipeline ensure that islands are valid at the start of frames,
except perhaps for the first frame. The frame layout should look something like:

	[1] solve island local system
		(1) We may now have broken islands
	[2] finalize bodies, cache contact data  
		(1) Islands contain up-to-date information and caches for bodies (which may no longer be connected) 
		(2) if (cache_map.entry[i] == no_update) => Connection corresponding to entry i no longer exists
	[3] construct new contact_data
		(2) if (cache_map(contact) ==    hit) => Connection remains, (possibly in a new island)
		(3) if (cache_map(contact) == no_hit) => A new connection has been established, 
						         (possibly between two islands)
       [4] update/construct islands

It follows that if we keep track of 

       (1) what contacts were removed from the contact_cache	- deleted links
       (2) what contacts were added to the contact cache	- new links
       (3) what contacts remain in the contact cache 		- persistent links

we have all the sufficient (and necessary) information to re-establish the invariant of correct islands at the
next frame.


----- Island Memory: Handling Lifetimes and Memory Layouts (Sanely) -----

The issue with persistent islands is that the lifetime of the island is not (generally) shared with to bodies
it rules over. It would be possible to limit the islands to using linked lists if we ideally only would have 
to iterate each list once. This would greatly simplify the memory management. We consider what data must be
delivered to and from the island at what time:


FRAME n: 	...
        	...
	==== Contact Cache ====
	[3, 4] construct new contact data + update/construct islands
		list of body indices 		=> island
		list of constraint indices 	=> island

FRAME n+1:
	==== Island ====
	[1] solve island local system
		island.constraints.data		=> solver
		island.bodies data		=> solver

	==== Solver ====
     [2] finalize bodies, cache contact data
       	solve.solution			=> contact cache
       	cache constraints		=> contact cache


Assuming that the island only contains linked lists of indices to various data, we wish to fully defer any
lookups into that data until the Solver stage. [1] We traverse the lists and retrieve the wanted data. This
data (ListData) is kept throughout [2], [3], and discarded at [4] when islands are split/merged.
*/

#define BODY_NO_ISLAND_INDEX 	U32_MAX

#define ISLAND_AWAKE		(0x1u << 0u)
#define ISLAND_SLEEP_RESET	(0x1u << 1u)	/* reset sleep timers on frame */
#define ISLAND_SPLIT		(0x1u << 2u)	/* flag island for splitting */
#define ISLAND_TRY_SLEEP	(0x1u << 3u)	/* flag island for being put to sleep at next solve iteration 
						 * (if the island is uninterrupted for a frame) This is needed
						 * since if we determine that an updated island should be put
						 * to sleep at end of a frame in island_solve, we must atleast
						 * update all rigid body proxies before butting the bodies to
						 * sleep as well, so keep the island awake for another frame
						 * without solving it at the end if it is uninterrupted.
						 */

#define ISLAND_AWAKE_BIT(is)		(((is)->flags & ISLAND_AWAKE) >> 0u)
#define ISLAND_SLEEP_RESET_BIT(is)	(((is)->flags & ISLAND_SLEEP_RESET) >> 1u)
#define ISLAND_SPLIT_BIT(is)		(((is)->flags & ISLAND_SPLIT) >> 2u)
#define ISLAND_TRY_SLEEP_BIT(is)	(((is)->flags & ISLAND_TRY_SLEEP) >> 3u)

#define ISLAND_NULL	U32_MAX 
#define ISLAND_STATIC	U32_MAX-1	/* static bodies are mapped to "island" ISLAND_STATIC */

struct island
{
	struct rigid_body **	bodies;	
	struct contact 	**	contacts;
	u32 *			body_index_map; /* body_index -> local indices of bodies in island:
						 * is->bodies[i] = pipeline->bodies[b] => 
						 * is->body_index_map[b] = i 
						 */

	//TODO REMOVE
	u32 cm_count;

	/* Persistent Island */
	u32 flags;

	u32 body_first;			/* index into first node in island_body_lists 		*/
	u32 contact_first;		/* index into first node in island_contact_lists 	*/

	u32 body_last;			/* index into last node in island_body_lists 		*/
	u32 contact_last;		/* index into last node in island_contact_lists 	*/

	u32 body_count;
	u32 contact_count;

#ifdef DS_PHYSICS_DEBUG
	vec4 color;
#endif
};

struct is_index_entry
{
	u32 index;
	u32 next;
};

struct island_database
{
	/* PERSISTENT DATA */
	struct bitVec island_usage;			/* NOT GROWABLE, bit vector for islands in use */
	struct array_list *islands;			/* NOT GROWABLE, set to max_body_count 	*/
	struct array_list *island_contact_lists;	/* GROWABLE, list nodes to contacts 	*/
	struct array_list *island_body_lists;		/* NOT GROWABLE, list nodes to bodies   */

	/* FRAME DATA */
	u32 *possible_splits;				/* Islands in which a contact has been broken during frame */
	u32 possible_splits_count;
};

#ifdef DS_PHYSICS_DEBUG
#define IS_DB_VALIDATE(pipeline)	is_db_validate((pipeline)
#else
#define IS_DB_VALIDATE(pipeline)	
#endif

struct physics_pipeline;

/* Setup and allocate memory for new database */
struct island_database 	is_db_alloc(struct arena *mem_persistent, const u32 initial_size);
/* Free any heap memory */
void		       	is_db_free(struct island_database *is_db);
/* Flush / reset the island database */
void			is_db_flush(struct island_database *is_db);
/* Clear any frame related data */
void			is_db_clear_frame(struct island_database *is_db);
/* remove island resources from database */
void 			is_db_island_remove(struct physics_pipeline *pipeline, struct island *is);
/* remove island resources related to body, and possibly the whole island, from database */
void 			is_db_island_remove_body_resources(struct physics_pipeline *pipeline, const u32 island_index, const u32 body);
/* Debug printing of island */
void is_db_print_island(FILE *file, const struct island_database *is_db, const struct contact_database *c_db, const u32 island, const char *desc);
/* Check if the database appears to be valid */
void 			is_db_validate(const struct physics_pipeline *pipeline);
/* Setup new island from single body */
struct island *		is_db_init_island_from_body(struct physics_pipeline *pipeline, const u32 body);
/* Add contact to island */
void 			is_db_add_contact_to_island(struct island_database *is_db, const u32 island, const u32 contact);
/* Return island that body is assigned to */
struct island *		is_db_body_to_island(struct physics_pipeline *pipeline, const u32 body);
/* Reserve enough memory to fit all possible split */
void			is_db_reserve_splits_memory(struct arena *mem_frame, struct island_database *is_db);
/* Release any unused reserved possible split memory */
void			is_db_release_unused_splits_memory(struct arena *mem_frame, struct island_database *is_db);
/* Tag the island that the body is in for splitting and push it onto split memory (if we havn't already) */
void 			is_db_tag_for_splitting(struct physics_pipeline *pipeline, const u32 body);
/* Merge islands (Or simply update if new local contact) using new contact */
void 			is_db_merge_islands(struct physics_pipeline *pipeline, const u32 ci, const u32 b1, const u32 b2);
/* Split island, or remake if no split happens: TODO: Make thread-safe  */
void 			is_db_split_island(struct arena *mem_tmp, struct physics_pipeline *pipeline, const u32 island_to_split);
/* Solve island constraints, and update bodies in pipeline */
//void 			island_solve(struct arena *mem_frame, struct physics_pipeline *pipeline, struct island *is, const f32 timestep);

/********* Threaded Island API *********/

struct island_solve_output
{
	u32 island;
	u32 island_asleep;
	u32 body_count;
	u32 *bodies;		/* bodies simulated in island */ 
	struct island_solve_output *next;
};

struct island_solve_input
{
	struct island *is;
	struct physics_pipeline *pipeline;
	struct island_solve_output *out;
	f32 timestep;
};

/*
 * Input: struct island_solve_in 
 * Output: struct island_solve_output
 *
 * Solves the given island using the global solver config. Since no island shares any contacts or bodies, and every
 * island is a unique task, no shared variables are being written to.
 *
 * - reads pipeline, solver config, c_db, is_db (basically everything)
 * - writes to island,		(unique to thread, memory in c_db)
 * - writes to island->contacts (unique to thread, memory in c_db)
 * - writes to island->bodies	(unique to thread, memory in pipeline)
 */
void	thread_island_solve(void *task_input);

/*
=================================================================================================================
|						Contact Solver				  	      	    	|
=================================================================================================================

contact_solver_config
=====================
Mumerical parameters configuration for solving islands.
*/

/*
 * Implementation of ([Iterative Dynamics with Temporal Coherence], Erin Catto, 2005) and
 * Box2D features.
 *
 * Planned Features:
 * (O) Block Solver
 * (O) Sleeping islands
 * (O) Friction Solver
 * () warmup impulse for contact points
 * (O) g_solver_config dampening constants (linear and angular)
 * (O) velocity biases: baumgarte bias linear slop (allowed position error we correct for, see def. in https://allenchou.net/2013/12/game-physics-resolution-contact-constraints/)
 * (O) Resitution base contacts [bounciness of objects, added to velocity bias in velocity constraint solver given
 * 	restitution threshold, see def. in https://allenchou.net/2013/12/game-physics-resolution-contact-constraints/
 * 	and box2D
 * () threshold for forces
 * (O) conditioning number of normal mass, must ensure stability.
 */
struct contact_solver_config
{
	u32 	iteration_count;	/* velocity solver iteration count */
	u32 	block_solver;		/* bool : Use block solver when applicable */
	u32 	warmup_solver;		/* bool : Should warmup solver when applicable */
	vec3 	gravity;
	f32 	baumgarte_constant;  	/* Range[0.0, 1.0] : Determine how quickly contacts are resolved, 1.0f max 
					   speed */
	f32 	max_condition;		/* max condition number of block normal mass */
	f32 	linear_dampening;	/* Range[0.0, inf] : coefficient in diff. eq. dv/dt = -coeff*v */
	f32 	angular_dampening;	/* Range[0.0, inf] : coefficient in diff. eq. dv/dt = -coeff*v */
	f32 	linear_slop;		/* Range[0.0, inf] : Allowed penetration before velocity steering gradually
					   sets in. */
	f32 restitution_threshold; 	/* Range[0.0, inf] : If -seperating_velocity >= threshold, we apply the 
					   restitution effect */

	u32 sleep_enabled;		/* bool : enable sleeping of bodies  */
	f32 sleep_time_threshold; /* Range(0.0, inf] :  Time threshold for which a body must have low velocity before being able to fall asleep */
	f32 sleep_linear_velocity_sq_limit; /* Range (0.0f, inf] : maximum linear velocity squared that a body falling asleep may have */
	f32 sleep_angular_velocity_sq_limit; /* Range (0.0f, inf] : maximum angular velocity squared that a body falling asleep may have */

	/* Pending updates */
	u32 pending_block_solver;		
	u32 pending_warmup_solver;		
	u32 pending_sleep_enabled;		
	u32 pending_iteration_count;
	f32 pending_baumgarte_constant;
	f32 pending_linear_slop;
	f32 pending_restitution_threshold;
	f32 pending_linear_dampening;
	f32 pending_angular_dampening;
};

extern struct contact_solver_config *g_solver_config;

void	contact_solver_config_init(const u32 iteration_count, const u32 block_solver, const u32 warmup_solver, const vec3 gravity, const f32 baumgarte_constant, const f32 max_condition, const f32 linear_dampening, const f32 angular_dampening, const f32 linear_slop, const f32 restitution_threshold, const u32 sleep_enabled, const f32 sleep_time_threshold, const f32 sleep_linear_velocity_sq_limit, const f32 sleep_angular_velocity_sq_limit);


/*
 * Memory layout: Three distictions
 *
 * struct velocity_constraint_point 	- constraint point local data (body center to manifold point, and so on)
 * struct velocity_constraint 		- contact local data (manifold normal, body indices, and so on)
 * solver->array			- shared data between contacts, i.e temporary body changes (velocities, ...)
 */

/*
velocity_constraint_point 
=========================
individual constraint for one point in the contact manifold
 */

struct velocity_constraint_point
{
	vec3 	r1;		/* vector from body 1's center to manifold point */
	vec3 	r2;		/* vector from body 2's center to manifold point */
	f32 	normal_impulse;	/* the normal impulse produced by the contact */
	f32	velocity_bias;	/* scale of velocity_bias along contact normal */
	f32	normal_mass;	/* 1.0f / row(J,i)*Inv(M)*J^T entry for point */
	f32	tangent_mass[2]; /* 1.0f / row(J_tangent,i)*Inv(M)*J_tangent^T entry for point */
	f32	tangent_impulse[2]; /* the tangent impulses produced by the contact */
};

struct velocity_constraint
{
	struct velocity_constraint_point *vcps;
	void * 	normal_mass;	/* mat2, mat3 or mat4 normal mass for block solver = Inv(J*Inv(M)*J^T) */
	void * 	inv_normal_mass;/* mat2, mat3 or mat4 inv normal mass for block solver = J*Inv(M)*J^T */

	/* contact base axes */
	vec3 	normal;		/* Currently shared contact manifold normal between all point constraints */
	vec3	tangent[2];	/* normalized friction directions of contact */

	u32 	lb1;		/* local body 1 index (index into solver arrays) */
	u32 	lb2;		/* local body 2 index (index into solver arrays) */
	u32 	vcp_count;	/* Number of contact points in the contact manifold */
	f32	restitution;	/* Range[0.0f, 1.0f] : higher => bouncy */
	//f32	tangent_impulse_bound;	/* TODO: contact_friction * gravity_constant * point_mass */
	f32	friction;	/* TODO: friction = f32_max(b1->friction, b2->friction) */
	u32	block_solve;	/* if config->block_solver && condition number of block normal mass is ok, then = 1 */
};

struct contact_solver
{
	f32 			timestep;
	u32			body_count;
	u32			contact_count;

	struct rigid_body **	bodies;
	mat3ptr			Iw_inv;		/* inverted world inertia tensors */
	struct velocity_constraint *vcs;	

	/* temporary state of bodies in island, static bodies index last element */
	vec3ptr			linear_velocity;
	vec3ptr			angular_velocity;
};

struct contact_solver *	contact_solver_init_body_data(struct arena *mem, struct island *is, const f32 timestep);
void 			contact_solver_init_velocity_constraints(struct arena *mem, struct contact_solver *solver, const struct physics_pipeline *pipeline, const struct island *is);
void 			contact_solver_iterate_velocity_constraints(struct contact_solver *solver);
void 			contact_solver_warmup(struct contact_solver *solver, const struct island *is);
void 			contact_solver_cache_impulse_data(struct contact_solver *solver, const struct island *is);

/*
=================================================================================================================
|						Physics Pipeline			  	      	    	|
=================================================================================================================
*/

/*
rigid_body
========== 
physics engine entity 
*/

#define RB_ACTIVE		((u32) 1 << 0)
#define RB_DYNAMIC		((u32) 1 << 1)
#define RB_AWAKE		((u32) 1 << 2)
#define RB_ISLAND		((u32) 1 << 3)
#define RB_MARKED_FOR_REMOVAL	((u32) 1 << 4)

#define RB_IS_ACTIVE(b)		((b->flags & RB_ACTIVE) >> 0u)
#define RB_IS_DYNAMIC(b)	((b->flags & RB_DYNAMIC) >> 1u)
#define RB_IS_AWAKE(b)		((b->flags & RB_AWAKE) >> 2u)
#define RB_IS_ISLAND(b)		((b->flags & RB_ISLAND) >> 3u)
#define RB_IS_MARKED(b)		((b->flags & RB_MARKED_FOR_REMOVAL) >> 4u)

#define IS_ACTIVE(flags)	((flags & RB_ACTIVE) >> 0u)
#define IS_DYNAMIC(flags)	((flags & RB_DYNAMIC) >> 1u)
#define IS_AWAKE(flags)		((flags & RB_AWAKE) >> 2u)
#define IS_ISLAND(flags)	((flags & RB_ISLAND) >> 3u)
#define IS_MARKED(flags)	((flags & RB_MARKED_FOR_REMOVAL) >> 4u)

struct rigid_body
{
	DLL_SLOT_STATE;
	POOL_SLOT_STATE;
	/* dynamic state */
	struct AABB	local_box;		/* bounding AABB */

	quat 		rotation;		
	vec3 		velocity;
	vec3 		angular_velocity;

	quat 		angular_momentum;	/* TODO: */
	vec3 		position;		/* center of mass world frame position */
	vec3 		linear_momentum;   	/* L = mv */

	u32		first_contact_index;
	u32		island_index;

	/* static state */
	u32 		entity;
	u32 		flags;
	i32 		proxy;
	f32 		margin;

	enum collision_shape_type shape_type;
	u32		shape_handle;

	mat3 		inertia_tensor;		/* intertia tensor of body frame */
	mat3 		inv_inertia_tensor;
	f32 		mass;			/* total body mass */
	f32 		restitution;
	f32 		friction;		/* Range [0.0, 1.0f] : bound tangent impulses to 
						   mix(b1->friction, b2->friction)*(normal impuse) */
	f32 		low_velocity_time;	/* Current uninterrupted time body has been in a low velocity state */
};

/*
rigid_body_prefab
=================
rigid body prefabs: used within editor and level editor file format, contains resuable preset values for creating
new bodies.
*/
struct rigid_body_prefab
{
	STRING_DATABASE_SLOT_STATE;
	u32		shape;

	mat3 		inertia_tensor;		/* intertia tensor of body frame */
	mat3 		inv_inertia_tensor;
	f32 		mass;			/* total body mass */
	f32		density;
	f32 		restitution;
	f32 		friction;		/* Range [0.0, 1.0f] : bound tangent impulses to 
						   mix(b1->friction, b2->friction)*(normal impuse) */
	u32		dynamic;		/* dynamic body is true, static if false */
};

void 	prefab_statics_setup(struct rigid_body_prefab *prefab, struct collision_shape *shape, const f32 density);

#define UNITS_PER_METER		1.0f
#define UNITS_PER_DECIMETER	0.1f
#define UNITS_PER_CENTIMETER	0.01f
#define UNITS_PER_MILIMETER	0.001f

#define COLLISION_MARGIN_DEFAULT 5.0f * UNITS_PER_MILIMETER 

#define UNIFORM_SIZE 256
#define GRAVITY_CONSTANT_DEFAULT 9.80665f

#define PHYSICS_EVENT_BODY(pipeline, event_type, body_index)						\
	{												\
		struct physics_event *__physics_debug_event = physics_pipeline_event_push(pipeline);	\
		__physics_debug_event->type = event_type;						\
		__physics_debug_event->body = body_index;						\
	}

#define PHYSICS_EVENT_ISLAND(pipeline, event_type, island_index)					\
	{												\
		struct physics_event *__physics_debug_event = physics_pipeline_event_push(pipeline);	\
		__physics_debug_event->type = event_type;						\
		__physics_debug_event->island = island_index;						\
	}

#ifdef DS_PHYSICS_DEBUG

#define	PHYSICS_EVENT_BODY_NEW(pipeline, body)			PHYSICS_EVENT_BODY(pipeline, PHYSICS_EVENT_BODY_NEW, body)
#define	PHYSICS_EVENT_BODY_REMOVED(pipeline, body)		PHYSICS_EVENT_BODY(pipeline, PHYSICS_EVENT_BODY_REMOVED, body)
#define	PHYSICS_EVENT_ISLAND_ASLEEP(pipeline, island)		PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_ASLEEP, island)
#define	PHYSICS_EVENT_ISLAND_AWAKE(pipeline, island)		PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_AWAKE, island)
#define	PHYSICS_EVENT_ISLAND_NEW(pipeline, island)		PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_NEW, island)
#define	PHYSICS_EVENT_ISLAND_EXPANDED(pipeline, island)		PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_EXPANDED, island)
#define	PHYSICS_EVENT_ISLAND_REMOVED(pipeline, island)		PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_REMOVED, island)
#define PHYSICS_EVENT_CONTACT_NEW(pipeline, body1_index, body2_index)					\
	{												\
		struct physics_event *__physics_debug_event = physics_pipeline_event_push(pipeline);	\
		__physics_debug_event->type = PHYSICS_EVENT_CONTACT_NEW;				\
		__physics_debug_event->contact_bodies.body1 = body1_index;				\
		__physics_debug_event->contact_bodies.body2 = body2_index;				\
	}
#define PHYSICS_EVENT_CONTACT_REMOVED(pipeline, body1_index, body2_index)				\
	{												\
		struct physics_event *__physics_debug_event = physics_pipeline_event_push(pipeline);	\
		__physics_debug_event->type = PHYSICS_EVENT_CONTACT_REMOVED;				\
		__physics_debug_event->contact_bodies.body1 = body1_index;				\
		__physics_debug_event->contact_bodies.body2 = body2_index;				\
	}

#else

#define	PHYSICS_EVENT_BODY_NEW(pipeline, body)
#define	PHYSICS_EVENT_BODY_REMOVED(pipeline, body)
#define	PHYSICS_EVENT_ISLAND_ASLEEP(pipeline, island)
#define	PHYSICS_EVENT_ISLAND_AWAKE(pipeline, island) 
#define	PHYSICS_EVENT_ISLAND_NEW(pipeline, island)   
#define	PHYSICS_EVENT_ISLAND_EXPANDED(pipeline, island)   
#define	PHYSICS_EVENT_ISLAND_REMOVED(pipeline, island)
#define PHYSICS_EVENT_CONTACT_NEW(pipeline, contact)
#define PHYSICS_EVENT_CONTACT_REMOVED(pipeline, body1, body2)

#endif

enum physics_event_type
{
	PHYSICS_EVENT_CONTACT_NEW,
	PHYSICS_EVENT_CONTACT_REMOVED,
	PHYSICS_EVENT_ISLAND_NEW,
	PHYSICS_EVENT_ISLAND_EXPANDED,
	PHYSICS_EVENT_ISLAND_REMOVED,
	PHYSICS_EVENT_ISLAND_AWAKE,
	PHYSICS_EVENT_ISLAND_ASLEEP,
	PHYSICS_EVENT_BODY_NEW,
	PHYSICS_EVENT_BODY_REMOVED,
	PHYSICS_EVENT_BODY_ORIENTATION,
	PHYSICS_EVENT_COUNT
};

struct physics_event
{
	POOL_SLOT_STATE;
	DLL_SLOT_STATE;

	u64			ns;	/* time of event */
	enum physics_event_type type;
	union
	{
		u32 island;
		u32 body;
		struct 
		{
			u32 body1;
			u32 body2;
		} contact_bodies;
	};
};

enum rigid_body_color_mode
{
	RB_COLOR_MODE_BODY = 0,
	RB_COLOR_MODE_COLLISION,
	RB_COLOR_MODE_ISLAND,
	RB_COLOR_MODE_SLEEP,
	RB_COLOR_MODE_COUNT
};

extern const char **body_color_mode_str;
/*
 * Physics Pipeline
 */
struct physics_pipeline 
{
	struct arena 		frame;			/* frame memory */

	u64			ns_start;		/* external ns at start of physics pipeline */
	u64			ns_elapsed;		/* actual ns elasped in pipeline (= 0 at start) */
	u64			ns_tick;		/* ns per game tick */
	u64 			frames_completed;	/* number of completed physics frames */ 

	struct strdb *shape_db;		/* externally owned */
	struct strdb *prefab_db;		/* externally owned */

	struct pool		body_pool;
	struct dll		body_marked_list;	/* bodies marked for removal */
	struct dll		body_non_marked_list;	/* bodies alive and non-marked  */

	struct pool		event_pool;
	struct dll		event_list;

	struct bvh 		dynamic_tree;

	struct contact_database	c_db;
	struct island_database 	is_db;

	struct collision_debug *debug;
	u32			debug_count;

	//TODO temporary, move somewhere else.
	vec3 			gravity;	/* gravity constant */

	u32			margin_on;
	f32			margin;

	/* frame data */
	u32			contact_new_count;
	u32			proxy_overlap_count;
	u32			cm_count;
	u32 *			contact_new;
	struct dbvh_overlap *	proxy_overlap;
	struct contact_manifold *cm;

	/* debug */
	enum rigid_body_color_mode	pending_body_color_mode;
	enum rigid_body_color_mode	body_color_mode;
	vec4				collision_color;
	vec4				static_color;
	vec4				sleep_color;
	vec4				awake_color;

	vec4				bounding_box_color;
	vec4				dbvh_color;
	vec4				sbvh_color;
	vec4				manifold_color;

	u32				draw_bounding_box;
	u32				draw_dbvh;
	u32				draw_sbvh;
	u32				draw_manifold;
	u32				draw_lines;
};

/**************** PHYISCS PIPELINE API ****************/

/* Initialize a new growable physics pipeline; ns_tick is the duration of a physics frame. */
struct physics_pipeline	physics_pipeline_alloc(struct arena *mem, const u32 initial_size, const u64 ns_tick, const u64 frame_memory, struct strdb *shape_db, struct strdb *prefab_db);
/* free pipeline resources */
void 			physics_pipeline_free(struct physics_pipeline *physics_pipeline);
/* flush pipeline resources */
void			physics_pipeline_flush(struct physics_pipeline *physics_pipeline);
/* pipeline main method: simulate a single physics frame and update internal state  */
void 			physics_pipeline_tick(struct physics_pipeline *pipeline);
/* allocate new rigid body in pipeline and return its slot */
struct slot		physics_pipeline_rigid_body_alloc(struct physics_pipeline *pipeline, struct rigid_body_prefab *prefab, const vec3 position, const quat rotation, const u32 entity);
/* deallocate a collision shape associated with the given handle. If no shape is found, do nothing */
void			physics_pipeline_rigid_body_tag_for_removal(struct physics_pipeline *pipeline, const u32 handle);
/* validate and assert internal state of physics pipeline */
void			physics_pipeline_validate(const struct physics_pipeline *pipeline);
/* If hit, return parameter (body,t) of ray at first collision. Otherwise return (U32_MAX, F32_INFINITY) */
u32f32 			physics_pipeline_raycast_parameter(struct arena *mem_tmp, const struct physics_pipeline *pipeline, const struct ray *ray);
/* enable sleeping in pipeline */
void 			physics_pipeline_enable_sleeping(struct physics_pipeline *pipeline);
/* disable sleeping in pipeline */
void 			physics_pipeline_disable_sleeping(struct physics_pipeline *pipeline);

#ifdef DS_PHYSICS_DEBUG
#define PHYSICS_PIPELINE_VALIDATE(pipeline)	physics_pipeline_validate(pipeline)
#else
#define PHYSICS_PIPELINE_VALIDATE(pipeline)	
#endif

/**************** PHYISCS PIPELINE INTERNAL API ****************/

/* push physics event into pipeline memory and return pointer to allocated event */
struct physics_event *	physics_pipeline_event_push(struct physics_pipeline *pipeline);

#endif
