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

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"
#include "ds_math.h"
#include "list.h"
#include "collision.h"
#include "hash_map.h"
#include "bit_vector.h"

struct ds_RigidBodyPipeline;

/*
ds_Shape
========
ds_Shapes are convex building blocks for constructing a ds_RigidBody. The structure
describes the volume's physical properties and its orientation within the local
frame of the body. A non-convex ds_RigidBody can be constructed by using multiple 
ds_Shapes.


::: Internals :::
When a user wishes to add a shape to a body, we will be working in some arbitrary 
space. For example, If we are working within blender and constructing our rigid
body by placing and rotating a set of shapes. Then, our arbitrary space is the one 
within blender. When we place this body within our world, we expect each sequence 
of transformation of the shape to be	

	Shape => Local Rotate => Local Offset	(Orientation within body frame)
	      => Body Rotate  => Body Offset	(Orientation within world space)

Now, if at runtime we wish to add an additional shape to the body, we still expect
the transform we supply with the shape to refer to this arbitrary space (unless we
have updated to a new arbitrary space at some point, which would have entailed 
updating the body's position and mass properties, and recalculating the local 
transforms of all of its shapes). For this to work, we must allow the local frame
of the body to be arbitrary; it is up to the user to update the local frame if he
or she so wishes. Hence, we cannot readjust the local frame of the body to always
have the center of mass as its origin; by doing so. Thus, in addition to storing 
the local-to-world transform, ds_RigidBody must also store its center of mass:

	ds_RigidBody
	{
		(...)
		ds_Transform	transform;	    // Local frame to World transform
		vec3            center_of_mass;	
	}

We now derive the transformations needed assuming an arbitrary center of mass.

*/

struct ds_Shape
{
	POOL_SLOT_STATE;
	DLL_SLOT_STATE;				            /* Node links in ds_RigidBody shape_list 	local   */

	u32 			body;		            /* ds_RigidBody owner of node 			            */
	u32			    contact_first;	        /* index to first contact in shape's list (nll)     */

	enum collisionShapeType cshape_type;	/* collisionShape type 				                */
	u32			    cshape_handle;	        /* handle to referenced collisionShape 		        */

	f32			    density;	            /* kg/m^3					                        */
	f32 			restitution;            /* Range [0.0, 1.0] : bounciness  		            */
	f32 			friction;	            /* Range [0.0, 1.0] : bound tangent impulses to 
						                       mix(b1->friction, b2->friction)*(normal impuse)  */
	f32		        margin;		            /* bouding box margin for dynamic BVH proxies 	    */

	ds_Transform	t_local;	            /* local body frame transform 			            */

	/* DYNAMIC STATE */
	u32			    proxy;		            /* BVH index 					                    */
};

struct ds_ShapePrefab
{
	STRING_DATABASE_SLOT_STATE;

	u32	    cshape;	        /* utf8 identifier of collisionShape 		        */
	f32		density;	    /* kg/m^3					                        */
	f32 	restitution;	/* Range [0.0, 1.0] : bounciness  		            */
	f32 	friction;	    /* Range [0.0, 1.0] : bound tangent impulses to 
						       mix(b1->friction, b2->friction)*(normal impuse)  */
	f32		margin;	        /* bouding box margin for dynamic BVH proxies 	    */
};

/* 
 * Allocates a shape according to the values set in Prefab and with given local body frame transform. On success, 
 * a valid address and index to the shape is returned. If the ID is invalid, a default collisionShape is assigned
 * to the shape. On failure, (NULL, POOL_NULL) is returned. 
 */
struct slot ds_ShapeAdd(struct ds_RigidBodyPipeline *pipeline, const struct ds_ShapePrefab *prefab, const ds_Transform *t, const u32 body);
/* Remove the specified shape of a DYNAMIC body and update the physics state into a valid state.  */
void		ds_ShapeDynamicRemove(struct ds_RigidBodyPipeline *pipeline, const u32 shape);
/* Remove the specified shape of a STATIC body and update the physics state into a valid state. */
void		ds_ShapeStaticRemove(struct ds_RigidBodyPipeline *pipeline, const u32 shape);
/* Calculate the world bounding box of the shape, taking into account the shape and its body's Transform. */
struct aabb ds_ShapeWorldBbox(const struct ds_RigidBodyPipeline *pipeline, struct ds_Shape *shape);


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

struct ds_RigidBody
{
	DLL2_SLOT_STATE;	/* island body_list node */
	DLL_SLOT_STATE;		/* body marked/non-marked list node */
	POOL_SLOT_STATE;

	struct dll	shape_list;		/* list of convex shapes constructing the rigid body 	*/
	ds_Transform	t_world;		/* local body frame to world transform 			*/
	vec3		local_center_of_mass;	/* local body frame center of mass 			*/

	/* dynamic state */

	struct aabb	local_box;		/* bounding AABB */

	quat 		rotation;		
	vec3 		velocity;
	vec3 		angular_velocity;

	quat 		angular_momentum;	/* TODO: */
	vec3 		position;		/* center of mass world frame position */
	vec3 		linear_momentum;   	/* L = mv */

	u32		contact_first;
	u32		island_index;

	/* static state */
	u32 		entity;
	u32 		flags;
	i32 		proxy;
	f32 		margin;

	enum collisionShapeType shape_type;
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
struct ds_RigidBodyPrefab
{
	STRING_DATABASE_SLOT_STATE;

    //TODO Make multishape possible
	u32     shape;

	mat3 	inertia_tensor;		/* intertia tensor of body frame */
	mat3 	inv_inertia_tensor;
	f32 	mass;			/* total body mass */
	f32     density;
	f32 	restitution;
	f32 	friction;		/* Range [0.0, 1.0f] : bound tangent impulses to 
						   mix(b1->friction, b2->friction)*(normal impuse) */
	u32	dynamic;		/* dynamic body is true, static if false */
};

//TODO remove 
void    PrefabStaticsSetup(struct ds_RigidBodyPrefab *prefab, struct collisionShape *shape, const f32 density);
void    RigidBodyUpdateLocalBox(struct ds_RigidBody *body, const struct collisionShape *shape);


//TODO
struct slot ds_RigidBodyAdd(struct ds_RigidBodyPipeline *pipeline, struct ds_RigidBodyPrefab *prefab, const vec3 position, const quat rotation, const u32 entity);
/* Free the body at the given index */
void		ds_RigidBodyRemove(struct ds_RigidBodyPipeline *pipeline, const u32 body);
/* Process the body's shape list and set its internal mass properties accordingly. */
void		ds_RigidBodyUpdateMassProperties(struct ds_RigidBodyPipeline *pipeline, const u32 body);

/*
=================================================================================================================
|						Contact Database						|
=================================================================================================================
*/

struct contact
{
	DLL_SLOT_STATE;		/* island->contact_list node */
	NLL_SLOT_STATE;		/* body->contact_net node */
	struct contactManifold 	cm;
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
 	3. cdb_ContactAdd(i1, i2, contact)	// add all new contacts 
 	4. solve
 	5. invalidate any contacts before caching them.
 	6. switch frame and cache 
 	7. reset frame
*/

struct cdb
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
	struct hashMap	contact_map;		

	/*
	 * frame-cached separation axes 
	 */
	struct pool	sat_cache_pool;
	struct dll	sat_cache_list;
	struct hashMap	sat_cache_map;		

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

struct cdb 	cdb_Alloc(struct arena *mem_persistent, const u32 initial_size);
void 		cdb_Free(struct cdb *c_db);
void		cdb_Flush(struct cdb *c_db);
void		cdb_Validate(const struct ds_RigidBodyPipeline *pipeline);
void		cdb_ClearFrame(struct cdb *c_db);
/* Update or add new contact depending on if the contact persisted from prevous frame. */
struct contact *cdb_ContactAdd(struct ds_RigidBodyPipeline *pipeline, const struct contactManifold *cm, const u32 i1, const u32 i2);
void 		cdb_ContactRemove(struct ds_RigidBodyPipeline *pipeline, const u64 key, const u32 index);
/* Remove all contacts associated with the given body */
void		cdb_BodyRemoveContacts(struct ds_RigidBodyPipeline *pipeline, const u32 body_index);
/* Remove all contacts associated with the given static body and update affected islands */
void		cdb_StaticRemoveContactsAndUpdateIslands(struct ds_RigidBodyPipeline *pipeline, const u32 static_index);
struct contact *cdb_ContactLookup(const struct cdb *c_db, const u32 b1, const u32 b2);
u32 		cdb_ContactLookupIndex(const struct cdb *c_db, const u32 i1, const u32 i2);
void 		cdb_UpdatePersistentContactsUsage(struct cdb *c_db);

/* add sat_cache to pipeline; if it already exists, reset the cache. */
void 			SatCacheAdd(struct cdb *c_db, const struct satCache *sat_cache);
/* lookup sat_cache to pipeline; if it does't exist, return NULL. */
struct satCache *	SatCacheLookup(const struct cdb *c_db, const u32 b1, const u32 b2);

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

#define ISLAND_NULL	POOL_NULL 
#define ISLAND_STATIC	POOL_NULL-1	/* static bodies are mapped to "island" ISLAND_STATIC */

struct island
{
	POOL_SLOT_STATE;
	DLL_SLOT_STATE;

	struct ds_RigidBody **	bodies;	
	struct contact 	**	contacts;
	u32 *			body_index_map; /* body_index -> local indices of bodies in island:
						 * is->bodies[i] = pipeline->bodies[b] => 
						 * is->body_index_map[b] = i 
						 */

	/* Persistent Island */
	u32 flags;

	struct dll	body_list;
	struct dll	contact_list;

#ifdef DS_PHYSICS_DEBUG
	vec4 color;
#endif
};

struct isdb
{
	/* PERSISTENT DATA */
	struct pool 	island_pool;	/* GROWABLE, list nodes of contacts and bodies	*/
	struct dll	island_list;
	/* FRAME DATA */
	u32 *		possible_splits;				/* Islands in which a contact has been broken during frame */
	u32 		possible_splits_count;
};

#ifdef DS_PHYSICS_DEBUG
#define IS_DB_VALIDATE(pipeline)	isdb_Validate((pipeline)
#else
#define IS_DB_VALIDATE(pipeline)	
#endif

struct ds_RigidBodyPipeline;

/* Setup and allocate memory for new database */
struct isdb 	isdb_Alloc(struct arena *mem_persistent, const u32 initial_size);
/* Free any heap memory */
void	       	isdb_Dealloc(struct isdb *is_db);
/* Flush / reset the island database */
void		isdb_Flush(struct isdb *is_db);
/* Clear any frame related data */
void		isdb_ClearFrame(struct isdb *is_db);
/* remove island resources from database */
void 		isdb_IslandRemove(struct ds_RigidBodyPipeline *pipeline, struct island *is);
/* remove island resources related to body, and possibly the whole island, from database */
void 		isdb_IslandRemoveBodyResources(struct ds_RigidBodyPipeline *pipeline, const u32 island_index, const u32 body);
/* Debug printing of island */
void 		isdb_PrintIsland(FILE *file, const struct ds_RigidBodyPipeline *pipeline, const u32 island, const char *desc);
/* Check if the database appears to be valid */
void 		isdb_Validate(const struct ds_RigidBodyPipeline *pipeline);
/* Setup new island from single body */
struct island *	isdb_InitIslandFromBody(struct ds_RigidBodyPipeline *pipeline, const u32 body);
/* Add contact to island */
void 		isdb_AddContactToIsland(struct ds_RigidBodyPipeline *pipeline, const u32 island, const u32 contact);
/* Return island that body is assigned to */
struct island *	isdb_BodyToIsland(struct ds_RigidBodyPipeline *pipeline, const u32 body);
/* Reserve enough memory to fit all possible split */
void		isdb_ReserveSplitsMemory(struct arena *mem_frame, struct isdb *is_db);
/* Release any unused reserved possible split memory */
void		isdb_ReleaseUnusedSplitsMemory(struct arena *mem_frame, struct isdb *is_db);
/* Tag the island that the body is in for splitting and push it onto split memory (if we havn't already) */
void 		isdb_TagForSplitting(struct ds_RigidBodyPipeline *pipeline, const u32 body);
/* Merge islands (Or simply update if new local contact) using new contact */
void 		isdb_MergeIslands(struct ds_RigidBodyPipeline *pipeline, const u32 ci, const u32 b1, const u32 b2);
/* Split island, or remake if no split happens: TODO: Make thread-safe  */
void 		isdb_SplitIsland(struct arena *mem_tmp, struct ds_RigidBodyPipeline *pipeline, const u32 island_to_split);

/********* Threaded Island API *********/

struct islandSolveOutput
{
	u32 island;
	u32 island_asleep;
	u32 body_count;
	u32 *bodies;		/* bodies simulated in island */ 
	struct islandSolveOutput *next;
};

struct islandSolveInput
{
	struct island *is;
	struct ds_RigidBodyPipeline *pipeline;
	struct islandSolveOutput *out;
	f32 timestep;
};

/*
 * Input: struct island_solve_in 
 * Output: struct islandSolveOutput
 *
 * Solves the given island using the global solver config. Since no island shares any contacts or bodies, and every
 * island is a unique task, no shared variables are being written to.
 *
 * - reads pipeline, solver config, c_db, is_db (basically everything)
 * - writes to island,		(unique to thread, memory in c_db)
 * - writes to island->contacts (unique to thread, memory in c_db)
 * - writes to island->bodies	(unique to thread, memory in pipeline)
 */
void	ThreadIslandSolve(void *task_input);

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
struct solverConfig
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
	f32 	restitution_threshold; 	/* Range[0.0, inf] : If -seperating_velocity >= threshold, we apply the 
					   restitution effect */

	u32 	sleep_enabled;		/* bool : enable sleeping of bodies  */
	f32 	sleep_time_threshold; /* Range(0.0, inf] :  Time threshold for which a body must have low velocity before being able to fall asleep */
	f32 	sleep_linear_velocity_sq_limit; /* Range (0.0f, inf] : maximum linear velocity squared that a body falling asleep may have */
	f32 	sleep_angular_velocity_sq_limit; /* Range (0.0f, inf] : maximum angular velocity squared that a body falling asleep may have */

	/* Pending updates */
	u32 	pending_block_solver;		
	u32 	pending_warmup_solver;		
	u32 	pending_sleep_enabled;		
	u32 	pending_iteration_count;
	f32 	pending_baumgarte_constant;
	f32 	pending_linear_slop;
	f32 	pending_restitution_threshold;
	f32 	pending_linear_dampening;
	f32 	pending_angular_dampening;
};

extern struct solverConfig *g_solver_config;

void	SolverConfigInit(const u32 iteration_count, const u32 block_solver, const u32 warmup_solver, const vec3 gravity, const f32 baumgarte_constant, const f32 max_condition, const f32 linear_dampening, const f32 angular_dampening, const f32 linear_slop, const f32 restitution_threshold, const u32 sleep_enabled, const f32 sleep_time_threshold, const f32 sleep_linear_velocity_sq_limit, const f32 sleep_angular_velocity_sq_limit);


/*
 * Memory layout: Three distictions
 *
 * struct velocityConstraintPoint 	- constraint point local data (body center to manifold point, and so on)
 * struct velocityConstraint 		- contact local data (manifold normal, body indices, and so on)
 * solver->array			- shared data between contacts, i.e temporary body changes (velocities, ...)
 */

/*
velocity_constraint_point 
=========================
individual constraint for one point in the contact manifold
 */

struct velocityConstraintPoint
{
	vec3 	r1;		/* vector from body 1's center to manifold point */
	vec3 	r2;		/* vector from body 2's center to manifold point */
	f32 	normal_impulse;	/* the normal impulse produced by the contact */
	f32	velocity_bias;	/* scale of velocity_bias along contact normal */
	f32	normal_mass;	/* 1.0f / row(J,i)*Inv(M)*J^T entry for point */
	f32	tangent_mass[2]; /* 1.0f / row(J_tangent,i)*Inv(M)*J_tangent^T entry for point */
	f32	tangent_impulse[2]; /* the tangent impulses produced by the contact */
};

struct velocityConstraint
{
	struct velocityConstraintPoint *vcps;
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

struct solver
{
	f32 			timestep;
	u32			body_count;
	u32			contact_count;

	struct ds_RigidBody **	bodies;
	mat3ptr			Iw_inv;		/* inverted world inertia tensors */
	struct velocityConstraint *vcs;	

	/* temporary state of bodies in island, static bodies index last element */
	vec3ptr			linear_velocity;
	vec3ptr			angular_velocity;
};

struct solver *	SolverInitBodyData(struct arena *mem, struct island *is, const f32 timestep);
void 		SolverInitVelocityConstraints(struct arena *mem, struct solver *solver, const struct ds_RigidBodyPipeline *pipeline, const struct island *is);
void 		SolverIterateVelocityConstraints(struct solver *solver);
void 		SolverWarmup(struct solver *solver, const struct island *is);
void 		SolverCacheImpulse(struct solver *solver, const struct island *is);

/*
=================================================================================================================
|						Physics Pipeline			  	      	    	|
=================================================================================================================
*/

#define UNITS_PER_METER		1.0f
#define UNITS_PER_DECIMETER	0.1f
#define UNITS_PER_CENTIMETER	0.01f
#define UNITS_PER_MILIMETER	0.001f

#define COLLISION_MARGIN_DEFAULT 5.0f * UNITS_PER_MILIMETER 

#define UNIFORM_SIZE 256
#define GRAVITY_CONSTANT_DEFAULT 9.80665f

#define PHYSICS_EVENT_BODY(pipeline, event_type, body_index)						\
	{												\
		struct physicsEvent *__physics_debug_event = PhysicsPipelineEventPush(pipeline);	\
		__physics_debug_event->type = event_type;						\
		__physics_debug_event->body = body_index;						\
	}

#define PHYSICS_EVENT_ISLAND(pipeline, event_type, island_index)					\
	{												\
		struct physicsEvent *__physics_debug_event = PhysicsPipelineEventPush(pipeline);	\
		__physics_debug_event->type = event_type;						\
		__physics_debug_event->island = island_index;						\
	}

#ifdef DS_PHYSICS_DEBUG

#define	PhysicsEventBodyNew(pipeline, body)		PHYSICS_EVENT_BODY(pipeline, PHYSICS_EVENT_BODY_NEW, body)
#define	PhysicsEventBodyRemoved(pipeline, body)		PHYSICS_EVENT_BODY(pipeline, PHYSICS_EVENT_BODY_REMOVED, body)
#define	PhysicsEventIslandAsleep(pipeline, island)	PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_ASLEEP, island)
#define	PhysicsEventIslandAwake(pipeline, island)	PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_AWAKE, island)
#define	PhysicsEventIslandNew(pipeline, island)		PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_NEW, island)
#define	PhysicsEventIslandExpanded(pipeline, island)	PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_EXPANDED, island)
#define	PhysicsEventIslandRemoved(pipeline, island)	PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_REMOVED, island)
#define PhysicsEventContactNew(pipeline, body1_index, body2_index)					\
	{												\
		struct physicsEvent *__physics_debug_event = PhysicsPipelineEventPush(pipeline);	\
		__physics_debug_event->type = PHYSICS_EVENT_CONTACT_NEW;				\
		__physics_debug_event->contact_bodies.body1 = body1_index;				\
		__physics_debug_event->contact_bodies.body2 = body2_index;				\
	}
#define PhysicsEventContactRemoved(pipeline, body1_index, body2_index)				\
	{												\
		struct physicsEvent *__physics_debug_event = PhysicsPipelineEventPush(pipeline);	\
		__physics_debug_event->type = PHYSICS_EVENT_CONTACT_REMOVED;				\
		__physics_debug_event->contact_bodies.body1 = body1_index;				\
		__physics_debug_event->contact_bodies.body2 = body2_index;				\
	}

#else

#define	PhysicsEventBodyNew(pipeline, body)
#define	PhysicsEventBodyRemoved(pipeline, body)
#define	PhysicsEventIslandAsleep(pipeline, island)
#define	PhysicsEventIslandAwake(pipeline, island) 
#define	PhysicsEventIslandNew(pipeline, island)   
#define	PhysicsEventIslandExpanded(pipeline, island)   
#define	PhysicsEventIslandRemoved(pipeline, island)
#define PhysicsEventContactNew(pipeline, contact)
#define PhysicsEventContactRemoved(pipeline, body1, body2)

#endif

enum physicsEventType
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

struct physicsEvent
{
	POOL_SLOT_STATE;
	DLL_SLOT_STATE;

	u64			ns;	/* time of event */
	enum physicsEventType type;
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

enum rigidBodyColorMode
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
struct ds_RigidBodyPipeline 
{
	struct arena 		frame;			/* frame memory */

	u64				ns_start;		/* external ns at start of physics pipeline */
	u64				ns_elapsed;		/* actual ns elasped in pipeline (= 0 at start) */
	u64				ns_tick;		/* ns per game tick */
	u64 			frames_completed;	/* number of completed physics frames */ 

	struct strdb *	cshape_db;		    /* externally owned */
	struct strdb *	body_prefab_db;		/* externally owned */

	struct pool		body_pool;
	struct dll		body_marked_list;	/* bodies marked for removal */
	struct dll		body_non_marked_list;	/* bodies alive and non-marked  */

	struct pool		shape_pool;

	struct pool		event_pool;
	struct dll		event_list;

	//TODO remove 
	struct bvh 		dynamic_tree;

	struct bvh 		shape_bvh;

	struct cdb		c_db;
	struct isdb 	is_db;

	struct collisionDebug *	debug;
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
	struct dbvhOverlap *	proxy_overlap;
	struct contactManifold *cm;

	/* debug */
	enum rigidBodyColorMode	pending_body_color_mode;
	enum rigidBodyColorMode	body_color_mode;
	vec4			collision_color;
	vec4			static_color;
	vec4			sleep_color;
	vec4			awake_color;

	vec4			bounding_box_color;
	vec4			dbvh_color;
	vec4			sbvh_color;
	vec4			manifold_color;

	u32			draw_bounding_box;
	u32			draw_dbvh;
	u32			draw_sbvh;
	u32			draw_manifold;
	u32			draw_lines;
};

/**************** PHYISCS PIPELINE API ****************/

/* Initialize a new growable physics pipeline; ns_tick is the duration of a physics frame. */
struct ds_RigidBodyPipeline	PhysicsPipelineAlloc(struct arena *mem, const u32 initial_size, const u64 ns_tick, const u64 frame_memory, struct strdb *cshape_db, struct strdb *prefab_db);
/* free pipeline resources */
void 			PhysicsPipelineFree(struct ds_RigidBodyPipeline *physics_pipeline);
/* flush pipeline resources */
void			PhysicsPipelineFlush(struct ds_RigidBodyPipeline *physics_pipeline);
/* pipeline main method: simulate a single physics frame and update internal state  */
void 			PhysicsPipelineTick(struct ds_RigidBodyPipeline *pipeline);
/* allocate new rigid body in pipeline and return its slot */
struct slot		PhysicsPipelineRigidBodyAlloc(struct ds_RigidBodyPipeline *pipeline, struct ds_RigidBodyPrefab *prefab, const vec3 position, const quat rotation, const u32 entity);
/* deallocate a collision shape associated with the given handle. If no shape is found, do nothing */
void			PhysicsPipelineRigidBodyTagForRemoval(struct ds_RigidBodyPipeline *pipeline, const u32 handle);
/* validate and ds_Assert internal state of physics pipeline */
void			PhysicsPipelineValidate(const struct ds_RigidBodyPipeline *pipeline);
/* If hit, return parameter (body,t) of ray at first collision. Otherwise return (U32_MAX, F32_INFINITY) */
u32f32 			PhysicsPipelineRaycastParameter(struct arena *mem_tmp, const struct ds_RigidBodyPipeline *pipeline, const struct ray *ray);
/* enable sleeping in pipeline */
void 			PhysicsPipelineSleepEnable(struct ds_RigidBodyPipeline *pipeline);
/* disable sleeping in pipeline */
void 			PhysicsPipelineSleepDisable(struct ds_RigidBodyPipeline *pipeline);

#ifdef DS_PHYSICS_DEBUG
#define PHYSICS_PIPELINE_VALIDATE(pipeline)	PhysicsPipelineValidate(pipeline)
#else
#define PHYSICS_PIPELINE_VALIDATE(pipeline)	
#endif

/**************** PHYISCS PIPELINE INTERNAL API ****************/

/* push physics event into pipeline memory and return pointer to allocated event */
struct physicsEvent *	PhysicsPipelineEventPush(struct ds_RigidBodyPipeline *pipeline);

#ifdef __cplusplus
} 
#endif

#endif
