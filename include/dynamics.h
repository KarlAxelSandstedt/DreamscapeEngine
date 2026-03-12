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
#include "ds_hash_map.h"
#include "bit_vector.h"

//TODO 
struct ds_RigidBodyPipeline;
struct cdb;
struct ds_Island;

/*
ds_Id
=====
Opaque generation based handles for user-interfacing structures. ds_Id supports
16-bit generations, and 32-bit indices. ds_IdF (frequent) supports 32-bit 
generations and 32-bit indices.
*/

typedef u64 ds_Id;
typedef u64 ds_IdF; /* ds_IdF (frequent) */

typedef ds_Id   ds_RigidBodyId;
typedef ds_Id   ds_ShapeId;
typedef ds_IdF  ds_ContactId;

#define DS_ID_NULL                      U64_MAX
#define DS_ID_INDEX_MASK                ((u64) 0x00000000ffffffff)
#define DS_ID_TAG_MASK                  ((u64) 0xffffffff00000000)
#define DS_ID_TAG_INCREMENT             ((u64) 0x0000100000000000)

#define DS_ID_TAG_UNUSED_MASK           0x0000ffff
#define DS_ID_TAG_GENERATION_MASK       0xffff0000
#define DS_ID_TAG_GENERATION_INCREMENT  0x00010000

#define ds_IdTag(id)                    ((u32) (id >> 32))
#define ds_IdIndex(id)                  ((u32) id)

#define DS_IDF_NULL                     U64_MAX
#define DS_IDF_INDEX_MASK               ((u64) 0x00000000ffffffff)
#define DS_IDF_GENERATION_MASK          ((u64) 0xffffffff00000000)

#define ds_IdFGeneration(id)            ((u32) (id >> 32))
#define ds_IdFIndex(id)                 ((u32) id)

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
	DLL_SLOT_STATE;				        /* Node links in ds_RigidBody shape_list 	local   */

    u32             tag;                /* Tag [ Generation(16) | Unused (16) ]             */
	u32 			body;		        /* ds_RigidBody owner of node 			            */
	u32			    contact_first;	    /* index to first contact in shape's list (nll)     */

	enum c_ShapeType cshape_type;	    /* collisionShape type 				                */
	u32			    cshape_handle;	    /* handle to referenced collisionShape 		        */

	f32			    density;	        /* kg/m^3					                        */
	f32 			restitution;        /* Range [0.0, 1.0] : bounciness  		            */
	f32 			friction;	        /* Range [0.0, 1.0] : bound tangent impulses to 
						                   mix(s1->friction, s2->friction)*(normal impuse)  */
	f32		        margin;		        /* bouding box margin for dynamic BVH proxies 	    */

	ds_Transform	t_local;	        /* local body frame transform 			            */

	/* DYNAMIC STATE */
	u32			    proxy;		        /* BVH index 					                    */
};

/*
ds_ShapePrefab  
==============
ds_ShapePrefabs are ds_Shape blueprints. When adding a ds_Shape, you will most likely
use the same set of parameters for multiple shapes. This warrants the use of a 
ds_ShapePrefab which stores a common set of parameters, and can be referenced using
a utf8 identifier.
*/

#define PREFAB_BUFSIZE  32

struct ds_ShapePrefab
{
    u8      id_buf[PREFAB_BUFSIZE];
	STRING_DATABASE_SLOT_STATE;

	u32	    cshape;	            /* referenced collisionShape handle  		        */
	f32		density;	        /* kg/m^3					                        */
	f32 	restitution;	    /* Range [0.0, 1.0] : bounciness  		            */
	f32 	friction;	        /* Range [0.0, 1.0] : bound tangent impulses to 
						           mix(s1->friction, s2->friction)*(normal impuse)  */
	f32		margin;	            /* bouding box margin for dynamic BVH proxies 	    */

    /* TODO: why here... Currently each shape <-> mesh, so for 
     * simplicity in led, we store the mesh reference in prefab. */
    u32 render_mesh;
};

/*
ds_ShapePrefabInstance
======================
ds_ShapePrefabInstances are helpers for constructing ds_RigidBodyPrefabs. Since a
body may contain multiple shapes, the ds_RigidBodyPrefab struct contains a list of
ds_ShapePrefabInstances. Each instance contains an identifier local to the
ds_RigidBodyPrefab, a local transform, and a reference to the instanced ds_Shape.
*/
struct ds_ShapePrefabInstance
{
    POOL_SLOT_STATE;
    DLL_SLOT_STATE;             /* ds_RigidBodyPrefab instance list  */
    u8              id_buf[PREFAB_BUFSIZE];
    utf8            id;         /* local identifier within a body    */
    u32             shape_prefab;
	ds_Transform	t_local;	/* local body frame transform        */
};

/* 
 * Allocates a shape according to the values set in Prefab and with given local body frame transform. On success, 
 * an identifier to the shape is returned. On failure, U64 is return. 
 */
ds_ShapeId  ds_ShapeAdd(struct ds_RigidBodyPipeline *pipeline, const struct ds_ShapePrefab *prefab, const ds_Transform *t, const ds_RigidBodyId body);
/* 
 * Remove the specified shape of a DYNAMIC body and update the island database and contact database state.  
 */
void        ds_ShapeDynamicRemove(struct ds_RigidBodyPipeline *pipeline, struct ds_Island *island, const u32 shape_index);
/* 
 * Remove the specified shape of a STATIC body and update the physics state into a valid state. 
 */
void		ds_ShapeStaticRemove(struct arena *mem_tmp, struct ds_RigidBodyPipeline *pipeline, const u32 index);
/*
 * Lookup the specified shape and return it if found. Otherwise return (NULL, POOL_NULL).
 */
struct slot ds_ShapeLookup(const struct ds_RigidBodyPipeline *pipeline, const ds_ShapeId id);
/*
 * Calculate the world transform of the shape.
 */
void        ds_ShapeWorldTransform(ds_Transform *t, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *shape);
/* 
 * Calculate the world bounding box of the shape, taking into account the shape and its body's Transform. 
 */
struct aabb ds_ShapeWorldBbox(const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *shape);
/* 
 * Test for intersection between shapes, with each shape having the given margin. returns 1 if intersecting, else 0 
 */
u32	        ds_ShapeTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *s1, const struct ds_Shape *s2, const f32 margin);
/* 
 * Return, if no intersection was found, the distance between shapes s1 and s2 (with no margin) and their 
 * respective closest points c1 and c2. If the shapes are intersecting, return 0.0f. 
 */
f32 	    ds_ShapeDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *s1, const struct ds_Shape *s2, const f32 margin);
/* 
 * Returns 1 if the shapes are colliding, 0 otherwise. If a collision is found, return a contact manifold
 * with normal pointing from s1 to s2 (and set the sat_cache if non-null and applicable). 
 */
u32         ds_ShapeContact(struct arena *tmp, struct c_Manifold *manifold, struct sat_Cache *cache, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *s1, const struct ds_Shape *s2, const f32 margin);
/* 
 * Return, if ray intersects shape, t such that ray.origin + t*ray.dir == closest point on shape. 
 *         Otherwise, return F32_INFINITY.
 */
f32 	    ds_ShapeRaycastParameter(struct arena *tmp, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *shape, const struct ray *ray);
/* 
 * Return 1 if ray hit shape, 0 otherwise. If hit, we return the closest intersection point 
 */
u32 	    ds_ShapeRaycast(struct arena *tmp, vec3 intersection, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *shape, const struct ray *ray);


/*
rigid_body
========== 
//TODO
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

    u32             tag;                    /* Tag [ Generation(16) | Unused (16) ]                 */
	u32 		    flags;
	u32		        island_index;

	struct dll      shape_list;		        /* list of convex shapes constructing the rigid body 	*/
	ds_Transform    t_world;		        /* local body frame to world transform. Rotation is 
                                               about the local origin (not center of mass!)         */
	vec3		    local_center_of_mass;	/* local body frame center of mass 			            */

	vec3 		    velocity;               /* linear velocity of body */
	vec3 		    angular_velocity;       /* angular velocity of body (about local center of mass,
                                               not local origin!)                                   */
	vec3 		    linear_momentum;   	    /* L = mv */
	f32 		    low_velocity_time;	    /* Current uninterrupted time body has been in a low velocity state */

	mat3 		    inv_inertia_tensor;
	f32 		    mass;			        /* total body mass */

    //TODO Why do we store this here ...
	u32 	        entity;
};

/*
rigid_body_prefab
=================
TODO
rigid body prefabs: used within editor and level editor file format, contains resuable preset values for creating
new bodies.
*/
struct ds_RigidBodyPrefab
{
    u8              id_buf[PREFAB_BUFSIZE];
	STRING_DATABASE_SLOT_STATE;

    struct dll      shape_list;         /* shape prefab instance list */
    ds_RigidBodyId  body;
    
	u32	            dynamic;	        /* dynamic body is true, static if false */

    //TODO pre-compute...?
	//(f32 	        mass;			    /* total body mass */
	//mat3 	        inv_inertia_tensor;
};

//TODO
ds_RigidBodyId  ds_RigidBodyAdd(struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBodyPrefab *prefab, const ds_Transform *t_world, const u32 entity);
/* Free the given body */
void            ds_RigidBodyRemove(struct arena *mem_tmp, struct ds_RigidBodyPipeline *pipeline, const ds_RigidBodyId id);
/* Lookup the given body and return it. If it does not exist, return DS_ID_NULL.  */
struct slot	    ds_RigidBodyLookup(const struct ds_RigidBodyPipeline *pipeline, const ds_RigidBodyId id);
/* Process the body's shape list and set its internal mass properties accordingly. */
void		    ds_RigidBodyUpdateMassProperties(struct ds_RigidBodyPipeline *pipeline, const ds_RigidBodyId id);

/*
ds_ContactKey
=============
ds_ContactKey is the unique key for a contact, and it used in the contact database
hash map. Since the key must be unique for a contact, we require it to be in 
canonical form, i.e. you may always assume that body0 < body1.  The shapes are the
subshapes of their respective bodys making contact. 
*/
struct ds_ContactKey
{
    u32 body0;      /* (body0 < body1)      */
    u32 shape0;     /* subshape of body0    */
    u32 body1;      /* (body0 < body1)      */
    u32 shape1;     /* subshape of body1    */
};

/* Return the Canonical key of (bodyA,shapeA) (bodyB,shapeB) */
struct ds_ContactKey    ds_ContactKeyCanonical(const u32 bodyA, const u32 shapeA, const u32 bodyB, const u32 shapeB);
/* Return a 32-bit hash of the key */
u32                     ds_ContactKeyHash(const struct ds_ContactKey *key);
/* Return 1 if the two keys are equivalent, otherwise return  0. */
u32                     ds_ContactKeyEquivalence(const struct ds_ContactKey *keyA, const struct ds_ContactKey *keyB);

/*
ds_Contact
==========
ds_Contact is the value mapped by a ds_ContactKey, and contains current and 
cached contact data and additional list node state. 
*/
struct ds_Contact
{
	DLL_SLOT_STATE;		                                /* island->contact_list node            */
	NLL_SLOT_STATE;		                                /* shape->contact_net node              */
    u32                     generation;                 
    struct ds_ContactKey    key;                        /* canonical-form key                   */
	struct c_Manifold 	    cm;                         /* Current contact manifold             */


	vec3 			        normal_cache;
	vec3 			        tangent_cache[2];
	vec3 			        v_cache[4];			        /* previous contact manifold vertices, 
					        			                   or { F32_MAX, F32_MAX, F32_MAX }     */
	f32 			        tangent_impulse_cache[4][2];
	f32 			        normal_impulse_cache[4];	/* contact_solver solution to contact 
                                                           constraint, or 0.0f                  */
	u32 			        cached_count;			    /* number of vertices in cache          */
};

/* Add and return new contact with unique key and update pipeline state */
struct slot ds_ContactAdd(struct ds_RigidBodyPipeline *pipeline, const struct c_Manifold *cm, const struct ds_ContactKey *key);
/* Remove contact at the given index and update pipeline state */
void 	    ds_ContactRemove(struct ds_RigidBodyPipeline *pipeline, const u32 index);
/* Return the contact associated with the given id. If no such contact is found, return (NULL, NLL_NULL) */
struct slot ds_ContactLookup(const struct ds_RigidBodyPipeline *pipeline, const ds_ContactId id);
/* Update contact at the given slot and update pipeline state. */
void        ds_ContactUpdate(struct ds_RigidBodyPipeline *pipeline, const struct slot slot, const struct c_Manifold *cm);
/* Return the contact associated with the given key. If no such contact is found, return (NULL, NLL_NULL) */
struct slot ds_ContactKeyLookup(const struct ds_RigidBodyPipeline *pipeline, const struct ds_ContactKey *key);

/*
sat_CacheKey
============
sat_CacheKey is the unique key for a sat_cache, and it used in the contact database
hash map. Since the key must be unique for a sat_cache, we require it to be in 
canonical form, i.e. you may always assume that Index(body0) < Index(body1). The 
shapes are the subshapes of their respective bodys making contact. 

::: Internals :::

The choice to using IDs instead of indices is required due to ABA issues; rigid 
bodies and shapes do not contain enough information themselves to easily lookup
any of their caches, so when we remove a body or a shape, we do not want to remove
its cache immediately. Instead, we lazily remove caches with 1 frame delay, which
introduces the ABA problems which justifies adding full ids to the cache key.

//TODO maybe this can be done in a less bloated way...
*/
struct sat_CacheKey
{
    ds_RigidBodyId  body0;      /* Index(body0) < Index(body1)  */
    ds_ShapeId      shape0;     /* subshape of body0            */
    ds_RigidBodyId  body1;      /* Index(body0) < Index(body1)  */
    ds_ShapeId      shape1;     /* subshape of body1            */
};

/* Return the Canonical key of (bodyA,shapeA) (bodyB,shapeB) */
struct sat_CacheKey sat_CacheKeyCanonical(const ds_RigidBodyId bodyA, const ds_ShapeId shapeA, const ds_RigidBodyId bodyB, const ds_ShapeId shapeB);
/* Return a 32-bit hash of the key */
u32                 sat_CacheKeyHash(const struct sat_CacheKey *key);
/* Return 1 if the two keys are equivalent, otherwise return  0. */
u32                 sat_CacheKeyEquivalence(const struct sat_CacheKey *keyA, const struct sat_CacheKey *keyB);

/*
sat_Cache
=========
Internal physics engine struct for caching SAT-based contact calculations
each frame.
*/
enum sat_CacheType
{
    SAT_CACHE_NOT_SET,      /* Cache not set            */
	SAT_CACHE_SEPARATION,   /* Seperation axis found    */
	SAT_CACHE_CONTACT_FV,   /* Face-Vertex Contact      */
	SAT_CACHE_CONTACT_EE,   /* Edge-Edge Contact        */
	SAT_CACHE_COUNT,
};

struct sat_Cache
{
    THASH_NODE;
    TPOOL_NODE;

	struct sat_CacheKey key;
	enum sat_CacheType	type;
	union
	{
		struct
		{
			u32 body;	/* body (0 or 1) containing face    */
			u32	face;	/* reference face 	                */
		};

		struct
		{
			u32	edge0;	/* body0 edge   */
			u32	edge1;	/* body1 edge   */
		};

		struct
		{
			vec3    separation_axis;
			f32	    separation;
		};
	};
};

TPOOL_DECLARE(sat_Cache)
THASH_DECLARE(sat_Cache, struct sat_CacheKey)

/* Alloc sat_Cache in pipeline. */
struct slot sat_CacheAdd(struct cdb *cdb, const struct sat_CacheKey *key);
/* Dealloc sat_Cache in pipeline. */
void        sat_CacheRemove(struct cdb *cdb, const u32 index);
/* Lookup sat_Cache in pipeline. If found, return (index, address). Otherwise (U32_MAX, NULL). */
struct slot sat_CacheLookup(struct cdb *cdb, const struct sat_CacheKey *key);

/*
contact_database
================
Database for last and current frame contacts. Any rigid body can lookup its cached
and current contacts, and if necessary, invalidate any contact data.
*/

struct cdb
{
	/* contact net list nodes are owned as follows:
	 *
	 *  contact->key.shape0 owns slot 0
	 *  contact->key.shape1 owns slot 1
	 *
	 * i.e. the smaller index owns slot 0 and the larger index owns slot 1.  */
	struct nll	                contact_net;
	struct ds_HashMap	        contact_map;		

	/* frame-cached separation axis results */
	struct sat_CacheTPool       sat_cache_pool;
	struct sat_CacheTHashMap    sat_cache_map;		

	/* PERSISTENT DATA, GROWABLE, keeps track of which slots in contact_net/sat_cache
     * from last frame that are still being used. At the end of every frame, it is
     * set to ***_frame_usage, after which and any new contacts/sat_Caches outside
     * of the slots covered by ***_frame_usage is appended.  
     */
	struct bitVec 	contact_persistent_usage; 
	struct bitVec 	sat_cache_persistent_usage; 

	/* FRAME DATA, NOT GROWABLE, keeps track of which slots in contact_net/sat_cache
     * in previous frame that are currently being used. Thus, all links in the current
     * frame are the ones in the bit array + any appended contacts/sat_caches which 
     * resulted in growing the array. */
	struct bitVec 	contact_frame_usage;	
	struct bitVec 	sat_cache_frame_usage;	

    /* FRAME DATA */
    u32     sat_cache_count;        /* Caches in the current frame              */
    u32     contact_count;          /* Contacts found in the current frame      */
	u32		contact_new_count;      /* New contacts found in the current frame  */
	u32 *   contact_new;
};

/* Allocate cdb resources */
struct cdb *cdb_Alloc(struct arena *mem_persistent, const u32 initial_size);
/* Deallocate cdb resources */
void 		cdb_Free(struct cdb *cdb);
/* Flush cdb resources */
void		cdb_Flush(struct cdb *cdb);
/* Validate cdb state */
void		cdb_Validate(const struct ds_RigidBodyPipeline *pipeline);
/* Flush cdb frame resources */
void		cdb_ClearFrame(struct cdb *cdb);

/*
ds_Island
=========
TODO remove and rewrite

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

struct ds_Island
{
	POOL_SLOT_STATE;
	DLL_SLOT_STATE;

	struct ds_RigidBody **	bodies;	
	struct ds_Contact 	**	contacts;
	u32 *			body_index_map; /* body_index -> local indices of bodies in island:
						 * is->bodies[i] = pipeline->bodies[b] => 
						 * is->body_index_map[b] = i 
						 */

	/* Persistent Island */
	u32 flags;

	struct dll	body_list;
	struct dll	contact_list;

//TODO RMEOVE
#ifdef DS_PHYSICS_DEBUG
	vec4 color;
#endif
};

struct isdb
{
	/* PERSISTENT DATA */
	struct ds_Pool 	island_pool;	/* GROWABLE, list nodes of contacts and bodies	*/
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
struct isdb	isdb_Alloc(struct arena *mem_persistent, const u32 initial_size);
/* Free any heap memory */
void	   	isdb_Dealloc(struct isdb *is_db);
/* Flush / reset the island database */
void		isdb_Flush(struct isdb *is_db);
/* Clear any frame related data */
void		isdb_ClearFrame(struct isdb *is_db);
/* remove island resources from database */
void 		isdb_IslandRemove(struct ds_RigidBodyPipeline *pipeline, struct ds_Island *is);
/* Debug printing of island */
void 		isdb_PrintIsland(FILE *file, const struct ds_RigidBodyPipeline *pipeline, const u32 island, const char *desc);
/* Check if the database appears to be valid */
void 		isdb_Validate(const struct ds_RigidBodyPipeline *pipeline);
/* Setup new island from single body */
struct ds_Island *	isdb_InitIslandFromBody(struct ds_RigidBodyPipeline *pipeline, const u32 body);
/* Add contact to island */
void 		isdb_AddContactToIsland(struct ds_RigidBodyPipeline *pipeline, const u32 island, const u32 contact);
/* Return island that body is assigned to */
struct ds_Island *	isdb_BodyToIsland(struct ds_RigidBodyPipeline *pipeline, const u32 body);
/* Merge islands (Or simply update if new local contact) using new contact */
void 		isdb_MergeIslands(struct ds_RigidBodyPipeline *pipeline, const u32 ci, const u32 b0, const u32 b1);
/* Split island, or remake if no split happens: TODO: Make thread-safe  */
void 		isdb_SplitIsland(struct arena *mem_tmp, struct ds_RigidBodyPipeline *pipeline, const u32 island_to_split);

/********* Threaded Island API *********/

struct ds_IslandSolveOutput
{
	u32 island;
	u32 island_asleep;
	u32 body_count;
	u32 *bodies;		/* bodies simulated in island */ 
	struct ds_IslandSolveOutput *next;
};

struct ds_IslandSolveInput
{
	struct ds_Island *is;
	struct ds_RigidBodyPipeline *pipeline;
	struct ds_IslandSolveOutput *out;
	f32 timestep;
};

/*
 * Input: struct ds_Island_solve_in 
 * Output: struct ds_IslandSolveOutput
 *
 * Solves the given island using the global solver config. Since no island shares any contacts or bodies, and every
 * island is a unique task, no shared variables are being written to.
 *
 * - reads pipeline, solver config, cdb, is_db (basically everything)
 * - writes to island,		(unique to thread, memory in cdb)
 * - writes to island->contacts (unique to thread, memory in cdb)
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
	f32	    restitution;	/* Range[0.0f, 1.0f] : higher => bouncy */
	//f32	tangent_impulse_bound;	/* TODO: contact_friction * gravity_constant * point_mass */
	f32	    friction;	/* TODO: friction = f32_max(b1->friction, b2->friction) */
	u32	    block_solve;	/* if config->block_solver && condition number of block normal mass is ok, then = 1 */
};

struct solver
{
	f32 			timestep;
	u32			body_count;
	u32			contact_count;

	struct ds_RigidBody **	    bodies;
    vec3ptr                     w_center_of_mass;   /* world-position center of mass of body */
	mat3ptr			            Iw_inv;		        /* inverted world inertia tensors */
	struct velocityConstraint * vcs;	

	/* temporary state of bodies in island, static bodies index last element */
	vec3ptr			linear_velocity;
	vec3ptr			angular_velocity;
};

struct solver *	SolverInitBodyData(struct arena *mem, struct ds_Island *is, const f32 timestep);
void 		SolverInitVelocityConstraints(struct arena *mem, struct solver *solver, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Island *is);
void 		SolverIterateVelocityConstraints(struct solver *solver);
void 		SolverWarmup(struct solver *solver, const struct ds_Island *is);
void 		SolverCacheImpulse(struct solver *solver, const struct ds_Island *is);

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

#define PHYSICS_EVENT_ISLAND(pipeline, event_type, island_index)					        \
	{												                                        \
		struct physicsEvent *__physics_debug_event = PhysicsPipelineEventPush(pipeline);	\
		__physics_debug_event->type = event_type;						                    \
		__physics_debug_event->island = island_index;						                \
	}

#ifdef DS_PHYSICS_DEBUG

#define	PhysicsEventBodyNew(pipeline, _body)		                                        \
	{												                                        \
		struct physicsEvent *__physics_debug_event = PhysicsPipelineEventPush(pipeline);	\
		__physics_debug_event->type = PHYSICS_EVENT_BODY_NEW;						        \
		__physics_debug_event->body = _body;						                        \
	}
#define	PhysicsEventBodyRemoved(pipeline, body_entity)                                      \
	{												                                        \
		struct physicsEvent *__physics_debug_event = PhysicsPipelineEventPush(pipeline);	\
		__physics_debug_event->type = PHYSICS_EVENT_BODY_REMOVED;						    \
		__physics_debug_event->entity = body_entity;						                \
	}
#define	PhysicsEventIslandAsleep(pipeline, island)	    PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_ASLEEP, island)
#define	PhysicsEventIslandAwake(pipeline, island)	    PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_AWAKE, island)
#define	PhysicsEventIslandNew(pipeline, island)		    PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_NEW, island)
#define	PhysicsEventIslandExpanded(pipeline, island)	PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_EXPANDED, island)
#define	PhysicsEventIslandRemoved(pipeline, island)	    PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_REMOVED, island)
#define PhysicsEventContactNew(pipeline, _contact)					                        \
	{												                                        \
		struct physicsEvent *__physics_debug_event = PhysicsPipelineEventPush(pipeline);	\
		__physics_debug_event->type = PHYSICS_EVENT_CONTACT_NEW;				            \
		__physics_debug_event->contact = _contact;				                            \
	}
#define PhysicsEventContactRemoved(pipeline, body0, shape0, body1, shape1)                  \
	{												                                        \
		struct physicsEvent *__physics_debug_event = PhysicsPipelineEventPush(pipeline);	\
		__physics_debug_event->type = PHYSICS_EVENT_CONTACT_REMOVED;				        \
		__physics_debug_event->contact_removed_bodies[0] = body0;				            \
		__physics_debug_event->contact_removed_bodies[1] = body1;				            \
		__physics_debug_event->contact_removed_shapes[0] = shape0;				            \
		__physics_debug_event->contact_removed_shapes[1] = shape1;				            \
	}

#else

#define	PhysicsEventBodyNew(pipeline, body)
#define	PhysicsEventBodyRemoved(pipeline, entity)
#define	PhysicsEventIslandAsleep(pipeline, island)
#define	PhysicsEventIslandAwake(pipeline, island) 
#define	PhysicsEventIslandNew(pipeline, island)   
#define	PhysicsEventIslandExpanded(pipeline, island)   
#define	PhysicsEventIslandRemoved(pipeline, island)
#define PhysicsEventContactNew(pipeline, contact)
#define PhysicsEventContactRemoved(pipeline, body0, shape0, body1, shape1)                

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
        u32                     entity;
		u32                     island;
		ds_RigidBodyId          body;
        ds_ContactId            contact;
        
        struct 
        {
            ds_RigidBodyId      contact_removed_bodies[2];
            ds_ShapeId          contact_removed_shapes[2];
        };
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

/*
 * Physics Pipeline
 */
struct ds_RigidBodyPipeline 
{
	struct arena 	frame;			        /* frame memory */

	u64				ns_start;		        /* external ns at start of physics pipeline */
	u64				ns_elapsed;		        /* actual ns elasped in pipeline (= 0 at start) */
	u64				ns_tick;		        /* ns per game tick */
	u64 			frames_completed;	    /* number of completed physics frames */ 

	struct strdb *	cshape_db;		        /* externally owned */
	struct strdb *	body_prefab_db;		    /* externally owned */

	struct ds_Pool	body_pool;
	struct dll		body_marked_list;	    /* bodies marked for removal */
	struct dll		body_non_marked_list;	/* bodies alive and non-marked  */

	struct ds_Pool	shape_pool;
	struct bvh 		shape_bvh;              /* dynamic bvh of shapes */

	struct ds_Pool	event_pool;
	struct dll		event_list;

	struct cdb *	cdb;
	struct isdb 	is_db;

	struct collisionDebug *	debug;
	u32			debug_count;

	//TODO temporary, move somewhere else.
	vec3 			gravity;	/* gravity constant */

	u32			    margin_on;
	f32			    margin;

	/* frame data */
	u32			    cm_count;
	struct c_Manifold *cm;
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
/* If hit, return parameter (shape,t) of ray at first collision. Otherwise return (U32_MAX, F32_INFINITY) */
u32f32 			PhysicsPipelineRaycastParameter(struct arena *mem_tmp1, struct arena *mem_tmp2, const struct ds_RigidBodyPipeline *pipeline, const struct ray *ray);
/* enable sleeping in pipeline */
void 			PhysicsPipelineSleepEnable(struct ds_RigidBodyPipeline *pipeline);
/* disable sleeping in pipeline */
void 			PhysicsPipelineSleepDisable(struct ds_RigidBodyPipeline *pipeline);
/* Print resource usage */
void            PhysicsPipelinePrintUsage(const struct ds_RigidBodyPipeline *pipeline);

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
