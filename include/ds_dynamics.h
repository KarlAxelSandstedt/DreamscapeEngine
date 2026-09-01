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
#include "ds_bitset.h"
#include "ds_job.h"

//TODO 
struct ds_RigidBodyPipeline;
struct ds_RigidBody;
struct ds_Island;
struct ds_ProxyRange;

/*
ds_NumericsConfig
==================
ds_NumericsConfig stores configurable values to be used in numerical calculations.
*/

#define DS_UNIT_M	1.0f
#define DS_UNIT_DM	0.1f
#define DS_UNIT_CM	0.01f
#define DS_UNIT_MM	0.001f

struct ds_NumericsConfig
{
    /* Maximum degrees allowed between two vectors for them to be considered parallel */
    f32 vec3_parallel_check_max_degrees_pending;
    f32 vec3_parallel_check_max_degrees;
    f32 vec3_parallel_check_eps;

    /* Maximum degrees allowed for temporal consistency between a cached and new manifold normal */
    f32 manifold_cache_normal_parallel_check_max_degrees_pending;
    f32 manifold_cache_normal_parallel_check_max_degrees;
    f32 manifold_cache_normal_parallel_check_eps;

    /* Maximum difference allowed for temporal consistency between a cached and new manifold depth */
    f32 manifold_cache_depth_max_diff_allowed_pending;
    f32 manifold_cache_depth_max_diff_allowed;

    /* 
     * Maximum velocity along the contact normal  allowed for temporal consistency between a cached and 
     * new manifold 
     */
    f32 manifold_cache_linear_velocity_max_diff_allowed_pending;
    f32 manifold_cache_linear_velocity_max_diff_allowed;

    /* Fraction of dirty dbvh proxies that may be re-inserted before triggering a dbvh rebuild  */
    f32 dbvh_reinsert_threshold_pending;
    f32 dbvh_reinsert_threshold;

    /* Max s_SatCache count per contact (Applies to mesh contacts in which we cache triangle ops)  */
    u32 cache_count_max_pending;
    u32 cache_count_max;
};
extern struct ds_NumericsConfig *g_numerics_config;

/* Return default config */
struct ds_NumericsConfig    ds_NumericsConfigDefault(void);
/* Update any pending values in config and push config to global */
void                        ds_NumericsConfigPush(struct ds_NumericsConfig *config);
/* Pop global config */
void                        ds_NumericsConfigPop(void);


/*
ds_Id
=====
Opaque generation based handles for user-interfacing structures. ds_Id supports
16-bit generations, and 32-bit indices. ds_IdF (frequent) supports 32-bit 
generations and 32-bit indices.
*/

typedef u64     ds_Id;
typedef u64     ds_IdF; /* ds_IdF (frequent) */

typedef ds_Id   ds_ShapeId;
typedef ds_Id   ds_RigidBodyId;
typedef ds_Id   ds_IslandId;
typedef ds_Id   ds_JointId;
typedef ds_IdF  ds_ContactId;

#define DS_ID_NULL                      U64_MAX
#define DS_ID_INDEX_MASK                ((u64) 0x00000000ffffffff)
#define DS_ID_TAG_MASK                  ((u64) 0xffffffff00000000)
#define DS_ID_GENERATION_INCREMENT      ((u64) 0x0001000000000000)

#define ds_IdTag(id)                    ((u32) ((id) >> 32))
#define ds_IdIndex(id)                  ((u32) (id))
#define ds_IdConstruct(index, tag)      ((((u64) (tag)) << 32) | (u64) (index))

#define DS_IDF_NULL                     U64_MAX
#define DS_IDF_INDEX_MASK               ((u64) 0x00000000ffffffff)
#define DS_IDF_GENERATION_MASK          ((u64) 0xffffffff00000000)
#define DS_IDF_GENERATION_INCREMENT     ((u64) 0x0000000100000000)

#define ds_IdFGeneration(id)            ((u32) ((id) >> 32))
#define ds_IdFIndex(id)                 ((u32) (id))
#define ds_IdFConstruct(index, tag)     ((((u64) (tag)) << 32) | (u64) (index))

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
or she so wishes. Hence, we cannot assume the local frame of the body to always
have the center of mass as its origin. Thus, in addition to storing the local-to-world
transform, ds_RigidBody must also store its center of mass:

	ds_RigidBody
	{
		(...)
		ds_Transform	transform;	    // Local frame to World transform
		vec3            center_of_mass;	
	}
*/

struct ds_Shape
{
	POOL_NODE;
    struct ds_DLLNode body_shape;

    ds_ShapeId      id;                 /* Generational identifier                          */
	u32 			body;		        /* ds_RigidBody owner of node 			            */
    struct ds_DLL   contact_list;       /* list of the shape's contacts                     */

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
POOL_DECLARE(ds_Shape);

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
    SDB_NODE;

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
SDB_DECLARE(ds_ShapePrefab);

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
ds_ShapeId              ds_ShapeAdd(struct ds_RigidBodyPipeline *pipeline, const struct ds_ShapePrefab *prefab, const ds_Transform *t, const ds_RigidBodyId body);
/* 
 * INTERNAL: Remove the specified shape of a DYNAMIC body and update the island database and contact database state.  
 */
void                    ds_ShapeDynamicRemove(struct ds_RigidBodyPipeline *pipeline, struct ds_RigidBody *body, const u32 shape_index, const u32 update_mass_properties);
/* 
 * INTERNAL: Remove the specified shape of a STATIC body and update the physics state into a valid state. 
 */
void        ds_ShapeStaticRemove(struct arena *mem_tmp, struct ds_RigidBodyPipeline *pipeline, struct ds_RigidBody *body, const u32 index);
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
 * Test for intersection between shapes. returns 1 if intersecting, else 0 
 */
u32	        ds_ShapeTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *s1, const struct ds_Shape *s2);
/* 
 * Return, if no intersection was found, the distance between shapes s1 and s2 and their respective
 * closest points c1 and c2. If the shapes are intersecting, return 0.0f. 
 */
f32 	    ds_ShapeDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *s1, const struct ds_Shape *s2);
/* 
 * Run the contact's collision computation and set its manifold(s) and cache(s).
 */
void        ds_ShapeContact(struct arena *frame, const struct ds_RigidBodyPipeline *pipeline, const u32 contact_index);
/* 
 * Return, if ray intersects shape, t such that ray.origin + t*ray.dir == closest point on shape. 
 *         Otherwise, return F32_INFINITY.
 */
f32 	    ds_ShapeRaycastParameter(const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *shape, const struct ray *ray);
/* 
 * Return 1 if ray hit shape, 0 otherwise. If hit, we return the closest intersection point 
 */
u32 	    ds_ShapeRaycast(vec3 intersection, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *shape, const struct ray *ray);


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
    SDB_NODE;

    struct dll      shape_list;         /* shape prefab instance list */
    ds_RigidBodyId  body;
    
	u32	            dynamic;	        /* dynamic body is true, static if false */

    //TODO pre-compute...?
	//(f32 	        mass;			    /* total body mass */
	//mat3 	        inv_inertia_tensor;
};
SDB_DECLARE(ds_RigidBodyPrefab);

/*
rigid_body
========== 
//TODO
*/

#define RB_DYNAMIC		((u32) 1 << 1)

#define RB_IS_STATIC(b)	    (!((b)->flags & RB_DYNAMIC))
#define RB_IS_DYNAMIC(b)	((b)->flags & RB_DYNAMIC)

#define RB_DYNAMIC_BIT(b)	(((b)->flags & RB_DYNAMIC) >> 1u)

#define IS_DYNAMIC(flags)	(((flags) & RB_DYNAMIC) >> 1u)

struct ds_RigidBody
{
	POOL_NODE;
	struct ds_DLLNode island_body;	        /* island body_list node */

    ds_RigidBodyId  id;                     /* generational identifier */
	u32 		    flags;
	u32		        island;

    u32             set;                    /* ds_SolverSet index                                   */
    u32             sim;                    /* ds_SolverSet data index                              */ 
	f32 		    low_velocity_time;	    /* Current uninterrupted time body has been in a low velocity state */

    struct ds_DLL   joint_list;             /* list of ds_Joint's attached to the body. Each joint is
                                               shared with one other body. */

	struct ds_DLL   shape_list;		        /* list of convex shapes constructing the rigid body 	*/

    vec3		    local_center_of_mass;	/* local body frame center of mass 			            */
	mat3 		    inv_inertia_tensor;
	f32 		    mass;			        /* total body mass */
	u32 	        entity;
};
POOL_DECLARE(ds_RigidBody);


/*
ds_RigidBodySim
===============
*/
struct ds_RigidBodySim
{
    u32             body;                   /* RigidBody index */
    u32             flags;
    f32             inv_mass;               /* TODO Inverse mass */
	ds_Transform    world;		            /* local body frame to world transform. Rotation is 
                                               about the local origin (not center of mass!)         */
	vec3		    local_center_of_mass;	/* local body frame center of mass */
	vec3		    world_center_of_mass;	/* world body frame center of mass */
	mat3 		    local_inv_inertia;      /* TODO local inertia tensor */
	mat3 		    world_inv_inertia;      /* TODO world inertia tensor */
};
DEFINE_CPOOL_STRUCT(ds_RigidBodySim);


/*
ds_RigidBodyCompute
===================
*/
struct ds_RigidBodyCompute
{
	vec3 		    linear_velocity;        /* linear velocity of body */
	vec3 		    angular_velocity;       /* angular velocity of body (about local center of mass,
                                               not local origin!)                                   */
    vec3            center_of_mass;         /* world-space  */
    quat            rotation;
    u32             flags;

    u8              pad[8];
};
DEFINE_CPOOL_STRUCT(ds_RigidBodyCompute);

//TODO
ds_RigidBodyId  ds_RigidBodyAdd(struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBodyPrefab *prefab, const ds_Transform *t_world, const u32 entity);
/* Free the given body */
void            ds_RigidBodyRemove(struct arena *mem_tmp, struct ds_RigidBodyPipeline *pipeline, const ds_RigidBodyId id);
/* Lookup the given body and return it. If it does not exist, return DS_ID_NULL.  */
struct slot	    ds_RigidBodyLookup(const struct ds_RigidBodyPipeline *pipeline, const ds_RigidBodyId id);
/* Process the body's shape list and set its internal mass properties accordingly. */
void		    ds_RigidBodyUpdateMassProperties(struct ds_RigidBodyPipeline *pipeline, const ds_RigidBodyId id);
/* Internal: Refresh and update rigid body simulation and compute/solver data in range [low, high) before solving */
void            ds_RigidBodyUpdateSolverDataRange(struct ds_RigidBodyPipeline *pipeline, const u32 low, const u32 high);
/* Internal: Integrate body velocites in range [low, high) */
void            ds_RigidBodyIntegrateVelocitiesRange(struct ds_RigidBodyPipeline *pipeline, const u32 low, const u32 high);
/* Internal: Update orientation of active bodies in range [low, high) */
void            ds_RigidBodyUpdateOrientationRange(struct ds_RigidBodyPipeline *pipeline, struct ds_ProxyRange *proxy_range, const u32 low, const u32 high);


/*
c_SatCache
==========
Internal physics engine struct for caching SAT-based contact calculations each frame. If a contact was found,
the contact results may be re-used the next frame if a set of conditions are fullfilled.

::: Internals :::

To ensure temporal coherence when reusing contact information, we currently test:

  1. A penetration check for the cached features: Are we even still penetrating for the given features?

  2. Has the new contact normal drifted to much from the cached normal? The number of degrees-drift per frame 
     is found in ds_NumericsConfig.

  3. Is the cached feature that produced the deepest point still the deepest feature in the new contact? This
     test is only relevant for face contacts, as the deepest point may come from the incident body's vertex set
     or from the body's edge set as a clipped point.

  4. We allow a maximum change in penetration depth per frame, governed by a value in ds_NumericsConfig. This test
     may be unnecessary as we are already doing a linear velocity check.

  5. We allow a maximum difference in velocity between the two bodies along the contact normal. If the separating
     velocity is high, the chances are that our manifold is not coherent and we may as well not even try to rebuilt
     it (a rather costly endevaour). As in the other case, the max difference is found in ds_NumericsConfig.
*/

enum sat_CacheType
{
	SAT_CACHE_SEPARATION,   /* Seperation axis found    */
	SAT_CACHE_CONTACT_FV,   /* Face-Vertex Contact      */
	SAT_CACHE_CONTACT_EE,   /* Edge-Edge Contact        */
	SAT_CACHE_COUNT,
};

typedef u32 sat_FeatureId;

#define SAT_FEATURE_ID_TYPE_MASK            (0xc0000000)
#define SAT_FEATURE_ID_INDEX_MASK           (0x3fffffff)

#define SAT_FEATURE_TYPE_FACE               2
#define SAT_FEATURE_TYPE_EDGE               1
#define SAT_FEATURE_TYPE_VERTEX             0

#define sat_FeatureIdVertexCheck(id)        (((id) >> 30) == SAT_FEATURE_TYPE_VERTEX)
#define sat_FeatureIdEdgeCheck(id)          (((id) >> 30) == SAT_FEATURE_TYPE_EDGE)
#define sat_FeatureIdFaceCheck(id)          (((id) >> 30) == SAT_FEATURE_TYPE_FACE)

#define sat_FeatureIdType(id)               ((id) >> 30)
#define sat_FeatureIdIndex(id)              ((id) & SAT_FEATURE_ID_INDEX_MASK)
#define sat_FeatureIdConstruct(index, type) ((((u32) type) << 30) | (SAT_FEATURE_ID_INDEX_MASK & (index)))

struct c_SatCache
{
    u32 tri;                    /* (Optional): Triangle in mesh. */
	enum sat_CacheType	type;
    
    /*
     * type == FACE:
     *  feature[i].type == FACE => i IS NOT incident face
     *  feature[i].type != FACE => i IS on incident face
     *  normal == face normal
     *
     * type == EDGE:
     *  feature[i].type == EDGE
     *  feature[i].type == EDGE
     *  normal == contact normal
     *
     * type == SEPARATION:
     *      normal == separation axis pointing from reference body 
     *      depth == separation distance (positive)
     */
    sat_FeatureId   feature[2];
    f32             depth;      
    vec3            normal;     
};


/*
ds_ContactKey
=============
The key is defined by the canonical form (s0,s1), where s0 < s1 and is used to lookup a contact
between shapes s0, s1.
*/
struct ds_ContactKey
{
    u32 shape[2];       /* Canonical ordering: s[0] < s[1] */
};

/* Return the Canonical key of (shape0, shape1) */
struct ds_ContactKey    ds_ContactKeyCanonical(const u32 shape0, const u32 shape1);
/* Return a 32-bit hash of the key */
u32                     ds_ContactKeyHash(const struct ds_ContactKey key);
/* Return 1 if the two keys are equivalent, otherwise return  0. */
u32                     ds_ContactKeyEquivalence(const struct ds_ContactKey key0, const struct ds_ContactKey key1);
/* Return the body and shape addresses of the key */
void                    ds_ContactKeyAddress(struct ds_RigidBody **b0, struct ds_Shape **s0, struct ds_RigidBody **b1, struct ds_Shape **s1, const struct ds_RigidBodyPipeline *pipeline, const struct ds_ContactKey key);
/* Validate contact state */
void		            ds_ContactValidateAll(const struct ds_RigidBodyPipeline *pipeline);


/*
ds_Contact
==========
Every touching and non-touching contact is a persistent ds_Contact. The structure stores general
connectivity data and where to find its simulation/compute data. 

    :: Every non-touching contact have a link (<->) stored in the active set, and have no compute
       data attached to it. It it not associated with an island. Any ds_SatCache that belongs to 
       the contact is alive for 2 frames and is renewed every frame.

    :: All contacts in an Awake island are stored in the constraint graph with a link (<->) and
       compute data. Any ds_SatCache that belongs to the contact is alive for 2 frames and is
       renewed every frame.

    :: All contacts in a sleeping island are stored in the island's sleeping set, with a link (<->)
       and compute data. Any ds_SatCache that belongs to the contact is moved onto the heap. 
*/
struct ds_Contact
{
    POOL_NODE;      

    ds_ContactId                id;                 /* Generational Identifier                  */
    struct ds_ContactKey        key;                /* shape pair key                           */
    
    u32                         island;             /* Index of contact's island                */
    u32                         set;                /* Index of contact's set                   */
    u32                         compute;            /* Index of contact's set OR color data     */
    u32                         color;              /* Index of contact's color (mesh triangle
                                                       contacts are treated as a single 
                                                       constraint!)                             */

    struct ds_DLLNode           island_contact;     /* island->contact_list node                */
    struct ds_DLLNode           shape_contact[2];   /* [i] is part of shape i's contact list    */

    /* 
     * The following data is regenerated per-frame and is alive for 2 frames. All data is 
     * pushed onto thread's current frame arena (double buffered). If the contact is sleeping,
     * the narrowphase is stored on the heap.
     */
    struct c_ContactResult      narrowphase;
};
POOL_DECLARE(ds_Contact);

/* Add and return new contact with unique key and update pipeline state */
struct slot ds_ContactAdd(struct ds_RigidBodyPipeline *pipeline, const struct ds_ContactKey key);
/* Remove contact at the given index and update pipeline state */
void 	    ds_ContactRemove(struct ds_RigidBodyPipeline *pipeline, const u32 contact_index);
/* Return the contact associated with the given id. If no such contact is found, return (U32_MAX, NULL) */
struct slot ds_ContactLookup(const struct ds_RigidBodyPipeline *pipeline, const ds_ContactId id);
/* Update contact at the given slot and update pipeline state. */
struct slot ds_ContactKeyLookup(const struct ds_RigidBodyPipeline *pipeline, const struct ds_ContactKey key);

/* Internal: Return 1 if contact shape bvh nodes still overlap, otherwise return 0. */
u32         ds_ContactCheckBvhOverlap(const struct ds_RigidBodyPipeline *pipeline, const u32 contact);
/* Internal: Promote contact from non-touching active set to color in constraint graph */
void        ds_ContactPromote(struct ds_RigidBodyPipeline *pipeline, const u32 contact);
/* Internal: Demote contact to non-touching active set from color in constraint graph */
void        ds_ContactDemote(struct ds_RigidBodyPipeline *pipeline, const u32 contact);
/* Internal: Wakeup contact from sleeping set */
void        ds_ContactWakeUp(struct arena *frame, struct ds_RigidBodyPipeline *pipeline, const u32 contact);
/* Internal: Put contact to sleep in sleeping set */
void        ds_ContactSleep(struct arena *mem_sleep, struct ds_RigidBodyPipeline *pipeline, const u32 contact, const u32 set);
/* Internal: Return bytes required to store contact narrowphase results */
u64         ds_ContactMemoryRequirement(const struct ds_RigidBodyPipeline *pipeline, const u32 contact);


/*
ds_ContactConstraintCache
=========================
TODO:
*/
struct ds_ContactConstraintCache
{
	vec3 			        normal;                     /* Cached contact normal                */
	vec3 			        tangent[2];                 /* Froms Contact basis with normal      */
	vec3 			        r1[4];			            /* previous local frame arm levers      */
    vec3                    r2[4];                   
	f32 			        tangent_impulse[4][2];
	f32 			        normal_impulse[4];	        /* contact_solver solution to contact 
                                                           constraint, or 0.0f                  */
    u32                     v_count;                    /* Number of vertices in cache          */
    u32                     tri;                        /* 0 if non-mesh contact                */
};


/*
ds_ContactCompute
=================
TODO: contact <-> compute <-> constraints?
*/
struct ds_ContactCompute
{
    struct ds_ContactConstraint *       cc;
    struct ds_ContactConstraintCache *  ccache;
    u32                                 cc_count;
    u32                                 ccache_count;
    u32                                 body_sim[2];    /* body->sim values of the two bodies in contact   */
};
DEFINE_CPOOL_STRUCT(ds_ContactCompute);


/*
ds_Joint
========
Library API level representation of all joint types. A ds_Joint constrain two rigid bodies according to its type,
and should be viewed as data co-owned by the two bodies. Internally, it stores book-keeping data, while physics
related data is stored in the ds_JointSim struct. It, in addition to the ds_Contact struct, also represents an
edge in the constraint graph (ds_CGraph), 
*/

enum ds_JointType
{
    DS_JOINT_TYPE_NONE,
    DS_JOINT_TYPE_DISTANCE,
    DS_JOINT_TYPE_COUNT
};

struct ds_Joint
{
    POOL_NODE;
    ds_JointId          id;                    /* generational identifier */

    u32                 island;
    struct ds_DLLNode   island_joint;

    u32                 body[2];        /* bodies sharing ownership of joint                                */
    struct ds_DLLNode   edge_node[2];   /* Each body stores the joint its dll; b == body[i] => edge_node[i] 
                                           is part of body b's dll.                                         */

    //TODO u32         island;     /* */

    u32         set;                    /* index to joint's set                                             */
    u32         color;                  /* edge color (or CG_COLOR_NOT_SET)                                 */
    u32         sim;                    /* owned ds_JointSim. If color is a valid color, the JointSim is 
                                           found in in the given color of the ds_CGraph. if 
                                           color == CG_COLOR_NOT_SET, sim is an index into the joint's 
                                           set->jointsim_pool.                                              */
};
POOL_DECLARE(ds_Joint);

/* 
 * Setup a joint between bodies b0 and b1 with anchors defined by the input local_frames. On success, 
 * a valid ds_JointId is returned. On Failure, DS_ID_NULL is returned.
 */
ds_JointId  ds_JointAdd(struct ds_RigidBodyPipeline *pipeline, const ds_RigidBodyId b0, const ds_Transform *t0, const ds_RigidBodyId b1, const ds_Transform *t1);
/*
 * Remove the specified joint corresponding to the id. If the joint no longer exist, the call becomes a NO-OP.
 */
void        ds_JointRemove(struct ds_RigidBodyPipeline *pipeline, const ds_JointId id);
/* 
 * On success, return the joint corresponding to the id. If the wasn't found, return an empty slot (U32_MAX, NULL)
 */
struct slot ds_JointLookup(const struct ds_RigidBodyPipeline *pipeline, const ds_JointId id);
/* 
 * INTERNAL: Remove the specified joint of a STATIC-DYNAMIC body pair and update the pipeline into a valid state.
 */
void        ds_JointStaticRemove(struct ds_RigidBodyPipeline *pipeline, struct ds_RigidBody *body, const u32 index);
/* 
 * INTERNAL: Remove the specified joint of a DYNAMIC-DYNAMIC body pair and update the pipeline into a valid state.
 */
void        ds_JointDynamicRemove(struct ds_RigidBodyPipeline *pipeline, struct ds_RigidBody *body, const u32 index);

/*
ds_DistanceJoint
================
//TODO
*/

struct ds_DistanceJointPrefab
{
    u32 tmp;
};

//TODO
struct ds_DistanceJoint
{
    u32 tmp;
};

/*
 * Set prefab to the default distance joint values.
 */
void    ds_DistanceJointPrefabDefault(struct ds_DistanceJointPrefab *prefab);
/* 
 * Setup a joint between bodies b0 and b1 with anchors defined by the input local_frames. On success, 
 * a valid ds_JointId is returned. On Failure, DS_ID_NULL is returned.
 */
ds_JointId ds_DistanceJointAdd(struct ds_RigidBodyPipeline *pipeline, const struct ds_DistanceJointPrefab *prefab, const ds_RigidBodyId b0, const ds_Transform *local_frame0, const ds_RigidBodyId b1, const ds_Transform *local_frame1);

/*
ds_JointSim
===========
ds_JointSim is a discriminating union storing all different types of physical joints. 
*/
struct ds_JointSim
{
    /* joint frame to body frame transform */
    ds_Transform        local_frame[2];
    
    u32                 joint;      /* index of general joint owning the struct */
    enum ds_JointType   type;
    union
    {
        struct ds_DistanceJoint distance;
    };
};
DEFINE_CPOOL_STRUCT(ds_JointSim);

/*
ds_SolverSet
============
//TODO 

::: Internals :::

Context: Unlike Box3d, we do not store non-touching contacts; instead, we run a full dynamic tree search for
potential contacts, and then run narrowphase on overlapping leaves. For any new contacts we then merge islands.
For any contact that has disappeared, we flag the merged/persistent island for splitting. Then we try to split
any flagged island. Then we solve any awake island. This is a simplified physics solver for now, so we try to
keep this structure unless you recommend us switching.

Furthermore, we do not at the moment store body data in the sets, only contact indices for sleeping sets
and ds_JoinSim for sleeping/disabled sets, and any islands in the set.  we will expand this struct as needed.

SOLVER_SET_DISABLED: TODO

SOLVER_SET_ACTIVE: TODO

SOLVER_SET_STATIC: TODO

SOLVER_SET_SLEEPING: TODO
*/

enum ds_SolverSetType
{
    SOLVER_SET_DISABLED,
    SOLVER_SET_STATIC,
    SOLVER_SET_ACTIVE,
    SOLVER_SET_SLEEPING_FIRST,
    SOLVER_SET_NULL = U32_MAX,
};

#define ACTIVE_BODY_DUMMY_INDEX U32_MAX

struct ds_SolverSet
{
    POOL_NODE;

    /* Sleeping set memory */
    struct arena                    mem;

    /* Body simulation state */
    ds_CPool(ds_RigidBodySim)       body_sim_pool;

    /* Body solver computation state */
    ds_CPool(ds_RigidBodyCompute)   body_compute_pool;

    /* Contact indices.  */
    ds_CPool(u32)                   contact_pool;
    
    /* Disabled/Sleep set stores non-active contacts that has been removed from the constraint graph */
    ds_CPool(ds_ContactCompute)     contact_compute_pool;

    /* Disabled/Sleep set stores non-active joints that has been removed from the constraint graph */
    ds_CPool(ds_JointSim)           joint_sim_pool;

    /* Islands in set */
    ds_CPool(u32)                   island_pool;
};
POOL_DECLARE(ds_SolverSet);


/* Allocate and setup a ds_SolverSet. If mem_set is provided, it will be used for allocating resources */
struct slot ds_SolverSetAdd(struct arena *mem_set, struct ds_RigidBodyPipeline *pipeline, const u32 initial_body_sim_count, const u32 initial_body_compute_count, const u32 initial_contact_count, const u32 initial_contact_compute_count,  const u32 initial_joint_count, const u32 initial_island_count);
/* Deallocate a the given ds_SolverSet */
void        ds_SolverSetRemove(struct ds_RigidBodyPipeline *pipeline, const u32 index);
/* Flush the given ds_SolverSet */
void        ds_SolverSetFlush(struct ds_RigidBodyPipeline *pipeline, const u32 index);
/* Wake up the given sleeping ds_SolverSet. If the solver set is not sleeping set, the call becomes a NO-OP. */
void        ds_SolverSetWakeUp(struct ds_RigidBodyPipeline *pipeline, const u32 index);
/* Try put the given island to sleep. On success, the island is moved from the active set to a sleeping set. */
void        ds_SolverSetSleep(struct ds_RigidBodyPipeline *pipeline, const u32 island);
/* Return the required memory size for putting the given island to sleep */
u64         ds_SolverSetSleepMemoryRequirement(const struct ds_RigidBodyPipeline *pipeline, const u32 island);
/* Debug validation for the given set */
void        ds_SolverSetValidate(const struct ds_RigidBodyPipeline *pipeline, const u32 set_index);

/* Internal: Move the body to the given set (Assumes the body is NOT part off the set) */
void        ds_SolverSetMoveBody(struct ds_RigidBodyPipeline *pipeline, const u32 body, const u32 set);


/*
ds_Island
=========
TODO 
*/

struct ds_Island
{
    ds_IslandId     id;                         /* generational identifier */

    POOL_NODE;
	u32             constraint_remove_count;    /* Constraints removed counter */
    u32             set;                        /* ds_SolverSet index */
    u32             set_island_index;           /* index into set.island_pool */

	struct ds_DLL	body_list;
	struct ds_DLL	contact_list;
    struct ds_DLL   joint_list;

//TODO RMEOVE
	vec4 color;
};
POOL_DECLARE(ds_Island);

/*  Return the island corresponding the id; If it doesn't exist, return (NULL, U32_MAX). */
struct slot ds_IslandLookup(struct ds_RigidBodyPipeline *pipeline, const ds_IslandId id);
/* remove island resources from database */
void 		ds_IslandRemove(struct ds_RigidBodyPipeline *pipeline, const u32 island);
/* Merge islands (Or simply update if new local contact) using new contact */
void 		ds_IslandMerge(struct ds_RigidBodyPipeline *pipeline, const u32 expand, const u32 merge);
/* Split island, or remake if no split happens.  */
void 		ds_IslandSplit(struct ds_RigidBodyPipeline *pipeline, const u32 island);
/* Debug printing of island */
void 		ds_IslandPrint(FILE *file, const struct ds_RigidBodyPipeline *pipeline, const u32 island, const char *desc);
/* Check if the database appears to be valid */
void 		ds_IslandValidateAll(const struct ds_RigidBodyPipeline *pipeline);


/*
ds_ContactConstraintPoint
=========================
Individual constraint point within a contact constraint.
*/
struct ds_ContactConstraintPoint 
{
    vec3    v;                  /* contact point                                */
    vec3    r[2];               /* levers: body center to contact_point         */
	f32 	normal_impulse;	    /* Normal impulse produced by the contact       */
	f32	    velocity_bias;	    /* scale of velocity_bias along contact normal  */
	f32	    normal_mass;	    /* 1.0f / row(J,i)*Inv(M)*J^T                   */
	f32	    tangent_mass[2];    /* 1.0f / row(J_tangent,i)*Inv(M)*J_tangent^T t */
	f32	    tangent_impulse[2]; /* the tangent impulses produced by the contact */
};


/*
ds_ContactConstraint 
====================
Each individual ds_Contact in the constraint graph may index its ds_ContactConstraint within
the contact's color. The ds_ContactConstraint stores all necessary data for the solver to solve
the contact's contact points.
*/

struct ds_ContactConstraint 
{
	u32 	ccp_count;	 /* Number of contact points in the manifold        */
	struct ds_ContactConstraintPoint ccp[4];

    //TODO make mat(3?)
	void * 	normal_mass;	/* mat2, mat3 or mat4 normal mass for block solver = Inv(J*Inv(M)*J^T) */
    //TODO make mat(3?)
	void * 	inv_normal_mass;/* mat2, mat3 or mat4 inv normal mass for block solver = J*Inv(M)*J^T */

	/* contact base axes */
	vec3 	normal;		/* Currently shared contact manifold normal between all point constraints */
	vec3	tangent[2];	/* normalized friction directions of contact */

	f32	    restitution;	/* Range[0.0f, 1.0f] : higher => bouncy */
	//f32	tangent_impulse_bound;	/* TODO: contact_friction * gravity_constant * point_mass */
	f32	    friction;	/* TODO: friction = f32_max(b1->friction, b2->friction) */
};
DEFINE_CPOOL_STRUCT(ds_ContactConstraint);


/* Initalize the given range [low, high) of ds_ContactConstraints in the constraint graph */
void 	ds_ContactConstraintInitRange(struct ds_RigidBodyPipeline *pipeline, const u32 color, const u32 low, const u32 high);
/* Warmup Range [low, high) of applicable ds_ContactConstraints for the given color in the constraint graph */
void 	ds_ContactConstraintWarmupRange(struct ds_RigidBodyPipeline *pipeline, const u32 color, const u32 low, const u32 high);
/* Compute a solver iteration over the given color for contact constraints */
void    ds_ContactConstraintIterateRange(struct ds_RigidBodyPipeline *pipeline, const u32 color, const u32 cc_low, const u32 cc_high);
/* Cache contact impulses and initialize position constraints in the range [low, high) for the given color */
void ds_PositionConstraintInitAndCacheImpulsesRange(struct ds_RigidBodyPipeline *pipeline, const u32 color, const u32 low, const u32 high);
/* Compute a solver iteration over the given color for position constraints in range [low, high) */
void ds_PositionConstraintIterateRange(struct ds_RigidBodyPipeline *pipeline, const u32 color, const u32 low, const u32 high);

/*
contact_solver_config
=====================
Mumerical parameters configuration for solving islands.
*/

struct solverConfig
{
	u32 	pgs_iteration_count;	/* velocity solver iteration count */
	u32 	ngs_iteration_count;	/* position solver iteration count */
	u32 	warmup_solver;		/* bool : Should warmup solver when applicable */
	vec3 	gravity;
	f32 	baumgarte_constant;  	/* Range[0.0, 1.0] : Determine how quickly contacts are resolved, 1.0f max speed */
    f32     max_linear_correction;           /* Range[0.0, inf] : max linear correction of constraint per iteration in the position solver */
    f32     max_linear_velocity_magnitude_inv;   /* Range[0.0, inf) :  max units (m) per second a body may travel */
    f32     max_angular_velocity_magnitude_inv;  /* Range[0.0, inf) : max units (radians) per second a body may travel */
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
	u32 	pending_warmup_solver;		
	u32 	pending_sleep_enabled;		
	u32 	pending_pgs_iteration_count;
	u32 	pending_ngs_iteration_count;
	f32 	pending_baumgarte_constant;
	f32 	pending_linear_slop;
	f32 	pending_restitution_threshold;
	f32 	pending_linear_dampening;
	f32 	pending_angular_dampening;
};

extern struct solverConfig *g_solver_config;

void    SolverConfigInit(const u32 pgs_iteration_count, 
                         const u32 ngs_iteration_count, 
                         const u32 warmup_solver, 
                         const vec3 gravity, 
                         const f32 baumgarte_constant, 
                         const f32 max_linear_correction, 
                         const f32 max_linear_velocity_magnitude, 
                         const f32 max_angular_velocity_magnitude, 
                         const f32 linear_dampening, 
                         const f32 angular_dampening, 
                         const f32 linear_slop, 
                         const f32 restitution_threshold, 
                         const u32 sleep_enabled, 
                         const f32 sleep_time_threshold, 
                         const f32 sleep_linear_velocity_sq_limit, 
                         const f32 sleep_angular_velocity_sq_limit);

/*
ds_CGraphColor
==============
//TODO
ds_CGraphColor stores the relevant physics data of active constraints, tightly packed for quick iterations.
*/
struct ds_CGraphColor
{
    struct ds_BitSet                body_bitset;
    ds_CPool(ds_ContactCompute)     contact_compute_pool;
    ds_CPool(u32)                   contact_pool;
    ds_CPool(ds_JointSim)           joint_sim_pool;
};

#define CG_COLOR_COUNT          12
#define CG_STATIC_COLOR_COUNT   4
#define CG_DYNAMIC_COLOR_COUNT  (CG_COLOR_COUNT - CG_STATIC_COLOR_COUNT - 1) 
#define CG_STATIC_COLOR_FIRST   0
#define CG_STATIC_COLOR_LAST    (CG_STATIC_COLOR_FIRST + CG_STATIC_COLOR_COUNT - 1)
#define CG_DYNAMIC_COLOR_FIRST  (CG_STATIC_COLOR_LAST + 1)
#define CG_DYNAMIC_COLOR_LAST   (CG_DYNAMIC_COLOR_FIRST + CG_DYNAMIC_COLOR_COUNT - 1)
#define CG_SERIAL_COLOR         (CG_COLOR_COUNT-1) 
#define CG_INVALID_COLOR        CG_COLOR_COUNT

/*
ds_CGraph
=========
ds_CGraph is a persistent graph that models all constraints in the pipeline. Each implicit vertex
in the graph represents a rigid body, and  each edge between two bodies represents a constraint
from either a contact or a joint. Whenever a new constraint is added between two bodies, it is 
assigned a unique color from all the other constraints that shares bodies (vertices) with it. If
we have exhausted the available colors, we assign the constraint to the serial color. 

Now that we have every constraint colored, We pay process all constraints with the same color in
parallel (expect the serial color). For good information on this, see Erin Catto's post "SIMD Matters".

Furthermore, similar to Box3D, in order to mitigate ghost collisions, we prioritize static-dynamic 
constraints by processing them first in each solver iteration. This yields a process ordering:

    CG_SERIAL_COLOR => CG_STATIC_COLOR_1 => ... => CG_STATIC_COLOR_N => CG_DYNAMIC_1 => .. CG_DYNAMIC_M
*/
struct ds_CGraph
{
    struct ds_CGraphColor color[CG_COLOR_COUNT];
};

/* Allocate and setup the pipeline's constraint graph */
void                    ds_CGraphAlloc(struct ds_RigidBodyPipeline *pipeline, const u32 initial_count);
/* Deallocate the pipeline's constraint graph */
void                    ds_CGraphDealloc(struct ds_RigidBodyPipeline *pipeline);
/* Flush the pipeline's constraint graph data */
void                    ds_CGraphFlush(struct ds_RigidBodyPipeline *pipeline);
/* Validate the state of the pipeline's constraint graph */
void                    ds_CGraphValidate(const struct ds_RigidBodyPipeline *pipeline);
/* Prepare the pipeline's constraint graph for the new frame, allocating and setting up new resources if necessary. */
void                    ds_CGraphFramePrepare(struct ds_RigidBodyPipeline *pipeline);
/* Allocate and setup a new ds_JointSim */
struct ds_JointSim *    ds_CGraphJointAdd(struct ds_RigidBodyPipeline *pipeline, struct ds_Joint *joint);
/* Deallocate a ds_JointSim */
void                    ds_CGraphJointRemove(struct ds_RigidBodyPipeline *pipeline, struct ds_Joint *joint);
/* TODO: for now, we only setup link contact <-> graph */
void                    ds_CGraphContactAdd(struct arena *frame, struct ds_RigidBodyPipeline *pipeline, struct ds_Contact *contact);
/* TODO: for now, we only remove link contact <-> graph */
void                    ds_CGraphContactRemove(struct ds_RigidBodyPipeline *pipeline, struct ds_Contact *contact);


/*
ds_BroadJobPhase
=================
*/

enum ds_BroadJobType
{
    BROAD_JOB_SEED,
    BROAD_JOB_COUNT
};

struct ds_BroadJob
{
    u32 tmp;
};

struct ds_BroadJobPhase
{
    struct ds_JobPhase              phase;

    struct ds_RigidBodyPipeline *   pipeline;

    struct ds_BroadJob *            job;
    u32                             job_count;

    struct ds_ParallelForChain      pf;
};

u32 ds_BroadJobPhaseDispatch(const ds_JobId job);

/*
ds_NarrowJobPhase
=================
*/

enum ds_NarrowJobType
{
    NARROW_JOB_SEED,
    NARROW_JOB_COUNT
};

struct ds_NarrowJob
{
    u32 tmp;
};

struct ds_NarrowJobPhase
{
    struct ds_JobPhase              phase;

    struct ds_RigidBodyPipeline *   pipeline;
    struct dbvhOverlap *            overlap;

    struct ds_NarrowJob *           job;
    u32                             job_count;

    struct ds_ParallelForChain      pf[CG_COLOR_COUNT + 1];
};

u32 ds_NarrowJobPhaseDispatch(const ds_JobId job);


/*
ds_SolverJobPhase
=================
*/

enum ds_SolverJobType
{
    SOLVER_JOB_SEED,
    SOLVER_JOB_COUNT
};

struct ds_SolverJob 
{
    u32 tmp;
};

struct ds_ProxyDirty
{
    struct aabb bbox;
    u32         shape;
    u32         reinsert; 
};

struct ds_ProxyQuery
{
    u32     dynamic_count;
    u32     static_count;
    u32 *   dynamic_query;
    u32 *   static_query;
};
DEFINE_CPOOL_STRUCT(ds_ProxyQuery);

struct ds_ProxyRange
{
    u32                     count;
    u32                     reinsert_count;
    struct ds_ProxyDirty *  proxy;
};

struct ds_SolverJobPhase
{
    struct ds_JobPhase              phase;

	struct ds_RigidBodyPipeline *   pipeline;

    struct ds_SolverJob *           job;
    u32                             job_count;

    struct ds_ParallelForChain      pf_body_update;
    struct ds_ParallelForChain      pf_contact_init;
    struct ds_ParallelForChain      pf_contact_warmup;
    struct ds_ParallelForChain      pf_velocity_solve;
    struct ds_ParallelForChain      pf_integrate;
    struct ds_ParallelForChain      pf_cache_impulse_and_position_init;
    struct ds_ParallelForChain      pf_position_solve;

    struct ds_ParallelForChain      pf_orientation;
    struct ds_ProxyRange *          proxy_range;
};

u32 ds_SolverJobPhaseDispatch(const ds_JobId job);

/*
=================================================================================================================
|						Physics Pipeline			  	      	    	|
=================================================================================================================
*/

#define COLLISION_MARGIN_DEFAULT 5.0f * DS_UNIT_MM 

#define UNIFORM_SIZE 256
#define GRAVITY_CONSTANT_DEFAULT 9.80665f

#define PHYSICS_EVENT_ISLAND(pipeline, event_type, island_index)					        \
	{												                                        \
		struct ds_PhysicsEvent *__physics_debug_event = ds_PhysicsEventPush(pipeline);	\
		__physics_debug_event->type = event_type;						                    \
		__physics_debug_event->island = island_index;						                \
	}

//#ifdef DS_PHYSICS_DEBUG

#define	PhysicsEventBodyNew(pipeline, _body)		                                        \
	{												                                        \
		struct ds_PhysicsEvent *__physics_debug_event = ds_PhysicsEventPush(pipeline);	\
		__physics_debug_event->type = PHYSICS_EVENT_BODY_NEW;						        \
		__physics_debug_event->body = _body;						                        \
	}
#define	PhysicsEventBodyRemoved(pipeline, body_entity)                                      \
	{												                                        \
		struct ds_PhysicsEvent *__physics_debug_event = ds_PhysicsEventPush(pipeline);	\
		__physics_debug_event->type = PHYSICS_EVENT_BODY_REMOVED;						    \
		__physics_debug_event->entity = body_entity;						                \
	}
#define	PhysicsEventIslandAsleep(pipeline, island)	    PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_ASLEEP, island)
#define	PhysicsEventIslandAwake(pipeline, island)	    PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_AWAKE, island)
#define	PhysicsEventIslandNew(pipeline, island)		    PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_NEW, island)
#define PhysicsEventIslandExpanded(pipeline, island)    PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_EXPANDED, island)
#define PhysicsEventIslandRemoved(pipeline, island)         PHYSICS_EVENT_ISLAND(pipeline, PHYSICS_EVENT_ISLAND_REMOVED, island)
#define PhysicsEventContactNew(pipeline, _contact)					                        \
	{												                                        \
		struct ds_PhysicsEvent *__physics_debug_event = ds_PhysicsEventPush(pipeline);	\
		__physics_debug_event->type = PHYSICS_EVENT_CONTACT_NEW;				            \
		__physics_debug_event->contact = _contact;				                            \
	}
#define PhysicsEventContactRemoved(pipeline, body0, shape0, body1, shape1)                  \
	{												                                        \
		struct ds_PhysicsEvent *__physics_debug_event = ds_PhysicsEventPush(pipeline);	\
		__physics_debug_event->type = PHYSICS_EVENT_CONTACT_REMOVED;				        \
		__physics_debug_event->contact_removed_bodies[0] = body0;				            \
		__physics_debug_event->contact_removed_bodies[1] = body1;				            \
		__physics_debug_event->contact_removed_shapes[0] = shape0;				            \
		__physics_debug_event->contact_removed_shapes[1] = shape1;				            \
	}

//#else
//
//#define	PhysicsEventBodyNew(pipeline, body)
//#define	PhysicsEventBodyRemoved(pipeline, entity)
//#define	PhysicsEventIslandAsleep(pipeline, island)
//#define	PhysicsEventIslandAwake(pipeline, island) 
//#define	PhysicsEventIslandNew(pipeline, island)   
//#define	PhysicsEventIslandRemoved(pipeline, island)
//#define   PhysicsEventIslandExpanded(pipeline, island) 
//#define PhysicsEventContactNew(pipeline, contact)
//#define PhysicsEventContactRemoved(pipeline, body0, shape0, body1, shape1)                
//
//#endif

enum ds_PhysicsEventType
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
	PHYSICS_EVENT_COUNT
};

struct ds_PhysicsEvent
{
	POOL_NODE;
    struct ds_DLLNode   node;

	u64			ns;	/* time of event */
	enum ds_PhysicsEventType type;
	union
	{
        u32                     entity;
		ds_IslandId             island;
		ds_RigidBodyId          body;
        ds_ContactId            contact;
        
        struct 
        {
            ds_RigidBodyId      contact_removed_bodies[2];
            ds_ShapeId          contact_removed_shapes[2];
        };
	};
};
POOL_DECLARE(ds_PhysicsEvent);

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
	struct arena 	            frame;			        /* frame memory */

	u64				            ns_start;		        /* external ns at start of physics pipeline */
	u64				            ns_elapsed;		        /* actual ns elasped in pipeline (= 0 at start) */
	u64				            ns_tick;		        /* ns per game tick */
	u64 			            frames_completed;	    /* number of completed physics frames */ 

    f32                         timestep;

	c_ShapeSDB *	            cshape_db;		        /* externally owned */
	ds_RigidBodyPrefabSDB *	    body_prefab_db;		    /* externally owned */

	struct ds_RigidBodyPool     body_pool;
    struct ds_BitSet            body_usage_set;         /* Bodies in use */

	struct ds_ShapePool	        shape_pool;
	struct bvh 		            dynamic_bvh;            /* bvh of dynamic shapes */
	struct bvh 		            static_bvh;             /* bvh of static shapes */

    struct ds_BitSet            dirty_shape_set;    
    ds_CPool(ds_ProxyQuery)     dirty_shape_query;

    struct ds_JointPool         joint_pool;

	struct ds_PhysicsEventPool  event_pool;
	struct ds_DLL		        event_list;

    struct ds_CGraph            cgraph;

    struct ds_SolverSetPool     solver_set_pool;    /* index 0,1,2 reserved for DISABLED,STATIC,ACTIVE sets */

    /* contact net list nodes are owned as follows:
	 *
	 *  contact->key.shape0 owns slot 0
	 *  contact->key.shape1 owns slot 1
	 *
	 * i.e. the smaller index owns slot 0 and the larger index owns slot 1.  */
    struct ds_ContactPool       contact_pool;   
	struct ds_HashMap	        contact_map;		

    ds_IslandId                 island_to_split;            /* */
	struct ds_IslandPool        island_pool;	    
    struct ds_BitSet            island_high_energy_set;   /* High energy islands per-frame. */

	struct collisionDebug *	    debug;
	u32			                debug_count;

	//TODO temporary, move somewhere else.
	vec3 			            gravity;	/* gravity constant */

	u32			                margin_on;
	f32			                margin;

    struct ds_BroadJobPhase *   broad_phase;
    struct ds_NarrowJobPhase *  narrow_phase;
    struct ds_SolverJobPhase *  solver_phase;

    struct ds_NumericsConfig    numerics_config;
};

/**************** PHYISCS PIPELINE API ****************/

/* Initialize a new growable physics pipeline; ns_tick is the duration of a physics frame. */
struct ds_RigidBodyPipeline	PhysicsPipelineAlloc(struct arena *mem, const u32 initial_size, const u64 ns_tick, const u64 frame_memory, c_ShapeSDB *cshape_db, ds_RigidBodyPrefabSDB *prefab_db);
/* free pipeline resources */
void 			PhysicsPipelineFree(struct ds_RigidBodyPipeline *physics_pipeline);
/* flush pipeline resources */
void			PhysicsPipelineFlush(struct ds_RigidBodyPipeline *physics_pipeline);
/* pipeline main method: simulate a single physics frame and update internal state  */
void 			PhysicsPipelineTick(struct ds_RigidBodyPipeline *pipeline);
/* Hash bodies in order from low to high and return the final hash  */
u64             PhysicsPipelineOrientationHash(const struct ds_RigidBodyPipeline *pipeline);
/* validate and ds_Assert internal state of physics pipeline */
void			PhysicsPipelineValidate(const struct ds_RigidBodyPipeline *pipeline);
/* If hit, return parameter (shape,t) of ray at first collision. Otherwise return (U32_MAX, F32_INFINITY) */
u32f32 			PhysicsPipelineRaycastParameter(const struct ds_RigidBodyPipeline *pipeline, const struct ray *ray);
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
struct ds_PhysicsEvent *	ds_PhysicsEventPush(struct ds_RigidBodyPipeline *pipeline);

#ifdef __cplusplus
} 
#endif

#endif
