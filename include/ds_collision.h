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

/*
ds_collision.h
==============
Collision shapes, the queries between two of them (intersection test, distance, contact manifold), raycasts,
and bounding volume hierarchies (BVHs).

    Placement   a shape's geometry is local; a ds_TransformWorld (f64 position) places it. Precision does not
                depend on the distance from the world origin.
    Pairs       c_<S0><S1><Query>(..., s0, t0, s1, t1): s0 has the higher type (sphere < capsule < hull <
                mesh) and is named first (c_CapsuleSphereTest: capsule = s0); normals point from s0 to s1.

Sections: BVH, shapes, tests, distances + (GJK), contacts, raycasts. GJK and the contacts read the pushed
numerics config. Sources: src/math/collision.
*/

#ifndef __DS_COLLISION_H__
#define __DS_COLLISION_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_base.h"
#include "ds_math.h"
#include "ds_string_database.h"
#include "ds_list.h"
#include "ds_queue.h"
#include "ds_binary_tree.h"
#include "ds_bitset.h"

DEFINE_ARRAY_STRUCT(plane);

/********************************** BVH **********************************/

/*
A struct BVH is a binary tree of boxes; a leaf holds a shape index and a body index (struct BVHNode).
    DBVH*                   dynamic: leaves are inserted and removed one at a time
    MeshConstruct           static, over a triangle mesh: built once; the leaves index mesh.leaf_tri
*/

struct BVHNode
{
    /* If leaf, bt_right == body_index, bt_left == shape index */
	BT_NODE;
    POOL_NODE;
	struct aabb bbox;
};
DEFINE_POOL(BVHNode);

struct BVH
{
    struct BT        bt;  
    struct ds_BitSet    leaf_set;       /* dynamic specific */
    struct ds_BitSet    internal_set;   /* dynamic specific */
    ds_Pool(BVHNode)  pool;
	struct MQA	    cost_queue;	    /* dynamic specific */
	u32			        heap_allocated;
};

/* free allocated resources */
void 		        BVHFree(struct BVH *tree);
/* Return the required size when serializing the bvh. */
u64                 BVHSerializeSize(const struct BVH *bvh);
/* Serialize the bvh (aligned). WARNING: Assumes bvh fits in the stream. */
void                BVHSerialize(struct SS *ss, const struct BVH *bvh);
/* Returns 1 on success and 0 on failure. Part of the bvh is always heap allocated: free it with BVHFree. */
u32                 BVHTryDeserialize(struct arena *mem, struct SS *ss, struct BVH *bvh, const u32 growable);
/* Derive all internal bounding boxes from the leaves' bounding boxes */
void                BVHPropagateBoundingBoxesFromLeaves(struct BVH *bvh);
/* validate (ds_Assert) internal coherence of bvh */
void 		        BVHValidate(const struct BVH *bvh);
/* return the tree's cost, a quality measure (lower is better) */
f32 		        BVHCost(const struct BVH *bvh);
/* the shapes whose leaf boxes overlap bbox */
ds_Array(u32)       BVHQuery(struct arena *mem, const struct BVH *bvh, const struct aabb *bbox);
/*
 * BVHQuery for the broadphase, each pair reported once by one of its shapes and never two shapes of one body: a
 * shape with a body index > node's (bt_child[1]), or a shape of another body whose bit in dirty is clear. node's
 * shape (bt_child[0]) must be dirty: only dirty shapes query, so a pair of two dirty shapes comes from the
 * smaller body index and a pair with one dirty shape from that shape.
 */
ds_Array(u32)       BVHQueryAndFilterOnBody(struct arena *mem, const struct BVH *bvh, const struct BVHNode *node, const struct ds_BitSet *dirty);

/* an empty dynamic bvh */
struct BVH		    DBVHAlloc(struct arena *mem, const u32 initial_length, const u32 growable);
/* flush / reset the hierarchy  */
void 			    DBVHFlush(struct BVH *bvh);
/* insert a leaf for (body, shape) with box bbox; return its node index (the handle for DBVHRemove) */
u32 			    DBVHInsert(struct BVH *bvh, const u32 body, const u32 shape, const struct aabb *bbox);
/* remove the leaf at node index index (DBVHInsert's return) */
void 			    DBVHRemove(struct BVH *bvh, const u32 index);

/*
 * triangle mesh: tri_count triangles, each three indices into the v_count vertices v; no adjacency. Precondition:
 * every triangle has nonzero area (asserted by the mesh contacts). A sphere or capsule gets no contact while its
 * center is behind a triangle's plane (its front: CCW about the normal), a hull while its center of mass is more than
 * linear_slop behind it. Raycasts are two-sided.
 */
struct mesh
{
	v3 *			        v;
	v3u32 *			        tri;
	u32			            v_count;
	u32			            tri_count;
	struct BVH		        bvh;
	u32 *			        leaf_tri;	/* leaf_tri[tri_count]: the mesh's triangle indices in leaf order; a leaf's shape index indexes it */
    u32                     depth;      /* the deepest leaf's depth, root = 0 */
};

/*
 * Build the bvh (bvh, leaf_tri, depth) of the mesh's v, tri, v_count and tri_count, which the caller sets; the
 * arrays stay the caller's (bin_count: split candidates per axis, build quality). No triangles or out of memory:
 * an empty bvh, leaf_tri NULL.
 */
void                MeshConstruct(struct arena *mem, struct mesh *mesh, const u32 bin_count);
/* Return the bounding box of all v_count vertices, unreferenced ones included; no vertices: the zero box */
struct aabb         MeshBbox(const struct mesh *mesh);
/* 
 * Return the triangles (allocated on arena) whose bounding boxes overlap bbox (mesh frame),
 * in leaf order.
 */
ds_Array(u32)       MeshQuery(struct arena *mem, const struct mesh *mesh, const struct aabb *bbox);

/********************************** SHAPES **********************************/

/*
A c_Shape is a sphere, capsule, convex hull (struct dcel) or triangle mesh with its BVH, stored in a c_ShapeDB
under a string id. Mass properties are local, at density 1; meshes have none (static bodies only):

    const struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(mem, hw) };
    const struct slot slot = c_ShapeDBAdd(&db, id, &box);
*/

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
    SDB_NODE;
    void *                  mem;                /* one ds_Alloc block: the id and the geometry (c_ShapeDBAdd) */
    void *                  render_mesh;        /* the renderer's mesh, one ds_Alloc block; NULL until it builds one */
    void *                  render_bvh;         /* the renderer's lines of a mesh's BVH, likewise */
    struct DLLNode          led_shape_node;     /* the editor's list of shapes (led.cs_shape_list) */

	m3	                    inertia_tensor;		/* local shape frame intertia tensor (Assumes density=1.0, 
			                		                to get the interia tensor given a density, just multiply
			                		                the matrix with the given density. */
	v3	                    center_of_mass;		/* local shape frame center of mass */
	f32	                    volume;
	f32	                    center_of_mass_surface_distance;
	f32	                    extent;

	enum c_ShapeType        type;
	union
	{
		struct sphere 		sphere;
		struct capsule 		capsule;
		struct dcel		    hull;
		struct mesh 		mesh;
	};
};
DEFINE_SDB(c_Shape);

/*
 * volume, center of mass, inertia tensor (density 1) and center_of_mass_surface_distance from the geometry; not for
 * meshes (asserted)
 */
void	c_ShapeUpdateMassProperties(struct c_Shape *shape);
/*
 * the shape's box at transform (shape frame -> parent frame); a hull's is its local box (dcel bbox) rotated, a mesh's
 * its rotated BVH root box (both not tight)
 */
struct aabb c_ShapeBbox(const struct c_Shape *shape, const ds_Transform *transform);

/*
c_ShapeDB
---------
c_Shapes under string ids, shared by pipelines; constant during a tick. SDB_STUB is the unit box: what a missing
id resolves to. Shapes and prefabs hold the reference counts; a referenced shape is not removed. A slot owns its
id, geometry and render mesh: all freed with it.
*/
struct c_ShapeDB
{
    ds_SDB(c_Shape) sdb;
};

/* Allocate and setup c_ShapeDB; mem and growable as ds_SDBAlloc; fatal on failure */
void        c_ShapeDBAlloc(struct arena *mem, struct c_ShapeDB *db, const u32 length, const u32 growable);
/* Free all memory allocated by the database */
void        c_ShapeDBDealloc(struct c_ShapeDB *db);
/*
 * Add a copy of src (id <= C_SHAPE_ID_SIZE bytes; a mesh as MeshConstruct built it) with its mass
 * properties; bad id, a convex shape without volume, a mesh without triangles, full or out of memory: warning,
 * the stub's slot.
 */
struct slot c_ShapeDBAdd(struct c_ShapeDB *db, const utf8 id, const struct c_Shape *src);
/* 1 if removed; 0 if absent or referenced (warning) */
u32         c_ShapeDBRemove(struct c_ShapeDB *db, const utf8 id);
/* WARNING: forceful removal of a shape, even if referenced. */
void        c_ShapeDBForceRemove(struct c_ShapeDB *db, const utf8 id);
/* the stub's slot if absent */
struct slot c_ShapeDBLookup(const struct c_ShapeDB *db, const utf8 id);
/* Reference a shape */
struct slot c_ShapeDBReference(struct c_ShapeDB *db, const utf8 id);
/* Dereference a shape */
void        c_ShapeDBDereference(struct c_ShapeDB *db, const u32 handle);
/* the address moves when the database grows */
static ds_ForceInline struct c_Shape *c_ShapeDBShape(const struct c_ShapeDB *db, const u32 handle);

/*
 * Return the convex hull of v[0 .. v_count) as a dcel pushed on mem (c_hull.c). Its tolerance is
 * DcelConvexHullTolerance(v, v_count), tied to GJK's touch band: points within it of a face's plane count as on it,
 * the faces are flat to ~1.5 times it, and every input point lies within ~2.5 times it of the hull. Input scattered
 * about a plane by more than the tolerance (noisy scans, perturbed boxes) is handled only roughly: faces are usually
 * flat and containing to about twice the scatter, rarely far worse (a face outline with a reflex corner, its plane
 * tilted), and about 1 in 500 such clouds gives the empty dcel. Fewer than 4 points or no tetrahedron (all points
 * within the tolerance of a point, line or plane): the empty dcel, nothing pushed. Works on 3 of the thread's scratch
 * arenas (the caller may hold at most thread_scratch_count - 3), about 370 B of one per point: some 2,800 points fit
 * the default 1 MiB. Too many points, a full conflict graph or a face merge that cannot be made: a warning, the empty
 * dcel. A finished hull with a zero-area face asserts in debug; in release it is discarded with a warning and
 * DcelBoxStub() is returned.
 */
struct dcel	DcelConvexHull(struct arena *mem, const v3 *v, const u32 v_count);
/* Return DcelConvexHull's tolerance for v[0 .. v_count): GJK_TOUCH_TOLERANCE / 4 * eps * max |v| */
f32		DcelConvexHullTolerance(const v3 *v, const u32 v_count);

/********************************** TESTS **********************************/

/* 1 if the shapes overlap or touch (hull pairs: within GJK's touch band), else 0. Mesh pairs: not implemented
 * yet (debug: assert; release: 1). */
u32     c_SphereTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32     c_CapsuleSphereTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32     c_CapsuleTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32     c_HullSphereTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32     c_HullCapsuleTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32     c_HullTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32     c_MeshSphereTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32     c_MeshCapsuleTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32     c_MeshHullTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);

/********************************** DISTANCES **********************************/

/*
GJK
---
Distance, closest points and normal of two vertex sets: each set's core (the convex hull of its points) grown by
its margin. Cores closer than GJK_TOUCH_TOLERANCE * eps * L (L ~ the shapes' extent) count as touching. Reads the
pushed numerics config (gjk_max_iterations).
    GJK         vset1 given in vset0's frame (p_0 = t_1_to_0.rotation p_1 + t_1_to_0.position); outputs in
                vset0's frame
    GJKWorld    world transforms; c0, c1 world (f64), n world

    const f32 d = GJK(&c0, &c1, &n, &cache, cache_in, vset0, vset1, &t_1_to_0, cutoff_distance);

    d == 0.0f           overlap or touch; c0 == c1 (a point of vset0's core, not a closest point), n = 0
    d == F32_INFINITY   farther than cutoff_distance; c0, c1, n garbage
    otherwise           the distance, the closest points and the unit normal from vset0 to vset1 (n accurate to
                        ~6 eps * L / (d + margin0 + margin1) radians, rarely worse for shapes of very different
                        sizes); only an upper bound when the loop stops early: the iteration limit, or a
                        degenerate or non-improving simplex (seen for shapes of very different sizes)
    cache_in            NULL or the pair's previous cache_out (warm start); cache_out is always written
    cutoff_distance     larger distances are of no interest (F32_INFINITY: none); the early exit needs the
                        distance to clear it by the touch band, and a finite result can still exceed it:
                        callers compare
*/

/*
 * GJK warm start between frames on the same pair: the support indices of the last simplex. A cache that
 * exists is valid: count is 1..4 and the indices belong to the pair's shapes. No cache is NULL, never an
 * empty cache.
 */
struct GJKCache
{
    f32 metric;             /* length/area/volume of the cached simplex; flushed if it changes > 2x or (count > 1) falls
                               below F32_EPSILON, an absolute threshold: tetrahedra below ~5 mm restart cold */
    u16 count;              /* 1..4 */
    u16 index0[4];
    u16 index1[4];
};

/* Shape vertex set/storage */
struct GJKVertexSet
{
    const v3 *  v;
    u32         v_count;
    f32         margin;
    v3          buf[2];     /* storage for sphere/capsule points */
};

/*
SATCache
==========
Internal physics engine struct for caching SAT-based contact calculations each frame. If a contact was found,
the contact results may be re-used the next frame if a set of conditions are fullfilled.

::: Internals :::

sat_CacheTryReuse (c_contact.c): a separating axis is reused while it still separates; a contact is rebuilt from its
cached features and reused while its depth changed by less than sat_cache_tolerance (ds_NumericsConfig).
*/

enum sat_CacheType
{
	SAT_CACHE_SEPARATION,   /* Seperation axis found    */
	SAT_CACHE_CONTACT_FV,   /* Face-Vertex Contact      */
	SAT_CACHE_CONTACT_EE,   /* Edge-Edge Contact        */
	SAT_CACHE_NONE,         /* Nothing to cache (fallback contact); never stored in a ds_ConcaveCache */
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

struct SATCache
{
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
     *
     * type == NONE:
     *  feature[i].type == FACE => i has the reference face
     *  feature[i].type == VERTEX => i has the deepest vertex
     */
    sat_FeatureId   feature[2];
    f32             depth;      
    v3            normal;     
};

struct ds_ConvexCache
{
    struct SATCache sat;
    struct GJKCache gjk;
};

struct ds_TriCache
{
    u32                 tri;
    struct SATCache     sat;
};
DEFINE_CPOOL_STRUCT(ds_TriCache);
DEFINE_ARRAY_STRUCT(ds_TriCache);
DEFINE_TSLAB(ds_TriCache);

struct ds_ConcaveCache
{
    ds_Array(ds_TriCache)   tri;            /* tri.length: capacity, grown by the slab, never shrunk */
    u32                     tri_count;
};

/*
 * touch band width in units of the rounding noise eps * L. Hull faces must be flat well within it: planes (SAT,
 * clipping) and vertices (GJK) describe a face only to its flatness. DcelConvexHull's tolerance is a quarter of the
 * band, its faces flat to ~37 eps L, below the contact tests' accuracy (64 eps L), below the band (100 eps L).
 */
#define GJK_TOUCH_TOLERANCE 100.0f

void    GJKVertexSetPoint(struct GJKVertexSet *vset, const v3 p);
void    GJKVertexSetSegment(struct GJKVertexSet *vset, const v3 p0, const v3 p1);
void    GJKVertexSetSphere(struct GJKVertexSet *vset, const struct sphere *sphere);
void    GJKVertexSetCapsule(struct GJKVertexSet *vset, const struct capsule *capsule);
void    GJKVertexSetHull(struct GJKVertexSet *vset, const struct dcel *hull);
void    GJKVertexSetTriangle(struct GJKVertexSet *vset, const v3 tri[3]);
/* Set up the vset of shape (hull, sphere or capsule) in its local frame. */
void    GJKVertexSetShapeNoMargin(struct GJKVertexSet *vset, const struct c_Shape *shape);

/* vset1 given in vset0's frame; outputs in vset0's frame */
f32     GJK(v3 *c0, v3 *c1, v3 *n, struct GJKCache *cache_out, const struct GJKCache *cache_in, const struct GJKVertexSet *vset0, const struct GJKVertexSet *vset1, const ds_Transform *t_1_to_0, const f32 cutoff_distance);
/* world transforms; c0, c1 world (f64), n world */
f32     GJKWorld(v3f64 *c0, v3f64 *c1, v3 *n, struct GJKCache *cache_out, const struct GJKCache *cache_in, const struct GJKVertexSet *vset0, const ds_TransformWorld *t0, const struct GJKVertexSet *vset1, const ds_TransformWorld *t1, const f32 cutoff_distance);

/* distance between the surfaces (>= 0); c0, c1 the closest points (world, f64), set only for a distance > 0 */
f32     c_SphereDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
f32     c_CapsuleSphereDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
f32     c_CapsuleDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
f32     c_HullSphereDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
f32     c_HullCapsuleDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
f32     c_HullDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
/* not implemented (debug: assert; release: 0) */
f32     c_MeshSphereDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
f32     c_MeshCapsuleDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
f32     c_MeshHullDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);

/********************************** CONTACTS **********************************/

/*
A c_Manifold describes how two shapes touch, for the solver: up to 4 points v[i] on the reference shape's
surface, their depths, and the normal n pointing away from the reference; the other shape's deepest point is
v[i] - depth[i] n. The c_*Contact functions return local manifolds, in the reference shape's frame:
    convex pairs    one manifold; s0 is the reference (n from s0 to s1)
    c_MeshContact   one manifold per contacting triangle; the convex shape is the reference (n from it to the
                    mesh)
ds_ShapeContact converts each to the solver form (ds_SolverManifold, ds_dynamics.h). tri is set by c_MeshContact
only.

Point ids: the solver's warm start matches a manifold's points to the previous tick's by id. Contract: an id is
unique within its manifold, and a point keeps its id across ticks while the same features produce it; a feature
switch (another reference face, a face contact becoming an edge contact) gives a new id and the old impulse is not
inherited. Laid out in 32 bits as the point's two features, each a type (C_POINT_*) and an index:

    31                                              0
    [ Type0(3) | index0(13) | Type1(3) | index1(13) ]

Half i is shape i's feature, except for a clipped face contact, whose point is where two edges meet: half 0 the
edge entering the point, half 1 the edge leaving it, each an EDGE_OF_0 or EDGE_OF_1 (the shape it belongs to).
An incident vertex is its two incident edges, a clipped point an incident edge and a reference edge, a reference
corner its two reference edges: no two points of a clip share both edges.
Points made analytically rather than by features (spheres, capsule ends) take c_PointIdConstant(k), k the rule's
point number: both types ANALYTICAL, so a constant never collides with a feature id.

Internal: TODO Expplain concisely my thoughts on why caches exist and work 
    () apply cached impulses to same normals
    () while normal may stay the same, levers change => different rotation, which may be fine
    () In case of analytical collision functions (sphere-sphere, capsule,capsule), we can
       argue that normals stay mostly continous and may therefore use a single identifier,
       or 2 identifiers in the case of parallel case of capsules
*/
typedef struct c_PointId
{
    u32 val;
} c_PointId;

/* The shape feature type of one of the halves in c_PointId.  */
#define C_POINT_ANALYTICAL      0x0
#define C_POINT_VERTEX          0x1
#define C_POINT_EDGE            0x2
#define C_POINT_FACE            0x3
#define C_POINT_EDGE_OF_0       0x4     /* a clipped face contact: an edge of shape 0, ... */
#define C_POINT_EDGE_OF_1       0x5     /* ... of shape 1 (EDGE_OF_0 + the shape index) */

/* Initialize identifier from its two features */
static ds_ForceInline c_PointId c_PointIdInit(const u32 type0, const u32 index0, const u32 type1, const u32 index1);
/* Initialize identifier with given constant */
static ds_ForceInline c_PointId c_PointIdConstant(const u32 constant);
/* 1 if the identifiers are equal */
static ds_ForceInline u32       c_PointIdEqual(const c_PointId a, const c_PointId b);

struct c_Manifold
{
	v3 	        v[4];       /* contact point on the reference shape surface     */
	f32 	    depth[4];   /* Contact point penetration depth (0.0f, INFINITY) */
	v3 	        n;		    /* Contact normal: Points away from reference       */
	u32 	    v_count;    /* Contact point count                              */
	c_PointId   id[4];      /* contact point geometric identifier               */
	u32         tri;
};
DEFINE_ARRAY_STRUCT(c_Manifold);

/* Print manifold to file stderr */
void 	c_ManifoldDebugPrint(const struct c_Manifold *cm);
/* Debug check (DS_DEBUG callers): v_count <= 4; n, v, depth finite; n != 0; depth >= 0. Return 1 if valid. */
u32     c_ManifoldCheck(const struct c_Manifold *cm);
/* Transform the manifold */
void	c_ManifoldTransform(struct c_Manifold *dst, const struct c_Manifold *src, const m3 rot, const v3 translation);

/* convex pairs: returns 0 or 1, out the local manifold in s0's frame (n from s0 to s1); cache is in-out, hull pairs use it */
u32                     c_SphereContact(struct c_Manifold *out, struct ds_ConvexCache *cache, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32                     c_CapsuleSphereContact(struct c_Manifold *out, struct ds_ConvexCache *cache, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32                     c_CapsuleContact(struct c_Manifold *out, struct ds_ConvexCache *cache, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32                     c_HullSphereContact(struct c_Manifold *out, struct ds_ConvexCache *cache, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32                     c_HullCapsuleContact(struct c_Manifold *out, struct ds_ConvexCache *cache, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32                     c_HullContact(struct c_Manifold *out, struct ds_ConvexCache *cache, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
/*
 * mesh and convex shape: one local manifold per contacting triangle (tri), in the convex shape's frame (n from it to
 * the mesh), pushed onto tmp. The query records every candidate triangle on one of the thread's scratch arenas:
 * more than it holds (about 8,700 at the default 1 MiB) is fatal. cache is in-out and always valid (empty: tri_count 0, with or without an
 * array); on return its first tri_count entries hold the hull's SAT entries by triangle (at most cache_count_max,
 * never SAT_CACHE_NONE), else tri_count is 0. The array comes from slab and grows when the entries do not fit.
 */
ds_Array(c_Manifold)    c_MeshContact(struct arena *tmp, ds_TSlab(ds_TriCache) *slab, struct ds_ConcaveCache *cache, const struct c_Shape *mesh, const ds_TransformWorld *t_mesh, const struct c_Shape *convex, const ds_TransformWorld *t_convex);

/********************************** RAYCASTS **********************************/

/*
XRayCastParameter: the smallest t >= 0 at which origin + t*dir is on the shape's surface, or F32_INFINITY.
Two-sided: an origin inside a solid returns its exit. Three forms with the same t:
    XRayCastParameter           the ray in the shape's local frame
    XRayCastWorldParameter      a world ray; the shape placed by a ds_TransformWorld
    c_XRayCastParameter         the world form on a c_Shape
Precondition: |ray->dir|^2 > 0 (asserted).

Raycasting your own primitives in a BVH's leaves (BVHRayCastInit takes all of mem: pass a scratch arena):

    struct arena *mem = ArenaPushScratch();
    struct BVHCastInfo info = BVHRayCastInit(mem, bvh, ray);
    while (info.hit_queue.count)
    {
        const u32f32 tuple = MQPop(&info.hit_queue);
        if (info.hit.f < tuple.f)
        {
            break;
        }

        if (BTLeafCheck(info.node + tuple.u))
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
            BVHCastTestAndPushChildren(&info, tuple);
        }
    }
    ArenaPopScratch();
*/

/* the ray prepared for repeated box tests */
struct rayAabb
{
	struct aabb	origin;
	v3	    inv_dir;	/* 1 / dir, +-inf on an axis where dir is 0 */
	v3u32	sign;		/* sign bit of dir: 1 -> the near slab plane is the box's max */
};

/* the ray's box-test form: inv_dir and the slab signs from dir (a zero component gives +-inf, see the struct) */
struct rayAabb  RayAabb(const struct ray *ray);
/*
 * Return the interval [low, high] of t for which origin + t*dir is in the box (the whole line, t < 0 included);
 * empty if high < low. Conservative: rounding never shortens it.
 */
intv            AabbRayCastInterval(const struct rayAabb *ray, const struct aabb *aabb);

struct BVHCastInfo
{
	u32f32			hit;
	struct rayAabb	ray;
	struct MQ	hit_queue;
	const struct BVH *	bvh;
	const struct BVHNode *	node;
};

/* Return the traversal state with the root queued if the ray hits its box; uses all of mem for the queue */
struct BVHCastInfo	BVHRayCastInit(struct arena *mem, const struct BVH *bvh, const struct ray *ray);
struct BVHCastInfo	BVHAabbCastInit(struct arena *mem, const struct BVH *bvh, const struct aabb *box, const v3 translation);
/* Queue the popped node's children whose boxes the ray hits, keyed by their entry t */
void 			        BVHCastTestAndPushChildren(struct BVHCastInfo *info, const u32f32 popped_tuple);

/* the ray prepared for many triangle tests (TriRayCastParameterEx); hits never fall through shared edges */
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
/* the ray's triangle-test form: the shear that maps dir onto +z of (kx, ky, kz); dir must be non-zero */
struct rayWoop  RayWoop(const struct ray *ray);

/* the first surface hit t >= 0 or F32_INFINITY; the ray in the shape's local frame */
f32 SphereRayCastParameter(const struct sphere *sphere, const struct ray *ray);
f32 CapsuleRayCastParameter(const struct capsule *capsule, const struct ray *ray);
/* hull: a closed convex polyhedron with planar faces */
f32 HullRayCastParameter(const struct dcel *hull, const struct ray *ray);
f32 AabbRayCastParameter(const struct aabb *aabb, const struct ray *ray);
f32 MeshRayCastParameter(const struct mesh *mesh, const struct ray *ray);
/* triangle abc, any winding (no culling) */
f32 TriRayCastParameter(const v3 a, const v3 b, const v3 c, const struct ray *ray);
/* TriRayCastParameter with the ray set up once (RayWoop) for many triangles, e.g. a mesh */
f32 TriRayCastParameterEx(const struct rayWoop *ray, const v3 a, const v3 b, const v3 c);

/* the local functions above, the ray in world space and the shape placed by transform */
f32 SphereRayCastWorldParameter(const struct sphere *sphere, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 CapsuleRayCastWorldParameter(const struct capsule *capsule, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 HullRayCastWorldParameter(const struct dcel *hull, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 AabbRayCastWorldParameter(const struct aabb *aabb, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 MeshRayCastWorldParameter(const struct mesh *mesh, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 TriRayCastWorldParameter(const v3 a, const v3 b, const v3 c, const ds_TransformWorld *transform, const struct rayWorld *ray);

/* the world functions above on a c_Shape of the matching type */
f32 c_SphereRayCastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 c_CapsuleRayCastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 c_HullRayCastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 c_MeshRayCastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray);

/********************************** Time Of Impact **********************************/

struct ds_TimeOfImpactArg
{
    const struct c_Shape *  shape;
    struct GJKVertexSet     vset;
    f32                     margin;
    q                       rotation_shape_world[2];
    v3                      com_anchor[2];
    v3                      com_shape;
};

/*
 * Setup the shape -> world-anchor-com-diff-space transform
 *
 *      com_shape    = t_local^-1 (com_body)                                    center-of-mass in shape space
 *      rotation_k   = t_body_world_k.rotation t_local.rotation                 ordinary shape-to-world rotation
 *      com_k        = t_body_world_k.position - anchor + t_body_world_k.rotation com_body
 *                   = com_world - world_anchor
 *                   = world_shape.rot*com_shape + world_shape.position  - world_anchor
 */
static inline void ds_TimeOfImpactArgSweep(struct ds_TimeOfImpactArg *arg, const ds_Transform *t_local, const ds_TransformWorld *t_body_world0, const ds_TransformWorld *t_body_world1, const v3 com_body, const v3f64 anchor);

/* TimeOfImpact shape initializers */
static inline void ds_TimeOfImpactArgSphere(struct ds_TimeOfImpactArg *arg, const struct c_Shape *shape, const ds_Transform *t_local, const ds_TransformWorld *t_body_world0, const ds_TransformWorld *t_body_world1, const v3 com_body, const v3f64 anchor);
static inline void ds_TimeOfImpactArgCapsule(struct ds_TimeOfImpactArg *arg, const struct c_Shape *shape, const ds_Transform *t_local, const ds_TransformWorld *t_body_world0, const ds_TransformWorld *t_body_world1, const v3 com_body, const v3f64 anchor);
static inline void ds_TimeOfImpactArgHull(struct ds_TimeOfImpactArg *arg, const struct c_Shape *shape, const ds_Transform *t_local, const ds_TransformWorld *t_body_world0, const ds_TransformWorld *t_body_world1, const v3 com_body, const v3f64 anchor);
static inline void ds_TimeOfImpactArgMesh(struct ds_TimeOfImpactArg *arg, const struct c_Shape *shape, const ds_Transform *t_local, const ds_TransformWorld *t_body_world, const v3f64 anchor);
static inline void ds_TimeOfImpactArgTriangle(struct ds_TimeOfImpactArg *arg, const struct ds_TimeOfImpactArg *mesh, const v3 tri[3]);

/*
 * Return the toi if a time-of-impact was found in [0, max_toi).
 * Return the last t in [0, max_toi) at which the inputs neither overlap nor have passed
 *        through each other, if an iteration limit was reached first.
 * Otherwise return max_toi (no collision).
 *
 * Note: If returned toi is 0.0f, the pair is left to discrete collision detection.
 */
f32     ds_TimeOfImpactConvex(const struct ds_TimeOfImpactArg *arg0, const struct ds_TimeOfImpactArg *arg1, const f32 max_toi);

f32     ds_TimeOfImpactConcave(struct arena *mem, const struct ds_TimeOfImpactArg *mesh, const struct ds_TimeOfImpactArg *convex, const f32 max_toi);



/********************************** SHAPES: definitions **********************************/

static ds_ForceInline struct c_Shape *c_ShapeDBShape(const struct c_ShapeDB *db, const u32 handle)
{
    ds_Assert(handle < db->sdb.pool.count_max);
    return db->sdb.pool.buf + handle;
}

/********************************** CONTACTS: definitions **********************************/

static ds_ForceInline c_PointId c_PointIdInit(const u32 type0, const u32 index0, const u32 type1, const u32 index1)
{
    ds_Assert(type0 <= 0x7 && type1 <= 0x7 && index0 <= 0x1fff && index1 <= 0x1fff);
    return (struct c_PointId) { .val = (((type0 << 13) | index0) << 16) | ((type1 << 13) | index1) };
}

static ds_ForceInline c_PointId c_PointIdConstant(const u32 constant)
{
    return (struct c_PointId) { .val = constant };
}

static ds_ForceInline u32 c_PointIdEqual(const c_PointId a, const c_PointId b)
{
    return a.val == b.val;
}

/********************************** Time Of Impact: definitions **********************************/

static inline void ds_TimeOfImpactArgSweep(struct ds_TimeOfImpactArg *arg, const ds_Transform *t_local, const ds_TransformWorld *t_body_world0, const ds_TransformWorld *t_body_world1, const v3 com_body, const v3f64 anchor)
{
    const v3 position0 = V3F64Demote(V3F64Sub(t_body_world0->position, anchor));
    const v3 position1 = V3F64Demote(V3F64Sub(t_body_world1->position, anchor));
    arg->rotation_shape_world[0] = QMul(t_body_world0->rotation, t_local->rotation);
    arg->rotation_shape_world[1] = QMul(t_body_world1->rotation, t_local->rotation);
    arg->com_anchor[0] = V3Add(position0, QUnitRotate(t_body_world0->rotation, com_body));
    arg->com_anchor[1] = V3Add(position1, QUnitRotate(t_body_world1->rotation, com_body));
    arg->com_shape = V3InverseTransform(com_body, t_local);
}

static inline void ds_TimeOfImpactArgSphere(struct ds_TimeOfImpactArg *arg, const struct c_Shape *shape, const ds_Transform *t_local, const ds_TransformWorld *t_body_world0, const ds_TransformWorld *t_body_world1, const v3 com_body, const v3f64 anchor)
{
    ds_Assert(shape->type == C_SHAPE_SPHERE);
    GJKVertexSetPoint(&arg->vset, V3Zero());
    arg->shape = shape;
    arg->margin = shape->sphere.radius;
    ds_TimeOfImpactArgSweep(arg, t_local, t_body_world0, t_body_world1, com_body, anchor);
}

static inline void ds_TimeOfImpactArgCapsule(struct ds_TimeOfImpactArg *arg, const struct c_Shape *shape, const ds_Transform *t_local, const ds_TransformWorld *t_body_world0, const ds_TransformWorld *t_body_world1, const v3 com_body, const v3f64 anchor)
{
    ds_Assert(shape->type == C_SHAPE_CAPSULE);
    const f32 h = shape->capsule.half_height;
    GJKVertexSetSegment(&arg->vset, V3(0.0f, h, 0.0f), V3(0.0f, -h, 0.0f));
    arg->shape = shape;
    arg->margin = shape->capsule.radius;
    ds_TimeOfImpactArgSweep(arg, t_local, t_body_world0, t_body_world1, com_body, anchor);
}

static inline void ds_TimeOfImpactArgHull(struct ds_TimeOfImpactArg *arg, const struct c_Shape *shape, const ds_Transform *t_local, const ds_TransformWorld *t_body_world0, const ds_TransformWorld *t_body_world1, const v3 com_body, const v3f64 anchor)
{
    ds_Assert(shape->type == C_SHAPE_CONVEX_HULL);
    GJKVertexSetHull(&arg->vset, &shape->hull);
    arg->shape = shape;
    arg->margin = 0.0f;
    ds_TimeOfImpactArgSweep(arg, t_local, t_body_world0, t_body_world1, com_body, anchor);
}

static inline void ds_TimeOfImpactArgMesh(struct ds_TimeOfImpactArg *arg, const struct c_Shape *shape, const ds_Transform *t_local, const ds_TransformWorld *t_body_world, const v3f64 anchor)
{
    ds_Assert(shape->type == C_SHAPE_TRI_MESH);
    arg->vset = (struct GJKVertexSet) { 0 };
    arg->shape = shape;
    arg->margin = 0.0f;
    ds_TimeOfImpactArgSweep(arg, t_local, t_body_world, t_body_world, t_local->position, anchor);
}

static inline void ds_TimeOfImpactArgTriangle(struct ds_TimeOfImpactArg *arg, const struct ds_TimeOfImpactArg *mesh, const v3 tri[3])
{
    ds_Assert(mesh->shape->type == C_SHAPE_TRI_MESH);
    arg->shape = mesh->shape;
    GJKVertexSetTriangle(&arg->vset, tri);
    arg->margin = 0.0f;
    arg->rotation_shape_world[0] = mesh->rotation_shape_world[0];
    arg->rotation_shape_world[1] = mesh->rotation_shape_world[1];
    arg->com_anchor[0] = mesh->com_anchor[0];
    arg->com_anchor[1] = mesh->com_anchor[1];
    arg->com_shape = mesh->com_shape;
}

#ifdef __cplusplus
} 
#endif

#endif
