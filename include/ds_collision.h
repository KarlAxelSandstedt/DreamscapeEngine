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

#define COLLISION_DEFAULT_MARGIN	(100.0f * F32_EPSILON)

/********************************** BVH **********************************/

/*
A struct BVH is a binary tree of boxes; a leaf holds a shape index and a body index (struct BVHNode).
    DBVH*                   dynamic: leaves are inserted and removed one at a time
    TriMeshBVHConstruct     static, over a triangle mesh: built once; the leaves index mesh_bvh.tri
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

struct BVHQuerySet
{
    u32     count;
    u32 *   shape;
};

/* free allocated resources */
void 		        BVHFree(struct BVH *tree);
/* Return the required size when serializing the bvh. */
u64                 BVHSerializeSize(const struct BVH *bvh);
/* Serialize the bvh. WARNING: Assumes bvh fits in the stream. */
void                BVHSerialize(struct ss *ss, const struct BVH *bvh);
/* Returns 1 on success and 0 on failure. Part of the bvh is always heap allocated: free it with BVHFree. */
u32                 BVHTryDeserialize(struct arena *mem, struct ss *ss, struct BVH *bvh, const u32 growable);
/* Derive all internal bounding boxes from the leaves' bounding boxes */
void                BVHPropagateBoundingBoxesFromLeaves(struct BVH *bvh);
/* validate (ds_Assert) internal coherence of bvh */
void 		        BVHValidate(const struct BVH *bvh);
/* return the tree's cost, a quality measure (lower is better) */
f32 		        BVHCost(const struct BVH *bvh);
/* the shapes whose leaf boxes overlap node's box */
struct BVHQuerySet BVHQuery(struct arena *mem, const struct BVH *bvh, const struct BVHNode *node);
/* BVHQuery, only shapes with a body index > node's (bt_right) */
struct BVHQuerySet BVHQueryAndFilterOnBody(struct arena *mem, const struct BVH *bvh, const struct BVHNode *node);

/* an empty dynamic bvh */
struct BVH		    DBVHAlloc(struct arena *mem, const u32 initial_length, const u32 growable);
/* flush / reset the hierarchy  */
void 			    DBVHFlush(struct BVH *bvh);
/* insert a leaf for (body, shape) with box bbox; return its node index (the handle for DBVHRemove) */
u32 			    DBVHInsert(struct BVH *bvh, const u32 body, const u32 shape, const struct aabb *bbox);
/* remove the leaf at node index index (DBVHInsert's return) */
void 			    DBVHRemove(struct BVH *bvh, const u32 index);

struct triMeshBVH
{
	const struct triMesh *	mesh;		
	struct BVH		        bvh;
	u32 *			        tri;		/* tri[tri_count]: the mesh's triangle indices in leaf order; a leaf's shape index indexes it */
	u32			            tri_count;	/* = mesh->tri_count */
    u32                     depth;      /* the deepest leaf's depth, root = 0 */
};

/* Return the mesh's bvh (bin_count: split candidates per axis, build quality); empty for an empty mesh. */
struct triMeshBVH   TriMeshBVHConstruct(struct arena *mem, const struct triMesh *mesh, const u32 bin_count);

/********************************** SHAPES **********************************/

/*
A c_Shape is a sphere, capsule, convex hull (struct dcel) or triangle mesh with its BVH, stored in a ds_SDB(c_Shape)
under a string id. Mass properties are local, at density 1; meshes have none (static bodies only):

    const struct slot slot = c_ShapeSDBAddAndAlias(&db, id);
    struct c_Shape *shape = slot.address;
    shape->id = Utf8CopyBuffered(shape->id_buf, C_SHAPE_ID_SIZE, id);
    shape->type = C_SHAPE_CONVEX_HULL;
    shape->hull = DcelBox(mem, hw);
    c_ShapeUpdateMassProperties(shape);
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
    u8                      id_buf[C_SHAPE_ID_SIZE];
    SDB_NODE;
    struct DLLNode          led_shape_node;     /* the editor's list of shapes (led.cs_shape_list) */

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
		struct triMeshBVH 	mesh_bvh;
	};
};
DEFINE_SDB(c_Shape);

/* volume, center of mass and inertia tensor (density 1) from the geometry; not for meshes (asserted) */
void	c_ShapeUpdateMassProperties(struct c_Shape *shape);

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
u32     c_TriMeshBVHSphereTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32     c_TriMeshBVHCapsuleTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
u32     c_TriMeshBVHHullTest(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);

/********************************** DISTANCES **********************************/

/*
GJK
---
Distance, closest points and normal of two convex shapes' cores: a sphere is its center, a capsule its segment
(callers add the radii). Shapes closer than GJK_TOUCH_TOLERANCE * eps * L (L ~ the shapes' extent) count as
touching. Reads the pushed numerics config (gjk_max_iterations).
    GJK         s1 given in s0's frame (p_0 = t_1_to_0.rotation p_1 + t_1_to_0.position); outputs in s0's frame
    GJKWorld    world transforms; c0, c1 world (f64), n world

    const f32 d = GJK(&c0, &c1, &n, &cache, cache_in, s0, s1, &t_1_to_0, cutoff_distance);

    d == 0.0f           overlap or touch; c0 == c1, n = 0
    d == F32_INFINITY   farther than cutoff_distance; c0, c1, n garbage
    otherwise           the distance, the closest points and the unit normal from s0 to s1 (n accurate to
                        ~6 eps * L / d radians, rarely worse for shapes of very different sizes); only an upper
                        bound when the loop stops early: the iteration limit, or a degenerate or non-improving
                        simplex (seen for shapes of very different sizes)
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
    u16 index_a[4];
    u16 index_b[4];
};

/*
 * touch band width in units of the rounding noise eps * L. Hull faces must be flat well within it: planes (SAT,
 * clipping) and vertices (GJK) describe a face only to its flatness. DcelConvexHull's tolerance is a quarter of the
 * band, its faces flat to ~37 eps L, below the contact tests' accuracy (64 eps L), below the band (100 eps L).
 */
#define GJK_TOUCH_TOLERANCE 100.0f

/* s1 given in s0's frame; outputs in s0's frame */
f32     GJK(v3 *c0, v3 *c1, v3 *n, struct GJKCache *cache_out, const struct GJKCache *cache_in, const struct c_Shape *s0, const struct c_Shape *s1, const ds_Transform *t_1_to_0, const f32 cutoff_distance);
/* world transforms; c0, c1 world (f64), n world */
f32     GJKWorld(v3f64 *c0, v3f64 *c1, v3 *n, struct GJKCache *cache_out, const struct GJKCache *cache_in, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1, const f32 cutoff_distance);

/* distance between the surfaces (>= 0); c0, c1 the closest points (world, f64), set only for a distance > 0 */
f32     c_SphereDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
f32     c_CapsuleSphereDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
f32     c_CapsuleDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
f32     c_HullSphereDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
f32     c_HullCapsuleDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
f32     c_HullDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
/* not implemented (debug: assert; release: 0) */
f32     c_TriMeshBVHSphereDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
f32     c_TriMeshBVHCapsuleDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
f32     c_TriMeshBVHHullDistance(v3f64 *c0, v3f64 *c1, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);

/********************************** CONTACTS **********************************/

/*
A c_Manifold describes how two shapes touch, for the solver: up to 4 points v[i] on the reference shape's
surface, their depths, and the normal n pointing away from the reference; the other shape's deepest point is
v[i] - depth[i] n. The c_*Contact functions return local manifolds, in the reference shape's frame:
    convex pairs    one manifold; s0 is the reference (n from s0 to s1)
    c_MeshContact   one manifold per contacting triangle; the convex shape is the reference (n from it to the
                    mesh)
ds_ShapeContact converts each in place to the solver form (ds_SolverManifold, ds_dynamics.h), hence the union
in c_ContactResult. cached_result: the pair's previous result, or an empty result (zeroed) for none; never NULL.
The new result's arrays are pushed onto frame.
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
/* Debug check (DS_DEBUG callers): v_count <= 4; n, v, depth finite; n != 0; depth >= 0. Return 1 if valid. */
u32     c_ManifoldCheck(const struct c_Manifold *cm);
/* Transform the manifold */
void	c_ManifoldTransform(struct c_Manifold *dst, const struct c_Manifold *src, const m3 rot, const v3 translation);

struct ds_SolverManifold;

struct c_ContactResult
{
    u32                         manifold_count;     /* Number of stored manifolds               */
    u32                         cache_count;        /* Number of stored caches                  */
    union
    {
        struct c_Manifold *         contact_manifold;   /* c_*Contact output: local manifolds (if any) */
        struct ds_SolverManifold *  manifold;           /* the same memory after ds_ShapeContact converted it
                                                           in place: solver form (ds_dynamics.h)         */
    };
    struct c_SatCache *         cache;              /* Caches (if any)                          */
    struct GJKCache *           gjk_cache;          /* GJK cache, at most one (NULL if none)    */
    u32 *                       tri;                /* Sorted triangles, low-to-high (if any)   */
    u32 *                       tri_manifold;       /* Sorted triangle manifold indices (if any)
                                                       m = tri_manifold[T] => manifold if tri[T]
                                                       is manifold[m].                          */
};

/* convex pairs: one local manifold in s0's frame (n from s0 to s1) */
struct c_ContactResult  c_SphereContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
struct c_ContactResult  c_CapsuleSphereContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
struct c_ContactResult  c_CapsuleContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
struct c_ContactResult  c_HullSphereContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
struct c_ContactResult  c_HullCapsuleContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
struct c_ContactResult  c_HullContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);
/*
 * mesh and convex shape: one local manifold per contacting triangle, in the convex shape's frame (n from it to the
 * mesh). The query records every candidate triangle on one of the thread's scratch arenas: more than it holds (about
 * 8,700 at the default 1 MiB) is fatal.
 */
struct c_ContactResult  c_MeshContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *mesh, const ds_TransformWorld *t_mesh, const struct c_Shape *convex, const ds_TransformWorld *t_convex);

/********************************** RAYCASTS **********************************/

/*
XRaycastParameter: the smallest t >= 0 at which origin + t*dir is on the shape's surface, or F32_INFINITY.
Two-sided: an origin inside a solid returns its exit. Three forms with the same t:
    XRaycastParameter           the ray in the shape's local frame
    XRaycastWorldParameter      a world ray; the shape placed by a ds_TransformWorld
    c_XRaycastParameter         the world form on a c_Shape
Precondition: |ray->dir|^2 > 0 (asserted).

Raycasting your own primitives in a BVH's leaves (BVHRaycastInit takes all of mem: pass a scratch arena):

    struct arena *mem = ArenaPushScratch();
    struct BVHRaycastInfo info = BVHRaycastInit(mem, bvh, ray);
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
            BVHRaycastTestAndPushChildren(&info, tuple);
        }
    }
    ArenaPopScratch();
*/

/* the ray prepared for repeated box tests */
struct rayAabb
{
	v3	    origin;
	v3	    inv_dir;	/* 1 / dir, +-inf on an axis where dir is 0 */
	v3u32	sign;		/* sign bit of dir: 1 -> the near slab plane is the box's max */
};

/* the ray's box-test form: inv_dir and the slab signs from dir (a zero component gives +-inf, see the struct) */
struct rayAabb  RayAabb(const struct ray *ray);
/*
 * Return the interval [low, high] of t for which origin + t*dir is in the box (the whole line, t < 0 included);
 * empty if high < low. Conservative: rounding never shortens it.
 */
intv            AabbRaycastInterval(const struct rayAabb *ray, const struct aabb *aabb);

struct BVHRaycastInfo
{
	u32f32			hit;
	struct rayAabb	ray;
	struct MQ	hit_queue;
	const struct BVH *	bvh;
	const struct BVHNode *	node;
};

/* Return the traversal state with the root queued if the ray hits its box; uses all of mem for the queue */
struct BVHRaycastInfo	BVHRaycastInit(struct arena *mem, const struct BVH *bvh, const struct ray *ray);
/* Queue the popped node's children whose boxes the ray hits, keyed by their entry t */
void 			        BVHRaycastTestAndPushChildren(struct BVHRaycastInfo *info, const u32f32 popped_tuple);

/* the ray prepared for many triangle tests (TriRaycastParameterEx); hits never fall through shared edges */
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
f32 SphereRaycastParameter(const struct sphere *sphere, const struct ray *ray);
f32 CapsuleRaycastParameter(const struct capsule *capsule, const struct ray *ray);
/* hull: a closed convex polyhedron with planar faces */
f32 HullRaycastParameter(const struct dcel *hull, const struct ray *ray);
f32 AabbRaycastParameter(const struct aabb *aabb, const struct ray *ray);
f32 TriMeshBVHRaycastParameter(const struct triMeshBVH *mesh_bvh, const struct ray *ray);
/* triangle abc, any winding (no culling) */
f32 TriRaycastParameter(const v3 a, const v3 b, const v3 c, const struct ray *ray);
/* TriRaycastParameter with the ray set up once (RayWoop) for many triangles, e.g. a mesh */
f32 TriRaycastParameterEx(const struct rayWoop *ray, const v3 a, const v3 b, const v3 c);

/* the local functions above, the ray in world space and the shape placed by transform */
f32 SphereRaycastWorldParameter(const struct sphere *sphere, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 CapsuleRaycastWorldParameter(const struct capsule *capsule, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 HullRaycastWorldParameter(const struct dcel *hull, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 AabbRaycastWorldParameter(const struct aabb *aabb, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 TriMeshBVHRaycastWorldParameter(const struct triMeshBVH *mesh_bvh, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 TriRaycastWorldParameter(const v3 a, const v3 b, const v3 c, const ds_TransformWorld *transform, const struct rayWorld *ray);

/* the world functions above on a c_Shape of the matching type */
f32 c_SphereRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 c_CapsuleRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 c_HullRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray);
f32 c_TriMeshBVHRaycastParameter(const struct c_Shape *shape, const ds_TransformWorld *transform, const struct rayWorld *ray);

#ifdef __cplusplus
} 
#endif

#endif
