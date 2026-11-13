/*
==========================================================================
    Copyright (C) 2026 Axel Sandstedt 

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

#include "ds_float.h"
#include "ds_vector.h"
#include "ds_quaternion.h"
#include "ds_matrix.h"


ds_ShapeId ds_ShapeAdd(struct ds_Dynamics *pipeline, const struct ds_ShapePrefab *prefab, const ds_Transform *t, const ds_BodyId body_id)
{
    struct slot	body_slot = ds_BodyLookup(pipeline, body_id);
    struct ds_Body *body = body_slot.address;
    if (!body)
    {
        return ds_ShapeIdNull();
    }

    /* a dynamic body's mass is density times volume, and the solvers divide by it */
    if (ds_BodyDynamicCheck(body) && !(F32FiniteCheck(prefab->density) && prefab->density > 0.0f))
    {
        Log(T_PHYSICS, S_WARNING, "shape add rejected: density %f on dynamic body %u, it must be finite and positive", prefab->density, ds_BodyIdIndex(body_id));
        return ds_ShapeIdNull();
    }

    ds_ShapeId id = ds_ShapeIdNull();
    const u32 old_max = pipeline->shape_pool.count_max;
    const struct slot shape_slot = ds_PoolAdd(pipeline->shape_pool);
	struct ds_Shape *shape = shape_slot.address;

    if (old_max != pipeline->shape_pool.count_max)
    {
        shape->id = ds_ShapeIdInit(shape_slot.index, 0);
        ds_CPoolPush(pipeline->dirty_shape_query);
    }
    shape->id = ds_ShapeIdNextGeneration(shape->id);

    if (pipeline->shape_dirty_set.bit_count <= shape_slot.index)
    {
        ds_BitSetIncreaseSize(&pipeline->shape_dirty_set, pipeline->shape_dirty_set.bit_count << 1, 0);
    }

    if (pipeline->shape_dynamic_usage_set.bit_count <= shape_slot.index)
    {
        ds_BitSetIncreaseSize(&pipeline->shape_dynamic_usage_set, pipeline->shape_dynamic_usage_set.bit_count << 1, 0);
    }

	DLLAppend(body->shape_list, pipeline->shape_pool.buf, shape_slot.index, body_shape);

    shape->flags = body->flags & SHAPE_FLAG_ALL;
	shape->body = ds_BodyIdIndex(body_id);
	shape->density = prefab->density;
	shape->restitution = prefab->restitution;
	shape->friction = prefab->friction;
	shape->t_local = *t;
	DLLFlush(shape->contact_list);

	const struct c_Shape *cshape = c_ShapeDBShape(pipeline->external->cshape_db, prefab->cshape);
	const struct slot cshape_slot = c_ShapeDBReference(pipeline->external->cshape_db, cshape->id);
	shape->cshape_handle = cshape_slot.index;
	shape->cshape_type = cshape->type;

	shape->margin = (ds_BodyDynamicCheck(body))
	              ? F32Max(pipeline->config.numerics.proxy_margin_min, pipeline->config.numerics.proxy_margin_scale * cshape->extent)
	              : pipeline->config.numerics.proxy_margin_min;

	struct aabb bbox_proxy = ds_ShapeWorldBbox(pipeline, shape);
    if (ds_BodyDynamicCheck(body))
    {
		bbox_proxy.min = V3AddConstant(bbox_proxy.min, -shape->margin);
		bbox_proxy.max = V3AddConstant(bbox_proxy.max, shape->margin);
        ds_BitSetSet(&pipeline->shape_dynamic_usage_set, shape_slot.index, 1);
        ds_BitSetSet(&pipeline->shape_dirty_set, shape_slot.index, 1);
        shape->proxy = DBVHInsert(&pipeline->dynamic_bvh, shape->body, shape_slot.index, &bbox_proxy);
    }
    else
    {
        shape->proxy = DBVHInsert(&pipeline->static_bvh, shape->body, shape_slot.index, &bbox_proxy);
    }
    
    ds_BodyUpdateMassProperties(pipeline, body_id);

    return shape->id;
}

void ds_ShapeDynamicRemove(struct ds_Dynamics *pipeline, struct ds_Body *body, const u32 shape_index, const u32 mass_properties_update)
{
    struct ds_Shape *shape = pipeline->shape_pool.buf + shape_index;

    while (shape->contact_list.count)
	{
        ds_ContactRemove(pipeline, shape->contact_list.first, 1);
    }

    if (mass_properties_update)
    {
        ds_BodyUpdateMassProperties(pipeline, body->id);
    }

    ds_BitSetSet(&pipeline->shape_dynamic_usage_set, shape_index, 0);
    ds_BitSetSet(&pipeline->shape_dirty_set, shape_index, 0);
    DLLRemove(body->shape_list, pipeline->shape_pool.buf, shape_index, body_shape);
	c_ShapeDBDereference(pipeline->external->cshape_db, shape->cshape_handle);
	DBVHRemove(&pipeline->dynamic_bvh, shape->proxy);
	ds_PoolRemove(pipeline->shape_pool, shape_index);
}

void ds_ShapeStaticRemove(struct ds_Dynamics *pipeline, struct ds_Body *body, const u32 index)
{
	struct ds_Shape *shape = pipeline->shape_pool.buf + index;

    while (shape->contact_list.count)
	{
        ds_ContactRemove(pipeline, shape->contact_list.first, 0);
    }

    ds_BitSetSet(&pipeline->shape_dirty_set, index, 0);
    DLLRemove(body->shape_list, pipeline->shape_pool.buf, index, body_shape);
	c_ShapeDBDereference(pipeline->external->cshape_db, shape->cshape_handle);
	DBVHRemove(&pipeline->static_bvh, shape->proxy);
	ds_PoolRemove(pipeline->shape_pool, index);
}

struct slot ds_ShapeLookup(const struct ds_Dynamics *pipeline, const ds_ShapeId shape_id)
{
    struct slot slot = { .address = NULL, .index = U32_MAX };
    const u32 index = ds_ShapeIdIndex(shape_id);
    if (index < pipeline->shape_pool.count_max)
    {
        struct ds_Shape *shape = pipeline->shape_pool.buf + index;
        if (ds_PoolSlotAllocated(shape) && ds_ShapeIdEqual(shape->id, shape_id))
        {
            slot.address = shape;
            slot.index = index;
        }
    }

    return slot;
}

void ds_ShapeWorldTransform(ds_TransformWorld *t, const struct ds_Dynamics *pipeline, const struct ds_Shape *shape)
{
	const struct ds_Body *body = pipeline->body_pool.buf + shape->body;
    const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + body->set;
    const struct ds_BodySim *sim = set->body_sim_pool.buf + body->sim;
    *t = ds_TransformWorldChild(&sim->world, &shape->t_local);
}

struct aabb ds_ShapeWorldBbox(const struct ds_Dynamics *pipeline, const struct ds_Shape *shape)
{
	v3 min = V3(F32_INFINITY, F32_INFINITY, F32_INFINITY);
	v3 max = V3(-F32_INFINITY, -F32_INFINITY, -F32_INFINITY);

	const struct c_Shape *cshape = c_ShapeDBShape(pipeline->external->cshape_db, shape->cshape_handle);

    ds_TransformWorld t_world;
    ds_ShapeWorldTransform(&t_world, pipeline, shape);
	const m3 rot = M3QUnit(t_world.rotation);
    /* the box relative to the shape's position (rotation only), then translated in f64 and rounded outward */

	if (shape->cshape_type == C_SHAPE_CONVEX_HULL)
	{
		for (u32 i = 0; i < cshape->hull.v_count; ++i)
		{
			const v3 v = M3V3Mul(rot, cshape->hull.v[i]);
			min = V3Min(min, v);
			max = V3Max(max, v);
		}
	}
	else if (shape->cshape_type == C_SHAPE_SPHERE)
	{
		const f32 r = cshape->sphere.radius;
		min = V3(-r, -r, -r);
		max = V3(r, r, r);
	}
	else if (shape->cshape_type == C_SHAPE_CAPSULE)
	{
		const v3 v = M3V3Mul(rot, V3(0.0f, cshape->capsule.half_height, 0.0f));
		max = V3AddConstant(V3Abs(v), cshape->capsule.radius);
		min = V3Negate(max);
	}
	else if (shape->cshape_type == C_SHAPE_TRI_MESH)
	{
		//TODO "We treat Tri meshes differently; a rigid body who has a tri mesh attached
		// views the tri mesh triangles and its shapes. Thus such a rigid body treats its
		// mesh shape to have position 0 and no rotation.
        ds_Assert(V3Length(shape->t_local.position) == 0.0f);
        ds_Assert(shape->t_local.rotation.w == 1.0f);
		const struct BVHNode *node = cshape->mesh_bvh.bvh.pool.buf;
		const struct aabb bbox = BboxRotate(node[cshape->mesh_bvh.bvh.bt.root].bbox, rot);
		min = bbox.min;
		max = bbox.max;
	}

	const struct aabb local = { .min = min, .max = max };
	struct aabb bbox;
	AabbTranslateOutward(&bbox, &local, t_world.position);
	return bbox;
}

/********************************** LOOKUP TABLES FOR SHAPES **********************************/

static u32 (*c_shape_tests[C_SHAPE_COUNT][C_SHAPE_COUNT])(const struct c_Shape *, const ds_TransformWorld *, const struct c_Shape *, const ds_TransformWorld *) =
{
	{ c_SphereTest, 		    0, 				            0, 			            0, },
	{ c_CapsuleSphereTest,	    c_CapsuleTest, 			    0, 			            0, },
	{ c_HullSphereTest, 		c_HullCapsuleTest,		    c_HullTest,		        0, },
	{ c_TriMeshBVHSphereTest,   c_TriMeshBVHCapsuleTest,    c_TriMeshBVHHullTest,	0, },
};

static f32 (*c_distance_methods[C_SHAPE_COUNT][C_SHAPE_COUNT])(v3f64 *c0, v3f64 *c1, const struct c_Shape *, const ds_TransformWorld *, const struct c_Shape *, const ds_TransformWorld *) =
{
	{ c_SphereDistance,	 	        0,				                0, 			                0, },
	{ c_CapsuleSphereDistance,	    c_CapsuleDistance, 		        0, 			                0, },
	{ c_HullSphereDistance, 		c_HullCapsuleDistance, 		    c_HullDistance,		        0, },
	{ c_TriMeshBVHSphereDistance,	c_TriMeshBVHCapsuleDistance, 	c_TriMeshBVHHullDistance,	0, },
};

static f32 (*c_raycast_parameter_methods[C_SHAPE_COUNT])(const struct c_Shape *, const ds_TransformWorld *, const struct rayWorld *) =
{
	c_SphereRaycastParameter,
	c_CapsuleRaycastParameter,
	c_HullRaycastParameter,
	c_TriMeshBVHRaycastParameter,
};

u32 ds_ShapeTest(const struct ds_Dynamics *pipeline, const struct ds_Shape *s0, const struct ds_Shape *s1)
{
 	const struct c_Shape *c_s0 = c_ShapeDBShape(pipeline->external->cshape_db, s0->cshape_handle);
	const struct c_Shape *c_s1 = c_ShapeDBShape(pipeline->external->cshape_db, s1->cshape_handle);

    ds_TransformWorld t0, t1;
    ds_ShapeWorldTransform(&t0, pipeline, s0);
    ds_ShapeWorldTransform(&t1, pipeline, s1);
	
	return (c_s0->type >= c_s1->type)  
		? c_shape_tests[c_s0->type][c_s1->type](c_s0, &t0, c_s1, &t1)
		: c_shape_tests[c_s1->type][c_s0->type](c_s1, &t1, c_s0, &t0);
}

f32 ds_ShapeDistance(v3f64 *c0, v3f64 *c1, const struct ds_Dynamics *pipeline, const struct ds_Shape *s0, const struct ds_Shape *s1)
{
 	const struct c_Shape *c_s0 = c_ShapeDBShape(pipeline->external->cshape_db, s0->cshape_handle);
	const struct c_Shape *c_s1 = c_ShapeDBShape(pipeline->external->cshape_db, s1->cshape_handle);

    ds_TransformWorld t0, t1;
    ds_ShapeWorldTransform(&t0, pipeline, s0);
    ds_ShapeWorldTransform(&t1, pipeline, s1);

	return (c_s0->type >= c_s1->type)  
		? c_distance_methods[c_s0->type][c_s1->type](c0, c1, c_s0, &t0, c_s1, &t1)
		: c_distance_methods[c_s1->type][c_s0->type](c1, c0, c_s1, &t1, c_s0, &t0);
}

/* convex types precede C_SHAPE_TRI_MESH; [s0->type][s1->type] with s0 the more complex shape */
static u32 (*c_convex_contact_methods[C_SHAPE_TRI_MESH][C_SHAPE_TRI_MESH])(struct c_Manifold *, struct ds_ConvexCache *, const struct c_Shape *, const ds_TransformWorld *, const struct c_Shape *, const ds_TransformWorld *) =
{
	{ c_SphereContact,          0,                      0,              },
	{ c_CapsuleSphereContact,   c_CapsuleContact,       0,              },
	{ c_HullSphereContact,      c_HullCapsuleContact,   c_HullContact,  },
};

/*
 * One local manifold to the solver form; rot is the world rotation R of the shape it is local to. If the other shape
 * is the reference (flip), its surface points are the local ones minus the depth:
 *      n' = -n,    v' = v - depth*n
 * Points move to body 0's center of mass com0, offset = demote(t.position - com0) with one f64 subtraction:
 *      anchor0 = R*v + offset
 */
static void ds_SolverManifoldConvex(struct ds_SolverManifold *out, const struct c_Manifold *local, const m3 rot, const v3 offset, const u32 flip)
{
#ifdef DS_DEBUG
    if (!c_ManifoldCheck(local))
    {
        c_ManifoldDebugPrint(local);
    }
    ds_Assert(c_ManifoldCheck(local));
#endif

    for (u32 i = 0; i < local->v_count; ++i)
    {
        const v3 v = (flip)
                   ? V3AddScaled(local->v[i], local->n, -local->depth[i])
                   : local->v[i];
        out->anchor0[i] = V3Add(M3V3Mul(rot, v), offset);
        out->depth[i] = local->depth[i];
        out->id[i] = local->id[i];
    }
    out->normal = M3V3Mul(rot, (flip) ? V3Negate(local->n) : local->n);
    out->point_count = local->v_count;
}

static void ds_SolverManifoldSetCache(struct ds_SolverManifold *m, const ds_Array(ds_SolverManifold) old)
{
    memset(m->normal_impulse, 0, sizeof(m->normal_impulse));
    memset(m->tangent_impulse, 0, sizeof(m->tangent_impulse));

    const struct ds_SolverManifold *o = NULL;
    for (u32 t = 0; t < old.length; ++t)
    {
        if (m->tri == old.buf[t].tri)
        {
            o = old.buf + t;
            break;
        }
    }

    if (o)
    {
        struct ds_DynamicsMetrics *metrics = &g_dynamics_worker[ds_ThreadSelfIndex()].metrics;
        metrics->warm_point_count += m->point_count;
        u32 claimed = 0;
        for (u32 p = 0; p < m->point_count; ++p)
        {
            for (u32 k = 0; k < o->point_count; ++k)
            {
                if (!(claimed & (1u << k)) && c_PointIdEqual(m->id[p], o->id[k]))
                {
                    claimed |= 1u << k;
                    metrics->warm_match_id += 1;
                    m->normal_impulse[p] = o->normal_impulse[k];
                    m->tangent_impulse[p] = o->tangent_impulse[k];
                    break;
                }
            }
        }
    }
}

static void ds_SolverManifoldConcave(struct arena *tmp, ds_TSlab(ds_SolverManifold) *slab, struct ds_SolverManifoldBuffer *buffer, const ds_Array(c_Manifold) *result, const m3 rot, const v3 offset, const u32 flip)
{
    if (result->length == 0)
    {
        if (buffer->type == SOLVER_MANIFOLD_EXTERNAL)
        {
            ds_TSlabRemove(*slab, buffer->manifold_array);
        }
        buffer->count = 0; 
        buffer->type = SOLVER_MANIFOLD_INTERNAL;
        return;
    }

    ds_Array(ds_SolverManifold) new = 
    {
        .buf = (buffer->type == SOLVER_MANIFOLD_INTERNAL) ? &buffer->manifold : buffer->manifold_array.buf,
        .length = buffer->count,
    };
    const ds_Array(ds_SolverManifold) old = ds_ArrayCopy(tmp, new);

    const u32 capacity = (buffer->type == SOLVER_MANIFOLD_INTERNAL) ? 1 : buffer->manifold_array.length;
    const u32 grow = capacity < result->length;
    if (grow)
    {
        new = ds_TSlabAdd(*slab, result->length);
    }

    for (u32 i = 0; i < result->length; ++i)
    {
        new.buf[i].tri = result->buf[i].tri;
        ds_SolverManifoldConvex(new.buf + i, result->buf + i, rot, offset, flip);
        ds_SolverManifoldSetCache(new.buf + i, old);
    }
    
    buffer->count = result->length;
    if (grow)
    {
        if (buffer->type == SOLVER_MANIFOLD_EXTERNAL)
        {
            ds_TSlabRemove(*slab, buffer->manifold_array);
        }
        buffer->type = SOLVER_MANIFOLD_EXTERNAL;
        buffer->manifold_array = new;
    }
}

/*
 * The pipeline's contact entry. The narrowphase returns manifolds local to one shape of the pair (c_*Contact: s0;
 * c_MeshContact: the convex shape). local and other are the contact key indices of that shape and of the other one.
 * local is the more complex shape (higher type; equal types: the contact key's order), except in mesh pairs: the
 * mesh has the highest type, but c_MeshContact returns manifolds local to the convex shape.
 */
void ds_ShapeContact(struct arena *tmp, struct ds_Dynamics *pipeline, const u32 contact_index)
{
    ArenaPushRecord(tmp);

    struct ds_Contact *c = pipeline->contact_pool.buf + contact_index;

    const struct ds_Shape *s[2] =
    {
        pipeline->shape_pool.buf + c->key.shape[0],
        pipeline->shape_pool.buf + c->key.shape[1],
    };

    const struct c_Shape *c_s[2] =
    {
        c_ShapeDBShape(pipeline->external->cshape_db, s[0]->cshape_handle),
        c_ShapeDBShape(pipeline->external->cshape_db, s[1]->cshape_handle),
    };

    ds_TransformWorld t_world[2];
    ds_ShapeWorldTransform(t_world + 0, pipeline, s[0]);
    ds_ShapeWorldTransform(t_world + 1, pipeline, s[1]);

    v3f64 com[2];
    for (u32 i = 0; i < 2; ++i)
    {
        const struct ds_Body *body = pipeline->body_pool.buf + s[i]->body;
        const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + body->set;
        com[i] = ds_BodySimWorldCenterOfMass(set->body_sim_pool.buf + body->sim);
    }
    c->com_offset = V3F64Demote(V3F64Sub(com[1], com[0]));

    u32 local;
    if (ds_ContactConvexCheck(c))
    {
        local = (c_s[0]->type >= c_s[1]->type) ? 0 : 1;
        const u32 flip_normal = local;
        const u32 other = 1 - local;

        struct c_Manifold local_manifold;
        const u32 contact = c_convex_contact_methods[c_s[local]->type][c_s[other]->type](&local_manifold, &c->cache.convex, c_s[local], &t_world[local], c_s[other], &t_world[other]);
        struct ds_SolverManifoldBuffer *buffer = pipeline->manifold_buffer_pool.buf + contact_index;

        struct ds_SolverManifold old_manifold = buffer->manifold;
        const ds_Array(ds_SolverManifold) old = { .buf = &old_manifold, .length = buffer->count, .base = U32_MAX };
        buffer->count = contact;
        if (contact)
        {
            const m3 rot = M3QUnit(t_world[local].rotation);
            const v3 offset = V3F64Demote(V3F64Sub(t_world[local].position, com[0]));
            ds_SolverManifoldConvex(&buffer->manifold, &local_manifold, rot, offset, flip_normal);
            ds_SolverManifoldSetCache(&buffer->manifold, old);
        }
    }
    else
    {
        const u32 mesh = ds_ContactStaticIndex(c);
        const u32 convex = 1 - mesh;
        const u32 flip_normal = convex;

        const ds_Array(c_Manifold) result = c_MeshContact(tmp, &pipeline->tri_cache_slab, &c->cache.concave, c_s[mesh], &t_world[mesh], c_s[convex], &t_world[convex]);
        const m3 rot = M3QUnit(t_world[convex].rotation);
        const v3 offset = V3F64Demote(V3F64Sub(t_world[convex].position, com[0]));
        ds_SolverManifoldConcave(tmp, &pipeline->manifold_slab, pipeline->manifold_buffer_pool.buf + contact_index, &result, rot, offset, flip_normal);
    }

    ArenaPopRecord(tmp);
}

f32 ds_ShapeRaycastParameter(const struct ds_Dynamics *pipeline, const struct ds_Shape *shape, const struct rayWorld *ray)
{
    ds_Assert(V3Dot(ray->dir, ray->dir) > 0.0f);
    ds_TransformWorld transform;
    ds_ShapeWorldTransform(&transform, pipeline, shape);
    const struct c_Shape *c_shape = c_ShapeDBShape(pipeline->external->cshape_db, shape->cshape_handle);

	return c_raycast_parameter_methods[c_shape->type](c_shape, &transform, ray);
}
