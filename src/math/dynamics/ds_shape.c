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

POOL_DEFINE(ds_Shape);
SDB_DEFINE(ds_ShapePrefab);
POOL_DEFINE(ds_ShapePrefabInstance);

ds_ShapeId ds_ShapeAdd(struct ds_Dynamics *pipeline, const struct ds_ShapePrefab *prefab, const ds_Transform *t, const ds_BodyId body_id)
{
    struct slot	body_slot = ds_BodyLookup(pipeline, body_id);
    struct ds_Body *body = body_slot.address;
    if (!body)
    {
        return DS_ID_NULL;
    }

    ds_ShapeId id = DS_ID_NULL;
    const u32 old_max = pipeline->shape_pool.count_max;
    const struct slot shape_slot = ds_ShapePoolAdd(&pipeline->shape_pool);
	struct ds_Shape *shape = shape_slot.address;

    if (old_max != pipeline->shape_pool.count_max)
    {
        shape->id = ds_IdConstruct(shape_slot.index, 0);
        ds_CPoolPush(pipeline->dirty_shape_query);
    }
    shape->id += DS_ID_GENERATION_INCREMENT;

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
	shape->body = ds_IdIndex(body_id);
	shape->density = prefab->density;
	shape->restitution = prefab->restitution;
	shape->friction = prefab->friction;
	shape->t_local = *t;
	shape->margin = prefab->margin;
	DLLFlush(shape->contact_list);

	const struct c_Shape *cshape = pipeline->cshape_db->pool.buf + prefab->cshape;
	const struct slot cshape_slot = c_ShapeSDBReference(pipeline->cshape_db, cshape->id);
	shape->cshape_handle = cshape_slot.index;
	shape->cshape_type = cshape->type;

	struct aabb bbox_proxy = ds_ShapeWorldBbox(pipeline, shape);
    if (ds_BodyDynamicCheck(body))
    {
		bbox_proxy.hw = V3AddConstant(bbox_proxy.hw, shape->margin);
        ds_BitSetSet(&pipeline->shape_dynamic_usage_set, shape_slot.index, 1);
        ds_BitSetSet(&pipeline->shape_dirty_set, shape_slot.index, 1);
        shape->proxy = DbvhInsert(&pipeline->dynamic_bvh, shape->body, shape_slot.index, &bbox_proxy);
    }
    else
    {
        shape->proxy = DbvhInsert(&pipeline->static_bvh, shape->body, shape_slot.index, &bbox_proxy);
    }
    
    ds_BodyUpdateMassProperties(pipeline, body_id);

    return shape->id;
}

void ds_ShapeDynamicRemove(struct ds_Dynamics *pipeline, struct ds_Body *body, const u32 shape_index, const u32 mass_properties_update)
{
    struct ds_Shape *shape = pipeline->shape_pool.buf + shape_index;

    while (shape->contact_list.count)
	{
        ds_ContactRemove(pipeline, shape->contact_list.first);
    }

    if (mass_properties_update)
    {
        ds_BodyUpdateMassProperties(pipeline, body->id);
    }

    ds_BitSetSet(&pipeline->shape_dynamic_usage_set, shape_index, 0);
    ds_BitSetSet(&pipeline->shape_dirty_set, shape_index, 0);
    DLLRemove(body->shape_list, pipeline->shape_pool.buf, shape_index, body_shape);
	c_ShapeSDBDereference(pipeline->cshape_db, shape->cshape_handle);
	DbvhRemove(&pipeline->dynamic_bvh, shape->proxy);
	ds_ShapePoolRemove(&pipeline->shape_pool, shape_index);
}

void ds_ShapeStaticRemove(struct arena *mem_tmp, struct ds_Dynamics *pipeline, struct ds_Body *body, const u32 index)
{
	struct ds_Shape *shape = pipeline->shape_pool.buf + index;

    while (shape->contact_list.count)
	{
        ds_ContactRemove(pipeline, shape->contact_list.first);
    }

    ds_BitSetSet(&pipeline->shape_dirty_set, index, 0);
    DLLRemove(body->shape_list, pipeline->shape_pool.buf, index, body_shape);
	c_ShapeSDBDereference(pipeline->cshape_db, shape->cshape_handle);
	DbvhRemove(&pipeline->static_bvh, shape->proxy);
	ds_ShapePoolRemove(&pipeline->shape_pool, index);
}

struct slot ds_ShapeLookup(const struct ds_Dynamics *pipeline, const ds_ShapeId shape_id)
{

    struct slot slot = { .address = NULL, .index = U32_MAX };
    struct ds_Shape *shape = pipeline->shape_pool.buf + ds_IdIndex(shape_id);
    if (shape_id != DS_ID_NULL && ds_PoolSlotAllocated(shape) && shape->id == shape_id)
    {
        slot.address = shape;
        slot.index = ds_IdIndex(shape_id);
    }

    return slot;
}

void ds_ShapeWorldTransform(ds_Transform *t, const struct ds_Dynamics *pipeline, const struct ds_Shape *shape)
{
	const struct ds_Body *body = pipeline->body_pool.buf + shape->body;
    const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + body->set;
    const struct ds_BodySim *sim = set->body_sim_pool.buf + body->sim;
    const q world_rotation = sim->world.rotation;
    const m3 rot = M3Q(world_rotation);

    t->rotation = QMul(world_rotation, shape->t_local.rotation);
    t->position = V3Add(M3V3Mul(rot, shape->t_local.position), sim->world.position);
}

struct aabb ds_ShapeWorldBbox(const struct ds_Dynamics *pipeline, const struct ds_Shape *shape)
{
	v3 min = V3(F32_INFINITY, F32_INFINITY, F32_INFINITY);
	v3 max = V3(-F32_INFINITY, -F32_INFINITY, -F32_INFINITY);

	const struct c_Shape *cshape = pipeline->cshape_db->pool.buf + shape->cshape_handle;

    ds_Transform t_world;
    ds_ShapeWorldTransform(&t_world, pipeline, shape);
	const m3 rot = M3Q(t_world.rotation);
	const v3 world_position = t_world.position;

	if (shape->cshape_type == C_SHAPE_CONVEX_HULL)
	{
		for (u32 i = 0; i < cshape->hull.v_count; ++i)
		{
			const v3 v = V3Add(M3V3Mul(rot, cshape->hull.v[i]), world_position);
			min = V3Min(min, v);
			max = V3Max(max, v);
		}
	}
	else if (shape->cshape_type == C_SHAPE_SPHERE)
	{
		const f32 r = cshape->sphere.radius;
		min = V3Add(V3(-r, -r, -r), world_position);
		max = V3Add(V3(r, r, r), world_position);
	}
	else if (shape->cshape_type == C_SHAPE_CAPSULE)
	{
		const v3 v = M3V3Mul(rot, V3(0.0f, cshape->capsule.half_height, 0.0f));
		max = V3AddConstant(V3Abs(v), cshape->capsule.radius);
		min = V3Negate(max);

		min = V3Add(min, world_position);
		max = V3Add(max, world_position);
	}
	else if (shape->cshape_type == C_SHAPE_TRI_MESH)
	{
		//TODO "We treat Tri meshes differently; a rigid body who has a tri mesh attached
		// views the tri mesh triangles and its shapes. Thus such a rigid body treats its
		// mesh shape to have position 0 and no rotation.
        ds_Assert(V3Length(shape->t_local.position) == 0.0f);
        ds_Assert(shape->t_local.rotation.w == 1.0f);
		const struct bvhNode *node = cshape->mesh_bvh.bvh.pool.buf;
		struct aabb bbox;
		AabbRotate(&bbox, &node[cshape->mesh_bvh.bvh.bt.root].bbox, rot);
		const v3 hw = bbox.hw;
		min = V3Add(V3Negate(hw), world_position);
		max = V3Add(hw, world_position);
	}

	const v3 hw = V3Scale(V3Sub(max, min), 0.5f);
	struct aabb bbox;
	bbox.hw = hw;
	bbox.center = V3Add(min, hw);
	return bbox;
}

/********************************** LOOKUP TABLES FOR SHAPES **********************************/

u32 (*c_shape_tests[C_SHAPE_COUNT][C_SHAPE_COUNT])(const struct c_Shape *, const ds_Transform *, const struct c_Shape *, const ds_Transform *) =
{
	{ c_SphereTest, 		    0, 				            0, 			            0, },
	{ c_CapsuleSphereTest,	    c_CapsuleTest, 			    0, 			            0, },
	{ c_HullSphereTest, 		c_HullCapsuleTest,		    c_HullTest,		        0, },
	{ c_TriMeshBvhSphereTest,   c_TriMeshBvhCapsuleTest,    c_TriMeshBvhHullTest,	0, },
};

f32 (*c_distance_methods[C_SHAPE_COUNT][C_SHAPE_COUNT])(v3 *c1, v3 *c2, const struct c_Shape *, const ds_Transform *, const struct c_Shape *, const ds_Transform *) =
{
	{ c_SphereDistance,	 	        0,				                0, 			                0, },
	{ c_CapsuleSphereDistance,	    c_CapsuleDistance, 		        0, 			                0, },
	{ c_HullSphereDistance, 		c_HullCapsuleDistance, 		    c_HullDistance,		        0, },
	{ c_TriMeshBvhSphereDistance,	c_TriMeshBvhCapsuleDistance, 	c_TriMeshBvhHullDistance,	0, },
};

struct c_ContactResult  (*c_contact_methods[C_SHAPE_COUNT][C_SHAPE_COUNT])(struct arena *, const struct c_ContactResult *, const struct c_Shape *[2], const ds_Transform [2], const u32) =
{
	{ c_SphereContact,	 	        0, 				            0,			                0, },
	{ c_CapsuleSphereContact, 	    c_CapsuleContact,			0,			                0, },
	{ c_HullSphereContact, 	  	    c_HullCapsuleContact,		c_HullContact, 		        0, },
	{ c_TriMeshBvhSphereContact,    c_TriMeshBvhCapsuleContact, c_TriMeshBvhHullContact,    0, },
};


f32 (*c_raycast_parameter_methods[C_SHAPE_COUNT])(const struct c_Shape *, const ds_Transform *, const struct ray *) =
{
	c_SphereRaycastParameter,
	c_CapsuleRaycastParameter,
	c_HullRaycastParameter,
	c_TriMeshBvhRaycastParameter,
};

u32 ds_ShapeTest(const struct ds_Dynamics *pipeline, const struct ds_Shape *s1, const struct ds_Shape *s2)
{
 	const struct c_Shape *c_s1 = pipeline->cshape_db->pool.buf + s1->cshape_handle;
	const struct c_Shape *c_s2 = pipeline->cshape_db->pool.buf + s2->cshape_handle;

    ds_Transform t1, t2;
    ds_ShapeWorldTransform(&t1, pipeline, s1);
    ds_ShapeWorldTransform(&t2, pipeline, s2);
	
	return (c_s1->type >= c_s2->type)  
		? c_shape_tests[c_s1->type][c_s2->type](c_s1, &t1, c_s2, &t2)
		: c_shape_tests[c_s2->type][c_s1->type](c_s2, &t2, c_s1, &t1);
}

f32 ds_ShapeDistance(v3 *c1, v3 *c2, const struct ds_Dynamics *pipeline, const struct ds_Shape *s1, const struct ds_Shape *s2)
{
 	const struct c_Shape *c_s1 = pipeline->cshape_db->pool.buf + s1->cshape_handle;
	const struct c_Shape *c_s2 = pipeline->cshape_db->pool.buf + s2->cshape_handle;

    ds_Transform t1, t2;
    ds_ShapeWorldTransform(&t1, pipeline, s1);
    ds_ShapeWorldTransform(&t2, pipeline, s2);

	return (c_s1->type >= c_s2->type)  
		? c_distance_methods[c_s1->type][c_s2->type](c1, c2, c_s1, &t1, c_s2, &t2)
		: c_distance_methods[c_s2->type][c_s1->type](c2, c1, c_s2, &t2, c_s1, &t1);
}

void ds_ShapeContact(struct arena *frame, const struct ds_Dynamics *pipeline, const u32 contact_index)
{
    struct ds_Contact *c = pipeline->contact_pool.buf + contact_index;

    const struct ds_Shape *s[2] =
    {
        pipeline->shape_pool.buf + c->key.shape[0],
        pipeline->shape_pool.buf + c->key.shape[1],
    };

    const struct c_Shape *c_s[2] =
    {
        pipeline->cshape_db->pool.buf + s[0]->cshape_handle,
        pipeline->cshape_db->pool.buf + s[1]->cshape_handle,
    };

    /* index of most complex shape in c_s_arr */
    const u32 m = (c_s[0]->type >= c_s[1]->type) 
                ? 0
                : 1;

    const u32 reference = (s[m]->body < s[1-m]->body)
                        ? 0
                        : 1;
    const struct c_Shape *c_s_arr[2] = 
    { 
        c_s[m], 
        c_s[1-m],
    };

    ds_Transform t_arr[2];
    ds_ShapeWorldTransform(t_arr + 0, pipeline, s[m]);
    ds_ShapeWorldTransform(t_arr + 1, pipeline, s[1-m]);

    const struct c_ContactResult result = c_contact_methods[c_s[m]->type][c_s[1-m]->type](frame, &c->narrowphase, c_s_arr, t_arr, reference);

    if (result.manifold_count && !c_ManifoldCheck(result.manifold))
    {
        c_ManifoldDebugPrint(result.manifold);
        Breakpoint(1);
        c->narrowphase = c_contact_methods[c_s[m]->type][c_s[1-m]->type](frame, &c->narrowphase, c_s_arr, t_arr, reference);
    }
	c->narrowphase = result;
}

f32 ds_ShapeRaycastParameter(const struct ds_Dynamics *pipeline, const struct ds_Shape *shape, const struct ray *ray)
{
    ds_Transform transform;
    ds_ShapeWorldTransform(&transform, pipeline, shape);
    const struct c_Shape *c_shape = pipeline->cshape_db->pool.buf + shape->cshape_handle;

	return c_raycast_parameter_methods[c_shape->type](c_shape, &transform, ray);
}

u32 ds_ShapeRaycast(v3 *intersection, const struct ds_Dynamics *pipeline, const struct ds_Shape *shape, const struct ray *ray)
{
	const f32 t = ds_ShapeRaycastParameter(pipeline, shape, ray);
	if (t == F32_INFINITY) return 0;

	*intersection = V3AddScaled(ray->origin, ray->dir, t);
	return 1;
}
