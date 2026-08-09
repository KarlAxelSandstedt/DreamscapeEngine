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

#include "dynamics.h"

POOL_DEFINE(ds_Shape);

ds_ShapeId ds_ShapeAdd(struct ds_RigidBodyPipeline *pipeline, const struct ds_ShapePrefab *prefab, const ds_Transform *t, const ds_RigidBodyId body)
{
    ds_ShapeId id = DS_ID_NULL;
    struct slot slot = ds_ShapePoolAdd(&pipeline->shape_pool);
	if (slot.address)
	{
		struct ds_RigidBody *body_ptr = pipeline->body_pool.buf + ds_IdIndex(body);
		ds_Assert(ds_PoolSlotAllocated(body_ptr));
		ds_DLLAppend(body_ptr->shape_list, pipeline->shape_pool.buf, slot.index, body_shape);

		struct ds_Shape *shape = slot.address;

        shape->tag += DS_ID_TAG_GENERATION_INCREMENT;
        id = ((u64) shape->tag << 32) | slot.index;
		shape->body = ds_IdIndex(body);
		shape->density = prefab->density;
		shape->restitution = prefab->restitution;
		shape->friction = prefab->friction;
		shape->t_local = *t;
		shape->margin = prefab->margin;
		ds_DLLFlush(&shape->contact_list);

		const struct c_Shape *cshape = strdb_Address(pipeline->cshape_db, prefab->cshape);
		const struct slot cshape_slot = strdb_Reference(pipeline->cshape_db, cshape->id);
		shape->cshape_handle = cshape_slot.index;
		shape->cshape_type = cshape->type;

		struct aabb bbox_proxy = ds_ShapeWorldBbox(pipeline, shape);
		if (shape->cshape_type != C_SHAPE_TRI_MESH)
		{
			Vec3Translate(bbox_proxy.hw, Vec3Inline(shape->margin, shape->margin, shape->margin));
		}
		shape->proxy = DbvhInsert(&pipeline->shape_bvh, slot.index, &bbox_proxy);

        ds_RigidBodyUpdateMassProperties(pipeline, body);
	}

    return id;
}

void ds_ShapeDynamicRemove(struct ds_RigidBodyPipeline *pipeline, struct ds_RigidBody *body, const u32 shape_index, const u32 mass_properties_update)
{
	struct ds_Island *island = pipeline->is_db.island_pool.buf + body->island_index;
    struct ds_Shape *dummy_shape, *shape = pipeline->shape_pool.buf + shape_index;
    struct ds_RigidBody *dummy_body;
    const ds_ShapeId s0 = ((u64) shape->tag << 32) | shape_index;
    const ds_RigidBodyId b0 = ((u64) body->tag << 32) | shape->body;

    ds_DLLRemove(body->shape_list, pipeline->shape_pool.buf, shape_index, body_shape);
	strdb_Dereference(pipeline->cshape_db, shape->cshape_handle);
	DbvhRemove(&pipeline->shape_bvh, shape->proxy);

    while (shape->contact_list.first != DLL_SENTINEL)
	{
        ds_ContactRemove(pipeline, shape->contact_list.first);
    }

    if (mass_properties_update)
    {
        ds_RigidBodyUpdateMassProperties(pipeline, b0);
    }

	ds_ShapePoolRemove(&pipeline->shape_pool, shape_index);
}

void ds_ShapeStaticRemove(struct arena *mem_tmp, struct ds_RigidBodyPipeline *pipeline, struct ds_RigidBody *body, const u32 index, const u32 mass_properties_update)
{
	struct ds_Shape *dynamic_shape, *shape = pipeline->shape_pool.buf + index;
    struct ds_RigidBody *dynamic_body;
    const u64 s0 = ((u64) shape->tag << 32) | index;
    const u64 b0 = ((u64) body->tag << 32) | shape->body;
    const u32 static_is_tri_mesh = shape->cshape_type == C_SHAPE_TRI_MESH;

    ds_Assert(pipeline->body_pool.buf[shape->body].island_index == ISLAND_STATIC);

	ArenaPushRecord(&pipeline->frame);
	struct memArray arr = ArenaPushAlignedAll(&pipeline->frame, sizeof(u32), sizeof(u32));
	u32 *island = arr.addr;
	u32 island_count = 0;

    while (shape->contact_list.first != DLL_SENTINEL)
	{
        const u32 ci = shape->contact_list.first;
		struct ds_Contact *c = pipeline->cdb->contact_pool.buf + ci;

		if (index == c->key.shape0 || (static_is_tri_mesh && INDIRECT_SHAPE_CHECK(c->key.shape0)))
		{
            ds_ContactKeyAddress(&body, &shape, &dynamic_body, &dynamic_shape, pipeline, &c->key);
		}
		else
		{
            ds_ContactKeyAddress(&dynamic_body, &dynamic_shape, &body, &shape, pipeline, &c->key);
		}

        ds_Assert(dynamic_body->island_index != ISLAND_STATIC);
		struct ds_Island *is = pipeline->is_db.island_pool.buf + dynamic_body->island_index;
		if ((is->flags & ISLAND_SPLIT) == 0)
		{
		    if (island_count == arr.len)
			{
				LogString(T_SYSTEM, S_FATAL, "Stack OOM in ds_ShapeStaticRemove");
				FatalCleanupAndExit();
			}
			island[island_count++] = dynamic_body->island_index;
			
			is->flags |= ISLAND_SPLIT;
		}
        ds_ContactRemove(pipeline, ci);
    }

	for (u32 i = 0; i < island_count; ++i)
	{
		struct ds_Island *is = pipeline->is_db.island_pool.buf + island[i];
		if (is->contact_list.count > 0)
		{
			isdb_SplitIsland(mem_tmp, pipeline, island[i]);
		}
		else
		{
			is->flags &= ~(ISLAND_SPLIT);
			if (is->set >= SOLVER_SET_SLEEPING_FIRST)
			{
				PhysicsEventIslandAwake(pipeline, island[i]);	
			}
			is->flags |= ISLAND_SLEEP_RESET;
		}
	}
    ArenaPopRecord(&pipeline->frame);

    if (mass_properties_update)
    {
        ds_RigidBodyUpdateMassProperties(pipeline, b0);
    }

    ds_DLLRemove(body->shape_list, pipeline->shape_pool.buf, index, body_shape);
	strdb_Dereference(pipeline->cshape_db, shape->cshape_handle);
	DbvhRemove(&pipeline->shape_bvh, shape->proxy);
	ds_ShapePoolRemove(&pipeline->shape_pool, index);
}

struct slot ds_ShapeLookup(const struct ds_RigidBodyPipeline *pipeline, const ds_ShapeId shape_id)
{
    struct slot slot = { .address = NULL, .index = 0 };
    struct ds_Shape *shape = pipeline->shape_pool.buf + ds_IdIndex(shape_id);
    if (shape_id != DS_ID_NULL && ds_PoolSlotAllocated(shape) && shape->tag == ds_IdTag(shape_id))
    {
        slot.address = shape;
        slot.index = ds_IdIndex(shape_id);
    }

    return slot;
}

void ds_ShapeWorldTransform(ds_Transform *t, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *shape)
{
	const struct ds_RigidBody *body = pipeline->body_pool.buf + shape->body;
    mat3 rot;
    Mat3Quat(rot, body->t_world.rotation);

    QuatMul(t->rotation, body->t_world.rotation, shape->t_local.rotation);
    Mat3VecMul(t->position, rot, shape->t_local.position);
    Vec3Translate(t->position, body->t_world.position);
}

struct aabb ds_ShapeWorldBbox(const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *shape)
{
	vec3 min = { F32_INFINITY, F32_INFINITY, F32_INFINITY };
	vec3 max = { -F32_INFINITY, -F32_INFINITY, -F32_INFINITY };

	const struct ds_RigidBody *body = pipeline->body_pool.buf + shape->body;
	const struct c_Shape *cshape = strdb_Address(pipeline->cshape_db, shape->cshape_handle);

    mat3 rot;
    ds_Transform t_world;
    ds_ShapeWorldTransform(&t_world, pipeline, shape);
	Mat3Quat(rot, t_world.rotation);

	vec3 v, tmp;
	if (shape->cshape_type == C_SHAPE_CONVEX_HULL)
	{
		for (u32 i = 0; i < cshape->hull.v_count; ++i)
		{
			Mat3VecMul(v, rot, cshape->hull.v[i]);
			Vec3Translate(v, t_world.position);

			min[0] = f32_min(min[0], v[0]); 
			min[1] = f32_min(min[1], v[1]);			
			min[2] = f32_min(min[2], v[2]);			
                                                   
			max[0] = f32_max(max[0], v[0]);			
			max[1] = f32_max(max[1], v[1]);			
			max[2] = f32_max(max[2], v[2]);			
		}
	}
	else if (shape->cshape_type == C_SHAPE_SPHERE)
	{
		const f32 r = cshape->sphere.radius;
		Vec3Set(min, -r, -r, -r);
		Vec3Set(max, r, r, r);
		Vec3Translate(min, shape->t_local.position);
		Vec3Translate(max, shape->t_local.position);
		Vec3Translate(min, body->t_world.position);
		Vec3Translate(max, body->t_world.position);
	}
	else if (shape->cshape_type == C_SHAPE_CAPSULE)
	{
		tmp[0] = 0.0f;	
		tmp[1] = cshape->capsule.half_height;	
		tmp[2] = 0.0f;	
		Mat3VecMul(v, rot, tmp);

		Vec3Abs(max, v);
		Vec3AddConstant(max, cshape->capsule.radius);
		Vec3Negate(min, max);

		Vec3Translate(min, t_world.position);
		Vec3Translate(max, t_world.position);
	}
	else if (shape->cshape_type == C_SHAPE_TRI_MESH)
	{
		//TODO "We treat Tri meshes differently; a rigid body who has a tri mesh attached
		// views the tri mesh triangles and its shapes. Thus such a rigid body treats its
		// mesh shape to have position 0 and no rotation.
        ds_Assert(Vec3Length(shape->t_local.position) == 0.0f);
        ds_Assert(shape->t_local.rotation[3] == 1.0f);
		const struct bvhNode *node = (struct bvhNode *) cshape->mesh_bvh.bvh.tree.pool.buf;
		struct aabb bbox; 
		AabbRotate(&bbox, &node[cshape->mesh_bvh.bvh.tree.root].bbox, rot);
		Vec3Scale(min, bbox.hw, -1.0f);
		Vec3Scale(max, bbox.hw, 1.0f);
		Vec3Translate(min, t_world.position);
		Vec3Translate(max, t_world.position);
	}

	struct aabb bbox;
	Vec3Sub(bbox.hw, max, min);
	Vec3ScaleSelf(bbox.hw, 0.5f);
	Vec3Add(bbox.center, min, bbox.hw);
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

f32 (*c_distance_methods[C_SHAPE_COUNT][C_SHAPE_COUNT])(vec3 c1, vec3 c2, const struct c_Shape *, const ds_Transform *, const struct c_Shape *, const ds_Transform *) =
{
	{ c_SphereDistance,	 	        0,				                0, 			                0, },
	{ c_CapsuleSphereDistance,	    c_CapsuleDistance, 		        0, 			                0, },
	{ c_HullSphereDistance, 		c_HullCapsuleDistance, 		    c_HullDistance,		        0, },
	{ c_TriMeshBvhSphereDistance,	c_TriMeshBvhCapsuleDistance, 	c_TriMeshBvhHullDistance,	0, },
};

u32 (*c_contact_methods[C_SHAPE_COUNT][C_SHAPE_COUNT])(struct c_Manifold *, struct sat_Cache *, const struct sat_Cache *, const struct c_Shape *[2], const ds_Transform [2], const u32) =
{
	{ c_SphereContact,	 	        0, 				            0,			                0, },
	{ c_CapsuleSphereContact, 	    c_CapsuleContact,			0,			                0, },
	{ c_HullSphereContact, 	  	    c_HullCapsuleContact,		c_HullContact, 		        0, },
	{ 0,	                        0,                          0,                          0, },
};

u32 (*c_mesh_contact_methods[C_SHAPE_COUNT])(struct arena *, struct c_Manifold **, u32 **, struct sat_Cache *, const struct c_Shape *[2], const ds_Transform [2], const u32) =
{
    c_TriMeshBvhSphereContact,
    c_TriMeshBvhCapsuleContact,
    c_TriMeshBvhHullContact,
    0,
};

f32 (*c_raycast_parameter_methods[C_SHAPE_COUNT])(const struct c_Shape *, const ds_Transform *, const struct ray *) =
{
	c_SphereRaycastParameter,
	c_CapsuleRaycastParameter,
	c_HullRaycastParameter,
	c_TriMeshBvhRaycastParameter,
};

u32 ds_ShapeTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *s1, const struct ds_Shape *s2)
{
 	const struct c_Shape *c_s1 = strdb_Address(pipeline->cshape_db, s1->cshape_handle);
	const struct c_Shape *c_s2 = strdb_Address(pipeline->cshape_db, s2->cshape_handle);

    ds_Transform t1, t2;
    ds_ShapeWorldTransform(&t1, pipeline, s1);
    ds_ShapeWorldTransform(&t2, pipeline, s2);
	
	return (c_s1->type >= c_s2->type)  
		? c_shape_tests[c_s1->type][c_s2->type](c_s1, &t1, c_s2, &t2)
		: c_shape_tests[c_s2->type][c_s1->type](c_s2, &t2, c_s1, &t1);
}

f32 ds_ShapeDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *s1, const struct ds_Shape *s2)
{
 	const struct c_Shape *c_s1 = strdb_Address(pipeline->cshape_db, s1->cshape_handle);
	const struct c_Shape *c_s2 = strdb_Address(pipeline->cshape_db, s2->cshape_handle);

    ds_Transform t1, t2;
    ds_ShapeWorldTransform(&t1, pipeline, s1);
    ds_ShapeWorldTransform(&t2, pipeline, s2);

	return (c_s1->type >= c_s2->type)  
		? c_distance_methods[c_s1->type][c_s2->type](c1, c2, c_s1, &t1, c_s2, &t2)
		: c_distance_methods[c_s2->type][c_s1->type](c2, c1, c_s2, &t2, c_s1, &t1);
}

u32 ds_ShapeContact(struct c_Manifold *manifold, struct sat_Cache *cache, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *s1, const struct ds_Shape *s2)
{
    ds_Transform t_arr[2];

    const struct c_Shape *c_s1 = strdb_Address(pipeline->cshape_db, s1->cshape_handle);
    const struct c_Shape *c_s2 = strdb_Address(pipeline->cshape_db, s2->cshape_handle);

    struct sat_Cache *cache_copy = NULL;
    struct sat_Cache cache_copy_mem;
    if (cache)
    {
        cache_copy = &cache_copy_mem;
        cache_copy_mem = *cache;
    }

	u32 collision_count;
	if (c_s1->type >= c_s2->type) 
	{
        const struct c_Shape *c_s_arr[2] = { c_s1, c_s2 };
        ds_ShapeWorldTransform(t_arr + 0, pipeline, s1);
        ds_ShapeWorldTransform(t_arr + 1, pipeline, s2);
        const u32 ref = (s1->body < s2->body)
                        ? 0
                        : 1;
		collision_count = c_contact_methods[c_s1->type][c_s2->type](manifold, cache, cache_copy, c_s_arr, t_arr, ref);
	}                                                                                         
	else                                                                                      
	{                                                                                         
        const struct c_Shape *c_s_arr[2] = { c_s2, c_s1 };
        ds_ShapeWorldTransform(t_arr + 0, pipeline, s2);
        ds_ShapeWorldTransform(t_arr + 1, pipeline, s1);
        const u32 ref = (s1->body < s2->body)
                        ? 1
                        : 0;
		collision_count = c_contact_methods[c_s2->type][c_s1->type](manifold, cache, cache_copy, c_s_arr, t_arr, ref);
	}

	return collision_count;
}

u32 ds_ShapeMeshContact(struct arena *frame, struct c_Manifold **manifold, u32 **triangle, struct sat_Cache *cache, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *s1, const struct ds_Shape *s2)
{
    ds_Transform t_arr[2];

    const struct c_Shape *c_s1 = strdb_Address(pipeline->cshape_db, s1->cshape_handle);
    const struct c_Shape *c_s2 = strdb_Address(pipeline->cshape_db, s2->cshape_handle);

    u32 collision_count;
    if (c_s2->type < c_s1->type)
    {
        const u32 ref = (s1->body < s2->body)
                    ? 0
                    : 1;
        const struct c_Shape *c_s_arr[2] = { c_s1, c_s2 };
        ds_ShapeWorldTransform(t_arr + 0, pipeline, s1);
        ds_ShapeWorldTransform(t_arr + 1, pipeline, s2);
	    collision_count = c_mesh_contact_methods[c_s2->type](frame, manifold, triangle, cache, c_s_arr, t_arr, ref);
    }
    else
    {
        const u32 ref = (s1->body < s2->body)
                    ? 1
                    : 0;
        const struct c_Shape *c_s_arr[2] = { c_s2, c_s1 };
        ds_ShapeWorldTransform(t_arr + 0, pipeline, s2);
        ds_ShapeWorldTransform(t_arr + 1, pipeline, s1);
	    collision_count = c_mesh_contact_methods[c_s1->type](frame, manifold, triangle, cache, c_s_arr, t_arr, ref);
    }
    return collision_count;
}


f32 ds_ShapeRaycastParameter(const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *shape, const struct ray *ray)
{
    ds_Transform transform;
    ds_ShapeWorldTransform(&transform, pipeline, shape);
    const struct c_Shape *c_shape = strdb_Address(pipeline->cshape_db, shape->cshape_handle);

	return c_raycast_parameter_methods[c_shape->type](c_shape, &transform, ray);
}

u32 ds_ShapeRaycast(vec3 intersection, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *shape, const struct ray *ray)
{
	const f32 t = ds_ShapeRaycastParameter(pipeline, shape, ray);
	if (t == F32_INFINITY) return 0;

	Vec3Copy(intersection, ray->origin);
	Vec3TranslateScaled(intersection, ray->dir, t);
	return 1;
}
