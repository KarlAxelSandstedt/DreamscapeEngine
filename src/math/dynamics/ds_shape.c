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

struct slot ds_ShapeAdd(struct ds_RigidBodyPipeline *pipeline, const struct ds_ShapePrefab *prefab, const ds_Transform *t, const u32 body)
{
	struct slot slot = PoolAdd(&pipeline->shape_pool);
	if (slot.address)
	{
		struct ds_RigidBody *body_ptr = PoolAddress(&pipeline->body_pool, body);
		ds_Assert(PoolSlotAllocated(body_ptr));
		dll_Append(&body_ptr->shape_list, pipeline->shape_pool.buf, slot.index);

		struct ds_Shape *shape = slot.address;
		shape->body = body;
		shape->contact_first = NLL_NULL;
		shape->density = prefab->density;
		shape->restitution = prefab->restitution;
		shape->friction = prefab->friction;
		shape->t_local = *t;
		shape->margin = prefab->margin;

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
	}

	return slot;
}

void ds_ShapeDynamicRemove(struct ds_RigidBodyPipeline *pipeline, const u32 shape_index)
{
	struct ds_Shape *shape = PoolAddress(&pipeline->shape_pool, shape_index);
	ds_Assert(PoolSlotAllocated(shape));

	//TODO Island is per-body specific, so it should not be here
	//TODO Contacts is per shape specific, makes sense
	//basically, we should separate and simplfiy the state management here.
	
	//isdb_IslandRemoveBodyResources(pipeline, body->island_index, shape_index);
	//cdb_BodyRemoveContacts(pipeline, shape_index);
	//const struct island *is = PoolAddress(&pipeline->is_db.island_pool, body->island_index);
	//if (PoolSlotAllocated(is) && is->contact_list.count > 0)
	//{
	//	isdb_SplitIsland(&pipeline->frame, pipeline, body->island_index);
	//}

	strdb_Dereference(pipeline->cshape_db, shape->cshape_handle);
	DbvhRemove(&pipeline->shape_bvh, shape->proxy);
	PoolRemove(&pipeline->shape_pool, shape_index);
}

void ds_ShapeStaticRemove(struct ds_RigidBodyPipeline *pipeline, const u32 shape_index)
{
	struct ds_Shape *shape = PoolAddress(&pipeline->shape_pool, shape_index);
	ds_Assert(PoolSlotAllocated(shape));

	//TODO
	//cdb_StaticRemoveContactsAndUpdateIslands(pipeline, shape_index);	

	strdb_Dereference(pipeline->cshape_db, shape->cshape_handle);
	DbvhRemove(&pipeline->shape_bvh, shape->proxy);
	PoolRemove(&pipeline->shape_pool, shape_index);
}

void ds_ShapeWorldTransform(ds_Transform *t, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *shape)
{
	const struct ds_RigidBody *body = PoolAddress(&pipeline->body_pool, shape->body);
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

	const struct ds_RigidBody *body = PoolAddress(&pipeline->body_pool, shape->body);
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

u32 (*c_shape_tests[C_SHAPE_COUNT][C_SHAPE_COUNT])(const struct c_Shape *, const ds_Transform *, const struct c_Shape *, const ds_Transform *, const f32 margin) =
{
	{ c_SphereTest, 		    0, 				            0, 			            0, },
	{ c_CapsuleSphereTest,	    c_CapsuleTest, 			    0, 			            0, },
	{ c_HullSphereTest, 		c_HullCapsuleTest,		    c_HullTest,		        0, },
	{ c_TriMeshBvhSphereTest,   c_TriMeshBvhCapsuleTest,    c_TriMeshBvhHullTest,	0, },
};

f32 (*c_distance_methods[C_SHAPE_COUNT][C_SHAPE_COUNT])(vec3 c1, vec3 c2, const struct c_Shape *, const ds_Transform *, const struct c_Shape *, const ds_Transform *, const f32) =
{
	{ c_SphereDistance,	 	        0,				                0, 			                0, },
	{ c_CapsuleSphereDistance,	    c_CapsuleDistance, 		        0, 			                0, },
	{ c_HullSphereDistance, 		c_HullCapsuleDistance, 		    c_HullDistance,		        0, },
	{ c_TriMeshBvhSphereDistance,	c_TriMeshBvhCapsuleDistance, 	c_TriMeshBvhHullDistance,	0, },
};

u32 (*c_contact_methods[C_SHAPE_COUNT][C_SHAPE_COUNT])(struct arena *, struct c_Manifold *, struct sat_Cache *, const struct sat_Cache *, const struct c_Shape *, const ds_Transform *, const struct c_Shape *, const ds_Transform *, const f32) =
{
	{ c_SphereContact,	 	        0, 				            0,			                0, },
	{ c_CapsuleSphereContact, 	    c_CapsuleContact,			0,			                0, },
	{ c_HullSphereContact, 	  	    c_HullCapsuleContact,		c_HullContact, 		        0, },
	{ c_TriMeshBvhSphereContact,	c_TriMeshBvhCapsuleContact, c_TriMeshBvhHullContact,    0, },
};

f32 (*c_raycast_parameter_methods[C_SHAPE_COUNT])(struct arena *, const struct c_Shape *, const ds_Transform *, const struct ray *) =
{
	c_SphereRaycastParameter,
	c_CapsuleRaycastParameter,
	c_HullRaycastParameter,
	c_TriMeshBvhRaycastParameter,
};

u32 ds_ShapeTest(const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *s1, const struct ds_Shape *s2, const f32 margin)
{
	ds_Assert(margin >= 0.0f);

 	const struct c_Shape *c_s1 = strdb_Address(pipeline->cshape_db, s1->cshape_handle);
	const struct c_Shape *c_s2 = strdb_Address(pipeline->cshape_db, s2->cshape_handle);

    ds_Transform t1, t2;
    ds_ShapeWorldTransform(&t1, pipeline, s1);
    ds_ShapeWorldTransform(&t2, pipeline, s2);
	
	return (c_s1->type >= c_s2->type)  
		? c_shape_tests[c_s1->type][c_s2->type](c_s1, &t1, c_s2, &t2, margin)
		: c_shape_tests[c_s2->type][c_s1->type](c_s2, &t2, c_s1, &t1, margin);
}

f32 ds_ShapeDistance(vec3 c1, vec3 c2, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *s1, const struct ds_Shape *s2, const f32 margin)
{
	ds_Assert(margin >= 0.0f);
	
 	const struct c_Shape *c_s1 = strdb_Address(pipeline->cshape_db, s1->cshape_handle);
	const struct c_Shape *c_s2 = strdb_Address(pipeline->cshape_db, s2->cshape_handle);

    ds_Transform t1, t2;
    ds_ShapeWorldTransform(&t1, pipeline, s1);
    ds_ShapeWorldTransform(&t2, pipeline, s2);

	return (c_s1->type >= c_s2->type)  
		? c_distance_methods[c_s1->type][c_s2->type](c1, c2, c_s1, &t1, c_s2, &t2, margin)
		: c_distance_methods[c_s2->type][c_s1->type](c2, c1, c_s2, &t2, c_s1, &t1, margin);
}

u32 ds_ShapeContact(struct arena *tmp, struct c_Manifold *manifold, struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *s1, const struct ds_Shape *s2, const f32 margin)
{
	ds_Assert(margin >= 0.0f);

    ds_Transform t1, t2;
    ds_ShapeWorldTransform(&t1, pipeline, s1);
    ds_ShapeWorldTransform(&t2, pipeline, s2);

    const struct c_Shape *c_s1 = strdb_Address(pipeline->cshape_db, s1->cshape_handle);
    const struct c_Shape *c_s2 = strdb_Address(pipeline->cshape_db, s2->cshape_handle);

    struct sat_Cache *cache = NULL;
    struct sat_Cache *cache_copy = NULL;
    struct sat_Cache cache_copy_mem;
    if (c_s1->type == C_SHAPE_CONVEX_HULL && c_s2->type == C_SHAPE_CONVEX_HULL)
    {
        const struct ds_ContactKey key = ds_ContactKeyCanonical(
                s1->body, 
                PoolIndex(&pipeline->shape_pool, s1), 
                s2->body, 
                PoolIndex(&pipeline->shape_pool, s2));
        cache = sat_CacheLookup(&pipeline->cdb, &key).address;
        if (cache)
        {
            cache->touched = 1;
            cache_copy = &cache_copy_mem;
            *cache_copy = *cache;
        }
        else
        {
            cache = sat_CacheAdd(&pipeline->cdb, &key).address;
        }
    }

	u32 collision;
	if (c_s1->type >= c_s2->type) 
	{
		collision = c_contact_methods[c_s1->type][c_s2->type](tmp, manifold, cache, cache_copy, c_s1, &t1, c_s2, &t2, margin);
	}                                                                                         
	else                                                                                      
	{                                                                                         
		collision = c_contact_methods[c_s2->type][c_s1->type](tmp, manifold, cache, cache_copy, c_s2, &t2, c_s1, &t1, margin);
		Vec3ScaleSelf(manifold->n, -1.0f);
	}

	return collision;
}

f32 ds_ShapeRaycastParameter(struct arena *tmp, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *shape, const struct ray *ray)
{
    ds_Transform transform;
    ds_ShapeWorldTransform(&transform, pipeline, shape);
    const struct c_Shape *c_shape = strdb_Address(pipeline->cshape_db, shape->cshape_handle);

	return c_raycast_parameter_methods[c_shape->type](tmp, c_shape, &transform, ray);
}

u32 ds_ShapeRaycast(struct arena *tmp, vec3 intersection, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Shape *shape, const struct ray *ray)
{
	const f32 t = ds_ShapeRaycastParameter(tmp, pipeline, shape, ray);
	if (t == F32_INFINITY) return 0;

	Vec3Copy(intersection, ray->origin);
	Vec3TranslateScaled(intersection, ray->dir, t);
	return 1;
}
