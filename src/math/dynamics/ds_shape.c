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
		dll_Append(&body_ptr->shape_list, &pipeline->shape_pool, slot.index);

		struct ds_Shape *shape = slot.address;
		shape->body = body;
		shape->contact_first = NLL_NULL;
		shape->density = prefab->density;
		shape->restitution = prefab->restitution;
		shape->friction = prefab->friction;
		shape->t_local = *t;
		shape->margin = prefab->margin;

		const struct slot cshape_slot = strdb_Reference(pipeline->shape_db, prefab->cshape_id);
		const struct collisionShape *cshape = slot.address;
		shape->cshape_handle = cshape_slot.index;
		shape->cshape_type = cshape->type;

		struct aabb bbox_proxy = ds_ShapeWorldBbox(pipeline, shape);
		if (shape->cshape_type != COLLISION_SHAPE_TRI_MESH)
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

	strdb_Dereference(pipeline->shape_db, shape->cshape_handle);
	DbvhRemove(&pipeline->shape_bvh, shape->proxy);
	PoolRemove(&pipeline->shape_pool, shape_index);
}

void ds_ShapeStaticRemove(struct ds_RigidBodyPipeline *pipeline, const u32 shape_index)
{
	struct ds_Shape *shape = PoolAddress(&pipeline->shape_pool, shape_index);
	ds_Assert(PoolSlotAllocated(shape));

	//TODO
	//cdb_StaticRemoveContactsAndUpdateIslands(pipeline, shape_index);	

	strdb_Dereference(pipeline->shape_db, shape->cshape_handle);
	DbvhRemove(&pipeline->shape_bvh, shape->proxy);
	PoolRemove(&pipeline->shape_pool, shape_index);
}

struct aabb ds_ShapeWorldBbox(const struct ds_RigidBodyPipeline *pipeline, struct ds_Shape *shape)
{
	vec3 min = { F32_INFINITY, F32_INFINITY, F32_INFINITY };
	vec3 max = { -F32_INFINITY, -F32_INFINITY, -F32_INFINITY };

	const struct ds_RigidBody *body = PoolAddress(&pipeline->body_pool, shape->body);
	const struct collisionShape *cshape = strdb_Address(pipeline->shape_db, shape->cshape_handle);
	mat3 shape_rot, body_rot;
	Mat3Quat(shape_rot, shape->t_local.rotation);
	Mat3Quat(body_rot, body->t_world.rotation);

	vec3 v, tmp;
	if (shape->cshape_type == COLLISION_SHAPE_CONVEX_HULL)
	{
		for (u32 i = 0; i < cshape->hull.v_count; ++i)
		{
			Mat3VecMul(tmp, shape_rot, cshape->hull.v[i]);
			Vec3Translate(tmp, shape->t_local.position);
			Mat3VecMul(v, body_rot, tmp);
			Vec3Translate(v, body->t_world.position);

			min[0] = f32_min(min[0], v[0]); 
			min[1] = f32_min(min[1], v[1]);			
			min[2] = f32_min(min[2], v[2]);			
                                                   
			max[0] = f32_max(max[0], v[0]);			
			max[1] = f32_max(max[1], v[1]);			
			max[2] = f32_max(max[2], v[2]);			
		}
	}
	else if (shape->cshape_type == COLLISION_SHAPE_SPHERE)
	{
		const f32 r = cshape->sphere.radius;
		Vec3Set(min, -r, -r, -r);
		Vec3Set(max, r, r, r);
		Vec3Translate(min, shape->t_local.position);
		Vec3Translate(max, shape->t_local.position);
		Vec3Translate(min, body->t_world.position);
		Vec3Translate(max, body->t_world.position);
	}
	else if (shape->cshape_type == COLLISION_SHAPE_CAPSULE)
	{
		tmp[0] = shape_rot[1][0] * cshape->capsule.half_height;	
		tmp[1] = shape_rot[1][1] * cshape->capsule.half_height;	
		tmp[2] = shape_rot[1][2] * cshape->capsule.half_height;	
		Mat3VecMul(v, body_rot, tmp);

		Vec3Abs(max, v);
		Vec3AddConstant(max, cshape->capsule.radius);
		Vec3Negate(min, max);

		Mat3VecMul(v, body_rot, shape->t_local.position);
		Vec3Translate(v, body->t_world.position);
		Vec3Translate(min, v);
		Vec3Translate(max, v);
	}
	else if (shape->cshape_type == COLLISION_SHAPE_TRI_MESH)
	{
		//TODO "We treat Tri meshes differently; a rigid body who has a tri mesh attached
		// views the tri mesh triangles and its shapes. Thus such a rigid body treats its
		// mesh shape to have position 0 and no rotation.
		const struct bvhNode *node = (struct bvhNode *) cshape->mesh_bvh.bvh.tree.pool.buf;
		struct aabb bbox; 
		AabbRotate(&bbox, &node[cshape->mesh_bvh.bvh.tree.root].bbox, body_rot);
		//Vec3Sub(min, bbox.center, bbox.hw);
		//Vec3Add(max, bbox.center, bbox.hw);
		Vec3Scale(min, bbox.hw, -1.0f);
		Vec3Scale(max, bbox.hw, 1.0f);
		Vec3Translate(min, body->t_world.position);
		Vec3Translate(max, body->t_world.position);
	}

	struct aabb bbox;
	Vec3Sub(bbox.hw, max, min);
	Vec3ScaleSelf(bbox.hw, 0.5f);
	Vec3Add(bbox.center, min, bbox.hw);
	return bbox;
}
