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

/*
 * TODO:
 * Issue:
 * 	We deal with three spaces: World space, Local body space, and the arbitrary body construction space.
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

		const struct slot cshape_slot = strdb_Reference(pipeline->shape_db, prefab->cshape_id);
		const struct collisionShape *cshape = slot.address;
		shape->cshape_handle = cshape_slot.index;
		shape->cshape_type = cshape->type;

		//TODO add to dbvh

		//TODO
		//f32 			mass;		
		//struct aabb		local_box;
	}

	return slot;
}

void ds_ShapeRemove(struct ds_RigidBodyPipeline *pipeline, const u32 shape)
{
/*
 * Remove the specified shape and update the physics state into a valid state.
 * TODO: be careful here, removing a shape in this way affects more or less every subsystem in the pipeline:
 *	
 *	removing contact 
 *			=> update some contact map? 
 *			=> update island contact dll
 *			=> update body contact nll
 *			=> wake up island
 *	removing shape
 *			=> deference collisionShape
 *			=> remove bvh node	
 *			=> update rigid body dll
 */
}

//static void RigidBodyUpdateLocalBox(struct ds_RigidBody *body, const struct collisionShape *shape)
//{
//	vec3 min = { F32_INFINITY, F32_INFINITY, F32_INFINITY };
//	vec3 max = { -F32_INFINITY, -F32_INFINITY, -F32_INFINITY };
//
//	vec3 v;
//	mat3 rot;
//	Mat3Quat(rot, body->rotation);
//
//	if (body->shape_type == COLLISION_SHAPE_CONVEX_HULL)
//	{
//		for (u32 i = 0; i < shape->hull.v_count; ++i)
//		{
//			Mat3VecMul(v, rot, shape->hull.v[i]);
//			min[0] = f32_min(min[0], v[0]); 
//			min[1] = f32_min(min[1], v[1]);			
//			min[2] = f32_min(min[2], v[2]);			
//                                                   
//			max[0] = f32_max(max[0], v[0]);			
//			max[1] = f32_max(max[1], v[1]);			
//			max[2] = f32_max(max[2], v[2]);			
//		}
//	}
//	else if (body->shape_type == COLLISION_SHAPE_SPHERE)
//	{
//		const f32 r = shape->sphere.radius;
//		Vec3Set(min, -r, -r, -r);
//		Vec3Set(max, r, r, r);
//	}
//	else if (body->shape_type == COLLISION_SHAPE_CAPSULE)
//	{
//		v[0] = rot[1][0] * shape->capsule.half_height;	
//		v[1] = rot[1][1] * shape->capsule.half_height;	
//		v[2] = rot[1][2] * shape->capsule.half_height;	
//		Vec3Set(max, 
//			f32_max(-v[0], v[0]),
//			f32_max(-v[1], v[1]),
//			f32_max(-v[2], v[2]));
//		Vec3AddConstant(max, shape->capsule.radius);
//		Vec3Negate(min, max);
//	}
//	else if (body->shape_type == COLLISION_SHAPE_TRI_MESH)
//	{
//		const struct bvhNode *node = (struct bvhNode *) shape->mesh_bvh.bvh.tree.pool.buf;
//		struct aabb bbox; 
//		AabbRotate(&bbox, &node[shape->mesh_bvh.bvh.tree.root].bbox, rot);
//		//Vec3Sub(min, bbox.center, bbox.hw);
//		//Vec3Add(max, bbox.center, bbox.hw);
//		Vec3Scale(min, bbox.hw, -1.0f);
//		Vec3Scale(max, bbox.hw, 1.0f);
//	}
//
//	Vec3Sub(body->local_box.hw, max, min);
//	Vec3ScaleSelf(body->local_box.hw, 0.5f);
//	Vec3Add(body->local_box.center, min, body->local_box.hw);
//}

//struct slot PhysicsPipelineRigidBodyAlloc(struct ds_RigidBodyPipeline *pipeline, struct ds_RigidBodyPrefab *prefab, const vec3 position, const quat rotation, const u32 entity)
//{
//	RigidBodyUpdateLocalBox(body, shape);
//	struct aabb proxy;
//	Vec3Add(proxy.center, body->local_box.center, body->position);
//	if (body->shape_type == COLLISION_SHAPE_TRI_MESH)
//	{
//		Vec3Set(proxy.hw, 
//			body->local_box.hw[0],
//			body->local_box.hw[1],
//			body->local_box.hw[2]);
//	}
//	else
//	{
//		Vec3Set(proxy.hw, 
//			body->local_box.hw[0] + body->margin,
//			body->local_box.hw[1] + body->margin,
//			body->local_box.hw[2] + body->margin);
//	}
//	body->proxy = DbvhInsert(&pipeline->dynamic_tree, slot.index, &proxy);
//}
