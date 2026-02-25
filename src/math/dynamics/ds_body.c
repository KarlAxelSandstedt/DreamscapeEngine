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

struct slot ds_RigidBodyAdd(struct ds_RigidBodyPipeline *pipeline, struct ds_RigidBodyPrefab *prefab, const vec3 position, const quat rotation, const u32 entity)
{
	struct slot slot = PoolAdd(&pipeline->body_pool);
	PhysicsEventBodyNew(pipeline, slot.index);
	struct ds_RigidBody *body = slot.address;
	dll_Append(&pipeline->body_non_marked_list, pipeline->body_pool.buf, slot.index);

	body->shape_list = dll_Init(struct ds_Shape);
	QuatCopy(body->t_world.rotation, rotation);
	Vec3Copy(body->t_world.position, position);

	body->entity = entity;
	Vec3Copy(body->position, position);
	QuatCopy(body->rotation, rotation);
	Vec3Set(body->velocity, 0.0f, 0.0f, 0.0f);
	Vec3Set(body->angular_velocity, 0.0f, 0.0f, 0.0f);
	Vec3Set(body->linear_momentum, 0.0f, 0.0f, 0.0f);

	const u32 dynamic_flag = (prefab->dynamic) ? RB_DYNAMIC : 0;
	body->flags = RB_ACTIVE | (g_solver_config->sleep_enabled * RB_AWAKE) | dynamic_flag;

	Mat3Copy(body->inertia_tensor, prefab->inertia_tensor);
	Mat3Copy(body->inv_inertia_tensor, prefab->inv_inertia_tensor);
	body->mass = prefab->mass;
	body->restitution = prefab->restitution;
	body->friction = prefab->friction;
	body->low_velocity_time = 0.0f;

	if (body->flags & RB_DYNAMIC)
	{
		isdb_InitIslandFromBody(pipeline, slot.index);
	}
	else
	{
		body->island_index = ISLAND_STATIC;
	}
	
	return slot;
}

void ds_RigidBodyRemove(struct ds_RigidBodyPipeline *pipeline, const u32 handle)
{
	struct ds_RigidBody *body = PoolAddress(&pipeline->body_pool, handle);
	ds_Assert(PoolSlotAllocated(body));

	struct ds_Shape *shape_ptr;
	if (body->island_index != ISLAND_STATIC)
	{
		for (u32 shape = body->shape_list.first; shape != DLL_NULL; shape = shape_ptr->dll_next)
		{
			shape_ptr = PoolAddress(&pipeline->shape_pool, shape);
			ds_ShapeDynamicRemove(pipeline, shape);
		}
	}
	else
	{
		for (u32 shape = body->shape_list.first; shape != DLL_NULL; shape = shape_ptr->dll_next)
		{
			shape_ptr = PoolAddress(&pipeline->shape_pool, shape);
			ds_ShapeStaticRemove(pipeline, shape);
		}
	}
	PoolRemove(&pipeline->body_pool, handle);
	PhysicsEventBodyRemoved(pipeline, handle);
}

void ds_RigidBodyUpdateLocalFrame(struct ds_RigidBodyPipeline *pipeline, const u32 body, const ds_Transform t_apply_to_local)
{
	//TODO
	ds_Assert(0);
}

void ds_RigidBodyUpdateMassProperties(struct ds_RigidBodyPipeline *pipeline, const u32 body_index)
{
	ArenaPushRecord(&pipeline->frame);

	struct ds_RigidBody *body = PoolAddress(&pipeline->body_pool, body_index);
	ds_Assert(PoolSlotAllocated(body));

	vec3 tmp;
	mat3 rot_world, rot_local, rot_local_inv, tmp1, tmp2, tmp3;
	Mat3Quat(rot_world, body->t_world.rotation);

	body->mass = 0.0f;
	Vec3Set(body->local_center_of_mass, 0.0f, 0.0f, 0.0f);
	Mat3Set(body->inertia_tensor, 
			0.0f, 0.0f, 0.0f, 
			0.0f, 0.0f, 0.0f, 
			0.0f, 0.0f, 0.0f);

	f32 *mass = ArenaPush(&pipeline->frame, body->shape_list.count*sizeof(f32));
	vec3ptr center_of_mass = ArenaPush(&pipeline->frame, body->shape_list.count*sizeof(vec3));
	mat3ptr inertia_tensor = ArenaPush(&pipeline->frame, body->shape_list.count*sizeof(mat3));

	struct ds_Shape *shape = NULL;
	u32 s = body->shape_list.first;
	for (u32 i = 0; i < body->shape_list.count; ++i)
	{
		shape = PoolAddress(&pipeline->shape_pool, s);
		s = shape->dll_next;
		const struct c_Shape *cshape = strdb_Address(pipeline->cshape_db, shape->cshape_handle);

		mass[i] = shape->density * cshape->volume;
		body->mass += mass[i];

		/* R, R^-1*/
		Mat3Quat(rot_local, shape->t_local.rotation);
		Mat3Transpose(rot_local_inv, rot_local);

		/* center_of_mass_Shape[i] = R*shape_center_of_mass + pos */
		Vec3Copy(tmp, cshape->center_of_mass);
		Mat3VecMul(center_of_mass[i], rot_local, tmp);
		Vec3Translate(center_of_mass[i], shape->t_local.position);
		Vec3TranslateScaled(body->local_center_of_mass, center_of_mass[i], mass[i]);

		/* I_Shape(i) = R * Shape_Inertia * R^-1 */
		Mat3Scale(tmp1, *((mat3ptr) &cshape->inertia_tensor), shape->density);
		Mat3Mul(tmp2, rot_local, tmp1);
		Mat3Mul(inertia_tensor[i], tmp2, rot_local_inv);
	}

	Vec3ScaleSelf(body->local_center_of_mass, 1.0f / body->mass);

	/* 
	 * d(i) = center_of_mass_Shape(i) - center_of_mass_Body
	 * I_Body = sum { I_Shape(i) + mass_Shape(i) * (Identity*DOT(d(i),d(i) - OUTER(d(i),d(i)))) } 
	 */
	vec3 d;
	for (u32 i = 0; i < body->shape_list.count; ++i)
	{
		Vec3Sub(d, center_of_mass[i], body->local_center_of_mass);

		Mat3Identity(tmp1);
		Mat3ScaleSelf(tmp1, mass[i]*Vec3Dot(d, d));

		Mat3OuterProduct(tmp2, d, d);
		Mat3ScaleSelf(tmp2, mass[i]);

		Mat3AddSelf(body->inertia_tensor, inertia_tensor[i]);
		Mat3AddSelf(body->inertia_tensor, tmp1);
		Mat3SubSelf(body->inertia_tensor, tmp2);
	}

	ArenaPopRecord(&pipeline->frame);
}
