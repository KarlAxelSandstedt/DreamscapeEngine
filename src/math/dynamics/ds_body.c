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

ds_RigidBodyId ds_RigidBodyAdd(struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBodyPrefab *prefab, const ds_Transform *t_world, const u32 entity)
{
	struct slot slot = ds_PoolAdd(&pipeline->body_pool);
	struct ds_RigidBody *body = slot.address;
    body->tag += DS_ID_TAG_GENERATION_INCREMENT;
    const ds_RigidBodyId id = ((u64) body->tag << 32) | slot.index;

	PhysicsEventBodyNew(pipeline, id);
	dll_Append(&pipeline->body_non_marked_list, pipeline->body_pool.buf, slot.index);

	body->shape_list = dll_Init(struct ds_Shape);
    body->t_world = *t_world;

	body->entity = entity;
	Vec3Set(body->velocity, 0.0f, 0.0f, 0.0f);
	Vec3Set(body->angular_velocity, 0.0f, 0.0f, 0.0f);
	Vec3Set(body->linear_momentum, 0.0f, 0.0f, 0.0f);

	const u32 dynamic_flag = (prefab->dynamic) ? RB_DYNAMIC : 0;
	body->flags = RB_ACTIVE | (g_solver_config->sleep_enabled * RB_AWAKE) | dynamic_flag;

	body->low_velocity_time = 0.0f;

	if (body->flags & RB_DYNAMIC)
	{
		isdb_InitIslandFromBody(pipeline, slot.index);
	}
	else
	{
		body->island_index = ISLAND_STATIC;
	}
	
	return id;
}

void ds_RigidBodyRemove(struct arena *mem_tmp, struct ds_RigidBodyPipeline *pipeline, const ds_RigidBodyId id)
{
	struct ds_RigidBody *body = ds_PoolAddress(&pipeline->body_pool, ds_IdIndex(id));
    if (body->tag != ds_IdTag(id))
    {
        return;
    }
	ds_Assert(PoolSlotAllocated(body));

    (RB_IS_MARKED(body))
		? dll_Remove(&pipeline->body_marked_list, pipeline->body_pool.buf, ds_IdIndex(id))
		: dll_Remove(&pipeline->body_non_marked_list, pipeline->body_pool.buf, ds_IdIndex(id));

	struct ds_Shape *shape_ptr;
	if (body->island_index != ISLAND_STATIC)
	{
	    struct ds_Island *island = ds_PoolAddress(&pipeline->is_db.island_pool, body->island_index);
        ds_Assert(PoolSlotAllocated(island));

		for (u32 shape = body->shape_list.first; shape != DLL_NULL;)
		{
			shape_ptr = ds_PoolAddress(&pipeline->shape_pool, shape);
            const u32 next = shape_ptr->dll_next;
			ds_ShapeDynamicRemove(pipeline, island, shape);
            shape = next;
		}

    	dll_Remove(&island->body_list, pipeline->body_pool.buf, ds_IdIndex(id)); 
    	if (island->body_list.count == 0)
    	{
    		ds_Assert(island->body_list.first == DLL_NULL);
    		ds_Assert(island->body_list.last == DLL_NULL);
    		isdb_IslandRemove(pipeline, island);
    	} 
        else if (island->body_list.count > 1) 
        {
	    	isdb_SplitIsland(mem_tmp, pipeline, body->island_index);
	    }
	}       
	else
	{
		for (u32 shape = body->shape_list.first; shape != DLL_NULL;)
		{
			shape_ptr = ds_PoolAddress(&pipeline->shape_pool, shape);
            const u32 next = shape_ptr->dll_next;
			ds_ShapeStaticRemove(mem_tmp, pipeline, shape);
            shape = next;
		}
	}

    const u32 entity = body->entity;
	ds_PoolRemove(&pipeline->body_pool, ds_IdIndex(id));
	PhysicsEventBodyRemoved(pipeline, entity);
}

struct slot ds_RigidBodyLookup(const struct ds_RigidBodyPipeline *pipeline, const ds_RigidBodyId id)
{
    struct slot slot = { .address = NULL, .index = 0 };
    struct ds_RigidBody *body = ds_PoolAddress(&pipeline->body_pool, ds_IdIndex(id));
    if (id != DS_ID_NULL && PoolSlotAllocated(body) && body->tag == ds_IdTag(id))
    {
        slot.address = body;
        slot.index = ds_IdIndex(id);
    }

    return slot;
}

void ds_RigidBodyUpdateLocalFrame(struct ds_RigidBodyPipeline *pipeline, const u32 body, const ds_Transform t_apply_to_local)
{
	//TODO
	ds_Assert(0);
}

void ds_RigidBodyUpdateMassProperties(struct ds_RigidBodyPipeline *pipeline, const ds_RigidBodyId id)
{
	ArenaPushRecord(&pipeline->frame);

	struct ds_RigidBody *body = ds_PoolAddress(&pipeline->body_pool, ds_IdIndex(id));
	ds_Assert(PoolSlotAllocated(body));

	vec3 tmp;
    mat3 body_inertia_tensor, rot_local, rot_local_inv, tmp1, tmp2;

	body->mass = 0.0f;
	Vec3Set(body->local_center_of_mass, 0.0f, 0.0f, 0.0f);
	Mat3Set(body_inertia_tensor, 
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
		shape = ds_PoolAddress(&pipeline->shape_pool, s);
		s = shape->dll_next;
		const struct c_Shape *cshape = strdb_Address(pipeline->cshape_db, shape->cshape_handle);

		mass[i] = shape->density * cshape->volume;
		body->mass += mass[i];

		/* R, R^-1 */
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

		Mat3AddSelf(body_inertia_tensor, inertia_tensor[i]);
		Mat3AddSelf(body_inertia_tensor, tmp1);
		Mat3SubSelf(body_inertia_tensor, tmp2);
	}
    Mat3Inverse(body->inv_inertia_tensor, body_inertia_tensor);

	ArenaPopRecord(&pipeline->frame);
}
