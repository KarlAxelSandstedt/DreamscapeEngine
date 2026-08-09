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

POOL_DEFINE(ds_RigidBody);

ds_RigidBodyId ds_RigidBodyAdd(struct ds_RigidBodyPipeline *pipeline, const struct ds_RigidBodyPrefab *prefab, const ds_Transform *t_world, const u32 entity)
{
	const struct slot body_slot = ds_RigidBodyPoolAdd(&pipeline->body_pool);
	struct ds_RigidBody *body = body_slot.address;
    body->tag += DS_ID_TAG_GENERATION_INCREMENT;
    const ds_RigidBodyId id = ((u64) body->tag << 32) | body_slot.index;
	PhysicsEventBodyNew(pipeline, id);

    if (pipeline->body_usage_set.bit_count <= body_slot.index)
    {
        ds_BitSetIncreaseSize(&pipeline->body_usage_set, pipeline->body_usage_set.bit_count << 1, 0);
    }
    ds_BitSetSet(&pipeline->body_usage_set, body_slot.index, 1);

    ds_DLLFlush(&body->joint_list);
    ds_DLLFlush(&body->shape_list);
    body->t_world = *t_world;

	body->entity = entity;
	Vec3Set(body->velocity, 0.0f, 0.0f, 0.0f);
	Vec3Set(body->angular_velocity, 0.0f, 0.0f, 0.0f);
	Vec3Set(body->linear_momentum, 0.0f, 0.0f, 0.0f);

	const u32 dynamic_flag = (prefab->dynamic) ? RB_DYNAMIC : 0;
	body->flags = dynamic_flag;

	body->low_velocity_time = 0.0f;

	if (body->flags & RB_DYNAMIC)
	{
        struct ds_SolverSet *active_set = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE; 
        const struct slot sim_slot = ds_CPoolPush(active_set->body_sim_pool);

        body->set = SOLVER_SET_ACTIVE;
        body->sim = sim_slot.index;

        struct ds_RigidBodySim *sim = sim_slot.address;
        sim->body = body_slot.index;

	    const struct slot island_slot = ds_IslandAlloc(pipeline, SOLVER_SET_ACTIVE);
	    struct ds_Island *island = island_slot.address;
	    body->island = island_slot.index;
	    ds_DLLAppend(island->body_list, pipeline->body_pool.buf, body_slot.index, island_body);
	}
	else
	{
        struct ds_SolverSet *static_set = pipeline->solver_set_pool.buf + SOLVER_SET_STATIC; 
        const struct slot sim_slot = ds_CPoolPush(static_set->body_sim_pool);

        body->set = SOLVER_SET_STATIC;
        body->sim = sim_slot.index;
		body->island = ISLAND_STATIC;

        struct ds_RigidBodySim *sim = sim_slot.address;
        sim->body = body_slot.index;
	}
	
	return id;
}

void ds_RigidBodyRemove(struct arena *mem_tmp, struct ds_RigidBodyPipeline *pipeline, const ds_RigidBodyId id)
{
    const u32 body_index = ds_IdIndex(id);
	struct ds_RigidBody *body = pipeline->body_pool.buf + body_index;
    if (body->tag != ds_IdTag(id))
    {
        return;
    }
    ds_Assert(ds_BitSetGet(&pipeline->body_usage_set, body_index));

    ds_BitSetSet(&pipeline->body_usage_set, body_index, 0);

    const u32 mass_properties_update = 0;

    struct ds_SolverSet *set = pipeline->solver_set_pool.buf + body->set;
    ds_CPoolRemoveAndSwap(set->body_sim_pool, body->sim);
    if (body->sim < set->body_sim_pool.count)
    {
        const struct ds_RigidBodySim *moved_sim = set->body_sim_pool.buf + body->sim;
        struct ds_RigidBody *moved_body = pipeline->body_pool.buf + moved_sim->body;
        ds_Assert(moved_body->set == body->set);
        ds_Assert(moved_body->sim == set->body_sim_pool.count);
        moved_body->sim = body->sim;
    }

	struct ds_Shape *shape_ptr;
	if (body->set != SOLVER_SET_STATIC)
	{
	    struct ds_Island *island = pipeline->island_pool.buf + body->island;
        ds_Assert(ds_PoolSlotAllocated(island));

        while (body->shape_list.count)
        {
			ds_ShapeDynamicRemove(pipeline, body, body->shape_list.first, mass_properties_update);
        }

        while (body->joint_list.count)
        {
            ds_JointDynamicRemove(pipeline, body, body->joint_list.first);
        }

    	ds_DLLRemove(island->body_list, pipeline->body_pool.buf, ds_IdIndex(id), island_body); 
    	if (island->body_list.count == 0)
    	{
    		ds_Assert(island->body_list.first == DLL_SENTINEL);
    		ds_Assert(island->body_list.last == DLL_SENTINEL);
    		ds_IslandRemove(pipeline, body->island);
    	} 
	}       
	else
	{
        while (body->shape_list.count)
        {
			ds_ShapeStaticRemove(mem_tmp, pipeline, body, body->shape_list.first, mass_properties_update);
        }

        while (body->joint_list.count)
        {
            ds_JointStaticRemove(pipeline, body, body->joint_list.first);
        }
	}

    const u32 entity = body->entity;
	ds_RigidBodyPoolRemove(&pipeline->body_pool, ds_IdIndex(id));
	PhysicsEventBodyRemoved(pipeline, entity);
}

struct slot ds_RigidBodyLookup(const struct ds_RigidBodyPipeline *pipeline, const ds_RigidBodyId id)
{
    struct slot slot = { .address = NULL, .index = 0 };
    struct ds_RigidBody *body = pipeline->body_pool.buf + ds_IdIndex(id);
    if (id != DS_ID_NULL && ds_PoolSlotAllocated(body) && body->tag == ds_IdTag(id))
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

	struct ds_RigidBody *body = pipeline->body_pool.buf + ds_IdIndex(id);
	ds_Assert(ds_PoolSlotAllocated(body));

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
		shape = pipeline->shape_pool.buf + s;
		s = shape->body_shape.next;
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
