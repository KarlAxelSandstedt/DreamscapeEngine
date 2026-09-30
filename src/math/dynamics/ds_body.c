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
#include "ds_math_bridge.h"

POOL_DEFINE(ds_Body);
SDB_DEFINE(ds_BodyPrefab);

ds_BodyId ds_BodyAdd(struct ds_Dynamics *pipeline, const struct ds_BodyPrefab *prefab, const ds_Transform *world, const u32 entity)
{
    const u32 old_max = pipeline->body_pool.count_max;
	const struct slot body_slot = ds_BodyPoolAdd(&pipeline->body_pool);
	struct ds_Body *body = body_slot.address;

    if (old_max != pipeline->body_pool.count_max)
    {
        body->id = ds_IdConstruct(body_slot.index, 0);
    }
    body->id += DS_ID_GENERATION_INCREMENT;
	PhysicsEventBodyNew(pipeline, body->id);

    if (pipeline->body_usage_set.bit_count <= body_slot.index)
    {
        ds_BitSetIncreaseSize(&pipeline->body_usage_set, pipeline->body_usage_set.bit_count << 1, 0);
    }
    ds_BitSetSet(&pipeline->body_usage_set, body_slot.index, 1);

    ds_DLLFlush(body->joint_list);
    ds_DLLFlush(body->shape_list);

	body->entity = entity;

	const u32 dynamic_flag = (prefab->dynamic) ? BODY_FLAG_DYNAMIC : 0;
	body->flags = dynamic_flag;

	body->low_velocity_time = 0.0f;

    struct ds_BodySim *sim;
	if (ds_BodyDynamicCheck(body))
	{
        struct ds_SolverSet *active_set = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE; 
        const struct slot sim_slot = ds_CPoolPush(active_set->body_sim_pool);
        const struct slot compute_slot = ds_CPoolPush(active_set->body_compute_pool);

        body->set = SOLVER_SET_ACTIVE;
        body->sim = sim_slot.index;

        sim = sim_slot.address;
        sim->flags = body->flags;

        struct ds_BodyCompute *compute = compute_slot.address;
        compute->flags = body->flags;
	    compute->linear_velocity = V3Zero();
	    compute->angular_velocity = V3Zero();

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
		body->island = POOL_NULL;

        sim = sim_slot.address;
        sim->flags = body->flags;
	}

    sim->body = body_slot.index;
    sim->flags = body->flags;
    sim->world = *world;
    sim->inv_mass = 0.0f;
    sim->local_center_of_mass = V3Zero();
    sim->local_inv_inertia = M3Identity();

	return body->id;
}

void ds_BodyRemove(struct arena *mem_tmp, struct ds_Dynamics *pipeline, const ds_BodyId id)
{
    const u32 body_index = ds_IdIndex(id);
	struct ds_Body *body = pipeline->body_pool.buf + body_index;
    if (body->id != id)
    {
        return;
    }
    ds_Assert(ds_BitSetGet(&pipeline->body_usage_set, body_index));

    const u32 mass_properties_update = 0;

    ds_SolverSetWakeUp(pipeline, body->set);

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
			ds_ShapeStaticRemove(mem_tmp, pipeline, body, body->shape_list.first);
        }

        while (body->joint_list.count)
        {
            ds_JointStaticRemove(pipeline, body, body->joint_list.first);
        }
	}

    const u32 entity = body->entity;
    struct ds_SolverSet *set = pipeline->solver_set_pool.buf + body->set;
    ds_CPoolRemoveAndSwap(set->body_sim_pool, body->sim);
    if (body->sim < set->body_sim_pool.count)
    {
        const struct ds_BodySim *moved_sim = set->body_sim_pool.buf + body->sim;
        struct ds_Body *moved_body = pipeline->body_pool.buf + moved_sim->body;
        ds_Assert(moved_body->set == body->set);
        ds_Assert(moved_body->sim == set->body_sim_pool.count);
        moved_body->sim = body->sim;
    }

    if (body->set == SOLVER_SET_ACTIVE)
    {
        ds_CPoolRemoveAndSwap(set->body_compute_pool, body->sim);
    }
	ds_BodyPoolRemove(&pipeline->body_pool, ds_IdIndex(id));
	PhysicsEventBodyRemoved(pipeline, entity);
}

struct slot ds_BodyLookup(const struct ds_Dynamics *pipeline, const ds_BodyId id)
{
    struct slot slot = { .address = NULL, .index = U32_MAX };
    struct ds_Body *body = pipeline->body_pool.buf + ds_IdIndex(id);
    if (ds_PoolSlotAllocated(body) && body->id == id)
    {
        slot.address = body;
        slot.index = ds_IdIndex(id);
    }

    return slot;
}

void ds_BodyUpdateLocalFrame(struct ds_Dynamics *pipeline, const u32 body, const ds_Transform t_apply_to_local)
{
	//TODO
	ds_Assert(0);
}

void ds_BodyUpdateMassProperties(struct ds_Dynamics *pipeline, const ds_BodyId id)
{
	struct ds_Body *body = pipeline->body_pool.buf + ds_IdIndex(id);
    if (!ds_PoolSlotAllocated(body) || id != body->id)
    {
        return;
    }

    struct arena *tmp = ArenaPushScratch();

    struct ds_SolverSet *set = pipeline->solver_set_pool.buf + body->set;
    struct ds_BodySim *sim = set->body_sim_pool.buf + body->sim;
	ds_Assert(ds_PoolSlotAllocated(body));

	body->mass = 0.0f;
    sim->local_center_of_mass = V3Zero();
	m3 body_inertia_tensor = M3(0.0f, 0.0f, 0.0f,
				    0.0f, 0.0f, 0.0f,
				    0.0f, 0.0f, 0.0f);

	f32 *mass = ArenaPush(tmp, body->shape_list.count*sizeof(f32));
	v3 *center_of_mass = ArenaPush(tmp, body->shape_list.count*sizeof(v3));
	m3 *inertia_tensor = ArenaPush(tmp, body->shape_list.count*sizeof(m3));

	struct ds_Shape *shape = NULL;
	u32 s = body->shape_list.first;
	for (u32 i = 0; i < body->shape_list.count; ++i)
	{
		shape = pipeline->shape_pool.buf + s;
		s = shape->body_shape.next;
		const struct c_Shape *cshape = pipeline->cshape_db->pool.buf + shape->cshape_handle;

		mass[i] = shape->density * cshape->volume;
		body->mass += mass[i];

		/* R, R^-1 */
		const m3 rot_local = M3Q(QLoad(shape->t_local.rotation));
		const m3 rot_local_inv = M3Transpose(rot_local);

		/* center_of_mass_Shape[i] = R*shape_center_of_mass + pos */
		center_of_mass[i] = V3Add(M3V3Mul(rot_local, V3Load(cshape->center_of_mass)), V3Load(shape->t_local.position));
		sim->local_center_of_mass = V3AddScaled(sim->local_center_of_mass, center_of_mass[i], mass[i]);

		/* I_Shape(i) = R * Shape_Inertia * R^-1 */
		const m3 shape_inertia = M3Scale(M3Load(*((mat3ptr) &cshape->inertia_tensor)), shape->density);
		inertia_tensor[i] = M3Mul(M3Mul(rot_local, shape_inertia), rot_local_inv);
	}

    sim->inv_mass = 1.0f / body->mass;
	sim->local_center_of_mass = V3Scale(sim->local_center_of_mass, sim->inv_mass);

	/*
	 * d(i) = center_of_mass_Shape(i) - center_of_mass_Body
	 * I_Body = sum { I_Shape(i) + mass_Shape(i) * (Identity*DOT(d(i),d(i) - OUTER(d(i),d(i)))) }
	 */
	for (u32 i = 0; i < body->shape_list.count; ++i)
	{
		const v3 d = V3Sub(center_of_mass[i], sim->local_center_of_mass);

		const m3 tmp1 = M3Scale(M3Identity(), mass[i]*V3Dot(d, d));
		const m3 tmp2 = M3Scale(M3OuterProduct(d, d), mass[i]);

		body_inertia_tensor = M3Add(body_inertia_tensor, inertia_tensor[i]);
		body_inertia_tensor = M3Add(body_inertia_tensor, tmp1);
		body_inertia_tensor = M3Sub(body_inertia_tensor, tmp2);
	}
    M3Inverse(&sim->local_inv_inertia, body_inertia_tensor);
    
    ArenaPopScratch();
}
