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

#include <stdlib.h>
#include <string.h>

#include "dynamics.h"

POOL_DEFINE(ds_SolverSet);

struct slot ds_SolverSetAdd(struct ds_RigidBodyPipeline *pipeline, const u32 initial_contact_count, const u32 initial_joint_count, const u32 initial_island_count)
{
    struct slot slot = ds_SolverSetPoolAdd(&pipeline->solver_set_pool);
    struct ds_SolverSet *set = slot.address;

    memset(&set->contact_pool, 0, sizeof(set->contact_pool));
    if (initial_contact_count)
    {
        ds_CPoolAlloc(NULL, set->contact_pool, initial_contact_count, GROWABLE);
    }

    memset(&set->joint_sim_pool, 0, sizeof(set->joint_sim_pool));
    if (initial_joint_count)
    {
        ds_CPoolAlloc(NULL, set->joint_sim_pool, initial_joint_count, GROWABLE);
    }

    memset(&set->island_pool, 0, sizeof(set->island_pool));
    if (initial_island_count)
    {
        ds_CPoolAlloc(NULL, set->island_pool, initial_island_count, GROWABLE);
    }

    return slot;
}

void ds_SolverSetRemove(struct ds_RigidBodyPipeline *pipeline, const u32 index)
{
    struct ds_SolverSet *set = pipeline->solver_set_pool.buf + index;
    ds_Assert(ds_PoolSlotAllocated(set));
    ds_Assert(0 == set->contact_pool.count);
    ds_Assert(0 == set->joint_sim_pool.count);
    ds_Assert(0 == set->island_pool.count);

    if (set->contact_pool.buf)
    {
        ds_CPoolDealloc(set->contact_pool);
    }

    if (set->joint_sim_pool.buf)
    {
        ds_CPoolDealloc(set->joint_sim_pool);
    }

    if (set->island_pool.buf)
    {
        ds_CPoolDealloc(set->island_pool);
    }

    ds_SolverSetPoolRemove(&pipeline->solver_set_pool, index);
}

void ds_SolverSetFlush(struct ds_RigidBodyPipeline *pipeline, const u32 index)
{
    struct ds_SolverSet *set = pipeline->solver_set_pool.buf + index;
    ds_Assert(ds_PoolSlotAllocated(set));

    if (set->contact_pool.buf)
    {
        ds_CPoolFlush(set->contact_pool);
    }

    if (set->joint_sim_pool.buf)
    {
        ds_CPoolFlush(set->joint_sim_pool);
    }

    if (set->island_pool.buf)
    {
        ds_CPoolFlush(set->island_pool);
    }
}

void ds_SolverSetWakeUp(struct ds_RigidBodyPipeline *pipeline, const u32 index)
{
    if (index < SOLVER_SET_SLEEPING_FIRST)
    {
        return;
    }

    struct ds_SolverSet *set = pipeline->solver_set_pool.buf + index;
    ds_Assert(ds_PoolSlotAllocated(set));

    //TODO
}

void ds_SolverSetTrySleep(struct ds_RigidBodyPipeline *pipeline, const u32 island_index)
{
    struct ds_Island *island = ds_PoolAddress(&pipeline->is_db.island_pool, island_index);
    if (island->set != SOLVER_SET_ACTIVE)
    {
        return;
    }

    struct ds_SolverSet *set = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    ds_Assert(island->set_island_index < set->island_pool.count);

    //TODO try sleep bla bla bla
}

void ds_SolverSetMerge(struct ds_RigidBodyPipeline *pipeline, const u32 set_expand, const u32 set_merge)
{
    struct ds_SolverSet *expand = pipeline->solver_set_pool.buf + set_expand;
    struct ds_SolverSet *merge = pipeline->solver_set_pool.buf + set_merge;
    ds_Assert(set_expand != set_merge);
    ds_Assert(set_merge >= SOLVER_SET_SLEEPING_FIRST);
    ds_Assert(ds_PoolSlotAllocated(expand));
    ds_Assert(ds_PoolSlotAllocated(merge));

    //TODO  merge bla bla
}

void ds_SolverSetValidate(const struct ds_RigidBodyPipeline *pipeline, const u32 set_index)
{
    const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + set_index;
    ds_Assert(ds_PoolSlotAllocated(set));
    for (u32 i = 0; i < set->island_pool.count; ++i)
    {
        const struct ds_Island *island = ds_PoolAddress(&pipeline->is_db.island_pool, set->island_pool.buf[i]);
        ds_Assert(island->set == set_index);
        ds_Assert(island->set_island_index == i);
    }

    for (u32 i = 0; i < set->joint_sim_pool.count; ++i)
    {
        const struct ds_JointSim *sim = set->joint_sim_pool.buf + i;
        const struct ds_Joint *joint = pipeline->joint_pool.buf + sim->joint;
        ds_Assert(joint->set == set_index);
        ds_Assert(joint->sim == i);
    }

    for (u32 i = 0; i < set->contact_pool.count; ++i)
    {
        const struct ds_Contact *c = nll_Address(&pipeline->cdb->contact_net, set->contact_pool.buf[i]);
        ds_Assert(c->set == set_index);
        ds_Assert(c->set_contact_index == i);
    }
}
