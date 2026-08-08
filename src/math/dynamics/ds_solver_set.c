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

struct slot ds_SolverSetAdd(struct ds_RigidBodyPipeline *pipeline, const u32 initial_body_count, const u32 initial_contact_count, const u32 initial_joint_count, const u32 initial_island_count)
{
    ProfZone;
    struct slot slot = ds_SolverSetPoolAdd(&pipeline->solver_set_pool);
    struct ds_SolverSet *set = slot.address;

    memset(&set->body_sim_pool, 0, sizeof(set->body_sim_pool));
    if (initial_body_count)
    {
        ds_CPoolAlloc(NULL, set->body_sim_pool, initial_body_count, GROWABLE);
    }

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

    ProfZoneEnd;
    return slot;
}

void ds_SolverSetRemove(struct ds_RigidBodyPipeline *pipeline, const u32 index)
{
    struct ds_SolverSet *set = pipeline->solver_set_pool.buf + index;
    ds_Assert(ds_PoolSlotAllocated(set));

    ds_Assert(index < SOLVER_SET_SLEEPING_FIRST || (set->contact_pool.count == 0) || (set->joint_sim_pool.count == 0) || (set->island_pool.count == 1));

    if (set->body_sim_pool.buf)
    {
        ds_CPoolDealloc(set->body_sim_pool);
    }

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

    if (set->body_sim_pool.buf)
    {
        ds_CPoolFlush(set->body_sim_pool);
    }

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

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    struct ds_SolverSet *set = pipeline->solver_set_pool.buf + index;
    ds_Assert(ds_PoolSlotAllocated(set));
    ds_Assert(set->island_pool.count == 1);

    //TODO Wake up island? 
    
    for (u32 i = 0; i < set->contact_pool.count; ++i)
    {
        const u32 ci = set->contact_pool.buf[i];
        struct ds_Contact *c = pipeline->cdb->contact_pool.buf + ci;
        ds_CGraphContactAdd(pipeline, c);
    }

    for (u32 i = 0; i < set->body_sim_pool.count; ++i)
    {
        const struct ds_RigidBodySim *old_sim = set->body_sim_pool.buf + i;
        struct ds_RigidBody *body = pipeline->body_pool.buf + old_sim->body;
        const struct slot slot = ds_CPoolPush(active->body_sim_pool);

        struct ds_RigidBodySim *new_sim = slot.address;
        memcpy(new_sim, old_sim, sizeof(*new_sim));

        body->set = SOLVER_SET_ACTIVE;
        body->sim = slot.index;
    }

    for (u32 i = 0; i < set->island_pool.count; ++i)
    {
        const u32 isi = set->island_pool.buf[i];
        struct ds_Island *island = pipeline->is_db.island_pool.buf + isi;
        island->set = SOLVER_SET_ACTIVE;
        island->set_island_index = ds_CPoolPush(active->island_pool).index; 
        active->island_pool.buf[ island->set_island_index ] = isi;
    }

    ds_SolverSetRemove(pipeline, index);
}

void ds_SolverSetSleep(struct ds_RigidBodyPipeline *pipeline, const u32 island_index)
{
    struct ds_CGraph *cg = &pipeline->cgraph;
    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    struct ds_Island *island = pipeline->is_db.island_pool.buf + island_index;

    ds_Assert(island->set == SOLVER_SET_ACTIVE);
    ds_Assert(island->set_island_index < active->island_pool.count);

    ds_CPoolRemoveAndSwap(active->island_pool, island->set_island_index);
    if (island->set_island_index < active->island_pool.count)
    {
        const u32 moved_index = active->island_pool.buf[ island->set_island_index ];
        struct ds_Island *moved = pipeline->is_db.island_pool.buf + moved_index;
        ds_Assert(moved->set == SOLVER_SET_ACTIVE);
        ds_Assert(moved->set_island_index == active->island_pool.count);
        moved->set_island_index = island->set_island_index;
    }

    struct slot slot = ds_SolverSetAdd(pipeline, island->body_list.count, island->contact_list.count, island->joint_list.count, 1);
    struct ds_SolverSet *set = slot.address;
    island->set = slot.index;
    island->set_island_index = 0;
    ds_CPoolPushValue(set->island_pool, island_index);

    struct ds_Joint *joint = pipeline->joint_pool.buf + island->joint_list.first;
    for (u32 i = 0; i < island->joint_list.count; ++i)
    {
        ds_Assert(joint->island == island_index);
        ds_Assert(joint->set == SOLVER_SET_NULL);
        struct ds_CGraphColor *color = cg->color + joint->color;

        slot = ds_CPoolPush(set->joint_sim_pool);
        memcpy(slot.address, color->joint_sim_pool.buf + joint->sim, sizeof(struct ds_JointSim));
        ds_CGraphJointRemove(pipeline, joint);

        joint->set = island->set;
        joint->sim = slot.index;
        struct ds_Joint *joint = pipeline->joint_pool.buf + joint->island_list_node.next;
    }

    struct ds_Contact *contact = NULL;
    for (u32 i = island->contact_list.first; (i32) i != DLL_SENTINEL; i = contact->island_contact.next)
    {
        contact = pipeline->cdb->contact_pool.buf + i;
        ds_Assert(contact->set == SOLVER_SET_NULL);
        //TODO: do we need this link?
        //ds_Assert(contact->island == island_index);
        struct ds_CGraphColor *color = cg->color + contact->color;

        slot = ds_CPoolPush(set->contact_pool);
        set->contact_pool.buf[ slot.index ] = ds_ContactPoolIndex(&pipeline->cdb->contact_pool, contact);
        ds_CGraphContactRemove(pipeline, contact);

        contact->set = island->set;
        contact->set_contact_index = slot.index;
    }

    struct ds_RigidBody *body = NULL;
    for (u32 i = island->body_list.first; (i32) i != DLL_SENTINEL; i = body->island_body.next)
    {
        body = pipeline->body_pool.buf + i;
        ds_Assert(body->island_index == island_index);
        ds_Assert(body->set == SOLVER_SET_ACTIVE);
        ds_SolverSetMoveBody(pipeline, i, island->set);
    }
}

void ds_SolverSetMerge(struct ds_RigidBodyPipeline *pipeline, const u32 set_expand, const u32 set_merge)
{
    struct ds_SolverSet *expand = pipeline->solver_set_pool.buf + set_expand;
    struct ds_SolverSet *merge = pipeline->solver_set_pool.buf + set_merge;
    ds_Assert(set_expand != set_merge);
    ds_Assert(ds_PoolSlotAllocated(expand));
    ds_Assert(ds_PoolSlotAllocated(merge));

    for (u32 i = 0; i < merge->body_sim_pool.count; ++i)
    {
        const struct ds_RigidBodySim *old_sim = merge->body_sim_pool.buf + i;
        struct ds_RigidBody *body = pipeline->body_pool.buf + old_sim->body;
        ds_Assert(body->set == set_merge);

        const struct slot slot = ds_CPoolPush(expand->body_sim_pool);
        body->set = set_expand;
        body->sim = slot.index;

        struct ds_RigidBodySim *new_sim = slot.address;
        memcpy(new_sim, old_sim, sizeof(*new_sim));
    }

    for (u32 i = 0; i < merge->island_pool.count; ++i)
    {
        const u32 isi = merge->island_pool.buf[i];
        struct ds_Island *is = pipeline->is_db.island_pool.buf + isi;
        ds_Assert(is->set == set_merge);

        is->set = set_expand;
        is->set_island_index = ds_CPoolPush(expand->island_pool).index;
        expand->island_pool.buf[ is->set_island_index ] = isi;
    }

    if (set_expand == SOLVER_SET_ACTIVE)
    {
        for (u32 i = 0; i < merge->contact_pool.count; ++i)
        {
            const u32 ci = merge->contact_pool.buf[i];
            struct ds_Contact *contact = pipeline->cdb->contact_pool.buf + ci;
            ds_CGraphContactAdd(pipeline, contact);
        }

        for (u32 i = 0; i < merge->joint_sim_pool.count; ++i)
        {
            struct ds_JointSim *old_sim = merge->joint_sim_pool.buf + i;
            struct ds_Joint *joint = pipeline->joint_pool.buf + old_sim->joint;
            struct ds_JointSim *new_sim = ds_CGraphJointAdd(pipeline, joint);
            memcpy(new_sim, old_sim, sizeof(struct ds_JointSim));
        }
    }
    else
    {
        for (u32 i = 0; i < merge->contact_pool.count; ++i)
        {
            const u32 ci = merge->contact_pool.buf[i];
            struct ds_Contact *contact = pipeline->cdb->contact_pool.buf + ci;
            contact->set_contact_index = ds_CPoolPush(expand->contact_pool).index;
            contact->set = set_expand;
            expand->contact_pool.buf[ contact->set_contact_index ] = ci;
        }

        for (u32 i = 0; i < merge->joint_sim_pool.count; ++i)
        {
            struct ds_JointSim *old_sim = merge->joint_sim_pool.buf + i;
            struct ds_Joint *joint = pipeline->joint_pool.buf + old_sim->joint;
            const struct slot slot = ds_CPoolPush(expand->joint_sim_pool);
            struct ds_JointSim *new_sim = slot.address;
            joint->sim = slot.index;
            joint->set = set_expand;
            memcpy(new_sim, old_sim, sizeof(struct ds_JointSim));
        }
    } 

    ds_SolverSetRemove(pipeline, set_merge);
}

void ds_SolverSetValidate(const struct ds_RigidBodyPipeline *pipeline, const u32 set_index)
{
    const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + set_index;
    ds_Assert(ds_PoolSlotAllocated(set));
    for (u32 i = 0; i < set->island_pool.count; ++i)
    {
        const struct ds_Island *island = pipeline->is_db.island_pool.buf + set->island_pool.buf[i];
        ds_Assert(island->set == set_index);
        ds_Assert(island->set_island_index == i);
    }

    for (u32 i = 0; i < set->body_sim_pool.count; ++i)
    {
        const struct ds_RigidBodySim *sim = set->body_sim_pool.buf + i;
        const struct ds_RigidBody *body = pipeline->body_pool.buf + sim->body;
        ds_Assert(body->set == set_index);
        ds_Assert(body->sim == i);
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
        const u32 ci = set->contact_pool.buf[i];
        const struct ds_Contact *c = pipeline->cdb->contact_pool.buf + ci;
        ds_Assert(c->set == set_index);
        ds_Assert(c->set_contact_index == i);
    }

    if (set_index == SOLVER_SET_ACTIVE)
    {
        ds_Assert(set->contact_pool.count == 0);
    }
}

void ds_SolverSetMoveBody(struct ds_RigidBodyPipeline *pipeline, const u32 body_index, const u32 set_index)
{
	struct ds_RigidBody *body = pipeline->body_pool.buf + body_index;
    ds_Assert(body->set != set_index);

    struct ds_SolverSet *old_set = pipeline->solver_set_pool.buf + body->set;
    struct ds_SolverSet *new_set = pipeline->solver_set_pool.buf + set_index;
    const struct ds_RigidBodySim *old_sim = old_set->body_sim_pool.buf + body->sim;

    const struct slot slot = ds_CPoolPush(new_set->body_sim_pool);
    struct ds_RigidBodySim *new_sim = slot.address;
    memcpy(new_sim, old_sim, sizeof(*new_sim));

    ds_CPoolRemoveAndSwap(old_set->body_sim_pool, body->sim);
    if (body->sim < old_set->body_sim_pool.count)
    {
        const struct ds_RigidBodySim *moved_sim = old_set->body_sim_pool.buf + body->sim;
        struct ds_RigidBody *moved_body = pipeline->body_pool.buf + moved_sim->body;
        ds_Assert(moved_body->set == body->set);
        ds_Assert(moved_body->sim == old_set->body_sim_pool.count);
        moved_body->sim = body->sim;
    }

    body->set = set_index;
    body->sim = slot.index;
}
