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

#include <string.h>

#include "dynamics.h"

void ds_CGraphAlloc(struct ds_RigidBodyPipeline *pipeline, const u32 initial_count)
{
    struct ds_CGraph *cg = &pipeline->cgraph;
    memset(cg, 0, sizeof(pipeline->cgraph));
    for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
    {
        ds_CPoolAlloc(NULL, cg->color[i].joint_sim_pool, initial_count, GROWABLE);
        if (i != CG_SERIAL_COLOR)
        {
            cg->color[i].body_bitset = ds_BitSetAlloc(NULL, pipeline->body_pool.length, 0, GROWABLE);
        }
    }
}

void ds_CGraphDealloc(struct ds_RigidBodyPipeline *pipeline)
{
    struct ds_CGraph *cg = &pipeline->cgraph;
    for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
    {
        ds_CPoolDealloc(cg->color[i].joint_sim_pool);
        if (i != CG_SERIAL_COLOR)
        {
            ds_BitSetDealloc(&cg->color[i].body_bitset);
        }
    }
}

void ds_CGraphFlush(struct ds_RigidBodyPipeline *pipeline)
{
    struct ds_CGraph *cg = &pipeline->cgraph;
    for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
    {
        ds_CPoolFlush(cg->color[i].joint_sim_pool);
        if (i != CG_SERIAL_COLOR)
        {
            ds_BitSetClear(&cg->color[i].body_bitset, 0);
        }
    }
}

void ds_CGraphFramePrepare(struct ds_RigidBodyPipeline *pipeline)
{
    struct ds_CGraph *cg = &pipeline->cgraph;
    if (cg->color[CG_DYNAMIC_COLOR_FIRST].body_bitset.bit_count < pipeline->body_pool.length)
    {
        for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
        {
            if (i != CG_SERIAL_COLOR)
            {
                ds_BitSetIncreaseSize(&cg->color[i].body_bitset, 64*pipeline->body_pool.length, 0);
            }
        }
    }
}

struct ds_JointSim *ds_CGraphJointAdd(struct ds_RigidBodyPipeline *pipeline, struct ds_Joint *joint)
{
    /*
     * Since static bodies does not have their orientations updated, we don't care about multiple
     * constraints of such bodies having the same color, hence we skip updating the body_bitset for
     * static bodies. Furthermore, we don't care about the body_bitset for CG_SERIAL_COLOR either.
     */

    struct ds_CGraph *cg = &pipeline->cgraph;

    const struct ds_RigidBody *b0 = ds_PoolAddress(&pipeline->body_pool, joint->body[0]);
    const struct ds_RigidBody *b1 = ds_PoolAddress(&pipeline->body_pool, joint->body[1]);

    const u32 dynamic_bit[2] = { RB_DYNAMIC_BIT(b0), RB_DYNAMIC_BIT(b1) };
    const u32 dynamic_dynamic = dynamic_bit[0] & dynamic_bit[1];
    struct slot slot = empty_slot;
        
    if (dynamic_dynamic)
    {
        for (u32 i = CG_DYNAMIC_COLOR_FIRST; i <= CG_DYNAMIC_COLOR_LAST; ++i)
        {
            if (!ds_BitSetGet(&cg->color[i].body_bitset, joint->body[0]) || !ds_BitSetGet(&cg->color[i].body_bitset, joint->body[1]))
            {
                continue;
            }

            ds_BitSetSet(&cg->color[i].body_bitset, joint->body[0], 1);
            ds_BitSetSet(&cg->color[i].body_bitset, joint->body[1], 1);
            slot = ds_CPoolPush(cg->color[i].joint_sim_pool);
            joint->color = i;
            break;
        }
    }
    else
    {
        ds_Assert(dynamic_bit[0] || dynamic_bit[1]);
        const u32 dynamic_body = dynamic_bit[1];
        for (u32 i = CG_STATIC_COLOR_FIRST; i <= CG_STATIC_COLOR_LAST; ++i)
        {
            if (!ds_BitSetGet(&cg->color[i].body_bitset, joint->body[dynamic_body]))
            {
                continue;
            }

            ds_BitSetSet(&cg->color[i].body_bitset, joint->body[dynamic_body], 1);
            slot = ds_CPoolPush(cg->color[i].joint_sim_pool);
            joint->color = i;
            break;
        }
    }

    if (!slot.address)
    {
        slot = ds_CPoolPush(cg->color[CG_SERIAL_COLOR].joint_sim_pool);
        joint->color = CG_SERIAL_COLOR;
    }

    joint->sim = slot.index;
    cg->color[joint->color].joint_sim_pool.buf[slot.index].joint = ds_JointPoolIndex(&pipeline->joint_pool, joint);
    return slot.address;
}

void ds_CGraphJointRemove(struct ds_RigidBodyPipeline *pipeline, struct ds_Joint *joint)
{
    struct ds_CGraph *cg = &pipeline->cgraph;
    struct ds_CGraphColor *color = cg->color + joint->color;

    if (joint->color != CG_SERIAL_COLOR)
    {
        ds_BitSetSet(&color->body_bitset, joint->body[0], 0);
        ds_BitSetSet(&color->body_bitset, joint->body[1], 0);
    }

    ds_CPoolRemoveAndSwap(color->joint_sim_pool, joint->sim);
    if (joint->sim < color->joint_sim_pool.count)
    {
        const struct ds_JointSim *moved_joint_sim = color->joint_sim_pool.buf + joint->sim;
        struct ds_Joint *moved_joint = pipeline->joint_pool.buf + moved_joint_sim->joint;
        ds_Assert(moved_joint->color == joint->color);
        ds_Assert(moved_joint->sim == color->joint_sim_pool.count);

        moved_joint->sim = joint->sim;
    }
}
