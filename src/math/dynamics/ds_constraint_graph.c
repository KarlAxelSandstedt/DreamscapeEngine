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

void ds_CGraphAlloc(struct ds_RigidBodyPipeline *pipeline, const u32 initial_count)
{
    struct ds_CGraph *cg = &pipeline->cgraph;
    memset(cg, 0, sizeof(pipeline->cgraph));
    for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
    {
        ds_CPoolAlloc(NULL, cg->color[i].joint_sim_pool, initial_count, GROWABLE);
        ds_CPoolAlloc(NULL, cg->color[i].contact_pool, initial_count, GROWABLE);
        ds_CPoolAlloc(NULL, cg->color[i].contact_compute_pool, initial_count, GROWABLE);
        //ds_CPoolAlloc(NULL, cg->color[i].contact_constraint_pool, initial_count, GROWABLE);
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
        ds_CPoolDealloc(cg->color[i].contact_pool);
        ds_CPoolDealloc(cg->color[i].contact_compute_pool);
        //ds_CPoolDealloc(cg->color[i].contact_constraint_pool);
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
        ds_CPoolFlush(cg->color[i].contact_pool);
        ds_CPoolFlush(cg->color[i].contact_compute_pool);
        //ds_CPoolFlush(cg->color[i].contact_constraint_pool);
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

static u32 ds_CGraphColorNext(struct ds_RigidBodyPipeline *pipeline, const u32 body[2])
{
    struct ds_CGraph *cg = &pipeline->cgraph;

    const struct ds_RigidBody *b0 = pipeline->body_pool.buf + body[0];
    const struct ds_RigidBody *b1 = pipeline->body_pool.buf + body[1];

    const u32 dynamic_bit[2] = { RB_DYNAMIC_BIT(b0), RB_DYNAMIC_BIT(b1) };
    const u32 dynamic_dynamic = dynamic_bit[0] & dynamic_bit[1];
    u32 color = CG_SERIAL_COLOR;

    /*
     * Since static bodies does not have their orientations updated, we don't care about multiple
     * constraints of such bodies having the same color, hence we skip updating the body_bitset for
     * static bodies. Furthermore, we don't care about the body_bitset for CG_SERIAL_COLOR either.
     */
    if (dynamic_dynamic)
    {
        for (u32 i = CG_DYNAMIC_COLOR_FIRST; i <= CG_DYNAMIC_COLOR_LAST; ++i)
        {
            if (ds_BitSetGet(&cg->color[i].body_bitset, body[0]) || ds_BitSetGet(&cg->color[i].body_bitset, body[1]))
            {
                continue;
            }

            ds_BitSetSet(&cg->color[i].body_bitset, body[0], 1);
            ds_BitSetSet(&cg->color[i].body_bitset, body[1], 1);
            color = i;
            break;
        }
    }
    else
    {
        ds_Assert(dynamic_bit[0] || dynamic_bit[1]);
        const u32 dynamic_body = dynamic_bit[1];
        for (u32 i = CG_STATIC_COLOR_FIRST; i <= CG_STATIC_COLOR_LAST; ++i)
        {
            if (ds_BitSetGet(&cg->color[i].body_bitset, body[dynamic_body]))
            {
                continue;
            }

            ds_BitSetSet(&cg->color[i].body_bitset, body[dynamic_body], 1);
            color = i;
            break;
        }
    }

    return color;
}

struct ds_JointSim *ds_CGraphJointAdd(struct ds_RigidBodyPipeline *pipeline, struct ds_Joint *joint)
{    
    joint->color = ds_CGraphColorNext(pipeline, joint->body);
    struct ds_CGraph *cg = &pipeline->cgraph;
    struct ds_CGraphColor *color = pipeline->cgraph.color + joint->color;
    struct slot slot = ds_CPoolPush(color->joint_sim_pool);

    joint->set = SOLVER_SET_NULL;
    joint->sim = slot.index;
    color->joint_sim_pool.buf[slot.index].joint = ds_JointPoolIndex(&pipeline->joint_pool, joint);
    return slot.address;
}

void ds_CGraphJointRemove(struct ds_RigidBodyPipeline *pipeline, struct ds_Joint *joint)
{
    ds_Assert(joint->set == SOLVER_SET_NULL);
    ds_Assert(joint->color != CG_INVALID_COLOR);

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

    joint->color = CG_INVALID_COLOR;
}

void ds_CGraphContactAdd(struct ds_RigidBodyPipeline *pipeline, struct ds_Contact *contact)
{
    ds_Assert(contact->set != SOLVER_SET_NULL)
    ds_Assert(contact->color == CG_INVALID_COLOR);
    
    const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + contact->set;

    const struct ds_Shape *shape[2] =
    {
        pipeline->shape_pool.buf + contact->key.shape[0],
        pipeline->shape_pool.buf + contact->key.shape[1],
    };

    const u32 body[2] = { shape[0]->body, shape[1]->body };
    contact->color = ds_CGraphColorNext(pipeline, body);

    struct ds_CGraphColor *color = pipeline->cgraph.color + contact->color;
    contact->compute = ds_CPoolPush(color->contact_pool).index;
    color->contact_pool.buf[ contact->compute ] = ds_ContactPoolIndex(&pipeline->contact_pool, contact);
    //TODO
    //ds_CPoolPush(color->contact_constraint_pool);
    
    contact->set = SOLVER_SET_NULL;
}

void ds_CGraphContactRemove(struct ds_RigidBodyPipeline *pipeline, struct ds_Contact *contact)
{
    ds_Assert(contact->set == SOLVER_SET_NULL);
    ds_Assert(contact->color != CG_INVALID_COLOR);

    struct ds_CGraph *cg = &pipeline->cgraph;
    struct ds_CGraphColor *color = cg->color + contact->color;

    if (contact->color != CG_SERIAL_COLOR)
    {
        const struct ds_Shape *shape[2] =
        {
            pipeline->shape_pool.buf + contact->key.shape[0],
            pipeline->shape_pool.buf + contact->key.shape[1],
        };

        ds_BitSetSet(&color->body_bitset, shape[0]->body, 0);
        ds_BitSetSet(&color->body_bitset, shape[1]->body, 0);
    }

    ds_CPoolRemoveAndSwap(color->contact_pool, contact->compute);
    //TODO
    //ds_CPoolRemoveAndSwap(color->contact_constraint_pool, contact->compute);
    if (contact->compute < color->contact_pool.count)
    {
        const u32 moved_index = color->contact_pool.buf[ contact->compute ];
        struct ds_Contact *moved_contact = pipeline->contact_pool.buf + moved_index;
        ds_Assert(moved_contact->color == contact->color);
        ds_Assert(moved_contact->compute == color->contact_pool.count);

        moved_contact->compute = contact->compute;
    }
        
    contact->color = CG_INVALID_COLOR;
}

void ds_CGraphValidate(const struct ds_RigidBodyPipeline *pipeline)
{
    for (u32 c = 0; c < CG_COLOR_COUNT; ++c)
    {
        const struct ds_CGraphColor *color = pipeline->cgraph.color + c;
        for (u32 i = 0; i < color->contact_pool.count; ++i)
        {
            const u32 ci = color->contact_pool.buf[i];
            const struct ds_Contact *contact = pipeline->contact_pool.buf + ci;
            const struct ds_Island *island = pipeline->island_pool.buf + contact->island;

            const struct ds_Shape *shape[2] =
            {
                pipeline->shape_pool.buf + contact->key.shape[0],
                pipeline->shape_pool.buf + contact->key.shape[1],
            };

            const struct ds_RigidBody *body[2] =
            {
                pipeline->body_pool.buf + shape[0]->body,
                pipeline->body_pool.buf + shape[1]->body,
            };

            ds_Assert(contact->color == c);
            ds_Assert(contact->compute == i);
            ds_Assert(contact->set == SOLVER_SET_NULL);
            ds_Assert(island->set == SOLVER_SET_ACTIVE);

            ds_StaticAssert(CG_STATIC_COLOR_FIRST == 0, "");
            if (c <= CG_STATIC_COLOR_LAST)
            {
                ds_Assert((!RB_IS_DYNAMIC(body[0]) 
                         && RB_IS_DYNAMIC(body[1]) 
                         && body[0]->set == SOLVER_SET_STATIC
                         && body[1]->set == SOLVER_SET_ACTIVE)
                    || 
                       (RB_IS_DYNAMIC(body[0]) 
                         && !RB_IS_DYNAMIC(body[1]) 
                         && body[1]->set == SOLVER_SET_STATIC 
                         && body[0]->set == SOLVER_SET_ACTIVE));

            }
            else if (CG_DYNAMIC_COLOR_FIRST <= c && c <= CG_DYNAMIC_COLOR_LAST)
            {
                ds_Assert(RB_IS_DYNAMIC(body[0]) && RB_IS_DYNAMIC(body[1]));
                ds_Assert(body[0]->set == SOLVER_SET_ACTIVE);
                ds_Assert(body[1]->set == SOLVER_SET_ACTIVE);
            }
        }

        for (u32 i = 0; i < color->joint_sim_pool.count; ++i)
        {
            const struct ds_JointSim *sim = color->joint_sim_pool.buf + i;
            const struct ds_Joint *joint = pipeline->joint_pool.buf + sim->joint;
            const struct ds_Island *island = pipeline->island_pool.buf + joint->island;
            const struct ds_RigidBody *body0 = pipeline->body_pool.buf + joint->body[0];
            const struct ds_RigidBody *body1 = pipeline->body_pool.buf + joint->body[1];
            ds_Assert(joint->color == c);
            ds_Assert(joint->sim == i);
            ds_Assert(joint->set == SOLVER_SET_NULL);
            ds_Assert(island->set == SOLVER_SET_ACTIVE);

            ds_StaticAssert(CG_STATIC_COLOR_FIRST == 0, "");
            if (c <= CG_STATIC_COLOR_LAST)
            {
                ds_Assert(RB_IS_DYNAMIC(body0) && RB_IS_DYNAMIC(body1));
                ds_Assert(body0->set == SOLVER_SET_ACTIVE);
                ds_Assert(body1->set == SOLVER_SET_ACTIVE);
            }
            else if (CG_DYNAMIC_COLOR_FIRST <= c && c <= CG_DYNAMIC_COLOR_LAST)
            {
                ds_Assert((!RB_IS_DYNAMIC(body0) &&  RB_IS_DYNAMIC(body1))
                       || ( RB_IS_DYNAMIC(body0) && !RB_IS_DYNAMIC(body1)));
            }
        }
    } 
}
