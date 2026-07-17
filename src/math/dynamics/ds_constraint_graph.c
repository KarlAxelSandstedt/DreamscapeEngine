#include "dynamics.h"

struct ds_CGraph ds_CGraphAlloc(const u32 initial_count)
{
    struct ds_CGraph cg = { 0 };

    for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
    {
        ds_CPoolAlloc(NULL, cg.color[i].joint_pool, initial_count, GROWABLE);
    }

    return cg;
}

void ds_CGraphDealloc(struct ds_CGraph *cg)
{
    for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
    {
        ds_CPoolDealloc(cg->color[i].joint_pool);
    }
}

void ds_CGraphFlush(struct ds_CGraph *cg)
{
    for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
    {
        ds_CPoolFlush(cg->color[i].joint_pool);
    }
}

struct slot ds_CGraphJointAdd(struct ds_RigidBodyPipeline *pipeline, struct ds_Joint *joint)
{

    //struct ds_JointSim *sim = slot.address;
    //joint->sim = slot.index;
}

void ds_CGraphJointRemove(struct ds_RigidBodyPipeline *pipeline, struct ds_Joint *joint)
{

}
