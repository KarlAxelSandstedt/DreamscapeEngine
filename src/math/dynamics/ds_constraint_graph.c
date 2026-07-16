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

struct slot ds_CGraphJointAdd(u32 *color, u32 *index, struct ds_RigidBodyPipeline *pipeline, const u32 body0, const u32 body1)
{

}

void ds_CGraphJointRemove(struct ds_RigidBodyPipeline *pipeline, const u32 color, const u32 index)
{

}
