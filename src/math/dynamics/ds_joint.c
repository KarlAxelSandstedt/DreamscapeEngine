#include "dynamics.h"

POOL_DEFINE(ds_Joint);

ds_JointId ds_JointAdd(struct ds_RigidBodyPipeline *pipeline, const ds_RigidBodyId b0_id, const ds_RigidBodyId b1_id)
{
    struct slot slot_b0 = ds_RigidBodyLookup(pipeline, b0_id);
    struct slot slot_b1 = ds_RigidBodyLookup(pipeline, b1_id);
    struct ds_RigidBody *b0 = slot_b0.address;
    struct ds_RigidBody *b1 = slot_b1.address;
    if (!b0 || !b1)
    {
        return DS_ID_NULL;
    }

    struct slot slot_joint = ds_JointPoolAdd(&pipeline->joint_pool);
    struct ds_Joint *joint = slot_joint.address;
    joint->tag += DS_ID_TAG_GENERATION_INCREMENT;
    const ds_JointId id = ds_IdConstruct(slot_joint.index, joint->tag);

    //TODO is the order here relevant? 
    joint->body[0] = slot_b0.index;
    joint->body[1] = slot_b1.index;
    ds_DLLAppend(b0->joint_list, pipeline->joint_pool.buf, slot_joint.index, edge_node[0]);
    ds_DLLAppend(b1->joint_list, pipeline->joint_pool.buf, slot_joint.index, edge_node[1]);

    struct slot slot_joint_sim = ds_CGraphJointAdd(pipeline, joint);

    return id;
}

void ds_JointRemove(struct ds_RigidBodyPipeline *pipeline, const ds_JointId id)
{
    struct slot slot = ds_JointLookup(pipeline, id);
    if (slot.address)
    {
        struct ds_Joint *joint = slot.address;
        ds_CGraphJointRemove(pipeline, joint);
        ds_JointPoolRemove(&pipeline->joint_pool, slot.index);
    }
}

struct slot ds_JointLookup(const struct ds_RigidBodyPipeline *pipeline, const ds_JointId id)
{
    const u32 index = DS_ID_INDEX_MASK & id;
    if (index <= pipeline->joint_pool.count_max)
    {
        return (struct slot) { .index = U32_MAX, .address = NULL };
    }
            
    struct ds_Joint *joint = pipeline->joint_pool.buf + index;
    return  (ds_PoolSlotAllocated(joint) && joint->tag == ds_IdTag(id))
        ? (struct slot) { .index = index, .address = joint }
        : (struct slot) { .index = U32_MAX, .address = NULL };
}
