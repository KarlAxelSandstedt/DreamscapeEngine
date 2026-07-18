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

POOL_DEFINE(ds_Joint);

ds_JointId  ds_JointAdd(struct ds_RigidBodyPipeline *pipeline, const ds_RigidBodyId b0_id, const ds_Transform *local_frame0, const ds_RigidBodyId b1_id, const ds_Transform *local_frame1)
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

    //TODO does order matter here?
    //TODO should static have slot 1 reserved?...
    joint->body[0] = slot_b0.index;
    joint->body[1] = slot_b1.index;
    const u32 i0 = (joint->body[0] == pipeline->joint_pool.buf[(i32) b0->joint_list.last].body[1]);
    const u32 i1 = (joint->body[1] == pipeline->joint_pool.buf[(i32) b1->joint_list.last].body[1]);
    ds_DLLAppendEx(b0->joint_list, pipeline->joint_pool.buf, slot_joint.index, edge_node[i0], edge_node[0]);
    ds_DLLAppendEx(b1->joint_list, pipeline->joint_pool.buf, slot_joint.index, edge_node[i1], edge_node[1]);

    //TODO does order matter here?
    //TODO should static have slot 1 reserved?...
    struct ds_JointSim *sim = ds_CGraphJointAdd(pipeline, joint);
    sim->local_frame[0] = *local_frame0;
    sim->local_frame[1] = *local_frame1;

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

void ds_DistanceJointPrefabDefault(struct ds_DistanceJointPrefab *prefab)
{
}

ds_JointId ds_DistanceJointAdd(struct ds_RigidBodyPipeline *pipeline, const struct ds_DistanceJointPrefab *prefab, const ds_RigidBodyId b0, const ds_Transform *local_frame0, const ds_RigidBodyId b1, const ds_Transform *local_frame1)
{
    const ds_JointId id = ds_JointAdd(pipeline, b0, local_frame0, b1, local_frame1);
    if (id == DS_ID_NULL)
    {
        return DS_ID_NULL;
    }

    struct ds_Joint *joint = pipeline->joint_pool.buf + ds_IdIndex(id);
    struct ds_JointSim *sim = pipeline->cgraph.color[joint->color].joint_pool.buf + joint->sim;

    return id;
}
