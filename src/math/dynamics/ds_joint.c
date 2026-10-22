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

ds_JointId ds_JointAdd(struct ds_Dynamics *pipeline, const enum ds_JointType type, const u32 collide_connected, const ds_BodyId b0_id, const ds_Transform *local_frame0, const ds_BodyId b1_id, const ds_Transform *local_frame1)
{
    struct slot slot_b0 = ds_BodyLookup(pipeline, b0_id);
    struct slot slot_b1 = ds_BodyLookup(pipeline, b1_id);
    struct ds_Body *b[2] = { slot_b0.address, slot_b1.address };
    if (!b[0] || !b[1])
    {
        Log(T_PHYSICS, S_WARNING, "joint add rejected: body %u or %u does not exist", ds_BodyIdIndex(b0_id), ds_BodyIdIndex(b1_id));
        return ds_JointIdNull();
    }

    if (slot_b0.index == slot_b1.index)
    {
        Log(T_PHYSICS, S_WARNING, "joint add rejected: both ends on body %u", slot_b0.index);
        return ds_JointIdNull();
    }

    const u32 dynamic[2] = { ds_BodyDynamicBit(b[0]), ds_BodyDynamicBit(b[1]) };
    const u32 expand_index = b[ dynamic[1] ]->island;
    const u32 merge_index = b[ 1-dynamic[1] ]->island;
    if (!dynamic[0] && !dynamic[1])
    {
        Log(T_PHYSICS, S_WARNING, "joint add rejected: bodies %u and %u are both static", slot_b0.index, slot_b1.index);
        return ds_JointIdNull();
    }

	struct ds_Island *expand = pipeline->island_pool.buf + expand_index;
    ds_SolverSetWakeUp(pipeline, expand->set);

    if (dynamic[0] && dynamic[1] && expand_index != merge_index)
    {
		ds_IslandMerge(pipeline, expand_index, merge_index);
    }

    const u32 old_max = pipeline->joint_pool.count_max;
    struct slot slot_joint = ds_PoolAdd(pipeline->joint_pool);
    struct ds_Joint *joint = slot_joint.address;
    if (old_max != pipeline->joint_pool.count_max)
    {
        joint->id = ds_JointIdInit(slot_joint.index, 0);
    }
    joint->id = ds_JointIdNextGeneration(joint->id);

    //TODO does order matter here?
    //TODO should static have slot 1 reserved?...
    joint->body[0] = slot_b0.index;
    joint->body[1] = slot_b1.index;
    joint->island = expand_index;
    const u32 i0 = (joint->body[0] == pipeline->joint_pool.buf[b[0]->joint_list.last].body[1]);
    const u32 i1 = (joint->body[1] == pipeline->joint_pool.buf[b[1]->joint_list.last].body[1]);
    DLLAppendEx(b[0]->joint_list, pipeline->joint_pool.buf, slot_joint.index, edge_node[i0], edge_node[0]);
    DLLAppendEx(b[1]->joint_list, pipeline->joint_pool.buf, slot_joint.index, edge_node[i1], edge_node[1]);
	DLLAppend(expand->joint_list, pipeline->joint_pool.buf, slot_joint.index, island_joint);

    /* Wonky but important: if collide_connected, we need not do anything; but if
     * !collide_connected, we must call Disable with joint->collide_connected = 1,
     * since the call otherwise become a no-op. */
    joint->collide_connected = 1;
    if (!collide_connected)
    {
        ds_JointCollisionDisable(pipeline, joint->id);
    }

    //TODO does order matter here?
    //TODO should static have slot 1 reserved?...
    struct ds_JointSim *sim = ds_CGraphJointAdd(pipeline, joint);
    sim->local_frame[0] = *local_frame0;
    sim->local_frame[1] = *local_frame1;
    sim->type = type;

    return joint->id;
}

static void ds_JointUnlink(struct ds_Dynamics *pipeline, const struct ds_Joint *joint, const u32 joint_index)
{
    for (u32 i = 0; i < 2; ++i)
    {
        struct ds_Body *body = pipeline->body_pool.buf + joint->body[i];

        const u32 bi = joint->body[i];
        const i32 prev = joint->edge_node[i].prev;
        const i32 next = joint->edge_node[i].next;
        const struct ds_Joint *joint_prev = pipeline->joint_pool.buf + prev;
        const struct ds_Joint *joint_next = pipeline->joint_pool.buf + next;
        const u32 pi = (bi == joint_prev->body[1]);
        const u32 ni = (bi == joint_next->body[1]);
        DLLRemoveEx(body->joint_list, pipeline->joint_pool.buf, joint_index, edge_node[pi], edge_node[i], edge_node[ni]);
    }

    struct ds_Island *island = pipeline->island_pool.buf + joint->island;
    island->constraint_remove_count += 1;
    DLLRemove(island->joint_list, pipeline->joint_pool.buf, joint_index, island_joint);
}

void ds_JointRemoveEx(struct ds_Dynamics *pipeline, const u32 index)
{
    struct ds_Joint *joint = pipeline->joint_pool.buf + index;
    ds_SolverSetWakeUp(pipeline, joint->set);

    ds_AssertString(joint->set == SOLVER_SET_NULL, "May implement INACTIVE joints later");
    //if (joint->set == SOLVER_SET_NULL)
    {
        ds_CGraphJointRemove(pipeline, joint);
    }
    //else
    //{
    //    ds_Assert(joint->color >= CG_INVALID_COLOR);
    //    struct ds_SolverSet *set = pipeline->solver_set_pool.buf + joint->set;
    //
    //    ds_CPoolRemoveAndSwap(set->joint_sim_pool, joint->sim);
    //    if (joint->sim < set->joint_sim_pool.count)
    //    {
    //        const struct ds_JointSim *moved_sim = set->joint_sim_pool.buf + joint->sim;
    //        struct ds_Joint *moved_joint = pipeline->joint_pool.buf + moved_sim->joint;
    //        ds_Assert(moved_joint->sim == set->joint_sim_pool.count);
    //        moved_joint->sim = joint->sim;
    //    }
    //}
    
    ds_JointCollisionEnable(pipeline, joint->id);
    ds_JointUnlink(pipeline, joint, index);
    ds_PoolRemove(pipeline->joint_pool, index);

    //TODO signal affected island for wakeup / slip
}

void ds_JointRemove(struct ds_Dynamics *pipeline, const ds_JointId id)
{
    struct slot slot = ds_JointLookup(pipeline, id);
    if (slot.address)
    {
        ds_JointRemoveEx(pipeline, slot.index);
    }
}

void ds_JointCollisionEnable(struct ds_Dynamics *pipeline, const ds_JointId id)
{
    struct slot slot = ds_JointLookup(pipeline, id);
    struct ds_Joint *joint = slot.address;
    if (!joint || joint->collide_connected)
    {
        return;
    }

    joint->collide_connected = 1;
    if (ds_BodyPairMayCollide(pipeline, joint->body[0], joint->body[1]))
    {
        const struct ds_Body *body[2] = 
        {
            pipeline->body_pool.buf + joint->body[0],
            pipeline->body_pool.buf + joint->body[1],
        };

        if (ds_BodyDynamicCheck(body[0]))
        {
            i32 si = body[0]->shape_list.first;
            while (si != DLL_SENTINEL)
            {
                ds_BitSetSet(&pipeline->shape_dirty_set, si, 1);
                si = pipeline->shape_pool.buf[si].body_shape.next;
            }
        }

        if (ds_BodyDynamicCheck(body[1]))
        {
            i32 si = body[1]->shape_list.first;
            while (si != DLL_SENTINEL)
            {
                ds_BitSetSet(&pipeline->shape_dirty_set, si, 1);
                si = pipeline->shape_pool.buf[si].body_shape.next;
            }
        }
    }
}

void ds_JointCollisionDisable(struct ds_Dynamics *pipeline, const ds_JointId id)
{
    struct slot slot = ds_JointLookup(pipeline, id);
    struct ds_Joint *joint = slot.address;
    if (!joint || !joint->collide_connected)
    {
        return;
    }

    if (ds_BodyPairMayCollide(pipeline, joint->body[0], joint->body[1]))
    {
        const struct ds_Body *b[2] =
        {
            pipeline->body_pool.buf + joint->body[0],
            pipeline->body_pool.buf + joint->body[1],
        };

        const u32 dynamic[2] =
        {
            ds_BodyDynamicBit(b[0]),
            ds_BodyDynamicBit(b[1]),
        };

        /* Pick any dynamic body as a first guess. If both are dynamic, we
         * choose the body with the least amount of shapes. */
        u32 body_slot_index = dynamic[1];
        if (dynamic[0] && dynamic[1])
        {
            body_slot_index = (b[0]->shape_list.count <= b[1]->shape_list.count)
                              ? 0
                              : 1;
        }
            
        const u32 body_index = joint->body[ body_slot_index ];
        const u32 body_neighbour_index = joint->body[ 1 - body_slot_index ];

        struct ds_Body *body = pipeline->body_pool.buf + body_index;
        ds_Assert(ds_BodyDynamicCheck(body));

        i32 si = body->shape_list.first; 
        while (si != DLL_SENTINEL)
        {
            const struct ds_Shape *shape = pipeline->shape_pool.buf + si;

            i32 ci = shape->contact_list.first;
            while (ci != DLL_SENTINEL)
            {
                struct ds_Contact *contact = pipeline->contact_pool.buf + ci;
                const u32 neighbour_index = (contact->key.shape[0] == (u32) si)
                            ? 1
                            : 0;
                const u32 ci_next = contact->shape_contact[1-neighbour_index].next;

                const struct ds_Shape *neighbour_shape = pipeline->shape_pool.buf + contact->key.shape[neighbour_index];
                if (neighbour_shape->body == body_neighbour_index)
                {
                    ds_ContactRemove(pipeline, ci, 0);
                }
                ci = ci_next;
            }
            si = shape->body_shape.next;
        }
    }

    joint->collide_connected = 0;
}

void ds_JointValidateAll(const struct ds_Dynamics *pipeline)
{
    u32 joint_count = 0;
    for (u32 i = 0; i < pipeline->joint_pool.count_max; ++i)
    {
        const struct ds_Joint *joint = pipeline->joint_pool.buf + i;
        if (!ds_PoolSlotAllocated(joint))
        {
            continue;
        }
        joint_count += 1;

        ds_Assert(ds_JointIdIndex(joint->id) == i);
        ds_Assert(joint->body[0] != joint->body[1]);
        ds_Assert(ds_PoolSlotAllocated(pipeline->body_pool.buf + joint->body[0]));
        ds_Assert(ds_PoolSlotAllocated(pipeline->body_pool.buf + joint->body[1]));

        const struct ds_Island *island = pipeline->island_pool.buf + joint->island;
        ds_Assert(joint->island < pipeline->island_pool.count_max && ds_PoolSlotAllocated(island));

        /* the sim lives in the graph colour while awake, in the island's set while asleep; both point back */
        const struct ds_JointSim *sim;
        if (joint->set == SOLVER_SET_NULL)
        {
            ds_Assert(island->set == SOLVER_SET_ACTIVE);
            ds_Assert(joint->color < CG_COLOR_COUNT);
            const struct ds_CGraphColor *color = pipeline->cgraph.color + joint->color;
            ds_Assert(joint->sim < color->joint_sim_pool.count);
            sim = color->joint_sim_pool.buf + joint->sim;
        }
        else
        {
            ds_Assert(island->set == joint->set);
            ds_Assert(joint->color == CG_INVALID_COLOR);
            const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + joint->set;
            ds_Assert(joint->sim < set->joint_sim_pool.count);
            sim = set->joint_sim_pool.buf + joint->sim;
        }
        ds_Assert(sim->joint == i);
        ds_Assert(DS_JOINT_TYPE_NONE < sim->type && sim->type < DS_JOINT_TYPE_COUNT);
    }

    /* every body's joint list holds exactly the joints attached to it: each joint is walked once per end */
    u32 edge_count = 0;
    for (u32 bi = 0; bi < pipeline->body_pool.count_max; ++bi)
    {
        const struct ds_Body *body = pipeline->body_pool.buf + bi;
        if (!ds_PoolSlotAllocated(body))
        {
            continue;
        }

        u32 list_length = 0;
        const struct ds_Joint *joint = NULL;
        for (u32 ji = body->joint_list.first; (i32) ji != DLL_SENTINEL; ji = joint->edge_node[bi == joint->body[1]].next)
        {
            list_length += 1;
            joint = pipeline->joint_pool.buf + ji;
            ds_Assert(ds_PoolSlotAllocated(joint));
            ds_Assert(joint->body[0] == bi || joint->body[1] == bi);
        }
        ds_Assert(list_length == body->joint_list.count);
        edge_count += list_length;
    }
    ds_Assert(edge_count == 2*joint_count);
}

struct slot ds_JointLookup(const struct ds_Dynamics *pipeline, const ds_JointId id)
{
    /* the index is bounded before the slot is read (as ds_BodyLookup) */
    const u32 index = ds_JointIdIndex(id);
    if (index >= pipeline->joint_pool.count_max)
    {
        return (struct slot) { .index = U32_MAX, .address = NULL };
    }
            
    struct ds_Joint *joint = pipeline->joint_pool.buf + index;
    return  (ds_PoolSlotAllocated(joint) && ds_JointIdEqual(joint->id, id))
        ? (struct slot) { .index = index, .address = joint }
        : (struct slot) { .index = U32_MAX, .address = NULL };
}

/*
 * A distance below linear_slop leaves the joint no axis to act along, so the prefabs keep every length at or
 * above it. The lower bound is what holds the anchors apart; at linear_slop it is as good as no bound.
 */
static f32 ds_DistanceJointClampLength(const f32 length, const char *name)
{
    const f32 slop = g_solver_config->linear_slop;
    if (length < slop)
    {
        Log(T_PHYSICS, S_WARNING, "distance joint prefab: %s %f below linear_slop %f, using linear_slop", name, length, slop);
        return slop;
    }
    return length;
}

static intv ds_DistanceJointClampRange(const intv range)
{
    intv clamped;
    clamped.low = ds_DistanceJointClampLength(range.low, "range.low");
    clamped.high = range.high;
    if (clamped.high < clamped.low)
    {
        Log(T_PHYSICS, S_WARNING, "distance joint prefab: range.high %f below range.low %f, using range.low", clamped.high, clamped.low);
        clamped.high = clamped.low;
    }
    return clamped;
}

void ds_DistanceJointPrefab(struct ds_DistanceJointPrefab *prefab, const f32 distance)
{
    *prefab = (struct ds_DistanceJointPrefab) { 0 };
    prefab->target = ds_DistanceJointClampLength(distance, "distance");
    prefab->range = Intv(prefab->target, prefab->target);
}

void ds_DistanceJointPrefabRange(struct ds_DistanceJointPrefab *prefab, const intv range)
{
    *prefab = (struct ds_DistanceJointPrefab) { 0 };
    prefab->range = ds_DistanceJointClampRange(range);
    prefab->target = prefab->range.low;
}

void ds_DistanceJointPrefabSpring(struct ds_DistanceJointPrefab *prefab, const f32 distance, const f32 hertz, const f32 damping_ratio)
{
    *prefab = (struct ds_DistanceJointPrefab) { 0 };
    prefab->target = ds_DistanceJointClampLength(distance, "distance");
    prefab->range = Intv(g_solver_config->linear_slop, F32_INFINITY);
    prefab->enable_spring = 1;
    prefab->hertz = hertz;
    prefab->damping_ratio = damping_ratio;
}

void ds_DistanceJointPrefabRangeSpring(struct ds_DistanceJointPrefab *prefab, const intv range, const f32 hertz, const f32 damping_ratio)
{
    *prefab = (struct ds_DistanceJointPrefab) { 0 };
    prefab->range = ds_DistanceJointClampRange(range);
    prefab->target = 0.5f*(prefab->range.low + prefab->range.high);
    prefab->enable_spring = 1;
    prefab->hertz = hertz;
    prefab->damping_ratio = damping_ratio;
}

ds_JointId ds_DistanceJointAdd(struct ds_Dynamics *pipeline, const struct ds_DistanceJointPrefab *prefab, const ds_BodyId b0, const ds_Transform *local_frame0, const ds_BodyId b1, const ds_Transform *local_frame1)
{
    const ds_JointId id = ds_JointAdd(pipeline, DS_JOINT_TYPE_DISTANCE, prefab->collide_connected, b0, local_frame0, b1, local_frame1);
    if (ds_JointIdNullCheck(id))
    {
        return ds_JointIdNull();
    }

    struct ds_Joint *joint = pipeline->joint_pool.buf + ds_JointIdIndex(id);
    struct ds_CGraphColor *color = pipeline->cgraph.color + joint->color;
    struct ds_JointSim *sim = color->joint_sim_pool.buf + joint->sim;

    /* the prefab constructors clamp these; hand-written prefabs must respect the same bounds */
    ds_Assert(prefab->target >= g_solver_config->linear_slop);
    ds_Assert(prefab->range.low >= g_solver_config->linear_slop);
    ds_Assert(prefab->range.high >= prefab->range.low);

    /* per-tick data is computed by prepare; the cached impulses start at zero */
    sim->distance = (struct ds_DistanceJoint) { 0 };
    sim->distance.target = prefab->target;
    sim->distance.range = prefab->range;
    sim->distance.enable_spring = prefab->enable_spring;
    sim->distance.hertz = prefab->hertz;
    sim->distance.damping_ratio = prefab->damping_ratio;

    return id;
}
