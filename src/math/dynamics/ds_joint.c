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

    /* canonical slot: a static body is body 1 (as in the contact constraints), whatever the caller's order */
    if (!ds_BodyDynamicBit(b[0]))
    {
        const struct slot slot_tmp = slot_b0;
        slot_b0 = slot_b1;
        slot_b1 = slot_tmp;
        b[0] = slot_b0.address;
        b[1] = slot_b1.address;
        const ds_Transform *frame_tmp = local_frame0;
        local_frame0 = local_frame1;
        local_frame1 = frame_tmp;
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

    struct ds_JointSim *sim = ds_CGraphJointAdd(pipeline, joint);
    sim->local_frame[0] = *local_frame0;
    sim->local_frame[1] = *local_frame1;
    sim->type = type;
    joint->type = type;

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

/* a joint spring's hertz is bounded by the pipeline's tick (spring_hertz_max); every spring joint applies it at add and set */
static f32 ds_SpringHertzBound(const struct ds_Dynamics *pipeline, const f32 hertz)
{
    if (hertz > pipeline->spring_hertz_max)
    {
        Log(T_PHYSICS, S_WARNING, "joint spring: hertz %f above spring_hertz_max %f, using the bound", hertz, pipeline->spring_hertz_max);
        return pipeline->spring_hertz_max;
    }
    return hertz;
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

/*
 * range.high is capped at JOINT_LENGTH_MAX instead of allowing infinity. The user loses nothing: a bound of 100 km
 * is never reached, so the high row stays inactive exactly as an infinite one would. The solver gains finite
 * arithmetic everywhere: the high row's speculative bias (C / h), a zero effective mass times that bias
 * (0 * inf = NaN) and the range spring's target (low + high) / 2 all stay finite, and no stage needs an
 * infinity check.
 */
static intv ds_DistanceJointClampRange(const intv range)
{
    intv clamped;
    clamped.low = ds_DistanceJointClampLength(range.low, "range.low");
    clamped.high = F32Min(range.high, JOINT_LENGTH_MAX);
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

/* a spring at 0 Hz is no spring: disable it; a negative damping ratio would feed energy in */
static void ds_DistanceJointPrefabSpringSet(struct ds_DistanceJointPrefab *prefab, const f32 hertz, const f32 damping_ratio)
{
    prefab->enable_spring = 1;
    prefab->hertz = hertz;
    prefab->damping_ratio = damping_ratio;
    if (hertz <= 0.0f)
    {
        Log(T_PHYSICS, S_WARNING, "distance joint prefab: hertz %f not positive, spring disabled", hertz);
        prefab->enable_spring = 0;
        prefab->hertz = 0.0f;
    }

    if (damping_ratio < 0.0f)
    {
        Log(T_PHYSICS, S_WARNING, "distance joint prefab: damping_ratio %f negative, using 0", damping_ratio);
        prefab->damping_ratio = 0.0f;
    }
}

void ds_DistanceJointPrefabSpring(struct ds_DistanceJointPrefab *prefab, const f32 distance, const f32 hertz, const f32 damping_ratio)
{
    *prefab = (struct ds_DistanceJointPrefab) { 0 };
    prefab->target = ds_DistanceJointClampLength(distance, "distance");
    prefab->range = Intv(g_solver_config->linear_slop, JOINT_LENGTH_MAX);
    ds_DistanceJointPrefabSpringSet(prefab, hertz, damping_ratio);
}

void ds_DistanceJointPrefabRangeSpring(struct ds_DistanceJointPrefab *prefab, const intv range, const f32 hertz, const f32 damping_ratio)
{
    *prefab = (struct ds_DistanceJointPrefab) { 0 };
    prefab->range = ds_DistanceJointClampRange(range);
    prefab->target = 0.5f*(prefab->range.low + prefab->range.high);
    ds_DistanceJointPrefabSpringSet(prefab, hertz, damping_ratio);
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

    /* the prefab constructors clamp these; prefabs come from them only (header WARNING), so a violation is a bug, not input */
    ds_Assert(prefab->target >= g_solver_config->linear_slop);
    ds_Assert(prefab->range.low >= g_solver_config->linear_slop);
    ds_Assert(prefab->range.high >= prefab->range.low);
    /* the spring block of prepare divides by c + h k, which these make positive */
    ds_Assert(!prefab->enable_spring || (prefab->hertz > 0.0f && prefab->damping_ratio >= 0.0f));

    /* per-tick data is computed by prepare; the cached impulses start at zero */
    sim->distance = (struct ds_DistanceJoint) { 0 };

    /*
     * Anchors within linear_slop give the joint no axis, and prepare keeps the previous one. A joint that starts
     * there gets an arbitrary first axis, +x, so a lower bound above linear_slop has a direction to push along;
     * the joint stays valid, only the direction of that first push is made up. A free lower bound sees no push.
     */
    {
        const f32 slop = g_solver_config->linear_slop;
        v3f64 anchor[2];
        for (u32 i = 0; i < 2; ++i)
        {
            /* the sim's frames, not the caller's: the add may have swapped the slots (static body in slot 1) */
            const struct ds_Body *body = pipeline->body_pool.buf + joint->body[i];
            const struct ds_BodySim *bsim = pipeline->solver_set_pool.buf[body->set].body_sim_pool.buf + body->sim;
            anchor[i] = V3F64Add(bsim->world.position, V3Promote(QUnitRotate(bsim->world.rotation, sim->local_frame[i].position)));
        }

        const f32 distance = V3Length(V3F64Demote(V3F64Sub(anchor[1], anchor[0])));
        if (distance <= slop)
        {
            if (prefab->range.low > slop)
            {
                Log(T_PHYSICS, S_WARNING, "distance joint add: anchors %f apart (within linear_slop %f) with range.low %f, first push along +x", distance, slop, prefab->range.low);
            }
            sim->distance.n = V3(1.0f, 0.0f, 0.0f);
        }
    }
    sim->distance.target = prefab->target;
    sim->distance.range = prefab->range;
    sim->distance.enable_spring = prefab->enable_spring;
    sim->distance.hertz = prefab->enable_spring ? ds_SpringHertzBound(pipeline, prefab->hertz) : prefab->hertz;
    sim->distance.damping_ratio = prefab->damping_ratio;

    return id;
}

void ds_DistanceJointConstraintInit(struct ds_JointSim *jsim, struct ds_BodyCompute *bcomp[2], struct ds_BodySim *bsim[2], const f32 timestep)
{
    struct ds_DistanceJoint *dj = &jsim->distance;

    jsim->r[0] = QUnitRotate(bsim[0]->world.rotation, V3Sub(jsim->local_frame[0].position, bsim[0]->local_center_of_mass));
    jsim->r[1] = QUnitRotate(bsim[1]->world.rotation, V3Sub(jsim->local_frame[1].position, bsim[1]->local_center_of_mass));
    /* computed, not the field: a static sim's world_center_of_mass is never written (ds_BodyUpdateSolverDataRange runs over the active set) */
    jsim->com_diff = V3F64Demote(V3F64Sub(ds_BodySimWorldCenterOfMass(bsim[1]), ds_BodySimWorldCenterOfMass(bsim[0])));

    const v3 diff = V3Sub(V3Add(jsim->com_diff, jsim->r[1]), jsim->r[0]);
    dj->distance = V3Length(diff);

    /* 
     * Allowed approach speed towards each bound, as we get closer to bound from within,
     * the allowed speed approaches zero. 
     */
    dj->allowed_separation_velocity_min = -F32Max(0.0f, dj->distance - dj->range.low) / timestep;
    dj->allowed_separation_velocity_max = F32Max(0.0f, dj->range.high - dj->distance) / timestep;
    
    /*
     * Below linear_slop the anchors give no axis; keep the previous tick's n (the add sets one when the anchors
     * start there), so the rows and the position pass still push along the direction the anchors came from.
     * The cached impulses keep their meaning along it.
     */
    if (dj->distance > g_solver_config->linear_slop)
    {
        dj->n = V3Scale(diff, 1.0f/dj->distance);
    }

    const v3 cross[2] = 
    {
        V3Cross(jsim->r[0], dj->n),
        V3Cross(jsim->r[1], dj->n),
    };

    const v3 tmp[2] =
    {
        M3V3Mul(bsim[0]->world_inv_inertia, cross[0]),
        M3V3Mul(bsim[1]->world_inv_inertia, cross[1]),
    };

    const f32 inv_normal_mass = bsim[0]->inv_mass + bsim[1]->inv_mass + V3Dot(cross[0], tmp[0]) + V3Dot(cross[1], tmp[1]);
    /* body 0 is dynamic with mass, so inv_mass >= inv_mass0 > 0; a jointed dynamic body without shapes is a setup bug */
    ds_Assert(inv_normal_mass > 0.0f);
    dj->normal_mass = 1.0f / inv_normal_mass;

    dj->gamma = 0.0f;
    dj->bias_velocity = 0.0f;
    dj->soft_mass = 0.0f;
    if (dj->enable_spring)
    {
        const f32 angular_frequency = dj->hertz * F32_PI2;
        const f32 k = dj->normal_mass * angular_frequency * angular_frequency;
        const f32 c = 2.0f * dj->normal_mass * dj->damping_ratio * angular_frequency;
        dj->gamma = 1.0f / (timestep *(c + k*timestep));
        const f32 beta = timestep * k * dj->gamma;
        dj->bias_velocity = beta * (dj->distance - dj->target);
        dj->soft_mass = 1.0f / (inv_normal_mass + dj->gamma);
    }
    /* the row toward target: rigid for a point range, the spring when enabled, none for a free range */
    dj->target_mass = (dj->range.low == dj->range.high) ? dj->normal_mass
                    : (dj->enable_spring)                ? dj->soft_mass
                    :                                      0.0f;
}

void ds_DistanceJointConstraintWarmup(struct ds_JointSim *jsim, struct ds_BodyCompute *bcomp[2], struct ds_BodySim *bsim[2])
{
    struct ds_DistanceJoint *dj = &jsim->distance;

    const v3 cached_impulse = V3Scale(dj->n, dj->impulse + dj->impulse_low - dj->impulse_high);
    bcomp[0]->linear_velocity = V3AddScaled(bcomp[0]->linear_velocity, cached_impulse, -bsim[0]->inv_mass);
    bcomp[1]->linear_velocity = V3AddScaled(bcomp[1]->linear_velocity, cached_impulse, bsim[1]->inv_mass);

    const v3 delta_w0 = M3V3Mul(bsim[0]->world_inv_inertia, V3Cross(jsim->r[0], cached_impulse));
    bcomp[0]->angular_velocity = V3Sub(bcomp[0]->angular_velocity, delta_w0);

    const v3 delta_w1 = M3V3Mul(bsim[1]->world_inv_inertia, V3Cross(jsim->r[1], cached_impulse));
    bcomp[1]->angular_velocity = V3Add(bcomp[1]->angular_velocity, delta_w1);
}

void ds_DistanceJointConstraintIterate(struct ds_JointSim *jsim, struct ds_BodyCompute *bcomp[2], struct ds_BodySim *bsim[2])
{
    struct ds_DistanceJoint *dj = &jsim->distance;

    /* Calculate separating velocity at point: JV */
    v3 relative_velocity = V3Sub(bcomp[1]->linear_velocity, bcomp[0]->linear_velocity);
    relative_velocity = V3Add(relative_velocity, V3Cross(bcomp[1]->angular_velocity, jsim->r[1]));
    relative_velocity = V3Sub(relative_velocity, V3Cross(bcomp[0]->angular_velocity, jsim->r[0]));
    const f32 separating_velocity = V3Dot(dj->n, relative_velocity);

    /*
     * target_mass == normal_mass for rigid distance joints
     *             == 0.0f        for range distance joints
     *             == soft_mass   for spring and spring range joints 
     *
     * Softness terms are initialized to 0.0f for non-springs.
     */
    f32 delta_impulse = -dj->target_mass * (separating_velocity + dj->bias_velocity + dj->gamma*dj->impulse);
    dj->impulse += delta_impulse;

    if (dj->range.low < dj->range.high)
    {
        /* 
         * delta_impulse_low >  0.0f when velocity <  min => apply impulse
         *                   <= 0.0f when velocity >= min  => release impulse
         *
         * separating velocity need to account for updated target impulse velocity;
         *
         *                  sep_new =             sep + delta_impulse / normal_mass
         *  =>  normal_mass*sep_new = normal_mass*sep + delta_impulse
         */
        f32 delta_impulse_low = dj->normal_mass * (dj->allowed_separation_velocity_min - separating_velocity) - delta_impulse;
        const f32 new_impulse_low = F32Max(0.0f, dj->impulse_low + delta_impulse_low);
        delta_impulse_low = new_impulse_low - dj->impulse_low;
        delta_impulse += delta_impulse_low;
        dj->impulse_low = new_impulse_low;

        /* 
         * delta_impulse_high >  0.0f when velocity >  max => apply impulse
         *                    <= 0.0f when velocity <= max  => release impulse
         *
         * separating velocity need to account for updated target and low impulse velocity;
         *
         *                  sep_new =             sep + delta_impulse / normal_mass
         *  =>  normal_mass*sep_new = normal_mass*sep + delta_impulse
         */
        f32 delta_impulse_high = dj->normal_mass * (separating_velocity - dj->allowed_separation_velocity_max) + delta_impulse;
        const f32 new_impulse_high = F32Max(0.0f, dj->impulse_high + delta_impulse_high);
        delta_impulse_high = new_impulse_high - dj->impulse_high;
        delta_impulse += -delta_impulse_high;
        dj->impulse_high = new_impulse_high;
    }

    /* update body velocities */
    const v3 impulse = V3Scale(dj->n, delta_impulse);

    const v3 delta_w0 = M3V3Mul(bsim[0]->world_inv_inertia, V3Cross(jsim->r[0], impulse));
    bcomp[0]->linear_velocity = V3AddScaled(bcomp[0]->linear_velocity, impulse, -bsim[0]->inv_mass);
    bcomp[0]->angular_velocity = V3Sub(bcomp[0]->angular_velocity, delta_w0);

    const v3 delta_w1 = M3V3Mul(bsim[1]->world_inv_inertia, V3Cross(jsim->r[1], impulse));
    bcomp[1]->linear_velocity = V3AddScaled(bcomp[1]->linear_velocity, impulse, bsim[1]->inv_mass);
    bcomp[1]->angular_velocity = V3Add(bcomp[1]->angular_velocity, delta_w1);
}

void ds_DistanceJointPositionConstraintIterate(struct ds_JointSim *jsim, struct ds_BodyCompute *bcomp[2], struct ds_BodySim *bsim[2])
{
    struct ds_DistanceJoint *dj = &jsim->distance;

    const m3 rot0 = M3QUnit(bcomp[0]->rotation);
    const m3 inv_inertia0 = M3Mul(M3Mul(rot0, bsim[0]->local_inv_inertia), M3Transpose(rot0));
    bsim[0]->world_inv_inertia = inv_inertia0;

    const m3 rot1 = M3QUnit(bcomp[1]->rotation);
    const m3 inv_inertia1 = M3Mul(M3Mul(rot1, bsim[1]->local_inv_inertia), M3Transpose(rot1));
    bsim[1]->world_inv_inertia = inv_inertia1;

    const v3 r0 = QUnitRotate(bcomp[0]->rotation, V3Sub(jsim->local_frame[0].position, bsim[0]->local_center_of_mass));
    const v3 r1 = (jsim->body_sim[1] == ACTIVE_BODY_DUMMY_INDEX)
                ? jsim->r[1]
                : QUnitRotate(bcomp[1]->rotation, V3Sub(jsim->local_frame[1].position, bsim[1]->local_center_of_mass));
    const v3 delta = V3Add(V3Sub(bcomp[1]->delta_position, bcomp[0]->delta_position), V3Sub(r1, r0));
    
    const v3 diff = V3Add(jsim->com_diff, delta);
    const f32 distance = V3Length(diff);
    
    /* below linear_slop: the axis prepare kept (see ds_DistanceJointConstraintInit) */
    const v3 n = (distance > g_solver_config->linear_slop)
          ? V3Scale(diff, 1.0f/distance)
          : dj->n;

    const v3 rn0 = V3Cross(r0, n);
    const v3 rn1 = V3Cross(r1, n);

    /* inverse effective mass? */
    const f32 K = bsim[0]->inv_mass + bsim[1]->inv_mass + V3Dot(M3V3Mul(inv_inertia0, rn0), rn0) + V3Dot(M3V3Mul(inv_inertia1, rn1), rn1);

    /* range is set to [target, target] for basic distance joint */
    const f32 correction = distance - F32Clamp(distance, dj->range.low, dj->range.high);
    const f32 C = F32Clamp(correction, -g_solver_config->max_linear_correction, g_solver_config->max_linear_correction);

    /* body 0 is dynamic with mass, so K >= inv_mass0 > 0 */
    ds_Assert(K > 0.0f);
    const f32 impulse = -C/K;

    const v3 impulse_vector = V3Scale(n, impulse);
    bcomp[0]->delta_position = V3AddScaled(bcomp[0]->delta_position, impulse_vector, -bsim[0]->inv_mass);
    bcomp[1]->delta_position = V3AddScaled(bcomp[1]->delta_position, impulse_vector,  bsim[1]->inv_mass);
    /* flipped cross for correct sign! */
    /* instantaneous torque, assume delta_t = 1 */
    const v3 w0 = M3V3Mul(inv_inertia0, V3Cross(impulse_vector, r0));
    /* Taylor expansion for sin, cos around 0 yields following approximation */
    const q quat_angle0 = Q(w0.x/2.0f, w0.y/2.0f, w0.z/2.0f, 1.0f);
    bcomp[0]->rotation = QNormalize(QMul(quat_angle0, bcomp[0]->rotation));

    const v3 w1 = M3V3Mul(inv_inertia1, V3Cross(r1, impulse_vector));
    const q quat_angle1 = Q(w1.x/2.0f, w1.y/2.0f, w1.z/2.0f, 1.0f);
    bcomp[1]->rotation = QNormalize(QMul(quat_angle1, bcomp[1]->rotation));
}

/*
 * The value functions turn a user's parameter into one the solver accepts: the parameter when it is valid,
 * otherwise default_value with a warning (a prefab passes its default, a setter the joint's current value).
 * The prefab constructors, the add and the setters share them; the hertz cap is ds_SpringHertzBound.
 */
static f32 ds_MouseJointHertzValue(const f32 hertz, const f32 default_value)
{
    if (hertz <= 0.0f)
    {
        Log(T_PHYSICS, S_WARNING, "mouse joint: hertz %f not positive, using %f", hertz, default_value);
        return default_value;
    }
    return hertz;
}

static f32 ds_MouseJointDampingRatioValue(const f32 damping_ratio, const f32 default_value)
{
    if (damping_ratio < 0.0f)
    {
        Log(T_PHYSICS, S_WARNING, "mouse joint: damping_ratio %f negative, using %f", damping_ratio, default_value);
        return default_value;
    }
    return damping_ratio;
}

static f32 ds_MouseJointMaxForceValue(const f32 max_force, const f32 default_value)
{
    if (max_force < 0.0f)
    {
        Log(T_PHYSICS, S_WARNING, "mouse joint: max_force %f negative, using %f", max_force, default_value);
        return default_value;
    }
    return max_force;
}

static f32 ds_MouseJointMaxTorqueValue(const f32 max_torque, const f32 default_value)
{
    if (max_torque < 0.0f)
    {
        Log(T_PHYSICS, S_WARNING, "mouse joint: max_torque %f negative, using %f", max_torque, default_value);
        return default_value;
    }
    return max_torque;
}

void ds_MouseJointPrefab(struct ds_MouseJointPrefab *prefab, const v3f64 target, const f32 hertz, const f32 damping_ratio, const f32 max_force, const f32 max_torque)
{
    *prefab = (struct ds_MouseJointPrefab) { 0 };

    prefab->target = target;
    prefab->hertz = ds_MouseJointHertzValue(hertz, 1.0f);
    prefab->damping_ratio = ds_MouseJointDampingRatioValue(damping_ratio, 1.0f);
    prefab->max_force = ds_MouseJointMaxForceValue(max_force, 0.0f);
    prefab->max_torque = ds_MouseJointMaxTorqueValue(max_torque, 0.0f);
}

void ds_MouseJointPrefabFromBody(struct ds_MouseJointPrefab *prefab, const struct ds_Dynamics *pipeline, const ds_BodyId body, const v3f64 target, const f32 hertz, const f32 damping_ratio, const f32 force_scale, const f32 torque_scale)
{
    const struct ds_Body *b = ds_BodyLookup(pipeline, body).address;
    if (!b || !ds_BodyDynamicCheck(b))
    {
        /* a prefab that pulls with nothing: the joint can still be added, and does nothing */
        Log(T_PHYSICS, S_WARNING, "mouse joint prefab: body %u missing or not dynamic, the prefab pulls with no force", ds_BodyIdIndex(body));
        ds_MouseJointPrefab(prefab, target, hertz, damping_ratio, 0.0f, 0.0f);
        return;
    }

    const struct ds_BodySim *bsim = pipeline->solver_set_pool.buf[b->set].body_sim_pool.buf + b->sim;
    const f32 weight = b->mass * V3Length(g_solver_config->gravity);

    /* radius of gyration sqrt(trace(I) / (3 m)); the sim stores Inv(I), and a dynamic body's inertia inverts */
    //TODO(Research): i do not understand
    m3 inertia;
    const f32 det = M3Inverse(&inertia, bsim->local_inv_inertia);
    ds_Assert(det > 0.0f);
    const f32 trace = inertia.a11 + inertia.a22 + inertia.a33;
    const f32 lever = F32Sqrt(trace / (3.0f * b->mass));

    ds_MouseJointPrefab(prefab, target, hertz, damping_ratio, force_scale * weight, torque_scale * lever * weight);
}

/* the setters write the sim of an awake mouse joint: wake its set first (a NO-OP for an awake island) */
void ds_MouseJointTargetSet(struct ds_Dynamics *pipeline, const ds_JointId id, const v3f64 target)
{
    struct ds_Joint *j = ds_JointLookup(pipeline, id).address;
    if (j && j->type == DS_JOINT_TYPE_MOUSE)
    {
        ds_SolverSetWakeUp(pipeline, j->set);
        struct ds_JointSim *sim = pipeline->cgraph.color[j->color].joint_sim_pool.buf + j->sim;
        sim->mouse.target = target;
    }
}

void ds_MouseJointSpringSet(struct ds_Dynamics *pipeline, const ds_JointId id, const f32 hertz, const f32 damping_ratio)
{
    struct ds_Joint *j = ds_JointLookup(pipeline, id).address;
    if (j && j->type == DS_JOINT_TYPE_MOUSE)
    {
        ds_SolverSetWakeUp(pipeline, j->set);
        struct ds_JointSim *sim = pipeline->cgraph.color[j->color].joint_sim_pool.buf + j->sim;
        sim->mouse.hertz = ds_SpringHertzBound(pipeline, ds_MouseJointHertzValue(hertz, sim->mouse.hertz));
        sim->mouse.damping_ratio = ds_MouseJointDampingRatioValue(damping_ratio, sim->mouse.damping_ratio);
    }
}

void ds_MouseJointStrengthSet(struct ds_Dynamics *pipeline, const ds_JointId id, const f32 max_force)
{
    struct ds_Joint *j = ds_JointLookup(pipeline, id).address;
    if (j && j->type == DS_JOINT_TYPE_MOUSE)
    {
        ds_SolverSetWakeUp(pipeline, j->set);
        struct ds_JointSim *sim = pipeline->cgraph.color[j->color].joint_sim_pool.buf + j->sim;
        sim->mouse.max_force = ds_MouseJointMaxForceValue(max_force, sim->mouse.max_force);
    }
}

void ds_MouseJointFrictionSet(struct ds_Dynamics *pipeline, const ds_JointId id, const f32 max_torque)
{
    struct ds_Joint *j = ds_JointLookup(pipeline, id).address;
    if (j && j->type == DS_JOINT_TYPE_MOUSE)
    {
        ds_SolverSetWakeUp(pipeline, j->set);
        struct ds_JointSim *sim = pipeline->cgraph.color[j->color].joint_sim_pool.buf + j->sim;
        sim->mouse.max_torque = ds_MouseJointMaxTorqueValue(max_torque, sim->mouse.max_torque);
    }
}

ds_JointId ds_MouseJointAdd(struct ds_Dynamics *pipeline, const struct ds_MouseJointPrefab *prefab, const ds_BodyId body, const ds_Transform *local_frame)
{
    const ds_Transform anchor_frame = ds_TransformIdentity();
    const ds_JointId id = ds_JointAdd(pipeline, DS_JOINT_TYPE_MOUSE, 0, body, local_frame, DS_BODY_MOUSE_ANCHOR_ID, &anchor_frame);
    if (ds_JointIdNullCheck(id))
    {
        return ds_JointIdNull();
    }

    struct ds_Joint *joint = pipeline->joint_pool.buf + ds_JointIdIndex(id);
    struct ds_CGraphColor *color = pipeline->cgraph.color + joint->color;
    struct ds_JointSim *sim = color->joint_sim_pool.buf + joint->sim;

    ds_Assert(prefab->hertz > 0.0f && prefab->damping_ratio >= 0.0f);
    sim->mouse = (struct ds_MouseJoint) { 0 };

    sim->mouse.target = prefab->target;
    sim->mouse.hertz = ds_SpringHertzBound(pipeline, prefab->hertz);
    sim->mouse.damping_ratio = prefab->damping_ratio;
    sim->mouse.max_force = prefab->max_force;
    sim->mouse.max_torque = prefab->max_torque;

    return id;
}

void ds_MouseJointConstraintInit(struct ds_JointSim *jsim, struct ds_BodyCompute *bcomp[2], struct ds_BodySim *bsim[2], const f32 timestep)
{
    //TODO
}

void ds_MouseJointConstraintWarmup(struct ds_JointSim *jsim, struct ds_BodyCompute *bcomp[2], struct ds_BodySim *bsim[2])
{
    //TODO
}

void ds_MouseJointConstraintIterate(struct ds_JointSim *jsim, struct ds_BodyCompute *bcomp[2], struct ds_BodySim *bsim[2])
{
    //TODO
}
