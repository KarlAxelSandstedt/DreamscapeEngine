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

/* every component in [0, 1] */
static v4 ds_DynamicsDrawConfigColor(const v4 c)
{
    return V4(F32Clamp(c.x, 0.0f, 1.0f), F32Clamp(c.y, 0.0f, 1.0f), F32Clamp(c.z, 0.0f, 1.0f), F32Clamp(c.w, 0.0f, 1.0f));
}

struct ds_DynamicsDrawConfig ds_DynamicsDrawConfigDerive(const struct ds_DynamicsDrawConfig *pending)
{
    struct ds_DynamicsDrawConfig config = *pending;

    config.draw_shapes = U32Min(config.draw_shapes, 1);
    config.shape_color_mode = (enum ds_ShapeColorMode) U32Min(config.shape_color_mode, DRAW_COLOR_MODE_COUNT - 1);
    config.draw_dbvh = U32Min(config.draw_dbvh, 1);
    config.draw_sbvh = U32Min(config.draw_sbvh, 1);
    config.draw_bounds = U32Min(config.draw_bounds, 1);
    config.draw_mesh_bvh = U32Min(config.draw_mesh_bvh, 1);
    config.draw_constraint_points = U32Min(config.draw_constraint_points, 1);
    config.draw_contact_forces = U32Min(config.draw_contact_forces, 1);
    config.draw_joints = U32Min(config.draw_joints, 1);
    config.point_color_mode = (enum ds_PointColorMode) U32Min(config.point_color_mode, DRAW_POINT_COLOR_MODE_COUNT - 1);
    config.draw_on_top = U32Min(config.draw_on_top, 1);
    config.draw_body_axes = U32Min(config.draw_body_axes, 1);
    config.draw_joint_forces = U32Min(config.draw_joint_forces, 1);
    config.draw_joint_breakage = U32Min(config.draw_joint_breakage, 1);

    config.force_scale = F32Clamp(config.force_scale, 0.0f, F32MaxPositiveNormal());
    config.point_size = F32Clamp(config.point_size, 0.0f, F32MaxPositiveNormal());
    config.joint_error_max = F32Clamp(config.joint_error_max, 0.0f, F32MaxPositiveNormal());
    config.axis_length = F32Clamp(config.axis_length, 0.0f, F32MaxPositiveNormal());
    config.refinement = U32Max(config.refinement, 1);
    config.energy_max = F32Clamp(config.energy_max, 0.0f, F32MaxPositiveNormal());
    config.shape_alpha = F32Clamp(config.shape_alpha, 0.0f, 1.0f);
    config.shape_saturation = F32Clamp(config.shape_saturation, 0.0f, 1.0f);
    config.shape_lightness = F32Clamp(config.shape_lightness, 0.0f, 1.0f);

    config.color_dbvh = ds_DynamicsDrawConfigColor(config.color_dbvh);
    config.color_sbvh = ds_DynamicsDrawConfigColor(config.color_sbvh);
    config.color_bounds = ds_DynamicsDrawConfigColor(config.color_bounds);
    config.color_mesh_bvh = ds_DynamicsDrawConfigColor(config.color_mesh_bvh);
    config.color_shape = ds_DynamicsDrawConfigColor(config.color_shape);
    config.color_static = ds_DynamicsDrawConfigColor(config.color_static);
    config.color_kinematic = ds_DynamicsDrawConfigColor(config.color_kinematic);
    config.color_dynamic = ds_DynamicsDrawConfigColor(config.color_dynamic);
    config.color_sleeping = ds_DynamicsDrawConfigColor(config.color_sleeping);
    config.color_awake = ds_DynamicsDrawConfigColor(config.color_awake);
    config.color_velocity_low = ds_DynamicsDrawConfigColor(config.color_velocity_low);
    config.color_velocity_high = ds_DynamicsDrawConfigColor(config.color_velocity_high);
    config.color_touching = ds_DynamicsDrawConfigColor(config.color_touching);
    config.color_not_touching = ds_DynamicsDrawConfigColor(config.color_not_touching);
    for (u32 i = 0; i < DRAW_POINT_COLOR_COUNT; ++i)
    {
        config.color_point[i] = ds_DynamicsDrawConfigColor(config.color_point[i]);
    }
    config.color_contact_force = ds_DynamicsDrawConfigColor(config.color_contact_force);
    config.color_friction_force = ds_DynamicsDrawConfigColor(config.color_friction_force);
    config.color_joint_force = ds_DynamicsDrawConfigColor(config.color_joint_force);
    config.color_joint = ds_DynamicsDrawConfigColor(config.color_joint);
    config.color_mouse_joint = ds_DynamicsDrawConfigColor(config.color_mouse_joint);
    config.color_joint_error = ds_DynamicsDrawConfigColor(config.color_joint_error);
    config.color_joint_break = ds_DynamicsDrawConfigColor(config.color_joint_break);
    config.color_mass_center = ds_DynamicsDrawConfigColor(config.color_mass_center);

    return config;
}

struct ds_DynamicsDrawConfig ds_DynamicsDrawConfigDefault(void)
{
    struct ds_DynamicsDrawConfig config = (struct ds_DynamicsDrawConfig)
    {
        .draw_shapes = 1,
        .shape_color_mode = DRAW_COLOR_RANDOM,
        .energy_max = 50.0f,
        .shape_alpha = 0.8f,
        .shape_saturation = 0.85f,
        .shape_lightness = 0.85f,
        .force_scale = 0.05f,
        .point_size = 6.0f,
        .draw_on_top = 1,
        .joint_error_max = 0.02f,           /* four linear slops: a held joint stays near color_joint, a failing one goes red */
        .axis_length = 0.25f,
        .refinement = 12,
        .color_dbvh = V4(0.8f, 0.1f, 0.0f, 0.6f),
        .color_sbvh = V4(0.0f, 0.8f, 0.1f, 0.6f),
        .color_bounds = V4(0.8f, 0.1f, 0.6f, 1.0f),
        .color_mesh_bvh = V4(0.0f, 0.8f, 0.1f, 0.6f),
        .color_shape = V4(0.7f, 0.7f, 0.7f, 1.0f),
        .color_static = V4(0.7f, 0.7f, 0.7f, 1.0f),
        .color_kinematic = V4(0.3f, 0.5f, 1.0f, 0.9f),
        .color_dynamic = V4(1.0f, 1.0f, 1.0f, 0.9f),
        .color_sleeping = V4(0.6f, 0.95f, 0.6f, 0.9f),
        .color_awake = V4(0.90f, 0.4f, 0.4f, 0.9f),
        .color_velocity_low = V4(0.2f, 0.2f, 1.0f, 0.9f),
        .color_velocity_high = V4(1.0f, 0.2f, 0.2f, 0.9f),
        .color_touching = V4(1.0f, 0.8f, 0.2f, 0.9f),
        .color_not_touching = V4(0.35f, 0.35f, 0.35f, 0.9f),
        /* the graph palette: saturated hues, spread so index neighbours differ; then one colour per mode */
        .color_point =
        {
            V4(1.00f, 0.00f, 0.00f, 1.0f),
            V4(0.40f, 1.00f, 0.20f, 1.0f),
            V4(0.00f, 0.50f, 1.00f, 1.0f),
            V4(1.00f, 0.20f, 0.80f, 1.0f),
            V4(1.00f, 1.00f, 0.00f, 1.0f),
            V4(0.20f, 1.00f, 0.80f, 1.0f),
            V4(0.50f, 0.00f, 1.00f, 1.0f),
            V4(1.00f, 0.40f, 0.20f, 1.0f),
            V4(0.00f, 1.00f, 0.00f, 1.0f),
            V4(0.20f, 0.40f, 1.00f, 1.0f),
            V4(1.00f, 0.00f, 0.50f, 1.0f),
            V4(0.80f, 1.00f, 0.20f, 1.0f),
            V4(0.00f, 1.00f, 1.00f, 1.0f),
            V4(0.80f, 0.20f, 1.00f, 1.0f),
            V4(1.00f, 0.50f, 0.00f, 1.0f),
            V4(0.20f, 1.00f, 0.40f, 1.0f),
            V4(0.00f, 0.00f, 1.00f, 1.0f),
            V4(1.00f, 0.20f, 0.40f, 1.0f),
            V4(0.50f, 1.00f, 0.00f, 1.0f),
            V4(0.20f, 0.80f, 1.00f, 1.0f),
            V4(1.00f, 0.00f, 1.00f, 1.0f),
            V4(1.00f, 0.80f, 0.20f, 1.0f),
            V4(0.00f, 1.00f, 0.50f, 1.0f),
            V4(0.40f, 0.20f, 1.00f, 1.0f),
            [CG_COLOR_COUNT + DRAW_POINT_UNIFORM] = V4(1.0f, 0.0f, 0.0f, 1.0f),
        },
        .color_contact_force = V4(1.0f, 0.9f, 0.1f, 1.0f),
        .color_friction_force = V4(0.6f, 1.0f, 1.0f, 1.0f),
        .color_joint_force = V4(1.0f, 0.6f, 1.0f, 1.0f),
        .color_joint = V4(0.5f, 0.8f, 1.0f, 1.0f),
        .color_mouse_joint = V4(0.8f, 0.6f, 1.0f, 1.0f),
        .color_joint_error = V4(0.8f, 0.0f, 0.0f, 1.0f),
        .color_joint_break = V4(0.45f, 0.0f, 0.05f, 1.0f),
        .color_mass_center = V4(1.0f, 1.0f, 1.0f, 1.0f),
    };

    config = ds_DynamicsDrawConfigDerive(&config);
    return config;
}

#define DS_DYNAMICS_DRAW_CONFIG_VERSION 1u
#define DS_DYNAMICS_DRAW_CONFIG_SIZE    (4 + 15*4 + 8*4 + (22 + DRAW_POINT_COLOR_COUNT)*16)

u64 ds_DynamicsDrawConfigSerializeSize(const struct ds_DynamicsDrawConfig *config)
{
    return DS_DYNAMICS_DRAW_CONFIG_SIZE;
}

void ds_DynamicsDrawConfigSerialize(struct SS *ss, const struct ds_DynamicsDrawConfig *config)
{
    ds_Assert(ds_DynamicsDrawConfigSerializeSize(config) <= SSBytesLeft(ss));

    SSWriteU32Le(ss, DS_DYNAMICS_DRAW_CONFIG_VERSION);
    SSWriteU32Le(ss, config->draw_shapes);
    SSWriteU32Le(ss, (u32) config->shape_color_mode);
    SSWriteU32Le(ss, config->draw_dbvh);
    SSWriteU32Le(ss, config->draw_sbvh);
    SSWriteU32Le(ss, config->draw_bounds);
    SSWriteU32Le(ss, config->draw_mesh_bvh);
    SSWriteU32Le(ss, config->draw_constraint_points);
    SSWriteU32Le(ss, config->draw_contact_forces);
    SSWriteU32Le(ss, config->draw_joints);
    SSWriteU32Le(ss, (u32) config->point_color_mode);
    SSWriteU32Le(ss, config->draw_on_top);
    SSWriteU32Le(ss, config->draw_body_axes);
    SSWriteU32Le(ss, config->draw_joint_forces);
    SSWriteU32Le(ss, config->draw_joint_breakage);
    SSWriteF32Le(ss, config->force_scale);
    SSWriteF32Le(ss, config->point_size);
    SSWriteF32Le(ss, config->joint_error_max);
    SSWriteF32Le(ss, config->axis_length);
    SSWriteU32Le(ss, config->refinement);
    SSWriteF32Le(ss, config->energy_max);
    SSWriteF32Le(ss, config->shape_alpha);
    SSWriteF32Le(ss, config->shape_saturation);
    SSWriteF32Le(ss, config->shape_lightness);
    SSWriteF32LeN(ss, config->color_dbvh.buf, 4);
    SSWriteF32LeN(ss, config->color_sbvh.buf, 4);
    SSWriteF32LeN(ss, config->color_bounds.buf, 4);
    SSWriteF32LeN(ss, config->color_mesh_bvh.buf, 4);
    SSWriteF32LeN(ss, config->color_shape.buf, 4);
    SSWriteF32LeN(ss, config->color_static.buf, 4);
    SSWriteF32LeN(ss, config->color_kinematic.buf, 4);
    SSWriteF32LeN(ss, config->color_dynamic.buf, 4);
    SSWriteF32LeN(ss, config->color_sleeping.buf, 4);
    SSWriteF32LeN(ss, config->color_awake.buf, 4);
    SSWriteF32LeN(ss, config->color_velocity_low.buf, 4);
    SSWriteF32LeN(ss, config->color_velocity_high.buf, 4);
    SSWriteF32LeN(ss, config->color_touching.buf, 4);
    SSWriteF32LeN(ss, config->color_not_touching.buf, 4);
    SSWriteF32LeN(ss, config->color_point[0].buf, 4*DRAW_POINT_COLOR_COUNT);
    SSWriteF32LeN(ss, config->color_contact_force.buf, 4);
    SSWriteF32LeN(ss, config->color_friction_force.buf, 4);
    SSWriteF32LeN(ss, config->color_joint_force.buf, 4);
    SSWriteF32LeN(ss, config->color_joint.buf, 4);
    SSWriteF32LeN(ss, config->color_mouse_joint.buf, 4);
    SSWriteF32LeN(ss, config->color_joint_error.buf, 4);
    SSWriteF32LeN(ss, config->color_joint_break.buf, 4);
    SSWriteF32LeN(ss, config->color_mass_center.buf, 4);
}

u32 ds_DynamicsDrawConfigTryDeserialize(struct SS *ss, struct ds_DynamicsDrawConfig *config)
{
    struct SS ss_mut = *ss;
    *config = (struct ds_DynamicsDrawConfig) { 0 };
    if (SSBytesLeft(&ss_mut) < DS_DYNAMICS_DRAW_CONFIG_SIZE || SSReadU32Le(&ss_mut) != DS_DYNAMICS_DRAW_CONFIG_VERSION)
    {
        return 0;
    }

    config->draw_shapes = SSReadU32Le(&ss_mut);
    config->shape_color_mode = (enum ds_ShapeColorMode) SSReadU32Le(&ss_mut);
    config->draw_dbvh = SSReadU32Le(&ss_mut);
    config->draw_sbvh = SSReadU32Le(&ss_mut);
    config->draw_bounds = SSReadU32Le(&ss_mut);
    config->draw_mesh_bvh = SSReadU32Le(&ss_mut);
    config->draw_constraint_points = SSReadU32Le(&ss_mut);
    config->draw_contact_forces = SSReadU32Le(&ss_mut);
    config->draw_joints = SSReadU32Le(&ss_mut);
    config->point_color_mode = (enum ds_PointColorMode) SSReadU32Le(&ss_mut);
    config->draw_on_top = SSReadU32Le(&ss_mut);
    config->draw_body_axes = SSReadU32Le(&ss_mut);
    config->draw_joint_forces = SSReadU32Le(&ss_mut);
    config->draw_joint_breakage = SSReadU32Le(&ss_mut);
    config->force_scale = SSReadF32Le(&ss_mut);
    config->point_size = SSReadF32Le(&ss_mut);
    config->joint_error_max = SSReadF32Le(&ss_mut);
    config->axis_length = SSReadF32Le(&ss_mut);
    config->refinement = SSReadU32Le(&ss_mut);
    config->energy_max = SSReadF32Le(&ss_mut);
    config->shape_alpha = SSReadF32Le(&ss_mut);
    config->shape_saturation = SSReadF32Le(&ss_mut);
    config->shape_lightness = SSReadF32Le(&ss_mut);
    SSReadF32LeN(&ss_mut, config->color_dbvh.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_sbvh.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_bounds.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_mesh_bvh.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_shape.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_static.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_kinematic.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_dynamic.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_sleeping.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_awake.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_velocity_low.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_velocity_high.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_touching.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_not_touching.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_point[0].buf, 4*DRAW_POINT_COLOR_COUNT);
    SSReadF32LeN(&ss_mut, config->color_contact_force.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_friction_force.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_joint_force.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_joint.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_mouse_joint.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_joint_error.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_joint_break.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_mass_center.buf, 4);
    *config = ds_DynamicsDrawConfigDerive(config);
    *ss = ss_mut;
    return 1;
}

/* color mixed towards color_joint_break by the larger of force / force_threshold and torque / torque_threshold (0 for F32_INFINITY) */
static v4 JointBreakageColor(const struct ds_DynamicsDrawConfig *config, const v4 color, const struct ds_JointSim *jsim, const f32 force, const f32 torque)
{
    const f32 ratio = F32Max(force / jsim->force_threshold, torque / jsim->torque_threshold);
    return V4AddScaled(color, V4Sub(config->color_joint_break, color), F32Clamp(ratio, 0.0f, 1.0f));
}

static void ds_ForceDrawPush(ds_CPool(ds_ForceDraw) *pool, const v3f64 origin, const v3 force, const v4 color)
{
    if (V3Dot(force, force) > 0.0f)
    {
        const struct ds_ForceDraw record =
        {
            .origin = origin,
            .force = force,
            .color = color,
        };
        ds_CPoolPushValue(*pool, record);
    }
}

/* through the body: its set and sim are current after the sleep pass, the joint sim's indices are not */
static const struct ds_BodySim *JointBodySim(const struct ds_Dynamics *pipeline, const struct ds_JointSim *jsim, const u32 b)
{
    const struct ds_Joint *joint = pipeline->joint_pool.buf + jsim->joint;
    const struct ds_Body *body = pipeline->body_pool.buf + joint->body[b];
    return pipeline->solver_set_pool.buf[body->set].body_sim_pool.buf + body->sim;
}

/* the anchor at the step's start */
static v3f64 JointAnchorStart(const struct ds_BodySim *sim, const struct ds_JointSim *jsim, const u32 b)
{
    const v3f64 com = (jsim->body_sim[b] != ACTIVE_BODY_DUMMY_INDEX) ? sim->world_center_of_mass : ds_BodySimWorldCenterOfMass(sim);
    return V3F64Add(com, V3Promote(jsim->r[b]));
}

/* the anchor at the end pose */
static v3f64 JointAnchorEnd(const struct ds_BodySim *sim, const struct ds_JointSim *jsim, const u32 b)
{
    return V3F64Add(sim->world.position, V3Promote(QUnitRotate(sim->world.rotation, jsim->local_frame[b].position)));
}

void ds_ContactDrawRange(struct ds_Dynamics *pipeline, const u32 color_index, const u32 low, const u32 high)
{
    struct ds_DynamicsDraw *draw = &g_dynamics_worker[ds_ThreadSelfIndex()].draw;
    const struct ds_DynamicsDrawConfig *config = &pipeline->runtime.draw_config;
    const struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    const struct ds_CGraphColor *color = pipeline->cgraph.color + color_index;
    const f32 timestep_inv = 1.0f / pipeline->timestep;
    const v4 point_color = config->color_point[(config->point_color_mode == DRAW_POINT_GRAPH_COLOR) ? color_index : CG_COLOR_COUNT + config->point_color_mode];

    for (u32 ci = low; ci < high; ++ci)
    {
        const struct ds_ContactCompute *ccomp = color->contact_compute_pool.buf + ci;
        /* slot 0 is never static (the prepare's canonical slot); the start centre plus the start lever is the manifold's point */
        const struct ds_BodySim *sim = active->body_sim_pool.buf + ccomp->body_sim[0];

        for (u32 cci = 0; cci < ccomp->cc_count; ++cci)
        {
            const struct ds_ContactConstraint *cc = ccomp->cc + cci;
            for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
            {
                const struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;
                const v3f64 point = V3F64Add(sim->world_center_of_mass, V3Promote(ccp->r[0]));

                if (config->draw_constraint_points)
                {
                    const struct r_PointWorld record = { .p = point, .color = point_color };
                    ds_CPoolPushValue(draw->point, record);
                }

                if (config->draw_contact_forces)
                {
                    /* the solver applies -impulse to body 0; friction per tangent */
                    ds_ForceDrawPush(&draw->force, point, V3Scale(cc->normal, -ccp->normal_impulse * timestep_inv), config->color_contact_force);
                    ds_ForceDrawPush(&draw->force, point, V3Scale(cc->tangent[0], -ccp->tangent_impulse[0] * timestep_inv), config->color_friction_force);
                    ds_ForceDrawPush(&draw->force, point, V3Scale(cc->tangent[1], -ccp->tangent_impulse[1] * timestep_inv), config->color_friction_force);
                }
            }
        }
    }
}

/* the segment between the end anchors: color_joint inside the range, blending to color_joint_error at joint_error_max outside it */
static void ds_DistanceJointDraw(struct ds_Dynamics *pipeline, const struct ds_JointSim *jsim, const v4 point_color)
{
    struct ds_DynamicsDraw *draw = &pipeline->runtime.draw;
    const struct ds_DynamicsDrawConfig *config = &pipeline->runtime.draw_config;
    const struct ds_DistanceJoint *dj = &jsim->distance;
    const struct ds_BodySim *sim[2] = { JointBodySim(pipeline, jsim, 0), JointBodySim(pipeline, jsim, 1) };
    /* the force on body 1; body 0 gets the opposite */
    const v3 force = V3Scale(dj->n, (dj->impulse + dj->impulse_low - dj->impulse_high) / pipeline->timestep);

    for (u32 b = 0; b < 2; ++b)
    {
        const v3f64 anchor = JointAnchorStart(sim[b], jsim, b);
        if (config->draw_constraint_points)
        {
            const struct r_PointWorld record = { .p = anchor, .color = point_color };
            ds_CPoolPushValue(draw->point, record);
        }

        if (config->draw_joint_forces && jsim->body_sim[b] != ACTIVE_BODY_DUMMY_INDEX)
        {
            ds_ForceDrawPush(&draw->force, anchor, (b == 0) ? V3Negate(force) : force, config->color_joint_force);
        }
    }

    if (config->draw_joints)
    {
        const v3f64 anchor[2] = { JointAnchorEnd(sim[0], jsim, 0), JointAnchorEnd(sim[1], jsim, 1) };
        const intv range = dj->range;
        const f32 distance = V3Length(V3F64Demote(V3F64Sub(anchor[1], anchor[0])));
        const f32 error = F32Abs(distance - F32Clamp(distance, range.low, range.high));
        const f32 t = F32Clamp(error / config->joint_error_max, 0.0f, 1.0f);
        const struct r_SegmentWorld segment =
        {
            .p0 = anchor[0],
            .p1 = anchor[1],
            .color = config->draw_joint_breakage
                ? JointBreakageColor(config, config->color_joint, jsim, V3Length(force), 0.0f)
                : V4AddScaled(config->color_joint, V4Sub(config->color_joint_error, config->color_joint), t),
        };
        ds_CPoolPushValue(draw->segment, segment);
    }
}

/* body 1 is the shapeless mouse anchor: no point there; the segment body anchor to target in color_mouse_joint */
static void ds_MouseJointDraw(struct ds_Dynamics *pipeline, const struct ds_JointSim *jsim, const v4 point_color)
{
    struct ds_DynamicsDraw *draw = &pipeline->runtime.draw;
    const struct ds_DynamicsDrawConfig *config = &pipeline->runtime.draw_config;
    const struct ds_BodySim *sim = JointBodySim(pipeline, jsim, 0);
    const v3f64 anchor_start = JointAnchorStart(sim, jsim, 0);
    /* the solver applies -impulse to body 0 */
    const v3 force = V3Scale(jsim->mouse.impulse, 1.0f / pipeline->timestep);

    if (config->draw_constraint_points)
    {
        const struct r_PointWorld record = { .p = anchor_start, .color = point_color };
        ds_CPoolPushValue(draw->point, record);
    }

    if (config->draw_joint_forces)
    {
        ds_ForceDrawPush(&draw->force, anchor_start, V3Negate(force), config->color_joint_force);
    }

    if (config->draw_joints)
    {
        const struct r_SegmentWorld segment =
        {
            .p0 = JointAnchorEnd(sim, jsim, 0),
            .p1 = jsim->mouse.target,
            .color = config->draw_joint_breakage
                ? JointBreakageColor(config, config->color_mouse_joint, jsim, V3Length(force), V3Length(jsim->mouse.angular_impulse) / pipeline->timestep)
                : config->color_mouse_joint,
        };
        ds_CPoolPushValue(draw->segment, segment);
    }
}

static void ds_JointDrawAll(struct ds_Dynamics *pipeline)
{
    const struct ds_DynamicsDrawConfig *config = &pipeline->runtime.draw_config;

    for (u32 color_index = 0; color_index < CG_COLOR_COUNT; ++color_index)
    {
        const struct ds_CGraphColor *color = pipeline->cgraph.color + color_index;
        const v4 point_color = config->color_point[(config->point_color_mode == DRAW_POINT_GRAPH_COLOR) ? color_index : CG_COLOR_COUNT + config->point_color_mode];
        for (u32 ji = 0; ji < color->joint_sim_pool.count; ++ji)
        {
            const struct ds_JointSim *jsim = color->joint_sim_pool.buf + ji;
            switch (jsim->type)
            {
                case DS_JOINT_TYPE_DISTANCE:    { ds_DistanceJointDraw(pipeline, jsim, point_color); } break;
                case DS_JOINT_TYPE_MOUSE:       { ds_MouseJointDraw(pipeline, jsim, point_color); } break;
                default: break;
            }
        }
    }
}

void ds_BodyDrawRange(struct ds_Dynamics *pipeline, const u32 low, const u32 high)
{
    struct ds_DynamicsDraw *draw = &g_dynamics_worker[ds_ThreadSelfIndex()].draw;
    const struct ds_DynamicsDrawConfig *config = &pipeline->runtime.draw_config;
    const struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    const v4 axis_color[3] = { V4(1.0f, 0.0f, 0.0f, 1.0f), V4(0.0f, 1.0f, 0.0f, 1.0f), V4(0.0f, 0.0f, 1.0f, 1.0f) };
    const v3 axis[3] = { V3(1.0f, 0.0f, 0.0f), V3(0.0f, 1.0f, 0.0f), V3(0.0f, 0.0f, 1.0f) };

    for (u32 i = low; i < high; ++i)
    {
        const struct ds_BodySim *sim = active->body_sim_pool.buf + i;
        const v3f64 com = ds_BodySimWorldCenterOfMass(sim);

        const struct r_PointWorld point = { .p = com, .color = config->color_mass_center };
        ds_CPoolPushValue(draw->point, point);

        for (u32 a = 0; a < 3; ++a)
        {
            const struct r_SegmentWorld segment =
            {
                .p0 = com,
                .p1 = V3F64Add(com, V3Promote(V3Scale(QUnitRotate(sim->world.rotation, axis[a]), config->axis_length))),
                .color = axis_color[a],
            };
            ds_CPoolPushValue(draw->segment, segment);
        }
    }
}

void ds_DynamicsDrawFrameEnd(struct ds_Dynamics *pipeline)
{
    const struct ds_DynamicsDrawConfig *config = &pipeline->runtime.draw_config;
    if (config->draw_constraint_points || config->draw_joint_forces || config->draw_joints)
    {
        ds_JointDrawAll(pipeline);
    }
}
