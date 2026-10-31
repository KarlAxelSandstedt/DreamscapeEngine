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

struct ds_DynamicsDrawConfig ds_DynamicsDrawConfigDefault(void)
{
    return (struct ds_DynamicsDrawConfig)
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
        .color_mass_center = V4(1.0f, 1.0f, 1.0f, 1.0f),
    };
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
    const struct ds_DynamicsDrawConfig *config = &pipeline->draw_config;
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
    struct ds_DynamicsDraw *draw = &pipeline->draw;
    const struct ds_DynamicsDrawConfig *config = &pipeline->draw_config;
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
            .color = V4AddScaled(config->color_joint, V4Sub(config->color_joint_error, config->color_joint), t),
        };
        ds_CPoolPushValue(draw->segment, segment);
    }
}

/* body 1 is the shapeless mouse anchor: no point there; the segment body anchor to target in color_mouse_joint */
static void ds_MouseJointDraw(struct ds_Dynamics *pipeline, const struct ds_JointSim *jsim, const v4 point_color)
{
    struct ds_DynamicsDraw *draw = &pipeline->draw;
    const struct ds_DynamicsDrawConfig *config = &pipeline->draw_config;
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
            .color = config->color_mouse_joint,
        };
        ds_CPoolPushValue(draw->segment, segment);
    }
}

static void ds_JointDrawAll(struct ds_Dynamics *pipeline)
{
    const struct ds_DynamicsDrawConfig *config = &pipeline->draw_config;

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
    const struct ds_DynamicsDrawConfig *config = &pipeline->draw_config;
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
    const struct ds_DynamicsDrawConfig *config = &pipeline->draw_config;
    if (config->draw_constraint_points || config->draw_joint_forces || config->draw_joints)
    {
        ds_JointDrawAll(pipeline);
    }
}
