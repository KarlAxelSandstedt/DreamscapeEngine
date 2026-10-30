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

/* Physics debug drawing: the pushes behind ds_DynamicsDrawConfig. */

static void ForceDrawPush(ds_CPool(ds_ForceDraw) *pool, const v3f64 origin, const v3 force, const v4 color)
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

static v4 PointColor(const struct ds_DynamicsDrawConfig *config, const u32 color_index, const v4 uniform)
{
    return (config->point_color_mode == DRAW_POINT_GRAPH_COLOR) ? config->color_graph[color_index] : uniform;
}

/* the joint frame's world position from the body's final pose (a static body through its own set) */
static v3f64 JointAnchor(const struct ds_Dynamics *pipeline, const struct ds_SolverSet *active, const struct ds_JointSim *jsim, const u32 b)
{
    const struct ds_BodySim *sim;
    if (jsim->body_sim[b] != ACTIVE_BODY_DUMMY_INDEX)
    {
        sim = active->body_sim_pool.buf + jsim->body_sim[b];
    }
    else
    {
        const struct ds_Joint *joint = pipeline->joint_pool.buf + jsim->joint;
        const struct ds_Body *body = pipeline->body_pool.buf + joint->body[b];
        sim = pipeline->solver_set_pool.buf[body->set].body_sim_pool.buf + body->sim;
    }
    return V3F64Add(sim->world.position, V3Promote(QUnitRotate(sim->world.rotation, jsim->local_frame[b].position)));
}

void ds_ContactDrawRange(struct ds_Dynamics *pipeline, const u32 color_index, const u32 low, const u32 high)
{
    struct ds_DynamicsDraw *draw = &g_dynamics_worker[ds_ThreadSelfIndex()].draw;
    const struct ds_DynamicsDrawConfig *config = &pipeline->draw_config;
    const struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    const struct ds_CGraphColor *color = pipeline->cgraph.color + color_index;
    const f32 timestep_inv = 1.0f / pipeline->timestep;
    const v4 point_color = PointColor(config, color_index, config->color_constraint_point);

    for (u32 ci = low; ci < high; ++ci)
    {
        const struct ds_ContactCompute *ccomp = color->contact_compute_pool.buf + ci;
        /* slot 0 is never static (the prepare's canonical slot) */
        const struct ds_BodySim *sim = active->body_sim_pool.buf + ccomp->body_sim[0];
        const struct ds_BodyCompute *bcomp = active->body_compute_pool.buf + ccomp->body_sim[0];
        const v3f64 com = V3F64Add(sim->world_center_of_mass, V3Promote(bcomp->delta_position));

        for (u32 cci = 0; cci < ccomp->cc_count; ++cci)
        {
            const struct ds_ContactConstraint *cc = ccomp->cc + cci;
            for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
            {
                const struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;
                const v3f64 point = V3F64Add(com, V3Promote(ccp->r[0]));

                if (config->draw_constraint_points)
                {
                    const struct r_PointWorld record = { .p = point, .color = point_color };
                    ds_CPoolPushValue(draw->primitive.point, record);
                }

                if (config->draw_contact_forces)
                {
                    /* the solver applies -impulse to body 0; friction per tangent */
                    ForceDrawPush(&draw->force, point, V3Scale(cc->normal, -ccp->normal_impulse * timestep_inv), config->color_contact_force);
                    ForceDrawPush(&draw->force, point, V3Scale(cc->tangent[0], -ccp->tangent_impulse[0] * timestep_inv), config->color_friction_force);
                    ForceDrawPush(&draw->force, point, V3Scale(cc->tangent[1], -ccp->tangent_impulse[1] * timestep_inv), config->color_friction_force);
                }
            }
        }
    }
}

void ds_JointDrawPush(struct ds_Dynamics *pipeline)
{
    struct ds_DynamicsDraw *draw = &pipeline->draw;
    const struct ds_DynamicsDrawConfig *config = &pipeline->draw_config;
    const struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    const f32 timestep_inv = 1.0f / pipeline->timestep;

    for (u32 color_index = 0; color_index < CG_COLOR_COUNT; ++color_index)
    {
        const struct ds_CGraphColor *color = pipeline->cgraph.color + color_index;
        const v4 point_color = PointColor(config, color_index, config->color_constraint_point);
        for (u32 ji = 0; ji < color->joint_sim_pool.count; ++ji)
        {
            const struct ds_JointSim *jsim = color->joint_sim_pool.buf + ji;

            /* the force on body 1; body 0 gets the opposite */
            v3 force;
            switch (jsim->type)
            {
                case DS_JOINT_TYPE_DISTANCE:
                {
                    const struct ds_DistanceJoint *dj = &jsim->distance;
                    force = V3Scale(dj->n, (dj->impulse + dj->impulse_low - dj->impulse_high) * timestep_inv);
                } break;
                case DS_JOINT_TYPE_MOUSE:
                {
                    force = V3Scale(jsim->mouse.impulse, timestep_inv);
                } break;
                default:
                {
                    force = V3Zero();
                } break;
            }

            /* the mouse joint's body 1 is the shapeless mouse anchor: its second point is the target */
            const u32 mouse = (jsim->type == DS_JOINT_TYPE_MOUSE);
            v3f64 anchor[2];
            for (u32 b = 0; b < 2; ++b)
            {
                anchor[b] = (mouse && b == 1) ? jsim->mouse.target : JointAnchor(pipeline, active, jsim, b);

                if (config->draw_constraint_points)
                {
                    const struct r_PointWorld record = { .p = anchor[b], .color = point_color };
                    ds_CPoolPushValue(draw->primitive.point, record);
                }

                if (config->draw_joint_forces && jsim->body_sim[b] != ACTIVE_BODY_DUMMY_INDEX)
                {
                    ForceDrawPush(&draw->force, anchor[b], (b == 0) ? V3Negate(force) : force, config->color_joint_force);
                }
            }

            if (config->draw_joints && (jsim->type == DS_JOINT_TYPE_DISTANCE || mouse))
            {
                /* a distance joint: the distance outside the range, as a fraction of joint_error_max, picks the colour */
                v4 color = config->color_mouse_joint;
                if (!mouse)
                {
                    const intv range = jsim->distance.range;
                    const f32 distance = V3Length(V3F64Demote(V3F64Sub(anchor[1], anchor[0])));
                    const f32 error = F32Abs(distance - F32Clamp(distance, range.low, range.high));
                    const f32 t = F32Clamp(error / config->joint_error_max, 0.0f, 1.0f);
                    color = V4AddScaled(config->color_joint, V4Sub(config->color_joint_error, config->color_joint), t);
                }

                const struct r_SegmentWorld segment =
                {
                    .p0 = anchor[0],
                    .p1 = anchor[1],
                    .color = color,
                };
                ds_CPoolPushValue(draw->primitive.segment, segment);
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
        const struct ds_BodyCompute *bcomp = active->body_compute_pool.buf + i;
        const v3f64 com = V3F64Add(sim->world_center_of_mass, V3Promote(bcomp->delta_position));

        const struct r_PointWorld point = { .p = com, .color = config->color_mass_center };
        ds_CPoolPushValue(draw->primitive.point, point);

        for (u32 a = 0; a < 3; ++a)
        {
            const struct r_SegmentWorld segment =
            {
                .p0 = com,
                .p1 = V3F64Add(com, V3Promote(V3Scale(QUnitRotate(bcomp->rotation, axis[a]), config->axis_length))),
                .color = axis_color[a],
            };
            ds_CPoolPushValue(draw->primitive.segment, segment);
        }
    }
}
