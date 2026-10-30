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

            /* the mouse joint's body 1 is the shapeless mouse anchor; its lever is never set */
            const u32 anchor_count = (jsim->type == DS_JOINT_TYPE_MOUSE) ? 1 : 2;
            for (u32 b = 0; b < anchor_count; ++b)
            {
                v3f64 anchor;
                if (jsim->body_sim[b] != ACTIVE_BODY_DUMMY_INDEX)
                {
                    const struct ds_BodySim *sim = active->body_sim_pool.buf + jsim->body_sim[b];
                    const struct ds_BodyCompute *bcomp = active->body_compute_pool.buf + jsim->body_sim[b];
                    anchor = V3F64Add(V3F64Add(sim->world_center_of_mass, V3Promote(bcomp->delta_position)), V3Promote(jsim->r[b]));
                    if (config->draw_joint_forces)
                    {
                        ForceDrawPush(&draw->force, anchor, (b == 0) ? V3Negate(force) : force, config->color_joint_force);
                    }
                }
                else
                {
                    /* a static body: its sim is in its own set and never moves */
                    const struct ds_Joint *joint = pipeline->joint_pool.buf + jsim->joint;
                    const struct ds_Body *body = pipeline->body_pool.buf + joint->body[b];
                    const struct ds_BodySim *sim = pipeline->solver_set_pool.buf[body->set].body_sim_pool.buf + body->sim;
                    anchor = V3F64Add(ds_BodySimWorldCenterOfMass(sim), V3Promote(jsim->r[b]));
                }

                if (config->draw_constraint_points)
                {
                    const struct r_PointWorld record = { .p = anchor, .color = point_color };
                    ds_CPoolPushValue(draw->primitive.point, record);
                }
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
