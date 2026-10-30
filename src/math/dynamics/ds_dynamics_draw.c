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
    const v4 point_color = PointColor(config, color_index, config->color_contact_point);

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

                if (config->draw_contact_points)
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

void ds_JointForceDrawPush(struct ds_Dynamics *pipeline)
{
    ds_CPool(ds_ForceDraw) *pool = &pipeline->draw.force;
    const struct ds_DynamicsDrawConfig *config = &pipeline->draw_config;
    const struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    const f32 timestep_inv = 1.0f / pipeline->timestep;

    for (u32 color_index = 0; color_index < CG_COLOR_COUNT; ++color_index)
    {
        const struct ds_CGraphColor *color = pipeline->cgraph.color + color_index;
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

            for (u32 b = 0; b < 2; ++b)
            {
                if (jsim->body_sim[b] == ACTIVE_BODY_DUMMY_INDEX)
                {
                    continue;
                }

                const struct ds_BodySim *sim = active->body_sim_pool.buf + jsim->body_sim[b];
                const struct ds_BodyCompute *bcomp = active->body_compute_pool.buf + jsim->body_sim[b];
                const v3f64 anchor = V3F64Add(V3F64Add(sim->world_center_of_mass, V3Promote(bcomp->delta_position)), V3Promote(jsim->r[b]));
                ForceDrawPush(pool, anchor, (b == 0) ? V3Negate(force) : force, config->color_joint_force);
            }
        }
    }
}
