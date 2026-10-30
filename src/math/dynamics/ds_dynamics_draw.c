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

void ds_ForceDrawVelocityStartRange(v3 *velocity_start, const struct ds_Dynamics *pipeline, const u32 low, const u32 high)
{
    const struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    for (u32 i = low; i < high; ++i)
    {
        velocity_start[i] = active->body_compute_pool.buf[i].linear_velocity;
    }
}

void ds_ForceDrawRange(struct ds_Dynamics *pipeline, const v3 *velocity_start, const u32 low, const u32 high)
{
    ds_CPool(ds_ForceDraw) *pool = &g_dynamics_worker[ds_ThreadSelfIndex()].draw.force;
    const struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    const f32 timestep_inv = 1.0f / pipeline->timestep;
    for (u32 i = low; i < high; ++i)
    {
        const struct ds_BodySim *sim = active->body_sim_pool.buf + i;
        const struct ds_BodyCompute *bcomp = active->body_compute_pool.buf + i;

        /* f = m dv / dt; the active set holds dynamic bodies only, so the mass is finite */
        const v3 force = V3Scale(V3Sub(bcomp->linear_velocity, velocity_start[i]), timestep_inv / sim->inv_mass);
        const f32 magnitude = V3Length(force);
        if (magnitude > 0.0f)
        {
            const struct ds_ForceDraw record =
            {
                .origin = V3F64Add(sim->world_center_of_mass, V3Promote(bcomp->delta_position)),
                .direction = V3Scale(force, 1.0f / magnitude),
                .magnitude = magnitude,
            };
            ds_CPoolPushValue(*pool, record);
        }
    }
}
