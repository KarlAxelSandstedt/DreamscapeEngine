/*
==========================================================================
    Copyright (C) 2025, 2026 Axel Sandstedt 

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

#include <stdlib.h>

#include "collision.h"
#include "ds_float.h"
#include "ds_vector.h"
#include "ds_quaternion.h"
#include "ds_matrix.h"
#include "ds_math_bridge.h"

struct solverConfig config_storage = { 0 };
struct solverConfig *g_solver_config = &config_storage;

static ds_ThreadLocal struct ds_BodyCompute tl_static_bcomp = 
{
    .linear_velocity = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
    .angular_velocity = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
    .center_of_mass = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
    .rotation = { .x = 0.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f },
    .flags = 0,
};

static ds_ThreadLocal struct ds_BodySim tl_static_body_sim = 
{
    .body = 0,
    .flags = 0,
    .inv_mass = 0,
    .world.position = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
    .world.rotation = { .x = 0.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f },
    .local_center_of_mass = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
    .world_center_of_mass = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
    .local_inv_inertia = { .buf = { 0.0f } },
    .world_inv_inertia = { .buf = { 0.0f } },
};


void SolverConfigInit(const u32 pgs_iteration_count, const u32 ngs_iteration_count, const u32 warmup_solver, const vec3 gravity, const f32 baumgarte_constant, const f32 max_linear_correction, const f32 max_linear_velocity_magnitude, const f32 max_angular_velocity_magnitude, const f32 linear_dampening, const f32 angular_dampening, const f32 linear_slop, const f32 restitution_threshold, const u32 sleep_enabled, const f32 sleep_time_threshold, const f32 sleep_linear_velocity_sq_limit, const f32 sleep_angular_velocity_sq_limit)
{
	ds_Assert(pgs_iteration_count >= 1);
	ds_Assert(ngs_iteration_count >= 1);

	g_solver_config->pgs_iteration_count = pgs_iteration_count;
	g_solver_config->ngs_iteration_count = ngs_iteration_count;
	g_solver_config->warmup_solver = warmup_solver;
	V3Store(g_solver_config->gravity, V3Load(gravity));
	g_solver_config->baumgarte_constant = baumgarte_constant;
	g_solver_config->max_linear_correction = max_linear_correction;
    g_solver_config->max_linear_velocity_magnitude_inv = (0.0f == max_linear_velocity_magnitude)
                                                        ? F32_INFINITY
                                                        : 1.0f / max_linear_velocity_magnitude;
    g_solver_config->max_angular_velocity_magnitude_inv = (0.0f == max_angular_velocity_magnitude)
                                                        ? F32_INFINITY
                                                        : 1.0f / max_angular_velocity_magnitude;
	g_solver_config->linear_dampening = linear_dampening;
	g_solver_config->angular_dampening = angular_dampening;
	g_solver_config->linear_slop = linear_slop;
	g_solver_config->restitution_threshold = restitution_threshold;

 	g_solver_config->sleep_enabled = sleep_enabled;
	g_solver_config->sleep_time_threshold = sleep_time_threshold;
	g_solver_config->sleep_linear_velocity_sq_limit = sleep_linear_velocity_sq_limit;
	g_solver_config->sleep_angular_velocity_sq_limit = sleep_angular_velocity_sq_limit;

	g_solver_config->pending_warmup_solver = g_solver_config->warmup_solver;
	g_solver_config->pending_sleep_enabled = g_solver_config->sleep_enabled;
	g_solver_config->pending_pgs_iteration_count = g_solver_config->pgs_iteration_count;
	g_solver_config->pending_ngs_iteration_count = g_solver_config->ngs_iteration_count;
	g_solver_config->pending_linear_slop = g_solver_config->linear_slop;
	g_solver_config->pending_baumgarte_constant = g_solver_config->baumgarte_constant;
	g_solver_config->pending_restitution_threshold = g_solver_config->restitution_threshold;
	g_solver_config->pending_linear_dampening = g_solver_config->linear_dampening;
	g_solver_config->pending_angular_dampening = g_solver_config->angular_dampening;
}

void ds_BodyUpdateSolverDataRange(struct ds_Dynamics *pipeline, const u32 low, const u32 high)
{
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    
    /* Apply dampening: 
	 *		dv/dt = -d*v
	 *	=>	d/dt[ve^(d*t)] = 0
	 *	=>	v(t) = v(0)*e^(-d*t)
	 *
	 *	approx e^(-d*t) = 1 - d*t + d^2*t^2 / 2! - ....
	 *	using Pade P^0_1 =>
	 *		1 - d*t = a0 / (1 + b1*t)
	 *		b0 = 1
	 *		a0 = c0 = 1
	 *		0 = a1 = c1 + c0*b1
	 *	=>	0 = b1 - d
	 *	=>	
	 *		e^(-d*t) ~= P^0_1(t) 
	 *			  =  a0 / (b0 + b1*t) 
	 *			  =  1 / (1 + d*t)
	 */
	const f32 linear_damp = 1.0f / (1.0f + g_solver_config->linear_dampening * pipeline->timestep);
	const f32 angular_damp = 1.0f / (1.0f + g_solver_config->angular_dampening * pipeline->timestep);

    for (u32 i = low; i < high; ++i)
    {
        struct ds_BodySim *sim = active->body_sim_pool.buf + i;
        struct ds_BodyCompute *bcomp = active->body_compute_pool.buf + i;

		/* setup inverted world inertia tensors and center of massses */
		const m3 rot = M3Q(sim->world.rotation);
		const m3 rot_inv = M3Transpose(rot);
        sim->world_inv_inertia = M3Mul(M3Mul(rot, sim->local_inv_inertia), rot_inv);

        bcomp->rotation = sim->world.rotation;
        bcomp->center_of_mass = V3Add(M3V3Mul(rot, sim->local_center_of_mass), sim->world.position);

        /* integrate new velocities using external forces */
        const v3 linear_velocity = V3AddScaled(bcomp->linear_velocity, V3Load(g_solver_config->gravity), pipeline->timestep);
		bcomp->linear_velocity = V3Scale(linear_velocity, linear_damp);
		bcomp->angular_velocity = V3Scale(bcomp->angular_velocity, angular_damp);
    }

    ProfZoneEnd;
}

void ds_BodyIntegrateVelocitiesRange(struct ds_Dynamics *pipeline, const u32 low, const u32 high)
{
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;

    for (u32 i = low; i < high; ++i)
    {
        struct ds_BodyCompute *bcomp = active->body_compute_pool.buf + i;

        /* update velocity and world center of mass */
        const v3 linear_velocity = bcomp->linear_velocity;
        const v3 angular_velocity = bcomp->angular_velocity;
        const f32 div_linear = V3Length(linear_velocity) * g_solver_config->max_linear_velocity_magnitude_inv;
        const f32 div_angular = V3Length(angular_velocity) * g_solver_config->max_angular_velocity_magnitude_inv;
        const f32 t_linear = 1.0f / F32Clamp(div_linear, 1.0f, F32_INFINITY);
        const f32 t_angular = 1.0f / F32Clamp(div_angular, 1.0f, F32_INFINITY);

	    bcomp->center_of_mass = V3AddScaled(bcomp->center_of_mass, linear_velocity, pipeline->timestep * t_linear);

        const q rotation = bcomp->rotation;
        const q a_vel_quat = Q(angular_velocity.x * t_angular,
                               angular_velocity.y * t_angular,
                               angular_velocity.z * t_angular,
                               0.0f);
	    const q rot_delta = QScale(QMul(a_vel_quat, rotation), pipeline->timestep / 2.0f);
	    bcomp->rotation = QNormalize(QAdd(rotation, rot_delta));
    }

    ProfZoneEnd;
}

void ds_BodyUpdateOrientationRange(struct ds_Dynamics *pipeline, struct ds_ProxyRange *proxy_range, const u32 low, const u32 high)
{
    ProfZone;

    struct arena *frame = pipeline->worker[ds_ThreadSelfIndex()].frame;
    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    struct ds_Shape *shape = NULL;

    struct memArray arr = ArenaPushAlignedAll(frame, sizeof(struct ds_ProxyDirty), 8);
    proxy_range->reinsert_count = 0;
    proxy_range->count = 0;
    proxy_range->proxy = arr.addr;
    struct ds_ProxyDirty *dirty = proxy_range->proxy + 0;

    for (u32 i = low; i < high; ++i)
    {
        struct ds_BodySim *sim = active->body_sim_pool.buf + i;
        const struct ds_BodyCompute *bcomp = active->body_compute_pool.buf + i;
        const struct ds_Body *body = pipeline->body_pool.buf + sim->body;
    
        /* derive new world transform from updated angle and world center of mass */
        const q rotation = bcomp->rotation;
        const v3 rotated_local_center_of_mass = QVec3Rotate(rotation, sim->local_center_of_mass);
        sim->world.position = V3Sub(bcomp->center_of_mass, rotated_local_center_of_mass);
        sim->world.rotation = rotation;

        for (u32 j = body->shape_list.first; (i32) j != DLL_SENTINEL; j = shape->body_shape.next)
        {
            shape = pipeline->shape_pool.buf + j;
            dirty->bbox = ds_ShapeWorldBbox(pipeline, shape);
            struct bvhNode *node = pipeline->dynamic_bvh.pool.buf + shape->proxy;
            struct aabb *proxy = &node->bbox;
            
            if (!AabbContains(proxy, &dirty->bbox))
            {
                dirty->shape = j;
                dirty->reinsert = 1;

                if (!ds_BTRootCheck(node))
                {
                    const u32 parent_index = node->bt_parent & BT_INDEX_MASK;
                    const struct bvhNode *parent = pipeline->dynamic_bvh.pool.buf + parent_index;
                    struct aabb enlarged_proxy;
                    AabbUnion(&enlarged_proxy, proxy, &dirty->bbox);
                    if (AabbContains(&parent->bbox, &enlarged_proxy))
                    {
                        *proxy = enlarged_proxy;
                        dirty->reinsert = 0;
                    }
                }

                if (dirty->reinsert)
                {
                    proxy_range->reinsert_count += 1;
                    dirty->bbox.hw = V3AddConstant(dirty->bbox.hw, shape->margin);
                }

                proxy_range->count += 1;
                dirty += 1;

                if (proxy_range->count >= arr.len)
                {
		            Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
		            FatalCleanupAndExit();
                }
            }
        }
    }

    ArenaPopPacked(frame, sizeof(struct ds_ProxyDirty)*(arr.len - proxy_range->count));
     
    ProfZoneEnd;
}

void ds_ContactConstraintInitRange(struct ds_Dynamics *pipeline, const u32 color_index, const u32 low, const u32 high)
{
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    struct ds_CGraph *cg = &pipeline->cgraph;
    struct arena *frame = pipeline->worker[ds_ThreadSelfIndex()].frame;

    struct ds_CGraphColor *color = cg->color + color_index;
    for (u32 ci = low; ci < high; ++ci)
	{			
        const struct ds_Contact *c = pipeline->contact_pool.buf + color->contact_pool.buf[ci];
        struct ds_ContactCompute *ccomp = color->contact_compute_pool.buf + ci;

	    struct ds_Body *b[2];
        struct ds_Shape *s[2];
        ds_ContactKeyAddress(b+0, s+0, b+1, s+1, pipeline, c->key);

        ccomp->body_sim[0] = ds_BodyDynamicCheck(b[0])
                ? b[0]->sim
                : ACTIVE_BODY_DUMMY_INDEX;
        ccomp->body_sim[1] = ds_BodyDynamicCheck(b[1])
                ? b[1]->sim
                : ACTIVE_BODY_DUMMY_INDEX;

        struct ds_BodySim *sim[2] =
        {
            ((ccomp->body_sim[0] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_body_sim : active->body_sim_pool.buf + ccomp->body_sim[0]),
            ((ccomp->body_sim[1] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_body_sim : active->body_sim_pool.buf + ccomp->body_sim[1]),       
        };

        struct ds_BodyCompute *bcomp[2] =
        {
            ((ccomp->body_sim[0] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_bcomp : active->body_compute_pool.buf + ccomp->body_sim[0]),
            ((ccomp->body_sim[1] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_bcomp : active->body_compute_pool.buf + ccomp->body_sim[1]),
        };

        ccomp->cc_count = c->narrowphase.manifold_count;
        ccomp->cc = ArenaPushPacked(frame, ccomp->cc_count*sizeof(struct ds_ContactConstraint));
        if (!ccomp->cc)
        {
		    Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
		    FatalCleanupAndExit();
        }

        for (u32 cci = 0; cci < ccomp->cc_count; ++cci)
        {
            const struct c_Manifold *m = (c->narrowphase.tri)
                                       ? c->narrowphase.manifold + c->narrowphase.tri_manifold[cci]
                                       : c->narrowphase.manifold + cci;

    	    struct ds_ContactConstraint *cc = ccomp->cc + cci;

		    cc->restitution = F32Max(s[0]->restitution, s[1]->restitution);
		    cc->friction = F32Sqrt(s[0]->friction*s[1]->friction);
            cc->normal = V3Load(m->n);
		    V3CreateBasis(&cc->tangent[0], &cc->tangent[1], cc->normal);

		    cc->ccp_count = m->v_count;
            for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
		    {
		    	struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;
		    	ccp->normal_impulse = 0.0f;
		    	ccp->tangent_impulse[0] = 0.0f;
		    	ccp->tangent_impulse[1] = 0.0f;

                ccp->v = V3Load(m->v[ccpi]);
		    	ccp->r[0] = V3Sub(ccp->v, bcomp[0]->center_of_mass);
		    	ccp->r[1] = V3AddScaled(V3Sub(ccp->v, bcomp[1]->center_of_mass), V3Load(m->n), -m->depth[ccpi]);

                /*
                 * Currently, we use a sentinel with COM = origin for static bodies. This becomes problematic
                 * for large levers r[0]/r[1]. Consider the case when the static body is the reference; we check
                 * the new r[0]=ccp->v, against the cached lever. For large enough r[0], it will always hold
                 * that |r[0] - r[0]_cached|^2 = 0.0f <= limit_sq, so we will continue alias the old contact despite
                 * moving far away from it.
                 */
                ds_Assert(V3Dot(ccp->v, ccp->v) < 10000.0f*10000.0f);

                const m3 inv_inertia0 = sim[0]->world_inv_inertia;
                const m3 inv_inertia1 = sim[1]->world_inv_inertia;

		    	v3 ccp_c = V3Cross(ccp->r[0], cc->normal);	/* (r1 x n) */
		    	v3 ccp_Ic = M3V3Mul(inv_inertia0, ccp_c);	/* Inw(I_1)(r1 x n) */
		    	ccp->normal_mass = sim[0]->inv_mass + V3Dot(ccp_Ic, ccp_c);

		    	v3 tmp1 = V3Cross(ccp->r[0], cc->tangent[0]);
		    	v3 tmp3 = V3Cross(ccp->r[0], cc->tangent[1]);
		    	v3 tmp2 = M3V3Mul(inv_inertia0, tmp1);
		    	v3 tmp4 = M3V3Mul(inv_inertia0, tmp3);
		    	ccp->tangent_mass[0] = sim[0]->inv_mass + V3Dot(tmp1, tmp2);
		    	ccp->tangent_mass[1] = sim[0]->inv_mass + V3Dot(tmp3, tmp4);

		    	ccp_c = V3Cross(ccp->r[1], cc->normal);
		    	ccp_Ic = M3V3Mul(inv_inertia1, ccp_c);
		    	ccp->normal_mass += sim[1]->inv_mass + V3Dot(ccp_Ic, ccp_c);

		    	tmp1 = V3Cross(ccp->r[1], cc->tangent[0]);
		    	tmp3 = V3Cross(ccp->r[1], cc->tangent[1]);
		    	tmp2 = M3V3Mul(inv_inertia1, tmp1);
		    	tmp4 = M3V3Mul(inv_inertia1, tmp3);
		    	ccp->tangent_mass[0] += sim[1]->inv_mass + V3Dot(tmp1, tmp2);
		    	ccp->tangent_mass[1] += sim[1]->inv_mass + V3Dot(tmp3, tmp4);

		    	ccp->normal_mass = 1.0f / ccp->normal_mass;
		    	ccp->tangent_mass[0] = 1.0f / ccp->tangent_mass[0];
		    	ccp->tangent_mass[1] = 1.0f / ccp->tangent_mass[1];

		    	/* TODO: This will run immediately again on the first iteration of the solver,
		    	 * could somehow remove it here, but would make stuff more complex than needed
		    	 * at this current point. */
		    	v3 relative_velocity = V3Sub(bcomp[1]->linear_velocity, bcomp[0]->linear_velocity);
		    	relative_velocity = V3Add(relative_velocity, V3Cross(bcomp[1]->angular_velocity, ccp->r[1]));
		    	relative_velocity = V3Sub(relative_velocity, V3Cross(bcomp[0]->angular_velocity, ccp->r[0]));
		    	const f32 separating_velocity = V3Dot(cc->normal, relative_velocity);

		    	/* if sufficiently fast collision happening, so apply the restitution effect */
		    	ccp->velocity_bias = (separating_velocity < -g_solver_config->restitution_threshold)
                    ? -separating_velocity * cc->restitution
                    : 0.0f;
		    }
        }
	}
    
    ProfZoneEnd;
}

void ds_ContactConstraintWarmupRange(struct ds_Dynamics *pipeline, const u32 color_index, const u32 low, const u32 high)
{
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    struct ds_CGraph *cg = &pipeline->cgraph;
    struct ds_CGraphColor *color = cg->color + color_index;
    for (u32 ci = low; ci < high; ++ci)
	{			
        const struct ds_Contact *c = pipeline->contact_pool.buf + color->contact_pool.buf[ci];
        struct ds_ContactCompute *ccomp = color->contact_compute_pool.buf + ci;

        struct ds_BodySim *sim[2] =
        {
            ((ccomp->body_sim[0] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_body_sim : active->body_sim_pool.buf + ccomp->body_sim[0]),
            ((ccomp->body_sim[1] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_body_sim : active->body_sim_pool.buf + ccomp->body_sim[1]),       
        };

        struct ds_BodyCompute *bcomp[2] =
        {
            ((ccomp->body_sim[0] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_bcomp : active->body_compute_pool.buf + ccomp->body_sim[0]),
            ((ccomp->body_sim[1] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_bcomp : active->body_compute_pool.buf + ccomp->body_sim[1]),
        };

        for (u32 cci = 0; cci < ccomp->cc_count; ++cci)
        {
    	    struct ds_ContactConstraint *cc = ccomp->cc + cci;
            struct ds_ContactConstraintCache *ccache = ccomp->ccache;
            struct c_Manifold *m = c->narrowphase.manifold;
            if (c->narrowphase.tri)
            {
                ccache = NULL;
                for (u32 t = 0; t < ccomp->ccache_count; ++t)
                {
                    if (c->narrowphase.tri[cci] <= ccomp->ccache[t].tri)
                    {
                        if (c->narrowphase.tri[cci] == ccomp->ccache[t].tri)
                        {
                            ccache = ccomp->ccache + t;
                            m = c->narrowphase.manifold + (c->narrowphase.tri_manifold[cci]);
                        }

                        break;
                    }
                }
            }
            
            if (ccache)
            {
                /*
                 * If the cached contact's normal differ to musch from current, evict whole cache. 
                 * Note that we do not reuse the contact normal, but reuse cached r1, r2.
                 */
                if (V3Dot(V3Load(m->n), ccache->normal) < 0.9f)
                {
                    continue;
                }

                const q body0_inverse_rotation = QInverse(sim[0]->world.rotation);

                for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
                {
                	struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;
                    const v3 r = QVec3Rotate(body0_inverse_rotation, ccp->r[0]);

                    //TODO Make this test better
	            	u32 best = U32_MAX;
	            	f32 closest_dist_sq = 0.01f * 0.01f;
	            	for (u32 k = 0; k < ccache->v_count; ++k)
	            	{
	            		const v3 diff = V3Sub(r, ccache->r1[k]);
	            		const f32 dist_sq = V3Dot(diff, diff);
	            		if (dist_sq < closest_dist_sq)
	            		{
	            			best = k;
	            			closest_dist_sq = dist_sq;
	            		}
	            	}

	            	if (best != U32_MAX)
	            	{
	            		const v3 old_tangent_impulse = V3AddScaled(V3Scale(ccache->tangent[0], ccache->tangent_impulse[best][0]),
	            		                                           ccache->tangent[1], ccache->tangent_impulse[best][1]);

	            		ccp->normal_impulse = ccache->normal_impulse[best];
	                    const f32 impulse_bound = cc->friction * ccp->normal_impulse;
	            		ccp->tangent_impulse[0] = V3Dot(cc->tangent[0], old_tangent_impulse);
	            		ccp->tangent_impulse[1] = V3Dot(cc->tangent[1], old_tangent_impulse);
	            		ccp->tangent_impulse[0] = F32Clamp(ccp->tangent_impulse[0], -impulse_bound, impulse_bound);
	            		ccp->tangent_impulse[1] = F32Clamp(ccp->tangent_impulse[1], -impulse_bound, impulse_bound);

	            		v3 total_cached_impulse = V3Scale(cc->normal, ccp->normal_impulse);
	            		total_cached_impulse = V3AddScaled(total_cached_impulse, cc->tangent[0], ccp->tangent_impulse[0]);
	            		total_cached_impulse = V3AddScaled(total_cached_impulse, cc->tangent[1], ccp->tangent_impulse[1]);

	            		bcomp[0]->linear_velocity = V3AddScaled(bcomp[0]->linear_velocity, total_cached_impulse, -sim[0]->inv_mass);
	            		bcomp[1]->linear_velocity = V3AddScaled(bcomp[1]->linear_velocity, total_cached_impulse, sim[1]->inv_mass);

                        ccp->r[0] = QVec3Rotate(sim[0]->world.rotation, ccache->r1[best]);
                        ccp->r[1] = QVec3Rotate(sim[1]->world.rotation, ccache->r2[best]);

	            		const v3 delta_w0 = M3V3Mul(sim[0]->world_inv_inertia, V3Cross(ccp->r[0], total_cached_impulse));
	            		bcomp[0]->angular_velocity = V3Sub(bcomp[0]->angular_velocity, delta_w0);

	            		const v3 delta_w1 = M3V3Mul(sim[1]->world_inv_inertia, V3Cross(ccp->r[1], total_cached_impulse));
	            		bcomp[1]->angular_velocity = V3Add(bcomp[1]->angular_velocity, delta_w1);
	            	}
                }
            }
        }
    }
    
    ProfZoneEnd;
}

void ds_ContactConstraintIterateRange(struct ds_Dynamics *pipeline, const u32 color_index, const u32 cc_low, const u32 cc_high)
{
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    struct ds_CGraphColor *color = pipeline->cgraph.color + color_index;

    for (u32 ci = cc_low; ci < cc_high; ++ci)
	{			
	    struct ds_ContactCompute *ccomp = color->contact_compute_pool.buf + ci;

        struct ds_BodySim *sim[2] =
        {
            ((ccomp->body_sim[0] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_body_sim : active->body_sim_pool.buf + ccomp->body_sim[0]),
            ((ccomp->body_sim[1] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_body_sim : active->body_sim_pool.buf + ccomp->body_sim[1]),       
        };

        struct ds_BodyCompute *bcomp[2] =
        {
            ((ccomp->body_sim[0] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_bcomp : active->body_compute_pool.buf + ccomp->body_sim[0]),
            ((ccomp->body_sim[1] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_bcomp : active->body_compute_pool.buf + ccomp->body_sim[1]),
        };

		/* solve friction constraints first, since normal constraints are more important */
        for (u32 cci = 0; cci < ccomp->cc_count; ++cci)
        {
            struct ds_ContactConstraint *cc = ccomp->cc + cci;
            for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
		    {
                struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;
		    	const f32 impulse_bound = cc->friction * ccp->normal_impulse;
		    	for (u32 k = 0; k < 2; ++k)
		    	{
		    	    /* Calculate separating velocity at point: JV */
		    	    v3 relative_velocity = V3Sub(bcomp[1]->linear_velocity, bcomp[0]->linear_velocity);
		    	    relative_velocity = V3Add(relative_velocity, V3Cross(bcomp[1]->angular_velocity, ccp->r[1]));
		    	    relative_velocity = V3Sub(relative_velocity, V3Cross(bcomp[0]->angular_velocity, ccp->r[0]));
		    	    const f32 separating_velocity = V3Dot(cc->tangent[k], relative_velocity);

		    	    /* update constraint point tangent impulse */
		    	    f32 delta_impulse = -ccp->tangent_mass[k] * separating_velocity;
		    	    const f32 old_impulse = ccp->tangent_impulse[k];
		    	    ccp->tangent_impulse[k] = F32Clamp(ccp->tangent_impulse[k] + delta_impulse, -impulse_bound, impulse_bound);
		    	    delta_impulse = ccp->tangent_impulse[k] - old_impulse;

		    	    /* update body velocities */
		    	    const v3 impulse = V3Scale(cc->tangent[k], delta_impulse);

		    	    const v3 delta_w0 = M3V3Mul(sim[0]->world_inv_inertia, V3Cross(ccp->r[0], impulse));
		    	    bcomp[0]->linear_velocity = V3AddScaled(bcomp[0]->linear_velocity, impulse, -sim[0]->inv_mass);
		    	    bcomp[0]->angular_velocity = V3Sub(bcomp[0]->angular_velocity, delta_w0);

		    	    const v3 delta_w1 = M3V3Mul(sim[1]->world_inv_inertia, V3Cross(ccp->r[1], impulse));
		    	    bcomp[1]->linear_velocity = V3AddScaled(bcomp[1]->linear_velocity, impulse, sim[1]->inv_mass);
		    	    bcomp[1]->angular_velocity = V3Add(bcomp[1]->angular_velocity, delta_w1);
                }
		    }

            for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
		    {
                struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;

		    	/* Calculate separating velocity at point: JV */
		    	v3 relative_velocity = V3Sub(bcomp[1]->linear_velocity, bcomp[0]->linear_velocity);
		    	relative_velocity = V3Add(relative_velocity, V3Cross(bcomp[1]->angular_velocity, ccp->r[1]));
		    	relative_velocity = V3Sub(relative_velocity, V3Cross(bcomp[0]->angular_velocity, ccp->r[0]));
		    	const f32 separating_velocity = V3Dot(cc->normal, relative_velocity);

		    	/* update constraint point normal impulse */
		    	f32 delta_impulse = ccp->normal_mass * (ccp->velocity_bias - separating_velocity);
		    	const f32 old_impulse = ccp->normal_impulse;
		    	ccp->normal_impulse = F32Max(0.0f, ccp->normal_impulse + delta_impulse);
		    	delta_impulse = ccp->normal_impulse - old_impulse;

		    	/* update body velocities */
		    	const v3 impulse = V3Scale(cc->normal, delta_impulse);

		    	const v3 delta_w0 = M3V3Mul(sim[0]->world_inv_inertia, V3Cross(ccp->r[0], impulse));
		    	bcomp[0]->linear_velocity = V3AddScaled(bcomp[0]->linear_velocity, impulse, -sim[0]->inv_mass);
		    	bcomp[0]->angular_velocity = V3Sub(bcomp[0]->angular_velocity, delta_w0);

		    	const v3 delta_w1 = M3V3Mul(sim[1]->world_inv_inertia, V3Cross(ccp->r[1], impulse));
		    	bcomp[1]->linear_velocity = V3AddScaled(bcomp[1]->linear_velocity, impulse, sim[1]->inv_mass);
		    	bcomp[1]->angular_velocity = V3Add(bcomp[1]->angular_velocity, delta_w1);
            }
        }
    }

    ProfZoneEnd;
}

void ds_PositionConstraintInitAndCacheImpulsesRange(struct ds_Dynamics *pipeline, const u32 color_index, const u32 low, const u32 high)
{
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    struct ds_CGraph *cg = &pipeline->cgraph;
    struct ds_CGraphColor *color = cg->color + color_index;
    struct arena *frame = pipeline->worker[ds_ThreadSelfIndex()].frame;

    for (u32 ci = low; ci < high; ++ci)
    {			
        struct ds_Contact *c = pipeline->contact_pool.buf + color->contact_pool.buf[ci];
        struct ds_ContactCompute *ccomp = color->contact_compute_pool.buf + ci;
        struct ds_BodySim *sim[2] =
        {
            ((ccomp->body_sim[0] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_body_sim : active->body_sim_pool.buf + ccomp->body_sim[0]),
            ((ccomp->body_sim[1] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_body_sim : active->body_sim_pool.buf + ccomp->body_sim[1]),       
        };

        struct ds_BodyCompute *bcomp[2] =
        {
            ((ccomp->body_sim[0] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_bcomp : active->body_compute_pool.buf + ccomp->body_sim[0]),
            ((ccomp->body_sim[1] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_bcomp : active->body_compute_pool.buf + ccomp->body_sim[1]),
        };

        ccomp->ccache_count = ccomp->cc_count;
        ccomp->ccache = ArenaPushPacked(frame, ccomp->ccache_count*sizeof(struct ds_ContactConstraintCache));
        if (!ccomp->ccache)
        {
		    Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
		    FatalCleanupAndExit();
        }

        for (u32 cci = 0; cci < ccomp->cc_count; ++cci)
        {
            struct ds_ContactConstraint *cc = ccomp->cc + cci;
            struct ds_ContactConstraintCache *ccache = ccomp->ccache + cci;

            ccache->tri = (c->narrowphase.tri)
                        ? c->narrowphase.tri[cci] 
                        : 0;

		    ccache->v_count = cc->ccp_count;
		    ccache->normal = cc->normal;
		    ccache->tangent[0] = cc->tangent[0];
		    ccache->tangent[1] = cc->tangent[1];

            const q sim_inv_rotation[2] =
            {
                QInverse(sim[0]->world.rotation),
                QInverse(sim[1]->world.rotation),
            };
            const q bcomp_inv_rotation[2] =
            {
                QInverse(bcomp[0]->rotation),
                QInverse(bcomp[1]->rotation),
            };
		    for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
		    {
                /* Cache */
                struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;
		    	ccache->r1[ccpi] = QVec3Rotate(sim_inv_rotation[0], ccp->r[0]);
		    	ccache->r2[ccpi] = QVec3Rotate(sim_inv_rotation[1], ccp->r[1]);
		    	ccache->normal_impulse[ccpi] = ccp->normal_impulse;
		    	ccache->tangent_impulse[ccpi][0] = ccp->tangent_impulse[0];
		    	ccache->tangent_impulse[ccpi][1] = ccp->tangent_impulse[1];

                /* Init Position */
                ccp->r[0] = QVec3Rotate(bcomp_inv_rotation[0], ccp->r[0]);
                ccp->r[1] = QVec3Rotate(bcomp_inv_rotation[1], ccp->r[1]);
            }
        }
    }

    ProfZoneEnd;
} 

void ds_PositionConstraintIterateRange(struct ds_Dynamics *pipeline, const u32 color_index, const u32 low, const u32 high)
{    
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    struct ds_CGraphColor *color = pipeline->cgraph.color + color_index;

    f32 min_separation = -F32_INFINITY;
    for (u32 ci = low; ci < high; ++ci)
	{			
	    struct ds_ContactCompute *ccomp = color->contact_compute_pool.buf + ci;

        struct ds_BodySim *sim[2] =
        {
            ((ccomp->body_sim[0] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_body_sim : active->body_sim_pool.buf + ccomp->body_sim[0]),
            ((ccomp->body_sim[1] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_body_sim : active->body_sim_pool.buf + ccomp->body_sim[1]),       
        };

        struct ds_BodyCompute *bcomp[2] =
        {
            ((ccomp->body_sim[0] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_bcomp : active->body_compute_pool.buf + ccomp->body_sim[0]),
            ((ccomp->body_sim[1] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_bcomp : active->body_compute_pool.buf + ccomp->body_sim[1]),
        };

        for (u32 cci = 0; cci < ccomp->cc_count; ++cci)
        {
            struct ds_ContactConstraint *cc = ccomp->cc + cci;
            for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
	        {
	        	struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;

		        const m3 rot0 = M3Q(bcomp[0]->rotation);
		        const m3 inv_inertia0 = M3Mul(M3Mul(rot0, sim[0]->local_inv_inertia), M3Transpose(rot0));
		        sim[0]->world_inv_inertia = inv_inertia0;

		        const m3 rot1 = M3Q(bcomp[1]->rotation);
		        const m3 inv_inertia1 = M3Mul(M3Mul(rot1, sim[1]->local_inv_inertia), M3Transpose(rot1));
		        sim[1]->world_inv_inertia = inv_inertia1;

                const v3 r0 = QVec3Rotate(bcomp[0]->rotation, ccp->r[0]);
                const v3 r1 = QVec3Rotate(bcomp[1]->rotation, ccp->r[1]);

		    	const v3 rn0 = V3Cross(r0, cc->normal);
		    	const v3 rn1 = V3Cross(r1, cc->normal);

                /* inverse effective mass? */
                const f32 K = sim[0]->inv_mass + sim[1]->inv_mass + V3Dot(M3V3Mul(inv_inertia0, rn0), rn0) + V3Dot(M3V3Mul(inv_inertia1, rn1), rn1);

                /* constraint */
                const v3 p0 = V3Add(r0, bcomp[0]->center_of_mass);
                const v3 p1 = V3Add(r1, bcomp[1]->center_of_mass);
                const f32 distance = V3Dot(p1, cc->normal) - V3Dot(p0, cc->normal);
                min_separation = F32Max(min_separation, distance);
                const f32 biased_slop_distance = g_solver_config->baumgarte_constant * (distance + g_solver_config->linear_slop);

                const f32 C = F32Clamp(biased_slop_distance, -g_solver_config->max_linear_correction, 0.0f);

                const f32 impulse = (K > 0.0f)
                    ? -C/K
                    : 0.0f;

                const v3 impulse_vector = V3Scale(cc->normal, impulse);
                bcomp[0]->center_of_mass = V3AddScaled(bcomp[0]->center_of_mass, impulse_vector, -sim[0]->inv_mass);
                bcomp[1]->center_of_mass = V3AddScaled(bcomp[1]->center_of_mass, impulse_vector,  sim[1]->inv_mass);
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
        }
    }

    ProfZoneEnd;
}
