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

struct solverConfig config_storage = { 0 };
struct solverConfig *g_solver_config = &config_storage;

static ds_ThreadLocal struct ds_RigidBodyCompute tl_static_body = 
{
    .linear_velocity = { 0, 0, 0 },
    .angular_velocity = { 0, 0, 0 },
    .center_of_mass = { 0, 0, 0, },
    .rotation = { 0, 0, 0, 1 },
    .flags = 0,
};


void SolverConfigInit(const u32 pgs_iteration_count, const u32 ngs_iteration_count, const u32 warmup_solver, const vec3 gravity, const f32 baumgarte_constant, const f32 max_linear_correction, const f32 max_linear_velocity_magnitude, const f32 max_angular_velocity_magnitude, const f32 linear_dampening, const f32 angular_dampening, const f32 linear_slop, const f32 restitution_threshold, const u32 sleep_enabled, const f32 sleep_time_threshold, const f32 sleep_linear_velocity_sq_limit, const f32 sleep_angular_velocity_sq_limit)
{
	ds_Assert(pgs_iteration_count >= 1);
	ds_Assert(ngs_iteration_count >= 1);

	g_solver_config->pgs_iteration_count = pgs_iteration_count;
	g_solver_config->ngs_iteration_count = ngs_iteration_count;
	g_solver_config->warmup_solver = warmup_solver;
	Vec3Copy(g_solver_config->gravity, gravity);
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

void ds_RigidBodyUpdateSolverDataAll(struct ds_RigidBodyPipeline *pipeline)
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

    mat3 tmp, rot, rot_inv;
    for (u32 i = 1; i < active->body_sim_pool.count; ++i)
    {
        struct ds_RigidBodySim *sim = active->body_sim_pool.buf + i;
        struct ds_RigidBodyCompute *compute = active->body_compute_pool.buf + i;

		/* setup inverted world inertia tensors and center of massses */
		Mat3Quat(rot, sim->world.rotation);
		Mat3Transpose(rot_inv, rot);
        Mat3Mul(tmp, rot, sim->local_inv_inertia);
        Mat3Mul(sim->world_inv_inertia, tmp, rot_inv);

        QuatCopy(compute->rotation, sim->world.rotation);
        Mat3VecMul(compute->center_of_mass, rot, sim->local_center_of_mass);
        Vec3Translate(compute->center_of_mass, sim->world.position);

        /* integrate new velocities using external forces */
		Vec3TranslateScaled(compute->linear_velocity, g_solver_config->gravity, pipeline->timestep);
		Vec3ScaleSelf(compute->linear_velocity, linear_damp);
		Vec3ScaleSelf(compute->angular_velocity, angular_damp);
    }

    ProfZoneEnd;
}

void ds_RigidBodyIntegrateVelocitiesAll(struct ds_RigidBodyPipeline *pipeline)
{
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;

    for (u32 i = 1; i < active->body_sim_pool.count; ++i)
    {
        struct ds_RigidBodyCompute *compute = active->body_compute_pool.buf + i;

        /* update velocity and world center of mass */
        const f32 div_linear = Vec3Length(compute->linear_velocity) * g_solver_config->max_linear_velocity_magnitude_inv;
        const f32 div_angular = Vec3Length(compute->angular_velocity) * g_solver_config->max_angular_velocity_magnitude_inv;
        const f32 t_linear = 1.0f / f32_clamp(div_linear, 1.0f, F32_INFINITY);
        const f32 t_angular = 1.0f / f32_clamp(div_angular, 1.0f, F32_INFINITY);

	    Vec3TranslateScaled(compute->center_of_mass, compute->linear_velocity, pipeline->timestep * t_linear);	

        quat a_vel_quat, rot_delta;
	    QuatSet(a_vel_quat, 
	    		compute->angular_velocity[0] * t_angular, 
	    		compute->angular_velocity[1] * t_angular, 
	    		compute->angular_velocity[2] * t_angular,
	    	      	0.0f);
	    QuatMul(rot_delta, a_vel_quat, compute->rotation);
	    QuatScale(rot_delta, pipeline->timestep / 2.0f);
	    QuatTranslate(compute->rotation, rot_delta);
	    QuatNormalize(compute->rotation);
    }

    ProfZoneEnd;
}

void ds_RigidBodyUpdateOrientationAll(struct ds_RigidBodyPipeline *pipeline)
{
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;

    for (u32 i = 1; i < active->body_sim_pool.count; ++i)
    {
        struct ds_RigidBodySim *sim = active->body_sim_pool.buf + i;
        struct ds_RigidBodyCompute *compute = active->body_compute_pool.buf + i;
    
        /* derive new world transform from updated angle and world center of mass */
        vec3 rotated_local_center_of_mass;
        QuatVec3Rotate(rotated_local_center_of_mass, compute->rotation, sim->local_center_of_mass);
        Vec3Sub(sim->world.position, compute->center_of_mass, rotated_local_center_of_mass);
        QuatCopy(sim->world.rotation, compute->rotation);

        struct ds_RigidBody *body = pipeline->body_pool.buf + sim->body;
		struct ds_PhysicsEvent *event = ds_PhysicsEventPush(pipeline);
		event->type = PHYSICS_EVENT_BODY_ORIENTATION;
		event->body = body->id;
    }

    ProfZoneEnd;
}

void ds_ContactConstraintInitAll(struct ds_RigidBodyPipeline *pipeline)
{
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    struct ds_CGraph *cg = &pipeline->cgraph;

	vec3 tmp1, tmp2, tmp3, tmp4;
	vec3 ccp_Ic; 	/* Temporary storage for Inw(I_1)(r1 x n) */
	vec3 ccp_c;	    /* Temporary storage for(r1 x n) */

    for (u32 color_index = 0; color_index < CG_COLOR_COUNT; ++color_index)
    {
        struct ds_CGraphColor *color = cg->color + color_index;
        for (u32 ci = 0; ci < color->contact_constraint_pool.count; ++ci)
	    {			
            const struct ds_Contact *c = pipeline->cdb->contact_pool.buf + color->contact_pool.buf[ci];
	        struct ds_ContactConstraint *cc = color->contact_constraint_pool.buf + ci;

            {
	            struct ds_RigidBody *b[2];
                struct ds_Shape *s[2];
                ds_ContactKeyAddress(b+0, s+0, b+1, s+1, pipeline, &c->key);

                cc->body_sim[0] = RB_IS_DYNAMIC(b[0])
                    ? b[0]->sim
                    : ACTIVE_BODY_DUMMY_INDEX;
                cc->body_sim[1] = RB_IS_DYNAMIC(b[1])
                    ? b[1]->sim
                    : ACTIVE_BODY_DUMMY_INDEX;

	    	    cc->restitution = f32_max(s[0]->restitution, s[1]->restitution);
	    	    cc->friction = f32_sqrt(s[0]->friction*s[1]->friction);
            }

            struct ds_RigidBodySim *sim[2] = 
            {
                active->body_sim_pool.buf + cc->body_sim[0],
                active->body_sim_pool.buf + cc->body_sim[1],
            };

            struct ds_RigidBodyCompute *compute[2] =
            {
                active->body_compute_pool.buf + cc->body_sim[0],
                active->body_compute_pool.buf + cc->body_sim[1],
            };

            Vec3Copy(cc->normal, c->cm.n);
	    	Vec3CreateBasis(cc->tangent[0], cc->tangent[1], cc->normal);

	    	cc->ccp_count = c->cm.v_count;
            for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
	    	{
	    		struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;
	    		ccp->normal_impulse = 0.0f;
	    		ccp->tangent_impulse[0] = 0.0f;
	    		ccp->tangent_impulse[1] = 0.0f;

                Vec3Copy(ccp->v, c->cm.v[ccpi]);
	    		Vec3Sub(ccp->r[0], ccp->v, compute[0]->center_of_mass);
	    		Vec3Sub(ccp->r[1], ccp->v, compute[1]->center_of_mass);
                Vec3TranslateScaled(ccp->r[1], c->cm.n, -c->cm.depth[ccpi]);

                ds_AssertString(Vec3Dot(ccp->v, ccp->v) < 10000.0f*10000.0f,
                        "Currently, we use a sentinel with COM = origin for static bodies. This becomes problematic\
                         for large levers r[0]/r[1]. Consider the case when the static body is the reference; we check\
                         the new r[0]=ccp->v, against the cached lever. For large enough r[0], it will always hold\
                         that |r[0] - r[0]_cached|^2 = 0.0f <= limit_sq, so we will continue alias the old contact despite\
                         moving far away from it.");

	    		Vec3Cross(ccp_c, ccp->r[0], cc->normal);
	    		Mat3VecMul(ccp_Ic, sim[0]->world_inv_inertia, ccp_c);
	    		ccp->normal_mass = sim[0]->inv_mass + Vec3Dot(ccp_Ic, ccp_c);

	    		Vec3Cross(tmp1, ccp->r[0], cc->tangent[0]);
	    		Vec3Cross(tmp3, ccp->r[0], cc->tangent[1]);
	    		Mat3VecMul(tmp2, sim[0]->world_inv_inertia, tmp1);
	    		Mat3VecMul(tmp4, sim[0]->world_inv_inertia, tmp3);
	    		ccp->tangent_mass[0] = sim[0]->inv_mass + Vec3Dot(tmp1, tmp2);
	    		ccp->tangent_mass[1] = sim[0]->inv_mass + Vec3Dot(tmp3, tmp4);

	    		Vec3Cross(ccp_c, ccp->r[1], cc->normal);
	    		Mat3VecMul(ccp_Ic, sim[1]->world_inv_inertia, ccp_c);
	    		ccp->normal_mass += sim[1]->inv_mass + Vec3Dot(ccp_Ic, ccp_c);

	    		Vec3Cross(tmp1, ccp->r[1], cc->tangent[0]);
	    		Vec3Cross(tmp3, ccp->r[1], cc->tangent[1]);
	    		Mat3VecMul(tmp2, sim[1]->world_inv_inertia, tmp1);
	    		Mat3VecMul(tmp4, sim[1]->world_inv_inertia, tmp3);
	    		ccp->tangent_mass[0] += sim[1]->inv_mass + Vec3Dot(tmp1, tmp2);
	    		ccp->tangent_mass[1] += sim[1]->inv_mass + Vec3Dot(tmp3, tmp4);

	    		ccp->normal_mass = 1.0f / ccp->normal_mass;
	    		ccp->tangent_mass[0] = 1.0f / ccp->tangent_mass[0];
	    		ccp->tangent_mass[1] = 1.0f / ccp->tangent_mass[1];

	    		/* TODO: This will run immediately again on the first iteration of the solver,
	    		 * could somehow remove it here, but would make stuff more complex than needed
	    		 * at this current point. */
	    		vec3 relative_velocity;
	    		Vec3Sub(relative_velocity, 
	    				compute[1]->linear_velocity,
	    				compute[0]->linear_velocity);
	    		Vec3Cross(tmp1, compute[1]->angular_velocity, ccp->r[1]);
	    		Vec3Cross(tmp2, compute[0]->angular_velocity, ccp->r[0]);
	    		Vec3Translate(relative_velocity, tmp1);
	    		Vec3TranslateScaled(relative_velocity, tmp2, -1.0f);
	    		const f32 separating_velocity = Vec3Dot(cc->normal, relative_velocity);

	    		/* if sufficiently fast collision happening, so apply the restitution effect */
	    		ccp->velocity_bias = (separating_velocity < -g_solver_config->restitution_threshold)
                    ? -separating_velocity * cc->restitution
                    : 0.0f;
	    	}
	    }
    }
    
    ProfZoneEnd;
}

void ds_ContactConstraintWarmupAll(struct ds_RigidBodyPipeline *pipeline)
{
    ProfZone;

    quat body0_inverse_rotation;
	vec3 r, tmp1, tmp2, tmp3, old_tangent_impulse, total_cached_impulse;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    struct ds_CGraph *cg = &pipeline->cgraph;

    for (u32 color_index = 0; color_index < CG_COLOR_COUNT; ++color_index)
    {
        struct ds_CGraphColor *color = cg->color + color_index;
        for (u32 ci = 0; ci < color->contact_constraint_pool.count; ++ci)
	    {			
            const struct ds_Contact *c = pipeline->cdb->contact_pool.buf + color->contact_pool.buf[ci];
	        struct ds_ContactConstraint *cc = color->contact_constraint_pool.buf + ci;

            struct ds_RigidBodySim *sim[2] =
            {
                active->body_sim_pool.buf + cc->body_sim[0],
                active->body_sim_pool.buf + cc->body_sim[1],
            };
            struct ds_RigidBodyCompute *compute[2] =
            {
                active->body_compute_pool.buf + cc->body_sim[0],
                active->body_compute_pool.buf + cc->body_sim[1],
            };

            /*
             * If the cached contact's normal differ to musch from current, evict whole cache. 
             * Note that we do not reuse the contact normal, but reuse cached r1, r2.
             */
            if (!c->cached_count || Vec3Dot(c->cm.n, c->normal_cache) < 0.9f)
            {
                continue;
            }
        
            QuatInverse(body0_inverse_rotation, sim[0]->world.rotation);

            for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
            {
    	    	struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;
                QuatVec3Rotate(r, body0_inverse_rotation, ccp->r[0]);

                //TODO Make this test better
		    	u32 best = U32_MAX;
		    	f32 closest_dist_sq = 0.01f * 0.01f;
		    	for (u32 k = 0; k < c->cached_count; ++k)
		    	{
		    		Vec3Sub(tmp1, r, c->r1_cache[k]);
		    		const f32 dist_sq = Vec3Dot(tmp1, tmp1);
		    		if (dist_sq < closest_dist_sq)
		    		{
		    			best = k;
		    			closest_dist_sq = dist_sq;
		    		}
		    	}

		    	if (best != U32_MAX)
		    	{
		    		Vec3Scale(old_tangent_impulse, c->tangent_cache[0], c->tangent_impulse_cache[best][0]);
		    		Vec3TranslateScaled(old_tangent_impulse, c->tangent_cache[1], c->tangent_impulse_cache[best][1]);

		    		ccp->normal_impulse = c->normal_impulse_cache[best];
		            const f32 impulse_bound = cc->friction * ccp->normal_impulse;
		    		ccp->tangent_impulse[0] = Vec3Dot(cc->tangent[0], old_tangent_impulse);
		    		ccp->tangent_impulse[1] = Vec3Dot(cc->tangent[1], old_tangent_impulse);
		    		ccp->tangent_impulse[0] = f32_clamp(ccp->tangent_impulse[0], -impulse_bound, impulse_bound);
		    		ccp->tangent_impulse[1] = f32_clamp(ccp->tangent_impulse[1], -impulse_bound, impulse_bound);

		    		Vec3Scale(total_cached_impulse, cc->normal, ccp->normal_impulse);
		    		Vec3TranslateScaled(total_cached_impulse, cc->tangent[0], ccp->tangent_impulse[0]);
		    		Vec3TranslateScaled(total_cached_impulse, cc->tangent[1], ccp->tangent_impulse[1]);


		    		Vec3TranslateScaled(compute[0]->linear_velocity, total_cached_impulse, -sim[0]->inv_mass);
		    		Vec3TranslateScaled(compute[1]->linear_velocity, total_cached_impulse, sim[1]->inv_mass);

                    QuatVec3Rotate(ccp->r[0], sim[0]->world.rotation, c->r1_cache[best]);
                    QuatVec3Rotate(ccp->r[1], sim[1]->world.rotation, c->r2_cache[best]);

		    		Vec3Cross(tmp2, ccp->r[0], total_cached_impulse);
		    		Mat3VecMul(tmp3, sim[0]->world_inv_inertia, tmp2);
		    		Vec3TranslateScaled(compute[0]->angular_velocity, tmp3, -1.0f);

		    		Vec3Cross(tmp2, ccp->r[1], total_cached_impulse);
		    		Mat3VecMul(tmp3, sim[1]->world_inv_inertia, tmp2);
		    		Vec3Translate(compute[1]->angular_velocity, tmp3);
		    	}
            }
	    }
    }
    
    ProfZoneEnd;
}

void ds_ContactConstraintColorIterate(struct ds_RigidBodyPipeline *pipeline, const u32 color_index, const u32 cc_low, const u32 cc_high)
{
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    struct ds_CGraphColor *color = pipeline->cgraph.color + color_index;

	vec4 b, new_total_impulse;
	vec3 tmp1, tmp2, tmp3;
	vec3 relative_velocity;

    //TODO Cannot update static body in multithreaded environment

    for (u32 cci = cc_low; cci < cc_high; ++cci)
	{			
        const struct ds_Contact *c = pipeline->cdb->contact_pool.buf + color->contact_pool.buf[cci];
	    struct ds_ContactConstraint *cc = color->contact_constraint_pool.buf + cci;

        struct ds_RigidBodySim *sim[2] =
        {
            active->body_sim_pool.buf + cc->body_sim[0],
            active->body_sim_pool.buf + cc->body_sim[1],
        };

        struct ds_RigidBodyCompute *compute[2] =
        {
            ((cc->body_sim[0] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_body : active->body_compute_pool.buf + cc->body_sim[0]),
            ((cc->body_sim[1] == ACTIVE_BODY_DUMMY_INDEX) ? &tl_static_body : active->body_compute_pool.buf + cc->body_sim[0]),
        };

		/* solve friction constraints first, since normal constraints are more important */
        for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
		{
            struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;
			const f32 impulse_bound = cc->friction * ccp->normal_impulse;
			for (u32 k = 0; k < 2; ++k)
			{
			    /* Calculate separating velocity at point: JV */
			    Vec3Sub(relative_velocity, compute[1]->linear_velocity, compute[0]->linear_velocity);
			    Vec3Cross(tmp2, compute[1]->angular_velocity, ccp->r[1]);
			    Vec3Cross(tmp3, compute[0]->angular_velocity, ccp->r[0]);
			    Vec3Translate(relative_velocity, tmp2);
			    Vec3TranslateScaled(relative_velocity, tmp3, -1.0f);
			    const f32 separating_velocity = Vec3Dot(cc->tangent[k], relative_velocity);

			    /* update constraint point tangent impulse */
			    f32 delta_impulse = -ccp->tangent_mass[k] * separating_velocity;
			    const f32 old_impulse = ccp->tangent_impulse[k];
			    ccp->tangent_impulse[k] = f32_clamp(ccp->tangent_impulse[k] + delta_impulse, -impulse_bound, impulse_bound);
			    delta_impulse = ccp->tangent_impulse[k] - old_impulse;

			    /* update body velocities */
			    Vec3Scale(tmp1, cc->tangent[k], delta_impulse);

			    Vec3Cross(tmp2, ccp->r[0], tmp1);
			    Mat3VecMul(tmp3, sim[0]->world_inv_inertia, tmp2);
			    Vec3TranslateScaled(compute[0]->linear_velocity, tmp1, -sim[0]->inv_mass);
			    Vec3TranslateScaled(compute[0]->angular_velocity, tmp3, -1.0f);

			    Vec3Cross(tmp2, ccp->r[1], tmp1);
			    Mat3VecMul(tmp3, sim[1]->world_inv_inertia, tmp2);
			    Vec3TranslateScaled(compute[1]->linear_velocity, tmp1,  sim[1]->inv_mass);
			    Vec3Translate(compute[1]->angular_velocity, tmp3);
            }
		}

        for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
		{
            struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;

			/* Calculate separating velocity at point: JV */
			Vec3Sub(relative_velocity, compute[1]->linear_velocity, compute[0]->linear_velocity);
			Vec3Cross(tmp2, compute[1]->angular_velocity, ccp->r[1]);
			Vec3Cross(tmp3, compute[0]->angular_velocity, ccp->r[0]);
			Vec3Translate(relative_velocity, tmp2);
			Vec3TranslateScaled(relative_velocity, tmp3, -1.0f);
			const f32 separating_velocity = Vec3Dot(cc->normal, relative_velocity);

			/* update constraint point normal impulse */
			f32 delta_impulse = ccp->normal_mass * (ccp->velocity_bias - separating_velocity);
			const f32 old_impulse = ccp->normal_impulse;
			ccp->normal_impulse = f32_max(0.0f, ccp->normal_impulse + delta_impulse);
			delta_impulse = ccp->normal_impulse - old_impulse;

			/* update body velocities */
			Vec3Scale(tmp1, cc->normal, delta_impulse);

			Vec3Cross(tmp2, ccp->r[0], tmp1);
			Mat3VecMul(tmp3, sim[0]->world_inv_inertia, tmp2);
			Vec3TranslateScaled(compute[0]->linear_velocity, tmp1, -sim[0]->inv_mass);
			Vec3TranslateScaled(compute[0]->angular_velocity, tmp3, -1.0f);

			Vec3Cross(tmp2, ccp->r[1], tmp1);
			Mat3VecMul(tmp3, sim[1]->world_inv_inertia, tmp2);
			Vec3TranslateScaled(compute[1]->linear_velocity, tmp1, sim[1]->inv_mass);
			Vec3Translate(compute[1]->angular_velocity, tmp3);
        }
    }

    ProfZoneEnd;
}

void ds_ContactConstraintCacheImpulse(struct ds_RigidBodyPipeline *pipeline)
{
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    struct ds_CGraph *cg = &pipeline->cgraph;
    quat body_inv_rotation[2];

    for (u32 color_index = 0; color_index < CG_COLOR_COUNT; ++color_index)
    {
        struct ds_CGraphColor *color = cg->color + color_index;
        for (u32 ci = 0; ci < color->contact_constraint_pool.count; ++ci)
	    {			
            struct ds_Contact *c = pipeline->cdb->contact_pool.buf + color->contact_pool.buf[ci];
	        struct ds_ContactConstraint *cc = color->contact_constraint_pool.buf + ci;

            const struct ds_RigidBodySim *sim[2] =
            {
                active->body_sim_pool.buf + cc->body_sim[0],
                active->body_sim_pool.buf + cc->body_sim[1],
            };

		    c->cached_count = cc->ccp_count;
		    Vec3Copy(c->normal_cache, cc->normal);
		    Vec3Copy(c->tangent_cache[0], cc->tangent[0]);
		    Vec3Copy(c->tangent_cache[1], cc->tangent[1]);

            QuatInverse(body_inv_rotation[0], sim[0]->world.rotation);
            QuatInverse(body_inv_rotation[1], sim[1]->world.rotation);
		    for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
		    {
                struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;
		    	QuatVec3Rotate(c->r1_cache[ccpi], body_inv_rotation[0], ccp->r[0]);
		    	QuatVec3Rotate(c->r2_cache[ccpi], body_inv_rotation[1], ccp->r[1]);
		    	c->normal_impulse_cache[ccpi] = ccp->normal_impulse;
		    	c->tangent_impulse_cache[ccpi][0] = ccp->tangent_impulse[0];
		    	c->tangent_impulse_cache[ccpi][1] = ccp->tangent_impulse[1];
		    }
        }
	}

    ProfZoneEnd;
}

void ds_PositionConstraintColorInitAll(struct ds_RigidBodyPipeline *pipeline)
{
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    struct ds_CGraph *cg = &pipeline->cgraph;

    quat body_inv_rotation[2];
    vec3 tmp1, tmp2, relative_velocity;

    for (u32 color_index = 0; color_index < CG_COLOR_COUNT; ++color_index)
    {
        struct ds_CGraphColor *color = cg->color + color_index;
        for (u32 ci = 0; ci < color->contact_constraint_pool.count; ++ci)
	    {			
            const struct ds_Contact *c = pipeline->cdb->contact_pool.buf + color->contact_pool.buf[ci];
	        struct ds_ContactConstraint *cc = color->contact_constraint_pool.buf + ci;

            const struct ds_RigidBodyCompute *compute[2] =
            {
                active->body_compute_pool.buf + cc->body_sim[0],
                active->body_compute_pool.buf + cc->body_sim[1],
            };

            QuatInverse(body_inv_rotation[0], compute[0]->rotation);
            QuatInverse(body_inv_rotation[1], compute[1]->rotation);
            for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
	    	{
	    		struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;
                QuatVec3RotateSelf(ccp->r[0], body_inv_rotation[0]);
                QuatVec3RotateSelf(ccp->r[1], body_inv_rotation[1]);
            }
        }
    }

    ProfZoneEnd;
}

void ds_PositionConstraintColorIterate(struct ds_RigidBodyPipeline *pipeline, const u32 color_index)
{    
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    struct ds_CGraphColor *color = pipeline->cgraph.color + color_index;

    mat3ptr mi;
    mat3 mat_tmp, rot, rot_inv;
	vec3 diff, r[2], rn[2], tmp[2], impulse_vector;
    quat quat_tmp, quat_angle;

    f32 min_separation = -F32_INFINITY;
    for (u32 ci = 0; ci < color->contact_constraint_pool.count; ++ci)
	{			
        const struct ds_Contact *c = pipeline->cdb->contact_pool.buf + color->contact_pool.buf[ci];
	    struct ds_ContactConstraint *cc = color->contact_constraint_pool.buf + ci;

        struct ds_RigidBodySim *sim[2] =
        {
            active->body_sim_pool.buf + cc->body_sim[0],
            active->body_sim_pool.buf + cc->body_sim[1],
        };

        struct ds_RigidBodyCompute *compute[2] =
        {
            active->body_compute_pool.buf + cc->body_sim[0],
            active->body_compute_pool.buf + cc->body_sim[1],
        };

        for (u32 ccpi = 0; ccpi < cc->ccp_count; ++ccpi)
	    {
	    	struct ds_ContactConstraintPoint *ccp = cc->ccp + ccpi;

		    Mat3Quat(rot, compute[0]->rotation);
		    Mat3Transpose(rot_inv, rot);
		    Mat3Mul(mat_tmp, rot, sim[0]->local_inv_inertia);
		    Mat3Mul(sim[0]->world_inv_inertia, mat_tmp, rot_inv);

		    Mat3Quat(rot, compute[1]->rotation);
		    Mat3Transpose(rot_inv, rot);
		    Mat3Mul(mat_tmp, rot, sim[1]->local_inv_inertia);
		    Mat3Mul(sim[1]->world_inv_inertia, mat_tmp, rot_inv);

            QuatVec3Rotate(r[0], compute[0]->rotation, ccp->r[0]);
            QuatVec3Rotate(r[1], compute[1]->rotation, ccp->r[1]);

			Vec3Cross(rn[0], r[0], cc->normal);
			Vec3Cross(rn[1], r[1], cc->normal);

			Mat3VecMul(tmp[0], sim[0]->world_inv_inertia, rn[0]);
			Mat3VecMul(tmp[1], sim[1]->world_inv_inertia, rn[1]);

            /* inverse effective mass? */
            const f32 K = sim[0]->inv_mass + sim[1]->inv_mass + Vec3Dot(tmp[0], rn[0]) + Vec3Dot(tmp[1], rn[1]);

            /* constraint */
            Vec3Add(tmp[0], r[0], compute[0]->center_of_mass);
            Vec3Add(tmp[1], r[1], compute[1]->center_of_mass);
            const f32 distance = Vec3Dot(tmp[1], cc->normal) - Vec3Dot(tmp[0], cc->normal); 
            min_separation = f32_max(min_separation, distance);
            const f32 biased_slop_distance = g_solver_config->baumgarte_constant * (distance + g_solver_config->linear_slop);

            const f32 C = f32_clamp(biased_slop_distance, -g_solver_config->max_linear_correction, 0.0f);

            const f32 impulse = (K > 0.0f) 
                ? -C/K 
                : 0.0f;

            Vec3Scale(impulse_vector, cc->normal, impulse);
            Vec3TranslateScaled(compute[0]->center_of_mass, impulse_vector, -sim[0]->inv_mass);
            Vec3TranslateScaled(compute[1]->center_of_mass, impulse_vector,  sim[1]->inv_mass);
            /* flipped cross for correct sign! */
            Vec3Cross(tmp[0], impulse_vector, r[0]);
            /* instantaneous torque, assume delta_t = 1 */
            Mat3VecMul(tmp[1], sim[0]->world_inv_inertia, tmp[0]);
            /* Taylor expansion for sin, cos around 0 yields following approximation */
            QuatSet(quat_angle, tmp[1][0]/2.0f, tmp[1][1]/2.0f, tmp[1][2]/2.0f, 1.0f);
            QuatCopy(quat_tmp, compute[0]->rotation);
            QuatMul(compute[0]->rotation, quat_angle, quat_tmp);
            QuatNormalize(compute[0]->rotation);

            Vec3Cross(tmp[0], r[1], impulse_vector);
            Mat3VecMul(tmp[1], sim[1]->world_inv_inertia, tmp[0]);
            QuatSet(quat_angle, tmp[1][0]/2.0f, tmp[1][1]/2.0f, tmp[1][2]/2.0f, 1.0f);
            QuatCopy(quat_tmp, compute[1]->rotation);
            QuatMul(compute[1]->rotation, quat_angle, quat_tmp);
            QuatNormalize(compute[1]->rotation);
        }
    }

    ProfZoneEnd;
}
