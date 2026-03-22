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
#include "dynamics.h"

/* used in contact solver to cleanup the code from if-statements */
struct ds_RigidBody static_body = { 0 };


struct solverConfig config_storage = { 0 };
struct solverConfig *g_solver_config = &config_storage;

void    SolverConfigInit(const u32 pgs_iteration_count, const u32 ngs_iteration_count, const u32 warmup_solver, const vec3 gravity, const f32 baumgarte_constant, const f32 max_linear_correction, const f32 linear_dampening, const f32 angular_dampening, const f32 linear_slop, const f32 restitution_threshold, const u32 sleep_enabled, const f32 sleep_time_threshold, const f32 sleep_linear_velocity_sq_limit, const f32 sleep_angular_velocity_sq_limit)
{
	ds_Assert(pgs_iteration_count >= 1);
	ds_Assert(ngs_iteration_count >= 1);

	g_solver_config->pgs_iteration_count = pgs_iteration_count;
	g_solver_config->ngs_iteration_count = ngs_iteration_count;
	g_solver_config->warmup_solver = warmup_solver;
	Vec3Copy(g_solver_config->gravity, gravity);
	g_solver_config->baumgarte_constant = baumgarte_constant;
	g_solver_config->max_linear_correction = max_linear_correction;
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

	static_body.mass = F32_INFINITY;
}

struct solver *SolverInitBodyData(struct arena *mem, struct ds_Island *is, const f32 timestep)
{
	struct solver *solver = ArenaPush(mem, sizeof(struct solver));

	solver->bodies = is->bodies;
	solver->timestep = timestep;
	solver->body_count = is->body_list.count;
	solver->contact_count = is->contact_list.count;

    solver->w_center_of_mass = ArenaPush(mem, (is->body_list.count + 1) * sizeof(vec3));
	solver->Iw_inv = ArenaPush(mem, (is->body_list.count + 1) * sizeof(mat3));
	solver->linear_velocity = ArenaPush(mem,  (is->body_list.count + 1) * sizeof(vec3));	/* last element is for static bodies with 0-value data */
	solver->angular_velocity = ArenaPush(mem, (is->body_list.count + 1) * sizeof(vec3));
	solver->rotation = ArenaPush(mem, (is->body_list.count + 1) * sizeof(quat));

	mat3ptr mi;
	mat3 rot, tmp1, rot_inv;

	solver->bodies[solver->body_count] = &static_body;
	Vec3Set(solver->w_center_of_mass[solver->body_count], 0.0f, 0.0f, 0.0f);
	Vec3Set(solver->linear_velocity[solver->body_count], 0.0f, 0.0f, 0.0f);
	Vec3Set(solver->angular_velocity[solver->body_count], 0.0f, 0.0f, 0.0f);
    QuatSet(solver->rotation[solver->body_count], 0.0f, 0.0f, 0.0f, 1.0f);  /* <- important for static references! */
	mi = solver->Iw_inv + solver->body_count;
	Mat3Set(*mi, 0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f);


	for (u32 i = 0; i < is->body_list.count; ++i)
	{	
		struct ds_RigidBody *b = solver->bodies[i];

		/* setup inverted world intertia tensors and center of massses */
        QuatCopy(solver->rotation[i], b->t_world.rotation);
		mat3ptr mi = solver->Iw_inv + i;
		Mat3Quat(rot, b->t_world.rotation);
		Mat3Transpose(rot_inv, rot);
		Mat3Mul(tmp1, rot, b->inv_inertia_tensor);
		Mat3Mul(*mi, tmp1, rot_inv);

        Mat3VecMul(solver->w_center_of_mass[i], rot, b->local_center_of_mass);
        Vec3Translate(solver->w_center_of_mass[i], b->t_world.position);

		/* integrate new velocities using external forces */
		Vec3Copy(solver->linear_velocity[i], b->velocity);
		Vec3Copy(solver->angular_velocity[i], b->angular_velocity);
		Vec3TranslateScaled(solver->linear_velocity[i], g_solver_config->gravity, timestep);

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
		const f32 linear_damp = 1.0f / (1.0f + g_solver_config->linear_dampening * timestep);
		const f32 angular_damp = 1.0f / (1.0f + g_solver_config->angular_dampening * timestep);
		Vec3ScaleSelf(solver->linear_velocity[i], linear_damp);
		Vec3ScaleSelf(solver->angular_velocity[i], angular_damp);
	}

	return solver;
}

void SolverInitVelocityConstraints(struct arena *mem, struct solver *solver, const struct ds_RigidBodyPipeline *pipeline, const struct ds_Island *is)
{
	solver->vcs = ArenaPush(mem, solver->contact_count * sizeof(struct velocityConstraint));

	vec3 tmp1, tmp2, tmp3, tmp4;
	vec3 vcp_Ic; 	/* Temporary storage for Inw(I_1)(r1 x n) */
	vec3 vcp_c;	    /* Temporary storage for(r1 x n) */
	for (u32 i = 0; i < solver->contact_count; ++i)
	{			
		struct velocityConstraint *vc = solver->vcs + i;
        {
		    const struct ds_RigidBody *b1 = ds_PoolAddress(&pipeline->body_pool, is->contacts[i]->key.body0);
		    const struct ds_RigidBody *b2 = ds_PoolAddress(&pipeline->body_pool, is->contacts[i]->key.body1);
		    	
            vc->lb1 = (b1->island_index == ISLAND_STATIC)
                ? solver->body_count
                : is->body_index_map[is->contacts[i]->key.body0];
            vc->lb2 = (b2->island_index == ISLAND_STATIC)
                ? solver->body_count
                : is->body_index_map[is->contacts[i]->key.body1];

            const struct ds_Shape *s1 = ds_PoolAddress(&pipeline->shape_pool, is->contacts[i]->key.shape0);
            const struct ds_Shape *s2 = ds_PoolAddress(&pipeline->shape_pool, is->contacts[i]->key.shape1);

		    vc->restitution = f32_max(s1->restitution, s2->restitution);
		    vc->friction = f32_sqrt(s1->friction*s2->friction);
        }

		mat3ptr Iw_inv1 = solver->Iw_inv + vc->lb1;
		mat3ptr Iw_inv2 = solver->Iw_inv + vc->lb2;

        Vec3Copy(vc->normal, is->contacts[i]->cm.n);
		Vec3CreateBasis(vc->tangent[0], vc->tangent[1], vc->normal);

		vc->vcp_count = is->contacts[i]->cm.v_count;
		vc->vcps = ArenaPush(mem, vc->vcp_count * sizeof(struct velocityConstraintPoint));

        for (u32 j = 0; j < vc->vcp_count; ++j)
		{
			struct velocityConstraintPoint *vcp = vc->vcps + j;
			vcp->normal_impulse = 0.0f;
			vcp->tangent_impulse[0] = 0.0f;
			vcp->tangent_impulse[1] = 0.0f;

            Vec3Copy(vcp->contact_point, is->contacts[i]->cm.v[j]);
			Vec3Sub(vcp->r1, vcp->contact_point, solver->w_center_of_mass[vc->lb1]);
			Vec3Sub(vcp->r2, vcp->contact_point, solver->w_center_of_mass[vc->lb2]);
            Vec3TranslateScaled(vcp->r2, is->contacts[i]->cm.n, -is->contacts[i]->cm.depth[j]);

            ds_AssertString(Vec3Dot(vcp->contact_point, vcp->contact_point) < 10000.0f*10000.0f,
                    "Currently, we use a sentinel with COM = origin for static bodies. This becomes problematic\
                     for large levers r1/r2. Consider the case when the static body is the reference; we check\
                     the new r1=vcp->contact_point, against the cached. for large enough r1, it will always hold\
                     that |r1 - r1_cached|^2 = 0.0f <= limit_sq, so we will continue alias the old contact despite\
                     moving far away from it.");

			Vec3Cross(vcp_c, vcp->r1, vc->normal);
			Mat3VecMul(vcp_Ic, *Iw_inv1, vcp_c);
			vcp->normal_mass = 1.0f / solver->bodies[vc->lb1]->mass + Vec3Dot(vcp_Ic, vcp_c);

			Vec3Cross(tmp1, vcp->r1, vc->tangent[0]);
			Vec3Cross(tmp3, vcp->r1, vc->tangent[1]);
			Mat3VecMul(tmp2, *Iw_inv1, tmp1);
			Mat3VecMul(tmp4, *Iw_inv1, tmp3);
			vcp->tangent_mass[0] = 1.0f / solver->bodies[vc->lb1]->mass + Vec3Dot(tmp1, tmp2);
			vcp->tangent_mass[1] = 1.0f / solver->bodies[vc->lb1]->mass + Vec3Dot(tmp3, tmp4);

			Vec3Cross(vcp_c, vcp->r2, vc->normal);
			Mat3VecMul(vcp_Ic, *Iw_inv2, vcp_c);
			vcp->normal_mass += 1.0f / solver->bodies[vc->lb2]->mass + Vec3Dot(vcp_Ic, vcp_c);

			Vec3Cross(tmp1, vcp->r2, vc->tangent[0]);
			Vec3Cross(tmp3, vcp->r2, vc->tangent[1]);
			Mat3VecMul(tmp2, *Iw_inv2, tmp1);
			Mat3VecMul(tmp4, *Iw_inv2, tmp3);
			vcp->tangent_mass[0] += 1.0f / solver->bodies[vc->lb2]->mass + Vec3Dot(tmp1, tmp2);
			vcp->tangent_mass[1] += 1.0f / solver->bodies[vc->lb2]->mass + Vec3Dot(tmp3, tmp4);

			vcp->normal_mass = 1.0f / vcp->normal_mass;
			vcp->tangent_mass[0] = 1.0f / vcp->tangent_mass[0];
			vcp->tangent_mass[1] = 1.0f / vcp->tangent_mass[1];

			/* TODO: This will run immediately again on the first iteration of the solver,
			 * could somehow remove it here, but would make stuff more complex than needed
			 * at this current point. */
			vec3 relative_velocity;
			Vec3Sub(relative_velocity, 
					solver->linear_velocity[vc->lb2],
					solver->linear_velocity[vc->lb1]);
			Vec3Cross(tmp1, solver->angular_velocity[vc->lb2], vcp->r2);
			Vec3Cross(tmp2, solver->angular_velocity[vc->lb1], vcp->r1);
			Vec3Translate(relative_velocity, tmp1);
			Vec3TranslateScaled(relative_velocity, tmp2, -1.0f);
			const f32 separating_velocity = Vec3Dot(vc->normal, relative_velocity);

			/* if sufficiently fast collision happening, so apply the restitution effect */
			vcp->velocity_bias = (separating_velocity < -g_solver_config->restitution_threshold)
                ? -separating_velocity * vc->restitution
                : 0.0f;
		}
	}
}

void SolverWarmup(struct solver *solver, const struct ds_Island *is)
{
    quat body1_inverse_rotation;
	vec3 r1, r2, tmp1, tmp2, tmp3, old_tangent_impulse, total_cached_impulse;

    for (u32 i = 0; i < solver->contact_count; ++i)
	{			
		struct ds_Contact *c = is->contacts[i];
		struct velocityConstraint *vc = solver->vcs + i;
        /* 
         * If the cached contact's normal differ to musch from current, evict whole cache. 
         * Note that we do not reuse the contact normal, but reuse cached r1, r2.
         */
        if (!c->cached_count || Vec3Dot(c->cm.n, c->normal_cache) < 0.9f)
        {
            continue;
        }
        
        QuatInverse(body1_inverse_rotation, solver->rotation[vc->lb1]);
        for (u32 j = 0; j < vc->vcp_count; ++j)
        {
    		struct velocityConstraintPoint *vcp = vc->vcps + j;
            QuatVec3Rotate(r1, body1_inverse_rotation, vcp->r1);

			u32 best = U32_MAX;
            //TODO Make this test better
			f32 closest_dist_sq = 0.01f * 0.01f;
			for (u32 k = 0; k < c->cached_count; ++k)
			{
				Vec3Sub(tmp1, r1, c->r1_cache[k]);
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

				vcp->normal_impulse = c->normal_impulse_cache[best];
		        const f32 impulse_bound = vc->friction * vcp->normal_impulse;
				vcp->tangent_impulse[0] = Vec3Dot(vc->tangent[0], old_tangent_impulse);
				vcp->tangent_impulse[1] = Vec3Dot(vc->tangent[1], old_tangent_impulse);
				vcp->tangent_impulse[0] = f32_clamp(vcp->tangent_impulse[0], -impulse_bound, impulse_bound);
				vcp->tangent_impulse[1] = f32_clamp(vcp->tangent_impulse[1], -impulse_bound, impulse_bound);

				Vec3Scale(total_cached_impulse, vc->normal, vcp->normal_impulse);
				Vec3TranslateScaled(total_cached_impulse, vc->tangent[0], vcp->tangent_impulse[0]);
				Vec3TranslateScaled(total_cached_impulse, vc->tangent[1], vcp->tangent_impulse[1]);


				Vec3TranslateScaled(solver->linear_velocity[vc->lb1], total_cached_impulse, -1.0f / solver->bodies[vc->lb1]->mass);
				Vec3TranslateScaled(solver->linear_velocity[vc->lb2], total_cached_impulse, 1.0f / solver->bodies[vc->lb2]->mass);

                QuatVec3Rotate(vcp->r1, solver->rotation[vc->lb1], c->r1_cache[best]);
                QuatVec3Rotate(vcp->r2, solver->rotation[vc->lb2], c->r2_cache[best]);

				Vec3Cross(tmp2, vcp->r1, total_cached_impulse);
				Mat3VecMul(tmp3, solver->Iw_inv[vc->lb1], tmp2);
				Vec3TranslateScaled(solver->angular_velocity[vc->lb1], tmp3, -1.0f);

				Vec3Cross(tmp2, vcp->r2, total_cached_impulse);
				Mat3VecMul(tmp3, solver->Iw_inv[vc->lb2], tmp2);
				Vec3Translate(solver->angular_velocity[vc->lb2], tmp3);
			}
        }
    }
}

void SolverCacheImpulse(struct solver *solver, const struct ds_Island *is)
{
    quat body1_inverse_rotation, body2_inverse_rotation;
	for (u32 i = 0; i < solver->contact_count; ++i)
	{			
		struct ds_Contact *c = is->contacts[i];
		struct velocityConstraint *vc = solver->vcs + i;

		c->cached_count = vc->vcp_count;
		Vec3Copy(c->normal_cache, vc->normal);
		Vec3Copy(c->tangent_cache[0], vc->tangent[0]);
		Vec3Copy(c->tangent_cache[1], vc->tangent[1]);

        QuatInverse(body1_inverse_rotation, solver->rotation[vc->lb1]);
        QuatInverse(body2_inverse_rotation, solver->rotation[vc->lb2]);
		for (u32 j = 0; j < vc->vcp_count; ++j)
		{
			QuatVec3Rotate(c->r1_cache[j], body1_inverse_rotation, vc->vcps[j].r1);
			QuatVec3Rotate(c->r2_cache[j], body2_inverse_rotation, vc->vcps[j].r2);
			c->normal_impulse_cache[j] = vc->vcps[j].normal_impulse;
			c->tangent_impulse_cache[j][0] = vc->vcps[j].tangent_impulse[0];
			c->tangent_impulse_cache[j][1] = vc->vcps[j].tangent_impulse[1];
		}
	}
}

void SolverIterateVelocityConstraints(struct solver *solver)
{
	vec4 b, new_total_impulse;
	vec3 tmp1, tmp2, tmp3;
	vec3 relative_velocity;
	for (u32 i = 0; i < solver->contact_count; ++i)
	{
		struct velocityConstraint *vc = solver->vcs + i;

		/* solve friction constraints first, since normal constraints are more important */
		for (u32 j = 0; j < vc->vcp_count; ++j)
		{
			struct velocityConstraintPoint *vcp = vc->vcps + j;
			const f32 impulse_bound = vc->friction * vcp->normal_impulse;

			for (u32 k = 0; k < 2; ++k)
			{
				/* Calculate separating velocity at point: JV */
				Vec3Sub(relative_velocity, 
						solver->linear_velocity[vc->lb2],
						solver->linear_velocity[vc->lb1]);
				Vec3Cross(tmp2, solver->angular_velocity[vc->lb2], vcp->r2);
				Vec3Cross(tmp3, solver->angular_velocity[vc->lb1], vcp->r1);
				Vec3Translate(relative_velocity, tmp2);
				Vec3TranslateScaled(relative_velocity, tmp3, -1.0f);
				const f32 separating_velocity = Vec3Dot(vc->tangent[k], relative_velocity);

				/* update constraint point tangent impulse */
				f32 delta_impulse = -vcp->tangent_mass[k] * separating_velocity;
				const f32 old_impulse = vcp->tangent_impulse[k];
				vcp->tangent_impulse[k] = f32_clamp(vcp->tangent_impulse[k] + delta_impulse, -impulse_bound, impulse_bound);
				delta_impulse = vcp->tangent_impulse[k] - old_impulse;

				/* update body velocities */
				Vec3Scale(tmp1, vc->tangent[k], delta_impulse);
				Vec3TranslateScaled(solver->linear_velocity[vc->lb1], tmp1, -1.0f / solver->bodies[vc->lb1]->mass);
				Vec3TranslateScaled(solver->linear_velocity[vc->lb2], tmp1, 1.0f / solver->bodies[vc->lb2]->mass);
				Vec3Cross(tmp2, vcp->r1, tmp1);
				Mat3VecMul(tmp3, solver->Iw_inv[vc->lb1], tmp2);
				Vec3TranslateScaled(solver->angular_velocity[vc->lb1], tmp3, -1.0f);
				Vec3Cross(tmp2, vcp->r2, tmp1);
				Mat3VecMul(tmp3, solver->Iw_inv[vc->lb2], tmp2);
				Vec3Translate(solver->angular_velocity[vc->lb2], tmp3);
			}
		}

		for (u32 j = 0; j < vc->vcp_count; ++j)
		{
			struct velocityConstraintPoint *vcp = vc->vcps + j;

			/* Calculate separating velocity at point: JV */
			Vec3Sub(relative_velocity, 
					solver->linear_velocity[vc->lb2],
					solver->linear_velocity[vc->lb1]);
			Vec3Cross(tmp2, solver->angular_velocity[vc->lb2], vcp->r2);
			Vec3Cross(tmp3, solver->angular_velocity[vc->lb1], vcp->r1);
			Vec3Translate(relative_velocity, tmp2);
			Vec3TranslateScaled(relative_velocity, tmp3, -1.0f);
			const f32 separating_velocity = Vec3Dot(vc->normal, relative_velocity);

			/* update constraint point normal impulse */
			f32 delta_impulse = vcp->normal_mass * (vcp->velocity_bias - separating_velocity);
			const f32 old_impulse = vcp->normal_impulse;
			vcp->normal_impulse = f32_max(0.0f, vcp->normal_impulse + delta_impulse);
			delta_impulse = vcp->normal_impulse - old_impulse;

			/* update body velocities */
			Vec3Scale(tmp1, vc->normal, delta_impulse);
			Vec3TranslateScaled(solver->linear_velocity[vc->lb1], tmp1, -1.0f / solver->bodies[vc->lb1]->mass);
			Vec3TranslateScaled(solver->linear_velocity[vc->lb2], tmp1, 1.0f / solver->bodies[vc->lb2]->mass);
			Vec3Cross(tmp2, vcp->r1, tmp1);
			Mat3VecMul(tmp3, solver->Iw_inv[vc->lb1], tmp2);
			Vec3TranslateScaled(solver->angular_velocity[vc->lb1], tmp3, -1.0f);
			Vec3Cross(tmp2, vcp->r2, tmp1);
			Mat3VecMul(tmp3, solver->Iw_inv[vc->lb2], tmp2);
			Vec3Translate(solver->angular_velocity[vc->lb2], tmp3);
		}
	}
}

void SolverInitPositionConstraints(struct solver *solver, const struct ds_Island *is)
{
    //vec3 tmp1, tmp2, relative_velocity;
    //for (u32 i = 0; i < solver->contact_count; ++i)
	//{
	//	struct velocityConstraint *vc = solver->vcs + i;
	//	for (u32 j = 0; j < vc->vcp_count; ++j)
	//	{
	//		struct velocityConstraintPoint *vcp = vc->vcps + j;

    //        /* Note that we use old vcp->r* values, as those are the correct applied velocities */
	//		Vec3Sub(relative_velocity, 
	//				solver->linear_velocity[vc->lb2],
	//				solver->linear_velocity[vc->lb1]);
	//		Vec3Cross(tmp1, solver->angular_velocity[vc->lb2], vcp->r2);
	//		Vec3Cross(tmp2, solver->angular_velocity[vc->lb1], vcp->r1);
	//		Vec3Translate(relative_velocity, tmp1);
	//		Vec3TranslateScaled(relative_velocity, tmp2, -1.0f);
	//		const f32 separating_velocity = Vec3Dot(vc->normal, relative_velocity);

    //        /* 
    //         * (1) If the velocity solver did not manage to make the separating velocity
    //         *     positive, the penetration has deepend to old_depth - t*sep_vel.
    //         *
    //         *              delta = (- t*sep_vel, old_depth - t*sep_vel)
    //         *
    //         * (2) If the velocity solver did manage to make the separating velocity
    //         *     positive, we get the bound of delta position correction:
    //         *               
    //         *              delta = (0.0f, old_depth)
    //         */

    //        /*
    //         * (1) new_depth = f32_max(0.0f, old_depth - delta_depth) = 
    //         * {
    //         *    delta_depth is negative, so return old_depth - delta_depth.
    //         *      (This is correct, the contact deepend, so we must adjust the position even more).
    //         * }
    //         *
    //         * (2) new_depth = f32_max(0.0f, old_depth - delta_depth) = 
    //         * {
    //         *    if velocity removed the collision all together, old_depth - delta_depth < 0 => return 0
    //         *      (The velocity removed the contact, so the position solver has nothing to do).
    //         *    if velocity did not removed the collision, old_depth - delta_depth > 0 => return old_depth
    //         *      (The velocity removed partial contact, the position solver fixes the rest of the penetration).
    //         * }
    //         */

    //        /* delta_depth : (-INF, old_depth) */
    //        const f32 delta_depth = f32_min(solver->timestep*separating_velocity, is->contacts[i]->cm.depth[j]);
    //        const f32 new_depth = is->contacts[i]->cm.depth[j] - delta_depth;
    //        vcp->target_distance = new_depth 
    //                             + Vec3Dot(solver->w_center_of_mass[vc->lb2], vc->normal) 
    //                             - Vec3Dot(solver->w_center_of_mass[vc->lb1], vc->normal); 
    //    }
    //}
}

void SolverIteratePositionConstraints(struct solver *solver)
{
    //mat3ptr mi;
    //mat3 mat_tmp, rot, rot_inv;
	//vec3 diff, r1, r2, rn1, rn2, tmp1, tmp2, impulse_vector;
    //quat quat_tmp, quat_angle;

	//for (u32 i = 0; i < solver->contact_count; ++i)
	//{
	//    struct velocityConstraint *vc = solver->vcs + i;
    //    struct ds_RigidBody *b1 = solver->bodies[vc->lb1];
    //    struct ds_RigidBody *b2 = solver->bodies[vc->lb2];

	//	for (u32 j = 0; j < vc->vcp_count; ++j)
	//	{
	//		struct velocityConstraintPoint *vcp = vc->vcps + j;

    //        mi = solver->Iw_inv + vc->lb1;
	//	    Mat3Quat(rot, solver->rotation[vc->lb1]);
	//	    Mat3Transpose(rot_inv, rot);
	//	    Mat3Mul(mat_tmp, rot, b1->inv_inertia_tensor);
	//	    Mat3Mul(*mi, mat_tmp, rot_inv);

    //        mi = solver->Iw_inv + vc->lb2;
	//	    Mat3Quat(rot, solver->rotation[vc->lb2]);
	//	    Mat3Transpose(rot_inv, rot);
	//	    Mat3Mul(mat_tmp, rot, b2->inv_inertia_tensor);
	//	    Mat3Mul(*mi, mat_tmp, rot_inv);

	//		Vec3Sub(r1, vcp->contact_point, solver->w_center_of_mass[vc->lb1]);
	//		Vec3Sub(r2, vcp->contact_point, solver->w_center_of_mass[vc->lb2]);

	//		Vec3Cross(rn1, r1, vc->normal);
	//		Vec3Cross(rn2, r2, vc->normal);

	//		Mat3VecMul(tmp1, solver->Iw_inv[vc->lb1], rn1);
	//		Mat3VecMul(tmp2, solver->Iw_inv[vc->lb2], rn2);

    //        /* inverse effective mass? */
    //        const f32 K = 1.0f/b1->mass + 1.0f/b2->mass + Vec3Dot(tmp1, rn1) + Vec3Dot(tmp2, rn2);

    //        /* constraint */
    //        const f32 distance = Vec3Dot(solver->w_center_of_mass[vc->lb2], vc->normal) - Vec3Dot(solver->w_center_of_mass[vc->lb1], vc->normal) - vcp->target_distance; 
    //        const f32 biased_slop_distance = g_solver_config->baumgarte_constant * (distance + g_solver_config->linear_slop);

    //        const f32 C = f32_clamp(biased_slop_distance, -g_solver_config->max_linear_correction, 0.0f);
    //        
    //        const f32 impulse = (K > 0.0f) 
    //            ? -C/K 
    //            : 0.0f;

    //        Vec3Scale(impulse_vector, vc->normal, impulse);
    //        Vec3TranslateScaled(solver->w_center_of_mass[vc->lb1], impulse_vector, -1.0f/b1->mass);
    //        Vec3TranslateScaled(solver->w_center_of_mass[vc->lb2], impulse_vector,  1.0f/b2->mass);

    //        /* flipped cross for correct sign! */
    //        Vec3Cross(tmp1, impulse_vector, r1);
    //        /* instantaneous torque, assume delta_t = 1 */
    //        Mat3VecMul(tmp2, solver->Iw_inv[vc->lb1], tmp1);
    //        /* Taylor expansion for sin, cos around 0 yields following approximation */
    //        QuatSet(quat_angle, tmp2[0]/2.0f, tmp2[1]/2.0f, tmp2[2]/2.0f, 1.0f);
    //        QuatCopy(quat_tmp, solver->rotation[vc->lb1]);
    //        QuatMul(solver->rotation[vc->lb1], quat_angle, quat_tmp);
    //        QuatNormalize(solver->rotation[vc->lb1]);

    //        Vec3Cross(tmp1, r2, impulse_vector);
    //        Mat3VecMul(tmp2, solver->Iw_inv[vc->lb2], tmp1);
    //        QuatSet(quat_angle, tmp2[0]/2.0f, tmp2[1]/2.0f, tmp2[2]/2.0f, 1.0f);
    //        QuatCopy(quat_tmp, solver->rotation[vc->lb2]);
    //        QuatMul(solver->rotation[vc->lb2], quat_angle, quat_tmp);
    //        QuatNormalize(solver->rotation[vc->lb2]);
    //    }
	//}
}
