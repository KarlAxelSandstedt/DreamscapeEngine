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

/* used in contact solver to cleanup the code from if-statements */
struct ds_RigidBody static_body = { 0 };


struct solverConfig config_storage = { 0 };
struct solverConfig *g_solver_config = &config_storage;

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

		/* setup inverted world inertia tensors and center of massses */
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
	        struct ds_RigidBody *b1, *b2;
            struct ds_Shape *s1, *s2;
            ds_ContactKeyAddress(&b1, &s1, &b2, &s2, pipeline, &is->contacts[i]->key);

            vc->lb1 = RB_IS_STATIC(b1)
                ? solver->body_count
                : is->body_index_map[is->contacts[i]->key.body0];
            vc->lb2 = RB_IS_STATIC(b2)
                ? solver->body_count
                : is->body_index_map[is->contacts[i]->key.body1];

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
    quat body1_inverse_rotation, body2_inverse_rotation;
    vec3 tmp1, tmp2, relative_velocity;
    for (u32 i = 0; i < solver->contact_count; ++i)
	{
		struct velocityConstraint *vc = solver->vcs + i;
        QuatInverse(body1_inverse_rotation, solver->rotation[vc->lb1]);
        QuatInverse(body2_inverse_rotation, solver->rotation[vc->lb2]);
		for (u32 j = 0; j < vc->vcp_count; ++j)
		{
			struct velocityConstraintPoint *vcp = vc->vcps + j;
            QuatVec3RotateSelf(vcp->r1, body1_inverse_rotation);
            QuatVec3RotateSelf(vcp->r2, body2_inverse_rotation);
        }
    }
}

u32 SolverIteratePositionConstraints(struct solver *solver)
{
    mat3ptr mi;
    mat3 mat_tmp, rot, rot_inv;
	vec3 diff, r1, r2, rn1, rn2, tmp1, tmp2, impulse_vector;
    quat quat_tmp, quat_angle;

    f32 min_separation = -F32_INFINITY;
	for (u32 i = 0; i < solver->contact_count; ++i)
	{
	    struct velocityConstraint *vc = solver->vcs + i;
        struct ds_RigidBody *b1 = solver->bodies[vc->lb1];
        struct ds_RigidBody *b2 = solver->bodies[vc->lb2];

		for (u32 j = 0; j < vc->vcp_count; ++j)
		{
			struct velocityConstraintPoint *vcp = vc->vcps + j;

            mi = solver->Iw_inv + vc->lb1;
		    Mat3Quat(rot, solver->rotation[vc->lb1]);
		    Mat3Transpose(rot_inv, rot);
		    Mat3Mul(mat_tmp, rot, b1->inv_inertia_tensor);
		    Mat3Mul(*mi, mat_tmp, rot_inv);

            mi = solver->Iw_inv + vc->lb2;
		    Mat3Quat(rot, solver->rotation[vc->lb2]);
		    Mat3Transpose(rot_inv, rot);
		    Mat3Mul(mat_tmp, rot, b2->inv_inertia_tensor);
		    Mat3Mul(*mi, mat_tmp, rot_inv);

            QuatVec3Rotate(r1, solver->rotation[vc->lb1], vcp->r1);
            QuatVec3Rotate(r2, solver->rotation[vc->lb2], vcp->r2);

			Vec3Cross(rn1, r1, vc->normal);
			Vec3Cross(rn2, r2, vc->normal);

			Mat3VecMul(tmp1, solver->Iw_inv[vc->lb1], rn1);
			Mat3VecMul(tmp2, solver->Iw_inv[vc->lb2], rn2);

            /* inverse effective mass? */
            const f32 K = 1.0f/b1->mass + 1.0f/b2->mass + Vec3Dot(tmp1, rn1) + Vec3Dot(tmp2, rn2);

            /* constraint */
            Vec3Add(tmp1, r1, solver->w_center_of_mass[vc->lb1]);
            Vec3Add(tmp2, r2, solver->w_center_of_mass[vc->lb2]);
            const f32 distance = Vec3Dot(tmp2, vc->normal) - Vec3Dot(tmp1, vc->normal); 
            min_separation = f32_max(min_separation, distance);
            const f32 biased_slop_distance = g_solver_config->baumgarte_constant * (distance + g_solver_config->linear_slop);

            const f32 C = f32_clamp(biased_slop_distance, -g_solver_config->max_linear_correction, 0.0f);
            
            const f32 impulse = (K > 0.0f) 
                ? -C/K 
                : 0.0f;

            Vec3Scale(impulse_vector, vc->normal, impulse);
            Vec3TranslateScaled(solver->w_center_of_mass[vc->lb1], impulse_vector, -1.0f/b1->mass);
            Vec3TranslateScaled(solver->w_center_of_mass[vc->lb2], impulse_vector,  1.0f/b2->mass);

            /* flipped cross for correct sign! */
            Vec3Cross(tmp1, impulse_vector, r1);
            /* instantaneous torque, assume delta_t = 1 */
            Mat3VecMul(tmp2, solver->Iw_inv[vc->lb1], tmp1);
            /* Taylor expansion for sin, cos around 0 yields following approximation */
            QuatSet(quat_angle, tmp2[0]/2.0f, tmp2[1]/2.0f, tmp2[2]/2.0f, 1.0f);
            QuatCopy(quat_tmp, solver->rotation[vc->lb1]);
            QuatMul(solver->rotation[vc->lb1], quat_angle, quat_tmp);
            QuatNormalize(solver->rotation[vc->lb1]);

            Vec3Cross(tmp1, r2, impulse_vector);
            Mat3VecMul(tmp2, solver->Iw_inv[vc->lb2], tmp1);
            QuatSet(quat_angle, tmp2[0]/2.0f, tmp2[1]/2.0f, tmp2[2]/2.0f, 1.0f);
            QuatCopy(quat_tmp, solver->rotation[vc->lb2]);
            QuatMul(solver->rotation[vc->lb2], quat_angle, quat_tmp);
            QuatNormalize(solver->rotation[vc->lb2]);
        }
	}

    /* 
     * If all penetration depths at start of iteration was smaller that 3*linear_slop, we view contacts as
     * okay and early exit. Note that the solver target separation is -solver->linear_slop, not 0.0f. 
     */
    return (min_separation >= -3.0f*g_solver_config->linear_slop)
        ? 1
        : 0;
}

void ds_RigidBodyUpdateSolverDataAll(struct ds_RigidBodyPipeline *pipeline)
{
    ProfZone;

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;

    mat3 tmp, rot, rot_inv;
    for (u32 i = 0; i < active->body_sim_pool.count; ++i)
    {
        struct ds_RigidBodySim *sim = active->body_sim_pool.buf + i;
        struct ds_RigidBodyCompute *compute = active->body_compute_pool.buf + i;

        memset(compute->linear_velocity, 0, sizeof(vec3));
        memset(compute->angular_velocity, 0, sizeof(vec3));
        
        QuatVec3Rotate(sim->world_center_of_mass, sim->world.rotation, sim->local_center_of_mass);
        Vec3Translate(sim->world_center_of_mass, sim->world.position);
    
		/* setup inverted world inertia tensors and center of massses */
		Mat3Quat(rot, sim->world.rotation);
		Mat3Transpose(rot_inv, rot);
        Mat3Mul(tmp, rot, sim->local_inv_inertia);
        Mat3Mul(sim->world_inv_inertia, tmp, rot_inv);

        Mat3VecMul(sim->world_center_of_mass, rot, sim->local_center_of_mass);
        Vec3Translate(sim->world_center_of_mass, sim->world.position);

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

		/* integrate new velocities using external forces */
		Vec3TranslateScaled(compute->linear_velocity, g_solver_config->gravity, pipeline->timestep);
		Vec3ScaleSelf(compute->linear_velocity, linear_damp);
		Vec3ScaleSelf(compute->angular_velocity, angular_damp);
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
            struct ds_RigidBodySim *sim[2];
            struct ds_RigidBodyCompute *compute[2];
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
	    		Vec3Sub(ccp->r[0], ccp->v, sim[0]->world_center_of_mass);
	    		Vec3Sub(ccp->r[1], ccp->v, sim[1]->world_center_of_mass);
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
