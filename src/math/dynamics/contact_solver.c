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
#include "sys_public.h"

/* used in contact solver to cleanup the code from if-statements */
struct rigid_body static_body = { 0 };


struct contact_solver_config config_storage = { 0 };
struct contact_solver_config *g_solver_config = &config_storage;

void contact_solver_config_init(const u32 iteration_count, const u32 block_solver, const u32 warmup_solver, const vec3 gravity, const f32 baumgarte_constant, const f32 max_condition, const f32 linear_dampening, const f32 angular_dampening, const f32 linear_slop, const f32 restitution_threshold, const u32 sleep_enabled, const f32 sleep_time_threshold, const f32 sleep_linear_velocity_sq_limit, const f32 sleep_angular_velocity_sq_limit)
{
	ds_Assert(iteration_count >= 1);

	g_solver_config->iteration_count = iteration_count;
	g_solver_config->block_solver = block_solver;
	g_solver_config->warmup_solver = warmup_solver;
	Vec3Copy(g_solver_config->gravity, gravity);
	g_solver_config->baumgarte_constant = baumgarte_constant;
	g_solver_config->max_condition = max_condition;
	g_solver_config->linear_dampening = linear_dampening;
	g_solver_config->angular_dampening = angular_dampening;
	g_solver_config->linear_slop = linear_slop;
	g_solver_config->restitution_threshold = restitution_threshold;

 	g_solver_config->sleep_enabled = sleep_enabled;
	g_solver_config->sleep_time_threshold = sleep_time_threshold;
	g_solver_config->sleep_linear_velocity_sq_limit = sleep_linear_velocity_sq_limit;
	g_solver_config->sleep_angular_velocity_sq_limit = sleep_angular_velocity_sq_limit;

	g_solver_config->pending_warmup_solver = g_solver_config->warmup_solver;
	g_solver_config->pending_block_solver = g_solver_config->block_solver;
	g_solver_config->pending_sleep_enabled = g_solver_config->sleep_enabled;
	g_solver_config->pending_iteration_count = g_solver_config->iteration_count;
	g_solver_config->pending_linear_slop = g_solver_config->linear_slop;
	g_solver_config->pending_baumgarte_constant = g_solver_config->baumgarte_constant;
	g_solver_config->pending_restitution_threshold = g_solver_config->restitution_threshold;
	g_solver_config->pending_linear_dampening = g_solver_config->linear_dampening;
	g_solver_config->pending_angular_dampening = g_solver_config->angular_dampening;

	static_body.mass = F32_INFINITY;
	static_body.restitution = 0.0f;
}

struct contact_solver *contact_solver_init_body_data(struct arena *mem, struct island *is, const f32 timestep)
{
	struct contact_solver *solver = ArenaPush(mem, sizeof(struct contact_solver));

	solver->bodies = is->bodies;
	solver->timestep = timestep;
	solver->body_count = is->body_count;
	solver->contact_count = is->contact_count;

	solver->Iw_inv = ArenaPush(mem, (is->body_count + 1) * sizeof(mat3));
	solver->linear_velocity = ArenaPush(mem,  (is->body_count + 1) * sizeof(vec3));	/* last element is for static bodies with 0-value data */
	solver->angular_velocity = ArenaPush(mem, (is->body_count + 1) * sizeof(vec3));

	mat3ptr mi;
	mat3 rot, tmp1, tmp2;

	solver->bodies[solver->body_count] = &static_body;
	Vec3Set(solver->linear_velocity[solver->body_count], 0.0f, 0.0f, 0.0f);
	Vec3Set(solver->angular_velocity[solver->body_count], 0.0f, 0.0f, 0.0f);
	mi = solver->Iw_inv + solver->body_count;
	mat3_set(*mi, 0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f);


	for (u32 i = 0; i < is->body_count; ++i)
	{	
		const struct rigid_body *b = solver->bodies[i];

		/* setup inverted world intertia tensors */
		mat3ptr mi = solver->Iw_inv + i;
		Mat3Quat(rot, b->rotation);
		mat3_mult(tmp1, rot, ((struct rigid_body *) b)->inv_inertia_tensor);
		mat3_transpose_to(tmp2, rot);
		mat3_mult(*mi, tmp1, tmp2);

		/* integrate new velocities using external forces */
		Vec3Copy(solver->linear_velocity[i], b->velocity);
		Vec3Copy(solver->angular_velocity[i], b->angular_velocity);

		/* TODO: Apply other external forces here other than gravity and angular velocity */
		/* TODO: Explicit vs. Implicit Euler stability */
		/* TODO: quaternion will drift, become unnormalized*/
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

void contact_solver_init_velocity_constraints(struct arena *mem, struct contact_solver *solver, const struct physics_pipeline *pipeline, const struct island *is)
{
	solver->vcs = ArenaPush(mem, solver->contact_count * sizeof(struct velocity_constraint));

	vec3 tmp1, tmp2, tmp3, tmp4;
	vec3 vcp_Ic1[4]; 	/* Temporary storage for Inw(I_1)(r1 x n) */
	vec3 vcp_Ic2[4];	/* Temporary storage for Inw(I_2)(r2 x n) */
	vec3 vcp_c1[4];		/* Temporary storage for(r1 x n) */
	vec3 vcp_c2[4];		/* Temporary storage for(r2 x n) */
	for (u32 i = 0; i < solver->contact_count; ++i)
	{			
		struct velocity_constraint *vc = solver->vcs + i;

		const struct rigid_body *b1 = PoolAddress(&pipeline->body_pool, is->contacts[i]->cm.i1);
		const struct rigid_body *b2 = PoolAddress(&pipeline->body_pool, is->contacts[i]->cm.i2);
			
		const u32 b1_static = (b1->island_index == ISLAND_STATIC) ? 1 : 0; 
		const u32 b2_static = (b2->island_index == ISLAND_STATIC) ? 1 : 0; 

		const f32 b1_friction = b1->friction;
	       	const f32 b2_friction = b2->friction;
		const u32 static_contact = (b1_static | b2_static);

		/* 
		 * We enforce the rule that b1 should be dynamic, and since the math assumes directions between
		 * b1 and b2, we must flip them. 
		 */
		if (b1_static)
		{
			vc->lb1 = is->body_index_map[is->contacts[i]->cm.i2];
			vc->lb2 = solver->body_count;
			Vec3Scale(vc->normal, is->contacts[i]->cm.n, -1.0f);
		}
		else
		{
			vc->lb1 = is->body_index_map[is->contacts[i]->cm.i1];
			vc->lb2 = (b2_static) ? solver->body_count : is->body_index_map[is->contacts[i]->cm.i2];
			Vec3Copy(vc->normal, is->contacts[i]->cm.n);
		}

		b1 = solver->bodies[vc->lb1];
		b2 = solver->bodies[vc->lb2];
		mat3ptr Iw_inv1 = solver->Iw_inv + vc->lb1;
		mat3ptr Iw_inv2 = solver->Iw_inv + vc->lb2;

		Vec3CreateBasis(vc->tangent[0], vc->tangent[1], vc->normal);
		vc->vcp_count = is->contacts[i]->cm.v_count;
		vc->block_solve = 0;
		vc->restitution = f32_max(b1->restitution, b2->restitution);
		vc->friction = f32_sqrt(b1_friction*b2_friction);
		vc->vcps = ArenaPush(mem, vc->vcp_count * sizeof(struct velocity_constraint_point));

		for (u32 j = 0; j < vc->vcp_count; ++j)
		{
			struct velocity_constraint_point *vcp = vc->vcps + j;
			vcp->normal_impulse = 0.0f;
			vcp->tangent_impulse[0] = 0.0f;
			vcp->tangent_impulse[1] = 0.0f;
			
			Vec3Sub(vcp->r1, is->contacts[i]->cm.v[j], b1->position);
			Vec3Cross(vcp_c1[j], vcp->r1, vc->normal);
			mat3_vec_mul(vcp_Ic1[j], *Iw_inv1, vcp_c1[j]);
			vcp->normal_mass = 1.0f / b1->mass + Vec3Dot(vcp_Ic1[j], vcp_c1[j]);

			Vec3Cross(tmp1, vcp->r1, vc->tangent[0]);
			Vec3Cross(tmp3, vcp->r1, vc->tangent[1]);
			mat3_vec_mul(tmp2, *Iw_inv1, tmp1);
			mat3_vec_mul(tmp4, *Iw_inv1, tmp3);
			vcp->tangent_mass[0] = 1.0f / b1->mass + Vec3Dot(tmp1, tmp2);
			vcp->tangent_mass[1] = 1.0f / b1->mass + Vec3Dot(tmp3, tmp4);

			if (static_contact)
			{
				Vec3Set(vcp->r2, 0.0f, 0.0f, 0.0f);
				Vec3Set(vcp_c2[j], 0.0f, 0.0f, 0.0f);
				Vec3Set(vcp_Ic2[j], 0.0f, 0.0f, 0.0f);
			}
			else
			{
				Vec3Sub(vcp->r2, is->contacts[i]->cm.v[j], b2->position);
				Vec3Cross(vcp_c2[j], vcp->r2, vc->normal);
				mat3_vec_mul(vcp_Ic2[j], *Iw_inv2, vcp_c2[j]);
				vcp->normal_mass += 1.0f / b2->mass + Vec3Dot(vcp_Ic2[j], vcp_c2[j]);

				Vec3Cross(tmp1, vcp->r2, vc->tangent[0]);
				Vec3Cross(tmp3, vcp->r2, vc->tangent[1]);
				mat3_vec_mul(tmp2, *Iw_inv2, tmp1);
				mat3_vec_mul(tmp4, *Iw_inv2, tmp3);
				vcp->tangent_mass[0] += 1.0f / b2->mass + Vec3Dot(tmp1, tmp2);
				vcp->tangent_mass[1] += 1.0f / b2->mass + Vec3Dot(tmp3, tmp4);
			}

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
			const f32 seperating_velocity = Vec3Dot(vc->normal, relative_velocity);

			/* Apply velocity bias, taking the accepted error into account */
			vcp->velocity_bias = f32_max(is->contacts[i]->cm.depth[j] - g_solver_config->linear_slop, 0.0f)  * g_solver_config->baumgarte_constant / solver->timestep;

			//if (vc->vcp_count == 4) fprintf(stderr, "%u -  bias (without restitution): %f\t depth: %f\t timestep %f\n", j, vcp->velocity_bias, is->contacts[i]->cm.depth[j], solver->timestep);

			/* sufficiently fast collision happening, so apply the restitution effect */
			if (g_solver_config->restitution_threshold < -seperating_velocity)
			{
				vcp->velocity_bias += -seperating_velocity * vc->restitution;
			}
		}

		if (vc->vcp_count >= 2 && g_solver_config->block_solver)
		{
			const f32 mm_inv = 1.0f/b1->mass + 1.0f/b2->mass;

			/* upper bound for condition number in Infinity norm, usually larger than L2 norm */
			f32 cond_inf_lb = 0.0f;

			switch (vc->vcp_count)
			{
				case 2: 
				{ 
					vc->normal_mass = ArenaPush(mem, sizeof(mat2));
					vc->inv_normal_mass = ArenaPush(mem, sizeof(mat2));
					
					const f32 A11 = 1.0f / vc->vcps[0].normal_mass;
					const f32 A22 = 1.0f / vc->vcps[1].normal_mass;
					const f32 A12 = mm_inv + Vec3Dot(vcp_c1[0], vcp_Ic1[1]) + Vec3Dot(vcp_c2[0], vcp_Ic2[1]);
					mat2_set(*((mat2ptr) vc->inv_normal_mass), A11, A12, A12, A22);
					const f32 det = mat2_inverse(*((mat2ptr) vc->normal_mass), *((mat2ptr) vc->inv_normal_mass));

					if (f32_abs(det) <= 1000.0f * F32_EPSILON)
					{
						cond_inf_lb = g_solver_config->max_condition + 1.0f;
						break;
					}
					
					/* L_Infinity usually larger than L2 condition number */
					cond_inf_lb = f32_abs(mat2_abs_max(*((mat2ptr) vc->normal_mass)) / mat2_abs_min(*((mat2ptr) vc->normal_mass)));
				} break;

				case 3: 
				{ 
					vc->normal_mass = ArenaPush(mem, sizeof(mat3));
					vc->inv_normal_mass = ArenaPush(mem, sizeof(mat3));

					const f32 A11 = 1.0f / vc->vcps[0].normal_mass;
					const f32 A22 = 1.0f / vc->vcps[1].normal_mass;
					const f32 A33 = 1.0f / vc->vcps[2].normal_mass;
					const f32 A12 = mm_inv + Vec3Dot(vcp_Ic1[0], vcp_c1[1]) + Vec3Dot(vcp_Ic2[0], vcp_c2[1]);
					const f32 A13 = mm_inv + Vec3Dot(vcp_Ic1[0], vcp_c1[2]) + Vec3Dot(vcp_Ic2[0], vcp_c2[2]);
					const f32 A23 = mm_inv + Vec3Dot(vcp_Ic1[1], vcp_c1[2]) + Vec3Dot(vcp_Ic2[1], vcp_c2[2]);

					mat3_set(*((mat3ptr) vc->inv_normal_mass), A11, A12, A13,
					   	    A12, A22, A23,
						    A13, A23, A33);
					
					const f32 det = mat3_inverse(*((mat3ptr) vc->normal_mass), *((mat3ptr) vc->inv_normal_mass));

					if (f32_abs(det) <= 1000.0f * F32_EPSILON)
					{
						cond_inf_lb = g_solver_config->max_condition + 1.0f;
						break;
					}
					
					cond_inf_lb = f32_abs(mat3_abs_max(*((mat3ptr) vc->normal_mass)) / mat3_abs_min(*((mat3ptr) vc->normal_mass)));
				} break;

				case 4: 
				{ 
					vc->normal_mass = ArenaPush(mem, sizeof(mat4));
					vc->inv_normal_mass = ArenaPush(mem, sizeof(mat4));

					const f32 A11 = 1.0f / vc->vcps[0].normal_mass;
					const f32 A22 = 1.0f / vc->vcps[1].normal_mass;
					const f32 A33 = 1.0f / vc->vcps[2].normal_mass;
					const f32 A44 = 1.0f / vc->vcps[3].normal_mass;
					const f32 A12 = mm_inv + Vec3Dot(vcp_Ic1[0], vcp_c1[1]) + Vec3Dot(vcp_Ic2[0], vcp_c2[1]);
					const f32 A13 = mm_inv + Vec3Dot(vcp_Ic1[0], vcp_c1[2]) + Vec3Dot(vcp_Ic2[0], vcp_c2[2]);
					const f32 A14 = mm_inv + Vec3Dot(vcp_Ic1[0], vcp_c1[3]) + Vec3Dot(vcp_Ic2[0], vcp_c2[3]);
					const f32 A23 = mm_inv + Vec3Dot(vcp_Ic1[1], vcp_c1[2]) + Vec3Dot(vcp_Ic2[1], vcp_c2[2]);
					const f32 A24 = mm_inv + Vec3Dot(vcp_Ic1[1], vcp_c1[3]) + Vec3Dot(vcp_Ic2[1], vcp_c2[3]);
					const f32 A34 = mm_inv + Vec3Dot(vcp_Ic1[2], vcp_c1[3]) + Vec3Dot(vcp_Ic2[2], vcp_c2[3]);


					mat4_set(*((mat4ptr) vc->inv_normal_mass),
						    A11, A12, A13, A14, 
						    A12, A22, A23, A24, 
						    A13, A23, A33, A34, 
						    A14, A24, A34, A44);
					const f32 det = mat4_inverse(*((mat4ptr) vc->normal_mass), *((mat4ptr) vc->inv_normal_mass));

					if (f32_abs(det) <= 1000.0f * F32_EPSILON)
					{
						cond_inf_lb = g_solver_config->max_condition + 1.0f;
						break;
					}
					
					/* L_Infinity usually larger than L2 condition number */
					cond_inf_lb = f32_abs(mat4_abs_max(*((mat4ptr) vc->normal_mass)) / mat4_abs_min(*((mat4ptr) vc->normal_mass)));

				} break;
			}
	
			if (cond_inf_lb <= g_solver_config->max_condition)
			{
				vc->block_solve = 1;
			}
			else
			{
				vc->block_solve = 0;
				//fprintf(stderr, "WARNING: bad estimated upper bound of condition number in normal mass: %f\n", cond_inf_lb);
			}
		}
	}
}

void contact_solver_warmup(struct contact_solver *solver, const struct island *is)
{

	vec3 tmp1, tmp2, tmp3;
	for (u32 i = 0; i < solver->contact_count; ++i)
	{			
		struct contact *c = is->contacts[i];
		struct velocity_constraint *vc = solver->vcs + i;
	
		if (vc->vcp_count == c->cached_count)
		{
			for (u32 j = 0; j < vc->vcp_count; ++j)
			{
				struct velocity_constraint_point *vcp = vc->vcps + j;
				u32 best = U32_MAX;
				f32 closest_dist_sq = 0.01f * 0.01f;
				for (u32 k = 0; k < c->cached_count; ++k)
				{
					Vec3Sub(tmp1, c->cm.v[j], c->v_cache[k]);
					const f32 dist_sq = Vec3Dot(tmp1, tmp1);
					if (dist_sq < closest_dist_sq)
					{
						best = k;
						closest_dist_sq = dist_sq;
					}
				}

				if (best != U32_MAX)
				{
					vec3 old_tangent_impulse;
					Vec3Scale(old_tangent_impulse, c->tangent_cache[0], c->tangent_impulse_cache[best][0]);
					Vec3TranslateScaled(old_tangent_impulse, c->tangent_cache[1], c->tangent_impulse_cache[best][1]);

					vcp->normal_impulse = c->normal_impulse_cache[best];
					//vcp->tangent_impulse[0] = Vec3Dot(vc->tangent[0], old_tangent_impulse);
					//vcp->tangent_impulse[1] = Vec3Dot(vc->tangent[1], old_tangent_impulse);
					vcp->tangent_impulse[0] = 0.0f;
					vcp->tangent_impulse[1] = 0.0f;

					Vec3Scale(tmp1, vc->normal, vcp->normal_impulse);
					Vec3TranslateScaled(tmp1, vc->tangent[0], vcp->tangent_impulse[0]);
					Vec3TranslateScaled(tmp1, vc->tangent[1], vcp->tangent_impulse[1]);

					Vec3TranslateScaled(solver->linear_velocity[vc->lb1], tmp1, -1.0f / solver->bodies[vc->lb1]->mass);
					Vec3TranslateScaled(solver->linear_velocity[vc->lb2], tmp1, 1.0f / solver->bodies[vc->lb2]->mass);
					Vec3Cross(tmp2, vcp->r1, tmp1);
					mat3_vec_mul(tmp3, solver->Iw_inv[vc->lb1], tmp2);
					Vec3TranslateScaled(solver->angular_velocity[vc->lb1], tmp3, -1.0f);
					Vec3Cross(tmp2, vcp->r2, tmp1);
					mat3_vec_mul(tmp3, solver->Iw_inv[vc->lb2], tmp2);
					Vec3Translate(solver->angular_velocity[vc->lb2], tmp3);
				}
			}
		}
	}
}

void contact_solver_cache_impulse_data(struct contact_solver *solver, const struct island *is)
{
	for (u32 i = 0; i < solver->contact_count; ++i)
	{			
		struct contact *c = is->contacts[i];
		struct velocity_constraint *vc = solver->vcs + i;

		c->cached_count = vc->vcp_count;
		Vec3Copy(c->normal_cache, vc->normal);
		Vec3Copy(c->tangent_cache[0], vc->tangent[0]);
		Vec3Copy(c->tangent_cache[1], vc->tangent[1]);

		u32 j = 0;
		for (; j < vc->vcp_count; ++j)
		{
			Vec3Copy(c->v_cache[j], c->cm.v[j]);
			c->normal_impulse_cache[j] = vc->vcps[j].normal_impulse;
			c->tangent_impulse_cache[j][0] = vc->vcps[j].tangent_impulse[0];
			c->tangent_impulse_cache[j][1] = vc->vcps[j].tangent_impulse[1];
		}

		//for (; j < 4; ++j)
		//{
		//	Vec3Set(c->v_cache[j], F32_INFINITY, F32_INFINITY, F32_INFINITY);
		//	c->normal_impulse_cache[j] = 0.0f;
		//	c->tangent_impulse_cache[j][0] = 0.0f;
		//	c->tangent_impulse_cache[j][1] = 0.0f;
		//}
	}
}

void contact_solver_iterate_velocity_constraints(struct contact_solver *solver)
{
	vec4 b, new_total_impulse;
	vec3 tmp1, tmp2, tmp3;
	vec3 relative_velocity;
	for (u32 i = 0; i < solver->contact_count; ++i)
	{
		struct velocity_constraint *vc = solver->vcs + i;

		/* solve friction constraints first, since normal constraints are more important */
		for (u32 j = 0; j < vc->vcp_count; ++j)
		{
			struct velocity_constraint_point *vcp = vc->vcps + j;
			const f32 impulse_bound = vc->friction * vcp->normal_impulse;

			for (u32 k = 0; k < 2; ++k)
			{
				/* Calculate seperating velocity at point: JV */
				Vec3Sub(relative_velocity, 
						solver->linear_velocity[vc->lb2],
						solver->linear_velocity[vc->lb1]);
				Vec3Cross(tmp2, solver->angular_velocity[vc->lb2], vcp->r2);
				Vec3Cross(tmp3, solver->angular_velocity[vc->lb1], vcp->r1);
				Vec3Translate(relative_velocity, tmp2);
				Vec3TranslateScaled(relative_velocity, tmp3, -1.0f);
				const f32 seperating_velocity = Vec3Dot(vc->tangent[k], relative_velocity);

				/* update constraint point tangent impulse */
				f32 delta_impulse = -vcp->tangent_mass[k] * seperating_velocity;
				const f32 old_impulse = vcp->tangent_impulse[k];
				vcp->tangent_impulse[k] = f32_clamp(vcp->tangent_impulse[k] + delta_impulse, -impulse_bound, impulse_bound);
				delta_impulse = vcp->tangent_impulse[k] - old_impulse;

				/* update body velocities */
				Vec3Scale(tmp1, vc->tangent[k], delta_impulse);
				Vec3TranslateScaled(solver->linear_velocity[vc->lb1], tmp1, -1.0f / solver->bodies[vc->lb1]->mass);
				Vec3TranslateScaled(solver->linear_velocity[vc->lb2], tmp1, 1.0f / solver->bodies[vc->lb2]->mass);
				Vec3Cross(tmp2, vcp->r1, tmp1);
				mat3_vec_mul(tmp3, solver->Iw_inv[vc->lb1], tmp2);
				Vec3TranslateScaled(solver->angular_velocity[vc->lb1], tmp3, -1.0f);
				Vec3Cross(tmp2, vcp->r2, tmp1);
				mat3_vec_mul(tmp3, solver->Iw_inv[vc->lb2], tmp2);
				Vec3Translate(solver->angular_velocity[vc->lb2], tmp3);
			}
		}

		if (vc->vcp_count == 1 || !vc->block_solve)
		{
			for (u32 j = 0; j < vc->vcp_count; ++j)
			{
				struct velocity_constraint_point *vcp = vc->vcps + j;

				/* Calculate seperating velocity at point: JV */
				Vec3Sub(relative_velocity, 
						solver->linear_velocity[vc->lb2],
						solver->linear_velocity[vc->lb1]);
				Vec3Cross(tmp2, solver->angular_velocity[vc->lb2], vcp->r2);
				Vec3Cross(tmp3, solver->angular_velocity[vc->lb1], vcp->r1);
				Vec3Translate(relative_velocity, tmp2);
				Vec3TranslateScaled(relative_velocity, tmp3, -1.0f);
				const f32 seperating_velocity = Vec3Dot(vc->normal, relative_velocity);

				/* update constraint point normal impulse */
				f32 delta_impulse = vcp->normal_mass * (vcp->velocity_bias - seperating_velocity);
				const f32 old_impulse = vcp->normal_impulse;
				vcp->normal_impulse = f32_max(0.0f, vcp->normal_impulse + delta_impulse);
				delta_impulse = vcp->normal_impulse - old_impulse;

				/* update body velocities */
				Vec3Scale(tmp1, vc->normal, delta_impulse);
				Vec3TranslateScaled(solver->linear_velocity[vc->lb1], tmp1, -1.0f / solver->bodies[vc->lb1]->mass);
				Vec3TranslateScaled(solver->linear_velocity[vc->lb2], tmp1, 1.0f / solver->bodies[vc->lb2]->mass);
				Vec3Cross(tmp2, vcp->r1, tmp1);
				mat3_vec_mul(tmp3, solver->Iw_inv[vc->lb1], tmp2);
				Vec3TranslateScaled(solver->angular_velocity[vc->lb1], tmp3, -1.0f);
				Vec3Cross(tmp2, vcp->r2, tmp1);
				mat3_vec_mul(tmp3, solver->Iw_inv[vc->lb2], tmp2);
				Vec3Translate(solver->angular_velocity[vc->lb2], tmp3);
			}
		}
		else
		{
			Vec3Sub(tmp1, solver->linear_velocity[vc->lb2], solver->linear_velocity[vc->lb1]);
			for (u32 j = 0; j < vc->vcp_count; ++j)
			{
				struct velocity_constraint_point *vcp = vc->vcps + j;
				/* Calculate seperating velocity at point: JV */
				Vec3Cross(tmp2, solver->angular_velocity[vc->lb2], vcp->r2);
				Vec3Cross(tmp3, solver->angular_velocity[vc->lb1], vcp->r1);
				Vec3Add(relative_velocity, tmp1, tmp2);
				Vec3TranslateScaled(relative_velocity, tmp3, -1.0f);
				const f32 seperating_velocity = Vec3Dot(vc->normal, relative_velocity);
				b[j] = vcp->velocity_bias - seperating_velocity;
			}
			
			u32 solution_found = 0;
			switch (vc->vcp_count)
			{
				case 2: 
				{ 
					mat2ptr normal_mass = vc->normal_mass;
					mat2ptr inv_normal_mass = vc->inv_normal_mass;

					/* (1) xn == 0 
					 * 	=> vn = -b
					 */
					if (b[0] <= 0.0f && b[1] <= 0.0f)
					{
						solution_found = 1;	
						Vec2Set(new_total_impulse, 0.0f, 0.0f);
						goto BLOCK_SOLVER_UPDATE;
					}

					/* (2) vn == 0  
					 *	=>	x = inv(A)*b
					 */
					mat2_vec_mul(new_total_impulse, *normal_mass, b);
					if (new_total_impulse[0] >= 0.0f && new_total_impulse[1] >= 0.0f)
					{
						solution_found = 1;
						goto BLOCK_SOLVER_UPDATE;
					}

					/* (3) xi != 0  
					 * 	=> 0 = Ai1 * x1 + Ai2 * x2 - bi 
					 * 	=> xi = bi / Aii
					 * 	=> vn = A*x_i - b
					 */
					for (u32 j = 0; j < vc->vcp_count; ++j)
					{
						struct velocity_constraint_point *vcp = vc->vcps + j;
						const f32 xj = vcp->normal_mass * b[j];
						const u32 i1 = (j+1) % 2;
						if (xj >= 0.0f && (xj*(*inv_normal_mass)[j][i1] - b[i1]) >= 0.0f)
						{
							solution_found = 1;
							new_total_impulse[j] = xj;
							new_total_impulse[i1] = 0.0f;
							goto BLOCK_SOLVER_UPDATE;
						}
					}
				} break;

				case 3: 
				{ 
					mat3ptr normal_mass = vc->normal_mass;
					mat3ptr inv_normal_mass = vc->inv_normal_mass;

					/* (1) xn == 0 
					 * 	=> vn = -b
					 */
					if (b[0] <= 0.0f && b[1] <= 0.0f && b[2] <= 0.0f)
					{
						solution_found = 1;	
						Vec3Set(new_total_impulse, 0.0f, 0.0f, 0.0f);
						goto BLOCK_SOLVER_UPDATE;
					}

					/* (2) vn == 0  
					 *	=>	x = inv(A)*b
					 */
					mat3_vec_mul(new_total_impulse, *normal_mass, b);
					if (new_total_impulse[0] >= 0.0f && new_total_impulse[1] >= 0.0f && new_total_impulse[2] >= 0.0f)
					{
						solution_found = 1;
						goto BLOCK_SOLVER_UPDATE;
					}

					/* (3) xi != 0  
					 * 	=> 0 = Ai1 * x1 + ... + Ai3 * x3 - bi 
					 * 	=> xi = bi / Aii
					 * 	=> vn = A*x_i - b
					 */
					for (u32 j = 0; j < vc->vcp_count; ++j)
					{
						struct velocity_constraint_point *vcp = vc->vcps + j;
						const f32 xj = vcp->normal_mass * b[j];
						const u32 i1 = (j+1) % 3;
						const u32 i2 = (j+2) % 3;
						const f32 vni1 = xj*(*inv_normal_mass)[j][i1] - b[i1];
						const f32 vni2 = xj*(*inv_normal_mass)[j][i2] - b[i2];
						if (xj >= 0.0f && vni1 >= 0.0f && vni2 >= 0.0f)
						{
							solution_found = 1;
							new_total_impulse[j] = xj;
							new_total_impulse[i1] = 0.0f;
							new_total_impulse[i2] = 0.0f;
							goto BLOCK_SOLVER_UPDATE;
						}
					}
	
					/* (4) vni != 0  
					 * 	=>	inv(A)*vn = x - inv(A)*b
					 * 	=>	inv(A)_ii*vni = -row(inv(A),i)*b
					 * 	=>	vni = -row(inv(A),i)*b/inv(A)_ii
					 * 	=>	x = inv(A)(vn + b)
					 */
					for (u32 j = 0; j < vc->vcp_count; ++j)
					{
						struct velocity_constraint_point *vcp = vc->vcps + j;
						const f32 vnj = -((*normal_mass)[0][j]*b[0] + (*normal_mass)[1][j]*b[1] + (*normal_mass)[2][j]*b[2]) / (*normal_mass)[j][j];
						
						if (vnj < 0.0f) { continue; }

						vec3 tmp;
						Vec3Copy(tmp, b);
						tmp[j] += vnj;
						mat3_vec_mul(new_total_impulse, vc->normal_mass, tmp);
						new_total_impulse[j] = 0.0f;

						const u32 i1 = (j+1) % 3;
						const u32 i2 = (j+2) % 3;

						if (new_total_impulse[i1] >= 0.0f && new_total_impulse[i2] >= 0.0f)
						{
							solution_found = 1;
							goto BLOCK_SOLVER_UPDATE;
						}
					}
				} break;

				case 4: 
				{ 
					mat4ptr normal_mass = vc->normal_mass;
					mat4ptr inv_normal_mass = vc->inv_normal_mass;

					/* (1) xn == 0 
					 * 	=> vn = -b
					 */
					if (b[0] <= 0.0f && b[1] <= 0.0f && b[2] <= 0.0f && b[3] <= 0.0f)
					{
						solution_found = 1;	
						Vec4Set(new_total_impulse, 0.0f, 0.0f, 0.0f, 0.0f);
						goto BLOCK_SOLVER_UPDATE;
					}

					/* (2) vn == 0  
					 *	=>	x = inv(A)*b
					 */
					mat4_vec_mul(new_total_impulse, *((mat4ptr) vc->normal_mass), b);
					if (new_total_impulse[0] >= 0.0f && new_total_impulse[1] >= 0.0f && new_total_impulse[2] >= 0.0f && new_total_impulse[3] >= 0.0f)
					{
						solution_found = 1;
						goto BLOCK_SOLVER_UPDATE;
					}

					/* (3) xi != 0  
					 * 	=> 0 = Ai1 * x1 + ... + Ai4 * x4 - bi 
					 * 	=> xi = bi / Aii
					 * 	=> vn = col(A,i)*x_i - b
					 */
					for (u32 j = 0; j < vc->vcp_count; ++j)
					{
						struct velocity_constraint_point *vcp = vc->vcps + j;
						const f32 xj = b[j] * vcp->normal_mass;
						const u32 i1 = (j+1) % 4;
						const u32 i2 = (j+2) % 4;
						const u32 i3 = (j+3) % 4;
						if (xj >= 0.0f  && (xj * (*inv_normal_mass)[j][i1] - b[i1]) >= 0.0f
								&& (xj * (*inv_normal_mass)[j][i2] - b[i2]) >= 0.0f
								&& (xj * (*inv_normal_mass)[j][i3] - b[i3]) >= 0.0f)
						{
							solution_found = 1;
							new_total_impulse[j] = xj;
							new_total_impulse[i1] = 0.0f;
							new_total_impulse[i2] = 0.0f;
							new_total_impulse[i3] = 0.0f;
							goto BLOCK_SOLVER_UPDATE;
						}
					}

					/* (4) vni != 0  
					 * 	=>	inv(A)*vn = x - inv(A)*b
					 * 	=>	inv(A)_ii*vni = -row(inv(A),i)*b
					 * 	=>	vni = -row(inv(A),i)*b/inv(A)_ii
					 * 	=>	x = inv(A)(vn + b)
					 */
					for (u32 j = 0; j < vc->vcp_count; ++j)
					{
						struct velocity_constraint_point *vcp = vc->vcps + j;
						const f32 vnj = -((*normal_mass)[0][j]*b[0] + (*normal_mass)[1][j]*b[1] + (*normal_mass)[2][j]*b[2] + (*normal_mass)[3][j]*b[3]) / vcp->normal_mass;
						
						if (vnj < 0.0f) { continue; }

						vec4 tmp;
						Vec4Copy(tmp, b);
						tmp[j] += vnj;
						mat4_vec_mul(new_total_impulse, *normal_mass, tmp);
						new_total_impulse[j] = 0.0f;

						const u32 i1 = (j+1) % 4;
						const u32 i2 = (j+2) % 4;
						const u32 i3 = (j+3) % 4;

						if (new_total_impulse[i1] >= 0.0f && new_total_impulse[i2] >= 0.0f && new_total_impulse[i3] >= 0.0f)
						{
							solution_found = 1;
							goto BLOCK_SOLVER_UPDATE;
						}
					}

					/* (5) xi, xj != 0  
					 *	=> 	[bi] = [Aii  Aij][xi]
					 *	=> 	[bj]   [Aji  Ajj][xj]
					 *
					 *	=>	1/(Aii*Ajj - Aij*Aji) * [ Ajj  -Aij] [bi] = [xi]
					 *					[-Aji   Aii] [bj]   [xj]
					 * 	=> vn = col(A,i)*xi + col(A,j)*x_j - b
					 */
					const u32 xj1_scheme[6] =  { 0, 0, 0, 1, 1, 2 };
					const u32 xj2_scheme[6] =  { 1, 2, 3, 2, 3, 3 };
					const u32 vnj1_scheme[6] = { 2, 1, 1, 0, 0, 0 };
					const u32 vnj2_scheme[6] = { 3, 3, 2, 3, 2, 1 };

					for (u32 j = 0; j < 6; ++j)
					{
						const u32 x_index1 = xj1_scheme[j];
						const u32 x_index2 = xj2_scheme[j];
						const u32 vn_index1 = vnj1_scheme[j];
						const u32 vn_index2 = vnj2_scheme[j];
					
						const f32 Aii = (*inv_normal_mass)[x_index1][x_index1];
						const f32 Aij = (*inv_normal_mass)[x_index2][x_index1];
						const f32 Ajj = (*inv_normal_mass)[x_index2][x_index2];
						const f32 det = Aii*Ajj - Aij*Aij;

						//TODO
						if (det*det <= 0.0001f) { continue; }
						
						const f32 det_inv = 1.0f / det;
						new_total_impulse[x_index1] = det_inv * ( Ajj*b[x_index1] - Aij*b[x_index2]);
						new_total_impulse[x_index2] = det_inv * (-Aij*b[x_index1] + Aii*b[x_index2]);

						if (new_total_impulse[x_index1] < 0.0f || new_total_impulse[x_index2] < 0.0f) { continue; }


						const f32 vnj1 = 
							  (*inv_normal_mass)[x_index1][vn_index1]*new_total_impulse[x_index1] 
							+ (*inv_normal_mass)[x_index2][vn_index1]*new_total_impulse[x_index2]
 							- b[vn_index1];
						const f32 vnj2 = 
							  (*inv_normal_mass)[x_index1][vn_index2]*new_total_impulse[x_index1] 
							+ (*inv_normal_mass)[x_index2][vn_index2]*new_total_impulse[x_index2]
 							- b[vn_index2];

						if (vnj1 >= 0.0f && vnj2 >= 0.0f)
						{
							new_total_impulse[vn_index1] = 0.0f;
							new_total_impulse[vn_index2] = 0.0f;
							solution_found = 1;
							goto BLOCK_SOLVER_UPDATE;
						}

					}
				} break;
			}

			BLOCK_SOLVER_UPDATE:
			if (solution_found)
			{
				for (u32 j = 0; j < vc->vcp_count; ++j)
				{
					struct velocity_constraint_point *vcp = vc->vcps + j;
					const f32 delta_impulse = new_total_impulse[j] - vcp->normal_impulse;
					vcp->normal_impulse = new_total_impulse[j];
					
					Vec3Scale(tmp1, vc->normal, delta_impulse);
					Vec3TranslateScaled(solver->linear_velocity[vc->lb2], tmp1, 1.0f/solver->bodies[vc->lb2]->mass);
					Vec3TranslateScaled(solver->linear_velocity[vc->lb1], tmp1, -1.0f/solver->bodies[vc->lb1]->mass);

					Vec3Cross(tmp2, vcp->r2, tmp1);
					Vec3Cross(tmp3, vcp->r1, tmp1);

					mat3_vec_mul(tmp1, solver->Iw_inv[vc->lb2], tmp2);
					Vec3Translate(solver->angular_velocity[vc->lb2], tmp1);
					mat3_vec_mul(tmp1, solver->Iw_inv[vc->lb1], tmp3);
					Vec3TranslateScaled(solver->angular_velocity[vc->lb1], tmp1, -1.0f);
				}
			}
		}
	}
}
