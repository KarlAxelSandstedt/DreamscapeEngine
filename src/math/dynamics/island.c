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

#include "dynamics.h"
#include "quaternion.h"
#include "ds_job.h"

/* Add new body to island */
static void isdb_AddBodyToIsland(struct physicsPipeline *pipeline, struct island *is, const u32 body)
{
	ds_Assert(is->body_list.first != DLL_NULL && is->body_list.last != DLL_NULL);

	struct rigidBody *b = PoolAddress(&pipeline->body_pool, body);
	b->island_index = PoolIndex(&pipeline->is_db.island_pool, is);
	dll_Append(&is->body_list, pipeline->body_pool.buf, body);
}

static struct slot isdb_IslandEmpty(struct physicsPipeline *pipeline)
{
	struct slot slot = PoolAdd(&pipeline->is_db.island_pool);
	dll_Append(&pipeline->is_db.island_list, pipeline->is_db.island_pool.buf, slot.index);
	PhysicsEventIslandNew(pipeline, slot.index);

	struct island *is = slot.address;
	is->contact_list = dll_Init(struct contact);
	is->body_list = dll2_Init(struct rigidBody);
	is->flags = g_solver_config->sleep_enabled * (ISLAND_AWAKE | ISLAND_SLEEP_RESET);

	return slot;
}

struct island *isdb_InitIslandFromBody(struct physicsPipeline *pipeline, const u32 body)
{
	struct slot slot = isdb_IslandEmpty(pipeline);
	struct island *is = slot.address;
	struct rigidBody *b = PoolAddress(&pipeline->body_pool, body);
	b->island_index = slot.index;
	dll_Append(&is->body_list, pipeline->body_pool.buf, body);

	return is;
}

void isdb_PrintIsland(FILE *file, const struct physicsPipeline *pipeline, const u32 island, const char *desc)
{
	const struct island *is = PoolAddress(&pipeline->is_db.island_pool, island);
	if (!is) { return; }

	const struct contact *c;
	const struct rigidBody *b;

	fprintf(file, "Island %u %s:\n{\n", island, desc);

	fprintf(file, "\tbody_list.count: %u\n", is->body_list.count);
	fprintf(file, "\tcontact_list.count: %u\n", is->contact_list.count);
		
	fprintf(file, "\t(Body):                     { ");
	for (u32 i = is->body_list.first; i != DLL_NULL; i = dll2_Next(b))
	{
		fprintf(file, "(%u) ", i);
		b = PoolAddress(&pipeline->body_pool, i);
	}
	fprintf(file, "}\n");

	fprintf(file, "\t(Contact):                  { ");
	for (u32 i = is->contact_list.first; i != DLL_NULL; i = dll_Next(c))
	{
		fprintf(file, "(%u) ", i);
		c = nll_Address(&pipeline->c_db.contact_net, i);
	}
	fprintf(file, "}\n");

	fprintf(file, "\tContacts (Body, Body2):     { ");
	for (u32 i = is->contact_list.first; i != DLL_NULL; i = dll_Next(c))
	{
		c = nll_Address(&pipeline->c_db.contact_net, i);
		fprintf(file, "(%u,%u) ", c->cm.i1, c->cm.i2);
	}
	fprintf(file, "}\n");

	fprintf(file, "\tflags:\n\t{\n");
	fprintf(file, "\t\tawake: %u\n", ISLAND_AWAKE_BIT(is));
	fprintf(file, "\t\tsleep_reset: %u\n", ISLAND_SLEEP_RESET_BIT(is));
	fprintf(file, "\t\tsplit: %u\n", ISLAND_SPLIT_BIT(is));
	fprintf(file, "\t}\n");

	fprintf(file, "}\n");
}

struct isdb isdb_Alloc(struct arena *mem_persistent, const u32 initial_size)
{
	struct isdb is_db = { 0 };

	is_db.island_pool = PoolAlloc(NULL, initial_size, struct island, GROWABLE);
	is_db.island_list = dll_Init(struct island);

	return is_db;
}

void isdb_Dealloc(struct isdb *is_db)
{
	PoolDealloc(&is_db->island_pool);
}

void isdb_Flush(struct isdb *is_db)
{
	isdb_ClearFrame(is_db);
	PoolFlush(&is_db->island_pool);
	dll_Flush(&is_db->island_list);
}

void isdb_ClearFrame(struct isdb *is_db)
{
	is_db->possible_splits = NULL;
	is_db->possible_splits_count = 0;
}

void isdb_Validate(const struct physicsPipeline *pipeline)
{
	const struct isdb *is_db = &pipeline->is_db;
	const struct cdb *c_db = &pipeline->c_db;

	const struct island *is = NULL;
	const struct rigidBody *body = NULL;
	const struct contact *c = NULL;
	for (u32 i = pipeline->is_db.island_list.first; i != DLL_NULL; i = dll_Next(is))
	{
		is = PoolAddress(&pipeline->is_db.island_pool, i);

 		/* 1. verify body-island map count == island.body_list.count */
		u32 count = 0;
		for (u32 j = 0; j < pipeline->body_pool.count_max; ++j)
		{
			const struct rigidBody *b = PoolAddress(&pipeline->body_pool, j);
			if (PoolSlotAllocated(b) && b->island_index == i)
			{
				count += 1;
			}	
		}
		
		ds_Assert(count == is->body_list.count && "Body count of island should be equal to the number of bodies mapped to the island");
 
		/* 2. verify body-island map  == island.bodies */
		u32 list_length = 0; 
		for (u32 index = is->body_list.first; index != DLL_NULL; index = dll2_Next(body))
		{
			list_length += 1;
			body = PoolAddress(&pipeline->body_pool, index);
			ds_Assert(PoolSlotAllocated(body) && body->island_index == i);
		}
		ds_Assert(list_length == is->body_list.count);

		/* 3. if island no contacts, ds_Assert body.contacts == NULL */
		if (is->contact_list.count == 0)
		{
			ds_Assert(is->body_list.count == 1);
			body = PoolAddress(&pipeline->body_pool, is->body_list.first);
			ds_Assert(PoolSlotAllocated(body) && body->contact_first == NLL_NULL);
		}
		else
		{
			/* 
			 * 4. For each contact in island
			 * 	1. check contact exist
			 * 	2. check bodies in contact are mapped to island
			 */
			list_length = 0;
			struct contact *c = NULL;
			for (u32 index = is->contact_list.first; index != DLL_NULL; index = dll_Next(c))
			{
				list_length += 1;
				c = nll_Address(&c_db->contact_net, index);
				const struct rigidBody *b1 = PoolAddress(&pipeline->body_pool, c->cm.i1);
				const struct rigidBody *b2 = PoolAddress(&pipeline->body_pool, c->cm.i2);
				ds_Assert(PoolSlotAllocated(c));
				ds_Assert(c != NULL);
				ds_Assert((b1->island_index == i) || (b1->island_index == ISLAND_STATIC));
				ds_Assert((b2->island_index == i) || (b2->island_index == ISLAND_STATIC));
			}
			ds_Assert(list_length == is->contact_list.count);
		}
	}

	/* 5. verify no body points to invalid island */
	for (u32 i = 0; i < pipeline->body_pool.count_max; ++i)
	{
		struct rigidBody *body = PoolAddress(&pipeline->body_pool, i);
		if (PoolSlotAllocated(body) && body->island_index != ISLAND_NULL && body->island_index != ISLAND_STATIC)
		{
			struct island *is = PoolAddress(&is_db->island_pool, body->island_index);
			ds_Assert(PoolSlotAllocated(is));
		}
	}
}

struct island *isdb_BodyToIsland(struct physicsPipeline *pipeline, const u32 body)
{
	const u32 is_index = ((struct rigidBody *) PoolAddress(&pipeline->body_pool, body))->island_index;
	return (is_index != ISLAND_NULL && is_index != ISLAND_STATIC)
		? PoolAddress(&pipeline->is_db.island_pool, is_index)
		: NULL;
}

void isdb_ReserveSplitsMemory(struct arena *mem_frame, struct isdb *is_db)
{
	is_db->possible_splits = ArenaPush(mem_frame, is_db->island_pool.count * sizeof(u32));
}

void isdb_ReleaseUnusedSplitsMemory(struct arena *mem_frame, struct isdb *is_db)
{
	ArenaPopPacked(mem_frame, (is_db->island_pool.count - is_db->possible_splits_count) * sizeof(u32));
}

void isdb_TagForSplitting(struct physicsPipeline *pipeline, const u32 body)
{
	const struct rigidBody *b = PoolAddress(&pipeline->body_pool, body);
	ds_Assert(b->island_index != U32_MAX);

	struct island *is = PoolAddress(&pipeline->is_db.island_pool, b->island_index);
	if (!(is->flags & ISLAND_SPLIT))
	{
		ds_Assert(pipeline->is_db.possible_splits_count < pipeline->is_db.island_pool.count);
		is->flags |= ISLAND_SPLIT;
		pipeline->is_db.possible_splits[pipeline->is_db.possible_splits_count++] = b->island_index;
	}
}

void isdb_MergeIslands(struct physicsPipeline *pipeline, const u32 ci, const u32 b1, const u32 b2)
{
	const struct rigidBody *body1 = PoolAddress(&pipeline->body_pool, b1);
	const struct rigidBody *body2 = PoolAddress(&pipeline->body_pool, b2);

	const u32 expand = body1->island_index;
	const u32 merge = body2->island_index;

	//fprintf(stderr, "merging islands (%u, %u) -> (%u)\n", expand, merge, merge);

	/* new local contact within island */
	if (expand == merge)
	{
		struct island *is = PoolAddress(&pipeline->is_db.island_pool, expand);
		ds_Assert(is->contact_list.count != 0);
		ds_Assert(is->contact_list.last != DLL_NULL);

		dll_Append(&is->contact_list, pipeline->c_db.contact_net.pool.buf, ci);
	}
	/* new contact between distinct islands */
	else
	{
		struct island *is_expand = PoolAddress(&pipeline->is_db.island_pool, expand);
		struct island *is_merge = PoolAddress(&pipeline->is_db.island_pool, merge);

		if (g_solver_config->sleep_enabled)
		{
			const u32 island_sleep_interrupted = 1 - ISLAND_AWAKE_BIT(is_merge)*ISLAND_AWAKE_BIT(is_expand)
						+ ISLAND_TRY_SLEEP_BIT(is_merge) + ISLAND_TRY_SLEEP_BIT(is_expand);
			ds_Assert(!(ISLAND_AWAKE_BIT(is_merge) == 0 && ISLAND_AWAKE_BIT(is_expand) == 0));
			if (island_sleep_interrupted)
			{
				if (!ISLAND_AWAKE_BIT(is_expand))
				{
					PhysicsEventIslandAwake(pipeline, expand);	
				}
				is_expand->flags = ISLAND_AWAKE | ISLAND_SLEEP_RESET;
			}
		}

		struct contact *contact_new = nll_Address(&pipeline->c_db.contact_net, ci);
		if (is_expand->contact_list.count == 0)
		{
			is_expand->contact_list.first = ci;
		}
		else
		{
			struct contact *contact = nll_Address(&pipeline->c_db.contact_net, is_expand->contact_list.last);
			ds_Assert(contact->dll_next == DLL_NULL);
			contact->dll_next = ci;
			contact_new->dll_prev = is_expand->contact_list.last;
		}

		if (is_merge->contact_list.count == 0)
		{
			is_expand->contact_list.last = ci;
			contact_new->dll_next = DLL_NULL;
		}
		else
		{
			is_expand->contact_list.last = is_merge->contact_list.last;
			struct contact *contact = nll_Address(&pipeline->c_db.contact_net, is_merge->contact_list.first);
			ds_Assert(contact->dll_prev == DLL_NULL);
			contact->dll_prev = ci;
			contact_new->dll_next = is_merge->contact_list.first;
		}

		is_expand->body_list.count += is_merge->body_list.count;
		is_expand->contact_list.count += is_merge->contact_list.count + 1;

		struct rigidBody *body = PoolAddress(&pipeline->body_pool, is_expand->body_list.last);
		struct rigidBody *body2 = PoolAddress(&pipeline->body_pool, is_merge->body_list.first);
		ds_Assert(body->dll2_next == DLL_NULL);
		ds_Assert(body2->dll2_prev == DLL_NULL);
		body->dll2_next = is_merge->body_list.first;
		body2->dll2_prev = is_expand->body_list.last;
		is_expand->body_list.last = is_merge->body_list.last;

		for (u32 i = is_merge->body_list.first; i != DLL_NULL; i = body->dll2_next)
		{
			body = PoolAddress(&pipeline->body_pool, i);
			body->island_index = expand;
		}

		dll_Remove(&pipeline->is_db.island_list, pipeline->is_db.island_pool.buf, merge);
		PoolRemove(&pipeline->is_db.island_pool, merge);
		PhysicsEventIslandExpanded(pipeline, expand);
		PhysicsEventIslandRemoved(pipeline, merge);
	}
}

void isdb_IslandRemove(struct physicsPipeline *pipeline, struct island *island)
{
	const u32 island_index = PoolIndex(&pipeline->is_db.island_pool, island);
	dll_Remove(&pipeline->is_db.island_list, pipeline->is_db.island_pool.buf, island_index);
	PoolRemove(&pipeline->is_db.island_pool, island_index);
	PhysicsEventIslandRemoved(pipeline, island_index);
}

void isdb_IslandRemoveBodyResources(struct physicsPipeline *pipeline, const u32 island_index, const u32 body)
{
	struct island *island = PoolAddress(&pipeline->is_db.island_pool, island_index);
	ds_Assert(PoolSlotAllocated(island));

	struct rigidBody *b = PoolAddress(&pipeline->body_pool, body);
	for (u32 i = b->contact_first; i != NLL_NULL; )
	{
		const struct contact *c = nll_Address(&pipeline->c_db.contact_net, i);
		const u32 next = (body == CONTACT_KEY_TO_BODY_0(c->key))
			? c->nll_next[0] 
			: c->nll_next[1];
		dll_Remove(&island->contact_list, pipeline->c_db.contact_net.pool.buf, i);
		i = next;
	}

	dll_Remove(&island->body_list, pipeline->body_pool.buf, body); 	

	if (island->body_list.count == 0)
	{
		ds_Assert(island->contact_list.first == DLL_NULL);
		ds_Assert(island->body_list.first == DLL_NULL);
		ds_Assert(island->contact_list.last == DLL_NULL);
		ds_Assert(island->body_list.last == DLL_NULL);
		ds_Assert(island->contact_list.count == 0);
		ds_Assert(island->body_list.count == 0);
		dll_Remove(&pipeline->is_db.island_list, pipeline->is_db.island_pool.buf, island_index);
		PoolRemove(&pipeline->is_db.island_pool, island_index);
		PhysicsEventIslandRemoved(pipeline, island_index);
	}
}

void isdb_SplitIsland(struct arena *mem_tmp, struct physicsPipeline *pipeline, const u32 island_to_split)
{
	ArenaPushRecord(mem_tmp);

	struct island *split_island = PoolAddress(&pipeline->is_db.island_pool, island_to_split);
	//isdb_PrintIsland(stderr, &pipeline->is_db, &pipeline->c_db, island_to_split, "SPLIT");
	u32 *body_stack = ArenaPush(mem_tmp, split_island->body_list.count*sizeof(u32));
	u32 sc = 0;

	struct rigidBody *body_last;
	for (u32 bi = split_island->body_list.first; bi != DLL_NULL; bi = body_last->dll2_next)
	{
		body_last = PoolAddress(&pipeline->body_pool, bi);
		ds_Assert(body_last->island_index == island_to_split);
		struct slot slot = isdb_IslandEmpty(pipeline);
		struct island *new_island = slot.address;
		split_island = PoolAddress(&pipeline->is_db.island_pool, island_to_split);
		body_last->island_index = slot.index;

		u32 next = bi;
		while (1)
		{
			struct rigidBody *body = PoolAddress(&pipeline->body_pool, next);
			u32 ci = body->contact_first;
			ds_Assert(ci == NLL_NULL ||
				((next == CONTACT_KEY_TO_BODY_0(((struct contact *) nll_Address(&pipeline->c_db.contact_net, ci))->key)) 
				 && ((struct contact *) nll_Address(&pipeline->c_db.contact_net, ci))->nll_prev[0] == NLL_NULL) ||
				((next == CONTACT_KEY_TO_BODY_1(((struct contact *) nll_Address(&pipeline->c_db.contact_net, ci))->key)) 
				 && ((struct contact *) nll_Address(&pipeline->c_db.contact_net, ci))->nll_prev[1] == NLL_NULL));

			while (ci != NLL_NULL)
			{
				const struct contact *c = nll_Address(&pipeline->c_db.contact_net, ci);
				ds_Assert(ci >= pipeline->c_db.contacts_frame_usage.bit_count 
						||  BitVecGetBit(&pipeline->c_db.contacts_frame_usage, ci) == 1)
				
				const u32 neighbour_index = (next == c->cm.i1) 
							? c->cm.i2 
							: c->cm.i1;
				body = PoolAddress(&pipeline->body_pool, neighbour_index);
				const u32 neighbour_island = body->island_index;

				if (neighbour_island == island_to_split)
				{
					dll_Remove(&split_island->body_list, pipeline->body_pool.buf, neighbour_index);
					isdb_AddBodyToIsland(pipeline, new_island, neighbour_index);
					body_stack[sc++] = neighbour_index;
				}
				
				ci = (next == CONTACT_KEY_TO_BODY_0(c->key))
					? c->nll_next[0] 
					: c->nll_next[1];
			}

			if (sc)
			{
				next = body_stack[--sc];
				continue;
			}
	
			break;
		}
	}

	/* create contact lists of new islands */
	struct contact *c;
	for (u32 i = split_island->contact_list.first; i != DLL_NULL; i = dll_Next(c))
	{
		c = nll_Address(&pipeline->c_db.contact_net, i);
		ds_Assert(PoolSlotAllocated(c));
		if (i >= pipeline->c_db.contacts_frame_usage.bit_count || BitVecGetBit(&pipeline->c_db.contacts_frame_usage, i) == 1)
		{
			const struct rigidBody *b1 = PoolAddress(&pipeline->body_pool, c->cm.i1);
			const struct rigidBody *b2 = PoolAddress(&pipeline->body_pool, c->cm.i2);
			const u32 island1 = b1->island_index;
			const u32 island2 = b2->island_index;
			struct island *is = (island1 != ISLAND_STATIC)
				? PoolAddress(&pipeline->is_db.island_pool, island1)
				: PoolAddress(&pipeline->is_db.island_pool, island2);
			dll_Append(&is->contact_list, pipeline->c_db.contact_net.pool.buf, i);
		}
	}

	/* Remove split island */
	isdb_IslandRemove(pipeline, split_island);
	ArenaPopRecord(mem_tmp);
}

static u32 *island_solve(struct arena *mem_frame, struct physicsPipeline *pipeline, struct island *is, const f32 timestep)
{
	u32 *bodies_simulated = ArenaPush(mem_frame, is->body_list.count*sizeof(u32));
	ArenaPushRecord(mem_frame);

	/* Important: Reserve extra space for static body defaults used in contact solver */
	is->bodies = ArenaPush(mem_frame, (is->body_list.count + 1) * sizeof(struct rigidBody *));
	is->contacts = ArenaPush(mem_frame, is->contact_list.count * sizeof(struct contact *));
	is->body_index_map = ArenaPush(mem_frame, pipeline->body_pool.count_max * sizeof(u32));

	/* init body and contact arrays */
	u32 k = is->body_list.first;
	for (u32 i = 0; i < is->body_list.count; ++i)
	{
		struct rigidBody *b = PoolAddress(&pipeline->body_pool, k);
		bodies_simulated[i] = k;
		is->bodies[i] = b;
		is->body_index_map[k] = i;
		k = b->dll2_next;
	}

	if (g_solver_config->sleep_enabled && ISLAND_TRY_SLEEP_BIT(is))
	{
		is->flags = 0;
		for (u32 i = 0; i < is->body_list.count; ++i)
		{
			struct rigidBody *b = is->bodies[i];
			b->flags ^= RB_AWAKE;
		}
		PhysicsEventIslandAsleep(pipeline, PoolIndex(&pipeline->is_db.island_pool, is));	
	}
	/* Island low energy state was interrupted, or island is simply awake */
	else
	{
		k = is->contact_list.first;
		for (u32 i = 0; i < is->contact_list.count; ++i)
		{
			is->contacts[i] = nll_Address(&pipeline->c_db.contact_net, k);
 			k = is->contacts[i]->dll_next;
		}

		/* init solver and velocity constraints */
		struct solver *solver = SolverInitBodyData(mem_frame, is, timestep);
		SolverInitVelocityConstraints(mem_frame, solver, pipeline, is);
		
		if (g_solver_config->warmup_solver)
		{
			SolverWarmup(solver, is);
		}

		for (u32 i = 0; i < g_solver_config->iteration_count; ++i)
		{
			SolverIterateVelocityConstraints(solver);
		}

		SolverCacheImpulse(solver, is);

		/* integrate final solver velocities and update bodies and find lowest low_velocity time */
		if (g_solver_config->sleep_enabled)
		{
			f32 min_low_velocity_time = F32_MAX_POSITIVE_NORMAL;
			for (u32 i = 0; i < is->body_list.count; ++i)
			{
				struct rigidBody *b = is->bodies[i];
				Vec3TranslateScaled(b->position, solver->linear_velocity[i], timestep);	
				Vec3Copy(b->velocity, solver->linear_velocity[i]);	

				quat a_vel_quat, rot_delta;
				Vec3Copy(b->angular_velocity, solver->angular_velocity[i]);	
				QuatSet(a_vel_quat, 
						solver->angular_velocity[i][0], 
						solver->angular_velocity[i][1], 
						solver->angular_velocity[i][2],
					      	0.0f);
				QuatMul(rot_delta, a_vel_quat, b->rotation);
				QuatScale(rot_delta, timestep / 2.0f);
				QuatTranslate(b->rotation, rot_delta);
				QuatNormalize(b->rotation);

				/* Always set RB_AWAKE, if island should sleep, we set it later,
				 * but the bodies may come in sleeping if island just woke up 
				 */
				b->flags |= RB_AWAKE;
				b->low_velocity_time = (1-ISLAND_SLEEP_RESET_BIT(is)) * b->low_velocity_time;
				const f32 lv_sq = Vec3Dot(b->velocity, b->velocity);
				const f32 av_sq = Vec3Dot(b->angular_velocity, b->angular_velocity);
				if (lv_sq <= g_solver_config->sleep_linear_velocity_sq_limit && av_sq <= g_solver_config->sleep_angular_velocity_sq_limit)
				{
					b->low_velocity_time += timestep;
				}
				min_low_velocity_time = f32_min(min_low_velocity_time, b->low_velocity_time);
			}

			is->flags &= ~ISLAND_SLEEP_RESET;
			if (g_solver_config->sleep_time_threshold <= min_low_velocity_time)
			{
				is->flags |= ISLAND_TRY_SLEEP;
			}
		}
		/* only integrate final solver velocities and update bodies  */
		else 
		{
			for (u32 i = 0; i < is->body_list.count; ++i)
			{
				struct rigidBody *b = is->bodies[i];
				Vec3TranslateScaled(b->position, solver->linear_velocity[i], timestep);	
				Vec3Copy(b->velocity, solver->linear_velocity[i]);	

				quat a_vel_quat, rot_delta;
				Vec3Copy(b->angular_velocity, solver->angular_velocity[i]);	
				QuatSet(a_vel_quat, 
						solver->angular_velocity[i][0], 
						solver->angular_velocity[i][1], 
						solver->angular_velocity[i][2],
					      	0.0f);
				QuatMul(rot_delta, a_vel_quat, b->rotation);
				QuatScale(rot_delta, timestep / 2.0f);
				QuatTranslate(b->rotation, rot_delta);
				QuatNormalize(b->rotation);
			}
		}
	}

	ArenaPopRecord(mem_frame);
	return bodies_simulated;
}

void ThreadIslandSolve(void *task_input)
{
	ProfZone;

	struct task *t_ctx = task_input;
	struct islandSolveInput *args = t_ctx->input;
	args->out->body_count = args->is->body_list.count;
	args->out->bodies = island_solve(&t_ctx->executor->mem_frame, args->pipeline, args->is, args->timestep);

	ProfZoneEnd;
}
