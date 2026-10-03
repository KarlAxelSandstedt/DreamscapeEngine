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

/*
Used for serialiing/deserializing the physics state.

Example (repetition testing):

    {
        ArenaPushRecord(mem);

        const u64 size = ds_DynamicsSerializeSize(pipeline);
        struct ss snap = ss_Alloc(mem, size);
        if (snap.buf == NULL)
        {
            LogString(T_PHYSICS, S_FATAL, "Failed to allocate physics snapshot");
            FatalCleanupAndExit();
        }
        ds_DynamicsSerialize(&snap, pipeline);

        while (measuring)
        {
            ArenaPushRecord(mem);
            struct ds_Dynamics copy;
            snap.bit_index = 0;                                 // rewind
            ds_DynamicsTryDeserialize(mem, &snap, &copy, pipeline->cshape_db);  // restore (setup, not measured)
            ds_DynamicsTick(&copy);                             // zone of interest
            ds_DynamicsFree(&copy);                             // worker frames are heap allocated
            ArenaPopRecord(mem);
        }

        ArenaPopRecord(mem);
        ds_DynamicsSetGlobals(pipeline);                        // copy's tick set its own globals
    }
 */

/* Header: ns_tick, frame_memory, worker_count, worker_frame_size, frames_completed, numerics_config, gravity, margin_on, margin, island_to_split */
#define DS_SNAPSHOT_HEADER_SIZE (5*sizeof(u64) + sizeof(u32) + sizeof(struct ds_NumericsConfig) + sizeof(v3) + sizeof(u32) + sizeof(f32))

u32 ds_DynamicsFrameDataTryDeserialize(struct ss *ss, void **dst, struct ds_Dynamics *pipeline, const u64 size)
{
	*dst = NULL;
	if (size == 0)
	{
		return 1;
	}

	if (size > ss_BytesLeft(ss))
	{
		return 0;
	}

	const u32 start = (u32) RngU64Range(0, pipeline->worker_count - 1);
	for (u32 i = 0; i < pipeline->worker_count && !*dst; ++i)
	{
		*dst = ArenaPushAligned(pipeline->worker[(start + i) % pipeline->worker_count].frame, size, 1);
	}

	if (!*dst)
	{
		return 0;
	}

	ss_Read8N((b8 *) *dst, ss, size);
	return 1;
}

u32 ds_DynamicsHeapDataTryDeserialize(struct arena *mem, struct ss *ss, void **dst, const u64 size)
{
	*dst = NULL;
	if (size == 0)
	{
		return 1;
	}

	if (size > ss_BytesLeft(ss))
	{
		return 0;
	}

	*dst = ArenaPushAligned(mem, size, 1);
	if (!*dst)
	{
		return 0;
	}

	ss_Read8N((b8 *) *dst, ss, size);
	return 1;
}

/*
 * Every contact's narrowphase data and every compute's ccache is serialized by its owner: cgraph
 * colors (touching, awake), the ACTIVE set (non-touching, awake) and the sleeping sets.
 */
u64 ds_DynamicsSerializeSize(const struct ds_Dynamics *pipeline)
{
	u64 size = DS_SNAPSHOT_HEADER_SIZE
		+ ds_BodyPoolSerializeSize(&pipeline->body_pool)
		+ ds_BitSetSerializeSize(&pipeline->body_usage_set)
		+ ds_JointPoolSerializeSize(&pipeline->joint_pool)
		+ ds_ShapePoolSerializeSize(&pipeline->shape_pool)
		+ ds_BitSetSerializeSize(&pipeline->shape_dynamic_usage_set)
		+ BvhSerializeSize(&pipeline->dynamic_bvh)
		+ BvhSerializeSize(&pipeline->static_bvh)
		+ ds_BitSetSerializeSize(&pipeline->shape_dirty_set)
		+ ds_CPoolSerializeSize(pipeline->dirty_shape_query)
		+ ds_PhysicsEventPoolSerializeSize(&pipeline->event_pool)
		+ DLLSerializeSize(pipeline->event_list)
		+ ds_ContactPoolSerializeSize(&pipeline->contact_pool)
		+ ds_HashMapSerializeSize(&pipeline->contact_map)
		+ ds_BitSetSerializeSize(&pipeline->contact_usage_set)
		+ ds_IslandPoolSerializeSize(&pipeline->island_pool)
		+ ds_BitSetSerializeSize(&pipeline->island_high_energy_set)
		+ ds_CGraphSerializeSize(pipeline)
		+ ds_SolverSetPoolSerializeSize(&pipeline->solver_set_pool);

	for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
	{
		if (ds_PoolSlotAllocated(pipeline->solver_set_pool.buf + i))
		{
			size += ds_SolverSetSerializeSize(pipeline, i);
		}
	}

	return size;
}

void ds_DynamicsSerialize(struct ss *ss, const struct ds_Dynamics *pipeline)
{
	ds_Assert(ss->bit_index % 8 == 0);
	ds_Assert(ds_DynamicsSerializeSize(pipeline) <= ss_BytesLeft(ss));

#ifdef DS_ASSERT_DEBUG
	/* a contact without an owner would keep dangling narrowphase pointers after a restore */
	u32 owned_contacts = 0;
	for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
	{
		owned_contacts += pipeline->cgraph.color[i].contact_pool.count;
	}
	for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
	{
		const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + i;
		owned_contacts += (ds_PoolSlotAllocated(set)) ? set->contact_pool.count : 0;
	}
	ds_Assert(owned_contacts == pipeline->contact_pool.count);
#endif

	ss_WriteU64Le(ss, pipeline->ns_tick);
	ss_WriteU64Le(ss, pipeline->frame.mem_size);
	ss_WriteU32Le(ss, pipeline->worker_count);
	ss_WriteU64Le(ss, pipeline->worker[0].frame_arr[0].mem_size);
	ss_WriteU64Le(ss, pipeline->frames_completed);
	ss_Write8N(ss, (const b8 *) &pipeline->numerics_config, sizeof(pipeline->numerics_config));
	ss_Write8N(ss, (const b8 *) &pipeline->gravity, sizeof(pipeline->gravity));
	ss_WriteU32Le(ss, pipeline->margin_on);
	ss_Write8N(ss, (const b8 *) &pipeline->margin, sizeof(pipeline->margin));
	ss_WriteU64Le(ss, pipeline->island_to_split);

	ds_BodyPoolSerialize(ss, &pipeline->body_pool);
	ds_BitSetSerialize(ss, &pipeline->body_usage_set);
	ds_JointPoolSerialize(ss, &pipeline->joint_pool);
	ds_ShapePoolSerialize(ss, &pipeline->shape_pool);
	ds_BitSetSerialize(ss, &pipeline->shape_dynamic_usage_set);
	BvhSerialize(ss, &pipeline->dynamic_bvh);
	BvhSerialize(ss, &pipeline->static_bvh);
	ds_BitSetSerialize(ss, &pipeline->shape_dirty_set);
	ds_CPoolSerialize(ss, pipeline->dirty_shape_query);
	ds_PhysicsEventPoolSerialize(ss, &pipeline->event_pool);
	DLLSerialize(ss, pipeline->event_list);
	ds_ContactPoolSerialize(ss, &pipeline->contact_pool);
	ds_HashMapSerialize(ss, &pipeline->contact_map);
	ds_BitSetSerialize(ss, &pipeline->contact_usage_set);
	ds_IslandPoolSerialize(ss, &pipeline->island_pool);
	ds_BitSetSerialize(ss, &pipeline->island_high_energy_set);

	ds_CGraphSerialize(ss, pipeline);

	ds_SolverSetPoolSerialize(ss, &pipeline->solver_set_pool);
	for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
	{
		if (ds_PoolSlotAllocated(pipeline->solver_set_pool.buf + i))
		{
			ds_SolverSetSerialize(ss, pipeline, i);
		}
	}
}

u32 ds_DynamicsTryDeserialize(struct arena *mem, struct ss *ss, struct ds_Dynamics *pipeline, c_ShapeSDB *cshape_db)
{
	ds_Assert(mem && ss->bit_index % 8 == 0);
	/* frame data placement uses the rng; restore its state so deserialization doesn't affect the simulation */
	RngPushState();

	const u64 bit_index = ss->bit_index;
	const u64 mem_left = mem->mem_left;
	*pipeline = (struct ds_Dynamics) { 0 };
	if (ss_BytesLeft(ss) < DS_SNAPSHOT_HEADER_SIZE)
	{
		RngPopState();
		return 0;
	}

	const u64 ns_tick = ss_ReadU64Le(ss);
	const u64 frame_memory = ss_ReadU64Le(ss);
	const u32 worker_count = ss_ReadU32Le(ss);
	const u64 worker_frame_size = ss_ReadU64Le(ss);
	if (worker_count == 0)
	{
		ss->bit_index = bit_index;
		RngPopState();
		return 0;
	}

	ds_DynamicsAllocShell(mem, pipeline, ns_tick, frame_memory, cshape_db, worker_count, worker_frame_size);
	pipeline->frames_completed = ss_ReadU64Le(ss);
	ss_Read8N((b8 *) &pipeline->numerics_config, ss, sizeof(pipeline->numerics_config));
	ss_Read8N((b8 *) &pipeline->gravity, ss, sizeof(pipeline->gravity));
	pipeline->margin_on = ss_ReadU32Le(ss);
	ss_Read8N((b8 *) &pipeline->margin, ss, sizeof(pipeline->margin));
	pipeline->island_to_split = ss_ReadU64Le(ss);

	/* frame data goes into the buffer the next tick reads (and does not flush) */
	const u64 f = pipeline->frames_completed & 0x1;
	for (u32 i = 0; i < pipeline->worker_count; ++i)
	{
		pipeline->worker[i].frame = pipeline->worker[i].frame_arr + f;
	}

	/* the contact pool precedes its owners, which check their contact indices against it */
	if (!ds_BodyPoolTryDeserialize(NULL, ss, &pipeline->body_pool, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->body_usage_set, GROWABLE)
		|| !ds_JointPoolTryDeserialize(NULL, ss, &pipeline->joint_pool, GROWABLE)
		|| !ds_ShapePoolTryDeserialize(NULL, ss, &pipeline->shape_pool, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->shape_dynamic_usage_set, GROWABLE)
		|| !BvhTryDeserialize(NULL, ss, &pipeline->dynamic_bvh, GROWABLE)
		|| !BvhTryDeserialize(NULL, ss, &pipeline->static_bvh, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->shape_dirty_set, GROWABLE)
		|| !ds_CPoolTryDeserialize(NULL, ss, pipeline->dirty_shape_query, GROWABLE)
		|| !ds_PhysicsEventPoolTryDeserialize(NULL, ss, &pipeline->event_pool, GROWABLE)
		|| !DLLTryDeserialize(ss, pipeline->event_list)
		|| !ds_ContactPoolTryDeserialize(NULL, ss, &pipeline->contact_pool, GROWABLE)
		|| !ds_HashMapTryDeserialize(NULL, ss, &pipeline->contact_map, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->contact_usage_set, GROWABLE)
		|| !ds_IslandPoolTryDeserialize(NULL, ss, &pipeline->island_pool, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->island_high_energy_set, GROWABLE)
		|| !ds_CGraphTryDeserialize(ss, pipeline)
		|| !ds_SolverSetPoolTryDeserialize(NULL, ss, &pipeline->solver_set_pool, GROWABLE))
	{
		goto failure;
	}

	/* the pool restored the sets' raw bytes: clear all of them first, so ds_DynamicsFree is safe after a failure */
	for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
	{
		struct ds_SolverSet *set = pipeline->solver_set_pool.buf + i;
		if (ds_PoolSlotAllocated(set))
		{
			const u32 pool_slot = set->pool_slot;
			memset(set, 0, sizeof(*set));
			set->pool_slot = pool_slot;
		}
	}

	for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
	{
		if (ds_PoolSlotAllocated(pipeline->solver_set_pool.buf + i) && !ds_SolverSetTryDeserialize(ss, pipeline, i))
		{
			goto failure;
		}
	}

	RngPopState();
	return 1;

failure:
	ds_DynamicsFree(pipeline);
	ArenaPopPacked(mem, mem_left - mem->mem_left);
	*pipeline = (struct ds_Dynamics) { 0 };
	ss->bit_index = bit_index;
	RngPopState();
	return 0;
}
