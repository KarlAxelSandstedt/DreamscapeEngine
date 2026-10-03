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
Serialization and Recording API TODO(Under Construction)
===============================
Used for serialiing/deserializing the physics state.

Data-structure serialization plans:
-----------------------------------

    minQueue (bvh cost_queue)                                       NONE
            Always empty between insertions (bvh.c:238); allocated
            empty on deserialization.

    Composite structures:
        ds_CGraph: CG_COLOR_COUNT x { body_bitset, 3 CPools }.
        ds_SolverSet: pool elements OWN 6 CPools, so the solver set pool
            needs per-element serialization; copying the struct is not
            enough.

    Pointers to non-pool memory (cannot be serialized as raw bytes):
        ds_Contact.narrowphase (c_ContactResult): manifold, cache, tri and
            tri_manifold point into the thread frame arenas (alive for 2
            frames, double buffered), or the heap for sleeping contacts.
            Serialize by value and re-point on restore. => The frame
            arenas ARE state, at least for contacts.
        ds_ContactCompute.ccache (in solver set and color CPools): the
            warm-start cache, written to the frame arena at the end of a
            solve (ds_solver.c:605) and read next tick; heap for sleeping
            contacts (ds_contact.c:341). Same treatment as narrowphase.
            ds_ContactCompute.cc is rebuilt every solve: not state.

Example usage: repeat a tick on identical input.
------------------------------------------------

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

/*
 * Contact memory section: for every allocated contact its narrowphase arrays (flags, manifold, cache,
 * tri, tri_manifold), then the ccache array of every contact compute (colors, then solver sets).
 */
#define DS_SNAPSHOT_CONTACT_CACHE   0x1
#define DS_SNAPSHOT_CONTACT_TRI     0x2

static u64 ds_SnapshotContactSize(const struct ds_Contact *c)
{
	const struct c_ContactResult *r = &c->narrowphase;
	u64 size = sizeof(u32) + (u64) r->manifold_count*sizeof(struct c_Manifold);
	if (r->cache)
	{
		size += (u64) r->cache_count*sizeof(struct c_SatCache);
	}
	if (r->tri)
	{
		size += 2*(u64) r->manifold_count*sizeof(u32);
	}
	return size;
}

static u64 ds_SnapshotComputesSize(const ds_CPool(ds_ContactCompute) *pool)
{
	u64 size = 0;
	for (u32 i = 0; i < pool->count; ++i)
	{
		size += (u64) pool->buf[i].ccache_count*sizeof(struct ds_ContactConstraintCache);
	}
	return size;
}

static void ds_SnapshotComputesSerialize(struct ss *ss, const ds_CPool(ds_ContactCompute) *pool)
{
	for (u32 i = 0; i < pool->count; ++i)
	{
		const struct ds_ContactCompute *compute = pool->buf + i;
		ss_Write8N(ss, (const b8 *) compute->ccache, (u64) compute->ccache_count*sizeof(struct ds_ContactConstraintCache));
	}
}

/*
 * Read size bytes into set_mem if provided, otherwise into the worker frame with the most space left
 * (leaves room in every worker frame for restores inside a tick).
 */
static u32 ds_SnapshotReadArray(struct ds_Dynamics *pipeline, void **dst, struct arena *set_mem, struct ss *ss, const u64 size)
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

	struct arena *arena = set_mem;
	if (!arena)
	{
		arena = pipeline->worker[0].frame;
		for (u32 i = 1; i < pipeline->worker_count; ++i)
		{
			if (pipeline->worker[i].frame->mem_left > arena->mem_left)
			{
				arena = pipeline->worker[i].frame;
			}
		}
	}

	*dst = ArenaPushAligned(arena, size, 1);
	if (!*dst)
	{
		return 0;
	}

	ss_Read8N((b8 *) *dst, ss, size);
	return 1;
}

static u32 ds_SnapshotComputesTryDeserialize(struct ds_Dynamics *pipeline, ds_CPool(ds_ContactCompute) *pool, struct arena *set_mem, struct ss *ss)
{
	for (u32 i = 0; i < pool->count; ++i)
	{
		struct ds_ContactCompute *compute = pool->buf + i;
		compute->cc = NULL;
		if (!ds_SnapshotReadArray(pipeline, (void **) &compute->ccache, set_mem, ss, (u64) compute->ccache_count*sizeof(struct ds_ContactConstraintCache)))
		{
			return 0;
		}
	}
	return 1;
}

static u64 ds_SnapshotContactMemorySize(const struct ds_Dynamics *pipeline)
{
	u64 size = 0;
	for (u32 i = 0; i < pipeline->contact_pool.count_max; ++i)
	{
		const struct ds_Contact *c = pipeline->contact_pool.buf + i;
		if (ds_PoolSlotAllocated(c))
		{
			size += ds_SnapshotContactSize(c);
		}
	}

	for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
	{
		size += ds_SnapshotComputesSize(&pipeline->cgraph.color[i].contact_compute_pool);
	}

	for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
	{
		const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + i;
		if (ds_PoolSlotAllocated(set))
		{
			size += ds_SnapshotComputesSize(&set->contact_compute_pool);
		}
	}

	return size;
}

static void ds_SnapshotContactMemorySerialize(struct ss *ss, const struct ds_Dynamics *pipeline)
{
	for (u32 i = 0; i < pipeline->contact_pool.count_max; ++i)
	{
		const struct ds_Contact *c = pipeline->contact_pool.buf + i;
		if (!ds_PoolSlotAllocated(c))
		{
			continue;
		}

		const struct c_ContactResult *r = &c->narrowphase;
		const u32 flags = ((r->cache) ? DS_SNAPSHOT_CONTACT_CACHE : 0) | ((r->tri) ? DS_SNAPSHOT_CONTACT_TRI : 0);
		ss_WriteU32Le(ss, flags);
		ss_Write8N(ss, (const b8 *) r->manifold, (u64) r->manifold_count*sizeof(struct c_Manifold));
		if (r->cache)
		{
			ss_Write8N(ss, (const b8 *) r->cache, (u64) r->cache_count*sizeof(struct c_SatCache));
		}
		if (r->tri)
		{
			ss_Write8N(ss, (const b8 *) r->tri, (u64) r->manifold_count*sizeof(u32));
			ss_Write8N(ss, (const b8 *) r->tri_manifold, (u64) r->manifold_count*sizeof(u32));
		}
	}

	for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
	{
		ds_SnapshotComputesSerialize(ss, &pipeline->cgraph.color[i].contact_compute_pool);
	}

	for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
	{
		const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + i;
		if (ds_PoolSlotAllocated(set))
		{
			ds_SnapshotComputesSerialize(ss, &set->contact_compute_pool);
		}
	}
}

/* Sleeping contacts (and computes) go into their set's memory, everything else into the worker frames. */
static u32 ds_SnapshotContactMemoryTryDeserialize(struct ds_Dynamics *pipeline, struct ss *ss)
{
	for (u32 i = 0; i < pipeline->contact_pool.count_max; ++i)
	{
		struct ds_Contact *c = pipeline->contact_pool.buf + i;
		if (!ds_PoolSlotAllocated(c))
		{
			continue;
		}

		struct arena *set_mem = NULL;
		if (c->set >= SOLVER_SET_SLEEPING_FIRST && c->set != SOLVER_SET_NULL)
		{
			struct ds_SolverSet *set = pipeline->solver_set_pool.buf + c->set;
			if (c->set >= pipeline->solver_set_pool.count_max || !ds_PoolSlotAllocated(set) || !set->mem.mem_size)
			{
				return 0;
			}
			set_mem = &set->mem;
		}

		if (ss_BytesLeft(ss) < sizeof(u32))
		{
			return 0;
		}

		struct c_ContactResult *r = &c->narrowphase;
		const u32 flags = ss_ReadU32Le(ss);
		r->cache = NULL;
		r->tri = NULL;
		r->tri_manifold = NULL;
		if (!ds_SnapshotReadArray(pipeline, (void **) &r->manifold, set_mem, ss, (u64) r->manifold_count*sizeof(struct c_Manifold))
			|| ((flags & DS_SNAPSHOT_CONTACT_CACHE) && !ds_SnapshotReadArray(pipeline, (void **) &r->cache, set_mem, ss, (u64) r->cache_count*sizeof(struct c_SatCache)))
			|| ((flags & DS_SNAPSHOT_CONTACT_TRI) && !ds_SnapshotReadArray(pipeline, (void **) &r->tri, set_mem, ss, (u64) r->manifold_count*sizeof(u32)))
			|| ((flags & DS_SNAPSHOT_CONTACT_TRI) && !ds_SnapshotReadArray(pipeline, (void **) &r->tri_manifold, set_mem, ss, (u64) r->manifold_count*sizeof(u32))))
		{
			return 0;
		}
	}

	for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
	{
		if (!ds_SnapshotComputesTryDeserialize(pipeline, &pipeline->cgraph.color[i].contact_compute_pool, NULL, ss))
		{
			return 0;
		}
	}

	for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
	{
		struct ds_SolverSet *set = pipeline->solver_set_pool.buf + i;
		if (ds_PoolSlotAllocated(set) 
			&& !ds_SnapshotComputesTryDeserialize(pipeline, &set->contact_compute_pool, (set->mem.mem_size) ? &set->mem : NULL, ss))
		{
			return 0;
		}
	}

	return 1;
}

/* Header: ns_tick, frame_memory, worker_count, worker_frame_size, frames_completed, numerics_config, gravity, margin_on, margin, island_to_split */
#define DS_SNAPSHOT_HEADER_SIZE (5*sizeof(u64) + sizeof(u32) + sizeof(struct ds_NumericsConfig) + sizeof(v3) + sizeof(u32) + sizeof(f32))

static u64 ds_SnapshotSolverSetsSize(const struct ds_Dynamics *pipeline)
{
	u64 size = ds_SolverSetPoolSerializeSize(&pipeline->solver_set_pool);
	for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
	{
		const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + i;
		if (ds_PoolSlotAllocated(set))
		{
			size += sizeof(u64)
				+ ds_CPoolSerializeSize(set->body_sim_pool)
				+ ds_CPoolSerializeSize(set->body_compute_pool)
				+ ds_CPoolSerializeSize(set->contact_pool)
				+ ds_CPoolSerializeSize(set->contact_compute_pool)
				+ ds_CPoolSerializeSize(set->joint_sim_pool)
				+ ds_CPoolSerializeSize(set->island_pool);
		}
	}
	return size;
}

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
		+ ds_BitSetSerializeSize(&pipeline->island_high_energy_set);

	for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
	{
		const struct ds_CGraphColor *color = pipeline->cgraph.color + i;
		size += ds_CPoolSerializeSize(color->joint_sim_pool)
			+ ds_CPoolSerializeSize(color->contact_pool)
			+ ds_CPoolSerializeSize(color->contact_compute_pool)
			+ ((i != CG_SERIAL_COLOR) ? ds_BitSetSerializeSize(&color->body_bitset) : 0);
	}

	return size + ds_SnapshotSolverSetsSize(pipeline) + ds_SnapshotContactMemorySize(pipeline);
}

void ds_DynamicsSerialize(struct ss *ss, const struct ds_Dynamics *pipeline)
{
	ds_Assert(ss->bit_index % 8 == 0);
	ds_Assert(ds_DynamicsSerializeSize(pipeline) <= ss_BytesLeft(ss));

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

	for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
	{
		const struct ds_CGraphColor *color = pipeline->cgraph.color + i;
		ds_CPoolSerialize(ss, color->joint_sim_pool);
		ds_CPoolSerialize(ss, color->contact_pool);
		ds_CPoolSerialize(ss, color->contact_compute_pool);
		if (i != CG_SERIAL_COLOR)
		{
			ds_BitSetSerialize(ss, &color->body_bitset);
		}
	}

	ds_SolverSetPoolSerialize(ss, &pipeline->solver_set_pool);
	for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
	{
		const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + i;
		if (ds_PoolSlotAllocated(set))
		{
			/* sleeping sets own one heap arena (mem_size != 0) holding their CPools and contact memory */
			ss_WriteU64Le(ss, set->mem.mem_size);
			ds_CPoolSerialize(ss, set->body_sim_pool);
			ds_CPoolSerialize(ss, set->body_compute_pool);
			ds_CPoolSerialize(ss, set->contact_pool);
			ds_CPoolSerialize(ss, set->contact_compute_pool);
			ds_CPoolSerialize(ss, set->joint_sim_pool);
			ds_CPoolSerialize(ss, set->island_pool);
		}
	}

	ds_SnapshotContactMemorySerialize(ss, pipeline);
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
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->island_high_energy_set, GROWABLE))
	{
		goto failure;
	}

	for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
	{
		struct ds_CGraphColor *color = pipeline->cgraph.color + i;
		if (!ds_CPoolTryDeserialize(NULL, ss, color->joint_sim_pool, GROWABLE)
			|| !ds_CPoolTryDeserialize(NULL, ss, color->contact_pool, GROWABLE)
			|| !ds_CPoolTryDeserialize(NULL, ss, color->contact_compute_pool, GROWABLE)
			|| (i != CG_SERIAL_COLOR && !ds_BitSetTryDeserialize(NULL, ss, &color->body_bitset, GROWABLE)))
		{
			goto failure;
		}
	}

	if (!ds_SolverSetPoolTryDeserialize(NULL, ss, &pipeline->solver_set_pool, GROWABLE))
	{
		goto failure;
	}

	/* the pool restored the sets' raw bytes: clear their stale memory first, so ds_DynamicsFree is safe */
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
		struct ds_SolverSet *set = pipeline->solver_set_pool.buf + i;
		if (!ds_PoolSlotAllocated(set))
		{
			continue;
		}

		if (ss_BytesLeft(ss) < sizeof(u64))
		{
			goto failure;
		}

		const u64 set_mem_size = ss_ReadU64Le(ss);
		struct arena *set_mem = NULL;
		if (set_mem_size)
		{
			set->mem = ArenaAlloc(NULL, set_mem_size);
			set_mem = &set->mem;
		}

		const u32 growable = (set_mem) ? NOT_GROWABLE : GROWABLE;
		if (!ds_CPoolTryDeserialize(set_mem, ss, set->body_sim_pool, growable)
			|| !ds_CPoolTryDeserialize(set_mem, ss, set->body_compute_pool, growable)
			|| !ds_CPoolTryDeserialize(set_mem, ss, set->contact_pool, growable)
			|| !ds_CPoolTryDeserialize(set_mem, ss, set->contact_compute_pool, growable)
			|| !ds_CPoolTryDeserialize(set_mem, ss, set->joint_sim_pool, growable)
			|| !ds_CPoolTryDeserialize(set_mem, ss, set->island_pool, growable))
		{
			goto failure;
		}
	}

	/* 2-frame data goes into the buffer the next tick reads (and does not flush) */
	const u64 f = pipeline->frames_completed & 0x1;
	for (u32 i = 0; i < pipeline->worker_count; ++i)
	{
		pipeline->worker[i].frame = pipeline->worker[i].frame_arr + f;
	}

	if (!ds_SnapshotContactMemoryTryDeserialize(pipeline, ss))
	{
		goto failure;
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
