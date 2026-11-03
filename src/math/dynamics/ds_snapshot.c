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
Used for serializing/deserializing the physics state.

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
        ds_DynamicsConfigPush(pipeline);                        // the copy's tick pushed and popped its own
    }
 */

/**************** ds_DynamicsConfig ****************/

#define DS_DYNAMICS_CONFIG_VERSION  1u

/*
 * Every value, X(kind, member, valid): the kind U32 (also enums and bools), U64, F32, V3, V4 or V4P (the point
 * palette), and its range with v the value. The stream's size, write and read, ds_DynamicsConfigCheck and
 * ds_DynamicsConfigUpdatePending expand this one list, so none of them can miss a field.
 */
#define DS_DYNAMICS_CONFIG_FIELDS(X)                                                                \
    X(V3,  gravity, CfgV3Finite(v))                                                                 \
    X(U32, initial_size, v > 0 && PowerOfTwoCheck(v))                                               \
    X(U64, frame_memory, v > 0)                                                                     \
    X(U32, solver.pgs_iteration_count, v >= 1)                                                      \
    X(U32, solver.ngs_iteration_count, v >= 1)                                                      \
    X(U32, solver.warmup_solver, v <= 1)                                                            \
    X(F32, solver.baumgarte_constant, CfgRange(v, 0.0f, 1.0f))                                      \
    X(F32, solver.max_linear_correction, CfgNonNeg(v))                                              \
    X(F32, solver.max_linear_velocity_magnitude, CfgNonNeg(v))                                      \
    X(F32, solver.max_angular_velocity_magnitude, CfgNonNeg(v))                                     \
    X(F32, solver.linear_dampening, CfgNonNeg(v))                                                   \
    X(F32, solver.angular_dampening, CfgNonNeg(v))                                                  \
    X(F32, solver.linear_slop, CfgNonNeg(v) && v > 0.0f)                                            \
    X(F32, solver.restitution_threshold, CfgNonNeg(v))                                              \
    X(U32, solver.sleep_enabled, v <= 1)                                                            \
    X(F32, solver.sleep_time_threshold, CfgNonNeg(v) && v > 0.0f)                                   \
    X(F32, solver.sleep_linear_velocity_sq_limit, CfgNonNeg(v) && v > 0.0f)                         \
    X(F32, solver.sleep_angular_velocity_sq_limit, CfgNonNeg(v) && v > 0.0f)                        \
    X(F32, numerics.dbvh_reinsert_threshold, CfgRange(v, 0.0f, 1.0f))                               \
    X(U32, numerics.cache_count_max, v >= 1)                                                        \
    X(F32, numerics.capsule_parallel_max_degrees, CfgRange(v, 0.0f, 45.0f))                         \
    X(F32, numerics.face_parallel_max_degrees, CfgRange(v, 0.0f, 45.0f))                            \
    X(F32, numerics.edge_parallel_max_degrees, CfgRange(v, 0.0f, 45.0f))                            \
    X(F32, numerics.sat_face_bias, CfgNonNeg(v))                                                    \
    X(F32, numerics.sat_cache_tolerance, CfgNonNeg(v))                                              \
    X(U32, numerics.gjk_max_iterations, v >= 1)                                                     \
    X(U32, draw.draw_shapes, v <= 1)                                                                \
    X(U32, draw.shape_color_mode, v < DRAW_COLOR_MODE_COUNT)                                        \
    X(U32, draw.draw_dbvh, v <= 1)                                                                  \
    X(U32, draw.draw_sbvh, v <= 1)                                                                  \
    X(U32, draw.draw_bounds, v <= 1)                                                                \
    X(U32, draw.draw_mesh_bvh, v <= 1)                                                              \
    X(U32, draw.draw_constraint_points, v <= 1)                                                     \
    X(U32, draw.draw_contact_forces, v <= 1)                                                        \
    X(U32, draw.draw_joints, v <= 1)                                                                \
    X(U32, draw.point_color_mode, v < DRAW_POINT_COLOR_MODE_COUNT)                                  \
    X(U32, draw.draw_on_top, v <= 1)                                                                \
    X(U32, draw.draw_body_axes, v <= 1)                                                             \
    X(U32, draw.draw_joint_forces, v <= 1)                                                          \
    X(U32, draw.draw_joint_breakage, v <= 1)                                                        \
    X(F32, draw.force_scale, CfgNonNeg(v))                                                          \
    X(F32, draw.point_size, CfgNonNeg(v))                                                           \
    X(F32, draw.joint_error_max, CfgNonNeg(v))                                                      \
    X(F32, draw.axis_length, CfgNonNeg(v))                                                          \
    X(U32, draw.refinement, v >= 1)                                                                 \
    X(F32, draw.energy_max, CfgNonNeg(v))                                                           \
    X(F32, draw.shape_alpha, CfgNonNeg(v))                                                          \
    X(F32, draw.shape_saturation, CfgNonNeg(v))                                                     \
    X(F32, draw.shape_lightness, CfgNonNeg(v))                                                      \
    X(V4,  draw.color_dbvh, CfgV4Finite(v))                                                         \
    X(V4,  draw.color_sbvh, CfgV4Finite(v))                                                         \
    X(V4,  draw.color_bounds, CfgV4Finite(v))                                                       \
    X(V4,  draw.color_mesh_bvh, CfgV4Finite(v))                                                     \
    X(V4,  draw.color_shape, CfgV4Finite(v))                                                        \
    X(V4,  draw.color_static, CfgV4Finite(v))                                                       \
    X(V4,  draw.color_kinematic, CfgV4Finite(v))                                                    \
    X(V4,  draw.color_dynamic, CfgV4Finite(v))                                                      \
    X(V4,  draw.color_sleeping, CfgV4Finite(v))                                                     \
    X(V4,  draw.color_awake, CfgV4Finite(v))                                                        \
    X(V4,  draw.color_velocity_low, CfgV4Finite(v))                                                 \
    X(V4,  draw.color_velocity_high, CfgV4Finite(v))                                                \
    X(V4,  draw.color_touching, CfgV4Finite(v))                                                     \
    X(V4,  draw.color_not_touching, CfgV4Finite(v))                                                 \
    X(V4P, draw.color_point, CfgV4PFinite(v))                                                       \
    X(V4,  draw.color_contact_force, CfgV4Finite(v))                                                \
    X(V4,  draw.color_friction_force, CfgV4Finite(v))                                               \
    X(V4,  draw.color_joint_force, CfgV4Finite(v))                                                  \
    X(V4,  draw.color_joint, CfgV4Finite(v))                                                        \
    X(V4,  draw.color_mouse_joint, CfgV4Finite(v))                                                  \
    X(V4,  draw.color_joint_error, CfgV4Finite(v))                                                  \
    X(V4,  draw.color_joint_break, CfgV4Finite(v))                                                  \
    X(V4,  draw.color_mass_center, CfgV4Finite(v))

#define CFG_SIZE_U32(m)     + 4
#define CFG_SIZE_U64(m)     + 8
#define CFG_SIZE_F32(m)     + 4
#define CFG_SIZE_V3(m)      + 12
#define CFG_SIZE_V4(m)      + 16
#define CFG_SIZE_V4P(m)     + 16*DRAW_POINT_COLOR_COUNT
#define CFG_SIZE(kind, m, valid)    CFG_SIZE_##kind(m)
#define DS_DYNAMICS_CONFIG_SIZE (4 DS_DYNAMICS_CONFIG_FIELDS(CFG_SIZE))

#define CFG_WRITE_U32(m)    ss_WriteU32Le(ss, (u32) config->m);
#define CFG_WRITE_U64(m)    ss_WriteU64Le(ss, config->m);
#define CFG_WRITE_F32(m)    ss_WriteF32Le(ss, config->m);
#define CFG_WRITE_V3(m)     ss_WriteF32LeN(ss, config->m.buf, 3);
#define CFG_WRITE_V4(m)     ss_WriteF32LeN(ss, config->m.buf, 4);
#define CFG_WRITE_V4P(m)    ss_WriteF32LeN(ss, config->m[0].buf, 4*DRAW_POINT_COLOR_COUNT);
#define CFG_WRITE(kind, m, valid)   CFG_WRITE_##kind(m)

#define CFG_READ_U32(m)     config->m = ss_ReadU32Le(ss);
#define CFG_READ_U64(m)     config->m = ss_ReadU64Le(ss);
#define CFG_READ_F32(m)     config->m = ss_ReadF32Le(ss);
#define CFG_READ_V3(m)      ss_ReadF32LeN(ss, config->m.buf, 3);
#define CFG_READ_V4(m)      ss_ReadF32LeN(ss, config->m.buf, 4);
#define CFG_READ_V4P(m)     ss_ReadF32LeN(ss, config->m[0].buf, 4*DRAW_POINT_COLOR_COUNT);
#define CFG_READ(kind, m, valid)    CFG_READ_##kind(m)

struct ds_DynamicsConfig ds_DynamicsConfigDefault(void)
{
    struct ds_DynamicsConfig config = { 0 };

    config.gravity = V3(0.0f, -GRAVITY_CONSTANT_DEFAULT, 0.0f);
    config.initial_size = 1024;
    config.frame_memory = 16*1024*1024;

    config.solver.pgs_iteration_count = 8;
    config.solver.ngs_iteration_count = 3;
    config.solver.warmup_solver = 1;
    config.solver.baumgarte_constant = 0.1f;
    config.solver.max_linear_correction = 0.2f;
    config.solver.max_linear_velocity_magnitude = 400.0f;
    config.solver.max_angular_velocity_magnitude = 10.0f * F32_PI;
    config.solver.linear_dampening = 0.1f;
    config.solver.angular_dampening = 0.1f;
    config.solver.linear_slop = 0.005f;
    config.solver.restitution_threshold = 0.001f;
    config.solver.sleep_enabled = 1;
    config.solver.sleep_time_threshold = 0.5f;
    config.solver.sleep_linear_velocity_sq_limit = 0.01f*0.01f;
    config.solver.sleep_angular_velocity_sq_limit = 0.05f*0.05f;

    config.numerics = ds_NumericsConfigDefault();
    config.draw = ds_DynamicsDrawConfigDefault();

    return config;
}

/* finite and in [low, high] (NaN fails both compares) */
static u32 CfgRange(const f32 val, const f32 low, const f32 high)
{
    return F32FiniteCheck(val) && val >= low && val <= high;
}

static u32 CfgNonNeg(const f32 val)
{
    return F32FiniteCheck(val) && val >= 0.0f;
}

static u32 CfgV3Finite(const v3 v)
{
    return F32FiniteCheck(v.x) && F32FiniteCheck(v.y) && F32FiniteCheck(v.z);
}

static u32 CfgV4Finite(const v4 v)
{
    return F32FiniteCheck(v.x) && F32FiniteCheck(v.y) && F32FiniteCheck(v.z) && F32FiniteCheck(v.w);
}

static u32 CfgV4PFinite(const v4 *palette)
{
    u32 ok = 1;
    for (u32 i = 0; i < DRAW_POINT_COLOR_COUNT; ++i)
    {
        ok = ok && CfgV4Finite(palette[i]);
    }

    return ok;
}

#define CFG_TYPE_U32    u32
#define CFG_TYPE_U64    u64
#define CFG_TYPE_F32    f32
#define CFG_TYPE_V3     v3
#define CFG_TYPE_V4     v4
#define CFG_TYPE_V4P    v4 *

#define CFG_CHECK(kind, m, valid)   { const CFG_TYPE_##kind v = config->m; ok = ok && (valid); }

u32 ds_DynamicsConfigCheck(const struct ds_DynamicsConfig *config)
{
    u32 ok = 1;
    DS_DYNAMICS_CONFIG_FIELDS(CFG_CHECK)

    return ok;
}

u64 ds_DynamicsConfigSerializeSize(const struct ds_DynamicsConfig *config)
{
    return DS_DYNAMICS_CONFIG_SIZE;
}

void ds_DynamicsConfigSerialize(struct ss *ss, const struct ds_DynamicsConfig *config)
{
    ds_Assert(ss->bit_index % 8 == 0);
    ds_Assert(ds_DynamicsConfigSerializeSize(config) <= ss_BytesLeft(ss));

    ss_WriteU32Le(ss, DS_DYNAMICS_CONFIG_VERSION);
    DS_DYNAMICS_CONFIG_FIELDS(CFG_WRITE)
}

u32 ds_DynamicsConfigTryDeserialize(struct ss *ss, struct ds_DynamicsConfig *config)
{
    const u64 bit_index = ss->bit_index;
    u32 success = 0;

    if (ss->bit_index % 8 != 0 || ss_BytesLeft(ss) < DS_DYNAMICS_CONFIG_SIZE)
    {
        goto end;
    }

    if (ss_ReadU32Le(ss) != DS_DYNAMICS_CONFIG_VERSION)
    {
        goto end;
    }

    *config = (struct ds_DynamicsConfig) { 0 };
    DS_DYNAMICS_CONFIG_FIELDS(CFG_READ)
    success = ds_DynamicsConfigCheck(config);

end:
    if (!success)
    {
        ss->bit_index = bit_index;
        *config = (struct ds_DynamicsConfig) { 0 };
    }

    return success;
}

/* Header: ns_tick, frame_memory, worker_count, worker_frame_size, frames_completed, the applied config, island_to_split */
#define DS_SNAPSHOT_HEADER_SIZE (5*sizeof(u64) + sizeof(u32) + DS_DYNAMICS_CONFIG_SIZE)

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

	/* a local generator seeded by the stream position: spread without touching the thread's numbers */
	struct rng rng;
	RngLocalInit(&rng, ss->bit_index);
	const u32 start = (u32) RngLocalU64Range(&rng, 0, pipeline->worker_count - 1);
	for (u32 i = 0; i < pipeline->worker_count && !*dst; ++i)
	{
		*dst = ArenaPushAligned(pipeline->worker[(start + i) % pipeline->worker_count].frame, size, 1);
	}

	if (!*dst)
	{
		return 0;
	}

	ss_Read8N(ss, (b8 *) *dst, size);
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

	ss_Read8N(ss, (b8 *) *dst, size);
	return 1;
}

/*
 * Every contact's narrowphase data and every compute's ccache is serialized by its owner: cgraph
 * colors (touching, awake), the ACTIVE set (non-touching, awake) and the sleeping sets.
 */
u64 ds_DynamicsSerializeSize(const struct ds_Dynamics *pipeline)
{
	u64 size = DS_SNAPSHOT_HEADER_SIZE
		+ ds_PoolSerializeSize(pipeline->body_pool)
		+ ds_BitSetSerializeSize(&pipeline->body_usage_set)
		+ ds_PoolSerializeSize(pipeline->joint_pool)
		+ ds_PoolSerializeSize(pipeline->shape_pool)
		+ ds_BitSetSerializeSize(&pipeline->shape_dynamic_usage_set)
		+ BVHSerializeSize(&pipeline->dynamic_bvh)
		+ BVHSerializeSize(&pipeline->static_bvh)
		+ ds_BitSetSerializeSize(&pipeline->shape_dirty_set)
		+ ds_CPoolSerializeSize(pipeline->dirty_shape_query)
		+ ds_PoolSerializeSize(pipeline->event_pool)
		+ DLLSerializeSize(pipeline->event_list)
		+ ds_PoolSerializeSize(pipeline->contact_pool)
		+ ds_HashMapSerializeSize(&pipeline->contact_map)
		+ ds_BitSetSerializeSize(&pipeline->contact_usage_set)
		+ ds_PoolSerializeSize(pipeline->island_pool)
		+ ds_BitSetSerializeSize(&pipeline->island_high_energy_set)
		+ ds_CGraphSerializeSize(pipeline)
		+ ds_PoolSerializeSize(pipeline->solver_set_pool);

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
	ds_DynamicsConfigSerialize(ss, &pipeline->config);
	ss_WriteU64Le(ss, pipeline->island_to_split.val);

	ds_PoolSerialize(ss, pipeline->body_pool);
	ds_BitSetSerialize(ss, &pipeline->body_usage_set);
	ds_PoolSerialize(ss, pipeline->joint_pool);
	ds_PoolSerialize(ss, pipeline->shape_pool);
	ds_BitSetSerialize(ss, &pipeline->shape_dynamic_usage_set);
	BVHSerialize(ss, &pipeline->dynamic_bvh);
	BVHSerialize(ss, &pipeline->static_bvh);
	ds_BitSetSerialize(ss, &pipeline->shape_dirty_set);
	ds_CPoolSerialize(ss, pipeline->dirty_shape_query);
	ds_PoolSerialize(ss, pipeline->event_pool);
	DLLSerialize(ss, pipeline->event_list);
	ds_PoolSerialize(ss, pipeline->contact_pool);
	ds_HashMapSerialize(ss, &pipeline->contact_map);
	ds_BitSetSerialize(ss, &pipeline->contact_usage_set);
	ds_PoolSerialize(ss, pipeline->island_pool);
	ds_BitSetSerialize(ss, &pipeline->island_high_energy_set);

	ds_CGraphSerialize(ss, pipeline);

	ds_PoolSerialize(ss, pipeline->solver_set_pool);
	for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
	{
		if (ds_PoolSlotAllocated(pipeline->solver_set_pool.buf + i))
		{
			ds_SolverSetSerialize(ss, pipeline, i);
		}
	}
}

u32 ds_DynamicsTryDeserialize(struct arena *mem, struct ss *ss, struct ds_Dynamics *pipeline, struct c_ShapeDB *cshape_db)
{
	ds_Assert(mem && ss->bit_index % 8 == 0);

	const u64 bit_index = ss->bit_index;
	const u64 mem_left = mem->mem_left;
	*pipeline = (struct ds_Dynamics) { 0 };
	if (ss_BytesLeft(ss) < DS_SNAPSHOT_HEADER_SIZE)
	{
		return 0;
	}

	const u64 ns_tick = ss_ReadU64Le(ss);
	const u64 frame_memory = ss_ReadU64Le(ss);
	const u32 worker_count = ss_ReadU32Le(ss);
	const u64 worker_frame_size = ss_ReadU64Le(ss);
	if (worker_count == 0)
	{
		ss->bit_index = bit_index;
		return 0;
	}

	ds_DynamicsAllocShell(mem, pipeline, ns_tick, frame_memory, cshape_db, worker_count, worker_frame_size);
	pipeline->frames_completed = ss_ReadU64Le(ss);
	if (!ds_DynamicsConfigTryDeserialize(ss, &pipeline->config_pending))
	{
		goto failure;
	}
	pipeline->config = pipeline->config_pending;
	ds_DynamicsConfigDerive(pipeline);
	pipeline->island_to_split = (ds_IslandId) { ss_ReadU64Le(ss) };

	/* frame data goes into the buffer the next tick reads (and does not flush) */
	const u64 f = pipeline->frames_completed & 0x1;
	for (u32 i = 0; i < pipeline->worker_count; ++i)
	{
		pipeline->worker[i].frame = pipeline->worker[i].frame_arr + f;
	}

	/* the contact pool precedes its owners, which check their contact indices against it */
	if (!ds_PoolTryDeserialize(NULL, ss, pipeline->body_pool, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->body_usage_set, GROWABLE)
		|| !ds_PoolTryDeserialize(NULL, ss, pipeline->joint_pool, GROWABLE)
		|| !ds_PoolTryDeserialize(NULL, ss, pipeline->shape_pool, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->shape_dynamic_usage_set, GROWABLE)
		|| !BVHTryDeserialize(NULL, ss, &pipeline->dynamic_bvh, GROWABLE)
		|| !BVHTryDeserialize(NULL, ss, &pipeline->static_bvh, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->shape_dirty_set, GROWABLE)
		|| !ds_CPoolTryDeserialize(NULL, ss, pipeline->dirty_shape_query, GROWABLE)
		|| !ds_PoolTryDeserialize(NULL, ss, pipeline->event_pool, GROWABLE)
		|| !DLLTryDeserialize(ss, pipeline->event_list)
		|| !ds_PoolTryDeserialize(NULL, ss, pipeline->contact_pool, GROWABLE)
		|| !ds_HashMapTryDeserialize(NULL, ss, &pipeline->contact_map, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->contact_usage_set, GROWABLE)
		|| !ds_PoolTryDeserialize(NULL, ss, pipeline->island_pool, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->island_high_energy_set, GROWABLE)
		|| !ds_CGraphTryDeserialize(ss, pipeline)
		|| !ds_PoolTryDeserialize(NULL, ss, pipeline->solver_set_pool, GROWABLE))
	{
		goto failure;
	}

	for (u32 i = 0; i < pipeline->worker_count; ++i)
	{
		pipeline->worker[i].joint_breaking_set = ds_BitSetAlloc(NULL, pipeline->joint_pool.length, 0, GROWABLE);
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

	return 1;

failure:
	ds_DynamicsFree(pipeline);
	ArenaPopPacked(mem, mem_left - mem->mem_left);
	*pipeline = (struct ds_Dynamics) { 0 };
	ss->bit_index = bit_index;
	return 0;
}
