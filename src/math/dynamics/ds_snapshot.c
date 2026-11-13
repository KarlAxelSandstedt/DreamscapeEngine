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

#include "ds_init.h"

/*
Used for serializing/deserializing the physics state.

Example (repetition testing):

    {
        ArenaPushRecord(mem);

        const u64 size = ds_DynamicsSerializeSize(pipeline);
        struct SS snap = SSAlloc(mem, size);
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
            ds_DynamicsTryDeserialize(mem, &snap, &copy, pipeline->external);  // restore (setup, not measured)
            ds_DynamicsConfigPush(&copy);
            copy.runtime.mem_tick = tmp;                        // a stage driven directly: the setups push on it
            {
                Dynamics code zone being measured...
            }
            ds_DynamicsDealloc(&copy);                             // worker frames are heap allocated
            ArenaPopRecord(mem);
        }

        ArenaPopRecord(mem);
        ds_DynamicsConfigPush(pipeline);                        // the copies were pushed for their ticks
    }
 */

/*************************** configs ***************************/

#define DS_NUMERICS_CONFIG_VERSION  2u
#define DS_NUMERICS_CONFIG_BITS     (32 + 10*32)

u64 ds_NumericsConfigSerializeBitPackedSize(const struct ds_NumericsConfig *config)
{
    return DS_NUMERICS_CONFIG_BITS;
}

void ds_NumericsConfigSerializeBitPacked(struct SS *ss, const struct ds_NumericsConfig *config)
{
    ds_Assert(DS_NUMERICS_CONFIG_BITS <= SSBitsLeft(ss));

    SSWriteU64Partial(ss, DS_NUMERICS_CONFIG_VERSION, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->dbvh_reinsert_threshold }.u, 32);
    SSWriteU64Partial(ss, config->cache_count_max, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->capsule_parallel_max_degrees }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->face_parallel_max_degrees }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->edge_parallel_max_degrees }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->sat_face_bias }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->sat_cache_tolerance }.u, 32);
    SSWriteU64Partial(ss, config->gjk_max_iterations, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->proxy_margin_min }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->proxy_margin_scale }.u, 32);
}

u32 ds_NumericsConfigTryDeserializeBitPacked(struct SS *ss, struct ds_NumericsConfig *config)
{
    struct SS ss_mut = *ss;
    *config = (struct ds_NumericsConfig) { 0 };
    if (SSBitsLeft(&ss_mut) < DS_NUMERICS_CONFIG_BITS || SSReadU64Partial(&ss_mut, 32) != DS_NUMERICS_CONFIG_VERSION)
    {
        return 0;
    }

    config->dbvh_reinsert_threshold = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->cache_count_max = (u32) SSReadU64Partial(&ss_mut, 32);
    config->capsule_parallel_max_degrees = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->face_parallel_max_degrees = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->edge_parallel_max_degrees = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->sat_face_bias = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->sat_cache_tolerance = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->gjk_max_iterations = (u32) SSReadU64Partial(&ss_mut, 32);
    config->proxy_margin_min = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->proxy_margin_scale = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    *config = ds_NumericsConfigDerive(config);
    *ss = ss_mut;
    return 1;
}

#define DS_SOLVER_CONFIG_VERSION    1u
#define DS_SOLVER_CONFIG_BITS       (32 + 18*32)

u64 ds_SolverConfigSerializeBitPackedSize(const struct ds_SolverConfig *config)
{
    return DS_SOLVER_CONFIG_BITS;
}

void ds_SolverConfigSerializeBitPacked(struct SS *ss, const struct ds_SolverConfig *config)
{
    ds_Assert(DS_SOLVER_CONFIG_BITS <= SSBitsLeft(ss));

    SSWriteU64Partial(ss, DS_SOLVER_CONFIG_VERSION, 32);
    SSWriteU64Partial(ss, config->pgs_iteration_count, 32);
    SSWriteU64Partial(ss, config->ngs_iteration_count, 32);
    SSWriteU64Partial(ss, config->warmup_solver, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->gravity.x }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->gravity.y }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->gravity.z }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->baumgarte_constant }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->max_linear_correction }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->max_linear_velocity_magnitude }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->max_angular_velocity_magnitude }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->linear_dampening }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->angular_dampening }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->linear_slop }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->restitution_threshold }.u, 32);
    SSWriteU64Partial(ss, config->sleep_enabled, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->sleep_time_threshold }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->sleep_linear_velocity_sq_limit }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->sleep_angular_velocity_sq_limit }.u, 32);
}

u32 ds_SolverConfigTryDeserializeBitPacked(struct SS *ss, struct ds_SolverConfig *config)
{
    struct SS ss_mut = *ss;
    *config = (struct ds_SolverConfig) { 0 };
    if (SSBitsLeft(&ss_mut) < DS_SOLVER_CONFIG_BITS || SSReadU64Partial(&ss_mut, 32) != DS_SOLVER_CONFIG_VERSION)
    {
        return 0;
    }

    config->pgs_iteration_count = (u32) SSReadU64Partial(&ss_mut, 32);
    config->ngs_iteration_count = (u32) SSReadU64Partial(&ss_mut, 32);
    config->warmup_solver = (u32) SSReadU64Partial(&ss_mut, 32);
    config->gravity.x = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->gravity.y = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->gravity.z = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->baumgarte_constant = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->max_linear_correction = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->max_linear_velocity_magnitude = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->max_angular_velocity_magnitude = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->linear_dampening = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->angular_dampening = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->linear_slop = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->restitution_threshold = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->sleep_enabled = (u32) SSReadU64Partial(&ss_mut, 32);
    config->sleep_time_threshold = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->sleep_linear_velocity_sq_limit = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->sleep_angular_velocity_sq_limit = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    *config = ds_SolverConfigDerive(config);
    *ss = ss_mut;
    return 1;
}

#define DS_DYNAMICS_CONFIG_VERSION  2u
/* version + initial_size, then the solver and numerics configs */
#define DS_DYNAMICS_CONFIG_FIXED_BITS (32 + 32)

u64 ds_DynamicsConfigSerializeBitPackedSize(const struct ds_DynamicsConfig *config)
{
    return DS_DYNAMICS_CONFIG_FIXED_BITS
         + ds_SolverConfigSerializeBitPackedSize(&config->solver)
         + ds_NumericsConfigSerializeBitPackedSize(&config->numerics);
}

void ds_DynamicsConfigSerializeBitPacked(struct SS *ss, const struct ds_DynamicsConfig *config)
{
    ds_Assert(ds_DynamicsConfigSerializeBitPackedSize(config) <= SSBitsLeft(ss));

    SSWriteU64Partial(ss, DS_DYNAMICS_CONFIG_VERSION, 32);
    SSWriteU64Partial(ss, config->initial_size, 32);
    ds_SolverConfigSerializeBitPacked(ss, &config->solver);
    ds_NumericsConfigSerializeBitPacked(ss, &config->numerics);
}

u32 ds_DynamicsConfigTryDeserializeBitPacked(struct SS *ss, struct ds_DynamicsConfig *config)
{
    struct SS ss_mut = *ss;
    *config = (struct ds_DynamicsConfig) { 0 };
    if (SSBitsLeft(&ss_mut) < DS_DYNAMICS_CONFIG_FIXED_BITS || SSReadU64Partial(&ss_mut, 32) != DS_DYNAMICS_CONFIG_VERSION)
    {
        return 0;
    }

    config->initial_size = (u32) SSReadU64Partial(&ss_mut, 32);
    if (!ds_SolverConfigTryDeserializeBitPacked(&ss_mut, &config->solver) || !ds_NumericsConfigTryDeserializeBitPacked(&ss_mut, &config->numerics))
    {
        *config = (struct ds_DynamicsConfig) { 0 };
        return 0;
    }

    *config = ds_DynamicsConfigDerive(config);
    *ss = ss_mut;
    return 1;
}

#define DS_DYNAMICS_DRAW_CONFIG_VERSION 1u
#define DS_DYNAMICS_DRAW_CONFIG_SIZE    (4 + 15*4 + 8*4 + (22 + DRAW_POINT_COLOR_COUNT)*16)

u64 ds_DynamicsDrawConfigSerializeSize(const struct ds_DynamicsDrawConfig *config)
{
    return DS_DYNAMICS_DRAW_CONFIG_SIZE;
}

void ds_DynamicsDrawConfigSerialize(struct SS *ss, const struct ds_DynamicsDrawConfig *config)
{
    ds_Assert(ds_DynamicsDrawConfigSerializeSize(config) <= SSBytesLeft(ss));

    SSWriteU32Le(ss, DS_DYNAMICS_DRAW_CONFIG_VERSION);
    SSWriteU32Le(ss, config->draw_shapes);
    SSWriteU32Le(ss, (u32) config->shape_color_mode);
    SSWriteU32Le(ss, config->draw_dbvh);
    SSWriteU32Le(ss, config->draw_sbvh);
    SSWriteU32Le(ss, config->draw_bounds);
    SSWriteU32Le(ss, config->draw_mesh_bvh);
    SSWriteU32Le(ss, config->draw_constraint_points);
    SSWriteU32Le(ss, config->draw_contact_forces);
    SSWriteU32Le(ss, config->draw_joints);
    SSWriteU32Le(ss, (u32) config->point_color_mode);
    SSWriteU32Le(ss, config->draw_on_top);
    SSWriteU32Le(ss, config->draw_body_axes);
    SSWriteU32Le(ss, config->draw_joint_forces);
    SSWriteU32Le(ss, config->draw_joint_breakage);
    SSWriteF32Le(ss, config->force_scale);
    SSWriteF32Le(ss, config->point_size);
    SSWriteF32Le(ss, config->joint_error_max);
    SSWriteF32Le(ss, config->axis_length);
    SSWriteU32Le(ss, config->refinement);
    SSWriteF32Le(ss, config->energy_max);
    SSWriteF32Le(ss, config->shape_alpha);
    SSWriteF32Le(ss, config->shape_saturation);
    SSWriteF32Le(ss, config->shape_lightness);
    SSWriteF32LeN(ss, config->color_dbvh.buf, 4);
    SSWriteF32LeN(ss, config->color_sbvh.buf, 4);
    SSWriteF32LeN(ss, config->color_bounds.buf, 4);
    SSWriteF32LeN(ss, config->color_mesh_bvh.buf, 4);
    SSWriteF32LeN(ss, config->color_shape.buf, 4);
    SSWriteF32LeN(ss, config->color_static.buf, 4);
    SSWriteF32LeN(ss, config->color_kinematic.buf, 4);
    SSWriteF32LeN(ss, config->color_dynamic.buf, 4);
    SSWriteF32LeN(ss, config->color_sleeping.buf, 4);
    SSWriteF32LeN(ss, config->color_awake.buf, 4);
    SSWriteF32LeN(ss, config->color_velocity_low.buf, 4);
    SSWriteF32LeN(ss, config->color_velocity_high.buf, 4);
    SSWriteF32LeN(ss, config->color_touching.buf, 4);
    SSWriteF32LeN(ss, config->color_not_touching.buf, 4);
    SSWriteF32LeN(ss, config->color_point[0].buf, 4*DRAW_POINT_COLOR_COUNT);
    SSWriteF32LeN(ss, config->color_contact_force.buf, 4);
    SSWriteF32LeN(ss, config->color_friction_force.buf, 4);
    SSWriteF32LeN(ss, config->color_joint_force.buf, 4);
    SSWriteF32LeN(ss, config->color_joint.buf, 4);
    SSWriteF32LeN(ss, config->color_mouse_joint.buf, 4);
    SSWriteF32LeN(ss, config->color_joint_error.buf, 4);
    SSWriteF32LeN(ss, config->color_joint_break.buf, 4);
    SSWriteF32LeN(ss, config->color_mass_center.buf, 4);
}

u32 ds_DynamicsDrawConfigTryDeserialize(struct SS *ss, struct ds_DynamicsDrawConfig *config)
{
    struct SS ss_mut = *ss;
    *config = (struct ds_DynamicsDrawConfig) { 0 };
    if (SSBytesLeft(&ss_mut) < DS_DYNAMICS_DRAW_CONFIG_SIZE || SSReadU32Le(&ss_mut) != DS_DYNAMICS_DRAW_CONFIG_VERSION)
    {
        return 0;
    }

    config->draw_shapes = SSReadU32Le(&ss_mut);
    config->shape_color_mode = (enum ds_ShapeColorMode) SSReadU32Le(&ss_mut);
    config->draw_dbvh = SSReadU32Le(&ss_mut);
    config->draw_sbvh = SSReadU32Le(&ss_mut);
    config->draw_bounds = SSReadU32Le(&ss_mut);
    config->draw_mesh_bvh = SSReadU32Le(&ss_mut);
    config->draw_constraint_points = SSReadU32Le(&ss_mut);
    config->draw_contact_forces = SSReadU32Le(&ss_mut);
    config->draw_joints = SSReadU32Le(&ss_mut);
    config->point_color_mode = (enum ds_PointColorMode) SSReadU32Le(&ss_mut);
    config->draw_on_top = SSReadU32Le(&ss_mut);
    config->draw_body_axes = SSReadU32Le(&ss_mut);
    config->draw_joint_forces = SSReadU32Le(&ss_mut);
    config->draw_joint_breakage = SSReadU32Le(&ss_mut);
    config->force_scale = SSReadF32Le(&ss_mut);
    config->point_size = SSReadF32Le(&ss_mut);
    config->joint_error_max = SSReadF32Le(&ss_mut);
    config->axis_length = SSReadF32Le(&ss_mut);
    config->refinement = SSReadU32Le(&ss_mut);
    config->energy_max = SSReadF32Le(&ss_mut);
    config->shape_alpha = SSReadF32Le(&ss_mut);
    config->shape_saturation = SSReadF32Le(&ss_mut);
    config->shape_lightness = SSReadF32Le(&ss_mut);
    SSReadF32LeN(&ss_mut, config->color_dbvh.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_sbvh.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_bounds.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_mesh_bvh.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_shape.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_static.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_kinematic.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_dynamic.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_sleeping.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_awake.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_velocity_low.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_velocity_high.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_touching.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_not_touching.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_point[0].buf, 4*DRAW_POINT_COLOR_COUNT);
    SSReadF32LeN(&ss_mut, config->color_contact_force.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_friction_force.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_joint_force.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_joint.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_mouse_joint.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_joint_error.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_joint_break.buf, 4);
    SSReadF32LeN(&ss_mut, config->color_mass_center.buf, 4);
    *config = ds_DynamicsDrawConfigDerive(config);
    *ss = ss_mut;
    return 1;
}

/*************************** per-type serializers ***************************/

u64 ds_BodySerializeSize(const struct ds_Body *body)
{
	//TODO
	return 0;
}

void ds_BodySerialize(struct SS *ss, const struct ds_Body *body)
{
	//TODO
}

u32 ds_BodyTryDeserialize(struct SS *ss, struct ds_Body *body)
{
	//TODO
	return 0;
}

u64 ds_ShapeSerializeSize(const struct ds_Shape *shape)
{
	//TODO
	return 0;
}

void ds_ShapeSerialize(struct SS *ss, const struct ds_Shape *shape)
{
	//TODO
}

u32 ds_ShapeTryDeserialize(struct SS *ss, struct ds_Shape *shape)
{
	//TODO
	return 0;
}

u64 ds_JointSerializeSize(const struct ds_Joint *joint)
{
	//TODO
	return 0;
}

void ds_JointSerialize(struct SS *ss, const struct ds_Joint *joint)
{
	//TODO
}

u32 ds_JointTryDeserialize(struct SS *ss, struct ds_Joint *joint)
{
	//TODO
	return 0;
}

u64 ds_PhysicsEventSerializeSize(const struct ds_PhysicsEvent *event)
{
	//TODO
	return 0;
}

void ds_PhysicsEventSerialize(struct SS *ss, const struct ds_PhysicsEvent *event)
{
	//TODO
}

u32 ds_PhysicsEventTryDeserialize(struct SS *ss, struct ds_PhysicsEvent *event)
{
	//TODO
	return 0;
}

u64 ds_ContactSerializeSize(const struct ds_Contact *contact)
{
	//TODO
	return 0;
}

void ds_ContactSerialize(struct SS *ss, const struct ds_Contact *contact)
{
	//TODO
}

u32 ds_ContactTryDeserialize(struct SS *ss, struct ds_Contact *contact)
{
	//TODO
	return 0;
}

u64 ds_IslandSerializeSize(const struct ds_Island *island)
{
	//TODO
	return 0;
}

void ds_IslandSerialize(struct SS *ss, const struct ds_Island *island)
{
	//TODO
}

u32 ds_IslandTryDeserialize(struct SS *ss, struct ds_Island *island)
{
	//TODO
	return 0;
}

u64 ds_ProxyQuerySerializeSize(const struct ds_ProxyQuery *query)
{
	//TODO
	return 0;
}

void ds_ProxyQuerySerialize(struct SS *ss, const struct ds_ProxyQuery *query)
{
	//TODO
}

u32 ds_ProxyQueryTryDeserialize(struct SS *ss, struct ds_ProxyQuery *query)
{
	//TODO
	return 0;
}

u64 ds_BodySimSerializeSize(const struct ds_BodySim *bsim)
{
	//TODO
	return 0;
}

void ds_BodySimSerialize(struct SS *ss, const struct ds_BodySim *bsim)
{
	//TODO
}

u32 ds_BodySimTryDeserialize(struct SS *ss, struct ds_BodySim *bsim)
{
	//TODO
	return 0;
}

u64 ds_BodyComputeSerializeSize(const struct ds_BodyCompute *bcomp)
{
	//TODO
	return 0;
}

void ds_BodyComputeSerialize(struct SS *ss, const struct ds_BodyCompute *bcomp)
{
	//TODO
}

u32 ds_BodyComputeTryDeserialize(struct SS *ss, struct ds_BodyCompute *bcomp)
{
	//TODO
	return 0;
}

u64 ds_ContactComputeSerializeSize(const struct ds_ContactCompute *compute)
{
	//TODO
	return 0;
}

void ds_ContactComputeSerialize(struct SS *ss, const struct ds_ContactCompute *compute)
{
	//TODO
}

u32 ds_ContactComputeTryDeserialize(struct SS *ss, struct ds_ContactCompute *compute)
{
	//TODO
	return 0;
}

u64 ds_JointSimSerializeSize(const struct ds_JointSim *jsim)
{
	//TODO
	return 0;
}

void ds_JointSimSerialize(struct SS *ss, const struct ds_JointSim *jsim)
{
	//TODO
}

u32 ds_JointSimTryDeserialize(struct SS *ss, struct ds_JointSim *jsim)
{
	//TODO
	return 0;
}

u64 ds_SolverManifoldSerializeSize(const struct ds_SolverManifold *manifold)
{
	//TODO
	return 0;
}

void ds_SolverManifoldSerialize(struct SS *ss, const struct ds_SolverManifold *manifold)
{
	//TODO
}

u32 ds_SolverManifoldTryDeserialize(struct SS *ss, struct ds_SolverManifold *manifold)
{
	//TODO
	return 0;
}

u64 ds_ContactConstraintCacheSerializeSize(const struct ds_ContactConstraintCache *ccache)
{
	//TODO
	return 0;
}

void ds_ContactConstraintCacheSerialize(struct SS *ss, const struct ds_ContactConstraintCache *ccache)
{
	//TODO
}

u32 ds_ContactConstraintCacheTryDeserialize(struct SS *ss, struct ds_ContactConstraintCache *ccache)
{
	//TODO
	return 0;
}

/*************************** pool images ***************************/

/*
 * A pool's image: length, count, count_max, the free list in order, then every allocated slot as its index and
 * element. The free list and the index of every slot are checked as ds_PoolTryDeserialize checks them.
 */
static void SnapshotPoolFreeListSerialize(struct SS *ss, const void *buf, const u32 length, const u32 count, const u32 count_max, const u32 next_free, const u64 slot_size, const u64 slot_offset)
{
	const u8 *base = buf;
	SSWriteU32Le(ss, length);
	SSWriteU32Le(ss, count);
	SSWriteU32Le(ss, count_max);

	u32 free_count = 0;
	for (u32 i = next_free; i != POOL_INDEX_MASK; i = *(const u32 *) (base + i*slot_size + slot_offset) & POOL_INDEX_MASK)
	{
		SSWriteU32Le(ss, i);
		free_count += 1;
	}
	ds_Assert(free_count == count_max - count);
}

/* the free list and the allocated indices must fit; the elements check themselves */
static u32 SnapshotPoolHeaderTryDeserialize(struct SS *ss, u32 *length, u32 *count, u32 *count_max)
{
	if (SSBytesLeft(ss) < 3*sizeof(u32))
	{
		return 0;
	}

	*length = SSReadU32Le(ss);
	*count = SSReadU32Le(ss);
	*count_max = SSReadU32Le(ss);
	return *count <= *count_max && *count_max <= *length && *length < POOL_INDEX_MASK && (u64) *count_max*sizeof(u32) <= SSBytesLeft(ss);
}

/*
 * every index in [0, count_max) is written once: pool_slot starts as POOL_INDEX_MASK, a state no finished slot has
 * (allocated: 0, free: mask bit set), and a second visit fails the deserialization
 */
static u32 SnapshotPoolFreeListTryDeserialize(struct SS *ss, void *buf, u32 *next_free, const u32 count, const u32 count_max, const u64 slot_size, const u64 slot_offset)
{
	u8 *base = buf;
	for (u32 i = 0; i < count_max; ++i)
	{
		u32 *pool_slot = (u32 *) (base + i*slot_size + slot_offset);
		UnpoisonAddress(pool_slot, sizeof(u32));
		*pool_slot = POOL_INDEX_MASK;
	}

	u32 *prev_link = next_free;
	for (u32 i = 0; i < count_max - count; ++i)
	{
		const u32 index = SSReadU32Le(ss);
		if (index >= count_max)
		{
			return 0;
		}

		u32 *link = (u32 *) (base + index*slot_size + slot_offset);
		if (*link != POOL_INDEX_MASK)
		{
			return 0;
		}

		/* end of the list until a successor is linked */
		*link = POOL_ALLOCATION_MASK | POOL_INDEX_MASK;
		*prev_link = (prev_link == next_free) ? index : (POOL_ALLOCATION_MASK | index);
		prev_link = link;
	}
	return 1;
}

#define DEFINE_SNAPSHOT_POOL(_T_, _ELEM_)                                                                     \
static u64 _T_##PoolSerializeSize(const ds_Pool(_T_) *pool)                                                   \
{                                                                                                             \
	u64 size = 3*sizeof(u32) + (u64) pool->count_max*sizeof(u32);                                             \
	for (u32 i = 0; i < pool->count_max; ++i)                                                                 \
	{                                                                                                         \
		if (ds_PoolSlotAllocated(pool->buf + i))                                                              \
		{                                                                                                     \
			size += _ELEM_##SerializeSize(pool->buf + i);                                                     \
		}                                                                                                     \
	}                                                                                                         \
	return size;                                                                                              \
}                                                                                                             \
                                                                                                              \
static void _T_##PoolSerialize(struct SS *ss, const ds_Pool(_T_) *pool)                                       \
{                                                                                                             \
	SnapshotPoolFreeListSerialize(ss, pool->buf, pool->length, pool->count, pool->count_max, pool->next_free, sizeof(pool->buf[0]), ds_PoolSlotOffset(*pool)); \
	for (u32 i = 0; i < pool->count_max; ++i)                                                                 \
	{                                                                                                         \
		if (ds_PoolSlotAllocated(pool->buf + i))                                                              \
		{                                                                                                     \
			SSWriteU32Le(ss, i);                                                                              \
			_ELEM_##Serialize(ss, pool->buf + i);                                                             \
		}                                                                                                     \
	}                                                                                                         \
}                                                                                                             \
                                                                                                              \
static u32 _T_##PoolTryDeserialize(struct SS *ss, ds_Pool(_T_) *pool)                                         \
{                                                                                                             \
	const u64 bit_index = ss->bit_index;                                                                      \
	u32 length, count, count_max;                                                                             \
	if (!SnapshotPoolHeaderTryDeserialize(ss, &length, &count, &count_max))                                   \
	{                                                                                                         \
		ss->bit_index = bit_index;                                                                            \
		return 0;                                                                                             \
	}                                                                                                         \
	if (!ds_PoolAlloc(NULL, *pool, length, GROWABLE))                                                         \
	{                                                                                                         \
		ss->bit_index = bit_index;                                                                            \
		return 0;                                                                                             \
	}                                                                                                         \
	if (!SnapshotPoolFreeListTryDeserialize(ss, pool->buf, &pool->next_free, count, count_max, sizeof(pool->buf[0]), ds_PoolSlotOffset(*pool))) \
	{                                                                                                         \
		goto failure;                                                                                         \
	}                                                                                                         \
                                                                                                              \
	for (u32 i = 0; i < count; ++i)                                                                           \
	{                                                                                                         \
		const u32 index = SSReadU32Le(ss);                                                                    \
		if (index >= count_max || pool->buf[index].pool_slot != POOL_INDEX_MASK)                              \
		{                                                                                                     \
			goto failure;                                                                                     \
		}                                                                                                     \
		UnpoisonAddress(pool->buf + index, sizeof(pool->buf[0]));                                             \
		if (!_ELEM_##TryDeserialize(ss, pool->buf + index))                                                   \
		{                                                                                                     \
			goto failure;                                                                                     \
		}                                                                                                     \
		pool->buf[index].pool_slot = 0;                                                                       \
	}                                                                                                         \
	pool->count = count;                                                                                      \
	pool->count_max = count_max;                                                                              \
	return 1;                                                                                                 \
                                                                                                              \
failure:                                                                                                      \
	ds_PoolDealloc(*pool);                                                                                    \
	ss->bit_index = bit_index;                                                                                \
	return 0;                                                                                                 \
}

/* a compact pool's image: length, count, then the elements */
static u32 SnapshotCPoolHeaderTryDeserialize(struct SS *ss, u32 *length, u32 *count)
{
	if (SSBytesLeft(ss) < 2*sizeof(u32))
	{
		return 0;
	}

	*length = SSReadU32Le(ss);
	*count = SSReadU32Le(ss);
	return *count <= *length && *count <= SSBytesLeft(ss);
}

#define DEFINE_SNAPSHOT_CPOOL(_T_, _ELEM_)                                                                    \
static u64 _T_##CPoolSerializeSize(const ds_CPool(_T_) *pool)                                                 \
{                                                                                                             \
	u64 size = 2*sizeof(u32);                                                                                 \
	for (u32 i = 0; i < pool->count; ++i)                                                                     \
	{                                                                                                         \
		size += _ELEM_##SerializeSize(pool->buf + i);                                                         \
	}                                                                                                         \
	return size;                                                                                              \
}                                                                                                             \
                                                                                                              \
static void _T_##CPoolSerialize(struct SS *ss, const ds_CPool(_T_) *pool)                                     \
{                                                                                                             \
	SSWriteU32Le(ss, pool->length);                                                                           \
	SSWriteU32Le(ss, pool->count);                                                                            \
	for (u32 i = 0; i < pool->count; ++i)                                                                     \
	{                                                                                                         \
		_ELEM_##Serialize(ss, pool->buf + i);                                                                 \
	}                                                                                                         \
}                                                                                                             \
                                                                                                              \
static u32 _T_##CPoolTryDeserialize(struct arena *mem, struct SS *ss, ds_CPool(_T_) *pool, const u32 growable) \
{                                                                                                             \
	const u64 bit_index = ss->bit_index;                                                                      \
	u32 length, count;                                                                                        \
	if (!SnapshotCPoolHeaderTryDeserialize(ss, &length, &count))                                              \
	{                                                                                                         \
		ss->bit_index = bit_index;                                                                            \
		return 0;                                                                                             \
	}                                                                                                         \
	ds_CPoolAlloc(mem, *pool, length, growable);                                                              \
	if (pool->buf == NULL)                                                                                    \
	{                                                                                                         \
		ss->bit_index = bit_index;                                                                            \
		return 0;                                                                                             \
	}                                                                                                         \
                                                                                                              \
	UnpoisonAddress(pool->buf, count*sizeof(pool->buf[0]));                                                   \
	for (u32 i = 0; i < count; ++i)                                                                           \
	{                                                                                                         \
		if (!_ELEM_##TryDeserialize(ss, pool->buf + i))                                                       \
		{                                                                                                     \
			ds_CPoolDealloc(*pool);                                                                           \
			ss->bit_index = bit_index;                                                                        \
			return 0;                                                                                         \
		}                                                                                                     \
	}                                                                                                         \
	pool->count = count;                                                                                      \
	return 1;                                                                                                 \
}

static u64 U32CPoolSerializeSize(const ds_CPool(u32) *pool)
{
	return 2*sizeof(u32) + (u64) pool->count*sizeof(u32);
}

static void U32CPoolSerialize(struct SS *ss, const ds_CPool(u32) *pool)
{
	SSWriteU32Le(ss, pool->length);
	SSWriteU32Le(ss, pool->count);
	SSWriteU32LeN(ss, pool->buf, pool->count);
}

static u32 U32CPoolTryDeserialize(struct arena *mem, struct SS *ss, ds_CPool(u32) *pool, const u32 growable)
{
	const u64 bit_index = ss->bit_index;
	u32 length, count;
	if (!SnapshotCPoolHeaderTryDeserialize(ss, &length, &count) || (u64) count*sizeof(u32) > SSBytesLeft(ss))
	{
		ss->bit_index = bit_index;
		return 0;
	}
	ds_CPoolAlloc(mem, *pool, length, growable);
	if (pool->buf == NULL)
	{
		ss->bit_index = bit_index;
		return 0;
	}

	UnpoisonAddress(pool->buf, count*sizeof(u32));
	SSReadU32LeN(ss, pool->buf, count);
	pool->count = count;
	return 1;
}

/* the set slot holds pools only; ds_SolverSetSerialize writes them */
static u64 SolverSetSlotSerializeSize(const struct ds_SolverSet *set)
{
	return 0;
}

static void SolverSetSlotSerialize(struct SS *ss, const struct ds_SolverSet *set)
{
}

static u32 SolverSetSlotTryDeserialize(struct SS *ss, struct ds_SolverSet *set)
{
	return 1;
}

DEFINE_SNAPSHOT_POOL(ds_Body, ds_Body)
DEFINE_SNAPSHOT_POOL(ds_Shape, ds_Shape)
DEFINE_SNAPSHOT_POOL(ds_Joint, ds_Joint)
DEFINE_SNAPSHOT_POOL(ds_PhysicsEvent, ds_PhysicsEvent)
DEFINE_SNAPSHOT_POOL(ds_Contact, ds_Contact)
DEFINE_SNAPSHOT_POOL(ds_Island, ds_Island)
DEFINE_SNAPSHOT_POOL(ds_SolverSet, SolverSetSlot)
DEFINE_SNAPSHOT_CPOOL(ds_ProxyQuery, ds_ProxyQuery)
DEFINE_SNAPSHOT_CPOOL(ds_BodySim, ds_BodySim)
DEFINE_SNAPSHOT_CPOOL(ds_BodyCompute, ds_BodyCompute)
DEFINE_SNAPSHOT_CPOOL(ds_ContactCompute, ds_ContactCompute)
DEFINE_SNAPSHOT_CPOOL(ds_JointSim, ds_JointSim)

/*************************** contact data ***************************/

/*
 * Push size bytes for restored contact data: onto mem (a sleeping set's arena), or with mem NULL into a random
 * worker frame (the next one if it is full). Spreading the data leaves room in every worker frame, so a restore
 * inside a tick doesn't starve a worker of frame memory. The worker is chosen by a local generator seeded by the
 * stream position, so the thread's numbers are untouched. Returns NULL when nothing fits.
 */
static void *SnapshotPush(struct arena *mem, struct ds_Dynamics *pipeline, const struct SS *ss, const u64 size)
{
	if (mem)
	{
		return ArenaPushAligned(mem, size, 1);
	}

	struct rng rng;
	RngLocalInit(&rng, ss->bit_index);
	const u32 start = (u32) RngLocalU64Range(&rng, 0, pipeline->runtime.worker_count - 1);
	void *dst = NULL;
	for (u32 i = 0; i < pipeline->runtime.worker_count && !dst; ++i)
	{
		dst = ArenaPushAligned(pipeline->runtime.worker[(start + i) % pipeline->runtime.worker_count].frame, size, 1);
	}
	return dst;
}

#define DS_CONTACT_NARROWPHASE_TRI       0x2

u64 ds_ContactNarrowphaseSerializeSize(const struct ds_Dynamics *pipeline, const u32 contact)
{
	const struct c_ContactResult *r = &pipeline->contact_pool.buf[contact].narrowphase;
	u64 size = sizeof(u32);
	for (u32 i = 0; i < r->manifold_count; ++i)
	{
		size += ds_SolverManifoldSerializeSize(r->manifold + i);
	}
	if (r->tri)
	{
		size += (u64) r->manifold_count*sizeof(u32);
	}
	return size;
}

void ds_ContactNarrowphaseSerialize(struct SS *ss, const struct ds_Dynamics *pipeline, const u32 contact)
{
	const struct c_ContactResult *r = &pipeline->contact_pool.buf[contact].narrowphase;
	const u32 flags = (r->tri) ? DS_CONTACT_NARROWPHASE_TRI : 0;
	SSWriteU32Le(ss, flags);
	for (u32 i = 0; i < r->manifold_count; ++i)
	{
		ds_SolverManifoldSerialize(ss, r->manifold + i);
	}
	if (r->tri)
	{
		SSWriteU32LeN(ss, r->tri, r->manifold_count);
	}
}

u32 ds_ContactNarrowphaseTryDeserialize(struct arena *mem, struct SS *ss, struct ds_Dynamics *pipeline, const u32 contact)
{
	struct c_ContactResult *r = &pipeline->contact_pool.buf[contact].narrowphase;
	r->manifold = NULL;
	r->tri = NULL;
	if (SSBytesLeft(ss) < sizeof(u32) || r->manifold_count > SSBytesLeft(ss))
	{
		return 0;
	}

	const u32 flags = SSReadU32Le(ss);
	if (r->manifold_count == 0)
	{
		return 1;
	}

	r->manifold = SnapshotPush(mem, pipeline, ss, (u64) r->manifold_count*sizeof(struct ds_SolverManifold));
	if (r->manifold == NULL)
	{
		return 0;
	}

	for (u32 i = 0; i < r->manifold_count; ++i)
	{
		if (!ds_SolverManifoldTryDeserialize(ss, r->manifold + i))
		{
			return 0;
		}
	}

	if (flags & DS_CONTACT_NARROWPHASE_TRI)
	{
		if ((u64) r->manifold_count*sizeof(u32) > SSBytesLeft(ss))
		{
			return 0;
		}

		r->tri = SnapshotPush(mem, pipeline, ss, (u64) r->manifold_count*sizeof(u32));
		if (r->tri == NULL)
		{
			return 0;
		}
		SSReadU32LeN(ss, r->tri, r->manifold_count);
	}
	return 1;
}

u64 ds_ContactComputeCacheSerializeSize(const struct ds_ContactCompute *compute)
{
	u64 size = 0;
	for (u32 i = 0; i < compute->ccache_count; ++i)
	{
		size += ds_ContactConstraintCacheSerializeSize(compute->ccache + i);
	}
	return size;
}

void ds_ContactComputeCacheSerialize(struct SS *ss, const struct ds_ContactCompute *compute)
{
	for (u32 i = 0; i < compute->ccache_count; ++i)
	{
		ds_ContactConstraintCacheSerialize(ss, compute->ccache + i);
	}
}

u32 ds_ContactComputeCacheTryDeserialize(struct arena *mem, struct SS *ss, struct ds_ContactCompute *compute, struct ds_Dynamics *pipeline)
{
	/* cc is rebuilt every solve */
	compute->cc = NULL;
	compute->ccache = NULL;
	if (compute->ccache_count == 0)
	{
		return 1;
	}

	if (compute->ccache_count > SSBytesLeft(ss))
	{
		return 0;
	}

	compute->ccache = SnapshotPush(mem, pipeline, ss, (u64) compute->ccache_count*sizeof(struct ds_ContactConstraintCache));
	if (compute->ccache == NULL)
	{
		return 0;
	}

	for (u32 i = 0; i < compute->ccache_count; ++i)
	{
		if (!ds_ContactConstraintCacheTryDeserialize(ss, compute->ccache + i))
		{
			return 0;
		}
	}
	return 1;
}

/* a concave contact's tri cache: tri_count, then the entries */
static u64 ContactTriCacheSerializeSize(const struct ds_Contact *c)
{
	u64 size = sizeof(u32);
	for (u32 i = 0; i < c->cache.concave.tri_count; ++i)
	{
		size += ds_TriCacheSerializeSize(c->cache.concave.tri.buf + i);
	}
	return size;
}

static void ContactTriCacheSerialize(struct SS *ss, const struct ds_Contact *c)
{
	SSWriteU32Le(ss, c->cache.concave.tri_count);
	for (u32 i = 0; i < c->cache.concave.tri_count; ++i)
	{
		ds_TriCacheSerialize(ss, c->cache.concave.tri.buf + i);
	}
}

/* the cache must be empty */
static u32 ContactTriCacheTryDeserialize(struct SS *ss, struct ds_Dynamics *pipeline, struct ds_Contact *c)
{
	ds_Assert(c->cache.concave.tri.buf == NULL && c->cache.concave.tri_count == 0);
	if (SSBytesLeft(ss) < sizeof(u32))
	{
		return 0;
	}

	const u32 count = SSReadU32Le(ss);
	if (count > g_numerics_config->cache_count_max || count > SSBytesLeft(ss))
	{
		return 0;
	}

	if (count == 0)
	{
		return 1;
	}

	c->cache.concave.tri = ds_TSlabAdd(pipeline->tri_cache_slab, count);
	if (c->cache.concave.tri.buf == NULL)
	{
		return 0;
	}

	for (u32 i = 0; i < count; ++i)
	{
		if (!ds_TriCacheTryDeserialize(ss, c->cache.concave.tri.buf + i))
		{
			return 0;
		}
	}
	c->cache.concave.tri_count = count;
	return 1;
}

/*************************** solver sets ***************************/

u64 ds_SolverSetSerializeSize(const struct ds_Dynamics *pipeline, const u32 set_index)
{
	const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + set_index;
	u64 size = sizeof(u64)
		+ ds_BodySimCPoolSerializeSize(&set->body_sim_pool)
		+ ds_BodyComputeCPoolSerializeSize(&set->body_compute_pool)
		+ U32CPoolSerializeSize(&set->contact_pool)
		+ ds_ContactComputeCPoolSerializeSize(&set->contact_compute_pool)
		+ ds_JointSimCPoolSerializeSize(&set->joint_sim_pool)
		+ U32CPoolSerializeSize(&set->island_pool);

	for (u32 i = 0; i < set->contact_pool.count; ++i)
	{
		size += ds_ContactNarrowphaseSerializeSize(pipeline, set->contact_pool.buf[i]);
	}
	for (u32 i = 0; i < set->contact_compute_pool.count; ++i)
	{
		size += ds_ContactComputeCacheSerializeSize(set->contact_compute_pool.buf + i);
	}

	return size;
}

void ds_SolverSetSerialize(struct SS *ss, const struct ds_Dynamics *pipeline, const u32 set_index)
{
	ds_Assert(ds_SolverSetSerializeSize(pipeline, set_index) <= SSBytesLeft(ss));

	const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + set_index;
	ds_Assert((set_index >= SOLVER_SET_SLEEPING_FIRST) == (set->mem.mem_size != 0));
	/* sleeping sets own one heap arena holding their pools and narrowphase data */
	SSWriteU64Le(ss, set->mem.mem_size);
	ds_BodySimCPoolSerialize(ss, &set->body_sim_pool);
	ds_BodyComputeCPoolSerialize(ss, &set->body_compute_pool);
	U32CPoolSerialize(ss, &set->contact_pool);
	ds_ContactComputeCPoolSerialize(ss, &set->contact_compute_pool);
	ds_JointSimCPoolSerialize(ss, &set->joint_sim_pool);
	U32CPoolSerialize(ss, &set->island_pool);

	for (u32 i = 0; i < set->contact_pool.count; ++i)
	{
		ds_ContactNarrowphaseSerialize(ss, pipeline, set->contact_pool.buf[i]);
	}
	for (u32 i = 0; i < set->contact_compute_pool.count; ++i)
	{
		ds_ContactComputeCacheSerialize(ss, set->contact_compute_pool.buf + i);
	}
}

static void ds_SolverSetClear(struct ds_SolverSet *set)
{
	const u32 pool_slot = set->pool_slot;
	memset(set, 0, sizeof(*set));
	set->pool_slot = pool_slot;
}

u32 ds_SolverSetTryDeserialize(struct SS *ss, struct ds_Dynamics *pipeline, const u32 set_index)
{
	ds_Assert(set_index < pipeline->solver_set_pool.count_max);
	struct ds_SolverSet *set = pipeline->solver_set_pool.buf + set_index;
	ds_Assert(ds_PoolSlotAllocated(set) && set->mem.mem_size == 0 && set->body_sim_pool.buf == NULL);

	const u64 bit_index = ss->bit_index;
	if (SSBytesLeft(ss) < sizeof(u64))
	{
		return 0;
	}

	const u32 sleeping = (set_index >= SOLVER_SET_SLEEPING_FIRST);
	const u64 mem_size = SSReadU64Le(ss);
	if ( (sleeping && !mem_size) || (!sleeping && mem_size) )
	{
		ss->bit_index = bit_index;
		return 0;
	}

	struct arena *mem = NULL;
	if (sleeping)
	{
		set->mem = ArenaAlloc(NULL, mem_size);
		if (!set->mem.mem_size)
		{
			ds_SolverSetClear(set);
			ss->bit_index = bit_index;
			return 0;
		}
		mem = &set->mem;
	}

	const u32 growable = (mem) ? NOT_GROWABLE : GROWABLE;
	if (!ds_BodySimCPoolTryDeserialize(mem, ss, &set->body_sim_pool, growable)
		|| !ds_BodyComputeCPoolTryDeserialize(mem, ss, &set->body_compute_pool, growable)
		|| !U32CPoolTryDeserialize(mem, ss, &set->contact_pool, growable)
		|| !ds_ContactComputeCPoolTryDeserialize(mem, ss, &set->contact_compute_pool, growable)
		|| !ds_JointSimCPoolTryDeserialize(mem, ss, &set->joint_sim_pool, growable)
		|| !U32CPoolTryDeserialize(mem, ss, &set->island_pool, growable))
	{
		goto failure;
	}

	for (u32 i = 0; i < set->contact_pool.count; ++i)
	{
		const u32 contact = set->contact_pool.buf[i];
		if (contact >= pipeline->contact_pool.count_max
			|| !ds_PoolSlotAllocated(pipeline->contact_pool.buf + contact)
			|| !ds_ContactNarrowphaseTryDeserialize(mem, ss, pipeline, contact))
		{
			goto failure;
		}
	}

	for (u32 i = 0; i < set->contact_compute_pool.count; ++i)
	{
		if (!ds_ContactComputeCacheTryDeserialize(mem, ss, set->contact_compute_pool.buf + i, pipeline))
		{
			goto failure;
		}
	}

	return 1;

failure:
	/* frame data already pushed onto worker frames stays there until the frames are flushed */
	if (mem)
	{
		ArenaFree(&set->mem);
	}
	else
	{
		ds_CPoolDealloc(set->body_sim_pool);
		ds_CPoolDealloc(set->body_compute_pool);
		ds_CPoolDealloc(set->contact_pool);
		ds_CPoolDealloc(set->contact_compute_pool);
		ds_CPoolDealloc(set->joint_sim_pool);
		ds_CPoolDealloc(set->island_pool);
	}
	ds_SolverSetClear(set);
	ss->bit_index = bit_index;
	return 0;
}

/*************************** constraint graph ***************************/

u64 ds_CGraphColorSerializeSize(const struct ds_Dynamics *pipeline, const u32 color_index)
{
	const struct ds_CGraphColor *color = pipeline->cgraph.color + color_index;
	u64 size = ds_JointSimCPoolSerializeSize(&color->joint_sim_pool)
		+ U32CPoolSerializeSize(&color->contact_pool)
		+ ds_ContactComputeCPoolSerializeSize(&color->contact_compute_pool)
		+ ds_BitSetSerializeSize(&color->body_bitset);

	for (u32 i = 0; i < color->contact_pool.count; ++i)
	{
		size += ds_ContactNarrowphaseSerializeSize(pipeline, color->contact_pool.buf[i]);
	}
	for (u32 i = 0; i < color->contact_compute_pool.count; ++i)
	{
		size += ds_ContactComputeCacheSerializeSize(color->contact_compute_pool.buf + i);
	}

	return size;
}

void ds_CGraphColorSerialize(struct SS *ss, const struct ds_Dynamics *pipeline, const u32 color_index)
{
	ds_Assert(ds_CGraphColorSerializeSize(pipeline, color_index) <= SSBytesLeft(ss));

	const struct ds_CGraphColor *color = pipeline->cgraph.color + color_index;
	ds_JointSimCPoolSerialize(ss, &color->joint_sim_pool);
	U32CPoolSerialize(ss, &color->contact_pool);
	ds_ContactComputeCPoolSerialize(ss, &color->contact_compute_pool);
	ds_BitSetSerialize(ss, &color->body_bitset);

	for (u32 i = 0; i < color->contact_pool.count; ++i)
	{
		ds_ContactNarrowphaseSerialize(ss, pipeline, color->contact_pool.buf[i]);
	}
	for (u32 i = 0; i < color->contact_compute_pool.count; ++i)
	{
		ds_ContactComputeCacheSerialize(ss, color->contact_compute_pool.buf + i);
	}
}

u32 ds_CGraphColorTryDeserialize(struct SS *ss, struct ds_Dynamics *pipeline, const u32 color_index)
{
	struct ds_CGraphColor *color = pipeline->cgraph.color + color_index;
	memset(color, 0, sizeof(*color));
	const u64 bit_index = ss->bit_index;

	if (!ds_JointSimCPoolTryDeserialize(NULL, ss, &color->joint_sim_pool, GROWABLE)
		|| !U32CPoolTryDeserialize(NULL, ss, &color->contact_pool, GROWABLE)
		|| !ds_ContactComputeCPoolTryDeserialize(NULL, ss, &color->contact_compute_pool, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &color->body_bitset, GROWABLE))
	{
		goto failure;
	}

	for (u32 i = 0; i < color->contact_pool.count; ++i)
	{
		const u32 contact = color->contact_pool.buf[i];
		if (contact >= pipeline->contact_pool.count_max
			|| !ds_PoolSlotAllocated(pipeline->contact_pool.buf + contact)
			|| !ds_ContactNarrowphaseTryDeserialize(NULL, ss, pipeline, contact))
		{
			goto failure;
		}
	}

	for (u32 i = 0; i < color->contact_compute_pool.count; ++i)
	{
		if (!ds_ContactComputeCacheTryDeserialize(NULL, ss, color->contact_compute_pool.buf + i, pipeline))
		{
			goto failure;
		}
	}

	return 1;

failure:
	/* frame data already pushed onto worker frames stays there until the frames are flushed */
	ds_CGraphColorDealloc(color);
	ss->bit_index = bit_index;
	return 0;
}

u64 ds_CGraphSerializeSize(const struct ds_Dynamics *pipeline)
{
	u64 size = 0;
	for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
	{
		size += ds_CGraphColorSerializeSize(pipeline, i);
	}
	return size;
}

void ds_CGraphSerialize(struct SS *ss, const struct ds_Dynamics *pipeline)
{
	for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
	{
		ds_CGraphColorSerialize(ss, pipeline, i);
	}
}

u32 ds_CGraphTryDeserialize(struct SS *ss, struct ds_Dynamics *pipeline)
{
	const u64 bit_index = ss->bit_index;
	memset(&pipeline->cgraph, 0, sizeof(pipeline->cgraph));
	for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
	{
		if (!ds_CGraphColorTryDeserialize(ss, pipeline, i))
		{
			for (u32 j = 0; j < i; ++j)
			{
				ds_CGraphColorDealloc(pipeline->cgraph.color + j);
			}
			ss->bit_index = bit_index;
			return 0;
		}
	}
	return 1;
}

/*************************** pipeline ***************************/

/* Header: ns_tick, frames_completed, the applied config, island_to_split */
#define DS_SNAPSHOT_HEADER_FIXED_SIZE (3*sizeof(u64))

/*
 * Every contact's narrowphase data and every compute's ccache is serialized by its owner: cgraph
 * colors (touching, awake), the ACTIVE set (non-touching, awake) and the sleeping sets.
 */
u64 ds_DynamicsSerializeSize(const struct ds_Dynamics *pipeline)
{
	u64 size = DS_SNAPSHOT_HEADER_FIXED_SIZE + ds_DynamicsConfigSerializeBitPackedSize(&pipeline->config) / 8
		+ ds_BodyPoolSerializeSize(&pipeline->body_pool)
		+ ds_BitSetSerializeSize(&pipeline->body_usage_set)
		+ ds_JointPoolSerializeSize(&pipeline->joint_pool)
		+ ds_ShapePoolSerializeSize(&pipeline->shape_pool)
		+ ds_BitSetSerializeSize(&pipeline->shape_dynamic_usage_set)
		+ BVHSerializeSize(&pipeline->dynamic_bvh)
		+ BVHSerializeSize(&pipeline->static_bvh)
		+ ds_BitSetSerializeSize(&pipeline->shape_dirty_set)
		+ ds_ProxyQueryCPoolSerializeSize(&pipeline->dirty_shape_query)
		+ ds_PhysicsEventPoolSerializeSize(&pipeline->event_pool)
		+ DLLSerializeSize(pipeline->event_list)
		+ ds_ContactPoolSerializeSize(&pipeline->contact_pool)
		+ ds_HashMapSerializeSize(&pipeline->contact_map)
		+ ds_BitSetSerializeSize(&pipeline->contact_usage_set)
		+ ds_IslandPoolSerializeSize(&pipeline->island_pool)
		+ ds_BitSetSerializeSize(&pipeline->island_high_energy_set)
		+ ds_CGraphSerializeSize(pipeline)
		+ ds_SolverSetPoolSerializeSize(&pipeline->solver_set_pool);

	for (u32 i = 0; i < pipeline->contact_pool.count_max; ++i)
	{
		const struct ds_Contact *c = pipeline->contact_pool.buf + i;
		if (ds_PoolSlotAllocated(c) && !ds_ContactConvexCheck(c))
		{
			size += ContactTriCacheSerializeSize(c);
		}
	}

	for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
	{
		if (ds_PoolSlotAllocated(pipeline->solver_set_pool.buf + i))
		{
			size += ds_SolverSetSerializeSize(pipeline, i);
		}
	}

	return size;
}

void ds_DynamicsSerialize(struct SS *ss, const struct ds_Dynamics *pipeline)
{
	ds_Assert(ds_DynamicsSerializeSize(pipeline) <= SSBytesLeft(ss));

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

	SSWriteU64Le(ss, pipeline->ns_tick);
	SSWriteU64Le(ss, pipeline->frames_completed);
	ds_DynamicsConfigSerializeBitPacked(ss, &pipeline->config);
	SSWriteU64Le(ss, pipeline->island_to_split.val);

	ds_BodyPoolSerialize(ss, &pipeline->body_pool);
	ds_BitSetSerialize(ss, &pipeline->body_usage_set);
	ds_JointPoolSerialize(ss, &pipeline->joint_pool);
	ds_ShapePoolSerialize(ss, &pipeline->shape_pool);
	ds_BitSetSerialize(ss, &pipeline->shape_dynamic_usage_set);
	BVHSerialize(ss, &pipeline->dynamic_bvh);
	BVHSerialize(ss, &pipeline->static_bvh);
	ds_BitSetSerialize(ss, &pipeline->shape_dirty_set);
	ds_ProxyQueryCPoolSerialize(ss, &pipeline->dirty_shape_query);
	ds_PhysicsEventPoolSerialize(ss, &pipeline->event_pool);
	DLLSerialize(ss, pipeline->event_list);
	ds_ContactPoolSerialize(ss, &pipeline->contact_pool);
	for (u32 i = 0; i < pipeline->contact_pool.count_max; ++i)
	{
		const struct ds_Contact *c = pipeline->contact_pool.buf + i;
		if (ds_PoolSlotAllocated(c) && !ds_ContactConvexCheck(c))
		{
			ContactTriCacheSerialize(ss, c);
		}
	}
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

u32 ds_DynamicsTryDeserialize(struct arena *mem, struct SS *ss, struct ds_Dynamics *pipeline, const struct ds_DynamicsExternal *external)
{
	const u64 bit_index = ss->bit_index;
	const u64 mem_left = (mem) ? mem->mem_left : 0;
	*pipeline = (struct ds_Dynamics) { 0 };
	if (SSBytesLeft(ss) < DS_SNAPSHOT_HEADER_FIXED_SIZE)
	{
		return 0;
	}

	const u64 ns_tick = SSReadU64Le(ss);
	*pipeline = (struct ds_Dynamics)
	{
		.ns_tick = ns_tick,
        .timestep = (f32) ns_tick / NSEC_PER_SEC,
		.external = external,
		.island_to_split = ds_IslandIdNull(),
	};
	DLLFlush(pipeline->event_list);
    ds_DynamicsRuntimeAlloc(mem, &pipeline->runtime);
	if (!ds_TSlabAlloc(pipeline->tri_cache_slab, g_config->vm_size_tri_cache_allocator, g_config->thread_count))
	{
		goto failure;
	}
	pipeline->frames_completed = SSReadU64Le(ss);
	if (!ds_DynamicsConfigTryDeserializeBitPacked(ss, &pipeline->config))
	{
		goto failure;
	}
	pipeline->config_pending = pipeline->config;
	pipeline->island_to_split = (ds_IslandId) { .val = SSReadU64Le(ss) };

	/* frame data goes into the buffer the next tick reads (and does not flush) */
	const u64 f = pipeline->frames_completed & 0x1;
	for (u32 i = 0; i < pipeline->runtime.worker_count; ++i)
	{
		pipeline->runtime.worker[i].frame = pipeline->runtime.worker[i].frame_arr + f;
	}

	/* the contact pool precedes its owners, which check their contact indices against it */
	if (!ds_BodyPoolTryDeserialize(ss, &pipeline->body_pool)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->body_usage_set, GROWABLE)
		|| !ds_JointPoolTryDeserialize(ss, &pipeline->joint_pool)
		|| !ds_ShapePoolTryDeserialize(ss, &pipeline->shape_pool)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->shape_dynamic_usage_set, GROWABLE)
		|| !BVHTryDeserialize(NULL, ss, &pipeline->dynamic_bvh, GROWABLE)
		|| !BVHTryDeserialize(NULL, ss, &pipeline->static_bvh, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->shape_dirty_set, GROWABLE)
		|| !ds_ProxyQueryCPoolTryDeserialize(NULL, ss, &pipeline->dirty_shape_query, GROWABLE)
		|| !ds_PhysicsEventPoolTryDeserialize(ss, &pipeline->event_pool)
		|| !DLLTryDeserialize(ss, pipeline->event_list)
		|| !ds_ContactPoolTryDeserialize(ss, &pipeline->contact_pool))
	{
		goto failure;
	}

	/* the contacts' data arrays are restored by their owners: empty them first, so ds_DynamicsDealloc is safe after a failure */
	for (u32 i = 0; i < pipeline->contact_pool.count_max; ++i)
	{
		struct ds_Contact *c = pipeline->contact_pool.buf + i;
		if (ds_PoolSlotAllocated(c))
		{
			c->narrowphase.manifold = NULL;
			c->narrowphase.tri = NULL;
			if (!ds_ContactConvexCheck(c))
			{
				c->cache.concave = (struct ds_ConcaveCache) { .tri = { .buf = NULL, .length = 0, .base = U32_MAX }, .tri_count = 0 };
			}
		}
	}

	for (u32 i = 0; i < pipeline->contact_pool.count_max; ++i)
	{
		struct ds_Contact *c = pipeline->contact_pool.buf + i;
		if (ds_PoolSlotAllocated(c) && !ds_ContactConvexCheck(c) && !ContactTriCacheTryDeserialize(ss, pipeline, c))
		{
			goto failure;
		}
	}

	if (!ds_HashMapTryDeserialize(NULL, ss, &pipeline->contact_map, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->contact_usage_set, GROWABLE)
		|| !ds_IslandPoolTryDeserialize(ss, &pipeline->island_pool)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->island_high_energy_set, GROWABLE)
		|| !ds_CGraphTryDeserialize(ss, pipeline)
		|| !ds_SolverSetPoolTryDeserialize(ss, &pipeline->solver_set_pool))
	{
		goto failure;
	}

	for (u32 i = 0; i < pipeline->runtime.worker_count; ++i)
	{
		pipeline->runtime.worker[i].joint_breaking_set = ds_BitSetAlloc(NULL, pipeline->joint_pool.length, 0, GROWABLE);
	}

	/* the set slots hold their pools: clear all of them first, so ds_DynamicsDealloc is safe after a failure */
	for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
	{
		struct ds_SolverSet *set = pipeline->solver_set_pool.buf + i;
		if (ds_PoolSlotAllocated(set))
		{
			ds_SolverSetClear(set);
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
	ds_DynamicsDealloc(pipeline);
	if (mem)
	{
		ArenaPopPacked(mem, mem_left - mem->mem_left);
	}
	*pipeline = (struct ds_Dynamics) { 0 };
	ss->bit_index = bit_index;
	return 0;
}
