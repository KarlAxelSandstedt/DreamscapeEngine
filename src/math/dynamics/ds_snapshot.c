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

/*************************** fields ***************************/

#define SNAPSHOT_V3_SIZE                (3*sizeof(f32))
#define SNAPSHOT_V3F64_SIZE             (3*sizeof(f64))
#define SNAPSHOT_V4_SIZE                (4*sizeof(f32))
#define SNAPSHOT_Q_SIZE                 (4*sizeof(f32))
#define SNAPSHOT_M3_SIZE                (9*sizeof(f32))
#define SNAPSHOT_TRANSFORM_SIZE         (SNAPSHOT_Q_SIZE + SNAPSHOT_V3_SIZE)
#define SNAPSHOT_TRANSFORM_WORLD_SIZE   (SNAPSHOT_Q_SIZE + SNAPSHOT_V3F64_SIZE)
#define SNAPSHOT_DLL_SIZE               (3*sizeof(u32))
#define SNAPSHOT_DLL_NODE_SIZE          (2*sizeof(i32))
#define SNAPSHOT_ID_SIZE                sizeof(u64)

/* component by component: the LeN macros read f32 storage through a b32 pointer, which the optimizer may not honour */
static ds_ForceInline void WriteF32N(struct SS *ss, const f32 *buf, const u32 count)
{
	for (u32 i = 0; i < count; ++i)
	{
		SSWriteF32Le(ss, buf[i]);
	}
}

static ds_ForceInline void ReadF32N(struct SS *ss, f32 *buf, const u32 count)
{
	for (u32 i = 0; i < count; ++i)
	{
		buf[i] = SSReadF32Le(ss);
	}
}

static ds_ForceInline void WriteV3(struct SS *ss, const v3 v)
{
	WriteF32N(ss, v.buf, 3);
}

static ds_ForceInline v3 ReadV3(struct SS *ss)
{
	v3 v;
	ReadF32N(ss, v.buf, 3);
	return v;
}

static ds_ForceInline void WriteV3F64(struct SS *ss, const v3f64 v)
{
	SSWriteF64Le(ss, v.x);
	SSWriteF64Le(ss, v.y);
	SSWriteF64Le(ss, v.z);
}

static ds_ForceInline v3f64 ReadV3F64(struct SS *ss)
{
	v3f64 v;
	v.x = SSReadF64Le(ss);
	v.y = SSReadF64Le(ss);
	v.z = SSReadF64Le(ss);
	return v;
}

static ds_ForceInline void WriteV4(struct SS *ss, const v4 v)
{
	WriteF32N(ss, v.buf, 4);
}

static ds_ForceInline v4 ReadV4(struct SS *ss)
{
	v4 v;
	ReadF32N(ss, v.buf, 4);
	return v;
}

static ds_ForceInline void WriteQ(struct SS *ss, const q a)
{
	WriteF32N(ss, a.buf, 4);
}

static ds_ForceInline q ReadQ(struct SS *ss)
{
	q a;
	ReadF32N(ss, a.buf, 4);
	return a;
}

static ds_ForceInline void WriteM3(struct SS *ss, const m3 *m)
{
	WriteF32N(ss, m->buf, 9);
}

static ds_ForceInline void ReadM3(struct SS *ss, m3 *m)
{
	ReadF32N(ss, m->buf, 9);
}

static ds_ForceInline void WriteTransform(struct SS *ss, const ds_Transform *t)
{
	WriteQ(ss, t->rotation);
	WriteV3(ss, t->position);
}

static ds_ForceInline void ReadTransform(struct SS *ss, ds_Transform *t)
{
	t->rotation = ReadQ(ss);
	t->position = ReadV3(ss);
}

static ds_ForceInline void WriteTransformWorld(struct SS *ss, const ds_TransformWorld *t)
{
	WriteQ(ss, t->rotation);
	WriteV3F64(ss, t->position);
}

static ds_ForceInline void ReadTransformWorld(struct SS *ss, ds_TransformWorld *t)
{
	t->rotation = ReadQ(ss);
	t->position = ReadV3F64(ss);
}

static ds_ForceInline void WriteDLLNode(struct SS *ss, const struct DLLNode node)
{
	SSWriteI32Le(ss, node.prev);
	SSWriteI32Le(ss, node.next);
}

static ds_ForceInline struct DLLNode ReadDLLNode(struct SS *ss)
{
	struct DLLNode node;
	node.prev = SSReadI32Le(ss);
	node.next = SSReadI32Le(ss);
	return node;
}

/*************************** per-type serializers ***************************/

#define DS_BODY_SERIALIZE_SIZE (SNAPSHOT_DLL_NODE_SIZE + SNAPSHOT_ID_SIZE + 4*sizeof(u32) + sizeof(f32) + 2*SNAPSHOT_DLL_SIZE + sizeof(f32) + sizeof(u32))

u64 ds_BodySerializeSize(const struct ds_Body *body)
{
	return DS_BODY_SERIALIZE_SIZE;
}

void ds_BodySerialize(struct SS *ss, const struct ds_Body *body)
{
	WriteDLLNode(ss, body->island_body);
	SSWriteU64Le(ss, body->id.val);
	SSWriteU32Le(ss, body->flags);
	SSWriteU32Le(ss, body->island);
	SSWriteU32Le(ss, body->set);
	SSWriteU32Le(ss, body->sim);
	SSWriteF32Le(ss, body->low_velocity_time);
	DLLSerialize(ss, body->joint_list);
	DLLSerialize(ss, body->shape_list);
	SSWriteF32Le(ss, body->mass);
	SSWriteU32Le(ss, body->entity);
}

u32 ds_BodyTryDeserialize(struct SS *ss, struct ds_Body *body)
{
	if (SSBytesLeft(ss) < DS_BODY_SERIALIZE_SIZE)
	{
		return 0;
	}

	body->island_body = ReadDLLNode(ss);
	body->id.val = SSReadU64Le(ss);
	body->flags = SSReadU32Le(ss);
	body->island = SSReadU32Le(ss);
	body->set = SSReadU32Le(ss);
	body->sim = SSReadU32Le(ss);
	body->low_velocity_time = SSReadF32Le(ss);
	DLLDeserialize(ss, body->joint_list);
	DLLDeserialize(ss, body->shape_list);
	body->mass = SSReadF32Le(ss);
	body->entity = SSReadU32Le(ss);
	return 1;
}

#define DS_SHAPE_SERIALIZE_SIZE (SNAPSHOT_DLL_NODE_SIZE + SNAPSHOT_ID_SIZE + 3*sizeof(u32) + SNAPSHOT_DLL_SIZE + 2*sizeof(u32) + 4*sizeof(f32) + SNAPSHOT_TRANSFORM_SIZE)

u64 ds_ShapeSerializeSize(const struct ds_Shape *shape)
{
	return DS_SHAPE_SERIALIZE_SIZE;
}

void ds_ShapeSerialize(struct SS *ss, const struct ds_Shape *shape)
{
	WriteDLLNode(ss, shape->body_shape);
	SSWriteU64Le(ss, shape->id.val);
	SSWriteU32Le(ss, shape->flags);
	SSWriteU32Le(ss, shape->body);
	SSWriteU32Le(ss, shape->proxy);
	DLLSerialize(ss, shape->contact_list);
	SSWriteU32Le(ss, (u32) shape->cshape_type);
	SSWriteU32Le(ss, shape->cshape_handle);
	SSWriteF32Le(ss, shape->density);
	SSWriteF32Le(ss, shape->restitution);
	SSWriteF32Le(ss, shape->friction);
	SSWriteF32Le(ss, shape->margin);
	WriteTransform(ss, &shape->t_local);
}

u32 ds_ShapeTryDeserialize(struct SS *ss, struct ds_Shape *shape)
{
	if (SSBytesLeft(ss) < DS_SHAPE_SERIALIZE_SIZE)
	{
		return 0;
	}

	shape->body_shape = ReadDLLNode(ss);
	shape->id.val = SSReadU64Le(ss);
	shape->flags = SSReadU32Le(ss);
	shape->body = SSReadU32Le(ss);
	shape->proxy = SSReadU32Le(ss);
	DLLDeserialize(ss, shape->contact_list);
	shape->cshape_type = (enum c_ShapeType) SSReadU32Le(ss);
	shape->cshape_handle = SSReadU32Le(ss);
	shape->density = SSReadF32Le(ss);
	shape->restitution = SSReadF32Le(ss);
	shape->friction = SSReadF32Le(ss);
	shape->margin = SSReadF32Le(ss);
	ReadTransform(ss, &shape->t_local);
	return shape->cshape_type < C_SHAPE_COUNT;
}

#define DS_JOINT_SERIALIZE_SIZE (SNAPSHOT_ID_SIZE + 2*sizeof(u32) + SNAPSHOT_DLL_NODE_SIZE + 2*sizeof(u32) + 2*SNAPSHOT_DLL_NODE_SIZE + 4*sizeof(u32))

u64 ds_JointSerializeSize(const struct ds_Joint *joint)
{
	return DS_JOINT_SERIALIZE_SIZE;
}

void ds_JointSerialize(struct SS *ss, const struct ds_Joint *joint)
{
	SSWriteU64Le(ss, joint->id.val);
	SSWriteU32Le(ss, (u32) joint->type);
	SSWriteU32Le(ss, joint->island);
	WriteDLLNode(ss, joint->island_joint);
	SSWriteU32LeN(ss, joint->body, 2);
	WriteDLLNode(ss, joint->edge_node[0]);
	WriteDLLNode(ss, joint->edge_node[1]);
	SSWriteU32Le(ss, joint->set);
	SSWriteU32Le(ss, joint->color);
	SSWriteU32Le(ss, joint->sim);
	SSWriteU32Le(ss, joint->collide_connected);
}

u32 ds_JointTryDeserialize(struct SS *ss, struct ds_Joint *joint)
{
	if (SSBytesLeft(ss) < DS_JOINT_SERIALIZE_SIZE)
	{
		return 0;
	}

	joint->id.val = SSReadU64Le(ss);
	joint->type = (enum ds_JointType) SSReadU32Le(ss);
	joint->island = SSReadU32Le(ss);
	joint->island_joint = ReadDLLNode(ss);
	SSReadU32LeN(ss, joint->body, 2);
	joint->edge_node[0] = ReadDLLNode(ss);
	joint->edge_node[1] = ReadDLLNode(ss);
	joint->set = SSReadU32Le(ss);
	joint->color = SSReadU32Le(ss);
	joint->sim = SSReadU32Le(ss);
	joint->collide_connected = SSReadU32Le(ss);
	return joint->type < DS_JOINT_TYPE_COUNT;
}

/* the union member the event's type fills (the editor reads them so) */
static u64 PhysicsEventPayloadSize(const enum ds_PhysicsEventType type)
{
	switch (type)
	{
		case PHYSICS_EVENT_CONTACT_NEW:         return SNAPSHOT_ID_SIZE;
		case PHYSICS_EVENT_CONTACT_REMOVED:     return 4*SNAPSHOT_ID_SIZE;
		case PHYSICS_EVENT_ISLAND_NEW:
		case PHYSICS_EVENT_ISLAND_EXPANDED:
		case PHYSICS_EVENT_ISLAND_REMOVED:
		case PHYSICS_EVENT_ISLAND_AWAKE:
		case PHYSICS_EVENT_ISLAND_ASLEEP:       return SNAPSHOT_ID_SIZE;
		case PHYSICS_EVENT_BODY_NEW:
		case PHYSICS_EVENT_BODY_REMOVED:        return sizeof(u32);
		case PHYSICS_EVENT_JOINT_BREAKING:      return SNAPSHOT_ID_SIZE;
		default:                                return 0;
	}
}

#define DS_PHYSICS_EVENT_FIXED_SERIALIZE_SIZE (SNAPSHOT_DLL_NODE_SIZE + sizeof(u64) + sizeof(u32))

u64 ds_PhysicsEventSerializeSize(const struct ds_PhysicsEvent *event)
{
	return DS_PHYSICS_EVENT_FIXED_SERIALIZE_SIZE + PhysicsEventPayloadSize(event->type);
}

void ds_PhysicsEventSerialize(struct SS *ss, const struct ds_PhysicsEvent *event)
{
	WriteDLLNode(ss, event->node);
	SSWriteU64Le(ss, event->ns);
	SSWriteU32Le(ss, (u32) event->type);
	switch (event->type)
	{
		case PHYSICS_EVENT_CONTACT_NEW:
		{
			SSWriteU64Le(ss, event->contact.val);
		} break;

		case PHYSICS_EVENT_CONTACT_REMOVED:
		{
			SSWriteU64Le(ss, event->contact_removed_bodies[0].val);
			SSWriteU64Le(ss, event->contact_removed_bodies[1].val);
			SSWriteU64Le(ss, event->contact_removed_shapes[0].val);
			SSWriteU64Le(ss, event->contact_removed_shapes[1].val);
		} break;

		case PHYSICS_EVENT_ISLAND_NEW:
		case PHYSICS_EVENT_ISLAND_EXPANDED:
		case PHYSICS_EVENT_ISLAND_REMOVED:
		case PHYSICS_EVENT_ISLAND_AWAKE:
		case PHYSICS_EVENT_ISLAND_ASLEEP:
		{
			SSWriteU64Le(ss, event->island.val);
		} break;

		case PHYSICS_EVENT_BODY_NEW:
		case PHYSICS_EVENT_BODY_REMOVED:
		{
			SSWriteU32Le(ss, event->entity);
		} break;

		case PHYSICS_EVENT_JOINT_BREAKING:
		{
			SSWriteU64Le(ss, event->joint.val);
		} break;

		default: break;
	}
}

u32 ds_PhysicsEventTryDeserialize(struct SS *ss, struct ds_PhysicsEvent *event)
{
	memset(event, 0, sizeof(*event));
	if (SSBytesLeft(ss) < DS_PHYSICS_EVENT_FIXED_SERIALIZE_SIZE)
	{
		return 0;
	}

	event->node = ReadDLLNode(ss);
	event->ns = SSReadU64Le(ss);
	event->type = (enum ds_PhysicsEventType) SSReadU32Le(ss);
	if (event->type >= PHYSICS_EVENT_COUNT || SSBytesLeft(ss) < PhysicsEventPayloadSize(event->type))
	{
		return 0;
	}

	switch (event->type)
	{
		case PHYSICS_EVENT_CONTACT_NEW:
		{
			event->contact.val = SSReadU64Le(ss);
		} break;

		case PHYSICS_EVENT_CONTACT_REMOVED:
		{
			event->contact_removed_bodies[0].val = SSReadU64Le(ss);
			event->contact_removed_bodies[1].val = SSReadU64Le(ss);
			event->contact_removed_shapes[0].val = SSReadU64Le(ss);
			event->contact_removed_shapes[1].val = SSReadU64Le(ss);
		} break;

		case PHYSICS_EVENT_ISLAND_NEW:
		case PHYSICS_EVENT_ISLAND_EXPANDED:
		case PHYSICS_EVENT_ISLAND_REMOVED:
		case PHYSICS_EVENT_ISLAND_AWAKE:
		case PHYSICS_EVENT_ISLAND_ASLEEP:
		{
			event->island.val = SSReadU64Le(ss);
		} break;

		case PHYSICS_EVENT_BODY_NEW:
		case PHYSICS_EVENT_BODY_REMOVED:
		{
			event->entity = SSReadU32Le(ss);
		} break;

		case PHYSICS_EVENT_JOINT_BREAKING:
		{
			event->joint.val = SSReadU64Le(ss);
		} break;

		default: break;
	}
	return 1;
}

/* the manifold buffer and a concave cache's array are restored after the contact pool; a convex cache is part of the contact */
#define DS_CONTACT_FIXED_SERIALIZE_SIZE (SNAPSHOT_ID_SIZE + 2*sizeof(u32) + 5*sizeof(u32) + 4*SNAPSHOT_DLL_NODE_SIZE + SNAPSHOT_V3_SIZE)

u64 ds_ContactSerializeSize(const struct ds_Contact *contact)
{
	u64 size = DS_CONTACT_FIXED_SERIALIZE_SIZE;
	if (ds_ContactConvexCheck(contact))
	{
		size += ds_ConvexCacheSerializeSize(&contact->cache.convex);
	}
	return size;
}

void ds_ContactSerialize(struct SS *ss, const struct ds_Contact *contact)
{
	SSWriteU64Le(ss, contact->id.val);
	SSWriteU32LeN(ss, contact->key.shape, 2);
	SSWriteU32Le(ss, contact->flags);
	SSWriteU32Le(ss, contact->island);
	SSWriteU32Le(ss, contact->set);
	SSWriteU32Le(ss, contact->compute);
	SSWriteU32Le(ss, contact->color);
	WriteDLLNode(ss, contact->island_joint);
	WriteDLLNode(ss, contact->island_contact);
	WriteDLLNode(ss, contact->shape_contact[0]);
	WriteDLLNode(ss, contact->shape_contact[1]);
	WriteV3(ss, contact->com_offset);
	if (ds_ContactConvexCheck(contact))
	{
		ds_ConvexCacheSerialize(ss, &contact->cache.convex);
	}
}

u32 ds_ContactTryDeserialize(struct SS *ss, struct ds_Contact *contact)
{
	if (SSBytesLeft(ss) < DS_CONTACT_FIXED_SERIALIZE_SIZE)
	{
		return 0;
	}

	contact->id.val = SSReadU64Le(ss);
	SSReadU32LeN(ss, contact->key.shape, 2);
	contact->flags = SSReadU32Le(ss);
	contact->island = SSReadU32Le(ss);
	contact->set = SSReadU32Le(ss);
	contact->compute = SSReadU32Le(ss);
	contact->color = SSReadU32Le(ss);
	contact->island_joint = ReadDLLNode(ss);
	contact->island_contact = ReadDLLNode(ss);
	contact->shape_contact[0] = ReadDLLNode(ss);
	contact->shape_contact[1] = ReadDLLNode(ss);
	contact->com_offset = ReadV3(ss);
	if (ds_ContactConvexCheck(contact))
	{
		return ds_ConvexCacheTryDeserialize(ss, &contact->cache.convex);
	}

	contact->cache.concave = (struct ds_ConcaveCache) { .tri = { .buf = NULL, .length = 0, .base = U32_MAX }, .tri_count = 0 };
	return 1;
}

#define DS_ISLAND_SERIALIZE_SIZE (SNAPSHOT_ID_SIZE + 3*sizeof(u32) + 3*SNAPSHOT_DLL_SIZE + SNAPSHOT_V4_SIZE)

u64 ds_IslandSerializeSize(const struct ds_Island *island)
{
	return DS_ISLAND_SERIALIZE_SIZE;
}

void ds_IslandSerialize(struct SS *ss, const struct ds_Island *island)
{
	SSWriteU64Le(ss, island->id.val);
	SSWriteU32Le(ss, island->constraint_remove_count);
	SSWriteU32Le(ss, island->set);
	SSWriteU32Le(ss, island->set_island_index);
	DLLSerialize(ss, island->body_list);
	DLLSerialize(ss, island->contact_list);
	DLLSerialize(ss, island->joint_list);
	WriteV4(ss, island->color);
}

u32 ds_IslandTryDeserialize(struct SS *ss, struct ds_Island *island)
{
	if (SSBytesLeft(ss) < DS_ISLAND_SERIALIZE_SIZE)
	{
		return 0;
	}

	island->id.val = SSReadU64Le(ss);
	island->constraint_remove_count = SSReadU32Le(ss);
	island->set = SSReadU32Le(ss);
	island->set_island_index = SSReadU32Le(ss);
	DLLDeserialize(ss, island->body_list);
	DLLDeserialize(ss, island->contact_list);
	DLLDeserialize(ss, island->joint_list);
	island->color = ReadV4(ss);
	return 1;
}

/* per-tick frame data, rebuilt by the broadphase before it is read: one empty slot per shape */
u64 ds_ProxyQuerySerializeSize(const struct ds_ProxyQuery *query)
{
	return 0;
}

void ds_ProxyQuerySerialize(struct SS *ss, const struct ds_ProxyQuery *query)
{
}

u32 ds_ProxyQueryTryDeserialize(struct SS *ss, struct ds_ProxyQuery *query)
{
	*query = (struct ds_ProxyQuery) { 0 };
	return 1;
}

#define DS_BODY_SIM_SERIALIZE_SIZE (2*sizeof(u32) + sizeof(f32) + SNAPSHOT_TRANSFORM_WORLD_SIZE + SNAPSHOT_V3_SIZE + SNAPSHOT_V3F64_SIZE + 2*SNAPSHOT_M3_SIZE)

u64 ds_BodySimSerializeSize(const struct ds_BodySim *bsim)
{
	return DS_BODY_SIM_SERIALIZE_SIZE;
}

void ds_BodySimSerialize(struct SS *ss, const struct ds_BodySim *bsim)
{
	SSWriteU32Le(ss, bsim->body);
	SSWriteU32Le(ss, bsim->flags);
	SSWriteF32Le(ss, bsim->inv_mass);
	WriteTransformWorld(ss, &bsim->world);
	WriteV3(ss, bsim->local_center_of_mass);
	WriteV3F64(ss, bsim->world_center_of_mass);
	WriteM3(ss, &bsim->local_inv_inertia);
	WriteM3(ss, &bsim->world_inv_inertia);
}

u32 ds_BodySimTryDeserialize(struct SS *ss, struct ds_BodySim *bsim)
{
	if (SSBytesLeft(ss) < DS_BODY_SIM_SERIALIZE_SIZE)
	{
		return 0;
	}

	bsim->body = SSReadU32Le(ss);
	bsim->flags = SSReadU32Le(ss);
	bsim->inv_mass = SSReadF32Le(ss);
	ReadTransformWorld(ss, &bsim->world);
	bsim->local_center_of_mass = ReadV3(ss);
	bsim->world_center_of_mass = ReadV3F64(ss);
	ReadM3(ss, &bsim->local_inv_inertia);
	ReadM3(ss, &bsim->world_inv_inertia);
	return 1;
}

#define DS_BODY_COMPUTE_SERIALIZE_SIZE (3*SNAPSHOT_V3_SIZE + SNAPSHOT_Q_SIZE + sizeof(u32))

u64 ds_BodyComputeSerializeSize(const struct ds_BodyCompute *bcomp)
{
	return DS_BODY_COMPUTE_SERIALIZE_SIZE;
}

void ds_BodyComputeSerialize(struct SS *ss, const struct ds_BodyCompute *bcomp)
{
	WriteV3(ss, bcomp->linear_velocity);
	WriteV3(ss, bcomp->angular_velocity);
	WriteV3(ss, bcomp->delta_position);
	WriteQ(ss, bcomp->rotation);
	SSWriteU32Le(ss, bcomp->flags);
}

u32 ds_BodyComputeTryDeserialize(struct SS *ss, struct ds_BodyCompute *bcomp)
{
	if (SSBytesLeft(ss) < DS_BODY_COMPUTE_SERIALIZE_SIZE)
	{
		return 0;
	}

	bcomp->linear_velocity = ReadV3(ss);
	bcomp->angular_velocity = ReadV3(ss);
	bcomp->delta_position = ReadV3(ss);
	bcomp->rotation = ReadQ(ss);
	bcomp->flags = SSReadU32Le(ss);
	return 1;
}

/* cc is rebuilt every solve */
#define DS_CONTACT_COMPUTE_SERIALIZE_SIZE (3*sizeof(u32))

u64 ds_ContactComputeSerializeSize(const struct ds_ContactCompute *compute)
{
	return DS_CONTACT_COMPUTE_SERIALIZE_SIZE;
}

void ds_ContactComputeSerialize(struct SS *ss, const struct ds_ContactCompute *compute)
{
	SSWriteU32Le(ss, compute->cc_count);
	SSWriteU32LeN(ss, compute->body_sim, 2);
}

u32 ds_ContactComputeTryDeserialize(struct SS *ss, struct ds_ContactCompute *compute)
{
	if (SSBytesLeft(ss) < DS_CONTACT_COMPUTE_SERIALIZE_SIZE)
	{
		return 0;
	}

	compute->cc = NULL;
	compute->cc_count = SSReadU32Le(ss);
	SSReadU32LeN(ss, compute->body_sim, 2);
	return 1;
}

#define DS_JOINT_SIM_FIXED_SERIALIZE_SIZE    (2*SNAPSHOT_TRANSFORM_SIZE + 2*sizeof(f32) + 2*sizeof(u32) + 2*sizeof(u32) + 3*SNAPSHOT_V3_SIZE)
#define DS_DISTANCE_JOINT_SERIALIZE_SIZE     (3*sizeof(f32) + sizeof(u32) + 2*sizeof(f32) + SNAPSHOT_V3_SIZE + 8*sizeof(f32) + 3*sizeof(f32))
#define DS_MOUSE_JOINT_SERIALIZE_SIZE        (SNAPSHOT_V3F64_SIZE + 4*sizeof(f32) + 2*SNAPSHOT_M3_SIZE + SNAPSHOT_V3_SIZE + SNAPSHOT_M3_SIZE + 2*sizeof(f32) + 2*SNAPSHOT_V3_SIZE)

static u64 JointSimPayloadSize(const enum ds_JointType type)
{
	switch (type)
	{
		case DS_JOINT_TYPE_DISTANCE:    return DS_DISTANCE_JOINT_SERIALIZE_SIZE;
		case DS_JOINT_TYPE_MOUSE:       return DS_MOUSE_JOINT_SERIALIZE_SIZE;
		default:                        return 0;
	}
}

u64 ds_JointSimSerializeSize(const struct ds_JointSim *jsim)
{
	return DS_JOINT_SIM_FIXED_SERIALIZE_SIZE + JointSimPayloadSize(jsim->type);
}

void ds_JointSimSerialize(struct SS *ss, const struct ds_JointSim *jsim)
{
	WriteTransform(ss, &jsim->local_frame[0]);
	WriteTransform(ss, &jsim->local_frame[1]);
	SSWriteF32Le(ss, jsim->force_threshold);
	SSWriteF32Le(ss, jsim->torque_threshold);
	SSWriteU32Le(ss, jsim->joint);
	SSWriteU32Le(ss, (u32) jsim->type);
	SSWriteU32LeN(ss, jsim->body_sim, 2);
	WriteV3(ss, jsim->r[0]);
	WriteV3(ss, jsim->r[1]);
	WriteV3(ss, jsim->com_diff);
	switch (jsim->type)
	{
		case DS_JOINT_TYPE_DISTANCE:
		{
			const struct ds_DistanceJoint *d = &jsim->distance;
			SSWriteF32Le(ss, d->target);
			WriteF32N(ss, d->range.v, 2);
			SSWriteU32Le(ss, d->enable_spring);
			SSWriteF32Le(ss, d->hertz);
			SSWriteF32Le(ss, d->damping_ratio);
			WriteV3(ss, d->n);
			SSWriteF32Le(ss, d->distance);
			SSWriteF32Le(ss, d->normal_mass);
			SSWriteF32Le(ss, d->soft_mass);
			SSWriteF32Le(ss, d->target_mass);
			SSWriteF32Le(ss, d->gamma);
			SSWriteF32Le(ss, d->bias_velocity);
			SSWriteF32Le(ss, d->allowed_separation_velocity_min);
			SSWriteF32Le(ss, d->allowed_separation_velocity_max);
			SSWriteF32Le(ss, d->impulse);
			SSWriteF32Le(ss, d->impulse_low);
			SSWriteF32Le(ss, d->impulse_high);
		} break;

		case DS_JOINT_TYPE_MOUSE:
		{
			const struct ds_MouseJoint *m = &jsim->mouse;
			WriteV3F64(ss, m->target);
			SSWriteF32Le(ss, m->hertz);
			SSWriteF32Le(ss, m->damping_ratio);
			SSWriteF32Le(ss, m->max_force);
			SSWriteF32Le(ss, m->max_torque);
			WriteM3(ss, &m->soft_mass);
			WriteM3(ss, &m->gamma);
			WriteV3(ss, m->bias_velocity);
			WriteM3(ss, &m->angular_mass);
			SSWriteF32Le(ss, m->max_impulse);
			SSWriteF32Le(ss, m->max_angular_impulse);
			WriteV3(ss, m->impulse);
			WriteV3(ss, m->angular_impulse);
		} break;

		default: break;
	}
}

u32 ds_JointSimTryDeserialize(struct SS *ss, struct ds_JointSim *jsim)
{
	memset(jsim, 0, sizeof(*jsim));
	if (SSBytesLeft(ss) < DS_JOINT_SIM_FIXED_SERIALIZE_SIZE)
	{
		return 0;
	}

	ReadTransform(ss, &jsim->local_frame[0]);
	ReadTransform(ss, &jsim->local_frame[1]);
	jsim->force_threshold = SSReadF32Le(ss);
	jsim->torque_threshold = SSReadF32Le(ss);
	jsim->joint = SSReadU32Le(ss);
	jsim->type = (enum ds_JointType) SSReadU32Le(ss);
	SSReadU32LeN(ss, jsim->body_sim, 2);
	jsim->r[0] = ReadV3(ss);
	jsim->r[1] = ReadV3(ss);
	jsim->com_diff = ReadV3(ss);
	if (jsim->type >= DS_JOINT_TYPE_COUNT || SSBytesLeft(ss) < JointSimPayloadSize(jsim->type))
	{
		return 0;
	}

	switch (jsim->type)
	{
		case DS_JOINT_TYPE_DISTANCE:
		{
			struct ds_DistanceJoint *d = &jsim->distance;
			d->target = SSReadF32Le(ss);
			ReadF32N(ss, d->range.v, 2);
			d->enable_spring = SSReadU32Le(ss);
			d->hertz = SSReadF32Le(ss);
			d->damping_ratio = SSReadF32Le(ss);
			d->n = ReadV3(ss);
			d->distance = SSReadF32Le(ss);
			d->normal_mass = SSReadF32Le(ss);
			d->soft_mass = SSReadF32Le(ss);
			d->target_mass = SSReadF32Le(ss);
			d->gamma = SSReadF32Le(ss);
			d->bias_velocity = SSReadF32Le(ss);
			d->allowed_separation_velocity_min = SSReadF32Le(ss);
			d->allowed_separation_velocity_max = SSReadF32Le(ss);
			d->impulse = SSReadF32Le(ss);
			d->impulse_low = SSReadF32Le(ss);
			d->impulse_high = SSReadF32Le(ss);
		} break;

		case DS_JOINT_TYPE_MOUSE:
		{
			struct ds_MouseJoint *m = &jsim->mouse;
			m->target = ReadV3F64(ss);
			m->hertz = SSReadF32Le(ss);
			m->damping_ratio = SSReadF32Le(ss);
			m->max_force = SSReadF32Le(ss);
			m->max_torque = SSReadF32Le(ss);
			ReadM3(ss, &m->soft_mass);
			ReadM3(ss, &m->gamma);
			m->bias_velocity = ReadV3(ss);
			ReadM3(ss, &m->angular_mass);
			m->max_impulse = SSReadF32Le(ss);
			m->max_angular_impulse = SSReadF32Le(ss);
			m->impulse = ReadV3(ss);
			m->angular_impulse = ReadV3(ss);
		} break;

		default: break;
	}
	return 1;
}

#define DS_SOLVER_MANIFOLD_SERIALIZE_SIZE (4*SNAPSHOT_V3_SIZE + 4*sizeof(f32) + SNAPSHOT_V3_SIZE + sizeof(u32) + 4*sizeof(u32) + 2*SNAPSHOT_V3_SIZE + 8*sizeof(f32) + 4*sizeof(f32) + sizeof(u32))

u64 ds_SolverManifoldSerializeSize(const struct ds_SolverManifold *manifold)
{
	return DS_SOLVER_MANIFOLD_SERIALIZE_SIZE;
}

void ds_SolverManifoldSerialize(struct SS *ss, const struct ds_SolverManifold *manifold)
{
	for (u32 i = 0; i < 4; ++i)
	{
		WriteV3(ss, manifold->anchor0[i]);
	}
	WriteF32N(ss, manifold->depth, 4);
	WriteV3(ss, manifold->normal);
	SSWriteU32Le(ss, manifold->point_count);
	for (u32 i = 0; i < 4; ++i)
	{
		SSWriteU32Le(ss, manifold->id[i].val);
	}
	WriteV3(ss, manifold->tangent[0]);
	WriteV3(ss, manifold->tangent[1]);
	WriteF32N(ss, manifold->tangent_impulse[0], 8);
	WriteF32N(ss, manifold->normal_impulse, 4);
	SSWriteU32Le(ss, manifold->tri);
}

u32 ds_SolverManifoldTryDeserialize(struct SS *ss, struct ds_SolverManifold *manifold)
{
	if (SSBytesLeft(ss) < DS_SOLVER_MANIFOLD_SERIALIZE_SIZE)
	{
		return 0;
	}

	for (u32 i = 0; i < 4; ++i)
	{
		manifold->anchor0[i] = ReadV3(ss);
	}
	ReadF32N(ss, manifold->depth, 4);
	manifold->normal = ReadV3(ss);
	manifold->point_count = SSReadU32Le(ss);
	for (u32 i = 0; i < 4; ++i)
	{
		manifold->id[i].val = SSReadU32Le(ss);
	}
	manifold->tangent[0] = ReadV3(ss);
	manifold->tangent[1] = ReadV3(ss);
	ReadF32N(ss, manifold->tangent_impulse[0], 8);
	ReadF32N(ss, manifold->normal_impulse, 4);
	manifold->tri = SSReadU32Le(ss);
	return manifold->point_count <= 4;
}

/*************************** pool images ***************************/

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
	if (!ds_CPoolTryDeserializeHeaderInternal(ss, &length, &count) || (u64) count*sizeof(u32) > SSBytesLeft(ss))
	{
		ss->bit_index = bit_index;
		return 0;
	}
	if (!ds_CPoolAllocInternal(mem, (void **) &pool->buf, &pool->length, &pool->count, &pool->growable, &pool->heap_allocated, sizeof(u32), length, growable))
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

DEFINE_POOL_SERIALIZE(ds_Body, ds_Body)
DEFINE_POOL_SERIALIZE(ds_Shape, ds_Shape)
DEFINE_POOL_SERIALIZE(ds_Joint, ds_Joint)
DEFINE_POOL_SERIALIZE(ds_PhysicsEvent, ds_PhysicsEvent)
DEFINE_POOL_SERIALIZE(ds_Contact, ds_Contact)
DEFINE_POOL_SERIALIZE(ds_Island, ds_Island)
DEFINE_POOL_SERIALIZE(ds_SolverSet, SolverSetSlot)
DEFINE_CPOOL_SERIALIZE(ds_ProxyQuery, ds_ProxyQuery)
DEFINE_CPOOL_SERIALIZE(ds_BodySim, ds_BodySim)
DEFINE_CPOOL_SERIALIZE(ds_BodyCompute, ds_BodyCompute)
DEFINE_CPOOL_SERIALIZE(ds_ContactCompute, ds_ContactCompute)
DEFINE_CPOOL_SERIALIZE(ds_JointSim, ds_JointSim)

/*************************** contact data ***************************/

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

static u64 ContactManifoldSerializeSize(const struct ds_Dynamics *pipeline, const u32 contact)
{
	const ds_Array(ds_SolverManifold) manifold = ds_SolverManifoldArray(pipeline, contact);
	u64 size = sizeof(u32);
	for (u32 i = 0; i < manifold.length; ++i)
	{
		size += ds_SolverManifoldSerializeSize(manifold.buf + i);
	}
	return size;
}

static void ContactManifoldSerialize(struct SS *ss, const struct ds_Dynamics *pipeline, const u32 contact)
{
	const ds_Array(ds_SolverManifold) manifold = ds_SolverManifoldArray(pipeline, contact);
	SSWriteU32Le(ss, manifold.length);
	for (u32 i = 0; i < manifold.length; ++i)
	{
		ds_SolverManifoldSerialize(ss, manifold.buf + i);
	}
}

/* the buffer must be INTERNAL and empty; more than one manifold go into a slab array (EXTERNAL) */
static u32 ContactManifoldTryDeserialize(struct SS *ss, struct ds_Dynamics *pipeline, struct ds_SolverManifoldBuffer *buffer)
{
	ds_Assert(buffer->type == SOLVER_MANIFOLD_INTERNAL && buffer->count == 0);
	if (SSBytesLeft(ss) < sizeof(u32))
	{
		return 0;
	}

	const u32 count = SSReadU32Le(ss);
	if (count > SSBytesLeft(ss))
	{
		return 0;
	}

	struct ds_SolverManifold *manifold = &buffer->manifold;
	if (count > 1)
	{
		buffer->manifold_array = ds_TSlabAdd(pipeline->manifold_slab, count);
		if (buffer->manifold_array.buf == NULL)
		{
			return 0;
		}
		buffer->type = SOLVER_MANIFOLD_EXTERNAL;
		manifold = buffer->manifold_array.buf;
	}

	for (u32 i = 0; i < count; ++i)
	{
		if (!ds_SolverManifoldTryDeserialize(ss, manifold + i))
		{
			return 0;
		}
	}
	buffer->count = count;
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

	return size;
}

void ds_SolverSetSerialize(struct SS *ss, const struct ds_Dynamics *pipeline, const u32 set_index)
{
	ds_Assert(ds_SolverSetSerializeSize(pipeline, set_index) <= SSBytesLeft(ss));

	const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + set_index;
	ds_Assert((set_index >= SOLVER_SET_SLEEPING_FIRST) == (set->mem.mem_size != 0));
	/* sleeping sets own one heap arena holding their pools */
	SSWriteU64Le(ss, set->mem.mem_size);
	ds_BodySimCPoolSerialize(ss, &set->body_sim_pool);
	ds_BodyComputeCPoolSerialize(ss, &set->body_compute_pool);
	U32CPoolSerialize(ss, &set->contact_pool);
	ds_ContactComputeCPoolSerialize(ss, &set->contact_compute_pool);
	ds_JointSimCPoolSerialize(ss, &set->joint_sim_pool);
	U32CPoolSerialize(ss, &set->island_pool);

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
		if (contact >= pipeline->contact_pool.count_max || !ds_PoolSlotAllocated(pipeline->contact_pool.buf + contact))
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
		if (contact >= pipeline->contact_pool.count_max || !ds_PoolSlotAllocated(pipeline->contact_pool.buf + contact))
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

/* Header: ns_tick, frames_completed, the applied config, island_to_split, the tri cache slots the original carved */
#define DS_SNAPSHOT_HEADER_FIXED_SIZE (3*sizeof(u64) + 2*sizeof(u32))
/* a copy restored onto mem holds its tri cache slab there: twice what the original carved, at least this */
#define DS_SNAPSHOT_TRI_CACHE_LENGTH_MIN 256
#define DS_SNAPSHOT_MANIFOLD_LENGTH_MIN 256

/*
 * Every contact's manifold buffer is serialized after the contact pool; computes by their owners: cgraph
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

	for (u32 i = 0; i < pipeline->contact_pool.count_max; ++i)
	{
		const struct ds_Contact *c = pipeline->contact_pool.buf + i;
		if (ds_PoolSlotAllocated(c))
		{
			size += ContactManifoldSerializeSize(pipeline, i);
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
	/* every contact has an owner: a cgraph color or a solver set */
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
	SSWriteU32Le(ss, pipeline->tri_cache_slab.shared.a_count_max);
	SSWriteU32Le(ss, pipeline->manifold_slab.shared.a_count_max);

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
	for (u32 i = 0; i < pipeline->contact_pool.count_max; ++i)
	{
		const struct ds_Contact *c = pipeline->contact_pool.buf + i;
		if (ds_PoolSlotAllocated(c))
		{
			ContactManifoldSerialize(ss, pipeline, i);
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
	pipeline->frames_completed = SSReadU64Le(ss);
	if (!ds_DynamicsConfigTryDeserializeBitPacked(ss, &pipeline->config))
	{
		goto failure;
	}
	pipeline->config_pending = pipeline->config;
	pipeline->island_to_split = (ds_IslandId) { .val = SSReadU64Le(ss) };
	const u32 tri_cache_carved = SSReadU32Le(ss);
	const u32 manifold_carved = SSReadU32Le(ss);

	u32 tri_cache_length;
	if (mem)
	{
		tri_cache_length = (tri_cache_carved < DS_SNAPSHOT_TRI_CACHE_LENGTH_MIN) ? DS_SNAPSHOT_TRI_CACHE_LENGTH_MIN : 2*tri_cache_carved;
	}
	else
	{
		const u64 tri_cache_slots = g_config->vm_size_tri_cache_allocator / sizeof(struct ds_TriCache);
		tri_cache_length = (tri_cache_slots < U32_MAX) ? (u32) tri_cache_slots : U32_MAX - 1;
	}
	if (!ds_TSlabAlloc(mem, pipeline->tri_cache_slab, tri_cache_length, g_config->thread_count))
	{
		goto failure;
	}

	u32 manifold_length;
	if (mem)
	{
		manifold_length = (manifold_carved < DS_SNAPSHOT_MANIFOLD_LENGTH_MIN) ? DS_SNAPSHOT_MANIFOLD_LENGTH_MIN : 2*manifold_carved;
	}
	else
	{
		const u64 manifold_slots = g_config->vm_size_manifold_allocator / sizeof(struct ds_SolverManifold);
		manifold_length = (manifold_slots < U32_MAX) ? (u32) manifold_slots : U32_MAX - 1;
	}
	if (!ds_TSlabAlloc(mem, pipeline->manifold_slab, manifold_length, g_config->thread_count))
	{
		goto failure;
	}

	/* frame data goes into the buffer the next tick reads (and does not flush) */
	const u64 f = pipeline->frames_completed & 0x1;
	for (u32 i = 0; i < pipeline->runtime.worker_count; ++i)
	{
		pipeline->runtime.worker[i].frame = pipeline->runtime.worker[i].frame_arr + f;
	}

	/* the contact pool precedes its owners, which check their contact indices against it */
	if (!ds_BodyPoolTryDeserialize(NULL, ss, &pipeline->body_pool, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->body_usage_set, GROWABLE)
		|| !ds_JointPoolTryDeserialize(NULL, ss, &pipeline->joint_pool, GROWABLE)
		|| !ds_ShapePoolTryDeserialize(NULL, ss, &pipeline->shape_pool, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->shape_dynamic_usage_set, GROWABLE)
		|| !BVHTryDeserialize(NULL, ss, &pipeline->dynamic_bvh, GROWABLE)
		|| !BVHTryDeserialize(NULL, ss, &pipeline->static_bvh, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->shape_dirty_set, GROWABLE)
		|| !ds_ProxyQueryCPoolTryDeserialize(NULL, ss, &pipeline->dirty_shape_query, GROWABLE)
		|| !ds_PhysicsEventPoolTryDeserialize(NULL, ss, &pipeline->event_pool, GROWABLE)
		|| !DLLTryDeserialize(ss, pipeline->event_list)
		|| !ds_ContactPoolTryDeserialize(NULL, ss, &pipeline->contact_pool, GROWABLE))
	{
		goto failure;
	}

	/* the contacts' data arrays are restored by their owners: empty them first, so ds_DynamicsDealloc is safe after a failure */
	for (u32 i = 0; i < pipeline->contact_pool.count_max; ++i)
	{
		struct ds_Contact *c = pipeline->contact_pool.buf + i;
		if (ds_PoolSlotAllocated(c))
		{
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

	ds_CPoolAlloc(NULL, pipeline->manifold_buffer_pool, pipeline->contact_pool.length, GROWABLE);
	if (pipeline->contact_pool.count_max)
	{
		ds_CPoolPushBatch(pipeline->manifold_buffer_pool, pipeline->contact_pool.count_max);
	}
	for (u32 i = 0; i < pipeline->contact_pool.count_max; ++i)
	{
		pipeline->manifold_buffer_pool.buf[i] = (struct ds_SolverManifoldBuffer) { .type = SOLVER_MANIFOLD_INTERNAL, .count = 0 };
	}

	for (u32 i = 0; i < pipeline->contact_pool.count_max; ++i)
	{
		const struct ds_Contact *c = pipeline->contact_pool.buf + i;
		if (ds_PoolSlotAllocated(c) && !ContactManifoldTryDeserialize(ss, pipeline, pipeline->manifold_buffer_pool.buf + i))
		{
			goto failure;
		}
	}

	if (!ds_HashMapTryDeserialize(NULL, ss, &pipeline->contact_map, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->contact_usage_set, GROWABLE)
		|| !ds_IslandPoolTryDeserialize(NULL, ss, &pipeline->island_pool, GROWABLE)
		|| !ds_BitSetTryDeserialize(NULL, ss, &pipeline->island_high_energy_set, GROWABLE)
		|| !ds_CGraphTryDeserialize(ss, pipeline)
		|| !ds_SolverSetPoolTryDeserialize(NULL, ss, &pipeline->solver_set_pool, GROWABLE))
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
