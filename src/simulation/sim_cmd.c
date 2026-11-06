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
sim_cmd.c
=========

Every external source of input that affects the simulation state is generalized
behind the ds_SimCmd API. By serializing all such commands, in order, to a file
we gain the ability to fully replay any simulation. However, such APIs can quickly
become cumbersome to use, so we discuss the issues and try to land on something
reasonable.

Judging from the words "external source" we can imagine these commands, in 
serialized forms, being sent over networks, read from files, and so on. This seem
like things that should be fully abstracted away and be part of the generalized
command pipeline; every command arrives to the engine/server fully serialized:

          File =>                               serialized(cmd) ---+
                                                                   |
       Network => Payload(serialized(cmd)) ---> serialized(cmd) ---+---> cmd.ValidateAndExecute()
                                                                   |
    LocalInput =>                               serialized(cmd) ---+

The more complex question is how the external sources dispatch commands to the
command queue. As in the previous case, this is preferable fully general, as and
easy-to-use as possible. 

We begin by acknowledging that any external command's destination is a specific
simulation's command queue. This seems to be the correct abstraction level when
specifying a command's destination; Presuming that a server generates a unique
identifier (ds_SimId) per simulation, whenever we want to send a command(s) to a
given simulation, we simply have to send an identifier with our command set:

    Network:       Payload(sim_id, serialized(cmd[])) 
    File:                  sim_id, serialized(cmd[])
    LocalInput:            sim_id, serialized(cmd[])

    struct ds_Core, struct ds_Engin
    {
        ds_Pool simulation_pool;    // One simulation = one game world...
    
        (...)
    };


    ServerHandleFramePayloads(...)
    { 
        server.payload = PayloadEmpty()
        server.cmd_pool = CPoolEmpty();

        // POSSIBLE FUNCTION 1 ABSTRACTION: payload = PayloadTryDeserializeAndVerify(&core, mem, ss)

        ext_payload[count]);
        for (i in stream_count)
        {
            ext_payload[i] = PayloadEmpty()

            if (!PayloadTryDeserialize(ext_payload[i], ext_stream[i]) || PayloadHeaderVerify(&ext_payload))
            {
                continue;
            }

            success = 1; 
            while (s.NotEmpty())
            {
                if (!CmdTryDeserialize(&cmd, ext_payload) || !CmdFormVerify(&cmd))
                {
                    server.cmd_pool.pop(push_count);
                    ext_payload[count] = PayloadEmpty();
                    break;
                }

                server.cmd_pool.push(cmd);
                push_count += 1;
            }
        }

        // POSSIBLE FUNCTION 2 ABSTRACTION: Append, we just append one payload onto another
        
        for (external in payload-streams[count])
        {
            payload.Append(external_payload);
        }

        // we now have commands in order to be executed, and a fused payload ready to be timestamped and recorded.
    }
 */

/*
TODO:

Payload Verification
--------------------
Is the expected amount of data there and does it make sense?

Can this be hacked/broken/corrupted?
Can this be used for Denial-of-Service (malicious payloads degrading performance)
...

*/


#include "ds_base.h"
#include "ds_simulation.h"


/**************** ds_SimCmd ****************/

/* a prefab id: non-empty and within the database's buffer */
static u32 ds_SimCmdPrefabIdCheck(const utf8 id)
{
    return id.size >= 1 && id.size < PREFAB_BUFSIZE;
}

/*
 * Serialized command: the type in DS_SIM_CMD_TYPE_BITS partial bits, then the type's body, bit-packed so that a
 * command starts at any bit. A body serialized by a byte serializer (ConfigLoad: ds_SimConfigSerialize) is carried
 * as its byte count in DS_SIM_CMD_SIZE_BITS and the bytes copied bit-exactly: the size is independent of the bit
 * offset and the byte serializers stay aligned, on their own buffer.
 */
#define DS_SIM_CMD_TYPE_BITS        8
#define DS_SIM_CMD_SIZE_BITS        32
#define DS_SIM_CMD_BODY_SIZE_MAX    (1 << 16)   /* a byte body larger than this is rejected before any allocation */

/* Copy size bytes of buf into ss at its bit offset. */
static void ds_SimCmdWriteBytes(struct SS *ss, const u8 *buf, const u64 size)
{
    struct SS src = SSBuffered((void *) buf, size);
    while (SSBitsLeft(&src))
    {
        const u64 bit_count = U64Min(SSBitsLeft(&src), 64);
        SSWriteU64Partial(ss, SSReadU64Partial(&src, bit_count), bit_count);
    }
}

/* Copy size bytes from ss's bit offset into buf. */
static void ds_SimCmdReadBytes(struct SS *ss, u8 *buf, const u64 size)
{
    struct SS dst = SSBuffered(buf, size);
    while (SSBitsLeft(&dst))
    {
        const u64 bit_count = U64Min(SSBitsLeft(&dst), 64);
        SSWriteU64Partial(&dst, SSReadU64Partial(ss, bit_count), bit_count);
    }
}

/* a prefab id: 8 bits of size, then its bytes */
#define DS_SIM_CMD_ID_BITS(id)      (8 + 8*(id).size)

/* the id's bytes go on mem; 0 if the stream is short or mem is full */
static u32 ds_SimCmdTryReadId(struct arena *mem, struct SS *ss, utf8 *id)
{
    if (SSBitsLeft(ss) < 8)
    {
        return 0;
    }

    const u32 size = (u32) SSReadU64Partial(ss, 8);
    if (SSBitsLeft(ss) < 8*size)
    {
        return 0;
    }

    u8 *buf = ArenaPush(mem, (size) ? size : 1);
    if (buf == NULL)
    {
        return 0;
    }

    for (u32 i = 0; i < size; ++i)
    {
        buf[i] = (u8) SSReadU64Partial(ss, 8);
    }
    *id = (utf8) { .buf = buf, .size = size, .len = size };
    return 1;
}
#define DS_SIM_CMD_JOINT_ADD_BITS   (32 + 32 + 2*32 + 32 + 2*32 + 2*32 + 2*32 + 2*8*28)


/******** ConfigLoad ********/

static u32 ds_LoadConfigCmdFormVerify(const struct ds_LoadConfigCmd *cmd)
{
    return ds_SimConfigCheck(cmd->config);
}

/* the seed is accepted as is; afterwards the alloc-time values are fixed */
static u32 ds_LoadConfigCmdDataVerify(const struct ds_Sim *sim, const struct ds_LoadConfigCmd *cmd)
{
    if (!sim->initialized)
    {
        return 1;
    }

    const struct ds_SimConfig *config = cmd->config;
    const struct ds_SimConfig *seed = &sim->config_seed;
    u32 valid = config->dynamics.initial_size == seed->dynamics.initial_size
             && config->dynamics.frame_memory == seed->dynamics.frame_memory;
    for (u32 i = 0; i < 4; ++i)
    {
        valid = valid && config->seed[i] == seed->seed[i];
    }

    for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
    {
        valid = valid && config->us_system_tick[i] == seed->us_system_tick[i];
    }

    return valid;
}

static u32 ds_LoadConfigCmdTryExecute(struct ds_Sim *sim, const struct ds_LoadConfigCmd *cmd)
{
    /* the dynamics (the game later) */
    sim->dynamics.config_pending = cmd->config->dynamics;
    ds_DynamicsConfigUpdatePending(&sim->dynamics);
    sim->initialized = 1;
    return 1;
}

static u64 ds_LoadConfigCmdSerializeBitsize(const struct ds_LoadConfigCmd *cmd)
{
    return DS_SIM_CMD_SIZE_BITS + 8*ds_SimConfigSerializeSize(cmd->config);
}

static void ds_LoadConfigCmdSerialize(struct SS *ss, const struct ds_LoadConfigCmd *cmd)
{
    struct arena *tmp = ArenaPushScratch();
    const u64 size = ds_SimConfigSerializeSize(cmd->config);
    struct SS body = SSAlloc(tmp, size);
    ds_SimConfigSerialize(&body, cmd->config);

    SSWriteU64Partial(ss, size, DS_SIM_CMD_SIZE_BITS);
    ds_SimCmdWriteBytes(ss, body.buf, size);
    ArenaPopScratch();
}

static u32 ds_LoadConfigCmdTryDeserialize(struct arena *mem, struct SS *ss, struct ds_LoadConfigCmd *cmd)
{
    if (SSBitsLeft(ss) < DS_SIM_CMD_SIZE_BITS)
    {
        return 0;
    }

    const u64 size = SSReadU64Partial(ss, DS_SIM_CMD_SIZE_BITS);
    if (size > DS_SIM_CMD_BODY_SIZE_MAX || 8*size > SSBitsLeft(ss))
    {
        return 0;
    }

    struct ds_SimConfig *config = ArenaPush(mem, sizeof(struct ds_SimConfig));
    if (config == NULL)
    {
        Log(T_SYSTEM, S_ERROR, "ds_LoadConfigCmdTryDeserialize: no room for a config (%lu bytes)", (u64) sizeof(struct ds_SimConfig));
        return 0;
    }

    struct arena *tmp = ArenaPushScratch();
    struct SS body = SSAlloc(tmp, size);
    ds_SimCmdReadBytes(ss, body.buf, size);
    /* the whole body must be the config: a short or a long body is malformed */
    const u32 success = ds_SimConfigTryDeserialize(&body, config) && body.bit_index == 8*size;
    ArenaPopScratch();

    cmd->config = config;
    return success;
}

/******** BodyAdd ********/

static u32 ds_BodyAddCmdFormVerify(const struct ds_BodyAddCmd *cmd)
{
    return ds_SimCmdPrefabIdCheck(cmd->prefab) && ds_TransformWorldFiniteCheck(&cmd->t_world);
}

static u32 ds_BodyAddCmdDataVerify(const struct ds_Sim *sim, const struct ds_BodyAddCmd *cmd)
{
    return ds_SDBLookup(*sim->body_prefab_db, cmd->prefab).index != SDB_STUB
        && ds_BodyIdNullCheck(ds_SimBodyByEntity(sim, cmd->entity));
}

static u32 ds_BodyAddCmdTryExecute(struct ds_Sim *sim, const struct ds_BodyAddCmd *cmd)
{
    const struct ds_BodyPrefab *prefab = ds_SDBLookup(*sim->body_prefab_db, cmd->prefab).address;
    const ds_BodyId body = ds_BodyAdd(&sim->dynamics, prefab, &cmd->t_world, cmd->entity);
    if (ds_BodyIdNullCheck(body))
    {
        Log(T_SYSTEM, S_ERROR, "ds_BodyAddCmdTryExecute: no body for entity %u", cmd->entity);
        return 0;
    }
    ds_HashMapAdd(&sim->entity_map, cmd->entity, ds_BodyIdIndex(body));
    return 1;
}

static u64 ds_BodyAddCmdSerializeBitsize(const struct ds_BodyAddCmd *cmd)
{
    return DS_SIM_CMD_ID_BITS(cmd->prefab) + 8*ds_TransformWorldSerializeSize(&cmd->t_world) + 32;
}

static void ds_BodyAddCmdSerialize(struct SS *ss, const struct ds_BodyAddCmd *cmd)
{
    SSWriteU64Partial(ss, cmd->prefab.size, 8);
    for (u32 i = 0; i < cmd->prefab.size; ++i)
    {
        SSWriteU64Partial(ss, cmd->prefab.buf[i], 8);
    }
    ds_TransformWorldSerialize(ss, &cmd->t_world);
    SSWriteU64Partial(ss, cmd->entity, 32);
}

static u32 ds_BodyAddCmdTryDeserialize(struct arena *mem, struct SS *ss, struct ds_BodyAddCmd *cmd)
{
    if (!ds_SimCmdTryReadId(mem, ss, &cmd->prefab) || SSBitsLeft(ss) < 8*ds_TransformWorldSerializeSize(&cmd->t_world) + 32)
    {
        return 0;
    }

    ds_TransformWorldDeserialize(ss, &cmd->t_world);
    cmd->entity = (u32) SSReadU64Partial(ss, 32);
    return 1;
}

/******** ShapeAdd ********/

static u32 ds_ShapeAddCmdFormVerify(const struct ds_ShapeAddCmd *cmd)
{
    return ds_SimCmdPrefabIdCheck(cmd->prefab) && ds_TransformFiniteCheck(&cmd->t_local);
}

static u32 ds_ShapeAddCmdDataVerify(const struct ds_Sim *sim, const struct ds_ShapeAddCmd *cmd)
{
    return ds_SDBLookup(*sim->shape_prefab_db, cmd->prefab).index != SDB_STUB
        && !ds_BodyIdNullCheck(ds_SimBodyByEntity(sim, cmd->entity));
}

static u32 ds_ShapeAddCmdTryExecute(struct ds_Sim *sim, const struct ds_ShapeAddCmd *cmd)
{
    const struct ds_ShapePrefab *prefab = ds_SDBLookup(*sim->shape_prefab_db, cmd->prefab).address;
    ds_ShapeAdd(&sim->dynamics, prefab, &cmd->t_local, ds_SimBodyByEntity(sim, cmd->entity));
    return 1;
}

static u64 ds_ShapeAddCmdSerializeBitsize(const struct ds_ShapeAddCmd *cmd)
{
    return DS_SIM_CMD_ID_BITS(cmd->prefab) + 8*ds_TransformSerializeSize(&cmd->t_local) + 32;
}

static void ds_ShapeAddCmdSerialize(struct SS *ss, const struct ds_ShapeAddCmd *cmd)
{
    SSWriteU64Partial(ss, cmd->prefab.size, 8);
    for (u32 i = 0; i < cmd->prefab.size; ++i)
    {
        SSWriteU64Partial(ss, cmd->prefab.buf[i], 8);
    }
    ds_TransformSerialize(ss, &cmd->t_local);
    SSWriteU64Partial(ss, cmd->entity, 32);
}

static u32 ds_ShapeAddCmdTryDeserialize(struct arena *mem, struct SS *ss, struct ds_ShapeAddCmd *cmd)
{
    if (!ds_SimCmdTryReadId(mem, ss, &cmd->prefab) || SSBitsLeft(ss) < 8*ds_TransformSerializeSize(&cmd->t_local) + 32)
    {
        return 0;
    }

    ds_TransformDeserialize(ss, &cmd->t_local);
    cmd->entity = (u32) SSReadU64Partial(ss, 32);
    return 1;
}

/******** JointAdd ********/

static u32 ds_JointAddCmdFormVerify(const struct ds_JointAddCmd *cmd)
{
    return F32FiniteCheck(cmd->target) && F32FiniteCheck(cmd->range.low) && F32FiniteCheck(cmd->range.high)
        && F32FiniteCheck(cmd->hertz) && F32FiniteCheck(cmd->damping_ratio)
        && !F32TestNan(cmd->breakage.force_threshold) && !F32TestNan(cmd->breakage.torque_threshold)   /* infinity: unbreakable */
        && cmd->range.low <= cmd->target && cmd->target <= cmd->range.high && cmd->hertz >= 0.0f && cmd->damping_ratio >= 0.0f
        && cmd->entity[0] != cmd->entity[1]
        && ds_TransformFiniteCheck(cmd->local_frame + 0) && ds_TransformFiniteCheck(cmd->local_frame + 1);
}

static u32 ds_JointAddCmdDataVerify(const struct ds_Sim *sim, const struct ds_JointAddCmd *cmd)
{
    return !ds_BodyIdNullCheck(ds_SimBodyByEntity(sim, cmd->entity[0]))
        && !ds_BodyIdNullCheck(ds_SimBodyByEntity(sim, cmd->entity[1]));
}

static u32 ds_JointAddCmdTryExecute(struct ds_Sim *sim, const struct ds_JointAddCmd *cmd)
{
    struct ds_DistanceJointPrefab prefab;
    if (cmd->enable_spring)
    {
        ds_DistanceJointPrefabRangeSpring(&prefab, cmd->range, cmd->hertz, cmd->damping_ratio, cmd->breakage);
    }
    else
    {
        ds_DistanceJointPrefabRange(&prefab, cmd->range, cmd->breakage);
    }
    prefab.target = F32Clamp(cmd->target, prefab.range.low, prefab.range.high);
    prefab.collide_connected = cmd->collide_connected;
    ds_DistanceJointAdd(&sim->dynamics, &prefab, ds_SimBodyByEntity(sim, cmd->entity[0]), cmd->local_frame + 0, ds_SimBodyByEntity(sim, cmd->entity[1]), cmd->local_frame + 1);
    return 1;
}

static u64 ds_JointAddCmdSerializeBitsize(const struct ds_JointAddCmd *cmd)
{
    return DS_SIM_CMD_JOINT_ADD_BITS;
}

static void ds_JointAddCmdSerialize(struct SS *ss, const struct ds_JointAddCmd *cmd)
{
    SSWriteU64Partial(ss, cmd->collide_connected, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->target }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->range.low }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->range.high }.u, 32);
    SSWriteU64Partial(ss, cmd->enable_spring, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->hertz }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->damping_ratio }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->breakage.force_threshold }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = cmd->breakage.torque_threshold }.u, 32);
    SSWriteU64Partial(ss, cmd->entity[0], 32);
    SSWriteU64Partial(ss, cmd->entity[1], 32);
    ds_TransformSerialize(ss, cmd->local_frame + 0);
    ds_TransformSerialize(ss, cmd->local_frame + 1);
}

static u32 ds_JointAddCmdTryDeserialize(struct SS *ss, struct ds_JointAddCmd *cmd)
{
    if (SSBitsLeft(ss) < DS_SIM_CMD_JOINT_ADD_BITS)
    {
        return 0;
    }

    cmd->collide_connected = (u32) SSReadU64Partial(ss, 32);
    cmd->target = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->range.low = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->range.high = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->enable_spring = (u32) SSReadU64Partial(ss, 32);
    cmd->hertz = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->damping_ratio = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->breakage.force_threshold = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->breakage.torque_threshold = (b32) { .u = (u32) SSReadU64Partial(ss, 32) }.f;
    cmd->entity[0] = (u32) SSReadU64Partial(ss, 32);
    cmd->entity[1] = (u32) SSReadU64Partial(ss, 32);
    ds_TransformDeserialize(ss, cmd->local_frame + 0);
    ds_TransformDeserialize(ss, cmd->local_frame + 1);
    return 1;
}

/******** ds_SimCmd: the switches over the commands ********/

u32 ds_SimCmdFormVerify(const struct ds_SimCmd *cmd)
{
    switch (cmd->type)
    {
        case SIM_CMD_CONFIG_LOAD:   { return ds_LoadConfigCmdFormVerify(&cmd->load_config); }
        case SIM_CMD_BODY_ADD:      { return ds_BodyAddCmdFormVerify(&cmd->body_add); }
        case SIM_CMD_SHAPE_ADD:     { return ds_ShapeAddCmdFormVerify(&cmd->shape_add); }
        case SIM_CMD_JOINT_ADD:     { return ds_JointAddCmdFormVerify(&cmd->joint_add); }
        default:                    { return 0; }
    }
}

/* Return 1 if the well-formed cmd is a valid command within sim's context, 0 otherwise.*/
static u32 ds_SimCmdDataVerify(const struct ds_Sim *sim, const struct ds_SimCmd *cmd)
{
    ds_Assert(ds_SimCmdFormVerify(cmd));

    switch (cmd->type)
    {
        case SIM_CMD_CONFIG_LOAD:   { return ds_LoadConfigCmdDataVerify(sim, &cmd->load_config); }
        case SIM_CMD_BODY_ADD:      { return ds_BodyAddCmdDataVerify(sim, &cmd->body_add); }
        case SIM_CMD_SHAPE_ADD:     { return ds_ShapeAddCmdDataVerify(sim, &cmd->shape_add); }
        case SIM_CMD_JOINT_ADD:     { return ds_JointAddCmdDataVerify(sim, &cmd->joint_add); }
        default:                    { return 0; }
    }
}

u32 ds_SimCmdTryExecute(struct ds_Sim *sim, const struct ds_SimCmd *cmd)
{
    if (!ds_SimCmdDataVerify(sim, cmd))
    {
        Log(T_SYSTEM, S_WARNING, "ds_SimCmdTryExecute: command of type %u rejected", (u32) cmd->type);
        return 0;
    }

    switch (cmd->type)
    {
        case SIM_CMD_CONFIG_LOAD:   { return ds_LoadConfigCmdTryExecute(sim, &cmd->load_config); }
        case SIM_CMD_BODY_ADD:      { return ds_BodyAddCmdTryExecute(sim, &cmd->body_add); }
        case SIM_CMD_SHAPE_ADD:     { return ds_ShapeAddCmdTryExecute(sim, &cmd->shape_add); }
        case SIM_CMD_JOINT_ADD:     { return ds_JointAddCmdTryExecute(sim, &cmd->joint_add); }
        default:                    { ds_AssertString(0, "ds_SimCmdTryExecute: unknown command type"); return 0; }
    }
}

static u64 ds_SimCmdSerializeBitsize(const struct ds_SimCmd *cmd)
{
    switch (cmd->type)
    {
        case SIM_CMD_CONFIG_LOAD:   { return DS_SIM_CMD_TYPE_BITS + ds_LoadConfigCmdSerializeBitsize(&cmd->load_config); }
        case SIM_CMD_BODY_ADD:      { return DS_SIM_CMD_TYPE_BITS + ds_BodyAddCmdSerializeBitsize(&cmd->body_add); }
        case SIM_CMD_SHAPE_ADD:     { return DS_SIM_CMD_TYPE_BITS + ds_ShapeAddCmdSerializeBitsize(&cmd->shape_add); }
        case SIM_CMD_JOINT_ADD:     { return DS_SIM_CMD_TYPE_BITS + ds_JointAddCmdSerializeBitsize(&cmd->joint_add); }
        default:                    { ds_AssertString(0, "ds_SimCmdSerializeBitsize: unknown command type"); return 0; }
    }
}

static void ds_SimCmdSerialize(struct SS *ss, const struct ds_SimCmd *cmd)
{
    ds_Assert(ds_SimCmdSerializeBitsize(cmd) <= SSBitsLeft(ss));

    SSWriteU64Partial(ss, (u64) cmd->type, DS_SIM_CMD_TYPE_BITS);
    switch (cmd->type)
    {
        case SIM_CMD_CONFIG_LOAD:   { ds_LoadConfigCmdSerialize(ss, &cmd->load_config); } break;
        case SIM_CMD_BODY_ADD:      { ds_BodyAddCmdSerialize(ss, &cmd->body_add); } break;
        case SIM_CMD_SHAPE_ADD:     { ds_ShapeAddCmdSerialize(ss, &cmd->shape_add); } break;
        case SIM_CMD_JOINT_ADD:     { ds_JointAddCmdSerialize(ss, &cmd->joint_add); } break;
        default:                    { ds_AssertString(0, "ds_SimCmdSerialize: unknown command type"); } break;
    }
}

u32 ds_SimCmdTryDeserialize(struct arena *mem, struct SS *ss, struct ds_SimCmd *cmd)
{
    struct SS ss_mut = *ss;
    if (SSBitsLeft(&ss_mut) < DS_SIM_CMD_TYPE_BITS)
    {
        return 0;
    }

    const u64 type = SSReadU64Partial(&ss_mut, DS_SIM_CMD_TYPE_BITS);
    u32 success;
    switch (type)
    {
        case SIM_CMD_CONFIG_LOAD:   { success = ds_LoadConfigCmdTryDeserialize(mem, &ss_mut, &cmd->load_config); } break;
        case SIM_CMD_BODY_ADD:      { success = ds_BodyAddCmdTryDeserialize(mem, &ss_mut, &cmd->body_add); } break;
        case SIM_CMD_SHAPE_ADD:     { success = ds_ShapeAddCmdTryDeserialize(mem, &ss_mut, &cmd->shape_add); } break;
        case SIM_CMD_JOINT_ADD:     { success = ds_JointAddCmdTryDeserialize(&ss_mut, &cmd->joint_add); } break;
        default:                    { success = 0; } break;
    }

    if (!success)
    {
        return 0;
    }

    cmd->type = (enum ds_SimCmdType) type;
    *ss = ss_mut;
    return 1;
}

/**************** ds_SimCmdPayload ****************/

struct ds_SimCmdPayload ds_SimCmdPayloadEmpty(const ds_SimId dst)
{
    return (struct ds_SimCmdPayload) { .dst = dst };
}

struct ds_SimCmdPayload ds_SimCmdPayloadAlloc(struct arena *mem, const ds_SimId dst, const u64 size)
{
    return (struct ds_SimCmdPayload)
    {
        .dst = dst,
        .cmd_count = 0,
        .stream = SSAlloc(mem, size),
    };
}

struct ds_SimCmdPayload ds_SimCmdPayloadPushAll(struct arena *mem, const ds_SimId dst, const ds_CPool(ds_SimCmd) *pool)
{
    u64 bits = 0;
    for (u32 i = 0; i < pool->count; ++i)
    {
        bits += ds_SimCmdSerializeBitsize(pool->buf + i);
    }

    struct ds_SimCmdPayload payload = ds_SimCmdPayloadAlloc(mem, dst, (bits + 7) / 8);
    if (bits && payload.stream.buf == NULL)
    {
        Log(T_SYSTEM, S_ERROR, "ds_SimCmdPayloadPushAll: no room for %lu bytes, %u commands dropped", (bits + 7) / 8, pool->count);
        return payload;
    }

    for (u32 i = 0; i < pool->count; ++i)
    {
        ds_SimCmdSerialize(&payload.stream, pool->buf + i);
    }
    payload.cmd_count = pool->count;
    return payload;
}

void ds_SimCmdPayloadDealloc(struct ds_SimCmdPayload *payload)
{
    SSDealloc(&payload->stream);
    *payload = ds_SimCmdPayloadEmpty(ds_SimIdNull());
}

#define DS_SIM_CMD_PAYLOAD_FIXED_SIZE   (sizeof(u64) + 2*sizeof(u32))  /* dst, cmd_count, command bits */

u64 ds_SimCmdPayloadSerializeSize(const struct ds_SimCmdPayload *payload)
{
    return DS_SIM_CMD_PAYLOAD_FIXED_SIZE + (payload->stream.bit_index + 7) / 8;
}

void ds_SimCmdPayloadSerialize(struct SS *ss, const struct ds_SimCmdPayload *payload)
{
    ds_Assert(ss->bit_index % 8 == 0);
    ds_Assert(payload->stream.bit_index <= U32_MAX);
    ds_Assert(ds_SimCmdPayloadSerializeSize(payload) <= SSBytesLeft(ss));

    SSWriteU64Le(ss, payload->dst.val);
    SSWriteU32Le(ss, payload->cmd_count);
    SSWriteU32Le(ss, (u32) payload->stream.bit_index);
    SSWriteU8N(ss, payload->stream.buf, (payload->stream.bit_index + 7) / 8);   /* the last byte's unused bits are zero */
}

u32 ds_SimCmdPayloadTryDeserializeAndAlias(struct SS *ss, struct ds_SimCmdPayload *payload)
{
    struct SS ss_mut = *ss;
    *payload = ds_SimCmdPayloadEmpty(ds_SimIdNull());
    struct ds_SimCmdPayload payload_mut = *payload;

    if ((ss_mut.bit_index % 8) || SSBytesLeft(&ss_mut) < DS_SIM_CMD_PAYLOAD_FIXED_SIZE)
    {
        return 0;
    }

    payload_mut.dst.val = SSReadU64Le(&ss_mut);
    payload_mut.cmd_count = SSReadU32Le(&ss_mut);
    const u64 bit_count = SSReadU32Le(&ss_mut);
    struct SSView view;
    if (!SSTryView(&view, &ss_mut, ss_mut.bit_index, bit_count))
    {
        return 0;
    }

    /* the payload's own stream: its buf at the commands (byte aligned by the format), all bit_count bits used */
    payload_mut.stream = SSBuffered(view.buf + view.bit_offset / 8, (bit_count + 7) / 8);
    payload_mut.stream.bit_index = bit_count;
    payload_mut.stream.bit_count = bit_count;

    const u32 pad = (bit_count % 8) 
        ? payload_mut.stream.buf[bit_count / 8] >> (bit_count % 8) 
        : 0;
    if (pad != 0)
    {
        return 0;
    }

    ss_mut.bit_index += 8*((bit_count + 7) / 8);

    *ss = ss_mut;
    *payload = payload_mut;
    return 1;
}

u32 ds_SimCmdPayloadAppend(struct ds_SimCmdPayload *payload, const struct ds_SimCmdPayload *src)
{
    struct SSView view;     /* src's used bits */
    if ((!ds_SimIdNullCheck(payload->dst) && payload->dst.val != src->dst.val)
        || !SSTryView(&view, &src->stream, 0, src->stream.bit_index)
        || SSBitsLeft(&payload->stream) < view.bit_count)
    {
        return 0;
    }

    struct SS s = SSSubstreamFromView(&view);
    while (SSBitsLeft(&s))
    {
        const u64 bit_count = U64Min(SSBitsLeft(&s), 64);
        SSWriteU64Partial(&payload->stream, SSReadU64Partial(&s, bit_count), bit_count);
    }

    payload->dst = src->dst;
    payload->cmd_count += src->cmd_count;
    return 1;
}
