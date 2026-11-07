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

#include "ds_base.h"
#include "ds_simulation.h"

/**************** ds_SimCmd ****************/

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
    u32 valid = config->dynamics.initial_size == seed->dynamics.initial_size;
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

#include "sim_cmd_tmp.c"

/******** ds_SimCmd: the switches over the commands ********/

u32 ds_SimCmdFormVerify(const struct ds_SimCmd *cmd)
{
    switch (cmd->type)
    {
        case SIM_CMD_CONFIG_LOAD:   { return ds_LoadConfigCmdFormVerify(&cmd->load_config); }
        case SIM_CMD_BODY_ADD:      { return ds_BodyAddCmdFormVerify(&cmd->body_add); }
        case SIM_CMD_SHAPE_ADD:     { return ds_ShapeAddCmdFormVerify(&cmd->shape_add); }
        case SIM_CMD_JOINT_ADD:     { return ds_JointAddCmdFormVerify(&cmd->joint_add); }
        case SIM_CMD_MOUSE_JOINT_ADD: { return ds_MouseJointAddCmdFormVerify(&cmd->mouse_joint_add); }
        case SIM_CMD_MOUSE_JOINT_TARGET_SET: { return ds_MouseJointTargetSetCmdFormVerify(&cmd->mouse_joint_target_set); }
        case SIM_CMD_JOINT_REMOVE: { return ds_JointRemoveCmdFormVerify(&cmd->joint_remove); }
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
        case SIM_CMD_MOUSE_JOINT_ADD: { return ds_MouseJointAddCmdDataVerify(sim, &cmd->mouse_joint_add); }
        case SIM_CMD_MOUSE_JOINT_TARGET_SET: { return ds_MouseJointTargetSetCmdDataVerify(sim, &cmd->mouse_joint_target_set); }
        case SIM_CMD_JOINT_REMOVE: { return ds_JointRemoveCmdDataVerify(sim, &cmd->joint_remove); }
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
        case SIM_CMD_MOUSE_JOINT_ADD: { return ds_MouseJointAddCmdTryExecute(sim, &cmd->mouse_joint_add); }
        case SIM_CMD_MOUSE_JOINT_TARGET_SET: { return ds_MouseJointTargetSetCmdTryExecute(sim, &cmd->mouse_joint_target_set); }
        case SIM_CMD_JOINT_REMOVE: { return ds_JointRemoveCmdTryExecute(sim, &cmd->joint_remove); }
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
        case SIM_CMD_MOUSE_JOINT_ADD: { return DS_SIM_CMD_TYPE_BITS + ds_MouseJointAddCmdSerializeBitsize(&cmd->mouse_joint_add); }
        case SIM_CMD_MOUSE_JOINT_TARGET_SET: { return DS_SIM_CMD_TYPE_BITS + ds_MouseJointTargetSetCmdSerializeBitsize(&cmd->mouse_joint_target_set); }
        case SIM_CMD_JOINT_REMOVE: { return DS_SIM_CMD_TYPE_BITS + ds_JointRemoveCmdSerializeBitsize(&cmd->joint_remove); }
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
        case SIM_CMD_MOUSE_JOINT_ADD: { ds_MouseJointAddCmdSerialize(ss, &cmd->mouse_joint_add); } break;
        case SIM_CMD_MOUSE_JOINT_TARGET_SET: { ds_MouseJointTargetSetCmdSerialize(ss, &cmd->mouse_joint_target_set); } break;
        case SIM_CMD_JOINT_REMOVE: { ds_JointRemoveCmdSerialize(ss, &cmd->joint_remove); } break;
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
        case SIM_CMD_MOUSE_JOINT_ADD: { success = ds_MouseJointAddCmdTryDeserialize(&ss_mut, &cmd->mouse_joint_add); } break;
        case SIM_CMD_MOUSE_JOINT_TARGET_SET: { success = ds_MouseJointTargetSetCmdTryDeserialize(&ss_mut, &cmd->mouse_joint_target_set); } break;
        case SIM_CMD_JOINT_REMOVE: { success = ds_JointRemoveCmdTryDeserialize(&ss_mut, &cmd->joint_remove); } break;
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

    if (SSBytesLeft(&ss_mut) < DS_SIM_CMD_PAYLOAD_FIXED_SIZE)
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
