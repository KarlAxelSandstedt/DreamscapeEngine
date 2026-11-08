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

/* Serialized command: the type in DS_SIM_CMD_TYPE_BITS, then the type's body, bit-packed: a command starts at any bit. */
#define DS_SIM_CMD_TYPE_BITS        8

static u32 ds_LoadConfigCmdFormVerify(const struct ds_LoadConfigCmd *cmd)
{
    ds_Assert(cmd->config);
    return ds_SimConfigCheck(cmd->config);
}

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
    if (!ds_LoadConfigCmdDataVerify(sim, cmd))
    {
        LogString(T_SYSTEM, S_WARNING, "ds_LoadConfigCmdTryExecute: rejected, the seed, tick periods or initial size differ from the simulation's");
        return 0;
    }

    /* the dynamics (the game later) */
    sim->dynamics.config_pending = cmd->config->dynamics;
    ds_DynamicsConfigUpdatePending(&sim->dynamics);
    sim->initialized = 1;
    return 1;
}

static u64 ds_LoadConfigCmdSerializeBitPackedSize(const struct ds_LoadConfigCmd *cmd)
{
    return ds_SimConfigSerializeBitPackedSize(cmd->config);
}

static void ds_LoadConfigCmdSerializeBitPacked(struct SS *ss, const struct ds_LoadConfigCmd *cmd)
{
    ds_SimConfigSerializeBitPacked(ss, cmd->config);
}

static u32 ds_LoadConfigCmdTryDeserializeBitPacked(struct arena *mem, struct SS *ss, struct ds_LoadConfigCmd *cmd)
{
    struct ds_SimConfig *config = ArenaPush(mem, sizeof(struct ds_SimConfig));
    if (config == NULL)
    {
        Log(T_SYSTEM, S_ERROR, "ds_LoadConfigCmdTryDeserializeBitPacked: no room for a config (%lu bytes)", (u64) sizeof(struct ds_SimConfig));
        return 0;
    }

    cmd->config = config;
    return ds_SimConfigTryDeserializeBitPacked(ss, config);
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

u32 ds_SimCmdTryExecute(struct ds_Sim *sim, const struct ds_SimCmd *cmd)
{
    ds_Assert(ds_SimCmdFormVerify(cmd));

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

static u64 ds_SimCmdSerializeBitPackedSize(const struct ds_SimCmd *cmd)
{
    switch (cmd->type)
    {
        case SIM_CMD_CONFIG_LOAD:   { return DS_SIM_CMD_TYPE_BITS + ds_LoadConfigCmdSerializeBitPackedSize(&cmd->load_config); }
        case SIM_CMD_BODY_ADD:      { return DS_SIM_CMD_TYPE_BITS + ds_BodyAddCmdSerializeBitPackedSize(&cmd->body_add); }
        case SIM_CMD_SHAPE_ADD:     { return DS_SIM_CMD_TYPE_BITS + ds_ShapeAddCmdSerializeBitPackedSize(&cmd->shape_add); }
        case SIM_CMD_JOINT_ADD:     { return DS_SIM_CMD_TYPE_BITS + ds_JointAddCmdSerializeBitPackedSize(&cmd->joint_add); }
        case SIM_CMD_MOUSE_JOINT_ADD: { return DS_SIM_CMD_TYPE_BITS + ds_MouseJointAddCmdSerializeBitPackedSize(&cmd->mouse_joint_add); }
        case SIM_CMD_MOUSE_JOINT_TARGET_SET: { return DS_SIM_CMD_TYPE_BITS + ds_MouseJointTargetSetCmdSerializeBitPackedSize(&cmd->mouse_joint_target_set); }
        case SIM_CMD_JOINT_REMOVE: { return DS_SIM_CMD_TYPE_BITS + ds_JointRemoveCmdSerializeBitPackedSize(&cmd->joint_remove); }
        default:                    { ds_AssertString(0, "ds_SimCmdSerializeBitPackedSize: unknown command type"); return 0; }
    }
}

static void ds_SimCmdSerializeBitPacked(struct SS *ss, const struct ds_SimCmd *cmd)
{
    ds_Assert(ds_SimCmdSerializeBitPackedSize(cmd) <= SSBitsLeft(ss));

    SSWriteU64Partial(ss, (u64) cmd->type, DS_SIM_CMD_TYPE_BITS);
    switch (cmd->type)
    {
        case SIM_CMD_CONFIG_LOAD:   { ds_LoadConfigCmdSerializeBitPacked(ss, &cmd->load_config); } break;
        case SIM_CMD_BODY_ADD:      { ds_BodyAddCmdSerializeBitPacked(ss, &cmd->body_add); } break;
        case SIM_CMD_SHAPE_ADD:     { ds_ShapeAddCmdSerializeBitPacked(ss, &cmd->shape_add); } break;
        case SIM_CMD_JOINT_ADD:     { ds_JointAddCmdSerializeBitPacked(ss, &cmd->joint_add); } break;
        case SIM_CMD_MOUSE_JOINT_ADD: { ds_MouseJointAddCmdSerializeBitPacked(ss, &cmd->mouse_joint_add); } break;
        case SIM_CMD_MOUSE_JOINT_TARGET_SET: { ds_MouseJointTargetSetCmdSerializeBitPacked(ss, &cmd->mouse_joint_target_set); } break;
        case SIM_CMD_JOINT_REMOVE: { ds_JointRemoveCmdSerializeBitPacked(ss, &cmd->joint_remove); } break;
        default:                    { ds_AssertString(0, "ds_SimCmdSerializeBitPacked: unknown command type"); } break;
    }
}

u32 ds_SimCmdTryDeserializeBitPacked(struct arena *mem, struct SS *ss, struct ds_SimCmd *cmd)
{
    struct SS ss_mut = *ss;
    if (SSBitsLeft(&ss_mut) < DS_SIM_CMD_TYPE_BITS)
    {
        return 0;
    }

    const u64 mem_left = mem->mem_left;
    const u64 type = SSReadU64Partial(&ss_mut, DS_SIM_CMD_TYPE_BITS);
    u32 success;
    switch (type)
    {
        case SIM_CMD_CONFIG_LOAD:   { success = ds_LoadConfigCmdTryDeserializeBitPacked(mem, &ss_mut, &cmd->load_config); } break;
        case SIM_CMD_BODY_ADD:      { success = ds_BodyAddCmdTryDeserializeBitPacked(mem, &ss_mut, &cmd->body_add); } break;
        case SIM_CMD_SHAPE_ADD:     { success = ds_ShapeAddCmdTryDeserializeBitPacked(mem, &ss_mut, &cmd->shape_add); } break;
        case SIM_CMD_JOINT_ADD:     { success = ds_JointAddCmdTryDeserializeBitPacked(&ss_mut, &cmd->joint_add); } break;
        case SIM_CMD_MOUSE_JOINT_ADD: { success = ds_MouseJointAddCmdTryDeserializeBitPacked(&ss_mut, &cmd->mouse_joint_add); } break;
        case SIM_CMD_MOUSE_JOINT_TARGET_SET: { success = ds_MouseJointTargetSetCmdTryDeserializeBitPacked(&ss_mut, &cmd->mouse_joint_target_set); } break;
        case SIM_CMD_JOINT_REMOVE: { success = ds_JointRemoveCmdTryDeserializeBitPacked(&ss_mut, &cmd->joint_remove); } break;
        default:                    { success = 0; } break;
    }

    if (!success)
    {
        ArenaPopPacked(mem, mem_left - mem->mem_left);
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

struct ds_SimCmdPayload ds_SimCmdPayloadAlloc(struct arena *mem, const ds_SimId dst, const u64 bytesize)
{
    return (struct ds_SimCmdPayload)
    {
        .dst = dst,
        .cmd_count = 0,
        .stream = SSAlloc(mem, bytesize),
    };
}

void ds_SimCmdPayloadDealloc(struct ds_SimCmdPayload *payload)
{
    SSDealloc(&payload->stream);
    *payload = ds_SimCmdPayloadEmpty(ds_SimIdNull());
}

struct ds_SimCmdPayload ds_SimCmdPayloadPushAll(struct arena *mem, const ds_SimId dst, const ds_CPool(ds_SimCmd) *pool)
{
    u64 bits = 0;
    for (u32 i = 0; i < pool->count; ++i)
    {
        bits += ds_SimCmdSerializeBitPackedSize(pool->buf + i);
    }

    struct ds_SimCmdPayload payload = ds_SimCmdPayloadAlloc(mem, dst, (bits + 7) / 8);
    if (bits && payload.stream.buf == NULL)
    {
        Log(T_SYSTEM, S_ERROR, "ds_SimCmdPayloadPushAll: no room for %lu bytes, %u commands dropped", (bits + 7) / 8, pool->count);
        return payload;
    }

    for (u32 i = 0; i < pool->count; ++i)
    {
        ds_SimCmdSerializeBitPacked(&payload.stream, pool->buf + i);
    }
    payload.cmd_count = pool->count;

    return payload;
}

u32 ds_SimCmdPayloadAppend(struct ds_SimCmdPayload *payload, const struct ds_SimCmdPayload *src)
{
    struct SSView view;     /* src's used bits */
    if (!SSTryView(&view, &src->stream, 0, src->stream.bit_index)
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

    payload->cmd_count += src->cmd_count;
    return 1;
}

#define DS_SIM_CMD_PAYLOAD_FIXED_SIZE   (sizeof(u64) + 2*sizeof(u32))

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
    if (SSBytesLeft(&ss_mut) < DS_SIM_CMD_PAYLOAD_FIXED_SIZE)
    {
        return 0;
    }

    const ds_SimId dst = { SSReadU64Le(&ss_mut) };
    const u32 cmd_count = SSReadU32Le(&ss_mut);
    const u64 bit_count = SSReadU32Le(&ss_mut);
    const u64 byte_count = (bit_count + 7) / 8;
    if (SSBytesLeft(&ss_mut) < byte_count)
    {
        return 0;
    }

    u8 *payload_base = ss_mut.buf + ss_mut.bit_index / 8;
    const u32 pad = (bit_count % 8)
                       ? payload_base[byte_count - 1] >> (bit_count % 8)
                       : 0;
    if (pad)
    {
        return 0;
    }

    payload->dst = dst;
    payload->cmd_count = cmd_count;
    payload->stream = SSBuffered(payload_base, byte_count);
    payload->stream.bit_index = bit_count;
    payload->stream.bit_count = bit_count;
    ss_mut.bit_index += 8*byte_count;
    *ss = ss_mut;
    return 1;
}
