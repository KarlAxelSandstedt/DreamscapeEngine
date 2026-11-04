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


/*

TODO: go through

The first command, ConfigLoad: a serialized ds_SimConfig. It is the seed batch at time 0 (the record's head
re-applied) and, later, the host's apply of its pending edits; executing it derives the config (every value clamped
into its range) and applies it to the systems (ds_DynamicsConfigUpdatePending for the dynamics). The alloc-time
values (seed, tick periods, capacities) are ignored after time 0. TODO: implement.
*/

#include "ds_base.h"
#include "ds_simulation.h"


/**************** ds_SimCmd ****************/

/* Return 1 if cmd is well-formed, 0 otherwise. */
static u32 ds_SimCmdFormVerify(const struct ds_SimCmd *cmd)
{
    switch (cmd->type)
    {
        case SIM_CMD_CONFIG_LOAD:
        {
            return ds_SimConfigCheck(&cmd->load_config.config);
        }

        default:
        {
            return 0;
        }
    }
}

/* Return 1 if the well-formed cmd is a valid command within sim's context, 0 otherwise.*/
static u32 ds_SimCmdDataVerify(const struct ds_Sim *sim, const struct ds_SimCmd *cmd)
{
    switch (cmd->type)
    {
        case SIM_CMD_CONFIG_LOAD:
        {
            return 1;
        }

        default:
        {
            return 0;
        }
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
        case SIM_CMD_CONFIG_LOAD:
        {
            //TODO apply: dynamics config_pending = config.dynamics, ds_DynamicsConfigUpdatePending (game later)
        } break;

        default:
        {
            ds_AssertString(0, "ds_SimCmdTryExecute: unknown command type");
        } break;
    }

    return 1;
}

static u64 ds_SimCmdSerializeBitsize(const struct ds_SimCmd *cmd)
{
    //TODO
    return 0;
}

static void ds_SimCmdSerialize(struct SS *ss, const struct ds_SimCmd *cmd)
{
    //TODO
}

static u32 ds_SimCmdTryDeserialize(struct SS *ss, struct ds_SimCmd *cmd)
{
    //TODO
    return 0;
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
