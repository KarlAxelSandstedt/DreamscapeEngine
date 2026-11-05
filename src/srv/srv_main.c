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
#include "ds_server.h"


/**************** ds_Server ****************/

/* Return the slot of the hosted simulation with the given id, or { NULL, U32_MAX } if there is none. */
static struct slot ds_SimLookup(const struct ds_Server *srv, const ds_SimId id)
{
    struct slot slot = { .address = NULL, .index = U32_MAX };
    /* the index is bounded before the slot is read: a foreign id must not read past the pool */
    const u32 index = ds_SimIdIndex(id);
    if (index < srv->sim_pool.count_max)
    {
        struct ds_SimHost *host = srv->sim_pool.buf + index;
        if (ds_PoolSlotAllocated(host) && ds_SimIdEqual(host->id, id))
        {
            slot.address = host;
            slot.index = index;
        }
    }

    return slot;
}

void ds_ServerAlloc(struct ds_Server *srv, const u64 frame_size, const u32 initial_sim_count)
{
    *srv = (struct ds_Server) { 0 };
    srv->frame = ArenaAlloc(NULL, frame_size);
    ds_PoolAlloc(NULL, srv->sim_pool, initial_sim_count, GROWABLE);
    DLLFlush(srv->sim_list);
    ds_CPoolAlloc(NULL, srv->payload_pool, 64, GROWABLE);
}

void ds_ServerDealloc(struct ds_Server *srv)
{
    for (i32 i = srv->sim_list.first; i != DLL_SENTINEL; i = srv->sim_pool.buf[i].srv_sim.next)
    {
        struct ds_SimHost *host = srv->sim_pool.buf + i;
        ds_SimDealloc(&host->sim);
        ds_CPoolDealloc(host->cmd_pool);
    }
    ds_PoolDealloc(srv->sim_pool);
    ds_CPoolDealloc(srv->payload_pool);
    ArenaFree(&srv->frame);
}

void ds_ServerTick(struct ds_Server *srv, const u64 ns_delta)
{
    ArenaFlush(&srv->frame);

    /************** Payload Deserialization and Aliasing **************/

    /* TODO receive payloads (network, local), route by dst. */

    /************************* Payload Fusing *************************/

    /*
     * At this point, we have a CPool of valid payloads which
     * we may not store and use beyond this fuse code (payload
     * memory will be managed by multiple remote systems). All
     * ops on payloads will stem from our frame-lifetime fused
     * payloads.
     */

    for (i32 i = srv->sim_list.first; i != DLL_SENTINEL; i = srv->sim_pool.buf[i].srv_sim.next)
    {
        srv->sim_pool.buf[i].frame_payload.stream.bit_count = 0;
    }

    for (u32 i = 0; i < srv->payload_pool.count; ++i)
    {
        struct ds_SimCmdPayload *payload = srv->payload_pool.buf + i;
        struct ds_SimHost *host = ds_SimLookup(srv, payload->dst).address;
        if (host == NULL)
        {
            Log(T_SYSTEM, S_WARNING, "ds_ServerTick: payload for unknown simulation %lu dropped", payload->dst.val);
            *payload = ds_SimCmdPayloadEmpty(ds_SimIdNull());
            continue;
        }

        const u32 old_count = host->cmd_pool.count;
        struct SSView view;
        SSTryView(&view, &payload->stream, 0, payload->stream.bit_index);
        struct SS ss = SSSubstreamFromView(&view);
        u32 valid = 1;
        for (u32 c = 0; c < payload->cmd_count && valid; ++c)
        {
            struct ds_SimCmd *cmd = ds_CPoolPush(host->cmd_pool).address;
            valid = ds_SimCmdTryDeserialize(&srv->frame, &ss, cmd) && ds_SimCmdFormVerify(cmd);
        }

        if (!valid || SSBitsLeft(&ss) != 0)
        {
            Log(T_SYSTEM, S_WARNING, "ds_ServerTick: malformed payload for simulation %lu dropped (%u commands)", payload->dst.val, payload->cmd_count);
            ds_CPoolPopBatch(host->cmd_pool, host->cmd_pool.count - old_count);
            *payload = ds_SimCmdPayloadEmpty(ds_SimIdNull());
            continue;
        }

        host->frame_payload.stream.bit_count += payload->stream.bit_index;
    }

    for (i32 i = srv->sim_list.first; i != DLL_SENTINEL; i = srv->sim_pool.buf[i].srv_sim.next)
    {
        struct ds_SimHost *host = srv->sim_pool.buf + i;
        const u64 bit_count = host->frame_payload.stream.bit_count;
        const u64 size =  (bit_count + 7) / 8;
        host->frame_payload = ds_SimCmdPayloadAlloc(&srv->frame, host->id, size);

        if (bit_count && host->frame_payload.stream.buf == NULL)
        {
            Log(T_SYSTEM, S_ERROR, "ds_ServerTick: no frame memory for %lu bytes, simulation %lu's payloads dropped", size, host->id.val);
            host->frame_payload = ds_SimCmdPayloadEmpty(host->id);
        }
    }

    for (u32 i = 0; i < srv->payload_pool.count; ++i)
    {
        const struct ds_SimCmdPayload *payload = srv->payload_pool.buf + i;
        if (ds_SimIdNullCheck(payload->dst))
        {
            continue;
        }

        struct ds_SimHost *host = ds_SimLookup(srv, payload->dst).address;
        if (host->frame_payload.stream.bit_count && !ds_SimCmdPayloadAppend(&host->frame_payload, payload))
        {
            Log(T_SYSTEM, S_ERROR, "ds_ServerTick: payload of %u commands for simulation %lu dropped (no room)", payload->cmd_count, payload->dst.val);
        }
    }

    /*************** Simulation Execution ***************/

    /*
     * At this point the CPool holding external payloads may be flushed since 
     * the payloads are viewed as aliasing some buffers somewhere else.
     *
     * NOTE: This may change, be wary...
     */
    ds_CPoolFlush(srv->payload_pool); 

    for (i32 i = srv->sim_list.first; i != DLL_SENTINEL; i = srv->sim_pool.buf[i].srv_sim.next)
    {
        struct ds_SimHost *host = srv->sim_pool.buf + i;

        /* wall-clock to simulation time: scaled by speed, the sub-microsecond rest carried, capped by the budget */
        const u64 ns_wanted = host->ns_remainder + (u64) (ns_delta * host->speed);
        const u64 ns_sim_delta = U64Min(ns_wanted, host->ns_budget);
        host->ns_remainder = ns_sim_delta % 1000;

        //TODO RECORD: timestamp (sim.us_time + 1) + frame_payload
        ds_SimTick(&host->sim, &host->cmd_pool, ns_sim_delta / 1000);
        ds_CPoolFlush(host->cmd_pool);
    }
}
