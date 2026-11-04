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

/* Return the slot of the simulation with the given id, or { NULL, U32_MAX } if there is none. */
static struct slot ds_SimLookup(const struct ds_Server *srv, const ds_SimId id)
{
    struct slot slot = { .address = NULL, .index = U32_MAX };
    /* the index is bounded before the slot is read: a foreign id must not read past the pool */
    const u32 index = ds_SimIdIndex(id);
    if (index < srv->sim_pool.count_max)
    {
        struct ds_Sim *sim = srv->sim_pool.buf + index;
        if (ds_PoolSlotAllocated(sim) && ds_SimIdEqual(sim->id, id))
        {
            slot.address = sim;
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
        ds_SimDealloc(srv->sim_pool.buf + i);
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
        srv->sim_pool.buf[i].srv_frame_payload.stream.bit_count = 0;
    }

    for (u32 i = 0; i < srv->payload_pool.count; ++i)
    {
        struct ds_SimCmdPayload *payload = srv->payload_pool.buf + i;
        struct ds_Sim *sim = ds_SimLookup(srv, payload->dst).address;
        if (sim == NULL)
        {
            Log(T_SYSTEM, S_WARNING, "ds_ServerTick: payload for unknown simulation %lu dropped", payload->dst.val);
            *payload = ds_SimCmdPayloadEmpty(ds_SimIdNull());
            continue;
        }

        sim->srv_frame_payload.stream.bit_count += payload->stream.bit_index;
    }

    for (i32 i = srv->sim_list.first; i != DLL_SENTINEL; i = srv->sim_pool.buf[i].srv_sim.next)
    {
        struct ds_Sim *sim = srv->sim_pool.buf + i;
        const u64 bit_count = sim->srv_frame_payload.stream.bit_count;
        const u64 size =  (bit_count + 7) / 8;
        sim->srv_frame_payload = ds_SimCmdPayloadAlloc(&srv->frame, sim->id, size);

        if (bit_count && sim->srv_frame_payload.stream.buf == NULL)
        {
            Log(T_SYSTEM, S_ERROR, "ds_ServerTick: no frame memory for %lu bytes, simulation %lu's payloads dropped", size, sim->id.val);
            sim->srv_frame_payload = ds_SimCmdPayloadEmpty(sim->id);
        }
    }

    for (u32 i = 0; i < srv->payload_pool.count; ++i)
    {
        const struct ds_SimCmdPayload *payload = srv->payload_pool.buf + i;
        if (ds_SimIdNullCheck(payload->dst))
        {
            continue;
        }

        struct ds_Sim *sim = ds_SimLookup(srv, payload->dst).address;
        if (sim->srv_frame_payload.stream.bit_count && !ds_SimCmdPayloadAppend(&sim->srv_frame_payload, payload))
        {
            Log(T_SYSTEM, S_ERROR, "ds_ServerTick: payload of %u commands for simulation %lu dropped (no room)", payload->cmd_count, payload->dst.val);
        }
    }

    /*************** Payload Timestamping and Recording ***************/

    /*
     * At this point the CPool holding external payloads may be flushed since 
     * the payloads are viewed as aliasing some buffers somewhere else.
     *
     * NOTE: This may change, be wary...
     */

    ds_CPoolFlush(srv->payload_pool); 

    // TODO stamp + record per simulation

    for (i32 i = srv->sim_list.first; i != DLL_SENTINEL; i = srv->sim_pool.buf[i].srv_sim.next)
    {
    }
}

/* Stamp and record the frame payload (RECORD), then execute external_cmd_buf in order and flush the frame's buffers. */
static void ds_SimDispatchFramePayload(struct ds_Sim *sim)
{
    //TODO stamp (us_time_old + 1) | EXTERNAL and record the frame payload
    //TODO ds_SimCmdTryExecute each command of external_cmd_buf in order
    ds_CPoolFlush(sim->external_cmd_buf);
    ds_CPoolFlush(sim->payload_pool);
}

