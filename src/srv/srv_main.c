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
#include "ds_compact_size.h"


/**************** ds_Server ****************/

struct slot ds_SimHostLookup(const struct ds_Server *srv, const ds_SimId id)
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

/* Append the entry (us_timestamp, payload) to the host's record file, staged on its record stream. */
/*
 * Record entry: timestamp delta from the previous entry, cmd_count and the command bit count as CompactSize, then
 * the command bytes (the last byte's unused bits zero). No destination: the file is the simulation's.
 */
static void ds_SimHostRecord(struct ds_SimHost *host, const u64 us_timestamp, const struct ds_SimCmdPayload *payload)
{
    ds_Assert(us_timestamp >= host->us_previous_record);
    const u64 us_delta = us_timestamp - host->us_previous_record;
    const u64 bit_count = payload->stream.bit_index;
    const u64 size = ds_CompactSizeSerializeSize(us_delta) + ds_CompactSizeSerializeSize(payload->cmd_count)
                   + ds_CompactSizeSerializeSize(bit_count) + (bit_count + 7) / 8;
    SSIncreaseSize(&host->record_stream, size);
    host->record_stream.bit_index = 0;
    ds_CompactSizeSerialize(&host->record_stream, us_delta);
    ds_CompactSizeSerialize(&host->record_stream, payload->cmd_count);
    ds_CompactSizeSerialize(&host->record_stream, bit_count);
    SSWriteU8N(&host->record_stream, payload->stream.buf, (bit_count + 7) / 8);
    host->us_previous_record = us_timestamp;
    if (FileWriteAppend(&host->record_file, host->record_stream.buf, size) != size)
    {
        Log(T_SYSTEM, S_ERROR, "ds_SimHostRecord: simulation %lu's record entry at %lu us not written whole", host->id.val, us_timestamp);
    }
}

/* The seed entry: the simulation's seed batch at time 0. */
static void ds_SimHostRecordSeed(struct ds_SimHost *host)
{
    struct arena *tmp = ArenaPushScratch();
    const struct ds_SimCmd seed = { .type = SIM_CMD_CONFIG_LOAD, .load_config.config = &host->sim.config_seed };
    ds_CPool(ds_SimCmd) cmds;
    ds_CPoolAlloc(tmp, cmds, 1, NOT_GROWABLE);
    ds_CPoolPushValue(cmds, seed);
    const struct ds_SimCmdPayload payload = ds_SimCmdPayloadPushAll(tmp, host->id, &cmds);
    ds_SimHostRecord(host, 0, &payload);
    ArenaPopScratch();
}

static void ds_SimHostDealloc(struct ds_SimHost *host)
{
    FileClose(&host->record_file);
    SSDealloc(&host->record_stream);
    ds_CPoolDealloc(host->cmd_pool);
    ds_SimDealloc(&host->sim);
}

ds_SimId ds_SimHostAdd(struct ds_Server *srv, struct arena *mem, const struct ds_SimConfig *config, const enum ds_SimMode mode, const char *path, struct c_ShapeDB *cshape_db, ds_SDB(ds_BodyPrefab) *prefab_db, const u32 worker_count, const u64 worker_frame_size)
{
    struct file file = FileNull();
    if ((mode == SIM_MODE_RECORD && FileTryCreateAtCwd(mem, &file, path, 1) != FS_SUCCESS)
        || (mode == SIM_MODE_REPLAY && FileTryOpenAtCwd(mem, &file, path, 0) != FS_SUCCESS))
    {
        Log(T_SYSTEM, S_ERROR, "ds_SimAdd: record file \"%s\" could not be %s, no simulation added", path, (mode == SIM_MODE_RECORD) ? "created" : "opened");
        return ds_SimIdNull();
    }

    const u32 old_max = srv->sim_pool.count_max;
    const struct slot slot = ds_PoolAdd(srv->sim_pool);
    struct ds_SimHost *host = slot.address;
    if (old_max != srv->sim_pool.count_max)
    {
        host->id = ds_SimIdInit(slot.index, 0);
    }
    host->id = ds_SimIdNextGeneration(host->id);

    host->mode = mode;
    host->speed = 1.0;
    host->ns_remainder = 0;
    host->ns_budget = 0;
    for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
    {
        host->ns_budget += 1000 * config->us_system_tick[i];
    }
    host->record_file = file;
    host->record_stream = (mode == SIM_MODE_RECORD) ? SSAlloc(NULL, 4096) : (struct SS) { 0 };
    host->us_previous_record = 0;
    ds_CPoolAlloc(NULL, host->cmd_pool, 64, GROWABLE);
    host->frame_payload = ds_SimCmdPayloadEmpty(host->id);

    ds_SimAlloc(mem, &host->sim, config, cshape_db, prefab_db, worker_count, worker_frame_size);
    DLLAppend(srv->sim_list, srv->sim_pool.buf, slot.index, srv_sim);

    if (mode == SIM_MODE_RECORD)
    {
        ds_SimHostRecordSeed(host);
    }

    return host->id;
}

void ds_SimHostReset(struct ds_Server *srv, const ds_SimId id)
{
    struct ds_SimHost *host = ds_SimHostLookup(srv, id).address;
    if (host == NULL)
    {
        Log(T_SYSTEM, S_WARNING, "ds_SimHostReset: no simulation %lu", id.val);
        return;
    }

    ds_CPoolFlush(host->cmd_pool);
    host->frame_payload = ds_SimCmdPayloadEmpty(host->id);
    ds_SimFlush(&host->sim);

    if (host->mode == SIM_MODE_RECORD)
    {
        if (!FileSetSize(&host->record_file, 0))
        {
            Log(T_SYSTEM, S_ERROR, "ds_SimHostReset: simulation %lu's record could not be truncated", id.val);
        }
        host->us_previous_record = 0;
        ds_SimHostRecordSeed(host);
    }
    //TODO REPLAY: rewind the record
}

void ds_SimHostRemove(struct ds_Server *srv, const ds_SimId id)
{
    const struct slot slot = ds_SimHostLookup(srv, id);
    if (slot.address == NULL)
    {
        Log(T_SYSTEM, S_WARNING, "ds_SimRemove: no simulation %lu", id.val);
        return;
    }

    ds_SimHostDealloc(slot.address);
    DLLRemove(srv->sim_list, srv->sim_pool.buf, slot.index, srv_sim);
    ds_PoolRemove(srv->sim_pool, slot.index);
}

void ds_ServerDealloc(struct ds_Server *srv)
{
    for (i32 i = srv->sim_list.first; i != DLL_SENTINEL; i = srv->sim_pool.buf[i].srv_sim.next)
    {
        ds_SimHostDealloc(srv->sim_pool.buf + i);
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
        struct ds_SimHost *host = ds_SimHostLookup(srv, payload->dst).address;
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

        struct ds_SimHost *host = ds_SimHostLookup(srv, payload->dst).address;
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

        if (host->mode == SIM_MODE_RECORD && host->frame_payload.cmd_count)
        {
            ds_SimHostRecord(host, host->sim.us_time, &host->frame_payload);
        }

        ds_SimTick(&host->sim, &host->cmd_pool, ns_sim_delta / 1000);
        ds_CPoolFlush(host->cmd_pool);
    }
}
