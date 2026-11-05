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

/* Record reader: the next entry's timestamp (ss's position kept). Returns 0 on a bad stream. */
static u32 ds_SimHostReplayPeek(const struct SS *record, const u64 us_previous, u64 *us_timestamp)
{
    struct SS ss = *record;
    u64 us_delta;
    if (!ds_CompactSizeTryDeserialize(&ss, &us_delta))
    {
        return 0;
    }

    *us_timestamp = us_previous + us_delta;
    return 1;
}

/*
 * Record reader: the next entry's commands onto cmds (their data on mem), *us_previous set to its timestamp. Returns
 * 0 on a bad stream or a malformed command: the stream position and cmds are as before.
 */
static u32 ds_SimHostReplayTryReadPayload(struct SS *record, u64 *us_previous, struct arena *mem, ds_CPool(ds_SimCmd) *cmds)
{
    struct SS ss = *record;
    const u32 old_count = cmds->count;
    u64 us_delta, cmd_count, bit_count;
    if (!ds_CompactSizeTryDeserialize(&ss, &us_delta)
        || !ds_CompactSizeTryDeserialize(&ss, &cmd_count)
        || !ds_CompactSizeTryDeserialize(&ss, &bit_count)
        || SSBytesLeft(&ss) < (bit_count + 7) / 8)
    {
        return 0;
    }

    struct SSView view;
    SSTryView(&view, &ss, ss.bit_index, bit_count);
    struct SS commands = SSSubstreamFromView(&view);
    u32 valid = 1;
    for (u64 c = 0; c < cmd_count && valid; ++c)
    {
        struct ds_SimCmd *cmd = ds_CPoolPush(*cmds).address;
        valid = ds_SimCmdTryDeserialize(mem, &commands, cmd) && ds_SimCmdFormVerify(cmd);
    }

    if (!valid || SSBitsLeft(&commands) != 0)
    {
        ds_CPoolPopBatch(*cmds, cmds->count - old_count);
        return 0;
    }

    ss.bit_index += 8 * ((bit_count + 7) / 8);
    *record = ss;
    *us_previous += us_delta;
    return 1;
}

/* Record reader: the seed entry (one ConfigLoad at 0) from the start of record; config set, the position after it. */
static u32 ds_SimHostReplayTrySeed(struct SS *record, u64 *us_previous, struct ds_SimConfig *config)
{
    record->bit_index = 0;
    *us_previous = 0;

    struct arena *tmp = ArenaPushScratch();
    ds_CPool(ds_SimCmd) cmds;
    ds_CPoolAlloc(tmp, cmds, 1, NOT_GROWABLE);
    const u32 valid = ds_SimHostReplayTryReadPayload(record, us_previous, tmp, &cmds)
                   && *us_previous == 0 && cmds.count == 1 && cmds.buf[0].type == SIM_CMD_CONFIG_LOAD;
    if (valid)
    {
        *config = *cmds.buf[0].load_config.config;
    }
    ArenaPopScratch();

    return valid;
}

static void ds_SimHostDealloc(struct ds_SimHost *host)
{
    FileClose(&host->record_file);
    if (host->mode == SIM_MODE_REPLAY)
    {
        FileMemoryUnmap(host->record_stream.buf, host->record_stream.bit_count / 8);
    }
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
        Log(T_SYSTEM, S_ERROR, "ds_SimHostAdd: record file \"%s\" could not be %s, no simulation added", path, (mode == SIM_MODE_RECORD) ? "created" : "opened");
        return ds_SimIdNull();
    }

    struct ds_SimConfig config_used = (config) ? *config : ds_SimConfigDefault();
    struct SS record = { 0 };
    u64 us_previous = 0;
    if (mode == SIM_MODE_REPLAY)
    {
        u64 size = 0;
        void *map = FileMemoryMap(&size, &file, FS_PROT_READ, FS_MAP_PRIVATE);
        record = SSBuffered(map, size);
        if (map == NULL || !ds_SimHostReplayTrySeed(&record, &us_previous, &config_used))
        {
            Log(T_SYSTEM, S_ERROR, "ds_SimHostAdd: record file \"%s\" has no seed entry, no simulation added", path);
            if (map) { FileMemoryUnmap(map, size); }
            FileClose(&file);
            return ds_SimIdNull();
        }
    }
    config = &config_used;

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
    host->record_stream = (mode == SIM_MODE_RECORD) ? SSAlloc(NULL, 4096) : record;
    host->us_previous_record = us_previous;
    host->replay_ended = 0;
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
    else if (host->mode == SIM_MODE_REPLAY)
    {
        struct ds_SimConfig seed;
        const u32 valid = ds_SimHostReplayTrySeed(&host->record_stream, &host->us_previous_record, &seed);
        ds_Assert(valid);
        host->replay_ended = 0;
    }
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
        if (host == NULL || (host->mode == SIM_MODE_REPLAY && !host->replay_ended))
        {
            Log(T_SYSTEM, S_WARNING, "ds_ServerTick: payload for %s simulation %lu dropped", (host) ? "replaying" : "unknown", payload->dst.val);
            *payload = ds_SimCmdPayloadEmpty(ds_SimIdNull());
            continue;
        }

//TODO guesses until there is data: what one payload may make the door decode (a hostile sender picks the counts)
#define DS_SIM_PAYLOAD_CMD_COUNT_MAX    4096
#define DS_SIM_PAYLOAD_BIT_COUNT_MAX    (1 << 20)
        if (payload->cmd_count > DS_SIM_PAYLOAD_CMD_COUNT_MAX || payload->stream.bit_index > DS_SIM_PAYLOAD_BIT_COUNT_MAX)
        {
            Log(T_SYSTEM, S_WARNING, "ds_ServerTick: payload of %u commands, %lu bits for simulation %lu dropped (over the cap)", payload->cmd_count, payload->stream.bit_index, payload->dst.val);
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
        const u64 us_sim_delta = ns_sim_delta / 1000;

        if (host->mode == SIM_MODE_REPLAY && !host->replay_ended)
        {
            const u64 sim_us_time_final = host->sim.us_time + us_sim_delta;
            while (1)
            {
                if (SSBitsLeft(&host->record_stream) == 0)
                {
                    Log(T_SYSTEM, S_NOTE, "ds_ServerTick: simulation %lu's record ends at %lu us, it runs on live", host->id.val, host->us_previous_record);
                    host->replay_ended = 1;
                    break;
                }

                u64 us_entry;
                if (!ds_SimHostReplayPeek(&host->record_stream, host->us_previous_record, &us_entry))
                {
                    Log(T_SYSTEM, S_ERROR, "ds_ServerTick: simulation %lu's record is corrupt after %lu us, it runs on live", host->id.val, host->us_previous_record);
                    host->replay_ended = 1;
                    break;
                }

                if (us_entry > sim_us_time_final)
                {
                    break;
                }

                ds_SimTick(&host->sim, &host->cmd_pool, us_entry - host->sim.us_time);
                if (!ds_SimHostReplayTryReadPayload(&host->record_stream, &host->us_previous_record, &srv->frame, &host->cmd_pool))
                {
                    Log(T_SYSTEM, S_ERROR, "ds_ServerTick: simulation %lu's record is corrupt at %lu us, it runs on live", host->id.val, us_entry);
                    host->replay_ended = 1;
                    break;
                }

                ds_SimTick(&host->sim, &host->cmd_pool, 0);
                ds_CPoolFlush(host->cmd_pool);
            }

            ds_SimTick(&host->sim, &host->cmd_pool, sim_us_time_final - host->sim.us_time);
            continue;
        }

        if (host->mode == SIM_MODE_RECORD && host->frame_payload.cmd_count)
        {
            ds_SimHostRecord(host, host->sim.us_time, &host->frame_payload);
        }

        ds_SimTick(&host->sim, &host->cmd_pool, us_sim_delta);
        ds_CPoolFlush(host->cmd_pool);
    }

    ArenaFlush(&srv->frame);    /* arrivals until the next tick may live here */
}
