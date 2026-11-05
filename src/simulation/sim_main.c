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
Todo-List (not complete, need to fill in as we get more questions)

() mode=Record 
    () record serialization
    () record deserialization and validation
() mode=Replay 
    () ds_AllocReplay
    () generalize simulation loop to handle replay binaries
() Commands
    () ds_SimConfigLoad         (always at time 0)
    () ds_DynamicsConfigUpdate  (Applies pending config updates)
    () ... port all old intput handling to commands
() CVars/CmdLine
    () need proper way of handling terminal inputs (modes, files, ...)

() Port determinism / teests / all main functions to provide config_path=null,path + mode=replay,record,none.
 */

#include "ds_base.h"
#include "ds_platform.h"
#include "ds_simulation.h"

ds_StaticAssert(SIM_SYSTEM_COUNT <= (1u << DS_SIM_KEY_SYSTEM_BITS), "the system tag must fit the key");

static ds_SimKey ds_SimKeyNull(void)
{
    return U64_MAX;
}

static ds_SimKey ds_SimKeyInit(const u64 us_timestamp, const enum ds_SimSystem system)
{
    ds_Assert(us_timestamp < (((u64) 1) << (64 - DS_SIM_KEY_SYSTEM_BITS)));
    ds_Assert((u32) system < (1u << DS_SIM_KEY_SYSTEM_BITS));
    return (us_timestamp << DS_SIM_KEY_SYSTEM_BITS) | (u64) system;
}

static u64 ds_SimKeyUs(const ds_SimKey key)
{
    return key >> DS_SIM_KEY_SYSTEM_BITS;
}

static enum ds_SimSystem ds_SimKeySystem(const ds_SimKey key)
{
    return (enum ds_SimSystem) (key & ((((u64) 1) << DS_SIM_KEY_SYSTEM_BITS) - 1));
}

/**************** ds_SimConfig ****************/

/* seed nonzero, external tick 0, other ticks in [1, DS_SIM_TICK_US_MAX] */
static u32 ds_SimConfigCheck(const struct ds_SimConfig *config)
{
    if ((config->seed[0] | config->seed[1] | config->seed[2] | config->seed[3]) == 0)
    {
        return 0;
    }

    if (config->us_system_tick[SIM_SYSTEM_EXTERNAL] != 0)
    {
        return 0;
    }

    for (u32 i = SIM_SYSTEM_EXTERNAL + 1; i < SIM_SYSTEM_COUNT; ++i)
    {
        if (config->us_system_tick[i] == 0 || config->us_system_tick[i] > DS_SIM_TICK_US_MAX)
        {
            return 0;
        }
    }

    return 1;
}

struct ds_SimConfig ds_SimConfigDefault(void)
{
    struct ds_SimConfig config = { 0 };

    config.seed[0] = 1;
    config.seed[1] = 3;
    config.seed[2] = 3;
    config.seed[3] = 7;

    config.us_system_tick[SIM_SYSTEM_EXTERNAL] = 0;
    config.us_system_tick[SIM_SYSTEM_DYNAMICS] = 1000000 / 60;
    config.us_system_tick[SIM_SYSTEM_GAME]     = 1000000 / 60;

    config.dynamics = ds_DynamicsConfigDefault();

    return config;
}

void ds_SimConfigSeed(struct ds_SimConfig *config, const u64 seed[4])
{
    const struct ds_SimConfig def = ds_SimConfigDefault();
    const u32 valid = (seed[0] | seed[1] | seed[2] | seed[3]) != 0;
    if (!valid)
    {
        LogString(T_SYSTEM, S_WARNING, "ds_SimConfigSeed: the all-zero seed is invalid, using the default seed");
    }

    for (u32 i = 0; i < 4; ++i)
    {
        config->seed[i] = (valid) ? seed[i] : def.seed[i];
    }
}

void ds_SimConfigGenerateSeed(struct ds_SimConfig *config)
{
    do
    {
        RngSystem(config->seed, sizeof(config->seed));
    } while ((config->seed[0] | config->seed[1] | config->seed[2] | config->seed[3]) == 0);
}

#define DS_SIM_CONFIG_VERSION       2u
/* version + seed + ticks */
#define DS_SIM_CONFIG_FIXED_SIZE    (4 + 4*8 + SIM_SYSTEM_COUNT*8)

u64 ds_SimConfigSerializeSize(const struct ds_SimConfig *config)
{
    return DS_SIM_CONFIG_FIXED_SIZE + ds_DynamicsConfigSerializeSize(&config->dynamics);
}

void ds_SimConfigSerialize(struct SS *ss, const struct ds_SimConfig *config)
{
    ds_Assert(ss->bit_index % 8 == 0);
    ds_Assert(ds_SimConfigSerializeSize(config) <= SSBytesLeft(ss));

    SSWriteU32Le(ss, DS_SIM_CONFIG_VERSION);
    for (u32 i = 0; i < 4; ++i)
    {
        SSWriteU64Le(ss, config->seed[i]);
    }

    for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
    {
        SSWriteU64Le(ss, config->us_system_tick[i]);
    }

    ds_DynamicsConfigSerialize(ss, &config->dynamics);
}

u32 ds_SimConfigTryDeserialize(struct SS *ss, struct ds_SimConfig *config)
{
    const u64 bit_index = ss->bit_index;
    u32 success = 0;

    if (ss->bit_index % 8 != 0 || SSBytesLeft(ss) < DS_SIM_CONFIG_FIXED_SIZE)
    {
        goto end;
    }

    if (SSReadU32Le(ss) != DS_SIM_CONFIG_VERSION)
    {
        goto end;
    }

    for (u32 i = 0; i < 4; ++i)
    {
        config->seed[i] = SSReadU64Le(ss);
    }

    for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
    {
        config->us_system_tick[i] = SSReadU64Le(ss);
    }

    success = ds_DynamicsConfigTryDeserialize(ss, &config->dynamics) && ds_SimConfigCheck(config);

end:
    if (!success)
    {
        ss->bit_index = bit_index;
        *config = (struct ds_SimConfig) { 0 };
    }

    return success;
}

/**************** ds_Sim ****************/

/* time 0: the seed batch first, then each system's first tick one period in */
static void ds_SimScheduleInit(struct ds_Sim *sim)
{
    sim->us_time = 0;
    sim->us_delta = 0;
    sim->ns_remainder = 0;

    sim->sys_schedule[SIM_SYSTEM_EXTERNAL] = ds_SimKeyNull();   /* the seed is a batch (ds_SimSeed) */
    for (u32 i = SIM_SYSTEM_EXTERNAL + 1; i < SIM_SYSTEM_COUNT; ++i)
    {
        sim->sys_schedule[i] = ds_SimKeyInit(sim->us_system_tick[i], (enum ds_SimSystem) i);
    }
}

/*
 * Seed the simulation at time 0: the config, as a ConfigLoad batch, goes through an ordinary frame so that live and
 * replay apply (and record) it the same way. Everything here lives within the call.
 */
static void ds_SimSeed(struct ds_Sim *sim, const struct ds_SimConfig *config)
{
    const struct ds_SimCmd seed = { .type = SIM_CMD_CONFIG_LOAD, .load_config.config = config };
    ds_CPoolPushValue(sim->cmd_pool, seed);
    ds_SimTick(sim, 0);     
    //TODO RECORD: the seed batch as a payload, the record's head (point 5)
}

void ds_SimAlloc(struct arena *mem, struct ds_Sim *sim, const struct ds_SimConfig *config, const u32 record, struct c_ShapeDB *cshape_db, ds_SDB(ds_BodyPrefab) *prefab_db, const u32 worker_count, const u64 worker_frame_size)
{
    ds_Assert(ds_SimConfigCheck(config));

    *sim = (struct ds_Sim) { 0 };
    sim->mode = (record) ? SIM_MODE_RECORD : SIM_MODE_NONE;
    sim->config_seed = *config;
    sim->speed = 1.0;

    sim->ns_budget = 0;
    for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
    {
        sim->us_system_tick[i] = config->us_system_tick[i];
        sim->ns_budget += 1000 * config->us_system_tick[i];
    }

    ds_CPoolAlloc(NULL, sim->payload_pool, 16, GROWABLE);
    ds_CPoolAlloc(NULL, sim->cmd_pool, 64, GROWABLE);

    sim->dynamics = ds_DynamicsAllocConfig(mem, &config->dynamics, 1000 * config->us_system_tick[SIM_SYSTEM_DYNAMICS], cshape_db, prefab_db, worker_count, worker_frame_size);
    //TODO record mode: open the record (the seed batch is its head)

    ds_SimScheduleInit(sim);
    ds_SimSeed(sim, &sim->config_seed);
}

void ds_SimDealloc(struct ds_Sim *sim)
{
    ds_DynamicsFree(&sim->dynamics);
    ds_CPoolDealloc(sim->payload_pool);
    ds_CPoolDealloc(sim->cmd_pool);
    //TODO record mode: close the record
}

void ds_SimFlush(struct ds_Sim *sim)
{
    ds_CPoolFlush(sim->payload_pool);
    ds_CPoolFlush(sim->cmd_pool);
    ds_DynamicsFlush(&sim->dynamics);
    ds_SimScheduleInit(sim);
    sim->initialized = 0;
    //TODO record mode: restart the record
    ds_SimSeed(sim, &sim->config_seed);
}

static void ds_SimExecuteCmdBatch(struct ds_Sim *sim)
{
    //TODO REPLAY: decode the record's batch onto cmd_pool first (point 6)
    for (u32 i = 0; i < sim->cmd_pool.count; ++i)
    {
        ds_SimCmdTryExecute(sim, sim->cmd_pool.buf + i);
    }
    ds_CPoolFlush(sim->cmd_pool);
}

void ds_SimTick(struct ds_Sim *sim, const u64 ns_delta)
{
    ds_DynamicsConfigPush(&sim->dynamics); 

    /* If external commands exist, they will be prioritized and executed immediately. */
    sim->sys_schedule[SIM_SYSTEM_EXTERNAL] = (sim->cmd_pool.count)
                                           ? ds_SimKeyInit(sim->us_time + 1, SIM_SYSTEM_EXTERNAL)
                                           : ds_SimKeyNull();

    const u64 ns_wanted = sim->ns_remainder + (u64) (ns_delta*sim->speed);
    const u64 ns_sim_delta = U64Min(ns_wanted, sim->ns_budget);
    sim->ns_remainder = ns_sim_delta % 1000;
    sim->us_delta = ns_sim_delta / 1000;
    sim->us_time += sim->us_delta;

    //TODO record the batch: its key (sys_schedule[SIM_SYSTEM_EXTERNAL]) + srv_frame_payload (RECORD); REPLAY: read the record's due entry instead
    while (1)
    {
        ds_SimKey key = ds_SimKeyNull();
        for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
        {
            key = U64Min(key, sim->sys_schedule[i]);
        }

        const u64 us = ds_SimKeyUs(key);
        const enum ds_SimSystem sys = ds_SimKeySystem(key);

        /* 
         * Last check comes from us always executing external command payloads
         * even if us_old == us_new, i.e. because we allow multiple command
         * payloads per time-unit (simplifies memory management in server).
         */
        if (sim->us_time < us && sys != SIM_SYSTEM_EXTERNAL)
        {
            break;
        }

        sim->sys_schedule[sys] = (sys == SIM_SYSTEM_EXTERNAL)
                               ? ds_SimKeyNull()
                               : ds_SimKeyInit(us + sim->us_system_tick[sys], sys);

        switch (sys)
        {
            case SIM_SYSTEM_EXTERNAL:
            {
                ds_SimExecuteCmdBatch(sim);
            } break;

            case SIM_SYSTEM_DYNAMICS:
            {
                ds_DynamicsTick(&sim->dynamics);
            } break;

            case SIM_SYSTEM_GAME:
            {
                //TODO  execute game
            } break;

            default:
            {
                ds_AssertString(0, "Invalid system in ds_SimKey");
            } break;
        }
    }

    ds_DynamicsConfigPop(&sim->dynamics);
}
