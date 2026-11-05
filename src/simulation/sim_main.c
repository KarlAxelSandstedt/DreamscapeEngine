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
#include "xxhash.h"

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

/* seed nonzero, system ticks in [1, DS_SIM_TICK_US_MAX] */
static u32 ds_SimConfigCheck(const struct ds_SimConfig *config)
{
    if ((config->seed[0] | config->seed[1] | config->seed[2] | config->seed[3]) == 0)
    {
        return 0;
    }

    for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
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

#define DS_SIM_CONFIG_VERSION       3u
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

static void ds_SimScheduleInit(struct ds_Sim *sim)
{
    sim->us_time = 0;
    sim->us_delta = 0;

    for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
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
    struct arena *tmp = ArenaPushScratch();

    const struct ds_SimCmd seed = { .type = SIM_CMD_CONFIG_LOAD, .load_config.config = config };
    ds_CPool(ds_SimCmd) cmds;
    ds_CPoolAlloc(tmp, cmds, 1, NOT_GROWABLE);
    ds_CPoolPushValue(cmds, seed);
    ds_SimTick(sim, NULL, &cmds, 0);

    ArenaPopScratch();
}

void ds_SimAlloc(struct arena *mem, struct ds_Sim *sim, const struct ds_SimConfig *config, struct c_ShapeDB *cshape_db, ds_SDB(ds_BodyPrefab) *prefab_db, const u32 worker_count, const u64 worker_frame_size)
{
    ds_Assert(ds_SimConfigCheck(config));

    *sim = (struct ds_Sim) { 0 };
    sim->config_seed = *config;
    for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
    {
        sim->us_system_tick[i] = config->us_system_tick[i];
    }

    sim->dynamics = ds_DynamicsAllocConfig(mem, &config->dynamics, 1000 * config->us_system_tick[SIM_SYSTEM_DYNAMICS], cshape_db, prefab_db, worker_count, worker_frame_size);

    ds_SimScheduleInit(sim);
    ds_SimSeed(sim, &sim->config_seed);
}

void ds_SimDealloc(struct ds_Sim *sim)
{
    ds_DynamicsFree(&sim->dynamics);
}

void ds_SimFlush(struct ds_Sim *sim)
{
    ds_DynamicsFlush(&sim->dynamics);
    ds_SimScheduleInit(sim);
    sim->initialized = 0;
    ds_SimSeed(sim, &sim->config_seed);
}

static void ds_SimExecuteCmdBatch(struct ds_Sim *sim, const ds_CPool(ds_SimCmd) *cmds)
{
    for (u32 i = 0; i < cmds->count; ++i)
    {
        ds_SimCmdTryExecute(sim, cmds->buf + i);
    }
}

u64 ds_SimHash(const struct ds_Sim *sim)
{
    u64 words[SIM_SYSTEM_COUNT] = { 0 };
    words[SIM_SYSTEM_DYNAMICS] = ds_DynamicsHash(&sim->dynamics);
    //TODO words[SIM_SYSTEM_GAME]
    return XXH3_64bits(words, sizeof(words));
}

void ds_SimTick(struct ds_Sim *sim, ds_CPool(u64) *hash_pool, const ds_CPool(ds_SimCmd) *cmds, const u64 us_delta)
{
    ds_DynamicsConfigPush(&sim->dynamics); 

    ds_SimExecuteCmdBatch(sim, cmds);

    sim->us_delta = us_delta;
    sim->us_time += us_delta;

    while (1)
    {
        ds_SimKey key = ds_SimKeyNull();
        for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
        {
            key = U64Min(key, sim->sys_schedule[i]);
        }

        const u64 us = ds_SimKeyUs(key);
        const enum ds_SimSystem sys = ds_SimKeySystem(key);

        if (sim->us_time < us)
        {
            break;
        }

        sim->sys_schedule[sys] = ds_SimKeyInit(us + sim->us_system_tick[sys], sys);

        switch (sys)
        {
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

        if (hash_pool)
        {
            const u64 words[3] = { us, (u64) sys, ds_SimHash(sim) };
            ds_CPoolPushValue(*hash_pool, XXH3_64bits(words, sizeof(words)));
        }
    }

    ds_DynamicsConfigPop(&sim->dynamics);
}
