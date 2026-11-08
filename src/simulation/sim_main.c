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
#define DS_SIM_CONFIG_FIXED_BITS    (32 + 4*64 + SIM_SYSTEM_COUNT*64)

u64 ds_SimConfigSerializeBitPackedSize(const struct ds_SimConfig *config)
{
    return DS_SIM_CONFIG_FIXED_BITS + ds_DynamicsConfigSerializeBitPackedSize(&config->dynamics);
}

void ds_SimConfigSerializeBitPacked(struct SS *ss, const struct ds_SimConfig *config)
{
    ds_Assert(ds_SimConfigSerializeBitPackedSize(config) <= SSBitsLeft(ss));

    SSWriteU64Partial(ss, DS_SIM_CONFIG_VERSION, 32);
    for (u32 i = 0; i < 4; ++i)
    {
        SSWriteU64Partial(ss, config->seed[i], 64);
    }

    for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
    {
        SSWriteU64Partial(ss, config->us_system_tick[i], 64);
    }

    ds_DynamicsConfigSerializeBitPacked(ss, &config->dynamics);
}

u32 ds_SimConfigTryDeserializeBitPacked(struct SS *ss, struct ds_SimConfig *config)
{
    struct SS ss_mut = *ss;
    *config = (struct ds_SimConfig) { 0 };
    if (SSBitsLeft(&ss_mut) < DS_SIM_CONFIG_FIXED_BITS || SSReadU64Partial(&ss_mut, 32) != DS_SIM_CONFIG_VERSION)
    {
        return 0;
    }

    for (u32 i = 0; i < 4; ++i)
    {
        config->seed[i] = SSReadU64Partial(&ss_mut, 64);
    }

    for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
    {
        config->us_system_tick[i] = SSReadU64Partial(&ss_mut, 64);
    }

    if (!ds_DynamicsConfigTryDeserializeBitPacked(&ss_mut, &config->dynamics) || !ds_SimConfigCheck(config))
    {
        *config = (struct ds_SimConfig) { 0 };
        return 0;
    }

    *ss = ss_mut;
    return 1;
}

/**************** ds_Sim ****************/

static void ds_SimScheduleInit(struct ds_Sim *sim)
{
    sim->us_time = 0;

    for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
    {
        sim->sys_schedule[i] = ds_SimKeyInit(sim->us_system_tick[i], (enum ds_SimSystem) i);
    }
}

static void ds_SimSeed(struct ds_Sim *sim, const struct ds_SimConfig *config)
{
    struct arena *tmp = ArenaPushScratch();

    const struct ds_SimCmd seed = { .type = SIM_CMD_CONFIG_LOAD, .load_config.config = config };
    ds_CPool(ds_SimCmd) cmds;
    ds_CPoolAlloc(tmp, cmds, 1, NOT_GROWABLE);
    ds_CPoolPushValue(cmds, seed);
    ds_SimTick(tmp, sim, NULL, &cmds, 0);

    ArenaPopScratch();
}

void ds_SimAlloc(struct ds_Sim *sim, const struct ds_SimConfig *config, const struct ds_DynamicsExternal *external)
{
    ds_Assert(ds_SimConfigCheck(config));
    ds_Assert(external && external->cshape_db && external->body_prefab_db && external->shape_prefab_db);

    *sim = (struct ds_Sim) { 0 };
    sim->config_seed = *config;
    sim->entity_map = ds_HashMapAlloc(NULL, 1024, 1024, GROWABLE);
    ds_CPoolAlloc(NULL, sim->joint_entity_pool, 8, GROWABLE);
    for (u32 i = 0; i < SIM_SYSTEM_COUNT; ++i)
    {
        sim->us_system_tick[i] = config->us_system_tick[i];
    }

    sim->dynamics = ds_DynamicsAlloc(&config->dynamics, external, 1000 * config->us_system_tick[SIM_SYSTEM_DYNAMICS]);

    ds_SimScheduleInit(sim);
    ds_SimSeed(sim, &sim->config_seed);
}

void ds_SimDealloc(struct ds_Sim *sim)
{
    ds_DynamicsDealloc(&sim->dynamics);
    ds_HashMapDealloc(&sim->entity_map);
    ds_CPoolDealloc(sim->joint_entity_pool);
}

/* TODO(Remove/Refactor) */
ds_BodyId ds_SimBodyByEntity(const struct ds_Sim *sim, const u32 entity)
{
    for (u32 i = ds_HashMapFirst(&sim->entity_map, entity); i != HASH_NULL; i = ds_HashMapNext(&sim->entity_map, i))
    {
        const struct ds_Body *body = sim->dynamics.body_pool.buf + i;
        if (ds_PoolSlotAllocated(body) && body->entity == entity)
        {
            return body->id;
        }
    }

    return ds_BodyIdNull();
}

/* TODO(Remove/Refactor) */
ds_JointId ds_SimJointByEntity(const struct ds_Sim *sim, const u32 joint_entity)
{
    for (u32 i = 0; i < sim->joint_entity_pool.count; ++i)
    {
        const u64 entry = sim->joint_entity_pool.buf[i];
        if ((u32) (entry >> 32) == joint_entity)
        {
            const struct ds_Joint *joint = sim->dynamics.joint_pool.buf + (u32) entry;
            return (ds_PoolSlotAllocated(joint)) ? joint->id : ds_JointIdNull();
        }
    }

    return ds_JointIdNull();
}

void ds_SimFlush(struct ds_Sim *sim)
{
    ds_DynamicsFlush(&sim->dynamics);
    ds_HashMapFlush(&sim->entity_map);
    ds_CPoolFlush(sim->joint_entity_pool);
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

void ds_SimTick(struct arena *tmp, struct ds_Sim *sim, ds_CPool(u64) *hash_pool, const ds_CPool(ds_SimCmd) *cmds, const u64 us_delta)
{
    ds_DynamicsConfigPush(&sim->dynamics); 

    ds_SimExecuteCmdBatch(sim, cmds);

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

        ArenaPushRecord(tmp);
        switch (sys)
        {
            case SIM_SYSTEM_DYNAMICS:
            {
                ds_DynamicsTick(tmp, &sim->dynamics, 1000 * sim->us_system_tick[SIM_SYSTEM_DYNAMICS]);
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
        ArenaPopRecord(tmp);

        if (hash_pool)
        {
            const u64 words[3] = { us, (u64) sys, ds_SimHash(sim) };
            ds_CPoolPushValue(*hash_pool, XXH3_64bits(words, sizeof(words)));
        }
    }

    ds_DynamicsConfigPop(&sim->dynamics);
}
