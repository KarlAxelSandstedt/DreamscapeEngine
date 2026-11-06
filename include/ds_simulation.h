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
ds_simulation.h
===============
The deterministic, replayable simulation: a timeline of system ticks and external command
batches ordered by Simulation Clock Time (SCT), recorded to and replayed from a command file.
Design: notes/replays.txt.
*/

#ifndef __DS_SIMULATION_H__
#define __DS_SIMULATION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_types.h"
#include "ds_allocator.h"
#include "ds_serialize.h"
#include "ds_dynamics.h"

struct ds_Sim;


/* Ordered by priority; on tie-breakers, dynamics precedes game ... */
enum ds_SimSystem
{
    SIM_SYSTEM_DYNAMICS,
    SIM_SYSTEM_GAME,
    SIM_SYSTEM_COUNT
};

#define DS_SIM_KEY_SYSTEM_BITS  4
#define DS_SIM_TICK_US_MAX      1000000     /* slowest system: 1 Hz */

/*
ds_SimConfig
============
ds_SimConfig stores all necessary data in order to set up a simulation with a proper,
initial state.
*/
struct ds_SimConfig
{
    u64 seed[4];    /* simulation seed */
    
    u64 us_system_tick[SIM_SYSTEM_COUNT];

    struct ds_DynamicsConfig dynamics;
};

/* Return a default config. */
struct ds_SimConfig     ds_SimConfigDefault(void);
/* Set config's seed to seed; the all-zero seed is invalid and in that case, the default seed is set instead. */
void                    ds_SimConfigSeed(struct ds_SimConfig *config, const u64 seed[4]);
/* Set config's seed from the OS's random source. */
void                    ds_SimConfigGenerateSeed(struct ds_SimConfig *config);
/* Return the required size when serializing the config. */
u64                     ds_SimConfigSerializeSize(const struct ds_SimConfig *config);
/* Serialize the config. WARNING: Assumes config fits in the stream. */
void                    ds_SimConfigSerialize(struct SS *ss, const struct ds_SimConfig *config);
/* Returns 1 on success. On failure return 0 (bad stream or bad config) and reset the stream position. */
u32                     ds_SimConfigTryDeserialize(struct SS *ss, struct ds_SimConfig *config);


/*
ds_SimCmd
=========
TODO (we document later)
*/
enum ds_SimCmdType
{
    SIM_CMD_CONFIG_LOAD,    /* Payload: serialized ds_SimConfig */
    SIM_CMD_BODY_ADD,       /* a body from a body prefab (by id) at a world transform, named by its entity */
    SIM_CMD_SHAPE_ADD,      /* a shape from a shape prefab (by id) on the entity's body, at a local transform */
    SIM_CMD_JOINT_ADD,      /* a distance joint between two entities' bodies */
    SIM_CMD_MOUSE_JOINT_ADD,        /* a mouse joint (joint entity) grabbing an entity's body at a world point */
    SIM_CMD_MOUSE_JOINT_TARGET_SET, /* move a mouse joint's target */
    SIM_CMD_JOINT_REMOVE,           /* remove a joint by its joint entity */
    SIM_CMD_COUNT
};

struct ds_SimCmd
{
    enum ds_SimCmdType  type;
    union
    {
        struct ds_LoadConfigCmd
        {
            const struct ds_SimConfig *config; 
        } load_config;

        struct ds_BodyAddCmd
        {
            utf8                prefab;     /* id in the body prefab database */
            ds_TransformWorld   t_world;
            u32                 entity;
        } body_add;

        struct ds_ShapeAddCmd
        {
            utf8                prefab;     /* id in the shape prefab database */
            ds_Transform        t_local;
            u32                 entity;
        } shape_add;

        struct ds_JointAddCmd
        {
            u32                 collide_connected;  /* a ds_DistanceJointPrefab's fields: rebuilt through its constructors */
            f32                 target;
            intv                range;
            u32                 enable_spring;
            f32                 hertz;
            f32                 damping_ratio;
            struct ds_BreakageOption breakage;
            u32                 entity[2];
            ds_Transform        local_frame[2];
        } joint_add;

        struct ds_MouseJointAddCmd
        {
            u32                 joint_entity;
            u32                 entity;         /* the grabbed body */
            v3f64               target;         /* the grabbed point (world), the first target */
            f32                 hertz;          /* ds_MouseJointPrefabFromBody's parameters */
            f32                 damping_ratio;
            f32                 force_scale;
            f32                 torque_scale;
        } mouse_joint_add;

        struct ds_MouseJointTargetSetCmd
        {
            u32                 joint_entity;
            v3f64               target;
        } mouse_joint_target_set;

        struct ds_JointRemoveCmd
        {
            u32                 joint_entity;
        } joint_remove;
    };
};
DEFINE_CPOOL_STRUCT(ds_SimCmd);

/* Returns 1 on success (the command's data is pushed on mem). On failure return 0 (bad stream or bad command, or no room: logged) and reset the stream position. */
u32     ds_SimCmdTryDeserialize(struct arena *mem, struct SS *ss, struct ds_SimCmd *cmd);
/* Return 1 if cmd is well-formed (Is it actually a valid command in form), 0 otherwise. */
u32     ds_SimCmdFormVerify(const struct ds_SimCmd *cmd);
/*
 * Verify cmd against sim's state (ds_SimCmdDataVerify) and execute it. Returns 1 on success and 
 * 0 if cmd is rejected
 */
u32     ds_SimCmdTryExecute(struct ds_Sim *sim, const struct ds_SimCmd *cmd);


/*
ds_SimId
========
Unique simulation identifier used by external sources to identify destination simulation.
*/
DS_DEFINE_ID(ds_SimId, DS_ID_GENERATION_SHIFT, DS_ID_GENERATION_MAX)
#define DS_SIM_ID_TEMPORARY     ds_SimIdInit(0, 0)

/*
ds_SimCmdPayload
================
External set of serialized ds_SimCmd with a destination simulation
*/

struct ds_SimCmdPayload
{
    //TODO not finalized, but captures the essence
    ds_SimId    dst;
    u32         cmd_count;
    struct SS   stream;     /* the commands bit-packed in order: bit_index bits used, bit_count the capacity */
};
DEFINE_CPOOL_STRUCT(ds_SimCmdPayload);

/* Return without allocating an empty payload for dst */
struct ds_SimCmdPayload ds_SimCmdPayloadEmpty(const ds_SimId dst);
/* Return an empty payload for dst with a size byte buffer in mem or on the heap (mem == NULL). */
struct ds_SimCmdPayload ds_SimCmdPayloadAlloc(struct arena *mem, const ds_SimId dst, const u64 size);
/* Free a heap payload's buffer (arena memory stays with the arena); the payload becomes empty. */
void                    ds_SimCmdPayloadDealloc(struct ds_SimCmdPayload *payload);
/* Return a payload for dst holding the pool's commands serialized in order, sized exactly, in mem or on the heap (mem == NULL); empty if there is no room (logged). */
struct ds_SimCmdPayload ds_SimCmdPayloadPushAll(struct arena *mem, const ds_SimId dst, const ds_CPool(ds_SimCmd) *pool);
/* Return the required size when serializing the payload. */
u64                     ds_SimCmdPayloadSerializeSize(const struct ds_SimCmdPayload *payload);
/* Serialize the payload. WARNING: Assumes payload fits in the stream. */
void                    ds_SimCmdPayloadSerialize(struct SS *ss, const struct ds_SimCmdPayload *payload);
/*
 * Read the stream's payload as a view of the stream's buffer (no copy; valid while the buffer is). Returns 1 on
 * success and 0 on failure (bad stream): the stream position is as before and the payload is empty.
 */
u32                     ds_SimCmdPayloadTryDeserializeAndAlias(struct SS *ss, struct ds_SimCmdPayload *payload);
/* Append src's commands to payload (a null dst takes src's). Returns 1 on success and 0 on another dst or no room: payload as before. */
u32                     ds_SimCmdPayloadAppend(struct ds_SimCmdPayload *payload, const struct ds_SimCmdPayload *src);




/*
ds_Sim
======
For full design see design doc. The crucial point here is that every state-change imposed
on the simulation by an external source, goes through the simulations exterkjnal command
buffer, enforcing a deterministic and replayable simulation.
*/

/* Internal scheduling key. */
typedef u64 ds_SimKey;

struct ds_Sim
{
    struct ds_SimConfig     config_seed;        /* the alloc-time config: the seed batch of ds_SimAlloc and ds_SimFlush */
    u32                     initialized;        /* a ConfigLoad was applied since alloc/flush: the seed; later ones must keep the alloc-time values */

    u64                     us_time;            /* Simulation time                          */
    u64                     us_delta;           /* Simulation time delta                    */

    struct ds_Dynamics      dynamics;
    //struct ds_Game          game;             /* TODO: ds_Game undefined */
    ds_SDB(ds_BodyPrefab)  *body_prefab_db;     /* externally owned */
    ds_SDB(ds_ShapePrefab) *shape_prefab_db;    /* externally owned */
    struct ds_HashMap       entity_map;         /* TODO(Remove/Refactor) entity -> body pool index, for the commands and the host */
    struct ds_HashMap       joint_entity_map;   /* TODO(Remove/Refactor) joint entity -> joint pool index */

    /* Scheduler: linear search for minimum on timestamped and system-tagged keys.  */
    u64                     us_system_tick[SIM_SYSTEM_COUNT];
    ds_SimKey               sys_schedule[SIM_SYSTEM_COUNT];
};

/*
 * Allocate a new simulation from config (ds_SimConfigCheck asserted) at time 0: the seed batch is the first external
 * command and the first system ticks are one period in. The databases are externally owned and must outlive sim.
 */
void    ds_SimAlloc(struct ds_Sim *sim, const struct ds_SimConfig *config, const struct ds_DynamicsExternal *external, ds_SDB(ds_BodyPrefab) *prefab_db, ds_SDB(ds_ShapePrefab) *shape_prefab_db);
//TODO
//void    ds_SimAllocReplay(struct ds_Sim *sim, struct ds_SimReplay *replay);
/* Free the simulation's heap resources. */
void    ds_SimDealloc(struct ds_Sim *sim);
/* Flush systems, reset schedule to time=0 and reseed the simulation. Config and capacities stay. */
void    ds_SimFlush(struct ds_Sim *sim);
/*
 * Advance the simulation by us_delta of simulation time: the frame's external commands cmds (may be empty) execute
 * first, at the current time, then every system tick that is due. srv_frame is the server's frame arena, the ticks' scratch memory: each system tick
 * runs between a push and a pop of a record on it. Optional(hash_pool): If non-null, hashes are generated
 * per-subsystem tick (of the tick's time, system and ds_SimHash) and pushed onto it.
 */
void    ds_SimTick(struct arena *srv_frame, struct ds_Sim *sim, ds_CPool(u64) *hash_pool, const ds_CPool(ds_SimCmd) *cmds, const u64 us_delta);
/* Return the hash of the simulation's state (every system's state hash). */
u64     ds_SimHash(const struct ds_Sim *sim);
/* TODO(Remove/Refactor) Return the body added for entity (SIM_CMD_BODY_ADD), or the null id. */
ds_BodyId ds_SimBodyByEntity(const struct ds_Sim *sim, const u32 entity);
/* TODO(Remove/Refactor) Return the joint added for the joint entity (SIM_CMD_MOUSE_JOINT_ADD), or the null id. */
ds_JointId ds_SimJointByEntity(const struct ds_Sim *sim, const u32 joint_entity);

//TODO Serialize API for sim 



#ifdef __cplusplus
}
#endif

#endif
