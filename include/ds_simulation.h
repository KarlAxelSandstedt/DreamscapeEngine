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

enum ds_SimMode
{
    SIM_MODE_NONE,   // Run new simulation without recording
    SIM_MODE_RECORD, // Run new simulation while recording
    SIM_MODE_REPLAY, // Run old simulation from replay
    SIM_MODE_COUNT
};


/* Ordered by priority; on tie-breakers, external precedes dynamics precedes game ... */
enum ds_SimSystem
{
    SIM_SYSTEM_EXTERNAL,    /* command batches with external sources as origin (users, ...) */
    SIM_SYSTEM_DYNAMICS,
    SIM_SYSTEM_GAME,
    SIM_SYSTEM_COUNT
};

#define DS_SIM_KEY_SYSTEM_BITS  4
#define DS_SIM_TICK_US_MAX      1000000     /* slowest system: 1 Hz */
#define DS_SIM_SPEED_MAX        1000.0

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

    f64 speed;      /* simulation speed (scales wall-clock) */

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
void                    ds_SimConfigSerialize(struct ss *ss, const struct ds_SimConfig *config);
/* Returns 1 on success. On failure return 0 (bad stream or bad config) and reset the stream position. */
u32                     ds_SimConfigTryDeserialize(struct ss *ss, struct ds_SimConfig *config);


/*
ds_SimCmd
=========
TODO
*/
struct ds_SimCmd
{
    //TODO
    u32 tmp;
};
DEFINE_CPOOL_STRUCT(ds_SimCmd);


/*
ds_Sim
======
For full design see design doc. The crucial point here is that every state-change imposed
on the simulation by an external source, goes through the simulations external command
buffer, enforcing a deterministic and replayable simulation.
*/

/* Internal scheduling key of system ticks and external commands.  */
typedef u64 ds_SimKey;

struct ds_Sim
{
    enum ds_SimMode  mode;
    //FILE                    record;           /* TODO Null if mode == NONE. */

    f64                     speed;              /* Simulation speed (scales wall-clock)     */
    u64                     us_time;            /* Simulation time                          */
    u64                     us_delta;           /* Simulation time delta                    */
    u64                     ns_remainder;       /* Unaccounted time from previous frame     */
    u64                     ns_budget;          /* Simulation time-budget, us_time is never
                                                   incremented by more than the budget
                                                    ns_budget = sum_i system_i.us_budget    */


    ds_CPool(ds_SimCmd)     external_cmd_buf;   /* external command batch, not yet stamped  */

    struct ds_Dynamics      dynamics;
    //struct ds_Game          game;             /* TODO: ds_Game undefined */

    /* Scheduler: linear search for minimum on timestamped and system-tagged keys.  */
    u64                     us_system_tick[SIM_SYSTEM_COUNT];
    ds_SimKey               sys_schedule[SIM_SYSTEM_COUNT];
};

/*
 * Allocate a new simulation from config (ds_SimConfigCheck asserted) at time 0: the systems' shells are pushed on mem
 * (ds_DynamicsShellMemoryRequirement for the dynamics), the seed batch is the first external command and the first
 * system ticks are one period in. The databases are externally owned and must outlive sim; worker_count and
 * worker_frame_size are passed on to ds_DynamicsAllocConfig. record != 0 records the simulation.
 */
void    ds_SimAlloc(struct arena *mem, struct ds_Sim *sim, const struct ds_SimConfig *config, const u32 record, struct c_ShapeDB *cshape_db, ds_SDB(ds_BodyPrefab) *prefab_db, const u32 worker_count, const u64 worker_frame_size);
//TODO
//void    ds_SimAllocReplay(struct ds_Sim *sim, struct ds_SimReplay *replay);
/* Free the simulation's heap resources. */
void    ds_SimDealloc(struct ds_Sim *sim);
/* Flush the external command buffer and systems, and reset schedule to time=0. Config and capacities stay. */
void    ds_SimFlush(struct ds_Sim *sim);
/* 
 * Advance the simulation by ns_delta of wall-clock time, scaled by speed and capped by the simulation's time budget.
 * Any pending external commands are dispatched, and executed before any pending system ticks.
 */
void    ds_SimTick(struct ds_Sim *sim, const u64 ns_delta);

//TODO Serialize API for sim 



#ifdef __cplusplus
}
#endif

#endif
