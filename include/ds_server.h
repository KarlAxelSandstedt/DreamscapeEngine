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

#ifndef __DS_SERVER_H__
#define __DS_SERVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_types.h"
#include "ds_allocator.h"
#include "ds_list.h"
#include "ds_platform.h"
#include "ds_simulation.h"

/*
ds_server
=========
Authoritative host of a set of simulations.  For design and reasoning, see notes/replays.txt
*/
struct ds_SimHost;
DEFINE_POOL(ds_SimHost);

struct ds_Server
{
    struct arena        frame;      /* flushed every tick */

    ds_Pool(ds_SimHost) sim_pool;   
    struct DLL          sim_list;   /* the hosted simulations */
    ds_CPool(ds_SimCmdPayload) payload_pool;    /* the frame's arrivals */
};

/* Allocate an empty server. */
void    ds_ServerAlloc(struct ds_Server *srv, const u64 frame_size, const u32 initial_sim_count);
/* Deallocate server resources. */
void    ds_ServerDealloc(struct ds_Server *srv);
/* Route frame command payloads, and advance simulations. */
void    ds_ServerTick(struct ds_Server *srv, const u64 ns_delta);


/*
ds_SimHost
==========
Server simulation state.
*/

enum ds_SimMode
{
    SIM_MODE_NONE,   /* Run new simulation without recording    */
    SIM_MODE_RECORD, /* Run new simulation while recording      */
    SIM_MODE_REPLAY, /* Run old simulation from replay          */
    SIM_MODE_COUNT
};

struct ds_SimHost
{
    POOL_NODE;
    struct DLLNode          srv_sim;        /* ds_Server.sim_list node          */
    ds_SimId                id;             /* generational identifier          */

    struct arena            mem;            /* host-lifetime data: the file paths */
    enum ds_SimMode         record_mode;
    f64                     speed;          /* wall-clock time scale            */
    u64                     ns_remainder;   /* sub-microsecond remainder        */
    u64                     ns_budget;      /* simulation time cap per tick     */

    struct file             record_file;    /* RECORD: appended; REPLAY: mapped */
    struct SS               record_stream;  /* RECORD: the frame's entry; 
                                               REPLAY: the mapped record        */
    u64                     us_previous_record; /* the last entry's time 
                                                   (entries store deltas)       */
    u32                     replay_ended;   

    u32                     hash;           /* hash simulation on system ticks  */
    struct file             hash_file;      /* RECORD: appended; REPLAY: mapped */
    struct SS               hash_stream;    /* RECORD: the frame's hashes; 
                                               REPLAY: the mapped file          */
    ds_CPool(u64)           hash_pool;      /* frame's hashes */
    u64                     hash_count;     /* hashes written or compared since
                                               add/reset                        */
    u32                     hash_success;   /* REPLAY: all hashes matched       */

    ds_CPool(ds_SimCmd)     cmd_pool;       /* the frame's commands             */
    struct ds_SimCmdPayload frame_payload;  /* the frame's arrivals fused       */

    struct ds_Sim           sim;
};

/*
 * Host a simulation and return its id, or the null id if a file fails. 
 *
 * If record_mode == RECORD, host writes record_path and optional hash_path.
 * If record_mode == REPLAY, host reads record_path and optional hash_path.
 */
ds_SimId    ds_SimHostAdd(struct ds_Server *srv, const struct ds_SimConfig *config, const enum ds_SimMode record_mode, const char *record_path, const char *hash_path, const struct ds_DynamicsExternal *external);
/* Remove host and release its resources. */
void        ds_SimHostRemove(struct ds_Server *srv, const ds_SimId id);
/* Return the host's slot, or { NULL, U32_MAX }. */
struct slot ds_SimHostLookup(const struct ds_Server *srv, const ds_SimId id);
/* Reset the hosted simulation to time 0, truncating or replaying any provided records/hashes. */
void        ds_SimHostReset(struct ds_Server *srv, const ds_SimId id);

#ifdef __cplusplus
}
#endif

#endif
