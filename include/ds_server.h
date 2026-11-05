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
ds_server.h
===========
Authoritative host of a set of simulations. 
//TODO can fill in more once we have more details settled 

*/

/*
TODO: server notes (copied from sim_cmd.c; update to the decided design: no buffering in simulations, the
server deserializes, routes by id, fuses, stamps on the simulation clock and records per simulation per frame)
=================================================================================================================

Every external source of input that affects the simulation state is generalized
behind the ds_SimCmd API. By serializing all such commands, in order, to a file
we gain the ability to fully replay any simulation. However, such APIs can quickly
become cumbersome to use, so we discuss the issues and try to land on something
reasonable.

Judging from the words "external source" we can imagine these commands, in 
serialized forms, being sent over networks, read from files, and so on. This seem
like things that should be fully abstracted away and be part of the generalized
command pipeline; every command arrives to the engine/server fully serialized:

          File =>                               serialized(cmd) ---+
                                                                   |
       Network => Payload(serialized(cmd)) ---> serialized(cmd) ---+---> cmd.ValidateAndExecute()
                                                                   |
    LocalInput =>                               serialized(cmd) ---+

The more complex question is how the external sources dispatch commands to the
command queue. As in the previous case, this is preferable fully general, as and
easy-to-use as possible. 

We begin by acknowledging that any external command's destination is a specific
simulation's command queue. This seems to be the correct abstraction level when
specifying a command's destination; Presuming that a server generates a unique
identifier (ds_SimId) per simulation, whenever we want to send a command(s) to a
given simulation, we simply have to send an identifier with our command set:

    Network:       Payload(sim_id, serialized(cmd[])) 
    File:                  sim_id, serialized(cmd[])
    LocalInput:            sim_id, serialized(cmd[])

    struct ds_Core, struct ds_Engin
    {
        ds_Pool simulation_pool;    // One simulation = one game world...
    
        (...)
    };


    ServerHandleFramePayloads(...)
    { 
        server.payload = PayloadEmpty()
        server.cmd_pool = CPoolEmpty();

        // POSSIBLE FUNCTION 1 ABSTRACTION: payload = PayloadTryDeserializeAndVerify(&core, mem, ss)

        ext_payload[count]);
        for (i in stream_count)
        {
            ext_payload[i] = PayloadEmpty()

            if (!PayloadTryDeserialize(ext_payload[i], ext_stream[i]) || PayloadHeaderVerify(&ext_payload))
            {
                continue;
            }

            success = 1; 
            while (s.NotEmpty())
            {
                if (!CmdTryDeserialize(&cmd, ext_payload) || !CmdFormVerify(&cmd))
                {
                    server.cmd_pool.pop(push_count);
                    ext_payload[count] = PayloadEmpty();
                    break;
                }

                server.cmd_pool.push(cmd);
                push_count += 1;
            }
        }

        // POSSIBLE FUNCTION 2 ABSTRACTION: Append, we just append one payload onto another
        
        for (external in payload-streams[count])
        {
            payload.Append(external_payload);
        }

        // we now have commands in order to be executed, and a fused payload ready to be timestamped and recorded.
    }

Issues with above (memory lifetimes)

Presume the engine recieves network payloads, and local payloads. 

If we treat the engine's loop iteration as immediatelly applying
any command payloads, we can make due with everything begin stored
on a engine frame arena, and all payload handling happens outside
the simulation. The simulations then execute these payloads 
immediately.

If we allow the engine to skip calls to ds_SimMain, we must store
all non-fused (or one we continuously fuse to, without knowing the
final size (complexity!)) payloads on simulation frame memory.

Clearly, the first choice is the more clean process, with very
predictable memory lifetimes (all payloads stays for a single engine 
frame, but it does enforce us to immediately process and execute
payloads.

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

enum ds_SimMode
{
    SIM_MODE_NONE,   // Run new simulation without recording
    SIM_MODE_RECORD, // Run new simulation while recording
    SIM_MODE_REPLAY, // Run old simulation from replay
    SIM_MODE_COUNT
};

/* A hosted simulation: the server's per-simulation state (clock conversion, mode, record, the frame's commands) and the simulation. */
struct ds_SimHost
{
    POOL_NODE;
    struct DLLNode          srv_sim;        /* ds_Server.sim_list */
    ds_SimId                id;             /* generational identifier: the pool slot */

    enum ds_SimMode         mode;
    f64                     speed;          /* scales wall-clock time */
    u64                     ns_remainder;   /* wall-clock time not yet accounted for (sub-microsecond) */
    u64                     ns_budget;      /* the simulation never advances more than this per tick: sum of the system periods */
    struct file             record_file;    /* RECORD: appended every frame with commands; REPLAY: read (TODO); else closed */
    struct SS               record_stream;       /* RECORD: the frame's entry staged (timestamp, frame payload) before it is appended */

    ds_CPool(ds_SimCmd)     cmd_pool;       /* the frame's commands, decoded at the door, executed by the simulation's tick */
    struct ds_SimCmdPayload frame_payload;  /* the frame's arrivals fused: the record's entry */

    struct ds_Sim           sim;
};
DEFINE_POOL(ds_SimHost);

struct ds_Server
{
    struct arena        frame;      /* frame memory: flushed every tick */

    ds_Pool(ds_SimHost) sim_pool;   
    struct DLL          sim_list;   /* the active simulations */
    ds_CPool(ds_SimCmdPayload) payload_pool;    /* the frame's arrived payloads, valid during the tick */
};

/* Allocate empty server. */
void    ds_ServerAlloc(struct ds_Server *srv, const u64 frame_size, const u32 initial_sim_count);
/* Deallocate server */
void    ds_ServerDealloc(struct ds_Server *srv);
/* Handle any incoming payloads after which every simulation is advanced by ns_delta of wall-clock time */
void    ds_ServerTick(struct ds_Server *srv, const u64 ns_delta);
/*
 * Host a new simulation (ds_SimAlloc on mem, its path copied there too) and return its id. RECORD creates (truncates)
 * the record file at path (relative to the cwd) and writes the seed entry, REPLAY opens it; path is ignored for NONE.
 * If the file fails, nothing is added (logged) and the null id is returned.
 */
ds_SimId    ds_SimAdd(struct ds_Server *srv, struct arena *mem, const struct ds_SimConfig *config, const enum ds_SimMode mode, const char *path, struct c_ShapeDB *cshape_db, ds_SDB(ds_BodyPrefab) *prefab_db, const u32 worker_count, const u64 worker_frame_size);
/* Remove the hosted simulation with the given id (ds_SimDealloc, the record closed); an unknown id is logged. */
void        ds_SimRemove(struct ds_Server *srv, const ds_SimId id);
/* Return the slot of the hosted simulation with the given id, or { NULL, U32_MAX } if there is none. */
struct slot ds_SimLookup(const struct ds_Server *srv, const ds_SimId id);

#ifdef __cplusplus
}
#endif

#endif
