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

#ifndef __DS_INIT_H__
#define __DS_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_types.h"
#include "ds_allocator.h"
#include "ds_string.h"

/*
ds_init.h
=========
Engine configuration, startup and shutdown.

API
---
Description:
    ds_Init starts the engine from a struct ds_RuntimeConfig (the engine's runtime parameters: RNG seed, threads,
    memory, headless) and publishes it, resolved, as g_config;
    ds_Shutdown stops it in reverse order. The calls are the same on every platform. The config starts
    as the defaults below; the config file then sets the fields it has valid lines for (an unreadable,
    empty or missing file sets none). Programs take key=value arguments (ds_ArgvParse, g_argv):
    ./Executable [config=path] [log=path] [record=path | replay=path] [hash=path] [headless].
    Defaults: system rng_seed, 256 MiB persistent arena, thread_count 0 (logical cores - 2), 4 MiB frame and
    5 x 1 MiB scratch arenas per thread, headless 0.
    Rules for the program:
    1. ds_Init and ds_Shutdown run once per process, on the same thread. That thread becomes thread
       index 0 and scheduler worker 0 for the rest of the program.
    2. Each worker has a unique, fixed index in [0, g_config->thread_count), ds_ThreadSelfIndex(),
       which picks its slot in per-worker data (job deques, ds_Dynamics workers, ...).
    3. No ds_ThreadClone before ds_Init. Threads cloned after it get indices >=
       thread_count: they own no slot and must not use per-worker data.
    4. No engine call before ds_Init and none after ds_Shutdown.
    5. Headless (g_config->headless): graphics (SDL, windows, UI, GL) is not started, so no graphics,
       UI or renderer calls. Assets are available.

Usage:
    ds_Init(argc, argv);                        (no config=: defaults; no log=: no log file)
    (program systems sized by g_config, e.g. led_Alloc, ds_DynamicsAlloc, r_Init; main loop)
    ds_Shutdown();

    Web: the browser owns the main loop. main() hands it to emscripten_set_main_loop, which unwinds
    main()'s stack, and the loop callback calls ds_Shutdown when the program quits. Keep all state the
    loop uses in globals.

Config API
----------
Description:
    Reads and writes a struct ds_RuntimeConfig as text: one "key: value" line per field, keys are the field
    names, values are decimal unsigned integers (rng_seed: four, space separated); '#' starts a comment.
    Reading applies the file's keys onto the given config: missing keys keep their values, and a bad
    line (no ':', unknown key, bad value) is logged as a warning and ignored. Writing replaces the file.

Usage:
    struct ds_RuntimeConfig config = *g_config;
    ds_ConfigTryRead(tmp, &config, "config.txt");       (0: unreadable file, config unchanged)
    ds_ConfigTryWrite(tmp, &config, "config.txt");

Internals:
    Paths are relative to the working directory; an empty file counts as unreadable. ds_Init reads its
    config before the log exists and logs the problems once it is up. Values are only checked against
    their types (sizes and scratch count >= 1, headless <= 1): a persistent arena or thread arenas too
    small for the engine's own startup are not detected and crash inside ds_Init.
*/

struct ds_RuntimeConfig
{
    u64             rng_seed[4];            /* Xoshiro256Init seed of the thread RNG streams; all zero (invalid
                                               for xoshiro256**): RngSystem. Tests rely on the sequence; a
                                               simulation never reads it (ds_SimConfig has its own seed)  */
    u64             persistent_size;        /* engine-owned persistent arena                            */
    u32             thread_count;           /* scheduler workers including the main thread; 0: logical
                                               cores - 2. Clamped to [1, min(logical cores,
                                               DS_THREAD_COUNT_MAX)]; on the web the core count is
                                               meant to be the pthread pool size (unverified)          */
    u64             thread_framesize;       /* per-thread frame arena size (main thread and workers)    */
    u64             thread_scratchsize;     /* per-thread scratch arena size                            */
    u32             thread_scratch_count;   /* per-thread scratch arena count                           */
    u32             headless;               /* 1: don't start graphics (rule 5)                         */
};

/* The config ds_Init ran with, defaults resolved (thread_count >= 1, rng_seed nonzero). NULL outside ds_Init..ds_Shutdown. */
extern const struct ds_RuntimeConfig *g_config;

/* The command line's keys: ./Executable key=value ... */
enum ds_ArgvType
{
    ARGV_LOG,           /* the log file                                              */
    ARGV_CONFIG,        /* the config file                                              */
    ARGV_SIMULATION,    /* TODO a serialized simulation state + its config (allowed with record only; the recorder then
                           seeds with a SimulationLoad instead of a ConfigLoad): rejected until it exists */
    ARGV_RECORD,        /* the record file to write                                     */
    ARGV_REPLAY,        /* the record file to replay                                    */
    ARGV_HASH,          /* the hash file: written with record, compared with replay     */
    ARGV_HEADLESS,      /* a flag (no value): no graphics, as the config's headless      */
    ARGV_ORIGIN,        /* x,y,z: the determinism test's scene origin (parsed by its main) */
    ARGV_COUNT
};

/* The parsed command line: g_argv[key] is the key's value ("1" for a given flag), empty when not given. Valid after ds_ArgvParse. */
extern utf8 g_argv[ARGV_COUNT];

/*
 * Parse argv[1..argc), key=value or a bare flag, into g_argv; a value's bytes are argv's (null-terminated in
 * place). Fatal: a bare key that is not a flag, a flag with a value, an empty key or value, an unknown or repeated
 * key, hash without record or replay, replay with record, simulation.
 */
void            ds_ArgvParse(const int argc, char *argv[]);

/*
 * Parse the command line (ds_ArgvParse) and start the engine with the defaults overridden by the config= file
 * (none: defaults only) and headless. Logs to the log= file (none: no log file; the web logs to the console
 * only). Fatal: a bad command line, an unsupported CPU or a persistent arena that can't be allocated.
 */
void 			ds_Init(const int argc, char *argv[]);
/* Stop the engine in reverse order and free its memory. Web: may be called from the main loop callback. */
void 			ds_Shutdown(void);

/* Apply the keys in the file at path onto config. Returns 0 if the file can't be read (config unchanged). */
u32             ds_ConfigTryRead(struct arena *tmp, struct ds_RuntimeConfig *config, const char *path);
/* Write every field of config to path, replacing the file. Returns 0 on failure. */
u32             ds_ConfigTryWrite(struct arena *tmp, const struct ds_RuntimeConfig *config, const char *path);

#ifdef __cplusplus
}
#endif

#endif
