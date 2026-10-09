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

/*
ds_init.h
=========
Engine configuration, startup and shutdown.

API
---
Description:
    ds_Init starts the engine from a struct ds_Config and publishes it, resolved, as g_config;
    ds_Shutdown stops it in reverse order. The calls are the same on every platform. The config starts
    as the defaults below; the config file then sets the fields it has valid lines for (an unreadable,
    empty or missing file sets none). Programs take the config path as their first argument:
    ./Executable [config_path].
    Defaults: system seed, 256 MiB persistent arena, thread_count 0 (logical cores - 2), 4 MiB frame and
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
    ds_Init(argv[1], "log.txt");                (config NULL: defaults; log NULL: no log file)
    (program systems sized by g_config, e.g. led_Alloc, ds_DynamicsAlloc, r_Init; main loop)
    ds_Shutdown();

    Web: the browser owns the main loop. main() hands it to emscripten_set_main_loop, which unwinds
    main()'s stack, and the loop callback calls ds_Shutdown when the program quits. Keep all state the
    loop uses in globals.

Config API
----------
Description:
    Reads and writes a struct ds_Config as text: one "key: value" line per field, keys are the field
    names, values are decimal unsigned integers (seed: four, space separated); '#' starts a comment.
    Reading applies the file's keys onto the given config: missing keys keep their values, and a bad
    line (no ':', unknown key, bad value) is logged as a warning and ignored. Writing replaces the file.

Usage:
    struct ds_Config config = *g_config;
    ds_ConfigTryRead(tmp, &config, "config.txt");       (0: unreadable file, config unchanged)
    ds_ConfigTryWrite(tmp, &config, "config.txt");

Internals:
    Paths are relative to the working directory; an empty file counts as unreadable. ds_Init reads its
    config before the log exists and logs the problems once it is up. Values are only checked against
    their types (sizes and scratch count >= 1, headless <= 1): a persistent arena or thread arenas too
    small for the engine's own startup are not detected and crash inside ds_Init.
*/

struct ds_Config
{
    u64             seed[4];                /* Xoshiro256Init seed; all zero (invalid for xoshiro256**):
                                               RngSystem. Workers derive their RNG streams from it.    */
    u64             persistent_size;        /* engine-owned persistent arena                            */
    u32             thread_count;           /* scheduler workers including the main thread; 0: logical
                                               cores - 2. Clamped to [1, logical cores]; on the web
                                               this is meant to be the pthread pool size
                                               (-sPTHREAD_POOL_SIZE, unverified)                        */
    u64             thread_framesize;       /* per-thread frame arena size (main thread and workers)    */
    u64             thread_scratchsize;     /* per-thread scratch arena size                            */
    u32             thread_scratch_count;   /* per-thread scratch arena count                           */
    u32             headless;               /* 1: don't start graphics (rule 5)                         */
};

/* The config ds_Init ran with, defaults resolved (thread_count >= 1, seed nonzero). NULL outside ds_Init..ds_Shutdown. */
extern const struct ds_Config *g_config;

/*
 * Start the engine with the defaults overridden by the file at config_path (NULL: defaults only). Logs to
 * log_path (NULL: no log file; the web logs to the console only). Fatal: an unsupported CPU or a
 * persistent arena that can't be allocated.
 */
void 			ds_Init(const char *config_path, const char *log_path);
/* Stop the engine in reverse order and free its memory. Web: may be called from the main loop callback. */
void 			ds_Shutdown(void);

/* Apply the keys in the file at path onto config. Returns 0 if the file can't be read (config unchanged). */
u32             ds_ConfigTryRead(struct arena *tmp, struct ds_Config *config, const char *path);
/* Write every field of config to path, replacing the file. Returns 0 on failure. */
u32             ds_ConfigTryWrite(struct arena *tmp, const struct ds_Config *config, const char *path);

#ifdef __cplusplus
}
#endif

#endif
