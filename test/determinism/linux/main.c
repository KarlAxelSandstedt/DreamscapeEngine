/*
==========================================================================
    Copyright (C) 2025, 2026 Axel Sandstedt 

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

#include <stdio.h>
#include <string.h>

#include "ds_base.h" 
#include "ds_init.h"
#include "ds_math.h"
#include "ds_platform.h"
#include "ds_graphics.h"
#include "ds_asset.h"
#include "ds_ui.h"
#include "ds_led.h"
#include "ds_job.h"

struct ds_DeterminismTest
{
    utf8            file;
    char *          file_cstr;
    char *          config_cstr;    /* file.cfg: the config --generate ran with */

    u32             generate;

    /* file data */
    u64             seed[4];    /* BE */

    ds_CPool(u64)   hash_pool;  /* BE */
};

static struct ds_DeterminismTest ds_DeterminismProcessArguments(struct arena *persistent, const utf8 *argument, const u32 argument_count)
{
    if (argument_count != 4)
    {
        fprintf(stderr, "Usage: DreamscapeTest config_path --generate|--load file, exiting.\n");
        exit(1);
    }
    
    const utf8 generate = Utf8Inline("--generate");
    const utf8 load = Utf8Inline("--load");

    struct ds_DeterminismTest test = 
    { 
        .file = argument[3],
        .file_cstr = CstrUtf8(persistent, argument[3]),
        .config_cstr = CstrUtf8(persistent, Utf8Format(persistent, "%k.cfg", &argument[3])),
        .generate = 0,
    };
    
    if (Utf8Equivalence(argument[2], generate))
    {
        test.generate = 1;
        test.seed[0] = g_config->seed[0];
        test.seed[1] = g_config->seed[1];
        test.seed[2] = g_config->seed[2];
        test.seed[3] = g_config->seed[3];
        ds_CPoolAlloc(NULL, test.hash_pool, 4096, GROWABLE);
    }
    else if (Utf8Equivalence(argument[2], load))
    {
        struct dsBuffer buf = FileDumpAtCwd(persistent, test.file_cstr);
        if (buf.size < 4*sizeof(u64) + sizeof(u32))
        {
            fprintf(stderr, "Bad determinism test file size, exiting.\n");
            exit(1);
        }
        
        struct ss ss = ss_Buffered(buf.data, buf.size);
        test.seed[0] = ss_ReadU64Be(&ss);
        test.seed[1] = ss_ReadU64Be(&ss);
        test.seed[2] = ss_ReadU64Be(&ss);
        test.seed[3] = ss_ReadU64Be(&ss);
        const u32 hash_count = ss_ReadU32Be(&ss);
        if (buf.size - 4*sizeof(u64) - sizeof(u32) != hash_count*sizeof(u64))
        {
            fprintf(stderr, "Bad determinism test file size, exiting.\n");
            exit(1);
        }

        ds_CPoolAlloc(persistent, test.hash_pool, hash_count, NOT_GROWABLE);
        for (u32 i = 0; i < hash_count; ++i)
        {
           ds_CPoolPush(test.hash_pool);
        }
        ss_ReadU64BeN(test.hash_pool.buf, &ss, hash_count);

        /* the RNG was seeded from the config in ds_Init */
        if (test.seed[0] != g_config->seed[0] || test.seed[1] != g_config->seed[1] || test.seed[2] != g_config->seed[2] || test.seed[3] != g_config->seed[3])
        {
            fprintf(stderr, "The config's seed differs from the test file's: load with the config --generate wrote (%s), exiting.\n", test.config_cstr);
            exit(1);
        }
    }
    else
    {
        fprintf(stderr, "Bad operation provided to executable, exiting.\n");
        exit(1);
    }

    return test;
}

static void ds_DeterminismGenerate(struct arena *persistent, struct ds_DeterminismTest *test)
{
    struct file file = { .handle = FILE_HANDLE_INVALID };
    const u32 truncate = 1;
    if (FS_SUCCESS != FileTryCreateAtCwd(persistent, &file, test->file_cstr, truncate))
    {
        fprintf(stderr, "Failed to create file, exiting.\n");
        exit(1);
    }

    const u64 bufsize = 4*sizeof(u64) + sizeof(u32) + (u64) test->hash_pool.count*sizeof(u64);
    u8 *buf = ArenaPush(persistent, bufsize);
    struct ss ss = ss_Buffered(buf, bufsize);
    ss_WriteU64Be(&ss, test->seed[0]);
    ss_WriteU64Be(&ss, test->seed[1]);
    ss_WriteU64Be(&ss, test->seed[2]);
    ss_WriteU64Be(&ss, test->seed[3]);
    ss_WriteU32Be(&ss, test->hash_pool.count);
    ss_WriteU64BeN(&ss, test->hash_pool.buf, test->hash_pool.count);

    const u64 bytes_written = FileWriteAppend(&file, buf, bufsize);
    ds_Assert(bytes_written == bufsize);

    FileClose(&file);

    ds_CPoolDealloc(test->hash_pool);

    if (!ds_ConfigTryWrite(persistent, g_config, test->config_cstr))
    {
        fprintf(stderr, "Failed to write %s, exiting.\n", test->config_cstr);
        exit(1);
    }
}

/*
 *  ./DreamscapeTest config_path --generate file    run the scene, write its per-frame hashes to file and the
 *                                                  config it ran with (seed, thread_count) to file.cfg
 *  ./DreamscapeTest config_path --load file        replay with config_path (file.cfg, possibly with another
 *                                                  thread_count) and compare every frame's hash
 */
int main(int argc, char *argv[])
{
    ds_Init((argc > 1) ? argv[1] : NULL, "log.txt");

	struct arena persistent = ArenaAlloc(NULL, 64*1024*1024);
	if (!persistent.stack_ptr)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to allocate the program's persistent arena");
		FatalCleanupAndExit();
	}

    const u32 argument_count = (u32) argc;
    utf8 *argument = ArenaPush(&persistent, sizeof(utf8)*argument_count);
    for (i32 i = 0; i < argc; ++i)
    {
        argument[i] = Utf8Cstr(&persistent, argv[i]);
    }
    struct ds_DeterminismTest test = ds_DeterminismProcessArguments(&persistent, argument, argument_count);

	struct led *editor = led_Alloc(g_config->thread_count, g_config->thread_framesize);

	const u64 renderer_framerate = 144;	
	r_Init(&persistent, NSEC_PER_SEC / renderer_framerate, 16*1024*1024, 1024, &editor->render_mesh_db);
	
    u32 success = 1;
    u64 frame = 0;
    while (editor->running)
    {
    	ProfFrameMark;
    
		ds_DeallocTaggedWindows();

        ds_JobSchedulerFrameClear();
    
		ds_ProcessEvents();

        led_Main(editor, editor->physics.ns_tick);
		led_UiMain(editor);
		r_EditorMain(editor);

        const u64 hash = ds_DynamicsOrientationHash(&editor->physics);
        if (test.generate)
        {
            const u32 index = ds_CPoolPush(test.hash_pool).index;
            test.hash_pool.buf[ index ] = hash;
        }
        else
        {
            fprintf(stderr, "checking index %lu of %u: (%lu, %lu)\n", frame, test.hash_pool.count, hash, ((frame >= test.hash_pool.count) ? U64_MAX : test.hash_pool.buf[ frame ]));
            if (frame >= test.hash_pool.count || hash != test.hash_pool.buf[ frame ])
            {
                success = 0;
            }
        }

        if (!success || editor->physics.solver_set_pool.buf[SOLVER_SET_ACTIVE].body_sim_pool.count == 0)
        {
            editor->running = 0;
        }

        frame += 1;
    }

    if (!test.generate)
    {
        (success)
            ? fprintf(stderr, "========================== SUCCESS ==========================\n")
            : fprintf(stderr, "========================== FAILURE ==========================\n");
    }
	
	led_Dealloc(editor);

    if (test.generate)
    {
        ds_DeterminismGenerate(&persistent, &test);
    }

	ArenaFree(&persistent);
	ds_Shutdown();

	return (success) ? 0 : 1;
}
