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
#include <inttypes.h>

#include "ds_base.h" 
#include "ds_init.h"
#include "ds_math.h"
#include "ds_platform.h"
#include "ds_graphics.h"
#include "ds_asset.h"
#include "ds_ui.h"
#include "ds_led.h"
#include "ds_job.h"

/*
 * The test beyond ds_Init's arguments: record= hash= generates the reference (the scene as commands, one hash per
 * system tick), replay= hash= with another config checks it; origin=x,y,z (generate only) builds the scene there.
 */
struct ds_DeterminismTest
{
    u32     generate;
    v3f64   origin;
};

/* 1 if str is a decimal number ([+-]d[.d][(e|E)[+-]d], d: digits) with a finite value, written to *val */
static u32 ds_DeterminismF64Parse(f64 *val, struct arena *tmp, const utf8 str)
{
    u32 i = 0;
    u32 digits = 0;
    if (i < str.len && (str.buf[i] == '+' || str.buf[i] == '-'))
    {
        i += 1;
    }
    for (; i < str.len && '0' <= str.buf[i] && str.buf[i] <= '9'; ++i)
    {
        digits += 1;
    }
    if (i < str.len && str.buf[i] == '.')
    {
        for (i += 1; i < str.len && '0' <= str.buf[i] && str.buf[i] <= '9'; ++i)
        {
            digits += 1;
        }
    }
    if (digits && i < str.len && (str.buf[i] == 'e' || str.buf[i] == 'E'))
    {
        i += 1;
        if (i < str.len && (str.buf[i] == '+' || str.buf[i] == '-'))
        {
            i += 1;
        }
        const u32 exponent_start = i;
        for (; i < str.len && '0' <= str.buf[i] && str.buf[i] <= '9'; ++i);
        if (i == exponent_start)
        {
            return 0;
        }
    }
    if (!digits || i != str.len)
    {
        return 0;
    }

    *val = F64Utf8(tmp, str);
    return (*val - *val == 0.0);
}

static void ds_DeterminismUsageAndExit(const char *problem)
{
    fprintf(stderr, "%s\nUsage: DreamscapeTest config=path record=file hash=file [origin=x,y,z] | config=path replay=file hash=file, exiting.\n", problem);
    exit(1);
}

static struct ds_DeterminismTest ds_DeterminismProcessArguments(struct arena *persistent)
{
    struct ds_DeterminismTest test = { .generate = g_argv[ARGV_RECORD].size != 0, .origin = V3F64(0.0, 0.0, 0.0) };
    if (!g_argv[ARGV_HASH].size || (g_argv[ARGV_RECORD].size == 0) == (g_argv[ARGV_REPLAY].size == 0))
    {
        ds_DeterminismUsageAndExit("The test needs record= or replay=, and hash=.");
    }

    if (g_argv[ARGV_ORIGIN].size)
    {
        if (!test.generate)
        {
            ds_DeterminismUsageAndExit("origin= is for record= only: a replay builds the scene the record holds.");
        }

        /* three comma separated numbers */
        const utf8 str = g_argv[ARGV_ORIGIN];
        f64 *component[3] = { &test.origin.x, &test.origin.y, &test.origin.z };
        u32 start = 0;
        for (u32 c = 0; c < 3; ++c)
        {
            u32 end = start;
            while (end < str.size && str.buf[end] != ',')
            {
                end += 1;
            }
            const utf8 number = { .buf = str.buf + start, .size = end - start, .len = end - start };
            if ((c < 2) != (end < str.size) || !ds_DeterminismF64Parse(component[c], persistent, number))
            {
                ds_DeterminismUsageAndExit("Bad origin: expected origin=x,y,z (finite decimal numbers).");
            }
            start = end + 1;
        }
    }

    return test;
}

/*
 * 1 while the run goes on. Generate: until every body sleeps. Replay: until the hash file is matched whole
 * (success), or the comparison stopped short of that (the file ended early, or every body sleeps first: failure);
 * a differing hash is fatal inside the server.
 */
static u32 ds_DeterminismRunning(const struct ds_DeterminismTest *test, const struct led *editor, u32 *success)
{
    const u32 all_asleep = editor->physics->solver_set_pool.buf[SOLVER_SET_ACTIVE].body_sim_pool.count == 0;
    if (test->generate)
    {
        return !all_asleep;
    }

    const struct ds_SimHost *host = ds_SimHostLookup(&editor->srv, editor->sim).address;
    if (host->hash_success)
    {
        return 0;
    }

    if (!host->hash || all_asleep)
    {
        *success = 0;
        return 0;
    }

    return 1;
}

/*
 *  ./DreamscapeTest config=path record=file hash=file [origin=x,y,z]
 *      run the scene (built around origin, default 0) on a recording host: file holds the scene as commands,
 *      the hash file one hash per system tick; the run ends when every body sleeps
 *  ./DreamscapeTest config=path replay=file hash=file
 *      replay the record with config (another thread_count) and compare every tick's hash; exit 0 on success
 */
int main(int argc, char *argv[])
{
    ds_Init(argc, argv);

	struct arena persistent = ArenaAlloc(NULL, 64*1024*1024);
	if (!persistent.stack_ptr)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to allocate the program's persistent arena");
		FatalCleanupAndExit();
	}

    const struct ds_DeterminismTest test = ds_DeterminismProcessArguments(&persistent);
    fprintf(stderr, "scene origin: (%.17g, %.17g, %.17g)\n", test.origin.x, test.origin.y, test.origin.z);

	struct led *editor = led_Alloc(g_argv[ARGV_SIMULATION], g_argv[ARGV_RECORD], g_argv[ARGV_REPLAY], g_argv[ARGV_HASH]);
    editor->scene_origin = test.origin;

	const u64 renderer_framerate = 144;	
	r_Init(&persistent, NSEC_PER_SEC / renderer_framerate, 16*1024*1024, 1024, &editor->render_mesh_db);
	
    u32 success = 1;
    while (editor->running)
    {
    	ProfFrameMark;

		ds_DeallocTaggedWindows();


		ds_ProcessEvents();

        led_Main(editor, editor->physics->ns_tick);
		led_UiMain(editor);
		r_EditorMain(editor);

        if (!ds_DeterminismRunning(&test, editor, &success))
        {
            editor->running = 0;
        }
    }

    if (!test.generate)
    {
        (success)
            ? fprintf(stderr, "========================== SUCCESS ==========================\n")
            : fprintf(stderr, "========================== FAILURE ==========================\n");
    }

	led_Dealloc(editor);
	ArenaFree(&persistent);
	ds_Shutdown();

	return (success) ? 0 : 1;
}
