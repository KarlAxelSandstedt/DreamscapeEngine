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
#include "ds_math.h"
#include "ds_platform.h"
#include "ds_graphics.h"
#include "ds_asset.h"
#include "ds_ui.h"
#include "ds_led.h"
#include "ds_job.h"

struct ds_DeterminismTest
{
    utf8    file;
    char *  file_cstr;

    u32     generate;
    u32     thread_count;

    /* file data */
    u64     seed[4];    /* BE */
    u64     hash;       /* BE */
};

static struct ds_DeterminismTest ds_DeterminismProcessArguments(struct arena *persistent, const utf8 *argument, const u32 argument_count)
{
    if (argument_count < 3 || 4 < argument_count)
    {
        fprintf(stderr, "Bad number of arguments to executable, exiting.\n");
        exit(0);
    }
    
    const utf8 generate = Utf8Inline("--generate");
    const utf8 load = Utf8Inline("--load");

    struct ds_DeterminismTest test = 
    { 
        .file = argument[2],
        .file_cstr = CstrUtf8(persistent, test.file),
        .generate = 0,
        .thread_count = 0,
    };
    
    if (Utf8Equivalence(argument[1], generate))
    {
        test.generate = 1;
	    RngSystem(test.seed, sizeof(test.seed));
    }
    else if (Utf8Equivalence(argument[1], load))
    {
        struct dsBuffer buf = FileDumpAtCwd(persistent, test.file_cstr);
        const u64 expected_size = sizeof(test.seed) + sizeof(test.hash);
        if (buf.size != expected_size)
        {
            fprintf(stderr, "Bad determinism test file size, exiting.\n");
            exit(0);
        }
        
        struct ss ss = ss_Buffered(buf.data, buf.size);
        test.seed[0] = ss_Read64Be(&ss).u;
        test.seed[1] = ss_Read64Be(&ss).u;
        test.seed[2] = ss_Read64Be(&ss).u;
        test.seed[3] = ss_Read64Be(&ss).u;
        test.hash = ss_Read64Be(&ss).u;
    }
    else
    {
        fprintf(stderr, "Bad operation provided to executable, exiting.\n");
        exit(0);
    }

    if (argument_count == 4)
    {
        const struct parseRetval ret = U64Utf8(argument[3]);
        test.thread_count = (ret.op_result == PARSE_SUCCESS) 
                          ? ret.u32
                          : 0;
    }

    return test;
}

static void ds_DeterminismGenerate(struct arena *persistent, const struct ds_DeterminismTest *test)
{
    struct file file = { .handle = FILE_HANDLE_INVALID };
    const u32 truncate = 1;
    if (FS_SUCCESS != FileTryCreateAtCwd(persistent, &file, test->file_cstr, truncate))
    {
        fprintf(stderr, "Failed to create file, exiting.\n");
        exit(0);
    }

    u8 buf[5*sizeof(u64)];
    struct ss ss = ss_Buffered(buf, sizeof(buf));
    ss_WriteU64Be(&ss, test->seed[0]);
    ss_WriteU64Be(&ss, test->seed[1]);
    ss_WriteU64Be(&ss, test->seed[2]);
    ss_WriteU64Be(&ss, test->seed[3]);
    ss_WriteU64Be(&ss, test->hash);

    const u64 bytes_written = FileWriteAppend(&file, buf, sizeof(buf));
    ds_Assert(bytes_written == sizeof(buf));

    FileClose(&file);
}

/*
 *  ./DeterminismTest --generate "determinism_test_file" Optional(thread_count) => generate determinism test file
 *  ./DeterminismTest --load     "determinism_test_file" Optional(thread_count) => run determinism test file 
 */
int main(int argc, char *argv[])
{		
	ds_MemApiInit();

	struct arena persistent = ArenaAlloc(NULL, 256*1024*1024);

    const u32 argument_count = (u32) argc;
    utf8 *argument = ArenaPush(&persistent, sizeof(utf8)*argument_count);
    for (i32 i = 0; i < argc; ++i)
    {
        argument[i] = Utf8Cstr(&persistent, argv[i]);
    }
    struct ds_DeterminismTest test = ds_DeterminismProcessArguments(&persistent, argument, argument_count);

	LogInit(&persistent, "log.txt");
	Xoshiro256Init(test.seed);
	
	ds_TimeApiInit(&persistent);

    const u64 thread_framesize = 4*1024*1024;
    const u64 thread_scratchsize = 1*1024*1024;
    const u64 scratch_count = 5;
	ds_ThreadMasterInit(&persistent, thread_framesize, thread_scratchsize, scratch_count);
	ds_ArchConfigInit(&persistent);

    if (test.thread_count == 0)
    {
        test.thread_count = g_arch_config->logical_core_count - 2;
    }

	ds_StringApiInit(test.thread_count);

	ds_PlatformApiInit(&persistent, thread_framesize, thread_scratchsize, scratch_count, test.thread_count);

	ds_GraphicsApiInit();

	ds_UiApiInit();

	AssetInit(&persistent);

	struct led *editor = led_Alloc();

	const u64 renderer_framerate = 144;	
	r_Init(&persistent, NSEC_PER_SEC / renderer_framerate, 16*1024*1024, 1024, &editor->render_mesh_db);
	
	u64 old_time = editor->ns;
	//while (editor->running)
	{
		ProfFrameMark;

		ds_DeallocTaggedWindows();

        ds_JobSchedulerFrameClear();

		const u64 new_time = ds_TimeNs();
		const u64 ns_tick = new_time - old_time;
		old_time = new_time;

		ds_ProcessEvents();

		led_Main(editor, ns_tick);
		led_UiMain(editor);
		r_EditorMain(editor);
	}
	
	led_Dealloc(editor);
	AssetShutdown();
	ds_GraphicsApiShutdown();
	ds_PlatformApiShutdown();
	LogShutdown();

    ds_DeterminismGenerate(&persistent, &test);

	ds_MemApiShutdown();

	return 0;
}
