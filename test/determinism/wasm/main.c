/*
==========================================================================
    Copyright (C) 2025,2026 Axel Sandstedt 

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
#include <emscripten/console.h>
#include <emscripten/wasm_worker.h>
#include <emscripten/threading.h>

#include "ds_base.h" 
#include "ds_math.h"
#include "ds_platform.h"
#include "ds_graphics.h"
#include "ds_asset.h"
#include "ds_ui.h"
#include "ds_led.h"
#include "ds_job.h"

struct arena mem_persistent;
struct led *editor;

static void ds_MainLoop(void)
{
	static u64 old_time = 0;
	old_time = editor->ns;
	if (editor->running)
	{
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
	else
	{
		static u32 cleanup = 1;
		if (cleanup)
		{
	        led_Dealloc(editor);
	        AssetShutdown();
	        ds_GraphicsApiShutdown();
	        ds_PlatformApiShutdown();
	        LogShutdown();
	        ds_MemApiShutdown();
			cleanup = 0;
		}
	}
}

int main(int argc, char *argv[])
{	
    u64 seed[4];
	RngSystem(seed, sizeof(seed));
	Xoshiro256Init(seed);

	ds_MemApiInit();

	mem_persistent = ArenaAlloc(NULL, 256*1024*1024);
	LogInit(&mem_persistent, "log.txt");

	ds_TimeApiInit(&mem_persistent);

    const u64 thread_framesize = 4*1024*1024;
    const u64 thread_scratchsize = 1*1024*1024;
    const u64 scratch_count = 5;
	ds_ThreadMasterInit(&mem_persistent, thread_framesize, thread_scratchsize, scratch_count);
	ds_ArchConfigInit(&mem_persistent);

	ds_StringApiInit(g_arch_config->logical_core_count);

	ds_PlatformApiInit(&mem_persistent, thread_framesize, thread_scratchsize, scratch_count);

	ds_GraphicsApiInit();

	ds_UiApiInit();

	AssetInit(&mem_persistent);

	editor = led_Alloc();

	const u64 renderer_framerate = 144;	
	r_Init(&mem_persistent, NSEC_PER_SEC / renderer_framerate, 16*1024*1024, 1024, &editor->render_mesh_db);
	
	emscripten_set_main_loop(ds_MainLoop, 0, 1);

	return 0;
}
