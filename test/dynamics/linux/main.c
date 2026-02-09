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

int main(int argc, char *argv[])
{	
	u64 seed[4];
	RngSystem(seed, sizeof(seed));
	Xoshiro256Init(seed);
		
	const u32 count_256B = 4*1024;
	const u32 count_1MB = 64;

	ds_MemApiInit(count_256B, count_1MB);

	struct arena persistent = ArenaAlloc(32*1024*1024);
	LogInit(&persistent, "log.txt");

	ds_TimeApiInit(&persistent);

	ds_ThreadMasterInit(&persistent);
	ds_ArchConfigInit(&persistent);

	ds_StringApiInit(g_arch_config->logical_core_count);

	ds_PlatformApiInit(&persistent);

	ds_GraphicsApiInit();

	ds_UiApiInit();

	AssetInit(&persistent);

	struct led *editor = led_Alloc();

	const u64 renderer_framerate = 144;	
	r_Init(&persistent, NSEC_PER_SEC / renderer_framerate, 16*1024*1024, 1024, &editor->render_mesh_db);
	
	u64 old_time = editor->ns;
	while (editor->running)
	{
		ProfFrameMark;

		ds_DeallocTaggedWindows();

		task_context_frame_clear();

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
	ds_MemApiShutdown();

	return 0;
}
