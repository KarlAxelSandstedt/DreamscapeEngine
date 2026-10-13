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

#include "ds_vector.h"

/* the program's own memory; ds_Init's persistent arena belongs to the engine */
static struct arena g_persistent;

/* ./DreamscapeTest [config_path] */
int main(int argc, char *argv[])
{
	ds_Init((argc > 1) ? argv[1] : NULL, "log.txt");

	g_persistent = ArenaAlloc(NULL, 64*1024*1024);
	if (!g_persistent.stack_ptr)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to allocate the program's persistent arena");
		FatalCleanupAndExit();
	}

	struct led *editor = led_Alloc(g_config->thread_count, g_config->thread_framesize);

	const u64 renderer_framerate = 144;	
	r_Init(&g_persistent, NSEC_PER_SEC / renderer_framerate, 16*1024*1024, 1024, &editor->render_mesh_db);
	
	u64 old_time = editor->ns;
	while (editor->running)
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
	ArenaFree(&g_persistent);
	ds_Shutdown();

	return 0;
}
