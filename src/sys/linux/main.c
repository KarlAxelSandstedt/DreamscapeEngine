/*
==========================================================================
    Copyright (C) 2025 Axel Sandstedt 

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
#include <stdint.h>
#include <string.h>
#include <float.h>
#include <sys/random.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/random.h>
#include <unistd.h>

#include "ds_common.h"
#include "linux_local.h"
#include "memory.h"
#include "ds_string.h"
#include "sys_public.h"
#include "asset_public.h"
#include "r_public.h"
#include "led_public.h"
#include "ui_public.h"
#include "ds_random.h"

#if defined(DS_TEST_CORRECTNESS) || defined(DS_TEST_PERFORMANCE)
#include "test_public.h"
#endif

const char * const main_str = "main";

int main(int argc, char *argv[])
{	
#if defined(DS_TEST_CORRECTNESS)
	u64 seed[4] = { 6712394175642371735lu, 15709062239796375561lu, 2231484769219996854lu, 779317575278281131lu };
#else
	u64 seed[4];
	if (getrandom(&seed[0], sizeof(seed), 0) == -1)
	{
		fprintf(stderr, "Couldn't initiate rng source, exiting\n");
		return 0;
	}
#endif
	Xoshiro256Init(seed);
	ThreadXoshiro256InitSequence();

	struct arena mem_persistent = ArenaAlloc(32*1024*1024);
	system_resources_init(&mem_persistent);
	cmd_alloc();
	ui_init_global_state();
	asset_database_init(&mem_persistent);
#if defined(DS_TEST_CORRECTNESS) || defined(DS_TEST_PERFORMANCE)
	test_main();
#else
	struct led *editor = led_alloc();
	
	const u64 renderer_framerate = 144;	
	r_init(&mem_persistent, NSEC_PER_SEC / renderer_framerate, 16*1024*1024, 1024, &editor->render_mesh_db);
	
	u64 old_time = editor->ns;
	while (editor->running)
	{
		ProfFrameMark;

		system_free_tagged_windows();

		task_context_frame_clear();

		const u64 new_time = ds_TimeNs();
		const u64 ns_tick = new_time - old_time;
		old_time = new_time;

		system_process_events();

		led_main(editor, ns_tick);
		led_ui_main(editor);
		r_led_main(editor);
	}

	led_dealloc(editor);
	asset_database_cleanup();
	cmd_free();
	system_resources_cleanup();
	ArenaFree(&mem_persistent);
#endif
	return 0;
}
