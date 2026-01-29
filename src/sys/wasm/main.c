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
#include <stdint.h>
#include <string.h>
#include <float.h>
#include <emscripten/console.h>
#include <emscripten/wasm_worker.h>
#include <emscripten/threading.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include "ds_common.h"
#include "wasm_local.h"
#include "memory.h"
#include "ds_string.h"
#include "sys_public.h"
#include "asset_public.h"
#include "r_public.h"
#include "led_public.h"
#include "ui_public.h"
#include "ds_random.h"

struct arena mem_persistent;
struct led *editor;

static void main_loop(void)
{
	static u64 old_time = 0;
	old_time = editor->ns;
	if (editor->running)
	{
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
	else
	{
		static u32 cleanup = 1;
		if (cleanup)
		{
			led_dealloc(editor);
			asset_database_cleanup();
			cmd_free();
			system_resources_cleanup();
			ArenaFree(&mem_persistent);
			cleanup = 0;
		}
	}
}

int main(int argc, char *argv[])
{	
	u64 seed[4];
	int fd = open("/dev/urandom", O_RDONLY);
	const ssize_t size_read = read(fd, seed, sizeof(seed));
	close(fd);
	if (size_read != sizeof(seed))
	{
		fprintf(stderr, "Couldn't read from rng source, exiting\n");
		return 0;
	}
	Xoshiro256Init(seed);
	ThreadXoshiro256InitSequence();

	mem_persistent = ArenaAlloc(32*1024*1024);
	system_resources_init(&mem_persistent);
	cmd_alloc();
	ui_init_global_state();
	asset_database_init(&mem_persistent);

	editor = led_alloc();
	//TODO set 0 => draw as fast as possible 
	const u64 renderer_framerate = 60;	
	r_init(&mem_persistent, NSEC_PER_SEC / renderer_framerate, 16*1024*1024, 1024, &editor->render_mesh_db);

	emscripten_set_main_loop(main_loop, 0, 1);

	return 0;
}
