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

#include <windows.h>
#include <bcrypt.h>
#include <stdio.h>

#include "kas_common.h"
#include "kas_random.h"
#include "sys_public.h"
#include "ds_led.h"
#include "ds_renderer.h"

static void win_init_rng(void)
{
#if defined(KAS_TEST_CORRECTNESS)
	u64 seed[4] = { 6712394175642371735lu, 15709062239796375561lu, 2231484769219996854lu, 779317575278281131lu };
#else
	u64 seed[4];
	BCRYPT_ALG_HANDLE hAlgorithm;
	LPCWSTR pszAlgId = BCRYPT_RNG_ALGORITHM;
	LPCWSTR pszImplementation = NULL;
	ULONG dwFlags = 0;
	NTSTATUS status = BCryptOpenAlgorithmProvider(&hAlgorithm, pszAlgId, pszImplementation, dwFlags);
	if (!BCRYPT_SUCCESS(status))
	{
		fprintf(stderr, "Couldn't close algorithm provider: %08x\n", status);
		return;
	}

	status = BCryptGenRandom(hAlgorithm, ((PUCHAR) seed), sizeof(seed), 0);
	if (!BCRYPT_SUCCESS(status))
	{
		fprintf(stderr, "Couldn't initiate rng source: %08x\n", status);
		return;
	}

	status = BCryptCloseAlgorithmProvider(hAlgorithm, 0);
	if (!BCRYPT_SUCCESS(status))
	{
		fprintf(stderr, "Couldn't close algorithm provider: %08x\n", status);
		return;
	}
#endif
	g_xoshiro_256_init(seed);
	thread_xoshiro_256_init_sequence();
}

int CALLBACK WinMain(HINSTANCE h_instance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
	win_init_rng();

	struct arena mem_persistent = arena_alloc(32*1024*1024);
	system_resources_init(&mem_persistent);
	cmd_alloc();
	ui_init_global_state();
	asset_database_init(&mem_persistent);
#if defined(KAS_TEST_CORRECTNESS) || defined(KAS_TEST_PERFORMANCE)
	test_main();
#else
	struct led *editor = led_alloc();
	
	const u64 renderer_framerate = 144;	
	r_init(&mem_persistent, NSEC_PER_SEC / renderer_framerate, 16*1024*1024, 1024, &editor->render_mesh_db);
	
	u64 old_time = editor->ns;
	while (editor->running)
	{
		PROF_FRAME_MARK;

		system_free_tagged_windows();

		task_context_frame_clear();

		const u64 new_time = time_ns();
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
	arena_free(&mem_persistent);
#endif
	return 0;
}
