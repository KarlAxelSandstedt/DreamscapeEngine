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

#include "ds_common.h"
#include "ds_random.h"
#include "sys_public.h"
#include "led_public.h"
#include "r_public.h"

static void win_init_rng(void)
{
#if defined(DS_TEST_CORRECTNESS)
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
	Xoshiro256Init(seed);
	ThreadXoshiro256InitSequence();
}

int CALLBACK WinMain(HINSTANCE h_instance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
	win_init_rng();

	struct arena mem_persistent = ArenaAlloc(32*1024*1024);
	ds_PlatformApiInit(&mem_persistent);
	ds_CmdApiInit();
	ds_UiApiInit();
	AssetInit(&mem_persistent);
#if defined(DS_TEST_CORRECTNESS) || defined(DS_TEST_PERFORMANCE)
	test_main();
#else
	struct led *editor = led_Alloc();
	
	const u64 renderer_framerate = 144;	
	r_Init(&mem_persistent, NSEC_PER_SEC / renderer_framerate, 16*1024*1024, 1024, &editor->render_mesh_db);
	
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
		led_ui_main(editor);
		r_EditorMain(editor);
	}

	led_Dealloc(editor);
	AssetShutdown();
	ds_CmdApiShutdown();
	ds_PlatformApiShutdown();
	ArenaFree(&mem_persistent);
#endif
	return 0;
}
