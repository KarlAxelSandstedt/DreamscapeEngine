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

#include "sys_local.h"
#include "Log.h"
#include "dtoa.h"

static struct dsSysEnv g_sys_env_storage = { 0 };
struct dsSysEnv *g_sys_env = &g_sys_env_storage;

void ds_sys_env_init(struct arena *mem)
{
	g_sys_env->user_privileged = SystemAdminCheck();	
	g_sys_env->cwd = FileNull();
	if (CwdSet(mem, ".") != FS_SUCCESS)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to open the current working directory");
		FatalCleanupAndExit(0);
	}
}

void system_resources_init(struct arena *mem)
{
	init_error_handling_func_ptrs();
	filesystem_init_func_ptrs();

 	ds_sys_env_init(mem);
	ds_ThreadMasterInit(mem);
	ds_TimeApiInit(mem);
	LogInit(mem, "Log.txt");

	if (!ds_ArchConfigInit(mem))
	{
		LogString(T_SYSTEM, S_FATAL, "Unsupported instrincs required");
		FatalCleanupAndExit(0);
	}

	/* must initalize stuff in multithreaded dtoa/strtod */
	dmg_dtoa_init(g_arch_config->Logical_core_count);

#if __DS_PLATFORM__ != __DS_WEB__
	Log(T_SYSTEM, S_NOTE, "clock resolution (us): %3f", (f64) NsResolution() / 1000.0);
	Log(T_SYSTEM, S_NOTE, "Rdtsc estimated frequency (GHz): %3f", (f32) TscFrequency() / 1000000000);
	for (u32 i = 0; i < g_arch_config->Logical_core_count; ++i)
	{
		Log(T_SYSTEM, S_NOTE, "core %u tsc skew (reltive to core 0): %lu", i, g_tsc_skew[i]);
	}
#endif

	/* 1GB */
	//const u32 count_256B = 1024*1024*4;
	const u32 count_256B = 1024*4;
	/* 1GB */
	//const u32 count_1MB = 1024;
	const u32 count_1MB = 64;

	global_thread_block_allocators_alloc(count_256B, count_1MB);
	system_graphics_init();
	task_context_init(mem, g_arch_config->Logical_core_count);
}

void system_resources_cleanup(void)
{
	task_context_destroy(g_task_ctx);
	system_graphics_destroy();
	global_thread_block_allocators_free();

	LogShutdown();
}
