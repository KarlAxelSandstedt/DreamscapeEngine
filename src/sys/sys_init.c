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

#include "ds_platform.h"

static struct dsSysEnv g_sys_env_storage = { 0 };
struct dsSysEnv *g_sys_env = &g_sys_env_storage;

static void DsSysEnvInit(struct arena *mem)
{
	g_sys_env->user_privileged = SystemAdminCheck();	
	g_sys_env->cwd = FileNull();
	if (CwdSet(mem, ".") != FS_SUCCESS)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to open the current working directory");
		FatalCleanupAndExit();
	}
}

void ds_PlatformApiInit(struct arena *mem)
{
 	DsSysEnvInit(mem);

#if __DS_PLATFORM__ != __DS_WEB__
	Log(T_SYSTEM, S_NOTE, "clock resolution (us): %3f", (f64) NsResolution() / 1000.0);
	Log(T_SYSTEM, S_NOTE, "Rdtsc estimated frequency (GHz): %3f", (f32) TscFrequency() / 1000000000);
	for (u32 i = 0; i < g_arch_config->logical_core_count; ++i)
	{
		Log(T_SYSTEM, S_NOTE, "core %u tsc skew (reltive to core 0): %lu", i, g_tsc_skew[i]);
	}
#endif
	//ds_GraphicsApiInit();
	//task_context_init(mem, g_arch_config->Logical_core_count);
}

void ds_PlatformApiShutdown(void)
{
	//task_context_destroy(g_task_ctx);
	//ds_GraphicsApiShutdown();
}
