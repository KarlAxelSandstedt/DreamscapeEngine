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

#define _GNU_SOURCE
#include "ds_base.h"

ds_ThreadLocal struct ds_Thread *g_tl_self = NULL;
u32 a_index_counter = 1;

const char *thread_profiler_id[] = 
{
    "Master",  "Worker 1",  "Worker 2",  "Worker 3",  "Worker 4",  "Worker 5",  "Worker 6",  "Worker 7", 
	"Worker 8",  "Worker 9", "Worker 10", "Worker 11", "Worker 12", "Worker 13", "Worker 14", "Worker 15",
    "Worker 16", "Worker 17", "Worker 18", "Worker 19", "Worker 20", "Worker 21", "Worker 22", "Worker 23",
    "Worker 24", "Worker 25", "Worker 26", "Worker 27", "Worker 28", "Worker 29", "Worker 30", "Worker 31",
    "Worker 32", "Worker 33", "Worker 34", "Worker 35", "Worker 36", "Worker 37", "Worker 38", "Worker 39",
	"Worker 40", "Worker 41", "Worker 42", "Worker 43", "Worker 44", "Worker 45", "Worker 46", "Worker 47",
    "Worker 48", "Worker 49", "Worker 50", "Worker 51", "Worker 52", "Worker 53", "Worker 54", "Worker 55",
    "Worker 56", "Worker 57", "Worker 58", "Worker 59", "Worker 60", "Worker 61", "Worker 62", "Worker 63",
};


static void ds_ThreadAllocMemory(struct arena *mem, struct ds_Thread *thr, const u64 frame_size, const u64 scratch_size, const u32 scratch_count)
{
    thr->scratch_next = 0;
    thr->scratch_count = scratch_count;
    thr->scratch = ArenaPush(mem, scratch_count*sizeof(struct arena));
    for (u32 i = 0; i < scratch_count; ++i)
    {
        thr->scratch[i] = ArenaAlloc(mem, scratch_size);
    }
    thr->frame_arr[0] = ArenaAlloc(mem, frame_size);
    thr->frame_arr[1] = ArenaAlloc(mem, frame_size);
    thr->frame_index = 0;
    thr->frame = thr->frame_arr + thr->frame_index;
    ArenaPush(mem, DS_CACHE_LINE);

    if (!thr->scratch || !thr->scratch[thr->scratch_count-1].stack_ptr || !thr->frame_arr[1].stack_ptr)
    {
		LogString(T_SYSTEM, S_FATAL, "Failed to alloc thread memory, aborting.");
		FatalCleanupAndExit();
    }
}
 
struct arena *ArenaPushScratch(void)
{
    if (g_tl_self->scratch_next == g_tl_self->scratch_count)
    {
		LogString(T_SYSTEM, S_FATAL, "Scratch arenas exhausted, aborting.");
		FatalCleanupAndExit();
    }

    const u32 index = g_tl_self->scratch_next++;
    struct arena *mem = g_tl_self->scratch + index;
    ArenaFlush(mem);
    return mem;
}

void ArenaPopScratch(void)
{
    ds_Assert(g_tl_self->scratch_next);
    g_tl_self->scratch_next -= 1;
}

void ArenaSwitchAndFlushFrame(void)
{
    g_tl_self->frame_index = 1 - g_tl_self->frame_index;
    g_tl_self->frame = g_tl_self->frame_arr + g_tl_self->frame_index;
    ArenaFlush(g_tl_self->frame);
}

#if __DS_PLATFORM__ == __DS_LINUX__ ||__DS_PLATFORM__ == __DS_WEB__

#include <unistd.h>
#include <pthread.h>

static void *ds_ThreadCloneStart(void *void_thr)
{
	g_tl_self = void_thr;
	struct ds_Thread *thr = void_thr;
	thr->ppid = getppid();
	thr->gtid = getpid();
	thr->tid = gettid();
	thr->index = AtomicFetchAddRlx32(&a_index_counter, 1);
	ThreadXoshiro256InitSequence();
	ProfThreadNamed(thread_profiler_id[thr->index]);
	thr->start(thr);

	return NULL;
}

void ds_ThreadMasterInit(struct arena *mem, const u64 frame_size, const u64 scratch_size, const u32 scratch_count)
{
	g_tl_self = ArenaPush(mem, sizeof(struct ds_Thread));
	g_tl_self->ppid = getppid();
	g_tl_self->gtid = getpid();
	g_tl_self->tid = gettid();
	g_tl_self->index = 0;
    ds_ThreadAllocMemory(mem, g_tl_self, frame_size, scratch_size, scratch_count);

	ProfThreadNamed(thread_profiler_id[g_tl_self->index]);
}

ds_Thread *ds_ThreadClone(struct arena *mem, void (*start)(ds_Thread *), void *args, const u64 stack_size, const u64 frame_size, const u64 scratch_size, const u32 scratch_count)
{
	ds_Assert(stack_size > 0);

	const u64 thr_size = (sizeof(ds_Thread) % g_arch_config->cacheline == 0)
		? sizeof(ds_Thread)
		: sizeof(ds_Thread) + g_arch_config->cacheline - (sizeof(ds_Thread) % g_arch_config->cacheline);

	ds_Thread *thr = NULL;
	thr = ArenaPushAligned(mem, thr_size, g_arch_config->cacheline);

	if (thr == NULL)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to alloc thread memory, aborting.");
		FatalCleanupAndExit();
	}

	ds_Assert((u64) thr % g_arch_config->cacheline == 0);

	thr->start = start;
	thr->args = args;
	thr->ret = NULL;
	thr->ret_size = 0;
	thr->stack_size = (stack_size % g_arch_config->pagesize == 0) 
				? stack_size 
				: stack_size + (g_arch_config->pagesize - stack_size % g_arch_config->pagesize);
    ds_ThreadAllocMemory(mem, thr, frame_size, scratch_size, scratch_count);

	pthread_attr_t attr;
	if (pthread_attr_init(&attr) != 0)
	{
		LogSystemError(S_FATAL);	
		FatalCleanupAndExit();
	}

	if (pthread_attr_setstacksize(&attr, thr->stack_size) != 0)
	{
		LogSystemError(S_FATAL);	
		FatalCleanupAndExit();
	}

	size_t real_size;
	pthread_attr_getstacksize(&attr, &real_size);
	ds_Assert(real_size == thr->stack_size);

	if (pthread_create(&thr->pthread, &attr, ds_ThreadCloneStart, thr) != 0)
	{
		LogSystemError(S_FATAL);	
		FatalCleanupAndExit();
	}

	if (pthread_attr_destroy(&attr) != 0)
	{
		LogSystemError(S_FATAL);	
		FatalCleanupAndExit();
	}

	return thr;
}

void ds_ThreadExit(void)
{
	g_tl_self = NULL;
	pthread_exit(0);
}

void ds_ThreadWait(const ds_Thread *thr)
{
	void *garbage;

	i32 status = pthread_join(thr->pthread, &garbage);
	if (status != 0)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to alloc thread memory, aborting.");
		FatalCleanupAndExit();
	}
}

#elif __DS_PLATFORM__ == __DS_WIN64__

DWORD WINAPI ds_ThreadCloneStart(LPVOID void_thr)
{
	g_tl_self = void_thr;
	struct ds_Thread *thr = void_thr;
	thr->tid = GetCurrentThreadId();
	thr->index = AtomicFetchAddRlx32(&a_index_counter, 1);
	ThreadXoshiro256InitSequence();
	ProfThreadNamed(thread_profiler_id[thr->index]);
	thr->start(thr);

	return 0;
}

void ds_ThreadMasterInit(struct arena *mem, const u64 frame_size, const u64 scratch_size, const u32 scratch_count)
{
	g_tl_self = ArenaPush(mem, sizeof(struct ds_Thread));
	g_tl_self->tid = GetCurrentThreadId();
	g_tl_self->index = 0;
    ds_ThreadAllocMemory(mem, g_tl_self, frame_size, scratch_size, scratch_count);

	ProfThreadNamed(thread_profiler_id[g_tl_self->index]);
}

ds_Thread *ds_ThreadClone(struct arena *mem, void (*start)(ds_Thread *), void *args, const u64 stack_size, const u64 frame_size, const u64 scratch_size, const u32 scratch_count)
{
	ds_Assert(stack_size > 0);

	const u64 thr_size = (sizeof(ds_Thread) % g_arch_config->cacheline == 0)
		? sizeof(ds_Thread)
		: sizeof(ds_Thread) + g_arch_config->cacheline - (sizeof(ds_Thread) % g_arch_config->cacheline);

	ds_Thread *thr = NULL;
	thr = ArenaPushAligned(mem, thr_size, g_arch_config->cacheline);

	if (thr == NULL)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to alloc thread memory, aborting.");
		FatalCleanupAndExit();
	}

	ds_Assert((u64) thr % g_arch_config->cacheline == 0);

	thr->start = start;
	thr->args = args;
	thr->ret = NULL;
	thr->ret_size = 0;
	thr->stack_size = (stack_size % g_arch_config->pagesize == 0) 
				? stack_size 
				: stack_size + (g_arch_config->pagesize - stack_size % g_arch_config->pagesize);
    ds_ThreadAllocMemory(mem, thr, frame_size, scratch_size, scratch_count);

  	LPSECURITY_ATTRIBUTES   lpThreadAttributes = NULL; 	/* default attributes */
  	DWORD                   dwCreationFlags = 0;		/* default creation flags */
	thr->native = CreateThread(lpThreadAttributes, stack_size, ds_ThreadCloneStart, thr, dwCreationFlags, NULL);
	if (!thr->native)
	{
		LogSystemError(S_FATAL);
		FatalCleanupAndExit();
	}

	return thr;
}

void ds_ThreadExit(void)
{
	g_tl_self = NULL;
	ExitThread(0);
}

void ds_ThreadWait(const ds_Thread *thr)
{
	u32 waited = 1;
	DWORD ret = WaitForSingleObjectEx(thr->native, INFINITE, FALSE);
	if (ret != WAIT_OBJECT_0)
	{
		if (ret == WAIT_FAILED)
		{
			LogSystemError(S_FATAL);
			FatalCleanupAndExit();
		}
		else
		{
			LogString(T_SYSTEM, S_ERROR, "Unexpected disruption of thread wait in ds_ThreadWait\n");
			return;
		}
	}

	DWORD garbage;
	if (!GetExitCodeThread(thr->native, &garbage))
	{
		LogSystemError(S_ERROR);
		return;
	}

	if (!CloseHandle(thr->native))
	{
		LogSystemError(S_ERROR);
	}

	return;
}

#endif

void *ds_ThreadReturnValue(const ds_Thread *thr)
{
	return thr->ret;	
}

void *ds_ThreadArguments(const ds_Thread *thr)
{
	return thr->args;
}

u64 ds_ThreadReturnValueSize(const ds_Thread *thr)
{
	return thr->ret_size;
}

tid ds_ThreadTid(const ds_Thread *thr)
{
	return thr->tid;
}

tid ds_ThreadSelfTid(void)
{
	return g_tl_self->tid;
}

u32 ds_ThreadIndex(const ds_Thread *thr)
{
	return thr->index;
}

u32 ds_ThreadSelfIndex(void)
{
	return g_tl_self->index;
}
