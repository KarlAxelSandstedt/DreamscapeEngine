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

dsThreadLocal struct dsThread *g_tl_self = NULL;
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

#if __DS_PLATFORM__ == __DS_LINUX__ ||__DS_PLATFORM__ == __DS_WEB__

#include <unistd.h>
#include <pthread.h>

/*
 *	Assuming we use CLONE_THREAD, we have the following:
 *	PPID - parent pid of master thread			   (Shared between all threads)
 *	TGID - thread group id, and the unique TID of the master thread (Shared between all threads due to CLONE_THREAD)
 *	TID  - unique thread identifier
 *
 *	We need CLONE_THREAD for kernel tracing, which means that we cannot use waitpid anymore on cloned threads;
 *	instead cpid is cleared to 0 on thread exit.
 *
 *	The PPID of a thread is retrieved by getppid();
 *	The TGID of a thread is retrieved by getpid();
 *	The TID of a thread is retrieved by gettid();
 */
struct dsThread
{
	void		(*start)(dsThread *);	/* beginning of execution for thread */
	void *		args;			/* thread arguments */
	void *		ret;			/* adress to returned value, if any */
	u64		ret_size;		/* size of returned value */
	u64		stack_size;		/* size of stack (not counting protected page at bottom) */
	pid_t		ppid;			/* native parent tid */
	pid_t		gtid;			/* native group tid */
	tid		tid;			/* native thread id */
	u32		index;			/* thread index, used for accessing thread data in arrays  */
	pthread_t	pthread;		/* thread internal */
};

static void *ds_ThreadCloneStart(void *void_thr)
{
	g_tl_self = void_thr;
	struct dsThread *thr = void_thr;
	thr->ppid = getppid();
	thr->gtid = getpid();
	thr->tid = gettid();
	thr->index = AtomicFetchAddRlx32(&a_index_counter, 1);
	ProfThreadNamed(thread_profiler_id[thr->index]);
	thr->start(thr);

	return NULL;
}

void ds_ThreadMasterInit(struct arena *mem)
{
	g_tl_self = ArenaPush(mem, sizeof(struct dsThread));
	g_tl_self->ppid = getppid();
	g_tl_self->gtid = getpid();
	g_tl_self->tid = gettid();
	g_tl_self->index = 0;
	ProfThreadNamed(thread_profiler_id[g_tl_self->index]);
}

dsThread *ds_ThreadClone(struct arena *mem, void (*start)(dsThread *), void *args, const u64 stack_size)
{
	ds_Assert(stack_size > 0);

	const u64 thr_size = (sizeof(dsThread) % g_arch_config->cacheline == 0)
		? sizeof(dsThread)
		: sizeof(dsThread) + g_arch_config->cacheline - (sizeof(dsThread) % g_arch_config->cacheline);

	dsThread *thr = NULL;
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

void ds_ThreadWait(const dsThread *thr)
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

struct dsThread
{
	void		(*start)(dsThread *);	/* beginning of execution for thread */
	void 		*args;			/* thread arguments */
	void 		*ret;			/* adress to returned value, if any */
	u64		ret_size;		/* size of returned value */
	u64		stack_size;		/* size of stack (not counting protected page at bottom) */
	u32		index;			/* thread index, used for accessing thread data in arrays  */
	DWORD		tid;			/* native thread id (pid_t on linux) */
	HANDLE		native;			/* native thread handle */
};

DWORD WINAPI ds_ThreadCloneStart(LPVOID void_thr)
{
	g_tl_self = void_thr;
	struct dsThread *thr = void_thr;
	thr->tid = GetCurrentThreadId();
	thr->index = AtomicFetchAddRlx32(&a_index_counter, 1);
	ThreadXoshiro256InitSequence();
	ProfThreadNamed(thread_profiler_id[thr->index]);
	thr->start(thr);

	return 0;
}

void ds_ThreadMasterInit(struct arena *mem)
{
	g_tl_self = ArenaPush(mem, sizeof(struct dsThread));
	g_tl_self->tid = GetCurrentThreadId();
	g_tl_self->index = 0;
	ProfThreadNamed(thread_profiler_id[g_tl_self->index]);
}

dsThread *ds_ThreadClone(struct arena *mem, void (*start)(dsThread *), void *args, const u64 stack_size)
{
	ds_Assert(stack_size > 0);

	const u64 thr_size = (sizeof(dsThread) % g_arch_config->cacheline == 0)
		? sizeof(dsThread)
		: sizeof(dsThread) + g_arch_config->cacheline - (sizeof(dsThread) % g_arch_config->cacheline);

	dsThread *thr = NULL;
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

void ds_ThreadWait(const dsThread *thr)
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

void *ds_ThreadReturnValue(const dsThread *thr)
{
	return thr->ret;	
}

void *ds_ThreadArguments(const dsThread *thr)
{
	return thr->args;
}

u64 ds_ThreadReturnValueSize(const dsThread *thr)
{
	return thr->ret_size;
}

tid ds_ThreadTid(const dsThread *thr)
{
	return thr->tid;
}

tid ds_ThreadSelfTid(void)
{
	return g_tl_self->tid;
}

u32 ds_ThreadIndex(const dsThread *thr)
{
	return thr->index;
}

u32 ds_ThreadSelfIndex(void)
{
	return g_tl_self->index;
}
