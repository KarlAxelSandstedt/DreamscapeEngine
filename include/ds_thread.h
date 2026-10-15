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

/*
ds_thread.h
===========
Engine threads (pthreads, Windows threads): an index in clone order and per-thread scratch and frame
arenas. ds_Init sets up the main thread (index 0) and the job scheduler clones the workers; the rules for
indices and for threads the engine didn't create are in ds_init.h.
*/

#ifndef __DREAMSCAPE_THREAD_H__
#define __DREAMSCAPE_THREAD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_allocator.h"

typedef struct ds_Thread ds_Thread;
/* the calling thread's ds_Thread; NULL on a thread the engine didn't create (and before ds_ThreadMasterInit) */
extern ds_ThreadLocal ds_Thread *g_tl_self;

#if __DS_PLATFORM__ == __DS_LINUX__ ||__DS_PLATFORM__ == __DS_WEB__

#include <pthread.h>
typedef pid_t	tid;

struct ds_Thread
{
	void	        (*start)(ds_Thread *);	/* entry point, called with this ds_Thread */
	void *	        args;
	void *	        ret;			/* NULL and 0 until start sets them */
	u64		        ret_size;
	u64		        stack_size;		/* bytes, rounded up to whole pages */
	pid_t	        ppid;			/* getppid(): the process's parent */
	pid_t	        gtid;			/* getpid(): the process (thread group) */
	tid		        tid;			/* gettid() */
	u32		        index;
	pthread_t       pthread;

    u32             scratch_next;
    u32             scratch_count;
    struct arena *  scratch;
    struct arena    frame_arr[2];   /* double-buffered frame arenas */
    struct arena *  frame;
    u32             frame_index;
};

#elif __DS_PLATFORM__ == __DS_WIN64__

#include <windows.h>
typedef DWORD	tid;

struct ds_Thread
{
	void	        (*start)(ds_Thread *);	/* entry point, called with this ds_Thread */
	void 	        *args;
	void 	        *ret;			/* NULL and 0 until start sets them */
	u64		        ret_size;
	u64		        stack_size;		/* bytes, rounded up to whole pages */
	u32		        index;
	DWORD	        tid;			/* GetCurrentThreadId() */
	HANDLE	        native;

    u32             scratch_next;
    u32             scratch_count;
    struct arena *  scratch;
    struct arena    frame_arr[2];   /* double-buffered frame arenas */
    struct arena *  frame;
    u32             frame_index;
};

#endif

/*
Arenas of the calling engine thread. ArenaPushScratch returns the next of its scratch arenas, emptied
(fatal when all are in use); ArenaPopScratch releases the latest one (LIFO). ArenaSwitchAndFlushFrame makes
the other frame arena current and empties it.
*/
struct arena *  ArenaPushScratch(void);
void            ArenaPopScratch(void);
void            ArenaSwitchAndFlushFrame(void);

/* Make the calling thread engine thread 0 with its arenas on mem; once, before any ds_ThreadClone. */
void        ds_ThreadMasterInit(struct arena *mem, const u64 framesize, const u64 scratch_size, const u32 scratch_count);
/*
Start a thread running start(thread) with its ds_Thread, arenas and stack size (rounded up to pages) taken
from mem; the next index in clone order (< DS_THREAD_COUNT_MAX) is assigned before it starts. Fatal on
failure.
*/
ds_Thread * ds_ThreadClone(struct arena *mem, void (*start)(ds_Thread *), void *args, const u64 stack_size, const u64 frame_size, const u64 scratch_size, const u32 scratch_count);
/* End the calling thread (its g_tl_self becomes NULL). */
void		ds_ThreadExit(void);
/* Block until thr has ended; once per thread (Windows closes its handle). */
void 		ds_ThreadWait(const ds_Thread *thr);
/* thr's ret, ret_size and args. */
void *		ds_ThreadReturnValue(const ds_Thread *thr);
u64		    ds_ThreadReturnValueSize(const ds_Thread *thr);
void *  	ds_ThreadArguments(const ds_Thread *thr);
/* The OS thread id: set by the thread itself as it starts, so read it from elsewhere only after that. */
tid		    ds_ThreadTid(const ds_Thread *thr);
tid 		ds_ThreadSelfTid(void);
/* The index: 0 for the master thread, then 1, 2, ... in clone order. Self: the caller's (an engine thread). */
u32		    ds_ThreadIndex(const ds_Thread *thr);
u32		    ds_ThreadSelfIndex(void);
/* Give the rest of the caller's time slice to the OS. */
void        ds_ThreadYield(void);

#ifdef __cplusplus
}
#endif

#endif
