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
struct ds_Thread
{
	void	        (*start)(ds_Thread *);	/* beginning of execution for thread, called with this ds_Thread */
	void *	        args;			/* thread arguments */
	void *	        ret;			/* address to returned value, if any (NULL until start sets it) */
	u64		        ret_size;		/* size of returned value (0 until start sets it) */
	u64		        stack_size;		/* size of stack (not counting protected page at bottom), rounded up to whole pages */
	pid_t	        ppid;			/* native parent tid: getppid(), the process's parent */
	pid_t	        gtid;			/* native group tid: getpid(), the process (thread group) */
	tid		        tid;			/* native thread id: gettid() */
	u32		        index;			/* thread index, used for accessing thread data in arrays  */
	pthread_t       pthread;		/* thread internal */

    u32             scratch_next;
    u32             scratch_count;
    struct arena *  scratch;
    struct arena    frame_arr[2];   /* Double buffered frame arena for easy caching */
    struct arena *  frame;
    u32             frame_index;
};

#elif __DS_PLATFORM__ == __DS_WIN64__

#include <windows.h>
typedef DWORD	tid;

struct ds_Thread
{
	void	        (*start)(ds_Thread *);	/* beginning of execution for thread, called with this ds_Thread */
	void 	        *args;			/* thread arguments */
	void 	        *ret;			/* address to returned value, if any (NULL until start sets it) */
	u64		        ret_size;		/* size of returned value (0 until start sets it) */
	u64		        stack_size;		/* size of stack (not counting protected page at bottom), rounded up to whole pages */
	u32		        index;			/* thread index, used for accessing thread data in arrays  */
	DWORD	        tid;			/* native thread id (pid_t on linux): GetCurrentThreadId() */
	HANDLE	        native;			/* native thread handle */

    u32             scratch_next;
    u32             scratch_count;
    struct arena *  scratch;
    struct arena    frame_arr[2];   /* Double buffered frame arena for easy caching */
    struct arena *  frame;
    u32             frame_index;
};

#endif

/* Push thread owned Scratch Arena: the next of the caller's scratch arenas, emptied; fatal when all are in use */
struct arena *  ArenaPushScratch(void);
/* Pop thread owned Scratch Arena: the latest one pushed (LIFO) */
void            ArenaPopScratch(void);
/* Switch thread owned frame arenas and clear the switched in arena. */
void            ArenaSwitchAndFlushFrame(void);

/* Alloc and initiate master thread information (index 0, arenas on mem); should only be called once, before any ds_ThreadClone! */
void        ds_ThreadMasterInit(struct arena *mem, const u64 framesize, const u64 scratch_size, const u32 scratch_count);
/*
 * Alloc and initiate thread. On success, return valid address. On failure, Fatally cleanup and exit.
 * The thread's index is the next one in clone order (< DS_THREAD_COUNT_MAX), assigned before the thread starts;
 * the stack size is rounded up to whole pages.
 */
ds_Thread * ds_ThreadClone(struct arena *mem, void (*start)(ds_Thread *), void *args, const u64 stack_size, const u64 frame_size, const u64 scratch_size, const u32 scratch_count);
/* Exit calling thread (its g_tl_self becomes NULL) */
void		ds_ThreadExit(void);
/* Wait for given thread to finish execution; once per thread (Windows closes its handle) */
void 		ds_ThreadWait(const ds_Thread *thr);
/* Get return value address */
void *		ds_ThreadReturnValue(const ds_Thread *thr);
/* Get return value size */
u64		    ds_ThreadReturnValueSize(const ds_Thread *thr);
/* Retrieve thread function arguments */
void *  	ds_ThreadArguments(const ds_Thread *thr);
/* Return thread id: set by the thread itself as it starts, so read it from elsewhere only after that */
tid		    ds_ThreadTid(const ds_Thread *thr);
/* Return thread tid of caller */
tid 		ds_ThreadSelfTid(void);
/* Return index of thread: 0 for the master thread, then 1, 2, ... in clone order */
u32		    ds_ThreadIndex(const ds_Thread *thr);
/* Return index of caller */ 
u32		    ds_ThreadSelfIndex(void);
/* Yield the thread's processor time to the OS. */
void        ds_ThreadYield(void);

#ifdef __cplusplus
} 
#endif

#endif
