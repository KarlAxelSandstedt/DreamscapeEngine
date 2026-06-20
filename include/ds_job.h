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

#ifndef __DS_JOB_H__
#define __DS_JOB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_base.h"


/*
ds_WSDeque
==========
ABP-based Deque by David Chase and Lev (Dynamic Circular Work-Stealing Deque). The first paper does not work with
weak memory models, and the paper (Correct adn Efficient Work-Stealing for Weak Memory Models) present versions
for such architectures. Note that the paper contains an integer over/underflow which we must fix.

Internals:
For simplicity, whenever a deque is reallocated by its owner, the old memory is not freed. The cost of this is small,
and it greatly simplifies the synchronization between threads. Since we double the size on each reallocation, the
wasted memory is upper bounded by the current memory usage:

        2^0 + 2^1 + ... 2^(n-1) = 2^n - 1 <= 2^n.

Ideally, no reallocations will ever occur, since the developer should tune the initial size to be reasonable for
the program that is being built. Note that ds_WSDequeDealloc does not only free the current memory, but also all
previously allocated memory slots of the deque. 
*/

/*
ds_WSDequeMem
=============
Helper for stealing threads to atomically load the memory and mask atomically. 
*/
struct ds_WSDequeMem
{
    u32 *               id; 
    u64                 len;
    u64                 mask;
    struct ds_MemSlot   mem;
};

#define DS_WSDEQUE_INVALID  U32_MAX

struct ds_WSDeque
{
    u64                     a_top;                                              /* Shared index for stealing */
    u8                      pad1[DS_CACHE_LINE - sizeof(u64)];
    u64                     a_bottom;                                           /* Thread owned index for pushing */
    u8                      pad2[DS_CACHE_LINE - sizeof(u64)];
    u32                     a_mem_count;
    u32                     owner;
    u8                      pad3[DS_CACHE_LINE - 2*sizeof(u32)];
    struct ds_WSDequeMem    mem[32];
};

/* Allocate and initialize a new Deque. */
void    ds_WSDequeAlloc(struct ds_WSDeque *deque, const u32 owner, const u64 len);
/* Deallocate Deque. */
void    ds_WSDequeDealloc(struct ds_WSDeque *deque);
/* Push an id at the bottom of the Deque. WARNING: Only to be used by the owner of the deque. */
void    ds_WSDequePushBottom(struct ds_WSDeque *deque, const u32 id);
/* Try pop the bottom of Deque. On success, the id is returned, otherwise DS_WSDEQUE_INVALID is returned. */
u32     ds_WSDequeTryPopBottom(struct ds_WSDeque *deque);
/* Try pop the top of Deque. On success, the id is returned, otherwise DS_WSDEQUE_INVALID is returned. */
u32     ds_WSDequeTrySteal(struct ds_WSDeque *deque);





#include "fifo_spmc.h"


/* NOTE: WE ASSUME MASTER THREAD/WORKER HAS ID AND INDEX 0. */

extern struct task_context *g_task_ctx;

typedef void (*TASK)(void *);

struct worker
{
	//TODO Cacheline alignment 
	struct arena	mem_frame;		/* Cleared at start of every frame */	
	dsThread *	thr;
	u32 		a_mem_frame_clear;	/* atomic sync-point: if set, on next task run flush mem_frame. */
};

/* Task bundle: set of tasks commited at the same time. */
struct task_bundle 
{
	semaphore 	bundle_completed;
	struct task *	tasks;
	u32 		task_count;
	u32 		a_tasks_left;
};

struct task_range 
{
	void *base;
	u64 count;
};

enum task_batch_type
{
	TASK_BATCH_BUNDLE,
	TASK_BATCH_STREAM,
};

struct task
{
	struct worker *executor;
	TASK task;
	void *input;			/* Possibly shared arguments between tasks */
	void *output;
	struct task_range *range; 	/* 	If task_range, Set if task is to run over a specific local 
					 * 	interval of range input.
					 */

	enum task_batch_type batch_type;
	void *batch;			/* pointer to bundle or stream.
					 * If task_bundle, if set, we keep track of when it is done.
					 * If task_stream, increment stream->a_completed at end.
					 * */
};

/* TODO Beware: Make sure to not false-share data between threads here, pad any structs if needed. */
struct task_context
{
	struct task_bundle bundle; /* TODO: Temporary */
	struct fifoSpmc *tasks;
	struct worker *workers;
	u32 worker_count;
};

/* Init task_context, setup threads inside task_main */
void 	task_context_init(struct arena *mem_persistent, const u32 thread_count);
/* Destory resources */
void 	task_context_destroy(struct task_context *ctx);
/* Clear any frame resources held by the task context and it's workers */
void	task_context_frame_clear(void);
/* main loop for slave workers */
void  	task_main(dsThread *thr);
/* master worker runs any available work */
void 	task_main_master_run_available_jobs(void);

/*********************** Task Streams ***********************/ 

/*
 * Simple lock-free data structure for continuously dispatching and keeping track of work. Every task dispatched
 * using api will increment a_completed on completion. 
 */
struct task_stream
{
	u32 a_completed;	/* atomic completed tasks counter */
	u32 task_count;		/* owned by main-thread 	  */
};

/* acquire resources (if any) */
struct task_stream *	task_stream_init(struct arena *mem);
/* Dispatch task for workers to immediately pick up */
void 			task_stream_dispatch(struct arena *mem, struct task_stream *stream, TASK func, void *args);
/* spin inside method until  a_completed == total */
void			task_stream_spin_wait(struct task_stream *stream);	
/* cleanup resources (if any) */
void			task_stream_cleanup(struct task_stream *stream);

/*********************** Task Bundles ***********************/ 

/* Split input range into split_count iterable intervals. task is then run iterably, and TODO: what do we return? */
struct task_bundle *	task_bundle_split_range(struct arena *mem_task_lifetime, TASK task, const u32 split_count, void *inputs, const u64 input_count, const u64 input_element_size, void *shared_arguments);
/* Blocked wait on bundle complete */
void			task_bundle_wait(struct task_bundle *bundle);
/* Clear and release task bundle for reallocation */
void			task_bundle_release(struct task_bundle *bundle);

#ifdef __cplusplus
}
#endif

#endif
