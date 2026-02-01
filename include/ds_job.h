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
