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

#include "ds_random.h"
#include "ds_job.h"

struct task_context t_ctx;
struct task_context *g_task_ctx = &t_ctx;

u32 a_startup_complete = 0;

static void worker_init(struct worker *w)
{
	w->mem_frame = ArenaAlloc1MB();
}

static void worker_exit(void *void_task)
{
	struct task *task = void_task;
	ds_ThreadExit();
}

static void task_run(struct task *task_info, struct worker *w)
{
	if (AtomicLoadAcq32(&w->a_mem_frame_clear))
	{
		ArenaFlush(&w->mem_frame);
		AtomicStoreRel32(&w->a_mem_frame_clear, 0);
	}

	task_info->executor = w;
	task_info->task(task_info);

	switch (task_info->batch_type)
	{
		case TASK_BATCH_BUNDLE:
		{
			/* ThreadSanitizer screams if we use anything less than ACQ_REL or SEQ_CST. Can the sanitizer not
			 * assume that the compiler won't reorder the native semaphore calls? 
			 * TODO: Investigate more.
			 */
			struct task_bundle *bundle = task_info->batch;
			if (AtomicSubFetchSeqCst32(&bundle->a_tasks_left, 1) == 0)
			{
				SemaphorePost(&bundle->bundle_completed);
			}
		} break;

		case TASK_BATCH_STREAM:
		{
			struct task_stream *stream = task_info->batch;
			AtomicAddFetchRel32(&stream->a_completed, 1);
		} break;
	}
}

void task_main(dsThread *thr)
{
	struct worker *w = ds_ThreadArguments(thr);
	ThreadXoshiro256InitSequence();

	while (AtomicLoadAcq32(&a_startup_complete) == 0);

	w->thr = thr;
	AtomicFetchAddSeqCst32(&a_startup_complete, 1);
	LogString(T_SYSTEM, S_NOTE, "task_worker setup finalized");

	while (1)
	{
		/* If there is work, we plow through it continuously */
		while (SemaphoreTryWait(&g_task_ctx->tasks->able_for_reservation))
		{
			task_run(FifoSpmcPop(g_task_ctx->tasks), w);
		}

		/* No more work, we go to sleep and wait until we aquire new work.
		 * Spurious wake ups may happen, so we keep this in a loop.
		 */
		while (!SemaphoreWait(&g_task_ctx->tasks->able_for_reservation));

		task_run(FifoSpmcPop(g_task_ctx->tasks), w);
	};
}

void task_main_master_run_available_jobs(void)
{
	struct worker *master = g_task_ctx->workers + 0;
	while (SemaphoreTryWait(&g_task_ctx->tasks->able_for_reservation))
	{
		task_run(FifoSpmcPop(g_task_ctx->tasks), master);
	}
}

static struct task_bundle task_bundle_init()
{
	struct task_bundle bundle = { 0 };
	SemaphoreInit(&bundle.bundle_completed, 0);
	return bundle;
}

static void task_bundle_destroy(struct task_bundle *bundle)
{
	SemaphoreDestroy(&bundle->bundle_completed);
}

void task_context_init(struct arena *mem_persistent, const u32 thread_count)
{
	//TODO 
	const u64 stack_size = 64*1024;
	#define TASK_MAX_COUNT 1024
	struct task_context ctx = 
	{ 
		.workers = NULL,
		.worker_count = thread_count,
	};

	Log(T_SYSTEM, S_NOTE, "Task system worker count: %u", thread_count);

	*g_task_ctx = ctx;
	g_task_ctx->bundle = task_bundle_init();
	g_task_ctx->workers = ArenaPush(mem_persistent, thread_count * sizeof(struct worker));	
	g_task_ctx->tasks = FifoSpmcInit(mem_persistent, TASK_MAX_COUNT);

	for (u32 i = 0; i < thread_count; ++i)
	{
		worker_init(g_task_ctx->workers + i);
	}

	/* NOTE: worker 0: reserved for main thread */
	for (u32 i = 1; i < thread_count; ++i)
	{
		ds_ThreadClone(mem_persistent, task_main, g_task_ctx->workers + i, stack_size);
	}

	AtomicStoreRel32(&a_startup_complete, 1);

	while ((u32) AtomicLoadSeqCst32(&a_startup_complete) < g_task_ctx->worker_count);
}

void task_context_frame_clear(void)
{
	for (u32 i = 0; i < g_task_ctx->worker_count; ++i)
	{
		AtomicStoreRel32(&g_task_ctx->workers[i].a_mem_frame_clear, 1);
	}
}

void task_context_destroy(struct task_context *ctx)
{
	struct task *exit_tasks = malloc(ctx->worker_count * sizeof(struct task));

	for (u32 i = 1; i < ctx->worker_count; ++i)
	{
		exit_tasks[i].task = &worker_exit;
		FifoSpmcPush(ctx->tasks, exit_tasks + i);
	}

	for (u32 i = 1; i < ctx->worker_count; ++i)
	{
		ds_ThreadWait(ctx->workers[i].thr);
	}

	for (u32 i = 0; i < ctx->worker_count; ++i)
	{
		ArenaFree1MB(&ctx->workers[i].mem_frame);
	}

	
	task_bundle_destroy(&ctx->bundle);
	FifoSpmcDestroy(ctx->tasks);
	free(exit_tasks);
}

struct task_bundle *task_bundle_split_range(struct arena *mem_task_lifetime, TASK task, const u32 split_count, void *inputs, const u64 input_count, const u64 input_element_size, void *shared_arguments)
{
	const u32 tasks_per_range = (u32) input_count / split_count;
	u32 extra_tasks = input_count % split_count;
	const u32 splits = (tasks_per_range) ? split_count : extra_tasks;

	if (!splits) { return NULL; }

	struct task_bundle *bundle = &g_task_ctx->bundle;
	struct task_range *range = ArenaPush(mem_task_lifetime, splits * sizeof(struct task_range));
	bundle->tasks = ArenaPush(mem_task_lifetime, splits * sizeof(struct task));
	bundle->task_count = splits;

	u64 offset = 0;
	for (u32 i = 0; i < splits; ++i)
	{
		bundle->tasks[i].task = task;
		bundle->tasks[i].input = shared_arguments;
		bundle->tasks[i].range = range + i;
		bundle->tasks[i].batch_type = TASK_BATCH_BUNDLE,
		range[i].count = tasks_per_range; 
		if (extra_tasks) 
		{ 
			extra_tasks -= 1;
			range[i].count += 1;
		}

		range[i].base = ((u8 *) inputs) + offset;
		offset += range[i].count * input_element_size;
		AtomicStoreRel64(&bundle->tasks[i].batch, bundle);
	}

	AtomicStoreRel32(&bundle->a_tasks_left, splits);
	/* Sync points, we release tasks->data, threads aquire tasks->data => threads will see all previous writes */
	for (u32 i = 0; i < splits; ++i)
	{
		FifoSpmcPush(g_task_ctx->tasks, bundle->tasks + i);
	}

	return bundle;
}

void task_bundle_wait(struct task_bundle *bundle)
{
	while (!SemaphoreWait(&bundle->bundle_completed));
}

void task_bundle_release(struct task_bundle *bundle)
{
	AtomicStoreRel32(&bundle->a_tasks_left, 0);
}

struct task_stream *task_stream_init(struct arena *mem)
{
	struct task_stream *stream = ArenaPush(mem, sizeof(struct task_stream));
	AtomicStoreRel32(&stream->a_completed, 0);
	stream->task_count = 0;

	return stream;
}

void task_stream_dispatch(struct arena *mem, struct task_stream *stream, TASK func, void *args)
{
	struct task *task = ArenaPush(mem, sizeof(struct task));
	task->task = func;
	task->input = args;
	task->batch_type = TASK_BATCH_STREAM;
	task->batch = stream;
	
	stream->task_count += 1;
	FifoSpmcPush(g_task_ctx->tasks, task);
}

void task_stream_spin_wait(struct task_stream *stream)
{
	while ((u32) AtomicLoadAcq32(&stream->a_completed) < stream->task_count);
}

void task_stream_cleanup(struct task_stream *stream)
{
	const u32 finished = (AtomicLoadAcq32(&stream->a_completed) == stream->task_count);
	ds_AssertString(finished, "Bad use of task stream, when (and only) the main thread enters task_stream_cleanup, all tasks must have been dispatched and completed.");
}
