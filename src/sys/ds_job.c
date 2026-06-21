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

#include <stdlib.h>

#include "ds_random.h"
#include "ds_job.h"


static void ds_WSDequeStaticAssert(void)
{
    ds_StaticAssert((u64) &((struct ds_WSDeque *)0)->a_top == 0, "");
    ds_StaticAssert((u64) &((struct ds_WSDeque *)0)->pad1 == 8, "");
    ds_StaticAssert((u64) &((struct ds_WSDeque *)0)->a_bottom == 64, "");
    ds_StaticAssert((u64) &((struct ds_WSDeque *)0)->pad2 == 72, "");
    ds_StaticAssert((u64) &((struct ds_WSDeque *)0)->a_mem_count == 128, "");
    ds_StaticAssert((u64) &((struct ds_WSDeque *)0)->owner == 132, "");
    ds_StaticAssert((u64) &((struct ds_WSDeque *)0)->pad3 == 136, "");
    ds_StaticAssert(sizeof(struct ds_WSDeque) == 27*DS_CACHE_LINE, "Unexpected size of ds_WSDeque");
}

void ds_WSDequeAlloc(struct ds_WSDeque *deque, const u32 owner, const u64 len)
{
    ds_Assert(len);

    deque->owner = owner;
    deque->mem[0].len = PowerOfTwoCeil(len);
    deque->mem[0].mask = deque->mem[0].len - 1;
    deque->mem[0].id = ds_Alloc(&deque->mem[0].mem, deque->mem[0].len*sizeof(deque->mem[0].id[0]), NO_HUGE_PAGES);
    if (!deque->mem[0].id) 
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to reallocate memSlot in ds_Alloc, exiting.");
		FatalCleanupAndExit();
	}

    /* We begin counting from 1 and not 0 in order to prevent an underflow bug in the original paper within PopBottom. */
    AtomicStoreRlx64(&deque->a_bottom, 1);
    AtomicStoreRel32(&deque->a_mem_count, 1);
    AtomicStoreRel64(&deque->a_top, 1);
}

static void ds_WSDequeRealloc(struct ds_WSDeque *deque)
{
    const u32 local_mem_count = AtomicLoadRlx32(&deque->a_mem_count);
    ds_Assert(local_mem_count < 32);
    ds_Assert(deque->mem[local_mem_count-1].len < (u64) 1 << 63);

    deque->mem[local_mem_count].len = 2*deque->mem[local_mem_count - 1].len;
    deque->mem[local_mem_count].mask = deque->mem[local_mem_count].len - 1;
    deque->mem[local_mem_count].id = ds_Alloc(&deque->mem[local_mem_count].mem 
                                              ,deque->mem[local_mem_count].len*sizeof(deque->mem[0].id[0])
                                              ,NO_HUGE_PAGES);
    if (!deque->mem[local_mem_count].id)
    {
		LogString(T_SYSTEM, S_FATAL, "Failed to reallocate memSlot in ds_Realloc, exiting.");
		FatalCleanupAndExit();
    }

    const u64 local_bottom = AtomicLoadRlx64(&deque->a_bottom);
    const u64 local_top = AtomicLoadRlx64(&deque->a_top);
    const u64 new_mask = deque->mem[local_mem_count].mask;
    const u64 old_mask = deque->mem[local_mem_count - 1].mask;
    for (u64 i = local_top; i < local_bottom; ++i)
    {
        AtomicStoreRlx32(&deque->mem[local_mem_count].id[i & new_mask]
                        , deque->mem[local_mem_count - 1].id[i & old_mask]);
    }

    AtomicStoreRel32(&deque->a_mem_count, local_mem_count+1);
}

void ds_WSDequeDealloc(struct ds_WSDeque *deque)
{
    const u32 local_mem_count = AtomicLoadAcq32(&deque->a_mem_count);
    if (1 < local_mem_count)
    {
        Log(T_SYSTEM, S_WARNING, "ds_WSDeque has been reallocated, largest size used: %lu. Since memory is not reclaimed, consider tuning the initial size.", deque->mem[local_mem_count-1].len);
    }

    for (u32 i = 0; i < local_mem_count; ++i)
    {
        ds_Free(&deque->mem[i].mem);
    }
}

void ds_WSDequePushBottom(struct ds_WSDeque *deque, const u32 id)
{
    ds_Assert(deque->owner == ds_ThreadSelfIndex());

    u32 local_mem_count = AtomicLoadRlx32(&deque->a_mem_count);
    const u64 local_bottom = AtomicLoadRlx64(&deque->a_bottom);
    /* Note: Paper uses Acquire here, not sure why as only a_top itself is modified by guest threads. */
    const u64 local_top = AtomicLoadRlx64(&deque->a_top);
    if (local_bottom - local_top >= deque->mem[local_mem_count-1].len)
    {
        ds_WSDequeRealloc(deque);
        local_mem_count = AtomicLoadRlx32(&deque->a_mem_count);
    }

    const u32 index = local_bottom & deque->mem[local_mem_count-1].mask;
    AtomicStoreRlx32(&deque->mem[local_mem_count-1].id[index], id);
    AtomicStoreRel64(&deque->a_bottom, local_bottom+1);
}

u32 ds_WSDequeTryPopBottom(struct ds_WSDeque *deque)
{
    ds_Assert(deque->owner == ds_ThreadSelfIndex());

    const u64 local_bottom_t0 = AtomicLoadRlx64(&deque->a_bottom) - 1;
    const u32 local_mem_count = AtomicLoadRlx32(&deque->a_mem_count);

    /*
     * In order to get a correct lower-bound of elements in the snapshot at time t0, we must enforce
     * the instruction (and memory) order
     *              (1) store: a_bottom = local_bottom_t0 
     *              (2)  load: local_top_t1 = a_top;
     *
     * The lower-bound of elements in the deque at t0 then becomes
     *
     *      local_bottom_t0 - local_top_t0 >= local_bottom_t0 - local_top_t1, 
     *
     * which enables the safe CAS-less pop for the common case of a filled Deque.
     * TODO: Seem to be some crazy links here with stealer's Fence in PopBottom.
     */
    AtomicStoreRlx64(&deque->a_bottom, local_bottom_t0);
    ds_MemoryFenceSeqCst;
    u64 local_top_t1 = AtomicLoadRlx64(&deque->a_top);

    u32 id = DS_WSDEQUE_INVALID;
    if (local_top_t1 <= local_bottom_t0)
    {
        const struct ds_WSDequeMem *mem = deque->mem + local_mem_count - 1;
        id = AtomicLoadRlx32(&mem->id[local_bottom_t0 & mem->mask]);
        if (local_top_t1 == local_bottom_t0)
        {
            /* We contend with stealers for the last element */
            //TODO why seq_cst on success?
            if (!AtomicCompareExchangeSeqCstRlx64(&deque->a_top, &local_top_t1, local_top_t1+1))
            {
                id = DS_WSDEQUE_INVALID;
            }
            AtomicStoreRlx64(&deque->a_bottom, local_bottom_t0 + 1);
        }
    }
    else
    {
        AtomicStoreRlx64(&deque->a_bottom, local_bottom_t0 + 1);
    }


    return id;
}

u32 ds_WSDequeTrySteal(struct ds_WSDeque *deque)
{
    u32 id = DS_WSDEQUE_INVALID;
    u64 local_top = AtomicLoadRlx64(&deque->a_top);
    /*
     * TODO Hmm.. me no understand the necessity but experts say...
     * Seem to be some crazy links here with the owner's Fence in PopBottom.
     */
    ds_MemoryFenceSeqCst;
    const u64 local_bottom = AtomicLoadAcq64(&deque->a_bottom);
    if (local_top < local_bottom)
    {
        /* 
         * At this point, owner may have allocated new memory so our acquisition of a_bottom 
         * may not be enough, thus we require a Aquire load here. 
         */
        const u32 local_mem_count = AtomicLoadAcq32(&deque->a_mem_count);
        const struct ds_WSDequeMem *mem = deque->mem + local_mem_count - 1;
        id = AtomicLoadRlx32(&mem->id[local_top & mem->mask]);
        //TODO why SeqCstRlx?
        if (!AtomicCompareExchangeSeqCstRlx64(&deque->a_top, &local_top, local_top+1))
        {
            id = DS_WSDEQUE_INVALID;
        }
    }

    return id;
}



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
