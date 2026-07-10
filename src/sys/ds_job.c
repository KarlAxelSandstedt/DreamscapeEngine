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


struct ds_JobScheduler *g_scheduler = NULL;

static void ds_WSDequeStaticAssert(void)
{
    ds_StaticAssert((u64) &((struct ds_WSDeque *)0)->a_top == 0, "");
    ds_StaticAssert((u64) &((struct ds_WSDeque *)0)->a_bottom == 64, "");
    ds_StaticAssert((u64) &((struct ds_WSDeque *)0)->a_mem_count == 128, "");
    ds_StaticAssert((u64) &((struct ds_WSDeque *)0)->owner == 132, "");
    ds_StaticAssert((u64) &((struct ds_WSDeque *)0)->to_publish == 136, "");
    ds_StaticAssert(sizeof(struct ds_WSDeque) == 27*DS_CACHE_LINE, "Unexpected size of ds_WSDeque");
}

static void ds_WorkerStaticAssert(void)
{
    ds_StaticAssert((u64) &((struct ds_Worker *)0)->thr == 0, "");
    ds_StaticAssert((u64) &((struct ds_Worker *)0)->a_mem_frame_switch < DS_CACHE_LINE, "");
    ds_StaticAssert(sizeof(struct ds_Worker) == DS_CACHE_LINE, "Unexpected size of ds_Worker");
}

static void ds_JobSchedulerStaticAssert(void)
{
    ds_StaticAssert((u64) &((struct ds_JobScheduler *)0)->worker == 0, "");
    ds_StaticAssert((u64) &((struct ds_JobScheduler *)0)->deque < DS_CACHE_LINE, "");
    ds_StaticAssert((u64) &((struct ds_JobScheduler *)0)->seed_deque < DS_CACHE_LINE, "");
    ds_StaticAssert((u64) &((struct ds_JobScheduler *)0)->phase < DS_CACHE_LINE, "");
    ds_StaticAssert((u64) &((struct ds_JobScheduler *)0)->worker_count < DS_CACHE_LINE, "");
    ds_StaticAssert((u64) &((struct ds_JobScheduler *)0)->a_running < DS_CACHE_LINE, "");
    ds_StaticAssert((u64) &((struct ds_JobScheduler *)0)->steal_attempts < DS_CACHE_LINE, "");
    ds_StaticAssert((u64) &((struct ds_JobScheduler *)0)->jobs_are_available == DS_CACHE_LINE, "");
    ds_StaticAssert((u64) &((struct ds_JobScheduler *)0)->a_workers_waiting == 2*DS_CACHE_LINE, "");
    ds_StaticAssert((u64) &((struct ds_JobScheduler *)0)->a_seeds_remaining == 3*DS_CACHE_LINE, "");
    ds_StaticAssert((u64) &((struct ds_JobScheduler *)0)->phase_completed == 4*DS_CACHE_LINE, "");
    ds_StaticAssert(sizeof(struct ds_JobScheduler) == 5*DS_CACHE_LINE, "Unexpected size of ds_JobScheduler");
}

static void ds_JobPhaseStaticAssert(void)
{
    ds_StaticAssert((u64) &((struct ds_JobPhase *)0)->a_jobs_remaining == 64, "");
    ds_StaticAssert(sizeof(struct ds_JobPhase) == 2*DS_CACHE_LINE, "Unexpected size of ds_JobPhase");
}

static void ds_PaddedCounterStaticAssert(void)
{
    ds_StaticAssert((u64) &((struct ds_PaddedCounter *)0)->a_counter == 0, "");
    ds_StaticAssert(sizeof(struct ds_PaddedCounter) == DS_CACHE_LINE, "Unexpected size of ds_PaddedCounter");
}

void ds_WSDequeAlloc(struct ds_WSDeque *deque, const u32 owner, const u64 len)
{
    ds_Assert(len);

    deque->owner = owner;
    deque->to_publish = 0;
    deque->mem[0].len = PowerOfTwoCeil(len);
    deque->mem[0].mask = deque->mem[0].len - 1;
    deque->mem[0].id = ds_Alloc(&deque->mem[0].mem, deque->mem[0].len*sizeof(ds_JobId), NO_HUGE_PAGES);
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
                                              ,deque->mem[local_mem_count].len*sizeof(ds_JobId)
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
    for (u64 i = local_top; i < local_bottom + deque->to_publish; ++i)
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

void ds_WSDequePushBottom(struct ds_WSDeque *deque, const ds_JobId id)
{
    ds_Assert(deque->owner == ds_ThreadSelfIndex() || 0 == ds_ThreadSelfIndex());

    u32 local_mem_count = AtomicLoadRlx32(&deque->a_mem_count);
    const u64 local_bottom = AtomicLoadRlx64(&deque->a_bottom);
    /* Note: Paper uses Acquire here, not sure why as only a_top itself is modified by guest threads. */
    const u64 local_top = AtomicLoadRlx64(&deque->a_top);
    if (deque->to_publish + local_bottom - local_top >= deque->mem[local_mem_count-1].len)
    {
        ds_WSDequeRealloc(deque);
        local_mem_count = AtomicLoadRlx32(&deque->a_mem_count);
    }

    const u32 index = (deque->to_publish + local_bottom) & deque->mem[local_mem_count-1].mask;
    deque->to_publish += 1;
    AtomicStoreRlx32(&deque->mem[local_mem_count-1].id[index], id);
}

void ds_WSDequePublish(struct ds_WSDeque *deque)
{
    const u64 local_bottom = AtomicLoadRlx64(&deque->a_bottom);
    AtomicStoreRel64(&deque->a_bottom, local_bottom + deque->to_publish);
    deque->to_publish = 0;
}

ds_JobId ds_WSDequeTryPopBottom(struct ds_WSDeque *deque)
{
    ds_Assert(deque->owner == ds_ThreadSelfIndex());
    ds_Assert(deque->to_publish == 0);

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

    ds_JobId id = DS_JOB_ID_EMPTY;
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
                id = DS_JOB_ID_NULL;
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

ds_JobId ds_WSDequeTrySteal(struct ds_WSDeque *deque)
{
    u32 id = DS_JOB_ID_NULL;
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
            id = DS_JOB_ID_NULL;
        }
    }

    return id;
}

void ds_WorkerRunJob(const ds_JobId job)
{
    struct ds_WSDeque *deque = g_scheduler->deque + ds_ThreadSelfIndex();
    const u32 job_diff = g_scheduler->phase->dispatch(job);

    /* TODO: If this is costly, maybe we can skip this while running tasks in own deque, and do 
     * a batch update before/after stealing. Trade-off becomes early publishing vs. less updates a_remaining
     */
    const u32 local_remaining = ds_JobPhaseAddFetchRemaining(g_scheduler->phase, job_diff);
    if (deque->to_publish)
    {
        ds_WSDequePublish(deque);
    }

    if (local_remaining == 0)
    {
        SemaphorePost(&g_scheduler->phase_completed);
    }
}

void ds_TrySeedAndRunJobs(struct ds_Worker *w, const u32 thread)
{
    ProfZone;

    if (AtomicLoadRlx32(&w->a_mem_frame_switch))
    {
        AtomicStoreRlx32(&w->a_mem_frame_switch, 0);
        ArenaSwitchAndFlushFrame();
    }

    /* When we get here, we know for sure that the deque we own is empty. We check if there are any special
     * seed tasks to grab, and seed of deque if that is the case. */

    //TODO(Simplification): Instead of using a separate Deque, we could just use a atomic counter, since all
    //seeds are assumed to be published immediately?
    //const u32 local_seed_next = AtomicFetchAddRlx32(&a_seeds_next);
    //if (local_seed_next < a_seeds_count)
    //{
    //    ds_WorkerRunJob(g_scheduler->phase->seed_job[local_seed_next]);
    //}
    
    /* (1) Seed and run as many jobs in owned Deque as possible */
    u32 job;
    u32 local_seeds_remaining = AtomicLoadRlx32(&g_scheduler->a_seeds_remaining);
    while (local_seeds_remaining)
    {
        if (AtomicCompareExchangeRlxRlx32(&g_scheduler->a_seeds_remaining, &local_seeds_remaining, local_seeds_remaining - 1))
        {
            while ((job = ds_WSDequeTrySteal(g_scheduler->seed_deque)) == DS_JOB_ID_NULL);
            ds_WorkerRunJob(job);

            while ((job = ds_WSDequeTryPopBottom(g_scheduler->deque + thread)) != DS_JOB_ID_EMPTY)
            {
                if (job != DS_JOB_ID_NULL)
                {
                    ds_WorkerRunJob(job);
                }
            }
        }
    }

    /* No more seed tasks, try to steal. If we succeed in stealing a task, we check our own deque again for work. */
RUN_JOBS:
    while ((job = ds_WSDequeTryPopBottom(g_scheduler->deque + thread)) != DS_JOB_ID_EMPTY)
    {
        if (job != DS_JOB_ID_NULL)
        {
            ds_WorkerRunJob(job);
        }
    }

    for (u32 i = 0; i < g_scheduler->steal_attempts; ++i)
    {
        const u32 index = RngU64Range(0, g_scheduler->worker_count-1);
        if (index != thread && (job = ds_WSDequeTrySteal(g_scheduler->deque + index)) != DS_JOB_ID_NULL)
        {
            ds_WorkerRunJob(job);
            goto RUN_JOBS;
        }
    }

    ProfZoneEnd;
}

void ds_WorkerMain(ds_Thread *thr)
{
	struct ds_Worker *w = ds_ThreadArguments(thr);
	ThreadXoshiro256InitSequence();

	while (AtomicLoadAcq32(&g_scheduler->a_running) == 0);

	w->thr = thr;
	AtomicFetchAddSeqCst32(&g_scheduler->a_running, 1);
	LogString(T_SYSTEM, S_NOTE, "ds_Worker setup fished");

    const u32 index_self = ds_ThreadSelfIndex();
	while (AtomicLoadRlx32(&g_scheduler->a_running))
	{
		/* No more work, we go to sleep and wait until we aquire new work.
         *
         * TODO:
         * Optimization: If sempahore turns out to be to much overhead explore 
         *      - futexes (Linux)
         *      - WaitOnAddress + WakeByAddressSingle (Windows)
		 */
        AtomicFetchAddRlx32(&g_scheduler->a_workers_waiting, 1);
		SemaphoreWait(&g_scheduler->jobs_are_available);
        AtomicFetchSubRlx32(&g_scheduler->a_workers_waiting, 1);

        ds_TrySeedAndRunJobs(g_scheduler->worker + index_self, index_self);
	}
}

void ds_MasterRunAvailableJobs(void)
{
    ds_Assert(ds_ThreadSelfIndex() == 0);

    ds_TrySeedAndRunJobs(g_scheduler->worker + 0, 0);
}

void ds_JobSchedulerInit(struct arena *mem_persistent, const u32 thread_count, const u64 stacksize, const u64 framesize, const u64 scratchsize, const u32 scratch_count, const u64 initial_deque_size)
{
	Log(T_SYSTEM, S_NOTE, "ds_JobScheduler worker count: %u", thread_count);

    g_scheduler = ArenaPushAligned(mem_persistent, sizeof(struct ds_JobScheduler), DS_CACHE_LINE);
    g_scheduler->worker_count = thread_count;
	g_scheduler->worker = ArenaPushAligned(mem_persistent, thread_count*sizeof(struct ds_Worker), DS_CACHE_LINE);	
    g_scheduler->deque = ArenaPushAligned(mem_persistent, thread_count*sizeof(struct ds_WSDeque), DS_CACHE_LINE);
    g_scheduler->seed_deque = ArenaPushAligned(mem_persistent, sizeof(struct ds_WSDeque), DS_CACHE_LINE);
    g_scheduler->phase = NULL;
    g_scheduler->steal_attempts = (4*thread_count < 64)
                                    ? 4*thread_count
                                    : 64;

    AtomicStoreRlx32(&g_scheduler->a_running, 0);
	SemaphoreInit(&g_scheduler->jobs_are_available, 0);
    SemaphoreInit(&g_scheduler->phase_completed, 0);
    ds_WSDequeAlloc(g_scheduler->seed_deque, 0, initial_deque_size);
	for (u32 i = 0; i < thread_count; ++i)
	{
        ds_WSDequeAlloc(g_scheduler->deque + i, i, initial_deque_size);
	}

	/* NOTE: worker 0: reserved for main thread */
    g_scheduler->worker[0].thr = g_tl_self;
	for (u32 i = 1; i < thread_count; ++i)
	{
		ds_ThreadClone(mem_persistent, ds_WorkerMain, g_scheduler->worker + i, stacksize, framesize, scratchsize, scratch_count);
	}

    AtomicStoreRlx32(&g_scheduler->a_seeds_remaining, 0);
    AtomicStoreRlx32(&g_scheduler->a_workers_waiting, 0);
	AtomicFetchAddRel32(&g_scheduler->a_running, 1);

	while ((u32) AtomicLoadSeqCst32(&g_scheduler->a_running) < g_scheduler->worker_count);
}

void ds_JobSchedulerShutdown(void)
{
    AtomicStoreRlx32(&g_scheduler->a_running, 0);
	for (u32 i = 1; i < g_scheduler->worker_count; ++i)
	{
        SemaphorePost(&g_scheduler->jobs_are_available);
	}

	for (u32 i = 1; i < g_scheduler->worker_count; ++i)
	{
		ds_ThreadWait(g_scheduler->worker[i].thr);
	}

	for (u32 i = 0; i < g_scheduler->worker_count; ++i)
	{
	    ds_WSDequeDealloc(g_scheduler->deque + i);
	}
	ds_WSDequeDealloc(g_scheduler->seed_deque);

	SemaphoreDestroy(&g_scheduler->jobs_are_available);
    SemaphoreDestroy(&g_scheduler->phase_completed);
}

void ds_JobSchedulerFrameClear(void)
{
	for (u32 i = 0; i < g_scheduler->worker_count; ++i)
	{
		AtomicStoreRel32(&g_scheduler->worker[i].a_mem_frame_switch, 1);
	}
}

void ds_JobPhaseAlloc(struct arena *mem, struct ds_JobPhase *phase, const u32 job_type_count, ds_JobDispatchFunction dispatch)
{
    AtomicStoreRlx32(&phase->a_jobs_remaining, 0);
    phase->next = ArenaPushAligned(mem, job_type_count * sizeof(struct ds_PaddedCounter), DS_CACHE_LINE);
    phase->next_len = job_type_count;
    phase->dispatch = dispatch;
}

void ds_JobPhaseBegin(struct ds_JobPhase *phase)
{
    ds_Assert(0 == SemaphoreTryWait(&g_scheduler->phase_completed));
    ds_Assert(g_scheduler->phase == NULL);

    g_scheduler->phase = phase;
    ds_Assert(AtomicLoadRlx32(&phase->a_jobs_remaining) == 0);
    AtomicStoreRlx32(&phase->a_jobs_remaining, 0);
    for (u32 i = 0; i < phase->next_len; ++i)
    {
        AtomicStoreRlx32(&phase->next[i].a_counter, 0); 
    }
}

void ds_JobPhaseEnd(void)
{
    SemaphoreWait(&g_scheduler->phase_completed);
    const u32 local_remaining = AtomicLoadAcq32(&g_scheduler->phase->a_jobs_remaining);
    ds_Assert(0 == SemaphoreTryWait(&g_scheduler->phase_completed));
    ds_Assert(0 == local_remaining);
    g_scheduler->phase = NULL;
}

u32 ds_JobPhaseAddFetchRemaining(struct ds_JobPhase *phase, const u32 new_jobs)
{
    return AtomicAddFetchRel32(&phase->a_jobs_remaining, new_jobs);
}

u32 ds_JobPhaseReserve(struct ds_JobPhase *phase, const u32 job_type, const u32 new_jobs_count)
{
    ds_Assert(job_type < phase->next_len);
    return AtomicFetchAddRlx32(&phase->next[job_type].a_counter, new_jobs_count);
}
