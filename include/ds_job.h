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
ds_job.h
========
The job system: one global scheduler (g_scheduler) runs one parallel phase at a time on its workers, thread
indices 0 .. thread_count - 1 (worker 0 is the main thread, the master; ds_init.h has the index rules). A phase
owns its jobs' data; a job is a ds_JobId (type tag, index) that the phase's dispatch function runs. Each worker
keeps its jobs in a work-stealing deque and steals from the others when it runs out; seed jobs, pushed by the
master into a shared seed deque, start a phase. ds_Init starts the scheduler (ds_PlatformApiInit).

Phases are complete examples in src/math/dynamics/ds_pipeline.c: the broad, narrow, solver, rebuild and removal
phases (ds_BroadJobPhase: ds_JobPhaseAlloc in the pipeline setup, ds_BroadJobPhaseDispatch, the phase between
ds_JobPhaseBegin and ds_JobPhaseEnd).

Sections: job ids; work-stealing deque; workers; phases; scheduler; spinning; parallel-for. Inline definitions and
macros at the bottom.
*/

#ifndef __DS_JOB_H__
#define __DS_JOB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_base.h"

/************************************** job ids **************************************/

/*
A ds_JobId is (tag: 8 bits, index: 24 bits): the tag is the job's type, the index finds the job in the running
phase. DS_JOB_ID_EMPTY and DS_JOB_ID_NULL are the deque's "no job" results (below), never job ids.
*/

typedef u32 ds_JobId;

#define DS_JOB_ID_EMPTY             U32_MAX
#define DS_JOB_ID_NULL              (U32_MAX-1)
#define DS_JOB_ID_TAG_MASK          ((u32) 0xff000000)
#define DS_JOB_ID_INDEX_MASK        ((u32) 0x00ffffff)

//#define ds_JobIdTag(id)             (tag of id)
//#define ds_JobIdIndex(id)           (index of id)
//#define ds_JobIdInit(tag, index)    (the id; tag < 256, index < 2^24)

/******************************** work-stealing deque ********************************/

/*
A worker's deque of job ids (Chase-Lev, with the orderings of Le et al. for weak memory models). Its owner pushes
ids at the bottom, publishes them, and pops them back (newest first); any thread steals from the top (oldest
first). It grows by doubling when full and keeps every older block until ds_WSDequeDealloc, so a stealer never
reads freed memory. Not reentrant for the owner.

- ds_WSDequePushBottom: owner only; the id is pending (invisible to stealers) until ds_WSDequePublish.
- ds_WSDequeTryPopBottom: owner only, nothing pending. DS_JOB_ID_EMPTY: the deque is empty; DS_JOB_ID_NULL: a
  stealer took the last id first (empty too).
- ds_WSDequeTrySteal: any thread. DS_JOB_ID_NULL: empty, or another thread won the race (try again or move on).
*/

struct ds_WSDequeMem
{
    ds_JobId *          id; 
    u64                 len;
    u64                 mask;
};

struct ds_WSDeque
{
    u64                     a_top;                                              /* Shared index for stealing */
    u8                      pad1[DS_CACHE_LINE - sizeof(u64)];
    u64                     a_bottom;                                           /* Thread owned index for pushing */
    u8                      pad2[DS_CACHE_LINE - sizeof(u64)];
    u32                     a_mem_count;
    u32                     owner;
    u64                     to_publish;
    u8                      pad3[DS_CACHE_LINE - 2*sizeof(u32) - sizeof(u64)];
    struct ds_WSDequeMem    mem[32];
};

/* Allocate the deque for owner (a thread index) with room for len ids (rounded up to a power of two); fatal if out of memory. */
void        ds_WSDequeAlloc(struct ds_WSDeque *deque, const u32 owner, const u64 len);
/* Free all of the deque's blocks; logs a warning if it ever grew (tune its initial size). */
void        ds_WSDequeDealloc(struct ds_WSDeque *deque);
void        ds_WSDequePushBottom(struct ds_WSDeque *deque, const ds_JobId id);
void        ds_WSDequePublish(struct ds_WSDeque *deque);
ds_JobId    ds_WSDequeTryPopBottom(struct ds_WSDeque *deque);
ds_JobId    ds_WSDequeTrySteal(struct ds_WSDeque *deque);

/************************************** workers **************************************/

/*
A scheduler worker: its thread, and a_mem_frame_switch, set by ds_JobSchedulerFrameClear: the worker switches and
flushes its frame arena before it next runs jobs. ds_WorkerMain is the loop of workers 1 .. thread_count - 1
(sleep until jobs are available, run seeds, own jobs, steals); the master runs jobs only inside a phase, through
ds_MasterRunAvailableJobs.
*/

struct ds_Worker
{
	ds_Thread *	    thr;
	u32 		    a_mem_frame_switch;	/* atomic sync-point: if set, on next task run flush mem_frame. */
    u8              pad[64 - sizeof(void *) - sizeof(u32)];
};

/* main loop for slave workers */
void  	ds_WorkerMain(ds_Thread *thr);
/* master worker runs any available work */
void 	ds_MasterRunAvailableJobs(void);

/************************************** phases ***************************************/

/*
A parallel phase: the user embeds struct ds_JobPhase in a struct holding all of the phase's job data, allocated up
front (each job type's maximum count; jobs are not allocated while the phase runs):

    struct ds_CollisionJobPhase
    {
        struct ds_JobPhase  phase;
        job_type1 *         jobs1;
        ...
    };

and gives ds_JobPhaseAlloc the dispatch function, which runs the job with the given id (looking it up through
g_scheduler->phase) and returns the number of jobs it created minus 1 (U32_MAX if none).

A phase on the master thread:

    ds_JobPhaseBegin(&p->phase);
    (1) set up the job data on the frame arena
    (2) push the seed jobs onto g_scheduler->seed_deque and reserve their indices, so that jobs of the same type
        spawned later (ds_JobPhaseReserve) get indices after them:
            ds_WSDequePushBottom(g_scheduler->seed_deque, ds_JobIdInit(SEED_JOB_TYPE, i)), for each seed i
            ds_JobPhaseReserve(&p->phase, SEED_JOB_TYPE, seed_count);
            AtomicStoreRlx32(&g_scheduler->a_seeds_remaining, seed_count);
            ds_JobPhaseAddFetchRemaining(&p->phase, seed_count);
    (3) publish the seeds and wake the workers:
            ds_WSDequePublish(g_scheduler->seed_deque);
            SemaphorePost(&g_scheduler->jobs_are_available), worker_count - 1 times
    (4) run jobs on the master too: ds_MasterRunAvailableJobs();
    (5) wait for the phase's jobs: ds_JobPhaseEnd();

A job spawns jobs by pushing them onto its own worker's deque (g_scheduler->deque + ds_ThreadSelfIndex()); they
are published after its dispatch returns.
*/

struct ds_PaddedCounter
{
    u32     a_counter;
    u8      pad[64 - sizeof(u32)];
};

typedef u32 (*ds_JobDispatchFunction)(const ds_JobId job);

struct ds_JobPhase
{
    ds_JobDispatchFunction      dispatch;

    struct ds_PaddedCounter *   next;       /* next[type].a_counter == Where next job of given type is pushed to */
    u32                         next_len;
    u8                          pad1[64 - 2*sizeof(void *) - sizeof(u32)];
    u32                         a_jobs_remaining;
    u8                          pad2[64 - sizeof(u32)];
};

/* Allocate phase resources on mem (job_type_count index counters) and set the dispatch function. */
void    ds_JobPhaseAlloc(struct arena *mem, struct ds_JobPhase *phase, const u32 job_type_count, ds_JobDispatchFunction dispatch);
/* Reset the phase's counters and make it g_scheduler's phase. Precondition: no phase running. */
void    ds_JobPhaseBegin(struct ds_JobPhase *phase);
/* Block until all jobs in the phase are completed, then clear g_scheduler's phase. */
void    ds_JobPhaseEnd(void);
/* Add new jobs to the remaining jobs counter (wrapping: U32_MAX subtracts one) and return the new value. */
u32     ds_JobPhaseAddFetchRemaining(struct ds_JobPhase *phase, const u32 new_jobs_count);
/* Reserve new_jobs_count job indices of the given type; returns the first. */
u32     ds_JobPhaseReserve(struct ds_JobPhase *phase, const u32 job_type, const u32 new_jobs_count);

/************************************* scheduler *************************************/

/*
The global scheduler: worker i owns deque[i]; seed_deque is owned by the master (worker 0). One phase runs at a
time (phase). Workers sleep on jobs_are_available; the last job of a phase posts phase_completed.
*/

struct ds_JobScheduler
{
	struct ds_Worker *  worker;
    struct ds_WSDeque * deque;              /* worker[i] owns deque[i] */
    struct ds_WSDeque * seed_deque;         /* Special deque used for seeding tasks */ 
    struct ds_JobPhase *phase;              /* Currently running phase, if any. */

    u32                 worker_count;
    u32                 a_running;
    u32                 steal_attempts;

    u8                  pad1[64 - 4*sizeof(void *) - 3*sizeof(u32)];

	semaphore 	        jobs_are_available;  
    u8                  pad2[64 - sizeof(semaphore)];

    u32                 a_workers_waiting;
    u8                  pad3[64 - sizeof(u32)];

    u32                 a_seeds_remaining;
    u8                  pad4[64 - sizeof(u32)];

    semaphore           phase_completed;
    u8                  pad5[64 - sizeof(semaphore)];
};

extern struct ds_JobScheduler *g_scheduler;

/*
Start the scheduler with thread_count workers: worker 0 is the calling thread, workers 1 .. thread_count - 1 are
cloned in order (so their thread indices match) with the given stack and arena sizes; every deque starts with
room for initial_deque_size ids. Returns once every worker has started. Called by ds_PlatformApiInit.
*/
void 	ds_JobSchedulerInit(struct arena *mem_persistent, const u32 thread_count, const u64 stacksize, const u64 framesize, const u64 scratchsize, const u32 scratch_count, const u64 initial_deque_size);
/* Stop and join the workers and free the deques. */
void 	ds_JobSchedulerShutdown(void);
/* Ask every worker to switch and flush its frame arena before it next runs jobs. */
void	ds_JobSchedulerFrameClear(void);

/************************************* spinning **************************************/

/*
ds_Spin spins while condition holds, with exponential back-off: the pauses per spin double up to
max_pauses_per_spin, and after max_pauses_per_yield pauses the thread yields (ds_ThreadYield). Both maxima > 0.
*/
//#define ds_Spin(condition, max_pauses_per_spin, max_pauses_per_yield)

/*********************************** parallel-for ************************************/

/*
A chain of parallel-for loops in which loop i + 1 depends on loop i (e.g. one loop per constraint graph color).
Every thread that enters a loop takes ranges of its index space until none are left; the last range to finish
releases the next loop.

    struct ds_ParallelForChain chain = ds_ParallelForChainAlloc(frame, count);
    for (u32 i = 0; i < count; ++i) { ds_ParallelForInit(chain.parallel_for + i, work[i], work_per_range[i]); }

    (on every participating thread)
    for (u32 ci = 0; ci < count; ++ci)
    {
        struct ds_ParallelFor *pf = chain.parallel_for + ci;
        ds_ParallelFor(pf, range_index)                 (waits until loop ci - 1 is done; range_index is
        {                                                these ranges' own, declared by the macro)
            u32 low, high;
            ds_ParallelForRange(&low, &high, pf, range_index);
            ... work on [low, high)
        }
    }
    ds_ParallelForChainWait(&chain);                    (optional: spin until the whole chain is done)

ds_ParallelForChainAlloc allocates count loops plus a terminating one on frame (fatal if it doesn't fit); loop 0 is
ready at once. ds_ParallelForInit splits index_count indices into ranges of at most range_index_count_max (> 0).
ds_ParallelForEx spins as ds_Spin while it waits; ds_ParallelFor uses 1 and 128.
*/

struct ds_ParallelFor
{
    u32 a_ready;
    u8  pad0[DS_CACHE_LINE - 4];
    u32 a_next;
    u8  pad1[DS_CACHE_LINE - 4];
    u32 a_completed;
    u8  pad2[DS_CACHE_LINE - 4];

    u32 index_count;
    u32 range_count;
    u32 range_index_count_max;
    u8  pad3[DS_CACHE_LINE - 12];
};

struct ds_ParallelForChain
{
    u32                     count;
    struct ds_ParallelFor * parallel_for;
};

struct ds_ParallelForChain  ds_ParallelForChainAlloc(struct arena *frame, const u32 count);
void                        ds_ParallelForChainWait(struct ds_ParallelForChain *chain);
void                        ds_ParallelForInit(struct ds_ParallelFor *pf, const u32 index_count, const u32 range_index_count_max);
/* Return the interval [low, high) of the given range index. */
void                        ds_ParallelForRange(u32 *low, u32 *high, const struct ds_ParallelFor *pf, const u32 range_index);

//#define ds_ParallelFor(pf, range_index)
//#define ds_ParallelForEx(pf, range_index, max_pauses_per_spin, max_pauses_per_yield)

/* Internal (the parallel-for macros): wait until pf is ready, take the next range index, count finished ranges. */
static inline u32 PFWait(struct ds_ParallelFor *pf, const u32 max_pauses_per_spin, const u32 max_pauses_per_yield);
static inline u32 PFNext(struct ds_ParallelFor *pf);
static inline u32 PFComplete(struct ds_ParallelFor *pf, const u32 complete_count);

/************************* job ids: macro definitions *************************/

#define ds_JobIdTag(id)             ((id) >> 24)
#define ds_JobIdIndex(id)           ((id) & DS_JOB_ID_INDEX_MASK)
#define ds_JobIdInit(tag, index)    (((u32) (tag) << 24) | (index))

/************************* spinning: macro definitions *************************/

#define ds_Spin(_spin_condition_, _max_pauses_per_spin_, _max_pauses_per_yield_)    \
do                                                                                  \
{                                                                                   \
    ds_Assert( (_max_pauses_per_spin_) );                                           \
    ds_Assert( (_max_pauses_per_yield_) );                                          \
    u32 _pause_count_ = 0;                                                          \
    u32 _pause_ = 1;                                                                \
    while ( (_spin_condition_) )                                                    \
    {                                                                               \
        ds_CpuPause(_pause_);                                                       \
                                                                                    \
        _pause_count_ += _pause_;                                                   \
        _pause_ <<= 1;                                                              \
                                                                                    \
        if ( (_max_pauses_per_spin_) < _pause_ )                                    \
        {                                                                           \
            _pause_ = (_max_pauses_per_spin_);                                      \
        }                                                                           \
                                                                                    \
        if( (_max_pauses_per_yield_) < _pause_count_ )                              \
        {                                                                           \
            _pause_count_ = 0;                                                      \
            ds_ThreadYield();                                                       \
        }                                                                           \
    }                                                                               \
} while (0)

/******************** parallel-for: inline and macro definitions ********************/

#define ds_ParallelFor(_pf_, _index_)       ds_ParallelForEx(_pf_, _index_, 1, 128)

#define ds_ParallelForEx(_pf_, _index_, _max_pauses_per_spin_, _max_pauses_per_yield_)          \
for (u32 _index_, _completed_ = PFWait(_pf_, _max_pauses_per_spin_, _max_pauses_per_yield_); (((_index_) = PFNext(_pf_)) < (_pf_)->range_count) || PFComplete(_pf_, _completed_); ++_completed_)

static inline u32 PFWait(struct ds_ParallelFor *pf, const u32 max_pauses_per_spin, const u32 max_pauses_per_yield)
{
    //ProfZone;

    ds_Spin(!AtomicLoadAcq32(&pf->a_ready), max_pauses_per_spin, max_pauses_per_yield);

    if (!pf->range_count)
    {
        AtomicStoreRel32(&(pf + 1)->a_ready, 1);
    }

    //ProfZoneEnd;

    return 0;
}


static inline u32 PFNext(struct ds_ParallelFor *pf)
{
    return AtomicFetchAddRlx32(&pf->a_next, 1);
}


static inline u32 PFComplete(struct ds_ParallelFor *pf, const u32 complete_count)
{
    if (complete_count)
    { 
        const u32 local_completed = AtomicAddFetchRel32(&pf->a_completed, complete_count);
        if (local_completed == pf->range_count)
        {
            AtomicLoadAcq32(&pf->a_completed);
            AtomicStoreRel32(&(pf + 1)->a_ready, 1);
        }
    }

    return 0;
}


#ifdef __cplusplus
}
#endif

#endif
