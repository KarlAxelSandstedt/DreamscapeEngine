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
ds_JobId
========
The ds_JobId is a tuple (tag(8), index(24)) where the tag defines what the type of the job is, and the index
is used to look up the job in the currently running ds_JobPhase (we assume only one phase in running at a time).
*/

typedef u32 ds_JobId;

#define DS_JOB_ID_EMPTY             U32_MAX
#define DS_JOB_ID_NULL              (U32_MAX-1)
#define DS_JOB_ID_TAG_MASK          ((u32) 0xff000000)
#define DS_JOB_ID_INDEX_MASK        ((u32) 0x00ffffff)

#define ds_JobIdTag(id)             ((id) >> 24)
#define ds_JobIdIndex(id)           ((id) & DS_JOB_ID_INDEX_MASK)
#define ds_JobIdInit(tag, index)    (((u32) (tag) << 24) | (index))

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
    ds_JobId *          id; 
    u64                 len;
    u64                 mask;
    struct ds_MemSlot   mem;
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

/* Allocate and initialize a new Deque. */
void        ds_WSDequeAlloc(struct ds_WSDeque *deque, const u32 owner, const u64 len);
/* Deallocate Deque. */
void        ds_WSDequeDealloc(struct ds_WSDeque *deque);
/* Push an unpublished id at the bottom of the Deque. WARNING: Only to be used by the owner of the deque. */
void        ds_WSDequePushBottom(struct ds_WSDeque *deque, const ds_JobId id);
/* Publish any pendings jobs */
void        ds_WSDequePublish(struct ds_WSDeque *deque);
/* 
 * Try pop the bottom of Deque. On success, the id is returned.
 * On a failed CAS, return DS_JOB_ID_NULL, 
 * otherwise DS_JOB_ID_EMPTY is returned. 
 */
ds_JobId    ds_WSDequeTryPopBottom(struct ds_WSDeque *deque);
/* Try pop the top of Deque. On success, the id is returned, otherwise DS_JOB_ID_NULL is returned. */
ds_JobId    ds_WSDequeTrySteal(struct ds_WSDeque *deque);


/*
ds_Worker
=========
TODO: We don't really use this at the moment...
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


/*
ds_JobPhase
===========
The ds_JobPhase struct is a helper for building parallel phases in the program. Ideally, all shared memory in a phase
can be pre-allocated. The master thread then spawns some "seeding" jobs, which is used to further spawn more jobs.
When all future jobs originating from the seeding jobs are done, the struct will signal its completion. 

Internals:

    Note 1: Sharing growable arrays of jobs is nasty business, so we require each ds_JobPhase to allocate the
            maximum number of each jobs per type up-front. Hopefully this scales well into the future, as it
            simplifies job allocation a damn lot at the cost of reserving additional memory. 


    Note 2: Each job is defined by its identfier, which breaks into (job_type, job_index). When building a new
            parallel phase, we would construct something like: 

                ds_CollisionJobPhase
                {
                    ds_JobPhase phase;
                    
                    job_type1 * jobs1;
                    ...
                    job_typeN * jobsN;
                }

            When a worker grabs a job, it can then look up the job by accessing the global phase. In addition
            to the struct, one would also have to provide

                ds_CollisionJobPhaseDispatch(ds_JobId id);

            which handles the job lookup and execution. The dispatch function must also return the number
            of jobs created within the job minus 1, i.e. If no jobs are created, U32_MAX is to be returned.
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

/* Allocate and Initalize phase resources. */
void    ds_JobPhaseAlloc(struct arena *mem, struct ds_JobPhase *phase, const u32 job_type_count, ds_JobDispatchFunction dispatch);
/* Reset phase resources and set the global ds_JobPhase in scheduler to phase. */
void    ds_JobPhaseBegin(struct ds_JobPhase *phase);
/* Block until all jobs in the phase are completed, and set the global ds_JobPhase in scheduler to NULL. */
void    ds_JobPhaseEnd(void);
/* Add new jobs to the remaining jobs counter, and return the new value. */
u32     ds_JobPhaseAddFetchRemaining(struct ds_JobPhase *phase, const u32 new_jobs_count);
/* Reserve a number of job slots of the given type, and return the starting index */
u32     ds_JobPhaseReserve(struct ds_JobPhase *phase, const u32 job_type, const u32 new_jobs_count);


/*
ds_JobScheduler
==============
ds_JobScheduler is the global scheduler of all jobs. It runs a single parallel phase at a time.


Usage Sketch:

    When using running a phase in the program, the user should setup seeding jobs, initital jobs whose sole
    purpose is the generate new jobs. This is essentially required as the master thread cannot easily wake 
    up worker threads and assign them these job spawning tasks. ds_JobScheduler supports this by having a 
    separate queue for seeding jobs. Any thread that wakes up begins by checking if any seeding jobs are to
    be executed, and if that is the case, executes the job. THis enables reasonable distribution of jobs during
    the phase.

    A Phase looks something like 

    ds_JobPhaseBegin(&collision_phase->job_phase)
    {
        (1) Reserve job memory
        collision_phase->seed_jobs = Arena(...)
        collision_phase->jobs1 = Arena(...)
        ...
        collision_phase->jobsN = Arena(...)

        (2) Setup seed jobs and push them onto the seed Deque
        for (job in collison_phase->seed_jobs) 
        { 
            ... 
            ds_WSDequePushBottom(g_scheduler->seed_deque, job_indentifier);
        }
        ds_JobPhaseAddFetchRemaining(&collision_phase->job_phase, collision_phase->seed_jobs_count);
        AtomicStoreRlx32(&g_scheduler->a_seeds_remaining, collision_phase->seed_jobs_count);

        (3) Publish the deque jobs and wake up threads
        ds_WSDequePublish(g_scheduler->seed_deque);
        for (u32 i = 1; i < g_scheduler->worker_count; ++i)
        {
            SemaphorePost(&g_scheduler->jobs_are_available);
        }

        (4) Let the master thread participate in the execution of jobs as well.
        ds_MasterRunAvailableJobs();

    }
    (5) Master waits until all remaining jobs in the phase have been completed. It then cleans up.
    ds_JobPhaseEnc()
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

/* Init Scheduler and setup threads inside ds_WorkerMain */
void 	ds_JobSchedulerInit(struct arena *mem_persistent, const u32 thread_count, const u64 stacksize, const u64 framesize, const u64 scratchsize, const u32 scratch_count, const u64 initial_deque_size);
/* Destory resources */
void 	ds_JobSchedulerShutdown(void);
/* Clear any frame resources held by the task context and it's workers */
void	ds_JobSchedulerFrameClear(void);

/*
ds_Spin
=======
ds_Spin is a spin-lock macro supporting exponential back-off + thread yielding.
*/

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

/*
ds_ParallelFor
==============
Spin-based Parallel-For helper. Our common-case is having multiple parallel-for loops in which 
loop i+1 depends on loop i; the ds_ParallelFor API is built to make this easy.

::: Usage ::: 

    Setup
    ===== 
    The user begins by allocating the set of dependent ds_ParallelFor structures by calling

        ds_ParallelForChainAlloc(...).

    This sets up internal values, including the dummy at the end. Afterwards, you must setup
    initalize each chain->parallel_for[i] structure with the number of indices of work that 
    is to be split into ranges, and how much work each range may represent: 

        ds_ParallelForInit(chain->parallel_for + i, work[i], work_per_range[i]);

    Note: chain->parallel_for[0].a_ready is set to 1 by default.

    Loop
    ====
    We provide some example:

        // Each color parallel-for loop depends on the previous color having
        // finished. This pattern is common and is the reason for why a chain
        // API is used.

        struct ds_ParallelForChain *chain;
        struct ds_ParallelFor *pf;
        u32 low, high;

        chain = &phase->pf_contact_init;
        {
            for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
            {
                // Thread enters parallel-for loop of color ci. range_index is the name and
                // is defined within the ds_ParallelFor macro. Within the parallel-for scope
                // it will be set to a unique range index of work that only the thread owns. 
                // Since we provided the number of indices of the parallel-for in the setup
                // stage, we can now map this range_index to an interval [low, high) of work.

                pf = chain->parallel_for + ci;
                ds_ParallelFor(pf, range_index)
                {
                    ds_ParallelForRange(&low, &high, pf, range_index);
                    ds_ContactConstraintInitRange(pipeline, ci, low, high);
                }
            }
        }

        // (Optional) Spin until all work in the chain has completed
        ds_ParallelForChainWait(chain);

::: Internals ::: 

    ds_ParallelForChain allocates a dummy at the end to support the following pattern:

        (1) Thread T1 reaches parallel-for Loop[i+1], spinning while acquiring Loop[i+1].a_ready
        (2) Thread T2 is working in parallel-for Loop[i]
        (3) Thread T2 finishes Loop[i], finally Releasing Loop[i+1].a_ready
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


/* Allocate chain of parallel-for structures and setup dummy at end of array. */
struct ds_ParallelForChain  ds_ParallelForChainAlloc(struct arena *frame, const u32 count);
/* Wait until the last parallel-for has finished in the chain. */
void                        ds_ParallelForChainWait(struct ds_ParallelForChain *chain);

/* Initialize the parallel-for according to the given parameters */
void                        ds_ParallelForInit(struct ds_ParallelFor *pf, const u32 index_count, const u32 range_index_count_max);
/* Return the interval [low, high) to be processed for the given range index. */
void                        ds_ParallelForRange(u32 *low, u32 *high, const struct ds_ParallelFor *pf, const u32 range_index);

/* Spin until the parallel-for is ready */
#define ds_ParallelFor(_pf_, _index_)       ds_ParallelForEx(_pf_, _index_, 1, 128)

/* 
 * Extended parallel-for: provide the max pause instructions emitted per spin (which doubles every spin), 
 * and the max pause instructions emitted before thread finally yielding.
 */
#define ds_ParallelForEx(_pf_, _index_, _max_pauses_per_spin_, _max_pauses_per_yield_)          \
for (u32 _index_, _completed_ = PFWait(_pf_, _max_pauses_per_spin_, _max_pauses_per_yield_); (((_index_) = PFNext(_pf_)) < (_pf_)->range_count) || PFComplete(_pf_, _completed_); ++_completed_)

static inline u32 PFWait(struct ds_ParallelFor *pf, const u32 max_pauses_per_spin, const u32 max_pauses_per_yield) 
{
    //ProfZone;

    /*
     * 2. Can we simplify the exponential pause stuff, or is it fine to only use << 1 increments?
     *
     * 3. We need to make sched_yield platform idependent,
     *
     *      PLATFORMS REQUIRED:
     *          
     *              Windows
     *              Linux
     *              WebAssembly + Emscripten
     *
     *      POSSIBLE FUTURE PLATFORMS
     *
     *              modern game consoles 
     */

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
