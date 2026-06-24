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
TODO
*/

struct ds_Worker
{
	struct arena	mem_frame;		/* Cleared at start of every frame */	
	dsThread *	    thr;
	u32 		    a_mem_frame_clear;	/* atomic sync-point: if set, on next task run flush mem_frame. */
    u8              pad[64 - sizeof(u32)];
};

/* main loop for slave workers */
void  	ds_WorkerMain(dsThread *thr);
/* master worker runs any available work */
void 	ds_MasterRunAvailableJobs(void);


/*
ds_JobScheduler
==============
TODO


TODO
Whenever a thread pushes a job to its deque, and there are zero published jobs in the
scheduler, it may be a good time for the thread to signal for some thread to wake up
and begin working.


TODO
Idea: 
1. Master splits up ranges of tasks to generate.
2. Each range becomes a seed task, an ordinary task which sole job is to generate new jobs.
3. Seed tasks are published, threads may take and run any number of them. Ideally they all take 1.
4. The ds_WSDeques have now been successfully seeded with jobs, 

*/

struct ds_JobScheduler
{
	struct ds_Worker *  worker;
    struct ds_WSDeque * deque;              /* worker[i] owns deque[i] */
    struct ds_WSDeque * seed_deque;         /* Special deque used for seeding tasks */ 

    u32                 worker_count;
    u32                 a_running;
    struct ds_JobPhase *phase;              /* Currently running phase, if any. */
    u8                  pad1[64 - 4*sizeof(void *) - 2*sizeof(u32)];

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
void 	ds_JobSchedulerInit(struct arena *mem_persistent, const u32 thread_count, const u64 stacksize, const u64 initial_deque_size);
/* Destory resources */
void 	ds_JobSchedulerShutdown(void);
/* Clear any frame resources held by the task context and it's workers */
void	ds_JobSchedulerFrameClear(void);


/*
ds_JobPhase
===========
The ds_JobPhase struct is a helper for building parallel phases in the program. Ideally, all shared memory in a phase
can be pre-allocated. The master thread then spawns some "seeding" jobs, which is used to further spawn more jobs.
When all future jobs originating from the seeding jobs are done, the struct will signal its completion. 

//TODO remove when implemented
What are we solving?
    1. We are running a set of jobs, all identified by its ID (Most likely to be implemented as an index to a thread-safe parallel pool of task?). 
    2. Each job may **Possibly** be part of a ds_JobPhase. We assume for the rest of the points that this is the case.
    3. If it is, the difference in remaining jobs the jobs produced is added to the phase's a_jobs_remaining. 
    4. IF (a_jobs_remaining == 0)
        THEN no thread is doing work anymore, and all future phase jobs must already have been reported in, so
             the phase is finished. these two variables must be atomically loaded in the correct order as well.
    6. The last thread to finish its work signal to the phase->completed_sem semaphore, which only the master thread
       may be waiting on.
    7. The master may do any cleanup and finish the phase.

Internals:

    Issue 1: We must first solve the issue of sharing memory between threads. The owner of some shared memory
             (job arrays in or context) must not grow the array, since that would require careful synchronization
             with guest threads. The obvious choice here is to allocate all shared memory up-front for the phase; 
             This requires us to know before-hand upper bounds of memory usage that aren't to large. The benefit
             is the simplicty, but we may run into issues with false-sharing since threads will randomly access
             and write to the shared memory of tasks.

             In the second case, while we still have to allocate all memory up-front, guest threads will at least
             not invalidate owner's cache-lines, as only the owner would write tasks to its own memory, while guests
             may only read it. Unknown if this would be worth it. 

             In the end, It is probably best to go with the simple solution of pre-allocated shared arrays. False
             sharing wrong task writing and access should be small in a reasonably constructed pipeline; if not, is
             is probably a signal that the tasks are to fine-grained. We continue with the first case.

    Issue 2: As all we get is an index identifier, we cannot know where this index is ment to be used. Suppose we
             wish to chain a set of dependent jobs in a phase. Then, the indices (i1, i2, ..., iN) of these dependent
             jobs would most likely index different arrays. How would the thread know which to index? One could expand
             the deque indentifiers by a tag

                        id = ( TAG(n) | INDEX(m) ) 
            
             which would allow the user to have 2^m tasks per job-type, and 2^n job-types. 

             For example, narrowphase = 0, broadphase = 1,

             In master:
                          jobs_broadphase  = ...
                          jobs_narrowphase = ...

             In RunJob:
                          const tag = id::TAG
                          const index = id::INDEX

                          if (tag == 0)
                            job = job_broadphase[index]
                          else if (tag == 1)
                            job = job_narrowphase[index]


             This seems to be a reasonable approach, and is what the library implements. If we don't require tags 
             in some phases, we can simply ignore them.

    Issue 3: All attempts at generalizing parallel phases into ds_JobPhase seem futile, as general ds_Job structure
             will become bloated, and unnecessarily annoying to work with (we will essentially be enforced to work
             with void * arguments always, and so on, in exchange for almost no benefit at all. A possible solution
             is to radically simplify ds_JobPhase to contain enough information to hold any possible combination
             of job types. Here is an example that illustrates the point:

                enum ds_JobType
                {
                   DS_JOB_BROADPHASE   = 0
                   DS_JOB_NARROWPHASE  = 1
                   ...
                   DS_JOB_INVALID      = N-1
                   DS_JOB_COUNT        = N
                }

                ds_StaticAssert(DS_JOB_COUNT <= Maximum Tag(n) value in job identifier)

                struct ds_PaddedCount
                {
                    u32 a_count;
                    u8 pad[cache_line_size - 4]
                }

                struct ds_JobPhase
                {
                    void *job_arrays_by_type[DS_JOB_COUNT];                 
                    u8 pad[cache_line_size - 4]
                    struct ds_PaddedCount next_by_type[DS_JOB_COUNT];

                    struct ds_PaddedCount a_jobs_remaining;
                    semaphore   done;
                }

          This way, ds_JobPhase becomes reusable, and all a thread has to do when pushing or grabing a job
          is to call the correct accessor function in a compile-time table using the job identifier. 
          The end product will result in something almost identical to the domain-specific version.

          The point of this fat struct is to simplify the creation and editing of new phases, while being
          as performant as context-tailored phase structures. Furthermore, threads no longer has to consider
          what phase it is in, and can solely focus of the type of its acquired job and immediately access
          the appropriate array in the ds_JobPhase.

          To illustrate this, consider the following scenario when a thread obtains a job to execute:

            enum ds_JobType type = id::Tag
            u32 index = id::Index
            
            // Grab the correct function pointer to call
            FUNC_PTR function = DispatchExecutionTable[type];

            // The function will grab job arguments from ( (argument_type *) phase->job_arrays_by_type[type] )[index]
            function(phase, index);

         Next, consider when a thread wishes to push a job. Since the thread knows the type it wishes to push, and we assume it has
         atomically retreived a slot index to store the task, we get something like:

            (argument_type *) array = phase->job_arrays_by_type[type];
            (argument_type *) job = array + index;
            
            // The thread may now fill job accordingly before publishing the job
            job->data = ...

        Now, all of this is fine and dandy, and the cost we pay is to my knowledge harder prediction in the hardware as
        any of the types may be dispatched at the same instruction address. An less general approach to this fat struct
        would be to slim it down slightly; instead, one could have a ds_JobPhasePhysics. It would look exactly the same
        expect with fewer JobTypes, which should improve prediction in the hardware, regardless if we choose indirect
        function calls or switches + direct function calls.
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















#include "fifo_spmc.h"


/* NOTE: WE ASSUME MASTER THREAD/WORKER HAS ID AND INDEX 0. */

extern struct task_context *g_task_ctx;

typedef void (*TASK)(void *);

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
	struct ds_Worker *executor;
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
	struct ds_Worker *workers;
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
