#include <stdio.h>

#include "ds_test.h"
#include "ds_job.h"

struct ds_WSDeque g_deque = { 0 };
static u32 allocated = 0;
static u64 g_sum = 0;
static u32 pushing = 1;
static u32 inc = 0;

void *ds_WSDequeTestInit(void)
{
    if (!allocated)
    {
        ds_WSDequeAlloc(&g_deque, 0, 1); 
    }
    allocated = 1;
    return &g_deque;
}

void ds_WSDequeTestReset(void *args)
{
    if (allocated)
    {
        ds_Assert(g_sum == 0 || g_sum == (99999lu*100000lu)/2 );
        AtomicStoreRlx64(&g_sum, 0);
        ds_WSDequeDealloc(&g_deque);
        AtomicStoreRlx32(&pushing, 1);
        AtomicStoreRlx32(&inc, 0);
    }
    ds_WSDequeAlloc(&g_deque, 0, 1); 
    allocated = 1;
}

void ds_WSDequeTestFree(void *args)
{
    if (allocated)
    {
        ds_WSDequeDealloc(&g_deque);
    }
    allocated = 0;
}

void ds_WSDequeTest(void *void_pool)
{
    if (ds_ThreadSelfIndex() == 0)
    {
        for (u32 i = 0; i < 100000; ++i)
        {
            ds_WSDequePushBottom(&g_deque, i); 
            ds_WSDequePublish(&g_deque); 
        }
        AtomicStoreRlx32(&pushing, 0);
    }

    u64 local_sum = 0;
    u32 local_inc = 0;
    u32 its = 0;
    do
    {
         const ds_JobId id = (ds_ThreadSelfIndex() == 0)
                            ? ds_WSDequeTryPopBottom(&g_deque)
                            : ds_WSDequeTrySteal(&g_deque);

         //if (!(id % 1000))
         //{
         //   fprintf(stderr, "(%lu, %lu): %u\n", local_top, AtomicLoadRlx64(&g_deque.a_bottom), id);
         //}
        
        if (id != DS_JOB_ID_NULL && id != DS_JOB_ID_EMPTY)
        {
            local_sum += id;
            local_inc += 1;
        }

        its++;
        if (its == 1000)
        {
            AtomicFetchAddRlx64(&inc, local_inc);
            its = 0;
            local_inc = 0;
        }

        //if (!AtomicLoadRlx32(&pushing) && local_top > local_bottom)
        //    fprintf(stderr, "(%u, %lu)\n", ds_ThreadSelfIndex(), local_top);
    }
    while (AtomicLoadRlx32(&inc) < 100000);
    AtomicFetchAddRlx64(&g_sum, local_sum);
}

struct test_PerformanceParallel jobscheduler_parallel_test[] =
{
    {
        .id = "ds_WSDequeTest",
        .size = 1,
        .test =       &ds_WSDequeTest,
        .test_init =  &ds_WSDequeTestInit,
        .test_reset = &ds_WSDequeTestReset,
        .test_free =  &ds_WSDequeTestFree,
    },
};

struct suite_Correctness storage_jobscheduler_correctness_suite =
{
    0
};

struct suite_Performance storage_jobscheduler_performance_suite =
{
	.id = "JobScheduler Parallel Tests",
	.parallel_test = jobscheduler_parallel_test,
	.parallel_test_count = sizeof(jobscheduler_parallel_test) / sizeof(jobscheduler_parallel_test[0]),
	.serial_test = NULL,
	.serial_test_count = 0,
};

struct suite_Performance *jobscheduler_performance_suite = &storage_jobscheduler_performance_suite;
struct suite_Correctness *jobscheduler_correctness_suite = &storage_jobscheduler_correctness_suite;
