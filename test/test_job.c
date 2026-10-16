#include <stdio.h>
#include <string.h>

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
            AtomicFetchAddRlx32(&inc, local_inc);
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

/*
 * ds_WSDeque on one thread: pops come back LIFO and steals FIFO, an empty deque gives DS_JOB_ID_EMPTY (pop) and
 * DS_JOB_ID_NULL (steal), unpublished ids are not stealable, and a deque of initial size 1 grows through many
 * reallocations without losing or reordering ids.
 */
static struct test_Output WSDequeSerialTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct ds_WSDeque *deque = ArenaPushAligned(env->mem_1, sizeof(struct ds_WSDeque), DS_CACHE_LINE);
    TEST_NOT_EQUAL(NULL, deque);
    ds_WSDequeAlloc(deque, ds_ThreadSelfIndex(), 1);

    TEST_EQUAL(DS_JOB_ID_EMPTY, ds_WSDequeTryPopBottom(deque));
    TEST_EQUAL(DS_JOB_ID_NULL, ds_WSDequeTrySteal(deque));

    /* pushed but not published: invisible to stealers */
    ds_WSDequePushBottom(deque, 7);
    TEST_EQUAL(DS_JOB_ID_NULL, ds_WSDequeTrySteal(deque));
    ds_WSDequePublish(deque);
    TEST_EQUAL(7, ds_WSDequeTrySteal(deque));

    const u32 n = 1000;
    for (u32 i = 0; i < n; ++i) { ds_WSDequePushBottom(deque, i); }
    ds_WSDequePublish(deque);
    for (u32 i = 0; i < n / 2; ++i) { TEST_EQUAL(i, ds_WSDequeTrySteal(deque)); }
    for (u32 i = n; i > n / 2; --i) { TEST_EQUAL(i - 1, ds_WSDequeTryPopBottom(deque)); }
    TEST_EQUAL(DS_JOB_ID_EMPTY, ds_WSDequeTryPopBottom(deque));
    TEST_EQUAL(DS_JOB_ID_NULL, ds_WSDequeTrySteal(deque));

    /* interleaved, across the ring's end */
    u32 next_push = 0;
    u32 next_steal = 0;
    for (u32 round = 0; round < 100; ++round)
    {
        for (u32 k = 0; k < 37; ++k) { ds_WSDequePushBottom(deque, next_push++); }
        ds_WSDequePublish(deque);
        for (u32 k = 0; k < 30; ++k) { TEST_EQUAL(next_steal, ds_WSDequeTrySteal(deque)); next_steal += 1; }
    }
    while (next_steal < next_push) { TEST_EQUAL(next_steal, ds_WSDequeTrySteal(deque)); next_steal += 1; }
    TEST_EQUAL(DS_JOB_ID_EMPTY, ds_WSDequeTryPopBottom(deque));

    ds_WSDequeDealloc(deque);

	return output;
}

#define WSDEQUE_PARALLEL_IDS    200000

struct test_WSDequeParallel
{
    struct ds_WSDeque * deque;
    u32 *               a_taken;        /* a_taken[id]: times id was popped or stolen */
    u32                 a_role;         /* the first job to claim it owns the deque */
    u32                 a_ready;        /* 1 once the owner allocated the deque */
    u32                 a_total;        /* ids taken so far */
};

static void WSDequeParallelTake(struct test_WSDequeParallel *t, const ds_JobId id)
{
    AtomicAddFetchRlx32(t->a_taken + id, 1);
    AtomicAddFetchRel32(&t->a_total, 1);
}

/* One job per worker: the first owns the deque (pushes in batches, pops now and then), the others steal. */
static void WSDequeParallelJob(void *args)
{
    struct test_WSDequeParallel *t = args;
    if (AtomicFetchAddRlx32(&t->a_role, 1) == 0)
    {
        ds_WSDequeAlloc(t->deque, ds_ThreadSelfIndex(), 1);
        AtomicStoreRel32(&t->a_ready, 1);

        u32 id = 0;
        while (id < WSDEQUE_PARALLEL_IDS)
        {
            const u32 batch = 1 + (id * 7919) % 64;
            for (u32 k = 0; k < batch && id < WSDEQUE_PARALLEL_IDS; ++k) { ds_WSDequePushBottom(t->deque, id++); }
            ds_WSDequePublish(t->deque);
            for (u32 k = 0; k < batch / 3; ++k)
            {
                const ds_JobId popped = ds_WSDequeTryPopBottom(t->deque);
                if (popped != DS_JOB_ID_EMPTY && popped != DS_JOB_ID_NULL) { WSDequeParallelTake(t, popped); }
            }
        }

        ds_JobId popped;
        while ((popped = ds_WSDequeTryPopBottom(t->deque)) != DS_JOB_ID_EMPTY)
        {
            if (popped != DS_JOB_ID_NULL) { WSDequeParallelTake(t, popped); }
        }
    }
    else
    {
        while (!AtomicLoadAcq32(&t->a_ready)) { ds_CpuPause(1); }
        while (AtomicLoadAcq32(&t->a_total) < WSDEQUE_PARALLEL_IDS)
        {
            const ds_JobId stolen = ds_WSDequeTrySteal(t->deque);
            if (stolen != DS_JOB_ID_NULL) { WSDequeParallelTake(t, stolen); }
        }
    }

    while (AtomicLoadAcq32(&t->a_total) < WSDEQUE_PARALLEL_IDS) { ds_CpuPause(1); }
}

/*
 * ds_WSDeque on every scheduler worker: one owner pushes 200000 ids in batches (growing the deque from size 1)
 * and pops some, every other worker steals; each id is taken exactly once.
 */
static struct test_Output WSDequeParallelTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct test_WSDequeParallel t;
    t.deque = ArenaPushAligned(env->mem_1, sizeof(struct ds_WSDeque), DS_CACHE_LINE);
    t.a_taken = ArenaPush(env->mem_1, WSDEQUE_PARALLEL_IDS*sizeof(u32));
    TEST_TRUE(t.deque != NULL && t.a_taken != NULL);
    memset(t.a_taken, 0, WSDEQUE_PARALLEL_IDS*sizeof(u32));
    AtomicStoreRel32(&t.a_role, 0);
    AtomicStoreRel32(&t.a_ready, 0);
    AtomicStoreRel32(&t.a_total, 0);

    ds_TestRunParallel(WSDequeParallelJob, &t);

    u32 wrong = 0;
    for (u32 id = 0; id < WSDEQUE_PARALLEL_IDS; ++id)
    {
        wrong += (AtomicLoadAcq32(t.a_taken + id) != 1);
    }
    TEST_EQUAL(0, wrong);
    TEST_EQUAL(WSDEQUE_PARALLEL_IDS, AtomicLoadAcq32(&t.a_total));
    ds_WSDequeDealloc(t.deque);

	return output;
}

#define JOB_TREE_SIZE   100000

/* A phase whose job i spawns jobs 2i + 1 and 2i + 2 (a binary tree below the one seed job 0). */
struct test_JobTreePhase
{
    struct ds_JobPhase  phase;
    u32 *               a_executed;     /* a_executed[i]: times job i ran */
    u32 *               a_thread;       /* a_thread[t]: jobs run by thread t */
};

static struct test_JobTreePhase *g_job_tree;

static u32 JobTreeDispatch(const ds_JobId job)
{
    const u32 i = ds_JobIdIndex(job);
    AtomicAddFetchRlx32(g_job_tree->a_executed + i, 1);
    AtomicAddFetchRlx32(g_job_tree->a_thread + ds_ThreadSelfIndex(), 1);

    u32 spawned = 0;
    for (u32 child = 2*i + 1; child <= 2*i + 2 && child < JOB_TREE_SIZE; ++child)
    {
        ds_WSDequePushBottom(g_scheduler->deque + ds_ThreadSelfIndex(), ds_JobIdInit(0, child));
        spawned += 1;
    }

    /* jobs created minus 1 */
    return spawned - 1;
}

/*
 * A real job phase: one seed job spawns a binary tree of 100000 jobs, spread over the workers by stealing.
 * Every job runs exactly once, ds_JobPhaseEnd returns with no job remaining, and a second phase on the same
 * struct does it again.
 */
static struct test_Output JobPhaseTreeTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct test_JobTreePhase tree;
    g_job_tree = &tree;
    tree.a_executed = ArenaPush(env->mem_1, JOB_TREE_SIZE*sizeof(u32));
    tree.a_thread = ArenaPush(env->mem_1, g_scheduler->worker_count*sizeof(u32));
    TEST_TRUE(tree.a_executed != NULL && tree.a_thread != NULL);
    ds_JobPhaseAlloc(env->mem_1, &tree.phase, 1, JobTreeDispatch);

    for (u32 pass = 0; pass < 2; ++pass)
    {
        memset(tree.a_executed, 0, JOB_TREE_SIZE*sizeof(u32));
        memset(tree.a_thread, 0, g_scheduler->worker_count*sizeof(u32));

        ds_JobPhaseBegin(&tree.phase);
        ds_WSDequePushBottom(g_scheduler->seed_deque, ds_JobIdInit(0, 0));
        ds_JobPhaseReserve(&tree.phase, 0, 1);
        AtomicStoreRlx32(&g_scheduler->a_seeds_remaining, 1);
        ds_JobPhaseAddFetchRemaining(&tree.phase, 1);
        ds_WSDequePublish(g_scheduler->seed_deque);
        for (u32 w = 1; w < g_scheduler->worker_count; ++w)
        {
            SemaphorePost(&g_scheduler->jobs_are_available);
        }
        ds_MasterRunAvailableJobs();
        ds_JobPhaseEnd();

        TEST_EQUAL(0, AtomicLoadAcq32(&tree.phase.a_jobs_remaining));
        u32 wrong = 0;
        u32 total = 0;
        for (u32 i = 0; i < JOB_TREE_SIZE; ++i)
        {
            wrong += (AtomicLoadRlx32(tree.a_executed + i) != 1);
        }
        for (u32 w = 0; w < g_scheduler->worker_count; ++w)
        {
            total += AtomicLoadRlx32(tree.a_thread + w);
        }
        TEST_EQUAL(0, wrong);
        TEST_EQUAL(JOB_TREE_SIZE, total);
    }

    g_job_tree = NULL;

	return output;
}

static struct test_Output (*jobscheduler_tests[])(struct test_Environment *) =
{
    WSDequeSerialTest,
    WSDequeParallelTest,
    JobPhaseTreeTest,
};

struct suite_Correctness storage_jobscheduler_correctness_suite =
{
	.id = "job scheduler",
	.unit_test = jobscheduler_tests,
	.unit_test_count = sizeof(jobscheduler_tests) / sizeof(jobscheduler_tests[0]),
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
