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

#include <stdio.h>
#include <string.h>

#include "ds_test.h"

static void run_suite(struct suite_Correctness *suite, struct test_Environment *env, const u64 verbose)
{
	if (verbose) { fprintf(stdout, ":::::::::: Running suite %s ::::::::::\n", suite->id); }

	u64 success_count = 0;
	for (u64 i = 0; i < suite->unit_test_count; ++i)
	{
		struct arena record_1 = *env->mem_1;
		struct arena record_2 = *env->mem_2;
		struct arena record_3 = *env->mem_3;
		struct arena record_4 = *env->mem_4;
		struct arena record_5 = *env->mem_5;
		struct arena record_6 = *env->mem_6;

		struct test_Output out = suite->unit_test[i](env);
		if (out.success)
		{
			success_count += 1;
			if (verbose)
			{
				fprintf(stdout, "\tTest %s\n", out.id);
			}
		}
		else if (verbose)
		{
			fprintf(stdout, "\tTest %s failed: %s:%llu\n", out.id, out.file, (long long unsigned int) out.line);
		}

		*env->mem_1 = record_1;
		*env->mem_2 = record_2;
		*env->mem_3 = record_3;
		*env->mem_4 = record_4;
		*env->mem_5 = record_5;
		*env->mem_6 = record_6;
	}

	for (u64 i = 0; i < suite->repetition_test_count; ++i)
	{
		RngPushState();
		struct test_Output out;
		u32 t;
		for (t = 0; t < suite->repetition_test[i].count; ++t)
		{
			out = suite->repetition_test[i].test();
			fprintf(stdout, "\tTest %s iteration (%u/%u)\r", out.id, (t+1), suite->repetition_test[i].count);
			if (!out.success)
			{
				break;
			}
		}

		if (out.success)
		{
			success_count += 1;
			if (verbose)
			{
				fprintf(stdout, "\tTest %s iteration (%u/%u)\n", out.id, t, suite->repetition_test[i].count);
			}
		}
		else if (verbose)
		{
			fprintf(stdout, "\tTest %s iteration (%u/%u)\tfailed: %s:%llu\n", out.id, (t+1), suite->repetition_test[i].count, out.file, (long long unsigned int) out.line);
		}
		RngPopState();
	}

	if (verbose) { fprintf(stdout, "Tests passed: (%llu/%llu)\n",  
			(long long unsigned int) success_count, 
			(long long unsigned int) suite->unit_test_count + suite->repetition_test_count); }
}

enum ds_TestJobType
{
    TEST_JOB,
    TEST_JOB_COUNT
};

struct ds_TestJob
{
	void *	args;
    void    (*test)(void *);
};

struct ds_TestJobPhase
{
    struct ds_JobPhase  phase;
    
    struct ds_TestJob * job;
    u32                 count;
    u32                 a_barrier;
};

u32 TestJob(struct ds_TestJobPhase *phase, struct ds_TestJob *job)
{
	while (!AtomicLoadAcq32(&phase->a_barrier));
	job->test(job->args);
    return U32_MAX;
}

u32 ds_TestJobPhaseDispatch(const ds_JobId job)
{
    struct ds_TestJobPhase *phase = (struct ds_TestJobPhase *) g_scheduler->phase;
    const u32 index = ds_JobIdIndex(job);
    const enum ds_TestJobType type = ds_JobIdTag(job);

    u32 job_diff = 0;
    switch (type)
    {
        case TEST_JOB: { job_diff = TestJob(phase, phase->job + index); } break;
        default: { ds_AssertString(0, "Should not be possible"); } break;
    };

    return job_diff;
}

static void run_performance_suite(struct suite_Performance *suite)
{
	fprintf(stdout, ":::::::::: Running peformance suite %s ::::::::::\n", suite->id);

	const u64 max_time_without_improvement = 10*TscFrequency();
	struct rt tester;
	struct arena mem = ArenaAlloc1MB();

	for (u32 i = 0; i < suite->parallel_test_count; ++i)
	{
		memset(&tester, 0, sizeof(tester));
		fprintf(stdout, "\t::: %s ::: \n", suite->parallel_test[i].id);


		ArenaFlush(&mem);
        
        struct ds_TestJobPhase phase;
        ds_JobPhaseAlloc(&mem, &phase.phase, TEST_JOB_COUNT, ds_TestJobPhaseDispatch);
        phase.job = ArenaPush(&mem, g_scheduler->worker_count * sizeof(struct ds_TestJob));
        phase.count = g_scheduler->worker_count;


		for (u32 k = 0; k < phase.count; ++k)
		{
			phase.job[k].test = suite->parallel_test[i].test;
			phase.job[k].args = (suite->parallel_test[i].test_init)
                ? suite->parallel_test[i].test_init()
                : NULL;
		}

		rt_Wave(&tester, suite->parallel_test[i].size, TscFrequency(), max_time_without_improvement, 1);
		do
		{
			RngPushState();
			ArenaPushRecord(&mem);
			AtomicStoreRel32(&phase.a_barrier, 0);

            {
                ds_JobPhaseBegin(&phase.phase);

                static u32 poo = 0;
                for (u32 k = 0; k < phase.count; ++k)
			    {
			    	if (suite->parallel_test[i].test_reset)
			    	{
			    		suite->parallel_test[i].test_reset(phase.job[k].args);
			    	}		

                    ds_WSDequePushBottom(g_scheduler->seed_deque, ds_JobIdInit(TEST_JOB, k));
			    }

                ds_JobPhaseReserve(&phase.phase, TEST_JOB, phase.count);
                AtomicStoreRlx32(&g_scheduler->a_seeds_remaining, phase.count);
                ds_JobPhaseAddFetchRemaining(&phase.phase, phase.count);
                ds_WSDequePublish(g_scheduler->seed_deque);

                for (u32 i = 1; i < phase.count; ++i)
                {
                    SemaphorePost(&g_scheduler->jobs_are_available);
                }
			    AtomicStoreRel32(&phase.a_barrier, 1);

			    rt_BeginTime(&tester);	

                ds_MasterRunAvailableJobs();
                ds_JobPhaseEnd();
			    
                rt_EndTime(&tester);
            }

			ArenaPopRecord(&mem);
			RngPopState();
		} while (rt_TestingCheck(&tester));

		if (suite->parallel_test[i].test_free)
		{
			for (u32 k = 0; k < g_scheduler->worker_count; ++k)
			{
				suite->parallel_test[i].test_free(phase.job[k].args);
			}
		}

		rt_PrintStatistics(&tester, stdout);
	}

    for (u32 i = 0; i < suite->serial_test_count; ++i)
	{
		memset(&tester, 0, sizeof(tester));
		fprintf(stdout, "\t::: %s ::: \n", suite->serial_test[i].id);

		void *args = (suite->serial_test[i].test_init)
			? suite->serial_test[i].test_init()
			: NULL;

		rt_Wave(&tester, suite->serial_test[i].size, TscFrequency(), max_time_without_improvement, 1);
		do
		{
			RngPushState();
			if (suite->serial_test[i].test_reset)
			{
				suite->serial_test[i].test_reset(args);
			}		

			rt_BeginTime(&tester);	
			suite->serial_test[i].test(args);	
			rt_EndTime(&tester);
		
			RngPopState();
		} while (rt_TestingCheck(&tester));

		if (suite->serial_test[i].test_free)
		{
			suite->serial_test[i].test_free(args);
		}
		rt_PrintStatistics(&tester, stdout);
	}

	ArenaFree1MB(&mem);
}

void ds_TestMainCorrectness(void)
{
	struct arena mem_1 = ArenaAlloc(NULL, 16*1024*1024);
	struct arena mem_2 = ArenaAlloc(NULL, 1024*1024);
	struct arena mem_3 = ArenaAlloc(NULL, 1024*1024);
	struct arena mem_4 = ArenaAlloc(NULL, 1024*1024);
	struct arena mem_5 = ArenaAlloc(NULL, 1024*1024);
	struct arena mem_6 = ArenaAlloc(NULL, 1024*1024);

	struct test_Environment env = 
	{
		.mem_1 = &mem_1,
		.mem_2 = &mem_2,
		.mem_3 = &mem_3,
		.mem_4 = &mem_4,
		.mem_5 = &mem_5,
		.mem_6 = &mem_6,
		.seed = 2984395893,
	};

	run_suite(jobscheduler_correctness_suite, &env, 1);
	run_suite(THashMap_correctness_suite, &env, 1);
	run_suite(allocator_correctness_suite, &env, 1);
	run_suite(kas_string_correctness_suite, &env, 1);
	run_suite(serialize_correctness_suite, &env, 1);
	//run_suite(array_list_correctness_suite, &env, 1);
	//run_suite(hierarchy_correctness_index_suite, &env, 1);
	//run_suite(math_correctness_suite, &env, 1);
}

void ds_TestMainPerformance(void)
{
	//run_performance_suite(jobscheduler_performance_suite);
	//run_performance_suite(THashMap_performance_suite);
	//run_performance_suite(allocator_performance_suite);
	//run_performance_suite(hash_performance_suite);
	//run_performance_suite(rng_performance_suite);
	run_performance_suite(serialize_performance_suite);
}
