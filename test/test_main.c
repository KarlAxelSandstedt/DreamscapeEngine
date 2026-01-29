/*
==========================================================================
    Copyright (C) 2025 Axel Sandstedt 

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

#include "test_local.h"

#ifdef __GNUC__
#include "linux_local.h"
#else
#include "win_local.h"
#endif

#include "log.h"

static void run_suite(struct suite *suite, struct test_environment *env, const u64 verbose)
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

		struct test_output out = suite->unit_test[i](env);
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
		struct test_output out;
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

void test_caller(void *task_input)
{
	struct task *t_ctx = task_input;
	struct performance_test_caller_input *input = t_ctx->input;
	while (!AtomicLoadAcq32(input->a_barrier));
	input->test(input->args);
}

static void run_performance_suite(struct performance_suite *suite)
{
	fprintf(stdout, ":::::::::: Running peformance suite %s ::::::::::\n", suite->id);

	const u64 max_time_without_improvement = 10*freq_rdtsc();
	struct repetition_tester tester;
	for (u32 i = 0; i < suite->serial_test_count; ++i)
	{
		memset(&tester, 0, sizeof(tester));
		fprintf(stdout, "\t::: %s ::: \n", suite->serial_test[i].id);

		void *args = (suite->serial_test[i].test_init)
			? suite->serial_test[i].test_init()
			: NULL;

		rt_wave(&tester, suite->serial_test[i].size, freq_rdtsc(), max_time_without_improvement, 1);
		do
		{
			RngPushState();
			if (suite->serial_test[i].test_reset)
			{
				suite->serial_test[i].test_reset(args);
			}		

			rt_begin_time(&tester);	
			suite->serial_test[i].test(args);	
			rt_end_time(&tester);
		
			RngPopState();
		} while (rt_is_testing(&tester));

		if (suite->serial_test[i].test_free)
		{
			suite->serial_test[i].test_free(args);
		}
		rt_print_statistics(&tester, stdout);
	}

	struct arena mem = ArenaAlloc1MB();
	for (u32 i = 0; i < suite->parallel_test_count; ++i)
	{
		memset(&tester, 0, sizeof(tester));
		fprintf(stdout, "\t::: %s ::: \n", suite->parallel_test[i].id);

		ArenaFlush(&mem);
		struct performance_test_caller_input *args = ArenaPush(&mem, g_task_ctx->worker_count * sizeof(struct performance_test_caller_input));
		u32 a_barrier;

		for (u32 k = 0; k < g_task_ctx->worker_count; ++k)
		{
			args[k].test = suite->parallel_test[i].test;
			args[k].a_barrier = &a_barrier;
			args[k].args = (suite->parallel_test[i].test_init)
				? suite->parallel_test[i].test_init()
				: NULL;
		}

		rt_wave(&tester, suite->parallel_test[i].size, freq_rdtsc(), max_time_without_improvement, 1);
		do
		{
			RngPushState();
			ArenaPushRecord(&mem);
			AtomicStoreRel32(&a_barrier, 0);
			struct task_stream *stream = task_stream_init(&mem);
			for (u32 k = 0; k < g_task_ctx->worker_count; ++k)
			{
				if (suite->parallel_test[i].test_reset)
				{
					suite->parallel_test[i].test_reset(args[k].args);
				}		

				task_stream_dispatch(&mem, stream, test_caller, args + k);
			}

			rt_begin_time(&tester);	
			AtomicStoreRel32(&a_barrier, 1);
			task_main_master_run_available_jobs();
			task_stream_spin_wait(stream);
			rt_end_time(&tester);

			task_stream_cleanup(stream);		
			ArenaPopRecord(&mem);
			RngPopState();
		} while (rt_is_testing(&tester));

		if (suite->parallel_test[i].test_free)
		{
			for (u32 k = 0; k < g_task_ctx->worker_count; ++k)
			{
				suite->parallel_test[i].test_free(args[k].args);
			}
		}

		rt_print_statistics(&tester, stdout);
	}
	ArenaFree1MB(&mem);
}

void test_main(void)
{
	struct arena mem_1 = ArenaAlloc(16*1024*1024);
	struct arena mem_2 = ArenaAlloc(1024*1024);
	struct arena mem_3 = ArenaAlloc(1024*1024);
	struct arena mem_4 = ArenaAlloc(1024*1024);
	struct arena mem_5 = ArenaAlloc(1024*1024);
	struct arena mem_6 = ArenaAlloc(1024*1024);

	struct test_environment env = 
	{
		.mem_1 = &mem_1,
		.mem_2 = &mem_2,
		.mem_3 = &mem_3,
		.mem_4 = &mem_4,
		.mem_5 = &mem_5,
		.mem_6 = &mem_6,
		.seed = 2984395893,
	};

#if defined(KAS_TEST_CORRECTNESS)
	run_suite(kas_string_suite, &env, 1);
	run_suite(serialize_suite, &env, 1);
	run_suite(array_list_suite, &env, 1);
	run_suite(hierarchy_index_suite, &env, 1);
	//run_suite(math_suite, &env, 1);
#elif defined(KAS_TEST_PERFORMANCE)
	run_performance_suite(hash_performance_suite);
	//run_performance_suite(rng_performance_suite);
	//run_performance_suite(allocator_performance_suite);
	//run_performance_suite(serialize_performance_suite);
#endif
}
