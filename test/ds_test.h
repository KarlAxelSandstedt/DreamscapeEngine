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

#ifndef __DS_TEST_H__
#define __DS_TEST_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include <stdio.h>

#include "ds_base.h"
#include "ds_math.h"
#include "ds_job.h"

/* Correctness Test framework entry point */
void ds_TestMainCorrectness(void);
/* Performance Test framework entry point */
void ds_TestMainPerformance(void);

/********************************** Performance Testing ************************************/

/*
rt
==
Repetition Tester: TODO
*/

enum rt_Type
{
	RT_TEST_PERFORMANCE,		    /* single threaded perfomance 		    */
	RT_TEST_PARALLEL_PERFORMANCE,	/* multithreaded threaded perfomance 	*/
	RT_TEST_COUNT
};

enum rt_State
{
	RT_UNINITIALIZED = 0,
	RT_TESTING,
	RT_COMPLETED,
	RT_ERROR,
};

struct rt
{
	u64 time;
	u64 bytes;
	u64 tsc_in_current_test;
	u64 bytes_in_current_test;
	enum rt_State state;
	u32 enter_count;
	u32 exit_count;
	u32 print : 1;

	u64 bytes_to_process;
	u64 tsc_retry_max;	/* maximum tsc since last new best iteration before we end the test */
	u64 tsc_freq;
	u64 tsc_start;

	u64 test_count;
	u64 total_time;
	u64 tsc_iteration_max;
	u64 tsc_iteration_min;

	u64 cycles_in_current_test;
	u64 cycles_min_time;
	u64 cycles_max_time;
	u64 cycles;
	
	u64 page_faults_in_current_test;
	u64 page_faults_min_time;
	u64 page_faults_max_time; /* page fault count on max count */
	u64 page_faults;

	u64 branch_misses_in_current_test;
	u64 branch_misses_min_time;
	u64 branch_misses_max_time;
	u64 branch_misses;

	u64 frontend_stalled_cycles_in_current_test;
	u64 frontend_stalled_cycles_min_time;
	u64 frontend_stalled_cycles_max_time;
	u64 frontend_stalled_cycles;

	u64 backend_stalled_cycles_in_current_test;
	u64 backend_stalled_cycles_min_time;
	u64 backend_stalled_cycles_max_time;
	u64 backend_stalled_cycles;
	
#ifdef __linux__

#define NUM_EVENTS 5
#define PAGE_FAULT_SAMPLING_PERIOD 1		/* counter period for new update */
#define BRANCH_MISSES_SAMPLING_PERIOD 1000
	u64 event_count;
	u64 pf_id; /* page faults, event group leader */
	u64 bm_id; /* branch_misses */
	u64 fnt_id; /* frontend stall cycles */
	u64 bck_id; /* backend stall cycles */
	u64 cyc_id;  /* non-cpu-frequency-scaled cycles */
	i32 pf_fd; 
	i32 bm_fd;  
	i32 fnt_fd;
	i32 bck_fd;
	i32 cyc_fd;
#endif
};

u32     rt_TestingCheck(struct rt *tester);
void    rt_Wave(struct rt *tester, const u64 bytes_to_process, const u64 tsc_freq, const u64 tsc_retry_max, const u32 print);
void    rt_BeginTime(struct rt *tester);
void    rt_EndTime(struct rt *tester);
void    rt_PrintStatistics(const struct rt *tester, FILE *file);

struct test_PerformanceSerial
{
	const char *	id;
	const u64		size;
	void *			(*test_init)(void);
	void 			(*test_reset)(void *);
	void 			(*test_free)(void *);
	void 			(*test)(void *);
};

struct test_PerformanceParallel
{
	const char *id;
	const u64	size;
	void *		(*test_init)(void);
	void 		(*test_reset)(void *);
	void 		(*test_free)(void *);
	TASK		test;
};

struct test_PerformanceInput
{
	u32 *	a_barrier;
	void *	args;
	TASK	test;
};

struct suite_Performance
{
	const char *			                id;
	const struct test_PerformanceSerial *   serial_test;
	const u32 		                        serial_test_count;
	const struct test_PerformanceParallel * parallel_test;
	const u32 		                        parallel_test_count;
};

/********************************** Correctness Testing  ************************************/

struct test_Environment
{
	struct arena *  mem_1;
	struct arena *  mem_2;
	struct arena *  mem_3;
	struct arena *  mem_4;
	struct arena *  mem_5;
	struct arena *  mem_6;
	const u64       seed;
};

struct test_Output
{
	const char *id;
	const char *file;
	u64         line;
	u64         success;
};

struct test_CorrectnessRepetition
{
	struct test_Output  (*test)(void);
	const u32           count;
};

struct suite_Correctness
{
	char *                                      id;
	struct test_Output                          (**unit_test)(struct test_Environment *);
	const u64                                   unit_test_count;
	const struct test_CorrectnessRepetition *   repetition_test;
	const u64                                   repetition_test_count;
};

#ifdef KAS_DEBUG
#define TEST_FAILURE { output.success = 0; output.file = __FILE__; output.line = __LINE__; Breakpoint(1); return output; }
#else
#define TEST_FAILURE { output.success = 0; output.file = __FILE__; output.line = __LINE__; return output; }
#endif 
#define PRINT_FAILURE(t1, t2, a, b, print) { fprintf(stderr, t1); print(stderr, a); fprintf(stderr, t2); print(stderr, b); }
#define PRINT_SINGLE(t, a, print) { fprintf(stderr, t); print(a); }

#define TEST_EQUAL(exp, act) if ((exp) != (act)) { TEST_FAILURE } else { output.success = 1; }
#define TEST_NOT_EQUAL(exp, act) if ((exp) == (act)) { TEST_FAILURE } else { output.success = 1; }
#define TEST_EQUAL_PRINT(exp, act, print) if ((exp) != (act)) { PRINT_FAILURE("EXPECTED:\t", "ACTUAL:\t", (exp), (act), (print)) TEST_FAILURE } else { output.success = 1; }
#define TEST_NOT_EQUAL_PRINT(exp, act, print) if ((exp) == (act)) { PRINT_FAILURE("NOT EXPECTED\t", "ACTUAL:\t", (exp), (act), (print)) TEST_FAILURE } else { output.success = 1; }

#define TEST_ZERO(exp) if (exp) { TEST_FAILURE } else { output.success = 1; }
#define TEST_NOT_ZERO(exp) if (!(exp)) { TEST_FAILURE } else { output.success = 1; }
#define TEST_ZERO_PRINT(exp, print) if (exp) { PRINT_SINGLE("EXPECTED ZERO:\t", exp, print) TEST_FAILURE } else { output.success = 1; }
#define TEST_NOT_ZERO_PRINT(exp, print) if (!(exp)) { PRINT_SINGLE("EXPECTED NOT ZERO:\t", exp, print) TEST_FAILURE } else { output.success = 1; }

#define TEST_FALSE(exp) if (exp) { TEST_FAILURE } else { output.success = 1; }
#define TEST_TRUE(exp) if (!(exp)) { TEST_FAILURE } else { output.success = 1; }

#ifdef __cplusplus
} 
#endif

/********************************** Suites *********************************/

extern struct suite_Performance *hash_performance_suite;
extern struct suite_Performance *rng_performance_suite;
extern struct suite_Performance *serialize_performance_suite;
extern struct suite_Performance *allocator_performance_suite;

//extern struct suite_Correctness *array_list_suite;
//extern struct suite_Correctness *hierarchy_index_suite;
extern struct suite_Correctness *allocator_suite;
extern struct suite_Correctness *math_suite;
extern struct suite_Correctness *kas_string_suite;
extern struct suite_Correctness *serialize_suite;

#endif
