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
#include <stdlib.h>
#include <string.h>

#include "ds_base.h"
#include "ds_math.h"
#include "ds_job.h"

/* Correctness Test framework entry point */
void ds_TestMainCorrectness(void);
/*
Run test(args) as worker_count jobs started together on the job scheduler's workers (main thread included;
usually one job per worker, but a worker may take several). Returns when all jobs are done.
*/
void ds_TestRunParallel(void (*test)(void *), void *args);
/* Performance Test framework entry point */
void ds_TestMainPerformance(void);

/********************************** Performance Testing ************************************/

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
	void 		(*test)(void *);
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

#define TEST_FAILURE { output.success = 0; output.file = __FILE__; output.line = __LINE__; return output; }
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
extern struct suite_Performance *string_performance_suite;
extern struct suite_Performance *allocator_performance_suite;
extern struct suite_Performance *THashMap_performance_suite;
extern struct suite_Performance *hierarchy_performance_suite;
extern struct suite_Performance *jobscheduler_performance_suite;
extern struct suite_Performance *memory_performance_suite;

//extern struct suite_Correctness *array_list_suite;
//extern struct suite_Correctness *hierarchy_index_suite;
extern struct suite_Correctness *allocator_correctness_suite;
//extern struct suite_Correctness *math_correctness_suite;
extern struct suite_Correctness *kas_string_correctness_suite;
extern struct suite_Correctness *serialize_correctness_suite;
extern struct suite_Correctness *THashMap_correctness_suite;
extern struct suite_Correctness *jobscheduler_correctness_suite;
extern struct suite_Correctness *geometry_correctness_suite;
extern struct suite_Correctness *voronoi_correctness_suite;
extern struct suite_Correctness *gjk_correctness_suite;
extern struct suite_Correctness *contact_correctness_suite;
extern struct suite_Correctness *mesh_contact_correctness_suite;
extern struct suite_Correctness *config_correctness_suite;
extern struct suite_Correctness *container_correctness_suite;
extern struct suite_Correctness *base_correctness_suite;
extern struct suite_Correctness *dynamics_correctness_suite;
extern struct suite_Correctness *asset_correctness_suite;
extern struct suite_Correctness *shape_correctness_suite;
extern struct suite_Correctness *raycast_correctness_suite;
extern struct suite_Correctness *bvh_correctness_suite;
extern struct suite_Correctness *renderer_correctness_suite;
extern struct suite_Correctness *ui_correctness_suite;
extern struct suite_Correctness *cmd_correctness_suite;
extern struct suite_Correctness *rng_correctness_suite;

#endif
