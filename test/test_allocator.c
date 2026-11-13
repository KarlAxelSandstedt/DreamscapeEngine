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

#include <string.h>

#include "ds_test.h"

struct block_allocator_stress_input
{
	u32 	allocations_left;
	u64		block_size;
};

struct list_node 
{
	void *next;
};

#define G_256B_COUNT  ((u64) 100000)
#define G_1MB_COUNT   ((u64) 10000)

struct test_PerformanceSerial allocator_serial_test[] =
{
    0
};

/* defined in test_allocator_correctness.c */
extern struct suite_Correctness storage_allocator_correctness_suite;

struct suite_Performance storage_performance_allocator_suite =
{
	.id = "Allocator Performance",
	.parallel_test = NULL,
	.parallel_test_count = 0,
	/* no serial tests yet: allocator_serial_test only holds an empty entry, whose NULL test the runner calls */
	.serial_test = NULL,
	.serial_test_count = 0,
};

struct suite_Performance *allocator_performance_suite = &storage_performance_allocator_suite;
struct suite_Correctness *allocator_correctness_suite = &storage_allocator_correctness_suite;
