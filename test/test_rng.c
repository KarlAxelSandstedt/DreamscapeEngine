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

#include "test_local.h"
#include "kas_random.h"

static const u64 g_rng_count = 1024*1024;
static void thread_local_rng_u64_test(void *void_count)
{
	for (u32 i = 0; i < g_rng_count; ++i)
	{
		RngU64();
	}
}

static void global_rng_u64_test(void *void_count)
{
	for (u32 i = 0; i < g_rng_count; ++i)
	{
		TestXoshiro256Next();
	}
}

struct serial_test rng_serial_test[] =
{
	{ 
		.id = "thread_local_rng_u64", 
		.size = 8*g_rng_count,
		.test = &thread_local_rng_u64_test,
		.test_init = NULL,
		.test_reset = NULL,
		.test_free = NULL,
	},

	{ 
		.id = "global_rng_u64_test", 
		.size = 8*g_rng_count,
		.test = &global_rng_u64_test,
		.test_init = NULL,
		.test_reset = NULL,
		.test_free = NULL,
	},
};

struct performance_suite storage_performance_rng_suite =
{
	.id = "RNG Performance",
	.serial_test = rng_serial_test,
	.serial_test_count = sizeof(rng_serial_test) / sizeof(rng_serial_test[0]),
};

struct performance_suite *rng_performance_suite = &storage_performance_rng_suite;

