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

#include "ds_test.h"
#include "ds_math.h"

/*
 * The local generator (struct rng) against reference values computed independently (SplitMix64 seeding, xoshiro256**):
 * seeds 0 and 0x0123456789abcdef give the expected state and first four numbers. Two generators with one seed agree;
 * using one leaves the thread's numbers unchanged; ranges stay in [min, max] (single-value and full ranges included).
 */
struct test_Output RngLocalTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const u64 seed[2] = { 0, 0x0123456789abcdef };
    const u64 state[2][4] =
    {
        { 0xe220a8397b1dcdaf, 0x6e789e6aa1b965f4, 0x06c45d188009454f, 0xf88bb8a8724c81ec },
        { 0x157a3807a48faa9d, 0xd573529b34a1d093, 0x2f90b72e996dccbe, 0xa2d419334c4667ec },
    };
    const u64 first[2][4] =
    {
        { 0x99ec5f36cb75f2b4, 0xbf6e1f784956452a, 0x1a5f849d4933e6e0, 0x6aa594f1262d2d2c },
        { 0xa2c2a42038d4ec3d, 0x05fc25d0738e7b0f, 0x625e7bff938e701e, 0x1ba4ddc6fe2b5726 },
    };
    for (u32 k = 0; k < 2; ++k)
    {
        struct rng rng;
        RngLocalInit(&rng, seed[k]);
        for (u32 i = 0; i < 4; ++i)
        {
            TEST_TRUE(rng.s[i] == state[k][i]);
        }
        for (u32 i = 0; i < 4; ++i)
        {
            TEST_TRUE(RngLocalU64(&rng) == first[k][i]);
        }
    }

    /* the thread's next number, before and after using a local generator */
    RngPushState();
    const u64 thread_before = RngU64();
    RngPopState();

    struct rng a, b;
    RngLocalInit(&a, 42);
    RngLocalInit(&b, 42);
    for (u32 i = 0; i < 1000; ++i)
    {
        TEST_TRUE(RngLocalU64(&a) == RngLocalU64(&b));
        const u64 r = RngLocalU64Range(&a, 10, 20);
        TEST_TRUE(10 <= r && r <= 20);
        TEST_TRUE(RngLocalU64Range(&a, 7, 7) == 7);
        RngLocalU64Range(&b, 10, 20);
        RngLocalU64Range(&b, 7, 7);
        TEST_TRUE(RngLocalU64Range(&a, 0, U64_MAX) == RngLocalU64Range(&b, 0, U64_MAX));
    }

    RngPushState();
    const u64 thread_after = RngU64();
    RngPopState();
    TEST_TRUE(thread_before == thread_after);

	return output;
}

static struct test_Output (*rng_tests[])(struct test_Environment *) =
{
    RngLocalTest,
};

struct suite_Correctness m_rng_suite =
{
	.id = "rng",
	.unit_test = rng_tests,
	.unit_test_count = sizeof(rng_tests) / sizeof(rng_tests[0]),
};

struct suite_Correctness *rng_correctness_suite = &m_rng_suite;

#define RNG_COUNT   1024*1024
static const u64 g_rng_count = RNG_COUNT;
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

struct test_PerformanceSerial rng_serial_test[] =
{
	{ 
		.id = "thread_local_rng_u64", 
		.size = 8*RNG_COUNT,
		.test = &thread_local_rng_u64_test,
		.test_init = NULL,
		.test_reset = NULL,
		.test_free = NULL,
	},

	{ 
		.id = "global_rng_u64_test", 
		.size = 8*RNG_COUNT,
		.test = &global_rng_u64_test,
		.test_init = NULL,
		.test_reset = NULL,
		.test_free = NULL,
	},
};

struct suite_Performance storage_performance_rng_suite =
{
	.id = "RNG Performance",
	.serial_test = rng_serial_test,
	.serial_test_count = sizeof(rng_serial_test) / sizeof(rng_serial_test[0]),
};

struct suite_Performance *rng_performance_suite = &storage_performance_rng_suite;

