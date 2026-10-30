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
#include "ds_platform.h"

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

    /* the thread's state before and after using a local generator */
    u64 thread_before[4];
    for (u32 i = 0; i < 4; ++i) { thread_before[i] = tl_xoshiro_256[i]; }

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

    for (u32 i = 0; i < 4; ++i) { TEST_TRUE(thread_before[i] == tl_xoshiro_256[i]); }

	return output;
}

/* the base state (ds_random.c), saved and restored around RngThreadTest */
extern u64 g_xoshiro_256[4];

/*
 * The thread generator's streams against reference values computed independently: xoshiro256**'s state update is
 * linear over GF(2), so stream k's state is T^(k * 2^128) s, with T^(2^128) found by squaring the 256x256 matrix.
 * Xoshiro256Init seeds the calling thread with stream 0; TestXoshiro256Next advances the base, so a stream seeded
 * afterwards starts one number later. The thread's state and the base are restored before checking, so the test
 * leaves both as it found them.
 */
struct test_Output RngThreadTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const u64 seed[4] = { 0x0123456789abcdef, 0xfedcba9876543210, 0x0f1e2d3c4b5a6978, 0x8796a5b4c3d2e1f0 };
    const u64 stream_state[2][4] =
    {
        { 0x562509e8dc029adf, 0x9a6c42a9513660f6, 0x18e74c7c7aaa05f3, 0x3a6c534749a71402 },
        { 0xfeee4b3130d89471, 0xe0cab1ed04f3215b, 0xf73da890ce9d00d9, 0x04068dd8c7f02619 },
    };
    const u64 stream_first[3] = { 0x6666666666666c65, 0x83dbe1a347859f12, 0xd0a354ef5e6e8139 };
    const u64 stream_0_second = 0xd90633608dbae0aa;

    u64 thread_saved[4];
    for (u32 i = 0; i < 4; ++i) { thread_saved[i] = tl_xoshiro_256[i]; }

    u64 base[4];
    for (u32 i = 0; i < 4; ++i) { base[i] = g_xoshiro_256[i]; }

    Xoshiro256Init(seed);
    const u64 init_first = RngU64();
    const u64 init_second = RngU64();

    u32 stream_ok = 1;
    u64 first[3];
    for (u32 k = 0; k < 3; ++k)
    {
        ThreadXoshiro256InitSequence(k);
        for (u32 i = 0; k > 0 && i < 4; ++i)
        {
            stream_ok = stream_ok && tl_xoshiro_256[i] == stream_state[k-1][i];
        }
        first[k] = RngU64();
    }

    const u64 base_first = TestXoshiro256Next();
    ThreadXoshiro256InitSequence(0);
    const u64 advanced_first = RngU64();

    /* ranges, from the seeded stream 0 */
    Xoshiro256Init(seed);
    u32 range_ok = 1;
    for (u32 i = 0; i < 10000; ++i)
    {
        const u64 r = RngU64Range(10, 20);
        const f32 n = RngF32Normalized();
        const f32 f = RngF32Range(-1.0f, 1.0f);
        range_ok = range_ok && 10 <= r && r <= 20
            && RngU64Range(7, 7) == 7
            && 0.0f <= n && n <= 1.0f
            && -1.0f <= f && f <= 1.0f
            && RngF32Range(2.0f, 2.0f) == 2.0f;
        RngU64Range(0, U64_MAX);
    }

    Xoshiro256Init(base);
    for (u32 i = 0; i < 4; ++i) { tl_xoshiro_256[i] = thread_saved[i]; }

    TEST_TRUE(init_first == stream_first[0] && init_second == stream_0_second);
    TEST_TRUE(stream_ok);
    TEST_TRUE(first[0] == stream_first[0] && first[1] == stream_first[1] && first[2] == stream_first[2]);
    TEST_TRUE(base_first == stream_first[0] && advanced_first == stream_0_second);
    TEST_TRUE(range_ok);
    for (u32 i = 0; i < 4; ++i)
    {
        TEST_TRUE(g_xoshiro_256[i] == base[i]);
    }

	return output;
}

/*
 * RngSystem fills the whole buffer: 4 KiB pre-filled with 0xa5 come back with every 64-byte block changed (a block
 * left as it was by chance: 2^-512) and two fills differ. Size 0 leaves the buffer as it was, and is no error with a
 * NULL buffer either (#197: fatal on Windows).
 */
struct test_Output RngSystemTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const u32 size = 4096;
    const u32 block = 64;
    u8 *a = ArenaPush(env->mem_1, size);
    u8 *b = ArenaPush(env->mem_1, size);
    TEST_TRUE(a != NULL && b != NULL);
    memset(a, 0xa5, size);
    memset(b, 0xa5, size);
    RngSystem(a, size);
    RngSystem(b, size);

    u32 blocks_changed = 0;
    for (u32 k = 0; k < size / block; ++k)
    {
        u32 changed = 0;
        for (u32 i = 0; i < block; ++i) { changed |= (a[k*block + i] != 0xa5); }
        blocks_changed += changed;
    }
    TEST_EQUAL(size / block, blocks_changed);
    TEST_TRUE(memcmp(a, b, size) != 0);

    u8 c[8];
    memset(c, 0xa5, sizeof(c));
    RngSystem(c, 0);
    u32 untouched = 1;
    for (u32 i = 0; i < sizeof(c); ++i) { untouched = untouched && c[i] == 0xa5; }
    TEST_TRUE(untouched);
    RngSystem(NULL, 0);

	return output;
}

static struct test_Output (*rng_tests[])(struct test_Environment *) =
{
    RngLocalTest,
    RngThreadTest,
    RngSystemTest,
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

