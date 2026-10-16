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

struct ds_Struct
{
    TPOOL_NODE;
    u8  pad[52];
};

DEFINE_TPOOL(ds_Struct);

/* the pool's address as a u64, so the 64-bit atomics work where pointers are 32 bits (wasm32) */
static u64 g_tpool;
static u32 first = 1;

void *ds_StructTPoolIncrementTestInit(void)
{
    if (ds_ThreadSelfIndex() == 0)
    {
        AtomicStoreRel64(&g_tpool, (u64) (uintptr_t) malloc(sizeof(ds_TPool(ds_Struct))));
    }

    return &g_tpool;
}

void ds_StructTPoolIncrementTestReset(void *args)
{
    if (ds_ThreadSelfIndex() == 0)
    {
	    ds_TPool(ds_Struct) *pool = (ds_TPool(ds_Struct) *) (uintptr_t) AtomicLoadAcq64(&g_tpool);
        if (!first)
        {
            ds_TPoolDealloc(*pool);
        }
        first = 0;

        ds_TPoolAlloc(*pool, g_arch_config->logical_core_count, 1);
    }
}

void ds_StructTPoolIncrementTestFree(void *args)
{
    if (ds_ThreadSelfIndex() == 0 && g_tpool)
    {
	    ds_TPool(ds_Struct) *pool = (ds_TPool(ds_Struct) *) (uintptr_t) AtomicLoadAcq64(&g_tpool);
        g_tpool = 0;
        ds_TPoolDealloc(*pool);
        free(pool);
        first = 1;
    }
}

void ds_StructTPoolIncrementTest(void *void_pool)
{
	ds_TPool(ds_Struct) *pool = (ds_TPool(ds_Struct) *) (uintptr_t) AtomicLoadAcq64(&g_tpool);
    for (u32 i = 0; i < 1024*64; ++i)
    {
        const u32 index = ds_TPoolIncrement(*pool).index;
        struct ds_Struct *addr = ds_TPoolAddress(*pool, index);
        memset(addr, 0xff, sizeof(*addr));
    }
}

void ds_StructTStackTest(void *void_pool)
{
	ds_TPool(ds_Struct) *pool = (ds_TPool(ds_Struct) *) (uintptr_t) AtomicLoadAcq64(&g_tpool);
    const u32 count = 200;
    if (ds_ThreadSelfIndex() == 0)
    {
        for (u32 i = 0; i < count*pool->state.free_list_count; ++i)
        {
            struct slot slot = ds_TPoolIncrement(*pool);
            ds_TPoolFreeListPush(*pool, 0, slot.index);
        }
    }

    for (u32 i = 0; i < count; )
    {
        struct slot slot = ds_TPoolFreeListPop(*pool, 0);
        if (slot.address)
        {
            i += 1;
            struct ds_Struct *addr = slot.address;
            memset(addr, 0xff, sizeof(*addr));
        }
    }
}

struct test_PerformanceSerial allocator_serial_test[] =
{
    0
};

/* defined in test_allocator_correctness.c */
extern struct suite_Correctness storage_allocator_correctness_suite;

struct test_PerformanceParallel allocator_parallel_test[] =
{    
    {
        .id = "ds_StructTStackTest",
        .size = 1,
        .test = &ds_StructTStackTest,
        .test_init = &ds_StructTPoolIncrementTestInit,
        .test_reset = &ds_StructTPoolIncrementTestReset,
        .test_free = &ds_StructTPoolIncrementTestFree,
    },

    {
        .id = "ds_StructTPoolIncrementTest",
        .size = 1,
        .test = &ds_StructTPoolIncrementTest,
        .test_init = &ds_StructTPoolIncrementTestInit,
        .test_reset = &ds_StructTPoolIncrementTestReset,
        .test_free = &ds_StructTPoolIncrementTestFree,
    },
};

struct suite_Performance storage_performance_allocator_suite =
{
	.id = "Allocator Performance",
	.parallel_test = allocator_parallel_test,
	.parallel_test_count = sizeof(allocator_parallel_test) / sizeof(allocator_parallel_test[0]),
	/* no serial tests yet: allocator_serial_test only holds an empty entry, whose NULL test the runner calls */
	.serial_test = NULL,
	.serial_test_count = 0,
};

struct suite_Performance *allocator_performance_suite = &storage_performance_allocator_suite;
struct suite_Correctness *allocator_correctness_suite = &storage_allocator_correctness_suite;
