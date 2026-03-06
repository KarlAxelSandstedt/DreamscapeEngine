/*
==========================================================================
    Copyright (C) 2026 Axel Sandstedt 

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
#include "ds_random.h" 
#include "ds_hash_map.h"

#define XXH_INLINE_ALL
#include "xxhash.h"

struct ds_Structure
{
    THASH_NODE;
    TPOOL_NODE;

    u64     key;
};

u32 u64Hash(const u64 *ptr)
{
    return (u32) XXH3_64bits(ptr, sizeof(u64)); 
}

u32 u64Equivalence(const u64 *a, const u64 *b)
{
    return *a == *b;
}

TPOOL_DECLARE(ds_Structure)
TPOOL_DEFINE(ds_Structure)
THASH_DECLARE(ds_Structure, u64)
THASH_DEFINE(ds_Structure, key, u64, u64Hash, u64Equivalence)

static struct ds_StructureTHashMap g_tmap;
static struct ds_StructureTPool *g_tpool;
static u32 first = 1;

void *ds_StructureTHashMapTestInit(void)
{
    if (ds_ThreadSelfIndex() == 0)
    {
        AtomicStoreRel64(&g_tpool, malloc(sizeof(struct ds_StructureTPool)));
        g_tmap = ds_StructureTHashMapAlloc(NULL, g_tpool, 4096);
    }

    return &g_tpool;
}

void ds_StructureTHashMapTestReset(void *args)
{
    if (ds_ThreadSelfIndex() == 0)
    {
	    struct ds_StructureTPool *pool = AtomicLoadAcq64(&g_tpool);
        if (!first)
        {
            ds_StructureTPoolDealloc(pool);
        }
        first = 0;

        for (u32 i = 0; i < g_tmap.hash_len; ++i)
        {
            ds_Assert((AtomicLoadRlx64(g_tmap.a_hash + i) & THASH_NEXT_MASK) == THASH_NULL);
        }
        ds_StructureTPoolAlloc(pool, g_arch_config->logical_core_count, 1);
        ds_StructureTHashMapFlush(&g_tmap);
    }
}

void ds_StructureTHashMapTestFree(void *args)
{
    if (ds_ThreadSelfIndex() == 0 && g_tpool)
    {
	    struct ds_StructureTPool *pool = AtomicLoadAcq64(&g_tpool);
        g_tpool = NULL;
        ds_StructureTPoolDealloc(pool);
        ds_StructureTHashMapDealloc(&g_tmap);
        free(pool);
        first = 1;
    }
}

void ds_StructureTHashMapAddAllRemoveAllTest(void *null)
{
	struct ds_StructureTPool *pool = AtomicLoadAcq64(&g_tpool);
    const u32 count = 10000;
    u32 indices[10000]; 
    const u32 t = ds_ThreadSelfIndex();
    for (u32 i = 0; i < count; ++i)
    {
        struct slot slot = ds_StructureTPoolIncrement(pool);
        indices[i] = slot.index;
        struct ds_Structure *n = slot.address;
        n->key = indices[i]; 
        AtomicStoreRlx64(&n->a_hash_node, 0);
        ds_StructureTHashMapAdd(&g_tmap, n, indices[i]);
    }

    for (u32 i = 0; i < count; ++i)
    {
        const u64 key = indices[i]; 
        ds_StructureTHashMapRemove(&g_tmap, &key);
        ds_StructureTPoolRemove(pool, indices[i]);
    }
}

//struct test_PerformanceSerial THashMap_serial_test[] =
//{
//	{
//		.id = "serial_block_t_hash_map_256B_test",
//		.size = g_256B_count * 256,
//		.test = &serial_block_t_hash_map_test_256B,
//		.test_init = NULL,
//		.test_reset = NULL,
//		.test_free = NULL,
//	},
//};

struct test_PerformanceParallel THashMap_parallel_test[] =
{
    {
        .id = "ds_StructureTHashMapAddAllRemoveAll",
        .size = 1,
        .test = &ds_StructureTHashMapAddAllRemoveAllTest,
        .test_init = &ds_StructureTHashMapTestInit,
        .test_reset = &ds_StructureTHashMapTestReset,
        .test_free = &ds_StructureTHashMapTestFree,
    },
};

struct suite_Correctness storage_THashMap_correctness_suite =
{
    0
};

struct suite_Performance storage_THashMap_performance_suite =
{
	.id = "THashMap Performance",
	.parallel_test = THashMap_parallel_test,
	.parallel_test_count = sizeof(THashMap_parallel_test) / sizeof(THashMap_parallel_test[0]),
	//.serial_test = THashMap_serial_test,
	//.serial_test_count = sizeof(THashMap_serial_test) / sizeof(THashMap_serial_test[0]),
	.serial_test = NULL,
	.serial_test_count = 0,
};

struct suite_Performance *THashMap_performance_suite = &storage_THashMap_performance_suite;
struct suite_Correctness *THashMap_correctness_suite = &storage_THashMap_correctness_suite;



