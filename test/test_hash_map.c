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

static u32 u64Hash(const u64 key)
{
    return (u32) XXH3_64bits(&key, sizeof(u64));
}

static u32 ds_StructureKeyEqual(const void *node, const void *key)
{
    return ((const struct ds_Structure *) node)->key == *(const u64 *) key;
}

DEFINE_TPOOL(ds_Structure);
DEFINE_THASH(ds_Structure);

static ds_THashMap(ds_Structure) g_tmap;
static ds_TPool(ds_Structure) *g_tpool;
static u32 first = 1;
static u32 g_lookup_mismatches = 0;

void *ds_StructureTHashMapTestInit(void)
{
    if (ds_ThreadSelfIndex() == 0)
    {
        AtomicStoreRel64(&g_tpool, malloc(sizeof(ds_TPool(ds_Structure))));
        ds_THashMapAlloc(NULL, g_tmap, g_tpool, 4096);
    }

    return &g_tpool;
}

void ds_StructureTHashMapTestReset(void *args)
{
    if (ds_ThreadSelfIndex() == 0)
    {
	    ds_TPool(ds_Structure) *pool = (ds_TPool(ds_Structure) *) AtomicLoadAcq64(&g_tpool);
        if (!first)
        {
            ds_TPoolDealloc(*pool);
        }
        first = 0;
        if (AtomicLoadRlx32(&g_lookup_mismatches))
        {
            Log(T_SYSTEM, S_ERROR, "THashMap performance test: %u lookups failed", AtomicLoadRlx32(&g_lookup_mismatches));
        }

        for (u32 i = 0; i < g_tmap.state.hash_len; ++i)
        {
            ds_Assert(AtomicLoadRlx64(g_tmap.state.a_bucket + i) == THASH_NULL);
        }
        ds_TPoolAlloc(*pool, g_arch_config->logical_core_count, 1);
        ds_THashMapFlush(g_tmap);
    }
}

void ds_StructureTHashMapTestFree(void *args)
{
    if (ds_ThreadSelfIndex() == 0 && g_tpool)
    {
	    ds_TPool(ds_Structure) *pool = (ds_TPool(ds_Structure) *) AtomicLoadAcq64(&g_tpool);
        g_tpool = NULL;
        ds_TPoolDealloc(*pool);
        ds_THashMapDealloc(g_tmap);
        free(pool);
        first = 1;
    }
}

/* every worker adds, looks up and removes its own 10000 nodes; the map is shared */
void ds_StructureTHashMapAddAllRemoveAllTest(void *null)
{
	ds_TPool(ds_Structure) *pool = (ds_TPool(ds_Structure) *) AtomicLoadAcq64(&g_tpool);
    const u32 count = 10000;
    u32 indices[10000];
    for (u32 i = 0; i < count; ++i)
    {
        struct slot slot = ds_TPoolIncrement(*pool);
        indices[i] = slot.index;
        struct ds_Structure *n = slot.address;
        n->key = indices[i];
        ds_THashMapAdd(g_tmap, u64Hash(n->key), indices[i]);
    }

    u32 mismatches = 0;
    for (u32 i = 0; i < count; ++i)
    {
        const u64 key = indices[i];
        mismatches += (ds_THashMapLookup(g_tmap, u64Hash(key), &key, ds_StructureKeyEqual).index != indices[i]);
    }
    /* keeps the lookups in optimized builds; checked by the reset */
    AtomicFetchAddRlx32(&g_lookup_mismatches, mismatches);

    for (u32 i = 0; i < count; ++i)
    {
        ds_THashMapRemove(g_tmap, u64Hash(indices[i]), indices[i]);
        ds_TPoolRemove(*pool, indices[i]);
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

#define THASH_TEST_COUNT 1000

static u32 THashMapTestMatches(ds_THashMap(ds_Structure) *map, const u32 *index, const u8 *alive)
{
    for (u32 i = 0; i < THASH_TEST_COUNT; ++i)
    {
        const u64 key = 7*(u64) i;
        const struct slot slot = ds_THashMapLookup(*map, u64Hash(key), &key, ds_StructureKeyEqual);
        if ((alive[i]) ? (slot.index != index[i] || ((struct ds_Structure *) slot.address)->key != key) : (slot.address != NULL))
        {
            return 0;
        }
        /* a key never added */
        const u64 missing = key + 1;
        if (ds_THashMapLookup(*map, u64Hash(missing), &missing, ds_StructureKeyEqual).address != NULL)
        {
            return 0;
        }
    }
    return 1;
}

/*
 * Single thread, few buckets (long chains): adds, lookups, removes (of nodes not in the map too), re-adds
 * of reused slots; serialize round trip over the same pool (LE); corrupt hash_len and truncated streams
 * fail and keep the stream position; flush; dealloc leaves the map empty.
 */
static struct test_Output THashMapSerialTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    ds_TPool(ds_Structure) pool;
    ds_TPoolAlloc(pool, g_arch_config->logical_core_count, 64);
    ds_THashMap(ds_Structure) map;
    TEST_EQUAL(1, ds_THashMapAlloc(env->mem_1, map, &pool, 60));
    TEST_EQUAL(64, map.state.hash_len);

    u32 *index = ArenaPush(env->mem_2, THASH_TEST_COUNT*sizeof(u32));
    u8 *alive = ArenaPushZero(env->mem_2, THASH_TEST_COUNT);
    for (u32 i = 0; i < THASH_TEST_COUNT; ++i)
    {
        const struct slot slot = ds_TPoolAdd(pool);
        struct ds_Structure *n = slot.address;
        n->key = 7*(u64) i;
        ds_THashMapAdd(map, u64Hash(n->key), slot.index);
        index[i] = slot.index;
        alive[i] = 1;
    }
    TEST_TRUE(THashMapTestMatches(&map, index, alive));

    for (u32 i = 0; i < THASH_TEST_COUNT; i += 2)
    {
        const u64 key = 7*(u64) i;
        ds_THashMapRemove(map, u64Hash(key), index[i]);
        /* no longer in the map: a no-op */
        ds_THashMapRemove(map, u64Hash(key), index[i]);
        ds_TPoolRemove(pool, index[i]);
        alive[i] = 0;
    }
    TEST_TRUE(THashMapTestMatches(&map, index, alive));

    /* the free slots come back: their links keep counting tags */
    for (u32 i = 0; i < THASH_TEST_COUNT; i += 4)
    {
        const struct slot slot = ds_TPoolAdd(pool);
        struct ds_Structure *n = slot.address;
        n->key = 7*(u64) i;
        ds_THashMapAdd(map, u64Hash(n->key), slot.index);
        index[i] = slot.index;
        alive[i] = 1;
    }
    TEST_TRUE(THashMapTestMatches(&map, index, alive));

    const u64 size = ds_THashMapSerializeSize(map);
    TEST_EQUAL(sizeof(u32) + 64*sizeof(u64), size);
    struct ss ss = ss_Alloc(env->mem_1, size);
    TEST_NOT_EQUAL(NULL, ss.buf);
    ds_THashMapSerialize(&ss, map);
    TEST_EQUAL(8*size, ss.bit_index);
    TEST_EQUAL(64, ss.buf[0]);
    ss.bit_index = 0;
    ds_THashMap(ds_Structure) read;
    TEST_EQUAL(1, ds_THashMapTryDeserialize(env->mem_3, &ss, read, &pool));
    TEST_EQUAL(8*size, ss.bit_index);
    TEST_TRUE(read.pool == &pool && read.state.hash_len == map.state.hash_len && read.state.hash_mask == map.state.hash_mask);
    TEST_TRUE(THashMapTestMatches(&read, index, alive));

    struct ss cut = ss_Buffered(ss.buf, size - 1);
    TEST_EQUAL(0, ds_THashMapTryDeserialize(env->mem_3, &cut, read, &pool));
    TEST_EQUAL(0, cut.bit_index);
    TEST_EQUAL(NULL, read.state.a_bucket);
    const u32 corrupt_hash_len[] = { 0, 63, 0x80000000 };
    for (u32 c = 0; c < sizeof(corrupt_hash_len) / sizeof(corrupt_hash_len[0]); ++c)
    {
        ss.bit_index = 0;
        ss_WriteU32Le(&ss, corrupt_hash_len[c]);
        ss.bit_index = 0;
        TEST_EQUAL(0, ds_THashMapTryDeserialize(env->mem_3, &ss, read, &pool));
        TEST_EQUAL(0, ss.bit_index);
    }

    ds_THashMapFlush(map);
    for (u32 i = 0; i < THASH_TEST_COUNT; ++i)
    {
        alive[i] = 0;
    }
    TEST_TRUE(THashMapTestMatches(&map, index, alive));

    ds_THashMapDealloc(map);
    TEST_TRUE(map.state.a_bucket == NULL && map.state.hash_len == 0);
    ds_TPoolDealloc(pool);
	return output;
}

static struct test_Output (*THashMap_tests[])(struct test_Environment *) =
{
    THashMapSerialTest,
};

struct suite_Correctness storage_THashMap_correctness_suite =
{
	.id = "THashMap",
	.unit_test = THashMap_tests,
	.unit_test_count = sizeof(THashMap_tests) / sizeof(THashMap_tests[0]),
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
