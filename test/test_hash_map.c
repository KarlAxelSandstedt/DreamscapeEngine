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

#define XXH_INLINE_ALL
#include "xxhash.h"
#include "ds_random.h" 

#define THASH_NULL      THASH_NEXT_MASK
/* [ABATag(32) | MarkedForRemoval(1) | Index(31) ] */
#define THASH_NODE      u64 a_hash_node

#define THASH_TAG_INCREMENT ((u64) 0x0000000100000000)
#define THASH_TAG_MASK      ((u64) 0xffffffff00000000)
#define THASH_MARK_MASK     ((u64) 0x0000000080000000)
#define THASH_NEXT_MASK     ((u64) 0x000000007fffffff)

#define THASH_TAG_FIELD_INCREMENTED(hash_node)      \
                    ((THASH_TAG_MASK & hash_node)   \
                    + THASH_TAG_INCREMENT)

struct ds_Structure
{
    THASH_NODE;
    TPOOL_NODE;

    u64     key;
};

TPOOL_DECLARE(ds_Structure);
TPOOL_DEFINE(ds_Structure);

struct THashMap
{
    struct ds_StructureTPool *  pool;
    u64 *                       a_hash;
    u32                         hash_len;
    u32                         hash_mask;
    struct ds_MemSlot           hash_mem;
};

struct THashMap THashMapAlloc(struct arena *mem, struct ds_StructureTPool *pool, const u32 hash_len);
void            THashMapFree(struct THashMap *map);
void            THashMapFlush(struct THashMap *map);
void            THashMapSerialize(struct ss *ss, const struct THashMap *map);
struct THashMap THashMapDeserialize(struct arena *mem, struct ss *ss);

struct THashMap THashMapAlloc(struct arena *mem, struct ds_StructureTPool *pool, const u32 hash_len)
{
	ds_Assert(hash_len && (hash_len >> 31) == 0);

    struct THashMap map;

	if (mem)
	{
		map.hash_len = (u32) PowerOfTwoCeil(hash_len);
        map.hash_mem = (struct ds_MemSlot) { 0 };
		map.a_hash = ArenaPushAligned(mem, map.hash_len * sizeof(u64), 8); 
	}
	else
	{
		map.hash_len = (u32) PowerOfTwoCeil( ds_AllocSizeCeil(hash_len * sizeof(u64))  / sizeof(u64) );
		map.a_hash = ds_Alloc(&map.hash_mem, map.hash_len * sizeof(u64), HUGE_PAGES);
	}

	if (!map.a_hash)
	{
		return (struct THashMap) { 0 };
	}

    map.pool = pool;
	map.hash_mask = map.hash_len-1;
    THashMapFlush(&map);
    return map;
}

void THashMapFree(struct THashMap *map)
{
    if (map->hash_mem.address)
    {
        ds_Free(&map->hash_mem);
    }
}

void THashMapFlush(struct THashMap *map)
{
	for (u32 i = 0; i < map->hash_len; ++i)
	{
        AtomicStoreRel64(map->a_hash + i, THASH_NULL);
	}
}

void THashMapSerialize(struct ss *ss, const struct THashMap *map)
{
    const u64 size_required = sizeof(u32) + map->hash_len*sizeof(u64);
    if (size_required > ss_BytesLeft(ss))
    {
        Log(T_SYSTEM, S_ERROR, "Serial stream OOM when Serializing THashMap, Trying to write %luB with %luB left.", size_required, ss_BytesLeft(ss));
        return;
    }
	ss_WriteU32Be(ss, map->hash_len);
	ss_WriteU64BeN(ss, map->a_hash, map->hash_len);
}

struct THashMap THashMapDeserialize(struct arena *mem, struct ss *ss)
{
    struct ss local_ss = *ss;
	if (sizeof(u32) > ss_BytesLeft(ss))
	{
		Log(T_SYSTEM, S_ERROR, "Deserializing THashMap past byte boundary: Trying to read 4B with %luB left.", ss_BytesLeft(ss));
		return (struct THashMap) { 0 };
	}

	const u32 hash_len = ss_ReadU32Be(ss);
	ds_Assert(PowerOfTwoCheck(hash_len));
	if (hash_len*sizeof(u64) > ss_BytesLeft(ss))
	{
		Log(T_SYSTEM, S_ERROR, "Deserializing hash map past byte boundary: Trying to read %luB with %luB left.", hash_len*sizeof(u64), ss_BytesLeft(ss));
        *ss = local_ss;
		return (struct THashMap) { 0 };
    }

	struct THashMap map = 
    {
        .hash_len = hash_len,
        .hash_mask = hash_len-1,
    };

	map.a_hash = (mem)
	    ? ArenaPushAligned(mem, map.hash_len*sizeof(u64), 8)
		: ds_Alloc(&map.hash_mem, hash_len*sizeof(u64), HUGE_PAGES);

	if (!map.a_hash)
	{
		LogString(T_SYSTEM, S_ERROR, "OOM when deserializing THashMap");
        *ss = local_ss;
		return (struct THashMap) { 0 };
	}

	ss_ReadU64BeN(map.a_hash, ss, hash_len);

	return map;
}

//TODO Yield/Backoff technique performance boost????
/*
 * If a snapshot in time is found where prev->cur, and cur contains the input key, Return 1. Otherwise return 0.
 */
u32 THashMapFindSnapshot(u64 **a_prev, u64 *snapshot_prev, u64 *snapshot_cur, struct ds_Structure **n_cur, struct THashMap *map, const u64 *key)
{
    const u32 h = ((u32) XXH3_64bits(key, sizeof(*key))) & map->hash_mask;
try_again:
    *a_prev = map->a_hash + h; 
    *snapshot_prev = AtomicLoadAcq64(*a_prev) & (~THASH_MARK_MASK);

    while(1)
    {
        if (((*snapshot_prev) & THASH_NEXT_MASK) == THASH_NULL)
        {
            return 0;
        }

        *n_cur = ds_StructureTPoolAddress(map->pool, THASH_NEXT_MASK & (*snapshot_prev));
        /* TODO: ... */
        *snapshot_cur = AtomicLoadAcq64(&(*n_cur)->a_hash_node);
       
        /* If snapshot_prev is marked, OR the link has been updated, try again */
        /* Acquire to get RW-reorder barrier */
        const u64 local_prev = AtomicLoadAcq64(*a_prev);
        if (local_prev != *snapshot_prev)
        {
            goto try_again;
        }

        /* Zombie node; unlink */
        if ((*snapshot_cur) & THASH_MARK_MASK)
        {
            const u64 new_prev = THASH_TAG_FIELD_INCREMENTED(*snapshot_prev) + (THASH_NEXT_MASK & (*snapshot_cur));
            /* TODO: ... */
            if (!AtomicCompareExchangeRelRlx64(*a_prev, snapshot_prev, new_prev))
            {
                goto try_again;
            }
            *snapshot_prev = new_prev;
        }
        /* node may be up-to-date, or may have been deleted and re-inserted at this point. */
        else
        {
            const int key_diff = memcmp(key, &(*n_cur)->key, sizeof(*key));
            ds_ReadWriteBarrier;
            if (AtomicLoadRlx64(*a_prev) != *snapshot_prev)
            {
                goto try_again;
            }

            if (key_diff == 0)
            {
                return 1;
            }
            *a_prev = &(*n_cur)->a_hash_node;
            *snapshot_prev = (*snapshot_cur) & (~THASH_MARK_MASK);
        }
    }
}

struct slot THashMapLookup(struct THashMap *map, const u64 *key)
{
    u64 *a_prev;
    u64 snapshot_prev; 
    u64 snapshot_cur;
    struct ds_Structure *n_cur;
    return (THashMapFindSnapshot(&a_prev, &snapshot_prev, &snapshot_cur, &n_cur, map, key))
            ? (struct slot) { .address = n_cur, .index = THASH_NEXT_MASK & snapshot_prev }
            : (struct slot) { .address = NULL, .index = U32_MAX };
}

void THashMapAdd(struct THashMap *map, struct ds_Structure *node, const u32 index)
{
    ds_Assert(THashMapLookup(map, &node->key).address == NULL);
    ds_Assert(index < THASH_NULL);

    const u32 h = ((u32) XXH3_64bits(&node->key, sizeof(node->key))) & map->hash_mask;
    const u64 tag = THASH_TAG_FIELD_INCREMENTED(AtomicLoadRlx64(&node->a_hash_node));

    u64 local_hash = AtomicLoadAcq64(map->a_hash + h);
    u64 new_hash;
    do
    {
        new_hash = THASH_TAG_FIELD_INCREMENTED(local_hash) | index;
        const u64 old_next = THASH_NEXT_MASK & local_hash;
        AtomicStoreRlx64(&node->a_hash_node, tag | old_next);
    } while (!AtomicCompareExchangeRelAcq64(map->a_hash + h, &local_hash, new_hash));
}

void THashMapRemove(struct THashMap *map, const u64 *key)
{
    u64 *a_prev;
    u64 snapshot_prev; 
    u64 snapshot_cur;
    struct ds_Structure *n_cur;
    while (1)
    {
        if (!THashMapFindSnapshot(&a_prev, &snapshot_prev, &snapshot_cur, &n_cur, map, key))
        {
            return;
        }

        const u64 new_cur = (snapshot_cur + THASH_TAG_INCREMENT) | THASH_MARK_MASK;
        if (!AtomicCompareExchangeRelRlx64(&n_cur->a_hash_node, &snapshot_cur, new_cur))
        {
            continue;
        }

        const u64 new_prev = THASH_TAG_FIELD_INCREMENTED(snapshot_prev) + (THASH_NEXT_MASK & snapshot_cur);
        if (!AtomicCompareExchangeRelRlx64(a_prev, &snapshot_prev, new_prev))
        {
            THashMapFindSnapshot(&a_prev, &snapshot_prev, &snapshot_cur, &n_cur, map, key);
        }
    }
}

static struct THashMap g_tmap;
static struct ds_StructureTPool *g_tpool;
static u32 first = 1;

void *ds_StructureTHashMapTestInit(void)
{
    if (ds_ThreadSelfIndex() == 0)
    {
        AtomicStoreRel64(&g_tpool, malloc(sizeof(struct ds_StructureTPool)));
        g_tmap = THashMapAlloc(NULL, g_tpool, 4096);
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
        THashMapFlush(&g_tmap);
    }
}

void ds_StructureTHashMapTestFree(void *args)
{
    if (ds_ThreadSelfIndex() == 0 && g_tpool)
    {
	    struct ds_StructureTPool *pool = AtomicLoadAcq64(&g_tpool);
        g_tpool = NULL;
        ds_StructureTPoolDealloc(pool);
        THashMapFree(&g_tmap);
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
        THashMapAdd(&g_tmap, n, indices[i]);
    }

    for (u32 i = 0; i < count; ++i)
    {
        const u64 key = indices[i]; 
        THashMapRemove(&g_tmap, &key);
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



