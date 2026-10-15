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

/*
ds_hash_map.h
=============
Hash maps from a 32-bit hash to indices of the caller's storage; the caller compares the keys.

Contents (in this order):
    ds_HashMap  hash -> chain of indices (u32 arrays), for pools and arrays; single-threaded, serializable.
    THashMap    ds_THashMap(T): the same, lock-free and intrusive, over the nodes of a ds_TPool(T).
*/

#ifndef __DS_HASH_MAP_H__
#define __DS_HASH_MAP_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"
#include "ds_serialize.h"

#define HASH_NULL 	POOL_NULL	

/*
ds_HashMap
----------
    struct ds_HashMap map = ds_HashMapAlloc(NULL, 256, 256, GROWABLE);
    ds_HashMapAdd(&map, Utf8Hash(id), index);
    for (u32 i = ds_HashMapFirst(&map, Utf8Hash(id)); i != HASH_NULL; i = ds_HashMapNext(&map, i))
        if (Utf8Equivalence(id, nodes[i].id)) { ... }
    ds_HashMapRemove(&map, Utf8Hash(id), index);

hash[hash & hash_mask] heads a chain linked through index[], so an index is in at most one chain at a
time; hash_len is a power of two and never changes, index_len >= hash_len grows (growable maps) to fit the
largest index added. Not thread-safe.
*/

struct ds_HashMap
{
	u32 *		        hash;
	u32 *		        index;
	u32		            hash_len;
	u32		            index_len;
	u32		            hash_mask;
	u32		            growable;
	u32		            heap_allocated;	/* hash and index are ds_Alloc blocks */
};

/*
Return an empty map on mem (fixed; growable must be 0) or on the heap (mem == NULL): hash_len (> 0, < 2^31)
rounded up to a power of two, index_len (> 0) raised to at least that. On failure, a map with hash NULL.
*/
struct ds_HashMap	ds_HashMapAlloc(struct arena *mem, const u32 hash_len, const u32 index_len, const u32 growable);
/*
ds_HashMapAlloc with exact lengths (hash_len a power of two <= index_len < 2^31) and both arrays
uninitialized: for filling them by hand, as deserialization does.
*/
struct ds_HashMap	ds_HashMapAllocEx(struct arena *mem, const u32 hash_len, const u32 index_len, const u32 growable);
/* Free a heap map's memory (an arena map's stays with the arena); the map becomes empty (hash NULL). */
void		        ds_HashMapDealloc(struct ds_HashMap *map);
/* Remove all (hash, index) pairs. */
void		        ds_HashMapFlush(struct ds_HashMap *map);
/* Bytes ds_HashMapSerialize writes. */
u64                 ds_HashMapSerializeSize(const struct ds_HashMap *map);
/*
Write the map to ss (at a byte boundary; it must fit: ds_HashMapSerializeSize). Format (LE): u32 hash_len,
u32 index_len, the hash array, the index array.
*/
void 		        ds_HashMapSerialize(struct ss *ss, const struct ds_HashMap *map);
/*
Read a map from ss into mem (fixed) or the heap (mem == NULL; growable allowed). Returns 1 on success; 0 if
the memory can't be allocated or ss holds no valid map (map has hash NULL, ss's position unchanged).
*/
u32                 ds_HashMapTryDeserialize(struct arena *mem, struct ss *ss, struct ds_HashMap *map, const u32 growable);
/*
Add the (hash, index) pair; index (< 2^31) must not be in the map. Returns 1 on success, 0 if index >=
index_len in a fixed map.
*/
u32		            ds_HashMapAdd(struct ds_HashMap *map, const u32 hash, const u32 index);
/* Remove the (hash, index) pair (index < index_len); no-op if it isn't in the map, also under another hash. */
void		        ds_HashMapRemove(struct ds_HashMap *map, const u32 hash, const u32 index);
/* Return the first index in the hash's chain, or HASH_NULL. */
u32		            ds_HashMapFirst(const struct ds_HashMap *map, const u32 hash);
/* Return the next index in the chain after index, or HASH_NULL. */
u32		            ds_HashMapNext(const struct ds_HashMap *map, const u32 index);

/*
THashMap
--------
Lock-free intrusive map from a 32-bit hash to the nodes of a ds_TPool(T): adds, removes and lookups may run
on any threads at the same time (Maged M. Michael, "High Performance Dynamic Lock-Free Hash Tables and
List-Based Sets"). As with ds_HashMap, the caller hashes the key; a node is added and removed by its pool
index, and only Lookup compares keys, through the caller's equal function.

    struct node { TPOOL_NODE; THASH_NODE; u64 key; };
    DEFINE_TPOOL(node);
    DEFINE_THASH(node);
    static u32 NodeKeyEqual(const void *node, const void *key)
    {
        return ((const struct node *) node)->key == *(const u64 *) key;
    }

    ds_THashMap(node) map;
    ds_THashMapAlloc(NULL, map, &pool, 4096);
    ds_THashMapAdd(map, hash, slot.index);                              (any thread)
    const struct slot found = ds_THashMapLookup(map, hash, &key, NodeKeyEqual);   (any thread)
    ds_THashMapRemove(map, hash, slot.index);                           (any thread)
    ds_THashMapDealloc(map);

struct T contains THASH_NODE (its field belongs to the map and needs no initialization). DEFINE_THASH(T)
defines ds_THashMap(T) (once, after DEFINE_TPOOL(T)); the macros below work on any THashMap type.

The macros (map is a ds_THashMap(T) lvalue; arguments may be evaluated more than once; Alloc, Dealloc, Flush,
Serialize and TryDeserialize only while no other thread uses the map):

    // Set up hash_len (> 0, < 2^31; rounded up to a power of two) empty buckets over pool (a ds_TPool(T) *),
    // on mem (fixed) or the heap (mem == NULL). Returns 1, or 0 if the memory can't be allocated (the map
    // is then empty: state.a_bucket NULL).
    u32             ds_THashMapAlloc(mem, map, pool, hash_len)

    // Free a heap map's buckets (an arena map's stay with the arena); the map becomes empty. The nodes stay
    // in the pool.
    void            ds_THashMapDealloc(map)

    // Empty every bucket.
    void            ds_THashMapFlush(map)

    // Add pool slot index (< 2^31) under hash. index must not be in the map, and the node's key in no
    // other node (Lookup would return either).
    void            ds_THashMapAdd(map, hash, index)

    // Return { address, index } of a node under hash that equal(node, key) accepts, or { NULL, U32_MAX }.
    // equal may run on a node another thread is removing or reusing; it must only read the node (its
    // answer is then discarded).
    struct slot     ds_THashMapLookup(map, hash, key, equal)

    // Remove the node index added under hash; no-op if it isn't in the map. Returns once the node is
    // unlinked, so the caller may then give it back to the pool (springclean 81: to be verified by stress
    // tests).
    void            ds_THashMapRemove(map, hash, index)

    // Bytes ds_THashMapSerialize writes.
    u64             ds_THashMapSerializeSize(map)

    // Write the buckets to ss (at a byte boundary; they must fit). Format (LE): u32 hash_len, the u64
    // buckets. The nodes' links go with their pool.
    void            ds_THashMapSerialize(ss, map)

    // Read the buckets from ss over pool (which must hold the same nodes) into mem (fixed) or the heap
    // (mem == NULL). Returns 1; 0 if the memory can't be allocated or ss holds no valid map (hash_len 0,
    // not a power of two, or truncated): the map is empty, ss's position unchanged.
    u32             ds_THashMapTryDeserialize(mem, ss, map, pool)
*/

/* a node's link, and a bucket: [tag (32) | removed mark (1) | next index (31)], THASH_NULL ends a chain */
#define THASH_NODE          u64 a_thash_next
#define THASH_NULL          ((u64) 0x000000007fffffff)

/* Internal: the type-independent part of a THashMap. */
struct ds_THashState
{
	u64 *	a_bucket;
	u32	hash_len;
	u32	hash_mask;
	u32	heap_allocated;	/* a_bucket is a ds_Alloc block */
};

#define ds_THashMap(T) ds_THashMap_ ## T

#define DEFINE_THASH(T)                         \
typedef struct ds_THashMap(T)                   \
{                                               \
	struct ds_THashState	state;              \
	ds_TPool(T) *		pool;                   \
} ds_THashMap(T)

/* Internal: see the macros above. */
u32  ds_THashMapAllocInternal(struct arena *mem, struct ds_THashState *state, const u32 hash_len);
void ds_THashMapDeallocInternal(struct ds_THashState *state);
void ds_THashMapFlushInternal(struct ds_THashState *state);
void ds_THashMapSerializeInternal(struct ss *ss, const struct ds_THashState *state);
u32  ds_THashMapTryDeserializeInternal(struct arena *mem, struct ss *ss, struct ds_THashState *state);
static ds_ForceInline u32 ds_THashFindInternal(u64 **a_prev, u64 *prev, u64 *cur, u32 *found, const struct ds_THashState *state, const struct ds_TPoolState *pool, void *const *block, const u64 slot_size, const u64 link_offset, const u32 hash, const u32 index, const void *key, u32 (*equal)(const void *, const void *));
static ds_ForceInline void ds_THashMapAddInternal(const struct ds_THashState *state, const struct ds_TPoolState *pool, void *const *block, const u64 slot_size, const u64 link_offset, const u32 hash, const u32 index);
static ds_ForceInline struct slot ds_THashMapLookupInternal(const struct ds_THashState *state, const struct ds_TPoolState *pool, void *const *block, const u64 slot_size, const u64 link_offset, const u32 hash, const void *key, u32 (*equal)(const void *, const void *));
static ds_ForceInline void ds_THashMapRemoveInternal(const struct ds_THashState *state, const struct ds_TPoolState *pool, void *const *block, const u64 slot_size, const u64 link_offset, const u32 hash, const u32 index);

/* Internal: byte offset of the link in T (a constant; never reads the pool, see ds_PoolSlotOffset). */
#define ds_THashLinkOffset(map)	((u64) &((1 ? NULL : (map).pool->block[0])->a_thash_next))

#define ds_THashMapPoolArgs(map)                                                                        \
	&(map).pool->state, (void *const *) (map).pool->block, sizeof((map).pool->block[0][0]), ds_THashLinkOffset(map)

#define ds_THashMapAlloc(mem, map, __pool, hash_len)                                                    \
	((map).pool = (__pool), ds_THashMapAllocInternal((mem), &(map).state, (hash_len)))

#define ds_THashMapDealloc(map)	ds_THashMapDeallocInternal(&(map).state)
#define ds_THashMapFlush(map)	ds_THashMapFlushInternal(&(map).state)

#define ds_THashMapAdd(map, hash, index)                                                                \
	ds_THashMapAddInternal(&(map).state, ds_THashMapPoolArgs(map), (hash), (index))

#define ds_THashMapLookup(map, hash, key, equal)                                                        \
	ds_THashMapLookupInternal(&(map).state, ds_THashMapPoolArgs(map), (hash), (key), (equal))

#define ds_THashMapRemove(map, hash, index)                                                             \
	ds_THashMapRemoveInternal(&(map).state, ds_THashMapPoolArgs(map), (hash), (index))

#define ds_THashMapSerializeSize(map)	(sizeof(u32) + (u64) (map).state.hash_len*sizeof(u64))
#define ds_THashMapSerialize(ss, map)	ds_THashMapSerializeInternal((ss), &(map).state)
#define ds_THashMapTryDeserialize(mem, ss, map, __pool)                                                 \
	((map).pool = (__pool), ds_THashMapTryDeserializeInternal((mem), (ss), &(map).state))

/******************************** THashMap inline definitions ********************************/

#define THASH_NEXT_MASK     ((u64) 0x000000007fffffff)
#define THASH_MARK          ((u64) 0x0000000080000000)
#define THASH_TAG_MASK      ((u64) 0xffffffff00000000)
#define THASH_TAG_ONE       ((u64) 0x0000000100000000)
/* link value with the tag incremented: a changed link never compares equal to an old snapshot (no ABA) */
#define THASH_RETAG(link)   (((link) & THASH_TAG_MASK) + THASH_TAG_ONE)

static ds_ForceInline u64 *ds_THashLinkInternal(const struct ds_TPoolState *pool, void *const *block, const u32 index, const u64 slot_size, const u64 link_offset)
{
	return (u64 *) (ds_TPoolAddressInternal(pool, block, index, slot_size) + link_offset);
}

/*
 * Walk the bucket of hash for the node index (equal == NULL) or a node equal() accepts, unlinking the
 * removed (marked) nodes passed. Found: returns 1 with *a_prev the link that points at it, *prev that
 * link's value, *cur the node's own link value and *found its index; else 0. A link that changed under
 * the walk restarts it from the bucket.
 */
static ds_ForceInline u32 ds_THashFindInternal(u64 **a_prev, u64 *prev, u64 *cur, u32 *found, const struct ds_THashState *state, const struct ds_TPoolState *pool, void *const *block, const u64 slot_size, const u64 link_offset, const u32 hash, const u32 index, const void *key, u32 (*equal)(const void *, const void *))
{
try_again:
	*a_prev = state->a_bucket + (hash & state->hash_mask);
	*prev = AtomicLoadAcq64(*a_prev);
	while ((*prev & THASH_NEXT_MASK) != THASH_NULL)
	{
		const u32 i = (u32) (*prev & THASH_NEXT_MASK);
		u64 *a_cur = ds_THashLinkInternal(pool, block, i, slot_size, link_offset);
		*cur = AtomicLoadAcq64(a_cur);
		/* prev was marked, relinked or reused since we read it: cur may not follow it */
		if (AtomicLoadAcq64(*a_prev) != *prev)
		{
			goto try_again;
		}

		if (*cur & THASH_MARK)
		{
			const u64 unlinked = THASH_RETAG(*prev) | (*cur & THASH_NEXT_MASK);
			if (!AtomicCompareExchangeRelRlx64(*a_prev, prev, unlinked))
			{
				goto try_again;
			}
			*prev = unlinked;
			continue;
		}

		if (equal)
		{
			const u32 match = equal((u8 *) a_cur - link_offset, key);
			/* the key reads complete before the re-check (LoadLoad): a reused node fails it */
			ds_MemoryFenceAcquire;
			if (AtomicLoadRlx64(*a_prev) != *prev)
			{
				goto try_again;
			}
			if (match)
			{
				*found = i;
				return 1;
			}
		}
		else if (i == index)
		{
			*found = i;
			return 1;
		}

		*a_prev = a_cur;
		*prev = *cur;
	}

	return 0;
}

static ds_ForceInline void ds_THashMapAddInternal(const struct ds_THashState *state, const struct ds_TPoolState *pool, void *const *block, const u64 slot_size, const u64 link_offset, const u32 hash, const u32 index)
{
	ds_Assert(index < THASH_NULL);
	u64 *a_node = ds_THashLinkInternal(pool, block, index, slot_size, link_offset);
	u64 *a_bucket = state->a_bucket + (hash & state->hash_mask);
	const u64 tag = THASH_RETAG(AtomicLoadRlx64(a_node));
	u64 head = AtomicLoadAcq64(a_bucket);
	u64 new_head;
	do
	{
		AtomicStoreRlx64(a_node, tag | (head & THASH_NEXT_MASK));
		new_head = THASH_RETAG(head) | index;
	} while (!AtomicCompareExchangeAcqRelAcq64(a_bucket, &head, new_head));
}

static ds_ForceInline struct slot ds_THashMapLookupInternal(const struct ds_THashState *state, const struct ds_TPoolState *pool, void *const *block, const u64 slot_size, const u64 link_offset, const u32 hash, const void *key, u32 (*equal)(const void *, const void *))
{
	u64 *a_prev;
	u64 prev, cur;
	u32 found;
	if (ds_THashFindInternal(&a_prev, &prev, &cur, &found, state, pool, block, slot_size, link_offset, hash, U32_MAX, key, equal))
	{
		return (struct slot) { .address = ds_TPoolAddressInternal(pool, block, found, slot_size), .index = found };
	}
	return (struct slot) { .address = NULL, .index = U32_MAX };
}

static ds_ForceInline void ds_THashMapRemoveInternal(const struct ds_THashState *state, const struct ds_TPoolState *pool, void *const *block, const u64 slot_size, const u64 link_offset, const u32 hash, const u32 index)
{
	u64 *a_prev;
	u64 prev, cur;
	u32 found;
	while (ds_THashFindInternal(&a_prev, &prev, &cur, &found, state, pool, block, slot_size, link_offset, hash, index, NULL, NULL))
	{
		/* mark: from now on nobody links past the node's link, and searches unlink it */
		u64 *a_cur = ds_THashLinkInternal(pool, block, index, slot_size, link_offset);
		const u64 marked = THASH_RETAG(cur) | THASH_MARK | (cur & THASH_NEXT_MASK);
		if (!AtomicCompareExchangeRelRlx64(a_cur, &cur, marked))
		{
			continue;
		}

		const u64 unlinked = THASH_RETAG(prev) | (cur & THASH_NEXT_MASK);
		if (!AtomicCompareExchangeRelRlx64(a_prev, &prev, unlinked))
		{
			/* a search for index passes the whole chain and unlinks the marked node on the way */
			ds_THashFindInternal(&a_prev, &prev, &cur, &found, state, pool, block, slot_size, link_offset, hash, index, NULL, NULL);
		}
		return;
	}
}

#ifdef __cplusplus
} 
#endif

#endif
