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

#ifndef __DS_HASH_MAP_H__
#define __DS_HASH_MAP_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"
#include "ds_serialize.h"

#define HASH_NULL 	POOL_NULL	

/*
hashMap
=======
hash map mapping a key to a set of possible indices. User dereference
indices to check for equality between identifiers 
*/

struct ds_HashMap
{
	u32 *		        hash;
	u32 *		        index;
	u32		            hash_len;
	u32		            index_len;
	u32		            hash_mask;
	u32		            growable;
	struct ds_MemSlot	mem_hash;
	struct ds_MemSlot	mem_index;
};

/* allocate hash map on heap if mem == NULL, otherwise push memory onto arena. On failure, returns { 0 }  */
struct ds_HashMap	ds_HashMapAlloc(struct arena *mem, const u32 hash_len, const u32 index_len, const u32 growable);
/* free hash map memory */
void		        ds_HashMapDealloc(struct ds_HashMap *map);
/* flush / reset the hash map */
void		        ds_HashMapFlush(struct ds_HashMap *map);
/* serialize hash map into stream  */
void 		        ds_HashMapSerialize(struct ss *ss, const struct ds_HashMap *map);
/* deserialize and construct hash_map on arena if defined, otherwise alloc on heap. On failure, returns NULL  */
struct ds_HashMap	ds_HashMapDeserialize(struct arena *mem, struct ss *ss, const u32 growable);
/* add the hash(key)-index pair to the hash map. return 1 on success, 0 on out-of-memory. */
u32		            ds_HashMapAdd(struct ds_HashMap *map, const u32 hash, const u32 index);
/* remove  hash(key)-index pair to the hash map. If the pair is not found, do nothing. */
void		        ds_HashMapRemove(struct ds_HashMap *map, const u32 hash, const u32 index);
/* Get the first (hash,index) pair of the map. If HASH_NULL is returned, no more pairs exist */
u32		            ds_HashMapFirst(const struct ds_HashMap *map, const u32 hash);
/* Get the next (hash,index) pair of the map. If HASH_NULL is returnsd, no more pairs exist */
u32		            ds_HashMapNext(const struct ds_HashMap *map, const u32 index);

/*
ThashMap
========
Intrusive hash map with thread-safe operations that can run concurrently. The hash
map assumes that the underlying storage is a TPool. The implementation is based on
Maged M. Michael's paper "High Performance Dynamic Lock-Free Hash Tables and List-Based Sets".

::: Usage and Documentation :::

To declare/define the THashMap functions a struct, say ds_Struct, name and add
the following in the definition of ds_Struct

    struct ds_Struct
    {
        ...
        THASH_NODE;
        ...
        key_Type    key_name;    
    };

and implement the functinos

u32 key_TypePtrHashFunction(const key_Type *key)
{
    // Hash key
}

u32 key_TypePtrCmpFunction(const key_Type *keyA, const key_Type *keyB)
{
    // Return 1 if keyA and keyB are equivalent, otherwise return 0.
}

and write

    THASH_DECLARE(ds_Struct, key_Type)
    THASH_DEFINE(ds_Struct, key_name, key_Type, key_TypePtrHashFunction, key_TypePtrCmpFunction) 

at the appropriate places. THASH_DECLARE defines the ds_StructTHashMap struct and
the ds_StructTHashMap*** functions and THASH_DEFINE generates the implementations
of each function. The functions generated are the following:

    // Allocate hash map on heap if mem == NULL, otherwise push memory onto arena.
    // On failure, returns { 0 }.
    struct ds_StructTHashMap ds_StructTHashMapAlloc(struct arena *mem, 
                                                    struct ds_StructureTPool *pool, 
                                                    const u32 hash_len)

    // Free hash map memory
    void                     ds_StructTHashMapDealloc(struct ds_StructTHashMap *map)

    // Flush/reset the hash map.
    void		             ds_StructTHashMapFlush(struct ds_StructTHashMap *map);

    // Serialize hash map into stream.
    void                     ds_StructTHashMapSerialize(struct ss *ss, const struct ds_StructTHashMap *map);

    // Deserialize and construct hash_map on arena if defined, otherwise alloc on
    // heap. On failure, returns { 0 }.
    struct ds_StructTHashMap ds_StructTHashMapDeserialize(struct arena *mem, struct ss *ss);

    //Return slot with given key if found. Otherwise return (NULL, U32_MAX)
    struct slot              ds_StructTHashMapLookup(struct ds_StructTHashMap *map, const key_Type *key)


    // Add the node (at the given index, with a unique key) to the hash map. 
    // NOTE: The node's key must be unique.
    void                     ds_StructTHashMapAdd(struct ds_StructTHashMap *map, 
                                                  struct ds_Structure *node, 
                                                  const u32 index)

    // Remove node with given key from HashMap. If no such node is found, no-op.
    void                     ds_StructTHashMapRemove(struct ds_StructTHashMap *map, const key_Type *key)

    //TODO Yield/Backoff technique performance boost????
    //
    // Internal function for looking up a node with the given key. If a snapshot in
    // time is found where prev->cur, and cur contains the given key, the function
    // returns 1 and sets teh following inputs:
    //
    //      *a_prev         = &n_prev->a_node_hash
    //      *snapshot_prev  =  n_prev->a_node_hash (snapshot)
    //      *snapshot_cur   =   n_cur->a_node_hash (snapshot)
    //      *n_cur          = NextNode(*snapshot_prev)
    //
    // If no snapshot is found, the function returns 0.
    u32                      ds_StructTHashMapFindSnapshot(u64 **a_prev, 
                                                           u64 *snapshot_prev, 
                                                           u64 *snapshot_cur, 
                                                           struct ds_Struct **n_cur, 
                                                           struct ds_StructTHashMap *map, 
                                                           const key_Type *key)


::: Internals :::

*/

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

#define THASH_DECLARE(struct_name, key_Type)                \
        THASH_STRUCT_DEFINE(struct_name);                   \
        THASH_ALLOC_DECLARE(struct_name);                   \
        THASH_DEALLOC_DECLARE(struct_name);                 \
        THASH_FLUSH_DECLARE(struct_name);                   \
        THASH_SERIALIZE_DECLARE(struct_name);               \
        THASH_DESERIALIZE_DECLARE(struct_name);             \
        THASH_FIND_SNAPSHOT_DECLARE(struct_name, key_Type); \
        THASH_LOOKUP_DECLARE(struct_name, key_Type);        \
        THASH_ADD_DECLARE(struct_name);                     \
        THASH_REMOVE_DECLARE(struct_name, key_Type);

#define THASH_DEFINE(struct_name, key_name, key_Type, key_TypePtrHashFunction, key_TypePtrCmpFunction)              \
        THASH_ALLOC_DEFINE(struct_name)                                                                             \
        THASH_DEALLOC_DEFINE(struct_name)                                                                           \
        THASH_FLUSH_DEFINE(struct_name)                                                                             \
        THASH_SERIALIZE_DEFINE(struct_name)                                                                         \
        THASH_DESERIALIZE_DEFINE(struct_name)                                                                       \
        THASH_FIND_SNAPSHOT_DEFINE(struct_name, key_name, key_Type, key_TypePtrHashFunction, key_TypePtrCmpFunction)\
        THASH_LOOKUP_DEFINE(struct_name, key_Type)                                                                  \
        THASH_ADD_DEFINE(struct_name, key_name, key_TypePtrHashFunction)                                            \
        THASH_REMOVE_DEFINE(struct_name, key_Type) 

#define THASH_STRUCT_DEFINE(struct_name)        \
struct struct_name ## THashMap                  \
{                                               \
    struct struct_name ## TPool *   pool;       \
    u64 *                           a_hash;     \
    u32                             hash_len;   \
    u32                             hash_mask;  \
    struct ds_MemSlot               hash_mem;   \
}

#define THASH_ALLOC_DECLARE(struct_name)                                                                \
struct struct_name ## THashMap                                                                          \
struct_name ## THashMapAlloc(struct arena *mem, struct struct_name ## TPool *pool, const u32 hash_len)

#define THASH_DEALLOC_DECLARE(struct_name)                          \
void                                                                \
struct_name ## THashMapDealloc(struct struct_name ## THashMap *map)

#define THASH_FLUSH_DECLARE(struct_name)                            \
void                                                                \
struct_name ## THashMapFlush(struct struct_name ## THashMap *map) 

#define THASH_SERIALIZE_DECLARE(struct_name)                                                \
void                                                                                        \
struct_name ## THashMapSerialize(struct ss *ss, const struct struct_name ## THashMap *map) 

#define THASH_DESERIALIZE_DECLARE(struct_name)                          \
struct struct_name ## THashMap                                          \
struct_name ## THashMapDeserialize(struct arena *mem, struct ss *ss) 

#define THASH_FIND_SNAPSHOT_DECLARE(struct_name, key_Type)                  \
u32                                                                         \
struct_name ## THashMapFindSnapshot(u64 **a_prev,                           \
                                    u64 *snapshot_prev,                     \
                                    u64 *snapshot_cur,                      \
                                    struct struct_name **n_cur,             \
                                    struct struct_name ## THashMap *map,    \
                                    const key_Type *key)                                         

#define THASH_LOOKUP_DECLARE(struct_name, key_Type)                                     \
struct slot                                                                             \
struct_name ## THashMapLookup(struct struct_name ## THashMap *map, const key_Type *key)

#define THASH_ADD_DECLARE(struct_name)                                                                      \
void                                                                                                        \
struct_name ## THashMapAdd(struct struct_name ## THashMap *map, struct struct_name *node, const u32 index)

#define THASH_REMOVE_DECLARE(struct_name, key_Type)                                     \
void                                                                                    \
struct_name ## THashMapRemove(struct struct_name ## THashMap *map, const key_Type *key)

#define THASH_ALLOC_DEFINE(struct_name)                                                                 \
THASH_ALLOC_DECLARE(struct_name)                                                                        \
{                                                                                                       \
    ds_Assert(hash_len && (hash_len >> 31) == 0);                                                       \
                                                                                                        \
    struct struct_name ## THashMap map;                                                                 \
	if (mem)                                                                                            \
	{                                                                                                   \
		map.hash_len = (u32) PowerOfTwoCeil(hash_len);                                                  \
        map.hash_mem = (struct ds_MemSlot) { 0 };                                                       \
		map.a_hash = ArenaPushAligned(mem, map.hash_len * sizeof(u64), 8);                              \
	}                                                                                                   \
	else                                                                                                \
	{                                                                                                   \
		map.hash_len = (u32) PowerOfTwoCeil( ds_AllocSizeCeil(hash_len * sizeof(u64))  / sizeof(u64) ); \
		map.a_hash = ds_Alloc(&map.hash_mem, map.hash_len * sizeof(u64), HUGE_PAGES);                   \
	}                                                                                                   \
                                                                                                        \
	if (!map.a_hash)                                                                                    \
	{                                                                                                   \
		return (struct struct_name ## THashMap) { 0 };                                                  \
	}                                                                                                   \
                                                                                                        \
    map.pool = pool;                                                                                    \
	map.hash_mask = map.hash_len-1;                                                                     \
    struct_name ## THashMapFlush(&map);                                                                 \
    return map;                                                                                         \
} 

#define THASH_DEALLOC_DEFINE(struct_name)   \
THASH_DEALLOC_DECLARE(struct_name)          \
{                                           \
    if (map->hash_mem.address)              \
    {                                       \
        ds_Free(&map->hash_mem);            \
    }                                       \
} 

#define THASH_FLUSH_DEFINE(struct_name)                 \
THASH_FLUSH_DECLARE(struct_name)                        \
{                                                       \
	for (u32 i = 0; i < map->hash_len; ++i)             \
	{                                                   \
        AtomicStoreRel64(map->a_hash + i, THASH_NULL);  \
	}                                                   \
} 

#define THASH_SERIALIZE_DEFINE(struct_name)                                     \
THASH_SERIALIZE_DECLARE(struct_name)                                            \
{                                                                               \
    const u64 size_required = sizeof(u32) + map->hash_len*sizeof(u64);          \
    if (size_required > ss_BytesLeft(ss))                                       \
    {                                                                           \
        Log(T_SYSTEM, S_ERROR, "Serial stream OOM when Serializing THashMap,    \
                                Trying to write %luB with %luB left.",          \
                                size_required, ss_BytesLeft(ss));               \
        return;                                                                 \
    }                                                                           \
	ss_WriteU32Be(ss, map->hash_len);                                           \
	ss_WriteU64BeN(ss, map->a_hash, map->hash_len);                             \
}

#define THASH_DESERIALIZE_DEFINE(struct_name)                               \
THASH_DESERIALIZE_DECLARE(struct_name)                                      \
{                                                                           \
    struct ss local_ss = *ss;                                               \
	if (sizeof(u32) > ss_BytesLeft(ss))                                     \
	{                                                                       \
		Log(T_SYSTEM, S_ERROR, "Deserializing THashMap past byte boundary:  \
                                Trying to read 4B with %luB left.",         \
                                ss_BytesLeft(ss));                          \
		return (struct struct_name ## THashMap) { 0 };                      \
	}                                                                       \
                                                                            \
	const u32 hash_len = ss_ReadU32Be(ss);                                  \
	ds_Assert(PowerOfTwoCheck(hash_len));                                   \
	if (hash_len*sizeof(u64) > ss_BytesLeft(ss))                            \
	{                                                                       \
		Log(T_SYSTEM, S_ERROR, "Deserializing hash map past byte boundary:  \
                                Trying to read %luB with %luB left.",       \
                                hash_len*sizeof(u64),                       \
                                ss_BytesLeft(ss));                          \
        *ss = local_ss;                                                     \
		return (struct struct_name ## THashMap) { 0 };                      \
    }                                                                       \
                                                                            \
	struct struct_name ## THashMap map =                                    \
    {                                                                       \
        .hash_len = hash_len,                                               \
        .hash_mask = hash_len-1,                                            \
    };                                                                      \
                                                                            \
	map.a_hash = (mem)                                                      \
	    ? ArenaPushAligned(mem, map.hash_len*sizeof(u64), 8)                \
		: ds_Alloc(&map.hash_mem, hash_len*sizeof(u64), HUGE_PAGES);        \
                                                                            \
	if (!map.a_hash)                                                        \
	{                                                                       \
		LogString(T_SYSTEM, S_ERROR, "OOM when deserializing THashMap");    \
        *ss = local_ss;                                                     \
		return (struct struct_name ## THashMap) { 0 };                      \
	}                                                                       \
                                                                            \
	ss_ReadU64BeN(map.a_hash, ss, hash_len);                                \
                                                                            \
	return map;                                                             \
}

#define THASH_FIND_SNAPSHOT_DEFINE(struct_name, key_name, key_Type, key_TypePtrHashFunction, key_TypePtrCmpFunction)\
THASH_FIND_SNAPSHOT_DECLARE(struct_name, key_Type)                                                                  \
{                                                                                                                   \
    const u32 h = key_TypePtrHashFunction(key) & map->hash_mask;                                                    \
try_again:                                                                                                          \
    *a_prev = map->a_hash + h;                                                                                      \
    *snapshot_prev = AtomicLoadAcq64(*a_prev) & (~THASH_MARK_MASK);                                                 \
                                                                                                                    \
    while(1)                                                                                                        \
    {                                                                                                               \
        if (((*snapshot_prev) & THASH_NEXT_MASK) == THASH_NULL)                                                     \
        {                                                                                                           \
            return 0;                                                                                               \
        }                                                                                                           \
                                                                                                                    \
        *n_cur = struct_name ## TPoolAddress(map->pool, THASH_NEXT_MASK & (*snapshot_prev));                        \
        /* TODO: Needed Acq?... */                                                                                  \
        *snapshot_cur = AtomicLoadAcq64(&(*n_cur)->a_hash_node);                                                    \
                                                                                                                    \
        /* If snapshot_prev is marked, OR the link has been updated, try again */                                   \
        /* TODO: Needed Acq?... */                                                                                  \
        const u64 local_prev = AtomicLoadAcq64(*a_prev);                                                            \
        if (local_prev != *snapshot_prev)                                                                           \
        {                                                                                                           \
            goto try_again;                                                                                         \
        }                                                                                                           \
                                                                                                                    \
        /* Zombie node; unlink */                                                                                   \
        if ((*snapshot_cur) & THASH_MARK_MASK)                                                                      \
        {                                                                                                           \
            const u64 new_prev = THASH_TAG_FIELD_INCREMENTED(*snapshot_prev) + (THASH_NEXT_MASK & (*snapshot_cur)); \
            /* TODO: Is this fine?... */                                                                            \
            if (!AtomicCompareExchangeRelRlx64(*a_prev, snapshot_prev, new_prev))                                   \
            {                                                                                                       \
                goto try_again;                                                                                     \
            }                                                                                                       \
            *snapshot_prev = new_prev;                                                                              \
        }                                                                                                           \
        /* node may be 
         * p-to-date, or may have been deleted and re-inserted at this point. */                       \
        else                                                                                                        \
        {                                                                                                           \
            const u32 key_found = key_TypePtrCmpFunction(key, &(*n_cur)->key_name);                                 \
            ds_ReadWriteBarrier;                                                                                    \
            if (AtomicLoadRlx64(*a_prev) != *snapshot_prev)                                                         \
            {                                                                                                       \
                goto try_again;                                                                                     \
            }                                                                                                       \
                                                                                                                    \
            if (key_found)                                                                                          \
            {                                                                                                       \
                return 1;                                                                                           \
            }                                                                                                       \
            *a_prev = &(*n_cur)->a_hash_node;                                                                       \
            *snapshot_prev = (*snapshot_cur) & (~THASH_MARK_MASK);                                                  \
        }                                                                                                           \
    }                                                                                                               \
}

#define THASH_LOOKUP_DEFINE(struct_name, key_Type)                                                          \
THASH_LOOKUP_DECLARE(struct_name, key_Type)                                                                 \
{                                                                                                           \
    u64 *a_prev;                                                                                            \
    u64 snapshot_prev;                                                                                      \
    u64 snapshot_cur;                                                                                       \
    struct struct_name *n_cur;                                                                              \
    return (struct_name ## THashMapFindSnapshot(&a_prev, &snapshot_prev, &snapshot_cur, &n_cur, map, key))  \
            ? (struct slot) { .address = n_cur, .index = THASH_NEXT_MASK & snapshot_prev }                  \
            : (struct slot) { .address = NULL, .index = U32_MAX };                                          \
}
#define THASH_ADD_DEFINE(struct_name, key_name, key_TypePtrHashFunction)                \
THASH_ADD_DECLARE(struct_name)                                                          \
{                                                                                       \
    ds_Assert(struct_name ## THashMapLookup(map, &node->key_name).address == NULL);     \
    ds_Assert(index < THASH_NULL);                                                      \
                                                                                        \
    const u32 h = key_TypePtrHashFunction(&node->key_name) & map->hash_mask;            \
    const u64 tag = THASH_TAG_FIELD_INCREMENTED(AtomicLoadRlx64(&node->a_hash_node));   \
                                                                                        \
    u64 local_hash = AtomicLoadAcq64(map->a_hash + h);                                  \
    u64 new_hash;                                                                       \
    do                                                                                  \
    {                                                                                   \
        new_hash = THASH_TAG_FIELD_INCREMENTED(local_hash) | index;                     \
        const u64 old_next = THASH_NEXT_MASK & local_hash;                              \
        AtomicStoreRlx64(&node->a_hash_node, tag | old_next);                           \
    } while (!AtomicCompareExchangeRelAcq64(map->a_hash + h, &local_hash, new_hash));   \
}

#define THASH_REMOVE_DEFINE(struct_name, key_Type)                                                          \
THASH_REMOVE_DECLARE(struct_name, key_Type)                                                                 \
{                                                                                                           \
    u64 *a_prev;                                                                                            \
    u64 snapshot_prev;                                                                                      \
    u64 snapshot_cur;                                                                                       \
    struct struct_name *n_cur;                                                                              \
    while (1)                                                                                               \
    {                                                                                                       \
        if (!struct_name ## THashMapFindSnapshot(&a_prev, &snapshot_prev, &snapshot_cur, &n_cur, map, key)) \
        {                                                                                                   \
            return;                                                                                         \
        }                                                                                                   \
                                                                                                            \
        const u64 new_cur = (snapshot_cur + THASH_TAG_INCREMENT) | THASH_MARK_MASK;                         \
        if (!AtomicCompareExchangeRelRlx64(&n_cur->a_hash_node, &snapshot_cur, new_cur))                    \
        {                                                                                                   \
            continue;                                                                                       \
        }                                                                                                   \
                                                                                                            \
        const u64 new_prev = THASH_TAG_FIELD_INCREMENTED(snapshot_prev) + (THASH_NEXT_MASK & snapshot_cur); \
        if (!AtomicCompareExchangeRelRlx64(a_prev, &snapshot_prev, new_prev))                               \
        {                                                                                                   \
            struct_name ## THashMapFindSnapshot(&a_prev, &snapshot_prev, &snapshot_cur, &n_cur, map, key);  \
        }                                                                                                   \
    }                                                                                                       \
}

#ifdef __cplusplus
} 
#endif

#endif
