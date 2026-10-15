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
string_database.h
=================
A typed database of nodes keyed by a utf8 id: a ds_Pool(T) (stable indices as handles), a hash map from
Utf8Hash(id) to indices, and a list of the allocated nodes. T is a struct that starts with SDB_NODE.

    SDB_DECLARE(T) / SDB_DEFINE(T)  generate T ## SDB and T ## SDBAlloc, ... below.

- Index SDB_STUB (0) is the stub node, created by Alloc and Flush, with the empty id. Lookup returns it
  for an id that is not in the database, so the empty id can't be added. Reference and Dereference leave
  the stub's count alone: callers use the stub as a default they reference freely.
- Ids are copied onto an arena (SDBAdd) or aliased (SDBAddAndAlias: the caller keeps the bytes alive for as
  long as the node exists). The database frees no id.
- reference_count is the callers' bookkeeping: Remove removes a node whatever its count.
- Not thread-safe.
*/

#ifndef __STRING_DATABASE_H__
#define __STRING_DATABASE_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_hash_map.h"
#include "ds_list.h"


#define SDB_STUB	0

#define SDB_NODE                        \
    utf8                id;             \
    u32                 reference_count;\
    struct DLLNode   strdb_allocated;\
    POOL_NODE

#define DEFINE_SDB_STRUCT(T)            \
typedef struct T ## SDB                 \
{                                       \
	struct ds_HashMap 	hash;           \
	ds_Pool(T)		pool;           \
	struct DLL	    allocated_list; \
} T ## SDB

#define SDB_DECLARE(T)                  \
    DEFINE_POOL(T);                    \
    DEFINE_SDB_STRUCT(T);               \
    DECLARE_SDB_ALLOC(T);               \
    DECLARE_SDB_DEALLOC(T);             \
    DECLARE_SDB_FLUSH(T);               \
    DECLARE_SDB_ADD(T);                 \
    DECLARE_SDB_ADD_AND_ALIAS(T);       \
    DECLARE_SDB_REMOVE(T);              \
    DECLARE_SDB_LOOKUP(T);              \
    DECLARE_SDB_REFERENCE(T);           \
    DECLARE_SDB_DEREFERENCE(T)          

#define SDB_DEFINE(T)                   \
    DEFINE_SDB_ALLOC(T)                 \
    DEFINE_SDB_DEALLOC(T)               \
    DEFINE_SDB_FLUSH(T)                 \
    DEFINE_SDB_ADD(T)                   \
    DEFINE_SDB_ADD_AND_ALIAS(T)         \
    DEFINE_SDB_REMOVE(T)                \
    DEFINE_SDB_LOOKUP(T)                \
    DEFINE_SDB_REFERENCE(T)             \
    DEFINE_SDB_DEREFERENCE(T)          

/* Allocate and set up the database (mem and growable as for pools); initial_length includes the stub. Fatal on failure. */
#define DECLARE_SDB_ALLOC(T)    \
T ## SDB    T ## SDBAlloc(struct arena *mem, const u32 initial_length, const u32 growable)

/* Deallocate the database; no id is freed. */
#define DECLARE_SDB_DEALLOC(T)    \
void        T ## SDBDealloc(T ## SDB *db)

/* Remove every node but a fresh stub. */
#define DECLARE_SDB_FLUSH(T)    \
void        T ## SDBFlush(T ## SDB *db)

/*
 * Add a node whose id is a copy of id_to_copy on mem; reference count 0. Returns its slot, or
 * { U32_MAX, NULL } if the id is empty or already present, the pool or hash map is full, or the copy fails.
 */
#define DECLARE_SDB_ADD(T)    \
struct slot T ## SDBAdd(struct arena *mem, T ## SDB *db, const utf8 id_to_copy)

/*
 * Add a node whose id aliases id_to_alias's bytes; reference count 0. Returns its slot, or { U32_MAX, NULL }
 * if the id is empty or already present, or the pool or hash map is full.
 */
#define DECLARE_SDB_ADD_AND_ALIAS(T)    \
struct slot T ## SDBAddAndAlias(T ## SDB *db, const utf8 id_to_alias)

/* Remove the id's node if present (whatever its reference count); the stub is never removed. */
#define DECLARE_SDB_REMOVE(T)    \
void        T ## SDBRemove(T ## SDB *db, const utf8 id)

/* Return the id's slot, or the stub's (SDB_STUB) if the id is not present. */
#define DECLARE_SDB_LOOKUP(T)   \
struct slot T ## SDBLookup(const T ## SDB *db, const utf8 id)

/* Lookup, and increment the found node's reference count (not the stub's). */
#define DECLARE_SDB_REFERENCE(T)    \
struct slot T ## SDBReference(T ## SDB *db, const utf8 id)

/* Decrement the node's reference count (not the stub's); index must be an allocated node or SDB_STUB. */
#define DECLARE_SDB_DEREFERENCE(T)    \
void        T ## SDBDereference(T ## SDB *db, const u32 index)

#define DEFINE_SDB_ALLOC(T)                                                                         \
DECLARE_SDB_ALLOC(T)                                                                                \
{                                                                                                   \
    struct T ## SDB db = { 0 };                                                                     \
    db.hash = ds_HashMapAlloc(mem, initial_length, initial_length, growable);                       \
    ds_PoolAlloc(mem, db.pool, initial_length, growable);                                        \
    DLLFlush(db.allocated_list);                                                                 \
                                                                                                    \
	if (!db.hash.hash || !db.pool.length)                                                           \
	{                                                                                               \
		LogString(T_SYSTEM, S_FATAL, "Failed to allocate string_database");                         \
		FatalCleanupAndExit();                                                                      \
	}                                                                                               \
                                                                                                    \
    const utf8 stub_id = Utf8Empty();                                                               \
	const u32 key = Utf8Hash(stub_id);                                                              \
                                                                                                    \
	struct slot slot = ds_PoolAdd(db.pool);                                                      \
	ds_Assert(slot.index == SDB_STUB);                                                              \
                                                                                                    \
	ds_HashMapAdd(&db.hash, key, slot.index);                                                       \
    db.pool.buf[slot.index].id = stub_id;                                                           \
    db.pool.buf[slot.index].reference_count = 0;                                                    \
    return db;                                                                                      \
}                                                                                                   

#define DEFINE_SDB_DEALLOC(T)                                                                       \
DECLARE_SDB_DEALLOC(T)                                                                              \
{                                                                                                   \
	ds_PoolDealloc(db->pool);                                                                    \
	ds_HashMapDealloc(&db->hash);                                                                   \
}                                                                                                   

#define DEFINE_SDB_FLUSH(T)                                                                         \
DECLARE_SDB_FLUSH(T)                                                                                \
{                                                                                                   \
    ds_HashMapFlush(&db->hash);                                                                     \
    ds_PoolFlush(db->pool);                                                                      \
    DLLFlush(db->allocated_list);                                                                \
                                                                                                    \
    const utf8 stub_id = Utf8Empty();                                                               \
	const u32 key = Utf8Hash(stub_id);                                                              \
                                                                                                    \
	struct slot slot = ds_PoolAdd(db->pool);                                                     \
	ds_Assert(slot.index == SDB_STUB);                                                              \
                                                                                                    \
	ds_HashMapAdd(&db->hash, key, slot.index);                                                      \
    db->pool.buf[slot.index].id = stub_id;                                                          \
    db->pool.buf[slot.index].reference_count = 0;                                                   \
}                                                                                                   

#define DEFINE_SDB_ADD(T)                                                                           \
DECLARE_SDB_ADD(T)                                                                                  \
{                                                                                                   \
	/* the empty id is the stub's */                                                                \
	if (id_to_copy.len == 0 || T ## SDBLookup(db, id_to_copy).index != SDB_STUB)                    \
	{                                                                                               \
		return (struct slot) { .index = U32_MAX, .address = NULL };                                 \
	}                                                                                               \
                                                                                                    \
	struct slot slot = ds_PoolAdd(db->pool);                                                        \
	if (slot.address == NULL)                                                                       \
	{                                                                                               \
		return (struct slot) { .index = U32_MAX, .address = NULL };                                 \
	}                                                                                               \
                                                                                                    \
	const utf8 id = Utf8Copy(mem, id_to_copy);                                                      \
	if (id.len == 0 || !ds_HashMapAdd(&db->hash, Utf8Hash(id), slot.index))                         \
	{                                                                                               \
		ds_PoolRemove(db->pool, slot.index);                                                        \
		return (struct slot) { .index = U32_MAX, .address = NULL };                                 \
	}                                                                                               \
                                                                                                    \
	DLLAppend(db->allocated_list, db->pool.buf, slot.index, strdb_allocated);                       \
	db->pool.buf[slot.index].id = id;                                                               \
	db->pool.buf[slot.index].reference_count = 0;                                                   \
	return slot;                                                                                    \
}

#define DEFINE_SDB_ADD_AND_ALIAS(T)                                                                 \
DECLARE_SDB_ADD_AND_ALIAS(T)                                                                        \
{                                                                                                   \
	/* the empty id is the stub's */                                                                \
	if (id_to_alias.len == 0 || T ## SDBLookup(db, id_to_alias).index != SDB_STUB)                  \
	{                                                                                               \
		return (struct slot) { .address = NULL, .index = U32_MAX };                                 \
	}                                                                                               \
                                                                                                    \
	struct slot slot = ds_PoolAdd(db->pool);                                                        \
	if (slot.address == NULL)                                                                       \
	{                                                                                               \
		return (struct slot) { .address = NULL, .index = U32_MAX };                                 \
	}                                                                                               \
                                                                                                    \
	if (!ds_HashMapAdd(&db->hash, Utf8Hash(id_to_alias), slot.index))                               \
	{                                                                                               \
		ds_PoolRemove(db->pool, slot.index);                                                        \
		return (struct slot) { .address = NULL, .index = U32_MAX };                                 \
	}                                                                                               \
                                                                                                    \
	DLLAppend(db->allocated_list, db->pool.buf, slot.index, strdb_allocated);                       \
	db->pool.buf[slot.index].id = id_to_alias;                                                      \
	db->pool.buf[slot.index].reference_count = 0;                                                   \
	return slot;                                                                                    \
}

#define DEFINE_SDB_REMOVE(T)                                                                        \
DECLARE_SDB_REMOVE(T)                                                                               \
{                                                                                                   \
	const struct slot slot = T ## SDBLookup(db, id);                                                \
	if (slot.index != SDB_STUB)                                                                     \
	{                                                                                               \
	    const u32 key = Utf8Hash(db->pool.buf[slot.index].id);                                      \
		DLLRemove(db->allocated_list, db->pool.buf, slot.index, strdb_allocated);                \
		ds_HashMapRemove(&db->hash, key, slot.index);                                               \
		ds_PoolRemove(db->pool, slot.index);                                                     \
	}                                                                                               \
}                                                                                                   

#define DEFINE_SDB_LOOKUP(T)                                                                        \
DECLARE_SDB_LOOKUP(T)                                                                               \
{                                                                                                   \
	const u32 key = Utf8Hash(id);                                                                   \
	struct slot slot = { .index = SDB_STUB, .address = db->pool.buf };                              \
	for (u32 i = ds_HashMapFirst(&db->hash, key); i != HASH_NULL; i = ds_HashMapNext(&db->hash, i)) \
	{                                                                                               \
		if (Utf8Equivalence(id, db->pool.buf[i].id))                                                \
		{                                                                                           \
			slot.index = i;                                                                         \
			slot.address = db->pool.buf + i;                                                        \
			break;                                                                                  \
		}                                                                                           \
	}                                                                                               \
                                                                                                    \
	return slot;                                                                                    \
}                                                                                                   

#define DEFINE_SDB_REFERENCE(T)                                                                     \
DECLARE_SDB_REFERENCE(T)                                                                            \
{                                                                                                   \
	struct slot slot = T ## SDBLookup(db, id);                                                      \
	if (slot.index != SDB_STUB)                                                                     \
	{                                                                                               \
		db->pool.buf[slot.index].reference_count += 1;                                              \
	}                                                                                               \
	return slot;                                                                                    \
}

#define DEFINE_SDB_DEREFERENCE(T)                                                                   \
DECLARE_SDB_DEREFERENCE(T)                                                                          \
{                                                                                                   \
	if (index != SDB_STUB)                                                                          \
	{                                                                                               \
		ds_Assert(db->pool.buf[index].reference_count >= 1);                                        \
		db->pool.buf[index].reference_count -= 1;                                                   \
	}                                                                                               \
}


#ifdef __cplusplus
} 
#endif

#endif
