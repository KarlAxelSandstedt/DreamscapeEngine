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

#ifndef __STRING_DATABASE_H__
#define __STRING_DATABASE_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_hash_map.h"
#include "list.h"


#define SDB_STUB	0

#define SDB_NODE                        \
    utf8                id;             \
    u32                 reference_count;\
    struct ds_DLLNode   strdb_allocated;\
    POOL_NODE

/*
 *	1. id aliasing: on deallocation, do nothing with identifier, 
 *		(up to user to free, BUT WE MUST ENSURE LIFETIME ATLEAST AS LONG AS DATABASE)
 *	2. malloc copy: on deallocation, free identifier 
 *	3. arena copy:  on deallocation, do nothing with identifier, (up to user to free)
 */
#define DEFINE_SDB_STRUCT(T)            \
typedef struct T ## SDB                 \
{                                       \
	struct ds_HashMap 	hash;           \
	struct T ## Pool	pool;           \
	struct ds_DLL	    allocated_list; \
} T ## SDB

#define SDB_DECLARE(T)                  \
    POOL_DECLARE(T);                    \
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
    POOL_DEFINE(T)                      \
    DEFINE_SDB_ALLOC(T)                 \
    DEFINE_SDB_DEALLOC(T)               \
    DEFINE_SDB_FLUSH(T)                 \
    DEFINE_SDB_ADD(T)                   \
    DEFINE_SDB_ADD_AND_ALIAS(T)         \
    DEFINE_SDB_REMOVE(T)                \
    DEFINE_SDB_LOOKUP(T)                \
    DEFINE_SDB_REFERENCE(T)             \
    DEFINE_SDB_DEREFERENCE(T)          

/* Allocate and setup database. If growable, allows the database to increase size when required. */
#define DECLARE_SDB_ALLOC(T)    \
T ## SDB    T ## SDBAlloc(struct arena *mem, const u32 initial_length, const u32 growable)

/* Deallocate the database. WARNING: that none of the database id strings are freed in the call. */
#define DECLARE_SDB_DEALLOC(T)    \
void        T ## SDBDealloc(T ## SDB *db)

/* Flush or reset the string database */
#define DECLARE_SDB_FLUSH(T)    \
void        T ## SDBFlush(T ## SDB *db)

/* 
 * Allocate a new database node with the given identifier and return its index (handle). The id
 * will be copied onto the arena. On failure, the null slot (U32_MAX, NULL) is returned. the 
 * reference count is set to 0. 
 * */
#define DECLARE_SDB_ADD(T)    \
struct slot T ## SDBAdd(struct arena *mem, T ## SDB *db, const utf8 id_to_copy)

/* 
 * allocate a new database node with the given identifier and return its index. The id will alias
 * the given string's content. On failure, the null slot (U32_MAX, NULL) is returned. the reference
 * count is set to 0. 
 */
#define DECLARE_SDB_ADD_AND_ALIAS(T)    \
struct slot T ## SDBAddAndAlias(T ## SDB *db, const utf8 id_to_alias)

/* remove the identifier's corresponding database node if found and update database state, otherwise do nothing. */
#define DECLARE_SDB_REMOVE(T)    \
void        T ## SDBRemove(T ## SDB *db, const utf8 id)

/* Lookup the identifer in the database. If it exist, return its slot. Otherwise return the stub node. */
#define DECLARE_SDB_LOOKUP(T)   \
struct slot T ## SDBLookup(const T ## SDB *db, const utf8 id)

/* 
 * Return the result of the lookup operation. furthermore, if the returned slot is not the stub node, 
 * increment the corresponding node's reference count.  
 */
#define DECLARE_SDB_REFERENCE(T)    \
struct slot T ## SDBReference(T ## SDB *db, const utf8 id)

/* Lookup the handle in the database. If it exist, decrement the corresponding node's reference count. */
#define DECLARE_SDB_DEREFERENCE(T)    \
void        T ## SDBDereference(T ## SDB *db, const u32 index)

#define DEFINE_SDB_ALLOC(T)                                                                         \
DECLARE_SDB_ALLOC(T)                                                                                \
{                                                                                                   \
    struct T ## SDB db = { 0 };                                                                     \
    db.hash = ds_HashMapAlloc(mem, initial_length, initial_length, growable);                       \
    db.pool = T ## PoolAlloc(mem, initial_length, growable);                                        \
    ds_DLLFlush(&db.allocated_list);                                                                \
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
	struct slot slot = T ## PoolAdd(&db.pool);                                                      \
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
	T ## PoolDealloc(&db->pool);                                                                    \
	ds_HashMapDealloc(&db->hash);                                                                   \
}                                                                                                   

#define DEFINE_SDB_FLUSH(T)                                                                         \
DECLARE_SDB_FLUSH(T)                                                                                \
{                                                                                                   \
    ds_HashMapFlush(&db->hash);                                                                     \
    T ## PoolFlush(&db->pool);                                                                      \
    ds_DLLFlush(&db->allocated_list);                                                               \
                                                                                                    \
    const utf8 stub_id = Utf8Empty();                                                               \
	const u32 key = Utf8Hash(stub_id);                                                              \
                                                                                                    \
	struct slot slot = T ## PoolAdd(&db->pool);                                                     \
	ds_Assert(slot.index == SDB_STUB);                                                              \
                                                                                                    \
	ds_HashMapAdd(&db->hash, key, slot.index);                                                      \
    db->pool.buf[slot.index].id = stub_id;                                                          \
    db->pool.buf[slot.index].reference_count = 0;                                                   \
}                                                                                                   

#define DEFINE_SDB_ADD(T)                                                                           \
DECLARE_SDB_ADD(T)                                                                                  \
{                                                                                                   \
    struct slot slot = { .index = U32_MAX, .address = NULL };                                       \
	if (T ## SDBLookup(db, id_to_copy).index != SDB_STUB)                                           \
	{                                                                                               \
		return slot;                                                                                \
	}                                                                                               \
                                                                                                    \
    utf8 id = Utf8Copy(mem, id_to_copy);                                                            \
	if (id.buf)                                                                                     \
	{                                                                                               \
		const u32 key = Utf8Hash(id);                                                               \
		struct slot slot = T ## PoolAdd(&db->pool);                                                 \
		ds_HashMapAdd(&db->hash, key, slot.index);                                                  \
                                                                                                    \
		ds_DLLAppend(db->allocated_list, db->pool.buf, slot.index, strdb_allocated);                \
        db->pool.buf[slot.index].id = id;                                                           \
        db->pool.buf[slot.index].reference_count = 0;                                               \
	}                                                                                               \
                                                                                                    \
	return slot;                                                                                    \
}                                                                                                   

#define DEFINE_SDB_ADD_AND_ALIAS(T)                                                                 \
DECLARE_SDB_ADD_AND_ALIAS(T)                                                                        \
{                                                                                                   \
	if (T ## SDBLookup(db, id_to_alias).index != SDB_STUB)                                          \
	{                                                                                               \
		return (struct slot) { .address = NULL, .index = U32_MAX };                                 \
	}                                                                                               \
                                                                                                    \
	const u32 key = Utf8Hash(id_to_alias);                                                          \
	struct slot slot = T ## PoolAdd(&db->pool);                                                     \
	ds_HashMapAdd(&db->hash, key, slot.index);                                                      \
                                                                                                    \
	ds_DLLAppend(db->allocated_list, db->pool.buf, slot.index, strdb_allocated);                    \
    db->pool.buf[slot.index].id = id_to_alias;                                                      \
    db->pool.buf[slot.index].reference_count = 0;                                                   \
                                                                                                    \
	return slot;                                                                                    \
}

#define DEFINE_SDB_REMOVE(T)                                                                        \
DECLARE_SDB_REMOVE(T)                                                                               \
{                                                                                                   \
	const struct slot slot = T ## SDBLookup(db, id);                                                \
	if (slot.index != SDB_STUB)                                                                     \
	{                                                                                               \
	    const u32 key = Utf8Hash(db->pool.buf[slot.index].id);                                      \
		ds_DLLRemove(db->allocated_list, db->pool.buf, slot.index, strdb_allocated);                \
		ds_HashMapRemove(&db->hash, key, slot.index);                                               \
		T ## PoolRemove(&db->pool, slot.index);                                                     \
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
    db->pool.buf[slot.index].reference_count += 1;                                                  \
	return slot;                                                                                    \
}                                                                                                   

#define DEFINE_SDB_DEREFERENCE(T)                                                                   \
DECLARE_SDB_DEREFERENCE(T)                                                                          \
{                                                                                                   \
	ds_Assert(db->pool.buf[index].reference_count >= 1 || index == SDB_STUB);                       \
    db->pool.buf[index].reference_count -= 1;                                                       \
}                                                                                                  


#ifdef __cplusplus
} 
#endif

#endif
