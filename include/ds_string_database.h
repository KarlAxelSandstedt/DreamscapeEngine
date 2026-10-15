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
ds_string_database.h
====================
A typed database of nodes keyed by a utf8 id: a ds_Pool(T) (stable indices as handles) and a hash map from
the id's hash to indices. T is a struct with SDB_NODE among its members.

    DEFINE_SDB(T);                                  ds_Pool(T) and the database type ds_SDB(T) (= T ## SDB)

    ds_SDBAlloc(mem, db, length, growable)          mem and growable as for pools; length includes the stub;
                                                    fatal on failure
    ds_SDBDealloc(db)                               no id is freed
    ds_SDBFlush(db)                                 every node removed but a fresh stub
    struct slot ds_SDBAdd(mem, db, str)             a node whose id is a copy of str on mem
    struct slot ds_SDBAddAlias(db, str)             a node whose id aliases str's bytes (kept alive by the
                                                    caller for as long as the node exists)
    void        ds_SDBRemove(db, str)               remove str's node if present, whatever its count
    struct slot ds_SDBLookup(db, str)               str's slot, or the stub's (SDB_STUB) if not present
    struct slot ds_SDBReference(db, str)            lookup, then increment the found node's count
    void        ds_SDBDereference(db, index)        decrement the node's count (index: a node or SDB_STUB)

db is a ds_SDB(T) lvalue; arguments may be evaluated more than once. Adds return the new node's slot
(reference count 0), or { U32_MAX, NULL } if the id is empty or present, the pool or hash map is full, or
the copy fails.

- Index SDB_STUB (0) is the stub node, made by Alloc and Flush, with the empty id. Lookup returns it for an
  id that is not present, so the empty id can't be added. Reference and Dereference leave the stub's count
  alone: callers use the stub as a default they reference freely.
- reference_count is the callers' bookkeeping. The database frees no id. Not thread-safe.
*/

#ifndef __DS_STRING_DATABASE_H__
#define __DS_STRING_DATABASE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ds_hash_map.h"
#include "ds_string.h"

#define SDB_STUB	0

#define SDB_NODE                        \
    utf8                id;             \
    u32                 id_hash;        \
    u32                 reference_count;\
    POOL_NODE

#define ds_SDB(T)	T ## SDB

#define DEFINE_SDB(T)                   \
DEFINE_POOL(T);                         \
typedef struct ds_SDB(T)                \
{                                       \
	ds_Pool(T)          pool;           \
	struct ds_HashMap   hash;           \
} ds_SDB(T)

#define ds_SDBAlloc(mem, db, length, growable)                                                          \
	ds_SDBAllocInternal((mem), ds_SDBPoolFields(db), &(db).hash, ds_SDBLayout(db), (length), (growable))

#define ds_SDBDealloc(db)                                                                               \
do                                                                                                      \
{                                                                                                       \
	ds_PoolDealloc((db).pool);                                                                          \
	ds_HashMapDealloc(&(db).hash);                                                                      \
} while (0)

#define ds_SDBFlush(db)                                                                                 \
	ds_SDBFlushInternal(ds_SDBPoolFields(db), &(db).hash, ds_SDBLayout(db))

#define ds_SDBAdd(mem, db, str)                                                                         \
	ds_SDBAddInternal((mem), ds_SDBPoolFields(db), &(db).hash, ds_SDBLayout(db), (str))

#define ds_SDBAddAlias(db, str)                                                                         \
	ds_SDBAddInternal(NULL, ds_SDBPoolFields(db), &(db).hash, ds_SDBLayout(db), (str))

#define ds_SDBRemove(db, str)                                                                           \
	ds_SDBRemoveInternal(ds_SDBPoolFields(db), &(db).hash, ds_SDBLayout(db), (str))

#define ds_SDBLookup(db, str)                                                                           \
	ds_SDBLookupInternal((db).pool.buf, &(db).hash, ds_SDBLayout(db), (str))

#define ds_SDBReference(db, str)                                                                        \
	ds_SDBReferenceInternal((db).pool.buf, &(db).hash, ds_SDBLayout(db), (str))

#define ds_SDBDereference(db, index)                                                                    \
do                                                                                                      \
{                                                                                                       \
	if ((index) != SDB_STUB)                                                                            \
	{                                                                                                   \
		ds_Assert((db).pool.buf[(index)].reference_count >= 1);                                         \
		(db).pool.buf[(index)].reference_count -= 1;                                                    \
	}                                                                                                   \
} while (0)

/* Internal: where a node's fields lie in T, constants from the macros above. */
struct ds_SDBLayout
{
	u64 slot_size;
	u64 pool_slot_offset;
	u64 id_offset;
	u64 id_hash_offset;
	u64 reference_count_offset;
};

/* Internal: byte offset of a field of T (as ds_PoolSlotOffset: buf is never read). */
#define ds_SDBFieldOffset(db, field)	((u64) &((1 ? NULL : (db).pool.buf)->field))

#define ds_SDBLayout(db)                                                                                \
	((struct ds_SDBLayout) { sizeof((db).pool.buf[0]), ds_PoolSlotOffset((db).pool),                   \
	                         ds_SDBFieldOffset(db, id), ds_SDBFieldOffset(db, id_hash),                 \
	                         ds_SDBFieldOffset(db, reference_count) })

/* Internal: the pool's fields, for the untyped implementation (as the ds_Pool macros pass them). */
struct ds_SDBPool
{
	void ** buf;
	u32 *   length;
	u32 *   count;
	u32 *   count_max;
	u32 *   next_free;
	u32 *   growable;
	u32 *   heap_allocated;
};

#define ds_SDBPoolFields(db)                                                                            \
	((struct ds_SDBPool) { (void **) &(db).pool.buf, &(db).pool.length, &(db).pool.count,              \
	                       &(db).pool.count_max, &(db).pool.next_free, &(db).pool.growable,             \
	                       &(db).pool.heap_allocated })

/* Internal: see the macros above. */
void        ds_SDBAllocInternal(struct arena *mem, const struct ds_SDBPool pool, struct ds_HashMap *hash, const struct ds_SDBLayout layout, const u32 length, const u32 growable);
void        ds_SDBFlushInternal(const struct ds_SDBPool pool, struct ds_HashMap *hash, const struct ds_SDBLayout layout);
/* mem == NULL: alias str */
struct slot ds_SDBAddInternal(struct arena *mem, const struct ds_SDBPool pool, struct ds_HashMap *hash, const struct ds_SDBLayout layout, const utf8 str);
void        ds_SDBRemoveInternal(const struct ds_SDBPool pool, struct ds_HashMap *hash, const struct ds_SDBLayout layout, const utf8 str);
static ds_ForceInline struct slot ds_SDBLookupInternal(void *buf, const struct ds_HashMap *hash, const struct ds_SDBLayout layout, const utf8 str);
static ds_ForceInline struct slot ds_SDBReferenceInternal(void *buf, const struct ds_HashMap *hash, const struct ds_SDBLayout layout, const utf8 str);

/******************************** inline definitions ********************************/

/* force-inlined: callers get constant field offsets; the stored hash is compared before the id */
static ds_ForceInline struct slot ds_SDBLookupInternal(void *buf, const struct ds_HashMap *hash, const struct ds_SDBLayout layout, const utf8 str)
{
	const u32 key = Utf8Hash(str);
	for (u32 i = ds_HashMapFirst(hash, key); i != HASH_NULL; i = ds_HashMapNext(hash, i))
	{
		u8 *node = (u8 *) buf + i*layout.slot_size;
		if (*(const u32 *) (node + layout.id_hash_offset) == key && Utf8Equivalence(str, *(const utf8 *) (node + layout.id_offset)))
		{
			return (struct slot) { .address = node, .index = i };
		}
	}

	return (struct slot) { .address = buf, .index = SDB_STUB };
}

static ds_ForceInline struct slot ds_SDBReferenceInternal(void *buf, const struct ds_HashMap *hash, const struct ds_SDBLayout layout, const utf8 str)
{
	const struct slot slot = ds_SDBLookupInternal(buf, hash, layout, str);
	if (slot.index != SDB_STUB)
	{
		*(u32 *) ((u8 *) slot.address + layout.reference_count_offset) += 1;
	}
	return slot;
}

#ifdef __cplusplus
}
#endif

#endif
