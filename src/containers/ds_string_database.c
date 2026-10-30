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

#include "ds_base.h"
#include "ds_string_database.h"

static u8 *internal_SdbNode(const struct ds_SDBPool pool, const struct ds_SDBLayout layout, const u32 index)
{
	return (u8 *) *pool.buf + index*layout.slot_size;
}

/* node = the new slot's address: set its id, hash and count, and link it into the hash map */
static u32 internal_SdbNodeSet(struct ds_HashMap *hash, const struct ds_SDBLayout layout, u8 *node, const u32 index, const utf8 id)
{
	const u32 key = Utf8Hash(id);
	if (!ds_HashMapAdd(hash, key, index))
	{
		return 0;
	}

	*(utf8 *) (node + layout.id_offset) = id;
	*(u32 *) (node + layout.id_hash_offset) = key;
	*(u32 *) (node + layout.reference_count_offset) = 0;
	return 1;
}

/* the pool and hash map are empty: add the stub (index SDB_STUB, empty id) */
static void internal_SdbStubAdd(const struct ds_SDBPool pool, struct ds_HashMap *hash, const struct ds_SDBLayout layout)
{
	const struct slot slot = ds_PoolAddInternal(pool.buf, pool.length, pool.count, pool.count_max, pool.next_free, *pool.growable, layout.slot_size, layout.pool_slot_offset);
	ds_Assert(slot.index == SDB_STUB);
	const u32 linked = internal_SdbNodeSet(hash, layout, slot.address, slot.index, Utf8Empty());
	ds_Assert(linked);
	(void) linked;
}

void ds_SDBAllocInternal(struct arena *mem, const struct ds_SDBPool pool, struct ds_HashMap *hash, const struct ds_SDBLayout layout, const u32 length, const u32 growable)
{
	*hash = ds_HashMapAlloc(mem, length, length, growable);
	const u32 allocated = ds_PoolAllocInternal(mem, pool.heap_allocated, pool.buf, pool.length, pool.count, pool.count_max, pool.next_free, pool.growable, layout.slot_size, length, growable);
	if (!hash->hash || !allocated || *pool.length == 0)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to allocate string_database");
		FatalCleanupAndExit();
	}

	internal_SdbStubAdd(pool, hash, layout);
}

void ds_SDBFlushInternal(const struct ds_SDBPool pool, struct ds_HashMap *hash, const struct ds_SDBLayout layout)
{
	ds_HashMapFlush(hash);
	*pool.count = 0;
	*pool.count_max = 0;
	*pool.next_free = POOL_INDEX_MASK;
	PoisonAddress(*pool.buf, layout.slot_size * *pool.length);

	internal_SdbStubAdd(pool, hash, layout);
}

struct slot ds_SDBAddInternal(struct arena *mem, const struct ds_SDBPool pool, struct ds_HashMap *hash, const struct ds_SDBLayout layout, const utf8 str)
{
	const struct slot failure = { .address = NULL, .index = U32_MAX };

	/* the empty id is the stub's */
	if (str.len == 0 || ds_SDBLookupInternal(*pool.buf, hash, layout, str).index != SDB_STUB)
	{
		return failure;
	}

	const struct slot slot = ds_PoolAddInternal(pool.buf, pool.length, pool.count, pool.count_max, pool.next_free, *pool.growable, layout.slot_size, layout.pool_slot_offset);
	if (slot.address == NULL)
	{
		return failure;
	}

	const utf8 id = (mem) ? Utf8Copy(mem, str) : str;
	if (id.len == 0 || !internal_SdbNodeSet(hash, layout, slot.address, slot.index, id))
	{
		ds_PoolRemoveInternal(*pool.buf, pool.count, pool.next_free, *pool.count_max, slot.index, layout.slot_size, layout.pool_slot_offset);
		return failure;
	}

	return slot;
}

void ds_SDBRemoveInternal(const struct ds_SDBPool pool, struct ds_HashMap *hash, const struct ds_SDBLayout layout, const utf8 str)
{
	const struct slot slot = ds_SDBLookupInternal(*pool.buf, hash, layout, str);
	if (slot.index != SDB_STUB)
	{
		const u8 *node = internal_SdbNode(pool, layout, slot.index);
		ds_HashMapRemove(hash, *(const u32 *) (node + layout.id_hash_offset), slot.index);
		ds_PoolRemoveInternal(*pool.buf, pool.count, pool.next_free, *pool.count_max, slot.index, layout.slot_size, layout.pool_slot_offset);
	}
}
