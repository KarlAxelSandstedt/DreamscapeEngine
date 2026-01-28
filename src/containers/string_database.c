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
#include "string_database.h"

struct strdb strdb_AllocInternal(struct arena *mem, const u32 hash_size, const u32 index_size, const u64 data_size, const u64 id_offset, const u64 reference_count_offset, const u64 allocated_prev_offset, const u64 allocated_next_offset, const u64 pool_state_offset, const u32 growable)
{
	ds_Assert(!growable || !mem);
	ds_Assert(index_size && hash_size);

	struct pool pool;
	struct hashMap hash = { 0 };
	struct strdb db = { 0 };

	hash = HashMapAlloc(mem, hash_size, index_size, growable);
	pool = PoolAllocInternal(mem, index_size, data_size, pool_state_offset, U64_MAX, growable);

	if (!hash.hash || !pool.length)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to allocate string_database");
		FatalCleanupAndExit();
	}

	db.hash = hash;
	db.pool = pool;
	db.id_offset = id_offset;
	db.reference_count_offset = reference_count_offset;
	db.allocated_prev_offset = allocated_prev_offset;
	db.allocated_next_offset = allocated_next_offset;
	db.allocated_dll = dll_InitInternal(data_size, allocated_prev_offset, allocated_next_offset);

	const utf8 stub_id = Utf8Empty();
	const u32 key = Utf8Hash(stub_id);

	struct slot slot = PoolAdd(&db.pool);
	HashMapAdd(&db.hash, key, slot.index);
	struct strdb_node *node = slot.address;

	utf8 *id = (utf8 *)(((u8 *) slot.address) + db.id_offset);
	*id = Utf8Empty();

	u32 *reference_count = (u32 *)(((u8 *) slot.address) + db.reference_count_offset);
	*reference_count = 0;

	ds_Assert(slot.index == STRING_DATABASE_STUB_INDEX);

	return db;
}

void strdb_Dealloc(struct strdb *db)
{
	PoolDealloc(&db->pool);
	HashMapFree(&db->hash);
}

void strdb_Flush(struct strdb *db)
{
	HashMapFlush(&db->hash);
	PoolFlush(&db->pool);
	dll_Flush(&db->allocated_dll);
	const utf8 stub_id = Utf8Empty();
	const u32 key = Utf8Hash(stub_id);

	struct slot slot = PoolAdd(&db->pool);
	HashMapAdd(&db->hash, key, slot.index);
	struct strdb_node *node = slot.address;

	utf8 *id = (utf8 *)(((u8 *) slot.address) + db->id_offset);
	*id = stub_id;

	u32 *reference_count = (u32 *)(((u8 *) slot.address) + db->reference_count_offset);
	*reference_count = 0;

	ds_Assert(slot.index == STRING_DATABASE_STUB_INDEX);
}

struct slot strdb_Add(struct arena *mem_db_lifetime, struct strdb *db, const utf8 copy)
{
	struct slot slot = { .index = STRING_DATABASE_STUB_INDEX, .address = db->pool.buf };
	if (strdb_Lookup(db, copy).index != STRING_DATABASE_STUB_INDEX)
	{
		return slot;
	}

	utf8 id = Utf8Copy(mem_db_lifetime, copy);
	if (id.buf)
	{
		const u32 key = Utf8Hash(copy);
		struct slot slot = PoolAdd(&db->pool);
		HashMapAdd(&db->hash, key, slot.index);

		utf8 *id_ptr = (utf8 *)(((u8 *) slot.address) + db->id_offset);
		*id_ptr = id;

		u32 *reference_count = (u32 *)(((u8 *) slot.address) + db->reference_count_offset);
		*reference_count = 0;

		dll_Append(&db->allocated_dll, db->pool.buf, slot.index);
	}

	return slot;
}

struct slot strdb_AddAndAlias(struct strdb *db, const utf8 id)
{
	struct slot slot = { .index = STRING_DATABASE_STUB_INDEX, .address = db->pool.buf };
	if (strdb_Lookup(db, id).index != STRING_DATABASE_STUB_INDEX)
	{
		return slot;
	}

	slot = PoolAdd(&db->pool);
	const u32 key = Utf8Hash(id);
	HashMapAdd(&db->hash, key, slot.index);

	utf8 *id_ptr = (utf8 *)(((u8 *) slot.address) + db->id_offset);
	*id_ptr = id;

	u32 *reference_count = (u32 *)(((u8 *) slot.address) + db->reference_count_offset);
	*reference_count = 0;
		
	dll_Append(&db->allocated_dll, db->pool.buf, slot.index);

	return slot;
}

void strdb_Remove(struct strdb *db, const utf8 id)
{
	const struct slot slot = strdb_Lookup(db, id);
	if (slot.index != STRING_DATABASE_STUB_INDEX)
	{
		ds_Assert(*(u32 *)((u8 *) slot.address + db->reference_count_offset) == 0);
		const u32 key = Utf8Hash(*(utf8 *)((u8 *) slot.address + db->id_offset));
		HashMapRemove(&db->hash, key, slot.index);
		PoolRemove(&db->pool, slot.index);
		dll_Remove(&db->allocated_dll, db->pool.buf, slot.index);
	}
}

struct slot strdb_Lookup(const struct strdb *db, const utf8 id)
{
	const u32 key = Utf8Hash(id);
	struct slot slot = { .index = STRING_DATABASE_STUB_INDEX, .address = db->pool.buf };
	for (u32 i = HashMapFirst(&db->hash, key); i != HASH_NULL; i = HashMapNext(&db->hash, i))
	{
		u8 *address = strdb_Address(db, i);
		utf8 *id_ptr = (utf8 *) (address + db->id_offset);
		if (Utf8Equivalence(id, *id_ptr))
		{
			slot.index = i;
			slot.address = address;
			break;
		}
	}

	return slot;
}

void *strdb_Address(const struct strdb *db, const u32 handle)
{
	u8 *address = PoolAddress(&db->pool, handle);
	ds_Assert((*(u32 *)(address + db->pool.slot_allocation_offset)) & 0x80000000);
	return address;
}

struct slot strdb_Reference(struct strdb *db, const utf8 id)
{
	struct slot slot = strdb_Lookup(db, id);
	u32 *reference_count = (u32 *)(((u8 *) slot.address) + db->reference_count_offset);
	*reference_count += 1;
	return slot;
}

void strdb_Dereference(struct strdb *db, const u32 handle)
{
	u32 *reference_count = (u32 *)(((u8 *) strdb_Address(db, handle)) + db->reference_count_offset);
	ds_Assert(*reference_count || handle == STRING_DATABASE_STUB_INDEX);
	*reference_count -= 1;
}
