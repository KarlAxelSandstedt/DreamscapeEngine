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
ds_pool.h
=========
Pool: ds_Pool(T) of POOL_NODE structs: stable indices, free list, serializable.
The inline definitions follow at the bottom, in the same order.
Included by ds_allocator.h after its Heap section, not on its own.
*/

#ifndef __DS_POOL_H__
#define __DS_POOL_H__

/*
Pool API
--------
Intrusive pool of struct Type: a slot keeps its index for its whole lifetime, so indices are the handles
others hold. Adds reuse removed slots, so live slots are scattered over buf[0, count_max). A pool lives in
an arena (fixed length) or on the heap (fixed, or growable: the length roughly doubles when an add finds
the pool full, and buf moves, so hold indices, never addresses). At most 2^31 - 2 slots: growing past
that is fatal.

::: Usage and Documentation :::

struct Type contains POOL_NODE (its pool_slot field belongs to the pool). DEFINE_POOL(Type) defines the
pool type ds_Pool(Type) (once, where Type is declared); the macros below work on any pool type.

    struct node { POOL_NODE; ... };
    DEFINE_POOL(node);

    ds_Pool(node) pool;
    ds_PoolAlloc(NULL, pool, 128, GROWABLE);
    const struct slot slot = ds_PoolAdd(pool);                (slot.index is the handle)
    for (u32 i = 0; i < pool.count_max; ++i)
        if (ds_PoolSlotAllocated(pool.buf + i)) { ... }       (iterate the live slots)
    ds_PoolRemove(pool, slot.index);
    ds_PoolDealloc(pool);

The macros (pool is a ds_Pool(Type) lvalue, addr a struct Type pointer; arguments may be evaluated more
than once):

    // Allocate length slots in mem (fixed) or on the heap (mem == NULL; growable allowed). Returns 1 on
    // success; 0 on failure, with the pool empty (buf NULL, length 0).
    u32             ds_PoolAlloc(mem, pool, length, growable)

    // Free a heap pool's memory (arena memory stays with the arena); the pool becomes empty.
    void            ds_PoolDealloc(pool)

    // Remove all slots; the next adds return indices 0, 1, 2, ...
    void            ds_PoolFlush(pool)

    // Allocate a slot and return { address, index }: the most recently removed slot, else the next unused
    // one. Its contents are uninitialized (pool_slot aside). Full and not growable: { NULL, POOL_NULL }.
    struct slot     ds_PoolAdd(pool)

    // Remove an allocated slot, by index or by address.
    void            ds_PoolRemove(pool, index)
    void            ds_PoolRemoveAddress(pool, addr)

    // Return the index of a slot's address.
    u32             ds_PoolIndex(pool, addr)

    // 1 if the slot at ptr (in buf[0, count_max)) is allocated.
    u32             ds_PoolSlotAllocated(ptr)

    // WARNING: Platform-dependent.
    // Bytes ds_PoolSerialize writes.
    u64             ds_PoolSerializeSize(pool)

    // Write the pool to ss (at a byte boundary; it must fit: ds_PoolSerializeSize).
    void            ds_PoolSerialize(ss, pool)

    // Read a pool from ss into mem (fixed) or the heap (mem == NULL; growable allowed), with slot positions
    // and free-list order as written (later adds return the same indices). Returns 1 on success; 0 if mem
    // is full or ss holds no valid pool: inconsistent counts, an index out of range or repeated (the pool
    // is empty, ss's position unchanged).
    u32             ds_PoolTryDeserialize(mem, ss, pool, growable)

    // Element-wise: define static T##PoolSerializeSize(const ds_Pool(T) *), T##PoolSerialize(ss, pool) and
    // T##PoolTryDeserialize(mem, ss, pool, growable) (as the three above, pool a pointer) over the element
    // functions ELEM##SerializeSize(const struct T *), ELEM##Serialize(ss, const struct T *) and
    // ELEM##TryDeserialize(ss, struct T *) (1 on success): the counts and the free list little-endian,
    // then every allocated slot as its index and element.
    DEFINE_POOL_SERIALIZE(T, ELEM)

::: Stub slot and serialized format :::

buf[-1] is a stub slot, so index -1 (DS_STUB_INDEX) is addressable: intrusive lists over pools
(ds_list.h) use it as their sentinel. Serialized format (LE): u32 length, count, count_max; the
count_max - count free indices in free-list order; the count allocated slots as raw bytes, each with
pool_slot replaced by its index. DEFINE_POOL_SERIALIZE writes the same counts and free list, then each
allocated slot as u32 index + element.
*/

#define POOL_NODE 	        u32 pool_slot
#define POOL_NULL		        0xffffffff
#define POOL_ALLOCATION_MASK	0x80000000
#define POOL_INDEX_MASK		    0x7fffffff

#define ds_Pool(T) ds_Pool_ ## T

#define DEFINE_POOL(T)                                                                              \
typedef struct ds_Pool(T)                                                                           \
{                                                                                                   \
	struct T *buf;		                                                                            \
	u32 length;			        /* array length 				                                */  \
	u32 count;			        /* current count of occupied slots 		                        */  \
	u32 count_max;		        /* max count used over the object's lifetime 	                */  \
	u32 next_free;		        /* next free index if != POOL_INDEX_MASK 		                */  \
	u32 growable;		        /* is the memory growable? 			                            */  \
	u32 heap_allocated;	        /* buf - 1 is a ds_Alloc block (else arena memory)              */  \
} ds_Pool(T)

#define ds_PoolSlotAllocated(ptr)	(!((ptr)->pool_slot & POOL_ALLOCATION_MASK))

/*
Internal: byte offset of pool_slot in the pool's slot type, a constant. (1 ? NULL : buf) is a null pointer of
buf's type that never reads buf (it may be uninitialized, e.g. before ds_PoolTryDeserialize).
*/
#define ds_PoolSlotOffset(pool)		((u64) &((1 ? NULL : (pool).buf)->pool_slot))

/* Internal: see ds_PoolAlloc. Returns 1 on success and 0 on failure. */
u32  ds_PoolAllocInternal(struct arena *mem, u32 *heap_allocated, void **buf, u32 *length, u32 *count, u32 *count_max, u32 *next_free, u32 *growable, const u64 slot_size, const u32 alloc_length, const u32 alloc_growable);
/* Internal: grow a full growable pool (the cold path of ds_PoolAdd). */
void ds_PoolGrowInternal(void **buf, u32 *length, const u64 slot_size);
/* Internal: the image's counts and free list (see DEFINE_POOL_SERIALIZE); the deserializers return 1 on success. */
void ds_PoolSerializeFreeListInternal(struct SS *ss, const void *buf, const u32 length, const u32 count, const u32 count_max, const u32 next_free, const u64 slot_size, const u64 pool_slot_offset);
u32  ds_PoolTryDeserializeHeaderInternal(struct SS *ss, u32 *length, u32 *count, u32 *count_max);
u32  ds_PoolTryDeserializeFreeListInternal(struct SS *ss, void *buf, u32 *next_free, const u32 count, const u32 count_max, const u64 slot_size, const u64 pool_slot_offset);
/* Internal: see ds_PoolSerialize. */
void ds_PoolSerializeInternal(struct SS *ss, const void *buf, const u32 length, const u32 count, const u32 count_max, const u32 next_free, const u64 slot_size, const u64 pool_slot_offset);
/* Internal: see ds_PoolTryDeserialize. */
u32  ds_PoolTryDeserializeInternal(struct arena *mem, struct SS *ss, u32 *heap_allocated, void **buf, u32 *length, u32 *count, u32 *count_max, u32 *next_free, u32 *growable, const u64 slot_size, const u64 pool_slot_offset, const u32 alloc_growable);

/* Internal: see ds_PoolAdd, ds_PoolRemove, and the debug checks of ds_PoolIndex (empty in release builds). */
static ds_ForceInline struct slot ds_PoolAddInternal(void **buf, u32 *length, u32 *count, u32 *count_max, u32 *next_free, const u32 growable, const u64 slot_size, const u64 pool_slot_offset);
static ds_ForceInline void ds_PoolRemoveInternal(void *buf, u32 *count, u32 *next_free, const u32 count_max, const u32 index, const u64 slot_size, const u64 pool_slot_offset);
static ds_ForceInline void ds_PoolIndexCheck(const void *buf, const u32 length, const void *addr, const u64 slot_size);

#define ds_PoolAlloc(mem, pool, __length, __growable)                                                   \
	ds_PoolAllocInternal((mem), &(pool).heap_allocated, (void **) &(pool).buf, &(pool).length,         \
	                     &(pool).count, &(pool).count_max, &(pool).next_free, &(pool).growable,         \
	                     sizeof((pool).buf[0]), (__length), (__growable))

#define ds_PoolDealloc(pool)                                                                            \
do                                                                                                      \
{                                                                                                       \
	if ((pool).heap_allocated && (pool).buf)                                                            \
	{                                                                                                   \
		ds_Free((pool).buf - 1);                                                                        \
	}                                                                                                   \
	(pool).buf = NULL;                                                                                  \
	(pool).length = 0;                                                                                  \
	(pool).count = 0;                                                                                   \
	(pool).count_max = 0;                                                                               \
	(pool).next_free = POOL_INDEX_MASK;                                                                 \
	(pool).heap_allocated = 0;                                                                          \
} while (0)

#define ds_PoolFlush(pool)                                                                              \
do                                                                                                      \
{                                                                                                       \
	(pool).count = 0;                                                                                   \
	(pool).count_max = 0;                                                                               \
	(pool).next_free = POOL_INDEX_MASK;                                                                 \
	PoisonAddress((pool).buf, sizeof((pool).buf[0]) * (pool).length);                                  \
} while (0)

#define ds_PoolAdd(pool)                                                                                \
	ds_PoolAddInternal((void **) &(pool).buf, &(pool).length, &(pool).count, &(pool).count_max,        \
	                   &(pool).next_free, (pool).growable, sizeof((pool).buf[0]), ds_PoolSlotOffset(pool))

#define ds_PoolRemove(pool, index)                                                                      \
	ds_PoolRemoveInternal((pool).buf, &(pool).count, &(pool).next_free, (pool).count_max, (index),     \
	                      sizeof((pool).buf[0]), ds_PoolSlotOffset(pool))

/* typed pointer difference: the compiler's exact division, and a type check of addr */
#define ds_PoolIndex(pool, addr)                                                                        \
	(ds_PoolIndexCheck((pool).buf, (pool).length, (addr), sizeof((pool).buf[0])),                      \
	 (u32) ((addr) - (pool).buf))

#define ds_PoolRemoveAddress(pool, addr)	ds_PoolRemove(pool, ds_PoolIndex(pool, addr))

#define ds_PoolSerializeSize(pool)                                                                      \
	(3*sizeof(u32) + (u64) ((pool).count_max - (pool).count)*sizeof(u32)                               \
	               + (u64) (pool).count*sizeof((pool).buf[0]))

#define ds_PoolSerialize(ss, pool)                                                                      \
	ds_PoolSerializeInternal((ss), (pool).buf, (pool).length, (pool).count, (pool).count_max,          \
	                         (pool).next_free, sizeof((pool).buf[0]), ds_PoolSlotOffset(pool))

#define ds_PoolTryDeserialize(mem, ss, pool, __growable)                                                \
	ds_PoolTryDeserializeInternal((mem), (ss), &(pool).heap_allocated, (void **) &(pool).buf,          \
	                              &(pool).length, &(pool).count, &(pool).count_max, &(pool).next_free,  \
	                              &(pool).growable, sizeof((pool).buf[0]), ds_PoolSlotOffset(pool),     \
	                              (__growable))

#define DEFINE_POOL_SERIALIZE(T, ELEM)                                                                  \
static u64 T##PoolSerializeSize(const ds_Pool(T) *pool)                                                 \
{                                                                                                       \
	u64 size = 3*sizeof(u32) + (u64) pool->count_max*sizeof(u32);                                       \
	for (u32 i = 0; i < pool->count_max; ++i)                                                           \
	{                                                                                                   \
		if (ds_PoolSlotAllocated(pool->buf + i))                                                        \
		{                                                                                               \
			size += ELEM##SerializeSize(pool->buf + i);                                                 \
		}                                                                                               \
	}                                                                                                   \
	return size;                                                                                        \
}                                                                                                       \
                                                                                                        \
static void T##PoolSerialize(struct SS *ss, const ds_Pool(T) *pool)                                     \
{                                                                                                       \
	ds_PoolSerializeFreeListInternal(ss, pool->buf, pool->length, pool->count, pool->count_max,         \
	                                 pool->next_free, sizeof(pool->buf[0]), ds_PoolSlotOffset(*pool));  \
	for (u32 i = 0; i < pool->count_max; ++i)                                                           \
	{                                                                                                   \
		if (ds_PoolSlotAllocated(pool->buf + i))                                                        \
		{                                                                                               \
			SSWriteU32Le(ss, i);                                                                        \
			ELEM##Serialize(ss, pool->buf + i);                                                         \
		}                                                                                               \
	}                                                                                                   \
}                                                                                                       \
                                                                                                        \
static u32 T##PoolTryDeserialize(struct arena *mem, struct SS *ss, ds_Pool(T) *pool, const u32 growable) \
{                                                                                                       \
	const u64 bit_index = ss->bit_index;                                                                \
	u32 length, count, count_max;                                                                       \
	if (!ds_PoolTryDeserializeHeaderInternal(ss, &length, &count, &count_max)                           \
		|| !ds_PoolAlloc(mem, *pool, length, growable))                                                 \
	{                                                                                                   \
		ss->bit_index = bit_index;                                                                      \
		return 0;                                                                                       \
	}                                                                                                   \
	if (!ds_PoolTryDeserializeFreeListInternal(ss, pool->buf, &pool->next_free, count, count_max,       \
	                                           sizeof(pool->buf[0]), ds_PoolSlotOffset(*pool)))        \
	{                                                                                                   \
		goto failure;                                                                                   \
	}                                                                                                   \
                                                                                                        \
	for (u32 i = 0; i < count; ++i)                                                                     \
	{                                                                                                   \
		if (SSBytesLeft(ss) < sizeof(u32))                                                              \
		{                                                                                               \
			goto failure;                                                                               \
		}                                                                                               \
		const u32 index = SSReadU32Le(ss);                                                              \
		if (index >= count_max || pool->buf[index].pool_slot != POOL_INDEX_MASK)                        \
		{                                                                                               \
			goto failure;                                                                               \
		}                                                                                               \
		UnpoisonAddress(pool->buf + index, sizeof(pool->buf[0]));                                       \
		if (!ELEM##TryDeserialize(ss, pool->buf + index))                                               \
		{                                                                                               \
			goto failure;                                                                               \
		}                                                                                               \
		pool->buf[index].pool_slot = 0;                                                                 \
	}                                                                                                   \
	pool->count = count;                                                                                \
	pool->count_max = count_max;                                                                        \
	return 1;                                                                                           \
                                                                                                        \
failure:                                                                                                \
	ds_PoolDealloc(*pool);                                                                              \
	ss->bit_index = bit_index;                                                                          \
	return 0;                                                                                           \
}

/******************************** Pool inline definitions *********************************/

/* Internal: see ds_PoolAdd. */
static ds_ForceInline struct slot ds_PoolAddInternal(void **buf, u32 *length, u32 *count, u32 *count_max, u32 *next_free, const u32 growable, const u64 slot_size, const u64 pool_slot_offset)
{
	if (*count == *length)
	{
		if (!growable)
		{
			return (struct slot) { .address = NULL, .index = POOL_NULL };
		}
		/* a full pool has no free slot, so the add below takes slot count_max */
		ds_PoolGrowInternal(buf, length, slot_size);
	}

	struct slot allocation;
	if (*next_free != POOL_INDEX_MASK)
	{
		allocation.index = *next_free;
		allocation.address = (u8 *) *buf + allocation.index*slot_size;
		UnpoisonAddress(allocation.address, slot_size);
		const u32 pool_slot = *(const u32 *) ((u8 *) allocation.address + pool_slot_offset);
		ds_Assert(pool_slot & POOL_ALLOCATION_MASK);
		*next_free = pool_slot & POOL_INDEX_MASK;
	}
	else
	{
		allocation.index = *count_max;
		allocation.address = (u8 *) *buf + allocation.index*slot_size;
		UnpoisonAddress(allocation.address, slot_size);
		*count_max += 1;
	}

	*(u32 *) ((u8 *) allocation.address + pool_slot_offset) = 0;
	*count += 1;
	return allocation;
}

/* Internal: see ds_PoolRemove. */
static ds_ForceInline void ds_PoolRemoveInternal(void *buf, u32 *count, u32 *next_free, const u32 count_max, const u32 index, const u64 slot_size, const u64 pool_slot_offset)
{
	ds_Assert(index < count_max);

	u8 *slot = (u8 *) buf + index*slot_size;
	u32 *pool_slot = (u32 *) (slot + pool_slot_offset);
	ds_Assert((*pool_slot & POOL_ALLOCATION_MASK) == 0);

	*pool_slot = POOL_ALLOCATION_MASK | *next_free;
	*next_free = index;
	*count -= 1;

	PoisonAddress(slot, pool_slot_offset);
	PoisonAddress(slot + pool_slot_offset + sizeof(u32), slot_size - pool_slot_offset - sizeof(u32));
}

/* Internal: debug checks of ds_PoolIndex (empty in release builds). */
static ds_ForceInline void ds_PoolIndexCheck(const void *buf, const u32 length, const void *addr, const u64 slot_size)
{
	ds_Assert((u64) addr >= (u64) buf);
	ds_Assert((u64) addr < (u64) buf + length*slot_size);
	ds_Assert(((u64) addr - (u64) buf) % slot_size == 0);
}

#endif
