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

void ds_PoolSerializeFreeListInternal(struct SS *ss, const void *buf, const u32 length, const u32 count, const u32 count_max, const u32 next_free, const u64 slot_size, const u64 pool_slot_offset)
{
	const u8 *base = buf;
	SSWriteU32Le(ss, length);
	SSWriteU32Le(ss, count);
	SSWriteU32Le(ss, count_max);

	u32 free_count = 0;
	for (u32 i = next_free; i != POOL_INDEX_MASK; i = *(const u32 *) (base + i*slot_size + pool_slot_offset) & POOL_INDEX_MASK)
	{
		SSWriteU32Le(ss, i);
		free_count += 1;
	}
	ds_Assert(free_count == count_max - count);
}

void ds_PoolSerializeInternal(struct SS *ss, const void *buf, const u32 length, const u32 count, const u32 count_max, const u32 next_free, const u64 slot_size, const u64 pool_slot_offset)
{
	ds_Assert(3*sizeof(u32) + (u64) (count_max - count)*sizeof(u32) + (u64) count*slot_size <= SSBytesLeft(ss));

	ds_PoolSerializeFreeListInternal(ss, buf, length, count, count_max, next_free, slot_size, pool_slot_offset);
	const u8 *base = buf;
	for (u32 i = 0; i < count_max; ++i)
	{
		const u8 *slot = base + i*slot_size;
		if (*(const u32 *) (slot + pool_slot_offset) & POOL_ALLOCATION_MASK)
		{
			continue;
		}

		SSWrite8N(ss, (const b8 *) slot, pool_slot_offset);
		SSWriteU32Le(ss, i);
		SSWrite8N(ss, (const b8 *) (slot + pool_slot_offset + sizeof(u32)), slot_size - pool_slot_offset - sizeof(u32));
	}
}

/*
 * pool_slot is 0 for an allocated slot; a removed slot holds POOL_ALLOCATION_MASK | the next free index,
 * POOL_INDEX_MASK ending the free list. Add and Remove are force-inlined (ds_pool.h): the macros pass
 * sizeof(Type) and the offset of pool_slot, so both are constants in the inlined code. Growth is out of
 * line (ds_PoolGrowInternal) and reallocates (length + 1)*2 slots, stub included.
 */
u32 ds_PoolAllocInternal(struct arena *mem, u32 *heap_allocated, void **buf, u32 *length, u32 *count, u32 *count_max, u32 *next_free, u32 *growable, const u64 slot_size, const u32 alloc_length, const u32 alloc_growable)
{
	ds_Assert(!alloc_growable || !mem);
	*heap_allocated = 0;
	*buf = NULL;
	*length = 0;
	*count = 0;
	*count_max = 0;
	*next_free = POOL_INDEX_MASK;
	*growable = alloc_growable;

	const u64 size = slot_size * ((u64) alloc_length + 1);
	u8 *addr = (mem)
		? ArenaPush(mem, size)
		: ds_Alloc(size, DS_CACHE_LINE);
	if (addr == NULL)
	{
		return 0;
	}

	*buf = addr + slot_size;
	*length = alloc_length;
	*heap_allocated = (mem == NULL);
	PoisonAddress(*buf, slot_size * alloc_length);
	return 1;
}

void ds_PoolGrowInternal(void **buf, u32 *length, const u64 slot_size)
{
	const u32 length_max = (U32_MAX >> 1);
	const u32 old_length_including_stub = *length + 1;
	if (old_length_including_stub == length_max)
	{
		LogString(T_SYSTEM, S_FATAL, "pool allocator full, exiting");
		FatalCleanupAndExit();
	}

	u32 new_length_including_stub = old_length_including_stub << 1;
	if (new_length_including_stub > length_max)
	{
		new_length_including_stub = length_max;
	}

	/* ds_Realloc is fatal on failure */
	u8 *base = ds_Realloc((u8 *) *buf - slot_size, old_length_including_stub*slot_size,
	                      new_length_including_stub*slot_size, DS_CACHE_LINE);
	UnpoisonAddress(base, old_length_including_stub*slot_size);
	PoisonAddress(base + old_length_including_stub*slot_size,
	              (new_length_including_stub - old_length_including_stub)*slot_size);

	*buf = base + slot_size;
	*length = new_length_including_stub - 1;
}

#if 0 /* TPool: retired, rebuilt on magazines and TStack */
/*
 * Free lists are lock-free (Treiber) stacks of slot indices, one per thread index, each on its own cache
 * line (struct ds_TStack); an add tries its own list, then 2*logical_core_count random others. A list's head
 * is (tag << 32) | index of its top slot, TSTACK_NULL if empty; tpool_tail in each slot holds the head below
 * it. Each push increments the slot's tpool_tag, so a slot popped and pushed back between another thread's
 * load and compare-exchange changes the head (no ABA).
 *
 * Block 0 has N = initial_length (a power of two) slots, block 1 also N, block k >= 1 has N << (k-1), so the
 * pool length doubles with each block and an index maps to its block without a search:
 *
 *     blocks:   [N][N][2N][4N][8N] ...   (block k >= 1 starts at index N << (k-1))
 *     shift = log2(N) = Ctz32(N)
 *
 *     i < N:   block 0, offset i
 *     i >= N:  block  = 1 + (bit index of the leading 1 of i >> shift) = 32 - Clz32(i >> shift)
 *              offset = i & ((N << (block-1)) - 1)       (block k >= 1 holds the indices with that leading bit)
 *
 * At most 32 blocks (a_length < 2^31). Add, Remove, Address, the free-list operations and the fast path of
 * Increment are force-inlined in ds_pool.h (sizeof(Type) and the node field offsets are constants
 * there); Alloc, Dealloc, Flush and adding a block are out of line.
 */
void ds_TPoolAllocInternal(struct ds_TPoolState *state, void **block, const u32 logical_core_count, const u32 initial_count, const u64 slot_size)
{
	ds_Assert(initial_count <= 0x80000000);
	ds_StaticAssert(sizeof(struct ds_TStack) == DS_CACHE_LINE, "A free list should fill one cache line");

	memset(state, 0, sizeof(*state));
	memset(block, 0, 32*sizeof(void *));
	state->block_count = 1;
	state->block_length_next = (u32) PowerOfTwoCeil(initial_count);
	state->initial_length = state->block_length_next;
	state->shift = Ctz32(state->initial_length);

	state->free_list_count = logical_core_count;
	state->steal_max_iterations = 2*logical_core_count;
	state->t_free_list = ds_Alloc(logical_core_count*sizeof(struct ds_TStack), DS_CACHE_LINE);
	if (!state->t_free_list)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to Allocate ds_TPool free list memory, exiting.");
		FatalCleanupAndExit();
	}

	for (u32 i = 0; i < logical_core_count; ++i)
	{
		AtomicStoreRel64(&state->t_free_list[i].a_head, TSTACK_NULL);
	}

	const u64 memsize = state->block_length_next*slot_size;
	block[0] = ds_Alloc(memsize, DS_CACHE_LINE);
	if (!block[0])
	{
		Log(T_SYSTEM, S_FATAL, "Failed to Allocate ds_TPool with initial size %lu, exiting.", memsize);
		FatalCleanupAndExit();
	}
	AtomicStoreRel32(&state->a_length, state->block_length_next);
}

void ds_TPoolDeallocInternal(struct ds_TPoolState *state, void **block)
{
	const u32 force_read = AtomicLoadAcq32(&state->a_adding_memory);
	ds_Assert(force_read == 0);
	for (u32 i = 0; i < state->block_count; ++i)
	{
		ds_Free(block[i]);
	}
	ds_Free(state->t_free_list);
}

void ds_TPoolFlushInternal(struct ds_TPoolState *state)
{
	for (u32 i = 0; i < state->free_list_count; ++i)
	{
		AtomicStoreRel64(&state->t_free_list[i].a_head, TSTACK_NULL);
	}
	AtomicStoreRel32(&state->a_count_max, 0);
}

void ds_TPoolGrowInternal(struct ds_TPoolState *state, void **block, const u32 index, const u64 slot_size)
{
	while (index >= (u32) AtomicLoadAcq32(&state->a_length))
	{
		/* winning the swap gives an up-to-date view of the pool and makes this thread its only grower */
		u32 cmp_val = 0;
		const u32 exch_val = 1;
		if (AtomicCompareExchangeAcqRlx32(&state->a_adding_memory, &cmp_val, exch_val))
		{
			if (index >= state->a_length)
			{
				ds_Assert(state->a_length < 0x80000000);
				const u32 new_length = 2*state->a_length;

				const u64 memsize = state->block_length_next*slot_size;
				block[state->block_count] = ds_Alloc(memsize, DS_CACHE_LINE);
				if (!block[state->block_count])
				{
					Log(T_SYSTEM, S_FATAL, "Failed to Allocate ds_TPool block with size %lu, exiting.", memsize);
					FatalCleanupAndExit();
				}

				state->block_count += 1;
				state->block_length_next *= 2;
				AtomicStoreRel32(&state->a_length, new_length);
			}
			AtomicStoreRel32(&state->a_adding_memory, cmp_val);
		}
	}
}

#endif

/* the free list must fit; the slots are checked by their reader */
u32 ds_PoolTryDeserializeHeaderInternal(struct SS *ss, u32 *length, u32 *count, u32 *count_max)
{
	if (SSBytesLeft(ss) < 3*sizeof(u32))
	{
		return 0;
	}

	*length = SSReadU32Le(ss);
	*count = SSReadU32Le(ss);
	*count_max = SSReadU32Le(ss);
	return *count <= *count_max && *count_max <= *length && *length < POOL_INDEX_MASK && (u64) (*count_max - *count)*sizeof(u32) <= SSBytesLeft(ss);
}

/*
 * indices are written into the pool buffer, so each one must be in range and appear once (free list
 * and allocated slots together cover [0, count_max)): pool_slot starts as POOL_INDEX_MASK, a state no
 * finished slot has (allocated: 0, free: mask bit set), and a second visit fails the deserialization
 */
u32 ds_PoolTryDeserializeFreeListInternal(struct SS *ss, void *buf, u32 *next_free, const u32 count, const u32 count_max, const u64 slot_size, const u64 pool_slot_offset)
{
	u8 *base = buf;
	for (u32 i = 0; i < count_max; ++i)
	{
		u32 *pool_slot = (u32 *) (base + i*slot_size + pool_slot_offset);
		UnpoisonAddress(pool_slot, sizeof(u32));
		*pool_slot = POOL_INDEX_MASK;
	}

	u32 *prev_link = next_free;
	for (u32 i = 0; i < count_max - count; ++i)
	{
		const u32 index = SSReadU32Le(ss);
		if (index >= count_max)
		{
			return 0;
		}

		u32 *link = (u32 *) (base + index*slot_size + pool_slot_offset);
		if (*link != POOL_INDEX_MASK)
		{
			return 0;
		}

		/* end of the list until a successor is linked */
		*link = POOL_ALLOCATION_MASK | POOL_INDEX_MASK;
		*prev_link = (prev_link == next_free) ? index : (POOL_ALLOCATION_MASK | index);
		prev_link = link;
	}
	return 1;
}

u32 ds_PoolTryDeserializeInternal(struct arena *mem, struct SS *ss, u32 *heap_allocated, void **buf, u32 *length, u32 *count, u32 *count_max, u32 *next_free, u32 *growable, const u64 slot_size, const u64 pool_slot_offset, const u32 alloc_growable)
{
	const u64 bit_index = ss->bit_index;
	const u64 mem_left = (mem) ? mem->mem_left : 0;
	*heap_allocated = 0;
	*buf = NULL;
	u32 serialized_length, serialized_count, serialized_count_max;
	if (!ds_PoolTryDeserializeHeaderInternal(ss, &serialized_length, &serialized_count, &serialized_count_max))
	{
		goto failure;
	}

	/* division instead of count*slot_size: a corrupt count must not overflow the check */
	const u64 free_count = (u64) serialized_count_max - serialized_count;
	if (serialized_count > (SSBytesLeft(ss) - free_count*sizeof(u32)) / slot_size
		|| !ds_PoolAllocInternal(mem, heap_allocated, buf, length, count, count_max, next_free, growable, slot_size, serialized_length, alloc_growable)
		|| !ds_PoolTryDeserializeFreeListInternal(ss, *buf, next_free, serialized_count, serialized_count_max, slot_size, pool_slot_offset))
	{
		goto failure;
	}

	u8 *base = *buf;
	for (u32 i = 0; i < serialized_count; ++i)
	{
		struct SS peek = *ss;
		peek.bit_index += 8*pool_slot_offset;
		const u32 index = SSReadU32Le(&peek);
		if (index >= serialized_count_max)
		{
			goto failure;
		}

		u8 *slot = base + index*slot_size;
		if (*(u32 *) (slot + pool_slot_offset) != POOL_INDEX_MASK)
		{
			goto failure;
		}

		UnpoisonAddress(slot, slot_size);
		SSRead8N(ss, (b8 *) slot, slot_size);
		*(u32 *) (slot + pool_slot_offset) = 0;
	}

	*count = serialized_count;
	*count_max = serialized_count_max;
	return 1;

failure:
	if (mem)
	{
		ArenaPopPacked(mem, mem_left - mem->mem_left);
	}
	else if (*buf)
	{
		ds_Free((u8 *) *buf - slot_size);
	}
	*heap_allocated = 0;
	*buf = NULL;
	*length = 0;
	*count = 0;
	*count_max = 0;
	*next_free = POOL_INDEX_MASK;
	*growable = alloc_growable;
	ss->bit_index = bit_index;
	return 0;
}
