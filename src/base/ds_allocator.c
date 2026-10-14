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

#if __DS_PLATFORM__ == __DS_LINUX__ || __DS_PLATFORM__ == __DS_WEB__

#define _GNU_SOURCE

#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "ds_base.h"
#include "ds_serialize.h"

struct memConfig g_mem_config_storage = { 0 };
struct memConfig *g_mem_config = &g_mem_config_storage;

u32 PowerOfTwoCheck(const u64 n)
{
	/* k > 0: 2^k =>   (10... - 1)  =>   (01...) = 0
	 *               & (10...    )     & (10...)
	 */

	/* k > 0: NOT 2^k =>   (1XXX10... - 1)  =>   (1XXX01...) = (1XXX00...
	 *                   & (1XXX10...    )     & (1XXX10...)
	 */

	return (n & (n-1)) == 0 && n > 0;
}

u64 PowerOfTwoCeil(const u64 n)
{
	ds_AssertString(n <= ((u64) 1 << 63), "Overflow in PowerOfTwoCeil");
	/* 2^k >= n > 2^(k-1) <=> n-1 has bit length k */
	return (n <= 1) ? 1 : (u64) 1 << (64 - Clz64(n - 1));
}

void ds_MemApiShutdown(void)
{
}

void ds_MemApiInit()
{
	g_mem_config->page_size = ds_Pagesize();
	ds_Assert( PowerOfTwoCheck( g_mem_config->page_size ) );
}

/* debug builds fill new memory: nothing may rely on it being zeroed */
#ifdef DS_DEBUG
	#define DS_ALLOC_DEBUG_FILL(addr, size)	memset((addr), 0xa5, (size))
#else
	#define DS_ALLOC_DEBUG_FILL(addr, size)
#endif

#if __DS_PLATFORM__ == __DS_LINUX__ || __DS_PLATFORM__ == __DS_WEB__

#include <stddef.h>
#include <sys/mman.h>

void *ds_Alloc(const u64 size, const u64 alignment)
{
	ds_Assert(PowerOfTwoCheck(alignment));
	if (size == 0)
	{
		return NULL;
	}

	/* posix_memalign: the alignment must be a multiple of sizeof(void *) */
	const u64 alignment_used = (alignment < sizeof(void *)) ? sizeof(void *) : alignment;
	void *addr = NULL;
	if (posix_memalign(&addr, alignment_used, size) != 0)
	{
		return NULL;
	}

	DS_ALLOC_DEBUG_FILL(addr, size);
	return addr;
}

/* resize a non-NULL block to new_size > 0; NULL on failure */
static void *ds_ReallocPlatform(void *addr, const u64 old_size, const u64 new_size, const u64 alignment)
{
	/* realloc keeps malloc's alignment (max_align_t); glibc moves large blocks with mremap, without a copy */
	if (alignment <= _Alignof(max_align_t))
	{
		return realloc(addr, new_size);
	}

	void *new_addr = ds_Alloc(new_size, alignment);
	if (new_addr)
	{
		memcpy(new_addr, addr, (old_size < new_size) ? old_size : new_size);
		free(addr);
	}
	return new_addr;
}

void ds_Free(void *addr)
{
	free(addr);
}

#elif __DS_PLATFORM__ == __DS_WIN64__

#include <malloc.h>

void *ds_Alloc(const u64 size, const u64 alignment)
{
	ds_Assert(PowerOfTwoCheck(alignment));
	if (size == 0)
	{
		return NULL;
	}

	void *addr = _aligned_malloc(size, alignment);
	if (addr)
	{
		DS_ALLOC_DEBUG_FILL(addr, size);
	}
	return addr;
}

/* resize a non-NULL block to new_size > 0; NULL on failure */
static void *ds_ReallocPlatform(void *addr, const u64 old_size, const u64 new_size, const u64 alignment)
{
	return _aligned_realloc(addr, new_size, alignment);
}

void ds_Free(void *addr)
{
	_aligned_free(addr);
}

#else

#error

#endif

/*
TODO: a dedicated heap allocator. It would know its block sizes, so old_size could go (only the aligned
copy on Linux/web needs it).
*/
void *ds_Realloc(void *addr, const u64 old_size, const u64 new_size, const u64 alignment)
{
	ds_Assert(PowerOfTwoCheck(alignment));
	if (new_size == 0)
	{
		ds_Free(addr);
		return NULL;
	}

	void *new_addr = (addr)
		? ds_ReallocPlatform(addr, old_size, new_size, alignment)
		: ds_Alloc(new_size, alignment);
	if (new_addr == NULL)
	{
		/* a failed resize leaves addr intact: a shrink keeps the larger block, only growth is fatal */
		if (addr && new_size <= old_size)
		{
			return addr;
		}
		LogString(T_SYSTEM, S_FATAL, "Failed to grow a block in ds_Realloc, exiting.");
		FatalCleanupAndExit();
	}

	if (addr && old_size < new_size)
	{
		DS_ALLOC_DEBUG_FILL((u8 *) new_addr + old_size, new_size - old_size);
	}
	return new_addr;
}


u32 ds_CPoolAllocInternal(struct arena *mem, void **buf, u32 *length, u32 *count, u32 *growable, u32 *heap_allocated, const u64 slot_size, const u32 alloc_length, const u32 alloc_growable)
{
	ds_Assert(!alloc_growable || !mem);
	*buf = NULL;
	*length = 0;
	*count = 0;
	*growable = alloc_growable;
	*heap_allocated = 0;

	/* the stub is allocated for alloc_length == 0 too: pushes onto a full pool write into it */
	const u64 size = slot_size*((u64) alloc_length + 1);
	void *addr = (mem)
		? ArenaPushAligned(mem, size, DS_CACHE_LINE)
		: ds_Alloc(size, DS_CACHE_LINE);

	ds_Assert((u64) addr % DS_CACHE_LINE == 0);
	if (addr == NULL)
	{
		return 0;
	}

	*buf = (u8 *) addr + slot_size;
	*length = alloc_length;
	*heap_allocated = (mem == NULL);
	PoisonAddress(*buf, size - slot_size);
	return 1;
}

void ds_CPoolSerializeInternal(struct ss *ss, const void *buf, const u32 length, const u32 count, const u64 slot_size)
{
	ds_Assert(ss->bit_index % 8 == 0);
	ds_Assert(2*sizeof(u32) + count*slot_size <= ss_BytesLeft(ss));

	ss_WriteU32Le(ss, length);
	ss_WriteU32Le(ss, count);
	ss_Write8N(ss, (const b8 *) buf, count*slot_size);
}

u32 ds_CPoolTryDeserializeInternal(struct arena *mem, struct ss *ss, void **buf, u32 *length, u32 *count, u32 *growable, u32 *heap_allocated, const u64 slot_size, const u32 alloc_growable)
{
	ds_Assert(ss->bit_index % 8 == 0);

	*buf = NULL;
	*length = 0;
	*count = 0;
	*growable = alloc_growable;
	*heap_allocated = 0;
	if (ss_BytesLeft(ss) < 2*sizeof(u32))
	{
		return 0;
	}

	const u64 bit_index = ss->bit_index;
	const u32 serialized_length = ss_ReadU32Le(ss);
	const u32 serialized_count = ss_ReadU32Le(ss);
	/* division instead of count*slot_size: a corrupt count must not overflow the check */
	if (serialized_count > serialized_length || serialized_count > ss_BytesLeft(ss) / slot_size
		|| !ds_CPoolAllocInternal(mem, buf, length, count, growable, heap_allocated, slot_size, serialized_length, alloc_growable))
	{
		ss->bit_index = bit_index;
		return 0;
	}

	UnpoisonAddress(*buf, serialized_count*slot_size);
	ss_Read8N((b8 *) *buf, ss, serialized_count*slot_size);
	*count = serialized_count;
	return 1;
}

void ds_PoolSerializeInternal(struct ss *ss, const void *buf, const u32 length, const u32 count, const u32 count_max, const u32 next_free, const u64 slot_size, const u64 pool_slot_offset)
{
	ds_Assert(ss->bit_index % 8 == 0);
	ds_Assert(3*sizeof(u32) + (u64) (count_max - count)*sizeof(u32) + (u64) count*slot_size <= ss_BytesLeft(ss));

	const u8 *base = buf;
	ss_WriteU32Le(ss, length);
	ss_WriteU32Le(ss, count);
	ss_WriteU32Le(ss, count_max);

	u32 free_count = 0;
	for (u32 i = next_free; i != POOL_INDEX_MASK; i = *(const u32 *) (base + i*slot_size + pool_slot_offset) & POOL_INDEX_MASK)
	{
		ss_WriteU32Le(ss, i);
		free_count += 1;
	}
	ds_Assert(free_count == count_max - count);

	for (u32 i = 0; i < count_max; ++i)
	{
		const u8 *slot = base + i*slot_size;
		if (*(const u32 *) (slot + pool_slot_offset) & POOL_ALLOCATION_MASK)
		{
			continue;
		}

		ss_Write8N(ss, (const b8 *) slot, pool_slot_offset);
		ss_WriteU32Le(ss, i);
		ss_Write8N(ss, (const b8 *) (slot + pool_slot_offset + sizeof(u32)), slot_size - pool_slot_offset - sizeof(u32));
	}
}

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

u32 ds_PoolTryDeserializeInternal(struct arena *mem, struct ss *ss, u32 *heap_allocated, void **buf, u32 *length, u32 *count, u32 *count_max, u32 *next_free, u32 *growable, const u64 slot_size, const u64 pool_slot_offset, const u32 alloc_growable)
{
	ds_Assert(ss->bit_index % 8 == 0);

	const u64 bit_index = ss->bit_index;
	const u64 mem_left = (mem) ? mem->mem_left : 0;
	*heap_allocated = 0;
	*buf = NULL;
	if (ss_BytesLeft(ss) < 3*sizeof(u32))
	{
		goto failure;
	}

	const u32 serialized_length = ss_ReadU32Le(ss);
	const u32 serialized_count = ss_ReadU32Le(ss);
	const u32 serialized_count_max = ss_ReadU32Le(ss);

	/* divisions instead of multiplications: corrupt counts must not overflow the checks */
	const u64 free_count = (u64) serialized_count_max - serialized_count;
	const u64 bytes_left = ss_BytesLeft(ss);
	if (serialized_count > serialized_count_max || serialized_count_max > serialized_length
		|| serialized_length >= POOL_INDEX_MASK || free_count > bytes_left / sizeof(u32)
		|| serialized_count > (bytes_left - free_count*sizeof(u32)) / slot_size
		|| !ds_PoolAllocInternal(mem, heap_allocated, buf, length, count, count_max, next_free, growable, slot_size, serialized_length, alloc_growable))
	{
		goto failure;
	}

	/*
	 * indices are written into the pool buffer, so each one must be in range and appear once (free list
	 * and allocated slots together cover [0, count_max)): pool_slot starts as POOL_INDEX_MASK, a state no
	 * finished slot has (allocated: 0, free: mask bit set), and a second visit fails the deserialization
	 */
	u8 *base = *buf;
	for (u32 i = 0; i < serialized_count_max; ++i)
	{
		u32 *pool_slot = (u32 *) (base + i*slot_size + pool_slot_offset);
		UnpoisonAddress(pool_slot, sizeof(u32));
		*pool_slot = POOL_INDEX_MASK;
	}

	u32 *prev_link = next_free;
	for (u64 i = 0; i < free_count; ++i)
	{
		const u32 index = ss_ReadU32Le(ss);
		if (index >= serialized_count_max)
		{
			goto failure;
		}

		u32 *link = (u32 *) (base + index*slot_size + pool_slot_offset);
		if (*link != POOL_INDEX_MASK)
		{
			goto failure;
		}

		/* end of the list until a successor is linked */
		*link = POOL_ALLOCATION_MASK | POOL_INDEX_MASK;
		*prev_link = (prev_link == next_free) ? index : (POOL_ALLOCATION_MASK | index);
		prev_link = link;
	}

	for (u32 i = 0; i < serialized_count; ++i)
	{
		struct ss peek = *ss;
		peek.bit_index += 8*pool_slot_offset;
		const u32 index = ss_ReadU32Le(&peek);
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
		ss_Read8N((b8 *) slot, ss, slot_size);
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

void ArenaPushRecord(struct arena *ar)
{
	const u64 rec_mem_left = ar->mem_left;
	struct arenaRecord *record = ArenaPush(ar, sizeof(struct arenaRecord));
	/* a missing record would make the matching ArenaPopRecord pop the outer record */
	if (!record)
	{
		LogString(T_SYSTEM, S_FATAL, "Arena full in ArenaPushRecord, exiting.");
		FatalCleanupAndExit();
	}

	record->prev = ar->record;
	record->rec_mem_left = rec_mem_left;
	ar->record = record;
}

void ArenaPopRecord(struct arena *ar)
{
	if (ar->record)
	{
		ds_Assert((u64) ar->record <= (u64) ar->stack_ptr);
		ds_Assert(ar->mem_left <= ar->record->rec_mem_left);
		const u64 rec_mem_left = ar->record->rec_mem_left;
		ar->record = ar->record->prev;
		ArenaPopPacked(ar, rec_mem_left - ar->mem_left);
	}
}

void ArenaRemoveRecord(struct arena *ar)
{
	if (ar->record)
	{
		ar->record = ar->record->prev;
	}
}

struct arena ArenaAlloc(struct arena *mem, const u64 size)
{
	struct arena ar =
	{
		.mem_size = 0,
		.mem_left = 0,
		.record = NULL,
	};

	ar.stack_ptr = (mem)
		? ArenaPushAligned(mem, size, DS_CACHE_LINE)
		: ds_Alloc(size, DS_CACHE_LINE);
	if (ar.stack_ptr)
	{
		ar.mem_size = size;
		ar.mem_left = size;
		ar.heap_allocated = (mem == NULL);
		PoisonAddress(ar.stack_ptr, ar.mem_left);
	}

	return ar;
}

void ArenaFree(struct arena *ar)
{
	ar->stack_ptr -= ar->mem_size - ar->mem_left;
	UnpoisonAddress(ar->stack_ptr, ar->mem_size);
	if (ar->heap_allocated)
	{
		ds_Free(ar->stack_ptr);
	}
	ar->heap_allocated = 0;
	ar->mem_size = 0;
	ar->mem_left = 0;
	ar->stack_ptr = NULL;
	ar->record = NULL;
}

void ArenaFlush(struct arena* ar)
{
	ar->stack_ptr -= ar->mem_size - ar->mem_left;
	ar->mem_left = ar->mem_size;
	ar->record = NULL;
	PoisonAddress(ar->stack_ptr, ar->mem_left);
}

void ArenaPopPacked(struct arena *ar, const u64 mem_to_pop)
{
	ds_AssertString(ar->mem_size - ar->mem_left >= mem_to_pop, "Trying to pop memory outside of arena");
	ar->stack_ptr -= mem_to_pop;
	ar->mem_left += mem_to_pop;
	PoisonAddress(ar->stack_ptr, mem_to_pop);
}

void *ArenaPushAligned(struct arena *ar, const u64 size, const u64 alignment)
{
	ds_Assert(PowerOfTwoCheck(alignment) == 1);

	void* alloc_addr = NULL;
	if (size) 
	{ 
		/* bytes up to the next multiple of alignment */
		const u64 push_alignment = (0 - (u64) ar->stack_ptr) & (alignment - 1);
		if (ar->mem_left >= size + push_alignment)
		{
			UnpoisonAddress(ar->stack_ptr + push_alignment, size);
			alloc_addr = ar->stack_ptr + push_alignment;
			ar->mem_left -= size + push_alignment;
			ar->stack_ptr += size + push_alignment;
            ds_Assert((u64) alloc_addr % alignment == 0);
		}
	}

	return alloc_addr;
}


void *ArenaPushAlignedMemcpy(struct arena *ar, const void *copy, const u64 size, const u64 alignment)
{
	void *addr = ArenaPushAligned(ar, size, alignment);
	if (addr)
	{
		memcpy(addr, copy, size);
	}
	return addr;
}

void *ArenaPushAlignedZero(struct arena *ar, const u64 size, const u64 alignment)
{
	void *addr = ArenaPushAligned(ar, size, alignment);
	if (addr)
	{
		memset(addr, 0, size);
	}
	return addr;
}

struct memArray ArenaPushAlignedAll(struct arena *ar, const u64 slot_size, const u64 alignment)
{
	ds_Assert(PowerOfTwoCheck(alignment) == 1 && slot_size > 0);

	struct memArray array = { .len = 0, .addr = NULL, .memPushed = 0 };
	/* bytes up to the next multiple of alignment */
	const u64 push_alignment = (0 - (u64) ar->stack_ptr) & (alignment - 1);
	if (push_alignment + slot_size <= ar->mem_left)
	{
		array.len = (ar->mem_left - push_alignment) / slot_size;
		array.addr = ar->stack_ptr + push_alignment;
		UnpoisonAddress(ar->stack_ptr + push_alignment, array.len * slot_size);
		array.memPushed = push_alignment + array.len * slot_size;
		ar->mem_left  -= push_alignment + array.len * slot_size;
		ar->stack_ptr += push_alignment + array.len * slot_size;
	}

	return array;
}

struct ring RingEmpty()
{
	return (struct ring) { .mem_total = 0, .mem_left = 0, .offset = 0, .buf = NULL };
}

#if __DS_PLATFORM__ == __DS_LINUX__

#include <fcntl.h>

struct ring RingAlloc(const u64 mem_hint)
{
	ds_Assert(mem_hint);

	struct ring ring = { 0 };
	/* whole pages (page_size is a power of two) */
	const u64 page = g_mem_config->page_size;
	ring.mem_total = (mem_hint + page - 1) & ~(page - 1);
	ring.mem_left = ring.mem_total;
	ring.offset = 0;
	ring.buf = mmap(NULL, ring.mem_total << 1, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

	if (ring.buf == MAP_FAILED)
	{
		Log(T_SYSTEM, S_ERROR, "Failed to allocate ring allocator: %s", strerror(errno));
		return RingEmpty();
	}
	void *p1 = mmap(ring.buf, ring.mem_total, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
	void *p2 = mmap(ring.buf + ring.mem_total, ring.mem_total, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
	if (p1 == MAP_FAILED || p2 == MAP_FAILED)
	{
		Log(T_SYSTEM, S_ERROR, "Failed to allocate ring allocator: %s", strerror(errno));
		return RingEmpty();
	}

	madvise(ring.buf, ring.mem_total << 1, MADV_HUGEPAGE);
	madvise(ring.buf, ring.mem_total << 1, MADV_WILLNEED);

	return ring;
}

void RingDealloc(struct ring *ring)
{
	if (munmap(ring->buf, 2*ring->mem_total) == -1)
	{
		Log(T_SYSTEM, S_ERROR, "%s:%d - %s", __FILE__, __LINE__, strerror(errno));
	}
	*ring = RingEmpty();
}

#elif __DS_PLATFORM__ == __DS_WIN64__

#include <memoryapi.h>

struct ring RingAlloc(const u64 mem_hint)
{
	ds_Assert(mem_hint);

	SYSTEM_INFO info;
	GetSystemInfo(&info);

	u64 bufsize = PowerOfTwoCeil(mem_hint);
	if (bufsize < info.dwAllocationGranularity)
	{
		bufsize = info.dwAllocationGranularity;
	}
	u8 *alloc = VirtualAlloc2(NULL, NULL, 2*bufsize, MEM_RESERVE | MEM_RESERVE_PLACEHOLDER, PAGE_NOACCESS, NULL, 0);
	if (alloc == NULL)
	{
		LogSystemError(S_ERROR);
		return RingEmpty();
	}

	/* split into two placeholders, [alloc, alloc + bufsize) and [alloc + bufsize, alloc + 2*bufsize) */
	if (!VirtualFree(alloc, bufsize, MEM_RELEASE | MEM_PRESERVE_PLACEHOLDER))
	{
		LogSystemError(S_ERROR);
		VirtualFree(alloc, 0, MEM_RELEASE);
		return RingEmpty();
	}

	/* fails with NULL, not INVALID_HANDLE_VALUE */
	HANDLE map = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, (DWORD) (bufsize >> 32), (DWORD) ((u32) bufsize), NULL);
	if (map == NULL)
	{
		LogSystemError(S_ERROR);
		goto release_placeholders;
	}

	u8 *buf = MapViewOfFile3(map, INVALID_HANDLE_VALUE, alloc, 0, bufsize, MEM_REPLACE_PLACEHOLDER, PAGE_READWRITE, NULL, 0);
	if (buf == NULL)
	{
		LogSystemError(S_ERROR);
		goto close_map;
	}

	if (MapViewOfFile3(map, INVALID_HANDLE_VALUE, alloc + bufsize, 0, bufsize, MEM_REPLACE_PLACEHOLDER, PAGE_READWRITE, NULL, 0) == NULL)
	{
		LogSystemError(S_ERROR);
		UnmapViewOfFile(buf);
		CloseHandle(map);
		VirtualFree(alloc + bufsize, 0, MEM_RELEASE);
		return RingEmpty();
	}

	/* the views keep the mapping alive */
	CloseHandle(map);

	return (struct ring) { .mem_total = bufsize, .mem_left = bufsize, .offset = 0, .buf = buf };

close_map:
	CloseHandle(map);
release_placeholders:
	VirtualFree(alloc, 0, MEM_RELEASE);
	VirtualFree(alloc + bufsize, 0, MEM_RELEASE);
	return RingEmpty();
}

void RingDealloc(struct ring *ring)
{
	if (!UnmapViewOfFile(ring->buf))
	{
		LogSystemError(S_ERROR);
	}
	if (!UnmapViewOfFile(ring->buf + ring->mem_total))
	{
		LogSystemError(S_ERROR);
	}
	*ring = RingEmpty();
}

#endif

void RingFlush(struct ring *ring)
{
	ring->mem_left = ring->mem_total;
	ring->offset = 0;
}

struct ringSlot RingPushStart(struct ring *ring, const u64 size)
{
	struct ringSlot buf = { 0 };
	if (size && size <= ring->mem_left)
	{
		ring->mem_left -= size;
		buf.address = ring->buf + ((ring->offset + ring->mem_left) % ring->mem_total);
		buf.size = size;
	}

	return buf;
}

struct ringSlot RingPushEnd(struct ring *ring, const u64 size)
{
	struct ringSlot buf = { 0 };
	if (size && size <= ring->mem_left)
	{
		buf.address = ring->buf + ring->offset;
		buf.size = size;
		ring->mem_left -= size;
		ring->offset = (ring->offset + size) % ring->mem_total;
	}

	return buf;
}

void RingPopStart(struct ring *ring, const u64 size)
{
	ds_Assert(size + ring->mem_left <= ring->mem_total);
	ring->mem_left += size;
}

void RingPopEnd(struct ring *ring, const u64 size)
{
	ds_Assert(size + ring->mem_left <= ring->mem_total);
	ring->mem_left += size;
	ring->offset = (ring->mem_total + ring->offset - size) % ring->mem_total;
}
