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
	if (n == 0)
	{
		return 1;
	}

	if (PowerOfTwoCheck(n))
	{
		return n;
	}

	/* [1, 63] */
	const u32 lz = Clz64(n);
	ds_AssertString(lz > 0, "Overflow in PowerOfTwoCeil");
	return (u64) 0x8000000000000000 >> (lz-1);
}

u64 ds_AllocSizeCeil(const u64 size)
{
	const u64 mod = size & (g_mem_config->page_size - 1);
	return (mod) 
		? size + g_mem_config->alloc_size_min - mod
		: size;
}

static void ds_MemApiInitShared(const u32 count_256B, const u32 count_1MB)
{
	ds_StaticAssert(((u64) &((struct threadBlockAllocator *) 0)->a_next % DS_CACHE_LINE_UB) == 0, "Expected Alignment");
	ThreadBlockAllocatorAlloc(&g_mem_config->block_allocator_256B, count_256B, 256);
	ThreadBlockAllocatorAlloc(&g_mem_config->block_allocator_1MB, count_1MB, 1024*1024);
}

void ds_MemApiShutdown(void)
{
	ThreadBlockAllocatorFree(&g_mem_config->block_allocator_256B);
	ThreadBlockAllocatorFree(&g_mem_config->block_allocator_1MB);
}

#if __DS_PLATFORM__ == __DS_LINUX__

#include <unistd.h>
#include <sys/mman.h>

void ds_MemApiInit(const u32 count_256B, const u32 count_1MB)
{
	g_mem_config->page_size = getpagesize();
	g_mem_config->alloc_size_min = g_mem_config->page_size;
	ds_MemApiInitShared(count_256B, count_1MB);
}

void *ds_Alloc(struct memSlot *slot, const u64 size, const u32 huge_pages)
{
	ds_Assert(size); 

	u64 size_used = ds_AllocSizeCeil(size);
	void *addr = mmap(NULL, size_used, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (addr == MAP_FAILED)
	{
		addr = NULL;
		size_used = 0;
	}
	else if (huge_pages)
	{
		madvise(addr, size_used, MADV_HUGEPAGE);
	}

	slot->address = addr;
	slot->size = size_used;
	slot->huge_pages = huge_pages;

	ds_Assert(((u64) slot->address) % g_mem_config->page_size == 0);

	return slot->address;
}

void *ds_Realloc(struct memSlot *slot, const u64 size)
{
	if (size < slot->size)
	{
		if (slot->huge_pages)
		{
			struct memSlot newSlot;
			if (ds_Alloc(&newSlot, size, HUGE_PAGES))
			{
				memcpy(newSlot.address, slot->address, slot->size);
			}
			ds_Free(slot);
			*slot = newSlot;
		}
		else
		{
			slot->address = mremap(slot->address, slot->size, size, MREMAP_MAYMOVE);
			slot->size = size;
		}

		if (slot->address == MAP_FAILED || slot->address == NULL)
		{
			LogString(T_SYSTEM, S_FATAL, "Failed to reallocate memSlot in ds_Realloc, exiting.");
			FatalCleanupAndExit();
		}
	}

	return slot->address;
}

void ds_Free(struct memSlot *slot)
{
	munmap(slot->address, slot->size);	
	slot->address = NULL;
	slot->size = 0;
	slot->huge_pages = 0;
}

#elif __DS_PLATFORM__ == __DS_WEB__

#include <unistd.h>
#include <sys/mman.h>

void ds_MemApiInit(const u32 count_256B, const u32 count_1MB)
{
	g_mem_config->page_size = getpagesize();
	g_mem_config->alloc_size_min = g_mem_config->page_size;
	ds_MemApiInitShared(count_256B, count_1MB);
}

void *ds_Alloc(struct memSlot *slot, const u64 size, const u32 garbage)
{
	ds_Assert(size); 

	u64 size_used = ds_AllocSizeCeil(size);
	void *addr = mmap(NULL, size_used, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (addr == MAP_FAILED)
	{
		addr = NULL;
		size_used = 0;
	}

	slot->address = addr;
	slot->size = size_used;
	slot->huge_pages = 0;

	ds_Assert(((u64) slot->address) % g_mem_config->page_size == 0);

	return slot->address;
}

void *ds_Realloc(struct memSlot *slot, const u64 size)
{
	ds_Assert(size > slot->size);

	struct memSlot newSlot;
	if (ds_Alloc(&newSlot, size, 0))
	{
		memcpy(newSlot.address, slot->address, slot->size);
	}
	ds_Free(slot);
	*slot = newSlot;
	
	if (slot->address == MAP_FAILED)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to reallocate memSlot in ds_Realloc, exiting.");
		FatalCleanupAndExit();
	}

	return slot->address;
}

void ds_Free(struct memSlot *slot)
{
	munmap(slot->address, slot->size);	
	slot->address = NULL;
	slot->size = 0;
	slot->huge_pages = 0;
}
#elif __DS_PLATFORM__ == __DS_WIN64__

#elif 

#error

#endif


void ArenaPushRecord(struct arena *ar)
{
	const u64 rec_mem_left = ar->mem_left;
	struct arenaRecord *record = ArenaPush(ar, sizeof(struct arenaRecord));
	if (record)
	{
		record->prev = ar->record;
		record->rec_mem_left = rec_mem_left;
		ar->record = record;
	}
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

struct arena ArenaAlloc(const u64 size)
{
	struct arena ar =
	{
		.mem_size = 0,
		.mem_left = 0,
		.record = NULL,
	};

	ar.stack_ptr = (size >= 2*1024*1024)
		? ds_Alloc(&ar.slot, size, HUGE_PAGES)
		: ds_Alloc(&ar.slot, size, NO_HUGE_PAGES);

	if (ar.stack_ptr)
	{
		ar.mem_size = ar.slot.size;
		ar.mem_left = ar.slot.size;
		PoisonAddress(ar.stack_ptr, ar.mem_left);
	}
	
	return ar;
}

void ArenaFree(struct arena *ar)
{
	ar->stack_ptr -= ar->mem_size - ar->mem_left;
	UnpoisonAddress(ar->stack_ptr, ar->mem_size);
	ds_Free(&ar->slot);
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
		const u64 mod = ((u64) ar->stack_ptr) & (alignment - 1);
		const u64 push_alignment = (!!mod) * (alignment - mod);

		if (ar->mem_left >= size + push_alignment) 
		{
			UnpoisonAddress(ar->stack_ptr + push_alignment, size);
			alloc_addr = ar->stack_ptr + push_alignment;
			ar->mem_left -= size + push_alignment;
			ar->stack_ptr += size + push_alignment;
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
	const u64 mod = ((u64) ar->stack_ptr) & (alignment - 1);
	const u64 push_alignment = (!!mod) * (alignment - mod);
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

struct arena ArenaAlloc1MB(void)
{
	struct arena ar =
	{
		.mem_size = 0,
		.mem_left = 0,
		.record = NULL,
	};

	ar.stack_ptr = ThreadAlloc1MB();
	if (ar.stack_ptr)
	{
		ar.mem_size = 1024*1024;
		ar.mem_left = 1024*1024;
		PoisonAddress(ar.stack_ptr, ar.mem_left);
	}

	return ar;
}

void ArenaFree1MB(struct arena *ar)
{
	ar->stack_ptr -= ar->mem_size - ar->mem_left;
	UnpoisonAddress(ar->stack_ptr, ar->mem_size);
	ThreadFree1MB(ar->stack_ptr);
}

enum threadAllocRet
{
	ALLOCATOR_SUCCESS,
	ALLOCATOR_FAILURE,
	ALLOCATOR_OUT_OF_MEMORY,
	ALLOCATOR_COUNT
};

struct threadBlockHeader
{
	u64 	id;
	u64 	next;
};

#define LOCAL_MAX_COUNT		32
#define LOCAL_FREE_LOW  	16
#define LOCAL_FREE_HIGH 	31
static dsThreadLocal u32 local_count = 1;	/* local_next[0] is dummy */
static dsThreadLocal u64 local_next[LOCAL_MAX_COUNT];

void ThreadBlockAllocatorAlloc(struct threadBlockAllocator *allocator, const u64 block_count, const u64 block_size)
{
	ds_StaticAssert(LOCAL_MAX_COUNT - 1 == LOCAL_FREE_HIGH, "");
	ds_StaticAssert(LOCAL_FREE_LOW <= LOCAL_FREE_HIGH, "");
	ds_StaticAssert(1 <= LOCAL_FREE_LOW, "");

	const u64 mod = (block_size % DS_CACHE_LINE_UB);
	allocator->block_size = (mod)
		? DS_CACHE_LINE_UB + block_size + (DS_CACHE_LINE_UB - mod)
		: DS_CACHE_LINE_UB + block_size;

	const u64 size_used = ds_AllocSizeCeil(block_count * allocator->block_size);
	allocator->max_count = size_used / allocator->block_size;
	allocator->block = ds_Alloc(&allocator->mem_slot, size_used, HUGE_PAGES);

	ds_AssertString(((u64) allocator->block & (DS_CACHE_LINE_UB-1)) == 0, "allocator block array should be cacheline aligned");
	if (!allocator->block)
	{
		LogString(T_SYSTEM, S_FATAL, "Failed to allocate block allocator->block");
		FatalCleanupAndExit();
	}
	/* sync point (gen, index) = (0,0) */
	AtomicStoreRel64(&allocator->a_next, 0);
}

void ThreadBlockAllocatorFree(struct threadBlockAllocator *allocator)
{
	ds_Free(&allocator->mem_slot);
}

static enum threadAllocRet ThreadBlockTryAlloc(void **addr, u64 *a_next, struct threadBlockAllocator *allocator)
{
	/* a_next has been AQUIRED, so no local write/reads can move above (compiler and cpu), 
	 * stores by other threads that have released the address is now visible locally.  */
	const u64 gen = *a_next >> 32;
	const u64 index = *a_next & U32_MAX;
	if (index == allocator->max_count) { return ALLOCATOR_OUT_OF_MEMORY; }
	
	/* This block may already be owned by another thread, or undefined. Thus, we view the header values
	 * as garbage if the generation is 0 OR if the allocator has been tampered with since our AQUIRE of
	 * a_next. */
	struct threadBlockHeader *header = (struct threadBlockHeader *) (allocator->block + index*allocator->block_size);

	/* Unallocated blocks always start on generation 0, that we we can identify if the free list is empty */
	const u64 new_next = (gen == 0)
		? index+1
		: AtomicLoadRlx64(&header->next);

	/* Relaxed store on success, Aquire read on failure: 
	 * 	If we succeed in the exchange, no tampering of the allocator has happened and we make 
	 * 	can make a relaxed store; The only store that other threads need to read from us is
	 * 	the sync point itself, so it may be relaxed. (GCC) Since the failure memorder may not 
	 * 	be weaker than the success order, we must use ACQ, ACQ.
	 *
	 * 	If we fail in the exchange, we need to make an aquired read of a_next again; The allocator
	 * 	may be in a state in which we will read thread written headers.  */
	if (AtomicCompareExchangeAcq64(&allocator->a_next, a_next, new_next))
	{
		*addr = (u8 *) header + DS_CACHE_LINE_UB;
		/* update generation */
		header->id = *a_next + ((u64) 1 << 32);
		return ALLOCATOR_SUCCESS;
	}
	else
	{
		return ALLOCATOR_FAILURE;
	}
}

static enum threadAllocRet ThreadBlockTryFree(struct threadBlockHeader *header, struct threadBlockAllocator *allocator, const u64 new_next)
{
	/*
	 * On success, we release store, making our local header changes visible for any thread trying to allocate
	 * this block again. 
	 *
	 * On failure, we may do a relaxed load of the next allocation identifier. We are never dereferencing it
	 * in our pop procedure, so this is okay.
	 */
	return (AtomicCompareExchangeRel64(&allocator->a_next, &header->next, new_next))
		? ALLOCATOR_SUCCESS
		: ALLOCATOR_FAILURE;
}

void *ThreadBlockAlloc(struct threadBlockAllocator *allocator)
{
	void *addr;
	enum threadAllocRet ret;

	u64 a_next = AtomicLoadAcq64(&allocator->a_next);
	while ((ret = ThreadBlockTryAlloc(&addr, &a_next, allocator)) == ALLOCATOR_FAILURE)
		;

	ds_Assert(ret != ALLOCATOR_OUT_OF_MEMORY);

	return (ret != ALLOCATOR_OUT_OF_MEMORY) 
		? addr 
		: NULL;
}


void ThreadBlockFree(struct threadBlockAllocator *allocator, void *addr)
{
	struct threadBlockHeader *header = (struct threadBlockHeader *) ((u8 *) addr - DS_CACHE_LINE_UB);
	header->next = AtomicLoadRlx64(&allocator->a_next);
	while (ThreadBlockTryFree(header, allocator, header->id) == ALLOCATOR_FAILURE);
}

void *ThreadBlockAlloc256B(struct threadBlockAllocator *allocator)
{
	void *addr;
	enum threadAllocRet ret;

	if (local_count > 1)
	{
		const u64 next = local_next[--local_count];
		const u32 index = next & U32_MAX;
		struct threadBlockHeader *header = (struct threadBlockHeader *) (allocator->block + index*allocator->block_size);
		header->id = next + ((u64) 1 << 32);
		addr = (u8 *) header + DS_CACHE_LINE_UB;
		ret = ALLOCATOR_SUCCESS;
	}
	else
	{
		u64 a_next = AtomicLoadAcq64(&allocator->a_next);
		while ((ret = ThreadBlockTryAlloc(&addr, &a_next, allocator)) == ALLOCATOR_FAILURE);
	}

	return (ret != ALLOCATOR_OUT_OF_MEMORY) 
		? addr 
		: NULL;
}

void ThreadBlockFree256B(struct threadBlockAllocator *allocator, void *addr)
{
	struct threadBlockHeader *header;
	if (local_count == LOCAL_MAX_COUNT)
	{
		/* local_next[0] (DUMMY)  <- local_next[1] <- ... <- local_next[LOCAL_FREE_HIGH] */
		u64 head = local_next[LOCAL_FREE_HIGH];
		u64 tail = local_next[LOCAL_FREE_LOW];

		header = (struct threadBlockHeader *) (allocator->block + (tail & U32_MAX)*allocator->block_size);
		header->next = AtomicLoadRlx64(&allocator->a_next);
		while (ThreadBlockTryFree(header, allocator, head) == ALLOCATOR_FAILURE);
		local_count = LOCAL_FREE_LOW;
	}

	/* local_next[0] (DUMMY)  <- local_next[1] <- ... <- local_next[local_count] */
	header = (struct threadBlockHeader *) ((u8 *) addr - DS_CACHE_LINE_UB);
	AtomicStoreRel32(&header->next, local_next[local_count-1]);
	local_next[local_count++] = header->id;
}

void *ThreadAlloc256B(void)
{
	return ThreadBlockAlloc256B(&g_mem_config->block_allocator_256B);
}

void *ThreadAlloc1MB(void)
{
	return ThreadBlockAlloc(&g_mem_config->block_allocator_1MB);
}

void ThreadFree256B(void *addr)
{
	ThreadBlockFree256B(&g_mem_config->block_allocator_256B, addr);
}

void ThreadFree1MB(void *addr)
{
	ThreadBlockFree(&g_mem_config->block_allocator_1MB, addr);
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
	const u64 mod = mem_hint % g_mem_config->page_size;

	struct ring ring = { 0 };
	ring.mem_total = mem_hint + (!!mod) * (g_mem_config->page_size - mod),
	ring.mem_left = ring.mem_total;
	ring.offset = 0;
	ring.buf = mmap(NULL, ring.mem_total << 1, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

	if (ring.buf == MAP_FAILED)
	{
		LogString(T_SYSTEM, S_ERROR, "Failed to allocate ring allocator: %s", strerror(errno));
		return RingEmpty();
	}
	void *p1 = mmap(ring.buf, ring.mem_total, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
	void *p2 = mmap(ring.buf + ring.mem_total, ring.mem_total, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
	if (p1 == MAP_FAILED || p2 == MAP_FAILED)
	{
		LogString(T_SYSTEM, S_ERROR, "Failed to allocate ring allocator: %s", strerror(errno));
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

	if (!VirtualFree(alloc, bufsize, MEM_RELEASE | MEM_PRESERVE_PLACEHOLDER))
	{
		LogSystemError(S_ERROR);
		return RingEmpty();
	}

	HANDLE map = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, (DWORD) (bufsize >> 32), (DWORD) ((u32) bufsize), NULL);
	if (map == INVALID_HANDLE_VALUE)
	{
		LogSystemError(S_ERROR);
		return RingEmpty();
	}

	u8 *buf = MapViewOfFile3(map, INVALID_HANDLE_VALUE, alloc, 0, bufsize, MEM_REPLACE_PLACEHOLDER, PAGE_READWRITE, NULL, 0);
	if (buf == NULL)
	{
		LogSystemError(S_ERROR);
		return RingEmpty();
	}

	if (MapViewOfFile3(map, INVALID_HANDLE_VALUE, alloc + bufsize, 0, bufsize, MEM_REPLACE_PLACEHOLDER, PAGE_READWRITE, NULL, 0) == NULL)
	{
		LogSystemError(S_ERROR);
		return RingEmpty();
	}

	CloseHandle(map);

	return (struct ring) { .mem_total = bufsize, .mem_left = bufsize, .offset = 0, .buf = buf };
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

struct memSlot RingPushStart(struct ring *ring, const u64 size)
{
	ds_AssertString(size <= ring->mem_left, "ring allocator OOM");

	struct memSlot buf = { 0 };
	if (size <= ring->mem_left)
	{
		ring->mem_left -= size;
		buf.address = ring->buf + ((ring->offset + ring->mem_left) % ring->mem_total);
		buf.size = size;
	}

	return buf;
}

struct memSlot RingPushEnd(struct ring *ring, const u64 size)
{
	ds_AssertString(size <= ring->mem_left, "ring allocator OOM");

	struct memSlot buf = { 0 };
	if (size <= ring->mem_left)
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

/* internal allocation of pool, use PoolAlloc macro instead */
struct pool PoolAllocInternal(struct arena *mem, const u32 length, const u64 slot_size, const u64 slot_allocation_offset, const u64 slot_generation_offset, const u32 growable)
{
	ds_Assert(!growable || !mem);

	struct pool pool = { 0 };

	void *buf;
	u32 length_used = length;
	if (mem)
	{
		buf = ArenaPush(mem, slot_size * length);
	}
	else
	{
		buf = ds_Alloc(&pool.mem_slot, slot_size * length, HUGE_PAGES);
		length_used = pool.mem_slot.size / slot_size;
	}

	if (buf)
	{
		pool.slot_size = slot_size;
		pool.slot_allocation_offset = slot_allocation_offset;
		pool.slot_generation_offset = slot_generation_offset;
		pool.buf = buf;
		pool.length = length_used;
		pool.count = 0;
		pool.count_max = 0;
		pool.next_free = POOL_NULL;
		pool.growable = growable;
		PoisonAddress(pool.buf, pool.slot_size * pool.length);
	}

	return pool;
}

void PoolDealloc(struct pool *pool)
{
	if (pool->mem_slot.address)
	{
		ds_Free(&pool->mem_slot);
	}	
}

void PoolFlush(struct pool *pool)
{
	pool->count = 0;
	pool->count_max = 0;
	pool->next_free = POOL_NULL;
	PoisonAddress(pool->buf, pool->slot_size * pool->length);
}

static void PoolReallocInternal(struct pool *pool)
{
	const u32 length_max = (U32_MAX >> 1);
	if (pool->length == length_max)
	{
		LogString(T_SYSTEM, S_FATAL, "pool allocator full, exiting");
		FatalCleanupAndExit();
	}
	
	u32 old_length = pool->length;
	pool->length <<= 1;
	if (pool->length > length_max)
	{
		pool->length = length_max;
	}

	pool->buf = ds_Realloc(&pool->mem_slot, pool->length*pool->slot_size);
	if (!pool->buf)
	{
		LogString(T_SYSTEM, S_FATAL, "pool reallocation failed, exiting");
		FatalCleanupAndExit();
	}

	UnpoisonAddress(pool->buf, pool->slot_size*old_length);
	PoisonAddress(pool->buf + old_length*pool->slot_size, (pool->length-old_length)*pool->slot_size);
}

struct slot PoolAdd(struct pool *pool)
{
	ds_Assert(pool->slot_generation_offset == U64_MAX);

	struct slot allocation = { .address = NULL, .index = POOL_NULL };

	u32* slot_state;
	if (pool->count < pool->length)
	{
		if (pool->next_free != POOL_NULL)
		{
			UnpoisonAddress(pool->buf + pool->next_free*pool->slot_size, pool->slot_allocation_offset);
			UnpoisonAddress(pool->buf + pool->next_free*pool->slot_size + pool->slot_allocation_offset + sizeof(u32), pool->slot_size - pool->slot_allocation_offset - sizeof(u32));

			allocation.address = pool->buf + pool->next_free*pool->slot_size;
			allocation.index = pool->next_free;

			slot_state = (u32 *) ((u8 *) allocation.address + pool->slot_allocation_offset);
			pool->next_free = *slot_state & POOL_INDEX_MASK;
			ds_Assert(*slot_state & POOL_ALLOCATION_MASK);
		}
		else
		{
			UnpoisonAddress(pool->buf + pool->count_max*pool->slot_size, pool->slot_size);
			allocation.address = (u8 *) pool->buf + pool->slot_size * pool->count_max;
			allocation.index = pool->count_max;
			slot_state = (u32 *) ((u8 *) allocation.address + pool->slot_allocation_offset);
			pool->count_max += 1;
		}	
		*slot_state = 0;
		pool->count += 1;
	}
	else if (pool->growable)
	{
		PoolReallocInternal(pool);
		UnpoisonAddress(pool->buf + pool->count_max*pool->slot_size, pool->slot_size);
		allocation.address = pool->buf + pool->slot_size*pool->count_max;
		allocation.index = pool->count_max;
		slot_state = (u32 *) ((u8 *) allocation.address + pool->slot_allocation_offset);
		*slot_state = 0;
		pool->count_max += 1;
		pool->count += 1;
	}

	return allocation;
}

struct slot GPoolAdd(struct pool *pool)
{
	ds_Assert(pool->slot_generation_offset != U64_MAX);

	struct slot allocation = { .address = NULL, .index = POOL_NULL };

	u32* slot_state;
	if (pool->count < pool->length)
	{
		if (pool->next_free != POOL_NULL)
		{
			UnpoisonAddress(pool->buf + pool->next_free*pool->slot_size, pool->slot_size);
			allocation.address = pool->buf + pool->next_free*pool->slot_size;
			allocation.index = pool->next_free;

			slot_state = (u32 *) ((u8 *) allocation.address + pool->slot_allocation_offset);
			pool->next_free = *slot_state & POOL_ALLOCATION_MASK;
			u32 *gen_state = (u32 *) ((u8 *) allocation.address + pool->slot_generation_offset);
			*gen_state += 1;
			ds_Assert(*slot_state & POOL_ALLOCATION_MASK);
		}
		else
		{
			UnpoisonAddress(pool->buf + pool->count_max*pool->slot_size, pool->slot_size);
			allocation.address = (u8 *) pool->buf + pool->slot_size * pool->count_max;
			allocation.index = pool->count_max;
			slot_state = (u32 *) ((u8 *) allocation.address + pool->slot_allocation_offset);
			u32 *gen_state = (u32 *) ((u8 *) allocation.address + pool->slot_generation_offset);
			*gen_state = 0;
			pool->count_max += 1;
		}	
		*slot_state = 0;
		pool->count += 1;
	}
	else if (pool->growable)
	{
		PoolReallocInternal(pool);
		UnpoisonAddress(pool->buf + pool->count_max*pool->slot_size, pool->slot_size);
		allocation.address = pool->buf + pool->slot_size*pool->count_max;
		allocation.index = pool->count_max;
		slot_state = (u32 *) ((u8 *) allocation.address + pool->slot_allocation_offset);
		u32 *gen_state = (u32 *) ((u8 *) allocation.address + pool->slot_generation_offset);
		*slot_state = 0;
		*gen_state = 0;
		pool->count_max += 1;
		pool->count += 1;
	}

	return allocation;
}

void PoolRemove(struct pool *pool, const u32 index)
{
	ds_Assert(index < pool->length);

	u8 *address = pool->buf + index*pool->slot_size;
	u32 *slot_state = (u32 *) (address + pool->slot_allocation_offset);
	ds_Assert(*slot_state);

	*slot_state = pool->next_free;
	pool->next_free = index;
	pool->count -= 1;

	PoisonAddress(pool->buf + index*pool->slot_size, pool->slot_allocation_offset);
	PoisonAddress(pool->buf + index*pool->slot_size + pool->slot_allocation_offset + sizeof(u32), pool->slot_size - pool->slot_allocation_offset - sizeof(u32));
}

void PoolRemoveAddress(struct pool *pool, void *slot)
{
	const u32 index = PoolIndex(pool, slot);
	PoolRemove(pool, index);
}

void *PoolAddress(const struct pool *pool, const u32 index)
{
	ds_Assert(index <= pool->count_max);
	return (u8 *) pool->buf + index*pool->slot_size;
}

u32 PoolIndex(const struct pool *pool, const void *slot)
{
	ds_Assert((u64) slot >= (u64) pool->buf);
	ds_Assert((u64) slot < (u64) pool->buf + pool->length*pool->slot_size);
	ds_Assert(((u64) slot - (u64) pool->buf) % pool->slot_size == 0); 
	return (u32) (((u64) slot - (u64) pool->buf) / pool->slot_size);
}

struct poolExternal_slot
{
	POOL_SLOT_STATE;
};

struct poolExternal PoolExternalAlloc(void **external_buf, const u32 length, const u64 slot_size, const u32 growable)
{
	ds_StaticAssert(sizeof(struct poolExternal_slot) == 4, "Expect size of poolExternal_slot is 4");

	*external_buf = NULL;
	struct poolExternal ext = { 0 };

	struct pool pool = PoolAlloc(NULL, length, struct poolExternal_slot, growable);
	if (pool.length)
	{
		*external_buf = ds_Alloc(&ext.mem_external, pool.length*slot_size, HUGE_PAGES);
		if (*external_buf)
		{
			ext.slot_size = slot_size;
			ext.external_buf = external_buf;
			ext.pool = pool;
			PoisonAddress(*external_buf, ext.slot_size * ext.pool.length);
		}
		else
		{
			PoolDealloc(&pool);
		}
	}

	return ext;
}

void PoolExternalDealloc(struct poolExternal *pool)
{
	PoolDealloc(&pool->pool);
	ds_Free(&pool->mem_external);
}

void PoolExternalFlush(struct poolExternal *pool)
{
	PoolFlush(&pool->pool);
	PoisonAddress(*pool->external_buf, pool->slot_size * pool->pool.length);
}

struct slot PoolExternalAdd(struct poolExternal *pool)
{
	const u32 old_length = pool->pool.length;
	struct slot slot = PoolAdd(&pool->pool);

	if (slot.index != POOL_NULL)
	{
		if (old_length != pool->pool.length)
		{
			*pool->external_buf = ds_Realloc(&pool->mem_external, pool->pool.slot_size*pool->pool.length);
			if (*pool->external_buf == NULL)
			{
				LogString(T_SYSTEM, S_FATAL, "Failed to reallocate external pool buffer");
				FatalCleanupAndExit();
			}
			UnpoisonAddress(*pool->external_buf, pool->slot_size*old_length);
			PoisonAddress(((u8 *)(*pool->external_buf) + pool->slot_size*old_length), pool->slot_size*(pool->pool.length - old_length)); 
		}
			
		UnpoisonAddress((u8*) *pool->external_buf + pool->slot_size*old_length, pool->slot_size);
	}

	return slot;
}

void PoolExternalRemove(struct poolExternal *pool, const u32 index)
{
	PoolRemove(&pool->pool, index);
	PoisonAddress(((u8*)*pool->external_buf) + index*pool->slot_size, pool->slot_size);
}

void PoolExternalRemoveAddress(struct poolExternal *pool, void *slot)
{
	const u32 index = PoolIndex(&pool->pool, slot);
	PoolRemove(&pool->pool, index);
	PoisonAddress(((u8*)*pool->external_buf) + index*pool->slot_size, pool->slot_size);
}

void *PoolExternalAddress(const struct poolExternal *pool, const u32 index)
{
	ds_Assert(index <= pool->pool.count_max);
	return ((u8*)*pool->external_buf) + index*pool->slot_size;
}

u32 PoolExternalIndex(const struct poolExternal *pool, const void *slot)
{
	ds_Assert((u64) slot >= (u64) (*pool->external_buf));
	ds_Assert((u64) slot < (u64) *pool->external_buf + pool->pool.length*pool->slot_size);
	ds_Assert(((u64) slot - (u64) *pool->external_buf) % pool->slot_size == 0); 
	return (u32) (((u64) slot - (u64) *pool->external_buf) / pool->slot_size);
}
