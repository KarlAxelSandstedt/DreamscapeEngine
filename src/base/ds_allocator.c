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
	const u64 mod = size & (g_mem_config->alloc_size_min - 1);
	return (mod) 
		? size + g_mem_config->alloc_size_min - mod
		: size;
}

void ds_MemApiShutdown(void)
{
}

void ds_MemApiInit()
{
	g_mem_config->page_size = ds_Pagesize();
	g_mem_config->alloc_size_min = g_mem_config->page_size;
	ds_Assert( PowerOfTwoCheck( g_mem_config->alloc_size_min ) );
}

void ds_SmallRealloc(void **addr, const u64 old_size, const u64 new_size)
{
	if (old_size < new_size)
	{
        void *new_addr;
        ds_SmallAlloc(&new_addr, new_size);
        if (new_addr == NULL)
        {
			LogString(T_SYSTEM, S_FATAL, "Failed to reallocate in ds_SmallRealloc, exiting.");
			FatalCleanupAndExit();
        }

        if (old_size)
        {
            memcpy(new_addr, *addr, old_size);
            ds_SmallFree(*addr);
        }
        *addr = new_addr;
    }
}

#if __DS_PLATFORM__ == __DS_LINUX__

#include <unistd.h>
#include <sys/mman.h>

void *ds_Alloc(struct ds_MemSlot *slot, const u64 size, const u32 huge_pages)
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

void *ds_Realloc(struct ds_MemSlot *slot, const u64 size)
{
	if (slot->size < size)
	{
		if (slot->huge_pages)
		{
			struct ds_MemSlot new_slot;
			if (ds_Alloc(&new_slot, size, HUGE_PAGES))
			{
				memcpy(new_slot.address, slot->address, slot->size);
			}
			ds_Free(slot);
			*slot = new_slot;
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

void ds_Free(struct ds_MemSlot *slot)
{
    if (slot->address)
    {
	    munmap(slot->address, slot->size);	
    }
	slot->address = NULL;
	slot->size = 0;
	slot->huge_pages = 0;
}

#elif __DS_PLATFORM__ == __DS_WEB__

#include <unistd.h>
#include <sys/mman.h>

void *ds_Alloc(struct ds_MemSlot *slot, const u64 size, const u32 garbage)
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

void *ds_Realloc(struct ds_MemSlot *slot, const u64 size)
{
	ds_Assert(size > slot->size);

	struct ds_MemSlot newSlot;
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

void ds_Free(struct ds_MemSlot *slot)
{
	munmap(slot->address, slot->size);	
	slot->address = NULL;
	slot->size = 0;
	slot->huge_pages = 0;
}
#elif __DS_PLATFORM__ == __DS_WIN64__

void *ds_Alloc(struct ds_MemSlot *slot, const u64 size, const u32 huge_pages)
{
	ds_Assert(size); 

	u64 size_used = ds_AllocSizeCeil(size);

	/* TODO: We skip huge pages here... */
	void *addr = VirtualAlloc(NULL, size_used, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
	if (!addr)
	{
		LogSystemError(S_ERROR);
		addr = NULL;
		size_used = 0;
	}

	slot->address = addr;
	slot->size = size_used;
	slot->huge_pages = 0;

	ds_Assert(((u64) slot->address) % g_mem_config->page_size == 0);

	return slot->address;
}

void *ds_Realloc(struct ds_MemSlot *slot, const u64 size)
{
	if (slot->size < size)
	{
		struct ds_MemSlot new_slot;
		if (ds_Alloc(&new_slot, size, NO_HUGE_PAGES))
		{
			memcpy(new_slot.address, slot->address, slot->size);
		}
		ds_Free(slot);
		*slot = new_slot;

		if (!slot->address)
		{
			LogString(T_SYSTEM, S_FATAL, "Failed to reallocate memSlot in ds_Realloc, exiting.");
			FatalCleanupAndExit();
		}
	}

	return slot->address;
}

void ds_Free(struct ds_MemSlot *slot)
{
	if (!VirtualFree(slot->address, 0, MEM_RELEASE))
	{
		LogSystemError(S_ERROR);
	}
	slot->address = NULL;
	slot->size = 0;
	slot->huge_pages = 0;
}

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

struct arena ArenaAlloc(struct arena *mem, const u64 size)
{
	struct arena ar =
	{
		.mem_size = 0,
		.mem_left = 0,
		.record = NULL,
	};

    if (mem)
    {
        ar.stack_ptr = ArenaPushAligned(mem, size, DS_CACHE_LINE);

	    if (ar.stack_ptr)
	    {
	    	ar.mem_size = size;
	    	ar.mem_left = size;
	    	PoisonAddress(ar.stack_ptr, ar.mem_left);
	    }
    }
    else
    {
	    ar.stack_ptr = (size >= 2*1024*1024)
	    	? ds_Alloc(&ar.slot, size, HUGE_PAGES)
	    	: ds_Alloc(&ar.slot, size, NO_HUGE_PAGES);

	    if (ar.stack_ptr)
	    {
	    	ar.mem_size = ar.slot.size;
	    	ar.mem_left = ar.slot.size;
	    	PoisonAddress(ar.stack_ptr, ar.mem_left);
	    }
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

struct ds_MemSlot RingPushStart(struct ring *ring, const u64 size)
{
	ds_AssertString(size <= ring->mem_left, "ring allocator OOM");

	struct ds_MemSlot buf = { 0 };
	if (size <= ring->mem_left)
	{
		ring->mem_left -= size;
		buf.address = ring->buf + ((ring->offset + ring->mem_left) % ring->mem_total);
		buf.size = size;
	}

	return buf;
}

struct ds_MemSlot RingPushEnd(struct ring *ring, const u64 size)
{
	ds_AssertString(size <= ring->mem_left, "ring allocator OOM");

	struct ds_MemSlot buf = { 0 };
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
