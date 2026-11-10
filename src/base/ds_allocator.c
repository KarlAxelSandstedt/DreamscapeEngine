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

#if __DS_PLATFORM__ == __DS_LINUX__

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

void ds_MemApiShutdown(void)
{
}

void ds_MemApiInit()
{
	g_mem_config->page_size = ds_Pagesize();
	ds_Assert( PowerOfTwoCheck( g_mem_config->page_size ) );
#if __DS_PLATFORM__ == __DS_LINUX__
	g_mem_config->vm_granularity = g_mem_config->page_size;
#elif __DS_PLATFORM__ == __DS_WIN64__
	SYSTEM_INFO info;
	GetSystemInfo(&info);
	g_mem_config->vm_granularity = info.dwAllocationGranularity;
#endif
	ds_Assert( PowerOfTwoCheck( g_mem_config->vm_granularity ) );
}

/* debug builds fill new memory: nothing may rely on it being zeroed */
#ifdef DS_DEBUG
	#define DS_ALLOC_DEBUG_FILL(addr, size)	memset((addr), 0xa5, (size))
#else
	#define DS_ALLOC_DEBUG_FILL(addr, size)
#endif

#if __DS_PLATFORM__ == __DS_LINUX__

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
#include <windows.h>

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

struct VM VMReserve(const u64 size)
{
    if (!size)
    {
        return (struct VM) { 0 };
    }

    const u64 granularity = g_mem_config->vm_granularity;
    const u64 reserved = PowerOfTwoMultipleCeil(size, granularity);

#if __DS_PLATFORM__ == __DS_LINUX__

    void *buf = mmap(NULL, reserved, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    if (buf == MAP_FAILED)
    {
        LogSystemError(S_ERROR);
        return (struct VM) { 0 };
    }

#elif __DS_PLATFORM__ == __DS_WIN64__

    void *buf = VirtualAlloc(NULL, reserved, MEM_RESERVE, PAGE_NOACCESS);
    if (buf == NULL)
    {
        LogSystemError(S_ERROR);
        return (struct VM) { 0 };
    }

#endif
    return (struct VM) { .buf = buf, .reserved = reserved, .a_committed = 0 };
}

/* VMCommitInternal commits whole slices, so the committed end moves in large steps (a multiple of vm_granularity) */
#define VM_COMMIT_SLICE		((u64) 1 << 20)

u32 VMCommitInternal(struct VM *vm, const u64 end)
{
    // TODO Must handle 
    ds_Assert(vm->buf && end <= vm->reserved);
    u64 target = PowerOfTwoMultipleCeil(end, VM_COMMIT_SLICE);
    if (target > vm->reserved)
    {
        target = vm->reserved;
    }

    /* threads that cross the end together commit overlapping slices, which the system accepts; the largest end wins */
    u64 committed = AtomicLoadAcq64(&vm->a_committed);
    while (committed < target)
    {
#if __DS_PLATFORM__ == __DS_WIN64__
        if (VirtualAlloc(vm->buf + committed, target - committed, MEM_COMMIT, PAGE_READWRITE) == NULL)
        {
            LogSystemError(S_ERROR);
            return 0;
        }
#endif
        if (AtomicCompareExchangeAcqRelAcq64(&vm->a_committed, &committed, target))
        {
            break;
        }
    }
    return 1;
}

void VMRelease(struct VM *vm)
{
    if (vm->buf)
    {
#if __DS_PLATFORM__ == __DS_LINUX__
        if (munmap(vm->buf, vm->reserved) != 0)
        {
            LogSystemError(S_ERROR);
        }
#elif __DS_PLATFORM__ == __DS_WIN64__
        if (!VirtualFree(vm->buf, 0, MEM_RELEASE))
        {
            LogSystemError(S_ERROR);
        }
#endif
    }
    *vm = (struct VM) { 0 };
}

#include "ds_arena.c"
#include "ds_ring.c"
#include "ds_compact_pool.c"
#include "ds_pool.c"
#include "ds_slab.c"
