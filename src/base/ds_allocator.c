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


#include "ds_arena.c"
#include "ds_ring.c"
#include "ds_compact_pool.c"
#include "ds_pool.c"
#include "ds_slab.c"
