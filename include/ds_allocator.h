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
ds_allocator.h
==============
Memory allocation for the engine: heap blocks, arenas, a mirrored ring buffer, and three pools for
objects of one type. Which allocator for which lifetime: AGENTS.md, "Memory and Lifetime Handling".

Contents (in this order):
    Poisoning   PoisonAddress/UnpoisonAddress: ASan markers (DS_ASAN), no-ops otherwise.
    Utilities   PowerOfTwoCheck/PowerOfTwoCeil; g_mem_config (page size).
    Heap        ds_Alloc/ds_Realloc/ds_Free: aligned blocks of exactly the requested size.
    Arena       stack allocator (push, pop, records) on a heap block or inside another arena.
    Ring        ring buffer whose end wraps through mirrored virtual memory (Linux, Windows).
    CPool       ds_CPool(T): compact typed array; removal moves the last element into the hole.
    Pool        ds_Pool(T) of POOL_NODE structs: stable indices, free list, serializable.
    TPool       ds_TPool(T) of TPOOL_NODE structs: thread-safe, addresses never move; lock-free free lists.
Poisoning, Utilities and Heap are below. The others live in ds_arena.h, ds_ring.h, ds_compact_pool.h, ds_pool.h
(Pool and TPool) and ds_slab.h, included at the end in this order; they are not included on their own.
*/

#ifndef __DS_ALLOCATOR_H__
#define __DS_ALLOCATOR_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include <stddef.h>
#include <string.h>

#include "ds_define.h"
#include "ds_types.h"
#include "ds_assert.h"
#include "ds_atomic.h"
#include "ds_random.h"

/* ds_thread.h includes this header: TPool needs only this declaration */
u32 ds_ThreadSelfIndex(void);

#define DEFAULT_MEMORY_ALIGNMENT	((u64) 8)

/***************************** Address sanitizing and poisoning ***************************/

#ifdef DS_ASAN
#include "sanitizer/asan_interface.h"

#define PoisonAddress(addr, size)	ASAN_POISON_MEMORY_REGION((addr), (size))
#define UnpoisonAddress(addr, size)	ASAN_UNPOISON_MEMORY_REGION((addr), (size))

#else

#define PoisonAddress(addr, size)		
#define UnpoisonAddress(addr, size)

#endif


/*
Memory utils 
============
Memory utility tools. 
*/

/* Return 1 if n = 2*k for some k >= 0, otherwise return 0 */
u32 PowerOfTwoCheck(const u64 n);
/* Return smallest value 2^k >= n where k >= 0 */
u64	PowerOfTwoCeil(const u64 n);

/*
memConfig 
=========
Global memory configuration structure.
*/

struct memConfig
{
	u64				page_size;
};
extern struct memConfig *g_mem_config;

/* Initalize global memConfig and allocate resources */
void ds_MemApiInit(void);
/* Shutdown global memConfig resources */
void ds_MemApiShutdown(void);


/*
Heap API
--------
General purpose heap allocator.
*/

/*
Return size bytes aligned to alignment (a power of two), or NULL on failure or size == 0. The contents are
uninitialized.
*/
void *	ds_Alloc(const u64 size, const u64 alignment);
/*
Resize addr, a block of old_size bytes allocated with this alignment (NULL: allocate), to new_size bytes,
keeping the first min(old_size, new_size) bytes; the block may move and added bytes are uninitialized.
new_size == 0 frees addr and returns NULL. If growing fails, FatalCleanupAndExit; if shrinking fails,
addr is returned unchanged.
*/
void *	ds_Realloc(void *addr, const u64 old_size, const u64 new_size, const u64 alignment);
/* Free a ds_Alloc/ds_Realloc block (never with free); no-op on NULL. */
void	ds_Free(void *addr);

#include "ds_arena.h"
#include "ds_ring.h"
#include "ds_compact_pool.h"
#include "ds_pool.h"
#include "ds_slab.h"


#ifdef __cplusplus
}
#endif

#endif