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
    Utilities   PowerOfTwo* utility functions; g_mem_config (page size, VM granularity and commit chunk).
    Heap        ds_Alloc/ds_Realloc/ds_Free: aligned blocks of exactly the requested size.
    VM          struct VM: reserved address space, committed in chunks on first need (Linux: by touch, Windows: MEM_COMMIT).
    TStack      struct TStack: lock-free LIFO of node indices over a buffer of TREIBER_NODE structs.
    Arena       stack allocator (push, pop, records) on a heap block or inside another arena.
    Ring        ring buffer whose end wraps through mirrored virtual memory (Linux, Windows).
    CPool       ds_CPool(T): compact typed array; removal moves the last element into the hole.
    Pool        ds_Pool(T) of POOL_NODE structs: stable indices, free list, serializable.
    TPool       retired (#if 0 in ds_pool.h): rebuilt on magazines and TStack.
Poisoning, Utilities, Heap, VM and TStack are below. The others live in ds_arena.h, ds_ring.h, ds_compact_pool.h, ds_pool.h
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

/* ds_thread.h includes this header: the thread-safe containers need only this declaration */
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

/* Return 1 if n = 2^k for some k >= 0, otherwise return 0 */
static ds_ForceInline u32 PowerOfTwoCheck(const u64 n);
/* Return smallest value 2^k >= n where k >= 0 */
static ds_ForceInline u64 PowerOfTwoCeil(const u64 n);
/* Return smallest value k*m >= n, where m is a power-of-two. WARNING: n + m must not overflow. */
static ds_ForceInline u64 PowerOfTwoMultipleCeil(const u64 n, const u64 m);
/* Return largest value k*m <= n, where m is a power-of-two. */
static ds_ForceInline u64 PowerOfTwoMultipleFloor(const u64 n, const u64 m);
/* Return 1 if n == k*m, where m is a power-of-two, and k some integer, otherwise return 0. */
static ds_ForceInline u32 PowerOfTwoMultipleCheck(const u64 n, const u64 m);

/*
memConfig 
=========
Global memory configuration structure.
*/

struct memConfig
{
	u64				page_size;
	u64				vm_granularity;		/* virutal memory reserve round-up size */
	u64				vm_commit_chunk;	/* virtual memory chuck commit size     */
};
extern struct memConfig *g_mem_config;

/* Init memory API */ 
void ds_MemApiInit(const u64 vm_commit_chunk_size);
/* Shutdown memory API */
void ds_MemApiShutdown(void);


/*
Heap API
========
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


/*
Virtual Memory API
==================
Virtual memory API for reserving and committing physical pages in increasing address order.
*/

struct ds_Align(DS_CACHE_LINE) VM
{
    u8 *    buf;        /* base                     */
    u64     reserved;   /* Virtual memory reserved  */
    u8      pad0[DS_CACHE_LINE - sizeof(u8 *) - sizeof(u64)];

    u64     a_committed;/* Physical memory committed */
    u8      pad1[DS_CACHE_LINE - sizeof(u64)];
};

/* On success return non-zero struct with buf pointing to reserved pages. */
struct VM   VMReserve(const u64 size);
/* 
 * (Thread-safe) Commit pages up to end. If end <= committed, NO-OP. 
 * Return 1 on success, 0 on failure. 
 */
static u32  VMCommit(struct VM *vm, const u64 end);    
/* Interal: sys-caller */
u32         VMCommitInternal(struct VM *vm, const u64 end);    
/* Release VM resources */
void        VMRelease(struct VM *vm);

/*
TStack
======
Intrusive Lock-free index stack (Treiber). To push struct indices onto stack, add 
TREIBER_NODE to struct. 

WARNING: It it illegal to push a struct onto multiple stacks simultaneously.
*/

#define TREIBER_NODE    u32 treiber_next
#define TSTACK_NULL     U32_MAX

typedef u64 TStackKey;

/* Key of (index, tag), the index of a key, the tag of a key. */
static ds_ForceInline TStackKey TStackKeyInit(const u32 index, const u32 tag);
static ds_ForceInline u32       TStackKeyIndex(const TStackKey key);
static ds_ForceInline u32       TStackKeyTag(const TStackKey key);

struct ds_Align(DS_CACHE_LINE) TStack
{
    TStackKey   a_head;
    u8          pad[DS_CACHE_LINE - sizeof(TStackKey)];
};

/* Master: initialize empty stack. */
static ds_ForceInline void TStackFlush(struct TStack *stack)
{
    stack->a_head = TSTACK_NULL;
};

/* a popped node's treiber_next is TSTACK_NULL in debug builds: a second push of a queued node asserts */
#ifdef DS_DEBUG
#define TStackDebugMark(_buf_, _index_)     AtomicStoreRlx32(&(_buf_)[_index_].treiber_next, TSTACK_NULL)
#else
#define TStackDebugMark(_buf_, _index_)
#endif

/* Thread-safe: Push the node at index (< TSTACK_NULL, not in any stack) onto stack. */
#define TStackPush(_stack_, _buf_, _index_)                                             \
do                                                                                      \
{                                                                                       \
    ds_Assert((_index_) < TSTACK_NULL);                                                 \
    ds_Assert(AtomicLoadRlx32(&(_buf_)[_index_].treiber_next) == TSTACK_NULL);          \
                                                                                        \
    TStackKey _head_ = AtomicLoadRlx64(&(_stack_)->a_head);                             \
    TStackKey _new_head_;                                                               \
    do                                                                                  \
    {                                                                                   \
        AtomicStoreRlx32(&(_buf_)[_index_].treiber_next, TStackKeyIndex(_head_));       \
        _new_head_ = TStackKeyInit((_index_), TStackKeyTag(_head_) + 1);                \
    } while (!AtomicCompareExchangeRelRlx64(&(_stack_)->a_head, &_head_, _new_head_));  \
} while (0)

/* Thread-safe: Pop and set _index_ to the top node's index from stack on success, TSTACK_NULL on failure. */
#define TStackTryPop(_index_, _stack_, _buf_)                                           \
do                                                                                      \
{                                                                                       \
    TStackKey _head_ = AtomicLoadAcq64(&(_stack_)->a_head);                             \
    TStackKey _new_head_;                                                               \
    do                                                                                  \
    {                                                                                   \
        (_index_) = TStackKeyIndex(_head_);                                             \
        if ((_index_) == TSTACK_NULL)                                                   \
        {                                                                               \
            break;                                                                      \
        }                                                                               \
        _new_head_ = TStackKeyInit(AtomicLoadRlx32(&(_buf_)[_index_].treiber_next),     \
                                   TStackKeyTag(_head_) + 1);                           \
    } while (!AtomicCompareExchangeAcqRelAcq64(&(_stack_)->a_head, &_head_, _new_head_));\
                                                                                        \
    if ((_index_) != TSTACK_NULL)                                                       \
    {                                                                                   \
        TStackDebugMark(_buf_, _index_);                                                \
    }                                                                                   \
} while (0)

/* ========================= INLINED IMPLEMENTATIONS ========================= */

static ds_ForceInline u32 PowerOfTwoCheck(const u64 n)
{
	/* k > 0: 2^k =>   (10... - 1)  =>   (01...) = 0
	 *               & (10...    )     & (10...)
	 */

	/* k > 0: NOT 2^k =>   (1XXX10... - 1)  =>   (1XXX01...) = (1XXX00...
	 *                   & (1XXX10...    )     & (1XXX10...)
	 */

	return (n & (n-1)) == 0 && n > 0;
}

static ds_ForceInline u64 PowerOfTwoCeil(const u64 n)
{
	ds_AssertString(n <= ((u64) 1 << 63), "Overflow in PowerOfTwoCeil");
	/* 2^k >= n > 2^(k-1) <=> n-1 has bit length k */
	return (n <= 1) ? 1 : (u64) 1 << (64 - Clz64(n - 1));
}

static ds_ForceInline u64 PowerOfTwoMultipleCeil(const u64 n, const u64 m)
{
	ds_Assert(PowerOfTwoCheck(m));
	ds_AssertString(n <= U64_MAX - m, "Overflow in PowerOfTwoMultipleCeil");
	return (n + m - 1) & ~(m - 1);
}

static ds_ForceInline u64 PowerOfTwoMultipleFloor(const u64 n, const u64 m)
{
	ds_Assert(PowerOfTwoCheck(m));
	return n & ~(m - 1);
}

static ds_ForceInline u32 PowerOfTwoMultipleCheck(const u64 n, const u64 m)
{
	ds_Assert(PowerOfTwoCheck(m));
	return (n & (m - 1)) == 0;
}

static ds_ForceInline TStackKey TStackKeyInit(const u32 index, const u32 tag)
{
    return ((TStackKey) tag << 32) | index;
}

static ds_ForceInline u32 TStackKeyIndex(const TStackKey key)
{
    return (u32) key;
}

static ds_ForceInline u32 TStackKeyTag(const TStackKey key)
{
    return (u32) (key >> 32);
}

static u32 VMCommit(struct VM *vm, const u64 end)
{
    u32 success = 1;
    if (AtomicLoadAcq64(&vm->a_committed) < end)
    {
        success = VMCommitInternal(vm, end);
    }
    return success;
}

#include "ds_arena.h"
#include "ds_ring.h"
#include "ds_compact_pool.h"
#include "ds_pool.h"
#include "ds_slab.h"


#ifdef __cplusplus
}
#endif

#endif
