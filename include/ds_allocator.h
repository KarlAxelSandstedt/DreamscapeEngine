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

#ifndef __DS_ALLOCATOR_H__
#define __DS_ALLOCATOR_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_define.h"
#include "ds_types.h"

#define DEFAULT_MEMORY_ALIGNMENT	((u64) 8)

/*
Memory utils 
============
Memory utility tools. 
*/

/* Return 1 if n = 2*k for some k >= 0, otherwise return 0 */
u32 PowerOfTwoCheck(const u64 n);
/* Return smallest value 2^k >= n where k >= 0 */
u64	PowerOfTwoCeil(const u64 n);
/* Return smallest possible allocation size for ds_Alloc that size fits in */
u64	ds_AllocSizeCeil(const u64 size);

/*
memSlot: ds_Alloc return value containing the required information for any sequent ds_Realloc or ds_Free call.
 */
struct ds_MemSlot
{
	void *  address;	/* base memory address */
	u64	    size;		/* memory size (>= requested size)  */
	u32	    huge_pages;	/* huge memory pages were requested (Up to the kernel to decide) */
};


/*
Thread-Safe block allocator
===========================
Thread-safe fixed size block allocator. 
*/

struct threadBlockAllocator
{
	/* pads for 64 and 128 cachelines */
	u8				pad1[DS_CACHE_LINE_UB];
	ds_Align(DS_CACHE_LINE_UB) u64	a_next;
	u8				pad2[DS_CACHE_LINE_UB];
	u8 *				block;
	u64				block_size;
	u64				max_count;
	struct ds_MemSlot			mem_slot;
};

/* initalize allocator with at least the given block count */
void 	ThreadBlockAllocatorAlloc(struct threadBlockAllocator *allocator, const u64 block_count, const u64 block_size);
/* Release allocator resources */
void	ThreadBlockAllocatorFree(struct threadBlockAllocator *allocator);
/* Returns pointer to requested block, or NULL if  out of memory */
void *	ThreadBlockAlloc(struct threadBlockAllocator *allocator);
/* Free block */
void  	ThreadBlockFree(struct threadBlockAllocator *allocator, void *addr);

/* returns a 256B cache aligned block on success, NULL on out-of-memory */
void *	ThreadAlloc256B(void);
/* returns a 1MB cache aligned block on success, NULL on out-of-memory */
void *	ThreadAlloc1MB(void);
/* free a 256B block */
void 	ThreadFree256B(void *addr);
/* free a 1MB block */
void 	ThreadFree1MB(void *addr);


/*
memConfig 
=========
Global memory configuration structure.
*/

struct memConfig
{
	struct threadBlockAllocator	block_allocator_256B;
	struct threadBlockAllocator 	block_allocator_1MB;
	u64				page_size;
	u64				alloc_size_min;
};
extern struct memConfig *g_mem_config;

/* Initalize global memConfig and allocate resources */
void ds_MemApiInit(const u32 count_256B, const u32 count_1MB);
/* Shutdown global memConfig resources */
void ds_MemApiShutdown(void);


/*
Heap allocator
==============
Heap allocation methods. Do not use free on any memory from ds_Alloc; instead, use ds_Free, which properly 
handles memSlot allocations. Note that, unlike Linux, Windows does not support overcommiting, so the total
sum of virtual memory used in the system must be <= Memory Cap (Physical + ...).  Furthermore, Wasm does 
not support virtual memory in any form (?), so it is even more constrained.  If needed, we can get around 
this on Windows when using arenas by commiting pages manually in VirtualAlloc as they are needed.

The API allows for HUGE_PAGE requests; this should be view as advising the platform of our memory usage, 
not as a requirement the platform must adhere to. 
*/

#define HUGE_PAGES	1
#define NO_HUGE_PAGES	0

/* 
 * Return a (at least) page size aligned allocation with at least size bytes. If huge_pages is true, the 
 * kernel is advised to use huge pages in the allocation. On success, the function sets the input memSlot
 * and returns a non-NULL valid memory address. On failure, the function returns NULL, and sets 
 * slot->address = NULL * and slot->size = 0;
 */
void *	ds_Alloc(struct ds_MemSlot *slot, const u64 size, const u32 huge_pages);
/* 
 * Reallocates the ds_Alloc memSlot, advising the kernel to use the same page policy for the new allocation. 
 * On failure, the application fatally cleans up and exit. 
 */
void *	ds_Realloc(struct ds_MemSlot *slot, const u64 size);
/*
 * Free a ds_Alloc memSlot. 
 */
void	ds_Free(struct ds_MemSlot *slot);


/*
Arena allocator
===============
Arena: contiguous memory allocator used for stack-like memory management. 
*/

/*
 * memArray: ArenaPushAlignedAll return value. To pop all memory acquired in the allocation, call
 *	
 *	ArenaPopPacked(arena, ret.memPushed).
 *
 * If you wish to keep N elements in the array, then call
 *
 *	ArenaPopPacked(arena, sizeof(element_type) * (ret.len - N)).
 */
struct memArray
{
	void *	addr;
	u64	len;		/* */
	u64	memPushed;	/* NOTE: recorded pushed number of bytes to be used in ArenaPopPacked(*) */
};

/* 
 * arenaRecord is an internal structu used to record an arena's state. The pushed records create a linked 
 * list in the arena, and by popping a record, the arena pops any allocations made after the most recently 
 * pushed record. 
 */
struct arenaRecord
{
	struct arenaRecord *	prev;
	u64 			rec_mem_left;
};

struct arena 
{
	u8 * 			stack_ptr;
	u64 			mem_size;
	u64 			mem_left;
	struct arenaRecord *	record;		/* NULL == no record */
	struct ds_MemSlot		slot;
};

/* setup arena using global block allocator */
struct arena	ArenaAlloc1MB(void);
/*  global block allocator free wrapper */
void		ArenaFree1MB(struct arena *mem);

/* Record arena memory position */
void		ArenaPushRecord(struct arena *ar);
/* Return to last recorded memory position, given that recorded mem_left >= current mem_left. */
void		ArenaPopRecord(struct arena *ar);
/* Remove last recorded memory position, if any. */
void 		ArenaRemoveRecord(struct arena *ar);

/* If allocation failed, return arena = { 0 } */
struct arena	ArenaAlloc(const u64 size);
/* free heap memory and set *ar = empty_arena */
void		ArenaFree(struct arena *ar);
/* flush contents, reset stack to start of stack */
void		ArenaFlush(struct arena* ar);	

/* pop arena memory */
void 		ArenaPopPacked(struct arena *ar, const u64 mem_to_pop);

/* Return address to aligned data of given size on success, otherwise return NULL. */
void *		ArenaPushAligned(struct arena *ar, const u64 size, const u64 alignment);
/* Return address to zeroed-out aligned data of given size on success, otherwise return NULL. */
void *		ArenaPushAlignedZero(struct arena *ar, const u64 size, const u64 alignment);
/* Return address to aligned data of given size and copy [size] bytes of copy's content into it on success, 
 * otherwise return NULL. */
void *		ArenaPushAlignedMemcpy(struct arena *ar, const void *copy, const u64 size, const u64 alignment);
/* ArenaPush_alignement but we push the whole arena given a slot size, and return the number of slots aquired.  */
struct memArray	ArenaPushAlignedAll(struct arena *ar, const u64 slot_size, const u64 alignment);

#define		ArenaPushPacked(ar, size)		ArenaPushAligned(ar, size, 1)
#define		ArenaPushPackedZero(ar, size)		ArenaPushAlignedZero(ar, size, 1)
#define		ArenaPushPackedMemcpy(ar, copy, size)	ArenaPushAlignedMemcpy(ar, copy, size, 1)

#define		ArenaPush(ar, size)			ArenaPushAligned(ar, size, DEFAULT_MEMORY_ALIGNMENT)
#define		ArenaPushZero(ar, size)			ArenaPushAlignedZero(ar, size, DEFAULT_MEMORY_ALIGNMENT)
#define		ArenaPushMemcpy(ar, copy, size)		ArenaPushAlignedMemcpy(ar, copy, size, DEFAULT_MEMORY_ALIGNMENT)

/*
Ring Allocator
==============
Ring allocator based on virtual memory wrapping.
*/

struct ring
{
	u64 mem_total;
	u64 mem_left;
	u64 offset;	/* offset to write from base pointer buf */
	u8 *buf;	
};

/* return an empty ring */
struct ring 	RingEmpty(void);	
/* 
 * Allocated virtual memory wrapped ring buffer using mem_hint as a minimum memsize.
 * The final size depends on the page size of the underlying system. Returns the
 * allocated ring allocator on SUCCESS, or an empty allocator on FAILURE.
 */
struct ring 	RingAlloc(const u64 mem_hint);	
/* free ring allocator resources */
void 		RingDealloc(struct ring *ring);			
/* flush ring memory and set offset to 0 */
void		RingFlush(struct ring *ring);
/* return allocaction[size], and do not advance the ring write offset on success; empty buffer on FAILURE. */
struct ds_MemSlot 	RingPushStart(struct ring *ring, const u64 size);
/* return allocaction[size], and advance the ring write offset on success; empty buffer on FAILURE. */
struct ds_MemSlot 	RingPushEnd(struct ring *ring, const u64 size);
/* release bytes in ring in fifo order. */
void 		RingPopStart(struct ring *ring, const u64 size); 
/* release bytes in ring in lifo order. */
void 		RingPopEnd(struct ring *ring, const u64 size); 

/*
Pool Allocator
==============
Intrusive pool allocator that handles allocation and deallocation of a specific struct. In order to use the
pool allocator for a specific struct, the struct should contain the POOL_SLOT_STATE macro; it defines
internal slot state for the struct. The pool allocator can allocate at most 2^31 slots. u32 generation
slots are supported by instead using the GPool Api instead, and replacing POOL_SLOT_STATE with 
GENERATIONAL_POOL_SLOT_STATE.

Internal: each struct contains a slot state variable (u32). For allocated slots the state is <= 0x7fffffff.
For unallocated slots, the most signficicant bit is set and the 31 lower bits represents an index to the 
next free slot in the chain. The end of the free chain is represented by POOL_NULL.
*/

#define POOL_SLOT_STATE 	u32 slot_allocation_state
#define POOL_NULL		        0xffffffff
#define POOL_ALLOCATION_MASK	0x80000000
#define POOL_INDEX_MASK		    0x7fffffff
#define PoolSlotAllocated(ptr)	(!((ptr)->slot_allocation_state & POOL_ALLOCATION_MASK))
#define PoolSlotNext(ptr)	((ptr)->slot_allocation_state & POOL_INDEX_MASK)
#define PoolSlotGeneration(ptr)	((ptr)->slot_generation_state)

#define GENERATIONAL_POOL_SLOT_STATE	u32 slot_allocation_state;	\
					u32 slot_generation_state

struct ds_Pool
{
	struct ds_MemSlot mem_slot;	/* If heap allocated, mem_slot.address set to  
					   valid address, otherwise NULL.		*/
	u64	slot_size;		/* size of struct containing POOL_SLOT_STATE 	*/
	u64	slot_allocation_offset;	/* offset of pool_slot_state variable in struct	*/
	u64	slot_generation_offset; /* Optional: set if slots contain generations, 
					   else == U64_MAX 				*/
	u8 *	buf;			
	u32 	length;			/* array length 				*/
	u32 	count;			/* current count of occupied slots 		*/
	u32 	count_max;		/* max count used over the object's lifetime 	*/
	u32 	next_free;		/* next free index if != U32_MAX 		*/
	u32 	growable;		/* is the memory growable? 			*/

};

/* internal allocation of pool, use ds_PoolAlloc macro instead */
struct ds_Pool 	ds_PoolAllocInternal(struct arena *mem, const u32 length, const u64 slot_size, const u64 slot_allocation_offset, const u64 slot_generation_offset, const u32 growable);
/* Allocation of pool. On error, an empty pool (length == 0), is returned.  */
#define 	ds_PoolAlloc(mem, length, STRUCT, growable)	ds_PoolAllocInternal(mem, length, sizeof(STRUCT), ((u64)&((STRUCT *)0)->slot_allocation_state), U64_MAX, growable)
/* dealloc pool */
void		ds_PoolDealloc(struct ds_Pool *pool);
/* dealloc all slot allocations */
void		ds_PoolFlush(struct ds_Pool *pool);
/* alloc new slot; on error return (NULL, U32_MAX) */
struct slot	ds_PoolAdd(struct ds_Pool *pool);
/* remove slot given index */
void		ds_PoolRemove(struct ds_Pool *pool, const u32 index);
/* remove slot given address */
void		ds_PoolRemoveAddress(struct ds_Pool *pool, void *address);
/* return address of index */
void *		ds_PoolAddress(const struct ds_Pool *pool, const u32 index);
/* return index of address */
u32		ds_PoolIndex(const struct ds_Pool *pool, const void *address);

#define ds_GPoolAlloc(mem, length, STRUCT, growable)	ds_PoolAllocInternal(mem, length, sizeof(STRUCT), ((u64)&((STRUCT *)0)->slot_allocation_state), ((u64)&((STRUCT *)0)->slot_generation_state), growable)
#define ds_GPoolDealloc(ds_PoolAddr)				ds_PoolDealloc(ds_PoolAddr)
#define	ds_GPoolFlush(ds_PoolAddr)				ds_PoolFlush(ds_PoolAddr)
/* Alloc new generational slot. On error return (NULL, U32_MAX) */
struct slot	ds_GPoolAddGenerational(struct ds_Pool *pool);
#define ds_GPoolAdd(pol_addr)				ds_GPoolAddGenerational(pol_addr)
#define	ds_GPoolRemove(ds_PoolAddr, index)			ds_PoolRemove(ds_PoolAddr, index)
#define	ds_GPoolRemove_address(ds_PoolAddr, addr)		ds_PoolRemoveAddress(ds_PoolAddr, addr)
#define ds_GPoolAddress(ds_PoolAddr, index)			ds_PoolAddress(ds_PoolAddr, index)
#define ds_GPoolIndex(ds_PoolAddr, addr)			ds_PoolIndex(ds_PoolAddr, addr)


/*
Pool External Allocator 
=======================
An extension of the pool allocator to handle an outside buffer instead of an internal one; 
It can be useful for cases when we want to pool C types such as f32, u32.
*/

struct ds_ds_PoolExternal
{
	u64		slot_size;
	void **		external_buf;
	struct ds_Pool	pool;
	struct ds_MemSlot	mem_external;
};

/* Allocation of pool. On error, an empty pool (length == 0), is returned.  */
struct ds_ds_PoolExternal ds_PoolExternalAlloc(void **external_buf, const u32 length, const u64 slot_size, const u32 growable);
/* dealloc poolExternal */
void			ds_PoolExternalDealloc(struct ds_ds_PoolExternal *pool);
/* dealloc all slot allocations */
void			ds_PoolExternalFlush(struct ds_ds_PoolExternal *pool);
/* alloc new slot; on error return (NULL, U32_MAX) */
struct slot		ds_PoolExternalAdd(struct ds_ds_PoolExternal *pool);
/* remove slot given index */
void			ds_PoolExternalRemove(struct ds_ds_PoolExternal *pool, const u32 index);
/* remove slot given address */
void			ds_PoolExternalRemoveAddress(struct ds_ds_PoolExternal *pool, void *slot);
/* return address of index */
void *			ds_PoolExternalAddress(const struct ds_ds_PoolExternal *pool, const u32 index);
/* return index of address */
u32			    ds_PoolExternalIndex(const struct ds_ds_PoolExternal *pool, const void *slot);

/*
TPool
=====
A thread-safe pool allocator where every operation can be called by any thread, at any time;
based on Treiber stack free-lists.

::: Usage and Documentation :::

To declare/define the allocator for a specific struct, say ds_Struct, add the 
following in the definition of ds_Struct

    struct ds_Struct
    {
        ...
        TPOOL_NODE;
        ...
    };

and write

    TPOOL_DECLARE(ds_Struct)
    TPOOL_DEFINE(ds_Struct)

at the appropriate places. TPOOL_DECLARE defines the dsStructTPool struct and
the ds_StructTPool*** functions, and TPOOL_DEFINE generates the implementations
of each function. The functions generated are the following:

    // Allocation of pool.
    void        ds_StructTPoolAlloc(struct ds_StructTPool *pool, const u32 initial_count, const u64 slot_size);
    
    // Deallocation of pool (passing ptr here since pool contains sync-point)
    void        ds_StructTPoolDealloc(struct ds_StructTPool *pool);
    
    // Dealloc all slot allocations, resetting the pool's count_max to 0
    void        ds_StructTPoolFlush(struct ds_StructTPool *pool);
    
    // Alloc new slot 
    struct slot ds_StructTPoolAdd(struct ds_StructTPool *pool);
    
    // Remove slot given index 
    void		ds_StructTPoolRemove(struct ds_StructTPool *pool, const u32 index);
    
    // Return address of index 
    void *		ds_StructTPoolAddress(const struct ds_StructTPool *pool, const u32 index);

    // Allocate a previously untouched slot by increment the global TPool a_count_max
    // counter. If additional memory is needed, the thread synchronizes with other 
    // threads to allocate the needed memory.
    struct slot ds_TPoolIncrement(struct ds_TPool *pool);


::: Internals :::

By enforcing that the pool length is always a power of two, and that the length
doubles every time we increase the pool's size, we derive addresses from indices
the following way:
                                                               bN   
    1.  Initial block contain PowerOfTwo slots   length: (0...010...0)
                                                 mask:   (0...001...1)

    2.  Each block doubles the max_count, so we get the following sequence of blocksizes

             0  1  2   3   4   5  
            [I][I][2I][4I][8I][16I] ...


The address of index i is derived as follows:

    N denotes the initial length of the pool.
    bN denotes the bit index of N (starting from 0)

    mask = N-1 
    shift = CountTrailingZeroes(N) = BitLength(mask)

    // Special case, we are indexing block 0
    if (i < N)
        block = 0
        index = i

    // Common case, we are indexing block m > 0
    else
        block = 1 + ( (bit index of leading 1 in i) - bN )
              = 1 + ( (bit index of leading 1 in (i >> shift) )
              = 1 + (31 - CountLeadingZeroes(i >> shift))
              = 32 - CountLeadingZeroes(i >> shift)

        // block 1 shares the same length as block 0 => (block-1)
        block_index_mask = (bit index of leading 1 in i) - 1
                         = (N << (block-1)) - 1
        index = i & block_index_mask

    return pool->mem[ block ].slot[ index ]
*/

#define TPOOL_NODE  TSTACK_NODE(FreeList)

#define TPOOL_DECLARE(struct_name)                                                                          \
        TPOOL_STRUCT_DEFINE(struct_name);                                                                   \
        TSTACK_DECLARE(struct_name, FreeList)                                                               \
        TPOOL_ALLOC_DECLARE(struct_name);                                                                   \
        TPOOL_DEALLOC_DECLARE(struct_name);                                                                 \
        TPOOL_FLUSH_DECLARE(struct_name);                                                                   \
        TPOOL_ADD_DECLARE(struct_name);                                                                     \
        TPOOL_REMOVE_DECLARE(struct_name);                                                                  \
        TPOOL_ADDRESS_DECLARE(struct_name);                                                                 \
        TPOOL_INCREMENT_DECLARE(struct_name);                                                                

#define TPOOL_DEFINE(struct_name)                                                                           \
        TSTACK_DEFINE(struct_name, FreeList)                                                                \
        TPOOL_ALLOC_DEFINE(struct_name)                                                                     \
        TPOOL_DEALLOC_DEFINE(struct_name)                                                                   \
        TPOOL_FLUSH_DEFINE(struct_name)                                                                     \
        TPOOL_ADD_DEFINE(struct_name)                                                                       \
        TPOOL_REMOVE_DEFINE(struct_name)                                                                    \
        TPOOL_ADDRESS_DEFINE(struct_name)                                                                   \
        TPOOL_INCREMENT_DEFINE(struct_name)                                                                 

#define TPOOL_STRUCT_DEFINE(struct_name)                                                                    \
struct struct_name ## TPool                                                                                 \
{                                                                                                           \
    u32                 a_count_max;                                                                        \
    u32                 a_length;                                                                           \
    u32                 a_adding_memory;   /* Bool: Is allocating additional memory */                      \
    u32                 block_count;                                                                        \
    u32                 block_length_next;                                                                  \
                                                                                                            \
    u32                 initial_length;                                                                     \
    u32                 shift;                                                                              \
                                                                                                            \
    /* indexed by thread's unique index */                                                                  \
    u32                 steal_max_iterations;                                                               \
    u32                 free_list_count;                                                                    \
    struct struct_name ## FreeList ## TStack *t_free_list;                                                  \
    struct ds_MemSlot   t_free_list_mem;                                                                    \
                                                                                                            \
    struct struct_name *block[32];                                                                          \
    struct ds_MemSlot   mem[32];                                                                            \
}

#define TPOOL_ALLOC_DECLARE(struct_name)                                                                    \
void                struct_name ## TPoolAlloc(struct struct_name ## TPool *pool,                            \
                                              const u32 logical_core_count,                                 \
                                              const u32 initial_count)

#define TPOOL_DEALLOC_DECLARE(struct_name)                                                                  \
void                struct_name ## TPoolDealloc(struct struct_name ## TPool *pool)

#define TPOOL_FLUSH_DECLARE(struct_name)                                                                    \
void                struct_name ## TPoolFlush(struct struct_name ## TPool *pool)

#define TPOOL_ADD_DECLARE(struct_name)                                                                      \
struct slot         struct_name ## TPoolAdd(struct struct_name ## TPool *pool)

#define TPOOL_REMOVE_DECLARE(struct_name)                                                                   \
void                struct_name ## TPoolRemove(struct struct_name ## TPool *pool, const u32 index)

#define TPOOL_ADDRESS_DECLARE(struct_name)                                                                  \
struct struct_name *struct_name ## TPoolAddress(const struct struct_name ## TPool *pool, const u32 index)

#define TPOOL_INCREMENT_DECLARE(struct_name)                                                                \
struct slot         struct_name ## TPoolIncrement(struct struct_name ## TPool *pool)


#define TPOOL_ALLOC_DEFINE(struct_name)                                                                     \
TPOOL_ALLOC_DECLARE(struct_name)                                                                            \
{                                                                                                           \
    ds_Assert(initial_count <= 0x80000000);                                                                 \
    memset(pool, 0, sizeof(struct struct_name ## TPool));                                                   \
    pool->block_count = 1;                                                                                  \
    pool->block_length_next = PowerOfTwoCeil(initial_count);                                                \
    pool->initial_length = pool->block_length_next;                                                         \
    pool->shift = Ctz32(pool->initial_length);                                                              \
                                                                                                            \
    ds_StaticAssert(sizeof(pool->t_free_list[0]) % 64 == 0, "Size of TStacks should be multiple of 64");    \
    pool->free_list_count = logical_core_count;                                                             \
    pool->steal_max_iterations = 2*logical_core_count;                                                      \
    ds_Alloc(&pool->t_free_list_mem, pool->free_list_count*sizeof(pool->t_free_list[0]), NO_HUGE_PAGES);    \
    if (!pool->t_free_list_mem.address)                                                                     \
    {                                                                                                       \
			LogString(T_SYSTEM, S_FATAL, "Failed to Allocate ds_TPool free list memory, exiting.");         \
			FatalCleanupAndExit();                                                                          \
    }                                                                                                       \
                                                                                                            \
    pool->t_free_list = pool->t_free_list_mem.address;                                                      \
    for (u32 i = 0; i < logical_core_count; ++i)                                                            \
    {                                                                                                       \
        struct_name ## FreeListTStackInit(pool->t_free_list + i, pool);                                     \
        ds_Assert((u64) (pool->t_free_list + i) % 64 == 0);                                                 \
    }                                                                                                       \
                                                                                                            \
    const u64 memsize = pool->block_length_next*sizeof(pool->block[0][0]);                                  \
    (memsize < 1024*1024)                                                                                   \
        ? ds_Alloc(pool->mem + 0, memsize, NO_HUGE_PAGES)                                                   \
        : ds_Alloc(pool->mem + 0, memsize, HUGE_PAGES);                                                     \
    pool->block[0] = pool->mem[0].address;                                                                  \
    if (!pool->mem[0].address)                                                                              \
    {                                                                                                       \
			Log(T_SYSTEM, S_FATAL, "Failed to Allocate ds_TPool with initial size %lu, exiting.", memsize); \
			FatalCleanupAndExit();                                                                          \
    }                                                                                                       \
    AtomicStoreRel32(&pool->a_length, pool->block_length_next);                                             \
}
 
#define TPOOL_DEALLOC_DEFINE(struct_name)                                                                   \
TPOOL_DEALLOC_DECLARE(struct_name)                                                                          \
{                                                                                                           \
    const u32 force_read = AtomicLoadAcq32(&pool->a_adding_memory);                                         \
    ds_Assert(force_read == 0);                                                                             \
    for (u32 i = 0; i < pool->block_count; ++i)                                                             \
    {                                                                                                       \
        ds_Free(pool->mem + i);                                                                             \
    }                                                                                                       \
    ds_Free(&pool->t_free_list_mem);                                                                        \
}

#define TPOOL_FLUSH_DEFINE(struct_name)                                                                     \
TPOOL_FLUSH_DECLARE(struct_name)                                                                            \
{                                                                                                           \
    for (u32 i = 0; i < pool->free_list_count; ++i)                                                             \
    {                                                                                                       \
        struct_name ## FreeListTStackFlush(pool->t_free_list + i);                                          \
    }                                                                                                       \
    AtomicStoreRel64(&pool->a_count_max, 0);                                                                \
}

#define TPOOL_INCREMENT_DEFINE(struct_name)                                                                 \
TPOOL_INCREMENT_DECLARE(struct_name)                                                                        \
{                                                                                                           \
    struct slot slot;                                                                                       \
    slot.index = AtomicFetchAddRlx32(&pool->a_count_max, 1);                                                \
    while (slot.index >= AtomicLoadAcq32(&pool->a_length))                                                  \
    {                                                                                                       \
        /* If we succeed to swap here, we get a fully up-to-date view of TPool in                           \
         * our memory and also own the relevant parts of it.                                                \
         */                                                                                                 \
        u32 cmp_val = 0;                                                                                    \
        const u32 exch_val = 1;                                                                             \
        if (AtomicCompareExchangeAcqRlx32(&pool->a_adding_memory, &cmp_val, exch_val))                      \
        {                                                                                                   \
            if (slot.index >= pool->a_length)                                                               \
            {                                                                                               \
                ds_Assert(pool->a_length < 0x80000000);                                                     \
                const u32 new_length = 2*pool->a_length;                                                    \
                                                                                                            \
                const u64 memsize = pool->block_length_next*sizeof(pool->block[0][0]);                      \
                (memsize < 1024*1024)                                                                       \
                    ? ds_Alloc(pool->mem + pool->block_count, memsize, NO_HUGE_PAGES)                       \
                    : ds_Alloc(pool->mem + pool->block_count, memsize, HUGE_PAGES);                         \
                pool->block[pool->block_count] = pool->mem[pool->block_count].address;                      \
                                                                                                            \
                if (!pool->mem[pool->block_count].address)                                                  \
                {                                                                                           \
	            		Log(T_SYSTEM, S_FATAL, "Failed to Allocate ds_TPool block with size %lu, exiting.", \
                                memsize);                                                                   \
	            		FatalCleanupAndExit();                                                              \
                }                                                                                           \
                                                                                                            \
                pool->block_count += 1;                                                                     \
                pool->block_length_next *= 2;                                                               \
                AtomicStoreRel32(&pool->a_length, new_length);                                              \
            }                                                                                               \
            AtomicStoreRel32(&pool->a_adding_memory, cmp_val);                                              \
        }                                                                                                   \
    }                                                                                                       \
                                                                                                            \
    slot.address = struct_name ## TPoolAddress(pool, slot.index);                                           \
    return slot;                                                                                            \
}

#define TPOOL_ADD_DEFINE(struct_name)                                                                       \
TPOOL_ADD_DECLARE(struct_name)                                                                              \
{                                                                                                           \
    struct slot slot = struct_name ## FreeListTStackPop(pool->t_free_list + ds_ThreadSelfIndex());          \
    if (slot.address)                                                                                       \
    {                                                                                                       \
        return slot;                                                                                        \
    }                                                                                                       \
                                                                                                            \
    for (u32 i = 0; i < pool->steal_max_iterations; ++i)                                                    \
    {                                                                                                       \
        const u32 t = RngU64Range(0, pool->free_list_count-1);                                              \
        slot = struct_name ## FreeListTStackPop(pool->t_free_list + t);                                     \
        if (slot.address)                                                                                   \
        {                                                                                                   \
            return slot;                                                                                    \
        }                                                                                                   \
    }                                                                                                       \
                                                                                                            \
    return struct_name ## TPoolIncrement(pool);                                                             \
}

#define TPOOL_REMOVE_DEFINE(struct_name)                                                                    \
TPOOL_REMOVE_DECLARE(struct_name)                                                                           \
{                                                                                                           \
    struct_name ## FreeListTStackPush(pool->t_free_list + ds_ThreadSelfIndex(), index);                     \
}

#define TPOOL_ADDRESS_DEFINE(struct_name)                                                                   \
TPOOL_ADDRESS_DECLARE(struct_name)                                                                          \
{                                                                                                           \
    u32 bi;                                                                                                 \
    u32 i;                                                                                                  \
    if (index < pool->initial_length)                                                                       \
    {                                                                                                       \
        bi = 0;                                                                                             \
        i = index;                                                                                          \
    }                                                                                                       \
    else                                                                                                    \
    {                                                                                                       \
        bi = 32 - Clz32(index >> pool->shift);                                                              \
        const u32 mask = (pool->initial_length << (bi-1)) - 1;                                              \
        i = index & mask;                                                                                   \
    }                                                                                                       \
                                                                                                            \
    return pool->block[bi] + i;                                                                             \
}                                                           

/*
TStack
======
An intrusive parallel Treiber Stack implementation built on top of TPools.

::: Usage and Documentation :::

To declare/define the Treiber Stack functions for a stack node struct (say ds_Struct),
name and add the necessary stack node state in the struct:

    struct ds_Struct
    {
        ...
        TSTACK_NODE(FreeList);
        ...
    };

and write:

    TSTACK_DECLARE(ds_Struct, FreeList)
    TSTACK_DEFINE(ds_Struct, FreeList)

at the appropriate places. This Declares and Generates the following functions
    
    // Init a TStack
    void ds_StructFreeListTStackInit(struct ds_StructFreeListTStack *stack, struct ds_StructTPool *pool)

    // Flush a TStack
    void ds_StructFreeListTStackFlush(ds_StructFreeListTStack *stack)

    // Push a TStack node
    void ds_StructFreeListTStackPush(ds_StructFreeListTStack *stack, const u32 index)
    
    // Pop a TStack node. On failure, return empty slot { .address = NULL, .index = U32_MAX }
    struct slot ds_StructFreeListTStackPop(struct ds_StructFreeListTStack *stack)
*/

/*
 * Name ## Tail: "previous" or "lower" node  in stack, (or TSTACK_NULL)
 * Name ## Tag: incremented generational value of node to remove ABA problems
 */
#define TSTACK_NODE(name)                                                                                       \
    u64 name ## Tail;                                                                                           \
    u32 name ## Tag

#define TSTACK_NULL U64_MAX

#define TSTACK_DECLARE(struct_name, name)                                                                       \
        TSTACK_STRUCT_DEFINE(struct_name, name);                                                                \
        TSTACK_INIT_DECLARE(struct_name, name);                                                                 \
        TSTACK_FLUSH_DECLARE(struct_name, name);                                                                \
        TSTACK_PUSH_DECLARE(struct_name, name);                                                                 \
        TSTACK_POP_DECLARE(struct_name, name);                                                                  \

#define TSTACK_DEFINE(struct_name, name)                                                                        \
        TSTACK_INIT_DEFINE(struct_name, name)                                                                   \
        TSTACK_FLUSH_DEFINE(struct_name, name)                                                                  \
        TSTACK_PUSH_DEFINE(struct_name, name)                                                                   \
        TSTACK_POP_DEFINE(struct_name, name)                                                                    \


#define TSTACK_STRUCT_DEFINE(struct_name, name)                                                                 \
struct struct_name ## name ## TStack                                                                            \
{                                                                                                               \
    struct struct_name ## TPool *   pool;                                                                       \
    u64                             a_head;                                                                     \
    u8                              pad[64 - sizeof(u64) - sizeof(void *)];                                     \
}

#define TSTACK_INIT_DECLARE(struct_name, name)                                                                  \
void        struct_name ## name ## TStackInit(struct struct_name ## name ## TStack *stack,                      \
                                                         struct struct_name ## TPool *pool)

#define TSTACK_FLUSH_DECLARE(struct_name, name)                                                                 \
void        struct_name ## name ## TStackFlush(struct struct_name ## name ## TStack *stack)

#define TSTACK_PUSH_DECLARE(struct_name, name)                                                                  \
void        struct_name ## name ## TStackPush(struct struct_name ## name ## TStack *stack, const u32 index)

#define TSTACK_POP_DECLARE(struct_name, name)                                                                   \
struct slot struct_name ## name ## TStackPop(struct struct_name ## name ## TStack *stack)

#define TSTACK_INIT_DEFINE(struct_name, name)                                                                   \
TSTACK_INIT_DECLARE(struct_name, name)                                                                          \
{                                                                                                               \
    stack->pool = pool;                                                                                         \
    AtomicStoreRel64(&stack->a_head, TSTACK_NULL);                                                              \
}

#define TSTACK_FLUSH_DEFINE(struct_name, name)                                                                  \
TSTACK_FLUSH_DECLARE(struct_name, name)                                                                         \
{                                                                                                               \
    AtomicStoreRel64(&stack->a_head, TSTACK_NULL);                                                              \
}

#define TSTACK_PUSH_DEFINE(struct_name, name)                                                                   \
TSTACK_PUSH_DECLARE(struct_name, name)                                                                          \
{                                                                                                               \
    struct struct_name *node = struct_name ## TPoolAddress(stack->pool, index);                                 \
    node->name ## Tag += 1;                                                                                     \
    const u64 new_head = ((u64) node->name ## Tag << 32) + index;                                               \
    node->name ## Tail = AtomicLoadRlx64(&stack->a_head);                                                       \
    while (!AtomicCompareExchangeRelRlx64(&stack->a_head, &node->name ## Tail, new_head));                      \
}

#define TSTACK_POP_DEFINE(struct_name, name)                                                                    \
TSTACK_POP_DECLARE(struct_name, name)                                                                           \
{                                                                                                               \
    struct struct_name *node;                                                                                   \
    u64 local_head = AtomicLoadAcq64(&stack->a_head);                                                           \
    do                                                                                                          \
    {                                                                                                           \
        if (local_head == TSTACK_NULL)                                                                          \
        {                                                                                                       \
            return (struct slot) { .address = NULL, .index = U32_MAX };                                         \
        }                                                                                                       \
        node = struct_name ## TPoolAddress(stack->pool, (u32) local_head);                                      \
    } while (!AtomicCompareExchangeAcqRelAcq64(&stack->a_head, &local_head, node->name ## Tail));               \
                                                                                                                \
    return (struct slot) { .address = node, .index = (u32) local_head };                                        \
}

/***************************** Address sanitizing and poisoning ***************************/

#ifdef DS_ASAN
#include "sanitizer/asan_interface.h"

#define PoisonAddress(addr, size)	ASAN_POISON_MEMORY_REGION((addr), (size))
#define UnpoisonAddress(addr, size)	ASAN_UNPOISON_MEMORY_REGION((addr), (size))

#else

#define PoisonAddress(addr, size)		
#define UnpoisonAddress(addr, size)

#endif


#ifdef __cplusplus
}
#endif

#endif
