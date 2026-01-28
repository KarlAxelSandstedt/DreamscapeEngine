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
u32	PowerOfTwoCheck(const u64 n);
/* Return smallest value 2^k >= n where k >= 0 */
u64 	PowerOfTwoCeil(const u64 n);
/* Return smallest possible allocation size for ds_Alloc that size fits in */
u64 	ds_AllocSizeCeil(const u64 size);

/*
memSlot: ds_Alloc return value containing the required information for any sequent ds_Realloc or ds_Free call.
 */
struct memSlot
{
	void *	address;	/* base memory address */
	u64	size;		/* memory size (>= requested size)  */
	u32	huge_pages;	/* huge memory pages were requested (Up to the kernel to decide) */
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
	struct memSlot			mem_slot;
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
void *	ds_Alloc(struct memSlot *slot, const u64 size, const u32 huge_pages);
/* 
 * Reallocates the ds_Alloc memSlot, advising the kernel to use the same page policy for the new allocation. 
 * On failure, the application fatally cleans up and exit. 
 */
void *	ds_Realloc(struct memSlot *slot, const u64 size);
/*
 * Free a ds_Alloc memSlot. 
 */
void	ds_Free(struct memSlot *slot);


/*
Arena allocator
===============
Arena: contiguous memory allocator used for stack-like memory management. 
*/

/*
 * memArray: ArenaPushAlignedAll return value. To pop all memory acquired in the allocation, call
 *	
 *	ArenaPopPacked(arena, ret.mem_pushed).
 *
 * If you wish to keep N elements in the array, then call
 *
 *	ArenaPopPacked(arena, sizeof(element_type) * (ret.len - N)).
 */
struct memArray
{
	void *	addr;
	u64	len;		/* */
	u64	mem_pushed;	/* NOTE: recorded pushed number of bytes to be used in ArenaPopPacked(*) */
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
	struct memSlot		slot;
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
struct memSlot 	RingPushStart(struct ring *ring, const u64 size);
/* return allocaction[size], and advance the ring write offset on success; empty buffer on FAILURE. */
struct memSlot 	RingPushEnd(struct ring *ring, const u64 size);
/* release bytes in ring in fifo order. */
void 		RingPopStart(struct ring *ring, const u64 size); 
/* release bytes in ring in lifo order. */
void 		RingPopEnd(struct ring *ring, const u64 size); 

/*
Pool Allocator
==============
Intrusive pool allocator that handles allocation and deallocation of a specific struct. In order to use the
pool allocator for a specific struct, the struct should contain the POOL_SLOT_STATE macro; it defines
internal slot state for the struct. The pool allocator can allocate at most 2^31 - 1 slots. u32 generation
slots are supported by instead using the GPool Api instead, and replacing POOL_SLOT_STATE with 
GENERATIONAL_POOL_SLOT_STATE.

Internal: each struct contains a slot state variable (u32). For allocated slots the state is 0x80000000.
For unallocated slots, the variable represents an index < 0x7fffffff to the next free slot in the chain.
The end of the free chain is represented by POOL_NULL.
*/

#define POOL_NULL		0x7fffffff
#define POOL_SLOT_STATE 	u32 slot_allocation_state
#define PoolSlotAllocated(ptr)	((ptr)->slot_allocation_state & 0x80000000)
#define PoolSlotNext(ptr)	((ptr)->slot_allocation_state & 0x7fffffff)
#define PoolSlotGeneration(ptr)	((ptr)->slot_generation_state)

#define GENERATIONAL_POOL_SLOT_STATE	u32 slot_allocation_state;	\
					u32 slot_generation_state

struct pool
{
	struct memSlot mem_slot;	/* If heap allocated, mem_slot.address set to  
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

/* internal allocation of pool, use PoolAlloc macro instead */
struct pool 	PoolAllocInternal(struct arena *mem, const u32 length, const u64 slot_size, const u64 slot_allocation_offset, const u64 slot_generation_offset, const u32 growable);
/* Allocation of pool. On error, an empty pool (length == 0), is returned.  */
#define 	PoolAlloc(mem, length, STRUCT, growable)	PoolAllocInternal(mem, length, sizeof(STRUCT), ((u64)&((STRUCT *)0)->slot_allocation_state), U64_MAX, growable)
/* dealloc pool */
void		PoolDealloc(struct pool *pool);
/* dealloc all slot allocations */
void		PoolFlush(struct pool *pool);
/* alloc new slot; on error return (NULL, U32_MAX) */
struct slot	PoolAdd(struct pool *pool);
/* remove slot given index */
void		PoolRemove(struct pool *pool, const u32 index);
/* remove slot given address */
void		PoolRemoveAddress(struct pool *pool, void *slot);
/* return address of index */
void *		PoolAddress(const struct pool *pool, const u32 index);
/* return index of address */
u32		PoolIndex(const struct pool *pool, const void *slot);

#define GPoolAlloc(mem, length, STRUCT, growable)	PoolAllocInternal(mem, length, sizeof(STRUCT), ((u64)&((STRUCT *)0)->slot_allocation_state), ((u64)&((STRUCT *)0)->slot_generation_state), growable)
#define GPoolDealloc(PoolAddr)				PoolDealloc(PoolAddr)
#define	GPoolFlush(PoolAddr)				PoolFlush(PoolAddr)
/* Alloc new generational slot. On error return (NULL, U32_MAX) */
struct slot	GPoolAddGenerational(struct pool *pool);
#define GPoolAdd(pol_addr)				GPoolAddGenerational(pol_addr)
#define	GPoolRemove(PoolAddr, index)			PoolRemove(PoolAddr, index)
#define	GPoolRemove_address(PoolAddr, addr)		PoolRemoveAddress(PoolAddr, addr)
#define GPoolAddress(PoolAddr, index)			PoolAddress(PoolAddr, index)
#define GPoolIndex(PoolAddr, addr)			PoolIndex(PoolAddr, addr)


/*
Pool External Allocator 
=======================
An extension of the pool allocator to handle an outside buffer instead of an internal one; 
It can be useful for cases when we want to pool C types such as f32, u32.
*/

struct poolExternal
{
	u64		slot_size;
	void **		external_buf;
	struct pool	pool;
	struct memSlot	mem_external;
};

/* Allocation of pool. On error, an empty pool (length == 0), is returned.  */
struct poolExternal 	PoolExternalAlloc(void **external_buf, const u32 length, const u64 slot_size, const u32 growable);
/* dealloc poolExternal */
void			PoolExternalDealloc(struct poolExternal *pool);
/* dealloc all slot allocations */
void			PoolExternalFlush(struct poolExternal *pool);
/* alloc new slot; on error return (NULL, U32_MAX) */
struct slot		PoolExternalAdd(struct poolExternal *pool);
/* remove slot given index */
void			PoolExternalRemove(struct poolExternal *pool, const u32 index);
/* remove slot given address */
void			PoolExternalRemoveAddress(struct poolExternal *pool, void *slot);
/* return address of index */
void *			PoolExternalAddress(const struct poolExternal *pool, const u32 index);
/* return index of address */
u32			PoolExternalIndex(const struct poolExternal *pool, const void *slot);

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
