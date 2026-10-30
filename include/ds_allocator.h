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
The pools' inline definitions follow at the bottom, in the same order.
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


/*
Arena API
---------
Stack allocator: pushes move a pointer forward, pops release the most recent pushes (LIFO).
*/

/*
ArenaPushAlignedAll result. Pop all of it with ArenaPopPacked(ar, array.memPushed); to keep the first N
slots, pop slot_size * (array.len - N).
*/
struct memArray
{
	void *	addr;		/* NULL if not even one slot fit */
	u64	len;		/* slots pushed */
	u64	memPushed;	/* bytes pushed, alignment padding included */
};

/* Internal: a record lives in its arena; records form a list from the latest to the oldest. */
struct arenaRecord
{
	struct arenaRecord *	prev;
	u64 			rec_mem_left;
};

struct arena 
{
	u8 * 			stack_ptr;	/* next free byte */
	u64 			mem_size;
	u64 			mem_left;
	struct arenaRecord *	record;		/* latest record, NULL == no record */
	u32			heap_allocated;	/* memory from ds_Alloc (else from a parent arena) */
};

/*
Return an arena of size bytes, cache-line aligned: a ds_Alloc block if mem == NULL, else pushed onto mem
(it is released when mem pops it, not by ArenaFree). On failure, the arena is { 0 }.
*/
struct arena ArenaAlloc(struct arena *mem, const u64 size);
/* Free a heap arena's block (no-op for an arena inside another) and leave the arena empty, size 0. */
void		ArenaFree(struct arena *ar);
/* Pop everything and drop all records. */
void		ArenaFlush(struct arena* ar);	

/* Record the current position. Fatal (FatalCleanupAndExit) if the arena has no room for the record. */
void		ArenaPushRecord(struct arena *ar);
/* Pop back to the latest record and remove it; no-op without records. */
void		ArenaPopRecord(struct arena *ar);
/* Remove the latest record without popping (the memory pushed since stays); no-op without records. */
void 		ArenaRemoveRecord(struct arena *ar);

/* Release the last mem_to_pop bytes pushed (alignment padding included); at most the bytes in use. */
void 		ArenaPopPacked(struct arena *ar, const u64 mem_to_pop);

/*
Return size bytes aligned to alignment (a power of two), or NULL (arena unchanged) if size == 0 or they
don't fit.
*/
void *		ArenaPushAligned(struct arena *ar, const u64 size, const u64 alignment);
/* ArenaPushAligned, with the bytes set to 0. */
void *		ArenaPushAlignedZero(struct arena *ar, const u64 size, const u64 alignment);
/* ArenaPushAligned, with size bytes copied from copy. */
void *		ArenaPushAlignedMemcpy(struct arena *ar, const void *copy, const u64 size, const u64 alignment);
/* Push every whole slot_size slot that fits after the alignment padding (see struct memArray). */
struct memArray	ArenaPushAlignedAll(struct arena *ar, const u64 slot_size, const u64 alignment);

/* Packed: no alignment. Default: DEFAULT_MEMORY_ALIGNMENT (8) bytes. */
#define		ArenaPushPacked(ar, size)		ArenaPushAligned(ar, size, 1)
#define		ArenaPushPackedZero(ar, size)		ArenaPushAlignedZero(ar, size, 1)
#define		ArenaPushPackedMemcpy(ar, copy, size)	ArenaPushAlignedMemcpy(ar, copy, size, 1)

#define		ArenaPush(ar, size)			ArenaPushAligned(ar, size, DEFAULT_MEMORY_ALIGNMENT)
#define		ArenaPushZero(ar, size)			ArenaPushAlignedZero(ar, size, DEFAULT_MEMORY_ALIGNMENT)
#define		ArenaPushMemcpy(ar, copy, size)		ArenaPushAlignedMemcpy(ar, copy, size, DEFAULT_MEMORY_ALIGNMENT)

/*
Ring API
--------
Ring buffer mapped twice back to back in virtual memory, so an allocation that runs past the end continues
at the start and is still contiguous. Web (no virtual memory): buf is a plain 2*mem_total block; every function
behaves the same, but the halves do not mirror each other, so reach a slot only through its own address.
*/

/*
The bytes in use are the mem_total - mem_left bytes ending at offset (wrapping), from the start (oldest)
to the end (newest).
*/
struct ring
{
	u64 mem_total;
	u64 mem_left;
	u64 offset;	/* end of the bytes in use: where RingPushEnd writes */
	u8 *buf;	/* buf[0, 2*mem_total): the second half mirrors the first (not on web) */
};

/* RingPushStart/RingPushEnd result: { NULL, 0 } on failure */
struct ringSlot
{
	u8 *	address;
	u64	size;
};

/* Return the empty ring (no memory). */
struct ring 	    RingEmpty(void);	
/*
Return a ring of at least mem_hint bytes (Linux, web: rounded up to whole pages; Windows: to a power of two of
at least the allocation granularity, 64 KiB), or the empty ring on failure.
*/
struct ring 	    RingAlloc(const u64 mem_hint);	
/* Unmap the ring and leave it empty. */
void 		        RingDealloc(struct ring *ring);			
/* Release all bytes and restart at offset 0. */
void		        RingFlush(struct ring *ring);
/* Push size bytes before the start (the end and offset stay); { NULL, 0 } if size == 0 or they don't fit. */
struct ringSlot 	RingPushStart(struct ring *ring, const u64 size);
/* Push size bytes at the end, advancing offset; { NULL, 0 } if size == 0 or they don't fit. */
struct ringSlot 	RingPushEnd(struct ring *ring, const u64 size);
/* Release the size oldest bytes (FIFO with RingPushEnd). */
void 		        RingPopStart(struct ring *ring, const u64 size); 
/* Release the size newest bytes, moving offset back (LIFO with RingPushEnd). */
void 		        RingPopEnd(struct ring *ring, const u64 size); 

/*
CPool API
---------
Compact pool of one type T: count elements packed in buf[0, count), length slots, so the data stays
dense and iterates as a plain array. Pushes append; removing any element but the last moves the last one
into its hole, so holders of the moved element's index must be updated. A CPool lives in an arena (fixed
length) or on the heap (fixed, or growable: the length doubles when a push finds the pool full, and buf
moves).

::: Usage and Documentation :::

ds_CPool(T) is the pool type. Defined here for voidptr, u8, u16, u32, u64, i8, i16, i32, i64, f32, f64,
v2, v3, v4 (DEFINE_CPOOL_NATIVE(T), T a type name) and intv (DEFINE_CPOOL_STRUCT(T), for struct T).

    ds_CPool(u32) pool;
    ds_CPoolAlloc(NULL, pool, 64, GROWABLE);
    ds_CPoolPushValue(pool, 7);
    for (u32 i = 0; i < pool.count; ++i) { ... pool.buf[i] ... }
    ds_CPoolRemoveAndSwap(pool, 0);
    ds_CPoolDealloc(pool);

The macros (pool is a ds_CPool(T) lvalue; arguments may be evaluated more than once):

    // Allocate length slots in mem (fixed) or on the heap (mem == NULL; growable allowed). On failure the
    // pool is empty (buf NULL, length 0). Length 0 is valid: a push then reports full (or grows).
    ds_CPoolAlloc(mem, pool, length, growable)

    // Free a heap pool's memory (arena memory stays with the arena); the pool becomes empty.
    ds_CPoolDealloc(pool)

    // Remove all elements.
    ds_CPoolFlush(pool)

    // Append an uninitialized slot and return { address, index }; full and not growable:
    // { stub address, U32_MAX }.
    struct slot     ds_CPoolPush(pool)

    // ds_CPoolPush, then copy sizeof(T) bytes from src into the slot (into the stub when full).
    struct slot     ds_CPoolPushMemcpy(pool, src)

    // ds_CPoolPush, then assign val to the slot (to the stub when full).
    void            ds_CPoolPushValue(pool, val)

    // Remove the last element, or the last pop_count elements (pop_count <= count).
    void            ds_CPoolPop(pool)
    void            ds_CPoolPopBatch(pool, pop_count)

    // Remove buf[index] (index < count): the last element moves into its slot.
    ds_CPoolRemoveAndSwap(pool, index)

    // The last element, or its address; on an empty pool, the stub.
    T               ds_CPoolTop(pool)
    T *             ds_CPoolTopAddress(pool)

    // Upper bound of the arena bytes ds_CPoolAlloc(mem, pool, length, ...) pushes for slot_size-byte slots.
    u64             ds_CPoolAllocMemoryRequirement(length, slot_size)

    // Bytes ds_CPoolSerialize writes.
    u64             ds_CPoolSerializeSize(pool)

    // Write the pool to ss (at a byte boundary; it must fit: ds_CPoolSerializeSize).
    void            ds_CPoolSerialize(ss, pool)

    // Read a pool from ss into mem (fixed) or the heap (mem == NULL; growable allowed). Returns 1 on
    // success; 0 if mem is full or ss holds no valid CPool (the pool is empty, ss's position unchanged).
    u32             ds_CPoolTryDeserialize(mem, ss, pool, growable)

::: Stub slot and serialized format :::

buf[-1] is a stub slot, allocated even for length 0, so a full pool always has an address to hand out:
pushes onto a full fixed pool write into it, harmlessly. Index -1 (DS_STUB_INDEX) is therefore
addressable: intrusive lists over pools (ds_list.h) use it as their sentinel. The allocation (stub
included) is cache-line aligned. Serialized format (LE): u32 length, u32 count, then the count elements as raw bytes.
*/

#define DS_STUB_INDEX               -1

#define ds_CPool(T) ds_CPool_ ## T

#define DEFINE_CPOOL_NATIVE(T)	    \
typedef struct ds_CPool(T)		    \
{					                \
	u32 		        length;		\
	u32 		        count;		\
	u32		            growable;	\
	u32		            heap_allocated;	\
	T *                 buf;	    \
} ds_CPool(T)
typedef void * voidptr;
DEFINE_CPOOL_NATIVE(voidptr);
DEFINE_CPOOL_NATIVE(u8);
DEFINE_CPOOL_NATIVE(u16);
DEFINE_CPOOL_NATIVE(u32);
DEFINE_CPOOL_NATIVE(u64);
DEFINE_CPOOL_NATIVE(i8);
DEFINE_CPOOL_NATIVE(i16);
DEFINE_CPOOL_NATIVE(i32);
DEFINE_CPOOL_NATIVE(i64);
DEFINE_CPOOL_NATIVE(f32);
DEFINE_CPOOL_NATIVE(f64);
DEFINE_CPOOL_NATIVE(v2);
DEFINE_CPOOL_NATIVE(v3);
DEFINE_CPOOL_NATIVE(v4);

#define DEFINE_CPOOL_STRUCT(T)	    \
typedef struct ds_CPool(T)		    \
{					                \
	u32 		        length;		\
	u32 		        count;		\
	u32		            growable;	\
	u32		            heap_allocated;	\
	struct T *          buf;	    \
} ds_CPool(T)
DEFINE_CPOOL_STRUCT(intv);

#define ds_CPoolAllocMemoryRequirement(__length, __slot_size)                           \
    (DS_CACHE_LINE + ((__length)+1)*(__slot_size))

/* Internal: ds_CPoolAlloc as a function. Returns 1 on success and 0 on failure. */
u32 ds_CPoolAllocInternal(struct arena *mem, void **buf, u32 *length, u32 *count, u32 *growable, u32 *heap_allocated, const u64 slot_size, const u32 alloc_length, const u32 alloc_growable);

#define ds_CPoolAlloc(mem, pool, __length, __growable)                                  \
do                                                                                      \
{                                                                                       \
    memset(&(pool), 0, sizeof(pool));                                                   \
    ds_CPoolAllocInternal((mem), (void **) &(pool).buf, &(pool).length, &(pool).count,  \
                          &(pool).growable, &(pool).heap_allocated, sizeof((pool).buf[0]), \
                          (__length), (__growable));                                    \
} while (0)

#define ds_CPoolDealloc(pool)                                                           \
do                                                                                      \
{                                                                                       \
    if ((pool).buf && (pool).heap_allocated)                                            \
    {                                                                                   \
        ds_Free((pool).buf-1);                                                          \
    }                                                                                   \
    (pool).buf = NULL;                                                                  \
    (pool).length = 0;                                                                  \
    (pool).count = 0;                                                                   \
} while (0)

#define ds_CPoolFlush( pool )                                                           \
do                                                                                      \
{                                                                                       \
    PoisonAddress((pool).buf, (pool).count*sizeof((pool).buf[0]));                      \
    (pool).count = 0;                                                                   \
} while (0)                                                                             

/* Internal: see ds_CPoolPush and ds_CPoolPushMemcpy. */
static inline struct slot ds_CPoolPushInternal(void **buf, u32 *count, u32 *length, u32 *heap_allocated, const u64 slot_size, const u32 growable);
static inline struct slot ds_CPoolPushMemcpyInternal(void **buf, u32 *count, u32 *length, u32 *heap_allocated, const u64 slot_size, const u32 growable, const void *src);

#define ds_CPoolPush( pool ) ds_CPoolPushInternal((void **) &(pool).buf, &(pool).count, &(pool).length, &(pool).heap_allocated, sizeof((pool).buf[0]), (pool).growable)
#define ds_CPoolPushMemcpy( pool, src ) ds_CPoolPushMemcpyInternal((void **) &(pool).buf, &(pool).count, &(pool).length, &(pool).heap_allocated, sizeof((pool).buf[0]), (pool).growable, (src))
#define ds_CPoolPushValue( pool, val )                                                      \
do                                                                                          \
{                                                                                           \
    const struct slot __slot = ds_CPoolPushInternal((void **) &(pool).buf,                  \
                                                  &(pool).count,                            \
                                                  &(pool).length,                           \
                                                  &(pool).heap_allocated,                   \
                                                  sizeof((pool).buf[0]),                    \
                                                  (pool).growable);                         \
    (pool).buf[(i32) __slot.index] = (val);                                                 \
} while (0)


#define ds_CPoolPopBatch( pool, pop_count )                                                 \
do									                                                        \
{									                                                        \
	ds_Assert((pop_count) <= (pool).count);						                            \
	PoisonAddress((pool).buf + (pool).count - (pop_count), (pop_count)*sizeof((pool).buf[0]));	\
	(pool).count -= (pop_count);						                                        \
} while (0)

#define ds_CPoolPop( pool ) ds_CPoolPopBatch(pool, 1)                                      

#define ds_CPoolRemoveAndSwap( pool, index )                                            \
do                                                                                      \
{                                                                                       \
	ds_Assert((pool).count && (index) < (pool).count);						            \
	(pool).count -= 1;						                                            \
    if ((index) != (pool).count)                                                        \
    {                                                                                   \
        memcpy((pool).buf + (index), (pool).buf + (pool).count, sizeof((pool).buf[0])); \
    }                                                                                   \
	PoisonAddress((pool).buf + (pool).count, sizeof((pool).buf[0]));	                \
} while (0)

#define ds_CPoolTop( pool ) ((pool).buf[(i32) (pool).count - 1])
#define ds_CPoolTopAddress( pool ) ((pool).buf + (i32) (pool).count - 1)

struct ss;
/* Internal: see ds_CPoolSerialize. */
void ds_CPoolSerializeInternal(struct ss *ss, const void *buf, const u32 length, const u32 count, const u64 slot_size);
/* Internal: see ds_CPoolTryDeserialize. */
u32  ds_CPoolTryDeserializeInternal(struct arena *mem, struct ss *ss, void **buf, u32 *length, u32 *count, u32 *growable, u32 *heap_allocated, const u64 slot_size, const u32 alloc_growable);

#define ds_CPoolSerializeSize( pool ) (2*sizeof(u32) + (u64) (pool).count*sizeof((pool).buf[0]))
#define ds_CPoolSerialize( ss, pool ) ds_CPoolSerializeInternal((ss), (pool).buf, (pool).length, (pool).count, sizeof((pool).buf[0]))
#define ds_CPoolTryDeserialize( mem, ss, pool, __growable )                                     \
    ds_CPoolTryDeserializeInternal((mem), (ss), (void **) &(pool).buf, &(pool).length,          \
                                   &(pool).count, &(pool).growable, &(pool).heap_allocated,     \
                                   sizeof((pool).buf[0]), (__growable))


/*
Pool API
--------
Intrusive pool of struct Type: a slot keeps its index for its whole lifetime, so indices are the handles
others hold. Adds reuse removed slots, so live slots are scattered over buf[0, count_max). A pool lives in
an arena (fixed length) or on the heap (fixed, or growable: the length roughly doubles when an add finds
the pool full, and buf moves, so hold indices, never addresses). At most 2^31 - 2 slots: growing past
that is fatal.

::: Usage and Documentation :::

struct Type contains POOL_NODE (its pool_slot field belongs to the pool). DEFINE_POOL(Type) defines the
pool type ds_Pool(Type) (once, where Type is declared); the macros below work on any pool type.

    struct node { POOL_NODE; ... };
    DEFINE_POOL(node);

    ds_Pool(node) pool;
    ds_PoolAlloc(NULL, pool, 128, GROWABLE);
    const struct slot slot = ds_PoolAdd(pool);                (slot.index is the handle)
    for (u32 i = 0; i < pool.count_max; ++i)
        if (ds_PoolSlotAllocated(pool.buf + i)) { ... }       (iterate the live slots)
    ds_PoolRemove(pool, slot.index);
    ds_PoolDealloc(pool);

The macros (pool is a ds_Pool(Type) lvalue, addr a struct Type pointer; arguments may be evaluated more
than once):

    // Allocate length slots in mem (fixed) or on the heap (mem == NULL; growable allowed). Returns 1 on
    // success; 0 on failure, with the pool empty (buf NULL, length 0).
    u32             ds_PoolAlloc(mem, pool, length, growable)

    // Free a heap pool's memory (arena memory stays with the arena); the pool becomes empty.
    void            ds_PoolDealloc(pool)

    // Remove all slots; the next adds return indices 0, 1, 2, ...
    void            ds_PoolFlush(pool)

    // Allocate a slot and return { address, index }: the most recently removed slot, else the next unused
    // one. Its contents are uninitialized (pool_slot aside). Full and not growable: { NULL, POOL_NULL }.
    struct slot     ds_PoolAdd(pool)

    // Remove an allocated slot, by index or by address.
    void            ds_PoolRemove(pool, index)
    void            ds_PoolRemoveAddress(pool, addr)

    // Return the index of a slot's address.
    u32             ds_PoolIndex(pool, addr)

    // 1 if the slot at ptr (in buf[0, count_max)) is allocated.
    u32             ds_PoolSlotAllocated(ptr)

    // Bytes ds_PoolSerialize writes.
    u64             ds_PoolSerializeSize(pool)

    // Write the pool to ss (at a byte boundary; it must fit: ds_PoolSerializeSize).
    void            ds_PoolSerialize(ss, pool)

    // Read a pool from ss into mem (fixed) or the heap (mem == NULL; growable allowed), with slot positions
    // and free-list order as written (later adds return the same indices). Returns 1 on success; 0 if mem
    // is full or ss holds no valid pool: inconsistent counts, an index out of range or repeated (the pool
    // is empty, ss's position unchanged).
    u32             ds_PoolTryDeserialize(mem, ss, pool, growable)

::: Stub slot and serialized format :::

buf[-1] is a stub slot, so index -1 (DS_STUB_INDEX) is addressable: intrusive lists over pools
(ds_list.h) use it as their sentinel. Serialized format (LE): u32 length, count, count_max; the
count_max - count free indices in free-list order; the count allocated slots as raw bytes, each with
pool_slot replaced by its index.
*/

#define POOL_NODE 	        u32 pool_slot
#define POOL_NULL		        0xffffffff
#define POOL_ALLOCATION_MASK	0x80000000
#define POOL_INDEX_MASK		    0x7fffffff

#define ds_Pool(T) ds_Pool_ ## T

#define DEFINE_POOL(T)                                                                              \
typedef struct ds_Pool(T)                                                                           \
{                                                                                                   \
	struct T *buf;		                                                                            \
	u32 length;			        /* array length 				                                */  \
	u32 count;			        /* current count of occupied slots 		                        */  \
	u32 count_max;		        /* max count used over the object's lifetime 	                */  \
	u32 next_free;		        /* next free index if != POOL_INDEX_MASK 		                */  \
	u32 growable;		        /* is the memory growable? 			                            */  \
	u32 heap_allocated;	        /* buf - 1 is a ds_Alloc block (else arena memory)              */  \
} ds_Pool(T)

#define ds_PoolSlotAllocated(ptr)	(!((ptr)->pool_slot & POOL_ALLOCATION_MASK))

/*
Internal: byte offset of pool_slot in the pool's slot type, a constant. (1 ? NULL : buf) is a null pointer of
buf's type that never reads buf (it may be uninitialized, e.g. before ds_PoolTryDeserialize).
*/
#define ds_PoolSlotOffset(pool)		((u64) &((1 ? NULL : (pool).buf)->pool_slot))

/* Internal: see ds_PoolAlloc. Returns 1 on success and 0 on failure. */
u32  ds_PoolAllocInternal(struct arena *mem, u32 *heap_allocated, void **buf, u32 *length, u32 *count, u32 *count_max, u32 *next_free, u32 *growable, const u64 slot_size, const u32 alloc_length, const u32 alloc_growable);
/* Internal: grow a full growable pool (the cold path of ds_PoolAdd). */
void ds_PoolGrowInternal(void **buf, u32 *length, const u64 slot_size);
/* Internal: see ds_PoolSerialize. */
void ds_PoolSerializeInternal(struct ss *ss, const void *buf, const u32 length, const u32 count, const u32 count_max, const u32 next_free, const u64 slot_size, const u64 pool_slot_offset);
/* Internal: see ds_PoolTryDeserialize. */
u32  ds_PoolTryDeserializeInternal(struct arena *mem, struct ss *ss, u32 *heap_allocated, void **buf, u32 *length, u32 *count, u32 *count_max, u32 *next_free, u32 *growable, const u64 slot_size, const u64 pool_slot_offset, const u32 alloc_growable);

/* Internal: see ds_PoolAdd, ds_PoolRemove, and the debug checks of ds_PoolIndex (empty in release builds). */
static ds_ForceInline struct slot ds_PoolAddInternal(void **buf, u32 *length, u32 *count, u32 *count_max, u32 *next_free, const u32 growable, const u64 slot_size, const u64 pool_slot_offset);
static ds_ForceInline void ds_PoolRemoveInternal(void *buf, u32 *count, u32 *next_free, const u32 count_max, const u32 index, const u64 slot_size, const u64 pool_slot_offset);
static ds_ForceInline void ds_PoolIndexCheck(const void *buf, const u32 length, const void *addr, const u64 slot_size);

#define ds_PoolAlloc(mem, pool, __length, __growable)                                                   \
	ds_PoolAllocInternal((mem), &(pool).heap_allocated, (void **) &(pool).buf, &(pool).length,         \
	                     &(pool).count, &(pool).count_max, &(pool).next_free, &(pool).growable,         \
	                     sizeof((pool).buf[0]), (__length), (__growable))

#define ds_PoolDealloc(pool)                                                                            \
do                                                                                                      \
{                                                                                                       \
	if ((pool).heap_allocated && (pool).buf)                                                            \
	{                                                                                                   \
		ds_Free((pool).buf - 1);                                                                        \
	}                                                                                                   \
	(pool).buf = NULL;                                                                                  \
	(pool).length = 0;                                                                                  \
	(pool).count = 0;                                                                                   \
	(pool).count_max = 0;                                                                               \
	(pool).next_free = POOL_INDEX_MASK;                                                                 \
	(pool).heap_allocated = 0;                                                                          \
} while (0)

#define ds_PoolFlush(pool)                                                                              \
do                                                                                                      \
{                                                                                                       \
	(pool).count = 0;                                                                                   \
	(pool).count_max = 0;                                                                               \
	(pool).next_free = POOL_INDEX_MASK;                                                                 \
	PoisonAddress((pool).buf, sizeof((pool).buf[0]) * (pool).length);                                  \
} while (0)

#define ds_PoolAdd(pool)                                                                                \
	ds_PoolAddInternal((void **) &(pool).buf, &(pool).length, &(pool).count, &(pool).count_max,        \
	                   &(pool).next_free, (pool).growable, sizeof((pool).buf[0]), ds_PoolSlotOffset(pool))

#define ds_PoolRemove(pool, index)                                                                      \
	ds_PoolRemoveInternal((pool).buf, &(pool).count, &(pool).next_free, (pool).count_max, (index),     \
	                      sizeof((pool).buf[0]), ds_PoolSlotOffset(pool))

/* typed pointer difference: the compiler's exact division, and a type check of addr */
#define ds_PoolIndex(pool, addr)                                                                        \
	(ds_PoolIndexCheck((pool).buf, (pool).length, (addr), sizeof((pool).buf[0])),                      \
	 (u32) ((addr) - (pool).buf))

#define ds_PoolRemoveAddress(pool, addr)	ds_PoolRemove(pool, ds_PoolIndex(pool, addr))

#define ds_PoolSerializeSize(pool)                                                                      \
	(3*sizeof(u32) + (u64) ((pool).count_max - (pool).count)*sizeof(u32)                               \
	               + (u64) (pool).count*sizeof((pool).buf[0]))

#define ds_PoolSerialize(ss, pool)                                                                      \
	ds_PoolSerializeInternal((ss), (pool).buf, (pool).length, (pool).count, (pool).count_max,          \
	                         (pool).next_free, sizeof((pool).buf[0]), ds_PoolSlotOffset(pool))

#define ds_PoolTryDeserialize(mem, ss, pool, __growable)                                                \
	ds_PoolTryDeserializeInternal((mem), (ss), &(pool).heap_allocated, (void **) &(pool).buf,          \
	                              &(pool).length, &(pool).count, &(pool).count_max, &(pool).next_free,  \
	                              &(pool).growable, sizeof((pool).buf[0]), ds_PoolSlotOffset(pool),     \
	                              (__growable))


/*
TPool API
---------
Thread-safe pool of struct Type: adds, removes and address lookups may run on any threads at the same
time. Slots are never moved (the pool grows by adding blocks), so addresses stay valid for the pool's
lifetime and may be held like indices. Which index an add returns depends on thread timing.

::: Usage and Documentation :::

struct Type contains TPOOL_NODE (its fields belong to the pool). DEFINE_TPOOL(Type) defines the pool type
ds_TPool(Type) (once, where Type is declared); the macros below work on any TPool type. Every thread using
the pool needs ds_ThreadSelfIndex() < the logical_core_count given to Alloc.

    struct node { TPOOL_NODE; ... };
    DEFINE_TPOOL(node);

    ds_TPool(node) pool;
    ds_TPoolAlloc(pool, g_arch_config->logical_core_count, 1024);
    const struct slot slot = ds_TPoolAdd(pool);                       (any thread)
    ds_TPoolRemove(pool, slot.index);                                 (any thread)
    ds_TPoolDealloc(pool);

The macros (pool is a ds_TPool(Type) lvalue; arguments may be evaluated more than once; Alloc, Dealloc and
Flush only while no other thread uses the pool):

    // Set up the pool with a first block of PowerOfTwoCeil(initial_count) slots (initial_count <= 2^31)
    // and one free list per thread index < logical_core_count. Fatal if memory runs out.
    void            ds_TPoolAlloc(pool, logical_core_count, initial_count)

    // Free all blocks and free lists.
    void            ds_TPoolDealloc(pool)

    // Remove all slots (the blocks stay); the next fresh slots start at index 0.
    void            ds_TPoolFlush(pool)

    // Allocate a slot and return { address, index }: from the calling thread's free list, else stolen
    // from another thread's list, else a fresh slot (ds_TPoolIncrement). Contents are uninitialized.
    struct slot     ds_TPoolAdd(pool)

    // Remove the slot at index onto the calling thread's free list.
    void            ds_TPoolRemove(pool, index)

    // Return the address of the slot at index.
    struct Type *   ds_TPoolAddress(pool, index)

    // Allocate a never used slot (past every slot handed out so far), adding a block if needed (one
    // thread adds it while the others wait). Fatal if memory runs out.
    struct slot     ds_TPoolIncrement(pool)

    // Push the slot at index onto, or pop the latest pushed slot from ({ NULL, U32_MAX } if empty),
    // free list list (< logical_core_count). ds_TPoolRemove/ds_TPoolAdd use the calling thread's list.
    void            ds_TPoolFreeListPush(pool, list, index)
    struct slot     ds_TPoolFreeListPop(pool, list)

A TPool must stay below 2^31 slots (asserted in debug builds only).
*/

#define TPOOL_NODE		u64 tpool_tail;		\
				u32 tpool_tag

#define TSTACK_NULL		U64_MAX

/* A free list, alone on its cache line. */
struct ds_TStack
{
	u64	a_head;
	u8	pad[DS_CACHE_LINE - sizeof(u64)];
};

/* Internal: the type-independent part of a TPool. */
struct ds_TPoolState
{
	u32			a_count_max;
	u32			a_length;
	u32			a_adding_memory;	/* Bool: Is allocating additional memory */
	u32			block_count;
	u32			block_length_next;

	u32			initial_length;
	u32			shift;

	u32			steal_max_iterations;
	u32			free_list_count;
	struct ds_TStack *	t_free_list;		/* indexed by thread index */
};

#define ds_TPool(T) ds_TPool_ ## T

#define DEFINE_TPOOL(T)                         \
typedef struct ds_TPool(T)                      \
{                                               \
	struct ds_TPoolState	state;              \
	struct T *		block[32];                  \
} ds_TPool(T)

/* Internal: see ds_TPoolAlloc, ds_TPoolDealloc, ds_TPoolFlush. */
void ds_TPoolAllocInternal(struct ds_TPoolState *state, void **block, const u32 logical_core_count, const u32 initial_count, const u64 slot_size);
void ds_TPoolDeallocInternal(struct ds_TPoolState *state, void **block);
void ds_TPoolFlushInternal(struct ds_TPoolState *state);
/* Internal: wait until slot index exists, adding a block if no other thread is (the cold path of Increment). */
void ds_TPoolGrowInternal(struct ds_TPoolState *state, void **block, const u32 index, const u64 slot_size);

/* Internal: block, offset in the block and untyped address of slot index. */
static ds_ForceInline u32 ds_TPoolBlockInternal(const struct ds_TPoolState *state, const u32 index);
static ds_ForceInline u32 ds_TPoolOffsetInternal(const struct ds_TPoolState *state, const u32 index);
static ds_ForceInline u8 *ds_TPoolAddressInternal(const struct ds_TPoolState *state, void *const *block, const u32 index, const u64 slot_size);
/* Internal: see ds_TPoolFreeListPush, ds_TPoolFreeListPop, ds_TPoolIncrement, ds_TPoolAdd. */
static ds_ForceInline void ds_TStackPushInternal(struct ds_TStack *stack, const struct ds_TPoolState *state, void *const *block, const u32 index, const u64 slot_size, const u64 tail_offset, const u64 tag_offset);
static ds_ForceInline struct slot ds_TStackPopInternal(struct ds_TStack *stack, const struct ds_TPoolState *state, void *const *block, const u64 slot_size, const u64 tail_offset);
static ds_ForceInline struct slot ds_TPoolIncrementInternal(struct ds_TPoolState *state, void **block, const u64 slot_size);
static ds_ForceInline struct slot ds_TPoolAddInternal(struct ds_TPoolState *state, void **block, const u64 slot_size, const u64 tail_offset);

/* Internal: node field offsets (constants; never reads block, see ds_PoolSlotOffset) */
#define ds_TPoolTailOffset(pool)	((u64) &((1 ? NULL : (pool).block[0])->tpool_tail))
#define ds_TPoolTagOffset(pool)		((u64) &((1 ? NULL : (pool).block[0])->tpool_tag))

#define ds_TPoolAlloc(pool, logical_core_count, initial_count)                                          \
	ds_TPoolAllocInternal(&(pool).state, (void **) (pool).block, (logical_core_count), (initial_count), \
	                      sizeof((pool).block[0][0]))

#define ds_TPoolDealloc(pool)	ds_TPoolDeallocInternal(&(pool).state, (void **) (pool).block)
#define ds_TPoolFlush(pool)	ds_TPoolFlushInternal(&(pool).state)

#define ds_TPoolAddress(pool, index)                                                                    \
	((pool).block[ds_TPoolBlockInternal(&(pool).state, (index))] + ds_TPoolOffsetInternal(&(pool).state, (index)))

#define ds_TPoolIncrement(pool)                                                                         \
	ds_TPoolIncrementInternal(&(pool).state, (void **) (pool).block, sizeof((pool).block[0][0]))

#define ds_TPoolAdd(pool)                                                                               \
	ds_TPoolAddInternal(&(pool).state, (void **) (pool).block, sizeof((pool).block[0][0]), ds_TPoolTailOffset(pool))

#define ds_TPoolFreeListPush(pool, list, index)                                                         \
	ds_TStackPushInternal((pool).state.t_free_list + (list), &(pool).state, (void **) (pool).block,     \
	                      (index), sizeof((pool).block[0][0]), ds_TPoolTailOffset(pool), ds_TPoolTagOffset(pool))

#define ds_TPoolFreeListPop(pool, list)                                                                 \
	ds_TStackPopInternal((pool).state.t_free_list + (list), &(pool).state, (void **) (pool).block,      \
	                     sizeof((pool).block[0][0]), ds_TPoolTailOffset(pool))

#define ds_TPoolRemove(pool, index)	ds_TPoolFreeListPush(pool, ds_ThreadSelfIndex(), index)


/******************************** CPool inline definitions ********************************/

static inline struct slot ds_CPoolPushInternal(void **buf, u32 *count, u32 *length, u32 *heap_allocated, const u64 slot_size, const u32 growable)
{
    if (*count == *length)
    {                                                                                   
        if (!growable)                                                           
        {                                                                               
            return (struct slot) { .index = U32_MAX, .address = (u8*)(*buf) - slot_size }; 
        } 
                                                                                        
        const u64 new_length = (*length)
                             ? (*length) << 1
                             : 1;
        const u64 new_size = (new_length+1) * slot_size;
        /* a deallocated pool has no buffer */
        const u64 old_size = (*buf) ? ((*length)+1) * slot_size : 0;
        void *base = (*buf) ? (u8 *)(*buf) - slot_size : NULL;
        base = ds_Realloc(base, old_size, new_size, DS_CACHE_LINE);
        PoisonAddress(base, new_size);                                
        UnpoisonAddress(base, (*count + 1)*slot_size);                
        *buf = (u8 *) base + slot_size;
        *length = (u32) new_length;
        *heap_allocated = 1;
    }

    const struct slot slot = { .index = *count, .address = (u8*)(*buf) + *count*slot_size };
    UnpoisonAddress(slot.address, slot_size);                
    *count += 1;                                                              
    return slot;
}

static inline struct slot ds_CPoolPushMemcpyInternal(void **buf, u32 *count, u32 *length, u32 *heap_allocated, const u64 slot_size, const u32 growable, const void *src)
{
    struct slot slot = ds_CPoolPushInternal(buf, count, length, heap_allocated, slot_size, growable);
    memcpy(slot.address, src, slot_size);
    return slot;
}

/******************************** Pool inline definitions *********************************/

/* Internal: see ds_PoolAdd. */
static ds_ForceInline struct slot ds_PoolAddInternal(void **buf, u32 *length, u32 *count, u32 *count_max, u32 *next_free, const u32 growable, const u64 slot_size, const u64 pool_slot_offset)
{
	if (*count == *length)
	{
		if (!growable)
		{
			return (struct slot) { .address = NULL, .index = POOL_NULL };
		}
		/* a full pool has no free slot, so the add below takes slot count_max */
		ds_PoolGrowInternal(buf, length, slot_size);
	}

	struct slot allocation;
	if (*next_free != POOL_INDEX_MASK)
	{
		allocation.index = *next_free;
		allocation.address = (u8 *) *buf + allocation.index*slot_size;
		UnpoisonAddress(allocation.address, slot_size);
		const u32 pool_slot = *(const u32 *) ((u8 *) allocation.address + pool_slot_offset);
		ds_Assert(pool_slot & POOL_ALLOCATION_MASK);
		*next_free = pool_slot & POOL_INDEX_MASK;
	}
	else
	{
		allocation.index = *count_max;
		allocation.address = (u8 *) *buf + allocation.index*slot_size;
		UnpoisonAddress(allocation.address, slot_size);
		*count_max += 1;
	}

	*(u32 *) ((u8 *) allocation.address + pool_slot_offset) = 0;
	*count += 1;
	return allocation;
}

/* Internal: see ds_PoolRemove. */
static ds_ForceInline void ds_PoolRemoveInternal(void *buf, u32 *count, u32 *next_free, const u32 count_max, const u32 index, const u64 slot_size, const u64 pool_slot_offset)
{
	ds_Assert(index < count_max);

	u8 *slot = (u8 *) buf + index*slot_size;
	u32 *pool_slot = (u32 *) (slot + pool_slot_offset);
	ds_Assert((*pool_slot & POOL_ALLOCATION_MASK) == 0);

	*pool_slot = POOL_ALLOCATION_MASK | *next_free;
	*next_free = index;
	*count -= 1;

	PoisonAddress(slot, pool_slot_offset);
	PoisonAddress(slot + pool_slot_offset + sizeof(u32), slot_size - pool_slot_offset - sizeof(u32));
}

/* Internal: debug checks of ds_PoolIndex (empty in release builds). */
static ds_ForceInline void ds_PoolIndexCheck(const void *buf, const u32 length, const void *addr, const u64 slot_size)
{
	ds_Assert((u64) addr >= (u64) buf);
	ds_Assert((u64) addr < (u64) buf + length*slot_size);
	ds_Assert(((u64) addr - (u64) buf) % slot_size == 0);
}

/******************************** TPool inline definitions ********************************/

/* Internal: block and offset in the block of slot index. */
static ds_ForceInline u32 ds_TPoolBlockInternal(const struct ds_TPoolState *state, const u32 index)
{
	return (index < state->initial_length) ? 0 : 32 - Clz32(index >> state->shift);
}

static ds_ForceInline u32 ds_TPoolOffsetInternal(const struct ds_TPoolState *state, const u32 index)
{
	return (index < state->initial_length) ? index : index & ((state->initial_length << (31 - Clz32(index >> state->shift))) - 1);
}

/* Internal: untyped address of slot index. */
static ds_ForceInline u8 *ds_TPoolAddressInternal(const struct ds_TPoolState *state, void *const *block, const u32 index, const u64 slot_size)
{
	return (u8 *) block[ds_TPoolBlockInternal(state, index)] + ds_TPoolOffsetInternal(state, index)*slot_size;
}

/* Internal: see ds_TPoolFreeListPush. */
static ds_ForceInline void ds_TStackPushInternal(struct ds_TStack *stack, const struct ds_TPoolState *state, void *const *block, const u32 index, const u64 slot_size, const u64 tail_offset, const u64 tag_offset)
{
	u8 *node = ds_TPoolAddressInternal(state, block, index, slot_size);
	u64 *tail = (u64 *) (node + tail_offset);
	u32 *tag = (u32 *) (node + tag_offset);
	*tag += 1;
	const u64 new_head = ((u64) *tag << 32) + index;
	/* a popper may load the link meanwhile: atomic (relaxed: the release CAS orders the store before the publish) */
	u64 expected = AtomicLoadRlx64(&stack->a_head);
	do
	{
		AtomicStoreRlx64(tail, expected);
	} while (!AtomicCompareExchangeRelRlx64(&stack->a_head, &expected, new_head));
}

/* Internal: see ds_TPoolFreeListPop. */
static ds_ForceInline struct slot ds_TStackPopInternal(struct ds_TStack *stack, const struct ds_TPoolState *state, void *const *block, const u64 slot_size, const u64 tail_offset)
{
	u8 *node;
	u64 local_head = AtomicLoadAcq64(&stack->a_head);
	do
	{
		if (local_head == TSTACK_NULL)
		{
			return (struct slot) { .address = NULL, .index = U32_MAX };
		}
		node = ds_TPoolAddressInternal(state, block, (u32) local_head, slot_size);
	} while (!AtomicCompareExchangeAcqRelAcq64(&stack->a_head, &local_head, AtomicLoadRlx64((u64 *) (node + tail_offset))));

	return (struct slot) { .address = node, .index = (u32) local_head };
}

/* Internal: see ds_TPoolIncrement. */
static ds_ForceInline struct slot ds_TPoolIncrementInternal(struct ds_TPoolState *state, void **block, const u64 slot_size)
{
	struct slot slot;
	slot.index = AtomicFetchAddRlx32(&state->a_count_max, 1);
	if (slot.index >= (u32) AtomicLoadAcq32(&state->a_length))
	{
		ds_TPoolGrowInternal(state, block, slot.index, slot_size);
	}
	slot.address = ds_TPoolAddressInternal(state, block, slot.index, slot_size);
	return slot;
}

/* Internal: see ds_TPoolAdd. */
static ds_ForceInline struct slot ds_TPoolAddInternal(struct ds_TPoolState *state, void **block, const u64 slot_size, const u64 tail_offset)
{
	struct slot slot = ds_TStackPopInternal(state->t_free_list + ds_ThreadSelfIndex(), state, block, slot_size, tail_offset);
	if (slot.address)
	{
		return slot;
	}

	for (u32 i = 0; i < state->steal_max_iterations; ++i)
	{
		const u32 t = (u32) RngU64Range(0, state->free_list_count-1);
		slot = ds_TStackPopInternal(state->t_free_list + t, state, block, slot_size, tail_offset);
		if (slot.address)
		{
			return slot;
		}
	}

	return ds_TPoolIncrementInternal(state, block, slot_size);
}


#ifdef __cplusplus
}
#endif

#endif
