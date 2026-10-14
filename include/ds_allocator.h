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
    Pool        POOL_NODE structs: stable indices, free list, serializable.
    TPool       TPOOL_NODE structs: thread-safe pool whose addresses never move.
    TStack      Treiber stack, the TPool's per-thread free lists.
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
at the start and is still contiguous. Linux and Windows only (no web implementation).
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
	u8 *buf;	/* buf[0, 2*mem_total): the second half mirrors the first */
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
Return a ring of at least mem_hint bytes (Linux: rounded up to whole pages; Windows: to a power of two of
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

::: Internals :::

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

struct Type contains POOL_NODE (its pool_slot field belongs to the pool). POOL_DECLARE(Type) declares
struct TypePool and the functions below (in a header), POOL_DEFINE(Type) defines them (in one source
file).

    struct node { POOL_NODE; ... };
    POOL_DECLARE(node);
    POOL_DEFINE(node)

    struct nodePool pool = nodePoolAlloc(NULL, 128, GROWABLE);
    const struct slot slot = nodePoolAdd(&pool);              (slot.index is the handle)
    for (u32 i = 0; i < pool.count_max; ++i)
        if (ds_PoolSlotAllocated(pool.buf + i)) { ... }       (iterate the live slots)
    nodePoolRemove(&pool, slot.index);
    nodePoolDealloc(&pool);

The functions:

    // Allocate length slots in mem (fixed) or on the heap (mem == NULL; growable allowed). On failure the
    // pool is empty (buf NULL, length 0).
    struct TypePool TypePoolAlloc(struct arena *mem, const u32 length, const u32 growable)

    // Free a heap pool's memory (arena memory stays with the arena); the pool becomes empty.
    void            TypePoolDealloc(struct TypePool *pool)

    // Remove all slots; the next adds return indices 0, 1, 2, ...
    void            TypePoolFlush(struct TypePool *pool)

    // Allocate a slot and return { address, index }: the most recently removed slot, else the next unused
    // one. Its contents are uninitialized (pool_slot aside). Full and not growable: { NULL, POOL_NULL }.
    struct slot     TypePoolAdd(struct TypePool *pool)

    // Remove an allocated slot, by index or by address.
    void            TypePoolRemove(struct TypePool *pool, const u32 index)
    void            TypePoolRemoveAddress(struct TypePool *pool, const struct Type *addr)

    // Return the index of a slot's address.
    u32             TypePoolIndex(const struct TypePool *pool, const struct Type *addr)

    // 1 if the slot at ptr (in buf[0, count_max)) is allocated (a macro).
    u32             ds_PoolSlotAllocated(const struct Type *ptr)

    // Bytes TypePoolSerialize writes.
    u64             TypePoolSerializeSize(const struct TypePool *pool)

    // Write the pool to ss (at a byte boundary; it must fit: TypePoolSerializeSize).
    void            TypePoolSerialize(struct ss *ss, const struct TypePool *pool)

    // Read a pool from ss into mem (fixed) or the heap (mem == NULL; growable allowed), with slot positions
    // and free-list order as written (later adds return the same indices). Returns 1 on success; 0 if mem
    // is full or ss holds no valid pool: inconsistent counts, an index out of range or repeated (the pool
    // is empty, ss's position unchanged).
    u32             TypePoolTryDeserialize(struct arena *mem, struct ss *ss, struct TypePool *pool, const u32 growable)

::: Internals :::

buf[-1] is a stub slot, so index -1 (DS_STUB_INDEX) is addressable: intrusive lists over pools
(ds_list.h) use it as their sentinel. pool_slot is 0 for an allocated slot; a removed slot holds
POOL_ALLOCATION_MASK | the next free index, POOL_INDEX_MASK ending the free list. Growth reallocates
(length + 1)*2 slots, stub included. Serialized format (LE): u32 length, count, count_max; the
count_max - count free indices in free-list order; the count allocated slots as raw bytes, each with
pool_slot replaced by its index.
*/

#define POOL_NODE 	        u32 pool_slot
#define POOL_NULL		        0xffffffff
#define POOL_ALLOCATION_MASK	0x80000000
#define POOL_INDEX_MASK		    0x7fffffff

#define ds_PoolSlotAllocated(ptr)	(!((ptr)->pool_slot & POOL_ALLOCATION_MASK))


/* Internal: see PoolAlloc. Returns 1 on success and 0 on failure. */
u32  ds_PoolAllocInternal(struct arena *mem, u32 *heap_allocated, void **buf, u32 *length, u32 *count, u32 *count_max, u32 *next_free, u32 *growable, const u64 slot_size, const u32 alloc_length, const u32 alloc_growable);
/* Internal: see PoolSerialize. */
void ds_PoolSerializeInternal(struct ss *ss, const void *buf, const u32 length, const u32 count, const u32 count_max, const u32 next_free, const u64 slot_size, const u64 pool_slot_offset);
/* Internal: see PoolTryDeserialize. */
u32  ds_PoolTryDeserializeInternal(struct arena *mem, struct ss *ss, u32 *heap_allocated, void **buf, u32 *length, u32 *count, u32 *count_max, u32 *next_free, u32 *growable, const u64 slot_size, const u64 pool_slot_offset, const u32 alloc_growable);

#define POOL_DECLARE(T)                                                                          \
        POOL_STRUCT_DEFINE(T);                                                                   \
        POOL_ALLOC_DECLARE(T);                                                                   \
        POOL_DEALLOC_DECLARE(T);                                                                 \
        POOL_FLUSH_DECLARE(T);                                                                   \
        POOL_ADD_DECLARE(T);                                                                     \
        POOL_REMOVE_DECLARE(T);                                                                  \
        POOL_REMOVE_ADDRESS_DECLARE(T);                                                          \
        POOL_INDEX_DECLARE(T);                                                                   \
        POOL_SERIALIZE_SIZE_DECLARE(T);                                                          \
        POOL_SERIALIZE_DECLARE(T);                                                               \
        POOL_TRY_DESERIALIZE_DECLARE(T)

#define POOL_DEFINE(T)                                                                           \
        POOL_ALLOC_DEFINE(T)                                                                     \
        POOL_DEALLOC_DEFINE(T)                                                                   \
        POOL_FLUSH_DEFINE(T)                                                                     \
        POOL_ADD_DEFINE(T)                                                                       \
        POOL_REMOVE_DEFINE(T)                                                                    \
        POOL_REMOVE_ADDRESS_DEFINE(T)                                                            \
        POOL_INDEX_DEFINE(T)                                                                     \
        POOL_SERIALIZE_SIZE_DEFINE(T)                                                            \
        POOL_SERIALIZE_DEFINE(T)                                                                 \
        POOL_TRY_DESERIALIZE_DEFINE(T)

#define POOL_STRUCT_DEFINE(T)                                                                       \
typedef struct T ## Pool                                                                            \
{                                                                                                   \
	struct T *buf;		                                                                            \
	u32 length;			        /* array length 				                                */  \
	u32 count;			        /* current count of occupied slots 		                        */  \
	u32 count_max;		        /* max count used over the object's lifetime 	                */  \
	u32 next_free;		        /* next free index if != POOL_INDEX_MASK 		                */  \
	u32 growable;		        /* is the memory growable? 			                            */  \
	u32 heap_allocated;	        /* buf - 1 is a ds_Alloc block (else arena memory)              */  \
} T ## Pool

#define POOL_ALLOC_DECLARE(T)                                                                       \
struct T ## Pool    T ## PoolAlloc(struct arena *mem,                                               \
                                 const u32 length,                                                  \
                                 const u32 growable)

#define POOL_DEALLOC_DECLARE(T)                                                                     \
void                T ## PoolDealloc(struct T ## Pool *pool)                                           

#define POOL_FLUSH_DECLARE(T)                                                                       \
void                T ## PoolFlush(struct T ## Pool *pool)                                         

#define POOL_ADD_DECLARE(T)                                                                         \
struct slot         T ## PoolAdd(struct T ## Pool *pool)

#define POOL_REMOVE_DECLARE(T)                                                                      \
void                T ## PoolRemove(struct T ## Pool *pool,                                         \
                                    const u32 index)   

#define POOL_REMOVE_ADDRESS_DECLARE(T)                                                              \
void                T ## PoolRemoveAddress(struct T ## Pool *pool,                                  \
                                           const struct T *addr)    

#define POOL_INDEX_DECLARE(T)                                                                       \
u32                 T ## PoolIndex(const struct T ## Pool *pool,                                    \
                                   const struct T *addr)

#define POOL_SERIALIZE_SIZE_DECLARE(T)                                                              \
u64                 T ## PoolSerializeSize(const struct T ## Pool *pool)

#define POOL_SERIALIZE_DECLARE(T)                                                                   \
void                T ## PoolSerialize(struct ss *ss, const struct T ## Pool *pool)

#define POOL_TRY_DESERIALIZE_DECLARE(T)                                                             \
u32                 T ## PoolTryDeserialize(struct arena *mem, struct ss *ss,                       \
                                            struct T ## Pool *pool, const u32 growable)

#define POOL_ALLOC_DEFINE(T)                                                                        \
POOL_ALLOC_DECLARE(T)                                                                               \
{                                                                                                   \
	struct T ## Pool pool;                                                                          \
	ds_PoolAllocInternal(mem, &pool.heap_allocated, (void **) &pool.buf, &pool.length, &pool.count, \
	                     &pool.count_max, &pool.next_free, &pool.growable, sizeof(struct T),        \
	                     length, growable);                                                         \
	return pool;                                                                                    \
}

#define POOL_DEALLOC_DEFINE(T)                                                                      \
POOL_DEALLOC_DECLARE(T)                                                                             \
{                                                                                                   \
	if (pool->heap_allocated && pool->buf)                                                          \
	{                                                                                               \
		ds_Free(pool->buf - 1);                                                                     \
	}                                                                                               \
	pool->buf = NULL;                                                                               \
	pool->length = 0;                                                                               \
	pool->count = 0;                                                                                \
	pool->count_max = 0;                                                                            \
	pool->next_free = POOL_INDEX_MASK;                                                              \
	pool->heap_allocated = 0;                                                                       \
}

#define POOL_FLUSH_DEFINE(T)                                                                        \
POOL_FLUSH_DECLARE(T)                                                                               \
{                                                                                                   \
	pool->count = 0;                                                                                \
	pool->count_max = 0;                                                                            \
	pool->next_free = POOL_INDEX_MASK;                                                              \
	PoisonAddress(pool->buf, sizeof(struct T) * pool->length);                                      \
}

#define POOL_ADD_DEFINE(T)                                                                          \
POOL_ADD_DECLARE(T)                                                                                 \
{                                                                                                   \
	struct slot allocation = { .address = NULL, .index = POOL_NULL };                               \
	if (pool->count < pool->length)                                                                 \
	{                                                                                               \
		if (pool->next_free != POOL_INDEX_MASK)                                                     \
		{                                                                                           \
			allocation.address = pool->buf + pool->next_free;                                       \
			allocation.index = pool->next_free;                                                     \
			UnpoisonAddress(allocation.address, sizeof(struct T));                                  \
                                                                                                    \
            pool->next_free = pool->buf[allocation.index].pool_slot & POOL_INDEX_MASK;              \
            ds_Assert(pool->buf[allocation.index].pool_slot & POOL_ALLOCATION_MASK);                \
		}                                                                                           \
		else                                                                                        \
		{                                                                                           \
			allocation.address = pool->buf + pool->count_max;                                       \
			allocation.index = pool->count_max;                                                     \
			UnpoisonAddress(allocation.address, sizeof(struct T));                                  \
			pool->count_max += 1;                                                                   \
		}	                                                                                        \
        pool->buf[allocation.index].pool_slot = 0;                                                  \
		pool->count += 1;                                                                           \
	}                                                                                               \
	else if (pool->growable)                                                                        \
	{                                                                                               \
	    const u32 length_max = (U32_MAX >> 1);                                                      \
        const u32 old_length_including_stub = pool->length + 1;                                     \
	    u32 new_length_including_stub = old_length_including_stub << 1;                             \
	    if (old_length_including_stub == length_max)                                                \
	    {                                                                                           \
	    	LogString(T_SYSTEM, S_FATAL, "pool allocator full, exiting");                           \
	    	FatalCleanupAndExit();                                                                  \
	    }                                                                                           \
	                                                                                                \
	    if (new_length_including_stub > length_max)                                                 \
	    {                                                                                           \
	    	new_length_including_stub = length_max;                                                 \
	    }                                                                                           \
	                                                                                                \
	    /* ds_Realloc is fatal on failure */                                                        \
	    pool->buf = ds_Realloc(pool->buf - 1, old_length_including_stub*sizeof(struct T),           \
	                           new_length_including_stub*sizeof(struct T), DS_CACHE_LINE);         \
	    UnpoisonAddress(pool->buf, old_length_including_stub*sizeof(struct T));                     \
	    PoisonAddress(pool->buf + old_length_including_stub,                                        \
                (new_length_including_stub-old_length_including_stub)*sizeof(struct T));            \
                                                                                                    \
        pool->buf += 1;                                                                             \
        pool->length = new_length_including_stub-1;                                                 \
	                                                                                                \
		allocation.address = pool->buf + pool->count_max;                                           \
		allocation.index = pool->count_max;                                                         \
		UnpoisonAddress(allocation.address, sizeof(struct T));                                      \
        pool->buf[allocation.index].pool_slot = 0;                                                  \
		pool->count_max += 1;                                                                       \
		pool->count += 1;                                                                           \
	}                                                                                               \
	return allocation;                                                                              \
}

#define POOL_REMOVE_DEFINE(T)                                                                       \
POOL_REMOVE_DECLARE(T)                                                                              \
{                                                                                                   \
    ds_Assert(index < pool->count_max);                                                             \
                                                                                                    \
    struct T *addr = pool->buf + index;                                                             \
	ds_Assert( (addr->pool_slot & POOL_ALLOCATION_MASK) == 0);                                      \
                                                                                                    \
    addr->pool_slot = POOL_ALLOCATION_MASK | pool->next_free;                                       \
	pool->next_free = index;                                                                        \
	pool->count -= 1;                                                                               \
                                                                                                    \
    const u64 offset = (u64) &(((struct T *)0)->pool_slot);                                         \
                                                                                                    \
	PoisonAddress(pool->buf + index, offset);                                                       \
	PoisonAddress((u8*)(pool->buf + index) + offset + sizeof(addr->pool_slot),                      \
            sizeof(*addr) - offset - sizeof(addr->pool_slot));                                      \
}

#define POOL_REMOVE_ADDRESS_DEFINE(T)                                                               \
POOL_REMOVE_ADDRESS_DECLARE(T)                                                                      \
{                                                                                                   \
	const u32 index = T ## PoolIndex(pool, addr);                                                   \
	T ## PoolRemove(pool, index);                                                                   \
}

#define POOL_INDEX_DEFINE(T)                                                                        \
POOL_INDEX_DECLARE(T)                                                                               \
{                                                                                                   \
	ds_Assert((u64) addr >= (u64) pool->buf);                                                       \
	ds_Assert((u64) addr < (u64) (pool->buf + pool->length));                                       \
	ds_Assert(((u8*) addr - (u8*) pool->buf) % sizeof(struct T) == 0);                              \
	return (u32) (addr - pool->buf);                                                                \
}

#define POOL_SERIALIZE_SIZE_DEFINE(T)                                                               \
POOL_SERIALIZE_SIZE_DECLARE(T)                                                                      \
{                                                                                                   \
	return 3*sizeof(u32) + (u64) (pool->count_max - pool->count)*sizeof(u32)                        \
	                     + (u64) pool->count*sizeof(struct T);                                      \
}

#define POOL_SERIALIZE_DEFINE(T)                                                                    \
POOL_SERIALIZE_DECLARE(T)                                                                           \
{                                                                                                   \
	ds_PoolSerializeInternal(ss, pool->buf, pool->length, pool->count, pool->count_max,             \
	                         pool->next_free, sizeof(struct T), (u64) &(((struct T *)0)->pool_slot));\
}

#define POOL_TRY_DESERIALIZE_DEFINE(T)                                                              \
POOL_TRY_DESERIALIZE_DECLARE(T)                                                                     \
{                                                                                                   \
	return ds_PoolTryDeserializeInternal(mem, ss, &pool->heap_allocated, (void **) &pool->buf,     \
	                                     &pool->length, &pool->count, &pool->count_max,             \
	                                     &pool->next_free, &pool->growable, sizeof(struct T),       \
	                                     (u64) &(((struct T *)0)->pool_slot), growable);            \
}


/*
TPool API
---------
Thread-safe pool of struct Type: adds, removes and address lookups may run on any threads at the same
time. Slots are never moved (the pool grows by adding blocks), so addresses stay valid for the pool's
lifetime and may be held like indices. Which index an add returns depends on thread timing.

::: Usage and Documentation :::

struct Type contains TPOOL_NODE (its fields belong to the pool). TPOOL_DECLARE(Type) declares struct
TypeTPool and the functions below (in a header), TPOOL_DEFINE(Type) defines them (in one source file).
Every thread using the pool needs ds_ThreadSelfIndex() < the logical_core_count given to Alloc.

    struct node { TPOOL_NODE; ... };
    TPOOL_DECLARE(node)
    TPOOL_DEFINE(node)

    struct nodeTPool pool;
    nodeTPoolAlloc(&pool, g_arch_config->logical_core_count, 1024);
    const struct slot slot = nodeTPoolAdd(&pool);                     (any thread)
    nodeTPoolRemove(&pool, slot.index);                               (any thread)
    nodeTPoolDealloc(&pool);

The functions (Alloc, Dealloc and Flush only while no other thread uses the pool):

    // Set up the pool with a first block of PowerOfTwoCeil(initial_count) slots (initial_count <= 2^31)
    // and one free list per thread index < logical_core_count. Fatal if memory runs out.
    void            TypeTPoolAlloc(struct TypeTPool *pool, const u32 logical_core_count, const u32 initial_count)

    // Free all blocks and free lists.
    void            TypeTPoolDealloc(struct TypeTPool *pool)

    // Remove all slots (the blocks stay); the next fresh slots start at index 0.
    void            TypeTPoolFlush(struct TypeTPool *pool)

    // Allocate a slot and return { address, index }: from the calling thread's free list, else stolen
    // from another thread's list, else a fresh slot (TypeTPoolIncrement). Contents are uninitialized.
    struct slot     TypeTPoolAdd(struct TypeTPool *pool)

    // Remove the slot at index onto the calling thread's free list.
    void            TypeTPoolRemove(struct TypeTPool *pool, const u32 index)

    // Return the address of the slot at index.
    struct Type *   TypeTPoolAddress(const struct TypeTPool *pool, const u32 index)

    // Allocate a never used slot (past every slot handed out so far), adding a block if needed (one
    // thread adds it while the others wait). Fatal if memory runs out.
    struct slot     TypeTPoolIncrement(struct TypeTPool *pool)

::: Internals :::

Free lists are TStacks (below), one per thread index, each on its own cache line; an add tries its own
list, then 2*logical_core_count random others. Block 0 has N = initial_length (a power of two) slots,
block 1 also N, block k >= 1 has N << (k-1), so the pool length doubles with each block and an index maps
to its block without a search:

    blocks:   [N][N][2N][4N][8N] ...   (block k >= 1 starts at index N << (k-1))
    shift = log2(N) = Ctz32(N)

    i < N:   block 0, offset i
    i >= N:  block  = 1 + (bit index of the leading 1 of i >> shift) = 32 - Clz32(i >> shift)
             offset = i & ((N << (block-1)) - 1)       (block k >= 1 holds the indices with that leading bit)

At most 32 blocks (a_length < 2^31).
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
                                                                                                            \
    struct struct_name *block[32];                                                                          \
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
    pool->t_free_list = ds_Alloc(pool->free_list_count*sizeof(pool->t_free_list[0]), DS_CACHE_LINE);        \
    if (!pool->t_free_list)                                                                                 \
    {                                                                                                       \
			LogString(T_SYSTEM, S_FATAL, "Failed to Allocate ds_TPool free list memory, exiting.");         \
			FatalCleanupAndExit();                                                                          \
    }                                                                                                       \
                                                                                                            \
    for (u32 i = 0; i < logical_core_count; ++i)                                                            \
    {                                                                                                       \
        struct_name ## FreeListTStackInit(pool->t_free_list + i, pool);                                     \
        ds_Assert((u64) (pool->t_free_list + i) % 64 == 0);                                                 \
    }                                                                                                       \
                                                                                                            \
    const u64 memsize = pool->block_length_next*sizeof(pool->block[0][0]);                                  \
    pool->block[0] = ds_Alloc(memsize, DS_CACHE_LINE);                                                      \
    if (!pool->block[0])                                                                                    \
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
        ds_Free(pool->block[i]);                                                                            \
    }                                                                                                       \
    ds_Free(pool->t_free_list);                                                                             \
}

#define TPOOL_FLUSH_DEFINE(struct_name)                                                                     \
TPOOL_FLUSH_DECLARE(struct_name)                                                                            \
{                                                                                                           \
    for (u32 i = 0; i < pool->free_list_count; ++i)                                                         \
    {                                                                                                       \
        struct_name ## FreeListTStackFlush(pool->t_free_list + i);                                          \
    }                                                                                                       \
    AtomicStoreRel32(&pool->a_count_max, 0);                                                                \
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
                pool->block[pool->block_count] = ds_Alloc(memsize, DS_CACHE_LINE);                          \
                if (!pool->block[pool->block_count])                                                        \
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
TStack API
----------
Lock-free (Treiber) stack of slot indices, intrusive in the slots of a TypeTPool: the TPool's free lists.
Push and pop may run on any threads at the same time.

::: Usage and Documentation :::

struct Type contains TSTACK_NODE(name); TSTACK_DECLARE(Type, name) / TSTACK_DEFINE(Type, name) generate
the functions below (TPOOL_DECLARE/TPOOL_DEFINE do it for TPOOL_NODE, name = FreeList).

    // Make the stack empty and bind it to pool (whose slots hold the nodes).
    void            TypenameTStackInit(struct TypenameTStack *stack, struct TypeTPool *pool)

    // Make the stack empty (not concurrently with push or pop).
    void            TypenameTStackFlush(struct TypenameTStack *stack)

    // Push the slot at index.
    void            TypenameTStackPush(struct TypenameTStack *stack, const u32 index)

    // Pop the latest pushed slot: { address, index }, or { NULL, U32_MAX } if the stack is empty.
    struct slot     TypenameTStackPop(struct TypenameTStack *stack)

::: Internals :::

The head is (tag << 32) | index of the top slot, TSTACK_NULL if empty; nameTail in each slot holds the
head below it. Each push increments the slot's nameTag, so a slot popped and pushed back between another
thread's load and compare-exchange changes the head (no ABA).
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
    u64                             a_head;                                                                     \
    struct struct_name ## TPool *   pool;                                                                       \
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

#ifdef __cplusplus
}
#endif

#endif
