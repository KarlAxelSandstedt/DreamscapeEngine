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
ds_compact_pool.h
=================
CPool: ds_CPool(T), compact typed array; removal moves the last element into the hole. The inline
definitions follow at the bottom.
Included by ds_allocator.h after its Heap section, not on its own.
*/

#ifndef __DS_COMPACT_POOL_H__
#define __DS_COMPACT_POOL_H__

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

buf[-1] is a stub slot: pushes onto a full fixed pool write into it, harmlessly, and intrusive lists over
pools (ds_list.h) use index -1 (DS_STUB_INDEX) as their sentinel. Length 0 allocates nothing.
The allocation (stub included) is cache-line aligned. Serialized format (LE): u32 length, u32 count, then the count elements as raw bytes.
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

struct SS;
/* Internal: see ds_CPoolSerialize. */
void ds_CPoolSerializeInternal(struct SS *ss, const void *buf, const u32 length, const u32 count, const u64 slot_size);
/* Internal: see ds_CPoolTryDeserialize. */
u32  ds_CPoolTryDeserializeInternal(struct arena *mem, struct SS *ss, void **buf, u32 *length, u32 *count, u32 *growable, u32 *heap_allocated, const u64 slot_size, const u32 alloc_growable);

#define ds_CPoolSerializeSize( pool ) (2*sizeof(u32) + (u64) (pool).count*sizeof((pool).buf[0]))
#define ds_CPoolSerialize( ss, pool ) ds_CPoolSerializeInternal((ss), (pool).buf, (pool).length, (pool).count, sizeof((pool).buf[0]))
#define ds_CPoolTryDeserialize( mem, ss, pool, __growable )                                     \
    ds_CPoolTryDeserializeInternal((mem), (ss), (void **) &(pool).buf, &(pool).length,          \
                                   &(pool).count, &(pool).growable, &(pool).heap_allocated,     \
                                   sizeof((pool).buf[0]), (__growable))

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

#endif
