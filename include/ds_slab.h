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
ds_slab.h
=========
ds_TSlab: per-struct thread-safe slab alloactor.
ds_TPool: per-struct thread-safe slot allocator (ds_TSlab restricted to 1-class)
*/

#ifndef __DS_SLAB_H__
#define __DS_SLAB_H__

////TODO poison slab memory

/*
Slab API
========

::: Usage :::

    struct item { ... };
    DEFINE_ARRAY_STRUCT(item);
    DEFINE_TSLAB(item);

    ds_TSlab(item) slab;
    ds_TSlabAlloc(NULL, slab, length, thread_count);
    ds_TSlabFlush(slab);
    ds_TSlabDealloc(slab);

::: Documentation :::

    // Master:
    // If mem == NULL, reserve memory and initialize allocator.
    // If mem != NULL, allocate memory and initialize allocator. 
    // Returns 1 on success; 0 on failure.
    u32 ds_TSlabAlloc(mem, slab, length, thread_count)

    // Master: 
    // Release any memory held by allocator
    void ds_TSlabDealloc(slab)

    // Master: 
    // Flush allocator resources, reseting counts.
    void ds_TSlabFlush(slab)

    // Thread-safe:
    // Returns ds_Array( T ) with length >= count (smallest power of two). 
    // On failure: { .length = 0, .buf = NULL, .base = U32_MAX }.
    ds_Array(T) ds_TSlabAdd(slab, count)

    // Thread-safe: 
    // Remove array.
    void ds_TSlabRemove(slab, array)

    // Thread-safe:
    // Returns reallocated array, or { .length = 0, .buf = NULL, .base = U32_MAX }
    // on count = 0. Removes any previous array.
    ds_Array(T) ds_TSlabRealloc(slab, array, count)

    // WARNING: Platform-dependent.
    // Master: 
    // Read an ds_Array(T) array from ss into a new slab array (of possibly greater size),
    // and read the written length in count. Returns 1 on success, and return 0 on failure.
    // Fails if the slab is full or ss holds no valid array (the array is empty, count 0, 
    // ss's position unchanged).
    u32 ds_TSlabArrayTryDeserialize(ss, slab, array, count)

TPool API
=========

is ds_TSlab(T), DEFINE_TPOOL(T) is

::: Usage :::

    struct item { ... };
    DEFINE_ARRAY_STRUCT(item);
    DEFINE_TPOOL(item);

    ds_TPool(item) pool;
    ds_TPoolAlloc(NULL, pool, length, thread_count);
    const struct slot slot = ds_TPoolAdd(pool);
    ds_TPoolRemove(pool, slot.index);
    ds_TPoolDealloc(pool);

::: Documentation :::

    // Master: ds_TSlabAlloc, ds_TSlabDealloc, ds_TSlabFlush.
    u32  ds_TPoolAlloc(mem, pool, length, thread_count)
    void ds_TPoolDealloc(pool)
    void ds_TPoolFlush(pool)

    // Thread-safe:
    // Add a slot and return { address, index }; full: { NULL, DS_TSLAB_NULL }.
    struct slot ds_TPoolAdd(pool)

    // Thread-safe:
    // Remove a slot.
    void ds_TPoolRemove(pool, index)
*/

#define DS_TSLAB_NULL            U32_MAX

struct ds_Align(DS_CACHE_LINE) ds_TSlabShared
{
    u32                     a_count_max;
    u8                      pad[DS_CACHE_LINE - sizeof(u32)];
    struct VM               vm;
    struct ds_MagazinePow2  magazine;
    u32                     length;
    u32                     heap_allocated;
};

#define ds_TSlab(T) ds_TSlab_ ## T
#define ds_TPool(T) ds_TSlab(T)

#define DEFINE_TSLAB(T)                                                                 \
typedef struct ds_TSlab(T)                                                              \
{                                                                                       \
    struct ds_TSlabShared    shared;                                                    \
    struct T *              buf;                                                        \
    ds_Array(T) *           array_type;     /* never set: the type of ds_TSlabAdd */   \
} ds_TSlab(T)

u32     ds_TSlabSharedAlloc(struct arena *mem,
                           struct ds_TSlabShared *shared,
                           void **external,
                           const u32 thread_count,
                           const u64 slot_size,
                           const u32 length);

void    ds_TSlabSharedDealloc(struct ds_TSlabShared *shared);

void    ds_TSlabSharedFlush(struct ds_TSlabShared *shared);

struct ds_ArrayUntyped  ds_TSlabSharedAdd(struct ds_TSlabShared *shared,
                                          const u64 slot_size,
                                          const u32 count);

void    ds_TSlabSharedRemove(struct ds_TSlabShared *shared, const u64 slot_size,
                             const u32 base, const u32 length);

struct ds_ArrayUntyped  ds_TSlabSharedRealloc(struct ds_TSlabShared *shared,
                                              const u64 slot_size,
                                              const struct ds_ArrayUntyped array,
                                              const u32 count);

u32     ds_TSlabSharedArrayTryDeserialize(struct SS *ss,
                                          struct ds_TSlabShared *shared,
                                          struct ds_ArrayUntyped *array,
                                          u32 *count,
                                          const u64 slot_size);

/* Internal: see ds_TPoolAdd. */
static ds_ForceInline struct slot ds_TSlabSharedAddSlot(struct ds_TSlabShared *shared, const u64 slot_size);

/* Internal: log and exit; the Log API is not visible here */
ds_NoReturn void ds_TSlabOutOfMemoryInternal(void);

/****************************** Slab inline definitions ******************************/

#define ds_TSlabAlloc(_mem_, _slab_, _length_, _thread_count_)                          \
    ds_TSlabSharedAlloc((_mem_), &(_slab_).shared, (void **) &(_slab_).buf,             \
                       (_thread_count_), sizeof((_slab_).buf[0]), (_length_))

#define ds_TSlabDealloc(_slab_)                                                         \
do                                                                                      \
{                                                                                       \
    ds_TSlabSharedDealloc(&(_slab_).shared);                                            \
    (_slab_).buf = NULL;                                                                \
} while (0)

#define ds_TSlabFlush(_slab_)      ds_TSlabSharedFlush(&(_slab_).shared)

#define ds_TSlabAdd(_slab_, _count_)                                                    \
    ((union { struct ds_ArrayUntyped u; ds_TypeOf(*(_slab_).array_type) t; })           \
        { .u = ds_TSlabSharedAdd(&(_slab_).shared, sizeof((_slab_).buf[0]),             \
                                 (_count_)) }).t

#define ds_TSlabRemove(_slab_, _array_)                                                 \
    ds_TSlabSharedRemove(&(_slab_).shared, sizeof((_slab_).buf[0]), (_array_).base,     \
                         (_array_).length)

#define ds_TSlabRealloc(_slab_, _array_, _count_)                                       \
    ((union { struct ds_ArrayUntyped u; ds_TypeOf(*(_slab_).array_type) t; })           \
        { .u = ds_TSlabSharedRealloc(&(_slab_).shared, sizeof((_slab_).buf[0]),         \
                                     (struct ds_ArrayUntyped)                           \
                                     {                                                  \
                                         .buf = (_array_).buf,                          \
                                         .length = (_array_).length,                    \
                                         .base = (_array_).base,                        \
                                     },                                                 \
                                     (_count_)) }).t

#define ds_TSlabArrayTryDeserialize(_ss_, _slab_, _array_, _count_)                     \
    ds_TSlabSharedArrayTryDeserialize((_ss_), &(_slab_).shared,                         \
                                      (struct ds_ArrayUntyped *) &(_array_), (_count_), \
                                      sizeof((_slab_).buf[0]))

/****************************** TPool inline definitions ******************************/

static ds_ForceInline struct slot ds_TSlabSharedAddSlot(struct ds_TSlabShared *shared, const u64 slot_size)
{
    const struct ds_ArrayUntyped arr = ds_TSlabSharedAdd(shared, slot_size, 1);
    return (struct slot) { .address = arr.buf, .index = arr.base };
}

#define DEFINE_TPOOL(T)     DEFINE_TSLAB(T)

#define ds_TPoolAlloc(_mem_, _pool_, _length_, _thread_count_)                          \
    ds_TSlabAlloc((_mem_), (_pool_), (_length_), (_thread_count_))
#define ds_TPoolDealloc(_pool_)     ds_TSlabDealloc(_pool_)
#define ds_TPoolFlush(_pool_)       ds_TSlabFlush(_pool_)

#define ds_TPoolAdd(_pool_)         ds_TSlabSharedAddSlot(&(_pool_).shared, sizeof((_pool_).buf[0]))

#define ds_TPoolRemove(_pool_, _index_)                                                 \
    ds_TSlabSharedRemove(&(_pool_).shared, sizeof((_pool_).buf[0]), (_index_), 1)

#endif
